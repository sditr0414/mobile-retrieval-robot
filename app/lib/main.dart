import 'dart:async';
import 'dart:convert';
import 'dart:typed_data';
import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_joystick/flutter_joystick.dart';
import 'package:flutter_classic_bluetooth/flutter_classic_bluetooth.dart';
import 'package:provider/provider.dart';

import 'protocol/robot_packet.dart';

/// 차량 화면의 물체 잡기 동작에서 확인된 마지막 안전 상태다.
enum PickupWorkflowState { idle, pickupPoseReady, holding }

/// 이전 알림을 오래 쌓아두지 않고 현재 작업 결과를 짧고 일관되게 표시한다.
void showAppMessage(
  BuildContext context,
  String message, {
  Color? backgroundColor,
  Duration duration = const Duration(milliseconds: 1400),
}) {
  final ScaffoldMessengerState messenger = ScaffoldMessenger.of(context);
  messenger
    ..hideCurrentSnackBar()
    ..showSnackBar(
      SnackBar(
        content: Text(message),
        backgroundColor: backgroundColor,
        duration: duration,
      ),
    );
}

/*
 * 앱 구조
 *
 * BluetoothController : HC-05 연결, 16바이트 패킷 송수신, STM32 응답 관리
 * TeachingController  : 앱이 실행되는 동안 티칭 시퀀스와 웨이포인트 보관
 * MainScreen          : 공통 안전 버튼과 RC카/로봇팔/티칭 화면 전환
 * 각 화면 Widget       : 사용자 입력을 Controller의 명령으로 변환
 *
 * 화면은 Controller의 상태를 구독하지만, Bluetooth 패킷을 직접 만들지 않는다.
 * 이 구조를 지키면 통신 규격이나 안전 조건을 한곳에서 확인할 수 있다.
 */

/// 세로·가로 회전을 허용하고 앱 전체에서 사용할 상태 관리자를 생성한다.
Future<void> main() async {
  WidgetsFlutterBinding.ensureInitialized();
  await SystemChrome.setPreferredOrientations([
    DeviceOrientation.portraitUp,
    DeviceOrientation.landscapeLeft,
    DeviceOrientation.landscapeRight,
  ]);

  runApp(
    MultiProvider(
      providers: [
        ChangeNotifierProvider(create: (_) => BluetoothController()),
        ChangeNotifierProvider(create: (_) => TeachingController()),
      ],
      child: const RobotApp(),
    ),
  );
}

class RobotApp extends StatelessWidget {
  const RobotApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'RC & Robot Controller',
      debugShowCheckedModeBanner: false,
      theme: ThemeData(
        colorScheme: ColorScheme.fromSeed(seedColor: Colors.blueGrey),
        useMaterial3: true,
        scaffoldBackgroundColor: const Color(0xFFF4F6F8),
        appBarTheme: const AppBarTheme(
          backgroundColor: Color(0xFF263238),
          foregroundColor: Colors.white,
          centerTitle: false,
        ),
        cardTheme: const CardThemeData(
          margin: EdgeInsets.zero,
          elevation: 0,
          clipBehavior: Clip.antiAlias,
          shape: RoundedRectangleBorder(
            borderRadius: BorderRadius.all(Radius.circular(14)),
            side: BorderSide(color: Color(0xFFDDE4E8)),
          ),
        ),
        filledButtonTheme: FilledButtonThemeData(
          style: ButtonStyle(
            minimumSize: const WidgetStatePropertyAll(Size(0, 46)),
            shape: WidgetStatePropertyAll(
              RoundedRectangleBorder(borderRadius: BorderRadius.circular(10)),
            ),
          ),
        ),
        outlinedButtonTheme: OutlinedButtonThemeData(
          style: ButtonStyle(
            minimumSize: const WidgetStatePropertyAll(Size(0, 46)),
            shape: WidgetStatePropertyAll(
              RoundedRectangleBorder(borderRadius: BorderRadius.circular(10)),
            ),
          ),
        ),
        inputDecorationTheme: const InputDecorationTheme(
          filled: true,
          isDense: true,
          border: OutlineInputBorder(),
          contentPadding: EdgeInsets.symmetric(horizontal: 12, vertical: 11),
        ),
        snackBarTheme: const SnackBarThemeData(
          behavior: SnackBarBehavior.floating,
          showCloseIcon: true,
        ),
      ),
      home: const MainScreen(),
    );
  }
}

// ======================================================
// Bluetooth 통신과 STM32 상태 관리자
// ======================================================

/// 전송을 기다리는 패킷과 해당 전송의 완료 상태를 함께 보관한다.
class _PendingPacket {
  _PendingPacket(this.bytes);

  final List<int> bytes;
  final Completer<void> completer = Completer<void>();
}

/// HC-05 연결부터 패킷 송수신, ACK 대기까지 담당하는 앱의 통신 중심 클래스다.
///
/// 화면은 이 클래스의 상태만 확인하며 Bluetooth 스트림을 직접 다루지 않는다.
class BluetoothController with ChangeNotifier {
  // 앱의 Kp 입력은 참조 PID 앱과 같이 0.01 단위, 최대 2.55로 제한한다.
  static const int pidKpMaxMilli = 2550;
  static const int pidKpStepMilli = 10;
  static const int defaultPidKpMilli = 2000;
  static const int defaultPidKiMilli = 1400;
  static const int defaultPidKdMilli = 0;
  static const double defaultJoystickThreshold = 0.10;

  static const int packetSize = RobotPacketCodec.packetSize;
  static const int dataSize = RobotPacketCodec.dataSize;
  static const int settingsSize = 53;
  static const int settingsReadPartCount = 7;
  static const int settingsWritePartCount = 6;
  static const List<bool> servoReversed = [true, true, false, true, true, true];
  static const List<List<int>> defaultServoCalibrationUs = [
    [2300, 1500, 700],
    [2300, 1500, 700],
    [700, 1500, 2300],
    [2300, 1500, 700],
    [2300, 1500, 700],
    [1800, 1500, 1200],
  ];
  static const List<int> defaultTravelPoseAngles = [90, 40, 180, 90, 150];

  final FlutterClassicBluetooth _bluetooth = FlutterClassicBluetooth();

  BtcConnection? connection;
  List<BtcDevice> bondedDevices = [];
  bool isLoadingBondedDevices = false;
  bool isConnecting = false;
  String? connectionErrorMessage;
  bool isEstopLatched = true;
  bool isArmEnabled = false;
  bool armPoseKnown = false;
  bool isDriveReady = false;
  PickupWorkflowState pickupWorkflowState = PickupWorkflowState.idle;
  bool pickupActionBusy = false;
  bool armCommandBusy = false;
  bool settingsLoaded = false;
  bool settingsBusy = false;
  bool servoPreviewMoving = false;
  bool pidApplied = false;
  bool hasFreshPidStatus = false;
  bool deviceCommandActive = false;
  bool devicePidRunning = false;
  bool devicePidReverse = false;
  bool deviceTargetUpdated = false;
  bool deviceImuAvailable = false;
  double deviceTargetYaw = 0;
  double deviceCurrentYaw = 0;
  double devicePidError = 0;
  double devicePidOutput = 0;
  int deviceLeftPwm = 0;
  int deviceRightPwm = 0;
  bool hasFreshImuStatus = false;
  bool deviceImuInitialized = false;
  bool deviceImuCalibrated = false;
  bool deviceImuValid = false;
  bool _imuInitializationFailureAnnounced = false;
  bool _imuCalibrationStartedAnnounced = false;
  bool _imuCalibrationCompletedAnnounced = false;
  double deviceTemperatureC = 0;
  double deviceGyroZDps = 0;
  double deviceYaw = 0;
  int deviceImuErrorCount = 0;
  bool isFeaturePreviewMode = false;
  int servoSpeedPercent = 100;
  int pidKpMilli = defaultPidKpMilli;
  int pidKiMilli = defaultPidKiMilli;
  int pidKdMilli = defaultPidKdMilli;
  double joystickThreshold = defaultJoystickThreshold;
  List<List<int>> servoCalibrationUs = defaultServoCalibrationUs
      .map((values) => List<int>.from(values))
      .toList();
  List<int> travelPoseAngles = List<int>.from(defaultTravelPoseAngles);
  String? lastAckMessage;
  bool lastAckSucceeded = true;
  bool lastAckIsWarning = false;
  bool _speedWarningShownForCurrentMotion = false;
  final List<int> _receiveBuffer = [];
  final List<_PendingPacket> _writeQueue = [];
  bool _isWriting = false;
  Completer<bool>? _teachingAckCompleter;
  int? _teachingAckSequence;
  Completer<bool>? _armAckCompleter;
  Completer<bool>? _teachingResetCompleter;
  int? _teachingResetSequence;
  Completer<bool>? _settingsReadCompleter;
  Completer<bool>? _settingsSaveCompleter;
  Completer<bool>? _estopClearCompleter;
  Completer<bool>? _teachingPlayCompleter;
  Completer<bool>? _pidToggleCompleter;
  Completer<bool>? _imuCalibrationCompleter;
  Completer<String?>? _teachingNameCompleter;
  Completer<List<List<int>>?>? _teachingSequenceCompleter;
  bool _armActivationPending = false;
  bool _drivePosePending = false;
  bool _suppressArmCompletionMessage = false;
  int _nextArmTransactionId = 1;
  int _nextSettingsRequestId = 1;
  int _nextPreviewRequestId = 1;
  int _nextTeachingNameRequestId = 1;
  int _nextTeachingSequenceRequestId = 1;
  int? _pendingArmTransactionId;
  int? _lastArmTransactionId;
  int? _teachingPlaySequence;
  int? _pendingSettingsRequestId;
  int? _pendingPidToggleRequestId;
  bool? _pendingPidToggleValue;
  int? _pendingImuCalibrationRequestId;
  bool _imuCalibrationObservedRunning = false;
  int? _pendingPreviewRequestId;
  int? _pendingPreviewJoint;
  int? _pendingPreviewPulseUs;
  int? _pendingTeachingNameSequence;
  int? _pendingTeachingNameRequestId;
  int? _pendingTeachingNameLength;
  int? _pendingTeachingSequence;
  int? _pendingTeachingSequenceRequestId;
  int? _pendingTeachingWaypointCount;
  StreamSubscription<Uint8List>? _inputSubscription;
  StreamSubscription<BtcConnectionState>? _connectionStateSubscription;
  final Uint8List _settingsReceiveBuffer = Uint8List(settingsSize);
  final Uint8List _teachingNameBuffer = Uint8List(
    TeachingController.maxNameBytes,
  );
  int _settingsReceiveParts = 0;
  int _teachingNameReceiveParts = 0;
  final List<List<int>?> _teachingWaypointBuffer = List.filled(
    TeachingController.maxWaypoints,
    null,
  );
  int _teachingWaypointReceiveMask = 0;
  int? _previewSavedSpeedPercent;
  int? _previewSavedKpMilli;
  int? _previewSavedKiMilli;
  int? _previewSavedKdMilli;
  double? _previewSavedJoystickThreshold;
  bool? _previewSavedSettingsLoaded;
  bool? _previewSavedPidApplied;
  List<List<int>>? _previewSavedCalibrationUs;
  List<int>? _previewSavedTravelPoseAngles;
  int _previousPidDriveDirection = 0;
  DateTime? _lastPidStatusAt;
  DateTime? _lastImuStatusAt;
  Timer? _statusWatchdogTimer;

  BluetoothController() {
    _statusWatchdogTimer = Timer.periodic(
      const Duration(milliseconds: 500),
      (_) => _checkStatusFreshness(),
    );
  }

  /// 오래된 상태를 실제 적용 중인 값처럼 표시하지 않도록 만료시킨다.
  void _checkStatusFreshness() {
    final DateTime now = DateTime.now();
    final bool pidFresh =
        _lastPidStatusAt != null &&
        now.difference(_lastPidStatusAt!).inMilliseconds <= 1500;
    final bool imuFresh =
        _lastImuStatusAt != null &&
        now.difference(_lastImuStatusAt!).inMilliseconds <= 1500;
    if (pidFresh != hasFreshPidStatus || imuFresh != hasFreshImuStatus) {
      hasFreshPidStatus = pidFresh;
      hasFreshImuStatus = imuFresh;
      notifyListeners();
    }
  }

  /// Little-endian 상태 패킷의 signed 16비트 값을 읽는다.
  int _readInt16Le(List<int> packet, int offset) {
    final int bits = packet[offset] | (packet[offset + 1] << 8);
    return (bits & 0x8000) != 0 ? bits - 0x10000 : bits;
  }

  /// Little-endian 상태 패킷의 unsigned 32비트 값을 읽는다.
  int _readUint32Le(List<int> packet, int offset) {
    return packet[offset] |
        (packet[offset + 1] << 8) |
        (packet[offset + 2] << 16) |
        (packet[offset + 3] << 24);
  }

  /// 실제 HC-05 연결 또는 기능 확인 모드가 명령을 받을 수 있으면 true다.
  bool get isConnected =>
      isFeaturePreviewMode || (connection?.isConnected ?? false);

  /// 기능 확인 모드가 아닌 실제 HC-05 연결 상태만 반환한다.
  bool get isPhysicallyConnected => connection?.isConnected ?? false;

  bool get isAtPickupPose =>
      pickupWorkflowState == PickupWorkflowState.pickupPoseReady;
  bool get isHoldingPayload =>
      pickupWorkflowState == PickupWorkflowState.holding;

  /// 잡기 전용 시퀀스 동안 차량 주행을 잠그고 화면 상태를 갱신한다.
  void beginPickupAction() {
    pickupActionBusy = true;
    isDriveReady = false;
    notifyListeners();
  }

  /// 시퀀스 결과에 따른 다음 동작 조건과 주행 가능 상태를 저장한다.
  void finishPickupAction({
    required PickupWorkflowState state,
    required bool driveReady,
  }) {
    pickupWorkflowState = state;
    pickupActionBusy = false;
    isDriveReady = driveReady;
    notifyListeners();
  }

  /// 연결 해제나 E-STOP 뒤에는 실제 물체 상태를 신뢰하지 않는다.
  void resetPickupWorkflow() {
    pickupWorkflowState = PickupWorkflowState.idle;
    pickupActionBusy = false;
    notifyListeners();
  }

  /// 실제 장치 없이 UI와 ACK 흐름을 확인하는 메모리 전용 모드를 시작한다.
  bool enableFeaturePreviewMode() {
    if (isPhysicallyConnected || isFeaturePreviewMode) {
      return false;
    }

    _previewSavedSpeedPercent = servoSpeedPercent;
    _previewSavedKpMilli = pidKpMilli;
    _previewSavedKiMilli = pidKiMilli;
    _previewSavedKdMilli = pidKdMilli;
    _previewSavedJoystickThreshold = joystickThreshold;
    _previewSavedSettingsLoaded = settingsLoaded;
    _previewSavedPidApplied = pidApplied;
    _previewSavedCalibrationUs = servoCalibrationUs
        .map((values) => List<int>.from(values))
        .toList();
    _previewSavedTravelPoseAngles = List<int>.from(travelPoseAngles);

    isFeaturePreviewMode = true;
    isEstopLatched = true;
    isArmEnabled = false;
    armPoseKnown = false;
    isDriveReady = false;
    pickupWorkflowState = PickupWorkflowState.idle;
    pickupActionBusy = false;
    settingsLoaded = true;
    settingsBusy = false;
    servoPreviewMoving = false;
    pidApplied = false;
    hasFreshPidStatus = false;
    deviceCommandActive = false;
    devicePidRunning = false;
    devicePidReverse = false;
    deviceTargetUpdated = false;
    deviceImuAvailable = false;
    deviceTargetYaw = 0;
    deviceCurrentYaw = 0;
    devicePidError = 0;
    devicePidOutput = 0;
    deviceLeftPwm = 0;
    deviceRightPwm = 0;
    hasFreshImuStatus = false;
    deviceImuInitialized = false;
    deviceImuCalibrated = false;
    deviceImuValid = false;
    deviceTemperatureC = 0;
    deviceGyroZDps = 0;
    deviceYaw = 0;
    deviceImuErrorCount = 0;
    _lastPidStatusAt = null;
    _lastImuStatusAt = null;
    _previousPidDriveDirection = 0;
    connectionErrorMessage = null;
    lastAckSucceeded = true;
    lastAckIsWarning = false;
    lastAckMessage = 'TEST 모드 · 장치 전송 없음';
    notifyListeners();
    return true;
  }

  /// 기능 확인 중 바꾼 설정을 버리고 실제 운용 전의 앱 상태로 복원한다.
  void disableFeaturePreviewMode() {
    if (!isFeaturePreviewMode) {
      return;
    }

    isFeaturePreviewMode = false;
    _clearConnectionState();
    servoSpeedPercent = _previewSavedSpeedPercent ?? servoSpeedPercent;
    pidKpMilli = _previewSavedKpMilli ?? pidKpMilli;
    pidKiMilli = _previewSavedKiMilli ?? pidKiMilli;
    pidKdMilli = _previewSavedKdMilli ?? pidKdMilli;
    joystickThreshold = _previewSavedJoystickThreshold ?? joystickThreshold;
    settingsLoaded = _previewSavedSettingsLoaded ?? false;
    pidApplied = _previewSavedPidApplied ?? false;
    final savedCalibration = _previewSavedCalibrationUs;
    if (savedCalibration != null) {
      servoCalibrationUs = savedCalibration
          .map((values) => List<int>.from(values))
          .toList();
    }
    final savedTravelPose = _previewSavedTravelPoseAngles;
    if (savedTravelPose != null) {
      travelPoseAngles = List<int>.from(savedTravelPose);
    }
    _previewSavedSpeedPercent = null;
    _previewSavedKpMilli = null;
    _previewSavedKiMilli = null;
    _previewSavedKdMilli = null;
    _previewSavedJoystickThreshold = null;
    _previewSavedSettingsLoaded = null;
    _previewSavedPidApplied = null;
    _previewSavedCalibrationUs = null;
    _previewSavedTravelPoseAngles = null;
    lastAckSucceeded = true;
    lastAckIsWarning = false;
    lastAckMessage = 'TEST 종료 · 기존 설정 복원';
    notifyListeners();
  }

  /// 0은 예약값으로 두고 로봇팔 명령을 구분할 1~255 번호를 순환 발급한다.
  int _takeArmTransactionId() {
    final int transactionId = _nextArmTransactionId;
    _nextArmTransactionId = (_nextArmTransactionId % 0xFF) + 1;
    return transactionId;
  }

  /// 다음 로봇팔 이동에 적용할 속도를 안전 범위인 50~100%로 제한한다.
  void setServoSpeedPercent(int percent) {
    final int normalized = percent.clamp(50, 100);
    if (servoSpeedPercent == normalized) {
      return;
    }
    if (normalized < servoSpeedPercent &&
        isArmEnabled &&
        !armPoseKnown &&
        !_speedWarningShownForCurrentMotion) {
      lastAckSucceeded = true;
      lastAckIsWarning = true;
      lastAckMessage = '이동 중 속도 변경 · 부드럽게 감속';
      _speedWarningShownForCurrentMotion = true;
    }
    servoSpeedPercent = normalized;
    notifyListeners();
  }

  /// PID 직진으로 판단할 조이스틱 X축 중앙 범위를 앱 실행 중에 저장한다.
  void setJoystickThreshold(double threshold) {
    if (!threshold.isFinite) {
      return;
    }
    final double normalized = threshold.clamp(0.0, 1.0).toDouble();
    if (joystickThreshold == normalized) {
      return;
    }
    joystickThreshold = normalized;
    notifyListeners();
  }

  /// 로봇팔 패킷이 관절 6개와 펌웨어 허용 범위로 구성됐는지 확인한다.
  bool _isArmPoseValid(List<int> angles) {
    return angles.length == 6 &&
        angles.every((angle) => angle >= 0 && angle <= 180);
  }

  /// Android에 미리 페어링된 Classic Bluetooth 장치 목록을 불러온다.
  Future<void> loadBondedDevices() async {
    isLoadingBondedDevices = true;
    connectionErrorMessage = null;
    notifyListeners();
    try {
      bondedDevices = await _bluetooth.getPairedDevices();
    } catch (e) {
      debugPrint("페어링 기기 로드 실패: $e");
      connectionErrorMessage = '페어링 장치 조회 실패: $e';
    } finally {
      isLoadingBondedDevices = false;
      notifyListeners();
    }
  }

  /// 선택한 장치에 연결하고 수신 스트림과 초기 설정 조회를 시작한다.
  Future<void> connectToDevice(BtcDevice device) async {
    isConnecting = true;
    connectionErrorMessage = null;
    notifyListeners();

    try {
      final BtcConnection establishedConnection = await _bluetooth.connect(
        address: device.address,
      );
      connection = establishedConnection;
      debugPrint(
        "✅ HC-05 연결 성공: ${device.name ?? 'Unknown'} (${device.address})",
      );
      isConnecting = false;
      isEstopLatched = true;
      isArmEnabled = false;
      armPoseKnown = false;
      isDriveReady = false;
      pickupWorkflowState = PickupWorkflowState.idle;
      pickupActionBusy = false;
      armCommandBusy = false;
      settingsLoaded = false;
      pidApplied = false;
      _imuInitializationFailureAnnounced = false;
      _imuCalibrationStartedAnnounced = false;
      _imuCalibrationCompletedAnnounced = false;
      notifyListeners();

      await _inputSubscription?.cancel();
      await _connectionStateSubscription?.cancel();
      _inputSubscription = establishedConnection.input.listen(
        (Uint8List data) {
          if (identical(connection, establishedConnection)) {
            _handleIncomingData(data);
          }
        },
        onDone: () {
          if (!identical(connection, establishedConnection)) {
            return;
          }
          debugPrint("원격 기기와의 연결이 종료되었습니다.");
          _clearConnectionState();
        },
        onError: (Object error) {
          if (!identical(connection, establishedConnection)) {
            return;
          }
          connectionErrorMessage = 'Bluetooth 수신 오류: $error';
          _clearConnectionState();
        },
      );
      _connectionStateSubscription = establishedConnection.stateStream.listen(
        (BtcConnectionState state) {
          if (identical(connection, establishedConnection) &&
              state == BtcConnectionState.disconnected) {
            connectionErrorMessage = 'HC-05 연결이 끊어졌습니다.';
            _clearConnectionState();
          }
        },
        onError: (Object error) {
          if (identical(connection, establishedConnection)) {
            connectionErrorMessage = 'Bluetooth 상태 확인 오류: $error';
            _clearConnectionState();
          }
        },
      );
      unawaited(
        Future<void>.delayed(const Duration(milliseconds: 50), () async {
          if (identical(connection, establishedConnection) &&
              establishedConnection.isConnected) {
            await requestSettings();
          }
        }),
      );
    } catch (e) {
      debugPrint("연결 에러: $e");
      connection = null;
      isConnecting = false;
      connectionErrorMessage = 'HC-05 연결 실패: $e';
      notifyListeners();
    }
  }

  /// 잘려서 들어오는 수신 바이트를 모아 유효한 16바이트 ACK만 처리한다.
  void _handleIncomingData(Uint8List data) {
    _receiveBuffer.addAll(data);

    // Bluetooth 수신 조각을 모아 16바이트 ACK 한 개씩 꺼낸다.
    while (true) {
      final List<int>? packet = RobotPacketCodec.takeNext(_receiveBuffer);
      if (packet == null) {
        return;
      }
      if (packet[1] == 0 && packet[2] == 3) {
        final int flags = packet[3];
        deviceCommandActive = (flags & 0x01) != 0;
        pidApplied = (flags & 0x02) != 0;
        devicePidRunning = (flags & 0x04) != 0;
        devicePidReverse = (flags & 0x08) != 0;
        deviceTargetUpdated = (flags & 0x10) != 0;
        deviceImuAvailable = (flags & 0x20) != 0;
        deviceTargetYaw = _readInt16Le(packet, 4) / 100;
        deviceCurrentYaw = _readInt16Le(packet, 6) / 100;
        devicePidError = _readInt16Le(packet, 8) / 100;
        devicePidOutput = _readInt16Le(packet, 10) / 100;
        deviceLeftPwm = packet[12];
        deviceRightPwm = packet[13];
        _lastPidStatusAt = DateTime.now();
        hasFreshPidStatus = true;
        if (_pidToggleCompleter?.isCompleted == false &&
            pidApplied == _pendingPidToggleValue) {
          lastAckSucceeded = true;
          lastAckIsWarning = false;
          lastAckMessage = pidApplied
              ? '방향 안정화 PID가 활성화되었습니다.'
              : '방향 안정화 PID가 비활성화되었습니다.';
          _pidToggleCompleter!.complete(true);
        }
        notifyListeners();
      } else if (packet[1] == 0 && packet[2] == 4) {
        final int flags = packet[3];
        deviceImuInitialized = (flags & 0x01) != 0;
        deviceImuCalibrated = (flags & 0x02) != 0;
        deviceImuValid = (flags & 0x04) != 0;
        deviceTemperatureC = _readInt16Le(packet, 4) / 100;
        deviceGyroZDps = _readInt16Le(packet, 6) / 100;
        deviceYaw = _readInt16Le(packet, 8) / 100;
        deviceImuErrorCount = _readUint32Le(packet, 10);
        if (_imuCalibrationCompleter?.isCompleted == false) {
          if (deviceImuInitialized && !deviceImuCalibrated) {
            _imuCalibrationObservedRunning = true;
          } else if (_imuCalibrationObservedRunning &&
              deviceImuCalibrated &&
              deviceImuValid) {
            _imuCalibrationCompleter!.complete(true);
          }
        }
        if (deviceImuCalibrated && !_imuCalibrationCompletedAnnounced) {
          _imuCalibrationCompletedAnnounced = true;
          lastAckSucceeded = true;
          lastAckIsWarning = false;
          lastAckMessage = 'IMU 보정 완료 · PID를 사용할 수 있습니다.';
        } else if (deviceImuInitialized &&
            !deviceImuCalibrated &&
            !_imuCalibrationStartedAnnounced) {
          _imuCalibrationStartedAnnounced = true;
          lastAckSucceeded = true;
          lastAckIsWarning = true;
          lastAckMessage = 'IMU 초기화 완료 · 3초간 차량을 움직이지 마세요.';
        } else if (!deviceImuInitialized &&
            deviceImuErrorCount > 0 &&
            !_imuInitializationFailureAnnounced) {
          _imuInitializationFailureAnnounced = true;
          lastAckSucceeded = false;
          lastAckIsWarning = false;
          lastAckMessage = 'IMU 초기화 실패 · 전원과 I2C 연결을 확인하세요.';
        }
        _lastImuStatusAt = DateTime.now();
        hasFreshImuStatus = true;
        notifyListeners();
      } else if (packet[1] == 0 && packet[2] == 2) {
        final bool succeeded = packet[3] == 1;
        lastAckSucceeded = succeeded;
        lastAckIsWarning = false;
        lastAckMessage = succeeded
            ? isFeaturePreviewMode
                  ? 'TEST STOP 해제'
                  : 'STOP 해제'
            : 'STOP 해제 거부';
        if (_estopClearCompleter?.isCompleted == false) {
          _estopClearCompleter!.complete(succeeded);
        }
        notifyListeners();
      } else if (packet[1] == 2 &&
          packet[2] == 5 &&
          packet[3] == _pendingTeachingNameSequence &&
          packet[6] == _pendingTeachingNameRequestId) {
        final int part = packet[4];
        final int length = packet[5];
        final int offset = part * 7;
        if (part < 3 &&
            length >= 1 &&
            length <= TeachingController.maxNameBytes &&
            offset < length &&
            (_pendingTeachingNameLength == null ||
                _pendingTeachingNameLength == length)) {
          _pendingTeachingNameLength = length;
          final int partLength = (length - offset).clamp(0, 7).toInt();
          _teachingNameBuffer.setRange(
            offset,
            offset + partLength,
            packet.sublist(7, 7 + partLength),
          );
          _teachingNameReceiveParts |= 1 << part;
          final int requiredParts = (length + 6) ~/ 7;
          final int requiredMask = (1 << requiredParts) - 1;
          if ((_teachingNameReceiveParts & requiredMask) == requiredMask &&
              _teachingNameCompleter?.isCompleted == false) {
            try {
              _teachingNameCompleter!.complete(
                utf8.decode(_teachingNameBuffer.sublist(0, length)),
              );
            } on FormatException {
              _teachingNameCompleter!.complete(null);
            }
          }
        }
      } else if (packet[1] == 2 &&
          packet[2] == 6 &&
          packet[3] == _pendingTeachingSequence) {
        final int kind = packet[4];
        if (kind == 0 && packet[6] == _pendingTeachingSequenceRequestId) {
          final int count = packet[5];
          if (count <= TeachingController.maxWaypoints) {
            _pendingTeachingWaypointCount = count;
            if (count == 0 &&
                _teachingSequenceCompleter?.isCompleted == false) {
              _teachingSequenceCompleter!.complete(<List<int>>[]);
            }
          }
        } else if (kind == 1 &&
            packet[12] == _pendingTeachingSequenceRequestId &&
            _pendingTeachingWaypointCount != null) {
          final int index = packet[5];
          final int count = _pendingTeachingWaypointCount!;
          final List<int> pose = packet.sublist(6, 12);
          if (index < count && pose.every((angle) => angle <= 180)) {
            _teachingWaypointBuffer[index] = pose;
            _teachingWaypointReceiveMask |= 1 << index;
            final int requiredMask = (1 << count) - 1;
            if ((_teachingWaypointReceiveMask & requiredMask) == requiredMask &&
                _teachingSequenceCompleter?.isCompleted == false) {
              _teachingSequenceCompleter!.complete(
                List.generate(
                  count,
                  (waypoint) =>
                      List<int>.from(_teachingWaypointBuffer[waypoint]!),
                ),
              );
            }
          }
        }
      } else if (packet[1] == 2 &&
          (packet[2] == 2 || packet[2] == 3 || packet[2] == 4)) {
        final int command = packet[2];
        final bool succeeded = packet[4] == 1;
        final bool speedWarning = command == 2 && succeeded && packet[5] == 1;
        lastAckSucceeded = succeeded;
        lastAckIsWarning = speedWarning;
        if (command == 2) {
          lastAckMessage = isFeaturePreviewMode
              ? "기능 확인: 시퀀스 ${packet[3]}번 재생 흐름을 완료했습니다."
              : speedWarning
              ? "주의: 시퀀스 ${packet[3]}번은 일부 구간에서 설정 속도를 보장할 수 없었습니다."
              : succeeded
              ? "시퀀스 ${packet[3]}번 재생 완료"
              : "시퀀스 ${packet[3]}번 재생 실패";
          if ((_teachingPlaySequence == packet[3]) &&
              (_teachingPlayCompleter?.isCompleted == false)) {
            _teachingPlayCompleter!.complete(succeeded);
          }
        } else if (command == 4) {
          lastAckMessage = isFeaturePreviewMode
              ? "기능 확인: 시퀀스 ${packet[3]}번 업로드 흐름을 완료했습니다."
              : succeeded
              ? "시퀀스 ${packet[3]}번 Flash 저장 성공!"
              : "시퀀스 ${packet[3]}번 Flash 저장 실패";
          if ((_teachingAckSequence == packet[3]) &&
              (_teachingAckCompleter?.isCompleted == false)) {
            _teachingAckCompleter!.complete(succeeded);
          }
        } else {
          lastAckMessage = isFeaturePreviewMode
              ? "기능 확인: 시퀀스 ${packet[3]}번 초기화 흐름을 완료했습니다."
              : succeeded
              ? "시퀀스 ${packet[3]}번 초기화 성공!"
              : "시퀀스 ${packet[3]}번 초기화 실패";
          if ((_teachingResetSequence == packet[3]) &&
              (_teachingResetCompleter?.isCompleted == false)) {
            _teachingResetCompleter!.complete(succeeded);
          }
        }
        notifyListeners();
      } else if (packet[1] == 3 &&
          packet[2] == 8 &&
          packet[5] == _pendingImuCalibrationRequestId) {
        final bool succeeded = packet[3] == 1;
        final int reason = packet[4];
        lastAckSucceeded = succeeded;
        lastAckIsWarning = false;
        lastAckMessage = succeeded
            ? 'IMU 영점 보정 완료 · PID를 적용합니다.'
            : reason == 1
            ? 'IMU가 연결되지 않아 영점 보정을 시작하지 못했습니다.'
            : reason == 2
            ? 'IMU 영점 보정이 이미 진행 중입니다.'
            : 'IMU 측정 오류로 영점 보정에 실패했습니다.';
        if (_imuCalibrationCompleter?.isCompleted == false) {
          _imuCalibrationCompleter!.complete(succeeded);
        }
        notifyListeners();
      } else if (packet[1] == 3 &&
          packet[2] == 7 &&
          packet[6] == _pendingPidToggleRequestId) {
        final bool succeeded = packet[3] == 1;
        final int reason = packet[5];
        pidApplied = packet[4] == 1;
        settingsBusy = false;
        lastAckSucceeded = succeeded;
        lastAckIsWarning = false;
        lastAckMessage = succeeded
            ? pidApplied
                  ? '방향 안정화 PID가 활성화되었습니다.'
                  : '방향 안정화 PID가 비활성화되었습니다.'
            : reason == 1
            ? 'IMU 보정이 완료되지 않았거나 현재 안전 잠금 상태입니다.'
            : reason == 2
            ? 'PID 계수가 모두 0이거나 허용 범위를 벗어났습니다.'
            : 'PID 적용 상태를 변경하지 못했습니다. 오류 코드: $reason';
        if (_pidToggleCompleter?.isCompleted == false) {
          _pidToggleCompleter!.complete(succeeded);
        }
        notifyListeners();
      } else if (packet[1] == 3 &&
          packet[2] == 5 &&
          packet[8] == _pendingPreviewRequestId) {
        final int joint = packet[3];
        final bool succeeded = packet[4] == 1;
        final int reason = packet[5];
        final int pulseUs = packet[6] | (packet[7] << 8);
        final bool expectedTarget =
            joint == _pendingPreviewJoint && pulseUs == _pendingPreviewPulseUs;

        if (expectedTarget) {
          servoPreviewMoving = false;
          _pendingPreviewRequestId = null;
          _pendingPreviewJoint = null;
          _pendingPreviewPulseUs = null;
          if (!succeeded) {
            if (reason == 5) {
              isArmEnabled = false;
              armPoseKnown = false;
            }
            lastAckSucceeded = false;
            lastAckIsWarning = false;
            lastAckMessage = '원점 보정 이동 실패: ${_armFailureMessage(reason)}';
          }
          notifyListeners();
        }
      } else if (packet[1] == 3 && packet[2] == 4) {
        final int part = packet[3];
        if (part >= 0 &&
            part < settingsReadPartCount &&
            packet[13] == _pendingSettingsRequestId) {
          pidApplied = packet[4] == 1;
          final int start = part * 8;
          final int length = (settingsSize - start).clamp(0, 8).toInt();
          _settingsReceiveBuffer.setRange(
            start,
            start + length,
            packet.sublist(5, 5 + length),
          );
          _settingsReceiveParts |= 1 << part;

          if (_settingsReceiveParts == (1 << settingsReadPartCount) - 1) {
            _decodeSettings(_settingsReceiveBuffer);
            settingsLoaded = true;
            settingsBusy = false;
            if (_settingsReadCompleter?.isCompleted == false) {
              _settingsReadCompleter!.complete(true);
            }
          }
          notifyListeners();
        }
      } else if (packet[1] == 3 && packet[2] == 3) {
        final bool succeeded = packet[3] == 1;
        final int reason = packet[4];
        pidApplied = packet[5] == 1;
        settingsBusy = false;
        lastAckSucceeded = succeeded;
        lastAckIsWarning = false;
        lastAckMessage = isFeaturePreviewMode && succeeded
            ? '기능 확인 모드의 메모리에 설정을 임시 저장했습니다.'
            : succeeded
            ? '중요 설정을 STM32 Flash에 저장했습니다.'
            : '설정 저장 실패: ${_settingsFailureMessage(reason)}';
        if (_settingsSaveCompleter?.isCompleted == false) {
          _settingsSaveCompleter!.complete(succeeded);
        }
        notifyListeners();
      } else if (packet[1] == 1) {
        final int command = packet[2];
        final bool succeeded = packet[3] == 1;
        final int reason = packet[4];
        final bool speedWarning = succeeded && reason == 6;
        final int transactionId = packet[5];
        final bool expectedTransaction =
            transactionId != 0 && transactionId == _pendingArmTransactionId;
        final bool latestTransaction =
            transactionId != 0 && transactionId == _lastArmTransactionId;
        lastAckSucceeded = succeeded;
        lastAckIsWarning = speedWarning;
        if ((command == 0 || command == 3) && latestTransaction) {
          armPoseKnown = succeeded;
          if (succeeded) {
            _speedWarningShownForCurrentMotion = false;
          }
        }

        if (command == 1) {
          isArmEnabled = succeeded;
          lastAckMessage = succeeded
              ? "로봇팔 출력이 활성화되어 원점으로 이동 중입니다."
              : "로봇팔 활성화 실패: ${_armFailureMessage(reason)}";
          if (!succeeded) {
            _armActivationPending = false;
            armCommandBusy = false;
          }
          if (!succeeded &&
              expectedTransaction &&
              _armAckCompleter?.isCompleted == false) {
            _armAckCompleter!.complete(succeeded);
          }
        } else if (command == 2) {
          isArmEnabled = false;
          isDriveReady = false;
          _armActivationPending = false;
          _drivePosePending = false;
          armCommandBusy = false;
          if (_armAckCompleter?.isCompleted == false) {
            _armAckCompleter!.complete(false);
          }
          lastAckMessage = succeeded ? "로봇팔 출력을 차단했습니다." : "로봇팔 출력 차단에 실패했습니다.";
        } else if (command == 0 &&
            succeeded &&
            expectedTransaction &&
            _armActivationPending) {
          _armActivationPending = false;
          isArmEnabled = true;
          lastAckMessage = speedWarning
              ? "주의: 현재 위치를 읽을 수 없어 첫 원점 이동 속도를 보장할 수 없습니다."
              : _suppressArmCompletionMessage
              ? null
              : "로봇팔 원점 이동이 완료되었습니다.";
          if (_armAckCompleter?.isCompleted == false) {
            _armAckCompleter!.complete(true);
          }
        } else if ((command == 0 || command == 3) &&
            succeeded &&
            expectedTransaction &&
            _drivePosePending) {
          _drivePosePending = false;
          lastAckMessage = speedWarning
              ? "주의: 이동 자세 실행 중 일부 구간에서 설정 속도를 초과했습니다."
              : "로봇팔이 이동 자세에 도착했습니다.";
          if (_armAckCompleter?.isCompleted == false) {
            _armAckCompleter!.complete(true);
          }
        } else if (!succeeded && expectedTransaction) {
          _armActivationPending = false;
          _drivePosePending = false;
          armCommandBusy = false;
          if (_armAckCompleter?.isCompleted == false) {
            _armAckCompleter!.complete(false);
          }
          lastAckMessage = "로봇팔 이동 실패: ${_armFailureMessage(reason)}";
        } else if (command == 0 &&
            succeeded &&
            latestTransaction &&
            speedWarning) {
          lastAckMessage = "주의: 현재 운동 상태 때문에 일부 구간에서 설정 속도를 초과했습니다.";
        }
        notifyListeners();
      }
    }
  }

  /// 사용자에게 표시할 로봇팔 실패 코드를 이해하기 쉬운 문장으로 바꾼다.
  String _armFailureMessage(int reason) {
    switch (reason) {
      case 1:
        return "각도 값이 올바르지 않습니다.";
      case 2:
        return "아직 보정되지 않은 관절이 있습니다.";
      case 3:
        return "먼저 로봇팔 출력을 활성화하세요.";
      case 4:
        return "다른 동작을 실행 중입니다.";
      case 5:
        return "PCA9685 통신 오류입니다.";
      default:
        return "알 수 없는 오류($reason)";
    }
  }

  /// 사용자에게 표시할 설정 저장 실패 코드를 문장으로 바꾼다.
  String _settingsFailureMessage(int reason) {
    switch (reason) {
      case 1:
        return '설정 조각이 모두 도착하지 않았습니다.';
      case 2:
        return '설정 CRC가 일치하지 않습니다.';
      case 3:
        return '원점 보정값 또는 이동 자세 각도가 올바르지 않습니다.';
      case 4:
        return '원점을 바꾸려면 먼저 로봇팔 출력을 끄세요.';
      case 5:
        return 'Flash 기록 또는 검증에 실패했습니다.';
      case 6:
        return 'PID가 동작 중입니다. 먼저 PID를 끄세요.';
      case 7:
        return 'PID 계수는 각각 0~100 범위여야 합니다.';
      default:
        return '알 수 없는 오류($reason)';
    }
  }

  /// 화면에 한 번 표시한 ACK 메시지를 비운다.
  void clearAckMessage() {
    lastAckMessage = null;
    lastAckSucceeded = true;
    lastAckIsWarning = false;
  }

  /// 안전 정지 패킷을 먼저 보낸 뒤 현재 Bluetooth 연결을 종료한다.
  Future<void> disconnect() async {
    final BtcConnection? activeConnection = connection;
    if (activeConnection == null) {
      return;
    }

    // 정상 연결 해제 전에는 차량과 로봇팔을 먼저 안전 상태로 만든다.
    await emergencyStop();
    if (identical(connection, activeConnection)) {
      _clearConnectionState();
    }
    try {
      await activeConnection.finish();
    } catch (error) {
      debugPrint("Bluetooth 연결 종료 오류: $error");
    } finally {
      activeConnection.dispose();
    }
  }

  /// 연결 종료 시 화면 상태, 대기 패킷과 ACK 대기를 모두 안전하게 초기화한다.
  void _clearConnectionState() {
    connection = null;
    unawaited(_inputSubscription?.cancel());
    unawaited(_connectionStateSubscription?.cancel());
    _inputSubscription = null;
    _connectionStateSubscription = null;
    isConnecting = false;
    isEstopLatched = true;
    isArmEnabled = false;
    armPoseKnown = false;
    isDriveReady = false;
    pickupWorkflowState = PickupWorkflowState.idle;
    pickupActionBusy = false;
    armCommandBusy = false;
    _speedWarningShownForCurrentMotion = false;
    _armActivationPending = false;
    _drivePosePending = false;
    _suppressArmCompletionMessage = false;
    _pendingArmTransactionId = null;
    _lastArmTransactionId = null;
    _pendingSettingsRequestId = null;
    _pendingPidToggleRequestId = null;
    _pendingPidToggleValue = null;
    _pendingImuCalibrationRequestId = null;
    _imuCalibrationObservedRunning = false;
    _pendingPreviewRequestId = null;
    _pendingPreviewJoint = null;
    _pendingPreviewPulseUs = null;
    _pendingTeachingNameSequence = null;
    _pendingTeachingNameRequestId = null;
    _pendingTeachingNameLength = null;
    _teachingNameReceiveParts = 0;
    _pendingTeachingSequence = null;
    _pendingTeachingSequenceRequestId = null;
    _pendingTeachingWaypointCount = null;
    _teachingWaypointReceiveMask = 0;
    settingsLoaded = false;
    settingsBusy = false;
    servoPreviewMoving = false;
    pidApplied = false;
    hasFreshPidStatus = false;
    deviceCommandActive = false;
    devicePidRunning = false;
    devicePidReverse = false;
    deviceTargetUpdated = false;
    deviceImuAvailable = false;
    deviceTargetYaw = 0;
    deviceCurrentYaw = 0;
    devicePidError = 0;
    devicePidOutput = 0;
    deviceLeftPwm = 0;
    deviceRightPwm = 0;
    hasFreshImuStatus = false;
    deviceImuInitialized = false;
    deviceImuCalibrated = false;
    deviceImuValid = false;
    deviceTemperatureC = 0;
    deviceGyroZDps = 0;
    deviceYaw = 0;
    deviceImuErrorCount = 0;
    _lastPidStatusAt = null;
    _lastImuStatusAt = null;
    _previousPidDriveDirection = 0;
    _imuInitializationFailureAnnounced = false;
    _imuCalibrationStartedAnnounced = false;
    _imuCalibrationCompletedAnnounced = false;
    _receiveBuffer.clear();

    for (final pending in _writeQueue) {
      if (!pending.completer.isCompleted) {
        pending.completer.completeError(StateError("Bluetooth 연결이 종료되었습니다."));
      }
    }
    _writeQueue.clear();

    if (_teachingAckCompleter?.isCompleted == false) {
      _teachingAckCompleter!.complete(false);
    }
    if (_armAckCompleter?.isCompleted == false) {
      _armAckCompleter!.complete(false);
    }
    if (_teachingPlayCompleter?.isCompleted == false) {
      _teachingPlayCompleter!.complete(false);
    }
    if (_teachingResetCompleter?.isCompleted == false) {
      _teachingResetCompleter!.complete(false);
    }
    if (_settingsReadCompleter?.isCompleted == false) {
      _settingsReadCompleter!.complete(false);
    }
    if (_settingsSaveCompleter?.isCompleted == false) {
      _settingsSaveCompleter!.complete(false);
    }
    if (_estopClearCompleter?.isCompleted == false) {
      _estopClearCompleter!.complete(false);
    }
    if (_pidToggleCompleter?.isCompleted == false) {
      _pidToggleCompleter!.complete(false);
    }
    if (_imuCalibrationCompleter?.isCompleted == false) {
      _imuCalibrationCompleter!.complete(false);
    }
    if (_teachingNameCompleter?.isCompleted == false) {
      _teachingNameCompleter!.complete(null);
    }
    if (_teachingSequenceCompleter?.isCompleted == false) {
      _teachingSequenceCompleter!.complete(null);
    }
    notifyListeners();
  }

  /// 패킷을 전송 큐에 넣고 실제 Bluetooth 쓰기가 끝날 때까지 기다린다.
  ///
  /// urgent는 대기 명령을 취소하고 앞에 배치하며, replacePending은 오래된
  /// 주행·로봇팔 명령을 최신 명령으로 교체한다.
  Future<void> sendPacket(
    List<int> packet, {
    bool urgent = false,
    bool replacePending = false,
  }) {
    if (packet.length != packetSize) {
      debugPrint("잘못된 패킷 길이: ${packet.length}");
      return Future<void>.value();
    }

    final pending = _PendingPacket(List<int>.from(packet));
    pending.completer.future.catchError((Object error) {
      debugPrint("패킷 전송 에러: $error");
    });

    if (urgent) {
      // 아직 전송하지 않은 일반 명령은 취소하고 E-STOP을 맨 앞에 둔다.
      for (final queued in _writeQueue) {
        if (!queued.completer.isCompleted) {
          queued.completer.completeError(StateError("긴급 정지로 대기 패킷이 취소되었습니다."));
        }
      }
      _writeQueue
        ..clear()
        ..insert(0, pending);
    } else {
      if (replacePending) {
        for (int index = _writeQueue.length - 1; index >= 0; index--) {
          final List<int> queued = _writeQueue[index].bytes;
          final bool normalDrive = queued[1] == 0 && queued[6] == 0;
          final bool normalArm = queued[1] == 1 && queued[8] == 0;
          if ((queued[1] == packet[1]) && (normalDrive || normalArm)) {
            final replaced = _writeQueue.removeAt(index);
            replaced.completer.complete();
          }
        }
      }
      _writeQueue.add(pending);
    }

    unawaited(_drainWriteQueue());
    return pending.completer.future;
  }

  /// 큐에 쌓인 패킷을 한 번에 하나씩 순서대로 Bluetooth에 기록한다.
  Future<void> _drainWriteQueue() async {
    if (_isWriting) {
      return;
    }

    _isWriting = true;
    try {
      while (_writeQueue.isNotEmpty) {
        final _PendingPacket pending = _writeQueue.removeAt(0);

        try {
          final BtcConnection? activeConnection = connection;
          if (activeConnection == null) {
            throw StateError("Bluetooth가 연결되어 있지 않습니다.");
          }
          await activeConnection.output.writeBytes(pending.bytes);
          if (!pending.completer.isCompleted) {
            pending.completer.complete();
          }
        } catch (error, stackTrace) {
          if (!pending.completer.isCompleted) {
            pending.completer.completeError(error, stackTrace);
          }
          connectionErrorMessage = 'Bluetooth 전송 오류: $error';
          _clearConnectionState();
          break;
        }
      }
    } finally {
      _isWriting = false;
    }
  }

  /// 기능 확인 모드에서 STM32가 보낼 ACK를 짧은 지연 뒤 앱 내부에서 만든다.
  Future<void> _simulateCommand(int mode, List<int> data) async {
    if (!isFeaturePreviewMode) {
      return;
    }

    if (mode == 0 && data.length >= 5 && data[4] == 2) {
      await Future<void>.delayed(const Duration(milliseconds: 30));
      if (isFeaturePreviewMode) {
        _handleIncomingData(
          Uint8List.fromList(RobotPacketCodec.build(0, [2, 1])),
        );
      }
      return;
    }

    if (mode == 1 && data.length >= 8) {
      final int control = data[6];
      final int transactionId = data[7];
      await Future<void>.delayed(const Duration(milliseconds: 30));
      if (!isFeaturePreviewMode) {
        return;
      }

      if (control == 1) {
        _handleIncomingData(
          Uint8List.fromList(
            RobotPacketCodec.build(1, [1, 1, 0, transactionId]),
          ),
        );
        await Future<void>.delayed(const Duration(milliseconds: 100));
        if (isFeaturePreviewMode) {
          _handleIncomingData(
            Uint8List.fromList(
              RobotPacketCodec.build(1, [0, 1, 0, transactionId]),
            ),
          );
        }
      } else {
        _handleIncomingData(
          Uint8List.fromList(
            RobotPacketCodec.build(1, [control, 1, 0, transactionId]),
          ),
        );
      }
      return;
    }

    if (mode == 2 && data.isNotEmpty) {
      final int command = data[0];
      int? sequenceId;
      if ((command == 2 || command == 3 || command == 7) && data.length >= 2) {
        sequenceId = data[1];
      } else if (command == 4 && data.length >= 3 && data[1] == 4) {
        sequenceId = data[2];
      }
      if (sequenceId != null) {
        await Future<void>.delayed(
          Duration(milliseconds: command == 2 || command == 7 ? 150 : 60),
        );
        if (isFeaturePreviewMode) {
          _handleIncomingData(
            Uint8List.fromList(
              RobotPacketCodec.build(2, [
                command == 7 ? 2 : command,
                sequenceId,
                1,
                0,
              ]),
            ),
          );
        }
      }
      return;
    }

    if (mode == 3 && data.isNotEmpty && data[0] == 1 && data.length >= 2) {
      final int requestId = data[1];
      final Uint8List settings = Uint8List(settingsSize);
      final ByteData bytes = ByteData.sublistView(settings);
      bytes.setInt32(0, pidKpMilli, Endian.little);
      bytes.setInt32(4, pidKiMilli, Endian.little);
      bytes.setInt32(8, pidKdMilli, Endian.little);
      for (int joint = 0; joint < 6; joint++) {
        for (int point = 0; point < 3; point++) {
          bytes.setUint16(
            12 + joint * 6 + point * 2,
            servoCalibrationUs[joint][point],
            Endian.little,
          );
        }
      }
      settings.setRange(48, 53, travelPoseAngles);

      for (int part = 0; part < settingsReadPartCount; part++) {
        await Future<void>.delayed(const Duration(milliseconds: 10));
        if (!isFeaturePreviewMode) {
          return;
        }
        final int start = part * 8;
        final int end = (start + 8).clamp(0, settingsSize).toInt();
        final List<int> partData = List<int>.filled(8, 0);
        partData.setRange(0, end - start, settings.sublist(start, end));
        _handleIncomingData(
          Uint8List.fromList(
            RobotPacketCodec.build(3, [4, part, 0, ...partData, requestId]),
          ),
        );
      }
      return;
    }

    if (mode == 3 && data.length >= 3 && data[0] == 7) {
      await Future<void>.delayed(const Duration(milliseconds: 40));
      if (isFeaturePreviewMode) {
        _handleIncomingData(
          Uint8List.fromList(
            RobotPacketCodec.build(3, [7, 1, data[1], 0, data[2]]),
          ),
        );
        _handleIncomingData(
          Uint8List.fromList(
            RobotPacketCodec.build(0, [
              3,
              data[1] == 1 ? 0x22 : 0x20,
              0,
              0,
              0,
              0,
              0,
              0,
              0,
              0,
              0,
              0,
            ]),
          ),
        );
        _handleIncomingData(
          Uint8List.fromList(
            RobotPacketCodec.build(0, [
              4,
              0x07,
              0x84,
              0x0D,
              0,
              0,
              0,
              0,
              0,
              0,
              0,
              0,
            ]),
          ),
        );
      }
      return;
    }

    if (mode == 3 && data.length >= 2 && data[0] == 8) {
      await Future<void>.delayed(const Duration(milliseconds: 100));
      if (isFeaturePreviewMode) {
        _handleIncomingData(
          Uint8List.fromList(RobotPacketCodec.build(3, [8, 1, 0, data[1]])),
        );
      }
      return;
    }

    if (mode == 3 && data.length >= 6 && data[0] == 5) {
      final int joint = data[1];
      final int pulseUs = data[2] | (data[3] << 8);
      final int requestId = data[5];
      await Future<void>.delayed(const Duration(milliseconds: 40));
      if (isFeaturePreviewMode) {
        _handleIncomingData(
          Uint8List.fromList(
            RobotPacketCodec.build(3, [
              5,
              joint,
              1,
              0,
              pulseUs & 0xFF,
              pulseUs >> 8,
              requestId,
            ]),
          ),
        );
      }
      return;
    }

    if (mode == 3 && data.isNotEmpty && data[0] == 3) {
      await Future<void>.delayed(const Duration(milliseconds: 60));
      if (isFeaturePreviewMode) {
        _handleIncomingData(
          Uint8List.fromList(RobotPacketCodec.build(3, [3, 1, 0, 0])),
        );
      }
    }
  }

  /// Mode와 Data를 고정 16바이트 패킷으로 만들고 체크섬을 계산해 전송한다.
  Future<void> sendCommand(
    int mode, {
    List<int> data = const [],
    bool urgent = false,
    bool replacePending = false,
  }) {
    try {
      final List<int> packet = RobotPacketCodec.build(mode, data);
      if (isFeaturePreviewMode) {
        return _simulateCommand(mode, data);
      }
      return sendPacket(packet, urgent: urgent, replacePending: replacePending);
    } on ArgumentError {
      debugPrint('전송하지 않은 잘못된 명령: mode=$mode, data=$data');
      return Future<void>.value();
    }
  }

  /// 좌우 모터 방향과 PWM을 전송한다. 주행 잠금 상태에서는 보내지 않는다.
  Future<void> sendDrive(
    int leftDirection,
    int leftPwm,
    int rightDirection,
    int rightPwm,
  ) {
    if (!isConnected || isEstopLatched || !isDriveReady || settingsBusy) {
      return Future<void>.value();
    }
    final bool straightPidRequested =
        pidApplied &&
        leftDirection == rightDirection &&
        leftPwm == rightPwm &&
        leftPwm > 0;
    final int direction = straightPidRequested
        ? (leftDirection == 0 ? 1 : -1)
        : 0;
    final bool refreshYawTarget =
        direction != 0 && direction != _previousPidDriveDirection;
    _previousPidDriveDirection = direction;

    return sendCommand(
      0,
      data: [
        leftDirection,
        leftPwm,
        rightDirection,
        rightPwm,
        0,
        1,
        straightPidRequested ? 1 : 0,
        refreshYawTarget ? 1 : 0,
      ],
      replacePending: true,
    );
  }

  /// 차량 PWM을 모두 0으로 만드는 정지 명령을 가장 먼저 전송한다.
  Future<void> stopDrive() {
    if (!isConnected) {
      return Future<void>.value();
    }
    _previousPidDriveDirection = 0;
    return sendCommand(0, data: [0, 0, 0, 0, 0, 0, 0, 0], urgent: true);
  }

  /// 앱 상태를 즉시 잠그고 차량과 로봇팔 출력을 함께 차단한다.
  Future<void> emergencyStop() async {
    isEstopLatched = true;
    pidApplied = false;
    isArmEnabled = false;
    armPoseKnown = false;
    isDriveReady = false;
    pickupWorkflowState = PickupWorkflowState.idle;
    pickupActionBusy = false;
    armCommandBusy = false;
    _armActivationPending = false;
    _drivePosePending = false;
    _suppressArmCompletionMessage = false;
    _pendingArmTransactionId = null;
    _lastArmTransactionId = null;
    _previousPidDriveDirection = 0;
    if (_armAckCompleter?.isCompleted == false) {
      _armAckCompleter!.complete(false);
    }
    if (_teachingPlayCompleter?.isCompleted == false) {
      _teachingPlayCompleter!.complete(false);
    }
    notifyListeners();

    // Control=1은 펌웨어에서 차량 정지 잠금과 로봇팔 출력을 함께 끈다.
    try {
      await sendCommand(0, data: [0, 0, 0, 0, 1], urgent: true);
    } catch (error) {
      debugPrint("비상정지 패킷 전송 실패: $error");
    }
  }

  /// 사용자의 안전 확인 후 STM32에 E-STOP 해제 명령을 보낸다.
  Future<void> clearEmergencyStop() async {
    if (!isConnected) {
      return;
    }

    if (_estopClearCompleter?.isCompleted == false) {
      return;
    }
    final Completer<bool> completer = Completer<bool>();
    _estopClearCompleter = completer;
    try {
      bool succeeded = false;
      for (
        int attempt = 0;
        attempt < 3 && !succeeded && isConnected;
        attempt++
      ) {
        await sendCommand(0, data: [0, 0, 0, 0, 2], urgent: true);
        try {
          succeeded = await completer.future.timeout(
            const Duration(milliseconds: 700),
          );
        } on TimeoutException {
          // STOP 해제는 멱등 명령이므로 ACK 유실 시 같은 안전 명령을 재전송한다.
        }
      }
      isEstopLatched = !succeeded;
      if (!succeeded) {
        lastAckSucceeded = false;
        lastAckMessage = 'STOP 해제 응답 없음';
      }
      notifyListeners();
    } finally {
      if (identical(_estopClearCompleter, completer)) {
        _estopClearCompleter = null;
      }
    }
  }

  /// 서보 출력을 처음 켜고 지정한 원점 자세의 완료 ACK를 기다린다.
  Future<bool> enableArm(List<int> homeAngles) async {
    if (!isConnected || isEstopLatched || !_isArmPoseValid(homeAngles)) {
      return false;
    }
    if (armCommandBusy) {
      return false;
    }
    if (_armAckCompleter?.isCompleted == false) {
      _armAckCompleter!.complete(false);
    }

    final Completer<bool> completer = Completer<bool>();
    final int transactionId = _takeArmTransactionId();
    _armAckCompleter = completer;
    _pendingArmTransactionId = transactionId;
    _lastArmTransactionId = transactionId;
    armPoseKnown = false;
    _armActivationPending = true;
    _drivePosePending = false;
    armCommandBusy = true;
    notifyListeners();

    try {
      await sendCommand(
        1,
        data: [...homeAngles, 1, transactionId, servoSpeedPercent],
      );
      final bool reached = await completer.future.timeout(
        const Duration(seconds: 10),
      );
      if (!reached) {
        return false;
      }
      return true;
    } on TimeoutException {
      lastAckSucceeded = false;
      lastAckMessage = '원점 응답 없음 · STOP 적용';
      await emergencyStop();
      notifyListeners();
      return false;
    } catch (error) {
      lastAckSucceeded = false;
      lastAckMessage = '원점 활성화 전송 실패: $error';
      notifyListeners();
      return false;
    } finally {
      if (identical(_armAckCompleter, completer)) {
        _armAckCompleter = null;
      }
      if (_pendingArmTransactionId == transactionId) {
        _pendingArmTransactionId = null;
      }
      _armActivationPending = false;
      armCommandBusy = false;
      notifyListeners();
    }
  }

  /// 이미 활성화된 로봇팔을 지정 자세로 이동하고 완료 ACK를 기다린다.
  Future<bool> moveArmToOrigin(
    List<int> homeAngles, {
    Duration timeout = const Duration(seconds: 10),
    bool announceCompletion = true,
  }) async {
    if (!isConnected ||
        !isArmEnabled ||
        isEstopLatched ||
        armCommandBusy ||
        !_isArmPoseValid(homeAngles)) {
      return false;
    }
    if (_armAckCompleter?.isCompleted == false) {
      _armAckCompleter!.complete(false);
    }

    final Completer<bool> completer = Completer<bool>();
    final int transactionId = _takeArmTransactionId();
    _armAckCompleter = completer;
    _pendingArmTransactionId = transactionId;
    _lastArmTransactionId = transactionId;
    armPoseKnown = false;
    _armActivationPending = true;
    _suppressArmCompletionMessage = !announceCompletion;
    armCommandBusy = true;
    notifyListeners();

    try {
      await sendCommand(
        1,
        data: [...homeAngles, 0, transactionId, servoSpeedPercent],
        replacePending: true,
      );
      final bool reached = await completer.future.timeout(timeout);
      return reached;
    } on TimeoutException {
      lastAckSucceeded = false;
      lastAckMessage = '로봇팔 응답 없음 · STOP 적용';
      await emergencyStop();
      return false;
    } catch (error) {
      lastAckSucceeded = false;
      lastAckMessage = '로봇팔 자세 이동 전송 실패: $error';
      return false;
    } finally {
      if (identical(_armAckCompleter, completer)) {
        _armAckCompleter = null;
      }
      if (_pendingArmTransactionId == transactionId) {
        _pendingArmTransactionId = null;
      }
      _armActivationPending = false;
      _suppressArmCompletionMessage = false;
      armCommandBusy = false;
      notifyListeners();
    }
  }

  /// PCA9685의 모든 로봇팔 채널 출력을 차단한다.
  Future<void> disableArm() async {
    await sendCommand(
      1,
      data: [0, 0, 0, 0, 0, 0, 2, _takeArmTransactionId(), servoSpeedPercent],
    );
    isArmEnabled = false;
    armPoseKnown = false;
    _armActivationPending = false;
    _drivePosePending = false;
    armCommandBusy = false;
    _speedWarningShownForCurrentMotion = false;
    notifyListeners();
  }

  /// 수동 조작에서 만든 최신 6관절 목표 자세를 STM32에 보낸다.
  Future<void> sendArmPose(List<int> angles) {
    if (!isConnected ||
        !isArmEnabled ||
        isEstopLatched ||
        armCommandBusy ||
        !_isArmPoseValid(angles)) {
      return Future<void>.value();
    }
    final int transactionId = _takeArmTransactionId();
    _lastArmTransactionId = transactionId;
    armPoseKnown = false;
    notifyListeners();
    return sendCommand(
      1,
      data: [...angles, 0, transactionId, servoSpeedPercent],
      replacePending: true,
    );
  }

  /// 완료된 동작의 마지막 명령 자세를 수동 제어 화면에 알린다.
  void rememberCommandedArmPose(
    List<int> pose, {
    bool preserveGripper = false,
  }) {
    RobotArmControlTab.applyCommandedPose(
      pose,
      preserveGripper: preserveGripper,
    );
    notifyListeners();
  }

  /// Flash의 이동 자세를 실행하며 현재 그리퍼 명령값을 유지한다.
  Future<bool> prepareTravelSequencePreservingGripper() async {
    if (!isConnected ||
        !isArmEnabled ||
        isEstopLatched ||
        armCommandBusy ||
        settingsBusy) {
      isDriveReady = false;
      notifyListeners();
      return false;
    }
    if (_armAckCompleter?.isCompleted == false) {
      _armAckCompleter!.complete(false);
    }

    final Completer<bool> completer = Completer<bool>();
    final int transactionId = _takeArmTransactionId();
    _armAckCompleter = completer;
    _pendingArmTransactionId = transactionId;
    _lastArmTransactionId = transactionId;
    armPoseKnown = false;
    isDriveReady = false;
    _drivePosePending = true;
    armCommandBusy = true;
    notifyListeners();

    try {
      await sendCommand(
        1,
        data: [0, 0, 0, 0, 0, 0, 3, transactionId, servoSpeedPercent],
        replacePending: true,
      );
      final bool reached = await completer.future.timeout(
        const Duration(minutes: 5),
        onTimeout: () => false,
      );
      isDriveReady = reached;
      return reached;
    } catch (error) {
      debugPrint('그리퍼 유지 이동 자세 전송 실패: $error');
      lastAckSucceeded = false;
      lastAckMessage = '물체를 유지한 이동 자세 실행에 실패했습니다.';
      return false;
    } finally {
      if (identical(_armAckCompleter, completer)) {
        _armAckCompleter = null;
      }
      if (_pendingArmTransactionId == transactionId) {
        _pendingArmTransactionId = null;
      }
      _drivePosePending = false;
      armCommandBusy = false;
      notifyListeners();
    }
  }

  /// 선택한 티칭 시퀀스의 Flash 저장 결과 ACK를 기다린다.
  Future<bool> waitForTeachingAck(int sequenceId) {
    if (_teachingAckCompleter?.isCompleted == false) {
      _teachingAckCompleter!.complete(false);
    }

    final Completer<bool> completer = Completer<bool>();
    _teachingAckCompleter = completer;
    _teachingAckSequence = sequenceId;

    return completer.future
        .timeout(const Duration(seconds: 5), onTimeout: () => false)
        .whenComplete(() {
          if (_teachingAckCompleter == completer) {
            _teachingAckCompleter = null;
            _teachingAckSequence = null;
          }
        });
  }

  /// 선택한 티칭 시퀀스의 초기화 결과 ACK를 기다린다.
  Future<bool> waitForTeachingResetAck(int sequenceId) {
    if (_teachingResetCompleter?.isCompleted == false) {
      _teachingResetCompleter!.complete(false);
    }

    final Completer<bool> completer = Completer<bool>();
    _teachingResetCompleter = completer;
    _teachingResetSequence = sequenceId;

    return completer.future
        .timeout(const Duration(seconds: 5), onTimeout: () => false)
        .whenComplete(() {
          if (_teachingResetCompleter == completer) {
            _teachingResetCompleter = null;
            _teachingResetSequence = null;
          }
        });
  }

  /// 선택한 티칭 시퀀스가 끝나거나 실패할 때 펌웨어가 보내는 ACK를 기다린다.
  Future<bool> waitForTeachingPlayAck(int sequenceId) {
    if (_teachingPlayCompleter?.isCompleted == false) {
      _teachingPlayCompleter!.complete(false);
    }

    final Completer<bool> completer = Completer<bool>();
    _teachingPlayCompleter = completer;
    _teachingPlaySequence = sequenceId;
    return completer.future
        .timeout(const Duration(minutes: 5), onTimeout: () => false)
        .whenComplete(() {
          if (identical(_teachingPlayCompleter, completer)) {
            _teachingPlayCompleter = null;
            _teachingPlaySequence = null;
          }
        });
  }

  /// STM32 RAM에 있는 UTF-8 시퀀스 이름을 7바이트 응답 조각으로 읽는다.
  Future<String?> requestTeachingName(int sequenceId) async {
    if (!isPhysicallyConnected ||
        sequenceId < 1 ||
        sequenceId > TeachingController.sequenceCount) {
      return null;
    }
    if (_teachingNameCompleter?.isCompleted == false) {
      _teachingNameCompleter!.complete(null);
    }

    final int requestId = _nextTeachingNameRequestId;
    _nextTeachingNameRequestId = (_nextTeachingNameRequestId % 0xFF) + 1;
    final Completer<String?> completer = Completer<String?>();
    _teachingNameCompleter = completer;
    _pendingTeachingNameSequence = sequenceId;
    _pendingTeachingNameRequestId = requestId;
    _pendingTeachingNameLength = null;
    _teachingNameReceiveParts = 0;
    _teachingNameBuffer.fillRange(0, _teachingNameBuffer.length, 0);

    try {
      await sendCommand(2, data: [5, sequenceId, requestId]);
      return await completer.future.timeout(
        const Duration(seconds: 2),
        onTimeout: () => null,
      );
    } finally {
      if (identical(_teachingNameCompleter, completer)) {
        _teachingNameCompleter = null;
        _pendingTeachingNameSequence = null;
        _pendingTeachingNameRequestId = null;
        _pendingTeachingNameLength = null;
        _teachingNameReceiveParts = 0;
      }
    }
  }

  /// STM32 Flash에서 부팅 시 읽은 시퀀스 웨이포인트를 앱으로 가져온다.
  Future<List<List<int>>?> requestTeachingSequence(int sequenceId) async {
    if (!isPhysicallyConnected ||
        sequenceId < 1 ||
        sequenceId > TeachingController.sequenceCount) {
      return null;
    }
    if (_teachingSequenceCompleter?.isCompleted == false) {
      _teachingSequenceCompleter!.complete(null);
    }

    final int requestId = _nextTeachingSequenceRequestId;
    _nextTeachingSequenceRequestId =
        (_nextTeachingSequenceRequestId % 0xFF) + 1;
    final Completer<List<List<int>>?> completer = Completer<List<List<int>>?>();
    _teachingSequenceCompleter = completer;
    _pendingTeachingSequence = sequenceId;
    _pendingTeachingSequenceRequestId = requestId;
    _pendingTeachingWaypointCount = null;
    _teachingWaypointReceiveMask = 0;
    _teachingWaypointBuffer.fillRange(0, _teachingWaypointBuffer.length, null);

    try {
      await sendCommand(2, data: [6, sequenceId, requestId]);
      return await completer.future.timeout(
        const Duration(seconds: 3),
        onTimeout: () => null,
      );
    } finally {
      if (identical(_teachingSequenceCompleter, completer)) {
        _teachingSequenceCompleter = null;
        _pendingTeachingSequence = null;
        _pendingTeachingSequenceRequestId = null;
        _pendingTeachingWaypointCount = null;
        _teachingWaypointReceiveMask = 0;
      }
    }
  }

  /// STM32 Flash의 PID, 서보 보정과 이동 자세 53바이트를 조각으로 읽는다.
  Future<bool> requestSettings() async {
    if (!isConnected || settingsBusy) {
      return false;
    }

    settingsBusy = true;
    final int requestId = _nextSettingsRequestId;
    _nextSettingsRequestId = (_nextSettingsRequestId % 0xFF) + 1;
    _pendingSettingsRequestId = requestId;
    _settingsReceiveParts = 0;
    _settingsReceiveBuffer.fillRange(0, settingsSize, 0);
    final Completer<bool> completer = Completer<bool>();
    _settingsReadCompleter = completer;
    notifyListeners();

    try {
      await sendCommand(3, data: [1, requestId]);
      return await completer.future.timeout(
        const Duration(seconds: 2),
        onTimeout: () => false,
      );
    } catch (error) {
      debugPrint('설정 조회 실패: $error');
      return false;
    } finally {
      if (_settingsReadCompleter == completer) {
        _settingsReadCompleter = null;
      }
      if (_pendingSettingsRequestId == requestId) {
        _pendingSettingsRequestId = null;
      }
      settingsBusy = false;
      notifyListeners();
    }
  }

  /// PID, 서보 3점 보정과 5관절 이동 자세를 조각 전송해 Flash에 저장한다.
  Future<bool> saveSettings({
    required int kpMilli,
    required int kiMilli,
    required int kdMilli,
    required List<List<int>> servoPulsesUs,
    required List<int> travelPose,
  }) async {
    if (!isConnected ||
        settingsBusy ||
        servoPulsesUs.length != 6 ||
        servoPulsesUs.any((values) => values.length != 3) ||
        travelPose.length != 5 ||
        travelPose.any((angle) => angle < 0 || angle > 180) ||
        kpMilli < 0 ||
        kpMilli > pidKpMaxMilli ||
        kpMilli % pidKpStepMilli != 0 ||
        kiMilli < 0 ||
        kiMilli > 100000 ||
        kdMilli < 0 ||
        kdMilli > 100000) {
      return false;
    }

    final List<List<int>> normalizedPulses = servoPulsesUs
        .map((values) => List<int>.from(values))
        .toList();
    normalizedPulses[5][1] =
        (normalizedPulses[5][0] + normalizedPulses[5][2]) ~/ 2;

    final Uint8List data = Uint8List(settingsSize);
    final ByteData bytes = ByteData.sublistView(data);
    bytes.setInt32(0, kpMilli, Endian.little);
    bytes.setInt32(4, kiMilli, Endian.little);
    bytes.setInt32(8, kdMilli, Endian.little);
    for (int joint = 0; joint < 6; joint++) {
      for (int point = 0; point < 3; point++) {
        bytes.setUint16(
          12 + joint * 6 + point * 2,
          normalizedPulses[joint][point],
          Endian.little,
        );
      }
    }
    data.setRange(48, 53, travelPose);

    settingsBusy = true;
    final Completer<bool> completer = Completer<bool>();
    _settingsSaveCompleter = completer;
    notifyListeners();

    try {
      for (int part = 0; part < settingsWritePartCount; part++) {
        final int start = part * 10;
        final int end = (start + 10 < settingsSize) ? start + 10 : settingsSize;
        final List<int> payload = List.filled(10, 0);
        payload.setRange(0, end - start, data.sublist(start, end));
        await sendCommand(3, data: [2, part, ...payload]);
      }

      final int crc = _calculateSettingsCrc(data);
      await sendCommand(3, data: [3, settingsSize, crc & 0xFF, crc >> 8]);
      final bool succeeded = await completer.future.timeout(
        const Duration(seconds: 5),
        onTimeout: () => false,
      );
      if (succeeded) {
        pidKpMilli = kpMilli;
        pidKiMilli = kiMilli;
        pidKdMilli = kdMilli;
        servoCalibrationUs = normalizedPulses
            .map((values) => List<int>.from(values))
            .toList();
        travelPoseAngles = List<int>.from(travelPose);
        settingsLoaded = true;
      }
      return succeeded;
    } catch (error) {
      debugPrint('설정 저장 전송 실패: $error');
      return false;
    } finally {
      if (_settingsSaveCompleter == completer) {
        _settingsSaveCompleter = null;
      }
      settingsBusy = false;
      notifyListeners();
    }
  }

  /// 차량을 정지한 뒤 방향 안정화 PID의 실제 적용 상태를 펌웨어에 요청한다.
  Future<bool> setPidEnabled(bool enabled) async {
    if (!isConnected ||
        settingsBusy ||
        (enabled && (isEstopLatched || !isDriveReady))) {
      return false;
    }

    await stopDrive();
    settingsBusy = true;
    lastAckMessage = null;
    notifyListeners();

    int? calibrationRequestId;
    int? pidRequestId;
    Completer<bool>? calibrationCompleter;
    Completer<bool>? pidCompleter;
    try {
      if (enabled) {
        calibrationRequestId = _nextSettingsRequestId;
        _nextSettingsRequestId = (_nextSettingsRequestId % 0xFF) + 1;
        _pendingImuCalibrationRequestId = calibrationRequestId;
        _imuCalibrationObservedRunning = false;
        calibrationCompleter = Completer<bool>();
        _imuCalibrationCompleter = calibrationCompleter;
        _imuCalibrationStartedAnnounced = false;
        _imuCalibrationCompletedAnnounced = false;
        lastAckSucceeded = true;
        lastAckIsWarning = true;
        lastAckMessage = 'IMU 영점 보정 중 · 3초간 차량을 움직이지 마세요.';
        notifyListeners();

        await sendCommand(3, data: [8, calibrationRequestId]);
        final bool calibrated = await calibrationCompleter.future.timeout(
          const Duration(seconds: 10),
          onTimeout: () => false,
        );
        if (!calibrated) {
          if (lastAckMessage == null || lastAckSucceeded) {
            lastAckSucceeded = false;
            lastAckIsWarning = false;
            lastAckMessage = 'IMU 영점 보정 ACK를 받지 못했습니다.';
          }
          return false;
        }
      }

      pidRequestId = _nextSettingsRequestId;
      _nextSettingsRequestId = (_nextSettingsRequestId % 0xFF) + 1;
      _pendingPidToggleRequestId = pidRequestId;
      _pendingPidToggleValue = enabled;
      pidCompleter = Completer<bool>();
      _pidToggleCompleter = pidCompleter;
      await sendCommand(3, data: [7, enabled ? 1 : 0, pidRequestId]);
      final bool succeeded = await pidCompleter.future.timeout(
        const Duration(seconds: 2),
        onTimeout: () => false,
      );
      if (!succeeded && lastAckMessage == null) {
        lastAckSucceeded = false;
        lastAckMessage = 'PID 적용 상태 변경 ACK를 받지 못했습니다.';
      }
      return succeeded;
    } catch (error) {
      lastAckSucceeded = false;
      lastAckMessage = 'PID 적용 상태 변경 전송 실패: $error';
      return false;
    } finally {
      if (identical(_imuCalibrationCompleter, calibrationCompleter)) {
        _imuCalibrationCompleter = null;
      }
      if (_pendingImuCalibrationRequestId == calibrationRequestId) {
        _pendingImuCalibrationRequestId = null;
      }
      if (identical(_pidToggleCompleter, pidCompleter)) {
        _pidToggleCompleter = null;
      }
      if (_pendingPidToggleRequestId == pidRequestId) {
        _pendingPidToggleRequestId = null;
        _pendingPidToggleValue = null;
      }
      settingsBusy = false;
      notifyListeners();
    }
  }

  /// 수신한 53바이트 설정을 PID, 관절별 펄스와 이동 자세로 해석한다.
  void _decodeSettings(Uint8List data) {
    final ByteData bytes = ByteData.sublistView(data);
    pidKpMilli = bytes.getInt32(0, Endian.little);
    pidKiMilli = bytes.getInt32(4, Endian.little);
    pidKdMilli = bytes.getInt32(8, Endian.little);
    servoCalibrationUs = List<List<int>>.generate(
      6,
      (joint) => List<int>.generate(
        3,
        (point) => bytes.getUint16(12 + joint * 6 + point * 2, Endian.little),
      ),
    );
    travelPoseAngles = data.sublist(48, 53);
  }

  /// 선택한 서보 하나를 현재 속도 설정의 S-curve로 시험 위치까지 이동한다.
  Future<void> previewServoPulse(int joint, int pulseUs) async {
    if (!isConnected || !isArmEnabled || joint < 0 || joint >= 6) {
      return;
    }

    final int requestId = _nextPreviewRequestId;
    _nextPreviewRequestId = (_nextPreviewRequestId % 0xFF) + 1;
    _pendingPreviewRequestId = requestId;
    _pendingPreviewJoint = joint;
    _pendingPreviewPulseUs = pulseUs;
    servoPreviewMoving = true;
    notifyListeners();

    try {
      await sendCommand(
        3,
        data: [
          5,
          joint,
          pulseUs & 0xFF,
          pulseUs >> 8,
          servoSpeedPercent,
          requestId,
        ],
      );
    } catch (error) {
      if (_pendingPreviewRequestId == requestId) {
        servoPreviewMoving = false;
        _pendingPreviewRequestId = null;
        _pendingPreviewJoint = null;
        _pendingPreviewPulseUs = null;
        lastAckSucceeded = false;
        lastAckIsWarning = false;
        lastAckMessage = '원점 보정 명령 전송 실패: $error';
        notifyListeners();
      }
    }
  }

  /// 원점 보정용 단일 서보 출력을 중지한다.
  Future<void> stopServoPreview() async {
    servoPreviewMoving = false;
    _pendingPreviewRequestId = null;
    _pendingPreviewJoint = null;
    _pendingPreviewPulseUs = null;
    notifyListeners();
    if (!isConnected) {
      return;
    }
    await sendCommand(3, data: [6]);
  }

  /// 설정 전체가 손상되지 않았는지 확인할 CRC-16/CCITT 값을 계산한다.
  int _calculateSettingsCrc(List<int> data) {
    int crc = 0xFFFF;
    for (final int value in data) {
      crc ^= value << 8;
      for (int bit = 0; bit < 8; bit++) {
        crc = (crc & 0x8000) != 0
            ? ((crc << 1) ^ 0x1021) & 0xFFFF
            : (crc << 1) & 0xFFFF;
      }
    }
    return crc;
  }

  @override
  void dispose() {
    _statusWatchdogTimer?.cancel();
    unawaited(_inputSubscription?.cancel());
    unawaited(_connectionStateSubscription?.cancel());
    connection?.dispose();
    super.dispose();
  }
}

// ======================================================
// 티칭 시퀀스와 웨이포인트 상태 관리자
// ======================================================

class _TeachingPreviewSnapshot {
  const _TeachingPreviewSnapshot(
    this.selectedSequence,
    this.names,
    this.waypoints,
    this.savedNames,
    this.savedWaypoints,
  );

  final int selectedSequence;
  final List<String> names;
  final List<List<List<int>>> waypoints;
  final List<String> savedNames;
  final List<List<List<int>>> savedWaypoints;
}

/// 앱이 실행되는 동안 12개 티칭 시퀀스의 웨이포인트를 메모리에 보관한다.
///
/// 앱 메모리의 변경 내용은 사용자가 업로드 버튼을 눌러야 STM32 Flash에 저장된다.
class TeachingController with ChangeNotifier {
  static const int sequenceCount = 12;
  static const int maxWaypoints = 30;
  static const int maxNameBytes = 21;
  static const int travelSequence = 9;
  static const int pickupPoseSequence = 10;
  static const int gripSequence = 11;
  static const int releaseSequence = 12;

  int _selectedSequence = 1;
  bool _uploading = false;
  bool _resetting = false;
  bool _playing = false;
  bool _loadingNames = false;
  bool _loadingSequence = false;
  Object? _syncedConnection;
  Object? _sequenceConnection;
  final Set<int> _loadedSequences = <int>{};
  final List<String> _sequenceNames = List.generate(
    sequenceCount,
    (index) => defaultSequenceName(index + 1),
  );
  final List<String> _savedSequenceNames = List.generate(
    sequenceCount,
    (index) => defaultSequenceName(index + 1),
  );
  final List<List<List<int>>> _sequenceWaypoints = List.generate(
    sequenceCount,
    (_) => [],
  );
  final List<List<List<int>>> _savedSequenceWaypoints = List.generate(
    sequenceCount,
    (_) => [],
  );

  int get selectedSequence => _selectedSequence;
  bool get isUploading => _uploading;
  bool get isResetting => _resetting;
  bool get isPlaying => _playing;
  bool get isBusy =>
      _uploading || _resetting || _playing || _loadingNames || _loadingSequence;
  String get selectedSequenceName => sequenceName(_selectedSequence);
  List<List<int>> get currentWaypoints =>
      _sequenceWaypoints[_selectedSequence - 1];

  /// 앱 RAM에 해당 전용 시퀀스가 있으면 마지막 명령 자세를 UI 동기화에 사용한다.
  List<int>? lastPoseForSequence(int sequence) {
    if (sequence < 1 || sequence > sequenceCount) {
      return null;
    }
    final List<List<int>> waypoints = _savedSequenceWaypoints[sequence - 1];
    return waypoints.isEmpty ? null : List<int>.from(waypoints.last);
  }

  /// Flash 시퀀스의 마지막 자세를 필요할 때 한 번 읽어 앱에 보관한다.
  Future<List<int>?> _loadLastPose(
    BluetoothController bluetooth,
    int sequence,
  ) async {
    final List<int>? cached = _loadedSequences.contains(sequence)
        ? lastPoseForSequence(sequence)
        : null;
    if (cached != null) {
      return cached;
    }
    final List<List<int>>? waypoints = await bluetooth.requestTeachingSequence(
      sequence,
    );
    if (waypoints == null ||
        waypoints.isEmpty ||
        waypoints.any((pose) => !_isPoseValid(pose))) {
      return null;
    }
    _savedSequenceWaypoints[sequence - 1]
      ..clear()
      ..addAll(waypoints.map((pose) => List<int>.from(pose)));
    _loadedSequences.add(sequence);
    return List<int>.from(waypoints.last);
  }

  static String defaultSequenceName(int sequence) {
    return switch (sequence) {
      travelSequence => '이동 자세',
      pickupPoseSequence => '잡기 위치',
      gripSequence => '잡기',
      releaseSequence => '놓기',
      _ => '시퀀스 $sequence',
    };
  }

  static bool isReservedSequence(int sequence) =>
      sequence >= travelSequence && sequence <= releaseSequence;

  static String storageName(int sequence, String editableName) {
    return switch (sequence) {
      travelSequence => 'Travel',
      pickupPoseSequence => 'Pickup Pose',
      gripSequence => 'Pick',
      releaseSequence => 'Place',
      _ => editableName,
    };
  }

  String sequenceName(int sequence) {
    if (sequence < 1 || sequence > sequenceCount) {
      return '시퀀스';
    }
    if (isReservedSequence(sequence)) {
      return defaultSequenceName(sequence);
    }
    return _sequenceNames[sequence - 1];
  }

  static bool _isNameValid(String name) {
    final String trimmed = name.trim();
    final List<int> bytes = utf8.encode(trimmed);
    return trimmed.isNotEmpty &&
        bytes.length <= maxNameBytes &&
        !trimmed.runes.any((rune) => rune < 0x20 || rune == 0x7F);
  }

  /// 일반 시퀀스 이름만 앱 RAM에서 변경한다. Flash 저장은 업로드 때 수행한다.
  String? renameSelectedSequence(String name) {
    if (isBusy || isReservedSequence(_selectedSequence)) {
      return '이 시퀀스 이름은 변경할 수 없습니다.';
    }
    final String trimmed = name.trim();
    if (!_isNameValid(trimmed)) {
      return '이름은 비워둘 수 없으며 UTF-8 기준 $maxNameBytes바이트 이하여야 합니다.';
    }
    _sequenceNames[_selectedSequence - 1] = trimmed;
    notifyListeners();
    return null;
  }

  /// 새 HC-05 연결마다 변경 가능한 1~8번 이름을 Flash에서 한 번 읽는다.
  Future<void> loadNamesFromSTM32(BluetoothController bluetooth) async {
    final Object? activeConnection = bluetooth.connection;
    if (_loadingNames ||
        activeConnection == null ||
        identical(_syncedConnection, activeConnection)) {
      return;
    }

    _loadingNames = true;
    notifyListeners();
    try {
      for (int sequence = 1; sequence < travelSequence; sequence++) {
        final String? name = await bluetooth.requestTeachingName(sequence);
        if (name != null && _isNameValid(name)) {
          _sequenceNames[sequence - 1] = name.trim();
          _savedSequenceNames[sequence - 1] = name.trim();
        }
      }
      _syncedConnection = activeConnection;
      notifyListeners();
    } finally {
      _loadingNames = false;
      notifyListeners();
    }
  }

  /// 새 연결에서 이름과 현재 선택 시퀀스를 버튼 없이 자동으로 읽는다.
  Future<void> syncFromSTM32(BluetoothController bluetooth) async {
    if (_loadingNames || _loadingSequence) {
      return;
    }
    await loadNamesFromSTM32(bluetooth);
    await loadSelectedSequenceFromSTM32(bluetooth);
  }

  /// 기능 확인 중 만든 웨이포인트를 나중에 버릴 수 있도록 전체 상태를 복사한다.
  _TeachingPreviewSnapshot _createFeaturePreviewSnapshot() {
    return _TeachingPreviewSnapshot(
      _selectedSequence,
      List<String>.from(_sequenceNames),
      _sequenceWaypoints
          .map(
            (sequence) => sequence.map((pose) => List<int>.from(pose)).toList(),
          )
          .toList(),
      List<String>.from(_savedSequenceNames),
      _savedSequenceWaypoints
          .map(
            (sequence) => sequence.map((pose) => List<int>.from(pose)).toList(),
          )
          .toList(),
    );
  }

  /// 기능 확인 모드 진입 전에 사용하던 시퀀스와 웨이포인트를 되돌린다.
  void _restoreFeaturePreviewSnapshot(_TeachingPreviewSnapshot snapshot) {
    if (isBusy ||
        snapshot.names.length != sequenceCount ||
        snapshot.waypoints.length != sequenceCount ||
        snapshot.savedNames.length != sequenceCount ||
        snapshot.savedWaypoints.length != sequenceCount) {
      return;
    }
    _sequenceNames
      ..clear()
      ..addAll(snapshot.names);
    for (int index = 0; index < sequenceCount; index++) {
      _sequenceWaypoints[index]
        ..clear()
        ..addAll(snapshot.waypoints[index].map((pose) => List<int>.from(pose)));
      _savedSequenceWaypoints[index]
        ..clear()
        ..addAll(
          snapshot.savedWaypoints[index].map((pose) => List<int>.from(pose)),
        );
    }
    _savedSequenceNames
      ..clear()
      ..addAll(snapshot.savedNames);
    _selectedSequence = snapshot.selectedSequence;
    notifyListeners();
  }

  /// 패킷에 넣을 수 있는 6개 관절 값인지 검사한다.
  bool _isPoseValid(List<int> pose) {
    return pose.length == 6 &&
        pose.every((value) => value >= 0 && value <= 180);
  }

  /// TEST 모드에서는 저장본으로 되돌린 뒤 다른 시퀀스를 선택한다.
  void selectSequence(int sequence) {
    if (isBusy || sequence < 1 || sequence > sequenceCount) {
      return;
    }
    _restoreSequence(_selectedSequence);
    _restoreSequence(sequence);
    _selectedSequence = sequence;
    notifyListeners();
  }

  void _restoreSequence(int sequence) {
    _sequenceNames[sequence - 1] = _savedSequenceNames[sequence - 1];
    _sequenceWaypoints[sequence - 1]
      ..clear()
      ..addAll(
        _savedSequenceWaypoints[sequence - 1].map(
          (pose) => List<int>.from(pose),
        ),
      );
  }

  /// 다른 시퀀스를 고르면 현재 미저장 편집을 버리고 Flash 원본을 자동 조회한다.
  Future<String?> selectSequenceFromSTM32(
    BluetoothController bluetooth,
    int sequence,
  ) async {
    if (isBusy || sequence < 1 || sequence > sequenceCount) {
      return '다른 티칭 작업이 진행 중이거나 시퀀스 번호가 잘못되었습니다.';
    }
    if (!bluetooth.isPhysicallyConnected) {
      selectSequence(sequence);
      return null;
    }

    _loadingSequence = true;
    notifyListeners();
    try {
      final String? name = isReservedSequence(sequence)
          ? storageName(sequence, '')
          : await bluetooth.requestTeachingName(sequence);
      final List<List<int>>? waypoints = await bluetooth
          .requestTeachingSequence(sequence);
      if (name == null || !_isNameValid(name) || waypoints == null) {
        return 'STM32에서 저장된 시퀀스를 읽지 못했습니다.';
      }

      _restoreSequence(_selectedSequence);
      _sequenceNames[sequence - 1] = isReservedSequence(sequence)
          ? defaultSequenceName(sequence)
          : name.trim();
      _savedSequenceNames[sequence - 1] = _sequenceNames[sequence - 1];
      _sequenceWaypoints[sequence - 1]
        ..clear()
        ..addAll(waypoints.map((pose) => List<int>.from(pose)));
      _savedSequenceWaypoints[sequence - 1]
        ..clear()
        ..addAll(waypoints.map((pose) => List<int>.from(pose)));
      _selectedSequence = sequence;
      _loadedSequences.add(sequence);
      notifyListeners();
      return null;
    } finally {
      _loadingSequence = false;
      notifyListeners();
    }
  }

  /// 새 HC-05 연결에서 현재 선택 시퀀스를 최초 한 번 자동으로 읽는다.
  Future<void> loadSelectedSequenceFromSTM32(
    BluetoothController bluetooth,
  ) async {
    final Object? activeConnection = bluetooth.connection;
    if (activeConnection == null || _loadingSequence) {
      return;
    }
    if (!identical(_sequenceConnection, activeConnection)) {
      _sequenceConnection = activeConnection;
      _loadedSequences.clear();
    }
    if (_loadedSequences.contains(_selectedSequence)) {
      return;
    }
    await selectSequenceFromSTM32(bluetooth, _selectedSequence);
  }

  /// 마지막으로 요청한 로봇팔 자세를 현재 시퀀스에 복사해 추가한다.
  bool addCurrentPose(List<int> pose) {
    if (isBusy ||
        currentWaypoints.length >= maxWaypoints ||
        !_isPoseValid(pose)) {
      return false;
    }
    currentWaypoints.add(List<int>.from(pose));
    notifyListeners();
    return true;
  }

  /// 기존 웨이포인트 하나를 사용자가 입력한 자세로 교체한다.
  bool updateWaypoint(int index, List<int> pose) {
    if (isBusy ||
        index < 0 ||
        index >= currentWaypoints.length ||
        !_isPoseValid(pose)) {
      return false;
    }
    currentWaypoints[index] = List<int>.from(pose);
    notifyListeners();
    return true;
  }

  /// 현재 시퀀스에서 선택한 웨이포인트 하나를 삭제한다.
  void removeWaypoint(int index) {
    if (isBusy || index < 0 || index >= currentWaypoints.length) {
      return;
    }
    currentWaypoints.removeAt(index);
    notifyListeners();
  }

  /// 현재 선택한 시퀀스의 앱 메모리만 비운다.
  void clearCurrentSequence() {
    if (isBusy) {
      return;
    }
    currentWaypoints.clear();
    notifyListeners();
  }

  Future<void> _sendSequenceParts(
    BluetoothController bluetooth,
    int sequenceId,
    List<int> nameBytes,
    List<List<int>> waypoints,
  ) async {
    await bluetooth.sendCommand(
      2,
      data: [4, 1, sequenceId, waypoints.length, nameBytes.length],
    );
    for (int offset = 0, part = 0; offset < nameBytes.length; part++) {
      final int end = (offset + 8).clamp(0, nameBytes.length).toInt();
      final List<int> payload = List<int>.filled(8, 0);
      payload.setRange(0, end - offset, nameBytes.sublist(offset, end));
      await bluetooth.sendCommand(
        2,
        data: [4, 5, sequenceId, part, ...payload],
      );
      offset = end;
    }
    for (int index = 0; index < waypoints.length; index++) {
      final List<int> pose = waypoints[index];
      await bluetooth.sendCommand(
        2,
        data: [4, 2, sequenceId, index, pose[0], pose[1], pose[2]],
      );
      await bluetooth.sendCommand(
        2,
        data: [4, 3, sequenceId, index, pose[3], pose[4], pose[5]],
      );
    }
  }

  void _rememberCurrentSequence() {
    _savedSequenceNames[_selectedSequence - 1] =
        _sequenceNames[_selectedSequence - 1];
    _savedSequenceWaypoints[_selectedSequence - 1]
      ..clear()
      ..addAll(currentWaypoints.map((pose) => List<int>.from(pose)));
    _loadedSequences.add(_selectedSequence);
  }

  /// 모든 웨이포인트를 전반부·후반부 조각으로 나눠 STM32 Flash에 저장한다.
  ///
  /// COMMIT ACK가 성공한 경우에만 업로드가 완료된 것으로 판단한다.
  Future<String?> uploadToSTM32(BluetoothController bluetooth) async {
    if (isBusy) {
      return null;
    }
    if (!bluetooth.isConnected) {
      return '먼저 HC-05에 연결해주세요.';
    }
    if (currentWaypoints.isEmpty) {
      return '업로드할 웨이포인트가 없습니다.';
    }

    final int sequenceId = _selectedSequence;
    final List<int> nameBytes = utf8.encode(
      storageName(sequenceId, sequenceName(sequenceId)),
    );
    final List<List<int>> waypoints = currentWaypoints
        .map((pose) => List<int>.from(pose))
        .toList();
    if (waypoints.any((pose) => !_isPoseValid(pose))) {
      return '웨이포인트에 잘못된 관절 값이 있습니다.';
    }
    final Future<bool> ack = bluetooth.waitForTeachingAck(sequenceId);

    _uploading = true;
    notifyListeners();

    try {
      /*
       * RFCOMM과 앱의 단일 전송 큐가 패킷 순서를 보장한다. 고정 지연을 넣으면
       * Flash ACK와 무관하게 UI만 늦게 풀리므로 각 write 완료 뒤 바로 이어 보낸다.
       */
      await _sendSequenceParts(bluetooth, sequenceId, nameBytes, waypoints);
      await bluetooth.sendCommand(2, data: [4, 4, sequenceId]);
      final bool succeeded = await ack;
      if (!succeeded) {
        return '저장 ACK를 받지 못했거나 Flash 저장에 실패했습니다.';
      }
      _rememberCurrentSequence();
      return null;
    } catch (error) {
      return '업로드 전송 실패: $error';
    } finally {
      _uploading = false;
      notifyListeners();
    }
  }

  /// 선택한 시퀀스를 STM32 RAM과 Flash에서 초기화하고 성공 후 앱 목록도 비운다.
  Future<String?> resetOnSTM32(BluetoothController bluetooth) async {
    if (isBusy) {
      return '다른 티칭 작업이 진행 중입니다.';
    }
    if (!bluetooth.isConnected) {
      return '먼저 HC-05에 연결해주세요.';
    }

    final int sequenceId = _selectedSequence;
    final Future<bool> ack = bluetooth.waitForTeachingResetAck(sequenceId);
    _resetting = true;
    notifyListeners();

    try {
      await bluetooth.sendCommand(2, data: [3, sequenceId]);
      final bool succeeded = await ack;
      if (!succeeded) {
        return '초기화 ACK를 받지 못했거나 Flash 초기화에 실패했습니다.';
      }
      currentWaypoints.clear();
      _sequenceNames[sequenceId - 1] = defaultSequenceName(sequenceId);
      _rememberCurrentSequence();
      return null;
    } catch (error) {
      return '초기화 전송 실패: $error';
    } finally {
      _resetting = false;
      notifyListeners();
    }
  }

  /// 현재 앱 편집본을 Flash에 쓰지 않고 STM32 임시 버퍼에서 재생한다.
  Future<String?> playOnSTM32(BluetoothController bluetooth) async {
    if (isBusy) {
      return '다른 티칭 작업이 진행 중입니다.';
    }
    if (!bluetooth.isConnected || !bluetooth.isArmEnabled) {
      return 'HC-05 연결과 로봇팔 출력 활성화를 먼저 확인해주세요.';
    }
    if (currentWaypoints.isEmpty) {
      return '재생할 웨이포인트가 없습니다.';
    }

    final int sequenceId = _selectedSequence;
    final List<List<int>> waypoints = currentWaypoints
        .map((pose) => List<int>.from(pose))
        .toList();
    final List<int> nameBytes = utf8.encode(
      storageName(sequenceId, sequenceName(sequenceId)),
    );
    if (waypoints.any((pose) => !_isPoseValid(pose))) {
      return '웨이포인트에 잘못된 관절 값이 있습니다.';
    }

    final Future<bool> ack = bluetooth.waitForTeachingPlayAck(sequenceId);
    _playing = true;
    notifyListeners();
    try {
      await _sendSequenceParts(bluetooth, sequenceId, nameBytes, waypoints);
      await bluetooth.sendCommand(
        2,
        data: [7, sequenceId, bluetooth.servoSpeedPercent],
      );
      final bool succeeded = await ack;
      if (succeeded) {
        bluetooth.rememberCommandedArmPose(waypoints.last);
      }
      return succeeded ? null : '시퀀스 재생 실패 또는 완료 ACK 시간 초과입니다.';
    } catch (error) {
      return '시퀀스 재생 전송 실패: $error';
    } finally {
      _playing = false;
      notifyListeners();
    }
  }

  /// 차량 전용 동작은 화면 편집본이 아닌 저장된 Flash 시퀀스를 재생한다.
  Future<String?> playSequenceOnSTM32(
    BluetoothController bluetooth,
    int sequenceId,
  ) async {
    if (isBusy) {
      return '다른 티칭 작업이 진행 중입니다.';
    }
    if (!bluetooth.isConnected || !bluetooth.isArmEnabled) {
      return 'HC-05 연결과 로봇팔 출력 활성화를 먼저 확인해주세요.';
    }

    if (sequenceId < 1 || sequenceId > sequenceCount) {
      return '잘못된 티칭 시퀀스 번호입니다.';
    }
    _playing = true;
    notifyListeners();
    try {
      final List<int>? finalPose = await _loadLastPose(bluetooth, sequenceId);
      if (finalPose == null) {
        return 'STM32에서 시퀀스의 마지막 자세를 읽지 못했습니다.';
      }
      final Future<bool> ack = bluetooth.waitForTeachingPlayAck(sequenceId);
      await bluetooth.sendCommand(
        2,
        data: [2, sequenceId, bluetooth.servoSpeedPercent],
      );
      final bool succeeded = await ack;
      if (succeeded) {
        bluetooth.rememberCommandedArmPose(finalPose);
      }
      return succeeded ? null : '시퀀스 재생 실패 또는 완료 ACK 시간 초과입니다.';
    } catch (error) {
      return '시퀀스 재생 전송 실패: $error';
    } finally {
      _playing = false;
      notifyListeners();
    }
  }

  /// Flash 이동 자세를 실행하고 현재 Gripper를 유지한 마지막 자세를 반영한다.
  Future<bool> prepareTravelSequencePreservingGripper(
    BluetoothController bluetooth,
  ) async {
    if (isBusy || !bluetooth.isConnected || !bluetooth.isArmEnabled) {
      return false;
    }
    _playing = true;
    notifyListeners();
    try {
      final List<int>? finalPose = await _loadLastPose(
        bluetooth,
        travelSequence,
      );
      if (finalPose == null) {
        return false;
      }
      final bool reached = await bluetooth
          .prepareTravelSequencePreservingGripper();
      if (reached) {
        bluetooth.rememberCommandedArmPose(finalPose, preserveGripper: true);
      }
      return reached;
    } catch (error) {
      debugPrint('이동 자세 조회 또는 실행 실패: $error');
      return false;
    } finally {
      _playing = false;
      notifyListeners();
    }
  }
}

/// 고정 시퀀스는 잠금 아이콘과 용도별 색으로 일반 시퀀스와 구분한다.
class _SequenceOptionLabel extends StatelessWidget {
  const _SequenceOptionLabel({required this.teaching, required this.sequence});

  final TeachingController teaching;
  final int sequence;

  Color? get _color => switch (sequence) {
    TeachingController.travelSequence => Colors.blue,
    TeachingController.pickupPoseSequence => Colors.amber.shade800,
    TeachingController.gripSequence => Colors.deepOrange,
    TeachingController.releaseSequence => Colors.green,
    _ => null,
  };

  @override
  Widget build(BuildContext context) {
    final Color? color = _color;
    return SizedBox(
      width: double.infinity,
      child: Row(
        children: [
          if (color != null) ...[
            Icon(Icons.lock, size: 15, color: color),
            const SizedBox(width: 5),
          ],
          Expanded(
            child: Text(
              teaching.sequenceName(sequence),
              overflow: TextOverflow.ellipsis,
              style: color == null
                  ? null
                  : TextStyle(color: color, fontWeight: FontWeight.w600),
            ),
          ),
        ],
      ),
    );
  }
}

/// 메인 화면과 설정 화면에서 같은 모양의 연결·안전 버튼을 사용한다.
Widget _buildAppBarAction({
  required String label,
  required String tooltip,
  required IconData icon,
  required VoidCallback? onPressed,
  required Color backgroundColor,
}) {
  return Padding(
    padding: const EdgeInsets.only(right: 4),
    child: Tooltip(
      message: tooltip,
      child: TextButton.icon(
        style: TextButton.styleFrom(
          foregroundColor: Colors.white,
          backgroundColor: backgroundColor,
          disabledForegroundColor: Colors.white38,
          disabledBackgroundColor: backgroundColor.withValues(alpha: 0.35),
          padding: const EdgeInsets.symmetric(horizontal: 7),
          minimumSize: const Size(0, 44),
          visualDensity: VisualDensity.compact,
          shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(8)),
        ),
        onPressed: onPressed,
        icon: Icon(icon, size: 18),
        label: Text(
          label,
          maxLines: 1,
          style: const TextStyle(fontSize: 11, fontWeight: FontWeight.bold),
        ),
      ),
    ),
  );
}

/// 연결부터 로봇팔 활성화까지 처음 필요한 세 단계를 짧게 안내한다.
class _SetupGuide extends StatelessWidget {
  const _SetupGuide({required this.controller});

  final BluetoothController controller;

  @override
  Widget build(BuildContext context) {
    final stages = <(String, bool)>[
      ('1  HC-05 연결', controller.isConnected),
      ('2  STOP 해제', !controller.isEstopLatched),
      ('3  로봇팔 켜기', controller.isArmEnabled),
    ];
    final String nextAction = !controller.isConnected
        ? '상단의 연결 버튼을 눌러 HC-05를 선택하세요.'
        : controller.isEstopLatched
        ? '주변이 안전한지 확인하고 상단의 STOP 해제를 누르세요.'
        : '로봇팔 화면에서 로봇팔 켜기 · 원점을 누르세요.';

    return Padding(
      padding: const EdgeInsets.fromLTRB(8, 8, 8, 2),
      child: Card(
        color: Theme.of(context).colorScheme.primaryContainer,
        child: Padding(
          padding: const EdgeInsets.all(10),
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              const Row(
                children: [
                  Icon(Icons.assistant_navigation, size: 20),
                  SizedBox(width: 7),
                  Text(
                    '처음 사용 순서',
                    style: TextStyle(fontWeight: FontWeight.bold),
                  ),
                ],
              ),
              const SizedBox(height: 8),
              Wrap(
                spacing: 6,
                runSpacing: 6,
                children: stages
                    .map(
                      (stage) => Chip(
                        avatar: Icon(
                          stage.$2 ? Icons.check_circle : Icons.circle_outlined,
                          size: 17,
                          color: stage.$2 ? Colors.green.shade700 : null,
                        ),
                        label: Text(stage.$1),
                        visualDensity: VisualDensity.compact,
                      ),
                    )
                    .toList(),
              ),
              const SizedBox(height: 6),
              Text(nextAction, style: const TextStyle(fontSize: 12)),
            ],
          ),
        ),
      ),
    );
  }
}

// ======================================================
// 공통 안전 바와 세 개의 주 화면
// ======================================================

/// RC카, 로봇팔, 티칭 화면과 항상 접근 가능한 안전 버튼을 배치한다.
class MainScreen extends StatefulWidget {
  const MainScreen({super.key});

  @override
  State<MainScreen> createState() => _MainScreenState();
}

class _MainScreenState extends State<MainScreen> with WidgetsBindingObserver {
  // 연결·E-STOP 해제·원점 활성화가 먼저 필요하므로 로봇팔 화면에서 시작한다.
  int _currentIndex = 1;
  bool _screenTransitionBusy = false;
  String? _scheduledAckMessage;
  Object? _syncedTeachingConnection;
  _TeachingPreviewSnapshot? _teachingPreviewSnapshot;
  List<int>? _armPosePreviewSnapshot;

  final List<Widget> _screens = [
    const JoystickTab(),
    const RobotArmControlTab(),
    const TeachingTab(),
  ];

  final List<String> _screenTitles = ['차량', '로봇팔', '티칭'];

  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance.addObserver(this);
  }

  @override
  void dispose() {
    WidgetsBinding.instance.removeObserver(this);
    super.dispose();
  }

  /// 앱이 화면을 벗어나면 펌웨어 타임아웃을 기다리지 않고 차량을 즉시 정지한다.
  @override
  void didChangeAppLifecycleState(AppLifecycleState state) {
    if (!mounted || state == AppLifecycleState.resumed) {
      return;
    }
    unawaited(context.read<BluetoothController>().stopDrive());
  }

  /// 넓은 화면에서 제어 요소가 과도하게 벌어지지 않도록 작업 영역을 중앙에 둔다.
  Widget _constrainedScreen(BluetoothController controller) {
    return Align(
      alignment: Alignment.topCenter,
      child: ConstrainedBox(
        constraints: const BoxConstraints(maxWidth: 1100),
        child: Column(
          children: [
            if (!controller.isConnected ||
                controller.isEstopLatched ||
                !controller.isArmEnabled)
              _SetupGuide(controller: controller),
            Expanded(child: _screens[_currentIndex]),
          ],
        ),
      ),
    );
  }

  /// 화면을 전환하며, 차량 진입 전에는 로봇팔의 이동 자세를 확인한다.
  Future<void> _selectScreen(int index, BluetoothController bluetooth) async {
    if (index == _currentIndex || _screenTransitionBusy) {
      return;
    }

    final TeachingController teaching = context.read<TeachingController>();
    setState(() => _screenTransitionBusy = true);
    try {
      if (index != 0) {
        setState(() => _currentIndex = index);
        await bluetooth.stopDrive();
        if (!mounted) {
          return;
        }
        unawaited(teaching.syncFromSTM32(bluetooth));
        return;
      }

      await bluetooth.stopDrive();
      if (!bluetooth.isArmEnabled || bluetooth.isEstopLatched) {
        if (mounted) {
          ScaffoldMessenger.of(context).showSnackBar(
            const SnackBar(content: Text('STOP 해제 후 로봇팔 원점을 활성화하세요.')),
          );
        }
        return;
      }

      /*
       * 잡기 위치는 펼친 자세를 유지한 채 주행할 수 있다. 물체를 잡은 상태라면
       * 이동 자세가 현재 그리퍼를 유지한 채 끝난 뒤 차량 화면을 연다.
       */
      if (bluetooth.isAtPickupPose && bluetooth.isDriveReady) {
        setState(() => _currentIndex = index);
        return;
      }

      if (bluetooth.isHoldingPayload) {
        bluetooth.beginPickupAction();
      }
      final bool ready = await teaching.prepareTravelSequencePreservingGripper(
        bluetooth,
      );
      if (bluetooth.isHoldingPayload) {
        bluetooth.finishPickupAction(
          state: PickupWorkflowState.holding,
          driveReady: ready,
        );
      }
      if (!mounted) {
        return;
      }
      if (!ready) {
        ScaffoldMessenger.of(context).showSnackBar(
          SnackBar(
            content: Text(bluetooth.lastAckMessage ?? '이동 자세 실행에 실패했습니다.'),
          ),
        );
        return;
      }
      setState(() => _currentIndex = index);
    } finally {
      if (mounted) {
        setState(() => _screenTransitionBusy = false);
      }
    }
  }

  /// E-STOP 해제 전에 주변 안전 상태를 사용자에게 다시 확인한다.
  Future<void> _confirmEstopClear(BluetoothController controller) async {
    final bool? confirmed = await showDialog<bool>(
      context: context,
      builder: (context) => AlertDialog(
        title: const Text('STOP 해제'),
        content: const Text('주변과 로봇 상태가 안전한가요?'),
        actions: [
          TextButton(
            onPressed: () => Navigator.pop(context, false),
            child: const Text('취소'),
          ),
          FilledButton(
            onPressed: () => Navigator.pop(context, true),
            child: const Text('해제'),
          ),
        ],
      ),
    );

    if (confirmed == true) {
      try {
        await controller.clearEmergencyStop();
      } catch (error) {
        if (mounted) {
          ScaffoldMessenger.of(
            context,
          ).showSnackBar(SnackBar(content: Text('STOP 해제 실패: $error')));
        }
      }
    }
  }

  /// Android에 페어링된 HC-05 목록을 보여주고 선택한 장치에 연결한다.
  void _showBluetoothDialog(BuildContext context) {
    final bltController = Provider.of<BluetoothController>(
      context,
      listen: false,
    );
    unawaited(bltController.loadBondedDevices());

    showDialog(
      context: context,
      builder: (context) {
        return AlertDialog(
          title: const Text('블루투스 기기 연결 (HC-05)'),
          content: SizedBox(
            width: double.maxFinite,
            height: 300,
            child: Consumer<BluetoothController>(
              builder: (context, controller, child) {
                if (controller.isLoadingBondedDevices) {
                  return const Center(child: CircularProgressIndicator());
                }
                if (controller.connectionErrorMessage != null) {
                  return Center(
                    child: Text(
                      controller.connectionErrorMessage!,
                      textAlign: TextAlign.center,
                      style: const TextStyle(color: Colors.red),
                    ),
                  );
                }
                if (controller.bondedDevices.isEmpty) {
                  return const Center(
                    child: Text(
                      'HC-05를 먼저 페어링하세요.',
                      textAlign: TextAlign.center,
                    ),
                  );
                }
                return ListView.builder(
                  itemCount: controller.bondedDevices.length,
                  itemBuilder: (context, index) {
                    final device = controller.bondedDevices[index];
                    return ListTile(
                      title: Text(
                        device.name ?? '알 수 없는 기기',
                        style: const TextStyle(fontWeight: FontWeight.bold),
                      ),
                      subtitle: Text(device.address),
                      trailing: ElevatedButton(
                        onPressed: controller.isConnecting
                            ? null
                            : () async {
                                await controller.connectToDevice(device);
                                if (!context.mounted) {
                                  return;
                                }
                                if (controller.isConnected) {
                                  Navigator.pop(context);
                                }
                              },
                        child: controller.isConnecting
                            ? const SizedBox(
                                width: 16,
                                height: 16,
                                child: CircularProgressIndicator(
                                  strokeWidth: 2,
                                ),
                              )
                            : const Text('연결'),
                      ),
                    );
                  },
                );
              },
            ),
          ),
          actions: [
            TextButton(
              onPressed: () => Navigator.pop(context),
              child: const Text('닫기'),
            ),
          ],
        );
      },
    );
  }

  /// 제목을 길게 눌렀을 때 실제 통신 없는 기능 확인 모드를 안전하게 전환한다.
  Future<void> _toggleFeaturePreviewMode(
    BluetoothController bluetooth,
    TeachingController teaching,
  ) async {
    if (bluetooth.isPhysicallyConnected) {
      ScaffoldMessenger.of(
        context,
      ).showSnackBar(const SnackBar(content: Text('HC-05 연결을 먼저 해제하세요.')));
      return;
    }
    if (teaching.isBusy ||
        bluetooth.armCommandBusy ||
        bluetooth.settingsBusy ||
        _screenTransitionBusy) {
      ScaffoldMessenger.of(
        context,
      ).showSnackBar(const SnackBar(content: Text('작업 완료 후 전환하세요.')));
      return;
    }

    final bool enabling = !bluetooth.isFeaturePreviewMode;
    final bool? confirmed = await showDialog<bool>(
      context: context,
      builder: (dialogContext) => AlertDialog(
        title: Text(enabling ? 'TEST 모드' : 'TEST 종료'),
        content: Text(
          enabling
              ? '장치에 연결하지 않고 UI를 확인합니다.\n변경값은 종료 시 복원됩니다.'
              : 'TEST 변경값을 버리고 돌아갈까요?',
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.pop(dialogContext, false),
            child: const Text('취소'),
          ),
          FilledButton(
            onPressed: () => Navigator.pop(dialogContext, true),
            child: Text(enabling ? '시작' : '종료'),
          ),
        ],
      ),
    );
    if (confirmed != true || !mounted) {
      return;
    }

    if (enabling) {
      _teachingPreviewSnapshot = teaching._createFeaturePreviewSnapshot();
      _armPosePreviewSnapshot = List<int>.from(
        RobotArmControlTab.currentAngles,
      );
      bluetooth.enableFeaturePreviewMode();
      return;
    }

    bluetooth.disableFeaturePreviewMode();
    final teachingSnapshot = _teachingPreviewSnapshot;
    if (teachingSnapshot != null) {
      teaching._restoreFeaturePreviewSnapshot(teachingSnapshot);
    }
    final armSnapshot = _armPosePreviewSnapshot;
    if (armSnapshot != null) {
      setState(() {
        RobotArmControlTab.currentAngles = List<int>.from(armSnapshot);
      });
    }
    _teachingPreviewSnapshot = null;
    _armPosePreviewSnapshot = null;
  }

  /// 진행 중 작업을 확인한 뒤 세로 NavigationBar와 가로 Rail의 요청을 처리한다.
  void _requestScreenChange(
    int index,
    BluetoothController bluetooth,
    TeachingController teaching,
  ) {
    if (teaching.isBusy ||
        bluetooth.armCommandBusy ||
        bluetooth.settingsBusy ||
        _screenTransitionBusy) {
      showAppMessage(
        context,
        teaching.isBusy
            ? '티칭 작업이 끝난 뒤 화면을 이동할 수 있습니다.'
            : bluetooth.armCommandBusy
            ? '로봇팔 이동이 끝난 뒤 화면을 이동할 수 있습니다.'
            : bluetooth.settingsBusy
            ? '설정 작업이 끝난 뒤 화면을 이동할 수 있습니다.'
            : '현재 화면 전환이 끝날 때까지 기다려주세요.',
      );
      return;
    }
    unawaited(_selectScreen(index, bluetooth));
  }

  @override
  Widget build(BuildContext context) {
    final bltController = Provider.of<BluetoothController>(context);
    final teachingController = Provider.of<TeachingController>(context);
    final bool isConnected = bltController.isConnected;
    final bool isLandscape =
        MediaQuery.orientationOf(context) == Orientation.landscape;

    final Object? activeConnection = bltController.connection;
    if (_currentIndex != 0 &&
        activeConnection != null &&
        !identical(_syncedTeachingConnection, activeConnection)) {
      _syncedTeachingConnection = activeConnection;
      WidgetsBinding.instance.addPostFrameCallback((_) {
        if (mounted) {
          unawaited(teachingController.syncFromSTM32(bltController));
        }
      });
    } else if (activeConnection == null) {
      _syncedTeachingConnection = null;
    }

    final String? ackMessage = bltController.lastAckMessage;
    final bool ackSucceeded = bltController.lastAckSucceeded;
    final bool ackIsWarning = bltController.lastAckIsWarning;
    if (ackMessage != null && ackMessage != _scheduledAckMessage) {
      _scheduledAckMessage = ackMessage;
      WidgetsBinding.instance.addPostFrameCallback((_) {
        if (!mounted) {
          return;
        }
        /*
         * 호출 화면이 더 구체적인 결과를 이미 표시했다면 오래된 ACK를 다시
         * 띄우지 않는다. 같은 메시지가 여러 build에서 예약되는 것도 막는다.
         */
        if (bltController.lastAckMessage == ackMessage) {
          showAppMessage(
            context,
            ackMessage,
            backgroundColor: ackIsWarning
                ? Colors.orange.shade800
                : ackSucceeded
                ? Colors.green
                : Colors.red,
            duration: Duration(
              milliseconds: ackIsWarning ? 4000 : (ackSucceeded ? 1000 : 2200),
            ),
          );
          bltController.clearAckMessage();
        }
        if (_scheduledAckMessage == ackMessage) {
          _scheduledAckMessage = null;
        }
      });
    }

    final bool operationBusy =
        teachingController.isBusy ||
        bltController.armCommandBusy ||
        bltController.settingsBusy ||
        _screenTransitionBusy;

    return PopScope(
      canPop:
          !teachingController.isBusy &&
          !bltController.armCommandBusy &&
          !_screenTransitionBusy,
      child: Scaffold(
        appBar: AppBar(
          title: GestureDetector(
            behavior: HitTestBehavior.opaque,
            onLongPress: () {
              unawaited(
                _toggleFeaturePreviewMode(bltController, teachingController),
              );
            },
            child: Text(
              bltController.isFeaturePreviewMode
                  ? 'TEST · ${_screenTitles[_currentIndex]}'
                  : _screenTitles[_currentIndex],
              maxLines: 1,
              overflow: TextOverflow.ellipsis,
              style: const TextStyle(fontWeight: FontWeight.bold, fontSize: 18),
            ),
          ),
          actions: [
            _buildAppBarAction(
              label: bltController.isFeaturePreviewMode
                  ? '테스트'
                  : isConnected
                  ? '연결됨'
                  : '연결',
              tooltip: bltController.isFeaturePreviewMode
                  ? '제목을 길게 눌러 기능 확인 모드 종료'
                  : isConnected
                  ? 'HC-05 연결 해제'
                  : 'HC-05 연결',
              icon: bltController.isFeaturePreviewMode
                  ? Icons.science_outlined
                  : isConnected
                  ? Icons.bluetooth_connected
                  : Icons.bluetooth_searching,
              backgroundColor: bltController.isFeaturePreviewMode
                  ? Colors.deepPurple.shade600
                  : isConnected
                  ? Colors.teal.shade700
                  : Colors.blueGrey.shade600,
              onPressed: () {
                if (bltController.isFeaturePreviewMode) {
                  ScaffoldMessenger.of(context).showSnackBar(
                    const SnackBar(
                      content: Text('상단 제목을 길게 눌러 테스트를 종료할 수 있습니다.'),
                    ),
                  );
                } else if (isConnected) {
                  unawaited(bltController.disconnect());
                } else {
                  _showBluetoothDialog(context);
                }
              },
            ),
            if (bltController.isEstopLatched)
              _buildAppBarAction(
                label: 'STOP 해제',
                tooltip: 'STOP 해제',
                icon: Icons.lock_open,
                backgroundColor: Colors.orange.shade800,
                onPressed: isConnected
                    ? () => _confirmEstopClear(bltController)
                    : null,
              ),
            _buildAppBarAction(
              label: 'STOP',
              tooltip: '차량과 로봇팔 긴급 정지',
              icon: Icons.warning_rounded,
              backgroundColor: Colors.red.shade700,
              onPressed: () {
                unawaited(bltController.emergencyStop());
              },
            ),
          ],
        ),
        bottomNavigationBar: isLandscape
            ? null
            : NavigationBar(
                selectedIndex: _currentIndex,
                height: 68,
                onDestinationSelected: (index) {
                  _requestScreenChange(
                    index,
                    bltController,
                    teachingController,
                  );
                },
                destinations: const [
                  NavigationDestination(
                    icon: Icon(Icons.directions_car_outlined),
                    selectedIcon: Icon(Icons.directions_car),
                    label: '차량',
                  ),
                  NavigationDestination(
                    icon: Icon(Icons.precision_manufacturing_outlined),
                    selectedIcon: Icon(Icons.precision_manufacturing),
                    label: '로봇팔',
                  ),
                  NavigationDestination(
                    icon: Icon(Icons.bookmark_add_outlined),
                    selectedIcon: Icon(Icons.bookmark_added),
                    label: '티칭',
                  ),
                ],
              ),
        body: Stack(
          children: [
            Positioned.fill(
              child: isLandscape
                  ? Row(
                      children: [
                        NavigationRail(
                          selectedIndex: _currentIndex,
                          labelType: NavigationRailLabelType.all,
                          onDestinationSelected: (index) =>
                              _requestScreenChange(
                                index,
                                bltController,
                                teachingController,
                              ),
                          destinations: const [
                            NavigationRailDestination(
                              icon: Icon(Icons.directions_car_outlined),
                              selectedIcon: Icon(Icons.directions_car),
                              label: Text('차량'),
                            ),
                            NavigationRailDestination(
                              icon: Icon(
                                Icons.precision_manufacturing_outlined,
                              ),
                              selectedIcon: Icon(Icons.precision_manufacturing),
                              label: Text('로봇팔'),
                            ),
                            NavigationRailDestination(
                              icon: Icon(Icons.bookmark_add_outlined),
                              selectedIcon: Icon(Icons.bookmark_added),
                              label: Text('티칭'),
                            ),
                          ],
                        ),
                        const VerticalDivider(width: 1),
                        Expanded(child: _constrainedScreen(bltController)),
                      ],
                    )
                  : _constrainedScreen(bltController),
            ),
            if (operationBusy)
              const Positioned(
                left: 0,
                right: 0,
                top: 0,
                child: LinearProgressIndicator(minHeight: 3),
              ),
          ],
        ),
      ),
    );
  }
}

// ======================================================
// RC카 주행 화면
// ======================================================

/// 한 손 조이스틱으로 좌우 모터의 방향과 속도를 만드는 화면이다.
class JoystickTab extends StatefulWidget {
  const JoystickTab({super.key});

  @override
  State<JoystickTab> createState() => _JoystickTabState();
}

class _JoystickTabState extends State<JoystickTab> {
  static const int _maxDrivePwm = 220;
  Timer? _heartbeatTimer;
  BluetoothController? _controller;
  int _leftDirection = 0;
  int _leftPwm = 0;
  int _rightDirection = 0;
  int _rightPwm = 0;
  bool _straightPidInput = false;

  @override
  /// Controller가 바뀌면 안전 상태 감시를 연결하고 100 ms 주행 갱신을 시작한다.
  void didChangeDependencies() {
    super.didChangeDependencies();
    final BluetoothController controller = Provider.of<BluetoothController>(
      context,
      listen: false,
    );
    if (!identical(_controller, controller)) {
      _controller?.removeListener(_handleControllerState);
      _controller = controller;
      controller.addListener(_handleControllerState);
    }
    _heartbeatTimer ??= Timer.periodic(
      const Duration(milliseconds: 100),
      (_) => _sendCurrentCommand(),
    );
  }

  /// 주행이 잠기면 이전 값을 지워 재활성화 순간의 급출발을 막는다.
  void _handleControllerState() {
    final BluetoothController? controller = _controller;
    if (controller == null ||
        (controller.isHoldingPayload && !controller.isEstopLatched) ||
        (controller.isDriveReady &&
            !controller.isEstopLatched &&
            controller.isConnected &&
            !controller.settingsBusy) ||
        (_leftPwm == 0 && _rightPwm == 0)) {
      return;
    }
    _straightPidInput = false;
    if (mounted) {
      setState(() {
        _leftDirection = 0;
        _leftPwm = 0;
        _rightDirection = 0;
        _rightPwm = 0;
      });
    }
  }

  /// 현재 좌우 모터 값을 통신 Controller에 전달한다.
  void _sendCurrentCommand() {
    final BluetoothController? controller = _controller;
    if (controller == null) {
      return;
    }

    unawaited(
      controller.sendDrive(
        _leftDirection,
        _leftPwm,
        _rightDirection,
        _rightPwm,
      ),
    );
  }

  /// 조이스틱의 X/Y 값을 좌우 모터의 차동 조향 값으로 변환한다.
  void _updateJoystick(double x, double y) {
    final double forward = -y;
    final BluetoothController controller = Provider.of<BluetoothController>(
      context,
      listen: false,
    );
    final double centerThreshold = _straightPidInput
        ? (controller.joystickThreshold + 0.04).clamp(0.0, 1.0).toDouble()
        : controller.joystickThreshold;
    _straightPidInput =
        controller.pidApplied &&
        forward.abs() > 0.05 &&
        x.abs() <= centerThreshold;
    final double steering = _straightPidInput ? 0 : x;
    double leftSpeed = (forward < 0)
        ? (forward - steering)
        : (forward + steering);
    double rightSpeed = (forward < 0)
        ? (forward + steering)
        : (forward - steering);

    leftSpeed = leftSpeed.clamp(-1.0, 1.0).toDouble();
    rightSpeed = rightSpeed.clamp(-1.0, 1.0).toDouble();

    setState(() {
      _leftDirection = leftSpeed >= 0 ? 0 : 1;
      _leftPwm = (leftSpeed.abs() * _maxDrivePwm).round();
      _rightDirection = rightSpeed >= 0 ? 0 : 1;
      _rightPwm = (rightSpeed.abs() * _maxDrivePwm).round();
    });
    if (controller.isHoldingPayload && !controller.isDriveReady) {
      if ((_leftPwm > 0 || _rightPwm > 0) &&
          !controller.pickupActionBusy &&
          !controller.armCommandBusy) {
        unawaited(_prepareHeldPayloadForDrive(controller));
      }
      return;
    }
    _sendCurrentCommand();
  }

  /// 모터 PWM과 방향을 사용자가 읽을 수 있는 상태 문자열로 바꾼다.
  String _motorState(int direction, int pwm) {
    if (pwm == 0) {
      return '정지';
    }
    return direction == 0 ? '전진' : '후진';
  }

  /// Flash에 저장된 1000배 정수 PID 값을 소수점 셋째 자리로 표시한다.
  String _formatPid(int milli) => (milli / 1000).toStringAsFixed(3);

  /// 설정 전 조이스틱 상태까지 지워 주기 전의 주행 명령이 다시 나가지 않게 한다.
  void _clearLocalDriveInput() {
    _straightPidInput = false;
    if (!mounted) {
      return;
    }
    setState(() {
      _leftDirection = 0;
      _leftPwm = 0;
      _rightDirection = 0;
      _rightPwm = 0;
    });
  }

  /// 물체를 잡은 뒤 첫 주행 입력에서 그리퍼를 유지한 이동 자세를 먼저 실행한다.
  Future<void> _prepareHeldPayloadForDrive(
    BluetoothController controller,
  ) async {
    if (!controller.isHoldingPayload ||
        controller.pickupActionBusy ||
        controller.isDriveReady) {
      return;
    }

    final TeachingController teaching = context.read<TeachingController>();
    controller.beginPickupAction();
    await controller.stopDrive();
    final bool ready = await teaching.prepareTravelSequencePreservingGripper(
      controller,
    );
    if (!mounted) {
      return;
    }
    controller.finishPickupAction(
      state: PickupWorkflowState.holding,
      driveReady: ready,
    );
    if (!ready) {
      _clearLocalDriveInput();
      ScaffoldMessenger.of(context).showSnackBar(
        const SnackBar(content: Text('이동 자세 실행에 실패해 차량 주행을 잠갔습니다.')),
      );
    }
  }

  /// 전용 티칭 시퀀스 전 차량을 정지하고 선행조건과 완료 결과를 상태로 반영한다.
  Future<void> _runPickupSequence(
    BluetoothController controller,
    TeachingController teaching,
    int sequenceId,
  ) async {
    if (controller.pickupActionBusy ||
        teaching.isBusy ||
        controller.armCommandBusy) {
      return;
    }

    final PickupWorkflowState previousState = controller.pickupWorkflowState;
    _clearLocalDriveInput();
    await controller.stopDrive();
    if (!mounted) {
      return;
    }
    controller.beginPickupAction();

    final String? error = await teaching.playSequenceOnSTM32(
      controller,
      sequenceId,
    );
    if (!mounted) {
      return;
    }
    if (error != null) {
      controller.finishPickupAction(state: previousState, driveReady: false);
      ScaffoldMessenger.of(
        context,
      ).showSnackBar(SnackBar(content: Text(error)));
      return;
    }

    if (sequenceId == TeachingController.pickupPoseSequence) {
      controller.finishPickupAction(
        state: PickupWorkflowState.pickupPoseReady,
        driveReady: true,
      );
      return;
    }

    if (sequenceId == TeachingController.gripSequence) {
      controller.finishPickupAction(
        state: PickupWorkflowState.holding,
        driveReady: false,
      );
      return;
    }

    // 놓기 완료 후에는 열린 그리퍼 값을 유지한 채 이동 자세를 자동 실행한다.
    final bool ready = await teaching.prepareTravelSequencePreservingGripper(
      controller,
    );
    if (!mounted) {
      return;
    }
    controller.finishPickupAction(
      state: PickupWorkflowState.idle,
      driveReady: ready,
    );
    if (!ready) {
      ScaffoldMessenger.of(
        context,
      ).showSnackBar(const SnackBar(content: Text('놓기 완료 · 이동 자세 실행 실패')));
    }
  }

  /// PID 전환 전에 차량 정지와 시험 공간을 확인하고 펌웨어의 실제 ACK를 기다린다.
  Future<void> _setPidEnabled(
    BluetoothController controller,
    bool enabled,
  ) async {
    _clearLocalDriveInput();
    await controller.stopDrive();
    if (!mounted) {
      return;
    }

    if (enabled) {
      final bool? confirmed = await showDialog<bool>(
        context: context,
        builder: (dialogContext) => AlertDialog(
          title: const Text('PID 켜기'),
          content: Text(
            'P ${_formatPid(controller.pidKpMilli)} · '
            'I ${_formatPid(controller.pidKiMilli)} · '
            'D ${_formatPid(controller.pidKdMilli)} · '
            '직진 범위 ${controller.joystickThreshold.toStringAsFixed(2)}\n'
            '차량을 띄우거나 넓은 곳에서 시험하세요.',
          ),
          actions: [
            TextButton(
              onPressed: () => Navigator.pop(dialogContext, false),
              child: const Text('취소'),
            ),
            FilledButton(
              onPressed: () => Navigator.pop(dialogContext, true),
              child: const Text('켜기'),
            ),
          ],
        ),
      );
      if (confirmed != true || !mounted) {
        return;
      }
    }

    await controller.setPidEnabled(enabled);
  }

  /// STM32 설정을 읽고 PID 값을 수정·저장하는 팝업을 연다.
  Future<void> _showPidSettings(BluetoothController controller) async {
    if (!controller.isConnected) {
      ScaffoldMessenger.of(
        context,
      ).showSnackBar(const SnackBar(content: Text('먼저 HC-05에 연결해주세요.')));
      return;
    }

    _clearLocalDriveInput();
    await controller.stopDrive();
    if (!mounted) {
      return;
    }

    if (!controller.settingsLoaded) {
      final bool loaded = await controller.requestSettings();
      if (!mounted) {
        return;
      }
      if (!loaded) {
        ScaffoldMessenger.of(
          context,
        ).showSnackBar(const SnackBar(content: Text('STM32 설정을 읽지 못했습니다.')));
        return;
      }
    }

    final controllers = <TextEditingController>[
      TextEditingController(text: _formatPid(controller.pidKpMilli)),
      TextEditingController(text: _formatPid(controller.pidKiMilli)),
      TextEditingController(text: _formatPid(controller.pidKdMilli)),
    ];
    final thresholdController = TextEditingController(
      text: controller.joystickThreshold.toStringAsFixed(2),
    );

    final ({List<int> pid, double threshold})?
    values = await showDialog<({List<int> pid, double threshold})>(
      context: context,
      builder: (dialogContext) {
        String? errorText;
        return StatefulBuilder(
          builder: (context, setDialogState) => AlertDialog(
            title: const Text('PID·조이스틱 설정'),
            content: SingleChildScrollView(
              child: Column(
                mainAxisSize: MainAxisSize.min,
                children: [
                  Text(
                    controller.isFeaturePreviewMode
                        ? 'TEST 모드 · 앱에만 저장'
                        : 'Flash 저장 후 PID를 다시 켜세요.',
                  ),
                  const SizedBox(height: 12),
                  ...List.generate(3, (index) {
                    const labels = ['P (Kp)', 'I (Ki)', 'D (Kd)'];
                    return Padding(
                      padding: const EdgeInsets.only(bottom: 10),
                      child: TextField(
                        controller: controllers[index],
                        keyboardType: const TextInputType.numberWithOptions(
                          decimal: true,
                          signed: false,
                        ),
                        textInputAction: index == 2
                            ? TextInputAction.done
                            : TextInputAction.next,
                        inputFormatters: [
                          FilteringTextInputFormatter.allow(
                            RegExp(
                              index == 0
                                  ? r'^\d{0,1}(\.\d{0,2})?$'
                                  : r'^\d{0,3}(\.\d{0,3})?$',
                            ),
                          ),
                        ],
                        decoration: InputDecoration(
                          labelText: labels[index],
                          helperText: index == 0
                              ? '0.00~2.55 · 0.01 단위'
                              : '0.000~100.000',
                        ),
                      ),
                    );
                  }),
                  TextField(
                    controller: thresholdController,
                    keyboardType: const TextInputType.numberWithOptions(
                      decimal: true,
                      signed: false,
                    ),
                    inputFormatters: [
                      FilteringTextInputFormatter.allow(
                        RegExp(r'^\d{0,1}(\.\d{0,2})?$'),
                      ),
                    ],
                    decoration: const InputDecoration(
                      labelText: '조이스틱 직진 판정 범위',
                      helperText: '|X|가 이 값 이하면 PID 직진으로 판단 · 0.00~1.00',
                    ),
                  ),
                  if (errorText != null)
                    Text(errorText!, style: const TextStyle(color: Colors.red)),
                ],
              ),
            ),
            actions: [
              TextButton(
                onPressed: () => Navigator.pop(dialogContext),
                child: const Text('취소'),
              ),
              FilledButton(
                onPressed: () {
                  final parsed = <int>[];
                  for (int index = 0; index < controllers.length; index++) {
                    final field = controllers[index];
                    final double? value = double.tryParse(field.text.trim());
                    if (index == 0) {
                      /*
                       * 참조 앱처럼 Kp는 ×100으로 양자화한다. 현재 펌웨어의
                       * 호환 가능한 ×1000 저장 형식에는 centi 값 ×10으로 넣는다.
                       */
                      final double scaledCenti = (value ?? double.nan) * 100;
                      if (!scaledCenti.isFinite ||
                          scaledCenti < 0 ||
                          scaledCenti > 255) {
                        setDialogState(() {
                          errorText = 'Kp는 0~2.55 범위로 입력해주세요.';
                        });
                        return;
                      }
                      parsed.add(scaledCenti.round() * 10);
                      continue;
                    }

                    final double scaledMilli = (value ?? double.nan) * 1000;
                    if (!scaledMilli.isFinite ||
                        scaledMilli < 0 ||
                        scaledMilli > 100000) {
                      setDialogState(() {
                        errorText = 'Ki와 Kd는 각각 0~100 범위로 입력해주세요.';
                      });
                      return;
                    }
                    parsed.add(scaledMilli.round());
                  }
                  if (parsed.every((value) => value == 0)) {
                    setDialogState(() {
                      errorText = 'P, I, D를 모두 0으로 저장할 수는 없습니다.';
                    });
                    return;
                  }
                  final double? threshold = double.tryParse(
                    thresholdController.text.trim(),
                  );
                  if (threshold == null ||
                      !threshold.isFinite ||
                      threshold < 0 ||
                      threshold > 1) {
                    setDialogState(() {
                      errorText = '조이스틱 직진 판정 범위는 0.00~1.00으로 입력해주세요.';
                    });
                    return;
                  }
                  Navigator.pop(dialogContext, (
                    pid: parsed,
                    threshold: threshold,
                  ));
                },
                child: const Text('저장'),
              ),
            ],
          ),
        );
      },
    );
    for (final field in controllers) {
      field.dispose();
    }
    thresholdController.dispose();

    if (values == null || !mounted) {
      return;
    }
    controller.setJoystickThreshold(values.threshold);

    /*
     * 실행 중인 제어기의 계수를 중간에 바꾸지 않는다.
     * PID를 끄고 저장한 뒤 사용자가 새 값으로 다시 활성화하게 한다.
     */
    if (controller.pidApplied) {
      final bool disabled = await controller.setPidEnabled(false);
      if (!disabled || !mounted) {
        if (mounted) {
          ScaffoldMessenger.of(context).showSnackBar(
            const SnackBar(content: Text('PID를 비활성화하지 못해 계수 저장을 취소했습니다.')),
          );
        }
        return;
      }
    }

    // PID 저장은 중요 설정 53바이트 전체를 함께 기록하므로, 오래된 서보
    // 보정값과 이동 자세를 덮어쓰지 않도록 저장 직전에 Flash 설정을 다시 읽는다.
    final bool refreshed = await controller.requestSettings();
    if (!refreshed || !mounted) {
      if (mounted) {
        ScaffoldMessenger.of(context).showSnackBar(
          const SnackBar(content: Text('최신 STM32 설정을 확인하지 못해 저장을 취소했습니다.')),
        );
      }
      return;
    }
    final bool saved = await controller.saveSettings(
      kpMilli: values.pid[0],
      kiMilli: values.pid[1],
      kdMilli: values.pid[2],
      servoPulsesUs: controller.servoCalibrationUs,
      travelPose: controller.travelPoseAngles,
    );
    if (mounted) {
      controller.clearAckMessage();
      showAppMessage(
        context,
        saved
            ? controller.isFeaturePreviewMode
                  ? 'PID와 조이스틱 범위를 기능 확인용 메모리에 저장했습니다.'
                  : 'PID는 Flash, 조이스틱 범위는 앱에 저장했습니다.'
            : 'PID 값 저장에 실패했습니다.',
        backgroundColor: saved ? Colors.green : Colors.red,
      );
    }
  }

  @override
  void dispose() {
    _heartbeatTimer?.cancel();
    final BluetoothController? controller = _controller;
    controller?.removeListener(_handleControllerState);
    if ((controller != null) && !controller.isEstopLatched) {
      unawaited(controller.stopDrive());
    }
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final controller = Provider.of<BluetoothController>(context);
    final teaching = Provider.of<TeachingController>(context);
    final bool driveControlsEnabled =
        controller.isConnected &&
        !controller.isEstopLatched &&
        controller.isDriveReady &&
        !controller.settingsBusy;
    final bool canPrepareHeldPayload =
        controller.isConnected &&
        !controller.isEstopLatched &&
        controller.isHoldingPayload &&
        !controller.pickupActionBusy &&
        !teaching.isBusy;
    final bool joystickEnabled = driveControlsEnabled || canPrepareHeldPayload;
    final bool pickupCommonEnabled =
        controller.isConnected &&
        controller.isArmEnabled &&
        !controller.isEstopLatched &&
        !controller.pickupActionBusy &&
        !controller.armCommandBusy &&
        !teaching.isBusy;

    return SingleChildScrollView(
      physics: const NeverScrollableScrollPhysics(),
      padding: const EdgeInsets.fromLTRB(12, 8, 12, 20),
      child: Column(
        children: [
          if (!controller.isDriveReady) ...[
            Card(
              color: Colors.orange.shade50,
              child: Padding(
                padding: const EdgeInsets.all(12),
                child: Row(
                  children: [
                    if (controller.isConnected &&
                        !controller.isEstopLatched &&
                        controller.isArmEnabled)
                      const SizedBox(
                        width: 20,
                        height: 20,
                        child: CircularProgressIndicator(strokeWidth: 2),
                      )
                    else
                      Icon(
                        !controller.isConnected
                            ? Icons.bluetooth_disabled
                            : controller.isEstopLatched
                            ? Icons.lock_outline
                            : Icons.precision_manufacturing_outlined,
                        color: Colors.orange.shade800,
                      ),
                    const SizedBox(width: 12),
                    Expanded(
                      child: Column(
                        crossAxisAlignment: CrossAxisAlignment.start,
                        children: [
                          Text(
                            !controller.isConnected
                                ? '차량 조작 전 연결이 필요합니다'
                                : controller.isEstopLatched
                                ? '안전 잠금 상태입니다'
                                : controller.isArmEnabled
                                ? '이동 자세로 이동 중입니다'
                                : '로봇팔을 먼저 켜세요',
                            style: const TextStyle(fontWeight: FontWeight.bold),
                          ),
                          const SizedBox(height: 2),
                          Text(
                            !controller.isConnected
                                ? '상단 연결 버튼에서 HC-05를 선택하세요.'
                                : controller.isEstopLatched
                                ? '주변을 확인한 뒤 상단의 STOP 해제를 누르세요.'
                                : controller.isArmEnabled
                                ? '로봇팔이 도착하면 조이스틱이 활성화됩니다.'
                                : '로봇팔 화면에서 로봇팔 켜기 · 원점을 누르세요.',
                            style: const TextStyle(
                              color: Colors.blueGrey,
                              fontSize: 12,
                            ),
                          ),
                        ],
                      ),
                    ),
                  ],
                ),
              ),
            ),
          ],
          const SizedBox(height: 10),
          Card(
            child: Padding(
              padding: const EdgeInsets.all(12),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.stretch,
                children: [
                  Row(
                    children: [
                      const Icon(Icons.inventory_2_outlined),
                      const SizedBox(width: 8),
                      const Expanded(
                        child: Text(
                          '물체 잡기',
                          style: TextStyle(fontWeight: FontWeight.bold),
                        ),
                      ),
                      if (controller.pickupActionBusy || teaching.isPlaying)
                        const SizedBox(
                          width: 18,
                          height: 18,
                          child: CircularProgressIndicator(strokeWidth: 2),
                        ),
                    ],
                  ),
                  const SizedBox(height: 6),
                  Text(
                    controller.pickupActionBusy
                        ? '로봇팔 동작 중'
                        : controller.isHoldingPayload
                        ? controller.isDriveReady
                              ? '잡기 완료 · 주행 가능'
                              : '잡기 완료 · 이동 자세 실행 필요'
                        : controller.isAtPickupPose
                        ? '잡기 위치 도착'
                        : '잡기 위치로 이동하세요.',
                    style: const TextStyle(
                      color: Colors.blueGrey,
                      fontSize: 12,
                    ),
                  ),
                  const SizedBox(height: 10),
                  Row(
                    children: [
                      Expanded(
                        child: OutlinedButton.icon(
                          onPressed:
                              pickupCommonEnabled &&
                                  controller.pickupWorkflowState ==
                                      PickupWorkflowState.idle
                              ? () => unawaited(
                                  _runPickupSequence(
                                    controller,
                                    teaching,
                                    TeachingController.pickupPoseSequence,
                                  ),
                                )
                              : null,
                          icon: const Icon(Icons.looks_one_outlined),
                          label: const Text('위치로'),
                        ),
                      ),
                      const SizedBox(width: 6),
                      Expanded(
                        child: FilledButton.tonalIcon(
                          onPressed:
                              pickupCommonEnabled && controller.isAtPickupPose
                              ? () => unawaited(
                                  _runPickupSequence(
                                    controller,
                                    teaching,
                                    TeachingController.gripSequence,
                                  ),
                                )
                              : null,
                          icon: const Icon(Icons.looks_two_outlined),
                          label: const Text('잡기'),
                        ),
                      ),
                      const SizedBox(width: 6),
                      Expanded(
                        child: FilledButton.tonalIcon(
                          onPressed:
                              pickupCommonEnabled && controller.isHoldingPayload
                              ? () => unawaited(
                                  _runPickupSequence(
                                    controller,
                                    teaching,
                                    TeachingController.releaseSequence,
                                  ),
                                )
                              : null,
                          icon: const Icon(Icons.looks_3_outlined),
                          label: const Text('놓기'),
                        ),
                      ),
                    ],
                  ),
                ],
              ),
            ),
          ),
          const SizedBox(height: 18),
          const Align(
            alignment: Alignment.centerLeft,
            child: Text(
              '차량 운전',
              style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
            ),
          ),
          const SizedBox(height: 2),
          Text(
            joystickEnabled
                ? '원을 드래그해 전후좌우로 이동하세요. 손을 떼면 정지합니다.'
                : '위의 안전 준비가 완료되면 조이스틱이 활성화됩니다.',
            style: const TextStyle(color: Colors.blueGrey, fontSize: 12),
          ),
          const SizedBox(height: 8),
          SizedBox(
            height: 240,
            child: Center(
              child: AbsorbPointer(
                absorbing: !joystickEnabled,
                child: Opacity(
                  opacity: joystickEnabled ? 1 : 0.35,
                  child: Joystick(
                    mode: JoystickMode.all,
                    listener: (details) {
                      _updateJoystick(details.x, details.y);
                    },
                  ),
                ),
              ),
            ),
          ),
          const SizedBox(height: 12),
          Row(
            children: [
              Expanded(
                child: Card(
                  child: Padding(
                    padding: const EdgeInsets.symmetric(
                      horizontal: 12,
                      vertical: 10,
                    ),
                    child: Column(
                      children: [
                        const Text(
                          '왼쪽',
                          style: TextStyle(fontWeight: FontWeight.bold),
                        ),
                        const SizedBox(height: 4),
                        Text(
                          '${_motorState(_leftDirection, _leftPwm)} · $_leftPwm',
                        ),
                      ],
                    ),
                  ),
                ),
              ),
              const SizedBox(width: 8),
              Expanded(
                child: Card(
                  child: Padding(
                    padding: const EdgeInsets.symmetric(
                      horizontal: 12,
                      vertical: 10,
                    ),
                    child: Column(
                      children: [
                        const Text(
                          '오른쪽',
                          style: TextStyle(fontWeight: FontWeight.bold),
                        ),
                        const SizedBox(height: 4),
                        Text(
                          '${_motorState(_rightDirection, _rightPwm)} · $_rightPwm',
                        ),
                      ],
                    ),
                  ),
                ),
              ),
            ],
          ),
          const SizedBox(height: 8),
          Card(
            margin: EdgeInsets.zero,
            child: Padding(
              padding: const EdgeInsets.fromLTRB(10, 8, 8, 8),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.stretch,
                children: [
                  Row(
                    children: [
                      Expanded(
                        child: Row(
                          children: [
                            const Text(
                              'PID',
                              style: TextStyle(fontWeight: FontWeight.bold),
                            ),
                            if (controller.settingsBusy) ...[
                              const SizedBox(width: 8),
                              const SizedBox(
                                width: 14,
                                height: 14,
                                child: CircularProgressIndicator(
                                  strokeWidth: 2,
                                ),
                              ),
                            ],
                          ],
                        ),
                      ),
                      Container(
                        padding: const EdgeInsets.symmetric(
                          horizontal: 8,
                          vertical: 4,
                        ),
                        decoration: BoxDecoration(
                          color: controller.pidApplied
                              ? Colors.green.shade50
                              : Colors.grey.shade200,
                          borderRadius: BorderRadius.circular(12),
                        ),
                        child: Text(
                          controller.pidApplied ? '적용' : '꺼짐',
                          style: TextStyle(
                            color: controller.pidApplied
                                ? Colors.green.shade800
                                : Colors.blueGrey,
                            fontSize: 12,
                            fontWeight: FontWeight.bold,
                          ),
                        ),
                      ),
                      Switch.adaptive(
                        value: controller.pidApplied,
                        onChanged:
                            driveControlsEnabled &&
                                controller.settingsLoaded &&
                                !controller.settingsBusy
                            ? (enabled) =>
                                  unawaited(_setPidEnabled(controller, enabled))
                            : null,
                      ),
                      const SizedBox(width: 4),
                      IconButton(
                        tooltip: 'PID 설정',
                        visualDensity: VisualDensity.compact,
                        onPressed:
                            controller.isConnected && !controller.settingsBusy
                            ? () => unawaited(_showPidSettings(controller))
                            : null,
                        icon: const Icon(Icons.tune),
                      ),
                    ],
                  ),
                  Text(
                    controller.settingsLoaded
                        ? 'P ${_formatPid(controller.pidKpMilli)}  ·  '
                              'I ${_formatPid(controller.pidKiMilli)}  ·  '
                              'D ${_formatPid(controller.pidKdMilli)}  ·  '
                              '직진 ${controller.joystickThreshold.toStringAsFixed(2)}'
                        : 'PID 값 대기 중',
                    style: const TextStyle(
                      color: Colors.blueGrey,
                      fontSize: 12,
                    ),
                  ),
                  const Divider(height: 12),
                  Row(
                    children: [
                      const Text(
                        '센서 · 제어',
                        style: TextStyle(
                          fontSize: 12,
                          fontWeight: FontWeight.bold,
                        ),
                      ),
                      const Spacer(),
                      Text(
                        controller.hasFreshPidStatus
                            ? controller.devicePidRunning
                                  ? controller.devicePidReverse
                                        ? '후진 보정 중'
                                        : '직진 보정 중'
                                  : controller.deviceCommandActive
                                  ? '주행 중'
                                  : controller.pidApplied
                                  ? 'PID 준비'
                                  : 'PID 비활성'
                            : 'PID 대기',
                        style: TextStyle(
                          color: controller.devicePidRunning
                              ? Colors.green.shade800
                              : Colors.blueGrey,
                          fontSize: 12,
                          fontWeight: FontWeight.bold,
                        ),
                      ),
                      const SizedBox(width: 8),
                      Text(
                        controller.hasFreshImuStatus
                            ? !controller.deviceImuInitialized
                                  ? 'IMU 미초기화'
                                  : !controller.deviceImuCalibrated
                                  ? 'IMU 보정 중'
                                  : controller.deviceImuValid
                                  ? 'IMU 정상'
                                  : 'IMU 오류'
                            : 'IMU 대기',
                        style: TextStyle(
                          color:
                              controller.hasFreshImuStatus &&
                                  controller.deviceImuValid
                              ? Colors.teal.shade800
                              : Colors.blueGrey,
                          fontSize: 12,
                          fontWeight: FontWeight.bold,
                        ),
                      ),
                    ],
                  ),
                  const SizedBox(height: 3),
                  Align(
                    alignment: Alignment.centerLeft,
                    child: FittedBox(
                      fit: BoxFit.scaleDown,
                      alignment: Alignment.centerLeft,
                      child: Text(
                        controller.hasFreshPidStatus
                            ? 'Yaw 현재 ${controller.deviceCurrentYaw.toStringAsFixed(2)}° · '
                                  '목표 ${controller.deviceTargetYaw.toStringAsFixed(2)}° · '
                                  '오차 ${controller.devicePidError.toStringAsFixed(2)}° · '
                                  '출력 ${controller.devicePidOutput.toStringAsFixed(2)} · '
                                  'PWM ${controller.deviceLeftPwm}/${controller.deviceRightPwm}'
                            : '주행을 시작하면 PID 제어값이 표시됩니다.',
                        maxLines: 1,
                        style: const TextStyle(fontSize: 11),
                      ),
                    ),
                  ),
                  const SizedBox(height: 2),
                  Align(
                    alignment: Alignment.centerLeft,
                    child: FittedBox(
                      fit: BoxFit.scaleDown,
                      alignment: Alignment.centerLeft,
                      child: Text(
                        controller.hasFreshImuStatus
                            ? 'IMU Yaw ${controller.deviceYaw.toStringAsFixed(2)}° · '
                                  'Gyro Z ${controller.deviceGyroZDps.toStringAsFixed(2)} dps · '
                                  '${controller.deviceTemperatureC.toStringAsFixed(1)} °C · '
                                  '오류 ${controller.deviceImuErrorCount}회'
                            : 'MPU6050 상태 대기 중',
                        maxLines: 1,
                        style: const TextStyle(
                          color: Colors.blueGrey,
                          fontSize: 11,
                        ),
                      ),
                    ),
                  ),
                ],
              ),
            ),
          ),
        ],
      ),
    );
  }
}

// ======================================================
// 로봇팔 수동 조종 화면
// ======================================================

/// 6개 관절의 마지막 명령 자세와 원점·이동 자세를 관리하는 화면이다.
class RobotArmControlTab extends StatefulWidget {
  const RobotArmControlTab({super.key});

  static const int jointCount = 6;
  static const int gripperIndex = 5;
  static const List<String> jointNames = [
    'Base',
    'Shoulder',
    'Elbow',
    'Wrist Tilt',
    'Wrist Rotate',
    'Gripper',
  ];
  static const List<String> jointLabels = [
    '베이스 회전',
    '어깨',
    '팔꿈치',
    '손목 상하',
    '손목 회전',
    '그리퍼',
  ];
  static const List<int> originPose = [90, 90, 90, 90, 90, 0];
  static List<int> currentAngles = List<int>.from(originPose);

  static void setOriginPose() {
    currentAngles = List<int>.from(originPose);
  }

  /// 완료 ACK를 받은 로봇팔 명령 자세를 수동 제어 화면에 반영한다.
  static void applyCommandedPose(
    List<int> pose, {
    bool preserveGripper = false,
  }) {
    if (pose.length != jointCount ||
        pose.any((value) => value < 0 || value > 180)) {
      return;
    }
    final int gripper = currentAngles[gripperIndex];
    currentAngles = List<int>.from(pose);
    if (preserveGripper) {
      currentAngles[gripperIndex] = gripper;
    }
  }

  /// 패킷의 0~180 값을 화면의 각도 또는 Gripper 퍼센트로 바꾼다.
  static int packetToDisplayValue(int index, int packetValue) {
    if (index == gripperIndex) {
      return ((packetValue * 100) / 180).round();
    }
    return packetValue - 90;
  }

  /// 화면의 각도 또는 Gripper 퍼센트를 패킷의 0~180 값으로 바꾼다.
  static int displayToPacketValue(int index, double displayValue) {
    if (index == gripperIndex) {
      return ((displayValue.clamp(0, 100) * 180) / 100).round();
    }
    return (displayValue + 90).round().clamp(0, 180).toInt();
  }

  /// 웨이포인트의 관절 값을 한 줄의 읽기 쉬운 문자열로 만든다.
  static String formatWaypoint(List<int> packetValues) {
    return packetValues
        .asMap()
        .entries
        .map((entry) {
          final value = packetToDisplayValue(entry.key, entry.value);
          return entry.key == gripperIndex ? '$value%' : '$value°';
        })
        .join(', ');
  }

  @override
  State<RobotArmControlTab> createState() => _RobotArmControlTabState();
}

class _RobotArmControlTabState extends State<RobotArmControlTab> {
  /// 필요하면 서보 출력을 활성화하고 원점 완료와 실제 도착을 확인한다.
  Future<void> _moveToOrigin(BluetoothController controller) async {
    final bool firstActivation = !controller.isArmEnabled;
    if (firstActivation) {
      final bool confirmed =
          await showDialog<bool>(
            context: context,
            barrierDismissible: false,
            builder: (context) => AlertDialog(
              title: const Text('원점 활성화'),
              content: const Text('주변과 케이블을 확인하세요.\n첫 움직임은 빠를 수 있습니다.'),
              actions: [
                TextButton(
                  onPressed: () => Navigator.of(context).pop(false),
                  child: const Text('취소'),
                ),
                FilledButton(
                  onPressed: () => Navigator.of(context).pop(true),
                  child: const Text('활성화'),
                ),
              ],
            ),
          ) ??
          false;
      if (!confirmed || !mounted) {
        return;
      }
    }

    setState(() {
      /* 사용자가 원점 버튼을 누른 경우에만 Gripper를 최대 열림으로 이동한다. */
      RobotArmControlTab.setOriginPose();
    });
    bool reached;
    if (controller.isArmEnabled) {
      reached = await controller.moveArmToOrigin(
        RobotArmControlTab.currentAngles,
      );
    } else {
      reached = await controller.enableArm(RobotArmControlTab.currentAngles);
    }

    if (!firstActivation || !reached || !mounted) {
      return;
    }

    final bool physicallyReached =
        await showDialog<bool>(
          context: context,
          barrierDismissible: false,
          builder: (dialogContext) => AlertDialog(
            title: const Text('원점 확인'),
            content: const Text('모든 관절이 멈추고 원점에 도착했나요?'),
            actions: [
              FilledButton.tonal(
                onPressed: () => Navigator.pop(dialogContext, false),
                child: const Text('아니요 · STOP'),
              ),
              FilledButton(
                onPressed: () => Navigator.pop(dialogContext, true),
                child: const Text('도착 확인'),
              ),
            ],
          ),
        ) ??
        false;
    if (!physicallyReached) {
      await controller.emergencyStop();
    }
  }

  /// 그리퍼 위치를 유지하면서 Flash의 이동 자세를 실행한다.
  void _moveToTravel(BluetoothController controller) {
    unawaited(
      context.read<TeachingController>().prepareTravelSequencePreservingGripper(
        controller,
      ),
    );
  }

  /// 화면의 관절 값을 패킷 값으로 갱신하고 최신 전체 자세를 전송한다.
  void _updateAngle(int originalIndex, double newAngle) {
    setState(() {
      final mappedAngle = RobotArmControlTab.displayToPacketValue(
        originalIndex,
        newAngle,
      );
      RobotArmControlTab.currentAngles[originalIndex] = mappedAngle;

      final controller = Provider.of<BluetoothController>(
        context,
        listen: false,
      );
      unawaited(controller.sendArmPose(RobotArmControlTab.currentAngles));
    });
  }

  /// 현재 화면을 가리지 않는 팝업에서 공통 서보 이동 속도를 조정한다.
  Future<void> _showServoSpeedDialog() {
    return showDialog<void>(
      context: context,
      builder: (dialogContext) => AlertDialog(
        title: const Row(
          children: [Icon(Icons.speed), SizedBox(width: 8), Text('서보 속도')],
        ),
        content: SizedBox(
          width: 360,
          child: Consumer<BluetoothController>(
            builder: (context, controller, _) => Column(
              mainAxisSize: MainAxisSize.min,
              children: [
                _EditableValueBadge(
                  valueName: '서보 속도',
                  value: controller.servoSpeedPercent,
                  minimum: 50,
                  maximum: 100,
                  unit: '%',
                  enabled: !controller.armCommandBusy,
                  textStyle: Theme.of(context).textTheme.headlineSmall
                      ?.copyWith(fontWeight: FontWeight.bold),
                  onApplied: controller.setServoSpeedPercent,
                ),
                const SizedBox(height: 8),
                _DirectAdjustSlider(
                  minimum: 50,
                  maximum: 100,
                  value: controller.servoSpeedPercent,
                  enabled: !controller.armCommandBusy,
                  adjustmentLabel: '%',
                  onChanged: controller.setServoSpeedPercent,
                  onChangeEnd: controller.setServoSpeedPercent,
                ),
              ],
            ),
          ),
        ),
        actions: [
          FilledButton(
            onPressed: () => Navigator.pop(dialogContext),
            child: const Text('완료'),
          ),
        ],
      ),
    );
  }

  /// 전체 관절을 S-curve 원점으로 이동한 뒤 토크를 유지하며 설정 화면을 연다.
  Future<void> _openServoCalibration(BluetoothController controller) async {
    if (!controller.isConnected) {
      ScaffoldMessenger.of(
        context,
      ).showSnackBar(const SnackBar(content: Text('먼저 HC-05에 연결해주세요.')));
      return;
    }
    if (controller.isEstopLatched) {
      ScaffoldMessenger.of(
        context,
      ).showSnackBar(const SnackBar(content: Text('먼저 STOP을 해제하세요.')));
      return;
    }
    if (!controller.settingsLoaded) {
      final bool loaded = await controller.requestSettings();
      if (!mounted) {
        return;
      }
      if (!loaded) {
        ScaffoldMessenger.of(
          context,
        ).showSnackBar(const SnackBar(content: Text('STM32 설정을 읽지 못했습니다.')));
        return;
      }
    }
    if (!mounted) {
      return;
    }
    final bool confirmed =
        await showDialog<bool>(
          context: context,
          barrierDismissible: false,
          builder: (dialogContext) => AlertDialog(
            title: const Text('보정 시작'),
            content: const Text(
              '로봇팔이 원점으로 이동합니다.\n'
              '주변과 케이블을 확인하고 STOP을 준비하세요.',
            ),
            actions: [
              TextButton(
                onPressed: () => Navigator.pop(dialogContext, false),
                child: const Text('취소'),
              ),
              FilledButton(
                onPressed: () => Navigator.pop(dialogContext, true),
                child: const Text('시작'),
              ),
            ],
          ),
        ) ??
        false;
    if (!confirmed || !mounted) {
      return;
    }

    setState(RobotArmControlTab.setOriginPose);
    final bool reached = controller.isArmEnabled
        ? await controller.moveArmToOrigin(RobotArmControlTab.currentAngles)
        : await controller.enableArm(RobotArmControlTab.currentAngles);
    if (!mounted) {
      return;
    }
    if (!reached) {
      ScaffoldMessenger.of(
        context,
      ).showSnackBar(const SnackBar(content: Text('원점 이동 실패')));
      return;
    }

    await Navigator.of(context).push<void>(
      MaterialPageRoute(builder: (_) => const ServoCalibrationScreen()),
    );
  }

  @override
  Widget build(BuildContext context) {
    final controller = Provider.of<BluetoothController>(context);

    return ListView(
      padding: const EdgeInsets.fromLTRB(8, 4, 8, 20),
      children: [
        Card(
          margin: EdgeInsets.zero,
          child: Padding(
            padding: const EdgeInsets.fromLTRB(10, 8, 8, 8),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.stretch,
              children: [
                Row(
                  children: [
                    Icon(
                      controller.isArmEnabled
                          ? Icons.check_circle
                          : Icons.pause_circle,
                      color: controller.isArmEnabled
                          ? Colors.green
                          : Colors.grey,
                    ),
                    const SizedBox(width: 8),
                    Expanded(
                      child: Column(
                        crossAxisAlignment: CrossAxisAlignment.start,
                        children: [
                          Text(
                            controller.isArmEnabled ? '로봇팔 준비됨' : '로봇팔 출력 꺼짐',
                            style: const TextStyle(fontWeight: FontWeight.bold),
                          ),
                          Text(
                            controller.isArmEnabled
                                ? '관절을 조절하거나 자세를 저장할 수 있습니다.'
                                : '안전 확인 후 로봇팔 켜기를 누르세요.',
                            style: const TextStyle(
                              color: Colors.blueGrey,
                              fontSize: 11,
                            ),
                          ),
                        ],
                      ),
                    ),
                    Tooltip(
                      message: '서보 속도 ${controller.servoSpeedPercent}%',
                      child: TextButton.icon(
                        style: TextButton.styleFrom(
                          visualDensity: VisualDensity.compact,
                        ),
                        onPressed: () => unawaited(_showServoSpeedDialog()),
                        icon: const Icon(Icons.speed, size: 18),
                        label: Text('${controller.servoSpeedPercent}%'),
                      ),
                    ),
                  ],
                ),
                const Divider(height: 12),
                Row(
                  children: [
                    Expanded(
                      child: FilledButton.icon(
                        onPressed:
                            controller.isEstopLatched ||
                                controller.armCommandBusy
                            ? null
                            : () => unawaited(_moveToOrigin(controller)),
                        icon: controller.armCommandBusy
                            ? const SizedBox(
                                width: 18,
                                height: 18,
                                child: CircularProgressIndicator(
                                  strokeWidth: 2,
                                ),
                              )
                            : const Icon(Icons.center_focus_strong),
                        label: Text(
                          controller.armCommandBusy
                              ? '응답 대기'
                              : controller.isArmEnabled
                              ? '원점으로 이동'
                              : '로봇팔 켜기 · 원점',
                        ),
                      ),
                    ),
                    const SizedBox(width: 8),
                    Expanded(
                      child: OutlinedButton.icon(
                        onPressed:
                            controller.isArmEnabled &&
                                !controller.isEstopLatched &&
                                !controller.armCommandBusy
                            ? () => _moveToTravel(controller)
                            : null,
                        icon: const Icon(Icons.directions_car_outlined),
                        label: const Text('이동 자세'),
                      ),
                    ),
                  ],
                ),
                const SizedBox(height: 4),
                Row(
                  children: [
                    Expanded(
                      child: TextButton.icon(
                        onPressed:
                            controller.isConnected && !controller.settingsBusy
                            ? () => unawaited(_openServoCalibration(controller))
                            : null,
                        icon: const Icon(Icons.tune, size: 18),
                        label: const Text('원점 보정'),
                      ),
                    ),
                    Expanded(
                      child: TextButton.icon(
                        onPressed: controller.isArmEnabled
                            ? () => unawaited(controller.disableArm())
                            : null,
                        icon: const Icon(Icons.power_settings_new, size: 18),
                        label: const Text('로봇팔 끄기'),
                      ),
                    ),
                  ],
                ),
              ],
            ),
          ),
        ),
        const SizedBox(height: 4),
        ...List.generate(RobotArmControlTab.jointCount, (index) {
          final int originalIndex = index;
          final isGripper = originalIndex == RobotArmControlTab.gripperIndex;
          final displayValue = RobotArmControlTab.packetToDisplayValue(
            originalIndex,
            RobotArmControlTab.currentAngles[originalIndex],
          ).toDouble();

          return Padding(
            padding: const EdgeInsets.only(bottom: 4),
            child: Card(
              margin: EdgeInsets.zero,
              child: Padding(
                padding: const EdgeInsets.fromLTRB(10, 4, 10, 2),
                child: Column(
                  children: [
                    Row(
                      children: [
                        Expanded(
                          child: Text(
                            RobotArmControlTab.jointLabels[originalIndex],
                            style: const TextStyle(fontWeight: FontWeight.bold),
                          ),
                        ),
                        _EditableValueBadge(
                          valueName:
                              RobotArmControlTab.jointLabels[originalIndex],
                          value: displayValue.round(),
                          minimum: isGripper ? 0 : -90,
                          maximum: isGripper ? 100 : 90,
                          unit: isGripper ? '%' : '°',
                          enabled:
                              controller.isArmEnabled &&
                              !controller.isEstopLatched &&
                              !controller.armCommandBusy,
                          onApplied: (value) =>
                              _updateAngle(originalIndex, value.toDouble()),
                        ),
                      ],
                    ),
                    _DirectAdjustSlider(
                      value: displayValue.round(),
                      minimum: isGripper ? 0 : -90,
                      maximum: isGripper ? 100 : 90,
                      enabled:
                          controller.isArmEnabled &&
                          !controller.isEstopLatched &&
                          !controller.armCommandBusy,
                      adjustmentLabel: isGripper ? '%' : '°',
                      onChanged: (value) =>
                          _updateAngle(originalIndex, value.toDouble()),
                      onChangeEnd: (value) =>
                          _updateAngle(originalIndex, value.toDouble()),
                    ),
                  ],
                ),
              ),
            ),
          );
        }),
        const SizedBox(height: 2),
        const SizedBox(height: 340, child: TeachingEditor(compact: true)),
      ],
    );
  }
}

/// 모든 서보 슬라이더에서 같은 방식으로 정확한 정수값을 입력하는 배지다.
class _EditableValueBadge extends StatelessWidget {
  const _EditableValueBadge({
    required this.valueName,
    required this.value,
    required this.minimum,
    required this.maximum,
    required this.unit,
    required this.enabled,
    required this.onApplied,
    this.textStyle,
  });

  final String valueName;
  final int value;
  final int minimum;
  final int maximum;
  final String unit;
  final bool enabled;
  final ValueChanged<int> onApplied;
  final TextStyle? textStyle;

  /// 팝업에서 검증된 값만 원래 슬라이더의 적용 함수로 전달한다.
  Future<void> _openInput(BuildContext context) async {
    if (!enabled) {
      return;
    }
    final int? enteredValue = await showDialog<int>(
      context: context,
      builder: (_) => _IntegerValueDialog(
        valueName: valueName,
        initialValue: value,
        minimum: minimum,
        maximum: maximum,
        unit: unit,
      ),
    );
    if (enteredValue != null && context.mounted) {
      onApplied(enteredValue);
    }
  }

  @override
  Widget build(BuildContext context) {
    return Tooltip(
      message: '$valueName 직접 입력',
      child: InkWell(
        borderRadius: BorderRadius.circular(12),
        onTap: enabled ? () => unawaited(_openInput(context)) : null,
        child: Container(
          padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 2),
          decoration: BoxDecoration(
            color: Theme.of(context).colorScheme.secondaryContainer,
            borderRadius: BorderRadius.circular(12),
          ),
          child: Row(
            mainAxisSize: MainAxisSize.min,
            children: [
              Text(
                '$value$unit',
                style:
                    textStyle ?? const TextStyle(fontWeight: FontWeight.bold),
              ),
              const SizedBox(width: 4),
              const Icon(Icons.edit_outlined, size: 14),
            ],
          ),
        ),
      ),
    );
  }
}

/// 직접 입력 팝업이 열려 있는 동안 숫자 입력 컨트롤러의 수명주기를 관리한다.
class _IntegerValueDialog extends StatefulWidget {
  const _IntegerValueDialog({
    required this.valueName,
    required this.initialValue,
    required this.minimum,
    required this.maximum,
    required this.unit,
  });

  final String valueName;
  final int initialValue;
  final int minimum;
  final int maximum;
  final String unit;

  @override
  State<_IntegerValueDialog> createState() => _IntegerValueDialogState();
}

class _IntegerValueDialogState extends State<_IntegerValueDialog> {
  late final TextEditingController _controller;
  String? _errorText;

  @override
  void initState() {
    super.initState();
    final String initialText = '${widget.initialValue}';
    _controller = TextEditingController(text: initialText)
      ..selection = TextSelection(
        baseOffset: 0,
        extentOffset: initialText.length,
      );
  }

  @override
  void dispose() {
    _controller.dispose();
    super.dispose();
  }

  /// 범위를 확인한 값만 호출 화면에 반환한다.
  void _applyValue() {
    final int? value = int.tryParse(_controller.text);
    if (value == null || value < widget.minimum || value > widget.maximum) {
      setState(
        () => _errorText = '${widget.minimum}~${widget.maximum} 범위로 입력하세요.',
      );
      return;
    }
    Navigator.pop(context, value);
  }

  @override
  Widget build(BuildContext context) {
    final bool acceptsNegative = widget.minimum < 0;
    final int maximumDigits = [
      widget.minimum.abs(),
      widget.maximum.abs(),
    ].reduce((a, b) => a > b ? a : b).toString().length;
    return AlertDialog(
      title: Text('${widget.valueName} 직접 입력'),
      content: TextField(
        controller: _controller,
        autofocus: true,
        keyboardType: TextInputType.numberWithOptions(signed: acceptsNegative),
        inputFormatters: [
          TextInputFormatter.withFunction((oldValue, newValue) {
            final RegExp pattern = acceptsNegative
                ? RegExp('^-?\\d{0,$maximumDigits}\$')
                : RegExp('^\\d{0,$maximumDigits}\$');
            return pattern.hasMatch(newValue.text) ? newValue : oldValue;
          }),
        ],
        decoration: InputDecoration(
          labelText: '${widget.minimum}~${widget.maximum}${widget.unit}',
          suffixText: widget.unit,
          errorText: _errorText,
        ),
        onSubmitted: (_) => _applyValue(),
      ),
      actions: [
        TextButton(
          onPressed: () => Navigator.pop(context),
          child: const Text('취소'),
        ),
        FilledButton(onPressed: _applyValue, child: const Text('적용')),
      ],
    );
  }
}

/// 손잡이 드래그만 허용해 트랙 오입력으로 값이 급변하지 않는 공통 슬라이더다.
///
/// 드래그 값은 즉시 적용하며, 양옆 버튼으로 1단계씩 정밀하게 조정할 수 있다.
class _DirectAdjustSlider extends StatefulWidget {
  const _DirectAdjustSlider({
    required this.value,
    required this.minimum,
    required this.maximum,
    required this.enabled,
    required this.onChanged,
    required this.onChangeEnd,
    this.adjustmentLabel = '단계',
  });

  final int value;
  final int minimum;
  final int maximum;
  final bool enabled;
  final ValueChanged<int> onChanged;
  final ValueChanged<int> onChangeEnd;
  final String adjustmentLabel;

  @override
  State<_DirectAdjustSlider> createState() => _DirectAdjustSliderState();
}

class _DirectAdjustSliderState extends State<_DirectAdjustSlider> {
  int _currentValue = 0;
  bool _dragging = false;

  @override
  void initState() {
    super.initState();
    _currentValue = widget.value;
  }

  @override
  void didUpdateWidget(covariant _DirectAdjustSlider oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (!widget.enabled) {
      _dragging = false;
      _currentValue = widget.value;
    } else if (!_dragging) {
      _currentValue = widget.value;
    }
  }

  /// 현재 표시값에서 지정한 양만큼 증가하거나 감소시킨다.
  void _changeBy(int amount) {
    if (!widget.enabled || amount == 0) {
      return;
    }
    final int nextValue = (_currentValue + amount)
        .clamp(widget.minimum, widget.maximum)
        .toInt();
    if (nextValue == _currentValue) {
      return;
    }
    setState(() => _currentValue = nextValue);
    widget.onChanged(nextValue);
  }

  /// 손잡이 위치를 지연 없이 정수 제어값에 반영한다.
  void _changeImmediately(double value) {
    final int nextValue = value.round().clamp(widget.minimum, widget.maximum);
    if (!widget.enabled || nextValue == _currentValue) {
      return;
    }
    setState(() {
      _currentValue = nextValue;
    });
    widget.onChanged(nextValue);
  }

  /// 드래그가 끝난 최종값을 한 번 더 전달해 마지막 명령 유실을 막는다.
  void _finishDrag(double value) {
    final int finalValue = value.round().clamp(widget.minimum, widget.maximum);
    setState(() {
      _currentValue = finalValue;
      _dragging = false;
    });
    widget.onChangeEnd(finalValue);
  }

  @override
  Widget build(BuildContext context) {
    final slider = Expanded(
      child: Slider(
        min: widget.minimum.toDouble(),
        max: widget.maximum.toDouble(),
        value: _currentValue.toDouble(),
        allowedInteraction: SliderInteraction.slideOnly,
        onChangeStart: widget.enabled
            ? (_) => setState(() => _dragging = true)
            : null,
        onChanged: widget.enabled ? _changeImmediately : null,
        onChangeEnd: widget.enabled ? _finishDrag : null,
      ),
    );

    return Row(
      children: [
        _SliderStepButton(
          icon: Icons.remove,
          semanticsLabel: '1 ${widget.adjustmentLabel} 감소',
          direction: -1,
          enabled: widget.enabled,
          onStep: _changeBy,
          onChangeEnd: () => widget.onChangeEnd(_currentValue),
        ),
        slider,
        _SliderStepButton(
          icon: Icons.add,
          semanticsLabel: '1 ${widget.adjustmentLabel} 증가',
          direction: 1,
          enabled: widget.enabled,
          onStep: _changeBy,
          onChangeEnd: () => widget.onChangeEnd(_currentValue),
        ),
      ],
    );
  }
}

/// 값을 1단계씩 또는 장기 누름으로 5단계씩 조정하는 공통 버튼이다.
class _SliderStepButton extends StatefulWidget {
  const _SliderStepButton({
    required this.icon,
    required this.semanticsLabel,
    required this.direction,
    required this.enabled,
    required this.onStep,
    required this.onChangeEnd,
  });

  final IconData icon;
  final String semanticsLabel;
  final int direction;
  final bool enabled;
  final ValueChanged<int> onStep;
  final VoidCallback onChangeEnd;

  @override
  State<_SliderStepButton> createState() => _SliderStepButtonState();
}

class _SliderStepButtonState extends State<_SliderStepButton> {
  Timer? _repeatTimer;

  void _tap() {
    if (!widget.enabled) {
      return;
    }
    widget.onStep(widget.direction);
    widget.onChangeEnd();
  }

  void _startRepeating() {
    if (!widget.enabled || _repeatTimer != null) {
      return;
    }
    widget.onStep(widget.direction * 5);
    _repeatTimer = Timer.periodic(const Duration(milliseconds: 80), (_) {
      widget.onStep(widget.direction * 5);
    });
  }

  void _stopRepeating() {
    _repeatTimer?.cancel();
    _repeatTimer = null;
    widget.onChangeEnd();
  }

  void _cancelTimers() {
    _repeatTimer?.cancel();
    _repeatTimer = null;
  }

  @override
  void didUpdateWidget(covariant _SliderStepButton oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (!widget.enabled) {
      _cancelTimers();
    }
  }

  @override
  void dispose() {
    _cancelTimers();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Semantics(
      button: true,
      enabled: widget.enabled,
      label: widget.semanticsLabel,
      onTap: widget.enabled ? _tap : null,
      child: RawGestureDetector(
        behavior: HitTestBehavior.opaque,
        gestures: {
          TapGestureRecognizer:
              GestureRecognizerFactoryWithHandlers<TapGestureRecognizer>(
                () => TapGestureRecognizer(),
                (recognizer) {
                  recognizer.onTap = widget.enabled ? _tap : null;
                },
              ),
          LongPressGestureRecognizer:
              GestureRecognizerFactoryWithHandlers<LongPressGestureRecognizer>(
                () => LongPressGestureRecognizer(
                  duration: const Duration(milliseconds: 300),
                ),
                (recognizer) {
                  recognizer
                    ..onLongPressStart = widget.enabled
                        ? (_) => _startRepeating()
                        : null
                    ..onLongPressEnd = widget.enabled
                        ? (_) => _stopRepeating()
                        : null
                    ..onLongPressCancel = widget.enabled
                        ? _stopRepeating
                        : null;
                },
              ),
        },
        child: Container(
          width: 44,
          height: 44,
          decoration: BoxDecoration(
            color: widget.enabled
                ? Theme.of(context).colorScheme.primaryContainer
                : Theme.of(context).colorScheme.surfaceContainerHighest,
            borderRadius: BorderRadius.circular(12),
          ),
          child: Icon(
            widget.icon,
            color: widget.enabled
                ? Theme.of(context).colorScheme.primary
                : Theme.of(context).disabledColor,
          ),
        ),
      ),
    );
  }
}

// ======================================================
// 관절별 3점 원점 보정 화면
// ======================================================

/// 패킷 위치 0~180을 저장된 -90°/0°/+90° 펄스 사이에서 선형 보간한다.
int calibrationPulseForPosition(List<int> pulses, int packetPosition) {
  assert(pulses.length == 3);
  final int position = packetPosition.clamp(0, 180).toInt();
  if (position <= 90) {
    return pulses[0] + (((pulses[1] - pulses[0]) * position) / 90).round();
  }
  return pulses[1] + (((pulses[2] - pulses[1]) * (position - 90)) / 90).round();
}

/// 관절과 보정점을 직접 선택해 한 서보씩 펄스폭을 조정한다.
///
/// 일반 관절은 -90°/0°/+90°, Gripper는 열림/닫힘을 조정하며 사용자가
/// 원점 저장 버튼을 누른 경우에만 전체 값을 STM32 Flash에 저장한다.
class ServoCalibrationScreen extends StatefulWidget {
  const ServoCalibrationScreen({super.key});

  @override
  State<ServoCalibrationScreen> createState() => _ServoCalibrationScreenState();
}

class _ServoCalibrationScreenState extends State<ServoCalibrationScreen> {
  static const Duration _verificationMoveTimeout = Duration(seconds: 20);
  static const Duration _verificationHoldTime = Duration(seconds: 1);
  static const List<String> _jointNames = RobotArmControlTab.jointLabels;
  static const List<String> _pointLabels = ['-90°', '0°', '+90°'];
  BluetoothController? _controller;
  List<List<int>> _pulseValues = [];
  List<int> _currentPulseUs = [];
  Timer? _previewTimer;
  int? _pendingPreviewJoint;
  int? _pendingPreviewPulseUs;
  int _currentJoint = 0;
  int _currentPoint = 1;
  bool _initialized = false;
  bool _saving = false;
  bool _returningOrigin = false;
  bool _calibrationDirty = false;
  // 이전 완료 화면 분기는 새 선택형 편집 UI에서는 사용하지 않지만, 저장된
  // 결과 표시 레이아웃을 유지하기 위해 항상 false인 상태로 둔다.
  final bool _completed = false;
  final bool _originCheckAttempted = false;
  final bool _originReached = false;
  String _verificationStatus = '원점 확인 시퀀스를 준비하고 있습니다.';

  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    final controller = Provider.of<BluetoothController>(context);
    _controller = controller;
    if (!_initialized) {
      _pulseValues = controller.servoCalibrationUs
          .map((values) => List<int>.from(values))
          .toList();
      _currentPulseUs = List<int>.generate(
        _jointNames.length,
        _pulseForLastCommand,
      );
      _initialized = true;
    }
  }

  /// 마지막 명령 각도를 기존 3점 보정값으로 보간해 첫 미리보기 펄스를 만든다.
  int _pulseForLastCommand(int joint) {
    return calibrationPulseForPosition(
      _pulseValues[joint],
      RobotArmControlTab.currentAngles[joint],
    );
  }

  /// 선택한 관절에 맞는 최소 펄스폭을 반환한다.
  int _minimumPulse(int joint) => joint == 5 ? 1000 : 350;

  /// 선택한 관절에 맞는 최대 펄스폭을 반환한다.
  int _maximumPulse(int joint) => joint == 5 ? 2000 : 2650;

  /// 슬라이더와 직접 입력이 동일한 RAM 갱신 및 서보 미리보기 경로를 사용한다.
  void _applyCalibrationPulse(
    int joint,
    int pulseUs, {
    required bool immediately,
  }) {
    setState(() {
      _currentPulseUs[joint] = pulseUs;
      _pulseValues[joint][_currentPoint] = pulseUs;
      if (joint == 5) {
        _pulseValues[5][1] = (_pulseValues[5][0] + _pulseValues[5][2]) ~/ 2;
      }
      _calibrationDirty = true;
    });
    if (immediately) {
      _previewImmediately(joint, pulseUs);
    } else {
      _schedulePreview(joint, pulseUs);
    }
  }

  /// 9600 baud 전송이 밀리지 않도록 가장 최근 미리보기 값만 예약한다.
  void _schedulePreview(int joint, int pulseUs) {
    _pendingPreviewJoint = joint;
    _pendingPreviewPulseUs = pulseUs;
    if (_previewTimer == null) {
      _flushPreview();
    }
  }

  /// 예약된 최신 펄스 한 개를 보내고 다음 전송 가능 시간을 설정한다.
  void _flushPreview() {
    final controller = _controller;
    final joint = _pendingPreviewJoint;
    final pulseUs = _pendingPreviewPulseUs;
    _pendingPreviewJoint = null;
    _pendingPreviewPulseUs = null;

    if (controller != null && joint != null && pulseUs != null && mounted) {
      unawaited(controller.previewServoPulse(joint, pulseUs));
    }

    /* 9600 baud에서 패킷이 밀리지 않도록 30 ms마다 최신 값만 보낸다. */
    _previewTimer = Timer(const Duration(milliseconds: 30), () {
      _previewTimer = null;
      if (_pendingPreviewJoint != null) {
        _flushPreview();
      }
    });
  }

  /// 버튼 입력이 끝났을 때 최종 펄스 값을 즉시 한 번 전송한다.
  void _previewImmediately(int joint, int pulseUs) {
    _previewTimer?.cancel();
    _previewTimer = null;
    _pendingPreviewJoint = null;
    _pendingPreviewPulseUs = null;
    final controller = _controller;
    if (controller != null) {
      unawaited(controller.previewServoPulse(joint, pulseUs));
    }
  }

  /// 예약된 미리보기와 실제 단일 서보 출력을 모두 중지한다.
  Future<void> _stopPreview() async {
    _previewTimer?.cancel();
    _previewTimer = null;
    _pendingPreviewJoint = null;
    _pendingPreviewPulseUs = null;
    await _controller?.stopServoPreview();
  }

  /// 모든 관절의 펄스 순서가 조립 방향과 맞는지 검사한다.
  String? _validationError() {
    for (int joint = 0; joint < 5; joint++) {
      final String? error = _jointValidationError(joint);
      if (error != null) {
        return error;
      }
    }

    return _jointValidationError(5);
  }

  /// 한 관절의 3점 또는 Gripper 열림·닫힘 펄스 순서를 검사한다.
  String? _jointValidationError(int joint) {
    final values = _pulseValues[joint];
    if (joint == 5) {
      if (values[0] > values[2]) {
        return null;
      }

      final String cause = values[0] == values[2]
          ? '열림과 닫힘에 같은 펄스값이 저장되었습니다.'
          : '열림과 닫힘 펄스값이 서로 바뀐 것으로 보입니다.';
      return 'Gripper 펄스 순서 오류\n\n'
          '현재값: 열림 ${values[0]} µs · 닫힘 ${values[2]} µs\n'
          '필요한 순서: 열림 > 닫힘\n\n'
          '확인할 부분: $cause';
    }

    final int minusPulse = values[0];
    final int centerPulse = values[1];
    final int plusPulse = values[2];
    final bool reversed = BluetoothController.servoReversed[joint];
    final bool valid = reversed
        ? minusPulse > centerPulse && centerPulse > plusPulse
        : minusPulse < centerPulse && centerPulse < plusPulse;
    if (!valid) {
      final List<String> causes = [];
      if (minusPulse == centerPulse ||
          centerPulse == plusPulse ||
          minusPulse == plusPulse) {
        causes.add('서로 다른 각도에 같은 펄스값이 있습니다.');
      }

      final bool endpointsReversed = reversed
          ? minusPulse < plusPulse
          : minusPulse > plusPulse;
      if (endpointsReversed) {
        causes.add('-90°와 +90° 펄스값이 서로 바뀐 것으로 보입니다.');
      }

      final int lowerEndpoint = minusPulse < plusPulse ? minusPulse : plusPulse;
      final int upperEndpoint = minusPulse > plusPulse ? minusPulse : plusPulse;
      if (centerPulse <= lowerEndpoint || centerPulse >= upperEndpoint) {
        causes.add('0° 펄스값이 -90°와 +90° 값 사이에 있지 않습니다.');
      }

      if (causes.isEmpty) {
        causes.add('각 단계의 펄스값과 선택한 관절을 다시 확인해주세요.');
      }

      final String expected = reversed
          ? '-90° > 0° > +90°'
          : '-90° < 0° < +90°';
      return '${_jointNames[joint]} 펄스 순서 오류\n\n'
          '현재값: -90° $minusPulse µs · 0° $centerPulse µs · '
          '+90° $plusPulse µs\n'
          '필요한 순서: $expected\n\n'
          '확인할 부분:\n- ${causes.join('\n- ')}';
    }
    return null;
  }

  /// 긴 오류 내용을 읽고 현재 단계에서 다시 조정할 수 있도록 확인창을 띄운다.
  Future<void> _showPulseOrderError(String message) async {
    await showDialog<void>(
      context: context,
      builder: (dialogContext) => AlertDialog(
        title: const Text('펄스 순서 확인'),
        content: SingleChildScrollView(child: SelectableText(message)),
        actions: [
          FilledButton(
            onPressed: () => Navigator.pop(dialogContext),
            child: const Text('값 다시 조정'),
          ),
        ],
      ),
    );
  }

  /// 현재 관절을 원점으로 복귀시킨 뒤 사용자가 고른 다음 관절을 활성화한다.
  Future<void> _selectJoint(int joint) async {
    final controller = _controller;
    if (controller == null ||
        joint == _currentJoint ||
        _saving ||
        _returningOrigin ||
        controller.servoPreviewMoving) {
      return;
    }

    final int previousHomePoint = _currentJoint == 5 ? 0 : 1;
    final bool returned = await _previewPulseAndWait(
      controller,
      _currentJoint,
      _pulseValues[_currentJoint][previousHomePoint],
    );
    if (!returned || !mounted) {
      return;
    }

    final int nextPoint = joint == 5 ? 0 : 1;
    setState(() {
      _currentJoint = joint;
      _currentPoint = nextPoint;
      _currentPulseUs[joint] = _pulseValues[joint][nextPoint];
    });
  }

  /// 선택한 관절에서 -90°/0°/+90° 중 원하는 보정점을 S-curve로 불러온다.
  void _selectCalibrationPoint(int point) {
    final controller = _controller;
    if (controller == null ||
        _saving ||
        _returningOrigin ||
        controller.servoPreviewMoving ||
        (_currentJoint == 5 && point == 1)) {
      return;
    }

    setState(() {
      _currentPoint = point;
      _currentPulseUs[_currentJoint] = _pulseValues[_currentJoint][point];
    });
    _previewImmediately(_currentJoint, _currentPulseUs[_currentJoint]);
  }

  /// 단일 펄스 이동 명령을 보내고 최신 완료 ACK 또는 제한 시간을 기다린다.
  Future<bool> _previewPulseAndWait(
    BluetoothController controller,
    int joint,
    int pulseUs,
  ) async {
    await controller.previewServoPulse(joint, pulseUs);
    final DateTime deadline = DateTime.now().add(_verificationMoveTimeout);
    while (mounted &&
        controller.isConnected &&
        !controller.isEstopLatched &&
        controller.servoPreviewMoving &&
        DateTime.now().isBefore(deadline)) {
      await Future<void>.delayed(const Duration(milliseconds: 20));
    }
    return mounted &&
        controller.isConnected &&
        !controller.isEstopLatched &&
        !controller.servoPreviewMoving &&
        DateTime.now().isBefore(deadline);
  }

  /// 원점 보정값만 검증해 저장하고 이동 자세는 마지막 저장값 그대로 보존한다.
  Future<void> _saveCalibrationOnly() async {
    final controller = _controller;
    if (controller == null ||
        _saving ||
        _returningOrigin ||
        controller.servoPreviewMoving) {
      return;
    }

    final String? error = _validationError();
    if (error != null) {
      await _showPulseOrderError(error);
      return;
    }

    // 마지막 미리보기 세션을 먼저 닫아 펌웨어의 2초 안전 타임아웃을
    // 기다리지 않고 설정 저장을 시작한다.
    await _stopPreview();
    if (!mounted) {
      return;
    }

    setState(() => _saving = true);
    final bool saved = await controller.saveSettings(
      kpMilli: controller.pidKpMilli,
      kiMilli: controller.pidKiMilli,
      kdMilli: controller.pidKdMilli,
      servoPulsesUs: _pulseValues,
      travelPose: controller.travelPoseAngles,
    );
    if (!mounted) {
      return;
    }
    setState(() => _saving = false);

    controller.clearAckMessage();
    showAppMessage(
      context,
      saved
          ? controller.isFeaturePreviewMode
                ? 'TEST 보정값 저장'
                : '원점 보정 저장 완료'
          : '원점 보정 저장 실패',
      backgroundColor: saved ? Colors.green : Colors.red,
    );
    if (!saved || !mounted) {
      return;
    }

    setState(() => _calibrationDirty = false);
    RobotArmControlTab.setOriginPose();
    final bool reached = await controller.moveArmToOrigin(
      RobotArmControlTab.currentAngles,
      timeout: _verificationMoveTimeout,
      announceCompletion: false,
    );
    if (!mounted) {
      return;
    }
    if (!reached) {
      ScaffoldMessenger.of(context).showSnackBar(
        const SnackBar(content: Text('저장은 완료됐지만 원점 복귀 ACK를 받지 못했습니다.')),
      );
    }
  }

  /// 저장한 모든 관절의 -90°/+90°/0°를 하나씩 움직여 최종 확인한다.
  Future<void> _returnToOrigin(BluetoothController controller) async {
    final bool confirmed =
        await showDialog<bool>(
          context: context,
          barrierDismissible: false,
          builder: (dialogContext) => AlertDialog(
            title: const Text('원점 확인 시퀀스'),
            content: const Text(
              '관절을 하나씩 -90° → +90° → 0°로 확인합니다.\n'
              'Gripper: 열림 → 닫힘 → 열림\n'
              '주변을 확인하고 STOP을 준비하세요.',
            ),
            actions: [
              TextButton(
                onPressed: () => Navigator.pop(dialogContext, false),
                child: const Text('취소'),
              ),
              FilledButton(
                onPressed: () => Navigator.pop(dialogContext, true),
                child: const Text('시작'),
              ),
            ],
          ),
        ) ??
        false;
    if (!confirmed || !mounted) {
      return;
    }

    setState(() {
      _returningOrigin = true;
      _verificationStatus = '모든 관절을 현재 보정값의 원점으로 이동 중입니다.';
    });

    bool reached = true;
    for (int joint = 0; joint < _jointNames.length && reached; joint++) {
      final int homePoint = joint == 5 ? 0 : 1;
      setState(() => _verificationStatus = '${_jointNames[joint]} 원점 복귀 중');
      reached = await _previewPulseAndWait(
        controller,
        joint,
        _pulseValues[joint][homePoint],
      );
    }

    for (int joint = 0; joint < 5 && reached; joint++) {
      for (final int point in const [0, 2, 1]) {
        setState(
          () => _verificationStatus =
              '${_jointNames[joint]} · ${_pointLabels[point]} 확인 중',
        );
        reached = await _previewPulseAndWait(
          controller,
          joint,
          _pulseValues[joint][point],
        );
        if (reached) {
          await Future<void>.delayed(_verificationHoldTime);
        }
      }
    }

    for (final int point in const [0, 2, 0]) {
      if (!reached) {
        break;
      }
      setState(
        () =>
            _verificationStatus = 'Gripper · ${point == 0 ? '열림' : '닫힘'} 확인 중',
      );
      reached = await _previewPulseAndWait(
        controller,
        5,
        _pulseValues[5][point],
      );
      if (reached) {
        await Future<void>.delayed(_verificationHoldTime);
      }
    }

    // 확인이 끝나면 미리보기 상태와 차량 이동 금지를 해제하되 마지막 원점
    // PWM은 유지한다.
    await _stopPreview();
    if (!mounted) {
      return;
    }

    bool physicallyReached = reached;
    if (reached && mounted) {
      physicallyReached =
          await showDialog<bool>(
            context: context,
            barrierDismissible: false,
            builder: (dialogContext) => AlertDialog(
              title: const Text('확인 완료'),
              content: const Text('걸림 없이 원점에 도착했나요?'),
              actions: [
                FilledButton.tonal(
                  onPressed: () => Navigator.pop(dialogContext, false),
                  child: const Text('아니요 · STOP'),
                ),
                FilledButton(
                  onPressed: () => Navigator.pop(dialogContext, true),
                  child: const Text('확인'),
                ),
              ],
            ),
          ) ??
          false;
    }

    if (!reached || !physicallyReached) {
      await controller.emergencyStop();
    }
    if (!mounted) {
      return;
    }
    setState(() {
      _returningOrigin = false;
      _verificationStatus = reached && physicallyReached
          ? '원점 확인 시퀀스를 완료했습니다.'
          : '원점 확인 시퀀스를 완료하지 못했습니다.';
      _currentPoint = _currentJoint == 5 ? 0 : 1;
      _currentPulseUs[_currentJoint] =
          _pulseValues[_currentJoint][_currentPoint];
    });
    RobotArmControlTab.setOriginPose();
  }

  /// 보정용 출력을 끈 뒤 로봇팔 화면으로 안전하게 돌아간다.
  Future<void> _closeScreen() async {
    if (_saving || _returningOrigin) {
      return;
    }
    await _stopPreview();
    if (mounted) {
      Navigator.pop(context);
    }
  }

  @override
  void dispose() {
    _previewTimer?.cancel();
    _pendingPreviewJoint = null;
    _pendingPreviewPulseUs = null;
    final controller = _controller;
    if (controller != null) {
      unawaited(controller.stopServoPreview());
    }
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final controller = Provider.of<BluetoothController>(context);
    final List<int> selectablePoints = _currentJoint == 5
        ? const [0, 2]
        : const [0, 1, 2];
    final bool controlsEnabled =
        controller.isConnected &&
        controller.isArmEnabled &&
        !controller.isEstopLatched &&
        !controller.armCommandBusy &&
        !_saving;

    return PopScope(
      canPop: !_saving && !_returningOrigin,
      child: Scaffold(
        appBar: AppBar(
          title: const Text('원점 보정'),
          leading: IconButton(
            onPressed: _saving || _returningOrigin
                ? null
                : () => unawaited(_closeScreen()),
            icon: const Icon(Icons.arrow_back),
          ),
          actions: [
            _buildAppBarAction(
              label: controller.isFeaturePreviewMode
                  ? '테스트'
                  : controller.isConnected
                  ? '연결됨'
                  : '끊김',
              tooltip: controller.isConnected ? '연결 상태 정상' : '연결 끊김',
              icon: controller.isFeaturePreviewMode
                  ? Icons.science_outlined
                  : controller.isConnected
                  ? Icons.bluetooth_connected
                  : Icons.bluetooth_disabled,
              backgroundColor: controller.isFeaturePreviewMode
                  ? Colors.deepPurple.shade600
                  : controller.isConnected
                  ? Colors.teal.shade700
                  : Colors.blueGrey.shade600,
              onPressed: () => showAppMessage(
                context,
                controller.isConnected ? '연결 상태가 정상입니다.' : '연결이 끊겼습니다.',
              ),
            ),
            _buildAppBarAction(
              label: 'STOP',
              tooltip: '차량과 로봇팔 긴급 정지',
              onPressed: () => unawaited(controller.emergencyStop()),
              icon: Icons.warning_rounded,
              backgroundColor: Colors.red.shade700,
            ),
          ],
        ),
        body: _returningOrigin
            ? Center(
                child: Column(
                  mainAxisSize: MainAxisSize.min,
                  children: [
                    const CircularProgressIndicator(),
                    const SizedBox(height: 16),
                    Text(_verificationStatus, textAlign: TextAlign.center),
                    const SizedBox(height: 8),
                    const Text(
                      '문제가 생기면 STOP을 누르세요.',
                      style: TextStyle(color: Colors.redAccent),
                    ),
                  ],
                ),
              )
            : _completed
            ? ListView(
                physics: const NeverScrollableScrollPhysics(),
                padding: const EdgeInsets.fromLTRB(12, 16, 12, 24),
                children: [
                  Card(
                    color: !_originCheckAttempted
                        ? Theme.of(context).colorScheme.primaryContainer
                        : _originReached
                        ? Colors.green.shade50
                        : Colors.orange.shade50,
                    child: Padding(
                      padding: const EdgeInsets.all(18),
                      child: Column(
                        children: [
                          Icon(
                            !_originCheckAttempted
                                ? Icons.save_rounded
                                : _originReached
                                ? Icons.check_circle
                                : Icons.warning_amber_rounded,
                            size: 56,
                            color: !_originCheckAttempted
                                ? Theme.of(context).colorScheme.primary
                                : _originReached
                                ? Colors.green
                                : Colors.orange,
                          ),
                          const SizedBox(height: 10),
                          Text(
                            !_originCheckAttempted
                                ? '원점 보정 저장 완료'
                                : _originReached
                                ? '원점 보정 완료'
                                : controller.isFeaturePreviewMode
                                ? '임시 저장 완료 · 원점 복귀 실패'
                                : 'Flash 저장 완료 · 원점 복귀 실패',
                            style: const TextStyle(
                              fontSize: 20,
                              fontWeight: FontWeight.bold,
                            ),
                            textAlign: TextAlign.center,
                          ),
                          const SizedBox(height: 6),
                          Text(
                            !_originCheckAttempted
                                ? '확인 시퀀스를 실행하거나 종료하세요.'
                                : _originReached
                                ? '원점 확인 완료'
                                : controller.isFeaturePreviewMode
                                ? '원점 확인 실패'
                                : '저장 완료 · 원점 확인 실패',
                            textAlign: TextAlign.center,
                          ),
                        ],
                      ),
                    ),
                  ),
                  const SizedBox(height: 10),
                  Card(
                    child: Padding(
                      padding: const EdgeInsets.all(14),
                      child: Column(
                        crossAxisAlignment: CrossAxisAlignment.start,
                        children: [
                          const Text(
                            '저장된 원점 보정값',
                            style: TextStyle(
                              fontSize: 18,
                              fontWeight: FontWeight.bold,
                            ),
                          ),
                          const SizedBox(height: 8),
                          ...List.generate(_jointNames.length, (joint) {
                            final values = _pulseValues[joint];
                            return ListTile(
                              dense: true,
                              contentPadding: EdgeInsets.zero,
                              title: Text(
                                _jointNames[joint],
                                style: const TextStyle(
                                  fontWeight: FontWeight.bold,
                                ),
                              ),
                              subtitle: Text(
                                joint == 5
                                    ? '열림 ${values[0]} µs  ·  닫힘 ${values[2]} µs'
                                    : '0° ${values[1]} µs  ·  '
                                          '-90° ${values[0]} µs  ·  '
                                          '+90° ${values[2]} µs',
                              ),
                            );
                          }),
                        ],
                      ),
                    ),
                  ),
                  const SizedBox(height: 12),
                  Column(
                    crossAxisAlignment: CrossAxisAlignment.stretch,
                    children: [
                      FilledButton.tonalIcon(
                        onPressed:
                            controller.isConnected &&
                                !controller.isEstopLatched &&
                                !controller.armCommandBusy
                            ? () => unawaited(_returnToOrigin(controller))
                            : null,
                        icon: const Icon(Icons.home),
                        label: Text(
                          !_originCheckAttempted
                              ? '원점 확인 시퀀스'
                              : _originReached
                              ? '확인 시퀀스 다시 실행'
                              : '확인 시퀀스 재시도',
                        ),
                      ),
                      const SizedBox(height: 8),
                      FilledButton.icon(
                        onPressed: () => unawaited(_closeScreen()),
                        icon: const Icon(Icons.check),
                        label: const Text('설정 종료'),
                      ),
                    ],
                  ),
                ],
              )
            : ListView(
                physics: const NeverScrollableScrollPhysics(),
                padding: const EdgeInsets.fromLTRB(12, 12, 12, 24),
                children: [
                  Card(
                    color: Theme.of(context).colorScheme.primaryContainer,
                    child: Padding(
                      padding: const EdgeInsets.all(16),
                      child: Column(
                        crossAxisAlignment: CrossAxisAlignment.start,
                        children: [
                          Text(
                            '1. 원점 보정',
                            style: const TextStyle(
                              fontSize: 22,
                              fontWeight: FontWeight.bold,
                            ),
                          ),
                          const SizedBox(height: 8),
                          const Text('관절을 선택한 뒤 아래에서 보정 위치와 펄스값을 함께 조정합니다.'),
                        ],
                      ),
                    ),
                  ),
                  const SizedBox(height: 10),
                  Card(
                    child: Padding(
                      padding: const EdgeInsets.all(12),
                      child: Column(
                        crossAxisAlignment: CrossAxisAlignment.start,
                        children: [
                          Text(
                            '관절 선택',
                            style: const TextStyle(fontWeight: FontWeight.bold),
                          ),
                          const SizedBox(height: 10),
                          Wrap(
                            spacing: 8,
                            runSpacing: 8,
                            children: List.generate(_jointNames.length, (
                              joint,
                            ) {
                              return ChoiceChip(
                                selected: joint == _currentJoint,
                                label: Text(_jointNames[joint]),
                                onSelected:
                                    controlsEnabled &&
                                        !controller.servoPreviewMoving
                                    ? (_) => unawaited(_selectJoint(joint))
                                    : null,
                              );
                            }),
                          ),
                        ],
                      ),
                    ),
                  ),
                  const SizedBox(height: 10),
                  const Card(
                    child: Padding(
                      padding: EdgeInsets.all(12),
                      child: Row(
                        crossAxisAlignment: CrossAxisAlignment.start,
                        children: [
                          Icon(Icons.warning_amber, color: Colors.orange),
                          SizedBox(width: 8),
                          Expanded(
                            child: Text(
                              '선택한 관절만 움직입니다. '
                              '걸림이나 떨림이 있으면 즉시 STOP을 누르세요.',
                            ),
                          ),
                        ],
                      ),
                    ),
                  ),
                  const SizedBox(height: 10),
                  Builder(
                    builder: (context) {
                      final int joint = _currentJoint;
                      final int minimum = _minimumPulse(joint);
                      final int maximum = _maximumPulse(joint);
                      final int value = _currentPulseUs[joint]
                          .clamp(minimum, maximum)
                          .toInt();
                      final String pointLabel = joint == 5
                          ? (_currentPoint == 0 ? '열림' : '닫힘')
                          : _pointLabels[_currentPoint];
                      final String targetLabel =
                          '${_jointNames[joint]} · $pointLabel';

                      return Padding(
                        padding: const EdgeInsets.only(bottom: 8),
                        child: Card(
                          child: Padding(
                            padding: const EdgeInsets.fromLTRB(12, 12, 12, 8),
                            child: Column(
                              crossAxisAlignment: CrossAxisAlignment.stretch,
                              children: [
                                Row(
                                  children: [
                                    Expanded(
                                      child: Text(
                                        targetLabel,
                                        style: const TextStyle(
                                          fontWeight: FontWeight.bold,
                                        ),
                                      ),
                                    ),
                                    _EditableValueBadge(
                                      valueName: targetLabel,
                                      value: value,
                                      minimum: minimum,
                                      maximum: maximum,
                                      unit: ' µs',
                                      enabled: controlsEnabled,
                                      textStyle: const TextStyle(
                                        fontSize: 16,
                                        fontWeight: FontWeight.bold,
                                      ),
                                      onApplied: (pulse) =>
                                          _applyCalibrationPulse(
                                            joint,
                                            pulse,
                                            immediately: true,
                                          ),
                                    ),
                                  ],
                                ),
                                const SizedBox(height: 8),
                                Wrap(
                                  spacing: 8,
                                  runSpacing: 8,
                                  children: selectablePoints.map((point) {
                                    final String label = joint == 5
                                        ? (point == 0 ? '열림' : '닫힘')
                                        : _pointLabels[point];
                                    return ChoiceChip(
                                      selected: point == _currentPoint,
                                      label: Text(label),
                                      onSelected:
                                          controlsEnabled &&
                                              !controller.servoPreviewMoving
                                          ? (_) =>
                                                _selectCalibrationPoint(point)
                                          : null,
                                    );
                                  }).toList(),
                                ),
                                _DirectAdjustSlider(
                                  minimum: minimum,
                                  maximum: maximum,
                                  value: value,
                                  enabled: controlsEnabled,
                                  adjustmentLabel: 'µs',
                                  onChanged: (pulse) {
                                    setState(() {
                                      _currentPulseUs[joint] = pulse;
                                      _pulseValues[joint][_currentPoint] =
                                          pulse;
                                      if (joint == 5) {
                                        _pulseValues[5][1] =
                                            (_pulseValues[5][0] +
                                                _pulseValues[5][2]) ~/
                                            2;
                                      }
                                      _calibrationDirty = true;
                                    });
                                    _schedulePreview(joint, pulse);
                                  },
                                  onChangeEnd: (pulse) =>
                                      _previewImmediately(joint, pulse),
                                ),
                                Text(
                                  '$minimum~$maximum µs · 드래그 또는 −/+ · 1 µs 단위',
                                  textAlign: TextAlign.center,
                                  style: Theme.of(context).textTheme.bodySmall,
                                ),
                                if (controller.servoPreviewMoving) ...[
                                  const SizedBox(height: 8),
                                  const Row(
                                    mainAxisAlignment: MainAxisAlignment.center,
                                    children: [
                                      SizedBox(
                                        width: 16,
                                        height: 16,
                                        child: CircularProgressIndicator(
                                          strokeWidth: 2,
                                        ),
                                      ),
                                      SizedBox(width: 8),
                                      Text('목표 펄스로 부드럽게 이동 중'),
                                    ],
                                  ),
                                ],
                              ],
                            ),
                          ),
                        ),
                      );
                    },
                  ),
                  const SizedBox(height: 4),
                  Row(
                    children: [
                      Expanded(
                        child: OutlinedButton.icon(
                          onPressed:
                              controlsEnabled && !controller.servoPreviewMoving
                              ? () => unawaited(_returnToOrigin(controller))
                              : null,
                          icon: const Icon(Icons.fact_check_outlined),
                          label: const Text('확인 시퀀스'),
                        ),
                      ),
                      const SizedBox(width: 8),
                      Expanded(
                        child: FilledButton.icon(
                          onPressed:
                              controlsEnabled && !controller.servoPreviewMoving
                              ? () => unawaited(_saveCalibrationOnly())
                              : null,
                          icon: _saving
                              ? const SizedBox(
                                  width: 18,
                                  height: 18,
                                  child: CircularProgressIndicator(
                                    strokeWidth: 2,
                                  ),
                                )
                              : const Icon(Icons.save),
                          label: Text(
                            _saving
                                ? '원점 저장 중'
                                : _calibrationDirty
                                ? '원점 보정 저장 *'
                                : '원점 보정 저장',
                          ),
                        ),
                      ),
                    ],
                  ),
                ],
              ),
      ),
    );
  }
}

/// 로봇팔 화면에서 현재 자세를 빠르게 웨이포인트로 저장하는 간단 편집기다.
class TeachingEditor extends StatelessWidget {
  const TeachingEditor({required this.compact, super.key});

  final bool compact;

  @override
  Widget build(BuildContext context) {
    final teaching = Provider.of<TeachingController>(context);
    final bluetooth = Provider.of<BluetoothController>(context);
    final List<List<int>> waypoints = teaching.currentWaypoints;

    return Card(
      elevation: 2,
      child: Padding(
        padding: const EdgeInsets.all(8.0),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.stretch,
          children: [
            const Text(
              '티칭 자세 준비',
              style: TextStyle(
                fontWeight: FontWeight.bold,
                color: Colors.blueGrey,
              ),
            ),
            const SizedBox(height: 4),
            DropdownButton<int>(
              isExpanded: true,
              value: teaching.selectedSequence,
              items: List.generate(
                TeachingController.sequenceCount,
                (index) => DropdownMenuItem<int>(
                  value: index + 1,
                  child: _SequenceOptionLabel(
                    teaching: teaching,
                    sequence: index + 1,
                  ),
                ),
              ),
              onChanged: teaching.isBusy
                  ? null
                  : (value) async {
                      if (value != null) {
                        final String? error = await teaching
                            .selectSequenceFromSTM32(bluetooth, value);
                        if (context.mounted && error != null) {
                          showAppMessage(
                            context,
                            error,
                            backgroundColor: Colors.redAccent,
                          );
                        }
                      }
                    },
            ),
            Row(
              children: [
                Expanded(
                  child: Text(
                    '${teaching.selectedSequenceName} '
                    '(${waypoints.length}/${TeachingController.maxWaypoints})',
                    style: const TextStyle(fontWeight: FontWeight.bold),
                  ),
                ),
                FilledButton.tonalIcon(
                  onPressed:
                      teaching.isBusy ||
                          !bluetooth.isArmEnabled ||
                          !bluetooth.armPoseKnown ||
                          bluetooth.armCommandBusy
                      ? null
                      : () {
                          final bool added = teaching.addCurrentPose(
                            RobotArmControlTab.currentAngles,
                          );
                          if (!added) {
                            ScaffoldMessenger.of(context).showSnackBar(
                              const SnackBar(
                                content: Text('웨이포인트는 최대 30개까지 저장할 수 있습니다.'),
                              ),
                            );
                          }
                        },
                  icon: const Icon(Icons.add),
                  label: const Text('현재 자세 추가'),
                ),
              ],
            ),
            const Divider(),
            Expanded(
              child: waypoints.isEmpty
                  ? const Center(
                      child: Text(
                        '추가된 자세가 없습니다.\n'
                        '로봇팔을 조절한 뒤 현재 자세 추가를 누르세요.',
                        textAlign: TextAlign.center,
                      ),
                    )
                  : ListView.builder(
                      itemCount: waypoints.length,
                      itemBuilder: (context, index) {
                        final List<int> waypoint = waypoints[index];
                        return ListTile(
                          dense: true,
                          contentPadding: const EdgeInsets.only(left: 8),
                          title: Text('웨이포인트 #${index + 1}'),
                          subtitle: Text(
                            RobotArmControlTab.formatWaypoint(waypoint),
                            maxLines: compact ? 1 : 2,
                            overflow: TextOverflow.ellipsis,
                          ),
                          trailing: IconButton(
                            tooltip: '삭제',
                            onPressed: teaching.isBusy
                                ? null
                                : () => teaching.removeWaypoint(index),
                            icon: const Icon(
                              Icons.delete_outline,
                              color: Colors.redAccent,
                            ),
                          ),
                        );
                      },
                    ),
            ),
          ],
        ),
      ),
    );
  }
}

// ======================================================
// 티칭 시스템 화면
// ======================================================

/// 웨이포인트 관절값 입력창이 TextEditingController의 생명주기를 직접 관리한다.
///
/// 다이얼로그를 닫은 직후 부모가 갱신되어도 입력 필드가 제거되기 전까지
/// 컨트롤러가 살아 있으므로 Flutter 의존 위젯 해제 오류가 발생하지 않는다.
class _WaypointAngleEditorDialog extends StatefulWidget {
  const _WaypointAngleEditorDialog({
    required this.initialPose,
    this.waypointIndex,
  });

  final List<int> initialPose;
  final int? waypointIndex;

  @override
  State<_WaypointAngleEditorDialog> createState() =>
      _WaypointAngleEditorDialogState();
}

class _WaypointAngleEditorDialogState
    extends State<_WaypointAngleEditorDialog> {
  static const List<String> _jointNames = RobotArmControlTab.jointLabels;

  late final List<TextEditingController> _fields;
  String? _errorText;

  @override
  void initState() {
    super.initState();
    _fields = List<TextEditingController>.generate(
      RobotArmControlTab.jointCount,
      (index) => TextEditingController(
        text: RobotArmControlTab.packetToDisplayValue(
          index,
          widget.initialPose[index],
        ).toString(),
      ),
    );
  }

  @override
  void dispose() {
    for (final field in _fields) {
      field.dispose();
    }
    super.dispose();
  }

  /// 입력 범위를 검사하고 Bluetooth 패킷에서 사용하는 값으로 변환해 반환한다.
  void _save() {
    final packetPose = <int>[];
    for (int index = 0; index < RobotArmControlTab.jointCount; index++) {
      final int? value = int.tryParse(_fields[index].text.trim());
      final bool gripper = index == RobotArmControlTab.gripperIndex;
      final bool valid =
          value != null &&
          (gripper ? value >= 0 && value <= 100 : value >= -90 && value <= 90);
      if (!valid) {
        setState(() => _errorText = '각 관절의 입력 범위를 확인해주세요.');
        return;
      }
      packetPose.add(
        RobotArmControlTab.displayToPacketValue(index, value.toDouble()),
      );
    }
    Navigator.pop(context, packetPose);
  }

  @override
  Widget build(BuildContext context) {
    final int? waypointIndex = widget.waypointIndex;
    return AlertDialog(
      title: Text(
        waypointIndex == null ? '자세 각도 입력' : '자세 ${waypointIndex + 1} 수정',
      ),
      content: SizedBox(
        width: double.maxFinite,
        child: SingleChildScrollView(
          child: Column(
            mainAxisSize: MainAxisSize.min,
            children: [
              const Text('관절은 -90~+90°, Gripper는 0~100%로 입력합니다.'),
              const SizedBox(height: 12),
              ...List.generate(RobotArmControlTab.jointCount, (index) {
                final bool gripper = index == RobotArmControlTab.gripperIndex;
                return Padding(
                  padding: const EdgeInsets.only(bottom: 8),
                  child: TextField(
                    controller: _fields[index],
                    keyboardType: TextInputType.numberWithOptions(
                      signed: !gripper,
                    ),
                    decoration: InputDecoration(
                      labelText: gripper
                          ? '${_jointNames[index]} (%)'
                          : '${_jointNames[index]} (°)',
                      border: const OutlineInputBorder(),
                    ),
                  ),
                );
              }),
              if (_errorText != null)
                Text(_errorText!, style: const TextStyle(color: Colors.red)),
            ],
          ),
        ),
      ),
      actions: [
        TextButton(
          onPressed: () => Navigator.pop(context),
          child: const Text('취소'),
        ),
        FilledButton(onPressed: _save, child: const Text('자세 저장')),
      ],
    );
  }
}

/// 웨이포인트 편집, 편집본 재생과 STM32 Flash 저장·초기화를 제공한다.
class TeachingTab extends StatefulWidget {
  const TeachingTab({super.key});

  @override
  State<TeachingTab> createState() => _TeachingTabState();
}

class _TeachingTabState extends State<TeachingTab> {
  /// 선택한 Flash 시퀀스를 재생하고 펌웨어의 완료 결과를 표시한다.
  Future<void> _playOnSTM32(
    BluetoothController bluetooth,
    TeachingController teaching,
  ) async {
    final String? errorMessage = await teaching.playOnSTM32(bluetooth);
    if (mounted && errorMessage != null) {
      ScaffoldMessenger.of(
        context,
      ).showSnackBar(SnackBar(content: Text(errorMessage)));
    }
  }

  /// 앱 메모리의 현재 시퀀스를 STM32에 업로드하고 결과를 표시한다.
  Future<void> _uploadToSTM32(BluetoothController blt) async {
    final teaching = Provider.of<TeachingController>(context, listen: false);
    final String? errorMessage = await teaching.uploadToSTM32(blt);
    if (!mounted || errorMessage == null) {
      return;
    }
    ScaffoldMessenger.of(
      context,
    ).showSnackBar(SnackBar(content: Text(errorMessage)));
  }

  /// 사용자 확인 후 선택 시퀀스를 STM32와 앱 메모리에서 초기화한다.
  Future<void> _resetOnSTM32(
    BluetoothController bluetooth,
    TeachingController teaching,
  ) async {
    final bool? confirmed = await showDialog<bool>(
      context: context,
      builder: (dialogContext) => AlertDialog(
        title: const Text('시퀀스 초기화'),
        content: Text(
          bluetooth.isFeaturePreviewMode
              ? '${teaching.selectedSequenceName} TEST 데이터를 지울까요?'
              : '${teaching.selectedSequenceName} 데이터를 모두 지울까요?',
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.pop(dialogContext, false),
            child: const Text('취소'),
          ),
          FilledButton(
            onPressed: () => Navigator.pop(dialogContext, true),
            child: const Text('초기화'),
          ),
        ],
      ),
    );
    if (confirmed != true || !mounted) {
      return;
    }

    final String? errorMessage = await teaching.resetOnSTM32(bluetooth);
    if (!mounted || errorMessage == null) {
      return;
    }
    ScaffoldMessenger.of(
      context,
    ).showSnackBar(SnackBar(content: Text(errorMessage)));
  }

  /// 1~8번 이름만 앱 RAM에서 바꾸고 다음 Flash 업로드에 함께 저장한다.
  Future<void> _renameSequence(TeachingController teaching) async {
    if (TeachingController.isReservedSequence(teaching.selectedSequence)) {
      return;
    }
    String editedName = teaching.selectedSequenceName;
    final String? name = await showDialog<String>(
      context: context,
      builder: (dialogContext) => AlertDialog(
        title: const Text('시퀀스 이름'),
        content: TextFormField(
          initialValue: editedName,
          onChanged: (value) => editedName = value,
          autofocus: true,
          inputFormatters: [
            FilteringTextInputFormatter.deny(RegExp(r'[\r\n]')),
          ],
          decoration: const InputDecoration(
            labelText: '동작 이름',
            helperText: 'UTF-8 기준 최대 21바이트',
          ),
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.pop(dialogContext),
            child: const Text('취소'),
          ),
          FilledButton(
            onPressed: () => Navigator.pop(dialogContext, editedName),
            child: const Text('적용'),
          ),
        ],
      ),
    );
    if (name == null || !mounted) {
      return;
    }
    final String? error = teaching.renameSelectedSequence(name);
    showAppMessage(
      context,
      error ?? '이름을 변경했습니다. Flash 저장 시 웨이포인트와 함께 저장됩니다.',
      backgroundColor: error == null ? null : Colors.redAccent,
    );
  }

  /// 새 웨이포인트를 만들거나 기존 웨이포인트의 관절 값을 수정한다.
  Future<void> _showAngleEditor(
    TeachingController teaching, {
    int? waypointIndex,
  }) async {
    final List<int> initial = waypointIndex == null
        ? List<int>.from(RobotArmControlTab.currentAngles)
        : List<int>.from(teaching.currentWaypoints[waypointIndex]);

    final List<int>? pose = await showDialog<List<int>>(
      context: context,
      builder: (_) => _WaypointAngleEditorDialog(
        initialPose: initial,
        waypointIndex: waypointIndex,
      ),
    );

    if (pose == null || !mounted) {
      return;
    }
    final bool saved = waypointIndex == null
        ? teaching.addCurrentPose(pose)
        : teaching.updateWaypoint(waypointIndex, pose);
    if (!saved) {
      ScaffoldMessenger.of(
        context,
      ).showSnackBar(const SnackBar(content: Text('웨이포인트를 저장하지 못했습니다.')));
    }
  }

  @override
  Widget build(BuildContext context) {
    final bltController = Provider.of<BluetoothController>(context);
    final teaching = Provider.of<TeachingController>(context);
    final List<List<int>> currentWaypoints = teaching.currentWaypoints;

    return ListView(
      padding: const EdgeInsets.fromLTRB(8, 4, 8, 20),
      children: [
        Card(
          child: Padding(
            padding: const EdgeInsets.all(12),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.stretch,
              children: [
                Container(
                  padding: const EdgeInsets.all(10),
                  decoration: BoxDecoration(
                    color: Theme.of(context).colorScheme.secondaryContainer,
                    borderRadius: BorderRadius.circular(10),
                  ),
                  child: const Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Text(
                        '티칭 사용 순서',
                        style: TextStyle(fontWeight: FontWeight.bold),
                      ),
                      SizedBox(height: 3),
                      Text(
                        '1  자세 추가  →  2  재생 확인  →  3  기기에 저장',
                        style: TextStyle(fontSize: 12),
                      ),
                    ],
                  ),
                ),
                const SizedBox(height: 12),
                const Text(
                  '1. 시퀀스 선택',
                  style: TextStyle(
                    color: Colors.blueGrey,
                    fontWeight: FontWeight.bold,
                  ),
                ),
                const SizedBox(height: 6),
                Row(
                  children: [
                    Expanded(
                      child: DropdownButtonFormField<int>(
                        isExpanded: true,
                        initialValue: teaching.selectedSequence,
                        decoration: const InputDecoration(
                          border: OutlineInputBorder(),
                          contentPadding: EdgeInsets.symmetric(
                            horizontal: 12,
                            vertical: 10,
                          ),
                        ),
                        items: List.generate(
                          TeachingController.sequenceCount,
                          (index) => DropdownMenuItem<int>(
                            value: index + 1,
                            child: _SequenceOptionLabel(
                              teaching: teaching,
                              sequence: index + 1,
                            ),
                          ),
                        ),
                        onChanged: teaching.isBusy
                            ? null
                            : (value) async {
                                if (value != null) {
                                  final String? error = await teaching
                                      .selectSequenceFromSTM32(
                                        bltController,
                                        value,
                                      );
                                  if (context.mounted && error != null) {
                                    showAppMessage(
                                      context,
                                      error,
                                      backgroundColor: Colors.redAccent,
                                    );
                                  }
                                }
                              },
                      ),
                    ),
                    const SizedBox(width: 8),
                    OutlinedButton.icon(
                      onPressed:
                          teaching.isBusy ||
                              TeachingController.isReservedSequence(
                                teaching.selectedSequence,
                              )
                          ? null
                          : () => unawaited(_renameSequence(teaching)),
                      icon: Icon(
                        TeachingController.isReservedSequence(
                              teaching.selectedSequence,
                            )
                            ? Icons.lock
                            : Icons.edit_outlined,
                      ),
                      label: Text(
                        TeachingController.isReservedSequence(
                              teaching.selectedSequence,
                            )
                            ? '고정'
                            : '이름',
                      ),
                    ),
                  ],
                ),
                if (TeachingController.isReservedSequence(
                  teaching.selectedSequence,
                )) ...[
                  const SizedBox(height: 6),
                  const Text(
                    '차량 동작에 사용하는 고정 시퀀스입니다. 이름을 변경할 수 없습니다.',
                    style: TextStyle(color: Colors.blueGrey, fontSize: 12),
                  ),
                ],
                const SizedBox(height: 10),
                FilledButton.icon(
                  style: FilledButton.styleFrom(backgroundColor: Colors.green),
                  onPressed: (bltController.isArmEnabled && !teaching.isBusy)
                      ? () => unawaited(_playOnSTM32(bltController, teaching))
                      : null,
                  icon: teaching.isPlaying
                      ? const SizedBox(
                          width: 18,
                          height: 18,
                          child: CircularProgressIndicator(strokeWidth: 2),
                        )
                      : const Icon(Icons.play_arrow),
                  label: Text(
                    teaching.isPlaying ? '현재 편집본 재생 중...' : '2. 현재 편집본 재생',
                  ),
                ),
                const Padding(
                  padding: EdgeInsets.only(top: 5),
                  child: Text(
                    '현재 편집본을 Flash에 저장하지 않고 바로 재생합니다.',
                    style: TextStyle(color: Colors.blueGrey, fontSize: 12),
                    textAlign: TextAlign.center,
                  ),
                ),
                const SizedBox(height: 8),
                FilledButton.tonalIcon(
                  onPressed: teaching.isBusy
                      ? null
                      : () => unawaited(_uploadToSTM32(bltController)),
                  icon: teaching.isUploading
                      ? const SizedBox(
                          width: 18,
                          height: 18,
                          child: CircularProgressIndicator(strokeWidth: 2),
                        )
                      : const Icon(Icons.upload_file),
                  label: Text(
                    teaching.isUploading
                        ? bltController.isFeaturePreviewMode
                              ? '저장 기능 확인 중...'
                              : '기기에 저장 중...'
                        : bltController.isFeaturePreviewMode
                        ? '3. 저장 기능 확인'
                        : '3. 기기에 저장',
                  ),
                ),
                const SizedBox(height: 8),
                OutlinedButton.icon(
                  style: OutlinedButton.styleFrom(
                    foregroundColor: Colors.redAccent,
                    side: const BorderSide(color: Colors.redAccent),
                  ),
                  onPressed: teaching.isBusy
                      ? null
                      : () => unawaited(_resetOnSTM32(bltController, teaching)),
                  icon: teaching.isResetting
                      ? const SizedBox(
                          width: 18,
                          height: 18,
                          child: CircularProgressIndicator(strokeWidth: 2),
                        )
                      : const Icon(Icons.delete_outline),
                  label: Text(
                    teaching.isResetting
                        ? bltController.isFeaturePreviewMode
                              ? '초기화 확인 중...'
                              : 'Flash 초기화 중...'
                        : '저장된 시퀀스 초기화',
                  ),
                ),
                if ((teaching.isUploading || teaching.isResetting) &&
                    !bltController.isFeaturePreviewMode)
                  const Padding(
                    padding: EdgeInsets.only(top: 8),
                    child: Text(
                      '완료 안내 전에는 앱을 종료하거나 로봇 전원을 끄지 마세요.',
                      style: TextStyle(color: Colors.orange),
                      textAlign: TextAlign.center,
                    ),
                  ),
              ],
            ),
          ),
        ),
        const SizedBox(height: 8),
        Card(
          child: Padding(
            padding: const EdgeInsets.all(10),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.stretch,
              children: [
                Text(
                  '자세 만들기 · ${teaching.selectedSequenceName} '
                  '(${currentWaypoints.length}/${TeachingController.maxWaypoints})',
                  style: const TextStyle(fontWeight: FontWeight.bold),
                ),
                const SizedBox(height: 8),
                Wrap(
                  spacing: 8,
                  runSpacing: 8,
                  children: [
                    FilledButton.tonalIcon(
                      onPressed:
                          teaching.isBusy ||
                              !bltController.isArmEnabled ||
                              !bltController.armPoseKnown ||
                              bltController.armCommandBusy
                          ? null
                          : () {
                              final bool added = teaching.addCurrentPose(
                                RobotArmControlTab.currentAngles,
                              );
                              if (!added) {
                                ScaffoldMessenger.of(context).showSnackBar(
                                  const SnackBar(
                                    content: Text(
                                      '웨이포인트는 최대 30개까지 저장할 수 있습니다.',
                                    ),
                                  ),
                                );
                              }
                            },
                      icon: const Icon(Icons.add),
                      label: Text(
                        bltController.armPoseKnown ? '현재 자세 추가' : '자세 확인 대기',
                      ),
                    ),
                    OutlinedButton.icon(
                      onPressed: teaching.isBusy
                          ? null
                          : () => unawaited(_showAngleEditor(teaching)),
                      icon: const Icon(Icons.edit_note),
                      label: const Text('각도 직접 입력'),
                    ),
                  ],
                ),
                const Divider(),
                if (currentWaypoints.isEmpty)
                  const Padding(
                    padding: EdgeInsets.symmetric(vertical: 36),
                    child: Text(
                      '아직 추가한 자세가 없습니다.\n'
                      '로봇팔을 원하는 모양으로 조절한 뒤 현재 자세 추가를 누르세요.',
                      textAlign: TextAlign.center,
                    ),
                  )
                else
                  ListView.separated(
                    shrinkWrap: true,
                    physics: const NeverScrollableScrollPhysics(),
                    itemCount: currentWaypoints.length,
                    separatorBuilder: (context, index) =>
                        const Divider(height: 1),
                    itemBuilder: (context, index) {
                      final List<int> waypoint = currentWaypoints[index];
                      return ListTile(
                        contentPadding: const EdgeInsets.only(left: 4),
                        title: Text(
                          '자세 ${index + 1}',
                          style: const TextStyle(fontWeight: FontWeight.bold),
                        ),
                        subtitle: Text(
                          RobotArmControlTab.formatWaypoint(waypoint),
                        ),
                        trailing: Row(
                          mainAxisSize: MainAxisSize.min,
                          children: [
                            IconButton(
                              tooltip: '각도 수정',
                              onPressed: teaching.isBusy
                                  ? null
                                  : () => unawaited(
                                      _showAngleEditor(
                                        teaching,
                                        waypointIndex: index,
                                      ),
                                    ),
                              icon: const Icon(Icons.edit_outlined),
                            ),
                            IconButton(
                              tooltip: '삭제',
                              onPressed: teaching.isBusy
                                  ? null
                                  : () => teaching.removeWaypoint(index),
                              icon: const Icon(
                                Icons.delete_outline,
                                color: Colors.redAccent,
                              ),
                            ),
                          ],
                        ),
                      );
                    },
                  ),
              ],
            ),
          ),
        ),
      ],
    );
  }
}
