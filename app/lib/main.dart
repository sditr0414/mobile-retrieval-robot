import 'dart:async';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_joystick/flutter_joystick.dart';
import 'package:flutter_classic_bluetooth/flutter_classic_bluetooth.dart';
import 'package:provider/provider.dart';

void main() {
  WidgetsFlutterBinding.ensureInitialized();
  SystemChrome.setPreferredOrientations([
    DeviceOrientation.landscapeLeft,
    DeviceOrientation.landscapeRight,
  ]).then((_) {
    runApp(
      ChangeNotifierProvider(
        create: (context) => BluetoothController(),
        child: const RobotApp(),
      ),
    );
  });
}

class RobotApp extends StatelessWidget {
  const RobotApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'RC & Robot Controller',
      debugShowCheckedModeBanner: false,
      theme: ThemeData(
        primarySwatch: Colors.blueGrey,
        useMaterial3: true,
      ),
      home: const MainScreen(),
    );
  }
}

// ======================================================
// [핵심] 블루투스 상태 관리자
// ======================================================
class _PendingPacket {
  _PendingPacket(this.bytes);

  final List<int> bytes;
  final Completer<void> completer = Completer<void>();
}

class BluetoothController with ChangeNotifier {
  final FlutterClassicBluetooth _bluetooth = FlutterClassicBluetooth();

  BtcConnection? connection;
  List<BtcDevice> bondedDevices = [];
  bool isConnecting = false;
  bool isEstopLatched = true;
  bool isArmEnabled = false;
  String? lastAckMessage;
  bool lastAckSucceeded = true;
  final List<int> _receiveBuffer = [];
  final List<_PendingPacket> _writeQueue = [];
  bool _isWriting = false;
  Completer<bool>? _teachingAckCompleter;
  int? _teachingAckSequence;

  bool get isConnected => connection != null;

  Future<void> loadBondedDevices() async {
    try {
      bondedDevices = await _bluetooth.getPairedDevices();
      notifyListeners();
    } catch (e) {
      debugPrint("페어링 기기 로드 실패: $e");
    }
  }

  Future<void> connectToDevice(BtcDevice device) async {
    isConnecting = true;
    notifyListeners();

    try {
      connection = await _bluetooth.connect(address: device.address);
      debugPrint("✅ HC-05 연결 성공: ${device.name ?? 'Unknown'} (${device.address})");
      isConnecting = false;
      isEstopLatched = true;
      isArmEnabled = false;
      notifyListeners();

      connection?.input.listen((Uint8List data) {
        _handleIncomingData(data);
      }, onDone: () {
        debugPrint("원격 기기와의 연결이 종료되었습니다.");
        connection = null;
        isEstopLatched = true;
        isArmEnabled = false;
        notifyListeners();
      });
    } catch (e) {
      debugPrint("연결 에러: $e");
      connection = null;
      isConnecting = false;
      notifyListeners();
    }
  }

  void _handleIncomingData(Uint8List data) {
    _receiveBuffer.addAll(data);

    // Bluetooth 수신 조각을 모아 11바이트 ACK 한 개씩 꺼낸다.
    while (_receiveBuffer.isNotEmpty) {
      final int headerIndex = _receiveBuffer.indexOf(0xAA);
      if (headerIndex < 0) {
        _receiveBuffer.clear();
        return;
      }
      if (headerIndex > 0) {
        _receiveBuffer.removeRange(0, headerIndex);
      }
      if (_receiveBuffer.length < 11) {
        return;
      }

      final List<int> packet = _receiveBuffer.sublist(0, 11);
      final int checksum =
          packet.sublist(1, 9).fold(0, (sum, value) => sum + value) & 0xFF;
      if (packet[10] != 0x55 || packet[9] != checksum) {
        _receiveBuffer.removeAt(0);
        continue;
      }

      _receiveBuffer.removeRange(0, 11);
      if (packet[1] == 2 && packet[2] == 4) {
        final bool succeeded = packet[4] == 1;
        lastAckSucceeded = succeeded;
        lastAckMessage = succeeded
            ? "시퀀스 ${packet[3]}번 플래시 메모리 저장 성공!"
            : "시퀀스 ${packet[3]}번 저장 실패";

        if ((_teachingAckSequence == packet[3]) &&
            (_teachingAckCompleter?.isCompleted == false)) {
          _teachingAckCompleter!.complete(succeeded);
        }
        notifyListeners();
      } else if (packet[1] == 1) {
        final int command = packet[2];
        final bool succeeded = packet[3] == 1;
        final int reason = packet[4];
        lastAckSucceeded = succeeded;

        if (command == 1) {
          isArmEnabled = succeeded;
          lastAckMessage = succeeded
              ? "로봇팔 출력이 활성화되었습니다."
              : "로봇팔 활성화 실패: ${_armFailureMessage(reason)}";
        } else if (command == 2) {
          isArmEnabled = false;
          lastAckMessage = succeeded
              ? "로봇팔 출력을 차단했습니다."
              : "로봇팔 출력 차단에 실패했습니다.";
        } else if (!succeeded) {
          lastAckMessage = "로봇팔 이동 실패: ${_armFailureMessage(reason)}";
        }
        notifyListeners();
      }
    }
  }

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

  void clearAckMessage() {
    lastAckMessage = null;
    lastAckSucceeded = true;
  }

  Future<void> disconnect() async {
    final BtcConnection? activeConnection = connection;
    if (activeConnection == null) {
      return;
    }

    // 정상 연결 해제 전에는 차량과 로봇팔을 먼저 안전 상태로 만든다.
    await emergencyStop();
    connection = null;
    isArmEnabled = false;
    try {
      await activeConnection.finish();
    } catch (error) {
      debugPrint("Bluetooth 연결 종료 오류: $error");
    }
    notifyListeners();
  }

  Future<void> sendPacket(
    List<int> packet, {
    bool urgent = false,
    bool replacePending = false,
  }) {
    if (packet.length != 11) {
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
          queued.completer.completeError(
            StateError("긴급 정지로 대기 패킷이 취소되었습니다."),
          );
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
        }
      }
    } finally {
      _isWriting = false;
    }
  }

  Future<void> sendCommand(
    int mode, {
    List<int> data = const [0, 0, 0, 0, 0, 0, 0],
    bool urgent = false,
    bool replacePending = false,
  }) {
    List<int> payload = List.filled(7, 0);
    for (int i = 0; i < data.length && i < 7; i++) {
      payload[i] = data[i];
    }

    int sum = mode;
    for (int value in payload) {
      sum += value;
    }
    int checksum = sum % 256;

    List<int> packet = [
      0xAA,
      mode,
      ...payload,
      checksum,
      0x55
    ];

    return sendPacket(
      packet,
      urgent: urgent,
      replacePending: replacePending,
    );
  }

  Future<void> sendDrive(
      int leftDirection, int leftPwm, int rightDirection, int rightPwm) {
    if (!isConnected || isEstopLatched) {
      return Future<void>.value();
    }
    return sendCommand(
      0,
      data: [leftDirection, leftPwm, rightDirection, rightPwm, 0],
      replacePending: true,
    );
  }

  Future<void> emergencyStop() async {
    isEstopLatched = true;
    isArmEnabled = false;
    notifyListeners();

    // Control=1은 펌웨어에서 차량 정지 잠금과 로봇팔 출력을 함께 끈다.
    try {
      await sendCommand(
        0,
        data: [0, 0, 0, 0, 1],
        urgent: true,
      );
    } catch (error) {
      debugPrint("비상정지 패킷 전송 실패: $error");
    }
  }

  Future<void> clearEmergencyStop() async {
    if (!isConnected) {
      return;
    }

    await sendCommand(0, data: [0, 0, 0, 0, 2]);
    isEstopLatched = false;
    notifyListeners();
  }

  Future<void> enableArm(List<int> homeAngles) {
    if (!isConnected || isEstopLatched) {
      return Future<void>.value();
    }
    return sendCommand(1, data: [...homeAngles, 1]);
  }

  Future<void> disableArm() async {
    await sendCommand(1, data: [0, 0, 0, 0, 0, 0, 2]);
    isArmEnabled = false;
    notifyListeners();
  }

  Future<void> sendArmPose(List<int> angles) {
    if (!isConnected || !isArmEnabled || isEstopLatched) {
      return Future<void>.value();
    }
    return sendCommand(
      1,
      data: [...angles, 0],
      replacePending: true,
    );
  }

  Future<bool> waitForTeachingAck(int sequenceId) {
    if (_teachingAckCompleter?.isCompleted == false) {
      _teachingAckCompleter!.complete(false);
    }

    final Completer<bool> completer = Completer<bool>();
    _teachingAckCompleter = completer;
    _teachingAckSequence = sequenceId;

    return completer.future.timeout(
      const Duration(seconds: 5),
      onTimeout: () => false,
    ).whenComplete(() {
      if (_teachingAckCompleter == completer) {
        _teachingAckCompleter = null;
        _teachingAckSequence = null;
      }
    });
  }
}

// ======================================================
// 1. 메인 뼈대 화면
// ======================================================
class MainScreen extends StatefulWidget {
  const MainScreen({super.key});

  @override
  State<MainScreen> createState() => _MainScreenState();
}

class _MainScreenState extends State<MainScreen> {
  int _currentIndex = 0;

  final List<Widget> _screens = [
    const JoystickTab(),
    const RobotArmControlTab(),
    const TeachingTab(),
  ];

  final List<String> _screenTitles = [
    'RC카 주행',
    '로봇팔 조종',
    '티칭 시스템',
  ];

  Future<void> _confirmEstopClear(BluetoothController controller) async {
    final bool? confirmed = await showDialog<bool>(
      context: context,
      builder: (context) => AlertDialog(
        title: const Text('비상정지 해제'),
        content: const Text('주변에 사람이 없고 로봇이 안전한 상태인지 확인했나요?'),
        actions: [
          TextButton(
            onPressed: () => Navigator.pop(context, false),
            child: const Text('취소'),
          ),
          FilledButton(
            onPressed: () => Navigator.pop(context, true),
            child: const Text('확인 후 해제'),
          ),
        ],
      ),
    );

    if (confirmed == true) {
      try {
        await controller.clearEmergencyStop();
      } catch (error) {
        if (mounted) {
          ScaffoldMessenger.of(context).showSnackBar(
            SnackBar(content: Text('비상정지 해제 전송 실패: $error')),
          );
        }
      }
    }
  }

  void _showBluetoothDialog(BuildContext context) {
    final bltController = Provider.of<BluetoothController>(context, listen: false);
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
                if (controller.bondedDevices.isEmpty) {
                  return const Center(
                    child: Text('페어링된 기기가 없습니다.\n기기 설정에서 HC-05를 먼저 페어링해주세요.',
                        textAlign: TextAlign.center),
                  );
                }
                return ListView.builder(
                  itemCount: controller.bondedDevices.length,
                  itemBuilder: (context, index) {
                    final device = controller.bondedDevices[index];
                    return ListTile(
                      title: Text(device.name ?? '알 수 없는 기기', style: const TextStyle(fontWeight: FontWeight.bold)),
                      subtitle: Text(device.address),
                      trailing: ElevatedButton(
                        onPressed: controller.isConnecting
                            ? null
                            : () {
                          unawaited(controller.connectToDevice(device));
                          Navigator.pop(context);
                        },
                        child: controller.isConnecting
                            ? const SizedBox(width: 16, height: 16, child: CircularProgressIndicator(strokeWidth: 2))
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

  @override
  Widget build(BuildContext context) {
    final bltController = Provider.of<BluetoothController>(context);
    bool isConnected = bltController.connection != null;

    final String? ackMessage = bltController.lastAckMessage;
    final bool ackSucceeded = bltController.lastAckSucceeded;
    if (ackMessage != null) {
      WidgetsBinding.instance.addPostFrameCallback((_) {
        if (!mounted) {
          return;
        }
        ScaffoldMessenger.of(context).showSnackBar(
          SnackBar(
            content: Text(ackMessage),
            backgroundColor: ackSucceeded ? Colors.green : Colors.red,
            duration: const Duration(seconds: 2),
          ),
        );
        bltController.clearAckMessage();
      });
    }

    return Scaffold(
      appBar: AppBar(
        title: Text('${_screenTitles[_currentIndex]} 제어', style: const TextStyle(fontWeight: FontWeight.bold, fontSize: 18)),
        backgroundColor: Colors.blueGrey[900],
        foregroundColor: Colors.white,
        actions: [
          Padding(
            padding: const EdgeInsets.symmetric(vertical: 8.0),
            child: ElevatedButton.icon(
              style: ElevatedButton.styleFrom(
                backgroundColor: isConnected ? Colors.blueAccent : Colors.grey[700],
                foregroundColor: Colors.white,
              ),
              onPressed: () {
                if (isConnected) {
                  unawaited(bltController.disconnect());
                } else {
                  _showBluetoothDialog(context);
                }
              },
              icon: Icon(isConnected ? Icons.bluetooth_connected : Icons.bluetooth_searching),
              label: Text(isConnected ? '연결됨' : '기기 찾기', style: const TextStyle(fontWeight: FontWeight.bold)),
            ),
          ),
          const SizedBox(width: 10),
          if (bltController.isEstopLatched)
            Padding(
              padding: const EdgeInsets.symmetric(vertical: 8.0),
              child: ElevatedButton.icon(
                style: ElevatedButton.styleFrom(
                  backgroundColor: Colors.orange,
                  foregroundColor: Colors.black,
                ),
                onPressed: isConnected
                    ? () => _confirmEstopClear(bltController)
                    : null,
                icon: const Icon(Icons.lock_open),
                label: const Text(
                  '정지 해제',
                  style: TextStyle(fontWeight: FontWeight.bold),
                ),
              ),
            ),
          Padding(
            padding: const EdgeInsets.symmetric(vertical: 8.0, horizontal: 10.0),
            child: ElevatedButton.icon(
              style: ElevatedButton.styleFrom(
                backgroundColor: Colors.redAccent,
                foregroundColor: Colors.white,
              ),
              onPressed: () {
                unawaited(bltController.emergencyStop());
                debugPrint("🚨 E-STOP 작동!");
              },
              icon: const Icon(Icons.warning_rounded),
              label: const Text('E-STOP', style: TextStyle(fontWeight: FontWeight.bold)),
            ),
          ),
        ],
      ),
      drawer: Drawer(
        child: ListView(
          padding: EdgeInsets.zero,
          children: [
            const DrawerHeader(
              decoration: BoxDecoration(color: Colors.blueGrey),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                mainAxisAlignment: MainAxisAlignment.end,
                children: [
                  Icon(Icons.precision_manufacturing, color: Colors.white, size: 36),
                  SizedBox(height: 10),
                  Text('제어 메뉴', style: TextStyle(color: Colors.white, fontSize: 18, fontWeight: FontWeight.bold)),
                ],
              ),
            ),
            ListTile(
              leading: const Icon(Icons.directions_car),
              title: const Text('RC카 주행'),
              selected: _currentIndex == 0,
              onTap: () {
                setState(() {
                  _currentIndex = 0;
                  unawaited(bltController.sendDrive(0, 0, 0, 0));
                });
                Navigator.pop(context);
              },
            ),
            ListTile(
              leading: const Icon(Icons.precision_manufacturing),
              title: const Text('로봇팔 조종'),
              selected: _currentIndex == 1,
              onTap: () {
                setState(() {
                  _currentIndex = 1;
                  unawaited(bltController.sendDrive(0, 0, 0, 0));
                });
                Navigator.pop(context);
              },
            ),
            ListTile(
              leading: const Icon(Icons.bookmark_added),
              title: const Text('티칭 시스템'),
              selected: _currentIndex == 2,
              onTap: () {
                setState(() {
                  _currentIndex = 2;
                  unawaited(bltController.sendDrive(0, 0, 0, 0));
                });
                Navigator.pop(context);
              },
            ),
          ],
        ),
      ),
      body: _screens[_currentIndex],
    );
  }
}

// ======================================================
// 2. 탭 1 화면 (RC카 주행 조이스틱)
// ======================================================
class JoystickTab extends StatefulWidget {
  const JoystickTab({super.key});

  @override
  State<JoystickTab> createState() => _JoystickTabState();
}

class _JoystickTabState extends State<JoystickTab> {
  Timer? _heartbeatTimer;
  BluetoothController? _controller;
  int _leftDirection = 0;
  int _leftPwm = 0;
  int _rightDirection = 0;
  int _rightPwm = 0;

  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    _controller = Provider.of<BluetoothController>(context, listen: false);
    _heartbeatTimer ??= Timer.periodic(
      const Duration(milliseconds: 100),
      (_) => _sendCurrentCommand(),
    );
  }

  void _sendCurrentCommand() {
    final BluetoothController? controller = _controller;
    if (controller == null) {
      return;
    }

    unawaited(controller.sendDrive(
      _leftDirection,
      _leftPwm,
      _rightDirection,
      _rightPwm,
    ));
  }

  void _updateJoystick(double x, double y) {
    final double forward = -y;
    double leftSpeed =
        (forward < 0) ? (forward - x) : (forward + x);
    double rightSpeed =
        (forward < 0) ? (forward + x) : (forward - x);

    leftSpeed = leftSpeed.clamp(-1.0, 1.0).toDouble();
    rightSpeed = rightSpeed.clamp(-1.0, 1.0).toDouble();

    _leftDirection = leftSpeed >= 0 ? 0 : 1;
    _leftPwm = (leftSpeed.abs() * 255).round();
    _rightDirection = rightSpeed >= 0 ? 0 : 1;
    _rightPwm = (rightSpeed.abs() * 255).round();
    _sendCurrentCommand();
  }

  @override
  void dispose() {
    _heartbeatTimer?.cancel();
    final BluetoothController? controller = _controller;
    if ((controller != null) && !controller.isEstopLatched) {
      unawaited(controller.sendDrive(0, 0, 0, 0));
    }
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Center(
      child: SingleChildScrollView(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            const Text('차동 조향(탱크 턴) 조이스틱',
                style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold, color: Colors.blueGrey)),
            const SizedBox(height: 10),
            Joystick(
              mode: JoystickMode.all,
              listener: (details) {
                _updateJoystick(details.x, details.y);
              },
            ),
          ],
        ),
      ),
    );
  }
}

// ======================================================
// 3. 탭 2 화면 (로봇팔 수동 조종)
// ======================================================
class HoldAngleButton extends StatefulWidget {
  const HoldAngleButton({
    required this.icon,
    required this.onStep,
    super.key,
  });

  final IconData icon;
  final VoidCallback? onStep;

  @override
  State<HoldAngleButton> createState() => _HoldAngleButtonState();
}

class _HoldAngleButtonState extends State<HoldAngleButton> {
  Timer? _repeatTimer;

  void _startRepeating() {
    widget.onStep?.call();
    _repeatTimer?.cancel();
    _repeatTimer = Timer.periodic(
      const Duration(milliseconds: 120),
      (_) => widget.onStep?.call(),
    );
  }

  void _stopRepeating() {
    _repeatTimer?.cancel();
    _repeatTimer = null;
  }

  @override
  void didUpdateWidget(covariant HoldAngleButton oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (widget.onStep == null) {
      _stopRepeating();
    }
  }

  @override
  void dispose() {
    _stopRepeating();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final enabled = widget.onStep != null;

    return GestureDetector(
      onTap: enabled ? widget.onStep : null,
      onLongPressStart: enabled ? (_) => _startRepeating() : null,
      onLongPressEnd: enabled ? (_) => _stopRepeating() : null,
      onLongPressCancel: enabled ? _stopRepeating : null,
      child: Padding(
        padding: const EdgeInsets.all(8),
        child: Icon(
          widget.icon,
          color: enabled ? Colors.blueAccent : Colors.grey,
        ),
      ),
    );
  }
}

class RobotArmControlTab extends StatefulWidget {
  const RobotArmControlTab({super.key});

  static const int gripperIndex = 5;
  static const int homePacketAngle = 90;
  static List<int> currentAngles = [90, 90, 90, 90, 90, 0];

  static int packetToDisplayValue(int index, int packetValue) {
    if (index == gripperIndex) {
      return ((packetValue * 100) / 180).round();
    }
    return packetValue - 90;
  }

  static int displayToPacketValue(int index, double displayValue) {
    if (index == gripperIndex) {
      return ((displayValue.clamp(0, 100) * 180) / 100).round();
    }
    return (displayValue + 90).round().clamp(0, 180).toInt();
  }

  static String formatWaypoint(List<int> packetValues) {
    return packetValues.asMap().entries.map((entry) {
      final value = packetToDisplayValue(entry.key, entry.value);
      return entry.key == gripperIndex ? '$value%' : '$value°';
    }).join(', ');
  }

  @override
  State<RobotArmControlTab> createState() => _RobotArmControlTabState();
}

class _RobotArmControlTabState extends State<RobotArmControlTab> {
  final List<String> jointNames = ['Gripper', 'Wrist Rotate', 'Wrist Tilt', 'Elbow', 'Shoulder', 'Base'];

  void _enableHome(BluetoothController controller) {
    setState(() {
      RobotArmControlTab.currentAngles = List<int>.filled(
        6,
        RobotArmControlTab.homePacketAngle,
      );
      RobotArmControlTab.currentAngles[RobotArmControlTab.gripperIndex] = 0;
    });
    unawaited(controller.enableArm(RobotArmControlTab.currentAngles));
  }

  void _updateAngle(int originalIndex, double newAngle) {
    setState(() {
      final mappedAngle = RobotArmControlTab.displayToPacketValue(
        originalIndex,
        newAngle,
      );
      RobotArmControlTab.currentAngles[originalIndex] = mappedAngle;

      final controller =
          Provider.of<BluetoothController>(context, listen: false);
      unawaited(controller.sendArmPose(RobotArmControlTab.currentAngles));
    });
  }

  @override
  Widget build(BuildContext context) {
    final controller = Provider.of<BluetoothController>(context);

    return Padding(
      padding: const EdgeInsets.symmetric(horizontal: 16.0, vertical: 6.0),
      child: Column(
        children: [
          Row(
            mainAxisAlignment: MainAxisAlignment.end,
            children: [
              Text(
                controller.isArmEnabled ? '출력 활성화됨' : '출력 차단됨',
                style: TextStyle(
                  color: controller.isArmEnabled ? Colors.green : Colors.grey,
                  fontWeight: FontWeight.bold,
                ),
              ),
              const SizedBox(width: 12),
              FilledButton.icon(
                onPressed: controller.isEstopLatched
                    ? null
                    : () {
                        if (controller.isArmEnabled) {
                          unawaited(controller.disableArm());
                        } else {
                          _enableHome(controller);
                        }
                      },
                icon: Icon(
                  controller.isArmEnabled
                      ? Icons.power_settings_new
                      : Icons.play_circle_outline,
                ),
                label: Text(controller.isArmEnabled ? '출력 끄기' : '홈 자세로 활성화'),
              ),
            ],
          ),
          ...List.generate(6, (index) {
          int originalIndex = 5 - index;
          final isGripper = originalIndex == RobotArmControlTab.gripperIndex;
          final displayValue = RobotArmControlTab.packetToDisplayValue(
            originalIndex,
            RobotArmControlTab.currentAngles[originalIndex],
          ).toDouble();

          return Expanded(
            child: Card(
              margin: const EdgeInsets.symmetric(vertical: 2.0),
              child: Padding(
                padding: const EdgeInsets.symmetric(horizontal: 12.0),
                child: Row(
                  children: [
                    SizedBox(width: 100, child: Text(jointNames[index], style: const TextStyle(fontWeight: FontWeight.bold, fontSize: 13))),
                    HoldAngleButton(
                      icon: Icons.remove_circle_outline,
                      onStep: controller.isArmEnabled
                          ? () => _updateAngle(originalIndex, displayValue - 1)
                          : null,
                    ),
                    Expanded(
                      child: Slider(
                        value: displayValue,
                        min: isGripper ? 0 : -90,
                        max: isGripper ? 100 : 90,
                        onChanged: controller.isArmEnabled
                            ? (value) => _updateAngle(originalIndex, value)
                            : null,
                      ),
                    ),
                    HoldAngleButton(
                      icon: Icons.add_circle_outline,
                      onStep: controller.isArmEnabled
                          ? () => _updateAngle(originalIndex, displayValue + 1)
                          : null,
                    ),
                    SizedBox(
                      width: 45,
                      child: Text(
                        isGripper
                            ? '${displayValue.round()}%'
                            : '${displayValue.round()}°',
                        textAlign: TextAlign.end,
                        style: const TextStyle(fontWeight: FontWeight.bold),
                      ),
                    ),
                  ],
                ),
              ),
            ),
          );
          }),
        ],
      ),
    );
  }
}

// ======================================================
// 4. 탭 3 화면 (티칭 시스템)
// ======================================================
class TeachingTab extends StatefulWidget {
  const TeachingTab({super.key});

  @override
  State<TeachingTab> createState() => _TeachingTabState();
}

class _TeachingTabState extends State<TeachingTab> {
  int selectedSequence = 1;
  bool _uploading = false;
  final List<String> sequenceNames = List.generate(12, (index) => '시퀀스 ${index + 1}');

  final List<List<List<int>>> sequenceWaypoints = List.generate(12, (_) => []);

  Future<void> _uploadToSTM32(BluetoothController blt) async {
    List<List<int>> currentList = sequenceWaypoints[selectedSequence - 1];

    if (_uploading) {
      return;
    }

    if (!blt.isConnected) {
      ScaffoldMessenger.of(context).showSnackBar(
        const SnackBar(content: Text('먼저 HC-05에 연결해주세요.')),
      );
      return;
    }

    if (currentList.isEmpty) {
      ScaffoldMessenger.of(context).showSnackBar(const SnackBar(content: Text('업로드할 웨이포인트가 없습니다.')));
      return;
    }

    setState(() => _uploading = true);
    final int sequenceId = selectedSequence;
    final Future<bool> ack = blt.waitForTeachingAck(sequenceId);

    try {
      // START 뒤 각 조각을 순서대로 보내 armQueue가 넘치지 않게 짧게 간격을 둔다.
      await blt.sendCommand(
        2,
        data: [4, 1, sequenceId, currentList.length],
      );
      await Future.delayed(const Duration(milliseconds: 50));

      for (int i = 0; i < currentList.length; i++) {
        final List<int> angles = currentList[i];
        await blt.sendCommand(
          2,
          data: [4, 2, sequenceId, i, angles[0], angles[1], angles[2]],
        );
        await Future.delayed(const Duration(milliseconds: 30));
        await blt.sendCommand(
          2,
          data: [4, 3, sequenceId, i, angles[3], angles[4], angles[5]],
        );
        await Future.delayed(const Duration(milliseconds: 30));
      }

      await blt.sendCommand(2, data: [4, 4, sequenceId]);
      final bool succeeded = await ack;
      if (!succeeded && mounted) {
        ScaffoldMessenger.of(context).showSnackBar(
          const SnackBar(
            content: Text('저장 ACK를 받지 못했거나 Flash 저장에 실패했습니다.'),
          ),
        );
      }
    } catch (error) {
      if (mounted) {
        ScaffoldMessenger.of(context).showSnackBar(
          SnackBar(content: Text('업로드 전송 실패: $error')),
        );
      }
    } finally {
      if (mounted) {
        setState(() => _uploading = false);
      }
    }
  }

  @override
  Widget build(BuildContext context) {
    final bltController = Provider.of<BluetoothController>(context);
    List<List<int>> currentWaypoints = sequenceWaypoints[selectedSequence - 1];

    return Padding(
      padding: const EdgeInsets.all(8.0),
      child: Row(
        children: [
          Expanded(
            flex: 5,
            child: Card(
              elevation: 2,
              child: Padding(
                padding: const EdgeInsets.all(8.0),
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Row(
                      mainAxisAlignment: MainAxisAlignment.spaceBetween,
                      children: [
                        Text('📌 ${sequenceNames[selectedSequence - 1]} 웨이포인트 (${currentWaypoints.length}/30)',
                            style: const TextStyle(fontWeight: FontWeight.bold, fontSize: 13, color: Colors.blueGrey)),
                        IconButton(
                          icon: const Icon(Icons.add_circle, color: Colors.blueAccent),
                          onPressed: _uploading ? null : () {
                            if (currentWaypoints.length < 30) {
                              setState(() {
                                currentWaypoints.add(List.from(RobotArmControlTab.currentAngles));
                              });
                            }
                          },
                        )
                      ],
                    ),
                    const Divider(height: 1),
                    Expanded(
                      child: currentWaypoints.isEmpty
                          ? const Center(child: Text('추가된 웨이포인트가 없습니다.\n+ 버튼을 눌러 현재 위치를 저장하세요.', textAlign: TextAlign.center))
                          : ListView.builder(
                        itemCount: currentWaypoints.length,
                        itemBuilder: (context, index) {
                          List<int> wp = currentWaypoints[index];
                          return Card(
                            color: Colors.grey.shade100,
                            margin: const EdgeInsets.symmetric(vertical: 3),
                            child: ListTile(
                              dense: true,
                              title: Text('웨이포인트 #${index + 1}', style: const TextStyle(fontWeight: FontWeight.bold)),
                              subtitle: Text(
                                '값: [${RobotArmControlTab.formatWaypoint(wp)}]',
                              ),
                               trailing: IconButton(
                                 icon: const Icon(Icons.delete_outline, color: Colors.redAccent, size: 18),
                                 onPressed: _uploading ? null : () {
                                  setState(() {
                                    currentWaypoints.removeAt(index);
                                  });
                                },
                              ),
                            ),
                          );
                        },
                      ),
                    ),
                  ],
                ),
              ),
            ),
          ),

          const SizedBox(width: 8),

          Expanded(
            flex: 4,
            child: Card(
              elevation: 2,
              child: SingleChildScrollView(
                padding: const EdgeInsets.all(12.0),
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.stretch,
                  children: [
                    const Text('대상 시퀀스 선택', style: TextStyle(fontSize: 12, color: Colors.blueGrey, fontWeight: FontWeight.bold)),
                    const SizedBox(height: 6),
                    DropdownButton<int>(
                      isExpanded: true,
                      value: selectedSequence,
                      items: List.generate(12, (index) {
                        return DropdownMenuItem<int>(
                          value: index + 1,
                          child: Text(sequenceNames[index], style: const TextStyle(fontWeight: FontWeight.bold)),
                        );
                      }),
                       onChanged: _uploading ? null : (val) {
                        if (val != null) setState(() => selectedSequence = val);
                      },
                    ),
                    const SizedBox(height: 12),
                    ElevatedButton.icon(
                      style: ElevatedButton.styleFrom(backgroundColor: Colors.green, foregroundColor: Colors.white),
                      onPressed: (bltController.isArmEnabled && !_uploading)
                          ? () => unawaited(bltController.sendCommand(
                                2,
                                data: [2, selectedSequence],
                              ))
                          : null,
                      icon: const Icon(Icons.play_arrow),
                      label: const Text('시퀀스 재생 (PLAY)'),
                    ),
                    const SizedBox(height: 8),
                    ElevatedButton.icon(
                      style: ElevatedButton.styleFrom(backgroundColor: Colors.deepPurple, foregroundColor: Colors.white),
                      onPressed: _uploading
                          ? null
                          : () => unawaited(_uploadToSTM32(bltController)),
                      icon: _uploading
                          ? const SizedBox(
                              width: 18,
                              height: 18,
                              child: CircularProgressIndicator(strokeWidth: 2),
                            )
                          : const Icon(Icons.upload_file),
                      label: Text(
                        _uploading
                            ? '업로드 중...'
                            : '기기(STM32) 플래시 업로드',
                      ),
                    ),
                    const SizedBox(height: 8),
                    OutlinedButton.icon(
                      style: OutlinedButton.styleFrom(side: const BorderSide(color: Colors.redAccent), foregroundColor: Colors.redAccent),
                      onPressed: _uploading
                          ? null
                          : () {
                              setState(() => currentWaypoints.clear());
                              unawaited(bltController.sendCommand(
                                2,
                                data: [3, selectedSequence],
                              ));
                            },
                      icon: const Icon(Icons.delete),
                      label: const Text('시퀀스 초기화 (RESET)'),
                    ),
                  ],
                ),
              ),
            ),
          ),
        ],
      ),
    );
  }
}
