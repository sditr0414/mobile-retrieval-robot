import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:mobile_retrieval_robot_app/main.dart';
import 'package:provider/provider.dart';

class _TrackingBluetoothController extends BluetoothController {
  int stopDriveCalls = 0;

  @override
  Future<void> stopDrive() async {
    stopDriveCalls++;
  }
}

class _DelayedSequenceBluetoothController extends BluetoothController {
  static const pose = <int>[100, 80, 70, 60, 50, 40];

  @override
  Future<List<List<int>>?> requestTeachingSequence(int sequenceId) async {
    await Future<void>.delayed(const Duration(milliseconds: 30));
    return [pose];
  }
}

void main() {
  test('Flash가 없을 때 안전 서보 초기 펄스 배열을 사용한다', () {
    expect(RobotArmControlTab.jointNames, [
      'Base',
      'Shoulder',
      'Elbow',
      'Wrist Tilt',
      'Wrist Rotate',
      'Gripper',
    ]);
    expect(BluetoothController().servoSpeedPercent, 100);
    expect(BluetoothController().pidKpMilli, 2000);
    expect(BluetoothController().pidKiMilli, 1400);
    expect(BluetoothController().pidKdMilli, 0);
    final thresholdController = BluetoothController();
    expect(thresholdController.joystickThreshold, 0.10);
    thresholdController.setJoystickThreshold(0.08);
    expect(thresholdController.joystickThreshold, 0.08);
    expect(BluetoothController.servoReversed, [
      true,
      true,
      false,
      true,
      true,
      true,
    ]);
    expect(BluetoothController.defaultServoCalibrationUs, [
      [2300, 1500, 700],
      [2300, 1500, 700],
      [700, 1500, 2300],
      [2300, 1500, 700],
      [2300, 1500, 700],
      [1800, 1500, 1200],
    ]);
    expect(BluetoothController.defaultTravelPoseAngles, [90, 40, 180, 90, 150]);
  });

  test('호환용 이동 자세 5바이트를 기존 설정 형식에 유지한다', () async {
    final controller = BluetoothController();
    expect(controller.enableFeaturePreviewMode(), isTrue);
    const travelPose = [100, 50, 170, 85, 140];

    final saved = await controller.saveSettings(
      kpMilli: controller.pidKpMilli,
      kiMilli: controller.pidKiMilli,
      kdMilli: controller.pidKdMilli,
      servoPulsesUs: controller.servoCalibrationUs,
      travelPose: travelPose,
    );

    expect(saved, isTrue);
    expect(controller.travelPoseAngles, travelPose);
    expect(await controller.requestSettings(), isTrue);
    expect(controller.travelPoseAngles, travelPose);
  });

  test('Kp는 ×100 단위와 2.55 최댓값을 지킨다', () async {
    final controller = BluetoothController();
    expect(controller.enableFeaturePreviewMode(), isTrue);

    Future<bool> saveKp(int kpMilli) => controller.saveSettings(
      kpMilli: kpMilli,
      kiMilli: 0,
      kdMilli: 0,
      servoPulsesUs: controller.servoCalibrationUs,
      travelPose: controller.travelPoseAngles,
    );

    expect(await saveKp(2550), isTrue);
    expect(controller.pidKpMilli, 2550);
    expect(await saveKp(2560), isFalse);
    expect(await saveKp(2555), isFalse);
    controller.dispose();
  });

  test('잡기 전용 시퀀스 이름과 선행 상태를 관리한다', () async {
    final teaching = TeachingController();
    expect(teaching.sequenceName(TeachingController.travelSequence), '이동 자세');
    expect(
      teaching.sequenceName(TeachingController.pickupPoseSequence),
      '잡기 위치',
    );
    expect(teaching.sequenceName(TeachingController.gripSequence), '잡기');
    expect(teaching.sequenceName(TeachingController.releaseSequence), '놓기');
    teaching.selectSequence(1);
    expect(teaching.renameSelectedSequence('선반 정리'), isNull);
    expect(teaching.addCurrentPose(<int>[90, 80, 70, 60, 50, 40]), isTrue);
    expect(teaching.sequenceName(1), '선반 정리');
    teaching.selectSequence(TeachingController.travelSequence);
    expect(teaching.renameSelectedSequence('변경 금지'), isNotNull);
    expect(teaching.sequenceName(TeachingController.travelSequence), '이동 자세');
    teaching.selectSequence(1);
    expect(teaching.sequenceName(1), '시퀀스 1');
    expect(teaching.currentWaypoints, isEmpty);

    final controller = BluetoothController();
    controller.enableFeaturePreviewMode();
    controller.isEstopLatched = false;
    controller.isArmEnabled = true;
    const lastPose = <int>[90, 80, 70, 60, 50, 40];
    RobotArmControlTab.setOriginPose();
    expect(teaching.addCurrentPose(lastPose), isTrue);
    expect(await teaching.playOnSTM32(controller), isNull);
    expect(RobotArmControlTab.currentAngles, lastPose);
    teaching.selectSequence(2);
    teaching.selectSequence(1);
    expect(teaching.currentWaypoints, isEmpty);
    controller.finishPickupAction(
      state: PickupWorkflowState.pickupPoseReady,
      driveReady: true,
    );
    expect(controller.isAtPickupPose, isTrue);
    expect(controller.isHoldingPayload, isFalse);

    controller.finishPickupAction(
      state: PickupWorkflowState.holding,
      driveReady: false,
    );
    expect(controller.isHoldingPayload, isTrue);
    expect(controller.isDriveReady, isFalse);
    expect(await controller.prepareTravelSequencePreservingGripper(), isTrue);
    expect(controller.isDriveReady, isTrue);
    controller.dispose();
  });

  test('저장 시퀀스 조회부터 완료까지 재생을 잠그고 마지막 자세를 반영한다', () async {
    final controller = _DelayedSequenceBluetoothController();
    final teaching = TeachingController();
    controller.enableFeaturePreviewMode();
    controller.isEstopLatched = false;
    controller.isArmEnabled = true;
    RobotArmControlTab.setOriginPose();

    final play = teaching.playSequenceOnSTM32(controller, 10);
    expect(teaching.isPlaying, isTrue);
    expect(await play, isNull);
    expect(teaching.isPlaying, isFalse);
    expect(
      RobotArmControlTab.currentAngles,
      _DelayedSequenceBluetoothController.pose,
    );

    controller.dispose();
    teaching.dispose();
  });

  testWidgets('티칭 선택창에 고정 시퀀스 이름을 모두 표시한다', (tester) async {
    final bluetooth = BluetoothController();
    final teaching = TeachingController()..selectSequence(9);
    bluetooth.enableFeaturePreviewMode();

    await tester.pumpWidget(
      MultiProvider(
        providers: [
          ChangeNotifierProvider<BluetoothController>.value(value: bluetooth),
          ChangeNotifierProvider.value(value: teaching),
        ],
        child: const MaterialApp(home: Scaffold(body: TeachingTab())),
      ),
    );
    await tester.tap(find.byType(DropdownButton<int>));
    await tester.pumpAndSettle();

    for (final name in const ['이동 자세', '잡기 위치', '잡기', '놓기']) {
      expect(find.text(name), findsWidgets);
    }

    await tester.pumpWidget(const SizedBox.shrink());
    bluetooth.dispose();
    teaching.dispose();
  });

  testWidgets('시퀀스 이름 편집을 취소해도 기존 이름을 유지한다', (tester) async {
    final bluetooth = BluetoothController();
    final teaching = TeachingController();
    await tester.pumpWidget(
      MultiProvider(
        providers: [
          ChangeNotifierProvider.value(value: bluetooth),
          ChangeNotifierProvider.value(value: teaching),
        ],
        child: const MaterialApp(home: Scaffold(body: TeachingTab())),
      ),
    );

    await tester.tap(find.widgetWithText(OutlinedButton, '이름'));
    await tester.pumpAndSettle();
    expect(find.text('시퀀스 이름'), findsOneWidget);
    await tester.tap(find.text('취소'));
    await tester.pumpAndSettle();

    expect(tester.takeException(), isNull);
    expect(teaching.sequenceName(1), '시퀀스 1');
    await tester.pumpWidget(const SizedBox.shrink());
    bluetooth.dispose();
    teaching.dispose();
  });

  test('원점 보정은 최신 목표 펄스 도착 응답 전까지 이동 중으로 표시한다', () async {
    final controller = BluetoothController();
    expect(controller.enableFeaturePreviewMode(), isTrue);
    controller.isArmEnabled = true;

    final movement = controller.previewServoPulse(0, 1500);
    expect(controller.servoPreviewMoving, isTrue);

    await movement;
    await Future<void>.delayed(const Duration(milliseconds: 150));
    expect(controller.servoPreviewMoving, isFalse);
  });

  testWidgets('모든 제어 슬라이더는 손잡이 드래그만 허용한다', (tester) async {
    await tester.pumpWidget(
      MultiProvider(
        providers: [
          ChangeNotifierProvider(create: (_) => BluetoothController()),
          ChangeNotifierProvider(create: (_) => TeachingController()),
        ],
        child: const RobotApp(),
      ),
    );

    final sliders = tester.widgetList<Slider>(find.byType(Slider)).toList();
    expect(sliders, isNotEmpty);
    for (final slider in sliders) {
      expect(slider.allowedInteraction, SliderInteraction.slideOnly);
    }
  });

  testWidgets('원점 보정 화면은 이동 자세 편집기를 표시하지 않는다', (tester) async {
    final controller = BluetoothController();
    controller.enableFeaturePreviewMode();
    controller.isEstopLatched = false;

    await tester.pumpWidget(
      MultiProvider(
        providers: [
          ChangeNotifierProvider.value(value: controller),
          ChangeNotifierProvider(create: (_) => TeachingController()),
        ],
        child: const RobotApp(),
      ),
    );

    await tester.tap(find.text('원점 보정'));
    await tester.pumpAndSettle();
    expect(find.text('보정 시작'), findsOneWidget);
    expect(
      find.descendant(of: find.byType(AppBar), matching: find.text('테스트')),
      findsOneWidget,
    );
    expect(
      find.descendant(of: find.byType(AppBar), matching: find.text('STOP')),
      findsOneWidget,
    );

    await tester.tap(find.text('시작'));
    await tester.pumpAndSettle();
    expect(controller.isArmEnabled, isTrue);
    expect(find.text('베이스 회전 · 0°'), findsOneWidget);
    expect(
      tester.widget<ListView>(find.byType(ListView)).physics,
      isA<NeverScrollableScrollPhysics>(),
    );
    expect(find.widgetWithText(ChoiceChip, '베이스 회전'), findsOneWidget);
    expect(find.widgetWithText(ChoiceChip, '어깨'), findsOneWidget);
    expect(find.widgetWithText(ChoiceChip, '팔꿈치'), findsOneWidget);
    expect(find.widgetWithText(ChoiceChip, '손목 상하'), findsOneWidget);
    expect(find.widgetWithText(ChoiceChip, '손목 회전'), findsOneWidget);
    expect(find.widgetWithText(ChoiceChip, '그리퍼'), findsOneWidget);
    expect(find.widgetWithText(ChoiceChip, '-90°'), findsOneWidget);
    expect(find.widgetWithText(ChoiceChip, '0°'), findsOneWidget);
    expect(find.widgetWithText(ChoiceChip, '+90°'), findsOneWidget);

    await tester.tap(find.widgetWithText(ChoiceChip, '그리퍼'));
    await tester.pumpAndSettle();
    expect(find.widgetWithText(ChoiceChip, '열림'), findsOneWidget);
    expect(find.widgetWithText(ChoiceChip, '닫힘'), findsOneWidget);
    expect(find.text('열림 (-90°)'), findsNothing);
    expect(find.text('닫힘 (+90°)'), findsNothing);

    expect(find.text('2. 이동 자세'), findsNothing);
    expect(find.text('확인 시퀀스'), findsOneWidget);
    expect(find.text('원점 보정 저장'), findsOneWidget);

    await tester.pumpWidget(const SizedBox.shrink());
    controller.dispose();
  });

  testWidgets('각도를 입력해 웨이포인트를 저장해도 다이얼로그가 안전하게 닫힌다', (tester) async {
    final bluetooth = BluetoothController();
    final teaching = TeachingController();

    await tester.pumpWidget(
      MultiProvider(
        providers: [
          ChangeNotifierProvider.value(value: bluetooth),
          ChangeNotifierProvider.value(value: teaching),
        ],
        child: const RobotApp(),
      ),
    );

    await tester.tap(find.text('티칭'));
    await tester.pumpAndSettle();

    final angleInput = find.text('각도 직접 입력');
    await tester.scrollUntilVisible(
      angleInput,
      500,
      scrollable: find.byType(Scrollable).last,
    );
    await tester.ensureVisible(angleInput);
    await tester.pumpAndSettle();
    await tester.tap(angleInput);
    await tester.pumpAndSettle();
    expect(find.text('자세 각도 입력'), findsOneWidget);

    await tester.tap(find.text('자세 저장'));
    await tester.pumpAndSettle();

    expect(find.text('자세 1'), findsOneWidget);
    expect(teaching.currentWaypoints, hasLength(1));
    expect(tester.takeException(), isNull);

    await tester.pumpWidget(const SizedBox.shrink());
    await tester.pump(const Duration(milliseconds: 400));
    bluetooth.dispose();
    teaching.dispose();
  });

  testWidgets('제목을 길게 누르면 Bluetooth 없는 기능 확인 모드를 시작한다', (tester) async {
    await tester.pumpWidget(
      MultiProvider(
        providers: [
          ChangeNotifierProvider(create: (_) => BluetoothController()),
          ChangeNotifierProvider(create: (_) => TeachingController()),
        ],
        child: const RobotApp(),
      ),
    );

    await tester.longPress(
      find.descendant(of: find.byType(AppBar), matching: find.text('로봇팔')),
    );
    await tester.pumpAndSettle();
    expect(find.text('TEST 모드'), findsOneWidget);

    await tester.tap(find.widgetWithText(FilledButton, '시작'));
    await tester.pumpAndSettle();

    expect(find.text('TEST · 로봇팔'), findsOneWidget);
    expect(find.text('테스트'), findsOneWidget);
    expect(find.text('STOP 해제'), findsOneWidget);

    await tester.tap(find.text('STOP 해제'));
    await tester.pumpAndSettle();
    await tester.tap(
      find.descendant(of: find.byType(AlertDialog), matching: find.text('해제')),
    );
    await tester.pump(const Duration(milliseconds: 150));

    final controller = Provider.of<BluetoothController>(
      tester.element(find.byType(MainScreen)),
      listen: false,
    );
    expect(controller.isFeaturePreviewMode, isTrue);
    expect(controller.isEstopLatched, isFalse);

    // 실제 차량 없이도 향후 PID ACK와 적용 상태 UI 흐름을 확인할 수 있다.
    controller.isDriveReady = true;
    final bool pidEnabled =
        await tester.runAsync(() => controller.setPidEnabled(true)) ?? false;
    expect(pidEnabled, isTrue);
    expect(controller.pidApplied, isTrue);
    expect(controller.hasFreshPidStatus, isTrue);
    expect(controller.hasFreshImuStatus, isTrue);
    expect(controller.deviceTemperatureC, closeTo(34.6, 0.01));
  });

  testWidgets('앱이 백그라운드로 가면 차량을 즉시 정지한다', (tester) async {
    final bluetooth = _TrackingBluetoothController();
    final teaching = TeachingController();

    await tester.pumpWidget(
      MultiProvider(
        providers: [
          ChangeNotifierProvider<BluetoothController>.value(value: bluetooth),
          ChangeNotifierProvider.value(value: teaching),
        ],
        child: const RobotApp(),
      ),
    );
    tester.binding.handleAppLifecycleStateChanged(AppLifecycleState.paused);
    await tester.pump();

    expect(bluetooth.stopDriveCalls, 1);

    tester.binding.handleAppLifecycleStateChanged(AppLifecycleState.resumed);
    await tester.pumpWidget(const SizedBox.shrink());
    bluetooth.dispose();
    teaching.dispose();
  });

  testWidgets('가로 화면에서는 좌측 NavigationRail을 사용한다', (tester) async {
    tester.view.physicalSize = const Size(1280, 720);
    tester.view.devicePixelRatio = 1;
    addTearDown(() {
      tester.view.resetPhysicalSize();
      tester.view.resetDevicePixelRatio();
    });

    await tester.pumpWidget(
      MultiProvider(
        providers: [
          ChangeNotifierProvider(create: (_) => BluetoothController()),
          ChangeNotifierProvider(create: (_) => TeachingController()),
        ],
        child: const RobotApp(),
      ),
    );
    await tester.pump();

    expect(find.byType(NavigationRail), findsOneWidget);
    expect(find.byType(NavigationBar), findsNothing);
    expect(
      find.descendant(
        of: find.byType(NavigationRail),
        matching: find.text('차량'),
      ),
      findsOneWidget,
    );
    expect(
      find.descendant(
        of: find.byType(NavigationRail),
        matching: find.text('로봇팔'),
      ),
      findsOneWidget,
    );
    expect(
      find.descendant(
        of: find.byType(NavigationRail),
        matching: find.text('티칭'),
      ),
      findsOneWidget,
    );
  });

  testWidgets('세로 화면에서는 하단 NavigationBar를 사용한다', (tester) async {
    tester.view.physicalSize = const Size(412, 915);
    tester.view.devicePixelRatio = 1;
    addTearDown(() {
      tester.view.resetPhysicalSize();
      tester.view.resetDevicePixelRatio();
    });

    await tester.pumpWidget(
      MultiProvider(
        providers: [
          ChangeNotifierProvider(create: (_) => BluetoothController()),
          ChangeNotifierProvider(create: (_) => TeachingController()),
        ],
        child: const RobotApp(),
      ),
    );
    await tester.pump();

    expect(find.byType(NavigationBar), findsOneWidget);
    expect(find.byType(NavigationRail), findsNothing);
    expect(
      find.descendant(
        of: find.byType(NavigationBar),
        matching: find.text('차량'),
      ),
      findsOneWidget,
    );
    expect(
      find.descendant(
        of: find.byType(NavigationBar),
        matching: find.text('로봇팔'),
      ),
      findsOneWidget,
    );
    expect(
      find.descendant(
        of: find.byType(NavigationBar),
        matching: find.text('티칭'),
      ),
      findsOneWidget,
    );
  });

  testWidgets('로봇팔 화면은 6개 관절과 속도 팝업을 한 화면에 표시한다', (tester) async {
    tester.view.physicalSize = const Size(412, 915);
    tester.view.devicePixelRatio = 1;
    addTearDown(() {
      tester.view.resetPhysicalSize();
      tester.view.resetDevicePixelRatio();
    });

    final controller = BluetoothController();
    controller.enableFeaturePreviewMode();
    controller.isEstopLatched = false;
    controller.isArmEnabled = true;

    await tester.pumpWidget(
      MultiProvider(
        providers: [
          ChangeNotifierProvider.value(value: controller),
          ChangeNotifierProvider(create: (_) => TeachingController()),
        ],
        child: const RobotApp(),
      ),
    );
    await tester.pump();

    expect(find.text('드래그하면 즉시 적용'), findsNothing);
    expect(find.text('그리퍼').hitTestable(), findsOneWidget);
    expect(find.text('이동 자세').hitTestable(), findsOneWidget);
    expect(find.text('주행 준비 자세'), findsNothing);
    expect(find.byTooltip('서보 속도 100%'), findsOneWidget);

    await tester.tap(find.byTooltip('서보 속도 100%'));
    await tester.pumpAndSettle();
    expect(find.text('서보 속도'), findsOneWidget);
    expect(
      find.descendant(
        of: find.byType(AlertDialog),
        matching: find.text('100%'),
      ),
      findsOneWidget,
    );
    expect(find.byTooltip('서보 속도 직접 입력'), findsOneWidget);

    await tester.tap(find.text('완료'));
    await tester.pumpAndSettle();

    await tester.tap(find.byTooltip('베이스 회전 직접 입력'));
    await tester.pumpAndSettle();
    expect(find.text('베이스 회전 직접 입력'), findsOneWidget);
    await tester.enterText(find.byType(TextField), '45');
    await tester.tap(find.text('적용'));
    await tester.pumpAndSettle();
    expect(RobotArmControlTab.currentAngles[0], 135);
    expect(find.text('45°'), findsOneWidget);

    await tester.pumpWidget(const SizedBox.shrink());
    controller.dispose();
  });

  testWidgets('좁은 차량 화면에서도 물체 잡기 버튼이 잘리지 않는다', (tester) async {
    tester.view.physicalSize = const Size(320, 800);
    tester.view.devicePixelRatio = 1;
    addTearDown(() {
      tester.view.resetPhysicalSize();
      tester.view.resetDevicePixelRatio();
    });

    final controller = BluetoothController();
    controller.enableFeaturePreviewMode();
    controller.isEstopLatched = false;
    controller.isArmEnabled = true;
    controller.isDriveReady = true;

    await tester.pumpWidget(
      MultiProvider(
        providers: [
          ChangeNotifierProvider.value(value: controller),
          ChangeNotifierProvider(create: (_) => TeachingController()),
        ],
        child: const MaterialApp(home: Scaffold(body: JoystickTab())),
      ),
    );
    // flutter_joystick의 초기 중앙 정렬 애니메이션이 끝난 뒤 레이아웃을 검사한다.
    await tester.pump(const Duration(seconds: 2));

    expect(
      tester
          .widget<SingleChildScrollView>(find.byType(SingleChildScrollView))
          .physics,
      isA<NeverScrollableScrollPhysics>(),
    );

    expect(find.widgetWithText(OutlinedButton, '위치로'), findsOneWidget);
    expect(find.text('잡기 위치로 이동하세요.'), findsOneWidget);
    expect(find.widgetWithText(FilledButton, '잡기'), findsOneWidget);
    expect(find.widgetWithText(FilledButton, '놓기'), findsOneWidget);
    expect(find.byType(ExpansionTile), findsNothing);
    expect(find.text('MPU6050 상태 대기 중').hitTestable(), findsOneWidget);
    final scrollable = tester.state<ScrollableState>(
      find.byType(Scrollable).first,
    );
    expect(scrollable.position.maxScrollExtent, 0);
    expect(tester.takeException(), isNull);

    await tester.tap(find.byTooltip('PID 설정'));
    await tester.pump(const Duration(milliseconds: 300));
    expect(find.text('조이스틱 직진 판정 범위'), findsOneWidget);
    expect(find.textContaining('|X|가 이 값 이하면'), findsOneWidget);

    await tester.pumpWidget(const SizedBox.shrink());
    await tester.pump(const Duration(seconds: 1));
    controller.dispose();
  });
}
