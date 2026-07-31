import 'package:flutter_test/flutter_test.dart';
import 'package:mobile_retrieval_robot_app/main.dart';

void main() {
  test('마지막 명령 각도를 일반 방향의 현재 펄스로 보간한다', () {
    const pulses = [500, 1500, 2500];

    expect(calibrationPulseForPosition(pulses, 0), 500);
    expect(calibrationPulseForPosition(pulses, 45), 1000);
    expect(calibrationPulseForPosition(pulses, 90), 1500);
    expect(calibrationPulseForPosition(pulses, 135), 2000);
    expect(calibrationPulseForPosition(pulses, 180), 2500);
  });

  test('반전 관절도 현재 위치에 맞는 펄스로 보간한다', () {
    const pulses = [2500, 1500, 500];

    expect(calibrationPulseForPosition(pulses, 45), 2000);
    expect(calibrationPulseForPosition(pulses, 135), 1000);
  });
}
