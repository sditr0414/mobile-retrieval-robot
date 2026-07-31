import 'package:flutter_test/flutter_test.dart';
import 'package:mobile_retrieval_robot_app/protocol/robot_packet.dart';

void main() {
  group('RobotPacketCodec', () {
    test('16바이트 패킷과 체크섬을 만든다', () {
      final packet = RobotPacketCodec.build(0, <int>[
        0,
        120,
        0,
        120,
        0,
        1,
        1,
        1,
      ]);

      expect(packet.length, RobotPacketCodec.packetSize);
      expect(packet.first, RobotPacketCodec.header);
      expect(packet.last, RobotPacketCodec.tail);
      expect(packet.sublist(2, 10), <int>[0, 120, 0, 120, 0, 1, 1, 1]);
      expect(RobotPacketCodec.isValid(packet), isTrue);
    });

    test('잡음과 손상 패킷 뒤에서 다음 정상 패킷을 복구한다', () {
      final bad = RobotPacketCodec.build(1, <int>[90, 90, 90]);
      bad[RobotPacketCodec.checksumIndex] ^= 0x01;
      final good = RobotPacketCodec.build(2, <int>[3, 1]);
      final buffer = <int>[1, 2, 3, ...bad, ...good];

      expect(RobotPacketCodec.takeNext(buffer), good);
      expect(buffer, isEmpty);
    });

    test('잘린 패킷은 다음 수신을 위해 버퍼에 남긴다', () {
      final packet = RobotPacketCodec.build(3, <int>[1]);
      final buffer = packet.sublist(0, 8);

      expect(RobotPacketCodec.takeNext(buffer), isNull);
      expect(buffer, packet.sublist(0, 8));
    });

    test('로봇팔 Transaction ID와 50~100% 속도 필드를 보존한다', () {
      final packet = RobotPacketCodec.build(1, <int>[
        90,
        90,
        90,
        90,
        90,
        0,
        0,
        7,
        75,
      ]);

      expect(packet[9], 7);
      expect(packet[10], 75);
      expect(RobotPacketCodec.isValid(packet), isTrue);
    });

    test('Gripper 유지 이동 자세 명령도 고정 16바이트를 사용한다', () {
      final packet = RobotPacketCodec.build(1, <int>[
        0,
        0,
        0,
        0,
        0,
        0,
        3,
        21,
        100,
      ]);

      expect(packet.length, RobotPacketCodec.packetSize);
      expect(packet.sublist(2, 11), <int>[0, 0, 0, 0, 0, 0, 3, 21, 100]);
      expect(RobotPacketCodec.isValid(packet), isTrue);
    });

    test('티칭 완료 ACK의 속도 경고 바이트를 보존한다', () {
      final packet = RobotPacketCodec.build(2, <int>[2, 3, 1, 1]);

      expect(packet[4], 1);
      expect(packet[5], 1);
      expect(RobotPacketCodec.isValid(packet), isTrue);
    });

    test('티칭 이름 업로드와 조회도 고정 16바이트를 사용한다', () {
      final start = RobotPacketCodec.build(2, <int>[4, 1, 1, 2, 9]);
      final namePart = RobotPacketCodec.build(2, <int>[
        4,
        5,
        1,
        0,
        0x53,
        0x68,
        0x65,
        0x6C,
        0x66,
        0,
        0,
        0,
      ]);
      final query = RobotPacketCodec.build(2, <int>[5, 1, 7]);
      final response = RobotPacketCodec.build(2, <int>[
        5,
        1,
        0,
        5,
        7,
        0x53,
        0x68,
        0x65,
        0x6C,
        0x66,
      ]);

      expect(start.length, RobotPacketCodec.packetSize);
      expect(namePart.length, RobotPacketCodec.packetSize);
      expect(query.length, RobotPacketCodec.packetSize);
      expect(response.length, RobotPacketCodec.packetSize);
      expect(response.sublist(2, 12), <int>[
        5,
        1,
        0,
        5,
        7,
        0x53,
        0x68,
        0x65,
        0x6C,
        0x66,
      ]);
    });

    test('편집본 재생과 저장 시퀀스 조회도 고정 16바이트를 사용한다', () {
      final preview = RobotPacketCodec.build(2, <int>[7, 3, 75]);
      final query = RobotPacketCodec.build(2, <int>[6, 3, 42]);

      expect(preview, hasLength(16));
      expect(preview.sublist(2, 5), <int>[7, 3, 75]);
      expect(RobotPacketCodec.isValid(preview), isTrue);
      expect(query, hasLength(16));
      expect(query.sublist(2, 5), <int>[6, 3, 42]);
      expect(RobotPacketCodec.isValid(query), isTrue);
    });

    test('원점 보정 미리보기의 속도와 요청 ID를 16바이트 안에 보존한다', () {
      final request = RobotPacketCodec.build(3, <int>[5, 4, 0xDC, 0x05, 75, 9]);
      final response = RobotPacketCodec.build(3, <int>[
        5,
        4,
        1,
        0,
        0xDC,
        0x05,
        9,
      ]);

      expect(request.length, RobotPacketCodec.packetSize);
      expect(request.sublist(2, 8), <int>[5, 4, 0xDC, 0x05, 75, 9]);
      expect(response.sublist(2, 9), <int>[5, 4, 1, 0, 0xDC, 0x05, 9]);
      expect(RobotPacketCodec.isValid(request), isTrue);
      expect(RobotPacketCodec.isValid(response), isTrue);
    });

    test('53바이트 설정의 마지막 호환 조각도 16바이트를 유지한다', () {
      final lastPart = RobotPacketCodec.build(3, <int>[
        2,
        5,
        180,
        90,
        150,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
      ]);
      final commit = RobotPacketCodec.build(3, <int>[3, 53, 0x34, 0x12]);

      expect(lastPart.length, RobotPacketCodec.packetSize);
      expect(lastPart.sublist(2, 7), <int>[2, 5, 180, 90, 150]);
      expect(commit[3], 53);
      expect(RobotPacketCodec.isValid(lastPart), isTrue);
      expect(RobotPacketCodec.isValid(commit), isTrue);
    });

    test('범위를 벗어난 값은 패킷으로 만들지 않는다', () {
      expect(
        () => RobotPacketCodec.build(4, const <int>[]),
        throwsArgumentError,
      );
      expect(
        () => RobotPacketCodec.build(0, const <int>[256]),
        throwsArgumentError,
      );
    });
  });
}
