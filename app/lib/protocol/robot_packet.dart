/// 앱과 STM32가 공통으로 사용하는 고정 16바이트 패킷을 만든다.
///
/// 패킷 구조:
/// `[Header, Mode, Data0..Data11, Checksum, Tail]`
class RobotPacketCodec {
  const RobotPacketCodec._();

  static const int packetSize = 16;
  static const int dataSize = 12;
  static const int header = 0xAA;
  static const int tail = 0x55;
  static const int checksumIndex = packetSize - 2;
  static const int tailIndex = packetSize - 1;

  /// Mode와 Data를 검증하고 빈 Data 영역을 0으로 채워 패킷을 만든다.
  static List<int> build(int mode, List<int> data) {
    if (mode < 0 ||
        mode > 3 ||
        data.length > dataSize ||
        data.any((value) => value < 0 || value > 0xFF)) {
      throw ArgumentError('잘못된 패킷 값: mode=$mode, data=$data');
    }

    final List<int> payload = List<int>.filled(dataSize, 0);
    payload.setRange(0, data.length, data);
    final List<int> packet = <int>[header, mode, ...payload, 0, tail];
    packet[checksumIndex] = calculateChecksum(packet);
    return packet;
  }

  /// Mode부터 Data11까지 더한 값의 하위 8비트를 반환한다.
  static int calculateChecksum(List<int> packet) {
    if (packet.length != packetSize) {
      throw ArgumentError('패킷 길이는 $packetSize바이트여야 합니다.');
    }
    return packet
            .sublist(1, checksumIndex)
            .fold<int>(0, (sum, value) => sum + value) &
        0xFF;
  }

  /// Header, Tail, Mode와 Checksum이 올바른 패킷인지 확인한다.
  static bool isValid(List<int> packet) {
    return packet.length == packetSize &&
        packet[0] == header &&
        packet[1] >= 0 &&
        packet[1] <= 3 &&
        packet[tailIndex] == tail &&
        packet[checksumIndex] == calculateChecksum(packet);
  }

  /// 수신 버퍼에서 다음 유효 패킷 하나를 꺼낸다.
  ///
  /// 앞부분의 잡음이나 손상된 Header는 버리고, 패킷이 덜 도착했으면 버퍼에
  /// 그대로 남겨 다음 Bluetooth 수신 조각과 이어 붙일 수 있게 한다.
  static List<int>? takeNext(List<int> buffer) {
    while (buffer.isNotEmpty) {
      final int headerIndex = buffer.indexOf(header);
      if (headerIndex < 0) {
        buffer.clear();
        return null;
      }
      if (headerIndex > 0) {
        buffer.removeRange(0, headerIndex);
      }
      if (buffer.length < packetSize) {
        return null;
      }

      final List<int> packet = buffer.sublist(0, packetSize);
      if (isValid(packet)) {
        buffer.removeRange(0, packetSize);
        return packet;
      }
      buffer.removeAt(0);
    }
    return null;
  }
}
