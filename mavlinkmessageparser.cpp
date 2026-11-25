#include "mavlinkmessageparser.h"
#include <QDataStream>
#include <QIODevice>
#include <cmath>

namespace {

quint32 messageId(const QByteArray &data)
{
    return static_cast<quint8>(data.at(7))
           | (static_cast<quint32>(static_cast<quint8>(data.at(8))) << 8)
           | (static_cast<quint32>(static_cast<quint8>(data.at(9))) << 16);
}

int payloadSize(quint32 id)
{
    switch (id) {
    case MAVLINK_MSG_ID_HEARTBEAT: return 9;
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: return 28;
    case MAVLINK_MSG_ID_COMMAND_LONG: return 33;
    case MAVLINK_MSG_ID_COMMAND_ACK: return 10;
    default: return 0;
    }
}

} // namespace

// Реализация CRC-16/MCRF4XX по стандарту MAVLink 2
quint16 MavlinkMessageParser::calculateCrc16(const QByteArray &dataWithoutMagicAndCrc, quint8 crcExtra)
{
    quint16 crc = 0xFFFF;
    auto accumulate = [&crc](quint8 byte) {
        quint8 tmp = byte ^ static_cast<quint8>(crc & 0xFF);
        tmp ^= (tmp << 4);
        crc = (crc >> 8) ^ (static_cast<quint16>(tmp) << 8)
              ^ (static_cast<quint16>(tmp) << 3)
              ^ (static_cast<quint16>(tmp) >> 4);
    };

    for (char c : dataWithoutMagicAndCrc) {
        accumulate(static_cast<quint8>(c));
    }
    accumulate(crcExtra);
    return crc;
}

quint8 MavlinkMessageParser::getCrcExtra(quint32 msgid)
{
    switch (msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT: return 50;
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: return 104;
    case MAVLINK_MSG_ID_COMMAND_LONG: return 152;
    case MAVLINK_MSG_ID_COMMAND_ACK: return 143;
    default: return 0;
    }
}

QString MavlinkMessageParser::flightModeToString(quint16 mode)
{
    switch (mode) {
    case 0: return "MANUAL";
    case 1: return "STABILIZED";
    case 2: return "ALT_HOLD";
    case 3: return "POS_HOLD";
    case 4: return "GUIDED";
    case 5: return "AUTO";
    case 6: return "LAND";
    default: return "UNKNOWN";
    }
}

bool MavlinkMessageParser::isValid(const QByteArray &data, QString *errorReason)
{
    // Минимальная длина MAVLink 2: 10 байт заголовка + 2 байта CRC = 12 байт
    if (data.size() < 12) {
        if (errorReason) *errorReason = "Слишком короткий MAVLink-пакет (< 12 байт)";
        return false;
    }

    const quint8 magic = static_cast<quint8>(data.at(0));
    if (magic != 0xFD) {
        if (errorReason) *errorReason = QString("Неверный стартовый байт: 0x%1 (ожидался 0xFD)")
                                            .arg(magic, 2, 16, QLatin1Char('0')).toUpper();
        return false;
    }

    if (data.at(2) != 0) {
        if (errorReason) *errorReason = "Неподдерживаемые флаги MAVLink 2";
        return false;
    }

    const quint8 payloadLen = static_cast<quint8>(data.at(1));
    const int expectedSize = 10 + payloadLen + 2;
    if (data.size() != expectedSize) {
        if (errorReason) *errorReason = QString("Несоответствие длины пакета: получено %1 байт, ожидалось %2")
                                            .arg(data.size()).arg(expectedSize);
        return false;
    }

    const quint32 msgid = messageId(data);
    if (payloadSize(msgid) == 0) {
        if (errorReason) *errorReason = QString("Сообщение MAVLink %1 не поддерживается").arg(msgid);
        return false;
    }
    if (payloadLen == 0 || payloadLen > payloadSize(msgid)) {
        if (errorReason) *errorReason = "Недопустимая длина полезной нагрузки MAVLink";
        return false;
    }

    const quint16 receivedCrc = static_cast<quint8>(data.at(expectedSize - 2))
                              | (static_cast<quint16>(static_cast<quint8>(data.at(expectedSize - 1))) << 8);

    const QByteArray crcRegion = data.mid(1, 9 + payloadLen);
    const quint16 computedCrc = calculateCrc16(crcRegion, getCrcExtra(msgid));

    if (receivedCrc != computedCrc) {
        if (errorReason) *errorReason = QString("Ошибка контрольной суммы CRC-16 (получено 0x%1, расчёт 0x%2)")
                                            .arg(receivedCrc, 4, 16, QLatin1Char('0')).toUpper()
                                            .arg(computedCrc, 4, 16, QLatin1Char('0')).toUpper();
        return false;
    }

    return true;
}

ParsedMavlinkPacket MavlinkMessageParser::parsePacket(const QByteArray &data)
{
    ParsedMavlinkPacket result;
    if (!isValid(data, &result.errorReason)) {
        result.valid = false;
        return result;
    }

    result.valid = true;
    const quint8 payloadLen = static_cast<quint8>(data.at(1));
    result.seq = static_cast<quint8>(data.at(4));
    result.sysid = static_cast<quint8>(data.at(5));
    result.compid = static_cast<quint8>(data.at(6));
    result.msgid = messageId(data);

    QByteArray payload = data.mid(10, payloadLen);
    payload.append(QByteArray(payloadSize(result.msgid) - payloadLen, '\0'));
    QDataStream stream(payload);
    stream.setByteOrder(QDataStream::LittleEndian);
    stream.setFloatingPointPrecision(QDataStream::SinglePrecision);

    if (result.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
        quint32 customMode = 0;
        quint8 type = 0, autopilot = 0, baseMode = 0;
        stream >> customMode >> type >> autopilot >> baseMode >> result.systemStatus;
        result.flightModeCode = static_cast<quint16>(customMode);
        result.flightModeName = flightModeToString(result.flightModeCode);
    } else if (result.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
        quint32 timeBootMs = 0;
        qint32 latE7 = 0, lonE7 = 0, altMm = 0, relativeAltMm = 0;
        stream >> timeBootMs >> latE7 >> lonE7 >> altMm >> relativeAltMm;
        result.latitude = latE7 / 1e7;
        result.longitude = lonE7 / 1e7;
        result.altitude = relativeAltMm / 1000.0;
    } else if (result.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
        stream.skipRawData(6 * sizeof(float));
        stream >> result.param7 >> result.command >> result.targetSystem >> result.targetComponent;
    } else if (result.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
        stream >> result.ackCommand >> result.ackResult;
        stream.skipRawData(5);
        stream >> result.targetSystem >> result.targetComponent;
    }

    return result;
}

static QByteArray wrapMavlink2Packet(quint8 seq, quint8 sysid, quint8 compid, quint32 msgid, QByteArray payload)
{
    while (payload.size() > 1 && payload.endsWith('\0')) {
        payload.chop(1);
    }
    QByteArray packet;
    packet.reserve(12 + payload.size());
    packet.append(static_cast<char>(0xFD)); // magic MAVLink 2
    packet.append(static_cast<char>(payload.size())); // len
    packet.append(static_cast<char>(0x00)); // incompat_flags
    packet.append(static_cast<char>(0x00)); // compat_flags
    packet.append(static_cast<char>(seq));
    packet.append(static_cast<char>(sysid));
    packet.append(static_cast<char>(compid));
    packet.append(static_cast<char>(msgid & 0xFF));
    packet.append(static_cast<char>((msgid >> 8) & 0xFF));
    packet.append(static_cast<char>((msgid >> 16) & 0xFF));
    packet.append(payload);

    const quint16 crc = MavlinkMessageParser::calculateCrc16(packet.mid(1), MavlinkMessageParser::getCrcExtra(msgid));
    packet.append(static_cast<char>(crc & 0xFF));
    packet.append(static_cast<char>((crc >> 8) & 0xFF));
    return packet;
}

QByteArray MavlinkMessageParser::buildHeartbeatPacket(quint8 seq, quint8 sysid, quint8 compid,
                                                      quint16 flightMode, quint8 status)
{
    QByteArray payload;
    QDataStream stream(&payload, QIODevice::WriteOnly);
    stream.setByteOrder(QDataStream::LittleEndian);
    stream << static_cast<quint32>(flightMode)
           << static_cast<quint8>(2)  // MAV_TYPE_QUADROTOR
           << static_cast<quint8>(0)  // MAV_AUTOPILOT_GENERIC
           << static_cast<quint8>(1)  // MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
           << status
           << static_cast<quint8>(3); // mavlink_version
    return wrapMavlink2Packet(seq, sysid, compid, MAVLINK_MSG_ID_HEARTBEAT, payload);
}

QByteArray MavlinkMessageParser::buildGlobalPositionPacket(quint8 seq, quint8 sysid, quint8 compid,
                                                           double latitude, double longitude, double altitude,
                                                           quint32 timeBootMs)
{
    QByteArray payload;
    QDataStream stream(&payload, QIODevice::WriteOnly);
    stream.setByteOrder(QDataStream::LittleEndian);
    const qint32 latE7 = static_cast<qint32>(std::llround(latitude * 1e7));
    const qint32 lonE7 = static_cast<qint32>(std::llround(longitude * 1e7));
    const qint32 altMm = static_cast<qint32>(std::llround(altitude * 1000.0));

    stream << timeBootMs << latE7 << lonE7 << altMm << altMm
           << qint16(0) << qint16(0) << qint16(0) << quint16(65535);
    return wrapMavlink2Packet(seq, sysid, compid, MAVLINK_MSG_ID_GLOBAL_POSITION_INT, payload);
}

QByteArray MavlinkMessageParser::buildCommandLongPacket(quint8 seq, quint8 sysid, quint8 compid,
                                                        quint8 targetSysid, quint8 targetCompid,
                                                        quint16 command, float param7)
{
    QByteArray payload;
    QDataStream stream(&payload, QIODevice::WriteOnly);
    stream.setByteOrder(QDataStream::LittleEndian);
    stream.setFloatingPointPrecision(QDataStream::SinglePrecision);
    for (int parameter = 1; parameter <= 6; ++parameter) {
        stream << 0.0f;
    }
    stream << param7 << command << targetSysid << targetCompid << quint8(0);
    return wrapMavlink2Packet(seq, sysid, compid, MAVLINK_MSG_ID_COMMAND_LONG, payload);
}

QByteArray MavlinkMessageParser::buildCommandAckPacket(quint8 seq, quint8 sysid, quint8 compid,
                                                       quint16 command, quint8 result,
                                                       quint8 targetSystem, quint8 targetComponent)
{
    QByteArray payload;
    QDataStream stream(&payload, QIODevice::WriteOnly);
    stream.setByteOrder(QDataStream::LittleEndian);
    stream << command << result << quint8(0) << qint32(0) << targetSystem << targetComponent;
    return wrapMavlink2Packet(seq, sysid, compid, MAVLINK_MSG_ID_COMMAND_ACK, payload);
}
