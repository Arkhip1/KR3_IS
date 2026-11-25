#ifndef MAVLINKMESSAGEPARSER_H
#define MAVLINKMESSAGEPARSER_H

#include <QByteArray>
#include <QString>
#include <QtGlobal>

// Идентификаторы сообщений MAVLink
constexpr quint32 MAVLINK_MSG_ID_HEARTBEAT = 0;
constexpr quint32 MAVLINK_MSG_ID_GLOBAL_POSITION_INT = 33;
constexpr quint32 MAVLINK_MSG_ID_COMMAND_LONG = 76;
constexpr quint32 MAVLINK_MSG_ID_COMMAND_ACK = 77;

// Команды MAV_CMD
constexpr quint16 MAV_CMD_NAV_LAND = 21;
constexpr quint16 MAV_CMD_NAV_TAKEOFF = 22;
constexpr quint16 MAV_CMD_DO_PAUSE_CONTINUE = 193;

constexpr quint8 MAV_STATE_ACTIVE = 4;
constexpr quint8 MAV_STATE_CRITICAL = 5;
constexpr quint8 MAV_STATE_EMERGENCY = 6;

// Результаты выполнения команды MAV_RESULT
constexpr quint8 MAV_RESULT_ACCEPTED = 0;
constexpr quint8 MAV_RESULT_TEMPORARILY_REJECTED = 1;
constexpr quint8 MAV_RESULT_DENIED = 2;
constexpr quint8 MAV_RESULT_UNSUPPORTED = 3;
constexpr quint8 MAV_RESULT_FAILED = 4;
constexpr quint8 MAV_RESULT_IN_PROGRESS = 5;

struct ParsedMavlinkPacket {
    bool valid = false;
    QString errorReason;

    quint8 seq = 0;
    quint8 sysid = 0;
    quint8 compid = 0;
    quint32 msgid = 0;

    // Поля HEARTBEAT / GLOBAL_POSITION_INT
    double latitude = 0.0;
    double longitude = 0.0;
    double altitude = 0.0;
    quint16 flightModeCode = 0;
    QString flightModeName = "UNKNOWN";
    quint8 systemStatus = 0;

    // Поля COMMAND_LONG
    quint16 command = 0;
    float param7 = 0.0f;
    quint8 targetSystem = 0;
    quint8 targetComponent = 0;

    // Поля COMMAND_ACK
    quint16 ackCommand = 0;
    quint8 ackResult = 0;
};

class MavlinkMessageParser
{
public:
    static quint16 calculateCrc16(const QByteArray &dataWithoutMagicAndCrc, quint8 crcExtra);
    static quint8 getCrcExtra(quint32 msgid);

    static bool isValid(const QByteArray &data, QString *errorReason = nullptr);
    static ParsedMavlinkPacket parsePacket(const QByteArray &data);

    static QByteArray buildHeartbeatPacket(quint8 seq, quint8 sysid, quint8 compid,
                                           quint16 flightMode, quint8 status = MAV_STATE_ACTIVE);
    static QByteArray buildGlobalPositionPacket(quint8 seq, quint8 sysid, quint8 compid,
                                                double latitude, double longitude, double altitude,
                                                quint32 timeBootMs);
    static QByteArray buildCommandLongPacket(quint8 seq, quint8 sysid, quint8 compid,
                                             quint8 targetSysid, quint8 targetCompid,
                                             quint16 command, float param7);
    static QByteArray buildCommandAckPacket(quint8 seq, quint8 sysid, quint8 compid,
                                            quint16 command, quint8 result,
                                            quint8 targetSystem = 0, quint8 targetComponent = 0);

    static QString flightModeToString(quint16 mode);
};

#endif // MAVLINKMESSAGEPARSER_H
