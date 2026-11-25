#ifndef MAVLINKMANAGER_H
#define MAVLINKMANAGER_H

#include <QObject>
#include <QUdpSocket>
#include <QHostAddress>
#include <QTimer>
#include <QDateTime>

class TelemetryModel;
class QNetworkDatagram;
struct ParsedMavlinkPacket;

class MavlinkManager : public QObject
{
    Q_OBJECT
public:
    explicit MavlinkManager(TelemetryModel *telemetry, QObject *parent = nullptr);

    bool start(quint16 listenPort);
    void stop();

    quint64 receivedPackets() const;
    quint64 lostPackets() const;
    quint64 corruptedPackets() const;

public slots:
    void sendTakeoff(double targetAltitude);
    void sendLand();
    void sendStop();

signals:
    void telemetryReceived(double latitude,
                           double longitude,
                           double altitude,
                           const QString &flightMode,
                           quint8 systemId,
                           quint8 componentId);

    void linkStateChanged(bool connected,
                          const QString &address,
                          quint16 port);

    void packetReceived(quint64 received,
                        quint64 lost,
                        quint64 corrupted);

    void commandSent(int command, double param7);
    void commandProgress(const QString &message);
    void packetTimeChanged(const QDateTime &time);
    void packetRejected(const QString &message);
    void vehicleHealthChanged(quint8 status);

    void commandResult(int command,
                       bool accepted,
                       const QString &message);

    void errorOccurred(const QString &message);

    // Событие для журнала сетевых событий (Вариант 3)
    void networkEventLogged(const QString &eventMessage);

private slots:
    void onReadyRead();
    void checkLinkTimeout();
    void onAckTimeout();

private:
    void processDatagram(const QNetworkDatagram &datagram);
    void handleAck(const ParsedMavlinkPacket &packet);
    void finishCommand(bool accepted, const QString &message);
    void sendCommandInternal(quint16 command, float param7, const QString &humanTitle);

    QUdpSocket *m_udpSocket = nullptr;
    QTimer *m_linkTimer = nullptr;
    QTimer *m_ackTimer = nullptr;

    QHostAddress m_targetAddress;
    quint16 m_targetPort = 0;

    QDateTime m_lastPacketTime;
    bool m_linkConnected = false;

    quint64 m_receivedPackets = 0;
    quint64 m_lostPackets = 0;
    quint64 m_corruptedPackets = 0;
    qint64 m_lastSequence = -1;

    quint8 m_systemId = 255;    // GCS sysid
    quint8 m_componentId = 190; // GCS compid
    quint8 m_targetSystemId = 1;
    quint8 m_targetComponentId = 1;
    quint8 m_txSequence = 0;

    QString m_lastFlightMode = "UNKNOWN";
    quint8 m_systemStatus = 0;
    bool m_heartbeatLogged = false;
    int m_pendingCommand = -1;
};

#endif // MAVLINKMANAGER_H
