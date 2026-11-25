#ifndef UAVSIMULATOR_H
#define UAVSIMULATOR_H

#include <QObject>
#include <QUdpSocket>
#include <QTimer>
#include <QHostAddress>
#include <QElapsedTimer>

class UavSimulator : public QObject
{
    Q_OBJECT
public:
    explicit UavSimulator(QObject *parent = nullptr);

    bool start(const QString &targetAddress, quint16 targetPort);
    void stop();
    bool isRunning() const;

public slots:
    void sendTelemetry();
    void sendHeartbeat();
    void corruptNextPacket();
    void dropNextPackets(int count = 2);
    void omitNextAck();
    void setSensorFault(bool enabled);

signals:
    void simulatorStateChanged(bool running);
    void simulatorLog(const QString &message);

private slots:
    void onReadyRead();

private:
    QUdpSocket *m_udpSocket = nullptr;
    QTimer *m_telemetryTimer = nullptr;
    QTimer *m_heartbeatTimer = nullptr;

    QString m_targetAddress = "127.0.0.1";
    quint16 m_targetPort = 14540;

    double m_altitude = 0.0;
    double m_targetAltitude = 0.0;
    double m_latitude = 55.7522200;
    double m_longitude = 37.6155600;
    double m_angle = 0.0;
    quint16 m_flightMode = 0; // 0=MANUAL, 4=GUIDED, 6=LAND, 3=POS_HOLD
    QElapsedTimer m_uptime;

    quint8 m_sequence = 0;
    quint8 m_systemId = 1;
    quint8 m_componentId = 1;

    bool m_corruptNextPacket = false;
    bool m_omitNextAck = false;
    bool m_sensorFault = false;
    int m_dropRemaining = 0;
};

#endif // UAVSIMULATOR_H
