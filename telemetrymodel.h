#ifndef TELEMETRYMODEL_H
#define TELEMETRYMODEL_H

#include <QObject>
#include <QDateTime>
#include <QString>

class TelemetryModel : public QObject
{
    Q_OBJECT
public:
    explicit TelemetryModel(QObject *parent = nullptr);

    void updateTelemetry(double latitude,
                         double longitude,
                         double altitude,
                         const QString &flightMode,
                         quint8 systemId = 1,
                         quint8 componentId = 1);

    void updateLink(bool connected,
                    const QString &address,
                    quint16 port);

    double latitude() const;
    double longitude() const;
    double altitude() const;
    QString flightMode() const;
    quint8 systemId() const;
    quint8 componentId() const;
    bool isLinkConnected() const;
    QString lastAddress() const;
    quint16 lastPort() const;
    QDateTime lastUpdate() const;

signals:
    void telemetryChanged(double latitude,
                          double longitude,
                          double altitude,
                          const QString &flightMode,
                          quint8 systemId,
                          quint8 componentId);

    void linkStateChanged(bool connected,
                          const QString &address,
                          quint16 port);

private:
    double m_latitude = 0.0;
    double m_longitude = 0.0;
    double m_altitude = 0.0;
    QString m_flightMode = "UNKNOWN";
    quint8 m_systemId = 1;
    quint8 m_componentId = 1;

    bool m_linkConnected = false;
    QString m_lastAddress;
    quint16 m_lastPort = 0;
    QDateTime m_lastUpdate;
};

#endif // TELEMETRYMODEL_H
