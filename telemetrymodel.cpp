#include "telemetrymodel.h"

TelemetryModel::TelemetryModel(QObject *parent)
    : QObject(parent)
{
}

void TelemetryModel::updateTelemetry(double latitude,
                                     double longitude,
                                     double altitude,
                                     const QString &flightMode,
                                     quint8 systemId,
                                     quint8 componentId)
{
    m_latitude = latitude;
    m_longitude = longitude;
    m_altitude = altitude;
    m_flightMode = flightMode;
    m_systemId = systemId;
    m_componentId = componentId;
    m_lastUpdate = QDateTime::currentDateTime();

    emit telemetryChanged(m_latitude, m_longitude, m_altitude, m_flightMode, m_systemId, m_componentId);
}

void TelemetryModel::updateLink(bool connected,
                                const QString &address,
                                quint16 port)
{
    m_linkConnected = connected;
    m_lastAddress = address;
    m_lastPort = port;

    emit linkStateChanged(m_linkConnected, m_lastAddress, m_lastPort);
}

double TelemetryModel::latitude() const { return m_latitude; }
double TelemetryModel::longitude() const { return m_longitude; }
double TelemetryModel::altitude() const { return m_altitude; }
QString TelemetryModel::flightMode() const { return m_flightMode; }
quint8 TelemetryModel::systemId() const { return m_systemId; }
quint8 TelemetryModel::componentId() const { return m_componentId; }
bool TelemetryModel::isLinkConnected() const { return m_linkConnected; }
QString TelemetryModel::lastAddress() const { return m_lastAddress; }
quint16 TelemetryModel::lastPort() const { return m_lastPort; }
QDateTime TelemetryModel::lastUpdate() const { return m_lastUpdate; }
