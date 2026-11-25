#include "uavsimulator.h"
#include "mavlinkmessageparser.h"
#include <QNetworkDatagram>
#include <algorithm>
#include <cmath>

UavSimulator::UavSimulator(QObject *parent)
    : QObject(parent)
{
    m_udpSocket = new QUdpSocket(this);
    connect(m_udpSocket, &QUdpSocket::readyRead, this, &UavSimulator::onReadyRead);

    m_telemetryTimer = new QTimer(this);
    m_telemetryTimer->setInterval(100);
    m_telemetryTimer->setTimerType(Qt::PreciseTimer);
    connect(m_telemetryTimer, &QTimer::timeout, this, &UavSimulator::sendTelemetry);

    m_heartbeatTimer = new QTimer(this);
    m_heartbeatTimer->setInterval(1000);
    connect(m_heartbeatTimer, &QTimer::timeout, this, &UavSimulator::sendHeartbeat);
}

bool UavSimulator::start(const QString &targetAddress, quint16 targetPort)
{
    if (isRunning()) {
        return true;
    }
    if (QHostAddress(targetAddress) != QHostAddress::LocalHost || targetPort == 0) {
        emit simulatorLog("Эмулятору требуется адрес 127.0.0.1 и действующий порт");
        return false;
    }
    // Один сокет отправляет телеметрию и принимает команды на выделенном ОС порту.
    if (m_udpSocket->state() != QAbstractSocket::BoundState
        && !m_udpSocket->bind(QHostAddress(QHostAddress::LocalHost), 0)) {
        emit simulatorLog("Не удалось открыть сокет эмулятора: " + m_udpSocket->errorString());
        return false;
    }

    m_targetAddress = targetAddress;
    m_targetPort = targetPort;
    m_flightMode = 0;
    m_altitude = 0.0;
    m_targetAltitude = 0.0;
    m_latitude = 55.7522200;
    m_longitude = 37.6155600;
    m_angle = 0.0;
    m_uptime.start();
    m_telemetryTimer->start();
    m_heartbeatTimer->start();

    emit simulatorStateChanged(true);
    emit simulatorLog(QString("Эмулятор запущен. Телеметрия: %1:%2, 10 Гц")
                          .arg(m_targetAddress).arg(m_targetPort));
    sendHeartbeat();
    sendTelemetry();
    return true;
}

void UavSimulator::stop()
{
    if (!isRunning()) {
        return;
    }
    m_telemetryTimer->stop();
    m_heartbeatTimer->stop();
    m_corruptNextPacket = false;
    m_omitNextAck = false;
    m_sensorFault = false;
    m_dropRemaining = 0;
    emit simulatorStateChanged(false);
    emit simulatorLog("Эмулятор остановлен (имитация потери связи)");
}

bool UavSimulator::isRunning() const
{
    return m_telemetryTimer->isActive();
}

void UavSimulator::corruptNextPacket()
{
    if (isRunning()) {
        m_corruptNextPacket = true;
        emit simulatorLog("Запланировано повреждение следующей телеметрии");
    }
}

void UavSimulator::dropNextPackets(int count)
{
    if (isRunning()) {
        m_dropRemaining = count;
        emit simulatorLog(QString("Запланирован пропуск следующих %1 пакетов").arg(count));
    }
}

void UavSimulator::omitNextAck()
{
    if (isRunning()) {
        m_omitNextAck = true;
        emit simulatorLog("Следующая команда останется без COMMAND_ACK");
    }
}

void UavSimulator::setSensorFault(bool enabled)
{
    if (isRunning() && m_sensorFault != enabled) {
        m_sensorFault = enabled;
        sendHeartbeat();
    }
}

void UavSimulator::sendHeartbeat()
{
    const auto packet = MavlinkMessageParser::buildHeartbeatPacket(
        m_sequence++, m_systemId, m_componentId, m_flightMode,
        m_sensorFault ? MAV_STATE_CRITICAL : MAV_STATE_ACTIVE);
    m_udpSocket->writeDatagram(packet, QHostAddress(m_targetAddress), m_targetPort);
}

void UavSimulator::sendTelemetry()
{
    if (m_flightMode == 4) {
        m_altitude += std::clamp(m_targetAltitude - m_altitude, -0.8, 0.8);
    } else if (m_flightMode == 6) {
        m_altitude = std::max(0.0, m_altitude - 0.8);
        if (m_altitude == 0.0) {
            m_flightMode = 0;
            sendHeartbeat();
        }
    }

    if (m_altitude > 0.5 && m_flightMode != 3) {
        m_angle += 0.02;
        m_latitude = 55.7522200 + 0.0005 * std::cos(m_angle);
        m_longitude = 37.6155600 + 0.0008 * std::sin(m_angle);
    }
    if (m_sensorFault) {
        return;
    }
    const quint8 seq = m_sequence++;
    if (m_dropRemaining > 0) {
        --m_dropRemaining;
        return;
    }

    QByteArray datagram = MavlinkMessageParser::buildGlobalPositionPacket(
        seq, m_systemId, m_componentId, m_latitude, m_longitude, m_altitude,
        static_cast<quint32>(m_uptime.elapsed()));
    if (m_corruptNextPacket) {
        datagram[datagram.size() - 1] = char(datagram.at(datagram.size() - 1) ^ 0x40);
        m_corruptNextPacket = false;
    }
    m_udpSocket->writeDatagram(datagram, QHostAddress(m_targetAddress), m_targetPort);
}

void UavSimulator::onReadyRead()
{
    while (m_udpSocket->hasPendingDatagrams()) {
        const auto datagram = m_udpSocket->receiveDatagram();
        if (!isRunning() || !datagram.isValid() || datagram.senderAddress() != QHostAddress(m_targetAddress)
            || datagram.senderPort() != m_targetPort) {
            continue;
        }
        const auto packet = MavlinkMessageParser::parsePacket(datagram.data());
        if (!packet.valid || packet.msgid != MAVLINK_MSG_ID_COMMAND_LONG
            || (packet.targetSystem && packet.targetSystem != m_systemId)
            || (packet.targetComponent && packet.targetComponent != m_componentId)) {
            continue;
        }
        if (m_omitNextAck) {
            m_omitNextAck = false;
            emit simulatorLog("Имитирован тайм-аут COMMAND_ACK");
            continue;
        }

        quint8 result = MAV_RESULT_ACCEPTED;
        if (packet.command == MAV_CMD_NAV_TAKEOFF) {
            if (m_sensorFault || !std::isfinite(packet.param7) || packet.param7 < 10 || packet.param7 > 500) {
                result = MAV_RESULT_DENIED;
            } else {
                m_targetAltitude = packet.param7;
                m_flightMode = 4;
                emit simulatorLog(QString("Принята команда TAKEOFF на высоту %1 м").arg(m_targetAltitude));
            }
        } else if (packet.command == MAV_CMD_NAV_LAND) {
            m_targetAltitude = 0.0;
            m_flightMode = 6;
            emit simulatorLog("Принята команда LAND (снижение)");
        } else if (packet.command == MAV_CMD_DO_PAUSE_CONTINUE) {
            m_targetAltitude = m_altitude;
            m_flightMode = 3;
            emit simulatorLog("Принята команда СТОП (удержание позиции)");
        } else {
            result = MAV_RESULT_UNSUPPORTED;
        }

        const auto ack = MavlinkMessageParser::buildCommandAckPacket(
            m_sequence++, m_systemId, m_componentId, packet.command, result, packet.sysid, packet.compid);
        m_udpSocket->writeDatagram(ack, datagram.senderAddress(), datagram.senderPort());
        if (result == MAV_RESULT_ACCEPTED) {
            sendHeartbeat();
        }
    }
}
