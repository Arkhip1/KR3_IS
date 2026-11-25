#include "mavlinkmanager.h"
#include "mavlinkmessageparser.h"
#include <QNetworkDatagram>

MavlinkManager::MavlinkManager(TelemetryModel *, QObject *parent)
    : QObject(parent)
{
    m_linkTimer = new QTimer(this);
    m_linkTimer->setSingleShot(true);
    m_linkTimer->setTimerType(Qt::PreciseTimer);
    m_linkTimer->setInterval(3000);
    connect(m_linkTimer, &QTimer::timeout, this, &MavlinkManager::checkLinkTimeout);

    m_ackTimer = new QTimer(this);
    m_ackTimer->setSingleShot(true);
    m_ackTimer->setTimerType(Qt::PreciseTimer);
    m_ackTimer->setInterval(3000);
    connect(m_ackTimer, &QTimer::timeout, this, &MavlinkManager::onAckTimeout);
}

bool MavlinkManager::start(quint16 listenPort)
{
    if (m_udpSocket) {
        stop();
    }
    m_udpSocket = new QUdpSocket(this);
    if (!m_udpSocket->bind(QHostAddress::LocalHost, listenPort, QUdpSocket::DontShareAddress)) {
        const QString error = m_udpSocket->errorString();
        delete m_udpSocket;
        m_udpSocket = nullptr;
        emit errorOccurred(QString("Не удалось открыть UDP-порт %1: %2").arg(listenPort).arg(error));
        return false;
    }
    connect(m_udpSocket, &QUdpSocket::readyRead, this, &MavlinkManager::onReadyRead);

    m_receivedPackets = 0;
    m_lostPackets = 0;
    m_corruptedPackets = 0;
    m_lastSequence = -1;
    m_lastPacketTime = QDateTime();
    emit linkStateChanged(false, QString(), 0);
    emit packetTimeChanged(m_lastPacketTime);
    emit packetReceived(m_receivedPackets, m_lostPackets, m_corruptedPackets);
    emit networkEventLogged(QString("UDP-порт %1 открыт. Ожидание телеметрии БПЛА…").arg(listenPort));
    return true;
}

void MavlinkManager::stop()
{
    m_linkTimer->stop();
    if (m_udpSocket) {
        m_udpSocket->close();
        m_udpSocket->deleteLater();
        m_udpSocket = nullptr;
    }

    m_linkConnected = false;
    m_targetAddress.clear();
    m_targetPort = 0;
    m_lastFlightMode = "UNKNOWN";
    m_systemStatus = 0;
    m_heartbeatLogged = false;
    emit linkStateChanged(false, QString(), 0);
    finishCommand(false, "Команда отменена: UDP-канал закрыт");
    emit vehicleHealthChanged(0);
    emit packetTimeChanged(QDateTime());
    emit networkEventLogged("UDP-канал закрыт оператором");
}

quint64 MavlinkManager::receivedPackets() const { return m_receivedPackets; }
quint64 MavlinkManager::lostPackets() const { return m_lostPackets; }
quint64 MavlinkManager::corruptedPackets() const { return m_corruptedPackets; }

void MavlinkManager::onReadyRead()
{
    while (m_udpSocket && m_udpSocket->hasPendingDatagrams()) {
        const QNetworkDatagram datagram = m_udpSocket->receiveDatagram();
        if (datagram.isValid()) {
            processDatagram(datagram);
        }
    }
}

void MavlinkManager::processDatagram(const QNetworkDatagram &datagram)
{
    const auto address = datagram.senderAddress();
    const auto port = static_cast<quint16>(datagram.senderPort());
    if (m_linkConnected && (address != m_targetAddress || port != m_targetPort)) {
        return;
    }

    const ParsedMavlinkPacket packet = MavlinkMessageParser::parsePacket(datagram.data());
    if (!packet.valid) {
        ++m_corruptedPackets;
        emit packetReceived(m_receivedPackets, m_lostPackets, m_corruptedPackets);
        emit packetRejected(packet.errorReason);
        emit networkEventLogged(QString("Ошибка пакета: %1").arg(packet.errorReason));
        return;
    }

    const bool isTelemetry = packet.msgid == MAVLINK_MSG_ID_HEARTBEAT
                             || packet.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT;
    if ((!m_linkConnected && !isTelemetry) || packet.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
        return;
    }
    if (m_linkConnected && (packet.sysid != m_targetSystemId || packet.compid != m_targetComponentId)) {
        return;
    }
    if (!m_linkConnected) {
        m_targetAddress = address;
        m_targetPort = port;
        m_targetSystemId = packet.sysid;
        m_targetComponentId = packet.compid;
        m_linkConnected = true;
        emit linkStateChanged(true, address.toString(), port);
        emit networkEventLogged(QString("Принят пакет от %1:%2").arg(address.toString()).arg(port));
    }

    ++m_receivedPackets;
    const int advance = (packet.seq - m_lastSequence + 256) % 256;
    const bool advancesSequence = m_lastSequence < 0 || (advance > 0 && advance < 128);
    if (advancesSequence) {
        if (m_lastSequence >= 0 && advance > 1) {
            const int lost = advance - 1;
            m_lostPackets += lost;
            emit networkEventLogged(QString("Потеря пакетов: %1 (ожидался seq=%2, получен seq=%3)")
                                        .arg(lost).arg((m_lastSequence + 1) % 256).arg(packet.seq));
        }
        m_lastSequence = packet.seq;
        m_lastPacketTime = QDateTime::currentDateTime();
        m_linkTimer->start();
        emit packetTimeChanged(m_lastPacketTime);
    }
    emit packetReceived(m_receivedPackets, m_lostPackets, m_corruptedPackets);

    if (packet.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
        handleAck(packet);
        return;
    }
    if (!advancesSequence) {
        return;
    }
    if (packet.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
        if (!m_heartbeatLogged) {
            m_heartbeatLogged = true;
            emit networkEventLogged(QString("MAVLink HEARTBEAT, sysid=%1, compid=%2")
                                        .arg(packet.sysid).arg(packet.compid));
        }
        if (m_lastFlightMode != packet.flightModeName) {
            emit networkEventLogged(QString("Режим полёта: %1").arg(packet.flightModeName));
            m_lastFlightMode = packet.flightModeName;
        }
        if (m_systemStatus != packet.systemStatus) {
            m_systemStatus = packet.systemStatus;
            emit vehicleHealthChanged(m_systemStatus);
            if (m_systemStatus == MAV_STATE_CRITICAL || m_systemStatus == MAV_STATE_EMERGENCY) {
                emit networkEventLogged("БПЛА сообщил об аварии (HEARTBEAT)");
            } else if (m_systemStatus == MAV_STATE_ACTIVE) {
                emit networkEventLogged("БПЛА сообщил о штатной работе (HEARTBEAT)");
            }
        }
    } else {
        emit telemetryReceived(packet.latitude, packet.longitude, packet.altitude,
                               m_lastFlightMode, packet.sysid, packet.compid);
    }
}

void MavlinkManager::handleAck(const ParsedMavlinkPacket &packet)
{
    if (packet.ackCommand != m_pendingCommand
        || (packet.targetSystem && packet.targetSystem != m_systemId)
        || (packet.targetComponent && packet.targetComponent != m_componentId)) {
        return;
    }

    if (packet.ackResult == MAV_RESULT_IN_PROGRESS) {
        m_ackTimer->start();
        emit commandProgress("Команда выполняется (IN_PROGRESS)");
        emit networkEventLogged(QString("COMMAND_ACK (CMD=%1): IN_PROGRESS").arg(packet.ackCommand));
        return;
    }

    QString message;
    switch (packet.ackResult) {
    case MAV_RESULT_ACCEPTED: message = "Команда принята (ACCEPTED)"; break;
    case MAV_RESULT_TEMPORARILY_REJECTED: message = "Команда временно отклонена"; break;
    case MAV_RESULT_DENIED: message = "Команда отклонена (DENIED)"; break;
    case MAV_RESULT_UNSUPPORTED: message = "Команда не поддерживается"; break;
    case MAV_RESULT_FAILED: message = "Ошибка выполнения команды"; break;
    default: message = QString("Результат COMMAND_ACK: %1").arg(packet.ackResult); break;
    }
    emit networkEventLogged(QString("Получен COMMAND_ACK (CMD=%1): %2").arg(packet.ackCommand).arg(message));
    finishCommand(packet.ackResult == MAV_RESULT_ACCEPTED, message);
}

void MavlinkManager::checkLinkTimeout()
{
    m_linkConnected = false;
    m_lastSequence = -1;
    m_heartbeatLogged = false;
    m_lastFlightMode = "UNKNOWN";
    m_systemStatus = 0;
    emit linkStateChanged(false, m_targetAddress.toString(), m_targetPort);
    emit vehicleHealthChanged(0);
    emit networkEventLogged(QString("Связь потеряна (последний пакет: %1)")
                                .arg(m_lastPacketTime.toString("hh:mm:ss")));
    finishCommand(false, "Подтверждение не получено: связь потеряна");
}

void MavlinkManager::onAckTimeout()
{
    emit networkEventLogged(QString("Тайм-аут COMMAND_ACK для команды %1").arg(m_pendingCommand));
    finishCommand(false, "Подтверждение не получено (тайм-аут 3 с)");
}

void MavlinkManager::finishCommand(bool accepted, const QString &message)
{
    if (m_pendingCommand < 0) {
        return;
    }
    m_ackTimer->stop();
    const int command = m_pendingCommand;
    m_pendingCommand = -1;
    emit commandResult(command, accepted, message);
}

void MavlinkManager::sendCommandInternal(quint16 command, float param7, const QString &title)
{
    if (m_pendingCommand >= 0) {
        return;
    }
    if (!m_udpSocket || !m_linkConnected) {
        emit commandResult(command, false, "Невозможно отправить команду: нет связи");
        return;
    }

    const QByteArray datagram = MavlinkMessageParser::buildCommandLongPacket(
        m_txSequence++, m_systemId, m_componentId, m_targetSystemId, m_targetComponentId,
        command, param7);

    if (m_udpSocket->writeDatagram(datagram, m_targetAddress, m_targetPort) != datagram.size()) {
        emit commandResult(command, false, "Ошибка отправки команды по UDP");
        return;
    }

    m_pendingCommand = command;
    m_ackTimer->start();
    emit commandSent(command, param7);
    emit networkEventLogged(QString("Отправлена команда %1 (CMD=%2)").arg(title).arg(command));
}

void MavlinkManager::sendTakeoff(double targetAltitude)
{
    sendCommandInternal(MAV_CMD_NAV_TAKEOFF, static_cast<float>(targetAltitude),
                        QString("ВЗЛЁТ на %1 м").arg(targetAltitude, 0, 'f', 0));
}

void MavlinkManager::sendLand()
{
    sendCommandInternal(MAV_CMD_NAV_LAND, 0.0f, "ПОСАДКА");
}

void MavlinkManager::sendStop()
{
    sendCommandInternal(MAV_CMD_DO_PAUSE_CONTINUE, 0.0f, "СТОП / УДЕРЖАНИЕ");
}
