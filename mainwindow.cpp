#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "telemetrymodel.h"
#include "mavlinkmanager.h"
#include "mavlinkmessageparser.h"
#include "uavsimulator.h"

#include <QMessageBox>
#include <QTime>
#include <QHeaderView>
#include <QTableWidgetItem>
#include <QFile>
#include <QSoundEffect>
#include <QStyle>
#include <QUrl>
#include <QSignalBlocker>
#include <algorithm>

namespace {

void setState(QLabel *label, const char *state)
{
    label->setProperty("state", QString::fromLatin1(state));
    label->style()->unpolish(label);
    label->style()->polish(label);
    label->update();
}

} // namespace

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setWindowTitle("Наземная станция — MAVLink, вариант 3");
    QFile theme(":/station/station.qss");
    if (theme.open(QIODevice::ReadOnly)) {
        setStyleSheet(QString::fromUtf8(theme.readAll()));
    }

    m_telemetryModel = new TelemetryModel(this);
    m_mavlinkManager = new MavlinkManager(m_telemetryModel, this);
    m_uavSimulator = new UavSimulator(this);

    // Настройка spinTakeoffAltitude по заданию 3
    ui->spinTakeoffAltitude->setMinimum(10.0);
    ui->spinTakeoffAltitude->setMaximum(500.0);
    ui->spinTakeoffAltitude->setSingleStep(10.0);
    ui->spinTakeoffAltitude->setValue(50.0);
    ui->spinTakeoffAltitude->setSuffix(" м");
    ui->spinTakeoffAltitude->setDecimals(0);

    setupChart();
    setupTelemetryTable();

    // Безопасный режим при старте: связи нет -> кнопки команд заблокированы (Вариант 3)
    setCommandButtonsEnabled(false);
    ui->btnStopLink->setEnabled(false);
    setSimulatorControls(false);
    const char *const sounds[] = {"connected", "accepted", "lost", "fault"};
    for (std::size_t index = 0; index < m_sounds.size(); ++index) {
        auto *sound = new QSoundEffect(this);
        sound->setSource(QUrl(QString("qrc:/station/%1.wav").arg(sounds[index])));
        sound->setVolume(0.32f);
        m_sounds[index] = sound;
    }
    connect(ui->btnSound, &QPushButton::toggled, this, [this](bool enabled) {
        ui->btnSound->setText(enabled ? "Звук включён" : "Звук выключен");
        if (!enabled) {
            for (auto *sound : m_sounds) {
                sound->stop();
            }
        }
    });

    // Соединения указательным синтаксисом connect
    connect(m_mavlinkManager, &MavlinkManager::telemetryReceived,
            m_telemetryModel, &TelemetryModel::updateTelemetry);
    connect(m_telemetryModel, &TelemetryModel::telemetryChanged,
            this, &MainWindow::onTelemetryChanged);

    connect(m_mavlinkManager, &MavlinkManager::linkStateChanged,
            m_telemetryModel, &TelemetryModel::updateLink);
    connect(m_telemetryModel, &TelemetryModel::linkStateChanged,
            this, &MainWindow::onLinkStateChanged);

    connect(m_mavlinkManager, &MavlinkManager::packetReceived,
            this, &MainWindow::onPacketReceived);
    connect(m_mavlinkManager, &MavlinkManager::commandResult,
            this, &MainWindow::onCommandResult);
    connect(m_mavlinkManager, &MavlinkManager::commandSent, this, [this] {
        m_commandPending = true;
        ui->labelCommandStatus->setText("Ожидание COMMAND_ACK…");
        setState(ui->labelCommandStatus, "waiting");
        updateCommandButtons();
    });
    connect(m_mavlinkManager, &MavlinkManager::commandProgress, this, [this](const QString &message) {
        ui->labelCommandStatus->setText(message);
        setState(ui->labelCommandStatus, "waiting");
    });
    connect(m_mavlinkManager, &MavlinkManager::packetTimeChanged, this, [this](const QDateTime &time) {
        ui->labelLastPacket->setText("Последний пакет: "
            + (time.isValid() ? time.toString("HH:mm:ss") : QString("—")));
    });
    connect(m_mavlinkManager, &MavlinkManager::packetRejected, this, [this](const QString &reason) {
        ui->labelPacketIssue->setText("Ошибка пакета");
        ui->labelPacketIssue->setToolTip(reason);
        setState(ui->labelPacketIssue, "alert");
        if (!m_faultSoundInterval.isValid() || m_faultSoundInterval.elapsed() >= 1800) {
            m_faultSoundInterval.start();
            playSound(Sound::Fault);
        }
    });
    connect(m_mavlinkManager, &MavlinkManager::vehicleHealthChanged, this, [this](quint8 status) {
        const bool fault = status == MAV_STATE_CRITICAL || status == MAV_STATE_EMERGENCY;
        ui->labelHealth->setText(fault ? "Борт: авария"
            : (status == MAV_STATE_ACTIVE ? "Борт: в норме" : "Борт: —"));
        setState(ui->labelHealth, fault ? "alert" : (status == MAV_STATE_ACTIVE ? "live" : ""));
        if (fault) {
            playSound(Sound::Fault);
        }
    });
    connect(m_mavlinkManager, &MavlinkManager::errorOccurred,
            this, &MainWindow::onMavlinkError);
    connect(m_mavlinkManager, &MavlinkManager::networkEventLogged,
            this, &MainWindow::appendNetworkEvent);

    connect(m_uavSimulator, &UavSimulator::simulatorLog,
            this, &MainWindow::appendNetworkEvent);
    connect(m_uavSimulator, &UavSimulator::simulatorStateChanged,
            this, &MainWindow::setSimulatorControls);
    connect(ui->btnOmitAck, &QPushButton::clicked, m_uavSimulator, &UavSimulator::omitNextAck);
    connect(ui->btnSensorFault, &QPushButton::toggled, m_uavSimulator, &UavSimulator::setSensorFault);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::setupChart()
{
    m_altitudeSeries = new QLineSeries();
    m_altitudeSeries->setName("Высота от точки старта");
    m_altitudeSeries->setPen(QPen(QColor("#278355"), 3.0, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));

    m_chart = new QChart();
    m_chart->addSeries(m_altitudeSeries);
    m_chart->setTitle("Высота от точки старта");
    m_chart->setTitleBrush(QBrush(QColor("#275341")));
    m_chart->setTitleFont(QFont("Segoe UI", 12, QFont::DemiBold));
    m_chart->setBackgroundBrush(QBrush(QColor("#f8fcf6")));
    m_chart->setBackgroundRoundness(34);
    m_chart->setPlotAreaBackgroundBrush(QBrush(QColor("#ffffff")));
    m_chart->setPlotAreaBackgroundVisible(true);
    m_chart->setMargins(QMargins(5, 5, 9, 3));
    m_chart->legend()->hide();

    m_axisX = new QValueAxis();
    m_axisX->setTitleText("Время, с");
    m_axisX->setRange(0, 30);

    m_axisY = new QValueAxis();
    m_axisY->setTitleText("Метры");
    m_axisY->setRange(0, 50);
    for (auto *axis : {m_axisX, m_axisY}) {
        axis->setLabelsColor(QColor("#476654"));
        axis->setLabelsFont(QFont("Segoe UI", 9));
        axis->setTitleBrush(QBrush(QColor("#275341")));
        axis->setTitleFont(QFont("Segoe UI", 9, QFont::DemiBold));
        axis->setGridLineColor(QColor("#dcebd9"));
        axis->setLinePenColor(QColor("#9cc6a1"));
        axis->setLabelFormat("%.0f");
        axis->setTickCount(6);
    }

    m_chart->addAxis(m_axisX, Qt::AlignBottom);
    m_chart->addAxis(m_axisY, Qt::AlignLeft);
    m_altitudeSeries->attachAxis(m_axisX);
    m_altitudeSeries->attachAxis(m_axisY);

    QChart *defaultChart = ui->chartView->chart();
    ui->chartView->setChart(m_chart);
    delete defaultChart;
    ui->chartView->setBackgroundBrush(QBrush(QColor("#eaf5e8")));
    ui->chartView->setRenderHint(QPainter::Antialiasing);
}

void MainWindow::setupTelemetryTable()
{
    // Вариант 3 задания 2: таблица телеметрии
    ui->tableTelemetry->setColumnCount(7);
    ui->tableTelemetry->setHorizontalHeaderLabels({
        "Время", "System ID", "Component ID", "Широта", "Долгота", "Высота, м", "Режим"
    });
    ui->tableTelemetry->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    ui->tableTelemetry->horizontalHeader()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
    ui->tableTelemetry->verticalHeader()->hide();
    ui->tableTelemetry->setSelectionBehavior(QAbstractItemView::SelectRows);
    ui->tableTelemetry->setEditTriggers(QAbstractItemView::NoEditTriggers);
}

void MainWindow::setCommandButtonsEnabled(bool enabled)
{
    // Вариант 3 задания 4: блокировка критичных кнопок при потере связи
    ui->btnTakeoff->setEnabled(enabled);
    ui->btnLand->setEnabled(enabled);
    ui->btnStop->setEnabled(enabled);
}

void MainWindow::updateCommandButtons()
{
    setCommandButtonsEnabled(m_telemetryModel->isLinkConnected() && !m_commandPending);
}

void MainWindow::setSimulatorControls(bool running)
{
    ui->btnStartUav->setEnabled(!running);
    ui->btnDisconnectUav->setEnabled(running);
    ui->btnCorruptPacket->setEnabled(running);
    ui->btnDropPackets->setEnabled(running);
    ui->btnOmitAck->setEnabled(running);
    ui->btnSensorFault->setEnabled(running);
    ui->spinListenPort->setEnabled(!running && ui->btnStartLink->isEnabled());
    if (!running) {
        const QSignalBlocker blocker(ui->btnSensorFault);
        ui->btnSensorFault->setChecked(false);
    }
}

void MainWindow::playSound(Sound sound)
{
    if (!ui->btnSound->isChecked()) {
        return;
    }
    for (auto *effect : m_sounds) {
        effect->stop();
    }
    m_sounds[static_cast<std::size_t>(sound)]->play();
}

void MainWindow::on_btnStartLink_clicked()
{
    const quint16 port = static_cast<quint16>(ui->spinListenPort->value());
    if (m_mavlinkManager->start(port)) {
        ui->btnStartLink->setEnabled(false);
        ui->btnStopLink->setEnabled(true);
        ui->spinListenPort->setEnabled(false);
        ui->labelLinkStatus->setText("Ожидание телеметрии");
        setState(ui->labelLinkStatus, "waiting");
        ui->labelPacketIssue->clear();
        ui->tableTelemetry->setRowCount(0);
        m_tableInterval.invalidate();
    }
}

void MainWindow::on_btnStopLink_clicked()
{
    m_mavlinkManager->stop();
    ui->btnStartLink->setEnabled(true);
    ui->btnStopLink->setEnabled(false);
    ui->spinListenPort->setEnabled(!m_uavSimulator->isRunning());
    setCommandButtonsEnabled(false);
}

void MainWindow::on_btnTakeoff_clicked()
{
    if (!m_telemetryModel->isLinkConnected()) {
        QMessageBox::warning(
            this,
            "Команда недоступна",
            "Невозможно отправить команду взлёта: нет связи с БПЛА.");
        return;
    }

    // Вариант 3 задания 3: подтверждение оператором перед отправкой команды взлёта
    const double altitude = ui->spinTakeoffAltitude->value();
    const QMessageBox::StandardButton answer = QMessageBox::question(
        this,
        "Подтверждение команды",
        "Отправить команду взлёта на высоту "
            + QString::number(altitude, 'f', 0)
            + " м?",
        QMessageBox::Yes | QMessageBox::No,
        QMessageBox::No);

    if (answer != QMessageBox::Yes) {
        return;
    }

    m_mavlinkManager->sendTakeoff(altitude);
}

void MainWindow::on_btnLand_clicked()
{
    if (!m_telemetryModel->isLinkConnected()) {
        QMessageBox::warning(this, "Команда недоступна", "Нет связи с БПЛА.");
        return;
    }
    m_mavlinkManager->sendLand();
}

void MainWindow::on_btnStop_clicked()
{
    if (!m_telemetryModel->isLinkConnected()) {
        QMessageBox::warning(this, "Команда недоступна", "Нет связи с БПЛА.");
        return;
    }
    m_mavlinkManager->sendStop();
}

void MainWindow::on_btnStartUav_clicked()
{
    const quint16 targetPort = static_cast<quint16>(ui->spinListenPort->value());
    m_uavSimulator->start("127.0.0.1", targetPort);
}

void MainWindow::on_btnDisconnectUav_clicked()
{
    m_uavSimulator->stop();
}

void MainWindow::on_btnCorruptPacket_clicked()
{
    m_uavSimulator->corruptNextPacket();
}

void MainWindow::on_btnDropPackets_clicked()
{
    m_uavSimulator->dropNextPackets(2);
}

void MainWindow::onTelemetryChanged(double latitude,
                                    double longitude,
                                    double altitude,
                                    const QString &flightMode,
                                    quint8 systemId,
                                    quint8 componentId)
{
    ui->labelLatitude->setText(QString::number(latitude, 'f', 7));
    ui->labelLongitude->setText(QString::number(longitude, 'f', 7));
    ui->labelAltitude->setText(QString::number(altitude, 'f', 1) + " м");
    ui->labelFlightMode->setText(flightMode);

    // Обновление графика (не более 300 точек)
    const double seconds = m_plotClock.elapsed() / 1000.0;
    m_altitudeSeries->append(seconds, altitude);
    if (m_altitudeSeries->count() > 300) {
        m_altitudeSeries->remove(0);
    }
    if (seconds > 30.0) {
        m_axisX->setRange(seconds - 30.0, seconds);
    }
    if (altitude + 20.0 > m_axisY->max()) {
        m_axisY->setMax(altitude + 30.0);
    }
    if (altitude < m_axisY->min()) {
        m_axisY->setMin(altitude - 10.0);
    }

    // Вариант 3 задания 2: добавление строки в таблицу не чаще 1 раза в секунду, лимит 200 строк
    if (!m_tableInterval.isValid() || m_tableInterval.elapsed() >= 1000) {
        m_tableInterval.start();
        const auto now = QTime::currentTime();

        const int row = ui->tableTelemetry->rowCount();
        ui->tableTelemetry->insertRow(row);
        ui->tableTelemetry->setItem(row, 0, new QTableWidgetItem(now.toString("hh:mm:ss")));
        ui->tableTelemetry->setItem(row, 1, new QTableWidgetItem(QString::number(systemId)));
        ui->tableTelemetry->setItem(row, 2, new QTableWidgetItem(QString::number(componentId)));
        ui->tableTelemetry->setItem(row, 3, new QTableWidgetItem(QString::number(latitude, 'f', 7)));
        ui->tableTelemetry->setItem(row, 4, new QTableWidgetItem(QString::number(longitude, 'f', 7)));
        ui->tableTelemetry->setItem(row, 5, new QTableWidgetItem(QString::number(altitude, 'f', 1)));
        ui->tableTelemetry->setItem(row, 6, new QTableWidgetItem(flightMode));

        if (ui->tableTelemetry->rowCount() > 200) {
            ui->tableTelemetry->removeRow(0);
        }
        ui->tableTelemetry->scrollToBottom();
    }
}

void MainWindow::onLinkStateChanged(bool connected,
                                    const QString &address,
                                    quint16 port)
{
    if (connected) {
        updateCommandButtons();
        m_plotClock.start();
        m_altitudeSeries->clear();
        m_axisX->setRange(0, 30);
        m_axisY->setRange(0, 50);
        if (m_linkLost) {
            ui->labelLinkStatus->setText("Связь восстановлена");
            appendNetworkEvent("Связь восстановлена");
        } else {
            ui->labelLinkStatus->setText("Связь установлена");
        }
        m_linkLost = false;
        setState(ui->labelLinkStatus, "live");
        ui->labelSender->setText(address + ":" + QString::number(port));
        playSound(Sound::Connected);
    } else {
        updateCommandButtons();
        m_linkLost = !address.isEmpty();
        ui->labelLinkStatus->setText(m_linkLost ? "Связь потеряна" : "Связь отсутствует");
        setState(ui->labelLinkStatus, "alert");
        ui->labelSender->setText(address.isEmpty() ? "—" : (address + ":" + QString::number(port)));
        for (auto *label : {ui->labelLatitude, ui->labelLongitude, ui->labelAltitude, ui->labelFlightMode}) {
            label->setText("—");
        }
        if (m_linkLost) {
            playSound(Sound::Lost);
        } else {
            m_altitudeSeries->clear();
            m_axisX->setRange(0, 30);
            m_axisY->setRange(0, 50);
        }
    }
}

void MainWindow::onPacketReceived(quint64 received, quint64 lost, quint64 corrupted)
{
    ui->labelPackets->setText(
        QString("Принято: %1   ·   Пропуски: %2   ·   Ошибки: %3")
            .arg(received)
            .arg(lost)
            .arg(corrupted));
}

void MainWindow::onCommandResult(int command,
                                 bool accepted,
                                 const QString &message)
{
    m_commandPending = false;
    updateCommandButtons();
    ui->labelCommandStatus->setText(message);
    setState(ui->labelCommandStatus, accepted ? "live" : "alert");
    if (!m_linkLost && m_telemetryModel->isLinkConnected()) {
        playSound(accepted ? Sound::Accepted : Sound::Fault);
    }

    const QString record = QString("[%1] COMMAND %2: %3")
                               .arg(QTime::currentTime().toString("hh:mm:ss"))
                               .arg(command)
                               .arg(message);
    ui->listCommandLog->insertItem(0, record);
    if (ui->listCommandLog->count() > 200) {
        delete ui->listCommandLog->takeItem(200);
    }
}

void MainWindow::onMavlinkError(const QString &message)
{
    ui->labelLinkStatus->setText("Ошибка: " + message);
    setState(ui->labelLinkStatus, "alert");
    appendNetworkEvent("Ошибка канала: " + message);
}

void MainWindow::appendNetworkEvent(const QString &eventMessage)
{
    const QString entry = QString("[%1] %2")
                              .arg(QTime::currentTime().toString("hh:mm:ss"))
                              .arg(eventMessage);
    ui->listNetworkLog->insertItem(0, entry);
    if (ui->listNetworkLog->count() > 200) {
        delete ui->listNetworkLog->takeItem(ui->listNetworkLog->count() - 1);
    }
}
