#include "uavsimulator.h"

#include <QApplication>
#include <QFile>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QListWidget>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSpinBox>
#include <QTime>
#include <QVBoxLayout>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    QWidget window;
    window.setObjectName("centralwidget");
    window.setWindowTitle("Эмулятор БПЛА — MAVLink, вариант 3");
    window.resize(740, 360);

    QFile theme(":/station/station.qss");
    if (theme.open(QIODevice::ReadOnly)) {
        window.setStyleSheet(QString::fromUtf8(theme.readAll()));
    }

    auto *layout = new QHBoxLayout(&window);
    layout->setContentsMargins(14, 14, 14, 14);
    layout->setSpacing(12);
    auto *controlPanel = new QGroupBox("Управление эмулятором", &window);
    auto *controlLayout = new QVBoxLayout(controlPanel);
    layout->addWidget(controlPanel);
    auto *portRow = new QFormLayout();
    auto *port = new QSpinBox();
    port->setRange(1024, 65535);
    port->setValue(14540);
    portRow->addRow("Порт НСУ · 127.0.0.1", port);
    controlLayout->addLayout(portRow);

    auto *controls = new QGridLayout();
    auto *start = new QPushButton("Запустить БПЛА");
    start->setObjectName("btnStartUav");
    auto *stop = new QPushButton("Отключить БПЛА");
    auto *corrupt = new QPushButton("Повредить пакет");
    auto *drop = new QPushButton("Пропустить 2 пакета");
    auto *omitAck = new QPushButton("Пропустить ACK");
    auto *sensor = new QPushButton("Сбой датчика");
    sensor->setCheckable(true);
    controls->addWidget(start, 0, 0);
    controls->addWidget(stop, 0, 1);
    controls->addWidget(corrupt, 1, 0);
    controls->addWidget(drop, 1, 1);
    controls->addWidget(omitAck, 2, 0);
    controls->addWidget(sensor, 2, 1);
    controlLayout->addLayout(controls);
    controlLayout->addStretch();

    auto *logPanel = new QGroupBox("События эмулятора", &window);
    auto *logLayout = new QVBoxLayout(logPanel);
    auto *log = new QListWidget();
    log->setAlternatingRowColors(true);
    log->setWordWrap(true);
    logLayout->addWidget(log);
    layout->addWidget(logPanel, 1);

    UavSimulator simulator;
    const auto setRunning = [=](bool running) {
        port->setEnabled(!running);
        start->setEnabled(!running);
        for (auto *button : {stop, corrupt, drop, omitAck, sensor}) {
            button->setEnabled(running);
        }
        if (!running) {
            const QSignalBlocker blocker(sensor);
            sensor->setChecked(false);
        }
    };
    setRunning(false);

    QObject::connect(start, &QPushButton::clicked, &simulator, [&] {
        simulator.start("127.0.0.1", static_cast<quint16>(port->value()));
    });
    QObject::connect(stop, &QPushButton::clicked, &simulator, &UavSimulator::stop);
    QObject::connect(corrupt, &QPushButton::clicked, &simulator, &UavSimulator::corruptNextPacket);
    QObject::connect(drop, &QPushButton::clicked, &simulator, [&] { simulator.dropNextPackets(); });
    QObject::connect(omitAck, &QPushButton::clicked, &simulator, &UavSimulator::omitNextAck);
    QObject::connect(sensor, &QPushButton::toggled, &simulator, &UavSimulator::setSensorFault);
    QObject::connect(&simulator, &UavSimulator::simulatorStateChanged, &window, setRunning);
    QObject::connect(&simulator, &UavSimulator::simulatorLog, log, [log](const QString &message) {
        log->insertItem(0, QString("[%1] %2").arg(QTime::currentTime().toString("HH:mm:ss"), message));
        if (log->count() > 200) {
            delete log->takeItem(200);
        }
    });

    window.show();
    return app.exec();
}
