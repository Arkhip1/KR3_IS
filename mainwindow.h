#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDateTime>
#include <QElapsedTimer>
#include <array>
#include <QtCharts/QChart>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
QT_CHARTS_USE_NAMESPACE
#endif

class TelemetryModel;
class MavlinkManager;
class UavSimulator;
class QSoundEffect;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    // Управление UDP-каналом НСУ
    void on_btnStartLink_clicked();
    void on_btnStopLink_clicked();

    // Команды управления БПЛА (Вариант 3)
    void on_btnTakeoff_clicked();
    void on_btnLand_clicked();
    void on_btnStop_clicked();

    // Управление встроенным эмулятором БПЛА
    void on_btnStartUav_clicked();
    void on_btnDisconnectUav_clicked();
    void on_btnCorruptPacket_clicked();
    void on_btnDropPackets_clicked();

    // Слоты обновления GUI от модели и сетевого слоя
    void onTelemetryChanged(double latitude,
                            double longitude,
                            double altitude,
                            const QString &flightMode,
                            quint8 systemId,
                            quint8 componentId);

    void onLinkStateChanged(bool connected,
                            const QString &address,
                            quint16 port);

    void onPacketReceived(quint64 received,
                          quint64 lost,
                          quint64 corrupted);

    void onCommandResult(int command,
                         bool accepted,
                         const QString &message);

    void onMavlinkError(const QString &message);
    void appendNetworkEvent(const QString &eventMessage);

private:
    void setupChart();
    void setupTelemetryTable();
    void setCommandButtonsEnabled(bool enabled);
    void updateCommandButtons();
    void setSimulatorControls(bool running);
    enum class Sound { Connected, Accepted, Lost, Fault };
    void playSound(Sound sound);

    Ui::MainWindow *ui;
    TelemetryModel *m_telemetryModel = nullptr;
    MavlinkManager *m_mavlinkManager = nullptr;
    UavSimulator *m_uavSimulator = nullptr;

    QChart *m_chart = nullptr;
    QLineSeries *m_altitudeSeries = nullptr;
    QValueAxis *m_axisX = nullptr;
    QValueAxis *m_axisY = nullptr;

    QElapsedTimer m_plotClock;
    QElapsedTimer m_tableInterval;
    QElapsedTimer m_faultSoundInterval;
    std::array<QSoundEffect *, 4> m_sounds{};
    bool m_linkLost = false;
    bool m_commandPending = false;
};

#endif // MAINWINDOW_H
