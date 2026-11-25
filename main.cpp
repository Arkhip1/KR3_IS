#include "mainwindow.h"
#include <QApplication>
#include <QScreen>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    const QSize available = a.primaryScreen()->availableGeometry().size() - QSize(32, 48);
    w.resize(w.size().boundedTo(available));
    w.show();
    return a.exec();
}
