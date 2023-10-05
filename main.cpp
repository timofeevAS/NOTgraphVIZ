#include "mainwindow.h"

#include <QApplication>

int main(int argc, char *argv[])
{

    QApplication a(argc, argv);
    QApplication::setWindowIcon(QIcon(":/icon_to_app.png"));
    MainWindow w;
    w.show();
    return a.exec();
}
