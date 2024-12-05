#pragma execution_character_set("utf-8")
#include "handeye.h"

#include <QApplication>
#include <QDesktopWidget>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    HandEye w;
    QRect screenGeometry = QApplication::desktop()->availableGeometry();

    w.setMaximumSize(screenGeometry.width(), screenGeometry.height());

    w.setMinimumSize(800, 600);
    w.setWindowTitle("机器人3D手眼标定软件");
    w.show();
    return a.exec();
}
