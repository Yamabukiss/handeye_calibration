#pragma execution_character_set("utf-8")
#include "handeye.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    HandEye w;
    w.setWindowTitle("机器人3D手眼标定软件");
    w.show();
    return a.exec();
}
