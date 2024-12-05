/********************************************************************************
** Form generated from reading UI file 'verification.ui'
**
** Created by: Qt User Interface Compiler version 5.15.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_VERIFICATION_H
#define UI_VERIFICATION_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/QTextBrowser>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Verification
{
public:
    QTableWidget *table_matrix;
    QTableWidget *table_base;
    QPushButton *pushButton;
    QLabel *label;
    QLabel *label_2;
    QTableWidget *table_error;
    QLabel *label_3;
    QPushButton *pushButton_2;
    QTableWidget *table_cam;
    QLabel *label_4;
    QTextBrowser *textBrowser_log;
    QPushButton *pushButton_3;

    void setupUi(QWidget *Verification)
    {
        if (Verification->objectName().isEmpty())
            Verification->setObjectName(QString::fromUtf8("Verification"));
        Verification->resize(1022, 676);
        Verification->setBaseSize(QSize(0, 0));
        table_matrix = new QTableWidget(Verification);
        if (table_matrix->columnCount() < 4)
            table_matrix->setColumnCount(4);
        if (table_matrix->rowCount() < 4)
            table_matrix->setRowCount(4);
        table_matrix->setObjectName(QString::fromUtf8("table_matrix"));
        table_matrix->setEnabled(true);
        table_matrix->setGeometry(QRect(70, 70, 521, 181));
        table_matrix->setBaseSize(QSize(0, 0));
        QFont font;
        font.setFamily(QString::fromUtf8("Times New Roman"));
        font.setPointSize(10);
        table_matrix->setFont(font);
        table_matrix->setRowCount(4);
        table_matrix->setColumnCount(4);
        table_base = new QTableWidget(Verification);
        if (table_base->columnCount() < 3)
            table_base->setColumnCount(3);
        if (table_base->rowCount() < 5)
            table_base->setRowCount(5);
        table_base->setObjectName(QString::fromUtf8("table_base"));
        table_base->setGeometry(QRect(70, 300, 391, 161));
        table_base->setFont(font);
        table_base->setRowCount(5);
        table_base->setColumnCount(3);
        pushButton = new QPushButton(Verification);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));
        pushButton->setGeometry(QRect(470, 320, 101, 51));
        QFont font1;
        font1.setFamily(QString::fromUtf8("\351\273\221\344\275\223"));
        font1.setPointSize(11);
        pushButton->setFont(font1);
        label = new QLabel(Verification);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(280, 10, 101, 31));
        label->setFont(font);
        label->setAlignment(Qt::AlignCenter);
        label_2 = new QLabel(Verification);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(220, 270, 101, 31));
        label_2->setFont(font);
        label_2->setAlignment(Qt::AlignCenter);
        table_error = new QTableWidget(Verification);
        if (table_error->columnCount() < 3)
            table_error->setColumnCount(3);
        if (table_error->rowCount() < 5)
            table_error->setRowCount(5);
        table_error->setObjectName(QString::fromUtf8("table_error"));
        table_error->setGeometry(QRect(330, 500, 391, 161));
        table_error->setFont(font);
        table_error->setEditTriggers(QAbstractItemView::NoEditTriggers);
        table_error->setRowCount(5);
        table_error->setColumnCount(3);
        label_3 = new QLabel(Verification);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(480, 470, 101, 31));
        label_3->setFont(font1);
        label_3->setAlignment(Qt::AlignCenter);
        pushButton_2 = new QPushButton(Verification);
        pushButton_2->setObjectName(QString::fromUtf8("pushButton_2"));
        pushButton_2->setGeometry(QRect(470, 390, 101, 51));
        pushButton_2->setFont(font1);
        table_cam = new QTableWidget(Verification);
        if (table_cam->columnCount() < 3)
            table_cam->setColumnCount(3);
        if (table_cam->rowCount() < 5)
            table_cam->setRowCount(5);
        table_cam->setObjectName(QString::fromUtf8("table_cam"));
        table_cam->setGeometry(QRect(580, 300, 391, 161));
        table_cam->setFont(font);
        table_cam->setRowCount(5);
        table_cam->setColumnCount(3);
        label_4 = new QLabel(Verification);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setGeometry(QRect(730, 270, 101, 31));
        label_4->setFont(font);
        label_4->setAlignment(Qt::AlignCenter);
        textBrowser_log = new QTextBrowser(Verification);
        textBrowser_log->setObjectName(QString::fromUtf8("textBrowser_log"));
        textBrowser_log->setGeometry(QRect(620, 60, 381, 191));
        textBrowser_log->setFont(font1);
        textBrowser_log->setReadOnly(true);
        pushButton_3 = new QPushButton(Verification);
        pushButton_3->setObjectName(QString::fromUtf8("pushButton_3"));
        pushButton_3->setGeometry(QRect(430, 10, 131, 51));
        pushButton_3->setFont(font1);

        retranslateUi(Verification);

        QMetaObject::connectSlotsByName(Verification);
    } // setupUi

    void retranslateUi(QWidget *Verification)
    {
        Verification->setWindowTitle(QCoreApplication::translate("Verification", "Form", nullptr));
        pushButton->setText(QCoreApplication::translate("Verification", "\345\256\214\346\210\220\350\276\223\345\205\245", nullptr));
        label->setText(QCoreApplication::translate("Verification", "TBase2Cam", nullptr));
        label_2->setText(QCoreApplication::translate("Verification", "PBase", nullptr));
        label_3->setText(QCoreApplication::translate("Verification", "\347\273\235\345\257\271\350\257\257\345\267\256", nullptr));
        pushButton_2->setText(QCoreApplication::translate("Verification", "\346\270\205\347\251\272\350\276\223\345\205\245", nullptr));
        label_4->setText(QCoreApplication::translate("Verification", "PCam", nullptr));
        pushButton_3->setText(QCoreApplication::translate("Verification", "\350\256\276\347\275\256\346\240\207\345\256\232\347\237\251\351\230\265", nullptr));
    } // retranslateUi

};

namespace Ui {
    class Verification: public Ui_Verification {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_VERIFICATION_H
