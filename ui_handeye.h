/********************************************************************************
** Form generated from reading UI file 'handeye.ui'
**
** Created by: Qt User Interface Compiler version 5.15.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_HANDEYE_H
#define UI_HANDEYE_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QScrollArea>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/QTextBrowser>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_HandEye
{
public:
    QWidget *centralwidget;
    QTextBrowser *textBrowser_log;
    QPushButton *scan_button;
    QPushButton *connect_button;
    QLabel *label_gray;
    QScrollArea *scrollArea;
    QWidget *scrollAreaWidgetContents;
    QTableWidget *tableWidget;
    QPushButton *input_button;
    QPushButton *calculate_button;
    QPushButton *save_button;
    QPushButton *update_button;
    QPushButton *test_button;
    QPushButton *drop_button;
    QPushButton *reset_button;
    QWidget *layoutWidget;
    QHBoxLayout *horizontalLayout_11;
    QLabel *batch_label_title;
    QLabel *batch_label;
    QWidget *layoutWidget1;
    QHBoxLayout *horizontalLayout;
    QLabel *cut_titile;
    QLineEdit *cut_edit;
    QWidget *layoutWidget2;
    QGridLayout *gridLayout;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label;
    QLineEdit *lineEdit;
    QHBoxLayout *horizontalLayout_3;
    QLabel *label_2;
    QLineEdit *lineEdit_2;
    QHBoxLayout *horizontalLayout_4;
    QLabel *label_3;
    QLineEdit *lineEdit_3;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *HandEye)
    {
        if (HandEye->objectName().isEmpty())
            HandEye->setObjectName(QString::fromUtf8("HandEye"));
        HandEye->resize(1200, 800);
        HandEye->setMinimumSize(QSize(1200, 800));
        HandEye->setMaximumSize(QSize(1200, 800));
        QFont font;
        font.setFamily(QString::fromUtf8("\351\273\221\344\275\223"));
        font.setPointSize(11);
        HandEye->setFont(font);
        centralwidget = new QWidget(HandEye);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        textBrowser_log = new QTextBrowser(centralwidget);
        textBrowser_log->setObjectName(QString::fromUtf8("textBrowser_log"));
        textBrowser_log->setGeometry(QRect(800, 190, 381, 171));
        scan_button = new QPushButton(centralwidget);
        scan_button->setObjectName(QString::fromUtf8("scan_button"));
        scan_button->setGeometry(QRect(940, 0, 121, 61));
        scan_button->setMinimumSize(QSize(101, 41));
        scan_button->setMaximumSize(QSize(55555, 55555));
        scan_button->setFont(font);
        connect_button = new QPushButton(centralwidget);
        connect_button->setObjectName(QString::fromUtf8("connect_button"));
        connect_button->setGeometry(QRect(810, 0, 121, 61));
        connect_button->setMinimumSize(QSize(101, 41));
        connect_button->setMaximumSize(QSize(55555, 55555));
        connect_button->setFont(font);
        label_gray = new QLabel(centralwidget);
        label_gray->setObjectName(QString::fromUtf8("label_gray"));
        label_gray->setGeometry(QRect(0, 0, 781, 751));
        label_gray->setMinimumSize(QSize(0, 0));
        label_gray->setMaximumSize(QSize(5555, 555555));
        label_gray->setStyleSheet(QString::fromUtf8("background-color: rgb(4, 4, 4);"));
        scrollArea = new QScrollArea(centralwidget);
        scrollArea->setObjectName(QString::fromUtf8("scrollArea"));
        scrollArea->setGeometry(QRect(800, 370, 391, 191));
        scrollArea->setWidgetResizable(true);
        scrollAreaWidgetContents = new QWidget();
        scrollAreaWidgetContents->setObjectName(QString::fromUtf8("scrollAreaWidgetContents"));
        scrollAreaWidgetContents->setGeometry(QRect(0, 0, 389, 189));
        tableWidget = new QTableWidget(scrollAreaWidgetContents);
        if (tableWidget->columnCount() < 3)
            tableWidget->setColumnCount(3);
        if (tableWidget->rowCount() < 5)
            tableWidget->setRowCount(5);
        tableWidget->setObjectName(QString::fromUtf8("tableWidget"));
        tableWidget->setGeometry(QRect(10, 0, 391, 192));
        QFont font1;
        font1.setFamily(QString::fromUtf8("Times New Roman"));
        font1.setPointSize(11);
        tableWidget->setFont(font1);
        tableWidget->setRowCount(5);
        tableWidget->setColumnCount(3);
        scrollArea->setWidget(scrollAreaWidgetContents);
        input_button = new QPushButton(centralwidget);
        input_button->setObjectName(QString::fromUtf8("input_button"));
        input_button->setGeometry(QRect(790, 580, 121, 61));
        input_button->setMinimumSize(QSize(101, 41));
        input_button->setMaximumSize(QSize(55555, 55555));
        input_button->setFont(font);
        calculate_button = new QPushButton(centralwidget);
        calculate_button->setObjectName(QString::fromUtf8("calculate_button"));
        calculate_button->setGeometry(QRect(930, 580, 121, 61));
        calculate_button->setMinimumSize(QSize(101, 41));
        calculate_button->setMaximumSize(QSize(55555, 55555));
        calculate_button->setFont(font);
        save_button = new QPushButton(centralwidget);
        save_button->setObjectName(QString::fromUtf8("save_button"));
        save_button->setGeometry(QRect(790, 650, 101, 51));
        save_button->setMinimumSize(QSize(101, 41));
        save_button->setMaximumSize(QSize(55555, 55555));
        save_button->setFont(font);
        update_button = new QPushButton(centralwidget);
        update_button->setObjectName(QString::fromUtf8("update_button"));
        update_button->setGeometry(QRect(790, 700, 101, 51));
        update_button->setMinimumSize(QSize(101, 41));
        update_button->setMaximumSize(QSize(55555, 55555));
        update_button->setFont(font);
        test_button = new QPushButton(centralwidget);
        test_button->setObjectName(QString::fromUtf8("test_button"));
        test_button->setGeometry(QRect(1070, 580, 121, 61));
        test_button->setMinimumSize(QSize(101, 41));
        test_button->setMaximumSize(QSize(55555, 55555));
        test_button->setFont(font);
        drop_button = new QPushButton(centralwidget);
        drop_button->setObjectName(QString::fromUtf8("drop_button"));
        drop_button->setGeometry(QRect(1070, 0, 121, 61));
        drop_button->setMinimumSize(QSize(101, 41));
        drop_button->setMaximumSize(QSize(55555, 55555));
        drop_button->setFont(font);
        reset_button = new QPushButton(centralwidget);
        reset_button->setObjectName(QString::fromUtf8("reset_button"));
        reset_button->setGeometry(QRect(1070, 80, 121, 61));
        reset_button->setMinimumSize(QSize(101, 41));
        reset_button->setMaximumSize(QSize(55555, 55555));
        reset_button->setFont(font);
        layoutWidget = new QWidget(centralwidget);
        layoutWidget->setObjectName(QString::fromUtf8("layoutWidget"));
        layoutWidget->setGeometry(QRect(800, 110, 153, 20));
        horizontalLayout_11 = new QHBoxLayout(layoutWidget);
        horizontalLayout_11->setObjectName(QString::fromUtf8("horizontalLayout_11"));
        horizontalLayout_11->setContentsMargins(0, 0, 0, 0);
        batch_label_title = new QLabel(layoutWidget);
        batch_label_title->setObjectName(QString::fromUtf8("batch_label_title"));
        batch_label_title->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);

        horizontalLayout_11->addWidget(batch_label_title);

        batch_label = new QLabel(layoutWidget);
        batch_label->setObjectName(QString::fromUtf8("batch_label"));
        batch_label->setEnabled(false);
        batch_label->setAlignment(Qt::AlignCenter);

        horizontalLayout_11->addWidget(batch_label);

        layoutWidget1 = new QWidget(centralwidget);
        layoutWidget1->setObjectName(QString::fromUtf8("layoutWidget1"));
        layoutWidget1->setGeometry(QRect(800, 140, 240, 28));
        horizontalLayout = new QHBoxLayout(layoutWidget1);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        horizontalLayout->setContentsMargins(0, 0, 0, 0);
        cut_titile = new QLabel(layoutWidget1);
        cut_titile->setObjectName(QString::fromUtf8("cut_titile"));
        cut_titile->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);

        horizontalLayout->addWidget(cut_titile);

        cut_edit = new QLineEdit(layoutWidget1);
        cut_edit->setObjectName(QString::fromUtf8("cut_edit"));

        horizontalLayout->addWidget(cut_edit);

        layoutWidget2 = new QWidget(centralwidget);
        layoutWidget2->setObjectName(QString::fromUtf8("layoutWidget2"));
        layoutWidget2->setGeometry(QRect(900, 650, 242, 100));
        gridLayout = new QGridLayout(layoutWidget2);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        gridLayout->setContentsMargins(0, 0, 0, 0);
        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        label = new QLabel(layoutWidget2);
        label->setObjectName(QString::fromUtf8("label"));

        horizontalLayout_2->addWidget(label);

        lineEdit = new QLineEdit(layoutWidget2);
        lineEdit->setObjectName(QString::fromUtf8("lineEdit"));
        lineEdit->setEnabled(true);

        horizontalLayout_2->addWidget(lineEdit);


        gridLayout->addLayout(horizontalLayout_2, 0, 0, 1, 1);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        label_2 = new QLabel(layoutWidget2);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        horizontalLayout_3->addWidget(label_2);

        lineEdit_2 = new QLineEdit(layoutWidget2);
        lineEdit_2->setObjectName(QString::fromUtf8("lineEdit_2"));

        horizontalLayout_3->addWidget(lineEdit_2);


        gridLayout->addLayout(horizontalLayout_3, 1, 0, 1, 1);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        label_3 = new QLabel(layoutWidget2);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        horizontalLayout_4->addWidget(label_3);

        lineEdit_3 = new QLineEdit(layoutWidget2);
        lineEdit_3->setObjectName(QString::fromUtf8("lineEdit_3"));

        horizontalLayout_4->addWidget(lineEdit_3);


        gridLayout->addLayout(horizontalLayout_4, 2, 0, 1, 1);

        HandEye->setCentralWidget(centralwidget);
        menubar = new QMenuBar(HandEye);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 1200, 16));
        HandEye->setMenuBar(menubar);
        statusbar = new QStatusBar(HandEye);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        HandEye->setStatusBar(statusbar);

        retranslateUi(HandEye);

        QMetaObject::connectSlotsByName(HandEye);
    } // setupUi

    void retranslateUi(QMainWindow *HandEye)
    {
        HandEye->setWindowTitle(QCoreApplication::translate("HandEye", "HandEye", nullptr));
        scan_button->setText(QCoreApplication::translate("HandEye", "\346\211\253\345\233\276", nullptr));
        connect_button->setText(QCoreApplication::translate("HandEye", "\350\277\236\346\216\245", nullptr));
        label_gray->setText(QString());
        input_button->setText(QCoreApplication::translate("HandEye", "\345\256\214\346\210\220\346\234\254\346\254\241\350\276\223\345\205\245", nullptr));
        calculate_button->setText(QCoreApplication::translate("HandEye", "\345\256\214\346\210\220\351\207\207\351\233\206", nullptr));
        save_button->setText(QCoreApplication::translate("HandEye", "\344\277\235\345\255\230\345\217\202\346\225\260", nullptr));
        update_button->setText(QCoreApplication::translate("HandEye", "\346\233\264\346\226\260\345\217\202\346\225\260", nullptr));
        test_button->setText(QCoreApplication::translate("HandEye", "\347\262\276\345\272\246\346\265\213\350\257\225", nullptr));
        drop_button->setText(QCoreApplication::translate("HandEye", "\346\212\233\345\274\203\346\240\267\346\234\254", nullptr));
        reset_button->setText(QCoreApplication::translate("HandEye", "\351\207\215\347\275\256", nullptr));
        batch_label_title->setText(QCoreApplication::translate("HandEye", "\345\275\223\345\211\215\346\211\271\345\244\204\347\220\206\350\241\214\346\225\260:", nullptr));
        batch_label->setText(QCoreApplication::translate("HandEye", "0", nullptr));
        cut_titile->setText(QCoreApplication::translate("HandEye", "\345\211\252\345\210\207\345\260\272\345\257\270(mm)", nullptr));
        label->setText(QCoreApplication::translate("HandEye", "\346\273\244\346\263\242\351\230\210\345\200\274", nullptr));
        label_2->setText(QCoreApplication::translate("HandEye", "\345\231\252\345\243\260\351\230\210\345\200\274", nullptr));
        label_3->setText(QCoreApplication::translate("HandEye", "\347\273\223\346\236\204\345\260\272\345\257\270", nullptr));
    } // retranslateUi

};

namespace Ui {
    class HandEye: public Ui_HandEye {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_HANDEYE_H
