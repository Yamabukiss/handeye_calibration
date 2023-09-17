#ifndef UTILS_H
#define UTILS_H

#include <QObject>
#include <QMessageBox>
#include <QTableWidget>
#include <QDoubleValidator>
#include <QDebug>
#include "delegate.h"

class Utils : public QObject
{
    Q_OBJECT
public:
    explicit Utils(QObject *parent = nullptr);
    void showWarnMsg(QString _text);

    void insertNewRow(QTableWidget* _table_ptr);

    void setValidator(QTableWidget* _table_ptr);

    bool tableCheck(QTableWidget* _table_ptr);

    void appearTable(QTableWidget* _table_ptr, int _max_row);

    void hiddenTable(QTableWidget* _table_ptr, int _start_row, int _max_row);

    void judgeEnterTable(std::vector<std::unique_ptr<QTableWidgetItem>> &_vp_item, QTableWidget* _table_ptr);

    int judgeAppearNum(QTableWidget* _table_ptr);

    void clearTable(std::vector<std::unique_ptr<QTableWidgetItem>> &_vp_item, QTableWidget* _table_ptr);

    void eraseAllRow(std::vector<std::unique_ptr<QTableWidgetItem>> &_vp_item);

    void tableItemInit(std::vector<std::unique_ptr<QTableWidgetItem>> &_vp_item, QTableWidget* _table_ptr, int _init_num);

private:
    Delegate* delegate_ptr_;
};

#endif // UTILS_H
