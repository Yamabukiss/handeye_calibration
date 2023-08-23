#ifndef UTILS_H
#define UTILS_H

#include <QObject>
#include <QMessageBox>
#include <QTableWidget>
#include <QDoubleValidator>
#include "delegate.h"

class Utils : public QObject
{
    Q_OBJECT
public:
    explicit Utils(QObject *parent = nullptr);
    void showWarnMsg(QString text);

    void insertNewRow(QTableWidget* _table_ptr);

    void setValidator(QTableWidget* _table_ptr);

    bool tableCheck(QTableWidget* _table_ptr);

private:
    std::vector<std::shared_ptr<Delegate>> delegate_ptr_vec_;
};

#endif // UTILS_H
