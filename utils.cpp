#pragma execution_character_set("utf-8")
#include "utils.h"

Utils::Utils(QObject *parent)
    : QObject{parent}
{
    delegate_ptr_vec_.reserve(3);
}

bool Utils::tableCheck(QTableWidget* _table_ptr)
{
    bool complete = true;

    int rows = _table_ptr->rowCount();
    int cols = _table_ptr->columnCount();

    for (int row = 0; row < rows; row++)
    {
        for (int col = 0; col < cols; col++)
        {
            QTableWidgetItem* item = _table_ptr->item(row, col);
            if (!item || item->text().isEmpty())
            {
                complete = false;
                break;
            }
        }
    }

    return complete;
}

void Utils::showWarnMsg(QString text)
{
    QMessageBox messageBox;

    messageBox.setWindowTitle("提示");

    messageBox.setText(text);

    messageBox.setIcon(QMessageBox::Warning);

    messageBox.setStandardButtons(QMessageBox::Ok);

    messageBox.exec();
}

void Utils::insertNewRow(QTableWidget* _table_ptr)
{
    int last_row = _table_ptr->rowCount() - 1;
    bool create_new_row = true;
    for (int col = 0; col < _table_ptr->columnCount(); ++col)
    {
        QTableWidgetItem *item = _table_ptr->item(last_row, col);
        if (!item || item->text().isEmpty())
        {
            create_new_row = false;
            break;
        }
    }
    if (create_new_row)
    {
        int new_row = last_row + 1;
        _table_ptr->insertRow(new_row);

        _table_ptr->setCurrentCell(new_row, 0);
    }
}

void Utils::setValidator(QTableWidget* _table_ptr)
{
    std::shared_ptr<Delegate> delegate_ptr(new Delegate);
    delegate_ptr_vec_.emplace_back(delegate_ptr);
    _table_ptr->setItemDelegate(delegate_ptr.get());
}
