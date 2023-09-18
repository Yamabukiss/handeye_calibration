#pragma execution_character_set("utf-8")
#include "utils.h"

Utils::Utils(QObject *parent)
    : QObject{parent}, delegate_ptr_(new Delegate)
{

}

bool Utils::tableCheck(QTableWidget* _table_ptr)
{
    bool complete = true;

    int rows = judgeAppearNum(_table_ptr);
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

void Utils::showWarnMsg(QString _text)
{
    QMessageBox messageBox;

    messageBox.setWindowTitle("提示");

    messageBox.setText(_text);

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
    _table_ptr->setItemDelegate(delegate_ptr_);
}

void Utils::appearTable(QTableWidget* _table_ptr, int _max_row)
{
    for (int row = 1; row <= _max_row; row++)
        _table_ptr->setRowHidden(row, false);
}

void Utils::hiddenTable(QTableWidget* _table_ptr, int _start_row, int _max_row)
{
    for (int row = _start_row; row < _max_row; row++)
        _table_ptr->setRowHidden(row, true);
}

int Utils::judgeAppearNum(QTableWidget* _table_ptr)
{
    int current_rows = _table_ptr->rowCount();
    int appear_rows = 1;

    for (int row = 1; row < current_rows; row++)
        if (!_table_ptr->item(row, 0)->text().isEmpty())
            appear_rows++;
    return appear_rows;
}

void Utils::judgeEnterTable(std::vector<std::unique_ptr<QTableWidgetItem>> &_vp_item, QTableWidget* _table_ptr)
{
    int current_rows = _table_ptr->rowCount();
    int base_rows = _vp_item.size() / 3;
    int appear_rows = 0;

    appear_rows = judgeAppearNum(_table_ptr);

    if (appear_rows == base_rows)
    {
        insertNewRow(_table_ptr);

        QTableWidgetItem* item_x = new QTableWidgetItem();
        _table_ptr->setItem(current_rows, 0, item_x);

        QTableWidgetItem* item_y = new QTableWidgetItem();
        _table_ptr->setItem(current_rows, 1, item_y);

        QTableWidgetItem* item_z = new QTableWidgetItem();
        _table_ptr->setItem(current_rows, 2, item_z);

        _vp_item.emplace_back(item_x);
        _vp_item.emplace_back(item_y);
        _vp_item.emplace_back(item_z);
    }
    else
    {
        appearTable(_table_ptr, appear_rows);
        _table_ptr->setCurrentCell(appear_rows, 0);
    }
}

void Utils::clearTable(std::vector<std::unique_ptr<QTableWidgetItem>> &_vp_item, QTableWidget* _table_ptr)
{
    eraseAllRow(_vp_item);
    int rows = static_cast<int>(_vp_item.size()) / 3;
    for (int row = 1; row < rows; row++)
        _table_ptr->setRowHidden(row, true);
}

void Utils::eraseAllRow(std::vector<std::unique_ptr<QTableWidgetItem>> &_vp_item)
{
    for (auto &item : _vp_item)
        item.get()->setText("");
}

void Utils::tableItemInit(std::vector<std::unique_ptr<QTableWidgetItem>> &_vp_item, QTableWidget* _table_ptr, int _init_num)
{
    for (int row = 0; row < _init_num; row++)
    {
        QTableWidgetItem* item_x = new QTableWidgetItem();

        _table_ptr->setItem(row, 0, item_x);

        QTableWidgetItem* item_y = new QTableWidgetItem();

        _table_ptr->setItem(row, 1, item_y);

        QTableWidgetItem* item_z = new QTableWidgetItem();

        _table_ptr->setItem(row, 2, item_z);

        _vp_item.emplace_back(item_x);
        _vp_item.emplace_back(item_y);
        _vp_item.emplace_back(item_z);
    }

    clearTable(_vp_item, _table_ptr);
}

bool Utils::checkLineEdit(const QString &value, int &param)
{
    if (value == "")
        return false;
    else
    {
        param = value.toInt();
        return true;
    }
}

Utils::~Utils()
{
    delete delegate_ptr_;
}


