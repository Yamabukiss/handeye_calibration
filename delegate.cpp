#include "delegate.h"

Delegate::Delegate() = default;

QWidget* Delegate::createEditor(QWidget* parent, const QStyleOptionViewItem& option, const QModelIndex& index) const
{
    QLineEdit* editor = new QLineEdit(parent);
    QValidator* validator = new QDoubleValidator(editor);
    editor->setValidator(validator);
    return editor;
}

void Delegate::setEditorData(QWidget* editor, const QModelIndex& index) const
{
    QLineEdit* line_edit = qobject_cast<QLineEdit*>(editor);
    QString value = index.model()->data(index, Qt::EditRole).toString();
    line_edit->setText(value);
}

void Delegate::setModelData(QWidget* editor, QAbstractItemModel* model, const QModelIndex& index) const
{
    QLineEdit* line_edit = qobject_cast<QLineEdit*>(editor);
    QString value = line_edit->text();
    model->setData(index, value, Qt::EditRole);
}

Delegate::~Delegate() = default;
