#include "treemodel.h"
#include "Selectable.h"
#include "globals.h"

#include <QStringList>

#include <cassert>

TreeModel::TreeModel(TreeItem * root, QObject *parent)
    : QAbstractItemModel(parent), rootItem(root)
{
}

TreeModel::~TreeModel()
{
}

int TreeModel::columnCount(const QModelIndex &parent) const
{
    if (parent.isValid())
        return static_cast<TreeItem*>(parent.internalPointer())->columnCount();
    return rootItem->columnCount();
}

QVariant TreeModel::data(const QModelIndex &index, int role) const
{
    if (!index.isValid())
        return QVariant();

    if (role != Qt::DisplayRole)
        return QVariant();

    TreeItem *item = static_cast<TreeItem*>(index.internalPointer());

    return QString(item->data(index.column()).c_str());
}

Qt::ItemFlags TreeModel::flags(const QModelIndex &index) const
{
    if (!index.isValid())
        return Qt::NoItemFlags;

    return QAbstractItemModel::flags(index);
}

QVariant TreeModel::headerData(int section, Qt::Orientation orientation,
                               int role) const
{
    if (orientation == Qt::Horizontal && role == Qt::DisplayRole)
        return QString(rootItem->data(section).c_str());

    return QVariant();
}

QModelIndex TreeModel::index(int row, int column, const QModelIndex &parent) const
{
    if (!hasIndex(row, column, parent))
        return QModelIndex();

    TreeItem *parentItem;

    if (!parent.isValid())
        parentItem = rootItem;
    else
        parentItem = static_cast<TreeItem*>(parent.internalPointer());

    TreeItem *childItem = parentItem->children()[row];
    if (childItem)
        return createIndex(row, column, childItem);
    return QModelIndex();
}

QModelIndex TreeModel::parent(const QModelIndex &index) const
{
    if (!index.isValid())
        return QModelIndex();

    // KB: Qt provides index with invalid internal pointer (dangling pointer)!
    // no better solution was found than check against a magic number (quasi check sum)
    TreeItem *self = static_cast<TreeItem*>(index.internalPointer());
    if (self->m_magicNumber != MAGICNUMBER)
        return QModelIndex();
    TreeItem *parentItem = self->getParent();
    if (parentItem->m_magicNumber != MAGICNUMBER)
        return QModelIndex();

    if (!parentItem)
        return QModelIndex();

    if (parentItem == rootItem)
        return QModelIndex();

    return createIndex(parentItem->row(), 0, parentItem);
}

int TreeModel::rowCount(const QModelIndex &parent) const
{
    TreeItem *parentItem;
    if (parent.column() > 0)
        return 0;

    if (!parent.isValid())
        parentItem = rootItem;
    else
        parentItem = static_cast<TreeItem*>(parent.internalPointer());

    return parentItem->children().size();
}

QModelIndex TreeModel::getIndexById(int id)
{
    Selectable * treeItem = rootItem->findSelectable(id);
    return createIndex(treeItem->row(), 0, treeItem);
}