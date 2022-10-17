#include "treemodel.h"
#include "treeitem.h"

#include <QStringList>

#include <cassert>

TreeModel::TreeModel(const QString &data, QObject *parent)
    : QAbstractItemModel(parent)
{
    rootItem = new TreeItem({tr("Type"), tr("Id")});
    m_itemsMap[-1] = rootItem;
}

TreeModel::~TreeModel()
{
    delete rootItem;
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

    return item->data(index.column());
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
        return rootItem->data(section);

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

    TreeItem *childItem = parentItem->child(row);
    if (childItem)
        return createIndex(row, column, childItem);
    return QModelIndex();
}

QModelIndex TreeModel::parent(const QModelIndex &index) const
{
    if (!index.isValid())
        return QModelIndex();

    TreeItem *childItem = static_cast<TreeItem*>(index.internalPointer());
    TreeItem *parentItem = childItem->parentItem();

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

    return parentItem->childCount();
}

void TreeModel::addItem(int parentIndex, int id, std::string type)
{
    TreeItem * parent = m_itemsMap[parentIndex];
    QVector<QVariant> columnData;
    columnData << type.c_str() << QString::number(id);
    auto item = new TreeItem(columnData, parent, id);
    parent->appendChild(item);
    m_itemsMap[id] = item;
    emit layoutChanged();
}

void TreeModel::delItem(int id)
{
    TreeItem * parent = m_itemsMap[id]->parentItem();
    parent->removeChild(m_itemsMap[id]);
    m_itemsMap.erase(m_itemsMap.find(id));
    emit layoutChanged();
}

QModelIndex TreeModel::getIndexById(int id)
{
    auto treeItem = m_itemsMap[id];
    return createIndex(treeItem->row(), 0, treeItem);
}