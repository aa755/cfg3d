/* 
 * File:   PTNodeTableModel.cpp
 * Author: aa755
 * 
 * Created on February 6, 2012, 4:25 PM
 */

#include "PTNodeTableModel.h"

 
 PTNodeTableModel::PTNodeTableModel(QObject *parent)
     :QAbstractTableModel(parent)
 {
 }

 int PTNodeTableModel::rowCount(const QModelIndex & /*parent*/) const
 {
    return nodesToMerge.size();
 }

 int PTNodeTableModel::columnCount(const QModelIndex & /*parent*/) const
 {
     return 1;
 }

 QVariant PTNodeTableModel::data(const QModelIndex &index, int role) const
 {
     if (role == Qt::DisplayRole)
     {
        return QString(nodesToMerge.at(index.row()).data());
     }
     return QVariant();
 }

void PTNodeTableModel::addItem(string node /* = 0 */)
{
   // beginInsertRows(index(0,0),0,nodesToMerge.size());
    nodesToMerge.push_back(node);
   // endInsertRows();
    reset();
   // submit();
//    select();
    //  emit dataChanged(createIndex(0,0),createIndex(nodesToMerge.size(),0));
      
//    emit modelReset();
}
