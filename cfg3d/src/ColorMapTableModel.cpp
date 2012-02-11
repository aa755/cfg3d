/* 
 * File:   ColorMapTableModel.cpp
 * Author: aa755
 * 
 * Created on February 10, 2012, 11:19 PM
 */

#include "ColorMapTableModel.h"
#include "qt4/QtCore/qbytearray.h"
#include "qt4/QtGui/qbrush.h"


 ColorMapTableModel::ColorMapTableModel(QObject *parent)
     :QAbstractTableModel(parent)
 {
 }

 int ColorMapTableModel::rowCount(const QModelIndex & /*parent*/) const
 {
    return colorMap.size();
 }

 int ColorMapTableModel::columnCount(const QModelIndex & /*parent*/) const
 {
     return 2;
 }

 QVariant ColorMapTableModel::data(const QModelIndex &index, int role) const
 {
     if (role == Qt::DisplayRole)
     {
         if(index.column()==0)
         {
                return QString(colorMap.at(index.row()).first.data());
         }
         else if(index.column()==1)
         {
             return QString("");
         }
     }
     else if(role==Qt::BackgroundRole)
     {
         if(index.column()==1)
         {
             return QBrush(colorMap.at(index.row()).second);
         }
         
     }
     return QVariant();
 }

 void ColorMapTableModel::addItem(string node, QColor color)
 {
     colorMap.push_back(pair<QString,QColor>(QString(node.data()),color));
     reset();
 }
 
 void ColorMapTableModel::clearAll()
 {
     colorMap.clear();
     reset();
 }