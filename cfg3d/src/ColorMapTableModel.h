/* 
 * File:   ColorMapTableModel.h
 * Author: aa755
 *
 * Created on February 10, 2012, 11:19 PM
 */

#ifndef COLORMAPTABLEMODEL_H
#define	COLORMAPTABLEMODEL_H
#include<string>
#include<vector>
using namespace std;
#include <QAbstractTableModel>
#include <QColor>

//class ColorMapTableModel : public QAbstractTableModel{
class ColorMapTableModel : public QAbstractTableModel
 {
     Q_OBJECT
 public:
     ColorMapTableModel(QObject *parent);
     int rowCount(const QModelIndex &parent = QModelIndex()) const ;
     int columnCount(const QModelIndex &parent = QModelIndex()) const;
     QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const;
     void addItem(string node, QColor color);
     void clearAll();
 protected:
     vector<pair<QString,QColor> > colorMap;
 };

#endif	/* COLORMAPTABLEMODEL_H */

