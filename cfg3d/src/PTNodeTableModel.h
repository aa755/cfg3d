/* 
 * File:   PTNodeTableModel.h
 * Author: aa755
 *
 * Created on February 6, 2012, 4:25 PM
 */

#ifndef PTNODETABLEMODEL_H
#define	PTNODETABLEMODEL_H
#include<string>
#include<vector>
using namespace std;

#include <QAbstractTableModel>

 class PTNodeTableModel : public QAbstractTableModel
 {
     Q_OBJECT
 public:
     PTNodeTableModel(QObject *parent);
     int rowCount(const QModelIndex &parent = QModelIndex()) const ;
     int columnCount(const QModelIndex &parent = QModelIndex()) const;
     QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const;
     
     void addItem(string node /* = 0 */);
 protected:
     vector<string> nodesToMerge;
 };

#endif	/* PTNODETABLEMODEL_H */

