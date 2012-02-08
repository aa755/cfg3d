/* 
 * File:   PTNodeTableModel.cpp
 * Author: aa755
 * 
 * Created on February 6, 2012, 4:25 PM
 */

#include "PTNodeTableModel.h"
#include "ParseTreeLablerForm.h"
 #include <QMessageBox>
 
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
void PTNodeTableModel::clearAll()
{
    nodesToMerge.clear();
    reset();
}

void PTNodeTableModel::combineAll(ParseTreeLablerForm* form)
{
    string newNodeType=string(form->widget.comboBox->currentText().toUtf8().constData());
    string newNodeName=newNodeType+"__"+lexical_cast<string>(++form->typeMaxId[newNodeType]);
    QStandardItem *newItem = new QStandardItem(newNodeName.data()); 
    for(vector<string>::iterator it=nodesToMerge.begin();it!=nodesToMerge.end();it++)
    {
        QStandardItem * node=form->nameToTreeNode[*it];
        assert(node!=NULL);
        if(node->parent()!=0)
        {
              QMessageBox::warning(form, tr("Parse Tree Labaler"),
                                tr("One of the nodes already had a parent and hence cannot be combined. "
                                   "Please clear the nodes and add nodes which have no parent"),
                                QMessageBox::Ok,QMessageBox::Ok);            
              return;
        }
        form->rootNode->takeRow(node->row()); // remove this node from it's current parent ... which should be root node
        newItem->appendRow(node);
        
    }
    form->nameToTreeNode[newNodeName]=newItem;
    form->rootNode->appendRow(newItem);
    form->widget.treeView->expandAll();
    clearAll();
//    form->standardModel->submit();
}
