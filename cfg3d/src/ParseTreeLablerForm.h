/* 
 * File:   ParseTreeLablerForm.h
 * Author: aa755
 *
 * Created on February 5, 2012, 9:20 PM
 */

#ifndef _PARSETREELABLERFORM_H
#define	_PARSETREELABLERFORM_H

#include "ui_ParseTreeLablerForm.h"

class ParseTreeLablerForm : public QDialog {
    Q_OBJECT
public:
    ParseTreeLablerForm();
    virtual ~ParseTreeLablerForm();
    Ui::ParseTreeLablerForm widget;
};

#endif	/* _PARSETREELABLERFORM_H */
