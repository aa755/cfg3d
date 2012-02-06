/* 
 * File:   labler.h
 * Author: aa755
 *
 * Created on February 5, 2012, 5:36 PM
 */

#ifndef _LABLER_H
#define	_LABLER_H

#include "ui_labler.h"

class labler : public QDialog {
    Q_OBJECT
public:
    labler();
    virtual ~labler();
public slots:
    void setString1();
private:
    Ui::labler widget;
};

#endif	/* _LABLER_H */
