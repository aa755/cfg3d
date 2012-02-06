/*
 * File:   labler.cpp
 * Author: aa755
 *
 * Created on February 5, 2012, 5:36 PM
 */


#include <qt4/QtGui/qlabel.h>

#include "labler.h"

labler::labler()
{
    widget.setupUi(this);
    connect(widget.buttonBox,SIGNAL(accepted()),this,SLOT(setString1()));
}

void labler::setString1()
{
    widget.label->setText("hello");
}

labler::~labler()
{
}
