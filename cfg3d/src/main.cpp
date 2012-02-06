#include <QtGui/QApplication>
//#include <QWidget>
#include "labler.h"
int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    labler labler;

    labler.show();
    while(true)
    {
        QCoreApplication::sendPostedEvents();
        QCoreApplication::processEvents();
    }
}