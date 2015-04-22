/**
 * @file /src/main.cpp
 *
 * @brief Qt based gui.
 *
 * @date November 2010
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QApplication>
#include "../include/cob_light_sim/main_window.hpp"

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

  int result = 0;
    /*********************
    ** Qt
    **********************/
    QApplication app(argc, argv);
    cob_light_sim::MainWindow w(argc,argv);
    w.show();
    if(w.connect())
    {
      app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
      int result = app.exec();
    }

	return result;
}
