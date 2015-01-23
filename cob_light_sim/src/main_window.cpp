/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/cob_light_sim/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cob_light_sim {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

	  setWindowIcon(QIcon(":/images/icon.png"));

	  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

}

MainWindow::~MainWindow() {}

bool MainWindow::connect()
{
  if ( !qnode.init() ) {
        showNoMasterMessage();
        return false;
  }
  return true;
}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}


void MainWindow::closeEvent(QCloseEvent *event)
{
	QMainWindow::closeEvent(event);
}

}  // namespace cob_light_sim

