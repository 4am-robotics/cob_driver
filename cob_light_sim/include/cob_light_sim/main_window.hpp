/**
 * @file /include/cob_light_sim/main_window.hpp
 *
 * @brief Qt based gui for cob_light_sim.
 *
 * @date November 2010
 **/
#ifndef cob_light_sim_MAIN_WINDOW_H
#define cob_light_sim_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace cob_light_sim {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	bool connect();

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();


private:
	Ui::MainWindowDesign ui;
	QNode qnode;
};

}  // namespace cob_light_sim

#endif // cob_light_sim_MAIN_WINDOW_H
