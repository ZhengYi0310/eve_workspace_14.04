/**
 * @file /include/barrett_controller_switcher/main_window.hpp
 *
 * @brief Qt based gui for barrett_controller_switcher.
 *
 * @date November 2010
 **/
#ifndef barrett_controller_switcher_MAIN_WINDOW_H
#define barrett_controller_switcher_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace barrett_controller_switcher {

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

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
	void on_button_connect_clicked(bool check );
    void on_button_SwitchController_clicked(bool check);
    void on_button_LoadControllerList_clicked(bool check);
    void on_button_SendJointPos_clicked(bool check);
    void on_button_GoHome_clicked(bool check);
    void on_button_SetGains_clicked(bool check);
	void on_checkbox_use_environment_stateChanged(int state);

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically

private:
    void switch_tab(int index);
    void fill_controllers_list();
    void fill_joint_pos_fields();
    void fill_joint_gains_fields();
    void field_error_msg_box(std::string field_name);
    void service_error_msg_box(std::string controller_name);
    void set_send_jointpos_label(QLabel* label, bool state);
	Ui::MainWindowDesign ui;
	QNode qnode;
};

}  // namespace barrett_controller_switcher

#endif // barrett_controller_switcher_MAIN_WINDOW_H
