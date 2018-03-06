/**
 * @file /include/barrett_controller_GUI/main_window.hpp
 *
 * @brief Qt based gui for barrett_controller_GUI.
 *
 * @date November 2010
 **/
#ifndef barrett_controller_GUI_MAIN_WINDOW_H
#define barrett_controller_GUI_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace barrett_controller_GUI {

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
    void on_button_SendCartPos_clicked(bool check);
    void on_button_GoHome_clicked(bool check);
    void on_button_SetGains_clicked(bool check);
    void on_button_SetGainsCart_clicked(bool check);
        void on_checkbox_use_environment_stateChanged(int state);

    /******************************************
    ** Manual connections
    *******************************************/
        void updateLoggingView(); // no idea why this can't connect automatically
        void update_joints_state();
        void update_joints_error();
        void update_cart_error();
        void update_cart_pos();
        void update_progress_data();

private:
        void switch_tab(int index);
        void fill_controllers_list();
        void fill_joint_pos_fields();
        void fill_joint_gains_fields();
        void fill_cart_pos_fields();
        void fill_cart_gains_fields();
        void field_error_msg_box(std::string field_name);
        void service_error_msg_box(std::string controller_name);
        void set_send_jointpos_label(QLabel* label, bool accepted, double elapsed, double duration);
        void set_send_cartpos_label(QLabel* label, bool accepted, double elapsed, double duration);
        void set_progress_bar(QProgressBar* bar, double elapsed_time, double total_time);
            Ui::MainWindowDesign ui;
            QNode qnode;
};

}  // namespace barrett_controller_GUI

#endif // barrett_controller_GUI_MAIN_WINDOW_H
