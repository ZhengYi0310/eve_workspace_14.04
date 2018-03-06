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
#include "../include/barrett_controller_GUI/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace barrett_controller_GUI {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();
	setWindowIcon(QIcon(":/images/icon.png"));
	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

	/*********************
	** Logging
	**********************/
	ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

    /*********************
    ** Auto Start
    **********************/
    if ( ui.checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked(true);
    }

    qnode.set_robot_namespace(argv[1]);
    qnode.init();


    // move the window to the center
    setGeometry(QStyle::alignedRect(Qt::LeftToRight,
                    Qt::AlignCenter,
                    size(),
                    qApp->desktop()->availableGeometry()));

    // force size of the window
    //setFixedSize(706, 681);

    // fill controller lists from robot_namespace_/controller_manager/ListControllers
    fill_controllers_list();

    // fill controllers gains fields with default gains
    fill_joint_gains_fields();
    //fill_hybrid_gains_fields();

    // connect joints state view update to joints state topic callback
    QObject::connect(&qnode, SIGNAL(jointsStateArrived()), this, SLOT(update_joints_state()));

    // connect joints error view update to joints error topic callback
    QObject::connect(&qnode, SIGNAL(jointsErrorArrived()), this, SLOT(update_joints_error()));

    // connect cartesian error view update to joints error topic callback
    QObject::connect(&qnode, SIGNAL(CartPosArrived()), this, SLOT(update_cart_pos()));

    QObject::connect(&qnode, SIGNAL(CartErrorArrived()), this, SLOT(update_cart_error()));

    // connect progress bars and send state labels update to progress data service call
    // service call is done in the run() method of qnode
    QObject::connect(&qnode, SIGNAL(progressDataArrived()), this, SLOT(update_progress_data()));
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_connect_clicked(bool check ) {
	if ( ui.checkbox_use_environment->isChecked() ) {
		if ( !qnode.init() ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
		}
	} else {
		if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
				   ui.line_edit_host->text().toStdString()) ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
			ui.line_edit_master->setReadOnly(true);
			ui.line_edit_host->setReadOnly(true);
			ui.line_edit_topic->setReadOnly(true);
		}
	}
}


void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
	bool enabled;
	if ( state == 0 ) {
		enabled = true;
	} else {
		enabled = false;
	}
	ui.line_edit_master->setEnabled(enabled);
	ui.line_edit_host->setEnabled(enabled);
	//ui.line_edit_topic->setEnabled(enabled);
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

void MainWindow::on_button_SetGains_clicked(bool check)
{
    bool outcome;
    bool state;
    std::vector<double> Kp_(7), Kv_(7);
    wam_dmp_controller::SetJointGainsMsg cmd, res;
    cmd.Kp_gains.resize(7);
    cmd.Kv_gains.resize(7);
    Kp_[0] = ui.Joint1KpGainText->text().toDouble(&outcome);
    if (!outcome || (Kp_[0] <= 0))
    {
        field_error_msg_box("Kp_J1");
        return;
    }
    Kv_[0] = ui.Joint1KvGainText->text().toDouble(&outcome);
    if (!outcome || (Kv_[0] <= 0))
    {
        field_error_msg_box("Kv_J1");
        return;
    }

    Kp_[1] = ui.Joint2KpGainText->text().toDouble(&outcome);
    if (!outcome || (Kp_[1] <= 0))
    {
        field_error_msg_box("Kp_J2");
        return;
    }
    Kv_[1] = ui.Joint2KvGainText->text().toDouble(&outcome);
    if (!outcome || (Kv_[1] <= 0))
    {
        field_error_msg_box("Kv_J2");
        return;
    }

    Kp_[2] = ui.Joint3KpGainText->text().toDouble(&outcome);
    if (!outcome || (Kp_[2] <= 0))
    {
        field_error_msg_box("Kp_J3");
        return;
    }
    Kv_[2] = ui.Joint3KvGainText->text().toDouble(&outcome);
    if (!outcome || (Kv_[2] <= 0))
    {
        field_error_msg_box("Kv_J3");
        return;
    }

    Kp_[3] = ui.Joint4KpGainText->text().toDouble(&outcome);
    if (!outcome || (Kp_[3] <= 0))
    {
        field_error_msg_box("Kp_J4");
        return;
    }
    Kv_[3] = ui.Joint4KvGainText->text().toDouble(&outcome);
    if (!outcome || (Kv_[3] <= 0))
    {
        field_error_msg_box("Kv_J4");
        return;
    }

    Kp_[4] = ui.Joint5KpGainText->text().toDouble(&outcome);
    if (!outcome || (Kp_[4] <= 0))
    {
        field_error_msg_box("Kp_J5");
        return;
    }
    Kv_[4] = ui.Joint5KvGainText->text().toDouble(&outcome);
    if (!outcome || (Kv_[4] <= 0))
    {
        field_error_msg_box("Kv_J5");
        return;
    }

    Kp_[5] = ui.Joint6KpGainText->text().toDouble(&outcome);
    if (!outcome || (Kp_[5] <= 0))
    {
        field_error_msg_box("Kp_J6");
        return;
    }
    Kv_[5] = ui.Joint6KvGainText->text().toDouble(&outcome);
    if (!outcome || (Kv_[5] <= 0))
    {
        field_error_msg_box("Kv_J6");
        return;
    }

    Kp_[6] = ui.Joint7KpGainText->text().toDouble(&outcome);
    if (!outcome || (Kp_[6] <= 0))
    {
        field_error_msg_box("Kp_J7");
        return;
    }
    Kv_[6] = ui.Joint7KvGainText->text().toDouble(&outcome);
    if (!outcome || (Kv_[6] <= 0))
    {
        field_error_msg_box("Kv_J7");
        return;
    }

    for (int i = 0; i < 7; i++)
    {
        cmd.Kp_gains[i] = Kp_[i];
        cmd.Kv_gains[i] = Kv_[i];
    }
    outcome = qnode.set_command<wam_dmp_controller::SetJointGains, wam_dmp_controller::SetJointGainsMsg>(cmd, res);
    if(!outcome)
      service_error_msg_box("joint_space_controller(SetJointGains)");
}

void MainWindow::on_button_SetGainsCart_clicked(bool check)
{
    bool outcome;
    wam_dmp_controller::ImpedanceControllerGainsMsg cmd, res;

    cmd.Kp_gains.resize(6);
    cmd.Kv_gains.resize(6);
    cmd.null_Kp_gains.resize(7);
    cmd.null_Kv_gains.resize(7);

    cmd.Kp_gains[0] = ui.CartPosXGainText_Kp->text().toDouble(&outcome);
    if (!outcome || (cmd.Kp_gains[0] <= 0))
    {
        field_error_msg_box("Kp_X");
        return;
    }
    cmd.Kp_gains[1] = ui.CartPosYGainText_Kp->text().toDouble(&outcome);
    if (!outcome || (cmd.Kp_gains[1] <= 0))
    {
        field_error_msg_box("Kp_Y");
        return;
    }
    cmd.Kp_gains[2] = ui.CartPosZGainText_Kp->text().toDouble(&outcome);
    if (!outcome || (cmd.Kp_gains[2] <= 0))
    {
        field_error_msg_box("Kp_Z");
        return;
    }
    cmd.Kp_gains[3] = ui.CartPosRollGainText_Kp->text().toDouble(&outcome);
    if (!outcome || (cmd.Kp_gains[3] <= 0))
    {
        field_error_msg_box("Kp_Roll");
        return;
    }
    cmd.Kp_gains[4] = ui.CartPosPitchGainText_Kp->text().toDouble(&outcome);
    if (!outcome || (cmd.Kp_gains[4] <= 0))
    {
        field_error_msg_box("Kp_Pitch");
        return;
    }
    cmd.Kp_gains[5] = ui.CartPosYawGainText_Kp->text().toDouble(&outcome);
    if (!outcome || (cmd.Kp_gains[5] <= 0))
    {
        field_error_msg_box("Kp_Yaw");
        return;
    }



    cmd.Kv_gains[0] = ui.CartPosXGainText_Kv->text().toDouble(&outcome);
    if (!outcome || (cmd.Kv_gains[0] <= 0))
    {
        field_error_msg_box("Kv_X");
        return;
    }
    cmd.Kv_gains[1] = ui.CartPosYGainText_Kv->text().toDouble(&outcome);
    if (!outcome || (cmd.Kv_gains[1] <= 0))
    {
        field_error_msg_box("Kv_Y");
        return;
    }
    cmd.Kv_gains[2] = ui.CartPosZGainText_Kv->text().toDouble(&outcome);
    if (!outcome || (cmd.Kv_gains[2] <= 0))
    {
        field_error_msg_box("Kv_Z");
        return;
    }
    cmd.Kv_gains[3] = ui.CartPosRollGainText_Kv->text().toDouble(&outcome);
    if (!outcome || (cmd.Kv_gains[3] <= 0))
    {
        field_error_msg_box("Kv_Roll");
        return;
    }
    cmd.Kv_gains[4] = ui.CartPosPitchGainText_Kv->text().toDouble(&outcome);
    if (!outcome || (cmd.Kv_gains[4] <= 0))
    {
        field_error_msg_box("Kv_Pitch");
        return;
    }
    cmd.Kv_gains[5] = ui.CartPosYawGainText_Kv->text().toDouble(&outcome);
    if (!outcome || (cmd.Kv_gains[5] <= 0))
    {
        field_error_msg_box("Kv_Yaw");
        return;
    }

    cmd.null_Kp_gains[0] = ui.JointGainText_nullKp_1->text().toDouble(&outcome);
    if (!outcome || (cmd.null_Kp_gains[0] <= 0))
    {
        field_error_msg_box("null_Kp_J1");
        return;
    }
    cmd.null_Kp_gains[1] = ui.JointGainText_nullKp_2->text().toDouble(&outcome);
    if (!outcome || (cmd.null_Kp_gains[1] <= 0))
    {
        field_error_msg_box("null_Kp_J2");
        return;
    }
    cmd.null_Kp_gains[2] = ui.JointGainText_nullKp_3->text().toDouble(&outcome);
    if (!outcome || (cmd.null_Kp_gains[2] <= 0))
    {
        field_error_msg_box("null_Kp_J3");
        return;
    }
    cmd.null_Kp_gains[3] = ui.JointGainText_nullKp_4->text().toDouble(&outcome);
    if (!outcome || (cmd.null_Kp_gains[3] <= 0))
    {
        field_error_msg_box("null_Kp_J4");
        return;
    }
    cmd.null_Kp_gains[4] = ui.JointGainText_nullKp_5->text().toDouble(&outcome);
    if (!outcome || (cmd.null_Kp_gains[4] <= 0))
    {
        field_error_msg_box("null_Kp_J5");
        return;
    }
    cmd.null_Kp_gains[5] = ui.JointGainText_nullKp_6->text().toDouble(&outcome);
    if (!outcome || (cmd.null_Kp_gains[5] <= 0))
    {
        field_error_msg_box("null_Kp_J6");
        return;
    }
    cmd.null_Kp_gains[6] = ui.JointGainText_nullKp_7->text().toDouble(&outcome);
    if (!outcome || (cmd.null_Kp_gains[6] <= 0))
    {
        field_error_msg_box("null_Kp_J7");
        return;
    }

    cmd.null_Kv_gains[0] = ui.JointGainText_nullKv_1->text().toDouble(&outcome);
    if (!outcome || (cmd.null_Kv_gains[0] <= 0))
    {
        field_error_msg_box("null_Kv_J1");
        return;
    }
    cmd.null_Kv_gains[1] = ui.JointGainText_nullKv_2->text().toDouble(&outcome);
    if (!outcome || (cmd.null_Kv_gains[1] <= 0))
    {
        field_error_msg_box("null_Kv_J2");
        return;
    }
    cmd.null_Kv_gains[2] = ui.JointGainText_nullKv_3->text().toDouble(&outcome);
    if (!outcome || (cmd.null_Kv_gains[2] <= 0))
    {
        field_error_msg_box("null_Kv_J3");
        return;
    }
    cmd.null_Kv_gains[3] = ui.JointGainText_nullKv_4->text().toDouble(&outcome);
    if (!outcome || (cmd.null_Kv_gains[3] <= 0))
    {
        field_error_msg_box("null_Kv_J4");
        return;
    }
    cmd.null_Kv_gains[4] = ui.JointGainText_nullKv_5->text().toDouble(&outcome);
    if (!outcome || (cmd.null_Kv_gains[4] <= 0))
    {
        field_error_msg_box("null_Kv_J5");
        return;
    }
    cmd.null_Kv_gains[5] = ui.JointGainText_nullKv_6->text().toDouble(&outcome);
    if (!outcome || (cmd.null_Kv_gains[5] <= 0))
    {
        field_error_msg_box("null_Kv_J6");
        return;
    }
    cmd.null_Kv_gains[6] = ui.JointGainText_nullKv_7->text().toDouble(&outcome);
    if (!outcome || (cmd.null_Kv_gains[6] <= 0))
    {
        field_error_msg_box("null_Kv_J7");
        return;
    }

    outcome = qnode.set_command<wam_dmp_controller::ImpedanceControllerGains, wam_dmp_controller::ImpedanceControllerGainsMsg>(cmd, res);
    if(!outcome)
      service_error_msg_box("operational_space_controller(SetJointGains)");


}

void MainWindow::on_button_GoHome_clicked(bool check)
{
    bool outcome;
    wam_dmp_controller::JointPosSplineMsg cmd, res;
    outcome = qnode.set_command<wam_dmp_controller::GoHomeSpline, wam_dmp_controller::JointPosSplineMsg>(cmd, res);
    if(!outcome)
      service_error_msg_box("joint_spaces_pline_controller(GoHomeSpline)");
    else
      set_send_jointpos_label(ui.GoHomeSentLabel, res.accepted, res.elapsed_time, res.p2p_traj_duration);
}

void MainWindow::on_button_SendJointPos_clicked(bool check)
{
    double j1, j2, j3, j4, j5, j6, j7, D_t;
    bool outcome;

    j1 = ui.Joint1PosText->text().toDouble(&outcome);
    if (!outcome || j1 <= -2.6 || j1 >= 2.6)
    {
        field_error_msg_box("Joint1PosText");
        return;
    }

    j2 = ui.Joint2PosText->text().toDouble(&outcome);
    if (!outcome || j2 <= -2.0 || j2 >= 2.0)
    {
        field_error_msg_box("Joint2PosText");
        return;
    }

    j3 = ui.Joint3PosText->text().toDouble(&outcome);
    if (!outcome || j3 <= -2.8 || j3 >= 2.8)
    {
        field_error_msg_box("Joint3PosText");
        return;
    }

    j4 = ui.Joint4PosText->text().toDouble(&outcome);
    if (!outcome || j4 <= -0.9 || j4 >= 3.1)
    {
        field_error_msg_box("Joint4PosText");
        return;
    }

    j5 = ui.Joint5PosText->text().toDouble(&outcome);
    if (!outcome || j5 <= -4.76 || j5 >= 1.24)
    {
        field_error_msg_box("Joint5PosText");
        return;
    }

    j6 = ui.Joint6PosText->text().toDouble(&outcome);
    if (!outcome || j6 <= -1.6 || j6 >= 1.6)
    {
        field_error_msg_box("Joint6PosText");
        return;
    }

    j7 = ui.Joint7PosText->text().toDouble(&outcome);
    if (!outcome || j7 <= -3.0 || j7 >= 3.0)
    {
        field_error_msg_box("Joint7PosText");
        return;
    }

    D_t = ui.JointPosDurationText->text().toDouble(&outcome);
    if (!outcome || D_t <= 5)
    {
        field_error_msg_box("JointPosDurationText");
        return;
    }

    wam_dmp_controller::JointPosSplineMsg cmd, res;
    cmd.joint_pos.resize(7);
    res.joint_pos.resize(7);
    cmd.joint_pos[0] = j1;
    cmd.joint_pos[1] = j2;
    cmd.joint_pos[2] = j3;
    cmd.joint_pos[3] = j4;
    cmd.joint_pos[4] = j5;
    cmd.joint_pos[5] = j6;
    cmd.joint_pos[6] = j7;
    cmd.p2p_traj_duration = D_t;

    bool state;
    outcome = qnode.set_command<wam_dmp_controller::JointPosSpline, wam_dmp_controller::JointPosSplineMsg>(cmd, res);

    if(!outcome)
      service_error_msg_box("joint_space_controller(SetJointPos)");
    else
      set_send_jointpos_label(ui.JointPosSentLabel, res.accepted, res.elapsed_time, res.p2p_traj_duration);
}

void MainWindow::on_button_SendCartPos_clicked(bool check)
{
    double x, y, z, roll, pitch, yaw, duration;
    bool outcome;

    x = ui.CartPosXText->text().toDouble(&outcome);
    if (!outcome )
    {
        field_error_msg_box("CartPosXText");
        return;
    }

    y = ui.CartPosYText->text().toDouble(&outcome);
    if (!outcome )
    {
        field_error_msg_box("CartPosYText");
        return;
    }

    z = ui.CartPosZText->text().toDouble(&outcome);
    if (!outcome )
    {
        field_error_msg_box("CartPosZText");
        return;
    }

    roll = ui.CartPosRollText->text().toDouble(&outcome);
    if (!outcome )
    {
        field_error_msg_box("CartPosRollText");
        return;
    }

    pitch = ui.CartPosPitchText->text().toDouble(&outcome);
    if (!outcome )
    {
        field_error_msg_box("CartPosPitchText");
        return;
    }

    yaw = ui.CartPosYawText->text().toDouble(&outcome);
    if (!outcome )
    {
        field_error_msg_box("CartPosYawText");
        return;
    }

    duration = ui.CartPosDurationText->text().toDouble(&outcome);
    if (!outcome )
    {
        field_error_msg_box("CartPosDurationText");
        return;
    }

    wam_dmp_controller::PoseRPYCmd cmd, response;

    cmd.position.x = x;
    cmd.position.y = y;
    cmd.position.z = z;
    cmd.orientation.roll = roll;
    cmd.orientation.pitch = pitch;
    cmd.orientation.yaw = yaw;
    cmd.p2p_traj_duration = duration;

    outcome = qnode.set_command<wam_dmp_controller::PoseRPYCommand,\
                                wam_dmp_controller::PoseRPYCmd>(cmd, response);
    if(!outcome)
      service_error_msg_box("operational_space_controller(PoseRPYCommand)");
    else
        set_send_cartpos_label(ui.CartPosSentLabel, response.accepted, response.elapsed_time, response.p2p_traj_duration);
}

void MainWindow::on_button_SwitchController_clicked(bool check)
{
    std::string stop_controller = ui.StartControllerCombo->currentText().toStdString();
    std::string start_controller = ui.StopControllerCombo->currentText().toStdString();

    bool switch_succeded;
    if(qnode.switch_controllers(start_controller, stop_controller, switch_succeded) && switch_succeded)
    {
        // wait before reloading controllers because after the service call
        // it is not sure that controller list provided by controller manager is updated
        sleep(1);
        // if controller switch succeded reload controller list in combo boxes
        fill_controllers_list();
        std::string robot_name_space;
        qnode.get_robot_namespace(robot_name_space);
        // Let the ros node knows that cartesian position controller is started
        // so that it can signal when new errors are available on each topic callback
        qnode.set_jointpos_controller_state(start_controller == (robot_name_space + "/joint_space_spline_controller"));
        qnode.set_cartpos_controller_state(start_controller == (robot_name_space + "/operational_space_impedance_spline_controller"));
        //qnode.set_hybrid_controller_state(start_controller == "hybrid_impedance_controller");
        if(start_controller == (robot_name_space + "/joint_space_spline_controller"))
        {
            fill_joint_pos_fields();
            fill_joint_gains_fields();
            switch_tab(0);
        }
        if(start_controller == (robot_name_space + "/operational_space_impedance_spline_controller"))
        {
            fill_cart_pos_fields();
            fill_cart_gains_fields();
            switch_tab(1);
        }
    }

}

void MainWindow::on_button_LoadControllerList_clicked(bool check)
{
    this->fill_controllers_list();
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
        ui.view_logging->scrollToBottom();
}

void MainWindow::set_progress_bar(QProgressBar* bar, double elapsed_time, double total_time)
{
  bar->setValue(elapsed_time / total_time * 100.0);
  //qnode.log(QNode::LogLevel::Info, std::string("progress bar updated"));
  //std::stringstream ss;
  //ss << (elapsed_time / total_time * 100.0);

  //qnode.log(QNode::LogLevel::Info, std::string("progress set to") + ss.str());
  //ss.str("");
}

void MainWindow::update_joints_state()
{
  std::vector<QLabel*> labels_list;
  std::vector<double> joints_position;

  // get new joints state
  qnode.get_joints_state(joints_position);

  labels_list.push_back(ui.Joint1Position_Val);
  labels_list.push_back(ui.Joint4Position_Val);
  labels_list.push_back(ui.Joint7Position_Val);
  labels_list.push_back(ui.Joint2Position_Val);
  labels_list.push_back(ui.Joint3Position_Val);
  labels_list.push_back(ui.Joint6Position_Val);
  labels_list.push_back(ui.Joint5Position_Val); // <-- ORDERING HERE IS IMPORTANT

  for (int i=0; i<7; i++)
    labels_list.at(i)->setText(QString::number(joints_position.at(i), 'f', 3));
    //labels_list.at(i)->setText(QString::number(180.0/3.14 * joints_position.at(i), 'f', 3));
}

void MainWindow::update_joints_error()
{
  std::vector<QLabel*> labels_list;
  std::vector<double> joints_error;

  // get new joints error
  qnode.get_joints_error(joints_error);

  labels_list.push_back(ui.Joint1Error_Val);
  labels_list.push_back(ui.Joint2Error_Val);
  labels_list.push_back(ui.Joint3Error_Val);
  labels_list.push_back(ui.Joint4Error_Val);
  labels_list.push_back(ui.Joint5Error_Val);
  labels_list.push_back(ui.Joint6Error_Val);
  labels_list.push_back(ui.Joint7Error_Val); // <-- ORDERING HERE IS IMPORTANT

  for (int i=0; i<7; i++)
    labels_list.at(i)->setText(QString::number(joints_error.at(i), 'f', 3));
}

void MainWindow::update_cart_pos()
{
    geometry_msgs::Vector3 trans;
    wam_dmp_controller::RPY rot;

    // get new joints error
    qnode.get_cart_pos(trans, rot);

    ui.CartPosX_Val->setText(QString::number(trans.x, 'f', 3));
    ui.CartPosY_Val->setText(QString::number(trans.y, 'f', 3));
    ui.CartPosZ_Val->setText(QString::number(trans.z, 'f', 3));
    ui.CartPosRoll_Val->setText(QString::number(rot.roll, 'f', 3));
    ui.CartPosPitch_Val->setText(QString::number(rot.pitch, 'f', 3));
    ui.CartPosYaw_Val->setText(QString::number(rot.yaw, 'f', 3));
}

void MainWindow::update_cart_error()
{
    geometry_msgs::Vector3 trans_err;
    wam_dmp_controller::RPY rot_err;

    // get new joints error
    qnode.get_cart_error(trans_err, rot_err);

    ui.CartErrorX_Val->setText(QString::number(trans_err.x, 'f', 3));
    ui.CartErrorY_Val->setText(QString::number(trans_err.y, 'f', 3));
    ui.CartErrorZ_Val->setText(QString::number(trans_err.z, 'f', 3));
    ui.CartErrorRoll_Val->setText(QString::number(rot_err.roll, 'f', 3));
    ui.CartErrorPitch_Val->setText(QString::number(rot_err.pitch, 'f', 3));
    ui.CartErrorYaw_Val->setText(QString::number(rot_err.yaw, 'f', 3));
}


void MainWindow::update_progress_data()
{
  double jointpos_elapsed;
  double jointpos_duration;
  double cartpos_elapsed;
  double cartpos_duration;

  //void get_progress_jointpos(double& elapsed, double& duration);
  if (qnode.get_jointpos_controller_state() == true)
  {
      qnode.get_progress_jointpos(jointpos_elapsed, jointpos_duration);
      set_progress_bar(ui.progressBar_JointPos, jointpos_elapsed, jointpos_duration);
      set_send_jointpos_label(ui.JointPosSentLabel, false, jointpos_elapsed, jointpos_duration);
      set_send_jointpos_label(ui.GoHomeSentLabel, false, jointpos_elapsed, jointpos_duration);
  }

  if (qnode.get_cartpos_controller_state() == true)
  {
      qnode.get_progress_cartpos(cartpos_elapsed, cartpos_duration);
      set_progress_bar(ui.progressBar_CartPos, cartpos_elapsed, cartpos_duration);
      set_send_cartpos_label(ui.CartPosSentLabel, false, cartpos_elapsed, cartpos_elapsed);
  }

}


void MainWindow::set_send_jointpos_label(QLabel* label, bool accepted, double elapsed, double duration)
{
  QString label_text;

  if(accepted)
    label_text = QString::fromStdString("Accepted");
  else
    {
  if (elapsed >= duration)
    label_text = QString::fromStdString("Completed ");
  else
    label_text = QString::fromStdString("Wait ") +\
      QString::number(duration - elapsed, 'f', 0) +\
      QString::fromStdString(" s");
    }

  label->setText(label_text);
}

void MainWindow::set_send_cartpos_label(QLabel* label, bool accepted, double elapsed, double duration)
{
  QString label_text;

  if(accepted)
    label_text = QString::fromStdString("Accepted");
  else
    {
  if (elapsed >= duration)
    label_text = QString::fromStdString("Completed ");
  else
    label_text = QString::fromStdString("Wait ") +\
      QString::number(duration - elapsed, 'f', 0) +\
      QString::fromStdString(" s");
    }

  label->setText(label_text);
}


void MainWindow::field_error_msg_box(std::string field_name)
{
  QMessageBox msg_box;
  msg_box.setGeometry(QStyle::alignedRect(Qt::LeftToRight,
                      Qt::AlignCenter,
                      msg_box.size(),
                      qApp->desktop()->availableGeometry()));
  QString error_pre = "The value inserted in the field ";
  QString error_post = " is invalid.";

  msg_box.setText(error_pre + QString::fromStdString(field_name) + error_post);
  msg_box.exec();
}

void MainWindow::service_error_msg_box(std::string controller_name)
{
  QMessageBox msg_box;
  msg_box.setGeometry(QStyle::alignedRect(Qt::LeftToRight,
                      Qt::AlignCenter,
                      msg_box.size(),
                      qApp->desktop()->availableGeometry()));
  QString error_pre = "Service execution related to ";
  QString error_post = " failed!";

  msg_box.setText(error_pre + QString::fromStdString(controller_name) + error_post);
  msg_box.exec();
}

void MainWindow::fill_controllers_list()
{

  // remove all items from combobox
  while(ui.StopControllerCombo->count() != 0)
    ui.StopControllerCombo->removeItem(0);
  while(ui.StartControllerCombo->count() != 0)
    ui.StartControllerCombo->removeItem(0);

  std::vector<std::string> running_controllers, stopped_controllers;
  if(!qnode.get_controllers_list(running_controllers, stopped_controllers))
    std::cout << "Error while fetching controllers list." << std::endl;

  for (std::vector<std::string>::iterator it = running_controllers.begin();
   it != running_controllers.end(); ++it)
    ui.StartControllerCombo->addItem(QString::fromStdString(*it));

  for (std::vector<std::string>::iterator it = stopped_controllers.begin();
   it != stopped_controllers.end(); ++it)
    ui.StopControllerCombo->addItem(QString::fromStdString(*it));

}

void MainWindow::fill_joint_pos_fields()
{
  bool outcome;

  // Joint Space Controller
  wam_dmp_controller::JointPosSplineMsg joint_pos_curr_goal;

  outcome = qnode.get_current_cmd<wam_dmp_controller::JointPosSpline,\
                                  wam_dmp_controller::JointPosSplineMsg>(joint_pos_curr_goal);
  if(!outcome)
    service_error_msg_box("joint_space_spline_controller(JointPosSpline)");

  ui.Joint1PosText->setText(QString::number(joint_pos_curr_goal.joint_pos[0],'f', 3));
  ui.Joint2PosText->setText(QString::number(joint_pos_curr_goal.joint_pos[1],'f', 3));
  ui.Joint3PosText->setText(QString::number(joint_pos_curr_goal.joint_pos[2],'f', 3));
  ui.Joint4PosText->setText(QString::number(joint_pos_curr_goal.joint_pos[3],'f', 3));
  ui.Joint5PosText->setText(QString::number(joint_pos_curr_goal.joint_pos[4],'f', 3));
  ui.Joint6PosText->setText(QString::number(joint_pos_curr_goal.joint_pos[5],'f', 3));
  ui.Joint7PosText->setText(QString::number(joint_pos_curr_goal.joint_pos[6],'f', 3));

}

void MainWindow::fill_cart_pos_fields()
{
  bool outcome;

  // Operational Space Controller
  wam_dmp_controller::PoseRPYCmd cart_pos_curr_goal;

  outcome = qnode.get_current_cmd<wam_dmp_controller::PoseRPYCommand,\
                                  wam_dmp_controller::PoseRPYCmd>(cart_pos_curr_goal);
  if(!outcome)
    service_error_msg_box("operational_space_spline_controller(PoseRPYCommand)");

  ui.CartPosXText->setText(QString::number(cart_pos_curr_goal.position.x,'f', 3));
  ui.CartPosYText->setText(QString::number(cart_pos_curr_goal.position.y,'f', 3));
  ui.CartPosZText->setText(QString::number(cart_pos_curr_goal.position.y,'f', 3));
  ui.CartPosRollText->setText(QString::number(cart_pos_curr_goal.orientation.roll,'f', 3));
  ui.CartPosPitchText->setText(QString::number(cart_pos_curr_goal.orientation.pitch,'f', 3));
  ui.CartPosYawText->setText(QString::number(cart_pos_curr_goal.orientation.yaw,'f', 3));

}

void MainWindow::fill_joint_gains_fields()
{
  bool outcome;

  // Joint Space Controller
  wam_dmp_controller::SetJointGainsMsg joint_gains_curr_goal;

  outcome = qnode.get_current_cmd<wam_dmp_controller::GetJointGains,\
                                  wam_dmp_controller::SetJointGainsMsg>(joint_gains_curr_goal);
  if(!outcome)
    service_error_msg_box("joint_space_spline_controller(GetJointGains)");

  ui.Joint1KpGainText->setText(QString::number(joint_gains_curr_goal.Kp_gains[0],'f', 3));
  ui.Joint2KpGainText->setText(QString::number(joint_gains_curr_goal.Kp_gains[1],'f', 3));
  ui.Joint3KpGainText->setText(QString::number(joint_gains_curr_goal.Kp_gains[2],'f', 3));
  ui.Joint4KpGainText->setText(QString::number(joint_gains_curr_goal.Kp_gains[3],'f', 3));
  ui.Joint5KpGainText->setText(QString::number(joint_gains_curr_goal.Kp_gains[4],'f', 3));
  ui.Joint6KpGainText->setText(QString::number(joint_gains_curr_goal.Kp_gains[5],'f', 3));
  ui.Joint7KpGainText->setText(QString::number(joint_gains_curr_goal.Kp_gains[6],'f', 3));

  ui.Joint1KvGainText->setText(QString::number(joint_gains_curr_goal.Kv_gains[0],'f', 3));
  ui.Joint2KvGainText->setText(QString::number(joint_gains_curr_goal.Kv_gains[1],'f', 3));
  ui.Joint3KvGainText->setText(QString::number(joint_gains_curr_goal.Kv_gains[2],'f', 3));
  ui.Joint4KvGainText->setText(QString::number(joint_gains_curr_goal.Kv_gains[3],'f', 3));
  ui.Joint5KvGainText->setText(QString::number(joint_gains_curr_goal.Kv_gains[4],'f', 3));
  ui.Joint6KvGainText->setText(QString::number(joint_gains_curr_goal.Kv_gains[5],'f', 3));
  ui.Joint7KvGainText->setText(QString::number(joint_gains_curr_goal.Kv_gains[6],'f', 3));


}

void MainWindow::fill_cart_gains_fields()
{
    bool outcome;
    // Joint Space Controller
    wam_dmp_controller::ImpedanceControllerGainsMsg cart_gains_curr_goal;

    outcome = qnode.get_current_cmd<wam_dmp_controller::ImpedanceControllerGains,\
                                    wam_dmp_controller::ImpedanceControllerGainsMsg>(cart_gains_curr_goal);
    if(!outcome)
      service_error_msg_box("operational_space_spline_controller(ImepdanceControllerGains)");

    ui.CartPosXGainText_Kp->setText(QString::number(cart_gains_curr_goal.Kp_gains[0],'f', 3));
    ui.CartPosYGainText_Kp->setText(QString::number(cart_gains_curr_goal.Kp_gains[1],'f', 3));
    ui.CartPosZGainText_Kp->setText(QString::number(cart_gains_curr_goal.Kp_gains[2],'f', 3));
    ui.CartPosRollGainText_Kp->setText(QString::number(cart_gains_curr_goal.Kp_gains[3],'f', 3));
    ui.CartPosPitchGainText_Kp->setText(QString::number(cart_gains_curr_goal.Kp_gains[4],'f', 3));
    ui.CartPosYawGainText_Kp->setText(QString::number(cart_gains_curr_goal.Kp_gains[5],'f', 3));

    ui.CartPosXGainText_Kv->setText(QString::number(cart_gains_curr_goal.Kv_gains[0],'f', 3));
    ui.CartPosYGainText_Kv->setText(QString::number(cart_gains_curr_goal.Kv_gains[1],'f', 3));
    ui.CartPosZGainText_Kv->setText(QString::number(cart_gains_curr_goal.Kv_gains[2],'f', 3));
    ui.CartPosRollGainText_Kv->setText(QString::number(cart_gains_curr_goal.Kv_gains[3],'f', 3));
    ui.CartPosPitchGainText_Kv->setText(QString::number(cart_gains_curr_goal.Kv_gains[4],'f', 3));
    ui.CartPosYawGainText_Kv->setText(QString::number(cart_gains_curr_goal.Kv_gains[5],'f', 3));

    ui.JointGainText_nullKp_1->setText(QString::number(cart_gains_curr_goal.null_Kp_gains[0],'f', 3));
    ui.JointGainText_nullKp_2->setText(QString::number(cart_gains_curr_goal.null_Kp_gains[1],'f', 3));
    ui.JointGainText_nullKp_3->setText(QString::number(cart_gains_curr_goal.null_Kp_gains[2],'f', 3));
    ui.JointGainText_nullKp_4->setText(QString::number(cart_gains_curr_goal.null_Kp_gains[3],'f', 3));
    ui.JointGainText_nullKp_5->setText(QString::number(cart_gains_curr_goal.null_Kp_gains[4],'f', 3));
    ui.JointGainText_nullKp_6->setText(QString::number(cart_gains_curr_goal.null_Kp_gains[5],'f', 3));
    ui.JointGainText_nullKp_7->setText(QString::number(cart_gains_curr_goal.null_Kp_gains[6],'f', 3));

    ui.JointGainText_nullKv_1->setText(QString::number(cart_gains_curr_goal.null_Kv_gains[0],'f', 3));
    ui.JointGainText_nullKv_2->setText(QString::number(cart_gains_curr_goal.null_Kv_gains[1],'f', 3));
    ui.JointGainText_nullKv_3->setText(QString::number(cart_gains_curr_goal.null_Kv_gains[2],'f', 3));
    ui.JointGainText_nullKv_4->setText(QString::number(cart_gains_curr_goal.null_Kv_gains[3],'f', 3));
    ui.JointGainText_nullKv_5->setText(QString::number(cart_gains_curr_goal.null_Kv_gains[4],'f', 3));
    ui.JointGainText_nullKv_6->setText(QString::number(cart_gains_curr_goal.null_Kv_gains[5],'f', 3));
    ui.JointGainText_nullKv_7->setText(QString::number(cart_gains_curr_goal.null_Kv_gains[6],'f', 3));
}

void MainWindow::switch_tab(int index)
{
  ui.ErrorsTabWidget->setCurrentIndex(index);
  ui.ControllersTabWidget->setCurrentIndex(index);
}


/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "barrett_controller_GUI");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
    	ui.line_edit_master->setEnabled(false);
    	ui.line_edit_host->setEnabled(false);
    	//ui.line_edit_topic->setEnabled(false);
    }
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "barrett_controller_GUI");
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}

}  // namespace barrett_controller_GUI

