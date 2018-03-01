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
#include "../include/barrett_controller_switcher/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace barrett_controller_switcher {

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

void MainWindow::on_button_SetGains_clicked(bool check)
{
    bool outcome;
    bool state;
    std::vector<double> Kp_(7), Kv_(7);
    wam_dmp_controller::SetJointGainsMsg cmd, res;

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
    outcome = qnode.set_command<wam_dmp_controller::SetJointGains, wam_dmp_controller::SetJointGainsMsg>(cmd, res, state);
    if(!outcome)
      service_error_msg_box("joint_space_controller(SetJointGains)");
}

void MainWindow::on_button_GoHome_clicked(bool check)
{
    bool outcome;
    bool state;
    wam_dmp_controller::GoHomeMsg cmd, res;
    outcome = qnode.set_command<wam_dmp_controller::GoHome, wam_dmp_controller::GoHomeMsg>(cmd, res, state);
    if(!outcome)
      service_error_msg_box("joint_space_controller(GoHome)");
    else
      set_send_jointpos_label(ui.GoHomeSentLabel, state);
}

void MainWindow::on_button_SendJointPos_clicked(bool check)
{
    double j1, j2, j3, j4, j5, j6, j7;
    bool outcome;

    j1 = ui.Joint1PosText->text().toDouble(&outcome);
    if (!outcome)
    {
        field_error_msg_box("Joint1PosText");
        return;
    }

    j2 = ui.Joint2PosText->text().toDouble(&outcome);
    if (!outcome)
    {
        field_error_msg_box("Joint2PosText");
        return;
    }

    j3 = ui.Joint3PosText->text().toDouble(&outcome);
    if (!outcome)
    {
        field_error_msg_box("Joint3PosText");
        return;
    }

    j4 = ui.Joint4PosText->text().toDouble(&outcome);
    if (!outcome)
    {
        field_error_msg_box("Joint4PosText");
        return;
    }

    j5 = ui.Joint5PosText->text().toDouble(&outcome);
    if (!outcome)
    {
        field_error_msg_box("Joint5PosText");
        return;
    }

    j6 = ui.Joint6PosText->text().toDouble(&outcome);
    if (!outcome)
    {
        field_error_msg_box("Joint6PosText");
        return;
    }

    j7 = ui.Joint7PosText->text().toDouble(&outcome);
    if (!outcome)
    {
        field_error_msg_box("Joint7PosText");
        return;
    }

    wam_dmp_controller::SetJointPosMsg cmd, res;
    cmd.joint_pos[0] = j1;
    cmd.joint_pos[1] = j2;
    cmd.joint_pos[2] = j3;
    cmd.joint_pos[3] = j4;
    cmd.joint_pos[4] = j5;
    cmd.joint_pos[5] = j6;
    cmd.joint_pos[6] = j7;

    bool state;
    outcome = qnode.set_command<wam_dmp_controller::SetJointPos, wam_dmp_controller::SetJointPosMsg>(cmd, res, state);

    if(!outcome)
      service_error_msg_box("joint_space_controller(SetJointPos)");
    else
      set_send_jointpos_label(ui.JointPosSentLabel, state);
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
        // Let the ros node knows that cartesian position controller is started
        // so that it can signal when new errors are available on each topic callback
        //qnode.set_jointpos_controller_state(start_controller == "joint_space_controller");
        //qnode.set_hybrid_controller_state(start_controller == "hybrid_impedance_controller");
        if(start_controller == "barrett_hw/joint_space_controller")
        {
            fill_joint_pos_fields();
            fill_joint_gains_fields();
            switch_tab(0);
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

void MainWindow::set_send_jointpos_label(QLabel* label, bool state)
{
  QString label_text;

  if(state)
    label_text = QString::fromStdString("Accepted");
  else
    {
      label_text = QString::fromStdString("Wait");
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
  wam_dmp_controller::SetJointPosMsg joint_pos_curr_goal;

  outcome = qnode.get_current_cmd<wam_dmp_controller::GetJointPos,\
                                  wam_dmp_controller::SetJointPosMsg>(joint_pos_curr_goal);
  if(!outcome)
    service_error_msg_box("joint_space_controller(GetJointPos)");

  ui.Joint1PosText->setText(QString::number(joint_pos_curr_goal.joint_pos[0],'f', 3));
  ui.Joint2PosText->setText(QString::number(joint_pos_curr_goal.joint_pos[1],'f', 3));
  ui.Joint3PosText->setText(QString::number(joint_pos_curr_goal.joint_pos[2],'f', 3));
  ui.Joint4PosText->setText(QString::number(joint_pos_curr_goal.joint_pos[3],'f', 3));
  ui.Joint5PosText->setText(QString::number(joint_pos_curr_goal.joint_pos[4],'f', 3));
  ui.Joint6PosText->setText(QString::number(joint_pos_curr_goal.joint_pos[5],'f', 3));
  ui.Joint7PosText->setText(QString::number(joint_pos_curr_goal.joint_pos[6],'f', 3));

}

void MainWindow::fill_joint_gains_fields()
{
  bool outcome;

  // Joint Space Controller
  wam_dmp_controller::SetJointGainsMsg joint_gains_curr_goal;

  outcome = qnode.get_current_cmd<wam_dmp_controller::GetJointGains,\
                                  wam_dmp_controller::SetJointGainsMsg>(joint_gains_curr_goal);
  if(!outcome)
    service_error_msg_box("joint_space_controller(GetJointGains)");

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

void MainWindow::switch_tab(int index)
{
  //ui.ErrorsTabWidget->setCurrentIndex(index);
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
    QSettings settings("Qt-Ros Package", "barrett_controller_switcher");
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
    QSettings settings("Qt-Ros Package", "barrett_controller_switcher");
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

}  // namespace barrett_controller_switcher

