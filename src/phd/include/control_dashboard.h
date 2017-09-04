#ifndef CONTROL_DASHBOARD_PANEL_H
#define CONTROL_DASHBOARD_PANEL_H

#ifndef Q_MOC_RUN
# include <ros/ros.h>

# include <rviz/panel.h>
#endif

#include <ui_control_panel.h>
#include <QProcess>
#include <QDir>
#include <QFileInfo>
#include "control_panel.hpp"
#include "vel_control.hpp"

namespace control_panel
{

class ControlPanel: public rviz::Panel
{
Q_OBJECT
public:

  explicit ControlPanel( QWidget* parent = 0);
  virtual ~ControlPanel();

protected:
protected Q_SLOTS:
//All of these functions are explained in control_dashboard.cpp
void do_nav();
void show_nav();
void exe_nav();
void do_step();
void do_estimate();
void fake_scan();
void scan();
void thickness();
void start_pt();
void fwd_vel();
void rev_vel();
void left_vel();
void right_vel();
void arm_fwd_vel();
void arm_left_vel();
void arm_right_vel();
void arm_rev_vel();
void arm_up_vel();
void arm_down_vel();
void roll_vel_neg();
void roll_vel_pos();
void pitch_vel_neg();
void pitch_vel_pos();
void yaw_vel_neg();
void yaw_vel_pos();
void localization_scan();
void localization_scan_2();
void clear_vel();
void cluster_1();
void trajectory();
void cluster_2();
void cluster_3();
void gen_trajectory();
Q_SIGNALS:

protected:
  Ui::Control_Form ui_;


private:
	//Create the control panel and manual operation nodes
	control_panel_ns::QNode control_panel;
	husky_control::QNode vel_control;
	//this is used to keep track of whick marker point is being clustered
	int cluster_index;

};
}
#endif // DASHBOARD_PANEL_H
