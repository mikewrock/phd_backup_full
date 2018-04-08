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
//#include "vel_control.hpp"
#include <phd/arm_msg.h>
#include "phd/empty.h"

namespace control_panel
{

class ControlPanel: public rviz::Panel
{
Q_OBJECT
public:

  explicit ControlPanel( QWidget* parent = 0);
  virtual ~ControlPanel();

protected:
void poseCB(const phd::arm_msg msg);
void movebaseCB(const move_base_msgs::MoveBaseActionResult result);
protected Q_SLOTS:
//All of these functions are explained in control_dashboard.cpp
void do_nav();
void show_nav();
void exe_nav();
void arm_step();
void do_estimate();
void fake_scan();
void scan();
void thickness();
void start_pt();
void localization_scan();
void localization_scan_2();
void cluster_1();
void trajectory();
void cluster_2();
void cluster_3();
void gen_trajectory();	
void onPTPCommand();
void onCPCommand();
void onTCommand();
void onJointCommand();
void onStringCommand();
void onShutdownCommand();
void onSpeedCommand();
void scan_360();
void load_traj();
void rosSpinner();
void do_poses();
void soft_stop();
void pose_step();
void pose_loop();
void arm_loop();
void reset_map();
void set_clean();
void set_sprayed();
void calculate_thickness();
void compare_results();
void save_selection();
float tgt_distance(phd::arm_msg arm, phd::arm_msg tgt);
Q_SIGNALS:

protected:
  Ui::Control_Form ui_;
  ros::NodeHandle nh_;
  ros::Subscriber pose_sub_;
  ros::Subscriber move_base_sub_;
  ros::Publisher reset_map_pub_;
  ros::ServiceClient reset_map_srv;

private:
	//Create the control panel and manual operation nodes
	control_panel_ns::QNode control_panel;
	//this is used to keep track of whick marker point is being clustered
	int cluster_index;
	int pose_ctr, pose_size, traj_ctr, traj_size;
	phd::arm_msg arm_tgt;
	bool auto_pose, auto_arm;

};
}
#endif // DASHBOARD_PANEL_H
