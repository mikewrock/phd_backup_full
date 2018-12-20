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
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <phd/arm_msg.h>
#include "phd/empty.h"
#include <boost/foreach.hpp>
#include <visualization_msgs/Marker.h>
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
	void cpCB(const std_msgs::String msg);
protected Q_SLOTS:
//All of these functions are explained in control_dashboard.cpp
void do_nav();
void show_nav();
void arm_step();
void arm_step_man();
void arm_loop();
void fake_scan();
void fake_scan2();
void scan();
void prescan();
void postscan();
void homing_scan();
void cluster();
void gen_trajectory();	
void onPTPCommand();
void onCPCommand();
void onTCommand();
void onJointCommand();
void onStringCommand();
void onShutdownCommand();
void onSpeedCommand();
void load_traj();
void print_markers();
void clear_markers();
void rosSpinner();
void soft_stop();
void pose_step();
void pose_loop();
void reset_map();
void set_clean();
void set_sprayed();
void calculate_thickness();
void thickness_from_file();
void save_selection();
void process_data();
void analyze();
float tgt_distance(phd::arm_msg arm, phd::trajectory_point tgt);
Q_SIGNALS:

protected:
  Ui::Control_Form ui_;
  ros::NodeHandle nh_;
  ros::Subscriber pose_sub_;
  ros::Subscriber cloud_sub_;
  ros::Subscriber move_base_sub_;
  ros::Publisher reset_map_pub_;
  ros::ServiceClient reset_map_srv;

private:
	//Create the control panel and manual operation nodes
	control_panel_ns::QNode control_panel;
	//this is used to keep track of whick marker point is being clustered
	int cluster_index;
	int traj_ctr, traj_size;
	phd::arm_msg next_tgt;
	phd::trajectory_point arm_tgt;
	bool auto_pose, auto_arm, auto_traj;
	std_msgs::String CLOUD, TRAJ, SCAN_COMPLETE;

};
}
#endif // DASHBOARD_PANEL_H
