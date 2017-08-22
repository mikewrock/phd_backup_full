/**
 * @file /include/test_panel/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef control_panel_AQNODE_HPP_
#define control_panel_AQNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/
// Always goes first
#define _CUSTOM_UINT64c
// ROS
#include <ros/ros.h>
# include <ros/node_handle.h>
# include <ros/publisher.h>
# include "rviz/tool.h"
#include "bcap/stdint.h"
//#include <atlbase.h>

#define _USE_MATH_DEFINES
#include <math.h>
#include <string>
#include <QThread>
#include <QStringListModel>
# include <QCursor>
# include <QObject>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/JointState.h>

#include "std_msgs/String.h"
#include <sstream>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <dynamic_reconfigure/server.h>
#include <phd/ParamConfig.h>
#include "phd/trajectory_array.h"
#include <pcl/filters/extract_indices.h>
// bCAP (Always last)
#include "bcap/bcap_client.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace control_panel {
namespace control_panel_ns {


/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	//See control_panel.cpp for description of these functions
	void jointCallback(const sensor_msgs::JointState::ConstPtr& msg);
	void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);
	QNode(){}
	virtual ~QNode();
	void scan(std::string, bool localize);
	void step();
	void show_nav();
	void exe_nav();
	void estimate(std::string);
	void nav_mode(float pos);
	void fscan(std::string, bool auto_localize);
	void cluster(std::string, int index);
	void start_pt();
	void gen_trajectory();
	void lscan();
	void run();
	bool init();
	phd::trajectory_point find_pose();
	//Variables for holding the first two points of the marker, when the third one comes in they all get saved to file
	float P1x, P1y, P1z, P2x, P2y, P2z;
	//Variables for holding the start point for the trajectory algorithm
	float T1x, T1y, T1z;


Q_SIGNALS:
	void rosShutdown();

private:
	int init_argc;
	char** init_argv;
	ros::NodeHandle nh_;
	//Subscribers, Publishers, and Services explained in control_panel.cpp
	ros::Subscriber joint_sub;
	ros::Subscriber cloud_sub;
	ros::Subscriber traj_sub;
	ros::Publisher cmd_pub;
	ros::Publisher arm_pub;
	ros::Publisher nav_vis_pub;
	ros::Publisher nav_pub;
	ros::Publisher pub;
	ros::Publisher pub2;
	ros::Publisher pub3;
	ros::Publisher pub4;
	ros::Publisher pub5;
	ros::ServiceClient client;
	ros::ServiceClient loc_client;
	ros::ServiceClient traj_client;
	//Dynamic Reconfigure
	dynamic_reconfigure::Server<phd::ParamConfig> server;
	dynamic_reconfigure::Server<phd::ParamConfig>::CallbackType callback_type;
	//Pointer to the pointcloud selection
	pcl::PointCloud<pcl::PointXYZI>::Ptr current_pc_;
	//Holds the entire pointcloud from laser_scan_assembler, used for calculating normals (useful at the edges of current_pc_)
	sensor_msgs::PointCloud2 cloud_surface;
	//Holds the entire arm trajectory
	phd::trajectory_array traj;
	//Counters for keeping track of current trajectory section and point
	int sec_ctr;
	int pt_ctr;
	int cloud_ctr;
};
}
}  // namespace 

#endif /* control_panel_QNODE_HPP_ */
