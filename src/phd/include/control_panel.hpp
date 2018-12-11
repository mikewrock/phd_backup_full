#ifndef control_panel_AQNODE_HPP_
#define control_panel_AQNODE_HPP_

// Always goes first
#define _CUSTOM_UINT64c
#define _USE_MATH_DEFINES
// ROS
#include <ros/ros.h>
# include <ros/node_handle.h>
# include <ros/publisher.h>
# include <geometry_msgs/PoseStamped.h>
# include <geometry_msgs/PoseArray.h>
# include "rviz/tool.h"
#include "bcap/stdint.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <math.h>
#include <string>
#include <QThread>
#include <QStringListModel>
# include <QCursor>
# include <QObject>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
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
#include "phd/empty.h"
#include "phd/arm_msg.h"
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
	void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
	QNode(){}
	virtual ~QNode();
	void scan(std::string, bool localize, std::string);
	phd::trajectory_point step(int traj_ctr, bool arm, float fig);
	void show_nav();
	void thickness_from_file(std::string,std::string);
	void nav_mode(float pos);
	void fscan(std::string, bool auto_localize,bool set_home,std::string,bool autocrop);
	void cluster(std::string, int index);
	int gen_trajectory(std::string);
	int load_traj(std::string);
	void run();
	bool init();
	void send_command(float x, float y, float z, float rx, float ry, float rz, float fig, int motion);
	void send_joint_command(float j1, float j2, float j3, float j4, float j5, float j6);
	void send_string(std::string user_string);
	void calculate_thickness(std::string);
	void set_clean();
	void save_selection();
	void set_sprayed();
	void set_speed(float speed);
	void soft_stop();
	void reset_things();
	bool check_pie(float,float);
	void find_and_move(bool marker);
	pcl::PointXYZI find_and_delete_point(pcl::PointCloud<pcl::PointXYZI>::Ptr line, pcl::PointXYZI target_pt);
	void show_markers(phd::trajectory_msg t_array);
	//Variables for holding the first two points of the marker, when the third one comes in they all get saved to file
	float P1x, P1y, P1z, P2x, P2y, P2z;
	pcl::PointCloud<pcl::PointXYZI>::Ptr raw_pc_;
	//Holds the entire arm trajectory
	phd::trajectory_msg traj;


Q_SIGNALS:
	void rosShutdown();

private:
	int init_argc;
	char** init_argv;
	ros::NodeHandle nh_;
	//Subscribers, Publishers, and Services explained in control_panel.cpp
	ros::Subscriber joint_sub;
	ros::Subscriber cloud_sub;
	ros::Subscriber pose_sub;
	ros::Publisher cmd_pub;
	ros::Publisher arm_pub;
	ros::Publisher nav_vis_pub;
	ros::Publisher pub;
	ros::Publisher rawpub;
	ros::Publisher marker_pub;
	ros::Publisher dir_pub;
	ros::Publisher path_pub;
	ros::Publisher arm_pose_pub;
	ros::Publisher reset_map_pub;
	ros::Publisher finished_scan_pub;
	ros::ServiceClient reset_map_srv;
	ros::ServiceClient client;
	ros::ServiceClient loc_client;
	ros::ServiceClient traj_client;
	ros::ServiceClient thick_client;
	//Dynamic Reconfigure
	dynamic_reconfigure::Server<phd::ParamConfig> server;
	dynamic_reconfigure::Server<phd::ParamConfig>::CallbackType callback_type;
	//Pointer to the pointcloud selection
	pcl::PointCloud<pcl::PointXYZI>::Ptr current_pc_;
	sensor_msgs::PointCloud2 clean_cloud;
	sensor_msgs::PointCloud2 sprayed_cloud;
	//Holds the entire pointcloud from laser_scan_assembler, used for calculating normals (useful at the edges of current_pc_)
	sensor_msgs::PointCloud2 cloud_surface_raw;
	sensor_msgs::PointCloud2 cloud_surface_world;
	int cloud_ctr;
	int tctr;
	tf::TransformBroadcaster br;
	tf::Transform transform;
	tf::Quaternion q;
	geometry_msgs::PoseArray goals;
};
}
}  // namespace 

#endif /* control_panel_QNODE_HPP_ */
