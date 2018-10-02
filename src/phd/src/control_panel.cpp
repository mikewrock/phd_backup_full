// Always goes first
#define _CUSTOM_UINT64
// ROS
#include <ros/ros.h>
//Header for this file
#include "../include/control_panel.hpp"
//ROS includes
#define _USE_MATH_DEFINES
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>  
#include <sensor_msgs/PointCloud2.h>
#include <ros/time.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float64.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <actionlib/client/simple_action_client.h>
//C++ includes
#include <math.h>
#include <string>
#include <cstdio>
#include <QVariant>
#include <iostream>
// Services
#include "laser_assembler/AssembleScans2.h"
#include "phd/localize_cloud.h"
#include "phd/simple_trajectory_service.h"
#include "phd/thickness_service.h"
// Messages
#include "sensor_msgs/PointCloud.h"
#include <phd/cube_msg.h>
#include <phd/arm_msg.h>
//PCL includes
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/search.h>
#include <pcl/keypoints/susan.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>
#include <pcl/filters/impl/box_clipper3D.hpp>
#include <pcl/features/fpfh.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/common/angles.h>
#include <pcl/filters/crop_box.h>
#define CALC true
#define DEBUG true
#define PTP 1
#define CP 2
#define JOINT 3
#define STRING 4
#define SPEED 7
#define WORKSPACE_WIDTH 0.5
#define H_SLICE 0.5
#define CHUNK_RADIUS 0.03
#define POSE_DISTANCE 0.5
#define X_BASE_TO_ARM .156971
#define Y_BASE_TO_ARM -.096013
#define Z_BASE_TO_ARM .405369

namespace control_panel{
namespace control_panel_ns{

//For displaying arm trajectory
visualization_msgs::Marker point_list;
visualization_msgs::Marker armpoint_list;
//For keeping track of the powercube joint
double joint1;
//For assembling the laser scans
laser_assembler::AssembleScans2 srv;
//For localizing a pointcloud
phd::localize_cloud loc_srv;
//For generating arm trajectories
phd::simple_trajectory_service traj_srv;
//Dynamic Reconfig Variables
std::string CLOUD;
int CLOUD_NUM;
float VINT_MIN,VINT_MAX;
float POSE_OFFSET;
double cropMaxX, cropMaxY, cropMaxZ, cropMinX, cropMinY, cropMinZ;
double occludeMaxX, occludeMaxY, occludeMaxZ, occludeMinX, occludeMinY, occludeMinZ;
	ros::Publisher crop_pub;
bool pose_waiting, marker_only, INIT_CLEAR;
std_msgs::String RESET_MAP, CLOUD_STR, TRAJ, SCAN_COMPLETE;

//This is our client for sending move_base goals
actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient("move_base", true);

//Destructor
QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}
//this callback is for dynamic reconfigure, it sets the global variables
void reconfig_callback(phd::ParamConfig &config, uint32_t level) {

	CLOUD = config.cloud_name;
	CLOUD_NUM = config.cloud_number;
	VINT_MIN = config.intensity_min;
	VINT_MAX = config.intensity_max;
	cropMaxX = config.auto_crop_max_x;
	cropMaxY = config.auto_crop_max_y;
	cropMaxZ = config.auto_crop_max_z;
	cropMinX = config.auto_crop_min_x;
	cropMinY = config.auto_crop_min_y;
	cropMinZ = config.auto_crop_min_z;
	occludeMaxX = config.auto_occlude_max_x;
	occludeMaxY = config.auto_occlude_max_y;
	occludeMaxZ = config.auto_occlude_max_z;
	occludeMinX = config.auto_occlude_min_x;
	occludeMinY = config.auto_occlude_min_y;
	occludeMinZ = config.auto_occlude_min_z;
	POSE_OFFSET = config.pose_offset;
	INIT_CLEAR = config.init_clear;
	/*//Generate a marker to visualize the auto-crop selection
	visualization_msgs::Marker crop_marker, keep_marker;
	crop_marker.header.frame_id = "/world";
	crop_marker.header.stamp = ros::Time::now();
	crop_marker.ns = "basic_shapes";
	crop_marker.id = 0;
	crop_marker.type = visualization_msgs::Marker::LINE_LIST;
	crop_marker.action = visualization_msgs::Marker::ADD;
	crop_marker.scale.x = .02;
	crop_marker.color.r = 0.0f;
	crop_marker.color.g = 1.0f;
	crop_marker.color.b = 1.0f;
	crop_marker.color.a = 1;
	crop_marker.lifetime = ros::Duration();
	crop_marker.points.resize(26);
	crop_marker.points[0].x = occludeMinX;
	crop_marker.points[0].y = occludeMinY;
	crop_marker.points[0].z = occludeMinZ;
	crop_marker.points[1].x = occludeMinX;
	crop_marker.points[1].y = occludeMaxY;
	crop_marker.points[1].z = occludeMinZ;
	crop_marker.points[2].x = occludeMinX;
	crop_marker.points[2].y = occludeMinY;
	crop_marker.points[2].z = occludeMinZ;
	crop_marker.points[3].x = occludeMinX;
	crop_marker.points[3].y = occludeMinY;
	crop_marker.points[3].z = occludeMaxZ;
	crop_marker.points[4].x = occludeMinX;
	crop_marker.points[4].y = occludeMinY;
	crop_marker.points[4].z = occludeMinZ;
	crop_marker.points[5].x = occludeMaxX;
	crop_marker.points[5].y = occludeMinY;
	crop_marker.points[5].z = occludeMinZ;
	crop_marker.points[6].x = occludeMaxX;
	crop_marker.points[6].y = occludeMinY;
	crop_marker.points[6].z = occludeMinZ;
	crop_marker.points[7].x = occludeMaxX;
	crop_marker.points[7].y = occludeMaxY;
	crop_marker.points[7].z = occludeMinZ;
	crop_marker.points[8].x = occludeMaxX;
	crop_marker.points[8].y = occludeMinY;
	crop_marker.points[8].z = occludeMinZ;
	crop_marker.points[9].x = occludeMaxX;
	crop_marker.points[9].y = occludeMinY;
	crop_marker.points[9].z = occludeMaxZ;

	crop_marker.points[10].x = occludeMaxX;
	crop_marker.points[10].y = occludeMaxY;
	crop_marker.points[10].z = occludeMaxZ;
	crop_marker.points[11].x = occludeMaxX;
	crop_marker.points[11].y = occludeMaxY;
	crop_marker.points[11].z = occludeMinZ;
	crop_marker.points[12].x = occludeMaxX;
	crop_marker.points[12].y = occludeMaxY;
	crop_marker.points[12].z = occludeMaxZ;
	crop_marker.points[13].x = occludeMaxX;
	crop_marker.points[13].y = occludeMinY;
	crop_marker.points[13].z = occludeMaxZ;
	crop_marker.points[14].x = occludeMaxX;
	crop_marker.points[14].y = occludeMaxY;
	crop_marker.points[14].z = occludeMaxZ;
	crop_marker.points[15].x = occludeMinX;
	crop_marker.points[15].y = occludeMaxY;
	crop_marker.points[15].z = occludeMaxZ;
	crop_marker.points[16].x = occludeMinX;
	crop_marker.points[16].y = occludeMaxY;
	crop_marker.points[16].z = occludeMaxZ;
	crop_marker.points[17].x = occludeMinX;
	crop_marker.points[17].y = occludeMaxY;
	crop_marker.points[17].z = occludeMinZ;
	crop_marker.points[18].x = occludeMinX;
	crop_marker.points[18].y = occludeMaxY;
	crop_marker.points[18].z = occludeMaxZ;
	crop_marker.points[19].x = occludeMaxX;
	crop_marker.points[19].y = occludeMaxY;
	crop_marker.points[19].z = occludeMaxZ;

	crop_marker.points[20].x = occludeMinX;
	crop_marker.points[20].y = occludeMaxY;
	crop_marker.points[20].z = occludeMinZ;
	crop_marker.points[21].x = occludeMaxX;
	crop_marker.points[21].y = occludeMaxY;
	crop_marker.points[21].z = occludeMinZ;
	crop_marker.points[22].x = occludeMinX;
	crop_marker.points[22].y = occludeMinY;
	crop_marker.points[22].z = occludeMaxZ;
	crop_marker.points[23].x = occludeMaxX;
	crop_marker.points[23].y = occludeMinY;
	crop_marker.points[23].z = occludeMaxZ;
	crop_marker.points[24].x = occludeMinX;
	crop_marker.points[24].y = occludeMinY;
	crop_marker.points[24].z = occludeMaxZ;
	crop_marker.points[25].x = occludeMinX;
	crop_marker.points[25].y = occludeMaxY;
	crop_marker.points[25].z = occludeMaxZ;
	
	

	keep_marker.header.frame_id = "/world";
	keep_marker.header.stamp = ros::Time::now();
	keep_marker.ns = "basic_shapes";
	keep_marker.id = 0;
	keep_marker.type = visualization_msgs::Marker::LINE_LIST;
	keep_marker.action = visualization_msgs::Marker::ADD;
	keep_marker.scale.x = .02;
	keep_marker.color.r = 1.0f;
	keep_marker.color.g = 0.2f;
	keep_marker.color.b = 0.0f;
	keep_marker.color.a = 1;
	keep_marker.lifetime = ros::Duration();
	keep_marker.points.resize(26);
	keep_marker.points[30].x = cropMinX;
	keep_marker.points[0].y = cropMinY;
	keep_marker.points[0].z = cropMinZ;
	keep_marker.points[1].x = cropMinX;
	keep_marker.points[1].y = cropMaxY;
	keep_marker.points[1].z = cropMinZ;
	keep_marker.points[2].x = cropMinX;
	keep_marker.points[2].y = cropMinY;
	keep_marker.points[2].z = cropMinZ;
	keep_marker.points[3].x = cropMinX;
	keep_marker.points[3].y = cropMinY;
	keep_marker.points[3].z = cropMaxZ;
	keep_marker.points[4].x = cropMinX;
	keep_marker.points[4].y = cropMinY;
	keep_marker.points[4].z = cropMinZ;
	keep_marker.points[5].x = cropMaxX;
	keep_marker.points[5].y = cropMinY;
	keep_marker.points[5].z = cropMinZ;
	keep_marker.points[6].x = cropMaxX;
	keep_marker.points[6].y = cropMinY;
	keep_marker.points[6].z = cropMinZ;
	keep_marker.points[7].x = cropMaxX;
	keep_marker.points[7].y = cropMaxY;
	keep_marker.points[7].z = cropMinZ;
	keep_marker.points[8].x = cropMaxX;
	keep_marker.points[8].y = cropMinY;
	keep_marker.points[8].z = cropMinZ;
	keep_marker.points[9].x = cropMaxX;
	keep_marker.points[9].y = cropMinY;
	keep_marker.points[9].z = cropMaxZ;

	keep_marker.points[10].x = cropMaxX;
	keep_marker.points[10].y = cropMaxY;
	keep_marker.points[10].z = cropMaxZ;
	keep_marker.points[11].x = cropMaxX;
	keep_marker.points[11].y = cropMaxY;
	keep_marker.points[11].z = cropMinZ;
	keep_marker.points[12].x = cropMaxX;
	keep_marker.points[12].y = cropMaxY;
	keep_marker.points[12].z = cropMaxZ;
	keep_marker.points[13].x = cropMaxX;
	keep_marker.points[13].y = cropMinY;
	keep_marker.points[13].z = cropMaxZ;
	keep_marker.points[14].x = cropMaxX;
	keep_marker.points[14].y = cropMaxY;
	keep_marker.points[14].z = cropMaxZ;
	keep_marker.points[15].x = cropMinX;
	keep_marker.points[15].y = cropMaxY;
	keep_marker.points[15].z = cropMaxZ;
	keep_marker.points[16].x = cropMinX;
	keep_marker.points[16].y = cropMaxY;
	keep_marker.points[16].z = cropMaxZ;
	keep_marker.points[17].x = cropMinX;
	keep_marker.points[17].y = cropMaxY;
	keep_marker.points[17].z = cropMinZ;
	keep_marker.points[18].x = cropMinX;
	keep_marker.points[18].y = cropMaxY;
	keep_marker.points[18].z = cropMaxZ;
	keep_marker.points[19].x = cropMaxX;
	keep_marker.points[19].y = cropMaxY;
	keep_marker.points[19].z = cropMaxZ;

	keep_marker.points[20].x = cropMinX;
	keep_marker.points[20].y = cropMaxY;
	keep_marker.points[20].z = cropMinZ;
	keep_marker.points[21].x = cropMaxX;
	keep_marker.points[21].y = cropMaxY;
	keep_marker.points[21].z = cropMinZ;
	keep_marker.points[22].x = cropMinX;
	keep_marker.points[22].y = cropMinY;
	keep_marker.points[22].z = cropMaxZ;
	keep_marker.points[23].x = cropMaxX;
	keep_marker.points[23].y = cropMinY;
	keep_marker.points[23].z = cropMaxZ;

	keep_marker.points[24].x = cropMinX;
	keep_marker.points[24].y = cropMaxY;
	keep_marker.points[24].z = cropMaxZ;
	keep_marker.points[25].x = cropMinX;
	keep_marker.points[25].y = cropMinY;
	keep_marker.points[25].z = cropMaxZ;
	

	crop_pub.publish(crop_marker);
	ros::Duration(0.5).sleep();
	ros::spinOnce();
	crop_pub.publish(keep_marker);
	ros::Duration(0.5).sleep();
	ros::spinOnce();
*/
//Generate a marker to visualize the auto-crop selection
	visualization_msgs::Marker crop_marker;
	crop_marker.header.frame_id = "/world";
	crop_marker.header.stamp = ros::Time::now();
	crop_marker.ns = "basic_shapes";
	crop_marker.id = 0;
	crop_marker.type = visualization_msgs::Marker::LINE_LIST;
	crop_marker.action = visualization_msgs::Marker::ADD;
	crop_marker.scale.x = .02;
	crop_marker.lifetime = ros::Duration();
	crop_marker.colors.resize(52);
	std_msgs::ColorRGBA teal;
	teal.r = 0.0f;
	teal.g = 1.0f;
	teal.b = 1.0f;
	teal.a = 1;
	std_msgs::ColorRGBA pink;
	pink.r = 1.0f;
	pink.g = 0.5f;
	pink.b = 0.0f;
	pink.a = 1;
	for(int i = 0; i < 24; ++i) crop_marker.colors[i] = pink;
	crop_marker.colors[50] = pink;
	crop_marker.colors[51] = pink;
	for(int i = 24; i < 50; ++i) crop_marker.colors[i] = teal;
	crop_marker.points.resize(52);
	crop_marker.points[0].x = occludeMinX;
	crop_marker.points[0].y = occludeMinY;
	crop_marker.points[0].z = occludeMinZ;
	crop_marker.points[1].x = occludeMinX;
	crop_marker.points[1].y = occludeMaxY;
	crop_marker.points[1].z = occludeMinZ;
	crop_marker.points[2].x = occludeMinX;
	crop_marker.points[2].y = occludeMinY;
	crop_marker.points[2].z = occludeMinZ;
	crop_marker.points[3].x = occludeMinX;
	crop_marker.points[3].y = occludeMinY;
	crop_marker.points[3].z = occludeMaxZ;
	crop_marker.points[4].x = occludeMinX;
	crop_marker.points[4].y = occludeMinY;
	crop_marker.points[4].z = occludeMinZ;
	crop_marker.points[5].x = occludeMaxX;
	crop_marker.points[5].y = occludeMinY;
	crop_marker.points[5].z = occludeMinZ;
	crop_marker.points[6].x = occludeMaxX;
	crop_marker.points[6].y = occludeMinY;
	crop_marker.points[6].z = occludeMinZ;
	crop_marker.points[7].x = occludeMaxX;
	crop_marker.points[7].y = occludeMaxY;
	crop_marker.points[7].z = occludeMinZ;
	crop_marker.points[8].x = occludeMaxX;
	crop_marker.points[8].y = occludeMinY;
	crop_marker.points[8].z = occludeMinZ;
	crop_marker.points[9].x = occludeMaxX;
	crop_marker.points[9].y = occludeMinY;
	crop_marker.points[9].z = occludeMaxZ;

	crop_marker.points[10].x = occludeMaxX;
	crop_marker.points[10].y = occludeMaxY;
	crop_marker.points[10].z = occludeMaxZ;
	crop_marker.points[11].x = occludeMaxX;
	crop_marker.points[11].y = occludeMaxY;
	crop_marker.points[11].z = occludeMinZ;
	crop_marker.points[12].x = occludeMaxX;
	crop_marker.points[12].y = occludeMaxY;
	crop_marker.points[12].z = occludeMaxZ;
	crop_marker.points[13].x = occludeMaxX;
	crop_marker.points[13].y = occludeMinY;
	crop_marker.points[13].z = occludeMaxZ;
	crop_marker.points[14].x = occludeMaxX;
	crop_marker.points[14].y = occludeMaxY;
	crop_marker.points[14].z = occludeMaxZ;
	crop_marker.points[15].x = occludeMinX;
	crop_marker.points[15].y = occludeMaxY;
	crop_marker.points[15].z = occludeMaxZ;
	crop_marker.points[16].x = occludeMinX;
	crop_marker.points[16].y = occludeMaxY;
	crop_marker.points[16].z = occludeMaxZ;
	crop_marker.points[17].x = occludeMinX;
	crop_marker.points[17].y = occludeMaxY;
	crop_marker.points[17].z = occludeMinZ;
	crop_marker.points[18].x = occludeMinX;
	crop_marker.points[18].y = occludeMaxY;
	crop_marker.points[18].z = occludeMaxZ;
	crop_marker.points[19].x = occludeMaxX;
	crop_marker.points[19].y = occludeMaxY;
	crop_marker.points[19].z = occludeMaxZ;

	crop_marker.points[20].x = occludeMinX;
	crop_marker.points[20].y = occludeMaxY;
	crop_marker.points[20].z = occludeMinZ;
	crop_marker.points[21].x = occludeMaxX;
	crop_marker.points[21].y = occludeMaxY;
	crop_marker.points[21].z = occludeMinZ;
	crop_marker.points[22].x = occludeMinX;
	crop_marker.points[22].y = occludeMinY;
	crop_marker.points[22].z = occludeMaxZ;
	crop_marker.points[23].x = occludeMaxX;
	crop_marker.points[23].y = occludeMinY;
	crop_marker.points[23].z = occludeMaxZ;

	crop_marker.points[30].x = cropMinX;
	crop_marker.points[30].y = cropMinY;
	crop_marker.points[30].z = cropMinZ;
	crop_marker.points[31].x = cropMinX;
	crop_marker.points[31].y = cropMaxY;
	crop_marker.points[31].z = cropMinZ;
	crop_marker.points[32].x = cropMinX;
	crop_marker.points[32].y = cropMinY;
	crop_marker.points[32].z = cropMinZ;
	crop_marker.points[33].x = cropMinX;
	crop_marker.points[33].y = cropMinY;
	crop_marker.points[33].z = cropMaxZ;
	crop_marker.points[34].x = cropMinX;
	crop_marker.points[34].y = cropMinY;
	crop_marker.points[34].z = cropMinZ;
	crop_marker.points[35].x = cropMaxX;
	crop_marker.points[35].y = cropMinY;
	crop_marker.points[35].z = cropMinZ;
	crop_marker.points[36].x = cropMaxX;
	crop_marker.points[36].y = cropMinY;
	crop_marker.points[36].z = cropMinZ;
	crop_marker.points[37].x = cropMaxX;
	crop_marker.points[37].y = cropMaxY;
	crop_marker.points[37].z = cropMinZ;
	crop_marker.points[38].x = cropMaxX;
	crop_marker.points[38].y = cropMinY;
	crop_marker.points[38].z = cropMinZ;
	crop_marker.points[39].x = cropMaxX;
	crop_marker.points[39].y = cropMinY;
	crop_marker.points[39].z = cropMaxZ;

	crop_marker.points[40].x = cropMaxX;
	crop_marker.points[40].y = cropMaxY;
	crop_marker.points[40].z = cropMaxZ;
	crop_marker.points[41].x = cropMaxX;
	crop_marker.points[41].y = cropMaxY;
	crop_marker.points[41].z = cropMinZ;
	crop_marker.points[42].x = cropMaxX;
	crop_marker.points[42].y = cropMaxY;
	crop_marker.points[42].z = cropMaxZ;
	crop_marker.points[43].x = cropMaxX;
	crop_marker.points[43].y = cropMinY;
	crop_marker.points[43].z = cropMaxZ;
	crop_marker.points[44].x = cropMaxX;
	crop_marker.points[44].y = cropMaxY;
	crop_marker.points[44].z = cropMaxZ;
	crop_marker.points[45].x = cropMinX;
	crop_marker.points[45].y = cropMaxY;
	crop_marker.points[45].z = cropMaxZ;
	crop_marker.points[46].x = cropMinX;
	crop_marker.points[46].y = cropMaxY;
	crop_marker.points[46].z = cropMaxZ;
	crop_marker.points[47].x = cropMinX;
	crop_marker.points[47].y = cropMaxY;
	crop_marker.points[47].z = cropMinZ;
	crop_marker.points[28].x = cropMinX;
	crop_marker.points[28].y = cropMaxY;
	crop_marker.points[28].z = cropMaxZ;
	crop_marker.points[29].x = cropMaxX;
	crop_marker.points[29].y = cropMaxY;
	crop_marker.points[29].z = cropMaxZ;

	crop_marker.points[24].x = cropMinX;
	crop_marker.points[24].y = cropMaxY;
	crop_marker.points[24].z = cropMinZ;
	crop_marker.points[25].x = cropMaxX;
	crop_marker.points[25].y = cropMaxY;
	crop_marker.points[25].z = cropMinZ;
	crop_marker.points[26].x = cropMinX;
	crop_marker.points[26].y = cropMinY;
	crop_marker.points[26].z = cropMaxZ;
	crop_marker.points[27].x = cropMaxX;
	crop_marker.points[27].y = cropMinY;
	crop_marker.points[27].z = cropMaxZ;

	crop_marker.points[48].x = cropMinX;
	crop_marker.points[48].y = cropMaxY;
	crop_marker.points[48].z = cropMaxZ;
	crop_marker.points[49].x = cropMinX;
	crop_marker.points[49].y = cropMinY;
	crop_marker.points[49].z = cropMaxZ;
	crop_marker.points[50].x = occludeMinX;
	crop_marker.points[50].y = occludeMinY;
	crop_marker.points[50].z = occludeMaxZ;
	crop_marker.points[51].x = occludeMinX;
	crop_marker.points[51].y = occludeMaxY;
	crop_marker.points[51].z = occludeMaxZ;

	crop_pub.publish(crop_marker);
	ros::Duration(0.5).sleep();
	ros::spinOnce();

}
//Callback for storing current powercube position
void QNode::jointCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	joint1 = msg->position[0];
}
//Store the point cloud selection in current_pc_
void QNode::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud)
{
	current_pc_.reset(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::fromROSMsg(*cloud, *this->current_pc_);


}

//Initialization function
bool QNode::init() {
	ROS_INFO( "Control Panel initializing");
	//The ros init function requires argc and argv, but we dont have any so we make empty ones
	int fargc = 0;
	char** fargv = (char**)malloc(sizeof(char*)*(fargc+1));
	ros::init(fargc,fargv,"ControlPanelNode");
	free(fargv);
	ros::start();
	//Our publishers for the powercube and manipulator
	cmd_pub = nh_.advertise<phd::cube_msg>("joint_cmd", 100);
	arm_pub = nh_.advertise<phd::arm_msg>("arm_cmd", 100);
	//Point cloud publisher
	pub = nh_.advertise<sensor_msgs::PointCloud2> ("assembled_cloud", 1);
	finished_scan_pub = nh_.advertise<std_msgs::String> ("finished_scan", 1);
	rawpub = nh_.advertise<sensor_msgs::PointCloud2> ("raw_cloud", 1);
	marker_pub = nh_.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
	path_pub = nh_.advertise<visualization_msgs::Marker>( "path_marker", 0 );
	dir_pub = nh_.advertise<visualization_msgs::Marker>( "dir_marker", 0 );
	arm_pose_pub = nh_.advertise<visualization_msgs::Marker>( "pose_marker", 0 );
	crop_pub = nh_.advertise<visualization_msgs::Marker>( "crop_marker", 0 );
	//Publisher for displaying navigation goal
	nav_vis_pub = nh_.advertise<geometry_msgs::PoseArray>( "nav_vis_goal", 0 );\
	reset_map_pub = nh_.advertise<std_msgs::String> ("syscommand", 1);
	reset_map_srv = nh_.serviceClient<phd::empty>("move_base/clear_costmaps");
	//Subscribers to listen to the powercube position and point cloud selection
	joint_sub = nh_.subscribe("cube_joint_states", 1, &control_panel::control_panel_ns::QNode::jointCallback,this);
	cloud_sub = nh_.subscribe("marker_selected_points", 1, &control_panel::control_panel_ns::QNode::cloudCallback,this);
	callback_type = boost::bind(&reconfig_callback, _1, _2);
	server.setCallback(callback_type);
	//Service client for calling the laser scan assembler
	client = nh_.serviceClient<laser_assembler::AssembleScans2>("assemble_scans2");
	//Service client for transforming th point cloud to the world coordinate frame
	loc_client = nh_.serviceClient<phd::localize_cloud>("localize_pcd");
	//Service client for gerneating an arm trajectory
	traj_client = nh_.serviceClient<phd::simple_trajectory_service>("trajectory_gen");
	//Service client for estimating thickness
	thick_client = nh_.serviceClient<phd::thickness_service>("thick_srv");
	//Initialize the trajectory display marker	
	point_list.header.frame_id = "/base_footprint";
	point_list.header.stamp = ros::Time::now();
	point_list.ns = "points_and_lines";
	point_list.action = visualization_msgs::Marker::ADD;
	point_list.pose.orientation.w = 1.0;
	point_list.id = 0;
	point_list.type = visualization_msgs::Marker::POINTS;
	point_list.scale.x = 0.1;
	point_list.scale.y = 0.1;
	point_list.color.r = 1.0;
	point_list.color.a = 1;
	//Start the counters for trajectory sections and points at 0
	cloud_ctr = 0;
	tctr = 0;
	current_pc_.reset(new pcl::PointCloud<pcl::PointXYZI>());
	raw_pc_.reset(new pcl::PointCloud<pcl::PointXYZI>());

	//Visualization of robot workspace
	ros::Publisher ws_pub; 
	ws_pub = nh_.advertise<visualization_msgs::Marker>( "ws_points_marker", 0 );
	visualization_msgs::Marker ws_points;
	ws_points.header.frame_id = "/base_footprint";
	ws_points.header.stamp = ros::Time::now();
	ws_points.ns = "basic_shapes";
	ws_points.id = 0;
	ws_points.type = visualization_msgs::Marker::SPHERE;
	ws_points.action = visualization_msgs::Marker::ADD;
	ws_points.pose.position.x = X_BASE_TO_ARM;
	ws_points.pose.position.y = Y_BASE_TO_ARM;
	ws_points.pose.position.z = Z_BASE_TO_ARM+.28;
	ws_points.pose.orientation.x = 0.0;
	ws_points.pose.orientation.y = 0.0;
	ws_points.pose.orientation.z = 0.0;
	ws_points.pose.orientation.w = 1.0;
	ws_points.scale.x = .8;
	ws_points.scale.y = .8;
	ws_points.scale.z = .8;
	ws_points.color.r = 1.0f;
	ws_points.color.g = 1.0f;
	ws_points.color.b = 1.0f;
	ws_points.color.a = 0.3;
	ws_points.lifetime = ros::Duration();
	ws_pub.publish(ws_points);
	ros::Duration(0.5).sleep();
	ros::spinOnce();
	pose_waiting = false;
	//Reset Map
	if(DEBUG) ROS_INFO("Resetting Map");
	std::stringstream ss, ss1, ss2, ss3;
	ss << "reset";
	ss1 << "CLOUD";
	ss2 << "TRAJ";
	ss3 << "SCAN_COMPLETE";
	phd::empty esrv;
	reset_map_srv.call(esrv);
	RESET_MAP.data = ss.str();
	CLOUD_STR.data = ss1.str();
	TRAJ.data = ss2.str();
	SCAN_COMPLETE.data = ss3.str();
	transform.setOrigin( tf::Vector3(0, 0, 0) );
	q.setRPY(0, 0, 0);
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "map"));
	ros::spinOnce();


}

//Soft stop for the arm and base
void QNode::soft_stop(void){

	//Send the arm to its home location
	phd::arm_msg msg;
	msg.j1 = 0;
	msg.j2 = 0;
	msg.j3 = 90;
	msg.j4 = 0;
	msg.j5 = 0;
	msg.j6 = 0;
	msg.motion_type = JOINT;
	arm_pub.publish(msg);
	//Cancel all move_base goals
	MoveBaseClient.cancelAllGoals();
	ros::spinOnce();

}

//Set the arm speed
void QNode::set_speed(float speed){

	phd::arm_msg msg;
	msg.vel = speed;
	msg.motion_type = SPEED;
	arm_pub.publish(msg);
	ros::spinOnce();

}
//Helper function for deleting the nearest point to "target_pt" from cloud "line"
bool delete_point(pcl::PointCloud<pcl::PointXYZI>::Ptr line, pcl::PointXYZI target_pt){
	float resolution = 128.0f; //Octree resolution
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> octree (resolution);
	int K = 1; //Number of points to find
	std::vector<int> pointIdxNKNSearch; //vector to hold points (used by pcl)
	std::vector<float> pointNKNSquaredDistance; //vector to hold distance of points (used by pcl)
	//Initialize the octree
	octree.setInputCloud (line);
	octree.addPointsFromInputCloud ();
	//find the nearest point in "line" to "target_pt"
	if (octree.nearestKSearch (target_pt, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0){
		//Delete the point only if its close enough to be considered part of a "chunk"
		if(pointNKNSquaredDistance[0] < CHUNK_RADIUS){
			line->points.erase(line->points.begin()+pointIdxNKNSearch[0]);
			return true;
		}else {
			return false;
		}
	}else return false;
	//return true if a point was deleted, false if it could not find a point nearby of all the points within "CHUNK_RADIUS"
}
//Deletes a chunk of the cloud "line" near point "target_pt"
void eat_chunk(pcl::PointCloud<pcl::PointXYZI>::Ptr line, pcl::PointXYZI target_pt){
	//Keep deleting points using the delete_point helper function until there are no points left in "line" or the function returns false
	while(delete_point(line, target_pt) && line->points.size() > 0);
}
//Find a point nearest to "point_holder" and delete all the points within CHUNK_RADIUS
bool find_and_delete_NN(pcl::PointXYZI point_holder, pcl::PointCloud<pcl::PointXYZI>::Ptr line, pcl::PointXYZI* ret_pt){

	int K = 1; //Number of points to find
	std::vector<int> pointIdxNKNSearch; //vector to hold points (used by pcl)
	std::vector<float> pointNKNSquaredDistance; //vector to hold distance of points (used by pcl)
	pcl::PointXYZI searchPoint; //Location of where to search
	float resolution = 128.0f; //Octree resolution
	if(line->points.size() > 0){ //make sure there are actually points in the line
		//Create the octree object
		pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> octree (resolution);
		//Set the octree input cloud, then add the points
		octree.setInputCloud (line);
		octree.addPointsFromInputCloud ();
		//Search for the nearest 1 neighbour
		if(std::isfinite(point_holder.x) && std::isfinite(point_holder.y) && std::isfinite(point_holder.z)){//make sure the point exists
			if (octree.nearestKSearch (point_holder, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0){
				*ret_pt = line->points[pointIdxNKNSearch[0]];
				//delete a chunk of points near the found point
				eat_chunk(line,*ret_pt);
				//return true if a point was found and chunk deleted
				return true;	
			}else{
				//point was not found nearby
				return false;
			}
		}else return false; //empty search point
	}else return false; //empty input cloud
}
//calculates the distance between two points
float vec_length(pcl::PointXYZI pointA,pcl::PointXYZI pointB){
	return sqrt(pow(pointB.x-pointA.x,2)+pow(pointB.y-pointA.y,2)+pow(pointB.z-pointA.z,2));
}

//Display the current naviagtion goals
void QNode::show_nav() {

	goals.header.frame_id = "/base_footprint";
	goals.header.stamp = ros::Time::now();
	nav_vis_pub.publish(goals);

}

//Main control loop, spins while ros is running
void QNode::run() {
	std::cout << "Control Panel Running" << std::endl;
	while ( ros::ok() ) {
		ros::spinOnce();
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}
//Helper function for performing dot products
tfScalar dot_p(float v1[3], float v2[3]){

	tfScalar ret = v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2];
	return ret;

}

//Set the required roll, pitch, and yaw for a given trajectory point orientation
void calc_RPY(phd::trajectory_point t_point, phd::arm_msg* msg){

	float bX[3], bY[3], bZ[3], temp[3];
	float aX[3], aY[3], aZ[3];
	aX[0] = 1;
	aX[1] = 0;
	aX[2] = 0;
	aY[0] = 0;
	aY[1] = 1;
	aY[2] = 0;
	aZ[0] = 0;
	aZ[1] = 0;
	aZ[2] = 1;
	bZ[0] = -t_point.nx;
	bZ[1] = t_point.ny;
	bZ[2] = t_point.nz;
	float length = sqrt(pow(bZ[0],2)+pow(bZ[1],2)+pow(bZ[2],2));
	//Convert to unit vector
	bZ[0] = bZ[0]/length;
	bZ[1] = bZ[1]/length;
	bZ[2] = bZ[2]/length;
	if(bZ[0] != 1){
		temp[0] = 0;
		temp[1] = bZ[2];
		temp[2]  = -bZ[1];
	}else{	
		temp[0] = -bZ[2];
		temp[1] = 0;
		temp[2]  = bZ[0];
	}
	bY[0] = bZ[1]*temp[2]-temp[1]*bZ[2];
	bY[1] = bZ[2]*temp[0]-temp[2]*bZ[0];
	bY[2]  = bZ[0]*temp[1]-temp[0]*bZ[1];
	length = sqrt(pow(bY[0],2)+pow(bY[1],2)+pow(bY[2],2));
	//Convert to unit vector
	bY[0] = bY[0]/length;
	bY[1] = bY[1]/length;
	bY[2] = bY[2]/length;
	bX[0] = bZ[1]*bY[2]-bY[1]*bZ[2];
	bX[1] = bZ[2]*bY[0]-bY[2]*bZ[0];
	bX[2]  = bZ[0]*bY[1]-bY[0]*bZ[1];
	length = sqrt(pow(bX[0],2)+pow(bX[1],2)+pow(bX[2],2));
	//Convert to unit vector
	bX[0] = bX[0]/length;
	bX[1] = bX[1]/length;
	bX[2] = bX[2]/length;
	tfScalar yaw, pitch, roll;
	length = sqrt(pow(bZ[0],2)+pow(bZ[1],2)+pow(bZ[2],2));
	tf::Matrix3x3 mat(dot_p(bX,aX),dot_p(bY,aX),dot_p(bZ,aX),dot_p(bX,aY),dot_p(bY,aY),dot_p(bZ,aY),dot_p(bX,aZ),dot_p(bY,aZ),dot_p(bZ,aZ));
	mat.getRPY(roll,pitch,yaw);
	msg->rx = roll;
	msg->ry = pitch;
	msg->rz = yaw;
}
//There is a pie slice shaped area that is not part of the workspace, this function checks if a point is within it
bool check_pie(float x, float y){
	float angle = atan2(y,x);
	if(angle > 2.79 || angle < -2.79){
		ROS_INFO("Pie");
		return true;
	}else return false;
}

//Helper function for when a trajectory point is outside the robot's workspace, returns an arm message
phd::arm_msg move_to_workspace(phd::trajectory_point t_point){

	phd::arm_msg msg;
	//calculate the distance from the wall point to the workspace centre
	float vec_len = sqrt(pow(t_point.x-X_BASE_TO_ARM,2)+pow(t_point.y-Y_BASE_TO_ARM,2)+pow(t_point.z-(Z_BASE_TO_ARM+.28),2));
	//set the normal for the trajectory point to aim the end effector at the wall target point
	t_point.nx = (t_point.x - X_BASE_TO_ARM) / vec_len;
	t_point.ny = (t_point.y - Y_BASE_TO_ARM) / vec_len;
	t_point.nz = (t_point.z - (Z_BASE_TO_ARM + .28)) / vec_len;
	//create the arm trajectory point near the limits of the robot workspace (.4m) then convert to mm
	msg.x = (.375*(t_point.nx))*1000;
	msg.y = (.375*(t_point.ny))*1000;
	msg.z = 280 + (.375*(t_point.nz))*1000;
	if(check_pie(msg.x,msg.y)){
		msg.x = -250;
		msg.y = 130*(msg.y/fabs(msg.y));
	}
	calc_RPY(t_point,&msg);
	float f1, f2, f3 ,f4;
	f1 = atan2(-t_point.nz,-t_point.ny)* (180.0/3.141592653589793238463);
	f2 = atan2(t_point.nz,-t_point.ny)* (180.0/3.141592653589793238463);
	f3 = atan2(-t_point.nz,t_point.ny)* (180.0/3.141592653589793238463);
	f4 = atan2(t_point.nz,t_point.ny)* (180.0/3.141592653589793238463);
	float d1, d2, d3 ,d4;
	d1 = atan2(-t_point.nz,-t_point.nx)* (180.0/3.141592653589793238463);
	d2 = atan2(t_point.nz,-t_point.nx)* (180.0/3.141592653589793238463);
	d3 = atan2(-t_point.nz,t_point.nx)* (180.0/3.141592653589793238463);
	d4 = atan2(t_point.nz,t_point.nx)* (180.0/3.141592653589793238463);
	float x1, x2, x3 ,x4;
	x1 = atan2(-t_point.ny,-t_point.nx)* (180.0/3.141592653589793238463);
	x2 = atan2(t_point.ny,-t_point.nx)* (180.0/3.141592653589793238463);
	x3 = atan2(-t_point.ny,t_point.nx)* (180.0/3.141592653589793238463);
	x4 = atan2(t_point.ny,t_point.nx)* (180.0/3.141592653589793238463);
	return msg;
}
//Function used to step through the trajectory points, traj_ctr tells the arm which trajectory point to go to and sim says whether to simulate movement or actually move the arm
phd::trajectory_point QNode::step(int traj_ctr,bool sim, float fig){
	//Generate and arm_msg and populate it with the current point in the trajectory array
	phd::arm_msg msg;
	phd::trajectory_point t_msg;
	geometry_msgs::Point p;	
	float P_OFFSET;
	ros::param::get("/global_offset", P_OFFSET);
	if(traj_ctr == 0) armpoint_list.points.clear(); //clear our marker array if we're starting fresh
	//create the arm message based on the surface points and normals and the offset (can be set using dynmic reconfigure)
	msg.x = (traj.points[traj_ctr].x-(traj.points[traj_ctr].nx*P_OFFSET) - X_BASE_TO_ARM)*1000;
	msg.y = (traj.points[traj_ctr].y-(traj.points[traj_ctr].ny*P_OFFSET) - Y_BASE_TO_ARM)*1000;
	msg.z = (traj.points[traj_ctr].z-(traj.points[traj_ctr].nz*P_OFFSET) - Z_BASE_TO_ARM)*1000;
	//check to see if the arm point is within the arms workspace
	float ws_dist = sqrt(pow(msg.x,2)+pow(msg.y,2)+pow(msg.z-280,2));
	if(ws_dist > 375 || check_pie(msg.x,msg.y)){
		if(DEBUG) ROS_INFO("Modifying %d, %f",traj_ctr,ws_dist);
		//if outside workspace, move within
		msg = move_to_workspace(traj.points[traj_ctr]);
		//marker to show arm goal
		p.x = msg.x/1000 + X_BASE_TO_ARM;
		p.y = msg.y/1000 + Y_BASE_TO_ARM;
		p.z = msg.z/1000 + Z_BASE_TO_ARM;
		armpoint_list.points.push_back(p);
		//marker to show surface points
		p.x = traj.points[traj_ctr].x;
		p.y = traj.points[traj_ctr].y;
		p.z = traj.points[traj_ctr].z;
		armpoint_list.points.push_back(p);
	}else{
		//Caluclate the rotation around the X and Y axis from the normal vector
		float length = sqrt(pow(traj.points[traj_ctr].nx,2)+pow(traj.points[traj_ctr].ny,2)+pow(traj.points[traj_ctr].nz,2));
		msg.rx = asin(traj.points[traj_ctr].ny / length);
		//Prevent divide by zero errors
		if(cos(msg.rx)!=0) msg.ry = asin( traj.points[traj_ctr].nx / (cos(msg.rx)*length) );
		else msg.ry = 0;
		//No need for the roll around the Z axis, since the spray pattern and radiation detection is conical
		msg.rz = 0;
		//marker to show arm goal
		p.x = msg.x/1000 + X_BASE_TO_ARM;
		p.y = msg.y/1000 + Y_BASE_TO_ARM;
		p.z = msg.z/1000 + Z_BASE_TO_ARM;
		armpoint_list.points.push_back(p);
		//marker to show surface points
		p.x = msg.x/1000 + X_BASE_TO_ARM + P_OFFSET*traj.points[traj_ctr].nx;
		p.y = msg.y/1000 + Y_BASE_TO_ARM + P_OFFSET*traj.points[traj_ctr].ny;
		p.z = msg.z/1000 + Z_BASE_TO_ARM + P_OFFSET*traj.points[traj_ctr].nz;
		armpoint_list.points.push_back(p);

	}
	//This is the elbow configuration of the arm
	msg.motion_type = PTP;
	msg.rx =  (msg.rx* (180.0/3.141592653589793238463));
	msg.ry =  (msg.ry* (180.0/3.141592653589793238463));
	msg.rz = (msg.rz* (180.0/3.141592653589793238463));
	ROS_INFO("ARM POSE %f %f %f %f %f %f",msg.x,msg.y,msg.z,msg.rx,msg.ry,msg.rz);
	t_msg.x = msg.x;
	t_msg.y = msg.y;
	t_msg.z = msg.z;
	//only send the arm command if sim is not true
	if(!sim){
		//Send the command to the arm
		arm_pub.publish(msg);
		ros::spinOnce();
	}else ROS_INFO("Marker Published");	

	//Initialize and display the trajectory display marker	
	armpoint_list.header.frame_id = "/base_footprint";
	armpoint_list.header.stamp = ros::Time::now();
	armpoint_list.ns = "points_and_lines";
	armpoint_list.action = visualization_msgs::Marker::ADD;
	armpoint_list.pose.position.x = 0.0;
	armpoint_list.pose.position.y = 0.0;
	armpoint_list.pose.position.z = 0.0;
	armpoint_list.pose.orientation.x = 0.0;
	armpoint_list.pose.orientation.y = 0.0;
	armpoint_list.pose.orientation.z = 0.0;
	armpoint_list.pose.orientation.w = 1.0;
	armpoint_list.id = 0;
	armpoint_list.type = visualization_msgs::Marker::LINE_LIST;
	armpoint_list.scale.x = 0.01;
	armpoint_list.scale.y = 0.01;
	armpoint_list.scale.z = 0.01;
	armpoint_list.color.r = 1.0;
	armpoint_list.color.b = 1.0;
	armpoint_list.color.g = 1.0;
	armpoint_list.color.a = 1;
	arm_pose_pub.publish(armpoint_list);
	ros::spinOnce();

	return t_msg;
}
//Calculates the thickness between the two clouds previously set, saving the result to "filename"
void QNode::calculate_thickness(std::string filename){
	phd::thickness_service thick_srv;
	//Call the localization service
	thick_srv.request.cloud_1 = clean_cloud;
	thick_srv.request.cloud_2 = sprayed_cloud;
	if(thick_client.call(thick_srv)){
		if(DEBUG) ROS_INFO("Thick Cloud %d", (uint32_t)(thick_srv.response.cloud_out.width));
		pub.publish(thick_srv.response.cloud_out);
		pcl::PointCloud<pcl::PointXYZI>::Ptr save_cloud (new pcl::PointCloud<pcl::PointXYZI>);
		std::stringstream pf;
		pcl::fromROSMsg(thick_srv.response.cloud_out, *save_cloud);
		pf << filename << "thickened.pcd";
	if(DEBUG) ROS_INFO("Saving to %s",pf.str().c_str());
	if(save_cloud->size() > 0) pcl::io::savePCDFileASCII (pf.str().c_str(), *save_cloud);
	}else ROS_INFO("Service Failed");

}
//Set the initial point cloud for thickness estimates
void QNode::set_clean(){
	//Convert from PCL to ROS
	pcl::toROSMsg(*current_pc_,clean_cloud);
}
//Set the final point cloud for thickness estimates
void QNode::set_sprayed(){
	//Convert from PCL to ROS
	pcl::toROSMsg(*current_pc_,sprayed_cloud);
}

//Perform a laser scan, either localize the cloud or set it as the global frame, and save to disk
void QNode::scan(std::string markername, bool localize){
	//move the arm out of the way
	QNode::send_joint_command(-90,45,90,0,0,0);
	//Create a message to send to the powercube node, moving the powercube to its start location
	phd::cube_msg cube_cmd;
	cube_cmd.j1 = -2.2;
	cube_cmd.vel = 0.5;
	cube_cmd.acc = 2;
	//pose=true means we're sending a position. False means its a velocity and should eventually time out
	cube_cmd.pose = true;
	//Publish and spin
	cmd_pub.publish(cube_cmd);
	ros::spinOnce();
	if(DEBUG) {
		printf("j1: %f\n",joint1);
		ROS_INFO("going to start");
	}	
	//Wait for the cube to reach the start location
	while(joint1 > -2.19 && ros::ok()){
	ros::spinOnce();
	} 
	//Set the start time for the laser assembler service	
	srv.request.begin = ros::Time::now();
	//Tell the cube to go to the end location
	cube_cmd.j1 = 1;
	cube_cmd.vel = 0.25;
	cmd_pub.publish(cube_cmd);
	ros::spinOnce();
	if(DEBUG) ROS_INFO("waiting for end");
	//Wait for cube to reach end location
	while(joint1 < .99 && ros::ok()){
		ros::spinOnce();
	} 
	if(DEBUG) ROS_INFO("done");
	//Set end time for laser assembler
	srv.request.end   = ros::Time::now();
	//Spin to send the end time to the service before calling it
	ros::spinOnce();

	// Make the service call
	if (client.call(srv)){
		ROS_INFO("Assembled Cloud %d", (uint32_t)(srv.response.cloud.width)) ;
		//Save the new scan as cloud_surface		
		cloud_surface_raw = srv.response.cloud;
		//publish the raw cloud
		cloud_surface_raw.header.stamp = ros::Time::now();
		cloud_surface_raw.header.frame_id = "/base_footprint";
		rawpub.publish(cloud_surface_raw);
		//Save the pointcloud to disk using markername CLOUD and the counter
		pcl::PointCloud<pcl::PointXYZI>::Ptr save_cloud (new pcl::PointCloud<pcl::PointXYZI>);
		std::stringstream pf;
		//Tell pcl what the 4th field information is
		cloud_surface_raw.fields[3].name = "intensity";
		ROS_INFO("Converting");
		pcl::fromROSMsg(cloud_surface_raw, *save_cloud);
		cloud_ctr++;
		pf << markername << cloud_ctr << "raw" << CLOUD << ".pcd";
		if(DEBUG) ROS_INFO("Saving to %s",pf.str().c_str());
		if(save_cloud->size() > 0) pcl::io::savePCDFileASCII (pf.str().c_str(), *save_cloud);
		//Tell ros what the 4th field information is
		cloud_surface_raw.fields[3].name = "intensities";
		//If set home box is checked, tell localization to save marker location to the filename specified, otheriwse load the file and localize
		//Crop points that are the robot itself
		Eigen::Vector4f minPoint; 
		minPoint[0]=occludeMinX;//minimum point x 
		minPoint[1]=occludeMinY;//minimum point y 
		minPoint[2]=occludeMinZ;//minimum point z 
		Eigen::Vector4f maxPoint; 
		maxPoint[0]=occludeMaxX;//max point x 
		maxPoint[1]=occludeMaxY;//max point y 
		maxPoint[2]=occludeMaxZ;//max point z 
		Eigen::Vector3f boxTranslatation; 
		boxTranslatation[0]=0;   
		boxTranslatation[1]=0;   
		boxTranslatation[2]=0;   
		Eigen::Vector3f boxRotation; 
		boxRotation[0]=0;  // rotation around x-axis 
		boxRotation[1]=0;  // rotation around y-axis 
		boxRotation[2]=0;  //in radians rotation around z-axis. this rotates your cube 45deg around z-axis. 
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOut (new pcl::PointCloud<pcl::PointXYZI>); 
		pcl::CropBox<pcl::PointXYZI> cropFilter; 

		cropFilter.setInputCloud (save_cloud); 
		cropFilter.setMin(minPoint); 
		cropFilter.setMax(maxPoint);
		cropFilter.setTranslation(boxTranslatation); 
		cropFilter.setRotation(boxRotation); 
		cropFilter.setNegative(true);
		cropFilter.filter (*cloudOut); 
		//Set the pointcloud to cover
		pcl::toROSMsg(*cloudOut,cloud_surface_world);
		raw_pc_ = cloudOut;
		if(DEBUG&&localize) ROS_INFO("Setting Home");	
		//Call the localization service, it will transform the cloud if the set home box is not checked
		loc_srv.request.cloud_in = cloud_surface_world;//raw;
		loc_srv.request.homing = localize;
		loc_srv.request.marker_file = markername;
		if(loc_client.call(loc_srv)){
			if(DEBUG) ROS_INFO("Localize Service Cloud %d", (uint32_t)(loc_srv.response.cloud_out.width));
			cloud_surface_world = loc_srv.response.cloud_out;
			
			pcl::PointCloud<pcl::PointXYZI>::Ptr full_cloud (new pcl::PointCloud<pcl::PointXYZI>);
			//Tell pcl what the 4th field information is
			cloud_surface_world.fields[3].name = "intensity";
			pcl::fromROSMsg(cloud_surface_world, *full_cloud);
			pf.str("");			
			if(localize){
				cloud_surface_world.header.frame_id = "/world";
				pf << markername << cloud_ctr << "home" << CLOUD << ".pcd";
				transform.setOrigin( tf::Vector3(0, 0, 0) );
				q.setRPY(0, 0, 0);
				transform.setRotation(q);
			}else{
				cloud_surface_world.header.frame_id = "/world";
				transform.setOrigin( tf::Vector3(loc_srv.response.transform_mat[3], loc_srv.response.transform_mat[7], loc_srv.response.transform_mat[11]) );
				tf::Matrix3x3 m(loc_srv.response.transform_mat[0], loc_srv.response.transform_mat[1], loc_srv.response.transform_mat[2], loc_srv.response.transform_mat[4], loc_srv.response.transform_mat[5], loc_srv.response.transform_mat[6], loc_srv.response.transform_mat[8], loc_srv.response.transform_mat[9], loc_srv.response.transform_mat[10]);
				m.getRotation(q);
				transform.setRotation(q);
				pf << markername << cloud_ctr << "localized" << CLOUD << ".pcd";
			}
			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base_footprint"));
			//fix the naming discrepancy between ROS and PCL (from "intensities" to "intensity")
			cloud_surface_world.fields[3].name = "intensities";
			sensor_msgs::PointCloud2 cloud_save;
			cloud_save = cloud_surface_world;
			cloud_save.fields[3].name = "intensity";
			save_cloud->clear();
			pcl::fromROSMsg(cloud_save, *save_cloud);
			if(save_cloud->size() > 0) pcl::io::savePCDFileASCII (pf.str().c_str(), *save_cloud);
			cloud_surface_world.header.stamp = cloud_surface_raw.header.stamp;
			pub.publish(cloud_surface_world);
		}else{
			ROS_INFO("Localize Service Failed");
		}

	}
	else ROS_ERROR("Error making service call to laser assembler\n") ;
	cube_cmd.j1 = -1.57;
	cube_cmd.vel = 0.75;
	cmd_pub.publish(cube_cmd);
	while(joint1 > -1.56 && ros::ok()){
		ros::spinOnce();
	}
	phd::empty esrv;
	reset_map_srv.call(esrv);
	reset_map_pub.publish(RESET_MAP);
	finished_scan_pub.publish(CLOUD_STR);
	ros::spinOnce();
	finished_scan_pub.publish(SCAN_COMPLETE);
	transform.setOrigin( tf::Vector3(0, 0, 0) );
	q.setRPY(0, 0, 0);
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "map"));
}

//Move the cube to a navigation orientation (horizontal) or other specified position
void QNode::nav_mode(float pos){
	//Create the message for the cube node
	phd::cube_msg cube_cmd;	
	cube_cmd.j1 = pos;
	cube_cmd.vel = 0.5;
	cube_cmd.acc = 2;
	//pose=true means we're sending a position. False means its a velocity and should eventually time out
	cube_cmd.pose = true;
	//Publish the command and spin
	cmd_pub.publish(cube_cmd);
	ros::spinOnce();

}

//Testing function for publishing point clouds from file
void QNode::fscan(std::string filename, bool auto_localize,bool set_home, std::string markername, bool autocrop){

	//Create a pcl pointer to load the clound in to
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI> );
	//Create a ros message to publish the cloud as
	sensor_msgs::PointCloud2 cloud_msg;

	current_pc_.reset(new pcl::PointCloud<pcl::PointXYZI>());
	//Load the file
	if (pcl::io::loadPCDFile<pcl::PointXYZI> (filename.c_str(), *cloud) == -1) PCL_ERROR ("Couldn't read file\n");
	else{
		if(DEBUG) ROS_INFO("File Opened");
		//Convert from PCL to ROS
		pcl::toROSMsg(*cloud,cloud_msg);
		//fix the naming discrepancy between ROS and PCL (from "intensities" to "intensity")
		cloud_msg.fields[3].name = "intensity";
		//Set the frame
		cloud_msg.header.frame_id = "/base_footprint";
		if(auto_localize){
			if(DEBUG) ROS_INFO("Auto Localizing");	
			//Call the localization service
			loc_srv.request.cloud_in = cloud_msg;
			loc_srv.request.homing = set_home;
			loc_srv.request.marker_file = markername;
			if(loc_client.call(loc_srv)){
				ROS_INFO("Localized Cloud %d", (uint32_t)(loc_srv.response.cloud_out.width));
				pub.publish(loc_srv.response.cloud_out);
				cloud_surface_world = loc_srv.response.cloud_out;

				pcl::PointCloud<pcl::PointXYZI>::Ptr save_cloud (new pcl::PointCloud<pcl::PointXYZI>);
				std::stringstream pf;
				//Tell pcl what the 4th field information is
				cloud_surface_world.fields[3].name = "intensity";
				pcl::fromROSMsg(cloud_surface_world, *save_cloud);
				pf << filename << "localized.pcd";
				if(DEBUG) ROS_INFO("Saving to %s",pf.str().c_str());
				if(save_cloud->size() > 0) pcl::io::savePCDFileASCII (pf.str().c_str(), *save_cloud);
				//Open file for saving marker location
				ofstream myfile;
				std::stringstream fs;
				fs << "/home/mike/results/poses.csv";
				myfile.open (fs.str().c_str(), std::ios::out|std::ios::app);
				myfile << CLOUD << cloud_ctr << ",";
				for(int i = 0; i <16; ++i){
					if(i%4 == 0) myfile << std::endl;
					myfile << loc_srv.response.transform_mat[i] << ",";
				}
				myfile << std::endl;
			}else ROS_INFO("Service Failed");
		}
		if(autocrop){
			//Crop points that are the robot itself
			Eigen::Vector4f minPoint; 
			minPoint[0]=occludeMinX;  // define minimum point x 
			minPoint[1]=occludeMinY;  // define minimum point y 
			minPoint[2]=occludeMinZ;  // define minimum point z 
			Eigen::Vector4f maxPoint; 
			maxPoint[0]=occludeMaxX;  // define max point x 
			maxPoint[1]=occludeMaxY;  // define max point y 
			maxPoint[2]=occludeMaxZ;  // define max point z 
			Eigen::Vector3f boxTranslatation; 
			boxTranslatation[0]=0;   
			boxTranslatation[1]=0;   
			boxTranslatation[2]=0;   
			Eigen::Vector3f boxRotation; 
			boxRotation[0]=0;  // rotation around x-axis 
			boxRotation[1]=0;  // rotation around y-axis 
			boxRotation[2]=0;  //in radians rotation around z-axis. this rotates your cube 45deg around z-axis. 
			pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOut (new pcl::PointCloud<pcl::PointXYZI>); 
			pcl::CropBox<pcl::PointXYZI> cropFilter; 
			cropFilter.setInputCloud (cloud); 
			cropFilter.setMin(minPoint); 
			cropFilter.setMax(maxPoint);
			cropFilter.setTranslation(boxTranslatation); 
			cropFilter.setRotation(boxRotation); 
			cropFilter.setNegative(true);
			cropFilter.filter (*cloudOut); 
			//Set the pointcloud to cover
			pcl::toROSMsg(*cloudOut,cloud_surface_world);
			//current_pc_ = cloudOut;
		}else{
		cloud_surface_world = cloud_msg;
		cloud_surface_world.fields[3].name = "intensities";
		//current_pc_ = cloud;
		}
			cloud_surface_world.header.frame_id = "/world";
			rawpub.publish(cloud_surface_world);
	}
	

}
//This function saves the location of the IR reflective ball by clustering the highly reflective points in to a single location
//On the third call to this function (index == 2) it will save the tranformation matrix from the robot to the marker as a rosbag
void QNode::cluster(std::string filename, int index){
	//Calculate how many points are being clustered
	if(DEBUG){
		int init_size =  current_pc_->width * current_pc_->height;
		ROS_INFO("Clustering %d, %d points", index,init_size) ;
	}	
	//Create containers for the filtered cloud
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_intensity_filtered (new pcl::PointCloud<pcl::PointXYZI>);
	// Create the filtering object
	pcl::PassThrough<pcl::PointXYZI> pass (true);
	//filter the data for intensity
	pass.setInputCloud (current_pc_);
	pass.setFilterFieldName ("intensity");
	pass.setFilterLimits (VINT_MIN,VINT_MAX);
	pass.filter (*cloud_intensity_filtered);
	//initialize the variables to hold the center of the points
	float xs=0;
	float ys=0;
	float zs=0;
	//Calculate how many points of sufficient intensity were found
	int size = cloud_intensity_filtered->width * cloud_intensity_filtered->height;
	//average all the points
	for(int i = 0; i < size; ++i){
		xs += cloud_intensity_filtered->points[i].x;
		ys += cloud_intensity_filtered->points[i].y;
		zs += cloud_intensity_filtered->points[i].z;
	}
	xs = xs/size;
	ys = ys/size;
	zs = zs/size;
	if(DEBUG) ROS_INFO("Saving Point: %f - %f - %f / %d",xs, ys, zs, size);
	//Open file for saving marker location
	ofstream myfile;
	std::stringstream fs;
	fs << filename << "marker.dat";
	//Save either point 1, 2, or 3 to file
	switch (index){
		//point 1
		case 0:	myfile.open (fs.str().c_str(), std::ios::out|std::ios::trunc);
			P1x = xs;
			P1y = ys;
			P1z = zs;
			myfile << xs << std::endl << ys << std::endl << zs << std::endl;
			break;
		//point 2
		case 1: myfile.open (fs.str().c_str(), std::ios::out|std::ios::app);
			P2x = xs;
			P2y = ys;
			P2z = zs;
			myfile << xs << std::endl << ys << std::endl << zs << std::endl;
			break;
		//point 3 (final)
		case 2: myfile.open (fs.str().c_str(), std::ios::out|std::ios::app);
			myfile << xs << std::endl << ys << std::endl << zs << std::endl;
			//Calculate the transformation matrix from the origin to the centre of the marker points
			Eigen::Vector3f a,n,c,o;
			Eigen::Matrix4f A_mat;
			float mn = sqrt(pow(xs-P2x,2)+pow(ys-P2y,2)+pow(zs-P2z,2));
			float mc = sqrt(pow(P1x-P2x,2)+pow(P1y-P2y,2)+pow(P1z-P2z,2));
			n(0) = (xs-P2x)/mn;
			n(1) = (ys-P2y)/mn;
			n(2) = (zs-P2z)/mn;
			c(0) = (P1x-P2x)/mc;
			c(1) = (P1y-P2y)/mc;
			c(2) = (P1z-P2z)/mc;
			a(0) = n(1)*c(2)-n(2)*c(1);
			a(1) = n(2)*c(0)-n(0)*c(2);
			a(2) = n(0)*c(1)-n(1)*c(0);
			float m = sqrt(pow(a(0),2)+pow(a(1),2)+pow(a(2),2));
			a(0) = a(0)/m;
			a(1) = a(1)/m;
			a(2) = a(2)/m;	
			o(0) = a(1)*n(2)-a(2)*n(1);
			o(1) = a(2)*n(0)-a(0)*n(2);
			o(2) = a(0)*n(1)-a(1)*n(0);
			A_mat(0,0) = n(0);
			A_mat(0,1) = o(0);
			A_mat(0,2) = a(0);
			A_mat(0,3) = (P1x+P2x+xs)/3;
			A_mat(1,0) = n(1);
			A_mat(1,1) = o(1);
			A_mat(1,2) = a(1);
			A_mat(1,3) = (P1y+P2y+ys)/3;
			A_mat(2,0) = n(2);
			A_mat(2,1) = o(2);
			A_mat(2,2) = a(2);
			A_mat(2,3) = (P1z+P2z+zs)/3;
			A_mat(3,0) = 0;
			A_mat(3,1) = 0;
			A_mat(3,2) = 0;
			A_mat(3,3) = 1;
			//Calculate the distance between each point (VAL1-3) and the dot product of vector P1P2 and P3P2 (VAL4)
			Eigen::Vector3f vec1, vec2;
			float VAL1 = sqrt(pow(P1x - P2x,2)+pow(P1y - P2y,2)+pow(P1z - P2z,2));
			float VAL3 = sqrt(pow(P1x - xs,2)+pow(P1y - ys,2)+pow(P1z - zs,2));
			float VAL2 = sqrt(pow(xs - P2x,2)+pow(ys - P2y,2)+pow(zs - P2z,2));
			vec1[0] = P2x - P1x;
			vec1[1] = P2y - P1y;
			vec1[2] = P2z - P1z;
			vec2[0] = P2x - xs;
			vec2[1] = P2y - ys;
			vec2[2] = P2z - zs;
			float VAL4 = vec1[0]*vec2[0] + vec1[1]*vec2[1] + vec1[2]*vec2[2];
			//Save VAL1-4 to disk
			myfile << VAL1 << std::endl << VAL2 << std::endl << VAL3 << std::endl << VAL4 << std::endl;
			myfile << vec1[0] << std::endl << vec1[1] << std::endl << vec1[2] << std::endl;
			myfile << vec2[0] << std::endl << vec2[1] << std::endl << vec2[2] << std::endl;
			myfile.close();
			//Save the transformation matrix as a rosbag
			rosbag::Bag bag;
			std::stringstream fs2;
			fs2 << filename << "marker.bag";
			bag.open(fs2.str().c_str(), rosbag::bagmode::Write);
			std_msgs::Float64 f;
			for(int fctr = 0; fctr < 4; ++fctr){
				for(int fctr2 = 0; fctr2 < 4; ++fctr2){
					f.data = A_mat(fctr2,fctr);
					bag.write("fiducial", ros::Time::now(), f);
				}
			}
			bag.close();
	}

}
//Call the trajectory service and provide filename to save trajectory to disk
int QNode::gen_trajectory(std::string filename){
	ROS_INFO("PC size %lu",current_pc_->size());
	if(current_pc_->size() != 0){
		//Set the pointcloud to cover
		sensor_msgs::PointCloud2 cloud_msg;
		pcl::toROSMsg(*current_pc_,cloud_msg);
		//fix the naming discrepancy between ROS and PCL (from "intensities" to "intensity")
		cloud_msg.fields[3].name = "intensities";
		cloud_msg.header.frame_id = "/base_footprint";
		traj_srv.request.cloud_in = cloud_msg;
	}else{
		//the part of the pointcloud to keep
		Eigen::Vector4f minPoint; 
		minPoint[0]=cropMinX;  // define minimum point x 
		minPoint[1]=cropMinY;  // define minimum point y 
		minPoint[2]=cropMinZ;  // define minimum point z 
		Eigen::Vector4f maxPoint; 
		maxPoint[0]=cropMaxX;  // define max point x 
		maxPoint[1]=cropMaxY;  // define max point y 
		maxPoint[2]=cropMaxZ;  // define max point z 
		Eigen::Vector3f boxTranslatation; 
		boxTranslatation[0]=0;   
		boxTranslatation[1]=0;   
		boxTranslatation[2]=0;   
		Eigen::Vector3f boxRotation; 
		boxRotation[0]=0;  // rotation around x-axis 
		boxRotation[1]=0;  // rotation around y-axis 
		boxRotation[2]=0;  //in radians rotation around z-axis. this rotates your cube 45deg around z-axis. 
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOut (new pcl::PointCloud<pcl::PointXYZI>); 
		pcl::CropBox<pcl::PointXYZI> cropFilter; 
		pcl::PointCloud<pcl::PointXYZI>::Ptr full_cloud (new pcl::PointCloud<pcl::PointXYZI>);
		//Tell pcl what the 4th field information is
		cloud_surface_world.fields[3].name = "intensity";
		pcl::fromROSMsg(cloud_surface_world, *full_cloud);
		cropFilter.setInputCloud (full_cloud); 
		cropFilter.setMin(minPoint); 
		cropFilter.setMax(maxPoint);
		cropFilter.setTranslation(boxTranslatation); 
		cropFilter.setRotation(boxRotation); 
		cropFilter.filter (*cloudOut); 
		//Crop points that are the robot itself
		minPoint[0]=occludeMinX;//-.01;  // define minimum point x 
		minPoint[1]=occludeMinY;//-.01;  // define minimum point y 
		minPoint[2]=occludeMinZ;//-0.01;  // define minimum point z 
		maxPoint[0]=occludeMaxX;//.0075;  // define max point x 
		maxPoint[1]=occludeMaxY;//.01;  // define max point y 
		maxPoint[2]=occludeMaxZ;//.014;  // define max point z 
		cropFilter.setInputCloud (cloudOut); 
		cropFilter.setMin(minPoint); 
		cropFilter.setMax(maxPoint);
		cropFilter.setNegative(true);
		cropFilter.filter (*cloudOut); 
		//Set the pointcloud to cover
		sensor_msgs::PointCloud2 cloud_msg;
		pcl::toROSMsg(*cloudOut,cloud_msg);
		//fix the naming discrepancy between ROS and PCL (from "intensities" to "intensity")
		cloud_msg.fields[3].name = "intensities";
		cloud_msg.header.frame_id = "/base_footprint";
		traj_srv.request.cloud_in = cloud_msg;
	}
	//Set the original surface for normals
	traj_srv.request.cloud_surface = cloud_surface_world;
	//Call the service
	if(traj_client.call(traj_srv)){
		if(DEBUG) ROS_INFO("Received Trajectory");
		//Save the trajectory response
		traj = traj_srv.response.trajectory;

		rosbag::Bag bag;
		std::stringstream bagname;
		bagname << filename << ++tctr << "trajectory.bag";
		bag.open(bagname.str().c_str(), rosbag::bagmode::Write);
		bag.write("trajectory", ros::Time::now(), traj);
		bag.close();
		finished_scan_pub.publish(TRAJ);



	}else{
		ROS_INFO("Service Failed");
		traj.points.clear();
	}	

	return traj.points.size();
}
//Load a prevously generated trajectory from file
int QNode::load_traj(std::string filename){
		
		rosbag::Bag bag;
		std::stringstream bagname;
		bagname << filename;
		ROS_INFO("Loading trajectory %s",bagname.str().c_str());
		bag.open(bagname.str().c_str(), rosbag::bagmode::Read);
		std::vector<std::string> topics;
		topics.push_back("trajectory");
		rosbag::View view(bag, rosbag::TopicQuery(topics));
		BOOST_FOREACH(rosbag::MessageInstance const m, view){

			phd::trajectory_msg::ConstPtr i = m.instantiate<phd::trajectory_msg>();
			if (i != NULL) traj = *i;
		}
		bag.close();	
		show_markers(traj);
		return traj.points.size();

}
//calculate thickess between two files
void QNode::thickness_from_file(std::string before,std::string after){


	pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_before (new pcl::PointCloud<pcl::PointXYZI> );
	pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_after (new pcl::PointCloud<pcl::PointXYZI> );
	//Create a ros message to publish the cloud as
	sensor_msgs::PointCloud2 cloud_before;
	sensor_msgs::PointCloud2 cloud_after;
	//Load the file
	if (pcl::io::loadPCDFile<pcl::PointXYZI> (before.c_str(), *pcl_before) == -1) PCL_ERROR ("Couldn't read file\n");

	//Load the file
	if (pcl::io::loadPCDFile<pcl::PointXYZI> (after.c_str(), *pcl_after) == -1) PCL_ERROR ("Couldn't read file\n");

	
	//Convert from PCL to ROS
	pcl::toROSMsg(*pcl_before,cloud_before);
	//fix the naming discrepancy between ROS and PCL (from "intensities" to "intensity")
	cloud_before.fields[3].name = "intensity";
	//Set the frame
	cloud_before.header.frame_id = "/world";
	//Convert from PCL to ROS
	pcl::toROSMsg(*pcl_after,cloud_after);
	//fix the naming discrepancy between ROS and PCL (from "intensities" to "intensity")
	cloud_after.fields[3].name = "intensity";
	//Set the frame
	cloud_after.header.frame_id = "/world";
		if(DEBUG) ROS_INFO("thickening");	
	phd::thickness_service thick_srv;
		//Call the localization service
		thick_srv.request.cloud_1 = cloud_before;
		thick_srv.request.cloud_2 = cloud_after;
		if(thick_client.call(thick_srv)){
			ROS_INFO("Thick Cloud %d", (uint32_t)(thick_srv.response.cloud_out.width));
			pub.publish(thick_srv.response.cloud_out);
			cloud_surface_world = thick_srv.response.cloud_out;

		pcl::PointCloud<pcl::PointXYZI>::Ptr save_cloud (new pcl::PointCloud<pcl::PointXYZI>);
		std::stringstream pf;
		//Tell pcl what the 4th field information is
		cloud_surface_world.fields[3].name = "intensity";
		pcl::fromROSMsg(cloud_surface_world, *save_cloud);
		pf << "/home/mike/testing/" << ++cloud_ctr << "thickened" << CLOUD << ".pcd";
		if(DEBUG) ROS_INFO("Saving to %s",pf.str().c_str());
		if(save_cloud->size() > 0) pcl::io::savePCDFileASCII (pf.str().c_str(), *save_cloud);


		}else ROS_INFO("Service Failed");

}
//save the selection
void QNode::save_selection(){
		std::stringstream pf;
		
		pf << "/home/mike/testing/" << ++cloud_ctr << "saved" << CLOUD << ".pcd";
		if(DEBUG) ROS_INFO("Saving to %s",pf.str().c_str());
		if(current_pc_->size() > 0) pcl::io::savePCDFileASCII (pf.str().c_str(), *current_pc_);

}

//Send a command to the DENSO node
void QNode::send_command(float x, float y, float z, float rx, float ry, float rz, float fig, int motion){

	phd::arm_msg msg;
	msg.x = x;
	msg.y = y;
	msg.z = z;
	msg.rx = rx;
	msg.ry = ry;
	msg.rz = rz;
	msg.fig = fig;
	msg.motion_type = motion;
	arm_pub.publish(msg);
	ros::spinOnce();


}
//Send a joint command to the DENSO node
void QNode::send_joint_command(float j1, float j2, float j3, float j4, float j5, float j6){
	 
	phd::arm_msg msg;
	msg.j1 = j1;
	msg.j2 = j2;
	msg.j3 = j3;
	msg.j4 = j4;
	msg.j5 = j5;
	msg.j6 = j6;
	msg.motion_type = JOINT;
	arm_pub.publish(msg);
	ros::spinOnce();
}

//Send a user entered string to the DENSO node
void QNode::send_string(std::string user_string){
	 
	phd::arm_msg msg;
	msg.user_string = user_string;
	msg.motion_type = STRING;
	arm_pub.publish(msg);
	ros::spinOnce();

}

//Generate RVIZ display to show trajectory path
void QNode::show_markers(phd::trajectory_msg t_msg){

	uint32_t shape = visualization_msgs::Marker::LINE_LIST;
	visualization_msgs::Marker marker;
	visualization_msgs::Marker path;
	visualization_msgs::Marker surface_path;
	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
	marker.header.frame_id = "/base_footprint";
	marker.header.stamp = ros::Time::now();
	marker.ns = "basic_shapes";
	marker.id = 0;
	marker.type = shape;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.01;
	marker.scale.y = 0.01;
	//marker.scale.y = 0.015;
	marker.color.r = 0.0f;
	marker.color.g = 0.0f;
	marker.color.b = 1.0f;
	marker.color.a = 0.8;
	marker.lifetime = ros::Duration();
	
	    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
	path.header.frame_id = "/base_footprint";
	path.header.stamp = ros::Time::now();
	// Set the namespace and id for this path.  This serves to create a unique ID
	// Any path sent with the same namespace and id will overwrite the old one
	path.ns = "path_shapes";
	path.id = 0;
	// Set the path type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
	path.type = visualization_msgs::Marker::LINE_STRIP;
	// Set the path action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	path.action = visualization_msgs::Marker::ADD;
	// Set the pose of the path.  This is a full 6DOF pose relative to the frame/time specified in the header
	path.pose.position.x = 0;
	path.pose.position.y = 0;
	path.pose.position.z = 0;
	path.pose.orientation.x = 0.0;
	path.pose.orientation.y = 0.0;
	path.pose.orientation.z = 0.0;
	path.pose.orientation.w = 1.0;
	// Set the scale of the path -- 1x1x1 here means 1m on a side
	path.scale.x = 0.01;
	// Set the color -- be sure to set alpha to something non-zero!
	path.color.r = 0.0f;
	path.color.g = 1.0f;
	path.color.b = 0.0f;
	path.color.a = 1.0;
	path.lifetime = ros::Duration();	    
	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
	surface_path.header.frame_id = "/base_footprint";
	surface_path.header.stamp = ros::Time::now();
	// Set the namespace and id for this path.  This serves to create a unique ID
	// Any path sent with the same namespace and id will overwrite the old one
	surface_path.ns = "surface_path_shapes";
	surface_path.id = 0;
	// Set the path type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
	surface_path.type = visualization_msgs::Marker::LINE_STRIP;
	// Set the path action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	surface_path.action = visualization_msgs::Marker::ADD;
	// Set the pose of the path.  This is a full 6DOF pose relative to the frame/time specified in the header
	surface_path.pose.position.x = 0;
	surface_path.pose.position.y = 0;
	surface_path.pose.position.z = 0;
	surface_path.pose.orientation.x = 0.0;
	surface_path.pose.orientation.y = 0.0;
	surface_path.pose.orientation.z = 0.0;
	surface_path.pose.orientation.w = 1.0;
	// Set the scale of the path -- 1x1x1 here means 1m on a side
	surface_path.scale.x = 0.01;
	// Set the color -- be sure to set alpha to something non-zero!
	surface_path.color.r = 1.0f;
	surface_path.color.g = 0.0f;
	surface_path.color.b = 0.0f;
	surface_path.color.a = 1.0;
	surface_path.lifetime = ros::Duration();
	path.id = 1;
	surface_path.id = 1;
	marker.id = 1;
	path.color.b = 1;
	marker.color.r = 1;
	int num_pts = t_msg.points.size();
	path.points.resize(num_pts);
	surface_path.points.resize(num_pts);
	marker.points.resize(2*num_pts);
	float P_OFFSET;
	ros::param::get("/global_offset", P_OFFSET);
	for(int i = 0; i < num_pts; ++i){
	
		ROS_INFO("SPoint %d: %f- %f- %f",i,t_msg.points[i].x,t_msg.points[i].y,t_msg.points[i].z);
		ROS_INFO("Normal %d: %f- %f- %f",i,t_msg.points[i].nx,t_msg.points[i].ny,t_msg.points[i].nz);
		path.points[i].x = t_msg.points[i].x-(P_OFFSET*t_msg.points[i].nx);
		path.points[i].y = t_msg.points[i].y-(P_OFFSET*t_msg.points[i].ny);
		path.points[i].z = t_msg.points[i].z-(P_OFFSET*t_msg.points[i].nz);
		surface_path.points[i].x = t_msg.points[i].x;
		surface_path.points[i].y = t_msg.points[i].y;
		surface_path.points[i].z = t_msg.points[i].z;

		ROS_INFO("Point %d: %f- %f- %f",i,path.points[i].x,path.points[i].y,path.points[i].z);
		marker.points[i*2].x = t_msg.points[i].x-(P_OFFSET*t_msg.points[i].nx);
		marker.points[i*2].y = t_msg.points[i].y-(P_OFFSET*t_msg.points[i].ny);
		marker.points[i*2].z = t_msg.points[i].z-(P_OFFSET*t_msg.points[i].nz);
		marker.points[(i*2)+1].x = t_msg.points[i].x;
		marker.points[(i*2)+1].y = t_msg.points[i].y;
		marker.points[(i*2)+1].z = t_msg.points[i].z;
	}

	dir_pub.publish(surface_path);
	ros::Duration(0.5).sleep();
	path_pub.publish(path);
	ros::Duration(0.5).sleep();
	marker_pub.publish(marker);
	ros::Duration(0.5).sleep();
	ros::spinOnce();

	

}
//Find a point near the location specified
pcl::PointXYZI QNode::find_and_delete_point(pcl::PointCloud<pcl::PointXYZI>::Ptr line, pcl::PointXYZI target_pt){
	pcl::PointXYZI ret;
	float resolution = 128.0f; //Octree resolution
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> octree (resolution);
	int K = 1; //Number of points to find
	std::vector<int> pointIdxNKNSearch; //vector to hold points (used by pcl)
	std::vector<float> pointNKNSquaredDistance; //vector to hold distance of points (used by pcl)
		octree.setInputCloud (line);
		octree.addPointsFromInputCloud ();

		
	if (octree.nearestKSearch (target_pt, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0){
		ret = line->points[pointIdxNKNSearch[0]];
			line->points.erase(line->points.begin()+pointIdxNKNSearch[0]);

	}else{
		ROS_INFO("octree error");
	}
	return ret;
}
//Reset various globals to their initial values
void  QNode::reset_things(){

	phd::trajectory_msg blank;
	traj = blank;

	current_pc_.reset(new pcl::PointCloud<pcl::PointXYZI>());
	raw_pc_.reset(new pcl::PointCloud<pcl::PointXYZI>());

}
//Find the next location to move to and go there
void  QNode::find_and_move(bool marker){

	pose_waiting = false;
	if(raw_pc_->size() == 0){
	if(DEBUG) ROS_INFO("Scanning");
		pose_waiting = true;
		marker_only = marker;
		QNode::scan("string",true);
	}else{
	if(DEBUG) ROS_INFO("Finding Pose");


	int K = 1; //Number of points to find
	std::vector<int> pointIdxNKNSearch; //vector to hold points (used by pcl)
	std::vector<float> pointNKNSquaredDistance; //vector to hold distance of points (used by pcl)
	float resolution = 128.0f; //Octree resolution
	pcl::PointXYZI foundpt; //Container for found point (to return)
	pcl::PointXYZI posept; //Container for found point (to return)
	//Create the octree object
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> octree (resolution);
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> octree2 (resolution);
	//Set the octree input cloud, then add the points
	//Set the pointcloud to cover
	sensor_msgs::PointCloud2 cloud_msg;
	Eigen::Vector4f minPoint = (Eigen::Vector4f() << 0,-100,1,0).finished();
	Eigen::Vector4f maxPoint = (Eigen::Vector4f() << 100,100,1.01,0).finished();	
	Eigen::Vector3f boxTranslatation = Eigen::ArrayXf::Zero(3); 
	Eigen::Vector3f boxRotation = Eigen::ArrayXf::Zero(3); 
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOut (new pcl::PointCloud<pcl::PointXYZI>); 
	pcl::CropBox<pcl::PointXYZI> cropFilter; 
	pcl::PointCloud<pcl::PointXYZI>::Ptr full_cloud (new pcl::PointCloud<pcl::PointXYZI>);
	//Tell pcl what the 4th field information is
	cloud_surface_world.fields[3].name = "intensity";
	cropFilter.setInputCloud (raw_pc_); 
	cropFilter.setMin(minPoint); 
	cropFilter.setMax(maxPoint);
	cropFilter.setTranslation(boxTranslatation); 
	cropFilter.setRotation(boxRotation); 
	cropFilter.filter (*cloudOut); 


	pcl::toROSMsg(*cloudOut,cloud_msg);
	cloud_msg.header.frame_id = "/base_footprint";
	cloud_msg.header.stamp = ros::Time::now();
	//pub3.publish(cloud_msg);
	pcl::PointCloud<pcl::PointXYZI> unsorted;
  	pcl::copyPointCloud(*cloudOut, unsorted);
	octree.setInputCloud (cloudOut);
	octree.addPointsFromInputCloud ();
	pcl::PointCloud<pcl::PointXYZI> sorted;
	pcl::PointXYZI point_holder;
	pcl::PointXYZI found_pt, prev_pt, next_pt, last_pt;
	pcl::PointCloud<pcl::PointXYZI>::iterator start_it, next_it;
	float c_val;
	pcl::PointXYZI dir, search_pt, d_pt, ctr_pt, ctr_pt2;
	float calc_d, d_total;
	float dir_val, old_d;	
	bool broken = false;
	Eigen::Vector4f zeroPoint, farPoint; 
	prev_pt.x =0;
	prev_pt.y =0;
	prev_pt.z =0;
	prev_pt.intensity = 0;
	next_pt = QNode::find_and_delete_point(cloudOut, prev_pt);
	next_pt.intensity = 0;
	sorted.push_back(next_pt);
	while(cloudOut->points.size()>0){	
		prev_pt = next_pt;
		next_pt = QNode::find_and_delete_point(cloudOut, prev_pt);
			next_pt.intensity = prev_pt.intensity + sqrt(pow(next_pt.x-prev_pt.x,2)+pow(next_pt.y-prev_pt.y,2)+pow(next_pt.z-prev_pt.z,2));
			//ROS_INFO("sotred %f - %f - %f / %f", next_pt.x, next_pt.y, next_pt.z, next_pt.intensity);
			if(fabs(next_pt.intensity - prev_pt.intensity) < 0.4) sorted.push_back(next_pt);
			else break;
		}
	ROS_INFO("return %lu", sorted.points.size());

	pcl::toROSMsg(sorted,cloud_msg);
	cloud_msg.header.frame_id = "/world";
	//pub4.publish(cloud_msg);



	//Crop points that should be finished
	minPoint[0]=-100;//-.01;  // define minimum point x 
	minPoint[1]=cropMinY;//-.01;  // define minimum point y 
	minPoint[2]=0;//-0.01;  // define minimum point z 
	maxPoint[0]=cropMaxX+1.5;//.0075;  // define max point x 
	maxPoint[1]=cropMaxY;//.01;  // define max point y 
	maxPoint[2]=10;//.014;  // define max point z 
	cropFilter.setInputCloud (unsorted.makeShared()); 
	cropFilter.setMin(minPoint); 
	cropFilter.setMax(maxPoint);
	cropFilter.setNegative(true);
	cropFilter.filter (*cloudOut); 

	pcl::toROSMsg(*cloudOut,cloud_msg);
	cloud_msg.header.frame_id = "/world";
	//pub5.publish(cloud_msg);

	pcl::PointXYZI target_pt;	
	target_pt.x = 0;	
	target_pt.y = 0;	
	target_pt.z = 1;
	ROS_INFO("cloud size %lu",cloudOut->points.size());
	if(cloudOut->points.size() > 0){
		foundpt = QNode::find_and_delete_point(cloudOut, target_pt);
		ROS_INFO("Moving to pt %f - %f - %f",foundpt.x,foundpt.y,foundpt.z);
		//pcl wants a cloud of points at which to calculate normals
		pcl::PointCloud<pcl::PointXYZI> cloudB;
		cloudB.push_back (foundpt);	
		// Create the normal estimation class
		pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
		// set the input for our normal estimation cloud
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloudptr = cloudB.makeShared();
		ne.setInputCloud (cloudptr);
		// Pass the original data (before downsampling) as the search surface
		ne.setSearchSurface (raw_pc_);
		// Create an empty kdtree representation, and pass it to the normal estimation object.
		// Its content will be filled inside the object, based on the given surface dataset.
		pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI> ());
		ne.setSearchMethod (tree);

		// Output datasets
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal> ());

		// Use all neighbors in a sphere of radius 5cm
		ne.setRadiusSearch (0.1);
		ne.setViewPoint (0, 0, 0); //Which direction to point the normal
		// Compute the features
		ne.compute (*cloud_normals);
		//since we want the normal to point inwards, but we can set the viewpoint at infinity, flip the result
		posept.x = foundpt.x-cloud_normals->points[0].normal[0];
		posept.y = foundpt.y-cloud_normals->points[0].normal[1];
		posept.z = foundpt.z-cloud_normals->points[0].normal[2];
		move_base_msgs::MoveBaseGoal goal;
		goal.target_pose.header.frame_id = "base_footprint";
		goal.target_pose.header.stamp = ros::Time::now();

		goal.target_pose.pose.position.x = foundpt.x+cloud_normals->points[0].normal[0]*POSE_OFFSET;
		goal.target_pose.pose.position.y = foundpt.y+cloud_normals->points[0].normal[1]*POSE_OFFSET;
		goal.target_pose.pose.position.z = 0;
		quaternionTFToMsg(tf::createQuaternionFromRPY(0, 0,(1.57 + atan2(cloud_normals->points[0].normal[1],cloud_normals->points[0].normal[0]))), goal.target_pose.pose.orientation);
		if(!marker){
			MoveBaseClient.sendGoal(goal);
			ROS_INFO("Sending goal");
			raw_pc_.reset(new pcl::PointCloud<pcl::PointXYZI>());
		}
		geometry_msgs::Pose ggoal;
		goals.poses.clear();
		ggoal.position = goal.target_pose.pose.position;
		ggoal.orientation = goal.target_pose.pose.orientation;
		goals.poses.push_back(ggoal);
		ggoal.position.x = foundpt.x;
		ggoal.position.y = foundpt.y;
		ggoal.position.z = 0;
		quaternionTFToMsg(tf::createQuaternionFromRPY(0, 0, 0), ggoal.orientation);

		goals.poses.push_back(ggoal);
		show_nav();
	}

}
}
}
};


