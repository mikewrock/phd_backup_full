// Always goes first
#define _CUSTOM_UINT64
// ROS
#include <ros/ros.h>
//Header for this file
#include "../include/control_panel.hpp"
//#include <atlbase.h>
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
//C++ includes
#include <math.h>
#include <string>
#include <cstdio>
#include <QVariant>
#include <iostream>
// Services
#include "laser_assembler/AssembleScans2.h"
#include "phd/localize_cloud.h"
#include "phd/trajectory_service.h"
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
#define OFFSET .8
#define WORKSPACE_WIDTH 0.5

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
phd::trajectory_service traj_srv;
//Dynamic Reconfig Variables
std::string CLOUD;
int CLOUD_NUM;
float VINT_MIN,VINT_MAX;

//DELETE
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_aligner (new pcl::PointCloud<pcl::PointXYZI> );

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
	ROS_INFO("Callback");


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
	rawpub = nh_.advertise<sensor_msgs::PointCloud2> ("raw_cloud", 1);
	pub2 = nh_.advertise<sensor_msgs::PointCloud2> ("aligned_cloud", 1);
	pub3 = nh_.advertise<sensor_msgs::PointCloud2> ("aligned_cloud3", 1);
	pub4 = nh_.advertise<sensor_msgs::PointCloud2> ("aligned_cloud4", 1);
	pub5 = nh_.advertise<sensor_msgs::PointCloud2> ("aligned_cloud5", 1);
	marker_pub = nh_.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
	path_pub = nh_.advertise<visualization_msgs::Marker>( "path_marker", 0 );
	dir_pub = nh_.advertise<visualization_msgs::Marker>( "dir_marker", 0 );
	arm_pose_pub = nh_.advertise<visualization_msgs::Marker>( "pose_marker", 0 );
	//Publisher for displaying navigation goal
	nav_vis_pub = nh_.advertise<visualization_msgs::Marker>( "nav_vis_goal", 0 );
	//Publisher for advertising navigation goal
	nav_pub = nh_.advertise<geometry_msgs::PoseStamped>( "nav_goal", 0 );
	//Subscribers to listen to the powercube position and point cloud selection
	joint_sub = nh_.subscribe("cube_joint_states", 1, &control_panel::control_panel_ns::QNode::jointCallback,this);
	cloud_sub = nh_.subscribe("marker_selected_points", 1, &control_panel::control_panel_ns::QNode::cloudCallback,this);
	//traj_sub = nh_.subscribe("trajectory_points", 1, &control_panel::control_panel_ns::QNode::cloudCallback,this);
	callback_type = boost::bind(&reconfig_callback, _1, _2);
	server.setCallback(callback_type);
	//Service client for calling the laser scan assembler
	client = nh_.serviceClient<laser_assembler::AssembleScans2>("assemble_scans2");
	//Service client for transforming th point cloud to the world coordinate frame
	loc_client = nh_.serviceClient<phd::localize_cloud>("localize_pcd");
	//Service client for gerneating an arm trajectory
	traj_client = nh_.serviceClient<phd::trajectory_service>("trajectory_gen");
	//Service client for estimating thickness
	thick_client = nh_.serviceClient<phd::thickness_service>("thick_srv");
	//Initialize the trajectory display marker	
	point_list.header.frame_id = "/base_link";
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
	sec_ctr = 0;
	pt_ctr = 0;
	cloud_ctr = 0;
	tctr = 0;
	current_pc_.reset(new pcl::PointCloud<pcl::PointXYZI>());
	//ros::start();
}

//Determines the appropriate robot base pose for executing the arm trajectory
phd::trajectory_point QNode::find_pose(void){
	phd::trajectory_point ret;
	//Custome messages dont have constructors, so this makes sure there is no random data in the message
	ret.x = 0;
	ret.y = 0;
	ret.z = 0;
	ret.nx = 0;
	ret.ny = 0;
	ret.nz = 0;
	ret.d = 0;
	//Counter for averaging the bottom points of the trajectory
	int avg_ctr = 0;
	if(DEBUG) ROS_INFO("start Point %f - %f",traj.sections[sec_ctr].points[0].x,traj.sections[sec_ctr].points[0].y);
	//Average the normals of the bottom line, and find the furthest point on the bottom line for determining the midpoint
	for(int ctr = 0; ctr < traj.sections[sec_ctr].points.size(); ++ctr){
		//Only look at the bottom line of the trajectory		
		if(traj.sections[sec_ctr].points[ctr].z - traj.sections[sec_ctr].points[0].z < 0.02){
			//Only average the XY values of the normal, we dont need Z			
			ret.nx += traj.sections[sec_ctr].points[ctr].nx;
			ret.ny += traj.sections[sec_ctr].points[ctr].ny;
			avg_ctr++;
			//Look for furthest trjectory point on the bottom line
			if(traj.sections[sec_ctr].points[ctr].d>ret.d){
				ret.x = traj.sections[sec_ctr].points[ctr].x;
				ret.y = traj.sections[sec_ctr].points[ctr].y;
				ret.d = traj.sections[sec_ctr].points[ctr].d;
			}
		}	
	}
	//Normals point toward the surface, so we flip them when making them in to an average
	ret.nx = -ret.nx/avg_ctr;
	ret.ny = -ret.ny/avg_ctr;
	//Find the midpoint of the bottom trajectory section
	ret.x = traj.sections[sec_ctr].points[0].x+((ret.x - traj.sections[sec_ctr].points[0].x)/2);
	ret.y = traj.sections[sec_ctr].points[0].y+((ret.y - traj.sections[sec_ctr].points[0].y)/2);
	ret.z = 0;
	if(DEBUG) ROS_INFO("Returning %f - %f/ %f - %f",ret.x,ret.y,ret.nx,ret.ny);
	return ret;
}
void QNode::calc_poses(){

	

}
//Calculate and display the current naviagtion goal
void QNode::show_nav() {

	visualization_msgs::Marker nav_goal;
	nav_goal.header.frame_id = "/base_link";
	nav_goal.header.stamp = ros::Time::now();
	nav_goal.ns = "nav_goal_marker";
	nav_goal.action = visualization_msgs::Marker::ADD;
	nav_goal.pose.orientation.w = 1.0;
	nav_goal.id = 0;
	nav_goal.type = visualization_msgs::Marker::ARROW;
	nav_goal.scale.x = 0.01;
	nav_goal.scale.y = 0.02;
	nav_goal.color.g = 1.0;
	nav_goal.color.a = 1;
	geometry_msgs::Point p;
	phd::trajectory_point tp;
	//Find the appropriate pose
	tp = find_pose();
	//Offset the pose from the surface in the direction normal to the surface
	p.x = tp.x+0.2*tp.nx;
	p.y = tp.y+0.2*tp.ny;
	p.z = 0;
	nav_goal.points.push_back(p);
	//The tip of the nav goal is a right angle to the normal (cross product of the Z axis and the normal)
	p.x -= 0.2*tp.ny;
	p.y += 0.2*tp.nx;
	nav_goal.points.push_back(p);
	//Display the marker
	nav_vis_pub.publish(nav_goal);

}

//Send the desired base pose to the global planner
void QNode::exe_nav() {

	geometry_msgs::PoseStamped pose;
	pose.header.frame_id = "/base_link";
	pose.header.stamp = ros::Time::now();
	pose.pose.position.x = 0;
	pose.pose.position.y = 0;
	pose.pose.position.z = 0;
	pose.pose.orientation.x = 1;
	pose.pose.orientation.y = 0;
	pose.pose.orientation.z = 0;
	pose.pose.orientation.w = 0;
	nav_pub.publish(pose);

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

//Testing function used to step through the trajectory points
void QNode::step(bool arm){
	//Generate and arm_msg and populate it with the current point in the trajectory array
	phd::arm_msg msg;
	msg.x = traj.sections[sec_ctr].points[pt_ctr].x-traj.sections[sec_ctr].points[pt_ctr].nx*OFFSET -.156971;
	msg.y = traj.sections[sec_ctr].points[pt_ctr].y-traj.sections[sec_ctr].points[pt_ctr].ny*OFFSET + .096013;
	msg.z = traj.sections[sec_ctr].points[pt_ctr].z-traj.sections[sec_ctr].points[pt_ctr].nz*OFFSET - .405369;
	msg.x = msg.x*1000;
	msg.y = msg.y*1000;
	msg.z = msg.z*1000;
	//Caluclate the rotation around the X and Y axis from the normal vector
	float length = sqrt(pow(traj.sections[sec_ctr].points[pt_ctr].nx,2)+pow(traj.sections[sec_ctr].points[pt_ctr].ny,2)+pow(traj.sections[sec_ctr].points[pt_ctr].nz,2));
	msg.rx = asin(traj.sections[sec_ctr].points[pt_ctr].ny / length);
	//Prevent divide by zero errors
	if(cos(msg.rx)!=0) msg.ry = asin( traj.sections[sec_ctr].points[pt_ctr].nx / (cos(msg.rx)*length) );
	else msg.ry = 0;
	//No need for the roll around the Z axis, since the spray pattern and radiation detection is conical
	msg.rz = 0;
	//This is the elbow configuration of the arm
	msg.fig = 1;
	//Increment the point counter, and if necessary the trajectory section counter
	++pt_ctr;
	if(pt_ctr >= traj.sections[sec_ctr].points.size()){
		++sec_ctr;
		pt_ctr = 0;
	}
	msg.motion_type = PTP;

	//DELETE THIS
	msg.rx = 0;
	msg.ry = 90;
	msg.rz = 0;
	//Initialize the trajectory display marker	
	armpoint_list.header.frame_id = "/BASE";
	armpoint_list.header.stamp = ros::Time::now();
	armpoint_list.ns = "points_and_lines";
	armpoint_list.action = visualization_msgs::Marker::ADD;
	armpoint_list.pose.orientation.w = 1.0;
	armpoint_list.id = 0;
	armpoint_list.type = visualization_msgs::Marker::POINTS;
	armpoint_list.scale.x = 0.01;
	armpoint_list.scale.y = 0.01;
	armpoint_list.color.r = 1.0;
	armpoint_list.color.a = 1;
	geometry_msgs::Point p;	
	p.x = -msg.x/1000;
	p.y = -msg.y/1000;
	p.z = msg.z/1000;
	armpoint_list.points.push_back(p);
	arm_pose_pub.publish(armpoint_list);
	ros::spinOnce();
	//Send the command to the arm
	if(arm) arm_pub.publish(msg);

}

//Perform a laser scan, either localize the cloud or set it as the global frame, and save to disk
void QNode::scan(std::string markername, bool localize){
	//Create a message to send to the powercube node, moving the powercube to its start location
	phd::cube_msg cube_cmd;
	cube_cmd.j1 = -2;
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
	while(joint1 > -1.9 && ros::ok()){
	ros::spinOnce();
	} 
	//Set the start time for the laser assembler service	
	srv.request.begin = ros::Time::now();
	//Tell the cube to go to the end location
	cube_cmd.j1 = 0;
	cube_cmd.vel = 0.25;
	cmd_pub.publish(cube_cmd);
	ros::spinOnce();
	if(DEBUG) ROS_INFO("waiting for end");
	//Wait for cube to reach end location
	while(joint1 < -.01 && ros::ok()){
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
		rawpub.publish(cloud_surface_raw);
		//Save the pointcloud to disk using markername CLOUD and the counter
		pcl::PointCloud<pcl::PointXYZI>::Ptr save_cloud (new pcl::PointCloud<pcl::PointXYZI>);
		std::stringstream pf;
		//Tell pcl what the 4th field information is
		cloud_surface_raw.fields[3].name = "intensity";
		pcl::fromROSMsg(cloud_surface_raw, *save_cloud);
		cloud_ctr++;
		pf << markername << cloud_ctr << "raw" << CLOUD << ".pcd";
		if(DEBUG) ROS_INFO("Saving to %s",pf.str().c_str());
		pcl::io::savePCDFileASCII (pf.str().c_str(), *save_cloud);
		//Tell ros what the 4th field information is
		cloud_surface_raw.fields[3].name = "intensities";
		//If set home box is checked, tell localization to save marker location to the filename specified, otheriwse load the file and localize
		if(DEBUG&&localize) ROS_INFO("Setting Home");	
		//Call the localization service, it will transform the cloud if the set home box is not checked
		loc_srv.request.cloud_in = cloud_surface_raw;
		loc_srv.request.homing = localize;
		loc_srv.request.marker_file = markername;
		if(loc_client.call(loc_srv)){
			if(DEBUG) ROS_INFO("Localize Service Cloud %d", (uint32_t)(loc_srv.response.cloud_out.width));
			pub.publish(loc_srv.response.cloud_out);
			sensor_msgs::PointCloud2 cloud_save;
			cloud_save = loc_srv.response.cloud_out;
			cloud_save.fields[3].name = "intensity";
			save_cloud->clear();
			pcl::fromROSMsg(cloud_save, *save_cloud);
			pf.str("");			
			if(localize){
				cloud_surface_world = loc_srv.response.cloud_out;
				pf << markername << cloud_ctr << "home" << CLOUD << ".pcd";
				transform.setOrigin( tf::Vector3(0, 0, 0) );
				q.setRPY(0, 0, 0);
				transform.setRotation(q);
				br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base_footprint"));
			}else{
				cloud_surface_world = loc_srv.response.cloud_out;
				transform.setOrigin( tf::Vector3(loc_srv.response.transform_mat[3], loc_srv.response.transform_mat[7], loc_srv.response.transform_mat[11]) );
				/*Eigen::Matrix3d m;
				m <<static_cast<float>(loc_srv.response.transform_mat[0]),static_cast<float>(loc_srv.response.transform_mat[1]),static_cast<float>(loc_srv.response.transform_mat[2]),
				  static_cast<float>(loc_srv.response.transform_mat[4]),static_cast<float>(loc_srv.response.transform_mat[5]),static_cast<float>(loc_srv.response.transform_mat[6]),
				  static_cast<float>(loc_srv.response.transform_mat[8]),static_cast<float>(loc_srv.response.transform_mat[9]),static_cast<float>(loc_srv.response.transform_mat[10]);
				m(0,0) = loc_srv.response.transform_mat[0];
				m(0,1) = loc_srv.response.transform_mat[1];
				m(0,2) = loc_srv.response.transform_mat[2];
				m(1,0) = loc_srv.response.transform_mat[4];
				m(1,1) = loc_srv.response.transform_mat[5];
				m(1,2) = loc_srv.response.transform_mat[6];
				m(2,0) = loc_srv.response.transform_mat[8];
				m(2,1) = loc_srv.response.transform_mat[9];
				m(2,2) = loc_srv.response.transform_mat[10];*/
				tf::Matrix3x3 m(loc_srv.response.transform_mat[0], loc_srv.response.transform_mat[1], loc_srv.response.transform_mat[2], loc_srv.response.transform_mat[4], loc_srv.response.transform_mat[5], loc_srv.response.transform_mat[6], loc_srv.response.transform_mat[8], loc_srv.response.transform_mat[9], loc_srv.response.transform_mat[10]);
				m.getRotation(q);
				transform.setRotation(q);
				br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base_footprint"));
				pf << markername << cloud_ctr << "localized" << CLOUD << ".pcd";
			}
			pcl::io::savePCDFileASCII (pf.str().c_str(), *save_cloud);
		}else{
			ROS_INFO("Localize Service Failed");
		}

	}
	else ROS_ERROR("Error making service call to laser assembler\n") ;
	ros::spinOnce();

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
void QNode::fscan(std::string filename, bool auto_localize, std::string markername){

//DELETE THIS
/*else {
//std::cerr << "Sending cloud with: " << cloud->width * cloud->height << " data points." << std::endl;
sensor_msgs::PointCloud2 cloud_msg;
	pcl::toROSMsg(*cloud_aligner,cloud_msg);
	//fix the naming discrepancy between ROS and PCL (from "intensities" to "intensity")
	cloud_msg.fields[3].name = "intensities";

	cloud_msg.header.frame_id = "/base_link";
		      pub.publish(cloud_msg);
	ros::spinOnce();
return;
}*/

	//Create a pcl pointer to load the clound in to
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI> );
	//Create a ros message to publish the cloud as
	sensor_msgs::PointCloud2 cloud_msg;
	//Load the file
	if (pcl::io::loadPCDFile<pcl::PointXYZI> (filename.c_str(), *cloud) == -1) PCL_ERROR ("Couldn't read file\n");
	else{
		if(DEBUG) ROS_INFO("File Opened");
	
	/*//////////Crop the top
	//Create containers for the filtered cloud
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
	// Create the filtering object
	pcl::PassThrough<pcl::PointXYZI> pass (true);
	//filter the data for intensity
	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (-10,1.3);
	pass.filter (*cloud_filtered);
	*/	
	//Convert from PCL to ROS
	pcl::toROSMsg(*cloud,cloud_msg);
	//fix the naming discrepancy between ROS and PCL (from "intensities" to "intensity")
	cloud_msg.fields[3].name = "intensity";
	//Set the frame
	cloud_msg.header.frame_id = "/base_link";
	if(auto_localize){
		if(DEBUG) ROS_INFO("Auto Localizing");	
		//Call the localization service
		loc_srv.request.cloud_in = cloud_msg;
		loc_srv.request.homing = false;
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
		pf << markername << ++cloud_ctr << "localized" << CLOUD << ".pcd";
		if(DEBUG) ROS_INFO("Saving to %s",pf.str().c_str());
		//pcl::io::savePCDFileASCII (pf.str().c_str(), *save_cloud);
			//Open file for saving marker location
	ofstream myfile;
	std::stringstream fs;
	fs << "/home/mike/testing/poses.csv";
	myfile.open (fs.str().c_str(), std::ios::out|std::ios::app);
	myfile << CLOUD << cloud_ctr << ",";
	for(int i = 0; i <16; ++i){
		myfile << loc_srv.response.transform_mat[i] << ",";
	}
	myfile << std::endl;
		}else ROS_INFO("Service Failed");
	}else {
		cloud_surface_world = cloud_msg;
		rawpub.publish(cloud_msg);
	}
	}
}
//Testing function: Localization scan, performs a laser scan and sets the frame /base_footprint as the origin of the global coordinate frame 
void QNode::lscan(){

	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI> );


	if (pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/mike/marker/refined/dotsf3.pcd", *cloud) == -1) 
	{
		PCL_ERROR ("Couldn't read file test_pcd.pcd \n");

	}else 
	std::cerr << "Sending cloud with: " << cloud->width * cloud->height << " data points." << std::endl;

	sensor_msgs::PointCloud2 cloud_msg;

	loc_srv.request.cloud_in = cloud_msg;
	loc_srv.request.homing = true;
	if(loc_client.call(loc_srv)){
	ROS_INFO("Received Cloud %d", (uint32_t)(loc_srv.response.cloud_out.width));
		      pub.publish(loc_srv.response.cloud_out);
			cloud_surface_world = loc_srv.response.cloud_out;
	}else ROS_INFO("Service Failed");
}
//This function save the location of the IR reflective ball by clustering the highly reflective points in to a single location
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
//The start pt is used to calculate the starting z height for trajectory generation and what corner of the selected area to start at
void QNode::start_pt(){

	T1x = current_pc_->points[0].data[0];
	T1y = current_pc_->points[0].data[1];
	T1z = current_pc_->points[0].data[2];


}
//Call the trajectory service, providing it with a starting point, pointcloud to cover, and original pointcloud surface (for calculating normals) 
void QNode::gen_trajectory(std::string filename){
	if(DEBUG) ROS_INFO("Generating Trajectory starting from %f - %f - %f",T1x,T1y,T1z) ;
	//Set the start point
	traj_srv.request.P1x = T1x;
	traj_srv.request.P1y = T1y;
	traj_srv.request.P1z = T1z;
	ROS_INFO("PC size %lu",current_pc_->size());
	if(current_pc_->size() != 0){
		//Set the pointcloud to cover
		sensor_msgs::PointCloud2 cloud_msg;
		pcl::toROSMsg(*current_pc_,cloud_msg);
		//fix the naming discrepancy between ROS and PCL (from "intensities" to "intensity")
		cloud_msg.fields[3].name = "intensities";
		cloud_msg.header.frame_id = "/world";
		traj_srv.request.cloud_in = cloud_msg;
	}else{
		Eigen::Vector4f minPoint; 
		minPoint[0]=0;  // define minimum point x 
		minPoint[1]=-WORKSPACE_WIDTH;  // define minimum point y 
		minPoint[2]=-0.1;  // define minimum point z 
		Eigen::Vector4f maxPoint; 
		maxPoint[0]=10;  // define max point x 
		maxPoint[1]=WORKSPACE_WIDTH;  // define max point y 
		maxPoint[2]=1.37;  // define max point z 
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
		//Set the pointcloud to cover
		sensor_msgs::PointCloud2 cloud_msg;
		pcl::toROSMsg(*cloudOut,cloud_msg);
		//fix the naming discrepancy between ROS and PCL (from "intensities" to "intensity")
		cloud_msg.fields[3].name = "intensities";
		cloud_msg.header.frame_id = "/world";
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



	}else ROS_INFO("Service Failed");

}

void QNode::load_traj(std::string filename){
		
		rosbag::Bag bag;
		std::stringstream bagname;
		bagname << filename;
		ROS_INFO("Loading trajectory %s",bagname.str().c_str());
		bag.open(bagname.str().c_str(), rosbag::bagmode::Read);
		std::vector<std::string> topics;
		topics.push_back("trajectory");
		rosbag::View view(bag, rosbag::TopicQuery(topics));
		BOOST_FOREACH(rosbag::MessageInstance const m, view){

			phd::trajectory_array::ConstPtr i = m.instantiate<phd::trajectory_array>();
			if (i != NULL) traj = *i;
		}
		bag.close();	
		show_markers(traj);


}
//calculate thickess between two files
void QNode::thickness(std::string before,std::string after){


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
	cloud_before.header.frame_id = "/base_link";
	//Convert from PCL to ROS
	pcl::toROSMsg(*pcl_after,cloud_after);
	//fix the naming discrepancy between ROS and PCL (from "intensities" to "intensity")
	cloud_after.fields[3].name = "intensity";
	//Set the frame
	cloud_after.header.frame_id = "/base_link";
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
		pcl::io::savePCDFileASCII (pf.str().c_str(), *save_cloud);


		}else ROS_INFO("Service Failed");

}
//Estimate the base position using registration
void QNode::estimate(std::string filename){
	if(DEBUG) setVerbosityLevel(pcl::console::L_VERBOSE); 
	//pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_tgt_downsampled (new pcl::PointCloud<pcl::PointXYZI> );
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_src_downsampled (new    pcl::PointCloud<pcl::PointXYZI> );
	pcl::VoxelGrid<pcl::PointXYZI> sor;
	//Downsample the alignment section
	sor.setInputCloud (cloud_aligner);
	sor.setLeafSize (0.01f, 0.01f, 0.01f);
	sor.filter (*cloud_src_downsampled);
	////Set source for the iterative closes point algorithm
	//icp.setInputSource (cloud_src_downsampled);
	//Load the target to which we will align the source
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_tgt (new pcl::PointCloud<pcl::PointXYZI> );
	std::stringstream fs;
	fs << filename << CLOUD << CLOUD_NUM << ".pcl";
	pcl::io::loadPCDFile<pcl::PointXYZI> (fs.str().c_str(), *cloud_tgt);
	//Downsample the target cloud	
	sor.setInputCloud (cloud_tgt);
	sor.filter (*cloud_tgt_downsampled);
	// Create the normal estimation class
	pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
	// set parameters
	ne.setInputCloud (cloud_aligner);
	// Pass the original data (before downsampling) as the search surface
	ne.setSearchSurface (cloud_aligner);
	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given surface dataset.
	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI> ());
	ne.setSearchMethod (tree);
	// Normal pointcloud for source 
	pcl::PointCloud<pcl::Normal>::Ptr cloud_src_normals (new pcl::PointCloud<pcl::Normal> ());
	// Use all neighbors in a sphere of radius RAD cm
	ne.setRadiusSearch (0.05);
	ne.setViewPoint (0, 0, 0);
	// Compute the features
	//The if/else is to speed up testing and avoid recalculating normals, you can load normals from file
	//Set filename for source normals file
	std::stringstream ns;
	ns << filename << CLOUD << "_src_normals.pcd";
	if(CALC){
		//Compute cloud normals
		ne.compute (*cloud_src_normals);
		pcl::io::savePCDFileASCII (ns.str().c_str(), *cloud_src_normals);
	}else
	pcl::io::loadPCDFile<pcl::Normal> (ns.str().c_str(), *cloud_src_normals);
	//Calculate the normals of the target cloud	
	ne.setInputCloud (cloud_tgt);
	// Pass the original data (before downsampling) as the search surface
	ne.setSearchSurface (cloud_tgt);
	// Normal pointcloud for target
	pcl::PointCloud<pcl::Normal>::Ptr cloud_tgt_normals (new pcl::PointCloud<pcl::Normal> ());
	//Compute target normals	
	ne.compute (*cloud_tgt_normals);
	//Set filename for target normals file
	std::stringstream nt;
	nt << filename << CLOUD << "_tgt_normals.pcd";
	pcl::io::savePCDFileASCII (nt.str().c_str(), *cloud_tgt_normals);
	//Pointclouds for intensity keypoints
	pcl::PointCloud<pcl::PointXYZI>::Ptr intensity_keypoints (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr intensity_keypoints_tgt (new pcl::PointCloud<pcl::PointXYZI>);
	//Pointcloud for SUSAN keypoints
	pcl::SUSANKeypoint<pcl::PointXYZI, pcl::PointXYZI> susan;
	//Set SUSAN keypoint parameters
	susan.setNonMaxSupression (true);
	if(DEBUG) ROS_INFO("Input %lu points",cloud_aligner->size());
	susan.setInputCloud (cloud_aligner);
	susan.setNormals (cloud_src_normals);
	susan.setRadiusSearch (0.03f);
	//Filename to save keypoints
	std::stringstream nk;
	nk << filename << CLOUD << "_src_keypoints.pcd";
	if(CALC){
		susan.compute (*intensity_keypoints);
		pcl::io::savePCDFileASCII (nk.str().c_str(), *intensity_keypoints);
	}else pcl::io::loadPCDFile<pcl::PointXYZI> (nk.str().c_str(), *intensity_keypoints);
	if(DEBUG) ROS_INFO("Found %lu src key points",intensity_keypoints->size());
	//Set input cloud and normals	
	susan.setInputCloud (cloud_tgt);
	susan.setNormals (cloud_tgt_normals);
	susan.compute (*intensity_keypoints_tgt);
	//Filename to save target keypoints
	std::stringstream nkt;
	nkt << filename << CLOUD << "_tgt_keypoints.pcd";
	pcl::io::savePCDFileASCII (nkt.str().c_str(), *intensity_keypoints_tgt);
	if(DEBUG) ROS_INFO("Found %lu tgt key points",intensity_keypoints_tgt->size());
	//Create src feature dataset
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr features (new pcl::PointCloud<pcl::FPFHSignature33>);
	//Use kd tree search method
	pcl::search::KdTree<pcl::PointXYZI>::Ptr search_method_xyz_ (new pcl::search::KdTree<pcl::PointXYZI>);
	//Feature estimation class
	pcl::FPFHEstimation<pcl::PointXYZI, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
	//Calculate normals of source keypoints
	ne.setInputCloud (intensity_keypoints);
	// Pass the original data (before downsampling) as the search surface
	ne.setSearchSurface (cloud_aligner);
	// Keypoint normal dataset
	pcl::PointCloud<pcl::Normal>::Ptr keypoint_normals (new pcl::PointCloud<pcl::Normal> ());
	//Compute src keypoint normals
	ne.compute (*keypoint_normals);
	//Calculate normals of target keypoints
	ne.setInputCloud (intensity_keypoints_tgt);
	// Pass the original data (before downsampling) as the search surface
	ne.setSearchSurface (cloud_tgt);
	// Keypoint normal dataset
	pcl::PointCloud<pcl::Normal>::Ptr keypoint_normals_tgt (new pcl::PointCloud<pcl::Normal> ());
	//Compute tgt keypoint normals
	ne.compute (*keypoint_normals_tgt);
	//Compute src features from src keypoints
	fpfh_est.setInputCloud (intensity_keypoints);
	fpfh_est.setInputNormals (keypoint_normals);
	fpfh_est.setSearchMethod (search_method_xyz_);
	fpfh_est.setRadiusSearch (0.03f);
	fpfh_est.compute (*features);
	if(DEBUG) ROS_INFO("computed %lu src featurs",features->size());
	//Create tgt feature dataset
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr features_tgt (new pcl::PointCloud<pcl::FPFHSignature33>);
	//Compute tgt features from tgt keypoints
	fpfh_est.setInputCloud (intensity_keypoints_tgt);
	fpfh_est.setInputNormals (keypoint_normals_tgt);
	fpfh_est.compute (*features_tgt);
	if(DEBUG) ROS_INFO("computed %lu tgt featurs",features_tgt->size());
	//create an alignment for the transform from src to tgt
	pcl::SampleConsensusInitialAlignment<pcl::PointXYZI, pcl::PointXYZI, pcl::FPFHSignature33> sac_ia_;
	// Intialize the parameters in the Sample Consensus Intial Alignment (SAC-IA) algorithm
	sac_ia_.setMinSampleDistance (0.03f);
	sac_ia_.setMaxCorrespondenceDistance (.05);
	sac_ia_.setCorrespondenceRandomness (5);
	sac_ia_.setMaximumIterations (1000);
	sac_ia_.setInputTarget (intensity_keypoints);
	sac_ia_.setTargetFeatures (features);
	sac_ia_.setInputSource (intensity_keypoints_tgt);
	sac_ia_.setSourceFeatures (features_tgt);
	//Create output dataset
	pcl::PointCloud<pcl::PointXYZI> registration_output;
	//Provide an initial alignment guess
	//Eigen::Matrix4f guess = Eigen::Matrix4f::Identity ();
	// sac_ia_.align (registration_output, guess);
	//Keep track of the alignment fitness	
	float fitness_score = 1;
	while(fitness_score > 0.0015 && ros::ok()){
		//If the fitness isn't better than FIT, we will keep calling the alignment algorithm
		sac_ia_.align (registration_output);
		fitness_score = (float) sac_ia_.getFitnessScore (.05);
		if(DEBUG) ROS_INFO("Alignment fitness %f", fitness_score);
	}
	//Store the transformation matrix
	Eigen::Matrix4f final_transformation = sac_ia_.getFinalTransformation ();
	//Create a pointloud to hold the aligned pointcloud
	pcl::PointCloud<pcl::PointXYZI> Final;
	if(DEBUG) ROS_INFO("Final alignment fitness %f", fitness_score);

	//Should be done here, the rest is testing functions

	/*


	Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
	pcl::PointCloud<pcl::PointXYZI>::Ptr output (new pcl::PointCloud<pcl::PointXYZI> );
	// Transform target to source frame
	pcl::transformPointCloud (*cloud_tgt, *output, final_transformation);
	sensor_msgs::PointCloud2 cloud_msg;
	


	//pcl::transformPointCloud (*intensity_keypoints_tgt, *output4, final_transformation);

	//icp.align (Final, guess);
	//std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
	//std::cout << icp.getFinalTransformation() << std::endl;
	// Eigen::Matrix4f T;
	//T = icp.getFinalTransformation();

	//targetToSource = T.inverse();

	// pcl::transformPointCloud (*cloud_tgt, *output2, targetToSource);
	//final_transform = targetToSource;
	//pcl::toROSMsg(*output,cloud_msg);
	pcl::toROSMsg(*intensity_keypoints,cloud_msg);
	//fix the naming discrepancy between ROS and PCL (from "intensities" to "intensity")
	cloud_msg.fields[3].name = "intensities";

	cloud_msg.header.frame_id = "/base_link";
	pub2.publish(cloud_msg);
	sensor_msgs::PointCloud2 cloud_msg2;
	//pcl::toROSMsg(*output2,cloud_msg2);

	pcl::toROSMsg(*intensity_keypoints_tgt,cloud_msg2);	
	//fix the naming discrepancy between ROS and PCL (from "intensities" to "intensity")
	cloud_msg2.fields[3].name = "intensities";

	cloud_msg2.header.frame_id = "/base_link";
	pub3.publish(cloud_msg2);

	sensor_msgs::PointCloud2 cloud_msg5;
	//pcl::toROSMsg(*output2,cloud_msg2);

	pcl::toROSMsg(*output2,cloud_msg5);	
	//fix the naming discrepancy between ROS and PCL (from "intensities" to "intensity")
	cloud_msg5.fields[3].name = "intensities";

	cloud_msg5.header.frame_id = "/base_link";
	pub5.publish(cloud_msg5);

	sensor_msgs::PointCloud2 cloud_msg4;
	//pcl::toROSMsg(*output2,cloud_msg2);

	pcl::toROSMsg(*output4,cloud_msg4);	
	//fix the naming discrepancy between ROS and PCL (from "intensities" to "intensity")
	cloud_msg4.fields[3].name = "intensities";

	cloud_msg4.header.frame_id = "/base_link";
	pub4.publish(cloud_msg4);
	*/
}



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


void QNode::send_string(std::string user_string){
	 
	phd::arm_msg msg;
	msg.user_string = user_string;
	msg.motion_type = STRING;
	arm_pub.publish(msg);
	ros::spinOnce();

}

//Perform a laser scan, either localize the cloud or set it as the global frame, and save to disk
void QNode::scan_360(){
	//Create a message to send to the powercube node, moving the powercube to its start location
	phd::cube_msg cube_cmd;
	cube_cmd.j1 = -1.62;
	cube_cmd.vel = 0.5;
	cube_cmd.acc = 1;
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
	while(joint1 > -1.6 && ros::ok()){
	ros::spinOnce();
	} 
	//Set the start time for the laser assembler service	
	srv.request.begin = ros::Time::now();
	//Tell the cube to go to the end location
	cube_cmd.j1 = 1.62;
	cube_cmd.vel = 0.25;
	cmd_pub.publish(cube_cmd);
	ros::spinOnce();
	if(DEBUG) ROS_INFO("waiting for end");
	//Wait for cube to reach end location
	while(joint1 < 1.6 && ros::ok()){
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
		rawpub.publish(cloud_surface_raw);
		//Save the pointcloud to disk using markername CLOUD and the counter
		pcl::PointCloud<pcl::PointXYZI>::Ptr save_cloud (new pcl::PointCloud<pcl::PointXYZI>);
		std::stringstream pf;
		//Tell pcl what the 4th field information is
		cloud_surface_raw.fields[3].name = "intensity";
		pcl::fromROSMsg(cloud_surface_raw, *save_cloud);
		cloud_ctr++;
		pf << "360_scan" << cloud_ctr << "raw" << CLOUD << ".pcd";
		if(DEBUG) ROS_INFO("Saving to %s",pf.str().c_str());
		pcl::io::savePCDFileASCII (pf.str().c_str(), *save_cloud);
		//Tell ros what the 4th field information is
		cloud_surface_raw.fields[3].name = "intensities";
		
	}
	else ROS_ERROR("Error making service call to laser assembler\n") ;
	ros::spinOnce();

}

void QNode::show_markers(phd::trajectory_array t_array){

	uint32_t shape = visualization_msgs::Marker::LINE_LIST;
	visualization_msgs::Marker marker;
	visualization_msgs::Marker path;
	visualization_msgs::Marker surface_path;
	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
	marker.header.frame_id = "/world";
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
	path.header.frame_id = "/world";
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
	surface_path.header.frame_id = "/world";
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
	phd::trajectory_msg t_msg;
	for(int ctr = 0; ctr < t_array.sections.size(); ++ctr){

		path.id = ctr;
		surface_path.id = ctr;
		marker.id = ctr;
		path.color.b = 0.2*ctr;
		marker.color.r = 0.2*ctr;
		t_msg.points.swap(t_array.sections[ctr].points);
		int num_pts = t_msg.points.size();
		path.points.resize(num_pts);
		surface_path.points.resize(num_pts);
		marker.points.resize(2*num_pts);
		for(int i = 0; i < num_pts; ++i){
		
			//ROS_INFO("SPoint %d: %f- %f- %f",i,t_msg.points[i].x,t_msg.points[i].y,t_msg.points[i].z);
			//ROS_INFO("Normal %d: %f- %f- %f",i,t_msg.points[i].nx,t_msg.points[i].ny,t_msg.points[i].nz);
			path.points[i].x = t_msg.points[i].x-(OFFSET*t_msg.points[i].nx);
			path.points[i].y = t_msg.points[i].y-(OFFSET*t_msg.points[i].ny);
			path.points[i].z = t_msg.points[i].z-(OFFSET*t_msg.points[i].nz);
			surface_path.points[i].x = t_msg.points[i].x;
			surface_path.points[i].y = t_msg.points[i].y;
			surface_path.points[i].z = t_msg.points[i].z;

			//ROS_INFO("Point %d: %f- %f- %f",i,path.points[i].x,path.points[i].y,path.points[i].z);
			marker.points[i*2].x = t_msg.points[i].x-(OFFSET*t_msg.points[i].nx);
			marker.points[i*2].y = t_msg.points[i].y-(OFFSET*t_msg.points[i].ny);
			marker.points[i*2].z = t_msg.points[i].z-(OFFSET*t_msg.points[i].nz);
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
	

}

}
};


