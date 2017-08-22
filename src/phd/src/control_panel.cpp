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

#define CALC true
#define DEBUG true

namespace control_panel{
namespace control_panel_ns{

//For displaying arm trajectory
visualization_msgs::Marker point_list;
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
	ROS_INFO("Callback");


}
//Callback for storing current powercube position
void QNode::jointCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	std::string str1 = "cube_5_joint";
	std::string str2 =msg->name[0] ;
	//Since there are multiple joint_state_publishers, we need to make sure the message is actually the powercube joint	
	if(str1==str2){
	  joint1 = msg->position[0];
	}
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
	pub2 = nh_.advertise<sensor_msgs::PointCloud2> ("aligned_cloud", 1);
	pub3 = nh_.advertise<sensor_msgs::PointCloud2> ("aligned_cloud3", 1);
	pub4 = nh_.advertise<sensor_msgs::PointCloud2> ("aligned_cloud4", 1);
	pub5 = nh_.advertise<sensor_msgs::PointCloud2> ("aligned_cloud5", 1);
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
	cloud_ctr = 1;
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
void QNode::step(){
	//Generate and arm_msg and populatie it with the current point in the trajectory array
	phd::arm_msg msg;
	msg.x = traj.sections[sec_ctr].points[pt_ctr].x;
	msg.y = traj.sections[sec_ctr].points[pt_ctr].y;
	msg.z = traj.sections[sec_ctr].points[pt_ctr].z;
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
	//Send the command to the arm
	arm_pub.publish(msg);

}

//Perform a laser scan, either localize the cloud or set it as the global frame, and save to disk
void QNode::scan(std::string filename, bool localize){
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
	if(DEBUG) printf("j1is: %f\n",joint1);
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
		ROS_INFO("Published Cloud %d", (uint32_t)(srv.response.cloud.width)) ;
		//Save the new scan as cloud_surface		
		cloud_surface = srv.response.cloud;
		//Tell ros what the 4th field information is
		cloud_surface.fields[3].name = "intensities";
		//If set home box is checked, tell localization to save marker location to the filename specified, otheriwse load the file and localize
		if(DEBUG&&localize) ROS_INFO("Setting Home");	
		//Call the localization service, it will transform the cloud if the set home box is not checked
		loc_srv.request.cloud_in = cloud_surface;
		loc_srv.request.homing = localize;
		loc_srv.request.marker_file = filename;
		loc_client.call(loc_srv);
		if(loc_client.call(loc_srv)){
			if(DEBUG) ROS_INFO("Localize Service Cloud %d", (uint32_t)(loc_srv.response.cloud_out.width));
			pub.publish(loc_srv.response.cloud_out);
			if(localize) cloud_surface = loc_srv.response.cloud_out;
		}else ROS_INFO("Localize Service Failed");
		//Save the pointcloud to disk using filename CLOUD and the counter
		pcl::PointCloud<pcl::PointXYZI>::Ptr save_cloud (new pcl::PointCloud<pcl::PointXYZI>);
		pcl::fromROSMsg(cloud_surface, *save_cloud);
		std::stringstream pf;
		pf << filename << CLOUD << cloud_ctr++ << ".pcl";
		pcl::io::savePCDFileASCII (pf.str().c_str(), *save_cloud);
	}
	else ROS_ERROR("Error making service call to laser assembler\n") ;

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
void QNode::fscan(std::string filename, bool auto_localize){

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
	}
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
		loc_client.call(loc_srv);
		if(loc_client.call(loc_srv)){
			ROS_INFO("Localized Cloud %d", (uint32_t)(loc_srv.response.cloud_out.width));
			pub.publish(loc_srv.response.cloud_out);
			cloud_surface = loc_srv.response.cloud_out;
		}else ROS_INFO("Service Failed");
	}else {
		pub.publish(cloud_msg);
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
			cloud_surface = loc_srv.response.cloud_out;
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
	pass.setFilterLimits (550,1100);
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
void QNode::gen_trajectory(){
	if(DEBUG) ROS_INFO("Generating Trajectory") ;
	//Set the start point
	traj_srv.request.P1x = T1x;
	traj_srv.request.P1y = T1y;
	traj_srv.request.P1z = T1z;
	//Set the pointcloud to cover
	sensor_msgs::PointCloud2 cloud_msg;
	pcl::toROSMsg(*current_pc_,cloud_msg);
	//fix the naming discrepancy between ROS and PCL (from "intensities" to "intensity")
	cloud_msg.fields[3].name = "intensities";
	cloud_msg.header.frame_id = "/base_link";
	traj_srv.request.cloud_in = cloud_msg;
	//Set the original surface for normals
	traj_srv.request.cloud_surface = cloud_surface;
	//Call the service
	if(traj_client.call(traj_srv)){
		if(DEBUG) ROS_INFO("Received Trajectory");
		//Save the trajectory response
		traj = traj_srv.response.trajectory;
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

}
};


