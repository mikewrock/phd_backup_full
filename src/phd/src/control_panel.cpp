// Always goes first
#define _CUSTOM_UINT64
// ROS
#include <ros/ros.h>

#include "../include/control_panel.hpp"
//#include <atlbase.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>  

#include <phd/cube_msg.h>
#include <cstdio>
// Services
#include "laser_assembler/AssembleScans2.h"
#include "phd/localize_cloud.h"
// Messages
#include "sensor_msgs/PointCloud.h"

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>

#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/PointCloud2.h>
#include <QVariant>
#include <pcl_conversions/pcl_conversions.h>
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
#include <pcl/registration/ia_ransac.h>
#include <visualization_msgs/Marker.h>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/common/angles.h>
#include <iostream>
#include <std_msgs/Float64.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>


#include "phd/trajectory_service.h"
#include <phd/arm_msg.h>

#define CALC true

namespace control_panel{
namespace control_panel_ns{


visualization_msgs::Marker point_list;
        double joint1;
laser_assembler::AssembleScans2 srv;
phd::localize_cloud loc_srv;
phd::trajectory_service traj_srv;

//DELETE
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_aligner (new pcl::PointCloud<pcl::PointXYZI> );

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}
void QNode::jointCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	std::string str1 = "cube_5_joint";
	  std::string str2 =msg->name[0] ;
	if(str1==str2){
	  joint1 = msg->position[0];
	}
}
void QNode::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud)
{

	current_pc_.reset(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::fromROSMsg(*cloud, *this->current_pc_);
	geometry_msgs::Point p;
	p.x = current_pc_->points[0].data[0];
	p.y = current_pc_->points[0].data[1];
	p.z = current_pc_->points[0].data[2];
	point_list.points.push_back(p);
	//vis_pub.publish(point_list);
}

bool QNode::init() {
	ROS_INFO( "Control Panel initializing");
	int fargc = 0;
	char** fargv = (char**)malloc(sizeof(char*)*(fargc+1));
	ros::init(fargc,fargv,"ControlPanelNode");
	free(fargv);
	ros::start();
	cmd_pub = nh_.advertise<phd::cube_msg>("joint_cmd", 100);
	arm_pub = nh_.advertise<phd::arm_msg>("arm_cmd", 100);
	pub = nh_.advertise<sensor_msgs::PointCloud2> ("assembled_cloud", 1);
	pub2 = nh_.advertise<sensor_msgs::PointCloud2> ("aligned_cloud", 1);
	pub3 = nh_.advertise<sensor_msgs::PointCloud2> ("aligned_cloud3", 1);
	pub4 = nh_.advertise<sensor_msgs::PointCloud2> ("aligned_cloud4", 1);
	pub5 = nh_.advertise<sensor_msgs::PointCloud2> ("aligned_cloud5", 1);
	joint_sub = nh_.subscribe("cube_joint_states", 1, &control_panel::control_panel_ns::QNode::jointCallback,this);
	cloud_sub = nh_.subscribe("marker_selected_points", 1, &control_panel::control_panel_ns::QNode::cloudCallback,this);
	traj_sub = nh_.subscribe("trajectory_points", 1, &control_panel::control_panel_ns::QNode::cloudCallback,this);
    //service client for calling the assembler
	client = nh_.serviceClient<laser_assembler::AssembleScans2>("assemble_scans2");
	loc_client = nh_.serviceClient<phd::localize_cloud>("localize_pcd");
	traj_client = nh_.serviceClient<phd::trajectory_service>("trajectory_gen");
	nav_vis_pub = nh_.advertise<visualization_msgs::Marker>( "nav_vis_goal", 0 );
	nav_pub = nh_.advertise<geometry_msgs::PoseStamped>( "nav_goal", 0 );
	point_list.header.frame_id = "/base_link";
	point_list.header.stamp = ros::Time::now();
	point_list.ns = "points_and_lines";
	point_list.action = visualization_msgs::Marker::ADD;
	point_list.pose.orientation.w = 1.0;
	point_list.id = 0;
	point_list.type = visualization_msgs::Marker::POINTS;
	point_list.scale.x = 0.1;
	point_list.scale.y = 0.1;
	// Line list is red
	point_list.color.r = 1.0;
	point_list.color.a = 1;

	sec_ctr = 0;
	pt_ctr = 0;

	ros::start();
}

phd::trajectory_point QNode::find_pose(void){
	phd::trajectory_point ret;
	ret.x = 0;
	ret.y = 0;
	ret.z = 0;
	ret.nx = 0;
	ret.ny = 0;
	ret.nz = 0;
	ret.d = 0;
	int avg_ctr = 0;
ROS_INFO("start Point %f - %f",traj.sections[sec_ctr].points[0].x,traj.sections[sec_ctr].points[0].y);
	for(int ctr = 0; ctr < traj.sections[sec_ctr].points.size(); ++ctr){
		if(traj.sections[sec_ctr].points[ctr].z - traj.sections[sec_ctr].points[0].z < 0.02){
			ret.nx += traj.sections[sec_ctr].points[ctr].nx;
			ret.ny += traj.sections[sec_ctr].points[ctr].ny;
			avg_ctr++;
		}
		if(traj.sections[sec_ctr].points[ctr].d>ret.d){
			ret.x = traj.sections[sec_ctr].points[ctr].x;
			ret.y = traj.sections[sec_ctr].points[ctr].y;
			ret.d = traj.sections[sec_ctr].points[ctr].d;
		}	
	}
	ret.nx = -ret.nx/avg_ctr;
	ret.ny = -ret.ny/avg_ctr;
	ret.x = traj.sections[sec_ctr].points[0].x+((ret.x - traj.sections[sec_ctr].points[0].x)/2);// + 0.5*ret.nx;
	ret.y = traj.sections[sec_ctr].points[0].y+((ret.y - traj.sections[sec_ctr].points[0].y)/2);// + 0.5*ret.ny;
	ret.z = 0;
	ROS_INFO("Returnin %f - %f/ %f - %f",ret.x,ret.y,ret.nx,ret.ny);
	return ret;
}

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
	tp = find_pose();
	p.x = tp.x+0.2*tp.nx;
	p.y = tp.y+0.2*tp.ny;
	p.z = 0;
	nav_goal.points.push_back(p);
	p.x -= 0.2*tp.ny;
	p.y += 0.2*tp.nx;
	nav_goal.points.push_back(p);
	nav_vis_pub.publish(nav_goal);

}
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
void QNode::run() {
	std::cout << "Control Panel Running" << std::endl;
	
	while ( ros::ok() ) {
		ros::spinOnce();
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;


	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::step(){

	phd::arm_msg msg;
	msg.x = traj.sections[sec_ctr].points[pt_ctr].x;
	msg.y = traj.sections[sec_ctr].points[pt_ctr].y;
	msg.z = traj.sections[sec_ctr].points[pt_ctr].z;
	float length = sqrt(pow(traj.sections[sec_ctr].points[pt_ctr].nx,2)+pow(traj.sections[sec_ctr].points[pt_ctr].ny,2)+pow(traj.sections[sec_ctr].points[pt_ctr].nz,2));
	msg.rx = asin(traj.sections[sec_ctr].points[pt_ctr].ny / length);
	if(cos(msg.rx)!=0) msg.ry = asin( traj.sections[sec_ctr].points[pt_ctr].nx / (cos(msg.rx)*length) );
	msg.rz = 0;
	msg.fig = 1;
	++pt_ctr;
	if(pt_ctr >= traj.sections[sec_ctr].points.size()){
		++sec_ctr;
		pt_ctr = 0;
	}
	arm_pub.publish(msg);

}

void QNode::scan(){

	phd::cube_msg cube_cmd;
		joint1 = 1;
	//ROS_INFO("Waiting for j1");
	//while((joint1 < -.8 || joint1 > .77) && ros::ok()){
	//ros::spinOnce();
	//printf("j1: %f\n",joint1);
	//} 
	cube_cmd.j1 = -2;
	cube_cmd.j2 = 0;
	cube_cmd.j3 = 0;
	cube_cmd.j4 = 0;
	cube_cmd.j5 = 0;
	cube_cmd.vel = 0.5;
	cube_cmd.acc = 2;
	cube_cmd.pose = true;
		//send the joint state and transform
	    // Populate our service request based on our timer callback times
	cmd_pub.publish(cube_cmd);
	ros::spinOnce();
	printf("j1: %f\n",joint1);
	ROS_INFO("going to start");
	while(joint1 > -1.9 && ros::ok()){
	ros::spinOnce();
	//printf("j1is: %f\n",joint1);
	} 
	 srv.request.begin = ros::Time::now();
	cube_cmd.j1 = 0;

	cube_cmd.vel = 0.25;
	cmd_pub.publish(cube_cmd);
	ros::spinOnce();
	ROS_INFO("waiting for end");
	while(joint1 < -.01 && ros::ok()){
	ros::spinOnce();
	} 
	ROS_INFO("done");
	 srv.request.end   = ros::Time::now();
	ros::spinOnce();
	    // Make the service call
	    if (client.call(srv))
	    {
	      ROS_INFO("Published Cloud %d", (uint32_t)(srv.response.cloud.width)) ;
			cloud_surface = srv.response.cloud;
			cloud_surface.fields[3].name = "intensities";
	      pub.publish(cloud_surface);
 ROS_INFO("Published Cloud");
	    }
	    else
	    {
	      ROS_ERROR("Error making service call\n") ;
	    }

}


void QNode::nav_mode(float pos){

phd::cube_msg cube_cmd;
		
	cube_cmd.j1 = pos;
	cube_cmd.j2 = 0;
	cube_cmd.j3 = 0;
	cube_cmd.j4 = 0;
	cube_cmd.j5 = 0;
	cube_cmd.vel = 0.5;
	cube_cmd.acc = 2;
	cube_cmd.pose = true;
		//send the joint state and transform
	    // Populate our service request based on our timer callback times
	cmd_pub.publish(cube_cmd);
	ros::spinOnce();

}


void QNode::fscan(int file){

pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI> );

if(file == 2){
if (pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/mike/marker/refined/dotsf4.pcd", *cloud) == -1) 
{
	PCL_ERROR ("Couldn't read file test_pcd.pcd \n");

}else 
std::cerr << "Sending cloud with: " << cloud->width * cloud->height << " data points." << std::endl;
}

if(file == 3){
if (pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/mike/marker/refined/dotsf5.pcd", *cloud) == -1) 
{
	PCL_ERROR ("Couldn't read file test_pcd.pcd \n");

}else 
std::cerr << "Sending cloud with: " << cloud->width * cloud->height << " data points." << std::endl;
}
if(file == 4){
if (pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/mike/marker/refined/dotsf3.pcd", *cloud_aligner) == -1) 
{
	PCL_ERROR ("Couldn't read file test_pcd.pcd \n");

}else {
//std::cerr << "Sending cloud with: " << cloud->width * cloud->height << " data points." << std::endl;
sensor_msgs::PointCloud2 cloud_msg;
	pcl::toROSMsg(*cloud_aligner,cloud_msg);
	//fix the naming discrepancy between ROS and PCL (from "intensities" to "intensity")
	cloud_msg.fields[3].name = "intensities";

	cloud_msg.header.frame_id = "/base_link";
		      pub.publish(cloud_msg);
	ros::spinOnce();
return;
}
}

sensor_msgs::PointCloud2 cloud_msg;
	pcl::toROSMsg(*cloud,cloud_msg);
	//fix the naming discrepancy between ROS and PCL (from "intensities" to "intensity")
	cloud_msg.fields[3].name = "intensities";

	cloud_msg.header.frame_id = "/base_link";
loc_srv.request.cloud_in = cloud_msg;
loc_srv.request.homing = false;
loc_client.call(loc_srv);
	if(loc_client.call(loc_srv)){
	ROS_INFO("Received Cloud %d", (uint32_t)(loc_srv.response.cloud_out.width));
		      pub.publish(loc_srv.response.cloud_out);
			cloud_surface = loc_srv.response.cloud_out;
	}else ROS_INFO("Service Failed");

}
void QNode::lscan(){

	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI> );


	if (pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/mike/marker/refined/dotsf3.pcd", *cloud) == -1) 
	{
		PCL_ERROR ("Couldn't read file test_pcd.pcd \n");

	}else 
	std::cerr << "Sending cloud with: " << cloud->width * cloud->height << " data points." << std::endl;

	sensor_msgs::PointCloud2 cloud_msg;
		pcl::toROSMsg(*cloud,cloud_msg);
		//fix the naming discrepancy between ROS and PCL (from "intensities" to "intensity")
		cloud_msg.fields[3].name = "intensities";

	cloud_msg.header.frame_id = "/base_link";
	loc_srv.request.cloud_in = cloud_msg;
	loc_srv.request.homing = true;
	if(loc_client.call(loc_srv)){
	ROS_INFO("Received Cloud %d", (uint32_t)(loc_srv.response.cloud_out.width));
		      pub.publish(loc_srv.response.cloud_out);
			cloud_surface = loc_srv.response.cloud_out;
	}else ROS_INFO("Service Failed");
}
void QNode::cluster(int index){
	int init_size =  current_pc_->width * current_pc_->height;
	ROS_INFO("Clustering %d, %d points", index,init_size) ;
	//Create containers for filtered clouds
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_intensity_filtered (new pcl::PointCloud<pcl::PointXYZI>);
	// Create the filtering object
	pcl::PassThrough<pcl::PointXYZI> pass (true);
	//filter the data for intensity
	pass.setInputCloud (current_pc_);
	pass.setFilterFieldName ("intensity");
	pass.setFilterLimits (550,1100);
	pass.filter (*cloud_intensity_filtered);

	float xs=0;
	float ys=0;
	float zs=0;
	int size = cloud_intensity_filtered->width * cloud_intensity_filtered->height;
	//average all the points
	for(int i = 0; i < size; ++i){
	//ROS_INFO("Adding Point: %f - %f - %f / %d",cloud_intensity_filtered->points[i].x,cloud_intensity_filtered->points[i].y,cloud_intensity_filtered->points[i].z,i);
		xs += cloud_intensity_filtered->points[i].x;
		ys += cloud_intensity_filtered->points[i].y;
		zs += cloud_intensity_filtered->points[i].z;
	}
	xs = xs/size;
	ys = ys/size;
	zs = zs/size;
	ROS_INFO("Saving Point: %f - %f - %f / %d",xs, ys, zs, size);
	ofstream myfile;
	std::stringstream fs;
	fs << "/home/mike/marker/location/" << "filename" << ".dat";
	switch (index){
		case 0:	myfile.open (fs.str().c_str(), std::ios::out|std::ios::trunc);
			P1x = xs;
			P1y = ys;
			P1z = zs;
			break;
		case 1: myfile.open (fs.str().c_str(), std::ios::out|std::ios::app);
			P2x = xs;
			P2y = ys;
			P2z = zs;
			break;
		case 2: myfile.open (fs.str().c_str(), std::ios::out|std::ios::app);
			rosbag::Bag bag;
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

			bag.open("/home/mike/marker/fiducial.bag", rosbag::bagmode::Write);
			std_msgs::Float64 f;
			for(int fctr = 0; fctr < 4; ++fctr){
				for(int fctr2 = 0; fctr2 < 4; ++fctr2){
					f.data = A_mat(fctr2,fctr);
					bag.write("fiducial", ros::Time::now(), f);
				}
			}
			bag.close();	
	}
	myfile << xs << std::endl << ys << std::endl << zs << std::endl;
	myfile.close();

}

void QNode::gen_trajectory(){
	ROS_INFO("Generating Trajectory") ;
	traj_srv.request.P1x = T1x;
	traj_srv.request.P1y = T1y;
	traj_srv.request.P1z = T1z;
	traj_srv.request.P2x = T2x;
	traj_srv.request.P2y = T2y;
	traj_srv.request.P2z = T2z;
	traj_srv.request.P3x = T3x;
	traj_srv.request.P3y = T3y;
	traj_srv.request.P3z = T3z;
	sensor_msgs::PointCloud2 cloud_msg;
	pcl::toROSMsg(*current_pc_,cloud_msg);
	//fix the naming discrepancy between ROS and PCL (from "intensities" to "intensity")
	cloud_msg.fields[3].name = "intensities";

	cloud_msg.header.frame_id = "/base_link";
	traj_srv.request.cloud_in = cloud_msg;
	traj_srv.request.cloud_surface = cloud_surface;
	int ret = traj_client.call(traj_srv);
	if(ret){
	ROS_INFO("Received Trajectory");
	traj = traj_srv.response.trajectory;
	}else ROS_INFO("Service Failed");

		


}

void QNode::trajectory(int index){

	T1x = current_pc_->points[0].data[0];
	T1y = current_pc_->points[0].data[1];
	T1z = current_pc_->points[0].data[2];


}
void QNode::estimate(){
	setVerbosityLevel(pcl::console::L_VERBOSE); 
	pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;

	pcl::PointCloud<pcl::PointXYZI>::Ptr cloudB_downsampled (new pcl::PointCloud<pcl::PointXYZI> );
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloudA_downsampled (new    pcl::PointCloud<pcl::PointXYZI> );
	pcl::VoxelGrid<pcl::PointXYZI> sor;
	sor.setInputCloud (cloud_aligner);
	sor.setLeafSize (0.01f, 0.01f, 0.01f);
	sor.filter (*cloudA_downsampled);
	ROS_INFO("curent pc setting");
	icp.setInputSource (cloudA_downsampled);
	ROS_INFO("curent pc set");
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_tgt (new pcl::PointCloud<pcl::PointXYZI> );
	pcl::PointCloud<pcl::PointXYZI>::Ptr output (new pcl::PointCloud<pcl::PointXYZI> );
	pcl::PointCloud<pcl::PointXYZI>::Ptr output2 (new pcl::PointCloud<pcl::PointXYZI> );
	pcl::PointCloud<pcl::PointXYZI>::Ptr output4 (new pcl::PointCloud<pcl::PointXYZI> );
	pcl::PointCloud<pcl::PointXYZI>::Ptr output5 (new pcl::PointCloud<pcl::PointXYZI> );
	pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/mike/cheatwall2.pcd", *cloud_tgt);
	for (size_t i = 0; i < cloud_tgt->points.size (); ++i)
	cloud_tgt->points[i].x = cloud_tgt->points[i].x + 0.7f;
	sor.setInputCloud (cloud_tgt);
	sor.filter (*cloudB_downsampled);

	//compute normals
	//ROS_INFO("Estimating Normal at %f - %f - %f",target_pt.x,target_pt.y,target_pt.z);
	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
	// set parameters
	ne.setInputCloud (cloud_aligner);
	// Pass the original data (before downsampling) as the search surface
	ne.setSearchSurface (cloud_aligner);
	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given surface dataset.
	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI> ());
	ne.setSearchMethod (tree);

	// Output datasets
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal> ());

	// Use all neighbors in a sphere of radius 3cm
	ne.setRadiusSearch (0.05);
	ne.setViewPoint (0, 0, 0);
	// Compute the features
	if(CALC){
		ne.compute (*cloud_normals);
		pcl::io::savePCDFileASCII ("/home/mike/files/cloud_normals2.pcd", *cloud_normals);
	}else
	pcl::io::loadPCDFile<pcl::Normal> ("/home/mike/files/cloud_normals2.pcd", *cloud_normals);
	ne.setInputCloud (cloud_tgt);
	// Pass the original data (before downsampling) as the search surface
	ne.setSearchSurface (cloud_tgt);

	// Output datasets
	pcl::PointCloud<pcl::Normal>::Ptr cloud_tgt_normals (new pcl::PointCloud<pcl::Normal> ());

	if(true){
		ne.compute (*cloud_tgt_normals);


		pcl::io::savePCDFileASCII ("/home/mike/files/cloud_tgt_normals2.pcd", *cloud_tgt_normals);
	}else
	pcl::io::loadPCDFile<pcl::Normal> ("/home/mike/files/cloud_tgt_normals2.pcd", *cloud_tgt_normals);
	pcl::PointCloud<pcl::PointXYZI>::Ptr intensity_keypoints (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr intensity_keypoints_tgt (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::SUSANKeypoint<pcl::PointXYZI, pcl::PointXYZI> susan;
	susan.setNonMaxSupression (true);

	std::cout << "input "<<cloud_aligner->size ()<<" points.\n";

	susan.setInputCloud (cloud_aligner);
	//susan.setRadius (0.03f);
	susan.setNormals (cloud_normals);
	susan.setRadiusSearch (0.03f);
	if(CALC){
		susan.compute (*intensity_keypoints);

		pcl::io::savePCDFileASCII ("/home/mike/files/keypoints2.pcd", *intensity_keypoints);
	}else
	pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/mike/files/keypoints2.pcd", *intensity_keypoints);
	std::cout << "Found "<<intensity_keypoints->size ()<<" key points.\n";
	//pcl::copyPointCloud (*intensity_keypoints, keypoint_indices);


	std::cout << "input "<<cloud_tgt->size ()<<" points.\n";

	susan.setInputCloud (cloud_tgt);
	//susan.setRadius (0.03f);
	susan.setNormals (cloud_tgt_normals);
	if(true){
		susan.compute (*intensity_keypoints_tgt);

		pcl::io::savePCDFileASCII ("/home/mike/files/keypoints_tgt2.pcd", *intensity_keypoints_tgt);
	}else
	pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/mike/files/keypoints_tgt2.pcd", *intensity_keypoints_tgt);
	std::cout << "Found "<<intensity_keypoints_tgt->size ()<<" key points.\n";
	//pcl::copyPointCloud (*intensity_keypoints, keypoint_indices);

	pcl::PointCloud<pcl::FPFHSignature33>::Ptr features (new pcl::PointCloud<pcl::FPFHSignature33>);

	pcl::search::KdTree<pcl::PointXYZI>::Ptr search_method_xyz_ (new pcl::search::KdTree<pcl::PointXYZI>);

	pcl::FPFHEstimation<pcl::PointXYZI, pcl::Normal, pcl::FPFHSignature33> fpfh_est;

	ne.setInputCloud (intensity_keypoints);
	// Pass the original data (before downsampling) as the search surface
	ne.setSearchSurface (cloud_aligner);

	// Output datasets
	pcl::PointCloud<pcl::Normal>::Ptr keypoint_normals (new pcl::PointCloud<pcl::Normal> ());

	ne.compute (*keypoint_normals);
	ne.setInputCloud (intensity_keypoints_tgt);
	// Pass the original data (before downsampling) as the search surface
	ne.setSearchSurface (cloud_tgt);

	// Output datasets
	pcl::PointCloud<pcl::Normal>::Ptr keypoint_normals_tgt (new pcl::PointCloud<pcl::Normal> ());

	ne.compute (*keypoint_normals_tgt);

	fpfh_est.setInputCloud (intensity_keypoints);
	fpfh_est.setInputNormals (keypoint_normals);
	fpfh_est.setSearchMethod (search_method_xyz_);
	fpfh_est.setRadiusSearch (0.03f);
	fpfh_est.compute (*features);

	ROS_INFO("computed %lu featurs",features->size());




	pcl::PointCloud<pcl::FPFHSignature33>::Ptr features_tgt (new pcl::PointCloud<pcl::FPFHSignature33>);


	fpfh_est.setInputCloud (intensity_keypoints_tgt);
	fpfh_est.setInputNormals (keypoint_normals_tgt);
	fpfh_est.compute (*features_tgt);

	ROS_INFO("computed %lu featurs",features_tgt->size());


	pcl::SampleConsensusInitialAlignment<pcl::PointXYZI, pcl::PointXYZI, pcl::FPFHSignature33> sac_ia_;

	// Intialize the parameters in the Sample Consensus Intial Alignment (SAC-IA) algorithm
	sac_ia_.setMinSampleDistance (0.03f);
	sac_ia_.setMaxCorrespondenceDistance (.05);
	sac_ia_.setCorrespondenceRandomness (5);
	sac_ia_.setMaximumIterations (1000);


	sac_ia_.setInputTarget (intensity_keypoints);
	sac_ia_.setTargetFeatures (features);


	//sac_ia_.setInputCloud (intensity_keypoints_tgt);
	sac_ia_.setInputSource (intensity_keypoints_tgt);
	sac_ia_.setSourceFeatures (features_tgt);

	ROS_INFO("aligning");
	pcl::PointCloud<pcl::PointXYZI> registration_output;
	//Eigen::Matrix4f guess = Eigen::Matrix4f::Identity ();
	// sac_ia_.align (registration_output, guess);
	float fitness_score = 1;
	while(fitness_score > 0.0015){

		sac_ia_.align (registration_output);
		fitness_score = (float) sac_ia_.getFitnessScore (.05);
		ROS_INFO("align set fitness %f", fitness_score);
	}
	Eigen::Matrix4f final_transformation = sac_ia_.getFinalTransformation ();


	pcl::PointCloud<pcl::PointXYZI> Final;
	ROS_INFO("align set fitness %f", fitness_score);
	// icp.align (Final, guess);
	//std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
	//std::cout << icp.getFinalTransformation() << std::endl;
	// Eigen::Matrix4f T;
	Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
	//T = icp.getFinalTransformation();
	// Transform target back in source frame
	pcl::transformPointCloud (*cloud_tgt, *output2, final_transformation);
	pcl::transformPointCloud (*intensity_keypoints_tgt, *output4, final_transformation);


	//targetToSource = T.inverse();

	// pcl::transformPointCloud (*cloud_tgt, *output2, targetToSource);
	//final_transform = targetToSource;
	sensor_msgs::PointCloud2 cloud_msg;
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

}

}
};


