#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <pcl/conversions.h>
#include "pcl_ros/transforms.h"
#include <iostream>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include "std_msgs/String.h"
#include <sstream>
#include <pcl/filters/extract_indices.h>
#include <pcl/octree/octree.h>
#include <vector>
#include <algorithm>
#include <ctime>
#include <boost/make_shared.hpp>
#include <pcl/point_representation.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <visualization_msgs/Marker.h>
#include <pcl/filters/normal_space.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/crop_box.h>
#include <boost/foreach.hpp>

#define DEBUG 1 //For displaying debug info
#define WORKSPACE 0.5 //Robot workspace width in m
#define HEIGHT_STEP .2 //Increment for each horizontal line
#define MAX_HEIGHT 1.3 //Maximum reachable height
#define OFFSET 0.1 //Optimal distance from wall to end effector
#define Z_HEIGHT 0.03 //Thickness of line slice
#define VIA_DISTANCE 0.05 //Distance between via points
#define NORM_ANGLE .1 //Maximum angle before adding additiona via points
#define PLANE_TOLERANCE .06 //Maximum distance from plane to keep for line points

//convenient typedefs **are these used?**
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_surface (new pcl::PointCloud<pcl::PointXYZI>); //The surface to be shotcreted
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_surface_full (new pcl::PointCloud<pcl::PointXYZI>); //The entire surface that was scanned (useful for normals athe cloud_surface edges)

//make the publishers global so we can publish from callbacks
ros::Publisher marker_pub; //Publishes normals
ros::Publisher dir_pub; //Path along the surface
ros::Publisher path_pub; //End effector path
ros::Publisher line_pub; //For debugging, publishes the cropped line that the via points are calculated on
ros::Publisher line_pub2; //For debugging, publishes the cropped line that the via points are calculated on
int nsid;


ofstream myfile;
int main(int argc, char** argv)
{
  ros::init(argc, argv, "cube_move");

    ros::NodeHandle nh;
/*
	marker_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
	line_pub = nh.advertise<sensor_msgs::PointCloud2>( "line_points", 0 );
	line_pub2 = nh.advertise<sensor_msgs::PointCloud2>( "line_points2", 0 );
	path_pub = nh.advertise<visualization_msgs::Marker>( "path_marker", 0 );
	dir_pub = nh.advertise<visualization_msgs::Marker>( "dir_marker", 0 );
	ROS_INFO("Trajectory Generator online");
	nsid = 0;  // Create a ROS subscriber for the input point cloud
if(argc < 2){
printf("usage: rosrun phd test_node \"filename\" \"source\" to save to \\home\\mike\\results\\saved\\filenamestatistics.csv and load  \\home\\mike\\results\\saved\\source.pcd");
return(0);
}
	std::stringstream fs;
	fs << "/home/mike/results/saved/" << argv[1] << "statistics.csv";
	myfile.open (fs.str().c_str(), std::ios::out|std::ios::app);
	std::stringstream ss;
	ss << "/home/mike/results/saved/" << argv[2] << ".pcd";

	//Create a pcl pointer to load the clound in to
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI> );
	//Load the file
	if (pcl::io::loadPCDFile<pcl::PointXYZI> (ss.str().c_str(), *cloud) == -1) PCL_ERROR ("Couldn't read file\n");

	BOOST_FOREACH (pcl::PointXYZI& pt, cloud->points){

		myfile << pt.intensity << "," << sqrt(pow(pt.x,2) + pow(pt.y,2) + pow(pt.z,2)) << std::endl;


	}

	myfile.close();*/

tf::Quaternion q_orig, q_rot, q_new;
double r=0, p=0, y=0;  // Rotate the previous pose by 180* about X
if(argc > 2){
r = atof(argv[1]);
p = atof(argv[2]);
y = atof(argv[3]);
}
q_rot = tf::createQuaternionFromRPY(r, p, y);

tfScalar yaw, pitch, roll;
tf::Matrix3x3 mat(q_rot);
mat.getEulerZYX(yaw, pitch, roll);

ROS_INFO("Z: %f Y: %f X: %f",yaw,pitch,roll);
}




