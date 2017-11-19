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
//User defined messages
#include "phd/trajectory_service.h"
#include "phd/trajectory_array.h"
#include "phd/trajectory_section.h"

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

pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);

//finds nearest point to given coords
pcl::PointXYZI find_pt(pcl::PointXYZI target_pt){	
	int K = 1; //Number of points to find
	std::vector<int> pointIdxNKNSearch; //vector to hold points (used by pcl)
	std::vector<float> pointNKNSquaredDistance; //vector to hold distance of points (used by pcl)
	float resolution = 128.0f; //Octree resolution
	pcl::PointXYZI foundpt; //Container for found point (to return)

ROS_INFO("created paracms");
	//Create the octree object
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> octree (resolution);
	//Set the octree input cloud, then add the points
	octree.setInputCloud (cloud);
	octree.addPointsFromInputCloud ();
	//Search for the nearest 1 neighbour
	if (octree.nearestKSearch (target_pt, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0){
		foundpt = cloud->points[pointIdxNKNSearch[0]];
	}else ROS_INFO("Could not find point");
	return foundpt;

}


//find normal at given location
pcl::PointXYZI find_normal(float x, float y, float z){
	//The search method requires a pcl point	
	pcl::PointXYZI target_pt;
	target_pt.x = x;
	target_pt.y = y;
	target_pt.z = z;
		
ROS_INFO("created params");
	pcl::PointXYZI search_pt = find_pt(target_pt);
	//vonatiner for return value
	pcl::PointXYZI foundpt;
	//pcl wants a cloud of points at which to calculate normals
	pcl::PointCloud<pcl::PointXYZI> cloudB;
	cloudB.push_back (search_pt);	
	// Create the normal estimation class
	pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
	// set the input for our normal estimation cloud
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloudptr = cloudB.makeShared();
	ne.setInputCloud (cloudptr);
	// Pass the original data (before downsampling) as the search surface
	ne.setSearchSurface (cloud);
	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given surface dataset.
	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI> ());
	ne.setSearchMethod (tree);

	// Output datasets
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal> ());

	// Use all neighbors in a sphere of radius 5cm
	ne.setRadiusSearch (0.1);
	ne.setViewPoint (0, 0, 0); //Which direction to point the notmal
	// Compute the features
	ne.compute (*cloud_normals);
	//since we want the normal to point inwards, but we can set the viewpoint at infinity, flip the result
	foundpt.x = -cloud_normals->points[0].normal[0];
	foundpt.y = -cloud_normals->points[0].normal[1];
	foundpt.z = -cloud_normals->points[0].normal[2];
	//ROS_INFO("Normal %f %f %f",foundpt.x,foundpt.y,foundpt.z);
	return foundpt;

}

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  
ROS_INFO("GOT A CLOUD!");
sensor_msgs::PointCloud2 cloud_msg2 = *cloud_msg;

ROS_INFO("created params");
//fix the naming discrepancy between ROS and PCL (from "intensities" to "intensity")
cloud_msg2.fields[3].name = "intensity";
//Create a PCL pointcloud
//Populate the PCL pointcloud with the ROS message
pcl::fromROSMsg(cloud_msg2,*cloud);
 
   Eigen::Vector4f minPoint; 
      minPoint[0]=0;  // define minimum point x 
      minPoint[1]=0;  // define minimum point y 
      minPoint[2]=0;  // define minimum point z 
     Eigen::Vector4f maxPoint; 
      maxPoint[0]=10;  // define max point x 
      maxPoint[1]=10;  // define max point y 
      maxPoint[2]=.05;  // define max point z 

     Eigen::Vector3f boxTranslatation; 
      boxTranslatation[0]=2.4;   
      boxTranslatation[1]=.13;   
      boxTranslatation[2]=.6;   
   // this moves your cube from (0,0,0)//minPoint to (1,2,3)  // maxPoint is now(6,8,10) 

pcl::PointXYZI rot = find_normal(2.4,.13,.6);
ROS_INFO("Normal size %f", sqrt(pow(rot.x,2)+pow(rot.y,2)+pow(rot.z,2)));
     Eigen::Vector3f boxRotation; 
      boxRotation[0]=acos(rot.x);  // rotation around x-axis 
      boxRotation[1]=acos(rot.y);  // rotation around y-axis 
      boxRotation[2]=acos(rot.z);  //in radians rotation around z-axis. this rotates your cube 45deg around z-axis. 

pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOut (new pcl::PointCloud<pcl::PointXYZI>); 

               pcl::CropBox<pcl::PointXYZI> cropFilter; 
        cropFilter.setInputCloud (cloud); 
               cropFilter.setMin(minPoint); 
               cropFilter.setMax(maxPoint); 
               cropFilter.setTranslation(boxTranslatation); 
               cropFilter.setRotation(boxRotation); 

   	cropFilter.filter (*cloudOut); 


	sensor_msgs::PointCloud2 line_cloud2;
	pcl::toROSMsg(*cloudOut,line_cloud2);
	ROS_INFO("Publishing %d pts", line_cloud2.width);
	line_cloud2.header.frame_id = "/world";
	line_cloud2.header.stamp = ros::Time::now();
	line_pub.publish(line_cloud2);


}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cube_move");

    ros::NodeHandle nh;
	marker_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
	line_pub = nh.advertise<sensor_msgs::PointCloud2>( "line_points", 0 );
	line_pub2 = nh.advertise<sensor_msgs::PointCloud2>( "line_points2", 0 );
	path_pub = nh.advertise<visualization_msgs::Marker>( "path_marker", 0 );
	dir_pub = nh.advertise<visualization_msgs::Marker>( "dir_marker", 0 );
	ROS_INFO("Trajectory Generator online");
	nsid = 0;  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("raw_cloud", 1, cloud_cb);
	ros::spin();  
}




