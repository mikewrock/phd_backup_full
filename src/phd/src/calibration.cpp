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

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  
ROS_INFO("GOT A CALIBRATION CLOUD!");
sensor_msgs::PointCloud2 cloud_msg2 = *cloud_msg;


	ROS_INFO("size %d pts", cloud_msg2.width);
//fix the naming discrepancy between ROS and PCL (from "intensities" to "intensity")
cloud_msg2.fields[3].name = "intensity";
//Create a PCL pointcloud
//Populate the PCL pointcloud with the ROS message
pcl::fromROSMsg(cloud_msg2,*cloud);
 
   Eigen::Vector4f minPoint; 
      minPoint[0]=0;  // define minimum point x 
      minPoint[1]=-.5;  // define minimum point y 
      minPoint[2]=0;  // define minimum point z 
     Eigen::Vector4f maxPoint; 
      maxPoint[0]=.7;  // define max point x 
      maxPoint[1]=.5;  // define max point y 
      maxPoint[2]=1;  // define max point z 

     Eigen::Vector3f boxTranslatation; 
      boxTranslatation[0]=0;   
      boxTranslatation[1]=0;   
      boxTranslatation[2]=0;   
   // this moves your cube from (0,0,0)//minPoint to (1,2,3)  // maxPoint is now(6,8,10) 

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

   	cropFilter.filter (*cloudOut); 


	sensor_msgs::PointCloud2 line_cloud2;
	pcl::toROSMsg(*cloudOut,line_cloud2);
	ROS_INFO("Publishing %d pts", line_cloud2.width);
	line_cloud2.header.frame_id = "/world";
	line_cloud2.header.stamp = ros::Time::now();
	line_pub.publish(line_cloud2);
//Create containers for the filtered cloud
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_intensity_filtered (new pcl::PointCloud<pcl::PointXYZI>);
	// Create the filtering object
	pcl::PassThrough<pcl::PointXYZI> pass (true);
	//filter the data for intensity
	pass.setInputCloud (cloudOut);
	pass.setFilterFieldName ("intensity");
	pass.setFilterLimits (800,2000);
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
	ROS_INFO("Location: %f, %f, %f",xs,ys,zs);

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




