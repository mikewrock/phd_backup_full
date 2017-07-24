#include <ros/ros.h>
// PCL specific includes
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
#include <sensor_msgs/JointState.h>  
#include <pcl/octree/octree.h>
#include <vector>
#include <ctime>
#include <boost/make_shared.hpp>
#include <pcl/point_representation.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

#define DEBUG 1

//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

  ros::Publisher pub1;

  pcl::PointCloud<pcl::PointXYZI>::Ptr marker (new pcl::PointCloud<pcl::PointXYZI> );
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudB (new pcl::PointCloud<pcl::PointXYZI> );
  //make the publisher global so we can publish from the callback

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "marker_node");
  ros::NodeHandle nh;

pub1 = nh.advertise<sensor_msgs::PointCloud2> ("assembled_cloud", 1);

    if (pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/mike/mine.pcd", *marker) == -1) 
    {
      PCL_ERROR ("Couldn't find marker file \n");

      return(0);
	}

   // Generate a ros point cloud message with the selected points in rviz
    sensor_msgs::PointCloud2 selected_points_ros;
  pcl::toROSMsg(*marker, selected_points_ros);
    selected_points_ros.header.frame_id = "base_link";
  selected_points_ros.header.stamp = ros::Time::now();
    selected_points_ros.fields.resize( 5 );
    selected_points_ros.fields[3].name = "intensities"; 
selected_points_ros.fields[4].name = "index";
    selected_points_ros.fields[4].offset = 16;
    selected_points_ros.fields[4].datatype = sensor_msgs::PointField::FLOAT32;
    selected_points_ros.fields[4].count = 1;
    ROS_INFO("publishing ");
ros::Duration duration(1);
    duration.sleep();
  pub1.publish(selected_points_ros);

//publish marker in marker frame
//periodically publish tf from marker to map/base_link?
ros::spin();

}
