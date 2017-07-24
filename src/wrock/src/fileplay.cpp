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
#define X_MIN 1
#define X_MAX 100
#define Z_MIN .75
#define Z_MAX 100
#define Y_MIN -100
#define Y_MAX -1

//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;


  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudA (new pcl::PointCloud<pcl::PointXYZI> );
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudB (new pcl::PointCloud<pcl::PointXYZI> );
  //make the publisher global so we can publish from the callback
  ros::Publisher pub;
  int cloudctr;
//convenient structure to handle our pointclouds
struct PCD
{
  PointCloud::Ptr cloud;
  std::string f_name;

  PCD() : cloud (new PointCloud) {};
};


////////////////////////////////////////////////////////////////////////////////
/** \brief Align a pair of PointCloud datasets and return the result
  * \param cloud_src the source PointCloud
  * \param cloud_tgt the target PointCloud
  * \param output the resultant aligned source PointCloud
  * \param final_transform the resultant transform between source and target
  */
void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false)
{

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource (cloud_src);
  icp.setInputTarget (cloud_tgt);
  typedef pcl::registration::TransformationEstimationLM <pcl::PointXYZ, pcl::PointXYZ> te;
  boost::shared_ptr<te> teLM (new te);
  icp.setTransformationEstimation (teLM);
  icp.setMaximumIterations(100);
  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align (Final);
  std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;
  Eigen::Matrix4f T;
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
  T = icp.getFinalTransformation();
  // Get the transformation from target to source
  targetToSource = T.inverse();
  // Transform target back in source frame
  pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);
  final_transform = targetToSource;
}



void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  ROS_INFO("GOT A CLOUD");
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  sensor_msgs::PointCloud2 cloud_msg2 = *cloud_msg;
  //fix the naming discrepancy between ROS and PCL (from "intensities" to "intensity")
  cloud_msg2.fields[3].name = "intensity";
  pcl::fromROSMsg(cloud_msg2,*cloud);
  //crop the cloud
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filteredxyz (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr garbage_filterx (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr garbage_filterxy (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr garbage_filterxyz (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr final (new pcl::PointCloud<pcl::PointXYZI>);

// Create the filtering object
  pcl::PassThrough<pcl::PointXYZI> pass (true);
//filter z values  
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (-.1,1.5865);
  pass.filter (*cloud_filteredxyz);

  pass.setInputCloud (cloud_filteredxyz);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (X_MIN,X_MAX);
  pass.filter (*garbage_filterx);  

  pass.setInputCloud (cloud_filteredxyz);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (Y_MIN,Y_MAX);
  pass.filter (*garbage_filterxy); 

  pass.setInputCloud (cloud_filteredxyz);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (Z_MIN,Z_MAX);
  pass.filter (*garbage_filterxyz);  
  
*final += *garbage_filterx;
*final += *garbage_filterxy;
*final += *garbage_filterxyz;

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*final, output);
  
  //store the cloud in th respective global
    if(cloudctr==0){
    //Populate the PCL pointcloud with the ROS message
    *cloudA = *final;
    ++cloudctr;
  }else{
    //Populate the PCL pointcloud with the ROS message
    *cloudB = *final;
    ++cloudctr;
  }
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;


  
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudA_filtered (new pcl::PointCloud<pcl::PointXYZI> );
  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("assembled_cloud", 1);
 if (pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/mike/newcloud1.pcd", *cloudA) == -1) 
    {
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");

      return(0);
    }
    sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*cloudA, output);
  output.header.frame_id = "base_footprint";
    pub.publish(output);
    
  
  ros::spinOnce();
    if (pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/mike/newcloud3.pcd", *cloudB) == -1) 
    //* load a file
    {
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");

      return(0);
    }   
     sensor_msgs::PointCloud2 output2;
  pcl::toROSMsg(*cloudB, output2);
  output2.header.frame_id = "base_footprint";
    pub.publish(output2);
    
  ros::spinOnce();
  std::cout << "Done" << std::endl;
  
}
