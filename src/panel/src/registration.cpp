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

#define DEBUG 0
 int cloudctr = 0;
//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;


  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudA (new pcl::PointCloud<pcl::PointXYZI> );
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudB (new pcl::PointCloud<pcl::PointXYZI> );
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




void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  ROS_INFO("GOT A CLOUD");
  sensor_msgs::PointCloud2 cloud_msg2 = *cloud_msg;
  //fix the naming discrepancy between ROS and PCL (from "intensities" to "intensity")
  cloud_msg2.fields[3].name = "intensity";
  if(cloudctr==0){
    //Populate the PCL pointcloud with the ROS message
    pcl::fromROSMsg(cloud_msg2,*cloudA);
    ++cloudctr;
  }else{
    //Populate the PCL pointcloud with the ROS message
    pcl::fromROSMsg(cloud_msg2,*cloudB);
    ++cloudctr;
  }
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  ros::Publisher pub;
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("assembled_cloud", 1, cloud_cb);

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudA_filtered (new pcl::PointCloud<pcl::PointXYZI> );
  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("laser_trans", 1);
  if(DEBUG){ 
  //* load a file
    if (pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/mike/newcloud1.pcd", *cloudA) == -1) 
    {
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");

      return(0);
    }
  }
  
 
  
// Create the filtering object
  pcl::PassThrough<pcl::PointXYZI> pass;
  pass.setInputCloud (cloudA);
  pass.setFilterFieldName ("intensity");
  pass.setFilterLimits (800, 1500);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloudA_filtered);

pcl::io::savePCDFileASCII ("/home/mike/filterA.pcd", *cloudA_filtered);

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudB (new pcl::PointCloud<pcl::PointXYZI> );
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudB_filtered (new pcl::PointCloud<pcl::PointXYZI> );
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudB_aligned (new pcl::PointCloud<pcl::PointXYZI> );
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudB_downsampled (new pcl::PointCloud<pcl::PointXYZI> );
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudA_downsampled (new    pcl::PointCloud<pcl::PointXYZI> );
   
  
  if (pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/mike/newcloud3.pcd", *cloudB) == -1) 
  //* load a file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");

return(0);
  }

  pass.setInputCloud (cloudB);
  pass.setFilterFieldName ("intensity");
  pass.setFilterLimits (800, 1500);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloudB_filtered);

pcl::io::savePCDFileASCII ("/home/mike/filterB.pcd", *cloudB_filtered);
	PointCloud::Ptr result (new PointCloud), source (new PointCloud), target (new PointCloud);
  Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity (), pairTransform;

  // Fill in the cloud data
  source->width  = cloudA_filtered->width;
  source->height = cloudA_filtered->height;
  source->points.resize (cloudA_filtered->width * cloudA_filtered->height);
std::cout << "Done Resizing" << std::endl;
  for (size_t i = 0; i < source->points.size (); ++i)
  {
    source->points[i].x = cloudA_filtered->points[i].x;
    source->points[i].y = cloudA_filtered->points[i].y;
    source->points[i].z = cloudA_filtered->points[i].z;
  }

  // Fill in the cloud data
  target->width  = cloudB_filtered->width;
  target->height = cloudB_filtered->height;
  target->points.resize (cloudB_filtered->width * cloudB_filtered->height);

  for (size_t i = 0; i < target->points.size (); ++i)
  {
    target->points[i].x = cloudB_filtered->points[i].x;
    target->points[i].y = cloudB_filtered->points[i].y;
    target->points[i].z = cloudB_filtered->points[i].z;
  }


    PointCloud::Ptr temp (new PointCloud);
    pairAlign (source, target, temp, pairTransform, false);
std::cout << "Done Aligning" << std::endl;
  pcl::transformPointCloud (*cloudB, *cloudB_aligned, pairTransform);

pcl::io::savePCDFileASCII ("/home/mike/newcloudaligned.pcd", *cloudB_aligned);
  

// Create the filtering object
  pcl::VoxelGrid<pcl::PointXYZI> sor;
  sor.setInputCloud (cloudA);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*cloudA_downsampled);


  std::cerr << "PointCloud after filtering: " << cloudA_downsampled->width * cloudA_downsampled->height 
       << " data points (" << pcl::getFieldsList (*cloudA_downsampled) << ").";
float resolution = 128.0f;

  pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> octree (resolution);

  octree.setInputCloud (cloudA_downsampled);
  octree.addPointsFromInputCloud ();

  pcl::PointXYZI searchPoint;

  // Neighbors within radius search

  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

  float radius = .5;
float mind = 100;
float maxd = 0;
int K = 1;

  std::vector<int> pointIdxNKNSearch;
  std::vector<float> pointNKNSquaredDistance;
 sor.setInputCloud (cloudB_aligned);
  sor.setLeafSize (0.02f, 0.02f, 0.02f);
  sor.filter (*cloudB_downsampled);

BOOST_FOREACH (pcl::PointXYZI& pt, cloudB_downsampled->points){
  searchPoint.x = pt.x;
  searchPoint.y = pt.y;
  searchPoint.z = pt.z;
 
/*
  if (octree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
  {
	pt.intensity = pointRadiusSquaredDistance[0];

    for (int i = 0; i < pointIdxRadiusSearch.size (); ++i){
	if(mind >  pointRadiusSquaredDistance[i]) mind = pointRadiusSquaredDistance[i];  
	if(maxd <  pointRadiusSquaredDistance[i]) maxd = pointRadiusSquaredDistance[i];  
	if(pt.intensity <  pointRadiusSquaredDistance[i]) pt.intensity = pointRadiusSquaredDistance[0];     
  }
}*/



  if (octree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
  {
pt.intensity = pointNKNSquaredDistance[0];
     }

}

pcl::io::savePCDFileASCII ("/home/mike/newcoloured.pcd", *cloudB_downsampled);
   sensor_msgs::PointCloud2 output;
output.header.frame_id = "base_footprint";
  pcl::toROSMsg(*cloudB_aligned, output);
  // Publish the data
  pub.publish (output);
ros::spinOnce();
std::cout << "Done" << std::endl;




}
