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
#include <visualization_msgs/Marker.h>
#include <pcl/filters/normal_space.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>


#define DEBUG 1

//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;


  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudA (new pcl::PointCloud<pcl::PointXYZI> );
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudB (new pcl::PointCloud<pcl::PointXYZI> );
  //make the publisher global so we can publish from the callback
  ros::Publisher pub;
 ros::Publisher marker_pub;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
 ROS_INFO("GOT A CLOUD");
  pcl::PointCloud<pcl::PointXYZI> cloudB;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  sensor_msgs::PointCloud2 cloud_msg2 = *cloud_msg;
  //fix the naming discrepancy between ROS and PCL (from "intensities" to "intensity")
  cloud_msg2.fields[3].name = "intensity";
  pcl::fromROSMsg(cloud_msg2,*cloud);
float maxX =0;
  float maxY =0;
  float maxZ =0;  
float minX =100;
  float minY =100;
  float minZ =100;  
float minzX,minzY;
  BOOST_FOREACH (pcl::PointXYZI& p, cloud->points){

    if(p.z < minZ){ 
      minZ = p.z;
      }
    if(p.z > maxZ){ 
      maxZ = p.z;
      }
    if(p.x > maxX){ 
      maxX = p.x;
      }
    if(p.y > maxY){ 
      maxY = p.y;
      }    
    if(p.x < minX){ 
      minX = p.x;
      }
    if(p.y < minY){ 
      minY = p.y;
      }

 }
  
  ROS_INFO("MAX: %f -- %f -- %f",minX,minY,minZ);
  
  pcl::PointXYZI start[12];
  pcl::PointXYZI foundpt[12];
float incX, incY, incZ;
incX = (maxX-minX)/4;
incY = (maxY-minY)/4;
incZ = (maxZ-minZ)/2;
    std::vector<int> pointIdxNKNSearch;
  std::vector<float> pointNKNSquaredDistance;
    pcl::PointXYZI searchPoint;
    int K = 1;
    float resolution = 128.0f;

  pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> octree (resolution);

  octree.setInputCloud (cloud);
  octree.addPointsFromInputCloud ();
  start[0].x = minX;
  start[0].y = minY;
  start[0].z = minZ;
  start[1].x = start[0].x+incX;
  start[1].y = start[0].y+incY;
  start[1].z = start[0].z;
  start[2].x = start[1].x+incX;
  start[2].y = start[1].y+incY;
  start[2].z = start[1].z;
  start[3].x = start[2].x+incX;
  start[3].y = start[2].y+incY;
  start[3].z = start[2].z;
  start[4].x = minX;
  start[4].y = minY;
  start[4].z = minZ+incZ;
  start[5].x = start[4].x+incX;
  start[5].y = start[4].y+incY;
  start[5].z = start[4].z;
  start[6].x = start[5].x+incX;
  start[6].y = start[5].y+incY;
  start[6].z = start[5].z;
  start[7].x = start[6].x+incX;
  start[7].y = start[6].y+incY;
  start[7].z = start[6].z;
  start[8].x = minX;
  start[8].y = minY;
  start[8].z = minZ+(2*incZ);
  start[9].x = start[8].x+incX;
  start[9].y = start[8].y+incY;
  start[9].z = start[8].z;
  start[10].x = start[9].x+incX;
  start[10].y = start[9].y+incY;
  start[10].z = start[9].z;
  start[11].x = start[10].x+incX;
  start[11].y = start[10].y+incY;
  start[11].z = start[10].z;
  
  
      
    
    uint32_t shape = visualization_msgs::Marker::ARROW;
     visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/base_link";
    marker.header.stamp = ros::Time::now();
    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;
    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;
    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;
    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(100);
    
  for(int ctr = 0; ctr < 12; ++ctr){
    if (octree.nearestKSearch (start[ctr], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0){
        foundpt[ctr] = cloud->points[pointIdxNKNSearch[0]];
        ROS_INFO("Found pt %f/%f/%f near %f/%f/%f",foundpt[ctr].x,foundpt[ctr].y,foundpt[ctr].z,start[ctr].x,start[ctr].y,start[ctr].z);
        }
        
     cloudB.push_back (foundpt[ctr]);
     }
        
    ROS_INFO("Size: %d",cloudB.width*cloudB.height);
    // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
    // set parameters
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudptr = cloudB.makeShared();
  ne.setInputCloud (cloudptr);

    ROS_INFO("Size2: %d -- %lu",cloudptr->width*cloudptr->height, cloudptr->points.size());
  // Pass the original data (before downsampling) as the search surface
  ne.setSearchSurface (cloud);
  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given surface dataset.
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI> ());
  ne.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal> ());

  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (0.05);
  ne.setViewPoint (-10, 10, 10);
  // Compute the features
  ne.compute (*cloud_normals);
  
  ROS_INFO("%lu",cloud_normals->points.size());
  

      marker.points.resize(2);
      
  for(int ctr = 0; ctr < 12; ++ctr){

     marker.id = ctr;
     marker.points[0].x = foundpt[ctr].x;
     marker.points[0].y = foundpt[ctr].y;
     marker.points[0].z = foundpt[ctr].z;
     marker.points[1].x = foundpt[ctr].x + cloud_normals->points[ctr].normal[0]/10;
     marker.points[1].y = foundpt[ctr].y + cloud_normals->points[ctr].normal[1]/10;
     marker.points[1].z = foundpt[ctr].z + cloud_normals->points[ctr].normal[2]/10;
     ROS_INFO("Found pt %f/%f/%f normal %f/%f/%f",foundpt[ctr].x,foundpt[ctr].y,foundpt[ctr].z,cloud_normals->points[ctr].normal[0],cloud_normals->points[ctr].normal[1],cloud_normals->points[ctr].normal[2]);
        ROS_INFO("%lu",cloud_normals->points.size ());
     marker_pub.publish(marker);
     }
      
    
     
std::cout << "Saving Cloud" << std::endl;
    sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*cloud, output);
  output.header.frame_id = "base_link";
    pub.publish(output);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "trajectory_generator");
  ros::NodeHandle nh;

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("assembled_cloud", 1);
  marker_pub = nh.advertise<visualization_msgs::Marker>("shotcrete_marker", 1);
  ros::Subscriber sub = nh.subscribe ("shotcrete_selected_points", 1, cloud_cb);

ros::spin();
  
  
}
