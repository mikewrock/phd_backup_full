#define PCL_NO_PRECOMPILE
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
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
#include <pcl/octree/impl/octree_search.hpp>
#include <vector>
#include <ctime>
#include <boost/make_shared.hpp>
#include <pcl/point_representation.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <limits>
#include <fstream>
#include <vector>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/narf.h>
#include <pcl/features/narf_descriptor.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include "phd/localize_cloud.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <std_msgs/Float64.h>
#include <boost/foreach.hpp>
#include <dynamic_reconfigure/server.h>
#include "phd/seg_configConfig.h"
#include <phd/accuracy_service.h>

#define DEBUG true
#define foreach BOOST_FOREACH
#define m_size 0.01

struct PointXYZIT
{
  PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
  float intensity;
  float C2M_signed_distances;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIT,(float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, C2M_signed_distances, C2M_signed_distances))



bool do_calc(phd::accuracy_service::Request  &req,
         phd::accuracy_service::Response &res)
{	
if(DEBUG) ROS_INFO("Calculating Accuracy");
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_one(new pcl::PointCloud<pcl::PointXYZI> );
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_two(new pcl::PointCloud<pcl::PointXYZI> );
	pcl::PointCloud<PointXYZIT>::Ptr pcl_datum(new pcl::PointCloud<PointXYZIT> );

	pcl::PointCloud<PointXYZIT>::Ptr cloud(new pcl::PointCloud<PointXYZIT> );
	sensor_msgs::PointCloud2 cloud_req1 = req.cloud_1;
	sensor_msgs::PointCloud2 cloud_req2 = req.cloud_2;
	sensor_msgs::PointCloud2 cloud_datum = req.cloud_datum;
	//fix the naming discrepancy between ROS and PCL (from "intensities" to "intensity")
	cloud_req1.fields[3].name = "intensity";
	pcl::fromROSMsg(cloud_req1,*cloud_one);
	cloud_req2.fields[3].name = "intensity";
	pcl::fromROSMsg(cloud_req2,*cloud_two);
	pcl::fromROSMsg(cloud_req2,*cloud);
	cloud_datum.fields[3].name = "intensity";
	//ROS_INFO("Inpuit datum");
	pcl::fromROSMsg(cloud_datum,*pcl_datum);
	float resolution = 0.01f;
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> octree (resolution);
	octree.setInputCloud (cloud_one);
	octree.addPointsFromInputCloud ();
	pcl::octree::OctreePointCloudSearch<PointXYZIT> dattree (resolution);
	dattree.setInputCloud (pcl_datum);
	dattree.addPointsFromInputCloud ();
	pcl::PointXYZI searchPoint;
	PointXYZIT searchPointT;
	// Neighbors within radius search
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	int K = 1;
	std::vector<int> pointIdxNKNSearch;
	std::vector<float> pointNKNSquaredDistance;
	//Look for the nearest neighbor in cloud 1 of each point in cloud 2 
	BOOST_FOREACH (PointXYZIT& pt, cloud->points){
		searchPoint.x = pt.x;
		searchPoint.y = pt.y;
		searchPoint.z = pt.z;
		searchPointT.x = pt.x;
		searchPointT.y = pt.y;
		searchPointT.z = pt.z;

		if(octree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0){
			if(pointNKNSquaredDistance[0]<.1)
				pt.intensity = pointNKNSquaredDistance[0]*pointNKNSquaredDistance[0];
			else
				pt.intensity = 0;
		}
		if(dattree.nearestKSearch (searchPointT, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0){
			if(pointNKNSquaredDistance[0]<.1){
	//ROS_INFO("Msg2");
	float num1 = pt.intensity;
	//ROS_INFO("Msg2");
	float num2 = pcl_datum->points[pointIdxNKNSearch[0]].C2M_signed_distances;

	//ROS_INFO("Msg %f",num2);
				pt.C2M_signed_distances = fabs(pt.intensity-pcl_datum->points[pointIdxNKNSearch[0]].C2M_signed_distances);
			}else
	//ROS_INFO("Msg5");
				pt.intensity = 0;
		}
	}

	//ROS_INFO("Msg4");
	sensor_msgs::PointCloud2 cloud_msg;
	pcl::toROSMsg(*cloud,cloud_msg);
	//fix the naming discrepancy between ROS and PCL (from "intensities" to "intensity")
	cloud_msg.fields[3].name = "intensities";
	cloud_msg.header.frame_id = "/base_footprint";
	res.cloud_out = cloud_msg;

	//ROS_INFO("Msg4");
	return true;
}

int
main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "accuracy_service");
	ros::NodeHandle nh;
	ros::ServiceServer service = nh.advertiseService("acc_srv", do_calc); 
if(DEBUG) ROS_INFO("Accuracy Service Started");
	ros::spin();


}
