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
#include <pcl/filters/crop_box.h>
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
#include <phd/calc_service.h>

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



bool do_calc(phd::calc_service::Request  &req,
         phd::calc_service::Response &res)
{	
if(DEBUG) ROS_INFO("Calculating...");
	std::vector<pcl::PointCloud<pcl::PointXYZI> > pre_clouds;
	std::vector<pcl::PointCloud<pcl::PointXYZI> > post_clouds;
	std::vector<pcl::PointCloud<PointXYZIT> > computed_clouds;
	pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_holder(new pcl::PointCloud<pcl::PointXYZI> );
//Create a pcl pointer to load the datum cloud in to
	pcl::PointCloud<PointXYZIT>::Ptr datum (new pcl::PointCloud<PointXYZIT> );
	std::vector<pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> > pretrees;
	std::vector<pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> > posttrees;
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> octree(0.01f);
	pcl::octree::OctreePointCloudSearch<PointXYZIT> dattree(0.01f);
	std::vector<float> avgs;
	std::vector<int> counts;
	avgs.resize(req.pre_ids.size());
	counts.resize(req.pre_ids.size());
	//Load the file
	if (pcl::io::loadPCDFile<PointXYZIT> (req.datum.c_str(), *datum) == -1) PCL_ERROR ("Couldn't read file\n");
	
			dattree.setInputCloud (datum);
			dattree.addPointsFromInputCloud ();
//Crop points that are the robot itself
			Eigen::Vector4f minPoint; 
			minPoint[0]=.99;  // define minimum point x 
			minPoint[1]=2.5;  // define minimum point y 
			minPoint[2]=.3;  // define minimum point z 
			Eigen::Vector4f maxPoint; 
			maxPoint[0]=1.44;  // define max point x 
			maxPoint[1]=3.5;  // define max point y 
			maxPoint[2]=1.19;  // define max point z 
			Eigen::Vector3f boxTranslatation; 
			boxTranslatation[0]=0;   
			boxTranslatation[1]=0;   
			boxTranslatation[2]=0;   
			Eigen::Vector3f boxRotation; 
			boxRotation[0]=0;  // rotation around x-axis 
			boxRotation[1]=0;  // rotation around y-axis 
			boxRotation[2]=0;  //in radians rotation around z-axis. this rotates your cube 45deg around z-axis. 
			pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOut (new pcl::PointCloud<pcl::PointXYZI>); 
			pcl::CropBox<pcl::PointXYZI> cropFilter; 
			cropFilter.setMin(minPoint); 
			cropFilter.setMax(maxPoint);
			cropFilter.setTranslation(boxTranslatation); 
			cropFilter.setRotation(boxRotation); 
			cropFilter.setNegative(false);
	for(int ctr = 0; ctr < req.post_ids.size();++ctr){
		std::stringstream fs;
		fs <<  req.location << "post/" << req.post_ids[ctr] << "localized.pcd";
		if (pcl::io::loadPCDFile<pcl::PointXYZI> (fs.str().c_str(), *pcl_holder) == -1) PCL_ERROR ("Couldn't read file\n");
		else{

			cloudOut.reset(new pcl::PointCloud<pcl::PointXYZI>());
			cropFilter.setInputCloud (pcl_holder); 
			cropFilter.filter (*cloudOut);  
			post_clouds.push_back(*cloudOut);
			posttrees.push_back(octree);
			posttrees[ctr].setInputCloud (cloudOut);
			posttrees[ctr].addPointsFromInputCloud ();
		}
	}
	for(int ctr2 = 0; ctr2 < req.pre_ids.size();++ctr2){
		std::stringstream ss;
		ss <<  req.location << "pre/" << req.pre_ids[ctr2] << "localized.pcd";
		if (pcl::io::loadPCDFile<pcl::PointXYZI> (ss.str().c_str(), *pcl_holder) == -1) PCL_ERROR ("Couldn't read file\n");
		else{
			cloudOut.reset(new pcl::PointCloud<pcl::PointXYZI>());
			cropFilter.setInputCloud (pcl_holder); 
			cropFilter.filter (*cloudOut);  
			pre_clouds.push_back(*cloudOut);
			pretrees.push_back(octree);
			pretrees[ctr2].setInputCloud (cloudOut);
			pretrees[ctr2].addPointsFromInputCloud ();
		}
	}
computed_clouds.resize(req.pre_ids.size());

//Save Optitrack Location
	ofstream myfile;
	std::stringstream qs;
	qs << req.location << "processed/averages.csv";
	myfile.open (qs.str().c_str(), std::ios::out|std::ios::app);
	
///////////////////////////////
	for(int ctr = 0; ctr < req.post_ids.size();++ctr){
		avgs.clear();
		counts.clear();
		avgs.resize(req.pre_ids.size());
		counts.resize(req.pre_ids.size());
		pcl::PointXYZI searchPoint;
		PointXYZIT searchPointT, savepointT;
		// Neighbors within radius search
		std::vector<int> pointIdxRadiusSearch;
		std::vector<float> pointRadiusSquaredDistance;
		int K = 1;
		std::vector<int> pointIdxNKNSearch;
		std::vector<float> pointNKNSquaredDistance;
		float datum_val;
		//Look for the nearest neighbor in cloud 1 of each point in cloud 2 
		int forctr = 0;
		BOOST_FOREACH (pcl::PointXYZI& pt, post_clouds[ctr].points){
			searchPoint.x = pt.x;
			searchPoint.y = pt.y;
			searchPoint.z = pt.z;
			searchPointT.x = pt.x;
			searchPointT.y = pt.y;
			searchPointT.z = pt.z;
			savepointT.x = pt.x;
			savepointT.y = pt.y;
			savepointT.z = pt.z;
			if(dattree.nearestKSearch (searchPointT, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0){
				if(pointNKNSquaredDistance[0]<.1){

					datum_val = datum->points[pointIdxNKNSearch[0]].C2M_signed_distances;
				}else
					datum_val = -1;
			}
			for(int ctr2 = 0; ctr2 < req.pre_ids.size();++ctr2){

				//ROS_INFO("%d-%d: %d/%lu",ctr,ctr2,forctr,post_clouds[ctr].points.size());
				if(pretrees[ctr2].nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0){
					if(pointNKNSquaredDistance[0]<.1)
						savepointT.intensity = sqrt(pointNKNSquaredDistance[0]);
					else
						savepointT.intensity = -1;
				}
				if(datum_val != -1)
					savepointT.C2M_signed_distances = fabs(savepointT.intensity-datum_val);
				else
					savepointT.C2M_signed_distances = -1;
				computed_clouds[ctr2].points.push_back(savepointT);
				avgs[ctr2] = avgs[ctr2] + savepointT.C2M_signed_distances;
				counts[ctr2] = counts[ctr2] + 1;
			}
			++forctr;
		}

		for(int ctr2 = 0; ctr2 < req.pre_ids.size();++ctr2){
			std::stringstream pf;
			pf << req.location << "processed/" << req.pre_ids[ctr2] << "to" << req.post_ids[ctr] << ".pcd";
			ROS_INFO("Saving to %s\ncloud size: %lu",pf.str().c_str(),computed_clouds[ctr2].points.size());
			computed_clouds[ctr2].width = computed_clouds[ctr2].points.size();
			computed_clouds[ctr2].height = 1;
			computed_clouds[ctr2].is_dense = false;
			pcl::io::savePCDFileASCII (pf.str().c_str(), computed_clouds[ctr2]);
			computed_clouds[ctr2].points.clear();
			myfile << req.pre_ids[ctr2] << "to" << req.post_ids[ctr] << "," << avgs[ctr2]<< "," << counts[ctr2] << std::endl;
	
		}
	}


ROS_INFO("done");
myfile.close();

//////////////////////////////////



	
		//ROS_INFO("Loaded %lu and %lu",pre_clouds.size(),post_clouds.size());




/*
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
*/
	return true;
}

int
main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "calc_service");
	ros::NodeHandle nh;
	ros::ServiceServer service = nh.advertiseService("calc_srv", do_calc); 
if(DEBUG) ROS_INFO("calc Service Started");
	ros::spin();


}
