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
#include <phd/SEGConfig.h>

#define DEBUG 0
#define foreach BOOST_FOREACH
#define m_size 0.01

//these globals can be set in a launch file or using dynamic reconfigure
float leaf_size;
float LACC, DOT, LACC2;
int k_search;
int max_iterations;
float radius_search;
float distance_threshold;
float normal_distance_weight;
float distance_threshold2;
float normal_distance_weight2;
float eps_angle;
bool optimize_coefficients;
bool optimize_coefficients2;
bool reco, downsample;

//this callback is for dynamic reconfigure, it sets the global variables
void callback(phd::SEGConfig &config, uint32_t level) {

	//this boolean is used to break the plane searching loop
	reco = true;
	
	k_search = config.k_search;
	max_iterations = config.max_iterations;
	radius_search = config.radius_search;
	distance_threshold = config.distance_threshold;
	normal_distance_weight = config.normal_distance_weight;
	distance_threshold2 = config.distance_threshold2;
	normal_distance_weight2 = config.normal_distance_weight2;
	eps_angle = config.eps_angle;
	leaf_size = config.leaf_size;
	optimize_coefficients = config.optimize_coefficients;
	optimize_coefficients2 = config.optimize_coefficients2;
	downsample = config.skip_downsample;
	DOT = config.dot_product;
	LACC = config.dot_product_accuracy;
	LACC2 = config.purpendicularity_accuracy;
}


float dot_product(Eigen::Vector3f first,Eigen::Vector3f second){
	return first(0)*second(0) + first(1)*second(1) + first(2)*second(2);
}
float length_of(Eigen::Vector3f first){
	return sqrt(first(0)*first(0) + first(1)*first(1) + first(2)*first(2) );
}
Eigen::Vector3f vector_of(pcl::PointXYZI first,pcl::PointXYZI second){
	Eigen::Vector3f ret;
	ret(0) = second.x - first.x;
	ret(1) = second.y - first.y;
	ret(2) = second.z - first.z;
	return ret;
}
Eigen::Vector3f cross_product(Eigen::Vector3f first,Eigen::Vector3f second){
	Eigen::Vector3f ret;
	ret(0) = first(1)*second(2)-first(2)*second(1);
	ret(1) = first(2)*second(0)-first(0)*second(2);
	ret(2) = first(0)*second(1)-first(1)*second(0);
	return ret;
}
Eigen::Vector3f normalized_cross_product(Eigen::Vector3f first,Eigen::Vector3f second){
	Eigen::Vector3f ret;
	ret(0) = first(1)*second(2)-first(2)*second(1);
	ret(1) = first(2)*second(0)-first(0)*second(2);
	ret(2) = first(0)*second(1)-first(1)*second(0);
	float m = sqrt(pow(ret(0),2)+pow(ret(1),2)+pow(ret(2),2));
	ret(0) = ret(0)/m;
	ret(1) = ret(1)/m;
	ret(2) = ret(2)/m;
	return ret;
}
Eigen::Vector3f normalized_cross_product(Eigen::Vector4f first,Eigen::Vector3f second){
	Eigen::Vector3f ret;
	ret(0) = first(1)*second(2)-first(2)*second(1);
	ret(1) = first(2)*second(0)-first(0)*second(2);
	ret(2) = first(0)*second(1)-first(1)*second(0);
	float m = sqrt(pow(ret(0),2)+pow(ret(1),2)+pow(ret(2),2));
	ret(0) = ret(0)/m;
	ret(1) = ret(1)/m;
	ret(2) = ret(2)/m;
	return ret;
}
Eigen::Vector3f normalized_cross_product(Eigen::Vector3f first,Eigen::Vector4f second){

	Eigen::Vector3f ret;
	ret(0) = first(1)*second(2)-first(2)*second(1);
	ret(1) = first(2)*second(0)-first(0)*second(2);
	ret(2) = first(0)*second(1)-first(1)*second(0);
	float m = sqrt(pow(ret(0),2)+pow(ret(1),2)+pow(ret(2),2));
	ret(0) = ret(0)/m;
	ret(1) = ret(1)/m;
	ret(2) = ret(2)/m;
	return ret;
}
Eigen::Vector3f normalized_cross_product(Eigen::Vector4f first,Eigen::Vector4f second){

	Eigen::Vector3f ret;
	ret(0) = first(1)*second(2)-first(2)*second(1);
	ret(1) = first(2)*second(0)-first(0)*second(2);
	ret(2) = first(0)*second(1)-first(1)*second(0);
	float m = sqrt(pow(ret(0),2)+pow(ret(1),2)+pow(ret(2),2));
	ret(0) = ret(0)/m;
	ret(1) = ret(1)/m;
	ret(2) = ret(2)/m;
	return ret;
}

//this function checks to see if the given planes fit the model by comparing the dot product of their normals
float check_planes(Eigen::Vector4f first,Eigen::Vector4f second){
	Eigen::Vector3f normal1, normal2, normal3, dots;
	normal1(0) = first(0);
	normal1(1) = first(1);
	normal1(2) = first(2);
	normal2(0) = second(0);
	normal2(1) = second(1);
	normal2(2) = second(2);
	normal3(0) = 0;
	normal3(1) = 0;
	normal3(2) = 1;
	dots(0) = dot_product(normal1,normal2);
	dots(1) = dot_product(normal1,normal3);
	dots(2) = dot_product(normal3,normal2);
	if(fabs((fabs(dots(0))-DOT)) < LACC && (fabs(dots(1)) + fabs(dots(2))) < LACC2) return dots(0);
	else return 200;

}

//this functon locates the marker within a point cloud, it takes any amount of normals, returns true if a marker is found, and sets the marker location and transformation matrix values at the adresses given
bool locate_marker(std::vector<Eigen::Vector4f>& planes_loc, Eigen::Matrix3f &marker_location, Eigen::Matrix4f &A_mat){
	
	int cnts = 0;
	int i = 0;
	int j = 0;
	int k = 0;
	bool marker_flag = false;
	Eigen::Vector3f intersection, holder;
	Eigen::Vector3f best_intersection(100,100,100);
	float best_value = 100;
	float value;
	Eigen::Vector3f purpendicular,parallel1, parallel2;
	float P1x,P1y,P1z,P2x,P2y,P2z,P3x,P3y,P3z;
	Eigen::Vector3f a,n,c,o;
	float mn, mc;
	//I left these as while loops so I could easily add conditions to stop after a marker is found, but if there are multiple slightly parallel planes its better to check them all and use the best fit 
	while( i < (planes_loc.size()-1) && planes_loc.size() >= 2 ){
		j = i+1; 
		while(  j < (planes_loc.size())){
				//here we check if the planes fit the model
				value = check_planes(planes_loc.at(i),planes_loc.at(j));
				if(fabs(fabs(value)-DOT) < best_value){
					//calculate the point of intersection of the planes
					intersection(2) = 0; 	
					intersection(1) = (-planes_loc.at(i)(3) + (planes_loc.at(i)(0)/planes_loc.at(j)(0))*planes_loc.at(j)(3))/(planes_loc.at(i)(1)-(planes_loc.at(i)(0)/planes_loc.at(j)(0))*planes_loc.at(j)(1));
					intersection(0)	= (-planes_loc.at(i)(1)*intersection(1)-planes_loc.at(i)(3))/planes_loc.at(i)(0);
					//create a vector for the third plane					
					purpendicular(0) = 0;
					purpendicular(1) = 0;
					purpendicular(2) = 1;
					holder = normalized_cross_product(planes_loc.at(i),planes_loc.at(j));
					if(holder(2) > 0){
						if(value > 0){
							parallel1 = normalized_cross_product(purpendicular,planes_loc.at(i));
							parallel2 = normalized_cross_product(purpendicular,planes_loc.at(j));
						}
						else{
							parallel2 = normalized_cross_product(planes_loc.at(i),purpendicular);
							parallel1 = normalized_cross_product(purpendicular,planes_loc.at(j));
						}
					}
					else{
						if(value > 0) {
							parallel2 = normalized_cross_product(planes_loc.at(i),purpendicular);
							parallel1 = normalized_cross_product(planes_loc.at(j),purpendicular);
						}
						else	{
							parallel1 = normalized_cross_product(purpendicular,planes_loc.at(i));
							parallel2 = normalized_cross_product(planes_loc.at(j),purpendicular);
						}
					}
				
					ROS_INFO("Found a marker at %f - %f - %f\nvalue - %f - holder - %f", intersection(0),intersection(1),intersection(2),value, holder(2)); 
					best_value = fabs(fabs(value)-DOT);
					marker_flag = true;

				}
		++j;
		}
	++i;
	}


	if(marker_flag == true){

		P1x = intersection(0);
		P1y = intersection(1);
		P1z = intersection(2);
		P2x = intersection(0) + parallel1(0);
		P2y = intersection(1) + parallel1(1);
		P2z = intersection(2) + parallel1(2);
		P3x = intersection(0) + parallel2(0);
		P3y = intersection(1) + parallel2(1);
		P3z = intersection(2) + parallel2(2);
		marker_location(0,0) = P1x;
		marker_location(0,1) = P1y;
		marker_location(0,2) = P1z;
		marker_location(1,0) = P2x;
		marker_location(1,1) = P2y;
		marker_location(1,2) = P2z;
		marker_location(2,0) = P3x;
		marker_location(2,1) = P3y;
		marker_location(2,2) = P3z;

		mn = sqrt(pow(P3x-P2x,2)+pow(P3y-P2y,2)+pow(P3z-P2z,2));
		mc = sqrt(pow(P1x-P2x,2)+pow(P1y-P2y,2)+pow(P1z-P2z,2));
		n(0) = (P3x-P2x)/mn;
		n(1) = (P3y-P2y)/mn;
		n(2) = (P3z-P2z)/mn;
		c(0) = (P1x-P2x)/mc;
		c(1) = (P1y-P2y)/mc;
		c(2) = (P1z-P2z)/mc;
		a = normalized_cross_product(n,c);
		o = cross_product(a,n);
		A_mat(0,0) = n(0);
		A_mat(0,1) = o(0);
		A_mat(0,2) = a(0);
		A_mat(0,3) = (P1x+P2x+P3x)/3;
		A_mat(1,0) = n(1);
		A_mat(1,1) = o(1);
		A_mat(1,2) = a(1);
		A_mat(1,3) = (P1y+P2y+P3y)/3;
		A_mat(2,0) = n(2);
		A_mat(2,1) = o(2);
		A_mat(2,2) = a(2);
		A_mat(2,3) = (P1z+P2z+P3z)/3;
		A_mat(3,0) = 0;
		A_mat(3,1) = 0;
		A_mat(3,2) = 0;
		A_mat(3,3) = 1;

}
	else ROS_INFO("Could not find marker");


	return marker_flag;

}

Eigen::Vector4f refine_plane( pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_src, const pcl::PointCloud<pcl::Normal>::Ptr cloud_normals){
	sensor_msgs::PointCloud2 output2;
	Eigen::Vector4f plane;
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
	// Create the filtering object
	pcl::ExtractIndices<pcl::PointXYZI> extract;

	pcl::SACSegmentationFromNormals<pcl::PointXYZI, pcl::Normal> seg2; 
	seg2.setOptimizeCoefficients (optimize_coefficients2);

	// Mandatory
	seg2.setModelType (11);
	seg2.setMethodType (pcl::SAC_RANSAC);
	seg2.setMaxIterations (max_iterations);
	seg2.setDistanceThreshold (distance_threshold2);
	seg2.setNormalDistanceWeight(normal_distance_weight2);
	seg2.setEpsAngle(eps_angle);

	seg2.setInputCloud (cloud_src);
	seg2.setInputNormals(cloud_normals); 
	seg2.segment (*inliers, *coefficients);

	// Extract the inliers
	extract.setInputCloud (cloud_src);
	extract.setIndices (inliers);
	extract.setNegative (false);
	extract.filter (*cloud_p);
	plane(0) = coefficients->values[0];
	plane(1) = coefficients->values[1];
	plane(2) = coefficients->values[2];
	plane(3) = coefficients->values[3];
	return plane;

}

bool localize(phd::localize_cloud::Request  &req,
         phd::localize_cloud::Response &res)
{	
	rosbag::Bag bag;
	bool marker = false;
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_saved(new pcl::PointCloud<pcl::PointXYZI> );
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_initial(new pcl::PointCloud<pcl::PointXYZI> );
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_aligned(new pcl::PointCloud<pcl::PointXYZI> );
	Eigen::Matrix4f transform_mat;

	//Eigen::Matrix4f transformA = transform_affine;
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI> );
	sensor_msgs::PointCloud2 cloud_req = req.cloud_in;
	//fix the naming discrepancy between ROS and PCL (from "intensities" to "intensity")
	cloud_req.fields[3].name = "intensity";
	pcl::fromROSMsg(cloud_req,*cloud);

	std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height << " data points." << std::endl;
	pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZI>), cloud_f (new pcl::PointCloud<pcl::PointXYZI>);
	Eigen::Vector4f plane, plane_r;
	std::vector<Eigen::Vector4f> planes;

	planes.clear();
	pcl::toPCLPointCloud2 ( *cloud,*cloud_blob);

	// Create the filtering object: downsample the dataset using a leaf size of 1cm
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud (cloud_blob);
	sor.setLeafSize (leaf_size,leaf_size, leaf_size);
	sor.filter (*cloud_filtered_blob);

	// Convert to the templated PointCloud
	pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);
	std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;
	//donsample cloud
	if(downsample){
		ROS_INFO("Skipping downsample");
		cloud_filtered.swap(cloud);
		std::cerr << "using " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;
		}	

	//prepare segmentation
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
	// Create the segmentation object
	pcl::SACSegmentationFromNormals<pcl::PointXYZI, pcl::Normal> seg; 
	// Optional
	seg.setOptimizeCoefficients (optimize_coefficients);
	// Mandatory
	seg.setModelType (17);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (max_iterations);
	seg.setDistanceThreshold (distance_threshold);
	seg.setNormalDistanceWeight(normal_distance_weight);
	seg.setEpsAngle(eps_angle);
	// Create the filtering object
	pcl::ExtractIndices<pcl::PointXYZI> extract;
	pcl::ExtractIndices<pcl::Normal> extract_normals;
	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
	ne.setInputCloud (cloud_filtered);
	ne.setSearchSurface (cloud);
	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI> ());
	ne.setSearchMethod (tree);
	// Output datasets
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_f (new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_p (new pcl::PointCloud<pcl::Normal>);
	// Use all neighbors in a sphere of radius 3cm
	ne.setRadiusSearch (radius_search);
	ne.setKSearch (k_search);
	// Compute the features
	ne.compute (*cloud_normals);

	//start looking through planes
	int ctr = 0;
	reco = false;
	while(ctr < 30 && reco == false && ros::ok() && marker == false){
		++ctr;


		ROS_INFO("Iteration %d",ctr);

		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud (cloud_filtered);
		seg.setInputNormals(cloud_normals); 
		seg.segment (*inliers, *coefficients);
		if (inliers->indices.size () == 0)
			std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;


		// Extract the inliers
		extract.setInputCloud (cloud_filtered);
		extract.setIndices (inliers);
		extract.setNegative (false);
		extract.filter (*cloud_p);   

		extract_normals.setInputCloud (cloud_normals);
		extract_normals.setIndices (inliers); 
		extract_normals.setNegative (false);
		extract_normals.filter (*cloud_normals_p);
		
		plane(0) = coefficients->values[0];
		plane(1) = coefficients->values[1];
		plane(2) = coefficients->values[2];
		plane(3) = coefficients->values[3];

		plane_r = refine_plane(cloud_p,cloud_normals_p);

		planes.push_back(plane_r);
		Eigen::Matrix3f marker_loc;
		if(locate_marker(planes, marker_loc, transform_mat )){
			
			marker = true;

		}
		//extract remining points
		extract.setNegative (true);
		extract.filter (*cloud_f);
		cloud_filtered.swap (cloud_f);
		extract_normals.setNegative (true);
		extract_normals.filter (*cloud_normals_f);
		cloud_normals.swap (cloud_normals_f);

		}
	

	 
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_int (new pcl::PointCloud<pcl::PointXYZI> );

	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_output (new pcl::PointCloud<pcl::PointXYZI> );

	if(req.homing){

		Eigen::Matrix4f transformAA = transform_mat.inverse();

		pcl::transformPointCloud (*cloud, *cloud_output, transformAA);

		bag.open("/home/mike/marker/test.bag", rosbag::bagmode::Write);

		std_msgs::Float64 f;
		for(int fctr = 0; fctr < 4; ++fctr){
			for(int fctr2 = 0; fctr2 < 4; ++fctr2){
				f.data = transform_mat(fctr2,fctr);
	
				bag.write("fiducial", ros::Time::now(), f);
			}
		}	
		    bag.close();
	}else{

		bag.open("/home/mike/marker/test.bag", rosbag::bagmode::Read);
		Eigen::Matrix4f transformA;
		    rosbag::View view(bag, rosbag::TopicQuery("fiducial"));
			int fctr = 0;
			int fctr2 = 0;
		    foreach(rosbag::MessageInstance const m, view)
		    {

			std_msgs::Float64::ConstPtr i = m.instantiate<std_msgs::Float64>();
			if (i != NULL)
			     transformA(fctr2,fctr) = i->data;
			++fctr2;
			if(fctr2 == 4){
				fctr2 = 0;
				++fctr;
			}
		    }

		    bag.close();	

		Eigen::Matrix4f transformAA = transformA.inverse();
		Eigen::Matrix4f transformAB = transformA*transform_mat.inverse();
		pcl::transformPointCloud (*cloud, *cloud_int, transformAB);
		pcl::transformPointCloud (*cloud_int, *cloud_output, transformAA);
	}

	sensor_msgs::PointCloud2 cloud_msg;
	pcl::toROSMsg(*cloud_output,cloud_msg);
	//fix the naming discrepancy between ROS and PCL (from "intensities" to "intensity")
	cloud_msg.fields[3].name = "intensities";
	cloud_msg.header.frame_id = "/world";
	res.cloud_out = cloud_msg;
  	return marker;
}

int
main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "pcd_localization_service");
	ros::NodeHandle nh;
	ros::ServiceServer service = nh.advertiseService("localize_pcd", localize); 
	dynamic_reconfigure::Server<phd::SEGConfig> server;
	dynamic_reconfigure::Server<phd::SEGConfig>::CallbackType f;

	f = boost::bind(&callback, _1, _2);
	server.setCallback(f);

	ros::spin();


}
