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
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <std_msgs/Float64.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <dynamic_reconfigure/server.h>
#include <phd/SEGConfig.h>

#define VACC 0.01
#define VINT 650
#define DEBUG 1
#define foreach BOOST_FOREACH

float VAL1, VAL2, VAL3, VAL4;

ros::Publisher tpub;
ros::Publisher vis_pub;


float dot_product(Eigen::Vector3f first,Eigen::Vector3f second){
	return first(0)*second(0) + first(1)*second(1) + first(2)*second(2);
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

std::vector<pcl::PointXYZI> cluster_points(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in){

//cluster points

	if(DEBUG) ROS_INFO("Clustering");


	std::vector<pcl::PointXYZI> Pts;
	for(int i = 0; i < (cloud_in->width * cloud_in->height) - 1 ; ++i){
		if(cloud_in->points[i].intensity != 0){
		std::vector<pcl::PointXYZI> cluster;
		cluster.push_back(cloud_in->points[i]);
		for(int j = i+ 1; j < (cloud_in->width * cloud_in->height); ++j){

			if(sqrt(pow(cloud_in->points[i].x - cloud_in->points[j].x,2)+pow(cloud_in->points[i].y - cloud_in->points[j].y,2)+pow(cloud_in->points[i].z - cloud_in->points[j].z,2)) < 0.06 && cloud_in->points[j].intensity != 0){
				cluster.push_back(cloud_in->points[j]);
				cloud_in->points[j].intensity = 0;
			}

		}

		float xs =0;
		float ys=0;
		float zs=0;
		float is=0;
		for(int k = 0; k < cluster.size(); ++k){
			xs+=cluster[k].x;
			ys+=cluster[k].y;
			zs+=cluster[k].z;
			is+=cluster[k].intensity;
			}
		pcl::PointXYZI Pt;
		Pt.x = xs/cluster.size();
		Pt.y = ys/cluster.size();
		Pt.z = zs/cluster.size();
		Pt.intensity = is/cluster.size();
		Pts.push_back(Pt);
		}
	}
	return Pts;
}


//this functon locates the marker within a point cloud, it takes any amount of normals, returns true if a marker is found, and sets the marker location and transformation matrix values at the adresses given
bool locate_marker(std::vector<pcl::PointXYZI> Pts, Eigen::Matrix3f &marker_location, Eigen::Matrix4f &A_mat){
	
	if(DEBUG) ROS_INFO("Locating from %d points", Pts.size());
	float bvecs = 1;
	float vecs;
	int cnts = 0;
	int i = 0;
	int j = 0;
	int k = 0;
	bool marker_flag = false;
	bool opt_flag = false;
	Eigen::Vector3f vec1, vec2, intersection, holder;
	Eigen::Vector3f best_intersection(100,100,100);
	float best_value = 1;
	float val1,val2,val3,val4;
	float P1x,P1y,P1z,P2x,P2y,P2z,P3x,P3y,P3z;
	Eigen::Vector3f a,n,c,o;
	float mn, mc;

	for(i = 0; i < Pts.size(); ++i){
		for(j = 0; j < Pts.size(); ++j){
			for(k = 0; k < Pts.size(); ++k){
				if(i != j && i != k && j != k && Pts[i].intensity > VINT && Pts[j].intensity > VINT && Pts[k].intensity > VINT){
					val1 = sqrt(pow(Pts[i].x - Pts[j].x,2)+pow(Pts[i].y - Pts[j].y,2)+pow(Pts[i].z - Pts[j].z,2));
					val3 = sqrt(pow(Pts[i].x - Pts[k].x,2)+pow(Pts[i].y - Pts[k].y,2)+pow(Pts[i].z - Pts[k].z,2));
					val2 = sqrt(pow(Pts[k].x - Pts[j].x,2)+pow(Pts[k].y - Pts[j].y,2)+pow(Pts[k].z - Pts[j].z,2));
					vec1[0] = Pts[j].x - Pts[i].x;
					vec1[1] = Pts[j].y - Pts[i].y;
					vec1[2] = Pts[j].z - Pts[i].z;
					vec2[0] = Pts[j].x - Pts[k].x;
					vec2[1] = Pts[j].y - Pts[k].y;
					vec2[2] = Pts[j].z - Pts[k].z;
					val4 = dot_product(vec1,vec2);
					if(fabs(val1-VAL1) < VACC && fabs(val2-VAL2) < VACC && fabs(val3-VAL3) < VACC && fabs(fabs(val4)-VAL4) < 3*VACC){ 
						if(DEBUG) ROS_INFO("Intensities: %f - %f - %f",Pts[i].intensity,Pts[j].intensity,Pts[k].intensity);	
						//if(DEBUG) ROS_INFO("Pt1 Vals: %f - %f - %f\nPt2 Vals: %f - %f - %f\nPt3 Vals: %f - %f - %f\n", Pts[i].x,Pts[i].y,Pts[i].z,Pts[j].x,Pts[j].y,Pts[j].z,Pts[k].x,Pts[k].y,Pts[k].z);	
						
						if(marker_flag == false) marker_flag = true;
						else opt_flag = true;
						P1x = Pts[i].x;
						P1y = Pts[i].y;
						P1z = Pts[i].z;
						P2x = Pts[j].x;
						P2y = Pts[j].y;
						P2z = Pts[j].z;
						P3x = Pts[k].x;
						P3y = Pts[k].y;
						P3z = Pts[k].z;
						if(DEBUG) ROS_INFO("Marker Location: %f/%f/%f - %f/%f/%f - %f/%f/%f", P1x,P1y,P1z,P2x,P2y,P2z,P3x,P3y,P3z);		
					}
				}
			}
		}
	}








	if(marker_flag == true && opt_flag == false){

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
	else {
		ROS_INFO("Could not find marker, switching to optimization");
		marker_flag = false;
		int vint_mod = 0;
	while(marker_flag == false && vint_mod < 160){	
	for(i = 0; i < Pts.size(); ++i){
		for(j = 0; j < Pts.size(); ++j){
			for(k = 0; k < Pts.size(); ++k){
				if(i != j && i != k && j != k && Pts[i].intensity > (VINT - vint_mod) && Pts[j].intensity > (VINT - vint_mod) && Pts[k].intensity > (VINT - vint_mod)){
					val1 = sqrt(pow(Pts[i].x - Pts[j].x,2)+pow(Pts[i].y - Pts[j].y,2)+pow(Pts[i].z - Pts[j].z,2));
					val3 = sqrt(pow(Pts[i].x - Pts[k].x,2)+pow(Pts[i].y - Pts[k].y,2)+pow(Pts[i].z - Pts[k].z,2));
					val2 = sqrt(pow(Pts[k].x - Pts[j].x,2)+pow(Pts[k].y - Pts[j].y,2)+pow(Pts[k].z - Pts[j].z,2));
					vec1[0] = Pts[j].x - Pts[i].x;
					vec1[1] = Pts[j].y - Pts[i].y;
					vec1[2] = Pts[j].z - Pts[i].z;
					vec2[0] = Pts[j].x - Pts[k].x;
					vec2[1] = Pts[j].y - Pts[k].y;
					vec2[2] = Pts[j].z - Pts[k].z;
					val4 = dot_product(vec1,vec2);
					vecs = fabs(val1-VAL1) + fabs(val2-VAL2) + fabs(val3-VAL3) + fabs(fabs(val4)-VAL4)/3;
					if(fabs(vecs) < bvecs){
						//ROS_INFO("Vals: %f - %f - %f - %f", val1, val2, val3, val4);
						bvecs = vecs;
						//}
					//if(fabs(val1-VAL1) < VACC && fabs(val2-VAL2) < VACC && fabs(val3-VAL3) < VACC && fabs(fabs(val4)-VAL4) < 3*VACC){ 
						if(DEBUG) ROS_INFO("Marker Vals: %f - %f - %f - %f / %f",val1,val2,val3,val4, vecs);		
						if(DEBUG) ROS_INFO("Intensities: %f - %f - %f",Pts[i].intensity,Pts[j].intensity,Pts[k].intensity);	
						
						marker_flag = true;
						P1x = Pts[i].x;
						P1y = Pts[i].y;
						P1z = Pts[i].z;
						P2x = Pts[j].x;
						P2y = Pts[j].y;
						P2z = Pts[j].z;
						P3x = Pts[k].x;
						P3y = Pts[k].y;
						P3z = Pts[k].z;
						if(DEBUG) ROS_INFO("Marker Location: %f/%f/%f - %f/%f/%f - %f/%f/%f", P1x,P1y,P1z,P2x,P2y,P2z,P3x,P3y,P3z);		
					}
				}
			}
		}
	}
	ROS_INFO("Dropping VINT %d", VINT - vint_mod);
	vint_mod += 25;
	}
	if(marker_flag == true){

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
	else 	ROS_INFO("Could not find marker");

}

	visualization_msgs::Marker point_list;
	point_list.header.frame_id = "/base_link";
	point_list.header.stamp = ros::Time::now();
	point_list.ns = "points_and_lines";
	point_list.action = visualization_msgs::Marker::ADD;
	point_list.pose.orientation.w = 1.0;

	point_list.id = 2;
	point_list.type = visualization_msgs::Marker::POINTS;



	point_list.scale.x = 0.1;
	point_list.scale.y = 0.1;


	// Line list is red
	point_list.color.r = 1.0;
	point_list.color.a = 0.5;

	geometry_msgs::Point p;
	p.x = P1x;
	p.y = P1y;
	p.z = P1z;
	point_list.points.push_back(p);
	p.x = P2x;
	p.y = P2y;
	p.z = P2z;
	point_list.points.push_back(p);
	p.x = P3x;
	p.y = P3y;
	p.z = P3z;
	point_list.points.push_back(p);
	vis_pub.publish(point_list);
	return marker_flag;

}


bool localize(phd::localize_cloud::Request  &req,
         phd::localize_cloud::Response &res)
{	
	bool marker = true;
	rosbag::Bag bag;
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_world(new pcl::PointCloud<pcl::PointXYZI> );
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_output (new pcl::PointCloud<pcl::PointXYZI>);
	sensor_msgs::PointCloud2 cloud_msg;
	sensor_msgs::PointCloud2 cloud_req = req.cloud_in;
	//fix the naming discrepancy between ROS and PCL (from "intensities" to "intensity")
	cloud_req.fields[3].name = "intensity";
	pcl::fromROSMsg(cloud_req,*cloud_world);
	//Create containers for filtered clouds
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_intensity_filtered (new pcl::PointCloud<pcl::PointXYZI>);
	// Create the filtering object
	pcl::PassThrough<pcl::PointXYZI> pass (true);
	//filter the data
	pass.setInputCloud (cloud_world);
	pass.setFilterFieldName ("intensity");
	pass.setFilterLimits (550,1100);
	pass.filter (*cloud_intensity_filtered);
	ROS_INFO("Points after filtering: %d",cloud_intensity_filtered->width);
	sensor_msgs::PointCloud2 cloud_debug;
	pcl::toROSMsg(*cloud_intensity_filtered,cloud_debug);
	//fix the naming discrepancy between ROS and PCL (from "intensities" to "intensity")
	cloud_debug.fields[3].name = "intensity";
	cloud_debug.header.frame_id = "/base_link";
	cloud_debug.header.stamp = ros::Time::now();
	tpub.publish(cloud_req);
	geometry_msgs::Point p1, p2, p3;
	ifstream marker_file( "/home/mike/marker/location/filename.dat");
	marker_file >> p1.x;
	marker_file >> p1.y;
	marker_file >> p1.z;
	marker_file >> p2.x;
	marker_file >> p2.y;
	marker_file >> p2.z;
	marker_file >> p3.x;
	marker_file >> p3.y;
	marker_file >> p3.z;
	Eigen::Vector3f vec1, vec2;
	VAL1 = sqrt(pow(p1.x - p2.x,2)+pow(p1.y - p2.y,2)+pow(p1.z - p2.z,2));
	VAL3 = sqrt(pow(p1.x - p3.x,2)+pow(p1.y - p3.y,2)+pow(p1.z - p3.z,2));
	VAL2 = sqrt(pow(p3.x - p2.x,2)+pow(p3.y - p2.y,2)+pow(p3.z - p2.z,2));
	vec1[0] = p2.x - p1.x;
	vec1[1] = p2.y - p1.y;
	vec1[2] = p2.z - p1.z;
	vec2[0] = p2.x - p3.x;
	vec2[1] = p2.y - p3.y;
	vec2[2] = p2.z - p3.z;
	VAL4 = dot_product(vec1,vec2);

	//if(DEBUG) ROS_INFO("Using VALS: %f - %f - %f - %f\n from pts %f - %f - %f\n%f - %f - %f\n%f - %f - %f\n", VAL1, VAL2, VAL3, VAL4, p1.x,p1.y,p1.z,p2.x,p2.y,p2.z,p3.x,p3.y,p3.z);
	std::vector<pcl::PointXYZI> Pts;

	Pts = cluster_points(cloud_intensity_filtered);
	if(DEBUG) ROS_INFO("Clustered Points: %d", Pts.size());
	Eigen::Matrix3f marker_loc;
	Eigen::Matrix4f transform_mat;

	bool ret = locate_marker(Pts, marker_loc, transform_mat);
	if(req.homing){
		//std::cout << "Transform Mat" << transform_mat << std::endl;
		pcl::toROSMsg(*cloud_world,cloud_msg);
	}else{

		bag.open("/home/mike/marker/fiducial.bag", rosbag::bagmode::Read);
		Eigen::Matrix4f transformA;
		rosbag::View view(bag, rosbag::TopicQuery("fiducial"));
		int fctr = 0;
		int fctr2 = 0;
		foreach(rosbag::MessageInstance const m, view){

			std_msgs::Float64::ConstPtr i = m.instantiate<std_msgs::Float64>();
			if (i != NULL) transformA(fctr2,fctr) = i->data;
			++fctr2;
			if(fctr2 == 4){
				fctr2 = 0;
				++fctr;
			}
		}
		bag.close();	

		//std::cout << "Transform Mat" << transform_mat << std::endl;
		//std::cout << "Transform A" << transformA << std::endl;
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_int (new pcl::PointCloud<pcl::PointXYZI>);
		Eigen::Matrix4f transformAB = transform_mat.inverse();
		Eigen::Matrix4f transformBB = transformA.inverse();
		pcl::transformPointCloud (*cloud_world, *cloud_int, transformAB);
		pcl::transformPointCloud (*cloud_int, *cloud_output, transformA);
		pcl::toROSMsg(*cloud_output,cloud_msg);
	}
	//fix the naming discrepancy between ROS and PCL (from "intensities" to "intensity")
	cloud_msg.fields[3].name = "intensity";
	cloud_msg.header.frame_id = "/base_link";
	cloud_msg.header.stamp = ros::Time::now();
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
	tpub = nh.advertise<sensor_msgs::PointCloud2> ("unaligned_cloud", 1);
	vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
	ros::spin();


}
