#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <phd/LocalizeConfig.h>
#include <phd/marker_val.h>
#include <phd/marker_msg.h>
#include <phd/doctor_msg.h>
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
#include <iterator>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
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
#include "phd/doctor_cloud.h"
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <std_msgs/Float64.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <dynamic_reconfigure/server.h>
#include "phd/seg_configConfig.h"

#define DEBUG 1
#define foreach BOOST_FOREACH

float VAL1, VAL2, VAL3, VAL4, CROP;
float VACC, CLUSTER_SIZE;
int VINT_MIN, VINT_MAX;
ros::Publisher vis_pub;
std::vector<phd::marker_msg> marker_list;
visualization_msgs::Marker point_list;
phd::localize_cloud loc_srv;
ros::ServiceClient loc_client;

//This callback is for dynamic reconfigure, it sets the global variables
void reconfig_callback(phd::LocalizeConfig &config, uint32_t level) {

	VACC = config.accuracy;
	CLUSTER_SIZE = config.cluster_size;
	VINT_MIN = config.intensity_min;
	VINT_MAX = config.intensity_max;
	CROP = config.crop_height;


}

//Helper function for performing dot products
float dot_product(Eigen::Vector3f first,Eigen::Vector3f second){
	return first(0)*second(0) + first(1)*second(1) + first(2)*second(2);
}

//Helper function for performing dot products
Eigen::Vector3f cross_product(Eigen::Vector3f first,Eigen::Vector3f second){
	Eigen::Vector3f ret;
	ret(0) = first(1)*second(2)-first(2)*second(1);
	ret(1) = first(2)*second(0)-first(0)*second(2);
	ret(2) = first(0)*second(1)-first(1)*second(0);
	return ret;
}

//Helper function for performing dot products
Eigen::Vector3f normalized_cross_product(Eigen::Vector3f first,Eigen::Vector3f second){
	Eigen::Vector3f ret = cross_product(first,second);
	float m = sqrt(pow(ret(0),2)+pow(ret(1),2)+pow(ret(2),2));
	ret(0) = ret(0)/m;
	ret(1) = ret(1)/m;
	ret(2) = ret(2)/m;
	return ret;
}

//cluster points
std::vector<pcl::PointXYZI> cluster_points(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in){
	//Make a Header stamp for the visualization marker
	point_list.header.stamp = ros::Time::now();
	//This vector holds all the keypoint candidates
	std::vector<pcl::PointXYZI> Pts;
	
	for(int i = 0; i < (cloud_in->width * cloud_in->height) - 1 ; ++i){
		if(cloud_in->points[i].intensity != 0){
			std::vector<pcl::PointXYZI> cluster;
			cluster.push_back(cloud_in->points[i]);
			for(int j = i+ 1; j < (cloud_in->width * cloud_in->height); ++j){

				if(sqrt(pow(cloud_in->points[i].x - cloud_in->points[j].x,2)+pow(cloud_in->points[i].y - cloud_in->points[j].y,2)+pow(cloud_in->points[i].z - cloud_in->points[j].z,2)) < CLUSTER_SIZE && cloud_in->points[j].intensity != 0){
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
			geometry_msgs::Point p;
			Pt.x = xs/cluster.size();
			Pt.y = ys/cluster.size();
			Pt.z = zs/cluster.size();
			Pt.intensity = is/cluster.size();
			p.x = Pt.x;
			p.y = Pt.y;
			p.z = Pt.z;
			//if(DEBUG) ROS_INFO("Found point %f/%f - %f - %f",Pt.intensity,Pt.x,Pt.y,Pt.z);
			Pts.push_back(Pt);
			point_list.points.push_back(p);
		}
	}
	//Write algorithm to detect if Pts.size > 3 and delete the cluster with the least points here if having issues

	point_list.color.r = 1.0;
	point_list.color.b = 0.0;
	vis_pub.publish(point_list);
	return Pts;
}


//this functon locates the marker within a point cloud, it takes any amount of normals, returns true if a marker is found, and sets the marker location and transformation matrix values at the adresses given
bool locate_marker(std::vector<pcl::PointXYZI> Pts, Eigen::Matrix3f &marker_location, Eigen::Matrix4f &A_mat){
	
	//if(DEBUG) ROS_INFO("Locating from %lu points", Pts.size());
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
	std::vector<phd::marker_val> val_array;
	phd::marker_val marker_val;
	point_list.color.r = 0.0;
	point_list.color.b = 1.0;

	geometry_msgs::Point p;	
	for(i = 0; i < Pts.size(); ++i){
		for(j = 0; j < Pts.size(); ++j){
			for(k = 0; k < Pts.size(); ++k){
				if(i != j && i != k && j != k){
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
					marker_val.val = fabs(val1-VAL1) + fabs(val2-VAL2) + fabs(val3-VAL3) + fabs(fabs(val4)-VAL4)/3;
					marker_val.i = i;
					marker_val.j = j;
					marker_val.k = k;
					val_array.push_back(marker_val);
					//if(fabs(val1-VAL1) < VACC && fabs(val2-VAL2) < VACC && fabs(val3-VAL3) < VACC && fabs(fabs(val4)-VAL4) < 3*VACC){ 
					if(marker_val.val < 4*VACC){
						//if(DEBUG) ROS_INFO("Marker Found, Val: %f",marker_val.val);	
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
						//if(DEBUG) ROS_INFO("Marker Location: %f/%f/%f - %f/%f/%f - %f/%f/%f", P1x,P1y,P1z,P2x,P2y,P2z,P3x,P3y,P3z);		
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
	if(opt_flag == false) ROS_INFO("Could not find marker, switching to optimization");
	else ROS_INFO("Found multiple markers, switching to optimization");
	
	
	float bval = 0.5;
	for(int ctr = 0; ctr < val_array.size(); ++ctr){

		if(val_array[ctr].val < bval){
			marker_flag = true;
			bval = val_array[ctr].val;
			P1x = Pts[val_array[ctr].i].x;
			P1y = Pts[val_array[ctr].i].y;
			P1z = Pts[val_array[ctr].i].z;
			P2x = Pts[val_array[ctr].j].x;
			P2y = Pts[val_array[ctr].j].y;
			P2z = Pts[val_array[ctr].j].z;
			P3x = Pts[val_array[ctr].k].x;
			P3y = Pts[val_array[ctr].k].y;
			P3z = Pts[val_array[ctr].k].z;
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
			point_list.color.r += 0.1;
			point_list.color.b -= 0.1;
			vis_pub.publish(point_list);
	if(DEBUG)ROS_INFO("Marker Location: %f/%f/%f - %f/%f/%f - %f/%f/%f -- %f",P1x,P1y,P1z,P2x,P2y,P2z,P3x,P3y,P3z,bval);		
		}


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



	return marker_flag;

}




//FLOAT RETURNING VERSION
//this functon locates the marker within a point cloud, it takes any amount of normals, returns true if a marker is found, and sets the marker location and transformation matrix values at the adresses given
float locate_marker(std::vector<pcl::PointXYZI> Pts, std::vector<phd::doctor_msg> &msg_list, std::vector<phd::marker_msg> markers){
	
	if(DEBUG) ROS_INFO("Locating from %lu points and %lu markers %lu", Pts.size(), markers.size(), msg_list.size());
	msg_list.clear();
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
	std::vector<phd::marker_val> val_array;
	int master = markers.size();
	std::vector<std::vector<phd::marker_val> > val_master(master);
	val_array.clear();
	val_master.clear();
	phd::marker_val marker_val;
	phd::doctor_msg doc_msg;
	point_list.color.r = 0.0;
	point_list.color.b = 1.0;

	geometry_msgs::Point p;	
	for(i = 0; i < Pts.size(); ++i){
		for(j = 0; j < Pts.size(); ++j){
			for(k = 0; k < Pts.size(); ++k){
				if(i != j && i != k && j != k){
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
					for(int ctr = 0; ctr < markers.size();++ctr){					
						marker_val.val = fabs(val1-markers[ctr].VAL1) + fabs(val2-markers[ctr].VAL2) + fabs(val3-markers[ctr].VAL3) + fabs(fabs(val4)-markers[ctr].VAL4)/3;
						marker_val.i = i;
						marker_val.j = j;
						marker_val.k = k;
						marker_val.index = ctr;
						val_master[ctr].push_back(marker_val);
						//val_array.push_back(marker_val);
					}
					//if(fabs(val1-marker.VAL1) < VACC && fabs(val2-marker.VAL2) < VACC && fabs(val3-marker.VAL3) < VACC && fabs(fabs(val4)-marker.VAL4) < 3*VACC){ 
					/*if(marker_val.val < 4*VACC){
						if(DEBUG) ROS_INFO("Marker Found, Val: %f",marker_val.val);	
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
						if(DEBUG) ROS_INFO("Marker Location: %f/%f/%f - %f/%f/%f - %f/%f/%f", P1x,P1y,P1z,P2x,P2y,P2z,P3x,P3y,P3z);		
					}*/
				}
			}
		}
	}


	float bval = 100;
	float lbval;
	int best_idx = -1;
	for(int vctr = 0; vctr < master; ++vctr){
	lbval = 100;
	for(int ctr = 0; ctr < val_master[vctr].size(); ++ctr){
		if(val_master[vctr][ctr].val < VACC*10){
	
			if(val_master[vctr][ctr].val < bval){
				bval = val_master[vctr][ctr].val;
				lbval = bval;
				best_idx = val_master[vctr][ctr].index;
				ROS_INFO("New best");
			}
			//idxs.push_back(best_idx);
			P1x = Pts[val_master[vctr][ctr].i].x;
			P1y = Pts[val_master[vctr][ctr].i].y;
			P1z = Pts[val_master[vctr][ctr].i].z;
			P2x = Pts[val_master[vctr][ctr].j].x;
			P2y = Pts[val_master[vctr][ctr].j].y;
			P2z = Pts[val_master[vctr][ctr].j].z;
			P3x = Pts[val_master[vctr][ctr].k].x;
			P3y = Pts[val_master[vctr][ctr].k].y;
			P3z = Pts[val_master[vctr][ctr].k].z;
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
			
			doc_msg.transform_mat[0] = n(0);
			doc_msg.transform_mat[1] = o(0);
			doc_msg.transform_mat[2] = a(0);
			doc_msg.transform_mat[3] = (P1x+P2x+P3x)/3;
			doc_msg.transform_mat[4] = n(1);
			doc_msg.transform_mat[5] = o(1);
			doc_msg.transform_mat[6] = a(1);
			doc_msg.transform_mat[7] = (P1y+P2y+P3y)/3;
			doc_msg.transform_mat[8] = n(2);
			doc_msg.transform_mat[9] = o(2);
			doc_msg.transform_mat[10] = a(2);
			doc_msg.transform_mat[11] = (P1z+P2z+P3z)/3;
			doc_msg.transform_mat[12] = 0;
			doc_msg.transform_mat[13] = 0;
			doc_msg.transform_mat[14] = 0;
			doc_msg.transform_mat[15] = 1;
			doc_msg.x = val_master[vctr][ctr].val;
			doc_msg.id = best_idx;
			//if(DEBUG)ROS_INFO("Marker Found: %f/%f/%f - %f/%f/%f - %f/%f/%f -- %f\nAt idx: %d num: %lu",P1x,P1y,P1z,P2x,P2y,P2z,P3x,P3y,P3z,bval,ctr,idxs.size());		
			msg_list.push_back(doc_msg);	
			if(DEBUG)ROS_INFO("Adding %lu Best Val: %f Using Marker: %d",msg_list.size(),bval,best_idx);	
		}
	}
	
	}
	return best_idx;

}





bool localize(phd::doctor_cloud::Request  &req, phd::doctor_cloud::Response &res){
	
	bool marker; //indicates if marker was found
	int marker_idx; //indicates if marker was found
	pcl::PointCloud<pcl::PointXYZI>::Ptr save_cloud (new pcl::PointCloud<pcl::PointXYZI>);
	std::vector<float> idxs;
	std::vector<phd::doctor_msg> doc_msgs;
	rosbag::Bag bag; //bag file for recording marker to disk
	phd::doctor_msg doc_msg;
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_world(new pcl::PointCloud<pcl::PointXYZI> ); //PCL formatted input cloud
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_output (new pcl::PointCloud<pcl::PointXYZI>); //PCL container for response
	sensor_msgs::PointCloud2 cloud_msg; //ROS container for response
	sensor_msgs::PointCloud2 cloud_req = req.cloud_in; //copy the cloud request so we can modify its contents
	cloud_req.fields[3].name = "intensity";
	pcl::fromROSMsg(cloud_req,*cloud_world); //copy cloud request to PCL format
	//create containers for filtered clouds
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_intensity_filtered (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_height_filtered (new pcl::PointCloud<pcl::PointXYZI>);
	//create the filtering object
	pcl::PassThrough<pcl::PointXYZI> pass (true);
	//filter the data for Z values from -100 to CROP (set using rqt_reconfigure)
	//this feature is only really necessary because there is a small hole in the mock mine that the scanner can see through and
	//it often detect potential kepoints in that area
	pass.setInputCloud (cloud_world);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (-100,CROP);
	pass.filter (*cloud_height_filtered);
	pass.setInputCloud (cloud_height_filtered);
	//filter the intensity values between VINT_MIN and VINT_MAX (set using rqt_reconfigure)
	pass.setFilterFieldName ("intensity");
	pass.setFilterLimits (VINT_MIN,VINT_MAX);
	pass.filter (*cloud_intensity_filtered);	
	//if the homing boolean is set, we dont need to localize the cloud since the scan is taken from the world origin
	if(req.homing){
		if(DEBUG) ROS_INFO("Homing scan, returning assembled_cloud");
		cloud_msg = cloud_req; //return the cloud sent to us unchanged
	}else{
		
		//container for unknown number of keypoint candidates
		std::vector<pcl::PointXYZI> Pts;
		//cluster points together
		Pts = cluster_points(cloud_intensity_filtered);
		//Open the marker.bag file 
		std::stringstream fs2;
		fs2 << req.marker_file << "marker.bag";
		ROS_INFO("Using marker file %s",fs2.str().c_str());
		bag.open(fs2.str().c_str(), rosbag::bagmode::Read);
		Eigen::Matrix4f transformB, transformA;
		rosbag::View view_marker(bag, rosbag::TopicQuery("markers"));
		phd::marker_msg new_marker;
		Eigen::Matrix4f transform_mat;
		std::vector<Eigen::Matrix4f> transform_mat_list;
		//input data from marker.bag
		marker_list.clear();
		foreach(rosbag::MessageInstance const m, view_marker){
			phd::marker_msg::ConstPtr i = m.instantiate<phd::marker_msg>();
			new_marker = *i;
			marker_list.push_back(new_marker);
		}



		marker_idx = locate_marker(Pts, doc_msgs, marker_list);
		bag.close();
		Eigen::Matrix3f marker_loc;
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_int (new pcl::PointCloud<pcl::PointXYZI>);
		Eigen::Matrix4f transformAB;
		if(marker_idx>=0){
			for(int dctr = 0; dctr < doc_msgs.size();++dctr){
				
				transformA(0,0) = marker_list[doc_msgs[dctr].id].transform[0];
				transformA(0,1) = marker_list[doc_msgs[dctr].id].transform[1];
				transformA(0,2) = marker_list[doc_msgs[dctr].id].transform[2];
				transformA(0,3) = marker_list[doc_msgs[dctr].id].transform[3];
				transformA(1,0) = marker_list[doc_msgs[dctr].id].transform[4];
				transformA(1,1) = marker_list[doc_msgs[dctr].id].transform[5];
				transformA(1,2) = marker_list[doc_msgs[dctr].id].transform[6];
				transformA(1,3) = marker_list[doc_msgs[dctr].id].transform[7];
				transformA(2,0) = marker_list[doc_msgs[dctr].id].transform[8];
				transformA(2,1) = marker_list[doc_msgs[dctr].id].transform[9];
				transformA(2,2) = marker_list[doc_msgs[dctr].id].transform[10];
				transformA(2,3) = marker_list[doc_msgs[dctr].id].transform[11];
				transformA(3,0) = marker_list[doc_msgs[dctr].id].transform[12];
				transformA(3,1) = marker_list[doc_msgs[dctr].id].transform[13];
				transformA(3,2) = marker_list[doc_msgs[dctr].id].transform[14];
				transformA(3,3) = marker_list[doc_msgs[dctr].id].transform[15];
				transformB(0,0) = doc_msgs[dctr].transform_mat[0];
				transformB(0,1) = doc_msgs[dctr].transform_mat[1];
				transformB(0,2) = doc_msgs[dctr].transform_mat[2];
				transformB(0,3) = doc_msgs[dctr].transform_mat[3];
				transformB(1,0) = doc_msgs[dctr].transform_mat[4];
				transformB(1,1) = doc_msgs[dctr].transform_mat[5];
				transformB(1,2) = doc_msgs[dctr].transform_mat[6];
				transformB(1,3) = doc_msgs[dctr].transform_mat[7];
				transformB(2,0) = doc_msgs[dctr].transform_mat[8];
				transformB(2,1) = doc_msgs[dctr].transform_mat[9];
				transformB(2,2) = doc_msgs[dctr].transform_mat[10];
				transformB(2,3) = doc_msgs[dctr].transform_mat[11];
				transformB(3,0) = doc_msgs[dctr].transform_mat[12];
				transformB(3,1) = doc_msgs[dctr].transform_mat[13];
				transformB(3,2) = doc_msgs[dctr].transform_mat[14];
				transformB(3,3) = doc_msgs[dctr].transform_mat[15];
				transformAB = transformB.inverse();
				doc_msg.transform_mat[0] = transformAB(0,0);
				doc_msg.transform_mat[1] = transformAB(0,1);
				doc_msg.transform_mat[2] = transformAB(0,2);
				doc_msg.transform_mat[3] = transformAB(0,3);
				doc_msg.transform_mat[4] = transformAB(1,0);
				doc_msg.transform_mat[5] = transformAB(1,1);
				doc_msg.transform_mat[6] = transformAB(1,2);
				doc_msg.transform_mat[7] = transformAB(1,3);
				doc_msg.transform_mat[8] = transformAB(2,0);
				doc_msg.transform_mat[9] = transformAB(2,1);
				doc_msg.transform_mat[10] = transformAB(2,2);
				doc_msg.transform_mat[11] = transformAB(2,3);
				doc_msg.transform_mat[12] = transformAB(3,0);
				doc_msg.transform_mat[13] = transformAB(3,1);
				doc_msg.transform_mat[14] = transformAB(3,2);
				doc_msg.transform_mat[15] = transformAB(3,3);
				//doc_msg.x = transform_mat_list[dctr](0,3);
				//doc_msg.y = transform_mat_list[dctr](1,3);
				//doc_msg.z = transform_mat_list[dctr](2,3);
				doc_msg.x = doc_msgs[dctr].x;
				doc_msg.id = doc_msgs[dctr].id;
				//first transform the unlocalized pointcloud to its marker's coordinate frame
				pcl::transformPointCloud (*cloud_world, *cloud_int, transformAB);
				//next transform the pointcloud from the marker coordinate frame to the world coordinate frame
				pcl::transformPointCloud (*cloud_int, *cloud_output, transformA);
				//convert the PCL cloud to a ROS cloud
				pcl::toROSMsg(*cloud_output,cloud_msg);

				//fix the naming discrepancy between ROS and PCL (from "intensities" to "intensity")
				cloud_msg.fields[3].name = "intensity";
				cloud_msg.header.frame_id = "/base_footprint";
				cloud_msg.header.stamp = ros::Time::now();
				doc_msg.cloud_out = cloud_msg;
				res.clouds.push_back(doc_msg);
				
			}
			marker = true;
		}	
	}
int cloud_ctr = 0;
	//Open file for saving marker location
			ofstream myfile;
			std::stringstream fs, pf, mn;
			fs << "/home/mike/processed/multiposes.csv";
				mn.str("");
				mn << "/home/mike/processed/tracker/";
			myfile.open (fs.str().c_str(), std::ios::out|std::ios::app);
			ROS_INFO("Response: %lu", res.clouds.size());
			float acc = 100;
			float nacc;
			float best_doc;
			float cloud_num = 0;			
			for(int sctr = 0; sctr < res.clouds.size(); ++ sctr){
				
				//Call the localization service
				loc_srv.request.cloud_in = res.clouds[sctr].cloud_out;
				loc_srv.request.homing = false;
				loc_srv.request.marker_file = mn.str();
				if(loc_client.call(loc_srv)){
					ROS_INFO("Found tracker %f - %f -%f",loc_srv.response.transform_mat[3],loc_srv.response.transform_mat[7],loc_srv.response.transform_mat[11]);	
				}
				nacc = sqrt(pow(loc_srv.response.transform_mat[3]-req.tgt_x,2)+pow(loc_srv.response.transform_mat[7]-req.tgt_y,2));
				
				if(nacc<acc){
					ROS_INFO("Best acc: %f",nacc);
					acc = nacc;
					best_doc = sctr;
					pf.str("");
					pf << "/home/mike/processed/" << req.num  << "best_V" << sctr << "-" << res.clouds[best_doc].id << ".pcd";				
					//Tell pcl what the 4th field information is
					res.clouds[sctr].cloud_out.fields[3].name = "intensity";
					pcl::fromROSMsg(res.clouds[best_doc].cloud_out, *save_cloud);
					if(DEBUG) ROS_INFO("Saving to %s",pf.str().c_str());
					if(save_cloud->size() > 0) pcl::io::savePCDFileASCII (pf.str().c_str(), *save_cloud);
					myfile << "Cloud: ," << req.num << ",";
					myfile << "Marker: ," << res.clouds[best_doc].id << ",Acc:," << nacc << ",Val:," << res.clouds[best_doc].x << ",";	
					myfile << "Tracker: ," << loc_srv.response.transform_mat[3] << "," << loc_srv.response.transform_mat[7] << "," << loc_srv.response.transform_mat[11] << ",";
					myfile << "Opti: ," << req.tgt_x << "," << req.tgt_y << ",";
					myfile << "Transform: ,";
					for(int i = 0; i <16; ++i){
						//if(i%4 == 0) myfile << std::endl;
						myfile << res.clouds[sctr].transform_mat[i] << ",";
					}
					myfile << std::endl;
				}
			}
			myfile.close();
  	return marker;
}

int main (int argc, char** argv){
	// Initialize ROS
	ros::init (argc, argv, "pcd_doctor_service");
	ros::NodeHandle nh;
	ros::ServiceServer service = nh.advertiseService("doctor_pcd", localize); 
	vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
	//Dynamic Reconfigure
	dynamic_reconfigure::Server<phd::LocalizeConfig> server;
	dynamic_reconfigure::Server<phd::LocalizeConfig>::CallbackType callback_type;
	callback_type = boost::bind(&reconfig_callback, _1, _2);
	server.setCallback(callback_type);
	loc_client = nh.serviceClient<phd::localize_cloud>("localize_pcd");
	//Visualization marker for possible marker points
	point_list.header.frame_id = "/base_footprint";
	point_list.header.stamp = ros::Time::now();
	point_list.ns = "points_and_lines";
	point_list.action = visualization_msgs::Marker::ADD;
	point_list.pose.orientation.w = 1.0;
	point_list.id = 2;
	point_list.type = visualization_msgs::Marker::POINTS;
	point_list.scale.x = 0.05;
	point_list.scale.y = 0.05;
	// Point list is red
	point_list.color.r = 1.0;
	point_list.color.b = 0.0;
	point_list.color.a = 0.5;
	ros::spin();
}