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
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <visualization_msgs/Marker.h>
#include <pcl/filters/normal_space.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/foreach.hpp>

#include "phd/trajectory_service.h"
#include "phd/trajectory_array.h"
#include "phd/trajectory_section.h"

#define DEBUG 1
#define WORKSPACE 0.5
#define HEIGHT_STEP .2
#define MAX_HEIGHT 1.3
#define OFFSET 0.1
#define POINTS 15
#define Z_HEIGHT 0.01
#define VIA_DISTANCE 0.05
#define NORM_ANGLE .1


//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;


pcl::PointCloud<pcl::PointXYZI>::Ptr cloudA (new pcl::PointCloud<pcl::PointXYZI> );
pcl::PointCloud<pcl::PointXYZI>::Ptr cloudB (new pcl::PointCloud<pcl::PointXYZI> );

pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_surface (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_surface_full (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr main_cloud (new pcl::PointCloud<pcl::PointXYZI>);
//make the publisher global so we can publish from the callback
ros::Publisher pub;
ros::Publisher marker_pub;
ros::Publisher dir_pub;
ros::Publisher path_pub;
ros::Publisher line_pub;



bool forward_sort (phd::trajectory_point i,phd::trajectory_point j) { return (i.d<j.d); }
bool reverse_sort (phd::trajectory_point i,phd::trajectory_point j) { return (i.d>j.d); }

float find_min(float i, float j){
	if(i<j) return i;
	else return j;
}
float find_max(float i, float j){
	if(i<j) return j;
	else return i;
}

//calculates the distance between two points
float vec_length(pcl::PointXYZI pointA,pcl::PointXYZI pointB){

	return sqrt(pow(pointB.x-pointA.x,2)+pow(pointB.y-pointA.y,2)+pow(pointB.z-pointA.z,2));

}

pcl::PointXYZI pt_copy(phd::trajectory_point in){
	pcl::PointXYZI out;
	out.x = in.x;
	out.y = in.y;
	out.z = in.z;
	return out;
}
phd::trajectory_point pt_copy(pcl::PointXYZI in){
	phd::trajectory_point out;
	out.x = in.x;
	out.y = in.y;
	out.z = in.z;
	return out;
}

//calculates a unit vector from the cross product
pcl::PointXYZI unit_cross(pcl::PointXYZI vectorA,pcl::PointXYZI vectorB){
	//ROS_INFO("Crossing %f - %f - %f / %f - %f - %f", vectorA.x,vectorA.y,vectorA.z,vectorB.x,vectorB.y,vectorB.z);
	pcl::PointXYZI result;
	result.z = vectorA.x*vectorB.y-vectorB.x*vectorA.y;
	result.x = vectorA.y*vectorB.z-vectorB.y*vectorA.z;
	result.y = vectorA.z*vectorB.x-vectorB.z*vectorA.x;
	float length = sqrt(pow(result.x,2)+pow(result.y,2)+pow(result.z,2));
	result.x = result.x/length;
	result.y = result.y/length;
	result.z = result.z/length;
	if(result.z < 0){
		result.x *= -1;
		result.y *= -1;
		result.z *= -1;
	}
	return result;

}
//calculates a unit vector
pcl::PointXYZI unit_vector(pcl::PointXYZI vectorA,pcl::PointXYZI vectorB){
//ROS_INFO("Crossing %f - %f - %f / %f - %f - %f", vectorA.x,vectorA.y,vectorA.z,vectorB.x,vectorB.y,vectorB.z);
	pcl::PointXYZI result;
	float d_total = vec_length(vectorA,vectorB);
	result.x = (vectorB.x-vectorA.x)/d_total;
	result.y = (vectorB.y-vectorA.y)/d_total;
	result.z = (vectorB.z-vectorA.z)/d_total;
	return result;

}
//calculates a unit vector
pcl::PointXYZI unit_vector(pcl::PointXYZI vectorA){
	pcl::PointXYZI result;
	float d_total = sqrt(pow(vectorA.x,2)+pow(vectorA.y,2)+pow(vectorA.z,2));
	result.x = (vectorA.x)/d_total;
	result.y = (vectorA.y)/d_total;
	result.z = (vectorA.z)/d_total;
	return result;

}

float dot_product(phd::trajectory_point i, phd::trajectory_point j){
	return fabs(i.nx*j.nx+i.ny*j.ny+i.nz*j.nz)/(sqrt(pow(i.nx,2)+pow(i.ny,2)+pow(i.nz,2))*sqrt(pow(j.nx,2)+pow(j.ny,2)+pow(j.nz,2)));
}

//finds nearest point to given coords
pcl::PointXYZI find_pt(pcl::PointXYZI target_pt){
	int K = 1;
	std::vector<int> pointIdxNKNSearch;
	std::vector<float> pointNKNSquaredDistance;
	pcl::PointXYZI searchPoint;
	float resolution = 128.0f;
	pcl::PointXYZI foundpt;

	pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> octree (resolution);

	octree.setInputCloud (cloud_surface);
	octree.addPointsFromInputCloud ();
//ROS_INFO("looking near %f/%f/%f",target_pt.x,target_pt.y,target_pt.z);
	if (octree.nearestKSearch (target_pt, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0){
		
		foundpt = cloud_surface->points[pointIdxNKNSearch[0]];
		//ROS_INFO("Found pt %f/%f/%f near %f/%f/%f",foundpt.x,foundpt.y,foundpt.z,target_pt.x,target_pt.y,target_pt.z);
	}else ROS_INFO("Could not find point");

	return foundpt;

}

//finds nearest point to given coords using defined cloud
pcl::PointXYZI find_pt(pcl::PointCloud<pcl::PointXYZI>::Ptr l_cloud, pcl::PointXYZI target_pt){
	int K = 1;
	std::vector<int> pointIdxNKNSearch;
	std::vector<float> pointNKNSquaredDistance;
	pcl::PointXYZI searchPoint;
	float resolution = 128.0f;
	pcl::PointXYZI foundpt;

	pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> octree (resolution);

	octree.setInputCloud (l_cloud);
	octree.addPointsFromInputCloud ();
//ROS_INFO("/ooking near %f/%f/%f",target_pt.x,target_pt.y,target_pt.z);
	if (octree.nearestKSearch (target_pt, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0){
		
		foundpt = l_cloud->points[pointIdxNKNSearch[0]];
		//ROS_INFO("Found pt %f/%f/%f near %f/%f/%f",foundpt.x,foundpt.y,foundpt.z,target_pt.x,target_pt.y,target_pt.z);
	}else ROS_INFO("Could not find point");

	return foundpt;

}



//find normal at given pt
pcl::PointXYZI find_normal(float x, float y, float z){
	pcl::PointXYZI target_pt;
	target_pt.x = x;
	target_pt.y = y;
	target_pt.z = z;
	pcl::PointXYZI foundpt;
	pcl::PointCloud<pcl::PointXYZI> cloudB;
	cloudB.push_back (target_pt);	
	//ROS_INFO("Estimating Normal at %f - %f - %f",target_pt.x,target_pt.y,target_pt.z);
	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
	// set parameters
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloudptr = cloudB.makeShared();
	ne.setInputCloud (cloudptr);
	// Pass the original data (before downsampling) as the search surface
	ne.setSearchSurface (cloud_surface_full);
	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given surface dataset.
	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI> ());
	ne.setSearchMethod (tree);

	// Output datasets
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal> ());

	// Use all neighbors in a sphere of radius 3cm
	ne.setRadiusSearch (0.05);
	ne.setViewPoint (0, 0, 0);
	// Compute the features
	ne.compute (*cloud_normals);
	foundpt.x = -cloud_normals->points[0].normal[0];
	foundpt.y = -cloud_normals->points[0].normal[1];
	foundpt.z = -cloud_normals->points[0].normal[2];
	//if(sqrt(pow(target_pt.x-foundpt.x
	return foundpt;

}

void show_markers(phd::trajectory_array t_array){

	uint32_t shape = visualization_msgs::Marker::LINE_LIST;
	visualization_msgs::Marker marker;
	visualization_msgs::Marker path;
	visualization_msgs::Marker surface_path;
	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
	marker.header.frame_id = "/base_link";
	marker.header.stamp = ros::Time::now();
	marker.ns = "basic_shapes";
	marker.id = 0;
	marker.type = shape;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.01;
	marker.scale.y = 0.01;
	//marker.scale.y = 0.015;
	marker.color.r = 0.0f;
	marker.color.g = 0.0f;
	marker.color.b = 1.0f;
	marker.color.a = 0.8;
	marker.lifetime = ros::Duration();
	
	    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
	path.header.frame_id = "/base_link";
	path.header.stamp = ros::Time::now();
	// Set the namespace and id for this path.  This serves to create a unique ID
	// Any path sent with the same namespace and id will overwrite the old one
	path.ns = "path_shapes";
	path.id = 0;
	// Set the path type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
	path.type = visualization_msgs::Marker::LINE_STRIP;
	// Set the path action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	path.action = visualization_msgs::Marker::ADD;
	// Set the pose of the path.  This is a full 6DOF pose relative to the frame/time specified in the header
	path.pose.position.x = 0;
	path.pose.position.y = 0;
	path.pose.position.z = 0;
	path.pose.orientation.x = 0.0;
	path.pose.orientation.y = 0.0;
	path.pose.orientation.z = 0.0;
	path.pose.orientation.w = 1.0;
	// Set the scale of the path -- 1x1x1 here means 1m on a side
	path.scale.x = 0.01;
	// Set the color -- be sure to set alpha to something non-zero!
	path.color.r = 0.0f;
	path.color.g = 1.0f;
	path.color.b = 0.0f;
	path.color.a = 1.0;
	path.lifetime = ros::Duration();	    
	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
	surface_path.header.frame_id = "/base_link";
	surface_path.header.stamp = ros::Time::now();
	// Set the namespace and id for this path.  This serves to create a unique ID
	// Any path sent with the same namespace and id will overwrite the old one
	surface_path.ns = "surface_path_shapes";
	surface_path.id = 0;
	// Set the path type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
	surface_path.type = visualization_msgs::Marker::LINE_STRIP;
	// Set the path action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	surface_path.action = visualization_msgs::Marker::ADD;
	// Set the pose of the path.  This is a full 6DOF pose relative to the frame/time specified in the header
	surface_path.pose.position.x = 0;
	surface_path.pose.position.y = 0;
	surface_path.pose.position.z = 0;
	surface_path.pose.orientation.x = 0.0;
	surface_path.pose.orientation.y = 0.0;
	surface_path.pose.orientation.z = 0.0;
	surface_path.pose.orientation.w = 1.0;
	// Set the scale of the path -- 1x1x1 here means 1m on a side
	surface_path.scale.x = 0.01;
	// Set the color -- be sure to set alpha to something non-zero!
	surface_path.color.r = 1.0f;
	surface_path.color.g = 0.0f;
	surface_path.color.b = 0.0f;
	surface_path.color.a = 1.0;
	surface_path.lifetime = ros::Duration();
	phd::trajectory_msg t_msg;
	for(int ctr = 0; ctr < t_array.sections.size(); ++ctr){

		path.id = ctr;
		surface_path.id = ctr;
		marker.id = ctr;
		path.color.b = 0.2*ctr;
		marker.color.r = 0.2*ctr;
		t_msg.points.swap(t_array.sections[ctr].points);
		int num_pts = t_msg.points.size();
		path.points.resize(num_pts);
		surface_path.points.resize(num_pts);
		marker.points.resize(2*num_pts);
		for(int i = 0; i < num_pts; ++i){
		
			//ROS_INFO("SPoint %d: %f- %f- %f",i,t_msg.points[i].x,t_msg.points[i].y,t_msg.points[i].z);
			//ROS_INFO("Normal %d: %f- %f- %f",i,t_msg.points[i].nx,t_msg.points[i].ny,t_msg.points[i].nz);
			path.points[i].x = t_msg.points[i].x-(OFFSET*t_msg.points[i].nx);
			path.points[i].y = t_msg.points[i].y-(OFFSET*t_msg.points[i].ny);
			path.points[i].z = t_msg.points[i].z-(OFFSET*t_msg.points[i].nz);
			surface_path.points[i].x = t_msg.points[i].x;
			surface_path.points[i].y = t_msg.points[i].y;
			surface_path.points[i].z = t_msg.points[i].z;

			//ROS_INFO("Point %d: %f- %f- %f",i,path.points[i].x,path.points[i].y,path.points[i].z);
			marker.points[i*2].x = t_msg.points[i].x-(OFFSET*t_msg.points[i].nx);
			marker.points[i*2].y = t_msg.points[i].y-(OFFSET*t_msg.points[i].ny);
			marker.points[i*2].z = t_msg.points[i].z-(OFFSET*t_msg.points[i].nz);
			marker.points[(i*2)+1].x = t_msg.points[i].x;
			marker.points[(i*2)+1].y = t_msg.points[i].y;
			marker.points[(i*2)+1].z = t_msg.points[i].z;
		}

		dir_pub.publish(surface_path);
		ros::Duration(0.5).sleep();
		path_pub.publish(path);
		ros::Duration(0.5).sleep();
		marker_pub.publish(marker);
		ros::Duration(0.5).sleep();
	}
	

}

void show_marker(pcl::PointXYZI one,pcl::PointXYZI two){

	uint32_t shape = visualization_msgs::Marker::LINE_LIST;
	visualization_msgs::Marker marker;
	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
	marker.header.frame_id = "/base_link";
	marker.header.stamp = ros::Time::now();
	marker.ns = "basic_shapes";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.01;
	marker.scale.y = 0.01;
	//marker.scale.y = 0.015;
	marker.color.r = 0.0f;
	marker.color.g = 0.0f;
	marker.color.b = 1.0f;
	marker.color.a = 0.8;
	marker.lifetime = ros::Duration();
	

		marker.id = 0;
		marker.points.resize(2);

			//ROS_INFO("Point %d: %f- %f- %f",i,path.points[i].x,path.points[i].y,path.points[i].z);
			marker.points[0].x = one.x;
			marker.points[0].y = one.y;
			marker.points[0].z = one.z;
			marker.points[1].x = two.x;
			marker.points[1].y = two.y;
			marker.points[1].z = two.z;

		marker_pub.publish(marker);
		ros::Duration(0.5).sleep();

	

}


phd::trajectory_array calc_line_points(float z_val, float P1x, float P1y, float P2x, float P2y, int dir){

	pcl::PointCloud<pcl::PointXYZI>::Ptr line (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PassThrough<pcl::PointXYZI> pass;
	//filter the data for intensity
	pass.setInputCloud (cloud_surface);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (z_val - Z_HEIGHT,z_val + Z_HEIGHT);
	pass.filter (*line);
	pass.setInputCloud (line);
	/*float xmin = find_min(P1x,P2x);
	float ymin = find_min(P1y,P2y);
	float xmax = find_max(P1x,P2x);
	float ymax = find_max(P1y,P2y);
	if(xmax-xmin>ymax-ymin){
	pass.setFilterFieldName ("x");
	pass.setFilterLimits (xmin,xmax);
	pass.filter (*line);
	}
	else{
	pass.setFilterFieldName ("y");
	pass.setFilterLimits (ymin,ymax);
	pass.filter (*line);
	}*/
	sensor_msgs::PointCloud2 line_cloud;
	pcl::toROSMsg(*line,line_cloud);
	line_pub.publish(line_cloud);

	pcl::PointXYZI start_pt;
	pcl::PointXYZI search_pt;
	phd::trajectory_point end_pt;
	search_pt.x = P1x;
	search_pt.y = P1y;
	search_pt.z = z_val;
	std::vector<phd::trajectory_point> line_points;
	std::vector<std::vector<phd::trajectory_point> > line_sections;
	phd::trajectory_point point_holder;
	phd::trajectory_array t_array;
	while(line->points.size()>0){		
		end_pt.d = 0;
		int lctr = 0;
		ROS_INFO("Size %d",line->points.size());
		for(pcl::PointCloud<pcl::PointXYZI>::iterator ctr = line->begin(); ctr < line->end(); ++ctr){
			point_holder.x = ctr->x;
			point_holder.y = ctr->y;
			point_holder.z = ctr->z;
			pcl::PointXYZI normal_pt = find_normal(ctr->x,ctr->y,ctr->z);
			point_holder.nx = normal_pt.x;
			point_holder.ny = normal_pt.y;
			point_holder.nz = normal_pt.z;
			point_holder.d = sqrt(pow(ctr->x-search_pt.x,2)+pow(ctr->y-search_pt.y,2));
			ROS_INFO("Point %f - %f - %f / %f",ctr->x,ctr->y,ctr->z,point_holder.d);
			if(point_holder.d < WORKSPACE){
				if(point_holder.d > end_pt.d) end_pt = point_holder;
				line_points.push_back(point_holder);
				line->erase(ctr);
			}
			++lctr;
			ROS_INFO("Loop %d",lctr);
			
		}
		
		search_pt.x = end_pt.x;
		search_pt.y = end_pt.y;
		if(dir == 1)		std::sort (line_points.begin(), line_points.end(), forward_sort);
		else		std::sort (line_points.begin(), line_points.end(), reverse_sort);
		ROS_INFO("Pushing %d points", line_points.size());
		line_sections.push_back(line_points);
		line_points.clear();	
		ROS_INFO("%d points remaining",line->points.size());
	}

	phd::trajectory_msg t_msg;
	for(int ictr = 0; ictr < line_sections.size(); ++ictr){
	int sizer = line_sections.size();
	ROS_INFO("line section size %d",sizer);
		line_points.clear();
		line_points = line_sections[ictr];
		if(dir == 1){
	ROS_INFO("Normal");
			t_msg.points.push_back(line_points[0]);
			int via_ctr = 0;
		  	for (int ctr = 1; ctr<line_points.size(); ++ctr){
				//ROS_INFO("Normal:  %f",dot_product(line_points[0],line_points[ctr]));
				if(line_points[ctr].d-line_points[via_ctr].d>VIA_DISTANCE){
					t_msg.points.push_back(line_points[ctr]);
					via_ctr = ctr;
					//ROS_INFO("Distance Point: %f - %f - %f / %f",line_points[ctr].x,line_points[ctr].y,line_points[ctr].z,line_points[ctr].d);
				}		
				else if(dot_product(line_points[via_ctr],line_points[ctr])<NORM_ANGLE){
					t_msg.points.push_back(line_points[ctr]);
					via_ctr = ctr;
					ROS_INFO("Normal Point: %f - %f - %f / %f",line_points[ctr].x,line_points[ctr].y,line_points[ctr].z,dot_product(line_points[via_ctr],line_points[ctr]));
				}
			}
		}
		else{
	ROS_INFO("Normal2");
			int via_ctr = line_points.size() - 1;
			t_msg.points.push_back(line_points[via_ctr]);
			for (int ctr = via_ctr - 1; ctr>=0; --ctr){
				//ROS_INFO("Normal:  %f",dot_product(line_points[0],line_points[ctr]));
				if(line_points[via_ctr].d-line_points[ctr].d>VIA_DISTANCE){
					t_msg.points.push_back(line_points[ctr]);
					via_ctr = ctr;
					//ROS_INFO("Distance Point: %f - %f - %f / %f",line_points[ctr].x,line_points[ctr].y,line_points[ctr].z,line_points[ctr].d);
				}		
				else if(dot_product(line_points[via_ctr],line_points[ctr])<NORM_ANGLE){
					t_msg.points.push_back(line_points[ctr]);
					via_ctr = ctr;
					ROS_INFO("Normal Point: %f - %f - %f / %f",line_points[ctr].x,line_points[ctr].y,line_points[ctr].z,dot_product(line_points[via_ctr],line_points[ctr]));
				}
			}
		}
	t_array.sections.push_back(t_msg);
	t_msg.points.clear();
	}
	ROS_INFO("Done line points");
	return t_array;

}

phd::trajectory_msg sort_line(pcl::PointCloud<pcl::PointXYZI>::Ptr line, pcl::PointXYZI start_point, int sort_dir){

	phd::trajectory_msg sorted;
	phd::trajectory_point point_holder;
	pcl::PointXYZI start_pt = find_pt(line,start_point);
	pcl::PointXYZI dir, search_pt, d_pt, ctr_pt, ctr_pt2;
	//ROS_INFO("Starting %f - %f - %f",start_point.x,start_point.y,start_point.z);
	float d, d_total;
	float dir_val, old_d;
	//for(pcl::PointCloud<pcl::PointXYZI>::iterator ctr = line->begin(); ctr < line->end(); ++ctr) ROS_INFO("P: %f - %f - %f", ctr->x,ctr->y,ctr->z);
	for(pcl::PointCloud<pcl::PointXYZI>::iterator ctr = line->begin(); ctr < line->end(); ++ctr){
		//ROS_INFO("CP: %f - %f - %f", ctr->x,ctr->y,ctr->z);
		d_total = 0;	
		dir_val = 0.01;
		//ctr_pt = start_pt;
		ctr_pt2 = *ctr;
//ROS_INFO("Start %f - %f - %f", start_pt.x,start_pt.y,start_pt.z);
		old_d = 100;
		while(sqrt(pow(ctr_pt2.x-start_pt.x,2)+pow(ctr_pt2.y-start_pt.y,2)) > 0.01 ){
			dir = unit_vector(ctr_pt2,start_pt);
			search_pt.x = ctr_pt2.x + dir_val*dir.x;
			search_pt.y = ctr_pt2.y + dir_val*dir.y;
			search_pt.z = ctr_pt2.z;
			//if(ctr->x!=ctr_pt.x){
			ctr_pt = find_pt(line,search_pt);
			//ROS_INFO("Walking %f -- %f",sqrt(pow(ctr_pt.x-ctr_pt2.x,2)+pow(ctr_pt.y-ctr_pt2.y,2)),d_total);
			//if(old_d < sqrt(pow(ctr->x-ctr_pt.x,2)+pow(ctr->y-ctr_pt.y,2))){
			//	ROS_INFO("Breaking");				
				//break;
			//}
			if(ctr_pt.x != ctr_pt2.x || ctr_pt.y != ctr_pt2.y){
				d_total+=sqrt(pow(ctr_pt.x-ctr_pt2.x,2)+pow(ctr_pt.y-ctr_pt2.y,2));
				ctr_pt2 = ctr_pt;
				dir_val = 0.01;
			}else{
				dir_val+=0.01;
				//ROS_INFO("Expanding search");
			}
		}
		point_holder.d = d_total;
		point_holder.d_abs = sqrt(pow(ctr->x-start_pt.x,2)+pow(ctr->y-start_pt.y,2));
		point_holder.x = ctr->x;
		point_holder.y = ctr->y;
		point_holder.z = ctr->z;
		pcl::PointXYZI normal_pt = find_normal(ctr->x,ctr->y,ctr->z);
		point_holder.nx = normal_pt.x;
		point_holder.ny = normal_pt.y;
		point_holder.nz = normal_pt.z;
		sorted.points.push_back(point_holder);
		ctr->intensity = point_holder.d;
		/*if(point_holder.d<WORKSPACE){
			ROS_INFO("Deleting");			
			 ctr = line->erase(ctr);		
		}*/
	
	}
	//for(int i = 0; i < sorted.points.size(); ++i) ROS_INFO("P %f",sorted.points[i].d);
	if(sort_dir ==1) std::sort (sorted.points.begin(), sorted.points.end(), forward_sort);
	else std::sort (sorted.points.begin(), sorted.points.end(), reverse_sort);
	sorted.start_pt = pt_copy(start_pt);
	return sorted;

}
phd::trajectory_msg recalc_d(phd::trajectory_msg sect, pcl::PointXYZI start_pt){
	phd::trajectory_msg ret;
	phd::trajectory_point pt_holder;
	for(int ctr = 0; ctr < sect.points.size(); ++ctr){
		pt_holder = sect.points[ctr];
		pt_holder.d_abs =  sqrt(pow(pt_holder.x-start_pt.x,2)+pow(pt_holder.y-start_pt.y,2));
		ret.points.push_back(pt_holder);
	}
	return ret;
}
phd::trajectory_array calc_points(float z_val, float P1x, float P1y, float zmax){

	pcl::PointCloud<pcl::PointXYZI>::Ptr line (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PassThrough<pcl::PointXYZI> pass;
	//filter the data for intensity
	phd::trajectory_point point_holder;
	pcl::PointXYZI search_pt;
	pcl::PointXYZI via_pt;
	search_pt.x = P1x;
	search_pt.y = P1y;
	pcl::PointXYZI start_pt, end_pt;
	std::vector<phd::trajectory_section> t_chunks;
	phd::trajectory_msg chunk;
	int cctr = 1;
	int dir = 1;
	phd::trajectory_array t_array,sorted_array, remainder_array;
	phd::trajectory_msg sorted_line, remainder, remainder_full, remainder_d, remainder_two;
	float max_d;

	while(z_val < zmax){

	ROS_INFO("zval %f, zmax %f",z_val,zmax);
		pass.setInputCloud (cloud_surface);
		pass.setFilterFieldName ("z");
		pass.setFilterLimits (z_val - Z_HEIGHT,z_val + Z_HEIGHT);
		pass.filter (*line);
		search_pt.z = z_val;
		sorted_line = sort_line(line, search_pt,dir);
		sorted_array.sections.push_back(sorted_line);
		z_val+=HEIGHT_STEP;
		dir = dir * -1;
	}
	ROS_INFO("Done while");
	//remainder_array.sections.resize(sorted_array.sections.size());
	for(int actr = 0; actr<sorted_array.sections.size(); ++actr){
		start_pt = pt_copy(sorted_array.sections[actr].points[0]);
		if(actr%2==0) chunk.points.push_back(sorted_array.sections[actr].points[0]);
		via_pt = start_pt;
		max_d = 0;
		for(int ctr = 0; ctr < sorted_array.sections[actr].points.size(); ++ctr){
			if(sorted_array.sections[actr].points[ctr].d_abs < WORKSPACE){
				if(sqrt(pow(sorted_array.sections[actr].points[ctr].x-via_pt.x,2)
					+pow(sorted_array.sections[actr].points[ctr].y-via_pt.y,2))>VIA_DISTANCE){
					// || dot_product(line_points[via_ctr],line_points[ctr])<NORM_ANGLE)
					chunk.points.push_back(sorted_array.sections[actr].points[ctr]);
					via_pt = pt_copy(sorted_array.sections[actr].points[ctr]);
					if(sorted_array.sections[actr].points[ctr].d>max_d){
						end_pt = pt_copy(sorted_array.sections[actr].points[ctr]);
						max_d = sorted_array.sections[actr].points[ctr].d;
					}
				}
			}
			else{
				remainder.points.push_back(sorted_array.sections[actr].points[ctr]);
			}
		}
		if(actr%2==1) chunk.points.push_back(sorted_array.sections[actr].points[sorted_array.sections[actr].points.size()-1]);
		remainder_d = recalc_d(remainder, end_pt);
		remainder.points.clear();
		remainder_array.sections.push_back(remainder_d);
		remainder_d.points.clear();
	}
	t_array.sections.push_back(chunk);
	chunk.points.clear();
	bool complete = false;
ROS_INFO("Done while");
	int r_pts = remainder_array.sections[0].points.size()+1;
	while(remainder_array.sections[0].points.size()>0 && remainder_array.sections[0].points.size() != r_pts){
		r_pts = remainder_array.sections[0].points.size();
		for(int actr = 0; actr<remainder_array.sections.size(); ++actr){
			start_pt = pt_copy(remainder_array.sections[actr].points[0]);
			if(actr%2==0 && remainder_array.sections[actr].points[0].d != 0) chunk.points.push_back(remainder_array.sections[actr].points[0]);
			via_pt = start_pt;
			max_d = 0;
			for(int ctr = 0; ctr < remainder_array.sections[actr].points.size(); ++ctr){
				if(remainder_array.sections[actr].points[ctr].d_abs < WORKSPACE){
					if(sqrt(pow(remainder_array.sections[actr].points[ctr].x-via_pt.x,2)
						+pow(remainder_array.sections[actr].points[ctr].y-via_pt.y,2))>VIA_DISTANCE){
						// || dot_product(line_points[via_ctr],line_points[ctr])<NORM_ANGLE)
						chunk.points.push_back(remainder_array.sections[actr].points[ctr]);
						via_pt = pt_copy(remainder_array.sections[actr].points[ctr]);
						if(remainder_array.sections[actr].points[ctr].d>max_d){
							end_pt = pt_copy(remainder_array.sections[actr].points[ctr]);
							max_d = remainder_array.sections[actr].points[ctr].d;
						}
					}
				}
				else{ 

					remainder.points.push_back(remainder_array.sections[actr].points[ctr]);

				}
			}
			if(actr%2==1 && remainder_array.sections[actr].points[remainder_array.sections[actr].points.size()-1].d != 0) chunk.points.push_back(remainder_array.sections[actr].points[remainder_array.sections[actr].points.size()-1]);

			remainder_d = recalc_d(remainder, end_pt);
			remainder.points.clear();
			remainder_array.sections[actr].points.clear();
			remainder_array.sections[actr].points.swap(remainder_d.points);
			remainder_d.points.clear();
		}
		t_array.sections.push_back(chunk);
		chunk.points.clear();
	}
		
	sensor_msgs::PointCloud2 line_cloud;
	pcl::toROSMsg(*line,line_cloud);
	line_pub.publish(line_cloud);
	show_markers(t_array);
	return t_array;

}


bool generate (phd::trajectory_service::Request  &req,
         phd::trajectory_service::Response &res)
{
	ROS_INFO("Received Trajectory Request");
	pcl::PointCloud<pcl::PointXYZI> cloudB;
	sensor_msgs::PointCloud2 cloud_msg1 = req.cloud_surface;
	sensor_msgs::PointCloud2 cloud_msg2 = req.cloud_in;
	//fix the naming discrepancy between ROS and PCL (from "intensities" to "intensity")
	cloud_msg1.fields[3].name = "intensity";
	pcl::fromROSMsg(cloud_msg1,*cloud_surface_full);
	cloud_msg2.fields[3].name = "intensity";
	pcl::fromROSMsg(cloud_msg2,*cloud_surface);
	phd::trajectory_msg t_msg_full, t_msg, t_remainder, t_second_remainder;
	phd::trajectory_array t_array, t_msg_array;
	std::vector<phd::trajectory_array> t_msg_array_vect;
	t_msg_full.points.clear();
	t_remainder.points.clear();
	float zmax = -10;
	float zmin = 100;
	BOOST_FOREACH (pcl::PointXYZI& p, cloud_surface->points){

		if(p.z > zmax) zmax = p.z;
		if(p.z < zmin) zmin = p.z;

	}
	float z_val = req.P1z;
	std::vector<float> z_vals;
	//ROS_INFO("KK %f - %f",zmax,zmin);
	int dir = 1;	
	float workspace_width = WORKSPACE;
	bool section_complete = true;	
	phd::trajectory_point end_pt;
	end_pt.d = 0;
	pcl::PointXYZI search_pt;
	search_pt.x = req.P1x;
	search_pt.y = req.P1y;
	search_pt.z = z_val;
	//pcl::PointXYZI start_pt = find_pt(search_pt);
	t_msg_array = calc_points(z_val, search_pt.x, search_pt.y, zmax);
		/*for(int i = 0; i < t_msg.points.size(); ++i){
			if(t_msg.points[i].d < workspace_width){
				t_msg_full.points.push_back(t_msg.points[i]);
				if(t_msg.points[i].d > end_pt.d) end_pt = t_msg.points[i];
				ROS_INFO("Adding (%d) %f - %f - %f/%f - %f - %f / %f",i,t_msg.points[i].x,t_msg.points[i].y,t_msg.points[i].z,t_msg.points[i].nx,t_msg.points[i].ny,t_msg.points[i].nz,t_msg.points[i].d);
			}
			else{
				ROS_INFO("Remainer %f",t_msg.points[i].d);
				section_complete = false;
				t_remainder.points.push_back(t_msg.points[i]);
			}
		}
		t_msg.points.clear();*/
	/*	dir = dir * (-1);
		t_msg_array_vect.push_back(t_msg_array);
		t_msg_array.sections.clear();
			
	//}
	int sec_count = 0;
	for(int ctr = 0; ctr < t_msg_array_vect.size(); ++ctr){
		if(t_msg_array_vect[ctr].sections.size() > sec_count) sec_count = t_msg_array_vect[ctr].sections.size();
	}

	phd::trajectory_msg v_sec;
	for(int sctr = 0; sctr < sec_count; ++sctr){

		for(int ctr = 0; ctr < t_msg_array_vect.size(); ++ctr){

			for(int vctr = 0; vctr < t_msg_array_vect[ctr].sections[sctr].points.size(); ++vctr){

				v_sec.points.push_back(t_msg_array_vect[ctr].sections[sctr].points[vctr]);

			}

		}
		t_array.sections.push_back(v_sec);

	}
	/*	
	t_array.sections.push_back(t_msg_full);
	res.sections.push_back(t_msg_full);
	int secnum = 2;
	while(section_complete == false){
		ROS_INFO("calling function");
		t_remainder = recalc_d(t_remainder,end_pt);
		ROS_INFO("done function");
		end_pt.d = 0;
		//workspace_width += WORKSPACE;
		section_complete = true;
		t_msg_full.points.clear();
		for(int i = 0; i < t_remainder.points.size(); ++i){
			if(t_remainder.points[i].d < workspace_width){
				if(t_remainder.points[i].d > end_pt.d) end_pt = t_remainder.points[i];
				t_msg_full.points.push_back(t_remainder.points[i]);
				ROS_INFO("Adding Section %d (%d) %f - %f - %f/%f - %f - %f / %f",secnum,i,t_remainder.points[i].x,t_remainder.points[i].y,t_remainder.points[i].z,t_remainder.points[i].nx,t_remainder.points[i].ny,t_remainder.points[i].nz,t_remainder.points[i].d);
			}
			else{
				section_complete = false;
				t_second_remainder.points.push_back(t_remainder.points[i]);
			}
		}
		++secnum;
		t_remainder.points.swap(t_second_remainder.points);
		t_second_remainder.points.clear();	
		t_array.sections.push_back(t_msg_full);
		res.sections.push_back(t_msg_full);
	}
	
	show_markers(t_array);*/
	bool ret = true;
	return ret;
}


int
main (int argc, char** argv)
{
	


	// Initialize ROS
	ros::init (argc, argv, "trajectory_generation_service");
	ros::NodeHandle nh;
	ros::ServiceServer service = nh.advertiseService("trajectory_gen", generate); 
	marker_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
	line_pub = nh.advertise<sensor_msgs::PointCloud2>( "line_points", 0 );
	path_pub = nh.advertise<visualization_msgs::Marker>( "path_marker", 0 );
	dir_pub = nh.advertise<visualization_msgs::Marker>( "dir_marker", 0 );
	ROS_INFO("Trajectory Generator online");
	ros::spin();





ros::spin();
  
  
}
