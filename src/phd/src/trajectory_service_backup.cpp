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
ROS_INFO("Crossing %f - %f - %f / %f - %f - %f", vectorA.x,vectorA.y,vectorA.z,vectorB.x,vectorB.y,vectorB.z);
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
ROS_INFO("looking near %f/%f/%f",target_pt.x,target_pt.y,target_pt.z);
	if (octree.nearestKSearch (target_pt, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0){
		
		foundpt = cloud_surface->points[pointIdxNKNSearch[0]];
		ROS_INFO("Found pt %f/%f/%f near %f/%f/%f",foundpt.x,foundpt.y,foundpt.z,target_pt.x,target_pt.y,target_pt.z);
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
		path.color.b = 0.25*ctr;
		marker.color.r = 0.25*ctr;
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

void show_chunks(std::vector<phd::trajectory_section> t_chunks){
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
	marker.color.a = 1;
	marker.lifetime = ros::Duration();
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
	path.header.frame_id = "/base_link";
	path.header.stamp = ros::Time::now();
	// Set the namespace and id for this path.  This serves to create a unique ID
	// Any path sent with the same namespace and id will overwrite the old one
	path.ns = "path_shapes";
	path.id = 0;
	// Set the path type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
	path.type = visualization_msgs::Marker::CUBE;
	// Set the path action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	path.action = visualization_msgs::Marker::ADD;
	// Set the pose of the path.  This is a full 6DOF pose relative to the frame/time specified in the header
	path.pose.orientation.x = 0.0;
	path.pose.orientation.y = 0.0;
	path.pose.orientation.z = 0.0;
	path.pose.orientation.w = 1.0;
	// Set the scale of the path -- 1x1x1 here means 1m on a side
	path.scale.x = 0.01;
	path.scale.y = 0.01;
	path.scale.z = 0.01;
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
	surface_path.type = visualization_msgs::Marker::CUBE;
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
	surface_path.scale.y = 0.01;
	surface_path.scale.z = 0.01;
	// Set the color -- be sure to set alpha to something non-zero!
	surface_path.color.r = 1.0f;
	surface_path.color.g = 0.0f;
	surface_path.color.b = 0.0f;
	surface_path.color.a = 1.0;
	surface_path.lifetime = ros::Duration();
	
	phd::trajectory_msg t_msg;
	ROS_INFO("chunks %d",t_chunks.size());
		int num_pts = t_chunks.size();
		marker.points.resize(2);
	for(int ctr = 0; ctr < t_chunks.size(); ++ctr){

		marker.id = ctr;
		marker.color.r = 0.02*ctr;		
		path.id = ctr;
		path.color.r = 0.02*ctr;
		//t_msg.points.swap(t_chunks[ctr].points);
		
	path.pose.position.x = t_chunks[ctr].start_point.x;
	path.pose.position.y = t_chunks[ctr].start_point.y;
	path.pose.position.z = t_chunks[ctr].start_point.z;		
		surface_path.id = ctr;
		surface_path.color.g = 0.1*ctr;
		//t_msg.points.swap(t_chunks[ctr].points);
		
	surface_path.pose.position.x = t_chunks[ctr].start_point.x;
	surface_path.pose.position.y = t_chunks[ctr].start_point.y;
	surface_path.pose.position.z = t_chunks[ctr].start_point.z;
			//ROS_INFO("Point %d: %f- %f- %f",i,path.points[i].x,path.points[i].y,path.points[i].z);
			marker.points[0].x = t_chunks[ctr].start_point.x;
			marker.points[0].y = t_chunks[ctr].start_point.y;
			marker.points[0].z = t_chunks[ctr].start_point.z;
			marker.points[1].x = t_chunks[ctr].end_point.x;
			marker.points[1].y = t_chunks[ctr].end_point.y;
			marker.points[1].z = t_chunks[ctr].end_point.z;



		dir_pub.publish(surface_path);
		ros::Duration(0.5).sleep();
		path_pub.publish(path);
		ros::Duration(0.5).sleep();
		marker_pub.publish(marker);
		ros::Duration(0.5).sleep();
	}
	


	

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


phd::trajectory_msg sort_line(pcl::PointCloud<pcl::PointXYZI>::Ptr line, pcl::PointXYZI start_pt){

	phd::trajectory_msg sorted;
	phd::trajectory_point point_holder;
	for(pcl::PointCloud<pcl::PointXYZI>::iterator ctr = line->begin(); ctr < line->end(); ++ctr){

		point_holder.d = sqrt(pow(ctr->x-start_pt.x,2)+pow(ctr->y-start_pt.y,2));
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
	std::sort (sorted.points.begin(), sorted.points.end(), forward_sort);
	for(int i = 0; i < sorted.points.size(); ++i) ROS_INFO("P %f",sorted.points[i].d);
	return sorted;

}

phd::trajectory_array calc_points(float z_val, float P1x, float P1y, float P2x, float P2y, int dir){

	pcl::PointCloud<pcl::PointXYZI>::Ptr line (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PassThrough<pcl::PointXYZI> pass;
	//filter the data for intensity
	pass.setInputCloud (cloud_surface);
	pass.setFilterFieldName ("z");
	float zmax = -10;
	float zmin = 100;
	BOOST_FOREACH (pcl::PointXYZI& p, cloud_surface->points){

		if(p.z > zmax) zmax = p.z;
		if(p.z < zmin) zmin = p.z;

	}
	phd::trajectory_point point_holder;
	pcl::PointXYZI search_pt;
	pcl::PointXYZI via_pt;
	search_pt.x = P1x;
	search_pt.y = P1y;
	search_pt.z = z_val;
	pcl::PointXYZI start_pt = find_pt(search_pt);
	std::vector<phd::trajectory_section> t_chunks;
	phd::trajectory_section chunk;
	int cctr = 1;
	while(z_val < zmax){

	pass.setInputCloud (cloud_surface);
	pass.setFilterFieldName ("z");
		pass.setFilterLimits (z_val - Z_HEIGHT,z_val + Z_HEIGHT);
		pass.filter (*line);

		while(line->points.size()>0){

			phd::trajectory_msg sorted_line = sort_line(line, start_pt);

	pass.setInputCloud (line);
			pass.setFilterFieldName ("intensity");
			pass.setFilterLimits (WORKSPACE, 10000);
			pass.filter (*line);
			ROS_INFO("Points left %d",sorted_line.points.size());
			chunk.end_point.d=0;
			chunk.start_point = pt_copy(start_pt);
			chunk.z_height = z_val;
			via_pt = start_pt;
			for(int ctr = 0; ctr < sorted_line.points.size(); ++ctr){
				if(sorted_line.points[ctr].d < WORKSPACE){
					if(sqrt(pow(sorted_line.points[ctr].x-via_pt.x,2)+pow(sorted_line.points[ctr].y-via_pt.y,2))>VIA_DISTANCE){// || dot_product(line_points[via_ctr],line_points[ctr])<NORM_ANGLE)
						chunk.points.push_back(sorted_line.points[ctr]);
						via_pt = pt_copy(sorted_line.points[ctr]);
						if(sorted_line.points[ctr].d>chunk.end_point.d){
							chunk.end_point = sorted_line.points[ctr];
							ROS_INFO("End point");
						}
					}
				}
			}
			if(chunk.end_point.d > 0){
				start_pt = pt_copy(chunk.end_point);
				if(dir == 1)	std::sort (chunk.points.begin(), chunk.points.end(), forward_sort);
				else		std::sort (chunk.points.begin(), chunk.points.end(), reverse_sort);
				t_chunks.push_back(chunk);
				ROS_INFO("Pushing %d",cctr);
				++cctr;
				chunk.points.clear();
			}else sorted_line.points.clear();
		}
		z_val+=HEIGHT_STEP;
	}
			
		sensor_msgs::PointCloud2 line_cloud;
		pcl::toROSMsg(*line,line_cloud);
		line_pub.publish(line_cloud);
	ROS_INFO("Made %d Chunks", t_chunks.size());
	show_chunks(t_chunks);
	phd::trajectory_array t_array;
	return t_array;

}

phd::trajectory_msg recalc_d(phd::trajectory_msg t_points, phd::trajectory_point end_pt){
	phd::trajectory_msg t_ret;
	for(int i = 0; i < t_points.points.size(); ++i){
 		t_ret.points.push_back(t_points.points[i]);
		t_ret.points[i].d = sqrt(pow(t_points.points[i].x-end_pt.x,2)+pow(t_points.points[i].y-end_pt.y,2));
	}	
		ROS_INFO("finished function");
	return t_ret;

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
	pcl::PointXYZI start_pt = find_pt(search_pt);
	/*while(z_val < zmax){
		z_vals.push_back(z_val);
		z_val += HEIGHT_STEP;
	}*/
		
		t_msg_array = calc_points(z_val, start_pt.x, start_pt.y, req.P2x, req.P2y, dir);
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
