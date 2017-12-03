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
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <visualization_msgs/Marker.h>
#include <pcl/filters/normal_space.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/crop_box.h>
#include <boost/foreach.hpp>
//User defined messages
#include "phd/simple_trajectory_service.h"
#include "phd/trajectory_array.h"
#include "phd/trajectory_section.h"

#define DEBUG 1 //For displaying debug info
#define WORKSPACE 0.5 //Robot workspace width in m
#define HEIGHT_STEP .25 //Increment for each horizontal line
#define MAX_HEIGHT 1.3 //Maximum reachable height
#define OFFSET 0.1 //Optimal distance from wall to end effector
#define Z_HEIGHT 0.03 //Thickness of line slice
#define VIA_DISTANCE 0.2 //Distance between via points
#define NORM_ANGLE .1 //Maximum angle before adding additiona via points
#define PLANE_TOLERANCE .06 //Maximum distance from plane to keep for line points
#define RADIAL_STEPS 10
#define PI 3.1415925

//convenient typedefs **are these used?**
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_surface (new pcl::PointCloud<pcl::PointXYZI>); //The surface to be shotcreted
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_surface_full (new pcl::PointCloud<pcl::PointXYZI>); //The entire surface that was scanned (useful for normals athe cloud_surface edges)

//make the publishers global so we can publish from callbacks
ros::Publisher marker_pub; //Publishes normals
ros::Publisher dir_pub; //Path along the surface
ros::Publisher path_pub; //End effector path
ros::Publisher line_pub; //For debugging, publishes the cropped line that the via points are calculated on
ros::Publisher line_pub2; //For debugging, publishes the cropped line that the via points are calculated on
int nsid;
pcl::PointCloud<pcl::PointXYZI> lines;
//Functions for sorting point lines
bool forward_sort (phd::trajectory_point i,phd::trajectory_point j) { return (i.d<j.d); }
bool forward_pcl_sort (pcl::PointXYZI i,pcl::PointXYZI j) { return (i.intensity<j.intensity); }
bool reverse_sort (phd::trajectory_point i,phd::trajectory_point j) { return (i.d>j.d); }

//Functions for finding max and min values
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

	}//calculates the magnitude of a vector
float vec_length(pcl::PointXYZI pointA){

	return sqrt(pow(pointA.x,2)+pow(pointA.y,2)+pow(pointA.z,2));

	}

//Functions for copying points
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
	out.d = in.intensity;
	return out;
	}

//calculates a unit vector from the cross product
pcl::PointXYZI unit_cross(pcl::PointXYZI vectorA,pcl::PointXYZI vectorB){
	pcl::PointXYZI result;
	//Cross product	
	result.z = vectorA.x*vectorB.y-vectorB.x*vectorA.y;
	result.x = vectorA.y*vectorB.z-vectorB.y*vectorA.z;
	result.y = vectorA.z*vectorB.x-vectorB.z*vectorA.x;
	float length = sqrt(pow(result.x,2)+pow(result.y,2)+pow(result.z,2));
	//Convert to unit vector
	result.x = result.x/length;
	result.y = result.y/length;
	result.z = result.z/length;
	//Make it a positive value
	if(result.z < 0){
		result.x *= -1;
		result.y *= -1;
		result.z *= -1;
	}
	return result;

	}
//calculates a unit vector from two points
pcl::PointXYZI unit_vector(pcl::PointXYZI vectorA,pcl::PointXYZI vectorB){
	pcl::PointXYZI result;
	float d_total = vec_length(vectorA,vectorB);
	result.x = (vectorB.x-vectorA.x)/d_total;
	result.y = (vectorB.y-vectorA.y)/d_total;
	result.z = (vectorB.z-vectorA.z)/d_total;
	return result;

	}
//calculates a step vector from two points
pcl::PointXYZI vertical_step_vector(pcl::PointXYZI vectorA,pcl::PointXYZI vectorB, float step){
	pcl::PointXYZI result;
	//ROS_INFO("vStepping %f - %f - %f / %f - %f - %f", vectorA.x,vectorA.y,vectorA.z,vectorB.x,vectorB.y,vectorB.z);
	float d_total = vec_length(vectorA,vectorB);
	result.x = vectorB.x + ((vectorB.x-vectorA.x)/d_total)*step;
	result.y = vectorB.y + ((vectorB.y-vectorA.y)/d_total)*step;
	result.z = vectorB.z + ((vectorB.z-vectorA.z)/d_total)*step;
	return result;

}
//calculates a step vector from two points
pcl::PointXYZI horizontal_step_vector(pcl::PointXYZI vectorA,pcl::PointXYZI vectorB, float step){
	pcl::PointXYZI result;
	ROS_INFO("hStepping %f - %f - %f / %f - %f - %f", vectorA.x,vectorA.y,vectorA.z,vectorB.x,vectorB.y,vectorB.z);
	float d_total = vec_length(vectorA,vectorB);
	result.x = vectorB.x + ((vectorB.x-vectorA.x)/d_total)*step;
	result.y = vectorB.y + ((vectorB.y-vectorA.y)/d_total)*step;
	result.z = vectorB.z;
	return result;

	}
//Converts a point to a unit vector
pcl::PointXYZI unit_vector(pcl::PointXYZI vectorA){
	pcl::PointXYZI result;
	float d_total = sqrt(pow(vectorA.x,2)+pow(vectorA.y,2)+pow(vectorA.z,2));
	result.x = (vectorA.x)/d_total;
	result.y = (vectorA.y)/d_total;
	result.z = (vectorA.z)/d_total;
	return result;

	}

//dot product function for 2 phd::trajectory_points
float dot_product(phd::trajectory_point i, phd::trajectory_point j){
	return fabs(i.nx*j.nx+i.ny*j.ny+i.nz*j.nz)/(sqrt(pow(i.nx,2)+pow(i.ny,2)+pow(i.nz,2))*sqrt(pow(j.nx,2)+pow(j.ny,2)+pow(j.nz,2)));
	}

//finds nearest point to given coords
pcl::PointXYZI find_pt(pcl::PointXYZI target_pt){	
	int K = 1; //Number of points to find
	std::vector<int> pointIdxNKNSearch; //vector to hold points (used by pcl)
	std::vector<float> pointNKNSquaredDistance; //vector to hold distance of points (used by pcl)
	float resolution = 128.0f; //Octree resolution
	pcl::PointXYZI foundpt; //Container for found point (to return)
	//Create the octree object
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> octree (resolution);
	//Set the octree input cloud, then add the points
	ROS_INFO("surface size %lu",cloud_surface->points.size());
	octree.setInputCloud (cloud_surface);
	octree.addPointsFromInputCloud ();
	//Search for the nearest 1 neighbour
	if (octree.nearestKSearch (target_pt, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0){
		foundpt = cloud_surface->points[pointIdxNKNSearch[0]];
	}else ROS_INFO("Could not find point");
	return foundpt;

	}

float avg_dist_NN(pcl::PointXYZI point_holder,pcl::PointCloud<pcl::PointXYZI>::Ptr line){

	int K = 4; //Number of points to find
	std::vector<int> pointIdxNKNSearch; //vector to hold points (used by pcl)
	std::vector<float> pointNKNSquaredDistance; //vector to hold distance of points (used by pcl)
	pcl::PointXYZI searchPoint; //Location of where to search
	float resolution = 128.0f; //Octree resolution
	float distances;
	//Create the octree object
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> octree (resolution);
	//Set the octree input cloud, then add the points
	octree.setInputCloud (line);
	octree.addPointsFromInputCloud ();
	//Search for the nearest 1 neighbour
	ROS_INFO("line size %lu",line->points.size());
	if (octree.nearestKSearch (point_holder, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0){
		distances = pow((pointNKNSquaredDistance[0] + pointNKNSquaredDistance[1] + pointNKNSquaredDistance[2] + pointNKNSquaredDistance[3])/4,0.5);
	}else ROS_INFO("Could not find point");
	//ROS_INFO("Dist %f - %f - %f - %f", pointNKNSquaredDistance[0], pointNKNSquaredDistance[1], pointNKNSquaredDistance[2], pointNKNSquaredDistance[3]);
	return distances;

	}

bool find_NN(pcl::PointXYZI point_holder,pcl::PointCloud<pcl::PointXYZI>::Ptr line, pcl::PointXYZI* ret_pt){

	int K = 1; //Number of points to find
	std::vector<int> pointIdxNKNSearch; //vector to hold points (used by pcl)
	std::vector<float> pointNKNSquaredDistance; //vector to hold distance of points (used by pcl)
	pcl::PointXYZI searchPoint; //Location of where to search
	float resolution = 128.0f; //Octree resolution

	//ROS_INFO("Looking %f - %f - %f",point_holder.x,point_holder.y,point_holder.z);
	//ROS_INFO("line size %lu",line->points.size());
	if(line->points.size() > 0){
	//Create the octree object
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> octree (resolution);
	//Set the octree input cloud, then add the points
	octree.setInputCloud (line);
	octree.addPointsFromInputCloud ();
		//Search for the nearest 1 neighbour
		if (octree.nearestKSearch (point_holder, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0){
		}else{
			ROS_INFO("Could not find point");
			return false;
		}
	//ROS_INFO("Dist %f - %f - %f - %f", pointNKNSquaredDistance[0], pointNKNSquaredDistance[1], pointNKNSquaredDistance[2], pointNKNSquaredDistance[3]);
	*ret_pt = line->points[pointIdxNKNSearch[0]];
	return true;	
	}else return false;



	}
bool find_and_delete_NN(pcl::PointXYZI point_holder,pcl::PointCloud<pcl::PointXYZI>::Ptr line, pcl::PointXYZI* ret_pt){

	int K = 1; //Number of points to find
	std::vector<int> pointIdxNKNSearch; //vector to hold points (used by pcl)
	std::vector<float> pointNKNSquaredDistance; //vector to hold distance of points (used by pcl)
	pcl::PointXYZI searchPoint; //Location of where to search
	float resolution = 128.0f; //Octree resolution

	//ROS_INFO("Looking %f - %f - %f",point_holder.x,point_holder.y,point_holder.z);
	//ROS_INFO("line size %lu",line->points.size());
	if(line->points.size() > 0){
	//Create the octree object
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> octree (resolution);
	//Set the octree input cloud, then add the points
	octree.setInputCloud (line);
	octree.addPointsFromInputCloud ();
		//Search for the nearest 1 neighbour
		if (octree.nearestKSearch (point_holder, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0){
		}else{
			ROS_INFO("Could not find point");
			return false;
		}
	//ROS_INFO("Dist %f - %f - %f - %f", pointNKNSquaredDistance[0], pointNKNSquaredDistance[1], pointNKNSquaredDistance[2], pointNKNSquaredDistance[3]);
	*ret_pt = line->points[pointIdxNKNSearch[0]];
	return true;	
	}else return false;



	}

//finds nearest point to given coords using defined cloud
pcl::PointXYZI find_pt(pcl::PointCloud<pcl::PointXYZI>::Ptr l_cloud, pcl::PointXYZI target_pt){
	int K = 1; //Number of points to find
	std::vector<int> pointIdxNKNSearch; //vector to hold points (used by pcl)
	std::vector<float> pointNKNSquaredDistance; //vector to hold distance of points (used by pcl)
	pcl::PointXYZI searchPoint; //Location of where to search
	float resolution = 128.0f; //Octree resolution
	pcl::PointXYZI foundpt; //Container for found point (to return)

	//Create the octree object
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> octree (resolution);
	//Set the octree input cloud, then add the points
	octree.setInputCloud(l_cloud);
	octree.addPointsFromInputCloud ();
	//Search for the nearest 1 neighbour
	if (octree.nearestKSearch (target_pt, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0){
		foundpt = cloud_surface->points[pointIdxNKNSearch[0]];
	}else ROS_INFO("Could not find point");
	return foundpt;

	}



//find normal at given location
pcl::PointXYZI find_normal(float x, float y, float z){
	//The search method requires a pcl point	
	pcl::PointXYZI target_pt;
	target_pt.x = x;
	target_pt.y = y;
	target_pt.z = z;
	//vonatiner for return value
	pcl::PointXYZI foundpt = find_pt(target_pt);
	//pcl wants a cloud of points at which to calculate normals
	pcl::PointCloud<pcl::PointXYZI> cloudB;
	cloudB.push_back (foundpt);	
	// Create the normal estimation class
	pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
	// set the input for our normal estimation cloud
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

	// Use all neighbors in a sphere of radius 5cm
	ne.setRadiusSearch (0.1);
	ne.setViewPoint (0, 0, 0); //Which direction to point the notmal
	// Compute the features
	ne.compute (*cloud_normals);
	//since we want the normal to point inwards, but we can set the viewpoint at infinity, flip the result
	foundpt.x = -cloud_normals->points[0].normal[0];
	foundpt.y = -cloud_normals->points[0].normal[1];
	foundpt.z = -cloud_normals->points[0].normal[2];
	//ROS_INFO("Normal %f %f %f",foundpt.x,foundpt.y,foundpt.z);
	return foundpt;

	}


phd::trajectory_point pt_copy_normal(pcl::PointXYZI in){
	phd::trajectory_point out;
	pcl::PointXYZI normal = find_normal(in.x,in.y,in.z);
	out.x = in.x;
	out.y = in.y;
	out.z = in.z;
	out.d = in.intensity;
	out.nx = normal.x;
	out.ny = normal.y;
	out.nz = normal.z;
	return out;
	}
pcl::PointCloud<pcl::PointXYZI> crop_plane(pcl::PointXYZI point){
  

	Eigen::Vector4f minPoint; 
	minPoint[0]=-10;  // define minimum point x 
	minPoint[1]=-10;  // define minimum point y 
	minPoint[2]=-.01;  // define minimum point z 
	Eigen::Vector4f maxPoint; 
	maxPoint[0]=10;  // define max point x 
	maxPoint[1]=10;  // define max point y 
	maxPoint[2]=0.01;  // define max point z 

	Eigen::Vector3f boxTranslatation; 
	boxTranslatation[0]=0;   
	boxTranslatation[1]=0;   
	boxTranslatation[2]=0;   
	Eigen::Vector3f boxRotation; 
	boxRotation[0]=0;  // rotation around x-axis 
	boxRotation[1]=-asin(point.z/vec_length(point));  // rotation around y-axis 
	boxRotation[2]=0;  //in radians rotation around z-axis. this rotates your cube 45deg around z-axis. 
	ROS_INFO("Croped plane %lu -- %lu", cloud_surface->size(),cloud_surface_full->size());
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOut (new pcl::PointCloud<pcl::PointXYZI>); 

	pcl::CropBox<pcl::PointXYZI> cropFilter; 
	cropFilter.setInputCloud (cloud_surface); 
	cropFilter.setMin(minPoint); 
	cropFilter.setMax(maxPoint); 
	cropFilter.setTranslation(boxTranslatation); 
	cropFilter.setRotation(boxRotation); 

	cropFilter.filter (*cloudOut); 

	return *cloudOut;


	}

void show_markers(phd::trajectory_msg t_msg){

	uint32_t shape = visualization_msgs::Marker::LINE_LIST;
	visualization_msgs::Marker marker;
	visualization_msgs::Marker path;
	visualization_msgs::Marker surface_path;
	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
	marker.header.frame_id = "/world";
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
	marker.scale.z = 0.01;
	marker.color.r = 0.0f;
	marker.color.g = 0.0f;
	marker.color.b = 1.0f;
	marker.color.a = 0.8;
	marker.lifetime = ros::Duration();
	
	    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
	path.header.frame_id = "/world";
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
	surface_path.header.frame_id = "/world";
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

void show_marker(pcl::PointXYZI one,pcl::PointXYZI two,pcl::PointXYZI three, pcl::PointXYZI four){

	uint32_t shape = visualization_msgs::Marker::LINE_LIST;
	visualization_msgs::Marker marker;
	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
	marker.header.frame_id = "/world";
	marker.header.stamp = ros::Time::now();
	marker.ns = "basic_shapes";
	marker.id = nsid;
	marker.type = visualization_msgs::Marker::LINE_LIST;
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
	marker.scale.z = 0.015;
	marker.color.r = 0.0f;
	marker.color.g = 0.0f;
	marker.color.b = 1.0f;
	marker.color.a = 0.8;
	marker.lifetime = ros::Duration();
	

		marker.id = nsid;
		marker.points.resize(2);

			//ROS_INFO("Point %d: %f- %f- %f",i,path.points[i].x,path.points[i].y,path.points[i].z);
			marker.points[0].x = one.x;
			marker.points[0].y = one.y;
			marker.points[0].z = one.z;
			marker.points[1].x = two.x;
			marker.points[1].y = two.y;
			marker.points[1].z = two.z;//ROS_INFO("Point %d: %f- %f- %f",i,path.points[i].x,path.points[i].y,path.points[i].z);
		marker_pub.publish(marker);
		ros::Duration(0.5).sleep();

		marker.id = nsid+1;
		marker.points.resize(2);
	marker.color.r = 0.0f;
	marker.color.g = 1.0f;
	marker.color.b = 0.0f;
			marker.points[0].x = one.x;
			marker.points[0].y = one.y;
			marker.points[0].z = one.z;
			marker.points[1].x = three.x;
			marker.points[1].y = three.y;
			marker.points[1].z = three.z;

		marker_pub.publish(marker);
		ros::Duration(0.5).sleep();

		marker.id = nsid+2;
		marker.points.resize(2);
	marker.color.r = 1.0f;
	marker.color.g = 0.0f;
	marker.color.b = 0.0f;
	marker.points[0].x = one.x;
			marker.points[0].y = one.y;
			marker.points[0].z = one.z;
			marker.points[1].x = four.x;
			marker.points[1].y = four.y;
			marker.points[1].z = four.z;

	
		marker_pub.publish(marker);
		ros::Duration(0.5).sleep();
	nsid += 3;

	}

////////////////////////////
pcl::PointCloud<pcl::PointXYZI> sort_vertical_line(pcl::PointCloud<pcl::PointXYZI>::Ptr line){

	pcl::PointCloud<pcl::PointXYZI> sorted;
	pcl::PointXYZI point_holder;
	float zmin = 100;
	pcl::PointXYZI start_pt, found_pt, prev_pt, next_pt;
	pcl::PointCloud<pcl::PointXYZI>::iterator start_it, next_it;
	float c_val;

	//pcl::PointCloud<pcl::PointXYZI>::Ptr line_ds (new    pcl::PointCloud<pcl::PointXYZI> );
	pcl::VoxelGrid<pcl::PointXYZI> sor;
	//Downsample the alignment section
	sor.setInputCloud (line);
	sor.setLeafSize (0.03f, 0.03f, 0.03f);
	sor.filter (*line);


	/*for(pcl::PointCloud<pcl::PointXYZI>::iterator ctr = line.begin(); ctr < line.end(); ++ctr){
		cval = sqrt(pow(ctr->x,2)+pow(ctr->z,2));
		if(cval < zmin){
			start_it = ctr;
			zmin = cval;
		}

	}*/
	pcl::PointXYZI dir, search_pt, d_pt, ctr_pt, ctr_pt2;
	float calc_d, d_total;
	float dir_val, old_d;	
	float step_sz;	
	bool broken = false;
	prev_pt.x = 0;
	prev_pt.y = 0;
	prev_pt.z = -10;
	prev_pt.intensity = 0;
	step_sz = 0.01;	
	pcl::PassThrough<pcl::PointXYZI> pass;
	pcl::PointCloud<pcl::PointXYZI> bottom_line;
	pass.setInputCloud (line);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (-0.1, .5);
	pass.filter (bottom_line);
	find_NN(prev_pt,bottom_line.makeShared(), &next_pt);
	next_pt.intensity = 0;
	sorted.push_back(next_pt);
	ROS_INFO("first pt %f - %f - %f",next_pt.x,next_pt.y,next_pt.z);
	step_sz = 0.01;	
	while(next_pt.x == sorted.points[0].x && next_pt.y == sorted.points[0].y && next_pt.z == sorted.points[0].z && step_sz < 1){
		find_NN(vertical_step_vector(prev_pt,sorted.points[0],step_sz),line, &next_pt);
		step_sz += 0.01;
	}
	next_pt.intensity = 0;//sqrt(pow(next_pt.x-prev_pt.x,2)+pow(next_pt.z-prev_pt.z,2));
	//sorted.push_back(next_pt);
	ROS_INFO("Starting loop %lu", line->points.size());
	pcl::PointXYZI stepp;
	//ROS_INFO("size %lu",sorted.points.size());
	do{	
		step_sz = 0.01;	
		sorted.push_back(next_pt);
		prev_pt = next_pt;	
		do{
			stepp = vertical_step_vector(sorted.points[sorted.points.size()-2],sorted.points[sorted.points.size()-1],step_sz);
			//ROS_INFO("Step %f - %f - %f / %f",stepp.x, stepp.y, stepp.z, step_sz);
			//if(sorted.points[sorted.points.size()-2].x != sorted.points[sorted.points.size()-1].x && sorted.points[sorted.points.size()-2].y != sorted.points[sorted.points.size()-1].y && sorted.points[sorted.points.size()-2].z != sorted.points[sorted.points.size()-1].z)
			if(find_NN(stepp,line, &next_pt)){
			//for(pcl::PointCloud<pcl::PointXYZI>::iterator ctr = line->begin(); ctr < line->end(); ++ctr){
			//	if(ctr->x == next_pt.x && ctr->y == next_pt.y && ctr->z == next_pt.z){
				//pcl::PointCloud<pcl::PointXYZI>::iterator itr = line->
				//line->erase(itr);
				
//					break;
//				}
//			}
			}else broken = true;
			step_sz += 0.01;
		ROS_INFO("inside");
		}while(next_pt.x == prev_pt.x && next_pt.y == prev_pt.y && next_pt.z == prev_pt.z && step_sz < 0.2 && broken == false);
	
		next_pt.intensity = prev_pt.intensity + sqrt(pow(next_pt.x-prev_pt.x,2)+pow(next_pt.y-prev_pt.y,2)+pow(next_pt.z-prev_pt.z,2));
		ROS_INFO("outside");
	}while(step_sz < 0.2 && prev_pt.intensity < 10 && broken == false);
	ROS_INFO("int %f", prev_pt.intensity);
	//std::sort (sorted.points.begin(), sorted.points.end(), forward_pcl_sort);
	return sorted;

}

pcl::PointXYZI find_plane(pcl::PointXYZI pt_location){

	pcl::PointXYZI norm = unit_vector(find_normal(pt_location.x, pt_location.y, pt_location.z));
	pcl::PointXYZI up, alt;
	up.z = 1;
	alt.x = 0;
	alt.y = 1;
	alt.z = 0;
	pcl::PointXYZI side;
	//if(norm.z > .3) 
	//side = unit_cross(norm,alt);
	//else side = unit_cross(norm,up);
	pcl::PointXYZI plane = unit_cross(norm, alt);
	//plane.x = plane.x - pt_location.x;
	//plane.y = plane.y - pt_location.y;
	//plane.z = plane.z - pt_location.z;
	pcl::PointXYZI marker1, marker2, marker3;
	marker1.x = pt_location.x + norm.x;
	marker1.y = pt_location.y + norm.y;
	marker1.z = pt_location.z + norm.z;
	marker2.x = pt_location.x + alt.x;
	marker2.y = pt_location.y + alt.y;
	marker2.z = pt_location.z + alt.z;
	marker3.x = pt_location.x + plane.x;
	marker3.y = pt_location.y + plane.y;
	marker3.z = pt_location.z + plane.z;
	show_marker(pt_location,marker3,marker2,marker1);
	return plane;

	}

pcl::PointCloud<pcl::PointXYZI> sort_horizontal(pcl::PointCloud<pcl::PointXYZI>::Ptr input){
	pcl::PointCloud<pcl::PointXYZI> output;
	pcl::PointXYZI start_pt, search_pt, prev_pt;
	start_pt.x = 0;
	start_pt.y = 10;
	start_pt.z = 0;
	find_NN(start_pt, input, &search_pt);
	ROS_INFO("Starting size %lu",input->size());
	for(pcl::PointCloud<pcl::PointXYZI>::iterator ctr = input->begin(); ctr < input->end(); ++ctr){
		if(ctr->x == search_pt.x && ctr->y == search_pt.y && ctr->z == search_pt.z){
			input->erase(ctr);
			break;
		}
	}
	search_pt.intensity = 0;
	while(input->size() > 0){
		prev_pt = search_pt;
		find_NN(start_pt, input, &search_pt);
		for(pcl::PointCloud<pcl::PointXYZI>::iterator ctr = input->begin(); ctr < input->end(); ++ctr){
			if(ctr->x == search_pt.x && ctr->y == search_pt.y && ctr->z == search_pt.z){
				input->erase(ctr);
				break;
			}
		}
		search_pt.intensity = prev_pt.intensity + sqrt(pow(search_pt.x-prev_pt.x,2)+pow(search_pt.y-prev_pt.y,2));
		output.push_back(search_pt);

	ROS_INFO("Remaining size %lu",input->size());
	}	
	return output;
	}

void calc_points(phd::trajectory_msg* trajectory, pcl::PointXYZI point, int dir){
	phd::trajectory_point tPoint;
	pcl::PointCloud<pcl::PointXYZI> horizontal_line = crop_plane(point);
	horizontal_line = sort_horizontal(horizontal_line.makeShared());
	//lines = lines+horizontal_line;
	pcl::PointXYZI prev_pt, next_pt;
	pcl::PointCloud<pcl::PointXYZI> sorted;
	int ctr = 0;
	sorted.push_back(horizontal_line.points[0]);
	prev_pt = horizontal_line.points[0];
	for(int pctr = 1; pctr < horizontal_line.size(); ++pctr){	
		if(vec_length(prev_pt,horizontal_line.points[pctr])>VIA_DISTANCE){
			sorted.push_back(horizontal_line.points[pctr]);
			prev_pt = horizontal_line.points[pctr];
		}
	}

		ROS_INFO("Tpoints %lu",sorted.size());
	if(dir > 0){
		for(ctr = 0; ctr < sorted.size(); ++ctr){
			tPoint = pt_copy_normal(sorted.points[ctr]);
			trajectory->points.push_back(tPoint);
		}
	}else{
		for(ctr = sorted.size()-1; ctr >= 0; --ctr){
			tPoint = pt_copy_normal(sorted.points[ctr]);
			trajectory->points.push_back(tPoint);
		}
	}
	}

bool generate (phd::simple_trajectory_service::Request  &req,
         phd::simple_trajectory_service::Response &res)
{
	ROS_INFO("Received Trajectory Request");
	pcl::PointCloud<pcl::PointXYZI> cloudB;
	sensor_msgs::PointCloud2 cloud_msg1 = req.cloud_surface;
	sensor_msgs::PointCloud2 cloud_msg2 = req.cloud_in;
	if(DEBUG) ROS_INFO("Cloud Surface: %d -- Cloud Selection %d",cloud_msg1.width,cloud_msg2.width) ;
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
	pcl::PointCloud<pcl::PointXYZI>::Ptr line (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PassThrough<pcl::PointXYZI> pass;
	/*pass.setInputCloud (cloud_surface);
	pass.setFilterFieldName ("y");
	pass.setFilterLimits (-Z_HEIGHT, Z_HEIGHT);
	pass.filter (*line);
	pass.setInputCloud (line);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (0, 1.5);
	pass.filter (*line);
*/
	

	Eigen::Vector4f minPoint; 
	minPoint[0]=-10;  // define minimum point x 
	minPoint[1]=-.010;  // define minimum point y 
	minPoint[2]=-10;  // define minimum point z 
	Eigen::Vector4f maxPoint; 
	maxPoint[0]=10;  // define max point x 
	maxPoint[1]=.010;  // define max point y 
	maxPoint[2]=10;  // define max point z 

	Eigen::Vector3f boxTranslatation; 
	boxTranslatation[0]=0;   
	boxTranslatation[1]=0;   
	boxTranslatation[2]=0;   
	Eigen::Vector3f boxRotation; 
	boxRotation[0]=0;  // rotation around x-axis 
	boxRotation[1]=0;  // rotation around y-axis 
	pcl::CropBox<pcl::PointXYZI> cropFilter; 
	cropFilter.setInputCloud (cloud_surface); 
	cropFilter.setMin(minPoint); 
	cropFilter.setMax(maxPoint); 
	cropFilter.setTranslation(boxTranslatation); 
	pcl::PointCloud<pcl::PointXYZI> vertical_line;

	lines.clear();
	for(int ctr = 0; ctr <= RADIAL_STEPS; ++ctr){
	
		boxRotation[2]=-PI/2 + ctr*(PI/RADIAL_STEPS);  //in radians rotation around z-axis 
		cropFilter.setRotation(boxRotation); 
		cropFilter.filter (*line); 
		lines = lines + *line;
		ROS_INFO("Cropped, sorting %lu",line->points.size());
		if(line->points.size() > 0) vertical_line = vertical_line + sort_vertical_line(line);

	}





	ROS_INFO("Sorted");

	sensor_msgs::PointCloud2 line_cloud;
	pcl::toROSMsg(vertical_line,line_cloud);
	ROS_INFO("Publishing %d ptd", line_cloud.width);
	line_cloud.header.frame_id = "/world";
	line_cloud.header.stamp = ros::Time::now();
	line_pub.publish(line_cloud);
	ros::spinOnce();
	int ctr = 1;
	float height_ctr = 0;
	ROS_INFO("line size %lu",vertical_line.size());
	pcl::PointXYZI prev_pt = vertical_line.points[0];
	//horizontal_line = calc_line_points(vertical_line.points[0]);
	int dir = 1;
	/*calc_points(&t_msg,vertical_line.points[0],dir);
	while(ctr < vertical_line.size()){
		if(vec_length(vertical_line.points[ctr],prev_pt) > HEIGHT_STEP){
			ROS_INFO("%f --- %f", vec_length(vertical_line.points[ctr],prev_pt), vertical_line.points[ctr].intensity);
			prev_pt = vertical_line.points[ctr];
			dir = dir * -1;
			calc_points(&t_msg,vertical_line.points[ctr],dir);
		}
		++ctr;
	}
	dir = dir * -1;
	calc_points(&t_msg,vertical_line.points[vertical_line.size()-1],dir);
*/
	/*sensor_msgs::PointCloud2 line_cloud2;
	pcl::toROSMsg(horizontal_line,line_cloud2);
	ROS_INFO("Publishing %d pts", line_cloud2.width);
	line_cloud2.header.frame_id = "/world";
	line_cloud2.header.stamp = ros::Time::now();
	line_pub.publish(line_cloud2);
	*/sensor_msgs::PointCloud2 line_cloud3;
	pcl::toROSMsg(lines,line_cloud3);
	line_cloud3.header.frame_id = "/world";
	line_cloud3.header.stamp = ros::Time::now();
	line_pub2.publish(line_cloud3);
	show_markers(t_msg);
	ros::spinOnce();
	res.trajectory = t_msg;
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
	line_pub2 = nh.advertise<sensor_msgs::PointCloud2>( "line_points2", 0 );
	path_pub = nh.advertise<visualization_msgs::Marker>( "path_marker", 0 );
	dir_pub = nh.advertise<visualization_msgs::Marker>( "dir_marker", 0 );
	ROS_INFO("Trajectory Generator online");
	nsid = 0;
	ros::spin();  
}
