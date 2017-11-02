#include<iostream>
#include<string>
#include<fstream>
#include<cstring>
// ROS
#include <ros/ros.h>
/* atof example: sine calculator */
#include <stdio.h>      /* printf, fgets */
#include <stdlib.h>     /* atof */
#include <math.h>       /* sin */
#include <Eigen/Core>
#include <Eigen/Dense>
using namespace std;
int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

//ifstream myFile("/home/mike/testing/poses-saved.csv");
ifstream myFile("/home/mike/testing/divided.csv");
double dat, dat2, dat3;
double odat = -1.38;
int ctr = 0;
int dctr = 0;
ofstream myfile;
	std::stringstream fs;
	fs << "/home/mike/testing/divided-calc.csv";
	myfile.open (fs.str().c_str(), std::ios::out|std::ios::app);
while(myFile.good())
{
++ctr;
    string line;

    getline(myFile, line);
	//ROS_INFO("%s",line.c_str());
    char *sArr = new char[line.length() + 1];
    strcpy(sArr, line.c_str());
ROS_INFO("%s", line.c_str());
    char *sPtr;
Eigen::Matrix4f transform_mat;
    sPtr = strtok(sArr, ",");
        sPtr = strtok(NULL, ",");
        sPtr = strtok(NULL, ",");
        sPtr = strtok(NULL, ",");
        sPtr = strtok(NULL, ",");
	dat = atof(sPtr);
        sPtr = strtok(NULL, ",");
	dat2 = atof(sPtr);
        sPtr = strtok(NULL, ",");
	dat3 = atof(sPtr);
        sPtr = strtok(NULL, ",");
    /*while(sPtr != NULL)
    {
//        cout << sPtr << " - ";
	dat = atof(sPtr);
	ROS_INFO("%f",dat);	
    }*/


if(fabs(dat-odat) < 0.002) ++dctr;
else{
 dctr = 0;

	odat = dat;
}
if(dctr > 10){
	dctr = 0;
	myfile << dat << "," << dat2 << "," << dat3 << std::endl;
}
	
/*

Eigen::Matrix4f transformAB = transform_mat.inverse();
ofstream myfile;
	std::stringstream fs;
	fs << "/home/mike/testing/poses-inv.csv";
	myfile.open (fs.str().c_str(), std::ios::out|std::ios::app);
	myfile << transformAB(0,0) << ",";
	myfile << transformAB(0,1) << ",";
	myfile << transformAB(0,2) << ",";
	myfile << transformAB(0,3) << ",";
	myfile << transformAB(1,0) << ",";
	myfile << transformAB(1,1) << ",";
	myfile << transformAB(1,2) << ",";
	myfile << transformAB(1,3) << ",";
	myfile << transformAB(2,0) << ",";
	myfile << transformAB(2,1) << ",";
	myfile << transformAB(2,2) << ",";
	myfile << transformAB(2,3) << ",";
	myfile << transformAB(3,0) << ",";
	myfile << transformAB(3,1) << ",";
	myfile << transformAB(3,2) << ",";
	myfile << transformAB(3,3) << ",";
	myfile << std::endl;
*/
}
myFile.close();


return 0;

}