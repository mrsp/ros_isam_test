#include <ros/ros.h>
#include "Isam.h"
#include "kparams.h"
#include <random>

int main(int argc, char** argv)
{


ros::init(argc, argv, "isam_ros_test");


ros::NodeHandle n;

kparams_t params;

Isam myIsam(params);
Affine3d tempPose = Affine3d::Identity();
Matrix<double,6,6> tempCov = Matrix<double,6,6>::Identity();
myIsam.init(tempPose,tempCov);
int originPoseIdx = 0;
std::cout<<"Temp Cov"<<tempCov<<std::endl;

tempPose(0,3)=0.79; //True is 0.99
tempPose(1,3)=1.09; //0.99 
tempPose(2,3)=0.9; //0.99

tempCov =  Matrix<double,6,6>::Identity() * 1e-1;
myIsam.addFrame(tempPose, tempCov);
int firstPoseIdx = 1;

Vector3d landMarkPosFromFirstPose = Vector3d(0,1.00,0);
Vector3d landMarkPosFromOrigin = Vector3d(-0.99,1.99,-0.99);
Matrix3d landMarkCov = Matrix3d::Identity()*1e-6;
int landIdx = myIsam.addLandmark(Vector3d::Zero());
myIsam.connectLandmark(landMarkPosFromFirstPose,landIdx,firstPoseIdx,landMarkCov);
myIsam.connectLandmark(landMarkPosFromOrigin,landIdx,originPoseIdx,landMarkCov);

double error = myIsam.optimize(0);




ROS_INFO("Error");
std::cout<<error<<std::endl;
ROS_INFO("TFs");
std::cout<<myIsam.getPose(0).translation()<<std::endl;
std::cout<<myIsam.getPose(1).translation()<<std::endl;



return 0;
}
