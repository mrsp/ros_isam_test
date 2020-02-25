#include <ros/ros.h>
#include "Isam.h"
#include "kparams.h"
#include <random>
#include "cov2DTo3D.h"
int main(int argc, char** argv)
{


ros::init(argc, argv, "isam_ros_test");


ros::NodeHandle n;

kparams_t params;

Isam myIsam(params);
Affine3d tempPose = Affine3d::Identity();
Matrix<double,6,6> tempCov = Matrix<double,6,6>::Identity()*1e-6;
myIsam.init(tempPose,tempCov);
int originPoseIdx = 0;



//True Displacement:
Vector3d trueDist = Vector3d(0.99,0.99,0.99);


//First Factor with Noise
tempPose(0,3)=0.49; // 0.99
tempPose(1,3)=1.49; //0.99 
tempPose(2,3)=0.3; //0.99
tempCov =  Matrix<double,6,6>::Identity() *10;
//Add Factor to the Graph
myIsam.addFrame(tempPose, tempCov);
int firstPoseIdx = 1;


//LANDMARKS
Vector3d landMarkPosFromFirstPose;
Vector3d landMarkPosFromOrigin;
Matrix3d landMarkCov = Matrix3d::Identity()*1e-3;
int landIdx; 


//Generate Landmarks
int i=0,NL = 5;
while(i<NL)
{
    landMarkPosFromFirstPose = Vector3d(i+1.00,i+1.00,i+1.00);
    landMarkPosFromOrigin = Vector3d(i+1.00+trueDist(0),i+1.00+trueDist(1),i+1+trueDist(2));
    landIdx = myIsam.addLandmark(Vector3d::Zero());
    myIsam.connectLandmark(landMarkPosFromFirstPose,landIdx,firstPoseIdx,landMarkCov);
    myIsam.connectLandmark(landMarkPosFromOrigin,landIdx,originPoseIdx,landMarkCov);
    i++;
}

//Optimize the Graph
double error = myIsam.optimize(0);


std::cout<<"Optimization Error"<<std::endl;
std::cout<<error<<std::endl;
std::cout<<"-----------------------------------"<<std::endl;
std::cout<<"Origin Position"<<std::endl;
std::cout<<myIsam.getPose(0).translation()<<std::endl;
std::cout<<"Origin Orientation"<<std::endl;
std::cout<<myIsam.getPose(0).linear()<<std::endl;
std::cout<<"True Origin Position"<<std::endl;
std::cout<<Vector3d::Zero()<<std::endl;
std::cout<<"True Origin Orientation"<<std::endl;
std::cout<<Matrix3d::Identity()<<std::endl;
std::cout<<"-----------------------------------"<<std::endl;
std::cout<<"First Pose Position"<<std::endl;
std::cout<<myIsam.getPose(1).translation()<<std::endl;
std::cout<<"First Pose Rotation"<<std::endl;
std::cout<<myIsam.getPose(1).linear()<<std::endl;
std::cout<<"True First Pose Position"<<std::endl;
std::cout<<trueDist<<std::endl;
std::cout<<"True First Pose Rotation"<<std::endl;
std::cout<<Matrix3d::Identity()<<std::endl;

return 0;
}
