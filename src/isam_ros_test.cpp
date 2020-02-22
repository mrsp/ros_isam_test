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

std::cout<<"Temp Cov"<<tempCov<<std::endl;

tempPose(0,3)=0.99;
tempPose(1,3)=0.99;
tempPose(2,3)=0.99;


myIsam.addFrame(tempPose, tempCov);
double error = myIsam.optimize(0);




ROS_INFO("Error");
std::cout<<error<<std::endl;
ROS_INFO("TFs");
std::cout<<myIsam.getPose(0).translation()<<std::endl;
std::cout<<myIsam.getPose(1).translation()<<std::endl;



return 0;
}
