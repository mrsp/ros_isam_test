#include <ros/ros.h>
#include "Isam.h"


int main(int argc, char** argv)
{


ros::init(argc, argv, "isam_ros_test");


ros::NodeHandle n;

Isam myIsam;
sMatrix4 tempPose;
sMatrix6 tempCov;
myIsam.init(tempPose);


tempPose(2,3)=9;
myIsam.addFrame(tempPose, tempCov);
tempPose(2,3)=12;
myIsam.addFrame(tempPose, tempCov);
tempPose(2,3)=15;
myIsam.addFrame(tempPose, tempCov);
double error = myIsam.optimize(0);


ROS_INFO("Error");
std::cout<<error<<std::endl;
ROS_INFO("TFs");
std::cout<<myIsam.getPose(0)<<std::endl;
std::cout<<myIsam.getPose(1)<<std::endl;
std::cout<<myIsam.getPose(2)<<std::endl;
std::cout<<myIsam.getPose(3)<<std::endl;


return 0;
}
