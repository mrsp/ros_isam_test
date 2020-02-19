#include <ros/ros.h>
#include "Isam.h"
#include "kernelscalls.h"
#include "kparams.h"
#include <random>
#include "sMatrix.h"

int main(int argc, char** argv)
{


ros::init(argc, argv, "isam_ros_test");


ros::NodeHandle n;

kparams_t params;
std::default_random_engine generator;
std::normal_distribution<double> distribution(0.0,0.1);

Isam myIsam;
sMatrix4 tempPose;
sMatrix6 tempCov;
myIsam.init(tempPose);


// int j=0,jj=0;
// while(j<4)
// {
// while(jj<4)
// {
//     tempPose(j,jj)=j+jj;
//     jj++;
// }
// j++;
// }
// tempPose(0,0)=1;
// std::cout<<"TEMP COV "<<std::endl;
// std::cout<<tempPose<<std::endl;
// std::cout<<inverse(tempPose)<<std::endl;


float3 *keyVert, *prevKeyVert;
int size = 1;
keyVert = (float3*) malloc(sizeof(float3)*size);
prevKeyVert = (float3*) malloc(sizeof(float3)*size);
int i=0;

std::vector<int> source_corr, target_corr;
source_corr.resize(size);
target_corr.resize(size);
while(i<size)
{
    double number = distribution(generator);
    std::cout<<"number "<<number<<std::endl;
    //number=0;
    keyVert[i].x = i+1 + number;
    keyVert[i].y = i+1 + number;
    keyVert[i].z = i+1 + number;
    prevKeyVert[i].x = i+0.01;
    prevKeyVert[i].y = i+0.01;
    prevKeyVert[i].z = i+0.01;
    source_corr[i] = i;
    target_corr[i] = i;
    i++;
}
tempPose(0,3)=0.99;
tempPose(1,3)=0.99;
tempPose(2,3)=0.99;



tempCov = calculatePoint2PointCov(keyVert,
                                         size,
                                         prevKeyVert,
                                         size,
                                         source_corr,
                                         target_corr,
                                         tempPose,
                                         params);

std::cout<<"Covariance "<<std::endl;
std::cout<<tempCov<<std::endl;
myIsam.addFrame(tempPose, tempCov);
double error = myIsam.optimize(0);




ROS_INFO("Error");
std::cout<<error<<std::endl;
ROS_INFO("TFs");
std::cout<<myIsam.getPose(0)<<std::endl;
std::cout<<myIsam.getPose(1)<<std::endl;



return 0;
}
