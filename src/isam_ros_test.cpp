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
std::normal_distribution<double> distribution(0.0,1.0);

Isam myIsam;
sMatrix4 tempPose;
sMatrix6 tempCov;
myIsam.init(tempPose);



float3 *keyVert, *prevKeyVert;
int size = 5;
keyVert = (float3*) malloc(sizeof(float3)*size);
prevKeyVert = (float3*) malloc(sizeof(float3)*size);
int i=0;

std::vector<int> source_corr, target_corr;
source_corr.resize(size);
target_corr.resize(size);


    double number = distribution(generator);
    std::cout<<"number "<<number<<std::endl;
    keyVert[0].x = 1.21 + number;
    keyVert[0].y = 1.33 + number;
    keyVert[0].z = 1.11 + number;
    prevKeyVert[0].x = 0.01;
    prevKeyVert[0].y = 0.01;
    prevKeyVert[0].z = 0.01;
    source_corr[0] = 0;
    target_corr[0] = 0;


    number = distribution(generator);
    std::cout<<"number "<<number<<std::endl;
    keyVert[1].x = 1.0 + number;
    keyVert[1].y = 1.0 + number;
    keyVert[1].z = 1.0 + number;
    prevKeyVert[1].x = 0.01 ;
    prevKeyVert[1].y = 0.01 ;
    prevKeyVert[1].z = 0.01;
    source_corr[1] = 1;
    target_corr[1] = 1;


    number = distribution(generator);
    std::cout<<"number "<<number<<std::endl;
    keyVert[2].x = 1.0 + number;
    keyVert[2].y = 1.0 + number;
    keyVert[2].z = 1.0 + number;
    prevKeyVert[2].x = 0.1 ;
    prevKeyVert[2].y = 0.1;
    prevKeyVert[2].z = 0.1 ;
    source_corr[2] = 2;
    target_corr[2] = 2;


    number = distribution(generator);
    std::cout<<"number "<<number<<std::endl;
    keyVert[3].x = 1.0 + number;
    keyVert[3].y = 1.0 + number;
    keyVert[3].z = 1.0 + number;
    prevKeyVert[3].x = 0.01 ;
    prevKeyVert[3].y = 0.01 ;
    prevKeyVert[3].z = 0.01 ;
    source_corr[3] = 3;
    target_corr[3] = 3;



    number = distribution(generator);
    std::cout<<"number "<<number<<std::endl;
    keyVert[4].x = 1.0 + number;
    keyVert[4].y = 1.0 + number;
    keyVert[4].z = 1.0 + number;
    prevKeyVert[4].x = 0.01 ;
    prevKeyVert[4].y = 0.01 ;
    prevKeyVert[4].z = 0.01 ;
    source_corr[4] = 4;
    target_corr[4] = 4;


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
