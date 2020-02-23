#include <ros/ros.h>
#include "Isam.h"
#include "kparams.h"
#include <random>
#include "dataio.h"
#include "covPoint2Point.h"

#include"cov2DTo3D.h"

Eigen::MatrixXd computeCov2DTo3DfromVert(Eigen::MatrixXd cov2D, Vector3d vertex, Matrix3d cam, double depth_noise_cov)
{
//     Vector3d v(vertex.x,vertex.y,vertex.z);
    Vector3d tmp=cam*vertex;
    double depth=tmp[2];
    double fx=cam(0,0);
    double fy=cam(1,1);
    double cx=cam(0,2);
    double cy=cam(1,2);
    
    
    Eigen::MatrixXd cov=computeCov2DTo3D(cov2D,depth,fx,fy,cx,cy,depth_noise_cov);
    return cov;

}

int main(int argc, char **argv)
{
    int initialFrame = 10; //from where to initialize ISAM
    int firstKeyFrame = 25; 
    int finalKeyFrame = 50;

    double fx=481.2;
    double fy=480;
    double cx=319.5;
    double cy=239.5;

    Matrix3d camMatrix=Matrix3d::Identity();
    camMatrix(0,0)=fx;
    camMatrix(1,1)=fy;
    camMatrix(0,2)=cx;
    camMatrix(1,2)=cy;
    
    
    
    //Gaussian Noise
    double factorNoiseStd = 0.01;
    double LandMarkNoiseStd = 0.01;
    std::default_random_engine generator;
    std::normal_distribution<double> distribution(0.0,factorNoiseStd);

    Matrix<double, 6, 6>  frameCov = Matrix<double, 6, 6>::Identity() * factorNoiseStd*factorNoiseStd; //Setting Frame Covariance
    Matrix<double, 6, 6> initialISAMCov = Matrix<double, 6, 6>::Identity() * 1e-6;

    Matrix3d landMarkCov = Matrix3d::Identity() * LandMarkNoiseStd*LandMarkNoiseStd; //Setting Landmark Covariance

    
    


    //ROS
    ros::init(argc, argv, "isam_ros_data");
    ros::NodeHandle n;

    kparams_t params;
    Isam myIsam(params);
    Affine3d  origin=readPoseEigen(initialFrame);
    myIsam.init(origin, initialISAMCov);

    Affine3d prevPose=origin;
    
    int i = initialFrame + 1;
    while (i < finalKeyFrame + 1)
    {
        //Add Factor to the Graph
        Affine3d tempPose = readPoseEigen(i);
        Affine3d delta= prevPose.inverse()*tempPose;

        delta.translation() += Vector3d(distribution(generator),distribution(generator),distribution(generator));

        Affine3d newPose=prevPose*delta;
        prevPose=newPose;
        myIsam.addFrame(newPose, frameCov);
        i++;
    }
    cout << "Added Factors " << i << endl;

    // //LANDMARKS
    int landIdx;
    Vector3d landMarkPosFromPose0, landMarkPosFromPose1;
    vector<Vector3d> points0 = readPointsEigen(firstKeyFrame);
    vector<Vector3d> points1 = readPointsEigen(finalKeyFrame);
    vector<corr_t> corr = readCorr(firstKeyFrame, finalKeyFrame);
    
    vector<descr_t> descr0 = readDescr(firstKeyFrame);
    vector<descr_t> descr1 = readDescr(finalKeyFrame);


    Affine3d  transform = readPoseEigen(finalKeyFrame).inverse() * readPoseEigen(firstKeyFrame);
    double sensor_noise_cov = 0.02;


    vector<Vector3d> corrVec0;
    vector<Vector3d> corrVec1;
    i = 0;
    while (i < corr.size())
    {
        landMarkPosFromPose0 = points0[corr[i].from];
        landMarkPosFromPose1 = points1[corr[i].to];
        
        corrVec0.push_back(points0[corr[i].from]);
        corrVec1.push_back(points1[corr[i].to]);
        i++;
    }
    
    Eigen::MatrixXd ICP_COV =  Eigen::MatrixXd::Zero(6,6);
    ICP_COV = computeICPCovPoint2Point(corrVec0, corrVec1,  sensor_noise_cov,  transform);
    landMarkCov = ICP_COV.block<3,3>(0,0);
    std::cout<<"ICP Covariance"<<std::endl;
    std::cout<<ICP_COV<<std::endl;


    i = 0;
    while (i < corr.size())
    {

        landIdx = myIsam.addLandmark(Vector3d::Zero());
        landMarkPosFromPose0 = corrVec0[i];
        landMarkPosFromPose1 = corrVec1[i];
        
        descr_t d0=descr0[corr[i].from];
        descr_t d1=descr0[corr[i].to];
        
        double radio0=d0.size/2;
        double radio1=d1.size/2;
        
        Matrix2d cov2d0=Matrix2d::Identity()*radio0*radio0;
        Matrix2d cov2d1=Matrix2d::Identity()*radio1*radio1;
        
        Matrix3d landMarkCov0=computeCov2DTo3DfromVert(cov2d0,landMarkPosFromPose0,camMatrix,sensor_noise_cov);
        Matrix3d landMarkCov1=computeCov2DTo3DfromVert(cov2d1,landMarkPosFromPose1,camMatrix,sensor_noise_cov);

        std::cout<<landMarkCov0<<std::endl;
        std::cout<<landMarkCov1<<std::endl;
        
        myIsam.connectLandmark(landMarkPosFromPose0, landIdx, firstKeyFrame - initialFrame, landMarkCov0);
        myIsam.connectLandmark(landMarkPosFromPose1, landIdx, finalKeyFrame - initialFrame, landMarkCov1);

        i++;
    }

    //Optimize the Graph
    double error = myIsam.optimize(0);

    std::cout << "Optimization Error" << std::endl;
    std::cout << error << std::endl;
    std::cout << "-----------------------------------" << std::endl;
    std::cout << "First KeyFrame Position" << std::endl;
    std::cout << myIsam.getPose(firstKeyFrame - initialFrame).translation() << std::endl;
    std::cout << "First KeyFrame Rotation" << std::endl;
    std::cout << myIsam.getPose(firstKeyFrame - initialFrame).linear() << std::endl;
    std::cout << "First KeyFrame GroundTruth Position" << std::endl;
    std::cout << readPoseEigen(firstKeyFrame).translation() << std::endl;
    std::cout << "First KeyFrame GroundTruth Rotation" << std::endl;
    std::cout << readPoseEigen(firstKeyFrame).linear() << std::endl;
    std::cout << "-----------------------------------" << std::endl;
    std::cout << "Second KeyFrame Position" << std::endl;
    std::cout << myIsam.getPose(finalKeyFrame - initialFrame).translation() << std::endl;
    std::cout << "Second KeyFrame Rotation" << std::endl;
    std::cout << myIsam.getPose(finalKeyFrame - initialFrame).linear() << std::endl;
    std::cout << "Second KeyFrame GroundTruth Position" << std::endl;
    std::cout << readPoseEigen(finalKeyFrame).translation() << std::endl;
    std::cout << "Second KeyFrame GroundTruth Rotation" << std::endl;
    std::cout << readPoseEigen(finalKeyFrame).linear() << std::endl;

    return 0;
}
