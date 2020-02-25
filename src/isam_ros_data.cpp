#include <ros/ros.h>
#include "Isam.h"
#include "kparams.h"
#include <random>
#include "dataio.h"
#include "covPoint2Plane.h"

#include"cov2DTo3D.h"

Eigen::MatrixXd computeCov2DTo3DfromVert(Eigen::MatrixXd cov2D, Vector3d vertex, Matrix3d cam, double depth_noise_cov)
{
    double depth=vertex[2];
    double fx=cam(0,0);
    double fy=cam(1,1);
    double cx=cam(0,2);
    double cy=cam(1,2);
    

    Eigen::MatrixXd cov=computeCov2DTo3D(cov2D,depth,fx,fy,cx,cy,depth_noise_cov);
    return cov;

}

int main(int argc, char **argv)
{
    int initialFrame = 4; //from where to initialize ISAM
    int firstKeyFrame = 15; 
    int midKeyFrame = 25;
    int finalKeyFrame = 35;


    //Camera Matrix
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
    double factorNoiseStd = 0.05;
    double LandMarkNoiseStd = 0.01;
    double depth_sensor_noise_cov = 0.02;


    Matrix<double, 6, 6>  frameCov = Matrix<double, 6, 6>::Identity() * factorNoiseStd*factorNoiseStd; //Setting Frame Covariance
    Matrix<double, 6, 6> initialISAMCov = Matrix<double, 6, 6>::Identity() * 1e-6;

    
    


    //ROS
    ros::init(argc, argv, "isam_ros_data");
    ros::NodeHandle n;

    kparams_t params;
    Isam myIsam(params);
    Affine3d  origin=readPoseEigen(initialFrame);
    cout<<"Initializing isam at Frame: "<<initialFrame<<" with TF:"<<endl;
    cout<<origin.translation()<<endl;
    cout<<origin.linear()<<endl;
    cout<<"Initialized"<<endl;
    std::cout << "-----------------------------------" << std::endl;

    myIsam.init(origin, initialISAMCov);

    Affine3d prevPose=origin;
    vector<Affine3d> GT;
    GT.push_back(readPoseGTEigen(initialFrame));



    int i = initialFrame + 1;
    while (i < finalKeyFrame + 1)
    {
        //Add Factor to the Graph
        myIsam.addFrame(readPoseEigen(i), frameCov);
        GT.push_back(readPoseGTEigen(i));
        i++;
    }
    cout << "Added Factors " << (i-1) << endl;
    std::cout << "-----------------------------------" << std::endl;


    
    int landIdx;
    Vector3d landMarkPosFromPose0, landMarkPosFromPose1;
    Matrix2d cov2d0, cov2d1;
    Matrix3d landMarkCov0, landMarkCov1;

    vector<Vector3d> points0 = readPointsEigen(firstKeyFrame);
    vector<Vector3d> normals0 = readNormalsEigen(firstKeyFrame);
    vector<Vector3d> points1 = readPointsEigen(midKeyFrame);
    vector<Vector3d> normals1 = readNormalsEigen(midKeyFrame);
    vector<Vector3d> points2 = readPointsEigen(finalKeyFrame);
    vector<Vector3d> normals2 = readNormalsEigen(finalKeyFrame);

    vector<Vector2d> keyp0 = readKeypoints(1);
    vector<Vector2d> keyp1 = readKeypoints(2);
    vector<Vector2d> keyp2 = readKeypoints(3);
    vector<Vector2d> corrVec00,corrVec01, corrVec11, corrVec12;
    vector<corr_t> corr0 = readCorr(1, 2);
    vector<corr_t> corr1 = readCorr(2, 3);

    //Find Correspondeces between 0 and 1 Keyframe
    i=0;
    while(i<corr0.size())
    {
        corrVec00[i] = keyp0[corr0[i].from];
        corrVec01[i] = keyp1[corr0[i].to];
        i++;
    }
    //Find Correspondeces between 1 and 2 Keyframe
    i=0;
    while(i<corr1.size())
    {
        corrVec11[i] = keyp1[corr1[i].from];
        corrVec12[i] = keyp2[corr1[i].to];
        i++;
    }

    // //Landmarks in Keyframe FirstKeyframe -- MidKeyFrame 
    i=0;
    while(i<corr0.size())
    {
        landMarkPosFromPose0 = projectTo3D(camMatrix,  corrVec00[i](0), corrVec00[i](1), readDepthAtPixel(points0,(int) corrVec00[i](0), (int) corrVec00[i](1),480));
        landMarkPosFromPose1 = projectTo3D(camMatrix,  corrVec01[i](0), corrVec01[i](1), readDepthAtPixel(points1,(int) corrVec01[i](0), (int) corrVec01[i](1),480));
        cov2d0=Matrix2d::Identity()*LandMarkNoiseStd*LandMarkNoiseStd;
        cov2d1=Matrix2d::Identity()*LandMarkNoiseStd*LandMarkNoiseStd;

        landIdx = myIsam.addLandmark(Vector3d::Zero());
        landMarkCov0=computeCov2DTo3DfromVert(cov2d0,landMarkPosFromPose0,camMatrix,depth_sensor_noise_cov);
        landMarkCov1=computeCov2DTo3DfromVert(cov2d1,landMarkPosFromPose1,camMatrix,depth_sensor_noise_cov);
        myIsam.connectLandmark(landMarkPosFromPose0, landIdx, firstKeyFrame - initialFrame, landMarkCov0);
        myIsam.connectLandmark(landMarkPosFromPose1, landIdx, midKeyFrame - initialFrame, landMarkCov1);
    }
   
    // //Landmarks in Keyframe FirstKeyframe -- MidKeyFrame 
    i=0;
    while(i<corr1.size())
    {
        landMarkPosFromPose0 = projectTo3D(camMatrix,  corrVec11[i](0), corrVec11[i](1), readDepthAtPixel(points1,(int) corrVec11[i](0), (int) corrVec11[i](1),480));
        landMarkPosFromPose1 = projectTo3D(camMatrix,  corrVec12[i](0), corrVec12[i](1), readDepthAtPixel(points2,(int) corrVec12[i](0), (int) corrVec12[i](1),480));
        cov2d0=Matrix2d::Identity()*LandMarkNoiseStd*LandMarkNoiseStd;
        cov2d1=Matrix2d::Identity()*LandMarkNoiseStd*LandMarkNoiseStd;

        landIdx = myIsam.addLandmark(Vector3d::Zero());
        landMarkCov0=computeCov2DTo3DfromVert(cov2d0,landMarkPosFromPose0,camMatrix,depth_sensor_noise_cov);
        landMarkCov1=computeCov2DTo3DfromVert(cov2d1,landMarkPosFromPose1,camMatrix,depth_sensor_noise_cov);
        myIsam.connectLandmark(landMarkPosFromPose0, landIdx, midKeyFrame - initialFrame, landMarkCov0);
        myIsam.connectLandmark(landMarkPosFromPose1, landIdx, finalKeyFrame - initialFrame, landMarkCov1);
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
    std::cout << GT[firstKeyFrame - initialFrame].translation() << std::endl;
    std::cout << "First KeyFrame GroundTruth Rotation" << std::endl;
    std::cout <<  GT[firstKeyFrame - initialFrame].linear() << std::endl;
    std::cout << "-----------------------------------" << std::endl;
    std::cout << "Second KeyFrame Position" << std::endl;
    std::cout << myIsam.getPose(midKeyFrame - initialFrame).translation() << std::endl;
    std::cout << "Second KeyFrame Rotation" << std::endl;
    std::cout << myIsam.getPose(midKeyFrame - initialFrame).linear() << std::endl;
    std::cout << "Second KeyFrame GroundTruth Position" << std::endl;
    std::cout << GT[midKeyFrame - initialFrame].translation() << std::endl;
    std::cout << "Second KeyFrame GroundTruth Rotation" << std::endl;
    std::cout <<  GT[midKeyFrame- initialFrame].linear() << std::endl;
    std::cout << "-----------------------------------" << std::endl;
    std::cout << "Third KeyFrame Position" << std::endl;
    std::cout << myIsam.getPose(finalKeyFrame - initialFrame).translation() << std::endl;
    std::cout << "Third KeyFrame Rotation" << std::endl;
    std::cout << myIsam.getPose(finalKeyFrame - initialFrame).linear() << std::endl;
    std::cout << "Third KeyFrame GroundTruth Position" << std::endl;
    std::cout << GT[finalKeyFrame - initialFrame].translation() << std::endl;
    std::cout << "Third KeyFrame GroundTruth Rotation" << std::endl;
    std::cout <<  GT[finalKeyFrame- initialFrame].linear() << std::endl;

    return 0;
}
