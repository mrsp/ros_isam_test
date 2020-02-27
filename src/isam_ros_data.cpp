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



#define USE_FIX_LANDMARK_COV 1


int main(int argc, char **argv)
{
    int initialFrame = 4; //from where to initialize ISAM
    int firstKeyFrame = 25; 
    int midKeyFrame = 50;
    int finalKeyFrame = 75;

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
    double factorTranslationNoiseStd = 1e-3;
    double factorOrientationNoiseStd = 1e-2;

    double LandMarkNoiseStd = 1e-2;
    double depth_sensor_noise_cov = 0.02;

    std::default_random_engine generator;
    std::normal_distribution<double> distribution0(0.0,factorTranslationNoiseStd);
    std::normal_distribution<double> distribution1(0.0,factorOrientationNoiseStd);

    Matrix<double, 6, 6>  frameCov =  Matrix<double, 6, 6>::Zero();
    frameCov.block<3,3>(0,0) = Matrix<double, 3, 3>::Identity() *factorTranslationNoiseStd*factorTranslationNoiseStd; //Setting Frame Covariance
    frameCov.block<3,3>(3,3) = Matrix<double, 3, 3>::Identity() *factorOrientationNoiseStd*factorOrientationNoiseStd; //Setting Frame Covariance

    Matrix<double, 6, 6>  initialISAMCov = Matrix<double, 6, 6>::Identity() * 1e-9;

    Matrix<double, 3, 3> LandMarkCov = Matrix<double, 3, 3>::Identity() * LandMarkNoiseStd * LandMarkNoiseStd;

    //ROS
    ros::init(argc, argv, "isam_ros_data");
    ros::NodeHandle n;

    kparams_t params;
    Isam myIsam(params);
    Affine3d  origin=readPoseGTEigen(initialFrame);
    //Affine3d  origin = Affine3d::Identity();
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
    prevPose = readPoseGTEigen(initialFrame);
    while (i < finalKeyFrame + 1)
    {
        //Add Factor to the Graph
        //myIsam.addFrame(readPoseEigen(i), frameCov);
        //Affine3d tempPose = prevPose.inverse()*readPoseGTEigen(i);
        Affine3d tempPose = readPoseGTEigen(i);
        // if(i>finalKeyFrame-3)
        // {
        tempPose.translation()(0) += distribution0(generator);
        tempPose.translation()(1) += distribution0(generator);
        tempPose.translation()(2) += distribution0(generator);
        Affine3d Pose = prevPose * tempPose;
        prevPose = Pose;
        myIsam.addFrame(tempPose, frameCov);
        // }
        // else
        // {
        //     myIsam.addFrame(readPoseGTEigen(i),Matrix<double,6,6>::Identity()*1e-10);
        // }
        
        GT.push_back(readPoseGTEigen(i));
        i++;
    }
    cout << "Added Factors " << (i-1) << endl;
    std::cout << "-----------------------------------" << std::endl;


    
    int landIdx;
    Vector3d landMarkPosFromPose0, landMarkPosFromPose1;
    Matrix3d landMarkCov0, landMarkCov1;

    // Matrix2d cov2d0, cov2d1;
    // vector<Vector3d> points0 = readPointsEigen(firstKeyFrame);
    // vector<Vector3d> normals0 = readNormalsEigen(firstKeyFrame);
    // vector<Vector3d> points1 = readPointsEigen(midKeyFrame);
    // vector<Vector3d> normals1 = readNormalsEigen(midKeyFrame);
    // vector<Vector3d> points2 = readPointsEigen(finalKeyFrame);
    // vector<Vector3d> normals2 = readNormalsEigen(finalKeyFrame);

    vector<Vector3d> keyp0 = readPointsEigen(4);
    vector<Vector3d> keyp1 = readPointsEigen(25);
    vector<Vector3d> keyp2 = readPointsEigen(50);
    vector<Vector3d> keyp3 = readPointsEigen(75);


    vector<Matrix3d> cov0 = readCovEigen(4);
    vector<Matrix3d> cov1 = readCovEigen(25);
    vector<Matrix3d> cov2 = readCovEigen(50);
    vector<Matrix3d> cov3 = readCovEigen(75);


     vector<Vector3d> corrVec00,corrVec01, corrVec11, corrVec12, corrVec22, corrVec23;
     vector<Matrix3d> corrCov00,corrCov01, corrCov11, corrCov12, corrCov22, corrCov23;
     vector<corr_t> corr0 = readCorr(4, 25);
     vector<corr_t> corr1 = readCorr(25, 50);
     vector<corr_t> corr2 = readCorr(50, 75);

    //Find Correspondeces between 0 and 1 Keyframe
    i=0;
    while(i<corr0.size())
    {
        corrVec00.push_back(keyp0[corr0[i].from]);
        corrVec01.push_back(keyp1[corr0[i].to]);
        corrCov00.push_back(cov0[corr0[i].from]);
        corrCov01.push_back(cov1[corr0[i].to]);
        i++;
    }
    //Find Correspondeces between 1 and 2 Keyframe
    i=0;
    while(i<corr1.size())
    {
        corrVec11.push_back(keyp1[corr1[i].from]);
        corrVec12.push_back(keyp2[corr1[i].to]);
        corrCov11.push_back(cov1[corr1[i].from]);
        corrCov12.push_back(cov2[corr1[i].to]);
        i++;
    }

    //Find Correspondeces between 2 and 3 Keyframe
    i=0;
    while(i<corr2.size())
    {
        corrVec22.push_back(keyp2[corr2[i].from]);
        corrVec23.push_back(keyp3[corr2[i].to]);
        corrCov22.push_back(cov2[corr2[i].from]);
        corrCov23.push_back(cov3[corr2[i].to]);
        i++;
    }


    // //Landmarks in Keyframe FirstKeyframe -- MidKeyFrame 
    i=0;
   
    while(i<corr0.size())
    {
      
        // Vector3d tempDist_ = Vector3d(i+1.00,i+1.00,i+1.00);
        // Vector3d tempDist0 = gt0.inverse() * tempDist_;
        // landMarkPosFromPose0 =tempDist0;
        //Vector3d tempDist1 = gt1.inverse()*tempDist_;
        //landMarkPosFromPose1 = tempDist1;
        landMarkPosFromPose0 = corrVec00[i];
        landMarkCov0 = corrCov00[i];
        landMarkPosFromPose1 = corrVec01[i];
        landMarkCov1 = corrCov01[i];
        landIdx = myIsam.addLandmark(Vector3d::Zero());
         #if USE_FIX_LANDMARK_COV
         myIsam.connectLandmark(landMarkPosFromPose0, landIdx,  initialFrame - initialFrame, LandMarkCov);
         myIsam.connectLandmark(landMarkPosFromPose1, landIdx,  firstKeyFrame - initialFrame,LandMarkCov);
         #else
         myIsam.connectLandmark(landMarkPosFromPose0, landIdx,  initialFrame - initialFrame , landMarkCov0);
         myIsam.connectLandmark(landMarkPosFromPose1, landIdx,  firstKeyFrame - initialFrame, landMarkCov1);
         #endif

        i++;
    }
   
    //Landmarks in Keyframe FirstKeyframe -- MidKeyFrame 
    i=0;
  
    while(i<corr1.size())
    {
       
        landIdx = myIsam.addLandmark(Vector3d::Zero());
       
        // Vector3d tempDist_ = Vector3d(i+10+1.00,i+10+1.00,i+10+1.00);
        // Vector3d tempDist0 = gt0.inverse() * tempDist_;
        //  landMarkPosFromPose0 =tempDist0;
        // Vector3d tempDist1 = gt1.inverse()*tempDist_;
        //  landMarkPosFromPose1 = tempDist1;


        landMarkPosFromPose0 = corrVec11[i];
        landMarkPosFromPose1 = corrVec12[i];
        landMarkCov0 = corrCov11[i];
        landMarkCov1 = corrCov12[i];
        
        #if USE_FIX_LANDMARK_COV
        myIsam.connectLandmark(landMarkPosFromPose0, landIdx, firstKeyFrame - initialFrame,  LandMarkCov);
        myIsam.connectLandmark(landMarkPosFromPose1, landIdx, midKeyFrame - initialFrame, LandMarkCov);
        #else
        myIsam.connectLandmark(landMarkPosFromPose0, landIdx, firstKeyFrame - initialFrame, landMarkCov0);
        myIsam.connectLandmark(landMarkPosFromPose1, landIdx, midKeyFrame - initialFrame, landMarkCov1);
        #endif
        i++;
    }


    //Landmarks in Keyframe FirstKeyframe -- FinalKeyFrame 
    i=0;
    // gt1 = GT[finalKeyFrame-initialFrame];
    // gt0 = GT[firstKeyFrame-initialFrame];
    
    while(i<corr2.size())
    {

        // Vector3d tempDist_ = Vector3d(i-1.00,i+3-1.00,i+5-1.00);
        // Vector3d tempDist0 = gt0.inverse() * tempDist_;
        //  landMarkPosFromPose0 =tempDist0;
        // Vector3d tempDist1 = gt1.inverse()*tempDist_;
        //  landMarkPosFromPose1 = tempDist1;
        landIdx = myIsam.addLandmark(Vector3d::Zero());
        landMarkPosFromPose0 = corrVec22[i];
        landMarkPosFromPose1 = corrVec23[i];
        landMarkCov0 = corrCov22[i];
        landMarkCov1 = corrCov23[i];
        #if USE_FIX_LANDMARK_COV
        myIsam.connectLandmark(landMarkPosFromPose0, landIdx, midKeyFrame - initialFrame, LandMarkCov);
        myIsam.connectLandmark(landMarkPosFromPose1, landIdx, finalKeyFrame - initialFrame, LandMarkCov);
        #else
        myIsam.connectLandmark(landMarkPosFromPose0, landIdx, midKeyFrame - initialFrame, landMarkCov0);
        myIsam.connectLandmark(landMarkPosFromPose1, landIdx, finalKeyFrame - initialFrame, landMarkCov1);
        #endif

        i++;
    }
   
    //for(  i = 0; i < myIsam.landmarks.size(); i++) cout << myIsam.landmarks[i]->value() << endl;
    //Optimize the Graph
    double error = myIsam.optimize(0);
    //for(  i = 0; i < myIsam.landmarks.size(); i++) cout << myIsam.landmarks[i]->value() << endl;

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
