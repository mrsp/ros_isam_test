#include <ros/ros.h>
#include "Isam.h"
#include "kparams.h"
#include <random>
#include "dataio.h"

int main(int argc, char **argv)
{
    int initialFrame = 10; //from where to initialize ISAM
    int firstKeyFrame = 25; 
    int finalKeyFrame = 50;

  

    //Gaussian Noise
    double factorNoiseStd = 0.1;
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
    myIsam.init(readPoseEigen(initialFrame), initialISAMCov);

    int i = initialFrame + 1;
    while (i < finalKeyFrame + 1)
    {
        //Add Factor to the Graph
        Affine3d tempPose = readPoseEigen(i);
        tempPose.translation() += Vector3d(distribution(generator),distribution(generator),distribution(generator));

        myIsam.addFrame(tempPose, frameCov);
        i++;
    }
    cout << "Added Factors " << i << endl;

    // //LANDMARKS
    int landIdx;
    Vector3d landMarkPosFromPose0, landMarkPosFromPose1;
    vector<Vector3d> points0 = readPointsEigen(firstKeyFrame);
    vector<Vector3d> points1 = readPointsEigen(finalKeyFrame);
    vector<corr_t> corr = readCorr(firstKeyFrame, finalKeyFrame);

    i = 0;
    while (i < corr.size())
    {

        landIdx = myIsam.addLandmark(Vector3d::Zero());
        landMarkPosFromPose0 = points0[corr[i].from];
        landMarkPosFromPose1 = points1[corr[i].to];

        myIsam.connectLandmark(landMarkPosFromPose0, landIdx, firstKeyFrame - initialFrame, landMarkCov);

        myIsam.connectLandmark(landMarkPosFromPose1, landIdx, finalKeyFrame - initialFrame, landMarkCov);

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