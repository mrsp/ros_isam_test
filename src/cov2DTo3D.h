
#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>

Eigen::MatrixXd computeCov2DTo3D(Eigen::MatrixXd cov2D, double depth, double fx, double fy, double cx, double cy, double depth_noise_cov)
{

    Eigen::MatrixXd cov3D = Eigen::MatrixXd::Zero(3,3);


    Eigen::MatrixXd F = Eigen::MatrixXd::Zero(3,2);
    F(0,0) = depth / fx;
    F(1,1) = depth / fy;

    Eigen::MatrixXd L = Eigen::MatrixXd::Zero(3,3);
    L(0,0) = - cx / fx;
    L(1,1) = - cy / fy;
    L(2,2) = 1.00;

    Eigen::MatrixXd Qz = Eigen::MatrixXd::Zero(3,3);
    Qz(2,2) = depth_noise_cov;

    cov3D.noalias() = F*cov2D*F.transpose();

    cov3D.noalias() += L*Qz*L.transpose();

    return cov3D;
}














