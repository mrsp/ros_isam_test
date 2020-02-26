#ifndef ISAM_H
#define ISAM_H


#include<isam/slam3d.h>
#include<isam/Properties.h>
#include<isam/isam.h>
#include<isam/Point3d.h>
#include<isam/Pose3d.h>

#include"PoseGraph.h"
#include"kparams.h"
#include"constant_parameters.h"

class Isam :public PoseGraph
{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        typedef isam::Point3d_Node Landmark;

        //Isam(const sMatrix4 &initalPose);
        Isam(const kparams_t &params);
        void addFrame(const Affine3d &pose,const Matrix<double, 6, 6> &cov);


        //void addLandmark(float3 pos1,float3 pos2,double dist);


        //Adds a landmark and return its idx
        virtual int addLandmark(Vector3d pos) override;
        Landmark* addLandmark();
        virtual void connectLandmark(Vector3d pos,int landIdx,int poseIdx, Matrix3d &cov) override;

        virtual void init(const Affine3d &initalPose,const Matrix<double, 6, 6> &cov) override;
        virtual void clear() override;

        void addFixPose(const Affine3d &fixPose);
        double optimize(int frame) override;
        void addPoseConstrain(int p1,int p2,const Affine3d &delta, const  Matrix<double, 6, 6> &cov);
        //void convertCovariance(sMatrix4 pose,sMatrix6 cov) override;

        void clearLandmarks();

        void popFront() override;
        virtual Affine3d getPose(int i)  override;

        virtual uint poseSize() const override
        {
            return pose_nodes.size();
        }

        virtual uint landmarksSize() const override
        {
            return landmarks.size();
        }

        static isam::Pose3d toIsamPose(const Affine3d &pose);
        static Affine3d fromIsamNode(isam::Pose3d_Node *node);


        static isam::Point3d toIsamPoint(const Vector3d &f);

    public:
        isam::Slam *slam;
        const kparams_t &params;

        std::vector<isam::Pose3d_Node*> pose_nodes;
        std::vector<isam::Factor*> factors;
        isam::Factor *poseConstrainFactor;
        std::vector<isam::Point3d_Node*> landmarks;

        std::vector<isam::Factor*> landmarkFactors;
        Affine3d prevPose;
};
#endif
