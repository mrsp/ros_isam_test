#ifndef POSE_GRAPH_H 
#define POSE_GRAPH_H

#include <eigen3/Eigen/Dense>
using namespace Eigen;
class PoseGraph
{
    public:
        PoseGraph(){}
        virtual ~PoseGraph() {}
        virtual void init(const Affine3d  &InitialPose, const Matrix<double, 6, 6> &InitialCov) = 0;
        virtual void addFrame(const Affine3d &pose,const Matrix<double, 6, 6> &cov) = 0;
        virtual int addLandmark(const Vector3d pos) = 0;
        virtual void connectLandmark(Vector3d pos,int landIdx,int poseIdx, Matrix3d &cov) = 0;
        virtual void connectLandmark(Vector2d pos, int lid, double Sigma2){}
        
        virtual void clear() = 0;
        virtual void popFront()=0;
        virtual double optimize(int frame) = 0;
        
        virtual Affine3d getPose(int i)  = 0;
        virtual uint poseSize() const = 0;
        virtual uint landmarksSize() const = 0;
        
        virtual void addFixPose(const Affine3d &fixPose) = 0;
        virtual void addPoseConstrain(int p1,int p2,const Affine3d &pose, const Matrix<double, 6, 6> &cov) = 0;
        
        inline bool hasLandmarks() const
        {
            return landmarksSize()>0;
        }
        
};
#endif
