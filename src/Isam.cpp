#include "Isam.h"


using namespace isam;

namespace isam
{
    const int Pose3d::dim;
}


Isam::Isam(const kparams_t &par)
    :params(par),
      poseConstrainFactor(0)
{
}

void Isam::clearLandmarks()
{
    for(int i=0;i<landmarks.size();i++)
    {
        Landmark *ld=landmarks[i];
        slam->remove_node(ld);
        delete ld;
    }
    for(int i=0;i<landmarkFactors.size();i++)
    {
        isam::Factor *f=landmarkFactors[i];
        delete f;
    }
    landmarks.clear();
    landmarkFactors.clear();
}

void Isam::popFront()
{
    Pose3d_Node *node=pose_nodes[0];
    isam::Factor *factor=factors[0];
    pose_nodes.erase(pose_nodes.begin() );
    factors.erase(factors.begin() );

    slam->remove_node(node);
    delete factor;
    delete node;

    if(poseConstrainFactor!=0)
    {
        delete poseConstrainFactor;
        poseConstrainFactor=0;
    }
}

void Isam::addFrame(const Affine3d &pose,const  Matrix<double, 6, 6> &cov)
{
    Affine3d delta= prevPose.inverse()*pose;
    prevPose=pose;

    Noise noise = isam::Covariance(cov);

    Pose3d vo=toIsamPose(delta);
    Pose3d_Node* new_pose_node = new Pose3d_Node();
    slam->add_node(new_pose_node);
    Pose3d_Pose3d_Factor* factor = new Pose3d_Pose3d_Factor(pose_nodes.back(),
                                                            new_pose_node, vo, noise);

    pose_nodes.push_back(new_pose_node);

    factors.push_back(factor);
    slam->add_factor(factor);
}

Isam::Landmark* Isam::addLandmark()
{

    Landmark *landmark=new Landmark();
    landmarks.push_back(landmark);

    slam->add_node(landmark);

    return landmark;

}

int Isam::addLandmark(Vector3d pos)
{
    (void)pos;
    int ret=landmarks.size();

    Point3d_Node *landmark=new Point3d_Node();
    landmarks.push_back(landmark);

    slam->add_node(landmark);

    return ret;
}

void Isam::connectLandmark(Vector3d pos,int landIdx,int poseIdx, Matrix3d &cov)
{
    if(poseIdx<0)
        poseIdx=poseSize()+poseIdx;
    
    Point3d point=toIsamPoint(pos);

    Noise noise = isam::Covariance(cov);

    Pose3d_Point3d_Factor* f=new Pose3d_Point3d_Factor(
                                           pose_nodes[poseIdx],landmarks[landIdx],point,noise);

    landmarkFactors.push_back(f);
    slam->add_factor(f); 
}

void Isam::addFixPose(const Affine3d &fixPose)
{
    Matrix<double,6,6> cov;
    cov=cov*params.cov_small;

    Affine3d delta=fixPose;
    
    Noise noise = isam::Covariance(cov);
    Pose3d vo= toIsamPose(delta);
    Pose3d_Pose3d_Factor* factor = new Pose3d_Pose3d_Factor(pose_nodes.front(),
                                                            pose_nodes.back(), vo, noise);
    
    factors.push_back(factor);

    slam->add_factor(factor);
}

void Isam::addPoseConstrain(int p1,int p2,const Affine3d &delta, const Matrix<double,6,6> &cov)
{

    Noise noise = isam::Covariance(cov);
    Pose3d vo= toIsamPose(delta);

    Pose3d_Pose3d_Factor* factor = new Pose3d_Pose3d_Factor(pose_nodes[p1],
                                                            pose_nodes[p2],
                                                            vo,
                                                            noise);

    poseConstrainFactor=factor;
    //factors.push_back(factor);

    slam->add_factor(factor);
}

double Isam::optimize(int frame)
{
    //std::cout<<"Start isam optimization"<<std::endl;
    
    //char buf[32];
    //sprintf(buf,"data/isam/f_%d_graph",frame);
    //slam->save(buf);

    slam->batch_optimization();


    //sprintf(buf,"data/isam/f_%d_graph_opt",frame);
    //slam->save(buf);

    //slam->print_stats();
    
    double error=slam->chi2();    

    //std::cout<<"End isam optimization"<<std::endl;
    return error;
}

void Isam::init(const Affine3d &initialPose,const Matrix<double,6,6> &cov)
{
    slam=new Slam();
    prevPose=initialPose;

    //Disable isam prints
    isam::Properties props=slam->properties();
    props.max_iterations=10;
    props.quiet=true;
    //props.method = DOG_LEG;
    props.method=LEVENBERG_MARQUARDT;
    slam->set_properties(props);

    Pose3d_Node *initial_pose_node = new Pose3d_Node();
    pose_nodes.push_back(initial_pose_node);
    slam->add_node(initial_pose_node);

    Noise noise = isam::Covariance(cov);

    Pose3d origin=toIsamPose(initialPose);
//     Noise noise =  isam::Covariance(Eigen::MatrixXd::Identity(6, 6)*params.cov_small );

    Pose3d_Factor* prior = new Pose3d_Factor(initial_pose_node, origin, noise);
    factors.push_back(prior);
    slam->add_factor(prior);
}

Affine3d Isam::getPose(int i) 
{
    return fromIsamNode(pose_nodes[i]);
}

void Isam::clear()
{
//    clearLandmarks();
    //slam->save("isam.graph");
    if(slam!=nullptr)
    {
        delete slam;
        slam=nullptr;
    }

    for(uint i=0; i<pose_nodes.size();i++ )
    {
        Pose3d_Node *node=pose_nodes[i];
        delete node;
    }

    for(uint i=0; i<factors.size();i++ )
    {
        Factor *f=factors[i];
        delete f;
    }

    for(uint i=0; i<landmarks.size();i++ )
    {
        isam::Point3d_Node *l=landmarks[i];
        delete l;
    }

    for(uint i=0; i<landmarkFactors.size();i++ )
    {
        Factor *f=landmarkFactors[i];
        delete f;
    }

    pose_nodes.clear();
    factors.clear();
    landmarkFactors.clear();
    landmarks.clear();
}

Point3d Isam::toIsamPoint(const Vector3d &f)
{
//    Vector3d p=fromVisionCordV(f);
    Vector3d p=f;
    return Point3d(p(0), p(1),p(2));
}

Pose3d Isam::toIsamPose(const Affine3d &pose)
{
//     Affine3d pose_trans=fromVisionCord(pose);
    Affine3d pose_trans=pose;
    Matrix4f eigen_p;
    Matrix3f rot;
    for (int i=0;i<4;i++)
    {
        for(int j=0;j<4;j++)
        {
            eigen_p(i,j)=pose_trans(i,j);
        }
    }

    isam::Pose3d delta(eigen_p.cast<double>() );
    return delta;
}

Affine3d Isam::fromIsamNode(Pose3d_Node *node)
{
    Affine3d ret;
    Matrix4d eigen_p=node->value().wTo();

    
    for (int i=0;i<4;i++)
        for(int j=0;j<4;j++)
            ret(i,j) = eigen_p(i,j);

//     ret=toVisionCord(ret);
    return ret;
}
