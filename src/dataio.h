
#include <string>
#include <fstream>
#include <iostream>
#include <vector>
#define DATA_DIR "/media/master/HDD/VIPGPUSLAM_Experiments/data"
using namespace std;

struct descr_t
{
    float data[128];
    float size;
};

struct float3
{
    float x;
    float y;
    float z;
};

struct corr_t
{
    int from;
    int to;
};

vector<descr_t> readDescr(int frame)
{
    vector<descr_t> ret;

    char buf[128];
    sprintf(buf, "%s/feat/descr%d.txt", DATA_DIR, frame);
    std::ifstream infile(buf, std::ifstream::in);
    while (infile.good())
    {
        descr_t d;
        for (int i = 0; i < 128; i++)
        {
            infile >> d.data[i];
        }
        infile >> d.size;
        ret.push_back(d);
    }
    infile.close();
    return ret;
};


vector<Vector2d> readKeypoints(int frame)
{
    vector<Vector2d> ret;

    char buf[128];
    sprintf(buf, "%s/feat/KEPS%d.txt", DATA_DIR, frame);
    std::ifstream infile(buf, std::ifstream::in);
    while (infile.good())
    {
        Vector2d d;
        infile >> d(0);
        infile >> d(1);        
        ret.push_back(d);
    }
    infile.close();
    return ret;
};


vector<float3> readPoints(int frame)
{
    vector<float3> ret;

    char buf[128];
    sprintf(buf, "%s/feat/points%d.txt", DATA_DIR, frame);
    std::ifstream infile(buf, std::ifstream::in);
    while (infile.good())
    {
        float3 p;
        infile >> p.x;
        infile >> p.y;
        infile >> p.z;
        ret.push_back(p);
    }
    infile.close();
    return ret;
};

vector<Vector3d> readPointsEigen(int frame)
{
    vector<Vector3d> ret;

    char buf[128];
    sprintf(buf, "%s/feat/points%d.txt", DATA_DIR, frame);
    std::ifstream infile(buf, std::ifstream::in);
    while (infile.good())
    {
        Vector3d p;
        infile >> p(0);
        infile >> p(1);
        infile >> p(2);
        ret.push_back(p);
    }
    infile.close();
    return ret;
};

vector<Matrix3d> readCovEigen(int frame)
{
    vector<Matrix3d> ret;

    char buf[128];
    sprintf(buf, "%s/feat/cov%d.txt", DATA_DIR, frame);
    std::ifstream infile(buf, std::ifstream::in);
    while (infile.good())
    {
        Matrix3d p;
        infile >> p(0,0);
        infile >> p(0,1);
        infile >> p(0,2);
        infile >> p(1,0);
        infile >> p(1,1);
        infile >> p(1,2);
        infile >> p(2,0);
        infile >> p(2,1);
        infile >> p(2,2);   
        ret.push_back(p);
    }
    infile.close();
    return ret;
};


vector<Vector3d> readNormalsEigen(int frame)
{
    vector<Vector3d> ret;

    char buf[128];
    sprintf(buf, "%s/norm/norm%d.txt", DATA_DIR, frame);
    std::ifstream infile(buf, std::ifstream::in);
    while (infile.good())
    {
        Vector3d p;
        infile >> p(0);
        infile >> p(1);
        infile >> p(2);
        ret.push_back(p);
    }
    infile.close();
    return ret;
};

void readPose(int frame, float p[4][4])
{
    char buf[128];
    sprintf(buf, "%s/poses/pose%d", DATA_DIR, frame);
    std::ifstream infile(buf, std::ifstream::in);

    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            infile >> p[i][j];
        }
    }
    infile.close();
};

vector<corr_t> readCorr(int from, int to)
{
    vector<corr_t> ret;
    char buf[128];
    sprintf(buf, "%s/feat/corr_from_%d_to%d.txt", DATA_DIR, from, to);
    std::ifstream infile(buf, std::ifstream::in);
    while (infile.good())
    {
        corr_t c;

        infile >> c.from;
        infile >> c.to;
        ret.push_back(c);
    }
    infile.close();
    return ret;
};

Affine3d readPoseEigen(int idx)
{
    Affine3d ret = Affine3d::Identity();
    float p[4][4];
    readPose(idx, p);
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            ret(i, j) = p[i][j];
        }
    }
    return ret;
};


void readGTPose(int frame, float p[4][4])
{
    char buf[128];
    sprintf(buf, "%s/gt/pose%d", DATA_DIR, frame);
    std::ifstream infile(buf, std::ifstream::in);

    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            infile >> p[i][j];
        }
    }
    infile.close();
};

Affine3d readPoseGTEigen(int idx)
{
    Affine3d ret = Affine3d::Identity();
    float p[4][4];
    readGTPose(idx, p);
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            ret(i, j) = p[i][j];
        }
    }
    return ret;
};


double readDepthAtPixel(vector<Vector3d> vertex,int x, int y, int sizey)
{
    return vertex[x*sizey+y](2);
}

Vector3d projectTo3D(Matrix3d camMatrix, double x, double y, double depth)
{
    Vector3d ret = Vector3d::Zero();
    ret(0) = (x - camMatrix(0,2))/camMatrix(0,0) * depth;
    ret(1) = (y - camMatrix(1,2))/camMatrix(1,1) * depth;
    ret(2) = depth;
    return ret;
}