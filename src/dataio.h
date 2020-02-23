
#include <string>
#include <fstream>
#include <iostream>
#include <vector>
#define DATA_DIR "/home/tavu/catkin_ws/src/ros_isam_test/data_2frames/"
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

void readPose(int frame, float p[4][4])
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

vector<corr_t> readCorr(int from, int to)
{
    vector<corr_t> ret;
    char buf[128];
    sprintf(buf, "%s/feat/corr_from%dto%d.txt", DATA_DIR, from, to);
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
