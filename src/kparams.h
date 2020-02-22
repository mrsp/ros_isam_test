#ifndef KPARAMS_H
#define KPARAMS_H

#include<vector>

typedef struct
{
    int compute_size_ratio=1;
    int integration_rate=1;
    int rendering_rate = 1;
    int tracking_rate=1;

    float optim_thr=100000000000;
    float cov_small=0;
    float cov_big=0;




    std::vector<int> pyramid = {10,5,4};
    float mu = 0.1;
    float icp_threshold = 1e-5;




} kparams_t;


#endif // KPARAMS_H
