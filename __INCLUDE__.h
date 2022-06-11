//
// Created by 75742 on 2022/6/10.
//

#ifndef CLIONPROJECT___INCLUDE___H
#define CLIONPROJECT___INCLUDE___H
#include <iostream>
#include <cstdio>
#include <cmath>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <stdexcept>
#include <map>
#include <algorithm>
#include <vector>
#include <utility>
using namespace Eigen;
using namespace std;

template <typename VarType>
pair<VarType, VarType> cul_max_min(vector<VarType> &vec){
    VarType max = vec[0];
    VarType min = vec[0];
    for(auto x : vec){
        if(x > max){
            max = x;
        }
        if(x < min){
            min = x;
        }
    }
    return {max, min};
}

#endif //CLIONPROJECT___INCLUDE___H
