#ifndef IK_H_INCLUDED
#define IK_H_INCLUDED

#include "curveevaluator.h"
#include "Eigen/Dense"
#include <vector>

using namespace Eigen;


class body
{
    
public:
    body(){ mLen = 0; mAngles.push_back(0); mAngles.push_back(0);};
    body(float len, float angle_z, float angle_y){mLen = len; mAngles.push_back(angle_z); mAngles.push_back(angle_y);};
    
    float mLen;
    std::vector<float> mAngles;
    
};


class IK
{
public:
   
    IK(){
    
        mStartPosition = VectorXf::Zero(3, 1);
        mCurPosition = VectorXf::Zero(3, 1);
        mGoalPostion = VectorXf::Zero(3, 1);
    
    };
    void start();
	
    VectorXf mStartPosition;
    VectorXf mCurPosition;
    VectorXf mGoalPostion;
    VectorXf dxdydz;
    
    
    MatrixXf jacobianInverse();
    
    void calculateAngles(VectorXf dxdydz);
    bool getGoal();
    
    std::vector<body*> mBodys;

};

#endif // IK_H_INCLUDED
