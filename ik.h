#ifndef IK_H_INCLUDED
#define IK_H_INCLUDED

#undef Success
#include "curveevaluator.h"
#include "Eigen/Dense"
#include <vector>

using namespace Eigen;


class body
{
    
public:
    body(){ mLen = 0; mAngle = 0;};
    body(float len, float angle){mLen = len; mAngle = angle;};
    
    float mLen;
    float mAngle;
    
};


class IK
{
public:
   
    IK(){
    
        mStartPosition = VectorXf::Zero(2, 1);
        mCurPosition = VectorXf::Zero(2, 1);
        mGoalPostion = VectorXf::Zero(2, 1);
    
    };
    void start();
	
    VectorXf mStartPosition;
    VectorXf mCurPosition;
    VectorXf mGoalPostion;
    VectorXf dxdy;
    
    
    MatrixXf jacobianInverse();
    
    void calculateAngles(VectorXf dxdy);
    bool getGoal();
    
    std::vector<body*> mBodys;

};

#endif // IK_H_INCLUDED
