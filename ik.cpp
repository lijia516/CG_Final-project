#include "ik.h"

#include <algorithm>
#include "modelerview.h"
#include "modelerdraw.h"
#include "graphwidget.h"
#include "animatoruiwindows.h"
#include "modelerapp.h"	// needed to read values from graph widget
#include "modelerui.h"	// needed to read values from graph widget


#include "mat.h"
#include <FL/gl.h>
#include <cstdlib>
#include <cassert>
#include <cmath>

using namespace std;

float PI = 3.1415926;

int jointNum = 2;


void IK::start(){
    
    ModelerApplication* app;
    int total_curves;	// This is the total number of items that appear in the
    // "model controls" pane, and equals the number of curves
    // for the model + number of curves for the camera.
    
    app = ModelerApplication::Instance();
    // The camera curves are always last in the list of "model curves" in the left pane.
    //total_curves = app->GetUI()->m_pwndGraphWidget->numCurves();
    //app->GetUI()->m_pwndGraphWidget->AddCtrlPt(1, 5,50);
    
    mCurPosition = mStartPosition;
    
    std::cout <<"goal:" << mGoalPostion(0) << "," << mGoalPostion(1) << "\n";
    std::cout <<"cur:" << mCurPosition(0) << "," << mCurPosition(1) << "\n";
    
    
    float dt = 0;
    
    float dx;
    float dy;
    
    dx = (mGoalPostion(0) - mCurPosition(0) > 0.1) ? 0.1 : -0.1;
    dy = (mGoalPostion(1) - mCurPosition(1) > 0.1) ? 0.1 : -0.1;
    
    
    
    std::cout <<"dx, dy:" << dx << "," << dy << "\n";

    
    app->GetUI()->m_pwndGraphWidget->ClearAllCtrlPt(23);
    app->GetUI()->m_pwndGraphWidget->ClearAllCtrlPt(32);
    
    while (!getGoal() && dt < 19) {
        
        dt += 0.5;
        
        std::cout <<"dt: "<< dt <<"\n";
        
        VectorXf dxdy = VectorXf::Zero(2, 1);
        dxdy(0) = dx;
        dxdy(1) = dy;
        
        calculateAngles(dxdy);
        app->GetUI()->m_pwndGraphWidget->AddCtrlPt(23, dt, mBodys[0]->mAngle * 180.0 / PI);
        app->GetUI()->m_pwndGraphWidget->AddCtrlPt(32, dt, mBodys[1]->mAngle * 180.0 / PI);
        
        
        dx = (mGoalPostion(0) - mCurPosition(0) > 0.1) ? 0.1 : -0.1;
        dy = (mGoalPostion(1) - mCurPosition(1) > 0.1) ? 0.1 : -0.1;
        
        
        std::cout <<"angle1, angle2:" << mBodys[0]->mAngle << "," << mBodys[1]->mAngle<<"\n";
//"," << mBodys[1]->mAngle << "\n";
        
    }
}



MatrixXf IK::jacobianInverse() {
    
    
    MatrixXf jacobianMatrx = MatrixXf::Zero(2,mBodys.size());
    
    
    for (int j = 0; j < mBodys.size(); j++) {
        
        float angle = 0;
        for (int i = 0; i < mBodys.size(); i++) {
            
            angle += mBodys[i]->mAngle;
            if (i >= j) {
                
                jacobianMatrx(0, j) += (float)(mBodys[i]->mLen * cos(angle));
                
                jacobianMatrx(1, j) += (float)(mBodys[i]->mLen * sin(angle));
                
            }
        }
    }

    
    MatrixXf jacobianMatrxInverse1 = MatrixXf::Zero(2,2);
    
    jacobianMatrxInverse1 = jacobianMatrx * (jacobianMatrx.transpose());
    
    
    float m00 = roundf(jacobianMatrxInverse1(0,0) * 1000000) / 1000000.0;
    float m01 = roundf(jacobianMatrxInverse1(0,1) * 1000000) / 1000000.0;
    float m10 = roundf(jacobianMatrxInverse1(1,0) * 1000000) / 1000000.0;
    float m11 = roundf(jacobianMatrxInverse1(1,1) * 1000000) / 1000000.0;

    
    Matrix2f jacobianMatrxInverse;
    
    jacobianMatrxInverse << m00, m01, m10, m11;
    
    return (jacobianMatrx.transpose() * jacobianMatrxInverse.inverse());

}



void IK::calculateAngles(VectorXf dxdy) {
    
    VectorXf dAngles = VectorXf::Zero(mBodys.size(), 1);
    
    dAngles = jacobianInverse() * dxdy;
    
    
    std::cout <<"jacobianInverse():" <<  jacobianInverse() (0,0) << "\n";
    std::cout <<"dAngles:" <<  dAngles << "\n";
    
    
    for (int i = 0; i < mBodys.size(); i++) {
        mBodys[i]->mAngle += dAngles(i);
    }
    
    
    for (int i = 0; i < mBodys.size(); i++) {
        
        while (mBodys[i]->mAngle < -2 * PI) {
            
            mBodys[i]->mAngle += PI;
        }
        
        while (mBodys[i]->mAngle > 2 * PI) {
            
            mBodys[i]->mAngle -= PI;
        }
    }
    
    
    
    float angle = 0;
    mCurPosition(0) = ParticleSystem::cloth_start[0];
    mCurPosition(1) = ParticleSystem::cloth_start[1];
    
    for (int i = 0; i < mBodys.size(); i++) {
        
        angle += mBodys[i]->mAngle;
        mCurPosition(0) += mBodys[i]->mLen * sin(angle);
        mCurPosition(1) -= mBodys[i]->mLen * cos(angle);
    }
    
}

bool IK::getGoal() {
    
    std::cout <<"fabs(mCurPosition(0) - mGoalPostion(0)):" << fabs(mCurPosition(0) - mGoalPostion(0)) << "\n";
    
    std::cout <<"fabs(mCurPosition(1) - mGoalPostion(1)):" << fabs(mCurPosition(1) - mGoalPostion(1)) << "\n";
    
    
    if (fabs(mCurPosition(0) - mGoalPostion(0)) <= 0.1f && fabs(mCurPosition(1) - mGoalPostion(1)) <= 0.1f) {
        
        std::cout << "true \n";
        
        return true;
    }
    
    std::cout << "false \n";
    return false;
}

