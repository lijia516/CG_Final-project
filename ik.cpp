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
    
    std::cout <<"goal:" << mGoalPostion(0) << "," << mGoalPostion(1) << "," << mGoalPostion(2)<< "\n";
    std::cout <<"cur:" << mCurPosition(0) << "," << mCurPosition(1) << "," << mCurPosition(2) << "\n";
    
    
    float dt = 0;
    
    float dx;
    float dy;
    float dz;
    
    dx = (mGoalPostion(0) - mCurPosition(0) > 0.1) ? 0.1 : -0.1;
    dy = (mGoalPostion(1) - mCurPosition(1) > 0.1) ? 0.1 : -0.1;
    dz = (mGoalPostion(2) - mCurPosition(2) > 0.1) ? 0.1 : -0.1;
    
    
    std::cout <<"dx, dy, dz:" << dx << "," << dy << "," << dz << "\n";

    
    app->GetUI()->m_pwndGraphWidget->ClearAllCtrlPt(23);
    app->GetUI()->m_pwndGraphWidget->ClearAllCtrlPt(22);
    
    app->GetUI()->m_pwndGraphWidget->ClearAllCtrlPt(32);
    app->GetUI()->m_pwndGraphWidget->ClearAllCtrlPt(31);
    
    while (!getGoal() && dt < 19) {
        
        dt += 0.3;
        
        std::cout <<"dt: "<< dt <<"\n";
        
        VectorXf dxdy = VectorXf::Zero(3, 1);
        dxdy(0) = dx;
        dxdy(1) = dy;
        dxdy(2) = dz;
        
        calculateAngles(dxdy);
        app->GetUI()->m_pwndGraphWidget->AddCtrlPt(23, dt, mBodys[0]->mAngles[0] * 180.0 / PI);
        app->GetUI()->m_pwndGraphWidget->AddCtrlPt(22, dt, mBodys[0]->mAngles[1] * 180.0 / PI);
        
        app->GetUI()->m_pwndGraphWidget->AddCtrlPt(32, dt, mBodys[1]->mAngles[0] * 180.0 / PI);
        app->GetUI()->m_pwndGraphWidget->AddCtrlPt(31, dt, mBodys[1]->mAngles[1] * 180.0 / PI);
        
        
        //dx = (mGoalPostion(0) - mCurPosition(0) > 0.1) ? 0.1 : -0.1;
        //dy = (mGoalPostion(1) - mCurPosition(1) > 0.1) ? 0.1 : -0.1;
        //dz = (mGoalPostion(2) - mCurPosition(2) > 0.1) ? 0.1 : -0.1;
        
        
        if (mGoalPostion(0) - mCurPosition(0) == 0) {
            
            dx = 0;
        
        } else if (mGoalPostion(0) - mCurPosition(0) > 0.5) {
            
            dx = 0.1;
            
        } else if (mGoalPostion(0) - mCurPosition(0) > 0){
            
            dx = 0.05;
        } else if (mGoalPostion(0) - mCurPosition(0) > -0.5) {
            
            dx = -0.05;
        } else {
            
            dx = -0.1;
        }
        
        if (mGoalPostion(1) - mCurPosition(1) == 0) {
        
            dy = 0;
        
        } else if (mGoalPostion(1) - mCurPosition(1) > 0.5) {
            
            dy = 0.1;
            
        } else if (mGoalPostion(1) - mCurPosition(1) > 0){
            
            dy = 0.05;
        } else if (mGoalPostion(1) - mCurPosition(1) > -0.5) {
            
            dy = -0.05;
        } else {
            
            dy = -0.1;
        }
        
        
        if (mGoalPostion(2) - mCurPosition(2) == 0) {
        
            dz = 0;
        
        } else if (mGoalPostion(2) - mCurPosition(2) > 0.5) {
            
            dz = 0.1;
            
        } else if (mGoalPostion(2) - mCurPosition(2) > 0){
            
            dz = 0.05;
            
        } else if (mGoalPostion(2) - mCurPosition(2) > -0.5) {
            
            dz = -0.05;
        } else {
            
            dz = -0.1;
        }


        
        std::cout <<"ls: angle1, angle2:" << mBodys[0]->mAngles[0] << "," << mBodys[0]->mAngles[1]<<"\n";
        std::cout <<"lf: angle1, angle2:" << mBodys[1]->mAngles[0] << "," << mBodys[1]->mAngles[1]<<"\n";
        
        
//"," << mBodys[1]->mAngle << "\n";
        
    }
}



MatrixXf IK::jacobianInverse() {
    
    
    MatrixXf jacobianMatrx = MatrixXf::Zero(3, mBodys.size() * 2);
    
    
    for (int j = 0; j < mBodys.size(); j++) {
        
        float angle_z = 0;
        float angle_y = 0;
        
        for (int i = 0; i < mBodys.size(); i++) {
            
            angle_z += mBodys[i]->mAngles[0];
            angle_y += mBodys[i]->mAngles[1];
            
            
            if (i >= j) {
                
                jacobianMatrx(0, j*2) += (float)(mBodys[i]->mLen * cos(angle_z) * cos(angle_y));
                jacobianMatrx(1, j*2) += (float)(mBodys[i]->mLen * sin(angle_z));
                jacobianMatrx(2, j*2) += (float)(- mBodys[i]->mLen * cos(angle_z) * sin(angle_y));
                
                
                jacobianMatrx(0, j*2+1) += (float)(-mBodys[i]->mLen * sin(angle_z) * sin(angle_y));
                jacobianMatrx(1, j*2+1) += (float)0.0f;
                jacobianMatrx(2, j*2+1) += (float)(- mBodys[i]->mLen * sin(angle_z) * cos(angle_y));
            
            }
        }
    }

    
    MatrixXf jacobianMatrxInverse1 = MatrixXf::Zero(3,3);
    
    jacobianMatrxInverse1 = jacobianMatrx * (jacobianMatrx.transpose());
    
    
    float m00 = roundf(jacobianMatrxInverse1(0,0) * 1000000) / 1000000.0;
    float m01 = roundf(jacobianMatrxInverse1(0,1) * 1000000) / 1000000.0;
    float m02 = roundf(jacobianMatrxInverse1(0,2) * 1000000) / 1000000.0;
    float m10 = roundf(jacobianMatrxInverse1(1,0) * 1000000) / 1000000.0;
    float m11 = roundf(jacobianMatrxInverse1(1,1) * 1000000) / 1000000.0;
    float m12 = roundf(jacobianMatrxInverse1(1,2) * 1000000) / 1000000.0;
    float m20 = roundf(jacobianMatrxInverse1(2,0) * 1000000) / 1000000.0;
    float m21 = roundf(jacobianMatrxInverse1(2,1) * 1000000) / 1000000.0;
    float m22 = roundf(jacobianMatrxInverse1(2,2) * 1000000) / 1000000.0;
    
    Matrix3f jacobianMatrxInverse;
    
    jacobianMatrxInverse << m00, m01, m02, m10, m11, m12 , m20, m21, m22;
    
    return (jacobianMatrx.transpose() * jacobianMatrxInverse.inverse());

}



void IK::calculateAngles(VectorXf dxdydz) {
    
    VectorXf dAngles = VectorXf::Zero(mBodys.size() * 2, 1);
    
    
    
    dAngles = jacobianInverse() * dxdydz;
    
    
    
    std::cout <<"jacobianInverse():" <<  jacobianInverse() << "\n";
    std::cout <<"dAngles:" <<  dAngles << "\n";
    
    
    for (int i = 0; i < mBodys.size(); i++) {
        
        for (int j = 0; j < 2; j++) {
        
            mBodys[i]->mAngles[j] += dAngles(i*2 + j);
            
        }
    }
    
   
    for (int i = 0; i < mBodys.size(); i++) {
        
        for (int j = 0; j < 2; j++) {
        
            while (mBodys[i]->mAngles[j] < -2 * PI) {
                
                mBodys[i]->mAngles[j] += PI;
            }
            
            while (mBodys[i]->mAngles[j] > 2 * PI) {
                
                mBodys[i]->mAngles[j] -= PI;
            }
            
            
        }
    }
    
    
    float angle_z = 0;
    float angle_y = 0;
    
    mCurPosition(0) = ParticleSystem::cloth_start[0];
    mCurPosition(1) = ParticleSystem::cloth_start[1];
    mCurPosition(2) = ParticleSystem::cloth_start[2];
    
    for (int i = 0; i < mBodys.size(); i++) {
        
        angle_z += mBodys[i]->mAngles[0];
        angle_y += mBodys[i]->mAngles[1];
        
        mCurPosition(0) += mBodys[i]->mLen * sin(angle_z) * cos(angle_y);
        mCurPosition(1) -= mBodys[i]->mLen * cos(angle_z);
        mCurPosition(2) -= mBodys[i]->mLen * sin(angle_z) * sin(angle_y);
    }
     
    
    
}

bool IK::getGoal() {
    
    std::cout <<"fabs(mCurPosition(0) - mGoalPostion(0)):" << fabs(mCurPosition(0) - mGoalPostion(0)) << "\n";
    
    std::cout <<"fabs(mCurPosition(1) - mGoalPostion(1)):" << fabs(mCurPosition(1) - mGoalPostion(1)) << "\n";
    
    
    if (fabs(mCurPosition(0) - mGoalPostion(0)) <= 0.1f && fabs(mCurPosition(1) - mGoalPostion(1)) <= 0.1f && fabs(mCurPosition(2) - mGoalPostion(2)) <= 0.1f) {
        
        std::cout << "true \n";
        
        return true;
    }
    
    std::cout << "false \n";
    return false;
}

