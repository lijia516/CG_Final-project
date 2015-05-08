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

#include <FL/gl.h>
#include <cstdlib>

using namespace std;

float PI = M_PI;

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
    
    
    std::cout <<"goal:" << mGoalPostion(0) << "," << mGoalPostion(1) << "," << mGoalPostion(2)<< "\n";
    std::cout <<"cur:" << mCurPosition(0) << "," << mCurPosition(1) << "," << mCurPosition(2) << "\n";
    
    std::cout <<"armstart:" << mArmStartPostion(0) << "," << mArmStartPostion(1) << "," << mArmStartPostion(2) << "\n";
    std::cout <<"pel:" << mPelPostion(0) << "," << mPelPostion(1) << "," << mPelPostion(2) << "\n";


    
    app->GetUI()->m_pwndGraphWidget->ClearAllCtrlPt(0);
    
    float dt = 0.5;
    
    // check if need to rotate body
    
    float angle = rotateBody();
    
    if (angle != 0) {
        
        
        updateHeadDir(angle);
        updateArmPosition();
        updateCurPositionInitial();
        app->GetUI()->m_pwndGraphWidget->AddCtrlPt(dt, 1, angle * 180.0 / PI);
        
    }

    
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
        
        dt += 0.5;
        
        std::cout <<"dt: "<< dt <<"\n";
        
        VectorXf dxdy = VectorXf::Zero(3, 1);
        dxdy(0) = dx;
        dxdy(1) = dy;
        dxdy(2) = dz;
        
        calculateAngles(dxdy, -angle);
        app->GetUI()->m_pwndGraphWidget->AddCtrlPt(23, dt, mBodys[0]->mAngles[0] * 180.0 / PI);
        app->GetUI()->m_pwndGraphWidget->AddCtrlPt(22, dt, mBodys[0]->mAngles[1] * 180.0 / PI);
        
        app->GetUI()->m_pwndGraphWidget->AddCtrlPt(32, dt, mBodys[1]->mAngles[0] * 180.0 / PI);
        app->GetUI()->m_pwndGraphWidget->AddCtrlPt(31, dt, mBodys[1]->mAngles[1] * 180.0 / PI);
        
        
        
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
     //   std::cout <<"lf: angle1, angle2:" << mBodys[1]->mAngles[0] << "," << mBodys[1]->mAngles[1]<<"\n";
        
        
    }
}



MatrixXf IK::jacobianInverse(float angle) {
    
    
    MatrixXf jacobianMatrx = MatrixXf::Zero(3, mBodys.size() * 2);
    
    
    for (int j = 0; j < mBodys.size(); j++) {
        
        float angle_z = 0;
        float angle_y = 0;
        
        for (int i = 0; i < mBodys.size(); i++) {
            
            angle_z += mBodys[i]->mAngles[0];
            angle_y += mBodys[i]->mAngles[1];
            
            
            if (i >= j) {
                
             //   jacobianMatrx(0, j*2) += (float)(mBodys[i]->mLen * cos(angle_z) * cos(angle_y));
             //   jacobianMatrx(1, j*2) += (float)(mBodys[i]->mLen * sin(angle_z));
             //   jacobianMatrx(2, j*2) += (float)(- mBodys[i]->mLen * cos(angle_z) * sin(angle_y));
                
                
                jacobianMatrx(0, j*2) += (float)(mBodys[i]->mLen * cos(angle_z) * (cos(angle) * cos(angle_y) + sin(angle) * sin(angle_y)));
                                                 
                jacobianMatrx(1, j*2) += (float)(mBodys[i]->mLen * sin(angle_z));
                                                 
                jacobianMatrx(2, j*2) += (float)(mBodys[i]->mLen * cos(angle_z) * (sin(angle) * cos(angle_y) - cos(angle) * sin(angle_y)));
                
                
            //    jacobianMatrx(0, j*2+1) += (float)(-mBodys[i]->mLen * sin(angle_z) * sin(angle_y));
            //    jacobianMatrx(1, j*2+1) += (float)0.0f;
            //    jacobianMatrx(2, j*2+1) += (float)(- mBodys[i]->mLen * sin(angle_z) * cos(angle_y));
            
                                                 
                jacobianMatrx(0, j*2+1) += (float) (mBodys[i]->mLen * sin(angle_z) * (sin(angle) * cos(angle_y) - cos(angle) * sin(angle_y)));
                
                jacobianMatrx(1, j*2+1) += (float)0.0f;
                
                jacobianMatrx(2, j*2+1) += (float) (-mBodys[i]->mLen * sin(angle_z) * (cos(angle) * cos(angle_y) + sin(angle) * sin(angle_y)));
                
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



void IK::calculateAngles(VectorXf dxdydz, float angle) {
    
    VectorXf dAngles = VectorXf::Zero(mBodys.size() * 2, 1);
    
    
    
    dAngles = jacobianInverse(angle) * dxdydz;
    
    
    
    std::cout <<"jacobianInverse():" <<  jacobianInverse(angle) << "\n";
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
    
    
    updateCurPosition(angle);
     
    
    
}

bool IK::getGoal() {
    
    std::cout <<"fabs(mCurPosition(0) - mGoalPostion(0)):" << fabs(mCurPosition(0) - mGoalPostion(0)) << "\n";
    
    std::cout <<"fabs(mCurPosition(1) - mGoalPostion(1)):" << fabs(mCurPosition(1) - mGoalPostion(1)) << "\n";
    
    
    std::cout <<"mCurPosition:" << mCurPosition(0) << "," << mCurPosition(1) << "," << mCurPosition(2) << "\n";
    
    
    if (fabs(mCurPosition(0) - mGoalPostion(0)) <= 0.1f && fabs(mCurPosition(1) - mGoalPostion(1)) <= 0.1f && fabs(mCurPosition(2) - mGoalPostion(2)) <= 0.1f) {
        
        std::cout << "true \n";
        
        return true;
    }
    
    std::cout << "false \n";
    return false;
}


float IK::rotateBody() {
    
    Vector3f directionGoal;
    directionGoal(0) = mGoalPostion(0);
    directionGoal(1) = 0;
    directionGoal(2) = mGoalPostion(2);
    
    float cosAngle = directionGoal.dot(headDirction);   // ignore |mGoalPosition| and |mCurposition| since we only want the sign of cosAngle
        
    cosAngle = cosAngle * 1.0 / (sqrt(pow(directionGoal(0),2) + pow(directionGoal(1),2) + pow(directionGoal(2),2))  * sqrt(pow(headDirction(0),2) + pow(headDirction(1),2) + pow(headDirction(2),2)));
    
        
    float angle = acos(cosAngle);
        
    Vector3f directionNew;
        
    directionNew(0) = - 1 * sin(angle);
    directionNew(1) = headDirction(1);
    directionNew(2) = - 1 * cos(angle);
    
    
    
    std::cout << "directionNew1: " << directionNew <<"\n";
    
    std::cout << "directionGoal: " << directionNew <<"\n";
    
    float cosAngleNew = mGoalPostion.dot(directionNew);
    
    cosAngleNew = cosAngleNew * 1.0 / (sqrt(pow(mGoalPostion(0),2) + pow(mGoalPostion(1),2) + pow(mGoalPostion(2),2))  * sqrt(pow(directionNew(0),2) + pow(directionNew(1),2) + pow(directionNew(2),2)));
    
    
    if (cosAngleNew >-0.5  && cosAngleNew < 0.5) {
        
        // update direction
        
        std::cout << "need to rotate body 1 \n";
        std::cout << "Angle: " << angle << "\n";
        
        return angle;
    }
    
    angle *= -1;
    
    std::cout << "need to rotate body 2 \n";
    std::cout << "Angle: " << angle << "\n";
    
    return angle;
}



void IK::updateHeadDir(float angle) {
    
    //update head direction
    headDirction(0) = - 1 * sin(angle);
    headDirction(2) = - 1 * cos(angle);
    
}

void IK::updateArmPosition() {
    
    // update arm start position
    
    Vector2f newPelDir;
    newPelDir(0) = headDirction(0);
    newPelDir(1) = headDirction(2);
    
    Matrix2f rotateMatrix;
    rotateMatrix << 0.0f, -1.0f, 1.0f, 0.0f;
    
    Vector2f newArmDir = rotateMatrix * newPelDir;
    
    mArmStartPostion(0) = newArmDir(0) * 0.6 + mPelPostion(0);
    mArmStartPostion(1) = mPelPostion(1) + 1.2;
    mArmStartPostion(2) = newArmDir(1) * 0.6 + mPelPostion(2);
    
    std::cout <<"new mArmStartPostion:" << mArmStartPostion(0) << "," << mArmStartPostion(1) << "," << mArmStartPostion(2) << "\n";
    
    std::cout << "finish update \n";
    
}


void IK :: updateCurPosition(float angle) {
    
    float angle_z = 0;
    float angle_y = 0;
    
    mCurPosition(0) = mArmStartPostion(0);
    mCurPosition(1) = mArmStartPostion(1);
    mCurPosition(2) = mArmStartPostion(2);
    
    for (int i = 0; i < mBodys.size(); i++) {
        
        angle_z += mBodys[i]->mAngles[0];
        angle_y += mBodys[i]->mAngles[1];
        
      //  mCurPosition(0) += mBodys[i]->mLen * sin(angle_z) * cos(angle_y);
      //  mCurPosition(1) -= mBodys[i]->mLen * cos(angle_z);
      //  mCurPosition(2) -= mBodys[i]->mLen * sin(angle_z) * sin(angle_y);
        
        
        mCurPosition(0) += (cos(angle) * mBodys[i]->mLen * sin(angle_z) * cos(angle_y) + sin(angle) * mBodys[i]->mLen * sin(angle_z) * sin(angle_y));
        mCurPosition(1) -= mBodys[i]->mLen * cos(angle_z);
        mCurPosition(2) += (sin(angle) * mBodys[i]->mLen * sin(angle_z) * cos(angle_y) - cos(angle) * mBodys[i]->mLen * sin(angle_z) * sin(angle_y));
    }
  
    
    std::cout << "new mCurPosition: " << mCurPosition << "\n";
    
}

void IK :: updateCurPositionInitial() {
    
    mCurPosition(0) = mArmStartPostion(0);
    mCurPosition(1) = mArmStartPostion(1) - 1.5 - 0.18;
    mCurPosition(2) = mArmStartPostion(2);
    
    std::cout << "new intial mCurPosition: " << mCurPosition << "\n";
    
}

