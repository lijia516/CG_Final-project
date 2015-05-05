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
    
    float x = (mGoalPostion(0) - mCurPosition(0)) * 1.0 / 28;
    float y = (mGoalPostion(1) - mCurPosition(1)) * 1.0 / 28;
    
    
    std::cout <<"goal:" << mGoalPostion(0) << "," << mGoalPostion(1) << "\n";
    std::cout <<"cur:" << mCurPosition(0) << "," << mCurPosition(1) << "\n";
    std::cout <<"dx, dy:" << x << "," << y << "\n";
    
    float dt = 1;
    float mStep = 0.5;

    
    while (!getGoal() && dt < 15) {
        
        dt += 0.5;
        
        std::cout <<"dt: "<< dt <<"\n";
        
        VectorXf dxdy = VectorXf::Zero(2, 1);
        dxdy(0) = x;
        dxdy(1) = y;
        
        calculateAngles(dxdy);
        app->GetUI()->m_pwndGraphWidget->AddCtrlPt(23, dt, mBodys[0]->mAngle);
     //   app->GetUI()->m_pwndGraphWidget->AddCtrlPt(32, dt, mBodys[1]->mAngle);
        
        std::cout <<"angle1, angle2:" << mBodys[0]->mAngle << "\n";
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
                
                jacobianMatrx(0, j) += (-mBodys[i]->mLen * cos(PI - angle));
                
                jacobianMatrx(1, j) += (mBodys[i]->mLen * sin(PI - angle));
                
            }
        }
    }
    
    std::cout <<"cos:" <<  cos(PI) << "\n";
    std::cout <<"Jacobian:" <<  jacobianMatrx << "\n";
    std::cout <<"Jacobian transpose:" <<  jacobianMatrx.transpose() << "\n";
    std::cout <<"jacobianMatrx * Jacobian transpose:" <<  jacobianMatrx * jacobianMatrx.transpose() << "\n";
    
    Matrix2f jacobianMatrxInverse1 = jacobianMatrx * (jacobianMatrx.transpose());
    
    std::cout <<"jacobianMatrxInverse1:" << jacobianMatrxInverse1 <<"\n";
    std::cout <<"jacobianMatrxInverse1:" << jacobianMatrxInverse1.inverse() <<"\n";
    
    Matrix2f A;
    
    float m00 = float(jacobianMatrxInverse1(0,0));
    float m01 = float(jacobianMatrxInverse1(0,1));
    float m10 = float(jacobianMatrxInverse1(1,0));
    float m11 = float(jacobianMatrxInverse1(1,1));
    
    A << m00, m01, m10, m11;
   // A << 0.357918, 0.116232, 0.116232, 0.0377455;
    cout << "Here is the matrix A:\n" << A << endl;
    cout << "The determinant of A is " << A.determinant() << endl;
    cout << "The inverse of A is:\n" << A.inverse() << endl;

    
    Matrix2f jacobianMatrxInverse;
    
    jacobianMatrxInverse << 0.357918, 0.116232, 0.116232, 0.0377455;
    
    
    
    std::cout <<"jacobianMatrxInverse:" << jacobianMatrxInverse <<"\n";
    
    std::cout <<"jacobianMatrxInverse:" << jacobianMatrxInverse.inverse() <<"\n";
    
    return (jacobianMatrx.transpose() * jacobianMatrxInverse);

}



void IK::calculateAngles(VectorXf dxdy) {
    
    VectorXf dAngles = VectorXf::Zero(mBodys.size(), 1);
    
    dAngles = jacobianInverse() * dxdy;
    
    
    std::cout <<"jacobianInverse():" <<  jacobianInverse() (0,0) << "\n";
    std::cout <<"dAngles:" <<  dAngles << "\n";
    
    
    for (int i = 0; i < mBodys.size(); i++) {
        mBodys[i]->mAngle += dAngles(i);
    }
}

bool IK::getGoal() {
    
    
    if (fabs(mCurPosition(0) - mGoalPostion(0)) <= 0.5f && fabs(mCurPosition(1) - mGoalPostion(1)) <= 0.5f) {
        
        std::cout << "true \n";
        
        return true;
    }
    
    std::cout << "false \n";
    return false;
}

