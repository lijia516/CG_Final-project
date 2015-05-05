#include "beziercurveevaluator.h"
#include <cassert>
#include "math.h"

#define PI 3.14159265
#define LIMIT 0.00002

int BezierCurveEvaluator::parameter( int n,  int k) const
{
    int r = 1;
    if(k > n)
        return 0;
    for( int d = 1; d <= k; d++)
    {
        r *= n--;
        r /= d;
    }
    return r;
}


void BezierCurveEvaluator::drawBezier( Point p0,  Point p1, Point p2, Point p3, std::vector<Point>& ptvEvaluatedCurvePts, float fAniLength) const
{

    
    if (s_SubCon) {
        
        float len0 = sqrt(pow(p1.x - p0.x, 2.0) + pow(p1.y - p0.y, 2.0));
        float len1 = sqrt(pow(p2.x - p1.x, 2.0) + pow(p2.y - p1.y, 2.0));
        float len2 = sqrt(pow(p3.x - p2.x, 2.0) + pow(p3.y - p2.y, 2.0));
        float len3 = sqrt(pow(p0.x - p3.x, 2.0) + pow(p0.y - p3.y, 2.0));
        
        if ((len0 + len1 + len2) * 1.0 / len3 < (1 + LIMIT)) {
            
             p0.x = p0.x > fAniLength ? p0.x - fAniLength : p0.x;
            
            ptvEvaluatedCurvePts.push_back(p0);
            std::cout << "s_SubCon x, y: " << p0.x <<"," << p0.y << std::endl;
            
        } else {
            
            
            float l1_x = (p1.x + p0.x) / 2.0;
            float l1_y = (p1.y + p0.y) / 2.0;
            
            float r2_x = (p3.x + p2.x) / 2.0;
            float r2_y = (p3.y + p2.y) / 2.0;
            
            float m_x = (p2.x + p1.x) / 2.0;
            float m_y = (p2.y + p1.y) / 2.0;
            
            float l2_x = (m_x + l1_x) / 2.0;
            float l2_y = (m_y + l1_y) / 2.0;
            
            float r1_x = (m_x + r2_x) / 2.0;
            float r1_y = (m_y + r2_y) / 2.0;
            
            float r0_x = (l2_x + r1_x) / 2.0;
            float r0_y = (l2_y + r1_y) / 2.0;
            
            drawBezier(p0, Point(l1_x, l1_y), Point(l2_x, l2_y), Point(r0_x, r0_y), ptvEvaluatedCurvePts, fAniLength);
            drawBezier(Point(r0_x, r0_y), Point(r1_x, r1_y), Point(r2_x, r2_y), p3, ptvEvaluatedCurvePts, fAniLength);
        }
        
        
    } else if (s_DeCaste) {
    
        for(float n=0; n < s_iSegCount; n++){
            
            float u = ((float)n)/((float)s_iSegCount - 1);
        
            float q0_x = ( (1 - u) * p0.x + u * p1.x) ;
            float q0_y = ( (1 - u) * p0.y + u * p1.y) ;
        
            float q1_x = ((1 - u) * p1.x + u * p2.x) ;
            float q1_y = ((1 - u) * p1.y + u * p2.y) ;
            
            float q2_x = ((1 - u) * p2.x + u * p3.x) ;
            float q2_y = ((1 - u) * p2.y + u * p3.y) ;
            
        
            float r0_x = ((1 - u) * q0_x + u * q1_x) ;
            float r0_y = ((1 - u) * q0_y + u * q1_y) ;
            
            float r1_x = ((1 - u) * q1_x + u * q2_x) ;
            float r1_y = ((1 - u) * q1_y + u * q2_y) ;
            
            float pu_x = ((1 - u) * r0_x + u * r1_x) ;
            float pu_y = ((1 - u) * r0_y + u * r1_y) ;
            
             pu_x = pu_x > fAniLength ? pu_x - fAniLength : pu_x;
            
            ptvEvaluatedCurvePts.push_back(Point(pu_x, pu_y));
            std::cout << "s_DeCaste x, y: " << pu_x <<"," << pu_y << std::endl;

        }
        
        
    } else {
        
        for(float n=0; n < s_iSegCount; n++){
            
            float u = ((float)n)/((float)s_iSegCount-1);
            
            float factor = parameter(3,0) * pow(u,0) * pow((1-u),3-0);
            float x = factor*(p0.x);
            float y = factor*(p0.y);
            
            factor =  parameter(3,1) * pow(u,1) * pow((1-u),3-1);
            x += factor*(p1.x);
            y += factor*(p1.y);
            
            factor =  parameter(3,2) * pow(u,2) * pow((1-u),3-2);
            x += factor*(p2.x);
            y += factor*(p2.y);
            
            factor =  parameter(3,3) * pow(u,3) * pow((1-u),3-3);
            
            x += factor*(p3.x);
            y += factor*(p3.y);
            
            
            x = x > fAniLength ? x - fAniLength : x;
            
            ptvEvaluatedCurvePts.push_back(Point(x,y));
            std::cout << "normal x, y: " << x <<"," << y << std::endl;
        
        }
    }
}


void BezierCurveEvaluator::evaluateCurve(const std::vector<Point>& ptvCtrlPts,
                                         std::vector<Point>& ptvEvaluatedCurvePts,
                                         const float& fAniLength,
                                         const bool& bWrap) const
{
    if (s_AddNewPt) return;
    
        
    std::cout << "subCon: true " << s_SubCon << std::endl;
    
    int iCtrlPtCount = ptvCtrlPts.size();
    ptvEvaluatedCurvePts.assign(ptvCtrlPts.begin(), ptvCtrlPts.end());
    ptvEvaluatedCurvePts.clear();
    
    float x = 0.0;
    float y1;
    
    if (iCtrlPtCount < 4) {
        ptvEvaluatedCurvePts.assign(ptvCtrlPts.begin(), ptvCtrlPts.end());
        
        if (bWrap) {
            // if wrapping is on, interpolate the y value at xmin and
            // xmax so that the slopes of the lines adjacent to the
            // wraparound are equal.
            
            if ((ptvCtrlPts[0].x + fAniLength) - ptvCtrlPts[iCtrlPtCount - 1].x > 0.0f) {
                y1 = (ptvCtrlPts[0].y * (fAniLength - ptvCtrlPts[iCtrlPtCount - 1].x) +
                      ptvCtrlPts[iCtrlPtCount - 1].y * ptvCtrlPts[0].x) /
                (ptvCtrlPts[0].x + fAniLength - ptvCtrlPts[iCtrlPtCount - 1].x);
            }
            else
                y1 = ptvCtrlPts[0].y;
        }
        else {
            // if wrapping is off, make the first and last segments of
            // the curve horizontal.
            
            y1 = ptvCtrlPts[0].y;
        }
        
        ptvEvaluatedCurvePts.push_back(Point(x, y1));
        
        /// set the endpoint based on the wrap flag.
        float y2;
        x = fAniLength;
        if (bWrap)
            y2 = y1;
        else
            y2 = ptvCtrlPts[iCtrlPtCount - 1].y;
        
        ptvEvaluatedCurvePts.push_back(Point(x, y2));
        
        return;
    }
    
    
    std::vector <float> atanRecod;
    
    // Bezier
    
    int count = -1;
    int i = 0;
    float gap = 1;
    for (; i + 3 < iCtrlPtCount; i += 3){
        
         std::cout << "i: "<< i <<std::endl;
        
        drawBezier( ptvCtrlPts[i],  ptvCtrlPts[i+1], ptvCtrlPts[i+2], ptvCtrlPts[i+3], ptvEvaluatedCurvePts, fAniLength);
    }
    
    
    std::cout << "finish middle: "<< std::endl;
    
    if (bWrap) {
        
        
        if (iCtrlPtCount - i == 3) {
        
            drawBezier(ptvCtrlPts[i],  ptvCtrlPts[i+1], ptvCtrlPts[i+2], Point(ptvCtrlPts[0].x + fAniLength, ptvCtrlPts[0].y), ptvEvaluatedCurvePts, fAniLength);
            
            
        } else {
            
             for (;i < iCtrlPtCount; i++)
                ptvEvaluatedCurvePts.push_back(ptvCtrlPts[i]);
            
            float y1;
            
            if ((ptvCtrlPts[0].x + fAniLength) - ptvCtrlPts[iCtrlPtCount - 1].x > 0.0f) {
                y1 = (ptvCtrlPts[0].y * (fAniLength - ptvCtrlPts[iCtrlPtCount - 1].x) +
                      ptvCtrlPts[iCtrlPtCount - 1].y * ptvCtrlPts[0].x) /
                (ptvCtrlPts[0].x + fAniLength - ptvCtrlPts[iCtrlPtCount - 1].x);
            }
            else
                y1 = ptvCtrlPts[0].y;
            
            ptvEvaluatedCurvePts.push_back(Point(0, y1));
            ptvEvaluatedCurvePts.push_back(Point(fAniLength, y1));
        
            
        }
        
    } else {
        
    
        for (;i < iCtrlPtCount; i++)
            ptvEvaluatedCurvePts.push_back(ptvCtrlPts[i]);
        
        // start
        ptvEvaluatedCurvePts.push_back(Point(0, ptvCtrlPts[0].y));
        // end
        ptvEvaluatedCurvePts.push_back(Point(fAniLength, ptvCtrlPts[iCtrlPtCount-1].y));
    
    }
}
