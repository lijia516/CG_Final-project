#include "bsplinescurveevaluator.h"
#include <cassert>
#include "math.h"





void BsplinesCurveEvaluator::evaluateCurve(const std::vector<Point>& ptvCtrlPts,
                                         std::vector<Point>& ptvEvaluatedCurvePts,
                                         const float& fAniLength,
                                         const bool& bWrap) const
{
    
    if (s_AddNewPt) return; 
    
    int iCtrlPtCount = ptvCtrlPts.size();
    ptvEvaluatedCurvePts.assign(ptvCtrlPts.begin(), ptvCtrlPts.end());
    ptvEvaluatedCurvePts.clear();
    
    float x = 0.0;
    float y1;
    
    // Bezier
    
    if (bWrap) {
        
        for (int i = 0; i < iCtrlPtCount; i++) {
            
            std::cout << "i" << i << std::endl;
            
            int p0 = (i) % iCtrlPtCount;
            int p1 = (i + 1) % iCtrlPtCount;
            int p2 = (i + 2) % iCtrlPtCount;
            int p3 = (i + 3) % iCtrlPtCount;
            
        
            for(float n=0; n < s_iSegCount; n++){
                
                float u = ((float)n)/((float)s_iSegCount-1);
                
                float y = (-1.0 / 6.0 * pow(u, 3) + 1.0 / 2.0 * pow(u, 2) -  1.0 / 2.0 * u + 1.0 / 6.0) * ptvCtrlPts[p0].y + \
                    ( 1.0 / 2.0 * pow(u, 3) -  pow(u, 2) + 2.0 / 3.0) * ptvCtrlPts[p1].y + \
                    (-1.0 / 2.0 * pow(u, 3) + 1.0 / 2.0 * pow(u, 2) +  1.0 / 2.0 * u + 1.0 / 6.0) * ptvCtrlPts[p2].y + \
                    (1.0 / 6.0 * pow(u, 3) ) * ptvCtrlPts[p3].y;
                
                
                float len = ptvCtrlPts[p2].x - ptvCtrlPts[p1].x;
                
                if (len < 0) len += fAniLength;
                
                float x = ptvCtrlPts[p1].x + u * len;
                
                if (x > fAniLength)
                    x = x - fAniLength;
                
                ptvEvaluatedCurvePts.push_back(Point(x,y));
            }
            
        }
        
    } else {
        
    
        std::vector<Point> newPtvCtrlPts;
        
        newPtvCtrlPts.push_back(ptvCtrlPts[0]);
        newPtvCtrlPts.push_back(ptvCtrlPts[0]);
        
        for(int i=0;i<iCtrlPtCount;i++){
            newPtvCtrlPts.push_back(ptvCtrlPts[i]);
        }
        if(iCtrlPtCount>1){
            newPtvCtrlPts.push_back(ptvCtrlPts[iCtrlPtCount-1]);
            newPtvCtrlPts.push_back(ptvCtrlPts[iCtrlPtCount-1]);
        }
        
        int i = 0;
        for(; i < newPtvCtrlPts.size()-3; i++){
            
            for(float n=0; n < s_iSegCount; n++){
                float u = ((float)n)/((float)s_iSegCount-1);
                float x=0.0;
                float y=0.0;
                
                x = (-1.0 / 6.0 * pow(u, 3) + 1.0 / 2.0 * pow(u, 2) -  1.0 / 2.0 * u + 1.0 / 6.0) * newPtvCtrlPts[i].x + \
                    ( 1.0 / 2.0 * pow(u, 3) -  pow(u, 2) + 2.0 / 3.0) * newPtvCtrlPts[i + 1].x + \
                    (-1.0 / 2.0 * pow(u, 3) + 1.0 / 2.0 * pow(u, 2) +  1.0 / 2.0 * u + 1.0 / 6.0) * newPtvCtrlPts[i + 2].x + \
                    (1.0 / 6.0 * pow(u, 3) ) * newPtvCtrlPts[i + 3].x;
                
                y = (-1.0 / 6.0 * pow(u, 3) + 1.0 / 2.0 * pow(u, 2) -  1.0 / 2.0 * u + 1.0 / 6.0) * newPtvCtrlPts[i].y + \
                    ( 1.0 / 2.0 * pow(u, 3) -  pow(u, 2) + 2.0 / 3.0) * newPtvCtrlPts[i + 1].y + \
                    (-1.0 / 2.0 * pow(u, 3) + 1.0 / 2.0 * pow(u, 2) +  1.0 / 2.0 * u + 1.0 / 6.0) * newPtvCtrlPts[i + 2].y + \
                    (1.0 / 6.0 * pow(u, 3) ) * newPtvCtrlPts[i + 3].y;
                
                
                ptvEvaluatedCurvePts.push_back(Point(x,y));
            }
        }
        
        // start
        ptvEvaluatedCurvePts.push_back(Point(0, ptvCtrlPts[0].y));
        // end
        ptvEvaluatedCurvePts.push_back(Point(fAniLength, ptvCtrlPts[iCtrlPtCount-1].y));
    
    }
}
