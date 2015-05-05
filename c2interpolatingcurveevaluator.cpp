#include "c2interpolatingcurveevaluator.h"
#include <cassert>
#include "math.h"
#include "vec.h"

#define M11	 1.0
#define M12	 0.0
#define M13	 0.0
#define M14	 0.0
#define M21	 0.0
#define M22	 0.0
#define M23	 1.0
#define M24	 0.0
#define M31	 -3.0
#define M32	 3.0
#define M33	 -2.0
#define M34	-1.0
#define M41	2.0
#define M42	 -2.0
#define M43	 1.0
#define M44	 1.0


void C2interpolatingCurveEvaluator::evaluateCurve(const std::vector<Point>& ptvCtrlPts,
                                                  std::vector<Point>& ptvEvaluatedCurvePts,
                                                  const float& fAniLength,
                                                  const bool& bWrap) const
{
    if (s_AddNewPt) return;
    
    int iCtrlPtCount = ptvCtrlPts.size();
    ptvEvaluatedCurvePts.clear();
    

    
    std::vector<float> temp1;
    std::vector<float> temp2;
    std::vector<float> newPtvCtrlPts;
    
    newPtvCtrlPts.resize(iCtrlPtCount, 0.0f);
    temp1.resize(iCtrlPtCount, 0.0f);
    temp2.resize(iCtrlPtCount, 0.0f);
    
    temp1[0] = 0.5f;
    for (int i = 1; i < iCtrlPtCount - 1; i++) {
        temp1[i] = 1.0f / (4.0f - temp1[i - 1]);
    }
    temp1[iCtrlPtCount - 1] = 1.0 / (2.0 - temp1[iCtrlPtCount - 2]);
    
    
    temp2[0] = 1.5 * (ptvCtrlPts[1].y - ptvCtrlPts[0].y);
    for (int i = 1; i < iCtrlPtCount - 1; i++) {
        temp2[i] = temp1[i] * (3 * (ptvCtrlPts[i+1].y - ptvCtrlPts[i-1].y) - temp2[i-1]);
    }
    temp2[iCtrlPtCount - 1] = temp1[iCtrlPtCount - 1] * (3 * (ptvCtrlPts[iCtrlPtCount - 1].y - ptvCtrlPts[iCtrlPtCount - 2].y) - temp2[iCtrlPtCount - 1]);
    
    
    newPtvCtrlPts[iCtrlPtCount - 1] = temp2[iCtrlPtCount - 1];
    for (int i = iCtrlPtCount - 2; i >= 0; i--) {
        newPtvCtrlPts[i] = temp2[i] - temp1[i] * newPtvCtrlPts[i+1];
    }
    
    
    int i = 0;
    for(; i < iCtrlPtCount - 1; i++){
        
        int p1 = i;
        int p2 = i + 1;
        
        float c1 = M11 * ptvCtrlPts[p1].y;
        float c2 = M23 * newPtvCtrlPts[p1];
        float c3 = M31 * ptvCtrlPts[p1].y + M32 * ptvCtrlPts[p2].y + M33 * newPtvCtrlPts[p1] + M34 * newPtvCtrlPts[p2];
        float c4 = M41 * ptvCtrlPts[p1].y + M42 * ptvCtrlPts[p2].y + M43 * newPtvCtrlPts[p1] + M44 * newPtvCtrlPts[p2];
        
        for (int n = 0; n < s_iSegCount; n++) {
            
            float u = (float)n / (float) (s_iSegCount - 1);

            float y = c1 + u * c2 + pow(u, 2.0)* c3 + pow(u, 3.0) * c4;
            
            float len = ptvCtrlPts[p2].x - ptvCtrlPts[p1].x;
            float x = ptvCtrlPts[p1].x + u * len;
            
            ptvEvaluatedCurvePts.push_back(Point(x, y));
        }
    }
    
    if (bWrap) {
        
        newPtvCtrlPts.resize(iCtrlPtCount + 1, 0.0f);
        temp1.resize(iCtrlPtCount + 1, 0.0f);
        temp2.resize(iCtrlPtCount + 1, 0.0f);
        
        temp1[iCtrlPtCount - 1] = 1.0f / (4.0f - temp1[iCtrlPtCount - 2]);
        temp1[iCtrlPtCount] = 1.0 / (2.0 - temp1[iCtrlPtCount - 1]);
        
        temp2[iCtrlPtCount - 1] = temp1[iCtrlPtCount - 1] * (3 * (ptvCtrlPts[0].y - ptvCtrlPts[iCtrlPtCount - 2].y) - temp2[iCtrlPtCount - 2]);
        temp2[iCtrlPtCount] = temp1[iCtrlPtCount] * (3 * (ptvCtrlPts[0].y - ptvCtrlPts[iCtrlPtCount - 1].y) - temp2[iCtrlPtCount]);
        
        newPtvCtrlPts[iCtrlPtCount] = temp2[iCtrlPtCount];
        
        
        int p1 = iCtrlPtCount - 1;
        int p2 = 0;
        
        float c1 = M11 * ptvCtrlPts[p1].y;
        float c2 = M23 * newPtvCtrlPts[p1];
        float c3 = M31 * ptvCtrlPts[p1].y + M32 * ptvCtrlPts[p2].y + M33 * newPtvCtrlPts[p1] + M34 * newPtvCtrlPts[p2];
        float c4 = M41 * ptvCtrlPts[p1].y + M42 * ptvCtrlPts[p2].y + M43 * newPtvCtrlPts[p1] + M44 * newPtvCtrlPts[p2];
        
        for (int n = 0; n < s_iSegCount; n++) {
            
            float u = (float)n / (float) (s_iSegCount - 1);
            
            float y = c1 + u * c2 + pow(u, 2.0)* c3 + pow(u, 3.0) * c4;
            
            float len = ptvCtrlPts[p2].x + fAniLength - ptvCtrlPts[p1].x;
            float x = ptvCtrlPts[p1].x + u * len;
            
            x = x > fAniLength ? x - fAniLength : x;
            
            ptvEvaluatedCurvePts.push_back(Point(x, y));
        }
        
        
        
    } else {
        
        // start
        ptvEvaluatedCurvePts.push_back(Point(0, ptvCtrlPts[0].y));
        // end
        ptvEvaluatedCurvePts.push_back(Point(fAniLength, ptvCtrlPts[iCtrlPtCount-1].y));
        
    }
    
}
