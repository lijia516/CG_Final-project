#include "catmullromcurveevaluator.h"
#include <cassert>
#include "math.h"


#define M11	 0.0
#define M12	 1.0
#define M13	 0.0
#define M14	 0.0
#define M21	-0.5
#define M22	 0.0
#define M23	 0.5
#define M24	 0.0
#define M31	 1.0
#define M32	-2.5
#define M33	 2.0
#define M34	-0.5
#define M41	-0.5
#define M42	 1.5
#define M43	-1.5
#define M44	 0.5


void CatmullromCurveEvaluator::evaluateCurve(const std::vector<Point>& ptvCtrlPts,
                                         std::vector<Point>& ptvEvaluatedCurvePts,
                                         const float& fAniLength,
                                         const bool& bWrap) const
{
    if (s_AddNewPt) return; 
    
    int iCtrlPtCount = ptvCtrlPts.size();
    ptvEvaluatedCurvePts.assign(ptvCtrlPts.begin(), ptvCtrlPts.end());
    ptvEvaluatedCurvePts.clear();
    
  
    
    if (bWrap) {
        
        
        for(int i = 0; i < iCtrlPtCount; i++){
            
            
            int p0 = (i - 1) % iCtrlPtCount ;
            int p1 = i % iCtrlPtCount;
            int p2 = (i + 1) % iCtrlPtCount;
            int p3 = (i + 2) % iCtrlPtCount;
            
            
            for(float n=0; n < s_iSegCount; n++){
                float u = ((float)n)/((float)s_iSegCount-1);
    
                double y1,y2,y3,y4;
                
                y1 = M12*ptvCtrlPts[p1].y;
                y2 = M21*ptvCtrlPts[p0].y + M23*ptvCtrlPts[p2].y;
                y3 = M31*ptvCtrlPts[p0].y + M32*ptvCtrlPts[p1].y + M33*ptvCtrlPts[p2].y + M34*ptvCtrlPts[p3].y;
                y4 = M41*ptvCtrlPts[p0].y + M42*ptvCtrlPts[p1].y + M43*ptvCtrlPts[p2].y + M44*ptvCtrlPts[p3].y;
                
                float y = (((y4*u + y3)*u +y2)*u + y1);
                
                
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
        
        for(int i=0;i<iCtrlPtCount;i++){
            newPtvCtrlPts.push_back(ptvCtrlPts[i]);
        }
        if(iCtrlPtCount>1){
            newPtvCtrlPts.push_back(ptvCtrlPts[iCtrlPtCount-1]);
        }
        
        int i = 1;
        for(; i < newPtvCtrlPts.size()-2; i++){
            
            for(float n=0; n < s_iSegCount; n++){
                float u = ((float)n)/((float)s_iSegCount-1);
                
                int p0 = i - 1;
                int p1 = i;
                int p2 = i + 1;
                int p3 = i + 2;
                
                double y1,y2,y3,y4;
                
                y1 = M12*newPtvCtrlPts[p1].y;
                y2 = M21*newPtvCtrlPts[p0].y + M23*newPtvCtrlPts[p2].y;
                y3 = M31*newPtvCtrlPts[p0].y + M32*newPtvCtrlPts[p1].y + M33*newPtvCtrlPts[p2].y + M34*newPtvCtrlPts[p3].y;
                y4 = M41*newPtvCtrlPts[p0].y + M42*newPtvCtrlPts[p1].y + M43*newPtvCtrlPts[p2].y + M44*newPtvCtrlPts[p3].y;
                
                float y = (((y4*u + y3)*u +y2)*u + y1);
                
                
                double x1,x2,x3,x4;
                
                x1 = M12*newPtvCtrlPts[p1].x;
                x2 = M21*newPtvCtrlPts[p0].x + M23*newPtvCtrlPts[p2].x;
                x3 = M31*newPtvCtrlPts[p0].x + M32*newPtvCtrlPts[p1].x + M33*newPtvCtrlPts[p2].x + M34*newPtvCtrlPts[p3].x;
                x4 = M41*newPtvCtrlPts[p0].x + M42*newPtvCtrlPts[p1].x + M43*newPtvCtrlPts[p2].x + M44*newPtvCtrlPts[p3].x;
                
                float x = (((x4*u + x3)*u +x2)*u + x1);
                
                ptvEvaluatedCurvePts.push_back(Point(x,y));
            }
        }
        
        // start
        ptvEvaluatedCurvePts.push_back(Point(0, ptvCtrlPts[0].y));
        // end
        ptvEvaluatedCurvePts.push_back(Point(fAniLength, ptvCtrlPts[iCtrlPtCount-1].y));
    
    }
}
