#ifndef INCLUDED_C2INTERPOLATING_CURVE_EVALUATOR_H
#define INCLUDED_C2INTERPOLATING_CURVE_EVALUATOR_H

#pragma warning(disable : 4786)  

#include "curveevaluator.h"
#include "mat.h"
#include "vec.h"
//using namespace std;

class C2interpolatingCurveEvaluator : public CurveEvaluator
{
public:
	void evaluateCurve(const std::vector<Point>& ptvCtrlPts, 
		std::vector<Point>& ptvEvaluatedCurvePts, 
		const float& fAniLength, 
		const bool& bWrap) const;
    
    
    void evaluate(const int p1, const int p2, const float& fAniLength,
                                    const std::vector<Point>& ptvCtrlPts, std::vector<Point>& ptvEvaluatedCurvePts,
                                    const std::vector<float>& deri_pts
                                    )const;
    static Mat4<float> m_basisMatrix;
    
};

#endif
