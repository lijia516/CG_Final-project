#ifndef INCLUDED_BSPLINE_CURVE_EVALUATOR_H
#define INCLUDED_BSPLINE_CURVE_EVALUATOR_H

#pragma warning(disable : 4786)  

#include "curveevaluator.h"

//using namespace std;

class BezierCurveEvaluator : public CurveEvaluator
{
public:
	void evaluateCurve(const std::vector<Point>& ptvCtrlPts, 
		std::vector<Point>& ptvEvaluatedCurvePts, 
		const float& fAniLength, 
		const bool& bWrap) const;
    
    void drawBezier( Point p0,  Point p1, Point p2, Point p3, std::vector<Point>& ptvEvaluatedCurvePts, float fAniLength) const;
    
    int parameter(int n, int k) const;
    
};

#endif
