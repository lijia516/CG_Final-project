#include "curveevaluator.h"

float CurveEvaluator::s_fFlatnessEpsilon = 0.00001f;
int CurveEvaluator::s_iSegCount = 16;
bool CurveEvaluator::s_AddNewPt = false;
bool CurveEvaluator::s_SubCon = false;
bool CurveEvaluator::s_DeCaste = false;

CurveEvaluator::~CurveEvaluator(void)
{
}
