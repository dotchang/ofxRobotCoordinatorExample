#ifndef _IK_FAST_UTIL_H
#define _IK_FAST_UTIL_H

#include "ofParameter.h"

typedef double IkReal;

int IKRangeCheck(std::vector<std::vector<IkReal> >& sol, std::vector<ofParameter<float>>& sol_range);
int IKSuitable(std::vector<std::vector<IkReal> >& sol, std::vector<ofParameter<float>>& sol_range);

#endif 