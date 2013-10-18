#include "IKFastUtil.h"
#define _USE_MATH_DEFINES
#include <math.h>

int IKRangeCheck(std::vector<std::vector<IkReal> >& sol, std::vector<ofParameter<float>>& sol_range)
{
	vector<vector<IkReal> >::iterator it = sol.begin();
	while(it != sol.end()){
		bool check = false;
		for(int i=0; i<it->size(); i++){
			if((sol_range[i].getMin() >= it->at(i)*180/M_PI) || (it->at(i)*180/M_PI >= sol_range[i].getMax())){
				check = true;
				break;
			}
		}
		if(check) it = sol.erase(it);
		else it++;
	}
	return sol.size();
}

int IKSuitable(std::vector<std::vector<IkReal> >& sol, std::vector<ofParameter<float>>& sol_range)
{
	IkReal min_dist = DBL_MAX;
	int min_idx = -1;
	for(int i=0; i<sol.size(); i++){
		IkReal tmp_dist = -1;
		for(int j=0; j<sol[i].size(); j++){
			tmp_dist = std::max(fabs(sol_range[j].get()-sol[i][j]*180.0/M_PI), tmp_dist);
		}
		if(tmp_dist<min_dist){
			min_dist = tmp_dist;
			min_idx = i;
		}
	}
	if(min_idx>=0){
		IkReal tmp;
		for(int j=0; j<sol[min_idx].size(); j++){
			tmp = sol[0][j];
			sol[0][j] = sol[min_idx][j];
			sol[min_idx][j] = tmp;
		}
	}
	return sol.size();
}