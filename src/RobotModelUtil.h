#ifndef _ROBOT_MODEL_UTIL_H
#define _ROBOT_MODEL_UTIL_H

#include "aiScene.h"
#include "ofParameter.h"
#include <map>

void printAiMatrix4x4(aiMatrix4x4 m);
void printNode(const aiNode* node);
void searchPrintNode(const aiNode* node);
void searchRegistOrigin(aiNode* node, map<string, aiMatrix4x4>& it);
inline float adjust_value(ofParameter<float> p, float value){
	float v = (p.getMax()-p.getMin())*value +p.getMin();
	if(v<p.getMin()) v = p.getMin();
	else if(p.getMax()<v) v = p.getMax();
	return v;
}

#endif