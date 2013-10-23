#ifndef _PA10_GUI_CONTROLER_H
#define _PA10_GUI_CONTROLER_H

#include "AbstractRobotModel.h"
#include "ofxAssimpModelLoader.h"
#include "ofxGui.h"
#include <map>

class PA10GUIControler : AbstractRobotModel
{
public:
	void setup();
	void update();
	void draw();
	void draw_gui();

	void keyPressed(int key);
	void mouseDragged(int x, int y, int button, ofEasyCam& cam);
	void mousePressed(int x, int y, int button, ofEasyCam& cam);

	int inverseKinematics();
	int getCameraPosition(int id, ofEasyCam& cam);

	ofxAssimpModelLoader model, dummy;
	std::map<std::string, aiMatrix4x4> TransformationOrigin;
	std::vector<int> nearestIndex;

	ofxPanel gui;
	ofParameter<float> arm[7];

	ofParameter<string> ik_target;
	ofParameter<bool> coords;
	ofParameter<ofVec3f> xyz, rpy;
	ofxPanel ik;
};

#endif