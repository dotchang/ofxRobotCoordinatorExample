#ifndef _HIRO_GUI_CONTROLER_H
#define _HIRO_GUI_CONTROLER_H

#include "AbstractRobotModel.h"
#include "ofxAssimpModelLoader.h"
#include "ofxGui.h"
#include <map>
#include "XmlRpc.h"

class HIRONXGUIControler : AbstractRobotModel
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
<<<<<<< HEAD
<<<<<<< HEAD
	int getCameraPosition(int id, ofEasyCam& cam);
=======
>>>>>>> 4df17d0b453847fd483ef940a384cf5e51645636
=======
>>>>>>> 4df17d0b453847fd483ef940a384cf5e51645636
	ofxAssimpModelLoader* getModel(){ return &model; }

	ofxAssimpModelLoader model;
	std::map<std::string, aiMatrix4x4> TransformationOrigin;
	std::vector<int> nearestIndex;
	ofParameter<float> chest, pan, tilt;
	ofParameter<float> rarm[6], larm[6];

	ofxPanel gui;

	ofParameter<string> target;
	ofParameter<bool> coords;
	ofParameter<ofVec3f> xyz, rpy;
	ofxPanel ik;

	ofParameter<bool> rpc_get_angles;
	ofParameter<bool> rpc_set_angles;
	XmlRpc::XmlRpcClient *c;
};

#endif