#ifndef _ABSTRACT_ROBOT_MODEL_H
#define _ABSTRACT_ROBOT_MODEL_H

#include "ofEasyCam.h"
class ofxAssimpModelLoader;

class AbstractRobotModel
{
public:
	virtual void setup() = 0;
	virtual void update() = 0;
	virtual void draw() = 0;
	virtual void draw_gui() = 0;

	virtual void keyPressed(int key) = 0;
	virtual void mouseDragged(int x, int y, int button, ofEasyCam& cam) = 0;
	virtual void mousePressed(int x, int y, int button, ofEasyCam& cam) = 0;

	virtual int getCameraPosition(int id, ofEasyCam& cam) = 0;
	virtual ofxAssimpModelLoader* getModel() = 0;
};

#endif 