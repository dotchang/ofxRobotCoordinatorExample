#include "testApp.h"
#include "aiScene.h"

#define _USE_HIRO
#ifdef _USE_HIRO
#include "HIRONXGUIControler.h"
HIRONXGUIControler hiro;
#endif

//#define _USE_PA10
#ifdef _USE_PA10
#include "PA10GUIControler.h"
PA10GUIControler pa10;
#endif

//#define _USE_GR001
#ifdef _USE_GR001
#include "GR001GUIControler.h"
GR001GUIControler gr001;
#endif

//--------------------------------------------------------------
void testApp::setup(){
	ofSetLogLevel(OF_LOG_VERBOSE);
	ofBackground(50, 0);
	ofSetWindowShape(1280,960);
	
	ofMatrix4x4 origin;
	origin.makeIdentityMatrix();

#ifdef _USE_HIRO
	hiro.setup();
	hiro.model.setPosition(0,0,0);
	model.push_back((AbstractRobotModel*)&hiro);
	origin = hiro.model.getModelMatrix()*hiro.model.getMeshHelper(0).matrix;
	target = (AbstractRobotModel*)&hiro;
#endif

	ofMatrix4x4 pos;
#ifdef _USE_PA10
	pa10.setup();
	target = (AbstractRobotModel*)&pa10;
	pos.setTranslation(-1,1,0);
	pos = pos*origin; 
	pa10.model.setPosition(pos.getTranslation().x,pos.getTranslation().y,pos.getTranslation().z);
	pa10.gui.setPosition(800,20);
	model.push_back((AbstractRobotModel*)&pa10);
#endif

#ifdef _USE_GR001
	gr001.setup();
	target = (AbstractRobotModel*)&gr001;
	pos.setTranslation(-0.5,-0.5,0);
	pos = pos*origin; 
	gr001.model.setPosition(pos.getTranslation().x,pos.getTranslation().y,pos.getTranslation().z);
	gr001.gui.setPosition(800,10);
	model.push_back((AbstractRobotModel*)&gr001);
#endif

	ofEnableBlendMode(OF_BLENDMODE_ALPHA);

	ofEnableDepthTest();

	glShadeModel(GL_SMOOTH); //some model / light stuff
	light.enable();
	ofEnableSeparateSpecularLight();

	cam.resize(3);
	for(int i=0; i<cam.size(); i++){
		cam[i].setFov(80);
		cam[i].setScale(0.001);
		cam[i].setPosition((float)ofGetWidth() * -0.5, (float)ofGetHeight() * -0.5 , 0);
		cam[i].setTarget(target->getModel()->getPosition());
		cam[i].lookAt(target->getModel()->getPosition(),ofVec3f(0,-1,0));
		cam[i].setDistance(1.2);
	}

	viewpoint = 0;
}

//--------------------------------------------------------------
void testApp::update(){
	for(int i=0; i<model.size(); i++){
		ofScale(1.0,1.0,1.0);
		model[i]->update();
	}
	target->getCameraPosition(viewpoint,cam[viewpoint]);
}

//--------------------------------------------------------------
void testApp::draw(){
	ofSetColor(255);

	cam[viewpoint].begin();
	for(int i=0; i<model.size(); i++){
		model[i]->draw();
	}
	cam[viewpoint].end();

	for(int i=0; i<model.size(); i++){
		model[i]->draw_gui();
	}
}

//--------------------------------------------------------------
void testApp::keyPressed(int key){
	switch(key){
	case 257: // F1
#ifdef _USE_HIRO
		target = (AbstractRobotModel*)&hiro;
#endif
		break;
	case 258: // F2
#ifdef _USE_PA10
		target = (AbstractRobotModel*)&pa10;
#endif
		break;
	case 259: // F3
#ifdef _USE_GR001
		target = (AbstractRobotModel*)&gr001;
#endif
		break;
	case '0': viewpoint = 0; break; // Normal View
	case '1': viewpoint = 1; break; // Left Camera View
	case '2': viewpoint = 2; break; // Right Camera View
	default: break;
	}
	target->keyPressed(key);
}

//--------------------------------------------------------------
void testApp::keyReleased(int key){
	//
}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button){
	for(int i=0; i<model.size(); i++){
		model[i]->mouseDragged(x,y,button,cam[viewpoint]);
	}
}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button){
	for(int i=0; i<model.size(); i++){
		model[i]->mousePressed(x,y,button,cam[viewpoint]);
	}
}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button){
	cam[viewpoint].enableMouseInput();
}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void testApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void testApp::dragEvent(ofDragInfo dragInfo){

}
