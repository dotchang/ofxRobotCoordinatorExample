#include "GR001GUIControler.h"
#include "RobotModelUtil.h"

void GR001GUIControler::setup(){
	model.setScaleNomalization(false);
	model.loadModel("GR001b.dae", false);
	searchRegistOrigin(model.getAssimpScene()->mRootNode, TransformationOrigin);
	searchPrintNode(model.getAssimpScene()->mRootNode);

	//waist.set("waist", 0, -180, 180);
	rleg[0].set("rleg0", 0, -150, 30.023);
	rleg[1].set("rleg1", 0, -90.0117, 90.0117);
	rleg[2].set("rleg2", 0, -119.977, 39.9925);
	rleg[3].set("rleg3", 0, 0, 130.004);
	rleg[4].set("rleg4", 0, -96.9964, 59.9887);
	rleg[5].set("rleg5", 0, -44.9772, 90.0117);
	lleg[0].set("lleg0", 0, -30.023, 150);
	lleg[1].set("lleg1", 0, -90.0117, 90.0117);
	lleg[2].set("lleg2", 0, -119.977, 39.9925);
	lleg[3].set("lleg3", 0, -0, 130.004);
	lleg[4].set("lleg4", 0, -94.9964, 59.9887);
	lleg[5].set("lleg5", 0, -90.0117, 44.9772);
	chest.set("chest", 0, 0, 94.9964);
	neck.set("neck", 0, -49.9619, 49.9619);
	rarm[0].set("rarm0", 0, -150, 150);
	rarm[1].set("rarm1", 0, -150, 39.9925);
	rarm[2].set("rarm2", 0, -130.004, 50.0192);
	larm[0].set("larm0", 0, -150, 150);
	larm[1].set("larm1", 0, -39.9925, 150);
	larm[2].set("larm2", 0, -130.004, 50.0192);

	gui.setup("GR001");
	//gui.add(waist);
	for(int i=0; i<6; i++){ gui.add(rleg[i]); }
	for(int i=0; i<6; i++){ gui.add(lleg[i]); }
	gui.add(chest);
	gui.add(neck);
	for(int i=0; i<3; i++){ gui.add(rarm[i]); }
	for(int i=0; i<3; i++){ gui.add(larm[i]); }
}

void GR001GUIControler::update(){
	//model.getAssimpScene()->mRootNode->FindNode("WAIST_LINK")->mTransformation = TransformationOrigin["WAIST_LINK"]*aiMatrix4x4().FromEulerAnglesXYZ(0,0,-waist*3.14/180.0);
	model.getAssimpScene()->mRootNode->FindNode("R_HIP_Y_LINK")->mTransformation = TransformationOrigin["R_HIP_Y_LINK"]*aiMatrix4x4().FromEulerAnglesXYZ(0,0,-rleg[0]*3.14/180.0);
	model.getAssimpScene()->mRootNode->FindNode("R_HIP_R_LINK")->mTransformation = TransformationOrigin["R_HIP_R_LINK"]*aiMatrix4x4().FromEulerAnglesXYZ(-rleg[1]*3.14/180.0,0,0);
	model.getAssimpScene()->mRootNode->FindNode("R_HIP_P_LINK")->mTransformation = TransformationOrigin["R_HIP_P_LINK"]*aiMatrix4x4().FromEulerAnglesXYZ(0,-rleg[2]*3.14/180.0,0);
	model.getAssimpScene()->mRootNode->FindNode("R_KNEE_P_LINK")->mTransformation = TransformationOrigin["R_KNEE_P_LINK"]*aiMatrix4x4().FromEulerAnglesXYZ(0,-rleg[3]*3.14/180.0,0);
	model.getAssimpScene()->mRootNode->FindNode("R_ANKLE_P_LINK")->mTransformation = TransformationOrigin["R_ANKLE_P_LINK"]*aiMatrix4x4().FromEulerAnglesXYZ(0,-rleg[4]*3.14/180.0,0);
	model.getAssimpScene()->mRootNode->FindNode("R_ANKLE_R_LINK")->mTransformation = TransformationOrigin["R_ANKLE_R_LINK"]*aiMatrix4x4().FromEulerAnglesXYZ(-rleg[5]*3.14/180.0,0,0);
	model.getAssimpScene()->mRootNode->FindNode("L_HIP_Y_LINK")->mTransformation = TransformationOrigin["L_HIP_Y_LINK"]*aiMatrix4x4().FromEulerAnglesXYZ(0,0,-lleg[0]*3.14/180.0);
	model.getAssimpScene()->mRootNode->FindNode("L_HIP_R_LINK")->mTransformation = TransformationOrigin["L_HIP_R_LINK"]*aiMatrix4x4().FromEulerAnglesXYZ(-lleg[1]*3.14/180.0,0,0);
	model.getAssimpScene()->mRootNode->FindNode("L_HIP_P_LINK")->mTransformation = TransformationOrigin["L_HIP_P_LINK"]*aiMatrix4x4().FromEulerAnglesXYZ(0,-lleg[2]*3.14/180.0,0);
	model.getAssimpScene()->mRootNode->FindNode("L_KNEE_P_LINK")->mTransformation = TransformationOrigin["L_KNEE_P_LINK"]*aiMatrix4x4().FromEulerAnglesXYZ(0,-lleg[3]*3.14/180.0,0);
	model.getAssimpScene()->mRootNode->FindNode("L_ANKLE_P_LINK")->mTransformation = TransformationOrigin["L_ANKLE_P_LINK"]*aiMatrix4x4().FromEulerAnglesXYZ(0,-lleg[4]*3.14/180.0,0);
	model.getAssimpScene()->mRootNode->FindNode("L_ANKLE_R_LINK")->mTransformation = TransformationOrigin["L_ANKLE_R_LINK"]*aiMatrix4x4().FromEulerAnglesXYZ(-lleg[5]*3.14/180.0,0,0);
	model.getAssimpScene()->mRootNode->FindNode("CHEST_P_LINK")->mTransformation = TransformationOrigin["CHEST_P_LINK"]*aiMatrix4x4().FromEulerAnglesXYZ(0,-chest*3.14/180.0,0);
	model.getAssimpScene()->mRootNode->FindNode("NECK_Y_LINK")->mTransformation = TransformationOrigin["NECK_Y_LINK"]*aiMatrix4x4().FromEulerAnglesXYZ(0,0,-neck*3.14/180.0);
	model.getAssimpScene()->mRootNode->FindNode("R_SHOULDER_P_LINK")->mTransformation = TransformationOrigin["R_SHOULDER_P_LINK"]*aiMatrix4x4().FromEulerAnglesXYZ(0,-rarm[0]*3.14/180.0,0);
	model.getAssimpScene()->mRootNode->FindNode("R_SHOULDER_R_LINK")->mTransformation = TransformationOrigin["R_SHOULDER_R_LINK"]*aiMatrix4x4().FromEulerAnglesXYZ(-rarm[1]*3.14/180.0,0,0);
	model.getAssimpScene()->mRootNode->FindNode("R_ELBOW_P_LINK")->mTransformation = TransformationOrigin["R_ELBOW_P_LINK"]*aiMatrix4x4().FromEulerAnglesXYZ(0,-rarm[2]*3.14/180.0,0);
	model.getAssimpScene()->mRootNode->FindNode("L_SHOULDER_P_LINK")->mTransformation = TransformationOrigin["L_SHOULDER_P_LINK"]*aiMatrix4x4().FromEulerAnglesXYZ(0,-larm[0]*3.14/180.0,0);
	model.getAssimpScene()->mRootNode->FindNode("L_SHOULDER_R_LINK")->mTransformation = TransformationOrigin["L_SHOULDER_R_LINK"]*aiMatrix4x4().FromEulerAnglesXYZ(-larm[1]*3.14/180.0,0,0);
	model.getAssimpScene()->mRootNode->FindNode("L_ELBOW_P_LINK")->mTransformation = TransformationOrigin["L_ELBOW_P_LINK"]*aiMatrix4x4().FromEulerAnglesXYZ(0,-larm[2]*3.14/180.0,0);
	model.update();
}

void GR001GUIControler::draw(){
	//model.drawFaces();

	for(int i=0; i<model.getMeshCount(); i++){
		ofPushMatrix();
		ofMultMatrix(model.getModelMatrix());
		ofxAssimpMeshHelper & meshHelper = model.getMeshHelper(i);
		ofMultMatrix(meshHelper.matrix);
		ofSetColor( meshHelper.material.getDiffuseColor() );
		model.getMesh(i).draw();
		ofPopMatrix();
	}
	ofDisableDepthTest();
	ofDisableLighting();
	for(int i=0; i<model.getMeshCount(); i++){
		ofPushMatrix();
		ofMultMatrix(model.getModelMatrix());
		ofxAssimpMeshHelper & meshHelper = model.getMeshHelper(i);
		ofMultMatrix(meshHelper.matrix);
		ofDrawAxis(0.01);
		ofPopMatrix();
	}
	ofEnableLighting();
	ofEnableDepthTest();
	ofScale(1.0,1.0,1.0);
}

void GR001GUIControler::draw_gui(){
	ofDisableDepthTest();
	ofDisableLighting();
	gui.draw();
	ofEnableLighting();
	ofEnableDepthTest();
}

void GR001GUIControler::mouseDragged(int x, int y, int button, ofEasyCam& cam)
{
	if(button == 1 && !nearestIndex.empty()){
		cam.disableMouseInput();	
		float value_x = x / (float)ofGetWidth();
		float value_y = y / (float)ofGetHeight();

		for(int i=0; i<(int)nearestIndex.size(); i++){
			aiString name = model.getMeshHelper(nearestIndex[i]).mesh->mName;
			//if(name == aiString("WAIST_LINK")){ waist = adjust_value(waist, value_x); }
			if(name == aiString("R_HIP_Y_LINK")){ rleg[0] = adjust_value(rleg[0], value_x); }
			else if(name == aiString("R_HIP_R_LINK")){ rleg[1] = adjust_value(rleg[1], value_x); }
			else if(name == aiString("R_HIP_P_LINK")){ rleg[2] = adjust_value(rleg[2], value_y); }
			else if(name == aiString("R_KNEE_P_LINK")){ rleg[3] = adjust_value(rleg[3], value_y); }
			else if(name == aiString("R_ANKLE_P_LINK")){ rleg[4] = adjust_value(rleg[4], value_y); }
			else if(name == aiString("R_ANKLE_R_LINK")){ rleg[5] = adjust_value(rleg[5], value_x); }
			else if(name == aiString("L_HIP_Y_LINK")){ lleg[0] = adjust_value(lleg[0], value_x); }
			else if(name == aiString("L_HIP_R_LINK")){ lleg[1] = adjust_value(lleg[1], value_x); }
			else if(name == aiString("L_HIP_P_LINK")){ lleg[2] = adjust_value(lleg[2], value_y); }
			else if(name == aiString("L_KNEE_P_LINK")){ lleg[3] = adjust_value(lleg[3], value_y); }
			else if(name == aiString("L_ANKLE_P_LINK")){ lleg[4] = adjust_value(lleg[4], value_y); }
			else if(name == aiString("L_ANKLE_R_LINK")){ lleg[5] = adjust_value(lleg[5], value_x); }
			else if(name == aiString("CHEST_P_LINK")){ chest = adjust_value(chest, value_y); }
			else if(name == aiString("NECK_Y_LINK")){ neck = adjust_value(neck, value_x); }
			else if(name == aiString("R_SHOULDER_P_LINK")){ rarm[0] = adjust_value(rarm[0], value_x); }
			else if(name == aiString("R_SHOULDER_R_LINK")){ rarm[1] = adjust_value(rarm[1], value_x); }
			else if(name == aiString("R_ELBOW_P_LINK")){ rarm[2] = adjust_value(rarm[2], value_x); }
			else if(name == aiString("L_SHOULDER_P_LINK")){ larm[0] = adjust_value(larm[0], value_x); }
			else if(name == aiString("L_SHOULDER_R_LINK")){ larm[1] = adjust_value(larm[1], value_x); }
			else if(name == aiString("L_ELBOW_P_LINK")){ larm[2] = adjust_value(larm[2], value_x); }
		}
	}
}

int GR001GUIControler::getCameraPosition(int id, ofEasyCam& cam)
{
	switch(id){
	case 0:
	case 1:
	case 2: // Normal
		cam.setTarget(model.getPosition());
		cam.lookAt(model.getPosition(),ofVec3f(0,-1,0));
		break;
	default:
		return -1;
	}
	return 0;
}

void GR001GUIControler::mousePressed(int x, int y, int button, ofEasyCam& cam)
{
	if(button==1){
		int n = model.getMeshCount();
		float nearestDistance = 0;
		ofVec2f nearestVertex;
		nearestIndex.clear();
		ofVec2f mouse(x,y);
		for(int i=0; i<n; i++){
			ofMatrix4x4 m = model.getMeshHelper(i).matrix * model.getModelMatrix();
			ofVec3f cur = cam.worldToScreen(m.getTranslation());
			float distance = cur.distance(mouse);
			if(i == 0|| distance < nearestDistance){
				nearestDistance = distance;
				nearestVertex = cur;
				nearestIndex.clear();
				nearestIndex.push_back(i);
			}
			else if( fabs(distance-nearestDistance) < 10){
				nearestIndex.push_back(i);
			}
		}
		if(nearestDistance > 20){
			nearestIndex.clear();
		}
		for(int i=0; i<(int)nearestIndex.size(); i++){
			cout << "Nearest Index = " << nearestIndex[i] << ", nearestDistance = " << nearestDistance << endl;
		}
	}
}