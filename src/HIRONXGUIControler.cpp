#include "HIRONXGUIControler.h"
#include "RobotModelUtil.h"
#include "IKFastUtil.h"

#define _USE_MATH_DEFINES
#include <math.h>

#if _DEBUG
#pragma comment(lib, "C:\\workspace\\xmlrpc++0.7\\Debug\\xmlrpc.lib")
#pragma comment(lib, "C:\\workspace\\ClapackWin-3.2.1-1.0.0\\Bin\\Debug_Win32\\ClapackWin.lib")
#else
#pragma comment(lib, "C:\\workspace\\xmlrpc++0.7\\Release\\xmlrpc.lib")
#pragma comment(lib, "C:\\workspace\\ClapackWin-3.2.1-1.0.0\\Bin\\Release_Win32\\ClapackWin.lib")
#endif

#include "kawada-hironx/ikfast-hironx.h"

void HIRONXGUIControler::setup(){
	model.setScaleNomalization(false);
	model.loadModel("kawada-hironx-parallelfingers.dae", false);
	searchRegistOrigin(model.getAssimpScene()->mRootNode, TransformationOrigin);
	searchPrintNode(model.getAssimpScene()->mRootNode);

	chest.set("chest", 0, -162.3, 162.3);
	pan.set("pan", 0, -70, 70);
	tilt.set("tilt", 0, -20, 70);
	rarm[0].set("rarm0", 0, -88, 88);
	rarm[1].set("rarm1", 0, -140, 60);
	rarm[2].set("rarm2", 0, -158, 0);
	rarm[3].set("rarm3", 0, -165, 105);
	rarm[4].set("rarm4", 0, -100, 100);
	rarm[5].set("rarm5", 0, -163, 163);
	larm[0].set("larm0", 0, -88, 88);
	larm[1].set("larm1", 0, -140, 60);
	larm[2].set("larm2", 0, -158, 0);
	larm[3].set("larm3", 0, -165, 105);
	larm[4].set("larm4", 0, -100, 100);
	larm[5].set("larm5", 0, -163, 163);

	gui.setup("HIRONX");
	gui.add(chest);
	gui.add(pan);
	gui.add(tilt);
	for(int i=0; i<6; i++){	gui.add(rarm[i]); }
	for(int i=0; i<6; i++){ gui.add(larm[i]); }

	ik.setup("IK");
	ik.add(target.set("LOOKAT3D"));
	ik.add(coords.set("Global", true));
	ik.add(xyz.set("xyz", ofPoint(0.6,0,0.6), ofPoint(-1,-1,-1), ofVec3f(1,1,1)));
	ik.add(rpy.set("rpy", ofPoint(0,-180,0), ofPoint(-180,-180,-180), ofVec3f(180,180,180)));
	ik.setPosition(10,370);

	gui.add(rpc_get_angles.set("get angles", false));
	gui.add(rpc_set_angles.set("set angles", false));
	c = new XmlRpc::XmlRpcClient( "localhost", 8000 );
}

void HIRONXGUIControler::update()
{
	if(rpc_get_angles){
		rpc_get_angles = false;
		XmlRpc::XmlRpcValue args, res;
		c->execute("getJointAnglesDeg", args, res);
		chest = (double)res[0];
		pan = (double)res[1];
		tilt = (double)res[2];
		for(int i=0; i<6; i++){ rarm[i] = (double)res[3+i]; }
		for(int i=0; i<6; i++){ larm[i] = (double)res[9+i]; }
	}
	if(rpc_set_angles){
		rpc_set_angles = false;
		XmlRpc::XmlRpcValue angles, robots, rarms, larms, rhands, lhands, args, res;
		angles.setSize(5);
		robots.setSize(3);
		rarms.setSize(6);
		larms.setSize(6);
		rhands.setSize(4);
		lhands.setSize(4);
		robots[0] = chest; robots[1] = pan; robots[2] = tilt;
		for(int i=0; i<6; i++){ rarms[i] = rarm[i]; }
		for(int i=0; i<6; i++){ larms[i] = larm[i]; }
		for(int i=0; i<4; i++){ rhands[i] = 0.0f; }
		for(int i=0; i<4; i++){ lhands[i] = 0.0f; }
		angles[0] = robots;
		angles[1] = rarms;
		angles[2] = larms;
		angles[3] = rhands;
		angles[4] = lhands;
		args.setSize(2);
		args[0] = angles;
		args[1] = 30;
		c->execute("setJointAnglesDeg", args, res);
	}

	model.getAssimpScene()->mRootNode->FindNode("CHEST_JOINT0_Link")->mTransformation = TransformationOrigin["CHEST_JOINT0_Link"]*aiMatrix4x4().FromEulerAnglesXYZ(0,0,-chest*M_PI/180.0);
	model.getAssimpScene()->mRootNode->FindNode("HEAD_JOINT0_Link")->mTransformation = TransformationOrigin["HEAD_JOINT0_Link"]*aiMatrix4x4().FromEulerAnglesXYZ(0,0,-pan*M_PI/180.0);
	model.getAssimpScene()->mRootNode->FindNode("HEAD_JOINT1_Link")->mTransformation = TransformationOrigin["HEAD_JOINT1_Link"]*aiMatrix4x4().FromEulerAnglesXYZ(0,-tilt*M_PI/180.0,0);
	model.getAssimpScene()->mRootNode->FindNode("RARM_JOINT0_Link")->mTransformation = TransformationOrigin["RARM_JOINT0_Link"]*aiMatrix4x4().FromEulerAnglesXYZ(0,0,-rarm[0]*M_PI/180.0);
	model.getAssimpScene()->mRootNode->FindNode("RARM_JOINT1_Link")->mTransformation = TransformationOrigin["RARM_JOINT1_Link"]*aiMatrix4x4().FromEulerAnglesXYZ(0,-rarm[1]*M_PI/180.0,0);
	model.getAssimpScene()->mRootNode->FindNode("RARM_JOINT2_Link")->mTransformation = TransformationOrigin["RARM_JOINT2_Link"]*aiMatrix4x4().FromEulerAnglesXYZ(0,-rarm[2]*M_PI/180.0,0);
	model.getAssimpScene()->mRootNode->FindNode("RARM_JOINT3_Link")->mTransformation = TransformationOrigin["RARM_JOINT3_Link"]*aiMatrix4x4().FromEulerAnglesXYZ(0,0,-rarm[3]*M_PI/180.0);
	model.getAssimpScene()->mRootNode->FindNode("RARM_JOINT4_Link")->mTransformation = TransformationOrigin["RARM_JOINT4_Link"]*aiMatrix4x4().FromEulerAnglesXYZ(0,-rarm[4]*M_PI/180.0,0);
	model.getAssimpScene()->mRootNode->FindNode("RARM_JOINT5_Link")->mTransformation = TransformationOrigin["RARM_JOINT5_Link"]*aiMatrix4x4().FromEulerAnglesXYZ(-rarm[5]*M_PI/180.0,0,0);
	model.getAssimpScene()->mRootNode->FindNode("LARM_JOINT0_Link")->mTransformation = TransformationOrigin["LARM_JOINT0_Link"]*aiMatrix4x4().FromEulerAnglesXYZ(0,0,-larm[0]*M_PI/180.0);
	model.getAssimpScene()->mRootNode->FindNode("LARM_JOINT1_Link")->mTransformation = TransformationOrigin["LARM_JOINT1_Link"]*aiMatrix4x4().FromEulerAnglesXYZ(0,-larm[1]*M_PI/180.0,0);
	model.getAssimpScene()->mRootNode->FindNode("LARM_JOINT2_Link")->mTransformation = TransformationOrigin["LARM_JOINT2_Link"]*aiMatrix4x4().FromEulerAnglesXYZ(0,-larm[2]*M_PI/180.0,0);
	model.getAssimpScene()->mRootNode->FindNode("LARM_JOINT3_Link")->mTransformation = TransformationOrigin["LARM_JOINT3_Link"]*aiMatrix4x4().FromEulerAnglesXYZ(0,0,-larm[3]*M_PI/180.0);
	model.getAssimpScene()->mRootNode->FindNode("LARM_JOINT4_Link")->mTransformation = TransformationOrigin["LARM_JOINT4_Link"]*aiMatrix4x4().FromEulerAnglesXYZ(0,-larm[4]*M_PI/180.0,0);
	model.getAssimpScene()->mRootNode->FindNode("LARM_JOINT5_Link")->mTransformation = TransformationOrigin["LARM_JOINT5_Link"]*aiMatrix4x4().FromEulerAnglesXYZ(-larm[5]*M_PI/180.0,0,0);
	model.update();

	inverseKinematics();
}

int HIRONXGUIControler::inverseKinematics()
{
	aiMatrix4x4 mat;
	std::vector<std::vector<IkReal> > sol;
	std::vector<IkReal> free_joint;
	mat.FromEulerAnglesXYZ(rpy.get().x*M_PI/180.0, rpy.get().y*M_PI/180.0, rpy.get().z*M_PI/180.0);
	IkReal eerot[9] = {mat.a1, mat.a2, mat.a3, mat.b1, mat.b2, mat.b3, mat.c1, mat.c2, mat.c3}; 
	IkReal eetrans[3] = {xyz.get().x, xyz.get().y, xyz.get().z};
	int ret = 0;
	std::vector<ofParameter<float>> sol_range;

	if(target.get() == "LOOKAT3D"){
		if(coords.get()){ // LOOKAT3DF0
			free_joint.push_back(chest*M_PI/180.0);
			sol_range.push_back(chest);
			ret = IKFAST_LOOKAT3DF0::ik_solve(eerot, eetrans, sol, free_joint);
		}
		else{
			ret = IKFAST_LOOKAT3D::ik_solve(eerot, eetrans, sol, free_joint);
		}

		if(!ret){
			sol_range.push_back(pan);
			sol_range.push_back(tilt);
			IKRangeCheck(sol, sol_range);
			IKSuitable(sol, sol_range);
			if(sol.size()){
				pan = sol[0].at(0+free_joint.size())*180.0/M_PI;
				tilt = sol[0].at(1+free_joint.size())*180.0/M_PI;
			}
		}
	}
	else if(target.get() == "RARM6D"){
		if(coords.get()){
			free_joint.push_back(chest*M_PI/180.0);
			sol_range.push_back(chest);
			ret = IKFAST_RIGHT6DF0::ik_solve(eerot, eetrans, sol, free_joint);
		}
		else {
			ret = IKFAST_RIGHT6D::ik_solve(eerot, eetrans, sol, free_joint);
		}

		if(!ret){
			for(int i=0; i<6; i++) sol_range.push_back(rarm[i]);
			IKRangeCheck(sol, sol_range);
			IKSuitable(sol, sol_range);
			if(sol.size()){
				for(int j=0; j<sol[0].size()-free_joint.size(); j++){
					rarm[j] = sol[0][j+free_joint.size()] * 180.0/M_PI;
				}
			}
		}
	}
	else if(target.get() == "LARM6D"){
		if(coords.get()){
			free_joint.push_back(chest*M_PI/180.0);
			sol_range.push_back(chest);
			ret = IKFAST_LEFT6DF0::ik_solve(eerot, eetrans, sol, free_joint);
		}
		else {
			ret = IKFAST_LEFT6D::ik_solve(eerot, eetrans, sol, free_joint);
		}

		if(!ret){
			for(int i=0; i<6; i++) sol_range.push_back(larm[i]);
			IKRangeCheck(sol, sol_range);
			IKSuitable(sol, sol_range);
			if(sol.size()){
				for(int j=0; j<sol[0].size()-free_joint.size(); j++){
					larm[j] = sol[0][j+free_joint.size()] * 180.0/M_PI;
				}
			}
		}
	}
	else if(target.get() == "RARM5D"){
		if(coords.get()) coords = false;
		free_joint.push_back(rarm[5]*M_PI/180.0);
		ret = IKFAST_RIGHT5DF8::ik_solve(eerot, eetrans, sol, free_joint);
		if(!ret){
			for(int i=0; i<6; i++) sol_range.push_back(rarm[i]);
			IKRangeCheck(sol, sol_range);
			IKSuitable(sol, sol_range);
			if(sol.size()){
				for(int j=0; j<sol[0].size()-free_joint.size(); j++){
					rarm[j] = sol[0][j] * 180.0/M_PI;
				}
			}
		}
	}
	else if(target.get() == "LARM5D"){
		if(coords.get()) coords = false;
		free_joint.push_back(larm[5]*M_PI/180.0);
		ret = IKFAST_LEFT5DF18::ik_solve(eerot, eetrans, sol, free_joint);
		if(!ret){
			for(int i=0; i<6; i++) sol_range.push_back(larm[i]);
			IKRangeCheck(sol, sol_range);
			IKSuitable(sol, sol_range);
			if(sol.size()){
				for(int j=0; j<sol[0].size()-free_joint.size(); j++){
					larm[j] = sol[0][j] * 180.0/M_PI;
				}
			}
		}
	}
	return 0;
}

void HIRONXGUIControler::draw()
{
	ofPushMatrix();	
	model.drawFaces();
	ofDisableDepthTest();
	ofDisableLighting();
	for(int i=0; i<model.getMeshCount()/*model.getNumMeshes()*/; i++){
		ofPushMatrix();
		ofMultMatrix(model.getModelMatrix());
		ofxAssimpMeshHelper & meshHelper = model.getMeshHelper(i);
		ofMultMatrix(meshHelper.matrix);
		ofDrawAxis(0.05);
		ofPopMatrix();
	}
	ofPopMatrix();
	ofEnableLighting();
	ofEnableDepthTest();

	aiMatrix4x4 mat;
	mat.FromEulerAnglesXYZ(rpy.get().x*M_PI/180.0, rpy.get().y*M_PI/180.0, rpy.get().z*M_PI/180.0);
	IkReal eerot[9] = {mat.a1, mat.a2, mat.a3, mat.b1, mat.b2, mat.b3, mat.c1, mat.c2, mat.c3}; 
	IkReal eetrans[3] = {xyz.get().x, xyz.get().y, xyz.get().z};
	ofPushMatrix();
	ofMultMatrix(model.getModelMatrix());
	if(coords.get()) ofMultMatrix(model.getMeshHelper(0).matrix); // HIRONX
	else ofMultMatrix(model.getMeshHelper(1).matrix); // chest
	ofMultMatrix(ofMatrix4x4(mat.a1,mat.b1,mat.c1,0, mat.a2, mat.b2, mat.c2, 0, mat.a3, mat.b3, mat.c3, 0 , eetrans[0], eetrans[1], eetrans[2], 1));
	ofSetColor(ofColor::yellow);
	ofSphere(0.02);
	ofDrawAxis(0.05);
	ofPopMatrix();
}

void HIRONXGUIControler::draw_gui()
{
	ofDisableDepthTest();
	ofDisableLighting();
	gui.draw();
	ik.draw();
	ofEnableLighting();
	ofEnableDepthTest();
}

void HIRONXGUIControler::mouseDragged(int x, int y, int button, ofEasyCam& cam)
{
	if(button == 1 && !nearestIndex.empty()){
		cam.disableMouseInput();	
		float value_x = x / (float)ofGetWidth();
		float value_y = y / (float)ofGetHeight();

		for(int i=0; i<(int)nearestIndex.size(); i++){
			aiString name = model.getMeshHelper(nearestIndex[i]).mesh->mName;
			if(name == aiString("CHEST_JOINT0_Link")){ chest = adjust_value(chest, value_x); }
			else if(name == aiString("HEAD_JOINT0_Link")){ pan =  adjust_value(pan, value_x); }
			else if(name == aiString("HEAD_JOINT1_Link")){ tilt = adjust_value(tilt, value_y); }
			else if(name == aiString("RARM_JOINT0_Link")){ rarm[0] = adjust_value(rarm[0], value_x); }
			else if(name == aiString("RARM_JOINT1_Link")){ rarm[1] = adjust_value(rarm[1], value_y); }
			else if(name == aiString("RARM_JOINT2_Link")){ rarm[2] = adjust_value(rarm[2], value_x); }
			else if(name == aiString("RARM_JOINT3_Link")){ rarm[3] = adjust_value(rarm[3], value_x); }
			else if(name == aiString("RARM_JOINT4_Link")){ rarm[4] = adjust_value(rarm[4], value_x); }
			else if(name == aiString("RARM_JOINT5_Link")){ rarm[5] = adjust_value(rarm[5], value_x); }
			else if(name == aiString("LARM_JOINT0_Link")){ larm[0] = adjust_value(larm[0], value_x); }
			else if(name == aiString("LARM_JOINT1_Link")){ larm[1] = adjust_value(larm[1], value_y); }
			else if(name == aiString("LARM_JOINT2_Link")){ larm[2] = adjust_value(larm[2], value_x); }
			else if(name == aiString("LARM_JOINT3_Link")){ larm[3] = adjust_value(larm[3], value_x); }
			else if(name == aiString("LARM_JOINT4_Link")){ larm[4] = adjust_value(larm[4], value_x); }
			else if(name == aiString("LARM_JOINT5_Link")){ larm[5] = adjust_value(larm[5], value_x); }
		}
	}
}

void HIRONXGUIControler::mousePressed(int x, int y, int button, ofEasyCam& cam){
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
			else if(distance == nearestDistance){
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

int HIRONXGUIControler::getCameraPosition(int id, ofEasyCam& cam)
{
	switch(id){
	case 0: // Normal
		cam.setTarget(model.getPosition());
		cam.lookAt(model.getPosition(),ofVec3f(0,-1,0));
		break;
	case 1: { // Left Eye
			ofMatrix4x4 mTilt = model.getMeshHelper(4).matrix; // tilt
			ofMatrix4x4 mLoffset, mLeye;
			mLoffset.setTranslation(0.085, 0.07, 0.08);
			mLeye = mLoffset * mTilt * model.getModelMatrix();
			cam.reset();
			cam.setFov(82.4);
			cam.setPosition(mLeye.getTranslation());
			cam.lookAt(ofVec3f(1,0,0)*mLeye, ofVec3f(0,-1,0));
		}
		break;
	case 2: { // Right Eye
			ofMatrix4x4 mTilt = model.getMeshHelper(4).matrix; // tilt
			ofMatrix4x4 mRoffset, mReye;
			mRoffset.setTranslation(0.085, -0.07, 0.08);
			mReye = mRoffset * mTilt * model.getModelMatrix();
			cam.reset();
			cam.setFov(82.4);
			cam.setPosition(mReye.getTranslation());
			cam.lookAt(ofVec3f(1,0,0)*mReye, ofVec3f(0,-1,0));
		}
		break;
	default:
		return -1;
	}
	return 0;
}

void HIRONXGUIControler::keyPressed(int key)
{
	switch(key){
	case 'c': target.set("LOOKAT3D"); break;
	case 'r': target.set("RARM6D"); break;
	case 'R': target.set("RARM5D"); break;
	case 'l': target.set("LARM6D"); break;
	case 'L': target.set("LARM5D"); break;
	case 'n': target.set("None"); break;
	default: break;
	}
}