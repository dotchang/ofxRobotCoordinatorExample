#include "PA10GUIControler.h"
#include "RobotModelUtil.h"
#include "IKFastUtil.h"

#include "mitsubishi-pa10/ikfast-pa10.h"

#define _USE_MATH_DEFINES
#include <math.h>

void PA10GUIControler::setup(){
	model.setScaleNomalization(false);
	//model.loadModel("openrave-pa10b.dae", false);
	model.loadModel("PA10b.dae", false);
	//model.loadModel("pa10.maind.dae", false);
	searchRegistOrigin(model.getAssimpScene()->mRootNode, TransformationOrigin);
	searchPrintNode(model.getAssimpScene()->mRootNode);

	arm[0].set("arm0", 0, -177, 177);
	arm[1].set("arm1", 0, -94, 94);
	arm[2].set("arm2", 0, -174, 174);
	arm[3].set("arm3", 0, -137, 137);
	arm[4].set("arm4", 0, -255, 255);
	arm[5].set("arm5", 0, -165, 165);
	arm[6].set("arm6", 0, -255, 255);

	gui.setup("PA10");
	for(int i=0; i<7; i++){	gui.add(arm[i]); }

	ik.setup("IK PA10");
	ik.add(ik_target.set("6DF51"));
	//ik.add(coords.set("Global", true));
	ik.add(xyz.set("xyz", ofPoint(0.6,0,0.6), ofPoint(-1,-1,-1), ofVec3f(1,1,1)));
	ik.add(rpy.set("rpy", ofPoint(0,-180,0), ofPoint(-180,-180,-180), ofVec3f(180,180,180)));
	ik.setPosition(800,370);

	dummy.setScaleNomalization(false);
	dummy.loadModel("PA10b.dae", false);
	dummy.setPosition(model.getPosition().x, model.getPosition().y, model.getPosition().z);
}

void jointAngleUpdate(ofxAssimpModelLoader& model, std::map<std::string, aiMatrix4x4> TransformationOrigin, ofParameter<float> arm[7])
{
	model.getAssimpScene()->mRootNode->FindNode("J1_LINK")->mTransformation = TransformationOrigin["J1_LINK"]*aiMatrix4x4().FromEulerAnglesXYZ(0,0,-arm[0]*3.14/180.0);
	model.getAssimpScene()->mRootNode->FindNode("J2_LINK")->mTransformation = TransformationOrigin["J2_LINK"]*aiMatrix4x4().FromEulerAnglesXYZ(-arm[1]*3.14/180.0,0,0);
	model.getAssimpScene()->mRootNode->FindNode("J3_LINK")->mTransformation = TransformationOrigin["J3_LINK"]*aiMatrix4x4().FromEulerAnglesXYZ(0,0,-arm[2]*3.14/180.0);
	model.getAssimpScene()->mRootNode->FindNode("J4_LINK")->mTransformation = TransformationOrigin["J4_LINK"]*aiMatrix4x4().FromEulerAnglesXYZ(-arm[3]*3.14/180.0,0,0);
	model.getAssimpScene()->mRootNode->FindNode("J5_LINK")->mTransformation = TransformationOrigin["J5_LINK"]*aiMatrix4x4().FromEulerAnglesXYZ(0,0,-arm[4]*3.14/180.0);
	model.getAssimpScene()->mRootNode->FindNode("J6_LINK")->mTransformation = TransformationOrigin["J6_LINK"]*aiMatrix4x4().FromEulerAnglesXYZ(-arm[5]*3.14/180.0,0,0);
	model.getAssimpScene()->mRootNode->FindNode("J7_LINK")->mTransformation = TransformationOrigin["J7_LINK"]*aiMatrix4x4().FromEulerAnglesXYZ(0,0,-arm[6]*3.14/180.0);

	model.getAssimpScene()->mRootNode->FindNode("J1_LINK_001")->mTransformation = TransformationOrigin["J1_LINK"]*aiMatrix4x4().FromEulerAnglesXYZ(0,0,-arm[0]*3.14/180.0);
	model.getAssimpScene()->mRootNode->FindNode("J1_LINK_002")->mTransformation = TransformationOrigin["J1_LINK"]*aiMatrix4x4().FromEulerAnglesXYZ(0,0,-arm[0]*3.14/180.0);
	model.getAssimpScene()->mRootNode->FindNode("J3_LINK_001")->mTransformation = TransformationOrigin["J3_LINK"]*aiMatrix4x4().FromEulerAnglesXYZ(0,0,-arm[2]*3.14/180.0);
	model.getAssimpScene()->mRootNode->FindNode("J3_LINK_002")->mTransformation = TransformationOrigin["J3_LINK"]*aiMatrix4x4().FromEulerAnglesXYZ(0,0,-arm[2]*3.14/180.0);
	model.getAssimpScene()->mRootNode->FindNode("J5_LINK_001")->mTransformation = TransformationOrigin["J5_LINK"]*aiMatrix4x4().FromEulerAnglesXYZ(0,0,-arm[4]*3.14/180.0);
	model.getAssimpScene()->mRootNode->FindNode("J5_LINK_002")->mTransformation = TransformationOrigin["J5_LINK"]*aiMatrix4x4().FromEulerAnglesXYZ(0,0,-arm[4]*3.14/180.0);
}

void PA10GUIControler::update(){
	jointAngleUpdate(model, TransformationOrigin, arm);
	model.update();

	inverseKinematics();
}

int PA10GUIControler::inverseKinematics()
{
	// IKfastのモデルと、openrave-pa10のモデルの座標系が合わない
	ofMatrix4x4 adjMat(0,-1,0,0, 1,0,0,0, 0,0,1,0, 0,0,0,1); //(0,-1,0,0, 1,0,0,0, 0,0,1,0, 0,0,0,1);
	aiMatrix4x4 mat;
	std::vector<std::vector<IkReal> > sol;
	std::vector<IkReal> free_joint;
	mat.FromEulerAnglesXYZ(0,0,90.0*M_PI/180.0); //
	ofMatrix4x4 yaw(mat.a1, mat.a2, mat.a3, mat.a4, mat.b1, mat.b2, mat.b3, mat.b4, mat.c1, mat.c2, mat.c3, mat.c4, mat.d1, mat.d2, mat.d3, mat.d4);
	mat.FromEulerAnglesXYZ(rpy.get().x*M_PI/180.0, rpy.get().y*M_PI/180.0, rpy.get().z*M_PI/180.0);
	ofMatrix4x4 tgtMat(mat.a1, mat.a2, mat.a3, xyz.get().x, mat.b1, mat.b2, mat.b3, xyz.get().y, mat.c1, mat.c2, mat.c3, xyz.get().z, 0, 0, 0, 1);
	tgtMat = adjMat * tgtMat * yaw;
	IkReal eerot[9] = {tgtMat(0,0), tgtMat(0,1), tgtMat(0,2), tgtMat(1,0), tgtMat(1,1), tgtMat(1,2), tgtMat(2,0), tgtMat(2,1), tgtMat(2,2)}; 
	IkReal eetrans[3] = {tgtMat(0,3), tgtMat(1,3), tgtMat(2,3)};
	int ret = 0;
	std::vector<ofParameter<float>> sol_range;

	if(ik_target.get() == "6DF51"){
		free_joint.push_back(arm[5]*M_PI/180.0);
		ret = IKFAST_6DF51::ik_solve(eerot, eetrans, sol, free_joint);
	}
	else if(ik_target.get() == "6DF41"){
		free_joint.push_back(arm[4]*M_PI/180.0);
		ret = IKFAST_6DF41::ik_solve(eerot, eetrans, sol, free_joint);
	}
	else if(ik_target.get() == "6DF3"){ // うまく動かない
		free_joint.push_back(arm[3]*M_PI/180.0);
		ret = IKFAST_6DF3::ik_solve(eerot, eetrans, sol, free_joint);
		cout << "ret =" << ret << endl;
		for(int i=0; i<sol.size(); i++){
			for(int j=0; j<sol[i].size(); j++){
				cout << sol[i][j] << ", ";
			}
			cout << endl;
		}
	}
	else if(ik_target.get() == "6DF21"){
		free_joint.push_back(arm[2]*M_PI/180.0);
		ret = IKFAST_6DF21::ik_solve(eerot, eetrans, sol, free_joint);
	}
	else if(ik_target.get() == "6DF11"){
		free_joint.push_back(arm[1]*M_PI/180.0);
		ret = IKFAST_6DF11::ik_solve(eerot, eetrans, sol, free_joint);
	}
	else if(ik_target.get() == "6DF01"){
		free_joint.push_back(arm[0]*M_PI/180.0);
		ret = IKFAST_6DF01::ik_solve(eerot, eetrans, sol, free_joint);
	}
	
	if(!ret){
		ofVec3f p0[7], p1[7];
		ofQuaternion q0[7], q1[7];
		for(int i=0; i<7; i++){
			p0[i] = model.getMeshHelper(i).matrix.getTranslation();
			q0[i] = model.getMeshHelper(i).matrix.getRotate();
		}

		for(int i=0; i<7; i++) sol_range.push_back(arm[i]);
		IKRangeCheck(sol, sol_range);
		IKSuitable(sol, sol_range);

		float min_d = 10000;
		int min_d_idx = 0;

		float min_dq = 10000;
		int min_dq_idx = 0;

		for(int i=0; i<sol.size(); i++){
			ofParameter<float> dummy_arm[7];
			for(int j=0; j<sol[i].size(); j++){
				dummy_arm[j] = sol[i].at(j) * 180.0/M_PI;
			}
			jointAngleUpdate(dummy, TransformationOrigin, dummy_arm);
			dummy.update();

			for(int j=0; j<7; j++){
				p1[j] = dummy.getMeshHelper(j).matrix.getTranslation();
				q1[j] = dummy.getMeshHelper(j).matrix.getRotate();
			}

			float d = 0;
			float dq = 0;
			for(int j=0; j<7; j++){
				d += p0[j].distance(p1[j]);
				dq += (q0[j]-q1[j]).length();
			}

			if(d<min_d){
				min_d = d;
				min_d_idx = i;
			}

			if(dq<min_dq){
				min_dq = dq;
				min_dq_idx = i;
			}
		}

		if(sol.size()){
			for(int i=0; i<7; i++){
				arm[i] = sol[min_dq_idx].at(i)*180.0/M_PI;
			}
		}
	}
	return 0;
}

void PA10GUIControler::draw(){
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
		ofDrawAxis(0.05);
		ofPopMatrix();
	}
	ofEnableLighting();
	ofEnableDepthTest();

	aiMatrix4x4 mat;
	mat.FromEulerAnglesXYZ(rpy.get().x*M_PI/180.0, rpy.get().y*M_PI/180.0, rpy.get().z*M_PI/180.0);
	IkReal eerot[9] = {mat.a1, mat.a2, mat.a3, mat.b1, mat.b2, mat.b3, mat.c1, mat.c2, mat.c3}; 
	IkReal eetrans[3] = {xyz.get().x, xyz.get().y, xyz.get().z};
	ofPushMatrix();
	ofMultMatrix(model.getModelMatrix());
	ofMultMatrix(model.getMeshHelper(0).matrix); // chest
	ofMultMatrix(ofMatrix4x4(mat.a1,mat.b1,mat.c1,0, mat.a2, mat.b2, mat.c2, 0, mat.a3, mat.b3, mat.c3, 0 , eetrans[0], eetrans[1], eetrans[2], 1));
	ofSetColor(ofColor::yellow);
	ofSphere(0.02);
	ofDrawAxis(0.05);
	ofPopMatrix();
}

void PA10GUIControler::draw_gui(){
	ofDisableDepthTest();
	ofDisableLighting();
	gui.draw();
	ik.draw();
	ofEnableLighting();
	ofEnableDepthTest();
}

void PA10GUIControler::mouseDragged(int x, int y, int button, ofEasyCam& cam)
{
	if(button == 1 && !nearestIndex.empty()){
		cam.disableMouseInput();	
		float value_x = x / (float)ofGetWidth();
		float value_y = y / (float)ofGetHeight();

		for(int i=0; i<(int)nearestIndex.size(); i++){
			aiString name = model.getMeshHelper(nearestIndex[i]).mesh->mName;
			if(name == aiString("J1_LINK")){ arm[0] = adjust_value(arm[0], value_x); }
			else if(name == aiString("J2_LINK")){ arm[1] = adjust_value(arm[1], value_x); }
			else if(name == aiString("J3_LINK")){ arm[2] = adjust_value(arm[2], value_x); }
			else if(name == aiString("J4_LINK")){ arm[3] = adjust_value(arm[3], value_x); }
			else if(name == aiString("J5_LINK")){ arm[4] = adjust_value(arm[4], value_x); }
			else if(name == aiString("J6_LINK")){ arm[5] = adjust_value(arm[5], value_x); }
			else if(name == aiString("J7_LINK")){ arm[6] = adjust_value(arm[6], value_x); }
		}
	}
}

void PA10GUIControler::mousePressed(int x, int y, int button, ofEasyCam& cam)
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

void PA10GUIControler::keyPressed(int key)
{
	switch(key){
	case 'q': ik_target.set("6DF51"); break;
	case 'w': ik_target.set("6DF41"); break;
	case 'e': ik_target.set("6DF3"); break;
	case 'r': ik_target.set("6DF21"); break;
	case 't': ik_target.set("6DF11"); break;
	case 'y': ik_target.set("6DF01"); break;
	case 'n': ik_target.set("None"); break;
	default: break;
	}
}