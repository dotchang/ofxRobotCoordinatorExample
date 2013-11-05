#ifndef _PLANE_PICKER
#define _PLANE_PICKER

#include "ofMatrix4x4.h"
#include "of3dGraphics.h"
#include "ofxGui.h"

class PlanePicker
{
public:
	void setWorldToRobotMatrix(ofMatrix4x4 im)
	{
		m_w2r = im;
	}

	ofVec3f worldToRobot(ofVec3f p){
		return m_w2r*p;
	}

	//����AB�ƕ��ʂ̌�_���v�Z����
	bool IntersectPlaneAndLine(
		ofVec3f* out, //�߂�l�@��_��������Ίi�[�����
		ofVec3f A,   //�����n�_
		ofVec3f B,   //�����I�_
		ofVec4f PL ) //����
	{
		//���ʏ�̓_P
		ofVec3f P( PL.x * PL.w, PL.y * PL.w, PL.z * PL.w );

		//PA PB�x�N�g��
		ofVec3f PA( P.x - A.x, P.y - A.y, P.z - A.z );
		ofVec3f PB( P.x - B.x, P.y - B.y, P.z - B.z );

		//PA PB���ꂼ��𕽖ʖ@���Ɠ���
		double dot_PA = PA.x * PL.x + PA.y * PL.y + PA.z * PL.z;
		double dot_PB = PB.x * PL.x + PB.y * PL.y + PB.z * PL.z;

		//����͐��[�����ʏ�ɂ��������̌v�Z�̌덷���z�����Ă��܂��B�������Ďg���Ă��������B
		if ( abs(dot_PA) < 0.000001 ) { dot_PA = 0.0; }	
		if ( abs(dot_PB) < 0.000001 ) { dot_PB = 0.0; }

		//��������
		if( dot_PA == 0.0 && dot_PB == 0.0 ) {
			//���[�����ʏ�ɂ���A��_���v�Z�ł��Ȃ��B
			return false;
		} else
			if ( ( dot_PA >= 0.0 && dot_PB <= 0.0 ) ||
				( dot_PA <= 0.0 && dot_PB >= 0.0 ) ) {
					//���ς̕Е����v���X�ŕЕ����}�C�i�X�Ȃ̂ŁA�������Ă���

			} else {
				//�������Ă��Ȃ�
				return false;
			}

			//�ȉ��A��_�����߂� 

			ofVec3f AB( B.x - A.x, B.y - A.y, B.z - A.z );

			//��_��A�̋��� : ��_��B�̋��� = dot_PA : dot_PB
			double hiritu = abs(dot_PA) / ( abs(dot_PA) + abs(dot_PB) );

			out->x = A.x + ( AB.x * hiritu );
			out->y = A.y + ( AB.y * hiritu );
			out->z = A.z + ( AB.z * hiritu );

			return true;
	}

	void setupModel(){
		plane_mesh = plane_mesh.plane(1.0, 1.0, 24, 24);
		ofQuaternion quat;
		quat.makeRotate(ofVec3f(0,1,0),ofVec3f(1,0,0));
		ofMatrix4x4 mv(quat);
		for(int i=0; i<plane_mesh.getNumVertices(); i++){
			ofVec3f v=plane_mesh.getVertex(i)*mv;
			plane_mesh.setVertex(i, v);
		}
	}

	void setup()
	{
		setupModel();
		gui.setup("Pick");
		gui.add(plane.set("plane", ofVec4f(1,0,0,0), ofVec4f(-1,-1,-1,-100),ofVec4f(1,1,1,100)));
		gui.setPosition(10,620);
	}

	bool update(ofVec3f p1r, ofVec3f p2r, ofVec3f p, ofVec3f n){
		p0 = p;
		p1 = p1r;
		p2 = p2r;
		ofVec3f nn = n.getNormalized();
		float d = nn.x*p0.x + nn.y*p0.y + nn.z*p0.z;
		plane = ofVec4f(n.x, n.y, n.z, d);
		return IntersectPlaneAndLine(&pf, p1, p2, plane);
	}

	void draw(){
		ofLine(p1,p2);
		ofSphere(pf,0.01);
		
		ofPushMatrix();
		ofQuaternion quat;
		quat.makeRotate(ofVec3f(0,0,1),ofVec3f(plane.get().x,plane.get().y,plane.get().z));
		ofMatrix4x4 m(quat);
		m.setTranslation(p0);
		ofMultMatrix(m);
		plane_mesh.drawVertices();
		ofPopMatrix();
	}

	void draw_gui(){
		gui.draw();
	}

	ofMatrix4x4 m_w2r;
	ofVec3f p0, p1, p2, pf;
	ofParameter<ofVec4f> plane;
	ofMesh plane_mesh;

	ofxPanel gui;
};

#endif
