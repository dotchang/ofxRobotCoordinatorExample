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

	//線分ABと平面の交点を計算する
	bool IntersectPlaneAndLine(
		ofVec3f* out, //戻り値　交点が見つかれば格納される
		ofVec3f A,   //線分始点
		ofVec3f B,   //線分終点
		ofVec4f PL ) //平面
	{
		//平面上の点P
		ofVec3f P( PL.x * PL.w, PL.y * PL.w, PL.z * PL.w );

		//PA PBベクトル
		ofVec3f PA( P.x - A.x, P.y - A.y, P.z - A.z );
		ofVec3f PB( P.x - B.x, P.y - B.y, P.z - B.z );

		//PA PBそれぞれを平面法線と内積
		double dot_PA = PA.x * PL.x + PA.y * PL.y + PA.z * PL.z;
		double dot_PB = PB.x * PL.x + PB.y * PL.y + PB.z * PL.z;

		//これは線端が平面上にあった時の計算の誤差を吸収しています。調整して使ってください。
		if ( abs(dot_PA) < 0.000001 ) { dot_PA = 0.0; }	
		if ( abs(dot_PB) < 0.000001 ) { dot_PB = 0.0; }

		//交差判定
		if( dot_PA == 0.0 && dot_PB == 0.0 ) {
			//両端が平面上にあり、交点を計算できない。
			return false;
		} else
			if ( ( dot_PA >= 0.0 && dot_PB <= 0.0 ) ||
				( dot_PA <= 0.0 && dot_PB >= 0.0 ) ) {
					//内積の片方がプラスで片方がマイナスなので、交差している

			} else {
				//交差していない
				return false;
			}

			//以下、交点を求める 

			ofVec3f AB( B.x - A.x, B.y - A.y, B.z - A.z );

			//交点とAの距離 : 交点とBの距離 = dot_PA : dot_PB
			double hiritu = abs(dot_PA) / ( abs(dot_PA) + abs(dot_PB) );

			out->x = A.x + ( AB.x * hiritu );
			out->y = A.y + ( AB.y * hiritu );
			out->z = A.z + ( AB.z * hiritu );

			return true;
	}

	void setupModel(){
		plane_mesh = plane_mesh.plane(width, height, 24, 24);
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
		width = 1.0;
		height = 1.0;
		setupModel();
		gui.setup("Pick");
		gui.add(p0.set("p0", ofPoint(0,0,0), ofPoint(-1,-1,-1), ofPoint(1,1,1)));
		gui.add(n.set("n", ofPoint(1,0,0), ofPoint(-1,-1,-1), ofPoint(1,1,1)));
		gui.add(plane.set("plane", ofVec4f(1,0,0,0), ofVec4f(-1,-1,-1,-1),ofVec4f(1,1,1,1)));
		gui.setPosition(10,620);
	}

	bool update(ofVec3f p1r, ofVec3f p2r){
		p1 = p1r;
		p2 = p2r;
		ofVec3f nn = n.get().getNormalized();
		float d = nn.x*p0.get().x + nn.y*p0.get().y + nn.z*p0.get().z;
		plane = ofVec4f(nn.x, nn.y, nn.z, d);
		return IntersectPlaneAndLine(&pf, p1, p2, plane);
	}

	bool update(ofVec3f p1r, ofVec3f p2r, ofVec3f p, ofVec3f n){
		p0 = p;
		p1 = p1r;
		p2 = p2r;
		ofVec3f nn = n.getNormalized();
		float d = nn.x*p0.get().x + nn.y*p0.get().y + nn.z*p0.get().z;
		plane = ofVec4f(nn.x, nn.y, nn.z, d);
		return IntersectPlaneAndLine(&pf, p1, p2, plane);
	}

	bool update(ofVec3f p1r, ofVec3f p2r, ofVec4f pl){
		p1 = p1r;
		p2 = p2r;
		plane = pl;
		return IntersectPlaneAndLine(&pf, p1, p2, plane);
	}

	ofVec3f robotToPlane(ofVec3f p){
		ofQuaternion quat;
		quat.makeRotate(ofVec3f(0,0,1),ofVec3f(plane.get().x,plane.get().y,plane.get().z));
		ofMatrix4x4 m(quat);
		m.setTranslation(p0);
		ofMatrix4x4 im = m.getInverse();
		return p*im;
	}

	ofVec3f robotToPlane(){ return robotToPlane(pf); }

	bool onPlane(ofVec3f p){
		ofVec3f v = robotToPlane(p);
		if(v.x<-0.5*width||0.5*width<v.x) return false;
		if(v.y<-0.5*height||0.5*height<v.y) return false;
		return true;
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
		ofBox(ofPoint(0,0,0),0.01);
		plane_mesh.drawVertices();
		ofPopMatrix();
	}

	void draw_gui(){
		gui.draw();
	}

	ofMatrix4x4 m_w2r;
	ofVec3f p1, p2, pf;
	ofParameter<ofVec4f> plane;
	float width, height;
	ofMesh plane_mesh;

	ofParameter<ofVec3f> p0, n;
	ofxPanel gui;
};

#endif
