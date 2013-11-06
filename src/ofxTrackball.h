/*
 *  ofxTrackball.h
 *
 *  Copyright (c) 2010 lab binaer GbR, www.labbinaer.de
 *  
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *  
 *  The above copyright notice and this permission notice shall be included in
 *  all copies or substantial portions of the Software.
 *  
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 *  THE SOFTWARE.
 *
 */
#ifndef _OFX_TRACKBALL
#define _OFX_TRACKBALL

#define NO_CONSTRAIN_AXIS -1
#define X_CONSTRAIN_AXIS   0
#define Y_CONSTRAIN_AXIS   1
#define Z_CONSTRAIN_AXIS   2


#include "ofMain.h"
#include "ofEvents.h";
//#include "ofxVectorMath.h";
//#include "ofxQuaternion.h";

class ofxTrackball
{
	
public:
	ofxTrackball();
	ofxTrackball( float x, float y, float z, float radius );
	~ofxTrackball();
	
	void rotate();
	void draw();
	void update( ofEventArgs &args );
	void reset();
	
	void setCenter( float x, float y, float z );
	void setRadius( float radius );
	const float getRadius();
	void setDamping( float damping );
	const float getDamping();

	
	void enableMouse();
	void disableMouse();	
	void cursorPressed ( ofMouseEventArgs &args );
	void cursorDragged ( ofMouseEventArgs &args );
	void cursorReleased( ofMouseEventArgs &args );
	
	
private:
	ofVec3f *center;
	float radius;
	float damping;
	
	ofVec3f *vecDown, *vecDrag;
	ofQuaternion *quatNow, *quatDown, *quatDrag, *quatRot;
	
	ofVec3f *axisSet[3];
	char constrainAxis;
	
	ofVec2f *cursor;
	ofVec2f *previousCursor;
	ofVec2f *velocity;
	bool  isCursorPressed;
	
	float elapsedTime;
	
	void mouseToSphere( float x, float y, ofVec3f *v );
	void constrainVector( ofVec3f *v, ofVec3f *axis );
	
	// convenience for drawing the sphere
	GLUquadricObj *quadratic;
};

#endif // _OFX_TRACKBALL