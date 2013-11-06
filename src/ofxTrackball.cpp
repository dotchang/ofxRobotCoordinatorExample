/*
 *  ofxTrackball.cpp
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

#include "ofxTrackball.h"

#pragma mark constructors and destructor

ofxTrackball::ofxTrackball()
{
	ofxTrackball( ofGetWidth()/2, ofGetHeight()/2, 0, ofGetHeight() );
}

ofxTrackball::ofxTrackball( float x, float y, float z, float radius )
{
	this->center = new ofVec3f( x, y, z );
	this->radius = radius;
	this->damping = 5.0;
	
	this->vecDown  = new ofVec3f();
	this->vecDrag  = new ofVec3f();
	this->quatNow  = new ofQuaternion();
	this->quatDown = new ofQuaternion();
	this->quatDrag = new ofQuaternion();
	this->quatRot  = new ofQuaternion();
	
	// setup constrain axis
	this->axisSet[X_CONSTRAIN_AXIS] = new ofVec3f(1.0, 0.0, 0.0);
	this->axisSet[Y_CONSTRAIN_AXIS] = new ofVec3f(0.0, 1.0, 0.0);
	this->axisSet[Z_CONSTRAIN_AXIS] = new ofVec3f(0.0, 0.0, 1.0);
	constrainAxis = NO_CONSTRAIN_AXIS;
	
	// set cursor
	cursor = new ofVec2f();
	previousCursor = new ofVec2f();
	velocity = new ofVec2f();
	isCursorPressed = false;
	
	// setup event handler
	enableMouse();
	ofAddListener( ofEvents().update, this, &ofxTrackball::update );
	
	// setup drawing
	quadratic=gluNewQuadric();
	
	// setup time
	elapsedTime = ofGetElapsedTimef();
}

ofxTrackball::~ofxTrackball()
{
	delete center;
	delete vecDown;
	delete vecDrag;
	delete quatNow;
	delete quatDown;
	delete quatDrag;
	delete axisSet;
	delete velocity;
	
	disableMouse();
	ofRemoveListener( ofEvents().update, this, &ofxTrackball::update );
}


#pragma mark apply rotation
void ofxTrackball::rotate()
{
	ofTranslate( center->x, center->y, center->z );
	{
		// update rotation quaternion
		float _w = quatDrag->w() * quatDown->w() - quatDrag->x() * quatDown->x() - quatDrag->y() * quatDown->y() - quatDrag->z() * quatDown->z();
		float _x = quatDrag->w() * quatDown->x() + quatDrag->x() * quatDown->w() + quatDrag->y() * quatDown->z() - quatDrag->z() * quatDown->y();
		float _y = quatDrag->w() * quatDown->y() + quatDrag->y() * quatDown->w() + quatDrag->z() * quatDown->x() - quatDrag->x() * quatDown->z();
		float _z = quatDrag->w() * quatDown->z() + quatDrag->z() * quatDown->w() + quatDrag->x() * quatDown->y() - quatDrag->y() * quatDown->x();
		quatNow->set( _x, _y, _z, _w );
		
		// apply rotation
		float angle, x, y, z;
		quatNow->getRotate( angle, x, y, z );
		ofRotate( ofRadToDeg(angle), x, y, z );
		
	}	
	ofTranslate( -center->x, -center->y, -center->z );
}

void ofxTrackball::draw()
{
	glDisable( GL_DEPTH_TEST );
	
	// radius
	ofNoFill();
	ofSetColor( 255, 255, 255 );
	ofDrawBitmapString( "radius "+ofToString(radius, 2), 20, 20 );
	ofCircle( center->x, center->y, radius );
	
	// trackball
	ofPushMatrix();
	{
		glEnable( GL_DEPTH_TEST );
		glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
		rotate();
		ofTranslate( center->x, center->y, center->z);
		ofEnableAlphaBlending();
		ofSetColor( 255, 255, 255, 64 );
		gluSphere( quadratic, radius, 32, 32 );	
		glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
		glDisable( GL_DEPTH_TEST );
	}
	ofPopMatrix();
	
	// center
	ofSetColor( 255, 255, 255 );
	ofLine( center->x-5, center->y, center->x+5, center->y );
	ofLine( center->x, center->y-5, center->x, center->y+5 );
	
	// quatNow
	float angle, x, y, z;
	quatNow->getRotate( angle, x, y, z );
	ofDrawBitmapString( "w "+ofToString( ofRadToDeg(angle), 2), 20, 50 );
	ofDrawBitmapString( "x "+ofToString( x, 2), 20, 70 );
	ofDrawBitmapString( "y "+ofToString( y, 2), 20, 90 );
	ofDrawBitmapString( "z "+ofToString( z, 2), 20, 110 );
	ofSetColor(255, 0, 0);
	
	//vecDrag
	ofSetColor( 255, 255, 255 );
	ofDrawBitmapString( "vecDrag", center->x+vecDrag->x*radius+10, center->y+vecDrag->y*radius-10 );	
	ofLine( center->x, center->y, center->x+vecDrag->x*radius, center->y+vecDrag->y*radius );
	ofFill();	
	ofSetColor( 0, 255, 0 );
	ofCircle( center->x+vecDrag->x*radius, center->y+vecDrag->y*radius, 5 );
	
	// vecDown
	ofSetColor( 255, 255, 255 );
	ofDrawBitmapString( "vecDown", center->x+vecDown->x*radius+10, center->y+vecDown->y*radius-10 );	
	ofLine( center->x, center->y, center->x+vecDown->x*radius, center->y+vecDown->y*radius );
	ofFill();	
	ofSetColor( 0, 255, 0 );
	ofCircle( center->x+vecDown->x*radius, center->y+vecDown->y*radius, 5 );
	
	// velocity
	ofSetColor( 255, 0, 0 );
	ofLine( center->x+vecDown->x*radius, center->y+vecDown->y*radius, center->x+vecDown->x*radius+velocity->x, center->y+vecDown->y*radius+velocity->y );
	
	glEnable( GL_DEPTH_TEST );
}


void ofxTrackball::update( ofEventArgs &args )
{
	// calculate elapsed time since last update
	// time independence
	float currentTime = ofGetElapsedTimef();
	float step = currentTime - elapsedTime;
	
	// decelerate
	float deceleration = 1.0-damping*step;
	if ( deceleration < 0 ) deceleration = 0;
	*velocity *= deceleration;
	
	// only roll when user is not interacting
	if ( !isCursorPressed )
	{	
		// stop at very slow velocity
		if ( velocity->length() < 0.01 )
		{
			// TODO: callback or stop event with rotation
			// TODO: snap to tick (slerp to snap step)
		}
		else
		{
			// calculate auto drag cursor
			quatDown->set( quatNow->x(), quatNow->y(), quatNow->z(), quatNow->w() );
			quatDrag->set( 0, 0, 0, 1 );	
			cursor->set( center->x+vecDown->x*radius+velocity->x*100*step, center->y+vecDown->y*radius+velocity->y*100*step );
			
			// calculate rotation
			mouseToSphere( cursor->x, cursor->y, vecDrag );
			quatDrag->makeRotate( *vecDown, *vecDrag );
			
			// TODO: callback or tick events when passing snapping points
		}
	}
	
	// update timestamp
	elapsedTime = currentTime;
}

void ofxTrackball::reset()
{
	vecDown->set ( 0, 0, 0 );
	vecDrag->set ( 0, 0, 0 );
	quatNow->set ( 0, 0, 0, 1 );
	quatDown->set( 0, 0, 0, 1 );
	quatDrag->set( 0, 0, 0, 1 );
	cursor->set( 0, 0 );
	previousCursor->set( 0, 0 );
	velocity->set( 0, 0 );
}


#pragma mark getter and setter
void ofxTrackball::setCenter( float x, float y, float z )
{
	center->set( x, y, z );
}

void ofxTrackball::setRadius( float r )
{
	if ( radius <= 0 ) this->radius = 1;
	else this->radius = r;
}

const float ofxTrackball::getRadius()
{
	return radius;
}

void ofxTrackball::setDamping( float damping )
{
	if ( damping < 0 ) damping = 0;
	this->damping = damping;
}

const float ofxTrackball::getDamping()
{
	return damping;
}


#pragma mark event handling

void ofxTrackball::enableMouse()
{
	ofAddListener( ofEvents().mousePressed,  this, &ofxTrackball::cursorPressed );
	ofAddListener( ofEvents().mouseDragged,  this, &ofxTrackball::cursorDragged );
	ofAddListener( ofEvents().mouseReleased, this, &ofxTrackball::cursorReleased );
}

void ofxTrackball::disableMouse()
{
	ofRemoveListener( ofEvents().mousePressed,  this, &ofxTrackball::cursorPressed );
	ofRemoveListener( ofEvents().mouseDragged,  this, &ofxTrackball::cursorDragged );
	ofRemoveListener( ofEvents().mouseReleased, this, &ofxTrackball::cursorReleased );
}

void ofxTrackball::cursorPressed( ofMouseEventArgs &args )
{
	if ( args.button == 0 )
	{
		isCursorPressed = true;
		cursor->set( args.x, args.y );
		previousCursor->set( args.x, args.y );


		mouseToSphere( cursor->x, cursor->y, vecDown );
		quatDown->set( quatNow->x(), quatNow->y(), quatNow->z(), quatNow->w() );
		quatDrag->set( 0, 0, 0, 1 );
	}
}

void ofxTrackball::cursorDragged( ofMouseEventArgs &args )
{
	if ( args.button == 0 )
	{
		previousCursor->set( cursor->x, cursor->y );
		cursor->set( args.x, args.y );
		*velocity = *cursor - *previousCursor;
		
		mouseToSphere( cursor->x, cursor->y, vecDrag );
		quatDrag->makeRotate( *vecDown, *vecDrag );
	}
}

void ofxTrackball::cursorReleased( ofMouseEventArgs &args )
{
	isCursorPressed = false;
}


# pragma mark calculations
void ofxTrackball::mouseToSphere( float x, float y, ofVec3f *v )
{
    v->x = (x - center->x) / radius;
    v->y = (y - center->y) / radius;
	
    float mag = v->x * v->x + v->y * v->y;
    if (mag > 1.0f)
    {
		v->normalize();
    }
    else
    {
		v->z = sqrt(1.0f - mag);
    }
    if (constrainAxis != -1) constrainVector( v, axisSet[constrainAxis] );
}

void ofxTrackball::constrainVector( ofVec3f *v, ofVec3f *axis )
{
	float dot = v->dot(*axis);
	ofVec3f *projection = new ofVec3f(*axis);
	*projection *= dot;
	*v -= *projection;
    v->normalize();
	delete projection;
}
