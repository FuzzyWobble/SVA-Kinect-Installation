#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
//#include "particle.h" //not using this, but we can
#include "line.h"
#include "ofxUI.h"

class testApp : public ofBaseApp {
public:
	
	//GENERAL OFX STUFF
	void setup();
	void update();
	void draw();
	void exit();
	void keyPressed(int key);
	void mouseMoved(int x, int y);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);
	
	//LINE STUFF
	ofPoint bodies[5];
	ofPoint bodiesMapped[5];
	int numBodies;
	float xenoToVal(float inVal, float catchVal, float speed);
	vector<ofPolyline> lines;
	vector<float> disruptance;
	vector<ofColor> colors;
	int lineCount;
	int numCircles;
	float totalRad; 
	float stepY; 
	float spaceX;
	float mouseX;
	
	//KINECT STUFF
	ofxKinect kinect;
	bool bDrawPointCloud;
	void drawPointCloud();
	float nearMm, farMm; //threshold in mm
	float nearThreshold, farThreshold;
	int angle;
	ofEasyCam easyCam;
	long updateKinectTimer;
	int updateKinectDuration;

	//OPEN-CV STUFF
	ofxCvColorImage colorImg;
	ofxCvGrayscaleImage grayImage; // grayscale depth image
	ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
	ofxCvGrayscaleImage grayThreshFar; // the far thresholded image
	ofxCvContourFinder contourFinder;
	bool bThreshWithOpenCV;
	void smoothContour();
	vector<ofPolyline> c_original;
	vector<ofPolyline> c_smooth;
	bool drawContour;
	
	//PARTICLE STUFF
	int stepA, stepB;
	vector<ofPoint> bodyPoints;
	int particleCount;
	int newParticleCount;
	float particleDamping;
	float newParticleDamping;
	float particleScale;
	float ppRepulsion,pbAttraction,pcAttraction;
	//vector <particle *> particles;
	long explodeTimer;
	bool trip;	
	
	//GUI STUFF
	bool hideGUI; 
	void setGUI();
	ofxUICanvas *gui;
	void guiEvent(ofxUIEventArgs &e);
	float *buffer; 
    ofImage *img; 
	float red, green, blue; 

};
