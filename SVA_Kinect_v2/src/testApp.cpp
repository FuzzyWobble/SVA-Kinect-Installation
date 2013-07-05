#include "testApp.h"

//~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//~ SVA Kinect Installment ~~//
//~ fuzzywobble.com ~~~~~~~~~//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~// 

//--------------------------------------------------------------
void testApp::setup() {
	
	ofSetLogLevel(OF_LOG_VERBOSE);
	kinect.setRegistration(true);
    ofEnableAlphaBlending();
	ofSetFullscreen(false); //FULL SCREEN ON/OFF (TRUE/FALSE)
	kinect.init();
	//kinect.init(true); // shows infrared instead of RGB video image
	//kinect.init(false, false); // only depth, disable video image (faster fps)
	kinect.open(); // opens first available kinect
	
	ofSetCircleResolution(40); //circle resolution. does this matter for small circles? (10 is low, 100 is high)
	
	//open CV stuff
	colorImg.allocate(kinect.width, kinect.height);
	grayImage.allocate(kinect.width, kinect.height);
	grayThreshNear.allocate(kinect.width, kinect.height);
	grayThreshFar.allocate(kinect.width, kinect.height);
	
	bThreshWithOpenCV = true; //use openCV to subtract background, opposed to doing it manually for every pixel
	
	ofSetFrameRate(60); //could drop this to 30 if we like
	updateKinectDuration = 70;
	updateKinectTimer = 0;
	
	//set kinect angle of projection
	angle = 0; 
	kinect.setCameraTiltAngle(angle);

	bDrawPointCloud = false; //toggle this with 'p'
	
	drawContour = false; //toggle this with 'c'
	
	nearMm = 500; //closest is 500mm?
	farMm = 1500; //max is 6000mm?	
	//we don't use mm, we use threshold
	farThreshold = 200; //not sure what this is in mm?
	nearThreshold = 255; //mm?
	
	setGUI();
	gui->loadSettings("GUI/guiSettings.xml");
	
}

//--------------------------------------------------------------
void testApp::update(){
	
	
	
	//update kinect with OpenCV
	if(ofGetElapsedTimeMillis() - updateKinectTimer > updateKinectDuration){
		kinect.update();	
		updateKinectTimer = ofGetElapsedTimeMillis();
		
		if(kinect.isFrameNew()){
			
			grayImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);

			if(bThreshWithOpenCV){ //we use this one
				grayThreshNear = grayImage;
				grayThreshFar = grayImage;
				grayThreshNear.threshold(nearThreshold, true);
				grayThreshFar.threshold(farThreshold);
				cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
			}else{
				unsigned char * pix = grayImage.getPixels();
				int numPixels = grayImage.getWidth() * grayImage.getHeight();
				for(int i = 0; i < numPixels; i++) {
					if(pix[i] < nearThreshold && pix[i] > farThreshold) {
						pix[i] = 255;
					}else{
						pix[i] = 0;
					}
				}
			}

			grayImage.flagImageChanged();

			//countourfinder arguments - image, min area, max area, #, holes?, approximate?
			//we find five bodies max
			contourFinder.findContours(grayImage, 3000, (kinect.width*kinect.height)/3, 5, false);
			
			smoothContour();			
		}
	}
	
}

//--------------------------------------------------------------
void testApp::draw() {
	
	ofBackground(0); //background color
	
	ofSetColor(255, 255, 255);
	
	if(bDrawPointCloud) {
		easyCam.begin();
		drawPointCloud();
		easyCam.end();
	}else{
		
		ofSetColor(220);
		
		//draw contours 
		ofSetColor(200,200,200);
		if(drawContour==true){
			//contourFinder.draw(0, 0, 640, 480); //rough countour
			
			ofSetLineWidth(2);
			for(int i=0;i<c_smooth.size();i++){
				ofSetColor(255);
				c_smooth[i].draw(); //smooth countour
				ofPoint centroidSmoothBlob = c_smooth[i].getCentroid2D();
				ofSetColor(0,255,0);
				ofCircle(centroidSmoothBlob, 5);
				bodies[i] = centroidSmoothBlob;
				
				//vector<ofPoint> outlinePts = c_smooth[i].getVertices();
				//for(int j=0;j<outlinePts.size();j++){ 
				//	ofRect(outlinePts[j].x,outlinePts[j].y,5,5);
				//}
			}
			
			ofNoFill();
			ofRect(0,0,640,480);
			ofFill();
			
			if(c_smooth.size()!=5){
				int numBlobsLeft = 5 - c_smooth.size();
				for(int i=c_smooth.size();i<numBlobsLeft;i++){
					ofPoint zeroPt;
					zeroPt.set(0,0);
					bodies[i] = zeroPt;
				}
			}
			
		}else{
			for(int i=0;i<5;i++){
				bodies[i].x=0;
				bodies[i].y=0;
			}
		}
		
		//remap bodies
		for(int i=0;i<5;i++){
			bodiesMapped[i].x=0;
			bodiesMapped[i].y=0;
			float bodyX_map = ofMap(bodies[i].x,0,640,0,ofGetWidth());
			float bodyY_map = ofMap(bodies[i].y,0,480,0,ofGetHeight());
			ofPoint tempMap;
			tempMap.set(bodyX_map,bodyY_map);
			bodiesMapped[i] = tempMap;
		}
		
		numBodies = 0;
		for(int i=0;i<5;i++){
			if(bodiesMapped[i].x!=0 && bodiesMapped[i].y!=0){
				ofSetColor(255,0,0);
				ofCircle(bodiesMapped[i].x, bodiesMapped[i].y, 10);
				numBodies++;
			}
		}

		//
		//
		//DRAW DOTS
		//
		//

		for(int i=0;i<128;i++){ //one step in the x direction
			
			for(int j=0;j<72;j++){ //one step in the y direction
				
				float xpos = (i*20)+5; //you can adjust X spacing here (and in the FOR LOOP)
				
				float ypos = (j*20)+5; //you can adjust Y spacing here (and in the FOR LOOP)
				
				radPlus = 2.0; //default radius of the circle
				
				//looping through the 5 tracked bodies
				for(int i=0;i<5;i++){
					if(bodiesMapped[i].x!=0 && bodiesMapped[i].y!=0){				
						dist = ofDist(bodiesMapped[i].x,bodiesMapped[i].y,xpos,ypos);
						
						if(dist<150){
							
							rad = ofMap(dist,0,150,10.0,1.0); //adjust max dot size here
							
							rad = abs(rad * sin(ofGetElapsedTimef()*0.8)); //change oscillation here
							
							radPlus += rad; 
							
						}
					}
				}
					
				ofSetColor(255);  //WHITE!
				
				ofCircle(xpos, ypos, radPlus);	//draw each dot!
				
			}
		}
		
	}
	
	//fps
	if(hideGUI==false){
		ofSetColor(120,120,120);
		ofDrawBitmapString("fps: "+ofToString(ofGetFrameRate())+", bodies: "+ofToString(numBodies)+", [p]oint cloud, [h]ide GUI, view [c]ontours", 7,15);
	}
}

//------------------------------------------------------------------
float testApp::xenoToVal(float inVal, float catchVal, float speed){
	
	inVal = speed * catchVal + (1-speed) * inVal;
	return inVal;
	
	// pos.x = spd * catchX + (1-spd) * pos.x;
	// pos.x = spd * catchX + (1-spd) * pos.x; - Zachs equation
	// xeno math explianed
	// A------B--------------------C
	// A is beginning, C is end
	// say you wanna move .25 of the remaining dist each iteration
	// your first iteration you moved to B, wich is 0.25 of the distance between A and C
	// the next iteration you will move .25 the distance between B and C
	// let the next iteration be called 'new'
	// pos.new = pos.b + (pos.c-pos.b)*0.25
	// now let's simplify this equation
	// pos.new = pos.b(1-.25) + pos.c(.25)
	// since pos.new and pos.b are analogous to pos.x
	// and pos.c is analogous to catchX 
	// we can write pos.x = pos.x(1-.25) + catchX(.25) 
	// this equation is the same as Zachs simplified equation
	
}

//------------------------------------------------------------------
void testApp::drawPointCloud() {
	int w = 640;
	int h = 480;
	ofMesh mesh;
	mesh.setMode(OF_PRIMITIVE_POINTS);
	int step = 2;
	for(int y = 0; y < h; y += step) {
		for(int x = 0; x < w; x += step) {
			if(kinect.getDistanceAt(x, y) > 0) {
				mesh.addColor(kinect.getColorAt(x,y));
				mesh.addVertex(kinect.getWorldCoordinateAt(x, y));
			}
		}
	}
	glPointSize(3);
	ofPushMatrix();
	// the projected points are 'upside down' and 'backwards' 
	ofScale(1, -1, -1);
	ofTranslate(0, 0, -1000); // center the points a bit
	glEnable(GL_DEPTH_TEST);
	mesh.drawVertices();
	glDisable(GL_DEPTH_TEST);
	ofPopMatrix();
}

//--------------------------------------------------------------
void testApp::smoothContour(){
	
	c_original.clear();
	c_smooth.clear();
	vector<ofxCvBlob>& blobs = contourFinder.blobs;  
	
	for(int i=0;i<blobs.size();i++){
		ofPolyline temp;
		//temp.addVertexes(blobs[i].pts);
		temp.addVertices(blobs[i].pts);
		c_original.push_back(temp);
	}
	for(int i=0;i<c_original.size();i++){
		ofPolyline temp2 = c_original[i].getResampledBySpacing(30);
		//temp2.simplify(0.9);
		temp2.close(); 
		c_smooth.push_back(temp2);
	}
	
}



//--------------------------------------------------------------
void testApp::exit() {
	//kinect.setCameraTiltAngle(0); // zero the tilt on exit
	kinect.close();
	gui->saveSettings("GUI/guiSettings.xml");
	delete gui; 
	delete[] buffer; 
    //delete img; 
}

//--------------------------------------------------------------
void testApp::keyPressed (int key) {
	switch (key) {
		case'p':
			bDrawPointCloud = !bDrawPointCloud;
			break;
			
		case OF_KEY_UP:
			angle++;
			if(angle>30) angle=30;
			kinect.setCameraTiltAngle(angle);
			break;
			
		case OF_KEY_DOWN:
			angle--;
			if(angle<-30) angle=-30;
			kinect.setCameraTiltAngle(angle);
			break;
			
		case 'h':
            gui->toggleVisible();
			break;
			
		case 'c':
            drawContour =! drawContour;
			break;
	}
}

//--------------------------------------------------------------
void testApp::guiEvent(ofxUIEventArgs &e){
	
	string name = e.widget->getName(); 
	int kind = e.widget->getKind(); 	
	
	if(name == "UPDATE-KINECT-MILLIS"){
		ofxUISlider *slider = (ofxUISlider *) e.widget; 
		updateKinectDuration = slider->getScaledValue(); 
	}else if(name == "FAR-THRESHOLD"){
		ofxUISlider *slider = (ofxUISlider *) e.widget; 
		farThreshold = slider->getScaledValue(); 
	}else{
		//do nothing
	}
	
}	

//--------------------------------------------------------------
void testApp::setGUI(){
	
	float dim = 12; 
	float xInit = OFX_UI_GLOBAL_WIDGET_SPACING; 
    float length = 255-xInit; 
	hideGUI = false; 	
	
	gui = new ofxUICanvas(0, 30, length+xInit, ofGetHeight()); 
	gui->addSlider("FAR-THRESHOLD", 1,250, farThreshold, length-xInit,dim);
	gui->addSlider("UPDATE-KINECT-MILLIS", 20, 200, updateKinectDuration, length-xInit,dim);
 	
	ofAddListener(gui->newGUIEvent,this,&testApp::guiEvent);
}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y){
	mouseX=x;
}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h)
{}
