#include "testApp.h"

//~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//~ SVA Kinect Installment ~~//
//~ Lines & Particles ~~~~~~~//
//~ fuzzywobble.com ~~~~~~~~~//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~// 

//--------------------------------------------------------------
// comparison routine for sort...fast!
/*
bool comparisonFunction(particle * a, particle * b) { 
	return a->pos.x < b->pos.x; 
}
*/

//--------------------------------------------------------------
void testApp::setup() {
	
	ofSetLogLevel(OF_LOG_VERBOSE);
	kinect.setRegistration(true);
    ofEnableAlphaBlending();
	ofSetFullscreen(true);
	kinect.init();
	//kinect.init(true); // shows infrared instead of RGB video image
	//kinect.init(false, false); // only depth, disable video image (faster fps)
	kinect.open(); // opens first available kinect
	
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
	
	//all particle stuff ---------
	/*
	newParticleCount = 3000;
	newParticleDamping = 0.1;
	particleScale = 1.0;
	ppRepulsion = 0.1;
	pbAttraction = 0.06;
	pcAttraction = 0.05;
	stepA = 50;
	stepB = 10;
	explodeTimer = 0;
	trip = false;
	*/
	//----------------------------
	
	//LINE STUFF
	lineCount = 140; //# lines
	numCircles = 6; //# circles
	totalRad = 2*PI*numCircles; //# circles, rad
	stepY = totalRad/400; //# steps vertical
	spaceX = (ofGetWidth()+200)/float(lineCount); //spacing between lines, horizontal
	
	for(int i=0;i<lineCount;i++){
		//populate disturbance vector
		disruptance.push_back(0.0);
		//populate color vector
		ofColor tempColor;
		tempColor.set(255,255,255);
		colors.push_back(tempColor);
	}
	
	

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
void testApp::update() {
	
	ofBackground(20);
	
	/*
	if(newParticleCount!=particleCount){
		particles.clear();
		for(int i=0;i<newParticleCount;i++){
			particle * p = new particle();
			p->pos.set(ofRandom(20,620),ofRandom(20,460));
			//p->posOrigin.set(x,y);
			p->vel.set(0,0);
			p->trailSize = 0; 
			particles.push_back(p);
		}	
		particleCount = newParticleCount;
	}
	if(newParticleDamping!=particleDamping){
		for(int i=0;i<newParticleCount;i++){
			particles[i]->damping = newParticleDamping;
		}
		newParticleDamping = particleDamping;
	}
	*/
	
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
			contourFinder.findContours(grayImage, 1000, (kinect.width*kinect.height)/3, 5, false);
			
			smoothContour();			
		}
	}
	
	/*
	//All the particle/physics stuff 
	
	for (int i = 0; i < particles.size(); i++){
		particles[i]->resetForce();
	}
	
	//particle-to-contour ATTRACTION forces
	for(int i=0;i<c_smooth.size();i++){ //2
		vector<ofPoint> outlinePts = c_smooth[i].getVertices();
		for(int j=0;j<outlinePts.size();j++){ //?
			for (int k=0;k<particles.size();k++){ 
				particles[k]->addAttractionForce(outlinePts[j].x, outlinePts[j].y, 80, pcAttraction);			}
		}
	}
	
	//particle-to-body ATTRACTION forces
	//find body points (points inside body)
	bodyPoints.clear();
	for(int i=0;i<c_smooth.size();i++){ //loop through all blobs
		ofRectangle tempRect = c_smooth[i].getBoundingBox();
		int step = stepA;
		for(int y=tempRect.y;y<tempRect.y+tempRect.height;y=y+step){
			for(int x=tempRect.x;x<tempRect.x+tempRect.width;x=x+step){	
				if(kinect.getDistanceAt(x,y) < 1500){
					step = stepB;
					ofPoint temp;
					temp.set(x,y);
					bodyPoints.push_back(temp);					
				}
				else{
					step = stepA;
				}
			}
			step = stepA;
		}
	}
	for (int i=0;i<particles.size();i++){
		for(int j=0;j<bodyPoints.size();j++){
			particles[i]->addAttractionForce(bodyPoints[j].x,bodyPoints[j].y,120,pbAttraction);	
		}
	}
	
	//particle-to-particle REPULSION forces
	sort( particles.begin(), particles.end(), comparisonFunction ); //sort by x      
	for (int i = 0; i < particles.size(); i++){
		for (int j = i-1; j >= 0; j--){
			if(fabs(particles[j]->pos.x - particles[i]->pos.x) >  30){ //tricky way to speed up this loop
				break;
			}
			if(particles[i]->addRepulsionForce(*particles[j],30,ppRepulsion)){
			
			}
		}
	}	

	for (int i = 0; i < particles.size(); i++){
		//particles[i]->addForce(sin(ofGetElapsedTimef()/8.0)*0.03,sin(ofGetElapsedTimef()/4.0)*0.05);
		particles[i]->addForce(0,0.01);
		particles[i]->trailUpdate();
		particles[i]->addDampingForce();
		particles[i]->bounceOffWalls();
		particles[i]->update();
	}	
	
	*/
	
	lines.clear(); //empty the lines vector
	
	for(int i=0;i<lineCount;i++){ //loop through lines
		
		float posX_zero = (i*spaceX)-100;
		
		float speed;
		//calculate disturbance
		if(drawContour==false){ //use mouse
			speed = abs(posX_zero - mouseX); //0 is closest
		}else{ //use body
			speed = abs(posX_zero - bodiesMapped[0].x); //0 is closest
		}
		
		//shape the speed
		float speed_shaped = pow(2.718281828,-0.04*speed) * 0.01;
		
		//shape the color
		float color_shaped = pow(2.718281828,-0.01*speed);
		
		//++ disruptance
		disruptance[i] = xenoToVal(disruptance[i], 50, speed_shaped);
		
		//-- disruptance
		disruptance[i] = xenoToVal(disruptance[i], 0, 0.02);
		
		//set color
		ofColor tempColor;
		tempColor.set(255*color_shaped,255*color_shaped,255*color_shaped);
		colors[i] = tempColor;
		
		//get line points
		ofPolyline tempLine;
		for(float j=0;j<totalRad;j=j+stepY){
			float posX = (sin(j) * disruptance[i]) + posX_zero;
			float posY = ofMap(j,0,totalRad,-100,ofGetHeight()+100);
			ofPoint tempPoint;
			tempPoint.set(posX,posY);
			tempLine.addVertex(tempPoint);
		}
		lines.push_back(tempLine);
		
	}
	
}

//--------------------------------------------------------------
void testApp::draw() {
	
	ofSetColor(255, 255, 255);
	
	if(bDrawPointCloud) {
		easyCam.begin();
		drawPointCloud();
		easyCam.end();
	} else {

		//kinect.drawDepth(10, 10, 400, 300);
		//kinect.draw(420, 10, 400, 300);
		//grayImage.draw(10, 320, 400, 300);
		//ofSetColor(50,50,50);
		//grayImage.draw(0, 0, 640, 480);
		//ofSetColor(255);
		//contourFinder.draw(0, 0, 640, 480);
		
		ofSetColor(220);
		
		//draw particles
		/*
		for (int i=0;i<particles.size();i++){
			ofColor col = kinect.getColorAt(particles[i]->pos.x, particles[i]->pos.y);
			int dist = kinect.getDistanceAt(particles[i]->pos.x,particles[i]->pos.y);
			float particleSize = (particles[i]->vel.length()*particleScale) + 1.0;
			int alpha = ofMap(dist,400,3000,255,0,true);
			col.setBrightness(brightnessVal); 
			//draw streak
//			for(int j=1;j<particles[i]->trailSize;j++){
//				ofSetLineWidth(((float)j/(float)particles[i]->trail.size())*particleSize);
//				ofSetColor(col.r,col.g,col.b,(alpha-100)*((float)j/(float)particles[i]->trail.size()));
//				ofLine(particles[i]->trail[j].x,particles[i]->trail[j].y,particles[i]->trail[j-1].x,particles[i]->trail[j-1].y);
//			}
			//draw particle
			ofSetColor(col.r,col.g,col.b,alpha);
			particles[i]->draw(particleSize);
		}
		*/
		
		//col.setBrightness(brightnessVal); 
		
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
		
		//DRAW LINES
		ofSetLineWidth(2); //set line width
		for(int i=0;i<lines.size();i++){
			ofSetColor(colors[i]);
			lines[i].draw();
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
	
	/*
	else if(name == "PARTICLE-DAMPING"){
		ofxUISlider *slider = (ofxUISlider *) e.widget; 
		newParticleDamping = slider->getScaledValue(); 
	}else if(name == "PARTICLE-SCALE"){
		ofxUISlider *slider = (ofxUISlider *) e.widget; 
		particleScale = slider->getScaledValue(); 		
	}else if(name == "PARTICLE-COUNT"){
		ofxUISlider *slider = (ofxUISlider *) e.widget; 
		newParticleCount = slider->getScaledValue(); 	
	}else if(name == "PARTICLE-PARTICLE-REPULSION"){
		ofxUISlider *slider = (ofxUISlider *) e.widget; 
		ppRepulsion = slider->getScaledValue();
	}else if(name == "PARTICLE-BODY-ATTRACTION"){
		ofxUISlider *slider = (ofxUISlider *) e.widget; 
		pbAttraction = slider->getScaledValue(); 
	}else if(name == "PARTICLE-CONTOUR-ATTRACTION"){
		ofxUISlider *slider = (ofxUISlider *) e.widget; 
		pcAttraction = slider->getScaledValue(); 
	}else if(name == "STEP-A"){
		ofxUISlider *slider = (ofxUISlider *) e.widget; 
		stepA = slider->getScaledValue(); 
	}else if(name == "STEP-B"){
		ofxUISlider *slider = (ofxUISlider *) e.widget; 
		stepB = slider->getScaledValue(); 
	}
	*/
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
	//gui->addSlider("BRIGHTNESS", 100, 250, brightnessVal, length-xInit,dim);
	
	/*
	//particle GUI
	gui->addSlider("PARTICLE-DAMPING", 0.01, 0.2, newParticleDamping, length-xInit,dim); 
	gui->addSlider("PARTICLE-SCALE", 0.5, 2.0, particleScale, length-xInit,dim);
	gui->addSlider("PARTICLE-COUNT", 500, 5000, newParticleCount, length-xInit,dim);
	gui->addSlider("PARTICLE-PARTICLE-REPULSION", 0.05, 0.3, ppRepulsion, length-xInit,dim);
	gui->addSlider("PARTICLE-BODY-ATTRACTION", 0.01, 0.2, pbAttraction, length-xInit,dim);
	gui->addSlider("PARTICLE-CONTOUR-ATTRACTION", 0.01, 0.2, pcAttraction, length-xInit,dim);
	gui->addSlider("STEP-A", 20, 100, stepA, length-xInit,dim);
	gui->addSlider("STEP-B", 5, 30, stepB, length-xInit,dim);
	*/
 	
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
