
#include "line.h"

//------------------------------------------------------------
line::line(float pass_posX, float pass_width){
	posX = pass_posX;
	width = pass_width;
	disruptance = 0.0;
	lineColor.set(200,200,200);
}

void line::draw(){
	ofSetColor(lineColor);
	ofSetLineWidth(1.0);
	thisLine.draw();
	//ofLine(posX,0,posX,ofGetHeight());
}

void line::addPoints(){
	thisLine.clear();
	float maxCircles = (2*PI) * 6; //6 circles
	float stepCircles = maxCircles/200.0; //200 steps
	for(float i=0;i<maxCircles;i=i+stepCircles){
		float cPosX = (sin(i) * disruptance) + posX;
		float cPosY = ofMap(i,0,maxCircles,0,ofGetHeight());
		ofPoint thisPoint;
		thisPoint.set(cPosX,cPosY);
		thisLine.addVertex(thisPoint);
	}
}

//when a body is overtop of the line, it becomes disrupted
void line::disrupt(ofPoint pass_bodyPos){
	if(pass_bodyPos.x!=0 && pass_bodyPos.y!=0){
		float dist = abs(pass_bodyPos.x - posX);
		//float dist_map = ofMap(dist,0,ofGetWidth(),ofGetWidth(),0);
		float dist_shaped = pow(2.718281828,-0.10*dist); //shaping function, @dist of zero returns 1, @dist of ~12 returns 0.01
		disruptance += dist*0.01;
		if(disruptance>10.0){
			disruptance = 10.0;
		}
	}
}

//bodies slowly normalize after disruption
void line::normalize(){
	disruptance -= 0.1;
	if(disruptance<0.0){
		disruptance = 0.0;
	}
}
