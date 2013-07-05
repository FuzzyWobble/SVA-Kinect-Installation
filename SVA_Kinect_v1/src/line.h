#ifndef LINE_H
#define LINE_H

#include "ofMain.h"

class line{
public:
	
	line(float pass_posX, float pass_width);
	void disrupt(ofPoint pass_bodyPos);
	void normalize();
	void addPoints();
	void draw();
	
	float posX;
	float width;
	ofColor lineColor;
	float disruptance;
	
	ofPolyline thisLine;
	
protected:
private:
};

#endif // LINE_H