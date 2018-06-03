#ifndef VPOINT_H
#define VPOINT_H

#include<iostream>




class VPoint
{
public:
    float x;
    float y;
    float z;
	
};


class VPointI : public VPoint
{
public:
    float intensity;  //0-255
	
};


class VPointRGB : public VPoint
{
public:
    unsigned char R;
	unsigned char G;
	unsigned char B;
	
};

#endif // VPOINT_H
