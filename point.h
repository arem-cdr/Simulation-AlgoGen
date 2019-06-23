// point.h

#pragma once
#include <iostream>
using namespace std;

class Point
{
public :
	Point()						{ m_x = 0; m_y = 0; m_angle=0; }
	~Point()	{;}
    Point (double x0, double y0,  double angle = 0)	{ m_x= x0; m_y = y0; m_angle = angle; }
    double getX () const { return m_x; }
    double getY () const { return m_y; }
	double getangle() const { return m_angle; }
	void setX (double nx) { m_x = nx; }
    void setY (double ny) { m_y = ny; }
	void Setangle(double val) { m_angle = val; }
 	void Setall(double x, double y, double angle) {m_x = x; m_y = y; m_angle = angle;}
	bool egal(const Point& p) const	{ return m_x == p.m_x && m_y == p.m_y; }
	void afficher() const { cout << "(" << m_x << ";" << m_y << ")"<<endl; }

private :
    double m_x, m_y;
		double m_angle;
};

