#include <windows.h>		// Header File For Windows
#include <time.h>

#include <gl/gl.h>			// Header File For The OpenGL32 Library
#include <gl/glu.h>			// Header File For The GLu32 Library
#include <gl/glut.h>

#include <cv.h>
#include <highgui.h>


#include "ball.h"

Ball::Ball()
{
	Reset(120);
}

void Ball::Reset(double space)
{
	basePosition = Vector3(90, 70, 150.0);

	position = Vector3(0, 0, 0);

	srand(timeGetTime());
	direction = Vector3(rand()%100, rand()%100, rand()%100);
	speed = 30;

	radius = 20;
	spatialRadius = radius * 2;

	this->space = space - radius;
}

void Ball::SetBasePosition()
{
	glTranslatef(basePosition.x, basePosition.y, basePosition.z);
}

void Ball::SpeedUp()
{
	speed += 5;
}

void Ball::SpeedDown()
{
	if(speed > 5)
		speed -= 5;
}


Vector3 Ball::GetPosition()
{
	return position;
}

void Ball::UpdatePosition()
{
	direction /= direction.getLength();
	direction *= speed;

	position += direction;
}

void Ball::Draw()
{
	glPushMatrix();

	glEnable(GL_LIGHTING);
	glColor4f(1, 0, 0, 1);
	glutSolidSphere(this->radius, 16, 16);

	glDisable(GL_LIGHTING);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE);

	glColor4f(1, 0, 0, 0.5);
	glutSolidSphere(this->spatialRadius, 16, 16);

	glDisable(GL_BLEND);
	glEnable(GL_LIGHTING);

	glPopMatrix();
}

void Ball::CollisoinDetect()
{
	if(-space > position.x+direction.x || position.x+direction.x > space)
		if(-space > position.x)
			position.x = -space;
		else if(position.x > space)
			position.x = space;
		else
			direction.x = -direction.x;

	if(-space > position.y+direction.y || position.y+direction.y > space)
		if(-space > position.y)
			position.y = -space;
		else if(position.y > space)
			position.y = space;
		else
			direction.y = -direction.y;

	if(-space > position.z+direction.z || position.z+direction.z > space)
		if(-space > position.z)
			position.z = -space;
		else if(position.z > space)
			position.z = space;
		else
			direction.z = -direction.z;


}

Vector3 Ball::CalcWorldCoordinate()
{
	return basePosition + position;
}