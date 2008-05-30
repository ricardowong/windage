
#include <gl/glut.h>

#include "windageVector.h"

#define OBJECT_STATE_NORMAL  1
#define OBJECT_STATE_HOLD  2
#define OBJECT_STATE_MOVE  4

struct VirtualObject
{
	// radius = shape.getLength()
	// position.x - shape.x ~ position.x + shape.x
	// position.y - shape.y ~ position.y + shape.y
	// position.z - shape.z ~ position.z + shape.z
	Vector3 shape;

	Vector3 position;
	Vector3 rotation;

	Vector3 baseRotation;
	Vector3 holdDirection;

	double radius;
	double weight;
	double elasticity;
	int state;

	// »ı¼ºÀÚ
	VirtualObject():radius(1.0f), weight(1.0f), elasticity(1.0f), state(OBJECT_STATE_NORMAL)
	{
		this->shape.initialize();
		this->position.initialize();
		this->rotation.initialize();
		this->baseRotation.initialize();
		this->holdDirection.initialize();
	};

	VirtualObject(Vector3 position, double radius=1.0f)
	{
		this->position = position;
		this->radius = radius;
		this->weight = 1.0f;
		this->elasticity = 1.0f;
		this->state = OBJECT_STATE_NORMAL;

		this->shape.initialize();

		this->rotation.initialize();
		this->baseRotation.initialize();
		this->holdDirection.initialize();
	};

	VirtualObject(double positionX, double positionY, double positionZ, double radius=1.0f)
	{
		this->position.x = positionX;
		this->position.y = positionY;
		this->position.z = positionZ;

		this->radius = radius;
		this->weight = 1.0f;
		this->elasticity = 1.0f;
		this->state = OBJECT_STATE_NORMAL;

		this->shape.initialize();
		
		this->rotation.initialize();
		this->baseRotation.initialize();
		this->holdDirection.initialize();
	};

	VirtualObject(Vector3 position, Vector3 rotation, double radius=1.0f, double weight=1.0f, double elasticity=1.0f, int state=1)
	{
		this->position = position;
		this->rotation = rotation;
		this->radius = radius;
		this->weight = weight;
		this->elasticity = elasticity;
		this->state = state;

		this->shape.initialize();

		this->baseRotation.initialize();
		this->holdDirection.initialize();
	};

	// operate overwrite
	void operator=(const VirtualObject& rhs)
	{
		this->position = rhs.position;
		this->rotation = rhs.rotation;
		this->baseRotation = rhs.baseRotation;
		this->holdDirection = rhs.holdDirection;

		this->radius = rhs.radius;
		this->weight = rhs.weight;
		this->elasticity = rhs.elasticity;
		this->state = rhs.state;
	}

	Vector3 getPosition()
	{
		return this->position;
	}

	void moveDirection(Vector3 direction, double scale)
	{
		this->position += (direction * scale);
	}

	void movePosition(Vector3 position)
	{
		this->position = position;
	}

	void drawOpenGL()
	{
		glPushMatrix();

		glTranslated(this->position.x, this->position.y, this->position.z);

		int rotation;
		rotation = (int)this->rotation.x;
		this->rotation.x = (double)(rotation % 360);
		rotation = (int)this->rotation.y;
		this->rotation.y = (double)(rotation % 360);
		rotation = (int)this->rotation.z;
		this->rotation.z = (double)(rotation % 360);

		glRotatef(this->rotation.x, 1, 0, 0);
//		glRotatef(this->rotation.y, 0, 1, 0);
		glRotatef(this->rotation.z, 0, 0, 1);

		glBegin(GL_QUADS);
			glNormal3f(0.0f, 0.0f, -1.0f);
			glVertex3f(- shape.x, - shape.y, - shape.z);
			glVertex3f(- shape.x, + shape.y, - shape.z);
			glVertex3f(+ shape.x, + shape.y, - shape.z);
			glVertex3f(+ shape.x, - shape.y, - shape.z);

			glNormal3f(0.0f, 0.0f, 1.0f);
			glVertex3f(- shape.x, - shape.y, + shape.z);
			glVertex3f(- shape.x, + shape.y, + shape.z);
			glVertex3f(+ shape.x, + shape.y, + shape.z);
			glVertex3f(+ shape.x, - shape.y, + shape.z);

			glNormal3f(-1.0f, 0.0f, 0.0f);
			glVertex3f(- shape.x, - shape.y, - shape.z);
			glVertex3f(- shape.x, - shape.y, + shape.z);
			glVertex3f(- shape.x, + shape.y, + shape.z);
			glVertex3f(- shape.x, + shape.y, - shape.z);

			glNormal3f(1.0f, 0.0f, 0.0f);
			glVertex3f(+ shape.x, - shape.y, - shape.z);
			glVertex3f(+ shape.x, - shape.y, + shape.z);
			glVertex3f(+ shape.x, + shape.y, + shape.z);
			glVertex3f(+ shape.x, + shape.y, - shape.z);

			glNormal3f(0.0f, -1.0f, 0.0f);
			glVertex3f(- shape.x, - shape.y, - shape.z);
			glVertex3f(- shape.x, - shape.y, + shape.z);
			glVertex3f(+ shape.x, - shape.y, + shape.z);
			glVertex3f(+ shape.x, - shape.y, - shape.z);

			glNormal3f(0.0f, 1.0f, 0.0f);
			glVertex3f(- shape.x, + shape.y, - shape.z);
			glVertex3f(- shape.x, + shape.y, + shape.z);
			glVertex3f(+ shape.x, + shape.y, + shape.z);
			glVertex3f(+ shape.x, + shape.y, - shape.z);
		glEnd();

		glPopMatrix();
	}
};