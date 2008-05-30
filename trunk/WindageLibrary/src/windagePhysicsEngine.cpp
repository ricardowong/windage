
#include <vector>

#include "../include/windageObject.h"
#include "../include/windageMatrix.h"

#define M_PI 3.14159265
#define CONVERT_DEGREE(RADIAN) ((180.0f / M_PI) * (RADIAN))
#define CONVERT_RADIAN(DEGREE) ((M_PI / 180.0f) * (DEGREE))

extern "C" __declspec(dllexport)
bool CollisionCalculate(Vector3* direction, VirtualObject* reactor, VirtualObject* actor)
{
	(*direction) = reactor->position - actor->position;

	double distance = direction->getLength();
	double scale = (reactor->radius + actor->radius - distance);

	if(scale < 1.0f)
	{
		scale = 0.0f;
		direction->x = 0.0f;
		direction->y = 0.0f;
		direction->z = 0.0f;
		return false;
	}
	
	if(distance != 0.0f)
		(*direction) = (*direction)*(scale/distance);
	return true;
}

extern "C" __declspec(dllexport)
bool SingleCollisionProcess(VirtualObject* reactor, VirtualObject* actor)
{
	Vector3 direction;

	if(CollisionCalculate(&direction, reactor, actor))
	{
		reactor->moveDirection(direction, reactor->elasticity * actor->elasticity / 2.0f);
		return true;
	}
	else
	{
		return false;
	}
}

// return radian
double GetTheta(Vector2 vector1, Vector2 vector2)
{
	vector1 /= vector1.getLength();
	vector2 /= vector2.getLength();

	return acos( (vector1 * vector2) );
}

double GetTheta(double x1, double y1, double x2, double y2)
{
	Vector2 vector1(x1, y1);
	Vector2 vector2(x2, y2);
	return GetTheta(vector1, vector2);
}

void GetRotate(Vector2* result, Vector2 start, double theta)
{
	Matrix2 rotation(cos(theta), -sin(theta),
					 sin(theta),  cos(theta) );
	(*result) = rotation * start;
}

extern "C" __declspec(dllexport)
void CalculateRotation(Vector3* result, Vector3 vector1, Vector3 vector2)
{
	Vector2 vec1, vec2;
	Vector3 ovec1, ovec2, ovecT;
	double theta;

	// yz-plane
	vec1.x = vector1.y;
	vec1.y = vector1.z;
	vec2.x = vector2.y;
	vec2.y = vector2.z;

	theta = GetTheta(vec1, vec2);

	ovec1.x = vec1.x;
	ovec1.y = vec1.y;
	ovec1.z = 0;
	ovec2.x = vec2.x;
	ovec2.y = vec2.y;
	ovec2.z = 0;

	ovecT = ovec1 ^ ovec2;

	if(ovecT.z > 0)
		result->x = CONVERT_DEGREE(theta);
	else
		result->x = CONVERT_DEGREE(-theta);


	// zx-plane
	vec1.x = vector1.z;
	vec1.y = vector1.x;
	vec2.x = vector2.z;
	vec2.y = vector2.x;

	theta = GetTheta(vec1, vec2);

	ovec1.x = vec1.x;
	ovec1.y = vec1.y;
	ovec1.z = 0;
	ovec2.x = vec2.x;
	ovec2.y = vec2.y;
	ovec2.z = 0;

	ovecT = ovec1 ^ ovec2;

	if(ovecT.z > 0)
		result->y = CONVERT_DEGREE(theta);
	else
		result->y = CONVERT_DEGREE(-theta);

	// xy-plane
	vec1.x = vector1.x;
	vec1.y = vector1.y;
	vec2.x = vector2.x;
	vec2.y = vector2.y;

	theta = GetTheta(vec1, vec2);

	ovec1.x = vec1.x;
	ovec1.y = vec1.y;
	ovec1.z = 0;
	ovec2.x = vec2.x;
	ovec2.y = vec2.y;
	ovec2.z = 0;

	ovecT = ovec1 ^ ovec2;

	if(ovecT.z > 0)
		result->z = CONVERT_DEGREE(theta);
	else
		result->z = CONVERT_DEGREE(-theta);
}


extern "C" __declspec(dllexport)
void MultiCollisionProcess(std::vector<VirtualObject>* reactor, std::vector<VirtualObject>* actor, double holdThreshold)
{
	int reactorCount = (int)reactor->size();
	int actorCount = (int)actor->size();

	int i, j, k;

	std::vector<VirtualObject> actionObject;
	Vector3 result, direction, rotation;

	for(i=0; i<reactorCount; i++)
	{
		result.initialize();

		actionObject.clear();
		for(j=0; j<actorCount; j++)
		{
			if( CollisionCalculate( &direction, &((*reactor)[i]), &((*actor)[j]) ) == true )
			{
				result += direction;
				actionObject.push_back((*actor)[j]);
			}
		}

		int actionCount = (int)actionObject.size();
		if(actionCount >= 2) // multi collision process (object hold check)
		{
			double length = result.getLength();

			if(length < holdThreshold) // hold object
			{
				// move object
				Vector3 holdPosition;
				holdPosition.initialize();
				for(k=0; k<2; k++)
				{
					holdPosition += actionObject[k].position;
				}
				holdPosition /= 2.0f;

				(*reactor)[i].movePosition(holdPosition);

				// rotation object
				if((*reactor)[i].state == OBJECT_STATE_NORMAL)
				{
					(*reactor)[i].holdDirection = actionObject[1].position - actionObject[0].position;
					(*reactor)[i].baseRotation = (*reactor)[i].rotation;
				}
				else if((*reactor)[i].state == OBJECT_STATE_HOLD) // rotate object
				{
					CalculateRotation(&rotation, (*reactor)[i].holdDirection, actionObject[1].position - actionObject[0].position);
					(*reactor)[i].rotation = (*reactor)[i].baseRotation + rotation;
				}
				else // etc...
				{
				}

				(*reactor)[i].state = OBJECT_STATE_HOLD;
			}
			else // not holding
			{
				(*reactor)[i].moveDirection(result, (*reactor)[i].elasticity);
				(*reactor)[i].state = OBJECT_STATE_NORMAL;
			}
		}
		else // single collision process
		{
			(*reactor)[i].moveDirection(result, (*reactor)[i].elasticity);
			(*reactor)[i].state = OBJECT_STATE_NORMAL;
		}

	}
}


