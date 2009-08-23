#include <cv.h>

#include "Utils/wVector.h"
using namespace windage;

class Ball
{
public:
	Vector3 basePosition;

	Vector3 position;
	Vector3 direction;
	double speed;
	double radius;
	double spatialRadius;

	double space;

	Ball();
	void Reset(double space);
	void SetBasePosition();

	void SpeedUp();
	void SpeedDown();

	Vector3 GetPosition();

	void UpdatePosition();
	void Draw();
	void CollisoinDetect();

	Vector3 CalcWorldCoordinate();
};