
#include <vector>

#include "windageObject.h"

#define M_PI 3.14159265
#define CONVERT_DEGREE(RADIAN) ((180.0f / M_PI) * (RADIAN))
#define CONVERT_RADIAN(DEGREE) ((M_PI / 180.0f) * (DEGREE))

extern "C" __declspec(dllimport)
bool CollisionCalculate(Vector3* direction, VirtualObject* reactor, VirtualObject* actor);

extern "C" __declspec(dllimport)
bool SingleCollisionProcess(VirtualObject* reactor, VirtualObject* actor);

extern "C" __declspec(dllimport)
void CalculateRotation(Vector3* result, Vector3 vector1, Vector3 vector2);

extern "C" __declspec(dllimport)
void MultiCollisionProcess(std::vector<VirtualObject>* reactor, std::vector<VirtualObject>* actor, double holdThreshold=50.0f);