#pragma once
#include "math/vec3.h"

struct FlowCellComponent
{
	int x;
	int z;
	Vec3 direction = Vec3(0,0,0);
	bool isLift = false;
	float liftHeight = 6.0f;
	float hoverDistanceCells = 2;
	int gridWidth = 0;
	int gridHeight = 0;
	float cellSize = 1.0f;
	float cellHeight = 0.0f;
	Vec3 origin = Vec3(0.0f, 0.0f, 0.0f);
	bool isKill = false;
	bool isGoal = false;
};
