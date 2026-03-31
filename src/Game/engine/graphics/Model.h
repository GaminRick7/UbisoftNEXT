#pragma once
#include <vector>
#include <string>

#include "Triangle.h"
#include "engine/spatial/AABB.h"

struct Model
{
	std::vector<Triangle> triangles;

	// generated bounds but they don't need to be used if not needed
	AABB bounds;

	bool loadFromOBJ(std::string fileName);
};
