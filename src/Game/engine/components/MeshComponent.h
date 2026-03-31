#pragma once
#include <memory>

#include "math/vec3.h"

struct MeshComponent
{
	// models are shared between meshes to avoid duplication
	std::shared_ptr<Model> mesh;
	Vec3 colour;
};
