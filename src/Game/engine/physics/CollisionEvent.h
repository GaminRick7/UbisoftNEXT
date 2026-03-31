#pragma once
#include "engine/ecs/core/Entity.h"
#include "engine/event/Event.h"
#include "math/vec3.h"

struct CollisionEvent : public Event
{
	std::pair<Entity, Entity> entities;
	//store the normal to the collision from entity A and B
	Vec3 normal = Vec3(0.0f, 1.0f, 0.0f);
	// store how far the entities are intersected into eachother
	float penetrationDepth = 0.0f;
};


