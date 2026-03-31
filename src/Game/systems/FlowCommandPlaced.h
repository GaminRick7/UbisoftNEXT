#pragma once

#include "engine/ecs/core/Entity.h"
#include "engine/event/Event.h"
#include "math/vec3.h"

struct FlowCommandPlaced : public Event {
	int x = 0;
	int z = 0;
	Vec3 position = Vec3(0.0f, 0.0f, 0.0f);
	Vec3 direction = Vec3(0.0f, 0.0f, 0.0f);
};
