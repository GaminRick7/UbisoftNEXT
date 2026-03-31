#pragma once

#include "engine/event/Event.h"
#include "math/vec3.h"

struct DroneDetected : public Event {
	int groupID = 0;
	Vec3 pos;
};
