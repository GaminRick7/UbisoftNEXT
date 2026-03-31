#pragma once

#include "Event.h"
#include "EventType.h"

class Observer {
public:
	virtual ~Observer() = default;
	virtual void OnNotify(EventType type, const Event& payload) = 0;
};

