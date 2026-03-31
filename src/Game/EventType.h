#pragma once

enum class EventType {
	// Engine commands
	COLLISION,

	// Game commands
	FLOW_COMMAND_PLACED,
	FLOW_GRID_MOVED,
	DRONE_DETECTED,
	DRONE_CLEARED,
	GOAL_COUNT_REACHED
};

