#pragma once

// Event payload that can be made to support any kind of data
struct Event {
	virtual ~Event() = default;
};

