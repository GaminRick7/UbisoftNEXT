#pragma once
#include <string>

// Stores parameters for App::Print calls.
struct UIPrintComponent
{
	float x = 0.0f;
	float y = 0.0f;
	std::string text;
	float r = 1.0f;
	float g = 1.0f;
	float b = 1.0f;
	void* font = nullptr;
};
