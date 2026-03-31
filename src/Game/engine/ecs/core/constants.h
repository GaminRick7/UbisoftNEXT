#pragma once
#include <bitset>

constexpr auto MAX_COMPONENTS = 2048;
constexpr auto MAX_ENTITIES = 20000;

using ComponentMask = std::bitset<MAX_COMPONENTS>;