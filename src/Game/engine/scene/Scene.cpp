#include "Scene.h"
#include "../Engine.h"

uint32_t Scene::s_nextID = 1;

Scene::Scene(const std::string& name)
	: m_name(name)
{
}
