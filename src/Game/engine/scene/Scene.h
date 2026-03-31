#pragma once
#include <string>
#include <vector>

class SceneManager;

class Scene {
	friend class SceneManager;

public:
	Scene(const std::string& name);

	const std::string& GetName() const { return m_name; }

	virtual void Init() = 0;
	virtual void Update(float deltaTime) = 0;
	virtual void Shutdown() = 0;

protected:
	std::string m_name;

	static uint32_t s_nextID;
};

