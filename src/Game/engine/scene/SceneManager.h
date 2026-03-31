#pragma once
#include <string>
#include <unordered_map>
#include <memory>
#include "Scene.h"

class SceneManager {
public:
	void RegisterScene(std::shared_ptr<Scene> scene);
	std::shared_ptr<Scene> GetScene(const std::string& name);
	void SwitchScene(const std::string& name);
	std::shared_ptr<Scene> GetActiveScene() const { return m_activeScene; }
	bool SceneExists(const std::string& name) const;
	void UpdateActiveScene(float deltaTime);
	// Cleanup
	void ClearScenes();
	void ResetSystems();

private:
	std::unordered_map<std::string, std::shared_ptr<Scene>> m_scenes;
	
	std::shared_ptr<Scene> m_activeScene;
};

