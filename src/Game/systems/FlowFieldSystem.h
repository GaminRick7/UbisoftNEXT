#pragma once
#include "engine/ecs/core/Entity.h"
#include "engine/ecs/core/System.h"
#include "engine/event/Subject.h"
#include "math/vec3.h"

class FlowFieldSystem : public System, public Subject {
public:
	void Init();
	void BuildFlowLevel(std::vector<int>& heightMap, int width, int length, int goalX, int goalZ);
	void updateLight();
	void handleInputs();
	void Update(float deltaTime);
	void Shutdown();
	bool PlaceCommand(int x, int z, const Vec3& direction);

	// TODO: only for render debug ---> remove!
	int GetFlowWidth() const { return m_flowWidth; }
	int GetFlowHeight() const { return m_flowLength; }
	float GetCellSize() const { return m_flowCellSize; }
	const Vec3& GetFlowOrigin() const { return m_flowOrigin; }
private:
	// track hovered + selected cell
	int m_hover = 0;
	int m_selected = -1;
	Entity m_hoverLight;
	bool m_hoverLightCreated = false;

	// flow field parameters saved for easy access
	std::vector<Entity> m_flowCellLookup;
	bool m_flowCellLookupBuilt = false;
	int m_flowWidth = 0;
	int m_flowLength = 0;
	float m_flowCellSize = 1.0f;
	Vec3 m_flowOrigin = Vec3(0.0f, 0.0f, 0.0f);

	// helpers
	void buildFlowCellLookup();
	void stepHoverFromDir(const Vec3& dir);
	Vec3 getAxisVectorFromDir(const Vec3& dir);
};
