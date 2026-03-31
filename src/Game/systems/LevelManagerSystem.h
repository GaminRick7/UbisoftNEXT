#pragma once
#include "engine/ecs/core/System.h"

class LevelManagerSystem : public System {
public:
	void Init();
	void CreateLevel();
	void Update();
	void Shutdown();
private:
	float	m_width = 100.0f;
	float	m_depth = 80.0f;
	float	m_height = 50.0f;

	float	m_platform_w;
	float	m_platform_d;

	int		m_num_platforms = 20;
	float	m_max_platform_height = 40.0f;

	static const int	m_grid_w = 5;
	static const int	m_grid_d = 4;
};
