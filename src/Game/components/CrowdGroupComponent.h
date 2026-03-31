#pragma once

struct CrowdGroupComponent {
	int groupId = -1;
	static constexpr float kCrowdNearDist = 40.0f;
	static constexpr float kCrowdMidDist = 60.0f;
	static constexpr float kCrowdFarDist = 100.0f;
};
