#pragma once
#include "Model.h"
#include <unordered_map>
#include <memory>
#include <string>

class ModelManager {
public:
	std::shared_ptr<Model> loadModel(std::string name) {
		// return loaded model if it is dound
		auto it = m_models.find(name);
		if (it != m_models.end()) {
			return it->second;
		}

		//otherwise load model from scratch and store in shared pointer
		auto model = std::make_shared<Model>();
		if (model->loadFromOBJ(name)) {
			m_models[name] = model;
			return model;
		}

		return nullptr;
	}
private:
	std::unordered_map<std::string, std::shared_ptr<Model>> m_models;
};
