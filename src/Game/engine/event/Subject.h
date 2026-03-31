#pragma once
#include <unordered_set>

#include "EventType.h"
#include "Observer.h"
#include "Event.h"

class Subject {
public:
	void AddObserver(Observer* observer) {
		if (!observer) {
			return;
		}
		m_observers.insert(observer);
	}

	void RemoveObserver(Observer* observer) {
		if (!observer) {
			return;
		}
		m_observers.erase(observer);
	}

	void NotifyObservers(EventType type, const Event& event) {
		for (Observer* observer : m_observers) {
			if (observer) {
				observer->OnNotify(type, event);
			}
		}
	}

private:
	std::unordered_set<Observer*> m_observers;
};
