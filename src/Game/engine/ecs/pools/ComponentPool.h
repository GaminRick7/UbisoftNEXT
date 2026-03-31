#pragma once
#include <cassert>
#include <iostream>
#include <array>
#include "../core/Entity.h"
#include "../core/constants.h"

class IComponentPool
{
public:
    // ensures component array's destructor is called
    virtual ~IComponentPool() = default;
    virtual void EntityDestroyed(EntityID e) = 0;
    virtual void clear() = 0;
};

// A packed component pool that a sparse–dense mapping to keep the active components densely packed together.
//    it's great for fast iteration for systems!
//    it's also type-safe and uses one pool per component type!!
template<class T>
class ComponentPool : public IComponentPool{
public:
    void add(EntityID entity, const T& component) {
        assert(!has(entity));
        assert(m_size < MAX_ENTITIES);

        const size_t index = m_size;

        m_componentArray[index] = component;
        m_indexToEntity[index]  = entity;
        m_entityToIndex[entity] = index;

        ++m_size;
    }

    T& get(EntityID entity) {
        assert(has(entity));
        return m_componentArray[m_entityToIndex[entity]];
    }

    // use a swap-and-pop to keep the density
    void remove(EntityID entity) {
        assert(entity < MAX_ENTITIES);
        assert(has(entity));

        const size_t index = m_entityToIndex[entity];
        const size_t last  = m_size - 1;

        // move last element into removed slot (if it's not already last)
        // density preserved :)
        if (index != last) {
            m_componentArray[index] = m_componentArray[last];
            m_indexToEntity[index]  = m_indexToEntity[last];
            m_entityToIndex[m_indexToEntity[index]] = index;
        }

        --m_size;
    }

    bool has(EntityID entity) const {
        assert(entity < MAX_ENTITIES);
        const size_t index = m_entityToIndex[entity];
        return index < m_size && m_indexToEntity[index] == entity;
    }

    void EntityDestroyed(EntityID e) override
    {
        if (has(e)) remove(e);
    }

    void clear() override {
        // clear all the mapping arrays
        for (size_t i = 0; i < m_size; ++i) {
            EntityID entityId = m_indexToEntity[i];
            m_entityToIndex[entityId] = MAX_ENTITIES;
        }
        m_size = 0;
    }


private:
    std::array<T, MAX_ENTITIES> m_componentArray{};
    std::array<EntityID, MAX_ENTITIES> m_indexToEntity{};
    std::array<size_t, MAX_ENTITIES> m_entityToIndex{};

    size_t m_size = 0;
};
