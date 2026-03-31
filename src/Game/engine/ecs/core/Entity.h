#pragma once
#include <cstdint>
#include <functional>

using EntityID = uint32_t;
using Generation = uint32_t;

// entity is essentially just an id
// + generation number (for extra safety when accessing)
struct Entity {
    EntityID id = UINT32_MAX;
    Generation generation = 0;

    bool operator==(const Entity& other) const {
        return id == other.id && generation == other.generation;
    }

    bool operator!=(const Entity& other) const {
        return !(*this == other);
    }
};

// hash function required for unordered_map key
// first time trying to write a custom hash function O_O
namespace std {
    template<>
    struct hash<Entity> {
        size_t operator()(const Entity& e) const {
            return hash<uint32_t>()(e.id) ^ (hash<uint32_t>()(e.generation) << 1);
        }
    };
}

