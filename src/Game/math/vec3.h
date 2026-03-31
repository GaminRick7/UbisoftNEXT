#pragma once
#include <valarray>
#include <cmath>

struct Vec3 {
    float x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(float x, float y, float z) : x(x), y(y), z(z) {}

    inline Vec3 operator+(const Vec3& o) const;
    inline Vec3 operator-(const Vec3& o) const;
    inline Vec3 operator*(float o) const;
    inline bool operator==(const Vec3& o) const;
    inline bool operator!=(const Vec3& o) const;

    inline float dot(const Vec3& v) const;
    inline Vec3 cross(const Vec3& o) const;
    inline float length() const;
    inline Vec3 normalized() const;

    inline Vec3& normalize();
};

Vec3 Vec3::operator+(const Vec3& o) const {
    return {x + o.x, y + o.y, z + o.z};
}

Vec3 Vec3::operator-(const Vec3& o) const {
    return {x - o.x, y - o.y, z - o.z};
}

Vec3 Vec3::operator*(float o) const {
    return {x * o, y * o, z * o};
}

bool Vec3::operator==(const Vec3& o) const {
    const float eps = 0.0001f;
    return std::fabs(x - o.x) < eps
        && std::fabs(y - o.y) < eps
        && std::fabs(z - o.z) < eps;
}

bool Vec3::operator!=(const Vec3& o) const {
    return !(*this == o);
}

float Vec3::dot(const Vec3& v) const {
    return x * v.x + y * v.y + z * v.z;
}

inline Vec3 Vec3::cross(const Vec3& o) const {
    return {y * o.z - z * o.y,
                z * o.x - x * o.z,
                x * o.y - y * o.x};
}

float Vec3::length() const {
    return std::sqrt(x * x + y * y + z * z);
}

Vec3 Vec3::normalized() const {
    const float len = length();
    if (len == 0) {
        return {0, 0, 0};
    }
    return (*this) * (1.0f/len);
}

Vec3& Vec3::normalize() {
    (*this) = normalized();
    return (*this);
}