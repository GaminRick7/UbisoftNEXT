#pragma once

struct Vec4 {
    float x, y, z, w;
    Vec4(float x, float y, float z, float w) : x(x), y(y), z(z), w(w){}

    inline Vec4 operator+(const Vec4& v) const;
    inline Vec4 operator-(const Vec4& v) const;
    inline Vec4 operator*(float o) const;

    inline float dot(const Vec4& v) const;
    inline float length() const;
    inline Vec4 normalized() const;

    inline Vec4& normalize();
};

Vec4 Vec4::operator+(const Vec4& v) const {
    return {x + v.x, y + v.y, z + v.z, w + v.w};
}

Vec4 Vec4::operator-(const Vec4& v) const {
    return {x - v.x, y - v.y, z - v.z, w - v.w};
}

Vec4 Vec4::operator*(float o) const {
    return {x * o, y * o, z * o, w * o};
}

float Vec4::dot(const Vec4& v) const {
    return x * v.x + y * v.y + z * v.z + w * v.w;
}

float Vec4::length() const {
    return std::sqrt(x * x + y * y + z * z + w * w);
}

Vec4 Vec4::normalized() const {
    const float len = length();
    if (len == 0) {
        return {0, 0, 0, 0};
    }
    return (*this) * (1.0f/len);
}

Vec4& Vec4::normalize() {
    (*this) = normalized();
    return (*this);
}
