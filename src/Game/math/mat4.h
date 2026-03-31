#pragma once
#include <iostream>
#include <cmath>
#include <cstring>
#include <stdexcept>

#include "vec3.h"
#include "vec4.h"

struct Mat4 {
    float m[16];

    inline Mat4();

    // static factory transformation functions that return a new matrix
    inline static Mat4 translation(const Vec3&);
    inline static Mat4 scaling(const Vec3&);
    inline static Mat4 rotation(const Vec3&);

    // static factory camera functions that return a new matrix
    inline static Mat4 perspective(float fovX, float aspectRatio, float zNear, float zFar);
    inline static Mat4 lookAt(const Vec3& pos, const Vec3& target, const Vec3& up);

    // mutating functions that apply transformations onto this matrix!
    inline Mat4& translate(const Vec3&);
    inline Mat4& scale(const Vec3&);
    inline Mat4& rotate(const Vec3&); // rotates based on ZYX (yaw -> pitch -> roll)

    // operators
    inline float& operator[](size_t i);
    inline const float& operator[](size_t i) const;
    inline Mat4 operator*(const Mat4& b) const;
    inline Vec4 operator*(const Vec4& v) const;
};

Mat4::Mat4() : m() {
    m[0] = m[5] = m[10] = m[15] = 1.0f;
}


Mat4 Mat4::translation(const Vec3& v) {
    Mat4 r{};
    r[12] = v.x;
    r[13] = v.y;
    r[14] = v.z;

    return r;
}

Mat4 Mat4::scaling(const Vec3& v) {
    Mat4 r{};
    r[0]  = v.x;
    r[5]  = v.y;
    r[10] = v.z;
    r[15] = 1.0f;
    return r;
}

Mat4 Mat4::rotation(const Vec3& euler) {
    // pitch = X, yaw = Y, roll = Z
    const float cx = std::cosf(euler.x);
    const float sx = std::sinf(euler.x);
    const float cy = std::cosf(euler.y);
    const float sy = std::sinf(euler.y);
    const float cz = std::cosf(euler.z);
    const float sz = std::sinf(euler.z);

    Mat4 m;

    m[0]  = cy * cz;
    m[1]  = cy * sz;
    m[2]  = -sy;
    m[3]  = 0.0f;

    m[4]  = sx * sy * cz - cx * sz;
    m[5]  = sx * sy * sz + cx * cz;
    m[6]  = sx * cy;
    m[7]  = 0.0f;

    m[8]  = cx * sy * cz + sx * sz;
    m[9]  = cx * sy * sz - sx * cz;
    m[10] = cx * cy;
    m[11] = 0.0f;

    m[12] = 0.0f;
    m[13] = 0.0f;
    m[14] = 0.0f;
    m[15] = 1.0f;

    return m;
}

Mat4 Mat4::perspective(float fovX, float aspectRatio, float zNear, float zFar) {
    const float DEG2RAD = std::acos(-1.0f) / 180;

    float tangent = std::tan(fovX/2 * DEG2RAD);
    float right = zNear * tangent;
    float top = right / aspectRatio;

    Mat4 r{};
    r[0]  =  zNear / right;
    r[5]  =  zNear / top;
    r[10] = -(zFar + zNear) / (zFar - zNear);
    r[11] = -1;
    r[14] = -(2 * zFar * zNear) / (zFar - zNear);
    r[15] =  0;
    return r;
}

Mat4 Mat4::lookAt(const Vec3& pos, const Vec3& target, const Vec3& up) {
    const Vec3 forward = (target - pos).normalized();
    const Vec3 right = forward.cross(up).normalized();
    const Vec3 trueUp = right.cross(forward);

    Mat4 result {};

    result[0] = right.x;
    result[1] = trueUp.x;
    result[2] = -forward.x;
    result[3] = 0.0f;

    result[4] = right.y;
    result[5] = trueUp.y;
    result[6] = -forward.y;
    result[7] = 0.0f;

    result[8]  = right.z;
    result[9]  = trueUp.z;
    result[10] = -forward.z;
    result[11] = 0.0f;

    result[12] = -right.dot(pos);
    result[13] = -trueUp.dot(pos);
    result[14] = forward.dot(pos);
    result[15] = 1.0f;

    return result;
}

Mat4& Mat4::translate(const Vec3& v) {
    const Mat4 translated = (*this) * translation(v);
    std::memcpy(m, translated.m, sizeof(translated.m));
    return (*this);
}

Mat4& Mat4::scale(const Vec3& v) {
    const Mat4 scaled = (*this) * scaling(v);
    std::memcpy(m, scaled.m, sizeof(scaled.m));
    return (*this);
}

Mat4& Mat4::rotate(const Vec3& euler) {
    const Mat4 rotated = (*this) * rotation(euler);
    std::memcpy(m, rotated.m, sizeof(rotated.m));
    return (*this);
}

float& Mat4::operator[](size_t i) {
    if (i >= 16)
        throw std::out_of_range("Mat4 index out of range");
    return m[i];
}

const float& Mat4::operator[](size_t i) const {
    if (i >= 16)
        throw std::out_of_range("Mat4 index out of range");
    return m[i];
}

Mat4 Mat4::operator*(const Mat4& b) const {
    Mat4 r{};
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            r[col*4 + row] =
                m[0*4 + row] * b[col*4 + 0] +
                m[1*4 + row] * b[col*4 + 1] +
                m[2*4 + row] * b[col*4 + 2] +
                m[3*4 + row] * b[col*4 + 3];
        }
    }
    return r;
}

Vec4 Mat4::operator*(const Vec4& v) const {
    return {
        m[0] * v.x + m[4] * v.y + m[8]  * v.z + m[12] * v.w,
        m[1] * v.x + m[5] * v.y + m[9]  * v.z + m[13] * v.w,
        m[2] * v.x + m[6] * v.y + m[10] * v.z + m[14] * v.w,
        m[3] * v.x + m[7] * v.y + m[11] * v.z + m[15] * v.w
    };
}
