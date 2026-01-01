#pragma once
#include "Matrix3x3.hpp"
#include "Quat.hpp"
#include <iostream>

struct Vec4
{
    double x = 0, y = 0, z = 0, w = 0;

    Vec4() = default;
    Vec4(double _x, double _y, double _z, double _w) : x(_x), y(_y), z(_z), w(_w) {}
    Vec4(const Vec3& v, double _w) : x(v.x), y(v.y), z(v.z), w(_w) {}
};

struct Matrix4x4
{
    // Row-major: m[row * 4 + col]
    double m[16] = { 0 };

    static Matrix4x4 Identity();
    double& At(std::size_t i, std::size_t j) { return m[i * 4 + j]; }
    double  At(std::size_t i, std::size_t j) const { return m[i * 4 + j]; }

    Matrix4x4 Multiply(const Matrix4x4& B) const;
    Vec4 Multiply(const Vec4& v) const;

    bool IsAffine() const;
	
    // Transformacions de punts i vectors
	Vec3 TransformPoint(const Vec3& p) const;
	Vec3 TransformVector(const Vec3& v) const;

    // Statics
    static Matrix4x4 Translate(const Vec3& t);
    static Matrix4x4 Scale(const Vec3& s);
    static Matrix4x4 Rotate(const Matrix3x3& R);
    static Matrix4x4 Rotate(const Quat& q);
    static Matrix4x4 FromTRS(const Vec3& t, const Matrix3x3& R, const Vec3& s);
    static Matrix4x4 FromTRS(const Vec3& t, const Quat& q, const Vec3& s);

	// Inverses
    Matrix4x4 InverseTR() const;
	Matrix4x4 InverseTRS() const;

    // Getters de components
    Vec3 GetTranslation() const;
	Matrix3x3 GetRotation() const;
	Quat GetRotationQuat() const;
	Vec3 GetScale() const;
    Matrix3x3 GetRotationScale() const;

	// Setters de components
	void SetTranslation(const Vec3& t);
	void SetRotation(const Matrix3x3& R);
	void SetRotation(const Quat& q);
	void SetScale(const Vec3& s);
	void SetRotationScale(const Matrix3x3& RS);
};