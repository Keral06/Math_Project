#pragma once
#include "Matrix3x3.hpp"

struct Quat 
{
    double s = 1, x = 0, y = 0, z = 0;

    Quat Normalized() const;
    Quat Multiply(const Quat& b) const;
    Quat operator*(const Quat& b) const
    {
        return Multiply(b);
	}

    Vec3 Rotate(const Vec3& v) const;

    static Quat FromMatrix3x3(const Matrix3x3& R);
    Matrix3x3 ToMatrix3x3() const;

    static Quat FromAxisAngle(const Vec3& u, double phi);
    void ToAxisAngle(Vec3& axis, double& angle) const;

    static Quat FromEulerZYX(double yaw, double pitch, double roll);
    void ToEulerZYX(double& yaw, double& pitch, double& roll) const;

    static Quat RotateFromTo(const Vec3& u, const Vec3& v);
    static Quat RotateToTarget(const Quat& initialRot, const Quat& finalRot);
};