#include "Matrix4x4.hpp"
#include <cmath>
#include <stdexcept>

#define TOL 1e-6

Matrix4x4 Matrix4x4::Identity()
{
    Matrix4x4 I;
    I.At(0, 0) = 1; I.At(1, 1) = 1; I.At(2, 2) = 1; I.At(3, 3) = 1;
    return I;
}

Matrix4x4 Matrix4x4::Multiply(const Matrix4x4& B) const
{
    Matrix4x4 C{};
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            double sum = 0.0;
            for (int k = 0; k < 4; ++k) {
                sum += At(i, k) * B.At(k, j);
            }
            C.At(i, j) = sum;
        }
    }
    return C;
}

Vec4 Matrix4x4::Multiply(const Vec4& v) const
{
    Vec4 res;
    res.x = At(0, 0) * v.x + At(0, 1) * v.y + At(0, 2) * v.z + At(0, 3) * v.w;
    res.y = At(1, 0) * v.x + At(1, 1) * v.y + At(1, 2) * v.z + At(1, 3) * v.w;
    res.z = At(2, 0) * v.x + At(2, 1) * v.y + At(2, 2) * v.z + At(2, 3) * v.w;
    res.w = At(3, 0) * v.x + At(3, 1) * v.y + At(3, 2) * v.z + At(3, 3) * v.w;
    return res;
}

// --------------------------------------------------------------------------
// TODO LAB 3
// --------------------------------------------------------------------------

bool Matrix4x4::IsAffine() const
{
    if (std::abs(At(3, 0)) > TOL) {
        return false;
    }
    if (std::abs(At(3, 1)) > TOL) {
        return false;
    }
    if (std::abs(At(3, 2)) > TOL) {
        return false;
    }
    if (std::abs(At(3, 3) - 1) > TOL) {
        return false;
    }
    else {
        return true;
    }
}

Vec3 Matrix4x4::TransformPoint(const Vec3& p) const
{
    Vec4 v4(p.x, p.y, p.z, 1.0f);
    Vec4 res = Multiply(v4);
    if (std::abs(res.w) > TOL && std::abs(res.w - 1.0f) > TOL) {
        float div = 1.0f / res.w;
        return Vec3(res.x * div, res.y * div, res.z * div);
    }
    Vec3 result = Vec3(res.x, res.y, res.z);
    return result;
}

Vec3 Matrix4x4::TransformVector(const Vec3& v) const
{
    Vec4 v4(v.x, v.y, v.z, 0.0f);

    Vec4 result = this->Multiply(v4);

    return Vec3(result.x, result.y, result.z);
}

Matrix4x4 Matrix4x4::Translate(const Vec3& t)
{
    Matrix4x4 M = Matrix4x4::Identity();

    M.At(0,3) = t.x;
    M.At(1,3) = t.y;
    M.At(2,3) = t.z;

    return M;
}

Matrix4x4 Matrix4x4::Scale(const Vec3& s)
{
    Matrix4x4 M = Matrix4x4::Identity();

    M.At(0, 0) = s.x;  
    M.At(1, 1) = s.y;  
    M.At(2, 2) = s.z;  


    return M;
}

Matrix4x4 Matrix4x4::Rotate(const Matrix3x3& R)
{
    Matrix4x4 M = Matrix4x4::Identity();

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            M.At(i, j) = R.At(i, j);
        }
    }

    return M;
}

Matrix4x4 Matrix4x4::Rotate(const Quat& q)
{
    Matrix4x4 M;
    Matrix3x3 R;
    R = q.ToMatrix3x3();
    M = Rotate(R);

    return M;
}

Matrix4x4 Matrix4x4::FromTRS(const Vec3& t, const Matrix3x3& R, const Vec3& s)
{
    Matrix4x4 M;
    M = Rotate(R);

    float scales[3] = { s.x, s.y, s.z };

    for (int j = 0; j < 3; ++j)
    {
        for (int i = 0; i < 3; ++i)
        {
            M.At(i, j) = M.At(i, j) * scales[j];
        }
    }

    M.At(0, 3) = t.x;
    M.At(1, 3) = t.y;
    M.At(2, 3) = t.z;

    return M;
}

Matrix4x4 Matrix4x4::FromTRS(const Vec3& t, const Quat& q, const Vec3& s)
{
    Matrix4x4 M;
    M = Rotate(q);

    float scales[3] = { s.x, s.y, s.z };

    for (int j = 0; j < 3; ++j)
    {
        for (int i = 0; i < 3; ++i)
        {
            M.At(i, j) = M.At(i, j) * scales[j];
        }
    }

    M.At(0, 3) = t.x;
    M.At(1, 3) = t.y;
    M.At(2, 3) = t.z;

    return M;
}

Matrix4x4 Matrix4x4::InverseTR() const
{
    if (!IsAffine()) {
        throw std::runtime_error("La matriu no és afí");
    }
    Matrix4x4 M;
    Matrix3x3 R;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            R.At(i, j) = At(j, i);
        }
    }

    Vec3 p(At(0, 3), At(1, 3), At(2, 3));
    Vec3 pi;
    pi.x = -(R.At(0, 0) * p.x + R.At(0, 1) * p.y + R.At(0, 2) * p.z);
    pi.y = -(R.At(1, 0) * p.x + R.At(1, 1) * p.y + R.At(1, 2) * p.z);
    pi.z = -(R.At(2, 0) * p.x + R.At(2, 1) * p.y + R.At(2, 2) * p.z);

    M = FromTRS(pi, R, Vec3(1.0f, 1.0f, 1.0f));
    return M;
}

Matrix4x4 Matrix4x4::InverseTRS() const
{
    if (!IsAffine()) {
        throw std::runtime_error("La matriu no és afí");
    }
    Matrix4x4 M; 
    Vec3 s = GetScale();
    Matrix3x3 R = GetRotation();
    Vec3 t = GetTranslation();

    Vec3 inversaesc;
    if (std::abs(s.x) > TOL) {
        inversaesc.x = 1.0 / s.x;
    }
    else {
        inversaesc.x = 0.0;
    }
    if (std::abs(s.y) > TOL) {
        inversaesc.y = 1.0 / s.y;
    }
    else {
        inversaesc.y = 0.0;
    }
    if (std::abs(s.z) > TOL) {
        inversaesc.z = 1.0 / s.z;
    }
    else {
        inversaesc.z = 0.0;
    }

    Matrix3x3 inversarot = R.Transposed();

    Matrix3x3 inversaRS;
    for (int i = 0; i < 3; ++i) {
        inversaRS.At(0, i) = inversarot.At(0, i) * inversaesc.x;
        inversaRS.At(1, i) = inversarot.At(1, i) * inversaesc.y;
        inversaRS.At(2, i) = inversarot.At(2, i) * inversaesc.z;
    }

    Vec3 invTrans = inversaRS.Multiply(t);
    invTrans.x = -invTrans.x;
    invTrans.y = -invTrans.y;
    invTrans.z = -invTrans.z;

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            M.At(i, j) = inversaRS.At(i, j);
        }
    }

    M.At(0, 3) = invTrans.x;
    M.At(1, 3) = invTrans.y;
    M.At(2, 3) = invTrans.z;
    M.At(3, 3) = 1.0;

    return M;
}

Vec3 Matrix4x4::GetTranslation() const
{
    if (!IsAffine()) {
        throw std::runtime_error("La matriu no és afí");
    }
    Vec3 result;
    result = { At(0, 3), At(1, 3), At(2, 3) };
    return result;
}

Matrix3x3 Matrix4x4::GetRotationScale() const
{
    if (!IsAffine()) {
        throw std::runtime_error("La matriu no és afí");
    }
    Matrix3x3 M;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            M.At(i, j) = At(i, j);
        }
    }
    return M;
}

Vec3 Matrix4x4::GetScale() const
{
    if (!IsAffine()) {
        throw std::runtime_error("La matriu no és afí");
    }
    Vec3 X(At(0, 0), At(1, 0), At(2, 0));
    Vec3 Y(At(0, 1), At(1, 1), At(2, 1));
    Vec3 Z(At(0, 2), At(1, 2), At(2, 2));
    Vec3 result(X.Norm(), Y.Norm(), Z.Norm());
	return result;
}

Matrix3x3 Matrix4x4::GetRotation() const
{
    if (!IsAffine()) {
        throw std::runtime_error("La matriu no és afí");
    }
    Matrix3x3 M;
    Vec3 s = GetScale();

    if (std::abs(s.x) > TOL) {
        M.At(0, 0) = At(0, 0) / s.x;
        M.At(1, 0) = At(1, 0) / s.x;
        M.At(2, 0) = At(2, 0) / s.x;
    }

    if (std::abs(s.y) > TOL) {
        M.At(0, 1) = At(0, 1) / s.y;
        M.At(1, 1) = At(1, 1) / s.y;
        M.At(2, 1) = At(2, 1) / s.y;
    }

    if (std::abs(s.z) > TOL) {
        M.At(0, 2) = At(0, 2) / s.z;
        M.At(1, 2) = At(1, 2) / s.z;
        M.At(2, 2) = At(2, 2) / s.z;
    }
    return M;
}

Quat Matrix4x4::GetRotationQuat() const
{
    if (!IsAffine()) {
        throw std::runtime_error("La matriu no és afí");
    }
    Matrix3x3 M;
    Quat R;
    M = GetRotation();
    R = Quat::FromMatrix3x3(M);
	return R;
}

void Matrix4x4::SetTranslation(const Vec3& t)
{
    if (!IsAffine()) {
        throw std::runtime_error("La matriu no és afí");
    }
    At(0, 3) = t.x;
    At(1, 3) = t.y;
    At(2, 3) = t.z;
}

void Matrix4x4::SetScale(const Vec3& s)
{
    if (!IsAffine()) {
        throw std::runtime_error("La matriu no és afí");
    }
    Matrix3x3 M;
    M = GetRotation();

    double escala[3] = { s.x, s.y, s.z };

    for (int j = 0; j < 3; ++j) {
        for (int i = 0; i < 3; ++i) {
            At(i, j) = M.At(i, j) * escala[j];
        }
    }
}

void Matrix4x4::SetRotation(const Matrix3x3& R)
{
    if (!IsAffine()) {
        throw std::runtime_error("La matriu no és afí");
    }
    Vec3 s = GetScale();
    double escala[3] = { s.x, s.y, s.z };

    for (int j = 0; j < 3; ++j) {
        for (int i = 0; i < 3; ++i) {
            At(i, j) = R.At(i, j) * escala[j];
        }
    }
    
}

void Matrix4x4::SetRotation(const Quat& q)
{
    if (!IsAffine()) {
        throw std::runtime_error("La matriu no és afí");
    }
    SetRotation(q.ToMatrix3x3());
}

void Matrix4x4::SetRotationScale(const Matrix3x3& RS)
{
    if (!IsAffine()) {
        throw std::runtime_error("La matriu no és afí");
    }
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            At(i, j) = RS.At(i, j);
        }
    }
}