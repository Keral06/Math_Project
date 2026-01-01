#include "Matrix3x3.hpp"
#include <stdexcept>

#define TOL 1e-6
#define PI 3.14159265358979323846

// ------------------ Vec3 -------------------------

double Vec3::Dot(const Vec3& a, const Vec3& b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3 Vec3::Cross(const Vec3& a, const Vec3& b)
{
    return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
}

double Vec3::Norm() const
{
    return std::sqrt(Dot(*this, *this));
}

Vec3 Vec3::Normalize() const
{
    double n = Norm();
    if (n == 0) throw std::invalid_argument("normalize: zero vector");
    return { x / n, y / n, z / n };
}

// ------------------ Matrix3x3 ---------------------

Matrix3x3 Matrix3x3::Identity()
{
    Matrix3x3 I;
    I.At(0, 0) = 1; I.At(1, 1) = 1; I.At(2, 2) = 1;
    return I;
}

Vec3 Matrix3x3::Multiply(const Vec3& x) const
{
    // y = A * x
    Vec3 y;
    y.x = At(0, 0) * x.x + At(0, 1) * x.y + At(0, 2) * x.z;
    y.y = At(1, 0) * x.x + At(1, 1) * x.y + At(1, 2) * x.z;
    y.z = At(2, 0) * x.x + At(2, 1) * x.y + At(2, 2) * x.z;
    return y;
}

Matrix3x3 Matrix3x3::Multiply(const Matrix3x3& B) const
{
    Matrix3x3 C{};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            double s = 0.0;
            for (int k = 0; k < 3; ++k) {
                s += At(i, k) * B.At(k, j);
            }
            C.At(i, j) = s;
        }
    }
    return C;
}

double Matrix3x3::Det() const
{
    const double a = At(0, 0), b = At(0, 1), c = At(0, 2);
    const double d = At(1, 0), e = At(1, 1), f = At(1, 2);
    const double g = At(2, 0), h = At(2, 1), i = At(2, 2);
    return a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
}

Matrix3x3 Matrix3x3::Transposed() const
{
    Matrix3x3 R{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            R.At(i, j) = At(j, i);
    return R;
}

double Matrix3x3::Trace() const
{
    return At(0, 0) + At(1, 1) + At(2, 2);
}


Matrix3x3 Matrix3x3::RotationAxisAngle(const Vec3& u_in, double phi)
{
    Vec3 u = u_in.Normalize();
    const double c = std::cos(phi);
    const double s = std::sin(phi);
    const double t = 1.0 - c;

    const double ux = u.x, uy = u.y, uz = u.z;

    Matrix3x3 R{};
    R.At(0, 0) = c + t * ux * ux;
    R.At(0, 1) = t * ux * uy - s * uz;
    R.At(0, 2) = t * ux * uz + s * uy;

    R.At(1, 0) = t * uy * ux + s * uz;
    R.At(1, 1) = c + t * uy * uy;
    R.At(1, 2) = t * uy * uz - s * ux;

    R.At(2, 0) = t * uz * ux - s * uy;
    R.At(2, 1) = t * uz * uy + s * ux;
    R.At(2, 2) = c + t * uz * uz;
    return R;
}

bool Matrix3x3::IsRotation() const
{
    Matrix3x3 Rt = this->Transposed();
    Matrix3x3 RtR = Rt.Multiply(*this);
    Matrix3x3 I = Identity();

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            if (std::fabs(RtR.At(i, j) - I.At(i, j)) > TOL) return false;
        }
    }

    if (std::fabs(Det() - 1.0) > TOL) return false;

    return true;
}

Vec3 Matrix3x3::Rotate(const Vec3& v) const
{
    return Multiply(v);
}

void Matrix3x3::ToAxisAngle(Vec3& axis, double& angle) const
{
    if (!IsRotation()) throw std::invalid_argument("ToAxisAngle: matrix is not a rotation");

    double tr = Trace();
    double cos_a = (tr - 1.0) * 0.5;
    angle = std::acos(cos_a);

    if (std::fabs(angle) < TOL)
    {
        axis = { 1,0,0 };
        return;
    }

    if (std::fabs(PI - angle) < TOL)
    {
        double xx = (At(0, 0) + 1.0) * 0.5;
        double yy = (At(1, 1) + 1.0) * 0.5;
        double zz = (At(2, 2) + 1.0) * 0.5;
        double x = std::sqrt(xx);
        double y = std::sqrt(yy);
        double z = std::sqrt(zz);

        if (At(0, 1) + At(1, 0) < 0.0) y = -y;
        if (At(0, 2) + At(2, 0) < 0.0) z = -z;

        axis = Vec3{ x, y, z }.Normalize();

        return;
    }

    double denom = 2.0 * std::sin(angle);
    axis.x = (At(2, 1) - At(1, 2)) / denom;
    axis.y = (At(0, 2) - At(2, 0)) / denom;
    axis.z = (At(1, 0) - At(0, 1)) / denom;
    axis = axis.Normalize();
}

Matrix3x3 Matrix3x3::FromEulerZYX(double yaw, double pitch, double roll)
{
    const double cy = std::cos(yaw), sy = std::sin(yaw);
    const double cp = std::cos(pitch), sp = std::sin(pitch);
    const double cr = std::cos(roll), sr = std::sin(roll);

    Matrix3x3 R{};
    R.At(0, 0) = cy * cp;
    R.At(0, 1) = cy * sp * sr - sy * cr;
    R.At(0, 2) = cy * sp * cr + sy * sr;

    R.At(1, 0) = sy * cp;
    R.At(1, 1) = sy * sp * sr + cy * cr;
    R.At(1, 2) = sy * sp * cr - cy * sr;

    R.At(2, 0) = -sp;
    R.At(2, 1) = cp * sr;
    R.At(2, 2) = cp * cr;
    return R;
}

void Matrix3x3::ToEulerZYX(double& yaw, double& pitch, double& roll) const
{
    double r20 = At(2, 0);

    if (std::fabs(r20) < 1.0 - TOL)
    {
        pitch = std::asin(-r20);
        yaw = std::atan2(At(1, 0), At(0, 0));
        roll = std::atan2(At(2, 1), At(2, 2));
    }
    else
    {
        pitch = (r20 < 0.0) ? +PI / 2 : -PI / 2;
        yaw = std::atan2(-At(0, 1), At(1, 1));
        roll = 0.0;
    }
}

Matrix3x3 Matrix3x3::RotateFromTo(const Vec3& u, const Vec3& v)
{
    Vec3 a{ u.Normalize()};
    Vec3 b{ v.Normalize()};

    double dot = Vec3::Dot(a, b);

    if (std::fabs(dot - 1.0) < TOL) 
    {
        return Identity();
    }

    if (std::fabs(dot + 1.0) < TOL) 
    {
        Vec3 arbitrary = (std::fabs(a.x) < 0.9) ? Vec3{ 1,0,0 } : Vec3{ 0,1,0 };
        Vec3 axis = Vec3::Cross(a, arbitrary).Normalize();
        return RotationAxisAngle(axis, PI);
    }

    Vec3 axis = Vec3::Cross(a, b).Normalize();
    double angle = std::acos(dot);
    return RotationAxisAngle(axis, angle);
}

Matrix3x3 Matrix3x3::RotateToTarget(const Matrix3x3& initialRot, const Matrix3x3& finalRot)
{
    if (!initialRot.IsRotation())
        throw std::invalid_argument("RotateToTarget: initialRot is not a rotation");
    if (!finalRot.IsRotation())
        throw std::invalid_argument("RotateToTarget: finalRot is not a rotation");

    Matrix3x3 RiT = initialRot.Transposed();
    Matrix3x3 Rdelta = finalRot.Multiply(RiT);
    return Rdelta;
}
