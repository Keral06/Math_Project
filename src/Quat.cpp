#include "Quat.hpp"
#include <stdexcept>

#define TOL 1e-6
#define PI 3.14159265358979323846

Quat Quat::FromAxisAngle(const Vec3& u_in, double phi)
{
    Vec3 u = u_in.Normalize();
    double half = 0.5 * phi;
    double c = std::cos(half);
    double s = std::sin(half);
    return Quat{ c, u.x * s, u.y * s, u.z * s };
}

Quat Quat::Normalized() const
{
    double n2 = s * s + x * x + y * y + z * z;
    double n = std::sqrt(n2);
    if (n == 0) throw std::invalid_argument("Quat::Normalized: zero norm");
    return { s / n, x / n, y / n, z / n };
}

Quat Quat::Multiply(const Quat& b) const
{
    const Quat& a = *this;
    Quat q;
    q.s = a.s * b.s - a.x * b.x - a.y * b.y - a.z * b.z;
    q.x = a.s * b.x + a.x * b.s + a.y * b.z - a.z * b.y;
    q.y = a.s * b.y - a.x * b.z + a.y * b.s + a.z * b.x;
    q.z = a.s * b.z + a.x * b.y - a.y * b.x + a.z * b.s;
    return q;
}

Vec3 Quat::Rotate(const Vec3& v) const
{
    Vec3 qv{ x, y, z };
    Vec3 t = Vec3::Cross(qv, v);
    t.x *= 2.0; t.y *= 2.0; t.z *= 2.0;
    Vec3 st{ s * t.x, s * t.y, s * t.z };
    Vec3 cqt = Vec3::Cross(qv, t);
    Vec3 w{ v.x + st.x + cqt.x, v.y + st.y + cqt.y, v.z + st.z + cqt.z };
    return w;
}

Matrix3x3 Quat::ToMatrix3x3() const
{
    Quat q = this->Normalized();
    const double ww = q.s, xx = q.x, yy = q.y, zz = q.z;

    Matrix3x3 R{};
    const double xx2 = xx * xx, yy2 = yy * yy, zz2 = zz * zz;
    const double xy2 = xx * yy, xz2 = xx * zz, yz2 = yy * zz;
    const double sx2 = ww * xx, sy2 = ww * yy, sz2 = ww * zz;

    R.At(0, 0) = 1.0 - 2.0 * (yy2 + zz2);
    R.At(0, 1) = 2.0 * (xy2 - sz2);
    R.At(0, 2) = 2.0 * (xz2 + sy2);

    R.At(1, 0) = 2.0 * (xy2 + sz2);
    R.At(1, 1) = 1.0 - 2.0 * (xx2 + zz2);
    R.At(1, 2) = 2.0 * (yz2 - sx2);

    R.At(2, 0) = 2.0 * (xz2 - sy2);
    R.At(2, 1) = 2.0 * (yz2 + sx2);
    R.At(2, 2) = 1.0 - 2.0 * (xx2 + yy2);
    return R;
}

Quat Quat::FromMatrix3x3(const Matrix3x3& R)
{
    if (!R.IsRotation()) throw std::invalid_argument("FromMatrix3x3: input not rotation");

    Quat q;
    double tr = R.At(0, 0) + R.At(1, 1) + R.At(2, 2);

    if (tr > 0.0)
    {
        double S = std::sqrt(tr + 1.0) * 2.0;
        q.s = 0.25 * S;
        q.x = (R.At(2, 1) - R.At(1, 2)) / S;
        q.y = (R.At(0, 2) - R.At(2, 0)) / S;
        q.z = (R.At(1, 0) - R.At(0, 1)) / S;
    }
    else if (R.At(0, 0) > R.At(1, 1) && R.At(0, 0) > R.At(2, 2))
    {
        double S = std::sqrt(1.0 + R.At(0, 0) - R.At(1, 1) - R.At(2, 2)) * 2.0;
        q.s = (R.At(2, 1) - R.At(1, 2)) / S;
        q.x = 0.25 * S;
        q.y = (R.At(0, 1) + R.At(1, 0)) / S;
        q.z = (R.At(0, 2) + R.At(2, 0)) / S;
    }
    else if (R.At(1, 1) > R.At(2, 2))
    {
        double S = std::sqrt(1.0 - R.At(0, 0) + R.At(1, 1) - R.At(2, 2)) * 2.0;
        q.s = (R.At(0, 2) - R.At(2, 0)) / S;
        q.x = (R.At(0, 1) + R.At(1, 0)) / S;
        q.y = 0.25 * S;
        q.z = (R.At(1, 2) + R.At(2, 1)) / S;
    }
    else
    {
        double S = std::sqrt(1.0 - R.At(0, 0) - R.At(1, 1) + R.At(2, 2)) * 2.0;
        q.s = (R.At(1, 0) - R.At(0, 1)) / S;
        q.x = (R.At(0, 2) + R.At(2, 0)) / S;
        q.y = (R.At(1, 2) + R.At(2, 1)) / S;
        q.z = 0.25 * S;
    }

    return q.Normalized();
}

void Quat::ToAxisAngle(Vec3& axis, double& angle) const
{
    Quat q = this->Normalized();

    angle = 2.0 * std::acos(q.s);

    double sin_half = std::sqrt(std::max(0.0, 1.0 - q.s * q.s));

    if (sin_half < TOL)
    {
        axis = { 1, 0, 0 };
        angle = 0.0;
        return;
    }

    axis = { q.x / sin_half, q.y / sin_half, q.z / sin_half };
    axis = axis.Normalize();
}

Quat Quat::RotateFromTo(const Vec3& u, const Vec3& v)
{
    Vec3 a{ u.Normalize()};
    Vec3 b{ v.Normalize()};

    double dot = Vec3::Dot(a, b);

    if (std::fabs(dot - 1.0) < TOL) {
        return Quat{}; // (1,0,0,0)
    }

    if (std::fabs(dot + 1.0) < TOL) {
        Vec3 arbitrary = (std::fabs(a.x) < 0.9) ? Vec3{ 1,0,0 } : Vec3{ 0,1,0 };
        Vec3 axis = Vec3::Cross(a, arbitrary).Normalize();
        return FromAxisAngle(axis, PI);
    }

    Vec3 axis = Vec3::Cross(a, b).Normalize();
    double angle = std::acos(dot);
    return FromAxisAngle(axis, angle);
}

Quat Quat::RotateToTarget(const Quat& initialRot, const Quat& finalRot)
{
    Quat qi = initialRot.Normalized();
    Quat qf = finalRot.Normalized();

    Quat qi_conj{ qi.s, -qi.x, -qi.y, -qi.z };

    Quat qdelta = qf.Multiply(qi_conj);
    return qdelta.Normalized();
}

Quat Quat::FromEulerZYX(double yaw, double pitch, double roll)
{
    Matrix3x3 R = Matrix3x3::FromEulerZYX(yaw, pitch, roll);
    return FromMatrix3x3(R);
}

void Quat::ToEulerZYX(double& yaw, double& pitch, double& roll) const
{
    Matrix3x3 R = ToMatrix3x3();
    R.ToEulerZYX(yaw, pitch, roll);
}