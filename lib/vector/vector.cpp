#include "vector.h"
Vector operator-(const Vector& right)
{
    return Vector(-right.x, -right.y);
}

Vector& operator+=(Vector& left, const Vector& right)
{
    left.x += right.x;
    left.y += right.y;

    return left;
}

Vector& operator-=(Vector& left, const Vector& right)
{
    left.x -= right.x;
    left.y -= right.y;

    return left;
}

Vector operator+(const Vector& left, const Vector& right)
{
    return Vector(left.x + right.x, left.y + right.y);
}

Vector operator-(const Vector& left, const Vector& right)
{
    return Vector(left.x - right.x, left.y - right.y);
}

Vector operator*(const Vector& left, const Vector& right)
{
    return Vector(left.x * right.x, left.y * right.y);
}

Vector operator*(const Vector& left, float right)
{
    return Vector(left.x * right, left.y * right);
}

Vector operator*(float left, const Vector& right)
{
    return Vector(right.x * left, right.y * left);
}

Vector& operator*=(Vector& left, float right)
{
    left.x *= right;
    left.y *= right;

    return left;
}
