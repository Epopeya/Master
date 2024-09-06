#pragma once
class Vector {
public:
    Vector(float _x, float _y)
        : x(_x)
        , y(_y)
    {
    }
    Vector()
        : x(0)
        , y(0)
    {
    }
    float x;
    float y;
};

Vector operator-(const Vector& right);
Vector& operator+=(Vector& left, const Vector& right);
Vector& operator-=(Vector& left, const Vector& right);
Vector operator+(const Vector& left, const Vector& right);
Vector operator-(const Vector& left, const Vector& right);
Vector operator*(const Vector& left, const Vector& right);
Vector operator*(const Vector& left, float right);
Vector operator*(float left, const Vector& right);
Vector& operator*=(Vector& left, float right);
