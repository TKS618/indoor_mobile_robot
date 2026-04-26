#pragma once
#include "config.hpp"

class Odometry{
public:
    Odometry();
    void reset();
    void update(float omega_r, float omega_l, float dt);
    float getX() const;
    float getY() const;
    float getTheta() const;

private:
    float x;
    float y;
    float theta;
};