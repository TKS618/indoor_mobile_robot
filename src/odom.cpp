#include "odom.hpp"
#include <math.h>

Odometry::Odometry()
    : x(0.0f), y(0.0f), theta(0.0f)
{
}

void Odometry::reset(){
    x = 0.0f;
    y = 0.0f;
    theta = 0.0f;
}

void Odometry::update(float omega_r, float omega_l, float dt){
    if(dt <= 0.0f) return;
    float vx  = 0.5f * (WHEEL_RADIUS*(omega_r + omega_l));
    float w_z = (WHEEL_RADIUS*(omega_r - omega_l))*WHEEL_BASE_INV;
    x     += vx*cosf(theta+0.5f*w_z*dt)*dt;
    y     += vx*sinf(theta+0.5f*w_z*dt)*dt;
    theta += w_z*dt;
    
    if(theta > M_PI){
        theta -= 2.0f*M_PI;
    }
    else if(theta < -M_PI){
        theta += 2.0f*M_PI;
    }
}

float Odometry::getX() const{
    return x;
}
float Odometry::getY() const{
    return y;
}
float Odometry::getTheta() const{
    return theta;
}