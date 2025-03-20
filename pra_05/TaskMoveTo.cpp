/*
 * TaskMoveTo.cpp
 * Copyright (c) 2025, ZHAW
 * All rights reserved.
 */

#include <cmath>
#include "TaskMoveTo.h"

using namespace std;

const float TaskMoveTo::DEFAULT_VELOCITY = 0.3f;    // default velocity value, given in [m/s]
const float TaskMoveTo::DEFAULT_ZONE = 0.05f;       // default zone value, given in [m]
const float TaskMoveTo::M_PI = 3.14159265f;         // the mathematical constant PI
const float TaskMoveTo::K1 = 2.0f;                  // position controller gain parameter
const float TaskMoveTo::K2 = 2.0f;                  // position controller gain parameter
const float TaskMoveTo::K3 = 1.0f;                  // position controller gain parameter

/**
 * Creates a task object that moves the robot to a given pose.
 * @param conroller a reference to the controller object of the robot.
 * @param x the x coordinate of the target position, given in [m].
 * @param y the y coordinate of the target position, given in [m].
 * @param alpha the target orientation, given in [rad].
 */
TaskMoveTo::TaskMoveTo(Controller& controller, float x, float y, float alpha) : controller(controller) {
    
    this->x = x;
    this->y = y;
    this->alpha = alpha;
    this->velocity = DEFAULT_VELOCITY;
    this->zone = DEFAULT_ZONE;
}

/**
 * Creates a task object that moves the robot to a given pose.
 * @param conroller a reference to the controller object of the robot.
 * @param x the x coordinate of the target position, given in [m].
 * @param y the y coordinate of the target position, given in [m].
 * @param alpha the target orientation, given in [rad].
 * @param velocity the maximum translational velocity, given in [m/s].
 */
TaskMoveTo::TaskMoveTo(Controller& controller, float x, float y, float alpha, float velocity) : controller(controller) {
    
    this->x = x;
    this->y = y;
    this->alpha = alpha;
    this->velocity = velocity;
    this->zone = DEFAULT_ZONE;
}

/**
 * Creates a task object that moves the robot to a given pose.
 * @param conroller a reference to the controller object of the robot.
 * @param x the x coordinate of the target position, given in [m].
 * @param y the y coordinate of the target position, given in [m].
 * @param alpha the target orientation, given in [rad].
 * @param velocity the maximum translational velocity, given in [m/s].
 * @param zone the zone threshold around the target position, given in [m].
 */
TaskMoveTo::TaskMoveTo(Controller& controller, float x, float y, float alpha, float velocity, float zone) : controller(controller) {
    
    this->x = x;
    this->y = y;
    this->alpha = alpha;
    this->velocity = velocity;
    this->zone = zone;
}

/**
 * Deletes the task object.
 */
TaskMoveTo::~TaskMoveTo() {}

/**
 * This method is called periodically by a task sequencer.
 * @param period the period of the task sequencer, given in [s].
 * @return the status of this task, i.e. RUNNING or DONE.
 */
int TaskMoveTo::run(float period) {
    float currentX = controller.getX();
    float currentY = controller.getY();
    float currentAlpha = controller.getAlpha();

    float targetX = this->x;
    float targetY = this->y;
    float targetAlpha = this->alpha;
    
    // rho
    float rho = sqrt(pow(targetX-currentX, 2) + pow(targetY-currentY, 2));

    if(rho < this->zone)
    {
        controller.setRotationalVelocity(0);
        controller.setTranslationalVelocity(0);
        return DONE;
    }

    // gamma
    float gamma = atan2(targetY - currentY, targetX - currentX) - currentAlpha;
    
    if(gamma < -M_PI)
        gamma += 2 * M_PI;

    if(gamma > M_PI)
        gamma -= 2 * M_PI;

    // delta
    float delta = gamma + currentAlpha - targetAlpha;

    if(delta < -M_PI)
        delta += 2 * M_PI;

    if(delta > M_PI)
        delta -= 2 * M_PI;


    float tv = K1 * rho * cos(gamma);
    float av = K2 * gamma + K1 * sin(gamma) * cos(gamma) * (gamma + K3 * rho) / gamma;


    if(gamma != 0){
        controller.setRotationalVelocity(av);
        controller.setTranslationalVelocity(tv);
    }else{
        controller.setRotationalVelocity(0);
        controller.setTranslationalVelocity(tv);
    }

    return RUNNING;
}
