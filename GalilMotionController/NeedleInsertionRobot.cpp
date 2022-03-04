//
//  NeedleInsertionRobot.cpp
//  GalilMotionController
//
//  Created by Dimitri Lezcano on 3/4/22.
//

#include "NeedleInsertionRobot.h"

NeedleInsertionRobot::NeedleInsertionRobot()
{
    NeedleInsertionRobot(DEFAULT_GALIL_IP);
    
} // default constructor

NeedleInsertionRobot::NeedleInsertionRobot(GCStringIn ipAddress)
{
    m_controller = new GalilController(ipAddress);
    
    // Set speed controls
    setSpeed(s_default_speed);
    setAcceleration(s_default_acceleration);
    setDeceleration(s_default_deceleration);
    
    // set PID controls
    setPID_P(s_default_kP);
    setPID_I(s_default_kI);
    setPID_D(s_default_kD);
    
} // constructor with IP address

NeedleInsertionRobot::~NeedleInsertionRobot()
{
    delete m_controller;

} // destructor

void NeedleInsertionRobot::motorsOn(const bool axes[ROBOT_NUM_AXES])
{
    bool* gc_axes = robotToGalilAxes(axes);
    
    m_controller->motorsOn(gc_axes);
    
} // NeedleInsertionRobot::motorsOn

void NeedleInsertionRobot::motorsOff(const bool axes[ROBOT_NUM_AXES])
{
    bool* gc_axes = robotToGalilAxes(axes);
    
    m_controller->motorsOff(gc_axes);
    
} // NeedleInsertionRobot::motorsOff


/* movement commands */
void NeedleInsertionRobot::moveAxesAbsolute(const float axes[ROBOT_NUM_AXES])
{
    // convert distance measurements to counts
    long* counts_axes = distanceToCounts(axes);
    
    // convert to galil controller axes
    long* gc_axes = robotToGalilAxes(counts_axes);
    
    // send the command
    m_controller->moveAxesAbsolute(gc_axes);
    
    
} // NeedleInsertionRobot::moveAxesAbsolute

void NeedleInsertionRobot::moveAxesRelative(const float axes[ROBOT_NUM_AXES])
{
    // convert distance measurements to counts
    long* counts_axes = distanceToCounts(axes);
    
    // convert to galil controller axes
    long* gc_axes = robotToGalilAxes(counts_axes);
    
    // send the command
    m_controller->moveAxesRelative(gc_axes);
    
} // NeedleInsertionRobot::moveAxesRelative

/* set PID commands */
void NeedleInsertionRobot::setPID_P(const long kp_axes[ROBOT_NUM_AXES])
{
    long* gc_axes = robotToGalilAxes(kp_axes);
    
    m_controller->setPID_P(gc_axes);
    
    
} // NeedleInsertionRobot::setPID_P

void NeedleInsertionRobot::setPID_I(const long ki_axes[ROBOT_NUM_AXES])
{
    long* gc_axes = robotToGalilAxes(ki_axes);
    
    m_controller->setPID_I(gc_axes);
    
    
} // NeedleInsertionRobot::setPID_I

void NeedleInsertionRobot::setPID_D(const long kd_axes[ROBOT_NUM_AXES])
{
    long* gc_axes = robotToGalilAxes(kd_axes);
    
    m_controller->setPID_D(gc_axes);
    
    
} // NeedleInsertionRobot::setPID_D


/* set speed commands */
void NeedleInsertionRobot::setAcceleration(const float ac_axes[ROBOT_NUM_AXES])
{
    long* l_ac_axes = distanceToCounts(ac_axes); // convert to encoder counts
    
    long* gc_axes = robotToGalilAxes(l_ac_axes); // convert to galil axes format
    
    m_controller->setAcceleration(gc_axes);
    
} //NeedleInsertionRobot::setAcceleration

void NeedleInsertionRobot::setDeceleration(const float dc_axes[ROBOT_NUM_AXES])
{
    long* l_dc_axes = distanceToCounts(dc_axes); // convert to encoder counts
    
    long* gc_axes = robotToGalilAxes(l_dc_axes); // convert to galil axes format
    
    m_controller->setDeceleration(gc_axes);
    
} //NeedleInsertionRobot::setDeceleration

void NeedleInsertionRobot::setSpeed(const float sp_axes[ROBOT_NUM_AXES])
{
    long* l_sp_axes = distanceToCounts(sp_axes); // convert to encoder counts
    
    long* gc_axes = robotToGalilAxes(l_sp_axes); // convert to galil axes format
    
    m_controller->setSpeed(gc_axes);
    
} //NeedleInsertionRobot::setSpeed

/* Axes commands */
void NeedleInsertionRobot::stopAxes(const bool axes[ROBOT_NUM_AXES])
{
    bool* gc_axes = robotToGalilAxes(axes);
    
    m_controller->stopAxes(gc_axes);
    
} // NeedleInsertionRobot::stopAxes


void NeedleInsertionRobot::zeroAxes(const bool axes[ROBOT_NUM_AXES])
{
    bool* gc_axes = robotToGalilAxes(axes);
    
    m_controller->zeroAxes(gc_axes);
    
} // NeedleInsertionRobot::zeroAxes




