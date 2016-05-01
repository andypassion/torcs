/***************************************************************************
 *
 *     file                 : driver.cpp
 *     created              : Sun May 1 04:23:30 PST 2016
 *     copyright            : (C) 2016 Xiao Zhang
 *
 ****************************************************************************/

/***************************************************************************
 *                                                                           *
 *     This program is free software; you can redistribute it and/or modify  *
 *     it under the terms of the GNU General Public License as published by  *
 *     the Free Software Foundation; either version 2 of the License, or     *
 *     (at your option) any later version.                                   *
 *                                                                           *
 ****************************************************************************/

#include "driver.h"

const float Driver::MAX_UNSTUCK_ANGLE = 30.0/180.0*PI;  /* [radians] */
const float Driver::UNSTUCK_TIME_LIMIT = 2.0;           /* [s] */
const float Driver::MAX_UNSTUCK_SPEED = 5.0;            /* [m/s] */
const float Driver::MIN_UNSTUCK_DIST = 3.0;             /* [m] */

Driver::Driver(int index)
{
    INDEX = index;
}

/* Called for every track change or new race. */
void Driver::initTrack(tTrack* t, void *carHandle, 
                       void **carParmHandle, tSituation *s)
{
    track = t;
    *carParmHandle = NULL;
}

/* Start a new race. */
void Driver::newRace(tCarElt* car, tSituation *s)
{
    MAX_UNSTUCK_COUNT = int(UNSTUCK_TIME_LIMIT/RCM_MAX_DT_ROBOTS);
    stuck = 0;
}

/* Drive during race. */
void Driver::drive(tCarElt* car, tSituation *s)
{
    update(car, s);

    memset(&car->ctrl, 0, sizeof(tCarCtrl));

    if (isStuck(car)) {
        car->ctrl.steer = -angle / car->_steerLock;
        car->ctrl.gear = -1; // reverse gear
        car->ctrl.accelCmd = 0.3; // 30% accelerator pedal
        car->ctrl.brakeCmd = 0.0; // no brakes
    } else {
        float steerangle = angle - car->_trkPos.toMiddle/car->_trkPos.seg->width;

       	car->ctrl.steer = steerangle / car->_steerLock;
        car->ctrl.gear = 1; // first gear
        car->ctrl.accelCmd = 0.3; // 30% accelerator pedal
	car->ctrl.brakeCmd = 0.0; // no brakes
    }
}

/* Set pitstop commands. */
int Driver::pitCommand(tCarElt* car, tSituation *s)
{
    return ROB_PIT_IM; /* return immediately */
}

/* End of the current race */
void Driver::endRace(tCarElt *car, tSituation *s)
{
}

/* Update my private data every timestep */
void Driver::update(tCarElt* car, tSituation *s)
{
    trackangle = RtTrackSideTgAngleL(&(car->_trkPos));
    angle = trackangle - car->_yaw;
    NORM_PI_PI(angle);
}

/* Check if I'm stuck */
bool Driver::isStuck(tCarElt* car)
{
    if (fabs(angle) > MAX_UNSTUCK_ANGLE &&
        car->_speed_x < MAX_UNSTUCK_SPEED &&
        fabs(car->_trkPos.toMiddle) > MIN_UNSTUCK_DIST) {
	// Car->_trkPos.toMiddle > 0 means car is right of the middle line.
	// Angle positive means car is driving towards the right of the middle line.
	// So if they have different sign, the car is trying to drive towards the
	// center of the lane, which is fine.
        if (stuck > MAX_UNSTUCK_COUNT && car->_trkPos.toMiddle*angle < 0.0) {
            return true;
        } else {
            stuck++;
            return false;
        }
    } else {
        stuck = 0;
        return false;
    }
}

