/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Betaflight. If not, see <http://www.gnu.org/licenses/>.
 */

#include "platform.h"

#ifndef USE_WING

#ifdef USE_POSITION_HOLD

#include "math.h"
#include "build/debug.h"
#include "common/maths.h"

#include "config/config.h"
#include "fc/core.h"
#include "fc/runtime_config.h"
#include "fc/rc.h"
#include "flight/autopilot.h"
#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/pid.h"
#include "flight/position.h"
#include "flight/position_estimator.h"
#include "rx/rx.h"

#include "pg/pos_hold.h"
#include "pos_hold.h"

#define POSHOLD_YAW_DEADBAND 0.10f // fraction of centre-to-endpoint RC travel
#define POSHOLD_YAW_RATE_LIMIT 90.0f // degrees/second

typedef struct posHoldState_s {
    bool isEnabled;
    bool isControlOk;
    bool areSensorsOk;
    float deadband;
    bool headingCaptured;
    float headingDeg;
    float yawRateDps;
} posHoldState_t;

static posHoldState_t posHold;

static void resetHeadingHold(void)
{
    posHold.headingCaptured = false;
    posHold.yawRateDps = 0.0f;
}

void posHoldInit(void)
{
    posHold.deadband = posHoldConfig()->deadband * 0.01f;
    resetHeadingHold();
}

static bool headingHoldAllowed(void)
{
    // Rescue and mission navigation retain their own yaw controllers. Neither
    // roll/pitch stick activity nor ALTHOLD releases this heading controller.
    if (!ARMING_FLAG(ARMED) || !FLIGHT_MODE(POS_HOLD_MODE)
        || FLIGHT_MODE(GPS_RESCUE_MODE | AUTOPILOT_MODE)
        || !isAutopilotInControl() || !positionEstimatorIsValidXY() || !imuIsHeadingValid()) {
        return false;
    }

    // 2026.6 uses +/-500 us before deadband for normalised +/-1 stick travel.
    // Compare before RC deadband, smoothing, rates and reversal. Exactly 10%
    // remains in hold. A failsafe receiver value must not release the target.
    return failsafeIsActive()
        || fabsf(rcData[FD_YAW] - rxConfig()->midrc) <= POSHOLD_YAW_DEADBAND * 500.0f;
}

bool posHoldHeadingControlActive(void)
{
    if (!headingHoldAllowed()) {
        // Mode, RX and sensor changes can precede the next position task.
        // Invalidate here too, including a short yaw input between task ticks.
        resetHeadingHold();
    }
    return posHold.headingCaptured;
}

float posHoldGetYawRate(void)
{
    return posHold.yawRateDps;
}

static void updateHeadingHold(void)
{
    if (!headingHoldAllowed()) {
        resetHeadingHold();
        return;
    }

    const float headingDeg = attitude.values.yaw * 0.1f;
    if (!posHold.headingCaptured) {
        posHold.headingDeg = headingDeg;
        posHold.headingCaptured = true;
    }

    // IMU compass heading is CW-positive, but gyro yaw/rate PID is CCW-positive.
    // Use current minus target, wrapped to the shortest turn. Do not apply the
    // pilot's yaw reversal to automatic feedback. No outer I, D or engage ramp.
    const float errorDeg = fmodf(headingDeg - posHold.headingDeg + 540.0f, 360.0f) - 180.0f;
    const float kp = currentPidProfile->pid[PID_MAG].P * 0.1f; // default 40 -> 4.0/s
    posHold.yawRateDps = constrainf(kp * errorDeg, -POSHOLD_YAW_RATE_LIMIT, POSHOLD_YAW_RATE_LIMIT);
}

static void posHoldCheckSticks(void)
{
    if (failsafeIsActive()) {
        setSticksActiveStatus(false);
        return;
    }
    const bool sticksDeflected = (getRcDeflectionAbs(FD_ROLL) > posHold.deadband) || (getRcDeflectionAbs(FD_PITCH) > posHold.deadband);
    setSticksActiveStatus(sticksDeflected);
}

static bool sensorsOk(void)
{
    // Optical flow position hold is heading-agnostic: the same yaw is used
    // to project flow into ENU and to rotate the correction back to body
    // frame, so a heading error cancels. GPS-assisted hold is not: GPS
    // provides absolute ENU measurements and a bad yaw in the body-frame
    // correction rotation will cause a flyaway.
    // Use the runtime GPS state (fix present + config allows GPS) rather than
    // the configured source alone, so AUTO mode with no GPS hardware correctly

    if (!positionEstimatorIsValidXY()) {
        return false; // always need valid XY data, can be optical only
    }

    if (positionEstimatorIsHeadingRequired()) {
        return imuIsHeadingValid(); // if heading is essential (ie no optical flow), pass or fail based on whether or not heading exists.
    } else {
        return true; // if no heading is needed, we don't care about it (optical flow situation)
    }
}

void updatePosHold(timeUs_t currentTimeUs) {
    UNUSED(currentTimeUs);
    if (FLIGHT_MODE(POS_HOLD_MODE) || FLIGHT_MODE(GPS_RESCUE_MODE)) {
        if (!posHold.isEnabled) {
            resetPositionControl(POSHOLD_TASK_RATE_HZ);
            posHold.isControlOk = true;
            posHold.isEnabled = true;
        }
    } else {
        if (posHold.isEnabled) {
            setSticksActiveStatus(false);
        }
        posHold.isEnabled = false;
    }

    if (posHold.isEnabled) {
        posHoldCheckSticks();
        const bool sensorsWereOk = posHold.areSensorsOk;
        posHold.areSensorsOk = sensorsOk();
        if (posHold.areSensorsOk) {
            if (!sensorsWereOk) {
                // Sensors came back after a dropout: the craft drifted while
                // blind, so resuming against the pre-dropout target would
                // lurch toward it — or trip the sanity fence on the spot.
                // Re-anchor the hold at the current position instead.
                positionControlReanchor();
            }
            posHold.isControlOk = positionControl();
        } else {
            for (unsigned i = 0; i < RP_AXIS_COUNT; i++) {
                autopilotAngle[i] = 0.0f;
            }
        }
    }
    updateHeadingHold();
}

bool isAutopilotInControl(void)
{
    return posHold.isEnabled && posHold.isControlOk && posHold.areSensorsOk;
}

bool posHoldFailure(void) {
    return FLIGHT_MODE(POS_HOLD_MODE) && (!posHold.isControlOk || !posHold.areSensorsOk);
}

// Pre-engagement readiness: the entry conditions alone, without the
// control-failure checks in posHoldFailure() that only apply once engaged.
bool posHoldReady(void) {
    return sensorsOk();
}

#endif // USE_POSITION_HOLD

#endif // !USE_WING
