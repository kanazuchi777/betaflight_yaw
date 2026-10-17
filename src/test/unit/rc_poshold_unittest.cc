// Run the RC regressions both without POSHOLD (rc_unittest) and with the
// complete POSHOLD/RC smoothing/feedforward path enabled in this target.
#include "rc_unittest.cc"

extern "C" {
#include "flight/pid.h"
#include "flight/pid_init.h"
#include "fc/controlrate_profile.h"
#include "sensors/gyro.h"

float posHoldGetYawRate(void) { return mockAutopilotYawRate; }
float gpsRescueGetYawRate(void) { return mockAutopilotYawRate; }
pidRuntime_t pidRuntime;
gyro_t gyro;
uint32_t targetPidLooptime = 1000;
static bool mockPosHoldYawActive;
bool posHoldHeadingControlActive(void) { return mockPosHoldYawActive; }
}

class RcPosHoldTest : public ::testing::Test {
protected:
    void SetUp() override {
        pgResetAll();
        loadControlRateProfile();
        mockPosHoldYawActive = false;
        mockAutopilotYawActive = false;
        mockAutopilotYawRate = 0;
        flightModeFlags = 0;
        rxConfigMutable()->midrc = 1500;
        rxConfigMutable()->mincheck = 1050;
        rxConfigMutable()->maxcheck = 1900;
        rxConfigMutable()->rc_smoothing_setpoint_cutoff = 30;
        rxConfigMutable()->rc_smoothing_throttle_cutoff = 30;
        for (float &channel : rcData) {
            channel = 1500;
        }
        pidRuntime.dT = 0.001f;
        pidRuntime.feedforwardSmoothFactor = 25;
        pidRuntime.feedforwardJitterFactorInv = 1;
        pidRuntime.feedforwardYawHoldTime = 100;
        pidRuntime.feedforwardYawHoldGain = 1;
        initRcProcessing();
    }

    void frame(float yawUs, timeUs_t timeUs) {
        rcData[YAW] = yawUs;
        updateRcRefreshRate(timeUs, true);
        updateRcCommands();
        processRcCommand();
    }
};

TEST_F(RcPosHoldTest, HeadingRateBypassesRcSmoothingAndDoesNotNeedNewRxFrame)
{
    for (bool smoothing : {false, true}) {
        rxConfigMutable()->rc_smoothing = smoothing;
        frame(1750, 10000);
        const float pilotRate = getSetpointRate(YAW);
        mockAutopilotYawActive = mockPosHoldYawActive = true;
        flightModeFlags = POS_HOLD_MODE | ALT_HOLD_MODE | MAG_MODE;
        mockAutopilotYawRate = 20;
        EXPECT_FLOAT_EQ(20, getSetpointRate(YAW));
        mockAutopilotYawRate = -90;
        EXPECT_FLOAT_EQ(-90, getSetpointRate(YAW));
        mockPosHoldYawActive = false;
        EXPECT_FLOAT_EQ(pilotRate, getSetpointRate(YAW));
    }
}

TEST_F(RcPosHoldTest, ResidualPilotFeedforwardIsSuppressedOnlyDuringHold)
{
    timeUs_t now = 100000;
    for (bool smoothing : {false, true}) {
        rxConfigMutable()->rc_smoothing = smoothing;
        mockPosHoldYawActive = false;
        // Establish RX timing and initialise the real smoothing cutoffs.
        for (int i = 0; i < 10; i++) {
            frame(1500, now += 10000);
        }
        frame(1750, now += 10000);
        const float pilotFeedforward = getFeedforward(YAW);
        ASSERT_NE(0.0f, pilotFeedforward);
        mockPosHoldYawActive = true;
        EXPECT_FLOAT_EQ(0, getFeedforward(YAW));
        mockPosHoldYawActive = false;
        EXPECT_FLOAT_EQ(pilotFeedforward, getFeedforward(YAW));
    }
}

TEST_F(RcPosHoldTest, PosHoldDoesNotPollutePilotRateHistory)
{
    rxConfigMutable()->rc_smoothing = false;
    frame(1500, 10000);
    mockPosHoldYawActive = mockAutopilotYawActive = true;
    mockAutopilotYawRate = 90;
    flightModeFlags = POS_HOLD_MODE;
    autopilotConfigMutable()->stickDeadband = 200;
    frame(1500, 20000);
    EXPECT_FLOAT_EQ(90, getSetpointRate(YAW));
    mockPosHoldYawActive = false;
    EXPECT_FLOAT_EQ(0, getSetpointRate(YAW));
    frame(1600, 30000);
    EXPECT_LT(getSetpointRate(YAW), 0); // normal pilot yaw direction
}

TEST_F(RcPosHoldTest, RescueKeepsItsExistingRatePath)
{
    rxConfigMutable()->rc_smoothing = false;
    mockAutopilotYawActive = true;
    mockAutopilotYawRate = -30;
    autopilotConfigMutable()->stickDeadband = 50;
    flightModeFlags = POS_HOLD_MODE | GPS_RESCUE_MODE;
    frame(1500, 10000);
    EXPECT_FLOAT_EQ(-30, getSetpointRate(YAW));
}
