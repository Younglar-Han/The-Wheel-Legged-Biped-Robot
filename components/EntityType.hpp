#ifndef ENTITYTYPE_HPP
#define ENTITYTYPE_HPP

enum EntityType
{
    ET_SENSOR = 0,
    ET_CONTROLLER,
    ET_ACTUATOR,

    ET_LENGTH
};

enum EntityClassType
{
    ECT_NULL = 0,
    ECT_M3508,
    ECT_M3508Sensor,
    ECT_M2006,
    ECT_M2006Sensor,
    ECT_G6020,
    ECT_G6020Sensor,
    ECT_LK9025,
    ECT_LK9025Sensor,
    ECT_HT04,
    ECT_HT04Sensor,
    ECT_ChassisController,
    ECT_LegController,
    ECT_WheelController,
	ECT_ShooterController,
    ECT_GimbalController,
    ECT_Imu,

    ECT_CrosshairDrawer,
    ECT_HudManager,

    ECT_LENGTH
};

#endif
