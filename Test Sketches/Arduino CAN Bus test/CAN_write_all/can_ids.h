#ifndef CAN_ID_H
#define CAN_ID_H

#define E_STOP_MSG_ID               4
#define MOTOR_LOCKOUT_MSG_ID        5

#define ECU_STATUS_MSG_ID           8
#define BMS_STATUS_MSG_ID           9
#define TPS_STATUS_MSG_ID           10
#define SAS_STATUS_MSG_ID           11
#define IMU_STATUS_MSG_ID           12
#define FW_STATUS_MSG_ID            13
#define RLW_STATUS_MSG_ID           15
#define RRW_STATUS_MSG_ID           16
#define FL_STATUS_MSG_ID            17
#define RL_STATUS_MSG_ID            18
#define INT_STATUS_MSG_ID           19

#define THROTTLE_BRAKE_MSG_ID       32
#define REV_BOOST_MSG_ID            34
#define BATT_STATS_MSG_ID           36
#define VOLTAGE VALUES_MSG_ID       37

#define STEERING_MSG_ID             44
#define INDIV_WHEEL_THROTTLES_MSG_ID    48
#define FL_SPEED_MSG_ID             52
#define FR_SPEED_MSG_ID             53
#define RL_SPEED_MSG_ID             54
#define RR_SPEED_MSG_ID             55
#define VEH_SPEED_MSG_ID            56

#define L_R_DIFF_MSG_ID             60
#define F_R_DIFF_MSG_ID             61

//TODO: Add in IMU messages?

#define CALLIBRATE_STEERING_MSG_ID  80

#define FRONT_LIGHT_MSG_ID          96
#define REAR_LIGHT_MSG_ID           97
#define INT_LIGHT_MSG_ID            99

#endif  //CAN_ID_H