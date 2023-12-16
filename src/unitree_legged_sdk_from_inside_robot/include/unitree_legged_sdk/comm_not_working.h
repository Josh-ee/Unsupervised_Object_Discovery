/*****************************************************************
 Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
******************************************************************/

#ifndef _UNITREE_LEGGED_COMM_H_
#define _UNITREE_LEGGED_COMM_H_

#include <stdint.h>

// amarco added 
// #include <iostream>
#include <array>

namespace UNITREE_LEGGED_SDK 
{

	constexpr int HIGHLEVEL    = 0x00;
	constexpr int LOWLEVEL     = 0xff;
	constexpr int TRIGERLEVEL  = 0xf0;
	constexpr double PosStopF  = (2.146E+9f);
	constexpr double VelStopF  = (16000.0f);
	extern const int HIGH_CMD_LENGTH;      // sizeof(HighCmd)
	extern const int HIGH_STATE_LENGTH;    // sizeof(HighState)
	extern const int LOW_CMD_LENGTH;       // shorter than sizeof(LowCmd),   bytes compressed LowCmd length  
	extern const int LOW_STATE_LENGTH;     // shorter than sizeof(LowState), bytes compressed LowState length

#pragma pack(1)

	typedef struct
	{
		uint8_t off;                       // off 0xA5
		uint8_t reserve[3];
	} BmsCmd;

	typedef struct
	{
		uint8_t version_h;
		uint8_t version_l;
		uint8_t bms_status;
		uint8_t SOC;                       // SOC 0-100%
		int32_t current;                   // mA
		uint16_t cycle;
		int8_t BQ_NTC[2];                  // x1 degrees centigrade
		int8_t MCU_NTC[2];                 // x1 degrees centigrade
		uint16_t cell_vol[10];             // cell voltage mV
	} BmsState;

	typedef struct
	{
		float x;
		float y;
		float z;
	} Cartesian;

	// typedef struct
	// {
	// 	float quaternion[4];               // quaternion, normalized, (w,x,y,z)
	// 	float gyroscope[3];                // angular velocity （unit: rad/s)    (raw data)
	// 	float accelerometer[3];            // m/(s2)                             (raw data)
	// 	float rpy[3];                      // euler angle（unit: rad)
	// 	int8_t temperature;
	// } IMU;                                 // when under accelerated motion, the attitude of the robot calculated by IMU will drift.

	// amarco: added because pybind can't compile the above
	typedef struct
	{
		std::array<float, 4> quaternion;               // quaternion, normalized, (w,x,y,z)
		std::array<float, 3> gyroscope;                // angular velocity （unit: rad/s)    (raw data)
		std::array<float, 3> accelerometer;            // m/(s2)                             (raw data)
		std::array<float, 3> rpy;                      // euler angle（unit: rad)
		int8_t temperature;
	} IMU;  


	typedef struct
	{
		uint8_t r;
		uint8_t g;
		uint8_t b;
	} LED;                                 // foot led brightness: 0~255

	typedef struct
	{
		uint8_t mode;                      // motor working mode 
		float q;                           // current angle (unit: radian)
		float dq;                          // current velocity (unit: radian/second)
		float ddq;                         // current acc (unit: radian/second*second)
		float tauEst;                      // current estimated output torque (unit: N.m)
		float q_raw;                       // current angle (unit: radian)
		float dq_raw;                      // current velocity (unit: radian/second)
		float ddq_raw;
		int8_t temperature;                // current temperature (temperature conduction is slow that leads to lag)
		// uint32_t reserve[2];
		std::array<uint32_t, 2> reserve; // amarco added
	} MotorState;                          // motor feedback

	typedef struct
	{
		uint8_t mode;                      // desired working mode
		float q;                           // desired angle (unit: radian) 
		float dq;                          // desired velocity (unit: radian/second)
		float tau;                         // desired output torque (unit: N.m)
		float Kp;                          // desired position stiffness (unit: N.m/rad )
		float Kd;                          // desired velocity stiffness (unit: N.m/(rad/s) )
		// uint32_t reserve[3];
		std::array<uint32_t, 3> reserve; // amarco added
	} MotorCmd;                            // motor control

	typedef struct
	{
		uint8_t levelFlag;                 // flag to distinguish high level or low level
		uint16_t commVersion;
		uint16_t robotID;
		uint32_t SN; 
		uint8_t bandWidth;
		IMU imu;
		std::array<MotorState, 20> motorState; // amarco: The first 12 elements correspond to the joints, as per quadruped.h; see example_torque.cpp or example_position.cpp
		std::array<int16_t, 4> footForce;              // force sensors
		std::array<int16_t, 4> footForceEst;           // force sensors
		// MotorState motorState[20];
		BmsState bms;
		// int16_t footForce[4];              // force sensors
		// int16_t footForceEst[4];           // force sensors
		uint32_t tick;                     // reference real-time from motion controller (unit: us)
		// uint8_t wirelessRemote[40];        // wireless commands
		std::array<uint8_t, 40> wirelessRemote;        // wireless commands
		uint32_t reserve;
		uint32_t crc;
	} LowState;                            // low level feedback

	typedef struct 
	{
		uint8_t levelFlag;
		uint16_t commVersion;
		uint16_t robotID;
		uint32_t SN;
		uint8_t bandWidth;
		// MotorCmd motorCmd[20];
		BmsCmd bms;
		// uint8_t wirelessRemote[40];
		std::array<MotorCmd, 20> motorCmd;
		// std::array<LED, 4> led;
		std::array<uint8_t, 40> wirelessRemote;
		uint32_t reserve;
		uint32_t crc;
	} LowCmd;                              // low level control

	typedef struct
	{
		uint8_t levelFlag;
		uint16_t commVersion;
		uint16_t robotID;
		uint32_t SN;
		uint8_t bandWidth;
		IMU imu;
		// MotorState motorState[20];
		std::array<MotorState, 20> motorState;    // foot position relative to body; amarco: The first 12 elements correspond to the joints, as per quadruped.h; see example_torque.cpp or example_position.cpp
		BmsState bms;
		// int16_t footForce[4];
		// int16_t footForceEst[4];
		std::array<int16_t, 4> footForce;
		std::array<int16_t, 4> footForceEst;
		uint8_t mode;
		float progress;
		uint8_t gaitType;                  // 0.idle  1.trot  2.trot running  3.climb stair  4.trot obstacle
		float footRaiseHeight;             // (unit: m, default: 0.08m), foot up height while walking
		std::array<float, 3> position;
		// float position[3];                 // (unit: m), from own odometry in inertial frame, usually drift
		float bodyHeight;                  // (unit: m, default: 0.28m),
		std::array<float, 3> velocity;
		// float velocity[3];                 // (unit: m/s), forwardSpeed, sideSpeed, rotateSpeed in body frame
		float yawSpeed;                    // (unit: rad/s), rotateSpeed in body frame        
		std::array<float, 4> rangeObstacle;
		// float rangeObstacle[4];
		// Cartesian footPosition2Body[4];    // foot position relative to body
		// Cartesian footSpeed2Body[4];       // foot speed relative to body
		std::array<Cartesian, 4> footPosition2Body;    // foot position relative to body
		std::array<Cartesian, 4> footSpeed2Body;       // foot speed relative to body
		// uint8_t wirelessRemote[40];
		std::array<uint8_t, 40> wirelessRemote;
		uint32_t reserve;
		uint32_t crc;
	} HighState;                           // high level feedback

	typedef struct
	{
		uint8_t levelFlag;
		uint16_t commVersion;
		uint16_t robotID;
		uint32_t SN;
		uint8_t bandWidth;
		uint8_t mode;                       // 0. idle, default stand  1. force stand (controlled by dBodyHeight + ypr)
											// 2. target velocity walking (controlled by velocity + yawSpeed)
											// 3. target position walking (controlled by position + ypr[0])
											// 4. path mode walking (reserve for future release)
											// 5. position stand down. 
											// 6. position stand up 
											// 7. damping mode 
											// 8. recovery stand
											// 9. backflip
											// 10. jumpYaw
											// 11. straightHand
											// 12. dance1
											// 13. dance2

		uint8_t gaitType;                  // 0.idle  1.trot  2.trot running  3.climb stair
		uint8_t speedLevel;                // 0. default low speed. 1. medium speed 2. high speed. during walking, only respond MODE 3
		float footRaiseHeight;             // (unit: m, default: 0.08m), foot up height while walking
		float bodyHeight;                  // (unit: m, default: 0.28m),
		std::array<float, 2> postion;
		std::array<float, 3> euler;
		std::array<float, 2> velocity;
		// float postion[2];                  // (unit: m), desired position in inertial frame
		// float euler[3];                    // (unit: rad), roll pitch yaw in stand mode
		// float velocity[2];                 // (unit: m/s), forwardSpeed, sideSpeed in body frame
		float yawSpeed;                    // (unit: rad/s), rotateSpeed in body frame
		BmsCmd bms;
		// LED led[4];
		// uint8_t wirelessRemote[40];
		std::array<LED, 4> led;
		std::array<uint8_t, 40> wirelessRemote;
		uint32_t reserve;
		uint32_t crc;
	} HighCmd;                             // high level control

#pragma pack()

	typedef struct     
	{
		unsigned long long TotalCount;     // total loop count
		unsigned long long SendCount;      // total send count
		unsigned long long RecvCount;      // total receive count
		unsigned long long SendError;      // total send error 
		unsigned long long FlagError;      // total flag error 
		unsigned long long RecvCRCError;   // total reveive CRC error	
		unsigned long long RecvLoseError;  // total lose package count	
	} UDPState;                            // UDP communication state

}

#endif
