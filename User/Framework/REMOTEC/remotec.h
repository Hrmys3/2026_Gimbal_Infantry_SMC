//
// Created by mjw on 2022/9/23.
//

#ifndef ROBO_TEST_REMOTEC_H
#define ROBO_TEST_REMOTEC_H
#ifdef __cplusplus
extern "C" {
#endif
#include <cctype>
#include "remoteio.h"
#include "stm32f4xx_hal.h"
#include "gimbalc.h"

#define VT13_RX_BUF_NUM 36u

#define RC_FRAME_LENGTH 21u

#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W            ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S            ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A            ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D            ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT        ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL         ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q            ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E            ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R            ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F            ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G            ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z            ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X            ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C            ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V            ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B            ((uint16_t)1 << 15)
/* ----------------------- Data Struct ------------------------------------- */

typedef __packed struct
{
	uint8_t Last_State: 1;
	uint8_t Now_State: 1;
	uint8_t Is_Click_Once: 1;
} Key_State; //键盘上任意一个按键的状态

typedef __packed struct
{
	__packed struct
	{
		int16_t ch[4];//摇杆
		uint8_t mode_sw;//中间三档开关 目前自左向右为小陀螺、随动、急停
		uint8_t stop;//暂停键
		uint8_t left_button;//fn;
		uint8_t	right_button;
		int16_t wheel;//拨轮
		uint8_t shutter;//扳机
	} rc;
	__packed struct
	{
		int16_t x;
		int16_t y;
		int16_t z;
//以下为使用到点按键值的键
		Key_State press_l;
		Key_State press_r;
		Key_State middle;
	} mouse;
	__packed struct
	{
		uint16_t value;
		uint8_t W: 1;
		uint8_t S: 1;
		uint8_t A: 1;
		uint8_t D: 1;
//以下为使用到点按键值的键
		Key_State SHIFT;
		Key_State CONTRL;
		Key_State Q;
		Key_State E;
		Key_State R;
		Key_State F;
		Key_State G;
		Key_State Z;
		Key_State X;
		Key_State C;
		Key_State V;
		Key_State B;
	} key;
	uint16_t crc16;
} RC_ctrl_t;//新协议

/* ----------------------- Internal Data ----------------------------------- */

/**
  * @brief          remote control init
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          遥控器初始化
  * @param[in]      none
  * @retval         none
  */
void REMOTEC_Init(void);

class remotec {
 public:
	//remote control data
	//遥控控制变量
	RC_ctrl_t rc_ctrl;
	uint32_t RC_GetNewData = 0;//检测键值是否在发送/更新
	uint8_t ControlMode = RC_MODE; //一般默认遥控器控制
	bool is_online = 0;

	void RC_DataHandle();

	float yaw_speed;
	float pih_speed;
	float vx = 0;
	float vy = 0;
//	float vz = 0;
	int8_t MotionMode;
	int8_t Last_MotionMode = SUIDONG;
	int8_t FricMode ;
	int8_t Last_FricMode = OPENFRIC;

	uint8_t portIsZimiao(void);
	uint8_t portIsRedrawing(void);

	void ShootSpeedTarget(float Shoot_Speed, float Ram_Speed, int8_t mode); //bug:有时候目标值弹跳 已解决
	// void sbus_to_rc( const uint8_t* sbus_buf);
	void vt13_to_rc( const uint8_t* VT13_buf);
	void update();
	void portHandle(Key_State* port); //非连续键值处理 即上一次是0，本次是1，判断为按了一次，用于处理状态切换等不适用于连续用手按的按键
 private:
	void portSetYawSpeed(void);
	void portSetPihSpeed(void);
	void portSetVx(void);
	void portSetVy(void);
	void portSetCarMode(void);
	void portSetProtect(void);
	void portSetHead(void);
	void Swich_ControlMode(void);
};
extern remotec MyRemote;
#ifdef __cplusplus
}
#endif
#endif //ROBO_TEST_REMOTEC_H
