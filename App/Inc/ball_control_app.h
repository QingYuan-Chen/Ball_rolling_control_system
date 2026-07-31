#ifndef BALLCONTROL_APP_H
#define BALLCONTROL_APP_H

#include <stdbool.h>
#include <stdint.h>

#include "ball_control_core.h"

void BallControl_Init(void);
void BallControl_Process(void);
/* TIM6 1 kHz ISR入口：只发布节拍，不在中断内执行控制计算。 */
void BallControl_Timer1kHzCallback(void);
void BallControl_UsbCdcSetConnected(bool connected);
void BallControl_UsbCdcReceive(const uint8_t *data, uint32_t length);
/* 以下控制API应从主循环上下文调用。配置不完整时不会接受使能请求。 */
bool BallControl_ApplyControlConfig(const BallControl_Config *config);
bool BallControl_RequestControlEnable(bool enabled);
void BallControl_SetControlSetpoint(float setpoint_mm);
void BallControl_GetControlStatus(BallControl_Status *status);

#endif
