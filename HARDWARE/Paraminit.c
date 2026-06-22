#include "mytype.h"
#include "CanBus_Task.h"
#include "PID.h"
#include "Paraminit.h"
#include "ControlTask.h"
#include "MotorControlTask.h"

void ParamInit(void)
{
    /* 全部轴(轨迹 + 双环 PID)由编译期配置表 axis_cfg[] 驱动初始化。
     * 每个电机的 V/A/J、PID、减速比、方向都在 MotorControlTask.c 的配置表里改。 */
    Axis_InitAll();
}
