# 达妙电机驱动代码深度解析

## 1. 头文件分析 (dvc_motor_dm.h)

### 1.1 文件概述

这是一个用于达妙电机的驱动头文件，支持传统模式和一拖四模式两种控制方式，版本0.1为新增功能。

### 1.2 包含的头文件

```cpp
#include "1_Middleware/2_Algorithm/PID/alg_pid.h"  // PID算法库
#include "1_Middleware/1_Driver/CAN/drv_can.h"    // CAN驱动库
#include "1_Middleware/1_Driver/Math/drv_math.h"  // 数学运算库
```

### 1.3 枚举类型定义

#### 1.3.1 电机状态枚举

```cpp
enum Enum_Motor_DM_Status
{
    Motor_DM_Status_DISABLE = 0,  // 电机断开连接
    Motor_DM_Status_ENABLE,       // 电机正常连接
};
```

**作用**: 表示达妙电机的连接状态。

#### 1.3.2 电机ID枚举（一拖四模式）

```cpp
enum Enum_Motor_DM_Motor_ID_1_To_4 : uint8_t
{
    Motor_DM_ID_0x301 = 1,  // 一拖四模式电机ID
    Motor_DM_ID_0x302,
    Motor_DM_ID_0x303,
    Motor_DM_ID_0x304,
    Motor_DM_ID_0x305,
    Motor_DM_ID_0x306,
    Motor_DM_ID_0x307,
    Motor_DM_ID_0x308,
};
```

**作用**: 定义一拖四模式下电机在CAN总线上的ID，范围为0x301-0x308。

#### 1.3.3 传统模式控制状态枚举

```cpp
enum Enum_Motor_DM_Control_Status_Normal
{
    Motor_DM_Control_Status_DISABLE = 0x0,        // 电机失能
    Motor_DM_Control_Status_ENABLE,               // 电机使能
    Motor_DM_Control_Status_OVERVOLTAGE = 0x8,    // 过压
    Motor_DM_Control_Status_UNDERVOLTAGE,         // 欠压
    Motor_DM_Control_Status_OVERCURRENT,          // 过流
    Motor_DM_Control_Status_MOS_OVERTEMPERATURE,  // MOS过温
    Motor_DM_Control_Status_ROTOR_OVERTEMPERATURE,// 转子过温
    Motor_DM_Control_Status_LOSE_CONNECTION,      // 失联
    Motor_DM_Control_Status_MOS_OVERLOAD,         // MOS过载
};
```

**作用**: 定义传统模式下电机的各种运行状态和故障状态。

#### 1.3.4 控制方式枚举

```cpp
enum Enum_Motor_DM_Control_Method
{
    Motor_DM_Control_Method_NORMAL_MIT = 0,    // 传统模式MIT控制
    Motor_DM_Control_Method_NORMAL_ANGLE_OMEGA,// 传统模式位置速度控制
    Motor_DM_Control_Method_NORMAL_OMEGA,      // 传统模式速度控制
    Motor_DM_Control_Method_NORMAL_EMIT,       // 传统模式EMIT控制
    Motor_DM_Control_Method_1_TO_4_CURRENT,    // 一拖四模式电流控制
    Motor_DM_Control_Method_1_TO_4_OMEGA,      // 一拖四模式速度控制
    Motor_DM_Control_Method_1_TO_4_ANGLE,      // 一拖四模式位置控制
};
```

**作用**: 定义达妙电机的控制模式，分为传统模式和一拖四模式两大类。

### 1.4 结构体定义

#### 1.4.1 传统模式CAN接收数据结构

```cpp
struct Struct_Motor_DM_CAN_Rx_Data_Normal
{
    uint8_t CAN_ID : 4;                        // CAN ID（4位）
    uint8_t Control_Status_Enum : 4;           // 控制状态（4位）
    uint16_t Angle_Reverse;                    // 角度（大小端转换）
    uint8_t Omega_11_4;                        // 角速度高4位
    uint8_t Omega_3_0_Torque_11_8;             // 角速度低4位+扭矩高4位
    uint8_t Torque_7_0;                        // 扭矩低8位
    uint8_t MOS_Temperature;                   // MOS温度
    uint8_t Rotor_Temperature;                 // 转子温度
} __attribute__((packed));
```

**作用**: 定义传统模式下从CAN总线接收的8字节数据包格式。

#### 1.4.2 一拖四模式CAN接收数据结构

```cpp
struct Struct_Motor_DM_CAN_Rx_Data_1_To_4
{
    uint16_t Encoder_Reverse;                  // 编码器值（大小端转换）
    int16_t Omega_Reverse;                     // 角速度（大小端转换，×100）
    int16_t Current_Reverse;                   // 电流值（大小端转换，mA）
    uint8_t Rotor_Temperature;                 // 转子温度
    uint8_t MOS_Temperature;                   // MOS温度
} __attribute__((packed));
```

**作用**: 定义一拖四模式下从CAN总线接收的8字节数据包格式。

#### 1.4.3 各种发送数据结构

```cpp
// MIT控制报文
struct Struct_Motor_DM_CAN_Tx_Data_Normal_MIT {
    uint16_t Control_Angle_Reverse;            // 控制角度（大小端转换）
    uint8_t Control_Omega_11_4;                // 控制角速度高4位
    uint8_t Control_Omega_3_0_K_P_11_8;        // 角速度低4位+Kp高4位
    uint8_t K_P_7_0;                          // Kp低8位
    uint8_t K_D_11_4;                         // Kd高4位
    uint8_t K_D_3_0_Control_Torque_11_8;       // Kd低4位+控制扭矩高4位
    uint8_t Control_Torque_7_0;               // 控制扭矩低8位
};

// 位置速度控制报文
struct Struct_Motor_DM_CAN_Tx_Data_Normal_Angle_Omega {
    float Control_Angle;                      // 控制角度（float）
    float Control_Omega;                      // 控制角速度（float）
};

// 速度控制报文
struct Struct_Motor_DM_CAN_Tx_Data_Normal_Omega {
    float Control_Omega;                      // 控制角速度（float）
};

// EMIT控制报文
struct Struct_Motor_DM_CAN_Tx_Data_Normal_EMIT {
    float Control_Angle;                      // 控制角度
    uint16_t Control_Omega;                   // 限定速度（×100）
    uint16_t Control_Current;                 // 限定电流（×10000）
};
```

**作用**: 定义不同控制模式下的CAN发送数据格式。

#### 1.4.4 处理后数据结构

```cpp
struct Struct_Motor_DM_Rx_Data_Normal
{
    Enum_Motor_DM_Control_Status_Normal Control_Status;  // 控制状态
    float Now_Angle;                                    // 当前角度
    float Now_Omega;                                    // 当前角速度
    float Now_Torque;                                   // 当前扭矩
    float Now_MOS_Temperature;                          // 当前MOS温度
    float Now_Rotor_Temperature;                        // 当前转子温度
    uint32_t Pre_Encoder;                               // 前一时刻编码器值
    int32_t Total_Encoder;                              // 总编码器值
    int32_t Total_Round;                                // 总圈数
};
```

**作用**: 存储解析后的传统模式电机状态数据。

### 1.5 两个电机类定义

#### 1.5.1 传统模式电机类

```cpp
class Class_Motor_DM_Normal
{
public:
    // 公共接口函数...
};
```

**特点**: 支持多种控制模式，包含丰富的电机状态信息。

#### 1.5.2 一拖四模式电机类

```cpp
class Class_Motor_DM_1_To_4
{
public:
    // 两层PID控制器
    Class_PID PID_Angle;      // 角度环PID
    Class_PID PID_Omega;      // 角速度环PID

    // 公共接口函数...
};
```

**特点**: 支持PID控制，类似于大疆电机的一拖四模式。

## 2. 实现文件分析 (dvc_motor_dm.cpp)

### 2.1 预定义消息数组

#### 2.1.1 传统模式控制命令

```cpp
// 清除电机错误信息
uint8_t DM_Motor_CAN_Message_Clear_Error[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfb};

// 使能电机
uint8_t DM_Motor_CAN_Message_Enter[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc};

// 失能电机
uint8_t DM_Motor_CAN_Message_Exit[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd};

// 保存当前电机位置为零点
uint8_t DM_Motor_CAN_Message_Save_Zero[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe};
```

**作用**: 定义传统模式下电机的基本控制命令，最后一位区分不同功能。

### 2.2 CAN发送缓冲区分配函数

```cpp
uint8_t *allocate_tx_data(CAN_HandleTypeDef *hcan, Enum_Motor_DM_Motor_ID_1_To_4 __CAN_Rx_ID_1_To_4)
{
    uint8_t *tmp_tx_data_ptr;
    if (hcan == &hcan1)
    {
        switch (__CAN_Rx_ID_1_To_4)
        {
        case (Motor_DM_ID_0x301):
            tmp_tx_data_ptr = &(CAN1_0x3fe_Tx_Data[0]);  // 0x301使用0x3fe组的第0个位置
            break;
        case (Motor_DM_ID_0x302):
            tmp_tx_data_ptr = &(CAN1_0x3fe_Tx_Data[2]);  // 0x302使用0x3fe组的第2个位置
            break;
        // ... 其他ID处理
        }
    }
    // ... CAN2处理
    return (tmp_tx_data_ptr);
}
```

**作用**: 为一拖四模式分配对应的CAN发送缓冲区。

### 2.3 传统模式类实现

#### 2.3.1 初始化函数

```cpp
void Class_Motor_DM_Normal::Init(CAN_HandleTypeDef *hcan, uint8_t __CAN_Rx_ID, uint8_t __CAN_Tx_ID, ...)
{
    if (hcan->Instance == CAN1)
        CAN_Manage_Object = &CAN1_Manage_Object;
    else if (hcan->Instance == CAN2)
        CAN_Manage_Object = &CAN2_Manage_Object;

    CAN_Rx_ID = __CAN_Rx_ID;
    switch (__Motor_DM_Control_Method)
    {
    case (Motor_DM_Control_Method_NORMAL_MIT):
        CAN_Tx_ID = __CAN_Tx_ID;  // MIT模式：发送ID = 原ID
        break;
    case (Motor_DM_Control_Method_NORMAL_ANGLE_OMEGA):
        CAN_Tx_ID = __CAN_Tx_ID + 0x100;  // 位置速度模式：发送ID = 原ID + 0x100
        break;
    case (Motor_DM_Control_Method_NORMAL_OMEGA):
        CAN_Tx_ID = __CAN_Tx_ID + 0x200;  // 速度模式：发送ID = 原ID + 0x200
        break;
    case (Motor_DM_Control_Method_NORMAL_EMIT):
        CAN_Tx_ID = __CAN_Tx_ID + 0x300;  // EMIT模式：发送ID = 原ID + 0x300
        break;
    }
    // 设置各种参数...
}
```

**作用**: 初始化传统模式电机，根据控制方式计算发送ID。

#### 2.3.2 专用控制命令函数

```cpp
void Class_Motor_DM_Normal::CAN_Send_Clear_Error()
{
    CAN_Send_Data(CAN_Manage_Object->CAN_Handler, CAN_Tx_ID, DM_Motor_CAN_Message_Clear_Error, 8);
}

void Class_Motor_DM_Normal::CAN_Send_Enter()
{
    CAN_Send_Data(CAN_Manage_Object->CAN_Handler, CAN_Tx_ID, DM_Motor_CAN_Message_Enter, 8);
}

void Class_Motor_DM_Normal::CAN_Send_Exit()
{
    CAN_Send_Data(CAN_Manage_Object->CAN_Handler, CAN_Tx_ID, DM_Motor_CAN_Message_Exit, 8);
}

void Class_Motor_DM_Normal::CAN_Send_Save_Zero()
{
    CAN_Send_Data(CAN_Manage_Object->CAN_Handler, CAN_Tx_ID, DM_Motor_CAN_Message_Save_Zero, 8);
}
```

**作用**: 发送传统模式下的特殊控制命令。

#### 2.3.3 存活检测函数

```cpp
void Class_Motor_DM_Normal::TIM_Alive_PeriodElapsedCallback()
{
    if (Flag == Pre_Flag)
    {
        Motor_DM_Status = Motor_DM_Status_DISABLE;
    }
    else
    {
        Motor_DM_Status = Motor_DM_Status_ENABLE;
    }

    Pre_Flag = Flag;

    if(Motor_DM_Status == Motor_DM_Status_DISABLE)
    {
        CAN_Send_Enter();  // 如果断线，尝试使能电机
    }
}
```

**作用**: 检测电机连接状态，断线时自动尝试重新使能。

#### 2.3.4 发送函数

```cpp
void Class_Motor_DM_Normal::TIM_Send_PeriodElapsedCallback()
{
    if (Rx_Data.Control_Status == Motor_DM_Status_ENABLE)
    {
        // 电机在线，正常控制
        Math_Constrain(&Control_Angle, -Angle_Max, Angle_Max);
        Math_Constrain(&Control_Omega, -Omega_Max, Omega_Max);
        Math_Constrain(&Control_Torque, -Torque_Max, Torque_Max);
        Math_Constrain(&Control_Current, -Current_Max, Current_Max);
        Math_Constrain(&K_P, 0.0f, 500.0f);
        Math_Constrain(&K_D, 0.0f, 5.0f);

        Output();
    }
    else if (Rx_Data.Control_Status == Motor_DM_Status_DISABLE)
    {
        // 电机可能掉线，使能电机
        CAN_Send_Enter();
    }
    else
    {
        // 电机错误，发送清除错误帧
        CAN_Send_Clear_Error();
    }
}
```

**作用**: 根据电机状态执行相应的控制策略。

#### 2.3.5 数据处理函数

```cpp
void Class_Motor_DM_Normal::Data_Process()
{
    Struct_Motor_DM_CAN_Rx_Data_Normal *tmp_buffer = (Struct_Motor_DM_CAN_Rx_Data_Normal *)CAN_Manage_Object->Rx_Buffer.Data;

    // 电机ID不匹配，不处理
    if(tmp_buffer->CAN_ID != (CAN_Tx_ID & 0x0f))
    {
        return;
    }

    // 处理大小端转换
    Math_Endian_Reverse_16((void *)&tmp_buffer->Angle_Reverse, &tmp_encoder);
    tmp_omega = (tmp_buffer->Omega_11_4 << 4) | (tmp_buffer->Omega_3_0_Torque_11_8 >> 4);
    tmp_torque = ((tmp_buffer->Omega_3_0_Torque_11_8 & 0x0f) << 8) | (tmp_buffer->Torque_7_0);

    Rx_Data.Control_Status = static_cast<Enum_Motor_DM_Control_Status_Normal>(tmp_buffer->Control_Status_Enum);

    // 计算圈数（关键逻辑）
    delta_encoder = tmp_encoder - Rx_Data.Pre_Encoder;
    if (delta_encoder < -(1 << 15))  // -32768
    {
        Rx_Data.Total_Round++;  // 正方向转过一圈
    }
    else if (delta_encoder > (1 << 15))  // 32768
    {
        Rx_Data.Total_Round--;  // 反方向转过一圈
    }
    Rx_Data.Total_Encoder = Rx_Data.Total_Round * (1 << 16) + tmp_encoder - ((1 << 15) - 1);

    // 计算各物理量（使用专门的数学转换函数）
    Rx_Data.Now_Angle = (float)(Rx_Data.Total_Encoder) / (float)((1 << 16) - 1) * Angle_Max * 2.0f;
    Rx_Data.Now_Omega = Math_Int_To_Float(tmp_omega, 0x7ff, (1 << 12) - 1, 0, Omega_Max);
    Rx_Data.Now_Torque = Math_Int_To_Float(tmp_torque, 0x7ff, (1 << 12) - 1, 0, Torque_Max);
    Rx_Data.Now_MOS_Temperature = tmp_buffer->MOS_Temperature + CELSIUS_TO_KELVIN;
    Rx_Data.Now_Rotor_Temperature = tmp_buffer->Rotor_Temperature + CELSIUS_TO_KELVIN;

    Rx_Data.Pre_Encoder = tmp_encoder;  // 保存当前编码器值
}
```

**作用**: 解析传统模式的CAN数据包，处理复杂的位字段组合和数值转换。

#### 2.3.6 输出函数

```cpp
void Class_Motor_DM_Normal::Output()
{
    switch (Motor_DM_Control_Method)
    {
    case (Motor_DM_Control_Method_NORMAL_MIT):
    {
        Struct_Motor_DM_CAN_Tx_Data_Normal_MIT *tmp_buffer = (Struct_Motor_DM_CAN_Tx_Data_Normal_MIT *)Tx_Data;

        // 浮点数转整数
        uint16_t tmp_angle = Math_Float_To_Int(Control_Angle, 0, Angle_Max, 0x7fff, (1 << 16) - 1);
        uint16_t tmp_omega = Math_Float_To_Int(Control_Omega, 0, Omega_Max, 0x7ff, (1 << 12) - 1);
        uint16_t tmp_torque = Math_Float_To_Int(Control_Torque, 0, Torque_Max, 0x7ff, (1 << 12) - 1);
        uint16_t tmp_k_p = Math_Float_To_Int(K_P, 0, 500.0f, 0, (1 << 12) - 1);
        uint16_t tmp_k_d = Math_Float_To_Int(K_D, 0, 5.0f, 0, (1 << 12) - 1);

        // 位字段组装
        tmp_buffer->Control_Angle_Reverse = Math_Endian_Reverse_16(&tmp_angle, nullptr);
        tmp_buffer->Control_Omega_11_4 = tmp_omega >> 4;
        tmp_buffer->Control_Omega_3_0_K_P_11_8 = ((tmp_omega & 0x0f) << 4) | (tmp_k_p >> 8);
        tmp_buffer->K_P_7_0 = tmp_k_p & 0xff;
        tmp_buffer->K_D_11_4 = tmp_k_d >> 4;
        tmp_buffer->K_D_3_0_Control_Torque_11_8 = ((tmp_k_d & 0x0f) << 4) | (tmp_torque >> 8);
        tmp_buffer->Control_Torque_7_0 = tmp_torque & 0xff;

        CAN_Send_Data(CAN_Manage_Object->CAN_Handler, CAN_Tx_ID, Tx_Data, 8);

        break;
    }
    case (Motor_DM_Control_Method_NORMAL_ANGLE_OMEGA):
    {
        Struct_Motor_DM_CAN_Tx_Data_Normal_Angle_Omega *tmp_buffer = (Struct_Motor_DM_CAN_Tx_Data_Normal_Angle_Omega *)Tx_Data;
        tmp_buffer->Control_Angle = Control_Angle;  // 直接赋值float
        tmp_buffer->Control_Omega = Control_Omega;
        CAN_Send_Data(CAN_Manage_Object->CAN_Handler, CAN_Tx_ID, Tx_Data, 8);
        break;
    }
    // ... 其他模式处理
    }
}
```

**作用**: 根据不同控制模式将控制参数打包成CAN消息发送。

### 2.4 一拖四模式类实现

#### 2.4.1 数据处理函数

```cpp
void Class_Motor_DM_1_To_4::Data_Process()
{
    Struct_Motor_DM_CAN_Rx_Data_1_To_4 *tmp_buffer = (Struct_Motor_DM_CAN_Rx_Data_1_To_4 *) CAN_Manage_Object->Rx_Buffer.Data;

    // 处理大小端转换
    Math_Endian_Reverse_16((void *) &tmp_buffer->Encoder_Reverse, (void *) &tmp_encoder);
    Math_Endian_Reverse_16((void *) &tmp_buffer->Omega_Reverse, (void *) &tmp_omega);
    Math_Endian_Reverse_16((void *) &tmp_buffer->Current_Reverse, (void *) &tmp_current);

    // 计算圈数（与大疆电机相同）
    int16_t delta_encoder = tmp_encoder - Rx_Data.Pre_Encoder;
    if (delta_encoder < -Encoder_Num_Per_Round / 2)
    {
        Rx_Data.Total_Round++;
    }
    else if (delta_encoder > Encoder_Num_Per_Round / 2)
    {
        Rx_Data.Total_Round--;
    }
    Rx_Data.Total_Encoder = Rx_Data.Total_Round * Encoder_Num_Per_Round + tmp_encoder + Encoder_Offset;

    // 计算各物理量
    Rx_Data.Now_Angle = (float) Rx_Data.Total_Encoder / (float) Encoder_Num_Per_Round * 2.0f * PI;
    Rx_Data.Now_Omega = tmp_omega / 100.0f * RPM_TO_RADPS;  // 除以100恢复原始值
    Rx_Data.Now_Current = tmp_current / 1000.0f;            // 除以1000转换为安培
    Rx_Data.Now_MOS_Temperature = tmp_buffer->MOS_Temperature + CELSIUS_TO_KELVIN;
    Rx_Data.Now_Rotor_Temperature = tmp_buffer->Rotor_Temperature + CELSIUS_TO_KELVIN;

    Rx_Data.Pre_Encoder = tmp_encoder;
}
```

**作用**: 解析一拖四模式的CAN数据包，处理数值转换。

#### 2.4.2 PID计算函数

```cpp
void Class_Motor_DM_1_To_4::PID_Calculate()
{
    switch (Motor_DM_Control_Method)
    {
    case (Motor_DM_Control_Method_1_TO_4_ANGLE):
    {
        // 两环PID：角度环 → 角速度环 → 电流
        PID_Angle.Set_Target(Target_Angle);
        PID_Angle.Set_Now(Rx_Data.Now_Angle);
        PID_Angle.TIM_Calculate_PeriodElapsedCallback();
        Target_Omega = PID_Angle.Get_Out();  // 角度环输出作为角速度环输入

        PID_Omega.Set_Target(Target_Omega + Feedforward_Omega);
        PID_Omega.Set_Now(Rx_Data.Now_Omega);
        PID_Omega.TIM_Calculate_PeriodElapsedCallback();
        Target_Current = PID_Omega.Get_Out();  // 角速度环输出作为电流输出

        break;
    }
    case (Motor_DM_Control_Method_1_TO_4_OMEGA):
    {
        // 单环PID：角速度环 → 电流
        PID_Omega.Set_Target(Target_Omega + Feedforward_Omega);
        PID_Omega.Set_Now(Rx_Data.Now_Omega);
        PID_Omega.TIM_Calculate_PeriodElapsedCallback();
        Target_Current = PID_Omega.Get_Out();

        break;
    }
    case (Motor_DM_Control_Method_1_TO_4_CURRENT):
    {
        // 电流控制，直接输出目标电流
        break;
    }
    default:
    {
        Target_Current = 0.0f;  // 默认停止
        break;
    }
    }
}
```

**作用**: 根据控制方式执行相应的PID控制计算。

## 3. 关键特性分析

### 3.1 两种工作模式

- **传统模式**: 支持MIT、位置速度、速度、EMIT四种控制模式，适合高精度控制
- **一拖四模式**: 类似大疆电机控制方式，支持PID闭环控制

### 3.2 复杂的数据打包/解包

- **传统模式**: 使用复杂的位字段组合，一个CAN消息包含多个控制参数
- **一拖四模式**: 简单的整数传输，易于处理

### 3.3 电机状态管理

- **故障诊断**: 详细的状态枚举，能够识别多种故障
- **自动恢复**: 断线时自动尝试使能，错误时自动清除

## 4. 类的作用域和外设资源

### 4.1 作用域

- **公共作用域(public)**: 提供初始化、数据获取/设置和控制函数
- **保护作用域(protected)**: 内部实现细节，包括数据存储、控制算法等

### 4.2 使用的外设资源

- **CAN接口**: 用于与达妙电机通信，支持CAN1和CAN2
- **定时器**: 用于电机存活检测和控制计算
- **内存资源**: 接收和发送缓冲区存储CAN数据
- **数学库**: 数值转换、约束函数等

### 4.3 工作流程

1. **传统模式**: 初始化时设置控制参数，通过专用命令控制电机，根据状态反馈调整策略
2. **一拖四模式**: 类似大疆电机的控制流程，支持PID闭环控制
3. 用户通过Get/Set系列函数获取/设置电机参数

这个驱动程序提供了完整的达妙电机控制解决方案，支持两种不同的控制模式，适应不同的应用场景需求。