# 大疆电机驱动代码深度解析

## 1. 头文件分析 (dvc_motor_dji.h)

### 1.1 文件概述

这是一个用于大疆电机（GM6020、C610、C620）的通用驱动头文件，支持多种控制方式和功率限制功能，版本1.2新增了功率控制接口。

### 1.2 包含的头文件

```cpp
#include "1_Middleware/2_Algorithm/PID/alg_pid.h"  // PID算法库
#include "1_Middleware/1_Driver/CAN/drv_can.h"    // CAN驱动库
```

### 1.3 枚举类型定义

#### 1.3.1 电机状态枚举

```cpp
enum Enum_Motor_DJI_Status
{
    Motor_DJI_Status_DISABLE = 0,  // 电机断开连接
    Motor_DJI_Status_ENABLE,       // 电机正常连接
};
```

**作用**: 表示大疆电机的连接状态。

#### 1.3.2 电机ID枚举

```cpp
enum Enum_Motor_DJI_ID
{
    Motor_DJI_ID_0x201 = 1,  // C6系列电机ID
    Motor_DJI_ID_0x202,
    Motor_DJI_ID_0x203,
    Motor_DJI_ID_0x204,
    Motor_DJI_ID_0x205,      // GM6020电机ID
    Motor_DJI_ID_0x206,
    Motor_DJI_ID_0x207,
    Motor_DJI_ID_0x208,
    Motor_DJI_ID_0x209,
    Motor_DJI_ID_0x20A,
    Motor_DJI_ID_0x20B,
};
```

**作用**: 定义大疆电机在CAN总线上的ID，C6系列为0x201-0x208，GM系列为0x205-0x20B。

#### 1.3.3 控制方式枚举

```cpp
enum Enum_Motor_DJI_Control_Method
{
    Motor_DJI_Control_Method_VOLTAGE = 0,  // 电压控制
    Motor_DJI_Control_Method_CURRENT,      // 电流控制
    Motor_DJI_Control_Method_TORQUE,       // 转矩控制
    Motor_DJI_Control_Method_OMEGA,        // 角速度控制
    Motor_DJI_Control_Method_ANGLE,        // 角度控制
};
```

**作用**: 定义电机的控制模式，支持多层嵌套控制。

#### 1.3.4 GM6020驱动版本枚举

```cpp
enum Enum_Motor_DJI_GM6020_Driver_Version
{
    Motor_DJI_GM6020_Driver_Version_DEFAULT = 0,  // 旧版电压驱动
    Motor_DJI_GM6020_Driver_Version_2023,        // 2023新版电流驱动
};
```

**作用**: 区分GM6020电机的不同驱动方式。

#### 1.3.5 功率限制状态枚举

```cpp
enum Enum_Motor_DJI_Power_Limit_Status
{
    Motor_DJI_Power_Limit_Status_DISABLE = 0,  // 功率限制关闭
    Motor_DJI_Power_Limit_Status_ENABLE,       // 功率限制开启
};
```

**作用**: 控制是否启用功率限制功能。

### 1.4 结构体定义

#### 1.4.1 CAN原始数据结构

```cpp
struct Struct_Motor_DJI_CAN_Rx_Data
{
    uint16_t Encoder_Reverse;    // 编码器值（大小端转换）
    int16_t Omega_Reverse;       // 角速度（大小端转换）
    int16_t Current_Reverse;     // 电流（大小端转换）
    uint8_t Temperature;         // 温度
    uint8_t Reserved;            // 保留字节
} __attribute__((packed));
```

**作用**: 定义从CAN总线接收的原始电机数据包格式。

#### 1.4.2 处理后数据结构

```cpp
struct Struct_Motor_DJI_Rx_Data
{
    float Now_Angle;           // 当前角度
    float Now_Omega;           // 当前角速度
    float Now_Current;         // 当前电流
    float Now_Temperature;     // 当前温度
    float Now_Power;           // 当前功率
    uint32_t Pre_Encoder;      // 前一时刻编码器值
    int32_t Total_Encoder;     // 总编码器值（包含圈数）
    int32_t Total_Round;       // 总圈数
};
```

**作用**: 存储解析和计算后的电机状态数据。

### 1.5 三个电机类定义

#### 1.5.1 GM6020无刷电机类

```cpp
class Class_Motor_DJI_GM6020
{
public:
    // 三层PID控制器
    Class_PID PID_Angle;      // 角度环PID
    Class_PID PID_Omega;      // 角速度环PID
    Class_PID PID_Current;    // 电流环PID

    // 公共接口函数...
};
```

**特点**: 支持电压/电流双模式控制，具备完整的三环控制。

#### 1.5.2 C610无刷电调类

```cpp
class Class_Motor_DJI_C610
{
public:
    // 两层PID控制器
    Class_PID PID_Angle;      // 角度环PID
    Class_PID PID_Omega;      // 角速度环PID

    // 公共接口函数...
};
```

**特点**: 内置电流环，只需控制目标电流。

#### 1.5.3 C620无刷电调类

```cpp
class Class_Motor_DJI_C620
{
public:
    // 两层PID控制器
    Class_PID PID_Angle;      // 角度环PID
    Class_PID PID_Omega;      // 角速度环PID

    // 公共接口函数...
};
```

**特点**: 支持功率限制功能的高级电机驱动。

## 2. 实现文件分析 (dvc_motor_dji.cpp)

### 2.1 辅助函数

#### 2.1.1 CAN发送缓冲区分配函数

```cpp
uint8_t *allocate_tx_data(CAN_HandleTypeDef *hcan, Enum_Motor_DJI_ID __CAN_ID, Enum_Motor_DJI_GM6020_Driver_Version __DJI_Motor_Driver_Version)
{
    uint8_t *tmp_tx_data_ptr;
    if (hcan == &hcan1)
    {
        switch (__CAN_ID)
        {
        case (Motor_DJI_ID_0x201):
            tmp_tx_data_ptr = &(CAN1_0x200_Tx_Data[0]);  // CAN1, ID 0x201, 使用0x200组
            break;
        case (Motor_DJI_ID_0x205):
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
                tmp_tx_data_ptr = &(CAN1_0x1ff_Tx_Data[0]);  // 旧版驱动使用0x1ff组
            else if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_2023)
                tmp_tx_data_ptr = &(CAN1_0x1fe_Tx_Data[0]);  // 新版驱动使用0x1fe组
            break;
        // ... 其他ID处理
        }
    }
    // ... CAN2处理
    return (tmp_tx_data_ptr);
}
```

**作用**: 根据CAN总线、电机ID和驱动版本，分配对应的发送缓冲区位置。

#### 2.1.2 功率计算函数

```cpp
float power_calculate(float K_0, float K_1, float K_2, float A, float Current, float Omega)
{
    return (K_0 * Current * Omega + K_1 * Omega * Omega + K_2 * Current * Current + A);
}
```

**作用**: 使用电机模型参数计算功率，公式为P = K₀Iω + K₁ω² + K₂I² + A。

### 2.2 GM6020类实现

#### 2.2.1 初始化函数

```cpp
void Class_Motor_DJI_GM6020::Init(CAN_HandleTypeDef *hcan, Enum_Motor_DJI_ID __CAN_Rx_ID, ...)
{
    if (hcan->Instance == CAN1)
        CAN_Manage_Object = &CAN1_Manage_Object;
    else if (hcan->Instance == CAN2)
        CAN_Manage_Object = &CAN2_Manage_Object;
    
    CAN_Rx_ID = __CAN_Rx_ID;
    // 设置各种参数...
    Tx_Data = allocate_tx_data(hcan, __CAN_Rx_ID, __Driver_Version);  // 分配发送缓冲区
}
```

**作用**: 初始化GM6020电机，绑定CAN总线并分配发送缓冲区。

#### 2.2.2 CAN接收回调函数

```cpp
void Class_Motor_DJI_GM6020::CAN_RxCpltCallback(uint8_t *Rx_Data)
{
    Flag += 1;  // 增加接收标志
    Data_Process();  // 处理数据
}
```

**作用**: CAN接收完成时的回调函数。

#### 2.2.3 定时器计算回调函数

```cpp
void Class_Motor_DJI_GM6020::TIM_Calculate_PeriodElapsedCallback()
{
    PID_Calculate();  // PID计算

    if (Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
    {
        float tmp_value = Target_Voltage + Feedforward_Voltage;
        Math_Constrain(&tmp_value, -Voltage_Max, Voltage_Max);
        Out = tmp_value * Voltage_To_Out;  // 电压转输出
    }
    else if (Driver_Version == Motor_DJI_GM6020_Driver_Version_2023)
    {
        float tmp_value = Target_Current + Feedforward_Current;
        Math_Constrain(&tmp_value, -Current_Max, Current_Max);
        Out = tmp_value * Current_To_Out;  // 电流转输出
    }

    // 计算功率估计值
    Power_Estimate = power_calculate(Power_K_0, Power_K_1, Power_K_2, Power_A, Target_Current, Rx_Data.Now_Omega);

    Output();  // 输出到CAN

    if (Power_Limit_Status == Motor_DJI_Power_Limit_Status_DISABLE)
    {
        Feedforward_Voltage = 0.0f;
        Feedforward_Current = 0.0f;
        Feedforward_Omega = 0.0f;
    }
}
```

**作用**: 执行PID控制计算、功率估计和输出。

#### 2.2.4 数据处理函数

```cpp
void Class_Motor_DJI_GM6020::Data_Process()
{
    Struct_Motor_DJI_CAN_Rx_Data *tmp_buffer = (Struct_Motor_DJI_CAN_Rx_Data *) CAN_Manage_Object->Rx_Buffer.Data;

    // 处理大小端转换
    Math_Endian_Reverse_16((void *) &tmp_buffer->Encoder_Reverse, (void *) &tmp_encoder);
    Math_Endian_Reverse_16((void *) &tmp_buffer->Omega_Reverse, (void *) &tmp_omega);
    Math_Endian_Reverse_16((void *) &tmp_buffer->Current_Reverse, (void *) &tmp_current);

    // 计算圈数（关键逻辑）
    delta_encoder = tmp_encoder - Rx_Data.Pre_Encoder;
    if (delta_encoder < -Encoder_Num_Per_Round / 2)
    {
        Rx_Data.Total_Round++;  // 正方向转过一圈
    }
    else if (delta_encoder > Encoder_Num_Per_Round / 2)
    {
        Rx_Data.Total_Round--;  // 反方向转过一圈
    }
    Rx_Data.Total_Encoder = Rx_Data.Total_Round * Encoder_Num_Per_Round + tmp_encoder + Encoder_Offset;

    // 计算各物理量
    Rx_Data.Now_Angle = (float) Rx_Data.Total_Encoder / (float) Encoder_Num_Per_Round * 2.0f * PI;
    Rx_Data.Now_Omega = (float) tmp_omega * RPM_TO_RADPS;
    Rx_Data.Now_Current = tmp_current / Current_To_Out;
    Rx_Data.Now_Temperature = tmp_buffer->Temperature + CELSIUS_TO_KELVIN;
    Rx_Data.Now_Power = power_calculate(...);

    Rx_Data.Pre_Encoder = tmp_encoder;  // 保存当前编码器值
}
```

**作用**: 解析CAN数据，处理圈数累计，计算电机状态。

#### 2.2.5 PID计算函数

```cpp
void Class_Motor_DJI_GM6020::PID_Calculate()
{
    switch (Motor_DJI_Control_Method)
    {
    case (Motor_DJI_Control_Method_ANGLE):
    {
        if (Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
        {
            // 三环控制：角度环 → 角速度环 → 电流环 → 电压
            PID_Angle.Set_Target(Target_Angle);
            PID_Angle.Set_Now(Rx_Data.Now_Angle);
            PID_Angle.TIM_Calculate_PeriodElapsedCallback();
            Target_Omega = PID_Angle.Get_Out();  // 角度环输出作为角速度环输入

            PID_Omega.Set_Target(Target_Omega + Feedforward_Omega);
            PID_Omega.Set_Now(Rx_Data.Now_Omega);
            PID_Omega.TIM_Calculate_PeriodElapsedCallback();
            Target_Current = PID_Omega.Get_Out();  // 角速度环输出作为电流环输入

            PID_Current.Set_Target(Target_Current + Feedforward_Current);
            PID_Current.Set_Now(Rx_Data.Now_Current);
            PID_Current.TIM_Calculate_PeriodElapsedCallback();
            Target_Voltage = PID_Current.Get_Out();  // 电流环输出作为电压
        }
        else if (Driver_Version == Motor_DJI_GM6020_Driver_Version_2023)
        {
            // 二环控制：角度环 → 角速度环 → 电流
            PID_Angle.Set_Target(Target_Angle);
            PID_Angle.Set_Now(Rx_Data.Now_Angle);
            PID_Angle.TIM_Calculate_PeriodElapsedCallback();
            Target_Omega = PID_Angle.Get_Out();

            PID_Omega.Set_Target(Target_Omega + Feedforward_Omega);
            PID_Omega.Set_Now(Rx_Data.Now_Omega);
            PID_Omega.TIM_Calculate_PeriodElapsedCallback();
            Target_Current = PID_Omega.Get_Out();
        }
        break;
    }
    // ... 其他控制模式
    }
}
```

**作用**: 根据控制方式执行相应的PID控制计算，支持多层嵌套控制。

#### 2.2.6 功率限制控制函数

```cpp
void Class_Motor_DJI_GM6020::Power_Limit_Control()
{
    if (Power_Estimate > 0.0f)
    {
        if (Power_Factor >= 1.0f)
        {
            // 无需功率控制
        }
        else
        {
            // 解一元二次方程求限制电流
            float a = Power_K_2;
            float b = Power_K_0 * Rx_Data.Now_Omega;
            float c = Power_A + Power_K_1 * Rx_Data.Now_Omega * Rx_Data.Now_Omega - Power_Factor * Power_Estimate;
            
            float delta = b * b - 4 * a * c;
            if (delta < 0.0f)
            {
                Target_Current = 0.0f;  // 无解
            }
            else
            {
                arm_sqrt_f32(delta, &h);
                float result_1 = (-b + h) / (2.0f * a);
                float result_2 = (-b - h) / (2.0f * a);

                // 选择合适的电流值
                if ((result_1 > 0.0f && result_2 < 0.0f) || (result_1 < 0.0f && result_2 > 0.0f))
                {
                    if ((Target_Current > 0.0f && result_1 > 0.0f) || (Target_Current < 0.0f && result_1 < 0.0f))
                        Target_Current = result_1;
                    else
                        Target_Current = result_2;
                }
                else
                {
                    if (Math_Abs(result_1) < Math_Abs(result_2))
                        Target_Current = result_1;
                    else
                        Target_Current = result_2;
                }
            }
        }
    }
}
```

**作用**: 通过解二次方程动态调整目标电流，限制电机功率不超过设定阈值。

#### 2.2.7 输出函数

```cpp
void Class_Motor_DJI_GM6020::Output()
{
    Tx_Data[0] = (int16_t) Out >> 8;    // 高8位
    Tx_Data[1] = (int16_t) Out;         // 低8位
}
```

**作用**: 将控制输出值写入CAN发送缓冲区。

### 2.3 C610和C620类

C610和C620的实现与GM6020类似，主要区别在于：

- **C610**: 只有两层PID（角度→角速度→电流），内置电流环
- **C620**: 支持功率限制，与GM6020类似但只有两层PID
- **减速比**: C610默认36，C620默认3591/187≈19.15

## 3. 关键特性分析

### 3.1 多层嵌套控制

- **GM6020**: 支持电压/电流双模式的三环控制
- **C610/C620**: 内置电流环的两环控制

### 3.2 圈数累计算法

通过编码器差值判断方向，实现连续角度跟踪，解决编码器跳变问题。

### 3.3 功率限制机制

使用电机模型参数建立功率估算，通过数学方法限制输出电流。

### 3.4 前馈控制

支持速度、电流、电压前馈，提高动态响应性能。

## 4. 类的作用域和外设资源

### 4.1 作用域

- **公共作用域(public)**: 提供PID控制器、初始化、数据获取/设置和回调函数
- **保护作用域(protected)**: 内部实现细节，包括数据存储、控制算法等

### 4.2 使用的外设资源

- **CAN接口**: 用于与大疆电机通信，支持CAN1和CAN2
- **定时器**: 用于100ms设备存活检测和控制计算
- **内存资源**: 接收和发送缓冲区存储CAN数据
- **数学库**: PID控制、约束函数、平方根计算等

### 4.3 工作流程

1. 初始化时绑定CAN总线并分配缓冲区
2. CAN接收数据触发回调函数，处理电机状态
3. 定时器执行PID控制计算和功率限制
4. 100ms定时器检测电机在线状态
5. 用户通过Get/Set系列函数获取/设置电机参数

这个驱动程序提供了完整的多类型大疆电机控制解决方案，支持多种控制模式和高级功能，是机器人控制系统的核心组件。