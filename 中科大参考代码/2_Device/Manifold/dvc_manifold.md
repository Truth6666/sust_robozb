# 视觉Manifold代码深度解析

## 1. 头文件分析 (dvc_manifold.h)

### 1.1 文件概述

这是一个用于视觉系统的Manifold驱动头文件，实现了与视觉处理系统之间的双向通信，支持自瞄功能。该系统通过UART与视觉系统通信，接收目标追踪信息并发送机器人状态信息。

### 1.2 包含的头文件

```cpp
#include "1_Middleware/1_Driver/Math/drv_math.h"  // 数学运算库
#include "1_Middleware/1_Driver/UART/drv_uart.h"  // UART驱动库
```

### 1.3 坐标系统说明

```
坐标定义：前x左y上z, 右手系决定旋转, ypr旋转顺序
```

- **前x**: 正前方为X轴正方向
- **左y**: 左方为Y轴正方向  
- **上z**: 上方为Z轴正方向
- **右手系**: 符合右手螺旋法则
- **YPR**: 偏航-俯仰-滚转旋转顺序

### 1.4 枚举类型定义

#### 1.4.1 视觉系统状态枚举

```cpp
enum Enum_Manifold_Status
{
    Manifold_Status_DISABLE = 0,  // 视觉系统断开连接
    Manifold_Status_ENABLE,       // 视觉系统正常连接
};
```

**作用**: 表示视觉Manifold的连接状态。

#### 1.4.2 敌方颜色枚举

```cpp
enum Enum_Manifold_Enemy_Color : uint8_t
{
    Manifold_Enemy_Color_RED = 0,   // 红色敌方
    Manifold_Enemy_Color_BLUE,      // 蓝色敌方
};
```

**作用**: 定义敌方机器人的颜色标识。

#### 1.4.3 敌方机器人ID枚举

```cpp
enum Enum_Manifold_Enemy_ID : uint8_t
{
    Manifold_Enemy_ID_NONE_0 = 0,   // 无目标
    Manifold_Enemy_ID_HERO_1,       // 英雄机器人(1号)
    Manifold_Enemy_ID_ENGINEER_2,   // 工程机器人(2号)
    Manifold_Enemy_ID_INFANTRY_3,   // 步兵机器人(3号)
    Manifold_Enemy_ID_INFANTRY_4,   // 步兵机器人(4号)
    Manifold_Enemy_ID_INFANTRY_5,   // 步兵机器人(5号)
    Manifold_Enemy_ID_SENTRY_7,     // 哨兵机器人(7号)
    Manifold_Enemy_ID_OUTPOST,      // 哨所
    Manifold_Enemy_ID_RUNE,         // 符文
    Manifold_Enemy_ID_UNKNOWN,      // 未知目标
};
```

**作用**: 定义可识别的敌方机器人类型和编号。

#### 1.4.4 自瞄优先级枚举

```cpp
enum Enum_Manifold_Aiming_Priority : uint8_t
{
    Manifold_Aiming_Priority_ARMOR = 0,  // 优先攻击装甲板
    Manifold_Aiming_Priority_RUNE,       // 优先攻击符文
};
```

**作用**: 设置视觉算法的目标优先级策略。

### 1.5 结构体定义

#### 1.5.1 接收数据结构（视觉系统→控制板）

```cpp
struct Struct_Manifold_UART_Rx_Data
{
    uint16_t Frame_Header;                    // 帧头标志
    uint8_t Reserved;                        // 保留字节
    uint8_t Shoot_Flag;                      // 发射标志
    float Gimbal_Pitch_Angle_Increment;      // 云台俯仰角度增量
    float Gimbal_Yaw_Angle_Increment;        // 云台偏航角度增量
    float Gimbal_Pitch_Omega_FeedForward;    // 云台俯仰角速度前馈
    float Gimbal_Yaw_Omega_FeedForward;      // 云台偏航角速度前馈
    Enum_Manifold_Enemy_ID Enemy_ID;         // 敌方机器人ID
    uint16_t Confidence_Level;               // 算法置信度
}__attribute__((packed));
```

**作用**: 定义从视觉系统接收的数据包格式，包含自瞄所需的所有信息。

#### 1.5.2 发送数据结构（控制板→视觉系统）

```cpp
struct Struct_Manifold_UART_Tx_Data
{
    uint16_t Frame_Header;                    // 帧头标志
    Enum_Manifold_Enemy_Color Enemy_Color;   // 敌方颜色
    Enum_Manifold_Aiming_Priority Aiming_Priority; // 自瞄优先级
    float Velocity_X;                        // X轴速度
    float Velocity_Y;                        // Y轴速度
}__attribute__((packed));
```

**作用**: 定义发送给视觉系统的信息，包含机器人状态和配置参数。

### 1.6 主类定义

#### 1.6.1 类成员变量

```cpp
class Class_Manifold
{
protected:
    // 初始化相关常量
    Struct_UART_Manage_Object *UART_Manage_Object;  // 绑定的UART管理对象
    uint16_t Frame_Header;                          // 数据包头标

    // 常量
    // 内部变量
    uint32_t Flag = 0;                              // 当前时刻接收标志
    uint32_t Pre_Flag = 0;                          // 前一时刻接收标志
    float Gimbal_Pitch_Angle_Increment_Max = PI / 6.0f;  // 俯仰角度增量最大值(30度)
    float Gimbal_Yaw_Angle_Increment_Max = PI / 3.0f;    // 偏航角度增量最大值(60度)

    // 云台角度补偿值
    float Gimbal_Pitch_Angle_Offset = 0.004f;      // 俯仰角度补偿
    float Gimbal_Yaw_Angle_Offset = 0.0f;          // 偏航角度补偿

    // 读变量
    Enum_Manifold_Status Manifold_Status = Manifold_Status_DISABLE;  // 视觉系统状态
    Struct_Manifold_UART_Rx_Data Rx_Data;          // 接收数据
    float Target_Gimbal_Pitch = 0.0f;              // 目标俯仰角度
    float Target_Gimbal_Yaw = 0.0f;                // 目标偏航角度

    // 写变量
    Struct_Manifold_UART_Tx_Data Tx_Data;          // 发送数据
    float Now_Gimbal_Pitch = 0.0f;                 // 当前俯仰角度
    float Now_Gimbal_Yaw = 0.0f;                   // 当前偏航角度

    // 内部函数
    void Data_Process(uint16_t Length);            // 数据处理函数
    void Output();                                 // 输出函数
};
```

#### 1.6.2 公共接口函数声明

```cpp
public:
    // 初始化函数
    void Init(UART_HandleTypeDef *__huart, uint16_t __Frame_Header = 0xcdab);
    
    // 状态获取函数
    inline Enum_Manifold_Status Get_Status();
    
    // 接收数据获取函数
    inline uint8_t Get_Shoot_Flag();                                  // 获取发射标志
    inline float Get_Gimbal_Pitch_Angle_Increment();                 // 获取俯仰角度增量
    inline float Get_Gimbal_Yaw_Angle_Increment();                   // 获取偏航角度增量
    inline float Get_Gimbal_Pitch_Omega_FeedForward();               // 获取俯仰角速度前馈
    inline float Get_Gimbal_Yaw_Omega_FeedForward();                 // 获取偏航角速度前馈
    inline Enum_Manifold_Enemy_ID Get_Enemy_ID();                    // 获取敌方ID
    inline uint16_t Get_Confidence_Level();                          // 获取置信度
    inline float Get_Target_Gimbal_Yaw();                            // 获取目标偏航角度
    inline float Get_Target_Gimbal_Pitch();                          // 获取目标俯仰角度
    
    // 发送数据设置函数
    inline void Set_Aiming_Priority(Enum_Manifold_Aiming_Priority __Aiming_Priority);  // 设置自瞄优先级
    inline void Set_Velocity_X(float __Velocity_X);                  // 设置X轴速度
    inline void Set_Velocity_Y(float __Velocity_Y);                  // 设置Y轴速度
    inline void Set_Enemy_Color(Enum_Manifold_Enemy_Color __Self_Color);  // 设置敌方颜色
    inline void Set_Now_Gimbal_Yaw(float __Now_Gimbal_Yaw);         // 设置当前偏航角度
    inline void Set_Now_Gimbal_Pitch(float __Now_Gimbal_Pitch);     // 设置当前俯仰角度
    
    // 回调函数
    void UART_RxCpltCallback(uint8_t *Rx_Data, uint16_t Length);    // UART接收完成回调
    void TIM_1000ms_Alive_PeriodElapsedCallback();                  // 定时器存活检测回调
    void TIM_10ms_Send_PeriodElapsedCallback();                     // 定时器发送回调
```

## 2. 实现文件分析 (dvc_manifold.cpp)

### 2.1 初始化函数

```cpp
void Class_Manifold::Init(UART_HandleTypeDef *huart, uint16_t __Frame_Header)
{
    // 根据UART实例绑定对应的管理对象
    if (huart->Instance == USART1) {
        UART_Manage_Object = &UART1_Manage_Object;
    }
    else if (huart->Instance == USART2) {
        UART_Manage_Object = &UART2_Manage_Object;
    }
    // ... 其他UART端口判断
    
    // 设置帧头标志
    Frame_Header = __Frame_Header;
    Tx_Data.Frame_Header = __Frame_Header;  // 同时设置发送数据的帧头
}
```

**作用**: 初始化视觉Manifold系统，绑定UART端口并设置通信协议参数。

### 2.2 UART接收回调函数

```cpp
void Class_Manifold::UART_RxCpltCallback(uint8_t *Rx_Data, uint16_t Length)
{
    // 滑动窗口，判断视觉Manifold是否在线
    Flag += 1;
    Data_Process(Length);
}
```

**作用**: UART接收完成时的回调函数，增加接收标志并处理接收到的数据。

### 2.3 定时器存活检测函数

```cpp
void Class_Manifold::TIM_1000ms_Alive_PeriodElapsedCallback()
{
    // 判断该时间段内是否接收过视觉Manifold数据
    if (Flag == Pre_Flag)
    {
        // 视觉Manifold断开连接
        Manifold_Status = Manifold_Status_DISABLE;
        UART_Reinit(UART_Manage_Object->UART_Handler);  // 重新初始化UART
    }
    else
    {
        // 视觉Manifold保持连接
        Manifold_Status = Manifold_Status_ENABLE;
    }
    Pre_Flag = Flag;  // 更新上一时刻标志
}
```

**作用**: 每1000ms执行一次，检测视觉系统是否在线。

### 2.4 定时器发送函数

```cpp
void Class_Manifold::TIM_10ms_Send_PeriodElapsedCallback()
{
    Output();  // 准备发送数据到UART缓冲区
    
    // 发送数据到UART
    UART_Send_Data(UART_Manage_Object->UART_Handler, UART_Manage_Object->Tx_Buffer, 12);
}
```

**作用**: 每10ms执行一次，将控制板信息发送给视觉系统。

### 2.5 数据处理函数（核心功能）

```cpp
void Class_Manifold::Data_Process(uint16_t Length)
{
    // 获取接收缓冲区数据
    Struct_Manifold_UART_Rx_Data *tmp_buffer = (Struct_Manifold_UART_Rx_Data *) UART_Manage_Object->Rx_Buffer;

    // 帧头校验
    if (tmp_buffer->Frame_Header != Frame_Header)
    {
        return;
    }

    // 直接复制发射标志
    Rx_Data.Shoot_Flag = tmp_buffer->Shoot_Flag;

    // 处理俯仰角度增量
    if(isnormal(tmp_buffer->Gimbal_Pitch_Angle_Increment) == true)
    {
        // 添加补偿值并进行限幅
        Rx_Data.Gimbal_Pitch_Angle_Increment = tmp_buffer->Gimbal_Pitch_Angle_Increment + Gimbal_Pitch_Angle_Offset;
        if(Rx_Data.Gimbal_Pitch_Angle_Increment > Gimbal_Pitch_Angle_Increment_Max)
        {
            Rx_Data.Gimbal_Pitch_Angle_Increment = Gimbal_Pitch_Angle_Increment_Max;
        }
        else if(Rx_Data.Gimbal_Pitch_Angle_Increment < -Gimbal_Pitch_Angle_Increment_Max)
        {
            Rx_Data.Gimbal_Pitch_Angle_Increment = -Gimbal_Pitch_Angle_Increment_Max;
        }
    }
    else
    {
        Rx_Data.Gimbal_Pitch_Angle_Increment = 0.0f;  // 异常值设为0
    }

    // 处理偏航角度增量
    if(isnormal(tmp_buffer->Gimbal_Yaw_Angle_Increment) == true)
    {
        // 添加补偿值并进行限幅
        Rx_Data.Gimbal_Yaw_Angle_Increment = tmp_buffer->Gimbal_Yaw_Angle_Increment + Gimbal_Yaw_Angle_Offset;
        if(Rx_Data.Gimbal_Yaw_Angle_Increment > Gimbal_Yaw_Angle_Increment_Max)
        {
            Rx_Data.Gimbal_Yaw_Angle_Increment = Gimbal_Yaw_Angle_Increment_Max;
        }
        else if(Rx_Data.Gimbal_Yaw_Angle_Increment < -Gimbal_Yaw_Angle_Increment_Max)
        {
            Rx_Data.Gimbal_Yaw_Angle_Increment = -Gimbal_Yaw_Angle_Increment_Max;
        }
    }
    else
    {
        Rx_Data.Gimbal_Yaw_Angle_Increment = 0.0f;  // 异常值设为0
    }

    // 处理俯仰角速度前馈
    if(isnormal(tmp_buffer->Gimbal_Pitch_Omega_FeedForward) == true)
    {
        Rx_Data.Gimbal_Pitch_Omega_FeedForward = tmp_buffer->Gimbal_Pitch_Omega_FeedForward;
    }
    else
    {
        Rx_Data.Gimbal_Pitch_Omega_FeedForward = 0.0f;  // 异常值设为0
    }

    // 处理偏航角速度前馈
    if(isnormal(tmp_buffer->Gimbal_Yaw_Omega_FeedForward) == true)
    {
        Rx_Data.Gimbal_Yaw_Omega_FeedForward = tmp_buffer->Gimbal_Yaw_Omega_FeedForward;
    }
    else
    {
        Rx_Data.Gimbal_Yaw_Omega_FeedForward = 0.0f;  // 异常值设为0
    }

    // 复制其他数据
    Rx_Data.Enemy_ID = tmp_buffer->Enemy_ID;
    Rx_Data.Confidence_Level = tmp_buffer->Confidence_Level;

    // 计算目标角度（当前角度 + 增量）
    Target_Gimbal_Yaw = Now_Gimbal_Yaw + Rx_Data.Gimbal_Yaw_Angle_Increment;
    Target_Gimbal_Pitch = Now_Gimbal_Pitch + Rx_Data.Gimbal_Pitch_Angle_Increment;
}
```

**作用**: 解析和处理从视觉系统接收到的数据，进行异常值检测、补偿和限幅处理。

### 2.6 输出函数

```cpp
void Class_Manifold::Output()
{
    // 获取发送缓冲区
    Struct_Manifold_UART_Tx_Data *tmp_buffer = (Struct_Manifold_UART_Tx_Data *) UART_Manage_Object->Tx_Buffer;

    // 复制发送数据到UART缓冲区
    tmp_buffer->Frame_Header = Tx_Data.Frame_Header;
    tmp_buffer->Aiming_Priority = Tx_Data.Aiming_Priority;
    tmp_buffer->Velocity_X = Tx_Data.Velocity_X;
    tmp_buffer->Velocity_Y = Tx_Data.Velocity_Y;
    tmp_buffer->Enemy_Color = Tx_Data.Enemy_Color;
}
```

**作用**: 将控制板的发送数据复制到UART发送缓冲区，准备发送给视觉系统。

## 3. 关键特性分析

### 3.1 双向通信协议

- **接收方向**: 控制板 ← 视觉系统（自瞄指令、目标信息）
- **发送方向**: 控制板 → 视觉系统（机器人状态、配置参数）

### 3.2 安全保护机制

- **角度限幅**: 俯仰角±30°，偏航角±60°
- **异常值检测**: 使用`isnormal()`检测浮点数有效性
- **补偿机制**: 角度补偿值消除系统误差

### 3.3 实时性能

- **接收频率**: 由UART中断触发，实时处理
- **发送频率**: 每10ms发送一次状态信息
- **存活检测**: 每1000ms检测连接状态

## 4. 类的作用域和外设资源

### 4.1 作用域

- **公共作用域(public)**: 提供对外接口，包括初始化、数据获取/设置和回调函数
- **保护作用域(protected)**: 内部实现细节，包括数据存储、处理算法等

### 4.2 使用的外设资源

- **UART接口**: 用于与视觉系统通信，支持USART1-8和UART4-8
- **定时器**: 用于1000ms设备存活检测和10ms状态发送
- **内存资源**: 接收和发送缓冲区存储通信数据

### 4.3 工作流程

1. 初始化时绑定UART端口和设置通信协议
2. UART接收数据触发回调函数，处理自瞄指令
3. 10ms定时器发送机器人状态给视觉系统
4. 1000ms定时器检测视觉系统在线状态
5. 用户通过Get/Set系列函数获取/设置相关参数

这个驱动程序实现了完整的视觉自瞄通信协议，支持实时目标追踪、角度控制和状态反馈功能，是机器人自瞄系统的核心组件。