# WIT姿态传感器代码深度解析

## 1. 头文件分析 (dvc_ahrs_wit.h)

### 1.1 文件概述

这是一个用于WIT姿态传感器的驱动头文件，版本1.1，新增了粘包处理功能。该传感器通过UART接口提供多种类型的姿态和运动数据。

### 1.2 包含的头文件

```cpp
#include "1_Middleware/1_Driver/Math/drv_math.h"  // 数学运算库
#include "1_Middleware/1_Driver/UART/drv_uart.h"  // UART驱动库
```

### 1.3 枚举类型定义

#### 1.3.1 设备状态枚举

```cpp
enum Enum_AHRS_WIT_Status
{
    AHRS_WIT_Status_DISABLE = 0,  // 设备禁用状态
    AHRS_WIT_Status_ENABLE,       // 设备启用状态
};
```

**作用**: 定义WIT姿态传感器的工作状态，用于判断设备是否正常连接。

#### 1.3.2 数据类型枚举

```cpp
enum Enum_AHRS_WIT_Data_Type : uint8_t
{
    AHRS_WIT_Data_Type_TIME = 0x50,              // 时间数据
    AHRS_WIT_Data_Type_ACCELERATE,               // 加速度数据
    AHRS_WIT_Data_Type_OMEGA,                    // 角速度数据
    AHRS_WIT_Data_Type_ANGLE,                    // 角度数据
    AHRS_WIT_Data_Type_MAGNETIC,                 // 磁场数据
    AHRS_WIT_Data_Type_PORT,                     // 端口数据
    AHRS_WIT_Data_Type_PRESSURE_ALTITUDE,        // 压力高度数据
    AHRS_WIT_Data_Type_LONGITUDE_LATITUDE,       // 经纬度数据
    AHRS_WIT_Data_Type_GROUND_SPEED,             // 地面速度数据
    AHRS_WIT_Data_Type_QUATERNION,               // 四元数数据
    AHRS_WIT_Data_Type_GPS,                      // GPS数据
    AHRS_WIT_Data_Type_REGISTER,                 // 寄存器数据
};
```

**作用**: 定义WIT传感器支持的各种数据类型，每个类型对应不同的传感器数据。

### 1.4 结构体定义

#### 1.4.1 UART原始数据结构

```cpp
struct Struct_AHRS_WIT_UART_Data
{
    uint8_t Frame_Header;                    // 帧头标志
    Enum_AHRS_WIT_Data_Type Data_Type;       // 数据类型
    uint8_t Data[8];                         // 数据内容（固定8字节）
    uint8_t Checksum;                        // 校验和
}__attribute__((packed));                    // 紧凑打包，避免内存对齐
```

**作用**: 定义WIT传感器的UART数据包格式，总长度为11字节（1+1+8+1）。

#### 1.4.2 加速度数据结构

```cpp
struct Struct_AHRS_WIT_Data_Accelerate
{
    float Accelerate_X;    // X轴加速度
    float Accelerate_Y;    // Y轴加速度
    float Accelerate_Z;    // Z轴加速度
    float Temperature;     // 温度
};
```

**作用**: 存储加速度传感器的三轴加速度值和温度数据。

#### 1.4.3 角速度数据结构

```cpp
struct Struct_AHRS_WIT_Data_Omega
{
    float Omega_X;         // X轴角速度
    float Omega_Y;         // Y轴角速度
    float Omega_Z;         // Z轴角速度
    float Voltage;         // 电压
};
```

**作用**: 存储陀螺仪的三轴角速度值和供电电压。

#### 1.4.4 角度数据结构

```cpp
struct Struct_AHRS_WIT_Data_Angle
{
    float Angle_Roll;      // 滚转角度
    float Angle_Pitch;     // 俯仰角度
    float Angle_Yaw;       // 航向角度
    uint16_t Version;      // 版本号
};
```

**作用**: 存储计算后的欧拉角（姿态角）和固件版本信息。

#### 1.4.5 四元数数据结构

```cpp
struct Struct_AHRS_WIT_Data_Quaternion
{
    float Q_0;             // 四元数w分量（实部）
    float Q_1;             // 四元数x分量
    float Q_2;             // 四元数y分量
    float Q_3;             // 四元数z分量
};
```

**作用**: 存储姿态解算得到的四元数值，用于无奇点的姿态表示。

### 1.5 主类定义

#### 1.5.1 类成员变量

```cpp
class Class_AHRS_WIT
{
protected:
    // 初始化相关常量
    Struct_UART_Manage_Object *UART_Manage_Object;  // 绑定的UART管理对象
    uint8_t Frame_Header;                           // 数据包头标（注意：这里声明为uint8_t，但Init函数参数是uint16_t）

    // 内部变量
    uint32_t Flag = 0;                              // 当前时刻接收标志
    uint32_t Pre_Flag = 0;                          // 前一时刻接收标志

    // 读变量
    Enum_AHRS_WIT_Status WIT_AHRS_Status = AHRS_WIT_Status_DISABLE;  // 设备状态
    Struct_AHRS_WIT_Data_Accelerate Data_Accelerate;                // 加速度数据
    Struct_AHRS_WIT_Data_Omega Data_Omega;                          // 角速度数据
    Struct_AHRS_WIT_Data_Angle Data_Angle;                          // 角度数据
    Struct_AHRS_WIT_Data_Quaternion Data_Quaternion;                // 四元数数据

    // 内部函数
    void Data_Process(uint16_t Length);                              // 数据处理函数
};
```

#### 1.5.2 公共接口函数声明

```cpp
public:
    // 初始化函数
    void Init(UART_HandleTypeDef *huart, uint16_t __Frame_Header = 0x55);
    
    // 状态获取函数
    inline Enum_AHRS_WIT_Status Get_Status();
    
    // 加速度数据获取函数
    inline float Get_Accelerate_X();  // 获取X轴加速度
    inline float Get_Accelerate_Y();  // 获取Y轴加速度
    inline float Get_Accelerate_Z();  // 获取Z轴加速度
    
    // 角速度数据获取函数
    inline float Get_Omega_X();       // 获取X轴角速度
    inline float Get_Omega_Y();       // 获取Y轴角速度
    inline float Get_Omega_Z();       // 获取Z轴角速度
    
    // 角度数据获取函数
    inline float Get_Angle_Roll();    // 获取滚转角度
    inline float Get_Angle_Pitch();   // 获取俯仰角度
    inline float Get_Angle_Yaw();     // 获取航向角度
    
    // 四元数数据获取函数
    inline float Get_Q_0();           // 获取四元数w分量
    inline float Get_Q_1();           // 获取四元数x分量
    inline float Get_Q_2();           // 获取四元数y分量
    inline float Get_Q_3();           // 获取四元数z分量
    
    // 回调函数
    void UART_RxCpltCallback(uint8_t *Rx_Data, uint16_t Length);  // UART接收完成回调
    void TIM_100ms_Alive_PeriodElapsedCallback();                 // 定时器存活检测回调
```

## 2. 实现文件分析 (dvc_ahrs_wit.cpp)

### 2.1 初始化函数

```cpp
void Class_AHRS_WIT::Init(UART_HandleTypeDef *huart, uint16_t __Frame_Header)
{
    // 根据UART实例绑定对应的管理对象
    if (huart->Instance == USART1) {
        UART_Manage_Object = &UART1_Manage_Object;
    }
    else if (huart->Instance == USART2) {
        UART_Manage_Object = &UART2_Manage_Object;
    }
    // ... 其他UART端口判断
    
    // 设置帧头标志（注意：这里存在类型转换问题）
    Frame_Header = __Frame_Header;
}
```

**作用**: 初始化WIT姿态传感器，绑定指定的UART端口并设置通信协议的帧头标志。

### 2.2 UART接收回调函数

```cpp
void Class_AHRS_WIT::UART_RxCpltCallback(uint8_t *Rx_Data, uint16_t Length)
{
    // 滑动窗口，判断陀螺仪是否在线
    Flag += 1;
    Data_Process(Length);
}
```

**作用**: UART接收完成时的回调函数，增加接收标志并处理接收到的数据。

### 2.3 定时器存活检测函数

```cpp
void Class_AHRS_WIT::TIM_100ms_Alive_PeriodElapsedCallback()
{
    // 判断该时间段内是否接收过陀螺仪数据
    if (Flag == Pre_Flag) {
        // 陀螺仪断开连接
        WIT_AHRS_Status = AHRS_WIT_Status_DISABLE;
        UART_Reinit(UART_Manage_Object->UART_Handler);  // 重新初始化UART
    } else {
        // 陀螺仪保持连接
        WIT_AHRS_Status = AHRS_WIT_Status_ENABLE;
    }
    Pre_Flag = Flag;  // 更新上一时刻标志
}
```

**作用**: 每100ms执行一次，通过比较接收标志判断设备是否存活，如果长时间没有数据则认为设备断线并尝试重新初始化。

### 2.4 数据处理函数（核心功能）

```cpp
void Class_AHRS_WIT::Data_Process(uint16_t Length)
{
    Struct_AHRS_WIT_UART_Data *tmp_buffer;

    // 遍历接收缓冲区中的所有可能数据包
    for (int i = 0; i < Length;)
    {
        // 尝试将当前位置解析为数据包
        tmp_buffer = (Struct_AHRS_WIT_UART_Data *) &UART_Manage_Object->Rx_Buffer[i];

        // 帧头校验：如果不是有效帧头，则移动一个字节继续查找
        if (tmp_buffer->Frame_Header != Frame_Header)
        {
            i++;
            continue;
        }
        
        // 校验和校验：验证数据完整性
        if (Math_Sum_8((uint8_t *) tmp_buffer, 10) != tmp_buffer->Checksum)
        {
            i++;
            continue;
        }
        
        // 长度校验：确保有足够的数据组成完整数据包
        if (i + 11 > Length)
        {
            break;  // 剩余数据不足一个完整包，等待下次接收
        }

        // 根据数据类型解析并存储数据
        switch (tmp_buffer->Data_Type)
        {
        case (AHRS_WIT_Data_Type_ACCELERATE):
        {
            // 解析加速度数据：将16位整数转换为浮点数，并进行单位换算
            Data_Accelerate.Accelerate_X = (int16_t) (tmp_buffer->Data[0] | tmp_buffer->Data[1] << 8) / 32768.0f * 16.0f * 9.8f;
            Data_Accelerate.Accelerate_Y = (int16_t) (tmp_buffer->Data[2] | tmp_buffer->Data[3] << 8) / 32768.0f * 16.0f * 9.8f;
            Data_Accelerate.Accelerate_Z = (int16_t) (tmp_buffer->Data[4] | tmp_buffer->Data[5] << 8) / 32768.0f * 16.0f * 9.8f;
            Data_Accelerate.Temperature = (int16_t) (tmp_buffer->Data[6] | tmp_buffer->Data[7] << 8) / 100.0f + CELSIUS_TO_KELVIN;
            break;
        }
        case (AHRS_WIT_Data_Type_OMEGA):
        {
            // 解析角速度数据：转换为弧度/秒
            Data_Omega.Omega_X = (int16_t) (tmp_buffer->Data[0] | tmp_buffer->Data[1] << 8) / 32768.0f * 2000.0f * DEG_TO_RAD;
            Data_Omega.Omega_Y = (int16_t) (tmp_buffer->Data[2] | tmp_buffer->Data[3] << 8) / 32768.0f * 2000.0f * DEG_TO_RAD;
            Data_Omega.Omega_Z = (int16_t) (tmp_buffer->Data[4] | tmp_buffer->Data[5] << 8) / 32768.0f * 2000.0f * DEG_TO_RAD;
            Data_Omega.Voltage = (int16_t) (tmp_buffer->Data[6] | tmp_buffer->Data[7] << 8) / 100.0f;
            break;
        }
        case (AHRS_WIT_Data_Type_ANGLE):
        {
            // 解析角度数据：转换为弧度
            Data_Angle.Angle_Roll = (int16_t) (tmp_buffer->Data[0] | tmp_buffer->Data[1] << 8) / 32768.0f * PI;
            Data_Angle.Angle_Pitch = (int16_t) (tmp_buffer->Data[2] | tmp_buffer->Data[3] << 8) / 32768.0f * PI;
            Data_Angle.Angle_Yaw = (int16_t) (tmp_buffer->Data[4] | tmp_buffer->Data[5] << 8) / 32768.0f * PI;
            Data_Angle.Version = (uint16_t) tmp_buffer->Data[6] | tmp_buffer->Data[7] << 8;
            break;
        }
        case (AHRS_WIT_Data_Type_QUATERNION):
        {
            // 解析四元数数据：归一化到[-1,1]范围
            Data_Quaternion.Q_0 = (int16_t) (tmp_buffer->Data[0] | tmp_buffer->Data[1] << 8) / 32768.0f;
            Data_Quaternion.Q_1 = (int16_t) (tmp_buffer->Data[2] | tmp_buffer->Data[3] << 8) / 32768.0f;
            Data_Quaternion.Q_2 = (int16_t) (tmp_buffer->Data[4] | tmp_buffer->Data[5] << 8) / 32768.0f;
            Data_Quaternion.Q_3 = (int16_t) (tmp_buffer->Data[6] | tmp_buffer->Data[7] << 8) / 32768.0f;
            break;
        }
        }

        // 移动到下一个可能的数据包位置（跳过当前包的11个字节）
        i += 11;
    }
}
```

**作用**: 这是整个驱动的核心函数，实现了粘包处理功能。通过滑动窗口方式遍历接收缓冲区，识别并解析多个连续的数据包。

## 3. 关键特性分析

### 3.1 粘包处理机制

- **滑动窗口**: 逐字节检查缓冲区寻找有效数据包
- **错误容忍**: 即使校验失败也能继续查找后续数据包
- **完整性检查**: 确保数据包长度足够后再处理

### 3.2 数据转换公式

- **加速度**: `(raw_value / 32768.0) * 16.0 * 9.8` (转换为m/s²)
- **角速度**: `(raw_value / 32768.0) * 2000.0 * DEG_TO_RAD` (转换为rad/s)
- **角度**: `(raw_value / 32768.0) * PI` (转换为弧度)
- **四元数**: `raw_value / 32768.0` (归一化到[-1,1])

### 3.3 校验机制

- **8位校验和**: `Math_Sum_8((uint8_t *) tmp_buffer, 10)` 计算前10字节的校验和

## 4. 类的作用域和外设资源

### 4.1 作用域

- **公共作用域(public)**: 提供对外接口，包括初始化、数据获取和回调函数
- **保护作用域(protected)**: 内部实现细节，包括数据存储、处理算法等

### 4.2 使用的外设资源

- **UART接口**: 用于与WIT姿态传感器通信，支持USART1-8和UART4-8
- **定时器**: 用于100ms周期性的设备存活检测
- **内存资源**: 接收缓冲区存储原始数据，结构体存储处理后的数据

### 4.3 工作流程

1. 初始化时绑定UART端口和设置通信协议参数
2. UART接收数据触发回调函数，增加接收标志并处理数据
3. 数据处理函数遍历缓冲区，识别并解析多个数据包
4. 定时器每100ms检测设备是否存活，无数据时自动重连
5. 用户通过Get系列函数获取处理后的姿态数据

这个驱动程序特别优化了粘包处理能力，能够可靠地处理连续接收的多个数据包，适用于高频率数据采集的应用场景。