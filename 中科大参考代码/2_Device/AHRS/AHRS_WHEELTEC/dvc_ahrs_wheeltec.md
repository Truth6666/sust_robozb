# Wheeltec姿态传感器代码深度解析

## 1. 头文件分析 (dvc_ahrs_wheeltec.h)

### 1.1 文件概述

这是一个用于Wheeltec姿态传感器的驱动头文件，实现了通过UART接口获取IMU（惯性测量单元）和AHRS（姿态航向参考系统）数据的功能。

### 1.2 包含的头文件

```cpp
#include "1_Middleware/1_Driver/Math/drv_math.h"  // 数学运算库
#include "1_Middleware/1_Driver/UART/drv_uart.h"  // UART驱动库
```

### 1.3 枚举类型定义

#### 1.3.1 设备状态枚举

```cpp
enum Enum_AHRS_WHEELTEC_Status
{
    AHRS_WHEELTEC_Status_DISABLE = 0,  // 设备禁用状态
    AHRS_WHEELTEC_Status_ENABLE,       // 设备启用状态
};
```

**作用**: 定义姿态传感器的工作状态，用于判断设备是否正常连接。

#### 1.3.2 数据类型枚举

```cpp
enum Enum_AHRS_WHEELTEC_Data_Type : uint8_t
{
    AHRS_WHEELTEC_Data_Type_IMU = 0x40,  // IMU数据类型（原始传感器数据）
    AHRS_WHEELTEC_Data_Type_AHRS,        // AHRS数据类型（融合后的姿态数据）
};
```

**作用**: 区分接收到的数据类型，IMU包含原始的陀螺仪、加速度计、磁力计数据，AHRS包含计算后的姿态角和四元数。

### 1.4 结构体定义

#### 1.4.1 UART原始数据结构

```cpp
struct Struct_AHRS_WHEELTEC_UART_Data
{
    uint8_t Frame_Header;                    // 帧头标志
    Enum_AHRS_WHEELTEC_Data_Type Data_Type;  // 数据类型
    uint8_t Data_Length;                     // 数据长度
    uint8_t Sequence;                        // 序列号
    uint8_t CRC_8;                           // 8位CRC校验
    uint16_t CRC_16;                         // 16位CRC校验
    uint8_t Data[128];                       // 实际数据
}__attribute__((packed));                    // 紧凑打包，避免内存对齐
```

**作用**: 定义从UART接收的原始数据包格式，包含帧头、类型、长度、校验等信息。

#### 1.4.2 IMU处理后数据结构

```cpp
struct Struct_AHRS_WHEELTEC_Data_IMU
{
    float Omega_X;              // X轴角速度
    float Omega_Y;              // Y轴角速度  
    float Omega_Z;              // Z轴角速度
    float Accelerate_X;         // X轴加速度
    float Accelerate_Y;         // Y轴加速度
    float Accelerate_Z;         // Z轴加速度
    float Magnetic_X;           // X轴磁场强度
    float Magnetic_Y;           // Y轴磁场强度
    float Magnetic_Z;           // Z轴磁场强度
    float Temperature;          // 温度
    float Pressure;             // 大气压力
    float Pressure_Temperature; // 气压计温度
    int64_t Timestamp;          // 时间戳
    uint8_t Frame_Rear;         // 帧尾标志
} __attribute__((packed));
```

**作用**: 存储IMU模式下的传感器原始数据，包括角速度、加速度、磁场强度等。

#### 1.4.3 AHRS处理后数据结构

```cpp
struct Struct_AHRS_WHEELTEC_Data_AHRS
{
    float Omega_Roll;   // 滚转角速度
    float Omega_Pitch;  // 俯仰角速度
    float Omega_Yaw;    // 航向角速度
    float Angle_Roll;   // 滚转角度
    float Angle_Pitch;  // 俯仰角度
    float Angle_Yaw;    // 航向角度
    float Q_0;          // 四元数0（实部）
    float Q_1;          // 四元数1
    float Q_2;          // 四元数2
    float Q_3;          // 四元数3
    uint64_t Timestamp; // 时间戳
    uint8_t Frame_Rear; // 帧尾标志
} __attribute__((packed));
```

**作用**: 存储AHRS模式下的姿态解算结果，包括欧拉角、角速度和四元数。

### 1.5 主类定义

#### 1.5.1 类成员变量

```cpp
class Class_AHRS_WHEELTEC
{
protected:
    // 初始化相关常量
    Struct_UART_Manage_Object *UART_Manage_Object;  // 绑定的UART管理对象
    uint8_t Frame_Header;                           // 数据包头标
    uint8_t Frame_Rear;                             // 数据包尾标

    // 内部变量
    uint32_t Flag = 0;                              // 当前时刻接收标志
    uint32_t Pre_Flag = 0;                          // 前一时刻接收标志

    // 读变量
    Enum_AHRS_WHEELTEC_Status WHEELTEC_AHRS_Status = AHRS_WHEELTEC_Status_DISABLE;  // 设备状态
    Struct_AHRS_WHEELTEC_Data_IMU Data_IMU;         // IMU数据
    Struct_AHRS_WHEELTEC_Data_AHRS Data_AHRS;       // AHRS数据

    // 内部函数
    uint8_t Verify_CRC_8(uint8_t *Message, uint32_t Length);     // 8位CRC校验
    uint16_t Verify_CRC_16(uint8_t *Message, uint32_t Length);   // 16位CRC校验
    void Data_Process();                                         // 数据处理函数
};
```

#### 1.5.2 公共接口函数声明

```cpp
public:
    // 初始化函数
    void Init(UART_HandleTypeDef *huart, uint8_t __Frame_Header = 0xfc, uint8_t __Frame_Rear = 0xfd);
    
    // 状态获取函数
    inline Enum_AHRS_WHEELTEC_Status Get_Status();
    
    // IMU数据获取函数
    inline float Get_Omega_X();         // 获取X轴角速度
    inline float Get_Omega_Y();         // 获取Y轴角速度
    inline float Get_Omega_Z();         // 获取Z轴角速度
    // ... 其他IMU数据获取函数
    
    // AHRS数据获取函数
    inline float Get_Omega_Roll();      // 获取滚转角速度
    inline float Get_Angle_Roll();      // 获取滚转角度
    // ... 其他AHRS数据获取函数
    
    // 回调函数
    void UART_RxCpltCallback(uint8_t *Rx_Data, uint16_t Length);  // UART接收完成回调
    void TIM_100ms_Alive_PeriodElapsedCallback();                 // 定时器存活检测回调
```

## 2. 实现文件分析 (dvc_ahrs_wheeltec.cpp)

### 2.1 CRC校验表定义

```cpp
// CRC8校验码表（256个元素）
static const uint8_t crc_8_table[256] = {...};

// CRC16校验码表（256个元素）  
static const uint16_t crc_16_table[256] = {...};
```

**作用**: 预定义的CRC查表法校验码，用于快速计算数据校验值，提高校验效率。

### 2.2 初始化函数

```cpp
void Class_AHRS_WHEELTEC::Init(UART_HandleTypeDef *huart, uint8_t __Frame_Header, uint8_t __Frame_Rear)
{
    // 根据UART实例绑定对应的管理对象
    if (huart->Instance == USART1) {
        UART_Manage_Object = &UART1_Manage_Object;
    }
    else if (huart->Instance == USART2) {
        UART_Manage_Object = &UART2_Manage_Object;
    }
    // ... 其他UART端口判断
    
    // 设置帧头帧尾标志
    Frame_Header = __Frame_Header;
    Frame_Rear = __Frame_Rear;
}
```

**作用**: 初始化姿态传感器，绑定指定的UART端口并设置通信协议的帧头帧尾标志。

### 2.3 UART接收回调函数

```cpp
void Class_AHRS_WHEELTEC::UART_RxCpltCallback(uint8_t *Rx_Data, uint16_t Length)
{
    // 滑动窗口，判断陀螺仪是否在线
    Flag += 1;
    Data_Process();
}
```

**作用**: UART接收完成时的回调函数，增加接收标志并处理接收到的数据。

### 2.4 定时器存活检测函数

```cpp
void Class_AHRS_WHEELTEC::TIM_100ms_Alive_PeriodElapsedCallback()
{
    // 判断该时间段内是否接收过陀螺仪数据
    if (Flag == Pre_Flag) {
        // 陀螺仪断开连接
        WHEELTEC_AHRS_Status = AHRS_WHEELTEC_Status_DISABLE;
        UART_Reinit(UART_Manage_Object->UART_Handler);  // 重新初始化UART
    } else {
        // 陀螺仪保持连接
        WHEELTEC_AHRS_Status = AHRS_WHEELTEC_Status_ENABLE;
    }
    Pre_Flag = Flag;  // 更新上一时刻标志
}
```

**作用**: 每100ms执行一次，通过比较接收标志判断设备是否存活，如果长时间没有数据则认为设备断线并尝试重新初始化。

### 2.5 CRC校验函数

#### 2.5.1 8位CRC校验

```cpp
uint8_t Class_AHRS_WHEELTEC::Verify_CRC_8(uint8_t *Message, uint32_t Length)
{
    uint8_t check = 0;
    
    if (Message == nullptr) {
        return (check);
    }
    
    for (int i = 0; i < Length; i++) {
        uint8_t value = Message[i];
        uint8_t new_index = check ^ value;
        check = crc_8_table[new_index];
    }
    return (check);
}
```

**作用**: 使用查表法计算8位CRC校验值，验证数据传输的完整性。

#### 2.5.2 16位CRC校验

```cpp
uint16_t Class_AHRS_WHEELTEC::Verify_CRC_16(uint8_t *Message, uint32_t Length)
{
    uint16_t check = 0;
    
    if (Message == nullptr) {
        return (check);
    }
    
    for (int i = 0; i < Length; i++) {
        uint8_t value = Message[i];
        check = crc_16_table[((check >> 8) ^ value) & 0xff] ^ (check << 8);
    }
    return (check);
}
```

**作用**: 使用查表法计算16位CRC校验值，用于验证大数据块的传输完整性。

### 2.6 数据处理函数

```cpp
void Class_AHRS_WHEELTEC::Data_Process()
{
    // 将接收缓冲区转换为数据结构
    Struct_AHRS_WHEELTEC_UART_Data *tmp_buffer = (Struct_AHRS_WHEELTEC_UART_Data *) UART_Manage_Object->Rx_Buffer;

    // 帧头校验
    if (tmp_buffer->Frame_Header != Frame_Header) {
        return;
    }
    
    // 8位CRC校验
    if (Verify_CRC_8((uint8_t *) tmp_buffer, 4) != tmp_buffer->CRC_8) {
        return;
    }
    
    // 16位CRC校验（注意字节序转换）
    if (Verify_CRC_16((uint8_t *) tmp_buffer + 7, tmp_buffer->Data_Length) != Math_Endian_Reverse_16(&tmp_buffer->CRC_16, nullptr)) {
        return;
    }

    // 根据数据类型进行处理
    switch (tmp_buffer->Data_Type) {
    case (AHRS_WHEELTEC_Data_Type_IMU): {
        // 复制IMU数据
        memcpy(&Data_IMU, tmp_buffer->Data, sizeof(Struct_AHRS_WHEELTEC_Data_IMU));
        break;
    }
    case (AHRS_WHEELTEC_Data_Type_AHRS): {
        // 复制AHRS数据
        memcpy(&Data_AHRS, tmp_buffer->Data, sizeof(Struct_AHRS_WHEELTEC_Data_AHRS));
        break;
    }
    }
}
```

**作用**: 解析和验证接收到的UART数据包，根据数据类型分别存储到对应的结构体中。

### 2.7 内联函数实现

所有`Get_`开头的函数都是简单的返回对应数据字段的值，例如：

```cpp
inline float Class_AHRS_WHEELTEC::Get_Omega_X()
{
    return (Data_IMU.Omega_X);
}
```

## 3. 类的作用域和外设资源

### 3.1 作用域

- **公共作用域(public)**: 提供对外接口，包括初始化、数据获取和回调函数
- **保护作用域(protected)**: 内部实现细节，包括数据存储、校验算法等

### 3.2 使用的外设资源

- **UART接口**: 用于与姿态传感器通信，支持USART1-8和UART4-8
- **定时器**: 用于100ms周期性的设备存活检测
- **内存资源**: 接收缓冲区存储原始数据，结构体存储处理后的数据

### 3.3 工作流程

1. 初始化时绑定UART端口和设置通信协议参数
2. UART接收数据触发回调函数，增加接收标志并处理数据
3. 数据处理函数验证校验和，解析数据类型，存储到对应结构体
4. 定时器每100ms检测设备是否存活，无数据时自动重连
5. 用户通过Get系列函数获取处理后的姿态数据

这个驱动程序实现了可靠的数据通信、校验和设备管理功能，适用于机器人、无人机等需要姿态感知的应用场景。