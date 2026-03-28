# 自制功率计驱动代码深度解析

## 1. 头文件分析 (dvc_powermeter.h)

### 1.1 文件概述

这是一个用于自制功率计的驱动头文件，版本0.1为新建功能，通过UART与功率计通信，获取电流、电压、功率等实时数据。

### 1.2 包含的头文件

```cpp
#include "1_Middleware/1_Driver/UART/drv_uart.h"  // UART驱动库
#include "1_Middleware/1_Driver/Math/drv_math.h"  // 数学运算库
```

### 1.3 枚举类型定义

#### 1.3.1 功率计状态枚举

```cpp
enum Enum_Powermeter_Status
{
    Powermeter_Status_DISABLE = 0,  // 功率计断开连接
    Powermeter_Status_ENABLE,       // 功率计正常连接
};
```

**作用**: 表示功率计的连接状态。

### 1.4 结构体定义

#### 1.4.1 UART原始数据结构

```cpp
struct Struct_Powermeter_UART_Data
{
    uint8_t Frame_Header;    // 帧头标识符
    float Current;           // 电流值
    float Voltage;           // 电压值
    float Power;             // 功率值
    uint8_t Checksum;        // 校验和
} __attribute__((packed));
```

**作用**: 定义从UART接收的原始功率计数据包格式，包含帧头、校验和等通信协议元素。

#### 1.4.2 处理后数据结构

```cpp
struct Struct_Powermeter_Data
{
    float Current;           // 电流值
    float Voltage;           // 电压值
    float Power;             // 功率值
};
```

**作用**: 存储经过验证和处理的功率计数据。

### 1.5 功率计类定义

#### 1.5.1 类结构

```cpp
class Class_Powermeter
{
public:
    // 公共接口函数...
    
protected:
    // 成员变量...
};
```

#### 1.5.2 公共接口函数声明

```cpp
public:
    // 初始化函数
    void Init(UART_HandleTypeDef *huart, uint8_t __Frame_Header = 0xab);
    
    // 状态获取函数
    inline Enum_Powermeter_Status Get_Status();
    
    // 数据获取函数
    inline float Get_Current();    // 获取电流值
    inline float Get_Voltage();    // 获取电压值
    inline float Get_Power();      // 获取功率值
    
    // 回调函数
    void UART_RxCpltCallback(uint8_t *Rx_Data, uint16_t Length);        // UART接收完成回调
    void TIM_100ms_Alive_PeriodElapsedCallback();                       // 定时器存活检测回调

protected:
    // 成员变量...
};
```

#### 1.5.3 保护成员变量

```cpp
protected:
    // 初始化相关常量
    Struct_UART_Manage_Object *UART_Manage_Object;  // 绑定的UART管理对象
    uint8_t Frame_Header;                           // 数据包头标

    // 内部变量
    uint32_t Flag = 0;                              // 当前时刻接收标志
    uint32_t Pre_Flag = 0;                          // 前一时刻接收标志

    // 读变量
    Enum_Powermeter_Status Powermeter_Status = Powermeter_Status_DISABLE;  // 功率计状态
    Struct_Powermeter_Data Data;                                           // 功率计数据

    // 内部函数
    void Data_Process(uint16_t Length);  // 数据处理函数
```

#### 1.5.4 内联函数实现

```cpp
// 获取功率计状态
inline Enum_Powermeter_Status Class_Powermeter::Get_Status()
{
    return (Powermeter_Status);
}

// 获取电流值
inline float Class_Powermeter::Get_Current()
{
    return (Data.Current);
}

// 获取电压值
inline float Class_Powermeter::Get_Voltage()
{
    return (Data.Voltage);
}

// 获取功率值
inline float Class_Powermeter::Get_Power()
{
    return (Data.Power);
}
```

## 2. 实现文件分析 (dvc_powermeter.cpp)

### 2.1 初始化函数

```cpp
void Class_Powermeter::Init(UART_HandleTypeDef *huart, uint8_t __Frame_Header)
{
    // 根据UART实例绑定对应的管理对象
    if (huart->Instance == USART1)
    {
        UART_Manage_Object = &UART1_Manage_Object;
    }
    else if (huart->Instance == USART2)
    {
        UART_Manage_Object = &UART2_Manage_Object;
    }
    else if (huart->Instance == USART3)
    {
        UART_Manage_Object = &UART3_Manage_Object;
    }
    else if (huart->Instance == UART4)
    {
        UART_Manage_Object = &UART4_Manage_Object;
    }
    else if (huart->Instance == UART5)
    {
        UART_Manage_Object = &UART5_Manage_Object;
    }
    else if (huart->Instance == USART6)
    {
        UART_Manage_Object = &UART6_Manage_Object;
    }
    else if (huart->Instance == UART7)
    {
        UART_Manage_Object = &UART7_Manage_Object;
    }
    else if (huart->Instance == UART8)
    {
        UART_Manage_Object = &UART8_Manage_Object;
    }

    // 设置帧头标识符
    Frame_Header = __Frame_Header;
}
```

**作用**: 初始化功率计系统，绑定UART端口并设置通信协议参数。

### 2.2 UART接收回调函数

```cpp
void Class_Powermeter::UART_RxCpltCallback(uint8_t *Rx_Data, uint16_t Length)
{
    // 滑动窗口，判断功率计是否在线
    Flag += 1;

    Data_Process(Length);  // 处理接收到的数据
}
```

**作用**: UART接收完成时的回调函数，增加接收标志并处理数据。

### 2.3 定时器存活检测函数

```cpp
void Class_Powermeter::TIM_100ms_Alive_PeriodElapsedCallback()
{
    // 判断该时间段内是否接收过功率计数据
    if (Flag == Pre_Flag)
    {
        // 功率计断开连接
        Powermeter_Status = Powermeter_Status_DISABLE;
    }
    else
    {
        // 功率计保持连接
        Powermeter_Status = Powermeter_Status_ENABLE;
    }
    Pre_Flag = Flag;  // 更新前一时刻标志
}
```

**作用**: 每100ms执行一次，检测功率计是否在线。

### 2.4 数据处理函数

```cpp
void Class_Powermeter::Data_Process(uint16_t Length)
{
    // 数据处理过程
    Struct_Powermeter_UART_Data *tmp_buffer;

    // 将接收缓冲区转换为功率计数据结构
    tmp_buffer = (Struct_Powermeter_UART_Data *)UART_Manage_Object->Rx_Buffer;

    // 1. 帧头校验
    if (tmp_buffer->Frame_Header != Frame_Header)
    {
        return;  // 帧头不匹配，丢弃数据
    }

    // 2. 校验和校验
    if (Math_Sum_8((uint8_t *)tmp_buffer + 1, sizeof(Struct_Powermeter_UART_Data) - 2) != tmp_buffer->Checksum)
    {
        return;  // 校验和错误，丢弃数据
    }

    // 3. 数据长度校验
    if (Length != sizeof(Struct_Powermeter_UART_Data))
    {
        return;  // 数据长度不符，丢弃数据
    }

    // 所有校验通过，更新数据
    Data.Current = tmp_buffer->Current;
    Data.Voltage = tmp_buffer->Voltage;
    Data.Power = tmp_buffer->Power;
}
```

**作用**: 解析和验证从UART接收的数据包，执行三次校验确保数据完整性。

## 3. 关键特性分析

### 3.1 三层校验机制

- **帧头校验**: 验证数据包来源是否正确
- **校验和校验**: 验证数据传输过程中是否出错
- **长度校验**: 验证数据包长度是否符合预期

### 3.2 通信协议设计

- **帧头**: 用于标识数据包类型
- **数据**: 包含电流、电压、功率的浮点数值
- **校验和**: 用于数据完整性验证

### 3.3 在线状态检测

- **滑动窗口**: 通过接收标志计数判断设备连接状态
- **定时检测**: 每100ms检查一次连接状态

## 4. 类的作用域和外设资源

### 4.1 作用域

- **公共作用域(public)**: 提供初始化、数据获取和回调函数接口
- **保护作用域(protected)**: 内部实现细节，包括数据存储、处理算法等

### 4.2 使用的外设资源

- **UART接口**: 用于与功率计通信，支持USART1-8和UART4-8
- **定时器**: 用于100ms设备存活检测
- **内存资源**: 接收缓冲区存储通信数据
- **数学库**: 校验和计算等数学运算

### 4.3 工作流程

1. 初始化时绑定UART端口和设置帧头标识符
2. UART接收数据触发回调函数，处理功率计数据
3. 100ms定时器检测功率计在线状态
4. 用户通过Get系列函数获取实时功率数据

这个驱动程序实现了完整的功率计通信协议，支持数据验证和状态监控功能，是机器人电源管理系统的重要组成部分。