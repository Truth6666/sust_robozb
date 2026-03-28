# RM24超级电容驱动代码深度解析

## 1. 头文件分析 (dvc_supercap_24.h)

### 1.1 文件概述

这是一个用于2024年RoboMaster赛季超级电容管理的驱动头文件，版本1.1适配了新赛季的超级电容协议，相比23赛季版本增加了更多功能和状态管理。

### 1.2 包含的头文件

```cpp
#include "1_Middleware/1_Driver/CAN/drv_can.h"    // CAN驱动库
#include "1_Middleware/1_Driver/Math/drv_math.h"  // 数学运算库
```

### 1.3 枚举类型定义

#### 1.3.1 超级电容状态枚举

```cpp
enum Enum_RM24_Supercap_Status
{
    Supercap_24_Status_DISABLE = 0,  // 超级电容断开连接
    Supercap_24_Status_ENABLE,       // 超级电容正常连接
};
```

**作用**: 表示超级电容的连接状态。

#### 1.3.2 数据状态枚举

```cpp
enum Enum_Supercap_24_Data_Status : uint8_t
{
    Supercap_24_Data_Status_DISABLE = 0,  // 数据禁用
    Supercap_24_Data_Status_ENABLE,       // 数据启用
};
```

**作用**: 通用的数据启用/禁用状态。

#### 1.3.3 能量状态枚举

```cpp
enum Enum_Supercap_24_Energy_Status : uint8_t
{
    Supercap_24_Energy_Status_DISABLE = 0,  // 能量不足，禁用
    Supercap_24_Energy_Status_WARNING,      // 能量警告状态
    Supercap_24_Energy_Status_NORMAL,       // 能量正常状态
};
```

**作用**: 表示超级电容的储能状态，用于电源管理决策。

### 1.4 结构体定义

#### 1.4.1 CAN接收数据结构

```cpp
struct Struct_Supercap_24_CAN_Rx_Data
{
    // 当前能量值
    int32_t Now_Energy;                           // 32位能量值
    // 当前状态, 指示是否可用
    Enum_Supercap_24_Energy_Status Energy_Status; // 能量状态
    // 底盘功率100倍
    int16_t Chassis_Power;                       // 底盘功率（放大100倍）
} __attribute__((packed));
```

**作用**: 定义从CAN总线接收的原始数据结构，其中功率值被放大100倍传输。

#### 1.4.2 处理后数据结构

```cpp
struct Struct_Supercap_24_Rx_Data
{
    // 当前能量值
    int32_t Now_Energy;                           // 32位能量值
    // 当前状态, 指示是否可用
    Enum_Supercap_24_Energy_Status Energy_Status; // 能量状态
    // 底盘功率
    float Chassis_Power;                          // 浮点型底盘功率
} __attribute__((packed));
```

**作用**: 存储经过处理后的超级电容数据。

#### 1.4.3 CAN发送数据结构

```cpp
struct Struct_Supercap_24_CAN_Tx_Data
{
    uint16_t Power_Limit_Max;                           // 最大功率限制
    uint16_t Chassis_Buffer_Energy;                    // 底盘缓冲能量
    uint16_t Power_Compensate_Max;                     // 最大补偿功率
    uint8_t Supercap_Enable_Status_Enum : 1;          // 超级电容启用状态（1位）
    uint8_t Buffer_Energy_Loop_Status_Enum : 1;       // 能量缓冲环状态（1位）
    uint8_t Reserved_1 : 6;                           // 保留位（6位）
    uint8_t Reserved_2;                               // 保留字节
} __attribute__((packed));
```

**作用**: 定义发送给超级电容的控制数据结构，使用位字段优化存储。

### 1.5 超级电容类定义

#### 1.5.1 类结构

```cpp
class Class_Supercap_24
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
    void Init(CAN_HandleTypeDef *hcan, uint16_t __CAN_Rx_ID = 0x030, uint16_t __CAN_Tx_ID = 0x02f);
    
    // 获取函数
    inline Enum_RM24_Supercap_Status Get_Status();              // 获取状态
    inline uint16_t Get_Now_Energy();                          // 获取当前能量
    inline Enum_Supercap_24_Energy_Status Get_Energy_Status();  // 获取能量状态
    inline float Get_Chassis_Power();                          // 获取底盘功率
    
    // 设置函数
    inline void Set_Power_Limit_Max(uint16_t __Power_Limit_Max);                    // 设置功率限制
    inline void Set_Chassis_Buffer_Energy(uint16_t __Chassis_Buffer_Energy);        // 设置缓冲能量
    inline void Set_Power_Compensate_Max(uint16_t __Power_Compensate_Max);          // 设置补偿功率
    inline void Set_Supercap_Enable_Status(Enum_Supercap_24_Data_Status __Supercap_Enable_Status);  // 设置启用状态
    inline void Set_Buffer_Energy_Loop_Status(Enum_Supercap_24_Data_Status __Buffer_Energy_Loop_Status);  // 设置缓冲环状态
    
    // 回调函数
    void CAN_RxCpltCallback(uint8_t *Rx_Data);                 // CAN接收回调
    void TIM_1000ms_Alive_PeriodElapsedCallback();             // 存活检测回调
    void TIM_10ms_Send_PeriodElapsedCallback();                // 发送数据回调
```

#### 1.5.3 保护成员变量

```cpp
protected:
    // 初始化相关常量
    Struct_CAN_Manage_Object *CAN_Manage_Object;  // CAN管理对象
    uint16_t CAN_Rx_ID;                          // 接收CAN ID (0x030)
    uint16_t CAN_Tx_ID;                          // 发送CAN ID (0x02f)
    uint8_t *Tx_Data;                            // 发送数据指针

    // 常量

    // 内部变量
    uint32_t Flag = 0;                           // 接收标志
    uint32_t Pre_Flag = 0;                       // 前一时刻标志

    // 读变量
    Enum_RM24_Supercap_Status Supercap_Status = Supercap_24_Status_DISABLE;  // 超级电容状态
    Struct_Supercap_24_Rx_Data Rx_Data;                                      // 接收数据

    // 写变量
    uint16_t Power_Limit_Max = 40.0f;            // 最大功率限制
    uint16_t Chassis_Buffer_Energy = 0.0f;       // 底盘缓冲能量
    uint16_t Power_Compensate_Max = 400.0f;      // 最大补偿功率
    Enum_Supercap_24_Data_Status Supercap_Enable_Status = Supercap_24_Data_Status_ENABLE;      // 启用状态
    Enum_Supercap_24_Data_Status Buffer_Energy_Loop_Status = Supercap_24_Data_Status_DISABLE;  // 缓冲环状态

    // 内部函数
    void Data_Process();                         // 数据处理
    void Output();                               // 输出数据
```

#### 1.5.4 内联函数实现

```cpp
// 获取超级电容在线状态
inline Enum_RM24_Supercap_Status Class_Supercap_24::Get_Status()
{
    return (Supercap_Status);
}

// 获取当前能量值
inline uint16_t Class_Supercap_24::Get_Now_Energy()
{
    return (Rx_Data.Now_Energy);
}

// 获取能量状态
inline Enum_Supercap_24_Energy_Status Class_Supercap_24::Get_Energy_Status()
{
    return (Rx_Data.Energy_Status);
}

// 获取底盘功率
inline float Class_Supercap_24::Get_Chassis_Power()
{
    return (Rx_Data.Chassis_Power);
}

// 设置最大功率限制
inline void Class_Supercap_24::Set_Power_Limit_Max(uint16_t __Power_Limit_Max)
{
    Power_Limit_Max = __Power_Limit_Max;
}

// 设置底盘缓冲能量
inline void Class_Supercap_24::Set_Chassis_Buffer_Energy(uint16_t __Chassis_Buffer_Energy)
{
    Chassis_Buffer_Energy = __Chassis_Buffer_Energy;
}

// 设置最大补偿功率
inline void Class_Supercap_24::Set_Power_Compensate_Max(uint16_t __Power_Compensate_Max)
{
    Power_Compensate_Max = __Power_Compensate_Max;
}

// 设置超级电容启用状态
inline void Class_Supercap_24::Set_Supercap_Enable_Status(Enum_Supercap_24_Data_Status __Supercap_Enable_Status)
{
    Supercap_Enable_Status = __Supercap_Enable_Status;
}

// 设置能量缓冲环状态
inline void Class_Supercap_24::Set_Buffer_Energy_Loop_Status(Enum_Supercap_24_Data_Status __Buffer_Energy_Loop_Status)
{
    Buffer_Energy_Loop_Status = __Buffer_Energy_Loop_Status;
}
```

## 2. 实现文件分析 (dvc_supercap_24.cpp)

### 2.1 初始化函数

```cpp
void Class_Supercap_24::Init(CAN_HandleTypeDef *hcan, uint16_t __CAN_Rx_ID, uint16_t __CAN_Tx_ID)
{
    // 根据CAN实例绑定对应的管理对象
    if (hcan->Instance == CAN1)
    {
        CAN_Manage_Object = &CAN1_Manage_Object;
    }
    else if (hcan->Instance == CAN2)
    {
        CAN_Manage_Object = &CAN2_Manage_Object;
    }
    
    // 设置CAN ID
    CAN_Rx_ID = __CAN_Rx_ID;  // 接收ID，默认0x030
    CAN_Tx_ID = __CAN_Tx_ID;  // 发送ID，默认0x02f
    
    // 设置发送数据指针
    Tx_Data = CAN_Supercap_Tx_Data;

    // 初始化各项参数
    Power_Limit_Max = 55;                // 最大功率限制
    Chassis_Buffer_Energy = 60;          // 底盘缓冲能量
    Power_Compensate_Max = 50;           // 最大补偿功率
    Supercap_Enable_Status = Supercap_24_Data_Status_ENABLE;      // 启用超级电容
    Buffer_Energy_Loop_Status = Supercap_24_Data_Status_DISABLE;  // 禁用缓冲环
}
```

**作用**: 初始化超级电容系统，绑定CAN端口并设置通信参数和默认值。

### 2.2 CAN接收回调函数

```cpp
void Class_Supercap_24::CAN_RxCpltCallback(uint8_t *Rx_Data)
{
    // 滑动窗口, 判断超级电容是否在线
    Flag += 1;

    Data_Process();  // 处理接收到的数据
}
```

**作用**: CAN接收完成时的回调函数，增加接收标志并处理数据。

### 2.3 存活检测函数

```cpp
void Class_Supercap_24::TIM_1000ms_Alive_PeriodElapsedCallback()
{
    // 判断该时间段内是否接收过超级电容数据
    if (Flag == Pre_Flag)
    {
        // 超级电容断开连接
        Supercap_Status = Supercap_24_Status_DISABLE;
    }
    else
    {
        // 超级电容保持连接
        Supercap_Status = Supercap_24_Status_ENABLE;
    }
    Pre_Flag = Flag;
}
```

**作用**: 每1000ms检测一次超级电容是否在线。

### 2.4 发送数据函数

```cpp
void Class_Supercap_24::TIM_10ms_Send_PeriodElapsedCallback()
{
    Output();  // 构建发送数据

    // 通过CAN总线发送8字节数据
    CAN_Send_Data(CAN_Manage_Object->CAN_Handler, CAN_Tx_ID, Tx_Data, 8);
}
```

**作用**: 每10ms执行一次，向超级电容发送控制数据。

### 2.5 数据处理函数

```cpp
void Class_Supercap_24::Data_Process()
{
    Struct_Supercap_24_CAN_Rx_Data *tmp_buffer = (Struct_Supercap_24_CAN_Rx_Data *) CAN_Manage_Object->Rx_Buffer.Data;

    // 直接复制能量值和状态
    Rx_Data.Now_Energy = tmp_buffer->Now_Energy;
    Rx_Data.Energy_Status = tmp_buffer->Energy_Status;
    
    // 处理功率值：进行字节序转换并除以100转换为实际功率
    Rx_Data.Chassis_Power = (float) ((int16_t) Math_Endian_Reverse_16(&tmp_buffer->Chassis_Power, nullptr)) / 100.0f;
}
```

**作用**: 解析和处理从CAN总线接收到的超级电容数据，进行字节序转换和单位转换。

### 2.6 输出函数

```cpp
void Class_Supercap_24::Output()
{
    Struct_Supercap_24_CAN_Tx_Data *tmp_buffer = (Struct_Supercap_24_CAN_Tx_Data *) Tx_Data;

    // 设置各项控制参数
    tmp_buffer->Power_Limit_Max = Power_Limit_Max;
    tmp_buffer->Chassis_Buffer_Energy = Chassis_Buffer_Energy;
    tmp_buffer->Power_Compensate_Max = Power_Compensate_Max;
    tmp_buffer->Supercap_Enable_Status_Enum = Supercap_Enable_Status;
    tmp_buffer->Buffer_Energy_Loop_Status_Enum = Buffer_Energy_Loop_Status;
}
```

**作用**: 将控制参数打包到发送缓冲区，准备通过CAN总线发送。

## 3. 关键特性分析

### 3.1 升级的功能

- **能量状态管理**: 增加了警告状态，更好的电源管理
- **多参数控制**: 支持功率限制、缓冲能量、补偿功率等多参数设置
- **功能开关**: 支持超级电容和能量缓冲环的独立控制

### 3.2 数据精度改进

- **32位能量值**: 更高的能量精度
- **功率放大传输**: 通过放大100倍提高传输精度

### 3.3 电源管理策略

- **缓冲环控制**: 可以控制能量缓冲环的启停
- **补偿功率**: 支持功率补偿功能

### 3.4 通信协议优化

- **CAN ID调整**: 接收0x030，发送0x02f
- **数据长度**: 8字节发送数据，包含更多信息

## 4. 类的作用域和外设资源

### 4.1 作用域

- **公共作用域(public)**: 提供初始化、数据获取、设置和通信接口
- **保护作用域(protected)**: 内部实现细节，包括数据处理、转换算法等

### 4.2 使用的外设资源

- **CAN接口**: 用于与超级电容通信，支持CAN1-2
- **定时器**: 用于1000ms存活检测和10ms数据发送
- **内存资源**: 发送和接收缓冲区存储通信数据
- **数学库**: 字节序转换等数学运算

### 4.3 工作流程

1. 初始化时绑定CAN端口和设置通信ID
2. 通过CAN接收回调函数处理超级电容数据
3. 1000ms定时器检测超级电容在线状态
4. 10ms定时器向超级电容发送控制参数
5. 用户通过Get/Set系列函数获取和设置电源管理参数

## 5. CAN通信协议

### 5.1 接收协议 (CAN_Rx_ID: 0x030)

- **数据格式**: `[能量值(32位)][能量状态(8位)][功率值(16位大端序)]`
- **能量值**: int32_t，表示当前能量
- **能量状态**: 枚举值，表示能量状态
- **功率值**: int16_t，除以100转换为浮点数

### 5.2 发送协议 (CAN_Tx_ID: 0x02f)

- **数据格式**: `[功率限制(16位)][缓冲能量(16位)][补偿功率(16位)][控制标志(8位)][保留(8位)]`
- **功率限制**: uint16_t，最大功率限制
- **缓冲能量**: uint16_t，底盘缓冲能量
- **补偿功率**: uint16_t，最大补偿功率
- **控制标志**: 位字段，包含启用状态和缓冲环状态

这个RM24超级电容驱动程序实现了一个功能更完善的电源管理系统，通过CAN总线与超级电容通信，实现精细化的能源管理和功率控制功能。