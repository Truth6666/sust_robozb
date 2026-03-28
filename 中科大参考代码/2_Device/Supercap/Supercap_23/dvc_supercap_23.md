# 超级电容驱动代码深度解析

## 1. 头文件分析 (dvc_supercap_23.h)

### 1.1 文件概述

这是一个用于超级电容管理的驱动头文件，版本0.1为23赛季定稿版本，用于机器人电源管理系统，但由于哨兵机器人无超级电容而未启用。

### 1.2 包含的头文件

```cpp
#include "1_Middleware/1_Driver/CAN/drv_can.h"    // CAN驱动库
#include "1_Middleware/1_Driver/Math/drv_math.h"  // 数学运算库
```

### 1.3 枚举类型定义

#### 1.3.1 超级电容状态枚举

```cpp
enum Enum_Supercap_23_Status
{
    Supercap_23_Status_DISABLE = 0,  // 超级电容断开连接
    Supercap_23_Status_ENABLE,       // 超级电容正常连接
};
```

**作用**: 表示超级电容的连接状态。

### 1.4 结构体定义

#### 1.4.1 CAN接收数据结构

```cpp
struct Struct_Supercap_23_CAN_Rx_Data
{
    // 当前能量值（大端序）
    int16_t Now_Energy_Reverse;
    // 当前功率（大端序）
    int16_t Chassis_Power_Reverse;
} __attribute__((packed));
```

**作用**: 定义从CAN总线接收的原始数据结构，使用大端序存储。

#### 1.4.2 处理后数据结构

```cpp
struct Struct_Supercap_23_Rx_Data
{
    // 当前能量值
    int16_t Now_Energy;
    // 当前功率
    float Chassis_Power;
} __attribute__((packed));
```

**作用**: 存储经过字节序转换后的超级电容数据。

#### 1.4.3 CAN发送数据结构

```cpp
struct Struct_Supercap_23_CAN_Tx_Data
{
    uint16_t Power_Limit_Max;  // 最大功率限制
} __attribute__((packed));
```

**作用**: 定义发送给超级电容的控制数据结构。

### 1.5 超级电容类定义

#### 1.5.1 类结构

```cpp
class Class_Supercap_23
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
    void Init(CAN_HandleTypeDef *hcan, uint16_t __CAN_Rx_ID = 0x210, uint16_t __CAN_Tx_ID = 0x220);
    
    // 获取函数
    inline float Get_Power_Threshold();     // 获取功率阈值
    inline Enum_Supercap_23_Status Get_Status();  // 获取状态
    inline uint16_t Get_Now_Energy();      // 获取当前能量
    inline float Get_Chassis_Power();      // 获取当前功率
    
    // 设置函数
    inline void Set_Power_Limit_Max(uint16_t __Power_Limit_Max);  // 设置功率限制
    
    // 回调函数
    void CAN_RxCpltCallback(uint8_t *Rx_Data);                  // CAN接收回调
    void TIM_1000ms_Alive_PeriodElapsedCallback();              // 存活检测回调
    void TIM_10ms_Send_PeriodElapsedCallback();                 // 发送数据回调
```

#### 1.5.3 保护成员变量

```cpp
protected:
    // 初始化相关常量
    Struct_CAN_Manage_Object *CAN_Manage_Object;  // CAN管理对象
    uint16_t CAN_Rx_ID;                          // 接收CAN ID
    uint16_t CAN_Tx_ID;                          // 发送CAN ID
    uint8_t *Tx_Data;                            // 发送数据指针

    // 常量
    float Power_Threshold = 300.0f;              // 过放保护能量阈值

    // 内部变量
    uint32_t Flag = 0;                           // 接收标志
    uint32_t Pre_Flag = 0;                       // 前一时刻标志

    // 读变量
    Enum_Supercap_23_Status Supercap_Status = Supercap_23_Status_DISABLE;  // 超级电容状态
    Struct_Supercap_23_Rx_Data Rx_Data;                                    // 接收数据

    // 写变量
    uint16_t Power_Limit_Max;                    // 最大功率限制

    // 内部函数
    void Data_Process();                         // 数据处理
    void Output();                               // 输出数据
```

#### 1.5.4 内联函数实现

```cpp
// 获取过放保护能量阈值
inline float Class_Supercap_23::Get_Power_Threshold()
{
    return (Power_Threshold);
}

// 获取超级电容在线状态
inline Enum_Supercap_23_Status Class_Supercap_23::Get_Status()
{
    return (Supercap_Status);
}

// 获取当前能量值
inline uint16_t Class_Supercap_23::Get_Now_Energy()
{
    return (Rx_Data.Now_Energy);
}

// 获取当前功率
inline float Class_Supercap_23::Get_Chassis_Power()
{
    return (Rx_Data.Chassis_Power);
}

// 设定最大功率限制
inline void Class_Supercap_23::Set_Power_Limit_Max(uint16_t __Power_Limit_Max)
{
    Power_Limit_Max = __Power_Limit_Max;
}
```

## 2. 实现文件分析 (dvc_supercap_23.cpp)

### 2.1 初始化函数

```cpp
void Class_Supercap_23::Init(CAN_HandleTypeDef *hcan, uint16_t __CAN_Rx_ID, uint16_t __CAN_Tx_ID)
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
    CAN_Rx_ID = __CAN_Rx_ID;
    CAN_Tx_ID = __CAN_Tx_ID;
    
    // 设置发送数据指针
    Tx_Data = CAN_Supercap_Tx_Data;

    // 初始化功率限制为45W
    Power_Limit_Max = 45;
}
```

**作用**: 初始化超级电容系统，绑定CAN端口并设置通信参数。

### 2.2 CAN接收回调函数

```cpp
void Class_Supercap_23::CAN_RxCpltCallback(uint8_t *Rx_Data)
{
    // 滑动窗口, 判断超级电容是否在线
    Flag += 1;

    Data_Process();  // 处理接收到的数据
}
```

**作用**: CAN接收完成时的回调函数，增加接收标志并处理数据。

### 2.3 存活检测函数

```cpp
void Class_Supercap_23::TIM_1000ms_Alive_PeriodElapsedCallback()
{
    // 判断该时间段内是否接收过超级电容数据
    if (Flag == Pre_Flag)
    {
        // 超级电容断开连接
        Supercap_Status = Supercap_23_Status_DISABLE;
    }
    else
    {
        // 超级电容保持连接
        Supercap_Status = Supercap_23_Status_ENABLE;
    }

    Pre_Flag = Flag;
}
```

**作用**: 每1000ms检测一次超级电容是否在线。

### 2.4 发送数据函数

```cpp
void Class_Supercap_23::TIM_10ms_Send_PeriodElapsedCallback()
{
    Output();  // 构建发送数据

    // 通过CAN总线发送数据
    CAN_Send_Data(CAN_Manage_Object->CAN_Handler, CAN_Tx_ID, Tx_Data, 2);
}
```

**作用**: 每10ms执行一次，向超级电容发送功率限制数据。

### 2.5 数据处理函数

```cpp
void Class_Supercap_23::Data_Process()
{
    int16_t tmp_energy, tmp_chassis_power;
    
    // 获取CAN接收缓冲区中的数据
    Struct_Supercap_23_CAN_Rx_Data *tmp_buffer = (Struct_Supercap_23_CAN_Rx_Data *) CAN_Manage_Object->Rx_Buffer.Data;

    // 字节序转换：将大端序转换为小端序
    Math_Endian_Reverse_16((void *) &tmp_buffer->Now_Energy_Reverse, &tmp_energy);
    Math_Endian_Reverse_16((void *) &tmp_buffer->Chassis_Power_Reverse, &tmp_chassis_power);

    // 更新处理后的数据
    Rx_Data.Now_Energy = tmp_energy;
    Rx_Data.Chassis_Power = tmp_chassis_power / 100.0f;  // 转换为浮点数，单位可能是W
}
```

**作用**: 解析和转换从CAN总线接收到的超级电容数据，进行字节序转换和单位转换。

### 2.6 输出函数

```cpp
void Class_Supercap_23::Output()
{
    Struct_Supercap_23_CAN_Tx_Data *tmp_buffer = (Struct_Supercap_23_CAN_Tx_Data *) Tx_Data;

    // 字节序转换：将小端序转换为大端序发送
    Math_Endian_Reverse_16(&Power_Limit_Max, &tmp_buffer->Power_Limit_Max);
}
```

**作用**: 将功率限制数据转换为大端序格式，准备发送到CAN总线。

## 3. 关键特性分析

### 3.1 双向通信协议

- **接收**: 从超级电容获取当前能量和功率信息
- **发送**: 向超级电容发送功率限制指令

### 3.2 字节序处理

- **接收时**: 将大端序转换为小端序
- **发送时**: 将小端序转换为大端序
- **目的**: 确保不同平台间的数据兼容性

### 3.3 电源管理功能

- **过放保护**: 300.0f的能量阈值
- **功率限制**: 根据比赛规则限制功率输出

### 3.4 定时器控制

- **1000ms**: 检测超级电容在线状态
- **10ms**: 发送功率限制数据

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
4. 10ms定时器向超级电容发送功率限制
5. 用户通过Get/Set系列函数获取和设置电源管理参数

## 5. CAN通信协议

### 5.1 接收协议 (CAN_Rx_ID)

- **数据格式**: `[能量值(16位大端序)][功率值(16位大端序)]`
- **能量值**: int16_t，表示当前能量
- **功率值**: int16_t，除以100转换为浮点数

### 5.2 发送协议 (CAN_Tx_ID)

- **数据格式**: `[功率限制(16位大端序)]`
- **功率限制**: uint16_t，比赛规则允许的最大功率

这个超级电容驱动程序实现了一个完整的电源管理系统，通过CAN总线与超级电容通信，实现能量监控和功率控制功能。