# 裁判系统驱动代码深度解析

## 1. 头文件分析 (dvc_referee.h)

### 1.1 文件概述

这是一个用于RM机器人比赛裁判系统的驱动头文件，版本1.1适配了1.6.1通信协议，通过UART与裁判系统通信，获取比赛状态、机器人状态、比赛结果等信息。

### 1.2 包含的头文件

```cpp
#include "1_Middleware/1_Driver/UART/drv_uart.h"  // UART驱动库
```

### 1.3 枚举类型定义

#### 1.3.1 裁判系统状态枚举

```cpp
enum Enum_Referee_Status
{
    Referee_Status_DISABLE = 0,  // 裁判系统断开连接
    Referee_Status_ENABLE,       // 裁判系统正常连接
};
```

**作用**: 表示裁判系统的连接状态。

#### 1.3.2 数据状态枚举

```cpp
enum Enum_Referee_Data_Status : uint8_t
{
    Referee_Data_Status_DISABLE = 0,  // 数据禁用
    Referee_Data_Status_ENABLE,       // 数据启用
};
```

**作用**: 通用的数据启用/禁用状态。

#### 1.3.3 命令ID枚举

```cpp
enum Enum_Referee_COMMAND_ID : uint16_t
{
    Referee_Command_ID_GAME_STATUS = 0x0001,      // 比赛状态
    Referee_Command_ID_GAME_RESULT,               // 比赛结果
    Referee_Command_ID_GAME_ROBOT_HP,             // 机器人血量
    Referee_Command_ID_EVENT_SELF_DATA = 0x0101,  // 场地事件
    Referee_Command_ID_ROBOT_STATUS = 0x0201,     // 机器人状态
    Referee_Command_ID_INTERACTION = 0x0301,      // 交互命令
};
```

**作用**: 定义裁判系统通信协议中的各种命令ID，按功能分类。

#### 1.3.4 机器人ID枚举

```cpp
enum Enum_Referee_Data_Robot_ID : uint8_t
{
    Referee_Data_Robot_ID_NULL = 0,
    Referee_Data_Robot_ID_HERO_1,      // 英雄机器人
    Referee_Data_Robot_ID_ENGINEER_2,  // 工程机器人
    Referee_Data_Robot_ID_INFANTRY_3,  // 步兵机器人
    Referee_Data_Robot_ID_AERIAL_6,    // 无人机
    Referee_Data_Robot_ID_SENTRY_7,    // 哨兵机器人
    // ... 更多机器人类型
};
```

**作用**: 定义单方机器人的ID。

#### 1.3.5 双方机器人ID枚举

```cpp
enum Enum_Referee_Data_Robots_ID : uint8_t
{
    Referee_Data_Robots_ID_NO = 0,
    Referee_Data_Robots_ID_RED_HERO_1,      // 红方英雄
    Referee_Data_Robots_ID_RED_ENGINEER_2,  // 红方工程
    // ...
    Referee_Data_Robots_ID_BLUE_HERO_1 = 101,  // 蓝方英雄
    Referee_Data_Robots_ID_BLUE_ENGINEER_2,    // 蓝方工程
};
```

**作用**: 定义双方所有机器人的ID，红方0-11，蓝方101-112。

### 1.4 结构体定义

#### 1.4.1 通用数据结构

```cpp
// 通信协议数据包格式
struct Struct_Referee_UART_Data
{
    uint8_t Frame_Header = 0xa5;  // 帧头
    uint16_t Data_Length;         // 数据长度
    uint8_t Sequence;             // 序列号
    uint8_t CRC_8;                // CRC8校验
    Enum_Referee_Command_ID Referee_Command_ID;  // 命令ID
    uint8_t Data[121];            // 数据区域
} __attribute__((packed));
```

**作用**: 定义裁判系统通信协议的通用数据包格式。

#### 1.4.2 比赛状态数据结构

```cpp
struct Struct_Referee_Rx_Data_Game_Status
{
    uint8_t Type_Enum : 4;    // 比赛类型（4位）
    uint8_t Stage_Enum : 4;   // 比赛阶段（4位）
    uint16_t Remaining_Time;  // 剩余时间
    uint64_t Timestamp;       // 时间戳
    uint16_t CRC_16;          // CRC16校验
} __attribute__((packed));
```

**作用**: 存储比赛状态信息，使用位字段节省空间。

#### 1.4.3 机器人血量数据结构

```cpp
struct Struct_Referee_Rx_Data_Game_Robot_HP
{
    uint16_t Red_Hero_1;        // 红方英雄血量
    uint16_t Red_Engineer_2;    // 红方工程血量
    uint16_t Red_Infantry_3;    // 红方步兵血量
    // ... 其他机器人血量
    uint16_t CRC_16;            // CRC16校验
};
```

**作用**: 存储所有机器人的实时血量信息。

#### 1.4.4 机器人状态数据结构

```cpp
struct Struct_Referee_Rx_Data_Robot_Status
{
    Enum_Referee_Data_Robots_ID Robot_ID;  // 机器人ID
    uint8_t Level;                        // 机器人等级
    uint16_t HP;                          // 当前血量
    uint16_t HP_Max;                      // 血量上限
    uint16_t Booster_Heat_CD;             // 发射机构冷却
    uint16_t Booster_Heat_Max;            // 发射机构热量上限
    uint16_t Chassis_Power_Max;           // 底盘功率上限
    uint8_t PM01_Gimbal_Status_Enum : 1;  // 云台供电状态
    uint8_t PM01_Chassis_Status_Enum : 1; // 底盘供电状态
    uint8_t PM01_Booster_Status_Enum : 1; // 发射机构供电状态
    uint8_t Reserved : 5;                 // 保留位
    uint16_t CRC_16;                      // CRC16校验
};
```

**作用**: 存储当前机器人自身的状态信息。

#### 1.4.5 功率热量数据结构

```cpp
struct Struct_Referee_Rx_Data_Robot_Power_Heat
{
    uint16_t Chassis_Voltage;      // 底盘电压(mV)
    uint16_t Chassis_Current;      // 底盘电流(mA)
    float Chassis_Power;           // 底盘功率(W)
    uint16_t Chassis_Energy_Buffer;// 底盘能量缓冲(J)
    uint16_t Booster_17mm_1_Heat;  // 17mm发射机构1热量
    uint16_t Booster_17mm_2_Heat;  // 17mm发射机构2热量
    uint16_t Booster_42mm_Heat;    // 42mm发射机构热量
    uint16_t CRC_16;               // CRC16校验
};
```

**作用**: 存储机器人的功率和热量相关信息。

### 1.5 图形配置结构体

```cpp
struct Struct_Referee_Data_Interaction_Graphic_Config
{
    uint8_t Index[3];           // 图形索引
    uint32_t Operation_Enum : 3; // 操作类型（3位）
    uint32_t Type_Enum : 3;     // 图形类型（3位）
    uint32_t Layer_Num : 4;     // 图层号（4位）
    uint32_t Color_Enum : 4;    // 颜色类型（4位）
    uint32_t Details_A : 9;     // 细节A（9位）
    uint32_t Details_B : 9;     // 细节B（9位）
    uint32_t Line_Width : 10;   // 线宽（10位）
    uint32_t Start_X : 11;      // 起始X坐标（11位）
    uint32_t Start_Y : 11;      // 起始Y坐标（11位）
    uint32_t Details_C : 10;    // 细节C（10位）
    uint32_t Details_D : 11;    // 细节D（11位）
    uint32_t Details_E : 11;    // 细节E（11位）
} __attribute__((packed));
```

**作用**: 定义裁判系统UI图形的配置参数，使用位字段优化存储。

### 1.6 裁判系统类定义

#### 1.6.1 类结构

```cpp
class Class_Referee
{
public:
    // 初始化函数
    void Init(UART_HandleTypeDef *huart, uint8_t __Frame_Header = 0xa5);
    
    // 状态获取函数
    inline Enum_Referee_Status Get_Status();
    
    // 比赛信息获取函数
    inline Enum_Referee_Game_Status_Type Get_Game_Type();
    inline uint16_t Get_Remaining_Time();
    inline uint64_t Get_Timestamp();
    
    // 机器人状态获取函数
    inline uint16_t Get_Self_HP();
    inline float Get_Chassis_Voltage();
    inline float Get_Chassis_Current();
    inline float Get_Location_X();
    // ... 更多获取函数
    
    // UI绘制函数
    inline Struct_Referee_Data_Interaction_Graphic_Config *Set_Referee_UI_Line(...);
    inline Struct_Referee_Data_Interaction_Graphic_Config *Set_Referee_UI_Rectangle(...);
    // ... 更多UI函数
    
    // 发送函数
    void UART_Send_Interaction_UI_Graphic_1(...);
    void UART_Send_Interaction_UI_Graphic_String(...);
    
    // 回调函数
    void UART_RxCpltCallback(uint8_t *Rx_Data, uint16_t Length);
    void TIM_1000ms_Alive_PeriodElapsedCallback();

protected:
    // 成员变量...
};
```

#### 1.6.2 保护成员变量

```cpp
protected:
    // UART管理对象
    Struct_UART_Manage_Object *UART_Manage_Object;
    uint8_t Frame_Header;
    
    // 内部变量
    uint32_t Flag = 0;           // 接收标志
    uint32_t Pre_Flag = 0;       // 前一时刻标志
    uint8_t Sequence = 0;        // 发送序列号
    uint8_t UI_Change_Flag[10][10] = {0};  // UI变化标志
    
    // 读变量 - 存储各种裁判系统数据
    Struct_Referee_Rx_Data_Game_Status Game_Status;        // 比赛状态
    Struct_Referee_Rx_Data_Robot_Status Robot_Status;      // 机器人状态
    Struct_Referee_Rx_Data_Robot_Power_Heat Robot_Power_Heat;  // 功率热量
    // ... 其他数据结构
    
    // 写变量
    Struct_Referee_Data_Interaction_Graphic_Config Graphic_Config[10][10];  // 图形配置
    
    // 读写变量
    Enum_Referee_Data_Status Referee_Trust_Status;  // 裁判系统信任状态
};
```

## 2. 实现文件分析 (dvc_referee.cpp)

### 2.1 CRC校验表

```cpp
// CRC8校验码表
static const uint8_t crc_8_table[256] = {
    0x00, 0x5e, 0xbc, 0xe2, // ... 完整表格
};

// CRC16校验码表
static const uint16_t crc_16_table[256] = {
    0x0000, 0x1189, 0x2312, 0x329b, // ... 完整表格
};
```

**作用**: 预计算的CRC校验码表，用于快速校验数据完整性。

### 2.2 初始化函数

```cpp
void Class_Referee::Init(UART_HandleTypeDef *huart, uint8_t __Frame_Header)
{
    // 根据UART实例绑定对应的管理对象
    if (huart->Instance == USART1)
        UART_Manage_Object = &UART1_Manage_Object;
    else if (huart->Instance == USART2)
        UART_Manage_Object = &UART2_Manage_Object;
    // ... 其他UART端口判断
    
    Frame_Header = __Frame_Header;  // 设置帧头标识
}
```

**作用**: 初始化裁判系统，绑定UART端口并设置通信协议参数。

### 2.3 UI发送函数

```cpp
void Class_Referee::UART_Send_Interaction_UI_Graphic_1(Struct_Referee_Data_Interaction_Graphic_Config *Graphic_1)
{
    Struct_Referee_UART_Data *tmp_buffer = (Struct_Referee_UART_Data *) UART_Manage_Object->Tx_Buffer;

    // 构建裁判系统帧头
    tmp_buffer->Frame_Header = 0xa5;
    tmp_buffer->Data_Length = sizeof(Struct_Referee_Tx_Data_Interaction_Graphic_1) - 2;
    tmp_buffer->Sequence = Sequence;  // 序列号递增
    tmp_buffer->CRC_8 = Verify_CRC_8((uint8_t *) tmp_buffer, 4);  // CRC8校验
    tmp_buffer->Referee_Command_ID = Referee_Command_ID_INTERACTION;

    // 构建交互数据
    Struct_Referee_Tx_Data_Interaction_Graphic_1 *tmp_data = (Struct_Referee_Tx_Data_Interaction_Graphic_1 *) tmp_buffer->Data;
    tmp_data->Header = Referee_Interaction_Command_ID_UI_GRAPHIC_1;
    tmp_data->Sender = Robot_Status.Robot_ID;  // 发送者为当前机器人
    tmp_data->Receiver = static_cast<Enum_Referee_Data_Robots_Client_ID>((int) (Robot_Status.Robot_ID) + 0x100);  // 接收者为对应客户端

    // 设置图形数据
    tmp_data->Graphic[0] = *Graphic_1;

    // 计算CRC16校验
    tmp_data->CRC_16 = Verify_CRC_16((uint8_t *) tmp_buffer, 7 + tmp_buffer->Data_Length);

    // 发送数据
    HAL_UART_Transmit(UART_Manage_Object->UART_Handler, (uint8_t *) tmp_buffer, 7 + sizeof(Struct_Referee_Tx_Data_Interaction_Graphic_1), 100);

    Sequence++;  // 序列号递增
}
```

**作用**: 发送单个图形到裁判系统UI，构建完整的通信协议包。

### 2.4 UART接收回调函数

```cpp
void Class_Referee::UART_RxCpltCallback(uint8_t *Rx_Data, uint16_t Length)
{
    Flag += 1;  // 增加接收标志

    Data_Process(Length);  // 处理接收到的数据
}
```

**作用**: UART接收完成时的回调函数，处理裁判系统数据。

### 2.5 存活检测函数

```cpp
void Class_Referee::TIM_1000ms_Alive_PeriodElapsedCallback()
{
    if (Flag == Pre_Flag)
    {
        Referee_Status = Referee_Status_DISABLE;  // 裁判系统断开
        UART_Reinit(UART_Manage_Object->UART_Handler);  // 重新初始化UART
    }
    else
    {
        Referee_Status = Referee_Status_ENABLE;  // 裁判系统在线
    }
    Pre_Flag = Flag;  // 更新标志
}
```

**作用**: 每1000ms检测一次裁判系统是否在线，断线时自动重连。

### 2.6 CRC校验函数

```cpp
uint8_t Class_Referee::Verify_CRC_8(uint8_t *Message, uint32_t Length)
{
    uint8_t index;
    uint8_t check = 0xff;  // 初始值为0xff

    if (Message == nullptr)
        return (check);

    while (Length--)
    {
        index = *Message;  // 获取当前字节
        Message++;
        check = crc_8_table[check ^ index];  // 查表计算CRC
    }
    return (check);
}

uint16_t Class_Referee::Verify_CRC_16(uint8_t *Message, uint32_t Length)
{
    uint8_t index;
    uint16_t check = 0xffff;  // 初始值为0xffff

    if (Message == nullptr)
        return (check);

    while (Length--)
    {
        index = *Message;
        Message++;
        // 查表计算CRC16
        check = ((uint16_t) (check) >> 8) ^ crc_16_table[((uint16_t) (check) ^ (uint16_t) (index)) & 0xff];
    }
    return (check);
}
```

**作用**: 实现CRC8和CRC16校验算法，确保数据传输的完整性。

### 2.7 数据处理函数

```cpp
void Class_Referee::Data_Process(uint16_t Length)
{
    Struct_Referee_UART_Data *tmp_buffer;

    for (int i = 0; i < Length;)
    {
        tmp_buffer = (Struct_Referee_UART_Data *) &UART_Manage_Object->Rx_Buffer[i];

        // 1. 帧头校验
        if (tmp_buffer->Frame_Header != Frame_Header)
        {
            i++;  // 头校验失败，向前移动一位
            continue;
        }
        
        // 2. CRC8校验
        if (Verify_CRC_8((uint8_t *) tmp_buffer, 4) != tmp_buffer->CRC_8)
        {
            i++;  // CRC8校验失败，向前移动一位
            continue;
        }
        
        // 3. CRC16校验
        if (Verify_CRC_16((uint8_t *) tmp_buffer, 7 + tmp_buffer->Data_Length) != 
            *(uint16_t *) ((uint32_t) tmp_buffer + 7 + tmp_buffer->Data_Length))
        {
            i += 9 + tmp_buffer->Data_Length;  // CRC16校验失败，跳过整个包
            continue;
        }
        
        // 4. 长度校验
        if (i + 7 + tmp_buffer->Data_Length + 2 > Length)
        {
            break;  // 包长度不够，结束处理
        }

        // 根据命令ID处理不同类型的数据
        switch (tmp_buffer->Referee_Command_ID)
        {
        case (Referee_Command_ID_GAME_STATUS):
            memcpy(&Game_Status, tmp_buffer->Data, sizeof(Struct_Referee_Rx_Data_Game_Status));
            break;
        case (Referee_Command_ID_ROBOT_STATUS):
            memcpy(&Robot_Status, tmp_buffer->Data, sizeof(Struct_Referee_Rx_Data_Robot_Status));
            break;
        case (Referee_Command_ID_ROBOT_POWER_HEAT):
            memcpy(&Robot_Power_Heat, tmp_buffer->Data, sizeof(Struct_Referee_Rx_Data_Robot_Power_Heat));
            break;
        // ... 其他命令处理
        }

        // 移动到下一个包
        i += 7 + tmp_buffer->Data_Length + 2;
    }
}
```

**作用**: 解析和处理从UART接收的裁判系统数据包，进行多层校验并分发到对应的数据结构。

## 3. 关键特性分析

### 3.1 通信协议特性

- **分层校验**: 帧头校验 → CRC8校验 → CRC16校验
- **滑动窗口**: 通过接收标志检测连接状态
- **多包处理**: 支持单次接收多个数据包

### 3.2 数据结构优化

- **位字段**: 在有限的带宽下最大化信息密度
- **结构化存储**: 将不同类型的裁判系统数据分别存储
- **内存对齐**: 使用`__attribute__((packed))`减少内存占用

### 3.3 UI交互功能

- **图形绘制**: 支持线条、矩形、圆形等多种图形
- **图层管理**: 支持最多10个图层，每个图层10个图形
- **动态更新**: 自动管理图形的添加、修改、删除操作

## 4. 类的作用域和外设资源

### 4.1 作用域

- **公共作用域(public)**: 提供数据获取、UI绘制、通信接口
- **保护作用域(protected)**: 内部数据存储、处理算法、校验函数

### 4.2 使用的外设资源

- **UART接口**: 用于与裁判系统通信，支持USART1-8和UART4-8
- **定时器**: 用于1000ms设备存活检测
- **内存资源**: 接收和发送缓冲区存储通信数据
- **数学库**: CRC校验算法

### 4.3 工作流程

1. 初始化时绑定UART端口并设置帧头
2. UART接收数据触发回调函数，解析裁判系统数据
3. 1000ms定时器检测裁判系统在线状态
4. 用户通过Get系列函数获取实时比赛信息
5. 通过UI函数向裁判系统发送图形绘制指令

这个驱动程序实现了完整的裁判系统通信协议，支持实时数据获取和UI交互功能，是机器人比赛系统的核心组件。