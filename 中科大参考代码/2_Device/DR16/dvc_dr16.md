# DR16遥控器代码深度解析

## 1. 头文件分析 (dvc_dr16.h)

### 1.1 文件概述

这是一个用于大疆DR16遥控器的驱动头文件，实现了通过UART接口接收和解析遥控器数据的功能，支持摇杆、开关、鼠标、键盘等多种输入设备。

### 1.2 包含的头文件

```cpp
#include "1_Middleware/1_Driver/UART/drv_uart.h"  // UART驱动库
```

### 1.3 宏定义

#### 1.3.1 开关位置宏定义

```cpp
#define DR16_SWITCH_UP (1)        // 开关向上
#define DR16_SWITCH_DOWN (2)      // 开关向下
#define DR16_SWITCH_MIDDLE (3)    // 开关中间
```

#### 1.3.2 按键状态宏定义

```cpp
#define DR16_KEY_FREE (0)         // 按键释放
#define DR16_KEY_PRESSED (1)      // 按键按下
```

#### 1.3.3 键位宏定义

```cpp
#define DR16_KEY_W 0      // W键索引
#define DR16_KEY_S 1      // S键索引
#define DR16_KEY_A 2      // A键索引
// ... 其他键位定义
```

### 1.4 枚举类型定义

#### 1.4.1 遥控器状态枚举

```cpp
enum Enum_DR16_Status
{
    DR16_Status_DISABLE = 0,  // 遥控器断开连接
    DR16_Status_ENABLE,       // 遥控器正常连接
};
```

**作用**: 表示DR16遥控器的连接状态。

#### 1.4.2 开关状态枚举

```cpp
enum Enum_DR16_Switch_Status
{
    DR16_Switch_Status_UP = 0,           // 开关在上方
    DR16_Switch_Status_TRIG_UP_MIDDLE,   // 从上到中触发状态
    DR16_Switch_Status_TRIG_MIDDLE_UP,   // 从中到上触发状态
    DR16_Switch_Status_MIDDLE,           // 开关在中间
    DR16_Switch_Status_TRIG_MIDDLE_DOWN, // 从中到下触发状态
    DR16_Switch_Status_TRIG_DOWN_MIDDLE, // 从下到中触发状态
    DR16_Switch_Status_DOWN,             // 开关在下方
};
```

**作用**: 不仅记录开关的当前位置，还记录切换过程中的触发状态。

#### 1.4.3 按键状态枚举

```cpp
enum Enum_DR16_Key_Status
{
    DR16_Key_Status_FREE = 0,              // 按键释放
    DR16_Key_Status_TRIG_FREE_PRESSED,     // 从释放到按下的触发状态
    DR16_Key_Status_TRIG_PRESSED_FREE,     // 从按下到释放的触发状态
    DR16_Key_Status_PRESSED,               // 按键按下
};
```

**作用**: 记录按键的状态变化，支持边沿触发检测。

### 1.5 结构体定义

#### 1.5.1 UART原始数据结构

```cpp
struct Struct_DR16_UART_Data
{
    uint64_t Channel_0 : 11;        // 右侧X轴摇杆（11位）
    uint64_t Channel_1 : 11;        // 右侧Y轴摇杆（11位）
    uint64_t Channel_2 : 11;        // 左侧X轴摇杆（11位）
    uint64_t Channel_3 : 11;        // 左侧Y轴摇杆（11位）
    uint64_t Switch_2 : 2;          // 右侧开关（2位）
    uint64_t Switch_1 : 2;          // 左侧开关（2位）
    int16_t Mouse_X;                // 鼠标X轴
    int16_t Mouse_Y;                // 鼠标Y轴
    int16_t Mouse_Z;                // 鼠标Z轴
    uint64_t Mouse_Left_Key : 8;    // 鼠标左键（8位，实际只用1位）
    uint64_t Mouse_Right_Key : 8;   // 鼠标右键（8位，实际只用1位）
    uint64_t Keyboard_Key : 16;     // 键盘按键（16位）
    uint64_t Channel_Yaw : 11;      // Yaw轴（11位）
} __attribute__((packed));
```

**作用**: 定义DR16遥控器发送的原始数据包格式，使用位域减少内存占用。

#### 1.5.2 处理后数据结构

```cpp
struct Struct_DR16_Data
{
    float Right_X;                      // 右侧X轴摇杆（归一化到-1~1）
    float Right_Y;                      // 右侧Y轴摇杆（归一化到-1~1）
    float Left_X;                       // 左侧X轴摇杆（归一化到-1~1）
    float Left_Y;                       // 左侧Y轴摇杆（归一化到-1~1）
    Enum_DR16_Switch_Status Left_Switch;  // 左侧开关状态
    Enum_DR16_Switch_Status Right_Switch; // 右侧开关状态
    float Mouse_X;                      // 鼠标X轴（归一化）
    float Mouse_Y;                      // 鼠标Y轴（归一化）
    float Mouse_Z;                      // 鼠标Z轴（归一化）
    Enum_DR16_Key_Status Mouse_Left_Key;  // 鼠标左键状态
    Enum_DR16_Key_Status Mouse_Right_Key; // 鼠标右键状态
    Enum_DR16_Key_Status Keyboard_Key[16]; // 16个键盘按键状态
    float Yaw;                          // Yaw轴（归一化到-1~1）
};
```

**作用**: 存储解析和归一化后的遥控器数据，便于应用程序使用。

### 1.6 主类定义

#### 1.6.1 类成员变量

```cpp
class Class_DR16
{
protected:
    // 初始化相关常量
    Struct_UART_Manage_Object *UART_Manage_Object;  // 绑定的UART管理对象

    // 常量
    float Rocker_Offset = 1024.0f;  // 摇杆中值偏移量
    float Rocker_Num = 660.0f;      // 摇杆数值范围

    // 内部变量
    Struct_DR16_UART_Data Pre_UART_Rx_Data;  // 前一时刻的原始数据（用于触发检测）
    uint32_t Flag = 0;                       // 当前时刻接收标志
    uint32_t Pre_Flag = 0;                   // 前一时刻接收标志

    // 读变量
    Enum_DR16_Status DR16_Status = DR16_Status_DISABLE;  // 遥控器状态
    Struct_DR16_Data Data;                               // 处理后的数据

    // 内部函数
    void Data_Process(uint16_t Length);                  // 数据处理函数
    void _Judge_Switch(Enum_DR16_Switch_Status *Switch, uint8_t Status, uint8_t Pre_Status);  // 开关状态判断
    void _Judge_Key(Enum_DR16_Key_Status *Key, uint8_t Status, uint8_t Pre_Status);          // 按键状态判断
};
```

#### 1.6.2 公共接口函数声明

```cpp
public:
    // 初始化函数
    void Init(UART_HandleTypeDef *huart);
    
    // 状态获取函数
    inline Enum_DR16_Status Get_Status();
    
    // 摇杆数据获取函数
    inline float Get_Right_X();     // 获取右侧X轴摇杆
    inline float Get_Right_Y();     // 获取右侧Y轴摇杆
    inline float Get_Left_X();      // 获取左侧X轴摇杆
    inline float Get_Left_Y();      // 获取左侧Y轴摇杆
    
    // 开关数据获取函数
    inline Enum_DR16_Switch_Status Get_Left_Switch();   // 获取左侧开关状态
    inline Enum_DR16_Switch_Status Get_Right_Switch();  // 获取右侧开关状态
    
    // 鼠标数据获取函数
    inline float Get_Mouse_X();     // 获取鼠标X轴
    inline float Get_Mouse_Y();     // 获取鼠标Y轴
    inline float Get_Mouse_Z();     // 获取鼠标Z轴
    inline Enum_DR16_Key_Status Get_Mouse_Left_Key();   // 获取鼠标左键状态
    inline Enum_DR16_Key_Status Get_Mouse_Right_Key();  // 获取鼠标右键状态
    
    // 键盘数据获取函数（16个按键）
    inline Enum_DR16_Key_Status Get_Keyboard_Key_W();   // 获取W键状态
    inline Enum_DR16_Key_Status Get_Keyboard_Key_S();   // 获取S键状态
    // ... 其他按键获取函数
    
    // Yaw轴数据获取函数
    inline float Get_Yaw();         // 获取Yaw轴
    
    // 回调函数
    void UART_RxCpltCallback(uint8_t *Rx_Data, uint16_t Length);  // UART接收完成回调
    void TIM_100ms_Alive_PeriodElapsedCallback();                 // 定时器存活检测回调
    void TIM_1ms_Calculate_PeriodElapsedCallback();               // 定时器计算回调
```

## 2. 实现文件分析 (dvc_dr16.cpp)

### 2.1 初始化函数

```cpp
void Class_DR16::Init(UART_HandleTypeDef *huart)
{
    // 根据UART实例绑定对应的管理对象
    if (huart->Instance == USART1) {
        UART_Manage_Object = &UART1_Manage_Object;
    }
    else if (huart->Instance == USART2) {
        UART_Manage_Object = &UART2_Manage_Object;
    }
    // ... 其他UART端口判断
}
```

**作用**: 初始化DR16遥控器驱动，绑定指定的UART端口。

### 2.2 UART接收回调函数

```cpp
void Class_DR16::UART_RxCpltCallback(uint8_t *Rx_Data, uint16_t Length)
{
    // 滑动窗口，判断遥控器DR16是否在线
    Flag += 1;
    Data_Process(Length);
}
```

**作用**: UART接收完成时的回调函数，增加接收标志并处理接收到的数据。

### 2.3 定时器存活检测函数

```cpp
void Class_DR16::TIM_100ms_Alive_PeriodElapsedCallback()
{
    // 判断该时间段内是否接收过遥控器DR16数据
    if (Flag == Pre_Flag)
    {
        // 遥控器DR16断开连接
        DR16_Status = DR16_Status_DISABLE;
        UART_Reinit(UART_Manage_Object->UART_Handler);  // 重新初始化UART
    }
    else
    {
        // 遥控器DR16保持连接
        DR16_Status = DR16_Status_ENABLE;
    }
    Pre_Flag = Flag;  // 更新上一时刻标志
}
```

**作用**: 每100ms执行一次，检测遥控器是否在线。

### 2.4 定时器计算函数

```cpp
void Class_DR16::TIM_1ms_Calculate_PeriodElapsedCallback()
{
    // 获取当前原始数据
    Struct_DR16_UART_Data *tmp_buffer = (Struct_DR16_UART_Data *) UART_Manage_Object->Rx_Buffer;

    // 判断拨码触发状态
    _Judge_Switch(&Data.Left_Switch, tmp_buffer->Switch_1, Pre_UART_Rx_Data.Switch_1);
    _Judge_Switch(&Data.Right_Switch, tmp_buffer->Switch_2, Pre_UART_Rx_Data.Switch_2);

    // 判断鼠标触发状态
    _Judge_Key(&Data.Mouse_Left_Key, tmp_buffer->Mouse_Left_Key, Pre_UART_Rx_Data.Mouse_Left_Key);
    _Judge_Key(&Data.Mouse_Right_Key, tmp_buffer->Mouse_Right_Key, Pre_UART_Rx_Data.Mouse_Right_Key);

    // 判断键盘触发状态
    for (int i = 0; i < 16; i++)
    {
        // 从16位键盘按键数据中提取第i位
        _Judge_Key(&Data.Keyboard_Key[i], 
                  ((tmp_buffer->Keyboard_Key) >> i) & 0x1, 
                  ((Pre_UART_Rx_Data.Keyboard_Key) >> i) & 0x1);
    }

    // 保存当前数据作为下一时刻的前一数据
    memcpy(&Pre_UART_Rx_Data, tmp_buffer, 18 * sizeof(uint8_t));
}
```

**作用**: 每1ms执行一次，主要用于检测状态变化的触发事件（边沿触发）。

### 2.5 数据处理函数

```cpp
void Class_DR16::Data_Process(uint16_t Length)
{
    // 获取原始数据
    Struct_DR16_UART_Data *tmp_buffer = (Struct_DR16_UART_Data *) UART_Manage_Object->Rx_Buffer;

    // 摇杆信息归一化处理（转换为-1到1的范围）
    Data.Right_X = (tmp_buffer->Channel_0 - Rocker_Offset) / Rocker_Num;
    Data.Right_Y = (tmp_buffer->Channel_1 - Rocker_Offset) / Rocker_Num;
    Data.Left_X = (tmp_buffer->Channel_2 - Rocker_Offset) / Rocker_Num;
    Data.Left_Y = (tmp_buffer->Channel_3 - Rocker_Offset) / Rocker_Num;

    // 鼠标信息归一化处理
    Data.Mouse_X = tmp_buffer->Mouse_X / 32768.0f;
    Data.Mouse_Y = tmp_buffer->Mouse_Y / 32768.0f;
    Data.Mouse_Z = tmp_buffer->Mouse_Z / 32768.0f;

    // Yaw轴信息归一化处理
    Data.Yaw = (tmp_buffer->Channel_Yaw - Rocker_Offset) / Rocker_Num;
}
```

**作用**: 解析原始数据并将其转换为标准化的浮点数值。

### 2.6 开关状态判断函数

```cpp
void Class_DR16::_Judge_Switch(Enum_DR16_Switch_Status *Switch, uint8_t Status, uint8_t Pre_Status)
{
    // 根据前一状态和当前状态判断开关状态
    switch (Pre_Status)
    {
    case (DR16_SWITCH_UP):
    {
        switch (Status)
        {
        case (DR16_SWITCH_UP):
            *Switch = DR16_Switch_Status_UP;  // 状态不变
            break;
        case (DR16_SWITCH_DOWN):
            *Switch = DR16_Switch_Status_TRIG_MIDDLE_DOWN;  // 从上到下的中间触发
            break;
        case (DR16_SWITCH_MIDDLE):
            *Switch = DR16_Switch_Status_TRIG_UP_MIDDLE;    // 从上到中的触发
            break;
        }
        break;
    }
    // ... 其他前一状态的判断
    }
}
```

**作用**: 根据开关的前一状态和当前状态，确定开关的变化情况和触发状态。

### 2.7 按键状态判断函数

```cpp
void Class_DR16::_Judge_Key(Enum_DR16_Key_Status *Key, uint8_t Status, uint8_t Pre_Status)
{
    // 根据前一状态和当前状态判断按键状态
    switch (Pre_Status)
    {
    case (DR16_KEY_FREE):
    {
        switch (Status)
        {
        case (DR16_KEY_FREE):
            *Key = DR16_Key_Status_FREE;  // 保持释放
            break;
        case (DR16_KEY_PRESSED):
            *Key = DR16_Key_Status_TRIG_FREE_PRESSED;  // 从释放到按下的触发
            break;
        }
        break;
    }
    case (DR16_KEY_PRESSED):
    {
        switch (Status)
        {
        case (DR16_KEY_FREE):
            *Key = DR16_Key_Status_TRIG_PRESSED_FREE;  // 从按下到释放的触发
            break;
        case (DR16_KEY_PRESSED):
            *Key = DR16_Key_Status_PRESSED;  // 保持按下
            break;
        }
        break;
    }
    }
}
```

**作用**: 根据按键的前一状态和当前状态，确定按键的变化情况和触发状态。

## 3. 关键特性分析

### 3.1 数据归一化

- **摇杆数据**: `(raw_value - 1024.0) / 660.0` 转换为-1到1的范围
- **鼠标数据**: `raw_value / 32768.0` 归一化处理

### 3.2 触发检测机制

- **开关触发**: 支持从一种状态到另一种状态的边沿触发检测
- **按键触发**: 支持按下和释放的边沿触发检测

### 3.3 双定时器机制

- **100ms定时器**: 检测设备在线状态
- **1ms定时器**: 检测状态变化触发

## 4. 类的作用域和外设资源

### 4.1 作用域

- **公共作用域(public)**: 提供对外接口，包括初始化、数据获取和回调函数
- **保护作用域(protected)**: 内部实现细节，包括数据存储、状态判断等

### 4.2 使用的外设资源

- **UART接口**: 用于与DR16遥控器通信，支持USART1-8和UART4-8
- **定时器**: 用于100ms设备存活检测和1ms状态变化检测
- **内存资源**: 接收缓冲区存储原始数据，结构体存储处理后的数据

### 4.3 工作流程

1. 初始化时绑定UART端口
2. UART接收数据触发回调函数，处理原始数据
3. 1ms定时器检测状态变化，生成触发事件
4. 100ms定时器检测设备在线状态
5. 用户通过Get系列函数获取处理后的遥控器数据

这个驱动程序提供了完整的DR16遥控器支持，包括摇杆、开关、鼠标、键盘等所有输入设备的状态获取和边沿触发检测功能。