# 串口绘图驱动代码深度解析

## 1. 头文件分析 (dvc_serialplot.h)

### 1.1 文件概述

这是一个用于串口绘图的驱动头文件，版本0.1为23赛季定稿版本，支持通过串口发送数据进行实时绘图，最多支持12个通道。

### 1.2 包含的头文件

```cpp
#include "1_Middleware/1_Driver/UART/drv_uart.h"    // UART驱动库
#include "1_Middleware/1_Driver/Math/drv_math.h"    // 数学运算库
#include <stdarg.h>                                  // 可变参数库
```

### 1.3 宏定义

```cpp
// 串口绘图单条指令最大长度
#define SERIALPLOT_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH (100)
```

**作用**: 定义接收变量名的最大长度限制。

### 1.4 枚举类型定义

#### 1.4.1 校验和开关枚举

```cpp
enum Enum_Serialplot_Checksum_8
{
    Serialplot_Checksum_8_DISABLE = 0,  // 禁用校验和
    Serialplot_Checksum_8_ENABLE,       // 启用校验和
};
```

**作用**: 控制是否启用8位校验和功能。

#### 1.4.2 数据类型枚举

```cpp
enum Enum_Serialplot_Data_Type
{
    Serialplot_Data_Type_UINT8 = 0,   // 无符号8位整型
    Serialplot_Data_Type_UINT16,     // 无符号16位整型
    Serialplot_Data_Type_UINT32,     // 无符号32位整型
    Serialplot_Data_Type_INT8,       // 有符号8位整型
    Serialplot_Data_Type_INT16,      // 有符号16位整型
    Serialplot_Data_Type_INT32,      // 有符号32位整型
    Serialplot_Data_Type_FLOAT,      // 单精度浮点型
    Serialplot_Data_Type_DOUBLE,     // 双精度浮点型
};
```

**作用**: 定义串口绘图支持的数据类型。

### 1.5 串口绘图类定义

#### 1.5.1 类结构

```cpp
class Class_Serialplot
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
    void Init(UART_HandleTypeDef *huart, 
              Enum_Serialplot_Checksum_8 __Checksum_8 = Serialplot_Checksum_8_ENABLE, 
              uint8_t __Rx_Variable_Assignment_Num = 0, 
              char **__Rx_Variable_Assignment_List = NULL, 
              Enum_Serialplot_Data_Type __Data_Type = Serialplot_Data_Type_FLOAT, 
              uint8_t __Frame_Header = 0xab);

    // 获取函数
    inline int8_t Get_Variable_Index();      // 获取接收变量索引
    inline float Get_Variable_Value();       // 获取接收变量值

    // 设置函数
    inline void Set_Data(uint8_t Number, ...);  // 设置发送数据

    // 回调函数
    void UART_RxCpltCallback(uint8_t *Rx_Data, uint16_t Length);  // UART接收回调
    void TIM_1ms_Write_PeriodElapsedCallback();                    // 定时器发送回调
```

#### 1.5.3 保护成员变量

```cpp
protected:
    // 初始化相关常量
    Struct_UART_Manage_Object *UART_Manage_Object;  // UART管理对象
    Enum_Serialplot_Checksum_8 Checksum_8;         // 校验和开关
    uint8_t Rx_Variable_Num;                       // 接收变量数量
    char **Rx_Variable_List;                       // 接收变量列表
    Enum_Serialplot_Data_Type Tx_Data_Type;        // 发送数据类型
    uint8_t Frame_Header;                          // 帧头标识

    // 内部变量
    const void *Data[25];                          // 发送数据地址数组
    uint8_t Data_Number = 0;                       // 发送数据数量
    int8_t Variable_Index = 0;                     // 接收变量索引
    float Variable_Value = 0.0f;                   // 接收变量值

    // 内部函数
    void Data_Process(uint16_t Length);             // 数据处理
    uint8_t _Judge_Variable_Name(uint16_t Length);  // 判断变量名
    void _Judge_Variable_Value(uint16_t Length, int flag);  // 判断变量值
    void Output();                                  // 输出数据
```

#### 1.5.4 内联函数实现

```cpp
// 获取接收变量索引
inline int8_t Class_Serialplot::Get_Variable_Index()
{
    return (Variable_Index);
}

// 获取接收变量值
inline float Class_Serialplot::Get_Variable_Value()
{
    return (Variable_Value);
}

// 设置发送数据
inline void Class_Serialplot::Set_Data(uint8_t Number, ...)
{
    va_list data_ptr;
    va_start(data_ptr, Number);
    for (int i = 0; i < Number; i++)
    {
        Data[i] = (void *) va_arg(data_ptr, int);  // 获取可变参数
    }
    va_end(data_ptr);
    Data_Number = Number;  // 更新数据数量
}
```

## 2. 实现文件分析 (dvc_serialplot.cpp)

### 2.1 初始化函数

```cpp
void Class_Serialplot::Init(UART_HandleTypeDef *huart, 
                           Enum_Serialplot_Checksum_8 __Checksum_8, 
                           uint8_t __Rx_Variable_Assignment_Num, 
                           char **__Rx_Variable_Assignment_List, 
                           Enum_Serialplot_Data_Type __Data_Type, 
                           uint8_t __Frame_Header)
{
    // 根据UART实例绑定对应的管理对象
    if (huart->Instance == USART1)
        UART_Manage_Object = &UART1_Manage_Object;
    else if (huart->Instance == USART2)
        UART_Manage_Object = &UART2_Manage_Object;
    // ... 其他UART端口判断
    
    // 设置各项参数
    Checksum_8 = __Checksum_8;
    Rx_Variable_Num = __Rx_Variable_Assignment_Num;
    Rx_Variable_List = __Rx_Variable_Assignment_List;
    Tx_Data_Type = __Data_Type;
    Frame_Header = __Frame_Header;

    // 在发送缓冲区首位置入帧头
    UART_Manage_Object->Tx_Buffer[0] = __Frame_Header;
}
```

**作用**: 初始化串口绘图系统，绑定UART端口并设置通信参数。

### 2.2 UART接收回调函数

```cpp
void Class_Serialplot::UART_RxCpltCallback(uint8_t *Rx_Data, uint16_t Length)
{
    Data_Process(Length);  // 处理接收到的数据
}
```

**作用**: UART接收完成时的回调函数，处理串口绘图接收的数据。

### 2.3 定时器发送回调函数

```cpp
void Class_Serialplot::TIM_1ms_Write_PeriodElapsedCallback()
{
    Output();  // 构建发送数据

    size_t data_length = 1;  // 至少包含帧头
    if(Checksum_8 == Serialplot_Checksum_8_ENABLE)
    {
        data_length++;  // 如果启用校验和，增加校验字节
    }

    // 根据数据类型计算总长度
    switch (Tx_Data_Type)
    {
    case (Serialplot_Data_Type_UINT8):
        data_length += Data_Number * sizeof(uint8_t);
        break;
    case (Serialplot_Data_Type_UINT16):
        data_length += Data_Number * sizeof(uint16_t);
        break;
    case (Serialplot_Data_Type_UINT32):
        data_length += Data_Number * sizeof(uint32_t);
        break;
    case (Serialplot_Data_Type_FLOAT):
        data_length += Data_Number * sizeof(float);
        break;
    case (Serialplot_Data_Type_DOUBLE):
        data_length += Data_Number * sizeof(double);
        break;
    // ... 其他数据类型
    }

    // 使用中断方式发送数据（因为没开DMA）
    HAL_UART_Transmit_IT(UART_Manage_Object->UART_Handler, UART_Manage_Object->Tx_Buffer, data_length);
}
```

**作用**: 每1ms执行一次，构建并发送串口绘图数据包。

### 2.4 数据处理函数

```cpp
void Class_Serialplot::Data_Process(uint16_t Length)
{
    int flag;
    flag = _Judge_Variable_Name(Length);  // 解析变量名
    _Judge_Variable_Value(Length, flag);  // 解析变量值
}
```

**作用**: 解析接收到的串口绘图数据，分为变量名解析和变量值解析两步。

### 2.5 变量名判断函数

```cpp
uint8_t Class_Serialplot::_Judge_Variable_Name(uint16_t Length)
{
    // 临时存储变量名
    char tmp_variable_name[SERIALPLOT_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH];
    // 等号位置标记
    int flag;

    // 记录变量名并标记等号位置
    for (flag = 0; UART_Manage_Object->Rx_Buffer[flag] != '=' && flag < Length && UART_Manage_Object->Rx_Buffer[flag] != 0; flag++)
    {
        tmp_variable_name[flag] = UART_Manage_Object->Rx_Buffer[flag];
    }
    tmp_variable_name[flag] = 0;  // 添加字符串结束符

    // 比对是否在列表中
    for (int i = 0; i < Rx_Variable_Num; i++)
    {
        // 如果在则标记变量名编号
        if (strcmp(tmp_variable_name, (char *) ((int) Rx_Variable_List + SERIALPLOT_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH * i)) == 0)
        {
            Variable_Index = i;
            return (flag + 1);  // 返回等号后的位置
        }
    }
    // 如果变量名不在则-1
    Variable_Index = -1;
    return (flag + 1);
}
```

**作用**: 解析接收到的字符串中的变量名部分，格式为"variable=value#"。

### 2.6 变量值判断函数

```cpp
void Class_Serialplot::_Judge_Variable_Value(uint16_t Length, int flag)
{
    // 小数点位置, 是否有负号
    int tmp_dot_flag, tmp_sign_coefficient, i;

    tmp_dot_flag = 0;
    tmp_sign_coefficient = 1;
    Variable_Value = 0.0f;

    // 列表里没有, 没必要比对直接返回
    if (Variable_Index == -1)
    {
        return;
    }

    // 判断是否有负号
    if (UART_Manage_Object->Rx_Buffer[flag] == '-')
    {
        tmp_sign_coefficient = -1;
        flag++;
    }

    // 计算值并注意小数点是否存在及其位置
    for (i = flag; UART_Manage_Object->Rx_Buffer[i] != '#' && i < Length && UART_Manage_Object->Rx_Buffer[flag] != 0; i++)
    {
        if (UART_Manage_Object->Rx_Buffer[i] == '.')
        {
            tmp_dot_flag = i;  // 记录小数点位置
        }
        else
        {
            Variable_Value = Variable_Value * 10.0f + (UART_Manage_Object->Rx_Buffer[i] - '0');  // 累加数字
        }
    }

    // 如果有小数点则考虑
    if (tmp_dot_flag != 0)
    {
        Variable_Value /= pow(10.0f, i - tmp_dot_flag - 1.0f);  // 处理小数部分
    }

    Variable_Value *= tmp_sign_coefficient;  // 应用符号
}
```

**作用**: 解析接收到的字符串中的变量值部分，支持正负号和小数点。

### 2.7 输出函数

```cpp
void Class_Serialplot::Output()
{
    uint8_t *tmp_buffer = UART_Manage_Object->Tx_Buffer;

    memset(tmp_buffer, 0, UART_BUFFER_SIZE);  // 清空发送缓冲区

    // 放置帧头
    tmp_buffer[0] = Frame_Header;

    // 根据数据类型填充数据
    if (Tx_Data_Type == Serialplot_Data_Type_UINT8 || Tx_Data_Type == Serialplot_Data_Type_INT8)
    {
        for (int i = 0; i < Data_Number; i++)
        {
            memcpy(tmp_buffer + i * sizeof(uint8_t) + 1, Data[i], sizeof(uint8_t));  // 复制8位数据
        }

        if(Checksum_8 != Serialplot_Checksum_8_DISABLE)
        {
            tmp_buffer[1 + Data_Number * sizeof(uint8_t)] = Math_Sum_8(tmp_buffer + 1, Data_Number * sizeof(uint8_t));  // 计算校验和
        }
    }
    else if (Tx_Data_Type == Serialplot_Data_Type_UINT16 || Tx_Data_Type == Serialplot_Data_Type_INT16)
    {
        for (int i = 0; i < Data_Number; i++)
        {
            memcpy(tmp_buffer + i * sizeof(uint16_t) + 1, Data[i], sizeof(uint16_t));  // 复制16位数据
        }

        if(Checksum_8 != Serialplot_Checksum_8_DISABLE)
        {
            tmp_buffer[1 + Data_Number * sizeof(uint16_t)] = Math_Sum_8(tmp_buffer + 1, Data_Number * sizeof(uint16_t));  // 计算校验和
        }
    }
    else if (Tx_Data_Type == Serialplot_Data_Type_UINT32 || Tx_Data_Type == Serialplot_Data_Type_INT32 || Tx_Data_Type == Serialplot_Data_Type_FLOAT)
    {
        for (int i = 0; i < Data_Number; i++)
        {
            memcpy(tmp_buffer + i * sizeof(uint32_t) + 1, Data[i], sizeof(uint32_t));  // 复制32位数据
        }

        if(Checksum_8 != Serialplot_Checksum_8_DISABLE)
        {
            tmp_buffer[1 + Data_Number * sizeof(uint32_t)] = Math_Sum_8(tmp_buffer + 1, Data_Number * sizeof(uint32_t));  // 计算校验和
        }
    }
    else if (Tx_Data_Type == Serialplot_Data_Type_DOUBLE)
    {
        for (int i = 0; i < Data_Number; i++)
        {
            memcpy(tmp_buffer + i * sizeof(uint64_t) + 1, Data[i], sizeof(uint64_t));  // 复制64位数据
        }

        if(Checksum_8 != Serialplot_Checksum_8_DISABLE)
        {
            tmp_buffer[1 + Data_Number * sizeof(uint64_t)] = Math_Sum_8(tmp_buffer + 1, Data_Number * sizeof(uint64_t));  // 计算校验和
        }
    }
}
```

**作用**: 将设置的数据打包成串口绘图协议格式，包含帧头、数据和校验和。

## 3. 关键特性分析

### 3.1 双向通信

- **发送**: 定期发送实时数据用于绘图
- **接收**: 解析接收的变量赋值指令

### 3.2 灵活的数据类型

- 支持多种整型和浮点型数据
- 可配置数据传输类型

### 3.3 校验和保护

- 可选的8位校验和功能
- 确保数据传输的可靠性

### 3.4 可变参数支持

- 使用`va_list`支持可变数量的数据
- 通过`Set_Data`函数灵活设置发送数据

## 4. 类的作用域和外设资源

### 4.1 作用域

- **公共作用域(public)**: 提供初始化、数据设置、获取和通信接口
- **保护作用域(protected)**: 内部实现细节，包括数据处理、解析算法等

### 4.2 使用的外设资源

- **UART接口**: 用于串口绘图通信，支持USART1-8和UART4-8
- **定时器**: 用于1ms周期性发送数据
- **内存资源**: 发送和接收缓冲区存储通信数据
- **数学库**: 校验和计算等数学运算

### 4.3 工作流程

1. 初始化时绑定UART端口和设置通信参数
2. 通过`Set_Data`函数设置需要发送的数据
3. 1ms定时器回调函数打包并发送数据
4. UART接收回调函数解析接收的变量赋值
5. 用户可通过Get系列函数获取解析结果

## 5. 通信协议格式

### 5.1 发送协议

- **格式**: `[帧头][数据1][数据2]...[校验和]
- **帧头**: 由`Frame_Header`参数指定
- **数据**: 根据`Tx_Data_Type`指定的数据类型
- **校验和**: 可选的8位校验和

### 5.2 接收协议

- **格式**: `variable_name=value#`
- **变量名**: 长度不超过100字符
- **数值**: 支持正负号和小数点
- **结束符**: `#`表示数据结束

这个串口绘图驱动程序提供了一个完整的双向通信解决方案，既可发送实时数据进行绘图，也可接收远程变量赋值指令，适用于调试和监控应用。