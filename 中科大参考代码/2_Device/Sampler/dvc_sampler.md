# ADC采样器驱动代码深度解析

## 1. 头文件分析 (dvc_sampler.h)

### 1.1 文件概述

这是一个用于ADC采样器的驱动头文件，版本0.1为23赛季定稿版本，支持多种滤波算法，是一个可复用的采样器类模板。

### 1.2 包含的头文件

```cpp
#include "1_Middleware/2_Algorithm/Filter/alg_filter.h"  // 滤波算法库
#include "1_Middleware/1_Driver/ADC/drv_adc.h"          // ADC驱动库
```

### 1.3 枚举类型定义

#### 1.3.1 滤波器类型枚举

```cpp
enum Enum_Sampler_Filter
{
    Sampler_Filter_NULL = 0,     // 无滤波
    Sampler_Filter_FOURIER,      // 傅里叶滤波
};
```

**作用**: 定义采样器可用的滤波算法类型。

### 1.4 模板类定义

#### 1.4.1 类模板声明

```cpp
template<uint32_t Filter_Fourier_Order>
class Class_Sampler
{
public:
    // 滤波器算法
    Class_Filter_Fourier<Filter_Fourier_Order> Filter_Fourier;
    
    // 公共接口函数
    void Init(ADC_HandleTypeDef *hadc, uint16_t __Sampler_Serial, Enum_Sampler_Filter __Sampler_Filter);
    inline float Get_Value();
    void TIM_Sampler_PeriodElapsedCallback();

protected:
    // 成员变量...
};
```

**作用**: 定义采样器类模板，`Filter_Fourier_Order`参数指定傅里叶滤波器的阶数。

#### 1.4.2 公共接口函数声明

```cpp
public:
    // 傅里叶滤波器实例
    Class_Filter_Fourier<Filter_Fourier_Order> Filter_Fourier;

    // 初始化函数
    void Init(ADC_HandleTypeDef *hadc, uint16_t __Sampler_Serial, Enum_Sampler_Filter __Sampler_Filter);
    
    // 获取采样值函数
    inline float Get_Value();
    
    // 定时器回调函数
    void TIM_Sampler_PeriodElapsedCallback();
```

#### 1.4.3 保护成员变量

```cpp
protected:
    // 初始化相关常量
    Struct_ADC_Manage_Object *ADC_Manage_Object;  // ADC管理对象
    uint16_t Sampler_Serial;                     // 采样通道序列号
    Enum_Sampler_Filter Sampler_Filter;          // 滤波器类型
    uint16_t *ADC_Value;                         // ADC数据指针

    // 读变量
    float Value = 0.0f;                          // 滤波后的采样值
```

#### 1.4.4 内联函数实现

```cpp
template<uint32_t Filter_Fourier_Order>
inline float Class_Sampler<Filter_Fourier_Order>::Get_Value()
{
    return (Value);
}
```

**作用**: 获取归一化的采样值（0~1范围）。

## 2. 实现文件分析 (dvc_sampler.cpp)

### 2.1 初始化函数

```cpp
template<uint32_t Filter_Fourier_Order>
void Class_Sampler<Filter_Fourier_Order>::Init(ADC_HandleTypeDef *hadc, uint16_t __Sampler_Serial, Enum_Sampler_Filter __Sampler_Filter)
{
    // 根据ADC实例绑定对应的管理对象
    if (hadc->Instance == ADC1)
    {
        ADC_Manage_Object = &ADC1_Manage_Object;
    }
    else if (hadc->Instance == ADC2)
    {
        ADC_Manage_Object = &ADC2_Manage_Object;
    }
    else if (hadc->Instance == ADC3)
    {
        ADC_Manage_Object = &ADC3_Manage_Object;
    }
    
    // 设置采样通道和滤波器类型
    Sampler_Serial = __Sampler_Serial;
    Sampler_Filter = __Sampler_Filter;

    // 设置ADC数据指针，指向指定通道的采样值
    ADC_Value = &ADC_Manage_Object->ADC_Data[Sampler_Serial];
}
```

**作用**: 初始化采样器系统，绑定ADC端口、设置采样通道和滤波器类型。

### 2.2 定时器采样回调函数

```cpp
template<uint32_t Filter_Fourier_Order>
void Class_Sampler<Filter_Fourier_Order>::TIM_Sampler_PeriodElapsedCallback()
{
    // 选择滤波器，若没有则直接输出
    switch (Sampler_Filter)
    {
    case (Sampler_Filter_NULL):
    {
        // 无滤波：直接归一化ADC值
        Value = *ADC_Value / 4096.0f;
    }
        break;
    case (Sampler_Filter_FOURIER):
    {
        // 傅里叶滤波：先设置输入值，然后计算，最后获取输出
        Filter_Fourier.Set_Now(*ADC_Value / 4096.0f);           // 设置当前ADC值（归一化）
        Filter_Fourier.TIM_Calculate_PeriodElapsedCallback();    // 执行滤波计算
        Value = Filter_Fourier.Get_Out();                        // 获取滤波结果
    }
        break;
    }
}
```

**作用**: 定时器中断回调函数，执行采样值的滤波处理。

## 3. 关键特性分析

### 3.1 模板设计

- **参数化滤波器阶数**: 通过模板参数`Filter_Fourier_Order`指定傅里叶滤波器的阶数
- **编译时优化**: 不同阶数的滤波器在编译时生成独立的代码

### 3.2 滤波器支持

- **无滤波模式**: 直接输出归一化的ADC值
- **傅里叶滤波模式**: 使用傅里叶变换进行信号滤波

### 3.3 数据归一化

- **ADC值范围**: 0~4095 (12位ADC)
- **输出范围**: 0.0~1.0 (归一化值)
- **计算公式**: `Value = *ADC_Value / 4096.0f`

## 4. 类的作用域和外设资源

### 4.1 作用域

- **公共作用域(public)**: 提供初始化、数据获取和定时器回调接口
- **保护作用域(protected)**: 内部实现细节，包括数据存储、滤波器实例等

### 4.2 使用的外设资源

- **ADC接口**: 用于模拟信号采样，支持ADC1-3
- **定时器**: 用于定期执行采样和滤波
- **内存资源**: 存储滤波器状态和采样数据
- **滤波算法**: 傅里叶滤波器算法库

### 4.3 工作流程

1. 初始化时绑定ADC端口和设置采样通道
2. 通过定时器周期性触发采样回调函数
3. 根据选择的滤波器类型处理ADC采样值
4. 用户通过Get_Value函数获取处理后的采样值

## 5. 代码结构详解

### 5.1 模板参数说明

```cpp
template<uint32_t Filter_Fourier_Order>
```

- `Filter_Fourier_Order`: 傅里叶滤波器的阶数，编译时确定
- 不同阶数生成不同的类实例，优化运行效率

### 5.2 滤波器选择机制

```cpp
switch (Sampler_Filter)
{
case (Sampler_Filter_NULL):      // 无滤波
case (Sampler_Filter_FOURIER):   // 傅里叶滤波
}
```

- 通过枚举值选择不同的滤波算法
- 支持运行时动态切换滤波器类型

### 5.3 数据指针优化

```cpp
ADC_Value = &ADC_Manage_Object->ADC_Data[Sampler_Serial];
```

- 使用指针直接访问ADC数据，提高访问效率
- 避免重复的数组索引计算

这个驱动程序提供了一个灵活的ADC采样器解决方案，支持多种滤波算法，适用于需要高质量模拟信号采集的应用场景。