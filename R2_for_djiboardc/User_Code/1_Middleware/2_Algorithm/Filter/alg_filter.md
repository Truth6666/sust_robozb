# 滤波器程序深度解析（小白友好版）
这份代码实现了**傅里叶滤波器（Fourier）** 和**卡尔曼滤波器（Kalman）** 两个核心滤波算法，用于对传感器或其他输入信号进行降噪、平滑处理。下面按「文件结构→基础概念→逐类逐函数解析→资源说明」的顺序拆解。

## 一、基础背景与文件结构
### 1. 文件分工
| 文件             | 作用                                                         |
| ---------------- | ------------------------------------------------------------ |
| `alg_filter.h`   | 头文件：定义滤波器的类、枚举、宏、函数声明（相当于“说明书”） |
| `alg_filter.cpp` | 源文件：实现头文件中声明的函数（相当于“具体执行的代码”）     |

### 2. 核心概念（小白版）
- **滤波器**：可以理解为“信号过滤器”，比如把抖动的传感器数据（如温度、速度）变得平滑，去掉噪声。
- **类（Class）**：代码的“模板”，封装了滤波器的变量和函数，方便重复使用（比如同时给温度、速度各做一个滤波器）。
- **模板类（template）**：傅里叶滤波器的“进阶模板”，可以自定义“阶数”（滤波强度），不用写多个重复代码。
- **回调函数（TIM_Calculate_PeriodElapsedCallback）**：定时器中断触发的函数，按固定频率执行滤波计算（比如每1ms算一次）。

## 二、头文件 `alg_filter.h` 逐段解析
### 1. 预处理指令（防重复包含）
```c
#ifndef ALG_FILTER_H
#define ALG_FILTER_H
// 中间是核心代码
#endif
```
- 作用：防止头文件被多次包含（比如多个文件引用`alg_filter.h`时，避免重复定义类/函数导致编译错误）。
- 小白理解：相当于给头文件加“锁”，只允许被编译一次。

### 2. 包含依赖
```c
#include "1_Middleware/1_Driver/Math/drv_math.h"
```
- 作用：引入数学工具函数（比如代码中用到的`Math_Sinc`（辛格函数）、`Math_Constrain`（限幅函数））。
- 外设关联：无直接外设，依赖自定义的数学驱动库。

### 3. 宏定义（常量）
```c
#define FOURIER_FILTER_DEFAULT_SAMPLING_FREQUENCY (1000.0f)
```
- 含义：傅里叶滤波器的**默认采样频率**为1000Hz（即每秒采集1000次数据）。
- 作用：简化代码，后续改采样频率只需改这个宏，不用逐行改。

### 4. 枚举类型（滤波器类型）
```c
enum Enum_Filter_Fourier_Type
{
    Filter_Fourier_Type_LOWPASS = 0,  // 低通：保留低频信号，滤除高频噪声（比如平滑温度）
    Filter_Fourier_Type_HIGHPASS,     // 高通：保留高频信号，滤除低频干扰（比如检测快速变化的振动）
    Filter_Fourier_Type_BANDPASS,     // 带通：只保留某一频段的信号（比如只留50-100Hz的声音）
    Filter_Fourier_Type_BANDSTOP,     // 带阻：滤除某一频段的信号（比如滤除50Hz市电干扰）
};
```
- 作用：用“名字”代替数字，让代码更易读（比如写`Filter_Fourier_Type_LOWPASS`比写`0`更直观）。
- 作用域：全局（整个程序都能用）。

### 5. 傅里叶滤波器类（模板类）
```c
template<uint32_t Filter_Fourier_Order = 50>
class Class_Filter_Fourier
{
public:
    // 公有函数（外部可调用）
    void Init(...);
    inline float Get_Out();
    inline void Set_Now(float __Now);
    void TIM_Calculate_PeriodElapsedCallback();

protected:
    // 保护变量（仅类内部/子类可访问，外部不可直接改）
    // 初始化参数
    float Value_Constrain_Low;   // 输入最小值（限幅）
    float Value_Constrain_High;  // 输入最大值（限幅）
    Enum_Filter_Fourier_Type Filter_Fourier_Type; // 滤波器类型
    float Frequency_Low;         // 特征低频
    float Frequency_High;        // 特征高频
    float Sampling_Frequency;    // 采样频率

    // 内部计算变量
    float System_Function[Filter_Fourier_Order + 1]; // 卷积核（滤波的核心系数）
    float Input_Signal[Filter_Fourier_Order + 1];    // 存储历史输入信号
    uint8_t Signal_Flag = 0;                         // 信号数组的索引标记（循环存数据）
    float Out = 0;                                   // 滤波输出值
};
```
#### 类的核心说明
- **模板参数 `Filter_Fourier_Order = 50`**：滤波器“阶数”（默认50），阶数越高滤波越平滑，但计算量越大、延迟越高。
- **访问权限**：
  - `public`：外部可以调用的函数（比如初始化、设置输入、获取输出）；
  - `protected`：类内部用的变量（防止外部误修改，保证封装性）。
- **作用域**：全局（只要包含头文件，任何地方都能创建这个类的对象）。

#### 成员函数逐解析
##### (1) `Init` 初始化函数
```c
template<uint32_t Filter_Fourier_Order>
void Class_Filter_Fourier<Filter_Fourier_Order>::Init(
    float __Value_Constrain_Low, 
    float __Value_Constrain_High, 
    Enum_Filter_Fourier_Type __Filter_Fourier_Type, 
    float __Frequency_Low, 
    float __Frequency_High, 
    float __Sampling_Frequency
)
{
    // 1. 保存初始化参数到类的成员变量
    Value_Constrain_Low = __Value_Constrain_Low;
    Value_Constrain_High = __Value_Constrain_High;
    Filter_Fourier_Type = __Filter_Fourier_Type;
    Frequency_Low = __Frequency_Low;
    Frequency_High = __Frequency_High;
    Sampling_Frequency = __Sampling_Frequency;

    // 2. 计算滤波器的核心系数（系统函数）
    float system_function_sum = 0.0f;
    float omega_low = 2.0f * PI * Frequency_Low / Sampling_Frequency; // 低频角频率
    float omega_high = 2.0f * PI * Frequency_High / Sampling_Frequency; // 高频角频率

    // 3. 根据滤波器类型计算卷积核
    switch (Filter_Fourier_Type)
    {
    case Filter_Fourier_Type_LOWPASS: // 低通滤波
        for (int i = 0; i < Filter_Fourier_Order + 1; i++)
        {
            System_Function[i] = omega_low / PI * Math_Sinc((i - Filter_Fourier_Order / 2.0f) * omega_low);
        }
        break;
    case Filter_Fourier_Type_HIGHPASS: // 高通滤波
        for (int i = 0; i < Filter_Fourier_Order + 1; i++)
        {
            System_Function[i] = Math_Sinc((i - Filter_Fourier_Order / 2.0f) * PI) - omega_high / PI * Math_Sinc((i - Filter_Fourier_Order / 2.0f) * omega_high);
        }
        break;
    case Filter_Fourier_Type_BANDPASS: // 带通滤波
        for (int i = 0; i < Filter_Fourier_Order + 1; i++)
        {
            System_Function[i] = omega_high / PI * Math_Sinc(...) - omega_low / PI * Math_Sinc(...);
        }
        break;
    case Filter_Fourier_Type_BANDSTOP: // 带阻滤波
        for (int i = 0; i < Filter_Fourier_Order + 1; i++)
        {
            System_Function[i] = Math_Sinc(...) + omega_low / PI * Math_Sinc(...) - omega_high / PI * Math_Sinc(...);
        }
        break;
    }

    // 4. 归一化：让所有系数之和为1（保证滤波后信号幅值不变）
    for (int i = 0; i < Filter_Fourier_Order + 1; i++)
    {
        system_function_sum += System_Function[i];
    }
    for (int i = 0; i < Filter_Fourier_Order + 1; i++)
    {
        System_Function[i] /= system_function_sum;
    }
}
```
- **作用**：初始化滤波器的参数，计算滤波的核心系数（卷积核）。
- 关键步骤：
  ① 保存外部传入的参数（限幅、类型、频率）；
  ② 把频率转换为“角频率”（数学计算需要）；
  ③ 根据滤波器类型，用`Math_Sinc`（辛格函数）计算每个系数；
  ④ 归一化系数（避免滤波后信号放大/缩小）。

##### (2) `Get_Out` 获取输出值
```c
template<uint32_t Filter_Fourier_Order>
inline float Class_Filter_Fourier<Filter_Fourier_Order>::Get_Out()
{
    return (Out);
}
```
- **关键字 `inline`**：内联函数，编译器会把函数代码直接嵌入调用处，减少函数调用的开销（适合简单函数）。
- **作用**：返回滤波后的输出值（外部代码通过这个函数获取结果）。

##### (3) `Set_Now` 设置当前输入值
```c
template<uint32_t Filter_Fourier_Order>
inline void Class_Filter_Fourier<Filter_Fourier_Order>::Set_Now(float __Now)
{
    // 1. 输入限幅：如果设置了上下限，把输入值限制在范围内
    if (Value_Constrain_Low != 0.0f || Value_Constrain_High != 0.0f)
    {
        Math_Constrain(&__Now, Value_Constrain_Low, Value_Constrain_High);
    }

    // 2. 把当前值存入输入信号数组（循环覆盖，类似“滑动窗口”）
    Input_Signal[Signal_Flag] = __Now;
    Signal_Flag++;

    // 3. 索引越界时重置（比如阶数50，索引到51就回到0）
    if (Signal_Flag == Filter_Fourier_Order + 1)
    {
        Signal_Flag = 0;
    }
}
```
- **作用**：接收外部的原始输入数据（比如传感器读数），限幅后存入数组，为后续卷积计算做准备。
- 核心逻辑：用`Signal_Flag`标记当前存储位置，数组存满后循环覆盖（实现“滑动窗口”，只保留最近的N个数据）。

##### (4) `TIM_Calculate_PeriodElapsedCallback` 定时器回调函数
```c
template<uint32_t Filter_Fourier_Order>
void Class_Filter_Fourier<Filter_Fourier_Order>::TIM_Calculate_PeriodElapsedCallback()
{
    Out = 0.0f;

    // 执行卷积操作：核心滤波计算
    for (int i = 0; i < Filter_Fourier_Order + 1; i++)
    {
        Out += System_Function[i] * Input_Signal[(Signal_Flag + i) % (Filter_Fourier_Order + 1)];
    }
}
```
- **作用**：按定时器周期执行（比如1ms一次），计算滤波输出（核心卷积操作）。
- 卷积逻辑：把“输入信号数组”和“系统函数数组”逐元素相乘后求和，得到平滑后的输出值。
- 取模运算 `%`：保证数组索引不越界，实现循环取历史数据。

### 6. 卡尔曼滤波器类
```c
class Class_Filter_Kalman
{
public:
    void Init(float __Error_Measure = 1.0f, float __Now = 0.0f, float __Error_Estimate = 1.0f);
    inline float Get_Out();
    inline void Set_Now(float __Now);
    void TIM_Calculate_PeriodElapsedCallback();

protected:
    // 初始化参数
    float Error_Measure; // 测量误差（传感器的固有误差，比如温度传感器误差±0.5℃）

    // 内部变量
    float Error_Estimate = 1.0f; // 估计误差（滤波后的误差）
    float Kalman_Gain = 0.0f;    // 卡尔曼增益（核心系数，决定信任测量值还是估计值）

    // 输出值
    float Out = 0.0f; // 滤波输出

    // 当前输入值
    float Now = 0.0f; // 外部传入的原始测量值
};
```
#### 成员函数逐解析
##### (1) `Init` 初始化函数（实现在`.cpp`文件）
```c
void Class_Filter_Kalman::Init(float __Error_Measure, float __Now, float __Error_Estimate)
{
    Error_Measure = __Error_Measure; // 保存测量误差
    Now = __Now;                     // 初始测量值
    Error_Estimate = __Error_Estimate; // 初始估计误差
}
```
- **作用**：初始化卡尔曼滤波器的核心参数（测量误差、初始值、估计误差）。
- 关键参数：
  - `Error_Measure`：传感器的误差（比如GPS的位置误差），需要根据硬件实际情况设置；
  - `Error_Estimate`：初始估计误差（一般设1.0即可）。

##### (2) `Get_Out` / `Set_Now` （和傅里叶滤波器逻辑一致）
```c
inline float Class_Filter_Kalman::Get_Out()
{
    return (Out);
}

inline void Class_Filter_Kalman::Set_Now(float __Now)
{
    Now = __Now;
}
```
- `Get_Out`：返回卡尔曼滤波后的输出值；
- `Set_Now`：接收外部的原始测量值（无协限幅，卡尔曼本身会自适应调整）。

##### (3) `TIM_Calculate_PeriodElapsedCallback` 定时器回调函数（实现在`.cpp`）
```c
void Class_Filter_Kalman::TIM_Calculate_PeriodElapsedCallback()
{
    // 1. 计算卡尔曼增益：决定“信任测量值”还是“信任估计值”
    Kalman_Gain = Error_Estimate / (Error_Estimate + Error_Measure);

    // 2. 更新估计值（核心公式）：用增益调整测量值和历史估计值
    Out = Out + Kalman_Gain * (Now - Out);

    // 3. 更新估计误差：迭代优化误差
    Error_Estimate = (1.0f - Kalman_Gain) * Error_Estimate;
}
```
- **核心逻辑**：
  ① 增益越大，越信任“当前测量值”；增益越小，越信任“历史估计值”；
  ② 用增益融合测量值和历史值，得到更平滑的估计值；
  ③ 迭代更新误差，让滤波效果越来越优。

## 三、源文件 `alg_filter.cpp` 解析
该文件仅实现了卡尔曼滤波器的两个函数（傅里叶滤波器的函数是模板函数，必须在头文件实现）：
1. `Class_Filter_Kalman::Init`：初始化卡尔曼参数；
2. `Class_Filter_Kalman::TIM_Calculate_PeriodElapsedCallback`：卡尔曼滤波的核心计算。

## 四、类的作用域与外设资源说明
### 1. 类的作用域
| 类                     | 作用域                                            | 核心用途                                                     |
| ---------------------- | ------------------------------------------------- | ------------------------------------------------------------ |
| `Class_Filter_Fourier` | 全局（public成员可外部调用，protected仅内部访问） | 基于傅里叶变换的滤波，支持低通/高通/带通/带阻，适合固定频率的噪声滤除 |
| `Class_Filter_Kalman`  | 全局（同上）                                      | 基于概率的自适应滤波，适合动态变化的信号（比如无人机定位、机器人导航） |

### 2. 外设资源依赖
| 资源类型      | 是否使用 | 说明                                                         |
| ------------- | -------- | ------------------------------------------------------------ |
| 定时器（TIM） | 间接使用 | 回调函数`TIM_Calculate_PeriodElapsedCallback`需要定时器中断触发（比如TIM1/TIM2），但代码中未直接操作定时器寄存器，只是约定“定时器周期到了调用这个函数” |
| 传感器        | 间接依赖 | 输入值`Now`通常来自传感器（如ADC、IMU、GPS），但代码中仅接收浮点值，不直接操作传感器外设 |
| 数学库        | 直接使用 | 依赖`drv_math.h`中的`Math_Sinc`（辛格函数）、`Math_Constrain`（限幅函数） |
| 内存          | 直接使用 | 类的成员变量（如`Input_Signal`数组）占用RAM，阶数越高占用越大（比如50阶傅里叶滤波器需要51个float，约204字节） |

## 五、小白实操总结（怎么用？）
以卡尔曼滤波器为例，使用步骤：
```c
// 1. 创建卡尔曼滤波器对象
Class_Filter_Kalman temp_filter;

// 2. 初始化：测量误差0.5，初始值25.0，估计误差1.0
temp_filter.Init(0.5f, 25.0f, 1.0f);

// 3. 主循环/传感器中断中设置当前值（比如读取温度传感器值）
float temp = Get_Temp_Sensor_Value(); // 假设这是读取传感器的函数
temp_filter.Set_Now(temp);

// 4. 定时器中断中调用回调函数（比如1ms中断）
void TIM2_IRQHandler()
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        temp_filter.TIM_Calculate_PeriodElapsedCallback();
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}

// 5. 任何地方获取滤波后的值
float filtered_temp = temp_filter.Get_Out();
```

傅里叶滤波器使用类似，只是创建对象时需要指定阶数：
```c
// 创建50阶低通傅里叶滤波器
Class_Filter_Fourier<50> speed_filter;
// 初始化：限幅0-100，低通，低频0，高频50，采样频率1000Hz
speed_filter.Init(0.0f, 100.0f, Filter_Fourier_Type_LOWPASS, 0.0f, 50.0f, 1000.0f);
```

## 六、核心区别（傅里叶 vs 卡尔曼）
| 特性     | 傅里叶滤波器                       | 卡尔曼滤波器                                |
| -------- | ---------------------------------- | ------------------------------------------- |
| 适用场景 | 固定频率噪声（如50Hz市电干扰）     | 动态变化的随机噪声（如GPS定位、机器人速度） |
| 计算量   | 阶数越高计算量越大（循环相乘求和） | 计算量小（仅几次加减乘除）                  |
| 灵活性   | 需提前知道噪声频率                 | 自适应调整，无需提前知道噪声特征            |
| 延迟     | 阶数越高延迟越大                   | 几乎无延迟                                  |

希望这份解析能帮你理解代码的每一个细节！如果有具体的代码使用问题（比如怎么改参数、怎么调试），可以继续问～