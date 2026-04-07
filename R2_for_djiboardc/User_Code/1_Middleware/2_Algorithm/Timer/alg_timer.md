# 定时器类程序深度解析（小白友好版）
这份代码实现了一个**可复用的毫秒级定时器类**，用于嵌入式系统中延时触发、状态管理（比如延时改变设备状态、超时判断等）。下面分模块、逐行解析，同时说明核心概念。

## 一、整体文件结构说明
| 文件            | 作用                                                   |
| --------------- | ------------------------------------------------------ |
| `alg_timer.h`   | 头文件，定义定时器的枚举、类结构、成员函数声明（接口） |
| `alg_timer.cpp` | 源文件，实现头文件中声明的类成员函数（具体逻辑）       |

嵌入式开发中，头文件负责“声明”（告诉编译器有什么），源文件负责“实现”（告诉编译器具体怎么做），这样的拆分便于代码复用和维护。

## 二、头文件 `alg_timer.h` 解析
### 1. 版权/注释头（1-13行）
```c
/**
 * @file alg_timer.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief 定时器, 用于延时改变状态等
 * @version 0.1
 * @date 2024-06-02 0.1 新建文件
 *
 * @copyright USTC-RoboWalker (c) 2024
 *
 */
```
- 这是**Doxygen风格注释**，用于自动生成文档，核心说明：
  - 文件功能：定时器，用于延时改变状态；
  - 版本/日期：2024-06-02 版本0.1；
  - 版权归属：USTC-RoboWalker团队。

### 2. 防止头文件重复包含（15-16行）
```c
#ifndef ALG_TIMER_H
#define ALG_TIMER_H
```
- 嵌入式开发必备的“头文件保护宏”：
  - `#ifndef`：如果 `ALG_TIMER_H` 未定义，则执行后续代码；
  - `#define`：定义 `ALG_TIMER_H`，确保后续重复包含该头文件时，不会重复编译（避免变量/类重复定义错误）；
  - 结尾的 `#endif`（112行）对应这里。

### 3. 包含依赖文件（19行）
```c
#include "1_Middleware/1_Driver/Math/drv_math.h"
```
- 引入数学相关的驱动文件（当前代码中暂未直接用到，可能是预留依赖）；
- 嵌入式开发中，`#include` 用于引入其他文件的声明（比如函数、宏、类型）。

### 4. 枚举：定时器状态（28-34行）
```c
enum Enum_Timer_Status
{
    Timer_Status_RESET = 0,    // 重置状态（初始/未启动）
    Timer_Status_WAIT,         // 等待状态（计时中）
    Timer_Status_TRIGGER,      // 触发状态（刚好到达延时时间）
    Timer_Status_TIMEOUT,      // 超时状态（超过延时时间）
};
```
- `enum`（枚举）：定义一组有名字的常量，替代魔法数字（比如用`Timer_Status_WAIT`代替1，可读性更高）；
- 状态说明：
  - `RESET`：定时器未工作，Tick（计数）和Delay（延时）都为0；
  - `WAIT`：正在计时，Tick < Delay；
  - `TRIGGER`：刚好到延时时间（Tick == Delay），仅触发一次；
  - `TIMEOUT`：计时超过延时时间（Tick > Delay）。

### 5. 核心类：Class_Timer（37-83行）
```c
class Class_Timer
{
public:
    void Init(uint32_t __Delay);                  // 初始化函数
    inline uint32_t Get_Tick();                   // 获取当前计数值（内联函数）
    inline Enum_Timer_Status Get_Now_Status();    // 获取当前状态（内联函数）
    inline void Set_Delay(uint32_t __Delay);      // 设置延时时间（内联函数）
    void TIM_1ms_Calculate_PeriodElapsedCallback(); // 1ms周期回调函数（核心计时逻辑）

protected:
    // 时钟计数（累计的毫秒数）
    uint32_t Tick = 0;
    // 延迟时间（需要延时的毫秒数）
    uint32_t Delay = 0;
    // 当前状态（枚举类型）
    Enum_Timer_Status Now_Status = Timer_Status_RESET;
};
```
#### （1）类的基础概念
- `class`：C++的类，是“数据+函数”的封装体，这里把定时器的**数据（Tick/Delay/状态）** 和**操作函数（初始化/计时/获取状态）** 封装在一起，实现“复用”（可以创建多个定时器对象，互不干扰）；
- 作用域：
  - `public`：公有的函数/变量，外部可以直接调用（比如`timer.Init(1000)`）；
  - `protected`：受保护的变量，仅类内部/子类可以访问，外部不能直接修改（比如不能直接写`timer.Tick = 100`，保证数据安全）。

#### （2）成员变量说明（protected）
| 变量名       | 类型                | 初始值               | 作用                                              |
| ------------ | ------------------- | -------------------- | ------------------------------------------------- |
| `Tick`       | `uint32_t`          | 0                    | 定时器累计计数（单位：ms），每1ms加1              |
| `Delay`      | `uint32_t`          | 0                    | 目标延时时间（单位：ms），比如设置1000就是延时1秒 |
| `Now_Status` | `Enum_Timer_Status` | `Timer_Status_RESET` | 定时器当前状态，默认重置                          |

#### （3）成员函数声明（public）
| 函数名                                    | 关键字   | 作用                                      |
| ----------------------------------------- | -------- | ----------------------------------------- |
| `Init`                                    | -        | 初始化延时时间（给Delay赋值）             |
| `Get_Tick`                                | `inline` | 获取当前Tick值（内联函数，执行更快）      |
| `Get_Now_Status`                          | `inline` | 获取当前定时器状态（内联函数）            |
| `Set_Delay`                               | `inline` | 重新设置延时时间，同时重置Tick和状态      |
| `TIM_1ms_Calculate_PeriodElapsedCallback` | -        | 1ms周期的回调函数（核心：更新Tick和状态） |

### 6. 内联函数实现（86-110行）
```c
inline uint32_t Class_Timer::Get_Tick()
{
    return (Tick);
}

inline Enum_Timer_Status Class_Timer::Get_Now_Status()
{
    return (Now_Status);
}

inline void Class_Timer::Set_Delay(uint32_t __Delay)
{
    Delay = __Delay;    // 赋值新的延时时间
    Tick = 0;           // 计数重置为0
    if (Delay != 0)
    {
        Now_Status = Timer_Status_WAIT; // 延时非0，进入等待状态
    }
    else
    {
        Now_Status = Timer_Status_RESET; // 延时为0，重置状态
    }
}
```
- `inline`（内联函数）：编译器会把内联函数的代码直接“嵌入”到调用处，没有函数调用的开销，适合简单、频繁调用的函数（比如获取状态/计数值）；
- 作用域解析符 `::`：`Class_Timer::Get_Tick()` 表示`Get_Tick`是`Class_Timer`类的成员函数；
- `Set_Delay` 逻辑：
  - 传入新的延时时间`__Delay`，覆盖`Delay`；
  - 计数`Tick`清零（重新开始计时）；
  - 根据延时是否为0，设置初始状态（非0则等待，0则重置）。

### 7. 头文件结束（112行）
```c
#endif
```
- 对应开头的`#ifndef ALG_TIMER_H`，结束头文件保护。

## 三、源文件 `alg_timer.cpp` 解析
### 1. 注释头+依赖包含（1-19行）
```c
/**
 * @file alg_timer.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief 斜坡函数, 用于速度规划等（注：注释笔误，实际是定时器实现）
 * @version 0.1
 * @date 2024-06-02 0.1 新建文件
 *
 * @copyright USTC-RoboWalker (c) 2024
 *
 */

#include "alg_timer.h"
```
- 注释中“斜坡函数”是笔误，实际是定时器的实现；
- `#include "alg_timer.h"`：引入头文件的类/函数声明，才能在源文件中实现。

### 2. 初始化函数实现（30-36行）
```c
void Class_Timer::Init(uint32_t __Delay)
{
    Delay = __Delay;
}
```
- 作用：初始化定时器的延时时间`Delay`；
- 对比`Set_Delay`：`Init`仅赋值`Delay`，不重置`Tick`和状态；`Set_Delay`会重置`Tick`并更新状态（更常用）。

### 3. 核心计时函数（39-55行）
```c
void Class_Timer::TIM_1ms_Calculate_PeriodElapsedCallback()
{
    Tick++;  // 每调用一次，计数+1（因为是1ms回调，所以Tick单位是ms）

    if (Delay == 0)
    {
        Now_Status = Timer_Status_RESET; // 延时为0，重置
    }
    else if (Tick < Delay)
    {
        Now_Status = Timer_Status_WAIT;  // 计数 < 延时，等待中
    }
    else if (Tick == Delay)
    {
        Now_Status = Timer_Status_TRIGGER; // 计数 == 延时，触发
    }
    else if (Tick > Delay)
    {
        Now_Status = Timer_Status_TIMEOUT; // 计数 > 延时，超时
    }
}
```
- 函数名含义：`TIM_1ms_Calculate_PeriodElapsedCallback` → “定时器1ms周期结束后的回调函数”；
- 核心逻辑：
  1. 每次调用，`Tick`加1（必须保证这个函数**每1ms被调用一次**，由硬件定时器中断触发）；
  2. 根据`Tick`和`Delay`的关系，更新定时器状态：
     - 延时为0 → 重置；
     - 计数没到延时 → 等待；
     - 计数刚好到延时 → 触发（仅一次）；
     - 计数超过延时 → 超时。

## 四、关键概念补充
### 1. 类的作用域总结
| 作用域      | 可访问范围           | 本类中涉及的成员                                             |
| ----------- | -------------------- | ------------------------------------------------------------ |
| `public`    | 类外部、类内部、子类 | `Init`/`Get_Tick`/`Get_Now_Status`/`Set_Delay`/`TIM_1ms_Calculate_PeriodElapsedCallback` |
| `protected` | 类内部、子类         | `Tick`/`Delay`/`Now_Status`                                  |

### 2. 外设资源依赖
- 核心依赖：**硬件定时器（提供1ms中断）**；
- 逻辑：嵌入式系统中，需要配置一个硬件定时器（比如STM32的TIM2/TIM3），设置为1ms触发一次中断，在中断服务函数中调用`TIM_1ms_Calculate_PeriodElapsedCallback()`；
- 举例（伪代码）：
  ```c
  // 硬件定时器1ms中断服务函数
  void TIM2_IRQHandler(void)
  {
      if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
      {
          // 调用定时器对象的回调函数，更新计数和状态
          timer_obj.TIM_1ms_Calculate_PeriodElapsedCallback();
          TIM_ClearITPendingBit(TIM2, TIM_IT_Update); // 清除中断标志
      }
  }
  ```
- 无其他外设依赖：代码中仅用到计数和逻辑判断，不直接操作GPIO、串口等外设。

### 3. 典型使用流程（小白示例）
```c
// 1. 创建定时器对象
Class_Timer my_timer;

// 2. 设置延时时间（比如延时1000ms=1秒）
my_timer.Set_Delay(1000);

// 3. 硬件定时器每1ms调用一次 my_timer.TIM_1ms_Calculate_PeriodElapsedCallback()

// 4. 业务逻辑中判断状态
while(1)
{
    if (my_timer.Get_Now_Status() == Timer_Status_TRIGGER)
    {
        // 延时1秒到达，执行操作（比如点亮LED）
        LED_ON();
    }
    else if (my_timer.Get_Now_Status() == Timer_Status_TIMEOUT)
    {
        // 超时（超过1秒），执行操作（比如熄灭LED）
        LED_OFF();
    }
}
```

## 五、核心功能总结
| 函数/变量                                 | 核心作用                                  |
| ----------------------------------------- | ----------------------------------------- |
| `Init`                                    | 初始化延时时间（仅赋值，不重置计数）      |
| `Set_Delay`                               | 重置延时时间+计数+初始状态（常用）        |
| `Get_Tick`                                | 获取当前累计计时（ms）                    |
| `Get_Now_Status`                          | 获取定时器当前状态（重置/等待/触发/超时） |
| `TIM_1ms_Calculate_PeriodElapsedCallback` | 1ms周期更新计数和状态（核心）             |
| `Tick`                                    | 累计计时数（ms）                          |
| `Delay`                                   | 目标延时数（ms）                          |
| `Enum_Timer_Status`                       | 标准化定时器状态，避免魔法数字            |

这份代码的核心价值是**封装了通用的毫秒级计时逻辑**，可以在嵌入式项目中快速实现“延时触发”“超时判断”等功能，比如机器人动作延时、传感器超时检测等。