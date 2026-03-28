# 有限自动机(FSM)程序深度解析
## 整体概述
这是一套基于C++实现的**可复用有限自动机(Finite State Machine, FSM)** 框架，主要用于有时间驱动需求的状态机场景（比如机器人控制、嵌入式系统的时序逻辑处理）。代码分为头文件(`alg_fsm.h`)和源文件(`alg_fsm.cpp`)，核心是`Class_FSM`类，封装了状态机的初始化、状态切换、时间计数等核心功能。

## 一、头文件 `alg_fsm.h` 解析
### 1. 头部保护与基础包含
```cpp
#ifndef ALG_FSM_H
#define ALG_FSM_H

/* Includes ------------------------------------------------------------------*/
#include "1_Middleware/1_Driver/Math/drv_math.h"

/* Exported macros -----------------------------------------------------------*/
#define STATUS_MAX (10)
```
- **头文件保护宏**：`#ifndef ALG_FSM_H` / `#define ALG_FSM_H` / `#endif`  
  防止头文件被重复包含（嵌入式开发中多文件编译时的基础规范）。
- **头文件包含**：`#include "1_Middleware/1_Driver/Math/drv_math.h"`  
  引入数学相关的驱动库（当前代码未直接使用，但为后续扩展预留）。
- **宏定义**：`#define STATUS_MAX (10)`  
  限定状态机最多支持10个状态，是状态数组的最大长度，可根据需求修改。

### 2. 枚举类型：状态阶段
```cpp
enum Enum_Status_Stage
{
    Status_Stage_DISABLE = 0,
    Status_Stage_ENABLE,
};
```
- 作用：定义状态的两种核心阶段（枚举值本质是整数，`DISABLE=0`，`ENABLE=1`）。
  - `Status_Stage_DISABLE`：状态**失能**（未激活）。
  - `Status_Stage_ENABLE`：状态**使能**（激活中）。
- 场景：标记每个状态当前是否处于工作状态，是状态机的核心状态标记。

### 3. 结构体：状态结构体
```cpp
struct Struct_Status
{
    Enum_Status_Stage Status_Stage;
    uint32_t Count_Time;
};
```
- 作用：封装单个状态的核心属性，每个状态都包含这两个参数：
  - `Status_Stage`：当前状态的阶段（使能/失能），类型是上面定义的枚举。
  - `Count_Time`：状态激活后的计时计数器（单位取决于定时器回调周期）。

### 4. 核心类：Class_FSM
#### 类的整体说明
- **作用**：可复用的有限自动机核心类，所有需要时间驱动状态切换的场景都可以继承这个类使用。
- **作用域说明**：
  - `public`：对外暴露的接口（初始化、状态读写、定时器回调）。
  - `protected`：仅子类可访问的内部变量（状态数量、当前状态编号等），防止外部误修改。
- **外设资源依赖**：
  - 核心依赖**定时器(TIM)** ：`TIM_Calculate_PeriodElapsedCallback`是定时器周期回调函数，需要硬件定时器触发（比如STM32的TIM定时器中断）。
  - 无其他直接外设依赖（GPIO、UART等），仅依赖定时器的时间基准。

#### 类成员变量（按作用域）
##### Public 成员
```cpp
Struct_Status Status[STATUS_MAX];
```
- 状态数组：存储最多10个状态的信息（每个元素是`Struct_Status`结构体），外部/子类可直接访问。

##### Protected 成员
```cpp
// 状态数量（初始化时设置）
uint8_t Status_Number;
// FSM当前状态编号（默认0）
uint8_t Now_Status_Serial = 0;
```
- `Status_Number`：实际使用的状态数量（比如只需要5个状态，就初始化该值为5，避免访问超出范围的数组）。
- `Now_Status_Serial`：当前激活的状态编号（比如0代表初始状态，1代表状态1，以此类推），仅子类可修改。

#### 类成员函数（按作用域）
##### Public 函数声明
```cpp
// 初始化函数
void Init(uint8_t __Status_Number, uint8_t __Now_Status_Serial = 0);
// 获取当前状态编号（内联函数）
inline uint8_t Get_Now_Status_Serial();
// 设置下一个状态（内联函数）
inline void Set_Status(uint8_t Next_Status_serial);
// 定时器周期回调函数
void TIM_Calculate_PeriodElapsedCallback();
```

##### 内联函数实现（类外但在头文件）
###### 1. 获取当前状态编号
```cpp
inline uint8_t Class_FSM::Get_Now_Status_Serial()
{
    return (Now_Status_Serial);
}
```
- **作用**：读取当前状态机的激活状态编号（对外提供只读接口，避免直接修改`Now_Status_Serial`）。
- **内联函数特性**：编译时直接嵌入调用处，无函数调用开销，适合简单的读操作。
- **使用场景**：外部代码需要判断当前状态时调用，比如：
  ```cpp
  if (fsm.Get_Now_Status_Serial() == 1) { /* 执行状态1的逻辑 */ }
  ```

###### 2. 设置下一个状态
```cpp
inline void Class_FSM::Set_Status(uint8_t Next_Status_serial)
{
    // 失能当前状态, 计数器清零
    Status[Now_Status_Serial].Status_Stage = Status_Stage_DISABLE;
    Status[Now_Status_Serial].Count_Time = 0;

    // 转到下一个状态
    Status[Next_Status_serial].Status_Stage = Status_Stage_ENABLE;
    Now_Status_Serial = Next_Status_serial;
}
```
- **核心作用**：状态切换的核心函数，完成3个关键操作：
  1. 把当前激活的状态标记为“失能”，并重置该状态的计时器。
  2. 把目标状态标记为“使能”。
  3. 更新`Now_Status_Serial`为新状态编号。
- **参数**：`Next_Status_serial`：要切换到的下一个状态编号（必须小于`Status_Number`）。
- **使用场景**：状态机需要切换状态时调用，比如“状态0运行5秒后切换到状态1”。

## 二、源文件 `alg_fsm.cpp` 解析
### 1. 头部注释与包含
```cpp
/**
 * @file alg_fsm.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief 有限自动机
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 * @copyright USTC-RoboWalker (c) 2023
 */

/* Includes ------------------------------------------------------------------*/
#include "alg_fsm.h"
```
- 注释：标准化的文件头注释，说明文件功能、作者、版本、版权等（嵌入式开发规范）。
- `#include "alg_fsm.h"`：包含头文件，关联类的声明与实现。

### 2. 初始化函数实现
```cpp
void Class_FSM::Init(uint8_t __Status_Number, uint8_t __Now_Status_Serial)
{
    Status_Number = __Status_Number;

    Now_Status_Serial = __Now_Status_Serial;

    // 所有状态全刷0
    for (int i = 0; i < Status_Number; i++)
    {
        Status[i].Status_Stage = Status_Stage_DISABLE;
        Status[i].Count_Time = 0;
    }

    // 使能初始状态
    Status[__Now_Status_Serial].Status_Stage = Status_Stage_ENABLE;
}
```
- **作用**：初始化状态机的核心参数，为状态机运行做准备。
- **参数**：
  - `__Status_Number`：状态机的总状态数量（比如5个状态就传5）。
  - `__Now_Status_Serial`：初始状态编号（默认0，可自定义）。
- **执行逻辑**：
  1. 把传入的状态数量和初始状态编号赋值给类的内部变量。
  2. 遍历所有状态，将每个状态标记为“失能”，计时器清零（初始状态统一复位）。
  3. 单独激活初始状态（标记为“使能”），让状态机从该状态开始运行。
- **使用场景**：状态机使用前必须调用，比如：
  ```cpp
  Class_FSM my_fsm;
  my_fsm.Init(5, 0); // 初始化5个状态，初始状态为0
  ```

### 3. 定时器回调函数实现
```cpp
void Class_FSM::TIM_Calculate_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Count_Time++;

    // 自己接着编写状态转移函数
}
```
- **核心作用**：定时器周期触发的回调函数（比如每10ms调用一次），实现状态的计时功能。
- **执行逻辑**：
  1. 对当前激活状态的`Count_Time`加1（计数单位 = 定时器回调周期）。
  2. 注释提示：用户需要根据实际需求，在这里编写状态转移的逻辑（比如“计数到100（1秒）后切换状态”）。
- **外设依赖**：必须绑定硬件定时器（如STM32的TIM1/TIM2等）的周期中断，否则无法触发计时。
- **扩展示例**（用户自定义逻辑）：
  ```cpp
  void Class_FSM::TIM_Calculate_PeriodElapsedCallback()
  {
      Status[Now_Status_Serial].Count_Time++;
  
      // 示例：状态0运行500ms（50次回调，假设10ms/次）后切换到状态1
      if (Now_Status_Serial == 0 && Status[0].Count_Time >= 50)
      {
          Set_Status(1);
      }
  }
  ```

## 三、关键概念补充
### 1. 类的作用域总结
| 作用域    | 成员/函数                                                    | 访问权限          | 用途                       |
| --------- | ------------------------------------------------------------ | ----------------- | -------------------------- |
| public    | Status数组、Init、Get_Now_Status_Serial、Set_Status、TIM_Calculate_PeriodElapsedCallback | 外部/子类均可访问 | 对外提供状态机操作接口     |
| protected | Status_Number、Now_Status_Serial                             | 仅子类可访问      | 内部核心变量，防止外部误改 |

### 2. 外设资源说明
- **核心外设**：定时器(TIM)  
  - 用途：触发`TIM_Calculate_PeriodElapsedCallback`函数，提供状态计时的时间基准。
  - 无其他外设依赖（GPIO、UART、SPI等均未使用）。
  - 适配性：可兼容任意带定时器的嵌入式MCU（STM32、ESP32、51单片机等），只需将该函数绑定到定时器中断即可。

### 3. 类的使用方式（小白友好版）
1. **继承类**：自定义状态机类继承`Class_FSM`（因为核心变量是protected，继承后可访问）。
   ```cpp
   class My_FSM : public Class_FSM
   {
   public:
       // 自定义状态转移逻辑
       void My_State_Logic()
       {
           if (Now_Status_Serial == 0 && Status[0].Count_Time >= 100)
           {
               Set_Status(1); // 状态0运行1秒后切状态1
           }
       }
   };
   ```
2. **初始化**：创建对象并初始化。
   ```cpp
   My_FSM my_fsm;
   my_fsm.Init(2, 0); // 2个状态，初始状态0
   ```
3. **绑定定时器**：将`TIM_Calculate_PeriodElapsedCallback`加入定时器中断函数（以STM32为例）。
   ```cpp
   void TIM2_IRQHandler(void)
   {
       if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
       {
           my_fsm.TIM_Calculate_PeriodElapsedCallback(); // 每10ms调用一次
           my_fsm.My_State_Logic(); // 执行自定义状态逻辑
           TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
       }
   }
   ```

## 四、核心函数总结表
| 函数名                              | 所在文件    | 作用域 | 核心作用                                                     |
| ----------------------------------- | ----------- | ------ | ------------------------------------------------------------ |
| Init                                | alg_fsm.cpp | public | 初始化状态机总数量、初始状态，复位所有状态为失能并激活初始状态 |
| Get_Now_Status_Serial               | alg_fsm.h   | public | 读取当前激活的状态编号（只读，安全）                         |
| Set_Status                          | alg_fsm.h   | public | 切换状态：失能当前状态（清零计时器），使能目标状态，更新当前状态编号 |
| TIM_Calculate_PeriodElapsedCallback | alg_fsm.cpp | public | 定时器回调函数，对当前状态计时+1，需用户补充状态转移逻辑     |

## 五、小白易懂的核心逻辑
这个程序的本质是：  
**给每个状态配一个“计时器”，通过定时器定期给当前激活的状态“计时”，当计时达到阈值时，调用`Set_Status`切换到下一个状态**。  
比如：
- 状态0（初始化）：计时1秒后→状态1（运行）。
- 状态1（运行）：计时10秒后→状态2（停止）。
所有状态的切换逻辑都可以基于“计时”实现，这也是嵌入式系统中最常用的时序控制方式。