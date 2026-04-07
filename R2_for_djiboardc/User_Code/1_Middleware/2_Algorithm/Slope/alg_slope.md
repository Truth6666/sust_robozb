# 斜坡函数程序深度解析（小白友好版）
本文将从**文件结构**、**数据类型**、**类与函数**、**逻辑流程**、**资源依赖**等维度，逐步解析这套斜坡函数代码，全程用通俗的语言讲解，避免专业术语堆砌。

## 一、整体背景与核心作用
这套代码实现了**斜坡函数（Slope Function）**，核心用途是让一个数值（比如电机速度、机器人运动指令）按照设定的“加速/减速幅度”逐步逼近目标值，而不是瞬间跳变。
- 应用场景：机器人速度规划、电机转速控制等（避免瞬间加减速导致的机械冲击/程序震荡）。
- 核心逻辑：每次计算时，数值只能增加/减少设定的最大幅度，直到达到目标值。

## 二、文件结构与基础语法
代码分为两个文件：
| 文件名称        | 作用                                                         |
| --------------- | ------------------------------------------------------------ |
| `alg_slope.h`   | 头文件：定义枚举、类的结构、函数声明（相当于“说明书”，告诉编译器有哪些东西） |
| `alg_slope.cpp` | 源文件：实现头文件中声明的函数（相当于“具体操作步骤”）       |

### 2.1 头文件保护宏（alg_slope.h 开头）
```c
#ifndef ALG_SLOPE_H
#define ALG_SLOPE_H
// 中间是头文件内容
#endif
```
- 作用：防止头文件被重复包含（比如多个文件引用该头文件时，避免重复定义类/函数，导致编译错误）。
- 小白理解：相当于给头文件加“锁”，只允许被编译一次。

### 2.2 头文件包含（Includes）
#### alg_slope.h 中：
```c
#include "1_Middleware/1_Driver/Math/drv_math.h"
```
- 作用：引入数学工具函数（比如代码中用到的`Math_Abs`绝对值函数）。
- 外设/资源依赖：无硬件外设，仅依赖自定义的数学驱动库`drv_math.h`。

#### alg_slope.cpp 中：
```c
#include "alg_slope.h"
```
- 作用：引入头文件中定义的类和函数声明，让源文件能“识别”头文件的结构。

## 三、枚举类型定义（alg_slope.h）
```c
/**
 * @brief 规划优先类型, 分为目标值优先和真实值优先
 * 目标值优先, 即硬规划
 * 真实值优先, 即当前真实值夹在当前规划值和目标值之间, 当前规划值转为当前真实值
 */
enum Enum_Slope_First
{
    Slope_First_REAL = 0,    // 真实值优先（默认）
    Slope_First_TARGET,      // 目标值优先
};
```
- 作用：定义“规划优先级”的可选类型，用枚举让代码更易读（避免用魔法数字0/1）。
- 通俗解释：
  - `Slope_First_REAL`：优先以“实际反馈值”为准（比如电机实际转速）；
  - `Slope_First_TARGET`：优先向“目标值”逼近（不管实际值）。

## 四、类的定义（Class_Slope）
### 4.1 类的核心作用
`Class_Slope`是“斜坡函数”的封装类，把**变量**和**操作函数**打包在一起，方便复用（比如可以创建多个斜坡函数实例，分别控制电机1、电机2的速度）。

### 4.2 类的作用域（访问权限）
类内分为`public`、`protected`两个作用域：
| 作用域      | 访问权限                    | 包含内容                                                     |
| ----------- | --------------------------- | ------------------------------------------------------------ |
| `public`    | 外部可直接访问（调用/修改） | 初始化函数、回调函数、内联读写函数（Set/Get）                |
| `protected` | 仅类内部/子类可访问         | 所有变量（输出值、规划值、真实值、增减幅度、目标值）、枚举类型变量 |

### 4.3 类内变量解析（protected 区域）
| 变量名           | 类型  | 初始值 | 通俗解释                                               |
| ---------------- | ----- | ------ | ------------------------------------------------------ |
| `Out`            | float | 0.0f   | 斜坡函数的最终输出值（每次计算后更新）                 |
| `Slope_First`    | 枚举  | REAL   | 规划优先级（默认真实值优先）                           |
| `Now_Planning`   | float | 0.0f   | 当前规划值（上一次计算后的输出值，作为本次计算的基础） |
| `Now_Real`       | float | 0.0f   | 当前真实值（比如电机实际转速、传感器反馈值，外部传入） |
| `Increase_Value` | float | 0.0f   | 单次计算的最大增幅（比如每次最多加速10rpm）            |
| `Decrease_Value` | float | 0.0f   | 单次计算的最大减幅（比如每次最多减速20rpm）            |
| `Target`         | float | 0.0f   | 目标值（最终要达到的数值，比如想让电机转到100rpm）     |

### 4.4 类内函数声明与实现
#### （1）初始化函数（Init）
##### 声明（alg_slope.h）：
```c
void Init(float __Increase_Value, float __Decrease_Value, Enum_Slope_First __Slope_First = Slope_First_REAL);
```
##### 实现（alg_slope.cpp）：
```c
void Class_Slope::Init(float __Increase_Value, float __Decrease_Value, Enum_Slope_First __Slope_First)
{
    Increase_Value = __Increase_Value;
    Decrease_Value = __Decrease_Value;
    Slope_First = __Slope_First;
}
```
- 作用：初始化斜坡函数的核心参数（最大增幅、最大减幅、规划优先级）。
- 参数说明：
  - `__Increase_Value`：传入的单次最大增幅；
  - `__Decrease_Value`：传入的单次最大减幅；
  - `__Slope_First`：规划优先级（默认真实值优先，可选填）。
- 调用示例（小白友好）：
  ```c
  Class_Slope Speed_Slope; // 创建一个斜坡函数实例（控制速度）
  Speed_Slope.Init(1.0f, 2.0f); // 单次最大增幅1，减幅2，默认真实值优先
  ```

#### （2）周期计算回调函数（TIM_Calculate_PeriodElapsedCallback）
##### 声明（alg_slope.h）：
```c
void TIM_Calculate_PeriodElapsedCallback();
```
##### 实现（alg_slope.cpp）：核心逻辑！
作用：**每次调用该函数，就执行一次斜坡计算**（计算周期由调用者决定，比如定时器中断每10ms调用一次）。
拆解逻辑（分步骤）：

###### 步骤1：真实值优先的额外处理
```c
if (Slope_First == Slope_First_REAL)
{
    if ((Target >= Now_Real && Now_Real >= Now_Planning) || (Target <= Now_Real && Now_Real <= Now_Planning))
    {
        Out = Now_Real;
    }
}
```
- 通俗解释：如果设置“真实值优先”，且“真实值夹在规划值和目标值之间”（比如规划值10、真实值15、目标值20），则直接把输出值设为真实值（避免规划值和真实值脱节）。

###### 步骤2：分情况计算输出值（核心）
代码把`Now_Planning`（当前规划值）分为**正数、负数、0** 三种情况，核心逻辑是：
> 如果目标值和当前规划值的差值 > 单次最大幅，则按最大幅增减；否则直接设为目标值。

- 子情况1：当前规划值为正数（Now_Planning > 0）
  ```c
  if (Target > Now_Planning)
  {
      // 正值加速：目标值 > 当前值，要加速
      if (Math_Abs(Now_Planning - Target) > Increase_Value)
      {
          Out += Increase_Value; // 没到最大幅，加增幅
      }
      else
      {
          Out = Target; // 差值小于最大幅，直接到目标值
      }
  }
  else if (Target < Now_Planning)
  {
      // 正值减速：目标值 < 当前值，要减速
      if (Math_Abs(Now_Planning - Target) > Decrease_Value)
      {
          Out -= Decrease_Value; // 没到最大幅，减减幅
      }
      else
      {
          Out = Target; // 差值小于最大幅，直接到目标值
      }
  }
  ```

- 子情况2：当前规划值为负数（Now_Planning < 0）
  ```c
  if (Target < Now_Planning)
  {
      // 负值加速：目标值 < 当前值（比如当前-10，目标-20），要“更负”（加速）
      if (Math_Abs(Now_Planning - Target) > Increase_Value)
      {
          Out -= Increase_Value; // 比如-10 -1 = -11
      }
      else
      {
          Out = Target;
      }
  }
  else if (Target > Now_Planning)
  {
      // 负值减速：目标值 > 当前值（比如当前-20，目标-10），要“没那么负”（减速）
      if (Math_Abs(Now_Planning - Target) > Decrease_Value)
      {
          Out += Decrease_Value; // 比如-20 +2 = -18
      }
      else
      {
          Out = Target;
      }
  }
  ```

- 子情况3：当前规划值为0（Now_Planning == 0）
  ```c
  if (Target > Now_Planning)
  {
      // 0→正：加速到目标值
      if (Math_Abs(Now_Planning - Target) > Increase_Value)
      {
          Out += Increase_Value;
      }
      else
      {
          Out = Target;
      }
  }
  else if (Target < Now_Planning)
  {
      // 0→负：加速到目标值
      if (Math_Abs(Now_Planning - Target) > Increase_Value)
      {
          Out -= Increase_Value;
      }
      else
      {
          Out = Target;
      }
  }
  ```

###### 步骤3：善后工作
```c
Now_Planning = Out;
```
- 作用：把本次计算的输出值，赋值给“当前规划值”，作为下一次计算的基础。

#### （3）内联读写函数（Set/Get）
这类函数用`inline`修饰（内联函数），作用是**快速读写类内的protected变量**（因为protected变量外部不能直接访问，需要通过函数中转）。

| 函数名                 | 作用                             | 示例调用                                                |
| ---------------------- | -------------------------------- | ------------------------------------------------------- |
| `Get_Out()`            | 获取斜坡函数的输出值`Out`        | `float speed = Speed_Slope.Get_Out();`                  |
| `Set_Now_Real()`       | 设置当前真实值`Now_Real`         | `Speed_Slope.Set_Now_Real(15.0f);`（传入电机实际转速）  |
| `Set_Increase_Value()` | 修改单次最大增幅`Increase_Value` | `Speed_Slope.Set_Increase_Value(1.5f);`                 |
| `Set_Decrease_Value()` | 修改单次最大减幅`Decrease_Value` | `Speed_Slope.Set_Decrease_Value(3.0f);`                 |
| `Set_Target()`         | 设置目标值`Target`               | `Speed_Slope.Set_Target(100.0f);`（想让电机转到100rpm） |

以`Get_Out()`为例，实现代码：
```c
inline float Class_Slope::Get_Out()
{
    return (Out);
}
```
- `inline`作用：编译器会把函数代码直接“嵌入”调用处，减少函数调用的开销（适合简单函数）。

## 五、外设/资源依赖总结
| 依赖项                | 类型     | 作用                                                         | 是否硬件外设                                 |
| --------------------- | -------- | ------------------------------------------------------------ | -------------------------------------------- |
| `drv_math.h`          | 软件库   | 提供`Math_Abs`绝对值函数                                     | 否                                           |
| 定时器（隐含）        | 硬件外设 | 调用`TIM_Calculate_PeriodElapsedCallback`的周期（比如10ms调用一次） | 是（但代码中未直接操作定时器，仅约定函数名） |
| 传感器/执行器（隐含） | 硬件外设 | 提供`Now_Real`（比如电机编码器反馈转速）、使用`Out`（比如控制电机PWM） | 是（代码中仅接收/输出数值，不直接操作）      |

## 六、小白级使用流程示例
假设要控制电机转速，步骤如下：
1. **创建实例**：`Class_Slope Motor_Speed_Slope;`
2. **初始化**：`Motor_Speed_Slope.Init(1.0f, 2.0f);`（单次最大加1rpm，减2rpm）
3. **设置目标值**：`Motor_Speed_Slope.Set_Target(50.0f);`（目标转速50rpm）
4. **定时器中断（每10ms）**：
   ```c
   // 步骤1：获取电机实际转速（假设从编码器读取）
   float real_speed = Get_Motor_Real_Speed();
   // 步骤2：设置真实值到斜坡函数
   Motor_Speed_Slope.Set_Now_Real(real_speed);
   // 步骤3：执行斜坡计算
   Motor_Speed_Slope.TIM_Calculate_PeriodElapsedCallback();
   // 步骤4：获取输出值，控制电机
   float output_speed = Motor_Speed_Slope.Get_Out();
   Set_Motor_Speed(output_speed);
   ```

## 七、核心逻辑简化总结
1. 初始化：设定“每次最多加多少、减多少”；
2. 每次计算：
   - （可选）如果真实值在规划值和目标值之间，直接用真实值；
   - 否则，按“最大幅”逐步逼近目标值；
3. 输出值用于控制设备，真实值来自设备反馈，形成闭环。

这套代码的核心优势是**数值平滑变化**，避免了“瞬间跳变”导致的设备冲击或程序不稳定，是机器人/电机控制中非常基础且实用的算法。