# 静态内存循环队列程序深度解析
## 一、整体概述
该程序实现了一个**C++模板类**的静态内存循环队列（Class_Queue），用于在嵌入式系统等场景下高效管理固定大小的数据队列。静态内存意味着队列的存储空间在编译期确定，不依赖动态内存分配（如new/delete），更适合对内存稳定性要求高的嵌入式环境。

### 核心特点
1. 模板化设计：支持任意数据类型（int、float、自定义结构体等）。
2. 循环队列：利用取模运算实现队列的循环复用，避免内存碎片。
3. 静态内存：队列大小由模板参数指定，存储在栈/全局区，无动态分配风险。
4. 核心操作：初始化、入队、出队、获取队首/队尾、获取长度、清空。

## 二、代码分段解析
### 1. 头文件保护与基础包含（alg_queue.h 1-22行）
```cpp
#ifndef ALG_QUEUE_H
#define ALG_QUEUE_H

/* Includes ------------------------------------------------------------------*/

#include "1_Middleware/1_Driver/Math/drv_math.h"
```
- **头文件保护宏**：`#ifndef ALG_QUEUE_H` / `#define ALG_QUEUE_H` / `#endif`  
  防止头文件被重复包含（多次包含会导致类/函数重复定义编译错误）。
- **头文件包含**：`#include "drv_math.h"`  
  引入数学相关驱动，但本队列代码未直接使用其中功能，仅为工程统一依赖。

### 2. 类的定义（alg_queue.h 25-63行）
```cpp
template<typename Type, uint32_t Max_Size = 200>
class Class_Queue
{
public:
    void Init();

    inline uint32_t Get_Length();

    inline Type Get_Front();

    inline Type Get_Rear();

    inline void Push(Type __Data);

    inline Type Pop();

    inline void Clear();

protected:
    Type Queue[Max_Size];
    uint32_t Front = 0;
    uint32_t Rear = 0;
    uint32_t Length = 0;
};
```

#### （1）模板参数
- `template<typename Type, uint32_t Max_Size = 200>`  
  - `Type`：模板类型参数，支持任意数据类型（如`int`、`float`、`struct`）。  
  - `Max_Size`：模板非类型参数，指定队列最大容量，默认值200（编译期确定）。

#### （2）访问权限与成员划分
| 作用域      | 成员类型 | 说明                                             |
| ----------- | -------- | ------------------------------------------------ |
| `public`    | 成员函数 | 对外提供的队列操作接口，外部可直接调用           |
| `protected` | 成员变量 | 队列内部状态变量，外部不可直接访问（子类可访问） |

#### （3）protected成员变量（核心状态）
| 变量名            | 类型       | 作用                                           |
| ----------------- | ---------- | ---------------------------------------------- |
| `Queue[Max_Size]` | `Type`数组 | 队列的存储空间，存储实际数据，大小为`Max_Size` |
| `Front`           | `uint32_t` | 队首元素的索引（指向当前队首的位置）           |
| `Rear`            | `uint32_t` | 队尾元素的索引（指向当前队尾的位置）           |
| `Length`          | `uint32_t` | 队列当前的有效元素个数                         |

### 3. 成员函数实现（alg_queue.h 66-178行）
#### （1）初始化函数 `Init()`
```cpp
template<typename Type, uint32_t Max_Size>
void Class_Queue<Type, Max_Size>::Init()
{
    Front = 0;
    Rear = 0;
    Length = 0;
}
```
- **作用**：初始化队列状态，将队首、队尾索引置0，有效长度置0。
- **调用时机**：使用队列前必须调用（如系统启动时），避免初始值混乱。
- **inline**：无（非内联，因为初始化逻辑简单但无需强制内联）。

#### （2）获取队列长度 `Get_Length()`
```cpp
template<typename Type, uint32_t Max_Size>
inline uint32_t Class_Queue<Type, Max_Size>::Get_Length()
{
    return (Length);
}
```
- **作用**：返回队列当前的有效元素个数。
- **inline**：内联函数（编译时直接展开，无函数调用开销，适合简单操作）。
- **返回值**：`uint32_t`类型的`Length`（0 ~ Max_Size）。

#### （3）获取队首元素 `Get_Front()`
```cpp
template<typename Type, uint32_t Max_Size>
inline Type Class_Queue<Type, Max_Size>::Get_Front()
{
    return (Queue[Front]);
}
```
- **作用**：读取队首元素（不弹出，仅查看）。
- **注意**：若队列为空（Length=0），返回的是`Queue[0]`的默认值（无边界检查，需外部保证队列非空）。
- **返回值**：队首位置的`Type`类型数据。

#### （4）获取队尾元素 `Get_Rear()`
```cpp
template<typename Type, uint32_t Max_Size>
inline Type Class_Queue<Type, Max_Size>::Get_Rear()
{
    return (Queue[Rear]);
}
```
- **作用**：读取队尾元素（不弹出，仅查看）。
- **注意**：若队列为空，返回`Queue[0]`默认值（需外部保证队列非空）。
- **返回值**：队尾位置的`Type`类型数据。

#### （5）入队函数 `Push()`
```cpp
template<typename Type, uint32_t Max_Size>
inline void Class_Queue<Type, Max_Size>::Push(Type __Data)
{
    if (Length == Max_Size)
    {
        return; // 队列满，直接返回，丢弃新数据
    }
    else
    {
        if (Length == 0)
        {
            Front = 0;
            Rear = 0; // 空队列首次入队，重置首尾索引
        }
        else
        {
            Rear = (Rear + 1) % Max_Size; // 队尾后移，取模实现循环
        }

        Queue[Rear] = __Data; // 新数据存入队尾
        Length++; // 有效长度+1
    }
}
```
- **作用**：将数据`__Data`加入队列尾部。
- **核心逻辑**：
  1. 先检查队列是否已满（`Length == Max_Size`），满则直接返回（无覆盖，丢弃新数据）。
  2. 空队列首次入队：重置`Front`和`Rear`为0。
  3. 非空队列：队尾索引`Rear`加1后对`Max_Size`取模（循环特性，如Max_Size=5，Rear=4时，+1后取模为0）。
  4. 存入数据，有效长度+1。
- **参数**：`__Data`：要入队的`Type`类型数据。

#### （6）出队函数 `Pop()`
```cpp
template<typename Type, uint32_t Max_Size>
inline Type Class_Queue<Type, Max_Size>::Pop()
{
    Type temp = Queue[Front]; // 先缓存队首数据

    if (Length == 0)
    {
        // 队列为空，无操作，返回缓存的默认值
    }
    else
    {
        Front = (Front + 1) % Max_Size; // 队首后移，取模实现循环
        Length--; // 有效长度-1
    }

    return (temp); // 返回队首数据（空队列返回默认值）
}
```
- **作用**：弹出并返回队首元素（队首索引后移，有效长度减1）。
- **核心逻辑**：
  1. 先缓存队首数据（避免移动索引后丢失）。
  2. 检查队列是否为空：空则直接返回缓存值（无意义，需外部保证非空）。
  3. 非空则队首索引`Front`加1后取模，有效长度-1。
  4. 返回缓存的队首数据。
- **返回值**：弹出的队首数据（空队列返回`Queue[0]`的默认值）。

#### （7）清空队列 `Clear()`
```cpp
template<typename Type, uint32_t Max_Size>
inline void Class_Queue<Type, Max_Size>::Clear()
{
    Front = 0;
    Rear = 0;
    Length = 0;
}
```
- **作用**：快速清空队列（仅重置索引和长度，不清理数组内存，节省开销）。
- **逻辑**：将`Front`、`Rear`置0，`Length`置0，队列恢复初始空状态。

### 4. 源文件（alg_queue.cpp 1-27行）
```cpp
#include "alg_queue.h"
// 其余为注释和空声明
```
- **作用**：仅包含头文件，无实际实现。  
  原因：C++模板类的成员函数需在头文件中实现（模板实例化特性），源文件仅作为工程结构补充，无实际逻辑。

## 三、类的作用域与外设资源说明
### 1. 类的作用域
| 作用域      | 可访问范围                       | 典型使用场景                                               |
| ----------- | -------------------------------- | ---------------------------------------------------------- |
| `public`    | 所有外部代码（如主函数、其他类） | 调用`Init()`初始化、`Push()`入队、`Pop()`出队等核心操作    |
| `protected` | 类内部、子类                     | 子类可扩展队列功能（如增加满队列覆盖策略、空队列报错逻辑） |

### 2. 外设资源依赖
该队列类**无任何外设资源依赖**：
- 不操作硬件寄存器（如GPIO、UART、定时器）。
- 不依赖操作系统（如RTOS的信号量、队列）。
- 仅使用C++基础语法和`uint32_t`（标准整数类型），可运行在任意支持C++的平台（嵌入式MCU、PC等）。

## 四、使用示例（小白友好版）
```cpp
// 1. 定义一个int类型、最大容量10的队列
Class_Queue<int, 10> my_queue;

// 2. 初始化队列
my_queue.Init();

// 3. 入队
my_queue.Push(10);
my_queue.Push(20);
my_queue.Push(30);

// 4. 获取长度（此时返回3）
uint32_t len = my_queue.Get_Length();

// 5. 获取队首/队尾（队首10，队尾30）
int front_data = my_queue.Get_Front();
int rear_data = my_queue.Get_Rear();

// 6. 出队（返回10，队列长度变为2）
int pop_data = my_queue.Pop();

// 7. 清空队列
my_queue.Clear();
```

## 五、关键注意事项（小白必看）
1. **队列满处理**：`Push()`时队列满会直接丢弃新数据，需外部判断`Get_Length()`避免数据丢失。
2. **队空处理**：`Pop()`/`Get_Front()`/`Get_Rear()`在队空时返回无效值，需先调用`Get_Length()`检查。
3. **模板类型限制**：`Type`需支持赋值操作（如基本类型、自定义结构体均可）。
4. **静态内存**：`Max_Size`不宜过大（如超过10000），避免栈溢出（建议全局实例化队列）。