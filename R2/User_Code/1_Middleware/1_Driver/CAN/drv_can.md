> # STM32 HAL库CAN通信详解与代码解析
>
> ## 一、STM32 HAL库中有关CAN的函数介绍
>
> STM32 HAL库提供了完整的CAN通信API，主要函数包括：
>
> 1. **初始化相关函数**
>    - `HAL_CAN_Init(CAN_HandleTypeDef *hcan)`: 初始化CAN外设
>    - `HAL_CAN_Start(CAN_HandleTypeDef *hcan)`: 启动CAN外设
>    - `HAL_CAN_ConfigFilter(CAN_HandleTypeDef *hcan, CAN_FilterTypeDef *sFilterConfig)`: 配置CAN过滤器
> 2. **发送相关函数**
>    - `HAL_CAN_AddTxMessage(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *TxHeader, uint8_t *Data, uint32_t *TxMailbox)`: 添加发送消息到发送邮箱
>    - `HAL_CAN_TxMailbox0CompleteCallback()`: 发送邮箱0完成回调
> 3. **接收相关函数**
>    - `HAL_CAN_GetRxMessage(CAN_HandleTypeDef *hcan, uint32_t FIFONumber, CAN_RxHeaderTypeDef *RxHeader, uint8_t *RxData)`: 从接收FIFO获取消息
>    - `HAL_CAN_RxFifo0MsgPendingCallback()`: FIFO0接收中断回调
>    - `HAL_CAN_RxFifo1MsgPendingCallback()`: FIFO1接收中断回调
> 4. **中断管理**
>    - `__HAL_CAN_ENABLE_IT(CAN_HandleTypeDef *hcan, uint32_t It)`: 使能CAN中断
>    - `__HAL_CAN_GET_IT_SOURCE()`: 获取中断源
>
> ## 二、CAN通信格式
>
> CAN协议采用帧结构进行通信，主要有两种帧格式：
>
> ### 1. 标准帧（Standard Frame）
>
> - 11位标识符
> - 11位标识符 + 1位IDE（标识符扩展位）= 12位标识符
> - 适用于大多数场景
>
> ### 2. 扩展帧（Extended Frame）
>
> - 29位标识符
> - 11位标识符 + 18位扩展标识符 = 29位标识符
> - 适用于需要更多标识符的场景
>
> ### CAN帧结构
>
> ```
> | 帧起始 | 标识符 | 控制字段 | 数据字段 | CRC字段 | 确认字段 | 帧结束 |
> |--------|--------|----------|----------|---------|----------|--------|
> | 1位    | 11/29位| 4位      | 0-8字节  | 15位    | 2位      | 7位    |
> ```
>
> ### CAN帧类型
>
> - **数据帧（Data Frame）**: 传输数据
> - **远程帧（Remote Frame）**: 请求数据（RTR位为1）
>
> ## 三、drv_can.h与drv_can.cpp深度解析
>
> ### 1. drv_can.h 头文件分析
>
> **文件作用**：定义CAN通信的结构体、宏定义和函数声明
>
> ```c
> #ifndef DRV_CAN_H
> #define DRV_CAN_H
> 
> /* Includes */
> #include "stm32f4xx_hal.h"
> #include "can.h"
> #include <string.h>
> 
> /* Exported macros */
> #define CAN_FILTER(x) ((x) << 3)       // 滤波器编号
> #define CAN_FIFO_0 (0 << 2)            // 接收队列0
> #define CAN_FIFO_1 (1 << 2)            // 接收队列1
> #define CAN_STDID (0 << 1)             // 标准帧
> #define CAN_EXTID (1 << 1)             // 扩展帧
> #define CAN_DATA_TYPE (0 << 0)         // 数据帧
> #define CAN_REMOTE_TYPE (1 << 0)       // 遥控帧
> 
> /* Exported types */
> struct Struct_CAN_Rx_Buffer {      // CAN接收缓冲区结构体
>     CAN_RxHeaderTypeDef Header;     // 接收头信息
>     uint8_t Data[8];                // 接收数据缓冲区(8字节)
> };
> 
> typedef void (*CAN_Call_Back)(Struct_CAN_Rx_Buffer *);  // 回调函数类型
> 
> struct Struct_CAN_Manage_Object {  // CAN通信管理对象
>     CAN_HandleTypeDef *CAN_Handler; // CAN外设句柄
>     Struct_CAN_Rx_Buffer Rx_Buffer; // 接收缓冲区
>     CAN_Call_Back Callback_Function; // 回调函数
> };
> 
> /* Exported variables */
> extern bool init_finished;          // 初始化完成标志
> extern CAN_HandleTypeDef hcan1;     // CAN1外设句柄
> extern CAN_HandleTypeDef hcan2;     // CAN2外设句柄
> extern Struct_CAN_Manage_Object CAN1_Manage_Object; // CAN1管理对象
> extern Struct_CAN_Manage_Object CAN2_Manage_Object; // CAN2管理对象
> extern uint8_t CAN1_0x1fe_Tx_Data[]; // CAN1 0x1fe发送缓冲区
> // ... 其他发送缓冲区定义
> 
> /* Exported function declarations */
> void can_filter_mask_config(CAN_HandleTypeDef *hcan, uint8_t Object_Para, uint32_t ID, uint32_t Mask_ID);
> void CAN_Init(CAN_HandleTypeDef *hcan, CAN_Call_Back Callback_Function);
> uint8_t CAN_Send_Data(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *Data, uint16_t Length);
> void TIM_1ms_CAN_PeriodElapsedCallback();
> #endif
> ```
>
> **关键点说明**：
>
> - 该头文件定义了CAN通信的接口，为用户提供统一的CAN操作方式
> - `Struct_CAN_Rx_Buffer` 用于存储接收到的CAN消息
> - `Struct_CAN_Manage_Object` 是CAN通信的核心管理结构体
> - 使用了`extern`关键字声明全局变量，方便在其他文件中使用
>
> ### 2. drv_can.cpp 源文件分析
>
> **文件作用**：实现CAN通信的具体功能
>
> ```c
> #include "drv_can.h"
> 
> /* Private variables */
> Struct_CAN_Manage_Object CAN1_Manage_Object = {0};  // CAN1管理对象
> Struct_CAN_Manage_Object CAN2_Manage_Object = {0};  // CAN2管理对象
> 
> // CAN通信发送缓冲区
> uint8_t CAN1_0x1fe_Tx_Data[8];  // CAN1 0x1fe发送缓冲区
> // ... 其他发送缓冲区定义
> uint8_t CAN_Supercap_Tx_Data[8]; // 超级电容专属发送缓冲区
> 
> /* Private function declarations */
> void can_filter_mask_config(CAN_HandleTypeDef *hcan, uint8_t Object_Para, uint32_t ID, uint32_t Mask_ID);
> void CAN_Init(CAN_HandleTypeDef *hcan, CAN_Call_Back Callback_Function);
> uint8_t CAN_Send_Data(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *Data, uint16_t Length);
> void TIM_1ms_CAN_PeriodElapsedCallback();
> ```
>
> #### 2.1 can_filter_mask_config 函数解析
>
> ```c
> void can_filter_mask_config(CAN_HandleTypeDef *hcan, uint8_t Object_Para, uint32_t ID, uint32_t Mask_ID)
> {
>     CAN_FilterTypeDef can_filter_init_structure;
>     assert_param(hcan != NULL);
>     
>     if ((Object_Para & 0x02)) { // 标准帧
>         can_filter_init_structure.FilterIdHigh = ID << 3 >> 16;
>         can_filter_init_structure.FilterIdLow = ID << 3 | ((Object_Para & 0x03) << 1);
>         can_filter_init_structure.FilterMaskIdHigh = Mask_ID << 3 << 16;
>         can_filter_init_structure.FilterMaskIdLow = Mask_ID << 3 | ((Object_Para & 0x03) << 1);
>     } else { // 扩展帧
>         can_filter_init_structure.FilterIdHigh = ID << 5;
>         can_filter_init_structure.FilterIdLow = ((Object_Para & 0x03) << 1);
>         can_filter_init_structure.FilterMaskIdHigh = Mask_ID << 5;
>         can_filter_init_structure.FilterMaskIdLow = ((Object_Para & 0x03) << 1);
>     }
>     
>     can_filter_init_structure.FilterBank = Object_Para >> 3; // 滤波器序号
>     can_filter_init_structure.FilterFIFOAssignment = (Object_Para >> 2) & 0x01; // FIFO分配
>     can_filter_init_structure.FilterActivation = ENABLE;
>     can_filter_init_structure.FilterMode = CAN_FILTERMODE_IDMASK;
>     can_filter_init_structure.FilterScale = CAN_FILTERSCALE_32BIT;
>     can_filter_init_structure.SlaveStartFilterBank = 14;
>     
>     HAL_CAN_ConfigFilter(hcan, &can_filter_init_structure);
> }
> ```
>
> **作用**：配置CAN过滤器，用于筛选特定ID的消息
>
> **关键参数**：
>
> - `Object_Para`: 组合参数，包含滤波器编号、FIFO分配、ID类型和帧类型
> - `ID`: 要过滤的ID
> - `Mask_ID`: 掩码ID，用于定义ID中哪些位需要匹配
>
> **参数组合说明**：
>
> ```
> Object_Para = (滤波器编号 << 3) | (FIFO分配 << 2) | (ID类型 << 1) | (帧类型)
> ```
>
> **滤波器配置流程**：
>
> 1. 根据ID类型（标准帧/扩展帧）计算FilterIdHigh/Low和FilterMaskIdHigh/Low
> 2. 设置滤波器编号（0-27）
> 3. 设置FIFO分配（0或1）
> 4. 使能滤波器
> 5. 设置滤波器模式为ID掩码模式
> 6. 设置滤波器规模为32位
> 7. 配置从机起始滤波器编号
> 8. 调用HAL函数完成配置
>
> #### 2.2 CAN_Init 函数解析
>
> ```c
> void CAN_Init(CAN_HandleTypeDef *hcan, CAN_Call_Back Callback_Function)
> {
>     HAL_CAN_Start(hcan);
>     __HAL_CAN_ENABLE_IT(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
>     __HAL_CAN_ENABLE_IT(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
>     
>     if (hcan->Instance == CAN1) {
>         CAN1_Manage_Object.CAN_Handler = hcan;
>         CAN1_Manage_Object.Callback_Function = Callback_Function;
>         can_filter_mask_config(hcan, CAN_FILTER(0) | CAN_FIFO_0 | CAN_STDID | CAN_DATA_TYPE, 0, 0);
>         can_filter_mask_config(hcan, CAN_FILTER(1) | CAN_FIFO_1 | CAN_STDID | CAN_DATA_TYPE, 0, 0);
>     } else if (hcan->Instance == CAN2) {
>         CAN2_Manage_Object.CAN_Handler = hcan;
>         CAN2_Manage_Object.Callback_Function = Callback_Function;
>         can_filter_mask_config(hcan, CAN_FILTER(14) | CAN_FIFO_0 | CAN_STDID | CAN_DATA_TYPE, 0, 0);
>         can_filter_mask_config(hcan, CAN_FILTER(15) | CAN_FIFO_1 | CAN_STDID | CAN_DATA_TYPE, 0, 0);
>     }
> }
> ```
>
> **作用**：初始化CAN通信，设置接收中断和过滤器
>
> **关键步骤**：
>
> 1. 启动CAN外设
> 2. 使能FIFO0和FIFO1的接收消息中断
> 3. 根据CAN实例（CAN1/CAN2）初始化对应的管理对象
> 4. 配置两个过滤器（FIFO0和FIFO1），用于接收所有标准帧数据
>
> **过滤器配置说明**：
>
> - CAN1: 使用滤波器0和1
> - CAN2: 使用滤波器14和15
> - 两个过滤器都设置为接收所有标准帧数据（ID和掩码ID均为0）
>
> #### 2.3 CAN_Send_Data 函数解析
>
> ```c
> uint8_t CAN_Send_Data(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *Data, uint16_t Length)
> {
>     CAN_TxHeaderTypeDef tx_header;
>     uint32_t used_mailbox;
>     
>     assert_param(hcan != NULL);
>     
>     tx_header.StdId = ID;
>     tx_header.ExtId = 0;
>     tx_header.IDE = 0;    // 标准帧
>     tx_header.RTR = 0;    // 数据帧
>     tx_header.DLC = Length;
>     
>     return (HAL_CAN_AddTxMessage(hcan, &tx_header, Data, &used_mailbox));
> }
> ```
>
> **作用**：发送CAN数据帧
>
> **关键参数**：
>
> - `hcan`: CAN外设句柄
> - `ID`: CAN消息ID
> - `Data`: 发送数据指针
> - `Length`: 数据长度（0-8字节）
>
> **发送流程**：
>
> 1. 设置发送头信息（ID、帧类型、数据长度等）
> 2. 调用HAL函数添加发送消息到发送邮箱
> 3. 返回发送状态
>
> #### 2.4 TIM_1ms_CAN_PeriodElapsedCallback 函数解析
>
> ```c
> void TIM_1ms_CAN_PeriodElapsedCallback()
> {
>     static int mod2 = 0;
>     mod2++;
>     if (mod2 == 2) {
>         mod2 = 0;
>         // CAN2半频电机
>         CAN_Send_Data(&hcan2, 0x1fe, CAN2_0x1fe_Tx_Data, 8);
>         CAN_Send_Data(&hcan2, 0x200, CAN2_0x200_Tx_Data, 8);
>     }
>     // 摩擦轮和拨弹盘电机
>     CAN_Send_Data(&hcan1, 0x200, CAN1_0x200_Tx_Data, 8);
>     // 云台电机
>     CAN_Send_Data(&hcan1, 0x1fe, CAN1_0x1fe_Tx_Data, 8);
> }
> ```
>
> **作用**：定时器中断回调函数，用于周期性发送CAN数据
>
> **关键点**：
>
> - 以2ms为周期发送（mod2计数）
> - CAN2（舵轮底盘）以半频发送
> - CAN1（摩擦轮、云台）以全频发送
>
> **发送策略**：
>
> - 0x1fe: 云台电机控制
> - 0x200: 摩擦轮/拨弹盘电机控制
> - 0x1fe/0x200: 舵轮底盘控制
>
> #### 2.5 HAL_CAN_RxFifo0MsgPendingCallback 函数解析
>
> ```c
> void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
> {
>     if (init_finished == false) {
>         return;
>     }
>     
>     if (hcan->Instance == CAN1) {
>         HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO0, &CAN1_Manage_Object.Rx_Buffer.Header, CAN1_Manage_Object.Rx_Buffer.Data);
>         if (CAN1_Manage_Object.Callback_Function != nullptr) {
>             CAN1_Manage_Object.Callback_Function(&CAN1_Manage_Object.Rx_Buffer);
>         }
>     } else if (hcan->Instance == CAN2) {
>         HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO0, &CAN2_Manage_Object.Rx_Buffer.Header, CAN2_Manage_Object.Rx_Buffer.Data);
>         if (CAN2_Manage_Object.Callback_Function != nullptr) {
>             CAN2_Manage_Object.Callback_Function(&CAN2_Manage_Object.Rx_Buffer);
>         }
>     }
> }
> ```
>
> **作用**：FIFO0接收中断处理函数
>
> **关键流程**：
>
> 1. 检查初始化是否完成
> 2. 根据CAN实例（CAN1/CAN2）获取接收消息
> 3. 调用注册的回调函数处理接收到的数据
>
> **中断处理流程**：
>
> ```
> CAN接收中断触发 → 获取接收消息 → 调用回调函数
> ```
>
> #### 2.6 HAL_CAN_RxFifo1MsgPendingCallback 函数解析
>
> ```c
> void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
> {
>     if (init_finished == false) {
>         return;
>     }
>     
>     if (hcan->Instance == CAN1) {
>         HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO1, &CAN1_Manage_Object.Rx_Buffer.Header, CAN1_Manage_Object.Rx_Buffer.Data);
>         if (CAN1_Manage_Object.Callback_Function != nullptr) {
>             CAN1_Manage_Object.Callback_Function(&CAN1_Manage_Object.Rx_Buffer);
>         }
>     } else if (hcan->Instance == CAN2) {
>         HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO1, &CAN2_Manage_Object.Rx_Buffer.Header, CAN2_Manage_Object.Rx_Buffer.Data);
>         if (CAN2_Manage_Object.Callback_Function != nullptr) {
>             CAN2_Manage_Object.Callback_Function(&CAN2_Manage_Object.Rx_Buffer);
>         }
>     }
> }
> ```
>
> **作用**：FIFO1接收中断处理函数
>
> **关键点**：
>
> - 与FIFO0处理函数类似，但处理的是FIFO1队列
> - 用于接收另一组CAN消息
>
> ## 四、系统架构图
>
> ```mermaid
> graph TD
>     A[STM32F4] --> B[CAN1]
>     A --> C[CAN2]
>     B --> D[电机控制]
>     C --> E[舵轮底盘]
>     D --> F[摩擦轮/拨弹盘]
>     D --> G[云台]
>     E --> H[舵向电机]
>     E --> I[轮向电机]
>     
>     subgraph CAN通信框架
>         B --> J[CAN1管理对象]
>         C --> K[CAN2管理对象]
>         J --> L[回调函数]
>         K --> M[回调函数]
>         L --> N[处理电机控制数据]
>         M --> O[处理舵轮底盘数据]
>     end
>     
>     subgraph 数据流
>         P[发送数据] -->|0x1fe| J
>         P -->|0x200| J
>         P -->|0x1fe| K
>         P -->|0x200| K
>         Q[接收数据] -->|0x1fe| J
>         Q -->|0x200| J
>         Q -->|0x1fe| K
>         Q -->|0x200| K
>     end
> ```
>
> ## 五、关键外设资源使用
>
> | 资源        | 用途                         | 实例         |
> | ----------- | ---------------------------- | ------------ |
> | CAN1        | 摩擦轮、拨弹盘、云台电机控制 | 0x1fe, 0x200 |
> | CAN2        | 舵轮底盘控制                 | 0x1fe, 0x200 |
> | 滤波器0-1   | CAN1的FIFO0和FIFO1           | 0x00, 0x01   |
> | 滤波器14-15 | CAN2的FIFO0和FIFO1           | 0x0E, 0x0F   |
> | 两个FIFO    | 接收消息队列                 | FIFO0, FIFO1 |
>
> ## 六、总结
>
> 该CAN通信驱动程序是一个高度模块化的实现，主要特点：
>
> 1. **双CAN外设支持**：同时支持CAN1和CAN2，分别用于不同的电机控制
> 2. **双FIFO机制**：每个CAN外设使用两个FIFO队列，提高接收效率
> 3. **回调函数机制**：通过回调函数实现灵活的消息处理
> 4. **定时发送机制**：通过定时器中断实现周期性发送
> 5. **标准帧过滤**：配置过滤器接收所有标准帧数据
>
> 该驱动程序适用于机器人控制、电机控制等场景，特别是需要同时控制多种电机的场合，如机器人底盘、云台、摩擦轮等。
>
> **最佳实践建议**：
>
> - 在初始化函数中注册回调函数
> - 使用发送缓冲区存储待发送数据
> - 根据实际硬件配置ID和数据格式
> - 通过定时器中断实现周期性发送，避免阻塞主程序
> - 为不同CAN通信任务分配不同的ID，提高可维护性