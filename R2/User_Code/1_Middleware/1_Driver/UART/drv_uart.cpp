/**
 * @file drv_uart.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief 仿照SCUT-Robotlab改写的UART通信初始化与配置流程
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 * @date 2023-11-18 1.1 修改成cpp
 * @date 2024-05-05 1.2 新增错误中断, 24赛季定稿
 * @date 2024-08-22 2.1 新增回调函数空指针判定
 * @date 2026-03-29 3.1 去除帧头标,适配vofa+串口调试软件
 * @date 2026-04-01 3.2 适配vofa+的justfloat协议,新增了关闭DMA传输过半中断
 * @date 2026-04-01 3.2 注释掉了中科大的程序初始化完成标志位
 *
 * @copyright USTC-RoboWalker (c) 2023-2024
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "drv_uart.h"
#include <string.h>

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

Struct_UART_Manage_Object UART1_Manage_Object = {0};
Struct_UART_Manage_Object UART2_Manage_Object = {0};
Struct_UART_Manage_Object UART3_Manage_Object = {0};
Struct_UART_Manage_Object UART4_Manage_Object = {0};
Struct_UART_Manage_Object UART5_Manage_Object = {0};
Struct_UART_Manage_Object UART6_Manage_Object = {0};
Struct_UART_Manage_Object UART7_Manage_Object = {0};
Struct_UART_Manage_Object UART8_Manage_Object = {0};

/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化UART
 *
 * @param huart UART编号
 * @param Callback_Function 处理回调函数
 * @param Rx_Buffer_Length 接收缓冲区长度
 */
void UART_Init(UART_HandleTypeDef *huart,
               UART_Call_Back Callback_Function,
               uint16_t Rx_Buffer_Length)
{
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    if (huart->Instance == USART1)
    {
        UART1_Manage_Object.UART_Handler = huart;
        UART1_Manage_Object.Callback_Function = Callback_Function;
        UART1_Manage_Object.Rx_Buffer_Length = Rx_Buffer_Length;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART1_Manage_Object.Rx_Buffer, UART1_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == USART2)
    {
        UART2_Manage_Object.UART_Handler = huart;
        UART2_Manage_Object.Callback_Function = Callback_Function;
        UART2_Manage_Object.Rx_Buffer_Length = Rx_Buffer_Length;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART2_Manage_Object.Rx_Buffer, UART2_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == USART3)
    {
        UART3_Manage_Object.UART_Handler = huart;
        UART3_Manage_Object.Callback_Function = Callback_Function;
        UART3_Manage_Object.Rx_Buffer_Length = Rx_Buffer_Length;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART3_Manage_Object.Rx_Buffer, UART3_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == UART4)
    {
        UART4_Manage_Object.UART_Handler = huart;
        UART4_Manage_Object.Callback_Function = Callback_Function;
        UART4_Manage_Object.Rx_Buffer_Length = Rx_Buffer_Length;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART4_Manage_Object.Rx_Buffer, UART4_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == UART5)
    {
        UART5_Manage_Object.UART_Handler = huart;
        UART5_Manage_Object.Callback_Function = Callback_Function;
        UART5_Manage_Object.Rx_Buffer_Length = Rx_Buffer_Length;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART5_Manage_Object.Rx_Buffer, UART5_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == USART6)
    {
        UART6_Manage_Object.UART_Handler = huart;
        UART6_Manage_Object.Callback_Function = Callback_Function;
        UART6_Manage_Object.Rx_Buffer_Length = Rx_Buffer_Length;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART6_Manage_Object.Rx_Buffer, UART6_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == UART7)
    {
        UART7_Manage_Object.UART_Handler = huart;
        UART7_Manage_Object.Callback_Function = Callback_Function;
        UART7_Manage_Object.Rx_Buffer_Length = Rx_Buffer_Length;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART7_Manage_Object.Rx_Buffer, UART7_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == UART8)
    {
        UART8_Manage_Object.UART_Handler = huart;
        UART8_Manage_Object.Callback_Function = Callback_Function;
        UART8_Manage_Object.Rx_Buffer_Length = Rx_Buffer_Length;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART8_Manage_Object.Rx_Buffer, UART8_Manage_Object.Rx_Buffer_Length);
    }
}

/**
 * @brief 掉线重新初始化UART
 *
 * @param huart UART编号
 */
void UART_Reinit(UART_HandleTypeDef *huart)
{
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    if (huart->Instance == USART1)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART1_Manage_Object.Rx_Buffer, UART1_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == USART2)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART2_Manage_Object.Rx_Buffer, UART2_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == USART3)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART3_Manage_Object.Rx_Buffer, UART3_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == UART4)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART4_Manage_Object.Rx_Buffer, UART4_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == UART5)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART5_Manage_Object.Rx_Buffer, UART5_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == USART6)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART6_Manage_Object.Rx_Buffer, UART6_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == UART7)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART7_Manage_Object.Rx_Buffer, UART7_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == UART8)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART8_Manage_Object.Rx_Buffer, UART8_Manage_Object.Rx_Buffer_Length);
    }
}

/**
 * @brief 发送JustFloat协议数据帧
 *
 * @param uart_obj UART管理对象指针
 * @param float_data 浮点数据数组指针
 * @param channel_count 通道数量
 * @return uint8_t HAL状态
 */
uint8_t UART_Send_JustFloat(struct Struct_UART_Manage_Object *uart_obj,
                            float *float_data,
                            uint8_t channel_count)
{
    // 检查参数有效性
    if (uart_obj == NULL || float_data == NULL || channel_count == 0)
    {
        return HAL_ERROR;
    }

    // 计算数据长度（字节数）
    uint16_t data_length = channel_count * sizeof(float);

    // 检查缓冲区是否足够（数据 + 4字节帧尾）
    if ((data_length + 4) > UART_BUFFER_SIZE)
    {
        return HAL_ERROR; // 缓冲区不足
    }

    // 1. 将浮点数据复制到发送缓冲区
    // STM32是小端格式，直接复制即可
    memcpy(uart_obj->Tx_Buffer, float_data, data_length);

    // 2. 添加帧尾：0x00, 0x00, 0x80, 0x7f
    uart_obj->Tx_Buffer[data_length] = 0x00;
    uart_obj->Tx_Buffer[data_length + 1] = 0x00;
    uart_obj->Tx_Buffer[data_length + 2] = 0x80;
    uart_obj->Tx_Buffer[data_length + 3] = 0x7f;

    // 3. 调用底层发送函数
    return HAL_UART_Transmit_DMA(uart_obj->UART_Handler, uart_obj->Tx_Buffer, data_length + 4);
}

/**
 * @brief 原始UART发送函数（保留）
 *
 * @param huart UART句柄
 * @param Data 数据指针
 * @param Length 长度
 * @return uint8_t HAL状态
 */
uint8_t UART_Send_Data(UART_HandleTypeDef *huart,
                       uint8_t *Data,
                       uint16_t Length)
{
    return HAL_UART_Transmit_DMA(huart, Data, Length);
}

/**
 * @brief UART的TIM定时器中断发送回调函数
 *
 */
void TIM_1ms_UART_PeriodElapsedCallback()
{
}

/**
 * @brief HAL库UART接收DMA空闲中断
 *
 * @param huart UART编号
 * @param Size 长度
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    // // 判断程序初始化完成
    // if (init_finished == false)
    // {
    //     return;
    // }

    // 选择回调函数
    if (huart->Instance == USART1)
    {
        if (UART1_Manage_Object.Callback_Function != nullptr)
        {
            UART1_Manage_Object.Callback_Function(UART1_Manage_Object.Rx_Buffer, Size);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART1_Manage_Object.Rx_Buffer, UART1_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == USART2)
    {
        if (UART2_Manage_Object.Callback_Function != nullptr)
        {
            UART2_Manage_Object.Callback_Function(UART2_Manage_Object.Rx_Buffer, Size);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART2_Manage_Object.Rx_Buffer, UART2_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == USART3)
    {
        if (UART3_Manage_Object.Callback_Function != nullptr)
        {
            UART3_Manage_Object.Callback_Function(UART3_Manage_Object.Rx_Buffer, Size);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART3_Manage_Object.Rx_Buffer, UART3_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == UART4)
    {
        if (UART4_Manage_Object.Callback_Function != nullptr)
        {
            UART4_Manage_Object.Callback_Function(UART4_Manage_Object.Rx_Buffer, Size);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART4_Manage_Object.Rx_Buffer, UART4_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == UART5)
    {
        if (UART5_Manage_Object.Callback_Function != nullptr)
        {
            UART5_Manage_Object.Callback_Function(UART5_Manage_Object.Rx_Buffer, Size);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART5_Manage_Object.Rx_Buffer, UART5_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == USART6)
    {
        if (UART6_Manage_Object.Callback_Function != nullptr)
        {
            UART6_Manage_Object.Callback_Function(UART6_Manage_Object.Rx_Buffer, Size);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART6_Manage_Object.Rx_Buffer, UART6_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == UART7)
    {
        if (UART7_Manage_Object.Callback_Function != nullptr)
        {
            UART7_Manage_Object.Callback_Function(UART7_Manage_Object.Rx_Buffer, Size);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART7_Manage_Object.Rx_Buffer, UART7_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == UART8)
    {
        if (UART8_Manage_Object.Callback_Function != nullptr)
        {
            UART8_Manage_Object.Callback_Function(UART8_Manage_Object.Rx_Buffer, Size);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART8_Manage_Object.Rx_Buffer, UART8_Manage_Object.Rx_Buffer_Length);
    }
    // 关闭DMA传输过半中断, 避免干扰正常的接收流程
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
}

/**
 * @brief HAL库UART错误中断
 *
 * @param huart UART编号
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART1_Manage_Object.Rx_Buffer, UART1_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == USART2)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART2_Manage_Object.Rx_Buffer, UART2_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == USART3)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART3_Manage_Object.Rx_Buffer, UART3_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == UART4)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART4_Manage_Object.Rx_Buffer, UART4_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == UART5)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART5_Manage_Object.Rx_Buffer, UART5_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == USART6)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART6_Manage_Object.Rx_Buffer, UART6_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == UART7)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART7_Manage_Object.Rx_Buffer, UART7_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == UART8)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART8_Manage_Object.Rx_Buffer, UART8_Manage_Object.Rx_Buffer_Length);
    }
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER *************************/
