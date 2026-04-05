/**
 * @file dvc_serialplot.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief 串口绘图
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 * @date 2026.3.29 0.2 去除帧头标,适配vofa+串口调试软件
 *
 * @copyright USTC-RoboWalker (c) 2023
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_serialplot.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 串口绘图初始化
 *
 * @param __huart 指定的UART
 * @param __Rx_Variable_Assignment_Num 接收指令字典的数量
 * @param __Rx_Variable_Assignment_List 接收指令字典列表
 * @param __Data_Type 传输数据类型, 默认float
 */
void Class_Serialplot::Init(UART_HandleTypeDef *huart,
                            Enum_Serialplot_Checksum_8 __Checksum_8,
                            uint8_t __Rx_Variable_Assignment_Num,
                            char **__Rx_Variable_Assignment_List,
                            Enum_Serialplot_Data_Type __Data_Type)
{
    if (huart->Instance == USART1)
    {
        UART_Manage_Object = &UART1_Manage_Object;
    }
    else if (huart->Instance == USART2)
    {
        UART_Manage_Object = &UART2_Manage_Object;
    }
    else if (huart->Instance == USART3)
    {
        UART_Manage_Object = &UART3_Manage_Object;
    }
    else if (huart->Instance == UART4)
    {
        UART_Manage_Object = &UART4_Manage_Object;
    }
    else if (huart->Instance == UART5)
    {
        UART_Manage_Object = &UART5_Manage_Object;
    }
    else if (huart->Instance == USART6)
    {
        UART_Manage_Object = &UART6_Manage_Object;
    }
    else if (huart->Instance == UART7)
    {
        UART_Manage_Object = &UART7_Manage_Object;
    }
    else if (huart->Instance == UART8)
    {
        UART_Manage_Object = &UART8_Manage_Object;
    }

    Checksum_8 = __Checksum_8;
    Rx_Variable_Num = __Rx_Variable_Assignment_Num;
    Rx_Variable_List = __Rx_Variable_Assignment_List;
    Tx_Data_Type = __Data_Type;
}

/**
 * @brief UART通信接收回调函数
 *
 * @param Rx_Data 接收的数据
 */
void Class_Serialplot::UART_RxCpltCallback(uint8_t *Rx_Data, uint16_t Length)
{
    Data_Process(Length);
}

/**
 * @brief TIM定时器中断增加数据到发送缓冲区
 *
 */
void Class_Serialplot::TIM_1ms_Write_PeriodElapsedCallback()
{
    Output();

    size_t data_length = 0;
    if (Checksum_8 == Serialplot_Checksum_8_ENABLE)
    {
        data_length++;
    }

    switch (Tx_Data_Type)
    {
    case (Serialplot_Data_Type_UINT8):
    {
        data_length += Data_Number * sizeof(uint8_t);

        break;
    }
    case (Serialplot_Data_Type_UINT16):
    {
        data_length += Data_Number * sizeof(uint16_t);

        break;
    }
    case (Serialplot_Data_Type_UINT32):
    {
        data_length += Data_Number * sizeof(uint32_t);

        break;
    }
    case (Serialplot_Data_Type_INT8):
    {
        data_length += Data_Number * sizeof(int8_t);

        break;
    }
    case (Serialplot_Data_Type_INT16):
    {
        data_length += Data_Number * sizeof(int16_t);

        break;
    }
    case (Serialplot_Data_Type_INT32):
    {
        data_length += Data_Number * sizeof(int32_t);

        break;
    }
    case (Serialplot_Data_Type_FLOAT):
    {
        data_length += Data_Number * sizeof(float);

        break;
    }
    case (Serialplot_Data_Type_DOUBLE):
    {
        data_length += Data_Number * sizeof(double);

        break;
    }
    }

    // 没开DMA所以要用IT
    HAL_UART_Transmit_IT(UART_Manage_Object->UART_Handler, UART_Manage_Object->Tx_Buffer, data_length);
}

/**
 * @brief 按VOFA+ JustFloat协议发送浮点通道数据
 *
 * 协议格式: Set_Data注册的float数组(小端) + 0x00 0x00 0x80 0x7f
 *
 * @return uint8_t HAL状态
 */
uint8_t Class_Serialplot::JustFloat_Output()
{
    if (UART_Manage_Object == NULL || Data_Number == 0)
    {
        return HAL_ERROR;
    }

    // Set_Data中注册的数据按float通道进行打包
    uint16_t payload_length = Data_Number * sizeof(float);
    // 协议长度 = payload + JustFloat帧尾(4B)
    uint16_t frame_length = payload_length + 4;

    if (frame_length > UART_BUFFER_SIZE)
    {
        return HAL_ERROR;
    }

    uint8_t *tmp_buffer = UART_Manage_Object->Tx_Buffer;
    memset(tmp_buffer, 0, UART_BUFFER_SIZE);

    // 1) 填充float通道数据
    for (uint8_t i = 0; i < Data_Number; i++)
    {
        memcpy(tmp_buffer + i * sizeof(float), Data[i], sizeof(float));
    }

    // 2) 添加JustFloat帧尾: 0x00 0x00 0x80 0x7f
    tmp_buffer[payload_length + 0] = 0x00;
    tmp_buffer[payload_length + 1] = 0x00;
    tmp_buffer[payload_length + 2] = 0x80;
    tmp_buffer[payload_length + 3] = 0x7f;

    // 3) 使用中断方式发送
    HAL_StatusTypeDef hal_ret = HAL_UART_Transmit_IT(UART_Manage_Object->UART_Handler, UART_Manage_Object->Tx_Buffer, frame_length);
    return (uint8_t)hal_ret;
}

/**
 * @brief 数据处理过程
 * @param Length 接收数据长度
 */
void Class_Serialplot::Data_Process(uint16_t Length)
{
    int flag;
    flag = _Judge_Variable_Name(Length);
    _Judge_Variable_Value(Length, flag);
}

/**
 * @brief 判断指令变量名
 *
 * @param Length 接收数据长度
 * @return uint8_t 指令数值位置的指针, 也就是"variable=value#"中v的坐标
 */
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
    tmp_variable_name[flag] = 0;

    // 比对是否在列表中
    for (int i = 0; i < Rx_Variable_Num; i++)
    {
        // 如果在则标记变量名编号
        if (strcmp(tmp_variable_name, (char *)((int)Rx_Variable_List + SERIALPLOT_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH * i)) == 0)
        {
            Variable_Index = i;
            return (flag + 1);
        }
    }
    // 如果变量名不在则-1
    Variable_Index = -1;
    return (flag + 1);
}

/**
 * @brief 判断指令变量数值
 *
 * @param Length 接收数据长度
 * @param flag 指令数值位置的指针
 */
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
            tmp_dot_flag = i;
        }
        else
        {
            Variable_Value = Variable_Value * 10.0f + (UART_Manage_Object->Rx_Buffer[i] - '0');
        }
    }

    // 如果有小数点则考虑
    if (tmp_dot_flag != 0)
    {
        Variable_Value /= pow(10.0f, i - tmp_dot_flag - 1.0f);
    }

    Variable_Value *= tmp_sign_coefficient;
}

/**
 * @brief 串口绘图数据输出到UART发送缓冲区
 *
 */
void Class_Serialplot::Output()
{
    uint8_t *tmp_buffer = UART_Manage_Object->Tx_Buffer;
    
    //清除发送缓冲区
    memset(tmp_buffer, 0, UART_BUFFER_SIZE);

    // 填充数据
    if (Tx_Data_Type == Serialplot_Data_Type_UINT8 || Tx_Data_Type == Serialplot_Data_Type_INT8)
    {
        for (int i = 0; i < Data_Number; i++)
        {
            memcpy(tmp_buffer + i * sizeof(uint8_t), Data[i], sizeof(uint8_t));
        }

        if (Checksum_8 != Serialplot_Checksum_8_DISABLE)
        {
            tmp_buffer[Data_Number * sizeof(uint8_t)] = Math_Sum_8(tmp_buffer, Data_Number * sizeof(uint8_t));
        }
    }
    else if (Tx_Data_Type == Serialplot_Data_Type_UINT16 || Tx_Data_Type == Serialplot_Data_Type_INT16)
    {
        for (int i = 0; i < Data_Number; i++)
        {
            memcpy(tmp_buffer + i * sizeof(uint16_t), Data[i], sizeof(uint16_t));
        }

        if (Checksum_8 != Serialplot_Checksum_8_DISABLE)
        {
            tmp_buffer[Data_Number * sizeof(uint16_t)] = Math_Sum_8(tmp_buffer, Data_Number * sizeof(uint16_t));
        }
    }
    else if (Tx_Data_Type == Serialplot_Data_Type_UINT32 || Tx_Data_Type == Serialplot_Data_Type_INT32 || Tx_Data_Type == Serialplot_Data_Type_FLOAT)
    {
        for (int i = 0; i < Data_Number; i++)
        {
            memcpy(tmp_buffer + i * sizeof(uint32_t), Data[i], sizeof(uint32_t));
        }

        if (Checksum_8 != Serialplot_Checksum_8_DISABLE)
        {
            tmp_buffer[Data_Number * sizeof(uint32_t)] = Math_Sum_8(tmp_buffer, Data_Number * sizeof(uint32_t));
        }
    }
    else if (Tx_Data_Type == Serialplot_Data_Type_DOUBLE)
    {
        for (int i = 0; i < Data_Number; i++)
        {
            memcpy(tmp_buffer + i * sizeof(uint64_t), Data[i], sizeof(uint64_t));
        }

        if (Checksum_8 != Serialplot_Checksum_8_DISABLE)
        {
            tmp_buffer[Data_Number * sizeof(uint64_t)] = Math_Sum_8(tmp_buffer, Data_Number * sizeof(uint64_t));
        }
    }
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
