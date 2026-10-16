#include "bsp_dual_adc.h"

#include "adc.h"
#include "tim.h"

#include <string.h>

/*
 * DMA缓冲区：
 * 2个半区，每个半区64组采样
 */
#define ADC_DMA_BUFFER_SIZE (DUAL_ADC_BLOCK_SIZE * 2U)

/*
 * ADC1 + ADC2 双模式DMA数据缓冲区
 *
 * 每个元素32位：
 * 低16位 = ADC1 CH7
 * 高16位 = ADC2 CH8
 *
 * 要求放在DMA1可访问的RAM。
 * 本版本要求D-Cache关闭。
 */
static uint32_t s_dma_buffer[ADC_DMA_BUFFER_SIZE]
    __attribute__((aligned(32)));


/* 软件数据块队列 */
static DualADC_Block s_queue[DUAL_ADC_QUEUE_SIZE];

/* 队列读写索引 */
static volatile uint32_t s_queue_head = 0U;
static volatile uint32_t s_queue_tail = 0U;

/* 采样统计 */
static volatile uint32_t s_sample_count = 0U;
static volatile uint32_t s_dropped_samples = 0U;

/* 错误统计 */
static volatile uint32_t s_error_count = 0U;
static volatile uint32_t s_last_error = 0U;
static volatile uint32_t s_start_error = 0U;


/*
 * 将DMA已经采集完成的一个半区复制到软件队列中。
 *
 * offset=0：前半区
 * offset=64：后半区
 */
static void DualADC_SaveBlock(uint32_t offset)
{
    uint32_t next_head;
    uint32_t first_index;

    next_head =
        (s_queue_head + 1U) % DUAL_ADC_QUEUE_SIZE;

    first_index = s_sample_count;

    /* 完成一块，推进采样编号 */
    s_sample_count += DUAL_ADC_BLOCK_SIZE;

    /* 队列满，记录丢样 */
    if (next_head == s_queue_tail)
    {
        s_dropped_samples += DUAL_ADC_BLOCK_SIZE;
        return;
    }

    /* 记录该数据块的起始编号 */
    s_queue[s_queue_head].first_index = first_index;

    /* 复制64组双通道原始数据 */
    memcpy(
        s_queue[s_queue_head].samples,
        &s_dma_buffer[offset],
        sizeof(s_queue[s_queue_head].samples)
    );

    /* 确保数据写入完成，再发布队列索引 */
    __DMB();

    s_queue_head = next_head;
}


/*
 * 校准ADC1、ADC2，并启动同步采集。
 *
 * 返回HAL_OK表示启动成功。
 */
HAL_StatusTypeDef BSP_DualADC_Start(void)
{
    /* 清空软件状态 */
    s_queue_head = 0U;
    s_queue_tail = 0U;

    s_sample_count = 0U;
    s_dropped_samples = 0U;

    s_error_count = 0U;
    s_last_error = 0U;
    s_start_error = 0U;

    /*
     * 第一步：ADC1校准
     */
    if (HAL_ADCEx_Calibration_Start(
            &hadc1,
            ADC_CALIB_OFFSET_LINEARITY,
            ADC_SINGLE_ENDED) != HAL_OK)
    {
        s_start_error = 1U;
        return HAL_ERROR;
    }

    /*
     * 第二步：ADC2校准
     */
    if (HAL_ADCEx_Calibration_Start(
            &hadc2,
            ADC_CALIB_OFFSET_LINEARITY,
            ADC_SINGLE_ENDED) != HAL_OK)
    {
        s_start_error = 2U;
        return HAL_ERROR;
    }

    /*
     * 第三步：启动双ADC多模式DMA
     */
    if (HAL_ADCEx_MultiModeStart_DMA(
            &hadc1,
            s_dma_buffer,
            ADC_DMA_BUFFER_SIZE) != HAL_OK)
    {
        s_start_error = 3U;
        return HAL_ERROR;
    }

    /*
     * 第四步：启动TIM2
     *
     * TIM2每1ms触发一组双ADC采样。
     */
    __HAL_TIM_SET_COUNTER(&htim2, 0U);

    if (HAL_TIM_Base_Start(&htim2) != HAL_OK)
    {
        s_start_error = 4U;

        HAL_ADCEx_MultiModeStop_DMA(&hadc1);

        return HAL_ERROR;
    }

    return HAL_OK;
}


/*
 * ADC DMA前半区完成回调
 */
void HAL_ADC_ConvHalfCpltCallback(
    ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        DualADC_SaveBlock(0U);
    }
}


/*
 * ADC DMA后半区完成回调
 */
void HAL_ADC_ConvCpltCallback(
    ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        DualADC_SaveBlock(DUAL_ADC_BLOCK_SIZE);
    }
}


/*
 * ADC硬件/DMA相关错误回调
 */
void HAL_ADC_ErrorCallback(
    ADC_HandleTypeDef *hadc)
{
    if ((hadc->Instance == ADC1) ||
        (hadc->Instance == ADC2))
    {
        s_error_count++;
        s_last_error = HAL_ADC_GetError(hadc);
    }
}


/*
 * 主循环读取一个完整数据块
 */
bool BSP_DualADC_ReadBlock(DualADC_Block *block)
{
    if (block == NULL)
    {
        return false;
    }

    /* 队列为空 */
    if (s_queue_head == s_queue_tail)
    {
        return false;
    }

    __DMB();

    /* 将队列数据复制给调用者 */
    memcpy(
        block,
        &s_queue[s_queue_tail],
        sizeof(DualADC_Block)
    );

    __DMB();

    /* 读取完成后更新读指针 */
    s_queue_tail =
        (s_queue_tail + 1U) % DUAL_ADC_QUEUE_SIZE;

    return true;
}


/* 获取已完成采样组数 */
uint32_t BSP_DualADC_GetSampleCount(void)
{
    return s_sample_count;
}


/* 获取软件队列丢失的采样组数 */
uint32_t BSP_DualADC_GetDroppedSamples(void)
{
    return s_dropped_samples;
}


/* 获取ADC错误次数 */
uint32_t BSP_DualADC_GetErrorCount(void)
{
    return s_error_count;
}


/* 获取最近一次ADC错误码 */
uint32_t BSP_DualADC_GetLastError(void)
{
    return s_last_error;
}


/* 获取启动失败阶段 */
uint32_t BSP_DualADC_GetStartError(void)
{
    return s_start_error;
}



