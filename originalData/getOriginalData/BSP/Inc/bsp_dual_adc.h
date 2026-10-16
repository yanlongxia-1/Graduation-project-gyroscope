#ifndef BSP_DUAL_ADC_H
#define BSP_DUAL_ADC_H

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* 每个数据块保存64组双通道ADC数据 */
#define DUAL_ADC_BLOCK_SIZE     64U

/* 软件队列总槽位数 */
#define DUAL_ADC_QUEUE_SIZE     8U


/*
 * 双通道原始数据块
 */
typedef struct
{
    /* 本数据块第一个采样点的序号 */
    uint32_t first_index;

    /* 每个32位元素包含两路16位ADC结果 */
    uint32_t samples[DUAL_ADC_BLOCK_SIZE];

} DualADC_Block;


/* 初始化校准并启动双ADC采集 */
HAL_StatusTypeDef BSP_DualADC_Start(void);

/* 读取一个完整的数据块 */
bool BSP_DualADC_ReadBlock(DualADC_Block *block);

/* 获取已完成采样组数 */
uint32_t BSP_DualADC_GetSampleCount(void);

/* 获取因软件队列满而丢失的采样组数 */
uint32_t BSP_DualADC_GetDroppedSamples(void);

/* 获取ADC错误回调次数 */
uint32_t BSP_DualADC_GetErrorCount(void);

/* 获取最近一次ADC HAL错误码 */
uint32_t BSP_DualADC_GetLastError(void);

/* 获取启动失败阶段，0表示无启动失败 */
uint32_t BSP_DualADC_GetStartError(void);


/* 从32位数据中获取ADC1 CH7 */
static inline uint16_t BSP_DualADC_GetCH7(uint32_t word)
{
    return (uint16_t)(word & 0xFFFFU);
}

/* 从32位数据中获取ADC2 CH8 */
static inline uint16_t BSP_DualADC_GetCH8(uint32_t word)
{
    return (uint16_t)((word >> 16) & 0xFFFFU);
}

#endif
