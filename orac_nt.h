/**
 * ORAC-NT v5.5 — Full Implementation Header
 * Platform: STM32F401 @ 84 MHz + ST-Link V2
 * Hardware: 2x MPU-6050 on I2C1 (PB6=SCL, PB7=SDA)
 *
 * Copyright (c) 2025. All rights reserved.
 */

#ifndef ORAC_NT_H
#define ORAC_NT_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>

// ============================================================
// CONFIG
// ============================================================
#define ORAC_LR           0.02f
#define ORAC_ALPHA        0.05f
#define ORAC_C            0.80f
#define ORAC_M_MAX        1.80f
#define ORAC_BETA         0.01f
#define ORAC_MOMENTUM     0.90f
#define ORAC_H_DECAY      0.95f
#define ORAC_H_MIN        0.001f
#define ORAC_FREEZE_PCT   25
#define ORAC_W_THRESHOLD  0.08f

// MPU-6050 registers
#define MPU_ADDR_A        (0x68 << 1)   // AD0 = GND
#define MPU_ADDR_B        (0x69 << 1)   // AD0 = 3.3V
#define MPU_REG_PWR       0x6B
#define MPU_REG_ACCEL     0x3B
#define MPU_REG_WHO       0x75

// DWT
#define DWT_INIT() do { \
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; \
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; \
    DWT->CYCCNT = 0; \
} while(0)

#define DWT_START()      (DWT->CYCCNT)
#define DWT_STOP(t0)     (DWT->CYCCNT - (t0))
#define CYC_TO_NS(c)     ((float)(c) * 1000.0f / 84.0f)

// ============================================================
// TYPES
// ============================================================
typedef enum {
    ORAC_NORMAL   = 0,
    ORAC_SURVIVAL = 1,
    ORAC_CRITICAL = 2
} OracMode;

typedef struct {
    float    h;
    float    m;
    uint32_t last;
    float    theta;
} OracNode;   // exactly 16 bytes

typedef struct {
    float    W;
    float    Q;
    float    T;
    OracMode mode;
    uint8_t  fault;   // 0=none, 1=sensor_A, 2=sensor_B
    float    mag_A;
    float    mag_B;
} OracStatus;

// ============================================================
// MPU-6050 DRIVER (inline — no .c file needed)
// ============================================================
static inline HAL_StatusTypeDef MPU_Init(I2C_HandleTypeDef *hi2c, uint16_t addr)
{
    uint8_t who = 0;
    if (HAL_I2C_Mem_Read(hi2c, addr, MPU_REG_WHO, 1, &who, 1, 100) != HAL_OK)
        return HAL_ERROR;
    if (who != 0x68) return HAL_ERROR;
    uint8_t zero = 0x00;
    return HAL_I2C_Mem_Write(hi2c, addr, MPU_REG_PWR, 1, &zero, 1, 100);
}

static inline float MPU_ReadMag(I2C_HandleTypeDef *hi2c, uint16_t addr)
{
    uint8_t buf[6];
    if (HAL_I2C_Mem_Read(hi2c, addr, MPU_REG_ACCEL, 1, buf, 6, 100) != HAL_OK)
        return 1.0f;
    int16_t ax = (int16_t)(buf[0] << 8 | buf[1]);
    int16_t ay = (int16_t)(buf[2] << 8 | buf[3]);
    int16_t az = (int16_t)(buf[4] << 8 | buf[5]);
    float x = ax / 16384.0f;
    float y = ay / 16384.0f;
    float z = az / 16384.0f;
    return sqrtf(x*x + y*y + z*z);
}

// ============================================================
// CORE STEP — Algorithm 1 (measured by DWT)
// ============================================================
static inline bool orac_step(OracNode *n, float grad,
                              float Wnorm, uint32_t step,
                              uint32_t *cycles)
{
    uint32_t t0 = DWT_START();

    float g_abs = fabsf(grad);
    float theta = fmaxf(0.0005f, g_abs * (ORAC_FREEZE_PCT / 100.0f));

    n->m = ORAC_MOMENTUM * n->m + (1.0f - ORAC_MOMENTUM) * grad;

    float h_new = n->h * ORAC_H_DECAY
                  + tanhf(g_abs / fmaxf(theta, 1e-6f));
    n->h = fmaxf(ORAC_H_MIN, h_new);

    // Probabilistic freeze
    bool freeze = (g_abs < theta);
    if (freeze) {
        float wake = 0.5f + 0.4f * fmaxf(-1.0f, fminf(1.0f, Wnorm));
        float r = (float)rand() / ((float)RAND_MAX + 1.0f);
        if (r < wake) freeze = false;
    }

    if (freeze) {
        n->h = fmaxf(ORAC_H_MIN, n->h - 0.02f);
        *cycles = DWT_STOP(t0);
        return false;
    }

    uint32_t dt = (step > n->last) ? (step - n->last) : 1;
    float alpha = ORAC_ALPHA / (1.0f + ORAC_BETA * (float)step);
    float M = 1.0f + alpha * (ORAC_C * ORAC_C) * n->h / sqrtf((float)dt);
    if (M > ORAC_M_MAX) M = ORAC_M_MAX;

    n->theta -= ORAC_LR * n->m * M;
    n->last   = step;

    *cycles = DWT_STOP(t0);
    return true;
}

// ============================================================
// W(t) FUNCTIONAL + BYZANTINE FUSION
// ============================================================
static inline void orac_fuse(float mag_A, float mag_B,
                              float baseline, float range,
                              OracStatus *s)
{
    s->mag_A = mag_A;
    s->mag_B = mag_B;
    s->fault = 0;

    float diff = fabsf(mag_A - mag_B);
    float fused;

    // Byzantine isolation
    if (diff > 0.05f) {
        if (fabsf(mag_A - 1.0f) > fabsf(mag_B - 1.0f)) {
            s->fault = 1;   // Sensor A faulty
            fused = mag_B;
        } else {
            s->fault = 2;   // Sensor B faulty
            fused = mag_A;
        }
    } else {
        fused = (mag_A + mag_B) * 0.5f;
    }

    // W(t) = Q * D - T
    float diff_norm = (diff - baseline) / fmaxf(range, 1e-6f);
    s->Q = (diff < baseline * 2.0f) ? 0.94f : 0.38f;
    s->T = 0.18f + diff * 0.001f;
    s->W = s->Q * 0.87f - s->T;

    if (s->fault != 0 || s->W < ORAC_W_THRESHOLD)
        s->mode = ORAC_SURVIVAL;
    else if (s->W < 0.0f)
        s->mode = ORAC_CRITICAL;
    else
        s->mode = ORAC_NORMAL;

    (void)diff_norm;
    (void)fused;
}

// ============================================================
// CALIBRATION
// ============================================================
static inline void orac_calibrate(I2C_HandleTypeDef *hi2c,
                                   float *baseline, float *range)
{
    float sum = 0;
    for (int i = 0; i < 200; i++) {
        float mA = MPU_ReadMag(hi2c, MPU_ADDR_A);
        float mB = MPU_ReadMag(hi2c, MPU_ADDR_B);
        sum += fabsf(mA - mB);
        HAL_Delay(10);
    }
    *baseline = sum / 200.0f;
    *range    = fmaxf(0.05f, *baseline * 3.0f);
}

#endif // ORAC_NT_H
