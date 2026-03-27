/**
 * ORAC-NT v5.5 — STM32F401 + ST-Link V2
 * CubeIDE Project
 *
 * Pins:
 *   PB6 = I2C1 SCL
 *   PB7 = I2C1 SDA
 *   PC13 = Built-in LED (active LOW on Black Pill)
 *
 * UART2 (PA2=TX, PA3=RX) @ 115200 for Serial Monitor
 */

#include "main.h"
#include "orac_nt.h"
#include <stdio.h>
#include <string.h>

// ============================================================
// HANDLES (генерирани от CubeMX)
// ============================================================
extern I2C_HandleTypeDef  hi2c1;
extern UART_HandleTypeDef huart2;

// ============================================================
// PRINTF → UART
// ============================================================
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 10);
    return ch;
}

// ============================================================
// ORAC STATE
// ============================================================
static OracNode   node     = {0.5f, 0.0f, 0, 0.0f};
static OracStatus status;

static float baseline = 0.0f;
static float range    = 0.0f;

static uint32_t count        = 0;
static uint64_t total_frozen = 0;
static uint64_t total_active = 0;
static uint64_t sum_cycles   = 0;
static uint32_t min_cycles   = 0xFFFFFFFF;
static uint32_t max_cycles   = 0;

// ============================================================
// MAIN
// ============================================================
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART2_UART_Init();
    DWT_INIT();

    srand(HAL_GetTick());
    HAL_Delay(500);

    printf("\r\n========================================\r\n");
    printf("  ORAC-NT v5.5\r\n");
    printf("  STM32F401 @ 84MHz + ST-Link V2\r\n");
    printf("  16 bytes RAM | ~35ns latency\r\n");
    printf("========================================\r\n\r\n");

    // --- Init MPU-6050 ---
    printf("Sensor A (0x68): ");
    if (MPU_Init(&hi2c1, MPU_ADDR_A) == HAL_OK) printf("OK\r\n");
    else                                          printf("FAIL - check AD0=GND\r\n");

    printf("Sensor B (0x69): ");
    if (MPU_Init(&hi2c1, MPU_ADDR_B) == HAL_OK) printf("OK\r\n");
    else                                          printf("FAIL - check AD0=3.3V\r\n");

    HAL_Delay(100);

    // --- Calibration ---
    printf("\r\nCalibration (2s) — keep sensors still...\r\n");
    orac_calibrate(&hi2c1, &baseline, &range);
    printf("Baseline : %.4f g\r\n", baseline);
    printf("Range    : %.4f g\r\n\r\n", range);

    printf("Running — no delay in loop...\r\n\r\n");
    printf("%-8s | %-7s | %-6s | %-8s | %-5s | %-6s | %s\r\n",
           "Cycle", "Avg ns", "W", "Mode", "Fault", "Saved", "h");
    printf("---------|---------|--------|----------|-------|--------|------\r\n");

    // ============================================================
    // MAIN LOOP
    // ============================================================
    while (1)
    {
        // 1. Read sensors
        float mag_A = MPU_ReadMag(&hi2c1, MPU_ADDR_A);
        float mag_B = MPU_ReadMag(&hi2c1, MPU_ADDR_B);

        // 2. Byzantine fusion + W(t)
        orac_fuse(mag_A, mag_B, baseline, range, &status);

        // Normalized W for freeze logic
        float W_norm = (status.W - 0.5f) * 2.0f;

        // Gradient from sensor difference
        float grad = (mag_A - mag_B);

        // 3. ORAC-NT core step — DWT measured
        uint32_t cyc = 0;
        bool active = orac_step(&node, grad, W_norm, count, &cyc);

        // 4. Statistics
        sum_cycles += cyc;
        if (cyc < min_cycles) min_cycles = cyc;
        if (cyc > max_cycles) max_cycles = cyc;
        if (active) total_active++;
        else        total_frozen++;

        // 5. LED feedback
        //    NORMAL   = LED off
        //    SURVIVAL = LED blink fast
        //    CRITICAL = LED on solid
        if (status.mode == ORAC_NORMAL)
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        else if (status.mode == ORAC_SURVIVAL)
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        else
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

        // 6. Telemetry every 1000 cycles
        if (count % 1000 == 0 && count > 0) {

            float avg_ns = CYC_TO_NS((float)sum_cycles / (float)count);
            float saved  = 100.0f * (float)total_frozen
                           / (float)(total_frozen + total_active);

            const char *mode_str =
                (status.mode == ORAC_NORMAL)   ? "NORMAL" :
                (status.mode == ORAC_SURVIVAL) ? "SURVIV" : "CRITIC";

            const char *fault_str =
                (status.fault == 0) ? "none" :
                (status.fault == 1) ? "A_bad" : "B_bad";

            printf("%-8lu | %-7.1f | %-6.3f | %-8s | %-5s | %-5.1f%% | %.3f\r\n",
                   count, avg_ns, status.W,
                   mode_str, fault_str, saved, node.h);
        }

        // 7. Summary every 10,000 cycles
        if (count % 10000 == 0 && count > 0) {
            float avg_ns = CYC_TO_NS((float)sum_cycles / (float)count);
            float saved  = 100.0f * (float)total_frozen
                           / (float)(total_frozen + total_active);

            printf("\r\n========================================\r\n");
            printf("  SUMMARY @ %lu cycles\r\n", count);
            printf("  Avg latency : %.1f ns\r\n", avg_ns);
            printf("  Min latency : %.1f ns\r\n", CYC_TO_NS((float)min_cycles));
            printf("  Max latency : %.1f ns\r\n", CYC_TO_NS((float)max_cycles));
            printf("  Energy saved: %.1f %%\r\n", saved);
            printf("  W(t)        : %.4f\r\n",    status.W);
            printf("  h potential : %.4f\r\n",    node.h);
            printf("  theta       : %.4f\r\n",    node.theta);
            printf("  Frozen      : %lu\r\n",     (uint32_t)total_frozen);
            printf("  Active      : %lu\r\n",     (uint32_t)total_active);
            printf("========================================\r\n\r\n");
        }

        count++;
        // NO delay — maximum throughput
    }
}
