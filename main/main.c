#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "driver/rtc_io.h"
#include "esp32/ulp.h"
#include "soc/rtc_io_reg.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"

#include "sdkconfig.h"

static const char* TAG = "ulp_reggp";

#define HULP_REGWR_VAL_SHIFT 10
#define HULP_REGWR_IMM_VAL(register, value) \
    (((value) << HULP_REGWR_VAL_SHIFT) | (SOC_REG_TO_ULP_PERIPH_SEL(register) << 8) | ((register & 0xFF) / sizeof(uint32_t)))

/**
 * Each combination of high_bit and low_bit require a few words at a specific PC
 *  
 * Due to limitations of this method, low_bit may only be 0, 8, 16, or 24
 */
#define HULP_REGWR_WORK_OFFSET(low_bit, high_bit) \
    (128 + (high_bit) * 4 + (low_bit) / 8)

/**
 * At each of these PCs, this is the number of instructions required to implement the register write
 */
#define HULP_REGWR_WORK_COUNT 3

#define HULP_REGWR_WORK_AREA_START (HULP_REGWR_WORK_OFFSET(0,0))
#define HULP_REGWR_WORK_AREA_END (HULP_REGWR_WORK_OFFSET(24,31) + HULP_REGWR_WORK_COUNT)

#define HULP_WR_REG_GEN_SAFE_ENTRY 1024
#define HULP_WR_REG_GEN_ENTRY 831

#define ULP_WR_TEST_REG         SENS_ULP_CP_SLEEP_CYC2_REG
#define ULP_WR_TEST_REG_LOW     0
#define ULP_WR_TEST_REG_HIGH    31
#define ULP_WR_TEST_REG_VAL     11     

static void load_basic_test()
{
    const ulp_insn_t program[] = {
        I_MOVI(R2, HULP_REGWR_IMM_VAL(ULP_WR_TEST_REG, ULP_WR_TEST_REG_VAL)),
        I_MOVI(R0, HULP_REGWR_WORK_OFFSET(ULP_WR_TEST_REG_LOW, ULP_WR_TEST_REG_HIGH)),
        M_MOVL(R3, 1),
        I_BXI(HULP_WR_REG_GEN_SAFE_ENTRY),
    M_LABEL(1),
        I_END(),
        I_HALT(),
    };

    size_t program_size = sizeof(program) / sizeof(program[0]);
    ESP_ERROR_CHECK( ulp_process_macros_and_load(0, program, &program_size) );

    ESP_ERROR_CHECK( ulp_set_wakeup_period(0, 1 * 1000 * 1000) );
    ESP_ERROR_CHECK( ulp_run(0) );

    vTaskDelay(100 / portTICK_PERIOD_MS);

    uint32_t get_reg = REG_READ(ULP_WR_TEST_REG);
    uint32_t check_val = (get_reg >> ULP_WR_TEST_REG_LOW) & ~(~0 << (ULP_WR_TEST_REG_HIGH - ULP_WR_TEST_REG_LOW + 1));
    ESP_LOGI(TAG, "Reg: %u", get_reg);
    if(check_val != ULP_WR_TEST_REG_VAL)
    {
        ESP_LOGE(TAG, "FAILED. Got %u, expected %u", check_val, ULP_WR_TEST_REG_VAL);
        vTaskDelay(portMAX_DELAY);
    }
}

esp_err_t load_branch_write_program()
{
    const ulp_insn_t program_1025[] = {
        I_MOVI(R1, R3), // Constant == 3
        I_ST(R1, R0, 2), // MUST BE AT 1025
        I_BXI(HULP_WR_REG_GEN_ENTRY),
    };
    uint32_t entry_1025 = 1025-1;
    size_t program_1025_size = sizeof(program_1025) / sizeof(program_1025[0]);
    ESP_ERROR_CHECK( ulp_process_macros_and_load(entry_1025, program_1025, &program_1025_size) );
    return ESP_OK;
}

esp_err_t load_wrreg_st_gen_program()
{
    const ulp_insn_t program_832[] = {
        I_MOVI(R1, (1 << 10) | (R0 << 2) | (R2 << 0)),
        I_ST(R1, R0, 0), // MUST BE AT 832
        I_BXR(R0),
    };
    uint32_t entry_832 = HULP_WR_REG_GEN_ENTRY;
    size_t program_832_size = sizeof(program_832) / sizeof(program_832[0]);
    ESP_ERROR_CHECK( ulp_process_macros_and_load(entry_832, program_832, &program_832_size) );
    return ESP_OK;
}

esp_err_t prepare_work_area(uint32_t offset)
{
    const ulp_insn_t r3_return = I_BXR(R3);
    RTC_SLOW_MEM[offset + 2] = r3_return.instruction;
    return ESP_OK;
}

void load_ranges_test()
{
    ulp_insn_t *movi_addr_and_val = &RTC_SLOW_MEM[0];
    ulp_insn_t *movi_work_area = &RTC_SLOW_MEM[1];

    uint8_t low_bits[] = {0,8,16,24};
    for(int l = 0; l < sizeof(low_bits)/sizeof(low_bits[0]); ++l)
    {
        uint8_t low = low_bits[l];
        for(int high = low; high < 32; ++high)
        {
            // Get a random value of the required number of bits
            uint8_t val = (esp_random() >> (31 - (high-low))) & 0x3F;

            *movi_addr_and_val = (ulp_insn_t)I_MOVI(R2, HULP_REGWR_IMM_VAL(ULP_WR_TEST_REG, val));
            *movi_work_area = (ulp_insn_t)I_MOVI(R0, HULP_REGWR_WORK_OFFSET(low, high));

            REG_WRITE(ULP_WR_TEST_REG, 0);
            ESP_ERROR_CHECK( ulp_run(0) );

            vTaskDelay(100 / portTICK_PERIOD_MS);
            if(REG_GET_FIELD(RTC_CNTL_STATE0_REG, RTC_CNTL_ULP_CP_SLP_TIMER_EN))
            {
                ESP_LOGW(TAG, "ULP still running?");
                vTaskDelay(portMAX_DELAY);
            }

            uint32_t get_reg = REG_READ(ULP_WR_TEST_REG);
            uint32_t check_val = (get_reg >> low) & ~(~0ULL << (high - low + 1));
            if(check_val != val)
            {
                ESP_LOGE(TAG, "FAILED [%u:%u]. Got %u, expected %u", high, low, check_val, ULP_WR_TEST_REG_VAL);
                vTaskDelay(portMAX_DELAY);
            }

            ESP_LOGI(TAG, "OK @ [%u:%u] 0x%08X , %u %u", high, low, get_reg, check_val, val);
        }
    }
}

static void load_ulp_increments()
{
    const ulp_insn_t program[] = {
        I_MOVI(R2, HULP_REGWR_IMM_VAL(ULP_WR_TEST_REG, 0)),
        I_MOVI(R0, HULP_REGWR_WORK_OFFSET(0, 5)),
        M_MOVL(R3, 1),
        I_WR_REG(SENS_SAR_START_FORCE_REG, SENS_PC_INIT_S + 0, SENS_PC_INIT_S +  7, 4),
        
        I_ADDI(R2, R2, 1 << HULP_REGWR_VAL_SHIFT),
        I_BXI(HULP_WR_REG_GEN_SAFE_ENTRY),
    M_LABEL(1),
        I_HALT(),
    };

    size_t program_size = sizeof(program) / sizeof(program[0]);
    ESP_ERROR_CHECK( ulp_process_macros_and_load(0, program, &program_size) );

    ESP_ERROR_CHECK( ulp_set_wakeup_period(0, 1 * 1000 * 1000) );

    REG_WRITE(ULP_WR_TEST_REG, 0);
    ESP_ERROR_CHECK( ulp_run(0) );

    uint32_t reg_current = 0;
    for(;;)
    {
        uint32_t reg_new = REG_READ(ULP_WR_TEST_REG);
        if(reg_new != reg_current)
        {
            reg_current = reg_new;
            ESP_LOGI(TAG, "ULP Wrote: %u", reg_current);
        }
        vTaskDelay(1);
    }
}

void app_main(void)
{
    load_branch_write_program();
    load_wrreg_st_gen_program();
    load_basic_test();
    load_ranges_test();
    load_ulp_increments();
    vTaskDelete(NULL);   
}