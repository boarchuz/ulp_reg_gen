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

static const char* TAG = "ulp_reg";

// 10 bits corresponding to the register value for a reg_wr are taken up by the register address, leaving 6 variable bits

#define ULP_TEST_WR_REG SENS_ULP_CP_SLEEP_CYC1_REG
#define ULP_TEST_WR_REG_HIGH 5
#define ULP_TEST_WR_REG_LOW  ((ULP_TEST_WR_REG_HIGH > 5) ? (ULP_TEST_WR_REG_HIGH - 5) : 0)

// 6 bit write to the above register[5:0] (val=0 is a placeholder)
#define I_ULP_TEST_INSN() I_WR_REG(ULP_TEST_WR_REG, ULP_TEST_WR_REG_LOW, ULP_TEST_WR_REG_HIGH, 0)

typedef struct {
    uint32_t val : 16;
    uint32_t ptr_reg : 2;
    uint32_t unused : 3;
    uint32_t pc : 11;
} ulp_stored_word_t;

_Static_assert(sizeof(ulp_stored_word_t) == sizeof(ulp_insn_t), "incompatible struct");


void ulp_generate_test_insn()
{
    memset(RTC_SLOW_MEM, 0, CONFIG_ESP32_ULP_COPROC_RESERVE_MEM);

    // This is what we want the result of our ST instruction to produce:
    ulp_insn_t desired_instruction = I_ULP_TEST_INSN();

    // This will lets us inspect what is needed to produce that result:
    ulp_stored_word_t* required_st_output = (ulp_stored_word_t*)&desired_instruction;

    ESP_LOGI(TAG, "ST (0x%08X) must be executed at PC %u with R%u as PTR and value %u", desired_instruction.instruction, required_st_output->pc, required_st_output->ptr_reg, required_st_output->val);
    if(required_st_output->unused != 0)
    {
        ESP_LOGE(TAG, "Desired instruction requires zero'd bits so be set. If wr_reg, check low.");
        abort();
    }

    uint8_t reg_temp = required_st_output->ptr_reg == R0 ? R1 : R0;
    uint16_t register_address = desired_instruction.instruction & 0x3FF;
    const ulp_insn_t st_program[] = {
        I_MOVI(reg_temp, register_address), // Set the register address (and val = 0)
        I_MOVI(required_st_output->ptr_reg, 0),     // Set ptr_reg to a known value so we can ST to RTC_SLOW_MEM[0] in the next instruction
        I_ST(reg_temp, required_st_output->ptr_reg, 0), // Create the instruction at RTC_SLOW_MEM[0]
        I_END(),
        I_HALT(), // Done
    };

    uint32_t st_program_entry = required_st_output->pc - 2; // There are 2 instructions before I_ST in the above program so align I_ST with the correct PC
    size_t st_program_size = sizeof(st_program) / sizeof(st_program)[0];
    ESP_ERROR_CHECK( ulp_process_macros_and_load(st_program_entry, st_program, &st_program_size) ); // Will abort if instruction is beyond reserved region

    uint32_t insn_before = RTC_SLOW_MEM[0];

    ESP_ERROR_CHECK( ulp_run(st_program_entry) );
    // Let ULP generate the instruction
    vTaskDelay(100 / portTICK_PERIOD_MS);

    ulp_insn_t *output_instruction = (ulp_insn_t *)&RTC_SLOW_MEM[0];
    ESP_LOGI(TAG, "Generated instruction: 0x%08X -> 0x%08X", insn_before, output_instruction->instruction);

    if(
        output_instruction->wr_reg.opcode != desired_instruction.wr_reg.opcode ||
        output_instruction->wr_reg.high != desired_instruction.wr_reg.high ||
        output_instruction->wr_reg.low != desired_instruction.wr_reg.low ||
        output_instruction->wr_reg.addr != desired_instruction.wr_reg.addr ||
        output_instruction->wr_reg.periph_sel != desired_instruction.wr_reg.periph_sel
    )
    {
        ESP_LOGE(TAG, "Incorrect instruction generated!");
        abort();
    }

    // The generated instruction is at [0]. Set this just after it so it will end/halt after executing the generated instruction.
    const ulp_insn_t end_program[] = {
        I_END(),
        I_HALT(),
    };
    size_t end_program_size = sizeof(end_program) / sizeof(end_program)[0];
    ESP_ERROR_CHECK( ulp_process_macros_and_load(1, end_program, &end_program_size) );

    uint32_t reg_before = REG_READ(ULP_TEST_WR_REG);
    ESP_ERROR_CHECK( ulp_run(0) );
    // Let ULP execute the generated instruction
    vTaskDelay(100 / portTICK_PERIOD_MS);
    uint32_t reg_after = REG_READ(ULP_TEST_WR_REG);
    ESP_LOGI(TAG, "Result: 0x%08X -> 0x%08X", reg_before, reg_after);

    if(reg_after == reg_before)
    {
        // This is a simple check just to see if the register changed at all
        // Doesn't necessarily mean it worked/failed but better than nothing
        abort();
    }
}

void ulp_generate_loop()
{
    memset(RTC_SLOW_MEM, 0, CONFIG_ESP32_ULP_COPROC_RESERVE_MEM);

    ulp_insn_t desired_instruction = I_ULP_TEST_INSN();
    ulp_stored_word_t* required_st_output = (ulp_stored_word_t*)&desired_instruction;

    uint8_t reg_temp = required_st_output->ptr_reg == R0 ? R1 : R0;
    uint8_t reg_scratch = required_st_output->ptr_reg == R2 ? R3 : R2;
    uint16_t register_address = desired_instruction.instruction & 0x3FF;
    const ulp_insn_t st_program[] = {
        I_LSHI(reg_scratch, reg_temp, 10),
        I_ORI(reg_scratch, reg_scratch, register_address),
        I_MOVI(required_st_output->ptr_reg, 0),
        I_ST(reg_scratch, required_st_output->ptr_reg, 1), // Create the instruction at RTC_SLOW_MEM[1]
        I_BXI(1), // Branch to the instruction
    };

    uint32_t st_program_entry = required_st_output->pc - 3;
    size_t st_program_size = sizeof(st_program) / sizeof(st_program)[0];
    ESP_ERROR_CHECK( ulp_process_macros_and_load(st_program_entry, st_program, &st_program_size) );

    const ulp_insn_t loop_program[] = {
        I_BXI(st_program_entry),
            I_HALT(), // Placeholder, this is where the subroutine will generate instruction, then return here to execute it
        I_ADDI(reg_temp, reg_temp, 1), // Increment value for next loop
        I_HALT(),
    };
    size_t loop_program_size = sizeof(loop_program) / sizeof(loop_program)[0];
    ESP_ERROR_CHECK( ulp_process_macros_and_load(0, loop_program, &loop_program_size) );

    ESP_ERROR_CHECK( ulp_set_wakeup_period(0, 1 * 1000 * 1000) );
    ESP_ERROR_CHECK( ulp_run(0) );

    uint32_t current_val = REG_READ(ULP_TEST_WR_REG);
    for(;;)
    {
        uint32_t new_val = REG_READ(ULP_TEST_WR_REG);
        if(new_val != current_val)
        {
            current_val = new_val;
            ESP_LOGI(TAG, "INSN: 0x%08X, Reg: 0x%08X", RTC_SLOW_MEM[1], current_val);
        }
        vTaskDelay(1);
    }
}

void app_main(void)
{
    // runs once to check
    ulp_generate_test_insn();
    // loops, incrementing each time
    ulp_generate_loop();
    vTaskDelete(NULL);   
}
