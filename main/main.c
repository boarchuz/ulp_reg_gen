#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "driver/rtc_io.h"
#include "esp32/ulp.h"
#include "soc/rtc_io_reg.h"
#include "soc/rtc_cntl_reg.h"

#include "sdkconfig.h"

static const char* TAG = "ulp_reg";

// Low bit of reg write instruction must be 0,8,16,24 as LSB 3 bits are always set to 0 by ST instruction
// As this example sets RTCIO output value reg, and they begin at 14 (RTC_GPIO_OUT_DATA_W1TS_S), low=16 and low=24 are options, ie. RTCIO 2 or 10
// RTCIO 2 (GPIO 38) has no output capability, so RTCIO 10 (GPIO 4) is selected

#define GPIO_NUM 4
#define RTCIO_NUM 10

#define I_SET_PIN_HIGH() I_WR_REG_BIT(RTC_GPIO_OUT_REG, (RTC_GPIO_OUT_DATA_S + (RTCIO_NUM)), 1)

#define I_DESIRED_INSTRUCTION   I_SET_PIN_HIGH

typedef struct {
    uint32_t val : 16;
    uint32_t ptr_reg : 2;
    uint32_t unused : 3;
    uint32_t pc : 11;
} ulp_stored_word_t;

_Static_assert(sizeof(ulp_stored_word_t) == sizeof(ulp_insn_t), "incompatible struct");

void ulp_generate_insn()
{
    memset(RTC_SLOW_MEM, 0, CONFIG_ULP_COPROC_RESERVE_MEM);

    // This is what we want the result of our ST instruction to produce:
    ulp_insn_t desired_instruction = I_DESIRED_INSTRUCTION();

    // This will lets us inspect what is needed to produce that result:
    ulp_stored_word_t* required_st_output = (ulp_stored_word_t*)&desired_instruction;

    ESP_LOGI(TAG, "ST (0x%08X) must be executed at PC %u with R%u as PTR and value %u", desired_instruction.instruction, required_st_output->pc, required_st_output->ptr_reg, required_st_output->val);
    if(required_st_output->unused != 0)
    {
        ESP_LOGE(TAG, "Desired instruction requires zero'd bits so be set. If wr_reg, check low.");
        abort();
    }

    uint8_t reg_temp = required_st_output->ptr_reg == R0 ? R1 : R0;
    const ulp_insn_t st_program[] = {
        I_MOVI(reg_temp, required_st_output->val),  // Prepare the value
        I_MOVI(required_st_output->ptr_reg, 0),     // Set ptr_reg to a known value so we can ST to RTC_SLOW_MEM[0] in the next instruction
        I_ST(reg_temp, required_st_output->ptr_reg, 0), // Create the instruction at RTC_SLOW_MEM[0]
        I_END(),
        I_HALT(), // Done
    };

    uint32_t st_program_entry = required_st_output->pc - 2;
    size_t st_program_size = sizeof(st_program) / sizeof(st_program)[0];
    ESP_ERROR_CHECK( ulp_process_macros_and_load(st_program_entry, st_program, &st_program_size) ); // Will abort if instruction is beyond reserved region

    uint32_t insn_before = RTC_SLOW_MEM[0];

    ESP_LOGI(TAG, "Pin %d output low. Starting ULP in 3 seconds...", GPIO_NUM);
    vTaskDelay(3000 / portTICK_PERIOD_MS);

    ESP_ERROR_CHECK( ulp_run(st_program_entry) );
    // Let ULP generate the instruction
    vTaskDelay(100 / portTICK_PERIOD_MS);

    ulp_insn_t *output_instruction = (ulp_insn_t *)&RTC_SLOW_MEM[0];
    ESP_LOGI(TAG, "Generated instruction: 0x%08X -> 0x%08X", insn_before, output_instruction->instruction);

    if(memcmp(output_instruction, &desired_instruction, sizeof(*output_instruction)) != 0)
    {
        ESP_LOGE(TAG, "Incorrect instruction generated! 0x%08X != 0x%08X", output_instruction->instruction, desired_instruction.instruction);
        abort();
    }

    // The generated instruction is at [0]. Set this just after it so it will end/halt after executing the generated instruction.
    const ulp_insn_t end_program[] = {
        I_END(),
        I_HALT(),
    };
    size_t end_program_size = sizeof(end_program) / sizeof(end_program)[0];
    ESP_ERROR_CHECK( ulp_process_macros_and_load(1, end_program, &end_program_size) );

    // Prepare the pin so we can see if it works. Set output low. If successful, it will be set high by the ULP.
    ESP_ERROR_CHECK( rtc_gpio_init(GPIO_NUM) );
    ESP_ERROR_CHECK( rtc_gpio_set_level(GPIO_NUM, 0) );
    ESP_ERROR_CHECK( rtc_gpio_set_direction(GPIO_NUM, RTC_GPIO_MODE_OUTPUT_ONLY) );

    uint32_t reg_before = REG_READ(RTC_GPIO_OUT_REG);
    uint32_t reg_expected = reg_before | (1 << (RTC_GPIO_OUT_DATA_S + (RTCIO_NUM)));
    ESP_ERROR_CHECK( ulp_run(0) );
    // Let ULP execute the generated instruction
    vTaskDelay(100 / portTICK_PERIOD_MS);
    uint32_t reg_after = REG_READ(RTC_GPIO_OUT_REG);
    if(reg_after != reg_expected)
    {
        ESP_LOGE(TAG, "Failed. Expected: 0x%08X != Actual: 0x%08X", reg_expected, reg_after);
    }
    else
    {
        ESP_LOGI(TAG, "Success! 0x%08X -> 0x%08X", reg_before, reg_after);
    }
}

void app_main(void)
{
    ulp_generate_insn();
    vTaskDelete(NULL);   
}