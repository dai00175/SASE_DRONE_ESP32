#include "motors.hpp"

#include <array>

#include "board_config.hpp"
#include "driver/mcpwm_cmpr.h"
#include "driver/mcpwm_gen.h"
#include "driver/mcpwm_oper.h"
#include "driver/mcpwm_timer.h"
#include "esp_check.h"
#include "esp_log.h"
#include "shared_state.hpp"

namespace motors
{
namespace
{

std::array<mcpwm_oper_handle_t, 2> g_motor_operators = {};
std::array<mcpwm_cmpr_handle_t, 4> g_motor_comparators = {};
std::array<mcpwm_gen_handle_t, 4> g_motor_generators = {};
mcpwm_timer_handle_t g_motor_timer = nullptr;

} // namespace

esp_err_t set_pulse_us(size_t motor_index, int pulse_us)
{
    if (motor_index >= g_motor_comparators.size() || g_motor_comparators[motor_index] == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }
    const int clamped = app::clamp_value(pulse_us, kBoardConfig.pwm_min_us, kBoardConfig.pwm_max_us);
    return mcpwm_comparator_set_compare_value(g_motor_comparators[motor_index], static_cast<uint32_t>(clamped));
}

void apply_outputs(const std::array<int32_t, 4> &outputs_us)
{
    std::array<int32_t, 4> clamped = outputs_us;
    for (size_t i = 0; i < clamped.size(); ++i)
    {
        clamped[i] = app::clamp_value(clamped[i], static_cast<int32_t>(kBoardConfig.pwm_min_us),
                                      static_cast<int32_t>(kBoardConfig.pwm_max_us));
        const esp_err_t err = set_pulse_us(i, clamped[i]);
        if (err != ESP_OK)
        {
            ESP_LOGE(app::kTag, "Failed to update motor %u output: %s", static_cast<unsigned>(i), esp_err_to_name(err));
        }
    }

    app::update_motor_runtime(clamped);
}

void apply_outputs_off()
{
    apply_outputs({
        kBoardConfig.pwm_min_us,
        kBoardConfig.pwm_min_us,
        kBoardConfig.pwm_min_us,
        kBoardConfig.pwm_min_us,
    });
}

esp_err_t init()
{
    mcpwm_timer_config_t timer_config = {};
    timer_config.group_id = 0;
    timer_config.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
    timer_config.resolution_hz = 1000000;
    timer_config.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
    timer_config.period_ticks = kBoardConfig.pwm_period_us;
    timer_config.flags.update_period_on_empty = 1;
    ESP_RETURN_ON_ERROR(mcpwm_new_timer(&timer_config, &g_motor_timer), app::kTag, "mcpwm_new_timer failed");

    for (size_t oper_index = 0; oper_index < g_motor_operators.size(); ++oper_index)
    {
        mcpwm_operator_config_t operator_config = {};
        operator_config.group_id = 0;
        ESP_RETURN_ON_ERROR(mcpwm_new_operator(&operator_config, &g_motor_operators[oper_index]), app::kTag,
                            "mcpwm_new_operator failed");
        ESP_RETURN_ON_ERROR(mcpwm_operator_connect_timer(g_motor_operators[oper_index], g_motor_timer), app::kTag,
                            "mcpwm_operator_connect_timer failed");
    }

    for (size_t motor_index = 0; motor_index < g_motor_comparators.size(); ++motor_index)
    {
        const size_t oper_index = motor_index / 2U;
        mcpwm_comparator_config_t comparator_config = {};
        comparator_config.flags.update_cmp_on_tez = 1;
        ESP_RETURN_ON_ERROR(
            mcpwm_new_comparator(g_motor_operators[oper_index], &comparator_config, &g_motor_comparators[motor_index]),
            app::kTag, "mcpwm_new_comparator failed");

        mcpwm_generator_config_t generator_config = {};
        generator_config.gen_gpio_num = kBoardConfig.motor_gpios[motor_index];
        ESP_RETURN_ON_ERROR(mcpwm_new_generator(g_motor_operators[oper_index], &generator_config,
                                                &g_motor_generators[motor_index]),
                            app::kTag, "mcpwm_new_generator failed");

        ESP_RETURN_ON_ERROR(
            mcpwm_generator_set_action_on_timer_event(
                g_motor_generators[motor_index],
                MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY,
                                             MCPWM_GEN_ACTION_HIGH)),
            app::kTag, "mcpwm_generator_set_action_on_timer_event failed");

        ESP_RETURN_ON_ERROR(
            mcpwm_generator_set_action_on_compare_event(
                g_motor_generators[motor_index],
                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, g_motor_comparators[motor_index],
                                               MCPWM_GEN_ACTION_LOW)),
            app::kTag, "mcpwm_generator_set_action_on_compare_event failed");

        ESP_RETURN_ON_ERROR(set_pulse_us(motor_index, kBoardConfig.pwm_min_us), app::kTag,
                            "initial motor pulse set failed");
    }

    ESP_RETURN_ON_ERROR(mcpwm_timer_enable(g_motor_timer), app::kTag, "mcpwm_timer_enable failed");
    ESP_RETURN_ON_ERROR(mcpwm_timer_start_stop(g_motor_timer, MCPWM_TIMER_START_NO_STOP), app::kTag,
                        "mcpwm_timer_start_stop failed");
    apply_outputs_off();
    return ESP_OK;
}

} // namespace motors
