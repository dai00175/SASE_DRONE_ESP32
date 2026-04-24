#include "flight_control.hpp"

#include <array>

#include "board_config.hpp"
#include "driver/gptimer.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "motors.hpp"
#include "pid_controller.hpp"
#include "shared_state.hpp"

namespace flight_control
{

namespace
{

gptimer_handle_t g_flight_timer = nullptr;

std::array<int32_t, 4> motors_off_outputs()
{
    return {
        kBoardConfig.pwm_min_us,
        kBoardConfig.pwm_min_us,
        kBoardConfig.pwm_min_us,
        kBoardConfig.pwm_min_us,
    };
}

void apply_idle_setpoint(control_setpoint_t &setpoint)
{
    setpoint.state = DroneState::IDLE;
    setpoint.throttle_us = kBoardConfig.pwm_min_us;
    setpoint.roll_deg = 0.0f;
    setpoint.pitch_deg = 0.0f;
    setpoint.yaw_deg = 0.0f;
}

bool IRAM_ATTR flight_timer_alarm_cb(gptimer_handle_t, const gptimer_alarm_event_data_t *, void *user_ctx)
{
    BaseType_t task_woken = pdFALSE;
    vTaskNotifyGiveFromISR(static_cast<TaskHandle_t>(user_ctx), &task_woken);
    return task_woken == pdTRUE;
}

void process_command(const command_message_t &command)
{
    control_setpoint_t setpoint = app::snapshot_control_setpoint();
    if (command.mission_id != 0)
    {
        setpoint.mission_id = command.mission_id;
    }

    switch (command.action)
    {
    case CommandAction::TAKEOFF:
        setpoint.state = DroneState::TAKEOFF;
        break;
    case CommandAction::LAND:
        apply_idle_setpoint(setpoint);
        break;
    case CommandAction::CANCEL:
        setpoint.state = DroneState::USERCNTRL;
        break;
    case CommandAction::PROGRAM_CONTROL:
        setpoint.state = DroneState::PRGMCNTRL;
        break;
    case CommandAction::USER_CONTROL:
        setpoint.state = DroneState::USERCNTRL;
        break;
    case CommandAction::SETPOINT_UPDATE:
    case CommandAction::NONE:
        break;
    }

    if (command.has_throttle)
    {
        setpoint.throttle_us = app::clamp_value(command.throttle_us, kBoardConfig.pwm_min_us, kBoardConfig.pwm_max_us);
    }
    if (command.has_roll)
    {
        setpoint.roll_deg = command.roll_deg;
    }
    if (command.has_pitch)
    {
        setpoint.pitch_deg = command.pitch_deg;
    }
    if (command.has_yaw)
    {
        setpoint.yaw_deg = command.yaw_deg;
    }

    if (setpoint.state == DroneState::IDLE)
    {
        apply_idle_setpoint(setpoint);
        pid::reset();
    }

    setpoint.command_timestamp_us = esp_timer_get_time();
    app::update_control_setpoint(setpoint);
}

} // namespace

void flight_task(void *)
{
    int64_t last_loop_us = 0;

    while (true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        const int64_t now_us = esp_timer_get_time();
        const float dt = last_loop_us == 0 ? (static_cast<float>(kBoardConfig.flight_loop_period_us) / 1000000.0f)
                                           : static_cast<float>(now_us - last_loop_us) / 1000000.0f;
        const int64_t loop_period_us = last_loop_us == 0 ? kBoardConfig.flight_loop_period_us : now_us - last_loop_us;
        last_loop_us = now_us;

        const imu_snapshot_t imu_snapshot = app::snapshot_imu();
        control_setpoint_t setpoint = app::snapshot_control_setpoint();
        if (setpoint.state == DroneState::IDLE)
        {
            apply_idle_setpoint(setpoint);
            pid::reset();
            motors::apply_outputs(motors_off_outputs());
            app::update_loop_runtime(loop_period_us, 0.0f, 0.0f, 0.0f);
            continue;
        }

        const pid::Output pid_output = pid::compute(setpoint, imu_snapshot, dt);
        std::array<int32_t, 4> motor_outputs = pid::mix_outputs(setpoint.throttle_us, pid_output);

        motors::apply_outputs(motor_outputs);
        app::update_loop_runtime(loop_period_us, pid_output.roll, pid_output.pitch, pid_output.yaw);
    }
}

void command_task(void *)
{
    command_message_t command = {};
    while (true)
    {
        if (xQueueReceive(app::command_queue(), &command, portMAX_DELAY) == pdTRUE)
        {
            process_command(command);
        }
    }
}

esp_err_t init_flight_timer()
{
    gptimer_config_t timer_config = {};
    timer_config.clk_src = GPTIMER_CLK_SRC_DEFAULT;
    timer_config.direction = GPTIMER_COUNT_UP;
    timer_config.resolution_hz = 1000000;
    ESP_RETURN_ON_ERROR(gptimer_new_timer(&timer_config, &g_flight_timer), app::kTag, "gptimer_new_timer failed");

    gptimer_event_callbacks_t callbacks = {};
    callbacks.on_alarm = flight_timer_alarm_cb;
    ESP_RETURN_ON_ERROR(gptimer_register_event_callbacks(g_flight_timer, &callbacks, app::flight_task_handle()),
                        app::kTag, "gptimer_register_event_callbacks failed");

    gptimer_alarm_config_t alarm_config = {};
    alarm_config.alarm_count = kBoardConfig.flight_loop_period_us;
    alarm_config.reload_count = 0;
    alarm_config.flags.auto_reload_on_alarm = 1;
    ESP_RETURN_ON_ERROR(gptimer_set_alarm_action(g_flight_timer, &alarm_config), app::kTag,
                        "gptimer_set_alarm_action failed");
    ESP_RETURN_ON_ERROR(gptimer_enable(g_flight_timer), app::kTag, "gptimer_enable failed");
    return gptimer_start(g_flight_timer);
}

} // namespace flight_control
