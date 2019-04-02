#include <stdio.h>
#include <espressif/esp_wifi.h>
#include <espressif/esp_sta.h>
#include <espressif/esp_common.h>
#include <esp/uart.h>
#include <esp8266.h>
#include <FreeRTOS.h>
#include <task.h>

#include <etstimer.h>
#include <esplibs/libmain.h>

#include <homekit/homekit.h>
#include <homekit/characteristics.h>
#include <wifi_config.h>

#include <dht/dht.h>

#include "fujitsu_ac_ir.h"


#define TEMPERATURE_POLL_PERIOD 10000
#define TEMPERATURE_SENSOR_GPIO 4
#define LED_GPIO 2
#define IR_RX_GPIO 12


#define MIN(a, b) (((a) <= (b)) ? (a) : (b))
#define MAX(a, b) (((a) >= (b)) ? (a) : (b))


void thermostat_on_callback(homekit_characteristic_t *_ch, homekit_value_t on, void *context);


void reset_configuration_task() {
    printf("Resetting Wifi Config\n");

    wifi_config_reset();

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    printf("Resetting HomeKit Config\n");

    homekit_server_reset();

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    printf("Restarting\n");

    sdk_system_restart();

    vTaskDelete(NULL);
}

void reset_configuration() {
    printf("Resetting configuration\n");
    xTaskCreate(reset_configuration_task, "Reset configuration", 256, NULL, 2, NULL);
}


void led_write(bool on) {
    gpio_write(LED_GPIO, on ? 0 : 1);
}

void led_init() {
    gpio_enable(LED_GPIO, GPIO_OUTPUT);
    led_write(false);
}


fujitsu_ac_state_t ac_state;

void update_state();


void on_update(homekit_characteristic_t *ch, homekit_value_t value, void *context) {
    update_state();
}


homekit_characteristic_t current_humidity = HOMEKIT_CHARACTERISTIC_(CURRENT_RELATIVE_HUMIDITY, 0);
homekit_characteristic_t current_temperature = HOMEKIT_CHARACTERISTIC_(CURRENT_TEMPERATURE, 0);
homekit_characteristic_t target_temperature  = HOMEKIT_CHARACTERISTIC_(
    TARGET_TEMPERATURE, 22,
    .min_value = (float[]) {AC_MIN_TEMPERATURE},
    .max_value = (float[]) {AC_MAX_TEMPERATURE},
    .callback=HOMEKIT_CHARACTERISTIC_CALLBACK(on_update),
);
homekit_characteristic_t units = HOMEKIT_CHARACTERISTIC_(TEMPERATURE_DISPLAY_UNITS, 0);
homekit_characteristic_t current_state = HOMEKIT_CHARACTERISTIC_(CURRENT_HEATING_COOLING_STATE, 0);
homekit_characteristic_t target_state = HOMEKIT_CHARACTERISTIC_(
    TARGET_HEATING_COOLING_STATE, 0,
    .callback=HOMEKIT_CHARACTERISTIC_CALLBACK(on_update)
);

uint8_t fan = 0;

void fan_active_set(homekit_value_t value) {
    fan = value.bool_value;
    update_state();
}

homekit_characteristic_t fan_active = HOMEKIT_CHARACTERISTIC_(ACTIVE, 0, .setter=fan_active_set);
homekit_characteristic_t fan_rotation_speed = HOMEKIT_CHARACTERISTIC_(
    ROTATION_SPEED, 0,
    .callback=HOMEKIT_CHARACTERISTIC_CALLBACK(on_update)
);
homekit_characteristic_t fan_swing_mode = HOMEKIT_CHARACTERISTIC_(
    SWING_MODE, 0,
    .callback=HOMEKIT_CHARACTERISTIC_CALLBACK(on_update)
);


void update_state() {
    homekit_value_t new_fan_active, new_current_state;
    uint8_t state = target_state.value.int_value;

    ac_state.command = ac_cmd_turn_on;
    if (state == 1) {
        ac_state.mode = ac_mode_heat;
        new_current_state = HOMEKIT_UINT8(1);
        new_fan_active = HOMEKIT_UINT8(1);
    } else if (state == 2) {
        ac_state.mode = ac_mode_cool;
        new_current_state = HOMEKIT_UINT8(2);
        new_fan_active = HOMEKIT_UINT8(1);
    } else if (state == 3) {
        ac_state.mode = ac_mode_auto;
        if (current_temperature.value.int_value < target_temperature.value.int_value) {
            new_current_state = HOMEKIT_UINT8(1);
        } else {
            new_current_state = HOMEKIT_UINT8(2);
        }
        new_fan_active = HOMEKIT_UINT8(1);
    } else if (state == 0 && fan) {
        ac_state.mode = ac_mode_fan;
        new_current_state = HOMEKIT_UINT8(0);
        new_fan_active = HOMEKIT_UINT8(1);
    } else {
        ac_state.mode = ac_mode_auto;
        ac_state.command = ac_cmd_turn_off;
        new_current_state = HOMEKIT_UINT8(0);
        new_fan_active = HOMEKIT_UINT8(0);
    }

    uint8_t rotation_speed = fan_rotation_speed.value.int_value;
    if (rotation_speed > 75) {
        ac_state.fan = ac_fan_high;
    } else if (rotation_speed > 45) {
        ac_state.fan = ac_fan_med;
    } else if (rotation_speed > 15) {
        ac_state.fan = ac_fan_low;
    } else {
        ac_state.fan = ac_fan_quiet;
    }

    ac_state.temperature = MIN(AC_MAX_TEMPERATURE, MAX(AC_MIN_TEMPERATURE, target_temperature.value.float_value));
    ac_state.swing = fan_swing_mode.value.int_value ? ac_swing_vert : ac_swing_off;

    fujitsu_ac_ir_send(&ac_state);

    if (!homekit_value_equal(&new_current_state, &current_state.value)) {
        current_state.value = new_current_state;
        homekit_characteristic_notify(&current_state, current_state.value);
    }

    if (!homekit_value_equal(&new_fan_active, &fan_active.value)) {
        fan_active.value = new_fan_active;
        homekit_characteristic_notify(&fan_active, fan_active.value);
    }
}


void temperature_sensor_task(void *_args) {
    gpio_set_pullup(TEMPERATURE_SENSOR_GPIO, false, false);

    float humidity_value, temperature_value;
    while (1) {
        bool success = dht_read_float_data(
            DHT_TYPE_DHT11, TEMPERATURE_SENSOR_GPIO,
            &humidity_value, &temperature_value
        );
        if (success) {
            printf("Got readings: temperature %g, humidity %g\n", temperature_value, humidity_value);
            current_temperature.value = HOMEKIT_FLOAT(temperature_value);
            current_humidity.value = HOMEKIT_FLOAT(humidity_value);

            homekit_characteristic_notify(&current_temperature, current_temperature.value);
            homekit_characteristic_notify(&current_humidity, current_humidity.value);
        } else {
            printf("Couldn't read data from sensor\n");
        }

        vTaskDelay(TEMPERATURE_POLL_PERIOD / portTICK_PERIOD_MS);
    }
}


void ir_rx_task(void *_args) {
    printf("Running IR task\n");
    ir_decoder_t *decoder = fujitsu_ac_ir_make_decoder();

    fujitsu_ac_state_t state;
    while (true) {
        uint16_t size = sizeof(state);
        int r = ir_recv(decoder, 0, &state, &size);
        if (r < 0) {
            printf("Bit decoding failed\n");
            continue;
        }

        printf("Decoded IR command\n");

        ac_state = state;

        homekit_value_t new_target_state, new_fan_active;
        if (state.command == ac_cmd_turn_off) {
            fan = 0;
            new_target_state = HOMEKIT_UINT8(0);
            new_fan_active = HOMEKIT_UINT8(0);
        } else if (state.command == ac_cmd_turn_on || state.command == ac_cmd_stay_on) {
            fan = 0;
            switch (state.mode) {
            case ac_mode_heat:
                new_target_state = HOMEKIT_UINT8(1);
                break;
            case ac_mode_cool:
                new_target_state = HOMEKIT_UINT8(2);
                break;
            case ac_mode_auto:
                new_target_state = HOMEKIT_UINT8(3);
                break;
            case ac_mode_dry:
            case ac_mode_fan:
                new_target_state = HOMEKIT_UINT8(0);
                break;
            }

            new_fan_active = HOMEKIT_UINT8(1);

            homekit_value_t new_target_temperature = HOMEKIT_FLOAT(state.temperature);
            homekit_value_t new_fan_rotation_speed;
            switch (state.fan) {
            case ac_fan_auto:
            case ac_fan_high:
                new_fan_rotation_speed = HOMEKIT_FLOAT(100);
                break;
            case ac_fan_med:
                new_fan_rotation_speed = HOMEKIT_FLOAT(75);
                break;
            case ac_fan_low:
                new_fan_rotation_speed = HOMEKIT_FLOAT(45);
                break;
            case ac_fan_quiet:
                new_fan_rotation_speed = HOMEKIT_FLOAT(15);
                break;
            }
            homekit_value_t new_fan_swing_mode = HOMEKIT_UINT8((state.swing == ac_swing_off) ? 0 : 1);

            if (!homekit_value_equal(&new_target_temperature, &target_temperature.value)) {
                target_temperature.value = new_target_temperature;
                homekit_characteristic_notify(&target_temperature, target_temperature.value);
            }

            if (!homekit_value_equal(&new_fan_rotation_speed, &fan_rotation_speed.value)) {
                fan_rotation_speed.value = new_fan_rotation_speed;
                homekit_characteristic_notify(&fan_rotation_speed, fan_rotation_speed.value);
            }

            if (!homekit_value_equal(&new_fan_swing_mode, &fan_swing_mode.value)) {
                fan_swing_mode.value = new_fan_swing_mode;
                homekit_characteristic_notify(&fan_swing_mode, fan_swing_mode.value);
            }
        }

        if (!homekit_value_equal(&new_target_state, &target_state.value)) {
            target_state.value = new_target_state;
            homekit_characteristic_notify(&target_state, target_state.value);
        }

        if (!homekit_value_equal(&new_fan_active, &fan_active.value)) {
            fan_active.value = new_fan_active;
            homekit_characteristic_notify(&fan_active, fan_active.value);
        }
    }

    decoder->free(decoder);

    vTaskDelete(NULL);
}


void thermostat_identify_task(void *_args) {
    for (int i=0; i<3; i++) {
        for (int j=0; j<2; j++) {
            led_write(true);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            led_write(false);
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }

        vTaskDelay(250 / portTICK_PERIOD_MS);
    }

    led_write(false);

    vTaskDelete(NULL);
}

void thermostat_identify(homekit_value_t _value) {
    printf("Thermostat identify\n");
    xTaskCreate(thermostat_identify_task, "Thermostat identify", 128, NULL, 2, NULL);
}

homekit_characteristic_t name = HOMEKIT_CHARACTERISTIC_(NAME, "Fujitsu AC");

homekit_accessory_t *accessories[] = {
    HOMEKIT_ACCESSORY(.id=1, .category=homekit_accessory_category_thermostat, .services=(homekit_service_t*[]){
        HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]){
            &name,
            HOMEKIT_CHARACTERISTIC(MANUFACTURER, "HaPK"),
            HOMEKIT_CHARACTERISTIC(SERIAL_NUMBER, "1"),
            HOMEKIT_CHARACTERISTIC(MODEL, "Basic"),
            HOMEKIT_CHARACTERISTIC(FIRMWARE_REVISION, "0.1"),
            HOMEKIT_CHARACTERISTIC(IDENTIFY, thermostat_identify),
            NULL
        }),
        HOMEKIT_SERVICE(THERMOSTAT, .primary=true, .characteristics=(homekit_characteristic_t*[]) {
            HOMEKIT_CHARACTERISTIC(NAME, "Thermostat"),
            &current_humidity,
            &current_temperature,
            &target_temperature,
            &current_state,
            &target_state,
            &units,
            NULL
        }),
        HOMEKIT_SERVICE(FAN2, .characteristics=(homekit_characteristic_t*[]) {
            HOMEKIT_CHARACTERISTIC(NAME, "Fan"),
            &fan_active,
            &fan_rotation_speed,
            &fan_swing_mode,
            NULL
        }),
        NULL
    }),
    NULL
};

homekit_server_config_t config = {
    .accessories = accessories,
    .password = "111-11-111"
};

void on_wifi_ready() {
    homekit_server_init(&config);
}

void create_accessory_name() {
    uint8_t macaddr[6];
    sdk_wifi_get_macaddr(STATION_IF, macaddr);

    int name_len = snprintf(NULL, 0, "Fujitsu AC-%02X%02X%02X",
                            macaddr[3], macaddr[4], macaddr[5]);
    char *name_value = malloc(name_len+1);
    snprintf(name_value, name_len+1, "Fujitsu AC-%02X%02X%02X",
             macaddr[3], macaddr[4], macaddr[5]);

    name.value = HOMEKIT_STRING(name_value);
}

void user_init(void) {
    uart_set_baud(0, 115200);

    led_init();
    create_accessory_name();

    wifi_config_init("fujitsu-ac", NULL, on_wifi_ready);

    ac_state.command = ac_cmd_turn_off;
    ac_state.temperature = 22;
    ac_state.mode = ac_mode_auto;

    ac_state.fan = ac_fan_auto;
    ac_state.swing = ac_swing_off;

    fujitsu_ac_ir_tx_init(fujitsu_ac_model_ARRAH2E);
    ir_rx_init(IR_RX_GPIO, 300);
    update_state();

    xTaskCreate(temperature_sensor_task, "Thermostat", 256, NULL, 2, NULL);
    xTaskCreate(ir_rx_task, "IR receiver", 1024, NULL, 2, NULL);
}
