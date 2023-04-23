#include <Wire.h>
#include "Adafruit_MPR121.h"
#include "RunningMedian.h"

#include "device_id.h"
#include "utils.h"
#include "Timer.hpp"

// lora / radiohead
#include "rfm95_config.h"
#include "NodeTypes.h"

const uint8_t g_lora_address = 123;

// analog-digital-converter (ADC)
#define ADC_BITS 10
constexpr uint32_t ADC_MAX = (1 << ADC_BITS) - 1U;

const int g_update_interval = 33;
char g_serial_buf[512];

//! time management
const int g_update_interval_params = 2000;
int g_time_accum = 0, g_time_accum_params = 0;
long g_last_time_stamp;

enum TimerEnum
{
  TIMER_LORA_SEND = 0,
  TIMER_LED_OFF = 1,
  TIMER_BATTERY_MEASURE = 2,
  NUM_TIMERS
};
kinski::Timer g_timer[NUM_TIMERS];

//! number of capacitve touch sensors used
#define NUM_SENSORS 1

// up to 4 on one i2c bus
// Default address is 0x5A, if tied to 3.3V its 0x5B
// If tied to SDA its 0x5C and if SCL then 0x5D
const uint8_t g_i2c_adresses[] = {0x5A, 0x5B, 0x5C, 0x5D};

struct sensor_struct_t
{
  Adafruit_MPR121 cap_sensor;

  uint16_t num_samples = 5;

  RunningMedian proxy_median = RunningMedian(5);

  //! contains a continuous measure for proximity
  float proxy_value;

  //! bitmasks for touch status
  uint16_t touch_buffer;
};

Adafruit_MPR121 g_cap_sensors[NUM_SENSORS];

const uint16_t g_num_samples = 5;
RunningMedian* g_proxy_medians = nullptr;

//! contains a continuous measure for proximity
float g_proxy_values[NUM_SENSORS];

//! bitmasks for touch status
uint16_t g_touch_buffer[NUM_SENSORS];

//! adjust these for desired sensitivity
const uint8_t g_thresh_touch = 12;
const uint8_t g_thresh_release = 6;
const uint8_t g_charge_time = 0;

// potis
#define NUM_POTIS 2
uint8_t g_poti_pins[NUM_POTIS] = {A4, A5};
const uint16_t g_num_poti_samples = 3;
float g_potis[NUM_POTIS];
RunningMedian g_poti_median(g_num_poti_samples);

// button
constexpr uint8_t g_button_pin = 12;
bool g_button_pressed = false;

// led blinking
uint32_t g_blink_accum = 0;
constexpr float g_led_duration = 0.1f;

// battery
constexpr uint8_t g_battery_pin = A7;
uint8_t g_battery_val = 0;

////////////////////////////////////////////////////////////////////////////////
// forward declared functions

void parse_line(char *the_line);

template <typename T> void process_input(T& the_device);

// easing helper
inline float easeInCubic(float t){ return t * t * t; }

////////////////////////////////////////////////////////////////////////////////

// lora assets
lora::config_t g_lora_config = {};

// bundle radio-driver, datagram-manager and crypto assets
lora::driver_struct_t m_rfm95 = {};

//! lora message buffer
uint8_t g_lora_buffer[RH_RF95_MAX_MESSAGE_LEN];

float g_lora_send_interval = .2f;

void set_address(uint8_t address)
{
    g_lora_config.address = address;

    // init RFM95 module
    if(lora::setup(g_lora_config, m_rfm95))
    {
        Serial.print("LoRa radio init complete -> now listening on adress: 0x");
        Serial.println(g_lora_config.address, HEX);
    }
    else{ Serial.println("LoRa radio init failed"); }
}

void lora_receive()
{
    uint8_t len = sizeof(g_lora_buffer);
    uint8_t from, to, msg_id, flags;

    // check for messages addressed to this node
    if(m_rfm95.manager->recvfromAck(g_lora_buffer, &len, &from, &to, &msg_id, &flags))
    {
        // received something
    }
}

template<typename T> bool lora_send_status(const T &data)
{
    // data + checksum
    constexpr size_t num_bytes = sizeof(T) + 1;

    uint8_t crc_data[3 + sizeof(T)];
    crc_data[0] = g_lora_config.address;
    crc_data[1] = RH_BROADCAST_ADDRESS;

    memcpy(crc_data + 2, &data, sizeof(T));
    crc_data[sizeof(crc_data) - 1] = crc8(crc_data, 2 + sizeof(T));

    // send a broadcast-message
    return m_rfm95.manager->sendto(crc_data + 2, num_bytes, RH_BROADCAST_ADDRESS);
}

////////////////////////////////////////////////////////////////////////////////

void blink_status_led()
{
    digitalWrite(13, LOW);
    delay(500);
    digitalWrite(13, HIGH);
    delay(500);
}

void setup()
{
    // drives our status LED
    pinMode(13, OUTPUT);

    // pullup style button
    pinMode(g_button_pin, INPUT_PULLUP);

    // indicate "not ready"
    digitalWrite(13, HIGH);

    // while(!Serial){ blink_status_led(); }
    Serial.begin(115200);

    g_proxy_medians = (RunningMedian*)malloc(NUM_SENSORS * sizeof(RunningMedian));

    for(uint8_t i = 0; i < NUM_SENSORS; ++i)
    {
        if(!g_cap_sensors[i].begin(g_i2c_adresses[i]))
        {
            // will block here if address is not found on i2c bus
        }
        g_cap_sensors[i].setThresholds(g_thresh_touch, g_thresh_release);

        // // set global charge current (0 ... 63uA)
        // g_cap_sensors[i].setChargeCurrentAndTime(63, g_charge_time);

        // // enable proximity mode
        // g_cap_sensors[i].setMode(Adafruit_MPR121::PROXI_01);

        g_touch_buffer[i] = 0;

        g_proxy_medians[i] = RunningMedian(g_num_samples);
    }

    // battery measuring
    g_timer[TIMER_BATTERY_MEASURE].set_callback([]()
    {
        // voltage is divided by 2, so multiply back
        constexpr float voltage_divider = 2.f;
        auto raw_bat_measure = analogRead(g_battery_pin);

        float voltage = 3.3f * (float)raw_bat_measure * voltage_divider / (float)ADC_MAX;
        g_battery_val = static_cast<uint8_t>(map_value<float>(voltage, 3.3f, 4.2f, 0.f, 255.f));
        // Serial.printf("battery: %d%%\n", 100 * g_battery_val / 255);
        Serial.printf("raw_bat_measure: %d\n", raw_bat_measure);
    });
    g_timer[TIMER_BATTERY_MEASURE].set_periodic();
    g_timer[TIMER_BATTERY_MEASURE].expires_from_now(10.f);

    // lora config
    set_address(g_lora_address);

    g_timer[TIMER_LORA_SEND].set_callback([]()
    {
        elevator_t elevator = {};
        elevator.button = g_button_pressed;
        elevator.touch_status = g_touch_buffer[0];
        elevator.intensity = 255 * easeInCubic(g_potis[0]);
        elevator.velocity = 255 * g_potis[1];

        g_button_pressed = false;
        g_touch_buffer[0] = 0;
        
        lora_send_status(elevator);
    });
    g_timer[TIMER_LORA_SEND].set_periodic();
    g_timer[TIMER_LORA_SEND].expires_from_now(g_lora_send_interval);

    // LED off timer
    g_timer[TIMER_LED_OFF].set_callback([](){ digitalWrite(13, 0); });

    digitalWrite(13, LOW);
}

void loop()
{
    // time measurement
    uint32_t delta_time = millis() - g_last_time_stamp;
    g_last_time_stamp = millis();
    g_time_accum += delta_time;
    g_time_accum_params += delta_time;

    // poll Timer objects
    for(uint32_t i = 0; i < NUM_TIMERS; ++i){ g_timer[i].poll(); }

    uint16_t touch_states = 0;

    for(uint8_t i = 0; i < NUM_SENSORS; i++)
    {
        int16_t base_value = g_cap_sensors[i].baselineData(12);
        g_proxy_medians[i].add(base_value - (int16_t)g_cap_sensors[i].filteredData(12));
        int16_t diff_to_base = g_proxy_medians[i].getMedian();

        const float decay_secs = .35f;
        float decay = 1.f / decay_secs * delta_time / 1000.f;
        g_proxy_values[i] = max(0, g_proxy_values[i] - decay);

        float val = clamp<float>(diff_to_base / 100.f, 0.f, 1.f);
        if(g_proxy_values[i] < val){ g_proxy_values[i] = val; }

        // touch_states |= (g_cap_sensors[i].touched() ? 1 : 0) << i;

        // uncomment to override: use inbuilt touch mechanism
        touch_states = g_cap_sensors[i].touched();

        // register all touch-events and keep them
        g_touch_buffer[i] |= touch_states;
    }

    // button
    g_button_pressed = g_button_pressed || !digitalRead(g_button_pin);

    // read potis
    for(size_t i = 0; i < NUM_POTIS; i++)
    {
        for(size_t k = 0; k < g_num_poti_samples; k++)
        {
            g_poti_median.add(analogRead(g_poti_pins[i]));
        }
        g_potis[i] = map_value<float>(g_poti_median.getMedian(), 0.f, ADC_MAX, 0.f, 1.f);
    }

    // frequency blink
    g_blink_accum += delta_time;

    float freq = map_value(easeInCubic(g_potis[0]), 0.f, 1.f, .1f, 10.f);
    auto freq_blink_interval = static_cast<uint32_t>(1000.f / freq);

    if(g_blink_accum >= freq_blink_interval)
    {
        g_blink_accum = 0;
        digitalWrite(13, 1);
        g_timer[TIMER_LED_OFF].expires_from_now(g_led_duration);
    }

    // if(g_time_accum >= g_update_interval)
    // {
    //     char fmt_buf[32];
    //     char *buf_ptr = g_serial_buf;
    //
    //     for(uint8_t i = 0; i < NUM_SENSORS; i++)
    //     {
    //         buf_ptr += sprintf(buf_ptr, "%d", g_touch_buffer[0]);
    //
    //         fmt_real_to_str(fmt_buf, g_proxy_values[i]);
    //         buf_ptr += sprintf(buf_ptr, " %s ", fmt_buf);
    //     }
    //     sprintf(buf_ptr - 1, "\n");
    //
    //     g_time_accum = 0;
    //     // g_touch_buffer[0] = 0;
    //
    //     // IO -> Serial
    //     process_input(Serial);
    //     Serial.print(g_serial_buf);
    // }
}

template <typename T> void process_input(T& the_device)
{
    uint16_t buf_idx = 0;

    while(the_device.available())
    {
        // get the new byte:
        char c = the_device.read();

        switch(c)
        {
            case '\r':
            case '\0':
                continue;

            case '\n':
                g_serial_buf[buf_idx] = '\0';
                buf_idx = 0;
                parse_line(g_serial_buf);
                break;

            default:
                g_serial_buf[buf_idx++] = c;
                break;
        }
    }
}

bool check_for_cmd(const char* the_str)
{
    if(strcmp(the_str, CMD_QUERY_ID) == 0)
    {
        char buf[32];
        sprintf(buf, "%s %s\n", the_str, DEVICE_ID);
        Serial.print(buf);
        #ifdef USE_BLUETOOTH
            g_bt_serial.write(buf);
        #endif
        return true;
    }
    return false;
}

void parse_line(char *the_line)
{
    const char* delim = " ";
    const size_t elem_count = 3;
    char *token = strtok(the_line, delim);
    int num_buf[elem_count];
    uint16_t i = 0;

    for(; token && (i < elem_count); i++)
    {
        if(check_for_cmd(token)){ break; }
        else{ num_buf[i] = atoi(token); }
        token = strtok(nullptr, delim);
    }

    if(i == elem_count)
    {
        for(uint8_t i = 0; i < NUM_SENSORS; ++i)
        {
            g_cap_sensors[i].setThresholds(num_buf[0], num_buf[1]);
            // g_cap_sensors[i].setChargeCurrentAndTime(num_buf[2], g_charge_time);
        }
    }
}
