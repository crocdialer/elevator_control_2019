#include "Arduino.h"
#include "ModeHelpers.h"
#include "Timer.hpp"
#include "device_id.h"

// #include <Scheduler.h>

// accelerometer
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include "RunningMedian.h"
#include "utils.h"
#include "vec3.h"

// lora / radiohead
#include "rfm95_config.h"

// serialization
#include "NodeTypes.h"

// update rate in Hz
#define UPDATE_RATE 30
#define SERIAL_BUFSIZE 128

// analog-digital-converter (ADC)
#define ADC_BITS 10
constexpr uint32_t ADC_MAX = (1 << ADC_BITS) - 1U;

char g_serial_buf[SERIAL_BUFSIZE];
uint32_t g_buf_index = 0;

// helper variables for time measurement
long g_last_time_stamp = 0;
uint32_t g_time_accum = 0;

// update interval in millis
const int g_update_interval = 1000 / UPDATE_RATE;

enum TimerEnum
{
    TIMER_LED_OFF = 0,
    TIMER_BRIGHTNESS_MEASURE,
    TIMER_BATTERY_MEASURE,
    TIMER_LORA_SEND,
    NUM_TIMERS
};

// an array of Timer objects
kinski::Timer g_timer[NUM_TIMERS];

// helper for flashing PIN 13 (red onboard LED)
// to indicate update frequency
bool g_indicator = false;

bool g_use_indicator = false;

//! define our run-modes here
enum RunMode
{
    MODE_DEBUG = 1 << 0,
    MODE_RUNNING = 1 << 1
};
uint32_t g_run_mode = MODE_RUNNING;

bool g_leds_enabled = true;
constexpr uint8_t g_num_paths = 1;
constexpr uint8_t g_path_lengths[] = {1};
const uint8_t g_led_pins[] = {5};

LED_Path* g_path[g_num_paths];
ModeHelper *g_mode_sinus = nullptr, *g_mode_current = nullptr;
Mode_ONE_COLOR *g_mode_colour = nullptr;
CompositeMode *g_mode_composite = nullptr;

// timeout before leds turn off
float g_led_timeout = 30.f;

// brightness measuring
constexpr uint8_t g_photo_pin = A5;
constexpr uint16_t g_photo_thresh = 60;
uint16_t g_photo_val = 0;

// battery measuring
constexpr uint8_t g_battery_pin = A6;
uint16_t g_battery_val = 0;

// acceleration measuring
Adafruit_LIS3DH g_accel;
constexpr lis3dh_range_t g_sensor_range = LIS3DH_RANGE_4_G;
sensors_event_t g_sensor_event;
const uint16_t g_sense_interval = 2;
float g_last_accel_val = 0;
float g_max_accel_val = 0;
float g_accel_thresh = 1.f;

// median filtering for accelerometer
const uint16_t g_num_samples = 3;
RunningMedian g_running_median = RunningMedian(g_num_samples);

// lora assets
lora::config_t g_lora_config = {};

// bundle radio-driver, datagram-manager and crypto assets
lora::driver_struct_t m_rfm95 = {};

//! lora message buffer
uint8_t g_lora_buffer[RH_RF95_MAX_MESSAGE_LEN];

float g_lora_send_interval = 1.f;

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
        // null-terminated
        // if(len < sizeof(g_lora_buffer)){ g_lora_buffer[len] = 0; }

        // if(from == m_node_info.server_address && to == g_lora_config.address)
        if(len >= sizeof(smart_bulb_t) && g_lora_buffer[0] == STRUCT_TYPE_SMART_BULB)
        {
            // auto rssi = m_rfm95.driver->lastRssi();
            // sprintf(g_serial_buf, "src: %d -- rssi: %d\n", from, rssi);
            // Serial.write(g_serial_buf);
            //
            // smart_bulb_t data = {};
            // memcpy(&data, g_lora_buffer, len);

            // sprintf(g_serial_buf, "{\n\tlight:%d\n\tacceleration: %d\n\tbattery: %d\n}\n",
            //         data.light_sensor, data.acceleration, data.battery);
            // Serial.write(g_serial_buf);
        }
    }
}

bool lora_send_status()
{
    bool ret = false;
    constexpr size_t num_bytes = sizeof(smart_bulb_t) + 1;

    uint8_t data[num_bytes];

    smart_bulb_t &smart_bulb = *(smart_bulb_t*)data;
    smart_bulb = {};
    smart_bulb.leds_enabled = g_leds_enabled;
    smart_bulb.battery = g_battery_val;
    smart_bulb.acceleration = static_cast<uint8_t>(map_value<float>(g_max_accel_val, 0.f, 4.f, 0, 255));
    smart_bulb.light_sensor = static_cast<uint8_t>(map_value<float>(g_photo_val, 0.f, 10 * g_photo_thresh,
                                                                    0, 255));
    g_max_accel_val = 0;

    // checksum
    data[num_bytes - 1] = crc8(data, sizeof(smart_bulb_t));

    // send a message to the lora mesh-server
    if(m_rfm95.manager->sendtoWait(data, num_bytes, RH_BROADCAST_ADDRESS))
    {
        // the message has been reliably delivered to the next node.
        ret = true;
    }
    else
    {
        // failed to send to next hop
        Serial.println("lora: could not find a route to destination ...");
    }
    return ret;
}

void blink()
{
    digitalWrite(13, true);
    delay(666);
    digitalWrite(13, false);
    delay(666);
}

void enable_leds(bool use_leds)
{
    for(uint8_t i = 0; i < g_num_paths; ++i)
    {
        for(uint8_t j = 0; j < g_path[i]->num_segments(); ++j)
        {
            Segment *s = g_path[i]->segment(j);
            s->set_active(use_leds);
        }
    }
    digitalWrite(13, use_leds);
    g_leds_enabled = use_leds;
}

void setup()
{
    // while(!Serial){ delay(10); }
    Serial.begin(115200);

    srand(analogRead(A0));

    // button
    pinMode(13, OUTPUT);
    pinMode(12, INPUT_PULLUP);

    // enable photo sense pin
    pinMode(g_photo_pin, INPUT);

    //setup accelerometer
    g_accel = Adafruit_LIS3DH();
    if(!g_accel.begin()){ Serial.print("could not connect accelerometer ..."); }
    g_accel.setRange(g_sensor_range);

    // init path objects with pin array
    for(uint8_t i = 0; i < g_num_paths; ++i)
    {
         g_path[i] = new LED_Path(g_led_pins[i], g_path_lengths[i]);
    }

    // create ModeHelper objects
    g_mode_colour = new Mode_ONE_COLOR();
    g_mode_sinus = new SinusFill();
    g_mode_current = g_mode_composite = new CompositeMode();
    g_mode_composite->add_mode(g_mode_colour);
    g_mode_composite->add_mode(g_mode_sinus);

    // timer callback to reset the runmode after streaming
    g_timer[TIMER_LED_OFF].set_callback([](){ enable_leds(false); });
    g_timer[TIMER_LED_OFF].expires_from_now(g_led_timeout);

    // start with leds turned on
    enable_leds(true);

    // brightness measuring
    g_timer[TIMER_BRIGHTNESS_MEASURE].set_callback([]()
    {
        g_photo_val = analogRead(g_photo_pin);
        bool use_leds = g_photo_val < g_photo_thresh;

        if(use_leds)
        {
            enable_leds(true);
            g_timer[TIMER_LED_OFF].expires_from_now(g_led_timeout);
        }

    });
    g_timer[TIMER_BRIGHTNESS_MEASURE].set_periodic();
    g_timer[TIMER_BRIGHTNESS_MEASURE].expires_from_now(.2f);

    // battery measuring
    g_timer[TIMER_BATTERY_MEASURE].set_callback([]()
    {
        // voltage is divided by 2, so multiply back
        constexpr float voltage_divider = 2.f;
        auto raw_bat_measure = analogRead(g_battery_pin);

        float voltage = 3.3f * (float)raw_bat_measure * voltage_divider / (float)ADC_MAX;
        g_battery_val = static_cast<uint8_t>(map_value<float>(voltage, 3.3f, 4.2f, 0.f, 255.f));
        // Serial.printf("battery: %d%%\n", 100 * g_battery_val / 255);
    });
    g_timer[TIMER_BATTERY_MEASURE].set_periodic();
    g_timer[TIMER_BATTERY_MEASURE].expires_from_now(10.f);

    // lora config
    set_address(69);

    g_timer[TIMER_LORA_SEND].set_callback([](){ lora_send_status(); });
    g_timer[TIMER_LORA_SEND].set_periodic();
    g_timer[TIMER_LORA_SEND].expires_from_now(g_lora_send_interval);
}

void loop()
{
    // time measurement
    uint32_t delta_time = millis() - g_last_time_stamp;
    g_last_time_stamp = millis();
    g_time_accum += delta_time;

    // measure acceleration
    for(uint8_t i = 0; i < g_num_samples; i++)
    {
        // sensor reading
        g_accel.getEvent(&g_sensor_event);
        vec3 measurement(g_sensor_event.acceleration.v);
        g_running_median.add(measurement.length2());

        // wait the desired interval
        delay(g_sense_interval);
    }
    constexpr float base_g =  9.82f;
    float g_factor = sqrtf(g_running_median.getMedian()) / base_g;
    g_last_accel_val = max(g_last_accel_val, max(g_factor - 1.f, 0.f));
    g_max_accel_val = max(g_last_accel_val, g_max_accel_val);

    // poll Timer objects
    for(uint32_t i = 0; i < NUM_TIMERS; ++i){ g_timer[i].poll(); }

    // receive
    lora_receive();

    // button
    bool button_pressed = !digitalRead(12);

    if(button_pressed || (g_last_accel_val > g_accel_thresh))
    {
        if(g_timer[TIMER_LED_OFF].has_expired()){ enable_leds(true); }
        g_timer[TIMER_LED_OFF].expires_from_now(g_led_timeout);
    }

    if(g_time_accum >= g_update_interval)
    {
        auto color = color_mix(ORANGE, AQUA, clamp(g_last_accel_val / 2.f, 0.f, 1.f));
        g_mode_colour->set_color(color);

        for(uint8_t i = 0; i < g_num_paths; ++i)
        {
            g_mode_current->process(g_path[i], g_time_accum);
            g_path[i]->update(g_time_accum);
        }

        // clear acceleration max
        g_last_accel_val = 0;

        // clear time accumulator
        g_time_accum = 0;
    }
    yield();
}
