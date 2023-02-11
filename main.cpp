#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "include/can.h"
#include "include/mcp2515.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/watchdog.h"

#define VER_MAJOR       2
#define VER_MINOR       0
#define VER_REVISION    0
#define VER_BUILD       1

#define CAN_BAUD_RATE 500000    //500 kbaud
#define CAN_NODE_ID 0x4
//NOTE: There is a max of 32 messages per Node ID
#define CAN_RESET_ADDRESS       (CAN_NODE_ID << 5) | 0x0
#define CAN_VER_ADDRESS         (CAN_NODE_ID << 5) | 0x1
#define CAN_SOC_ADDRESS         (CAN_NODE_ID << 5) | 0x2
#define CAN_BAT_V_ADDRESS       (CAN_NODE_ID << 5) | 0x3
#define CAN_BAT_V0_ADDRESS      (CAN_NODE_ID << 5) | 0x4
#define CAN_BAT_V1_ADDRESS      (CAN_NODE_ID << 5) | 0x5
#define CAN_BAT_V2_ADDRESS      (CAN_NODE_ID << 5) | 0x6

#define CAN_RECEIVE_MASK    0xE0
#define CAN_RECEIVE_FILTER  (CAN_NODE_ID << 5)

//Battery Interface
#define BATTERY_V2_PIN 28
#define BATTERY_V2_CHAN 2
#define BATTERY_V1_PIN 27
#define BATTERY_V1_CHAN 1
#define BATTERY_V0_PIN 26
#define BATTERY_V0_CHAN 0
//CAN Interface
#define CAN_SCK_PIN 2
#define CAN_TX_PIN 3
#define CAN_RX_PIN 4
#define CAN_CS_PIN 5
//LEDs
#define GREEN_LED_PIN 16
#define RED_LED_PIN 17
#define LED_PWM_SLICE 0 //(PinNum >> 1) & 7 ... Can also use: uint pwm_gpio_to_slice_num(pinNum)
#define LED_PWM_COUNTER_MIN 0
#define LED_PWM_RED_ON_LVL  80
#define LED_PWM_GRN_ON_LVL  40
#define LED_PWM_COUNTER_MAX 100
#define GREEN_LED_PWM_CHAN PWM_CHAN_A
#define RED_LED_PWM_CHAN   PWM_CHAN_B

//Negative time means the timer will restart at the beginning of the callback (as opposed to the end)
#define VOLTAGE_READ_TIMER_INTERVAL -20
#define CAN_SEND_TIMER_INTERVAL     -500
#define CAN_RECEIVE_TIMER_INTERVAL  -50

#define WATCHDOG_REBOOT_DELAY       50
#define BOOT_WAIT_TIME              1000

#define STDIO_UART_PERIPHERAL uart0
#define CANBUS_SPI_PERIPHERAL spi0

float batteryVoltageDividend[3] = {929.4, 436.5, 322.6};
uint32_t voltageRawADCVal[3];
uint16_t voltageReadCounter;
float cellVoltage[3];
float batteryVoltage;
float stateOfCharge;
can_frame frame;
uint8_t frameID = 0;
struct repeating_timer voltageReadTimer;
struct repeating_timer canSendTimer;
struct repeating_timer canReceiveTimer;

float socOutMap[] = {0.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0,
                        55.0, 60.0, 65.0, 70.0, 75.0, 80.0, 85.0, 90.0, 95.0, 100.0};
//Input array must be in increasing order
float voltInMap[] = {9.82, 10.83, 11.06, 11.12, 11.18,
                   11.24, 11.30, 11.36, 11.39, 11.45,
                   11.51, 11.56, 11.62, 11.74, 11.86,
                   11.95, 12.07, 12.25, 12.33, 12.43, 12.6};
#define SOC_MAP_LENGTH 21

MCP2515 mcp2515(CANBUS_SPI_PERIPHERAL, CAN_CS_PIN, CAN_TX_PIN, CAN_RX_PIN, CAN_SCK_PIN, CAN_BAUD_RATE);

float calculateSoC(float voltage)
{
    uint index = 1;

    if(voltage <= voltInMap[0]) return socOutMap[0];
    if(voltage >= voltInMap[SOC_MAP_LENGTH - 1]) return socOutMap[SOC_MAP_LENGTH - 1];

    while(voltage > voltInMap[index]) index++;
    if(voltage == voltInMap[index]) return socOutMap[index];

    return (voltage - voltInMap[index-1]) * (socOutMap[index] - socOutMap[index-1]) / (voltInMap[index] - voltInMap[index-1]) + socOutMap[index-1];
}

void stopAllTimers()
{
    cancel_repeating_timer(&voltageReadTimer);
    cancel_repeating_timer(&canSendTimer);
    cancel_repeating_timer(&canReceiveTimer);
}

void rebootDevice()
{
    printf("A reboot has been triggered");
    stopAllTimers();
    watchdog_reboot(0, 0, WATCHDOG_REBOOT_DELAY);
    while(1);    
}

bool voltageReadTimerCallback(struct repeating_timer *t)
{
    adc_select_input(BATTERY_V0_CHAN);
    voltageRawADCVal[0] += adc_read();
    adc_select_input(BATTERY_V1_CHAN);
    voltageRawADCVal[1] += adc_read();
    adc_select_input(BATTERY_V2_CHAN);
    voltageRawADCVal[2] += adc_read();

    voltageReadCounter++;

    return true;
}

bool canSendTimerCallback(struct repeating_timer *t)
{
    can_frame frame;

    for(int i = 0; i < 3; i++)
    {
        voltageRawADCVal[i] /= voltageReadCounter;
        cellVoltage[i] = voltageRawADCVal[i] / batteryVoltageDividend[i];
    }
    
    batteryVoltage = cellVoltage[2];
    stateOfCharge = calculateSoC(batteryVoltage);

    cellVoltage[2] -= cellVoltage[1];
    cellVoltage[1] -= cellVoltage[0];

    printf("ID: %3d, Raw: [0] - %5d, [1] - %5d, [2] - %5d, Voltage: [0] - %5.1f V, [1] - %5.1f V, [2] - %5.1f V, [Bat] - %5.1f V, State of Charge: %4.0f%%, Sample Count: %5d\r\n",
                frameID, voltageRawADCVal[0], voltageRawADCVal[1], voltageRawADCVal[2],
                cellVoltage[0], cellVoltage[1], cellVoltage[2], batteryVoltage, stateOfCharge, voltageReadCounter);

    //Set up the CAN frame
    frame.can_dlc = 5;  //Values are 32 bit float plus frame ID
    //Add the frame ID so the other side can match all of the messages together
    frame.data[0] = frameID;

    frame.can_id = CAN_SOC_ADDRESS;
    memcpy(frame.data + 1, &stateOfCharge, sizeof(float));
    mcp2515.sendMessage(&frame);

    frame.can_id = CAN_BAT_V_ADDRESS;
    memcpy(frame.data + 1, &batteryVoltage, sizeof(float));
    mcp2515.sendMessage(&frame);

    frame.can_id = CAN_BAT_V0_ADDRESS;
    memcpy(frame.data + 1, &cellVoltage[0], sizeof(float));
    mcp2515.sendMessage(&frame);

    frame.can_id = CAN_BAT_V1_ADDRESS;
    memcpy(frame.data + 1, &cellVoltage[1], sizeof(float));
    mcp2515.sendMessage(&frame);

    frame.can_id = CAN_BAT_V2_ADDRESS;
    memcpy(frame.data + 1, &cellVoltage[2], sizeof(float));
    mcp2515.sendMessage(&frame);

    frameID++;
    for(int i = 0; i < 3; i++)
        voltageRawADCVal[i] = 0;
    voltageReadCounter = 0;

    return true;
}

bool canReceiveTimerCallback(struct repeating_timer *t)
{
    MCP2515::ERROR error;
    can_frame frame;

    error = mcp2515.readMessage(&frame);
        if(error == MCP2515::ERROR_OK)
            if(frame.can_id == CAN_RESET_ADDRESS)
                rebootDevice();

    return true;
}

void setupTimer()
{
    add_repeating_timer_ms(VOLTAGE_READ_TIMER_INTERVAL, voltageReadTimerCallback, NULL, &voltageReadTimer);
    add_repeating_timer_ms(CAN_SEND_TIMER_INTERVAL, canSendTimerCallback, NULL, &canSendTimer);
    add_repeating_timer_ms(CAN_RECEIVE_TIMER_INTERVAL, canReceiveTimerCallback, NULL, &canReceiveTimer);
}

void startupStdio()
{
    stdio_init_all();

    printf("\r\n\r\nWait...\r\n");
    sleep_ms(BOOT_WAIT_TIME);

    printf("\r\n\r\nVersion: %d.%d.%d build %d\r\n", VER_MAJOR, VER_MINOR, VER_REVISION, VER_BUILD);
}

void startupCANBus()
{
    printf("Setting up MCP2515...\n");

    mcp2515.reset();
    mcp2515.setBitrate(CAN_500KBPS, MCP_16MHZ);
    mcp2515.setFilterMask(MCP2515::MASK0, false, CAN_RECEIVE_MASK);
    mcp2515.setFilter(MCP2515::RXF0, false, CAN_RECEIVE_FILTER);
    mcp2515.setNormalMode();

    can_frame frame;

    frame.can_id = CAN_VER_ADDRESS;
    frame.can_dlc = 4;
    frame.data[3] = VER_MAJOR;
    frame.data[2] = VER_MINOR;
    frame.data[1] = VER_REVISION;
    frame.data[0] = VER_BUILD;
    MCP2515::ERROR err = mcp2515.sendMessage(&frame);
    if(err != MCP2515::ERROR_OK) printf("MCP Error: %d\r\n", err);

    printf("MCP2515 setup complete\n");
}

void setRedLED(bool on)
{
    pwm_set_chan_level(LED_PWM_SLICE, RED_LED_PWM_CHAN, on ? LED_PWM_RED_ON_LVL : LED_PWM_COUNTER_MIN);
}

void setGreenLED(bool on)
{
    pwm_set_chan_level(LED_PWM_SLICE, GREEN_LED_PWM_CHAN, on ? LED_PWM_GRN_ON_LVL : LED_PWM_COUNTER_MIN);
}

void setupGPIO()
{
    gpio_set_function(GREEN_LED_PIN, GPIO_FUNC_PWM);
    gpio_set_function(RED_LED_PIN, GPIO_FUNC_PWM);
    pwm_set_wrap(LED_PWM_SLICE, LED_PWM_COUNTER_MAX);
    setRedLED(false);
    setGreenLED(false);
    pwm_set_enabled(LED_PWM_SLICE, true);

}

void setupADC()
{
    adc_init();
    adc_gpio_init(BATTERY_V0_PIN);
    adc_gpio_init(BATTERY_V1_PIN);
    adc_gpio_init(BATTERY_V2_PIN);
    adc_select_input(BATTERY_V2_CHAN);
}

void peripheralStartup()
{
    setupGPIO();
    setRedLED(true);

    startupStdio();

    printf("Setting up Pins and Peripherals...\n");

    startupCANBus();
    setupADC();
    setupTimer();

    printf("Pins and Peripherals setup complete\n");

    setRedLED(false);
    setGreenLED(true);
}

int main ()
{
    peripheralStartup();

    while(true)
    { /*Don't do anything here, use timers instead*/ }
}