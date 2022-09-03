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

#define VER_MAJOR       1
#define VER_MINOR       1
#define VER_REVISION    0
#define VER_BUILD       3

#define CAN_BAUD_RATE 500000    //500 kbaud
#define CAN_NODE_ID 0x4
//NOTE: There is a max of 32 messages per Node ID
#define CAN_VER_ADDRESS         (CAN_NODE_ID << 5) | 0x0
#define CAN_SOC_ADDRESS         (CAN_NODE_ID << 5) | 0x1
#define CAN_BAT_V_ADDRESS       (CAN_NODE_ID << 5) | 0x2

//Battery Interface
#define BATTERY_VOLTAGE_PIN 29
#define BATTERY_VOLTAGE_CHAN 3
#define CONVERT_ADC_TO_VOLTAGE(raw) (raw / 256.3)   //The formula to convert ADC counts to voltage
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
#define CAN_SEND_TIMER_INTERVAL -500

#define STDIO_UART_PERIPHERAL uart0
#define CANBUS_SPI_PERIPHERAL spi0

uint32_t voltageRawADCVal;
uint16_t voltageReadCounter;
float batteryVoltage;
float stateOfCharge;
can_frame frame;
uint8_t frameID = 0;
struct repeating_timer voltageReadTimer;
struct repeating_timer canSendTimer;

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

bool voltageReadTimerCallback(struct repeating_timer *t)
{
    voltageRawADCVal += adc_read();
    voltageReadCounter++;

    return true;
}

bool canSendTimerCallback(struct repeating_timer *t)
{
    can_frame frame;

    voltageRawADCVal /= voltageReadCounter;
    batteryVoltage = CONVERT_ADC_TO_VOLTAGE(voltageRawADCVal);
    stateOfCharge = calculateSoC(batteryVoltage);
    printf("ID: %3d, Raw: %5d, Voltage: %5.1f V, State of Charge: %4.0f%%, Sample Count: %5d\r\n",
                frameID, voltageRawADCVal, batteryVoltage, stateOfCharge, voltageReadCounter);

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

    frameID++;
    voltageRawADCVal = 0;
    voltageReadCounter = 0;

    return true;
}

void startupStdio()
{
    stdio_init_all();

    for(int i = 0; i < 10; i++)
    {
        printf(".");
        sleep_ms(500);
        
    }

    printf("\r\n\r\n\r\nVersion: %d.%d.%d build %d\r\n", VER_MAJOR, VER_MINOR, VER_REVISION, VER_BUILD);
}

void startupCANBus()
{
    printf("Setting up MCP2515...\n");

    mcp2515.reset();
    mcp2515.setBitrate(CAN_500KBPS, MCP_16MHZ);
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
    adc_gpio_init(BATTERY_VOLTAGE_PIN);
    adc_select_input(BATTERY_VOLTAGE_CHAN);
}

void setupTimer()
{
    add_repeating_timer_ms(VOLTAGE_READ_TIMER_INTERVAL, voltageReadTimerCallback, NULL, &voltageReadTimer);
    add_repeating_timer_ms(CAN_SEND_TIMER_INTERVAL, canSendTimerCallback, NULL, &canSendTimer);
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
    {

    }
}