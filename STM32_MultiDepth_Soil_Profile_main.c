/* USER CODE BEGIN Header */

/*

 * Project: Multi-Depth Soil Nutrient Profiling System

 * Board:   STM32 NUCLEO-F446RE

 *

 * Moisture:

 *   Depth 1 -> PA0 / ADC1_IN0

 *   Depth 2 -> PA1 / ADC1_IN1

 *   Depth 3 -> PA4 / ADC1_IN4

 *

 * DS18B20:

 *   Shared 1-Wire data -> PB6

 *

 * RS485:

 *   STM32 PA9  -> RS485 DI

 *   STM32 PA10 <- RS485 RO

 *   USART1: 4800 baud, 8N1

 *

 * ESP32:

 *   STM32 PA2  -> ESP32 RX

 *   STM32 PA3  <- ESP32 TX

 *   USART2: 115200 baud, 8N1

 *

 * Relay:

 *   PB5

 *

 * Servo:

 *   PA8 / TIM1_CH1

 */

/* USER CODE END Header */



#include "main.h"

#include <stdio.h>

#include <string.h>

#include <stdint.h>

#include <stdlib.h>



/* Peripheral handles */

ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart1;

UART_HandleTypeDef huart2;

TIM_HandleTypeDef htim1;



/* Function prototypes */

void SystemClock_Config(void);

static void MX_GPIO_Init(void);

static void MX_ADC1_Init(void);

static void MX_USART1_UART_Init(void);

static void MX_USART2_UART_Init(void);

static void MX_TIM1_Init(void);

void Error_Handler(void);
void DWT_Init(void);
void Delay_us(uint32_t us);
void Read_Moisture_Sensors(void);
void Read_All_Temperatures(void);
uint8_t Read_NPK(void);
void Pump_Control(void);
void Pump_Set(uint8_t state);
void Servo_Set_Angle(uint8_t angle);
void Send_Data_To_ESP32(void);
uint8_t DS_Search_All(void);



/* ---------------- USER SETTINGS ---------------- */



/* Replace these with your measured ADC calibration values.

   DRY = sensor reading in dry soil/air.

   WET = sensor reading in fully wet soil. */

#define D1_DRY  4095

#define D1_WET  1500



#define D2_DRY  4095

#define D2_WET  1500



#define D3_DRY  4095

#define D3_WET  1500



#define PUMP_ON_THRESHOLD   30.0f

#define PUMP_OFF_THRESHOLD  45.0f



/* Set to 1 if your relay turns ON when PB5 is LOW. */

#define RELAY_ACTIVE_LOW    0



#define RELAY_PIN           GPIO_PIN_5

#define RELAY_PORT          GPIOB



#define ONEWIRE_PIN         GPIO_PIN_6

#define ONEWIRE_PORT        GPIOB



#define MAX_DS18B20         3



/* Modbus sensor settings */

#define MODBUS_ADDRESS      0x01

#define MODBUS_TIMEOUT      1000



/* ------------------------------------------------ */



typedef struct

{

    uint8_t rom[8];

} DS18B20_Device;



DS18B20_Device ds_devices[MAX_DS18B20];

uint8_t ds_count = 0;



float moisture1 = 0.0f;

float moisture2 = 0.0f;

float moisture3 = 0.0f;



float temperature1 = -127.0f;

float temperature2 = -127.0f;

float temperature3 = -127.0f;



uint16_t nitrogen = 0;

uint16_t phosphorus = 0;

uint16_t potassium = 0;



uint8_t npk_valid = 0;

uint8_t pump_state = 0;



/* =================================================

   CRC16 FOR MODBUS

   \================================================= */



uint16_t Modbus_CRC16(uint8_t *buffer, uint16_t length)

{

    uint16_t crc = 0xFFFF;



    for (uint16_t i = 0; i < length; i++)

    {

        crc ^= buffer[i];



        for (uint8_t j = 0; j < 8; j++)

        {

            if (crc & 1)

                crc = (crc >> 1) ^ 0xA001;

            else

                crc >>= 1;

        }

    }



    return crc;

}



/* =================================================

   ADC / MOISTURE SENSOR

   \================================================= */



uint16_t Read_ADC(uint32_t channel)

{

    ADC_ChannelConfTypeDef config = {0};



    config.Channel = channel;

    config.Rank = 1;

    config.SamplingTime = ADC_SAMPLETIME_144CYCLES;



    if (HAL_ADC_ConfigChannel(&hadc1, &config) != HAL_OK)

        return 0;



    if (HAL_ADC_Start(&hadc1) != HAL_OK)

        return 0;



    if (HAL_ADC_PollForConversion(&hadc1, 100) != HAL_OK)

    {

        HAL_ADC_Stop(&hadc1);

        return 0;

    }



    uint16_t value = (uint16_t)HAL_ADC_GetValue(&hadc1);



    HAL_ADC_Stop(&hadc1);



    return value;

}



float Convert_Moisture(uint16_t raw, uint16_t dry, uint16_t wet)

{

    if (dry == wet)

        return 0.0f;



    float percentage =

        ((float)dry - (float)raw) * 100.0f /

        ((float)dry - (float)wet);



    if (percentage < 0.0f)

        percentage = 0.0f;



    if (percentage > 100.0f)

        percentage = 100.0f;



    return percentage;

}



void Read_Moisture_Sensors(void)

{

    uint16_t raw1 = Read_ADC(ADC_CHANNEL_0);

    uint16_t raw2 = Read_ADC(ADC_CHANNEL_1);

    uint16_t raw3 = Read_ADC(ADC_CHANNEL_4);



    moisture1 = Convert_Moisture(raw1, D1_DRY, D1_WET);

    moisture2 = Convert_Moisture(raw2, D2_DRY, D2_WET);

    moisture3 = Convert_Moisture(raw3, D3_DRY, D3_WET);

}



/* =================================================

   DWT MICROSECOND DELAY

   \================================================= */



void DWT_Init(void)

{

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    DWT->CYCCNT = 0;

    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

}



void Delay_us(uint32_t us)

{

    uint32_t start = DWT->CYCCNT;

    uint32_t ticks = us * (HAL_RCC_GetHCLKFreq() / 1000000U);



    while ((DWT->CYCCNT - start) < ticks)

    {

    }

}



/* =================================================

   DS18B20 1-WIRE LOW-LEVEL FUNCTIONS

   \================================================= */



void OW_Low(void)

{

    HAL_GPIO_WritePin(ONEWIRE_PORT, ONEWIRE_PIN, GPIO_PIN_RESET);

}



void OW_Release(void)

{

    HAL_GPIO_WritePin(ONEWIRE_PORT, ONEWIRE_PIN, GPIO_PIN_SET);

}



uint8_t OW_ReadPin(void)

{

    return (uint8_t)HAL_GPIO_ReadPin(ONEWIRE_PORT, ONEWIRE_PIN);

}



uint8_t OW_Reset(void)

{

    uint8_t presence;



    OW_Low();

    Delay_us(480);



    OW_Release();

    Delay_us(70);



    presence = !OW_ReadPin();



    Delay_us(410);



    return presence;

}



void OW_WriteBit(uint8_t bit)

{

    OW_Low();



    if (bit)

    {

        Delay_us(6);

        OW_Release();

        Delay_us(64);

    }

    else

    {

        Delay_us(60);

        OW_Release();

        Delay_us(10);

    }

}



uint8_t OW_ReadBit(void)

{

    uint8_t bit;



    OW_Low();

    Delay_us(6);



    OW_Release();

    Delay_us(9);



    bit = OW_ReadPin();



    Delay_us(55);



    return bit;

}



void OW_WriteByte(uint8_t data)

{

    for (uint8_t i = 0; i < 8; i++)

    {

        OW_WriteBit(data & 0x01);

        data >>= 1;

    }

}



uint8_t OW_ReadByte(void)

{

    uint8_t data = 0;



    for (uint8_t i = 0; i < 8; i++)

    {

        if (OW_ReadBit())

            data |= (1U << i);

    }



    return data;

}



/* =================================================

   DS18B20 CRC8

   \================================================= */



uint8_t DS_CRC8(uint8_t *data, uint8_t length)

{

    uint8_t crc = 0;



    for (uint8_t i = 0; i < length; i++)

    {

        uint8_t inbyte = data[i];



        for (uint8_t j = 0; j < 8; j++)

        {

            uint8_t mix = (crc ^ inbyte) & 0x01;

            crc >>= 1;



            if (mix)

                crc ^= 0x8C;



            inbyte >>= 1;

        }

    }



    return crc;

}



/* =================================================

   DS18B20 DEVICE SEARCH

   \================================================= */



uint8_t DS_Search_All(void)

{

    uint8_t rom[8];

    uint8_t last_discrepancy = 0;

    uint8_t last_device = 0;

    uint8_t found = 0;



    memset(ds_devices, 0, sizeof(ds_devices));



    while (!last_device && found < MAX_DS18B20)

    {

        uint8_t bit_number = 1;

        uint8_t last_zero = 0;

        uint8_t rom_byte = 0;

        uint8_t rom_mask = 1;

        uint8_t search_failed = 0;



        memset(rom, 0, sizeof(rom));



        if (!OW_Reset())

            break;



        OW_WriteByte(0xF0); /* SEARCH ROM */



        while (bit_number <= 64)

        {

            uint8_t bit_value = OW_ReadBit();

            uint8_t complement = OW_ReadBit();

            uint8_t direction;



            if (bit_value && complement)

            {

                search_failed = 1;

                break;

            }



            if (bit_value != complement)

            {

                direction = bit_value;

            }

            else

            {

                if (bit_number < last_discrepancy)

                {

                    direction =

                        ((rom[rom_byte] & rom_mask) != 0);

                }

                else

                {

                    direction =

                        (bit_number == last_discrepancy);

                }



                if (direction == 0)

                    last_zero = bit_number;

            }



            if (direction)

                rom[rom_byte] |= rom_mask;



            OW_WriteBit(direction);



            bit_number++;

            rom_mask <<= 1;



            if (rom_mask == 0)

            {

                rom_byte++;

                rom_mask = 1;

            }

        }



        if (search_failed || bit_number <= 64)

            break;



        last_discrepancy = last_zero;



        if (last_discrepancy == 0)

            last_device = 1;



        if (rom[0] != 0x28 || DS_CRC8(rom, 7) != rom[7])

            continue;



        memcpy(ds_devices[found].rom, rom, 8);

        found++;

    }



    return found;

}



/* =================================================

   DS18B20 TEMPERATURE READING

   \================================================= */



void DS_Start_Conversion(void)

{

    if (!OW_Reset())

        return;



    OW_WriteByte(0xCC); /* SKIP ROM */

    OW_WriteByte(0x44); /* CONVERT T */

}



float DS_Read_Device_Temperature(uint8_t index)

{

    uint8_t scratchpad[9];



    if (index >= ds_count)

        return -127.0f;



    if (!OW_Reset())

        return -127.0f;



    OW_WriteByte(0x55); /* MATCH ROM */



    for (uint8_t i = 0; i < 8; i++)

        OW_WriteByte(ds_devices[index].rom[i]);



    OW_WriteByte(0xBE); /* READ SCRATCHPAD */



    for (uint8_t i = 0; i < 9; i++)

        scratchpad[i] = OW_ReadByte();



    if (DS_CRC8(scratchpad, 8) != scratchpad[8])

        return -127.0f;



    int16_t raw =

        (int16_t)((scratchpad[1] << 8) | scratchpad[0]);



    return (float)raw / 16.0f;

}



void Read_All_Temperatures(void)

{

    temperature1 = -127.0f;

    temperature2 = -127.0f;

    temperature3 = -127.0f;



    if (ds_count == 0)

        return;



    /* Start conversion on all connected sensors together. */

    DS_Start_Conversion();



    HAL_Delay(750);



    if (ds_count >= 1)

        temperature1 = DS_Read_Device_Temperature(0);



    if (ds_count >= 2)

        temperature2 = DS_Read_Device_Temperature(1);



    if (ds_count >= 3)

        temperature3 = DS_Read_Device_Temperature(2);

}



/* =================================================

   RS485 MODBUS - READ N, P, K

   \================================================= */



uint8_t Read_NPK(void)

{

    uint8_t request[8];

    uint8_t response[11];



    uint16_t crc;



    request[0] = MODBUS_ADDRESS;

    request[1] = 0x03;

    request[2] = 0x00;

    request[3] = 0x04;

    request[4] = 0x00;

    request[5] = 0x03;



    crc = Modbus_CRC16(request, 6);



    request[6] = crc & 0xFF;

    request[7] = (crc >> 8) & 0xFF;



    memset(response, 0, sizeof(response));



    if (HAL_UART_Transmit(&huart1, request, 8, MODBUS_TIMEOUT) != HAL_OK)

        return 0;



    if (HAL_UART_Receive(&huart1, response, 11, MODBUS_TIMEOUT) != HAL_OK)

        return 0;



    if (response[0] != MODBUS_ADDRESS ||

        response[1] != 0x03 ||

        response[2] != 0x06)

        return 0;



    uint16_t received_crc =

        (uint16_t)response[9] |

        ((uint16_t)response[10] << 8);



    uint16_t calculated_crc = Modbus_CRC16(response, 9);



    if (received_crc != calculated_crc)

        return 0;



    nitrogen =

        ((uint16_t)response[3] << 8) | response[4];



    phosphorus =

        ((uint16_t)response[5] << 8) | response[6];



    potassium =

        ((uint16_t)response[7] << 8) | response[8];



    return 1;

}



/* =================================================

   PUMP CONTROL

   \================================================= */



void Pump_Set(uint8_t state)

{

    GPIO_PinState output;



    if (RELAY_ACTIVE_LOW)

        output = state ? GPIO_PIN_RESET : GPIO_PIN_SET;

    else

        output = state ? GPIO_PIN_SET : GPIO_PIN_RESET;



    HAL_GPIO_WritePin(RELAY_PORT, RELAY_PIN, output);



    pump_state = state;

}



void Pump_Control(void)

{

    float average_moisture =

        (moisture1 + moisture2 + moisture3) / 3.0f;



    if (average_moisture < PUMP_ON_THRESHOLD)

    {

        Pump_Set(1);

    }

    else if (average_moisture >= PUMP_OFF_THRESHOLD)

    {

        Pump_Set(0);

    }



    /* Between 30% and 45%, keep the previous pump state. */

}



/* =================================================

   SERVO CONTROL - PA8 / TIM1 CHANNEL 1

   \================================================= */



void Servo_Set_Angle(uint8_t angle)

{

    if (angle > 180)

        angle = 180;



    uint32_t pulse =

        1000U + ((uint32_t)angle * 1000U / 180U);



    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse);

}



/* =================================================

   SEND DATA TO ESP32

   \================================================= */



void Send_Data_To_ESP32(void)

{

    char message[400];



    float average_moisture =

        (moisture1 + moisture2 + moisture3) / 3.0f;



    snprintf(message, sizeof(message),

        "{"

        "\\"D1_Moisture\\":%.1f,"

        "\\"D2_Moisture\\":%.1f,"

        "\\"D3_Moisture\\":%.1f,"

        "\\"D1_Temperature\\":%.2f,"

        "\\"D2_Temperature\\":%.2f,"

        "\\"D3_Temperature\\":%.2f,"

        "\\"N\\":%u,"

        "\\"P\\":%u,"

        "\\"K\\":%u,"

        "\\"NPK_valid\\":%u,"

        "\\"Average_Moisture\\":%.1f,"

        "\\"Pump\\":%u"

        "}\r\n",



        moisture1,

        moisture2,

        moisture3,



        temperature1,

        temperature2,

        temperature3,



        nitrogen,

        phosphorus,

        potassium,



        npk_valid,

        average_moisture,

        pump_state

    );



    HAL_UART_Transmit(

        &huart2,

        (uint8_t *)message,

        strlen(message),

        1000

    );

}



/* =================================================

   MAIN FUNCTION

   \================================================= */



int main(void)

{

    HAL_Init();



    SystemClock_Config();



    MX_GPIO_Init();

    MX_ADC1_Init();

    MX_USART1_UART_Init();

    MX_USART2_UART_Init();

    MX_TIM1_Init();



    DWT_Init();



    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)

        Error_Handler();



    Servo_Set_Angle(90);



    Pump_Set(0);



    HAL_Delay(1000);



    /* Find connected DS18B20 sensors. */

    ds_count = DS_Search_All();



while (1)
    {
        Read_Moisture_Sensors();
        Read_All_Temperatures();
        npk_valid = Read_NPK();
        Pump_Control();
        Send_Data_To_ESP32();
        HAL_Delay(5000);
    }

}



/* =================================================

   SYSTEM CLOCK - 84 MHz

   \================================================= */



void SystemClock_Config(void)

{

    RCC_OscInitTypeDef RCC_OscInitStruct = {0};

    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};



    __HAL_RCC_PWR_CLK_ENABLE();



    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);



    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;

    RCC_OscInitStruct.HSIState = RCC_HSI_ON;

    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;



    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;

    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;

    RCC_OscInitStruct.PLL.PLLM = 16;

    RCC_OscInitStruct.PLL.PLLN = 336;

    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;

    RCC_OscInitStruct.PLL.PLLQ = 7;



    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)

        Error_Handler();



    RCC_ClkInitStruct.ClockType =

        RCC_CLOCKTYPE_HCLK |

        RCC_CLOCKTYPE_SYSCLK |

        RCC_CLOCKTYPE_PCLK1 |

        RCC_CLOCKTYPE_PCLK2;



    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;

    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;

    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;

    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;



    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)

        Error_Handler();

}



/* =================================================

   GPIO INITIALIZATION

   \================================================= */



static void MX_GPIO_Init(void)

{

    GPIO_InitTypeDef GPIO_InitStruct = {0};



    __HAL_RCC_GPIOA_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();



    /* Relay PB5 */

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);



    GPIO_InitStruct.Pin = GPIO_PIN_5;

    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;

    GPIO_InitStruct.Pull = GPIO_NOPULL;

    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;



    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);



    /* DS18B20 PB6 - open drain */

    GPIO_InitStruct.Pin = GPIO_PIN_6;

    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;

    GPIO_InitStruct.Pull = GPIO_PULLUP;

    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;



    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);



    OW_Release();



    /* Moisture sensor ADC pins */

    GPIO_InitStruct.Pin =

        GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4;



    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;

    GPIO_InitStruct.Pull = GPIO_NOPULL;



    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);



    /* USART1 PA9 TX, PA10 RX */

    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;

    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;

    GPIO_InitStruct.Pull = GPIO_NOPULL;

    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;



    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);



    /* USART2 PA2 TX, PA3 RX */

    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;

    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;



    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);



    /* TIM1 CH1 servo PA8 */

    GPIO_InitStruct.Pin = GPIO_PIN_8;

    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;



    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}



/* =================================================

   ADC1 INITIALIZATION

   \================================================= */



static void MX_ADC1_Init(void)

{

    __HAL_RCC_ADC1_CLK_ENABLE();



    hadc1.Instance = ADC1;

    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;

    hadc1.Init.Resolution = ADC_RESOLUTION_12B;

    hadc1.Init.ScanConvMode = *DISABLE*;

    hadc1.Init.ContinuousConvMode = *DISABLE*;

    hadc1.Init.DiscontinuousConvMode = *DISABLE*;

    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;

    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;

    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;

    hadc1.Init.NbrOfConversion = 1;

    hadc1.Init.DMAContinuousRequests = *DISABLE*;

    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;



    if (HAL_ADC_Init(&hadc1) != HAL_OK)

        Error_Handler();

}



/* =================================================

   USART1 - RS485 - 4800 BAUD

   \================================================= */



static void MX_USART1_UART_Init(void)

{

    __HAL_RCC_USART1_CLK_ENABLE();



    huart1.Instance = USART1;

    huart1.Init.BaudRate = 4800;

    huart1.Init.WordLength = UART_WORDLENGTH_8B;

    huart1.Init.StopBits = UART_STOPBITS_1;

    huart1.Init.Parity = UART_PARITY_NONE;

    huart1.Init.Mode = UART_MODE_TX_RX;

    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;

    huart1.Init.OverSampling = UART_OVERSAMPLING_16;



    if (HAL_UART_Init(&huart1) != HAL_OK)

        Error_Handler();

}



/* =================================================

   USART2 - ESP32 - 115200 BAUD

   \================================================= */



static void MX_USART2_UART_Init(void)

{

    __HAL_RCC_USART2_CLK_ENABLE();



    huart2.Instance = USART2;

    huart2.Init.BaudRate = 115200;

    huart2.Init.WordLength = UART_WORDLENGTH_8B;

    huart2.Init.StopBits = UART_STOPBITS_1;

    huart2.Init.Parity = UART_PARITY_NONE;

    huart2.Init.Mode = UART_MODE_TX_RX;

    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;

    huart2.Init.OverSampling = UART_OVERSAMPLING_16;



    if (HAL_UART_Init(&huart2) != HAL_OK)

        Error_Handler();

}



/* =================================================

   TIM1 PWM - SERVO - 50 Hz

   \================================================= */



static void MX_TIM1_Init(void)

{

    TIM_OC_InitTypeDef sConfigOC = {0};



    __HAL_RCC_TIM1_CLK_ENABLE();



    htim1.Instance = TIM1;

    htim1.Init.Prescaler = 83;

    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;

    htim1.Init.Period = 19999;

    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

    htim1.Init.RepetitionCounter = 0;

    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;



    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)

        Error_Handler();



    sConfigOC.OCMode = TIM_OCMODE_PWM1;

    sConfigOC.Pulse = 1500;

    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;

    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;



    if (HAL_TIM_PWM_ConfigChannel(

            &htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)

        Error_Handler();

}



/* =================================================

   ERROR HANDLER

   \================================================= */



void Error_Handler(void)

{

    __disable_irq();



    while (1)

    {

        /* Stop here if initialization fails. */

    }

}
