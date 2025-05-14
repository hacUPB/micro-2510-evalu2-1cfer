## Proyecto: Lectura de Sensor I2C con STM32F407

En este proyecto, se utilizó la tarjeta STM32F407 y STMCubeMX para leer un valor de un sensor I2C. A diferencia del sensor analógico, en este caso la comunicación es digital a través del protocolo I2C. El objetivo fue configurar el bus I2C del microcontrolador, leer los datos del sensor, y mostrarlos en la consola del sistema.

### Configuración Inicial

1. **STMCubeMX**:
   Se usó STMCubeMX para configurar el proyecto en el entorno de desarrollo. En este caso, se configuraron los pines necesarios para el bus I2C. Los pines predeterminados para I2C1 en el STM32F407 son:
   - **SCL (Clock Line)**: PB6
   - **SDA (Data Line)**: PB7

   Se habilitó el bus I2C1 y se configuraron los parámetros adecuados para la comunicación, como la velocidad de transmisión (por ejemplo, 100kHz o 400kHz) y el modo de 7 bits o 10 bits para las direcciones.

2. **Configuración de I2C**:
   El I2C fue configurado en modo maestro, con la velocidad de reloj adecuada y la dirección del dispositivo sensor definida. Además, se habilitaron las interrupciones de I2C si es necesario, para mejorar la eficiencia de la lectura.

3. **Código de la Aplicación**:
   Después de generar el código base con STMCubeMX, se modificó el código para realizar la comunicación I2C con el sensor. En este caso, se utilizó la función `HAL_I2C_Master_Transmit()` para enviar comandos al sensor y `HAL_I2C_Master_Receive()` para leer los datos desde el sensor.

4. **Uso de HAL**:
   Se utilizaron las funciones de la librería HAL para gestionar la comunicación I2C, y se empleó la función `printf` para mostrar los valores leídos en la consola.

### Flujo del Código

- **Inicialización**:
  Se inicializan los periféricos necesarios: I2C, GPIO (para los pines SCL y SDA) y configuración del sistema de reloj.

- **Comunicación I2C**:
  Se envían comandos al sensor para leer datos a través del bus I2C. Dependiendo del sensor, puede ser necesario enviar una dirección de registro primero, seguida de la lectura de los datos desde esa dirección.

- **Salida de Datos**:
  El valor leído desde el sensor se imprime en la consola de STMCube usando `printf`.

### Consideraciones

- **Dirección del Sensor**: La dirección I2C del sensor puede variar, por lo que es importante verificar en el datasheet del sensor cuál es la dirección correcta a utilizar.
- **Tiempo de Espera**: Es importante tener en cuenta los tiempos de respuesta del sensor, especialmente si se utilizan interrupciones o funciones no bloqueantes.
- **Errores de Comunicación**: Si la comunicación I2C falla, se debe manejar adecuadamente con códigos de error, como errores de tiempo de espera o fallos en la transmisión.

### Código Completo

```c
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HMC5883L_ADDRESS 0x1E << 1 // Dirección I2C (0x1E en 7-bit, desplazada para HAL)
#define HMC5883L_REG_CONFIG_A 0x00
#define HMC5883L_REG_CONFIG_B 0x01
#define HMC5883L_REG_MODE     0x02
#define HMC5883L_REG_DATA_X_MSB 0x03
#define HMC5883L_REG_DATA_X_LSB 0x04
#define HMC5883L_REG_DATA_Z_MSB 0x05
#define HMC5883L_REG_DATA_Z_LSB 0x06
#define HMC5883L_REG_DATA_Y_MSB 0x07
#define HMC5883L_REG_DATA_Y_LSB 0x08
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
int16_t magData[3]; // Array para X, Y, Z
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void HMC5883L_Init(void);
void HMC5883L_ReadData(int16_t *magData);
void printMagData(int16_t x, int16_t y, int16_t z);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Inicializa el HMC5883L
void HMC5883L_Init(void) {
    uint8_t configA = 0x70; // 8 muestras promedio, 15Hz, modo normal
    uint8_t configB = 0xA0; // Ganancia ±1.3 Ga (1090 LSB/Gauss)
    uint8_t mode = 0x00;    // Modo continuo

    HAL_I2C_Mem_Write(&hi2c1, HMC5883L_ADDRESS, HMC5883L_REG_CONFIG_A, 1, &configA, 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Write(&hi2c1, HMC5883L_ADDRESS, HMC5883L_REG_CONFIG_B, 1, &configB, 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Write(&hi2c1, HMC5883L_ADDRESS, HMC5883L_REG_MODE, 1, &mode, 1, HAL_MAX_DELAY);
}

// Lee los 3 ejes (X, Y, Z)
void HMC5883L_ReadData(int16_t *magData) {
    uint8_t buffer[6]; // Buffer para X, Z, Y (MSB + LSB cada uno)
    HAL_I2C_Mem_Read(&hi2c1, HMC5883L_ADDRESS, HMC5883L_REG_DATA_X_MSB, 1, buffer, 6, HAL_MAX_DELAY);

    // Orden de registros: X, Z, Y (ver datasheet)
    magData[0] = (int16_t)((buffer[0] << 8) | buffer[1]); // X
    magData[1] = (int16_t)((buffer[4] << 8) | buffer[5]); // Y
    magData[2] = (int16_t)((buffer[2] << 8) | buffer[3]); // Z
}

// Envía datos a SWV ITM Console
void printMagData(int16_t x, int16_t y, int16_t z) {
    char msg[64];
    sprintf(msg, "X: %d, Y: %d, Z: %d\r\n", x, y, z);
    for (uint32_t i = 0; i < strlen(msg); i++) {
        ITM_SendChar(msg[i]);
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
    /* MCU Configuration--------------------------------------------------------*/
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();

    /* USER CODE BEGIN 2 */
    HMC5883L_Init(); // Configura el magnetómetro
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        HMC5883L_ReadData(magData); // Lee datos
        printMagData(magData[0], magData[1], magData[2]); // Muestra en SWV
        HAL_Delay(500); // Espera 500ms
        /* USER CODE END WHILE */
    }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void) {
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_ENABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void) {
    __HAL_RCC_GPIOB_CLK_ENABLE();
}

/* USER CODE BEGIN 4 */
/* (Las funciones HMC5883L ya están definidas en USER CODE BEGIN 0) */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
    __disable_irq();
    while (1) {}
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif /* USE_FULL_ASSERT */
```