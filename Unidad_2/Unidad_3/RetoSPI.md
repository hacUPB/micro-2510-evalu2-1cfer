## Proyecto: Comunicación SPI con STM32F407

En este proyecto, se utilizó la tarjeta STM32F407 para realizar una comunicación SPI (Serial Peripheral Interface) con un dispositivo periférico. A través de este protocolo, el STM32F407 actúa como maestro y se encarga de enviar y recibir datos de un dispositivo esclavo. 

### Configuración Inicial

1. **STMCubeMX**:
   Se usó STMCubeMX para generar la configuración básica del microcontrolador. Los pines utilizados para el bus SPI1 en el STM32F407 son:
   - **SCK (Clock)**: PA5
   - **MISO (Master In Slave Out)**: PA6
   - **MOSI (Master Out Slave In)**: PA7
   - **NSS (Slave Select)**: PA4

   Se habilitó el bus SPI1 en modo maestro y se configuraron los parámetros necesarios, como la polaridad y la fase del reloj, el tamaño de los datos y la velocidad del reloj.

2. **Configuración de SPI**:
   Se configuró el SPI en modo maestro, utilizando un prescaler de 2 para establecer la velocidad de transmisión. Además, se configuraron los pines de forma adecuada para la comunicación SPI.

3. **Código de la Aplicación**:
   Después de generar el código base con STMCubeMX, se añadió la lógica de comunicación SPI para transmitir y recibir datos con un periférico esclavo.

4. **Uso de HAL**:
   Se utilizaron las funciones proporcionadas por la librería HAL para gestionar la comunicación SPI. En particular, se usaron las funciones `HAL_SPI_Transmit()` y `HAL_SPI_Receive()` para enviar y recibir datos a través del bus SPI.

### Flujo del Código

- **Inicialización**:
  En primer lugar, se inicializan los periféricos: SPI1, GPIO (para los pines SPI) y el sistema de reloj.
  
- **Comunicación SPI**:
  El STM32F407 envía y recibe datos desde el dispositivo periférico utilizando el protocolo SPI. Esto se logra con las funciones `HAL_SPI_Transmit()` para enviar datos y `HAL_SPI_Receive()` para recibir.

- **Salida de Datos**:
  Los datos recibidos o enviados a través de SPI pueden ser procesados y visualizados en la consola o en una pantalla, dependiendo de la aplicación.

### Consideraciones

- **Configuración de los Pines**: Es crucial verificar que los pines del SPI estén correctamente configurados en el microcontrolador y el dispositivo periférico.
- **Control de Errores**: Si la comunicación SPI falla, es importante implementar un manejo adecuado de errores.
- **Velocidad de SPI**: La velocidad de transmisión del bus SPI depende de la configuración del prescaler y la frecuencia del reloj del sistema.

### Código Completo

```c
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <string.h>

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
float temperature = 0.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);

/* USER CODE BEGIN 0 */
float MAX6675_ReadTemperature(void) {
    uint8_t rxData[2];
    uint16_t rawData;

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // CS low
    HAL_SPI_Receive(&hspi1, rxData, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);   // CS high

    rawData = (rxData[0] << 8) | rxData[1];
    if (rawData & 0b0010) {
        return -1.0f; // Open thermocouple detected
    }

    rawData >>= 3; // Remove unused bits
    return rawData * 0.25f; // Convert to Celsius
}

void printTemperature(float temp) {
    char msg[64];

        sprintf(msg, "Temperature: %.2f C\r\n", temp);
        for (uint32_t i = 0; i < strlen(msg); i++) {
        ITM_SendChar(msg[i]);
    }
}
/* USER CODE END 0 */

/* Application entry point ---------------------------------------------------*/
int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_SPI1_Init();

    while (1) {
        temperature = MAX6675_ReadTemperature();
        printTemperature(temperature);
        HAL_Delay(500);
    }
}

/* Peripheral initialization functions ---------------------------------------*/
static void MX_SPI1_Init(void) {
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER; x
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

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
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
        Error_Handler();
    }
}

void Error_Handler(void) {
    __disable_irq();
    while (1) {}
}
```
