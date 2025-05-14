
## Proyecto: Lectura de Sensor Analógico con STM32F407

En este proyecto, se utilizó la tarjeta STM32F407 y STMCubeMX para leer un valor de un sensor analógico, en este caso una fotoresistencia conectada al pin PA0. El objetivo fue configurar y programar el ADC del microcontrolador para realizar conversiones de la señal analógica a digital y mostrar los resultados en la consola de STMCube.

### Configuración Inicial

1. **STMCubeMX**:
   Se usó STMCubeMX para configurar el proyecto en el entorno de desarrollo, donde se configuró el ADC1 para que funcione en modo de conversión continua. También se habilitó el reloj para el puerto GPIOA, donde está conectado el sensor analógico. La resolución del ADC fue configurada en 12 bits y se eligió el canal 0 (PA0), que es donde está conectada la fotoresistencia.

2. **Configuración del ADC**:
   El ADC se configuró en modo de conversión continua con el prescaler de reloj adecuado para que funcione correctamente. El ADC está configurado para realizar conversiones de un solo canal (el canal 0) con una resolución de 12 bits.

3. **Código de la Aplicación**:
   Posteriormente, se generó el código base con STMCubeMX y se modificó para iniciar las conversiones del ADC de manera manual y leer el valor convertido. El valor del ADC se imprimió en la consola del sistema usando la funcionalidad de ITM (Instrumentation Trace Macrocell), lo que permitió ver los resultados de las conversiones en tiempo real.

4. **Uso de HAL**:
   Se utilizó la librería HAL para interactuar con el hardware, iniciar el ADC, realizar las conversiones y obtener los valores de las lecturas. El valor ADC se muestra en la consola con la función `printf`.

### Flujo del Código

- **Inicialización**:
  Se inicializan los periféricos necesarios: ADC, GPIO y configuración del sistema de reloj.
  
- **Lectura del ADC**:
  Se inicia la conversión ADC de manera manual, se espera a que la conversión esté lista, y luego se lee el valor convertido.

- **Salida de Datos**:
  El valor leído se imprime en la consola de STMCube usando `printf` a través de ITM.

### Código Completo

```c

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;  // Handle para el ADC

/* USER CODE BEGIN PV */
uint32_t value_adc;  // Variable para almacenar el valor leído por el ADC
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
int _write(int file, char *ptr, int len);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  Punto de entrada principal de la aplicación.
  * @retval int
  */
int main(void)
{
    // Inicialización de la HAL y configuración de sistemas
    HAL_Init();
    SystemClock_Config();  // Configura el reloj del sistema
    MX_GPIO_Init();       // Inicializa los GPIO
    MX_ADC1_Init();       // Inicializa el ADC

    /* USER CODE BEGIN 2 */
    HAL_ADC_Start(&hadc1);  // Inicia la conversión del ADC
    /* USER CODE END 2 */

    /* Bucle principal */
    while (1)
    {
        // Inicia una conversión ADC de manera manual
        HAL_ADC_Start(&hadc1);

        // Espera hasta que la conversión termine
        if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
        {
            // Obtiene el valor de la conversión
            value_adc = HAL_ADC_GetValue(&hadc1);

            // Imprime el valor del ADC a través del puerto ITM
            printf("Valor del ADC: %lu\r\n", value_adc);
        }

        // Retardo entre cada lectura
        HAL_Delay(100);
    }
}

/**
  * @brief Configuración del reloj del sistema
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /* Configura el regulador de voltaje interno */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /* Inicializa los osciladores del RCC */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /* Inicializa los relojes de los buses del CPU, AHB y APB */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief Inicialización del ADC1
  * @retval None
  */
static void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    /* Configuración del ADC1 */
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = DISABLE;
    hadc1.Init.ContinuousConvMode = ENABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;

    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }

    /* Configura el canal regular del ADC */
    sConfig.Channel = ADC_CHANNEL_0;  // Canal 0 (PA0)
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief Inicialización de los GPIO
  * @retval None
  */
static void MX_GPIO_Init(void)
{
    /* Habilitar reloj para GPIOA */
    __HAL_RCC_GPIOA_CLK_ENABLE();
}

/**
  * @brief Función de manejo de errores
  * @retval None
  */
void Error_Handler(void)
{
    /* Aquí puedes añadir tu propio código para manejar errores */
    __disable_irq();
    while (1)
    {
        // Bucle infinito para indicar error
    }
}

/* Redirección de printf a través de ITM */
int __io_putchar(int ch)
{
    ITM_SendChar(ch);  // Enviar un carácter a través de ITM
    return ch;
}

int _write(int file, char *ptr, int len)
{
    for (int i = 0; i < len; i++)
    {
        ITM_SendChar((*ptr++));  // Enviar cada carácter uno por uno
    }
    return len;
}
```