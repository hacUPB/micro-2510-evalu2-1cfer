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

```