## 2. Preguntas de control

1. **¿Por qué usamos un temporizador (30 ms) para validar el estado del pulsador?**
    - **Justificación:** Usar un temporizador con un retardo de 30 ms ayuda a filtrar los efectos de los rebotes mecánicos que pueden ocurrir cuando el pulsador cambia de estado. Los rebotes son cambios rápidos e indeseados que ocurren al presionar o soltar el pulsador, lo que podría causar múltiples lecturas erróneas del estado del pulsador. Al aplicar un retardo, nos aseguramos de que el pulsador se estabilice antes de leer su estado, evitando lecturas falsas y proporcionando una lectura más confiable.

2. **¿En qué se diferencia una MEF de tipo Moore de una Mealy y por qué en antirrebote es más común Moore?**
    - **Diferencia entre MEF de Moore y Mealy:**
      - En una **MEF de tipo Moore**, las salidas dependen exclusivamente de los **estados** del sistema. Es decir, para cada estado, hay una salida definida.
      - En una **MEF de tipo Mealy**, las salidas dependen tanto de los **estados** como de las **entradas**. Esto significa que las salidas pueden cambiar en función de las entradas en cualquier momento, no solo cuando el sistema cambia de estado.
    - **Razón de uso de Moore en antirrebote:** En el caso del antirrebote, es preferible usar una MEF de tipo Moore porque las salidas son más estables y no dependen de las fluctuaciones rápidas en las entradas (como los rebotes del pulsador). Esto asegura que el sistema solo cambie su salida cuando se ha confirmado que el estado del pulsador ha cambiado de manera estable, lo que es crucial para evitar lecturas incorrectas debido a rebotes.

3. **¿Cómo podrías escalar este código para manejar múltiples pulsadores en simultáneo?**
    - **Escalabilidad del código:**
      Para manejar múltiples pulsadores simultáneamente, podemos adaptar el código utilizando una **estructura de datos**, como un arreglo o un conjunto de registros, para almacenar los estados y tiempos de cada pulsador. Cada pulsador tendría su propio contador de rebote (`lastChangeTime`) y su estado de debounce. La función de actualización debería iterar sobre todos los pulsadores, validando el estado de cada uno de manera independiente. Se pueden usar múltiples máquinas de estados o una estructura común que valide todos los pulsadores en paralelo, actualizando sus respectivos estados y tiempos sin interferir entre sí.

4. **¿Por qué es importante actualizar el `lastChangeTime` cuando se pasa a un estado de debounce?**
    - **Función de `lastChangeTime`:** La variable `lastChangeTime` es crucial para asegurar que el cambio de estado del pulsador se haya estabilizado antes de ser procesado. Cuando el pulsador pasa a un estado de debounce, significa que se ha detectado un cambio en el estado (presionado o liberado) pero se necesita un tiempo de espera para asegurarse de que no haya más rebotes. Actualizar `lastChangeTime` en ese momento garantiza que solo se considerarán cambios estables en el pulsador después del tiempo de espera, evitando lecturas falsas y asegurando que el sistema responda solo cuando el estado del pulsador esté confirmado.


## Codigo Leds

Este código está diseñado para controlar el parpadeo de tres LEDs conectados a los pines PA5, PA6 y PA7 de un microcontrolador STM32F4xx. Utiliza el temporizador SysTick para generar interrupciones con diferentes frecuencias y hacer que los LEDs se enciendan y apaguen secuencialmente. La frecuencia de parpadeo de los LEDs se almacena en una lista de frecuencias que el programa recorre en un ciclo continuo. 

Este código está escrito en lenguaje ensamblador, optimizado para el STM32F4xx y no utiliza la biblioteca estándar de C/C++. Esto significa que no se inicializa el entorno de ejecución de C, lo que lo hace adecuado para aplicaciones de bajo nivel. 

```c
/*********************************************************************
*                    SEGGER Microcontroller GmbH                     *
*                        The Embedded Experts                        *
**********************************************************************
*                                                                    *
*            (c) 2014 - 2024 SEGGER Microcontroller GmbH             *
*                                                                    *
*       www.segger.com     Support: support@segger.com               *
*                                                                    *
**********************************************************************
*                                                                    *
* All rights reserved.                                               *
*                                                                    *
* Redistribution and use in source and binary forms, with or         *
* without modification, are permitted provided that the following    *
* condition is met:                                                  *
*                                                                    *
* - Redistributions of source code must retain the above copyright   *
*   notice, this condition and the following disclaimer.             *
*                                                                    *
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND             *
* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,        *
* INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF           *
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE           *
* DISCLAIMED. IN NO EVENT SHALL SEGGER Microcontroller BE LIABLE FOR *
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR           *
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT  *
* OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;    *
* OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF      *
* LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT          *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE  *
* USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH   *
* DAMAGE.                                                            *
*                                                                    *
*********************************************************************/

/*********************************************************************
*
*       _start
*
*  Function description
*  Defines entry point for an STM32F4xx assembly code only
*  application.
*
*  Additional information
*    Please note, as this is an assembly code only project, the C/C++ 
*    runtime library has not been initialised. So do not attempt to call 
*    any C/C++ library functions because they probably won't work.
*/
.syntax unified
.global _start
.text
.thumb_func

/* Definiciones de direcciones y offsets */
.equ RCC_BASE, 0x40023800 /*Dirección base del registro del sistema de reloj y control*/
.equ RCC_AHB1ENR_OFFSET, 0x30 /*Offset de registro que habilita los relojes de los perifericos en el bus*/
.equ GPIOA_BASE, 0x40020000 /* Dirección base para habilitar el puerto GPIOA */
.equ GPIOA_MODER_OFFSET, 0x0 /*Offset de registro para configurar el modo de los pines del GPIOA*/
.equ GPIOx_OTYPER_OFFSET, 0x4 /*Ofsset de registro que define la salida push- pull oder open drain*/
.equ GPIOx_ODR_OFFSET, 0x14 /* Offset de registro de los datos de salida que permiten encender o no los pines */
.equ SYST_CSR, 0xE000E010 /*Registros de configuración del temporizador del Systick*/
.equ SYST_RVR, 0xE000E014
.equ SYST_CVR, 0xE000E018

.data /* Se almacena en la RAM, se usa para definir variables, constantes etc, es donde se almacenarüan valores incialiyadps */
frecuencias: /*Etiqueta*/
    .word 2000000, 4000000, 6000000 /* Diferentes períodos de parpadeo 125, 250, 375*/
    .word 0 /* Fin de la lista */

.text
_start:
    /* Habilitar el reloj para GPIOA */
    LDR R0, =RCC_BASE /*Carga el valor de RCC_ BASE en R0*/
    LDR R1, [R0, #RCC_AHB1ENR_OFFSET] /* Carga el valor actualizado del registro AHB...*/
    ORR R1, R1, #0x01 /*Activa el bit 0, lo que habilita el reloj para el GPIOA*/
    STR R1, [R0, #RCC_AHB1ENR_OFFSET] /*Guarda el valor del registro modificado*/

    /* Configurar los pines 5, 6 y 7 como salida */
    LDR R0, =GPIOA_BASE
    LDR R1, [R0, #GPIOA_MODER_OFFSET]
    BIC R1, R1, #(0b11 << 10) | (0b11 << 12) | (0b11 << 14) /* Limpiar MODER5, MODER6, MODER7 */
    ORR R1, R1, #(0b01 << 10) | (0b01 << 12) | (0b01 << 14) /* Configurar como salida */
    STR R1, [R0, #GPIOA_MODER_OFFSET]

    /* Configurar pines en modo push-pull */
    LDR R1, [R0, #GPIOx_OTYPER_OFFSET]
    BIC R1, R1, #(0b1 << 5) | (0b1 << 6) | (0b1 << 7) /* PA5, PA6, PA7 push-pull */
    STR R1, [R0, #GPIOx_OTYPER_OFFSET]

    /* Inicializar LEDs apagados */
    MOV R1, #0
    STR R1, [R0, #GPIOx_ODR_OFFSET]

    /* Configurar SysTick */
    LDR R4, =frecuencias /* Cargar la dirección de la lista de frecuencias */
    MOV R5, #0 /* Contador de LED */
loop:
    LDR R1, [R4], #4 /* Leer frecuencia y avanzar en la lista */
    CMP R1, #0
    BEQ restart_frequencies

    /* Configurar SysTick con la nueva frecuencia */
    LDR R0, =SYST_RVR
    STR R1, [R0] /*Carga la nueva frecuencia en R0 q se encuentra en R1 */
    LDR R0, =SYST_CVR /* Carga la dirección del Systick en R0 */
    MOV R1, #0 /* Carga 0 en R1 para reiniciar el contador*/
    STR R1, [R0]   /*Resetea el Systick-GUARDA 0 EN  SYST*/
    LDR R0, =SYST_CSR /* Carga la dirección del Systick*/
    MOV R1, #0x07 /* Carga 0x07 en R1, Bit 2 clicksourse = 1 usa el reloj del procesador, Bit 1 = 1 HABILITA LA INTERRUPCIÓN DEL SYSTICK, Bit 0= 1 ACTIVA EL SYSTICK*/
    STR R1, [R0] /* Guarda 0x07 en SYST_CSR, activando SysTick */

    /* Esperar interrupciones de SysTick */
    WFI /* Wait for interrupti */
    B loop /* cuando ocurre la interrupción vuelve al inicio del bucle */

restart_frequencies:
    LDR R4, =frecuencias /* Carga nuevamente la dirección de la lista de frecuencias */
    B loop /* Vuelve al incio del Bucle */

.global SysTick_Handler
.thumb_func
SysTick_Handler:
    LDR R0, =GPIOA_BASE /* Carga la dirección base del puertp GPIOA */
    LDR R2, [R0, #GPIOx_ODR_OFFSET] /* Carga el estado actual de los pines en R2 (Registro ODR) */

    CMP R5, #0 /* Compara R5 con 0 1 2 para saber q led debe prender */
    BEQ encender_PA6 /* Si r5= 0 va encender  PA6*/
    CMP R5, #1 /* Si r5= 1 va encender  PA7*/
    BEQ encender_PA7
    CMP R5, #2/* Si r5= 2 va encender  PA5*/
    BEQ encender_PA5

    MOV R5, #0 /* Si no coincide con ninguno reinicia R5=0 * Y RETORNA BX LR*/
    BX LR

encender_PA6:
    BIC R2, R2, #( (0b1 << 7) | (0b1 << 5) )  /* Apagar PA7 y PA5 */
    ORR R2, R2, #(0b1 << 6)  
    STR R2, [R0, #GPIOx_ODR_OFFSET]
    MOV R5, #1
    BX LR

encender_PA7:
    BIC R2, R2, #( (0b1 << 6) | (0b1 << 5) )  /* Apagar PA6 y PA5 */
    ORR R2, R2, #(0b1 << 7)  
    STR R2, [R0, #GPIOx_ODR_OFFSET]
    MOV R5, #2
    BX LR

encender_PA5:
    BIC R2, R2, #( (0b1 << 6) | (0b1 << 7) )  /* Apagar PA6 y PA7 */
    ORR R2, R2, #(0b1 << 5)  
    STR R2, [R0, #GPIOx_ODR_OFFSET]
    MOV R5, #0
    BX LR
```