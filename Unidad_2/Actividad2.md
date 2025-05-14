## 📤 **Ejercicio 1 - para subir al repositorio**

### 1️⃣ **Lenguajes para Programar Sistemas Embebidos**

- **C**  
- **C++**  
- **Assembly**  
- **Python (MicroPython, CircuitPython)**  
- **Rust**  
- **Ada**  
- **Java (Embedded Java)**  
- **Lua**  
- **Go**  
- **MATLAB/Simulink**  

### 2️⃣ **Ventajas y Desventajas Comparadas con C**

| Lenguaje         | Ventajas                                                                 | Desventajas                                                                   |
|-------------------|--------------------------------------------------------------------------|-------------------------------------------------------------------------------|
| **C**            | Eficiencia, control de hardware, ampliamente soportado.                 | Complejidad para aplicaciones modernas, falta de seguridad en memoria.       |
| **C++**          | Soporte para OOP, mejor estructura de código.                           | Mayor uso de recursos, mayor complejidad.                                    |
| **Assembly**      | Control absoluto sobre el hardware, máximo rendimiento.                | Baja productividad, difícil mantenimiento, no portátil.                      |
| **Python**        | Facilidad de uso, alta productividad, comunidad extensa.               | No es adecuado para tareas críticas en tiempo real, mayor consumo de memoria.|
| **Rust**          | Seguridad en memoria, alta eficiencia.                                 | Curva de aprendizaje pronunciada, menor soporte en sistemas embebidos.       |
| **Ada**           | Fiabilidad, robustez en sistemas críticos.                             | Uso limitado, curva de aprendizaje alta.                                     |
| **Java**          | Portabilidad, herramientas avanzadas.                                  | Requiere una JVM, mayor consumo de recursos.                                 |
| **Lua**           | Ligero, fácil de integrar como lenguaje embebido en sistemas mayores.  | Limitado para tareas complejas.                                              |
| **Go**            | Concurrencia nativa, eficiente en sistemas multicore.                 | Menor soporte en hardware específico.                                        |
| **MATLAB/Simulink** | Ideal para modelado y simulación.                                     | Alto costo, no siempre adecuado para producción directa.                     |

### 3️⃣ **Ranking de Lenguajes para Sistemas Embebidos**

Un ranking reconocido es el informe de **IEEE Spectrum**. En su sección de lenguajes de programación, frecuentemente incluye análisis de lenguajes populares para sistemas embebidos.

**Link al Ranking:** [IEEE Spectrum - Top Programming Languages](https://spectrum.ieee.org/top-programming-languages)  

**Percepción:** C sigue siendo el lenguaje líder en sistemas embebidos debido a su eficiencia y control de hardware. Sin embargo, lenguajes como Rust y Python están ganando popularidad por su enfoque en la seguridad y productividad, respectivamente.

## 🧾 **Ejercicio 2**

### **Descripción**

Se solicita crear un script para automatizar la configuración inicial y escritura de código en C para el IDE utilizado en la programación del microcontrolador. Este script deberá ser reutilizable para los próximos ejercicios.

### **Script**

A continuación, se presenta un script de ejemplo en Python que genera automáticamente un archivo de código base en C con una estructura típica para proyectos de microcontroladores:

```python
import os

# Configuración
proyecto = "mi_proyecto_embebido"
archivo_c = "main.c"
directorio = os.path.join(os.getcwd(), proyecto)

# Estructura base del código en C
codigo_base = """#include <stdio.h>

// Declaraciones de funciones
void setup();
void loop();

// Programa principal
int main() {
    setup();
    while (1) {
        loop();
    }
    return 0;
}

// Definición de funciones
void setup() {
    // Configuración inicial
}

void loop() {
    // Código principal
}
"""

# Crear el directorio y el archivo
os.makedirs(directorio, exist_ok=True)
ruta_archivo = os.path.join(directorio, archivo_c)

with open(ruta_archivo, "w") as archivo:
    archivo.write(codigo_base)

print(f"Proyecto creado en: {directorio}")
print(f"Archivo generado: {ruta_archivo}")

## #️⃣ Ejercicio 3

A continuación, se presentan las macros diseñadas para manipular registros en sistemas embebidos. Estas macros son ejemplos prácticos aplicables en microcontroladores.

### **1️⃣ Aplicar una máscara a un registro**

**Descripción:** Permite modificar bits específicos en un registro sin afectar el resto. 

```c
// Macro para aplicar una máscara a un registro
#define SET_REGISTER_MASK(REG, MASK) ((REG) |= (MASK))

// Ejemplo de uso
// Supongamos que PORTA representa un registro de puerto y 0x0F es la máscara
SET_REGISTER_MASK(PORTA, 0x0F); // Activa los bits 0-3 del registro PORTA
```
### **2️⃣ Verificar si un periférico está habilitado**
**Descripción:** Determina si un periférico está presente consultando su bit en un registro de control.

```c
// Macro para verificar si un periférico está habilitado
#define IS_PERIPHERAL_ENABLED(REG, BIT) (((REG) & (1 << (BIT))) != 0)

// Ejemplo de uso
// Suponiendo que PCC_PORTC es un registro de control y el bit 5 indica la habilitación
if (IS_PERIPHERAL_ENABLED(PCC_PORTC, 5)) {
    // El periférico está habilitado
}
```
### **3️⃣ Alternar un bit en un registro**
**Descripción:** Cambia el estado de un bit específico en un registro.

```c
// Macro para alternar un bit de un registro
#define TOGGLE_BIT(REG, BIT) ((REG) ^= (1 << (BIT)))

// Ejemplo de uso
// Suponiendo que GPIOB representa un registro y queremos alternar el bit 2
TOGGLE_BIT(GPIOB, 2); // Cambia el estado del bit 2 del registro GPIOB
```

### **4️⃣ Macro personalizada: Limpiar un bit específico**
**Descripción:** Resetea un bit específico en un registro, útil para desactivar funciones o limpiar banderas.

```c
// Macro para limpiar un bit en un registro
#define CLEAR_BIT(REG, BIT) ((REG) &= ~(1 << (BIT)))

// Ejemplo de uso
// Suponiendo que STATUS_REGISTER tiene un bit de estado en la posición 3
CLEAR_BIT(STATUS_REGISTER, 3); // Limpia el bit 3 del registro STATUS_REGISTER
```

## 🧾 **Ejercicio 4**


1. **Bucle infinito en el primer `for`:**
   - **Problema:** `i--` hace que `i` nunca llegue a 10.
   - **Solución:** Cambiar `i--` por `i++` para incrementar `i`.
   ```c
   for (i = 1; i < 10; i++) {
       printf("Valor de i: %d\n", i);
   }
   ```
2. **BAcceso fuera de los límites del arreglo en el segundo`for`:**
   - **Problema:** Se accede a `array[5]`, que está fuera de los límites.
   - **Solución:** ambiar la condición a `i < 5`.

```c
for (i = 0; i < 5; i++) {
    printf("Elemento del array: %d\n", array[i]);
}
```

3. **Codigo completo Corregido**
```c
#include <stdio.h>

int main() {
    int i;
    int num = 10;
    int array[5] = {1, 2, 3, 4, 5};
    int contador = 0;

    for (i = 1; i < 10; i++) {
        printf("Valor de i: %d\n", i);
    }

    for (i = 0; i < 5; i++) {
        printf("Elemento del array: %d\n", array[i]);
    }

    while (num != 0) {
        printf("Valor de num: %d\n", num);
        num = num - 1;  
    }

    while (contador < 5) {
        printf("Valor de contador: %d\n", contador);
        contador++;
    }

    return 0;
}

```

