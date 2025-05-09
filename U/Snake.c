#include "stm32f4xx.h"
#include <stdlib.h>

#define GRID_SIZE 8
#define INITIAL_SPEED 1500  // 1.5 segundos (en ms)

typedef struct {
    int x;
    int y;
} Point;

// Variables del juego
Point snake[64];   // Máximo tamaño de la serpiente (toda la matriz)
int snake_length = 1;
Point food;
int direction = 1; // 0 = Arriba, 1 = Derecha, 2 = Abajo, 3 = Izquierda
int game_over = 0;
volatile uint32_t tick = 0;  // SysTick contador de milisegundos

void SysTick_Handler(void) {
    tick++;
}

void delay_ms(uint32_t ms) {
    uint32_t start = tick;
    while ((tick - start) < ms);
}

// Inicializa los pines GPIO para LEDs y botones
void GPIO_Init(void) {
    RCC->AHB1ENR |= (1 << 0); // Habilita GPIOA

    // Configurar pines de LEDs (salida)
    for (int i = 0; i < 16; i++) {
        GPIOA->MODER |= (1 << (i * 2)); // Modo salida
    }

    // Configurar botones (entrada con pull-up)
    RCC->AHB1ENR |= (1 << 1); // Habilita GPIOB
    for (int i = 0; i < 4; i++) {
        GPIOB->MODER &= ~(3 << (i * 2)); // Entrada
        GPIOB->PUPDR |= (1 << (i * 2));  // Pull-up
    }
}

// Función para leer los botones
void read_buttons(void) {
    if (!(GPIOB->IDR & (1 << 0)) && direction != 2) direction = 0; // Arriba
    if (!(GPIOB->IDR & (1 << 1)) && direction != 3) direction = 1; // Derecha
    if (!(GPIOB->IDR & (1 << 2)) && direction != 0) direction = 2; // Abajo
    if (!(GPIOB->IDR & (1 << 3)) && direction != 1) direction = 3; // Izquierda
}

// Genera una posición aleatoria para la comida
void generate_food(void) {
    int valid = 0;
    while (!valid) {
        food.x = rand() % GRID_SIZE;
        food.y = rand() % GRID_SIZE;
        valid = 1;

        for (int i = 0; i < snake_length; i++) {
            if (snake[i].x == food.x && snake[i].y == food.y) {
                valid = 0;
                break;
            }
        }
    }
}

// Mueve la serpiente en la dirección actual
void move_snake(void) {
    if (game_over) return;

    // Guarda la posición anterior de la cabeza
    Point new_head = snake[0];

    // Mueve la cabeza en la dirección actual
    switch (direction) {
        case 0: new_head.y = (new_head.y - 1 + GRID_SIZE) % GRID_SIZE; break; // Arriba
        case 1: new_head.x = (new_head.x + 1) % GRID_SIZE; break; // Derecha
        case 2: new_head.y = (new_head.y + 1) % GRID_SIZE; break; // Abajo
        case 3: new_head.x = (new_head.x - 1 + GRID_SIZE) % GRID_SIZE; break; // Izquierda
    }

    // Verifica si la cabeza choca con el cuerpo
    for (int i = 1; i < snake_length; i++) {
        if (snake[i].x == new_head.x && snake[i].y == new_head.y) {
            game_over = 1;
            return;
        }
    }

    // Desplaza el cuerpo de la serpiente
    for (int i = snake_length; i > 0; i--) {
        snake[i] = snake[i - 1];
    }

    // Actualiza la cabeza
    snake[0] = new_head;

    // Si la cabeza come la comida, la serpiente crece
    if (new_head.x == food.x && new_head.y == food.y) {
        snake_length++;
        generate_food();
    }
}

// Renderiza los LEDs con multiplexado (un LED a la vez)
void render_leds(void) {
    if (game_over) return;

    for (int y = 0; y < GRID_SIZE; y++) {
        for (int x = 0; x < GRID_SIZE; x++) {
            int led_on = 0;

            for (int i = 0; i < snake_length; i++) {
                if (snake[i].x == x && snake[i].y == y) {
                    led_on = 1;
                    break;
                }
            }

            if (food.x == x && food.y == y) led_on = 1;

            if (led_on) {
                GPIOA->ODR |= (1 << (x + y * GRID_SIZE));
                delay_ms(1);
                GPIOA->ODR &= ~(1 << (x + y * GRID_SIZE));
            }
        }
    }
}

// Función principal
int main(void) {
    SysTick_Config(SystemCoreClock / 1000);  // Configura SysTick cada 1ms
    GPIO_Init();
    generate_food();

    snake[0].x = GRID_SIZE / 2;
    snake[0].y = GRID_SIZE / 2;

    uint32_t last_move = 0;

    while (1) {
        read_buttons();

        if ((tick - last_move) >= INITIAL_SPEED) {
            move_snake();
            last_move = tick;
        }

        render_leds();

        if (game_over) {
            GPIOA->ODR = 0x0000;  // Apaga todos los LEDs
            while (1);  // Loop infinito
        }
    }
}
