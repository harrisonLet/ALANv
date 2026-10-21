#include "system.h"
#include "console.h"
#include "shared_configuration.h"

#include "servoSail.h"
#include "servoRudder.h"
#include "queue.h"

/*
 * Serial command console on USART3 (the ST-LINK virtual COM port, 115200 baud).
 * Real board:  screen /dev/tty.usbmodem* 115200
 * Simulator:   screen /tmp/alan-board 115200   (or the console box in the dashboard's Bench tab)
 *
 *   tail <deg>        set the tail (sail) servo, -90..90
 *   rudder <deg>      set the rudder servo, -90..90
 *   center            both servos to 0
 *   sweep tail|rudder sweep a servo end to end and back to 0
 *   status            print the last commanded angles
 *   help              this list
 *
 * Characters arrive by interrupt and are queued, so the task sleeps until you type (no busy-waiting).
 */

#define CONSOLE_LINE_MAX 48

TaskHandle_t task_console;
static QueueHandle_t rx_queue;
static int16_t tail_deg = 0;
static int16_t rudder_deg = 0;

void console_hardwareInit(void)
{
    rx_queue = xQueueCreate(64, sizeof(char));
    // USART3 is already set up by BSP_COM_Init in main.c; just turn on its receive interrupt
    HAL_NVIC_SetPriority(USART3_IRQn, 6, 0);      // >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY (5)
    HAL_NVIC_EnableIRQ(USART3_IRQn);
    __HAL_UART_ENABLE_IT(&hcom_uart[COM1], UART_IT_RXNE);
}

void USART3_IRQHandler(void)
{
    USART_TypeDef *u = hcom_uart[COM1].Instance;
    BaseType_t woken = pdFALSE;
    if (u->ISR & USART_ISR_ORE) { u->ICR = USART_ICR_ORECF; }       // overrun: drop it, keep going
    while (u->ISR & USART_ISR_RXNE_RXFNE)
    {
        char c = (char)(u->RDR & 0xFF);
        if (rx_queue) { xQueueSendFromISR(rx_queue, &c, &woken); }
    }
    portYIELD_FROM_ISR(woken);
}

static void set_tail(int v)
{
    if (v > 90) v = 90;
    if (v < -90) v = -90;
    tail_deg = (int16_t)v;
#ifdef CM7_SERVO_SAIL_ENABLE
    servoSail_setAngle(tail_deg);
#endif
}

static void set_rudder(int v)
{
    if (v > 90) v = 90;
    if (v < -90) v = -90;
    rudder_deg = (int16_t)v;
#ifdef CM7_SERVO_RUDDER_ENABLE
    servoRudder_setAngle(rudder_deg);
#endif
}

static void sweep(void (*set)(int), int limit)
{
    for (int a = 0; a <= limit; a += 5)   { set(a); vTaskDelay(pdMS_TO_TICKS(40)); }
    for (int a = limit; a >= -limit; a -= 5) { set(a); vTaskDelay(pdMS_TO_TICKS(40)); }
    for (int a = -limit; a <= 0; a += 5)  { set(a); vTaskDelay(pdMS_TO_TICKS(40)); }
}

static void help(void)
{
    printf("[CONSOLE] commands: tail <deg> | rudder <deg> | center | sweep tail|rudder | status | help\r\n");
}

static void run(char *line)
{
    char word[12] = {0};
    int value = 0;
    int n = sscanf(line, "%11s %d", word, &value);
    if (n <= 0) { return; }

    if (strcmp(word, "tail") == 0 && n == 2)            { set_tail(value);   printf("[CONSOLE] OK tail %d\r\n", tail_deg); }
    else if (strcmp(word, "rudder") == 0 && n == 2)     { set_rudder(value); printf("[CONSOLE] OK rudder %d\r\n", rudder_deg); }
    else if (strcmp(word, "center") == 0)               { set_tail(0); set_rudder(0); printf("[CONSOLE] OK centered\r\n"); }
    else if (strcmp(word, "sweep") == 0 && strstr(line, "tail"))   { sweep(set_tail, 90);   printf("[CONSOLE] OK tail swept\r\n"); }
    else if (strcmp(word, "sweep") == 0 && strstr(line, "rudder")) { sweep(set_rudder, 35); printf("[CONSOLE] OK rudder swept\r\n"); }
    else if (strcmp(word, "status") == 0)               { printf("[CONSOLE] tail %d deg, rudder %d deg\r\n", tail_deg, rudder_deg); }
    else if (strcmp(word, "help") == 0)                 { help(); }
    else                                                { printf("[CONSOLE] unknown: \"%s\" (type help)\r\n", line); }
}

void console_handler(void *argument)
{
    char line[CONSOLE_LINE_MAX];
    int len = 0;
    help();
    for (;;)
    {
        char c;
        if (xQueueReceive(rx_queue, &c, portMAX_DELAY) != pdTRUE) { continue; }
        if (c == '\r' || c == '\n')
        {
            if (len > 0) { line[len] = '\0'; run(line); len = 0; }
        }
        else if ((c == '\b' || c == 127) && len > 0) { len--; }
        else if (len < CONSOLE_LINE_MAX - 1 && c >= ' ') { line[len++] = c; }
    }
}
