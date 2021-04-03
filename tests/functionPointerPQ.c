#include "priorityQueue.h"

int main()
{
    priorityQueue PQ;
    initialize(&PQ);
    QueTask(10, 2, &PQ);
    QueTask(14, 2, &PQ);
    QueTask(16, 4, &PQ);
    QueTask(12, 3, &PQ);
    //uint8_t i = peek(PQ);
    int i = peek(PQ);
    sprintf(an, "%i", i);
    HAL_UART_Transmit(&huart2, (uint8_t *)an, sizeof(an), 10); /* Print to UART */
}
