#include "priorityQueue.h"

void taskA()
{
    HAL_UART_Transmit(&huart2, (uint8_t *)"task A", sizeof("task A"), 10); /* Print to UART */
}
void taskB()
{
    HAL_UART_Transmit(&huart2, (uint8_t *)"task B", sizeof("task B"), 10); /* Print to UART */
}
void taskC()
{
    HAL_UART_Transmit(&huart2, (uint8_t *)"task C", sizeof("task C"), 10); /* Print to UART */
}
void taskD()
{
    HAL_UART_Transmit(&huart2, (uint8_t *)"task D", sizeof("task D"), 10); /* Print to UART */
}

int main()
{
    priorityQueue PQ;
    initialize(&PQ);
    QueTask(taskA, 2, &PQ);
    QueTask(taskB, 2, &PQ);
    QueTask(taskC, 4, &PQ);
    QueTask(taskD, 3, &PQ);
    int i = peek(PQ);
    PQ.pr[i].task();

    dequeue(&PQ);
    i = peek(PQ);
    PQ.pr[i].task();
}
