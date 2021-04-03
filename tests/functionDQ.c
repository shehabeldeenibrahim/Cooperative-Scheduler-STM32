priorityQueue PQ;
delayQueue DQ;
int tick = 0;
void printUART(char *c)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)c, sizeof(c), 10); /* Print to UART */
}
void taskA()
{
    printUART("A");
}
void taskB()
{
    printUART("B");
}
void taskC()
{
    printUART("C");
}
void taskD()
{
    printUART("D");
}
void Init()
{
    initialize(&PQ);
    initializeDQ(&DQ);
}

int main()
{
    QueDelay(taskA, 2, &DQ);
    QueDelay(taskB, 2, &DQ);
    QueDelay(taskC, 4, &DQ);
    QueDelay(taskD, 3, &DQ);

    while (1)
    {
        //Dispatch(&PQ);
        decrement(&DQ);
        Delay(50);
    }
}