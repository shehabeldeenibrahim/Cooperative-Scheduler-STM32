void taskA()
{
    printUART("A");
    ReRunMe(taskA, 5, PRIORITY_A);
}
void taskB()
{
    printUART("B");
}
void taskC()
{
    printUART("C");
    ReRunMe(taskC, 5, PRIORITY_C);
}
void taskD()
{
    printUART("D");
}

void main(void)
{
    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        Init();
        QueTask(taskA, PRIORITY_A, &PQ);
        QueTask(taskB, PRIORITY_B, &PQ);
        QueTask(taskC, PRIORITY_C, &PQ);
        QueTask(taskD, PRIORITY_D, &PQ);
        //    QueDelay(taskA, 2, 2, &DQ);
        //    QueDelay(taskB, 2, 3, &DQ);
        //    QueDelay(taskC, 4, 4, &DQ);
        //    QueDelay(taskD, 3, 3, &DQ);

        while (1)
        {
            Dispatch(&PQ);
        }
    }
}
