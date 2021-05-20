# Function-Queue-Cooperative-Scheduler-ARM STM32
The objective of this project is to develope a task scheduler that works on a Real-Time Operating System, specifically the ARM Cortex STM32. The scheduler relies on priority queues that store the tasks provided by the user, and sort them based on their priorities. It also relies on a set of APIs to facilitate the interaction between the user's application and the scheduler.

### Project directory
    .
    ├── tests                   # Test cases for the priority and delay queues
    ├── parking                 # Folder containing parking sensor application
    ├── temperature             # Folder containing temperature sensor application
    ├── main.c                  # Parking application + test cases
    ├── stm32l4xx_it            # Interrupts file
    ├── priorityQueue.h         # Implementation of the priority queue data-structure
    ├── delayQueue.h            # Implementation of the delay queue data-structure

# Implementation Details
The scheduler uses 2 priority queues, namely `readyQueue` and `delayQueue`. The `readyQueue` is used to store the ready to run tasks, and the `delayQueue` is used to store the tasks for a specific amount of time (specified by the user), and then it's transferred to the ready queue. 
### Priority Queue Implementation Description
The Priority Queue struct contains the following members:
- A struct named *item* that has a member of type function pointer and an integer value representing the priority
- An array of structs items that stores the tasks
- An integer size holding the number of elements in the queue
- Methods:
  - `QueTask` inserts the task into the queue and sets its members; *size* and *priority*, it can be called from ISR or from other tasks
  - `Peek`      returns the index of the highest priority task
  - `Dequeue` removes the task with the highest priority, and decrements the *size*
  - `Dispatch`retrieves the highest priority task, runs it and calls `dequeue` to remove it from the queue 
  - `Setters & Getters` a bunch of setters and getters to set and retrieve members in the struct

The Delay Queue struct contains the same members and methods as the `priorityQueue` except the following:
- Methods:
  - `PeekDelay` returns the index of the task with **lowest delay**
  - `Decrement` decrements delay value for each task in the delay queue, this is called every tick in the ISR (*explained below*)

### Queue APIs Implementation Description
- `RerunMe` for the task to enqueue itself in:
  - `readyQueue` if called with 0 as an argument
  - `delayQueue` if called with positive argument specifying the delay
- `Init` Initializes all needed data-structures
- `Dispatch` retrieves the highest priority task, runs it and calls `dequeue` to remove it from the queue

### Interupts Implementation Desciption
- `SysTick_Handler` increments the global variable *tick* every 50 msec
- `TIM2_IRQHandler` has channel 1 in `capture_mode` to detect *rise* and *falling* edges and calculates the distance of the **ultrasound senor** (*for parking sensor app*)

# Units Tests
The *tests* directory contains C files to that validates different components of the scheduler. *UART2* has been used to print the task's name in *Tera-Term* screen to make sure that each task runs based on its given priority.

The following code snippet shows how a test task is defined
```
void taskA()
{
    printUART("A"); /* print using uart2 */
    ReRunMe(taskA, 5, PRIORITY_A); /* rerun with 5 seconds delay with the given priority */
}
```
The following code snippet shows how a test is done
```
void main()
{
    Init(); /* Initialize queues */
    QueTask(taskA, PRIORITY_A, &PQ); /* Insert taskA */
    QueTask(taskB, PRIORITY_B, &PQ); /* Insert taskB */
    .
    .
    while (1)
    {
        Dispatch(&PQ); /* Run each task, with priority */
    }
}
```
![alt text](https://github.com/shehabeldeenibrahim/function-queue-scheduler-stm32/blob/master/tests/test.png)

# Scheduler Applications
To test the functionality of the scheduler on a real-life application, 2 applications have been developed and tested on the STM32 using CubeMX and Keil v5
### Parking Sensor
In this application, the end goal was to develope a parking sensor using the HC-SR04 ultra-sonic sensor, and an active buzzer. The program reads the input from the sensor, calculates the distance, and changes the frequency and the tone of the passive buzzer based on the distance from the obstacle; the beeps keep increasing in frequency and tone until it hits the obstacle and produces a constant beep, just like a regular car sensor. To demonstrate the usage of the `PriorityQueue`, the application was split into 3 tasks:
`ReadParkingSensor` which pulls the TRIG high then low, and starts the capture interrupt on *Channel 1*, then calls `RerunMe` with delay 1 tick
`StartSound` which Changes the frequency and tone of the buzzer based on the distance retrieved by the Motion sensor, then calls `RerunMe` with delay 2 tick
`ParkingSensorApp` is used to initialize the queues and enqueue each task, giving the buzzer a higher priority than the reading, and is called in the `main()` function

### Temperature Sensor
In this application, the end goal was to develope a utilize the temperature sensor in the DS3231 to measure the surrounding temperature, and indicate (using an LED) when the temperature exceeds a certain threshold, which is specified by the user using UART1 through Tera-Term
To demonstrate the usage of the `PriorityQueue`, the application was split into the following tasks:

`ReadTemperature` reads the temperature sensor every 30 seconds (written every 5 seconds for quicker testing) by interfacing with the sensor using SPI, checks if the input value exceeds the user set threshold to enqueue `ToggleLED` and prints to UART2, then calls `ReRunMe(30)` to enqueue itself again after 30 seconds

`ToggleLED` writes to a GPIO pin to switch on the LED

`ReadThreshold` starts the UART1 interrupt to receive threshold from the user enqueued in the `main`

`SetThreshold` sets the new threshold when the user hits the *enter* key in the terminal after converting the string of digits to int

Also there is the `UART interrupt handler` receives a character and appends to the buffer until the user hits *enter*, it enqueues `setThreshold` to set the temperature threshold
# Resources
- [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html)
- [KeilV5](https://www2.keil.com/mdk5)
- [Tera-Term](https://ttssh2.osdn.jp/index.html.en)
