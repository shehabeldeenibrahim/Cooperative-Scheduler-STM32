#pragma anon_unions
typedef struct delayQueue delayQueue;
typedef struct itemDQ itemDQ;
typedef void (*task)(void);

// Structure for the elements in the
// delay queue
struct itemDQ
{
	task task;
	int priority;
	int delay;
};
void setValueDQ(task task, itemDQ *i)
{
	i->task = task;
}
task getValueDQ(itemDQ i)
{
	return i.task;
}
void setDelay(int delay, itemDQ *i)
{
	i->delay = delay;
}
int getDelay(itemDQ i)
{
	return i.delay;
}
void setPriorityDQ(int priority, itemDQ *i)
{
	i->priority = priority;
}
struct delayQueue
{

	// Store the element of a delay queue
	struct itemDQ pr[10];

	// Pointer to the last index
	int size;
};
void setSizeDQ(int value, delayQueue *PQ)
{
	PQ->size = value;
}
int getSizeDQ(delayQueue PQ)
{
	return PQ.size;
}
void initializeDQ(delayQueue *PQ)
{
	PQ->size = -1;
}
// Function to insert a new element
// into delay queue
void QueDelay(task value, int delay, int priority, delayQueue *PQ)
{

	// Increase the size
	//PQ.size++;
	setSizeDQ(getSizeDQ(*PQ) + 1, PQ);
	//int s = getSizeDQ(*PQ);
	//HAL_UART_Transmit(&huart2, (uint8_t *)s, sizeof(s), HAL_MAX_DELAY);

	// Insert the element
	setValueDQ(value, &(PQ->pr[PQ->size]));
	setDelay(delay, &(PQ->pr[PQ->size]));
	setPriorityDQ(priority, &(PQ->pr[PQ->size]));
}

// Function to check the top element
int peekDelay(delayQueue PQ)
{
	int lowestDelay = 21474836;
	int ind = -1;
	// Check for the element with
	// highest delay
	for (int i = getSizeDQ(PQ); i >= 0; i--)
	{
		// If delay is same choose
		// the element with the
		// highest value
		if (lowestDelay >= getDelay(PQ.pr[i]))
		{
			lowestDelay = getDelay(PQ.pr[i]);
			ind = i;
		}
	}
	// Return position of the element
	return ind;
}
// Function to remove the element with
// in the given index
void dequeueDelay(delayQueue *PQ, int ind)
{

	// Shift the element one index before
	// from the postion of the element
	// with highest priortity is found
	for (int i = ind; i < PQ->size; i++)
	{
		PQ->pr[i] = PQ->pr[i + 1];
	}

	// Decrease the size of the
	// delay queue by one
	PQ->size--;
}

void decrement(delayQueue *DQ, priorityQueue *PQ)
{
	volatile unsigned int delay;
	for (volatile int i = 0; i <= getSizeDQ(*DQ); i++)
	{
		// loop on array
		// decrement delay
		delay = getDelay(DQ->pr[i]);
		delay-=1;
		// check if delay == 0 -> dequeue and enqueue in ready
		if (delay <= 0)
		{
			QueTask(DQ->pr[i].task, DQ->pr[i].priority, PQ);
			//DQ->pr[i].task();
			dequeueDelay(DQ, i);
			i = -1;
			
		}
		else
		{
			setDelay(delay, &DQ->pr[i]);
		}
	}
}
