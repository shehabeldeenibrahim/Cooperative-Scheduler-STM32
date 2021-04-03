#pragma anon_unions
#include "main.h"
typedef struct priorityQueue priorityQueue;
typedef struct item item;
typedef void (*task)(void);
// Structure for the elements in the
// priority queue
struct item
{
	void (*task)(void);
	int priority;
};
void setValue(task task, item *i)
{
	i->task = task;
}
task getValue(item i)
{
	return i.task;
}
void setPriority(int priority, item *i)
{
	i->priority = priority;
}
int getPriority(item i)
{
	return i.priority;
}

struct priorityQueue
{

	// Store the element of a priority queue
	struct item pr[10];

	// Pointer to the last index
	int size;
};
void setSize(int value, priorityQueue *PQ)
{
	PQ->size = value;
}
int getSize(priorityQueue PQ)
{
	return PQ.size;
}
void initialize(priorityQueue *PQ)
{
	PQ->size = -1;
}
// Function to insert a new element
// into priority queue
void QueTask(task value, int priority, priorityQueue *PQ)
{
	extern UART_HandleTypeDef huart2;
	// Increase the size
	//PQ.size++;
	setSize(getSize(*PQ) + 1, PQ);
	//int s = getSize(*PQ);
	//HAL_UART_Transmit(&huart2, (uint8_t *)s, sizeof(s), HAL_MAX_DELAY);

	// Insert the element
	setValue(value, &(PQ->pr[PQ->size]));
	setPriority(priority, &(PQ->pr[PQ->size]));
}

// Function to check the top element
int peek(priorityQueue PQ)
{
	int highestPriority = -2147483648;
	int ind = -1;
	// Check for the element with
	// highest priority
	for (int i = 0; i <= getSize(PQ); i++)
	{
		// If priority is same choose
		// the element with the
		// highest value
		if (highestPriority == getPriority(PQ.pr[i]) && ind > -1 && getValue(PQ.pr[ind]) > getValue(PQ.pr[i]))
		{
			highestPriority = getPriority(PQ.pr[i]);
			ind = i;
		}
		else if (highestPriority < getPriority(PQ.pr[i]))
		{
			highestPriority = getPriority(PQ.pr[i]);
			ind = i;
		}
	}
	// Return position of the element
	return ind;
}

// Function to remove the element with
// the highest priority
void dequeue(priorityQueue *PQ)
{
	// Find the position of the element
	// with highest priority
	int ind = peek(*PQ);

	// Shift the element one index before
	// from the postion of the element
	// with highest priortity is found
	for (int i = ind; i < PQ->size; i++)
	{
		PQ->pr[i] = PQ->pr[i + 1];
	}

	// Decrease the size of the
	// priority queue by one
	PQ->size--;
}

void Dispatch(priorityQueue *PQ)
{
	if (PQ->size >= 0)
	{
		int i = peek(*PQ);
		PQ->pr[i].task();
		dequeue(PQ);
	}
}
