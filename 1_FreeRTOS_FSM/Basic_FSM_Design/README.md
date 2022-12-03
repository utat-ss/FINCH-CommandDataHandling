# Description
Problem: Create the following FSM. Use 2 MCUs: 1 to cycle through the states, and one to provide it with the inputs via CAN.
![FSM](https://github.com/spacesys-finch/CommandDataHandling/blob/main/1_FreeRTOS_FSM/Basic_FSM_Design/Images/FSM.png?raw=true)

The current design requires 4 types of threads, an IRQ Handler, 2 Semaphores, 1 Message Queue:

1. Superloop Thread:
    
    The one superloop thread is instantiated and begins execution at the start. This thread must have the highest priority in the whole program. Within the loop, it checks whether there are any available messages in the message queue. If no, the thread is blocked and lower priority threads are allowed to run. If yes, the thread dequeues the messages, and decides which task should be started, by created a thread with the needed started function. After starting the task, or deciding the task is already running, the loop begins again, and checks for messages on the queue.
    
2. Passive Threads
    
    These threads can either be defined at the beginnning, and started within the superloop as needed, or they can be defined and started within the superloop. Currently, they are all defined at the start, and started within the superloop. These threads **can** be interrupted at anytime during their runtime by a higher priority thread. All passive threads must have a lower priority than all follow-through threads, and lower than the ThreadTerminator.
    
3. Follow-Through Threads
    
    These threads can either be defined at the beginnning, and started within the superloop as needed, or they can be defined and started within the superloop. Currently, they are all defined at the start, and started within the superloop. These threads **cannot** be interrupted at anytime during their runtime by a higher priority thread. If a higher priority thread is started, it will sleep on the semaphore, and allow the current thread to continue running. 
    
    The thread begins by checking if the Sempahore FollowThroughSem is available. If it is not, the thread is blocked until the sempahore is available. If it is, then the thread begins execution. At the end of execution, the thread increments the Sempahore ThreadExitSem. Because all Follow-Through threads of any priority are still blocked, incrementing this sempahore allows the thread to reach its final command and exit properly, then ThreadTerminator can begin running, as ThreadExitSem was incremented.
    
4. ThreadTerminator
    
    This thread is instantiated and begins execution at the start. This thread must have a lower priority than all Follow-Through threads, and higher than all passive threads. This ensures that all Follow-Through threads are allowed to finish execution completely before the semaphore required by other Follow-Through threads is incremented. 
    
    This thread begins execution by checking if ThreadExitSem is available. If it is not, the thread is blocked until the sempahore is available. If it is, then the thread decrements ThreadExitSem, then increments FollowThroughSem. These two instructions only execute once a previously running Follow-Through thread has finished execution. Incrementing FollowThroughSem allows the highest priority Follow-Through thread to begin execution.
    

IRQ Handlers:

1. CAN Handler
    
    CAN Messages come in and wake up interrupt handler. Interrupt handler places these messages in Message Queue
    
2. USART Handler
    
    Used for debugging. uin8_t comes in from keyboard and wakes up interrupt handler. Interrupt handler places these messages in Message Queue.
    

Sempahores:

1. ThreadExitSem
    
    Used to ensure that a Follow-Through finishes execution before a higher priority follow through thread begins.
    
2. FollowThroughSem
    
    Used to ensure that a Follow-Through thread is not interrupted through execution, no matter the priority.


![FSM](https://github.com/spacesys-finch/CommandDataHandling/blob/main/1_FreeRTOS_FSM/Basic_FSM_Design/Images/fsm.drawio.svg?raw=true)