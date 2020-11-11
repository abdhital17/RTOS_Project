// RTOS Framework - Fall 2020
// J Losh

// Student Name:
// Abhishek Dhital

// Add xx_ prefix to all files in your project
// xx_rtos.c
// xx_tm4c123gh6pm_startup_ccs.c
// xx_other files
// (xx is a unique number that will be issued in class)
// Please do not change any function name in this code or the thread priorities

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// 6 Pushbuttons and 5 LEDs, UART
// LEDS on these pins:
// Blue:   PF2 (on-board)
// Red:    PE1
// Orange: PE2
// Yellow: PE3
// Green:  PE4
// PBs on these pins
// PB0:    PA2
// PB1:    PA3
// PB2:    PA4
// PB3:    PA5
// PB4:    PA6
// PB5:    PA7
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "uart0.h"
#include "wait.h"

// REQUIRED: correct these bitbanding references for the off-board LEDs
#define RED_LED       (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 1*4)))
#define GREEN_LED     (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 4*4)))
#define BLUE_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))
#define YELLOW_LED    (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 3*4)))
#define ORANGE_LED    (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 2*4)))

#define PUSH_BUTTON0  (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 2*4)))
#define PUSH_BUTTON1  (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 3*4)))
#define PUSH_BUTTON2  (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 4*4)))
#define PUSH_BUTTON3  (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 5*4)))
#define PUSH_BUTTON4  (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 6*4)))
#define PUSH_BUTTON5  (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 7*4)))

// PortE masks for LED's
#define GREEN_LED_MASK 16
#define RED_LED_MASK 2
#define ORANGE_LED_MASK 4
#define YELLOW_LED_MASK 8

//PortF mask for blue LED
#define BLUE_LED_MASK 4

//PortA masks for pushbuttons
#define PUSH_BUTTON_MASK_0 4
#define PUSH_BUTTON_MASK_1 8
#define PUSH_BUTTON_MASK_2 16
#define PUSH_BUTTON_MASK_3 32
#define PUSH_BUTTON_MASK_4 64
#define PUSH_BUTTON_MASK_5 128


//SVC Number MACROS
#define YIELD 10
#define SLEEP 20
#define WAIT  30
#define POST  40

//allocation of 28KiB of heap starting from address 0x20001000 in SRAM
#pragma DATA_SECTION(stack, ".heap")
static uint32_t stack [7168];

uint32_t heap_pointer = 0x20001000;

//assembly functions declaration
extern uint32_t getPSPaddress();
extern void setASPbit();
extern void setPSPaddress(uint32_t address);
extern uint8_t getSVCNumber();




//-----------------------------------------------------------------------------
// RTOS Defines and Kernel Variables
//-----------------------------------------------------------------------------

// function pointer
typedef void (*_fn)();

// semaphore
#define MAX_SEMAPHORES 5
#define MAX_QUEUE_SIZE 5
typedef struct _semaphore
{
    uint16_t count;
    uint16_t queueSize;
    uint32_t processQueue[MAX_QUEUE_SIZE]; // store task index here
} semaphore;

semaphore semaphores[MAX_SEMAPHORES];

uint8_t semaphoreCount = 0;

uint8_t keyPressed, keyReleased, flashReq, resource;

// task
#define STATE_INVALID    0 // no task
#define STATE_UNRUN      1 // task has never been run
#define STATE_READY      2 // has run, can resume at any time
#define STATE_DELAYED    3 // has run, but now awaiting timer
#define STATE_BLOCKED    4 // has run, but now blocked by semaphore

#define MAX_TASKS 12       // maximum number of valid tasks
uint8_t taskCurrent = 0;   // index of last dispatched task
uint8_t taskCount = 0;     // total number of valid tasks

// REQUIRED: add store and management for the memory used by the thread stacks
//           thread stacks must start on 1 kiB boundaries so mpu can work correctly

struct _tcb
{
    uint8_t state;                 // see STATE_ values above
    void *pid;                     // used to uniquely identify thread
    void *spInit;                  // location of original stack pointer
    void *sp;                      // location of stack pointer for thread
    int8_t priority;               // 0=highest to 15=lowest
    int8_t currentPriority;        // used for priority inheritance
    uint32_t ticks;                // ticks until sleep complete
    char name[16];                 // name of task used in ps command
    void *semaphore;               // pointer to the semaphore that is blocking the thread
} tcb[MAX_TASKS];

//-----------------------------------------------------------------------------
// RTOS Kernel Functions
//-----------------------------------------------------------------------------

// REQUIRED: initialize systick for 1ms system timer
void initRtos()
{
    uint8_t i;
    // no tasks running
    taskCount = 0;
    // clear out tcb records
    for (i = 0; i < MAX_TASKS; i++)
    {
        tcb[i].state = STATE_INVALID;
        tcb[i].pid = 0;
    }
}

// REQUIRED: Implement prioritization to 16 levels
int rtosScheduler()
{
    bool ok;
    static uint8_t task = 0xFF;
    ok = false;
    while (!ok)
    {
        task++;
        if (task >= MAX_TASKS)
            task = 0;
        ok = (tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN);
    }
    return task;
}

void PrintIntToHex(uint32_t value)
{
    char hexStr[9] = {0};

    char HexAlphabets [] = "ABCDEF";
    int i =0;
    while (value != 0)
    {
       char temp;

       if ((value % 16) <= 15 && (value % 16) >= 10)
            temp = HexAlphabets [(value % 16) % 10];

       else
           temp = (value % 16) + 48;

       hexStr[7-i] = temp;

       value = value / 16;
       i++;
    }

    putsUart0("0x");
    i=0;
    while(i <= 7)
    {
        if (hexStr[i] != 0)
            putcUart0(hexStr[i]);
        else
            putcUart0('0');
        i++;
    }


}

bool createThread(_fn fn, const char name[], uint8_t priority, uint32_t stackBytes)
{
    bool ok = false;
    uint8_t i = 0;
    bool found = false;
    // REQUIRED: store the thread name
    // add task if room in task list
    // allocate stack space for a thread and assign to sp below
    if (taskCount < MAX_TASKS)
    {
        // make sure fn not already in list (prevent reentrancy)
        while (!found && (i < MAX_TASKS))
        {
            found = (tcb[i++].pid ==  fn);
        }
        if (!found)
        {
            // find first available tcb record
            i = 0;
            while (tcb[i].state != STATE_INVALID) {i++;}

            //storing the thread name
            uint8_t j = 0;
            for (j=0; j<16; j++)
            {
                if (name[j] == 0)
                        break;
                tcb[i].name[j] = name [j];
            }

            tcb[i].state = STATE_UNRUN;
            tcb[i].pid = fn;

            //moving the heap pointer to the stack space of this thread
            tcb[i].sp = heap_pointer;
            heap_pointer += stackBytes;
            tcb[i].spInit = heap_pointer;

            tcb[i].priority = priority;
            tcb[i].currentPriority = priority;
            // increment task count
            taskCount++;
            ok = true;

            //CODE & DISCARD
            putsUart0("Name: ");
            putsUart0(tcb[i].name);
            putsUart0("\n\r");

            putsUart0("Address of fn: ");
            PrintIntToHex((uint32_t) fn);
            putsUart0("\n\r");

            putsUart0("Priority of the task: ");
            PrintIntToHex(tcb[i].priority);
            putsUart0("\n\r");

            putsUart0("Stackinit address: ");
            PrintIntToHex(tcb[i].spInit);
            putsUart0("\n\r");

            putsUart0("Stack address: ");
            PrintIntToHex(tcb[i].sp);
            putsUart0("\n\r");
            putsUart0("\n\r");

        }
    }
    // REQUIRED: allow tasks switches again
    return ok;
}

// REQUIRED: modify this function to restart a thread
void restartThread(_fn fn)
{
}

// REQUIRED: modify this function to destroy a thread
// REQUIRED: remove any pending semaphore waiting
// NOTE: see notes in class for strategies on whether stack is freed or not
void destroyThread(_fn fn)
{
}

// REQUIRED: modify this function to set a thread priority
void setThreadPriority(_fn fn, uint8_t priority)
{
}

int8_t createSemaphore(uint8_t count)
{
    int8_t index = -1;
    if (semaphoreCount < MAX_SEMAPHORES)
    {
        semaphores[semaphoreCount].count = count;
        index = semaphoreCount;
        semaphoreCount++;
    }
    return index;
}


// REQUIRED: modify this function to start the operating system, using all created tasks
void startRtos()
{
    taskCurrent = rtosScheduler();
    uint32_t sp = tcb[taskCurrent].spInit;
    setPSPaddress(sp);

    tcb[taskCurrent].state = STATE_READY;
    setASPbit();

    _fn fn = tcb[taskCurrent].pid;
    fn();
}


// REQUIRED: modify this function to yield execution back to scheduler using pendsv
// push registers, call scheduler, pop registers, return to new function
void yield()
{
   __asm(" SVC #10");

}

// REQUIRED: modify this function to support 1ms system timer
// execution yielded back to scheduler until time elapses using pendsv
// push registers, set state to delayed, store timeout, call scheduler, pop registers,
// return to new function (separate unrun or ready processing)
void sleep(uint32_t tick)
{
    __asm(" SVC #20");
}


// REQUIRED: modify this function to wait a semaphore with priority inheritance
// return if avail (separate unrun or ready processing), else yield to scheduler using pendsv
void wait(int8_t semaphore)
{
    __asm(" SVC #30");
}

// REQUIRED: modify this function to signal a semaphore is available using pendsv
void post(int8_t semaphore)
{
    __asm(" SVC #40");
}

// REQUIRED: modify this function to add support for the system timer
// REQUIRED: in preemptive code, add code to request task switch
void systickIsr()
{
    uint8_t count = 0;
    for (count = 0; count < MAX_TASKS; count++)
    {
        if (tcb[count].state == STATE_DELAYED)
        {
            (tcb[count].ticks)--;

            if (tcb[count].ticks == 0)
                tcb[count].state = STATE_READY;
        }
    }
}

//function to push registers R4-11 to PSP in Isr when ASP = 1
void PushRegstoPSP()
{
    __asm("         MRS R0, PSP");
    __asm("         STR R4,  [R0, #-4]!");
    __asm("         STR R5,  [R0, #-4]!");
    __asm("         STR R6,  [R0, #-4]!");
    __asm("         STR R7,  [R0, #-4]!");
    __asm("         STR R8,  [R0, #-4]!");
    __asm("         STR R9,  [R0, #-4]!");
    __asm("         STR R10, [R0, #-4]!");
    __asm("         STR R11, [R0, #-4]!");
}


//function to pop registers R4-11 from PSP in ISR when ASP = 1
void PopRegsFromPSP()
{
    __asm("          MRS R0, PSP");
    __asm("          LDR R4,  [R0, #-4]!");
    __asm("          LDR R5,  [R0, #-4]!");
    __asm("          LDR R6,  [R0, #-4]!");
    __asm("          LDR R7,  [R0, #-4]!");
    __asm("          LDR R8,  [R0, #-4]!");
    __asm("          LDR R9,  [R0, #-4]!");
    __asm("          LDR R10, [R0, #-4]!");
    __asm("          LDR R11, [R0, #-4]!");
}
// REQUIRED: in coop and preemptive, modify this function to add support for task switching
// REQUIRED: process UNRUN and READY tasks differently
void pendSvIsr()
{
    BLUE_LED = 1;           //code & discard
    PushRegstoPSP();
    tcb[taskCurrent].sp = getPSPaddress();

    taskCurrent=rtosScheduler();

    if(tcb[taskCurrent].state == STATE_READY)
    {
        setPSPaddress(tcb[taskCurrent].sp);
        PopRegsFromPSP();
    }

    else
    {
        tcb[taskCurrent].state = STATE_READY;
        void* pid = tcb[taskCurrent].pid;

        uint32_t* psp_add = tcb[taskCurrent].spInit;

        psp_add -= 1;
        *(psp_add ) = 0x61000000;

        psp_add -= 1;
        *(psp_add ) = pid;

        psp_add -= 1;
        *(psp_add ) = 0xFFFFFFFD;

        psp_add -= 1;
        *(psp_add ) = 96;

        psp_add -= 1;
        *(psp_add ) = 80;

        psp_add -= 1;
        *(psp_add ) = 64;

        psp_add -= 1;
        *(psp_add ) = 48;

        psp_add -= 1;
        *(psp_add) = 32;

        setPSPaddress(psp_add);
     }

    BLUE_LED = 0;

}

// REQUIRED: modify this function to add support for the service call
// REQUIRED: in preemptive code, add code to handle synchronization primitives
void svCallIsr()
{
    uint32_t r0, r1, r2, r3, r12;
    uint8_t N;
    //getting the PSP stack of the process
    uint32_t* psp = getPSPaddress();
    r0 = *(psp);
    r1 = *(psp + 1);
    r2 = *(psp + 2);
    r3 = *(psp + 3);
    r12 = *(psp + 4);

    N = getSVCNumber();

   switch(N)
   {
       //yield
       case 10:
               NVIC_INT_CTRL_R   |= 0x10000000;  //setting the pendSV active bit at bit #28  of the NVIC_INT_CTRL register to enable the PendSV ISR call
               break;

       //sleep
       case 20:
               tcb[taskCurrent].ticks = r0;
               tcb[taskCurrent].state = STATE_DELAYED;
               NVIC_INT_CTRL_R   |= 0x10000000;  //setting the pendSV active bit at bit #28  of the NVIC_INT_CTRL register to enable the PendSV ISR call

               NVIC_ST_RELOAD_R |= 39999;
               NVIC_ST_CTRL_R |= 0x7;

               break;

       //wait
       case 30:
               if (semaphores[r0].count > 0)
                   (semaphores[r0].count--);

               else
               {
                   tcb[taskCurrent].state = STATE_BLOCKED;
                   tcb[taskCurrent].semaphore = &semaphores[r0];
                   semaphores[r0].processQueue[semaphores[r0].queueSize] = taskCurrent;
                   (semaphores[r0].queueSize)++;
                   NVIC_INT_CTRL_R   |= 0x10000000;  //setting the pendSV active bit at bit #28  of the NVIC_INT_CTRL register to enable the PendSV ISR call
               }
               break;

       //post
       case 40:
               (semaphores[r0].count)++;
               if(semaphores[r0].queueSize > 0)
               {
                   tcb[semaphores[r0].processQueue[0]].state = STATE_READY;
                   (semaphores[r0].count--);


                   uint8_t pQ = 0;
                   while(pQ < (semaphores[r0].queueSize - 1))
                   {
                       semaphores[r0].processQueue[pQ] = semaphores[r0].processQueue[pQ + 1];
                       pQ++;
                   }

                   (semaphores[r0].queueSize--);

               }
               break;


   }

}

// REQUIRED: code this function
void mpuFaultIsr()
{
}

// REQUIRED: code this function
void hardFaultIsr()
{
}

// REQUIRED: code this function
void busFaultIsr()
{
}

// REQUIRED: code this function
void usageFaultIsr()
{
}

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
// REQUIRED: Add initialization for blue, orange, red, green, and yellow LEDs
//           6 pushbuttons, code readpushbutton function
void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, sysdivider of 5, creating system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable clocks
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4 | SYSCTL_RCGCGPIO_R0 | SYSCTL_RCGCGPIO_R5;
    _delay_cycles(3);

    // Configure LED and pushbutton pins
    GPIO_PORTE_DIR_R |= GREEN_LED_MASK | RED_LED_MASK | YELLOW_LED_MASK | ORANGE_LED_MASK;   // bits 1 and 3 are outputs, other pins are inputs
    GPIO_PORTE_DR2R_R |= GREEN_LED_MASK | RED_LED_MASK | YELLOW_LED_MASK | ORANGE_LED_MASK;  // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTE_DEN_R |= GREEN_LED_MASK | RED_LED_MASK | YELLOW_LED_MASK | ORANGE_LED_MASK;

    GPIO_PORTF_DIR_R |= BLUE_LED_MASK;
    GPIO_PORTF_DR2R_R |= BLUE_LED_MASK;
    GPIO_PORTF_DEN_R |=  BLUE_LED_MASK;

    GPIO_PORTA_DIR_R  &= ~(PUSH_BUTTON_MASK_0 | PUSH_BUTTON_MASK_1 | PUSH_BUTTON_MASK_2 | PUSH_BUTTON_MASK_3 | PUSH_BUTTON_MASK_4 |PUSH_BUTTON_MASK_5);
    //GPIO_PORTA_DR2R_R |= PUSH_BUTTON_MASK_0 | PUSH_BUTTON_MASK_1 | PUSH_BUTTON_MASK_2 | PUSH_BUTTON_MASK_3 | PUSH_BUTTON_MASK_4 |PUSH_BUTTON_MASK_5;
    GPIO_PORTA_DEN_R |= PUSH_BUTTON_MASK_0 | PUSH_BUTTON_MASK_1 | PUSH_BUTTON_MASK_2 | PUSH_BUTTON_MASK_3 | PUSH_BUTTON_MASK_4 |PUSH_BUTTON_MASK_5;

    GPIO_PORTA_PUR_R |= PUSH_BUTTON_MASK_0 | PUSH_BUTTON_MASK_1 | PUSH_BUTTON_MASK_2 | PUSH_BUTTON_MASK_3 | PUSH_BUTTON_MASK_4 |PUSH_BUTTON_MASK_5;
    // enable internal pull-up for push button
}

// REQUIRED: add code to return a value from 0-63 indicating which of 6 PBs are pressed
uint8_t readPbs()
{
    uint8_t readValue = 0;
    if(~PUSH_BUTTON0)
        readValue += 1;

    if(~PUSH_BUTTON1)
        readValue += 2;

    if(~PUSH_BUTTON2)
        readValue += 4;

    if(~PUSH_BUTTON3)
        readValue += 8;

    if(~PUSH_BUTTON4)
        readValue += 16;

    if(~PUSH_BUTTON5)
        readValue += 32;

    return readValue;
}

//-----------------------------------------------------------------------------
// YOUR UNIQUE CODE
// REQUIRED: add any custom code in this space
//-----------------------------------------------------------------------------

// ------------------------------------------------------------------------------
//  Task functions
// ------------------------------------------------------------------------------

//code & discard
uint32_t getSP()
{
    __asm("         MOV R0, SP");
    __asm("         BX LR");
}

void idle2()
{
    while(true)
    {
        RED_LED = 1;
        waitMicrosecond(1000);
        RED_LED = 0;
        //code & discard
        __asm("       MOV   R0, #0");
        __asm("       MOV   R1, #1");
        __asm("       MOV   R2, #2");
        __asm("       MOV   R3, #3");
        __asm("       MOV   R4, #40");
        __asm("       MOV   R5, #50");
        __asm("       MOV   R6, #60");
        __asm("       MOV   R7, #70");
        __asm("       MOV   R8, #80");
        __asm("       MOV   R9, #90");
        __asm("       MOV   R10, #100");
        __asm("       MOV   R11, #111");
        __asm("       MOV   R12, #122");
        yield();
    }
}

// one task must be ready at all times or the scheduler will fail
// the idle task is implemented for this purpose
void idle()
{
    while(true)
    {
        ORANGE_LED = 1;
        waitMicrosecond(1000);
        ORANGE_LED = 0;

        uint32_t stack_pointer = getSP();
        //code & discard
        __asm("       MOV   R0, #0");
        __asm("       MOV   R1, #1");
        __asm("       MOV   R2, #2");
        __asm("       MOV   R3, #3");
        __asm("       MOV   R4, #4");
        __asm("       MOV   R5, #5");
        __asm("       MOV   R6, #6");
        __asm("       MOV   R7, #7");
        __asm("       MOV   R8, #8");
        __asm("       MOV   R9, #9");
        __asm("       MOV   R10, #10");
        __asm("       MOV   R11, #11");
        __asm("       MOV   R12, #12");

        putsUart0("SP: ");
        PrintIntToHex(stack_pointer);
        putsUart0("\n\r");



        yield();
    }
}

void flash4Hz()
{
    while(true)
    {
        GREEN_LED ^= 1;
        sleep(125);
    }
}

void oneshot()
{
    while(true)
    {
        wait(flashReq);
        YELLOW_LED = 1;
        sleep(1000);
        YELLOW_LED = 0;
    }
}

void partOfLengthyFn()
{
    // represent some lengthy operation
    waitMicrosecond(990);
    // give another process a chance to run
    yield();
}

void lengthyFn()
{
    uint16_t i;
    while(true)
    {
        wait(resource);
        for (i = 0; i < 5000; i++)
        {
            partOfLengthyFn();
        }
        RED_LED ^= 1;
        post(resource);
    }
}

void readKeys()
{
    uint8_t buttons;
    while(true)
    {
        wait(keyReleased);
        buttons = 0;
        while (buttons == 0)
        {
            buttons = readPbs();
            yield();
        }
        post(keyPressed);
        if ((buttons & 1) != 0)
        {
            YELLOW_LED ^= 1;
            RED_LED = 1;
        }
        if ((buttons & 2) != 0)
        {
            post(flashReq);
            RED_LED = 0;
        }
        if ((buttons & 4) != 0)
        {
            restartThread(flash4Hz);
        }
        if ((buttons & 8) != 0)
        {
            destroyThread(flash4Hz);
        }
        if ((buttons & 16) != 0)
        {
            setThreadPriority(lengthyFn, 4);
        }
        yield();
    }
}

void debounce()
{
    uint8_t count;
    while(true)
    {
        wait(keyPressed);
        count = 10;
        while (count != 0)
        {
            sleep(10);
            if (readPbs() == 0)
                count--;
            else
                count = 10;
        }
        post(keyReleased);
    }
}

void uncooperative()
{
    while(true)
    {
        while (readPbs() == 8)
        {
        }
        yield();
    }
}

void errant()
{
    uint32_t* p = (uint32_t*)0x20000000;
    while(true)
    {
        while (readPbs() == 32)
        {
            *p = 0;
        }
        yield();
    }
}

void important()
{
    while(true)
    {
        wait(resource);
        BLUE_LED = 1;
        sleep(1000);
        BLUE_LED = 0;
        post(resource);
    }
}

// REQUIRED: add processing for the shell commands through the UART here
void shell()
{
    while (true)
    {
    }
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    bool ok;

    // Initialize hardware
    initHw();
    initUart0();

    initRtos();

    // Setup UART0 baud rate
    setUart0BaudRate(115200, 40e6);
    // Power-up flash
    GREEN_LED = 1;
    waitMicrosecond(250000);
    GREEN_LED = 0;
    waitMicrosecond(250000);

    // Initialize semaphores
    keyPressed = createSemaphore(1);
    keyReleased = createSemaphore(0);
    flashReq = createSemaphore(5);
    resource = createSemaphore(1);

    // Add required idle process at lowest priority
      ok =  createThread(idle, "Idle", 15, 1024);
  //  ok &= createThread(idle2, "Idle2", 15, 1024);  //cod & discard

//    // Add other processes
//    ok &= createThread(lengthyFn, "LengthyFn", 12, 1024);
//      ok & = createThread(flash4Hz, "Flash4Hz", 8, 1024);
      ok &= createThread(oneshot, "OneShot", 4, 1024);
//    ok &= createThread(readKeys, "ReadKeys", 12, 1024);
//    ok &= createThread(debounce, "Debounce", 12, 1024);
//    ok &= createThread(important, "Important", 0, 1024);
//    ok &= createThread(uncooperative, "Uncoop", 10, 1024);
//    ok &= createThread(errant, "Errant", 8, 1024);
//    ok &= createThread(shell, "Shell", 8, 1024);

    // Start up RTOS
    if (ok)
        startRtos(); // never returns
    else
        RED_LED = 1;

    return 0;
}
