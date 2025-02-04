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


//*******THIS IS A VISUAL REPRESENTATION OF HOW THE MPU IS CONFIGURED TO PROTECT THE 4GiB of MEMORY ADDRESSES IN THE CONTROLLER*****************************************
//                  The priority of the MPU regions is
//                  higher the region number,
//                  higher will be the priority of that region
//
//                  MPU Model implemented in this code showing all regions
//  *************************************************************************************
//  *                                                                                   *
//  * -1 default region covering all of addressable 4GiB of memory                      *
//  *  priv mode    rwx access                                                          *
//  *  Unpriv mode  no access                                                           *
//  *  ___________________________________________________________________________      *
//  * |  ********************                       region #0                     |     *
//  * |  * 256 KiB          *                       priv mode- r/w access         |     *
//  * |  * Flash Memory     *                       unpriv mode - r/w access      |     *
//  * |  * region #1        *                                                     |     *
//  * |  * priv mode- rwx   *                                                     |     *
//  * |  *unpriv mode - rwx *                                                     |     *
//  * |  ********************       _____________________________                 |     *
//  * |                             |      (total 32KiB SRAM)    |                |     *
//  * |                             |  divided into 4  subregions|                |     *
//  * |                             |         region #2-5        |                |     *
//  * |                             |  priv mode- rw access      |                |     *
//  * |                             |  unpriv mode - each thread |                |     *
//  * |                             |       gets rw access to    |                |     *
//  * |                             |       only its own stack   |                |     *
//  * |                             |____________________________|                |     *
//  * |                                                                           |     *
//  * |                                                                           |     *
//  * |                                                                           |     *
//  * |                                                                           |     *
//  * |                                                                           |     *
//  * |                        2GiB                                               |     *
//  * |                        Peripherals/Bitband                                |     *
//  * |                                                                           |     *
//  * |                                                                           |     *
//  * |                                                                           |     *
//  * |___________________________________________________________________________|     *
//  *                                                                                   *
//  *************************************************************************************

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


#define Line_Break "\n\r"


//allocation of 28KiB of heap starting from address 0x20001000 in SRAM
#pragma DATA_SECTION(stack, ".heap")
static uint32_t stack [7168];   //doing this will ensure that
                                //the OS variables are declared and stored in fixed region of SRAM
                                //starting from 0x2000.0800 to 0x2000.0FFF (2KiB);
                                //The MSP stack is from 0x2000.0000 to 0x2000.07FF (2KiB)
uint32_t heap_pointer = 0x20001000; //This is where the stack of first task should start;
                                    //In this case, we have stack of Idle starting from this address to upper 1KiB memory

//assembly functions declaration
extern void unprivilegedMode();
extern void privilegedMode();
extern void setPSPaddress(uint32_t address);
extern uint32_t getPSPaddress();
extern uint32_t getMSPaddress();
extern void setASPbit();


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
    char     name[16];  //store semaphore name here
} semaphore;

semaphore semaphores[MAX_SEMAPHORES];

uint8_t semaphoreCount = 0;


#define keyPressed  1
#define keyReleased 2
#define flashReq    3
#define resource    4

// task
#define STATE_INVALID    0 // no task
#define STATE_UNRUN      1 // task has never been run
#define STATE_READY      2 // has run, can resume at any time
#define STATE_DELAYED    3 // has run, but now awaiting timer
#define STATE_BLOCKED    4 // has run, but now blocked by semaphore
#define STATE_KILLED     5 //has been killed by using the shell command -> "kill PID"

#define MAX_TASKS 12       // maximum number of valid tasks
uint8_t taskCurrent = 0;   // index of last dispatched task
uint8_t taskCount = 0;     // total number of valid tasks


//booleans that tell the current scheduling mode, preemption ON/OFF, Priority Inheritance ON/OFF
bool priorityScheduling = true; //true if priority scheduling mode
                                 //false if round-robin scheduling mode
bool preemptON = true;         //true if preemption is on
                                //false if off
bool piON = false;              //true if priority inheritance on
                               //false if priority inheritance off

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
    uint8_t nameLength;             //length of name stored
    void *semaphore;               // pointer to the semaphore that is blocking the thread

    uint32_t MPUpermissions;        //permissions mask to determine which subregion of MPU to give access to the task
} tcb[MAX_TASKS];




//variables to store the CPU Time
//The Kernel calculates and stores the CPU time of tasks in the data structure below
//And copies those values to the shell's stack when needed (when PS command is entered in shell
uint32_t CPU_TIME [2][MAX_TASKS + 2] = {0}; //data structure to hold the data required to calculate the CPU time percentage
                                            //last two index hold the kernel time and total task time respectively

#define TOTAL_TIME_INDEX     13  //Index of the array where the total task time is stored
#define KERNEL_TIME_INDEX    12  //Index of the array where the kernel time is stored

uint8_t pingpong = 0;                       //variable used to switch between two
                                            //buffers in the pingpong buffer system

uint8_t  systickCount = 0;                  //increments with each sysTick ISR call. On every 100th call, this variable is cleared


//data structure in which the Shell gets the CPU time values from the Kernel/OS
struct PSData
{
    char name[16];              //stores the name of a task
    void *pid;                  //stores the pid of a task
    uint16_t CPU;                //stores the CPU time % of a particular task
    uint8_t prio;               //stores the priority of the task
    uint16_t kernelTime;         //stores the kernel time; stored only at index[0] for convenience
    uint8_t nameLength;          //stores the length of the name of task; required for print formatting purpose
    uint8_t valid;           //valid bit; shell will print this entry only if this is set
};





//-----------------------------------------------------------------------------
// RTOS Shell Variables/Macro/Structures/Functions
//-----------------------------------------------------------------------------
#define MAX_CHARS 80
#define MAX_FIELDS 5



typedef struct _USER_DATA
{
char buffer[MAX_CHARS+1];
uint8_t fieldCount;
uint8_t fieldPosition[MAX_FIELDS];
char fieldType[MAX_FIELDS];
} USER_DATA;


//function which stores the string entered in the UART by the user
void getsUart0(USER_DATA* d)
{
  uint8_t c=0; //counter variable
  char ch;
  while (1)  //loop starts
  {

    ch=getcUart0();
    if ((ch==8 || ch==127) && c>0) c--;

    else if (ch==13)
        {
         d->buffer[c]=0;
         return;
        }
    else if (ch>=32)
     {
        d->buffer[c]=ch;
        //putcUart0(ch);
        c++;
        if (c==MAX_CHARS)
        {
            d->buffer[c]='\0';
            return;
        }
     }
     else continue;
  }
}


//function which parses the given string and it is used in processing the commands
void parseFields(USER_DATA* d)
{
    uint8_t i=0;
    char prev=0;
    d->fieldCount=0;
    while(d->buffer[i]!='\0')
    {
        if((d->fieldCount)>=MAX_FIELDS)
        {
            break;
        }

        char temp=d->buffer[i];

        if(((temp>=97 && temp<=122) || (temp>=65&&temp<=90)) && prev!='a' )
        {
            prev='a';
            d->fieldType[(d->fieldCount)]='a';
            d->fieldPosition[(d->fieldCount)]=i;
            d->fieldCount+=1;
        }

        else if ((temp>=48 && temp<=57) && prev!='n')
           {
                prev='n';
                d->fieldType[d->fieldCount]='n';
                d->fieldPosition[d->fieldCount]=i;
                d->fieldCount+=1;
            }
        else if(!((temp>=97 && temp<=122) || (temp>=65&&temp<=90)) && !(temp>=48 && temp<=57) )
           {
             prev=0;
             d->buffer[i]='\0';
           }
        i++;
   }
}

//function that gets the string from the input command, given the fieldNumber
char* getFieldString(USER_DATA* data, uint8_t fieldNumber)
{
  if(fieldNumber<=data->fieldCount)
      {
        return &(data->buffer[data->fieldPosition[fieldNumber]]);
      }
  else
      return -1;
}


//function to convert a number of data type-char to 32 bit integer
int32_t alphabetToInteger(char* numStr)
{
    int32_t num=0;
    while (*numStr != 0)
      {
        if(*numStr >= 48 && *numStr <= 57)
        {
              num = num*10 + ((*numStr) - 48);
              numStr++;
        }

      }
    return num;
}


//returns true if given two strings are equal
//false if not equal
bool stringCompare(const char* str1,const char* str2)
{
   bool equal = true;
   while(*str1 != 0 || *str2 != 0)
   {
       if((*str1 == 0 && *str2 != 0) || (*str1 != 0 && *str2 ==0))
           return false;

       if(!(*str1 == *str2 || (*str1 + 32) == *str2 || *str1 == (*str2+32) || (*str1 - 32) == *str2 || *str1 == (*str2 - 32)))
       {
           equal = false;
           break;
       }

       str1++;
       str2++;
   }
   return equal;
}


//returns the integer in the entered command, given the fieldNumber
int32_t getFieldInteger(USER_DATA* data, uint8_t fieldNumber)
{
    if (fieldNumber<=data->fieldCount && data->fieldType[fieldNumber]=='n')
    {
        return alphabetToInteger(getFieldString(data, fieldNumber));
    }
    else
        return 0;
}


//prints the decimal equivalent of the given hexadecimal number (in string type)
uint32_t hexToInt(char* hex)
{
    uint32_t dec = 0;
    uint8_t count = 0;

    while(*hex != 0)
    {
        hex++;
        count++;
    }

    hex = hex - count;

    while (*hex != 0)
    {
        uint8_t character = *hex;
        uint32_t temp;

        if (character >= 48 && character <= 57)
        {
            temp = character - 48;
            uint8_t i = 0;
            for (i=0; i < count-1; i++)
            {
                temp = temp * 16;
            }

            count--;
        }

        else if ((character >= 65 && character <= 70 ) || (character >= 97 && character <= 102))
               {
                    if (character >= 65 && character <= 70 )
                        temp = character - 55;

                    else
                        temp = character - 87;

                    uint8_t i = 0;
                    for (i=0; i < count-1; i++)
                    {
                        temp = temp * 16;
                    }
                    count--;
               }
        dec = dec + temp;
        hex++;
    }
    return dec;
}


//function to check whether the entered command matches any of the kernel shell commands
//returns true if the entered command is valid
//false if invalid
bool isCommand(USER_DATA* data, const char strCommand[], uint8_t minArguments)
{
 if(stringCompare(strCommand,getFieldString(data,0)) && (data->fieldCount)>minArguments)
     return true;
 return false;
}


//handles the "ps" shell command
void ps(struct PSData* dataStruct)
{
    __asm("     svc #53");
    //putsUart0("Name\tPID\t%CPU\t\n\r");
}

//handles the "ipcs" shell command
void ipcs(void)
{
    putsUart0("Semaphore Name\tCount\tWaiting THreads\t\n\r");
    __asm("     svc #55");
}

//handles the "kill  #pid" shell command
void kill(uint32_t pid)
{
    __asm("  svc  #50");
}

//handles the "pi ON/OFF" shell command
void pi(uint8_t on)
{
  __asm("       SVC #58");
}

//handles the "preempt ON/OFF" shell command
void preempt(uint8_t on)
{
  __asm("       SVC #59");
}

//handles the "sched prio/rr" shell command
void sched(uint8_t prio_on)
{
    __asm("     SVC #57");
}

//handles the "pidof TASKNAME" shell command
void pidof(char name[])
{
    __asm("     SVC  #54");
}

//handles the "reboot" command
void reboot(void)
{
    __asm(" svc  #51");
}


//handles the "run PROCESS_NAME" shell command
void run(char* Process_Name)
{
    __asm(" svc   #52");
}


//-----------------------------------------------------------------------------
// RTOS Kernel Functions
//-----------------------------------------------------------------------------

//function to initialize and configure the Memory Protection Unit
void init_MPU()
{
//    //region #0 for the background region
    //RW access (no X access) for both privileged and unprivileged mode for RAM, bitband and peripherals
    NVIC_MPU_NUMBER_R |= 0x0;        //select region 0
    NVIC_MPU_BASE_R   |= 0x00000000; //addr=0x0000.0000 for complete 4GiB of memory, valid=0, region already set
    NVIC_MPU_ATTR_R   |= 0x1304003E; //S=0, C=1, B=0, size=31(11111 for 4GiB), XN=1(instruction fetch disabled), TEX=000
    NVIC_MPU_ATTR_R   |= 0x1;        //MPU  region enable
//
//
    //region #1 for the 256kiB flash memory
    //RWX access for both privileged and unprivileged mode
    NVIC_MPU_NUMBER_R |= 0x1;        //select region 1
    NVIC_MPU_BASE_R   |= 0x00000000; //addr=0x00000000 for base address of flash bits, 256KiB, valid =0, region already set in the NUMBER register
    NVIC_MPU_ATTR_R   |= 0x03020022; //S=0, C=1, B=0, size=17(10001) for 256KB, XN=0(instruction fetch enabled), TEX=000
    NVIC_MPU_ATTR_R   |= 0x1;        //MPU  region enable
//
//
//
//    //we divide the 32KiB SRAM into 4 different regions in the MPU
//    //each of those regions in the MPU will protect 8KiB of SRAM each
//    //the 8KiB within the regions are further divided into 8 subregions of 1KiB each
//
    //first region of SRAM- region 2
    NVIC_MPU_NUMBER_R   = 0x2;       //select region 2
    NVIC_MPU_BASE_R    |= 0x20000000;//addr=0x20000000,  valid =0, region already set in NUMBER register
    NVIC_MPU_ATTR_R    |= 0x11060018;//for internal SRAM S=1, C=1, B=0, size=12(1100) for 8KiB, XN=1(instruction fetch disabled), TEX=000
                                     //all 8 of the subregions enabled
    NVIC_MPU_ATTR_R    |= 0x1;

    //second region of SRAM- region 3
    NVIC_MPU_NUMBER_R   = 0x3;       //select region 3
    NVIC_MPU_BASE_R    |= 0x20002000;//addr=0x20002000,  valid =0, region already set in NUMBER register
    NVIC_MPU_ATTR_R    |= 0x11060018;//for internal SRAM S=1, C=1, B=0, size=12(1100) for 8KiB, XN=1(instruction fetch disabled), TEX=000
                                     //all 8 of the subregions enabled
    NVIC_MPU_ATTR_R    |= 0x1;

    //third region of SRAM- region 4
    NVIC_MPU_NUMBER_R   = 0x4;       //select region 4
    NVIC_MPU_BASE_R    |= 0x20004000;//addr=0x20004000,  valid =0, region already set in NUMBER register
    NVIC_MPU_ATTR_R    |= 0x11060018;//for internal SRAM S=1, C=1, B=0, size=12(1100) for 8KiB, XN=1(instruction fetch disabled), TEX=000
                                     //all 8 of the subregions enabled
    NVIC_MPU_ATTR_R    |= 0x1;

    //fourth region of SRAM- region 5
    NVIC_MPU_NUMBER_R   = 0x5;       //select region 5
    NVIC_MPU_BASE_R    |= 0x20006000;//addr=0x20006000,  valid =0, region already set in NUMBER register
    NVIC_MPU_ATTR_R    |= 0x11060018;//for internal SRAM S=1, C=1, B=0, size=12(1100) for 8KiB, XN=1(instruction fetch disabled), TEX=000
                                     //all but the last subregion enabled
    NVIC_MPU_ATTR_R    |= 0x1;

//
    NVIC_MPU_CTRL_R    |= 0x7;       //MPU enable, default region enable, MPU enabled during hard faults

    NVIC_SYS_HND_CTRL_R |= 0x70000;     //enable the MPU, BUS and USAGE fault interrupts


}

//sets all the tasks to STATE INVALID. First step to start the RTOS
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
    init_MPU();
}

// Implement prioritization to 16 levels
//Implements both Round Robin and Priority Scheduling Modes
int rtosScheduler()
{
    static uint8_t task = 0xFF;
    if (!priorityScheduling)        //round robin
    {
        bool ok;
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

    else        //priority scheduling
    {
        bool found = false;
        uint8_t checkPriority = tcb[taskCurrent].currentPriority;
      while(!found)
      {
          uint8_t i = taskCurrent + 1;
          while (i != taskCurrent)
        {
            if (tcb[i].state == STATE_READY || tcb[i].state == STATE_UNRUN)
            {
                if (tcb[i].currentPriority < checkPriority)
                {
                    checkPriority = tcb[i].currentPriority;
                    task = i;
                    found = true;
                }
            }

            i = (i + 1) % MAX_TASKS;
        }

           if (!found)
               checkPriority++;     //will never get >16 because Idle runs at priority 15 and is always ready
      }
      return task;
    }

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

    //putsUart0("0x");
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


void getIntString(uint32_t num)
{
    uint32_t temp = num;
    uint32_t length = 0;
    char getBack[3] ={'0','0',0};
    //getBack[2] = 0;

    while (temp != 0)
    {
        temp = temp/10;
        length++;
    }

    uint8_t i =0;

    if (length == 2)
    {
        for(i=1; i <= length; i++)
        {
            char c = (num % 10) + '0';
            num = num/10;
//        putcUart0(c);
            getBack[length - i] = c;
        }
    }

    else if (length == 1)
    {
        getBack[1] = (num % 10) + '0';
    }

    putsUart0(getBack);
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
                {
                    tcb[i].name[j] = '\0';
                    break;
                }
                tcb[i].name[j] = name[j];
            }
            tcb[i].nameLength = j;

            tcb[i].state = STATE_UNRUN;
            tcb[i].pid = fn;


            //calculate the stackBytes in terms of 1 KiB (1024Bytes) regions
            //for instance, if a thread/task only requires 100 Bytes of stack
            //we still have to allocate it a 1KiB region since our MPU is based on this
            uint32_t stackTemp = stackBytes/1024;
            if (stackBytes%1024 > 0)
                stackTemp++;

            stackBytes = stackTemp * 1024;

            //calculating the permissions field for the thread/task
            //This is used to determine which sub-region/s to give it access to and which not to give access
            tcb[i].MPUpermissions = 0xFF;
            //uint8_t numSubRegions = stackBytes/1024;
            tcb[i].MPUpermissions >>= (8-stackBytes/1024);
            tcb[i].MPUpermissions <<= ((heap_pointer - 0x20000000)/0x400);


            //moving the heap pointer to the stack space of this thread
            tcb[i].sp = heap_pointer;
            heap_pointer += stackBytes;
            tcb[i].spInit = heap_pointer;


            tcb[i].priority = priority;
            tcb[i].currentPriority = priority;
            // increment task count
            taskCount++;
            ok = true;

        }
    }
    // REQUIRED: allow tasks switches again
    return ok;
}

// REQUIRED: modify this function to restart a thread
void restartThread(_fn fn)
{
    __asm("   SVC  #60");
}

// REQUIRED: modify this function to destroy a thread
// REQUIRED: remove any pending semaphore waiting
// NOTE: see notes in class for strategies on whether stack is freed or not
void destroyThread(_fn fn)
{
    kill(fn);
}

// REQUIRED: modify this function to set a thread priority
void setThreadPriority(_fn fn, uint8_t priority)
{
    __asm("     SVC  #56");
}

//int8_t createSemaphore(uint8_t count, char name[])
//{
//    int8_t index = -1;
//    if (semaphoreCount < MAX_SEMAPHORES)
//    {
//        semaphores[semaphoreCount].count = count;
//        index = semaphoreCount;
//        semaphoreCount++;
//
//        uint8_t i = 0;
//        while(name[i] != 0)
//        {
//            semaphores[index].name[i] = name[i];
//            i++;
//        }
//        semaphores[index].name[i] = '\0';
//    }
//    return index;
//}


bool createSemaphore(uint8_t s, uint8_t count, char name[])
{
    bool ok = (s < MAX_SEMAPHORES);
    if (ok)
    {
        semaphores[s].count = count;

        uint8_t i = 0;
        while(name[i] != 0)
        {
          semaphores[s].name[i] = name[i];
          i++;
         }
        semaphores[s].name[i] = '\0';

    }
    return ok;
}


// REQUIRED: modify this function to start the operating system, using all created tasks
void startRtos()
{
    taskCurrent = rtosScheduler();
    uint32_t sp = tcb[taskCurrent].spInit;

    //for some reason, when the PSP stack pointer is set and function is called using fn(),
    //the stack pointer tends to move upward by 1 byte; subtracting 12 before setting the PSP gives a "breathing space"
    //for the task to run
    setPSPaddress(sp-12);   //this is only needed once; when the tasks are switched using pendSV it is not needed.
                            //after several trials and errors, figured out this was the way to "prime the pump" for this step

    TIMER1_CTL_R |= TIMER_CTL_TAEN;         //turn on the timer

    tcb[taskCurrent].state = STATE_READY;   //set the state to ready

//    NVIC_MPU_NUMBER_R   = 0x3;
//    NVIC_MPU_ATTR_R   &= ~0xFF00;
//    NVIC_MPU_ATTR_R   |=  0x0400;

    setASPbit();            //set the ASP bit in the control register. After this, the thread mode uses PSP

    _fn fn = (_fn) tcb[taskCurrent].pid;    //pointer to the first thread to be run after starting the RTOS

    uint8_t i = 2;
    for(i=2; i<6; i++)                  //configuring the MPUATTR registers based on the stack size of the task
    {
        NVIC_MPU_NUMBER_R   = i;
        NVIC_MPU_ATTR_R &= ~(0xFF00);
        NVIC_MPU_ATTR_R |= ((tcb[taskCurrent].MPUpermissions >> 8*(i-2)) & 0xFF) << 8;
    }


    unprivilegedMode();     //turn on bit 0 in the CONTROL register for unprivileged mode
    fn();
}


//  this function is used to yield execution back to scheduler using pendsv
// push registers, call scheduler, pop registers, return to new function
void yield()
{
   __asm(" SVC #10");

}

// this function is used to support 1ms system timer
// execution yielded back to scheduler until time elapses using pendsv
// push registers, set state to delayed, store timeout, call scheduler, pop registers,
// return to new function (separate unrun or ready processing)
void sleep(uint32_t tick)
{
    __asm(" SVC #20");
}


//this function is used to wait a semaphore with priority inheritance
// return if avail (separate unrun or ready processing), else yield to scheduler using pendsv
void wait(int8_t semaphore)
{
    __asm(" SVC #30");
}

// this function is used to signal a semaphore is available using pendsv
void post(int8_t semaphore)
{
    __asm(" SVC #40");
}

// this function is used to add support for the system timer
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

    if (systickCount == 100)
    {
        pingpong = 1 - pingpong;
        systickCount = 0;
        uint8_t i = 0;
        while (i < (MAX_TASKS + 2))
        {
            CPU_TIME[pingpong][i] = 0;
            i++;
        }
    }

    systickCount++;

    //start preemption code here
    if (preemptON)
        NVIC_INT_CTRL_R   |= 0x10000000;

    //end preemption code here
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
// this function adds support for task switching
// process UNRUN and READY tasks are handled differently
void pendSvIsr()
{
    PushRegstoPSP();
    tcb[taskCurrent].spInit = getPSPaddress();

    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off timer
    TIMER1_IMR_R &= ~TIMER_IMR_TATOIM;              // turn-off interrupts
    CPU_TIME [pingpong][taskCurrent] += TIMER1_TAV_R;
    CPU_TIME [pingpong][TOTAL_TIME_INDEX] += TIMER1_TAV_R;



    taskCurrent=rtosScheduler();


    uint8_t i = 2;
    for(i=2; i<6; i++)          //configuring the MPUATTR register for the particular stack of the current task
    {
        NVIC_MPU_NUMBER_R   = i;
        NVIC_MPU_ATTR_R &= ~(0xFF00);
        NVIC_MPU_ATTR_R |= ((tcb[taskCurrent].MPUpermissions >> 8*(i-2)) & 0xFF) << 8;
    }

    if(tcb[taskCurrent].state == STATE_READY)
    {
        setPSPaddress(tcb[taskCurrent].spInit);
        PopRegsFromPSP();
        TIMER1_TAV_R=0;
        TIMER1_CTL_R |= TIMER_CTL_TAEN;  // turn on timer for this thread
    }

    else            //if the state is UNRUN
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

        TIMER1_TAV_R=0;
        TIMER1_CTL_R |= TIMER_CTL_TAEN;  // turn on timer for this thread
     }

}

// this function adds support for the service call
void svCallIsr()
{
    uint32_t r0, r1, r2, r3, r12;
    uint8_t N;
    //getting the PSP stack of the process
    uint32_t* psp = (uint32_t*) getPSPaddress();
    r0 = *(psp);
    r1 = *(psp + 1);
    r2 = *(psp + 2);
    r3 = *(psp + 3);
    r12 = *(psp + 4);

    N = getSVCNumber();

   switch(N)
   {
       //_________________________________________________yield_________________________________________________________________
       case 10:
               NVIC_INT_CTRL_R   |= 0x10000000;  //setting the pendSV active bit at bit #28  of the NVIC_INT_CTRL register to enable the PendSV ISR call
               break;

       //_________________________________________________sleep______________________________________________________________
       case 20:
               tcb[taskCurrent].ticks = r0;
               tcb[taskCurrent].state = STATE_DELAYED;
               NVIC_INT_CTRL_R   |= 0x10000000;  //setting the pendSV active bit at bit #28  of the NVIC_INT_CTRL register to enable the PendSV ISR call

               NVIC_ST_RELOAD_R = 39999;
               NVIC_ST_CTRL_R |= 0x7;

               break;

       //___________________________________________________wait_______________________________________________________________
       case 30:
               if (semaphores[r0].count > 0)
                   (semaphores[r0].count--);

               else
               {
                   tcb[taskCurrent].state = STATE_BLOCKED;
                   tcb[taskCurrent].semaphore = &semaphores[r0];
                   semaphores[r0].processQueue[semaphores[r0].queueSize] = taskCurrent;
                   (semaphores[r0].queueSize)++;


                   //start editing priority inheritance here
                   if(piON)
                   {
                       uint8_t piIndex = 0;
                       bool EXIST = false;
                       while (piIndex < MAX_TASKS && !EXIST)
                       {
                           if ((tcb[piIndex].semaphore == &semaphores[r0]) && tcb[piIndex].currentPriority > tcb[taskCurrent].currentPriority)
                           {

                               tcb[piIndex].currentPriority = tcb[taskCurrent].currentPriority;
                               EXIST = true;
                           }
                           piIndex++;
                       }
                   //end editing priority inheritance here
                   }

                   NVIC_INT_CTRL_R   |= 0x10000000;  //setting the pendSV active bit at bit #28  of the NVIC_INT_CTRL register to enable the PendSV ISR call
               }
               break;

       //___________________________________________________________post______________________________________________________
       case 40:
               (semaphores[r0].count)++;
               if(semaphores[r0].queueSize > 0)
               {
                   if(tcb[semaphores[r0].processQueue[0]].state == STATE_BLOCKED)
                   {
                       tcb[semaphores[r0].processQueue[0]].state = STATE_READY;
                       (semaphores[r0].count--);

                       //start editing priority inheritance here
                       if(piON && tcb[taskCurrent].currentPriority != tcb[taskCurrent].priority)
                       {
                           tcb[taskCurrent].currentPriority = tcb[taskCurrent].priority;
                       //end editing priority inheritance here
                       }
                   }

                   uint8_t pQ = 0;
                   while(pQ < (semaphores[r0].queueSize - 1))
                   {
                       semaphores[r0].processQueue[pQ] = semaphores[r0].processQueue[pQ + 1];
                       pQ++;
                   }

                   (semaphores[r0].queueSize--);

               }
               break;

       //__________________________________________kill PID; destroyThread_____________________________________________________
       case 50: ;
               uint8_t j;
               j = 0;
               bool check = false;
               while (j < MAX_TASKS)
               {
                   if (tcb[j].pid == r0 && (tcb[j].state != STATE_KILLED && tcb[j].state != STATE_UNRUN))
                   {
                       check = true;
                       if (stringCompare(tcb[j].name, "idle"))
                       {
                           putsUart0("Cannot kill idle\n\r");
                           break;
                       }


                       //freeing up the semaphore if the task to be killed is using a semaphore
                       semaphore* sem =  (tcb[j].semaphore);
                       if(sem != 0x00000000)    //if the task has a valid semaphore entry
                       {
                           if(tcb[j].state == STATE_READY || tcb[j].state == STATE_DELAYED)     //if the task is currently running or holding a semaphore
                           {                                                                    //free up that semaphore and give it to the next task in the queue
                               sem->count++;
                               if(sem->queueSize > 0)
                               {
                                   if(tcb[sem->processQueue[0]].state == STATE_BLOCKED)
                                   {
                                       tcb[sem->processQueue[0]].state = STATE_READY;
                                       (sem->count--);
                                   }

                                   uint8_t pQ = 0;
                                   while(pQ < (sem->queueSize - 1))
                                   {
                                       sem->processQueue[pQ] = sem->processQueue[pQ + 1];
                                       pQ++;
                                   }

                                   (sem->queueSize--);

                               }
                           }

                           else     //if the task is currently being blocked by a semaphore
                           {        //remove it from the semaphore waiting queue
                               uint8_t taskQueueIndex = 0;
                               while (sem->processQueue[taskQueueIndex] != j) { taskQueueIndex++;}   //find the index of the task in the semaphoreQueue
                                                                                                    //we do this to remove it from the queue and move the other waiting task to its place in the queue

                               uint8_t pQ = taskQueueIndex;
                               while(pQ < (sem->queueSize - 1))
                               {
                                   sem->processQueue[pQ] = sem->processQueue[pQ + 1];
                                   pQ++;
                               }

                               (sem->queueSize--);
                           }

                       }


                       //changing the state of task to killed
                       tcb[j].state = STATE_KILLED;
                       PrintIntToHex(r0);
                       putsUart0(" Killed\n\r");

                       break;
                   }
                   j++;
               }

               if (!check)  //if task not found; maybe the task has already been killed; or doesn't exist at all
                   putsUart0("pid not found\n\r");

               break;

       //____________________________________________________reboot__________________________________________________________
       case 51:
               NVIC_APINT_R = 0x05FA0004;
               break;

       //_________________________________________________run task_name_______________________________________________________
       case 52: ;
               char *Process_Name = r0;
               uint8_t ind = 0;
               bool notFound = true;
               while(ind < MAX_TASKS)
               {
                   if (stringCompare(Process_Name, tcb[ind].name))
                   {
                       notFound = false;
                       if (tcb[ind].state == STATE_UNRUN || tcb[ind].state == STATE_KILLED)
                       {
                           tcb[ind].state = STATE_READY;

                           if(tcb[ind].semaphore != 0x00000000)
                           {
                               semaphore* sem = (tcb[ind].semaphore);
                               tcb[ind].state = STATE_BLOCKED;
                               sem->processQueue[sem->queueSize] = ind;
                               sem->queueSize++;
                           }
                       }

                       else
                       {
                           putsUart0(tcb[ind].name);
                           putsUart0(" already running\n\r");
                       }
                       break;
                   }
                   ind++;
               }
               if (notFound)
               {
                   putsUart0(Process_Name);
                   putsUart0(" not found\n\r");
               }

               break;

//_____________________________________________________________ps______________________________________________________
       case 53: ;
               uint8_t ind1 = 0;
               struct PSData* array = r0;
               uint32_t temporary = 0;

               CPU_TIME [1 - pingpong] [KERNEL_TIME_INDEX] = 4000000 - CPU_TIME [1 - pingpong] [TOTAL_TIME_INDEX];

               temporary = (CPU_TIME[1-pingpong][KERNEL_TIME_INDEX])/400;

               //store kernel time index
               array[0].kernelTime = temporary;

               if (r0 < tcb[taskCurrent].sp) {putsUart0("Shell stack size not enough!\n\r"); break;}   //struct array size around 288 bytes

               for(ind1 = 0; ind1<MAX_TASKS; ind1++)
               {
                   if(ind1 < MAX_TASKS && tcb[ind1].state != STATE_INVALID)
                   {
                       uint8_t nameInd = 0;
                       while(tcb[ind1].name[nameInd] != 0)
                       {
                           array[ind1].name[nameInd] = tcb[ind1].name[nameInd];
                           nameInd++;
                       }
                       array[ind1].name[nameInd] = '\0';
                       array[ind1].nameLength = nameInd;

                       //store pid
                       array[ind1].pid = tcb[ind1].pid;

                       //store cpu time %
                       temporary = (CPU_TIME[1-pingpong][ind1])/400;     //         temporary = raw-value/40E5 * 10000 = raw-value/400; 40E5 because we clear it every N ms(=100 ms)
                       array[ind1].CPU = temporary;

                       //store priority
                       array[ind1].prio = tcb[ind1].currentPriority;

                       //set valid bit
                       array[ind1].valid = 1;
                   }

                   else
                       array[ind1].valid = 0;
               }

               break;

               //________________________________________________________pidOf_______________________________________________
       case 54: ;
               uint8_t ind2 = 0;
               bool pidFound = false;
               char* checkSTRING = (char*) r0;
               for (ind2 = 0; ind2 < MAX_TASKS; ind2++)
                   {
                       if (stringCompare(checkSTRING, tcb[ind2].name))
                       {
                           putsUart0("pid of ");
                           putsUart0(checkSTRING);
                           putsUart0(": ");
                           PrintIntToHex(tcb[ind2].pid);
                           putsUart0("\n\r");
                           pidFound = true;
                           break;
                       }
                   }
               if (!pidFound)
               {
                   putsUart0(r0);
                   putsUart0(" not found\n\r");
               }
               break;

               //___________________________________________________ipcs_____________________________________________________
       case 55: ;
               uint8_t ind3 = 0;
               for (ind3 = 0; ind3< MAX_SEMAPHORES; ind3++)
               {
                  if(semaphores[ind3].queueSize > 0)
                  {
                       putsUart0(semaphores[ind3].name);
                       putsUart0("\t");
                       getIntString(semaphores[ind3].count);
                       putsUart0("\t");

                       uint8_t waiting = 0;
                       for (waiting=0; waiting<semaphores[ind3].queueSize; waiting++)
                       {
                           putsUart0(tcb[semaphores[ind3].processQueue[waiting]].name);
                           putsUart0("\n");
                       }
                       putsUart0("\r");
                   }
               }
               break;

               //_________________________________________________setThreadPriority______________________________________________
       case 56: ;
               uint8_t ind4 = 0;
               _fn testfn = r0;
               for (ind4 = 0; ind4 < MAX_TASKS; ind4++)
               {
                   if (tcb[ind4].pid == testfn)
                   {
                       tcb[ind4].currentPriority = r1;
                       break;
                   }
               }
               break;

               //______________________________________________priorityScheduling_______________________________________________
       case 57:
               if (r0)
               {
                   priorityScheduling = true;
                   putsUart0("Priority Scheduling turned ON\n\r");
               }
               else
               {
                   priorityScheduling = false;
                   putsUart0("Round Robin Scheduling turned ON\n\r");
               }
               break;

               //_________________________________________Priority Inheritance__________________________________________________
       case 58:
               if(r0)
               {
                   piON = true;
                   putsUart0("Priority Inheritance turned on\n\r");
               }

               else
               {
                   piON = false;
                   putsUart0("Priority Inheritance turned OFF\n\r");
               }
              break;

               //_______________________________________________preempt_________________________________________________________
       case 59:
               if(r0)
               {
                   preemptON = true;
                   putsUart0("Preemption turned ON\n\r");
               }
               else
               {
                   preemptON = false;
                   putsUart0("Preemption turned OFF\n\r");
               }
               break;

               //_________________________________________________restart thread______________________________________________
       case 60: ;
               uint32_t Process_ID = r0;
               uint8_t ind5 = 0;
//               notFound = true;
               while(ind5 < MAX_TASKS)
               {
                   if (tcb[ind5].pid == Process_ID)
                   {
//                       notFound = false;
                       if (tcb[ind5].state == STATE_UNRUN || tcb[ind5].state == STATE_KILLED)
                       {
                           tcb[ind5].state = STATE_READY;

                           if(tcb[ind5].semaphore != 0x00000000)
                           {
                               semaphore* sem = (tcb[ind5].semaphore);
                               tcb[ind5].state = STATE_BLOCKED;
                               sem->processQueue[sem->queueSize] = ind5;
                               sem->queueSize++;
                           }
                       }

                       break;
                   }
                   ind5++;
               }

               break;


   }

}


//Handler for MPU Faults
void mpuFaultIsr()
{
       NVIC_INT_CTRL_R   |= 0x10000000;  //setting the pendSV active bit at bit #28  of the NVIC_INT_CTRL register to enable the PendSV ISR call
       NVIC_SYS_HND_CTRL_R  &= ~0x2000;     //clearing the MPU fault pending bit at bit #13 of the NVIC_SYS_HNDCTRL register
       //_delay_cycles(5);

       putsUart0(Line_Break);
       putsUart0("MPU Fault in process ");
       putsUart0(tcb[taskCurrent].name);
       putsUart0(Line_Break);

       putsUart0("PSP: ");
       PrintIntToHex(getPSPaddress());
       putsUart0(Line_Break);

       putsUart0("MSP: ");
       PrintIntToHex(getMSPaddress());
       putsUart0(Line_Break);

       putsUart0("MFault Flag: ");
       PrintIntToHex(NVIC_FAULT_STAT_R & 0xFF); //accessing the MFault flags in the NVIC_FAULT_STAT_R at bits 0:7
       putsUart0(Line_Break);


       //Process Stack Dump
       uint32_t* PSP_Address = (uint32_t*) getPSPaddress();


       putsUart0("Offending instruction address: ");
       PrintIntToHex(*(PSP_Address + 6));
       putsUart0(Line_Break);

       putsUart0("Offending Data address: ");
       PrintIntToHex(NVIC_MM_ADDR_R);
       putsUart0(Line_Break);


//       putsUart0("Offending thread: ");
//       putsUart0(tcb[taskCurrent].name);
//       putsUart0(Line_Break);

       putsUart0("xPSR: ");
       PrintIntToHex(*(PSP_Address + 7));
       putsUart0(Line_Break);

       putsUart0("PC:   ");
       PrintIntToHex(*(PSP_Address + 6));
       putsUart0(Line_Break);

       putsUart0("LR:   ");
       PrintIntToHex(*(PSP_Address + 5));
       putsUart0(Line_Break);

       putsUart0("R12:  ");
       PrintIntToHex(*(PSP_Address + 4));
       putsUart0(Line_Break);

       putsUart0("R3:   ");
       PrintIntToHex(*(PSP_Address + 3));
       putsUart0(Line_Break);

       putsUart0("R2:   ");
       PrintIntToHex(*(PSP_Address + 2));
       putsUart0(Line_Break);

       putsUart0("R1:   ");
       PrintIntToHex(*(PSP_Address + 1));
       putsUart0(Line_Break);

       putsUart0("R0:   ");
       PrintIntToHex(*PSP_Address);
       putsUart0(Line_Break);

       tcb[taskCurrent].state = STATE_KILLED;
     //  NVIC_SYS_HND_CTRL_R  |= 0x400;


}

// Handler for Hard Fault
void hardFaultIsr()
{
    putsUart0("Hard fault in process ");
    putsUart0(tcb[taskCurrent].name);
    putsUart0(Line_Break);

    putsUart0("PSP: ");
    PrintIntToHex(getPSPaddress());
    putsUart0(Line_Break);

    putsUart0("MSP: ");
    PrintIntToHex(getMSPaddress());
    putsUart0(Line_Break);

    putsUart0("Hard Fault Flag: ");
    PrintIntToHex(NVIC_HFAULT_STAT_R);
    putsUart0(Line_Break);


}

// Handler for bus fault
void busFaultIsr()
{
    putsUart0("Bus fault in process ");
    putsUart0(tcb[taskCurrent].name);
    putsUart0(Line_Break);

}

// Handler for usage fault
void usageFaultIsr()
{
    putsUart0("Usage fault in process ");
    putsUart0(tcb[taskCurrent].name);
    putsUart0(Line_Break);

}

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
// initialization for blue, orange, red, green, and yellow LEDs
// 6 pushbuttons, and Timer 1
void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, sysdivider of 5, creating system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable clocks
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4 | SYSCTL_RCGCGPIO_R0 | SYSCTL_RCGCGPIO_R5;
    SYSCTL_RCGCTIMER_R |=SYSCTL_RCGCTIMER_R1;
    _delay_cycles(3);

    // Configure LED and pushbutton pins
    GPIO_PORTE_DIR_R |= GREEN_LED_MASK | RED_LED_MASK | YELLOW_LED_MASK | ORANGE_LED_MASK;
    GPIO_PORTE_DR2R_R |= GREEN_LED_MASK | RED_LED_MASK | YELLOW_LED_MASK | ORANGE_LED_MASK;
    GPIO_PORTE_DEN_R |= GREEN_LED_MASK | RED_LED_MASK | YELLOW_LED_MASK | ORANGE_LED_MASK;

    GPIO_PORTF_DIR_R |= BLUE_LED_MASK;
    GPIO_PORTF_DR2R_R |= BLUE_LED_MASK;
    GPIO_PORTF_DEN_R |=  BLUE_LED_MASK;

    GPIO_PORTA_DIR_R  &= ~(PUSH_BUTTON_MASK_0 | PUSH_BUTTON_MASK_1 | PUSH_BUTTON_MASK_2 | PUSH_BUTTON_MASK_3 | PUSH_BUTTON_MASK_4 |PUSH_BUTTON_MASK_5);
    //GPIO_PORTA_DR2R_R |= PUSH_BUTTON_MASK_0 | PUSH_BUTTON_MASK_1 | PUSH_BUTTON_MASK_2 | PUSH_BUTTON_MASK_3 | PUSH_BUTTON_MASK_4 |PUSH_BUTTON_MASK_5;
    GPIO_PORTA_DEN_R |= PUSH_BUTTON_MASK_0 | PUSH_BUTTON_MASK_1 | PUSH_BUTTON_MASK_2 | PUSH_BUTTON_MASK_3 | PUSH_BUTTON_MASK_4 |PUSH_BUTTON_MASK_5;

    GPIO_PORTA_PUR_R |= PUSH_BUTTON_MASK_0 | PUSH_BUTTON_MASK_1 | PUSH_BUTTON_MASK_2 | PUSH_BUTTON_MASK_3 | PUSH_BUTTON_MASK_4 |PUSH_BUTTON_MASK_5;
    // enable internal pull-up for push button


    // Configure Timer 1 as the time base
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD | TIMER_TAMR_TACDIR;          // configure for periodic mode (count up)

    TIMER1_TAILR_R =40000000;                         // set load value to 40E6 for 1 Hz interrupt rate
    TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
    NVIC_EN0_R |= 1 << (INT_TIMER1A-16);            // turn-on interrupt 37 (TIMER1A)
    TIMER1_CTL_R |= TIMER_CTL_TAEN;                // turn-on timer
}


//Timer1 Interrupt Service Routine
void timer1Isr()
{
    //TIMER1_TAV_R=0;
    TIMER1_ICR_R = TIMER_ICR_TATOCINT;               // clear interrupt flag
}


// code to return a value from 0-63 indicating which of 6 PBs are pressed
uint8_t readPbs()
{
    uint8_t readValue = 0;
    if(!PUSH_BUTTON0)
        readValue += 1;

    if(!PUSH_BUTTON1)
        readValue += 2;

    if(!PUSH_BUTTON2)
        readValue += 4;

    if(!PUSH_BUTTON3)
        readValue += 8;

    if(!PUSH_BUTTON4)
        readValue += 16;

    if(!PUSH_BUTTON5)
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


// one task must be ready at all times or the scheduler will fail
// the idle task is implemented for this purpose
void idle()
{
    while(true)
    {
        ORANGE_LED = 1;
        waitMicrosecond(1000);
        ORANGE_LED = 0;

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



void shell()
{
    USER_DATA data;
    while(1)
    {
    putsUart0("\r");
    getsUart0(&data);
    putsUart0("\n\r");
    putsUart0(data.buffer);
    putsUart0("\n\r");

    parseFields(&data);
    bool valid=false;


    if(isCommand(&data,"ps",0))
    {
       struct PSData psdata[MAX_TASKS];
       ps(&psdata);
       uint8_t ind1 = 0;

       putsUart0("Name\t\t\tPID\t\t\t%CPU\t\tPriority\n\r\n");

       for(ind1 = 0; ind1<MAX_TASKS; ind1++)
       {
           if(psdata[ind1].valid)
           {
               putsUart0(psdata[ind1].name);
               if(psdata[ind1].nameLength >=8)
                   putsUart0("\t\t");
               else
                   putsUart0("\t\t\t");

               PrintIntToHex(psdata[ind1].pid);
               putsUart0("\t\t");

               getIntString((psdata[ind1].CPU) / 100);
               putcUart0('.');

               getIntString((psdata[ind1].CPU) % 100);
               putsUart0(" %\t\t");

               getIntString(psdata[ind1].prio);
               putsUart0("\n\r");
           }
       }

       putsUart0("\nKernel Time\t\t\t\t\t");
       getIntString(psdata[0].kernelTime / 100);
       putcUart0('.');
       getIntString(psdata[0].kernelTime % 100);
       putsUart0(" %\t\n\r");

       putsUart0("TOTAL\t\t\t\t\t\t");
       putsUart0("100.00%");
       putsUart0("\t\n\r");

       valid = true;
    }

    else if (isCommand(&data, "ipcs", 0))
    {
        ipcs();
        valid = true;
    }

    else if (isCommand(&data, "kill", 1))
    {
        kill(hexToInt(getFieldString(&data, 1)));
        valid = true;
    }

    else if (isCommand(&data, "pi", 1))
    {
        char* ONOFF = getFieldString(&data, 1);
        if (stringCompare("ON", ONOFF) || stringCompare("OFF", ONOFF))
        {
            if(stringCompare(ONOFF, "ON"))
                pi(1);

            else
                pi(0);

            valid = true;
        }
    }

    else if (isCommand(&data, "preempt", 1))
    {
        char* ONOFF = getFieldString(&data, 1);
        if (stringCompare("ON", ONOFF) || stringCompare("OFF", ONOFF))
        {
            if(stringCompare(ONOFF, "ON"))
                preempt(1);

            else
                preempt(0);

            valid = true;
        }

    }

    else if (isCommand(&data, "sched", 1))
    {
        if (stringCompare("PRIO", getFieldString(&data, 1)) || stringCompare("RR", getFieldString(&data, 1)))
        {
            if(stringCompare("PRIO", getFieldString(&data, 1)))
                sched(1);

            else
                sched(0);

            valid = true;
        }
    }

    else if (isCommand(&data, "pidof", 1))
    {
        pidof(getFieldString(&data, 1));
        valid = true;
    }

    else if (isCommand(&data, "run", 1))
    {
      run(getFieldString(&data, 1));

      valid = true;
    }

    else if (isCommand(&data, "reboot", 0))
    {
        valid = true;
        reboot();
    }


    if(!valid)
        putsUart0("\rInvalid command\n");

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
    createSemaphore(keyPressed, 1, "keyPressed");
    createSemaphore(keyReleased, 0, "keyReleased");
    createSemaphore(flashReq, 5, "flashReq");
    createSemaphore(resource, 1, "resource");

    // Add required idle process at lowest priority
      ok =  createThread(idle, "Idle", 15, 1024);

//    // Add other processes
      ok &= createThread(lengthyFn, "LengthyFn", 12, 1024);
      ok &= createThread(flash4Hz, "Flash4Hz", 8, 1024);
      ok &= createThread(oneshot, "OneShot", 4, 1024);
      ok &= createThread(readKeys, "ReadKeys", 12, 1024);
      ok &= createThread(debounce, "Debounce", 12, 1024);
      ok &= createThread(important, "Important", 0, 1024);
      ok &= createThread(uncooperative, "Uncoop", 12, 1024);
      ok &= createThread(errant, "Errant", 12, 1024);
      ok &= createThread(shell, "Shell", 12, 1024);

    // Start up RTOS
    if (ok)
        startRtos(); // never returns
    else
        RED_LED = 1;

    return 0;
}
