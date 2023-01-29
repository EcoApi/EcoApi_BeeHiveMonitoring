
#include <Arduino.h>
#include <HardwareSerial.h>
#include <malloc.h>

HardwareSerial *_Debug = NULL;

extern "C" char *sbrk(int i);

/* Use linker definition */
extern char _end;
extern char _sdata;
extern char _estack;
extern char _Min_Stack_Size;

static char *ramstart = &_sdata;
static char *ramend = &_estack;
static char *minSP = (char*)(ramend - &_Min_Stack_Size);

#define NUM_BLOCKS 100
#define BLOCK_SIZE 4

void display_mallinfo(void) {
  char *heapend = (char*) sbrk(0);
  char *stack_ptr = (char*) __get_MSP();
  struct mallinfo mi = mallinfo();

  if(_Debug == NULL)
    return;

#if 0
  _Debug->print("Total non-mmapped bytes (arena):       ");
  _Debug->println(mi.arena);
  _Debug->print("# of free chunks (ordblks):            ");
  _Debug->println(mi.ordblks);
  _Debug->print("# of free fastbin blocks (smblks):     ");
  _Debug->println(mi.smblks);
  _Debug->print("# of mapped regions (hblks):           ");
  _Debug->println(mi.hblks);
  _Debug->print("Bytes in mapped regions (hblkhd):      ");
  _Debug->println(mi.hblkhd);
  _Debug->print("Max. total allocated space (usmblks):  ");
  _Debug->println(mi.usmblks);
  _Debug->print("Free bytes held in fastbins (fsmblks): ");
  _Debug->println(mi.fsmblks);
  _Debug->print("Total allocated space (uordblks):      ");
  _Debug->println(mi.uordblks);
  _Debug->print("Total free space (fordblks):           ");
  _Debug->println(mi.fordblks);
  _Debug->print("Topmost releasable block (keepcost):   ");
  _Debug->println(mi.keepcost);

  _Debug->print("RAM Start at:       0x");
  _Debug->println((unsigned long)ramstart, HEX);
  _Debug->print("Data/Bss end at:    0x");
  _Debug->println((unsigned long)&_end, HEX);
  _Debug->print("Heap end at:        0x");
  _Debug->println((unsigned long)heapend, HEX);
  _Debug->print("Stack Ptr end at:   0x");
  _Debug->println((unsigned long)stack_ptr, HEX);
  _Debug->print("RAM End at:         0x");
  _Debug->println((unsigned long)ramend, HEX);
#endif

  _Debug->print("[DEBUG] Heap RAM Used:      ");
  _Debug->println(mi.uordblks);
  _Debug->print("[DEBUG] Program RAM Used:   ");
  _Debug->println(&_end - ramstart);
  _Debug->print("[DEBUG] Stack RAM Used:     ");
  _Debug->println(ramend - stack_ptr);
  _Debug->print("[DEBUG] Estimated Free RAM: ");
  _Debug->println(((stack_ptr < minSP) ? stack_ptr : minSP) - heapend + mi.fordblks);
}

extern "C" {
void printErrorMsg(const char * errMsg);
void printUsageErrorMsg(uint32_t CFSRValue);
void printBusFaultErrorMsg(uint32_t CFSRValue);
void printMemoryManagementErrorMsg(uint32_t CFSRValue);
void stackDump(uint32_t stack[]);

void Hard_Fault_Handler(uint32_t stack[]) {
  static char msg[80];
  //if((CoreDebug->DHCSR & 0x01) != 0) {
    printErrorMsg("In Hard Fault Handler\n");
    sprintf(msg, "SCB->HFSR = 0x%08x\n", SCB->HFSR);
    printErrorMsg(msg);
    if ((SCB->HFSR & (1 << 30)) != 0) {
      printErrorMsg("Forced Hard Fault\n");
      sprintf(msg, "SCB->CFSR = 0x%08x\n", SCB->CFSR );
      printErrorMsg(msg);
  
      if((SCB->CFSR & 0xFFFF0000) != 0) {
        printUsageErrorMsg(SCB->CFSR);
      } 
  
      if((SCB->CFSR & 0xFF00) != 0) {
        printBusFaultErrorMsg(SCB->CFSR);
      }
  
      if((SCB->CFSR & 0xFF) != 0) {
        printMemoryManagementErrorMsg(SCB->CFSR);
      }      
    }  
  
    stackDump(stack);
    __ASM volatile("BKPT #01");
  //}
  
  while(true);
}

void printErrorMsg(const char * errMsg) {
  while(*errMsg != '\0') {
    ITM_SendChar(*errMsg);
    ++errMsg;
  }
}

void printUsageErrorMsg(uint32_t CFSRValue) {
  printErrorMsg("Usage fault: ");
  
  CFSRValue >>= 16; // right shift to lsb
   
  if((CFSRValue & (1<<9)) != 0) {
    printErrorMsg("Divide by zero\n");
  }
}

void printBusFaultErrorMsg(uint32_t CFSRValue) {
  printErrorMsg("Bus fault: ");
  
  CFSRValue = ((CFSRValue & 0x0000FF00) >> 8); // mask and right shift to lsb
}

void printMemoryManagementErrorMsg(uint32_t CFSRValue) {
  printErrorMsg("Memory Management fault: ");
   
  CFSRValue &= 0x000000FF; // mask just mem faults
}

enum { r0, r1, r2, r3, r12, lr, pc, psr};

void stackDump(uint32_t stack[]) {
   static char msg[80];
   sprintf(msg, "r0  = 0x%08x\n", stack[r0]);
   printErrorMsg(msg);
   sprintf(msg, "r1  = 0x%08x\n", stack[r1]);
   printErrorMsg(msg);
   sprintf(msg, "r2  = 0x%08x\n", stack[r2]);
   printErrorMsg(msg);
   sprintf(msg, "r3  = 0x%08x\n", stack[r3]);
   printErrorMsg(msg);
   sprintf(msg, "r12 = 0x%08x\n", stack[r12]);
   printErrorMsg(msg);
   sprintf(msg, "lr  = 0x%08x\n", stack[lr]);
   printErrorMsg(msg);
   sprintf(msg, "pc  = 0x%08x\n", stack[pc]);
   printErrorMsg(msg);
   sprintf(msg, "psr = 0x%08x\n", stack[psr]);
   printErrorMsg(msg);
}

__attribute__((naked)) void HardFault_Handler(void) {
  __asm volatile
  (
    " tst lr, #4                                                \n"
    " ite eq                                                    \n"
    " mrseq r0, msp                                             \n"
    " mrsne r0, psp                                             \n"
    " ldr r1, [r0, #24]                                         \n"
    " ldr r2, handler2_address_const                            \n"
    " bx r2                                                     \n"
    " handler2_address_const: .word Hard_Fault_Handler          \n"
  );
}
}

void setDebugSerial(HardwareSerial *Debug) {
  _Debug = Debug;
}