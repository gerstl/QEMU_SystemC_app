#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <fcntl.h>

#define REG_SIZE 256UL
#define REG_MASK (REG_SIZE - 1)

#define MEM_SIZE 64*1024UL
#define MEM_MASK (MEM_SIZE - 1)

int main(int argc, char * argv[]) {
  volatile unsigned int *base, *mem, *address;
  unsigned long addr1, addr2, addr3, addr4, addr0, addrm, offset, value;
  unsigned long val, result;

  //Predefined addresses.
  addr0 = 0xa0000000ul;  // DEBUG_TIME
  addr1 = 0xa0000004ul;  // DEBUG_WRITE
  addr2 = 0xa0000008ul;  // DEBUG_STOP
  addr3 = 0xa000000Cul;  // DEBUG_IRQ
  addr4 = 0xa0000010ul;  // DEBUG_REALTIME

  addrm = 0xa0800000ul;  // Memory

  //Ensure proper usage
  if(argc > 2)
  {
    printf("Usage: %s [val]\n",argv[0]);
    return -1;
  }

  //Open memory as a file
  int fd = open("/dev/mem", O_RDWR|O_SYNC);
  if(!fd)
    {
      printf("Unable to open /dev/mem\n");
      return -1;
    }	

  // Map the physical base address to local pointer (in virtual address space)
  base = (unsigned int *)mmap(NULL, REG_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, addr0 & ~REG_MASK);	
  if((base == MAP_FAILED))
  {
    printf("Register mapping failed\n");
    fflush(stdout);
    close(fd);
    return -1;
  }

  // Do the same for the device memory region
  mem = (unsigned int *)mmap(NULL, MEM_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, addrm & ~MEM_MASK);	
  if((mem == MAP_FAILED))
  {
    printf("Memory mapping failed\n");
    fflush(stdout);
    munmap((void*)base, REG_SIZE);
    close(fd);
    return -1;
  }

  if(argc > 1) {
    // Assign val
    val = atol(argv[1]);

    // Write to addr0
    address = base + ((addr0 & REG_MASK)>>2);
    *address = val;

    // and to memory
    *mem = val;

  } else {
    // Read hardware 
    address = base + ((addr0 & REG_MASK)>>2);
    result = *address;

    printf("The SystemC time is %lu ns\n", result);

    address = base + ((addr4 & REG_MASK)>>2);
    result = *address;

    printf("The SystemC clock is %lu\n", result);

    // Read memory
    result = *mem;

    printf("Memory dump is %lu\n", result);
  }

  // In the end, unmap the memory regions and close the memory file
  munmap((void*)base, REG_SIZE);
  munmap((void*)mem, MEM_SIZE);
  close(fd);

  return 0;
}
