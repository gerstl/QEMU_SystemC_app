#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <fcntl.h>

#define MAP_SIZE 4096UL
#define MAP_MASK (MAP_SIZE - 1)

int main(int argc, char * argv[]) {
  volatile unsigned int *base, *address;
  unsigned long addr1, addr2, addr3, addr4, addr0, offset, value;
  unsigned long val, result;

  //Predefined addresses.
  addr0 = 0xa0000000ul;  // DEBUG_TIME
  addr1 = 0xa0000004ul;  // DEBUG_WRITE
  addr2 = 0xa0000008ul;  // DEBUG_STOP
  addr3 = 0xa000000Cul;  // DEBUG_IRQ
  addr4 = 0xa0000010ul;  // DEBUG_REALTIME

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
      printf("Unable to open /dev/mem.  Ensure it exists (major=1, minor=1)\n");
      return -1;
    }	

  //Map the physical base address to local pointer (in virtual address space)
  base = (unsigned int *)mmap(NULL, MAP_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, addr0 & ~MAP_MASK);	
  if((base == MAP_FAILED))
  {
    printf("mapping failed\n");
    fflush(stdout);
  }

  if(argc > 1) {
    //Assign val
    val = atol(argv[1]);

    //Write to addr0
    address = base + ((addr0 & MAP_MASK)>>2);
    *address = val;

  } else {

    //Read hardware 
    address = base + ((addr0 & MAP_MASK)>>2);
    result = *address;

    printf("The SystemC time is %lu ns\n", result);

    address = base + ((addr4 & MAP_MASK)>>2);
    result = *address;

    printf("The SystemC clock is %lu\n", result);
  }

  //In the end, unmap the base address and close the memory file
  munmap((void*)base, MAP_SIZE);
  close(fd);

  return 0;
}
