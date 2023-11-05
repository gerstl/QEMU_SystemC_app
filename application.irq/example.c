#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <signal.h>
#include <unistd.h>
#include <assert.h>
#include <err.h>

#define READ_CMD  (0x0 << 31)
#define WRITE_CMD (0x1 << 31)

#define COMMAND_MASK 0x80000000


int det_int = 0;

// signal handler for receiving events from hardware driver
void sighandler(int signo)
{
  if(signo==SIGIO)
    {
      det_int++;
      printf("\nInterrupt detected\n");
    }
  return;
}


int main(int argc, char * argv[]) 
{
  unsigned long val, result;
  struct sigaction action;
  int fd;

  // ensure proper usage
  if(argc > 2)
  {
    printf("Usage: %s [val]\n",argv[0]);
    return -1;
  }

  // install signal handler
  sigemptyset(&action.sa_mask);
  sigaddset(&action.sa_mask, SIGIO);

  action.sa_handler = sighandler;
  action.sa_flags=0;

  if(sigaction(SIGIO, &action, NULL)) err(1, "Setting signal handler");

  // open hardware device (driver)
  fd=open("/dev/fpga", O_RDWR);
  if(fd < 0)
  {

      printf("Unable to open /dev/fpga.  Ensure it exists!\n");
      return -1;
  }
  if(fcntl(fd, F_SETOWN, getpid())) err(1, "Set owner");
  if(fcntl(fd, F_SETFL, fcntl(fd, F_GETFL)|O_ASYNC)) err(1, "Set flags");

  if(argc > 1) {
    // Assign val
    val = atol(argv[1]);

    // Write to addr0
    if(ioctl(fd, WRITE_CMD + 0, &val)) err(1, "Writing");

    // and to memory
    write(fd, &val, sizeof(val));

  } else {
    // Read hardware 
    if(ioctl(fd, READ_CMD + 0, &result)) err(1, "Reading SystemC time");

    printf("The SystemC time is %lu ns\n", result);

    if(ioctl(fd, READ_CMD + 4, &result)) err(1, "Reading SystemC clock");

    printf("The SystemC clock is %lu\n", result);

    // Read memory
    read(fd, &result, sizeof(result));

    printf("Memory dump is %lu\n", result);
  }

  // Read interrupt
  if(ioctl(fd, READ_CMD + 3, &result)) err(1, "Reading interrupt");
  printf("Interrupt is %lu\n", result);

  // Trigger interrupt
  val = 1;
  if(ioctl(fd, WRITE_CMD + 3, &val)) err(1, "Trigger interrupt");

  // Wait for interrupt
  while(!det_int) continue;

  printf("Interrupt received\n");

  //In the end, close the device driver
  close(fd);

  return 0;
}
