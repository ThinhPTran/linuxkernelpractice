#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>

#define DEVICE_NODE "/dev/vchar_dev"

// fucntion to check entry point open of vchar driver
int open_chardev() {
  int fd = open(DEVICE_NODE, O_RDWR); 
  if (fd < 0) {
     printf("Can not open the device file\n"); 
     exit(1); 
  }
  return fd; 
}

// function to check entry point release of vchar driver
void close_chardev(int fd) {
  close(fd); 
}

int main() {
  int ret = 0; 
  char option = 'q'; 
  int fd = -1; 
  printf("Select below options:\n");
  printf("\to (to open a device node)\n");
  printf("\tc (to close a device node)\n");
  printf("\tq (to quite the application)\n");

  while(1) {
    printf("Enter your option: ");
    scanf(" %c", &option);  

    switch(option) {
      case 'o':
	if (fd < 0)
          fd = open_chardev(); 
	else 
	  printf("%s has already opended\n", DEVICE_NODE);
        break;
      case 'c':
        if (fd > -1)
          close_chardev(fd);
	else 
          printf("%s has not opened yet! Can not close\n", DEVICE_NODE);

        fd = -1; 	
	break;	
      case 'q':
	if (fd > -1) 
	  close_chardev(fd); 
	printf("Quit the application. Good bye!\n");
        return 0; 
      default:
        printf("invalid option %c\n", option);
        break;	
    }
  }	  
}
