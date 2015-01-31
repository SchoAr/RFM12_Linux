#include <stdio.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>

int main(voi){
  
  char str[20];
  
  printf("Hello World \n");

  int fp = open("/dev/RFM12_RW", O_RDWR);
  
  if (fp< 0) {
    printf("Problem opening %s\n","/dev/RFM12_RW");
    return 1;
  }
  
  write(fp,"Hallo", 5);
  close(fp);
  
  return 0;
}