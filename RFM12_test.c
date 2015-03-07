#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>

char send_message[255] = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";

int main(voi){
  
  char str[20];
  
  printf("Hello World \n");

  int fp = open("/dev/RFM12_RW", O_RDWR);
  
  if (fp< 0) {
    printf("Problem opening %s\n","/dev/RFM12_RW");
    return 1;
  }
   int i = 1;
   int j = 0;
   while (1) {
	write(fp,send_message, i);
	i++;
	for (j=0; j<i; j++) {
	    printf("%c", send_message[j]);
	}
 	printf("\n");
	usleep(1000*1000);
	if(i > 10){
	    break;
	}
  }
  close(fp);
  
  printf("\n\nPROGRAM ENDED !!!!! \n");
  return 0;
}