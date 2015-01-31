#include <stdio.h>
#include <string.h>
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
  int ret; 
  ret = read(fp,&str,10);
  printf("Return form Read = %d\n",ret);
  printf("length of Read = %d\n",strlen(str)); 
  printf("Read from Module = \n", str);
  int i;
  for(i = 0; i <= strlen(str) ; i++){
     printf("%c",str[i]);
  }
   printf("\n");
  
  close(fp);
  return 0;
}