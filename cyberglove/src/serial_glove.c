
/** 
@file
  This program talks to a cyberglove on the serial port

  The only purpose is to continually sample the glove and to store the values in the
  array \c glove_positions

  The program provides the function \fn void* read_glove_sensors(void)
  that may be used together with a thread

  serial_glove.c
   (C) Shadow Robot Company 2006
   GPL
*/

//! sudo cc -c -I/usr/realtime/include/ serial_glove.c -o serial_glove.o



#include <stdio.h>
#include <error.h>
#include <stdio.h>
#include <unistd.h>

#include <termios.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <signal.h>
#include <string.h>
#include <pwd.h>
#include <sys/time.h>

#include "cyberglove/serial_glove.h"

int VERBOSE=0;

FILE * serial;

static float values[GLOVE_SIZE];

 
float glove_positions[GLOVE_SIZE];

int gloveButtonValue=0;

int glove_start_reads=0, glove_reads=0;

char glove_message[100];


struct termios termios_save;
static int serial_port_fd = -1;

void open_board(char *port) {
  struct termios termios_p;

  serial_port_fd = open(port, O_RDWR | O_NOCTTY); //will return a file descriptor based on an actual file

  if (serial_port_fd < 0) error(1, errno, "can't open serial port! maybe run as root...");

  //! get attributes associated with the serial port
  tcgetattr(serial_port_fd, &termios_p);

  //changes in termios_p won't affect termios_save behond this line...
  //memcpy(&termios_save, &termios_p, sizeof(struct termios)); //not used

  termios_p.c_cflag = B115200;
  termios_p.c_cflag |= CS8;
  termios_p.c_cflag |= CREAD;
  termios_p.c_iflag = IGNPAR | IGNBRK;
  termios_p.c_cflag |= CLOCAL;
  termios_p.c_oflag = 0;
  termios_p.c_lflag = 0; // NOT in canonical mode
  termios_p.c_cc[VTIME] = 1; //5; // half a second timeout
  termios_p.c_cc[VMIN] = 0;  // select timeout mode



  memcpy(&termios_save, &termios_p, sizeof(struct termios));
  
  //! trying the following lines to restart serial port, see read_stepping() function
  tcsetattr(serial_port_fd, TCSANOW, &termios_p);
  tcflush(serial_port_fd, TCOFLUSH);  
  tcflush(serial_port_fd, TCIFLUSH);

  //  unsigned char out_data[2]={0xff, 0};
  //  write(serial_port_fd, out_data, 1);

  sleep(1);
}





/**
called by read_values
*b is length GLOVE_SIZE + 2 because the first two characters returned are G and ' '
*/
int read_stepping(int fd, unsigned char *b, int n) {
  int i=0;
  //fd_set fdset;
  int res=0;
  int remain;
  //int counter=0;
  remain = n;

  do {
    res=read(fd, &b[i], remain);
    if (res>0) { 
      remain -= res;
      i += res;
    };

    if (res<0) { error(1, errno, "readg!"); };
    if (res==0) {
      strcpy(glove_message, "(glove read failed, is it connected?)");
      //counter++; error(0,0,"returned no data");
      //HACK without this the glove will sometimes freeze
      tcsetattr(serial_port_fd, TCSANOW, &termios_save);
      return 1;
    };

    if (remain) usleep (remain * 500); //wait for more characters to appear on the port
  } while ( /* (counter<10) && */ remain);

  return (remain!=0);
}

void  writeg(int fd, char *b, int n) {
  int i;
  for (i=0;i<n;i++) {
    write(fd, &b[i], 1);
  };
  //  usleep(10*n);
}

//read the position values from the glove
void read_values(float *dest) {
 restart:
  glove_start_reads++;
  tcflush(serial_port_fd, TCIFLUSH);
  tcflush(serial_port_fd, TCOFLUSH);
  writeg(serial_port_fd, "G", 1);
  int i;
  unsigned char ch[GLOVE_SIZE+2]={0}; //assigns 0 to the first char

  usleep(GLOVE_SIZE*10);
  //  int res=read(serial_port_fd, &ch, GLOVE_SIZE+2);
  //  if (res<0) error(1,errno, "reading from serial port");
  if (read_stepping(serial_port_fd, &ch, GLOVE_SIZE+2)) {
    //error(0,0,"serial port timeout");
    goto restart;
  };
  //  error(0,0,"res=%d, %02x %02x %02x %02x %02x %02x", res, ch[0],ch[1],ch[2],ch[3],ch[4],ch[5]);

  if (ch[0]!='G') {
    //error(0,0,"NO G");
    //    log_message("%s: Failure reading glove: No G", timestamp());
    goto restart; 
  };

  //! if any of the values are zero restart read as not considered reliable
  for (i=0; i<GLOVE_SIZE; i++) {
    if (ch[i+1]==0) { 
      //error(0,0,"Restart %d!", i);
      goto restart;
    };
  };

  for (i=0; i<GLOVE_SIZE; i++) {
    dest[i] = (ch[i+1]-1.0)/254.0;
  }


  strcpy(glove_message, "");  
  glove_reads++;
}

//read the button value from the glove
int read_button_value() {
 restartButtonRead:
  tcflush(serial_port_fd, TCIFLUSH);
  tcflush(serial_port_fd, TCOFLUSH);
  writeg(serial_port_fd, "?W", 2);
  int i;
  unsigned char ch[3]={0};    //assigns 0 to the first char

  //TODO : this should be reduced ? 
  usleep(GLOVE_SIZE*10);

  if (read_stepping(serial_port_fd, &ch, 3)) {
    goto restartButtonRead;
  };

  return ch[2];
}


int setup_glove(const char* path_to_glove)
 {
  open_board(path_to_glove);

  int i;
  for (i=0;i<GLOVE_SIZE;i++) 
    {
      values[i]=0.0;
      glove_positions[i] = 0.0;
    }
  
  return 0;
 }


//at end of file
//static void compute_setpoints(void);
 


//! sample the glove values - need to call setup_glove first
float* glove_get_values() {
  read_values(values);

  int i;
  for (i=0; i<GLOVE_SIZE; i++)
    glove_positions[i] = values[i];

  return glove_positions;
}


