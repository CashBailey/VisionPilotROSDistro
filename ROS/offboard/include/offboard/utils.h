

#ifndef UTILS_H 
#define UTILS_H

//Librerías para la función kbhit
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include<stdio.h>



int kbhit(void); //Windows kbhit function implemented for Linux 



#endif // UTILS_H

