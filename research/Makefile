
CC = gcc
CFLAGS  = -g -Wall

# typing 'make' will invoke the first target entry in the file
# (in this case the default target entry)
# you can name this target entry anything, but "default" or "all"
# are the most commonly used names by convention
#
#include "BookA.h"
    #include "SheetA.h"
    #include "FormatA.h"
    #include "FontA.h"
    #include "AutoFilterA.h"
    #include "FilterColumnA.h"
default: MPPI

MPPI:  main.o myLib.o
	$(CC) $(CFLAGS) -o MPPI main.o -lm myLib.o -lm

main.o:  main.c myLib.h
	$(CC) $(CFLAGS) -c main.c

myLib.o:  myLib.c myLib.h
	$(CC) $(CFLAGS) -c myLib.c

# To start over from scratch, type 'make clean'.  This
# removes the executable file, as well as old .o object
# files and *~ backup files:
#
clean:
	$(RM) MPPI *.o *~
