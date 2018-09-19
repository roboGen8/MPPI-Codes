
CC = gcc
CFLAGS  = -g -Wall

default: MPPI

MPPI:  mainMPPI.o myLib.o
	$(CC) $(CFLAGS) -o MPPI mainMPPI.o -lm myLib.o -lm

main.o:  mainMPPI.c myLib.h
	$(CC) $(CFLAGS) -c mainMPPI.c

myLib.o:  myLib.c myLib.h
	$(CC) $(CFLAGS) -c myLib.c

# To start over from scratch, type 'make clean'.  This
# removes the executable file, as well as old .o object
# files and *~ backup files:
#
clean:
	$(RM) MPPI *.o *~
