CC      = /usr/bin/gcc
CFLAGS  = -Wall -g 
LDFLAGS = -lm -lcurl
PREFIX  = usr/bin/
DESTDIR = 

OBJ = datalogger.o lowlevel.o datalog-decode.o main.o agps-download.o

PROG = skytraq-datalogger

$(PROG): $(OBJ)
	$(CC) $(CFLAGS) -o $(PROG) $(OBJ) $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c $<

clean:
	rm -f *.o $(PROG) test

install:
	cp  $(PROG)  $(DESTDIR)/$(PREFIX)
