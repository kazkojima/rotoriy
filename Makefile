obj= main.o RotorIy.o c3ga.o
src= $(obj:.cpp=.o)

CPFLAGS= -g -O3
CP= g++

all: dreck

dreck: $(obj)
	$(CP) $(CPFLAGS) -o $@ $(obj) -lm

%.o: %.cpp
	$(CP) $(CPFLAGS) -c -o $@ $<

main.cpp, c3ga.cpp: c3ga.h

clean:
	rm -f dreck *~ *.o
