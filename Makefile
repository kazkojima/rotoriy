obj= main.o RotorIy.o c3ga.o
src= $(obj:.cpp=.o)

CPFLAGS= -g -O3
CP= g++

all: dreck

dreck: $(obj)
	$(CP) $(CPFLAGS) -o $@ $(obj) -lm

%.o: %.cpp
	$(CP) $(CPFLAGS) -c -o $@ $<

c3ga.o: c3ga.h
RotorIy.o: RotorIy.h
main.o: c3ga.h RotorIy.h

clean:
	rm -f dreck *~ *.o
