CC = ccache g++
#CFLAGS = -g -Wall -std=c++17 -O0 -fsanitize=undefined,address -DLOCAL

# core dump出力用
#CFLAGS = -g -Wall --std=c++17 -O0 -DLOCAL

CFLAGS = -Wall --std=c++17 -O2 -DLOCAL
#CFLAGS = -pg -g -Wall --std=c++17 -O0

ALL: main.o SqDistScheduler.o Graph.o
	$(CC) $(CFLAGS) -o main \
	main.o \
	SqDistScheduler.o \
	Graph.o
	
clean:
	rm main *.o

run:
	./main

.cpp.o:
	$(CC) $(CFLAGS) -o $@ -c $<
