CC = ccache g++
#CFLAGS = -g -Wall -std=c++17 -O0 -fsanitize=undefined,address -DLOCAL

# core dump出力用
#CFLAGS = -g -Wall --std=c++17 -O0 -DLOCAL

CFLAGS = -Wall --std=c++17 -O2 -DLOCAL
#CFLAGS = -pg -g -Wall --std=c++17 -O0

ALL: main.o Graph.o ConnectionSet.o XorShift.o UnionFind.o BypassSet.o
	$(CC) $(CFLAGS) -o main \
	main.o \
	Graph.o \
	ConnectionSet.o \
	XorShift.o \
	UnionFind.o \
	 BypassSet.o \

clean:
	rm main *.o

run:
	./main

Graph.o: ../Graph.cpp
	$(CC) $(CFLAGS) -o $@ -c $<

ConnectionSet.o: ../ConnectionSet.cpp
	$(CC) $(CFLAGS) -o $@ -c $<

XorShift.o: ../XorShift.cpp
	$(CC) $(CFLAGS) -o $@ -c $<

UnionFind.o: ../UnionFind.cpp
	$(CC) $(CFLAGS) -o $@ -c $<

BypassSet.o: ../BypassSet.cpp
	$(CC) $(CFLAGS) -o $@ -c $<

.cpp.o:
	$(CC) $(CFLAGS) -o $@ -c $<
