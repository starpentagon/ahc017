CC = ccache g++
#CFLAGS = -g -Wall -std=c++17 -O0 -fsanitize=undefined,address -DLOCAL
# core dump出力用
#CFLAGS = -g -Wall --std=c++17 -O0 -DLOCAL

CFLAGS = -Wall --std=c++17 -O2 -DLOCAL
#CFLAGS = -pg -g -Wall --std=c++17 -O0

ALL: main.o Graph.o DualGraph.o FaceGroup.o FaceGroupSchedulerExp.o UnionFind.o XorShift.o ShortestTree.o
	$(CC) $(CFLAGS) -o main \
	main.o \
	Graph.o \
	FaceGroup.o \
	FaceGroupSchedulerExp.o \
	DualGraph.o \
	ShortestTree.o \
	UnionFind.o \
	XorShift.o \
	
clean:
	rm main *.o

run:
	./main

.cpp.o:
	$(CC) $(CFLAGS) -o $@ -c $<
