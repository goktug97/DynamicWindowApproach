all: ./src/dwa.c ./src/dwa.h
	gcc -o dwa ./src/dwa.c -I./src -lm

