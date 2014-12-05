CC = g++
CFLAGS = -DGL_GLEXT_PROTOTYPES -Wall -ansi -Ieigen -Iglut-3.7.6-bin -O3
CFLAGS_g = -g -DGL_GLEXT_PROTOTYPES -Wall -ansi -Ieigen -Iglut-3.7.6-bin
LDFLAGS = -lGL -lGLU -lglut
all: main
main:  main.cpp 
	$(CC) $(CFLAGS) main.cpp -o main $(LDFLAGS)


clean:
	rm -rf main