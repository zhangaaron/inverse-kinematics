CC = g++
CFLAGS = -DGL_GLEXT_PROTOTYPES -Wall -ansi -Ieigen -Iglut-3.7.6-bin
CFLAGS_g = -g -DGL_GLEXT_PROTOTYPES -Wall -ansi -Ieigen -Iglut-3.7.6-bin
LDFLAGS = -lGL -lGLU -lglut
all: main
main:  main.cpp IK_Logic.cpp GL_Render.cpp
	$(CC) $(CFLAGS) main.cpp IK_Logic.cpp GL_Render.cpp -o main $(LDFLAGS)


clean:
	rm -rf main