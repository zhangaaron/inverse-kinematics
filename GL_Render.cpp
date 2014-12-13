/*
	Acknowledgements: Robert Bateman for Motion function and Mouse function. 

	References: GLProgramming.com, opengl.org forums for lighting, shading, keyboard guide.
*/

#include <iostream>
#include <fstream>
#include <cmath>
#include <stdlib.h>
#include "GL_Render.h"

using namespace std;
using namespace Eigen;

float zoom = 15.0f;
float rotx = 0;
float roty = 0.001f;
float tx = 0;
float ty = 0;
int lastx = 0;
int lasty = 0;
unsigned char Buttons[3] = {0};
Arm *GL_Arm;

void init() 
{
	glEnable(GL_DEPTH_TEST);

}

void display()
{
	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();

	glTranslatef(0,0,-zoom);
	glTranslatef(tx,ty,0);
	glRotatef(rotx,1,0,0);
	glRotatef(roty,0,1,0);
	glEnable(GL_NORMALIZE);

	/*Draw grid */
	glBegin(GL_LINES);
		for (int i = 0; i <= 4; i ++) {
			glVertex3f(i, -4, 0);
			glVertex3f(i, 4, 0);

			glVertex3f(-i, -4, 0);
			glVertex3f(-i, 4, 0);

			glVertex3f(-4, i, 0);
			glVertex3f(4, i, 0);

			glVertex3f(-4, -i, 0);
			glVertex3f(4, -i, 0);
		}
	glEnd();
	glMatrixMode(GL_MODELVIEW); //I don't think this call is necssary but it can't hurt.
	GL_Arm->GL_Render_Arm();
	glutSwapBuffers();
}

void reshape(int w, int h)
{
	// prevent divide by 0 error when minimised
	if(w == 0) h = 1;

	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45 ,(float)w/h, 0.1, 100);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void Motion(int x,int y)
{
	int diffx = x-lastx;
	int diffy = y-lasty;
	lastx=x;
	lasty=y;

	if( Buttons[0] && Buttons[1] )
	{
		zoom -= (float) 0.05f * diffx;
	}
	else
		if( Buttons[0] )
		{
			rotx += (float) 0.5f * diffy;
			roty += (float) 0.5f * diffx;
		}
		else
			if( Buttons[1] )
			{
				tx += (float) 0.05f * diffx;
				ty -= (float) 0.05f * diffy;
			}
			glutPostRedisplay();
}

void MyKeyboardFunc(unsigned char Key, int x, int y){
	switch(Key){
		case 'i':
			roty -= (float) 0.5f * 10;
			break;
		case 'j':
			rotx += (float) 0.5f * 10;
			break;
		case 'k':
			rotx -= (float) 0.5f * 10;
			break;
		case 'l':
			roty += (float) 0.5f * 10;
			break;
		case 45: // zoom out
			zoom += (float) 0.05f * 10;
			break;
		case 43: //zoom in
			zoom -= (float) 0.05f * 10;
			break;
		default:
			break;
	}
	glutPostRedisplay();
}

void Mouse(int b,int s,int x,int y)
{
	lastx=x;
	lasty=y;
	switch(b)
	{
	case GLUT_LEFT_BUTTON:
		Buttons[0] = ((GLUT_DOWN == s) ? 1:0);
		break;
	case GLUT_MIDDLE_BUTTON:
		Buttons[1] = ((GLUT_DOWN == s) ? 1:0);
		break;
	case GLUT_RIGHT_BUTTON:
		Buttons[2] = ((GLUT_DOWN == s) ? 1:0);
		break;
	default:
		break;
	}
	glutPostRedisplay();
}

void run_glut(Arm toRender, int *argcp, char **argv){
	glutInit(argcp,argv);
	glutInitDisplayMode(GLUT_DOUBLE|GLUT_RGBA|GLUT_DEPTH);
	glutInitWindowSize(640,480);
	glutInitWindowPosition(100,100);
	glutCreateWindow("IK");
	GL_Arm = &toRender;

	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
	glutMouseFunc(Mouse);
	glutKeyboardFunc(MyKeyboardFunc);
	glutMotionFunc(Motion);

	init();

	glutMainLoop();


}
