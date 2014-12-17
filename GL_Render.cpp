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
Vector3f goal;
bool UPDATE = true;

void init() 
{
	glEnable(GL_DEPTH_TEST);

}
/*Parametrically defined update function*/
Vector3f path_function(int time) {
	float x = 10 * cos((float) time / 100);
	float y = 10 * sin((float) time / 100);
	float z = 10 * cos((float) time /100);
	return Vector3f(x, y, z);
}

void draw_coords() {
	glColor3f(1, 0, 0);
	glVertex3f(0, 0, 0);
	glVertex3f(0, 0, 1);
	glColor3f(0, 1, 0);
	glVertex3f(0, 0, 0);
	glVertex3f(0, 1, 0);
	glColor3f(0, 0, 1);
	glVertex3f(0, 0, 0);
	glVertex3f(1, 0, 0);
}
void draw_grid() {
	for (int i = 0; i <= 20; i ++) {
		glVertex3f((float)i / 5 , -4, 0);
		glVertex3f((float)i / 5 , 4, 0);

		glVertex3f((float)-i / 5, -4, 0);
		glVertex3f((float)-i / 5, 4, 0);

		glVertex3f(-4, (float)i / 5 , 0);
		glVertex3f(4, (float)i / 5 , 0);

		glVertex3f(-4, (float)-i / 5, 0);
		glVertex3f(4, (float)-i / 5, 0);
	}
}
void timer_func(int time) {
	goal = path_function(time);
	UPDATE = true;
	glutTimerFunc(50, timer_func, time + 1);
	glutPostRedisplay();
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

	/*Draw coords */
	glBegin(GL_LINES);
		draw_coords();
	glEnd();

	GL_Arm->GL_Render_Arm();

	glColor3f(1, 0, 0);
	glPushMatrix();
	glTranslatef(goal(0), goal(1), goal(2));
	glutWireSphere(0.3, 10, 10);
	glPopMatrix();
	glColor3f(1, 1, 1);
	int i = 0;
	while(UPDATE && i < 30 && !GL_Arm->iterative_update(goal) ) {
		i++;
	}
	if (i == 30) printf("Warning: we updated 30 times and did not reach error threshold\n");
	UPDATE = false;
	GL_Arm->GL_Render_Arm();
	glColor3f(0, 1, 0);
	Vector3f actual = GL_Arm->get_end_pos();
	glPushMatrix();
	glTranslatef(actual(0), actual(1), actual(2));
	glutWireSphere(0.3, 10, 10);
	glPopMatrix();
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
	lastx = x;
	lasty = y;
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
	glutCreateWindow("IK v.4.20");
	GL_Arm = &toRender;
	glutTimerFunc(50, timer_func, 0);
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
	glutMouseFunc(Mouse);
	glutKeyboardFunc(MyKeyboardFunc);
	glutMotionFunc(Motion);

	init();
	glutMainLoop();


}
