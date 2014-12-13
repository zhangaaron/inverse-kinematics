/*
	Given set of polygons, we generate a scene. 
*/
#pragma once

#include <vector>
#include <iostream>
#include <fstream>
#include <cmath>
#include <stdlib.h>
#include <Eigen/Dense>
#include <GL/glut.h>
#include <GL/glu.h>
#include "IK_Logic.h"


using namespace std;
using namespace Eigen;

void display();
void run_glut(Arm myArm, int *argcp, char **argv);