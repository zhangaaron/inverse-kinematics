#include <vector>
#include <iostream>
#include <fstream>
#include <cmath>
#include <stdlib.h>
#include <Eigen/Dense>
#include <GL/glut.h>
#include <GL/glu.h>
#include "IK_Logic.h"
#include "GL_Render.h"

using namespace Eigen;
using namespace std;

int main ( int argc, char **argv) {
	printf("Main starting");
	vector<float> basic_arm = vector<float>();
	basic_arm.push_back(4.0);
	basic_arm.push_back(2.0);
	basic_arm.push_back(1.0);
	run_glut(Arm(basic_arm), &argc, argv);
}