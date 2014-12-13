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
//IOFormat CommaInitFmt(StreamPrecision, DontAlignCols, ", ", ", ", "", "", " << ", ";");
int main ( int argc, char **argv) {
	vector<float> basic_arm = vector<float>();
	basic_arm.push_back(4.0);
	basic_arm.push_back(2.0);
	basic_arm.push_back(1.0);
	Arm test_arm = Arm(basic_arm); 
	test_arm.rotate_arm(0, Vector3f(45, 45, 45));
	test_arm.rotate_arm(1, Vector3f(45, 45, 45));
	test_arm.rotate_arm(2, Vector3f(45, 45, 45));
	//cout << "End arm pos : " << test_arm.get_end_pos() << "\n";
	run_glut(test_arm, &argc, argv);
}