#include "IK_Logic.h"
#include "GL_Render.h"


using namespace Eigen;
using namespace std;
//IOFormat CommaInitFmt(StreamPrecision, DontAlignCols, ", ", ", ", "", "", " << ", ";");
int main ( int argc, char **argv) {
	vector<float> basic_arm = vector<float>();

	basic_arm.push_back(1.0);
	basic_arm.push_back(2.0);
	basic_arm.push_back(4.0);
	basic_arm.push_back(5.0);

	Arm test_arm = Arm(basic_arm); 
	test_arm.rotate_arm(0, Vector3f(0, 0, 0));
	test_arm.rotate_arm(1, Vector3f(0, 0, 0));
	test_arm.rotate_arm(2, Vector3f(0, 0, 0));
	// int i = 0;
	// while(!test_arm.iterative_update(goal)) {
	// 	i++;
	// 	assert (i < 10);
	// }
	Matrix3f m;
	// m = AngleAxisf(0.25*M_PI, Vector3f::UnitX())
	//   * AngleAxisf(0.5*M_PI,  Vector3f::UnitY())
	//   * AngleAxisf(0.33*M_PI, Vector3f::UnitZ());
	// Vector3f x = Vector3f(1, 1, 1);
	// cout << m * x << endl;
	run_glut(test_arm, &argc, argv);
}