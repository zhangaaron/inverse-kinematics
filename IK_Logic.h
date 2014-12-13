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

using namespace std;
using namespace Eigen;
enum JOINT_TYPES {
  BALL = 0
};
class ArmSegment {
public: 
	ArmSegment(float arm_length, int joint_type);
	Vector3f get_joint_orientation();
	void set_joint_orientation(Vector3f orientation);
	float get_arm_length();
	int get_joint_type();
	void GL_Render_ArmSegment();
private:
	int joint_type;
	float arm_length;
	Vector3f joint_orientation;
};

class Arm {
	public:

		Arm(vector<float> arm_length_sequence); //by default uses ball joints. 
		void GL_Render_Arm();
		void IK_Solve(Vector3f pos);
	private: 
		vector<ArmSegment> arm_sequence;
		Vector3f sys_to_world;
};

