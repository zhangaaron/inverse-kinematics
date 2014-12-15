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
#include <math.h>

using namespace std;
using namespace Eigen;


#define PI 3.14159265
#define EPS  0.0420
enum JOINT_TYPES {
  BALL = 0
};

enum coords {
	X = 1,
	Y = 2,
	Z = 3
};

/*An arm segment consists of the joint that the segment swivels on, and the length of the arm. Orientations are specified using angle axis notation
with magnitude being the total angle being rotated in degrees.*/
class ArmSegment {
public: 
	ArmSegment(float arm_length, int joint_type);
	Vector3f get_joint_orientation();

	void set_joint_orientation(Vector3f orientation);
	float get_arm_length();
	int get_joint_type();

	void GL_Render_ArmSegment(); /*Object space representation of the arm, untransformed. */
private:
	int joint_type;
	float arm_length;
	Vector3f joint_orientation;
};

class Arm {
	public:		
		
		Arm(vector<float> arm_length_sequence); //by default uses ball joints. 
		Arm(Arm *toCopy); //Copies an existing arm sequence to create a new arm.

		float get_arm_length();
		vector<ArmSegment> get_arm_sequence();

		void GL_Render_Arm(); /*Render the arm in openGL when in callback function display*/
		void IK_Solve(Vector3f pos); /*Best-effort arrangement of joints to get arm close as possible to position pos.*/
		void rotate_arm(int seg, Vector3f orientation); /*Rotate a specific joint by orientation*/
		bool update(Vector3f goal_pos); /*Perform the update algorithm on the arm to move it to goal_pos*/
		Vector3f get_end_pos(); /*Computes the end position of the arm, from base at Vector3f(0,0,0)*/
		MatrixXf compute_Jacobian(); /*Constructs an array of 3x3N dP/dt partial derivatives for each joint with 3 DOF*/
		Vector3f dPdT(int joint, int axis);/*Computes dp/dtheta for a specific joint along a certain axis to construct Jacobian*/
	private:
		vector<ArmSegment> arm_sequence; 
		float arm_length;
		Vector3f sys_to_world;
};

