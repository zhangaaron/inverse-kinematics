#include "IK_Logic.h"

using namespace std;
using namespace Eigen;
/*Constructs an arm. Sequence of arms needs to be specified in order with base arm segment at index 0, and last arm at index size - 1*/
Arm::Arm(vector<float> sequence) {
	sys_to_world = Vector3f(0, 0, 0);
	arm_sequence = vector<ArmSegment>();
	for (int i = 0; i < sequence.size(); i++) {
		ArmSegment a = ArmSegment(sequence.at(i), BALL);
		arm_sequence.push_back(a);
	}
}
/*Call this inside display callback of GL to draw the arm as a sequence of cylinders*/
void Arm::GL_Render_Arm(){
	glPushMatrix();
	//glMultmatrixf(sys_to_world); //Default should be 0/

	for (int i = 0; i < arm_sequence.size(); i++ ) {
		//Do transformation
		ArmSegment seg = arm_sequence.at(i);
		Vector3f orientation = seg.get_joint_orientation();
		float mag = orientation.norm();
		glRotatef(mag, orientation(0)/mag, orientation(1)/mag, orientation(2)/mag);
		seg.GL_Render_ArmSegment();
		glTranslatef(0, 0, seg.get_arm_length());
	}
	glPopMatrix();
}


ArmSegment::ArmSegment(float arm_length, int joint_type) {
	this->arm_length = arm_length;
	this->joint_type = joint_type;
	this->joint_orientation = Vector3f(0, 0, 1); //Default no rotation about the z-axis
}
Vector3f ArmSegment::get_joint_orientation() {
	return joint_orientation;
}
void ArmSegment::set_joint_orientation(Vector3f orientation) {
	joint_orientation = orientation;
}
float ArmSegment::get_arm_length() {
	return arm_length;
}
int ArmSegment::get_joint_type() {
	return joint_type;
}

void ArmSegment::GL_Render_ArmSegment() {
	glutWireCone((float) arm_length / 2, arm_length, 20, 20);
}