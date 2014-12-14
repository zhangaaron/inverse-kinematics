/*For detailed function and class documentation see the header file. */

#include "IK_Logic.h"

using namespace std;
using namespace Eigen;

//for debug

IOFormat CommaInitFmt(StreamPrecision, DontAlignCols, ", ", ", ", "", "", " << ", ";");
/*Constructs an arm. Sequence of arms needs to be specified in order with base arm segment at index 0, and last arm at index size - 1*/
Arm::Arm(vector<float> sequence) {
	sys_to_world = Vector3f(0, 0, 0);
	arm_sequence = vector<ArmSegment>();
	for (int i = 0; i < sequence.size(); i++) {
		ArmSegment a = ArmSegment(sequence.at(i), BALL);
		arm_sequence.push_back(a);
	}
}
Arm::Arm(vector<ArmSegment> sequence) {
	arm_sequence = sequence;
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
void Arm::rotate_arm(int seg, Vector3f orientation) {
	arm_sequence.at(seg).set_joint_orientation(orientation);
}

Vector3f Arm::get_end_pos() {
	Vector3f start = sys_to_world;
	float x = sys_to_world(0);
	float y = sys_to_world(1);
	float z = sys_to_world(2);
	float sum_x_theta = 0;
	float sum_y_theta = 0;
	float sum_z_theta = 0;
	for (int i = 0; i < arm_sequence.size(); i ++) {
		Vector3f orientation = arm_sequence.at(i).get_joint_orientation();
		sum_x_theta += orientation(0);
		sum_y_theta += orientation(1);
		sum_z_theta += orientation(2);
		x += arm_sequence.at(i).get_arm_length() * cos(sum_x_theta * PI / 180);
		y += arm_sequence.at(i).get_arm_length() * cos(sum_y_theta * PI / 180);
		z += arm_sequence.at(i).get_arm_length() * cos(sum_z_theta * PI / 180);
	}
	return Vector3f(x, y, z);
}

MatrixXf Arm::compute_Jacobian() {
	/*X, Y, Z are three different orthogonal axises on which a ball joint can rotate. */
	printf("Hello world\n");
	MatrixXf Jacobian = MatrixXf(3, 3 * arm_sequence.size() ); 
	for (int i = 0; i < arm_sequence.size(); i++) {
		Vector3f dPdT_joint_i_X = dPdT(i, X);
		Vector3f dPdT_joint_i_Y = dPdT(i, Y);
		Vector3f dPdT_joint_i_Z = dPdT(i, Z);
		printf("We good here\n");
		Jacobian.col(i) << dPdT_joint_i_X;
		Jacobian.col(i + 1) << dPdT_joint_i_Y;
		Jacobian.col(i + 2) << dPdT_joint_i_Z;
	}
	return Jacobian;
}
Vector3f Arm::dPdT(int joint, int axis) {
	Arm jittered_arm = Arm(this->arm_sequence);
	cout << "new arm" << 	jittered_arm.arm_sequence.at(2).get_joint_orientation().format(CommaInitFmt) << "\n";
	Vector3f original_orientation = jittered_arm.arm_sequence.at(joint).get_joint_orientation();
	Vector3f new_orientation = Vector3f(0, 0, 0); //placeholder initialization.	
	switch (axis) { //Jitter by 1 degree for any axis
		case X:
			new_orientation = original_orientation + Vector3f(1, 0, 0);
			break;
		case Y:
			new_orientation = original_orientation + Vector3f(0, 1, 0);
			break;
		case Z:
			new_orientation = original_orientation + Vector3f(0, 0, 1);
			break;
		default:
			printf("Unexpected axis value in dPdT\n");
			exit(0);
	}
	// cout << "Old orientation : " << original_orientation.format(CommaInitFmt) << "\n" << "new orientation "
	// 		<< new_orientation.format(CommaInitFmt) << "\n";
	 jittered_arm.arm_sequence.at(joint).set_joint_orientation(new_orientation);
	// cout << "Old end pos" << jittered_arm.get_end_pos().format(CommaInitFmt) << "\n" << "new end_pos "
	// 		<< this->get_end_pos().format(CommaInitFmt) << "\n";

	return jittered_arm.get_end_pos() - this->get_end_pos(); 
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
	glutWireCone((float) arm_length / 4, arm_length, 20, 20);
}