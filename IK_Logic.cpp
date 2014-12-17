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
	arm_length = 0;
	for (int i = 0; i < sequence.size(); i++) {
		ArmSegment a = ArmSegment(sequence.at(i), BALL);
		arm_sequence.push_back(a);
		arm_length += a.get_arm_length();
	}
}
Arm::Arm(Arm *toCopy) {
	sys_to_world = Vector3f(0, 0, 0);
	arm_sequence = toCopy->get_arm_sequence();
	arm_length = toCopy->get_arm_length();
}

float Arm::get_arm_length() {
	return arm_length;
}

vector<ArmSegment> Arm::get_arm_sequence() {
	return arm_sequence;
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
		if (mag) glRotatef(mag, orientation(0)/mag, orientation(1)/mag, orientation(2)/mag);
		seg.GL_Render_ArmSegment();
		glTranslatef(0, 0, seg.get_arm_length());
	}
	glPopMatrix();
}
void Arm::rotate_arm(int seg, Vector3f orientation) {
	arm_sequence.at(seg).set_joint_orientation(orientation);
}

void Arm::set_orientations(vector<Vector3f> vals) {
	for (int i = 0; i < arm_sequence.size(); i++) {
		arm_sequence.at(i).set_joint_orientation(vals.at(i));
	}
}

vector<Vector3f> Arm::get_orientations() {
	vector<Vector3f> to_rtn = vector<Vector3f>();
	for (int i = 0; i < arm_sequence.size(); i ++) {
		to_rtn.push_back(arm_sequence.at(i).get_joint_orientation());
	}
}

/*Returns when we found a configuration of the arm closer to goal. Doesn't mean we have reached an acceptable threshold though for the goal. */
bool Arm::iterative_update(Vector3f goal_pos) {
	goal_pos = goal_pos - sys_to_world;
	float arm_len = get_arm_length();
	if (goal_pos.norm() > arm_len) {
		goal_pos.normalize();
		goal_pos *= arm_len;
		printf("Goal we got was out of reach, so we normalized to arm length\n");
	}
	float t = 1.0;
	/*goal is linearly interpolated from current position to starting position. We start with t = 1, so that iter_goal = */
	Vector3f iter_goal = goal_pos * t + get_end_pos() * (1.0 - t);
	Vector3f dP = goal_pos - get_end_pos();
	cout << "dP is " << dP << endl;
	vector<Vector3f> orientations = get_orientations();
	while (!linear_update(iter_goal)) {
		t *= 0.5; //halve t by two so it's closer to the goal. 
		assert (t > 0.01); //if t gets this small indicates we are doing something wrong
		iter_goal = goal_pos * t + get_end_pos() * (1.0 - t);
		set_orientations(orientations); //reset orientations to where we were before since we didn't improve. 
	}
	return ((goal_pos - get_end_pos()).norm() < EPS);
}
/*Returns true if distance decreased otherwise return false*/
bool Arm::linear_update(Vector3f goal_pos) {
	Vector3f dP = goal_pos - get_end_pos();
	cout << "dP is " << dP << endl;
	MatrixXf Jacobian = compute_Jacobian();
	JacobiSVD<MatrixXf> svd(Jacobian, ComputeThinU | ComputeThinV); 
	cout << "singular values are:" << endl << svd.singularValues() << endl;
	MatrixXf dT = svd.solve(dP);
	 cout << "solved for dT = " << dT << endl;
	for (int i = 0; i < get_arm_sequence().size(); i++) {
		rotate_arm(i, Vector3f(dT(3 * i), dT(3 * i + 1), dT(3 * i + 2)));
	}
	return (dP.norm() > (goal_pos - get_end_pos()).norm() ); 
}

/*We gon need a transformation stack to keep track of our transformation*/
Vector3f Arm::get_end_pos() {
	Vector3f end_effector = sys_to_world;
	AngleAxisf transformation_stack = AngleAxisf(0, Vector3f(0, 0, 0));

	for (int i = 0; i < arm_sequence.size(); i ++) {
		Vector3f orientation = arm_sequence.at(i).get_joint_orientation();
		float norm = orientation.norm();
		orientation.normalized();
		transformation_stack = AngleAxisf(norm, orientation)*  transformation_stack;
		end_effector += transformation_stack * Vector3f(0, 0, arm_sequence.at(i).get_arm_length());
	}
	return end_effector;
}

MatrixXf Arm::compute_Jacobian() {
	/*X, Y, Z are three different orthogonal axises on which a ball joint can rotate. */
	MatrixXf Jacobian = MatrixXf(3, 3 * arm_sequence.size() ); 
	for (int i = 0; i < arm_sequence.size(); i++) {
		Vector3f dPdT_joint_i_X = dPdT(i, X);
		Vector3f dPdT_joint_i_Y = dPdT(i, Y);
		Vector3f dPdT_joint_i_Z = dPdT(i, Z);
		Jacobian.col(3 * i) << dPdT_joint_i_X;
		Jacobian.col(3 * i + 1) << dPdT_joint_i_Y;
		Jacobian.col(3 * i + 2) << dPdT_joint_i_Z;
	}
	cout << "Jacobian \n" << Jacobian << endl;
	return Jacobian;
}
Vector3f Arm::dPdT(int joint, int axis) {
	Arm jittered_arm = Arm(this);
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
	 jittered_arm.arm_sequence.at(joint).set_joint_orientation(new_orientation);
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
	glutWireSphere((float) arm_length /4, 15, 15);
	glutWireCone((float) arm_length / 4, arm_length, 20, 20);
}