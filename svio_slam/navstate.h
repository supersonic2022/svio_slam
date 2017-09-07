#ifndef NAVSTATE_H
#define NAVSTATE_H

#include "global.h"

typedef Eigen::Matrix<double, 15, 1> Vector15d;
typedef Eigen::Matrix<double, 9, 1> Vector9d; // P V R
typedef Eigen::Matrix<double, 6, 1> Vector6d; // bg ba

class NavState
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	NavState();
	NavState(const NavState& _ns);

	Sophus::SO3d Get_R() const { return _R; }
	Eigen::Matrix3d Get_RotMatrix() const { return _R.matrix(); } //rotation
	Eigen::Vector3d Get_P() const { return _P; }         // position
	Eigen::Vector3d Get_V() const { return _V; }         // velocity
	
	void Set_Pos(const Eigen::Vector3d &pos) { _P = pos; }
	void Set_Vel(const Eigen::Vector3d &vel) { _V = vel; }
	void Set_Rot(const Eigen::Matrix3d &rot) { _R = Sophus::SO3d(rot); }
	void Set_Rot(const Sophus::SO3d &rot) { _R = rot; }

	Eigen::Vector3d Get_BiasGyr() const { return _BiasGyr; }   // bias of gyroscope, keep unchanged after init and during optimization
	Eigen::Vector3d Get_BiasAcc() const { return _BiasAcc; }   // bias of accelerometer
	void Set_BiasGyr(const Eigen::Vector3d &bg) { _BiasGyr = bg; }
	void Set_BiasAcc(const Eigen::Vector3d &ba) { _BiasAcc = ba; }

	Eigen::Vector3d Get_dBias_Gyr() const { return _dBias_g; }  // delta bias of gyroscope, init as 0, change during optimization
	Eigen::Vector3d Get_dBias_Acc() const { return _dBias_a; }  // delta bias of accelerometer
	void Set_DeltaBiasGyr(const Eigen::Vector3d &dbg) { _dBias_g = dbg; }
	void Set_DeltaBiasAcc(const Eigen::Vector3d &dba) { _dBias_a = dba; }

	// incremental addition, delta = [dP, dV, dPhi, dBa, dBg]
	void IncSmall(Vector15d delta);
	void IncSmallPVR(Vector9d dPVR);
	void IncSmallPR(Vector6d dPR);
	void IncSmallV(Eigen::Vector3d dV);
	void IncSmallBias(Vector6d dBias);

private:
	Eigen::Vector3d _P;         // position
	Eigen::Vector3d _V;         // velocity
	Sophus::SO3d _R;			// rotation

	// keep unchanged during optimization
	Eigen::Vector3d _BiasGyr;   // bias of gyroscope
	Eigen::Vector3d _BiasAcc;   // bias of accelerometer

	// update below term during optimization
	Eigen::Vector3d _dBias_g;  // delta bias of gyroscope, correction term computed in optimization
	Eigen::Vector3d _dBias_a;  // delta bias of accelerometer
};


#endif // NAVSTATE_H