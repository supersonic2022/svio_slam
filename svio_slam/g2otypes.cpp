#include "g2otypes.h"

namespace g2o 
{
	//this error this between navstate and preintegration which in my implementation is the same
	//so the error should be modified ***
	void EdgeNavStatePVR::computeError()
	{
		const VertexNavStatePVR* vPVRi = static_cast<const VertexNavStatePVR*>(_vertices[0]);
		const VertexNavStatePVR* vPVRj = static_cast<const VertexNavStatePVR*>(_vertices[1]);
		const VertexNavStateBias* vBiasi = static_cast<const VertexNavStateBias*>(_vertices[2]);

		// terms need to computer error in vertex i, except for bias error
		const NavState& NSPVRi = vPVRi->estimate();
		Eigen::Vector3d Pi = NSPVRi.Get_P();
		Eigen::Vector3d Vi = NSPVRi.Get_V();
		Sophus::SO3d Ri = NSPVRi.Get_R();
		// Bias from the bias vertex
		const NavState& NSBiasi = vBiasi->estimate();
		Eigen::Vector3d dBgi = NSBiasi.Get_dBias_Gyr();
		Eigen::Vector3d dBai = NSBiasi.Get_dBias_Acc();

		// terms need to computer error in vertex j, except for bias error
		const NavState& NSPVRj = vPVRj->estimate();
		Eigen::Vector3d Pj = NSPVRj.Get_P();
		Eigen::Vector3d Vj = NSPVRj.Get_V();
		Sophus::SO3d Rj = NSPVRj.Get_R();

		// IMU Preintegration measurement
		const Preintegration& M = _measurement;
		double dTij = M._delta_time;   // Delta Time
		double dT2 =  dTij * dTij;
		Eigen::Vector3d dPij = M._delta_P;    // Delta Position pre-integration measurement
		Eigen::Vector3d dVij = M._delta_V;    // Delta Velocity pre-integration measurement
		Sophus::SO3d dRij = Sophus::SO3d(M._delta_R);  // Delta Rotation pre-integration measurement

		// tmp variable, transpose of Ri
		Sophus::SO3d RiT = Ri.inverse();
		// residual error of Delta Position measurement
		Eigen::Vector3d rPij = RiT * (Pj - Pi - Vi * dTij - 0.5 * GravityVec * dT2)
			- (dPij + M._J_P_Biasg * dBgi + M._J_P_Biasa * dBai);   // this line includes correction term of bias change.
		// residual error of Delta Velocity measurement
		Eigen::Vector3d rVij = RiT * (Vj - Vi - GravityVec * dTij)
			- (dVij + M._J_V_Biasg * dBgi + M._J_V_Biasa * dBai);   //this line includes correction term of bias change
		// residual error of Delta Rotation measurement
		Sophus::SO3d dR_dbg = Sophus::SO3d::exp(M._J_R_Biasg * dBgi);
		Sophus::SO3d rRij = (dRij * dR_dbg).inverse() * RiT * Rj;
		Eigen::Vector3d rPhiij = rRij.log();

		Vector9d err;  // typedef Matrix<double, D, 1> ErrorVector; ErrorVector _error; D=9
		err.setZero();

		// 9-Dim error vector order:
		// position-velocity-rotation
		// rPij - rVij - rPhiij
		err.segment<3>(0) = rPij;       // position error
		err.segment<3>(3) = rVij;       // velocity error
		err.segment<3>(6) = rPhiij;     // rotation phi error

		_error = err;

		//Test log
		if ((NSPVRi.Get_BiasGyr() - NSBiasi.Get_BiasGyr()).norm()>1e-6 || (NSPVRi.Get_BiasAcc() - NSBiasi.Get_BiasAcc()).norm()>1e-6)
		{
			std::cerr << "id pvri/pvrj/biasi: " << vPVRi->id() << "/" << vPVRj->id() << "/" << vBiasi->id() << std::endl;
			std::cerr << "bias gyr not equal for PVR/Bias vertex" << std::endl << NSPVRi.Get_BiasGyr().transpose() << " / " << NSBiasi.Get_BiasGyr().transpose() << std::endl;
			std::cerr << "bias acc not equal for PVR/Bias vertex" << std::endl << NSPVRi.Get_BiasAcc().transpose() << " / " << NSBiasi.Get_BiasAcc().transpose() << std::endl;
		}
	}

	void EdgeNavStatePVR::linearizeOplus()
	{
		const VertexNavStatePVR* vPVRi = static_cast<const VertexNavStatePVR*>(_vertices[0]);
		const VertexNavStatePVR* vPVRj = static_cast<const VertexNavStatePVR*>(_vertices[1]);
		const VertexNavStateBias* vBiasi = static_cast<const VertexNavStateBias*>(_vertices[2]);

		// terms need to computer error in vertex i, except for bias error
		const NavState& NSPVRi = vPVRi->estimate();
		Eigen::Vector3d Pi = NSPVRi.Get_P();
		Eigen::Vector3d Vi = NSPVRi.Get_V();
		Eigen::Matrix3d Ri = NSPVRi.Get_RotMatrix();
		// bias
		const NavState& NSBiasi = vBiasi->estimate();
		Eigen::Vector3d dBgi = NSBiasi.Get_dBias_Gyr();
		// Eigen::Vector3d dBai = NSBiasi.Get_dBias_Acc();

		// terms need to computer error in vertex j, except for bias error
		const NavState& NSPVRj = vPVRj->estimate();
		Eigen::Vector3d Pj = NSPVRj.Get_P();
		Eigen::Vector3d Vj = NSPVRj.Get_V();
		Eigen::Matrix3d Rj = NSPVRj.Get_RotMatrix();

		// IMU Preintegration measurement
		const Preintegration& M = _measurement;
		double dTij = M._delta_time;  // Delta Time
		double dT2 = dTij * dTij;

		// some temp variable
		Eigen::Matrix3d I3x3 = Eigen::Matrix3d::Identity();   // I_3x3
		Eigen::Matrix3d O3x3 = Eigen::Matrix3d::Zero();       // 0_3x3
		Eigen::Matrix3d RiT = Ri.transpose();          // Ri^T
		Eigen::Matrix3d RjT = Rj.transpose();          // Rj^T
		Eigen::Vector3d rPhiij = _error.segment<3>(6); // residual of rotation, rPhiij
		Eigen::Matrix3d JrInv_rPhi = Preintegration::JacobianRInv(rPhiij);    // inverse right jacobian of so3 term #rPhiij#
		Eigen::Matrix3d J_rPhi_dbg = M._J_R_Biasg;     // jacobian of preintegrated rotation-angle to gyro bias i

		// 1.
		// increment is the same as Forster 15'RSS
		// pi = pi + dpi,    pj = pj + dpj
		// vi = vi + dvi,       vj = vj + dvj
		// Ri = Ri*Exp(dphi_i), Rj = Rj*Exp(dphi_j)
		//      Note: the optimized bias term is the 'delta bias'
		// dBgi = dBgi + dbgi_update,    dBgj = dBgj + dbgj_update
		// dBai = dBai + dbai_update,    dBaj = dBaj + dbaj_update

		// 2.
		// 9-Dim error vector order in PVR:
		// position-velocity-rotation
		// rPij - rVij - rPhiij
		//      Jacobian row order:
		// J_rPij_xxx
		// J_rVij_xxx
		// J_rPhiij_xxx

		// 3.
		// order in 'update_' in PVR
		// Vertex_i : dPi, dVi, dPhi_i
		// Vertex_j : dPj, dVj, dPhi_j
		// 6-Dim error vector order in Bias:
		// dBiasg_i - dBiasa_i

		// 4.
		// For Vertex_PVR_i
		Eigen::Matrix<double, 9, 9> JPVRi;
		JPVRi.setZero();

		// 4.1
		// J_rPij_xxx_i for Vertex_PVR_i
		//JPVRi.block<3, 3>(0, 0) = -RiT;			//J_rP_dpi seemes like wrong
		JPVRi.block<3, 3>(0, 0) = -I3x3;
		JPVRi.block<3, 3>(0, 3) = -RiT * dTij;  //J_rP_dvi
		//JPVRi.block<3,3>(0,6) = SO3Calc::skew( RiT*(Pj-Pi-Vi*dTij-0.5*GravityVec*dT2)  );    //J_rP_dPhi_i
		JPVRi.block<3, 3>(0, 6) = Sophus::SO3d::hat(RiT * (Pj - Pi - Vi * dTij - 0.5 * GravityVec * dT2));    //J_rP_dPhi_i

		// 4.2
		// J_rVij_xxx_i for Vertex_PVR_i
		JPVRi.block<3, 3>(3, 0) = O3x3;    //dpi
		JPVRi.block<3, 3>(3, 3) = -RiT;    //dvi
		//JPVRi.block<3,3>(3,6) = SO3Calc::skew( RiT*(Vj-Vi-GravityVec*dTij) );    //dphi_i
		JPVRi.block<3, 3>(3, 6) = Sophus::SO3d::hat(RiT * (Vj - Vi - GravityVec * dTij));    //dphi_i

		// 4.3
		// J_rPhiij_xxx_i for Vertex_PVR_i
		//Matrix3d ExprPhiijTrans = SO3Calc::Expmap(rPhiij).transpose();  //Exp( rPhi )^T
		//Matrix3d JrBiasGCorr = SO3Calc::JacobianR(J_rPhi_dbg*dBgi);     //Jr( M.J_rPhi_bg * dBgi )
		Eigen::Matrix3d ExprPhiijTrans = Sophus::SO3d::exp(rPhiij).inverse().matrix();
		Eigen::Matrix3d JrBiasGCorr = Preintegration::JacobianR(J_rPhi_dbg * dBgi);
		JPVRi.block<3, 3>(6, 0) = O3x3;    //dpi
		JPVRi.block<3, 3>(6, 3) = O3x3;    //dvi
		JPVRi.block<3, 3>(6, 6) = -JrInv_rPhi * RjT * Ri;    //dphi_i


		// 5.
		// For Vertex_PVR_j
		Eigen::Matrix<double, 9, 9> JPVRj;
		JPVRj.setZero();

		// 5.1
		// J_rPij_xxx_j for Vertex_PVR_j
		JPVRj.block<3, 3>(0, 0) = RiT * Rj;	   //dpj
		JPVRj.block<3, 3>(0, 3) = O3x3;    //dvj
		JPVRj.block<3, 3>(0, 6) = O3x3;    //dphi_j

		// 5.2
		// J_rVij_xxx_j for Vertex_PVR_j
		JPVRj.block<3, 3>(3, 0) = O3x3;    //dpj
		JPVRj.block<3, 3>(3, 3) = RiT;     //dvj
		JPVRj.block<3, 3>(3, 6) = O3x3;    //dphi_j

		// 5.3
		// J_rPhiij_xxx_j for Vertex_PVR_j
		JPVRj.block<3, 3>(6, 0) = O3x3;    //dpj
		JPVRj.block<3, 3>(6, 3) = O3x3;    //dvj
		JPVRj.block<3, 3>(6, 6) = JrInv_rPhi;    //dphi_j


		// 6.
		// For Vertex_Bias_i
		Eigen::Matrix<double, 9, 6> JBiasi;
		JBiasi.setZero();

		// 5.1
		// J_rPij_xxx_j for Vertex_Bias_i
		JBiasi.block<3, 3>(0, 0) = -M._J_P_Biasg;     //J_rP_dbgi
		JBiasi.block<3, 3>(0, 3) = -M._J_P_Biasa;     //J_rP_dbai

		// J_rVij_xxx_j for Vertex_Bias_i
		JBiasi.block<3, 3>(3, 0) = -M._J_V_Biasg;    //dbg_i
		JBiasi.block<3, 3>(3, 3) = -M._J_V_Biasa;    //dba_i

		// J_rPhiij_xxx_j for Vertex_Bias_i
		JBiasi.block<3, 3>(6, 0) = -JrInv_rPhi * ExprPhiijTrans * JrBiasGCorr * J_rPhi_dbg;    //dbg_i
		JBiasi.block<3, 3>(6, 3) = O3x3;    //dba_i

		// Evaluate _jacobianOplus
		_jacobianOplus[0] = JPVRi;
		_jacobianOplus[1] = JPVRj;
		_jacobianOplus[2] = JBiasi;
	}

	void EdgeNavStateBias::computeError()
	{
		const VertexNavStateBias* vBiasi = static_cast<const VertexNavStateBias*>(_vertices[0]);
		const VertexNavStateBias* vBiasj = static_cast<const VertexNavStateBias*>(_vertices[1]);

		const NavState& NSi = vBiasi->estimate();
		const NavState& NSj = vBiasj->estimate();

		// residual error of Gyroscope's bias, Forster 15'RSS
		Eigen::Vector3d rBiasG = (NSj.Get_BiasGyr() + NSj.Get_dBias_Gyr())
			- (NSi.Get_BiasGyr() + NSi.Get_dBias_Gyr());

		// residual error of Accelerometer's bias, Forster 15'RSS
		Eigen::Vector3d rBiasA = (NSj.Get_BiasAcc() + NSj.Get_dBias_Acc())
			- (NSi.Get_BiasAcc() + NSi.Get_dBias_Acc());

		Vector6d err;  // typedef Matrix<double, D, 1> ErrorVector; ErrorVector _error; D=6
		err.setZero();
		// 6-Dim error vector order:
		// deltabiasGyr_i-deltabiasAcc_i
		// rBiasGi - rBiasAi
		err.segment<3>(0) = rBiasG;     // bias gyro error
		err.segment<3>(3) = rBiasA;    // bias acc error

		_error = err;
	}

	void EdgeNavStateBias::linearizeOplus()
	{
		// 6-Dim error vector order:
		// deltabiasGyr_i-deltabiasAcc_i
		// rBiasGi - rBiasAi

		_jacobianOplusXi = -Eigen::Matrix<double, 6, 6>::Identity();
		_jacobianOplusXj = -Eigen::Matrix<double, 6, 6>::Identity();

	}

	void EdgeNavStatePVRPointXYZ::linearizeOplus()
	{
		const VertexSBAPointXYZ* vPoint = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
		const VertexNavStatePVR* vNavState = static_cast<const VertexNavStatePVR*>(_vertices[1]);

		const NavState& ns = vNavState->estimate();
		Eigen::Matrix3d Rwb = ns.Get_RotMatrix();
		Eigen::Vector3d Pwb = ns.Get_P();
		const Eigen::Vector3d& Pw = vPoint->estimate();

		Eigen::Matrix3d Rcb = Rbc.transpose();
		Eigen::Vector3d Pc = Rcb * Rwb.transpose() * (Pw - Pwb) - Rcb * Pbc;

		double x = Pc[0];
		double y = Pc[1];
		double z = Pc[2];

		// Jacobian of camera projection
		Eigen::Matrix<double, 2, 3> Maux;
		Maux.setZero();
		Maux(0, 0) = fx;
		Maux(0, 1) = 0;
		Maux(0, 2) = -x / z*fx;
		Maux(1, 0) = 0;
		Maux(1, 1) = fy;
		Maux(1, 2) = -y / z*fy;
		Eigen::Matrix<double, 2, 3> Jpi = Maux / z;

		// error = obs - pi( Pc )
		// Pw <- Pw + dPw,          for Point3D
		// Rwb <- Rwb*exp(dtheta),  for NavState.R
		// Pwb <- Pwb + dPwb,   for NavState.P

		// Jacobian of error w.r.t Pw
		_jacobianOplusXi = -Jpi * Rcb * Rwb.transpose();

		// Jacobian of Pc/error w.r.t dPwb
		//Matrix3d J_Pc_dPwb = -Rcb;
		Eigen::Matrix<double, 2, 3> JdPwb = -Jpi * (-Rcb*Rwb.transpose());
		// Jacobian of Pc/error w.r.t dRwb
		Eigen::Vector3d Paux = Rcb*Rwb.transpose()*(Pw - Pwb);
		//Matrix3d J_Pc_dRwb = Sophus::SO3::hat(Paux) * Rcb;
		Eigen::Matrix<double, 2, 3> JdRwb = -Jpi * (Sophus::SO3d::hat(Paux) * Rcb);

		// Jacobian of Pc w.r.t NavState
		// order in 'update_': dP, dV, dPhi
		Eigen::Matrix<double, 2, 9> JNavState = Eigen::Matrix<double, 2, 9>::Zero();
		JNavState.block<2, 3>(0, 0) = JdPwb;
		//JNavState.block<2,3>(0,3) = 0;
		JNavState.block<2, 3>(0, 6) = JdRwb;
		//JNavState.block<2,3>(0,9) = 0;
		//JNavState.block<2,3>(0,12) = 0;

		// Jacobian of error w.r.t NavState
		_jacobianOplusXj = JNavState;
	}
}