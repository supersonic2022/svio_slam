#include "globaloptimize.h"
#include <g2o/core/block_solver.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include "imudata.h"
#include "point.h"
#include "feature.h"


GlobalOptimize::GlobalOptimize(svo::Map* pMap_, EuRoCData* param) : pMap(pMap_), param_(param)
{
	nIterations = 10;
	bRobust = true;
}

void GlobalOptimize::run()
{
	while (1)
	{
		std::this_thread::sleep_for(2s);
		get_kfs();		
		if (tmp_kfs.size() < 5)
			continue;
		GlobalBundleAdjustmentNavState();
	}
}

void GlobalOptimize::get_kfs()
{
	if (!tmp_kfs.empty())
		tmp_kfs.clear();
	tmp_kfs = pMap->keyframes_;
}

void GlobalOptimize::GlobalBundleAdjustmentNavState()
{
	// Extrinsics
	Eigen::Matrix4d Tbc = param_->cam_params[0].T_BS;
	Eigen::Matrix3d Rbc = Tbc.topLeftCorner(3, 3);
	Eigen::Vector3d Pbc = Tbc.topRightCorner(3, 1);
	// Gravity vector in world frame
	Eigen::Vector3d GravityVec = param_->gravity;

	std::vector<bool> vbNotIncludedMP;
	vbNotIncludedMP.resize(tmp_kfs.size());

	g2o::SparseOptimizer optimizer;
	g2o::BlockSolverX::LinearSolverType * linearSolver;

	linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
	g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);
	g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
	optimizer.setAlgorithm(solver);

	long unsigned int maxKFid = 0;

	// Set KeyFrame vertices
	for (auto kf_it : tmp_kfs)
	{
		svo::FramePtr pKF = kf_it;

		// PVR
		g2o::VertexNavStatePVR * vNSPVR = new g2o::VertexNavStatePVR();
		vNSPVR->setEstimate(pKF->imuState);
		vNSPVR->setId(pKF->kf_id_ * 2);
		vNSPVR->setFixed(pKF->kf_id_ == 0);
		optimizer.addVertex(vNSPVR);
		// Bias
		g2o::VertexNavStateBias * vNSBias = new g2o::VertexNavStateBias();
		vNSBias->setEstimate(pKF->imuState);
		vNSBias->setId(pKF->kf_id_ * 2 + 1);
		vNSBias->setFixed(pKF->kf_id_ == 0);
		optimizer.addVertex(vNSBias);

		if (pKF->kf_id_ * 2 + 1 > maxKFid)
			maxKFid = pKF->kf_id_ * 2 + 1;
	}

	// Add NavState PVR/Bias edges
	const float thHuberNavStatePVR = sqrt(21.666);
	const float thHuberNavStateBias = sqrt(16.812);
	// Inverse covariance of bias random walk
	Eigen::Matrix<double, 6, 6> InvCovBgaRW = Eigen::Matrix<double, 6, 6>::Identity();
	InvCovBgaRW.topLeftCorner(3, 3) = Eigen::Matrix3d::Identity() / IMUData::getGyrBiasRW2();       // Gyroscope bias random walk, covariance INVERSE
	InvCovBgaRW.bottomRightCorner(3, 3) = Eigen::Matrix3d::Identity() / IMUData::getAccBiasRW2();   // Accelerometer bias random walk, covariance INVERSE

	bool firstKF = true;
	svo::FramePtr pKF0 = NULL;
	for (auto kf_it : tmp_kfs)
	{

		if (firstKF)
		{
			pKF0 = kf_it;
			firstKF = false;
			continue;
		}
		svo::FramePtr pKF1 = kf_it;

		//if (!pKF0)
		//{
		//	if (pKF1->kf_id_ != 0) std::cerr << "Previous KeyFrame is NULL?" << std::endl;  //Test log
		//	continue;
		//}

		// PVR edge
		{
			g2o::EdgeNavStatePVR * epvr = new g2o::EdgeNavStatePVR();
			epvr->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(2 * pKF0->kf_id_)));
			epvr->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(2 * pKF1->kf_id_)));
			epvr->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(2 * pKF0->kf_id_ + 1)));
			epvr->setMeasurement(pKF1->getIMUPreInt());

			Eigen::Matrix<double, 9, 9> InvCovPVR = pKF1->getIMUPreInt()._cov_P_V_Phi.inverse();
			epvr->setInformation(InvCovPVR);
			epvr->setParams(GravityVec);

			if (bRobust)
			{
				g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
				epvr->setRobustKernel(rk);
				rk->setDelta(thHuberNavStatePVR);
			}

			optimizer.addEdge(epvr);
		}
		// Bias edge
		{
			g2o::EdgeNavStateBias * ebias = new g2o::EdgeNavStateBias();
			ebias->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(2 * pKF0->kf_id_ + 1)));
			ebias->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(2 * pKF1->kf_id_ + 1)));
			ebias->setMeasurement(pKF1->getIMUPreInt());

			ebias->setInformation(InvCovBgaRW / pKF1->getIMUPreInt()._delta_time);

			if (bRobust)
			{
				g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
				ebias->setRobustKernel(rk);
				rk->setDelta(thHuberNavStateBias);
			}

			optimizer.addEdge(ebias);

		}

		pKF0 = pKF1;
	}

	const float thHuber2D = sqrt(5.99);

	// Set MapPoint vertices
	for (auto kf_it : tmp_kfs)
		for (auto feat_it : kf_it->fts_)
		{
			svo::Point* pMP = feat_it->point;
			//if (pMP->isBad())
			//	continue;
			g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
			vPoint->setEstimate(pMP->pos_);
			const int id = pMP->id_ + maxKFid + 1;
			vPoint->setId(id);
			vPoint->setMarginalized(true);
			optimizer.addVertex(vPoint);

			auto observations = pMP->obs_;

			int nEdges = 0;
			//SET EDGES
			for (auto mit : observations)
			{
				svo::Frame* pKF = mit->frame;
				if (!pKF->isKeyframe())
					continue;
				if (2 * pKF->kf_id_ > maxKFid)
					continue;
				nEdges++;

				//may cause problems
				Eigen::Matrix<double, 2, 1> obs = mit->px;

				g2o::EdgeNavStatePVRPointXYZ* e = new g2o::EdgeNavStatePVRPointXYZ();

				e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
				e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(2 * pKF->kf_id_)));
				e->setMeasurement(obs);
				const float &weight = 1.0 / (1 << mit->level);
				e->setInformation(Eigen::Matrix2d::Identity()*weight);

				if (bRobust)
				{
					g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
					e->setRobustKernel(rk);
					rk->setDelta(thHuber2D);
				}

				e->SetParams(param_->cam_params[0].intrinsics[0],
						param_->cam_params[0].intrinsics[1],
						param_->cam_params[0].intrinsics[2], 
						param_->cam_params[0].intrinsics[3], 
						Rbc, Pbc);

				optimizer.addEdge(e);

			}
		}

	// Optimize!
	optimizer.initializeOptimization();
	optimizer.optimize(nIterations);

	// Recover optimized data

	//Keyframes
	for (auto kf_it : tmp_kfs)
	{
		svo::FramePtr pKF = kf_it;

		//g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->kf_id_));
		//g2o::SE3Quat SE3quat = vSE3->estimate();
		g2o::VertexNavStatePVR* vNSPVR = static_cast<g2o::VertexNavStatePVR*>(optimizer.vertex(2 * pKF->kf_id_));
		g2o::VertexNavStateBias* vNSBias = static_cast<g2o::VertexNavStateBias*>(optimizer.vertex(2 * pKF->kf_id_ + 1));
		const NavState& nspvr = vNSPVR->estimate();
		const NavState& nsbias = vNSBias->estimate();
		NavState ns_recov = nspvr;
		ns_recov.Set_DeltaBiasGyr(nsbias.Get_dBias_Gyr());
		ns_recov.Set_DeltaBiasAcc(nsbias.Get_dBias_Acc());

		pKF->setNavState(ns_recov);
		pKF->UpdatePoseFromNS(param_->cam_params[0].T_BS);

		//Points
		for (auto kf_it : tmp_kfs)
			for (auto feat_it : kf_it->fts_)
			{
				svo::Point* pMP = feat_it->point;

				g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->id_ + maxKFid + 1));

				pMP->pos_ = vPoint->estimate();
				//pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
				//pMP->UpdateNormalAndDepth();

			}
	}
}