#pragma once

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include "global.h"
#include "navstate.h"
#include "preintegration.h"
#include <g2o/core/base_multi_edge.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

namespace g2o
{
	class VertexNavStatePVR : public BaseVertex<9, NavState>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		VertexNavStatePVR() : BaseVertex<9, NavState>() {}

		bool read(std::istream& is) { return true; }
		bool write(std::ostream& os) const { return true; }

		virtual void setToOriginImpl()
		{
			_estimate = NavState();
		}

		virtual void oplusImpl(const double* update_)
		{
			Eigen::Map<const Vector9d> update(update_);
			_estimate.IncSmallPVR(update);
		}
	};

	class VertexNavStateBias : public BaseVertex<6, NavState>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		VertexNavStateBias() : BaseVertex<6, NavState>() {}

		bool read(std::istream& is) { return true; }
		bool write(std::ostream& os) const { return true; }

		virtual void setToOriginImpl()
		{
			_estimate = NavState();
		}

		virtual void oplusImpl(const double* update_)
		{
			Eigen::Map<const Vector6d> update(update_);
			_estimate.IncSmall(update);
		}
	};

	class EdgeNavStatePVR : public BaseMultiEdge<9, Preintegration>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		EdgeNavStatePVR() : BaseMultiEdge<9, Preintegration>() { resize(3); }
		
		bool read(std::istream& is) { return true; }
		bool write(std::ostream& os) const { return true; }

		void computeEror();

		virtual void linearizeOplus();

		void setParams(const Eigen::Vector3d& gw) { GravityVec = gw; }

	protected:
		Eigen::Vector3d GravityVec;
	};

	class EdgeNavStateBias : public BaseBinaryEdge<6, Preintegration, VertexNavStateBias, VertexNavStateBias>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		EdgeNavStateBias() : BaseBinaryEdge<6, Preintegration, VertexNavStateBias, VertexNavStateBias>() {}

		bool read(std::istream& is) { return true; }
		bool write(std::ostream& os) const { return true; }

		void computeError();

		virtual void linearizeOplus();
	};

	class EdgeNavStatePVRPointXYZ : public BaseBinaryEdge<2, Eigen::Vector2d, VertexSBAPointXYZ, VertexNavStatePVR>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		EdgeNavStatePVRPointXYZ() : BaseBinaryEdge<2, Eigen::Vector2d, VertexSBAPointXYZ, VertexNavStatePVR>() {}

		bool read(std::istream& is) { return true; }
		bool write(std::ostream& os) const { return true; }

		void computeError()
		{
			Eigen::Vector3d Pc = computePc();
			Eigen::Vector2d obs(_measurement);

			_error = obs - cam_project(Pc);
		}

		bool isDepthPositive()
		{
			Eigen::Vector3d Pc = computePc();
			return Pc(2) > 0.0;
		}

		Eigen::Vector3d computePc()
		{
			const VertexSBAPointXYZ* vPoint = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
			const VertexNavStatePVR* vNavState = static_cast<const VertexNavStatePVR*>(_vertices[1]);

			const NavState& ns = vNavState->estimate();
			Eigen::Matrix3d Rwb = ns.Get_RotMatrix();
			Eigen::Vector3d Pwb = ns.Get_P();
			const Eigen::Vector3d& Pw = vPoint->estimate();

			Eigen::Matrix3d Rcb = Rbc.transpose();
			Eigen::Vector3d Pc = Rcb * Rwb.transpose() * (Pw - Pwb) - Rcb * Pbc;

			return Pc;
		}

		inline Eigen::Vector2d project2d(const Eigen::Vector3d& v) const 
		{
			Eigen::Vector2d res;
			res(0) = v(0) / v(2);
			res(1) = v(1) / v(2);
			return res;
		}

		Eigen::Vector2d cam_project(const Eigen::Vector3d& trans_xyz) const 
		{
			Eigen::Vector2d proj = project2d(trans_xyz);
			Eigen::Vector2d res;
			res[0] = proj[0] * fx + cx;
			res[1] = proj[1] * fy + cy;
			return res;
		}

		virtual void linearizeOplus();

		void SetParams(const double& fx_, const double& fy_, const double& cx_, const double& cy_,
			const Eigen::Matrix3d& Rbc_, const Eigen::Vector3d& Pbc_)
		{
			fx = fx_;
			fy = fy_;
			cx = cx_;
			cy = cy_;
			Rbc = Rbc_;
			Pbc = Pbc_;
		}

	protected:
		double fx, fy, cx, cy;
		Eigen::Matrix3d Rbc;
		Eigen::Vector3d Pbc;
	};
}