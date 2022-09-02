// GNSS 坐标做观测时使用的先验边

#ifndef __GRAPH_OPTIMIZER_G2O_EDGE_SE3_PRIORXYZ_H
#define __GRAPH_OPTIMIZER_G2O_EDGE_SE3_PRIORXYZ_H

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>

// using namespace g2o;

namespace g2o {
	// 继承一元边: [观测值3维,类型Vector3d,连接的顶点类型VertexSE3]
class EdgeSE3PriorXYZ : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3> {
  public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	EdgeSE3PriorXYZ()
    	: g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3>() {
	}

	void computeError() override {
		// 这个类型的边是单边,因此只需要计算_vertices0
		const g2o::VertexSE3* v1 = static_cast<const g2o::VertexSE3*>(_vertices[0]);

		Eigen::Vector3d estimate = v1->estimate().translation(); // translation(): ???????
		_error = estimate - _measurement;		// _measurement: 链接的顶点的预测
	}

    void setMeasurement(const Eigen::Vector3d& m) override {		// 重写顶点观测值函数, 在AddSe3PriorXYZEdge调用
		_measurement = m;
	}

	virtual bool read(std::istream& is) override {		// 输入流 都在该类的对象调用
    	Eigen::Vector3d v;
		is >> v(0) >> v(1) >> v(2);
    	setMeasurement(Eigen::Vector3d(v));

		for (int i = 0; i < information().rows(); ++i)		//information()函数,返回信息矩阵_information 
			for (int j = i; j < information().cols(); ++j) {
				is >> information()(i, j);
				if (i != j)
					information()(j, i) = information()(i, j);
			}
			return true;
	}

	virtual bool write(std::ostream& os) const override {		// 输出流 打印
    	Eigen::Vector3d v = _measurement;
		os << v(0) << " " << v(1) << " " << v(2) << " ";
		for (int i = 0; i < information().rows(); ++i)
			for (int j = i; j < information().cols(); ++j)
				os << " " << information()(i, j);
		return os.good();
	}
};
}

#endif
