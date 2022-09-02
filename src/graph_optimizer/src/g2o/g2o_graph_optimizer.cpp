#include "graph_optimizer/g2o/g2o_graph_optimizer.h"

// 初始化稀疏求解器 and 鲁棒核
// input: 求解方式
G2oGraphOptimizer::G2oGraphOptimizer(const std::string &solver_type) {
    graph_ptr_.reset(new g2o::SparseOptimizer());

    g2o::OptimizationAlgorithmFactory *solver_factory = g2o::OptimizationAlgorithmFactory::instance();
    g2o::OptimizationAlgorithmProperty solver_property;
    g2o::OptimizationAlgorithm *solver = solver_factory->construct(solver_type, solver_property);
    graph_ptr_->setAlgorithm(solver);

    if (!graph_ptr_->solver()) {
        std::cout << "G2O Create Failed!" << std::endl;
    }
    robust_kernel_factory_ = g2o::RobustKernelFactory::instance();
}
// 设置鲁棒核(对边的限制)
void G2oGraphOptimizer::SetEdgeRobustKernel(std::string robust_kernel_name,
        double robust_kernel_size) { 
    robust_kernel_name_ = robust_kernel_name; 
    robust_kernel_size_ = robust_kernel_size; 
    need_robust_kernel_ = true; 
}
// 优化
bool G2oGraphOptimizer::Optimize() {
    static int optimize_cnt = 0;
    if(graph_ptr_->edges().size() < 1) {
        return false;
    }

    TicToc optimize_time;
    graph_ptr_->initializeOptimization();
    graph_ptr_->computeInitialGuess();
    graph_ptr_->computeActiveErrors();
    graph_ptr_->setVerbose(false);  // 打印调试输出信息

    double chi2 = graph_ptr_->chi2();   // 获得（加权）最小二乘误差等，这些可以用来判断某条边的误差是否过大,通过setLevel来决定是否优化,卡方分布
    int iterations = graph_ptr_->optimize(max_iterations_num_);     // 启动优化器

    std::cout << std::endl << "____already finish optimized times: " << ++optimize_cnt << std::endl
              << "vertices size: " << graph_ptr_->vertices().size() << ", edges_size: " << graph_ptr_->edges().size() << std::endl
              << "now iterations times: " << iterations << "/" << max_iterations_num_ << std::endl
              << "used time: " << optimize_time.toc() << std::endl
              << "error changed from: " << chi2 << "--->" << graph_ptr_->chi2()
              << std::endl << std::endl;

    return true;
}

// 获取优化后的顶点位姿
bool G2oGraphOptimizer::GetOptimizedPose(std::deque<Eigen::Matrix4f>& optimized_pose) {
    optimized_pose.clear();
    int vertex_num = graph_ptr_->vertices().size();

    for (int i = 0; i < vertex_num; i++) { 
        g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(graph_ptr_->vertex(i));
        Eigen::Isometry3d pose = v->estimate(); // 获取顶点的优化后的估计值
        optimized_pose.push_back(pose.matrix().cast<float>());
    }
    return true;
}
// 获取顶点数
int G2oGraphOptimizer::GetNodeNum() {
    return graph_ptr_->vertices().size();
}

// 添加顶点:是否固定,固定之后不进行优化,一般第一个顶点固定,但是
// 第一帧如果有观测就不固定
void G2oGraphOptimizer::AddSe3Node(const Eigen::Isometry3d &pose, bool need_fix) {
    g2o::VertexSE3* vertex(new g2o::VertexSE3());
    vertex->setId(graph_ptr_->vertices().size());
    vertex->setEstimate(pose);      // 设置估计值(预测值)
    if (need_fix) {
        vertex->setFixed(true);
    } 
    graph_ptr_->addVertex(vertex);  // 添加到优化器
} 

// 添加该边对应的两个顶点
// relative_pose: 两个顶点的关系
void G2oGraphOptimizer::AddSe3Edge(int vertex_index1,
                                      int vertex_index2,
                                      const Eigen::Isometry3d &relative_pose,
                                      const Eigen::VectorXd noise) {
    // 信息矩阵:协方差矩阵的逆(每条边的噪声)                                      
    Eigen::MatrixXd information_matrix = CalculateSe3EdgeInformationMatrix(noise);
    g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(graph_ptr_->vertex(vertex_index1));
    g2o::VertexSE3* v2 = dynamic_cast<g2o::VertexSE3*>(graph_ptr_->vertex(vertex_index2));

    g2o::EdgeSE3 *edge(new g2o::EdgeSE3());
    edge->setMeasurement(relative_pose);        // 测量值(观测值)
    edge->setInformation(information_matrix);   // 设置信息矩阵
    edge->vertices()[0] = v1;                // 添加该边对应的两个顶点 编号 
    edge->vertices()[1] = v2;
    graph_ptr_->addEdge(edge);               // 添加到优化器
    if (need_robust_kernel_) {
        AddRobustKernel(edge, robust_kernel_name_, robust_kernel_size_);
    }
}

Eigen::MatrixXd G2oGraphOptimizer::CalculateSe3EdgeInformationMatrix(Eigen::VectorXd noise) {
    Eigen::MatrixXd information_matrix = Eigen::MatrixXd::Identity(6, 6);
    information_matrix = CalculateDiagMatrix(noise);
    return information_matrix;
}

Eigen::MatrixXd G2oGraphOptimizer::CalculateDiagMatrix(Eigen::VectorXd noise) {
    Eigen::MatrixXd information_matrix = Eigen::MatrixXd::Identity(noise.rows(), noise.rows()); // 行 输入是列向量
    for (int i = 0; i < noise.rows(); i++) {
        information_matrix(i, i) /= noise(i);
    }
    return information_matrix;
}

// 添加鲁棒核
// cauchy核，huber核...
void G2oGraphOptimizer::AddRobustKernel(g2o::OptimizableGraph::Edge *edge, const std::string &kernel_type, double kernel_size) {
    if (kernel_type == "NONE") 
        return;

    g2o::RobustKernel *kernel = robust_kernel_factory_->construct(kernel_type);
    if (kernel == nullptr) {
        std::cerr << "warning : invalid robust kernel type: " << kernel_type << std::endl;
        return;
    }

    kernel->setDelta(kernel_size);
    edge->setRobustKernel(kernel);
}

// 位置观测边
// 边连接的1个顶点
void G2oGraphOptimizer::AddSe3PriorXYZEdge(int se3_vertex_index,
                                            const Eigen::Vector3d &xyz,
                                            Eigen::VectorXd noise) {
    Eigen::MatrixXd information_matrix = CalculateDiagMatrix(noise);
    g2o::VertexSE3 *v_se3 = dynamic_cast<g2o::VertexSE3 *>(graph_ptr_->vertex(se3_vertex_index));   // 获取该id对应的顶点
    
    g2o::EdgeSE3PriorXYZ *edge(new g2o::EdgeSE3PriorXYZ());
    edge->setMeasurement(xyz);
    edge->setInformation(information_matrix);
    edge->vertices()[0] = v_se3;
    graph_ptr_->addEdge(edge);
}

// 姿态观测
void G2oGraphOptimizer::AddSe3PriorQuaternionEdge(int se3_vertex_index,
                                        const Eigen::Quaterniond &quat,
                                        Eigen::VectorXd noise) {
    Eigen::MatrixXd information_matrix = CalculateSe3PriorQuaternionEdgeInformationMatrix(noise);
    g2o::VertexSE3 *v_se3 = dynamic_cast<g2o::VertexSE3 *>(graph_ptr_->vertex(se3_vertex_index));
    g2o::EdgeSE3PriorQuat *edge(new g2o::EdgeSE3PriorQuat());
    edge->setMeasurement(quat);
    edge->setInformation(information_matrix);
    edge->vertices()[0] = v_se3;
    graph_ptr_->addEdge(edge);
}

//TODO: 姿态观测的信息矩阵尚未添加
Eigen::MatrixXd G2oGraphOptimizer::CalculateSe3PriorQuaternionEdgeInformationMatrix(Eigen::VectorXd noise) {
    Eigen::MatrixXd information_matrix;
    return information_matrix;
}
