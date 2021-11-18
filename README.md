# g2o_analysis
![g2o](https://raw.githubusercontent.com/zouyajing/g2o_analysis/main/v2-fb35b76967fb6a81bdb35b17aa334d26_r.jpeg 'g2o')

# 架构
g2o 的架构主要包含了三层：
1. 外层。用户自定义一个图结构(sparse optimizer)，向图中添加自定义的点和边。
2. 中间层。g2o实现的非线性优化算法，主要包括GN，LM，Dog-leg算法，负责每一步的迭代更新。
3. 内层。求解增量矩阵Hx=b，方法比如LDU，LDLT，LLT，QR，SVD，可调用Eigen或者其他库实现。

# 外层
外层的基类是HyperGraph，一个抽象类，类内声明了Vertex和Edge，同时声明了一些简单的函数，比如Vertex和Edge的set，get，insert，和remove。
OPtimizeGraph继承了HypergGraph，仍然是一个抽象类，类内声明了更具体的Vertex和Edge。Vertex的更新函数，Edge的计算误差的函数和计算雅可比的函数。
SparseOptimizer继承了OPtimizeGraph，是g2o的主要接口，它定义了三类函数：
1. 初始化函数。默认优化level为0的点和边。
2. 辅助函数。获得维度最大的节点和激活边。
3. 优化函数。判断迭代是否停止，发布输出和概率统计信息。

BaseVertex继承了OptimizableGraph中的Vertex类。模版参数为估计值（待优化变量）的维度D和类型T。
类中包含变量_hessian和_b来记录这个节点的海塞矩阵和b，节点当前的估计值则存储在_estimate中

BaseEdge继承了OptimizableGraph中的Edge类，是一个抽象类。BaseUnaryEdge, BaseBinaryEdge和BaseMultiEdge继承BaseEdge类。
BaseEdge类主要包含观测_measurement，信息矩阵_information和误差error_。主要函数有三种：
1. computeError。计算误差。
2. linearizeOplus。计算雅可比。
3. constructQuadraticForm。计算增量方程中对应的H和b。

另外，g2o声明了一个RobustKernel的虚基类，实现了常见的Huber，Cauchy等多个核函数。

# 中间层
OptimizationAlgorithm类也是一个虚基类，它定义了一个优化器的大部分接口，如init, computeMarginals和solve等，需要对应的派生类来进行实现。

OptimizationAlgorithmWithHessian就是这个对应的派生类，但由于没有实现solve这个最重要的虚函数，因此它仍然是一个虚基类。它最重要的功能有两点。一个是定义了一个线性求解器_solver，后续派生类中，具体的矩阵求解等就通过这个线性求解器来完成。第二个就是这个虚派生类实现了OptimizationAlgorighm中的一部分虚函数，如init, computeMarginals等。以init函数为例，算法会遍历当前的activeVertices，如果有需要进行边缘化的节点，并且对应的线性求解器也支持Schur补，则会进行边缘化，并调用_solver的初始化函数来进行初始化。g2o提供了三种优化算法：GN，LM和dog-leg，各是一个类，最主要的函数都是solve函数。

# 内层
最后就是线性求解器Solver类。Solver类是一个虚基类，实现了大部分所需要的函数用作交互，包括用于求解器初始化的init, buildStructure和buildSystem函数等；求解Ax=b的solve()函数；和一些辅助函数。

Solver的主要派生类是BlockSolver类。这是一个模板类，其模板参数是另一个模板类BlockSolverTraits。这是一个很常见的用法，BlockSolverTraits的两个模板参数分别是位姿的维度_poseDim和路标点的维度_landmarkDim。

1. buildStructure函数

中间层的优化算法调用线性求解器时，如果是第一次迭代，那么要调用buildStructure来构建Hx=b这样一个线性系统。

2. buildSystem函数

buildStructure只有在还没开始迭代的时候先调用一次。然后，每次迭代求解之前，都需要调用buildSystem函数来初始化。

3. solve函数

最后调用solve函数来求解。线性求解器LinearSolver是一个模板虚基类。它是solvers文件夹下各个线性求解器的interface. 它还有一个派生类LinearSolverCCS，主要是利用迭代的方式（如PCG）来对线性系统进行求解。这里就不详细介绍了。比较简单的求解器有LinearSolverDense，大概介绍下让大家对这个先行求解器有所了解。顾名思义，LinearSolverDense是用来求解一个稠密矩阵的，比如舒尔补之后的机器人位姿矩阵。的solve函数中，先准备好一个矩阵_H（传给_linearSolver的海塞矩阵H在solve函数的形参名称为A，可以认为是H的CRS存储形式（一种矩阵存储方式），所以这里面是在求Ax=b的解），这里假设传入的A矩阵是一个稀疏矩阵，并且按照CRS的方式存储。因此需要将A矩阵恢复成一个真正的ｍ＊ｎ大小的矩阵，存放在_H中。最后，调用Eigen中的Cholesky分解来求解Ax=b，并返回最后的结果。


