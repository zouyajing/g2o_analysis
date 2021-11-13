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
最后就是线性求解器Solver类。Solver类是一个虚基类，实现了大部分所需要的函数用作交互，包括用于求解器初始化的init, buildStructure和buildSystem函数等；求解Ax=b的solve()函数；和一些辅助函数：_x和_b的get，图_optimizer的set和get，和levenberg法相关的设置lambda等函数。

Solver的主要派生类是BlockSolver类。这是一个模板类，其模板参数是另一个模板类BlockSolverTraits。这是一个很常见的用法，BlockSolverTraits的两个模板参数分别是位姿的维度_poseDim和路标点的维度_landmarkDim。

1. buildStructure函数

中间层的优化算法调用线性求解器时，如果是第一次迭代，那么要调用buildStructure来构建Hx=b这样一个线性系统。通过这个函数我们可以发现，g2o默认边缘化掉landmark，且它们的顺序（即id）在机器人位姿之后，所以前面必须初始化为PoseMatrixType。buildStructure通过遍历多次图来构建这个线性问题：

2. buildSystem函数

buildStructure只有在还没开始迭代的时候先调用一次。然后，每次迭代求解之前，都需要调用buildSystem函数来初始化。

因为通过buildStructure函数，整个系统的H矩阵的结构我们已经构建好了。现在，经过之前的迭代求解，观测量的值发生了变化（或者没经过求解，但也要把初始化的值添加到构建好的结构里），因此要对整个系统（的值）进行更新。

先遍历所有节点，对每个结点，清空之前的H和b（clearQuadraticForm函数），并且清空_Hpp矩阵。如果要进行舒尔补，还要清空_Hll和_Hpl矩阵。然后遍历所有边，重新进行线性化操作（lineaizeOplus函数）并构建H和b（constructQuadraticForm函数）。此时，大的H矩阵已经完成更新，还需要遍历所有节点来更新b向量（因为BaseVertex中，_hessian是Eigen::Map类型，而_b是Eigen::Matrix类型，因此虽然没有显式地更新系统的H矩阵，但在遍历边时，更新的H就已经更新在_Hpp等变量里面了）。


3. solve函数

最后，我们就可以调用solve函数来求解了。如果我们没有进行schur补操作，那么没啥说的，直接调用_linearSolver的solve函数来求解Hx=b就可以了（调用_linearSolver）。

如果我们要进行schur补操作，那就稍微有点复杂了，但也只是有点，整体也是按照舒尔补的公式来进行schur补进行求解。由于路标点对应的H矩阵显然是个稀疏的block diagonal matrix，因此我们遍历路标点，单独对其_Hll矩阵进行求逆（即单独求一个3*3矩阵的逆）。然后按照公式进行schur补相关矩阵的计算。这里先求出的机器人位姿，因此先计算其相关的矩阵（_Hschur, _bschur等）；然后调用_linearSolver，输入相关矩阵进行求解。如果机器人位姿求解成功了，就按照公式将之反代回schur补公式中，对路标点的位置进行求解。

4. 其他函数

要么没怎么用到，如updateStructure函数，这是外部传入一组节点和边，以此来构建大的H矩阵。这个函数不支持schur补，因此比buildStructure函数简单多了，主要就是完成buildStruct函数的那三个步骤。而且由于不进行舒尔补，所以步骤1是不需要的，就当所有节点都是机器人位置，然后挨个放进去就可以。
要么很简单。如setLambda和restoreDiagonal这一对函数，就是将H矩阵备份，然后在主对角元素上加上lambda或者从备份中恢复原来的H矩阵。init函数就是清空各个矩阵块，然后调用_linearSolver的init函数即可。
最后，线性求解器LinearSolver是一个模板虚基类。它是solvers文件夹下各个线性求解器的interface. 它还有一个派生类LinearSolverCCS，主要是利用迭代的方式（如PCG）来对线性系统进行求解。

这里就不详细介绍了。比较简单的求解器有LinearSolverDense，大概介绍下让大家对这个先行求解器有所了解。

顾名思义，LinearSolverDense是用来求解一个稠密矩阵的，比如舒尔补之后的机器人位姿矩阵。的solve函数中，先准备好一个矩阵_H（传给_linearSolver的海塞矩阵H在solve函数的形参名称为A，可以认为是H的CRS存储形式（一种矩阵存储方式），所以这里面是在求Ax=b的解），这里假设传入的A矩阵是一个稀疏矩阵，并且按照CRS的方式存储。因此需要将A矩阵恢复成一个真正的ｍ＊ｎ大小的矩阵，存放在_H中。最后，调用Eigen中的Cholesky分解来求解Ax=b，并返回最后的结果。


