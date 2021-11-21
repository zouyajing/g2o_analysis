# g2o_analysis
![g2o](https://raw.githubusercontent.com/zouyajing/g2o_analysis/main/v2-fb35b76967fb6a81bdb35b17aa334d26_r.jpeg 'g2o')

## 架构
g2o 的架构主要包含了三层：
1. 外层。用户自定义一个图结构(sparse optimizer)，向图中添加自定义的点和边。
2. 中间层。g2o实现的非线性优化算法，主要包括GN，LM，Dog-leg算法，负责每一步的迭代更新。
3. 内层。求解增量矩阵Hx=b，方法比如LDU，LDLT，LLT，QR，SVD，可调用Eigen或者其他库实现。
