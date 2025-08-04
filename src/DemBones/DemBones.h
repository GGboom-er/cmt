
///////////////////////////////////////////////////////////////////////////////
//               Dem Bones - Skinning Decomposition Library                  //
//         Copyright (c) 2019, Electronic Arts. All rights reserved.         //
///////////////////////////////////////////////////////////////////////////////
//               Dem Bones - 蒙皮分解库                                      //
//         版权所有 (c) 2019, Electronic Arts。保留所有权利。                  //
///////////////////////////////////////////////////////////////////////////////


#ifndef DEM_BONES_DEM_BONES
#define DEM_BONES_DEM_BONES

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/StdVector>
#include <algorithm>
#include <queue>
#include "ConvexLS.h"

#ifndef DEM_BONES_MAT_BLOCKS
#include "MatBlocks.h"
#define DEM_BONES_DEM_BONES_MAT_BLOCKS_UNDEFINED
#endif

namespace Dem
{

	/** @mainpage 概述
		主要元素:
		- @ref DemBones : 基类，包含使用相对骨骼变换 DemBones::m 的核心求解器。
		- @ref DemBonesExt : 扩展类，用于处理具有局部旋转/平移和绑定矩阵的层次化骨骼。
		- DemBones/MatBlocks.h: 宏定义，方便访问打包的变换/位置矩阵的子块。

		包含 DemBones/DemBonesExt.h (或 DemBones/DemBones.h)，并可选择性包含 DemBones/MatBlocks.h，然后按照以下步骤使用该库：
		-# 加载基类中的必需数据：
			- 静止姿态形状： DemBones::u, DemBones::fv, DemBones::nV
			- 动画序列： DemBones::v, DemBones::nF, DemBones::fStart, DemBones::subjectID, DemBones::nS
			- 骨骼数量： DemBones::nB
		-# 加载基类中的可选数据：
			- 蒙皮权重： DemBones::w
			- 骨骼变换： DemBones::m
		-# [可选] 在基类中设置参数：
			- DemBones::nIters
			- DemBones::nInitIters
			- DemBones::nTransIters, DemBones::transAffine, DemBones::transAffineNorm
			- DemBones::nWeightsIters, DemBones::nnz, DemBones::weightsSmooth, DemBones::weightsSmoothStep, DemBones::weightEps
		-# [可选] 设置扩展类：
			- 加载数据： DemBonesExt::parent, DemBonesExt::preMulInv, DemBonesExt::rotOrder, DemBonesExt::bind
			- 设置参数： DemBonesExt::bindUpdate
		-# [可选] 在基类 @ref DemBones 中重写回调函数 (cb...)
		-# 调用分解函数 DemBones::compute(), DemBones::computeWeights(), DemBones::computeTranformations(), 或 DemBones::init()
		-# [可选] 使用 DemBonesExt::computeRTB() 获取局部变换/绑定姿态
	*/

	/** @class DemBones DemBones.h "DemBones/DemBones.h"
		@brief 具有刚性骨骼和稀疏、凸权重的平滑蒙皮分解
		@brief Smooth skinning decomposition with rigid bones and sparse, convex weights

		@details 设置所需的数据、参数，并调用 compute(), computeWeights(), computeTranformations(), 或 init()。
		@details Setup the required data, parameters, and call either compute(), computeWeights(), computeTranformations(), or init().

		回调函数和只读值可用于报告进度：cbInitSplitBegin(), cbInitSplitEnd(),
		cbIterBegin(), cbIterEnd(), cbWeightsBegin(), cbWeightsEnd(), cbTranformationsBegin(), cbTransformationsEnd(), cbTransformationsIterBegin(),
		cbTransformationsIterEnd(), cbWeightsIterBegin(), cbWeightsIterEnd(), rmse(), #iter, #iterTransformations, #iterWeights.

		@b _Scalar 是浮点数据类型。@b _AniMeshScalar 是网格序列 #v 的浮点数据类型。
		@b _Scalar is the floating-point data type. @b _AniMeshScalar is the floating-point data type of mesh sequence #v.
	*/
	template<class _Scalar, class _AniMeshScalar>
	class DemBones {
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

			// 类型定义别名
			using MatrixX = Eigen::Matrix<_Scalar, Eigen::Dynamic, Eigen::Dynamic>;
		using Matrix4 = Eigen::Matrix<_Scalar, 4, 4>;
		using Matrix3 = Eigen::Matrix<_Scalar, 3, 3>;
		using VectorX = Eigen::Matrix<_Scalar, Eigen::Dynamic, 1>;
		using Vector4 = Eigen::Matrix<_Scalar, 4, 1>;
		using Vector3 = Eigen::Matrix<_Scalar, 3, 1>;
		using SparseMatrix = Eigen::SparseMatrix<_Scalar>;
		using Triplet = Eigen::Triplet<_Scalar>;

		//! [参数] 全局迭代次数, 默认 = 30
		//! [@c parameter] Number of global iterations, @c default = 30
		int nIters;

		//! [参数] 初始化中聚类更新的迭代次数, 默认 = 10
		//! [@c parameter] Number of clustering update iterations in the initalization, @c default = 10
		int nInitIters;

		//! [参数] 每次全局迭代中骨骼变换更新的迭代次数, 默认 = 5
		//! [@c parameter] Number of bone transformations update iterations per global iteration, @c default = 5
		int nTransIters;
		//! [参数] 平移亲和度软约束, 默认 = 10.0
		//! [@c parameter] Translations affinity soft constraint, @c default = 10.0
		_Scalar transAffine;
		//! [参数] 骨骼平移亲和度软约束的p-范数, 默认 = 4.0
		//! [@c parameter] p-norm for bone translations affinity soft constraint, @c default = 4.0
		_Scalar transAffineNorm;

		//! [参数] 每次全局迭代中权重更新的迭代次数, 默认 = 3
		//! [@c parameter] Number of weights update iterations per global iteration, @c default = 3
		int nWeightsIters;
		//! [参数] 每个顶点的非零权重数量, 默认 = 8
		//! [@c parameter] Number of non-zero weights per vertex, @c default = 8
		int nnz;
		//! [参数] 权重平滑度软约束, 默认 = 1e-4
		//! [@c parameter] Weights smoothness soft constraint, @c default = 1e-4
		_Scalar weightsSmooth;
		//! [参数] 权重平滑度软约束的步长, 默认 = 1.0
		//! [@c parameter] Step size for the weights smoothness soft constraint, @c default = 1.0
		_Scalar weightsSmoothStep;
		//! [参数] 权重求解器的epsilon值(一个很小的数，用于判断收敛或截断), 默认 = 1e-15
		//! [@c parameter] Epsilon for weights solver, @c default = 1e-15
		_Scalar weightEps;

		/** @brief 构造函数并设置默认参数
		 *  @brief Constructor and setting default parameters
		*/
		DemBones() : nIters(30), nInitIters(10),
			nTransIters(5), transAffine(_Scalar(10)), transAffineNorm(_Scalar(4)),
			nWeightsIters(3), nnz(8), weightsSmooth(_Scalar(1e-4)), weightsSmoothStep(_Scalar(1)),
			weightEps(_Scalar(1e-15)),
			iter(_iter), iterTransformations(_iterTransformations), iterWeights(_iterWeights) {
			clear();
		}

		//! [数据] 顶点数, 通常用索引 i
		//! Number of vertices, typically indexed by @p i
		int nV;
		//! [数据] 骨骼数, 通常用索引 j
		//! Number of bones, typically indexed by @p j
		int nB;
		//! [数据] 主体数 (例如，不同的角色或动画序列), 通常用索引 s
		//! Number of subjects, typically indexed by @p s
		int nS;
		//! [数据] 总帧数, 通常用索引 k, #nF = #fStart(#nS)
		//! Number of total frames, typically indexed by @p k, #nF = #fStart(#nS)
		int nF;

		//! [数据] 起始帧索引, [大小] = #nS+1, #fStart(s) 和 #fStart(s+1) 之间是主体 s 的数据帧
		//! Start frame indices, @c size = #nS+1, #fStart(@p s), #fStart(@p s+1) are data frames for subject @p s
		Eigen::VectorXi fStart;
		//! [数据] 帧所属的主体索引, [大小] = #nF, #subjectID(k)=s, 其中 #fStart(s) <= k < #fStart(s+1)
		//! Subject index of the frame, @c size = #nF, #subjectID(@p k)=@p s, where #fStart(@p s) <= @p k < #fStart(<tt>s</tt>+1)
		Eigen::VectorXi subjectID;

		//! [数据] 静止姿态的几何体, [大小] = [3*#nS, #nV], #u.col(i).segment(3*s, 3) 是主体 s 的顶点 i 的静止姿态位置
		//! Geometry at the rest poses, @c size = [3*#nS, #nV], #u.@a col(@p i).@a segment(3*@p s, 3) is the rest pose of vertex @p i of subject @p s
		MatrixX u;

		//! [输出/输入] 蒙皮权重, [大小] = [#nB, #nV], #w.col(i) 是顶点 i 的蒙皮权重, #w(j, i) 是骨骼 j 对顶点 i 的影响
		//! Skinning weights, @c size = [#nB, #nV], #w.@a col(@p i) are the skinning weights of vertex @p i, #w(@p j, @p i) is the influence of bone @p j to vertex @p i
		SparseMatrix w;

		/** @brief [输出/输入] 骨骼变换, [大小] = [4*#nF, 4*#nB], #m.blk4(k, j) 是骨骼 j 在第 k 帧的 4x4 相对变换矩阵
			@brief Bone transformations, @c size = [4*#nF*4, 4*#nB], #m.@a blk4(@p k, @p j) is the 4*4 relative transformation matrix of bone @p j at frame @p k
			@details 注意这些变换是相对的，即将骨骼 j 的全局变换从静止姿态带到第 k 帧的姿态。
			@details Note that the transformations are relative, that is #m.@a blk4(@p k, @p j) brings the global transformation of bone @p j from the rest pose to the pose at frame @p k.
		*/
		MatrixX m;
		MatrixX origM; // 用于存储原始输入的骨骼变换

		//! [数据] 动画网格序列, [大小] = [3*#nF, #nV], #v.col(i).segment(3*k, 3) 是顶点 i 在第 k 帧的位置
		//! Animated mesh sequence, @c size = [3*#nF, #nV], #v.@a col(@p i).@a segment(3*@p k, 3) is the position of vertex @p i at frame @p k
		Eigen::Matrix<_AniMeshScalar, Eigen::Dynamic, Eigen::Dynamic> v;

		//! [数据] 网格拓扑, [大小]=[多边形数量], #fv[p] 是多边形 p 的顶点索引向量
		//! Mesh topology, @c size=[<tt>number of polygons</tt>], #fv[@p p] is the vector of vertex indices of polygon @p p
		std::vector<std::vector<int>> fv;

		//! [只读] 当前全局迭代次数，可用于回调函数 (从0开始)
		//! [<tt>zero indexed</tt>, <tt>read only</tt>] Current global iteration number that can be used for callback functions
		const int& iter;

		//! [只读] 当前骨骼变换更新的迭代次数，可用于回调函数 (从0开始)
		//! [<tt>zero indexed</tt>, <tt>read only</tt>] Current bone transformations update iteration number that can be used for callback functions
		const int& iterTransformations;

		//! [只读] 当前权重更新的迭代次数，可用于回调函数 (从0开始)
		//! [<tt>zero indexed</tt>, <tt>read only</tt>] Current weights update iteration number that can be used for callback functions
		const int& iterWeights;

		/** @brief 清除所有数据
		 *  @brief Clear all data
		*/
		void clear() {
			nV = nB = nS = nF = 0;
			fStart.resize(0);
			subjectID.resize(0);
			u.resize(0, 0);
			w.resize(0, 0);
			m.resize(0, 0);
			v.resize(0, 0);
			fv.resize(0);
			modelSize = -1;
			laplacian.resize(0, 0);
		}

		/** @brief 初始化缺失的蒙皮权重和/或骨骼变换
			@brief Initialize missing skinning weights and/or bone transformations
			@details 根据 #w 和 #m 的状态，此函数将执行：
				- 如果 #w 和 #m 都已设置：什么都不做。
				- 如果 #w 或 #m 中只有一个缺失（大小为零）：初始化缺失的矩阵。
				- 如果 #w 和 #m 都缺失（大小为零）：使用刚性蒙皮初始化两者，骨骼数约为 #nB，即 #w 的值为0或1。
				此时将使用网格序列 #v、静止姿态几何体 #u 和拓扑 #fv 执行LBG-VQ聚类算法。
				@b 注意: 由于初始化不一定使用精确的 #nB 个骨骼，当 #w 和 #m 都缺失时，#nB 的值可能会改变。

			此函数在每个计算更新函数的开头被调用，作为安全保障。
		*/
		bool init() {
			if (modelSize < 0) modelSize = sqrt((u - (u.rowwise().sum() / nV).replicate(1, nV)).squaredNorm() / nV / nS);
			if (laplacian.cols() != nV) computeSmoothSolver();

			if (((int)w.rows() != nB) || ((int)w.cols() != nV)) { // 没有蒙皮权重
				if (((int)m.rows() != nF * 4) || ((int)m.cols() != nB * 4)) { // 也没有变换
					int targetNB = nB;
					// LBG-VQ 聚类算法
					nB = m.cols() == 0 ? 1 : (int)m.cols() / 4;
					label = Eigen::VectorXi::Zero(nV);
					computeTransFromLabel();
					auto prevNB = nB;
					bool cont = true;
					while (cont) {
						cbInitSplitBegin();
						split(targetNB, nnz);
						cont = (nB < targetNB);
						for (int rep = 0; rep < nInitIters; rep++) {
							computeTransFromLabel();
							computeLabel();
							//pruneBones(nnz);
						}
						cbInitSplitEnd();
						if (prevNB == nB) {
							std::cout << "无法初始化骨骼变换。" << std::endl;
							return false;
						}
						prevNB = nB;
					}
					m.conservativeResize(nF * 4, nB * 4);
					if (origM.rows() && m.cols() >= origM.cols()) {
						m.block(0, 0, origM.rows(), origM.cols()) = origM;
					}
					labelToWeights();
				}
				else initWeights(); // 有变换，初始化权重
			}
			else { // 有蒙皮权重
				if (((int)m.rows() != nF * 4) || ((int)m.cols() != nB * 4)) { // 没有变换
					m = Matrix4::Identity().replicate(nF, nB);
				}
			}
			return true;
		}

		/** @brief 通过运行 #nTransIters 次迭代更新骨骼变换，使用 #transAffine 和 #transAffineNorm 正则化器
			@brief Update bone transformations by running #nTransIters iterations with #transAffine and #transAffineNorm regularizers
			@details 必需输入数据:
				- 静止姿态形状: #u, #fv, #nV
				- 动画序列: #v, #nF, #fStart, #subjectID, #nS
				- 骨骼数量: #nB

			可选输入数据:
				- 蒙皮权重: #w
				- 骨骼变换: #m

			输出: #m. 缺失的 #w 和/或 #m (大小为零) 将由 init() 初始化。
		*/
		void computeTranformations() {
			if (nTransIters == 0) return;

			init();
			cbTranformationsBegin();

			compute_vuT();
			compute_uuT();

			for (_iterTransformations = 0; _iterTransformations < nTransIters; _iterTransformations++) {
				cbTransformationsIterBegin();
#pragma omp parallel for
				for (int k = 0; k < nF; k++)
					for (int j = 0; j < nB; j++) {
						Matrix4 qpT = vuT.blk4(k, j);
						for (int it = uuT.outerIdx(j); it < uuT.outerIdx(j + 1); it++)
							if (uuT.innerIdx(it) != j) qpT -= m.blk4(k, uuT.innerIdx(it)) * uuT.val.blk4(subjectID(k), it);
						qpT2m(qpT, k, j);

					}

				if (origM.rows() && m.cols() >= origM.cols()) {
					m.block(0, 0, origM.rows(), origM.cols()) = origM;
				}
				cbTransformationsIterEnd();
			}

			cbTransformationsEnd();
		}

		/** @brief 通过运行 #nWeightsIters 次迭代更新蒙皮权重，使用 #weightsSmooth 和 #weightsSmoothStep 正则化器
			@brief Update skinning weights by running #nWeightsIters iterations with #weightsSmooth and #weightsSmoothStep regularizers
			@details 必需输入数据:
				- 静止姿态形状: #u, #fv, #nV
				- 动画序列: #v, #nF, #fStart, #subjectID, #nS
				- 骨骼数量: #nB

			可选输入数据:
				- 蒙皮权重: #w
				- 骨骼变换: #m

			输出: #w. 缺失的 #w 和/或 #m (大小为零) 将由 init() 初始化。
		*/
		void computeWeights() {
			if (nWeightsIters == 0) return;

			init();
			cbWeightsBegin();

			compute_mTm();

			aTb = MatrixX::Zero(nB, nV);
			wSolver.init(nnz);
			std::vector<Triplet, Eigen::aligned_allocator<Triplet>> trip;
			trip.reserve(nV * nnz);

			for (_iterWeights = 0; _iterWeights < nWeightsIters; _iterWeights++) {
				cbWeightsIterBegin();

				compute_ws();
				compute_aTb();

				double reg = pow(modelSize, 2) * nF * weightsSmooth;

				trip.clear();
#pragma omp parallel for
				for (int i = 0; i < nV; i++) {
					MatrixX aTai;
					compute_aTa(i, aTai);
					aTai += reg * MatrixX::Identity(nB, nB);
					VectorX aTbi = aTb.col(i) + reg * ws.col(i);

					VectorX x = ws.col(i);
					Eigen::ArrayXi idx = Eigen::ArrayXi::LinSpaced(nB, 0, nB - 1);
					std::sort(idx.data(), idx.data() + nB, [&x](int i1, int i2) { return x(i1) > x(i2); });
					int nnzi = std::min(nnz, nB);
					while (x(idx(nnzi - 1)) < weightEps) nnzi--;

					x = indexing_vector(w.col(i).toDense().cwiseMax(0.0), idx.head(nnzi));
					_Scalar s = x.sum();
					if (s > _Scalar(0.1)) x /= s; else x = VectorX::Constant(nnzi, _Scalar(1) / nnzi);

					wSolver.solve(indexing_row_col(aTai, idx.head(nnzi), idx.head(nnzi)), indexing_vector(aTbi, idx.head(nnzi)), x, true, true);

#pragma omp critical
					for (int j = 0; j < nnzi; j++)
						if (x(j) != 0) trip.push_back(Triplet(idx[j], i, x(j)));
				}

				w.resize(nB, nV);
				w.setFromTriplets(trip.begin(), trip.end());

				cbWeightsIterEnd();
			}

			cbWeightsEnd();
		}

		/** @brief 通过 #nIters 次交替更新权重和骨骼变换来进行蒙皮分解
			@brief Skinning decomposition by #nIters iterations of alternative updating weights and bone transformations
			@details 必需输入数据:
				- 静止姿态形状: #u, #fv, #nV
				- 动画序列: #v, #nF, #fStart, #subjectID, #nS
				- 骨骼数量: #nB

			可选输入数据:
				- 蒙皮权重: #w
				- 骨骼变换: #m

			输出: #w, #m. 缺失的 #w 和/或 #m (大小为零) 将由 init() 初始化。
		*/
		bool compute() {
			if (!init()) {
				return false;
			}

			for (_iter = 0; _iter < nIters; _iter++) {
				cbIterBegin();
				computeTranformations();
				computeWeights();
				cbIterEnd();
			}
			return true;
		}

		//! @return 返回均方根重构误差
		//! @return Root mean squared reconstruction error
		_Scalar rmse() {
			_Scalar e = 0;
#pragma omp parallel for
			for (int i = 0; i < nV; i++) {
				_Scalar ei = 0;
				Matrix4 mki;
				for (int k = 0; k < nF; k++) {
					mki.setZero();
					for (typename SparseMatrix::InnerIterator it(w, i); it; ++it) mki += it.value() * m.blk4(k, it.row());
					ei += (mki.template topLeftCorner<3, 3>() * u.vec3(subjectID(k), i) + mki.template topRightCorner<3, 1>() - v.vec3(k, i).template cast<_Scalar>()).squaredNorm();
				}
#pragma omp atomic
				e += ei;
			}
			return std::sqrt(e / nF / nV);
		}

		// ---- 回调函数定义 ----
		//! 在初始化中每次分裂骨骼簇之前调用的回调函数
		//! Callback function invoked before each spliting of bone clusters in initialization
		virtual void cbInitSplitBegin() {}
		//! 在初始化中每次分裂骨骼簇之后调用的回调函数
		//! Callback function invoked after each spliting of bone clusters in initialization
		virtual void cbInitSplitEnd() {}

		//! 在每次全局迭代更新之前调用的回调函数
		//! Callback function invoked before each global iteration update
		virtual void cbIterBegin() {}
		//! 在每次全局迭代更新之后调用的回调函数
		//! Callback function invoked after each global iteration update
		virtual void cbIterEnd() {}

		//! 在每次蒙皮权重更新之前调用的回调函数
		//! Callback function invoked before each skinning weights update
		virtual void cbWeightsBegin() {}
		//! 在每次蒙皮权重更新之后调用的回调函数
		//! Callback function invoked after each skinning weights update
		virtual void cbWeightsEnd() {}

		//! 在每次骨骼变换更新之前调用的回调函数
		//! Callback function invoked before each bone transformations update
		virtual void cbTranformationsBegin() {}
		//! 在每次骨骼变换更新之后调用的回调函数
		//! Callback function invoked after each bone transformations update
		virtual void cbTransformationsEnd() {}

		//! 在每次局部骨骼变换更新迭代之前调用的回调函数
		//! Callback function invoked before each local bone transformations update iteration
		virtual void cbTransformationsIterBegin() {}
		//! 在每次局部骨骼变换更新迭代之后调用的回调函数
		//! Callback function invoked after each local bone transformations update iteration
		virtual void cbTransformationsIterEnd() {}

		//! 在每次局部权重更新迭代之前调用的回调函数
		//! Callback function invoked before each local weights update iteration
		virtual void cbWeightsIterBegin() {}
		//! 在每次局部权重更新迭代之后调用的回调函数
		//! Callback function invoked after each local weights update iteration
		virtual void cbWeightsIterEnd() {}

	private:
		int _iter, _iterTransformations, _iterWeights;

		/** @brief 从协方差矩阵计算最佳刚性变换 (Polar Decomposition)
			@brief Best rigid transformation from covariance matrix
			@param _qpT 4x4 协方差矩阵
			@param k 帧号
			@param j 骨骼索引
		*/
		void qpT2m(const Matrix4& _qpT, int k, int j) {
			if (_qpT(3, 3) != 0) {
				Matrix4 qpT = _qpT / _qpT(3, 3);
				Eigen::JacobiSVD<Matrix3> svd(qpT.template topLeftCorner<3, 3>() - qpT.template topRightCorner<3, 1>() * qpT.template bottomLeftCorner<1, 3>(), Eigen::ComputeFullU | Eigen::ComputeFullV);
				Matrix3 d = Matrix3::Identity();
				d(2, 2) = (svd.matrixU() * svd.matrixV().transpose()).determinant();
				m.rotMat(k, j) = svd.matrixU() * d * svd.matrixV().transpose();
				m.transVec(k, j) = qpT.template topRightCorner<3, 1>() - m.rotMat(k, j) * qpT.template bottomLeftCorner<1, 3>().transpose();
			}
		}

		/** @brief 计算单个顶点在单个骨骼下的拟合误差
			@brief Fitting error
			@param i 顶点索引
			@param j 骨骼索引
		*/
		_Scalar errorVtxBone(int i, int j, bool par = true) {
			_Scalar e = 0;
#pragma omp parallel for if(par)
			for (int k = 0; k < nF; k++)
#pragma omp atomic
				e += (m.rotMat(k, j) * u.vec3(subjectID(k), i) + m.transVec(k, j) - v.vec3(k, i).template cast<_Scalar>()).squaredNorm();
			return e;
		}

		//! label(i) 是与顶点 i 关联的骨骼索引 (用于初始化聚类)
		Eigen::VectorXi label;

		//! 用于优先队列的比较器，值最小的在顶部
		struct TripletLess {
			bool operator() (const Triplet& t1, const Triplet& t2) {
				return t1.value() > t2.value();
			}
		};

		/** @brief 更新顶点的标签 (即，为每个顶点找到误差最小的骨骼)
		 *  @brief Update labels of vertices
		*/
		void computeLabel() {
			VectorX ei(nV);
			Eigen::VectorXi seed = Eigen::VectorXi::Constant(nB, -1);
			VectorX gMin(nB);
#pragma omp parallel for
			for (int i = 0; i < nV; i++) {
				int j = label(i);
				if (j != -1) {
					ei(i) = errorVtxBone(i, j, false);
					if ((seed(j) == -1) || (ei(i) < gMin(j))) {
#pragma omp critical
						if ((seed(j) == -1) || (ei(i) < gMin(j))) {
							gMin(j) = ei(i);
							seed(j) = i;
						}
					}
				}
			}

			std::priority_queue<Triplet, std::vector<Triplet, Eigen::aligned_allocator<Triplet>>, TripletLess> heap;
			for (int j = 0; j < nB; j++) if (seed(j) != -1) heap.push(Triplet(j, seed(j), ei(seed(j))));

			if (laplacian.cols() != nV) computeSmoothSolver();

			std::vector<bool> dirty(nV, true);
			while (!heap.empty()) {
				Triplet top = heap.top();
				heap.pop();
				int i = (int)top.col();
				int j = (int)top.row();
				if (dirty[i]) {
					label(i) = j;
					ei(i) = top.value();
					dirty[i] = false;
					for (typename SparseMatrix::InnerIterator it(laplacian, i); it; ++it) {
						int i2 = (int)it.row();
						if (dirty[i2]) {
							double tmp = (label(i2) == j) ? ei(i2) : errorVtxBone(i2, j);
							heap.push(Triplet(j, i2, tmp));
						}
					}
				}
			}

#pragma omp parallel for
			for (int i = 0; i < nV; i++)
				if (label(i) == -1) {
					_Scalar gMin;
					for (int j = 0; j < nB; j++) {
						_Scalar ej = errorVtxBone(i, j, false);
						if ((label(i) == -1) || (gMin > ej)) {
							gMin = ej;
							label(i) = j;
						}
					}
				}
		}

		/** @brief 根据顶点标签更新骨骼变换
		 *  @brief Update bone transformation from label
		*/
		void computeTransFromLabel() {
			m = Matrix4::Identity().replicate(nF, nB);
#pragma omp parallel for
			for (int k = 0; k < nF; k++) {
				MatrixX qpT = MatrixX::Zero(4, 4 * nB);
				for (int i = 0; i < nV; i++)
					if (label(i) != -1) qpT.blk4(0, label(i)) += Vector4(v.vec3(k, i).template cast<_Scalar>().homogeneous()) * u.vec3(subjectID(k), i).homogeneous().transpose();
				for (int j = 0; j < nB; j++) qpT2m(qpT.blk4(0, j), k, j);
			}
		}

		/** @brief 根据顶点标签设置权重矩阵 w (刚性绑定)
		 *  @brief Set matrix w from label
		*/
		void labelToWeights() {
			std::vector<Triplet, Eigen::aligned_allocator<Triplet>> trip(nV);
			for (int i = 0; i < nV; i++) trip[i] = Triplet(label(i), i, _Scalar(1));
			w.resize(nB, nV);
			w.setFromTriplets(trip.begin(), trip.end());
		}

		/** @brief 分裂骨骼簇 (用于初始化)
			@brief Split bone clusters
			@param maxB 最大骨骼数
			@param threshold*2 要分裂的骨骼簇的最小尺寸
		*/
		void split(int maxB, int threshold) {
			// Centroids
			MatrixX cu = MatrixX::Zero(3 * nS, nB);
			Eigen::VectorXi s = Eigen::VectorXi::Zero(nB);
			for (int i = 0; i < nV; i++) {
				cu.col(label(i)) += u.col(i);
				s(label(i))++;
			}
			for (int j = 0; j < nB; j++) if (s(j) != 0) cu.col(j) /= _Scalar(s(j));

			// Seed & cluster error
			Eigen::VectorXi seed = Eigen::VectorXi::Constant(nB, -1);
			VectorX gMax(nB);
			VectorX ce = VectorX::Zero(nB);

#pragma omp parallel for
			for (int i = 0; i < nV; i++) {
				int j = label(i);

				double e = errorVtxBone(i, j, false);
#pragma omp atomic
				ce(j) += e;

				double tmp = e * (u.col(i) - cu.col(j)).squaredNorm();

				if ((seed(j) == -1) || (tmp > gMax(j))) {
#pragma omp critical
					if ((seed(j) == -1) || (tmp > gMax(j))) {
						gMax(j) = tmp;
						seed(j) = i;
					}
				}
			}

			int countID = nB;
			_Scalar avgErr = ce.sum() / nB;
			for (int j = 0; j < nB; j++)
				if ((countID < maxB) && (s(j) > threshold * 2) && (ce(j) > avgErr / 100)) {
					int newLabel = countID++;
					int i = seed(j);
					for (typename SparseMatrix::InnerIterator it(laplacian, i); it; ++it) label(it.row()) = newLabel;
				}
			nB = countID;
		}

		/** @brief 移除关联顶点数过少的骨骼
			@brief Remove bones with small number of associated vertices
			@param threshold 骨骼关联的最小顶点数
		*/
		void pruneBones(int threshold) {
			Eigen::VectorXi s = Eigen::VectorXi::Zero(nB);
#pragma omp parallel for
			for (int i = 0; i < nV; i++) {
#pragma omp atomic
				s(label(i))++;
			}
			auto current = nB;
			Eigen::VectorXi newID(nB);
			int countID = 0;
			for (int j = 0; j < nB; j++)
				if (s(j) < threshold) newID(j) = -1; else newID(j) = countID++;

			if (countID == nB) return;

			for (int j = 0; j < nB; j++)
				if (newID(j) != -1) m.template middleCols<4>(newID(j) * 4) = m.template middleCols<4>(j * 4);

#pragma omp parallel for
			for (int i = 0; i < nV; i++) label(i) = newID(label(i));

			nB = countID;
			std::cout << "骨骼数量从 " << current << " 修剪到 " << nB << std::endl;
			m.conservativeResize(nF * 4, nB * 4);
			computeLabel();
		}

		/** @brief 使用到最佳骨骼的刚性绑定来初始化蒙皮权重
		 *  @brief Initialize skinning weights with rigid bind to the best bone
		*/
		void initWeights() {
			label = Eigen::VectorXi::Constant(nV, -1);
#pragma omp parallel for
			for (int i = 0; i < nV; i++) {
				_Scalar gMin;
				for (int j = 0; j < nB; j++) {
					_Scalar ej = errorVtxBone(i, j, false);
					if ((label(i) == -1) || (gMin > ej)) {
						gMin = ej;
						label(i) = j;
					}
				}
			}
			computeLabel();
			labelToWeights();
		}

		//! vuT.blk4(k, j) = sum_{i=0}^{nV-1} w(j, i)*v.vec3(k, i).homogeneous()*u.vec3(subjectID(k), i).homogeneous()^T
		MatrixX vuT;

		/** @brief 预计算 vuT，并加入骨骼平移亲和度软约束
		 *  @brief Pre-compute vuT with bone translations affinity soft constraint
		*/
		void compute_vuT() {
			vuT = MatrixX::Zero(nF * 4, nB * 4);
#pragma omp parallel for
			for (int k = 0; k < nF; k++) {
				MatrixX vuTp = MatrixX::Zero(4, nB * 4);
				for (int i = 0; i < nV; i++)
					for (typename SparseMatrix::InnerIterator it(w, i); it; ++it) {
						Matrix4 tmp = Vector4(v.vec3(k, i).template cast<_Scalar>().homogeneous()) * u.vec3(subjectID(k), i).homogeneous().transpose();
						vuT.blk4(k, it.row()) += it.value() * tmp;
						vuTp.blk4(0, it.row()) += pow(it.value(), transAffineNorm) * tmp;
					}
				for (int j = 0; j < nB; j++)
					if (vuTp(3, j * 4 + 3) != 0)
						vuT.blk4(k, j) += (transAffine * vuT(k * 4 + 3, j * 4 + 3) / vuTp(3, j * 4 + 3)) * vuTp.blk4(0, j);
			}
		}

		//! uuT 是一个稀疏块矩阵, uuT(j, k).block<4, 4>(s*4, 0) = sum{i=0}{nV-1} w(j, i)*w(k, i)*u.col(i).segment<3>(s*3).homogeneous().transpose()*u.col(i).segment<3>(s*3).homogeneous()
		struct SparseMatrixBlock {
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
				MatrixX val;
			Eigen::VectorXi innerIdx, outerIdx;
		} uuT;

		/** @brief 预计算用于骨骼变换更新的 uuT 矩阵
		 *  @brief Pre-compute uuT for bone transformations update
		*/
		void compute_uuT() {
			Eigen::MatrixXi pos = Eigen::MatrixXi::Constant(nB, nB, -1);
#pragma omp parallel for
			for (int i = 0; i < nV; i++)
				for (typename SparseMatrix::InnerIterator it(w, i); it; ++it)
					for (typename SparseMatrix::InnerIterator jt(w, i); jt; ++jt)
						pos(it.row(), jt.row()) = 1;

			uuT.outerIdx.resize(nB + 1);
			uuT.innerIdx.resize(nB * nB);
			int nnz = 0;
			for (int j = 0; j < nB; j++) {
				uuT.outerIdx(j) = nnz;
				for (int i = 0; i < nB; i++)
					if (pos(i, j) != -1) {
						uuT.innerIdx(nnz) = i;
						pos(i, j) = nnz++;
					}
			}
			uuT.outerIdx(nB) = nnz;
			uuT.innerIdx.conservativeResize(nnz);
			uuT.val = MatrixX::Zero(nS * 4, nnz * 4);

#pragma omp parallel for
			for (int i = 0; i < nV; i++)
				for (typename SparseMatrix::InnerIterator it(w, i); it; ++it)
					for (typename SparseMatrix::InnerIterator jt(w, i); jt; ++jt)
						if (it.row() >= jt.row()) {
							double _w = it.value() * jt.value();
							MatrixX _uuT(4 * nS, 4);
							Vector4 _u;
							for (int s = 0; s < nS; s++) {
								_u = u.vec3(s, i).homogeneous();
								_uuT.blk4(s, 0) = _w * _u * _u.transpose();
							}
							int p = pos(it.row(), jt.row()) * 4;
							for (int c = 0; c < 4; c++)
								for (int r = 0; r < 4 * nS; r++)
#pragma omp atomic
									uuT.val(r, p + c) += _uuT(r, c);
						}

			for (int i = 0; i < nB; i++)
				for (int j = i + 1; j < nB; j++)
					if (pos(i, j) != -1)
						uuT.val.middleCols(pos(i, j) * 4, 4) = uuT.val.middleCols(pos(j, i) * 4, 4);
		}



		//! mTm.size = (4*nS*nB, 4*nB), 其中 mTm.block<4, 4>(s*nB+i, j) = sum_{k=fStart(s)}^{fStart(s+1)-1} m.block<3, 4>(k*4, i*4)^T*m.block<3, 4>(k*4, j*4)
		MatrixX mTm;

		/** @brief 预计算用于权重更新的 mTm 矩阵
		 *  @brief Pre-compute mTm for weights update
		*/
		void compute_mTm() {
			Eigen::MatrixXi idx(2, nB * (nB + 1) / 2);
			int nPairs = 0;
			for (int i = 0; i < nB; i++)
				for (int j = i; j < nB; j++) {
					idx(0, nPairs) = i;
					idx(1, nPairs) = j;
					nPairs++;
				}

			mTm = MatrixX::Zero(nS * nB * 4, nB * 4);
#pragma omp parallel for
			for (int p = 0; p < nPairs; p++) {
				int i = idx(0, p);
				int j = idx(1, p);
				for (int k = 0; k < nF; k++)
					mTm.blk4(subjectID(k) * nB + i, j) += m.blk4(k, i).template topRows<3>().transpose() * m.blk4(k, j).template topRows<3>();
				if (i != j) for (int s = 0; s < nS; s++) mTm.blk4(s * nB + j, i) = mTm.blk4(s * nB + i, j);
			}
		}

		//! aTb.col(i) 是顶点 i 的 A^T*b, 其中 A.size = (3*nF, nB), A.col(j).segment<3>(f*3) 是顶点 i 被骨骼 j 在第 f 帧变换后的位置, b = v.col(i).
		MatrixX aTb;

		/** @brief 预计算用于权重更新的 aTb 矩阵
		 *  @brief Pre-compute aTb for weights update
		*/
		void compute_aTb() {
#pragma omp parallel for
			for (int i = 0; i < nV; i++)
				for (int j = 0; j < nB; j++)
					if ((aTb(j, i) == 0) && (ws(j, i) > weightEps))
						for (int k = 0; k < nF; k++)
							aTb(j, i) += v.vec3(k, i).template cast<_Scalar>().dot(m.blk4(k, j).template topRows<3>() * u.vec3(subjectID(k), i).homogeneous());
		}

		//! 模型尺寸 = 到质心的均方根距离 (用于正则化)
		_Scalar modelSize;

		//! 拉普拉斯矩阵
		SparseMatrix laplacian;

		//! 拉普拉斯矩阵的LU分解 (用于平滑求解)
		Eigen::SparseLU<SparseMatrix> smoothSolver;

		/** @brief 预计算拉普拉斯矩阵及其LU分解
		 *  @brief Pre-compute Laplacian and LU factorization
		*/
		void computeSmoothSolver() {
			int nFV = (int)fv.size();

			_Scalar epsDis = 0;
			for (int f = 0; f < nFV; f++) {
				int nf = (int)fv[f].size();
				for (int g = 0; g < nf; g++) {
					int i = fv[f][g];
					int j = fv[f][(g + 1) % nf];
					epsDis += (u.col(i) - u.col(j)).norm();
				}
			}
			epsDis = epsDis * weightEps / (_Scalar)nS;

			std::vector<Triplet, Eigen::aligned_allocator<Triplet>> triplet;
			VectorX d = VectorX::Zero(nV);

#pragma omp parallel for
			for (int f = 0; f < nFV; f++) {
				int nf = (int)fv[f].size();
				for (int g = 0; g < nf; g++) {
					int i = fv[f][g];
					int j = fv[f][(g + 1) % nf];

					if (i < j) {
						double val = 0;
						for (int s = 0; s < nS; s++) {
							double du = (u.vec3(s, i) - u.vec3(s, j)).norm();
							for (int k = fStart(s); k < fStart(s + 1); k++)
								val += pow((v.vec3(k, i).template cast<_Scalar>() - v.vec3(k, j).template cast<_Scalar>()).norm() - du, 2);
						}
						val = 1 / (sqrt(val / nF) + epsDis);

#pragma omp critical
						triplet.push_back(Triplet(i, j, -val));
#pragma omp atomic
						d(i) += val;

#pragma omp critical
						triplet.push_back(Triplet(j, i, -val));
#pragma omp atomic
						d(j) += val;
					}
				}
			}

			for (int i = 0; i < nV; i++)
				triplet.push_back(Triplet(i, i, d(i)));

			laplacian.resize(nV, nV);
			laplacian.setFromTriplets(triplet.begin(), triplet.end());

			for (int i = 0; i < nV; i++)
				if (d(i) != 0) laplacian.row(i) /= d(i);

			laplacian = weightsSmoothStep * laplacian + SparseMatrix((VectorX::Ones(nV)).asDiagonal());
			smoothSolver.compute(laplacian);
		}

		//! 平滑后的蒙皮权重 (作为正则化目标)
		MatrixX ws;

		/** @brief 隐式蒙皮权重拉普拉斯平滑
		 *  @brief Implicit skinning weights Laplacian smoothing
		*/
		void compute_ws() {
			ws = w.transpose();
#pragma omp parallel for
			for (int j = 0; j < nB; j++) ws.col(j) = smoothSolver.solve(ws.col(j));
			ws.transposeInPlace();

#pragma omp parallel for
			for (int i = 0; i < nV; i++) {
				ws.col(i) = ws.col(i).cwiseMax(0.0);
				_Scalar si = ws.col(i).sum();
				if (si < _Scalar(0.1)) ws.col(i) = VectorX::Constant(nB, _Scalar(1) / nB); else ws.col(i) /= si;
			}
		}

		//! 逐顶点的权重求解器
		ConvexLS<_Scalar> wSolver;

		/** @brief 预计算单个顶点的 aTa 矩阵 (用于权重更新)
			@brief Pre-compute aTa for weights update on one vertex
			@param i 顶点索引
			@param aTa 顶点 i 的 A^TA 输出（通过引用传递）, 其中 A.size = (3*nF, nB), A.col(j).segment<3>(f*3) 是顶点 i 被骨骼 j 在第 f 帧变换后的位置。
		*/
		void compute_aTa(int i, MatrixX& aTa) {
			aTa = MatrixX::Zero(nB, nB);
			for (int j1 = 0; j1 < nB; j1++)
				for (int j2 = j1; j2 < nB; j2++) {
					for (int s = 0; s < nS; s++) aTa(j1, j2) += u.vec3(s, i).homogeneous().dot(mTm.blk4(s * nB + j1, j2) * u.vec3(s, i).homogeneous());
					if (j1 != j2) aTa(j2, j1) = aTa(j1, j2);
				}
		}
	};

}

#ifdef DEM_BONES_DEM_BONES_MAT_BLOCKS_UNDEFINED
#undef blk4
#undef rotMat
#undef transVec
#undef vec3
#undef DEM_BONES_MAT_BLOCKS
#endif

#endif