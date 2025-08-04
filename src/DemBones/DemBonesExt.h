
///////////////////////////////////////////////////////////////////////////////
//               Dem Bones - Skinning Decomposition Library                  //
//         Copyright (c) 2019, Electronic Arts. All rights reserved.         //
///////////////////////////////////////////////////////////////////////////////
//               Dem Bones - 蒙皮分解库                                      //
//         版权所有 (c) 2019, Electronic Arts。保留所有权利。                  //
///////////////////////////////////////////////////////////////////////////////


#ifndef DEM_BONES_EXT
#define DEM_BONES_EXT

#include "DemBones.h"

#include <Eigen/Geometry> 

#ifndef DEM_BONES_MAT_BLOCKS
#include "MatBlocks.h"
#define DEM_BONES_DEM_BONES_EXT_MAT_BLOCKS_UNDEFINED
#endif

namespace Dem
{

	/**  @class DemBonesExt DemBonesExt.h "DemBones/DemBonesExt.h"
		@brief 扩展类，用于处理具有局部旋转/平移和绑定矩阵的层次化骨骼。
		@brief Extended class to handle hierarchical skeleton with local rotations/translations and bind matrices

		@details 在蒙皮分解完成并设置好其他数据后，调用 computeRTB() 来获取局部旋转/平移和绑定矩阵。
		@details Call computeRTB() to get local rotations/translations and bind matrices after skinning decomposition is done and other data is set.

		@b _Scalar 是浮点数据类型。 @b _AniMeshScalar 是网格序列 #v 的浮点数据类型。
		@b _Scalar is the floating-point data type. @b _AniMeshScalar is the floating-point data type of mesh sequence #v.
	*/
	template<class _Scalar, class _AniMeshScalar>
	class DemBonesExt : public DemBones<_Scalar, _AniMeshScalar> {
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

		// 从基类继承成员变量
		using DemBones<_Scalar, _AniMeshScalar>::nIters;
		using DemBones<_Scalar, _AniMeshScalar>::nInitIters;
		using DemBones<_Scalar, _AniMeshScalar>::nTransIters;
		using DemBones<_Scalar, _AniMeshScalar>::transAffine;
		using DemBones<_Scalar, _AniMeshScalar>::transAffineNorm;
		using DemBones<_Scalar, _AniMeshScalar>::nWeightsIters;
		using DemBones<_Scalar, _AniMeshScalar>::nnz;
		using DemBones<_Scalar, _AniMeshScalar>::weightsSmooth;
		using DemBones<_Scalar, _AniMeshScalar>::weightsSmoothStep;
		using DemBones<_Scalar, _AniMeshScalar>::weightEps;


		using DemBones<_Scalar, _AniMeshScalar>::nV;
		using DemBones<_Scalar, _AniMeshScalar>::nB;
		using DemBones<_Scalar, _AniMeshScalar>::nS;
		using DemBones<_Scalar, _AniMeshScalar>::nF;
		using DemBones<_Scalar, _AniMeshScalar>::fStart;
		using DemBones<_Scalar, _AniMeshScalar>::subjectID;
		using DemBones<_Scalar, _AniMeshScalar>::u;
		using DemBones<_Scalar, _AniMeshScalar>::w;
		using DemBones<_Scalar, _AniMeshScalar>::m;
		using DemBones<_Scalar, _AniMeshScalar>::v;
		using DemBones<_Scalar, _AniMeshScalar>::fv;

		using DemBones<_Scalar, _AniMeshScalar>::iter;
		using DemBones<_Scalar, _AniMeshScalar>::iterTransformations;
		using DemBones<_Scalar, _AniMeshScalar>::iterWeights;

		//! [数据] 骨骼变换 #m 的时间戳, [大小] = #nS, #fTime(k) 是第 k 帧的时间戳
		//! Timestamps for bone transformations #m, [@c size] = #nS, #fTime(@p k) is the timestamp of frame @p k
		Eigen::VectorXd fTime;

		//! [数据] 骨骼名称, [大小] = #nB, #boneName(j) 是骨骼 j 的名称
		//! Name of bones, [@c size] = #nB, #boneName(@p j) is the name bone of @p j
		std::vector<std::string> boneName;

		//! [数据] 父骨骼索引, [大小] = #nB, #parent(j) 是骨骼 j 的父骨骼索引, 如果 j 没有父骨骼，则 #parent(j) = -1
		//! Parent bone index, [@c size] = #nB, #parent(@p j) is the index of parent bone of @p j, #parent(@p j) = -1 if @p j has no parent.
		Eigen::VectorXi parent;

		//! [数据] 原始绑定预乘矩阵, [大小] = [4*#nS, 4*#nB], #bind.block(4*s, 4*j, 4, 4) 是主体 s 在静止姿态下骨骼 j 的全局绑定矩阵
		//! Original bind pre-matrix, [@c size] = [4*#nS, 4*#nB], #bind.@a block(4*@p s, 4*@p j, 4, 4) is the global bind matrix of bone @p j on subject @p s at the rest pose
		MatrixX bind;

		//! [数据] 逆预乘矩阵, [大小] = [4*#nS, 4*#nB], #preMulInv.block(4*s, 4*j, 4, 4) 是主体 s 上骨骼 j 的预局部变换的逆矩阵
		//! Inverse pre-multiplication matrices, [@c size] = [4*#nS, 4*#nB], #preMulInv.@a block(4*@p s, 4*@p j, 4, 4) is the inverse of pre-local transformation of bone @p j on subject @p s 
		MatrixX preMulInv;

		//! [数据] 旋转顺序, [大小] = [3*#nS, #nB], #rotOrder.col(j).segment<3>(3*s) 是主体 s 上骨骼 j 的旋转顺序, 0=X, 1=Y, 2=Z, 例如 {0, 1, 2} 是 XYZ 顺序
		//! Rotation order, [@c size] = [3*#nS, #nB], #rotOrder.@a col(@p j).@a segment<3>(3*@p s) is the rotation order of bone @p j on subject @p s, 0=@c X, 1=@c Y, 2=@c Z, e.g. {0, 1, 2} is @c XYZ order  
		Eigen::MatrixXi rotOrder;

		//! [参数] 绑定变换更新方式, 0=保持原始值, 1=将平移设置为p-范数质心 (使用 #transAffineNorm) 并将旋转设置为单位旋转
		//! Bind transformation update, 0=keep original, 1=set translations to p-norm centroids (using #transAffineNorm) and rotations to identity
		int bindUpdate;

		/** @brief 构造函数并设置默认参数
		 *  @brief Constructor and setting default parameters
		*/
		DemBonesExt() : bindUpdate(0) {
			clear();
		}

		/** @brief 清除所有数据
		 *  @brief Clear all data
		*/
		void clear() {
			fTime.resize(0);
			boneName.resize(0);
			parent.resize(0);
			bind.resize(0, 0);
			preMulInv.resize(0, 0);
			rotOrder.resize(0, 0);
			DemBones<_Scalar, _AniMeshScalar>::clear();
		}

		/** @brief 计算一个主体的局部旋转、平移和全局绑定矩阵
			@brief Local rotations, translations and global bind matrices of a subject
			@details 需要基类中的所有数据: #u, #fv, #nV, #v, #nF, #fStart, #subjectID, #nS, #m, #w, #nB
			@details Required all data in the base class: #u, #fv, #nV, #v, #nF, #fStart, #subjectID, #nS, #m, #w, #nB

			此函数将为缺失的属性初始化以下默认值：
			- #parent: 全为-1的向量, [大小] = #nB
			- #preMulInv: 4x4单位矩阵块, [大小] = [4*#nS, 4*#nB]
			- #rotOrder: {0, 1, 2} 向量块, [大小] = [3*#nS, #nB]
			This function will initialize these default values for missing attributes:
			- #parent: -1 vector, [@c size] = #nB
			- #preMulInv: 4*4 identity matrix blocks, [@c size] = [4*#nS, 4*#nB]
			- #rotOrder: {0, 1, 2} vector blocks, [@c size] = [3*#nS, #nB]

			@param[in] s 主体索引
			@param[out] lr [3*nFr, #nB] 大小的局部旋转输出（通过引用传递）, lr.col(j).segment<3>(3*k) 是骨骼 j 在第 k 帧的 (rx, ry, rz)
			@param[out] lt [3*nFr, #nB] 大小的局部平移输出（通过引用传递）, lt.col(j).segment<3>(3*k) 是骨骼 j 在第 k 帧的 (tx, ty, tz)
			@param[out] gb [4, 4*#nB] 大小的全局绑定矩阵输出（通过引用传递）, gb.block(0, 4*j, 4, 4) 是骨骼 j 的绑定矩阵
			@param[out] lbr [3, #nB] 大小的绑定姿态局部旋转输出（通过引用传递）, lbr.col(j) 是骨骼 j 的 (rx, ry, rz)
			@param[out] lbt [3, #nB] 大小的绑定姿态局部平移输出（通过引用传递）, lbt.col(j) 是骨骼 j 的 (tx, ty, tz)
			@param[in] degreeRot=true 将以度为单位输出旋转，否则以弧度输出
		*/
		void computeRTB(int s, MatrixX& lr, MatrixX& lt, MatrixX& gb, MatrixX& lbr, MatrixX& lbt, bool degreeRot = true) {
			computeBind(s, gb);

			// 如果父骨骼信息不完整，则用-1填充
			if (parent.size() != nB) {
				// Preserve any existing values
				Eigen::VectorXi temp = Eigen::VectorXi::Constant(nB, -1);
				temp.head(parent.size()) = parent;
				parent = temp;
			}
			// 如果逆预乘矩阵不完整，则用单位矩阵填充
			if (preMulInv.size() != 16 * nS * nB) {
				// Preserve any existing values
				MatrixX temp = MatrixX::Identity(4, 4).replicate(nS, nB);
				temp.block(0, 0, preMulInv.rows(), preMulInv.cols()) = preMulInv;
				preMulInv = temp;
			}
			// 如果旋转顺序不完整，则用默认的XYZ顺序填充
			if (rotOrder.size() != 3 * nS * nB) {
				// Preserve any existing values
				Eigen::MatrixXi temp = Eigen::Vector3i(0, 1, 2).replicate(nS, nB);
				temp.block(0, 0, rotOrder.rows(), rotOrder.cols()) = rotOrder;
				rotOrder = temp;
			}

			int nFs = fStart(s + 1) - fStart(s); // 当前主体的帧数
			lr.resize(nFs * 3, nB);
			lt.resize(nFs * 3, nB);
			lbr.resize(3, nB);
			lbt.resize(3, nB);

			MatrixX lm(4 * nFs, 4 * nB); // 存储局部变换矩阵
#pragma omp parallel for
			for (int j = 0; j < nB; j++) {
				Eigen::Vector3i ro = rotOrder.col(j).template segment<3>(s * 3); // 获取旋转顺序

				Matrix4 lb; // 局部的绑定姿态变换
				// 计算局部绑定姿态变换，如果骨骼有父节点，需要乘以父节点全局绑定矩阵的逆
				if (parent(j) == -1) lb = preMulInv.blk4(s, j) * gb.blk4(0, j);
				else lb = preMulInv.blk4(s, j) * gb.blk4(0, parent(j)).inverse() * gb.blk4(0, j);

				Vector3 curRot = Vector3::Zero();
				toRot(lb.template topLeftCorner<3, 3>(), curRot, ro); // 从旋转矩阵计算欧拉角
				lbr.col(j) = curRot;
				lbt.col(j) = lb.template topRightCorner<3, 1>(); // 提取平移向量

				Matrix4 lm; // 局部的动画帧变换
				for (int k = 0; k < nFs; k++) {
					// 计算局部动画帧变换
					if (parent(j) == -1) lm = preMulInv.blk4(s, j) * m.blk4(k + fStart(s), j) * gb.blk4(0, j);
					else lm = preMulInv.blk4(s, j) * (m.blk4(k + fStart(s), parent(j)) * gb.blk4(0, parent(j))).inverse() * m.blk4(k + fStart(s), j) * gb.blk4(0, j);
					toRot(lm.template topLeftCorner<3, 3>(), curRot, ro); // 从旋转矩阵计算欧拉角
					lr.vec3(k, j) = curRot;
					lt.vec3(k, j) = lm.template topRightCorner<3, 1>(); // 提取平移向量
				}
			}

			// 如果需要，将旋转从弧度转换为度
			if (degreeRot) {
				lr *= 180 / EIGEN_PI;
				lbr *= 180 / EIGEN_PI;
			}
		}

	private:
		/** @brief 计算质心作为骨骼位置，并将旋转设置为单位旋转
			@brief p-norm centroids (using #transAffineNorm) and rotations to identity
			@param s 主体索引
			@param b [4, 4*#nB] 大小的全局绑定矩阵输出（通过引用传递）, b.block(0, 4*j, 4, 4) 是骨骼 j 的绑定矩阵
		*/
		void computeCentroids(int s, MatrixX& b) {
			MatrixX c = MatrixX::Zero(4, nB);
			// 根据权重计算每个骨骼影响的顶点的加权平均位置（p-范数质心）
			for (int i = 0; i < nV; i++)
				for (typename SparseMatrix::InnerIterator it(w, i); it; ++it)
					c.col(it.row()) += pow(it.value(), transAffineNorm) * u.vec3(s, i).homogeneous();
			b = MatrixX::Identity(4, 4).replicate(1, nB);
			for (int j = 0; j < nB; j++)
				if (c(3, j) != 0)	b.transVec(0, j) = c.col(j).template head<3>() / c(3, j);
		}

		/** @brief 计算全局绑定姿态
			@brief Global bind pose
			@param s 主体索引
			@param bindUpdate 绑定姿态的更新类型, 0=保持原始值, 1=将平移设置为p-范数质心 (使用 #transAffineNorm) 并将旋转设置为单位旋转
			@param b [4, 4*#nB] 大小的全局绑定矩阵输出（通过引用传递）, b.block(0, 4*j, 4, 4) 是骨骼 j 的绑定矩阵
		*/
		void computeBind(int s, MatrixX& b) {
			if (bind.size() != nS * 4, nB * 4) {
				MatrixX bindOrig = bind;
				bind.resize(nS * 4, nB * 4);
				MatrixX b_temp;
				for (int k = 0; k < nS; k++) computeCentroids(k, b_temp);
				bind.block(4 * s, 0, 4, 4 * nB) = b_temp;
				// 如果存在旧的绑定姿态，则保留
				// Override bind pose with existing bind pose if preserving existing bones
				bind.block(0, 0, bindOrig.rows(), bindOrig.cols()) = bindOrig;
			}

			switch (bindUpdate) {
			case 0:	b = bind.block(4 * s, 0, 4, 4 * nB); break; // 保持原始绑定姿态
			case 1: computeCentroids(s, b); break; // 重新计算质心作为绑定姿态
			}
		}

		/** @brief 从旋转矩阵计算欧拉角
			@brief Euler angles from rotation matrix
			@param rMat 3x3 旋转矩阵
			@param curRot 输入的当前欧拉角，同时也是输出的与 rMat 对应的最接近的欧拉角（通过引用传递）。这有助于保持动画的连续性，避免万向节锁和角度跳变。
			@param ro 旋转顺序, 0=X, 1=Y, 2=Z, 例如 {0, 1, 2} 是 XYZ 顺序
			@param eps 容差值
		*/
		void toRot(const Matrix3& rMat, Vector3& curRot, const Eigen::Vector3i& ro, _Scalar eps = _Scalar(1e-10)) {
			// Eigen库的默认求解器，可能存在多种解
			Vector3 r0 = rMat.eulerAngles(ro(2), ro(1), ro(0)).reverse();
			_Scalar gMin = (r0 - curRot).squaredNorm();
			Vector3 rMin = r0;
			Vector3 r;
			Matrix3 tmpMat;
			// 暴力搜索所有等价的欧拉角表示（通过加减PI或2PI），找到与上一帧(curRot)最接近的解
			for (int fx = -1; fx <= 1; fx += 2)
				for (_Scalar sx = -2 * EIGEN_PI; sx < 2.1 * EIGEN_PI; sx += EIGEN_PI) {
					r(0) = fx * r0(0) + sx;
					for (int fy = -1; fy <= 1; fy += 2)
						for (_Scalar sy = -2 * EIGEN_PI; sy < 2.1 * EIGEN_PI; sy += EIGEN_PI) {
							r(1) = fy * r0(1) + sy;
							for (int fz = -1; fz <= 1; fz += 2)
								for (_Scalar sz = -2 * EIGEN_PI; sz < 2.1 * EIGEN_PI; sz += EIGEN_PI) {
									r(2) = fz * r0(2) + sz;
									// 根据新的欧拉角构建旋转矩阵
									tmpMat = Matrix3(Eigen::AngleAxis<_Scalar>(r(ro(2)), Vector3::Unit(ro(2)))) *
										Eigen::AngleAxis<_Scalar>(r(ro(1)), Vector3::Unit(ro(1))) *
										Eigen::AngleAxis<_Scalar>(r(ro(0)), Vector3::Unit(ro(0)));
									// 检查构建的矩阵是否与原始矩阵足够接近
									if ((tmpMat - rMat).squaredNorm() < eps) {
										_Scalar tmp = (r - curRot).squaredNorm();
										// 如果这个解离上一帧的解更近，则更新为最优解
										if (tmp < gMin) {
											gMin = tmp;
											rMin = r;
										}
									}
								}
						}
				}
			curRot = rMin;
		}
	};

}

#ifdef DEM_BONES_DEM_BONES_EXT_MAT_BLOCKS_UNDEFINED
#undef blk4
#undef rotMat
#undef transVec
#undef vec3
#undef DEM_BONES_MAT_BLOCKS
#endif

#undef rotMatFromEuler

#endif
