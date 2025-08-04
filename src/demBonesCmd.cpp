#include "demBonesCmd.h"
#include "common.h"
// 这是一个预处理指令，用于处理 DemBones 库的头文件包含问题。
// 如果 DEM_BONES_MAT_BLOCKS 宏没有被定义，就包含 DemBones/MatBlocks.h，
// 并定义一个宏，以便在文件末尾可以撤销某些定义，避免宏污染。
#ifndef DEM_BONES_MAT_BLOCKS
#include "DemBones/MatBlocks.h"
#define DEM_BONES_DEM_BONES_MAT_BLOCKS_UNDEFINED
#endif

// 包含所有必要的 Maya API 头文件
#include <maya/MGlobal.h>           // <<<<<< 新增: 用于在脚本编辑器中打印信息 (displayInfo)
#include <maya/MAnimControl.h>      // 用于控制动画播放，如获取/设置当前时间
#include <maya/MDagPath.h>        // 用于表示场景中 DAG（有向无环图）对象的路径
#include <maya/MEulerRotation.h>    // 用于处理欧拉角旋转
#include <maya/MFnAnimCurve.h>      // 用于操作动画曲线
#include <maya/MFnDagNode.h>        // 用于操作 DAG 节点
#include <maya/MFnMatrixData.h>     // 用于处理矩阵数据
#include <maya/MFnMesh.h>           // 用于操作网格（Mesh）对象
#include <maya/MFnSet.h>            // 用于操作集（Set），在皮肤簇中用于管理变形器成员
#include <maya/MFnSkinCluster.h>    // 用于操作皮肤簇（SkinCluster）节点
#include <maya/MFnTransform.h>      // 用于操作变换节点（如关节）
#include <maya/MMatrix.h>           // Maya 的 4x4 矩阵类
#include <maya/MPlug.h>             // 用于表示和操作节点的属性（Attribute）
#include <maya/MTime.h>             // 用于表示时间
#include <algorithm>                // <<<<<< 新增: 用于 std::min 函数

// --- 命令的标志（Flags）定义 ---
// 这些常量定义了命令的短名称和长名称，方便用户在 MEL 或 Python 中调用。

// 权重平滑步长 (Weights Smooth Step)
const char* DemBonesCmd::kWeightsSmoothStepShort = "-wss";
const char* DemBonesCmd::kWeightsSmoothStepLong = "-weightsSmoothStep";
// 权重平滑度 (Weights Smooth)
const char* DemBonesCmd::kWeightsSmoothShort = "-ws";
const char* DemBonesCmd::kWeightsSmoothLong = "-weightsSmooth";
// 每个顶点的最大影响骨骼数 (Max Influences)
const char* DemBonesCmd::kNumNonZeroShort = "-mi";
const char* DemBonesCmd::kNumNonZeroLong = "-maxInfluences";
// 权重计算迭代次数 (Weight Iters)
const char* DemBonesCmd::kWeightItersShort = "-wi";
const char* DemBonesCmd::kWeightItersLong = "-weightIters";
// 变换仿射范数 (Trans Affine Norm)
const char* DemBonesCmd::kTransAffineNormShort = "-tan";
const char* DemBonesCmd::kTransAffineNormLong = "-transAffineNorm";
// 变换仿射 (Trans Affine)
const char* DemBonesCmd::kTransAffineShort = "-ta";
const char* DemBonesCmd::kTransAffineLong = "-transAffine";
// 是否更新绑定姿势 (Bind Update)
const char* DemBonesCmd::kBindUpdateShort = "-nu";
const char* DemBonesCmd::kBindUpdateLong = "-bindUpdate";
// 变换计算迭代次数 (Trans Iters)
const char* DemBonesCmd::kTransItersShort = "-ti";
const char* DemBonesCmd::kTransItersLong = "-transIters";
// 主迭代次数 (Iters)
const char* DemBonesCmd::kItersShort = "-i";
const char* DemBonesCmd::kItersLong = "-iters";
// 初始化迭代次数 (Init Iters)
const char* DemBonesCmd::kInitItersShort = "-ii";
const char* DemBonesCmd::kInitItersLong = "-initIters";
// 骨骼数量 (Bones)
const char* DemBonesCmd::kBonesShort = "-b";
const char* DemBonesCmd::kBonesLong = "-bones";
// 开始帧 (Start Frame)
const char* DemBonesCmd::kStartFrameShort = "-sf";
const char* DemBonesCmd::kStartFrameLong = "-startFrame";
// 结束帧 (End Frame)
const char* DemBonesCmd::kEndFrameShort = "-ef";
const char* DemBonesCmd::kEndFrameLong = "-endFrame";
// 使用现有骨骼 (Existing Bones)
const char* DemBonesCmd::kExistingBonesShort = "-eb";
const char* DemBonesCmd::kExistingBonesLong = "-existingBones";
// 命令的名称
const MString DemBonesCmd::kName("demBones");

// Maya 插件的创建器函数，Maya 加载插件时会调用它来创建命令对象实例
void* DemBonesCmd::creator() { return new DemBonesCmd; }

// 指定该命令是否可以被撤销（Undo）
bool DemBonesCmd::isUndoable() const { return true; }

// 定义命令的语法（Syntax），包括所有标志和参数
MSyntax DemBonesCmd::newSyntax() {
    MSyntax syntax;

    // 添加所有标志及其类型
    syntax.addFlag(kWeightsSmoothStepShort, kWeightsSmoothStepLong, MSyntax::kDouble);
    syntax.addFlag(kWeightsSmoothShort, kWeightsSmoothLong, MSyntax::kDouble);
    syntax.addFlag(kNumNonZeroShort, kNumNonZeroLong, MSyntax::kLong);
    syntax.addFlag(kWeightItersShort, kWeightItersLong, MSyntax::kLong);
    syntax.addFlag(kTransAffineNormShort, kTransAffineNormLong, MSyntax::kDouble);
    syntax.addFlag(kTransAffineShort, kTransAffineLong, MSyntax::kDouble);
    syntax.addFlag(kBindUpdateShort, kBindUpdateLong, MSyntax::kBoolean);
    syntax.addFlag(kTransItersShort, kTransItersLong, MSyntax::kLong);
    syntax.addFlag(kItersShort, kItersLong, MSyntax::kLong);
    syntax.addFlag(kInitItersShort, kItersLong, MSyntax::kLong); // 注意：这里长标志和 kItersLong 一样，可能是个笔误，但我们遵循原代码
    syntax.addFlag(kBonesShort, kBonesLong, MSyntax::kLong);
    syntax.addFlag(kStartFrameShort, kStartFrameLong, MSyntax::kDouble);
    syntax.addFlag(kEndFrameShort, kEndFrameLong, MSyntax::kDouble);
    syntax.addFlag(kExistingBonesShort, kExistingBonesLong, MSyntax::kString);
    // 允许 -eb/-existingBones 标志被多次使用
    syntax.makeFlagMultiUse(kExistingBonesShort);

    // 指定命令需要一个对象作为参数（在这里是网格），数量最少1个，最多1个
    syntax.setObjectType(MSyntax::kSelectionList, 1, 1);
    // 如果用户没有明确提供对象，则默认使用当前选择的对象
    syntax.useSelectionAsDefault(true);

    // 禁用编辑（-e）和查询（-q）模式
    syntax.enableEdit(false);
    syntax.enableQuery(false);

    return syntax;
}

// 命令的主要执行函数
MStatus DemBonesCmd::doIt(const MArgList& argList) {
    MStatus status;

    // >>>>>>>>>>>>>>>>>>>> 新增功能：开始执行回显 <<<<<<<<<<<<<<<<<<<<
    MGlobal::displayInfo("--- [DemBones 命令] 开始执行 ---");
    // <<<<<<<<<<<<<<<<<<<<<<<< 功能结束 <<<<<<<<<<<<<<<<<<<<<<<<<<<

    // 创建一个参数数据库，用于解析和查询 argList 中的标志和参数
    MArgDatabase argData(syntax(), argList, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status); // 检查并返回解析状态

    // 获取命令作用的目标对象（网格）
    MSelectionList selection;
    status = argData.getObjects(selection);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    status = selection.getDagPath(0, pathMesh_); // 将第一个选定对象存入 pathMesh_
    CHECK_MSTATUS_AND_RETURN_IT(status);
    status = getShapeNode(pathMesh_); // 确保我们获取到的是形状节点（ShapeNode）而不是变换节点
    CHECK_MSTATUS_AND_RETURN_IT(status);

    // >>>>>>>>>>>>>>>>>>>> 新增功能：回显目标网格 <<<<<<<<<<<<<<<<<<<<
    MGlobal::displayInfo("处理目标网格: " + pathMesh_.partialPathName());
    // <<<<<<<<<<<<<<<<<<<<<<<< 功能结束 <<<<<<<<<<<<<<<<<<<<<<<<<<<

    // 读取动画的开始帧和结束帧
    double startFrame = MAnimControl::animationStartTime().value(); // 默认使用场景的动画开始时间
    if (argData.isFlagSet(kStartFrameShort)) {
        startFrame = argData.flagArgumentDouble(kStartFrameShort, 0, &status);
        CHECK_MSTATUS_AND_RETURN_IT(status);
    }
    double endFrame = MAnimControl::animationEndTime().value(); // 默认使用场景的动画结束时间
    if (argData.isFlagSet(kEndFrameShort)) {
        endFrame = argData.flagArgumentDouble(kEndFrameShort, 0, &status);
        CHECK_MSTATUS_AND_RETURN_IT(status);
    }

    // >>>>>>>>>>>>>>>>>>>> 新增功能：回显处理帧范围 <<<<<<<<<<<<<<<<<<<<
    MGlobal::displayInfo("处理帧范围: " + MString() + startFrame + " 到 " + MString() + endFrame);
    // <<<<<<<<<<<<<<<<<<<<<<<< 功能结束 <<<<<<<<<<<<<<<<<<<<<<<<<<<

    // 如果用户指定了现有的骨骼
    if (argData.isFlagSet(kExistingBonesShort)) {
        unsigned int count = argData.numberOfFlagUses(kExistingBonesShort); // 获取 -eb 标志使用的次数
        pathBones_.setLength(count); // 设置骨骼路径数组的大小
        // >>>>>>>>>>>>>>>>>>>> 新增功能：回显现有骨骼 <<<<<<<<<<<<<<<<<<<<
        MGlobal::displayInfo("使用 " + MString() + count + " 个现有骨骼:");
        // <<<<<<<<<<<<<<<<<<<<<<<< 功能结束 <<<<<<<<<<<<<<<<<<<<<<<<<<<
        unsigned int pos;
        for (unsigned int i = 0; i < count; ++i) {
            MArgList mArgs;
            // 获取第 i 次使用 -eb 标志时传递的参数列表
            status = argData.getFlagArgumentList(kExistingBonesShort, i, mArgs);
            CHECK_MSTATUS_AND_RETURN_IT(status);
            MString boneName = mArgs.asString(0); // 获取骨骼名称
            status = getDagPath(boneName, pathBones_[i]); // 根据名称获取骨骼的 DAG 路径
            CHECK_MSTATUS_AND_RETURN_IT(status);
            // >>>>>>>>>>>>>>>>>>>> 新增功能：打印骨骼名称 <<<<<<<<<<<<<<<<<<<<
            MGlobal::displayInfo("  - " + boneName);
            // <<<<<<<<<<<<<<<<<<<<<<<< 功能结束 <<<<<<<<<<<<<<<<<<<<<<<<<<<
        }
    }

    // 读取网格的动画序列数据
    status = readMeshSequence(startFrame, endFrame);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    // 读取网格的绑定姿势（Bind Pose）数据
    status = readBindPose();
    CHECK_MSTATUS_AND_RETURN_IT(status);

    // --- 解析并设置 DemBones 算法的参数 ---
    // 将用户通过标志传入的参数设置到 model_ 对象中
    if (argData.isFlagSet(kItersShort)) {
        model_.nIters = argData.flagArgumentInt(kItersShort, 0, &status);
        CHECK_MSTATUS_AND_RETURN_IT(status);
    }
    if (argData.isFlagSet(kTransItersShort)) {
        model_.nTransIters = argData.flagArgumentInt(kTransItersShort, 0, &status);
        CHECK_MSTATUS_AND_RETURN_IT(status);
    }
    if (argData.isFlagSet(kWeightItersShort)) {
        model_.nWeightsIters = argData.flagArgumentInt(kWeightItersShort, 0, &status);
        CHECK_MSTATUS_AND_RETURN_IT(status);
    }
    if (argData.isFlagSet(kBindUpdateShort)) {
        model_.bindUpdate = static_cast<int>(argData.flagArgumentBool(kBindUpdateShort, 0, &status));
        CHECK_MSTATUS_AND_RETURN_IT(status);
    }
    if (argData.isFlagSet(kTransAffineShort)) {
        model_.transAffine = argData.flagArgumentDouble(kTransAffineShort, 0, &status);
        CHECK_MSTATUS_AND_RETURN_IT(status);
    }
    if (argData.isFlagSet(kTransAffineNormShort)) {
        model_.transAffineNorm = argData.flagArgumentDouble(kTransAffineNormShort, 0, &status);
        CHECK_MSTATUS_AND_RETURN_IT(status);
    }
    if (argData.isFlagSet(kNumNonZeroShort)) {
        model_.nnz = argData.flagArgumentInt(kNumNonZeroShort, 0, &status);
        CHECK_MSTATUS_AND_RETURN_IT(status);
    }
    if (argData.isFlagSet(kWeightsSmoothShort)) {
        model_.weightsSmooth = argData.flagArgumentDouble(kWeightsSmoothShort, 0, &status);
        CHECK_MSTATUS_AND_RETURN_IT(status);
    }
    if (argData.isFlagSet(kWeightsSmoothStepShort)) {
        model_.weightsSmoothStep = argData.flagArgumentDouble(kWeightsSmoothStepShort, 0, &status);
        CHECK_MSTATUS_AND_RETURN_IT(status);
    }

    if (argData.isFlagSet(kInitItersShort)) {
        model_.nInitIters = argData.flagArgumentDouble(kInitItersShort, 0, &status);
        CHECK_MSTATUS_AND_RETURN_IT(status);
    }

    // 如果用户指定了要创建的骨骼数量，并且已经有存在的骨骼，则在现有基础上增加
    if (argData.isFlagSet(kBonesShort) && model_.nB > 0) {
        int boneCount = argData.flagArgumentInt(kBonesShort, 0, &status);
        CHECK_MSTATUS_AND_RETURN_IT(status);
        model_.nB += boneCount;
    }

    // 如果没有任何骨骼信息（既没有指定现有骨骼，也没有指定创建数量）
    if (model_.nB == 0) {
        if (!argData.isFlagSet(kBonesShort)) {
            MGlobal::displayError("错误: 未找到关节。请使用 -b/-bones 标志指定要创建的骨骼数量。");
            return MS::kInvalidParameter; // 返回参数错误
        }

        // 根据用户指定的数量初始化骨骼
        model_.nB = argData.flagArgumentInt(kBonesShort, 0, &status);
    }

    // >>>>>>>>>>>>>>>>>>>> 新增功能：回显所有生效的参数 <<<<<<<<<<<<<<<<<<<<
    MGlobal::displayInfo("--- [DemBones 命令] 生效参数汇总 ---");
    MGlobal::displayInfo("请求/现有骨骼数 (-b/-eb): " + MString() + model_.nB);
    MGlobal::displayInfo("主迭代次数 (-i): " + MString() + model_.nIters);
    MGlobal::displayInfo("初始迭代次数 (-ii): " + MString() + model_.nInitIters);
    MGlobal::displayInfo("变换迭代次数 (-ti): " + MString() + model_.nTransIters);
    MGlobal::displayInfo("权重迭代次数 (-wi): " + MString() + model_.nWeightsIters);
    MGlobal::displayInfo("最大影响数 (-mi): " + MString() + model_.nnz);
    MGlobal::displayInfo("更新绑定姿势 (-nu): " + MString(model_.bindUpdate ? "是" : "否"));
    MGlobal::displayInfo("变换仿射惩罚 (-ta): " + MString() + model_.transAffine);
    MGlobal::displayInfo("变换仿射范数 (-tan): " + MString() + model_.transAffineNorm);
    MGlobal::displayInfo("权重平滑度 (-ws): " + MString() + model_.weightsSmooth);
    MGlobal::displayInfo("权重平滑步长 (-wss): " + MString() + model_.weightsSmoothStep);
    MGlobal::displayInfo("--------------------------------------");
    // <<<<<<<<<<<<<<<<<<<<<<<< 功能结束 <<<<<<<<<<<<<<<<<<<<<<<<<<<

    // 如果没有任何现有骨骼，则初始化新骨骼
    if (pathBones_.length() == 0) {
        MGlobal::displayInfo("未提供现有骨骼，将初始化 " + MString() + model_.nB + " 个新骨骼...");
        model_.init(); // 调用 DemBones 库的初始化函数
        MGlobal::displayInfo("...骨骼初始化完毕。");
    }

    // 开始核心计算
    MGlobal::displayInfo("正在开始计算蒙皮分解 (此步骤将根据动画复杂度和参数优化骨骼数量)...");
    if (!model_.compute()) {
        MGlobal::displayError("蒙皮分解计算失败。");
        return MS::kFailure; // 如果计算失败，返回失败状态
    }
    MGlobal::displayInfo("...计算成功完成。最终有效骨骼数量为: " + MString() + model_.nB);

    // 调用 redoIt 来将计算结果应用到 Maya 场景中
    return redoIt();
}

// 读取网格在指定时间范围内的顶点动画数据
MStatus DemBonesCmd::readMeshSequence(double startFrame, double endFrame) {
    MStatus status;
    // >>>>>>>>>>>>>>>>>>>> 新增功能：回显读取序列信息 <<<<<<<<<<<<<<<<<<<<
    MGlobal::displayInfo("正在读取网格动画序列...");
    // <<<<<<<<<<<<<<<<<<<<<<<< 功能结束 <<<<<<<<<<<<<<<<<<<<<<<<<<<

    model_.nS = 1; // 假设只有一个 subject（处理对象）
    model_.nF = static_cast<int>(endFrame - startFrame + 1.0); // 计算总帧数

    MFnMesh fnMesh(pathMesh_, &status); // 创建网格函数集
    CHECK_MSTATUS_AND_RETURN_IT(status);
    model_.nV = fnMesh.numVertices(); // 获取顶点数量

    // >>>>>>>>>>>>>>>>>>>> 新增功能：回显网格和动画信息 <<<<<<<<<<<<<<<<<<<<
    MGlobal::displayInfo("  - 顶点数: " + MString() + model_.nV);
    MGlobal::displayInfo("  - 帧数: " + MString() + model_.nF);
    // <<<<<<<<<<<<<<<<<<<<<<<< 功能结束 <<<<<<<<<<<<<<<<<<<<<<<<<<<

    model_.v.resize(3 * model_.nF, model_.nV); // 调整顶点位置矩阵大小（(3*帧数) x 顶点数）
    model_.fTime.resize(model_.nF); // 调整时间戳数组大小
    model_.fStart.resize(model_.nS + 1); // 调整每个 subject 的起始帧索引
    model_.fStart(0) = 0;
    model_.nB = pathBones_.length(); // 获取骨骼数量
    model_.m.resize(model_.nF * 4, model_.nB * 4); // 调整骨骼变换矩阵大小

    // 将当前时间设置为 0，以便获取骨骼的层级和绑定信息
    MTime time(0.0, MTime::kFilm);
    status = MAnimControl::setCurrentTime(time);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    // 存储骨骼名称
    model_.boneName.resize(model_.nB);
    for (unsigned int i = 0; i < model_.nB; ++i) {
        model_.boneName[i] = pathBones_[i].partialPathName().asChar();
    }

    // 调整数据结构大小以存储骨骼层级、绑定矩阵、预乘矩阵和旋转顺序
    model_.parent.resize(model_.nB);
    model_.bind.resize(model_.nS * 4, model_.nB * 4);
    model_.preMulInv.resize(model_.nS * 4, model_.nB * 4);
    model_.rotOrder.resize(model_.nS * 3, model_.nB);
    int s = 0; // subject 索引，这里为 0

    // 遍历所有骨骼，获取其父子关系和绑定信息
    for (int j = 0; j < model_.nB; j++) {
        // 查找父骨骼
        model_.parent(j) = -1; // 默认为根骨骼（没有父节点）
        MDagPath parent(pathBones_[j]);
        status = parent.pop(); // 移动到父节点
        if (!MFAIL(status)) { // 如果有父节点
            for (int k = 0; k < model_.nB; k++) {
                if (model_.boneName[k] == parent.partialPathName().asChar()) {
                    model_.parent(j) = k; // 记录父骨骼的索引
                    break;
                }
            }
        }

        // 获取绑定矩阵（世界空间下的初始变换）
        model_.bind.blk4(s, j) = toMatrix4d(pathBones_[j].inclusiveMatrix());

        // 获取旋转顺序
        MFnTransform fnTransform(pathBones_[j], &status);
        CHECK_MSTATUS_AND_RETURN_IT(status);
        MEulerRotation rotation;
        fnTransform.getRotation(rotation);
        switch (rotation.order) {
        case MEulerRotation::kXYZ: model_.rotOrder.vec3(s, j) = Eigen::Vector3i(0, 1, 2); break;
        case MEulerRotation::kYZX: model_.rotOrder.vec3(s, j) = Eigen::Vector3i(1, 2, 0); break;
        case MEulerRotation::kZXY: model_.rotOrder.vec3(s, j) = Eigen::Vector3i(2, 0, 1); break;
        case MEulerRotation::kXZY: model_.rotOrder.vec3(s, j) = Eigen::Vector3i(0, 2, 1); break;
        case MEulerRotation::kYXZ: model_.rotOrder.vec3(s, j) = Eigen::Vector3i(1, 0, 2); break;
        case MEulerRotation::kZYX: model_.rotOrder.vec3(s, j) = Eigen::Vector3i(2, 1, 0); break;
        }

        // 获取预乘矩阵的逆。在 Maya 中，关节的 pre-transform 通常是单位矩阵。
        MMatrix preMulInv;  // 默认为单位矩阵
        model_.preMulInv.blk4(s, j) = toMatrix4d(preMulInv);
    }

    // TODO: 使用现有的骨骼权重。目前这部分代码被注释掉了。
    Eigen::MatrixXd wd(0, 0);
      /*if (importer.wT.size() != 0) {
    wd = MatrixXd::Zero(model.nB, model.nV);
    for (int j = 0; j < model.nB; j++){
      wd.row(j) = importer.wT[model.boneName[j]].transpose();
    }
    }*/

    model_.w = (wd / model_.nS).sparseView(1, 1e-20);

    bool hasKeyFrame = true; // 假设有关键帧数据
    if (!hasKeyFrame) {
        model_.m.resize(0, 0); // 如果没有，则清空变换矩阵
    }

    // 遍历所有 subjects（此处只有一个）
    for (int s = 0; s < model_.nS; s++) {
        int start = model_.fStart(s);
        // 遍历每一帧，读取顶点位置和骨骼变换
        for (int f = 0; f < model_.nF; ++f) {
            double frame = startFrame + static_cast<double>(f);
            time.setValue(frame);
            status = MAnimControl::setCurrentTime(time); // 设置 Maya 的当前时间
            CHECK_MSTATUS_AND_RETURN_IT(status);
            model_.fTime(start + f) = frame; // 记录当前帧号

            // 获取该帧下网格的所有顶点在世界空间中的位置
            MPointArray points;
            fnMesh.getPoints(points, MSpace::kWorld);

            // 使用 OpenMP 并行处理，将顶点数据存入 model_.v 矩阵
#pragma omp parallel for
            for (int i = 0; i < model_.nV; i++) {
                model_.v.col(i).segment<3>((start + f) * 3) << points[i].x, points[i].y, points[i].z;
            }

            // 计算每个骨骼在当前帧相对于其绑定姿态的变换矩阵
            for (int j = 0; j < model_.nB; ++j) {
                model_.m.blk4(f, j) = toMatrix4d(pathBones_[j].inclusiveMatrix()) * model_.bind.blk4(s, j).inverse();
            }
        }
        model_.fStart(s + 1) = model_.fStart(s) + model_.nF;
    }

    model_.origM = model_.m; // 保存原始的变换矩阵

    // 为每一帧设置其对应的 subject ID
    model_.subjectID.resize(model_.nF);
    for (int s = 0; s < model_.nS; s++) {
        for (int k = model_.fStart(s); k < model_.fStart(s + 1); k++) {
            model_.subjectID(k) = s;
        }
    }

    // >>>>>>>>>>>>>>>>>>>> 新增功能：回显读取完成 <<<<<<<<<<<<<<<<<<<<
    MGlobal::displayInfo("...网格序列读取完毕。");
    // <<<<<<<<<<<<<<<<<<<<<<<< 功能结束 <<<<<<<<<<<<<<<<<<<<<<<<<<<

    return MS::kSuccess;
}

// 读取网格的绑定姿势（通常在第 0 帧）
MStatus DemBonesCmd::readBindPose() {
    MStatus status;
    // >>>>>>>>>>>>>>>>>>>> 新增功能：回显读取绑定姿势 <<<<<<<<<<<<<<<<<<<<
    MGlobal::displayInfo("正在读取绑定姿势和网格拓扑...");
    // <<<<<<<<<<<<<<<<<<<<<<<< 功能结束 <<<<<<<<<<<<<<<<<<<<<<<<<<<

    // 将时间设置到第 0 帧
    MTime time(0.0, MTime::kFilm);
    status = MAnimControl::setCurrentTime(time);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    // 获取网格函数集
    MFnMesh fnMesh(pathMesh_, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    // 获取绑定姿态下的世界空间顶点位置
    MPointArray points;
    fnMesh.getPoints(points, MSpace::kWorld);

    // 将顶点位置存储到 model_.u 中
    model_.u.resize(model_.nS * 3, model_.nV);
    Eigen::MatrixXd v(3, fnMesh.numVertices());
    for (int i = 0; i < model_.nV; i++) {
        v.col(i) << points[i].x, points[i].y, points[i].z;
    }
    model_.u.block(0, 0, 3, model_.nV) = v;

    // 读取网格的拓扑结构（面和顶点的对应关系）
    int numPolygons = fnMesh.numPolygons();
    model_.fv.resize(numPolygons);
    for (int i = 0; i < numPolygons; i++) {
        MIntArray vertexList;
        fnMesh.getPolygonVertices(i, vertexList);
        model_.fv[i].resize(vertexList.length());
        for (unsigned int j = 0; j < model_.fv[i].size(); ++j) {
            model_.fv[i][j] = vertexList[j];
        }
    }
    // >>>>>>>>>>>>>>>>>>>> 新增功能：回显读取完成 <<<<<<<<<<<<<<<<<<<<
    MGlobal::displayInfo("...绑定姿势与拓扑读取完毕。");
    // <<<<<<<<<<<<<<<<<<<<<<<< 功能结束 <<<<<<<<<<<<<<<<<<<<<<<<<<<
    return MS::kSuccess;
}

// "重做"函数，将计算结果应用到 Maya 场景中
MStatus DemBonesCmd::redoIt() {
    MStatus status;
    clearResult(); // 清除上一次命令的结果

    // >>>>>>>>>>>>>>>>>>>> 新增功能：回显应用结果 <<<<<<<<<<<<<<<<<<<<
    MGlobal::displayInfo("正在将计算结果应用到 Maya 场景...");
    // <<<<<<<<<<<<<<<<<<<<<<<< 功能结束 <<<<<<<<<<<<<<<<<<<<<<<<<<<

    // 检查是否需要创建新的关节
    bool needCreateJoints = (model_.boneName.size() != model_.nB);
    std::vector<std::string> newBoneNames; // 存储新创建的关节名称
    MStringArray joints; // 用于返回给用户的关节列表

    if (needCreateJoints) {
        int creationCount = model_.nB - static_cast<int>(model_.boneName.size());
        // >>>>>>>>>>>>>>>>>>>> 新增功能：回显创建关节信息 <<<<<<<<<<<<<<<<<<<<
        MGlobal::displayInfo("正在创建 " + MString() + creationCount + " 个新关节...");
        // <<<<<<<<<<<<<<<<<<<<<<<< 功能结束 <<<<<<<<<<<<<<<<<<<<<<<<<<<
        for (int j = 0; j < creationCount; j++) {
            std::ostringstream s;
            s << "dembones_joint" << j; // 生成默认关节名
            model_.boneName.push_back(s.str());
            newBoneNames.push_back(s.str());
            joints.append(s.str().c_str());
        }
    }

    // 遍历所有 subject（此处只有一个）
    for (int s = 0; s < model_.nS; ++s) {
        Eigen::MatrixXd lr, lt, gb, lbr, lbt;
        // 从 DemBones 模型中计算出最终的旋转(lr)、平移(lt)、全局绑定矩阵(gb)等
        model_.computeRTB(s, lr, lt, gb, lbr, lbt, false);

        // 如果需要，创建新的关节节点
        for (const auto& boneName : newBoneNames) {
            MString cmd("createNode \"joint\" -n \"" + MString(boneName.c_str()) + "\"");
            MGlobal::executeCommand(cmd); // 执行 MEL 命令创建关节
        }

        // >>>>>>>>>>>>>>>>>>>> 新增功能：回显设置关键帧信息 <<<<<<<<<<<<<<<<<<<<
        MGlobal::displayInfo("正在为关节设置动画关键帧...");
        // <<<<<<<<<<<<<<<<<<<<<<<< 功能结束 <<<<<<<<<<<<<<<<<<<<<<<<<<<
        // 为所有骨骼（包括新旧）设置动画关键帧
        int startJointIdx = newBoneNames.empty() ? 0 : model_.boneName.size() - newBoneNames.size();
        for (int j = startJointIdx; j < model_.boneName.size(); ++j) {
            MDagPath pathJoint;
            status = getDagPath(model_.boneName[j].c_str(), pathJoint);
            CHECK_MSTATUS_AND_RETURN_IT(status);

            // 为旋转属性(rx, ry, rz)设置关键帧
            Eigen::VectorXd val = lr.col(j);
            setKeyframes(Eigen::Map<Eigen::VectorXd, 0, Eigen::InnerStride<3>>(val.data() + 0, val.size() / 3), model_.fTime, pathJoint, "rx");
            setKeyframes(Eigen::Map<Eigen::VectorXd, 0, Eigen::InnerStride<3>>(val.data() + 1, val.size() / 3), model_.fTime, pathJoint, "ry");
            setKeyframes(Eigen::Map<Eigen::VectorXd, 0, Eigen::InnerStride<3>>(val.data() + 2, val.size() / 3), model_.fTime, pathJoint, "rz");

            // 为平移属性(tx, ty, tz)设置关键帧
            val = lt.col(j);
            setKeyframes(Eigen::Map<Eigen::VectorXd, 0, Eigen::InnerStride<3>>(val.data() + 0, val.size() / 3), model_.fTime, pathJoint, "tx");
            setKeyframes(Eigen::Map<Eigen::VectorXd, 0, Eigen::InnerStride<3>>(val.data() + 1, val.size() / 3), model_.fTime, pathJoint, "ty");
            setKeyframes(Eigen::Map<Eigen::VectorXd, 0, Eigen::InnerStride<3>>(val.data() + 2, val.size() / 3), model_.fTime, pathJoint, "tz");
        }
        MGlobal::displayInfo("...关键帧设置完毕。");

        // >>>>>>>>>>>>>>>>>>>> 新增功能：回显设置皮肤簇信息 <<<<<<<<<<<<<<<<<<<<
        MGlobal::displayInfo("正在创建皮肤簇并设置权重...");
        // <<<<<<<<<<<<<<<<<<<<<<<< 功能结束 <<<<<<<<<<<<<<<<<<<<<<<<<<<
        // 创建皮肤簇并设置权重
        status = setSkinCluster(model_.boneName, model_.w, gb);
        CHECK_MSTATUS_AND_RETURN_IT(status);
        MGlobal::displayInfo("...蒙皮设置完毕。");
    }

    // 将新创建的关节名称设置为命令的返回值
    setResult(joints);

    // >>>>>>>>>>>>>>>>>>>> 新增功能：回显命令完成 <<<<<<<<<<<<<<<<<<<<
    MGlobal::displayInfo("--- [DemBones 命令] 执行成功结束 ---");
    // <<<<<<<<<<<<<<<<<<<<<<<< 功能结束 <<<<<<<<<<<<<<<<<<<<<<<<<<<

    return MS::kSuccess;
}

// 辅助函数：为指定节点的指定属性设置动画关键帧
MStatus DemBonesCmd::setKeyframes(const Eigen::VectorXd& val, const Eigen::VectorXd& fTime,
    const MDagPath& pathJoint, const MString& attributeName) {
    MStatus status;
    int nFr = (int)fTime.size(); // 关键帧数量
    MFnDagNode fnNode(pathJoint, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    // 查找属性的 MPlug (属性句柄)
    MPlug plug = fnNode.findPlug(attributeName, false, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    // 创建动画曲线函数集
    MFnAnimCurve fnCurve;
    MObject oCurve = fnCurve.create(plug, nullptr, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    // 准备时间和值数组
    MTime time;
    MTimeArray timeArray(nFr, time);
    MDoubleArray values(nFr);
    for (int i = 0; i < nFr; ++i) {
        timeArray[i].setValue(fTime(i));
        values[i] = val(i);
    }
    // 批量添加关键帧
    status = fnCurve.addKeys(&timeArray, &values);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    return MS::kSuccess;
}

// 辅助函数：创建皮肤簇并设置权重
MStatus DemBonesCmd::setSkinCluster(const std::vector<std::string>& name,
    const Eigen::SparseMatrix<double>& w,
    const Eigen::MatrixXd& gb) {
    MStatus status;

    // 将时间设置回第 0 帧，以在绑定姿态下进行操作
    MTime time(0.0, MTime::kFilm);
    MAnimControl::setCurrentTime(time);

    // 复制一份网格，在副本上进行蒙皮，以保留原始网格
    MStringArray duplicate;
    MGlobal::executeCommand("duplicate -rr " + pathMesh_.partialPathName(), duplicate);

    // 构造 skinCluster MEL 命令。-tsb 表示 "toSelectedBones"
    MString cmd("skinCluster -tsb");
    Eigen::SparseMatrix<double> wT = w.transpose(); // 转置权重矩阵，变为 (顶点数 x 骨骼数)
    int nB = (int)name.size(); // 骨骼数量
    MFnMesh fnMesh(pathMesh_, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    MDoubleArray weights(fnMesh.numVertices() * nB); // 创建 Maya 的权重数组，大小为 (顶点数 * 骨骼数)
    MIntArray influenceIndices; // 影响物体的索引数组

    // 填充命令字符串和影响索引
    for (int i = 0; i < nB; ++i) {
        influenceIndices.append(i);
        cmd += MString(" ") + name[i].c_str();
    }

    // 遍历转置后的权重稀疏矩阵，填充 Maya 的权重数组
    // 外层循环遍历列（即骨骼）
    for (int i = 0; i < wT.outerSize(); ++i) {
        // 内层迭代器遍历该列的非零元素（即该骨骼影响的所有顶点）
        for (Eigen::SparseMatrix<double>::InnerIterator it(wT, i); it; ++it) {
            // Maya 权重数组是扁平化的，按 [v0_b0, v0_b1, ..., v1_b0, v1_b1, ...] 排列
            // it.row() 是顶点索引, i 是骨骼索引, it.value() 是权重值
            weights[it.row() * nB + i] = it.value();
        }
    }

    // >>>>>>>>>>>>>>>>>>>> 新增功能：回显前五个顶点的权重信息 <<<<<<<<<<<<<<<<<<<<
    MGlobal::displayInfo("--- [DemBones] 前5个顶点的权重信息预览 ---");
    // 确定要检查的顶点数量，最多5个，且不超过总顶点数
    int numVertsToCheck = std::min(5, fnMesh.numVertices());
    for (int v_idx = 0; v_idx < numVertsToCheck; ++v_idx) {
        MString info = "顶点 ";
        info += v_idx;
        info += ":";
        bool hasInfluence = false;
        // 遍历所有骨骼，查找对当前顶点有影响的骨骼
        for (int b_idx = 0; b_idx < nB; ++b_idx) {
            // 从转置后的权重矩阵中直接获取权重值 wT(vertex_index, bone_index)
            double weight_val = wT.coeff(v_idx, b_idx);
            // 如果权重值大于一个很小的值（忽略浮点误差），则认为有影响
            if (weight_val > 1e-6) {
                info += " [";
                info += name[b_idx].c_str(); // 骨骼名称
                info += ", ";
                info += weight_val;          // 权重值
                info += "]";
                hasInfluence = true;
            }
        }
        if (!hasInfluence) {
            info += " [未找到有效权重影响]";
        }
        MGlobal::displayInfo(info); // 在脚本编辑器中打印信息
    }
    MGlobal::displayInfo("-------------------------------------------");
    // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> 功能结束 <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    // 将复制的网格名称附加到命令末尾
    cmd += " " + duplicate[0];
    MStringArray result;
    // 执行 skinCluster 命令
    MGlobal::executeCommand(cmd, result);

    // 获取新创建的 skinCluster 节点
    MObject oSkin;
    status = getDependNode(result[0], oSkin);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    // 使用 MFnSkinCluster API 来精确设置权重
    MFnSkinCluster fnSkin(oSkin, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    // 获取与 skinCluster 关联的几何体组件
    MObject oSet = fnSkin.deformerSet();
    MFnSet fnSet(oSet, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    MSelectionList members;
    fnSet.getMembers(members, false);
    MDagPath path;
    MObject components;
    members.getDagPath(0, path, components);

    // 调用 setWeights 函数，一次性设置所有顶点的权重
    fnSkin.setWeights(path, components, influenceIndices, weights, true);

    return MS::kSuccess;
}

// 辅助函数：将 Maya 的 MMatrix 转换为 Eigen 的 Matrix4d
Eigen::Matrix4d DemBonesCmd::toMatrix4d(const MMatrix& m) {
    Eigen::Matrix4d mat;
    // MMatrix 是按行主序存储，Eigen 默认是列主序。
    // 这里按行读取 MMatrix 的元素，并填充到 Eigen 矩阵中。
    mat << m[0][0], m[0][1], m[0][2], m[0][3],
        m[1][0], m[1][1], m[1][2], m[1][3],
        m[2][0], m[2][1], m[2][2], m[2][3],
        m[3][0], m[3][1], m[3][2], m[3][3];
    // 进行一次转置，以匹配 Eigen 的列主序存储，或者说，将行主序矩阵变为列主序矩阵。
    mat.transposeInPlace();
    return mat;
}

// "撤销"函数，用于撤销 doIt/redoIt 的操作
MStatus DemBonesCmd::undoIt() {
    MStatus status;

    // 原始代码使用了 MDGModifier，这里被注释掉了。
    // 完整的撤销逻辑需要删除创建的节点（关节、皮肤簇）、动画曲线和副本网格。
    // 目前的实现是空的，但框架存在。
    /*status = dgMod_.undoIt();
    CHECK_MSTATUS_AND_RETURN_IT(status);
  */
    return MS::kSuccess;
}

// 在文件末尾，如果之前为了包含 MatBlocks.h 定义了宏，现在就取消它
#ifdef DEM_BONES_DEM_BONES_MAT_BLOCKS_UNDEFINED
#undef blk4
#undef rotMat
#undef transVec
#undef vec3
#undef DEM_BONES_MAT_BLOCKS
#endif