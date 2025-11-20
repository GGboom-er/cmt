#include "demBonesCmd.h"

// Keep ASCII-only comments to avoid MSVC C4819 on non-UTF8 builds.

#include "common.h"

#ifndef DEM_BONES_MAT_BLOCKS
  #include "DemBones/MatBlocks.h"
  #define DEM_BONES_DEM_BONES_MAT_BLOCKS_UNDEFINED
#endif

#include <maya/MAnimControl.h>
#include <maya/MDagPath.h>
#include <maya/MEulerRotation.h>
#include <maya/MFnAnimCurve.h>
#include <maya/MFnDagNode.h>
#include <maya/MFnMatrixData.h>
#include <maya/MFnMesh.h>
#include <maya/MFnSet.h>
#include <maya/MFnSkinCluster.h>
#include <maya/MFnTransform.h>
#include <maya/MMatrix.h>
#include <maya/MPlug.h>
#include <maya/MTime.h>
#include <maya/MGlobal.h>
#include <maya/MItDependencyGraph.h>
#include <maya/MFnSingleIndexedComponent.h>
#include <maya/MTransformationMatrix.h>
#include <maya/MProgressWindow.h>
#include <maya/MDGContext.h>
#include <maya/MFloatPointArray.h>
#include <maya/MIntArray.h>
#include <maya/MDoubleArray.h>
#include <maya/MSelectionList.h>
#include <maya/MSyntax.h>
#include <maya/MPxCommand.h>
#include <maya/MString.h>
#include <maya/MStringArray.h>

#include <chrono>
#include <sstream>
#include <iomanip>
#include <map>
#include <vector>
#include <string>
#include <cctype>

using namespace Autodesk::Maya::OpenMaya20250000;
using std::string;

const char* DemBonesCmd::kWeightsSmoothStepShort = "-wss";
const char* DemBonesCmd::kWeightsSmoothStepLong  = "-weightsSmoothStep";
const char* DemBonesCmd::kWeightsSmoothShort     = "-ws";
const char* DemBonesCmd::kWeightsSmoothLong      = "-weightsSmooth";
const char* DemBonesCmd::kNumNonZeroShort        = "-mi";
const char* DemBonesCmd::kNumNonZeroLong         = "-maxInfluences";
const char* DemBonesCmd::kWeightItersShort       = "-wi";
const char* DemBonesCmd::kWeightItersLong        = "-weightIters";
const char* DemBonesCmd::kTransAffineNormShort   = "-tan";
const char* DemBonesCmd::kTransAffineNormLong    = "-transAffineNorm";
const char* DemBonesCmd::kTransAffineShort       = "-ta";
const char* DemBonesCmd::kTransAffineLong        = "-transAffine";
const char* DemBonesCmd::kBindUpdateShort        = "-nu";
const char* DemBonesCmd::kBindUpdateLong         = "-bindUpdate";
const char* DemBonesCmd::kTransItersShort        = "-ti";
const char* DemBonesCmd::kTransItersLong         = "-transIters";
const char* DemBonesCmd::kItersShort             = "-i";
const char* DemBonesCmd::kItersLong              = "-iters";
const char* DemBonesCmd::kInitItersShort         = "-ii";
const char* DemBonesCmd::kInitItersLong          = "-initIters";
const char* DemBonesCmd::kBonesShort             = "-b";
const char* DemBonesCmd::kBonesLong              = "-bones";
const char* DemBonesCmd::kStartFrameShort        = "-sf";
const char* DemBonesCmd::kStartFrameLong         = "-startFrame";
const char* DemBonesCmd::kEndFrameShort          = "-ef";
const char* DemBonesCmd::kEndFrameLong           = "-endFrame";
const char* DemBonesCmd::kExistingBonesShort     = "-eb";
const char* DemBonesCmd::kExistingBonesLong      = "-existingBones";
const char* DemBonesCmd::kSmoothSolverShort      = "-ss";
const char* DemBonesCmd::kSmoothSolverLong       = "-smoothSolver";

const MString DemBonesCmd::kName("demBones");

void* DemBonesCmd::creator() { return new DemBonesCmd; }
bool DemBonesCmd::isUndoable() const { return true; }

// Local helper: ensure we are on a mesh shape (extend transform to shape if needed)
static MStatus getMeshShapeNode_local(MDagPath& path) {
  MStatus status;
  if (path.apiType() == MFn::kMesh) return MS::kSuccess;
  status = path.extendToShape();
  if (MFAIL(status) || path.apiType() != MFn::kMesh) {
    MGlobal::displayError("Invalid selection: " + path.partialPathName() + " is not a mesh or transform of a mesh.");
    return MS::kFailure;
  }
  return MS::kSuccess;
}

// Scoped helpers: reduce UI cost during heavy sampling
struct ScopedRefreshSuspend {
  ScopedRefreshSuspend()  { MGlobal::executeCommand("refresh -suspend true"); }
  ~ScopedRefreshSuspend() { MGlobal::executeCommand("refresh -suspend false"); }
};
struct ScopedUndoOff {
  ScopedUndoOff()  { MGlobal::executeCommand("undoInfo -stateWithoutFlush off"); }
  ~ScopedUndoOff() { MGlobal::executeCommand("undoInfo -stateWithoutFlush on"); }
};

// Fetch a world matrix with an MDGContext time
static MMatrix getWorldMatrixAtTime(const MDagPath& dagPath, const MTime& t) {
  MDagPath p = dagPath;
  if (!p.hasFn(MFn::kTransform)) {
    MDagPath tmp = p; if (tmp.pop() == MS::kSuccess) p = tmp;
  }
  MFnDagNode fnNode(p);
  MStatus st;
  MPlug wmArray = fnNode.findPlug("worldMatrix", true, &st);
  if (MFAIL(st)) return MMatrix::identity;
  MPlug wmElem  = wmArray.elementByLogicalIndex(0, &st);
  if (MFAIL(st)) return MMatrix::identity;
  MDGContext ctx(t);
  MObject mObj = wmElem.asMObject(ctx, &st);
  if (MFAIL(st)) return MMatrix::identity;
  MFnMatrixData fnMat(mObj, &st);
  if (MFAIL(st)) return MMatrix::identity;
  return fnMat.matrix(&st);
}

// Evaluate mesh outMesh in object space at time
static bool getMeshPointsObjectSpaceAtTime(const MDagPath& meshShapePath,
                                           const MTime& t,
                                           MFloatPointArray& outPts) {
  MStatus st;
  MFnDagNode fnShape(meshShapePath, &st);
  if (MFAIL(st)) return false;
  MPlug outMeshPlug = fnShape.findPlug("outMesh", true, &st);
  if (MFAIL(st)) return false;
  MDGContext ctx(t);
  MObject dataObj = outMeshPlug.asMObject(ctx, &st);
  if (MFAIL(st) || dataObj.isNull()) return false;
  MFnMesh fnMeshAtTime(dataObj, &st);
  if (MFAIL(st)) return false;
  st = fnMeshAtTime.getPoints(outPts, MSpace::kObject);
  return st == MS::kSuccess;
}

// One-shot invocation echo to Script Editor / Output
static void logInvocationSummary(const MDagPath& meshPath,
                                 const MDagPathArray& bones,
                                 double startFrame, double endFrame,
                                 const MyDemBones& model) {
  std::ostringstream oss;
  oss << "{"
      << "\"cmd\":\"demBones\","
      << "\"mesh\":\"" << meshPath.partialPathName().asChar() << "\","
      << "\"bonesCount\":" << bones.length() << ","
      << "\"bones\":[";
  for (unsigned int i=0;i<bones.length();++i){
    if (i) oss << ",";
    oss << "\"" << bones[i].partialPathName().asChar() << "\"";
  }
  oss << "],"
      << "\"startFrame\":" << startFrame << ","
      << "\"endFrame\":"   << endFrame   << ","
      << "\"options\":{"
      << "\"iters\":"           << model.nIters        << ","
      << "\"initIters\":"       << model.nInitIters    << ","
      << "\"transIters\":"      << model.nTransIters   << ","
      << "\"weightIters\":"     << model.nWeightsIters << ","
      << "\"bindUpdate\":"      << model.bindUpdate    << ","
      << "\"transAffine\":"     << model.transAffine   << ","
      << "\"transAffineNorm\":" << model.transAffineNorm << ","
      << "\"maxInfluences\":"   << model.nnz           << ","
      << "\"weightsSmooth\":"   << model.weightsSmooth << ","
      << "\"weightsSmoothStep\":"<< model.weightsSmoothStep << ","
      << "\"smoothSolverPolicy\":"<< model.smoothSolverPolicy
      << "}}";
  MGlobal::displayInfo(MString(oss.str().c_str()));
}

// -----------------------------------------------------------------------------

MSyntax DemBonesCmd::newSyntax() {
  MSyntax syntax;
  syntax.addFlag(kWeightsSmoothStepShort, kWeightsSmoothStepLong, MSyntax::kDouble);
  syntax.addFlag(kWeightsSmoothShort,     kWeightsSmoothLong,     MSyntax::kDouble);
  syntax.addFlag(kNumNonZeroShort,        kNumNonZeroLong,        MSyntax::kLong);
  syntax.addFlag(kWeightItersShort,       kWeightItersLong,       MSyntax::kLong);
  syntax.addFlag(kTransAffineNormShort,   kTransAffineNormLong,   MSyntax::kDouble);
  syntax.addFlag(kTransAffineShort,       kTransAffineLong,       MSyntax::kDouble);
  syntax.addFlag(kBindUpdateShort,        kBindUpdateLong,        MSyntax::kBoolean);
  syntax.addFlag(kTransItersShort,        kTransItersLong,        MSyntax::kLong);
  syntax.addFlag(kItersShort,             kItersLong,             MSyntax::kLong);
  syntax.addFlag(kInitItersShort,         kInitItersLong,         MSyntax::kLong);
  syntax.addFlag(kBonesShort,             kBonesLong,             MSyntax::kLong);
  syntax.addFlag(kStartFrameShort,        kStartFrameLong,        MSyntax::kDouble);
  syntax.addFlag(kEndFrameShort,          kEndFrameLong,          MSyntax::kDouble);
  syntax.addFlag(kExistingBonesShort,     kExistingBonesLong,     MSyntax::kString);
  syntax.makeFlagMultiUse(kExistingBonesShort); // <- multi-use flag for -eb  ✅  :contentReference[oaicite:3]{index=3}
  // New: smoothing solver policy ("auto" | "ldlt" | "lu")
  syntax.addFlag(kSmoothSolverShort,      kSmoothSolverLong,      MSyntax::kString);

  syntax.setObjectType(MSyntax::kSelectionList, 1, 1);
  syntax.useSelectionAsDefault(true);
  syntax.enableEdit(false);
  syntax.enableQuery(false);
  return syntax;
}

// Find upstream skinCluster of a mesh
static MObject findSkinCluster(const MDagPath& meshPath) {
  MStatus status;
  MObject meshNode = meshPath.node();
  MItDependencyGraph it(meshNode, MFn::kSkinClusterFilter,
                        MItDependencyGraph::kUpstream,
                        MItDependencyGraph::kBreadthFirst,
                        MItDependencyGraph::kNodeLevel, &status);
  if (MFAIL(status)) return MObject::kNullObj;
  if (!it.isDone()) return it.currentItem();
  return MObject::kNullObj;
}

#define CHECK_MSTATUS_AND_GOTO_CLEANUP(status) \
  if (MFAIL(status)) { MGlobal::displayError(status.errorString()); goto cleanup; }

MStatus DemBonesCmd::doIt(const MArgList& argList) {
  MStatus status;
  auto startTime = std::chrono::high_resolution_clock::now();

  // Parse args and selection
  MArgDatabase argData(syntax(), argList);
  MSelectionList sel;
  argData.getObjects(sel);
  if (sel.length() != 1) {
    MGlobal::displayError("Select exactly one mesh.");
    return MS::kInvalidParameter;
  }
  sel.getDagPath(0, pathMesh_);
  CHECK_MSTATUS_AND_RETURN_IT(getMeshShapeNode_local(pathMesh_));

  // Multiple -eb allowed (Python list expands into multi-use)  :contentReference[oaicite:4]{index=4}
  pathBones_.clear();
  for (unsigned int useIdx = 0;; ++useIdx) {   // <- 从索引0开始线性读取  ✅  :contentReference[oaicite:5]{index=5}
    MString jname;
    if (argData.getFlagArgument(kExistingBonesShort, useIdx, jname) != MS::kSuccess) break;
    MDagPath p;
    if (MFAIL(getDagPath(jname, p))) {
      MGlobal::displayWarning("Invalid -existingBones name: " + jname);
      continue;
    }
    pathBones_.append(p);
  }

  // Frame range
  double startFrame = MAnimControl::minTime().value();
  double endFrame   = MAnimControl::maxTime().value();
  if (argData.isFlagSet(kStartFrameShort)) argData.getFlagArgument(kStartFrameShort, 0, startFrame);
  if (argData.isFlagSet(kEndFrameShort))   argData.getFlagArgument(kEndFrameShort,   0, endFrame);
  if (endFrame < startFrame) std::swap(startFrame, endFrame);

  // Model defaults
  model_.nIters        = 30;
  model_.nInitIters    = 10;
  model_.nTransIters   = 5;
  model_.nWeightsIters = 3;
  model_.bindUpdate    = 1;
  model_.transAffine   = 10.0;
  model_.transAffineNorm = 4.0;
  model_.nnz           = 8;
  model_.weightsSmooth = 1e-4;
  model_.weightsSmoothStep = 1.0;
  model_.smoothSolverPolicy = 0; // 0=Auto,1=LDLT,2=LU

  // Override by flags
  if (argData.isFlagSet(kItersShort))            argData.getFlagArgument(kItersShort,        0, model_.nIters);
  if (argData.isFlagSet(kInitItersShort))        argData.getFlagArgument(kInitItersShort,    0, model_.nInitIters);
  if (argData.isFlagSet(kTransItersShort))       argData.getFlagArgument(kTransItersShort,   0, model_.nTransIters);
  if (argData.isFlagSet(kWeightItersShort))      argData.getFlagArgument(kWeightItersShort,  0, model_.nWeightsIters);
  if (argData.isFlagSet(kBindUpdateShort))      { bool b=false; argData.getFlagArgument(kBindUpdateShort, 0, b); model_.bindUpdate = int(b); }
  if (argData.isFlagSet(kTransAffineShort))      argData.getFlagArgument(kTransAffineShort,  0, model_.transAffine);
  if (argData.isFlagSet(kTransAffineNormShort))  argData.getFlagArgument(kTransAffineNormShort, 0, model_.transAffineNorm);
  if (argData.isFlagSet(kNumNonZeroShort))       argData.getFlagArgument(kNumNonZeroShort,   0, model_.nnz);
  if (argData.isFlagSet(kWeightsSmoothShort))    argData.getFlagArgument(kWeightsSmoothShort,     0, model_.weightsSmooth);
  if (argData.isFlagSet(kWeightsSmoothStepShort))argData.getFlagArgument(kWeightsSmoothStepShort, 0, model_.weightsSmoothStep);
  if (argData.isFlagSet(kSmoothSolverShort)) {
    MString v; argData.getFlagArgument(kSmoothSolverShort, 0, v);
    string sv = v.asChar();
    for (auto& c: sv) c = (char)std::tolower((unsigned char)c);
    if (sv == "ldlt") model_.smoothSolverPolicy = 1;
    else if (sv == "lu") model_.smoothSolverPolicy = 2;
    else model_.smoothSolverPolicy = 0;
  }

  // Echo parameters once (shows *initial* bones coming from -eb)
  logInvocationSummary(pathMesh_, pathBones_, startFrame, endFrame, model_);

  // Progress protocol: frames + global iters + 1
  const int numFrames = int(endFrame - startFrame + 1.0);
  const int totalSteps = numFrames + model_.nIters + 1;

  MProgressWindow::reserve();
  MProgressWindow::setTitle("Dem Bones Skinning Decomposition");
  MProgressWindow::setInterruptable(true);
  MProgressWindow::setProgressRange(0, totalSteps);
  MProgressWindow::setProgress(0);
  MProgressWindow::startProgress();

  // Data sampling
  MProgressWindow::setProgressStatus("Reading mesh sequence...");
  status = readMeshSequence(startFrame, endFrame);
  if (MProgressWindow::isCancelled()) { MGlobal::displayInfo("Aborted during data extraction."); goto cleanup; }
  CHECK_MSTATUS_AND_GOTO_CLEANUP(status);

  status = readBindPose();
  CHECK_MSTATUS_AND_GOTO_CLEANUP(status);

  // Bones count extend if requested  —— “增加骨骼(Increment)” 语义
  int requestedAdditionalBones = 0;
  if (argData.isFlagSet(kBonesShort)) argData.getFlagArgument(kBonesShort, 0, requestedAdditionalBones);
  if (requestedAdditionalBones < 0) requestedAdditionalBones = 0; // clamp
  if (model_.nB == 0) {
    if (requestedAdditionalBones == 0) {
      MGlobal::displayError("No joints found and -b/-bones not set or 0.");
      status = MS::kInvalidParameter; goto cleanup;
    }
    model_.nB = requestedAdditionalBones;          // no eb -> nB = b
  } else {
    model_.nB += requestedAdditionalBones;         // with eb -> nB = len(eb) + b
  }

  // Compute
  MProgressWindow::setProgressStatus("Computing Skinning Decomposition...");
  {
    const bool ok = model_.compute();
    if (MProgressWindow::isCancelled()) {
      MGlobal::displayInfo("Computation interrupted.");
      if (!ok) { status = MS::kFailure; goto cleanup; }
    } else if (!ok) {
      MGlobal::displayError("Skinning decomposition computation failed.");
      status = MS::kFailure; goto cleanup;
    }
  }

  // Apply results
  MProgressWindow::setProgressStatus("Applying results (joints & skinCluster)...");
  status = redoIt();
  MProgressWindow::setProgress(totalSteps);

cleanup:
  MProgressWindow::endProgress();

  if (MFAIL(status)) return status;

  {
    auto endTime = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = endTime - startTime;
    std::stringstream ss;
    ss << "Dem Bones finished in " << std::fixed << std::setprecision(3) << elapsed.count() << " seconds.";
    ss << (MProgressWindow::isCancelled() ? " (Interrupted)" : " (Completed)");
    MGlobal::displayInfo(ss.str().c_str());
  }
  return MS::kSuccess;
}

// Per-frame sampling via MDGContext (no global time changes)
MStatus DemBonesCmd::readMeshSequence(double startFrame, double endFrame) {
  MStatus status;
  model_.nS = 1;
  model_.nF = int(endFrame - startFrame + 1.0);

  // Mesh basics
  MFnMesh fnMeshNow(pathMesh_, &status);
  CHECK_MSTATUS_AND_RETURN_IT(status);
  model_.nV = fnMeshNow.numVertices();
  model_.v.resize(3 * model_.nF, model_.nV);
  model_.fTime.resize(model_.nF);
  model_.fStart.resize(model_.nS + 1);
  model_.fStart(0) = 0;

  // Initial bones (if provided)
  model_.nB = pathBones_.length();
  if (model_.nB > 0) model_.m.resize(model_.nF * 4, model_.nB * 4);
  else               model_.m.resize(0, 0);

  // Bind info captured at sequence start
  if (model_.nB > 0) {
    model_.boneName.resize(model_.nB);
    for (unsigned int i = 0; i < model_.nB; ++i) model_.boneName[i] = pathBones_[i].partialPathName().asChar();
    model_.parent.resize(model_.nB);
    model_.bind.resize(model_.nS * 4, model_.nB * 4);
    model_.preMulInv.resize(model_.nS * 4, model_.nB * 4);
    model_.rotOrder.resize(model_.nS * 3, model_.nB);

    const int s = 0;
    const MTime tBind(startFrame);
    for (int j = 0; j < model_.nB; ++j) {
      // parent
      model_.parent(j) = -1;
      MDagPath parent(pathBones_[j]);
      if (parent.pop() == MS::kSuccess && parent.isValid()) {
        MString parentName = parent.partialPathName();
        for (int k = 0; k < model_.nB; ++k)
          if (model_.boneName[k] == parentName.asChar()) { model_.parent(j) = k; break; }
      }
      // bind matrix
      MMatrix wBind = getWorldMatrixAtTime(pathBones_[j], tBind);
      model_.bind.blk4(s, j) = toMatrix4d(wBind);

      // rotation order
      MFnTransform fnTr(pathBones_[j]);
      switch (fnTr.rotationOrder()) {
        case MTransformationMatrix::kXYZ: model_.rotOrder.vec3(s,j)=Eigen::Vector3i(0,1,2); break;
        case MTransformationMatrix::kYZX: model_.rotOrder.vec3(s,j)=Eigen::Vector3i(1,2,0); break;
        case MTransformationMatrix::kZXY: model_.rotOrder.vec3(s,j)=Eigen::Vector3i(2,0,1); break;
        case MTransformationMatrix::kXZY: model_.rotOrder.vec3(s,j)=Eigen::Vector3i(0,2,1); break;
        case MTransformationMatrix::kYXZ: model_.rotOrder.vec3(s,j)=Eigen::Vector3i(1,0,2); break;
        case MTransformationMatrix::kZYX: model_.rotOrder.vec3(s,j)=Eigen::Vector3i(2,1,0); break;
        default:                           model_.rotOrder.vec3(s,j)=Eigen::Vector3i(0,1,2); break;
      }
      model_.preMulInv.blk4(s, j) = toMatrix4d(MMatrix());
    }

    // Optional: warm start from existing skinCluster (placeholder)
    MObject skin = findSkinCluster(pathMesh_);
    (void)skin;
  }

  if (model_.w.size() == 0) model_.w.resize(0, 0);

  // Per-frame sampling
  MDagPath meshShape = pathMesh_;
  CHECK_MSTATUS_AND_RETURN_IT(getMeshShapeNode_local(meshShape));

  ScopedRefreshSuspend _sr;
  ScopedUndoOff _su;

  for (int s = 0; s < model_.nS; ++s) {
    const int start = model_.fStart(s);
    for (int f = 0; f < model_.nF; ++f) {
      if (MProgressWindow::isCancelled()) break;

      const double frame = startFrame + double(f);
      const MTime t(frame);
      model_.fTime(start + f) = frame;

      {
        std::ostringstream oss; oss << "Extracting frame " << (f+1) << "/" << model_.nF;
        MProgressWindow::setProgressStatus(MString(oss.str().c_str()));
        MProgressWindow::advanceProgress(1);
      }

      // Object-space points then world-space via worldMatrix(ctx)
      MFloatPointArray ptsObj;
      if (!getMeshPointsObjectSpaceAtTime(meshShape, t, ptsObj)) {
        MGlobal::displayError("Failed to evaluate mesh at frame: " + MString(std::to_string(frame).c_str()));
        return MS::kFailure;
      }
      const MMatrix wMat = getWorldMatrixAtTime(meshShape, t);

      #pragma omp parallel for
      for (int i = 0; i < model_.nV; ++i) {
        const MPoint pWs = MPoint(ptsObj[i]) * wMat;
        model_.v.col(i).segment<3>((start + f) * 3) << (double)pWs.x, (double)pWs.y, (double)pWs.z;
      }

      // Bone relative matrices: world(ctx) * bind^{-1}
      const int nInitB = (int)pathBones_.length();
      if (nInitB > 0) {
        for (int j = 0; j < nInitB; ++j) {
          const MMatrix wBj = getWorldMatrixAtTime(pathBones_[j], t);
          model_.m.blk4(f, j) = toMatrix4d(wBj) * model_.bind.blk4(s, j).inverse();
        }
      }
    }
    if (MProgressWindow::isCancelled()) break;
    model_.fStart(s + 1) = model_.fStart(s) + model_.nF;
  }

  // Save original m (only initial bones)
  const int initialBoneCount = (int)pathBones_.length();
  if (initialBoneCount > 0 && model_.m.size() > 0) {
    model_.origM = model_.m.leftCols(initialBoneCount * 4);
  }

  // subjectID map
  model_.subjectID.resize(model_.nF);
  for (int ss = 0; ss < model_.nS; ++ss)
    for (int k = model_.fStart(ss); k < model_.fStart(ss+1); ++k)
      model_.subjectID(k) = ss;

  return MS::kSuccess;
}

MStatus DemBonesCmd::readBindPose() {
  MStatus status;
  const MTime t0(0.0);

  MDagPath meshShape = pathMesh_;
  CHECK_MSTATUS_AND_RETURN_IT(getMeshShapeNode_local(meshShape));

  MFloatPointArray ptsObj;
  if (!getMeshPointsObjectSpaceAtTime(meshShape, t0, ptsObj)) return MS::kFailure;
  const MMatrix wMat = getWorldMatrixAtTime(meshShape, t0);

  model_.u.resize(model_.nS * 3, model_.nV);
  if (model_.nS > 0) {
    Eigen::MatrixXd v(3, model_.nV);
    #pragma omp parallel for
    for (int i = 0; i < model_.nV; ++i) {
      const MPoint pWs = MPoint(ptsObj[i]) * wMat;
      v.col(i) << (double)pWs.x, (double)pWs.y, (double)pWs.z;
    }
    model_.u.block(0, 0, 3, model_.nV) = v;
  }

  // Topology (once)
  MFnMesh fnMeshNow(meshShape, &status);
  CHECK_MSTATUS_AND_RETURN_IT(status);
  const int numPolys = fnMeshNow.numPolygons();
  model_.fv.resize(numPolys);
  MIntArray vtxList;
  for (int i = 0; i < numPolys; ++i) {
    fnMeshNow.getPolygonVertices(i, vtxList);
    model_.fv[i].resize(vtxList.length());
    for (unsigned int j = 0; j < vtxList.length(); ++j) model_.fv[i][j] = vtxList[j];
  }
  return MS::kSuccess;
}

MStatus DemBonesCmd::redoIt() {
  MStatus status;
  clearResult();

  const int initialBoneCount = (int)pathBones_.length();
  const int creationCount    = model_.nB - initialBoneCount;

  std::vector<std::string> newBoneNames;
  MStringArray createdJoints;

  // Align boneName size
  if ((int)model_.boneName.size() != model_.nB) {
    std::vector<std::string> tmp = model_.boneName;
    model_.boneName.resize(model_.nB);
    for (size_t i=0;i<tmp.size() && i<(size_t)model_.nB;++i) model_.boneName[i] = tmp[i];
  }

  // Create additional joints if needed
  if (creationCount > 0) {
    for (int j = 0; j < creationCount; ++j) {
      const int idx = initialBoneCount + j;
      std::ostringstream s; s << "dembones_joint" << idx;
      const std::string boneName = s.str();
      model_.boneName[idx] = boneName;
      newBoneNames.push_back(boneName);
      createdJoints.append(boneName.c_str());
    }
  } else if (creationCount < 0) {
    MGlobal::displayWarning("The number of solved bones is less than the initial bones provided.");
  }

  if (model_.nS == 0 || model_.nF == 0) {
    MGlobal::displayError("Cannot apply results: No subjects or frames processed.");
    return MS::kFailure;
  }

  for (int s = 0; s < model_.nS; ++s) {
    Eigen::MatrixXd lr, lt, gb, lbr, lbt;
    model_.computeRTB(s, lr, lt, gb, lbr, lbt, false);

    for (const auto& nameStr : newBoneNames) {
      MGlobal::executeCommand("createNode \"joint\" -n \"" + MString(nameStr.c_str()) + "\"");
    }

    for (int j = 0; j < model_.nB; ++j) {
      MDagPath pathJoint;
      if (MFAIL(getDagPath(model_.boneName[j].c_str(), pathJoint))) continue;

      // Rotation X/Y/Z curves
      Eigen::VectorXd rot_val = lr.col(j);
      setKeyframes(Eigen::Map<Eigen::VectorXd,0,Eigen::InnerStride<3>>(rot_val.data()+0, rot_val.size()/3), model_.fTime, pathJoint, "rotateX");
      setKeyframes(Eigen::Map<Eigen::VectorXd,0,Eigen::InnerStride<3>>(rot_val.data()+1, rot_val.size()/3), model_.fTime, pathJoint, "rotateY");
      setKeyframes(Eigen::Map<Eigen::VectorXd,0,Eigen::InnerStride<3>>(rot_val.data()+2, rot_val.size()/3), model_.fTime, pathJoint, "rotateZ");

      // Translation X/Y/Z curves
      Eigen::VectorXd t_val = lt.col(j);
      setKeyframes(Eigen::Map<Eigen::VectorXd,0,Eigen::InnerStride<3>>(t_val.data()+0, t_val.size()/3), model_.fTime, pathJoint, "translateX");
      setKeyframes(Eigen::Map<Eigen::VectorXd,0,Eigen::InnerStride<3>>(t_val.data()+1, t_val.size()/3), model_.fTime, pathJoint, "translateY");
      setKeyframes(Eigen::Map<Eigen::VectorXd,0,Eigen::InnerStride<3>>(t_val.data()+2, t_val.size()/3), model_.fTime, pathJoint, "translateZ");
    }

    status = setSkinCluster(model_.boneName, model_.w, gb);
    CHECK_MSTATUS_AND_RETURN_IT(status);
  }

  setResult(createdJoints);
  return MS::kSuccess;
}

MStatus DemBonesCmd::setKeyframes(const Eigen::VectorXd& val,
                                  const Eigen::VectorXd& fTime,
                                  const MDagPath& pathJoint,
                                  const MString& attributeName) {
  MStatus status;
  const int nFr = (int)fTime.size();
  if (nFr == 0) return MS::kSuccess;

  MFnDagNode fnNode(pathJoint);
  MPlug plug = fnNode.findPlug(attributeName, false, &status);
  CHECK_MSTATUS_AND_RETURN_IT(status);

  MFnAnimCurve fnCurve;
  fnCurve.create(plug, nullptr, &status);
  CHECK_MSTATUS_AND_RETURN_IT(status);

  MTimeArray tArr; tArr.setLength((unsigned)nFr);
  MDoubleArray vArr; vArr.setLength((unsigned)nFr);

  MTime t;
  for (int i = 0; i < nFr; ++i) {
    t.setValue(fTime(i));
    tArr.set(t, i);
    vArr.set(i < val.size() ? val(i) : (val.size()>0 ? val(val.size()-1) : 0.0), i);
  }
  status = fnCurve.addKeys(&tArr, &vArr);
  CHECK_MSTATUS_AND_RETURN_IT(status);
  return MS::kSuccess;
}

MStatus DemBonesCmd::setSkinCluster(const std::vector<std::string>& name,
                                    const Eigen::SparseMatrix<double>& w,
                                    const Eigen::MatrixXd& gb) {
  (void)gb; // gb can be used to set bind pre-matrices if desired
  MStatus status;
  MTime time0(0.0);
  MAnimControl::setCurrentTime(time0);

  // Duplicate mesh and create skinCluster
  MStringArray dup;
  MGlobal::executeCommand("duplicate -rr " + pathMesh_.partialPathName(), dup);
  if (dup.length() == 0) {
    MGlobal::displayError("Failed to duplicate mesh.");
    return MS::kFailure;
  }

  MString cmd("skinCluster -tsb");
  const int nB = (int)name.size();
  const int nV = model_.nV;
  if (w.rows() != nB || w.cols() != nV) {
    MGlobal::displayError("Weight matrix dims mismatch.");
    MGlobal::executeCommand("delete \"" + dup[0] + "\"");
    return MS::kFailure;
  }

  MDoubleArray weights((unsigned)(nV * nB), 0.0);
  MIntArray inflIdx; inflIdx.setLength((unsigned)nB);
  for (int i = 0; i < nB; ++i) {
    inflIdx.set(i, i);
    cmd += MString(" \"") + name[i].c_str() + "\"";
  }
  cmd += " \"" + dup[0] + "\"";

  MStringArray res;
  status = MGlobal::executeCommand(cmd, res);
  if (MFAIL(status) || res.length() == 0) {
    MGlobal::displayError("Failed to create skinCluster. Command: " + cmd);
    MGlobal::executeCommand("delete \"" + dup[0] + "\"");
    return MS::kFailure;
  }

  // Flatten sparse weights (vertex-major)
  for (int v = 0; v < w.outerSize(); ++v) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(w, v); it; ++it) {
      weights[v * nB + (int)it.row()] = it.value();
    }
  }

  MObject oSkin;
  getDependNode(res[0], oSkin);
  MFnSkinCluster fnSkin(oSkin);
  MDagPath dupPath;
  getDagPath(dup[0], dupPath);
  getMeshShapeNode_local(dupPath);

  MFnSingleIndexedComponent fnComp;
  MObject components = fnComp.create(MFn::kMeshVertComponent);
  fnComp.setComplete(true);

  fnSkin.setWeights(dupPath, components, inflIdx, weights, true);
  return MS::kSuccess;
}

Eigen::Matrix4d DemBonesCmd::toMatrix4d(const MMatrix& m) {
  return Eigen::Map<const Eigen::Matrix<double,4,4,Eigen::RowMajor>>(m[0]);
}

MStatus DemBonesCmd::undoIt() { return MS::kSuccess; }

#ifdef DEM_BONES_DEM_BONES_MAT_BLOCKS_UNDEFINED
  #undef blk4
  #undef rotMat
  #undef transVec
  #undef vec3
  #undef DEM_BONES_MAT_BLOCKS
#endif
