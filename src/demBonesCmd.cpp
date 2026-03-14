#include "demBonesCmd.h"
#include "demBonesCache.h"

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
#include <unordered_map>
#include <vector>
#include <string>
#include <cctype>
#include <omp.h>

using namespace Autodesk::Maya::OpenMaya20250000;

// RAII wrapper for MProgressWindow to ensure cleanup on exceptions
struct ScopedProgressWindow {
    bool reserved_;
    ScopedProgressWindow(const MString& title, int range) : reserved_(false) {
        if (MProgressWindow::reserve()) {
            reserved_ = true;
            MProgressWindow::setTitle(title);
            MProgressWindow::setProgressRange(0, range);
            MProgressWindow::startProgress();
        }
    }
    ~ScopedProgressWindow() {
        if (reserved_) {
            MProgressWindow::endProgress();
        }
    }
    bool isReserved() const { return reserved_; }
};

// Escape special characters in MEL command strings to prevent injection
static MString escapeMelString(const MString& input) {
    MString result;
    for (unsigned int i = 0; i < input.length(); ++i) {
        char c = input.asChar()[i];
        if (c == '"' || c == '\\') {
            result += "\\";
        }
        result += c;
    }
    return result;
}
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
const char* DemBonesCmd::kBindFrameShort         = "-bf";
const char* DemBonesCmd::kBindFrameLong          = "-bindFrame";
const char* DemBonesCmd::kUseExistingWeightsShort = "-uw";
const char* DemBonesCmd::kUseExistingWeightsLong  = "-useWeights";
const char* DemBonesCmd::kLockBonesShort         = "-lb";
const char* DemBonesCmd::kLockBonesLong          = "-lockBones";
const char* DemBonesCmd::kWeightsOnlyShort       = "-wo";
const char* DemBonesCmd::kWeightsOnlyLong        = "-weightsOnly";
const char* DemBonesCmd::kDeformThresholdShort   = "-dt";
const char* DemBonesCmd::kDeformThresholdLong    = "-deformThreshold";
const char* DemBonesCmd::kTotalBonesShort        = "-tb";
const char* DemBonesCmd::kTotalBonesLong         = "-totalBones";
const char* DemBonesCmd::kOutputMeshShort        = "-om";
const char* DemBonesCmd::kOutputMeshLong         = "-outputMesh";

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
                                 double bindFrame, bool useExistingWeights,
                                 bool weightsOnly,
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
      << "\"bindFrame\":"  << bindFrame  << ","
      << "\"useExistingWeights\":" << (useExistingWeights ? "true" : "false") << ","
      << "\"weightsOnly\":" << (weightsOnly ? "true" : "false") << ","
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
  syntax.makeFlagMultiUse(kExistingBonesShort);
  // Smoothing solver policy ("auto" | "ldlt" | "lu")
  syntax.addFlag(kSmoothSolverShort,      kSmoothSolverLong,      MSyntax::kString);
  // Bind pose frame (default: startFrame)
  syntax.addFlag(kBindFrameShort,         kBindFrameLong,         MSyntax::kDouble);
  // Use existing skinCluster weights as warm start
  syntax.addFlag(kUseExistingWeightsShort, kUseExistingWeightsLong, MSyntax::kBoolean);
  // Lock specific bones - their weights will not be modified (multi-use)
  syntax.addFlag(kLockBonesShort,         kLockBonesLong,         MSyntax::kString);
  syntax.makeFlagMultiUse(kLockBonesShort);
  // Weights-only mode: only solve weights, keep bone transforms from animation
  syntax.addFlag(kWeightsOnlyShort,       kWeightsOnlyLong,       MSyntax::kBoolean);
  // Deform threshold: vertices with max displacement below this are locked (0 = disabled)
  syntax.addFlag(kDeformThresholdShort,   kDeformThresholdLong,   MSyntax::kDouble);
  // Total bones: target total bone count for incremental mode (auto-discovers existing dembones_joint*)
  syntax.addFlag(kTotalBonesShort,        kTotalBonesLong,        MSyntax::kLong);
  // Output mesh name: if specified, update existing mesh instead of creating new one
  syntax.addFlag(kOutputMeshShort,        kOutputMeshLong,        MSyntax::kString);

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

  // Bind frame (defaults to startFrame)
  bindFrame_ = startFrame;
  if (argData.isFlagSet(kBindFrameShort)) argData.getFlagArgument(kBindFrameShort, 0, bindFrame_);

  // Use existing weights from skinCluster
  useExistingWeights_ = false;
  if (argData.isFlagSet(kUseExistingWeightsShort)) argData.getFlagArgument(kUseExistingWeightsShort, 0, useExistingWeights_);

  // Weights-only mode
  weightsOnly_ = false;
  if (argData.isFlagSet(kWeightsOnlyShort)) argData.getFlagArgument(kWeightsOnlyShort, 0, weightsOnly_);

  // Deform threshold for static vertex detection (0 = disabled)
  deformThreshold_ = 0.0;
  if (argData.isFlagSet(kDeformThresholdShort)) argData.getFlagArgument(kDeformThresholdShort, 0, deformThreshold_);

  // Output mesh name: if specified, will update existing mesh or create with this name
  outputMeshName_ = "";
  if (argData.isFlagSet(kOutputMeshShort)) argData.getFlagArgument(kOutputMeshShort, 0, outputMeshName_);

  // Parse bone count parameters early
  // -b N : Create N new bones from scratch (ignore any existing bones)
  // -tb N: Incremental mode - reuse existing dembones_joint* and their animations/weights,
  //        then add more bones to reach total of N
  int requestedNewBones = 0;      // From -b flag: create this many NEW bones
  int requestedTotalBones = 0;    // From -tb flag: target TOTAL bone count
  bool useIncrementalMode = false;

  if (argData.isFlagSet(kBonesShort)) {
    argData.getFlagArgument(kBonesShort, 0, requestedNewBones);
    if (requestedNewBones < 0) requestedNewBones = 0;
  }
  if (argData.isFlagSet(kTotalBonesShort)) {
    argData.getFlagArgument(kTotalBonesShort, 0, requestedTotalBones);
    if (requestedTotalBones < 0) requestedTotalBones = 0;
    useIncrementalMode = true;
  }

  // -tb (totalBones) incremental mode:
  // 1. Auto-discover existing dembones_joint* in scene
  // 2. Reuse their animation data
  // 3. Later: read existing skinCluster weights as warm start
  // 4. Add new bones to reach the target total
  int existingBoneCount = 0;
  if (useIncrementalMode && pathBones_.length() == 0) {
    MStringArray existingJoints;
    // Search for existing dembones_joint* in scene
    for (int idx = 0; idx < 1000; ++idx) {
      std::ostringstream ss;
      ss << "dembones_joint" << idx;
      MString jointName(ss.str().c_str());
      MDagPath jointPath;
      if (getDagPath(jointName, jointPath) == MS::kSuccess) {
        pathBones_.append(jointPath);
        existingJoints.append(jointName);
      } else {
        break;  // Sequential - stop at first missing
      }
    }

    existingBoneCount = (int)existingJoints.length();
    if (existingBoneCount > 0) {
      std::ostringstream oss;
      oss << "Incremental mode: Found " << existingBoneCount << " existing dembones joints";
      MGlobal::displayInfo(MString(oss.str().c_str()));

      // Enable reading existing weights for warm start
      useExistingWeights_ = true;

      // Calculate additional bones needed
      requestedNewBones = std::max(0, requestedTotalBones - existingBoneCount);

      oss.str("");
      oss << "Target: " << requestedTotalBones << " total, existing: " << existingBoneCount
          << ", will create: " << requestedNewBones << " additional bones";
      MGlobal::displayInfo(MString(oss.str().c_str()));
    } else {
      // No existing bones, -tb acts like -b
      requestedNewBones = requestedTotalBones;
      useIncrementalMode = false;
      MGlobal::displayInfo(MString("No existing dembones joints found. Creating all from scratch."));
    }
  }

  // For -b mode: ignore existing bones, pathBones_ should be empty (unless -eb was used)
  if (!useIncrementalMode && requestedNewBones > 0 && pathBones_.length() == 0) {
    // Pure -b mode: no existing bones to reuse
    MGlobal::displayInfo(MString("Creating new bones from scratch."));
  }

  // -eb + -b combination: existing bones from -eb plus new bones from -b
  // This is similar to incremental mode - we should preserve existing bone weights
  const int ebBoneCount = (int)pathBones_.length();
  if (!useIncrementalMode && ebBoneCount > 0 && requestedNewBones > 0) {
    // Mark as incremental-like mode for proper handling
    existingBoneCount = ebBoneCount;
    // Auto-enable reading existing weights if not explicitly disabled
    if (!argData.isFlagSet(kUseExistingWeightsShort)) {
      useExistingWeights_ = true;
      MGlobal::displayInfo(MString("Auto-enabled weight reading for -eb + -b mode."));
    }
    std::ostringstream oss;
    oss << "Adding " << requestedNewBones << " new bones to " << ebBoneCount << " existing bones from -eb";
    MGlobal::displayInfo(MString(oss.str().c_str()));
  }

  // Parse locked bones from -lb flags (multi-use) - for future use
  // NOTE: In the new incremental mode, we don't lock bones by default
  // We let DemBones re-optimize all bones with the existing transforms as warm start
  lockedBoneIndices_.clear();
  for (unsigned int useIdx = 0;; ++useIdx) {
    MString boneName;
    if (argData.getFlagArgument(kLockBonesShort, useIdx, boneName) != MS::kSuccess) break;
    for (unsigned int i = 0; i < pathBones_.length(); ++i) {
      if (pathBones_[i].partialPathName() == boneName ||
          pathBones_[i].fullPathName() == boneName) {
        lockedBoneIndices_.insert((int)i);
        break;
      }
    }
  }

  // Model defaults - will be adjusted for incremental mode
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
  logInvocationSummary(pathMesh_, pathBones_, startFrame, endFrame, bindFrame_,
                       useExistingWeights_, weightsOnly_, model_);

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
  status = readMeshSequence(startFrame, endFrame, bindFrame_);
  if (MProgressWindow::isCancelled()) { MGlobal::displayInfo("Aborted during data extraction."); goto cleanup; }
  CHECK_MSTATUS_AND_GOTO_CLEANUP(status);

  status = readBindPose(bindFrame_);
  CHECK_MSTATUS_AND_GOTO_CLEANUP(status);

  // Set final bone count and resize matrices for additional bones FIRST
  // This must happen before reading weights to ensure proper matrix dimensions
  // -b N: requestedNewBones = N, create N new bones from scratch
  // -tb N: requestedNewBones = max(0, N - existingBoneCount), add bones to reach N total
  {
    const int initialBoneCount = model_.nB;  // Bones read from pathBones_
    int finalBoneCount = initialBoneCount;

    if (initialBoneCount == 0) {
      if (requestedNewBones == 0 && requestedTotalBones == 0) {
        MGlobal::displayError("No joints found and -b/-bones or -tb/-totalBones not set.");
        status = MS::kInvalidParameter; goto cleanup;
      }
      // No existing bones: create requestedNewBones (from -b) or requestedTotalBones (from -tb)
      finalBoneCount = requestedNewBones > 0 ? requestedNewBones : requestedTotalBones;
    } else {
      // Have existing bones: add requestedNewBones on top
      finalBoneCount = initialBoneCount + requestedNewBones;
    }

    // INCREMENTAL MODE: When adding new bones to existing ones
    // Strategy: Preserve existing bone transforms, let DemBones reinitialize all weights
    // This allows the algorithm to redistribute weights across all bones optimally
    if (finalBoneCount > initialBoneCount && initialBoneCount > 0) {
      std::ostringstream oss;
      oss << "Incremental mode: " << initialBoneCount << " existing bones + "
          << (finalBoneCount - initialBoneCount) << " new bones = " << finalBoneCount << " total";
      MGlobal::displayInfo(MString(oss.str().c_str()));

      // Resize m matrix to accommodate new bones
      // Existing bones keep their transforms, new bones need initialization
      Eigen::MatrixXd newM(model_.nF * 4, finalBoneCount * 4);

      // Copy existing bone transforms
      if (model_.m.size() > 0) {
        newM.block(0, 0, model_.m.rows(), model_.m.cols()) = model_.m;
      }

      // Initialize new bone transforms from existing bones
      // The actual transform values will be computed by DemBones optimization
      // This just provides a reasonable starting point (not identity at origin)
      for (int f = 0; f < model_.nF; ++f) {
        for (int newJ = initialBoneCount; newJ < finalBoneCount; ++newJ) {
          // Copy transform from an existing bone (cycle through them)
          int srcBone = (newJ - initialBoneCount) % initialBoneCount;
          newM.block(f * 4, newJ * 4, 4, 4) = model_.m.block(f * 4, srcBone * 4, 4, 4);
        }
      }
      model_.m = newM;

      // Resize related matrices
      model_.boneName.resize(finalBoneCount);
      model_.parent.conservativeResize(finalBoneCount);
      for (int j = initialBoneCount; j < finalBoneCount; ++j) {
        model_.parent(j) = -1;
      }

      // Resize bind matrix - copy from existing bones
      Eigen::MatrixXd newBind(model_.nS * 4, finalBoneCount * 4);
      if (model_.bind.size() > 0) {
        newBind.block(0, 0, model_.bind.rows(), model_.bind.cols()) = model_.bind;
      }
      for (int newJ = initialBoneCount; newJ < finalBoneCount; ++newJ) {
        int srcBone = (newJ - initialBoneCount) % initialBoneCount;
        for (int s = 0; s < model_.nS; ++s) {
          newBind.block(s * 4, newJ * 4, 4, 4) = model_.bind.block(s * 4, srcBone * 4, 4, 4);
        }
      }
      model_.bind = newBind;

      // Resize preMulInv
      Eigen::MatrixXd newPreMulInv = Eigen::MatrixXd::Identity(4, 4).replicate(model_.nS, finalBoneCount);
      if (model_.preMulInv.size() > 0) {
        newPreMulInv.block(0, 0, model_.preMulInv.rows(), model_.preMulInv.cols()) = model_.preMulInv;
      }
      model_.preMulInv = newPreMulInv;

      // Resize rotOrder
      Eigen::MatrixXi newRotOrder = Eigen::MatrixXi::Zero(model_.nS * 3, finalBoneCount);
      if (model_.rotOrder.size() > 0) {
        newRotOrder.block(0, 0, model_.rotOrder.rows(), model_.rotOrder.cols()) = model_.rotOrder;
      }
      for (int j = initialBoneCount; j < finalBoneCount; ++j) {
        for (int s = 0; s < model_.nS; ++s) {
          newRotOrder.block(s * 3, j, 3, 1) << 0, 1, 2;
        }
      }
      model_.rotOrder = newRotOrder;

      // IMPORTANT: Instead of clearing weights completely, we preserve existing bone weights
      // and let initNewBonesFromResidual() add initial weights for new bones.
      // Clearing weights would lose all existing bone information which breaks incremental mode.

      // Save existing bone count for initNewBonesFromResidual()
      model_.existingBoneCount = initialBoneCount;

      MGlobal::displayInfo("Incremental mode: Preserving existing bone weights, will initialize new bone weights");
    }

    model_.nB = finalBoneCount;
    if (model_.existingBoneCount == 0) {
      model_.existingBoneCount = initialBoneCount;
    }
  }

  // Read existing weights AFTER setting final bone count
  // This ensures the weight matrix dimensions are consistent with nB
  if (useExistingWeights_) {
    MProgressWindow::setProgressStatus("Reading existing weights...");
    status = readExistingWeights();
    if (MFAIL(status)) {
      MGlobal::displayWarning("Could not read existing weights, will initialize from scratch.");
      // Clear weight matrix to trigger full initialization
      model_.w.resize(0, 0);
    } else {
      // Expand weight matrix to accommodate new bones if needed
      if (model_.w.rows() < model_.nB) {
        std::ostringstream oss;
        oss << "Expanding weight matrix from " << model_.w.rows() << " to " << model_.nB << " bones";
        MGlobal::displayInfo(MString(oss.str().c_str()));

        // Create expanded weight matrix with existing weights preserved
        std::vector<Eigen::Triplet<double>> triplets;
        triplets.reserve(model_.w.nonZeros());

        for (int k = 0; k < model_.w.outerSize(); ++k) {
          for (Eigen::SparseMatrix<double>::InnerIterator it(model_.w, k); it; ++it) {
            triplets.emplace_back(it.row(), it.col(), it.value());
          }
        }

        Eigen::SparseMatrix<double> expandedW(model_.nB, model_.nV);
        expandedW.setFromTriplets(triplets.begin(), triplets.end());
        model_.w = expandedW;

        MGlobal::displayInfo(MString("Weight matrix expanded - existing bone weights preserved"));
      }
    }
  }

  // In incremental mode, we don't lock bones - let DemBones optimize everything
  // The existing bone transforms provide a good starting point
  model_.lockedBones.clear();
  model_.lockedVertices.clear();

  // Detect static vertices based on deformation threshold
  // Vertices with max displacement below threshold will keep their original weights
  model_.lockedVertices.clear();
  if (deformThreshold_ > 0.0 && model_.nV > 0 && model_.nF > 1) {
    MProgressWindow::setProgressStatus("Detecting static vertices...");

    // Use squared threshold to avoid sqrt in inner loop
    const double thresholdSq = deformThreshold_ * deformThreshold_;

    // Thread-local collection to avoid critical section
    std::vector<std::vector<int>> threadLocalLocked(omp_get_max_threads());

    #pragma omp parallel
    {
      int tid = omp_get_thread_num();
      std::vector<int>& localLocked = threadLocalLocked[tid];

      #pragma omp for nowait
      for (int i = 0; i < model_.nV; ++i) {
        // Get bind pose position (from u matrix, subject 0)
        Eigen::Vector3d bindPos = model_.u.col(i).segment<3>(0);

        // Check if any frame exceeds threshold (early exit)
        bool isStatic = true;
        for (int k = 0; k < model_.nF && isStatic; ++k) {
          Eigen::Vector3d framePos = model_.v.col(i).segment<3>(k * 3).template cast<double>();
          double dispSq = (framePos - bindPos).squaredNorm();
          if (dispSq >= thresholdSq) {
            isStatic = false;
          }
        }

        if (isStatic) {
          localLocked.push_back(i);
        }
      }
    }

    // Merge thread-local results (single-threaded, but fast)
    for (const auto& localVec : threadLocalLocked) {
      for (int idx : localVec) {
        model_.lockedVertices.insert(idx);
      }
    }

    int lockedCount = (int)model_.lockedVertices.size();
    std::ostringstream oss;
    oss << "Static vertices detected: " << lockedCount << " / " << model_.nV
        << " (threshold: " << deformThreshold_ << ")";
    MGlobal::displayInfo(MString(oss.str().c_str()));
  }

  // Compute
  MProgressWindow::setProgressStatus("Computing Skinning Decomposition...");
  {
    // In weights-only mode, skip transform iterations by setting nTransIters=0
    if (weightsOnly_) {
      if (model_.m.rows() == 0 || model_.m.cols() == 0) {
        MGlobal::displayError("Weights-only mode requires existing bone transforms.");
        status = MS::kFailure; goto cleanup;
      }
      model_.nTransIters = 0;
      model_.nInitIters = 0;
    }

    // INCREMENTAL MODE: Residual-focused optimization with aggressive parameters
    bool isIncrementalMode = (model_.existingBoneCount > 0 && model_.existingBoneCount < model_.nB);

    if (isIncrementalMode) {
      const int newBoneCount = model_.nB - model_.existingBoneCount;
      const double boneFactor = newBoneCount / double(model_.existingBoneCount);

      std::ostringstream modeLog;
      modeLog << "【增量模式激活】: " << model_.existingBoneCount << " → " << model_.nB
              << " 根骨骼 (+" << newBoneCount << ", " << std::fixed << std::setprecision(1) << (boneFactor*100) << "%)";
      MGlobal::displayInfo(MString(modeLog.str().c_str()));

      // 激进的参数调整用于增量优化
      // 新骨骼必须与已优化的现有骨骼竞争
      // 策略：高迭代数 + 低平滑度 + 高稀疏性

      // 保存原始迭代数以便报告
      int origIters = model_.nIters;

      // 根据骨骼增加比例调整迭代数
      // 更多骨骼 = 需要更多迭代才能收敛
      if (boneFactor > 3.0) {  // 大幅增加 (>300%)
        model_.nIters = std::max(150, model_.nIters * 5);
        model_.nInitIters = std::max(20, model_.nInitIters * 2);
        model_.nWeightsIters = std::max(6, model_.nWeightsIters * 2);
        model_.nTransIters = std::max(8, model_.nTransIters * 2);
        model_.weightsSmooth = 1e-6;  // Very low smoothness for flexibility
        MGlobal::displayInfo("  Large increase (>300%) - Ultra-aggressive parameters:");
      } else if (boneFactor > 1.0) {  // Medium increase (100-300%)
        model_.nIters = std::max(100, model_.nIters * 3);
        model_.nInitIters = std::max(15, model_.nInitIters * 2);
        model_.nWeightsIters = std::max(5, model_.nWeightsIters * 2);
        model_.nTransIters = std::max(6, model_.nTransIters);
        model_.weightsSmooth = 5e-6;  // Low smoothness
        MGlobal::displayInfo("  Medium increase (100-300%) - Aggressive parameters:");
      } else {  // Small increase (<100%)
        model_.nIters = std::max(80, model_.nIters * 2);
        model_.nInitIters = std::max(12, model_.nInitIters);
        model_.nWeightsIters = std::max(4, model_.nWeightsIters);
        model_.nTransIters = std::max(5, model_.nTransIters);
        model_.weightsSmooth = 1e-5;
        MGlobal::displayInfo("  Small increase (<100%) - Enhanced parameters:");
      }

      // Increase sparsity limit to allow more bone influences per vertex
      int oldNnz = model_.nnz;
      model_.nnz = std::min(model_.nB, std::max(model_.nnz, 16));

      std::ostringstream paramLog;
      paramLog << "    Iterations: " << origIters << " -> " << model_.nIters
               << " (init=" << model_.nInitIters
               << ", trans=" << model_.nTransIters
               << ", weights=" << model_.nWeightsIters << ")\n"
               << "    Weight smoothness: " << std::scientific << model_.weightsSmooth << "\n"
               << "    Max influences: " << oldNnz << " -> " << model_.nnz;
      MGlobal::displayInfo(MString(paramLog.str().c_str()));
    }

    // Log matrix dimensions and weight statistics before optimization
    {
      std::ostringstream oss;
      oss << "Before optimization: nB=" << model_.nB
          << ", m=[" << model_.m.rows() << "x" << model_.m.cols() << "]"
          << ", w=[" << model_.w.rows() << "x" << model_.w.cols() << "]"
          << ", nF=" << model_.nF << ", nV=" << model_.nV;
      MGlobal::displayInfo(MString(oss.str().c_str()));

      // Per-bone weight statistics (before optimization)
      if (model_.w.rows() > 0 && model_.w.cols() > 0) {
        std::ostringstream wStats;
        wStats << "Initial weight statistics per bone:\n";
        for (int b = 0; b < std::min(model_.nB, 10); ++b) {  // Show first 10 bones
          int nonZeroCount = 0;
          double maxWeight = 0.0;
          for (int v = 0; v < model_.nV; ++v) {
            double w = model_.w.coeff(b, v);
            if (w > 1e-6) {
              nonZeroCount++;
              if (w > maxWeight) maxWeight = w;
            }
          }
          wStats << "  Bone " << b << ": " << nonZeroCount << " verts, max=" << std::fixed << std::setprecision(4) << maxWeight;
          if (b < model_.existingBoneCount) wStats << " (existing)";
          else wStats << " (new)";
          wStats << "\n";
        }
        if (model_.nB > 10) {
          wStats << "  ... (" << (model_.nB - 10) << " more bones)\n";
        }
        MGlobal::displayInfo(MString(wStats.str().c_str()));
      }
    }

    // CRITICAL: 根据模式选择优化方法
    bool ok = false;
    if (isIncrementalMode) {
      // 增量模式：使用残差拟合防止新骨骼被淘汰
      MGlobal::displayInfo("\n========================================");
      MGlobal::displayInfo(">>> Using RESIDUAL-BASED METHOD for incremental optimization <<<");
      MGlobal::displayInfo("========================================\n");
      MProgressWindow::setProgressStatus("Residual-based mode: computing residuals...");
      ok = model_.fitNewBonesToResidual();
    } else {
      // 标准模式：从零开始的标准DemBones优化
      MGlobal::displayInfo("\n========================================");
      MGlobal::displayInfo(">>> Using STANDARD OPTIMIZATION METHOD <<<");
      MGlobal::displayInfo("========================================\n");
      MProgressWindow::setProgressStatus("Computing skinning decomposition...");
      ok = model_.compute();
    }
    if (MProgressWindow::isCancelled()) {
      MGlobal::displayInfo("Computation interrupted.");
      if (!ok) { status = MS::kFailure; goto cleanup; }
    } else if (!ok) {
      MGlobal::displayError("Skinning decomposition computation failed.");
      status = MS::kFailure; goto cleanup;
    }

    // Compute and report reconstruction error (RMSE and per-vertex max error)
    double rmse = model_.rmse();

    // Compute per-vertex max error for quality assessment
    double maxError = 0.0;
    int worstVertex = -1;
    #pragma omp parallel
    {
      double localMax = 0.0;
      int localWorst = -1;
      #pragma omp for nowait
      for (int i = 0; i < model_.nV; ++i) {
        double vertexMaxErr = 0.0;
        Eigen::Matrix4d mki;
        for (int k = 0; k < model_.nF; ++k) {
          mki.setZero();
          for (Eigen::SparseMatrix<double>::InnerIterator it(model_.w, i); it; ++it) {
            mki += it.value() * model_.m.block(k * 4, (int)it.row() * 4, 4, 4);
          }
          Eigen::Vector3d reconstructed = mki.topLeftCorner<3, 3>() * model_.u.col(i).segment<3>(0)
                                        + mki.topRightCorner<3, 1>();
          Eigen::Vector3d original = model_.v.col(i).segment<3>(k * 3).template cast<double>();
          double err = (reconstructed - original).norm();
          if (err > vertexMaxErr) vertexMaxErr = err;
        }
        if (vertexMaxErr > localMax) {
          localMax = vertexMaxErr;
          localWorst = i;
        }
      }
      #pragma omp critical
      {
        if (localMax > maxError) {
          maxError = localMax;
          worstVertex = localWorst;
        }
      }
    }

    std::ostringstream errOss;
    errOss << "Reconstruction quality: RMSE=" << std::fixed << std::setprecision(6) << rmse
           << ", MaxError=" << std::setprecision(4) << maxError;
    if (worstVertex >= 0) {
      errOss << " (vertex " << worstVertex << ")";
    }
    errOss << ", Bones=" << model_.nB;
    MGlobal::displayInfo(MString(errOss.str().c_str()));

    // Analyze convergence quality for incremental mode
    if (isIncrementalMode) {
      std::ostringstream convLog;
      convLog << "\nIncremental Mode Analysis:";

      // Assess RMSE quality
      if (rmse < 0.001) {
        convLog << "\n  ✓ Excellent convergence (RMSE < 0.001)";
      } else if (rmse < 0.01) {
        convLog << "\n  ○ Good convergence (RMSE < 0.01), consider more bones for better quality";
      } else if (rmse < 0.1) {
        convLog << "\n  △ Moderate convergence (RMSE < 0.1), strongly recommend adding more bones";
      } else {
        convLog << "\n  ✗ Poor convergence (RMSE >= 0.1), optimization may have failed";
        convLog << "\n    Try: (1) Use -tb to add more bones iteratively";
        convLog << "\n         (2) Increase -i (iterations) to 200+";
        convLog << "\n         (3) Check if mesh has extreme deformations";
      }

      // Weight distribution analysis - detailed per-bone statistics
      int newBonesWithWeight = 0;
      int newBonesEmpty = 0;
      double totalNewBoneWeight = 0.0;

      std::ostringstream newBoneDetails;
      newBoneDetails << "\n  New bone details (showing all):";

      for (int b = model_.existingBoneCount; b < model_.nB; ++b) {
        int vtxCount = 0;
        double boneWeight = 0.0;
        double maxW = 0.0;
        int maxWVertex = -1;

        for (int v = 0; v < model_.nV; ++v) {
          double w = model_.w.coeff(b, v);
          if (w > 1e-6) {
            vtxCount++;
            boneWeight += w;
            if (w > maxW) {
              maxW = w;
              maxWVertex = v;
            }
          }
        }

        if (vtxCount > 0) {
          newBonesWithWeight++;
          totalNewBoneWeight += boneWeight;
          newBoneDetails << "\n    Bone " << b << " (new" << (b - model_.existingBoneCount) << "): "
                        << vtxCount << " verts, total=" << std::fixed << std::setprecision(2) << boneWeight
                        << ", max=" << std::setprecision(4) << maxW << " @ vtx" << maxWVertex;
        } else {
          newBonesEmpty++;
          newBoneDetails << "\n    Bone " << b << " (new" << (b - model_.existingBoneCount) << "): UNUSED";
        }
      }

      const int newBoneCount = model_.nB - model_.existingBoneCount;
      convLog << "\n  New bone utilization: " << newBonesWithWeight << "/" << newBoneCount << " active";
      if (newBonesEmpty > 0) {
        convLog << " (" << newBonesEmpty << " unused)";
      }
      convLog << "\n  Total weight captured by new bones: " << std::fixed << std::setprecision(1) << totalNewBoneWeight;
      convLog << newBoneDetails.str();  // Append detailed per-bone info

      if (newBonesEmpty > newBoneCount / 2) {
        convLog << "\n  ⚠ Warning: Most new bones are unused. Try reducing bone count or increasing mesh complexity.";
      } else if (newBonesWithWeight == newBoneCount) {
        convLog << "\n  ✓ All new bones are being utilized!";
      }

      MGlobal::displayInfo(MString(convLog.str().c_str()));
    } else {
      // Provide guidance for initial mode
      if (maxError > 0.1) {
        MGlobal::displayInfo(MString("Tip: High reconstruction error. Consider using -tb to add more bones."));
      } else if (rmse < 0.001) {
        MGlobal::displayInfo(MString("Excellent quality achieved!"));
      }
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
MStatus DemBonesCmd::readMeshSequence(double startFrame, double endFrame, double bindFrame) {
  MStatus status;
  model_.nS = 1;
  model_.nF = int(endFrame - startFrame + 1.0);

  // Check if we can use cached data
  DemBonesCache& cache = DemBonesCache::instance();
  bool useCache = false;
  if (cache.numVertices() > 0 &&
      cache.meshName() == pathMesh_.fullPathName().asChar() &&
      cache.startFrame() == startFrame &&
      cache.endFrame() == endFrame) {
    useCache = true;
    MGlobal::displayInfo("Using cached mesh data (faster execution)");
  }

  // Mesh basics
  MFnMesh fnMeshNow(pathMesh_, &status);
  CHECK_MSTATUS_AND_RETURN_IT(status);
  model_.nV = fnMeshNow.numVertices();

  // Verify cache vertex count matches
  if (useCache && cache.numVertices() != model_.nV) {
    MGlobal::displayWarning("Cache vertex count mismatch, falling back to live sampling");
    useCache = false;
  }

  // Memory allocation sanity check
  const size_t estimatedSize = static_cast<size_t>(model_.nF) * 3 * model_.nV * sizeof(float);
  if (estimatedSize > 4ULL * 1024 * 1024 * 1024) {  // 4GB warning threshold
    std::ostringstream warn;
    warn << "Warning: Large memory allocation requested (~" << (estimatedSize / (1024*1024)) << " MB) "
         << "for " << model_.nF << " frames x " << model_.nV << " vertices";
    MGlobal::displayWarning(MString(warn.str().c_str()));
  }
  model_.v.resize(3 * model_.nF, model_.nV);
  model_.fTime.resize(model_.nF);
  model_.fStart.resize(model_.nS + 1);
  model_.fStart(0) = 0;

  // Initial bones (if provided)
  model_.nB = pathBones_.length();
  if (model_.nB > 0) model_.m.resize(model_.nF * 4, model_.nB * 4);
  else               model_.m.resize(0, 0);

  // Bind info captured at bind frame (not necessarily startFrame)
  if (model_.nB > 0) {
    model_.boneName.resize(model_.nB);
    for (unsigned int i = 0; i < model_.nB; ++i) model_.boneName[i] = pathBones_[i].partialPathName().asChar();
    model_.parent.resize(model_.nB);
    model_.bind.resize(model_.nS * 4, model_.nB * 4);
    model_.preMulInv.resize(model_.nS * 4, model_.nB * 4);
    model_.rotOrder.resize(model_.nS * 3, model_.nB);

    const int s = 0;
    const MTime tBind(bindFrame);  // Use bindFrame instead of startFrame
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

  // Per-frame sampling (or use cache if available)
  MDagPath meshShape = pathMesh_;
  CHECK_MSTATUS_AND_RETURN_IT(getMeshShapeNode_local(meshShape));

  ScopedRefreshSuspend _sr;
  ScopedUndoOff _su;

  if (useCache) {
    // Fast path: copy vertex data from cache
    MGlobal::displayInfo("Copying vertex data from cache...");
    model_.v = cache.vertices().cast<float>();

    // Still need to sample bone transforms (usually fast)
    for (int s = 0; s < model_.nS; ++s) {
      const int start = model_.fStart(s);
      for (int f = 0; f < model_.nF; ++f) {
        const double frame = startFrame + double(f);
        const MTime t(frame);
        model_.fTime(start + f) = frame;

        // Bone relative matrices: world(ctx) * bind^{-1}
        const int nInitB = (int)pathBones_.length();
        if (nInitB > 0) {
          for (int j = 0; j < nInitB; ++j) {
            const MMatrix wBj = getWorldMatrixAtTime(pathBones_[j], t);
            model_.m.blk4(f, j) = toMatrix4d(wBj) * model_.bind.blk4(s, j).inverse();
          }
        }
      }
      model_.fStart(s + 1) = model_.fStart(s) + model_.nF;
    }
  } else {
    // Standard path: sample from Maya scene
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

  // ========== 数据验证输出 ==========
  MGlobal::displayInfo("\n========== Data Verification ==========");

  // Verify v matrix (animated vertex data)
  if (model_.v.size() > 0) {
    double vMin = model_.v.minCoeff();
    double vMax = model_.v.maxCoeff();
    double vMean = model_.v.mean();
    std::ostringstream vStats;
    vStats << "Animated vertex matrix v: [" << model_.v.rows() << " x " << model_.v.cols() << "]\n"
           << "  - Value range: [" << std::fixed << std::setprecision(4) << vMin << ", " << vMax << "]\n"
           << "  - Mean: " << vMean << "\n"
           << "  - Frames: " << model_.nF << ", Vertices: " << model_.nV;
    MGlobal::displayInfo(MString(vStats.str().c_str()));

    // Sample first and last frame positions for vertex 0
    if (model_.nF > 0 && model_.nV > 0) {
      Eigen::Vector3d firstFrame = model_.v.col(0).segment<3>(0).template cast<double>();
      Eigen::Vector3d lastFrame = model_.v.col(0).segment<3>((model_.nF - 1) * 3).template cast<double>();
      std::ostringstream sampleLog;
      sampleLog << "  - Sample vertex 0: frame 1 (" << std::fixed << std::setprecision(4)
                << firstFrame.x() << ", " << firstFrame.y() << ", " << firstFrame.z() << ") "
                << "-> frame " << model_.nF << " ("
                << lastFrame.x() << ", " << lastFrame.y() << ", " << lastFrame.z() << ")";
      MGlobal::displayInfo(MString(sampleLog.str().c_str()));
    }
  }

  // Verify m matrix (existing bone transforms)
  if (initialBoneCount > 0 && model_.m.size() > 0) {
    std::ostringstream mStats;
    mStats << "Existing bone transform matrix m: [" << model_.m.rows() << " x " << model_.m.cols() << "]\n"
           << "  - Initial bone count: " << initialBoneCount << "\n"
           << "  - 4 columns per bone (4x4 matrix)\n"
           << "  - Total rows = frames x 4 = " << model_.nF << " x 4 = " << model_.m.rows();
    MGlobal::displayInfo(MString(mStats.str().c_str()));

    // Sample first bone's first frame transform
    if (model_.m.rows() >= 4 && model_.m.cols() >= 4) {
      Eigen::Matrix4d firstBoneFirstFrame = model_.m.block<4, 4>(0, 0);
      std::ostringstream matLog;
      matLog << "  - Sample bone 0 frame 1 transform:\n";
      for (int r = 0; r < 4; ++r) {
        matLog << "    [";
        for (int c = 0; c < 4; ++c) {
          matLog << std::fixed << std::setprecision(4) << std::setw(8) << firstBoneFirstFrame(r, c);
          if (c < 3) matLog << ", ";
        }
        matLog << "]\n";
      }
      MGlobal::displayInfo(MString(matLog.str().c_str()));
    }
  }

  MGlobal::displayInfo("========================================\n");

  return MS::kSuccess;
}

MStatus DemBonesCmd::readBindPose(double bindFrame) {
  MStatus status;
  const MTime tBind(bindFrame);  // Use bindFrame instead of hardcoded 0.0

  MDagPath meshShape = pathMesh_;
  CHECK_MSTATUS_AND_RETURN_IT(getMeshShapeNode_local(meshShape));

  MFloatPointArray ptsObj;
  if (!getMeshPointsObjectSpaceAtTime(meshShape, tBind, ptsObj)) return MS::kFailure;
  const MMatrix wMat = getWorldMatrixAtTime(meshShape, tBind);

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

  // ========== Bind Pose verification ==========
  MGlobal::displayInfo("\n========== Bind Pose Loaded - Data Verification ==========");

  // Verify u matrix (bind pose vertex data)
  if (model_.u.size() > 0) {
    double uMin = model_.u.minCoeff();
    double uMax = model_.u.maxCoeff();
    double uMean = model_.u.mean();
    std::ostringstream uStats;
    uStats << "Bind pose matrix u: [" << model_.u.rows() << " x " << model_.u.cols() << "]\n"
           << "  - Value range: [" << std::fixed << std::setprecision(4) << uMin << ", " << uMax << "]\n"
           << "  - Mean: " << uMean << "\n"
           << "  - Vertices: " << model_.nV << ", Bind frame: " << bindFrame;
    MGlobal::displayInfo(MString(uStats.str().c_str()));

    // Sample first 3 vertex positions
    if (model_.nV >= 3) {
      std::ostringstream sampleLog;
      sampleLog << "  - Sample vertex positions:\n";
      for (int i = 0; i < 3; ++i) {
        Eigen::Vector3d pos = model_.u.col(i).segment<3>(0);
        sampleLog << "    Vertex " << i << ": (" << std::fixed << std::setprecision(4)
                  << pos.x() << ", " << pos.y() << ", " << pos.z() << ")\n";
      }
      MGlobal::displayInfo(MString(sampleLog.str().c_str()));
    }
  }

  // Verify mesh topology
  std::ostringstream topoLog;
  topoLog << "Mesh topology: " << numPolys << " polygons";
  MGlobal::displayInfo(MString(topoLog.str().c_str()));

  MGlobal::displayInfo("========================================\n");

  return MS::kSuccess;
}

// Read existing skinCluster weights for warm start
MStatus DemBonesCmd::readExistingWeights() {
  MStatus status;

  // Determine which mesh to read weights from:
  // - If -om (outputMesh) is specified, try to read from that mesh first
  //   (this is the typical incremental workflow where weights are on the output mesh)
  // - Otherwise fall back to the input mesh (pathMesh_)
  MDagPath weightSourceMesh = pathMesh_;
  MObject skinObj;

  if (outputMeshName_.length() > 0) {
    MDagPath outputPath;
    if (getDagPath(outputMeshName_, outputPath) == MS::kSuccess) {
      // Ensure we have the shape node, not the transform
      if (getMeshShapeNode_local(outputPath) == MS::kSuccess) {
        MObject outputSkin = findSkinCluster(outputPath);
        if (!outputSkin.isNull()) {
          weightSourceMesh = outputPath;
          skinObj = outputSkin;
          MGlobal::displayInfo(MString("Reading existing weights from output mesh: ") + outputMeshName_);
        } else {
          MGlobal::displayInfo(MString("Output mesh found but has no skinCluster: ") + outputMeshName_);
        }
      } else {
        MGlobal::displayWarning(MString("Could not get shape node for output mesh: ") + outputMeshName_);
      }
    } else {
      MGlobal::displayInfo(MString("Output mesh not found in scene: ") + outputMeshName_);
    }
  }

  // If we didn't find a skinCluster on the output mesh, try the input mesh
  if (skinObj.isNull()) {
    MDagPath inputMeshShape = pathMesh_;
    if (getMeshShapeNode_local(inputMeshShape) == MS::kSuccess) {
      skinObj = findSkinCluster(inputMeshShape);
      if (!skinObj.isNull()) {
        weightSourceMesh = inputMeshShape;
        MGlobal::displayInfo(MString("Reading existing weights from input mesh."));
      }
    }
  }

  if (skinObj.isNull()) {
    MGlobal::displayWarning("No skinCluster found on mesh for warm start.");
    return MS::kFailure;
  }

  MFnSkinCluster fnSkin(skinObj, &status);
  CHECK_MSTATUS_AND_RETURN_IT(status);

  // Get influence objects (joints) from skinCluster
  MDagPathArray influences;
  unsigned int numInfluences = fnSkin.influenceObjects(influences, &status);
  CHECK_MSTATUS_AND_RETURN_IT(status);

  if (numInfluences == 0) {
    MGlobal::displayWarning("SkinCluster has no influences.");
    return MS::kFailure;
  }

  // Build hash map for O(1) bone name lookup (instead of O(n*m) nested loop)
  std::unordered_map<std::string, int> boneNameToIndex;
  boneNameToIndex.reserve(model_.nB);
  for (int j = 0; j < model_.nB; ++j) {
    boneNameToIndex[model_.boneName[j]] = j;
  }

  // Build mapping from skinCluster influence index to our bone index
  std::vector<int> influenceToModelBone(numInfluences, -1);
  for (unsigned int i = 0; i < numInfluences; ++i) {
    std::string influenceName = influences[i].partialPathName().asChar();
    auto it = boneNameToIndex.find(influenceName);
    if (it != boneNameToIndex.end()) {
      influenceToModelBone[i] = it->second;
    }
  }

  // Get all vertex weights from the weight source mesh
  MDagPath meshShape = weightSourceMesh;
  CHECK_MSTATUS_AND_RETURN_IT(getMeshShapeNode_local(meshShape));

  MFnSingleIndexedComponent fnComp;
  MObject allVerts = fnComp.create(MFn::kMeshVertComponent);
  fnComp.setCompleteData(model_.nV);

  MDoubleArray weights;
  unsigned int numInfluencesOut;
  status = fnSkin.getWeights(meshShape, allVerts, weights, numInfluencesOut);
  CHECK_MSTATUS_AND_RETURN_IT(status);

  // Build sparse weight matrix
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(model_.nV * model_.nnz);

  for (int v = 0; v < model_.nV; ++v) {
    for (unsigned int i = 0; i < numInfluences; ++i) {
      double w = weights[v * numInfluences + i];
      if (w > 1e-6) {  // Only store non-zero weights
        int boneIdx = influenceToModelBone[i];
        if (boneIdx >= 0 && boneIdx < model_.nB) {
          triplets.emplace_back(boneIdx, v, w);
        }
      }
    }
  }

  // Set the weights matrix
  model_.w.resize(model_.nB, model_.nV);
  model_.w.setFromTriplets(triplets.begin(), triplets.end());

  MGlobal::displayInfo(MString("Loaded existing weights: ") +
                       std::to_string(triplets.size()).c_str() + " non-zero entries.");

  return MS::kSuccess;
}

MStatus DemBonesCmd::redoIt() {
  MStatus status;
  clearResult();

  // Clear undo data
  createdJoints_.clear();
  createdMeshName_ = "";
  createdSkinClusterName_ = "";

  const int initialBoneCount = (int)pathBones_.length();
  const int creationCount    = model_.nB - initialBoneCount;

  std::vector<std::string> newBoneNames;

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
      createdJoints_.append(boneName.c_str());
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

    // Create new joints and set their initial positions from bind pose
    for (size_t idx = 0; idx < newBoneNames.size(); ++idx) {
      const auto& nameStr = newBoneNames[idx];
      const int boneIdx = initialBoneCount + idx;

      // Create joint
      MGlobal::executeCommand("createNode \"joint\" -n \"" + MString(nameStr.c_str()) + "\"");

      // Set initial position from bind pose translation
      // Extract bind pose translation for this bone
      Eigen::Vector3d bindTranslation = lbt.col(boneIdx);

      // Set the joint's translate attributes at bind frame
      MString setTransCmd;
      setTransCmd.format("setAttr \"^1s.translate\" ^2s ^3s ^4s;",
                        MString(nameStr.c_str()),
                        MString(std::to_string(bindTranslation(0)).c_str()),
                        MString(std::to_string(bindTranslation(1)).c_str()),
                        MString(std::to_string(bindTranslation(2)).c_str()));
      MGlobal::executeCommand(setTransCmd);

      // Also set rotation from bind pose
      Eigen::Vector3d bindRotation = lbr.col(boneIdx);
      MString setRotCmd;
      setRotCmd.format("setAttr \"^1s.rotate\" ^2s ^3s ^4s;",
                      MString(nameStr.c_str()),
                      MString(std::to_string(bindRotation(0)).c_str()),
                      MString(std::to_string(bindRotation(1)).c_str()),
                      MString(std::to_string(bindRotation(2)).c_str()));
      MGlobal::executeCommand(setRotCmd);
    }

    for (int j = 0; j < model_.nB; ++j) {
      MDagPath pathJoint;
      if (MFAIL(getDagPath(model_.boneName[j].c_str(), pathJoint))) continue;

      // Rotation X/Y/Z curves
      Eigen::VectorXd rot_val = lr.col(j);
      if (rot_val.size() % 3 != 0) {
        MGlobal::displayWarning(MString("Rotation data size not divisible by 3 for bone ") + j);
      }
      const int nRotFrames = static_cast<int>(rot_val.size() / 3);
      setKeyframes(Eigen::Map<Eigen::VectorXd,0,Eigen::InnerStride<3>>(rot_val.data()+0, nRotFrames), model_.fTime, pathJoint, "rotateX");
      setKeyframes(Eigen::Map<Eigen::VectorXd,0,Eigen::InnerStride<3>>(rot_val.data()+1, nRotFrames), model_.fTime, pathJoint, "rotateY");
      setKeyframes(Eigen::Map<Eigen::VectorXd,0,Eigen::InnerStride<3>>(rot_val.data()+2, nRotFrames), model_.fTime, pathJoint, "rotateZ");

      // Translation X/Y/Z curves
      Eigen::VectorXd t_val = lt.col(j);
      if (t_val.size() % 3 != 0) {
        MGlobal::displayWarning(MString("Translation data size not divisible by 3 for bone ") + j);
      }
      const int nTransFrames = static_cast<int>(t_val.size() / 3);
      setKeyframes(Eigen::Map<Eigen::VectorXd,0,Eigen::InnerStride<3>>(t_val.data()+0, nTransFrames), model_.fTime, pathJoint, "translateX");
      setKeyframes(Eigen::Map<Eigen::VectorXd,0,Eigen::InnerStride<3>>(t_val.data()+1, nTransFrames), model_.fTime, pathJoint, "translateY");
      setKeyframes(Eigen::Map<Eigen::VectorXd,0,Eigen::InnerStride<3>>(t_val.data()+2, nTransFrames), model_.fTime, pathJoint, "translateZ");
    }

    status = setSkinCluster(model_.boneName, model_.w, gb);
    CHECK_MSTATUS_AND_RETURN_IT(status);
  }

  setResult(createdJoints_);
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

  const int nB = (int)name.size();
  const int nV = model_.nV;
  if (w.rows() != nB || w.cols() != nV) {
    MGlobal::displayError("Weight matrix dims mismatch.");
    return MS::kFailure;
  }

  MString targetMeshName;
  MDagPath targetMeshPath;
  MObject existingSkinCluster;
  bool updateExisting = false;

  // Check if we should update an existing mesh or create a new one
  if (outputMeshName_.length() > 0) {
    // Try to find existing mesh with this name
    if (getDagPath(outputMeshName_, targetMeshPath) == MS::kSuccess) {
      // Mesh exists - check if it has a skinCluster
      existingSkinCluster = findSkinCluster(targetMeshPath);
      if (!existingSkinCluster.isNull()) {
        updateExisting = true;
        targetMeshName = outputMeshName_;
        MGlobal::displayInfo(MString("Updating existing skinCluster on: ") + targetMeshName);
      } else {
        // Mesh exists but no skinCluster - delete and recreate
        MGlobal::executeCommand("delete \"" + escapeMelString(outputMeshName_) + "\"");
        targetMeshName = outputMeshName_;
      }
    } else {
      // Mesh doesn't exist - will create with specified name
      targetMeshName = outputMeshName_;
    }
  }

  if (!updateExisting) {
    // Duplicate mesh and optionally rename
    MStringArray dup;
    MGlobal::executeCommand("duplicate -rr " + pathMesh_.partialPathName(), dup);
    if (dup.length() == 0) {
      MGlobal::displayError("Failed to duplicate mesh.");
      return MS::kFailure;
    }

    // Rename if outputMeshName_ is specified
    if (targetMeshName.length() > 0) {
      MGlobal::executeCommand("rename \"" + escapeMelString(dup[0]) + "\" \"" + escapeMelString(targetMeshName) + "\"");
    } else {
      targetMeshName = dup[0];
    }

    // Save created mesh name for undo
    createdMeshName_ = targetMeshName;

    // Create skinCluster
    MString cmd("skinCluster -tsb");
    for (int i = 0; i < nB; ++i) {
      cmd += MString(" \"") + name[i].c_str() + "\"";
    }
    cmd += " \"" + targetMeshName + "\"";

    MStringArray res;
    status = MGlobal::executeCommand(cmd, res);
    if (MFAIL(status) || res.length() == 0) {
      MGlobal::displayError("Failed to create skinCluster. Command: " + cmd);
      MGlobal::executeCommand("delete \"" + targetMeshName + "\"");
      return MS::kFailure;
    }

    // Save created skinCluster name for undo
    createdSkinClusterName_ = res[0];

    status = getDependNode(res[0], existingSkinCluster);
    if (MFAIL(status)) {
      MGlobal::displayError("Failed to get skinCluster node: " + res[0]);
      return status;
    }
    status = getDagPath(targetMeshName, targetMeshPath);
    if (MFAIL(status)) {
      MGlobal::displayError("Failed to get target mesh path: " + targetMeshName);
      return status;
    }
  } else {
    // Update existing skinCluster - may need to add new influences
    MFnSkinCluster fnSkin(existingSkinCluster);
    MDagPathArray influences;
    fnSkin.influenceObjects(influences, &status);

    // Build set of existing influence names
    std::unordered_set<std::string> existingInfluences;
    for (unsigned int i = 0; i < influences.length(); ++i) {
      existingInfluences.insert(influences[i].partialPathName().asChar());
    }

    // Add any missing influences
    for (int i = 0; i < nB; ++i) {
      if (existingInfluences.find(name[i]) == existingInfluences.end()) {
        MDagPath jointPath;
        if (getDagPath(name[i].c_str(), jointPath) == MS::kSuccess) {
          MString addCmd = "skinCluster -e -ai \"" + MString(name[i].c_str()) + "\" -wt 0 \"" +
                           fnSkin.name() + "\"";
          MGlobal::executeCommand(addCmd);
          MGlobal::displayInfo(MString("Added new influence: ") + name[i].c_str());
        }
      }
    }
  }

  // Build influence index mapping (bone name -> skinCluster influence index)
  MFnSkinCluster fnSkin(existingSkinCluster);
  MDagPathArray influences;
  fnSkin.influenceObjects(influences, &status);
  CHECK_MSTATUS_AND_RETURN_IT(status);

  std::unordered_map<std::string, int> nameToInfluenceIdx;
  for (unsigned int i = 0; i < influences.length(); ++i) {
    nameToInfluenceIdx[influences[i].partialPathName().asChar()] = (int)i;
  }

  // Build the weight array matching skinCluster's influence order
  const int numInfluences = (int)influences.length();
  MDoubleArray weights((unsigned)(nV * numInfluences), 0.0);
  MIntArray inflIdx; inflIdx.setLength((unsigned)numInfluences);
  for (int i = 0; i < numInfluences; ++i) {
    inflIdx.set(i, i);
  }

  // Fill weights from our sparse matrix
  for (int v = 0; v < w.outerSize(); ++v) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(w, v); it; ++it) {
      int boneIdx = (int)it.row();
      if (boneIdx < nB) {
        auto mapIt = nameToInfluenceIdx.find(name[boneIdx]);
        if (mapIt != nameToInfluenceIdx.end()) {
          weights[v * numInfluences + mapIt->second] = it.value();
        }
      }
    }
  }

  // Get mesh shape path
  getMeshShapeNode_local(targetMeshPath);

  MFnSingleIndexedComponent fnComp;
  MObject components = fnComp.create(MFn::kMeshVertComponent);
  fnComp.setComplete(true);

  fnSkin.setWeights(targetMeshPath, components, inflIdx, weights, true);

  MGlobal::displayInfo(MString("Applied weights to: ") + targetMeshName);
  return MS::kSuccess;
}

Eigen::Matrix4d DemBonesCmd::toMatrix4d(const MMatrix& m) {
  return Eigen::Map<const Eigen::Matrix<double,4,4,Eigen::RowMajor>>(m[0]);
}

MStatus DemBonesCmd::undoIt() {
  MStatus status = MS::kSuccess;

  // Delete created mesh and its skinCluster (deleting mesh will auto-delete skinCluster)
  if (createdMeshName_.length() > 0) {
    MString delCmd = "delete \"" + createdMeshName_ + "\"";
    status = MGlobal::executeCommand(delCmd);
    if (MFAIL(status)) {
      MGlobal::displayWarning("Failed to delete created mesh: " + createdMeshName_);
    } else {
      MGlobal::displayInfo("Deleted mesh: " + createdMeshName_);
    }
    createdMeshName_ = "";
    createdSkinClusterName_ = "";
  }

  // Delete created joints in reverse order (child before parent)
  for (int i = (int)createdJoints_.length() - 1; i >= 0; --i) {
    MString jointName = createdJoints_[i];
    MString delCmd = "delete \"" + jointName + "\"";
    status = MGlobal::executeCommand(delCmd);
    if (MFAIL(status)) {
      MGlobal::displayWarning("Failed to delete joint: " + jointName);
    } else {
      MGlobal::displayInfo("Deleted joint: " + jointName);
    }
  }
  createdJoints_.clear();

  MGlobal::displayInfo("DemBones command undone successfully.");
  return MS::kSuccess;
}

#ifdef DEM_BONES_DEM_BONES_MAT_BLOCKS_UNDEFINED
  #undef blk4
  #undef rotMat
  #undef transVec
  #undef vec3
  #undef DEM_BONES_MAT_BLOCKS
#endif
