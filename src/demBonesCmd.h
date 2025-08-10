#ifndef DEM_BONES_CMD_H
#define DEM_BONES_CMD_H

// Keep ASCII-only comments to avoid MSVC C4819 on non-UTF8 builds.

#include <maya/MPxCommand.h>
#include <maya/MSyntax.h>
#include <maya/MArgDatabase.h>
#include <maya/MArgList.h>
#include <maya/MSelectionList.h>
#include <maya/MDagPath.h>
#include <maya/MDagPathArray.h>
#include <maya/MString.h>
#include <maya/MStringArray.h>
#include <maya/MProgressWindow.h>

#include <Eigen/Sparse>

#include <vector>
#include <string>

#include "DemBones/DemBonesExt.h"   // project-local
#include "common.h"        // getDagPath/getDependNode helpers

// Maya 2025 uses versioned namespaces. Bring symbols into scope.
using namespace Autodesk::Maya::OpenMaya20250000;

#ifndef DEM_BONES_MAT_BLOCKS
  #include "DemBones/MatBlocks.h"
  #define DEM_BONES_DEM_BONES_MAT_BLOCKS_UNDEFINED
#endif

// A small extension class to tap into progress window if desired.
// You may keep these callbacks no-op if you prefer.
struct MyDemBones : public Dem::DemBonesExt<double, float> {
  using Base = Dem::DemBonesExt<double, float>;
  // Advance one progress step at the start of each global iteration.
  void cbIterBegin() override {
    if (!MProgressWindow::isCancelled()) {
      MProgressWindow::advanceProgress(1);
    }
  }
};

class DemBonesCmd : public MPxCommand {
public:
  DemBonesCmd() = default;
  ~DemBonesCmd() override = default;

  static void*   creator();
  static MSyntax newSyntax();

  MStatus doIt (const MArgList& args) override;
  MStatus redoIt() override;
  MStatus undoIt() override;
  bool    isUndoable() const override;

  // Flags
  static const char* kWeightsSmoothStepShort;
  static const char* kWeightsSmoothStepLong;
  static const char* kWeightsSmoothShort;
  static const char* kWeightsSmoothLong;
  static const char* kNumNonZeroShort;
  static const char* kNumNonZeroLong;
  static const char* kWeightItersShort;
  static const char* kWeightItersLong;
  static const char* kTransAffineNormShort;
  static const char* kTransAffineNormLong;
  static const char* kTransAffineShort;
  static const char* kTransAffineLong;
  static const char* kBindUpdateShort;
  static const char* kBindUpdateLong;
  static const char* kTransItersShort;
  static const char* kTransItersLong;
  static const char* kItersShort;
  static const char* kItersLong;
  static const char* kInitItersShort;
  static const char* kInitItersLong;
  static const char* kBonesShort;
  static const char* kBonesLong;
  static const char* kStartFrameShort;
  static const char* kStartFrameLong;
  static const char* kEndFrameShort;
  static const char* kEndFrameLong;
  static const char* kExistingBonesShort;
  static const char* kExistingBonesLong;
  static const char* kSmoothSolverShort;   // "auto"|"ldlt"|"lu"
  static const char* kSmoothSolverLong;

  static const MString kName;

private:
  // Helpers implemented in .cpp
  MStatus readMeshSequence(double startFrame, double endFrame);
  MStatus readBindPose();

  MStatus setKeyframes(const Eigen::VectorXd& values,
                       const Eigen::VectorXd& frameTimes,
                       const MDagPath& pathJoint,
                       const MString& attributeName);

  MStatus setSkinCluster(const std::vector<std::string>& boneNames,
                         const Eigen::SparseMatrix<double>& w,
                         const Eigen::MatrixXd& gb);

  static Eigen::Matrix4d toMatrix4d(const MMatrix& mm);

private:
  MDagPath      pathMesh_;
  MDagPathArray pathBones_;
  MyDemBones    model_;
};

#ifdef DEM_BONES_DEM_BONES_MAT_BLOCKS_UNDEFINED
  #undef blk4
  #undef rotMat
  #undef transVec
  #undef vec3
  #undef DEM_BONES_MAT_BLOCKS
#endif

#endif // DEM_BONES_CMD_H
