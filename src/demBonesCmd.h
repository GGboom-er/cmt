#ifndef CMT_DEMBONESCMD_H
#define CMT_DEMBONESCMD_H

#include <maya/MArgDatabase.h>
#include <maya/MArgList.h>
#include <maya/MDGModifier.h>
#include <maya/MDagPath.h>
#include <maya/MDagPathArray.h>
#include <maya/MGlobal.h>
#include <maya/MObject.h>
#include <maya/MPxCommand.h>
#include <maya/MSelectionList.h>
#include <maya/MSyntax.h>

#include "DemBones/DemBonesExt.h"

// Standard library
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <iostream>
#include <variant>

template <typename AniMeshScalar>
class MyDemBones : public Dem::DemBonesExt<double, AniMeshScalar> {
 public:
  void cbIterBegin() { std::cout << "    Iter #" << iter << ": "; }

  void cbIterEnd() { std::cout << "RMSE = " << rmse() << "\n"; }

  void cbInitSplitBegin() { std::cout << ">"; }

  void cbInitSplitEnd() { std::cout << nB; }

  void cbWeightsBegin() { std::cout << "Updating weights"; }

  void cbWeightsEnd() { std::cout << " Done! "; }

  void cbTransformationsBegin() { std::cout << "Updating trans"; }

  void cbTransformationsEnd() { std::cout << " Done! "; }

  void cbTransformationsIterEnd() { std::cout << "."; }

  void cbWeightsIterEnd() { std::cout << "."; }
};

class DemBonesCmd : public MPxCommand {
 public:
  MStatus doIt(const MArgList& argList) override;
  MStatus redoIt() override;
  MStatus undoIt() override;
  bool isUndoable() const override;

  static void* creator();
  static MSyntax newSyntax();

  static const MString kName;
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
  static const char* kDoubleAniMeshShort;
  static const char* kDoubleAniMeshLong;

 private:
  template <typename Model>
  MStatus readMeshSequence(Model& model, double startFrame, double endFrame);
  template <typename Model>
  MStatus readBindPose(Model& model);
  MStatus setKeyframes(const Eigen::VectorXd& val, const Eigen::VectorXd& fTime,
                       const MDagPath& pathJoint, const MString& attributeName);
  MStatus setSkinCluster(const std::vector<std::string>& name, const Eigen::SparseMatrix<double>& w,
                         const Eigen::MatrixXd& gb);
  Eigen::Matrix4d toMatrix4d(const MMatrix& m);

  std::variant<MyDemBones<float>, MyDemBones<double>> model_;
  bool useDoubleAniMesh_;
  MDGModifier dgMod_;
  MString name_;
  MDagPath pathMesh_;
  MDagPathArray pathBones_;
};

#endif
