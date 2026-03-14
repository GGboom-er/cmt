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
#include <maya/MGlobal.h>

#include <Eigen/Sparse>

#include <vector>
#include <string>
#include <algorithm>
#include <unordered_set>
#include <sstream>
#include <iomanip>

#include "DemBones/DemBonesExt.h"   // project-local
#include "common.h"                 // getDagPath/getDependNode helpers

// Maya 2025 uses versioned namespaces. Bring symbols into scope.
using namespace Autodesk::Maya::OpenMaya20250000;

#ifndef DEM_BONES_MAT_BLOCKS
  #include "DemBones/MatBlocks.h"
  #define DEM_BONES_DEM_BONES_MAT_BLOCKS_UNDEFINED
#endif

// Small extension: progress callback, weight locking, and residual-driven initialization.
struct MyDemBones : public Dem::DemBonesExt<double, float> {
  using Base = Dem::DemBonesExt<double, float>;

  // Set of bone indices whose weights should not change (unordered for O(1) lookup)
  std::unordered_set<int> lockedBones;

  // Set of vertex indices whose weights should not change (unordered for O(1) lookup)
  std::unordered_set<int> lockedVertices;

  // Number of existing bones (for incremental mode)
  // New bones (indices >= existingBoneCount) will be initialized via residual-driven clustering
  int existingBoneCount = 0;

  // Compute per-vertex reconstruction error with current skinning
  // Returns vector of (vertexIndex, maxError) for all vertices
  void computePerVertexError(Eigen::VectorXd& errors) const {
    errors.resize(nV);
    errors.setZero();

    if (w.rows() == 0 || m.rows() == 0 || nV == 0 || nF == 0) return;
    if (existingBoneCount <= 0) return;  // No existing bones to compute residual from

    #pragma omp parallel for
    for (int i = 0; i < nV; ++i) {
      double maxErr = 0.0;
      Eigen::Matrix4d mki;
      for (int k = 0; k < nF; ++k) {
        mki.setZero();
        for (Eigen::SparseMatrix<double>::InnerIterator it(w, i); it; ++it) {
          int boneIdx = (int)it.row();
          // Boundary check: ensure boneIdx is within valid range for 4x4 block access
          // block(k*4, boneIdx*4, 4, 4) requires (k+1)*4 <= rows and (boneIdx+1)*4 <= cols
          if (boneIdx >= 0 && boneIdx < existingBoneCount &&
              (boneIdx + 1) * 4 <= m.cols() && (k + 1) * 4 <= m.rows()) {
            mki += it.value() * m.block(k * 4, boneIdx * 4, 4, 4);
          }
        }
        Eigen::Vector3d reconstructed = mki.topLeftCorner<3, 3>() * u.col(i).segment<3>(0)
                                      + mki.topRightCorner<3, 1>();
        Eigen::Vector3d original = v.col(i).segment<3>(k * 3).template cast<double>();
        double err = (reconstructed - original).squaredNorm();
        if (err > maxErr) maxErr = err;
      }
      errors(i) = std::sqrt(maxErr);
    }
  }

  // Initialize new bone weights based on high-residual vertices
  // Called after init() when in incremental mode
  // Only initializes weights - DemBones will handle transform initialization via LBG-VQ
  void initNewBonesFromResidual() {
    if (existingBoneCount <= 0 || existingBoneCount >= nB) return;
    if (w.rows() == 0 || m.rows() == 0) return;

    const int newBoneCount = nB - existingBoneCount;

    MGlobal::displayInfo(MString("Initializing weights for ") + newBoneCount + " new bones based on residual error...");

    // Compute per-vertex residual error
    Eigen::VectorXd errors;
    computePerVertexError(errors);

    // Find top vertices with highest error as seeds for new bones
    std::vector<std::pair<double, int>> errorWithIndex(nV);
    for (int i = 0; i < nV; ++i) {
      errorWithIndex[i] = {errors(i), i};
    }
    std::partial_sort(errorWithIndex.begin(),
                      errorWithIndex.begin() + std::min(newBoneCount * 10, nV),
                      errorWithIndex.end(),
                      std::greater<std::pair<double, int>>());

    // Select seed vertices for new bones (spread out spatially)
    std::vector<int> seedVertices;
    seedVertices.reserve(newBoneCount);

    // Adaptive minimum distance: start large, reduce if we can't find enough seeds
    const double bboxDiag = std::sqrt((u.colwise().maxCoeff() - u.colwise().minCoeff()).squaredNorm());
    double minDistance = 0.15 * bboxDiag / std::sqrt(double(newBoneCount));  // Adaptive based on bone count

    // Try multiple passes with decreasing distance threshold
    for (int pass = 0; pass < 3 && (int)seedVertices.size() < newBoneCount; ++pass) {
      if (pass > 0) {
        minDistance *= 0.5;  // Reduce distance threshold for next pass
        std::ostringstream oss;
        oss << "  Pass " << (pass + 1) << ": minDistance=" << minDistance
            << ", found " << seedVertices.size() << "/" << newBoneCount << " seeds";
        MGlobal::displayInfo(MString(oss.str().c_str()));
      }

      for (const auto& pair : errorWithIndex) {
        if ((int)seedVertices.size() >= newBoneCount) break;
        int vi = pair.second;
        Eigen::Vector3d pos = u.col(vi).segment<3>(0);

        // Check distance from existing seeds
        bool tooClose = false;
        for (int sv : seedVertices) {
          if ((u.col(sv).segment<3>(0) - pos).norm() < minDistance) {
            tooClose = true;
            break;
          }
        }
        if (!tooClose) {
          // Check if not already added
          if (std::find(seedVertices.begin(), seedVertices.end(), vi) == seedVertices.end()) {
            seedVertices.push_back(vi);
          }
        }
      }
    }

    // If still not enough seeds, forcefully add high-error vertices without distance check
    if ((int)seedVertices.size() < newBoneCount) {
      std::ostringstream oss;
      oss << "  Forcing remaining seeds without distance check: "
          << (newBoneCount - seedVertices.size()) << " bones";
      MGlobal::displayInfo(MString(oss.str().c_str()));

      for (const auto& pair : errorWithIndex) {
        if ((int)seedVertices.size() >= newBoneCount) break;
        int vi = pair.second;
        if (std::find(seedVertices.begin(), seedVertices.end(), vi) == seedVertices.end()) {
          seedVertices.push_back(vi);
        }
      }
    }

    // NOTE: We do NOT manually initialize bone transforms here.
    // DemBones will initialize them via LBG-VQ clustering based on the initial weights.
    // This ensures proper bone placement without interfering with the optimization algorithm.

    // Initialize weights for new bones based on residual error
    // Without initial weights, the optimization cannot update transforms (chicken-egg problem)
    // Strategy:
    // 1. Compute initial weight for new bones based on proximity and error
    // 2. Scale DOWN existing bone weights proportionally to make room for new weights
    // 3. Ensure total per-vertex weight remains 1.0 (preserving existing bone ratios)

    // First pass: compute raw new bone weights per vertex
    // newBoneWeights[v][newBoneLocalIdx] = raw weight
    std::vector<std::vector<double>> newBoneWeights(nV);
    for (int vi = 0; vi < nV; ++vi) {
      newBoneWeights[vi].resize(newBoneCount, 0.0);  // Allocate for ALL new bones
    }

    // Adaptive influence radius based on bone density
    const double sigma = bboxDiag / std::sqrt(double(newBoneCount)) * 3.0;  // Increased from 1.5 to 3.0
    const double sigmaSq = sigma * sigma;
    const double maxErr = errors.maxCoeff();

    // Safety check: if maxErr is zero or near-zero, all vertices fit perfectly - no need for new bones
    if (maxErr < 1e-10) {
      MGlobal::displayWarning("Warning: All vertices have near-zero error. New bones may not be needed.");
      // Assign uniform tiny weights to avoid division by zero
      const double uniformWeight = 0.01;
      for (int vi = 0; vi < nV; ++vi) {
        for (int newIdx = 0; newIdx < newBoneCount; ++newIdx) {
          newBoneWeights[vi][newIdx] = uniformWeight / newBoneCount;
        }
      }
      // Skip normal weight initialization
    } else {
      std::ostringstream sigmaLog;
      sigmaLog << "  Weight initialization: sigma=" << sigma
               << ", seedVertices=" << seedVertices.size()
               << ", newBoneCount=" << newBoneCount
               << ", maxErr=" << std::scientific << maxErr;
      MGlobal::displayInfo(MString(sigmaLog.str().c_str()));

      // Assign weights for all seed-based bones
      for (int newIdx = 0; newIdx < (int)seedVertices.size(); ++newIdx) {
        int seedVtx = seedVertices[newIdx];
        Eigen::Vector3d seedPos = u.col(seedVtx).segment<3>(0);

        int assignedVertCount = 0;
        for (int vi = 0; vi < nV; ++vi) {
          double err = errors(vi);
          if (err < 1e-6) continue;

          Eigen::Vector3d vPos = u.col(vi).segment<3>(0);
          double distSq = (vPos - seedPos).squaredNorm();

          // Gaussian weight based on distance, scaled by error ratio
          double spatialWeight = std::exp(-distSq / (2.0 * sigmaSq));
          double errorRatio = std::min(err / maxErr, 1.0);

          // Linear scaling for error weight - avoids over-aggressive reduction
          // for medium-error vertices that squared/cubic scaling causes
          // Example: errorRatio=0.5 now gives 0.5*0.8=0.4 weight factor
          // (was 0.25*0.625=0.15625 with squared scaling - 84% reduction!)
          double rawWeight = spatialWeight * errorRatio * 0.8;

          if (rawWeight > 0.001) {  // Lowered threshold from 0.005 to 0.001
            newBoneWeights[vi][newIdx] = rawWeight;
            assignedVertCount++;
          }
        }

        if (newIdx < 5 || assignedVertCount == 0) {  // Log first 5 bones or problematic ones
          std::ostringstream boneLog;
          boneLog << "    NewBone " << newIdx << " (seed vtx " << seedVtx
                  << "): " << assignedVertCount << " vertices assigned";
          MGlobal::displayInfo(MString(boneLog.str().c_str()));
        }
      }

      // Fill remaining bones (if seedVertices.size() < newBoneCount)
      // Use uniform distribution across high-error regions
      if ((int)seedVertices.size() < newBoneCount) {
        std::ostringstream fillLog;
        fillLog << "  Filling " << (newBoneCount - seedVertices.size())
                << " remaining bones with uniform weights";
        MGlobal::displayInfo(MString(fillLog.str().c_str()));

        for (int newIdx = (int)seedVertices.size(); newIdx < newBoneCount; ++newIdx) {
          // Assign small uniform weight to high-error vertices
          for (int vi = 0; vi < nV; ++vi) {
            double err = errors(vi);
            if (err > maxErr * 0.3) {  // Only high-error vertices
              newBoneWeights[vi][newIdx] = 0.1 / newBoneCount;  // Uniform small weight
            }
          }
        }
      }
    }  // End of if (maxErr >= 1e-10) block

    // Second pass: build new weight matrix
    // - Keep existing bone weights but scale them down if new bones get weight
    // - Add new bone weights for ALL newBoneCount bones
    // - Ensure sum = 1.0
    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(w.nonZeros() + newBoneCount * nV / 5);

    int verticesWithNewWeights = 0;
    for (int vi = 0; vi < nV; ++vi) {
      // Calculate total new bone weight for this vertex
      double newBonesTotal = 0.0;
      for (double nw : newBoneWeights[vi]) {
        newBonesTotal += nw;
      }

      // Adaptive capping based on vertex error
      // High error vertices: allow new bones up to 90% influence
      // Low error vertices: limit new bones to 50% influence
      double errRatio = (maxErr > 1e-10) ? std::min(errors(vi) / maxErr, 1.0) : 0.0;
      double maxNewInfluence = 0.5 + errRatio * 0.4;  // 50% to 90%
      newBonesTotal = std::min(newBonesTotal, maxNewInfluence);

      // Get existing bone weights
      double existingTotal = 0.0;
      std::vector<std::pair<int, double>> existingWeights;
      for (Eigen::SparseMatrix<double>::InnerIterator it(w, vi); it; ++it) {
        int boneIdx = (int)it.row();
        if (boneIdx < existingBoneCount) {  // Only existing bones
          existingWeights.emplace_back(boneIdx, it.value());
          existingTotal += it.value();
        }
      }

      // Scale factor for existing bones: they get (1 - newBonesTotal) of the weight budget
      double existingScale = (existingTotal > 1e-10) ? (1.0 - newBonesTotal) / existingTotal : 0.0;

      // Add scaled existing bone weights
      for (const auto& p : existingWeights) {
        double scaledWeight = p.second * existingScale;
        if (scaledWeight > 1e-10) {
          triplets.emplace_back(p.first, vi, scaledWeight);
        }
      }

      // Normalize and add new bone weights for ALL new bones
      double newBonesSum = 0.0;
      for (double nw : newBoneWeights[vi]) newBonesSum += nw;

      if (newBonesSum > 1e-10 && newBonesTotal > 1e-10) {
        double newScale = newBonesTotal / newBonesSum;  // Normalize to newBonesTotal
        bool hasNewWeight = false;
        for (int newIdx = 0; newIdx < newBoneCount; ++newIdx) {  // ALL new bones, not just seeds
          double nw = newBoneWeights[vi][newIdx] * newScale;
          if (nw > 1e-10) {
            int boneIdx = existingBoneCount + newIdx;
            triplets.emplace_back(boneIdx, vi, nw);
            hasNewWeight = true;
          }
        }
        if (hasNewWeight) verticesWithNewWeights++;
      }
    }

    // Rebuild weight matrix
    Eigen::SparseMatrix<double> newW(nB, nV);
    newW.setFromTriplets(triplets.begin(), triplets.end());
    w = newW;

    std::ostringstream oss;
    oss << "Initialized " << newBoneCount << " new bones: "
        << seedVertices.size() << " seed-based, "
        << (newBoneCount - seedVertices.size()) << " uniform. "
        << verticesWithNewWeights << "/" << nV << " vertices affected.";
    MGlobal::displayInfo(MString(oss.str().c_str()));
  }

  void cbIterBegin() override {
    if (!MProgressWindow::isCancelled()) {
      MProgressWindow::advanceProgress(1);
    }
  }

  // Called before weights update
  void cbWeightsBegin() override {
    // No special action needed in current implementation
  }

  // Called after weights update
  // In the new incremental mode, we let DemBones optimize all weights freely
  // so no restoration is needed
  void cbWeightsEnd() override {
    // No special action needed - DemBones handles weight optimization
  }

  // ========================================================================
  // INCREMENTAL BONE OPTIMIZATION (Residual-Aware Initialization)
  // ========================================================================
  // Incrementally add new bones to improve reconstruction quality.
  // Key idea: Initialize new bones based on high-error regions, then optimize together.
  //
  // Algorithm:
  // 1. Compute per-vertex residual error with existing bones
  // 2. Initialize new bone transforms and weights based on high-error vertices
  // 3. Lock existing bone transforms (keep them fixed)
  // 4. Run standard DemBones optimization (all bones together)
  //
  // This avoids the "bones at origin" problem by using original animation data (v)
  // instead of residual data for optimization.
  //
  // Returns true if successful, false if failed.
  bool fitNewBonesToResidual() {
    // Boundary checks
    if (existingBoneCount <= 0 || existingBoneCount >= nB) {
      MGlobal::displayError("Residual fit error: Invalid existing bone count");
      return false;
    }
    if (nV <= 0 || nF <= 0) {
      MGlobal::displayError("Residual fit error: Invalid vertex or frame count");
      return false;
    }
    if (m.rows() != nF * 4 || m.cols() < existingBoneCount * 4) {
      MGlobal::displayError("Residual fit error: Transform matrix dimension mismatch");
      return false;
    }
    if (w.rows() < existingBoneCount || w.cols() != nV) {
      MGlobal::displayError("Residual fit error: Weight matrix dimension mismatch");
      return false;
    }

    const int newBoneCount = nB - existingBoneCount;
    MGlobal::displayInfo("========================================");
    MGlobal::displayInfo("RESIDUAL-BASED MODE: Three-Stage Optimization");
    MGlobal::displayInfo("========================================");

    std::ostringstream configLog;
    configLog << "Configuration:\n"
              << "  Total bones: " << nB << " (" << existingBoneCount << " existing + "
              << newBoneCount << " new)\n"
              << "  Vertices: " << nV << "\n"
              << "  Frames: " << nF << "\n"
              << "  Max influences (nnz): " << nnz;
    MGlobal::displayInfo(MString(configLog.str().c_str()));

    // ========================================================================
    // Stage 1: Compute residual from existing bones
    // ========================================================================
    MGlobal::displayInfo("\n[Stage 1] Computing reconstruction residuals from existing bones...");

    Eigen::MatrixXf residual(3 * nF, nV);
    residual.setZero();

    double totalResidualNorm = 0.0;
    double maxVertexResidual = 0.0;
    int maxResidualVertex = -1;

    MGlobal::displayInfo("  Reconstructing mesh with existing bones...");

    #pragma omp parallel for reduction(+:totalResidualNorm)
    for (int i = 0; i < nV; ++i) {
      double vertexResidualSum = 0.0;
      for (int k = 0; k < nF; ++k) {
        // 用现有骨骼重建顶点位置
        Eigen::Matrix4d mki = Eigen::Matrix4d::Zero();
        for (Eigen::SparseMatrix<double>::InnerIterator it(w, i); it; ++it) {
          int boneIdx = (int)it.row();
          // Boundary check: ensure boneIdx is within valid range for 4x4 block access
          if (boneIdx >= 0 && boneIdx < existingBoneCount &&
              (k + 1) * 4 <= m.rows() && (boneIdx + 1) * 4 <= m.cols()) {
            mki += it.value() * m.block(k * 4, boneIdx * 4, 4, 4);
          }
        }

        Eigen::Vector3d reconstructed = mki.topLeftCorner<3, 3>() * u.col(i).segment<3>(0)
                                      + mki.topRightCorner<3, 1>();
        Eigen::Vector3d original = v.col(i).segment<3>(k * 3).template cast<double>();
        Eigen::Vector3d res = original - reconstructed;

        residual.col(i).segment<3>(k * 3) = res.cast<float>();
        vertexResidualSum += res.squaredNorm();
      }

      double vertexRMSE = std::sqrt(vertexResidualSum / nF);
      totalResidualNorm += vertexResidualSum;

      #pragma omp critical
      {
        if (vertexRMSE > maxVertexResidual) {
          maxVertexResidual = vertexRMSE;
          maxResidualVertex = i;
        }
      }
    }

    double residualRMSE = std::sqrt(totalResidualNorm / (nF * nV));

    std::ostringstream stage1Log;
    stage1Log << "[Stage 1] Residual Analysis Results:\n"
              << "  Residual RMSE: " << std::fixed << std::setprecision(6) << residualRMSE << "\n"
              << "  Max vertex residual: " << std::setprecision(6) << maxVertexResidual
              << " (vertex " << maxResidualVertex << ")\n"
              << "  Total residual energy: " << std::scientific << totalResidualNorm;
    MGlobal::displayInfo(MString(stage1Log.str().c_str()));

    if (residualRMSE < 1e-6) {
      MGlobal::displayWarning("Warning: Residual near zero. Existing bones fit perfectly. New bones may be ineffective.");
      // Continue anyway but user should know
    }

    // ========================================================================
    // Stage 2: Initialize new bones based on high-error regions
    // ========================================================================
    MGlobal::displayInfo("\n[Stage 2] Initializing new bones based on high-error regions...");

    // Call initNewBonesFromResidual() to initialize new bone transforms and weights
    // This function uses the residual error computed in Stage 1 to select seed vertices
    MGlobal::displayInfo("  Calling initNewBonesFromResidual() to initialize new bones...");
    initNewBonesFromResidual();

    // Verify initialization
    std::ostringstream initStats;
    initStats << "[Stage 2] New bone initialization complete:\n"
              << "  - Transform matrix m: [" << m.rows() << " x " << m.cols() << "]\n"
              << "  - Weight matrix w: [" << w.rows() << " x " << w.cols() << "]\n"
              << "  - Existing bones: " << existingBoneCount << "\n"
              << "  - New bones: " << newBoneCount << "\n"
              << "  - Total bones: " << nB;
    MGlobal::displayInfo(MString(initStats.str().c_str()));

    // Sample new bone transforms to verify they're not at origin
    if (newBoneCount > 0 && m.rows() >= 4 && m.cols() >= (existingBoneCount + 1) * 4) {
      int firstNewBone = existingBoneCount;
      Eigen::Matrix4d firstNewTransform = m.block<4, 4>(0, firstNewBone * 4);
      Eigen::Vector3d firstNewPos = firstNewTransform.topRightCorner<3, 1>();
      std::ostringstream sampleLog;
      sampleLog << "  - Sample new bone " << firstNewBone << " frame 1 position: ("
                << std::fixed << std::setprecision(4)
                << firstNewPos.x() << ", " << firstNewPos.y() << ", " << firstNewPos.z() << ")";
      MGlobal::displayInfo(MString(sampleLog.str().c_str()));
    }

    // ========================================================================
    // Stage 3: Run standard DemBones optimization with existing bones protected
    // ========================================================================
    MGlobal::displayInfo("\n[Stage 3] Running incremental optimization (protecting existing bones)...");

    // The existing bone transforms are already saved in origM
    // DemBones::computeTransformations() will automatically restore them
    // (see DemBones.h line 298-300)

    MGlobal::displayInfo("  Step 3.1: Verifying existing bone protection mechanism...");
    if (origM.rows() > 0) {
      std::ostringstream protLog;
      protLog << "  - origM matrix (existing bone transforms): [" << origM.rows() << " x " << origM.cols() << "]\n"
              << "  - Protecting transforms of first " << existingBoneCount << " bones from modification";
      MGlobal::displayInfo(MString(protLog.str().c_str()));
    }

    // Get initial RMSE before optimization
    double initialRMSE = rmse();
    std::ostringstream preOptLog;
    preOptLog << "  - Pre-optimization RMSE: " << std::fixed << std::setprecision(6) << initialRMSE;
    MGlobal::displayInfo(MString(preOptLog.str().c_str()));

    // Run standard DemBones compute()
    MGlobal::displayInfo("  Step 3.2: Starting standard DemBones optimization...");
    MGlobal::displayInfo("  (This may take several minutes. Existing bones remain unchanged, only new bones are optimized)");

    bool success = false;
    try {
      success = Base::compute();  // Call base class compute()
    } catch (const std::exception& e) {
      std::ostringstream errLog;
      errLog << "Optimization failed (exception): " << e.what();
      MGlobal::displayError(MString(errLog.str().c_str()));
      return false;
    }

    if (!success) {
      MGlobal::displayError("Optimization failed");
      return false;
    }

    MGlobal::displayInfo("  Step 3.3: Verifying optimization results...");

    // Compute final reconstruction quality
    double finalRMSE = rmse();

    // Verify existing bones are unchanged
    bool existingBonesIntact = true;
    if (origM.rows() > 0 && m.cols() >= origM.cols()) {
      double transformDiff = (m.block(0, 0, origM.rows(), origM.cols()) - origM).norm();
      existingBonesIntact = (transformDiff < 1e-8);
      std::ostringstream verifyLog;
      verifyLog << "  - Existing bone transform difference: " << std::scientific << transformDiff
                << (existingBonesIntact ? " (OK - unchanged)" : " (WARNING - modified)");
      MGlobal::displayInfo(MString(verifyLog.str().c_str()));
    }

    // Per-bone statistics
    MGlobal::displayInfo("  Step 3.4: Computing per-bone weight distribution...");
    std::ostringstream boneStats;
    boneStats << "  Bone weight statistics (first 10 bones):\n";
    for (int b = 0; b < std::min(nB, 10); ++b) {
      int vertCount = 0;
      double maxWeight = 0.0;
      for (int v = 0; v < nV; ++v) {
        double wVal = w.coeff(b, v);
        if (wVal > 1e-6) {
          vertCount++;
          if (wVal > maxWeight) maxWeight = wVal;
        }
      }
      boneStats << "    Bone " << b << ": " << vertCount << " vertices, max weight="
                << std::fixed << std::setprecision(4) << maxWeight;
      if (b < existingBoneCount) boneStats << " (existing)";
      else boneStats << " (new)";
      boneStats << "\n";
    }
    MGlobal::displayInfo(MString(boneStats.str().c_str()));

    // Final report
    std::ostringstream finalLog;
    finalLog << "\n========================================\n"
             << "INCREMENTAL OPTIMIZATION COMPLETE\n"
             << "========================================\n"
             << "Pre-optimization RMSE: " << std::fixed << std::setprecision(6) << initialRMSE << "\n"
             << "  - Existing bones RMSE: " << residualRMSE << "\n"
             << "Post-optimization RMSE: " << finalRMSE << "\n";

    if (initialRMSE > 1e-10) {
      double totalImprovement = (initialRMSE - finalRMSE) / initialRMSE * 100.0;
      finalLog << "  - Relative improvement: " << std::setprecision(2) << totalImprovement << "%\n";
    }

    if (residualRMSE > 1e-10) {
      double residualReduction = (residualRMSE - finalRMSE) / residualRMSE * 100.0;
      finalLog << "  - Residual reduction: " << std::setprecision(2) << residualReduction << "%\n";
    }

    finalLog << "Bone Statistics:\n"
             << "  - Existing bones: " << existingBoneCount << (existingBonesIntact ? " (unchanged)" : " (modified)") << "\n"
             << "  - New bones: " << newBoneCount << "\n"
             << "  - Total bones: " << nB << "\n"
             << "========================================";
    MGlobal::displayInfo(MString(finalLog.str().c_str()));

    return true;
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
  static const char* kBindFrameShort;      // bind pose frame
  static const char* kBindFrameLong;
  static const char* kUseExistingWeightsShort;  // warm start from skinCluster
  static const char* kUseExistingWeightsLong;
  static const char* kLockBonesShort;      // lock bone weights (multi-use)
  static const char* kLockBonesLong;
  static const char* kWeightsOnlyShort;    // only solve weights, keep transforms
  static const char* kWeightsOnlyLong;
  static const char* kDeformThresholdShort;  // threshold for detecting static vertices
  static const char* kDeformThresholdLong;
  static const char* kTotalBonesShort;       // target total bone count (incremental mode)
  static const char* kTotalBonesLong;
  static const char* kOutputMeshShort;       // output mesh name (for iterative workflow)
  static const char* kOutputMeshLong;

  static const MString kName;

private:
  // Helpers implemented in .cpp
  MStatus readMeshSequence(double startFrame, double endFrame, double bindFrame);
  MStatus readBindPose(double bindFrame);
  MStatus readExistingWeights();

  MStatus setKeyframes(const Eigen::VectorXd& values,
                       const Eigen::VectorXd& frameTimes,
                       const MDagPath& pathJoint,
                       const MString& attributeName);

  MStatus setSkinCluster(const std::vector<std::string>& name,
                         const Eigen::SparseMatrix<double>& w,
                         const Eigen::MatrixXd& gb);

  Eigen::Matrix4d toMatrix4d(const MMatrix& m);

private:
  MDagPath      pathMesh_;
  MDagPathArray pathBones_;
  MyDemBones    model_;

  // New parameters
  double        bindFrame_;
  bool          useExistingWeights_;
  bool          weightsOnly_;
  double        deformThreshold_;    // threshold for detecting static vertices (0 = disabled)
  std::unordered_set<int> lockedBoneIndices_;  // indices of bones whose weights should not change
  MString       outputMeshName_;     // output mesh name (empty = create new, specified = update existing)

  // Undo data - objects created by this command that need to be deleted on undo
  MStringArray  createdJoints_;      // list of joint names created by this command
  MString       createdMeshName_;    // name of duplicated mesh (if created)
  MString       createdSkinClusterName_;  // name of skinCluster (if created)
};

#ifdef DEM_BONES_DEM_BONES_MAT_BLOCKS_UNDEFINED
  #undef blk4
  #undef rotMat
  #undef transVec
  #undef vec3
  #undef DEM_BONES_MAT_BLOCKS
#endif

#endif // DEM_BONES_CMD_H
