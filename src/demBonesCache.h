#pragma once
// DemBones data cache for interactive UI
// Caches vertex positions, bind pose, and topology to avoid repeated Maya queries

#include <maya/MDagPath.h>
#include <maya/MFnMesh.h>
#include <maya/MPointArray.h>
#include <maya/MIntArray.h>
#include <maya/MTime.h>
#include <maya/MAnimControl.h>
#include <maya/MGlobal.h>

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <mutex>

class DemBonesCache {
public:
    static DemBonesCache& instance() {
        static DemBonesCache inst;
        return inst;
    }

    // Check if cache is valid for given mesh
    bool isValid(const MDagPath& meshPath) const {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!valid_) return false;
        return cachedMeshName_ == meshPath.fullPathName().asChar();
    }

    // Invalidate cache (call when mesh changes)
    void invalidate() {
        std::lock_guard<std::mutex> lock(mutex_);
        valid_ = false;
        cachedMeshName_.clear();
        vertices_.resize(0, 0);
        bindPose_.resize(0, 0);
        faceVertices_.clear();
    }

    // Cache mesh data for frame range
    bool cacheMeshData(const MDagPath& meshPath, double startFrame, double endFrame) {
        std::lock_guard<std::mutex> lock(mutex_);

        MStatus status;
        MFnMesh meshFn(meshPath, &status);
        if (MFAIL(status)) return false;

        int nV = meshFn.numVertices();
        int nF = (int)(endFrame - startFrame) + 1;
        if (nV <= 0 || nF <= 0) return false;

        // Allocate storage
        vertices_.resize(3 * nF, nV);

        // Sample each frame
        MTime originalTime = MAnimControl::currentTime();
        MPointArray points;

        for (int f = 0; f < nF; ++f) {
            double frame = startFrame + f;
            MAnimControl::setCurrentTime(MTime(frame, MTime::uiUnit()));

            meshFn.getPoints(points, MSpace::kWorld);
            for (int v = 0; v < nV; ++v) {
                vertices_(f * 3 + 0, v) = points[v].x;
                vertices_(f * 3 + 1, v) = points[v].y;
                vertices_(f * 3 + 2, v) = points[v].z;
            }
        }

        // Restore time
        MAnimControl::setCurrentTime(originalTime);

        // Cache bind pose (first frame)
        bindPose_.resize(3, nV);
        MAnimControl::setCurrentTime(MTime(startFrame, MTime::uiUnit()));
        meshFn.getPoints(points, MSpace::kWorld);
        for (int v = 0; v < nV; ++v) {
            bindPose_(0, v) = points[v].x;
            bindPose_(1, v) = points[v].y;
            bindPose_(2, v) = points[v].z;
        }
        MAnimControl::setCurrentTime(originalTime);

        // Cache topology
        MIntArray faceCounts, faceVerts;
        meshFn.getVertices(faceCounts, faceVerts);
        faceVertices_.resize(faceVerts.length());
        for (unsigned int i = 0; i < faceVerts.length(); ++i) {
            faceVertices_[i] = faceVerts[i];
        }
        faceCounts_.resize(faceCounts.length());
        for (unsigned int i = 0; i < faceCounts.length(); ++i) {
            faceCounts_[i] = faceCounts[i];
        }

        // Mark valid
        cachedMeshName_ = meshPath.fullPathName().asChar();
        cachedStartFrame_ = startFrame;
        cachedEndFrame_ = endFrame;
        nVertices_ = nV;
        nFrames_ = nF;
        valid_ = true;

        return true;
    }

    // Getters
    const Eigen::MatrixXd& vertices() const { return vertices_; }
    const Eigen::MatrixXd& bindPose() const { return bindPose_; }
    const std::vector<int>& faceVertices() const { return faceVertices_; }
    const std::vector<int>& faceCounts() const { return faceCounts_; }
    int numVertices() const { return nVertices_; }
    int numFrames() const { return nFrames_; }
    double startFrame() const { return cachedStartFrame_; }
    double endFrame() const { return cachedEndFrame_; }
    const std::string& meshName() const { return cachedMeshName_; }

private:
    DemBonesCache() : valid_(false), nVertices_(0), nFrames_(0),
                      cachedStartFrame_(0), cachedEndFrame_(0) {}
    ~DemBonesCache() = default;
    DemBonesCache(const DemBonesCache&) = delete;
    DemBonesCache& operator=(const DemBonesCache&) = delete;

    mutable std::mutex mutex_;
    bool valid_;
    std::string cachedMeshName_;
    double cachedStartFrame_;
    double cachedEndFrame_;
    int nVertices_;
    int nFrames_;

    Eigen::MatrixXd vertices_;    // [3*nF, nV] - all frames
    Eigen::MatrixXd bindPose_;    // [3, nV] - bind pose
    std::vector<int> faceVertices_;
    std::vector<int> faceCounts_;
};
