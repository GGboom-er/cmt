#include "demBonesCacheCmd.h"
#include "demBonesCache.h"

#include <maya/MGlobal.h>
#include <maya/MSelectionList.h>
#include <maya/MFnDagNode.h>
#include <maya/MAnimControl.h>
#include <maya/MFnMesh.h>

#include <sstream>
#include <iomanip>
#include <chrono>

const char* DemBonesCacheCmd::kName = "demBonesCache";

const char* DemBonesCacheCmd::kCacheShort = "-c";
const char* DemBonesCacheCmd::kCacheLong = "-cache";
const char* DemBonesCacheCmd::kQueryShort = "-q";
const char* DemBonesCacheCmd::kQueryLong = "-query";
const char* DemBonesCacheCmd::kClearShort = "-cl";
const char* DemBonesCacheCmd::kClearLong = "-clear";
const char* DemBonesCacheCmd::kMeshShort = "-m";
const char* DemBonesCacheCmd::kMeshLong = "-mesh";
const char* DemBonesCacheCmd::kStartFrameShort = "-sf";
const char* DemBonesCacheCmd::kStartFrameLong = "-startFrame";
const char* DemBonesCacheCmd::kEndFrameShort = "-ef";
const char* DemBonesCacheCmd::kEndFrameLong = "-endFrame";

MSyntax DemBonesCacheCmd::newSyntax() {
    MSyntax syntax;
    syntax.addFlag(kCacheShort, kCacheLong, MSyntax::kNoArg);
    syntax.addFlag(kQueryShort, kQueryLong, MSyntax::kNoArg);
    syntax.addFlag(kClearShort, kClearLong, MSyntax::kNoArg);
    syntax.addFlag(kMeshShort, kMeshLong, MSyntax::kString);
    syntax.addFlag(kStartFrameShort, kStartFrameLong, MSyntax::kDouble);
    syntax.addFlag(kEndFrameShort, kEndFrameLong, MSyntax::kDouble);
    return syntax;
}

MStatus DemBonesCacheCmd::doIt(const MArgList& args) {
    MStatus status;
    MArgDatabase argData(syntax(), args, &status);
    if (MFAIL(status)) {
        MGlobal::displayError("Failed to parse arguments");
        return status;
    }

    DemBonesCache& cache = DemBonesCache::instance();

    // Query mode
    if (argData.isFlagSet(kQueryShort)) {
        if (cache.numVertices() > 0) {
            std::ostringstream oss;
            oss << "Cached: " << cache.meshName()
                << " | Vertices: " << cache.numVertices()
                << " | Frames: " << cache.numFrames()
                << " (" << cache.startFrame() << "-" << cache.endFrame() << ")";
            setResult(MString(oss.str().c_str()));
        } else {
            setResult(MString("No cache"));
        }
        return MS::kSuccess;
    }

    // Clear mode
    if (argData.isFlagSet(kClearShort)) {
        cache.invalidate();
        MGlobal::displayInfo("DemBones cache cleared");
        setResult(MString("cleared"));
        return MS::kSuccess;
    }

    // Cache mode
    if (argData.isFlagSet(kCacheShort)) {
        // Get mesh name
        MString meshName;
        if (argData.isFlagSet(kMeshShort)) {
            argData.getFlagArgument(kMeshShort, 0, meshName);
        } else {
            // Try selection
            MSelectionList sel;
            MGlobal::getActiveSelectionList(sel);
            if (sel.length() > 0) {
                MDagPath dagPath;
                sel.getDagPath(0, dagPath);
                if (dagPath.node().hasFn(MFn::kTransform)) {
                    dagPath.extendToShape();
                }
                if (dagPath.node().hasFn(MFn::kMesh)) {
                    meshName = dagPath.fullPathName();
                }
            }
        }

        if (meshName.length() == 0) {
            MGlobal::displayError("No mesh specified. Use -mesh flag or select a mesh.");
            return MS::kInvalidParameter;
        }

        // Get mesh dag path
        MSelectionList selList;
        selList.add(meshName);
        MDagPath meshPath;
        status = selList.getDagPath(0, meshPath);
        if (MFAIL(status)) {
            MGlobal::displayError("Invalid mesh: " + meshName);
            return MS::kInvalidParameter;
        }

        // Ensure we have shape node
        if (meshPath.node().hasFn(MFn::kTransform)) {
            meshPath.extendToShape();
        }
        if (!meshPath.node().hasFn(MFn::kMesh)) {
            MGlobal::displayError("Not a mesh: " + meshName);
            return MS::kInvalidParameter;
        }

        // Get frame range
        double startFrame = MAnimControl::minTime().value();
        double endFrame = MAnimControl::maxTime().value();
        if (argData.isFlagSet(kStartFrameShort)) {
            argData.getFlagArgument(kStartFrameShort, 0, startFrame);
        }
        if (argData.isFlagSet(kEndFrameShort)) {
            argData.getFlagArgument(kEndFrameShort, 0, endFrame);
        }

        if (endFrame <= startFrame) {
            MGlobal::displayError("Invalid frame range");
            return MS::kInvalidParameter;
        }

        // Check if already cached with same parameters
        if (cache.numVertices() > 0 &&
            cache.meshName() == meshPath.fullPathName().asChar() &&
            cache.startFrame() == startFrame &&
            cache.endFrame() == endFrame) {
            MGlobal::displayInfo("Cache already valid for this mesh and frame range");
            setResult(MString("cached"));
            return MS::kSuccess;
        }

        // Cache the mesh data
        auto startTime = std::chrono::high_resolution_clock::now();

        MGlobal::displayInfo("Caching mesh data...");
        bool success = cache.cacheMeshData(meshPath, startFrame, endFrame);

        auto endTime = std::chrono::high_resolution_clock::now();
        double elapsed = std::chrono::duration<double>(endTime - startTime).count();

        if (success) {
            std::ostringstream oss;
            oss << "Cached " << cache.numVertices() << " vertices x "
                << cache.numFrames() << " frames in " << std::fixed
                << std::setprecision(2) << elapsed << "s";
            MGlobal::displayInfo(MString(oss.str().c_str()));
            setResult(MString("cached"));
        } else {
            MGlobal::displayError("Failed to cache mesh data");
            setResult(MString("failed"));
            return MS::kFailure;
        }

        return MS::kSuccess;
    }

    // No valid flag
    MGlobal::displayError("Use -cache, -query, or -clear flag");
    return MS::kInvalidParameter;
}
