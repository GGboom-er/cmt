#pragma once
// DemBones Cache Command - manages mesh data caching for interactive use
// Commands:
//   demBonesCache -cache -mesh "meshName" -sf 1 -ef 100  : Cache mesh data
//   demBonesCache -query                                  : Query cache status
//   demBonesCache -clear                                  : Clear cache

#include <maya/MPxCommand.h>
#include <maya/MSyntax.h>
#include <maya/MArgDatabase.h>
#include <maya/MDagPath.h>

class DemBonesCacheCmd : public MPxCommand {
public:
    static void* creator() { return new DemBonesCacheCmd(); }
    static MSyntax newSyntax();

    MStatus doIt(const MArgList& args) override;

    static const char* kName;

private:
    // Flag names
    static const char* kCacheShort;
    static const char* kCacheLong;
    static const char* kQueryShort;
    static const char* kQueryLong;
    static const char* kClearShort;
    static const char* kClearLong;
    static const char* kMeshShort;
    static const char* kMeshLong;
    static const char* kStartFrameShort;
    static const char* kStartFrameLong;
    static const char* kEndFrameShort;
    static const char* kEndFrameLong;
};
