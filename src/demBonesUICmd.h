#pragma once

#include <maya/MPxCommand.h>
#include <maya/MSyntax.h>
#include <maya/MArgDatabase.h>

// DemBones UI Command - Shows native Qt window
// Usage: demBonesUI        - Show window
//        demBonesUI -close - Close window
class DemBonesUICmd : public MPxCommand {
public:
    static const char* kName;

    static void* creator() { return new DemBonesUICmd(); }
    static MSyntax newSyntax();

    MStatus doIt(const MArgList& args) override;

private:
    static const char* kCloseShort;
    static const char* kCloseLong;
};
