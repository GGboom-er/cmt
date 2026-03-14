#include "demBonesUICmd.h"
#include "demBonesWindow.h"

#include <maya/MGlobal.h>

const char* DemBonesUICmd::kName = "demBonesUI";
const char* DemBonesUICmd::kCloseShort = "-c";
const char* DemBonesUICmd::kCloseLong = "-close";

MSyntax DemBonesUICmd::newSyntax() {
    MSyntax syntax;
    syntax.addFlag(kCloseShort, kCloseLong, MSyntax::kNoArg);
    return syntax;
}

MStatus DemBonesUICmd::doIt(const MArgList& args) {
    MStatus status;
    MArgDatabase argData(syntax(), args, &status);
    if (MFAIL(status)) {
        MGlobal::displayError("Failed to parse arguments");
        return status;
    }

    // Close mode
    if (argData.isFlagSet(kCloseShort)) {
        DemBonesWindow::closeWindow();
        setResult("closed");
        return MS::kSuccess;
    }

    // Show window
    DemBonesWindow::showWindow();
    setResult("opened");
    return MS::kSuccess;
}
