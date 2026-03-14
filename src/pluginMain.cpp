
#include <maya/MFnPlugin.h>
#include <maya/MGlobal.h>

#include "demBonesCmd.h"
#include "demBonesCacheCmd.h"
#include "demBonesUICmd.h"
#include "demBonesWindow.h"
#include "demBonesCache.h"
#include "ikRigNode.h"
#include "rbfNode.h"
#include "swingTwistCmd.h"
#include "swingTwistNode.h"

// Create DemBones menu on plugin load
static void createDemBonesMenu() {
  // Create menu if not exists - uses native C++ UI command
  MString menuCmd =
      "if (!`menu -exists demBonesMenu`) {\n"
      "    global string $gMainWindow;\n"
      "    menu -parent $gMainWindow -label \"DemBones\" -tearOff true demBonesMenu;\n"
      "    menuItem -label \"Open DemBones UI\" -command \"demBonesUI\" demBonesMenuItem;\n"
      "    menuItem -divider true;\n"
      "    menuItem -label \"Cache Selected Mesh\" -command \"demBonesCache -cache\" demBonesCacheItem;\n"
      "    menuItem -label \"Query Cache\" -command \"demBonesCache -query\" demBonesQueryItem;\n"
      "    menuItem -label \"Clear Cache\" -command \"demBonesCache -clear\" demBonesClearItem;\n"
      "}\n";
  MGlobal::executeCommand(menuCmd);
}

// Remove DemBones menu on plugin unload
static void removeDemBonesMenu() {
  MString cmd =
      "if (`menu -exists demBonesMenu`) {\n"
      "    deleteUI demBonesMenu;\n"
      "}\n";
  MGlobal::executeCommand(cmd);
}

MStatus initializePlugin(MObject obj) {
  MStatus status;

  MFnPlugin plugin(obj, "Chad Vernon", "1.0", "any");

  status = plugin.registerNode(SwingTwistNode::kName, SwingTwistNode::id, SwingTwistNode::creator,
                               SwingTwistNode::initialize);
  CHECK_MSTATUS_AND_RETURN_IT(status);
  status = plugin.registerCommand(SwingTwistCmd::kName, SwingTwistCmd::creator,
                                  SwingTwistCmd::newSyntax);
  CHECK_MSTATUS_AND_RETURN_IT(status);
  status = plugin.registerNode(RBFNode::kName, RBFNode::id, RBFNode::creator, RBFNode::initialize);
  CHECK_MSTATUS_AND_RETURN_IT(status);
  status = plugin.registerCommand(DemBonesCmd::kName, DemBonesCmd::creator, DemBonesCmd::newSyntax);
  CHECK_MSTATUS_AND_RETURN_IT(status);
  status = plugin.registerCommand(DemBonesCacheCmd::kName, DemBonesCacheCmd::creator,
                                  DemBonesCacheCmd::newSyntax);
  CHECK_MSTATUS_AND_RETURN_IT(status);
  status = plugin.registerCommand(DemBonesUICmd::kName, DemBonesUICmd::creator,
                                  DemBonesUICmd::newSyntax);
  CHECK_MSTATUS_AND_RETURN_IT(status);

  status = plugin.registerNode(IKRigNode::kName, IKRigNode::id, IKRigNode::creator,
                               IKRigNode::initialize);
  CHECK_MSTATUS_AND_RETURN_IT(status);

  // Create DemBones menu
  createDemBonesMenu();

  return status;
}

MStatus uninitializePlugin(MObject obj) {
  MStatus status;
  MFnPlugin plugin(obj);

  // CRITICAL: Destroy Qt window BEFORE deregistering commands
  // Otherwise the window may try to call commands that no longer exist
  if (DemBonesWindow::hasInstance()) {
    DemBonesWindow::destroyWindow();
  }

  // Clear mesh cache to free memory
  DemBonesCache::instance().invalidate();

  // Remove DemBones menu
  removeDemBonesMenu();

  status = plugin.deregisterNode(IKRigNode::id);
  CHECK_MSTATUS_AND_RETURN_IT(status);
  status = plugin.deregisterCommand(DemBonesCmd::kName);
  CHECK_MSTATUS_AND_RETURN_IT(status);
  status = plugin.deregisterCommand(DemBonesCacheCmd::kName);
  CHECK_MSTATUS_AND_RETURN_IT(status);
  status = plugin.deregisterCommand(DemBonesUICmd::kName);
  CHECK_MSTATUS_AND_RETURN_IT(status);
  status = plugin.deregisterNode(RBFNode::id);
  CHECK_MSTATUS_AND_RETURN_IT(status);
  status = plugin.deregisterCommand(SwingTwistCmd::kName);
  CHECK_MSTATUS_AND_RETURN_IT(status);
  status = plugin.deregisterNode(SwingTwistNode::id);
  CHECK_MSTATUS_AND_RETURN_IT(status);

  return status;
}
