# ==============================================================
# CMakeLists.txt ― Maya 2025 插件 (Plugin) / DevKit 双环境构建
# ==============================================================

cmake_minimum_required(VERSION 3.22.1)
project(MayaTools LANGUAGES CXX)                     # 插件名称(Plugin Name)

# ---------- 1. Maya / DevKit 根路径 -------------------------------------------------
# 优先使用 MAYA_LOCATION 环境变量 (Environment Variable)，否则回退默认安装目录
if(DEFINED ENV{MAYA_LOCATION})
    set(MAYA_ROOT "$ENV{MAYA_LOCATION}")
else()
    set(MAYA_ROOT "C:/Program Files/Autodesk/Maya2025")
endif()
set(ENV{DEVKIT_LOCATION} "${MAYA_ROOT}")             # DevKit 宏依赖

# ---------- 2. 插入 Maya CMake 模块搜索路径 -----------------------------------------
list(APPEND CMAKE_MODULE_PATH "${MAYA_ROOT}/cmake")
set(PLUGIN_ENTRY "${MAYA_ROOT}/cmake/pluginEntry.cmake")
if(NOT EXISTS "${PLUGIN_ENTRY}")
    message(FATAL_ERROR "❌ 未找到 pluginEntry.cmake：${PLUGIN_ENTRY}\n请确认已安装 Maya 2025 DevKit。")
endif()
include("${PLUGIN_ENTRY}")                           # 引入 DevKit 宏

# ---------- 3. 编译器 & 语言标准 ----------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(MSVC)
    add_compile_options(/bigobj)                     # Eigen Debug 需要
endif()

# ---------- 4. OpenMP (并行库 / Parallel Library) -----------------------------------
find_package(OpenMP QUIET)
if(OpenMP_CXX_FOUND)
    add_compile_options(${OpenMP_CXX_FLAGS})
    list(APPEND PACKAGE_LIBS ${OpenMP_CXX_LIB_NAMES})# 交给 build_plugin 统一处理
endif()

# ---------- 5. 第三方头文件 (Eigen) -------------------------------------------------
set(EIGEN3_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../third-party/Eigen"
    CACHE PATH "Eigen3 include directory (local copy)")
find_package(Eigen3 3.4 REQUIRED)                    # 头文件库(Header-Only)

# ---------- 6. 源文件列表 (Source Files) --------------------------------------------
set(SOURCE_FILES
    pluginMain.cpp
    cmtConfig.h.in
    common.h           common.cpp
    linearRegressionSolver.h   linearRegressionSolver.cpp
    swingTwistNode.h           swingTwistNode.cpp
    swingTwistCmd.h            swingTwistCmd.cpp
    demBonesCmd.h              demBonesCmd.cpp
    rbfNode.h                  rbfNode.cpp
    ikRigNode.h                ikRigNode.cpp
)

set(DEMBONES_SOURCE
    DemBones/ConvexLS.h
    DemBones/DemBones.h
    DemBones/DemBonesExt.h
    DemBones/Indexing.h
    DemBones/MatBlocks.h
)
source_group("DemBones" FILES ${DEMBONES_SOURCE})
list(APPEND SOURCE_FILES ${DEMBONES_SOURCE})

# ---------- 7. Maya 核心库 (Core Libraries) -----------------------------------------
# 仅列出库名，由 build_plugin() 自动解析实际 .lib/.dylib 路径
set(LIBRARIES
    Foundation
    OpenMaya
    OpenMayaAnim
    OpenMayaUI
    OpenMayaRender
)
# 可选：TBB (Intel 线程构建模块 / Threading Building Blocks)
find_tbb()  # pluginEntry.cmake 提供的宏，自动更新 INCLUDE_DIRS/PACKAGE_LIBS

# ---------- 8. 头文件与附加搜索路径 -------------------------------------------------
list(APPEND INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}   # 本工程头
    ${EIGEN3_INCLUDE_DIR}         # Eigen 头
)

# ---------- 9. 生成插件 (Plugin) -----------------------------------------------------
# 由 DevKit 宏 build_plugin() 生成 .mll / .so / .bundle
build_plugin()

# ---------- 10. 安装规则 (Installation Rule) ----------------------------------------
install(TARGETS ${PROJECT_NAME} ${MAYA_TARGET_TYPE}
        DESTINATION "plug-ins/${MAYA_VERSION}")

# ---------- 11. 日志输出 (Log) ------------------------------------------------------
message(STATUS "✅ Maya SDK      : ${MAYA_ROOT}")
message(STATUS "✅ Eigen3 Include: ${EIGEN3_INCLUDE_DIR}")
message(STATUS "✅ 插件输出名称  : $<TARGET_FILE_NAME:${PROJECT_NAME}>")
