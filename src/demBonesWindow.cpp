#include "demBonesWindow.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QApplication>
#include <QCloseEvent>
#include <QTimer>

#include <maya/MGlobal.h>
#include <maya/MQtUtil.h>
#include <maya/MString.h>
#include <maya/MSelectionList.h>
#include <maya/MDagPath.h>
#include <maya/MFnMesh.h>
#include <maya/MFnDagNode.h>
#include <maya/MAnimControl.h>

// Singleton instance
QPointer<DemBonesWindow> DemBonesWindow::s_instance = nullptr;

DemBonesWindow* DemBonesWindow::instance() {
    if (!s_instance) {
        // Get Maya main window as parent
        QWidget* mayaMainWindow = MQtUtil::mainWindow();
        s_instance = new DemBonesWindow(mayaMainWindow);
    }
    return s_instance;
}

void DemBonesWindow::showWindow() {
    DemBonesWindow* win = instance();
    win->show();
    win->raise();
    win->activateWindow();
}

void DemBonesWindow::closeWindow() {
    if (s_instance) {
        s_instance->close();
    }
}

void DemBonesWindow::destroyWindow() {
    // Force destroy window - called during plugin unload
    if (s_instance) {
        // Mark as no longer executing to prevent further operations
        s_instance->m_executing = false;

        // Disconnect all signals to prevent callbacks during destruction
        s_instance->disconnect();

        // Hide first to prevent visual glitches
        s_instance->hide();

        // Process any pending events to ensure clean state
        QApplication::processEvents();

        // Delete the window - will also call destructor
        delete s_instance.data();
        s_instance = nullptr;
    }
}

bool DemBonesWindow::isWindowOpen() {
    return s_instance && s_instance->isVisible();
}

bool DemBonesWindow::hasInstance() {
    return !s_instance.isNull();
}

DemBonesWindow::DemBonesWindow(QWidget* parent)
    : QDialog(parent)
    , m_cached(false)
    , m_executing(false) {

    setWindowTitle("DemBones Interactive");
    setMinimumWidth(480);
    setWindowFlags(windowFlags() | Qt::Tool);

    setupUI();
    connectSignals();

    // Initialize
    onGetTimeline();
    updateCacheStatus();
}

DemBonesWindow::~DemBonesWindow() {
    s_instance = nullptr;
}

void DemBonesWindow::closeEvent(QCloseEvent* event) {
    s_instance = nullptr;
    QDialog::closeEvent(event);
}

void DemBonesWindow::setupUI() {
    QVBoxLayout* mainLayout = new QVBoxLayout(this);
    mainLayout->setSpacing(10);

    // ===== Mesh Selection Group =====
    QGroupBox* meshGroup = new QGroupBox("Mesh Selection");
    QVBoxLayout* meshLayout = new QVBoxLayout(meshGroup);

    // Mesh row
    QHBoxLayout* meshRow = new QHBoxLayout();
    m_meshLabel = new QLabel("No mesh selected");
    m_meshLabel->setStyleSheet("font-weight: bold;");
    m_selectBtn = new QPushButton("Get Selected");
    meshRow->addWidget(m_meshLabel, 1);
    meshRow->addWidget(m_selectBtn);
    meshLayout->addLayout(meshRow);

    // Frame range row
    QHBoxLayout* frameRow = new QHBoxLayout();
    frameRow->addWidget(new QLabel("Frame Range:"));
    m_startFrameSpin = new QSpinBox();
    m_startFrameSpin->setRange(-10000, 100000);
    frameRow->addWidget(m_startFrameSpin);
    frameRow->addWidget(new QLabel(" to "));
    m_endFrameSpin = new QSpinBox();
    m_endFrameSpin->setRange(-10000, 100000);
    frameRow->addWidget(m_endFrameSpin);
    m_timelineBtn = new QPushButton("Timeline");
    m_timelineBtn->setMaximumWidth(70);
    frameRow->addWidget(m_timelineBtn);
    frameRow->addStretch();
    meshLayout->addLayout(frameRow);

    // Cache row
    QHBoxLayout* cacheRow = new QHBoxLayout();
    m_cacheBtn = new QPushButton("Cache Mesh Data");
    m_cacheBtn->setEnabled(false);
    m_cacheLabel = new QLabel("No cache");
    m_cacheLabel->setStyleSheet("color: gray;");
    m_clearCacheBtn = new QPushButton("Clear");
    m_clearCacheBtn->setMaximumWidth(50);
    cacheRow->addWidget(m_cacheBtn);
    cacheRow->addWidget(m_cacheLabel, 1);
    cacheRow->addWidget(m_clearCacheBtn);
    meshLayout->addLayout(cacheRow);

    mainLayout->addWidget(meshGroup);

    // ===== Parameters Group =====
    QGroupBox* paramGroup = new QGroupBox("Parameters (adjust and re-execute)");
    QGridLayout* paramLayout = new QGridLayout(paramGroup);

    // Bone Count
    paramLayout->addWidget(new QLabel("Bone Count:"), 0, 0);
    m_boneSlider = new QSlider(Qt::Horizontal);
    m_boneSlider->setRange(1, 100);
    m_boneSlider->setValue(5);
    m_boneSpin = new QSpinBox();
    m_boneSpin->setRange(1, 500);
    m_boneSpin->setValue(5);
    paramLayout->addWidget(m_boneSlider, 0, 1);
    paramLayout->addWidget(m_boneSpin, 0, 2);

    // Iterations
    paramLayout->addWidget(new QLabel("Iterations:"), 1, 0);
    m_iterSlider = new QSlider(Qt::Horizontal);
    m_iterSlider->setRange(1, 100);
    m_iterSlider->setValue(30);
    m_iterSpin = new QSpinBox();
    m_iterSpin->setRange(1, 500);
    m_iterSpin->setValue(30);
    paramLayout->addWidget(m_iterSlider, 1, 1);
    paramLayout->addWidget(m_iterSpin, 1, 2);

    // Max Influences (range 1-12)
    paramLayout->addWidget(new QLabel("Max Influences:"), 2, 0);
    m_inflSlider = new QSlider(Qt::Horizontal);
    m_inflSlider->setRange(1, 12);
    m_inflSlider->setValue(4);
    m_inflSpin = new QSpinBox();
    m_inflSpin->setRange(1, 12);
    m_inflSpin->setValue(4);
    paramLayout->addWidget(m_inflSlider, 2, 1);
    paramLayout->addWidget(m_inflSpin, 2, 2);

    mainLayout->addWidget(paramGroup);

    // ===== Options Group =====
    QGroupBox* optionsGroup = new QGroupBox("Options");
    QHBoxLayout* optionsLayout = new QHBoxLayout(optionsGroup);
    m_deleteExistingCB = new QCheckBox("Delete existing dembones joints before execution");
    m_deleteExistingCB->setChecked(true);
    optionsLayout->addWidget(m_deleteExistingCB);
    mainLayout->addWidget(optionsGroup);

    // ===== Status Group =====
    QGroupBox* statusGroup = new QGroupBox("Status");
    QVBoxLayout* statusLayout = new QVBoxLayout(statusGroup);
    m_progressBar = new QProgressBar();
    m_progressBar->setRange(0, 100);
    m_statusLabel = new QLabel("Ready - Select a mesh and cache data for fast iteration");
    statusLayout->addWidget(m_progressBar);
    statusLayout->addWidget(m_statusLabel);
    mainLayout->addWidget(statusGroup);

    // ===== Buttons =====
    QHBoxLayout* btnLayout = new QHBoxLayout();
    m_executeBtn = new QPushButton("Execute DemBones");
    m_executeBtn->setEnabled(false);
    m_executeBtn->setMinimumHeight(35);
    m_executeBtn->setStyleSheet("font-weight: bold;");
    m_closeBtn = new QPushButton("Close");
    btnLayout->addWidget(m_executeBtn, 2);
    btnLayout->addWidget(m_closeBtn, 1);
    mainLayout->addLayout(btnLayout);
}

void DemBonesWindow::connectSignals() {
    connect(m_selectBtn, &QPushButton::clicked, this, &DemBonesWindow::onSelectMesh);
    connect(m_timelineBtn, &QPushButton::clicked, this, &DemBonesWindow::onGetTimeline);
    connect(m_cacheBtn, &QPushButton::clicked, this, &DemBonesWindow::onCacheMesh);
    connect(m_clearCacheBtn, &QPushButton::clicked, this, &DemBonesWindow::onClearCache);
    connect(m_executeBtn, &QPushButton::clicked, this, &DemBonesWindow::onExecute);
    connect(m_closeBtn, &QPushButton::clicked, this, &QDialog::close);

    // Sync sliders and spinboxes
    connect(m_boneSlider, &QSlider::valueChanged, this, &DemBonesWindow::onBoneCountChanged);
    connect(m_boneSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &DemBonesWindow::onBoneCountChanged);

    connect(m_iterSlider, &QSlider::valueChanged, this, &DemBonesWindow::onIterationsChanged);
    connect(m_iterSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &DemBonesWindow::onIterationsChanged);

    connect(m_inflSlider, &QSlider::valueChanged, this, &DemBonesWindow::onMaxInfluencesChanged);
    connect(m_inflSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &DemBonesWindow::onMaxInfluencesChanged);

    // Frame range changes
    connect(m_startFrameSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &DemBonesWindow::updateCacheStatus);
    connect(m_endFrameSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &DemBonesWindow::updateCacheStatus);
}

void DemBonesWindow::onBoneCountChanged(int value) {
    if (m_boneSlider->value() != value) m_boneSlider->setValue(qMin(value, 100));
    if (m_boneSpin->value() != value) m_boneSpin->setValue(value);
}

void DemBonesWindow::onIterationsChanged(int value) {
    if (m_iterSlider->value() != value) m_iterSlider->setValue(qMin(value, 100));
    if (m_iterSpin->value() != value) m_iterSpin->setValue(value);
}

void DemBonesWindow::onMaxInfluencesChanged(int value) {
    if (m_inflSlider->value() != value) m_inflSlider->setValue(qMin(value, 12));
    if (m_inflSpin->value() != value) m_inflSpin->setValue(value);
}

QString DemBonesWindow::executeCommand(const QString& cmd) {
    MString result;
    MStatus status = MGlobal::executeCommand(MString(cmd.toUtf8().constData()), result);
    if (MFAIL(status)) {
        return QString();
    }
    return QString::fromUtf8(result.asChar());
}

void DemBonesWindow::onSelectMesh() {
    // Get selected mesh using Maya C++ API
    MSelectionList selection;
    MGlobal::getActiveSelectionList(selection);

    if (selection.length() == 0) {
        setStatus("Please select a mesh", true);
        return;
    }

    MDagPath dagPath;
    MStatus status = selection.getDagPath(0, dagPath);
    if (MFAIL(status)) {
        setStatus("Failed to get selection", true);
        return;
    }

    // If transform selected, extend to shape
    if (dagPath.node().hasFn(MFn::kTransform)) {
        dagPath.extendToShape();
    }

    // Check if it's a mesh
    if (!dagPath.node().hasFn(MFn::kMesh)) {
        setStatus("Please select a mesh", true);
        return;
    }

    // Get transform name (parent of shape)
    MFnDagNode dagNode(dagPath);
    MDagPath parentPath;
    MDagPath::getAPathTo(dagPath.transform(), parentPath);
    MFnDagNode parentNode(parentPath);
    QString transformName = QString::fromUtf8(parentNode.name().asChar());

    // Get vertex count
    MFnMesh meshFn(dagPath);
    int vertexCount = meshFn.numVertices();

    // Store the transform name for commands
    m_currentMesh = transformName;

    // Update UI
    m_meshLabel->setText(QString("%1 (%2 verts)").arg(transformName).arg(vertexCount));
    m_cacheBtn->setEnabled(true);
    m_executeBtn->setEnabled(true);
    setStatus("Mesh selected. Cache data for faster iteration, or execute directly.");
    updateCacheStatus();
}

void DemBonesWindow::onGetTimeline() {
    // Use Maya C++ API for timeline
    double startFrame = MAnimControl::minTime().value();
    double endFrame = MAnimControl::maxTime().value();

    m_startFrameSpin->setValue(static_cast<int>(startFrame));
    m_endFrameSpin->setValue(static_cast<int>(endFrame));
}

void DemBonesWindow::updateCacheStatus() {
    QString result = executeCommand("demBonesCache -query");

    if (!result.isEmpty() && result != "No cache") {
        m_cacheLabel->setText(result);
        m_cacheLabel->setStyleSheet("color: green;");
        m_cached = true;
    } else {
        m_cacheLabel->setText("No cache");
        m_cacheLabel->setStyleSheet("color: gray;");
        m_cached = false;
    }
}

void DemBonesWindow::onCacheMesh() {
    if (m_currentMesh.isEmpty()) {
        setStatus("No mesh selected", true);
        return;
    }

    int startFrame = m_startFrameSpin->value();
    int endFrame = m_endFrameSpin->value();

    if (endFrame <= startFrame) {
        setStatus("Invalid frame range", true);
        return;
    }

    setStatus("Caching mesh data...");
    m_progressBar->setValue(30);
    QApplication::processEvents();

    QString cmd = QString("demBonesCache -cache -mesh \"%1\" -sf %2 -ef %3")
                      .arg(m_currentMesh)
                      .arg(startFrame)
                      .arg(endFrame);
    QString result = executeCommand(cmd);

    m_progressBar->setValue(100);
    updateCacheStatus();

    if (result == "cached") {
        setStatus("Mesh data cached. Parameter changes now execute faster!");
    } else {
        setStatus("Cache operation completed: " + result);
    }

    m_progressBar->setValue(0);
}

void DemBonesWindow::onClearCache() {
    executeCommand("demBonesCache -clear");
    updateCacheStatus();
    setStatus("Cache cleared");
}

void DemBonesWindow::onExecute() {
    if (m_currentMesh.isEmpty()) {
        setStatus("No mesh selected", true);
        return;
    }

    if (m_executing) {
        return;
    }

    int startFrame = m_startFrameSpin->value();
    int endFrame = m_endFrameSpin->value();

    if (endFrame <= startFrame) {
        setStatus("Invalid frame range", true);
        return;
    }

    // Delete existing joints if requested
    if (m_deleteExistingCB->isChecked()) {
        QString existing = executeCommand("ls \"dembones_joint*\" -type joint");
        if (!existing.trimmed().isEmpty()) {
            executeCommand("delete `ls \"dembones_joint*\" -type joint`");
        }
    }

    m_executing = true;
    QString cacheStatus = m_cached ? " (using cache)" : " (sampling mesh)";
    setStatus("Executing DemBones..." + cacheStatus);
    m_progressBar->setValue(50);
    m_executeBtn->setEnabled(false);
    QApplication::processEvents();

    QString cmd = QString("demBones -b %1 -sf %2 -ef %3 -mi %4 -i %5 \"%6\"")
                      .arg(m_boneSpin->value())
                      .arg(startFrame)
                      .arg(endFrame)
                      .arg(m_inflSpin->value())
                      .arg(m_iterSpin->value())
                      .arg(m_currentMesh);

    executeCommand(cmd);

    m_progressBar->setValue(100);
    setStatus(QString("Completed: %1 bones created. Adjust parameters and execute again!").arg(m_boneSpin->value()));

    m_executing = false;
    m_executeBtn->setEnabled(true);

    // Reset progress bar after delay - use QPointer to guard against window destruction
    QPointer<DemBonesWindow> guard(this);
    QTimer::singleShot(2000, this, [guard]() {
        if (guard && guard->m_progressBar) {
            guard->m_progressBar->setValue(0);
        }
    });
}

void DemBonesWindow::setStatus(const QString& message, bool isError) {
    m_statusLabel->setText(message);
    if (isError) {
        m_statusLabel->setStyleSheet("QLabel { color: red; }");
    } else {
        m_statusLabel->setStyleSheet("QLabel { color: green; }");
    }
}
