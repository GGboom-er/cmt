#pragma once

#include <QDialog>
#include <QSlider>
#include <QSpinBox>
#include <QLabel>
#include <QPushButton>
#include <QCheckBox>
#include <QProgressBar>
#include <QGroupBox>
#include <QPointer>

// Forward declaration
class QVBoxLayout;

// DemBones Qt Window - Native C++ GUI for interactive parameter control
// Singleton pattern ensures only one window instance
class DemBonesWindow : public QDialog {
    Q_OBJECT

public:
    // Singleton access
    static DemBonesWindow* instance();
    static void showWindow();
    static void closeWindow();
    static void destroyWindow();  // Force destroy for plugin unload
    static bool isWindowOpen();
    static bool hasInstance();

    ~DemBonesWindow() override;

protected:
    void closeEvent(QCloseEvent* event) override;

private slots:
    void onSelectMesh();
    void onGetTimeline();
    void onCacheMesh();
    void onClearCache();
    void onExecute();
    void onBoneCountChanged(int value);
    void onIterationsChanged(int value);
    void onMaxInfluencesChanged(int value);
    void updateCacheStatus();

private:
    // Private constructor for singleton
    explicit DemBonesWindow(QWidget* parent = nullptr);

    void setupUI();
    void connectSignals();
    void setStatus(const QString& message, bool isError = false);
    QString executeCommand(const QString& cmd);

    // Singleton instance
    static QPointer<DemBonesWindow> s_instance;

    // Current state
    QString m_currentMesh;
    bool m_cached;
    bool m_executing;

    // UI Elements - Mesh Selection
    QLabel* m_meshLabel;
    QPushButton* m_selectBtn;
    QSpinBox* m_startFrameSpin;
    QSpinBox* m_endFrameSpin;
    QPushButton* m_timelineBtn;
    QPushButton* m_cacheBtn;
    QLabel* m_cacheLabel;
    QPushButton* m_clearCacheBtn;

    // UI Elements - Parameters
    QSlider* m_boneSlider;
    QSpinBox* m_boneSpin;
    QSlider* m_iterSlider;
    QSpinBox* m_iterSpin;
    QSlider* m_inflSlider;
    QSpinBox* m_inflSpin;

    // UI Elements - Options
    QCheckBox* m_deleteExistingCB;

    // UI Elements - Status
    QProgressBar* m_progressBar;
    QLabel* m_statusLabel;

    // UI Elements - Buttons
    QPushButton* m_executeBtn;
    QPushButton* m_closeBtn;
};
