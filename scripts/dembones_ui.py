# DemBones Interactive UI for Maya 2025
# Uses PySide6 (bundled with Maya) for GUI, calls C++ demBones command
# Supports mesh data caching for fast parameter iteration

from PySide6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QGridLayout,
    QGroupBox, QLabel, QPushButton, QSlider, QSpinBox,
    QProgressBar, QApplication, QCheckBox
)
from PySide6.QtCore import Qt, QTimer
from maya import cmds
from maya import mel
import maya.OpenMayaUI as omui
from shiboken6 import wrapInstance


def get_maya_main_window():
    """Get Maya main window as QWidget."""
    main_window_ptr = omui.MQtUtil.mainWindow()
    return wrapInstance(int(main_window_ptr), QDialog)


class DemBonesUI(QDialog):
    """Interactive DemBones UI with sliders and cached mesh data support."""

    _instance = None

    @classmethod
    def show_ui(cls):
        """Show the UI (singleton pattern)."""
        if cls._instance is None:
            cls._instance = DemBonesUI()
        cls._instance.show()
        cls._instance.raise_()
        cls._instance.activateWindow()
        return cls._instance

    @classmethod
    def close_ui(cls):
        """Close the UI."""
        if cls._instance is not None:
            cls._instance.close()
            cls._instance = None

    def __init__(self, parent=None):
        # Try to parent to Maya main window
        try:
            maya_main = get_maya_main_window()
            super().__init__(maya_main)
        except:
            super().__init__(parent)

        self.setWindowTitle("DemBones Interactive")
        self.setMinimumWidth(480)
        self.setWindowFlags(self.windowFlags() | Qt.Tool)

        self.current_mesh = ""
        self.cached = False
        self.executing = False

        self._setup_ui()
        self._connect_signals()
        self._update_frame_range()
        self._check_cache_status()

    def _setup_ui(self):
        """Setup the UI layout."""
        main_layout = QVBoxLayout(self)

        # ===== Mesh Selection Group =====
        mesh_group = QGroupBox("Mesh Selection")
        mesh_layout = QVBoxLayout(mesh_group)

        # Mesh row
        mesh_row = QHBoxLayout()
        self.mesh_label = QLabel("No mesh selected")
        self.mesh_label.setStyleSheet("font-weight: bold;")
        self.select_btn = QPushButton("Get Selected")
        mesh_row.addWidget(self.mesh_label, 1)
        mesh_row.addWidget(self.select_btn)
        mesh_layout.addLayout(mesh_row)

        # Frame range row
        frame_row = QHBoxLayout()
        frame_row.addWidget(QLabel("Frame Range:"))
        self.start_frame_spin = QSpinBox()
        self.start_frame_spin.setRange(-10000, 100000)
        frame_row.addWidget(self.start_frame_spin)
        frame_row.addWidget(QLabel(" to "))
        self.end_frame_spin = QSpinBox()
        self.end_frame_spin.setRange(-10000, 100000)
        frame_row.addWidget(self.end_frame_spin)
        self.get_timeline_btn = QPushButton("Timeline")
        self.get_timeline_btn.setMaximumWidth(70)
        frame_row.addWidget(self.get_timeline_btn)
        frame_row.addStretch()
        mesh_layout.addLayout(frame_row)

        # Cache row
        cache_row = QHBoxLayout()
        self.cache_btn = QPushButton("Cache Mesh Data")
        self.cache_btn.setEnabled(False)
        self.cache_status = QLabel("No cache")
        self.cache_status.setStyleSheet("color: gray;")
        self.clear_cache_btn = QPushButton("Clear")
        self.clear_cache_btn.setMaximumWidth(50)
        cache_row.addWidget(self.cache_btn)
        cache_row.addWidget(self.cache_status, 1)
        cache_row.addWidget(self.clear_cache_btn)
        mesh_layout.addLayout(cache_row)

        main_layout.addWidget(mesh_group)

        # ===== Parameters Group =====
        param_group = QGroupBox("Parameters (adjust and re-execute)")
        param_layout = QGridLayout(param_group)

        # Bone Count
        param_layout.addWidget(QLabel("Bone Count:"), 0, 0)
        self.bone_slider = QSlider(Qt.Horizontal)
        self.bone_slider.setRange(1, 100)
        self.bone_slider.setValue(5)
        self.bone_spin = QSpinBox()
        self.bone_spin.setRange(1, 100)
        self.bone_spin.setValue(5)
        param_layout.addWidget(self.bone_slider, 0, 1)
        param_layout.addWidget(self.bone_spin, 0, 2)

        # Iterations
        param_layout.addWidget(QLabel("Iterations:"), 1, 0)
        self.iter_slider = QSlider(Qt.Horizontal)
        self.iter_slider.setRange(1, 100)
        self.iter_slider.setValue(30)
        self.iter_spin = QSpinBox()
        self.iter_spin.setRange(1, 100)
        self.iter_spin.setValue(30)
        param_layout.addWidget(self.iter_slider, 1, 1)
        param_layout.addWidget(self.iter_spin, 1, 2)

        # Max Influences
        param_layout.addWidget(QLabel("Max Influences:"), 2, 0)
        self.infl_slider = QSlider(Qt.Horizontal)
        self.infl_slider.setRange(1, 16)
        self.infl_slider.setValue(4)
        self.infl_spin = QSpinBox()
        self.infl_spin.setRange(1, 16)
        self.infl_spin.setValue(4)
        param_layout.addWidget(self.infl_slider, 2, 1)
        param_layout.addWidget(self.infl_spin, 2, 2)

        main_layout.addWidget(param_group)

        # ===== Options Group =====
        options_group = QGroupBox("Options")
        options_layout = QHBoxLayout(options_group)
        self.delete_existing_cb = QCheckBox("Delete existing dembones joints before execution")
        self.delete_existing_cb.setChecked(True)
        options_layout.addWidget(self.delete_existing_cb)
        main_layout.addWidget(options_group)

        # ===== Status Group =====
        status_group = QGroupBox("Status")
        status_layout = QVBoxLayout(status_group)
        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 100)
        self.status_label = QLabel("Ready - Select a mesh and cache data for fast iteration")
        status_layout.addWidget(self.progress_bar)
        status_layout.addWidget(self.status_label)
        main_layout.addWidget(status_group)

        # ===== Buttons =====
        btn_layout = QHBoxLayout()
        self.execute_btn = QPushButton("Execute DemBones")
        self.execute_btn.setEnabled(False)
        self.execute_btn.setMinimumHeight(35)
        self.execute_btn.setStyleSheet("font-weight: bold;")
        self.close_btn = QPushButton("Close")
        btn_layout.addWidget(self.execute_btn, 2)
        btn_layout.addWidget(self.close_btn, 1)
        main_layout.addLayout(btn_layout)

    def _connect_signals(self):
        """Connect UI signals."""
        self.select_btn.clicked.connect(self._on_select_mesh)
        self.get_timeline_btn.clicked.connect(self._update_frame_range)
        self.cache_btn.clicked.connect(self._on_cache_mesh)
        self.clear_cache_btn.clicked.connect(self._on_clear_cache)

        # Sync sliders and spinboxes
        self.bone_slider.valueChanged.connect(self.bone_spin.setValue)
        self.bone_spin.valueChanged.connect(self.bone_slider.setValue)

        self.iter_slider.valueChanged.connect(self.iter_spin.setValue)
        self.iter_spin.valueChanged.connect(self.iter_slider.setValue)

        self.infl_slider.valueChanged.connect(self.infl_spin.setValue)
        self.infl_spin.valueChanged.connect(self.infl_slider.setValue)

        self.execute_btn.clicked.connect(self._on_execute)
        self.close_btn.clicked.connect(self.close)

        # Frame range changes invalidate cache relevance
        self.start_frame_spin.valueChanged.connect(self._on_frame_range_changed)
        self.end_frame_spin.valueChanged.connect(self._on_frame_range_changed)

    def _update_frame_range(self):
        """Update frame range from timeline."""
        start = cmds.playbackOptions(q=True, min=True)
        end = cmds.playbackOptions(q=True, max=True)
        self.start_frame_spin.setValue(int(start))
        self.end_frame_spin.setValue(int(end))

    def _check_cache_status(self):
        """Check current cache status from plugin."""
        try:
            result = mel.eval('demBonesCache -query')
            if result and result != "No cache":
                self.cache_status.setText(result)
                self.cache_status.setStyleSheet("color: green;")
                self.cached = True
            else:
                self.cache_status.setText("No cache")
                self.cache_status.setStyleSheet("color: gray;")
                self.cached = False
        except:
            self.cache_status.setText("Cache command not available")
            self.cache_status.setStyleSheet("color: red;")
            self.cached = False

    def _on_frame_range_changed(self):
        """Handle frame range changes."""
        # Cache may no longer be valid for new frame range
        self._check_cache_status()

    def _on_select_mesh(self):
        """Handle mesh selection."""
        sel = cmds.ls(selection=True, dag=True, shapes=True, type="mesh")
        if not sel:
            # Try getting transform and find shape
            sel_transforms = cmds.ls(selection=True, type="transform")
            if sel_transforms:
                shapes = cmds.listRelatives(sel_transforms[0], shapes=True, type="mesh")
                if shapes:
                    sel = shapes

        if not sel:
            self._set_status("Please select a mesh", error=True)
            return

        mesh = sel[0]
        # Get transform name for display
        parents = cmds.listRelatives(mesh, parent=True)
        transform = parents[0] if parents else mesh

        vtx_count = cmds.polyEvaluate(mesh, vertex=True)
        self.current_mesh = transform
        self.mesh_label.setText("{} ({} verts)".format(transform, vtx_count))
        self.cache_btn.setEnabled(True)
        self.execute_btn.setEnabled(True)
        self._set_status("Mesh selected. Cache data for faster iteration, or execute directly.")
        self._check_cache_status()

    def _on_cache_mesh(self):
        """Cache mesh data for fast parameter iteration."""
        if not self.current_mesh:
            self._set_status("No mesh selected", error=True)
            return

        start = self.start_frame_spin.value()
        end = self.end_frame_spin.value()

        if end <= start:
            self._set_status("Invalid frame range", error=True)
            return

        self._set_status("Caching mesh data...")
        self.progress_bar.setValue(30)
        QApplication.processEvents()

        try:
            cmd = 'demBonesCache -cache -mesh "{}" -sf {} -ef {}'.format(
                self.current_mesh, start, end)
            result = mel.eval(cmd)

            self.progress_bar.setValue(100)
            self._check_cache_status()

            if result == "cached":
                self._set_status("Mesh data cached. Parameter changes now execute faster!")
            else:
                self._set_status("Cache operation completed: " + str(result))

        except Exception as e:
            self._set_status("Cache error: {}".format(str(e)), error=True)

        self.progress_bar.setValue(0)

    def _on_clear_cache(self):
        """Clear the mesh cache."""
        try:
            mel.eval('demBonesCache -clear')
            self._check_cache_status()
            self._set_status("Cache cleared")
        except Exception as e:
            self._set_status("Error clearing cache: {}".format(str(e)), error=True)

    def _on_execute(self):
        """Execute DemBones command."""
        if not self.current_mesh:
            self._set_status("No mesh selected", error=True)
            return

        if self.executing:
            return

        # Validate frame range
        start = self.start_frame_spin.value()
        end = self.end_frame_spin.value()
        if end <= start:
            self._set_status("Invalid frame range", error=True)
            return

        # Delete existing dembones joints if requested
        if self.delete_existing_cb.isChecked():
            existing = cmds.ls("dembones_joint*", type="joint")
            if existing:
                cmds.delete(existing)
                self._set_status("Deleted {} existing joints".format(len(existing)))
                QApplication.processEvents()

        # Build and execute command
        self.executing = True
        self._set_status("Executing DemBones..." + (" (using cache)" if self.cached else " (sampling mesh)"))
        self.progress_bar.setValue(50)
        self.execute_btn.setEnabled(False)
        QApplication.processEvents()

        try:
            cmd = 'demBones -b {bones} -sf {sf} -ef {ef} -mi {mi} -i {iters} "{mesh}"'.format(
                bones=self.bone_spin.value(),
                sf=start,
                ef=end,
                mi=self.infl_spin.value(),
                iters=self.iter_spin.value(),
                mesh=self.current_mesh
            )

            mel.eval(cmd)

            self.progress_bar.setValue(100)
            self._set_status("Completed: {} bones created. Adjust parameters and execute again!".format(
                self.bone_spin.value()))

        except Exception as e:
            self._set_status("Error: {}".format(str(e)), error=True)

        finally:
            self.executing = False
            self.execute_btn.setEnabled(True)
            QTimer.singleShot(2000, lambda: self.progress_bar.setValue(0))

    def _set_status(self, message, error=False):
        """Update status label."""
        self.status_label.setText(message)
        if error:
            self.status_label.setStyleSheet("QLabel { color: red; }")
        else:
            self.status_label.setStyleSheet("QLabel { color: green; }")

    def closeEvent(self, event):
        """Handle close event."""
        DemBonesUI._instance = None
        super().closeEvent(event)


def show():
    """Show the DemBones UI."""
    return DemBonesUI.show_ui()


def close():
    """Close the DemBones UI."""
    DemBonesUI.close_ui()


# For testing in Maya Script Editor:
# import dembones_ui
# dembones_ui.show()
