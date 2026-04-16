import sys
import json
import time
import threading
import numpy as np
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QPushButton, QListWidget, QLabel, 
                             QSpinBox, QComboBox, QFileDialog, QMessageBox, 
                             QGroupBox, QCheckBox, QSlider, QLineEdit, QDialog,
                             QDialogButtonBox)
from PyQt5.QtCore import Qt, pyqtSignal
import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
import matplotlib.animation as animation

class MplCanvas(FigureCanvasQTAgg):
    def __init__(self):
        self.fig = Figure(figsize=(8, 8))
        self.ax_arm = self.fig.add_subplot(211)
        self.ax_timeline = self.fig.add_subplot(212)
        self.fig.tight_layout(pad=3)
        super().__init__(self.fig)
        
        self.setup_plots()
        
    def setup_plots(self):
        # Arm
        self.ax_arm.clear()
        self.ax_arm.set_xlim(-2, 2)
        self.ax_arm.set_ylim(-0.5, 2)
        self.ax_arm.set_aspect('equal')
        self.ax_arm.set_title('Arm Position', fontsize=14, weight='bold')
        self.ax_arm.grid(True, alpha=0.3)
        
        base = plt.Circle((0, 0), 0.1, color='gray')
        self.ax_arm.add_patch(base)
        
        self.arm_line, = self.ax_arm.plot([0, 0], [0, 1.5], 'b-', linewidth=6)
        self.arm_tip = plt.Circle((0, 1.5), 0.1, color='red')
        self.ax_arm.add_patch(self.arm_tip)
        self.angle_text = self.ax_arm.text(0, -0.3, '0°', ha='center', fontsize=16, weight='bold')
        
        # Timeline
        self.ax_timeline.clear()
        self.ax_timeline.set_xlabel('Time (seconds)', fontsize=12)
        self.ax_timeline.set_ylabel('Angle (degrees)', fontsize=12)
        self.ax_timeline.set_title('Movement Timeline', fontsize=14, weight='bold')
        self.ax_timeline.grid(True, alpha=0.3)
        self.ax_timeline.axhline(0, color='black', linewidth=1)
        
        self.draw()

class BulkEditDialog(QDialog):
    def __init__(self, selected_items, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Bulk Edit Selected Steps")
        self.selected_items = selected_items
        
        layout = QVBoxLayout()
        
        info = QLabel(f"Editing {len(selected_items)} selected steps")
        info.setStyleSheet("font-weight: bold; font-size: 12pt;")
        layout.addWidget(info)
        
        # What to edit
        edit_group = QGroupBox("What to edit:")
        edit_layout = QVBoxLayout()
        
        self.edit_speed = QCheckBox("Change speed for all 'Set Speed' steps")
        edit_layout.addWidget(self.edit_speed)
        
        self.speed_spin = QSpinBox()
        self.speed_spin.setRange(50, 10000)
        self.speed_spin.setValue(2000)
        self.speed_spin.setSingleStep(100)
        self.speed_spin.setEnabled(False)
        self.edit_speed.toggled.connect(self.speed_spin.setEnabled)
        edit_layout.addWidget(self.speed_spin)
        
        self.edit_angle = QCheckBox("Change angle for all 'Set Angle' steps")
        edit_layout.addWidget(self.edit_angle)
        
        self.angle_spin = QSpinBox()
        self.angle_spin.setRange(5, 360)
        self.angle_spin.setValue(90)
        self.angle_spin.setEnabled(False)
        self.edit_angle.toggled.connect(self.angle_spin.setEnabled)
        edit_layout.addWidget(self.angle_spin)
        
        self.edit_wait = QCheckBox("Change wait time for all 'Wait' steps")
        edit_layout.addWidget(self.edit_wait)
        
        self.wait_spin = QSpinBox()
        self.wait_spin.setRange(0, 10000)
        self.wait_spin.setValue(1000)
        self.wait_spin.setSingleStep(100)
        self.wait_spin.setEnabled(False)
        self.edit_wait.toggled.connect(self.wait_spin.setEnabled)
        edit_layout.addWidget(self.wait_spin)
        
        self.multiply_wait = QCheckBox("Multiply all wait times by:")
        edit_layout.addWidget(self.multiply_wait)
        
        self.multiply_spin = QSpinBox()
        self.multiply_spin.setRange(1, 10)
        self.multiply_spin.setValue(2)
        self.multiply_spin.setPrefix("×")
        self.multiply_spin.setEnabled(False)
        self.multiply_wait.toggled.connect(self.multiply_spin.setEnabled)
        edit_layout.addWidget(self.multiply_spin)
        
        self.insert_wait_before = QCheckBox("Insert wait BEFORE each selected step:")
        edit_layout.addWidget(self.insert_wait_before)
        
        self.insert_wait_spin = QSpinBox()
        self.insert_wait_spin.setRange(0, 10000)
        self.insert_wait_spin.setValue(500)
        self.insert_wait_spin.setSingleStep(100)
        self.insert_wait_spin.setSuffix(" ms")
        self.insert_wait_spin.setEnabled(False)
        self.insert_wait_before.toggled.connect(self.insert_wait_spin.setEnabled)
        edit_layout.addWidget(self.insert_wait_spin)
        
        self.delete_selected = QCheckBox("Delete all selected steps")
        self.delete_selected.setStyleSheet("color: red;")
        edit_layout.addWidget(self.delete_selected)
        
        edit_group.setLayout(edit_layout)
        layout.addWidget(edit_group)
        
        # Buttons
        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)
        
        self.setLayout(layout)

class SequenceBuilder(QMainWindow):
    def __init__(self):
        super().__init__()
        self.sequence = []
        self.serial_port = None
        self.animation = None
        self.timeline_data = None
        
        # Global settings that affect all moves
        self.global_angle = 90
        self.global_speed = 2000
        self.global_braking = True
        self.global_softstart = True
        
        self.setWindowTitle("Stepper Sequence Builder - Multi-Edit Edition")
        self.setGeometry(100, 100, 1400, 900)
        
        self.init_ui()
        
    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        
        # Left panel
        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)
        left_panel.setMaximumWidth(450)
        
        # Connection
        conn_group = QGroupBox("Connection")
        conn_layout = QHBoxLayout()
        
        self.port_combo = QComboBox()
        self.refresh_ports()
        conn_layout.addWidget(self.port_combo)
        
        connect_btn = QPushButton("Connect")
        connect_btn.clicked.connect(self.connect)
        conn_layout.addWidget(connect_btn)
        
        self.status_label = QLabel("Not connected")
        self.status_label.setStyleSheet("color: red; font-weight: bold;")
        conn_layout.addWidget(self.status_label)
        
        conn_group.setLayout(conn_layout)
        left_layout.addWidget(conn_group)
        
        # Global Settings
        settings_group = QGroupBox("Global Settings (affects all moves)")
        settings_layout = QVBoxLayout()
        
        # Angle
        angle_layout = QHBoxLayout()
        angle_layout.addWidget(QLabel("Default Angle:"))
        self.global_angle_spin = QSpinBox()
        self.global_angle_spin.setRange(5, 360)
        self.global_angle_spin.setValue(90)
        self.global_angle_spin.valueChanged.connect(self.update_global_settings)
        angle_layout.addWidget(self.global_angle_spin)
        angle_layout.addWidget(QLabel("°"))
        settings_layout.addLayout(angle_layout)
        
        # Speed
        speed_layout = QHBoxLayout()
        speed_layout.addWidget(QLabel("Default Speed:"))
        self.global_speed_spin = QSpinBox()
        self.global_speed_spin.setRange(50, 10000)
        self.global_speed_spin.setValue(2000)
        self.global_speed_spin.setSingleStep(100)
        self.global_speed_spin.valueChanged.connect(self.update_global_settings)
        speed_layout.addWidget(self.global_speed_spin)
        speed_layout.addWidget(QLabel("sps"))
        settings_layout.addLayout(speed_layout)
        
        # Checkboxes
        check_layout = QHBoxLayout()
        self.global_brake_check = QCheckBox("Braking")
        self.global_brake_check.setChecked(True)
        self.global_brake_check.stateChanged.connect(self.update_global_settings)
        check_layout.addWidget(self.global_brake_check)
        
        self.global_softstart_check = QCheckBox("Soft-start")
        self.global_softstart_check.setChecked(True)
        self.global_softstart_check.stateChanged.connect(self.update_global_settings)
        check_layout.addWidget(self.global_softstart_check)
        settings_layout.addLayout(check_layout)
        
        settings_group.setLayout(settings_layout)
        left_layout.addWidget(settings_group)
        
        # Quick patterns
        pattern_group = QGroupBox("Quick Patterns")
        pattern_layout = QVBoxLayout()
        
        btn_pendulum = QPushButton("Add Pendulum (5 swings)")
        btn_pendulum.clicked.connect(self.add_pendulum_pattern)
        pattern_layout.addWidget(btn_pendulum)
        
        btn_buildup = QPushButton("Add Gradual Build-up")
        btn_buildup.clicked.connect(self.add_buildup_pattern)
        pattern_layout.addWidget(btn_buildup)
        
        btn_whipcrack = QPushButton("Add Whip Crack")
        btn_whipcrack.clicked.connect(self.add_whipcrack_pattern)
        pattern_layout.addWidget(btn_whipcrack)
        
        pattern_group.setLayout(pattern_layout)
        left_layout.addWidget(pattern_group)
        
        # Single step builder
        step_group = QGroupBox("Add Single Step")
        step_layout = QVBoxLayout()
        
        # Step type
        type_layout = QHBoxLayout()
        type_layout.addWidget(QLabel("Type:"))
        self.step_type_combo = QComboBox()
        self.step_type_combo.addItems([
            "Move Left", "Move Right", "Neutral", "Wait",
            "Set Speed", "Set Angle", "Set Braking", "Set Soft-start",
            "Motor ON", "Motor OFF"
        ])
        self.step_type_combo.currentTextChanged.connect(self.update_step_fields)
        type_layout.addWidget(self.step_type_combo)
        step_layout.addLayout(type_layout)
        
        # Value input (dynamic based on type)
        self.value_layout = QHBoxLayout()
        self.value_label = QLabel("Wait time (ms):")
        self.value_layout.addWidget(self.value_label)
        
        self.value_spin = QSpinBox()
        self.value_spin.setRange(0, 10000)
        self.value_spin.setValue(1000)
        self.value_layout.addWidget(self.value_spin)
        
        step_layout.addLayout(self.value_layout)
        
        btn_add_step = QPushButton("Add Step")
        btn_add_step.clicked.connect(self.add_single_step)
        step_layout.addWidget(btn_add_step)
        
        step_group.setLayout(step_layout)
        left_layout.addWidget(step_group)
        
        # Sequence list
        seq_group = QGroupBox("Sequence (Ctrl+Click or Shift+Click to multi-select)")
        seq_layout = QVBoxLayout()
        
        self.seq_list = QListWidget()
        self.seq_list.setStyleSheet("font-family: monospace; font-size: 11pt;")
        self.seq_list.setSelectionMode(QListWidget.ExtendedSelection)  # Enable multi-select
        seq_layout.addWidget(self.seq_list)
        
        # Sequence controls
        seq_btn_layout1 = QHBoxLayout()
        
        btn_bulk_edit = QPushButton("✏️ Bulk Edit")
        btn_bulk_edit.setStyleSheet("background-color: #FF9800; color: white; font-weight: bold;")
        btn_bulk_edit.clicked.connect(self.bulk_edit)
        seq_btn_layout1.addWidget(btn_bulk_edit)
        
        btn_remove = QPushButton("Remove")
        btn_remove.clicked.connect(self.remove_step)
        seq_btn_layout1.addWidget(btn_remove)
        
        btn_clear = QPushButton("Clear All")
        btn_clear.clicked.connect(self.clear_sequence)
        seq_btn_layout1.addWidget(btn_clear)
        
        seq_layout.addLayout(seq_btn_layout1)
        
        seq_btn_layout2 = QHBoxLayout()
        
        btn_up = QPushButton("↑")
        btn_up.clicked.connect(self.move_up)
        seq_btn_layout2.addWidget(btn_up)
        
        btn_down = QPushButton("↓")
        btn_down.clicked.connect(self.move_down)
        seq_btn_layout2.addWidget(btn_down)
        
        btn_duplicate = QPushButton("Duplicate")
        btn_duplicate.clicked.connect(self.duplicate_selected)
        seq_btn_layout2.addWidget(btn_duplicate)
        
        seq_layout.addLayout(seq_btn_layout2)
        
        # File controls
        seq_btn_layout3 = QHBoxLayout()
        btn_save = QPushButton("💾 Save")
        btn_save.clicked.connect(self.save_sequence)
        seq_btn_layout3.addWidget(btn_save)
        
        btn_load = QPushButton("📁 Load")
        btn_load.clicked.connect(self.load_sequence)
        seq_btn_layout3.addWidget(btn_load)
        seq_layout.addLayout(seq_btn_layout3)
        
        seq_group.setLayout(seq_layout)
        left_layout.addWidget(seq_group)
        
        main_layout.addWidget(left_panel)
        
        # Right panel - Visualization
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        
        # Control buttons
        control_layout = QHBoxLayout()
        
        btn_preview = QPushButton("▶ Preview Animation")
        btn_preview.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold; padding: 10px;")
        btn_preview.clicked.connect(self.visualize)
        control_layout.addWidget(btn_preview)
        
        btn_stop = QPushButton("⏹ Stop")
        btn_stop.clicked.connect(self.stop_visualization)
        control_layout.addWidget(btn_stop)
        
        btn_run = QPushButton("▶▶ Run on Motor")
        btn_run.setStyleSheet("background-color: #2196F3; color: white; font-weight: bold; padding: 10px;")
        btn_run.clicked.connect(self.run_sequence)
        control_layout.addWidget(btn_run)
        
        right_layout.addLayout(control_layout)
        
        # Stats
        stats_layout = QHBoxLayout()
        self.stats_label = QLabel("Steps: 0 | Duration: 0.0s | Final position: 0°")
        self.stats_label.setStyleSheet("font-size: 12pt; padding: 5px; background: #1a1a2e; border-radius: 5px;")
        stats_layout.addWidget(self.stats_label)
        right_layout.addLayout(stats_layout)
        
        # Matplotlib canvas
        self.canvas = MplCanvas()
        right_layout.addWidget(self.canvas)
        
        main_layout.addWidget(right_panel)
        
        # Initialize
        self.update_step_fields()
        
    def update_step_fields(self):
        """Update value fields based on selected step type"""
        step_type = self.step_type_combo.currentText()
        
        # Clear current widgets
        while self.value_layout.count():
            item = self.value_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()
        
        if step_type == "Wait":
            self.value_label = QLabel("Duration (ms):")
            self.value_layout.addWidget(self.value_label)
            self.value_spin = QSpinBox()
            self.value_spin.setRange(0, 10000)
            self.value_spin.setValue(1000)
            self.value_spin.setSingleStep(100)
            self.value_layout.addWidget(self.value_spin)
            
        elif step_type == "Set Speed":
            self.value_label = QLabel("Speed (sps):")
            self.value_layout.addWidget(self.value_label)
            self.value_spin = QSpinBox()
            self.value_spin.setRange(50, 10000)
            self.value_spin.setValue(self.global_speed)
            self.value_spin.setSingleStep(100)
            self.value_layout.addWidget(self.value_spin)
            
        elif step_type == "Set Angle":
            self.value_label = QLabel("Angle (°):")
            self.value_layout.addWidget(self.value_label)
            self.value_spin = QSpinBox()
            self.value_spin.setRange(5, 360)
            self.value_spin.setValue(self.global_angle)
            self.value_layout.addWidget(self.value_spin)
            
        elif step_type in ["Set Braking", "Set Soft-start"]:
            self.value_label = QLabel("Enable:")
            self.value_layout.addWidget(self.value_label)
            self.value_check = QCheckBox()
            self.value_check.setChecked(True)
            self.value_layout.addWidget(self.value_check)
            
        else:
            # No value needed
            self.value_label = QLabel("(no parameters)")
            self.value_layout.addWidget(self.value_label)
    
    def update_global_settings(self):
        self.global_angle = self.global_angle_spin.value()
        self.global_speed = self.global_speed_spin.value()
        self.global_braking = self.global_brake_check.isChecked()
        self.global_softstart = self.global_softstart_check.isChecked()
        
        # Re-visualize if sequence exists
        if self.sequence:
            self.visualize()
    
    def add_single_step(self):
        step_type = self.step_type_combo.currentText()
        
        if step_type == "Move Left":
            step = ("left", None)
        elif step_type == "Move Right":
            step = ("right", None)
        elif step_type == "Neutral":
            step = ("neutral", None)
        elif step_type == "Wait":
            step = ("wait", self.value_spin.value())
        elif step_type == "Set Speed":
            step = ("speed", self.value_spin.value())
        elif step_type == "Set Angle":
            step = ("angle", self.value_spin.value())
        elif step_type == "Set Braking":
            step = ("brakeon" if self.value_check.isChecked() else "brakeoff", None)
        elif step_type == "Set Soft-start":
            step = ("softstarton" if self.value_check.isChecked() else "softstartoff", None)
        elif step_type == "Motor ON":
            step = ("on", None)
        elif step_type == "Motor OFF":
            step = ("off", None)
        
        self.sequence.append(step)
        self.update_sequence_display()
        self.visualize()
    
    def bulk_edit(self):
        """Open bulk edit dialog for selected items"""
        selected_indices = [item.row() for item in self.seq_list.selectedIndexes()]
        
        if not selected_indices:
            QMessageBox.warning(self, "No Selection", "Please select steps to edit (Ctrl+Click for multiple)")
            return
        
        dialog = BulkEditDialog(selected_indices, self)
        
        if dialog.exec_() == QDialog.Accepted:
            new_sequence = []
            offset = 0  # Track index changes from insertions
            
            for i, (cmd, val) in enumerate(self.sequence):
                adjusted_i = i + offset
                
                # Check if this index was selected
                if i in selected_indices:
                    # Delete if requested
                    if dialog.delete_selected.isChecked():
                        continue  # Skip this step
                    
                    # Insert wait before if requested
                    if dialog.insert_wait_before.isChecked():
                        new_sequence.append(("wait", dialog.insert_wait_spin.value()))
                        offset += 1
                    
                    # Modify the step based on its type
                    if cmd == "speed" and dialog.edit_speed.isChecked():
                        new_sequence.append(("speed", dialog.speed_spin.value()))
                    elif cmd == "angle" and dialog.edit_angle.isChecked():
                        new_sequence.append(("angle", dialog.angle_spin.value()))
                    elif cmd == "wait":
                        if dialog.edit_wait.isChecked():
                            new_sequence.append(("wait", dialog.wait_spin.value()))
                        elif dialog.multiply_wait.isChecked():
                            new_sequence.append(("wait", val * dialog.multiply_spin.value()))
                        else:
                            new_sequence.append((cmd, val))
                    else:
                        new_sequence.append((cmd, val))
                else:
                    # Not selected, keep as is
                    new_sequence.append((cmd, val))
            
            self.sequence = new_sequence
            self.update_sequence_display()
            self.visualize()
            
            QMessageBox.information(self, "Bulk Edit Complete", 
                                   f"Modified {len(selected_indices)} steps")
    
    def duplicate_selected(self):
        """Duplicate selected steps"""
        selected_indices = sorted([item.row() for item in self.seq_list.selectedIndexes()], reverse=True)
        
        if not selected_indices:
            QMessageBox.warning(self, "No Selection", "Please select steps to duplicate")
            return
        
        # Insert duplicates right after each selected item
        for idx in selected_indices:
            self.sequence.insert(idx + 1, self.sequence[idx])
        
        self.update_sequence_display()
        self.visualize()
        QMessageBox.information(self, "Duplicated", f"Duplicated {len(selected_indices)} steps")
    
    def add_pendulum_pattern(self):
        """Add a pendulum pattern: 5 swings back and forth"""
        steps = [
            ("on", None),
            ("angle", self.global_angle),
            ("speed", self.global_speed),
        ]
        
        for i in range(5):
            steps.extend([
                ("left", None),
                ("wait", 500),
                ("right", None),
                ("wait", 500),
            ])
        
        steps.append(("neutral", None))
        
        self.sequence.extend(steps)
        self.update_sequence_display()
        self.visualize()
        QMessageBox.information(self, "Pattern Added", "Pendulum pattern added (5 swings)")
    
    def add_buildup_pattern(self):
        """Add gradual build-up pattern: increasing angles"""
        steps = [
            ("on", None),
            ("speed", self.global_speed),
        ]
        
        for angle in [45, 90, 135, 180]:
            steps.extend([
                ("angle", angle),
                ("left", None),
                ("wait", 1000),
                ("right", None),
                ("wait", 1000),
            ])
        
        steps.append(("neutral", None))
        
        self.sequence.extend(steps)
        self.update_sequence_display()
        self.visualize()
        QMessageBox.information(self, "Pattern Added", "Gradual build-up pattern added")
    
    def add_whipcrack_pattern(self):
        """Add whip crack: fast swing with sharp reversal"""
        steps = [
            ("on", None),
            ("softstarton", None),
            ("brakeon", None),
            ("angle", 180),
            ("speed", 6000),
            ("left", None),
            ("wait", 100),
            ("brakeoff", None),  # Sharp stop for crack
            ("right", None),
            ("wait", 2000),
            ("brakeon", None),  # Re-enable for safety
            ("neutral", None),
        ]
        
        self.sequence.extend(steps)
        self.update_sequence_display()
        self.visualize()
        QMessageBox.information(self, "Pattern Added", "Whip crack pattern added (CAUTION: High stress on gearbox!)")
    
    def refresh_ports(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combo.clear()
        self.port_combo.addItems(ports)
        
    def connect(self):
        try:
            port = self.port_combo.currentText()
            self.serial_port = serial.Serial(port, 115200, timeout=1)
            time.sleep(2)
            self.status_label.setText("Connected")
            self.status_label.setStyleSheet("color: green; font-weight: bold;")
            QMessageBox.information(self, "Success", f"Connected to {port}")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Connection failed: {e}")
    
    def update_sequence_display(self):
        self.seq_list.clear()
        for i, (cmd, val) in enumerate(self.sequence):
            if val is not None:
                text = f"{i+1}. {cmd} {val}"
            else:
                text = f"{i+1}. {cmd}"
            self.seq_list.addItem(text)
    
    def remove_step(self):
        selected_indices = sorted([item.row() for item in self.seq_list.selectedIndexes()], reverse=True)
        
        if not selected_indices:
            return
        
        for idx in selected_indices:
            del self.sequence[idx]
        
        self.update_sequence_display()
        self.visualize()
    
    def move_up(self):
        selected_indices = sorted([item.row() for item in self.seq_list.selectedIndexes()])
        
        if not selected_indices or selected_indices[0] == 0:
            return
        
        for idx in selected_indices:
            self.sequence[idx], self.sequence[idx-1] = self.sequence[idx-1], self.sequence[idx]
        
        self.update_sequence_display()
        # Re-select moved items
        for idx in selected_indices:
            self.seq_list.item(idx - 1).setSelected(True)
        self.visualize()
    
    def move_down(self):
        selected_indices = sorted([item.row() for item in self.seq_list.selectedIndexes()], reverse=True)
        
        if not selected_indices or selected_indices[0] >= len(self.sequence) - 1:
            return
        
        for idx in selected_indices:
            self.sequence[idx], self.sequence[idx+1] = self.sequence[idx+1], self.sequence[idx]
        
        self.update_sequence_display()
        # Re-select moved items
        for idx in selected_indices:
            self.seq_list.item(idx + 1).setSelected(True)
        self.visualize()
    
    def clear_sequence(self):
        reply = QMessageBox.question(self, 'Clear', 'Clear entire sequence?',
                                     QMessageBox.Yes | QMessageBox.No)
        if reply == QMessageBox.Yes:
            self.sequence = []
            self.update_sequence_display()
            self.canvas.setup_plots()
            self.stats_label.setText("Steps: 0 | Duration: 0.0s | Final position: 0°")
    
    def save_sequence(self):
        filename, _ = QFileDialog.getSaveFileName(self, "Save Sequence", "", "JSON Files (*.json)")
        if filename:
            with open(filename, 'w') as f:
                json.dump(self.sequence, f, indent=2)
            QMessageBox.information(self, "Saved", f"Sequence saved to {filename}")
    
    def load_sequence(self):
        filename, _ = QFileDialog.getOpenFileName(self, "Load Sequence", "", "JSON Files (*.json)")
        if filename:
            with open(filename, 'r') as f:
                self.sequence = json.load(f)
            self.update_sequence_display()
            self.visualize()
            QMessageBox.information(self, "Loaded", f"Sequence loaded from {filename}")
    
    def parse_sequence(self):
        events = []
        current_time = 0
        current_pos = 0
        current_angle = self.global_angle
        current_speed = self.global_speed
        current_braking = self.global_braking
        current_softstart = self.global_softstart
        
        for cmd, value in self.sequence:
            if cmd == 'left':
                target_pos = current_angle
                move_time = self._calc_move_time(current_pos, target_pos, current_speed, 
                                                 current_braking, current_softstart)
                events.append({
                    'type': 'move',
                    'start_time': current_time,
                    'end_time': current_time + move_time,
                    'start_pos': current_pos,
                    'end_pos': target_pos,
                    'braking': current_braking,
                    'softstart': current_softstart
                })
                current_pos = target_pos
                current_time += move_time
                
            elif cmd == 'right':
                target_pos = -current_angle
                move_time = self._calc_move_time(current_pos, target_pos, current_speed,
                                                 current_braking, current_softstart)
                events.append({
                    'type': 'move',
                    'start_time': current_time,
                    'end_time': current_time + move_time,
                    'start_pos': current_pos,
                    'end_pos': target_pos,
                    'braking': current_braking,
                    'softstart': current_softstart
                })
                current_pos = target_pos
                current_time += move_time
                
            elif cmd == 'neutral':
                target_pos = 0
                move_time = self._calc_move_time(current_pos, target_pos, current_speed,
                                                 current_braking, current_softstart)
                events.append({
                    'type': 'move',
                    'start_time': current_time,
                    'end_time': current_time + move_time,
                    'start_pos': current_pos,
                    'end_pos': target_pos,
                    'braking': current_braking,
                    'softstart': current_softstart
                })
                current_pos = target_pos
                current_time += move_time
                
            elif cmd == 'wait':
                wait_time = value / 1000.0
                events.append({
                    'type': 'wait',
                    'start_time': current_time,
                    'end_time': current_time + wait_time,
                    'position': current_pos
                })
                current_time += wait_time
                
            elif cmd == 'angle':
                current_angle = value
                
            elif cmd == 'speed':
                current_speed = value
                
            elif cmd == 'brakeon':
                current_braking = True
                
            elif cmd == 'brakeoff':
                current_braking = False
                
            elif cmd == 'softstarton':
                current_softstart = True
                
            elif cmd == 'softstartoff':
                current_softstart = False
        
        return {'events': events, 'total_time': current_time, 'final_pos': current_pos}
    
    def _calc_move_time(self, start_pos, end_pos, speed, braking=True, softstart=True):
        distance_deg = abs(end_pos - start_pos)
        motor_angle = distance_deg * 5.0
        steps = (motor_angle / 360.0) * 400
        if steps == 0:
            return 0
        
        base_time = steps / speed
        
        # Add time for acceleration/deceleration
        accel_time = 0
        if softstart:
            accel_time += 0.3  # Soft-start adds time
        if braking:
            accel_time += 0.3  # Braking adds time
        
        return base_time + accel_time
    
    def visualize(self):
        if not self.sequence:
            return
        
        self.stop_visualization()
        self.timeline_data = self.parse_sequence()
        
        self.canvas.setup_plots()
        
        # Update stats
        self.stats_label.setText(
            f"Steps: {len(self.sequence)} | "
            f"Duration: {self.timeline_data['total_time']:.1f}s | "
            f"Final position: {self.timeline_data['final_pos']:.0f}°"
        )
        
        # Draw timeline with different colors for braking/no-braking
        for event in self.timeline_data['events']:
            if event['type'] == 'move':
                times = [event['start_time'], event['end_time']]
                positions = [event['start_pos'], event['end_pos']]
                
                # Color based on braking/softstart
                if event.get('braking') and event.get('softstart'):
                    color = 'blue'  # Smooth
                elif not event.get('braking'):
                    color = 'red'  # Sharp stop
                else:
                    color = 'orange'  # Medium
                
                self.canvas.ax_timeline.plot(times, positions, color=color, linewidth=3, alpha=0.7)
                
            elif event['type'] == 'wait':
                times = [event['start_time'], event['end_time']]
                positions = [event['position'], event['position']]
                self.canvas.ax_timeline.plot(times, positions, 'gray', linewidth=2, linestyle='--')
        
        if self.timeline_data['events']:
            all_times = []
            all_positions = []
            for e in self.timeline_data['events']:
                if e['type'] == 'move':
                    all_times.extend([e['start_time'], e['end_time']])
                    all_positions.extend([e['start_pos'], e['end_pos']])
                elif e['type'] == 'wait':
                    all_times.extend([e['start_time'], e['end_time']])
                    all_positions.extend([e['position'], e['position']])
            
            if all_times:
                self.canvas.ax_timeline.set_xlim(0, max(all_times) * 1.1)
                self.canvas.ax_timeline.set_ylim(min(all_positions) - 20, max(all_positions) + 20)
        
        # Add legend
        from matplotlib.patches import Patch
        legend_elements = [
            Patch(facecolor='blue', label='Smooth (brake+soft)'),
            Patch(facecolor='orange', label='Medium'),
            Patch(facecolor='red', label='Sharp (no brake)'),
            Patch(facecolor='gray', label='Wait')
        ]
        self.canvas.ax_timeline.legend(handles=legend_elements, loc='upper right', fontsize=9)
        
        self.time_marker = self.canvas.ax_timeline.axvline(0, color='black', linewidth=2, linestyle=':')
        
        # Animation
        def update(frame):
            if not self.timeline_data:
                return
            
            total_frames = 200
            current_time = frame * self.timeline_data['total_time'] / total_frames
            
            current_pos = 0
            for event in self.timeline_data['events']:
                if event['type'] == 'move':
                    if event['start_time'] <= current_time <= event['end_time']:
                        progress = (current_time - event['start_time']) / (event['end_time'] - event['start_time'])
                        current_pos = event['start_pos'] + (event['end_pos'] - event['start_pos']) * progress
                        break
                    elif current_time >= event['end_time']:
                        current_pos = event['end_pos']
                elif event['type'] == 'wait':
                    if event['start_time'] <= current_time <= event['end_time']:
                        current_pos = event['position']
                        break
            
            angle_rad = np.radians(-current_pos)
            tip_x = 1.5 * np.sin(angle_rad)
            tip_y = 1.5 * np.cos(angle_rad)
            
            self.canvas.arm_line.set_data([0, tip_x], [0, tip_y])
            self.canvas.arm_tip.center = (tip_x, tip_y)
            self.canvas.angle_text.set_text(f'{current_pos:.0f}°')
            self.time_marker.set_xdata([current_time])
            
            return self.canvas.arm_line, self.canvas.arm_tip, self.canvas.angle_text, self.time_marker
        
        self.animation = animation.FuncAnimation(
            self.canvas.fig, update, frames=200, interval=50,
            blit=False, repeat=True
        )
        
        self.canvas.draw()
    
    def stop_visualization(self):
        if self.animation:
            self.animation.event_source.stop()
            self.animation = None
    
    def run_sequence(self):
        if not self.serial_port:
            QMessageBox.critical(self, "Error", "Not connected to motor!")
            return
        
        if not self.sequence:
            QMessageBox.warning(self, "Warning", "Sequence is empty!")
            return
        
        reply = QMessageBox.question(self, 'Confirm', 'Run sequence on motor?',
                                     QMessageBox.Yes | QMessageBox.No)
        if reply != QMessageBox.Yes:
            return
        
        def execute():
            for cmd, value in self.sequence:
                if cmd in ['left', 'right', 'neutral', 'on', 'off', 
                          'brakeon', 'brakeoff', 'softstarton', 'softstartoff']:
                    self.serial_port.write((cmd + '\n').encode())
                    time.sleep(0.1)
                elif cmd == 'wait':
                    time.sleep(value / 1000.0)
                elif cmd in ['angle', 'speed']:
                    command = f"{cmd} {value}"
                    self.serial_port.write((command + '\n').encode())
                    time.sleep(0.1)
            
            QMessageBox.information(self, "Complete", "Sequence finished!")
        
        threading.Thread(target=execute, daemon=True).start()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = SequenceBuilder()
    window.show()
    sys.exit(app.exec_())