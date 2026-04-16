import sys
import time
import serial
import serial.tools.list_ports
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QPushButton, QLabel, QSlider, 
                             QComboBox, QGroupBox, QCheckBox)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QPalette, QColor

class WhipInstrument(QMainWindow):
    def __init__(self):
        super().__init__()
        self.serial_port = None
        
        # Performance state
        self.is_playing = False
        self.current_mode = "manual"  # manual, pulse, tremolo, build
        
        # Real-time parameters
        self.angle = 90
        self.speed = 2000
        self.intensity = 50  # 0-100 scale
        
        # Timers for auto modes
        self.pulse_timer = QTimer()
        self.pulse_timer.timeout.connect(self.pulse_tick)
        self.pulse_direction = 1
        
        self.setWindowTitle("Whip Instrument - Real-time Control")
        self.setGeometry(100, 100, 1000, 700)
        self.setStyleSheet("background: #0a0a0a; color: white;")
        
        self.init_ui()
        
    def init_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        
        # Connection at top
        conn_layout = QHBoxLayout()
        self.port_combo = QComboBox()
        self.refresh_ports()
        conn_layout.addWidget(QLabel("Port:"))
        conn_layout.addWidget(self.port_combo)
        
        btn_connect = QPushButton("Connect")
        btn_connect.clicked.connect(self.connect)
        conn_layout.addWidget(btn_connect)
        
        self.status_label = QLabel("Not connected")
        self.status_label.setStyleSheet("color: red; font-weight: bold; padding: 5px;")
        conn_layout.addWidget(self.status_label)
        conn_layout.addStretch()
        layout.addLayout(conn_layout)
        
        # Main performance area
        main_layout = QHBoxLayout()
        
        # Left: Performance controls
        left_panel = QWidget()
        left_panel.setMaximumWidth(400)
        left_layout = QVBoxLayout(left_panel)
        
        # PLAY/STOP (big button)
        self.play_btn = QPushButton("▶ START")
        self.play_btn.setStyleSheet("""
            QPushButton {
                background: #4CAF50;
                color: white;
                font-size: 32pt;
                font-weight: bold;
                padding: 30px;
                border-radius: 15px;
            }
            QPushButton:pressed {
                background: #45a049;
            }
        """)
        self.play_btn.clicked.connect(self.toggle_play)
        left_layout.addWidget(self.play_btn)
        
        # Mode selector
        mode_group = QGroupBox("Performance Mode")
        mode_layout = QVBoxLayout()
        
        modes = [
            ("Manual", "manual", "Direct control - you trigger each movement"),
            ("Pulse", "pulse", "Automatic back-and-forth rhythm"),
            ("Tremolo", "tremolo", "Rapid oscillation"),
            ("Build", "build", "Gradually increasing intensity"),
        ]
        
        for name, mode_id, desc in modes:
            btn = QPushButton(f"{name}\n{desc}")
            btn.setCheckable(True)
            btn.setStyleSheet("""
                QPushButton {
                    padding: 10px;
                    text-align: left;
                    background: #1a1a1a;
                    border: 2px solid #333;
                    border-radius: 5px;
                }
                QPushButton:checked {
                    background: #7c4dff;
                    border: 2px solid #9575ff;
                }
            """)
            btn.clicked.connect(lambda checked, m=mode_id: self.set_mode(m))
            if mode_id == "manual":
                btn.setChecked(True)
            mode_layout.addWidget(btn)
            setattr(self, f"mode_btn_{mode_id}", btn)
        
        mode_group.setLayout(mode_layout)
        left_layout.addWidget(mode_group)
        
        main_layout.addWidget(left_panel)
        
        # Right: Expression controls (the "instrument")
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        
        # Angle (vertical slider - like volume fader)
        angle_group = QGroupBox("ANGLE (Range of Motion)")
        angle_layout = QHBoxLayout()
        
        self.angle_slider = QSlider(Qt.Vertical)
        self.angle_slider.setRange(5, 180)
        self.angle_slider.setValue(90)
        self.angle_slider.setTickPosition(QSlider.TicksLeft)
        self.angle_slider.setTickInterval(30)
        self.angle_slider.valueChanged.connect(self.angle_changed)
        self.angle_slider.setStyleSheet("""
            QSlider::groove:vertical {
                background: #1a1a1a;
                width: 20px;
                border-radius: 10px;
            }
            QSlider::handle:vertical {
                background: #ff3860;
                height: 40px;
                margin: 0 -10px;
                border-radius: 20px;
            }
        """)
        angle_layout.addWidget(self.angle_slider)
        
        angle_labels = QVBoxLayout()
        self.angle_value_label = QLabel("90°")
        self.angle_value_label.setStyleSheet("font-size: 24pt; font-weight: bold;")
        angle_labels.addWidget(self.angle_value_label)
        angle_labels.addWidget(QLabel("180°\n\n\n\n90°\n\n\n\n0°"))
        angle_layout.addLayout(angle_labels)
        
        angle_group.setLayout(angle_layout)
        right_layout.addWidget(angle_group)
        
        # Speed (horizontal slider - like tempo)
        speed_group = QGroupBox("SPEED (Tempo)")
        speed_layout = QVBoxLayout()
        
        self.speed_value_label = QLabel("2000 sps (60 rpm)")
        self.speed_value_label.setStyleSheet("font-size: 18pt; font-weight: bold; color: #7c4dff;")
        speed_layout.addWidget(self.speed_value_label)
        
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setRange(500, 8000)
        self.speed_slider.setValue(2000)
        self.speed_slider.setTickPosition(QSlider.TicksBelow)
        self.speed_slider.setTickInterval(1000)
        self.speed_slider.valueChanged.connect(self.speed_changed)
        self.speed_slider.setStyleSheet("""
            QSlider::groove:horizontal {
                background: #1a1a1a;
                height: 15px;
                border-radius: 7px;
            }
            QSlider::handle:horizontal {
                background: #7c4dff;
                width: 30px;
                margin: -8px 0;
                border-radius: 15px;
            }
        """)
        speed_layout.addWidget(self.speed_slider)
        
        speed_presets = QHBoxLayout()
        for label, val in [("Slow", 1000), ("Medium", 2000), ("Fast", 4000), ("Extreme", 6000)]:
            btn = QPushButton(label)
            btn.clicked.connect(lambda _, v=val: self.speed_slider.setValue(v))
            speed_presets.addWidget(btn)
        speed_layout.addLayout(speed_presets)
        
        speed_group.setLayout(speed_layout)
        right_layout.addWidget(speed_group)
        
        # Intensity (affects multiple parameters at once)
        intensity_group = QGroupBox("INTENSITY (Global Feel)")
        intensity_layout = QVBoxLayout()
        
        self.intensity_value_label = QLabel("50%")
        self.intensity_value_label.setStyleSheet("font-size: 18pt; font-weight: bold; color: #ff9800;")
        intensity_layout.addWidget(self.intensity_value_label)
        
        self.intensity_slider = QSlider(Qt.Horizontal)
        self.intensity_slider.setRange(0, 100)
        self.intensity_slider.setValue(50)
        self.intensity_slider.valueChanged.connect(self.intensity_changed)
        self.intensity_slider.setStyleSheet("""
            QSlider::groove:horizontal {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #1a1a1a, stop:0.5 #ff9800, stop:1 #ff3860);
                height: 20px;
                border-radius: 10px;
            }
            QSlider::handle:horizontal {
                background: white;
                width: 40px;
                margin: -10px 0;
                border-radius: 20px;
                border: 3px solid #ff9800;
            }
        """)
        intensity_layout.addWidget(self.intensity_slider)
        intensity_layout.addWidget(QLabel("Affects: Speed multiplier, braking, rhythm"))
        
        intensity_group.setLayout(intensity_layout)
        right_layout.addWidget(intensity_group)
        
        # Manual triggers (only active in manual mode)
        trigger_group = QGroupBox("Manual Triggers (Manual Mode Only)")
        trigger_layout = QHBoxLayout()
        
        self.btn_left = QPushButton("⬅ LEFT")
        self.btn_left.setStyleSheet("padding: 20px; font-size: 16pt; background: #2196F3;")
        self.btn_left.clicked.connect(self.trigger_left)
        trigger_layout.addWidget(self.btn_left)
        
        self.btn_right = QPushButton("RIGHT ➡")
        self.btn_right.setStyleSheet("padding: 20px; font-size: 16pt; background: #2196F3;")
        self.btn_right.clicked.connect(self.trigger_right)
        trigger_layout.addWidget(self.btn_right)
        
        self.btn_neutral = QPushButton("⬤ CENTER")
        self.btn_neutral.setStyleSheet("padding: 20px; font-size: 16pt; background: #555;")
        self.btn_neutral.clicked.connect(self.trigger_neutral)
        trigger_layout.addWidget(self.btn_neutral)
        
        trigger_group.setLayout(trigger_layout)
        right_layout.addWidget(trigger_group)
        
        # Performance options
        options_group = QGroupBox("Performance Options")
        options_layout = QHBoxLayout()
        
        self.brake_check = QCheckBox("Smooth Braking")
        self.brake_check.setChecked(True)
        self.brake_check.stateChanged.connect(self.update_settings)
        options_layout.addWidget(self.brake_check)
        
        self.softstart_check = QCheckBox("Soft Start")
        self.softstart_check.setChecked(True)
        self.softstart_check.stateChanged.connect(self.update_settings)
        options_layout.addWidget(self.softstart_check)
        
        options_group.setLayout(options_layout)
        right_layout.addWidget(options_group)
        
        right_layout.addStretch()
        
        main_layout.addWidget(right_panel)
        layout.addLayout(main_layout)
        
        # Status bar at bottom
        self.perf_status = QLabel("Ready to perform. Connect and press START.")
        self.perf_status.setStyleSheet("""
            background: #1a1a1a; 
            padding: 10px; 
            font-size: 12pt;
            border-radius: 5px;
        """)
        layout.addWidget(self.perf_status)
        
    def refresh_ports(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combo.clear()
        self.port_combo.addItems(ports)
    
    def connect(self):
        try:
            port = self.port_combo.currentText()
            self.serial_port = serial.Serial(port, 115200, timeout=1)
            time.sleep(2)
            self.status_label.setText("Connected ✓")
            self.status_label.setStyleSheet("color: #4CAF50; font-weight: bold; padding: 5px;")
            
            # Initialize motor
            self.send_cmd("on")
            self.update_settings()
            
        except Exception as e:
            self.status_label.setText(f"Error: {e}")
            self.status_label.setStyleSheet("color: red; font-weight: bold; padding: 5px;")
    
    def send_cmd(self, cmd):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.write((cmd + '\n').encode())
            time.sleep(0.01)
    
    def toggle_play(self):
        self.is_playing = not self.is_playing
        
        if self.is_playing:
            self.play_btn.setText("⏹ STOP")
            self.play_btn.setStyleSheet("""
                QPushButton {
                    background: #ff3860;
                    color: white;
                    font-size: 32pt;
                    font-weight: bold;
                    padding: 30px;
                    border-radius: 15px;
                }
            """)
            self.start_performance()
        else:
            self.play_btn.setText("▶ START")
            self.play_btn.setStyleSheet("""
                QPushButton {
                    background: #4CAF50;
                    color: white;
                    font-size: 32pt;
                    font-weight: bold;
                    padding: 30px;
                    border-radius: 15px;
                }
            """)
            self.stop_performance()
    
    def set_mode(self, mode):
        self.current_mode = mode
        
        # Update button states
        for m in ["manual", "pulse", "tremolo", "build"]:
            btn = getattr(self, f"mode_btn_{m}")
            btn.setChecked(m == mode)
        
        # Enable/disable manual triggers
        manual = (mode == "manual")
        self.btn_left.setEnabled(manual)
        self.btn_right.setEnabled(manual)
        self.btn_neutral.setEnabled(manual)
        
        self.perf_status.setText(f"Mode: {mode.upper()}")
    
    def start_performance(self):
        if not self.serial_port:
            self.perf_status.setText("❌ Not connected! Connect first.")
            self.is_playing = False
            return
        
        self.send_cmd("on")
        
        if self.current_mode == "pulse":
            # Start pulsing
            bpm = 60 * self.speed / 2000 * (self.intensity / 50)
            interval_ms = int(60000 / bpm)
            self.pulse_timer.start(interval_ms)
            self.perf_status.setText(f"🎵 Pulsing at {bpm:.0f} BPM")
            
        elif self.current_mode == "tremolo":
            # Rapid oscillation
            self.pulse_timer.start(200)  # Fast!
            self.perf_status.setText("⚡ Tremolo mode active")
            
        elif self.current_mode == "build":
            # Gradual build
            self.build_phase = 0
            self.pulse_timer.start(2000)
            self.perf_status.setText("📈 Building intensity...")
            
        elif self.current_mode == "manual":
            self.perf_status.setText("🎹 Manual control active - use triggers")
    
    def stop_performance(self):
        self.pulse_timer.stop()
        self.send_cmd("neutral")
        self.perf_status.setText("⏸ Performance stopped")
    
    def pulse_tick(self):
        """Called by timer for automatic modes"""
        if self.current_mode == "pulse" or self.current_mode == "tremolo":
            if self.pulse_direction == 1:
                self.send_cmd("left")
                self.pulse_direction = -1
            else:
                self.send_cmd("right")
                self.pulse_direction = 1
                
        elif self.current_mode == "build":
            # Gradual increase
            self.build_phase += 1
            new_angle = min(30 + (self.build_phase * 15), 180)
            self.angle_slider.setValue(new_angle)
            
            if self.build_phase % 2 == 0:
                self.send_cmd("left")
            else:
                self.send_cmd("right")
            
            if new_angle >= 180:
                self.build_phase = 0  # Reset
    
    def trigger_left(self):
        if self.is_playing and self.current_mode == "manual":
            self.send_cmd("left")
            self.perf_status.setText("⬅ Triggered LEFT")
    
    def trigger_right(self):
        if self.is_playing and self.current_mode == "manual":
            self.send_cmd("right")
            self.perf_status.setText("➡ Triggered RIGHT")
    
    def trigger_neutral(self):
        if self.is_playing and self.current_mode == "manual":
            self.send_cmd("neutral")
            self.perf_status.setText("⬤ Returned to CENTER")
    
    def angle_changed(self, value):
        self.angle = value
        self.angle_value_label.setText(f"{value}°")
        self.send_cmd(f"angle {value}")
    
    def speed_changed(self, value):
        self.speed = value
        rpm = int((value / 400.0) * 60 / 5)  # Output RPM with 5:1 gearbox
        self.speed_value_label.setText(f"{value} sps ({rpm} rpm)")
        self.send_cmd(f"speed {value}")
        
        # Update pulse timing if in pulse mode
        if self.is_playing and self.current_mode == "pulse":
            bpm = 60 * value / 2000 * (self.intensity / 50)
            interval_ms = int(60000 / bpm)
            self.pulse_timer.setInterval(interval_ms)
    
    def intensity_changed(self, value):
        self.intensity = value
        self.intensity_value_label.setText(f"{value}%")
        
        # Intensity affects speed multiplier
        if value > 70:
            # High intensity = disable braking for sharp movements
            self.brake_check.setChecked(False)
            self.perf_status.setText("⚡ HIGH INTENSITY - Sharp movements!")
        elif value < 30:
            # Low intensity = smooth and gentle
            self.brake_check.setChecked(True)
            self.softstart_check.setChecked(True)
            self.perf_status.setText("🌊 Low intensity - Smooth and gentle")
    
    def update_settings(self):
        if self.serial_port:
            self.send_cmd("brakeon" if self.brake_check.isChecked() else "brakeoff")
            self.send_cmd("softstarton" if self.softstart_check.isChecked() else "softstartoff")

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = WhipInstrument()
    window.show()
    sys.exit(app.exec_())
    