# vim: set ts=4 sw=4 sts=4 et :
import logging

try:
    from PyQt6.QtWidgets import (
        QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
        QGridLayout, QSpinBox, QComboBox,
        # Qt 6 only
        QAbstractSpinBox
    )
    from PyQt6.QtGui import QIcon
    from PyQt6.QtCore import Qt
    PYQT_VERSION = 6
except ImportError:
    from PyQt5.QtWidgets import (
        QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
        QGridLayout, QSpinBox, QComboBox,
    )
    from PyQt5.QtGui import QIcon
    from PyQt5.QtCore import Qt
    PYQT_VERSION = 5


# Set up logger
logger = logging.getLogger(__name__)


class HwSetupTab(QWidget):
    """Tab for configuring controller hardware setup."""

    presets = {
        'SimpleBGC32 \"Regular\"': {
            # TODO: use enum strings for the enums
            "config.hw.main-imu.type": 1,
            "config.hw.main-imu.i2c.bus": 0,
            "config.hw.main-imu.i2c.addr": 0x68,
            "config.hw.frame-imu.type": 1,
            "config.hw.frame-imu.i2c.bus": 0,
            "config.hw.frame-imu.i2c.addr": 0x69,
            "config.hw.motor.0.type": 1,
            "config.hw.motor.1.type": 2,
            "config.hw.motor.2.type": 3,
            "config.hw.encoder.0.type": 0,
            "config.hw.encoder.1.type": 0,
            "config.hw.encoder.2.type": 0,
            "config.vbat.scale": 32030, # TODO
            "config.vbat.lvco": 9900,
        },
        'PilotFly H2': {
            "config.hw.main-imu.type": 1,
            "config.hw.main-imu.i2c.bus": 0,
            "config.hw.main-imu.i2c.addr": 0x68,
            "config.hw.frame-imu.type": 1,
            "config.hw.frame-imu.i2c.bus": 0,
            "config.hw.frame-imu.i2c.addr": 0x69,
            "config.hw.motor.0.type": 1,
            "config.hw.motor.1.type": 5,
            "config.hw.motor.1.sbgc32-id": 0,
            "config.hw.motor.2.type": 5,
            "config.hw.motor.2.sbgc32-id": 3,
            "config.hw.encoder.0.type": 1,
            "config.hw.encoder.0.i2c.bus": 0,
            "config.hw.encoder.0.i2c.addr": 0x36,
            "config.hw.encoder.1.type": 2,
            "config.hw.encoder.1.sbgc32-type": 7,
            "config.hw.encoder.2.type": 2,
            "config.hw.encoder.2.sbgc32-type": 7,
            "config.vbat.scale": 27708,
            "config.vbat.lvco": 13200,
        },
        'STorM32 BGC v1.3': {
            'config.hw.main-imu.type': 'obgc_hw_config_s::obgc_imu_hw_config_s::OBGC_IMU_I2C_MPU6050',
            'config.hw.main-imu.i2c.bus': 0,
            'config.hw.main-imu.i2c.addr': 0x68,
            'config.hw.frame-imu.type': 'obgc_hw_config_s::obgc_imu_hw_config_s::OBGC_IMU_I2C_MPU6050',
            'config.hw.frame-imu.i2c.bus': 1,
            'config.hw.frame-imu.i2c.addr': 0x69,
            'config.hw.motor.0.type': 'obgc_hw_config_s::obgc_motor_hw_config_s::OBGC_MOTOR_DRV_ONBOARD0',
            'config.hw.motor.1.type': 'obgc_hw_config_s::obgc_motor_hw_config_s::OBGC_MOTOR_DRV_ONBOARD1',
            'config.hw.motor.2.type': 'obgc_hw_config_s::obgc_motor_hw_config_s::OBGC_MOTOR_DRV_ONBOARD2',
            'config.vbat.scale': 32030,
            'config.vbat.lvco': 9900,
        },
        'STorM32-NT BGC v1.3': {
            'config.hw.main-imu.type': 'obgc_hw_config_s::obgc_imu_hw_config_s::OBGC_IMU_NT',
            'config.hw.main-imu.nt_id': 'obgc_hw_config_s::obgc_imu_hw_config_s::OBGC_NT_IMU_ID_IMU1',
            'config.hw.frame-imu.type': 'obgc_hw_config_s::obgc_imu_hw_config_s::OBGC_IMU_I2C_MPU6050',
            'config.hw.frame-imu.i2c.bus': 1,
            'config.hw.frame-imu.i2c.addr': 0x69,
            'config.hw.motor.0.type': 'obgc_hw_config_s::obgc_motor_hw_config_s::OBGC_MOTOR_DRV_ONBOARD0',
            'config.hw.motor.1.type': 'obgc_hw_config_s::obgc_motor_hw_config_s::OBGC_MOTOR_DRV_ONBOARD1',
            'config.hw.motor.2.type': 'obgc_hw_config_s::obgc_motor_hw_config_s::OBGC_MOTOR_DRV_ONBOARD2',
            'config.vbat.scale': 32030,
            'config.vbat.lvco': 9900,
        },
    }

    def __init__(self, connection, parent=None):
        super().__init__(parent)
        self.connection = connection
        self.unsaved_params = set()
        self.param_values = {}
        self.param_controls = {}
        self.row_params = {}
        self.row_labels = {}
        self.loading = False

        layout = QVBoxLayout()

        header_layout = QHBoxLayout()
        info_label = QLabel(
            "Configure peripheral models and how they're connected to the STM32 MCU.  "
            "Some of these are part of the controller board and some or all may be "
            "connected to I²C or NT ports.  Changes, including the presets, are not "
            "sent to the gimbal firmware until you use the Send button, and have to "
            "be separately written to persistent memory in the Connection tab.  They "
            "only take effect after gimbal reboot.\n\n"
            "It's generally safe to try most configurations, even if incorrect, and "
            "watch the boot console log at the bottom for success/error messages and "
            "iterate.  For onboard (PWM) motor controllers there's no feedback from "
            "the driver so an \"Ok\" message doesn't rule out bad configuration.\n\n"
            "The motor drivers here do not need to be in a specific order (like "
            "outer-middle-inner), the motor to joint corresponence is autodetected "
            "later during calibration.  But the encoder numbers need to match the "
            "motor driver numbers.\n\n"
            "Complete this setup before starting calibration.\n\n"
        )
        info_label.setWordWrap(True)
        header_layout.addWidget(info_label, 1)

        preset_layout = QVBoxLayout()
        self.preset_combo = QComboBox()
        self.preset_combo.addItems(["No preset"] + list(self.presets.keys()))
        self.preset_combo.currentIndexChanged.connect(self.on_preset_changed)
        self.load_preset_btn = QPushButton("Load Preset")
        self.load_preset_btn.setEnabled(False)
        self.load_preset_btn.clicked.connect(self.on_load_preset)
        preset_layout.addWidget(self.preset_combo)
        preset_layout.addWidget(self.load_preset_btn)
        header_layout.addLayout(preset_layout)
        layout.addLayout(header_layout)

        layout.addStretch(1)

        grid = QGridLayout()
        grid.setColumnStretch(1, 1)

        self.imu_type_options = [
            ("None", 0),
            ("(I²C) Invensense MPU-6050", 1),
            ("(I²C) Invensense MPU-9250", 2),
            ("NT IMU module", 3),
        ]
        self.motor_type_options = [
            ("None", 0),
            ("Onboard 0", 1),
            ("Onboard 1", 2),
            ("Onboard 2", 3),
            ("Custom 3x PWM outputs", 4),
            ("SBGC I2C Drv", 5),
            ("NT Motor module", 6),
        ]
        self.encoder_type_options = [
            ("None", 0),
            ("(I²C) AS5600", 1),
            ("SBGC I2C Drv", 2),
            ("NT Motor module", 3),
        ]
        self.i2c_bus_options = [("Main", 0), ("Internal", 1)]
        self.sbgc32_id_options = [("1", 0), ("2", 1), ("3", 2), ("4", 3)]
        self.nt_imu_id_options = [("IMU1", 0), ("IMU2", 1), ("IMU3", 2)]
        self.nt_motor_id_options = [("Pitch", 0), ("Roll", 1), ("Yaw", 2)]
        self.sbgc32_enc_type_options = [
            ("AS5048A", 1),
            ("AS5048B", 2),
            ("AMT203", 3),
            ("MA3 10-bit", 4),
            ("MA3 12-bit", 5),
            ("Analog", 6),
            ("AS5600", 7),
            ("AS5050A", 8),
            ("AS5055A", 9),
        ]
        self.pin_options = []
        for port_index, port_name in enumerate(["A", "B", "C", "D"]):
            for pin in range(16):
                self.pin_options.append((f"P{port_name}_{pin}", (port_index << 4) + pin))

        row = 0
        row = self.add_imu_row(
            grid,
            row,
            "Main IMU",
            "config.hw.main-imu.type",
            "config.hw.main-imu.i2c.bus",
            "config.hw.main-imu.i2c.addr",
            "config.hw.main-imu.nt-id",
        )
        row = self.add_imu_row(
            grid,
            row,
            "Frame IMU (optional)",
            "config.hw.frame-imu.type",
            "config.hw.frame-imu.i2c.bus",
            "config.hw.frame-imu.i2c.addr",
            "config.hw.frame-imu.nt-id",
        )
        for i in range(0, 3):
            row = self.add_motor_row(
                grid,
                row,
                f"Motor driver {i}",
                f"config.hw.motor.{i}.type",
                f"config.hw.motor.{i}.sbgc32-id",
                f"config.hw.motor.{i}.nt-id",
                f"config.hw.motor.{i}.pins.in",
                f"config.hw.motor.{i}.pins.en",
            )
        for i in range(0, 3):
            row = self.add_encoder_row(
                grid,
                row,
                f"Encoder {i} (optional)",
                f"config.hw.encoder.{i}.type",
                f"config.hw.encoder.{i}.i2c.bus",
                f"config.hw.encoder.{i}.i2c.addr",
                f"config.hw.encoder.{i}.sbgc32-type",
            )

        layout.addLayout(grid)

        self.send_btn = QPushButton("Send changes")
        self.send_btn.setEnabled(False)
        self.send_btn.clicked.connect(self.on_send_to_gimbal)
        self.send_btn.setMinimumWidth(250);
        layout.addWidget(self.send_btn, 0, Qt.AlignmentFlag.AlignLeft)

        layout.addStretch(1)
        self.setLayout(layout)

        self.connection.connection_changed.connect(self.on_connection_changed)

        # TODO: only read the address/ID params valid for the currently selected .type
        self.all_params = [
            "config.hw.main-imu.type",
            "config.hw.main-imu.i2c.bus",
            "config.hw.main-imu.i2c.addr",
            "config.hw.main-imu.nt-id",
            "config.hw.frame-imu.type",
            "config.hw.frame-imu.i2c.bus",
            "config.hw.frame-imu.i2c.addr",
            "config.hw.frame-imu.nt-id",
            "config.hw.motor.0.type",
            "config.hw.motor.0.pins.in",
            "config.hw.motor.0.pins.en",
            "config.hw.motor.0.sbgc32-id",
            "config.hw.motor.0.nt-id",
            "config.hw.motor.1.type",
            "config.hw.motor.1.pins.in",
            "config.hw.motor.1.pins.en",
            "config.hw.motor.1.sbgc32-id",
            "config.hw.motor.1.nt-id",
            "config.hw.motor.2.type",
            "config.hw.motor.2.pins.in",
            "config.hw.motor.2.pins.en",
            "config.hw.motor.2.sbgc32-id",
            "config.hw.motor.2.nt-id",
            "config.hw.encoder.0.type",
            "config.hw.encoder.0.i2c.bus",
            "config.hw.encoder.0.i2c.addr",
            "config.hw.encoder.0.sbgc32-type",
            "config.hw.encoder.1.type",
            "config.hw.encoder.1.i2c.bus",
            "config.hw.encoder.1.i2c.addr",
            "config.hw.encoder.1.sbgc32-type",
            "config.hw.encoder.2.type",
            "config.hw.encoder.2.i2c.bus",
            "config.hw.encoder.2.i2c.addr",
            "config.hw.encoder.2.sbgc32-type",
            "config.vbat.scale",
            "config.vbat.lvco",
        ]

    def add_row_label(self, grid, row, text):
        label = QLabel(text)
        grid.addWidget(label, row, 0)
        self.row_labels[text] = label
        return label

    def add_combo(self, options, param_name, row_label):
        combo = QComboBox()
        for text, value in options:
            combo.addItem(text, value)
        combo.currentIndexChanged.connect(
            lambda index, name=param_name, widget=combo, row=row_label:
                self.on_param_changed(name, widget.currentData(), row)
        )
        self.param_controls[param_name] = {
            "widget": combo,
            "setter": lambda value, w=combo: self.set_combo_value(w, value),
        }
        self.row_params.setdefault(row_label, set()).add(param_name)
        return combo

    def add_spin(self, param_name, row_label, minimum=0, maximum=127, hex_display=False):
        spin = QSpinBox()
        spin.setRange(minimum, maximum)
        if hex_display:
            spin.setDisplayIntegerBase(16)
            spin.setPrefix("0x")
        spin.valueChanged.connect(
            lambda value, name=param_name, row=row_label: self.on_param_changed(name, value, row)
        )
        self.param_controls[param_name] = {
            "widget": spin,
            "setter": lambda value, w=spin: w.setValue(int(value)),
        }
        self.row_params.setdefault(row_label, set()).add(param_name)
        return spin

    def add_pin_combo(self, param_name, row_label, allow_none=False):
        combo = QComboBox()
        if allow_none:
            combo.addItem("None", 0)
        for text, value in self.pin_options:
            if allow_none and value == 0:
                continue
            combo.addItem(text, value)
        combo.currentIndexChanged.connect(
            lambda index, name=param_name, widget=combo, row=row_label:
                self.on_param_changed(name, widget.currentData(), row)
        )
        self.param_controls[param_name] = {
            "widget": combo,
            "setter": lambda value, w=combo: self.set_combo_value(w, value),
        }
        self.row_params.setdefault(row_label, set()).add(param_name)
        return combo

    def add_pwm_pins(self, param_name, row_label, count=3):
        combos = []
        for idx in range(count):
            combo = QComboBox()
            for text, value in self.pin_options:
                combo.addItem(text, value)
            combo.currentIndexChanged.connect(
                lambda index, name=param_name, i=idx, w=combo, row=row_label:
                    self.on_array_param_changed(name, i, w.currentData(), row)
            )
            combos.append(combo)

        def set_pins(values, widgets=combos):
            if not isinstance(values, list):
                values = [values] * len(widgets)
            for i, widget in enumerate(widgets):
                if i < len(values):
                    self.set_combo_value(widget, values[i])

        self.param_controls[param_name] = {
            "widget": combos,
            "setter": set_pins,
        }
        self.row_params.setdefault(row_label, set()).add(param_name)
        return combos

    def add_imu_row(self, grid, row, label_text, type_param, bus_param, addr_param, nt_param):
        row_label = label_text
        self.add_row_label(grid, row, label_text)
        container = QWidget()
        row_layout = QHBoxLayout()
        row_layout.setContentsMargins(0, 0, 0, 0)
        row_layout.setSpacing(6)

        type_combo = self.add_combo(self.imu_type_options, type_param, row_label)
        row_layout.addWidget(type_combo)

        bus_label = QLabel("Bus")
        bus_combo = self.add_combo(self.i2c_bus_options, bus_param, row_label)
        addr_label = QLabel("Addr")
        addr_input = self.add_spin(addr_param, row_label, 0, 127, True)
        nt_label = QLabel("ID")
        nt_combo = self.add_combo(self.nt_imu_id_options, nt_param, row_label)

        row_layout.addWidget(bus_label)
        row_layout.addWidget(bus_combo)
        row_layout.addWidget(addr_label)
        row_layout.addWidget(addr_input)
        row_layout.addWidget(nt_label)
        row_layout.addWidget(nt_combo)
        row_layout.addStretch()
        container.setLayout(row_layout)
        grid.addWidget(container, row, 1)

        self.param_controls[type_param]["on_change"] = lambda: self.update_imu_visibility(
            type_combo, bus_label, bus_combo, addr_label, addr_input, nt_label, nt_combo
        )
        self.update_imu_visibility(
            type_combo, bus_label, bus_combo, addr_label, addr_input, nt_label, nt_combo
        )
        return row + 1

    def add_motor_row(self, grid, row, label_text, type_param, sbgc_param, nt_param, pins_in_param, pin_en_param):
        row_label = label_text
        self.add_row_label(grid, row, label_text)
        container = QWidget()
        row_layout = QHBoxLayout()
        row_layout.setContentsMargins(0, 0, 0, 0)
        row_layout.setSpacing(6)

        type_combo = self.add_combo(self.motor_type_options, type_param, row_label)
        row_layout.addWidget(type_combo)

        sbgc_label = QLabel("ID")
        sbgc_combo = self.add_combo(self.sbgc32_id_options, sbgc_param, row_label)
        nt_label = QLabel("ID")
        nt_combo = self.add_combo(self.nt_motor_id_options, nt_param, row_label)

        pwm_label = QLabel("PWM pins")
        pwm_inputs = self.add_pwm_pins(pins_in_param, row_label, 3)
        en_label = QLabel("EN pin")
        en_combo = self.add_pin_combo(pin_en_param, row_label, allow_none=True)

        row_layout.addWidget(sbgc_label)
        row_layout.addWidget(sbgc_combo)
        row_layout.addWidget(nt_label)
        row_layout.addWidget(nt_combo)
        row_layout.addWidget(pwm_label)
        for combo in pwm_inputs:
            row_layout.addWidget(combo)
        row_layout.addWidget(en_label)
        row_layout.addWidget(en_combo)
        row_layout.addStretch()
        container.setLayout(row_layout)
        grid.addWidget(container, row, 1)

        self.param_controls[type_param]["on_change"] = lambda: self.update_motor_visibility(
            type_combo, sbgc_label, sbgc_combo, nt_label, nt_combo,
            pwm_label, pwm_inputs, en_label, en_combo
        )
        self.update_motor_visibility(
            type_combo, sbgc_label, sbgc_combo, nt_label, nt_combo,
            pwm_label, pwm_inputs, en_label, en_combo
        )
        return row + 1

    def add_encoder_row(self, grid, row, label_text, type_param, bus_param, addr_param, sbgc_param):
        row_label = label_text
        self.add_row_label(grid, row, label_text)
        container = QWidget()
        row_layout = QHBoxLayout()
        row_layout.setContentsMargins(0, 0, 0, 0)
        row_layout.setSpacing(6)

        type_combo = self.add_combo(self.encoder_type_options, type_param, row_label)
        row_layout.addWidget(type_combo)

        bus_label = QLabel("Bus")
        bus_combo = self.add_combo(self.i2c_bus_options, bus_param, row_label)
        sbgc_label = QLabel("Type")
        sbgc_combo = self.add_combo(self.sbgc32_enc_type_options, sbgc_param, row_label)

        row_layout.addWidget(bus_label)
        row_layout.addWidget(bus_combo)
        row_layout.addWidget(sbgc_label)
        row_layout.addWidget(sbgc_combo)
        row_layout.addStretch()
        container.setLayout(row_layout)
        grid.addWidget(container, row, 1)

        self.param_controls[type_param]["on_change"] = lambda: self.update_encoder_visibility(
            type_combo, bus_label, bus_combo, sbgc_label, sbgc_combo, addr_param, row_label
        )
        self.update_encoder_visibility(type_combo, bus_label, bus_combo, sbgc_label, sbgc_combo, addr_param, row_label)
        return row + 1

    def set_combo_value(self, combo, value):
        try:
            value = int(value)
        except Exception:
            pass
        for i in range(combo.count()):
            if combo.itemData(i) == value or combo.itemText(i) == value:
                combo.setCurrentIndex(i)
                return

    def update_imu_visibility(self, type_combo, bus_label, bus_combo, addr_label, addr_input, nt_label, nt_combo):
        imu_type = type_combo.currentData()
        is_i2c = imu_type in [1, 2]
        is_nt = imu_type == 3
        bus_label.setVisible(is_i2c)
        bus_combo.setVisible(is_i2c)
        addr_label.setVisible(is_i2c)
        addr_input.setVisible(is_i2c)
        nt_label.setVisible(is_nt)
        nt_combo.setVisible(is_nt)

    def update_motor_visibility(
        self,
        type_combo,
        sbgc_label,
        sbgc_combo,
        nt_label,
        nt_combo,
        pwm_label,
        pwm_inputs,
        en_label,
        en_input,
    ):
        motor_type = type_combo.currentData()
        is_sbgc = motor_type == 5
        is_nt = motor_type == 6
        is_pwm = motor_type == 4
        sbgc_label.setVisible(is_sbgc)
        sbgc_combo.setVisible(is_sbgc)
        nt_label.setVisible(is_nt)
        nt_combo.setVisible(is_nt)
        pwm_label.setVisible(is_pwm)
        for combo in pwm_inputs:
            combo.setVisible(is_pwm)
        en_label.setVisible(is_pwm)
        en_input.setVisible(is_pwm)

    def update_encoder_visibility(self, type_combo, bus_label, bus_combo, sbgc_label, sbgc_combo, addr_param, row_label):
        enc_type = type_combo.currentData()
        is_i2c = enc_type == 1
        is_sbgc = enc_type == 2
        bus_label.setVisible(is_i2c)
        bus_combo.setVisible(is_i2c)
        sbgc_label.setVisible(is_sbgc)
        sbgc_combo.setVisible(is_sbgc)
        if is_i2c and not self.loading:
            self.param_values[addr_param] = 0x36
            self.unsaved_params.add(addr_param)
            self.update_unsaved_ui()

    def on_param_changed(self, param_name, value, row_label):
        if self.loading:
            return
        self.param_values[param_name] = value
        self.unsaved_params.add(param_name)
        ctrl = self.param_controls.get(param_name)
        if ctrl and ctrl.get("on_change"):
            ctrl["on_change"]()
        self.update_unsaved_ui()

    def on_array_param_changed(self, param_name, index, value, row_label):
        if self.loading:
            return
        current = self.param_values.get(param_name)
        if not isinstance(current, list):
            current = [0, 0, 0]
        while len(current) <= index:
            current.append(0)
        current[index] = int(value)
        self.param_values[param_name] = current
        self.unsaved_params.add(param_name)
        ctrl = self.param_controls.get(param_name)
        if ctrl and ctrl.get("on_change"):
            ctrl["on_change"]()
        self.update_unsaved_ui()

    def update_unsaved_ui(self):
        for row_label, label in self.row_labels.items():
            is_dirty = any(param in self.unsaved_params for param in self.row_params.get(row_label, []))
            font = label.font()
            font.setBold(is_dirty)
            label.setFont(font)
        self.send_btn.setEnabled(self.connection.is_connected() and len(self.unsaved_params) > 0)

    def on_preset_changed(self, index):
        self.load_preset_btn.setEnabled(self.preset_combo.currentIndex() > 0)

    def on_load_preset(self):
        if self.preset_combo.currentText() not in self.presets:
            return

        values = self.presets[self.preset_combo.currentText()]
        by_name = {pdef.name: pdef for pdef in param_defs.params.values()}

        self.loading = True
        for param_name, value in values.items():
            pdef = by_name[param_name]
            param_type_cls = param_utils.ctype_to_construct(pdef.typ, pdef.size)
            self.param_values[param_name] = param_type_cls(value)
            ctrl = self.param_controls.get(param_name)
            if ctrl:
                ctrl["setter"](value)
        self.loading = False
        for ctrl in self.param_controls.values():
            if ctrl.get("on_change"):
                ctrl["on_change"]()
        self.unsaved_params.update(values.keys())
        self.update_unsaved_ui()

    def on_send_to_gimbal(self):
        if not self.connection.is_connected():
            return
        error = self.validate_devices()
        if error:
            logger.error(error)
            return
        for param_name in sorted(self.unsaved_params):
            value = self.param_values.get(param_name)
            if value is not None:
                self.connection.write_param(param_name, value)
        self.unsaved_params.clear()
        self.update_unsaved_ui()

    def validate_devices(self):
        imu_entries = [
            ("Main IMU", "config.hw.main-imu.type", "config.hw.main-imu.i2c.bus", "config.hw.main-imu.i2c.addr", "config.hw.main-imu.nt-id"),
            ("Frame IMU", "config.hw.frame-imu.type", "config.hw.frame-imu.i2c.bus", "config.hw.frame-imu.i2c.addr", "config.hw.frame-imu.nt-id"),
        ]
        motor_entries = [
            ("Motor 0", "config.hw.motor.0.type", "config.hw.motor.0.sbgc32-id", "config.hw.motor.0.nt-id"),
            ("Motor 1", "config.hw.motor.1.type", "config.hw.motor.1.sbgc32-id", "config.hw.motor.1.nt-id"),
            ("Motor 2", "config.hw.motor.2.type", "config.hw.motor.2.sbgc32-id", "config.hw.motor.2.nt-id"),
        ]
        enc_entries = [
            ("Encoder 0", "config.hw.encoder.0.type", "config.hw.encoder.0.i2c.bus", "config.hw.encoder.0.i2c.addr", "config.hw.encoder.0.sbgc32-type"),
            ("Encoder 1", "config.hw.encoder.1.type", "config.hw.encoder.1.i2c.bus", "config.hw.encoder.1.i2c.addr", "config.hw.encoder.1.sbgc32-type"),
            ("Encoder 2", "config.hw.encoder.2.type", "config.hw.encoder.2.i2c.bus", "config.hw.encoder.2.i2c.addr", "config.hw.encoder.2.sbgc32-type"),
        ]

        def get_value(name, default=0):
            return int(self.param_values.get(name, default))

        seen_imus = {}
        for label, type_param, bus_param, addr_param, nt_param in imu_entries:
            imu_type = get_value(type_param)
            if imu_type == 0:
                continue
            if imu_type in [1, 2]:
                key = ("i2c", get_value(bus_param), get_value(addr_param))
            else:
                key = ("nt", get_value(nt_param))
            if key in seen_imus:
                return f"{label} conflicts with {seen_imus[key]}."
            seen_imus[key] = label

        seen_motors = {}
        for label, type_param, sbgc_param, nt_param in motor_entries:
            motor_type = get_value(type_param)
            if motor_type == 0:
                continue
            if motor_type == 5:
                key = ("sbgc32", get_value(sbgc_param))
            elif motor_type == 6:
                key = ("nt", get_value(nt_param))
            else:
                key = ("type", motor_type)
            if key in seen_motors:
                return f"{label} conflicts with {seen_motors[key]}."
            seen_motors[key] = label

        seen_encoders = {}
        for idx, (label, type_param, bus_param, addr_param, sbgc_param) in enumerate(enc_entries):
            enc_type = get_value(type_param)
            if enc_type == 0:
                continue
            if enc_type == 1:
                key = ("i2c", get_value(bus_param), 0x36)
            elif enc_type == 2:
                motor_type = get_value(motor_entries[idx][1])
                if motor_type == 5:
                    key = ("sbgc32", get_value(motor_entries[idx][2]))
                else:
                    key = ("sbgc32", idx)
            else:
                motor_type = get_value(motor_entries[idx][1])
                if motor_type == 6:
                    key = ("nt", get_value(motor_entries[idx][2]))
                else:
                    key = ("nt", idx)
            if key in seen_encoders:
                return f"{label} conflicts with {seen_encoders[key]}."
            seen_encoders[key] = label

        i2c_seen = {}
        for label, type_param, bus_param, addr_param, _ in imu_entries:
            imu_type = get_value(type_param)
            if imu_type in [1, 2]:
                key = (get_value(bus_param), get_value(addr_param))
                if key in i2c_seen:
                    return f"{label} I2C address conflicts with {i2c_seen[key]}."
                i2c_seen[key] = label
        for idx, (label, type_param, sbgc_param, nt_param) in enumerate(motor_entries):
            motor_type = get_value(type_param)
            if motor_type == 5:
                addr = 0x19 + get_value(sbgc_param)
                key = (0, addr)
                if key in i2c_seen and i2c_seen[key] != f"Encoder {idx}":
                    return f"{label} I2C address conflicts with {i2c_seen[key]}."
                i2c_seen[key] = label
        for idx, (label, type_param, bus_param, addr_param, sbgc_param) in enumerate(enc_entries):
            enc_type = get_value(type_param)
            if enc_type == 1:
                key = (get_value(bus_param), 0x36)
                if key in i2c_seen:
                    return f"{label} I2C address conflicts with {i2c_seen[key]}."
                i2c_seen[key] = label
            elif enc_type == 2:
                motor_type = get_value(motor_entries[idx][1])
                if motor_type == 5:
                    addr = 0x19 + get_value(motor_entries[idx][2])
                    key = (0, addr)
                    if key in i2c_seen and i2c_seen[key] not in [label, motor_entries[idx][0]]:
                        return f"{label} I2C address conflicts with {i2c_seen[key]}."
                    i2c_seen[key] = label
            elif enc_type == 3:
                motor_type = get_value(motor_entries[idx][1])
                if motor_type != 6:
                    return f"{label} uses NT Drv but motor {idx} is not NT."

        return None

    def on_connection_changed(self, connected):
        self.send_btn.setEnabled(connected and len(self.unsaved_params) > 0)
        if connected:
            self.refresh_from_gimbal()

    def refresh_from_gimbal(self):
        if not self.connection.is_connected():
            return
        self.loading = True
        self.connection.read_param(self.all_params, self.on_params_loaded)

    def on_params_loaded(self, values):
        self.loading = False
        if values is None:
            return
        if not isinstance(values, list):
            values = [values]
        if len(values) != len(self.all_params):
            return
        self.loading = True
        for param_name, value in zip(self.all_params, values):
            if isinstance(value, list):
                self.param_values[param_name] = value
            else:
                try:
                    self.param_values[param_name] = int(value)
                except Exception:
                    self.param_values[param_name] = value
            ctrl = self.param_controls.get(param_name)
            if ctrl:
                ctrl["setter"](value)
        self.loading = False
        for ctrl in self.param_controls.values():
            if ctrl.get("on_change"):
                ctrl["on_change"]()
        self.unsaved_params.clear()
        self.update_unsaved_ui()

    def start_updates(self):
        self.refresh_from_gimbal()

    def stop_updates(self):
        pass
