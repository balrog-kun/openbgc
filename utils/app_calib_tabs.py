#! /usr/bin/python3
# vim: set ts=4 sw=4 sts=4 et :
import math
import logging

try:
    from PyQt6.QtWidgets import (
        QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QGroupBox,
        QGridLayout, QCheckBox, QDoubleSpinBox, QSpinBox, QComboBox,
        # Qt 6 only
        QAbstractSpinBox
    )
    from PyQt6.QtGui import QIcon
    from PyQt6.QtCore import Qt, QTimer
    PYQT_VERSION = 6
except ImportError:
    from PyQt5.QtWidgets import (
        QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QGroupBox,
        QGridLayout, QCheckBox, QDoubleSpinBox, QSpinBox, QComboBox,
    )
    from PyQt5.QtGui import QIcon
    from PyQt5.QtCore import Qt, QTimer
    PYQT_VERSION = 5

import sbgcserialapi.cmd as cmd

import app_widgets

# Set up logger
logger = logging.getLogger(__name__)


class CalibrationTab(QWidget):
    """Tab for calibration status and controls."""

    def __init__(self, connection, geometry, main_window=None):
        super().__init__(None)
        self.connection = connection
        self.geometry = geometry
        self.main_window = main_window

        layout = QVBoxLayout()

        # Calibration status label
        self.status_label = QLabel("There are 4 calibration steps to be completed before " +
            "the gimbal can correctly interpret the joint and camera orientations from " +
            "the available sensors.  There are 2 more steps after that to enable the " +
            "motors to be driven and stabilize the camera head.\n\n" +
            "The list below shows the recommended sequence of calibration steps.  Some " +
            "steps can only be done in a specific order, some may need to be redone if " +
            "an earlier step is re-done.\n\n" +
            "Hover over buttons to see specific info on each step.\n\n" +
            "Each procedure saves the output data to the gimbal's RAM only.  Remember to " +
            "test and save the new config to preserve changes after power off.")
        self.status_label.setWordWrap(True)
        layout.addWidget(self.status_label)

        # Calibration controls
        controls_group = QGroupBox("Calibration Steps")
        controls_layout = QGridLayout()
        controls_layout.setHorizontalSpacing(40)

        # TODO: keep as one line, let the toolkit do the line wrapping
        descriptions = {
            "balance": ("Manually balance the camera/load on the camera head so that it\n" +
                "stays static in any orientation without holding.  Recommended first step."),
            "gyro": ("Takes 5-10 seconds, can be re-done at any time.  Keep the gimbal\n" +
                "absolutely static while it runs.  This saves the reference gyroscope\n" +
                "readings (bias) which vary from device to device and with time and\n" +
                "temperature.\n" +
                "Recommmended before running axis calibration to reduce drift.\n" +
                "Also done automatically on boot, unless disabled."),
            "axes": ("Autodetects your gimbal's basic arms geometry, joint order, IMU\n" +
                "and mounting angles, encoder magnet angle and polarity.\n"
                "Requires you to move the gimbal in a specific sequence manually,\n" +
                "See instructions in the tab."),
            "home": ("Tells the firmware what the reference (level) camera head orientation\n" +
                "is.  Manually level or orient the camera head, looking in the forward\n" +
                "direction with regards to the handle/base, then press \"Set\" to remember\n" +
                "this position as home position."),
            "forward": ("Do this immediately after setting the home position.  It'll tell the\n" +
                "firmware where the forward direction is, i.e. where the camera lens is\n" +
                "pointing.  Even though it already knows the home orientation, without\n" +
                "this extra step it can't know where the camera's roll and pitch axes\n" +
                "are.  Note that the camera can be mounted diagonally or at any angle\n" +
                "relative to the *joint* axes, those are conceptually separate from\n" +
                "the camera's Euler axes that we define here.\n" +
                "Rotate the camera head from the home position strictly in the roll\n" +
                "axis to the right (CW) by around 45 degrees then hit \"Set\".  The\n" +
                "firmware will remember the axis of rotation between these two\n" +
                "orientations the forward vector.  Rolling here means that the\n" +
                "lens keeps pointing at the same point, only the the image in camera\n" +
                "rotates around its center.  You can have it on and zoomed in to\n" +
                "make a precise roll rotation."),
            "motor": ("Detects the number of each motor's rotor poles, their angle with\n" +
                "relation to the encoder (aka. field offset) and the direction of the\n" +
                "stator winding sequence wrt. current polarity.  Needed to drive the\n" +
                "motors."),
            "pid": ("Configure each motor's driver feedback loop for the load.  This\n" +
                "will need to be adjusted every time the camera is changed to ensure\n" +
                "the firmware applies the right amounts of torque for smooth,\n" +
                "controlled motion.  This is a manual step, see instructions in the\n" +
                "tab."),
            "limit": ("Autodetect or manually set the positions of the hard stops on\n" +
                "the joints, if present, to allow the firmware to avoid hitting them\n" +
                "and plan motion trajectories that don't cross the limit angles."),
            "parking": ("Save the current joint angles as the parking position.  Optional."),
            "voltage": ("Set the battery/supply voltage sensing scale for correct\n" +
                "low voltage alarm and motor force compensation.  Optional."),
        }

        motor_geom_params = [f'config.motor-calib.{i}.bldc-with-encoder.pole-pairs' for i in range(3)]
        motor_pid_params = [f'config.motor-pid.{i}.kp' for i in range(3)]
        vbat_params = ['config.vbat.scale', 'config.vbat.lvco']

        # Define calibration steps with their corresponding geometry flags
        self.calibration_steps = [
            ('balance', "Camera balance",
             [],
             [],
             lambda: True),
            ('gyro', "Gyro calibration",
             [("Redo", self.on_gyro_redo, 'view-refresh')],
             [],
             lambda: True),
            ('axes', "Axes & encoders geometry",
             [("Go to tab", self.on_go_to_axes_tab, None)],
             [],
             lambda: self.geometry.have_axes),
            ('home', "Home position",
             [("Set", self.on_set_home, 'go-next')],
             [],
             lambda: self.geometry.have_home),
            ('forward', "Forward direction",
             [("Set", self.on_set_forward, 'go-next')],
             [],
             lambda: self.geometry.have_forward),
            ('motor', "Motor geometry",
             [("Go to tab", self.on_go_to_motor_tab, None)],
             motor_geom_params,
             lambda: all([self.params.get(p) for p in motor_geom_params])),
            ('pid', "Motor PIDs",
             [("Go to tab", self.on_go_to_pid_tab, None)],
             motor_geom_params,
             # No way to know whether configured?  Maybe check if the Kp's are different
             # from the default value of 0.03.  Although nothing is wrong with that value,
             # it'll probably have been touched and the probability to end up with the same
             # 32-bit float value again after tuning is tiny.
             lambda: all([self.params.get(p) for p in motor_geom_params])),
            ('limit', "Joint limits",
             [("Go to tab", self.on_go_to_limit_tab, None)],
             [],
             lambda: any(self.geometry.has_limits)),
            ('parking', "Parking position",
             [("Set", self.on_set_parking, 'go-next')],
             [],
             lambda: self.geometry.have_parking),
            ('voltage', "Battery voltage sensing",
             [("Go to tab", self.on_go_to_vbat_tab, None)],
             vbat_params,
             lambda: all([self.params.get(p) != 0 for p in vbat_params])),
        ]

        self.params = {}

        self.calibration_checkboxes = {}
        self.calibration_labels = {}
        self.calibration_btns = {}

        row = 0
        for step_id, step_name, buttons, _, _ in self.calibration_steps:
            # Checkbox in first column
            checkbox = QCheckBox()
            checkbox.setEnabled(False) # Read-only
            checkbox.setToolTip(descriptions[step_id])
            checkbox.setStyleSheet('background-color: #ccc;')
            # This would have been better but it overrides the native rendering completely
            #checkbox.setStyleSheet('QCheckBox::indicator { background-color: #ccc; }')
            checkbox.setMaximumSize(checkbox.sizeHint())

            controls_layout.addWidget(checkbox, row, 0)
            self.calibration_checkboxes[step_id] = checkbox

            # Label in second column
            label = QLabel(step_name)
            label.setToolTip(descriptions[step_id])
            controls_layout.addWidget(label, row, 1)
            self.calibration_labels[step_id] = label

            # Button in third column
            column = 2
            self.calibration_btns[step_id] = []
            for button_text, handler, icon_name in buttons:
                button = QPushButton(button_text)
                button.setToolTip(descriptions[step_id])
                if icon_name is not None:
                    try:
                        button.setIcon(QIcon.fromTheme(icon_name))
                    except:
                        pass
                button.clicked.connect(handler)
                controls_layout.addWidget(button, row, column)
                column += 1
                self.calibration_btns[step_id].append(button)
                # TODO: disable all while motors on, calibration running, not connected, etc.

            row += 1

        controls_group.setLayout(controls_layout)
        layout.addWidget(controls_group)

        layout.addStretch()
        self.setLayout(layout)

        # Connect to geometry changes to update checkboxes and buttons
        self.geometry.geometry_changed.connect(self.update_checkboxes)
        self.connection.calibrating_changed.connect(self.update_buttons)

    def update_buttons(self):
        """Enable or disable all calibration buttons."""
        for step_id in self.calibration_btns:
            enabled = not self.connection.calibrating
            if step_id == 'limit':
                enabled = enabled and self.geometry.have_axes

            for button in self.calibration_btns[step_id]:
                button.setEnabled(enabled)

    def update_checkboxes(self):
        for step_id, _, _, _, check in self.calibration_steps:
            checked = check()
            checkbox = self.calibration_checkboxes[step_id]
            checkbox.setChecked(checked)
            checkbox.setToolTip("Already calibrated once" if checked else "Not done")

    def start_updates(self):
        """Update calibration status display."""
        if not self.connection.is_connected():
            return

        # Update checkboxes based on current geometry state
        self.update_checkboxes()
        self.update_buttons()

        req_params = {}
        for _, _, _, params, _ in self.calibration_steps:
            for param in params:
                req_params[param] = None
        keys = list(req_params.keys())

        def read_cb(values):
            if values is None:
                return

            # Update checkboxes again now with the rest of the params
            self.params.update(zip(keys, values))
            self.update_checkboxes()

        self.connection.read_param(keys, read_cb)

    def stop_updates(self):
        self.update_buttons()

    def on_gyro_redo(self):
        """Handle gyro calibration redo button click."""
        if not self.connection.is_connected():
            return

        def done(now):
            self.connection.set_calibrating(False)

        # Drain queues, then send calibration command
        def after_drain():
            self.connection.send_raw(b'c')
            self.connection.read_param("now", done)

        self.connection.set_calibrating(True)
        self.connection.drain_queues(after_drain)

    def on_go_to_axes_tab(self):
        """Switch to axis calibration tab."""
        self.main_window.switch_to_tab('calib-axes')

    def on_set_home(self):
        self.connection.send_raw(b'k')
        logger.info(f"New home position set")
        self.geometry.update()

    def on_set_forward(self):
        self.connection.send_raw(b'K')
        logger.info(f"New forward vector set")
        self.geometry.update()

    def on_go_to_motor_tab(self):
        self.main_window.switch_to_tab('calib-motor')

    def on_go_to_pid_tab(self):
        self.main_window.switch_to_tab('calib-pid')

    def on_go_to_limit_tab(self):
        self.main_window.switch_to_tab('calib-limit')

    def on_set_parking(self):
        def encoders_read_cb(values):
            if values is None:
                logger.error(f"Can't set parking angles, read error")
                return

            self.connection.write_param('config.control.park-angles',
                                        [math.radians(values[i]) for i in range(3)])
            self.connection.write_param('config.control.have-parking', True)
            self.geometry.update()
            logger.info(f"New parking angles set")

        self.connection.read_param([f"encoders.{i}.reading" for i in range(3)], encoders_read_cb)

    def on_go_to_vbat_tab(self):
        self.main_window.switch_to_tab('calib-vbat')


class AxisCalibrationTab(QWidget):
    """Tab for axes and encoders geometry calibration."""

    def __init__(self, connection, geometry, parent=None):
        super().__init__(parent)
        self.connection = connection
        self.geometry = geometry
        self.calibration_active = False
        self.done_count = 0

        layout = QVBoxLayout()

        # Top HBox with label and buttons
        top_hbox = QHBoxLayout()

        # Placeholder label on the left
        info_label = QLabel("This procedure autodetects your gimbal's basic arms geometry, " +
                "joint order, IMU mounting angles and encoder magnet angle and polarity.  " +
                "The procedure requires you to move the gimbal arms in a specific sequence " +
                " manually whie the firmware is going to be matching the IMU readings with the " +
                " encoder readings and bulding an map of the geometry parameters.  Follow the " +
                "instructions from the firmware below.\n\n" +
                "After succesful calibration make sure to write the new configuration to non-" +
                "volatile storage.  Follow with the home position calibration step.\n\n" +
                "If you cancel, some parameters may have been overwritten so a re-read of " +
                "non-volatile storage is recommended (or a reset).")
        top_hbox.addWidget(info_label)
        info_label.setWordWrap(True)

        top_hbox.addStretch()

        right_vbox = QVBoxLayout()

        # Start and Cancel buttons on the right
        self.start_btn = QPushButton("Start")
        try:
            self.start_btn.setIcon(QIcon.fromTheme("go-next"))
        except:
            pass
        self.start_btn.clicked.connect(self.on_start)
        self.cancel_btn = QPushButton("Cancel")
        self.cancel_btn.clicked.connect(self.on_cancel)
        self.flip_btns = [QPushButton("Flip axis " + str(i)) for i in range(3)]
        for i in range(3):
            self.flip_btns[i].clicked.connect(lambda checked, i=i: self.on_flip(i))
            self.flip_btns[i].setToolTip(
                "The calibration sequence cannot detect whether the joint is physically\n" +
                "on one side of the camera or the other (or both), only where the joint's\n" +
                "rotation axis is as an infinite straigth line.  And it doesn't matter for\n" +
                "any of the maths performed internally as long as the encoder scale sign\n" +
                "matches the axis vector's direction.  But the visualization in the\n" +
                "Status Tab uses the axis vector direction to show the joint on one side\n" +
                "or on the other (the vector points from joint towards camera) so invert\n" +
                "the vector if the joint appearing on the wrong side bothers you.")

            try:
                self.flip_btns[i].setIcon(QIcon.fromTheme("object-flip-horizontal"))
            except:
                pass

        # TODO: backup/restore buttons here or in the main calibration tab
        # together with the Write all to NV mem, reread all from NV mem

        right_vbox.addWidget(self.start_btn)
        right_vbox.addWidget(self.cancel_btn)
        for button in self.flip_btns:
            right_vbox.addWidget(button)
        top_hbox.addLayout(right_vbox)
        layout.addLayout(top_hbox)

        # Console widget below
        self.console = app_widgets.ConsoleWidget(self.connection)
        self.console.set_active(False)
        self.console.setToolTip('Calibration status text sent by the firmware')
        layout.addWidget(self.console)

        self.setLayout(layout)

        # Connect to text_logged signal to watch for calibration completion
        self.connection.text_logged.connect(self.on_text_logged)

        self.connection.calibrating_changed.connect(self.update_buttons)

        self.update_buttons()

    def update_buttons(self):
        self.start_btn.setEnabled(self.connection.is_connected() and not self.connection.calibrating)
        self.cancel_btn.setEnabled(self.connection.is_connected() and self.calibration_active)
        for button in self.flip_btns:
            button.setEnabled(self.connection.is_connected() and not self.connection.calibrating and
                              self.geometry.have_axes)

    def on_start(self):
        """Handle start button click."""
        if not self.connection.is_connected():
            return

        self.calibration_active = True
        self.connection.set_calibrating(True)
        self.update_buttons()
        self.done_count = 0

        def after_drain():
            self.console.set_active(True)
            self.connection.send_raw(b'C')

        self.connection.drain_queues(after_drain)

    def done(self):
        self.calibration_active = False
        self.connection.set_calibrating(False)

        self.update_buttons()
        self.geometry.update()

    def on_cancel(self):
        """Handle cancel button click."""
        if not self.connection.is_connected() or not self.calibration_active:
            return

        # Send "q" command to cancel calibration
        self.connection.send_raw(b'q')

        self.done()

    def on_flip(self, axis_num):
        enc_num = self.geometry.axis_to_encoder[axis_num]

        logger.info(f'Inverting axis {axis_num} vector and related settings')

        self.connection.write_param(f'config.axes.axes.{axis_num}', [-v for v in self.geometry.axes[axis_num]])
        self.connection.write_param(f'config.axes.encoder-scale.{enc_num}', -self.geometry.encoder_scale[enc_num])
        self.connection.write_param(f'config.axes.limit-min.{axis_num}', -self.geometry.limit_max[axis_num])
        self.connection.write_param(f'config.axes.limit-max.{axis_num}', -self.geometry.limit_min[axis_num])
        self.geometry.update()

    def on_text_logged(self, text):
        """Handle text logged signal - watch for calibration completion."""
        if not self.calibration_active:
            return

        lines = text.split('\n')
        for line in lines:
            # Ignore content in square brackets at the beginning
            bracket_end = line.find(']')
            while bracket_end >= 0:
                line = line[bracket_end + 1:].lstrip()
                bracket_end = line.find(']')

            # Check for "Cancelled" or "Done."
            if "Cancelled" in line:
                self.console.set_active(False)
                self.on_cancel()
                return
            elif line.startswith("Done."):
                self.done_count += 1
                if self.done_count >= 3:
                    self.console.set_active(False)
                    self.done()

    def start_updates(self):
        self.update_buttons()
    def stop_updates(self):
        self.update_buttons()


class MotorGeometryCalibrationTab(QWidget):
    """Tab for motor geometry calibration."""

    def __init__(self, connection, parent=None):
        super().__init__(parent)
        self.connection = connection

        layout = QVBoxLayout()

        # Explanation label at the top
        explanation_label = QLabel(
            "There are 3 motor geometry parameters per motor and they're needed before "
            "the firwmare can make any use of the motors.  Luckily the firmware can guess "
            "their values by commanding specific electrical angles to the motors and "
            "watching the encoder feedback.\n\n"
            "The logic is quite strict so it will report "
            "failure if the feedback is even slightly different from expected.  Retry the "
            "auto-detection until it succeeds varying the starting angle of the joint by "
            "moving it manually.\n\n"
            "Remember to write new configuration to non-volatile storage to preserve "
            "after power off."
        )
        explanation_label.setWordWrap(True)
        layout.addWidget(explanation_label)

        # Grid layout for motor parameters
        grid_group = QGroupBox("Parameters")
        grid_layout = QGridLayout()

        # Headers
        grid_layout.addWidget(QLabel("Pole Pairs"), 0, 2)
        grid_layout.addWidget(QLabel("Zero Offset"), 0, 3)
        grid_layout.addWidget(QLabel("Direction"), 0, 4)

        self.autodetect_btns = []
        self.pole_pairs_inputs = []
        self.zero_offset_inputs = []
        self.sensor_direction_combos = []
        self.override_btns = []

        # Create rows for each joint
        # TODO: remap if have_axes?
        #joints = ["Outer", "Middle", "Inner"]
        joints = ["First", "Second", "Third"]
        for joint_num in range(3):
            row = joint_num + 1

            # Joint label
            joint_label = QLabel(f"{joints[joint_num]} joint motor ({joint_num})")
            grid_layout.addWidget(joint_label, row, 0)

            # Autodetect button
            autodetect_btn = QPushButton("Autodetect")
            autodetect_btn.clicked.connect(lambda checked, n=joint_num: self.on_autodetect(n))
            autodetect_btn.setToolTip(
                f"Automatically detect motor geometry parameters for joint {joint_num}.\n"
                "This will temporarily disable other motors, run the calibration, and\n"
                "restore all motor settings.  Takes 5-10 seconds.  Watch the console for\n"
                "results."
            )
            try:
                autodetect_btn.setIcon(QIcon.fromTheme('go-next'))
            except:
                pass
            grid_layout.addWidget(autodetect_btn, row, 1)
            self.autodetect_btns.append(autodetect_btn)

            # Pole pairs spinbox
            pole_pairs_input = QSpinBox()
            pole_pairs_input.setMinimum(2)
            pole_pairs_input.setMaximum(200)
            pole_pairs_input.setMaximumWidth(70)
            pole_pairs_input.setToolTip(
                f"Number of pole pairs for joint {joint_num} motor.\n"
                "This is half the number of magnets on the rotor."
            )
            grid_layout.addWidget(pole_pairs_input, row, 2)
            self.pole_pairs_inputs.append(pole_pairs_input)

            # Zero electric offset spinbox (with adaptive step)
            zero_offset_input = QDoubleSpinBox()
            zero_offset_input.setDecimals(1)
            zero_offset_input.setRange(0, 360)
            zero_offset_input.setMaximumWidth(80)
            # Set adaptive step type
            if PYQT_VERSION == 6:
                zero_offset_input.setStepType(QAbstractSpinBox.StepType.AdaptiveDecimalStepType)
            else:
                # PyQt5 doesn't have StepType enum, use setSingleStep with a small value
                zero_offset_input.setSingleStep(1)
            zero_offset_input.setToolTip(
                f"Zero electric offset for joint {joint_num} motor.\n"
                "This is the electrical angle offset between the encoder zero position\n"
                "and the motor's nearest electrical zero position."
            )
            grid_layout.addWidget(zero_offset_input, row, 3)
            self.zero_offset_inputs.append(zero_offset_input)

            # Sensor direction combo
            sensor_direction_combo = QComboBox()
            sensor_direction_combo.addItems(["1", "-1"])
            sensor_direction_combo.setMaximumWidth(60)
            sensor_direction_combo.setToolTip(
                f"Sensor direction for joint {joint_num} motor.\n"
                "1 for electrical angle directly proportional to encoder angle, -1 for\n"
                "inversely proportional."
            )
            grid_layout.addWidget(sensor_direction_combo, row, 4)
            self.sensor_direction_combos.append(sensor_direction_combo)

            # Override button
            override_btn = QPushButton("Override")
            override_btn.clicked.connect(lambda checked, n=joint_num: self.on_override(n))
            override_btn.setToolTip(
                f"Write the manually entered values for joint {joint_num} motor to the gimbal."
            )
            grid_layout.addWidget(override_btn, row, 5)
            self.override_btns.append(override_btn)

        grid_group.setLayout(grid_layout)
        layout.addWidget(grid_group)

        layout.addStretch()
        self.setLayout(layout)

        self.connection.calibrating_changed.connect(self.update_buttons)
        self.update_buttons()

    def update_buttons(self):
        """Update button states based on connection and calibration status."""
        enabled = self.connection.is_connected() and not self.connection.calibrating
        for btn in self.autodetect_btns + self.override_btns:
            btn.setEnabled(enabled)

    def load_values(self, cb=None):
        """Load current motor calibration parameter values."""
        if not self.connection.is_connected():
            return

        def read_cb(values):
            if values is not None:
                for joint_num in range(3):
                    self.pole_pairs_inputs[joint_num].setValue(int(values[0 + joint_num]))
                    self.zero_offset_inputs[joint_num].setValue(float(values[3 + joint_num]))
                    idx = 0 if int(values[6 + joint_num]) == 1 else 1
                    self.sensor_direction_combos[joint_num].setCurrentIndex(idx)

            if cb is not None:
                cb()

        self.connection.read_param(
            [f'config.motor-calib.{i}.bldc-with-encoder.pole-pairs' for i in range(3)] +
            [f'config.motor-calib.{i}.bldc-with-encoder.zero-electric-offset' for i in range(3)] +
            [f'config.motor-calib.{i}.bldc-with-encoder.sensor-direction' for i in range(3)],
            read_cb)

    def on_autodetect(self, joint_num):
        """Handle autodetect button click for a specific joint."""
        if not self.connection.is_connected():
            return

        # Store original use-motor values
        self.orig_use_motor = [None, None, None]

        self.connection.set_calibrating(True)

        def done_cb():
            for i in range(3):
                enabled = (i == joint_num)
                if enabled != self.orig_use_motor[i]:
                    self.connection.send_raw(f't{i}'.encode())

            self.connection.set_calibrating(False)

        def read_cb(values):
            if values is None:
                self.connection.set_calibrating(False)
                return

            for i in range(3):
                self.orig_use_motor[i] = bool(values[i])

                # Toggle use-motor.{i} if needed, could also self.connection.write_param(f'use-motor.{i}', want_enabled)
                want_enabled = (i == joint_num)
                if want_enabled != self.orig_use_motor[i]:
                    self.connection.send_raw(f't{i}'.encode())

            # Start calibration
            self.connection.send_raw(b'm')

            # Firmware processes no parameter reads until calibration done so queue
            # the re-read and when that returns we know calibration has finished
            self.load_values(done_cb)

        def after_drain():
            # Read all use-motor values
            self.connection.read_param([f'use-motor.{i}' for i in range(3)], read_cb)

        self.connection.drain_queues(after_drain)

    def on_override(self, joint_num):
        """Handle override button click for a specific joint."""
        # Get values from UI
        pole_pairs = self.pole_pairs_inputs[joint_num].value()
        zero_offset = self.zero_offset_inputs[joint_num].value()
        sensor_direction = 1 if self.sensor_direction_combos[joint_num].currentText() == "1" else -1

        # Write parameters
        self.connection.write_param(
            f'config.motor-calib.{joint_num}.bldc-with-encoder.pole-pairs',
            pole_pairs
        )
        self.connection.write_param(
            f'config.motor-calib.{joint_num}.bldc-with-encoder.zero-electric-offset',
            zero_offset
        )
        self.connection.write_param(
            f'config.motor-calib.{joint_num}.bldc-with-encoder.sensor-direction',
            sensor_direction
        )

    def start_updates(self):
        self.load_values()
        self.update_buttons()

    def stop_updates(self):
        """Stop updates."""
        self.update_buttons()


class MotorPidEditorTab(QWidget):
    """Tab for motor PID parameter editing."""

    def __init__(self, connection, motors, parent=None):
        super().__init__(parent)
        self.connection = connection
        self.motors = motors

        layout = QVBoxLayout()

        # Info label with instructions
        info_label = QLabel( # TODO: proper list formatting
            "These parameters control the low-level motor control loop.  The PID loop "
            "variable is joint velocity or angular rate and the output is the motor "
            "winding voltage phasor, mapping roughly to the torque.\n\n"
            "Defaults are unlikely to work well so tuning this is a necessary, manual "
            "step.  No auto-tuning at this time.  Recommended initial sequence:\n"
            "• Tune each motor separately, test using the individual constant-velocity\n"
            "  buttons here before enabling all 3 motors and the higher level control\n"
            "  loop with the On button.  More tuning may be needed after all motors\n"
            "  are on and interfere with each other.\n"
            "• Set P of 0.01, zero I, zero D\n"
            "• Increase P until oscillation starts, then reduce by 40%.\n"
            "  The oscillation is likely in the 10-300Hz range so more like buzzing.\n"
            "  The goal with P is for velocities to reach commanded setpoints quickly\n"
            "  (tracking) but without overshooting.\n"
            "• Increase I until oscillation starts, then reduce by 20%, too.\n"
            "  This will be a slower, visible oscillation.  The goal is for I to correct\n"
            "  for cogging, frictions, non-ideal camera balance but not for low P.\n"
            "• D is likely unneeded.  It can help increase I range but reduces P range.\n"
        )
        info_label.setWordWrap(True)
        layout.addWidget(info_label)

        # Grid layout for PID parameters
        grid_group = QGroupBox("PID Parameters")
        grid_layout = QGridLayout()

        # Headers
        # TODO: remap if have_axes? requires geometry
        #l = [QLabel("Outer (0)"), QLabel("Middle (1)"), QLabel("Inner (2)")]
        l = [QLabel("Motor 0"), QLabel("Motor 1"), QLabel("Motor 2")]
        for i in range(3):
            grid_layout.addWidget(l[i], 0, 1 + i)
            grid_layout.setAlignment(l[i], Qt.AlignmentFlag.AlignCenter)

        # Parameter rows
        params = [
            ("P Gain", "kp", (0.001, 10),
             "Standard PID Kp"),
            ("I Gain", "ki", (0, 10),
             "Standard PID Ki"),
            ("D Gain", "kd", (0, 10),
             "PID differential gain expressed in time units"),
            ("P trust", "kp-trust", (0, 0.99),
             "Replace this fraction of velocity from feedback sensors with the velocity\n"
             "requested in previous iteration.  When the Kp value has been tuned and\n"
             "tested and works well, the loop can rely less on the sensor and assume\n"
             "that whatever it commanded in the previous iteration has taken effect\n"
             "perfectly.  The benefit it less sensor noise in the loop input, the\n"
             "downside is slower reactions to unpredictable external torques, so this\n"
             "may be mainly useful in tripod mode."),
            ("I falloff", "ki-falloff", (0, 0.5),
             "In each iteration drop this fraction of the accumulated error integral,\n"
             "basically multiply the accumulated I by (1 - this) to limit its buildup."),
            ("V_max", "v-max", (0.001, 1),
             "Limit commanded winding voltage (roughly proportional to torque) to this\n"
             "fraction of the total battery  voltage.  It's unitless, not in volts,\n"
             "because it's a fraction of Vbat.\n"
             "Use e.g. for overheat safety and camera stress safety, when you know you\n"
             "don't need torques larger than some part of the maximum torque the\n"
             "motor+battery combination can impart."),
            ("Drag", "kdrag", (0, 1),
             "Assume and compensate for a drag torque proportional to this fraction of\n"
             "current velocity.  Units undefined for now."),
        ]
        self.suffixes = [p[1] for p in params]

        self.spinboxes = {}
        self.send_btns = {}
        self.test_btns = []

        row = 1
        for name, param_suffix, valrange, desc in params:
            # Parameter label
            namelabel = QLabel(name)
            namelabel.setToolTip(desc)
            grid_layout.addWidget(namelabel, row, 0)

            # Container widget for each motor
            for motor_num in range(3):
                # Top row: spinbox + send button
                param_layout = QHBoxLayout()
                param_layout.setAlignment(Qt.AlignmentFlag.AlignCenter | Qt.AlignmentFlag.AlignVCenter)

                spinbox = QDoubleSpinBox()
                spinbox.setDecimals(3) # TODO: subclass to drop trailing zeros the increase to 4
                spinbox.setRange(valrange[0], valrange[1])
                spinbox.setMaximumWidth(80)
                spinbox.setToolTip(
                    f"{name} value for motor {motor_num}\nRange ({valrange[0]}, {valrange[1]})\n{desc}")

                # Set adaptive step for PyQt6
                if PYQT_VERSION == 6:
                    spinbox.setStepType(QAbstractSpinBox.StepType.AdaptiveDecimalStepType)
                else:
                    spinbox.setSingleStep(0.01)

                # Connect Enter key to trigger button
                spinbox.lineEdit().returnPressed.connect(
                    lambda motor=motor_num, param=param_suffix: self.on_send_value(motor, param)
                )

                param_layout.addWidget(spinbox)

                # Small send button with icon
                send_btn = QPushButton()
                send_btn.setMaximumWidth(30)
                send_btn.setMaximumHeight(25)
                send_btn.setToolTip(f"Send {name} value for motor {motor_num}")
                send_btn.clicked.connect(
                    lambda checked, motor=motor_num, param=param_suffix: self.on_send_value(motor, param)
                )

                # Set send icon
                try:
                    send_btn.setIcon(QIcon.fromTheme('go-next'))
                    send_btn.setText("")  # Hide text when icon is available
                except:
                    send_btn.setText("→")  # Fallback to text

                param_layout.addWidget(send_btn)

                grid_layout.addLayout(param_layout, row, motor_num + 1)

                # Store references
                key = f"{param_suffix}_{motor_num}"
                self.spinboxes[key] = spinbox
                self.send_btns[key] = send_btn

            row += 1

        # Test setpoint row
        tooltip = (
            "Enable only one motor, power it on and set constant velocity setpoint\n"
            "without the position control loop.  Note that 0 sets a 0 velocity setpoint\n"
            "but the motor is on and using power, don't forget to press \"Off\" when\n"
            "done.")
        test_label = QLabel("Test setpoint (deg/s)")
        test_label.setToolTip(tooltip)
        grid_layout.addWidget(test_label, row, 0)

        for motor_num in range(3):
            # Create container for test buttons
            test_layout = QHBoxLayout()
            test_layout.setAlignment(Qt.AlignmentFlag.AlignCenter)

            # -5 button
            neg5_btn = QPushButton("-5")
            neg5_btn.setMaximumWidth(35)
            neg5_btn.clicked.connect(lambda checked, n=motor_num: self.on_test_setpoint(n, -5))
            neg5_btn.setToolTip(tooltip)
            test_layout.addWidget(neg5_btn)

            # 0 button
            zero_btn = QPushButton("0")
            zero_btn.setMaximumWidth(35)
            zero_btn.clicked.connect(lambda checked, n=motor_num: self.on_test_setpoint(n, 0))
            zero_btn.setToolTip(tooltip)
            test_layout.addWidget(zero_btn)

            # 5 button
            pos5_btn = QPushButton("5")
            pos5_btn.setMaximumWidth(35)
            pos5_btn.clicked.connect(lambda checked, n=motor_num: self.on_test_setpoint(n, 5))
            pos5_btn.setToolTip(tooltip)
            test_layout.addWidget(pos5_btn)

            grid_layout.addLayout(test_layout, row, motor_num + 1)
            self.test_btns += [neg5_btn, zero_btn, pos5_btn]

        row += 1

        tip_label = QLabel("Switch away and back to this tab to reload values from gimbal")
        font = tip_label.font()
        font.setPointSize(int(font.pointSize() * 0.8))
        tip_label.setFont(font)
        grid_layout.addWidget(tip_label, row, 0, row, -1, Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)

        grid_layout.setHorizontalSpacing(40) # grid_layout.setColumnStretch({1,2,3}, 2) doesn't seem to do much
        grid_group.setLayout(grid_layout)
        layout.addWidget(grid_group)

        # Motor status and controls (copied from Status tab)
        motor_group = QGroupBox("Motor Power")
        motor_layout = QHBoxLayout()
        motor_layout.addWidget(QLabel("Status:"))
        self.motor_status_label = QLabel("unknown")
        motor_layout.addWidget(self.motor_status_label)
        motor_layout.addStretch()

        self.motor_on_btn = QPushButton("On")
        self.motor_on_btn.clicked.connect(self.motors.on)
        self.motor_on_btn.setEnabled(False)  # Disabled until connected
        motor_layout.addWidget(self.motor_on_btn)

        self.motor_off_btn = QPushButton("Off")
        self.motor_off_btn.clicked.connect(self.on_motors_off)
        self.motor_off_btn.setEnabled(False)  # Disabled until connected
        motor_layout.addWidget(self.motor_off_btn)

        motor_group.setLayout(motor_layout)
        layout.addWidget(motor_group)

        layout.addStretch()
        self.setLayout(layout)

        # Connect to connection changes
        self.connection.calibrating_changed.connect(self.update_buttons)
        self.motors.on_changed.connect(self.new_motors_status)

        # Load initial values
        self.load_values()
        self.update_buttons()

    def load_values(self):
        """Load current motor PID parameter values."""
        if not self.connection.is_connected():
            return

        def read_cb(values):
            if values is not None:
                for i in range(3):  # motors
                    for j in range(len(self.suffixes)):  # parameters
                        param_idx = i * len(self.suffixes) + j
                        if param_idx < len(values):
                            spinbox_key = f"{self.suffixes[j]}_{i}"
                            self.spinboxes[spinbox_key].setValue(float(values[param_idx]))

        self.connection.read_param(
            [f'config.motor-pid.{i}.{p}' for i in range(3) for p in self.suffixes],
            read_cb)

    def on_send_value(self, motor_num, param_suffix):
        """Send the current spinbox value to the gimbal."""
        if not self.connection.is_connected():
            return

        key = f"{param_suffix}_{motor_num}"
        value = self.spinboxes[key].value()

        param_name = f'config.motor-pid.{motor_num}.{param_suffix}'
        self.connection.write_param(param_name, value)

    def on_test_setpoint(self, motor_num, setpoint):
        """Handle test setpoint button clicks."""
        if not self.connection.is_connected():
            return

        motor_ctrl_seq = [(b'[D', b'[C'), (b'[B', b'[A'), (b'[6~', b'[5~')]

        self.connection.send_raw(b's')

        # Enable only the specified motor
        self.set_single_motor_enabled(motor_num)

        # Send the setpoint command
        if setpoint == -5:
            self.connection.send_raw(b'S\x1b' + motor_ctrl_seq[motor_num][0])
        elif setpoint == 0:
            self.connection.send_raw(b'S')  # S only
        elif setpoint == 5:
            self.connection.send_raw(b'S\x1b' + motor_ctrl_seq[motor_num][1])

    def set_single_motor_enabled(self, motor_num):
        """Enable only the specified motor, disable others."""
        for i in range(3):
            enabled = (i == motor_num)
            self.connection.write_param(f'use-motor.{i}', enabled)

    def on_motors_off(self):
        """Handle motors off button click."""
        self.motors.off()
        # Reset use-motors to all disabled
        for i in range(3):
            self.connection.write_param(f'use-motor.{i}', False)

    def new_motors_status(self):
        """Update motor status display."""
        status_text = "ON" if self.motors.is_on == True else ("Off" if self.motors.is_on == False else "unknown")
        self.motor_status_label.setText(status_text)
        # Make text red if motors are ON, default color otherwise
        if status_text == "ON":
            self.motor_status_label.setStyleSheet("color: red;")
        else:
            self.motor_status_label.setStyleSheet("")
        self.update_buttons()

    def update_buttons(self):
        """Update button enabled states."""
        connection_ok = self.connection.is_connected() and not self.connection.calibrating
        self.motor_on_btn.setEnabled(self.motors.is_on == False and connection_ok)
        self.motor_off_btn.setEnabled(connection_ok)
        # PID parameter buttons
        for btn in list(self.send_btns.values()) + self.test_btns:
            btn.setEnabled(connection_ok)
        # TODO: speed setpoint buttons only if have_axes and have_forward

    def start_updates(self):
        """Start updates."""
        self.load_values()
        self.motors.get(self)
        self.new_motors_status() # Calls update_buttons

    def stop_updates(self):
        """Stop updates."""
        self.motor_status_label.setText("unknown")
        self.motor_status_label.setStyleSheet("")
        self.update_buttons()
        self.motors.put(self)


class JointLimitsTab(QWidget):
    """Tab for joint limits calibration and configuration."""

    PATH_LIMIT_SEARCH = 'control_data_s::CONTROL_PATH_LIMIT_SEARCH' # 4 works too
    PATH_INTERPOLATE_JOINT = 'control_data_s::CONTROL_PATH_INTERPOLATE_JOINT' # 0

    def __init__(self, connection, geometry, motors, parent=None):
        super().__init__(parent)
        self.connection = connection
        self.geometry = geometry

        layout = QVBoxLayout()

        # Top row: info label and search button
        top_layout = QHBoxLayout()

        info_label = QLabel(
            "One continuous avoid zone can be defined per joint as a from/to range of angles.  "
            "This can be either a physical limit like a hard stop preventing rotation beyond "
            "some point or an area to avoid for any other reason, like avoiding cable twist.\n\n"
            "The automatic search moves each joints in both directions until it notices a "
            "blockage. It's pretty slow (see 'config.control.limit-search-v' parameter) so "
            "rotating the joints manually to their limit positions and using the \"Get cur\" "
            "buttons may be easier.\n\n"
            "The avoid zone extends from min to max and the assumed safe operating range "
            "from max to min angles, not the other way.  \"Write\" sends new values to the "
            "gimbal but doesn't store them to its non-volatile memory."
        )
        info_label.setWordWrap(True)
        top_layout.addWidget(info_label)

        self.search_btn = QPushButton("Search automatically")
        self.search_btn.setToolTip(
            "Automatically search for joint limits by moving axes to their hard stops.\n"
            "May take up to 4 minutes, cannot be stopped, and requires motors to be on."
        )
        self.search_btn.clicked.connect(self.on_search_automatically)
        top_layout.addWidget(self.search_btn)

        layout.addLayout(top_layout)

        grid_group = QGroupBox("Limit zones")
        grid_layout = QGridLayout()

        self.axis_checkboxes = []
        self.min_inputs = []
        self.max_inputs = []
        self.get_min_btns = []
        self.get_max_btns = []
        self.swap_btns = []
        self.write_btns = []

        # Create rows for each axis
        axis_names = ["Outer", "Middle", "Inner"]
        for axis_num in range(3):
            row = axis_num

            # Zone name
            zone_label = QLabel(f"{axis_names[axis_num]} joint ({axis_num}) avoid zone")
            grid_layout.addWidget(zone_label, row, 0)

            # Checkbox for has-limits
            has_limits_checkbox = QCheckBox()
            has_limits_checkbox.setToolTip(f"Enable/disable avoid zone for axis {axis_num}")
            has_limits_checkbox.stateChanged.connect(self.update_buttons)
            grid_layout.addWidget(has_limits_checkbox, row, 1)
            self.axis_checkboxes.append(has_limits_checkbox)

            from_container = QWidget()
            from_layout = QHBoxLayout(from_container)
            from_layout.setAlignment(Qt.AlignmentFlag.AlignCenter)
            from_layout.addWidget(QLabel("From"))

            # Min spinbox
            min_input = QDoubleSpinBox()
            min_input.setRange(0, 360)
            min_input.setDecimals(1)
            min_input.setSingleStep(1.0)
            min_input.setWrapping(True)
            min_input.setSuffix("°")
            min_input.setToolTip(f"Start angle of the limit zone for axis {axis_num}")
            from_layout.addWidget(min_input)
            self.min_inputs.append(min_input)

            # Get current min button
            get_min_btn = QPushButton("Get cur")
            get_min_btn.setToolTip(f"Load with current joint position.  Not written automatically.")
            get_min_btn.clicked.connect(lambda checked, n=axis_num: self.on_get_current(n, True))
            from_layout.addWidget(get_min_btn)
            self.get_min_btns.append(get_min_btn)

            grid_layout.addWidget(from_container, row, 2)

            to_container = QWidget()
            to_layout = QHBoxLayout(to_container)
            to_layout.setAlignment(Qt.AlignmentFlag.AlignCenter)
            to_layout.addWidget(QLabel("To"))

            # Max spinbox
            max_input = QDoubleSpinBox()
            max_input.setRange(0, 360)
            max_input.setDecimals(1)
            max_input.setSingleStep(1.0)
            max_input.setWrapping(True)
            max_input.setSuffix("°")
            max_input.setToolTip(f"End angle of the limit zone for axis {axis_num}")
            to_layout.addWidget(max_input)
            self.max_inputs.append(max_input)

            # Get current max button
            get_max_btn = QPushButton("Get cur")
            get_max_btn.setToolTip(f"Load with current joint position.  Not written automatically.")
            get_max_btn.clicked.connect(lambda checked, n=axis_num: self.on_get_current(n, False))
            to_layout.addWidget(get_max_btn)
            self.get_max_btns.append(get_max_btn)

            grid_layout.addWidget(to_container, row, 3)

            # Swap button
            swap_btn = QPushButton()
            swap_btn.setMaximumWidth(30)
            swap_btn.setMaximumHeight(25)
            swap_btn.setToolTip(f"Swap min/max limits for axis {axis_num}.  Not written automatically.")
            try:
                swap_btn.setIcon(QIcon.fromTheme("object-flip-horizontal"))
                swap_btn.setText("")
            except:
                swap_btn.setText("↔")
            swap_btn.clicked.connect(lambda checked, n=axis_num: self.on_swap_limits(n))
            grid_layout.addWidget(swap_btn, row, 4)
            self.swap_btns.append(swap_btn)

            # Write button
            write_btn = QPushButton("Write")
            write_btn.setToolTip(f"Actually write the settings for axis {axis_num} to the gimbal")
            try:
                write_btn.setIcon(QIcon.fromTheme('go-next'))
            except:
                pass
            write_btn.clicked.connect(lambda checked, n=axis_num: self.on_write_axis(n))
            grid_layout.addWidget(write_btn, row, 5)
            self.write_btns.append(write_btn)

        tip_label = QLabel("Switch away and back to this tab to reload values from gimbal")
        font = tip_label.font()
        font.setPointSize(int(font.pointSize() * 0.8))
        tip_label.setFont(font)
        grid_layout.addWidget(tip_label, 3, 0, 3, -1, Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)

        grid_group.setLayout(grid_layout)
        layout.addWidget(grid_group)

        # Bottom section: buffer width
        tooltip = "Start braking at this many degrees from the limit zone in any axis"
        buffer_layout = QHBoxLayout()
        label = QLabel("Buffer width around limit angles:")
        label.setToolTip(tooltip)
        buffer_layout.addWidget(label)

        self.buffer_input = QDoubleSpinBox()
        self.buffer_input.setRange(0.0, 45.0)
        self.buffer_input.setDecimals(1)
        self.buffer_input.setSingleStep(1.0)
        self.buffer_input.setSuffix("°")
        self.buffer_input.setToolTip(tooltip)
        buffer_layout.addWidget(self.buffer_input)

        buffer_layout.addStretch()

        self.buffer_write_btn = QPushButton("Write")
        self.buffer_write_btn.setToolTip("Write the 'config.control.limit-margin' parameter")
        try:
            self.buffer_write_btn.setIcon(QIcon.fromTheme('go-next'))
        except:
            pass
        self.buffer_write_btn.clicked.connect(self.on_write_buffer)
        buffer_layout.addWidget(self.buffer_write_btn)

        layout.addLayout(buffer_layout)

        layout.addStretch()
        self.setLayout(layout)

        # Auto-search timer
        self.search_timer = QTimer()
        self.search_timer.timeout.connect(self.check_search_complete)
        self.search_timer.setInterval(1000)  # 1 Hz

        # Connect to signals
        self.geometry.geometry_changed.connect(self.update_values)
        self.connection.calibrating_changed.connect(self.update_buttons)

        # Initial state
        self.update_values()
        self.update_buttons()

    def on_search_automatically(self):
        """Handle automatic limit search."""
        if not self.connection.is_connected() or self.connection.calibrating:
            return
        # If motors are already on, still start the search

        def after_drain():
            # Disable all limits first
            for i in range(3):
                self.connection.write_param(f'config.axes.has-limits.{i}', False)

            self.connection.write_param('control.path-type', self.PATH_LIMIT_SEARCH)
            self.connection.send_command(cmd.CmdId.CMD_MOTORS_ON)

            # Start monitoring
            self.search_timer.start()

        logger.info("Resetting has-limits.N and starting the limit search")
        self.connection.set_calibrating(True)
        self.connection.drain_queues(after_drain)

    def check_search_complete(self):
        """Check if automatic limit search is complete."""
        if not self.connection.is_connected():
            return

        def callback(value):
            if values is not None:
                path_type, motors_on = values
                if not motors_on or str(path_type) != self.PATH_LIMIT_SEARCH:
                    logger.info("Limit search done")
                    self.search_timer.stop()
                    self.connection.set_calibrating(False)
                    if motors_on == True:
                        self.connection.send_command(cmd.CmdId.CMD_MOTORS_OFF, cmd.MotorsOffRequest.build({}))
                    if str(path_type) == self.PATH_LIMIT_SEARCH:
                        self.connection.write_param('control.path-type', self.PATH_INTERPOLATE_JOINT)
                    self.geometry.update()

        # Can't use self.motors.is_on here because self.connection.calibrating is True
        self.connection.read_param(['control.path-type', 'motors-on'], callback)

    def on_get_current(self, axis_num, is_min):
        """Get current position and set as limit."""
        if not self.connection.is_connected() or self.connection.calibrating:
            return

        enc_num = self.geometry.axis_to_encoder[axis_num]

        # Read current angle from the encoder
        def callback(value):
            if value is not None:
                joint_angle = self.geometry.angle_normalize_360(float(value) * self.geometry.encoder_scale[enc_num])
                spinboxes = self.min_inputs if is_min else self.max_inputs
                spinboxes[axis_num].setValue(joint_angle)

        self.connection.read_param(f'encoders.{enc_num}.reading', callback)

    def on_swap_limits(self, axis_num):
        """Swap min and max limit values."""
        min_val = self.min_inputs[axis_num].value()
        max_val = self.max_inputs[axis_num].value()

        self.min_inputs[axis_num].setValue(max_val)
        self.max_inputs[axis_num].setValue(min_val)

    def on_write_axis(self, axis_num):
        """Write limit settings for a specific axis."""
        if not self.connection.is_connected() or self.connection.calibrating:
            return

        enabled = self.axis_checkboxes[axis_num].isChecked()
        min_val = math.radians(self.min_inputs[axis_num].value())
        max_val = math.radians(self.max_inputs[axis_num].value())

        self.connection.write_param(f'config.axes.has-limits.{axis_num}', enabled)
        self.connection.write_param(f'config.axes.limit-min.{axis_num}', min_val)
        self.connection.write_param(f'config.axes.limit-max.{axis_num}', max_val)

        self.geometry.update()
        logger.info("New limit values sent to gimbal for axis " + str(axis_num))

    def on_write_buffer(self):
        """Write buffer width setting."""
        if not self.connection.is_connected() or self.connection.calibrating:
            return

        self.connection.write_param('config.control.limit-margin', math.radians(self.buffer_input.value()))
        logger.info("New 'config.control.limit-margin' sent to gimbal")

    def update_values(self):
        """Update all displayed values from gimbal."""
        for i in range(3):
            self.axis_checkboxes[i].setChecked(self.geometry.has_limits[i])
            self.min_inputs[i].setValue(self.geometry.limit_min[i])
            self.max_inputs[i].setValue(self.geometry.limit_max[i])

        self.update_buttons()

    def update_buttons(self):
        """Update button enabled states."""
        enabled = (self.connection.is_connected() and
                  not self.connection.calibrating and
                  self.geometry.have_axes)

        self.search_btn.setEnabled(enabled)

        # Enable/disable axis controls
        for axis_num in range(3):
            self.write_btns[axis_num].setEnabled(enabled)
            self.axis_checkboxes[axis_num].setEnabled(enabled)

            axis_enabled = enabled and self.axis_checkboxes[axis_num].isChecked()
            self.min_inputs[axis_num].setEnabled(axis_enabled)
            self.max_inputs[axis_num].setEnabled(axis_enabled)
            self.get_min_btns[axis_num].setEnabled(axis_enabled)
            self.get_max_btns[axis_num].setEnabled(axis_enabled)
            self.swap_btns[axis_num].setEnabled(axis_enabled)

        self.buffer_input.setEnabled(enabled)
        self.buffer_write_btn.setEnabled(enabled)

    def start_updates(self):
        """Start updates."""
        self.update_values() # calls update_buttons()

        if not self.connection.is_connected() or self.connection.calibrating:
            return

        def buffer_cb(value):
            if value is not None:
                self.buffer_input.setValue(math.degrees(float(value)))

        self.connection.read_param('config.control.limit-margin', buffer_cb)

        # Check if a limit search is ongoing
        def path_type_cb(values):
            if values is not None:
                path_type, motors_on = values
                if bool(motors_on) and str(path_type) == self.PATH_LIMIT_SEARCH:
                    self.search_timer.start()
                    self.connection.set_calibrating(True)

        self.connection.read_param(['control.path-type', 'motors-on'], path_type_cb)

    def stop_updates(self):
        """Stop updates."""
        self.search_timer.stop()
        self.update_buttons()


class VbatSenseTab(QWidget):
    def __init__(self, connection, parent=None):
        super().__init__(parent)
        self.connection = connection

        layout = QVBoxLayout()

        info_label = QLabel(
            "Set up correct battery voltage (Vbat) sensing and low-voltage cut-off "
            "(LVCO) to protect the battery.\n"
            "The voltage scale may need to be adjusted if the voltage reported by "
            "OpenBGC is consistently lower or higher than actual battery voltage.  "
            "Different controller boards will have different voltage divider values "
            "to adapt the input range to values that the Analog-to-Digital converter "
            "can handle resulting in different scales.\n"
            "The LVCO minimum voltage will depend on the battery type used, for "
            "Li-Po packs set to 3.3V times cell count (in series)."
        )
        info_label.setWordWrap(True)
        layout.addWidget(info_label)

        grid_layout = QGridLayout()

        tooltip = "Voltage currently reported by OpenBGC using default\nscale or what it was last set to."
        info_label = QLabel("Detected voltage: ")
        info_label.setToolTip(tooltip)
        grid_layout.addWidget(info_label, 0, 0, Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        self.cur_label = QLabel("-")
        self.cur_label.setToolTip(tooltip)
        grid_layout.addWidget(self.cur_label, 0, 1)

        tooltip = (
            "To set correct voltage scale provide the actual voltage as measured by an\n"
            "external tool (smart charger or multimeter), or the design max. voltage if\n"
            "the battery is known to be fully charged, or the power supply's output\n"
            "voltage if one is connected."
        )
        info_label = QLabel("Actual voltage: ")
        info_label.setToolTip(tooltip)
        grid_layout.addWidget(info_label, 1, 0, Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        self.actual_input = QDoubleSpinBox()
        self.actual_input.setRange(0.5, 100.0)
        self.actual_input.setDecimals(1)
        self.actual_input.setSingleStep(0.2)
        self.actual_input.setSuffix("V")
        self.actual_input.setToolTip(tooltip)
        self.actual_input.setMaximumWidth(80)
        grid_layout.addWidget(self.actual_input, 1, 1)
        self.scale_btn = QPushButton("Update scale")
        try:
            self.scale_btn.setIcon(QIcon.fromTheme('go-next'))
        except:
            pass
        self.scale_btn.setToolTip(
            "Set new scale based on the detected and actual voltages.\n"
            "Battery or power supply must be connected."
        )
        self.scale_btn.clicked.connect(self.on_set_scale)
        grid_layout.addWidget(self.scale_btn, 1, 2, Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)

        tooltip = (
            "If the input voltage goes below this value, motors or any other load under\n"
            "OpenBGC control will be disabled."
        )
        info_label = QLabel("Minimum voltage: ")
        info_label.setToolTip(tooltip)
        grid_layout.addWidget(info_label, 2, 0, Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        self.min_input = QDoubleSpinBox()
        self.min_input.setRange(0.5, 100.0)
        self.min_input.setDecimals(1)
        self.min_input.setSingleStep(0.2)
        self.min_input.setSuffix("V")
        self.min_input.setToolTip(tooltip)
        self.min_input.setMaximumWidth(80)
        grid_layout.addWidget(self.min_input, 2, 1)
        self.min_btn = QPushButton("Update")
        try:
            self.min_btn.setIcon(QIcon.fromTheme('go-next'))
        except:
            pass
        self.min_btn.clicked.connect(self.on_set_lvco)
        grid_layout.addWidget(self.min_btn, 2, 2, Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)

        grid_layout.setColumnStretch(0, 2)
        grid_layout.setColumnStretch(1, 0)
        grid_layout.setColumnStretch(2, 2)
        layout.addLayout(grid_layout)

        layout.addStretch()
        self.setLayout(layout)

        # Auto-search timer
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_vbat)
        self.update_timer.setInterval(1000)  # 1 Hz

        # Connect to signals
        self.connection.calibrating_changed.connect(self.update_buttons)

        # Initial state
        self.update_values()

    def on_set_scale(self):
        def callback(values):
            if values is None:
                return

            cur_vbat, cur_scale = values    # cur_vbat in bat mV, cur_scale in 0.001 (bat V / adc V)
            if cur_vbat == 0 or cur_scale == 0:
                logger.error("VBAT must be positive")
                return
            raw_vbat = cur_vbat / cur_scale # in adc V
            actual_vbat = self.actual_input.value() * 1000 # in bat mV
            new_scale = actual_vbat / raw_vbat # in 0.001 (bat V / adc V) again
            self.connection.write_param("config.vbat.scale", int(new_scale))

        self.connection.read_param(["vbat", "config.vbat.scale"], callback)

    def on_set_lvco(self):
        new_lvco = self.min_input.value() * 1000
        self.connection.write_param("config.vbat.lvco", int(new_lvco))

    def update_values(self):
        self.update_actual = True
        self.vbat = 0
        self.update_buttons()

        def callback(value):
            if value is not None:
                self.min_input.setValue(value * 0.001)

        self.connection.read_param("config.vbat.lvco", callback)

    def update_buttons(self):
        enabled = self.connection.is_connected() and not self.connection.calibrating

        self.scale_btn.setEnabled(enabled and self.vbat > 0 and not self.update_actual)
        self.min_btn.setEnabled(enabled)

    def update_vbat(self):
        if not self.connection.is_connected() or self.connection.calibrating:
            return

        def callback(value):
            if value is None:
                self.cur_label.setText('-')
                self.vbat = 0

            self.vbat = value * 0.001
            self.cur_label.setText(f'{self.vbat:.1f}V')
            if self.update_actual:
                self.update_actual = False
                self.actual_input.setValue(self.vbat)
            self.update_buttons()

        self.connection.read_param("vbat", callback)

    def start_updates(self):
        """Start updates."""
        self.update_values() # calls update_buttons()
        self.update_timer.start()

    def stop_updates(self):
        """Stop updates."""
        self.update_timer.stop()
        self.update_buttons()
