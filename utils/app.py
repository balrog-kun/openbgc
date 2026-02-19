#! /usr/bin/python3
# vim: set ts=4 sw=4 sts=4 et :
"""
PyQt GUI companion app for gimbals running OpenBGC
"""

import sys
import math
import threading
import time
import logging
import argparse
import traceback
import platform
import json
from typing import Optional, Callable, Any
import construct
import random
import os

try:
    from PyQt6.QtWidgets import (
        QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
        QLabel, QLineEdit, QPushButton, QListWidget, QStackedWidget,
        QTreeWidget, QTreeWidgetItem, QGroupBox, QGridLayout, QCheckBox,
        QSplitter, QDoubleSpinBox, QSpinBox, QComboBox, QFileDialog,
        QScrollArea, QSizePolicy, QSpacerItem,
        # Qt 6 only
        QAbstractSpinBox
    )
    from PyQt6.QtGui import QIcon
    from PyQt6.QtCore import (
        Qt, QTimer, pyqtSignal, QObject, QSocketNotifier, QSize, QTime
    )
    from PyQt6.QtGui import QCursor
    PYQT_VERSION = 6
except ImportError:
    from PyQt5.QtWidgets import (
        QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
        QLabel, QLineEdit, QPushButton, QListWidget, QStackedWidget,
        QTreeWidget, QTreeWidgetItem, QGroupBox, QGridLayout, QCheckBox,
        QSplitter, QDoubleSpinBox, QSpinBox, QComboBox, QFileDialog,
        QScrollArea, QSizePolicy, QSpacerItem,
    )
    from PyQt5.QtGui import QIcon
    from PyQt5.QtCore import (
        Qt, QTimer, pyqtSignal, QObject, QSocketNotifier, QSize, QTime
    )
    from PyQt5.QtGui import QCursor
    PYQT_VERSION = 5

# Platform-specific serial port monitoring
PORT_MONITORING_AVAILABLE = False
try:
    if platform.system() == 'Linux':
        import pyudev
        PORT_MONITORING_AVAILABLE = True
    elif platform.system() == 'Windows':
        import wmi
        import pythoncom
        PORT_MONITORING_AVAILABLE = True
except ImportError:
    PORT_MONITORING_AVAILABLE = False
    print("Port list monitoring not available")

import serial
import serial.tools.list_ports

import sbgcserialapi.cmd as cmd
import sbgcserialapi.cmd_obgc as cmd_obgc
import sbgcserialapi.frame as fr
import sbgcserialapi.unit as sbgc_unit

import param_defs
import param_utils

import app_widgets
import app_iio
import app_calib_tabs as calib

# Set up logger
logger = logging.getLogger(__name__)


class GimbalGeometry(QObject):
    """Data class for gimbal geometry information."""

    geometry_changed = pyqtSignal()

    def __init__(self, connection: 'GimbalConnection', parent=None):
        super().__init__(parent)
        self.connection = connection

        self._reset_to_defaults()

        # Connect to connection changes
        self.connection.connection_changed.connect(self.on_connection_changed)

    def on_connection_changed(self, connected: bool):
        """Handle connection state changes."""
        if connected:
            self.request_geometry()
        else:
            self.reset_to_defaults()

    def request_geometry(self):
        """Request all geometry parameters from the gimbal."""
        # TODO: use one array of (param_id, local attr name, default value), base everything
        # else on this array

        # Request flags
        self.connection.read_param("config.have-axes", self.update_have_axes)
        self.connection.read_param("config.control.have-home", self.update_have_home)
        self.connection.read_param("config.control.have-parking", self.update_have_parking)
        self.connection.read_param("config.control.have-forward", self.update_have_forward)

        # Request axes data
        axes_params = [f"config.axes.axes.{i}" for i in range(3)]
        self.connection.read_param(axes_params, self.update_axes)

        axis_to_encoder_params = [f"config.axes.axis-to-encoder.{i}" for i in range(3)]
        self.connection.read_param(axis_to_encoder_params, self.update_axis_to_encoder)

        encoder_scale_params = [f"config.axes.encoder-scale.{i}" for i in range(3)]
        self.connection.read_param(encoder_scale_params, self.update_encoder_scale)

        # Request other data
        self.connection.read_param("config.axes.main-imu-mount-q", self.update_main_imu_mount_q)
        self.connection.read_param("config.control.home-q", self.update_home_q)
        self.connection.read_param("config.control.home-frame-q", self.update_home_frame_q)
        self.connection.read_param("config.control.home-angles", self.update_home_angles)
        self.connection.read_param("config.control.park-angles", self.update_park_angles)
        self.connection.read_param("config.control.forward-vec", self.update_forward_vec)

        # Request limit information
        has_limits_params = [f"config.axes.has-limits.{i}" for i in range(3)]
        self.connection.read_param(has_limits_params, self.update_has_limits)

        limit_min_params = [f"config.axes.limit-min.{i}" for i in range(3)]
        self.connection.read_param(limit_min_params, self.update_limit_min)

        limit_max_params = [f"config.axes.limit-max.{i}" for i in range(3)]
        self.connection.read_param(limit_max_params, self.update_limit_max_and_finish)

    def _reset_to_defaults(self):
        self.have_axes = False
        self.have_home = False
        self.have_parking = False
        self.have_forward = False
        self.axes = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
        self.axis_to_encoder = [0, 1, 2]
        self.encoder_scale = [1.0, 1.0, 1.0]
        self.main_imu_mount_q = (1.0, 0.0, 0.0, 0.0)
        self.home_angles = [0.0, 0.0, 0.0]
        self.home_q = (1.0, 0.0, 0.0, 0.0)
        self.home_frame_q = (1.0, 0.0, 0.0, 0.0)
        self.park_angles = [0.0, 0.0, 0.0]
        self.forward_vec = (1.0, 0.0)
        self.has_limits = [False, False, False]
        self.limit_min = [0.0, 0.0, 0.0]
        self.limit_max = [0.0, 0.0, 0.0]

        self.update_aligned_home_q()

        self.loaded = False

    def reset_to_defaults(self):
        """Reset all values to defaults."""
        self._reset_to_defaults()
        self.geometry_changed.emit()

    def update(self):
        """Manual update trigger."""
        if self.connection.is_connected():
            self.request_geometry()

    def load_error(self):
        self._reset_to_defaults()
        # TODO: retry in a moment

    # Update methods for individual parameters
    def update_have_axes(self, value):
        if value is not None:
            self.have_axes = bool(value)
        else:
            self.load_error()

    def update_have_home(self, value):
        if value is not None:
            self.have_home = bool(value)
            self.update_aligned_home_q()
        else:
            self.load_error()

    def update_have_parking(self, value):
        if value is not None:
            self.have_parking = bool(value)
        else:
            self.load_error()

    def update_have_forward(self, value):
        if value is not None:
            self.have_forward = bool(value)
            self.update_aligned_home_q()
        else:
            self.load_error()

    def update_axes(self, values):
        if values and len(values) == 3:
            self.axes = tuple([tuple(v) for v in values])
        else:
            self.load_error()

    def update_axis_to_encoder(self, values):
        if values and len(values) == 3:
            self.axis_to_encoder = [int(v) for v in values]
        else:
            self.load_error()

    def update_encoder_scale(self, values):
        if values and len(values) == 3:
            self.encoder_scale = [float(v) for v in values]
        else:
            self.load_error()

    def update_main_imu_mount_q(self, value):
        if value:
            self.main_imu_mount_q = tuple([float(v) for v in value])
        else:
            self.load_error()

    def update_home_angles(self, value):
        if value is not None:
            self.home_angles = [math.degrees(value[i]) for i in range(3)]
        else:
            self.load_error()

    def update_home_q(self, value):
        if value is not None:
            self.home_q = tuple([float(v) for v in value])
            self.update_aligned_home_q()
        else:
            self.load_error()

    def update_home_frame_q(self, value):
        if value is not None:
            self.home_frame_q = tuple([float(v) for v in value])
        else:
            self.load_error()

    def update_park_angles(self, value):
        if value is not None:
            self.park_angles = [math.degrees(value[i]) for i in range(3)]
        else:
            self.load_error()

    def update_forward_vec(self, value):
        if value is not None:
            self.forward_vec = tuple([float(v) for v in value])
            self.update_aligned_home_q()
        else:
            self.load_error()

    def update_has_limits(self, values):
        if values is not None:
            self.has_limits = [bool(v) for v in values]
        else:
            self.load_error()

    def update_limit_min(self, values):
        if values and len(values) == 3:
            self.limit_min = [math.degrees(v) for v in values]
        else:
            self.load_error()

    def update_limit_max_and_finish(self, values):
        if values and len(values) == 3:
            self.limit_max = [math.degrees(v) for v in values]

            self.loaded = True
            self.geometry_changed.emit()
        else:
            self.load_error()

    def update_aligned_home_q(self):
        if self.have_forward:
            self.forward_az = math.atan2(self.forward_vec[1], self.forward_vec[0])
        else:
            self.forward_az = math.pi / 2

        # Rotate home_q in the negative yaw direction to align "forward" with 0-yaw
        yaw_rotate = self.quaternion_from_axis_angle((0, 0, 1), -self.forward_az)
        aligned_home_q = self.quaternion_multiply(yaw_rotate, self.home_q)
        self.conj_aligned_home_q = self.quaternion_conjugate(aligned_home_q)

    def get_joint_angles(self, encoder_angles):
        return [ encoder_angles[enc_num] * self.encoder_scale[enc_num] for enc_num in self.axis_to_encoder ]

    def get_home_angles(self):
        return self.get_joint_angles(self.home_angles)

    def get_park_angles(self):
        return self.get_joint_angles(self.park_angles)

    def get_rel_q(self, joint_angles):
        q = [ self.quaternion_from_axis_angle(self.axes[anum], math.radians(joint_angles[anum])) for anum in range(3) ]
        q01 = self.quaternion_multiply(q[0], q[1])
        q23 = self.quaternion_multiply(q[2], self.main_imu_mount_q)
        return self.quaternion_multiply(q01, q23)

    def get_euler(self, joint_angles, main_imu_q):
        main_pose_q = self.quaternion_multiply(main_imu_q, self.conj_aligned_home_q)
        yaw, pitch, roll = self.quaternion_to_euler(main_pose_q)

        # frame_pose_q is global and should include both physical rotation of the frame
        # from the home orientation, and the accumlulated main IMU yaw drift so we need that
        # to correct main_imu_q
        #
        # Note: this assumes yaw-following (which is the default but optional) so the
        # yaw 0 angle is in reference to frame
        conj_rel_q = self.quaternion_conjugate(self.get_rel_q(joint_angles))
        frame_q = self.quaternion_multiply(main_imu_q, conj_rel_q)
        frame_pose_q = self.quaternion_multiply(frame_q, self.quaternion_conjugate(self.home_frame_q))
        rel_yaw = self.quaternion_to_euler(frame_pose_q)[0]

        return (
            self.angle_normalize_180(math.degrees(yaw - rel_yaw - self.forward_az)),
            math.degrees(pitch), math.degrees(roll)
        )

    # TODO: move all the math utils to app_math.py or similar
    def quaternion_multiply(self, q1, q2):
        """Multiply two quaternions: q1 * q2."""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return (
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        )

    def quaternion_conjugate(self, q):
        """Return conjugate of quaternion."""
        return (q[0], -q[1], -q[2], -q[3])

    def quaternion_to_matrix(self, q):
        """Convert quaternion to rotation matrix."""
        w, x, y, z = q
        return (
            (1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
            (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
            (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y))
        )

    def quaternion_to_euler(self, q):
        # Roll (X-axis rotation)
        sinr_cosp = 2 * (q[0] * q[1] + q[2] * q[3])
        cosr_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2])
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (Y-axis rotation)
        sinp = 2 * (q[0] * q[2] - q[3] * q[1])
        pitch = math.asin(sinp) if abs(sinp) < 1 else (math.pi if sinp >= 0 else -math.pi)

        # Yaw (Z-axis rotation)
        siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2])
        cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3])
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return (yaw, pitch, roll)

    def quaternion_from_axis_angle(self, axis, angle):
        """Convert axis-angle to quaternion."""
        half_angle = angle * 0.5
        s = math.sin(half_angle)
        c = math.cos(half_angle)
        return tuple([c, *self.vector_mult_scalar(axis, s)])

    def vector_rotate(self, v, q):
        """Rotate vector v by quaternion q."""
        # v' = q * v * q*
        # Simplified: use rotation matrix
        m = self.quaternion_to_matrix(q)
        return (self.vector_dot(m[0], v), self.vector_dot(m[1], v), self.vector_dot(m[2], v))

    def vector_sum(self, v0, v1):
        return (v0[0] + v1[0], v0[1] + v1[1], v0[2] + v1[2])

    def vector_sub(self, v0, v1):
        return (v0[0] - v1[0], v0[1] - v1[1], v0[2] - v1[2])

    def vector_dot(self, v0, v1):
        return v0[0] * v1[0] + v0[1] * v1[1] + v0[2] * v1[2]

    def vector_cross(self, v0, v1):
        return (v0[1] * v1[2] - v0[2] * v1[1], v0[2] * v1[0] - v0[0] * v1[2], v0[0] * v1[1] - v0[1] * v1[0])

    def vector_mult_scalar(self, v, s):
        return (*(c * s for c in v),)

    def vector_norm(self, v):
        return math.hypot(*v)

    def matrix_t(self, m):
        return [[m[i][j] for i in range(3)] for j in range(3)]

    def angle_normalize_180(self, angle):
        angle = math.fmod(angle, 360)
        if angle >= 180:
            return angle - 360
        if angle < -180:
            return angle + 360
        return angle

    def angle_normalize_360(self, angle):
        angle = math.fmod(angle, 360)
        return angle if angle >= 0 else angle + 360

    def angle_normalize_pi(self, angle):
        angle = math.fmod(angle, 2 * math.pi)
        if angle >= math.pi:
            return angle - 2 * math.pi
        if angle < -math.pi:
            return angle + 2 * math.pi
        return angle


class GimbalConnection(QObject):
    """Handles serial communication with the gimbal."""

    param_received = pyqtSignal(str, object) # param_name, value
    connection_changed = pyqtSignal(bool)    # connected
    error_occurred = pyqtSignal(str)         # error message
    text_logged = pyqtSignal(str)
    busy_state_changed = pyqtSignal(bool)    # busy: True when read queues busy or calibrating
    calibrating_changed = pyqtSignal(bool)

    def __init__(self, debug=False):
        super().__init__()
        self.debug = debug
        self.port: Optional[serial.Serial] = None
        self.port_path: Optional[str] = None
        self.pending_requests: list = [] # Queue of (param_id, callback, pdef, param_name) - waiting for response
        self.queued_requests: list = []  # Queue of (param_name, callback) - waiting to be sent
        self.max_pending_requests = 4
        self._busy_timer = QTimer()
        self._busy_timer.setSingleShot(True)
        self._busy_timer.timeout.connect(self._on_busy_timeout)
        self._is_busy_signaled = False
        self.calibrating = False  # Flag indicating calibration is in progress
        self.in_stream = fr.InStream(
            text_cb=self._text_cb,
            frame_cb=self._frame_cb,
            debug_cb=self._debug_cb if debug else None
        )
        self.serial_notifier: Optional[QSocketNotifier] = None
        self.new_connection = False

    def _debug_cb(self, info):
        """Debug callback for InStream."""
        logger.debug(f"InStream: {info}")

    def connect(self, port_path: str) -> bool:
        """Connect to the gimbal at the specified port."""
        if self.port is not None:
            return False

        try:
            self.port = serial.Serial(port_path, baudrate=115200, timeout=0, write_timeout=1)
            self.serial_notifier = QSocketNotifier(self.port.fileno(), QSocketNotifier.Type.Read, self)
            self.serial_notifier.activated.connect(self._read_serial_data)
            self.serial_notifier.setEnabled(True)
            self.port_path = port_path
            self.new_connection = True
            return True
        except Exception as e:
            self.port = None
            error_msg = f"Connection error: {e}"
            logger.debug(error_msg)
            self.error_occurred.emit(error_msg)
            return False

    def disconnect(self):
        """Disconnect from the gimbal."""
        if self.port is None:
            return

        if self.port:
            if self.serial_notifier:
                self.serial_notifier.setEnabled(False)
                self.serial_notifier.deleteLater()
                self.serial_notifier = None
            self.port.close()
            self.port = None
        self.port_path = None
        self.pending_requests.clear()
        self.queued_requests.clear()
        if not self.new_connection:
            self.connection_changed.emit(False)
        self.new_connection = False
        self.set_calibrating(False)
        self._check_busy_state()

    def is_connected(self) -> bool:
        """Check if connected."""
        return self.port is not None and self.port.is_open and not self.new_connection

    def _read_serial_data(self):
        """Read available data from the serial port and feed it to the InStream."""
        try:
            if self.port and self.port.is_open and self.port.in_waiting > 0:
                data = self.port.read(self.port.in_waiting)
                self.in_stream.feed(data)
        except Exception as e:
            error_msg = f"Serial read error: {e}:\n{traceback.format_exc()}"
            logger.debug(error_msg)
            self.error_occurred.emit(error_msg)
            self.disconnect()

    def _text_cb(self, text: str):
        """Handle text from serial port -- anything that's not a protocol frame."""
        self.text_logged.emit(text)

    def _frame_cb(self, frame):
        """Handle parsed frames from serial port."""
        if self.debug:
            logger.debug(f"Frame received: cmd_id={frame.hdr.cmd_id}, size={frame.hdr.size}, payload_size={len(frame.pld.data)}")

        if frame.hdr.cmd_id == cmd.CmdId.CMD_ERROR:
            in_payload = cmd.ErrorResponse.parse(frame.pld.data)
            if in_payload.cmd_id == int(cmd_obgc.CmdId.CMD_OBGC) and in_payload.error_data[0] == int(cmd_obgc.SubcmdId.GET_PARAM):
                if self.pending_requests:
                    param_ids, callback, pdefs, param_names = self.pending_requests.pop(0)
                    if self.debug:
                        logger.debug(f"Processing error for param_ids={param_ids} ({param_names}), actual_size={len(param_data)}")
                    self._handle_param_response(param_ids, None, callback, pdefs, param_names)
                    self._send_queued_requests()
                    return

            logger.debug(f"CMD_ERROR received: {in_payload}, payload_size={len(frame.pld.data)}")
            self.error_occurred.emit(f"Remote error: {in_payload}")
            return

        if frame.hdr.cmd_id == int(cmd_obgc.CmdId.CMD_OBGC):
            # GetParamResponse is just GreedyBytes - the raw parameter value
            param_data = frame.pld.data

            if self.debug:
                logger.debug(f"CMD_OBGC response: payload_size={len(param_data)}, pending_requests={len(self.pending_requests)}")

            # Process the first pending request (responses come in order)
            if self.pending_requests:
                param_ids, callback, pdefs, param_names = self.pending_requests.pop(0)
                if self.debug:
                    logger.debug(f"Processing response for param_ids={param_ids} ({param_names}), actual_size={len(param_data)}")
                self._handle_param_response(param_ids, param_data, callback, pdefs, param_names)

                # Try to send more requests from queue now that we have capacity
                self._send_queued_requests()
            else:
                if self.debug:
                    logger.warning(f"CMD_OBGC response received but no pending requests")

    def read_param(self, param_names, callback: Callable[[Any], None]):
        """Read one or more parameters by name. Callback receives the parsed values as a list."""
        if self.port is None:
            callback(None)
            return

        # Convert single parameter to list
        if isinstance(param_names, str):
            param_names = [param_names]

        # Check if the first parameter is already being requested
        first_param = param_names[0]
        for queued_req in self.queued_requests:
            if queued_req[0] == first_param or (isinstance(queued_req[0], list) and queued_req[0][0] == first_param):
                return
        for pending_req in self.pending_requests:
            if pending_req[3] == first_param or (isinstance(pending_req[3], list) and pending_req[3][0] == first_param):
                return

        # Queue the request
        self.queued_requests.append((param_names, callback))

        # Try to send requests from queue
        self._send_queued_requests()

    def _send_queued_requests(self):
        """Send requests from the queue if we have capacity."""
        while len(self.pending_requests) < self.max_pending_requests and self.queued_requests:
            param_names, callback = self.queued_requests.pop(0)
            self._send_single_request(param_names, callback)

        self._check_busy_state()

    def _send_single_request(self, param_names, callback: Callable[[Any], None]):
        """Send a parameter request for one or more parameters."""
        try:
            # Convert single parameter to list
            if isinstance(param_names, str):
                param_names = [param_names]

            # Find parameter definitions
            pdefs = []
            param_ids = []
            for param_name in param_names:
                try:
                    pdef = param_defs.params[int(param_name, 0)]
                except:
                    by_name = {pdef.name: pdef for pdef in param_defs.params.values()}
                    if param_name not in by_name:
                        self._handle_param_error(callback)
                        return
                    pdef = by_name[param_name]
                pdefs.append(pdef)
                param_ids.append(pdef.id)

            # Build request
            out_payload = cmd_obgc.GetParamRequest.build(dict(param_ids=param_ids))
            out_frame = fr.FrameV1.build(dict(
                hdr=dict(cmd_id=cmd_obgc.CmdId.CMD_OBGC, size=len(out_payload)),
                pld=out_payload
            ))

            if self.debug:
                logger.debug(f"Request param_ids={param_ids} ({param_names}), request_size={len(out_payload)}, pending_requests={len(self.pending_requests)}, queued_requests={len(self.queued_requests)}")

            # Store callback for this request (queue-based, process in order)
            self.pending_requests.append((param_ids, callback, pdefs, param_names))

            # Send request
            self.port.write(out_frame)

        except Exception as e:
            error_msg = f"Read param error: {e}"
            logger.error(error_msg)
            self.error_occurred.emit(error_msg)
            self._handle_param_error(callback)

    def _handle_param_error(self, callback):
        callback(None)

        if self.new_connection:
            self.disconnect()

    def _handle_param_response(self, param_ids, data, callback, pdefs, param_names):
        """Handle a parameter response for one or more parameters."""
        if data is None:
            self._handle_param_error(callback)
            return

        total_size = sum([pdef.size for pdef in pdefs])
        if len(data) != total_size:
            error_msg = f"Reading params {str(param_names)} size mismatch: expected {total_size}, got {len(data)}"
            logger.error(error_msg)
            self._handle_param_error(callback)
            # TODO: clear pending_requests but do invoke the callbacks, ratelimit messages
            # If this happens, we should recover sync after clearing queue and ignoring a few responses
            # Maybe we should move pending_requests back to queued_requests
            logger.error("Discarding queues")
            self.pending_requests.clear()
            # Also clear queued_requests as a way to ignoring the next few responses, otherwise
            # we'd immediately send more requests and try to match responses against them
            self.queued_requests.clear()
            return

        values = []
        try:
            # Parse the array of parameter values
            offset = 0
            for i, pdef in enumerate(pdefs):
                param_data = data[offset:offset + pdef.size]
                offset += pdef.size
                param_type_cls = param_utils.ctype_to_construct(pdef.typ, pdef.size)
                val = list_convert(param_type_cls.parse(param_data))
                values.append(val)
        except Exception as e:
            error_msg = f"Parse param response error: {e}"
            logger.error(error_msg)
            if self.debug:
                logger.debug(f"Param response parse error, data={data.hex()}")
            self.error_occurred.emit(error_msg)
            self._handle_param_error(callback)
            return

        if self.new_connection:
            self.new_connection = False
            self.connection_changed.emit(True)

        if len(values) == 1:
            callback(values[0])
        else:
            callback(values)

    def send_command(self, cmd_id, payload=None):
        """Send a command to the gimbal."""
        if self.port is None:
            return False

        try:
            if payload is None:
                payload = b''
            out_frame = fr.FrameV1.build(dict(
                hdr=dict(cmd_id=cmd_id),
                pld=payload
            ))
            self.port.write(out_frame)
            return True
        except Exception as e:
            error_msg = f"Send command error: {e}"
            logger.error(error_msg)
            self.error_occurred.emit(error_msg)
            return False

    def _check_busy_state(self):
        """Check if request queues are busy and manage signaling."""
        busy = len(self.pending_requests) > 0 or self.calibrating

        if busy and not self._is_busy_signaled:
            # Queues became busy, start 100ms timer
            if not self._busy_timer.isActive():
                self._busy_timer.start(100)
        elif not busy:
            self._busy_timer.stop()
            # Queues became empty, signal immediately
            if self._is_busy_signaled:
                self._is_busy_signaled = False
                self.busy_state_changed.emit(False)

            self._check_drain_queues()

    def _on_busy_timeout(self):
        """Called after 100ms of continuous busy state."""
        self._is_busy_signaled = True
        self.busy_state_changed.emit(True)

    def set_calibrating(self, val):
        self.calibrating = val
        self.calibrating_changed.emit(val)

        if val and not self._is_busy_signaled:
            self._busy_timer.stop()
            self._is_busy_signaled = True
            self.busy_state_changed.emit(True)

        if not val:
            self._check_busy_state()

    def _check_drain_queues(self):
        if not hasattr(self, 'drain_cb'):
            return

        if not self.pending_requests and not self.queued_requests:
            cb = self.drain_cb
            del self.drain_cb
            cb()

    def drain_queues(self, callback):
        """Wait for request queues to be empty, then call callback."""
        self.drain_cb = callback
        self._check_drain_queues()

    def send_raw(self, data: bytes):
        """Send raw data to the serial port."""
        if self.port is None:
            raise Exception("Not connected")
        self.port.write(data)

    def control(self, angles=None, speeds=None, nofollow=None): # degrees and degrees/s respectively
        modes = [
            cmd.ControlMode.MODE_ANGLE if angles and angles[i] is not None else
            cmd.ControlMode.MODE_SPEED if speeds and speeds[i] is not None else
            cmd.ControlMode.MODE_IGNORE for i in range(3)
        ]
        targets = [
            {'angle': int(angles[i] / sbgc_unit.degree_factor)} if modes[i] == cmd.ControlMode.MODE_ANGLE else
            {'speed': int(speeds[i] / sbgc_unit.degree_per_sec_factor)} if modes[i] == cmd.ControlMode.MODE_SPEED else
            None for i in range(3)
        ]
        modes = [ modes[i] | (0 if nofollow and nofollow[i] else cmd.ControlMode.CONTROL_FLAG_MIX_FOLLOW) for i in range(3) ]
        return self.send_command(cmd.CmdId.CMD_CONTROL, cmd.ControlRequest.build(dict(control_mode=modes, target=targets)))

    def write_param(self, param_name, value):
        by_name = {pdef.name: pdef for pdef in param_defs.params.values()}
        pdef = by_name[param_name]
        param_type_cls = param_utils.ctype_to_construct(pdef.typ, pdef.size)
        value_bytes = param_type_cls.build(value)

        # TODO: perhaps drain the read queue before doing this, for now let callers do it if needed
        return self.send_command(cmd_obgc.CmdId.CMD_OBGC, cmd_obgc.SetParamRequest.build(dict(param_id=pdef.id, value=value_bytes)))


def list_convert(val):
    """Recursively convert construct lists to Python lists."""
    if isinstance(val, list):
        return [list_convert(elem) for elem in val]
    return val


class PortMonitor(QObject):
    """Cross-platform serial port monitoring with fallback to polling."""

    ports_changed = pyqtSignal(list)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.monitor_thread = None
        self.monitoring = False

        if PORT_MONITORING_AVAILABLE:
            if platform.system() == 'Linux':
                self.start_linux_monitoring()
            elif platform.system() == 'Windows':
                self.start_windows_monitoring()
            # else: no monitoring on unsupported platforms

    def start_linux_monitoring(self):
        """Monitor serial ports using udev on Linux."""
        def monitor_loop():
            try:
                context = pyudev.Context()
                monitor = pyudev.Monitor.from_netlink(context)
                monitor.filter_by(subsystem='tty')
                self.monitoring = True

                for device in iter(monitor.poll, None):
                    if not self.monitoring:
                        break
                    if device.action in ('add', 'remove'):
                        self.update_ports()
            except Exception as e:
                logger.warning(f"Linux port monitoring failed: {e}")
                self.start_polling_monitoring()

        self.monitor_thread = threading.Thread(target=monitor_loop, daemon=True)
        self.monitor_thread.start()

    def start_windows_monitoring(self):
        """Monitor serial ports using WMI on Windows."""
        def monitor_loop():
            try:
                pythoncom.CoInitialize()
                c = wmi.WMI()

                # Watch for creation
                create_watcher = c.watch_for(
                    notification_type="Creation",
                    wmi_class="Win32_SerialPort"
                )

                # Watch for deletion
                delete_watcher = c.watch_for(
                    notification_type="Deletion",
                    wmi_class="Win32_SerialPort"
                )

                self.monitoring = True
                while self.monitoring:
                    try:
                        # Wait for either creation or deletion
                        result = create_watcher(timeout_ms=1000)
                        if result:
                            self.update_ports()
                        else:
                            # Check for deletions
                            try:
                                delete_result = delete_watcher(timeout_ms=1)
                                if delete_result:
                                    self.update_ports()
                            except:
                                pass
                    except wmi.x_wmi_timed_out:
                        continue
                    except Exception as e:
                        logger.warning(f"Windows port monitoring error: {e}")
                        break

            except Exception as e:
                logger.warning(f"Windows port monitoring failed: {e}")
                self.start_polling_monitoring()

        self.monitor_thread = threading.Thread(target=monitor_loop, daemon=True)
        self.monitor_thread.start()

    def start_polling_monitoring(self):
        """Fallback polling-based monitoring."""
        def poll_loop():
            last_ports = set()

            while self.monitoring:
                try:
                    ports = serial.tools.list_ports.comports()
                    current_ports = {f"{p.device} - {p.description}" for p in ports}

                    if current_ports != last_ports:
                        last_ports = current_ports
                        self.ports_changed.emit(ports)

                    time.sleep(2.0)  # Poll every 2 seconds
                except Exception as e:
                    logger.warning(f"Port polling error: {e}")
                    time.sleep(5.0)  # Wait longer on error

        self.monitoring = True
        self.monitor_thread = threading.Thread(target=poll_loop, daemon=True)
        self.monitor_thread.start()

    def update_ports(self):
        """Update the current port list."""
        try:
            ports = serial.tools.list_ports.comports()
            self.ports_changed.emit(ports)
        except Exception as e:
            logger.warning(f"Port update error: {e}")

    def stop(self):
        """Stop monitoring."""
        self.monitoring = False
        if self.monitor_thread:
            self.monitor_thread.join(timeout=1.0)


class ParamMonitor(QObject):
    changed = pyqtSignal()

    def __init__(self, connection: GimbalConnection, parent=None):
        super().__init__(parent)
        self.connection = connection
        self.users = {}

        self.update_timer = QTimer()

        self.connection.connection_changed.connect(self.update_running)
        self.connection.calibrating_changed.connect(self.update_running)

        self.reset_value()

    def get(self, obj):
        at1 = not self.users
        self.users[id(obj)] = None

        if at1:
            self.update_running()

    def put(self, obj):
        self.users.pop(id(obj), None)

        if not self.users:
            self.update_running()

    def update_running(self):
        if bool(self.users) and self.connection.is_connected() and not self.connection.calibrating:
            self.update_timer.start()
        else:
            self.update_timer.stop()
            self.reset_value()


class MotorsMonitor(ParamMonitor):
    def __init__(self, connection: GimbalConnection, parent=None):
        super().__init__(connection, parent)
        self.update_timer.timeout.connect(self.update_is_on)
        self.update_timer.setInterval(1000) # 1 Hz

        # Note: Might need a get/put pair for monitor users and another for actual motors users

    def reset_value(self):
        self.is_on = None

    def update_is_on(self):
        def callback(value):
            if value is not None:
                is_on = bool(value)
            else:
                is_on = None

            if self.is_on != is_on:
                self.is_on = is_on
                self.changed.emit()

        self.connection.read_param("motors-on", callback)

    def on(self):
        if not self.connection.is_connected() or self.connection.calibrating:
            return

        self.connection.send_command(cmd.CmdId.CMD_MOTORS_ON)
        self.reset_value()
        self.changed.emit()
        self.update_running() # Calls self.update_timer.start() and restarts the timer

    def off(self):
        if not self.connection.is_connected() or self.connection.calibrating:
            return

        self.connection.send_command(cmd.CmdId.CMD_MOTORS_OFF, cmd.MotorsOffRequest.build({}))
        self.reset_value()
        self.changed.emit()
        self.update_running() # Calls self.update_timer.start() and restarts the timer


class VbatMonitor(ParamMonitor):
    def __init__(self, connection: GimbalConnection, parent=None):
        super().__init__(connection, parent)
        self.update_timer.timeout.connect(self.update_value)
        self.update_timer.setInterval(1000) # 1 Hz

    def reset_value(self):
        self.value = None

    def update_value(self):
        def callback(value):
            if value is not None:
                value = float(value) * 0.001
            else:
                value = None

            if self.value != value:
                self.value = value
                self.changed.emit()

        self.connection.read_param("vbat", callback)


class ConnectionTab(QWidget):
    """Tab for managing serial connection."""

    def __init__(self, connection: GimbalConnection, parent=None):
        super().__init__(parent)
        self.connection = connection

        layout = QVBoxLayout()

        # Port selection
        port_group = QGroupBox("Serial Port")
        port_layout = QVBoxLayout()

        port_input_layout = QHBoxLayout()
        self.port_input = QLineEdit("/dev/ttyUSB0")
        self.port_input.setPlaceholderText("Enter serial port path")
        port_input_layout.addWidget(QLabel("Port:"))
        port_input_layout.addWidget(self.port_input)

        self.refresh_btn = QPushButton("Refresh")
        self.refresh_btn.setToolTip('Update list of known serial ports (normally automatic)')
        try:
            self.refresh_btn.setIcon(QIcon.fromTheme('view-refresh'))
        except:
            pass
        self.refresh_btn.clicked.connect(self.refresh_ports)
        port_input_layout.addWidget(self.refresh_btn)

        self.show_all_checkbox = QCheckBox("Show all")
        self.show_all_checkbox.setToolTip('Show system serial ports other than USB or Bluetooth')
        self.show_all_checkbox.setChecked(False)
        self.show_all_checkbox.stateChanged.connect(self.refresh_ports)
        port_input_layout.addWidget(self.show_all_checkbox)

        port_layout.addLayout(port_input_layout)

        self.port_list = QListWidget()
        self.port_list.itemDoubleClicked.connect(self.on_port_selected)
        # Set height for approximately 6 lines
        font_metrics = self.port_list.fontMetrics()
        line_height = font_metrics.lineSpacing()
        self.port_list.setMaximumHeight(line_height * 6 + 4)  # +4 for borders
        port_layout.addWidget(self.port_list)

        port_group.setLayout(port_layout)
        layout.addWidget(port_group)

        # Connection controls
        control_layout = QHBoxLayout()
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.setToolTip('Attempt connect to given port.  On success automatically\n'
                                    'loads some parameters from gimbal.')
        try:
            self.connect_btn.setIcon(QIcon.fromTheme('go-next'))
        except:
            pass
        self.connect_btn.clicked.connect(self.on_connect)
        self.disconnect_btn = QPushButton("Disconnect")
        self.disconnect_btn.clicked.connect(self.on_disconnect)
        self.disconnect_btn.setEnabled(False)

        control_layout.addWidget(self.connect_btn)
        control_layout.addWidget(self.disconnect_btn)
        control_layout.addStretch()

        layout.addLayout(control_layout)

        # Status
        self.status_label = QLabel("Disconnected")
        self.status_label.setToolTip('Latest connection status')
        layout.addWidget(self.status_label)

        layout.addStretch(1)

        # Firmware info
        firmware_group = QGroupBox("Firmware")
        firmware_layout = QVBoxLayout()
        self.firmware_version_label = QLabel()
        self.firmware_build_label = QLabel()
        firmware_layout.addWidget(self.firmware_version_label)
        firmware_layout.addWidget(self.firmware_build_label)

        firmware_btn_layout = QHBoxLayout()
        self.reboot_btn = QPushButton("Reboot")
        self.reboot_btn.clicked.connect(self.on_reboot)
        self.reboot_btn.setToolTip('Restart currently running firmware and wait for it to come back.\n'
                                   'Unsaved config changes will be lost.\n'
                                   'This sends a command to the firmware and will only work if the firmware\n'
                                   'is active and accepts commands.  The command is compatible with both\n'
                                   'OpenBGC and original SimpleBGC32 firmware.')
        self.bootloader_btn = QPushButton()
        self.bootloader_btn.clicked.connect(self.on_bootloader)
        self.bootloader_btn.setToolTip('Command the firmware to shut down and restart the MCU in bootloader mode.\n'
                                       'Closes current connection.  If not connected, connects to selected port,\n'
                                       'sends the command and disconnects.  Compatible with both OpenBGC and\n'
                                       'original SimpleBGC32 firmware.  If the MCU is already in bootloader mode\n'
                                       'the command will probably confuse the bootloader and you may need to\n'
                                       'manually power cycle the gimbal to reboot.')
        self.boot_btn = QPushButton('Connect && go back to firmware')
        self.boot_btn.clicked.connect(self.on_boot)
        self.boot_btn.setToolTip('UNIMPLEMENTED\n\n'
                                 'Command the STM32 bootloader to jump back to firmware and wait for it\n'
                                 'to boot.  If the MCU is not currently in bootloader mode, the firmware\n'
                                 'should ignore the command.')
        firmware_btn_layout.addWidget(self.reboot_btn)
        firmware_btn_layout.addWidget(self.bootloader_btn)
        firmware_btn_layout.addWidget(self.boot_btn)
        firmware_btn_layout.addStretch()
        firmware_layout.addLayout(firmware_btn_layout)
        firmware_group.setLayout(firmware_layout)
        layout.addWidget(firmware_group)

        # Config storage
        config_group = QGroupBox("Config")
        config_layout = QVBoxLayout()
        self.config_info_label = QLabel('Store and restore gimbal configuration to/from non-volatile memory.  '
                                        'Any config changes made in other tabs immediately go live but will '
                                        'not persist over power off unless written to persistent storage with '
                                        'the button below.\n'
                                        'The storage may be MCU flash or onboard EEPROM depending on firmware '
                                        'choice.  Shouldn\'t conflict with SimpleBGC32 config storage but keep '
                                        'backups anyway.')
        self.config_info_label.setWordWrap(True)
        config_layout.addWidget(self.config_info_label)
        config_btn_layout = QHBoxLayout()
        self.cfg_write_btn = QPushButton("Write to storage")
        self.cfg_write_btn.clicked.connect(self.on_cfg_write)
        self.cfg_write_btn.setToolTip('Save any changes in configuration to persistent storage in the gimbal.\n'
                                      'Do this after any changes made in the Calibration tabs or Parameter\n'
                                      'Editor tab, to preserve them after power off or reset.\n\n'
                                      'Warning: overwrites previous config, no confirmation asked.')
        self.cfg_revert_btn = QPushButton("Revert from storage")
        self.cfg_revert_btn.clicked.connect(self.on_cfg_revert)
        self.cfg_revert_btn.setToolTip('Reread last saved config from persistent storage in the gimbal reverting\n'
                                       'changes made since then.\n\n'
                                       'Warning: discards current config, no confirmation asked.')
        config_btn_layout.addWidget(self.cfg_write_btn)
        config_btn_layout.addWidget(self.cfg_revert_btn)
        config_btn_layout.addStretch()
        config_layout.addLayout(config_btn_layout)
        config_group.setLayout(config_layout)
        layout.addWidget(config_group)

        self.setLayout(layout)

        # Start port monitoring
        self.port_monitor = PortMonitor()
        self.port_monitor.ports_changed.connect(self.on_ports_changed)

        # Refresh ports on init
        self.refresh_ports()

        # Connect signals
        self.connection.connection_changed.connect(self.on_connection_changed)
        self.connection.calibrating_changed.connect(self.update_buttons)
        self.connection.error_occurred.connect(self.on_error)

        self.boot_param_timer = QTimer()
        self.boot_param_timer.setSingleShot(True)
        self.on_connection_changed(self.connection.is_connected())

    def refresh_ports(self):
        """Refresh the list of available serial ports."""
        self.port_list.clear()
        ports = serial.tools.list_ports.comports()

        show_all = self.show_all_checkbox.isChecked()

        for port in ports:
            # Filter ports if "Show all" is not checked
            if not show_all:
                # Only show USB and rfcomm ports
                if not (port.device.startswith('/dev/ttyUSB') or
                       port.device.startswith('/dev/ttyACM') or
                       'rfcomm' in port.device.lower() or
                       'usb' in port.description.lower() or
                       'bluetooth' in port.description.lower()):
                    continue

            self.port_list.addItem(f"{port.device} - {port.description}")

    def on_ports_changed(self, ports):
        """Handle port list changes from monitoring."""
        self.refresh_ports()

    def on_port_selected(self, item):
        """Handle port selection from list."""
        port_text = item.text()
        port_path = port_text.split(" - ")[0]
        self.port_input.setText(port_path)

    def on_connect(self):
        """Handle connect button click."""
        port_path = self.port_input.text().strip()
        if not port_path:
            self.status_label.setText("Error: Please enter a port path")
            return

        self.status_label.setText(f"Connecting to {port_path}...")

        if self.connection.connect(port_path):
            self.request_firmware_info()
        else:
            self.status_label.setText("Connection failed")

    def on_disconnect(self):
        """Handle disconnect button click."""
        self.boot_param_timer.stop()
        self.connection.disconnect()
        self.status_label.setText("Disconnected")

    def update_buttons(self):
        """Handle connection state change."""
        connected_fully = self.connection.is_connected()
        connected = self.connection.port is not None
        running = connected_fully and not self.connection.calibrating

        self.connect_btn.setEnabled(not connected)
        self.disconnect_btn.setEnabled(connected)

        self.reboot_btn.setEnabled(running)
        self.bootloader_btn.setEnabled(running or not connected)
        if connected:
            self.bootloader_btn.setText("Go to bootloader")
        else:
            self.bootloader_btn.setText("Connect && go to bootloader")
        self.boot_btn.setEnabled(not connected)

        self.cfg_write_btn.setEnabled(running)
        self.cfg_revert_btn.setEnabled(running)

    def on_connection_changed(self, connected):
        # This is only called after GimbalConnection has successfully read a parameter
        # confirming communication, so pretty late.
        if connected:
            self.status_label.setText(f"Connected to {self.connection.port_path}")
        else:
            self.status_label.setText("Disconnected")
            self.reset_firmware_info()
            self.boot_param_timer.stop()

        self.update_buttons()

    def on_error(self, error_msg: str):
        """Handle error from connection."""
        logger.error(f"Connection error: {error_msg}")
        self.status_label.setText(f"Error: {error_msg}")
        self.reset_firmware_info()

    def request_firmware_info(self):
        def read_cb(values):
            if self.connection.calibrating:
                # Note: in the on_boot case, the connection_changed notifictions will be
                # emitted and the listeners may queue their connection.read_param()
                # calls but they don't check connection.calibrating right after connect
                # so it doesn't matter what order these signals are emitted in.
                #
                # In the on_reboot case there's no connection_changed, although maybe
                # there should be so as to trigger geometry params reread etc. (TODO)
                self.connection.set_calibrating(False)
                if values is not None:
                    self.status_label.setText(f"Boot Ok")

            if values is None:
                self.reset_firmware_info()
                self.status_label.setText("Firmware info request error")
                return

            self.firmware_version_label.setText(f"Running version: {values[0]}")
            self.firmware_build_label.setText(f"Build date: {values[1]}")

        # TODO: set ~10s timer, kill connection attempt on timeout
        # note we're called from on_connect, on_reboot or on_boot
        self.connection.read_param(['commit-str', 'build-str'], read_cb)

    def reset_firmware_info(self):
        self.firmware_version_label.setText("Running version: -")
        self.firmware_build_label.setText("Build date: -")

    def on_reboot(self):
        def after_drain():
            payload = cmd.ResetRequest.build(None)
            self.connection.send_command(cmd.CmdId.CMD_RESET, payload)

            # Wait 4s.  Even though we request no delay in the CMD_RESET, there's an
            # at least 2s delay on boot for emergency 'q' command listen time and it
            # flushes serial data so we have to send our GET_PARAMs after that.
            self.boot_param_timer.timeout.connect(self.request_firmware_info)
            self.boot_param_timer.start(4000)

            self.status_label.setText("Reboot requested, waiting for status")

        self.connection.set_calibrating(True)
        self.connection.drain_queues(after_drain)

    def on_bootloader(self):
        def after_drain():
            payload = cmd.BootMode3Request.build(None)
            self.connection.send_command(cmd.CmdId.CMD_BOOT_MODE_3, payload)

            self.status_label.setText("Jump to bootloader requested, disconnecting")
            self.connection.disconnect()

        if not self.connection.is_connected():
            port_path = self.port_input.text().strip()
            if not port_path:
                self.status_label.setText("Error: Please enter a port path")
                return
            if not self.connection.connect(port_path):
                self.status_label.setText("Connection failed")
                return
            after_drain()
        else:
            self.connection.drain_queues(after_drain)

    def on_boot(self):
        # TODO
        pass

    def on_cfg_write(self):
        self.connection.send_raw(b'w')

    def on_cfg_revert(self):
        self.connection.send_raw(b'r')

    def closeEvent(self, event):
        """Clean up resources when widget is closed."""
        if hasattr(self, 'port_monitor'):
            self.port_monitor.stop()
        self.boot_param_timer.stop()
        self.connection.disconnect()
        super().closeEvent(event)

    def start_updates(self):
        pass
    def stop_updates(self):
        pass


class StatusTab(QWidget):
    """Tab for displaying gimbal status and 3D visualization."""

    def __init__(self, connection: GimbalConnection, geometry: GimbalGeometry, motors: MotorsMonitor, vbat: VbatMonitor, parent=None):
        super().__init__(parent)
        self.connection = connection
        self.geometry = geometry
        self.motors = motors
        self.vbat = vbat

        layout = QVBoxLayout()

        # 3D visualization
        self.view_3d = app_widgets.Gimbal3DWidget(self.geometry)
        layout.addWidget(self.view_3d)

        # Connect to geometry changes
        self.geometry.geometry_changed.connect(self.on_geometry_changed)

        # Joints display
        # TODO: try .setFlat(True) on all QGroupBoxes, see if it looks better/saves space
        joints_group = QGroupBox("Joints")
        joints_layout = QGridLayout()

        # Column headers
        joints_layout.addWidget(QLabel("Angle"), 0, 2)
        joints_layout.addWidget(QLabel("Force (torque)"), 0, 3)

        self.joint_labels = []
        self.encoder_labels = []
        self.angle_bars = []
        self.force_bars = []
        self.joint_map = [0, 1, 2]  # Default mapping without axes calibration

        # Camera angles and speeds
        self.camera_angle_labels = []
        self.camera_angle_bars = []
        self.camera_angle_inputs = []
        self.angle_send_btns = []
        self.camera_speed_labels = []
        self.camera_speed_inputs = []
        self.camera_speed_bars = []
        self.speed_send_btns = []
        self.max_vel = 60.0  # Default max deg/s
        self.prev_camera_angles = [0.0, 0.0, 0.0]
        self.prev_camera_time = time.time()

        # Create placeholder widgets that will be updated when axes calibration changes
        for i in range(3):
            # Joint name label (placeholder)
            joint_label = QLabel("")
            joints_layout.addWidget(joint_label, i + 1, 0)
            self.joint_labels.append(joint_label)

            # Angle label
            angle_label = QLabel("unknown")
            angle_label.setMinimumWidth(60)
            angle_label.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            joints_layout.addWidget(angle_label, i + 1, 1)
            joints_layout.setAlignment(angle_label, Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            self.encoder_labels.append(angle_label)

            # Angle bar widget
            angle_bar = app_widgets.AngleBarWidget()
            joints_layout.addWidget(angle_bar, i + 1, 2)
            joints_layout.setAlignment(angle_bar, Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
            self.angle_bars.append(angle_bar)

            # Force bar widget
            force_bar = app_widgets.ForceBarWidget()
            joints_layout.addWidget(force_bar, i + 1, 3)
            self.force_bars.append(force_bar)

        joints_layout.setColumnStretch(0, 0)
        joints_layout.setColumnStretch(1, 1)
        joints_layout.setColumnStretch(2, 3)
        joints_layout.setColumnStretch(3, 3)
        joints_group.setLayout(joints_layout)
        layout.addWidget(joints_group)

        self.update_joint_config()

        # Camera angles display
        self.camera_group = QGroupBox("Camera angles")
        camera_layout = QGridLayout()

        # Column headers:
        # Euler angle name label
        # Current angle label
        camera_layout.addWidget(QLabel("Angle"), 0, 2) # Angle bar
        # Target angle input
        # Send button
        # Current speed label
        camera_layout.addWidget(QLabel("Speed (rate)"), 0, 6) # Speed bar
        # Target speed input
        # Send button

        camera_angle_names = ["Roll", "Pitch", "Yaw"]

        for i in range(3):
            # Angle name label
            camera_layout.addWidget(QLabel(camera_angle_names[i]), i + 1, 0)

            # Angle label (current value)
            angle_label = QLabel("unknown")
            angle_label.setMinimumWidth(60)
            angle_label.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            camera_layout.addWidget(angle_label, i + 1, 1)
            camera_layout.setAlignment(angle_label, Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            self.camera_angle_labels.append(angle_label)

            # Pitch goes from -90 to 90, the other axes -180 to 180.  Set the widget range
            # accordingly.  The other easy solution would be to set -180 to 180 for all
            # three bar widgets, to keep the scale consistent, but mark 90-180 (-90--180)
            # inactive with .set_limits(True, 90, -90)
            maxangle = 180 if i != 1 else 89

            # Angle bar widget
            angle_input = QDoubleSpinBox()
            angle_bar = app_widgets.AngleBarWidget(-maxangle, maxangle, angle_input)
            angle_bar.set_home_angle(0)
            angle_bar.angle_changed.connect(lambda angle, axis_idx=i: self.send_control(axis_idx, True, angle))
            camera_layout.addWidget(angle_bar, i + 1, 2)
            camera_layout.setAlignment(angle_bar, Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
            self.camera_angle_bars.append(angle_bar)

            # Angle input field
            angle_input.setMaximumWidth(60)
            angle_input.setRange(-maxangle, maxangle)
            angle_input.setSingleStep(0.1)       # Note: QAbstractSpinBox.Adaptive sounds pretty cool too
            angle_input.setWrapping(i in [0, 2]) # Roll and yaw wrap around, pitch doesn't
            camera_layout.addWidget(angle_input, i + 1, 3)
            camera_layout.setAlignment(angle_input, Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
            self.camera_angle_inputs.append(angle_input)

            # Send angle button
            angle_send_btn = QPushButton("Send")
            angle_send_btn.setMaximumWidth(55)
            try:
                angle_send_btn.setIcon(QIcon.fromTheme('go-next'))
            except:
                pass
            angle_send_btn.clicked.connect(lambda checked, axis_idx=i: self.send_control_from_input(axis_idx, True))
            camera_layout.addWidget(angle_send_btn, i + 1, 4)
            camera_layout.setAlignment(angle_send_btn, Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
            self.angle_send_btns.append(angle_send_btn)

            # TODO: mark current movement target from control.target-ypr-offsets
            # TODO: mark frame relative angles too? especially for non-follow axes

            # Speed label (current value)
            speed_label = QLabel("0.0°/s")
            speed_label.setMinimumWidth(60)
            speed_label.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            camera_layout.addWidget(speed_label, i + 1, 5)
            camera_layout.setAlignment(speed_label, Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            self.camera_speed_labels.append(speed_label)

            # Speed bar widget
            speed_input = QDoubleSpinBox()
            speed_bar = app_widgets.SpeedBarWidget(self.max_vel, speed_input)
            speed_bar.speed_changed.connect(lambda speed, axis_idx=i: self.send_control(axis_idx, False, speed))
            camera_layout.addWidget(speed_bar, i + 1, 6)
            camera_layout.setAlignment(speed_bar, Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
            self.camera_speed_bars.append(speed_bar)

            # Speed input field
            speed_input.setMaximumWidth(60)
            speed_input.setRange(-self.max_vel, self.max_vel)
            speed_input.setSingleStep(0.1)
            camera_layout.addWidget(speed_input, i + 1, 7)
            camera_layout.setAlignment(speed_input, Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
            self.camera_speed_inputs.append(speed_input)

            # Send speed button
            speed_send_btn = QPushButton("Send")
            speed_send_btn.setMaximumWidth(55)
            try:
                speed_send_btn.setIcon(QIcon.fromTheme('go-next'))
            except:
                pass
            speed_send_btn.clicked.connect(lambda checked, axis_idx=i: self.send_control_from_input(axis_idx, False))
            camera_layout.addWidget(speed_send_btn, i + 1, 8)
            camera_layout.setAlignment(speed_send_btn, Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
            self.speed_send_btns.append(speed_send_btn)

        self.camera_group.setLayout(camera_layout)
        layout.addWidget(self.camera_group)

        camera_layout.setColumnStretch(1, 3)
        camera_layout.setColumnStretch(2, 0)
        camera_layout.setColumnStretch(3, 0)
        camera_layout.setColumnStretch(4, 0)
        camera_layout.setColumnStretch(5, 3)
        camera_layout.setColumnStretch(6, 0)
        camera_layout.setColumnStretch(7, 0)
        camera_layout.setColumnStretch(8, 0)

        # Battery voltage display
        vbat_group = QGroupBox("Battery Voltage")
        vbat_layout = QGridLayout()
        self.vbat_label = QLabel("unknown")
        vbat_layout.addWidget(QLabel("Voltage:"), 0, 0)
        vbat_layout.addWidget(self.vbat_label, 0, 1)
        vbat_group.setLayout(vbat_layout)
        layout.addWidget(vbat_group)

        # Motor status and controls
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
        self.motor_off_btn.clicked.connect(self.motors.off)
        self.motor_off_btn.setEnabled(False)  # Disabled until connected
        motor_layout.addWidget(self.motor_off_btn)

        motor_group.setLayout(motor_layout)
        layout.addWidget(motor_group)

        layout.addStretch()
        self.setLayout(layout)

        # Update timer (5 Hz for encoders and forces)
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_encoders_and_forces)
        self.update_timer.setInterval(200)  # 5 Hz

        self.connection.calibrating_changed.connect(self.update_buttons)
        self.motors.changed.connect(self.new_motors_status)
        self.vbat.changed.connect(self.new_vbat)

        self.update_buttons()

    def start_updates(self):
        """Start update timers."""
        self.motors.get(self)
        self.vbat.get(self)
        if self.connection.is_connected():
            self.update_timer.start()
            self.new_motors_status() # Calls update_buttons
            self.new_vbat()
            if not self.connection.calibrating:
                self.connection.read_param("config.control.max-vel", self.update_max_vel)

    def stop_updates(self):
        """Stop update timers."""
        self.update_timer.stop()
        self.motors.put(self)
        self.vbat.put(self)
        # Reset displays and disable buttons
        self.vbat_label.setText("unknown")
        self.motor_status_label.setText("unknown")
        self.motor_on_btn.setEnabled(False)
        self.motor_off_btn.setEnabled(False)

    def update_encoders_and_forces(self):
        """Update encoder angle and motor force displays."""
        if not self.connection.is_connected() or self.connection.calibrating:
            return

        if not hasattr(self, 'current_encoder_angles'):
            self.current_encoder_angles = [0.0, 0.0, 0.0]

        req_params = []

        # Read encoder angles
        def callback_encoders(values):
            if values is None:
                return

            home_angles = self.geometry.get_home_angles()

            for axis_idx in range(3):
                encoder_idx = self.geometry.axis_to_encoder[axis_idx]
                angle = float(values[encoder_idx]) * self.geometry.encoder_scale[encoder_idx]
                angle = self.geometry.angle_normalize_360(angle)
                self.current_encoder_angles[axis_idx] = angle

                if self.geometry.have_axes and self.geometry.have_home:
                    # Display relative to home angles if available
                    angle = self.geometry.angle_normalize_180(angle - home_angles[axis_idx])

                display_idx = self.joint_map[encoder_idx]
                self.encoder_labels[display_idx].setText(f"{angle:.2f}°")
                self.angle_bars[display_idx].set_angle(angle)

        req_params += [f"encoders.{i}.reading" for i in range(3)]

        # Read main AHRS orientation
        def callback_q(value):
            # value is a list of 4 floats (w, x, y, z)
            self.current_main_ahrs_q = tuple(value)
            self.view_3d.update_pose(self.current_encoder_angles, self.current_main_ahrs_q)

            # Calculate camera angles and speeds
            current_time = time.time()
            dt = current_time - getattr(self, 'prev_camera_time', current_time)
            setattr(self, 'prev_camera_time', current_time)

            camera_angles = self.geometry.get_euler(self.current_encoder_angles, self.current_main_ahrs_q)
            prev_angles = getattr(self, 'prev_camera_angles', camera_angles)
            camera_speeds = [self.geometry.angle_normalize_180(camera_angles[i] - prev_angles[i]) / dt
                             if dt > 0.001 else 0.0 for i in range(3)]
            setattr(self, 'prev_camera_angles', camera_angles)

            # Update camera angles display
            for i in range(3):
                axis_num = 2 - i
                self.camera_angle_labels[i].setText(f"{camera_angles[axis_num]:.1f}°")
                self.camera_angle_bars[i].set_angle(camera_angles[axis_num])
                self.camera_speed_labels[i].setText(f"{camera_speeds[axis_num]:.1f}°/s")
                self.camera_speed_bars[i].set_speed(camera_speeds[axis_num])


        req_params.append("main-ahrs.q")

        def callback_forces(values):
            for i in range(3):
                force = float(values[i])
                normalized_force = max(-1.0, min(1.0, force / 500.0))
                # Map through joint_map to match joint display order
                display_idx = self.joint_map[i]
                self.force_bars[display_idx].set_force(normalized_force)

        if self.motors.is_on:
            req_params += [f"motors.{i}.pid-stats.i" for i in range(3)]

        def callback(values):
            if values is None or len(values) != len(req_params):
                return

            callback_encoders(values[:3])
            callback_q(values[3])

            if len(values) > 4:
                callback_forces(values[4:])

        self.connection.read_param(req_params, callback)

    def new_vbat(self):
        """Update battery voltage display."""
        if self.vbat.value is None:
            self.vbat_label.setText("...")
            return

        self.vbat_label.setText(f"{self.vbat.value:.1f} V")
        # TODO: read config.vbat.lvco on start_updates, display this in red if vbat near or below min and not zero

    def new_motors_status(self):
        status_text = "On" if self.motors.is_on == True else ("Off" if self.motors.is_on == False else "...")
        self.motor_status_label.setText(status_text)
        self.update_buttons()

        if not self.motors.is_on:
            for i in range(3):
                self.force_bars[i].set_force(0)

    def update_max_vel(self, value):
        """Update max velocity setting."""
        if value is not None:
            # Convert from rad/s to deg/s
            self.max_vel = math.degrees(value)
            # Update all speed bars with new max velocity
            for speed_bar in self.camera_speed_bars:
                speed_bar.set_max_speed(self.max_vel)

    def on_geometry_changed(self):
        """Handle geometry changes."""
        self.update_joint_config()

        if self.geometry.have_home:
            self.camera_group.setTitle("Camera angles")
        else:
            self.camera_group.setTitle("IMU angles")

    def update_joint_config(self):
        """Update joint labels, mapping, and angle bar ranges based on current geometry status."""
        if self.geometry.have_home:
            home_angles = self.geometry.get_home_angles()
        else:
            for i in range(3):
                self.angle_bars[i].set_home_angle(None)
            home_angles = (0, 0, 0)

        if self.geometry.have_parking:
            park_angles = self.geometry.get_park_angles()
        else:
            for i in range(3):
                self.angle_bars[i].set_park_angle(None)

        if self.geometry.have_axes:
            # With axes calibration: show joint names and create reverse mapping
            joint_names = ["Inner (2)", "Middle (1)", "Outer (0)"]

            # Update angle bar ranges and home positions
            for i in range(3):
                axis_idx = 2 - i
                encoder_idx = self.geometry.axis_to_encoder[axis_idx]
                self.joint_map[encoder_idx] = i

                self.angle_bars[i].min_angle = -180
                self.angle_bars[i].max_angle = 180
                self.angle_bars[i].set_home_angle(0 if self.geometry.have_home else None)

                # Set limits
                if self.geometry.has_limits[axis_idx]:
                    lmin = self.geometry.limit_min[axis_idx] - home_angles[axis_idx] # 0 if unset
                    lmax = self.geometry.limit_max[axis_idx] - home_angles[axis_idx]
                    lmin = self.geometry.angle_normalize_180(lmin)
                    lmax = self.geometry.angle_normalize_180(lmax)
                    self.angle_bars[i].set_limits(True, lmin, lmax)
                else:
                    self.angle_bars[i].set_limits(False, None, None)

                if self.geometry.have_parking:
                    self.angle_bars[i].set_park_angle(self.geometry.angle_normalize_180(
                        park_angles[axis_idx] - home_angles[axis_idx]))

                # TODO: also read 'config.control.limit-margin' and add in degrees to both sides of the limit zone,
                # maybe mark in a different colour
        else:
            # Without axes calibration: show encoder numbers
            joint_names = [f"Encoder {i}" for i in range(3)]
            self.joint_map = [0, 1, 2]  # Direct mapping

            # Update angle bar ranges (0-360), set fixed angles
            for i in range(3):
                self.angle_bars[i].min_angle = 0
                self.angle_bars[i].max_angle = 360
                self.angle_bars[i].set_limits(False, None, None)

            if self.geometry.have_home:
                self.angle_bars[i].set_home_angle(home_angles[i])

            if self.geometry.have_parking:
                self.angle_bars[i].set_park_angle(park_angles[i])

        # Update the labels
        for i in range(3):
            self.joint_labels[i].setText(joint_names[i] + ':')

    def send_control(self, euler_idx, is_angle, value):
        """Send control command for the specified axis."""
        if not self.motors.is_on or not self.connection.is_connected():
            return

        angles = [None, None, None]
        speeds = [None, None, None]
        if is_angle:
            angles[euler_idx] = value
            self.camera_angle_bars[euler_idx].set_target_angle(value)
        else:
            speeds[euler_idx] = value
        self.connection.control(angles, speeds)

    def send_control_from_input(self, euler_idx, is_angle):
        widgets = self.camera_angle_inputs if is_angle else self.camera_speed_inputs
        self.send_control(euler_idx, is_angle, widgets[euler_idx].value())

    def update_buttons(self):
        """Update button states based on motor status."""
        connection_ok = self.connection.is_connected() and not self.connection.calibrating
        # Update button states: On only when motors are off
        self.motor_on_btn.setEnabled(self.motors.is_on == False and connection_ok)
        self.motor_off_btn.setEnabled(connection_ok)

        enabled = bool(self.motors.is_on) and connection_ok
        for btn in self.angle_send_btns + self.speed_send_btns:
            btn.setEnabled(enabled)


class PassthroughTab(QWidget):
    def __init__(self, connection: GimbalConnection, geometry: GimbalGeometry, motors: MotorsMonitor, parent=None):
        super().__init__(parent)
        self.connection = connection
        self.geometry = geometry
        self.motors = motors

        layout = QVBoxLayout()

        # Top row: info label and search button
        top_layout = QHBoxLayout()

        info_label = QLabel(
            "\"Link\" camera position to an input device or variable by continuously "
            "sending updated values to the gimbal.  The movement will still be "
            "limited by the maximum velocity set in 'config.control.max-vel' and "
            "maximum acceleration set in 'config.control.max-accel'.\n\n"
            "Inputs are scaled to roughly [0, 1] range, e.g. pointer position is "
            "divided by screen width.  Outputs are interpreted as degrees or degrees "
            "per second."
        )
        info_label.setWordWrap(True)
        top_layout.addWidget(info_label)

        right_layout = QVBoxLayout()

        self.start_btn = QPushButton('Start')
        self.start_btn.setToolTip('Start sending position updates')
        try:
            self.start_btn.setIcon(QIcon.fromTheme("go-next")) # player-start/stop icons probably too colourful
        except:
            pass
        self.start_btn.clicked.connect(self.on_start)
        right_layout.addWidget(self.start_btn)

        self.stop_btn = QPushButton('Stop')
        self.stop_btn.setToolTip('Start sending position updates')
        self.stop_btn.clicked.connect(self.stop)
        right_layout.addWidget(self.stop_btn)

        self.refresh_sources_btn = QPushButton('Refresh Sources')
        self.refresh_sources_btn.setToolTip('Update the choice of source variables for hotplug input devices')
        self.refresh_sources_btn.clicked.connect(self.update_sources)
        right_layout.addWidget(self.refresh_sources_btn)

        top_layout.addLayout(right_layout)
        layout.addLayout(top_layout)

        grid_group = QGroupBox("Rules")
        grid_layout = QGridLayout()

        # Headers
        grid_layout.addWidget(QLabel('Enable'), 0, 1)
        grid_layout.addWidget(QLabel('Source'), 0, 2)
        grid_layout.addWidget(QLabel('Offset source'), 0, 3)
        grid_layout.addWidget(QLabel('Const offset'), 0, 4)
        grid_layout.addWidget(QLabel('Scale'), 0, 5)
        grid_layout.addWidget(QLabel('Min'), 0, 6)
        grid_layout.addWidget(QLabel('Max'), 0, 7)
        grid_layout.addWidget(QLabel('Control mode'), 0, 8)

        # TODO: do we want to select the output from a combo box for extra flexibility, instead of hardcoding one per axis?
        # that will allow some outputs to be offset sources for other, in effect combining various inputs for one output

        self.angle_name = ['Roll', 'Pitch', 'Yaw']
        self.camera_to_euler = [0, 1, 2] # connection.control() uses RPY yoo
        self.controls = [{}, {}, {}]

        for angle_num in range(3):
            row = angle_num + 1

            # Angle name
            angle_label = QLabel(self.angle_name[angle_num])
            grid_layout.addWidget(angle_label, row, 0)

            # Enable
            enable_checkbox = QCheckBox()
            enable_checkbox.setToolTip(f'Enable/disable control of this angle')
            grid_layout.addWidget(enable_checkbox, row, 1)
            self.controls[angle_num]['enable'] = enable_checkbox

            # Source
            source_combo = QComboBox()
            source_combo.setToolTip('Source variable')
            source_combo.setMinimumWidth(150)
            grid_layout.addWidget(source_combo, row, 2)
            self.controls[angle_num]['source'] = source_combo

            # Offset source
            offset_src_combo = QComboBox()
            offset_src_combo.addItems(['Value at start', 'None']) # TODO: add outputs from other axes
            offset_src_combo.setToolTip('Offset variable to be subtracted from source variable value')
            offset_src_combo.setMinimumWidth(80)
            grid_layout.addWidget(offset_src_combo, row, 3)
            self.controls[angle_num]['offset-src'] = offset_src_combo

            for column, cid, tooltip, limits, default in (
                    (4, 'offset', 'Constant offset value to be added to the source variable value', (-5, 5), 0),
                    (5, 'scale', 'Multiply input after applying offsets by this', (-1000, 1000), 10),
                    (6, 'min', 'Clamp the input after scaling to this value', (-360, 360), -20),
                    (7, 'max', 'Clamp the input after scaling to this value', (-360, 360), 20),
                ):
                sb = QDoubleSpinBox()
                sb.setDecimals(1)
                if PYQT_VERSION == 6:
                   sb.setStepType(QAbstractSpinBox.StepType.AdaptiveDecimalStepType)
                else:
                    # PyQt5 doesn't have StepType enum, use setSingleStep with a small value
                    sb.setSingleStep(1)
                sb.setToolTip(tooltip)
                sb.setMaximumWidth(60)
                sb.setRange(limits[0], limits[1])
                sb.setValue(default)
                grid_layout.addWidget(sb, row, column)
                self.controls[angle_num][cid] = sb

            # Angle vs. speed control
            mode_combo = QComboBox()
            mode_combo.addItems(['Angle', 'Speed'])
            mode_combo.setToolTip('Offset variable to be subtracted from source variable')
            grid_layout.addWidget(mode_combo, row, 8)
            self.controls[angle_num]['mode'] = mode_combo

        grid_group.setLayout(grid_layout)
        layout.addWidget(grid_group)
        layout.addStretch()
        self.setLayout(layout)

        # TODO: also copy the motors-on controls from other tabs

        # Update timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.send_control)
        self.timer.setInterval(100)  # 10Hz

        # Connect to signals
        self.connection.calibrating_changed.connect(self.on_calibrating)
        self.geometry.geometry_changed.connect(self.update_buttons)
        # Or we could set a flag ourselves to prevent calibration and disable StatusTab controls
        self.motors.changed.connect(self.new_motors_status)

        # Initial state
        self.running = False
        self.update_sources()
        self.update_buttons()

    def on_start(self):
        if not self.connection.is_connected() or self.connection.calibrating:
            return

        self.running = True
        self.were_motors_off = not self.motors.is_on
        self.init_values = [None, None, None]
        self.timer.start()
        self.update_buttons()
        # TODO: set a property on self.connection to signal that calibration etc. should be blocked

        self.state = [None, None, None]
        self.params_used = {}
        self.iio_devs_used = {}

        for i in range(3):
            enable = self.controls[i]['enable']
            if not enable.isChecked(): # TODO: skip this if the axis is in use as offset source for another axis (future)
                continue

            source_num = self.controls[i]['source'].currentIndex()
            name, init, get = self.sources[source_num]
            self.state[i] = init()

        for n in self.iio_devs_used:
            self.iio_devs_used[n].start_buffering()

        if self.were_motors_off:
            self.motors.on()

    def stop(self):
        self.timer.stop()
        self.running = False
        del self.state
        del self.params_used
        for n in self.iio_devs_used:
            self.iio_devs_used[n].stop_buffering()
        del self.iio_devs_used
        self.update_buttons()

        if self.were_motors_off and self.motors.is_on != False:
            self.motors.off()

    def on_calibrating(self):
        if self.connection.calibrating and self.running:
            self.stop()
        else:
            self.update_buttons()

    def new_motors_status(self):
        if self.running and self.motors.is_on == False:
            self.stop()

    def send_control(self):
        if not self.connection.is_connected() or self.connection.calibrating or not self.running:
            return

        angles = [None, None, None]
        speeds = [None, None, None]

        def update_axis(angle_num):
            ctrls = self.controls[angle_num]
            enable = ctrls['enable']
            if not enable.isChecked(): # TODO: skip this if the axis is in use as offset source for another axis (future)
                return

            source_num = ctrls['source'].currentIndex()
            name, init, get = self.sources[source_num]
            in_value = get(self.state[angle_num])

            if in_value is None: # Send IGNORE for this axis for now, even let it time out if no values for long enough
                return
            if self.init_values[angle_num] is None:
                self.init_values[angle_num] = in_value

            offset_src_text = ctrls['offset-src'].currentText()
            if offset_src_text == 'Value at start':
                in_value -= self.init_values[angle_num]

            in_value += ctrls['offset'].value()
            in_value *= ctrls['scale'].value()
            value = min(max(in_value, ctrls['min'].value()), ctrls['max'].value())

            euler_num = self.camera_to_euler[angle_num]
            if ctrls['mode'].currentText() == 'Speed':
                speeds[euler_num] = value
            else:
                angles[euler_num] = value

        for n in self.iio_devs_used:
            self.iio_devs_used[n].get_latest()

        def update_all():
            for i in range(3):
                update_axis(i)

            self.connection.control(angles, speeds)

        # TODO: if self.params_used, request params, in callback store results and update_all()
        update_all()

    def read_float_helper(self, path):
        with open(path, 'r') as f:
            return float(f.read().strip())

    def update_sources(self):
        if self.running:
            return

        self.sources = []

        self.sources.append((
                'Mouse pointer X',
                lambda: QApplication.primaryScreen().geometry(),
                lambda state: QCursor.pos().x() / state.width(),
            ))
        self.sources.append((
                'Mouse pointer Y',
                lambda: QApplication.primaryScreen().geometry(),
                lambda state: QCursor.pos().y() / state.width(), # width, for uniform scaling
            ))
        self.sources.append((
                'Seconds (within minute)', lambda: None,
                lambda state: QTime.currentTime().second() / 60,
            ))
        self.sources.append((
                'Minutes (within hour)', lambda: None,
                lambda state: QTime.currentTime().minute() / 60,
            ))
        self.sources.append((
                'Random', lambda: None,
                lambda state: random.random()
            ))

        # Add system accelerometers/gyroscopes on Linux
        fallback = False
        acc = []
        gyro = []
        for dev in app_iio.detect_iio_devices():
            use = False
            for ch in dev.channels:
                if ch.startswith('accel_') or ch.startswith('anglvel_'):
                    if not use:
                        if not dev.check_access():
                            fallback = True
                            break
                    use = True
                    if ch.startswith('accel_'):
                        devtype = 'accelerometer'
                        axis = ch[6:]
                        l = acc
                    else:
                        devtype = 'gyroscope'
                        axis = ch[8:]
                        l = gyro

                    def iio_src_init(dev, ch):
                        if dev.name in self.iio_devs_used:
                            dev.selected_channels[ch] = 1
                        else:
                            dev.selected_channels = {ch: 1}
                            self.iio_devs_used[dev.name] = dev
                            # Could start buffering in a oneshot timer but there could be races
                        return (dev, ch)
                    l.append((
                            f'System {devtype} {dev.name} {axis.upper()}',
                            lambda dev=dev, ch=ch: iio_src_init(dev, ch),
                            lambda state: (lambda dev, ch: dev.readings.get(ch))(*state) # Will this optimize well?
                        ))
                    # Note: with the IIO buffer usage there will usually be a delay before the values
                    # start flowing in.

                    # Units are m/s^2 for the accelerometer values, rad/s for the gyro values

        if fallback:
            logger.info('Permissions not set up for IIO buffers, falling back to sysfs which may not work great')
            self.update_iio_sysfs_only()
        else:
            logger.debug(f'Found {len(acc)} system IIO accelerometers, {len(gyro)} gyroscopes')
            self.sources += acc + gyro

        # TODO: frame IMU rotation speeds using rel_q and main-ahrs.velocity-vec? that won't be very precise
        # TODO: main IMU accelerations using main-ahrs.acc-reading

        names = [name for name, init, get in self.sources]

        for i in range(3):
            combo = self.controls[i]['source']
            prev_selection = combo.currentText()
            combo.clear()
            combo.addItems(names)
            try:
                combo.setCurrentText(prev_selection)
            except:
                pass

    def add_iio_sysfs_only(self):
        # The less reliable method to access IIO devices using only /sys but without any
        # write access needed.  It seems that on Intel chips the ICH sensor bridge driver
        # will stop updating the readings if we read them out at more than 1Hz so we only
        # ever read a single value for each axis.  But it may work on other hardware.
        try:
            acnt = 0
            gcnt = 0
            for device in os.listdir('/sys/bus/iio/devices'):
                device_path = f'/sys/bus/iio/devices/{device}'
                name_file = f'{device_path}/name'
                if os.path.exists(name_file):
                    with open(name_file, 'r') as f:
                        device_name = f.read().strip()
                else:
                    device_name = 'IIO ' + device

                for axis in ['x', 'y', 'z']:
                    accel_file = f'{device_path}/in_accel_{axis}_raw'
                    if os.path.exists(accel_file):
                        acnt += 1
                        scale = self.read_float_helper(f'{device_path}/in_accel_scale')
                        self.sources.append((
                                'System accelerometer ' + device_name + ' ' + axis.upper(), lambda: None,
                                lambda state, path=accel_file, scale=scale: self.read_float_helper(path) * scale,
                            ))
                    gyro_file = f'{device_path}/in_anglvel_{axis}_raw'
                    if os.path.exists(gyro_file):
                        gcnt += 1
                        scale = self.read_float_helper(f'{device_path}/in_anglvel_scale')
                        self.sources.append((
                                'System gyroscope ' + device_name + ' ' + axis.upper(), lambda: None,
                                lambda state, path=gyro_file, scale=scale: self.read_float_helper(path) * scale,
                            ))
            logger.debug(f'Found {acnt} system IIO accelerometers, {gcnt} gyroscopes')
        except Exception as e:
            logger.debug(f'Error adding IIO sources: {e}')
            raise e

    def update_buttons(self):
        """Update button enabled states."""
        enabled = (self.connection.is_connected() and not self.connection.calibrating and
                   self.geometry.have_axes and self.geometry.have_forward)

        self.start_btn.setEnabled(enabled and not self.running)
        self.stop_btn.setEnabled(enabled and self.running)
        self.refresh_sources_btn.setEnabled(enabled and not self.running)

        for i in range(3):
            self.controls[i]['enable'].setEnabled(not self.running)
            self.controls[i]['source'].setEnabled(not self.running)

    def start_updates(self):
        self.update_buttons()
        self.motors.get(self)
        self.new_motors_status() # Calls update_buttons

    def stop_updates(self):
        if self.running:
            self.stop()

        self.update_buttons()
        self.motors.put(self)


class ParameterEditorTab(QWidget):
    """Tab for generic parameter editing."""

    def __init__(self, connection: GimbalConnection, parent=None):
        super().__init__(parent)
        self.connection = connection
        self.param_values = {}  # Cache of parameter values
        self.param_widgets = {}  # Cache of parameter widgets

        layout = QVBoxLayout()

        # Top row: info label and buttons
        top_layout = QHBoxLayout()

        info_label = QLabel(
            "Generic parameter editor. Allows viewing and modifying all gimbal parameters. "
            "Use 'Read all' to load current values from the gimbal, then modify individual "
            "parameters and use the Read/Write buttons or 'Save JSON' to export current values."
        )
        info_label.setWordWrap(True)
        top_layout.addWidget(info_label)

        right_layout = QVBoxLayout()

        self.read_all_btn = QPushButton("Read all")
        self.read_all_btn.clicked.connect(self.on_read_all)
        self.read_all_btn.setToolTip("Read all parameters from the gimbal")
        self.read_all_btn.setMinimumWidth(200)
        right_layout.addWidget(self.read_all_btn)

        self.save_json_btn = QPushButton("Save JSON")
        self.save_json_btn.clicked.connect(self.on_save_json)
        self.save_json_btn.setToolTip("Save currently loaded parameter values to JSON file")
        right_layout.addWidget(self.save_json_btn)

        top_layout.addLayout(right_layout)
        top_layout.setStretch(0, 1)
        layout.addLayout(top_layout)

        # Filter field
        filter_layout = QHBoxLayout()
        filter_layout.addWidget(QLabel("Filter:"))
        self.filter_input = QLineEdit()
        self.filter_input.setPlaceholderText("Type to filter parameter names...")
        self.filter_input.textChanged.connect(self.on_filter_changed)
        filter_layout.addWidget(self.filter_input)
        layout.addLayout(filter_layout)

        # Scrollable parameter grid
        self.scroll_area = QScrollArea()
        self.scroll_area.setWidgetResizable(True)
        self.scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)

        self.param_container = QWidget()
        self.param_layout = QGridLayout(self.param_container)
        self.param_layout.setColumnStretch(1, 3)  # Value column expands more
        self.param_layout.setColumnStretch(5, 1)  # Type column expands less

        # Headers
        headers = ["Parameter", "Value", "", "", "Size", "Type"]
        for col, header in enumerate(headers):
            label = QLabel(header)
            label.setStyleSheet("font-weight: bold;")
            self.param_layout.addWidget(label, 0, col)

        self.scroll_area.setWidget(self.param_container)
        layout.addWidget(self.scroll_area)

        self.setLayout(layout)

        # Connect signals
        self.connection.calibrating_changed.connect(self.update_buttons)

        self.update_buttons()

        self.populated = False

    def create_parameter_widgets(self):
        """Create widgets for all parameters."""
        # TODO: possibly use a tree display for the names, group parameters by path prefixes
        # TODO: a tree of dicts would also work in the saved JSON file
        # TODO: until then perhaps just sort by name or add sorting controls in the UI

        row = 1
        for param_id, pdef in param_defs.params.items():
            # Parameter name
            name_label = QLabel(pdef.name)
            name_label.setToolTip(f"Parameter ID: {param_id}")
            self.param_layout.addWidget(name_label, row, 0)
            # TODO: add read-only icon after name, if read-only

            # Value widget (determined by type)
            value_widget = self.create_toplevel_value_widget(pdef)
            value_scroll_area = app_widgets.AutoScrollArea(QSize(80, 0), QSize(300, 100))
            value_scroll_area.setWidget(value_widget)
            value_scroll_area.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
            value_scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
            value_scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
            self.param_layout.addWidget(value_scroll_area, row, 1)

            # Read button
            read_btn = QPushButton()
            read_btn.setToolTip(f"Read {pdef.name} from gimbal")
            read_btn.clicked.connect(lambda checked, pid=param_id: self.on_read_param(pid))
            try:
                read_btn.setIcon(QIcon.fromTheme("view-refresh"))
                read_btn.setText("")
            except:
                read_btn.setText("R")
            self.param_layout.addWidget(read_btn, row, 2)

            # Write button
            write_btn = QPushButton()
            write_btn.setToolTip(f"Write {pdef.name} to gimbal")
            write_btn.clicked.connect(lambda checked, pid=param_id: self.on_write_param(pid))
            try:
                write_btn.setIcon(QIcon.fromTheme("go-next"))
                write_btn.setText("")
            except:
                write_btn.setText("W")
            # Disable write button if parameter is read-only
            if pdef.flags & param_defs.ParamFlag.ro:
                write_btn.setEnabled(False)
                write_btn.setToolTip(f"{pdef.name} is read-only")
            self.param_layout.addWidget(write_btn, row, 3)

            # Size
            size_label = QLabel(str(pdef.size))
            self.param_layout.addWidget(size_label, row, 4)

            # Type
            type_label = QLabel(pdef.typ)
            type_label.setWordWrap(False)
            type_scroll_area = app_widgets.AutoScrollArea(QSize(80, 0), QSize(200, 100))
            type_scroll_area.setWidget(type_label)
            type_scroll_area.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
            type_scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
            type_scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
            self.param_layout.addWidget(type_scroll_area, row, 5)

            # Store widget references
            self.param_widgets[param_id] = {
                'name': name_label,
                'value': value_widget,
                'value_area': value_scroll_area,
                'read': read_btn,
                'write': write_btn,
                'size': size_label,
                'type': type_scroll_area,
                'row': row
            }

            row += 1

        # Empty space after all the parameters if param_layout needs to stretch
        self.param_layout.addItem(
            QSpacerItem(0, 0, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding),
            row, 0, 1, -1)

        self.populated = True

    def create_toplevel_value_widget(self, pdef):
        """Create appropriate widget based on parameter type."""
        ro = pdef.flags & param_defs.ParamFlag.ro

        try:
            construct_type = param_utils.ctype_to_construct(pdef.typ, pdef.size)
            return self.create_value_widget(construct_type, ro)
        except Exception as e:
            # Fallback to QLabel for unknown types
            label = QLabel("Unknown type")
            label.setStyleSheet("color: red;")
            return label

    def create_value_widget(self, construct_type, ro):
        # Handle arrays
        if hasattr(construct_type, 'subcon') and hasattr(construct_type, 'count'):
            # Array type - create QHBoxLayout with individual widgets
            container = QWidget()
            layout = QHBoxLayout(container)
            layout.setContentsMargins(0, 0, 0, 0)
            layout.setSpacing(1)
            layout.addWidget(QLabel('('))

            for i in range(construct_type.count):
                element_widget = self.create_value_widget(construct_type.subcon, ro)
                layout.addWidget(element_widget)

            layout.addWidget(QLabel(')'))
            return container

        # Single value
        return self.create_single_value_widget(construct_type, ro)

    def create_single_value_widget(self, construct_type, ro):
        """Create widget for a single construct type."""
        if isinstance(construct_type, construct.Enum):
            # Enum - use QComboBox
            combo = QComboBox()
            for name in construct_type.encmapping:
                combo.addItem(str(name))
            combo.setEnabled(not ro)
            return combo

        elif construct_type == construct.Flag:
            # Boolean - use QCheckBox
            checkbox = QCheckBox()
            checkbox.setEnabled(not ro)
            return checkbox

        elif hasattr(construct_type, 'fmtstr') and construct_type.fmtstr[-1] in "def":
            # Float - use QDoubleSpinBox, QLabel if read-only (TODO: should be something select/copy'able)
            if ro:
                label = QLabel()
                label.setStyleSheet("border: 1px solid #cccccc; padding: 2px;")
                return label
            spinbox = QDoubleSpinBox()
            spinbox.setRange(-1e5, 1e5)
            spinbox.setDecimals(4)
            spinbox.setEnabled(not ro)
            spinbox.setMaximumWidth(80)
            if PYQT_VERSION == 6:
                spinbox.setStepType(QAbstractSpinBox.StepType.AdaptiveDecimalStepType)
            else:
                # PyQt5 doesn't have StepType enum, use setSingleStep with a small value
                spinbox.setSingleStep(0.1)
            return spinbox

        elif isinstance(construct_type, construct.BytesInteger):
            # Integer - use QSpinBox or QDoubleSpinBox based on signedness, QLabel if read-only (TODO: should be something select/copy'able)
            if ro:
                label = QLabel()
                label.setStyleSheet("border: 1px solid #cccccc; padding: 2px;")
                return label
            if construct_type.signed:
                spinbox = QSpinBox()
                min_val = -(2 ** (construct_type.length * 8 - 1))
                max_val = 2 ** (construct_type.length * 8 - 1) - 1
            else:
                spinbox = QSpinBox()
                min_val = 0
                max_val = 2 ** (construct_type.length * 8) - 1
            spinbox.setRange(min_val, max_val)
            spinbox.setMaximumWidth(70)
            return spinbox

        elif isinstance(construct_type, construct.StringEncoded): # TODO: avoid internal types
            # String - use QLabel (read-only) (TODO: should be something select/copy'able)
            if not ro:
                info.error('Read-write string parameter')
            label = QLabel()
            label.setStyleSheet("border: 1px solid #cccccc; padding: 2px;")
            label.setWordWrap(False)
            return label

        else:
            # Unknown - use QLabel
            label = QLabel("Unsupported")
            label.setStyleSheet("color: orange;")
            return label

    def on_read_param(self, param_id):
        """Read a single parameter."""
        if not self.connection.is_connected() or self.connection.calibrating:
            return

        pdef = param_defs.params[param_id]

        def callback(value):
            if value is not None:
                self.param_values[param_id] = value
                self.update_value_widget(param_id, value)
                self.update_buttons()

        self.connection.read_param(pdef.name, callback)

    def on_write_param(self, param_id):
        """Write a single parameter."""
        if not self.connection.is_connected() or self.connection.calibrating:# or param_id not in self.param_values:
            return

        pdef = param_defs.params[param_id]
        self.param_values[param_id] = self.get_toplevel_widget_value(param_id)
        value = self.param_values[param_id]

        self.connection.write_param(pdef.name, value)
        logger.info(f"Parameter {pdef.name} written to gimbal")

        self.update_buttons()

    def on_read_all(self):
        """Read all parameters from gimbal."""
        if not self.connection.is_connected() or self.connection.calibrating:
            return

        # Get all parameter names
        all_params = list(param_defs.params.values())

        # Read in batches of 5
        batch_size = 5
        self.num_outstanding = 0
        for i in range(0, len(all_params), batch_size):
            batch = all_params[i:i + batch_size]
            param_names = [p.name for p in batch]

            def batch_callback(values, batch=batch):
                if values is not None:
                    if len(batch) == 1:
                        values = [values]
                    # Update our cached values and widgets
                    for j, pdef in enumerate(batch):
                        if j < len(values):
                            self.param_values[pdef.id] = values[j]
                            self.update_value_widget(pdef.id, values[j])
                else:
                    # TODO: if the batch read failed, add reads of each
                    # param in the batch individually to the end of the queue
                    pass

                self.num_outstanding -= 1
                if self.num_outstanding == 0:
                    del self.num_outstanding
                    self.update_buttons()

            self.connection.read_param(param_names, batch_callback)
            self.num_outstanding += 1
            if self.num_outstanding == 1:
                self.update_buttons()

    def on_save_json(self):
        """Save currently loaded parameters to JSON file."""
        if not self.param_values:
            print("No params loaded")
            return

        # Prepare data for JSON
        json_data = {}
        for param_id, value in self.param_values.items():
            pdef = param_defs.params[param_id]
            json_data[pdef.name] = value

        # Get save file path
        file_path, _ = QFileDialog.getSaveFileName(
            self, "Save Parameters to JSON", "", "JSON files (*.json);;All files (*)"
        )

        if file_path:
            try:
                with open(file_path, 'w') as f:
                    json.dump(json_data, f, indent=2)
                logger.info(f"Parameters saved to {file_path}")
            except Exception as e:
                logger.error(f"Failed to save parameters: {e}")

    def on_filter_changed(self, text):
        """Handle filter text change."""
        filter_text = text.lower()
        for param_id, widgets in self.param_widgets.items():
            pdef = param_defs.params[param_id]
            visible = filter_text in pdef.name.lower()
            for widget in widgets.values():
                if hasattr(widget, 'setVisible'):
                    widget.setVisible(visible)

    def update_value_widget(self, param_id, value):
        """Update the value widget with the given value."""
        if param_id not in self.param_widgets:
            return

        value_widget = self.param_widgets[param_id]['value']
        pdef = param_defs.params[param_id]

        try:
            construct_type = param_utils.ctype_to_construct(pdef.typ, pdef.size)
            self.set_widget_value(value_widget, construct_type, value)
        except Exception as e:
            logger.error(f"Failed to update widget for {pdef.name}: {e}")

    def set_widget_value(self, widget, construct_type, value):
        # Handle arrays
        if hasattr(construct_type, 'subcon') and hasattr(construct_type, 'count'):
            # Array - update each element widget
            layout = widget.layout()
            for i in range(construct_type.count):
                element_widget = layout.itemAt(i + 1).widget()
                if element_widget:
                    self.set_widget_value(element_widget, construct_type.subcon, value[i])
            widget.adjustSize()
        else:
            # Single value
            self.set_single_widget_value(widget, construct_type, value)

    def set_single_widget_value(self, widget, construct_type, value):
        """Set value for a single widget."""
        if isinstance(construct_type, construct.Enum):
            # Enum - set combo box
            widget.setCurrentText(str(value))

        elif construct_type == construct.Flag:
            # Boolean - set checkbox
            widget.setChecked(bool(value))

        elif hasattr(construct_type, 'fmtstr') and construct_type.fmtstr[-1] in "def":
            # Float - set spinbox
            if isinstance(widget, QDoubleSpinBox):
                widget.setValue(float(value))
            elif isinstance(widget, QLabel):
                widget.setText(f'{float(value):f}')
                widget.adjustSize()

        elif isinstance(construct_type, construct.BytesInteger):
            # Integer - set spinbox
            if isinstance(widget, QSpinBox):
                widget.setValue(int(value))
            elif isinstance(widget, QLabel):
                i = int(value)
                if i >= 0:
                    widget.setText(f'{i} - 0x{i:0{construct_type.length * 2}x}')
                else:
                    widget.setText(str(i))
                widget.adjustSize()

        elif isinstance(construct_type, construct.StringEncoded): # TODO: avoid internal types
            # String - set label
            # TODO: use some form of escaping similar to b'hello\x00\x00\x00' instead of utf8
            widget.setText(value)
            widget.adjustSize()

    def get_toplevel_widget_value(self, param_id):
        """Get value from the value widget."""
        if param_id not in self.param_widgets:
            return None

        value_widget = self.param_widgets[param_id]['value']
        pdef = param_defs.params[param_id]

        try:
            construct_type = param_utils.ctype_to_construct(pdef.typ, pdef.size)
            return self.get_widget_value(value_widget, construct_type)
        except Exception as e:
            logger.error(f"Failed to get widget value for {pdef.name}: {e}")
            return None

    def get_widget_value(self, widget, construct_type):
        # Handle arrays
        if hasattr(construct_type, 'subcon') and hasattr(construct_type, 'count'):
            # Array - collect values from each element widget
            layout = widget.layout()
            values = []
            for i in range(construct_type.count):
                element_widget = layout.itemAt(i + 1).widget()
                if element_widget:
                    val = self.get_widget_value(element_widget, construct_type.subcon)
                    if val is not None:
                        values.append(val)
            return values
        else:
            # Single value
            return self.get_single_widget_value(widget, construct_type)

    def get_single_widget_value(self, widget, construct_type):
        """Get value from a single widget."""
        if isinstance(construct_type, construct.Enum):
            # Enum - get from combo box
            return widget.currentText()

        elif construct_type == construct.Flag:
            # Boolean - get from checkbox
            return widget.isChecked()

        elif hasattr(construct_type, 'fmtstr') and construct_type.fmtstr[-1] in "def":
            # Float - get from spinbox
            if isinstance(widget, (QSpinBox, QDoubleSpinBox)):
                return widget.value()

        elif isinstance(construct_type, construct.BytesInteger):
            # Integer - get from spinbox
            if isinstance(widget, QSpinBox):
                return widget.value()

        return None

    def update_buttons(self):
        """Update button enabled states."""
        enabled = self.connection.is_connected() and not self.connection.calibrating \
            and not hasattr(self, 'num_outstanding')
        self.read_all_btn.setEnabled(enabled)
        self.save_json_btn.setEnabled(enabled and len(self.param_values))

        # Update individual parameter buttons
        for param_id, widgets in self.param_widgets.items():
            widgets['read'].setEnabled(enabled)
            pdef = param_defs.params[param_id]
            widgets['write'].setEnabled(enabled and not (pdef.flags & param_defs.ParamFlag.ro))

    def start_updates(self):
        """Start updates."""
        if not self.populated:
            self.create_parameter_widgets()
        self.update_buttons()

    def stop_updates(self):
        """Stop updates."""
        self.update_buttons()


class PopoutWindow(QMainWindow):
    closed = pyqtSignal()

    def closeEvent(self, event):
        self.closed.emit()
        super().closeEvent(event)


class MainWindow(QMainWindow):
    """Main application window."""

    def __init__(self, debug=False):
        super().__init__()
        self.setWindowTitle("OpenBGC Gimbal app")
        self.setGeometry(300, 100, 1000, 800)

        # Create connection and geometry
        self.connection = GimbalConnection(debug)
        self.geometry = GimbalGeometry(self.connection)
        self.motors = MotorsMonitor(self.connection)
        self.vbat = VbatMonitor(self.connection)

        is_enabled_normal = lambda: self.connection.is_connected() and not self.connection.calibrating
        is_enabled_ready = lambda: self.connection.is_connected() and not self.connection.calibrating and \
            self.geometry.have_axes and self.geometry.have_forward
        is_enabled_calib = lambda: self.connection.is_connected()
        is_enabled_calib_with_axes = lambda: is_enabled_calib() and self.geometry.have_axes

        self.tabs = {
            'status': (0, 'Status', StatusTab(self.connection, self.geometry, self.motors, self.vbat), QTreeWidgetItem(), is_enabled_normal, True),
            'passthrough': (1, 'Control Passthrough', PassthroughTab(self.connection, self.geometry, self.motors), QTreeWidgetItem(), is_enabled_ready, True),
            'calibration': (2, 'Calibration', calib.CalibrationTab(self.connection, self.geometry, self), QTreeWidgetItem(), is_enabled_calib, False),
            'calib-axes': (3, 'Axis calibration', calib.AxisCalibrationTab(self.connection, self.geometry), QTreeWidgetItem(), is_enabled_calib, True),
            'calib-motor': (4, 'Motor geometry calibration', calib.MotorGeometryCalibrationTab(self.connection), QTreeWidgetItem(), is_enabled_calib, True),
            'calib-pid': (5, 'Motor PID editor', calib.MotorPidEditorTab(self.connection, self.motors), QTreeWidgetItem(), is_enabled_calib, True),
            'calib-limit': (6, 'Joint limits', calib.JointLimitsTab(self.connection, self.geometry, self.motors), QTreeWidgetItem(), is_enabled_calib_with_axes, True),
            'calib-vbat': (6, 'Battery voltage', calib.VbatSenseTab(self.connection, self.vbat), QTreeWidgetItem(), is_enabled_normal, True),
            'params': (7, 'Parameter Editor', ParameterEditorTab(self.connection), QTreeWidgetItem(), is_enabled_normal, True),
            'connection': (8, 'Connection & config', ConnectionTab(self.connection), QTreeWidgetItem(), lambda: True, False),
        }

        self.popped_out_tabs = {}

        # Set global style sheet for disabled buttons
        self.setStyleSheet("""
            QPushButton:hover {
                color: #888888;
            }
            QPushButton:disabled {
                color: #aaaaaa;
            }
        """)

        # Create splitter for main content and bottom strip
        splitter = self.create_splitter()
        self.setCentralWidget(splitter)

        # Connect signals
        self.connection.connection_changed.connect(self.on_connection_changed)
        self.connection.calibrating_changed.connect(self.update_tabs)
        self.geometry.geometry_changed.connect(self.update_tabs)

        # Initially disable non-connection tabs
        self.on_connection_changed(False)

        # Select status tab initially
        self.switch_to_tab('status')

    def create_splitter(self):
        """Create the main splitter with tab content and bottom strip."""
        splitter = QSplitter(Qt.Orientation.Vertical)
        splitter.setStyleSheet("QSplitter::handle { background-color: #bbbbbb; }")

        # Top part: main content
        main_widget = QWidget()
        main_layout = QHBoxLayout()

        # Left sidebar with tab tree
        self.tab_selector = QTreeWidget()
        self.tab_selector.setHeaderHidden(True)
        self.tab_selector.setColumnCount(1)
        self.tab_selector.setColumnWidth(0, 175)
        self.tab_selector.setFixedWidth(180)
        self.tab_selector.setUniformRowHeights(False)  # Allow variable row heights for word wrapping
        self.tab_selector.itemSelectionChanged.connect(self.update_tabs)
        #self.tab_selector.setIndentation(12)

        items = [(index, tab_id, item) for tab_id, (index, _, _, item, _, _) in self.tabs.items()]
        for _, tab_id, item in sorted(items): # Sorting tuples is lexicographic so by first element
            if tab_id.startswith('calib-'):
                parent_item.addChild(item)
            else:
                self.tab_selector.addTopLevelItem(item)
                parent_item = item

        # Ensure disabled items are visually distinct
        self.tab_selector.setStyleSheet("""
            QTreeWidget::item { color: #000000; }
            QTreeWidget::item:hover { background-color: #f5f5f5; }
            QTreeWidget::item:disabled { color: #aaaaaa; }
            QTreeWidget::item QLabel { color: #000000; }
            QTreeWidget::item QLabel:disabled { color: #aaaaaa; }
        """)
        #   QTreeWidget::item QLabel:hover { color: #888888; }
        main_layout.addWidget(self.tab_selector)

        # Right side: stacked widget for tab content
        self.stacked_widget = QStackedWidget()

        main_layout.addWidget(self.stacked_widget)
        main_widget.setLayout(main_layout)

        # Set up tabs and tab_selector items
        for tab_id in self.tabs:
            _, text, tab, item, _, pop_out_ok = self.tabs[tab_id]

            self.stacked_widget.addWidget(tab)

            item.widget = app_widgets.TabItemWidget(text, item, self.tab_selector,
                (lambda checked=False, tid=tab_id: self.pop_out_tab(tid)) if pop_out_ok else None)
            self.tab_selector.setItemWidget(item, 0, item.widget)

        # Expand calibration by default
        self.tab_selector.expandAll()

        # Bottom part: status strip
        bottom_widget = QWidget()
        bottom_layout = QHBoxLayout()
        #bottom_layout.setContentsMargins(5, 5, 5, 5)

        # Port
        self.port_status = QLabel()
        self.port_status.setToolTip('Currently connected serial port path, see Connection tab')
        bottom_layout.addWidget(self.port_status, 0)  # Fixed size

        # Busy indicator
        self.busy_indicator = app_widgets.BusyIndicator(self.connection)
        bottom_layout.addWidget(self.busy_indicator, 0)  # Fixed size

        # Console widget
        self.console = app_widgets.ConsoleWidget(self.connection)
        self.console.setToolTip('Log of human-readable messages from the firmware -- all messages other than serial API protocol frames')
        bottom_layout.addWidget(self.console, 1)  # Takes remaining space

        bottom_widget.setLayout(bottom_layout)

        # Add to splitter
        splitter.addWidget(main_widget)
        splitter.addWidget(bottom_widget)

        # Set initial sizes (main content gets most space)
        splitter.setSizes([750, 50])
        splitter.setCollapsible(0, False)  # Main content not collapsible
        splitter.setCollapsible(1, True)   # Bottom strip collapsible

        self.update_tabs()
        return splitter

    def pop_out_tab(self, tab_id):
        _, text, tab, item, is_enabled, pop_out_ok = self.tabs[tab_id]
        if not pop_out_ok:# or not is_enabled():
            return

        if tab_id in self.popped_out_tabs:
            window = self.popped_out_tabs[tab_id]
            window.show()
            window.raise_()
            window.activateWindow()
            return

        if self.tab_selector.currentItem() == item:
            self.switch_to_tab('connection')

        item.setHidden(True)
        self.stacked_widget.removeWidget(tab)

        window = PopoutWindow(self)
        window.setWindowTitle('OpenBGC - ' + text)
        window.setAttribute(Qt.WidgetAttribute.WA_DeleteOnClose, True)
        tab.setParent(window)
        window.setCentralWidget(tab)
        tab.setVisible(True)

        window.closed.connect(lambda tid=tab_id: self.on_popout_closed(tid))
        self.popped_out_tabs[tab_id] = window

        window.show()
        self.update_tabs()

    def on_popout_closed(self, tab_id):
        window = self.popped_out_tabs.pop(tab_id, None)
        if not window:
            return

        _, _, tab, item, _, _ = self.tabs[tab_id]
        tab.setParent(self.stacked_widget)
        self.stacked_widget.addWidget(tab)
        item.setHidden(False)

        self.update_tabs()

    def update_tabs(self):
        current_item = self.tab_selector.currentItem()
        for tab_id, (index, _, tab, item, is_enabled, _) in self.tabs.items():
            enabled = is_enabled()
            updating = enabled and (item == current_item or tab_id in self.popped_out_tabs)

            if enabled:
                item.setFlags(item.flags() | Qt.ItemFlag.ItemIsEnabled)
                if item == current_item:
                    self.stacked_widget.setCurrentWidget(tab)
            else:
                item.setFlags(item.flags() & ~Qt.ItemFlag.ItemIsEnabled)
            item.widget.label.setEnabled(enabled)

            if updating and not hasattr(tab, 'updating'):
                tab.start_updates()
                tab.updating = True
            elif not updating and hasattr(tab, 'updating'):
                del tab.updating
                tab.stop_updates()

    def switch_to_tab(self, tab_id):
        """Switch to a specific tab by id."""
        _, _, _, item, is_enabled, _ = self.tabs[tab_id]
        if not is_enabled():
            return

        if tab_id in self.popped_out_tabs:
            window = self.popped_out_tabs[tab_id]
            window.show()
            window.raise_()
            window.activateWindow()
            return

        self.tab_selector.setCurrentItem(item)

    def on_connection_changed(self, connected: bool):
        """Handle connection state change."""
        self.update_tabs()

        # Enable/disable tabs based on connection
        if connected:
            # Switch to status tab
            self.switch_to_tab('status')

            self.port_status.setText(self.connection.port_path)
        else:
            # Switch to connection tab
            self.switch_to_tab('connection')

            self.port_status.setText("Offline")


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description="OpenBGC Gimbal Control GUI")
    parser.add_argument('-d', '--debug', action='store_true', help='Enable debug mode with verbose logging')
    args = parser.parse_args()

    # Set up logging
    if args.debug:
        logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        logger.setLevel(logging.DEBUG)
        logger.info(f"Debug mode enabled. app_widgets.HAS_OPENGL: {app_widgets.HAS_OPENGL}")
    else:
        logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
        logger.setLevel(logging.INFO)

    app = QApplication(sys.argv)
    window = MainWindow(debug=args.debug)
    window.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
