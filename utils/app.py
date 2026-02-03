#! /usr/bin/python3
# vim: set ts=4 sw=4 sts=4 et :
"""
PyQt GUI companion app for gimbals running OpenBGC
"""

import sys
import math
import struct
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
        QScrollArea, QSizePolicy, QSpacerItem, QToolButton, QStyle,
        QHeaderView,
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
        QScrollArea, QSizePolicy, QSpacerItem, QToolButton, QStyle,
        QHeaderView,
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
            self.connection_changed.emit(True)
            return True
        except Exception as e:
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
        self.connection_changed.emit(False)
        self._check_busy_state()

    def is_connected(self) -> bool:
        """Check if connected."""
        return self.port is not None and self.port.is_open

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
        if not self.is_connected():
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
                        callback(None)
                        return
                    pdef = by_name[param_name]
                pdefs.append(pdef)
                param_ids.append(pdef.id)

            # Build request
            out_payload = cmd_obgc.GetParamRequest.build(dict(param_ids=param_ids))
            out_frame = fr.FrameV1.build(dict(
                hdr=dict(cmd_id=int(cmd_obgc.CmdId.CMD_OBGC), size=len(out_payload)),
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
            callback(None)

    def _handle_param_response(self, param_ids, data, callback, pdefs, param_names):
        """Handle a parameter response for one or more parameters."""
        if data is None:
            callback(None)
            return

        total_size = sum([pdef.size for pdef in pdefs])
        if len(data) != total_size:
            error_msg = f"Reading params {str(param_names)} size mismatch: expected {total_size}, got {len(data)}"
            logger.error(error_msg)
            callback(None)
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
            callback(None)
            return

        if len(values) == 1:
            callback(values[0])
        else:
            callback(values)

    def send_command(self, cmd_id, payload=None):
        """Send a command to the gimbal."""
        if not self.is_connected():
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
        if not self.is_connected():
            raise Exception("Not connected")
        self.port.write(data)

    def control(self, angles=None, speeds=None): # degrees and degrees/s respectively
        modes = [
            cmd.ControlMode.MODE_ANGLE if angles is not None and angles[i] is not None else
            cmd.ControlMode.MODE_SPEED if speeds is not None and speeds[i] is not None else
            cmd.ControlMode.MODE_IGNORE for i in range(3)
        ]
        targets = [
            {'angle': int(angles[i] / sbgc_unit.degree_factor)} if modes[i] == cmd.ControlMode.MODE_ANGLE else
            {'speed': int(speeds[i] / sbgc_unit.degree_per_sec_factor)} if modes[i] == cmd.ControlMode.MODE_SPEED else
            None for i in range(3)
        ]
        self.send_command(cmd.CmdId.CMD_CONTROL, cmd.ControlRequest.build(dict(control_mode=modes, target=targets)))

    def write_param(self, param_name, value):
        by_name = {pdef.name: pdef for pdef in param_defs.params.values()}
        pdef = by_name[param_name]
        param_type_cls = param_utils.ctype_to_construct(pdef.typ, pdef.size)
        value_bytes = param_type_cls.build(value)

        # Build request
        out_payload = cmd_obgc.SetParamRequest.build(dict(param_id=pdef.id, value=value_bytes))
        out_frame = fr.FrameV1.build(dict(
            hdr=dict(cmd_id=int(cmd_obgc.CmdId.CMD_OBGC)),
            pld=out_payload
        ))

        # TODO: perhaps drain the read queue before doing this, for now let callers do it if needed
        self.port.write(out_frame)


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
        # Set height for approximately 5 lines
        #font_metrics = self.port_list.fontMetrics()
        #line_height = font_metrics.lineSpacing()
        #self.port_list.setFixedHeight(line_height * 5 + 4)  # +4 for borders
        port_layout.addWidget(self.port_list)

        port_group.setLayout(port_layout)
        layout.addWidget(port_group)

        # Connection controls
        control_layout = QHBoxLayout()
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.setToolTip('Attempt connect to given port, directly loads some data from gimbal')
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

        layout.addStretch()
        self.setLayout(layout)

        # Start port monitoring
        self.port_monitor = PortMonitor()
        self.port_monitor.ports_changed.connect(self.on_ports_changed)

        # Refresh ports on init
        self.refresh_ports()

        # Connect signals
        self.connection.connection_changed.connect(self.on_connection_changed)
        self.connection.error_occurred.connect(self.on_error)

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

        if self.connection.connect(port_path):
            self.status_label.setText(f"Connected to {port_path}")
        else:
            self.status_label.setText("Connection failed")

    def on_disconnect(self):
        """Handle disconnect button click."""
        self.connection.disconnect()
        self.status_label.setText("Disconnected")

    def on_connection_changed(self, connected: bool):
        """Handle connection state change."""
        self.connect_btn.setEnabled(not connected)
        self.disconnect_btn.setEnabled(connected)
        if connected:
            self.status_label.setText(f"Connected to {self.connection.port_path}")
        else:
            self.status_label.setText("Disconnected")

    def on_error(self, error_msg: str):
        """Handle error from connection."""
        logger.error(f"Connection error: {error_msg}")
        self.status_label.setText(f"Error: {error_msg}")

    def closeEvent(self, event):
        """Clean up resources when widget is closed."""
        if hasattr(self, 'port_monitor'):
            self.port_monitor.stop()
        super().closeEvent(event)

    def start_updates(self):
        pass
    def stop_updates(self):
        pass


class StatusTab(QWidget):
    """Tab for displaying gimbal status and 3D visualization."""

    def __init__(self, connection: GimbalConnection, geometry: GimbalGeometry, parent=None):
        super().__init__(parent)
        self.connection = connection
        self.geometry = geometry

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
        self.motors_on = None
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
        self.motor_on_btn.clicked.connect(self.on_motor_on)
        self.motor_on_btn.setEnabled(False)  # Disabled until connected
        motor_layout.addWidget(self.motor_on_btn)

        self.motor_off_btn = QPushButton("Off")
        self.motor_off_btn.clicked.connect(self.on_motor_off)
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

        # Update timer (1 Hz for battery voltage and motor status)
        self.vbat_timer = QTimer()
        self.vbat_timer.timeout.connect(self.update_vbat)
        self.vbat_timer.timeout.connect(self.update_motors_status)
        self.vbat_timer.setInterval(1000)  # 1 Hz

        self.connection.calibrating_changed.connect(self.update_buttons)

        self.update_buttons()

    def start_updates(self):
        """Start update timers."""
        if self.connection.is_connected():
            self.update_timer.start()
            self.vbat_timer.start()
            self.update_buttons()
            if not self.connection.calibrating:
                self.connection.read_param("config.control.max-vel", self.update_max_vel)

    def stop_updates(self):
        """Stop update timers."""
        self.update_timer.stop()
        self.vbat_timer.stop()
        # Reset displays and disable buttons
        self.vbat_label.setText("unknown")
        self.motor_status_label.setText("unknown")
        self.motors_on = None
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

        if self.motors_on:
            req_params += [f"motors.{i}.pid-stats.i" for i in range(3)]

        def callback(values):
            if values is None or len(values) != len(req_params):
                return

            callback_encoders(values[:3])
            callback_q(values[3])

            if len(values) > 4:
                callback_forces(values[4:])

        self.connection.read_param(req_params, callback)

    def update_vbat(self):
        """Update battery voltage display."""
        if not self.connection.is_connected() or self.connection.calibrating:
            return

        def callback(value):
            if value is not None:
                # vbat is in millivolts, convert to volts with 1 decimal place
                voltage = float(value) / 1000.0
                self.vbat_label.setText(f"{voltage:.1f} V")
                # TODO: add and read vbat_min occasionally, display this in red if vbat near or below vbat_min

        self.connection.read_param("vbat", callback)

    def new_motors_status(self):
        status_text = "On" if self.motors_on == True else ("Off" if self.motors_on == False else "...")
        self.motor_status_label.setText(status_text)
        self.update_buttons()

        if not self.motors_on:
            for i in range(3):
                self.force_bars[i].set_force(0)

    def update_motors_status(self):
        """Update motors status display."""
        if not self.connection.is_connected() or self.connection.calibrating:
            return

        def callback(value):
            if value is not None:
                motors_on = bool(value)
            else:
                motors_on = None

            if self.motors_on != motors_on:
                self.motors_on = motors_on
                self.new_motors_status()

        self.connection.read_param("motors-on", callback)

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
        if not self.motors_on or not self.connection.is_connected():
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
        self.motor_on_btn.setEnabled(not self.motors_on and connection_ok)
        self.motor_off_btn.setEnabled(connection_ok)

        enabled = bool(self.motors_on) and connection_ok
        for btn in self.angle_send_btns + self.speed_send_btns:
            btn.setEnabled(enabled)

    def on_motor_on(self):
        """Handle motor on button click."""
        self.connection.send_command(cmd.CmdId.CMD_MOTORS_ON)
        self.motors_on = None
        self.new_motors_status()

    def on_motor_off(self):
        """Handle motor off button click."""
        self.connection.send_command(cmd.CmdId.CMD_MOTORS_OFF, cmd.MotorsOffRequest.build({}))
        self.motors_on = None
        self.new_motors_status()

        # Read calibration state
        def callback_have_axes(value):
            if value is None:
                return


class PassthroughTab(QWidget):
    def __init__(self, connection: GimbalConnection, geometry: GimbalGeometry, parent=None):
        super().__init__(parent)
        self.connection = connection
        self.geometry = geometry

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
        # Or we could set a flag ourselves to prevent calibration and disable StatusTab controls

        # Initial state
        self.running = False
        self.update_sources()
        self.update_buttons()

    def on_start(self):
        if not self.connection.is_connected() or self.connection.calibrating:
            return

        self.running = True
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

    def stop(self):
        self.timer.stop()
        self.running = False
        del self.state
        del self.params_used
        for n in self.iio_devs_used:
            self.iio_devs_used[n].stop_buffering()
        del self.iio_devs_used
        self.update_buttons()

    def on_calibrating(self):
        if self.connection.calibrating and self.running:
            self.stop()
        else:
            self.update_buttons()

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
            logger.info(f'Permissions not set up for IIO buffers, falling back to sysfs which may not work great')
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
        enabled = (self.connection.is_connected() and
                   not self.connection.calibrating) # TODO: and motors_on

        self.start_btn.setEnabled(enabled and not self.running)
        self.stop_btn.setEnabled(enabled and self.running)
        self.refresh_sources_btn.setEnabled(enabled and not self.running)

        for i in range(3):
            self.controls[i]['enable'].setEnabled(not self.running)
            self.controls[i]['source'].setEnabled(not self.running)

    def start_updates(self):
        # TODO: load motors-on
        self.update_buttons()

    def stop_updates(self):
        if self.running:
            self.stop()

        self.update_buttons()


class CalibrationTab(QWidget):
    """Tab for calibration status and controls."""

    def __init__(self, connection: GimbalConnection, geometry: GimbalGeometry, main_window=None):
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
            "voltage": ("TODO: Set the battery/supply voltage sensing scale for correct\n" +
                "low voltage alarm and motor force compensation.  Optional."),
        }

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
             [f'config.motor-calib.{i}.bldc-with-encoder.pole-pairs' for i in range(3)],
             lambda: False), # TODO
            ('pid', "Motor PIDs",
             [("Go to tab", self.on_go_to_pid_tab, None)],
             [f'config.motor-pid.{i}.kp' for i in range(3)],
             lambda: True), # No way to know whether configured (?)
            ('limit', "Joint limits",
             [("Go to tab", self.on_go_to_limit_tab, None)],
             [f'config.axes.has-limits.{i}' for i in range(3)],
             lambda: False), # TODO
            ('parking', "Parking position",
             [("Set", self.on_set_parking, 'go-next')],
             [],
             lambda: self.geometry.have_parking),
            ('voltage', "Voltage sense",
             [],
             [],
             lambda: False), # TODO
        ]

        self.calibration_checkboxes = {}
        self.calibration_labels = {}
        self.calibration_btns = {}

        row = 0
        for step_id, step_name, buttons, params, check in self.calibration_steps:
            # TODO: read the params

            # Checkbox in first column
            checkbox = QCheckBox()
            checkbox.setEnabled(False) # Read-only
            checkbox.setToolTip(descriptions[step_id])
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
        for step_id, step_name, buttons, params, check in self.calibration_steps:
            self.calibration_checkboxes[step_id].setChecked(check())

    def start_updates(self):
        """Update calibration status display."""
        if not self.connection.is_connected():
            return

        # Update checkboxes based on current geometry state
        self.update_checkboxes()
        self.update_buttons()

    def stop_updates(self):
        pass

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
        self.main_window.switch_to_tab('calib_axes')

    def on_set_home(self):
        self.connection.send_raw(b'k')
        logger.info(f"New home position set")
        self.geometry.update()

    def on_set_forward(self):
        self.connection.send_raw(b'K')
        logger.info(f"New forward vector set")
        self.geometry.update()

    def on_go_to_motor_tab(self):
        self.main_window.switch_to_tab('calib_motor')

    def on_go_to_pid_tab(self):
        self.main_window.switch_to_tab('calib_pid')

    def on_go_to_limit_tab(self):
        self.main_window.switch_to_tab('calib_limit')

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

        self.read_param([f"encoders.{i}.reading" for i in range(3)], encoders_read_cb)


class AxisCalibrationTab(QWidget):
    """Tab for axes and encoders geometry calibration."""

    def __init__(self, connection: GimbalConnection, geometry, parent=None):
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
        pass


class MotorGeometryCalibrationTab(QWidget):
    """Tab for motor geometry calibration."""

    def __init__(self, connection: GimbalConnection, parent=None):
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

    def __init__(self, connection: GimbalConnection, parent=None):
        super().__init__(parent)
        self.connection = connection
        self.motors_on = None  # Unknown initially

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
        # TODO: remap if have_axes?
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
        self.motor_on_btn.clicked.connect(self.on_motor_on)
        self.motor_on_btn.setEnabled(False)  # Disabled until connected
        motor_layout.addWidget(self.motor_on_btn)

        self.motor_off_btn = QPushButton("Off")
        self.motor_off_btn.clicked.connect(self.on_motor_off)
        self.motor_off_btn.setEnabled(False)  # Disabled until connected
        motor_layout.addWidget(self.motor_off_btn)

        motor_group.setLayout(motor_layout)
        layout.addWidget(motor_group)

        layout.addStretch()
        self.setLayout(layout)

        # Motor status update timer (1 Hz)
        self.motor_status_timer = QTimer()
        self.motor_status_timer.timeout.connect(self.update_motor_status)
        self.motor_status_timer.setInterval(1000)  # 1 Hz

        # Connect to connection changes
        self.connection.calibrating_changed.connect(self.update_buttons)

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

    def on_motor_on(self):
        """Handle motor on button click."""
        self.connection.send_command(cmd.CmdId.CMD_MOTORS_ON)
        self.motors_on = None
        self.new_motors_status()

    def on_motor_off(self):
        """Handle motor off button click."""
        self.connection.send_command(cmd.CmdId.CMD_MOTORS_OFF, cmd.MotorsOffRequest.build({}))
        self.motors_on = None
        self.new_motors_status()
        # Reset use-motors to all disabled
        for i in range(3):
            self.connection.write_param(f'use-motor.{i}', False)

    def update_motor_status(self):
        """Update motors status display."""
        if not self.connection.is_connected() or self.connection.calibrating:
            return

        def callback(value):
            if value is not None:
                motors_on = bool(value)
            else:
                motors_on = None

            if self.motors_on != motors_on:
                self.motors_on = motors_on
                self.new_motors_status()

        self.connection.read_param("motors-on", callback)

    def new_motors_status(self):
        """Update motor status display."""
        status_text = "ON" if self.motors_on == True else ("Off" if self.motors_on == False else "unknown")
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
        self.motor_on_btn.setEnabled(not self.motors_on and connection_ok)
        self.motor_off_btn.setEnabled(connection_ok)
        # PID parameter buttons
        for btn in list(self.send_btns.values()) + self.test_btns:
            btn.setEnabled(connection_ok)

    def start_updates(self):
        """Start updates."""
        self.load_values()
        self.motor_status_timer.start()
        self.update_buttons()

    def stop_updates(self):
        """Stop updates."""
        self.motor_status_timer.stop()
        self.motor_status_label.setText("unknown")
        self.motor_status_label.setStyleSheet("")
        self.motors_on = None
        self.update_buttons()


class JointLimitsTab(QWidget):
    """Tab for joint limits calibration and configuration."""

    PATH_LIMIT_SEARCH = 'control_data_s::CONTROL_PATH_LIMIT_SEARCH' # 4 works too

    def __init__(self, connection: GimbalConnection, geometry: GimbalGeometry, parent=None):
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
        if not self.connection.is_connected():
            return

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
            if value is not None and str(value) != self.PATH_LIMIT_SEARCH:
                logger.info("Limit search done")
                self.search_timer.stop()
                self.connection.send_command(cmd.CmdId.CMD_MOTORS_OFF, cmd.MotorsOffRequest.build({}))
                self.connection.set_calibrating(False)
                self.geometry.update()

        self.connection.read_param('control.path-type', callback)

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
        min_val = self.min_inputs[axis_num].value()
        max_val = self.max_inputs[axis_num].value()

        self.connection.write_param(f'config.axes.has-limits.{axis_num}', enabled)
        self.connection.write_param(f'config.axes.limit-min.{axis_num}', min_val)
        self.connection.write_param(f'config.axes.limit-max.{axis_num}', max_val)

        self.geometry.update()
        logger.info("New limit values sent to gimbal for axis " + str(axis_num))

    def on_write_buffer(self):
        """Write buffer width setting."""
        if not self.connection.is_connected() or self.connection.calibrating:
            return

        self.connection.write_param('config.control.limit-margin', self.buffer_input.value())
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
                self.buffer_input.setValue(float(value))

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

        self.popped_out_tabs = {}

        # Set global style sheet for disabled buttons
        self.setStyleSheet("""
            QPushButton:disabled {
                color: #aaaaaa;
            }
            QPushButton:disabled:hover {
            }
            QPushButton:disabled:pressed {
            }
        """)

        # Create connection and geometry
        self.connection = GimbalConnection(debug)
        self.geometry = GimbalGeometry(self.connection)

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
        self.tab_selector.setColumnCount(2)
        self.tab_selector.setMinimumWidth(150)
        self.tab_selector.setMaximumWidth(150)
        self.tab_selector.setWordWrap(True)
        self.tab_selector.setUniformRowHeights(False)  # Allow variable row heights for word wrapping
        self.tab_selector.setTextElideMode(Qt.TextElideMode.ElideNone)
        self.tab_selector.itemSelectionChanged.connect(self.update_tabs)

        # Create top-level items
        status_item = QTreeWidgetItem(self.tab_selector, ["Status"])
        passthrough_item = QTreeWidgetItem(self.tab_selector, ["Control Passthrough"])
        calibration_item = QTreeWidgetItem(self.tab_selector, ["Calibration"])
        param_editor_item = QTreeWidgetItem(self.tab_selector, ["Parameter Editor"])
        connection_item = QTreeWidgetItem(self.tab_selector, ["Connection"])

        # Add sub-items for calibration
        axis_calibration_item = QTreeWidgetItem(calibration_item, ["Axis calibration"])
        calibration_item.addChild(axis_calibration_item)
        motor_calibration_item = QTreeWidgetItem(calibration_item, ["Motor geometry calibration"])
        calibration_item.addChild(motor_calibration_item)
        pid_calibration_item = QTreeWidgetItem(calibration_item, ["Motor PID editor"])
        calibration_item.addChild(pid_calibration_item)
        limits_calibration_item = QTreeWidgetItem(calibration_item, ["Joint limits"])
        calibration_item.addChild(limits_calibration_item)

        # Expand calibration by default
        calibration_item.setExpanded(True)
        #calibration_item.setChildIndicatorPolicy(QTreeWidgetItem.ChildIndicatorPolicy.DontShowIndicator) <-- hides the children, why?

        # Set column width to allow word wrapping
        self.tab_selector.setColumnWidth(0, 200)
        self.tab_selector.setColumnWidth(1, 18)
        header = self.tab_selector.header()
        header.setMinimumSectionSize(18)
        header.setStretchLastSection(False)
        header.setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.ResizeMode.ResizeToContents)

        # Ensure disabled items are visually distinct
        self.tab_selector.setStyleSheet("""
            QTreeWidget::item:disabled { color: #aaaaaa; }
            QTreeWidget::item:enabled { color: #000000; }
        """)
        main_layout.addWidget(self.tab_selector)

        # Right side: stacked widget for tab content
        self.stacked_widget = QStackedWidget()

        is_enabled_normal = lambda: self.connection.is_connected() and not self.connection.calibrating
        is_enabled_calib = lambda: self.connection.is_connected()
        is_enabled_calib_with_axes = lambda: is_enabled_calib() and self.geometry.have_axes

        self.tabs = {
            'status': (0, status_item, StatusTab(self.connection, self.geometry), is_enabled_normal, True),
            'passthrough': (1, passthrough_item, PassthroughTab(self.connection, self.geometry), is_enabled_normal, True),
            'calibration': (2, calibration_item, CalibrationTab(self.connection, self.geometry, self), is_enabled_calib, False),
            'calib_axes': (3, axis_calibration_item, AxisCalibrationTab(self.connection, self.geometry), is_enabled_calib, True),
            'calib_motor': (4, motor_calibration_item, MotorGeometryCalibrationTab(self.connection), is_enabled_calib, True),
            'calib_pid': (5, pid_calibration_item, MotorPidEditorTab(self.connection), is_enabled_calib, True),
            'calib_limit': (6, limits_calibration_item, JointLimitsTab(self.connection, self.geometry), is_enabled_calib_with_axes, True),
            'params': (7, param_editor_item, ParameterEditorTab(self.connection), is_enabled_normal, True),
            'connection': (8, connection_item, ConnectionTab(self.connection), lambda: True, False),
        }

        for tab_id, (_, item, _, _, pop_out_ok) in self.tabs.items():
            if pop_out_ok:
                self.add_popout_button(tab_id, item)

        for _, _, tab, _, _ in sorted(self.tabs.values()): # Sorting tuples is lexicographic so by first element
            self.stacked_widget.addWidget(tab)

        main_layout.addWidget(self.stacked_widget)
        main_widget.setLayout(main_layout)

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

    def add_popout_button(self, tab_id, item):
        button = QToolButton()
        button.setIcon(self.style().standardIcon(QStyle.StandardPixmap.SP_TitleBarNormalButton))
        button.setAutoRaise(True)
        button.setIconSize(QSize(12, 12))
        button.setFixedSize(QSize(16, 16))
        button.setStyleSheet("QToolButton { padding: 0px; }")
        button.setToolTip("Open this tab in a separate window.\nClose the window to see it back here.")
        button.clicked.connect(lambda checked=False, tid=tab_id: self.pop_out_tab(tid))
        self.tab_selector.setItemWidget(item, 1, button)

    def pop_out_tab(self, tab_id):
        _, item, tab, is_enabled, pop_out_ok = self.tabs[tab_id]
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
        window.setWindowTitle('OpenBGC - ' + item.text(0))
        window.setAttribute(Qt.WidgetAttribute.WA_DeleteOnClose, True)
        tab.setParent(None)
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

        index, item, tab, _, _ = self.tabs[tab_id]
        tab.setParent(self.stacked_widget)
        self.stacked_widget.insertWidget(index, tab)
        item.setHidden(False)

        self.update_tabs()

    def update_tabs(self):
        current_item = self.tab_selector.currentItem()
        for tab_id, (_, item, tab, is_enabled, _) in self.tabs.items():
            enabled = is_enabled()
            updating = enabled and (item == current_item or tab_id in self.popped_out_tabs)

            if enabled:
                item.setFlags(item.flags() | Qt.ItemFlag.ItemIsEnabled)
                if item == current_item:
                    self.stacked_widget.setCurrentWidget(tab)
            else:
                item.setFlags(item.flags() & ~Qt.ItemFlag.ItemIsEnabled)

            if updating and not hasattr(tab, 'updating'):
                tab.start_updates()
                tab.updating = True
            elif not updating and hasattr(tab, 'updating'):
                del tab.updating
                tab.stop_updates()

    def switch_to_tab(self, tab_id):
        """Switch to a specific tab by id."""
        _, item, _, is_enabled, _ = self.tabs[tab_id]
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
