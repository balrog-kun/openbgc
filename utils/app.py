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
from typing import Optional, Callable, Dict, Any

try:
    from PyQt6.QtWidgets import (
        QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
        QLabel, QLineEdit, QPushButton, QListWidget, QStackedWidget,
        QTreeWidget, QTreeWidgetItem, QGroupBox, QGridLayout, QCheckBox,
        QSplitter, QPlainTextEdit, QToolTip
    )
    from PyQt6.QtCore import (
        Qt, QTimer, pyqtSignal, QObject, QThread, QSocketNotifier, QRect
    )
    from PyQt6.QtOpenGLWidgets import QOpenGLWidget
    from PyQt6.QtGui import QColor, QPainter, QTextOption, QCursor, QPen
    PYQT_VERSION = 6
except ImportError:
    from PyQt5.QtWidgets import (
        QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
        QLabel, QLineEdit, QPushButton, QListWidget, QStackedWidget,
        QTreeWidget, QTreeWidgetItem, QGroupBox, QGridLayout, QCheckBox,
        QSplitter, QPlainTextEdit, QToolTip
    )
    from PyQt5.QtCore import (
        Qt, QTimer, pyqtSignal, QObject, QThread, QSocketNotifier, QRect
    )
    from PyQt5.QtOpenGL import QOpenGLWidget
    from PyQt5.QtGui import QColor, QPainter, QTextOption, QCursor, QPen
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

try:
    from OpenGL import GL
    from OpenGL.GL import (
        glClear, glClearColor, glBegin, glEnd, glVertex3f, glNormal3f,
        glColor3f, glLineWidth, GL_COLOR_BUFFER_BIT, GL_DEPTH_BUFFER_BIT,
        GL_DEPTH_TEST, GL_LIGHTING, GL_LIGHT0, GL_POSITION, GL_AMBIENT,
        GL_DIFFUSE, GL_SPECULAR, GL_SHININESS, GL_COLOR_MATERIAL,
        GL_LINE_LOOP, GL_LINES, GL_TRIANGLES, GL_TRIANGLE_STRIP,
        GL_TRIANGLE_FAN, GL_FRONT, GL_NORMALIZE, GL_FLAT, GL_SMOOTH,
        GL_LINE_SMOOTH, GL_POLYGON_SMOOTH, GL_BLEND, GL_SRC_ALPHA_SATURATE,
        GL_ONE, glBlendFunc, glShadeModel,
        glMatrixMode, glLoadIdentity, GL_PROJECTION, GL_MODELVIEW,
        glRotatef, glMultMatrixf, glTranslatef, glPushMatrix, glPopMatrix,
        glViewport, glEnable, glDisable, glOrtho, glLightfv, glMaterialf,
        glMaterialfv
    )
    from OpenGL.GLU import gluLookAt
    HAS_OPENGL = True
except ImportError:
    HAS_OPENGL = False
    print("Warning: PyOpenGL not available, 3D visualization will be limited")


import serial
import serial.tools.list_ports

import sbgcserialapi.cmd as cmd
import sbgcserialapi.cmd_obgc as cmd_obgc
import sbgcserialapi.frame as fr

import param_defs
import param_utils

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
        self.connection.read_param("config.control.home-angles", self.update_home_angles)
        self.connection.read_param("config.control.park-angles", self.update_park_angles)
        self.connection.read_param("config.control.forward-vec", self.update_forward_vec)

        # Request limit information
        has_limits_params = [f"config.axes.has-limits.{i}" for i in range(3)]
        self.connection.read_param(has_limits_params, self.update_has_limits)

        limit_min_params = [f"config.axes.limit-min.{i}" for i in range(3)]
        self.connection.read_param(limit_min_params, self.update_limit_min)

        limit_max_params = [f"config.axes.limit-max.{i}" for i in range(3)]
        self.connection.read_param(limit_max_params, self.update_limit_max_and_emit)

    def _reset_to_defaults(self):
        self.have_axes = False
        self.have_home = False
        self.have_parking = False
        self.have_forward = False
        self.axes = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        self.axis_to_encoder = [0, 1, 2]
        self.encoder_scale = [1.0, 1.0, 1.0]
        self.main_imu_mount_q = [1.0, 0.0, 0.0, 0.0]
        self.home_angles = [0.0, 0.0, 0.0]
        self.home_q = [1.0, 0.0, 0.0, 0.0]
        self.park_angles = [0.0, 0.0, 0.0]
        self.forward_vec = [1.0, 0.0]
        self.has_limits = [False, False, False]
        self.limit_min = [0.0, 0.0, 0.0]
        self.limit_max = [0.0, 0.0, 0.0]

    def reset_to_defaults(self):
        """Reset all values to defaults."""
        self._reset_to_defaults()
        self.geometry_changed.emit()

    def update(self):
        """Manual update trigger."""
        if self.connection.is_connected():
            self.request_geometry()

    # Update methods for individual parameters
    def update_have_axes(self, value):
        if value is not None:
            self.have_axes = bool(value)

    def update_have_home(self, value):
        if value is not None:
            self.have_home = bool(value)

    def update_have_parking(self, value):
        if value is not None:
            self.have_parking = bool(value)

    def update_have_forward(self, value):
        if value is not None:
            self.have_forward = bool(value)

    def update_axes(self, values):
        if values and len(values) == 3:
            self.axes = [list(v) if isinstance(v, list) else [1.0, 0.0, 0.0] for v in values]

    def update_axis_to_encoder(self, values):
        if values and len(values) == 3:
            self.axis_to_encoder = [int(v) for v in values]

    def update_encoder_scale(self, values):
        if values and len(values) == 3:
            self.encoder_scale = [float(v) for v in values]

    def update_main_imu_mount_q(self, value):
        if value is not None:
            self.main_imu_mount_q = list(value) if isinstance(value, list) else [1.0, 0.0, 0.0, 0.0]

    def update_home_angles(self, value):
        if value is not None:
            self.home_angles = [math.degrees(a) for a in value] if isinstance(value, list) else [0.0, 0.0, 0.0]

    def update_home_q(self, value):
        if value is not None:
            self.home_q = list(value) if isinstance(value, list) else [1.0, 0.0, 0.0, 0.0]

    def update_park_angles(self, value):
        if value is not None:
            self.park_angles = [math.degrees(a) for a in value] if isinstance(value, list) else [0.0, 0.0, 0.0]

    def update_forward_vec(self, value):
        if value is not None:
            self.forward_vec = list(value) if isinstance(value, list) else [1.0, 0.0]

    def update_has_limits(self, values):
        if values and len(values) == 3:
            self.has_limits = [bool(v) for v in values]

    def update_limit_min(self, values):
        if values and len(values) == 3:
            self.limit_min = [math.degrees(v) for v in values]

    def update_limit_max_and_emit(self, values):
        if values and len(values) == 3:
            self.limit_max = [math.degrees(v) for v in values]
            self.geometry_changed.emit()


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

    def axis_angle_to_quaternion(self, axis, angle):
        """Convert axis-angle to quaternion."""
        half_angle = angle * 0.5
        s = math.sin(half_angle)
        c = math.cos(half_angle)
        return [c, *self.vector_mult_scalar(axis, s)]

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
    busy_state_changed = pyqtSignal(bool)    # busy: True when queues busy for 100ms, False when empty

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
            logger.error(error_msg)
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
            logger.error(error_msg)
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
            error_msg = f"Remote error: {in_payload}"
            logger.error(f"CMD_ERROR received: {in_payload}, payload_size={len(frame.pld.data)}")
            if self.debug:
                logger.debug(f"CMD_ERROR details: {in_payload}")
            self.error_occurred.emit(error_msg)
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

    def _handle_param_response(self, param_ids, data: bytes, callback, pdefs, param_names):
        """Handle a parameter response for one or more parameters."""
        try:
            # Parse the array of parameter values
            values = []
            offset = 0
            for i, pdef in enumerate(pdefs):
                param_size = pdef.size
                if offset + param_size > len(data):
                    error_msg = f"Param {param_names[i]} data truncated: expected {param_size} bytes, got {len(data) - offset}"
                    logger.error(error_msg)
                    callback(None)
                    return

                param_data = data[offset:offset + param_size]
                param_type_cls = param_utils.ctype_to_construct(pdef.typ, pdef.size)
                val = param_type_cls.parse(param_data)
                val = list_convert(val)
                values.append(val)
                offset += param_size

            if len(values) == 1:
                callback(values[0])
            else:
                callback(values)
        except Exception as e:
            error_msg = f"Parse param response error: {e}"
            logger.error(error_msg)
            if self.debug:
                logger.debug(f"Param response parse error, data={data.hex()}")
            self.error_occurred.emit(error_msg)
            callback(None)

    def send_command(self, cmd_id, payload=None):
        """Send a command to the gimbal."""
        if not self.is_connected():
            return False

        try:
            if payload is None:
                payload = b''
            out_frame = fr.FrameV1.build(dict(
                hdr=dict(cmd_id=cmd_id, size=len(payload)),
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
        busy = len(self.pending_requests) > 0

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

    def _on_busy_timeout(self):
        """Called after 100ms of continuous busy state."""
        self._is_busy_signaled = True
        self.busy_state_changed.emit(True)


def list_convert(val):
    """Recursively convert construct lists to Python lists."""
    if isinstance(val, list):
        return [list_convert(elem) for elem in val]
    return val


class Gimbal3DWidget(QOpenGLWidget):
    """3D visualization widget for gimbal status."""

    def __init__(self, geometry: GimbalGeometry, parent=None):
        super().__init__(parent)
        self.geometry = geometry
        if not HAS_OPENGL:
            logger.warning("OpenGL not available, 3D visualization will be limited")
            self.setMinimumSize(400, 300)
            return

        self.setMinimumSize(400, 300)

        self.shading = True  # Enable/disable simple OpenGL shading

        # Connect to geometry changes
        self.geometry.geometry_changed.connect(self.update)

        # Current pose
        self.encoder_angles = [0.0, 0.0, 0.0]  # degrees
        self.main_ahrs_q = [1.0, 0.0, 0.0, 0.0]  # quaternion (w, x, y, z)

        # Camera
        self.rotation_x = 0.0
        self.rotation_y = 0.0
        self.zoom = 0.4

        # Dimensions (in meters)
        self.camera_width = 0.12 # 12cm
        self.camera_height = 0.08 # 8cm
        self.camera_depth = 0.04 # 4cm
        self.arm_width = 0.03
        self.arm_thickness = 0.01
        # Outer arm length normal to joint axis
        self.arm_radius = 0.15
        # Motor height, also how much shorter middle/inner arms are than the immediately previous arm
        self.motor_height = 0.05
        self.motor_radius = 0.03


    def update_pose(self, encoder_angles, main_ahrs_q):
        """Update current pose (encoder angles in degrees, quaternion as list)."""
        self.encoder_angles = encoder_angles
        self.main_ahrs_q = main_ahrs_q
        self.update()

    def calculate_camera_orientation(self):
        """Calculate final camera orientation quaternion."""
        q = self.main_ahrs_q

        if self.geometry.have_home:
            # Multiply by conjugate of home-q
            home_q_conj = self.geometry.quaternion_conjugate(self.geometry.home_q)
            q = self.geometry.quaternion_multiply(home_q_conj, q)

        return q

    def calculate_arm_positions(self):
        """Calculate gimbal arm joint positions working backwards from camera."""
        if not self.geometry.have_axes:
            return []

        # Start with IMU orientation
        q = self.main_ahrs_q

        # Apply inverse of main-imu-mount-q to get inner arm
        q_inner = self.geometry.quaternion_multiply(q, self.geometry.quaternion_conjugate(self.geometry.main_imu_mount_q))

        # Work backwards through joints
        orientations = [q_inner]
        axes_world = []

        for num in [2, 1, 0]:
            angle_rad = math.radians(self.encoder_angles[self.geometry.axis_to_encoder[num]]) * self.geometry.encoder_scale[num]
            axis = self.geometry.axes[num]

            # Find arm orientation
            q_joint_inv = self.geometry.axis_angle_to_quaternion(axis, angle_rad)
            q_current = self.geometry.quaternion_multiply(orientations[-1], q_joint_inv)
            orientations.append(q_current)
            axes_world.append(self.geometry.vector_rotate(axis, q_current)) # Nop for the inner joint axis

        orientations = orientations[::-1]
        axes_world = axes_world[::-1]

        # Calculate positions and orientations of motors and arm segments
        motors = []
        segments = []
        camera_pos = (0, 0, 0)
        camera_q = self.calculate_camera_orientation()
        camera_size = self.geometry.vector_rotate((self.camera_width, self.camera_depth, self.camera_height), camera_q)
        camera_pos_dist = -0.5 * self.geometry.vector_dot(camera_size, axes_world[2])
        # Leave a 1cm space between camera box and the top of the inner joint motor
        # maybe should just use self.arm_radius - self.motor_height * 2
        motor_top = self.geometry.vector_sum(camera_pos, self.geometry.vector_mult_scalar(axes_world[2], -(camera_pos_dist + 0.01)))
        motor_base = self.geometry.vector_sum(motor_top, self.geometry.vector_mult_scalar(axes_world[2], -self.motor_height))

        for num in [2, 1]:
            motor_x = self.geometry.vector_cross(axes_world[num], axes_world[num - 1]) # Normal to the plane containing the axes
            motor_y = self.geometry.vector_cross(axes_world[num], motor_x)             # Perpendicular to current axis towards previous joint
            motor_z = axes_world[num]
            motors.append((motor_top, motor_base, self.geometry.matrix_t((motor_x, motor_y, motor_z))))

            sin_angle = self.geometry.vector_norm(motor_x)
            arm_length = (self.arm_radius - self.motor_height * (num - 1)) * sin_angle
            arm_elbow_pos = self.geometry.vector_sum(motor_base, self.geometry.vector_mult_scalar(motor_y, arm_length))
            segments.append((motor_base, arm_elbow_pos, arm_length, self.geometry.matrix_t((motor_x, motor_y, motor_z))))

            seg_x = motor_x                                                # Normal to the plane containing the axes
            seg_y = self.geometry.vector_cross(axes_world[num - 1], seg_x) # Perpendicular to current axis away from next joint
            seg_z = axes_world[num - 1]
            seg_start = self.geometry.vector_sum(arm_elbow_pos, self.geometry.vector_mult_scalar(seg_y, arm_length))
            segments.append((arm_elbow_pos, seg_start, arm_length, self.geometry.matrix_t((seg_x, seg_y, seg_z))))
            motor_top = self.geometry.vector_sum(seg_start, self.geometry.vector_mult_scalar(seg_z, self.arm_thickness))
            motor_base = self.geometry.vector_sum(motor_top, self.geometry.vector_mult_scalar(seg_z, -self.motor_height))

        motors.append((motor_top, motor_base, (seg_x, seg_y, seg_z)))

        return orientations, axes_world, motors[::-1], segments[::-1]

    def gl_rotate_m(self, m):
        # Column-major matrix for OpenGL
        glMultMatrixf([m[0][0], m[1][0], m[2][0], 0,
                       m[0][1], m[1][1], m[2][1], 0,
                       m[0][2], m[1][2], m[2][2], 0,
                       0, 0, 0, 1])

    def gl_rotate_q(self, q):
        self.gl_rotate_m(self.geometry.quaternion_to_matrix(q))

    def initializeGL(self):
        """Initialize OpenGL."""
        if not HAS_OPENGL:
            return
        glClearColor(0.6, 0.6, 0.6, 1.0)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LINE_SMOOTH)
        glEnable(GL_BLEND)

        if self.shading:
            glEnable(GL_LIGHTING)
            glEnable(GL_LIGHT0)
            # Position light at camera position
            glLightfv(GL_LIGHT0, GL_POSITION, (0, -1, 0.3, 1))
            glLightfv(GL_LIGHT0, GL_AMBIENT, (0.3, 0.3, 0.3, 1))
            glLightfv(GL_LIGHT0, GL_DIFFUSE, (1.0, 1.0, 1.0, 1))
            glLightfv(GL_LIGHT0, GL_SPECULAR, (1.0, 1.0, 1.0, 1))
            glEnable(GL_COLOR_MATERIAL)
            glEnable(GL_NORMALIZE)
        else:
            glDisable(GL_LIGHTING)

    def resizeGL(self, w, h):
        """Handle resize."""
        if not HAS_OPENGL:
            return
        glViewport(0, 0, w, h)

    def paintGL(self):
        """Render the 3D scene."""
        if not HAS_OPENGL:
            return

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        # Set up projection
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        # Orthographic projection for no perspective effect
        # X right, Z up, Y front. Let's make the view symmetric around origin for now.
        # The values here define the clipping planes (left, right, bottom, top, near, far)
        aspect_ratio = self.width() / max(self.height(), 1)
        view_size = 0.5 * self.zoom # Adjust view_size based on zoom for effective scaling
        glOrtho(-view_size * aspect_ratio, view_size * aspect_ratio, -view_size, view_size, 0.0, 2.0)

        # Set up modelview
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

        # Eye position: (0, -1, 0.30) - looking along positive Y
        # Center: (0, 0, 0.30)
        # Up vector: (0, 0, 1) - Z is up
        gluLookAt(0, -1, 0.30, 0, 0, 0.30, 0, 0, 1)

        # Draw coordinate axes
        glLineWidth(2.0)
        glBegin(GL_LINES)
        glColor3f(1.0, 0.0, 0.0)  # X - red
        glVertex3f(0, 0, 0)
        glVertex3f(0.10, 0, 0)
        glColor3f(0.0, 1.0, 0.0)  # Y - green
        glVertex3f(0, 0, 0)
        glVertex3f(0, 0.10, 0)
        glColor3f(0.0, 0.0, 1.0)  # Z - blue
        glVertex3f(0, 0, 0)
        glVertex3f(0, 0, 0.10)
        glEnd()

        if self.shading:
            # Set material properties for shaded objects
            glMaterialfv(GL_FRONT, GL_SPECULAR, (0.6, 0.6, 0.6, 1.0))  # White specular for gloss on any color
            glMaterialf(GL_FRONT, GL_SHININESS, 100.0)  # Higher for sharper highlights
            glShadeModel(GL_SMOOTH)

        glTranslatef(0, 0, 0.30)

        if not self.geometry.have_axes:
            # Just show camera model
            self.gl_rotate_q(self.calculate_camera_orientation())
            self._draw_camera()
        else:
            orientations, axes, motors, segments = self.calculate_arm_positions()

            # Draw camera
            # Note: all positions based on the camera centered at 0, 0, 0
            # If user selects handle centering, use glTranslatef(*-motors[0][1])?
            glPushMatrix()
            self.gl_rotate_q(self.calculate_camera_orientation())
            self._draw_camera()
            glPopMatrix()

            # Draw motors in black
            glColor3f(0.1, 0.1, 0.1)
            for top, base, orientation in motors:
                glPushMatrix()
                glTranslatef(*base)
                self.gl_rotate_m(orientation)
                self._draw_cylinder(self.motor_radius, self.motor_height)
                glPopMatrix()

            # Draw arm segments in black
            glColor3f(0.1, 0.1, 0.1)
            for end, start, length, orientation in segments:
                glPushMatrix()
                center = self.geometry.vector_mult_scalar(self.geometry.vector_sum(end, start), 0.5)
                glTranslatef(*center)
                self.gl_rotate_m(orientation)
                self._draw_trapezoid(self.arm_width, self.arm_width, self.arm_thickness, length)
                glPopMatrix()

            # Draw rotation axes in faint red
            glLineWidth(1.0)
            glBegin(GL_LINES)
            glNormal3f(0, -1, 0)
            glColor3f(0.4, 0.1, 0.1)
            for i in range(len(axes)):
                axis = axes[i]
                pos = motors[i][1]
                glVertex3f(*self.geometry.vector_sum(pos, self.geometry.vector_mult_scalar(axis, -0.30)))
                glVertex3f(*self.geometry.vector_sum(pos, self.geometry.vector_mult_scalar(axis, 0.10 + self.motor_height)))
            glEnd()

    def _draw_cylinder(self, radius, height, slices=16):
        """Draw a cylinder along the Z-axis."""
        # Base and top circles
        glBegin(GL_TRIANGLES)
        for i in range(slices):
            angle = 2 * math.pi * i / slices
            next_angle = 2 * math.pi * (i + 1) / slices
            x1, y1 = radius * math.cos(angle), radius * math.sin(angle)
            x2, y2 = radius * math.cos(next_angle), radius * math.sin(next_angle)

            # Base
            glNormal3f(0, 0, -1)
            glVertex3f(0.0, 0.0, 0.0)
            glVertex3f(x1, y1, 0.0)
            glVertex3f(x2, y2, 0.0)

            # Top
            glNormal3f(0, 0, 1)
            glVertex3f(0.0, 0.0, height)
            glVertex3f(x2, y2, height)
            glVertex3f(x1, y1, height)
        glEnd()

        # Side walls
        glBegin(GL_TRIANGLE_STRIP)
        for i in range(slices + 1):
            angle = 2 * math.pi * i / slices
            x, y = radius * math.cos(angle), radius * math.sin(angle)
            glNormal3f(x, y, 0)
            glVertex3f(x, y, 0)
            glVertex3f(x, y, height)
        glEnd()

    def _draw_trapezoid(self, bottom_width, top_width, height, depth):
        """Draw a trapezoidal prism along the Z-axis, centered."""
        half_bw = bottom_width / 2.0
        half_tw = top_width / 2.0
        half_d = depth / 2.0

        # Vertices for the bottom rectangle
        b1 = [-half_bw, -half_d, 0.0]
        b2 = [half_bw, -half_d, 0.0]
        b3 = [half_bw, half_d, 0.0]
        b4 = [-half_bw, half_d, 0.0]

        # Vertices for the top rectangle (at height)
        t1 = [-half_tw, -half_d, height]
        t2 = [half_tw, -half_d, height]
        t3 = [half_tw, half_d, height]
        t4 = [-half_tw, half_d, height]

        def quad(a, b, c, d):
            glNormal3f(*self.geometry.vector_cross(self.geometry.vector_sub(c, b), self.geometry.vector_sub(b, a)))
            glVertex3f(*a)
            glVertex3f(*b)
            glVertex3f(*c)
            glVertex3f(*a)
            glVertex3f(*c)
            glVertex3f(*d)

        glBegin(GL_TRIANGLES)
        quad(b1, b2, b3, b4) # Bottom
        quad(t4, t3, t2, t1) # Top
        quad(b1, t1, t2, b2) # Front
        quad(b4, b3, t3, t4) # Back
        quad(b1, b4, t4, t1) # Left
        quad(b2, t2, t3, b3) # Right
        glEnd()

    def _draw_circle(self, radius, slices=16):
        """Draw a circle in the XY plane."""
        glBegin(GL_TRIANGLE_FAN)
        glNormal3f(0, 0, -1)
        glVertex3f(0.0, 0.0, 0.0) # Center of the circle
        for i in range(slices + 1):
            angle = 2 * math.pi * i / slices
            x, y = radius * math.cos(angle), radius * math.sin(angle)
            glNormal3f(-x, -y, 0)
            glVertex3f(x, y, 0.0)
        glEnd()

    def _draw_camera(self):
        """Draw a simple camera model at position with orientation."""
        # Set camera color to black
        glColor3f(0.1, 0.1, 0.1)  # Dark gray / almost black

        # Camera body (main box)
        # Dimensions: width, height, depth
        # X-axis is width, Y-axis is depth, Z-axis is height
        # Trapezoid parameters: bottom_width, top_width, height, depth
        # Here, `height` parameter of _draw_trapezoid corresponds to camera's Z (height)
        # `bottom_width` and `top_width` correspond to camera's X (width)
        # `depth` parameter of _draw_trapezoid corresponds to camera's Y (depth)
        self._draw_trapezoid(self.camera_width, self.camera_width, self.camera_height, self.camera_depth)

        # Screen on the back
        screen_width = self.camera_width * 0.6
        screen_height = self.camera_height * 0.7
        screen_depth = 0.001
        screen_center_width = -(self.camera_width - screen_width) * 0.5 + self.camera_width * 0.1;
        screen_center_depth = -(self.camera_depth / 2 + screen_depth / 2)
        screen_bottom_height = (self.camera_height - screen_height) / 2
        glPushMatrix()
        glTranslatef(screen_center_width, screen_center_depth, screen_bottom_height);
        glColor3f(0.6, 0.6, 0.6)  # Gray screen
        self._draw_trapezoid(screen_width, screen_width, screen_height, screen_depth)
        glPopMatrix()

        # Lens (cylinder on the front)
        lens_radius = self.camera_height * 0.9 / 2
        lens_length = self.camera_height * 0.7
        lens_base_center_width = screen_center_width
        lens_base_center_height = self.camera_height / 2
        lens_base_center_depth = self.camera_depth / 2
        glPushMatrix()
        glTranslatef(lens_base_center_width, lens_base_center_depth, lens_base_center_height);
        glRotatef(90.0, -1.0, 0.0, 0.0) # Rotate to align with camera Y-axis (which is depth, here represented by Z of cylinder)
        glColor3f(0.1, 0.1, 0.1)  # Black lens body
        self._draw_cylinder(lens_radius, lens_length)

        # Lens glass
        lens_glass_radius = lens_radius - 0.008 # 8mm less
        glTranslatef(0.0, 0.0, lens_length + 0.001) # Move to the front of the cylinder
        glColor3f(0.5, 0.5, 0.6)
        self._draw_circle(lens_glass_radius)

        glPopMatrix()

        # Viewfinder/Flash (trapezoid on top)
        vf_bottom_width = self.camera_width * 0.4
        vf_top_width = self.camera_width * 0.3
        vf_height = self.camera_height * 0.2
        vf_depth = self.camera_depth + 0.02
        vf_center_width = screen_center_width
        vf_center_depth = 0
        vf_bottom_height = self.camera_height
        glPushMatrix()
        glTranslatef(vf_center_width, vf_center_depth, vf_bottom_height) # On top of the camera body
        glColor3f(0.1, 0.1, 0.1)  # Black viewfinder
        self._draw_trapezoid(vf_bottom_width, vf_top_width, vf_height, vf_depth)

        # Viewfinder screen (tiny rectangle before it)
        vf_screen_width = vf_top_width * 0.4
        vf_screen_height = vf_height * 0.5
        vf_screen_depth = 0.001
        glTranslatef(0.0, -vf_depth / 2 - vf_screen_depth / 2, (vf_height - vf_screen_height) / 2)
        glColor3f(0.6, 0.6, 0.6)  # Gray screen
        self._draw_trapezoid(vf_screen_width, vf_screen_width, vf_screen_height, vf_screen_depth)

        glPopMatrix()


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
        self.port_edit = QLineEdit("/dev/ttyUSB0")
        self.port_edit.setPlaceholderText("Enter serial port path")
        port_input_layout.addWidget(QLabel("Port:"))
        port_input_layout.addWidget(self.port_edit)

        self.refresh_btn = QPushButton("Refresh")
        self.refresh_btn.clicked.connect(self.refresh_ports)
        port_input_layout.addWidget(self.refresh_btn)

        self.show_all_cb = QCheckBox("Show all")
        self.show_all_cb.setChecked(False)
        self.show_all_cb.stateChanged.connect(self.refresh_ports)
        port_input_layout.addWidget(self.show_all_cb)

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

        show_all = self.show_all_cb.isChecked()

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
        self.port_edit.setText(port_path)

    def on_connect(self):
        """Handle connect button click."""
        port_path = self.port_edit.text().strip()
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


class ConsoleWidget(QPlainTextEdit): # QTextEdit if we need colours, highlights, etc.
    """Custom console widget for displaying text log with terminal-like behavior."""

    def __init__(self, text_log_ref, parent=None):
        super().__init__(parent)
        self.text_log_ref = text_log_ref
        self.setMinimumHeight(25)
        self.setReadOnly(True)
        self.setMaximumBlockCount(1000)
        self.setUndoRedoEnabled(False)
        self.setOverwriteMode(True)
        self.setWordWrapMode(QTextOption.WrapMode.WrapAtWordBoundaryOrAnywhere)
        self.newline = False
        #self.setStyleSheet("QPlainTextEdit { background-color: #bbbbbb; color: #eeeeee; }")

        # Use monospace font
        font = self.font()
        font.setFamily("Monospace")
        font.setPointSize(int(font.pointSize() * 0.8))
        font.setStyleHint(font.StyleHint.TypeWriter)
        self.setFont(font)

        text_log_ref.text_logged.connect(self.on_text_logged)

    def on_text_logged(self, text):
        """Update the display with new log content."""
        cursor = self.textCursor()
        for char in text:
            if char == '\n':
                # Avoid keeping an empty line at the end of the console
                self.newline = True
            elif char == '\r':
                cursor.movePosition(cursor.MoveOperation.StartOfLine)
            else:
                if self.newline:
                    cursor.movePosition(cursor.MoveOperation.End)
                    cursor.insertText('\n')
                    self.newline = False
                cursor.insertText(char)
        self.ensureCursorVisible()
        return
        '''
        line = self.lines[-1]
        line_len = len(line)
        # This could be a lot smarter by procesing chunks until next control
        # char but we may need some more logic in the future so leave it at
        # this
        for char in text:
            if char == '\n':
                self.lines[-1] = line
                line = ''
                line_len = 0
                self.lines.append(line) # TODO: This only adds a literal '', not a reference to line, why?
                self.cursor_position = 0
            elif char == '\r': # Carriage return
                self.cursor_position = 0
            elif self.cursor_position >= line_len:
                line += char
                line_len += 1
                self.cursor_position += 1
            else:
                line = line[:self.cursor_position] + char + line[self.cursor_position + 1:]
                self.cursor_position += 1
        self.lines[-1] = line

        if len(self.lines) > self.max_lines:
            self.lines = self.lines[-self.max_lines:]

        self.update()

    def paintEvent(self, event):
        """Draw the console content."""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.TextAntialiasing)

        font_metrics = painter.fontMetrics()
        line_height = font_metrics.lineSpacing()
        char_width = font_metrics.horizontalAdvance('M')  # Monospace width

        # Calculate visible area
        visible_lines = self.height() // line_height
        total_lines = len(self.lines)

        # Handle scrolling
        if total_lines > visible_lines:
            start_line = max(0, total_lines - visible_lines)
        else:
            start_line = 0

        # Draw background
        painter.fillRect(self.rect(), QColor(0, 0, 0))

        # Draw text
        painter.setPen(QColor(0, 255, 0))  # Green text on black background
        y = 0
        for i in range(start_line, min(start_line + visible_lines, total_lines)):
            line = self.lines[i]
            # Simple word wrapping at character level
            wrapped_lines = self._wrap_text(line, self.width() // char_width)
            for wrapped_line in wrapped_lines:
                if y + line_height > self.height():
                    break
                painter.drawText(2, y + font_metrics.ascent(), wrapped_line)
                y += line_height

    def _wrap_text(self, text, max_chars):
        """Wrap text at character level."""
        if not text:
            return [""]
        return [text[i:i + max_chars] for i in range(0, len(text), max_chars)]

    def wheelEvent(self, event):
        """Handle mouse wheel for scrolling."""
        # Basic scrolling support
        if event.angleDelta().y() > 0:
            self.scroll_position = max(0, self.scroll_position - 1)
        else:
            max_scroll = max(0, len(self.lines) - (self.height() // self.fontMetrics().lineSpacing()))
            self.scroll_position = min(max_scroll, self.scroll_position + 1)
        self.update()
        '''


class BusyIndicator(QWidget):
    """Widget that shows busy state based on signals from GimbalConnection."""

    def __init__(self, connection, parent=None):
        super().__init__(parent)
        self.connection = connection
        self.is_busy = False

        self.connection.busy_state_changed.connect(self.on_busy_state_changed)
        self.animation_timer = QTimer()
        self.animation_timer.timeout.connect(self.update)  # Trigger repaint

        self.setFixedSize(20, 20)

        self.pen = QPen(QColor(50, 50, 40))
        self.pen.setWidth(2)

    def on_busy_state_changed(self, busy):
        """Handle busy state changes from GimbalConnection."""
        self.is_busy = busy
        self.update()

        if busy:
             self.animation_timer.start(100)
        else:
            self.animation_timer.stop()

    def paintEvent(self, event):
        """Draw the busy indicator."""
        if not self.is_busy:
            return  # Invisible when not busy

        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.setPen(self.pen)

        center = self.rect().center()
        radius = 8

        # Draw arcs to create spinning effect
        angle = int(time.time() * 400) % 360  # Rotate based on time

        painter.drawArc(center.x() - radius, center.y() - radius,
                        radius * 2, radius * 2,
                        angle * 16, 270 * 16)  # 270 degrees arc


class ForceBarWidget(QWidget):
    """Custom widget to display motor force as a horizontal bar."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(150, 10)
        self.setMaximumSize(150, 10)
        self.force_value = 0.0

    def set_force(self, force):
        """Set the force value (-1.0 to 1.0 range)."""
        self.force_value = max(-1.0, min(1.0, force))
        self.update()

    def paintEvent(self, event):
        """Draw the force bar."""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        # Draw black frame
        painter.setPen(QColor(0, 0, 0))
        painter.setBrush(Qt.BrushStyle.NoBrush)
        painter.drawRect(0, 0, self.width() - 1, self.height() - 1)

        # Draw center line
        center_y = self.height() // 2
        painter.setPen(QColor(128, 128, 128))
        painter.drawLine(0, center_y, self.width(), center_y)

        # Draw force bar
        if self.force_value != 0.0:
            painter.setPen(Qt.PenStyle.NoPen)
            painter.setBrush(QColor(255, 0, 0))  # Red

            center_x = self.width() // 2
            bar_width = int(abs(self.force_value) * (self.width() // 2 - 2))

            if self.force_value > 0:
                # Positive force - extend right from center
                painter.drawRect(center_x, 2, bar_width, self.height() - 4)
            else:
                # Negative force - extend left from center
                painter.drawRect(center_x - bar_width, 2, bar_width, self.height() - 4)


class AngleBarWidget(QWidget):
    """Custom widget to display angle as a bar with notches and zones."""

    def __init__(self, min_angle, max_angle, parent=None):
        super().__init__(parent)
        self.setMinimumSize(180, 10)
        self.setMaximumSize(180, 10)
        self.setMouseTracking(True)  # Enable mouse tracking without button press
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.current_angle = 0.0
        self.home_angle = None
        self.target_angle = None
        self.limit_min = None
        self.limit_max = None
        self.has_limits = False

    def set_angle(self, angle):
        """Set the current angle value."""
        self.current_angle = angle
        self.update()

    def set_home_angle(self, home_angle):
        """Set the home angle position."""
        self.home_angle = home_angle
        self.update()

    def set_target_angle(self, target_angle):
        """Set the target angle position (for future use)."""
        self.target_angle = target_angle
        self.update()

    def set_limits(self, has_limits, limit_min, limit_max):
        """Set limit information."""
        self.has_limits = has_limits
        self.limit_min = limit_min
        self.limit_max = limit_max
        self.update()

    def angle_to_pixel(self, angle):
        """Convert angle to pixel position."""
        range_size = self.max_angle - self.min_angle
        normalized = (angle - self.min_angle) % range_size / range_size
        return int(normalized * self.width())

    def pixel_to_angle(self, angle):
        """Convert pixel position to angle."""
        range_size = self.max_angle - self.min_angle
        normalized = angle / self.width()
        return self.min_angle + normalized * range_size

    def paintEvent(self, event):
        """Draw the angle bar."""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        # Draw background
        painter.fillRect(self.rect(), QColor(240, 240, 240))

        # Draw black frame
        painter.setPen(QColor(0, 0, 0))
        painter.setBrush(Qt.BrushStyle.NoBrush)
        painter.drawRect(0, 0, self.width() - 1, self.height() - 1)

        center_y = self.height() // 2

        # Draw limit zones if enabled
        if self.has_limits and self.limit_min is not None and self.limit_max is not None:
            painter.setPen(Qt.PenStyle.NoPen)
            painter.setBrush(QColor(255, 0, 0, 128))  # Semi-transparent red

            min_pixel = self.angle_to_pixel(self.limit_min)
            max_pixel = self.angle_to_pixel(self.limit_max) + 1

            if min_pixel <= max_pixel: # Note the rounding can make this check fail in extreme cases
                # Normal case: single rectangle
                if max_pixel - min_pixel < 4:
                    add = (4 - (max_pixel - min_pixel)) // 2
                    min_pixel -= add
                    max_pixel += 4 - add
                painter.drawRect(min_pixel, center_y - 5, max_pixel - min_pixel, 10)
            else:
                # Wraparound case: two rectangles
                if self.width() - min_pixel < 3:
                    min_pixel = self.width() - 3
                painter.drawRect(min_pixel, center_y - 5, self.width() - min_pixel, 10)
                painter.drawRect(0, center_y - 5, min(max_pixel, 3), 10)

        # Draw center line
        painter.setPen(QColor(128, 128, 128))
        painter.drawLine(0, center_y, self.width(), center_y)

        # Draw home angle notch
        if self.home_angle is not None:
            painter.setPen(QColor(0, 128, 0))  # Green
            painter.setBrush(QColor(0, 128, 0))
            home_pixel = self.angle_to_pixel(self.home_angle)
            painter.drawRect(home_pixel - 1, center_y - 5, 2, 10)

        # Draw target angle notch
        if self.target_angle is not None:
            painter.setPen(QColor(0, 0, 255))  # Blue
            painter.setBrush(QColor(0, 0, 255))
            target_pixel = self.angle_to_pixel(self.target_angle)
            painter.drawRect(target_pixel - 1, center_y - 5, 2, 10)

        # Draw current angle notch last to be on top as the most dynamic
        painter.setPen(QColor(0, 0, 0))  # Red
        painter.setBrush(QColor(0, 0, 0))
        current_pixel = self.angle_to_pixel(self.current_angle)
        painter.drawRect(current_pixel - 1, center_y - 5, 2, 10)

    def mouseMoveEvent(self, event):
        """Handle mouse movement for tooltip."""
        pixel = event.position().x()
        angle = self.pixel_to_angle(pixel)

        tooltip = f"{angle:.0f}°"

        # Check if hovering over marks
        current_pixel = self.angle_to_pixel(self.current_angle)
        if abs(pixel - current_pixel) < 3:
            tooltip += "\nCurrent angle"

        if self.home_angle is not None:
            home_pixel = self.angle_to_pixel(self.home_angle)
            if abs(pixel - home_pixel) < 3:
                tooltip += "\nHome angle"

        if self.target_angle is not None:
            target_pixel = self.angle_to_pixel(self.target_angle)
            if abs(pixel - target_pixel) < 3:
                tooltip += "\nTarget angle"

        if self.has_limits:
            # Check if in limit zones
            min_pixel = self.angle_to_pixel(self.limit_min)
            max_pixel = self.angle_to_pixel(self.limit_max)

            in_limit_zone = False
            if min_pixel < max_pixel:
                in_limit_zone = min_pixel - 2 <= pixel <= max_pixel + 2
            else:
                in_limit_zone = pixel >= min_pixel - 2 or pixel <= max_pixel + 2

            if in_limit_zone:
                tooltip += "\nLimit zone"

        #self.setToolTip(tooltip)
        QToolTip.showText(QCursor.pos(), tooltip, self, QRect(), 0)
        #super().mouseMoveEvent(event)


class StatusTab(QWidget):
    """Tab for displaying gimbal status and 3D visualization."""

    def __init__(self, connection: GimbalConnection, geometry: GimbalGeometry, parent=None):
        super().__init__(parent)
        self.connection = connection
        self.geometry = geometry

        layout = QVBoxLayout()

        # 3D visualization
        self.view_3d = Gimbal3DWidget(self.geometry)
        layout.addWidget(self.view_3d)

        # Connect to geometry changes
        self.geometry.geometry_changed.connect(self.on_geometry_changed)

        # Joints display
        joints_group = QGroupBox("Joints")
        joints_layout = QGridLayout()

        # Column headers
        joints_layout.addWidget(QLabel("Angle"), 0, 2)
        joints_layout.addWidget(QLabel("Force"), 0, 3)

        self.joint_labels = []
        self.encoder_labels = []
        self.angle_bars = []
        self.force_bars = []
        self.motors_on = None
        self.joint_map = [0, 1, 2]  # Default mapping without axes calibration

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
            angle_bar = AngleBarWidget(0, 360)  # Default range
            joints_layout.addWidget(angle_bar, i + 1, 2)
            joints_layout.setAlignment(angle_bar, Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
            self.angle_bars.append(angle_bar)

            # Force bar widget
            force_bar = ForceBarWidget()
            joints_layout.addWidget(force_bar, i + 1, 3)
            self.force_bars.append(force_bar)

        joints_layout.setColumnStretch(1, 3)
        joints_layout.setColumnStretch(2, 3)
        joints_group.setLayout(joints_layout)
        layout.addWidget(joints_group)

        self.update_joint_labels()

        # Battery voltage display
        vbat_group = QGroupBox("Battery Voltage")
        vbat_layout = QGridLayout()
        self.vbat_label = QLabel("unknown")
        vbat_layout.addWidget(QLabel("Voltage:"), 0, 0)
        vbat_layout.addWidget(self.vbat_label, 0, 1)
        vbat_group.setLayout(vbat_layout)
        layout.addWidget(vbat_group)

        # Motor status and controls
        motor_group = QGroupBox("Motor Control")
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

        # Track if we've loaded geometry once
        self.geometry_loaded = False
        self.last_have_axes = False

    def start_updates(self):
        """Start update timers."""
        if self.connection.is_connected():
            self.update_timer.start()
            self.vbat_timer.start()
            # Enable motor control buttons
            self.motor_on_btn.setEnabled(True)
            self.motor_off_btn.setEnabled(True)

    def stop_updates(self):
        """Stop update timers."""
        self.update_timer.stop()
        self.vbat_timer.stop()
        # Reset displays and disable buttons
        self.vbat_label.setText("Disconnected")
        self.motor_status_label.setText("Disconnected")
        self.motor_on_btn.setEnabled(False)
        self.motor_off_btn.setEnabled(False)

    def update_encoders_and_forces(self):
        """Update encoder angle and motor force displays."""
        if not self.connection.is_connected():
            return

        if not hasattr(self, 'current_encoder_angles'):
            self.current_encoder_angles = [0.0, 0.0, 0.0]

        req_params = []

        # Read encoder angles
        def callback_encoders(values):
            for i in range(3):
                angle = self.geometry.angle_normalize_360(float(values[i]))
                self.current_encoder_angles[i] = angle
                # Map to display position using joint_map
                display_idx = self.joint_map[i]
                # Display relative to home angles if available
                if self.geometry.have_home:
                    relative_angle = self.geometry.angle_normalize_180(angle - self.geometry.home_angles[i])
                    self.encoder_labels[display_idx].setText(f"{relative_angle:.2f}°")
                    self.angle_bars[display_idx].set_angle(relative_angle)
                else:
                    self.encoder_labels[display_idx].setText(f"{angle:.2f}°")
                    self.angle_bars[display_idx].set_angle(angle)

        req_params += [f"encoders.{i}.reading" for i in range(3)]

        # Read main AHRS orientation
        def callback_q(value):
            # value is a list of 4 floats (w, x, y, z)
            self.current_main_ahrs_q = list(value) if isinstance(value, list) else [1.0, 0.0, 0.0, 0.0]
            self.view_3d.update_pose(self.current_encoder_angles, self.current_main_ahrs_q)

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
        if not self.connection.is_connected():
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
        # Update button states: On only when motors are off
        self.motor_on_btn.setEnabled(not self.motors_on and self.connection.is_connected())
        self.motor_off_btn.setEnabled(self.connection.is_connected())

        if not self.motors_on:
            for i in range(3):
                self.force_bars[i].set_force(0)

    def update_motors_status(self):
        """Update motors status display."""
        if not self.connection.is_connected():
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

    def on_geometry_changed(self):
        """Handle geometry changes."""
        self.update_joint_labels()

    def update_joint_labels(self):
        """Update joint labels, mapping, and angle bar ranges based on current geometry status."""
        if self.geometry.have_axes:
            # With axes calibration: show joint names and create reverse mapping
            joint_names = ["Inner (2)", "Middle (1)", "Outer (0)"]
            # Create reverse mapping: encoder index -> axis index
            # axis_to_encoder tells us which encoder each axis uses
            # We want encoder_to_axis: which axis each encoder drives
            encoder_to_axis = [0] * 3
            for axis_idx, encoder_idx in enumerate(self.geometry.axis_to_encoder):
                encoder_to_axis[encoder_idx] = axis_idx
            # Joints are displayed in reverse order (2, 1, 0)
            self.joint_map = [2 - axis for axis in encoder_to_axis]

            # Update angle bar ranges and home positions
            for i in range(3):
                axis_idx = 2 - i
                self.angle_bars[i].min_angle = -180
                self.angle_bars[i].max_angle = 180
                self.angle_bars[i].set_home_angle(0 if self.geometry.have_home else None)

                # Set limits
                if self.geometry.has_limits[axis_idx]:
                    lmin = self.geometry.limit_min[axis_idx] - self.geometry.home_angles[axis_idx] # 0 if unset
                    lmax = self.geometry.limit_max[axis_idx] - self.geometry.home_angles[axis_idx]
                    lmin = self.geometry.angle_normalize_180(lmin)
                    lmax = self.geometry.angle_normalize_180(lmax)
                    self.angle_bars[i].set_limits(True, lmin, lmax)
                else:
                    self.angle_bars[i].set_limits(False, None, None)

                # TODO: also read 'config.control.limit-margin' and add in degrees to both sides of the limit zone
        else:
            # Without axes calibration: show encoder numbers
            joint_names = [f"Encoder {i}" for i in range(3)]
            self.joint_map = [0, 1, 2]  # Direct mapping

            # Update angle bar ranges (0-360) and clear home positions
            for i in range(3):
                self.angle_bars[i].min_angle = 0
                self.angle_bars[i].max_angle = 360
                self.angle_bars[i].set_home_angle(None)
                self.angle_bars[i].set_limits(False, None, None)

        # Update the labels
        for i in range(3):
            self.joint_labels[i].setText(joint_names[i] + ':')

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


class CalibrationTab(QWidget):
    """Tab for calibration status and controls."""

    def __init__(self, connection: GimbalConnection, parent=None):
        super().__init__(parent)
        self.connection = connection

        layout = QVBoxLayout()

        # Calibration status
        status_group = QGroupBox("Calibration Status")
        status_layout = QVBoxLayout()

        self.status_labels = {}
        for name in ["Axes & Encoders", "Home Position", "Forward Direction"]:
            label = QLabel(f"{name}: Not calibrated")
            status_layout.addWidget(label)
            self.status_labels[name] = label

        status_group.setLayout(status_layout)
        layout.addWidget(status_group)

        # Calibration controls
        controls_group = QGroupBox("Calibration Steps")
        controls_layout = QVBoxLayout()

        calibration_steps = [
            "Gyro calibration",
            "Axes&encoders geometry",
            "Home position",
            "Forward direction",
            "Motor geometry",
            "Motor PID config"
        ]

        self.calibration_buttons = {}
        for step in calibration_steps:
            step_layout = QHBoxLayout()
            label = QLabel(step)
            button = QPushButton("Start")
            button.setEnabled(False)  # Placeholder, non-functional
            step_layout.addWidget(label)
            step_layout.addWidget(button)
            step_layout.addStretch()
            controls_layout.addLayout(step_layout)
            self.calibration_buttons[step] = button

        controls_group.setLayout(controls_layout)
        layout.addWidget(controls_group)

        layout.addStretch()
        self.setLayout(layout)

        # Update timer for status
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self.update_status)
        self.status_timer.setInterval(1000)  # 1 Hz

    def start_updates(self):
        """Start status update timer."""
        if self.connection.is_connected():
            self.status_timer.start()
            self.update_status()

    def stop_updates(self):
        """Stop status update timer."""
        self.status_timer.stop()

    def update_status(self):
        """Update calibration status display."""
        if not self.connection.is_connected():
            return

        def callback_have_axes(value):
            if value is not None:
                status = "Calibrated" if bool(value) else "Not calibrated"
                self.status_labels["Axes & Encoders"].setText(f"Axes & Encoders: {status}")

        def callback_have_home(value):
            if value is not None:
                status = "Calibrated" if bool(value) else "Not calibrated"
                self.status_labels["Home Position"].setText(f"Home Position: {status}")

        def callback_have_forward(value):
            if value is not None:
                status = "Calibrated" if bool(value) else "Not calibrated"
                self.status_labels["Forward Direction"].setText(f"Forward Direction: {status}")

        self.connection.read_param("config.have-axes", callback_have_axes)
        self.connection.read_param("config.control.have-home", callback_have_home)
        self.connection.read_param("config.control.have-forward", callback_have_forward)


class MainWindow(QMainWindow):
    """Main application window."""

    def __init__(self, debug=False):
        super().__init__()
        self.setWindowTitle("OpenBGC Gimbal app")
        self.setGeometry(100, 100, 1200, 800)

        # Create connection and geometry
        self.connection = GimbalConnection(debug)
        self.geometry = GimbalGeometry(self.connection)

        # Create splitter for main content and bottom strip
        splitter = self.create_splitter()
        self.setCentralWidget(splitter)

        # Connect signals
        self.connection.connection_changed.connect(self.on_connection_changed)

        # Initially disable non-connection tabs
        self.on_connection_changed(False)

        # Select status tab initially (connection is now at index 2)
        self.tab_selector.setCurrentRow(2)

    def create_splitter(self):
        """Create the main splitter with tab content and bottom strip."""
        splitter = QSplitter(Qt.Orientation.Vertical)
        splitter.setStyleSheet("QSplitter::handle { background-color: #bbbbbb; }")

        # Top part: main content
        main_widget = QWidget()
        main_layout = QHBoxLayout()

        # Left sidebar with tab list
        self.tab_selector = QListWidget()
        self.tab_selector.addItems(["Status", "Calibration", "Connection"])
        self.tab_selector.currentRowChanged.connect(self.on_tab_changed)
        self.tab_selector.setMaximumWidth(150)

        # Ensure disabled items are visually distinct
        self.tab_selector.setStyleSheet("""
            QListWidget::item:disabled { color: #aaaaaa; }
            QListWidget::item:enabled { color: #000000; }
        """)
        # background-color: #f0f0f0; background-color: transparent;
        main_layout.addWidget(self.tab_selector)

        # Right side: stacked widget for tab content
        self.stacked_widget = QStackedWidget()

        # Create tabs
        self.status_tab = StatusTab(self.connection, self.geometry)
        self.calibration_tab = CalibrationTab(self.connection)
        self.connection_tab = ConnectionTab(self.connection)

        self.stacked_widget.addWidget(self.status_tab)
        self.stacked_widget.addWidget(self.calibration_tab)
        self.stacked_widget.addWidget(self.connection_tab)

        main_layout.addWidget(self.stacked_widget)
        main_widget.setLayout(main_layout)

        # Bottom part: status strip
        bottom_widget = QWidget()
        bottom_layout = QHBoxLayout()
        #bottom_layout.setContentsMargins(5, 5, 5, 5)

        # Port
        self.port_status = QLabel()
        bottom_layout.addWidget(self.port_status, 0)  # Fixed size

        # Busy indicator
        self.busy_indicator = BusyIndicator(self.connection)
        bottom_layout.addWidget(self.busy_indicator, 0)  # Fixed size

        # Console widget
        self.console = ConsoleWidget(self.connection)
        bottom_layout.addWidget(self.console, 1)  # Takes remaining space

        bottom_widget.setLayout(bottom_layout)

        # Add to splitter
        splitter.addWidget(main_widget)
        splitter.addWidget(bottom_widget)

        # Set initial sizes (main content gets most space)
        splitter.setSizes([750, 50])
        splitter.setCollapsible(0, False)  # Main content not collapsible
        splitter.setCollapsible(1, True)   # Bottom strip collapsible

        return splitter

    def on_tab_changed(self, index):
        """Handle tab selection change."""
        if index >= 0:
            self.stacked_widget.setCurrentIndex(index)
            current_widget = self.stacked_widget.currentWidget()

            # If connecting and switching to a tab, start updates
            if index == 0 and self.connection.is_connected():
                self.status_tab.start_updates()
            else:
                self.status_tab.stop_updates()

            if index == 1 and self.connection.is_connected():
                self.calibration_tab.start_updates()
            else:
                self.calibration_tab.stop_updates()

    def on_connection_changed(self, connected: bool):
        """Handle connection state change."""
        # Enable/disable tabs based on connection
        if connected:
            # Enable all tabs
            for i in range(self.tab_selector.count()):
                item = self.tab_selector.item(i)
                if item:
                    item.setFlags(item.flags() | Qt.ItemFlag.ItemIsEnabled)

            # Switch to status tab
            self.tab_selector.setCurrentRow(0)
            self.status_tab.start_updates()

            self.port_status.setText(self.connection.port_path)
        else:
            # Disable status and calibration tabs (connection tab is at index 2)
            for i in range(2):
                item = self.tab_selector.item(i)
                if item:
                    item.setFlags(item.flags() & ~Qt.ItemFlag.ItemIsEnabled)

            # Stop updates
            self.status_tab.stop_updates()
            self.calibration_tab.stop_updates()

            # Switch to connection tab
            self.tab_selector.setCurrentRow(2)

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
        logger.info(f"Debug mode enabled. HAS_OPENGL: {HAS_OPENGL}")
    else:
        logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
        logger.setLevel(logging.INFO)

    app = QApplication(sys.argv)
    window = MainWindow(debug=args.debug)
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
