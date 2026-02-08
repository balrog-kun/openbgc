# vim: set ts=4 sw=4 sts=4 et :
import math
import time

try:
    from PyQt6.QtWidgets import (
        QWidget, QPlainTextEdit, QToolTip, QScrollArea, QSizePolicy,
        QLabel, QHBoxLayout, QToolButton, QStyle,
    )
    from PyQt6.QtCore import (
        Qt, QTimer, pyqtSignal, QRect, QSize,
    )
    from PyQt6.QtOpenGLWidgets import QOpenGLWidget
    from PyQt6.QtGui import QColor, QPainter, QTextOption, QCursor, QPen
    PYQT_VERSION = 6
except ImportError:
    from PyQt5.QtWidgets import (
        QWidget, QPlainTextEdit, QToolTip, QScrollArea, QSizePolicy,
        QLabel, QHBoxLayout, QToolButton, QStyle,
    )
    from PyQt5.QtCore import (
        Qt, QTimer, pyqtSignal, QRect, QSize,
    )
    from PyQt5.QtOpenGL import QOpenGLWidget
    from PyQt5.QtGui import QColor, QPainter, QTextOption, QCursor, QPen
    PYQT_VERSION = 5

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


class ConsoleWidget(QPlainTextEdit): # QTextEdit if we need colours, highlights, etc.
    """Custom console widget for displaying text log with terminal-like behavior."""

    def __init__(self, text_log_ref, parent=None):
        super().__init__(parent)
        self.text_log_ref = text_log_ref
        self.setMinimumHeight(25)
        self.setReadOnly(True)
        self.setMaximumBlockCount(1000)
        self.setUndoRedoEnabled(False)
        self.setWordWrapMode(QTextOption.WrapMode.WrapAtWordBoundaryOrAnywhere)
        self.newline = False
        #self.setStyleSheet("QPlainTextEdit { background-color: #bbbbbb; color: #eeeeee; }")

        # Use monospace font
        font = self.font()
        font.setFamily("Monospace")
        font.setPointSize(int(font.pointSize() * 0.8))
        font.setStyleHint(font.StyleHint.TypeWriter)
        self.setFont(font)

        self._active = True
        text_log_ref.text_logged.connect(self.on_text_logged)

    def on_text_logged(self, text):
        """Update the display with new log content."""
        if not self._active:
            return

        cursor = self.textCursor()
        after_newline = False
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
                    after_newline = True
                elif not after_newline:
                    # self.setOverwriteMode(True) doesn't seem to work for QTextCursor-based edits
                    cursor.deleteChar()
                cursor.insertText(char)
        self.setTextCursor(cursor)
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

    def set_active(self, active):
        """Set whether the console should process text."""
        self._active = active


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

    angle_changed = pyqtSignal(float)  # Signal emitted when angle is set via click

    def __init__(self, min_angle=0, max_angle=360, angle_input=None, parent=None):
        super().__init__(parent)
        self.setMinimumSize(180, 10)
        self.setMaximumSize(180, 10)
        self.setMouseTracking(True)  # Enable mouse tracking without button press
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.angle_input = angle_input
        self.current_angle = 0.0
        self.home_angle = None
        self.park_angle = None
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

    def set_park_angle(self, park_angle):
        """Set the home angle position."""
        self.park_angle = park_angle
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
                painter.drawRect(0, center_y - 5, max(max_pixel, 3), 10)

        # Draw center line
        painter.setPen(QColor(128, 128, 128))
        painter.drawLine(0, center_y, self.width(), center_y)

        # Draw park angle notch
        if self.park_angle is not None:
            painter.setPen(QColor(128, 0, 128))  # Violet
            painter.setBrush(QColor(128, 0, 128))
            park_pixel = self.angle_to_pixel(self.park_angle)
            painter.drawRect(park_pixel - 1, center_y - 5, 2, 10)

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
        """Handle mouse movement for tooltip and input field."""
        pixel = event.position().x()
        angle = self.pixel_to_angle(pixel)

        # Don't show angle in tooltip if we have an input field
        tooltip = "" if self.angle_input else f"{angle:.0f}°"

        # Check if hovering over marks
        current_pixel = self.angle_to_pixel(self.current_angle)
        if abs(pixel - current_pixel) < 3:
            tooltip += ("\n" if tooltip else "") + "Current angle"

        if self.park_angle is not None:
            park_pixel = self.angle_to_pixel(self.park_angle)
            if abs(pixel - park_pixel) < 3:
                tooltip += ("\n" if tooltip else "") + "Park angle"

        if self.home_angle is not None:
            home_pixel = self.angle_to_pixel(self.home_angle)
            if abs(pixel - home_pixel) < 3:
                tooltip += ("\n" if tooltip else "") + "Home angle"

        if self.target_angle is not None:
            target_pixel = self.angle_to_pixel(self.target_angle)
            if abs(pixel - target_pixel) < 3:
                tooltip += ("\n" if tooltip else "") + "Target angle"

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
                tooltip += ("\n" if tooltip else "") + "Limit zone"

        if tooltip:
            QToolTip.showText(QCursor.pos(), tooltip, self, QRect(), 0)
        else:
            QToolTip.hideText()

        if self.angle_input:
            self.angle_input.setValue(angle)
            if not tooltip:
                self.setToolTip("Click to set target")

    def mousePressEvent(self, event):
        """Handle mouse press for sending angle commands."""
        if event.button() == Qt.MouseButton.LeftButton and self.angle_input:
            pixel = event.position().x()
            angle = self.pixel_to_angle(pixel)
            self.angle_changed.emit(angle)
        super().mousePressEvent(event)


class SpeedBarWidget(QWidget):
    """Custom widget to display angular speed as a horizontal bar."""

    speed_changed = pyqtSignal(float)  # Signal emitted when speed is set via click

    def __init__(self, max_speed, speed_input=None, parent=None):
        super().__init__(parent)
        self.setMinimumSize(100, 10)
        self.setMaximumSize(100, 10)
        self.max_speed = max_speed
        self.speed_input = speed_input
        self.current_speed = 0.0
        self.setMouseTracking(True)

    def set_speed(self, speed):
        """Set the current speed value."""
        self.current_speed = speed
        self.update()

    def set_max_speed(self, max_speed):
        """Set the maximum speed for the bar."""
        self.max_speed = max_speed
        self.update()

    def paintEvent(self, event):
        """Draw the speed bar."""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        # Draw background
        painter.fillRect(self.rect(), QColor(240, 240, 240))

        # Draw black frame
        painter.setPen(QColor(0, 0, 0))
        painter.setBrush(Qt.BrushStyle.NoBrush)
        painter.drawRect(0, 0, self.width()-1, self.height()-1)

        # Draw center line
        center_y = self.height() // 2
        painter.setPen(QColor(128, 128, 128))
        painter.drawLine(0, center_y, self.width(), center_y)

        # Draw speed bar
        if abs(self.current_speed) > 0.1:  # Only draw if significant speed
            painter.setPen(Qt.PenStyle.NoPen)
            painter.setBrush(QColor(0, 128, 255))  # Blue

            center_x = self.width() // 2
            speed_ratio = min(abs(self.current_speed) / self.max_speed, 1.0)
            bar_width = int(speed_ratio * (self.width() // 2 - 2))

            if self.current_speed > 0:
                # Positive speed - extend right from center
                painter.drawRect(center_x, 2, bar_width, self.height() - 4)
            else:
                # Negative speed - extend left from center
                painter.drawRect(center_x - bar_width, 2, bar_width, self.height() - 4)

    def pixel_to_value(self, pixel):
        center_x = self.width() / 2
        return (pixel - center_x) / center_x * self.max_speed

    def mouseMoveEvent(self, event):
        """Handle mouse movement for speed input field."""
        if self.speed_input:
            pixel = event.position().x()
            speed = self.pixel_to_value(pixel)
            self.speed_input.setValue(speed)
            self.setToolTip("Click to set target")

    def mousePressEvent(self, event):
        """Handle mouse press for sending speed commands."""
        if event.button() == Qt.MouseButton.LeftButton and self.speed_input:
            pixel = event.position().x()
            speed = self.pixel_to_value(pixel)
            self.speed_changed.emit(speed)
        super().mousePressEvent(event)


class Gimbal3DWidget(QOpenGLWidget):
    """3D visualization widget for gimbal status."""

    def __init__(self, geometry, parent=None):
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
        self.joint_angles = [0.0, 0.0, 0.0]  # degrees
        self.main_ahrs_q = (1.0, 0.0, 0.0, 0.0)  # quaternion (w, x, y, z)

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


    def update_pose(self, joint_angles, main_ahrs_q):
        """Update current pose (encoder angles in degrees, quaternion as list)."""
        self.joint_angles = joint_angles
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
            angle_rad = -math.radians(self.joint_angles[num])
            axis = self.geometry.axes[num]

            # Find arm orientation
            q_joint_inv = self.geometry.quaternion_from_axis_angle(axis, angle_rad)
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
        axis_body = self.geometry.vector_rotate(axes_world[2], self.geometry.quaternion_conjugate(camera_q))
        camera_size = (self.camera_width, self.camera_depth, self.camera_height)
        # We can ignore signs in the local frame because the camera box is assumed symmetric
        camera_pos_dist = 0.5 * self.geometry.vector_dot(camera_size, [abs(v) for v in axis_body])
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

        motors.append((motor_top, motor_base, self.geometry.matrix_t((seg_x, seg_y, seg_z))))

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
        glTranslatef(0, 0, -self.camera_height / 2)

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
        screen_center_width = -(self.camera_width - screen_width) * 0.5 + self.camera_width * 0.1
        screen_center_depth = -(self.camera_depth / 2 + screen_depth / 2)
        screen_bottom_height = (self.camera_height - screen_height) / 2
        glPushMatrix()
        glTranslatef(screen_center_width, screen_center_depth, screen_bottom_height)
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
        glTranslatef(lens_base_center_width, lens_base_center_depth, lens_base_center_height)
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


class AutoScrollArea(QScrollArea):
    """A QScrollArea that:
    * automatically adjusts its size based on content size up to max_size.
    * no frame.
    * expands when scrollbars are needed, doesn't shrink content area.
      Content area always between content_min_size and content_max_size.
      Those are set separately from set{Minimum,Maximum}{Size,Width,Height}()
      because these public properties communicate our preferences to the parent
      layout and we have no control beyond that meaning that we couldn't take
      the content size into account.  Instead we calculate our ideal size given
      content_min_size, content_max_size and the content's current size at any
      time and we internally set the two public (min and max) properties to
      that one size so as to leave the parent layout no choice.
    """

    def __init__(self, content_min_size, content_max_size, parent: QWidget | None = None):
        super().__init__(parent)

        # No frame around the contents
        self.setFrameShape(self.Shape.NoFrame)

        # By default, show scrollbars only when needed
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)

        # Don't make widget resizable - we control sizing manually
        self.setWidgetResizable(False)

        # Track when we're adjusting size to avoid recursion
        self._adjusting_size = False

        self.content_min_size = content_min_size
        self.content_max_size = content_max_size
        self._last_size = QSize()
        self._scrollbar_spacing = 8

    def setWidget(self, widget: QWidget):
        """Set the widget inside the scroll area."""
        super().setWidget(widget)
        widget.installEventFilter(self)
        self.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)

    def eventFilter(self, obj, event):
        """Filter events to detect size changes in the content widget."""
        if obj == self.widget() and event.type() in [event.Type.Resize]:
            # Schedule size update - don't do it immediately to avoid recursion
            QTimer.singleShot(0, self._update_fixed_size)
        return super().eventFilter(obj, event)

    def showEvent(self, event):
        """Handle show events to ensure proper initial sizing."""
        super().showEvent(event)
        self._update_fixed_size()

    def _update_fixed_size(self):
        """Update the fixed size based on content and constraints."""
        if self._adjusting_size or not self.widget():
            return

        self._adjusting_size = True

        self._last_size = self._calculate_desired_size()
        self.setFixedSize(self._last_size)
        # Update geometry to force layout reconsideration
        #self.updateGeometry()
        self._adjusting_size = False

    def _calculate_desired_size(self) -> QSize:
        """Calculate the desired size based on content and scrollbars."""
        if not self.widget():
            return self.content_min_size

        # Start with content's natural size
        content_size = self.widget().sizeHint()
        if content_size.isEmpty():
            content_size = self.widget().size()
        desired_width = content_size.width()
        desired_height = content_size.height()

        # Check if content width exceeds what would fit without scrollbar
        horizontal_needed = (
            self.horizontalScrollBarPolicy() == Qt.ScrollBarPolicy.ScrollBarAlwaysOn or
                (self.horizontalScrollBarPolicy() == Qt.ScrollBarPolicy.ScrollBarAsNeeded and
                 desired_width > self.content_max_size.width()))
        vertical_needed = (
            self.verticalScrollBarPolicy() == Qt.ScrollBarPolicy.ScrollBarAlwaysOn or
                (self.verticalScrollBarPolicy() == Qt.ScrollBarPolicy.ScrollBarAsNeeded and
                 desired_height > self.content_max_size.height()))

        # If scrollbars might appear, account for their size
        scrollbar_extent = self.style().pixelMetric(self.style().PixelMetric.PM_ScrollBarExtent)

        # Add scrollbar dimensions if needed
        if horizontal_needed:
            desired_height += scrollbar_extent + self._scrollbar_spacing
        if vertical_needed:
            desired_width += scrollbar_extent + self._scrollbar_spacing

        # Apply min/max constraints
        desired_width = max(desired_width, self.content_min_size.width())
        desired_height = max(desired_height, self.content_min_size.height())
        desired_width = min(desired_width, self.content_max_size.width())
        desired_height = min(desired_height, self.content_max_size.height())

        return QSize(desired_width, desired_height)

    def sizeHint(self) -> QSize:
        """Return the preferred size of the scroll area."""
        return self._last_size

    def minimumSizeHint(self) -> QSize:
        """Return the minimum size hint."""
        return self._last_size


class TabItemWidget(QWidget):
    def __init__(self, text, item, parent=None, button_cb=None):
        super().__init__(parent)
        self.text = text
        self.item = item
        self.tab_selector = parent

        self.label = QLabel(text)
        self.label.setWordWrap(True)
        self.label.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)

        self.layout = QHBoxLayout()
        self.layout.setContentsMargins(0, 0, 0, 0)
        self.layout.addWidget(self.label, 1)

        if button_cb is not None:
            self.button = QToolButton()
            self.button.setIcon(self.style().standardIcon(QStyle.StandardPixmap.SP_TitleBarNormalButton))
            self.button.setAutoRaise(True)
            self.button.setIconSize(QSize(12, 12))
            self.button.setFixedSize(QSize(16, 16))
            self.button.setToolTip("Open this tab in a separate window.\nClose it to see it back here.")
            self.button.clicked.connect(button_cb)
            self.layout.addWidget(self.button, 0, Qt.AlignmentFlag.AlignTop)
        else:
            self.button = None

        self.setLayout(self.layout)

        self.in_size_hint = False

    def sizeHint(self):
        if self.in_size_hint:
            return QSize(10, 10)
        self.in_size_hint = True
        width = self.tab_selector.visualRect(self.tab_selector.indexFromItem(self.item, 0)).width()
        self.in_size_hint = False

        if self.button is not None:
            bsize = self.button.sizeHint()
            width -= bsize.width()# + self.layout.spacing()

        metrics = self.label.fontMetrics()
        text_rect = metrics.boundingRect(0, 0, width, 10000, Qt.TextFlag.TextWordWrap, self.text)
        height = text_rect.height() + 6
        if self.button is not None:
            height = max(height, bsize.height())
        return QSize(width, height)
