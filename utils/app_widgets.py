from PyQt6.QtWidgets import QScrollArea, QWidget, QSizePolicy
from PyQt6.QtCore import Qt, QSize, QTimer
#TODO: PyQt5 fallback


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
