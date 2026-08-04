from typing import Callable

import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets

from .occupancy import FREE, OCCUPIED, UNKNOWN, grid_to_display_image


class MapGraphicsView(QtWidgets.QGraphicsView):
    def __init__(self, canvas):
        super().__init__()
        self.canvas = canvas
        self.setRenderHint(QtGui.QPainter.Antialiasing, False)
        self.setRenderHint(QtGui.QPainter.SmoothPixmapTransform, False)
        self.setDragMode(QtWidgets.QGraphicsView.NoDrag)
        self.setMouseTracking(True)
        self.setTransformationAnchor(QtWidgets.QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QtWidgets.QGraphicsView.AnchorViewCenter)
        self.setFocusPolicy(QtCore.Qt.StrongFocus)
        self._panning = False
        self._last_pan_pos = None
        self._painting = False
        self._drag_paint_value = FREE

    def wheelEvent(self, event):
        factor = 1.25 if event.angleDelta().y() > 0 else 0.8
        self.canvas.zoom_at_view_pos(factor, event.pos())
        event.accept()

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.MiddleButton:
            self._start_pan(event.pos())
            event.accept()
            return

        if event.button() == QtCore.Qt.LeftButton and event.modifiers() & QtCore.Qt.ShiftModifier:
            self._start_pan(event.pos())
            event.accept()
            return

        if event.button() == QtCore.Qt.LeftButton:
            self._painting = True
            self._drag_paint_value = self.canvas.paint_value
            self.canvas.paint_at_view_pos(event.pos(), self._drag_paint_value)
            event.accept()
            return

        if event.button() == QtCore.Qt.RightButton:
            self._painting = True
            self._drag_paint_value = OCCUPIED
            self.canvas.paint_at_view_pos(event.pos(), self._drag_paint_value)
            event.accept()
            return

        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if self._panning:
            delta = event.pos() - self._last_pan_pos
            self._last_pan_pos = event.pos()
            self.horizontalScrollBar().setValue(self.horizontalScrollBar().value() - delta.x())
            self.verticalScrollBar().setValue(self.verticalScrollBar().value() - delta.y())
            event.accept()
            return

        if self._painting and event.buttons() & (QtCore.Qt.LeftButton | QtCore.Qt.RightButton):
            self.canvas.paint_at_view_pos(event.pos(), self._drag_paint_value)
            event.accept()
            return

        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if event.button() in (QtCore.Qt.MiddleButton, QtCore.Qt.LeftButton):
            self._panning = False
            self._last_pan_pos = None

        if self._painting and event.button() in (QtCore.Qt.LeftButton, QtCore.Qt.RightButton):
            self._painting = False
            event.accept()
            return

        super().mouseReleaseEvent(event)

    def _start_pan(self, pos):
        self._panning = True
        self._last_pan_pos = pos
        self.setCursor(QtCore.Qt.ClosedHandCursor)

    def leaveEvent(self, event):
        if not self._panning:
            self.unsetCursor()
        super().leaveEvent(event)

    def keyPressEvent(self, event):
        step = 80
        if event.modifiers() & QtCore.Qt.ShiftModifier:
            step = 200
        if event.key() == QtCore.Qt.Key_Left:
            self.horizontalScrollBar().setValue(self.horizontalScrollBar().value() - step)
            event.accept()
            return
        if event.key() == QtCore.Qt.Key_Right:
            self.horizontalScrollBar().setValue(self.horizontalScrollBar().value() + step)
            event.accept()
            return
        if event.key() == QtCore.Qt.Key_Up:
            self.verticalScrollBar().setValue(self.verticalScrollBar().value() - step)
            event.accept()
            return
        if event.key() == QtCore.Qt.Key_Down:
            self.verticalScrollBar().setValue(self.verticalScrollBar().value() + step)
            event.accept()
            return
        super().keyPressEvent(event)


class MapEditorCanvas(QtWidgets.QMainWindow):
    def __init__(
        self,
        window_name: str,
        max_window_size: int,
        brush_radius: int,
        initial_zoom: float,
        max_zoom: float,
        on_paint: Callable[[int, int, int], None],
        on_save: Callable[[], None],
        on_quit: Callable[[], None],
    ):
        super().__init__()
        self.setWindowTitle(window_name)
        self.max_window_size = max_window_size
        self.brush_radius = brush_radius
        self.paint_value = FREE
        self.zoom = max(0.1, float(initial_zoom))
        self.max_zoom = max(self.zoom, float(max_zoom))
        self.on_paint = on_paint
        self.on_save = on_save
        self.on_quit = on_quit

        self.scene = QtWidgets.QGraphicsScene(self)
        self.pixmap_item = QtWidgets.QGraphicsPixmapItem()
        self.pixmap_item.setTransformationMode(QtCore.Qt.FastTransformation)
        self.scene.addItem(self.pixmap_item)

        self.view = MapGraphicsView(self)
        self.view.setScene(self.scene)
        self.view.setFixedSize(self.max_window_size, self.max_window_size)

        self.status = QtWidgets.QLabel('')
        self.buttons = {}
        toolbar = QtWidgets.QToolBar()
        toolbar.setMovable(False)
        self.addToolBar(QtCore.Qt.TopToolBarArea, toolbar)
        for label, action in (
            ('FREE', 'free'),
            ('BLOCK', 'occupied'),
            ('UNKNOWN', 'unknown'),
            ('BRUSH-', 'brush_down'),
            ('BRUSH+', 'brush_up'),
            ('SAVE OVERWRITE', 'save'),
            ('QUIT', 'quit'),
        ):
            button = QtWidgets.QToolButton()
            button.setText(label)
            button.clicked.connect(lambda _checked=False, a=action: self._handle_button(a))
            toolbar.addWidget(button)
            self.buttons[action] = button

        central = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(central)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.view)
        layout.addWidget(self.status)
        self.setCentralWidget(central)
        self.resize(self.max_window_size + 40, self.max_window_size + 90)
        self._apply_zoom(reset=True)
        self._update_button_states()
        self.show()
        self.view.setFocus()

    def update_scale(self, _width: int, _height: int) -> None:
        self._apply_zoom(reset=True)

    def display_to_grid(self, display_x: int, display_y: int, height: int, width: int):
        scene_pos = self.view.mapToScene(QtCore.QPoint(display_x, display_y))
        gx = int(scene_pos.x())
        gy = height - 1 - int(scene_pos.y())
        if gx < 0 or gx >= width or gy < 0 or gy >= height:
            return None
        return gx, gy

    def paint_at_view_pos(self, pos, value: int) -> None:
        self.on_paint(pos.x(), pos.y(), value)

    def show_waiting(self, source_path: str) -> None:
        self.status.setText('Waiting for map file: %s' % source_path)
        QtWidgets.QApplication.processEvents()

    def show_grid(self, grid, dirty: bool, save_path: str) -> None:
        image = grid_to_display_image(grid)
        qimage = self._gray_to_qimage(image)
        self.pixmap_item.setPixmap(QtGui.QPixmap.fromImage(qimage))
        self.scene.setSceneRect(self.pixmap_item.boundingRect())
        self.status.setText(
            'Save overwrites: %s | brush=%d | zoom=%.2fx%s'
            % (save_path, self.brush_radius, self.zoom, ' | modified' if dirty else '')
        )
        self._update_button_states()

    def wait_key(self) -> None:
        QtWidgets.QApplication.processEvents()

    def zoom_by(self, factor: float) -> None:
        self.zoom = min(self.max_zoom, max(0.1, self.zoom * factor))
        self._apply_zoom(reset=False)
        self._update_button_states()

    def zoom_at_view_pos(self, factor: float, view_pos) -> None:
        old_scene_pos = self.view.mapToScene(view_pos)
        self.zoom_by(factor)
        new_view_pos = self.view.mapFromScene(old_scene_pos)
        delta = new_view_pos - view_pos
        self.view.horizontalScrollBar().setValue(self.view.horizontalScrollBar().value() + delta.x())
        self.view.verticalScrollBar().setValue(self.view.verticalScrollBar().value() + delta.y())

    def closeEvent(self, event):
        self.on_quit()
        event.accept()

    def _handle_button(self, action: str) -> None:
        if action == 'free':
            self.paint_value = FREE
        elif action == 'occupied':
            self.paint_value = OCCUPIED
        elif action == 'unknown':
            self.paint_value = UNKNOWN
        elif action == 'brush_down':
            self.brush_radius = max(1, self.brush_radius - 1)
        elif action == 'brush_up':
            self.brush_radius += 1
        elif action == 'save':
            self.on_save()
        elif action == 'quit':
            self.on_quit()
        self._update_button_states()

    def _apply_zoom(self, reset: bool) -> None:
        transform = QtGui.QTransform()
        transform.scale(self.zoom, self.zoom)
        self.view.setTransform(transform)
        if reset:
            self.view.centerOn(self.pixmap_item)

    def _update_button_states(self) -> None:
        active = {
            'free': self.paint_value == FREE,
            'occupied': self.paint_value == OCCUPIED,
            'unknown': self.paint_value == UNKNOWN,
        }
        for action, button in self.buttons.items():
            button.setCheckable(action in active)
            if action in active:
                button.setChecked(active[action])

    @staticmethod
    def _gray_to_qimage(image: np.ndarray) -> QtGui.QImage:
        contiguous = np.ascontiguousarray(image)
        height, width = contiguous.shape
        qimage = QtGui.QImage(
            contiguous.data,
            width,
            height,
            contiguous.strides[0],
            QtGui.QImage.Format_Grayscale8,
        )
        return qimage.copy()
