"""再描画後も位置を保つ、ドラッグ可能な凡例。"""
class MovingLegends:
    def __init__(self, canvas, changed):
        self.canvas = canvas
        self.changed = changed
        self.items = {}
        self.positions = {}
        self.dragging = None
        canvas.mpl_connect('button_press_event', self.press)
        canvas.mpl_connect('button_release_event', self.release)

    def press(self, event):
        if event.button != 1:
            return
        for key, legend in self.items.items():
            if legend.get_window_extent().contains(event.x, event.y):
                self.dragging = key
                return

    def release(self, event):
        if self.dragging is None:
            return
        legend = self.items.get(self.dragging)
        if legend is not None:
            box = legend.get_window_extent()
            self.positions[self.dragging] = tuple(legend.axes.transAxes.inverted().transform((box.x0, box.y0)))
        self.dragging = None
        self.changed()

    def clear(self):
        for legend in self.items.values():
            legend.set_draggable(False)
        self.items.clear()

    def add(self, axis, key, **options):
        if key in self.positions:
            options.update(loc='lower left', bbox_to_anchor=self.positions[key], borderaxespad=0)
        legend = axis.legend(**options)
        legend.set_draggable(True)
        for text in legend.get_texts():
            text.set_color('white')
        self.items[key] = legend
        return legend
