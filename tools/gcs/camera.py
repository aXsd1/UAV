"""Gazebo camera topic -> latest JPEG frame (gz-transport Python bindings + Pillow)."""

from __future__ import annotations

import io
import threading
import time

from gz.msgs10.image_pb2 import Image
from gz.transport13 import Node
from PIL import Image as PILImage

# gz.msgs.PixelFormatType
RGB_INT8, RGBA_INT8, BGRA_INT8, BGR_INT8 = 3, 4, 5, 8
MODES = {RGB_INT8: ("RGB", "RGB"), BGR_INT8: ("RGB", "BGR"), RGBA_INT8: ("RGBA", "RGBA"),
         BGRA_INT8: ("RGBA", "BGRA")}


class Camera:
    def __init__(self, topic: str, max_fps: float = 12.0):
        self.topic = topic
        self.min_dt = 1.0 / max_fps
        self.cond = threading.Condition()
        self.jpeg: bytes | None = None
        self.seq = 0
        self.t_frame = 0.0  # last encoded frame
        self.t_msg = 0.0  # last image received
        self.viewers = 0
        self.node = Node()
        if not self.node.subscribe(Image, topic, self._on_image):
            raise RuntimeError(f"cannot subscribe to {topic}")

    def _on_image(self, msg: Image):
        now = time.time()
        self.t_msg = now
        if self.viewers == 0 or now - self.t_frame < self.min_dt:
            return
        mode = MODES.get(msg.pixel_format_type)
        if mode is None:
            return
        img = PILImage.frombytes(mode[0], (msg.width, msg.height), msg.data, "raw", mode[1])
        buf = io.BytesIO()
        img.convert("RGB").save(buf, "JPEG", quality=75)
        with self.cond:
            self.jpeg = buf.getvalue()
            self.seq += 1
            self.t_frame = now
            self.cond.notify_all()

    def alive(self) -> bool:
        return time.time() - self.t_msg < 3.0

    def frames(self):
        """Yield JPEG frames as they arrive (blocks); yields None every 2 s without frames."""
        self.viewers += 1
        try:
            seen = -1
            while True:
                with self.cond:
                    self.cond.wait_for(lambda: self.seq != seen, timeout=2.0)
                    if self.seq == seen:
                        frame = None
                    else:
                        seen, frame = self.seq, self.jpeg
                yield frame
        finally:
            self.viewers -= 1
