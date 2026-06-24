"""
Video recording utilities for creating MP4 demos with sensor overlays.

Provides VideoRecorder class for capturing simulation frames and writing to MP4,
along with helper functions for adding text and sensor visualizations.
"""

import numpy as np
import imageio
from typing import Optional, Tuple


class VideoRecorder:
    """
    Records simulation frames and writes to MP4 using imageio.

    Collects (1080, 1920, 3) uint8 RGB frames and writes them to an MP4 file
    at 30fps when finalize() is called.
    """

    def __init__(self, output_path: str, width: int = 1920, height: int = 1080, fps: int = 30):
        """
        Initialize the video recorder.

        Args:
            output_path: Path where MP4 will be saved.
            width: Frame width in pixels (default 1920).
            height: Frame height in pixels (default 1080).
            fps: Frames per second (default 30).
        """
        self.output_path = output_path
        self.width = width
        self.height = height
        self.fps = fps
        self.frames = []

    def add_frame(self, pixels: np.ndarray) -> None:
        """
        Add a frame to the video.

        Args:
            pixels: Frame as (height, width, 3) uint8 RGB array.
        """
        if pixels.shape != (self.height, self.width, 3):
            raise ValueError(
                f"Frame shape {pixels.shape} does not match expected ({self.height}, {self.width}, 3)"
            )
        if pixels.dtype != np.uint8:
            raise TypeError(f"Frame must be uint8, got {pixels.dtype}")

        self.frames.append(pixels)

    def finalize(self) -> None:
        """
        Write all collected frames to MP4 file and print summary.
        """
        if not self.frames:
            print("Warning: No frames to write")
            return

        # Write video using imageio
        with imageio.get_writer(self.output_path, fps=self.fps) as writer:
            for frame in self.frames:
                writer.append_data(frame)

        # Print summary
        duration = len(self.frames) / self.fps
        print(f"✓ Video saved: {self.output_path}")
        print(f"  Duration: {duration:.2f}s")
        print(f"  Frames: {len(self.frames)}")


def add_text_overlay(
    image: np.ndarray,
    text: str,
    position: Tuple[int, int] = (10, 30),
    color: Tuple[int, int, int] = (255, 255, 255),
    fontsize: int = 14
) -> np.ndarray:
    """
    Add text overlay to an image (placeholder implementation).

    Args:
        image: Input image as uint8 array.
        text: Text to overlay.
        position: (x, y) position for text.
        color: RGB color tuple.
        fontsize: Font size in pixels.

    Returns:
        Image with text overlay (simplified version without actual text rendering).
    """
    # Placeholder: Return image unchanged
    # Full implementation would use PIL or cv2.putText
    return image.copy()


def create_force_heatmap(
    data,
    sensor_reader,
    width: int = 1920,
    height: int = 1080
) -> np.ndarray:
    """
    Create a force heatmap overlay visualization (placeholder).

    Args:
        data: MuJoCo data object.
        sensor_reader: SensorReader instance for reading forces.
        width: Output width in pixels.
        height: Output height in pixels.

    Returns:
        Heatmap as (height, width, 3) uint8 RGB array.
    """
    # Placeholder: Return black image
    # Full implementation would create actual heatmap visualization
    return np.zeros((height, width, 3), dtype=np.uint8)
