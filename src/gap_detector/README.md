# gap_detector

Finds the largest free-space gap in a depth image and publishes its normalised horizontal coordinate, intended to guide a wheeled robot toward the safest direction of travel.

## Algorithm

1. Samples a horizontal band near the bottom of the frame (`sample_row_ratio`, default 0.85) where ground-level obstacles are visible.
2. Averages pixel intensity across the band into a 1-D column profile.
3. Applies a Gaussian blur to the profile to suppress per-pixel noise.
4. Selects the column with the minimum intensity — darkest = farthest = safest gap.
5. Publishes the result as a normalised u coordinate (−1 = left edge, 0 = centre, +1 = right edge).

## Usage

```bash
colcon build --symlink-install --packages-select gap_detector
source install/setup.bash
ros2 run gap_detector gap_detector
```

## Topics

| Topic | Type | Direction |
|---|---|---|
| `/camera/depth/image_raw` | `sensor_msgs/Image` (mono8) | Sub |
| `/gap_detector/point` | `geometry_msgs/PointStamped` | Pub — normalised gap coordinate |
| `/gap_detector/debug` | `sensor_msgs/Image` (bgr8) | Pub — annotated debug view |

The published `PointStamped` inherits the depth image header (timestamp and frame id). Fields:

| Field | Value |
|---|---|
| `point.x` | u ∈ [−1, +1] — normalised horizontal gap position |
| `point.y` | 0.0 (fixed) |
| `point.z` | 0.0 (fixed) |

## Parameters

All parameters are live-adjustable via `ros2 param set`.

| Parameter | Default | Effect |
|---|---|---|
| `sample_row_ratio` | 0.85 | Fractional row position to sample (0 = top, 1 = bottom) |
| `band_height_ratio` | 0.10 | Height of the sampled band as a fraction of image height |
| `smoothing_kernel` | 7 | Gaussian kernel width in pixels for 1-D profile smoothing |

## Tuning

| Symptom | Adjustment |
|---|---|
| Gap jumps between frames | Increase `smoothing_kernel` (e.g. 15–31) |
| Gap always detected at image edge | Camera too low — raise `sample_row_ratio` toward 0.7 |
| Ground obstacles not captured | Camera too high — lower `sample_row_ratio` toward 0.9 |
| Band too narrow, noisy result | Increase `band_height_ratio` to 0.15–0.20 |

---
