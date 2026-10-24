# Event camera versus RealSense RGB

## Configuration

- Stimulus: Code 128-B moving across a 60 Hz HDMI LCD.
- Payloads: nine seven-character values in catalog groups of three.
- Speeds: 800, 1200, 1600, 2000 and 2400 display px/s.
- Event camera: CenturyArks SilkyCam VGA, filtered asynchronous events.
- RGB camera: Intel RealSense D435i color stream, 640×480 at 60 fps, manual exposure control 50.
- Acceptance: decoded value must equal the displayed payload. RGB values require a valid Code 128 checksum.

## 49-pass result

| Speed (px/s) | Event camera | RGB camera |
|---:|---:|---:|
| 800 | 10/10 | 10/10 |
| 1200 | 10/10 | 10/10 |
| 1600 | 10/10 | 10/10 |
| 2000 | 10/10 | 10/10 |
| 2400 | 9/9 | 9/9 |
| **Overall** | **49/49 (100%)** | **49/49 (100%)** |

Mean successful read time measured from the beginning of each screen pass was 1536.8 ms for the event pipeline and 1552.1 ms for RGB. This includes travel into each camera's field of view, so it is an operational timing measurement rather than pure algorithm latency. The approximately 1% difference is not statistically or practically sufficient to claim that either sensor is faster.

## Interpretation

The temporal event tracker now accumulates confidence-weighted evidence throughout a pass. It preserves strong reads and combines partial high-speed event slices, producing reliability equal to RGB in this test. The side-by-side software and measurement system are working, but this experiment still cannot support a claim that the event camera is meaningfully faster. LCD animation is frame-stepped: the screen holds a sharp barcode between refreshes. This removes the continuous exposure blur that motivates event-camera sensing.

The next research experiment should move a printed barcode using a conveyor, linear rail or rotating drum. Both cameras should observe the same path under the same illumination. Speed must be measured in m/s, and success rate should be estimated from repeated randomized passes. The speed ratio can then be reported as:

`event maximum speed at the chosen reliability threshold / RGB maximum speed at the same threshold`
