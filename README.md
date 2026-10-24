# SilkyCam event-camera barcode reader

This repository contains a complete C++ demonstration for reading fast-moving Code 128-B barcodes with a CenturyArks SilkyCam VGA event camera and comparing it side by side with an Intel RealSense D435i RGB camera.

The program generates barcodes, moves them across an external monitor, reads the same stimulus with both cameras, shows both results live, and records pass-by-pass measurements. The current implementation is a controlled research prototype rather than a general-purpose industrial barcode reader.

## Current result

In the final controlled LCD experiment, both pipelines decoded every completed pass:

| Display speed | SilkyCam event reader | RealSense RGB reader |
|---:|---:|---:|
| 800 px/s | 10/10 | 10/10 |
| 1200 px/s | 10/10 | 10/10 |
| 1600 px/s | 10/10 | 10/10 |
| 2000 px/s | 10/10 | 10/10 |
| 2400 px/s | 9/9 | 9/9 |
| **Overall** | **49/49 (100%)** | **49/49 (100%)** |

Mean successful read time, measured from the start of each screen pass, was 1536.8 ms for the event pipeline and 1552.1 ms for RGB. The approximately 1% difference is too small to claim a meaningful speed advantage. This time also includes the barcode travelling into each camera's field of view, so it is not pure algorithm latency.

The detailed result and its interpretation are in `COMPARISON_RESULTS.md`. Experimental CSV files and dashboard images are in `artifacts/`.

## Hardware used

- CenturyArks SilkyCam VGA event camera, 640 x 480, connected by USB.
- Intel RealSense D435i; its colour stream is used at 640 x 480 and 60 fps.
- Laptop running Ubuntu.
- External HDMI display used to show the moving barcode.
- Both cameras aimed at the same display region.

The external display used during development began at desktop coordinate `3840,0` and had a 1920 x 1080 stimulus window. This coordinate can be changed from the command line.

## Software requirements

- A C++17 compiler.
- CMake 3.16 or newer.
- OpenCV 4 with `core`, `imgproc`, `highgui`, `imgcodecs`, and `videoio`.
- Metavision SDK 5.x with the `core` and `stream` components and a working SilkyCam HAL plugin.
- Linux V4L2 access for the RealSense colour camera. The program uses OpenCV/V4L2 directly and does not require librealsense.
- An X11/XWayland desktop for reliable placement of the stimulus on the external monitor.

The supplied launcher defaults to this development SDK build:

```text
/home/aitsam/Documents/metavision_pro/openeb-5.3.0/build
```

On another machine, point `METAVISION_ROOT` at the Metavision build or installation before running:

```bash
METAVISION_ROOT=/path/to/metavision/build ./run_demo.sh
```

## Quick start

Connect both cameras and the external display, then run:

```bash
./run_demo.sh
```

`run_demo.sh` configures and builds the release executable, sets the Metavision plugin/library paths, selects the X11 Qt backend, and starts the demo.

Run the deterministic Code 128 tests without cameras:

```bash
./run_demo.sh --self-test
```

Run a bounded 30-second hardware experiment:

```bash
./run_demo.sh --duration 30 --output artifacts/run_30s
```

Use a new output directory for each formal experiment. CSV files are intentionally opened in append mode so repeated runs in the same directory are preserved.

## Useful options

```text
--self-test              Test the Code 128 generator and decoders
--headless               Capture and process without GUI windows
--stimulus-only          Show moving barcodes without opening a camera
--no-rgb                 Disable the RealSense comparison
--rgb-device N           Manually override RealSense /dev/videoN detection
--data-rate-plot         Show the live Event-ROI versus RGB data-rate plot
--no-data-rate-plot      Hide that plot; this is the default
--input FILE             Replay a Metavision RAW recording
--duration SECONDS       Stop automatically; 0 means continuous
--output DIR             Select the diagnostics and CSV directory
--stimulus-x PIXELS      Set the external display's desktop X coordinate
--stimulus-size WxH      Set the stimulus window size
--module-pixels N        Set the rendered width of one barcode module
```

Example for an external screen beginning at X coordinate 1920:

```bash
./run_demo.sh --stimulus-x 1920
```

The RealSense device number is detected automatically. This is important because Linux can change `/dev/videoN` numbers after a reboot. Discovery checks the device name and selects the RealSense node that supports the YUYV colour format. The `--rgb-device` option is only needed as a manual override.

## Keyboard controls

- `Q` or `Esc`: quit.
- `Space`: switch between automatic and fixed-speed operation.
- `1` to `5`: select 800, 1200, 1600, 2000, or 2400 px/s.
- `S`: save a dashboard snapshot.

## What appears on screen

The external monitor shows the moving Code 128 stimulus. The laptop shows a comparison dashboard:

- Left: SilkyCam event activity, detected scan band, candidate bounding box, decoded ID, event rate, speed, confidence, pass number, and read count.
- Right: RealSense RGB image, decoded ID, speed, frame count, pass number, and read count.
- Optional lower plot: host-side data volume for filtered event data inside the selected barcode ROI versus complete RGB BGR frames.

In the event view, cyan pixels are ON events and red pixels are OFF events. Grey horizontal lines show the automatically selected scan band. A green rectangle shows the matched barcode region.

The data-rate plot is not a measurement of USB wire bandwidth. Event MB/s is calculated from accepted events inside the current ROI and the in-memory `EventCD` size. RGB MB/s is calculated from all pixels in each complete BGR frame. The plot is useful for comparing what each processing pipeline handles, provided this definition is reported with the result.

## How the complete demo works

### 1. Barcode generation and motion

`src/code128.cpp` implements Code 128-B encoding, including the start symbol, payload symbols, modulo-103 checksum, stop symbol, and standard module-width patterns. It renders high-contrast barcode images without relying on an external barcode library.

The demo cycles these nine equal-length IDs:

```text
EVENT42  SILKY01  FAST128  BOX2048  TRACK99
JET9000  CODE777  RAPID88  SCAN555
```

They move horizontally over a white 1920 x 1080 display canvas at 800, 1200, 1600, 2000, and 2400 display pixels per second. A 450 ms blank interval separates passes. Equal-length payloads prevent a shorter template from matching only part of a longer barcode. For event matching, the current payload is compared with its group of three candidates; this limits ambiguity in the controlled demo.

### 2. Event acquisition and noise filtering

Metavision supplies asynchronous `(x, y, polarity, timestamp)` events from the SilkyCam. The program requests tuned ON/OFF sensitivity biases; the hardware may quantise the requested values to nearby supported settings.

Each incoming event passes two filters:

- Background-activity filter: an event must have a neighbouring event in a 3 x 3 area within the previous 2500 microseconds. This removes isolated hot-pixel activity.
- Refractory filter: another event from the same pixel within 180 microseconds is ignored. This reduces repeated ON/OFF firing from the same LCD transition.

The most recent timestamp and polarity are stored per pixel. The live image uses a 30 ms persistence window, while decoding uses the newest 12 ms to preserve sharp high-speed edges.

### 3. Event ROI and one-dimensional signal

The decoder counts recent activity in every image row and chooses the most active horizontal band, about 10% of image height. Restricting the scan band reduces perspective effects and LCD scan-out skew.

Events in that band are projected onto the X axis to make three smoothed signals:

- total event activity;
- ON-event activity;
- OFF-event activity.

This turns vertical barcode edges into peaks along a one-dimensional profile.

### 4. Event candidate matching and tracking

For each candidate Code 128 pattern, the program searches possible horizontal positions and module scales. It scores how well expected edges align with event peaks, whether ON/OFF polarity alternates correctly, how much of the pattern is covered, and whether unwanted activity appears between expected edges.

A single event slice may contain only part of the barcode, especially during fast motion or an LCD refresh. The temporal tracker therefore:

- slowly decays old evidence;
- adds confidence-weighted evidence from each new slice;
- requires at least three supporting observations;
- requires the leading candidate to exceed the runner-up by a margin;
- can also lock immediately on a very strong, unambiguous match;
- uses recent-match voting as another stable acceptance route.

The detected module scale is updated only from strong matches. A generic edge decoder and Code 128 checksum provide additional confirmation when a complete edge sequence is available.

### 5. RGB baseline

The RealSense runs independently in a capture thread at 640 x 480 and 60 fps using YUYV. Manual exposure control is shortened to improve its motion performance and make the comparison a strong RGB baseline.

Each RGB frame is converted to greyscale. Eleven horizontal scan lines are blurred slightly, thresholded with Otsu's method, and converted into alternating black/white run widths. The generic Code 128 decoder tests the standard symbol patterns in both directions and accepts a result only if its checksum is valid.

Unlike the controlled event matcher, the RGB decoder reads the barcode symbols from the image rather than selecting only from the current three event candidates.

### 6. Measurement and logging

A pass starts when a new barcode begins moving. Each pipeline records the first time it returns the correct displayed ID. A pass is successful only when the decoded string exactly matches that ID before it leaves the stimulus window.

The comparison CSV includes:

- pass number, expected payload, and display speed;
- event decode, success, confidence, and first-read time;
- RGB decode, success, first-read time, and frame count;
- cumulative event count and experiment elapsed time.

The program also saves the latest dashboard, final dashboard, and the dashboard with peak observed activity.

## Experiments performed and why

The archived logs document the development process. Intermediate and rejected tests are kept because they explain the final design decisions.

| Experiment/log | Result | Why it was performed and what we learned |
|---|---:|---|
| `rejected_mixed_length_trial.csv` | Event 8/24 | Mixed payload lengths allowed partial-pattern ambiguity. All demo IDs were changed to seven characters. |
| `rejected_eight_candidate_trial.csv` | Event 6/44 | Comparing too many visually similar candidates at once was unreliable. Candidates were divided into controlled groups of three. |
| `validation_pre_lock_19_of_24.csv` | Event 19/24 | Measured the event matcher before stronger confidence locking. |
| `validation_22_of_24.csv` | Event 22/24 | Showed that threshold and stability changes improved event reliability but misses remained. |
| `validation_18_of_20.csv` | Event 18/20 | Repeated event-only validation to expose intermittent high-speed misses. |
| `validation_20_of_20_three_codes.csv` | Event 20/20 | Confirmed that the three-candidate, equal-length configuration could be reliable. |
| `comparison_18_event_20_rgb.csv` | Event 18/20; RGB 20/20 | Established the first direct RGB comparison and showed the event tracker still needed improvement. |
| `comparison_before_temporal_tracker.csv` | Event 42/49; RGB 49/49 | Longer controlled baseline showing that hard per-slice event decisions lost incomplete fast patterns. |
| `comparison_threading_rejected.csv` | Event 1/15; RGB 15/15 | A threaded event-processing revision severely degraded the event result and was rejected rather than reported as an improvement. |
| `comparison_tracker_24_of_24.csv` | Event 24/24; RGB 24/24 | Short validation of confidence-weighted temporal evidence. |
| Final controlled run | Event 49/49; RGB 49/49 | Confirmed equal reliability across all five display speeds in this LCD setup. |

`tuning_trials.csv` and `trials.csv` contain accumulated exploratory event-only runs from multiple tuning stages, so they should not be treated as one controlled experiment. `artifacts/comparison_trials.csv` is also append-only and now contains later smoke tests after the first final 49 rows. The frozen, paper-facing summary of that first 49-pass block is `COMPARISON_RESULTS.md`.

## Why the temporal tracker mattered

Before temporal evidence accumulation, the event reader achieved 42/49 while RGB achieved 49/49. The event signal from an LCD is fragmented by the display's refresh and scan-out behaviour. Requiring one short event slice to contain a complete, high-confidence pattern discarded useful partial observations.

The final tracker combines those observations across time while still demanding repeated support and separation from the runner-up. A later 24-pass validation and the final 49-pass run both reached 100% for event and RGB.

## What the result does and does not prove

The experiment proves that:

- the SilkyCam and RealSense can be run and measured together;
- the event filtering, ROI selection, pattern matching, temporal tracking, RGB decoding, logging, and visualisation work as one live demo;
- the event pipeline can match RGB reliability for the tested barcodes and display speeds;
- selected-ROI event data and full-frame RGB data can be displayed live using clearly defined host-side data-rate metrics.

It does **not** prove that the event camera is a specific number of times faster than RGB. A 60 Hz LCD uses sample-and-hold frames: it displays a sharp barcode at a sequence of positions rather than producing truly continuous physical motion. This gives the RGB camera relatively sharp frames and does not reproduce conveyor-belt motion blur. Display speed in px/s also cannot be converted into a general conveyor speed without geometry and distance calibration.

The event pipeline is not yet fully generalised. It uses nine known, equal-length demo IDs and searches only a group of three at a time. Its tracker is designed for predominantly horizontal motion and the current camera/display geometry.

## Recommended paper experiment

For a defensible event-versus-RGB speed comparison:

1. Move printed barcodes with a conveyor, linear rail, or rotating drum.
2. Give both cameras the same target path, field of view, distance, angle, and illumination.
3. Measure physical velocity in m/s independently of either camera.
4. Randomise IDs and test unseen barcodes rather than selecting from a small known set.
5. Use several barcode sizes, contrasts, angles, directions, and lighting levels.
6. Record enough repetitions at each speed for confidence intervals.
7. Choose a reliability threshold in advance, for example 99% successful reads.
8. Measure algorithm time from sufficient sensor data to result, separately from travel into the field of view.

Only then report a speed ratio:

```text
event maximum physical speed at the chosen reliability threshold
----------------------------------------------------------------
 RGB maximum physical speed at the same reliability threshold
```

Future generalisation should include arbitrary Code 128 payloads, a larger and unseen test set, velocity estimation, scale/rotation tolerance, automatic exposure and bias sweeps, resource measurements, and recorded RAW datasets for repeatability.

## Repository layout

```text
CMakeLists.txt                         CMake build definition
run_demo.sh                            Build/environment/run launcher
src/main.cpp                           Cameras, filtering, tracking, UI, logging
src/code128.hpp                        Code 128 public interface
src/code128.cpp                        Generator and generic Code 128 decoders
metavision_sdk_get_started.cpp         Original Metavision starter example
README.md                              Complete setup and project explanation
COMPARISON_RESULTS.md                  Frozen final-result summary
event_camera_fast_barcode_reader.md    Original research/design plan
artifacts/                             CSV evidence and diagnostic images
artifacts_plot_test/                   Optional data-rate plot test output
```

Generated `build/` files are deliberately not included in the source archive because they are machine-specific and are recreated by `run_demo.sh`.

## Troubleshooting

### RealSense fails after a restart

Current code automatically locates the RealSense YUYV colour stream even if Linux changes its device number. If necessary, inspect the video devices and temporarily force one with:

```bash
./run_demo.sh --rgb-device 4
```

### Barcode appears on the laptop instead of the external monitor

Check the monitor's X coordinate with `xrandr --current`, then pass it explicitly:

```bash
./run_demo.sh --stimulus-x 3840
```

The launcher selects `QT_QPA_PLATFORM=xcb` because native Wayland window placement does not reliably honour absolute monitor coordinates.

### Run only one side

For event-only capture:

```bash
./run_demo.sh --no-rgb
```

For display/stimulus testing without cameras:

```bash
./run_demo.sh --stimulus-only
```

### Rebuild from scratch

The normal launcher rebuilds changed files automatically. If a clean build is required, remove only the local `build/` directory and run `./run_demo.sh` again.
