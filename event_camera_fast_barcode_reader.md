# Fast Event-Based Barcode Reader

## Project Overview

This project develops a real-time **high-speed barcode reader using an
event camera**. The objective is to demonstrate that event-based vision
can identify barcodes under rapid relative motion where a conventional
RGB camera begins to suffer from motion blur.

The initial prototype will use a **CenturyArks SilkyCam VGA event camera
(640 × 480)** with the **Metavision SDK**, with the option to deploy the
final pipeline on a Raspberry Pi 5.

## Core Research Question

> How fast can a barcode travel while remaining readable using
> asynchronous events, compared with a conventional frame-based camera?

## Demo Concept

A barcode moves rapidly across the field of view of a fixed event
camera.

The processing pipeline is:

**Moving Barcode → Event Stream → Event Filtering → Barcode ROI
Detection → Event-Based Representation/Signal → Barcode Decoding → ID +
Latency + Speed**

The demonstration should progressively increase barcode speed and
compare event-camera performance with a conventional RGB camera.

## Why Event Cameras?

Conventional cameras expose an entire image at a fixed frame rate. At
high relative motion, barcode edges can become blurred during exposure.

An event camera instead reports changes in brightness asynchronously.
Barcode edges moving across the sensor therefore generate strong ON/OFF
event patterns with very high temporal resolution.

This makes barcode reading a useful demonstration of event-based vision
for:

-   High-speed conveyor systems
-   Industrial sorting
-   Package tracking
-   Logistics
-   Robotic perception
-   High-speed identification
-   Low-latency machine vision

## Initial Target

Start with **1D barcodes** rather than QR codes.

1D barcodes are particularly suitable because alternating black and
white bars produce structured ON/OFF event sequences when moving across
the sensor.

The first target should be:

> Reliably decode one 1D barcode moving horizontally through the camera
> field of view at progressively increasing speeds.

## Hardware

### Required

-   CenturyArks SilkyCam VGA event camera
-   USB connection to development computer
-   Ubuntu 24.04 workstation/laptop
-   Printed 1D barcodes
-   Controlled lighting
-   Mechanism for moving the barcode

### Optional

-   Raspberry Pi 5
-   RGB camera for baseline comparison
-   Conveyor belt or rotating wheel
-   Linear rail or motorised slider
-   High-speed display for showing barcode ID and latency

## Software

-   Ubuntu 24.04
-   Metavision SDK 5
-   Python or C++
-   OpenCV
-   NumPy
-   Barcode decoding library for the reconstructed-image baseline
-   Logging/visualisation utilities

C++ should ultimately be preferred for the real-time embedded
demonstration, although Python is suitable for rapid prototyping.

## Development Strategy

### Stage 1: Capture Events

Connect the SilkyCam and verify live event acquisition.

Record event tuples:

`(x, y, polarity, timestamp)`

Save short RAW recordings at several barcode speeds for repeatable
offline development.

### Stage 2: Visualise the Moving Barcode

Accumulate events over a short temporal window, initially around:

-   1 ms
-   2 ms
-   5 ms
-   10 ms

Generate an event image in which:

-   ON events represent increasing brightness
-   OFF events represent decreasing brightness

Determine which temporal window preserves barcode structure best at each
speed.

### Stage 3: Barcode Region Detection

Automatically identify the barcode region.

Possible methods include:

-   Event-density thresholding
-   Connected components
-   Spatial histogram analysis
-   Line/edge orientation
-   Activity clustering

The output should be a cropped Region of Interest (ROI).

### Stage 4: Baseline Image Reconstruction

Create a simple event-frame representation from the ROI.

Example:

`event_image[y, x] += polarity`

Normalise the resulting representation and pass it to a conventional
barcode decoder.

This is the fastest route to establishing a working proof-of-concept.

### Stage 5: Direct Event-Based Barcode Decoding

The more interesting approach is to avoid reconstructing a conventional
image.

For horizontal barcode motion, select one or several scan lines through
the barcode ROI.

As alternating dark and bright bars cross these locations, they generate
temporal ON/OFF event transitions.

Pipeline:

**Events → Scan Line → Temporal Event Signal → Edge Transitions →
Bar/Space Width Estimation → Barcode Symbol Decoding**

This makes the system genuinely event-driven.

### Stage 6: Motion Estimation

Estimate barcode velocity from the event stream.

Possible approaches:

-   Track leading/trailing barcode edges
-   Event optical flow
-   Time difference between spatial locations
-   Plane fitting in x-y-t event space

Velocity can then be used to convert temporal intervals between
transitions into spatial barcode widths.

This is particularly useful for direct asynchronous decoding.

## Proposed Direct Decoder

Assume the barcode moves predominantly along the x-axis.

For a selected row `y0`, collect events close to:

`y = y0`

Sort events by timestamp.

The resulting sequence can be represented as:

`t1 → ON` `t2 → OFF` `t3 → ON` `t4 → OFF` `...`

Calculate:

`Δt_i = t_(i+1) - t_i`

If barcode velocity is `v`, approximate the corresponding bar width as:

`w_i = v × Δt_i`

Normalise the measured widths against the narrowest expected barcode
element.

The resulting narrow/wide or module-width sequence can then be mapped to
the barcode encoding standard.

Multiple scan lines should be combined using voting or confidence
weighting to improve robustness.

## First Barcode Standard

A practical starting point is **Code 128** because it is common in
logistics and industrial applications.

Alternative initial standards:

-   Code 39
-   EAN-13
-   UPC-A

The first implementation should support only one standard to keep
development fast.

## RGB Comparison

A strong demonstration should use an RGB camera observing the same
moving barcode.

For each speed, record:

-   RGB decoding success
-   Event-camera decoding success
-   RGB image/frame
-   Event representation
-   Processing latency

Increase speed until RGB decoding begins to fail because of motion blur.

Continue increasing speed to determine the event-camera operating limit.

## Metrics

Measure:

### Detection Rate

`Detection Rate = Successful Reads / Total Passes`

### End-to-End Latency

Measure time from sufficient barcode events becoming available to
decoded barcode ID output.

### Maximum Readable Speed

Determine the highest barcode velocity at which the system maintains a
predefined reliability level.

### Event Count

Record the number of events required for each successful decode.

### Computational Cost

Measure:

-   CPU utilisation
-   Memory usage
-   Processing time
-   Event throughput

### Optional Power Measurement

For Raspberry Pi deployment, measure system power consumption during
decoding.

## Experimental Variables

Test performance against:

-   Barcode speed
-   Barcode size
-   Distance from camera
-   Illumination
-   Motion direction
-   Rotation angle
-   Event accumulation window
-   Contrast
-   Background complexity

## Demo Interface

The live demonstration can display:

**Event Camera**

-   Live event visualisation
-   Detected barcode ROI
-   Decoded barcode ID
-   Estimated velocity
-   Processing latency
-   Current event rate
-   Read success indicator

**RGB Camera**

-   Live RGB frame
-   Decoded barcode ID or FAIL
-   Exposure/frame rate
-   Processing latency

A speed indicator can show the point at which the conventional camera
stops decoding reliably while the event-based system continues.

## Development Timeline

### Day 1: Basic Proof-of-Concept

Goal: make the barcode visible and potentially readable.

Tasks:

-   Connect SilkyCam
-   Capture event stream
-   Record moving barcode
-   Generate 1--10 ms event frames
-   Tune camera biases if required
-   Extract barcode ROI
-   Attempt conventional barcode decoding from event frames

**Expected outcome:** visible barcode structure from moving events.

### Day 2: Automatic Decoding

Tasks:

-   Automatic barcode localisation
-   ROI tracking
-   Improve event accumulation
-   Integrate barcode decoder
-   Log decoding results
-   Test different speeds

**Expected outcome:** automatic live barcode identification.

### Day 3: High-Speed Optimisation

Tasks:

-   Implement temporal/direct event decoder
-   Estimate barcode velocity
-   Optimise event filtering
-   Reduce latency
-   Test increasing motion speed
-   Add RGB comparison

**Expected outcome:** clear event-camera advantage under rapid motion.

### Day 4: Polished Demonstration

Tasks:

-   Build live visualisation
-   Display ID, velocity and latency
-   Run repeated trials
-   Calculate detection rate
-   Produce comparison plots/results
-   Prepare reproducible demonstration

**Expected outcome:** presentation-ready high-speed barcode
demonstrator.

## Minimum Viable Demo

The minimum successful demonstration is:

1.  Print a Code 128 barcode.
2.  Move it horizontally in front of the SilkyCam.
3.  Capture events.
4.  Generate a short-window event representation.
5.  Automatically locate the barcode.
6.  Decode its ID.
7.  Display the decoded value in real time.

## Strong Demo

A stronger version adds:

1.  Event camera and RGB camera observing the same barcode.
2.  Controlled increase in barcode velocity.
3.  Real-time velocity measurement.
4.  Live read-success indicators.
5.  Latency measurement.
6.  A speed at which RGB decoding fails while event decoding continues.

## Advanced Version

Once the basic demonstration works:

-   Eliminate event-frame reconstruction.
-   Decode directly from asynchronous event timestamps.
-   Deploy processing on Raspberry Pi 5.
-   Investigate adaptive temporal windows.
-   Support arbitrary motion direction.
-   Support multiple simultaneous barcodes.
-   Extend to QR/Data Matrix codes.
-   Explore event-driven neural or spiking decoding.
-   Integrate the reader with a robotic/conveyor sorting task.

## Potential Research Direction

The most scientifically interesting version is not simply:

> "Can an event camera read a barcode?"

Instead, investigate:

> "Can barcode information be decoded directly from asynchronous
> spatio-temporal events without reconstructing conventional image
> frames?"

This allows investigation of:

-   Event-native feature extraction
-   Motion-aware asynchronous decoding
-   Latency versus accuracy
-   Event efficiency
-   Maximum readable velocity
-   Comparison with frame-based sensing
-   Edge/embedded deployment

## Success Criteria

### MVP

-   Live event capture
-   Automatic barcode localisation
-   Correct barcode decoding
-   Real-time operation

### Good Demonstrator

-   95% reads under selected test conditions

-   Real-time velocity estimate

-   Live latency measurement

-   RGB baseline

-   Demonstrated high-speed operating regime

### Research-Grade Prototype

-   Direct asynchronous decoding
-   Quantitative RGB comparison
-   Controlled speed experiments
-   Multiple environmental conditions
-   Embedded deployment
-   Detailed latency/event/energy analysis

## Immediate Next Experiment

Print several Code 128 barcodes containing known IDs.

Fix the SilkyCam in place and move the barcode horizontally across the
field of view. Record approximately 10--20 passes at slow, medium and
fast speeds.

Before developing a complete decoder, inspect whether the barcode
produces stable and distinguishable ON/OFF event patterns.

If the patterns are clean, proceed directly towards the asynchronous
temporal decoder.

------------------------------------------------------------------------

## Project Summary

**Project:** Fast Event-Based Barcode Reader\
**Sensor:** CenturyArks SilkyCam VGA, 640 × 480\
**Initial target:** Code 128 1D barcode\
**Platform:** Ubuntu + Metavision SDK\
**Embedded target:** Raspberry Pi 5\
**MVP development time:** approximately 1 day\
**Polished demonstration:** approximately 2--4 days\
**Core novelty:** high-speed, low-latency barcode decoding from
asynchronous visual events, with the longer-term goal of direct
event-native decoding without conventional frame reconstruction.
