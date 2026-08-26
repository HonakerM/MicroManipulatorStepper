#!/usr/bin/env python3
"""Record accelerometer data from a Phidget and run an FFT on a CSV.

Usage:
    python accel_tool.py record --output run1.csv
    python accel_tool.py fft run1.csv --output run1_fft.csv
"""

import csv
import threading
from pathlib import Path

import typer

app = typer.Typer(add_completion=False, help="Phidget accelerometer recorder + FFT analysis.")


# --------------------------------------------------------------------------
# Command 1: record
# --------------------------------------------------------------------------
@app.command()
def record(
    output: Path = typer.Option(Path("acceleration.csv"), "--output", "-o", help="CSV file to write."),
    interval: int = typer.Option(8, "--interval", "-i", help="Requested sample interval in ms."),
    change_trigger: float = typer.Option(
        0.0,
        "--change-trigger",
        help="Minimum change in g before the device reports a sample. 0 reports every sample.",
    ),
    channel: int = typer.Option(0, "--channel", "-c", help="Phidget channel number."),
    serial: int = typer.Option(-1, "--serial", "-s", help="Device serial number (-1 = any)."),
    timeout: int = typer.Option(5000, "--timeout", help="Attach timeout in ms."),
):
    """Stream x/y/z acceleration to a CSV until you press Enter."""
    from Phidget22.Devices.Accelerometer import Accelerometer
    from Phidget22.PhidgetException import PhidgetException

    fh = open(output, "w", newline="")
    writer = csv.writer(fh)
    writer.writerow(["time_s", "accel_x", "accel_y", "accel_z"])

    lock = threading.Lock()
    state = {"t0": None, "count": 0, "last_t": 0.0}

    def on_acceleration_change(_channel, acceleration, timestamp):
        # timestamp comes from the device in milliseconds
        with lock:
            if state["t0"] is None:
                state["t0"] = timestamp
            t = (timestamp - state["t0"]) / 1000.0
            writer.writerow([f"{t:.6f}", *(f"{a:.6f}" for a in acceleration)])
            state["count"] += 1
            state["last_t"] = t
            if state["count"] % 100 == 0:
                fh.flush()

    accel = Accelerometer()
    if serial >= 0:
        accel.setDeviceSerialNumber(serial)
    accel.setChannel(channel)
    accel.setOnAccelerationChangeHandler(on_acceleration_change)

    try:
        print("Waiting for Phidget accelerometer...")
        accel.openWaitForAttachment(timeout)

        # 0.0 reports every sample; larger values suppress samples that barely moved.
        try:
            t_lo = accel.getMinAccelerationChangeTrigger()
            t_hi = accel.getMaxAccelerationChangeTrigger()
            trigger = min(max(change_trigger, t_lo), t_hi)
            accel.setAccelerationChangeTrigger(trigger)
            if trigger != change_trigger:
                print(f"Change trigger clamped to {trigger} g (device allows {t_lo}-{t_hi} g).")
            if trigger > 0:
                print(
                    f"Change trigger is {trigger} g: samples below that threshold are dropped, "
                    "so the CSV will have gaps and the FFT may show spurious harmonics."
                )
        except PhidgetException:
            pass

        lo, hi = accel.getMinDataInterval(), accel.getMaxDataInterval()
        actual = min(max(interval, lo), hi)
        accel.setDataInterval(actual)
        if actual != interval:
            print(f"Interval clamped to {actual} ms (device allows {lo}-{hi} ms).")

        print(f"Recording to {output} at ~{1000 / actual:.1f} Hz. Press Enter to stop.")
        try:
            input()
        except KeyboardInterrupt:
            pass
    except PhidgetException as e:
        print(f"Phidget error: {e.details}")
        raise typer.Exit(code=1)
    finally:
        try:
            accel.close()
        except PhidgetException:
            pass
        with lock:
            fh.flush()
            fh.close()

    n, dur = state["count"], state["last_t"]
    rate = n / dur if dur > 0 else 0.0
    print(f"Saved {n} samples over {dur:.2f} s ({rate:.1f} Hz effective) to {output}")


# --------------------------------------------------------------------------
# Command 2: fft
# --------------------------------------------------------------------------
@app.command()
def fft(
    input_csv: Path = typer.Argument(..., exists=True, readable=True, help="CSV whose first column is time."),
    output: Path = typer.Option(None, "--output", "-o", help="Where to write the spectrum (default: <input>_fft.csv)."),
    remove_mean: bool = typer.Option(True, "--remove-mean/--keep-mean", help="Subtract the mean before transforming."),
    resample: bool = typer.Option(True, "--resample/--no-resample", help="Interpolate onto a uniform time grid first."),
    max_freq: float = typer.Option(None, "--max-freq", help="Only keep frequencies up to this value (Hz)."),
):
    """Run an FFT on every column of a CSV, using the first column as timestamps."""
    import numpy as np

    names, data = _load_csv(input_csv)
    if data.shape[1] < 2:
        print("Need at least a time column and one data column.")
        raise typer.Exit(code=1)

    t = data[:, 0]
    signals = data[:, 1:]
    labels = names[1:]

    order = np.argsort(t)
    t, signals = t[order], signals[order]

    n = len(t)
    if n < 2:
        print("Not enough rows to transform.")
        raise typer.Exit(code=1)

    diffs = np.diff(t)
    dt = float(np.median(diffs))
    if dt <= 0:
        print("Timestamps are not increasing.")
        raise typer.Exit(code=1)

    jitter = float(np.std(diffs) / dt)
    if resample and jitter > 0.01:
        grid = np.arange(t[0], t[-1], dt)
        signals = np.column_stack([np.interp(grid, t, signals[:, i]) for i in range(signals.shape[1])])
        n = len(grid)
        print(f"Sampling jitter {jitter:.1%}; resampled onto {n} uniform points.")

    fs = 1.0 / dt
    if remove_mean:
        signals = signals - signals.mean(axis=0)

    spectrum = np.fft.rfft(signals, axis=0)
    freqs = np.fft.rfftfreq(n, d=dt)

    # Single-sided amplitude: double every bin except DC (and Nyquist when n is even).
    amps = np.abs(spectrum) * (2.0 / n)
    amps[0] /= 2.0
    if n % 2 == 0:
        amps[-1] /= 2.0

    if max_freq is not None:
        keep = freqs <= max_freq
        freqs, amps = freqs[keep], amps[keep]

    out_path = output or input_csv.with_name(input_csv.stem + "_fft.csv")
    with open(out_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["frequency_hz"] + [f"{label}_amplitude" for label in labels])
        for i, fr in enumerate(freqs):
            w.writerow([f"{fr:.6f}"] + [f"{a:.8f}" for a in amps[i]])

    print(f"Sample rate {fs:.2f} Hz, {n} samples, resolution {freqs[1] - freqs[0]:.4f} Hz")
    for j, label in enumerate(labels):
        search = amps[1:, j] if len(freqs) > 1 else amps[:, j]
        offset = 1 if len(freqs) > 1 else 0
        peak = int(np.argmax(search)) + offset
        print(f"  {label}: peak {freqs[peak]:.3f} Hz (amplitude {amps[peak, j]:.4f})")
    print(f"Wrote {len(freqs)} frequency bins to {out_path}")


def _load_csv(path: Path):
    """Return (column_names, float array). Handles files with or without a header."""
    import csv
    import numpy as np

    with open(path, newline="") as f:
        rows = [r for r in csv.reader(f) if r]
    if not rows:
        print("Empty CSV.")
        raise typer.Exit(code=1)

    try:
        [float(v) for v in rows[0]]
        header_present = False
    except ValueError:
        header_present = True

    if header_present:
        raw_names = rows[0]
        data_rows = rows[1:]
    else:
        raw_names = ["time"] + [f"col{i}" for i in range(1, len(rows[0]))]
        data_rows = rows

    # Ensure all data rows match the expected width before transposing
    width = len(raw_names)
    valid_data_rows = [r for r in data_rows if len(r) == width]

    if not valid_data_rows and not header_present:
        print("No valid data rows found.")
        raise typer.Exit(code=1)

    # Transpose data to inspect columns: list of tuples, where each tuple is a column
    columns = list(zip(*valid_data_rows))

    keep_indices = []
    for i, col in enumerate(columns):
        # Check if the column has at least one non-empty, non-whitespace string
        if any(v.strip() for v in col):
            keep_indices.append(i)

    if not keep_indices:
        print("All columns are empty.")
        raise typer.Exit(code=1)

    # Filter names and columns based on the indices we want to keep
    filtered_names = [
        (raw_names[i].strip() or f"col{i}") if header_present else raw_names[i]
        for i in keep_indices
    ]
    
    filtered_columns = [columns[i] for i in keep_indices]

    # Re-transpose back to rows
    filtered_rows = list(zip(*filtered_columns))

    # Convert to float array, skipping rows that fail parsing
    values = []
    for r in filtered_rows:
        try:
            values.append([float(v) for v in r])
        except ValueError:
            continue

    if not values:
        print("No numeric rows found after filtering.")
        raise typer.Exit(code=1)

    return filtered_names, np.asarray(values, dtype=float)

if __name__ == "__main__":
    app()