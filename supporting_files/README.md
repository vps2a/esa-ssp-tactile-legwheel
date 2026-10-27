# LegWheel Supporting Tools

This directory contains standalone analysis tools that do not participate in
online ROS processing or rosbag recording:

- `kinematics_calculator.ipynb` verifies the symbolic robot kinematics;
- `dataset_browser.py` is a read-only desktop browser for derived packet
  datasets.

## Requirements

- [uv](https://docs.astral.sh/uv/getting-started/installation/)
- Python 3.12 (managed automatically by `uv`)
- VS Code with Microsoft’s **Python** and **Jupyter** extensions, if using the
  notebook

## Installation

From the repository root:

```bash
cd supporting_files
uv sync
```

This creates or updates `supporting_files/.venv` from the exact versions in
`uv.lock`. PySide6, NumPy and Matplotlib required by the dataset browser are
installed by this command as well.

To add or update a dependency during development, use `uv add` rather than
calling `pip` directly:

```bash
cd supporting_files
uv add <package-name>
```

## Dataset browser

### Starting the browser

From the repository root:

```bash
cd supporting_files
uv sync
uv run python dataset_browser.py
```

The installed console-script equivalent is:

```bash
cd supporting_files
uv run legwheel-dataset-browser
```

An explicit dataset may be supplied when scripting or testing:

```bash
uv run python dataset_browser.py \
  --dataset /path/to/run_1/derived/CONFIGURATION_HASH
```

The directory picker expects the configuration-hash directory itself. It must
contain `provenance.json`, `manifest.jsonl`, and `packets/`. The last valid
selection is remembered in the operating system's application settings. Use
**Change dataset…** at the top-right to select another one.

### Controls and views

- Click **Previous packet** and **Next packet**, or press the left/right arrow
  keys, to move through the chronologically ordered manifest. Navigation stops
  at the first and last packet.
- To open a packet directly, enter its 1-based position in the **Go to** field
  at the bottom and click **Jump to packet** or press **Enter**. For example,
  entering `64` opens the 64th packet in the sorted list. Invalid or
  out-of-range values leave the current packet unchanged and show an inline
  explanation.
- The title identifies the experiment and run. The lower navigation label shows
  the current packet number and packet ID.
- **Images and patches** displays the complete RGB and metric depth images with
  the physical patch highlighted. **Show cropped masked patches** switches both
  images to the smallest crop containing that patch.
- Depth uses fixed dataset-wide limits by default, so colours remain comparable
  between packets. Per-packet automatic scaling is available as an explicit
  checkbox.
- The bold along-travel and cross-track slopes are the primary Stage 2 outputs.
  Plane-fit diagnostics appear beside them.
- Motor and joint signals are plotted on the left; IMU signals are plotted on
  the right. The synchronized tabs select the local contact window, the longer
  spectral window, or the stored FFT power spectrum.
- **3D diagnostic** shows the depth point map, RANSAC inliers, physical patch
  boundaries, and fitted plane normal.
- **Metadata** lists timestamps, synchronization, angles, contact timing,
  travel directions, and plane-fit quality.

Stage 1-only packets remain inspectable. Their images, masks, metadata and
available telemetry are displayed, while slope, FFT and 3D views clearly state
that Stage 2 is unavailable.

### Data safety

The browser is strictly read-only. It reads `.json`, `.jsonl`, `.npy` and
`.npz` files without writing into the selected dataset. The remembered dataset
path is stored in the normal per-user Qt application settings outside the
dataset directory.

### Troubleshooting

If Qt cannot connect to the graphical display, start the program from a desktop
terminal in the Ubuntu VM rather than an SSH session. On Linux, missing Qt
system-library errors should be resolved through the distribution package
manager; do not install a second Python environment over `.venv`.

## Running the kinematics notebook

1. Open an `.ipynb` file in VS Code.
2. Click **Select Kernel → Python Environments**.
3. Select `supporting_files/.venv/bin/python`.
4. Run cells with **Shift+Enter**.

No separate Jupyter server or manual environment activation is required.

## Development and checks

Run the standalone unit tests without requiring ROS:

```bash
cd supporting_files
uv run python -m unittest discover -s tests -v
```

Commit source files, `pyproject.toml`, and `uv.lock`. Keep `.venv/` and
`.ipynb_checkpoints/` out of Git.

Before committing, restart the kernel and run all cells to verify reproducibility.

This environment is intended for standalone calculations. Notebooks importing ROS packages require a compatible ROS environment.
