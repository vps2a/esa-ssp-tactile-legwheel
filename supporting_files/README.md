# LegWheel Supporting Tools

This directory contains standalone analysis tools that do not participate in
online ROS processing or rosbag recording:

- `kinematics_calculator.ipynb` verifies the symbolic robot kinematics;
- `dataset_browser.py` is a desktop browser for inspecting derived packets
  and manually classifying their terrain patches.
- `batch_dataset_building_tool.py` builds datasets for selected runs across
  multiple recorded experiments through a desktop wizard.

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

### Manual terrain classification

The row above the plots contains four class buttons:

| Class | Highlighted terrain patch |
| --- | --- |
| 0 | Firm Terrain |
| 1 | Loose Terrain |
| 2 | Firm to Loose Transition |
| 3 | Loose to Firm Transition |

**Manual Classification Mode** is off when the browser starts and when another
dataset is opened. The buttons are disabled until it is enabled. Enabling it
shows one confirmation warning that subsequent selections will manually
overwrite previous classifications; enabling the mode alone changes no labels.

Click a class button or press **0**, **1**, **2** or **3** to save the current
packet's classification immediately and advance to the next packet. Number
shortcuts are ignored while typing in **Go to**, and holding a number does not
classify multiple packets. Manual mode remains enabled while navigating within
the current dataset. At the last packet, a selection saves and stays there.

An existing class is highlighted in light blue, including while manual mode is
off. The **Metadata** tab shows the class, classification method and model ID.
No automatic classifier is run by the browser.

### Data safety

Opening an older dataset adds missing `class_id`, `classification_method` and
`model_id` fields to every packet and existing feature record. Unclassified
fields are `null`; existing labels are preserved. This upgrade also runs with
manual mode off and requires write access to the selected dataset.

Each manual selection synchronizes `metadata.json`, `stage2/features.json` and
the matching dataset-wide `features.jsonl` row. A manual classification records
`classification_method: "manual"` and `model_id: null`. Stage 1-only packets
update metadata; Stage 2 later carries the label into feature files. Writes use
complete temporary files followed by replacement, with rollback if a save
fails. The browser only advances after the save succeeds.

Images, numerical arrays and the manifest remain unchanged. The remembered
dataset path is stored in the normal per-user Qt application settings outside
the dataset directory; manual mode is never remembered.

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

## Batch dataset builder

Run this tool on Ubuntu with ROS 2 Jazzy and the built workspace sourced.
From the repository root:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
cd supporting_files
uv sync
uv run python batch_dataset_building_tool.py
```

The installed command is `uv run legwheel-batch-dataset-builder`. Optionally
provide `--directory /path/to/Experiment_Recordings` to start with a directory
already loaded. The GUI uses the supporting-tools environment; each analysis
runs under `/usr/bin/python3` so it can load the sourced ROS libraries. For a
different ROS Python installation, pass `--ros-python /path/to/python3`.
Rebuild/source `legwheel_dataset` after updating this repository so that the
worker sees the current processing code.

1. Choose a directory containing experiment folders as immediate children.
   Experiment YAML files are recognized by their contents, including the
   experiment ID and geometry/hardware mappings, rather than their filenames.
2. Select experiments, then choose runs on a separate page for each experiment.
   Runs are ordered numerically. **Select all available** and **Clear all**
   controls are provided for both experiments and runs.
3. Each run shows **Completed Datasets: N**. **Partial Datasets** and **Failed
   Attempts** appear only when their counts are nonzero. A completed dataset
   finished Stage 2; Stage 1-only or otherwise incomplete outputs are partial.
   Failed and cancelled attempts are counted as failures.
4. Invalid and unreviewed runs remain selectable with a warning. **Mark valid**
   immediately creates or updates `run_valid: true` in `run_config.yaml`,
   preserving its other values. This is saved even if no batch is started.
   Runs missing required inputs remain visible but cannot be selected.
5. Edit the shared settings, initially populated from the builder's defaults.
   All 16 numeric settings are available, with units and hover explanations.
   **Start batch** validates them and saves the same commented, timestamped
   `postprocess.yaml` in every selected run, replacing previous settings.
6. The tool processes one run at a time using its recorded experiment snapshot
   and the production packet/feature stages. Outputs use the normal
   `run_N/derived/CONFIGURATION_HASH/` location.

The progress screen shows overall 0–100% progress, the current experiment/run,
its own progress bar and processing phase, and an approximate remaining time.
The ETA initially says **Estimating…** and updates from observed processing
speed. Each run contributes equally to overall progress; longer runs can take
more time despite contributing the same share of the percentage.

A completed dataset matching the selected settings is skipped and listed in
the final summary. Before retrying an incomplete or failed output at the normal
location, that folder is renamed with an `_attempt_TIMESTAMP_UNIQUEID` suffix.
Its files and any classifications are preserved. The tool never deletes an
earlier dataset or changes the raw recording.

Individual failures do not stop later runs. **Cancel batch** immediately kills
the active analysis process, preserves its partial output and records its
cancelled status in `batch_build_status.json`. Later runs are left unstarted;
completed runs remain available. Closing the window during a build performs
the same cancellation. If cancellation happens while settings are being saved,
only the settings files already written are changed.

The final table lists each run as completed, skipped, failed, cancelled or not
started, with its output path and explanatory message. **New batch** refreshes
dataset counts. **Show processing log** exposes detailed errors when needed.
