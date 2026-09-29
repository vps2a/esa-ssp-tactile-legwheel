# Kinematics Notebooks

Jupyter notebooks for symbolic robot kinematics using SymPy, with NumPy and Matplotlib for numerical evaluation and plotting.

## Requirements

- [uv](https://docs.astral.sh/uv/getting-started/installation/)
- VS Code with Microsoft’s **Python** and **Jupyter** extensions

## Installation

From the repository root:

```bash
cd supporting_files
uv sync
```

This installs the dependencies specified in `pyproject.toml` and `uv.lock` into a local `.venv`.

If setting up this directory for the first time and these files do not yet exist, run instead:

```bash
cd notebooks
uv init --bare --python 3.12
uv add sympy numpy matplotlib
uv add --dev ipykernel
```

## Running a Notebook

1. Open an `.ipynb` file in VS Code.
2. Click **Select Kernel → Python Environments**.
3. Select `supporing_files/.venv/bin/python`.
4. Run cells with **Shift+Enter**.

No separate Jupyter server or manual environment activation is required.

## Development

Add dependencies from this directory:

```bash
uv add <package>
```

Commit notebooks, `pyproject.toml`, and `uv.lock`. Keep `.venv/` and `.ipynb_checkpoints/` out of Git.

Before committing, restart the kernel and run all cells to verify reproducibility.

This environment is intended for standalone calculations. Notebooks importing ROS packages require a compatible ROS environment.
