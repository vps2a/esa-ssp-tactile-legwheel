"""Interactive Matplotlib diagnostics for one validation midpoint pair."""

from typing import Any

import numpy as np

from legwheel_dataset.bag_reader import extract_selected_images
from legwheel_dataset.images import decode_image
from legwheel_dataset.processor import ValidationCameraView, ValidationPreview


def _decode_display_image(message, *, rgb: bool) -> np.ndarray:
    """Decode one recorded image and normalize RGB channel ordering."""
    image = decode_image(message)
    encoding = str(message.encoding).lower()
    if rgb and encoding == "bgr8":
        return image[:, :, ::-1]
    if rgb and encoding == "bgra8":
        return image[:, :, [2, 1, 0, 3]]
    return image


def load_preview_images(
    preview: ValidationPreview,
) -> tuple[dict[str, np.ndarray] | None, str | None]:
    """Load only the selected RGB and depth messages in one bounded bag pass."""
    images: dict[str, np.ndarray] = {}

    def capture(label: str, *, rgb: bool):
        def callback(message) -> None:
            images[label] = _decode_display_image(message, rgb=rgb)

        return callback

    requests = {
        (preview.rgb.image_topic, preview.rgb.metadata.timestamp_ns): capture(
            "rgb", rgb=True
        ),
        (
            preview.depth.image_topic,
            preview.depth.metadata.timestamp_ns,
        ): capture("depth", rgb=False),
    }
    found = extract_selected_images(preview.bag_directory, requests)
    missing = set(requests) - found
    if missing:
        missing_text = ", ".join(
            f"{topic}@{timestamp_ns}" for topic, timestamp_ns in sorted(missing)
        )
        return None, f"selected_images_not_found_in_bag: {missing_text}"
    return images, None


def _depth_display(image: np.ndarray) -> tuple[np.ndarray, dict[str, Any]]:
    """Mask invalid depth and choose a readable robust display range."""
    depth = image.astype(float)
    invalid = ~np.isfinite(depth) | (depth <= 0.0)
    masked = np.ma.masked_where(invalid, depth)
    finite = depth[~invalid]
    options: dict[str, Any] = {"cmap": "viridis"}
    if finite.size:
        lower, upper = np.percentile(finite, [2.0, 98.0])
        if upper > lower:
            options.update(vmin=float(lower), vmax=float(upper))
    return masked, options


def _patch_polygon(view: ValidationCameraView) -> np.ndarray:
    polygon = np.vstack((
        view.patch.inner_pixels,
        view.patch.outer_pixels[::-1],
    ))
    if polygon.size:
        polygon = np.vstack((polygon, polygon[0]))
    return polygon


def _patch_centre_pixel(view: ValidationCameraView) -> np.ndarray:
    centre_index = view.patch.inner_pixels.shape[0] // 2
    return (
        view.patch.inner_pixels[centre_index]
        + view.patch.outer_pixels[centre_index]
    ) / 2.0


def _finite_rows(values: np.ndarray) -> np.ndarray:
    values = np.asarray(values, dtype=float)
    return values[np.all(np.isfinite(values), axis=1)]


def _expanded_limits(view: ValidationCameraView) -> tuple[float, ...]:
    """Include the camera rectangle and every finite configured-patch point.

    Invalid full-circle samples can approach the camera plane and project
    millions of pixels away. Letting those numerical asymptotes set the axes
    would make the requested patch unreadable, so the patch defines the
    diagnostic expansion while track samples are drawn where they intersect it.
    """
    width = float(view.metadata.width)
    height = float(view.metadata.height)
    groups = [
        np.asarray([[0.0, 0.0], [width, height]]),
        view.patch.inner_pixels,
        view.patch.outer_pixels,
    ]
    finite_groups = [_finite_rows(group) for group in groups]
    points = np.vstack([group for group in finite_groups if group.size])
    minimum = np.min(points, axis=0)
    maximum = np.max(points, axis=0)
    span = np.maximum(maximum - minimum, [width, height])
    padding = np.maximum(span * 0.05, 10.0)
    return (
        float(minimum[0] - padding[0]),
        float(maximum[0] + padding[0]),
        float(minimum[1] - padding[1]),
        float(maximum[1] + padding[1]),
    )


def _draw_image(ax, image: np.ndarray, view: ValidationCameraView) -> None:
    extent = (0.0, view.metadata.width, view.metadata.height, 0.0)
    if view.label == "Depth":
        displayed, options = _depth_display(image)
        ax.imshow(displayed, extent=extent, origin="upper", **options)
    else:
        cmap = "gray" if image.ndim == 2 else None
        ax.imshow(image, extent=extent, origin="upper", cmap=cmap)


def _draw_overlay(ax, view: ValidationCameraView, *, expanded: bool) -> None:
    """Draw full tracks, patch, centre, invalid points and camera boundary."""
    from matplotlib.patches import Rectangle

    for pixels, valid, color, label in (
        (
            view.inner_track_pixels,
            view.inner_track_valid,
            "#00bcd4",
            "Inner track",
        ),
        (
            view.outer_track_pixels,
            view.outer_track_valid,
            "#ff9800",
            "Outer track",
        ),
    ):
        if np.any(valid):
            ax.scatter(
                pixels[valid, 0],
                pixels[valid, 1],
                s=10,
                c=color,
                marker=".",
                label=label,
                zorder=4,
            )
        invalid = ~valid & np.all(np.isfinite(pixels), axis=1)
        if expanded and np.any(invalid):
            ax.scatter(
                pixels[invalid, 0],
                pixels[invalid, 1],
                s=8,
                c=color,
                marker="x",
                alpha=0.35,
                zorder=3,
            )

    polygon = _patch_polygon(view)
    if polygon.size:
        ax.plot(
            polygon[:, 0],
            polygon[:, 1],
            color="#f44336",
            linewidth=2.5,
            label="Configured patch",
            zorder=6,
        )
    patch_pixels = np.vstack((
        view.patch.inner_pixels,
        view.patch.outer_pixels,
    ))
    patch_valid = np.concatenate((
        view.patch.inner_valid,
        view.patch.outer_valid,
    ))
    invalid_patch = ~patch_valid & np.all(np.isfinite(patch_pixels), axis=1)
    if np.any(invalid_patch):
        ax.scatter(
            patch_pixels[invalid_patch, 0],
            patch_pixels[invalid_patch, 1],
            s=45,
            c="#f44336",
            marker="x",
            label="Invalid patch point",
            zorder=7,
        )
    centre = _patch_centre_pixel(view)
    if np.all(np.isfinite(centre)):
        ax.scatter(
            [centre[0]],
            [centre[1]],
            s=110,
            c="#e040fb",
            marker="*",
            edgecolors="black",
            linewidths=0.5,
            label="Patch centre",
            zorder=8,
        )

    ax.add_patch(Rectangle(
        (0.0, 0.0),
        view.metadata.width,
        view.metadata.height,
        fill=False,
        edgecolor="#76ff03",
        linewidth=2.0,
        linestyle="--",
        label="Camera boundary",
        zorder=9,
    ))


def _format_reasons(view: ValidationCameraView) -> str:
    return ", ".join(
        reason.replace("_", " ") for reason in view.visibility_reasons
    )


def _configure_axis(
    ax,
    view: ValidationCameraView,
    *,
    expanded: bool,
) -> None:
    if expanded:
        min_u, max_u, min_v, max_v = _expanded_limits(view)
        ax.set_xlim(min_u, max_u)
        ax.set_ylim(max_v, min_v)
        view_name = "expanded diagnostic"
    else:
        ax.set_xlim(0.0, view.metadata.width)
        ax.set_ylim(view.metadata.height, 0.0)
        view_name = "camera window"
    visibility = "VISIBLE" if view.patch.fully_visible else "NOT FULLY VISIBLE"
    ax.set_title(
        f"{view.label} {view_name} — {visibility}\n"
        f"{view.metadata.width} × {view.metadata.height} px",
        fontsize=11,
    )
    ax.set_xlabel("u [pixels]")
    ax.set_ylabel("v [pixels]")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.2)


def create_validation_figure(
    preview: ValidationPreview,
    images: dict[str, np.ndarray],
):
    """Build the four-panel figure without displaying or saving it."""
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(
        2,
        2,
        figsize=(18, 11),
        constrained_layout=True,
    )
    for column, (view, image) in enumerate((
        (preview.rgb, images["rgb"]),
        (preview.depth, images["depth"]),
    )):
        for row, expanded in enumerate((False, True)):
            axis = axes[row, column]
            _draw_image(axis, image, view)
            _draw_overlay(axis, view, expanded=expanded)
            _configure_axis(axis, view, expanded=expanded)
        detail = (
            f"timestamp={view.metadata.timestamp_ns} ns\n"
            f"theta_y={view.theta_y_rad:.5f} rad, "
            f"knee={view.knee_joint_rad:.5f} rad, "
            f"theta_p={view.theta_p_rad:.5f} rad\n"
            f"status: {_format_reasons(view)}"
        )
        axes[0, column].text(
            0.01,
            0.01,
            detail,
            transform=axes[0, column].transAxes,
            fontsize=8,
            verticalalignment="bottom",
            bbox={"facecolor": "white", "alpha": 0.78, "pad": 4},
            zorder=10,
        )

    axes[0, 0].legend(loc="upper right", fontsize=8, ncol=2)
    midpoint_offset_ms = (
        preview.pair.image_time_ns - preview.steady_midpoint_ns
    ) / 1_000_000.0
    figure.suptitle(
        "Configured track patch at the synchronized pair nearest the "
        "steady-motion midpoint\n"
        f"pair midpoint={preview.pair.image_time_ns} ns, "
        f"steady midpoint={preview.steady_midpoint_ns} ns, "
        f"offset={midpoint_offset_ms:+.3f} ms, "
        f"RGB-depth delta={preview.pair.sync_error_ns / 1_000_000.0:+.3f} ms, "
        f"patch angle={preview.patch_centre_angle_rad:.5f} rad",
        fontsize=13,
    )
    if hasattr(figure.canvas.manager, "set_window_title"):
        figure.canvas.manager.set_window_title(
            "LegWheel validate_config — midpoint patch diagnostic"
        )
    return figure


def show_validation_preview(
    preview: ValidationPreview,
) -> tuple[bool, str | None]:
    """Load the selected images and open a blocking Matplotlib window."""
    try:
        import matplotlib
        import matplotlib.pyplot as plt
    except ImportError:
        return False, "matplotlib_is_not_installed"

    images, error = load_preview_images(preview)
    if images is None:
        return False, error
    backend = str(matplotlib.get_backend()).lower()
    noninteractive_backends = {
        "agg",
        "cairo",
        "pdf",
        "pgf",
        "ps",
        "svg",
        "template",
    }
    if (
        backend in noninteractive_backends
        or "matplotlib_inline" in backend
    ):
        return False, f"matplotlib_backend_{backend}_cannot_open_a_window"
    create_validation_figure(preview, images)
    plt.show(block=True)
    return True, None
