"""A picture of what the pick is about to work on: the target's point cloud, which is
what SAM3's mask became in 3D, with the best few grasps drawn on it as open hands.

Saved beside the run's detections (core.perception.sam3_client.run_folder), for
showing afterwards. It is never on the path of the pick itself: DETECTIONS=0 switches
it off, and a failure to draw is reported, not raised."""

import json
import os

import numpy as np

from core.perception import sam3_client

SHOWN = 4      # grasps drawn, in the order MTC will try them
FINGER = 0.06  # m, drawn finger length; only for the picture
COLOURS = ["#d62728", "#1f77b4", "#2ca02c", "#ff7f0e"]


def hand_outline(palm, width, rotation, pad):
    """The open hand as a polyline in the cloud's frame: finger, back of the hand, finger,
    and a short stem pointing back along the approach."""
    canonical = palm @ np.linalg.inv(rotation)
    closing, approach = canonical[:3, 0], canonical[:3, 2]
    centre = canonical[:3, 3] + canonical[:3, :3] @ pad
    left, right = centre + closing * width / 2, centre - closing * width / 2
    back = -approach * FINGER
    middle = (left + right) / 2 + back
    return np.array([left, left + back, middle, middle + back * 0.8, middle, right + back, right])


def save(prompt, points, grasps, widths, kind):
    if os.environ.get("DETECTIONS", "1") == "0":
        return
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from core.grasping import pick

        folder = sam3_client.run_folder()
        folder.mkdir(parents=True, exist_ok=True)
        name = "grasps_" + sam3_client._slug(prompt)
        grasps, widths = np.asarray(grasps)[:SHOWN], np.asarray(widths)[:SHOWN]
        np.savez(folder / f"{name}.npz", points=points, grasps=grasps, widths=widths)
        rotation = np.array(json.loads((pick.GRIPPER_DIR / "config.json").read_text())["base_rotation"])
        hands = [hand_outline(palm, width, rotation, pick.pad_for_width(width))
                 for palm, width in zip(grasps, widths)]
        everything = np.vstack([points, *hands])
        centre, half = everything.mean(0), np.ptp(everything, axis=0).max() / 2 * 1.05
        figure = plt.figure(figsize=(12, 6))
        for column, (elevation, azimuth, title) in enumerate([(22, -60, "from the side"), (89, -90, "from above")], 1):
            axes = figure.add_subplot(1, 2, column, projection="3d")
            axes.scatter(*points.T, s=2, c=points[:, 2], cmap="viridis", alpha=0.6)
            for rank, (hand, colour) in enumerate(zip(hands, COLOURS), 1):
                axes.plot(*hand.T, color=colour, linewidth=2.5,
                          label=f"grasp {rank}, opening {widths[rank - 1] * 1000:.0f} mm")
            for axis, middle in zip("xyz", centre):
                getattr(axes, f"set_{axis}lim")(middle - half, middle + half)
                getattr(axes, f"set_{axis}label")(f"{axis} (m)")
            axes.view_init(elevation, azimuth)
            axes.set_title(title)
            if elevation > 80:
                # Straight down, the height axis is a point and its labels pile up on it.
                axes.set_zticks([])
                axes.set_zlabel("")
        axes.legend(loc="upper right", fontsize=8)
        figure.suptitle(f'"{prompt}"  |  {folder.name}\n{len(points)} points, fitted as {kind}; the best '
                        f"{len(hands)} of GraspGenX's grasps, in the order they are tried", fontsize=11)
        figure.tight_layout()
        figure.savefig(folder / f"{name}.png", dpi=130)
        plt.close(figure)
        print(f"[GRASP] cloud and best {len(hands)} grasps drawn in {folder / (name + '.png')}", flush=True)
    except (OSError, RuntimeError, ValueError) as error:
        print(f"[GRASP] picture not saved: {error}", flush=True)
