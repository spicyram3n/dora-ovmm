"""Find a named object in the world: one camera frame, one segmentation, one
point cloud, in whatever frame the caller needs.

Every stage of the pipeline wants this same three-step move, and it used to be
written out separately in run_pipeline and search_object -- which meant two
copies drifting apart, and two camera grabs where one would do.
"""

from perception import pointcloud, sam3_client
from perception.camera_ros2 import grab_rgbd


class NotFound(RuntimeError):
    """The object is not visible from here. A routine outcome of looking in the
    wrong place, not a failure -- the caller should try somewhere else."""


def find(prompt, target_frame):
    """(points, score) for `prompt`, as an (N, 3) cloud in `target_frame`.

    Raises NotFound if the segmenter sees nothing, or if everything it saw
    lacked valid depth.
    """
    rgb, depth_m, k, transform = grab_rgbd(target_frame=target_frame)
    try:
        mask, score = sam3_client.detect(rgb, prompt)
    except RuntimeError as error:
        raise NotFound(str(error)) from error

    points = pointcloud.deproject(depth_m, k, mask)
    if len(points) == 0:
        raise NotFound("found it, but no valid depth inside the mask")
    return pointcloud.transform_points(transform, points), score
