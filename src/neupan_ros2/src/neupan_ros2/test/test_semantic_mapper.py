import numpy as np
from nav_msgs.msg import OccupancyGrid
from pathlib import Path

from neupan_ros2.semantic_mapper import (
    MOCK_SEMANTIC_CLASSES,
    VoxelVoteMap,
    build_obstacle_layer,
    make_polygon_mask,
)


def test_voxel_votes_once_per_frame():
    vote_map = VoxelVoteMap(voxel_size=1.0, class_count=3)
    points = np.array(
        [[0.1, 0.1, 0.1], [0.2, 0.2, 0.2], [1.1, 0.1, 0.1]]
    )

    for labels in ([0, 0, 1], [1, 0, 1], [0, 0, 2]):
        vote_map.add_frame(points, np.asarray(labels))

    points, labels, totals, ratios = vote_map.resolved(3, 0.6)
    assert len(MOCK_SEMANTIC_CLASSES) == 4
    assert len(points) == 2
    assert set(labels.tolist()) == {0, 1}
    assert totals.tolist() == [3, 3]
    assert np.allclose(sorted(ratios), [2 / 3, 1.0])


def test_normalized_self_mask():
    mask = make_polygon_mask(10, 10, [0.25, 0.25, 0.75, 0.25, 0.5, 0.75])
    assert mask.dtype == np.bool_
    assert mask[4, 5]
    assert not mask[9, 9]


def test_obstacle_layer_filters_surfaces_on_rotated_map():
    occupancy = OccupancyGrid()
    occupancy.info.resolution = 1.0
    occupancy.info.width = 4
    occupancy.info.height = 3
    occupancy.info.origin.position.x = 10.0
    occupancy.info.origin.position.y = 20.0
    occupancy.info.origin.orientation.z = np.sin(np.pi / 4)
    occupancy.info.origin.orientation.w = np.cos(np.pi / 4)
    grid = np.zeros((3, 4), dtype=np.int8)
    grid[1, 2:4] = 100
    occupancy.data = grid.ravel().tolist()

    points = np.array(
        [
            [8.5, 22.5, 0.5],  # obstacle in occupied cell (row=1, col=2)
            [8.5, 22.5, 1.0],  # ignored background label
            [8.5, 23.5, 0.0],  # ignored surface label
            [8.5, 23.5, 2.0],  # ignored background label
        ],
        dtype=np.float32,
    )
    output = build_obstacle_layer(
        points,
        np.array([19, 0, 3, 5], dtype=np.uint16),
        np.full(4, 3, dtype=np.uint16),
        np.ones(4, dtype=np.float32),
        occupancy,
        {0, 3, 5},
        occupancy_threshold=50,
        snap_radius=0.0,
        fill_radius=1.1,
        layer_z=0.06,
    )
    layer_points, labels, _, _, cells = output

    assert cells.tolist() == [[1, 2], [1, 3]]
    assert labels.tolist() == [19, 19]
    assert np.allclose(layer_points[:, 2], 0.06)
    assert np.allclose(layer_points[:, :2], [[8.5, 22.5], [8.5, 23.5]])


def test_obstacle_layer_snaps_to_nearest_occupied_cell():
    occupancy = OccupancyGrid()
    occupancy.info.resolution = 1.0
    occupancy.info.width = 3
    occupancy.info.height = 1
    occupancy.info.origin.orientation.w = 1.0
    occupancy.data = [100, 0, 100]

    output = build_obstacle_layer(
        np.array([[1.6, 0.5, 0.5]], dtype=np.float32),
        np.array([19], dtype=np.uint16),
        np.array([3], dtype=np.uint16),
        np.array([1.0], dtype=np.float32),
        occupancy,
        set(),
        occupancy_threshold=50,
        snap_radius=2.0,
        fill_radius=0.0,
        layer_z=0.06,
    )

    assert output[4].tolist() == [[0, 2]]


def test_semantic_mapper_publishes_synchronized_class_images():
    source = (
        Path(__file__).parents[1] / "neupan_ros2" / "semantic_mapper.py"
    ).read_text(encoding="utf-8")
    assert '"/semantic_mapping/class_ids"' in source
    assert '"/semantic_mapping/class_confidence"' in source
    assert 'encoding="mono8"' in source
    assert 'encoding="32FC1"' in source
    assert 'Image, "/semantic_mapping/overlay", image_output_qos' in source
    assert "reliability=ReliabilityPolicy.RELIABLE" in source
