import json

from route_planning import ArtifactStore, GridMap, GridPoint, RoutePlanner, parse_waypoints


def grid():
    return GridMap(width=10, height=10, resolution=1.0, origin_x=0.0, origin_y=0.0,
                   data=[0] * 100, inflation_radius=0.0, scale=4)


class Adapter:
    ready = True
    last_transport = "test"

    def infer(self, image, request, feedback=None, **_kwargs):
        assert request["contract"] == "scout.route-waypoints.v1"
        return '{"waypoints":[]}'


def test_request_is_structured_without_prompt_text():
    request = RoutePlanner._request(grid(), GridPoint(1.5, 1.5), GridPoint(5.5, 3.5))
    assert request["contract"] == "scout.route-waypoints.v1"
    assert "response_schema" in request
    assert "GREEN" not in json.dumps(request)


def test_route_planner_accepts_adapter_response(tmp_path):
    planner = RoutePlanner(Adapter(), ArtifactStore(tmp_path))
    result = planner.plan("task", grid(), GridPoint(1.5, 1.5), [GridPoint(8.5, 1.5)])
    assert result.waypoints == (GridPoint(8.5, 1.5),)


def test_waypoint_parser_is_strict():
    assert parse_waypoints('{"waypoints":[]}', 2) == []
