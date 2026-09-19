"""Bounded request-id reservation and deduplication."""

from __future__ import annotations

from collections import OrderedDict


class RequestRegistry:
    def __init__(self, capacity: int = 256) -> None:
        if capacity < 1:
            raise ValueError("request registry capacity must be positive")
        self.capacity = int(capacity)
        self._pending: set[str] = set()
        self._known: OrderedDict[str, None] = OrderedDict()

    def reserve_topic(self, request_id: str) -> bool:
        if not request_id or request_id in self._pending or request_id in self._known:
            return False
        self._pending.add(request_id)
        return True

    def accept_goal(self, request_id: str) -> bool:
        if not request_id:
            return False
        self._pending.discard(request_id)
        if request_id in self._known:
            return False
        self._known[request_id] = None
        self._known.move_to_end(request_id)
        while len(self._known) > self.capacity:
            self._known.popitem(last=False)
        return True

    def release_topic(self, request_id: str) -> None:
        self._pending.discard(request_id)

    def contains(self, request_id: str) -> bool:
        return request_id in self._pending or request_id in self._known
