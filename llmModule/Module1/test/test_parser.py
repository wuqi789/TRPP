import json
from pathlib import Path

import pytest
import yaml

from llm_agent.json_parser import NavigationIntentParseError, parse_navigation_intent
from llm_agent.llm_interface import LLMInterface


PACKAGE = Path(__file__).parents[1]


def test_default_is_local_mock_without_provider_details():
    config = yaml.safe_load((PACKAGE / "config" / "config.yaml").read_text())
    assert config == {
        "mode": "mock",
        "adapter_env": "SCOUT_NAVIGATION_INTENT_ADAPTER",
    }


def test_mock_instruction_returns_contract_fixture():
    raw = LLMInterface({"mode": "mock"}).infer("Navigate to the door")
    intent = parse_navigation_intent(raw)
    assert intent.goal_object == "demo_goal"
    assert intent.constraints == ()


def test_external_mode_requires_adapter(monkeypatch):
    monkeypatch.delenv("SCOUT_NAVIGATION_INTENT_ADAPTER", raising=False)
    with pytest.raises(ValueError, match="external adapter"):
        LLMInterface({"mode": "external"})


def test_parser_rejects_unknown_schema_field():
    with pytest.raises(NavigationIntentParseError, match="Unexpected"):
        parse_navigation_intent(json.dumps({"goal_object": "door", "extra": 1}))
