import json

from groundingdino_vlm.vlm import ExternalVLM, VLMConfigurationError


def test_external_adapter_requires_factory(monkeypatch):
    monkeypatch.delenv("SCOUT_TARGET_VLM_ADAPTER", raising=False)
    try:
        ExternalVLM({})
    except VLMConfigurationError as exc:
        assert str(exc) == "adapter is not configured"
    else:
        raise AssertionError("missing adapter must fail closed")


def test_external_adapter_passes_public_contract(monkeypatch):
    class Adapter:
        def verify(self, image, request):
            assert image == b"image"
            assert request["contract"] == "scout.target-verification.v1"
            return {"accepted": True}

    import sys
    import types
    module = types.ModuleType("test_target_adapter")
    module.create = lambda: Adapter()
    sys.modules[module.__name__] = module
    monkeypatch.setenv("SCOUT_TARGET_VLM_ADAPTER", "test_target_adapter:create")
    result = ExternalVLM({}).verify(b"image", "curtain", [])
    assert result is True
