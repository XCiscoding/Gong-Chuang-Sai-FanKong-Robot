#!/usr/bin/env python3
"""Block unjustified debug-tool expansion before it reaches hardware work."""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class Decision:
    passed: bool
    route: str
    reasons: tuple[str, ...] = ()


def decide(payload: dict) -> Decision:
    reasons: list[str] = []
    required_text = ("unknown", "direct_evidence", "selected_instrument", "selected_observation_point", "operator")
    for field in required_text:
        if not isinstance(payload.get(field), str) or not payload[field].strip():
            reasons.append(f"MISSING_{field.upper()}")

    instruments = payload.get("available_instruments")
    if not isinstance(instruments, list) or not instruments:
        reasons.append("NO_CONFIRMED_INSTRUMENT_INVENTORY")
    else:
        selected = [
            item
            for item in instruments
            if isinstance(item, dict)
            and item.get("confirmed") is True
            and item.get("name") == payload.get("selected_instrument")
            and item.get("observation_point") == payload.get("selected_observation_point")
        ]
        if not selected:
            reasons.append("SELECTED_INSTRUMENT_OR_OBSERVATION_POINT_NOT_CONFIRMED")

    expansion = payload.get("proposed_expansion", "none")
    if expansion not in {"none", "gui", "script", "hardware", "wiring"}:
        reasons.append("INVALID_PROPOSED_EXPANSION")
    if expansion != "none" and not str(payload.get("existing_tool_insufficiency", "")).strip():
        reasons.append("EXPANSION_WITHOUT_DIRECT_EVIDENCE_GAP")

    if payload.get("user_self_operates") is True and payload.get("agent_terminal") != "WORKBENCH_READY":
        reasons.append("USER_SELF_OPERATION_REQUIRES_WORKBENCH_READY")

    if reasons:
        return Decision(False, "BLOCK", tuple(reasons))
    return Decision(True, "USE_CONFIRMED_EXISTING_INSTRUMENT")


def report(label: str, decision: Decision) -> int:
    if decision.passed:
        print(f"PASS debug_instrument_preflight label={label} route={decision.route}")
        return 0
    print(f"BLOCK debug_instrument_preflight label={label} reasons={';'.join(decision.reasons)}")
    return 1


def self_test() -> int:
    base = {
        "unknown": "whether the target emits the expected frame",
        "direct_evidence": "the exact received bytes at the selected RX point",
        "available_instruments": [{"name": "serial-assistant", "observation_point": "target-tx", "confirmed": True}],
        "selected_instrument": "serial-assistant",
        "selected_observation_point": "target-tx",
        "operator": "user",
        "proposed_expansion": "none",
        "existing_tool_insufficiency": "",
        "user_self_operates": True,
        "agent_terminal": "WORKBENCH_READY",
    }
    cases = (
        ("USER_SERIAL_ASSISTANT", base, True, "USE_CONFIRMED_EXISTING_INSTRUMENT"),
        ("GUI_WITHOUT_EVIDENCE_GAP", {**base, "proposed_expansion": "gui"}, False, "BLOCK"),
        ("AGENT_OPERATES_USER_TOOL", {**base, "agent_terminal": "AGENT_SENDS_FRAME"}, False, "BLOCK"),
        ("UNCONFIRMED_ANALYZER", {**base, "selected_instrument": "logic-analyzer"}, False, "BLOCK"),
    )
    failures = 0
    for label, payload, expected_pass, expected_route in cases:
        actual = decide(payload)
        ok = actual.passed == expected_pass and actual.route == expected_route
        print(
            f"{'PASS' if ok else 'FAIL'} self_test label={label} "
            f"expected={expected_route} actual={actual.route} "
            f"reasons={';'.join(actual.reasons) or 'NONE'}"
        )
        failures += 0 if ok else 1
    if failures:
        print(f"BLOCK self_test failures={failures}")
        return 1
    print(f"PASS self_test cases={len(cases)}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path)
    parser.add_argument("--label", default="DEBUG_INSTRUMENT_PREFLIGHT")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        return self_test()
    if args.manifest is None:
        parser.error("--manifest is required unless --self-test is used")
    try:
        payload = json.loads(args.manifest.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        print(f"BLOCK debug_instrument_preflight label={args.label} reasons=INVALID_MANIFEST:{exc}")
        return 1
    return report(args.label, decide(payload))


if __name__ == "__main__":
    raise SystemExit(main())
