#!/usr/bin/env python3
"""Pure target-selection helpers shared by cabinet validation tests."""

from __future__ import annotations

from collections.abc import Sequence


class NoAlternativeTargetError(ValueError):
    """The live state cannot be changed to another advertised state."""


def select_non_noop_target_state(
    state_ids: Sequence[str],
    current_state: str,
    *,
    control_id: str,
) -> str:
    """Return the first advertised state different from the live state.

    A reachability validator must not submit ``SET_STATE`` to the state the
    control already occupies: that can produce planning diagnostics without
    ever validating a non-zero manipulation path.
    """
    live_state = str(current_state)
    advertised_states = [str(state_id) for state_id in state_ids]
    if not live_state or live_state not in advertised_states:
        raise NoAlternativeTargetError(
            "NOT APPLICABLE: "
            f"{control_id} has no advertised live current_state "
            f"({live_state or 'missing'}); a non-zero validation target "
            "cannot be proven."
        )
    for state_id in advertised_states:
        if state_id and state_id != live_state:
            return state_id
    raise NoAlternativeTargetError(
        "NOT APPLICABLE: "
        f"{control_id} advertises no target different from live state "
        f"{live_state!r}; refusing a no-op reachability validation."
    )
