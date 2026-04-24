"""Decision record emitted by the policy engine on every tick."""
from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any


@dataclass
class Decision:
    """What the controller *would* apply if the wire were connected.

    Phase 0: logged only. Phase 2: serialised to a wire packet.

    `reason` is a short human-readable trigger string used heavily in
    Phase 0 for pilot-vs-controller correlation.
    `knobs_changed` lists the specific knob names that moved this tick;
    empty on steady-state ticks.
    """
    timestamp: float
    mcs: int
    bandwidth: int
    tx_power_dBm: int
    k: int
    n: int
    depth: int
    bitrate_kbps: int
    idr_request: bool
    reason: str = ""
    knobs_changed: list[str] = field(default_factory=list)
    # Informational — the controller's internal state when this
    # decision was emitted. Kept out of the Phase 2 wire packet.
    signals_snapshot: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict:
        return asdict(self)
