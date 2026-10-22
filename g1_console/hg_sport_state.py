"""Receive-only binding of the pinned official HG SportModeState_ definition.

Source and SHA256: assets/safety_evidence_sources/SportModeState_.hpp and
g1_safety_evidence_audit.json. This message has no foot force/contact fields.
"""
from dataclasses import dataclass
import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate
import cyclonedds.idl.types as types


@dataclass
@annotate.final
@annotate.autoid("sequential")
class SportModeState_(idl.IdlStruct, typename="unitree_hg.msg.dds_.SportModeState_"):
    fsm_id: types.uint32
    fsm_mode: types.uint32
    task_id: types.uint32
    task_time: types.float32
