# ApprovedRequest mirrors the Safety Island's actuator-facing supervised
# output, defined in the SI-side IDL
# (autoware-safety-island/actuation_module/src/autoware/autoware_msgs/msg/
# ApprovedRequest.idl, module safety_island_msgs::msg). Keep both definitions
# field-for-field identical: the SI IDL is the wire source of truth and this
# .msg is its Humble mirror for probes and the CARLA actuator.
#
# SI system clock when this request was produced.
builtin_interfaces/Time stamp
# SI boot/session identity (changes on SI restart).
uint32 session
# Strictly increasing output sequence within one SI session.
uint64 output_sequence
# Requested driving decision: 0 = NORMAL (approved request computed this
# cycle by the follower or passed through from VisionPilot), 1 = SI_STOP
# (supervisor stop; see fault_id), 2 = HOLD (keep actuating the previous
# approved state). Any value outside 0..2 is treated as HOLD by the
# actuator.
uint8 decision
# Configured supervision mode: 0 = SI_CONTROL (follower follows the
# selected trajectory source), 1 = VP_CONTROL (VisionPilot command
# supervised by SI, not recomputed).
uint8 mode
# Where the NORMAL control payload comes from: 0 = FOLLOWER
# (trajectory_follower on the selected trajectory), 1 = VP_COMMAND
# (accepted VisionPilot DrivingCommand, passed through).
uint8 selected_source
# VisionPilot cycle identity of the accepted input this output carries
# (0/0 when the source is not VisionPilot or no cycle was accepted).
uint32 source_session
uint64 source_cycle
# Identity of the fault event that caused the current SI_STOP/HOLD.
# Increments per detected fault; 0 whenever decision is NORMAL.
uint32 fault_id
# The actuation payload: for NORMAL, the approved driving request; for
# SI_STOP, an explicit stop (target speed 0, signed negative acceleration)
# so the actuator brakes without re-deciding itself.
autoware_control_msgs/Control control
