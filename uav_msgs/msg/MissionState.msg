# What the aircraft is doing, as one message: the autonomy state and the task.
#
# Published by mission_planner and consumed by ocs_client, which passes these
# two strings straight into the OCS heartbeat. That is why they are STRINGS and
# not an enum of our own: the values are the protobuf enum names from
# RoboNation's schema (common.proto RobotState, rx_common.proto RxTask), so they
# travel end to end with no mapping layer to get out of step.
#
# The planner is the ONLY node that decides these. heartbeat_core used to decide
# state itself from a flat list of modes; two places answering one question is
# how they eventually answer it differently.
std_msgs/Header header

# STATE_AUTO | STATE_MANUAL.
# Never STATE_UNKNOWN: it is the proto zero value, the OCS refuses it, and for a
# UAV nothing may be UNKNOWN at all. When the state is not knowable the planner
# publishes nothing and ocs_client lets the heartbeat lapse, which is truthful.
string state

# TASK_NONE | TASK_SAFE_PASSAGE | TASK_INFRA_SURVEY_REPAIR |
# TASK_COORDINATED_LOGISTICS | TASK_DYNAMIC_INCIDENT.
# TASK_UNKNOWN is never sent -- the handbook forbids it outright.
string task

# Why the state is what it is, in words: "armed in GUIDED", "armed in LOITER
# under an executing mission", "disarmed". Goes to the operator page.
string reason

# Whether a mission is executing. This is the fact the flight mode cannot
# carry: a mission flying the aircraft in LOITER under CV control IS autonomous,
# and without this flag the mode alone would report STATE_MANUAL throughout.
bool mission_active

# Which mission, when one is running. Empty otherwise.
string mission_name
