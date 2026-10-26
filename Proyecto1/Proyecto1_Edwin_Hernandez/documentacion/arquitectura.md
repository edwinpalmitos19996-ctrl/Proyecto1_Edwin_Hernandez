```mermaid
graph TD
    IDLE((IDLE))
    RUNNING((RUNNING))
    FAULT((FAULT_LOCKOUT))

    IDLE -- "START_CMD\n[onStart]" --> RUNNING
    RUNNING -- "ESTOP_TRIPPED\n[onFault]" --> FAULT
    FAULT -- "RESET_CMD\n[onReset]" --> IDLE

    IDLE -. "NONE\n[onNoOp]" .-> IDLE
    RUNNING -. "NONE\n[onNoOp]" .-> RUNNING
    FAULT -. "NONE / ESTOP\n[onNoOp]" .-> FAULT