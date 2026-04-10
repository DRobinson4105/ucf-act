<!-- Responsibility: Brief ownership summary for the motor_protocol component. -->

# motor_protocol

Brief responsibility split for the `motor_protocol` component.

## File responsibilities

### `include/motor_protocol.h`

Public API surface of the component. This header should stay small and contain
only declarations that other components are allowed to use directly.

### `include/motor_types.h`

Shared public protocol data model. This header owns the public protocol nouns:
command structs, parsed RX structs, and public enums for families, parameters,
errors, notifications, and RX kinds.

### `motor_codec.c`

Mechanical protocol layer. This file is the place for exact encode/decode
mechanics such as identifier composition, control-word transformation, ACK bit
handling, and byte packing or unpacking helpers.

### `motor_cmd.c`

Outgoing command catalog. This file should build readable outgoing
`motor_cmd_t` objects for protocol families and command types by relying on the
lower-level codec helpers.

### `motor_rx.c`

Incoming frame interpreter. This file should parse and classify received motor
frames into structured `motor_rx_t` data without owning transport calls or
higher-level control decisions.

### `motor_names.c`

Stable naming dictionary. This file should map protocol enums and codes to
short readable names for logs, diagnostics, and debugging.

## Dependency direction

Intended layering:

- `motor_cmd.c -> motor_codec.c`
- `motor_rx.c -> motor_codec.c`
- `motor_names.c -> motor_types.h`

Conceptually:

- `motor_codec.c` owns mechanics
- `motor_cmd.c` owns outgoing intent
- `motor_rx.c` owns incoming interpretation
- `motor_names.c` owns human-readable labels
