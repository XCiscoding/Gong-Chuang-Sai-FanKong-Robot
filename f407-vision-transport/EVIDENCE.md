# Text TRACK adapter

This candidate corrects the protocol direction after the B3 detour.

- The transport baseline remains `uart_link_test.py`; canonical text frames now
  end in CRLF (`\r\n`) and receivers strip trailing CR/LF before parsing.
- F407 sends `MODE,TRACK` and expects `ACK,TRACK`.
- K230 sends `TRACK,<seq>,<dx>,0,<quality>` or `TRACK,<seq>,LOST`.
- `quality=100` is required by `Vision_IsFresh()`; quality 50 remains diagnostic.
- Duplicate/backward sequence numbers are rejected.
- No forward sequence for 200 ms destroys the session and restarts the 500 ms
  mode-request loop.
- `VISION_LINK_TEST_ACTIVE=1U`; the candidate firmware is passive and excludes
  motor initialisation/service.

Static/build tests do not establish physical UART or camera acceptance.

## Verification completed

- `test_line_track_source.py`: PASS (fragmented `MODE,TRACK`, ACK, five-frame
  stability, left/center scaling, jump rejection and LOST state clearing).
- `vision_text_protocol_harness.c`: PASS on the host (ACK/session reset,
  quality 50/100 gate, duplicate/backward rejection, 65535->0 wrap, LOST,
  malformed input and 200 ms reconnect).
- Full STM32CubeIDE ARM rebuild after CRLF unification: PASS, `text=12252`,
  `data=12`, `bss=2388`.
- Linked symbols include `Vision_Init`, `Vision_Service`, `VisionUart_Send` and
  `USART2_IRQHandler`; linked image does not contain `MotorTest_Init` or
  `StepMotor_Init`.

## Exact deliverables

- Current K230 TRACK script SHA256:
  `36BBFF06F906017E054532471CCC7DF583655595C892C0774C4FE813EFC45C97`
  (`../../k230/line_track_uart.py`). The historic K230 source snapshot used
  during the original adapter build had SHA256
  `BD96A0DB88E6D8CBB961EB5A50599656E73899786945A59D25F76274A324E59C`;
  do not treat that older hash as the current runtime-script identity.
- UART probe SHA256:
  `627297D6B00068552E76E8F7EF73D13741AAE0DF270D1BAAEB27238C4636D857`
- ELF SHA256:
  `DA92C7B138E80F01E8FD45517A1EA4AD79575F5D9BD04A70AC39FD63BE73C412`
- HEX SHA256:
  `288D998A78F003977FCF49BFC9A19D2E6C9ABC8E70EEA4E59020B02467CA2007`
- BIN SHA256:
  `2078B51CAECD750C7D361B15A7256688B968621F964EDBB915EC0FADDDCE69AE`

Flash only `PBRoBot-text-track-passive.hex` for this test. The prior B3 image
is historical and is not compatible with the project's passed text probe.
