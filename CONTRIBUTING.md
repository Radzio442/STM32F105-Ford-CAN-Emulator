# Contributing

Contributions, bench-test results and documentation corrections are welcome.

## Before opening an issue

Include enough information to reproduce the behavior:

- MCU board and CAN transceivers used;
- module part numbers and bench wiring;
- CAN channel, arbitration ID, DLC and full payload;
- expected and observed behavior;
- whether an APIM or ACM was connected;
- relevant CAN log captured before and after the change.

Remove VINs and other identifying vehicle data from public logs.

## Pull requests

1. Keep unrelated frames and behavior unchanged.
2. Use `CAN1` and `CAN2` names rather than ambiguous bus-speed labels.
3. Bound all array indexes and avoid unbounded waits in the main loop.
4. Document new IDs and payloads in the README.
5. Build with STM32CubeIDE and test on an isolated bench setup.
6. Describe the tested hardware and result in the pull request.

Do not submit secrets, private VINs, proprietary databases or copyrighted
service documentation.
