# `sbnalg`: SBN software not depending on art framework

This repository includes algorithms and more general code that can be used in "stand-alone" C++ and Python application.
They are expected to be applicable to both SBND and ICARUS.

## Dependencies

This repository formally depends on `sbnobj` and `larsoftobj` (including `larcorealg` and `lardataalg`).
The software in this repository do not depend on the _art_ framework.

Acceptable dependencies for the software in this repository are:
 * `fhiclcpp` for configuration access
 * `messagefacility` for logging
 * CERN ROOT libraries
 * ... and anything that these dependencies pull in

The algorithmic C++ code in here is built using `cetbuildtools`.
