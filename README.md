amber
=====

[![Build Status](https://travis-ci.org/etheriqa/amber.svg?branch=master)](https://travis-ci.org/etheriqa/amber)

**amber** is a global illumination renderer for the purpose of learning.

Synopsis
--------

```sh
amber # renders a cornel box
amber --scene [FILE] --algorithm [ALGORITHM] --time [SECONDS] --spp [SPP] --output [FILE_WITHOUT_SUFFIX]
```

Following algorithms are available:

- **pt** - Path Tracing
- **lt** - Light Tracing
- **bdpt** - Bidirectional Path Tracing
- **pssmlt** - Primary Sample Space Metropolis Light Transport
- **sppm** - Stochastic Progressive Photon Mapping
- **mppm** - Memoryless Progressive Photon Mapping
- **ups** - Unified Path Sampling (default)

Requirements
------------

- Unix-like system (for now; we will support Windows systems before long)
- C++14 compatible compiler
- CMake
- Boost
- OpenCV
- assimp
- Google Test (optional)
