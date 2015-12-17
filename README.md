amber
=====

[![Build Status](https://travis-ci.org/etheriqa/amber.svg?branch=master)](https://travis-ci.org/etheriqa/amber)

**amber** is a global illumination renderer for the purpose of learning.

![](https://gist.githubusercontent.com/etheriqa/fbec5f25fa05084c5abf/raw/6a87d18ae18c56ecf0ee85a00acd9c661a0b59db/pssmlt16g.png)

(PSSMLT with 16Gi mutations, rendered in 9.6 hours on MacBook Air with Intel Core i5-4250U)

Features
--------

- Rendering algorithms
    - `pt` Path tracing
    - `lt` Light tracing
    - `bdpt` Bidirectional path tracing
    - `pssmlt` Primary sample space Metropolis light transport
    - `mmlt` Multiplexed Metropolis light transport
    - `pm` Photon mapping
    - `ppm` Progressive photon mapping
    - `sppm` Stochastic progressive photon mapping
- Geometric primitives
    - Triangle
    - Disk
    - Sphere
    - Cylinder
- Materials
    - Lambertian
    - Phong
    - Refraction
    - Specular
- Depth of Field
- Tonemapping
    - Reinhard
    - Filmic

Dependencies
------------

- Boost >= 1.55
- OpenCV >= 2.3
