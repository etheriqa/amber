amber
=====

[![Build Status](https://travis-ci.org/etheriqa/amber.svg?branch=master)](https://travis-ci.org/etheriqa/amber)

**amber** is a global illumination renderer.

![](https://gist.githubusercontent.com/etheriqa/fbec5f25fa05084c5abf/raw/0cb4dfee766e84562bcd8cfdde8e4ce423bac403/pt65536spp.png)

Features
--------

- Rendering algorithms
    - Path tracing
    - Bidirectional path tracing
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
- Reinhard tonemapping

References
----------

- V. Havran, *Heuristic Ray Shooting Algorithms*, Ph.D. Dissertation, Czech Technical University, 2000.
- E. Lafortune and Y. Willems, Using the Modified Phong Reflectance Model for Physically Based Rendering, Report CW197, Department of Computer Science, Katholieke Universiteit Leuven, 1994.
- T. Möller and B. Trumbore, Fast, Minimum Storage Ray/Triangle Intersection, *Journal of Graphics Tools* **2** (1), 1997.
- E. Reinhard, M. Stark, P. Shirley and J. Ferwerda, Photographic Tone Reproduction for Digital Images, *ACM Transactions on Graphics* **21** (3), 2002.
- E. Veach, *Robust Monte Carlo Methods for Light Transport Simulation*, Ph.D. Dissertation, Stanford University, 1997.
- I. Wald and V. Havran, On building fast kd-Trees for Ray Tracing, and on doing that in O(N log N), Proceedings of *the 2006 IEEE Symposium on Interactive Ray Tracing*, 2006.
