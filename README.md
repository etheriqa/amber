amber
=====

[![Build Status](https://travis-ci.org/etheriqa/amber.svg?branch=master)](https://travis-ci.org/etheriqa/amber)

**amber** is a global illumination renderer.

![](https://gist.githubusercontent.com/etheriqa/fbec5f25fa05084c5abf/raw/0cb4dfee766e84562bcd8cfdde8e4ce423bac403/pt65536spp.png)
![](https://gist.githubusercontent.com/etheriqa/fbec5f25fa05084c5abf/raw/4e672b6e4f5f37af1eeda9f764b61715e0d73eec/pm16m256spp.png)

Features
--------

- Rendering algorithms
    - Path tracing
    - Bidirectional path tracing
    - Photon mapping
    - Primary sample space Metropolis light transport
- Geometric primitives
    - Triangle
    - Disk
    - Sphere
    - Cylinder
- Materials
    - Lambertian
    - Phong
    - Refraction
    - ~~Spectral refraction~~
    - Specular
- Depth of Field
- Tonemapping
    - Reinhard
    - Filmic

References
----------

- Filmic Tonemapping Operators, http://filmicgames.com/archives/75.
- T. Hachisuka, S. Ogaki, and H. W. Jensen, Progressive Photon Mapping, *ACM Transactions on Graphics* **27** (5), 2008.
- V. Havran, *Heuristic Ray Shooting Algorithms*, Ph.D. Dissertation, Czech Technical University, 2000.
- H. W. Jensen, and P. Christensen, High Quality Rendering using Ray Tracing and Photon Mapping, *ACM SIGGRAPH 2007* Course 8, 2007.
- C. Kelemen, L. Szirmay-Kalos, G. Antal, and F. Csonka, A Simple and Robust Mutation Strategy for the Metropolis Light Transport Algorithm, *Computer Graphics Forum* **21** (3), 2002.
- E. Lafortune, and Y. Willems, Using the Modified Phong Reflectance Model for Physically Based Rendering, Report CW197, Department of Computer Science, Katholieke Universiteit Leuven, 1994.
- T. Möller, and B. Trumbore, Fast, Minimum Storage Ray/Triangle Intersection, *Journal of Graphics Tools* **2** (1), 1997.
- E. Reinhard, M. Stark, P. Shirley, and J. Ferwerda, Photographic Tone Reproduction for Digital Images, *ACM Transactions on Graphics* **21** (3), 2002.
- E. Veach, *Robust Monte Carlo Methods for Light Transport Simulation*, Ph.D. Dissertation, Stanford University, 1997.
- I. Wald, and V. Havran, On building fast kd-Trees for Ray Tracing, and on doing that in O(N log N), Proceedings of *the 2006 IEEE Symposium on Interactive Ray Tracing*, 2006.
