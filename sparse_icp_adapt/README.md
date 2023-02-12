# Sparse Iterative Closest Point Algorithm #

This repository contains an implementation of the [Sparse Iterative Closest Point](http://dl.acm.org/citation.cfm?id=2600305). 

## Dependencies ##

The dependencies are header-only and are all included in the ext directory. As a consequence, there is nothing to do.
For the record, here is the list of dependencies :

* [nanoflann](https://github.com/jlblancoc/nanoflann)
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main\_Page)

## Usage ##


* `./icpSparse source.obj target.obj source_regist.obj (output) p_norm (0.4) iterations (30 or 1000)
## Reference ##

Bouaziz, Sofien, Andrea Tagliasacchi, et Mark Pauly. « Sparse Iterative Closest Point ». In Proceedings of the Eleventh Eurographics/ACMSIGGRAPH Symposium on Geometry Processing, 113–123. SGP ’13. Aire-la-Ville, Switzerland, Switzerland: Eurographics Association, 2013. doi:10.1111/cgf.12178.

