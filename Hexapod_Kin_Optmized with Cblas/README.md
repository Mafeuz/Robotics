# Newton-Raphson-Optimized-in-C
Conversion of a MATLAB Newton Method code for Hexapod Forward Kinematics to C using the optmized OpenBLAS library.

* HexapodNewtonMATLAB.m contains the working code made with MATLAB.

* HexapodForwardKinOPENBLAS.c contains the working code (hardcoded) in C (using the Open Blas library) tested in a [Docker Container](https://github.com/ogrisel/docker-openblas).

# Possible Advances:

* Use a Lapacke solver to solve the Newton Method Linear system instead of a separated Inversion and Multiplication.
