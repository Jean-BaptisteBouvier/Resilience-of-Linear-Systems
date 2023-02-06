# Resilience-of-Linear-Systems

This project studies the resilience of linear systems to a partial loss of control authority over their actuators.

This project is based on the paper "Resilience of Linear Systems to Partial Loss of Control Authority" by Jean-Baptiste Bouvier and Melkior Ornik, published by Automatica.
The ArXiv version of this work is available [here](https://arxiv.org/abs/2209.08034).


This project contains all the MATLAB code for the simulations shown in the paper.
Requires the CORA toolbox available on MATLAB to compute the zonotopes.

The ADMIRE simulation is run with amire_reachability.m
The temperature simulation is run with temperature.m
The functions time_optimal_Eaton.m and time_optimal_Sakawa.m compute the minimal reachability times for linear systems. They are used in temperature.m


To cite this work use:

add bibtex citation when published
