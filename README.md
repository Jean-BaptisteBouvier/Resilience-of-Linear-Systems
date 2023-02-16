# Resilience-of-Linear-Systems

This project studies the resilience of linear systems to a partial loss of control authority over their actuators.

This project is based on the paper ["Resilience of Linear Systems to Partial Loss of Control Authority"](https://arxiv.org/abs/2209.08034) by Jean-Baptiste Bouvier and Melkior Ornik, published by Automatica.


This project contains all the MATLAB code for the simulations shown in the paper.
Requires the CORA toolbox available on MATLAB to compute the zonotopes.


**File Structure**
---

- The ADMIRE simulation is run with amire_reachability.m
- The temperature simulation is run with temperature.m
- The functions time_optimal_Eaton.m and time_optimal_Sakawa.m compute the minimal reachability times for linear systems. They are used in temperature.m




**Running**
---




**Citation**
---
```
@article{bouvier2023resilience,  
  title = {Resilience of Linear Systems to Partial Loss of Control Authority},   
  author = {Jean-Baptiste Bouvier and Melkior Ornik},    
  journal = {Automatica},    
  year = {2023},   
  volume = {},  
  number = {},  
  pages = {}  
}
```


**Contributors**
---
- [Jean-Baptiste Bouvier](https://github.com/Jean-BaptisteBouvier)
- [Melkior Ornik](https://mornik.web.illinois.edu/)


