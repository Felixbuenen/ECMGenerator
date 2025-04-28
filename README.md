# Crowd Simulation Engine

## Intro
ECMGenerator is a crowd simulation engine, built in C++17. The focus lies on achieving real-time, large-scale crowd
simulations. The basis of the crowd simulator lies in the Explicit Corridor Map (ECM) navigation mesh structure. More information on ECM: 
> https://webspace.science.uu.nl/~gerae101/UU_crowd_simulation_publications_ecm.html.

## Features
- Real-time 2D agent-based simulation
- ORCA obstacle avoidance
- Create your own simulation environment using spawn-, goal- and obstacle areas.
- Data-oriented memory layout

## Video demo
Below is a demonstration of the latest version (April 2025).

[![](https://markdown-videos-api.jorgenkh.no/youtube/8guPXsBxL5w)](https://youtu.be/8guPXsBxL5w)

## Build instructions
TODO

## Roadmap
Currently the software can run approximately 10k agents in real-time (60fps, excluding rendering) on a machine with 12th Gen Intel Core i7 (16GB RAM). There is still a lot of room for improvement. The main focus in the coming months lies on the following points:
- (Performance) Improvement of data layout for cache locality
- (Performance) Multithreaded implementation
- (UX) More customization options to configure the simulation (spawn rate, different agent profiles...)
- (UX) Save/load simulation configurations
