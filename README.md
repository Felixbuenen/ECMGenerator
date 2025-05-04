# Crowd Simulation Engine

## Intro
ECMGenerator is a crowd simulation engine, built in C++17. The focus lies on achieving real-time, large-scale crowd
simulations. The basis of the crowd simulator lies in the Explicit Corridor Map (ECM) navigation mesh structure. More information on ECM: 
> https://webspace.science.uu.nl/~gerae101/UU_crowd_simulation_publications_ecm.html.

## Features
- Real-time 2D agent-based simulation
- Creation of the Explicit Corridor Map navigation mesh
- Path planning- and following for agents of various sizes
- ORCA obstacle avoidance
- Editor to build simulation environments
- Data-oriented memory layout

## Video demo
Below is a demonstration of the latest version (April 2025).

[![](https://markdown-videos-api.jorgenkh.no/youtube/I7Dk0TetvgI)](https://youtu.be/I7Dk0TetvgI)

## Build instructions
This project relies on a few third party libraries:
- ImGui (UI)
- Boost (Voronoi diagram creation for ECM)
- SDL2 (rendering)

At this moment, the build process does not yet automatically include or fetch these dependencies. You will need to install them manually before building the project. Automating this setup (e.g. via CMake or vcpkg) is on the roadmap.

Once the libraries are installed, do the following:
- Create a folder "external" in the root folder of the repo;
- Place the dependencies in this folder;
- Navigate to the root folder and run <code>premake5 [your configuration, e.g. vs2022]</code>

## Roadmap
Currently the software can run approximately 5k agents in real-time (60fps, excluding rendering) on a machine with 12th Gen Intel Core i7 (16GB RAM). There is still a lot of room for improvement. The main focus in the coming months lies on the following points:
- (Performance) Improvement of data layout for cache locality
- (Performance) Multithreaded implementation
- (UX) More customization options to configure the simulation (spawn rate, different agent profiles and behaviours...)
- (UX) Save/load simulation configurations
- (Misc) Automating fetching and building dependencies