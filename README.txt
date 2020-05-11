This repository contains source code and image data to reproduce a frog musculoskeletal model and simulation created by Amber Collings and Enrico Eberhard and modified by Chris Basu and Chris Richards.  It is currently incomplete and under development and has not been validated or fully tested and therefore is not yet intended for public use.

The folder structure is as follows:

"_LINUX, _OSX, _WIN" contain binary compiled code for running MuJoCo programs
"data" contains misc. data used in the workflow, but not direct input/output of simulations. 
"documentation" - for instructions as well as information on muscles and dof of model
"inc" contains include .h c++ source code, including custom toolboxes for MuJoCo
"input" contains input data files directly used in MuJoCo simulations
"models" contains xml files and associated assets (stl, obj meshes, etc) for MuJoCo model
"output" contains data written by simulation
"scripts" contains any associated scripts in Matlab (.m), mathematica (.nb) or other.  Also contains metadata files as well as some other useful items
"src" contains all c++ source code for MuJoCo simulations