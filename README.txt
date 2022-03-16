This repository contains source code and image data to reproduce a frog musculoskeletal model and simulation created initially by Amber Collings and Enrico Eberhard and modified by Chris Basu and Chris Richards.  Like anything in science, it is a work in progress.  You may dowload it and work with it freely or you can fork it locally to stay up-to-date with changes.  

If you use this model, please use the following citation:
Amber J Collings, Enrico Eberhard, Chris Basu, Chris Richards
"Functional analysis of anuran pelvic and thigh anatomy using musculoskeletal modeling of Phlyctimantis maculatus", DOI: 10.3389/fbioe.2022.806174, Frontiers Bioengineering

Take a moment to check the folder structure, below.  Then open \documentation\README_FIRST.txt

"_LINUX, _OSX, _WIN" contain binary compiled code for running MuJoCo programs
"data" contains misc. data used in the workflow, but not direct input/output of simulations. 
"documentation" - for instructions as well as information on muscles and dof of model
"inc" contains include .h c++ source code, including custom toolboxes for MuJoCo
"input" contains input data files directly used in MuJoCo simulations
"models" contains xml files and associated assets (stl, obj meshes, etc) for MuJoCo model
"output" contains data written by simulation
"scripts" contains any associated scripts in Matlab (.m), mathematica (.nb) or other.  Also contains metadata files as well as some other useful items
"src" contains all c++ source code for MuJoCo simulations

