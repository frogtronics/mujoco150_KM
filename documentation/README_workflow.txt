Root: ../FrogWork/CODE/_mjpro150_KM/

PART 1. Generate Quaternions file from XYZ kinematics file
Software: MATLAB: kassinainputs.m
Input: XYZ marker file (works for 12 markers, currently - so that is the old 2015 dataset including walking/running)
Dependencies:

PART 1A. Use MJmodelling_testLateralRotation_XX.nb in Mathematica to generate the hypothetical test conditions for rotating the pelvis and femur angles.


PART 2. Create a mapdata_k.txt to place in the input folder which labels each dof as follows:
jointname_X where X is the index of that joint's dof (not cumulative).  X will be an integer between 0 and 6 depending on the joint type.
e.g. 
j_joint1_0
j_joint1_1
j_joint1_2
j_joint1_3
j_joint2_0
...

In the future, the kassinainputs.m file can be modified to automatically generate mapdata_k, but for now it is made manually.


PART 3. Run mapModel to generate mapdata_m which is a list of dof for the model.  Must be done each time the model is changed (e.g. dof's changed in number or in name).
USAGE: mapModel -m modelfile -o ../input/mapdata_m.txt


PART 4. Run changeAngles to replay kinematics and record tendon lengths and moment arms (also save files)

USAGE: changeAngles -m modelfile -i inputfile
optional additional arguments
-t tendondataOutputdile 
-f footpositionOutputfile

For example: ./changeAngles -i ../input/instructionsFile.txt
This will open the instructions file which contains the file list for which conditions to run.  All hypothetical conditions are in TST ("test"), so the instructions file should read something like:

../models/Kassina/Kassina.xml
../input/Kassina_inputFiles_EXEMPLAR_RUN.txt
../input/Kassina_inputTrials.txt

for the run trial, or...

../models/Kassina/Kassina.xml
../input/Kassina_inputFiles_TST.txt
../input/Kassina_inputTrials.txt

to run the test cases.  They can all be stored in the same folder for further analysis in mathematica

NOTE: The TST cases are stored in the input folder and the EXEMPLAR_RUN cases are stored in output/walking/...

ALTERNATIVE USAGE:
changeAngles -i inputfile
This will load a text file of instructions.
The instructions file has the following:
Row 0 model filename (in form of ../models/...)
Row 1 filenames list filename (in form of ../input/...)
Row 2 trials list filename (in form of ../input/...)

the filenames are the names of text file data to be loaded into the simulation.  The trials file are the list of trial names, in case they are needed. q

PART 5.  Extract and organise data in Mathematica
For browsing data and for making preliminary plots use MJmodellingXX.nb
For creating figures for the paper (Fronteirs, 1st attempt) use MJmodelling_Paper_FigsXX.nb