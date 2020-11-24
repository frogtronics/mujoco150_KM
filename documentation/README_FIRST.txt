How to get started:

To all users

1. Download the entire repository and store it locally.  Please do not alter the content/location of folders unless you know what you're doing.

2. OPTIONAL: If you'd like to stay updated with later versions of this repository or experiment with your own changes, create a fork using your own GitHub.

3. Open a command line terminal

4. change the directory to _OSX or _WIN or _LINUX as appropriate for your operating system
- for MAC, type
cd your/directory/goes/here/_OSX 
  for WIN, type
cd your\directory\goes\here\_WIN

5.  Run the changeAngles program which renders kinematics from a chosen trial then saves the data to an arbitrary output file.

./changeAngles -m ../models/Kassina/Kassina.xml -i ../input/KM08_RUN_09.txt -t ../output/tendondata.txt  

OR

./changeAngles -i ../input/instructionsFile.txt

Where the instructions file is a list of filenames pointing to the trials to use, etc.  See README_workflow.txt

- for WIN, type

changeAngles -m ..\models\Kassina\Kassina.xml -i ..\input\KM08_RUN_09.txt -t ..\output\tendondata.txt 

OR

changeAngles -i ..\input\instructionsFile.txt

NOTE: the simulation will take a couple seconds to render then show the kinematics which will loop indefinitely until you close the window.  Press space to pause.

6. What to read next: README_inputdata.txt


To authors of the study
(Amber Collings, Enrico Eberhard, Chris Basu, Chris Richards)
The repository will be curated by C. Richards.  You will be asked to be contributors if you wish (no worries if not, just do the above steps if you like).  If you would like to contribute using your GH account, you can create your own branch with proposed changes, publish it and generate a pull request which I'll later merge. 
