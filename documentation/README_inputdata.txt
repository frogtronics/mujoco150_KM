Input data is in terms of quaternions for ..models/Kassina/Kassina.xml model
See README_workflow.txt for information on how these relate to raw kinematics data

Hop = jumping data (recorded by L. Porro, E. Eberhard, 2016 [1])
Run = walking data (recorded by A. Collings, L. Porro, K. Chadwick, C. Richards, 2015 [1,2])

"fixed" refers to hypothetical pelvis which has been mathematically "glued" such that there is no lateral rotation [3].  For all other cases, all global translation/rotational motions of the body have been removed, but otherwise no further alterations to kinematics.  See publications for details.

NOTE: You can make a list of muscle paths to highlight. This is done by modifying the muscle_highlight.txt.  This will colour only those paths and all other muscles will be transparent (they will still be present in the model).  For an example, see muscle_highlight_example.txt (this is just to show you how to write the file - the example file is ignored by the simulator).


See README_workflow.txt for information on how quaternions files were generated

[1]Richards, C. T., Porro, L. B., & Collings, A. J. (2017). Kinematic control of extreme jump angles in the red-legged running frog, Kassina maculata. Journal of Experimental Biology, 220(10), 1894-1904.

[2]Porro, L. B., Collings, A. J., Eberhard, E. A., Chadwick, K. P., & Richards, C. T. (2017). Inverse dynamic modelling of jumping in the red-legged running frog, Kassina maculata. Journal of Experimental Biology, 220(10), 1882-1893.

[3]Collings, A. J., Porro, L. B., Hill, C., & Richards, C. T. (2019). The impact of pelvic lateral rotation on hindlimb kinematics and stride length in the red-legged running frog, Kassina maculata. Royal Society open science, 6(5), 190060.

