Notes: 

Muscle groupings, last modified 6th August 2021:

CIdist, IL1, IEprox, AMcrv, and IImed.

LCI = LCIdist;
RCI = RCIdist;
LIL = LIL1;
RIL = RIL1;
LIE = LIEprox;
LAM = LAMcrv;
LII = LIImed;

axialGroup = {"LCI", "LIL", "RCI", "RIL"};
protractorGroup = {"LIE", "LSA", "LAL", "LAM", "LII"};
retractorGroup = {"LSM", "LIFB", "LOE", "LGR", "LIFM"};
miscGroup = {"LPY", "LGL", "LCR"};


Filenames are muscles with _ degree of freedom for that given moment arm
L = left; R = right
e.g.
LCR_hipL  means left cruralis moment arm about left hip

LAR - long axis rotation
FE - Flexion extension
AA - Abduction adduction

ISY is dof of pelvic lateral rotation

Note: static femur angle does not impact moment arms of intrinsic pelvic muscles
Likewise, static ISX angle does not impact moment arms of leg muscles

For further information, see documentation\README_inputdata.txt and README_outputdata.txt