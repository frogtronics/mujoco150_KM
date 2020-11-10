//  rollingJoint.h
//  Creates a "rolling joint" if called in the xml file.  
//  Uses three sliding joints to make a 2D ellipse path which can be traced in 3D
//  That ellipse path is meant to approximate the rolling surface of the joint.
//  In the XML, if there is a "_rollingX" (or ...Y or Z) after joint
//  name, this function is called.  First, it opens a file with the specific rolling joint parameters
//  whose filename is the name of the joint.  
//  e.g. j_kneeL_rollingX expects a corresponding file "j_kneeL_rolling.txt" in the input folder;
//  If the file is not found or invalid, the function 
//  returns a warning and the corresponding slide joints have a value of 0.
//  All values in local coordinates in the body where the joints reside
//  Created by Chris Richards, 10/2020
//  

#include "mujoco.h"
#include "math.h"
#include "string.h"
//#include "fileToolbox.h"

//The parameters file has the following format (note all vectors have values separated by commas)

// NOTE!! YOU CAN DISABLE THE ROLLING JOINT BY MAKING THE MAJOR RADIUS NEGATIVE

// XYZ coordinates of ellipse center in local coordinates of the body where the joint is
// major axis radius
// minor axis radius
// XYZ coordinates of normal vector (i.e. normal to the plane in which the ellipse is), loocal coordinates - needn't be normalised
// XYZ coordinates of a radial axis - this determines the axis of the major axis, loocal coordinates - needn't be normalised
// position offset (e.g. changes starting point along ellilse) - in units 0 - 1.
// most flexed angle (used py jointAngle2Roll) - the scalar joint angle corresponding to one extreme of the rolling range
// most extended angle (used py jointAngle2Roll) - the scalar joint angle corresponding to the opposite extreme of the rolling range
// roll range (used py jointAngle2Roll) - scalar range (radians) of displacement along ellipse.  E.g. if the joint rolls over half
//           of the ellipse, then scalar range would be Pi.

// function computes the centre of rotation of the rolling joint as a point on the ellipse

//result is XYZ centre of rotation in 
//filename is the name of the X (or Y or Z) slide joint returned from mj_id2name function
//position is the relative position along the ellipse, from 0 to 1 (converted to radians in the code)
void rollingCenter(mjtNum result[3], std::string filename, mjtNum position)
{
  //step 1.  Open parameter file and extract data into parameter_list
  std::string filename_rolling = "../input/" + (filename.substr(0, filename.size() - 1) ) + ".txt";
  int n_lines = numberOfLines(filename_rolling);
  //printf("%s   %i\n", filename_rolling.c_str(), n_lines);
  std::string *parameter_list = new std::string[n_lines];
  file2StringList(filename_rolling, parameter_list);
  //step 2. CHECK VALIDITY OF PARAMETER FILE 
  if (numberOfCharacters(parameter_list[0], ',') != 2)
  {
    printf("WARNING: ROLLING JOINT FAILED - first row of parameter file must have\n3 values separated by commas, XYZ center position\nHave a nice day\n");
    return;
  }
  else if (parameter_list[1] == "") 
  {
    printf("WARNING: ROLLING JOINT FAILED - second row of parameter file must have\n1 value = major axis radius\nHave a nice day\n");
    return;
  }
  else if (parameter_list[2] == "") 
  {
    printf("WARNING: ROLLING JOINT FAILED - third row of parameter file must have\n1 value = minor axis radius\nHave a nice day\n");
    return;
  }
  if (numberOfCharacters(parameter_list[3], ',') != 2)
  {
    printf("WARNING: ROLLING JOINT FAILED - fourth row of parameter file must have\n3 values separated by commas, XYZ norm axis\nHave a nice day\n");
    return;
  }
  if (numberOfCharacters(parameter_list[4], ',') != 2)
  {
    printf("WARNING: ROLLING JOINT FAILED - fifith row of parameter file must have\n3 values separated by commas, XYZ radial axis\nHave a nice day\n");
    return;
  }

  //step 3.  Extract vectors and normalise them as appropriate
  std::string center_string_list[3];
  CSVstring2list(center_string_list, parameter_list[0]);
  mjtNum center_XYZ[3];
  mjtNum norm_XYZ[3];
  mjtNum major_XYZ[3];
  CSVstring2values(center_XYZ, parameter_list[0]);
  CSVstring2values(norm_XYZ, parameter_list[3]);
  CSVstring2values(major_XYZ, parameter_list[4]);

  //normalise
  mju_normalize3(norm_XYZ);
  mju_normalize3(major_XYZ);

  //step 4. Find minor axis
  mjtNum minor_XYZ[3];
  mju_cross(minor_XYZ, major_XYZ, norm_XYZ);

  //step 5.  Build terms and find position on ellipse, about origin
  mjtNum pos = position;
  mjtNum r_major = stod(parameter_list[1]);
  mjtNum r_minor = stod(parameter_list[2]);

  bool isEnabled = r_major > 0;//rolling joint is only enabled if the major axis is positive

  if (pos < 0)
  pos = 0;
  if (pos > 1)
  pos = 1;

  mjtNum pos_offset_raw = stod(parameter_list[5]);

  if (pos_offset_raw < 0)
  pos_offset_raw = 0;
  if (pos_offset_raw > 1)
  pos_offset_raw = 1;

  mjtNum pos_offset = pos_offset_raw * 6.28;
  mjtNum pos_rad = pos * 6.28 + pos_offset;
  mjtNum ellipse_XYZ_nonOffset[3];
  mjtNum aCosTheta = r_major * cos(pos_rad);
  mjtNum bSinTheta = r_minor * sin(pos_rad);

  mjtNum termA[3];
  mjtNum termB[3];

  mju_scl3(termA, major_XYZ, aCosTheta);
  mju_scl3(termB, minor_XYZ, bSinTheta);

  mju_add3(ellipse_XYZ_nonOffset, termA, termB);

  //step 6. offset to desired origin in local coordinates
  mjtNum ellipse_XYZ[3];
  //printf("%f %f %f\n", ellipse_XYZ[0], ellipse_XYZ[1], ellipse_XYZ[2] );
  if (isEnabled)
  {
  mju_add3(ellipse_XYZ, ellipse_XYZ_nonOffset, center_XYZ);
  }

  mju_copy3(result, ellipse_XYZ);


}

// a somewhat specific function which converts a scalar joint angle value (e.g. flexion-extension angle, radians) 
// to a position along the ellipse  
// accepts data from the final three lines of the parameter file
// Note joint extension ranges come from min/max for jumping from Porro et al. 2017, 28 deg min, 146 deg max for knee (3D angle).  The 
// range of the rolling motion of 75 degrees is based on Kargo and Rome.

// outputs centre of ellipse as above
void jointAngle2Roll (mjtNum result[3], std::string filename, mjtNum joint_angle_rad)
{
  //Step 1. Open parameter file and pull out last three lines
  std::string filename_rolling = "../input/" + (filename.substr(0, filename.size() - 1) ) + ".txt";
  int n_lines = numberOfLines(filename_rolling);
  std::string *parameter_list = new std::string[n_lines];
  file2StringList(filename_rolling, parameter_list);
  mjtNum min_joint_angle = stod(parameter_list[6]);
  mjtNum max_joint_angle = stod(parameter_list[7]);
  mjtNum roll_range = stod(parameter_list[8]);
  //Step 2. Enforce the min/max range of the input angle
  if (joint_angle_rad < min_joint_angle)
    joint_angle_rad = min_joint_angle;
  if (joint_angle_rad > max_joint_angle)
    joint_angle_rad = max_joint_angle;

  //printf("%f %f %f \n", min_joint_angle, max_joint_angle, roll_range);
  //Step 3. Convert angle parameters to position on ellipse
  mjtNum roll_gain = roll_range / (max_joint_angle - min_joint_angle);
  mjtNum roll_pos = roll_gain * (joint_angle_rad - min_joint_angle) / 6.28; //position along the ellipse

  rollingCenter(result, filename, roll_pos);
  //printf("%f\n", roll_pos);


}
