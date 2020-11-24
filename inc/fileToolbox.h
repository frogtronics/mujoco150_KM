//  fileToolbox.h
//  
//
//  Created by Chris Richards, 2/2018
//  

#include <cstdlib>
#include <iostream>
#include <time.h>
#include <string>
#include <sstream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <iterator>
#include <iomanip>

//-----------------  MISC string parsing and finding functions



// --- just to get the end of a sring....

std::string tail(std::string const& source, size_t const length) {
  if (length >= source.size()) { return source; }
  return source.substr(source.size() - length);
} // tail

//count number of characters in a string
int numberOfCharacters(std::string s, char character) 
{
  int count = 0;

  for (int i = 0; i < s.size(); i++)
    if (s[i] == character) count++;

  return count;
}

void CSVstring2list(std::string* string_list, std::string s)
{

  std::string delimiter = ",";
  size_t pos = 0;
  std::string token;
  int n_commas = numberOfCharacters(s, delimiter[0]);
   for ( int i = 0; i < n_commas + 1; i ++ ) 
  {
      pos = s.find(delimiter);
      token = s.substr(0, pos);
      string_list[i] = token;
      s.erase(0, pos + delimiter.length());
  }

}

void CSVstring2values(mjtNum* value_list, std::string s)
{
  std::string delimiter = ",";
  size_t pos = 0;
  std::string token;
  int n_commas = numberOfCharacters(s, delimiter[0]);
   for ( int i = 0; i < n_commas + 1; i ++ ) 
  {
      pos = s.find(delimiter);
      token = s.substr(0, pos);
      value_list[i] = std::stod(token);
      s.erase(0, pos + delimiter.length());
  }
}

//copied code for finding replacing all occurrences
// std::string teststr = "this is my string";
// findAndReplaceAll(teststr, "my", "your");
// >> this is your string
void findAndReplaceAll(std::string & data, std::string toSearch, std::string replaceStr)
{
    // Get the first occurrence
    size_t pos = data.find(toSearch);
    // Repeat till end is reached
    while( pos != std::string::npos)
    {
        // Replace this occurrence of Sub String
        data.replace(pos, toSearch.size(), replaceStr);
        // Get the next occurrence from the current position
        pos =data.find(toSearch, pos + replaceStr.size());
    }
}

// returns number of lines in a text file
int numberOfLines (const std::string filename) { 
    int number_of_lines = 0;
    std::string line;
    std::ifstream inputFile(filename);

    while (std::getline(inputFile, line)) {
       ++number_of_lines;
    }
    
    return number_of_lines;
}

//converts a multi-line text file to a vector of strings
// returns the number of lines
int file2StringList (const std::string filename, std::string* string_list) {
	int number_of_lines = 0;
    std::string line;
    std::ifstream inputFile(filename);

    while (std::getline(inputFile, line)) {
    	string_list[number_of_lines] = line.c_str();
    	++number_of_lines;
    }
    
    return number_of_lines;
}



//load data from files listed in a textfile. data has the length n_files_in_list * n_lines_in_each_file
//function returns the number of files in list

int loadDataFromFileList (const std::string filelistname, double* data) {

  //std::ifstream inputFile("filelist.txt");
  std::ifstream inputFile(filelistname);
  std::vector<std::string> fileList;
  std::string line;

  if(!inputFile) {
    std::cerr << "fileToolbox::loadDataFromFileListFile list could not be opened\n";
    return 1;
  }
  int n_files = 0;
  while(std::getline(inputFile, line)) {
  	++n_files;
    fileList.push_back(line);
  }

  std::cout << "Number of files to be analyzed: " << fileList.size() << "\n";

  std::vector<std::string>::const_iterator it(fileList.begin());
  std::vector<std::string>::const_iterator end(fileList.end());
  int itr_file = 0;
  int n_lines_total = 0;
  for(;it != end;++it) {
    std::ifstream inputTxt(it->c_str());

    if(!inputTxt) {
      std::cerr << "fileToolbox::loadDataFromFileList::Data file could not be opened:" << *it << "\n";
      return 1;
    }
    int itr = 0;
    
    while(std::getline(inputTxt, line)) {
    	std::string::size_type sz;     // alias of size_t
  		double teststr_number = std::stof (line,&sz);
  		data[n_lines_total] = teststr_number;
  		n_lines_total ++;
    }
    
    itr_file ++;
  }
  return n_files;
}

//ref http://www.cplusplus.com/forum/general/15952/
std::string integerString(int num, int width)
{
    std::ostringstream ss;
    ss << std::setw( width ) << std::setfill( '0' ) << num;
    return ss.str();
}