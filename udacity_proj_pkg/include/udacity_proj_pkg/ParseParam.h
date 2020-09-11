#ifndef PARSE_PARAM_H
#define PARSE_PARAM_H

#include <Eigen/Dense>
class ParseParam
{
public:
	ParseParam();
	~ParseParam();

   Eigen::MatrixXd ConvertStringToDiagMatrix(std::string string_);
   double ConvertStringToDouble(dummy);
     
	
};



#endif 