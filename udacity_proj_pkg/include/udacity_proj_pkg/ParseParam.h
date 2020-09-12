#ifndef PARSE_PARAM_H
#define PARSE_PARAM_H

#include <Eigen/Dense>
#include <string>
#include <boost/algorithm/string/erase.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string.hpp>

class ParseParam
{
public:
	ParseParam();
	~ParseParam();

protected:
   Eigen::MatrixXd ConvertStringToDiagMatrix(std::string &string_);
   double ConvertStringToDouble(const std::string &string_);
     
	
};



#endif 