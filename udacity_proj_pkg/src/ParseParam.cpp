#include "udacity_proj_pkg/ParseParam.h"
//Eigen::Vector3d v2(v1.data());

ParseParam::ParseParam()
{
}

ParseParam::~ParseParam()
{
}//
 
Eigen::MatrixXd ParseParam::ConvertStringToDiagMatrix( std::string &string_)
{
    std::vector<std::string> names;
    boost::erase_all(string_, " ");
    boost::split(names, string_, boost::is_any_of(","));
    std::vector<double> tmp;
    for(int i =0; i < names.size(); ++i)
    {
        tmp.push_back(stod(names[i]));

    }
    //Eigen::VectorXd dummy(tmp.data());
    Eigen::VectorXd dummy = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(tmp.data(), tmp.size());
    return dummy.asDiagonal();
}

double ParseParam::ConvertStringToDouble(const std::string &string_)
{
    return stod(string_);
}