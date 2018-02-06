/*************************************************************************
	> File Name: io_utils.cpp
	> Author: Yi Zheng  
	> Mail: hczhengcq@gmail.com
	> Created Time: Thu 14 Dec 2017 12:27:43 PM PST
 ************************************************************************/

#include <iostream>
#include "utils/io_utils.hpp"
using namespace std;
using namespace ops_wbc_utils;

void IOUtils::print(const Eigen::MatrixXd& m)
{
    std::cout << "[ ";
    for(uint32_t i = 0; i < m.rows(); ++i)
    {
        for(uint32_t j = 0; j < m.cols(); ++j)
        {
            std::cout << m.coeff(i, j);
            if(j < m.cols() - 1)
                std::cout << ", ";
        }
        if(i < m.rows() - 1)
            std::cout << std::endl;
    }
    std::cout << " ]" << std::endl;
}

bool IOUtils::getValues(std::ifstream& ifs, Eigen::MatrixXd& dst)
{
    if (!ifs)
        return false;

    std::string str;
    if (!std::getline(ifs, str))
        return false;

    std::vector<std::string> words;
    StrUtils::separate(str, words, ",;: \t");

    dst = Eigen::MatrixXd::Zero(words.size(), 1);
    for (uint32_t i = 0; i < words.size(); i++)
    {
        StrUtils::convertToNum(words[i], dst.coeffRef(i, 0));
    }

    return true;
}

bool IOUtils::getValues(std::ifstream& ifs, std::vector<Eigen::MatrixXd>& dst)
{
  Eigen::MatrixXd temp;
  while(IOUtils::getValues(ifs, temp))
  {
    dst.push_back(temp);
  }

  if(dst.size() == 0)
    return false;

  return true;
}

unsigned long IOUtils::getLineNum(std::ifstream& ifs)
{
}

