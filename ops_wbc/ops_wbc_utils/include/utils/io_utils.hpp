#ifndef __OPS_WBC_UTILS_IO_UTILS_HPP
#define __OPS_WBC_UTILS_IO_UTILS_HPP

#include <iostream>
#include <fstream>
#include <vector>
#include <limits>
#include <Eigen/Dense>
#include "utils/str_utils.hpp"

namespace ops_wbc_utils
{
    class IOUtils
    {
        public:
            static void print(const Eigen::MatrixXd& m);

            template<class T>
            static void print(const std::vector<T>& v)
            {
                std::cout << "[ ";
                for (uint32_t i = 0; i < v.size(); i++)
                {
                    std::cout << v[i];
                    if (i < v.size() - 1)
                    {
                        std::cout << ", ";
                    }
                }
                std::cout << " ]" << std::endl;
            }

            template<class T>
            static void print(const std::vector<std::vector<T> >& m)
            {
                std::cout << "[ ";
                for(uint32_t i = 0; i < m.size(); ++i)
                {
                    for(uint32_t j = 0; j < m[i].size(); ++j)
                    {
                        std::cout << m[i][j];
                        if(j < m[i].size() - 1)
                        std::cout << ", ";
                    }
                    if(i < m.size() - 1)
                        std::cout << std::endl;
                }
                std::cout << " ]" << std::endl;
            }

            static bool getValues(std::ifstream& ifs, Eigen::MatrixXd& dst);
            static bool getValues(std::ifstream& ifs, std::vector<Eigen::MatrixXd>& dst);

            template<class T>
            static bool getValues(std::ifstream& ifs, std::vector<T>& dst)
            {
                if (!ifs)
                    return false;
                
                std::string str;
                if (!std::getline(ifs, str))
                    return false;
                
                return StrUtils::convertToVector(str, dst, ",;: \t");
            }

            template<class T>
            static bool getValues(std::ifstream& ifs, std::vector<std::vector<T> >& dst)
            {
                dst.clear();

                std::vector<T> tmp;
                while (IOUtils::getValues(ifs, tmp))
                {
                    dst.push_back(tmp);
                }

                if (dst.size() == 0)
                    return false;

                return true;
            }

            static unsigned long getLineNum(std::ifstream& ifs);
    };
}
#endif // __OPS_WBC_UTILS_IO_UTILS_HPP
