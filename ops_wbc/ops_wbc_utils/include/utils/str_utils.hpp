#ifndef __OPS_WBC_UTILS_STR_UTILS_HPP
#define __OPS_WBC_UTILS_STR_UTILS_HPP

#include <string>
#include <sstream>
#include <vector>

namespace ops_wbc_utils  
{
    class StrUtils
    {
        public:
            static bool have(const std::string& str, char ch);
            static bool have(const std::string& str, const std::vector<char>& chs);
            static bool have(const std::string& str, const std::string& word);
            static bool isAlphabet(const std::string& str);

            static bool convertToBoolean(const std::string& str, bool& dst)
            {
                std::stringstream ss(str);
                std::string tmp;

                if (!(ss >> tmp))
                    return false;
                
                if(tmp == std::string("true") || tmp == std::string("TRUE"))
                {
                    dst = true;
                }
                else if(tmp == std::string("false") || tmp == std::string("FALSE"))
                {
                    dst = false;
                }
                else
                {
                    return false;
                }
                return true;
            }

            template <class T>
            static bool convertToNum(const std::string& str, T& dst)
            {
                std::stringstream ss(str);
                double tmp;

                if(!(ss >> tmp))
                {
                    return false;
                }
                dst = static_cast<T>(tmp);
                return true;
            }

            template <class T>
            static bool convertToVector(const std::string& str, std::vector<T>& vec, const std::string& delimiters)
            {
                std::vector<std::string> words;
                StrUtils::separate(str, words, delimiters);

                if (words.size() == 0)
                    return false;
                
                vec.resize(words.size());
                for (uint32_t i = 0; i < words.size(); i++)
                {
                    if (!StrUtils::convertToNum<T>(words[i], vec[i]))  
                        return false;
                }
                return true;
            }

            static bool isIPAddress(const std::string& str);
            static void removeSpace(std::string& str);
            static void removeIndent(std::string& str);
            static void remove(std::string& str, char ch);
            static void separate(const std::string& str, std::vector<std::string>& words, const std::string& delimiters);
    };
}
#endif // __OPS_WBC_UTILS_STR_UTILS_HPP
