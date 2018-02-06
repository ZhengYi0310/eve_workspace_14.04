/*************************************************************************
	> File Name: str_utils.cpp
	> Author: Yi Zheng 
	> Mail: hchengcq@gmail.com
	> Created Time: Wed 13 Dec 2017 05:17:05 PM PST
 ************************************************************************/

#include <cctype>
#include <iostream>
#include <boost/tokenizer.hpp>
#include <boost/algorithm/string.hpp>
#include "utils/str_utils.hpp"

using namespace std;
using namespace ops_wbc_utils;

bool StrUtils::have(const std::string& str, char ch)
{
    uint32_t length = str.length();
    for (uint32_t i = 0; i < length; i++)
    {
        if (str[i] == ch)
            return true;
    }

    return false;
}

bool StrUtils::have(const std::string& str, const std::vector<char>& chs)
{
    uint32_t size = chs.size();

    for (uint32_t i = 0; i < size; i++)
    {
        if (!StrUtils::have(str, chs[i]))
            return false;
    }

    return true;
}

bool StrUtils::have(const std::string& str, const std::string& word)
{
    return boost::contains(str, word);
}

bool StrUtils::isAlphabet(const std::string& str)
{
    uint32_t length = str.length();
    
    if (length == 0)
        return false;

    for (uint32_t i = 0; i < length; i++)
    {
        if (!std::isalpha(str[i]))
            return false;
    }

    return true;
}

bool StrUtils::isIPAddress(const std::string& str)
{
    std::string temp = str;
    StrUtils::removeSpace(temp);
    std::vector<std::string> words;

    StrUtils::separate(temp, words, ".:");

    if (words.size() != 4)
        return false;

    for (int32_t i = 0; i < words.size(); i++)
    {
        for (int32_t j = 0; j < words[i].length(); j++)
        {
            if (words[i][j] < '0' || words[i][j] > '9')
                return false;
        }

        if (std::atoi(words[i].c_str()) < 0 || std::atoi(words[i].c_str()) > 255)
            return false;
    }
    
    return true;
}

void StrUtils::removeSpace(std::string& str)
{
    size_t pos;
    while ((pos = str.find_first_of(" 　\t")) != std::string::npos)
    {
        str.erase(pos, 1);
    }
}

void StrUtils::removeIndent(std::string& str)
{
    for (uint32_t i = 0; i < str.length(); i++)
    {
        if (str[i] == ' ' || str[i] == '\t')
        {
            str.erase(i, 1);
            i--;
        }
    }

    return;
}

void StrUtils::remove(std::string& str, char ch)
{
    for (uint32_t i = 0; i < str.length(); i++)
    {
        if (str[i] == ch)
        {
            str.erase(i , 1);
            i--;
        }
    }
}

void StrUtils::separate(const std::string& str, std::vector<std::string>& words, const std::string& delimiters)
{
    typedef boost::char_separator<char> char_separator;
    typedef boost::tokenizer<char_separator> tokenizer;

    char_separator sep(delimiters.c_str(), "", boost::keep_empty_tokens);
    tokenizer tokens(str, sep);

    words.clear();
    for (auto it = std::begin(tokens); it != std::end(tokens); it++)
    {
        if (*it != "")
            words.push_back(*it);
    }
}
