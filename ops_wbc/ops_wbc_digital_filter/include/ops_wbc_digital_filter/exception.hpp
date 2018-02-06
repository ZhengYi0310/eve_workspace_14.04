#ifndef __OPS_WBC_DIGITAL_FILTER_EXCEPTION_HPP
#define __OPS_WBC_DIGITAL_FILTER_EXCEPTION_HPP

#include <sstream>

namespace ops_wbc_digital_filter
{
    class Exception
    {
        public:
            explicit Exception(const std::string& src, const std::string& msg) throw()
                            : src_(src), msg_(msg)
                    {}

            const char* what() const throw()
            {
                std::stringstream ss;

                ss << "ops_wbc_filter::Exception occurred." << std::endl
                << "  src : " << src_ << std::endl
                << "  msg : " << msg_;

                return ss.str().c_str();
            }

        private:
            std::string src_;
            std::string msg_;
    };
}

#endif // __OPS_WBC_DIGITAL_FILTER_EXCEPTION_HPP
