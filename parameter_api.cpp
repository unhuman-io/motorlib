#include "parameter_api.h"
#include "logger.h"
//#include <regex>
//#include <sstream>

static std::string trim(std::string_view s)
{
    auto first = s.find_first_not_of(' ');
    auto last = s.find_last_not_of(' ');
    return std::string(s.substr(first, (last-first+1)));
}

void ParameterAPI::add_api_variable(std::string_view name, APIVariable *var) {
    if (is_rom((void *) name.data())) {
        variable_map_[name] = var;
        auto_complete_.add_match_string(name);
    } else {
        logger.log_printf("API variable %s not in ROM, not adding, location: %p", std::string(name), name.data());
    }
}

void ParameterAPI::add_api_variable(std::string_view name, const APIVariable *var) {
    if (is_rom((void *) name.data())) {
        const_variable_map_[name] = var;
        auto_complete_.add_match_string(name);
    } else {
        logger.log_printf("API variable %s not in ROM, not adding, location: %p", std::string(name), name.data());
    }
}

bool ParameterAPI::set_api_variable(std::string_view name, std::string value) {
    if (variable_map_.count(name))  {
        variable_map_[name]->set(value);
        return true;
    }
    return false;
}

std::string ParameterAPI::get_api_variable(std::string_view name) {
    std::string out;
    if (variable_map_.count(name)) {
        out = variable_map_[name]->get();
    } else if (const_variable_map_.count(name)) {
        out = const_variable_map_[name]->get();
    }
    return out;
}

std::string ParameterAPI::parse_string(std::string_view s) {
    std::string out;

    try {
        bool autocomplete = false;
        if (s.size() == 1) {
            autocomplete = true;
            out = auto_complete_.autocomplete(s[0]);
            if (out == "\n") {
                s = auto_complete_.last_string();
            } else {
                return out;
            }
        }

        auto equal_pos = s.find("=");
        if (equal_pos != std::string::npos) {
            auto variable = trim(s.substr(0,equal_pos));
            auto value = trim(s.substr(equal_pos+1));
            if (variable == "api_name") {
                out = get_api_variable_name(std::stoi(value));
            } else {
                if (set_api_variable(variable, value)) {
                    out = variable + " set " + value;
                } else {
                    out = variable + " error";
                }
                
            }
        } else {
            out = get_api_variable(s);
        }
        
        if (autocomplete) {
            return "\n" + out + "\n";
        } else {
            return out;
        }
    } catch(...) {
        return "error";
    }
}

std::string ParameterAPI::get_all_api_variables() const {
    std::string s;
    s = std::to_string(variable_map_.size() + const_variable_map_.size()) + " variables:\n";
    for(auto const& m : variable_map_) {
        s += std::string(m.first) + "\n";
    }
    for(auto const& m : const_variable_map_) {
        s += std::string(m.first) + "\n";
    }
    return s;
}

uint16_t ParameterAPI::get_api_length() const {
    return variable_map_.size() + const_variable_map_.size();
}

std::string ParameterAPI::get_api_variable_name(uint16_t index) const {
    std::string retval = "";
    if (index < get_api_length()) {
        if (index >= variable_map_.size()) {
            auto it = const_variable_map_.begin();
            std::advance(it, index - variable_map_.size());
            retval = it->first;
        } else {
            auto it = variable_map_.begin();
            std::advance(it, index);
            retval = it->first;
        }
    }
    return retval;
}

void APIFloat::set(std::string s) {
    *value_ = std::stof(s);
}
