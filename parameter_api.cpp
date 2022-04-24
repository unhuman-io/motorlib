#include "parameter_api.h"
//#include <regex>
//#include <sstream>

static std::string trim(std::string s)
{
    auto first = s.find_first_not_of(' ');
    auto last = s.find_last_not_of(' ');
    return s.substr(first, (last-first+1));
}

void ParameterAPI::add_api_variable(std::string name, APIVariable *var) {
    variable_map_[name] = var;
    auto_complete_.add_match_string(name);
}

void ParameterAPI::add_api_variable(std::string name, const APIVariable *var) {
    const_variable_map_[name] = var;
    auto_complete_.add_match_string(name);
}

void ParameterAPI::set_api_variable(std::string name, std::string value) {
    if (variable_map_.count(name))  {
        variable_map_[name]->set(value);
    }
}

std::string ParameterAPI::get_api_variable(std::string name) {
    std::string out;
    if (variable_map_.count(name)) {
        out = variable_map_[name]->get();
    } else if (const_variable_map_.count(name)) {
        out = const_variable_map_[name]->get();
    }
    return out;
}

std::string ParameterAPI::parse_string(std::string s) {
    std::string out;

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
        set_api_variable(variable, value);
        out = variable + " set " + value;
    } else {
        out = get_api_variable(s);
    }
    
    if (autocomplete) {
        return "\n" + out + "\n";
    } else {
        return out;
    }
}

std::string ParameterAPI::get_all_api_variables() const {
    std::string s;
    s = std::to_string(variable_map_.size() + const_variable_map_.size()) + " variables:\n";
    for(auto const& m : variable_map_) {
        s += m.first + "\n";
    }
    for(auto const& m : const_variable_map_) {
        s += m.first + "\n";
    }
    return s;
}

void APIFloat::set(std::string s) {
    *value_ = std::stof(s);
}
