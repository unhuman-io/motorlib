#include "parameter_api.h"
#include "logger.h"
#include <charconv>
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
        variable_vector_.emplace_back(var);
        variable_name_vector_.emplace_back(name);
        auto_complete_.add_match_string(name);
    } else {
        logger.log_printf("API variable %s not in ROM, not adding, location: %p", std::string(name), name.data());
    }
}

void ParameterAPI::add_api_variable(std::string_view name, const APIVariable *var) {
    if (is_rom((void *) name.data())) {
        const_variable_vector_.push_back(var);
        const_name_vector_.push_back(name);
        auto_complete_.add_match_string(name);
    } else {
        logger.log_printf("API variable %s not in ROM, not adding, location: %p", std::string(name), name.data());
    }
}

bool ParameterAPI::set_api_variable(std::string_view name, std::string value) {
    uint16_t index = lookup_index_by_string(name);
    if (index < variable_vector_.size()) {
        variable_vector_[index]->set(value);
        return true;
    }
    return false;
}

bool ParameterAPI::set_api_variable(uint16_t index, std::string value) {
    if (index < variable_vector_.size()) {
        variable_vector_[index]->set(value);
        return true;
    }
    return false;
}

std::string ParameterAPI::get_api_variable(std::string_view name) {
    uint16_t index = lookup_index_by_string(name);
    return get_api_variable(index);
}

std::string ParameterAPI::get_api_variable(uint16_t index) {
    std::string out;
    if (index < variable_vector_.size()) {
        out = variable_vector_[index]->get();
    } else if (index < const_variable_vector_.size() + variable_vector_.size()) {
        out = const_variable_vector_[index-variable_vector_.size()]->get();
    }
    return out;
}

uint16_t ParameterAPI::lookup_index_by_string(std::string_view name) const {
    if (std::all_of(name.begin(), name.end(), ::isdigit)) {
        uint16_t index;
        std::from_chars_result result = std::from_chars(name.data(), name.data() + name.size(), index);
        logger.log_printf("from_chars result: %d, ec: %d", index, result.ec);
        if (result.ec == std::errc()) {
            return index;
        }
    }
    auto it = std::find(variable_name_vector_.begin(), variable_name_vector_.end(), name);
    if (it != variable_name_vector_.end()) {
        return std::distance(variable_name_vector_.begin(), it);
    }
    it = std::find(const_name_vector_.begin(), const_name_vector_.end(), name);
    if (it != const_name_vector_.end()) {
        return std::distance(const_name_vector_.begin(), it) + variable_vector_.size();
    }
    return variable_vector_.size() + const_variable_vector_.size();
}

std::string ParameterAPI::parse_string(std::string_view s) {
    std::string out;

    try {
        bool autocomplete = false;
        if (s.size() == 1 && !isdigit(s[0])) {
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
            } else if (variable == "api_index") {
                uint16_t index = lookup_index_by_string(value);
                if (index < get_api_length()) {
                    out = std::to_string(index);
                } else {
                    out = value + " index lookup error";
                }
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

uint16_t ParameterAPI::get_api_length() const {
    return variable_vector_.size() + const_variable_vector_.size();
}

std::string_view ParameterAPI::get_api_variable_name(uint16_t index) const {
    std::string retval = "";
    if (index < get_api_length()) {
        retval = variable_name_vector_[index];
    }
    return retval;
}

void APIFloat::set(std::string s) {
    *value_ = std::stof(s);
}
