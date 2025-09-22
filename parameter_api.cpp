#include "parameter_api.h"
#include "logger.h"
//#include <regex>
//#include <sstream>
#include <cstring>

uint32_t ParameterAPI::AllocatorBase::index_ = 0;
uint32_t ParameterAPI::AllocatorBase::mem_[API_SIZE] __attribute((section(".bss.api")));

static std::string trim(std::string_view s)
{
    auto first = s.find_first_not_of(' ');
    auto last = s.find_last_not_of(' ');
    return std::string(s.substr(first, (last-first+1)));
}

void ParameterAPI::add_api_variable(std::string_view name, APIVariable *var) {
    if (is_rom((void *) name.data())) {
        try {
            variable_map_.emplace(name, var);
        } catch (const std::bad_alloc &e) {
            logger.log_printf("Error adding variable %s (%d): %s", std::string(name).c_str(), variable_map_.size(), e.what());
            return;
        }
        auto_complete_.add_match_string(name);
    } else {
        logger.log_printf("API variable %s not in ROM, not adding, location: %p", std::string(name).c_str(), name.data());
    }
}

void ParameterAPI::add_api_variable(std::string_view name, const APIVariable *var) {
    if (is_rom((void *) name.data())) {
        try {
            const_variable_map_.emplace(name, var);
        } catch (const std::bad_alloc &e) {
            logger.log_printf("Error adding const variable %s (%d): %s", std::string(name).c_str(), const_variable_map_.size(), e.what());
            return;
        }
        auto_complete_.add_match_string(name);
    } else {
        logger.log_printf("API variable %s not in ROM, not adding, location: %p", std::string(name).c_str(), name.data());
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

        // GDB commands
        if (s.rfind("$", 0) == 0) {
            auto command = trim(s.substr(1));
            if (command.rfind("m", 0) == 0) {
                auto comma_pos = command.find(",");
                if (comma_pos != std::string::npos) {
                    auto length_str = trim(command.substr(comma_pos + 1));
                    auto address_str = trim(command.substr(1, comma_pos - 1));
                    uint32_t address = std::stoul(address_str, nullptr, 16);
                    uint32_t length = std::stoul(length_str, nullptr, 16);
                    std::vector<char>bytes((char *) address,(char *) address+length);
                    out += bytes_to_hex(bytes);
                }
            } else if (command.rfind("M", 0) == 0) {
                auto comma_pos = command.find(",");
                if (comma_pos != std::string::npos) {
                    auto colon_pos = command.find(":", comma_pos + 1);
                    if (colon_pos != std::string::npos) {
                        auto address_str = trim(command.substr(1, comma_pos - 1));
                        auto length_str = trim(command.substr(comma_pos + 1, colon_pos - comma_pos - 1));
                        auto data_str = trim(command.substr(colon_pos + 1));
                        uint32_t address = std::stoul(address_str, nullptr, 16);
                        uint32_t length = std::stoul(length_str, nullptr, 16);
                        std::vector<char> bytes = hex_to_bytes(data_str);
                        if (bytes.size() == length) {
                            std::memcpy((void *) address, bytes.data(), length);
                            out = "OK";
                        } else {
                            out = "E00";
                        }
                    }
                }
            }
            return out;
        } else {
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

struct ContextState {
  unsigned int r0, r1, r2, r3;
  unsigned int r12, lr;
  unsigned int return_address;
  unsigned int xpsr;
};

struct GDBRegisters {
  unsigned int r0, r1, r2, r3, r4, r5, r6, r7, r8, r9, r10, r11, r12;
  unsigned int sp, lr, pc;
  unsigned int xpsr;
};

std::string make_reg_string(ContextState *context_state) {
  unsigned int *sp = (unsigned int *) context_state;
  register unsigned int r4 asm ("r4");
  register unsigned int r5 asm ("r5");
  register unsigned int r6 asm ("r6");
  register unsigned int r7 asm ("r7");
  register unsigned int r8 asm ("r8");
  register unsigned int r9 asm ("r9");
  register unsigned int r10 asm ("r10");
  register unsigned int r11 asm ("r11");
  register unsigned int r12 asm ("r12");
  GDBRegisters regs;
  regs.r0 = context_state->r0;
  regs.r1 = context_state->r1;
  regs.r2 = context_state->r2;
  regs.r3 = context_state->r3;
  regs.r4 = r4;
  regs.r5 = r5;
  regs.r6 = r6;
  regs.r7 = r7;
  regs.r8 = r8;
  regs.r9 = r9;
  regs.r10 = r10;
  regs.r11 = r11;
  regs.r12 = r12;
  regs.sp = (unsigned int) sp;
  regs.lr = context_state->lr;
  regs.pc = context_state->return_address;
  regs.xpsr = context_state->xpsr;
  std::vector<char> bytes((char *) &regs,(char *) &regs + sizeof(GDBRegisters));
  return bytes_to_hex(bytes);
}

extern "C" __attribute__((used))
void debug_mon_handler(ContextState *context_state) {
  std::string reg_string = make_reg_string(context_state);
  logger.log(reg_string);
}
