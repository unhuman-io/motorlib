#ifndef UNHUMAN_MOTORLIB_AUTOCOMPLETE_H_
#define UNHUMAN_MOTORLIB_AUTOCOMPLETE_H_

#include <string_view>
#include <vector>
#include <algorithm>

// return longest common string starting at the beginning 
inline std::string max_string_match(std::string_view s1, std::string_view s2) {
    // I couldn't think of a stl way do this
    unsigned int i;
    for (i=0; i<std::min(s1.size(), s2.size()); i++) {
        if (s1[i] != s2[i]) {
            break;
        }
    }
    return std::string(s1.substr(0, i));
}

class APIVariable;
class AutoComplete {
 public:
    void add_match_string(std::string_view s) {
    }
    std::string autocomplete(char c) {
        std::vector<std::string_view> matches;
        std::string str_out;
        switch (c) {
            case '\t': {
                auto &str = str_;
                if (str.size() > 0) { // todo not enough memory to do all strs
                    std::copy_if(strs_->begin(), strs_->end(), [](std::back_inserter(matches), [&str](std::map<std::string_view, APIVariable *>::const_iterator &s){return (s->first).rfind(str,0)==0;});
                    if (matches.size() == 1) {
                        str_ = matches[0];
                        str_out = '\r' + str_;
                    } else if (matches.size()) {
                        std::string max_match = std::string(matches[0]);
                        if (last_key_ == '\t') {
                            str_out = '\n';
                            for (auto &match : matches) {
                                max_match = max_string_match(max_match, match);
                                str_out += std::string(match) + '\t';
                            }
                            // move str_ out to farthest in common characters
                            str_ = max_match;
                            str_out += '\n' + str_;
                        } else {
                            for (auto &match : matches) {
                                max_match = max_string_match(max_match, match);
                            }
                            str_ = max_match;
                            str_out = '\r' + str_;
                        }
                    }
                }
                break;
            }
            case '\n':
                str_out = '\n';
                if (str_ != "") {
                    last_str_ = str_;
                }
                str_ = "";
                break;
            case 127:
                if (str_.size()) {
                    str_.erase(str_.end()-1,str_.end());
                    str_out = "\b \b";
                }
                break;
            default:
                str_ += c;
                str_out = c;
                break;
        }
        last_key_ = c;
        return str_out;
    }
    void set_strs(std::map<std::string_view, APIVariable *> * map) { strs_ = map; }
    std::string_view last_string() const { return last_str_; }
 private:
    char last_key_ = 0;
    std::string str_, last_str_;
    std::map<std::string_view, APIVariable *> * strs_;
};

#endif  // UNHUMAN_MOTORLIB_AUTOCOMPLETE_H_
