#ifndef UNHUMAN_MOTORLIB_PARAMETER_API_H_
#define UNHUMAN_MOTORLIB_PARAMETER_API_H_

#include <string_view>
#include <map>
#include <vector>
#include "util.h"
#include <algorithm>
#include "autocomplete.h"

#define API_ADD_FILTER(name, type, location) \
    api.add_api_variable(#name, new APICallbackFloat([]{ return location.get_frequency(); },\
        [](float f){ location.set_frequency(f); }));

#define API_ADD_FILTER_WITH_API(api, name, location) \
    api.add_api_variable(#name, new APICallbackFloat([]{ return location.get_frequency(); },\
        [](float f){ location.set_frequency(f); }));


class APIVariable {
 public:
   virtual std::string get() const = 0;
   virtual void set(std::string) = 0;
};

template<class T>
class APIVariable2 : public APIVariable {
 public:
   APIVariable2(T *value) : value_(value) {};
   APIVariable2(volatile T *value) : value_(value) {};
   APIVariable2(const T* value) : value_(const_cast<T*>(value)) {}
   virtual std::string get() const { return std::to_string(*value_); }
   virtual void set(std::string) = 0;
 protected:
   volatile T *value_;
};

class APIFloat : public APIVariable2<float> {
 public:
   APIFloat(float *f) : APIVariable2(f) {}
   APIFloat(volatile float *f) : APIVariable2(f) {}
   APIFloat(const  float *f) : APIVariable2(f) {}
   void set(std::string);
};

template<class T>
class APIInt : public APIVariable2<T> {
  public:
    APIInt(T *u) : APIVariable2<T>(u) {}
    APIInt(volatile T *u) : APIVariable2<T>(u) {}
    APIInt(const T *u) : APIVariable2<T>(u) {}
    void set(std::string s) {
      *this->value_ = std::stoi(s);
    }
};

typedef APIInt<uint32_t> APIUint32;
typedef APIInt<uint16_t> APIUint16;
typedef APIInt<uint8_t> APIUint8;
typedef APIInt<int32_t> APIInt32;
typedef APIInt<int16_t> APIInt16;
typedef APIInt<int8_t> APIInt8;
typedef APIInt<bool> APIBool;

template<class T>
class APIHex : public APIInt<T> {
 public:
    APIHex(T *u) : APIInt<T>(u) {}
    APIHex(const T *u) : APIInt<T>(u) {}
    void set(std::string s) {
      *this->value_ = std::stoi(s, nullptr, 16);
    }
    virtual std::string get() const { 
      std::vector<char>bytes((char *) this->value_,(char *) this->value_+sizeof(T)); 
      std::reverse(bytes.begin(),bytes.end());
      return bytes_to_hex(bytes); }
};

#include <functional>

class APICallback : public APIVariable {
 public:
  APICallback(std::function<std::string()> getfun, std::function<void(std::string)> setfun) : getfun_(getfun), setfun_(setfun) {}
  APICallback(std::function<std::string()> getfun) : getfun_(getfun) {}
  void set(std::string s) { setfun_(s); }
  std::string get() const {return getfun_(); }
 private:
  std::function<std::string()> getfun_;
  std::function<void(std::string)> setfun_;
};

class APICallbackFloat : public APIVariable {
 public:
   APICallbackFloat(float (*const getfun)(), void (*const setfun)(float)) : getfun_(getfun), setfun_(setfun) {}
   APICallbackFloat(float (*const getfun)()) : getfun_(getfun) {}
   void set(std::string s) { setfun_(stof(s)); }
   std::string get() const { return std::to_string(getfun_()); };
 private:
   float (*const getfun_)();
   void (*const setfun_)(float) = nullptr;
};

template<class T>
class APICallbackUint : public APIVariable {
 public:
   APICallbackUint(T (*const getfun)(), void (*const setfun)(T)) : getfun_(getfun), setfun_(setfun) {}
   APICallbackUint(T (*const getfun)()) : getfun_(getfun) {}
   void set(std::string s) { setfun_(std::stoi(s)); }
   std::string get() const { return std::to_string(getfun_()); }
 private:
   T (*const getfun_)();
   void (*const setfun_)(T) = nullptr;
};

typedef APICallbackUint<uint32_t> APICallbackUint32;
typedef APICallbackUint<uint16_t> APICallbackUint16;
typedef APICallbackUint<uint8_t> APICallbackUint8;
typedef APICallbackUint<int32_t> APICallbackInt32;
typedef APICallbackUint<int16_t> APICallbackInt16;
typedef APICallbackUint<int8_t> APICallbackInt8;

template<class T>
class APICallbackHex : public APIVariable {
 public:
   APICallbackHex(std::function<T()> getfun , std::function<void(T)> setfun) : getfun_(getfun), setfun_(setfun) {}
   APICallbackHex(std::function<T()> getfun) : getfun_(getfun) {}
   void set(std::string s) { setfun_(std::stoul(s, nullptr, 16)); }
   std::string get() const {
      T value = getfun_();
      std::vector<char>bytes((char *) &value,(char *) &value+sizeof(T)); 
      std::reverse(bytes.begin(),bytes.end());
      return bytes_to_hex(bytes);
   }
 private:
   std::function<T()> getfun_;
   std::function<void(T)> setfun_;
};

// allows for setting variables through text commands
class ParameterAPI {
 public:
    // type is used by scanf to parse the string
    void add_api_variable(const std::string_view name, APIVariable *variable);
    void add_api_variable(const std::string_view name, const APIVariable *variable);
    bool set_api_variable(const std::string_view name, std::string value);
    std::string get_api_variable(std::string_view name);
    std::string parse_string(std::string_view);
    std::string get_all_api_variables() const;
    uint16_t get_api_length() const;
    std::string get_api_variable_name(uint16_t index) const;
 private:
    std::map<std::string_view, APIVariable *> variable_map_;
    std::map<std::string_view, const APIVariable *> const_variable_map_;
    AutoComplete auto_complete_;
};

#endif  // UNHUMAN_MOTORLIB_PARAMETER_API_H_
