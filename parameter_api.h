#ifndef UNHUMAN_MOTORLIB_PARAMETER_API_H_
#define UNHUMAN_MOTORLIB_PARAMETER_API_H_

#include <string_view>
#include <map>
#include <vector>
#include "util.h"
#include <algorithm>
#include "autocomplete.h"
#include "logger.h"

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

class APIStringView : public APIVariable {
 public:
   APIStringView(const std::string_view s) : value_(s) {}
   void set(std::string s) {}
   std::string get() const { return std::string(value_); }
 private:
   std::string_view value_;
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
  APICallback(std::string (*const getfun)(), void (*const setfun)(std::string)) : getfun_(getfun), setfun_(setfun) {}
  APICallback(std::string (*const getfun)()) : getfun_(getfun) {}
  void set(std::string s) { setfun_(s); }
  std::string get() const {return getfun_(); }
 private:
  std::string (*const getfun_)();
  void (*const setfun_)(std::string) = nullptr;
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
   APICallbackHex(T (*const getfun)(), void (*const setfun)(T)) : getfun_(getfun), setfun_(setfun) {}
   APICallbackHex(T (*const getfun)()) : getfun_(getfun) {}
   void set(std::string s) { setfun_(std::stoul(s, nullptr, 16)); }
   std::string get() const {
      T value = getfun_();
      std::vector<char>bytes((char *) &value,(char *) &value+sizeof(T)); 
      std::reverse(bytes.begin(),bytes.end());
      return bytes_to_hex(bytes);
   }
 private:
   T (*const getfun_)();
   void (*const setfun_)(T) = nullptr;
};


// allows for setting variables through text commands
class ParameterAPI {
 public:
    void add_api_variable(const std::string_view name, APIVariable *variable);
    void add_api_variable(const std::string_view name, const APIVariable *variable);
    bool set_api_variable(const std::string_view name, std::string value);
    std::string get_api_variable(std::string_view name);
    std::string parse_string(std::string_view);
    std::string get_all_api_variables() const;
    uint16_t get_api_length() const;
    std::string get_api_variable_name(uint16_t index) const;
    uint32_t memory_used() const {
      return AllocatorBase::index_ * sizeof(uint32_t);
    }

#define API_SIZE 5000
    class AllocatorBase {
      public:
        static uint32_t index_;
        static uint32_t mem_[API_SIZE];
    };
    template<typename T>
    class Allocator : public AllocatorBase {
      public:
        using value_type = T;
    
        T* allocate(std::size_t n) {
            uint32_t new_index_ = index_ + (n * sizeof(T) + sizeof(uint32_t) - 1) / sizeof(uint32_t);
            if (new_index_ > API_SIZE) {
                throw std::bad_alloc();
            }
            T* ptr = reinterpret_cast<T*>(&mem_[index_]);
            index_ = new_index_;
            return ptr;
        }
        void deallocate(T* p, std::size_t) {
            // never deallocates
        }
    };
 private:
    std::map<std::string_view, APIVariable *, std::less<std::string_view>, Allocator<std::pair<const std::string_view, APIVariable *>>>
      variable_map_;
    std::map<std::string_view, const APIVariable *, std::less<std::string_view>, Allocator<std::pair<const std::string_view, const APIVariable *>>>
      const_variable_map_;
    AutoComplete auto_complete_;
};

#endif  // UNHUMAN_MOTORLIB_PARAMETER_API_H_



#include <concepts>
struct Item {
    virtual constexpr const char *get() const = 0;
    virtual constexpr void set (char *cin) const = 0;
    virtual constexpr char * set_with_response(char *cin) const = 0;
};

struct ROItem : public Item {
    virtual constexpr const char * get() const { return "item"; }
    virtual constexpr void set(char *cin) const {}
    virtual constexpr char * set_with_response(char *cin) const { return cin; }
};

struct RWItem : public ROItem {
    int a = 2;
    char *c = "abc";
    constexpr const char * get() { return c; }
    constexpr void set(char *cin) const override { c[1] = cin[0];  }
};

struct ResponseItem : public RWItem {
    constexpr char* set_with_response(char *cin) const { c[2] = cin[0]; return c; }
};

struct SpecialItem : public RWItem {
    SpecialItem() {
        c = "hij";
    }
};

template<typename T, typename base>
struct IntItem : public base {
    IntItem(T& t) : t(t) {}
    virtual constexpr const char * get() const { return "iteem"; }
    virtual constexpr void set(char *cin) const {
        if constexpr (std::derived_from<base, RWItem>) {
            t = cin[0];
        }
    }
    virtual constexpr char * set_with_response(char *cin) const { 
        if constexpr (std::derived_from<base, ResponseItem>) {
            t = cin[0];
            return &t;
        }
        return "no way";
    }
    T& t;
};


struct LessSpecialItem : public ROItem {
    constexpr const char *get() const override { return "less special item";}
};

const ROItem i;
const RWItem b, b2;
const SpecialItem s;
const LessSpecialItem l;
int a;
const IntItem<int, ROItem> ro_int(a);
constinit const Item* const items[] {&i, &b, &b2, &s, &l, &ro_int};

//constinit const Item* const items2[] {new const ROItem};

const char* fun(int i) {
    items[i]->set("def");
    return items[i]->get();
}