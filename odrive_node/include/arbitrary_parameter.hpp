#ifndef ARBITRARY_PARAMETER_HPP
#define ARBITRARY_PARAMETER_HPP

#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>
#include <boost/bimap.hpp>
#include <mutex>
#include <condition_variable>
#include "demangle.hpp"

class ArbitraryParameter {
public:
    ArbitraryParameter(){}
    ~ArbitraryParameter(){}
    void init(const std::string& json_file_path);
    bool contains(const std::string& name);
    bool contains(uint16_t id);
    void set_fresh(uint16_t id, float input_val);
    void get_fresh(uint16_t id, float& output_val);
    std::string get_type(uint16_t id); // obtain mangled type name
    std::string get_type_demangled(uint16_t id); // obtain demangled type name
    uint16_t get_id(const std::string& name);
private:
    bool contains_float(uint16_t id);
    std::string get_name(uint16_t id);
    std::mutex& get_mutex(uint16_t id);
    std::condition_variable& get_cv(uint16_t id);
    bool& get_flag(uint16_t id);

    std::map<uint16_t, bool>     bool_parameters;
    std::map<uint16_t, uint16_t> uint16_parameters;
    std::map<uint16_t, uint32_t> uint32_parameters;
    std::map<uint16_t, uint64_t> uint64_parameters;
    std::map<uint16_t, int8_t>   uint8_parameters;
    std::map<uint16_t, int32_t>  int32_parameters;
    std::map<uint16_t, int64_t>  int64_parameters;
    std::map<uint16_t, float>    float_parameters;
    //std::map<uint16_t, uint16_t> endpoint_ref_parameters;
    //std::map<uint16_t, uint16_t> function_parameters;
    //std::vector<uint16_t> ids_;
    boost::bimaps::bimap<uint16_t, std::string> names_;
    //std::map<uint16_t, std::string> names_;
    std::map<uint16_t, std::string> types_;

    std::map<uint16_t, std::mutex> mutexes_;
    std::map<uint16_t, std::condition_variable> cvs_;
    std::map<uint16_t, bool> flags_;
};

#endif // ARBITRARY_PARAMETER_HPP
