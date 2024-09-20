#include "arbitrary_parameter.hpp"
#include "demangle.hpp"

void ArbitraryParameter::init(const std::string& json_file_path) {
  std::ifstream fin(json_file_path);
  if(!fin) {
    std::cerr << "json file " << json_file_path << " does not exist" << std::endl;
  } else {
    nlohmann::json flat_endpoints_json = nlohmann::json::parse(fin);
    for(nlohmann::json::iterator it = flat_endpoints_json["endpoints"].begin(); it != flat_endpoints_json["endpoints"].end(); ++it){
      uint16_t id = it.value()["id"];
      std::string name = it.key();
      std::string type_raw = it.value()["type"];

      names_.insert(boost::bimaps::bimap<uint16_t, std::string>::value_type(id, name));

      const std::map<std::string, std::string> type_map {
        {"bool",   typeid(bool).name()},
          {"uint8",  typeid(uint8_t).name()},
          {"uint16", typeid(uint16_t).name()},
          {"uint32", typeid(uint32_t).name()},
          {"uint64", typeid(uint64_t).name()},
          {"int32",  typeid(int32_t).name()},
          {"int64",  typeid(int64_t).name()},
          {"float",  typeid(float).name()},
          {"endpoint_ref", "ENDPOINT_REF IS NOT DEFINED"},
          {"function", "FUNCTION IS NOT DEFINED"},
      };

      std::string type;
      try {
        type = type_map.at(type_raw);
      } catch(const std::out_of_range& e) {
        type = "TYPE NOT FOUND";
      }
      types_.insert(std::make_pair(id, type));

      mutexes_.emplace(std::piecewise_construct, std::forward_as_tuple(id), std::forward_as_tuple());
      cvs_.emplace(std::piecewise_construct, std::forward_as_tuple(id), std::forward_as_tuple());
      flags_.insert(std::make_pair(id, bool()));

      if(type == typeid(bool).name()) {
        bool_parameters.insert(std::make_pair(id, bool()));
      } else if(type == typeid(uint8_t).name()) {
        uint8_parameters.insert(std::make_pair(id, uint8_t()));
      } else if(type == typeid(uint16_t).name()) {
        uint16_parameters.insert(std::make_pair(id, uint16_t()));
      } else if(type == typeid(uint32_t).name()) {
        uint32_parameters.insert(std::make_pair(id, uint32_t()));
      } else if(type == typeid(uint64_t).name()) {
        uint64_parameters.insert(std::make_pair(id, uint64_t()));
      } else if(type == typeid(int32_t).name()) {
        int32_parameters.insert(std::make_pair(id, int32_t()));
      } else if(type == typeid(int64_t).name()) {
        int64_parameters.insert(std::make_pair(id, int64_t()));
      } else if(type == typeid(float).name()) {
        float_parameters.insert(std::make_pair(id, float()));
      } else {
        std::cerr << "type " << type << " is not implemented yet" << std::endl;
      }
    }
  }
}

std::string ArbitraryParameter::get_name(uint16_t id) {
  if(!contains(id)) {
    std::cerr << "id " << id << " does not exist in names_" << std::endl;
    return "ID NOT FOUND";
  }
  return names_.left.at(id);
}

uint16_t ArbitraryParameter::get_id(const std::string& name) {
  if(!contains(name)) {
    std::cerr << "id " << name << " does not exist in names_" << std::endl;
    return 0;
  }
  return names_.right.at(name);
}

bool ArbitraryParameter::contains_float(uint16_t id) {
  return float_parameters.find(id) != float_parameters.end();
}

bool ArbitraryParameter::contains(const std::string& name) {
  return names_.right.find(name) != names_.right.end();
}

bool ArbitraryParameter::contains(uint16_t id) {
  return names_.left.find(id) != names_.left.end();
}

bool& ArbitraryParameter::get_flag(uint16_t id) {
  return flags_.at(id);
}

std::string ArbitraryParameter::get_type(uint16_t id) {
  return types_.at(id);
}

std::string ArbitraryParameter::get_type_demangled(uint16_t id) {
  return demangle(get_type(id).c_str());
}

std::mutex& ArbitraryParameter::get_mutex(uint16_t id) {
  return mutexes_.at(id);
}

std::condition_variable& ArbitraryParameter::get_cv(uint16_t id) {
  return cvs_.at(id);
}

void ArbitraryParameter::set_fresh(uint16_t id, float input_val) {
  std::string name = get_name(id);
  if(!contains(name)) {
    std::cerr << "endpoint name " << name << " does not exist" << std::endl;
    return;
  }
  if(!contains_float(id)) {
    std::cerr << "endpoint name " << name << " is not included in float type parameters" << std::endl;
    return;
  }
  {
    std::mutex& input_val_mutex = get_mutex(id);
    std::unique_lock<std::mutex> guard(input_val_mutex);
    bool& resource_ready = get_flag(id);
    float& target_parameter = float_parameters.at(id);
    target_parameter = input_val;
    resource_ready = true;
  }
  get_cv(id).notify_all();
  return;
}

void ArbitraryParameter::get_fresh(uint16_t id, float& output_val) {
  std::string name = get_name(id);
  if(!contains(name)) {
    std::cerr << "endpoint name " << name << " does not exist" << std::endl;
    return;
  }
  if(!contains_float(id)) {
    std::cerr << "endpoint name " << name << " is not included in float type parameters" << std::endl;
    return;
  }
  std::mutex& output_val_mutex = get_mutex(id);
  std::unique_lock<std::mutex> guard(output_val_mutex);
  bool& resource_ready = get_flag(id);
  if(get_cv(id).wait_for(guard, std::chrono::milliseconds(100), [&resource_ready]{return resource_ready;})) {
    // parameter is updated in time
    output_val = float_parameters.find(id)->second;
    resource_ready = false;
  } else {
    // parameter is not updated
    std::cerr << "sync thread timeout: cannot receive " << name << " in time" << std::endl;
  }
  return;
}
