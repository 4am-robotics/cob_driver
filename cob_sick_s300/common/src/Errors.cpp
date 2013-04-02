
#include "cob_sick_s300/Errors.hpp"
namespace brics_oodl {

Errors::Errors() {
  // Bouml preserved body begin 000211F1
  iter = this->occurredErrors.begin();
  // Bouml preserved body end 000211F1
}

Errors::~Errors() {
  // Bouml preserved body begin 00021271
  this->occurredErrors.clear();
  // Bouml preserved body end 00021271
}

void Errors::getNextError(std::string& name, std::string& description) {
  // Bouml preserved body begin 00022AFC
  if(iter == this->occurredErrors.end()){
    iter = this->occurredErrors.begin();
  }else{
    iter++;
  }
  name = iter->first;
  description = iter->second;
  // Bouml preserved body end 00022AFC
}

void Errors::getAllErrors(map<std::string, std::string>& allErrors) {
  // Bouml preserved body begin 00022B7C
  allErrors = this->occurredErrors;
  // Bouml preserved body end 00022B7C
}

unsigned int Errors::getAmountOfErrors() {
  // Bouml preserved body begin 00021171
  return this->occurredErrors.size();
  // Bouml preserved body end 00021171
}

void Errors::addError(std::string name, std::string description) {
  // Bouml preserved body begin 000212F1
  this->occurredErrors[name] = description;
  //  std::cout << "ERROR: " << name << " " << description << std::endl;


  LOG(error) << name << ": " << description;

  // Bouml preserved body end 000212F1
}

void Errors::deleteAllErrors() {
  // Bouml preserved body begin 00021371
  this->occurredErrors.clear();
  // Bouml preserved body end 00021371
}

void Errors::printErrorsToConsole() {
  // Bouml preserved body begin 0002FA71
  map<std::string,std::string>::iterator iterator;;
  for(iterator = this->occurredErrors.begin();iterator != this->occurredErrors.end(); iterator++){
    std::cout << iterator->first << ": " << iterator->second << std::endl;
  }

  // Bouml preserved body end 0002FA71
}


} // namespace brics_oodl
