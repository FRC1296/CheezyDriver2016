#include "external_limit_switch.h"


ExternalLimitSwitch::ExternalLimitSwitch(sdf::ElementPtr sdf) {
  sensor = boost::dynamic_pointer_cast<sensors::ContactSensor>(
               sensors::get_sensor(sdf->Get<std::string>("sensor")));

  gzmsg << "\texternal limit switch: " << " sensor=" << sensor->GetName() << std::endl;
}

ExternalLimitSwitch::~ExternalLimitSwitch() {}

bool ExternalLimitSwitch::Get() {
  return sensor->GetContacts().contact().size() > 0;
}
