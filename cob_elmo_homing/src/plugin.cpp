/*
 * Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include <canopen_402/motor.h>
#include <socketcan_interface/make_shared.h>
#include <pluginlib/class_list_macros.h>


namespace cob_elmo_homing {

class ElmoMotor402 : public canopen::Motor402 {

    canopen::ObjectStorage::Entry<uint64_t> command_entry_;
    canopen::ObjectStorage::Entry<uint64_t> response_entry_;

    static const uint64_t byte_3_bit_6 = (1  << (3*8+6));;
    static const uint64_t compare_mask = 0xFFFFFFFF ^ byte_3_bit_6;

    int32_t offset_;
    int32_t event_;
    int32_t speed_;
    uint32_t timeout_;

    uint64_t readResponse(uint64_t command)  {
        boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
        uint64_t response = response_entry_.get();
        return (response & compare_mask) == (command & compare_mask) ? response : 0;
    }

    bool set(char c1, char c2, uint16_t index, uint32_t val){
        uint64_t response = 0;
        uint64_t command = c1 | (c2 << (1*8)) | ((index & 0x3FFF) << (2*8)) |  static_cast<uint64_t>(val) << (4*8);
        command_entry_.set(command);

        canopen::time_point timeout = canopen::get_abs_time(boost::chrono::seconds(1));
        do{
            response = readResponse(command);
        }while(response != command && (response & byte_3_bit_6) == 0 && canopen::get_abs_time() < timeout);

        bool ok = response == command && (response & byte_3_bit_6) == 0;
        return ok;
    }
    template<typename T> bool test(char c1, char c2, uint16_t index, uint32_t val, T dur){
        uint64_t response = 0;
        uint64_t command =  c1 | (c2 << (1*8)) | ((index & 0x3FFF) << (2*8)) | byte_3_bit_6;

        canopen::time_point timeout = canopen::get_abs_time(dur);
        do{
            command_entry_.set(command);
            response = readResponse(command);
        }while( (response >> (4*8)) != val  && canopen::get_abs_time() < timeout);

        return (response & byte_3_bit_6) == 0  && (response >> (4*8)) == val;
    }

public:
    ElmoMotor402(const std::string &name, canopen::ObjectStorageSharedPtr storage, const canopen::Settings &settings)
    : Motor402(name, storage, settings)
    {
        storage->entry(command_entry_, 0x2012);
        storage->entry(response_entry_, 0x2013);
        event_ = settings.get_optional<int32_t>("homing_event", -1);
        speed_ = settings.get_optional<int32_t>("homing_speed", 50000);
        offset_ = settings.get_optional<int32_t>("homing_offset", 0);
        timeout_= settings.get_optional<uint32_t>("homing_timeout", 60);
    }

    void handleInit(canopen::LayerStatus &status){
        Motor402::handleInit(status);
        if(status.bounded<canopen::LayerStatus::Ok>() && event_ >= 0){
            if(!command_entry_.valid()){
                status.error("Command entry is not valid");
            }else if(!response_entry_.valid()){
                status.error("Response entry is not valid");
            }else if(!enterModeAndWait(Profiled_Velocity)){
                status.error("Could not switch mode");
            }else{
                if(!set('H','M', 1, 0) || // reset homing
                !set('H','M', 2, offset_) ||
                !set('H','M', 3, event_) ||
                !set('H','M', 4, 2) || // do not stop after homing
                !set('H','M', 5, 0) || // set PX to offset
                !set('H','M', 1, 1) || // start homing
                !setTarget(speed_)){
                    status.error("could not initialize homing");
                    return;
                }

                if(!test('H','M', 1, 0, boost::chrono::seconds(timeout_))) status.error("homing timeout");

                if(!setTarget(0)) status.error("could not stop motor");
                if(!set('H','M', 1, 0)) status.error("could not stop homing");
            }
        }
    }

    class Allocator : public canopen::MotorBase::Allocator{
    public:
      virtual canopen::MotorBaseSharedPtr allocate(const std::string& name,
                                                   canopen::ObjectStorageSharedPtr storage,
                                                   const canopen::Settings& settings)
      {
        return ROSCANOPEN_MAKE_SHARED<ElmoMotor402>(name, storage, settings);
      }
    };
};

}

PLUGINLIB_EXPORT_CLASS(cob_elmo_homing::ElmoMotor402::Allocator, canopen::MotorBase::Allocator)
