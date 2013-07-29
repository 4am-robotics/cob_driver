//#include <cob_phidgets/AnalogSensor.h>
//#include <cob_phidgets/DigitalSensor.h>

#include <libphidgets/phidget21.h>
#include <string>

enum class SensorType {ANALOG = 0, DIGITAL_IN = 1, DIGITAL_OUT = 2};

class Sensor
{
public:
	virtual ~Sensor(){;}

	auto getHwIndex() const -> int {return _hwIndex;}

	virtual auto update(int value) -> int = 0;
	auto getValue() -> int {return _value;};

protected:
	int _hwIndex;
	int _value;

	CPhidgetInterfaceKitHandle* _iKitHandle;

protected:
	Sensor(const int hw_index, const std::string &sensor_name, CPhidgetInterfaceKitHandle* iKitHandle)
		:_hwIndex(hw_index), _iKitHandle(iKitHandle) {;}

};

class AnalogInSensor : public Sensor
{
	public:
		AnalogInSensor(const int hw_index, const std::string &sensor_name, CPhidgetInterfaceKitHandle* iKitHandle);

		auto setDataRate(int ms) -> int;
		auto getDataRate() -> int;

		auto getDataRateMax() -> int;
		auto getDataRateMin() -> int;

		auto setSensorChangeTrigger(int trigger) -> int;
		auto getSensorChangeTrigger() -> int;

		auto getSensorRawValue() -> int;
		auto getSensorValue() -> int;

		auto update(int value) -> int {return 1;}

	private:
		int _dataRateMs;
		int _dataRateMsMax;
		int _dataRateMsMin;
		int _sensorChangeTrigger;
		int _sensorRawValue;

};

class DigitalInSensor : public Sensor
{
	public:
		DigitalInSensor(const int hw_index, const std::string &sensor_name, CPhidgetInterfaceKitHandle* iKitHandle);

		auto getInputState() -> int;

		auto update(int value) -> int {return 1;}

	private:
};

class DigitalOutSensor : public Sensor
{
	public:
		DigitalOutSensor(const int hw_index, const std::string &sensor_name, CPhidgetInterfaceKitHandle* iKitHandle);

		auto setOutputState(int state) -> int;
		auto getOutputState() -> int;

		auto update(int value) -> int {return 1;}

	private:
		
};