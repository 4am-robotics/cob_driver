#include <cob_bms_driver/cob_bms_driver_node.h>

typedef std::vector<BmsParameter> BmsParameters;
typedef std::map<uint8_t, std::vector<BmsParameter> > ConfigMap;

//template to be used to read CAN frames received from the BMS 
template<int N> void big_endian_to_host(const void* in, void* out);
template<> void big_endian_to_host<1>(const void* in, void* out){ *(uint8_t*)out = *(uint8_t*)in;}
template<> void big_endian_to_host<2>(const void* in, void* out){ *(uint16_t*)out = be16toh(*(uint16_t*)in);}
template<> void big_endian_to_host<4>(const void* in, void* out){ *(uint32_t*)out = be32toh(*(uint32_t*)in);}
template<> void big_endian_to_host<8>(const void* in, void* out){ *(uint64_t*)out = be64toh(*(uint64_t*)in);}
template<typename T> T read_value(const can::Frame &f, uint8_t offset){
	T res;
	big_endian_to_host<sizeof(T)>(&f.data[offset], &res);
	return res;
}

CobBmsDriverNode::CobBmsDriverNode()
: nh_priv_("~")
{}

CobBmsDriverNode::~CobBmsDriverNode() 
{
	socketcan_interface_.shutdown();
}

//initlializes SocketCAN interface, saves data from ROS parameter server, loads polling lists and sets up diagnostic updater
bool CobBmsDriverNode::prepare() 
{
	//reads parameters from ROS parameter server and saves them in respective member variable: config_map_, poll_period_for_two_ids_in_ms_, can_device_, bms_id_to_poll_
	if (!getParams())
	{
		ROS_ERROR("Could not prepare driver for start up");
		return false;
	}

	//goes through config_map_ and loads the pollings lists 
	loadPollingLists();

	//initalize polling lists iterators
	polling_list1_it_ = polling_list1_.begin();
	polling_list2_it_ = polling_list2_.begin();

	updater_.setHardwareID("bms"); 
	updater_.add("cob_bms_dagnostics_updater", this, &CobBmsDriverNode::produceDiagnostics);

	updater_timer_ = nh_.createTimer(ros::Duration(updater_.getPeriod()), &CobBmsDriverNode::diagnosticsTimerCallback, this);

	//initialize the socketcan interface
	if(!socketcan_interface_.init(can_device_, false)) {
		ROS_ERROR("cob_bms_driver initialization failed");
		return false;	
	}

	//create listener for CAN frames
	frame_listener_  = socketcan_interface_.createMsgListener(can::CommInterface::FrameDelegate(this, &CobBmsDriverNode::handleFrames));

	return true;
}

//function to get ROS parameters from parameter server
bool CobBmsDriverNode::getParams()
{
	//local declarations
	XmlRpc::XmlRpcValue diagnostics;
	std::vector <std::string> topics;
	int poll_frequency_hz;

	if (!nh_priv_.getParam("topics", topics)) 
	{
		ROS_INFO_STREAM("Did not find \"topics\" on parameter server");
	}    

	if (!nh_priv_.getParam("diagnostics", diagnostics)) 
	{
		ROS_INFO_STREAM("Did not find \"diagnostics\" on parameter server");
	}
	loadConfigMap(diagnostics, topics);	
	if (createPublishersFor(topics) == false)
	{
		return false;
	}

	if (!nh_priv_.getParam("can_device", can_device_)) 
	{
		ROS_INFO_STREAM("Did not find \"can_device\" on parameter server. Using default value: can0");
		can_device_ = "can0";
	}

	if (!nh_priv_.getParam("bms_id_to_poll", bms_id_to_poll_)) 
	{
		ROS_INFO_STREAM("Did not find \"bms_id_to_poll\" on parameter server. Using default value: 0x200");
		bms_id_to_poll_ = 0x200;
	}

	if (!nh_priv_.getParam("poll_frequency_hz", poll_frequency_hz)) 
	{
		ROS_INFO_STREAM("Did not find \"poll_frequency\" on parameter server. Using default value: 20 Hz");
		poll_frequency_hz = 20;
	} 
	evaluatePollPeriodFrom(poll_frequency_hz);
	return true;
}

//function to create a publisher for each Topic that is listed in the configuration file
bool CobBmsDriverNode::createPublishersFor(std::vector<std::string> topics) 
{
	if (topics.empty())
	{
		ROS_INFO("Topic list is empty. No publisher created");
		return true;
	}

	//for all topics 
	for (std::vector<std::string>::iterator it_topic = topics.begin(); it_topic!=topics.end(); ++it_topic)
	{
		bool match_already_found = false;
		//verify that the topic name matches with a BmsParameter name from config file
		ConfigMap::iterator it_map = config_map_.begin();
		for (it_map; it_map!=config_map_.end(); ++it_map) 
		{
			//find a matching name in std::vector<BmsParameter>
			for (std::vector<BmsParameter>::iterator it_bms_vec = it_map->second.begin(); it_bms_vec != it_map->second.end(); ++it_bms_vec)
			{
				if (!it_topic->empty() )
				{
					if (it_bms_vec->name == *it_topic)
					{
						//found a match, so create publisher
						bms_diagnostics_publishers_[*it_topic] = nh_priv_.advertise<std_msgs::Float64> (*it_topic, 100, true);
						ROS_INFO_STREAM("Created publisher for: " << *it_topic);
						match_already_found = true;
						break;	//dont need to look anymore in the config_map_
					}
				}
				else 
				{
					ROS_ERROR("Topic name can not be empty");
					return false;
				}
			}
			if (match_already_found) break;
		}
		//no match found for current topic
		if (it_map == config_map_.end())
		{
			ROS_WARN_STREAM("Didn't create publisher for: " << *it_topic << ". Make sure \"" << *it_topic << "\" matches a BmsParameter name in the Configuration file.");
		}
	}
	return true;
}

//function to interpret the diagnostics XmlRpcValue and save data in config_map_
void CobBmsDriverNode::loadConfigMap(XmlRpc::XmlRpcValue diagnostics, std::vector<std::string> topics) 
{
	XmlRpc::XmlRpcValue config_l0_array, config_l1_struct, config_l2, config_l3_struct, xdiagnostics, xdiagnostic_elements, xfields, xpair,temp;
	BmsParameters bms_parameters;
	BmsParameter bms_parameter_temp;
	uint8_t id;

	config_l0_array = diagnostics;	//just for better readability

	ROS_ASSERT(config_l0_array.getType() == XmlRpc::XmlRpcValue::TypeArray);  
	//for each id in list of ids
	for (size_t i = 0; i < config_l0_array.size(); ++i) 
	{
		bool has_id = false, has_bms_parameters = false; 
		
		ROS_ASSERT(config_l0_array[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
		config_l1_struct = config_l0_array[i];
		for (XmlRpc::XmlRpcValue::iterator it1=config_l1_struct.begin(); it1!=config_l1_struct.end(); ++it1) 
		{
			config_l2 = it1->second;
			//XmlRpcValue at config_l2 might be an id (TypeInt) or a list of fields (TypeArray)
			if (config_l2.getType()==XmlRpc::XmlRpcValue::TypeInt) 
			{
				//id = static_cast<char>(static_cast<int>(config_l2));
				id = static_cast<uint8_t>(static_cast<int>(config_l2));
				has_id = true;
			} 
			else if (config_l2.getType()==XmlRpc::XmlRpcValue::TypeArray)
			{
				//for each field in field list. (each field is a BMSParameter)
				for(int32_t j=0; j<config_l2.size(); ++j) 
				{
					//booleans to help ensure that all required information is provided for a BMSParameter
					bool has_name = false, has_offset = false, has_len = false, has_is_signed = false;

					ROS_ASSERT(config_l2[j].getType()==XmlRpc::XmlRpcValue::TypeStruct);
					config_l3_struct = config_l2[j];
					//for each element in a field
					for (XmlRpc::XmlRpcValue::iterator it3=config_l3_struct.begin(); it3!=config_l3_struct.end(); ++it3) 
					{
						if (it3->first == "name" && !has_name) 
						{
							ROS_ASSERT(it3->second.getType()==XmlRpc::XmlRpcValue::TypeString);
							bms_parameter_temp.name = static_cast<std::string>(it3->second);
							has_name = true;
						}
						else if (it3->first == "offset" && !has_offset) 
						{
							ROS_ASSERT(it3->second.getType()==XmlRpc::XmlRpcValue::TypeInt);
							bms_parameter_temp.offset = static_cast<int>(it3->second);
							has_offset = true;
						}
						else if (it3->first == "len" && !has_len) 
						{
							ROS_ASSERT(it3->second.getType()==XmlRpc::XmlRpcValue::TypeInt);
							bms_parameter_temp.length = static_cast<int>(it3->second);
							has_len = true;
						}
						else if (it3->first == "is_signed" && !has_is_signed) 
						{
							ROS_ASSERT(it3->second.getType()==XmlRpc::XmlRpcValue::TypeBoolean);
							bms_parameter_temp.is_signed = static_cast<bool>(it3->second);
							has_is_signed = true;
						}
						else if (it3->first == "factor") 
						{
							ROS_ASSERT(it3->second.getType()==XmlRpc::XmlRpcValue::TypeDouble);
							bms_parameter_temp.factor = static_cast<double>(it3->second);
						}
						else if (it3->first == "unit")
						{
							ROS_ASSERT(it3->second.getType()==XmlRpc::XmlRpcValue::TypeString);
							bms_parameter_temp.unit = "[" + static_cast<std::string>(it3->second) + "]";
						} 
						else ROS_ERROR_STREAM("Config: Unexpected Key: " << it3->first);
					}

					//iff name of the BmsParameter matches with a name of a topic in the Configuration file, then mark this parameter as a topic
					bms_parameter_temp.is_topic = (find(topics.begin(), topics.end(), bms_parameter_temp.name) != topics.end());

					//set diagnostic_msgs::KeyValue member of the BmsParameter
					bms_parameter_temp.kv.key = bms_parameter_temp.name+bms_parameter_temp.unit;

					if (has_name && has_offset && has_len && has_is_signed)
					{
						//bms_parameter_temp is properly filled at this point, so save it in bms_parameters
						bms_parameters.push_back(bms_parameter_temp);
						has_bms_parameters = true;
					}
					else 
					{
						ROS_WARN_STREAM("Config: id # " << i+1 << ", field # " << j+1 << " does not have all the required fields set. This field will not be used.");
					}
				}
			}
			else ROS_ERROR_STREAM("Config: Expected either XmlRpc Type: " << XmlRpc::XmlRpcValue::TypeArray << " or " << XmlRpc::XmlRpcValue::TypeInt << ". But found: " << config_l2.getType());
		}
		
		if (has_id && has_bms_parameters)
		{
			//save bms_parameters in config map for interpreting them later and producing diagnostics as well as publishing data on topics
			config_map_[id] = bms_parameters;
			ROS_INFO_STREAM("Got "<< bms_parameters.size() << " BmsParameter(s) with CAN-ID: 0x" << std::hex << (unsigned int) id << std::dec);
			//clear bms_parameters for processing next id
			bms_parameters.clear();
		}
		else 
		{
			if (!has_id)
				ROS_WARN_STREAM("Config: id # " << i+1 << ", \'id\' is missing.");
			else if (!has_bms_parameters)
				ROS_WARN_STREAM("Config: id # " << i+1 << ", valid \'field(s)\' is missing.");
				
			ROS_WARN_STREAM("Config: id # " << i+1 << " will not be used");
		}
	}
}

//helper function to evaluate poll period from given poll frequency
void CobBmsDriverNode::evaluatePollPeriodFrom(int poll_frequency_hz)
{
	//check the validity of given poll_frequency_hz
	if ((poll_frequency_hz < 0) && (poll_frequency_hz > 20)) 
	{
		ROS_WARN_STREAM("Invalid ROS parameter value: poll_frequency_hz = "<< poll_frequency_hz << ". Setting poll_frequency_hz to 20 Hz");
		poll_frequency_hz = 20;
	}
	//now evaluate and save poll period
	poll_period_for_two_ids_in_ms_ = (1/double(poll_frequency_hz))*1000;
	ROS_INFO_STREAM("Evaluated polling period: "<< poll_period_for_two_ids_in_ms_ << " ms");
}

//function that goes through config_map_ and fills polling_list1_ and polling_list2_. If topics are found on ROS Parameter Server, they are kept in list1 otherwise, all parameter id are divided between both lists.
void CobBmsDriverNode::loadPollingLists() 
{
	if (config_map_.empty()) 
	{
		ROS_ERROR("config_map_ is empty! Can not load polling lists!");
		return;
	}

	if(!bms_diagnostics_publishers_.empty())  
	{
		for (ConfigMap::iterator it = config_map_.begin(); it != config_map_.end(); ++it) 
		{
			BmsParameters current_parameter_list = it->second;
			uint8_t parameter_can_id = it->first;

			for (size_t j=0; j<current_parameter_list.size(); ++j) 
			{
				//second condition here is to ensure that the list1 is always smaller or equal to list2. This is important because otherwise BmsParameters which are topics would get slower updates (possible when topics list is large!).
				if ((current_parameter_list.at(j).is_topic) && (polling_list1_.size() <= polling_list2_.size())) 
				{
					polling_list1_.push_back(parameter_can_id);
					break; //parameter_can_id needs to be saved only once
				}
				else
				{
					polling_list2_.push_back(parameter_can_id);
					break; //parameter_can_id needs to be saved only once
				}
			}
		}
	}
	else 
	{
		//No BmsParameter is a topic, so load lists such that all ids are polled at equal intervals
		ROS_INFO("No publishers found. Distributing the polling CAN-ID(s) equally.");
		bool toggle = true;
		for (ConfigMap::iterator it = config_map_.begin(); it != config_map_.end(); ++it) 
		{
			toggle? polling_list1_.push_back(it->first) : polling_list2_.push_back(it->first);
			toggle = !toggle;
		}
	}
	ROS_INFO_STREAM("Loaded \'"<< polling_list1_.size() << "\' CAN-ID(s) in polling_list1_ and \'"<< polling_list2_.size() <<"\' CAN-ID(s) in polling_list2_");
}

//function that polls BMS for given ids
void CobBmsDriverNode::pollBmsForIds(const uint16_t first_id, const uint16_t second_id)
{
	can::Frame f(can::Header(bms_id_to_poll_,false,false,false),4);
	f.data[0] = first_id >> 8;	//high_byte
	f.data[1] = first_id & 0xff;	//low_byte
	f.data[2] = second_id >> 8;	
	f.data[3] = second_id & 0xff;

	socketcan_interface_.send(f);

	boost::this_thread::sleep_for(boost::chrono::milliseconds(poll_period_for_two_ids_in_ms_));
}

//cycles through polling lists and sends 2 ids at a time (one from each list) to the BMS
void CobBmsDriverNode::pollNextInLists()
{
	//restart if reached the end of polling lists
	if (polling_list1_it_ == polling_list1_.end()) polling_list1_it_ = polling_list1_.begin();
	if (polling_list2_it_ == polling_list2_.end()) polling_list2_it_ = polling_list2_.begin();
	//clear stat_, so that it can be refilled with new data
	stat_.clear();

	uint16_t first_id = (polling_list1_it_ == polling_list1_.end()) ? 0 : (*polling_list1_it_ | 0x0100);
	uint16_t second_id = (polling_list2_it_ == polling_list2_.end()) ? 0 : (*polling_list2_it_ | 0x0100);

	ROS_DEBUG_STREAM("polling BMS for CAN-IDs: 0x" << std::hex << (int)first_id << " and 0x" << (int) second_id << std::dec);

	pollBmsForIds(first_id,second_id); 

	//increment iterators for next poll 
	if (!polling_list1_.empty()) ++polling_list1_it_;
	if (!polling_list2_.empty()) ++polling_list2_it_;
}

//callback function to handle all types of frames received from BMS
void CobBmsDriverNode::handleFrames(const can::Frame &f)
{
	boost::mutex::scoped_lock lock(data_mutex_);

	//id to find in config map
	uint32_t frame_id = f.id;

	//find frame_id in config_map_
	ConfigMap::iterator config_map_it = config_map_.find(frame_id);
	if (config_map_it==config_map_.end()) return;

	for (BmsParameters::iterator param = config_map_it->second.begin(); param!=config_map_it->second.end(); ++param) 
	{
		double data = 0;
		switch (param->length)
		{
			case 1:
				data = param->is_signed ? read_value<int8_t> (f, param->offset) * param->factor : read_value<uint8_t> (f, param->offset) * param->factor;
				break;

			case 2:
				data = param->is_signed ? read_value<int16_t> (f, param->offset) * param->factor : read_value<uint16_t> (f, param->offset) * param->factor;
				break;

			case 4:
				data = param->is_signed ? read_value<int32_t> (f, param->offset) * param->factor : read_value<uint32_t> (f, param->offset) * param->factor;
				break;

			default: 
				ROS_WARN_STREAM("Unknown length of BmsParameter: " << param->name << ". Cannot read data!");
				return;	//only go on with next step if data was read successfully
		}

		//save data for diagnostics updater (and round to two digits for readability)
		param->kv.value = boost::lexical_cast<std::string>(boost::format("%.2f") % data);

		//if the BmsParameter is a topic, publish data to the topic
		if (param->is_topic)
		{
			//find publisher for this topic in bms_diagnostics_publishers_ 
			std::map<std::string, ros::Publisher>::const_iterator it_pub = bms_diagnostics_publishers_.find(param->name);
			if (it_pub != bms_diagnostics_publishers_.end())
			{
				std_msgs::Float64 float_msg;
				float_msg.data = data;
				(it_pub->second).publish(float_msg);
			}
			else 
			{
				ROS_ERROR_STREAM("Could not find a publisher for: " << param->name);
			}
		}
	}	
}

//updates the diagnostics data with the new data received from BMS
void CobBmsDriverNode::produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
	boost::mutex::scoped_lock lock(data_mutex_);

	can::State state = socketcan_interface_.getState();
	stat.add("error_code", state.error_code);
	stat.add("can_error_code", state.internal_error);
	switch (state.driver_state)
	{
		case can::State::closed: 
			stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Driver State: Closed");
			break;

		case can::State::open: 
			stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Driver State: Opened");
			break;

		case can::State::ready: 
			stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Driver State: Ready");
			break;

		default: 
			stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Driver State: Unknown");
			break;
	}

	for (ConfigMap::iterator cm_it = config_map_.begin(); cm_it != config_map_.end(); ++cm_it)
	{
		for (BmsParameters::iterator bp_it = cm_it->second.begin(); bp_it != cm_it->second.end(); ++bp_it)
		{
			stat.values.push_back(bp_it->kv);
		}
	}
}

void CobBmsDriverNode::diagnosticsTimerCallback(const ros::TimerEvent& event)
{
	//update diagnostics
	updater_.update();
}

int main(int argc, char **argv) 
{
	ros::init(argc, argv, "bms_driver_node");

	CobBmsDriverNode cob_bms_driver_node;

	if (!cob_bms_driver_node.prepare()) return 1;

	ROS_INFO("Started polling BMS...");
	while (cob_bms_driver_node.nh_.ok())
	{
		cob_bms_driver_node.pollNextInLists();
		ros::spinOnce();
	}
	return 0;
}
