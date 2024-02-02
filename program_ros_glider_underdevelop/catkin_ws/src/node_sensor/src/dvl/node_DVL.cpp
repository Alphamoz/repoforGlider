/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, LABUST, UNIZG-FER
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the LABUST nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#include <labust/navigation/NavQuestNode.hpp>
#include <labust/navigation/DVLdataClass.h>
#include <labust/navigation/NavQuestMessages.hpp>
#include <labust/archive/delimited_iarchive.hpp>
#include <labust/archive/delimited_oarchive.hpp>
#include <labust/preprocessor/clean_serializator.hpp>
#include <labust/tools/conversions.hpp>
#include <labust/math/NumberManipulation.hpp>
#include <labust/tools/StringUtilities.hpp>

#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
// #include <cola2_msgs/LinkquestDvl.h>
#include <string>

#include <boost/bind.hpp>
#include <boost/regex.hpp>
#include <boost/serialization/string.hpp>

#include <iosfwd>
#include <algorithm>

#include <signal.h>
#include <glider_msg/dvlmsg.h>

PP_LABUST_CLEAN_ARRAY_OSERIALIZATOR_IMPL(labust::archive::delimited_oarchive)
PP_LABUST_CLEAN_ARRAY_ISERIALIZATOR_IMPL(labust::archive::delimited_iarchive)

using namespace labust::navigation;
glider_msg::dvlmsg dvl;

int labust::navigation::error_code(const NQRes& data)
{
	std::stringstream ss(data.error_code);
	int error;
	ss>>std::hex>>error;
	return error;
}
// defining constructor for class named NavQuestNode
NavQuestNode::NavQuestNode():
// io is input output operation
// port to communicate serial device
// useFixed to determines wether the node used fix orientation or not
// base Orientation is double value representing orientation of the base of the robot.
					io(),
					port(io),
					useFixed(true),
					base_orientation(0)
{
	// on Init function is then called
	this->onInit();
}


NavQuestNode::~NavQuestNode()
{
	// std::string stop_command("#&!LQNQ.COMD2828");
	// port.write_some(boost::asio::buffer(stop_command, stop_command.size()));
	io.stop();
	runner.join();
}

void NavQuestNode::onInit()
{
	ros::NodeHandle nh, ph("~");
	ros::Rate r(1);
	bool setupOk(false);

	while (!(setupOk = this->setup_port()) && ros::ok())
	{
		ROS_ERROR("NavQuestNode::Failed to open port.");
		r.sleep();

                std::cout<<"port salah cukk !!!"<<"\n";           // added by Jenana for testing
	}
	std::string start_command("#&!LQNQ.COMD2525");
	port.write_some(boost::asio::buffer(start_command, start_command.size()));
	if (setupOk)
	{
		//Advertise beam data
		// pub_dvl = nh.advertise<cola2_msgs::LinkquestDvl>("/gb_navigation/DVL", 1);
		// beam_pub["velo_rad"].reset(new NavQuestBP(nh, "velo_rad"));
		// beam_pub["wvelo_rad"].reset(new NavQuestBP(nh, "wvelo_rad"));
		// beam_pub["altitude_beams"].reset(new NavQuestBP(nh, "altitude_beams"));
		// speed_pub["velo_instrument"].reset(
		// 		new TwistPublisher(nh, "velo_instrument", "dvl_frame"));
		// speed_pub["velo_earth"].reset(
		// 		new TwistPublisher(nh, "velo_earth", "local"));
		// speed_pub["water_velo_instrument"].reset(
		// 		new TwistPublisher(nh, "water_velo_instrument", "dvl_frame"));
		// speed_pub["water_velo_earth"].reset(
		// 		new TwistPublisher(nh, "water_velo_earth", "local"));
		// lock = nh.advertise<std_msgs::Bool>("dvl_bottom",1);
		// altitude = nh.advertise<std_msgs::Float32>("altitude",1);
		// imuPub = nh.advertise<auv_msgs::RPY>("dvl_rpy",1);

		useFixed = ph.getParam("fixed_orientation", base_orientation);
		nh.param("magnetic_declination",magnetic_declination, 0.0);

		dvlMessage = nh.advertise<glider_msg::dvlmsg>("messageDVL", 100);
		//Start the receive cycle
		this->start_receive();
		runner = boost::thread(boost::bind(&boost::asio::io_service::run,&io));
		setup_messaging();

		std::cout<<"port gada masalah cuk, UART4 bener berrti"<<"\n";          // added by Jenana for testing
	}
}

void NavQuestNode::setup_messaging()
{
	//Setup map for more than one message
}

void NavQuestNode::setup_publishers()
{
	//Setup maps for publishing
}

void NavQuestNode::start_receive()
{
	using namespace boost::asio;
	async_read_until(port, buffer,
			boost::regex("\r\n"),
			boost::bind(&NavQuestNode::onDvlData, this, _1,_2));

	std::cout<<"Sampai start receive data sudah bisa berrti disini !!!"<<"\n";	// added by Jenana for  testing
}

bool NavQuestNode::setup_port()
{
	ros::NodeHandle ph("~");
	std::string portName("/dev/ttyACM0");
	//std::string portName("/dev/ttyO4");
	int baud(115200);

	ph.param("PortName",portName,portName);
	ph.param("Baud",baud,baud);

	using namespace boost::asio;
	port.open(portName);
	port.set_option(serial_port::baud_rate(baud));
	port.set_option(serial_port::flow_control(
			serial_port::flow_control::none));
			

	return port.is_open();
}

bool NavQuestNode::test_header(const std::string& match, const std::string& stream)
{
	return stream.substr(0,match.size()) == match;
}

void NavQuestNode::publishDvlData(const NQRes& data, std::string rawData)
{
	geometry_msgs::TwistStamped::Ptr twist(new geometry_msgs::TwistStamped());

	bool testDVL = data.velo_instrument[0] == data.velo_instrument[1];
	testDVL = testDVL && (data.velo_instrument[1] == data.velo_instrument[2]);
	testDVL = testDVL && (data.velo_instrument[2] == 0);

	if (error_code(data) !=0) return;

	//Data validity
	bool beamValidity = true;
	for (int i=0; i<4; ++i)
	{
	  beamValidity = beamValidity && (data.beam_status[i] == 1);
	  ROS_INFO("Beam validity %d: %d", i, data.beam_status[i]);
	  ROS_INFO("Water vel credit %d: %f", i, data.wvelo_credit[i]);
	}


	if (testDVL)
	{
	  ROS_INFO("All zero dvl. Ignore measurement.");
	  return;
	}
	
	if (!beamValidity)
	{
	  ROS_WARN("One or more beams are invalid. Ignore measurement: %f %f", data.velo_instrument[0]/1000, data.velo_instrument[1]/1000);
	  return;
	}
	else
	{
	  ROS_DEBUG("Beams are valid. Accept measurement: %f %f", data.velo_instrument[0]/1000, data.velo_instrument[1]/1000);
	}

	// dvl.header.stamp= ros::Time::now() ;
	// dvl.header.frame_id = "GaneshBlue/dvl";
	
	for (int i=0; i<4; ++i)
	{
		dvl.dataGood[i] = data.beam_status[i];
		dvl.bottomVelocityBeam[i] = data.velo_rad[i]/1000 ;
		dvl.waterVelocityBeam[i] = data.wvelo_rad[i]/1000 ;
		dvl.waterVelocityCredit[i] = data.wvelo_credit[i];
	
	}  
	
	for (int i=0; i<3; ++i)
	{
		dvl.velocityInst[i] = data.velo_instrument[i]/1000 ;
		dvl.velocityEarth[i] = data.velo_earth[i]/1000 ;
		dvl.waterVelocityInst[i] = data.water_velo_instrument[i]/1000 ;
		dvl.waterVelocityEarth[i] = data.water_velo_earth[i]/1000 ;
	}  

	dvl.velocityInstFlag = data.velo_instrument[3] ;
	dvl.velocityEarthFlag = data.velo_earth[3] ;
	dvl.waterVelocityInstFlag = data.water_velo_instrument[3] ;
	dvl.waterVelocityEarthFlag = data.water_velo_earth[3] ;  

	// pub_dvl.publish(dvl);

	dvl.roll = data.rph[0];
	dvl.pitch = data.rph[1];
	dvl.heading = data.rph[2];
         
	// (*beam_pub["velo_rad"])(data.velo_rad);
	// (*beam_pub["wvelo_rad"])(data.wvelo_rad);
	// (*beam_pub["altitude_beams"])(data.v_altitude);
	// (*speed_pub["velo_instrument"])(data.velo_instrument);
	// (*speed_pub["velo_earth"])(data.velo_earth);
	// (*speed_pub["water_velo_instrument"])(data.water_velo_instrument);
	// (*speed_pub["water_velo_earth"])(data.water_velo_earth);
	dvl.rawData = rawData;

	dvl.temperature = data.temperature;
	dvl.salinity = data.salinity;
	dvl.pressure = data.pressure;
	dvl.soundSpeed = data.sound_speed;

	//Bottom lock flag
	enum{valid_flag=3};
	bool water_lock= (data.velo_instrument[valid_flag]==2) || (data.velo_earth[valid_flag]==2);
	bool valid=data.velo_instrument[valid_flag] && data.velo_earth[valid_flag];
	std_msgs::Bool bottom_lock;
	bottom_lock.data = !water_lock && valid;
	// lock.publish(bottom_lock);
	//ROS_INFO("Has bottom lock %d", bottom_lock.data);

	
	//Altitude
	if (data.altitude_estimate > 0)
	{
	  std::vector<double> salt;
	  for (int i=0; i < 4; ++i) {
	  salt.push_back(data.v_altitude[i]);
	  dvl.altitudeBeam[i] = data.v_altitude[i];
	  }
	  std::sort(salt.begin(), salt.end());

	//   std_msgs::Float32Ptr alt(new std_msgs::Float32());
	  dvl.altitude = (salt[1] + salt[2])/2;
	//   altitude.publish(alt);
	}

	//RPY
	// auv_msgs::RPY::Ptr rpy(new auv_msgs::RPY());
	// rpy->roll  = data.rph[roll ];
	// rpy->pitch = data.rph[pitch];
	// rpy->yaw   = data.rph[yaw  ];
	// rpy->roll = labust::math::wrapRad(data.rph[roll]/180*M_PI);
	// rpy->pitch = labust::math::wrapRad(data.rph[pitch]/180*M_PI);
	// rpy->yaw = labust::math::wrapRad(data.rph[yaw]/180*M_PI);
	// imuPub.publish(rpy);

	dvlMessage.publish(dvl);
}

void NavQuestNode::conditionDvlData(NQRes& data)
{
	//Make the instrument coordinate system right handed by converting Z from up to down
	//TODO correct adequately the earth values if needed
	data.velo_instrument[2] *= -1;
	data.water_velo_instrument[2] *= -1;
}

void NavQuestNode::onDvlData(const boost::system::error_code& e,
		std::size_t size)
{

/*
    if (!e){
	std::istream is(&buffer);
	std::cout<<"gada error di onDVL function"<<"\n";
        
	std::string data(size,'\0');
	is.read(&data[0],size);
        if (test_header("$#NQ.RES",data)){
              std::cout<<"header yang diset sama "<<"\n";
	      std::cout<<"data sama :"<<data<<"\n";
        } else {
	    std::cout<<"header yang di set tidak sama "<<"\n";
	    std::cout<<"data tidak sama :"<<data<<"\n";
	}
	
    }else{
	std::cout<<"error nih di onDVL function"<<"\n";
    }

*/

// code di bawah ini adalah code  defaultnya , jadi pake yang dibawah yakk 

	if (!e)
	{
		std::istream is(&buffer);
		std::string data(size,'\0');
		is.read(&data[0],size);

		if (test_header("$#NQ.RES", data))
		{
			NQRes dvl_data;
			int chk = labust::tools::getChecksum(reinterpret_cast<unsigned char*>(&data[15]), data.size()-3-15);
			std::istringstream is;
			is.rdbuf()->pubsetbuf(&data[0],size);
			labust::archive::delimited_iarchive ia(is);
			ia>>dvl_data;
			
			//if (error_code(dvl_data) == 0) publishDvlData(dvl_data);
			conditionDvlData(dvl_data);
			publishDvlData(dvl_data, data);
			ROS_INFO("Calculated checksum:calc=%d, recvd=%d", chk, dvl_data.checksum);
			
			std::cout<<"Sudah berhasil ini"<<"\n";
			std::cout<<"Data Benar :"<<data<<"\n";
			//ROS_INFO("DVL decoded: header:%s, error:%s.",
			//		dvl_data.header.c_str(),
			//		dvl_data.error_code.c_str());
		}
	}
	else
	{
		ROS_ERROR("NavQuestNode: %s",e.message().c_str());
	}

	this->start_receive();


	// commment by Jenana for testing or debugging
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "node_DVL");
	NavQuestNode node;
	ros::spin();

	return 0;
}


