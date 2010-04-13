#include <cstdlib>
#include <iostream>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include "boost/date_time/posix_time/posix_time_types.hpp"
#include "TimeStamp.h""


#ifndef UDPSOCKET_INCLUDEDEF_H
#define UDPSOCKET_INCLUDEDEF_H

union DblBuf
{
  double d;
  char buf[sizeof(double)];
};


using boost::asio::ip::udp;
class RCVServer
{
public:
  RCVServer(boost::asio::io_service& io_service, short port)
    : io_service_(io_service),
      socket_(io_service, udp::endpoint(udp::v4(), port))
  {
    m_CurrentValues.push_back(0.0);
    m_CurrentValues.push_back(0.0);
    m_CurrentValues.push_back(0.0);
   m_TimeStamp.SetNow();
    //    RCV_service = io_service;
    socket_.async_receive_from(
        boost::asio::buffer(data_, max_length), sender_endpoint_,
        boost::bind(&RCVServer::handle_receive_from, this,
          boost::asio::placeholders::error,
          boost::asio::placeholders::bytes_transferred));
  }

  boost::asio::io_service RCV_service;


  void handle_receive_from(const boost::system::error_code& error,
      size_t bytes_recvd)
  {
    std::vector<double> v;
    if(bytes_recvd >= 3*sizeof(double))
    {
        for(int i=0; i < 3; i++)
        {
            union DblBuf db;
            for(int j=0;j<sizeof(double);j++)
                db.buf[j] = data_[(j+i*sizeof(double))];
            v.push_back(db.d);
        }
    }    
    m_CurrentValues = v;
	m_TimeStamp.SetNow();
    if (!error && bytes_recvd > 0)
    {
      socket_.async_send_to(
          boost::asio::buffer(data_, bytes_recvd), sender_endpoint_,
          boost::bind(&RCVServer::handle_send_to, this,
            boost::asio::placeholders::error,
            boost::asio::placeholders::bytes_transferred));
    }
    else
    {
      socket_.async_receive_from(
          boost::asio::buffer(data_, max_length), sender_endpoint_,
          boost::bind(&RCVServer::handle_receive_from, this,
            boost::asio::placeholders::error,
            boost::asio::placeholders::bytes_transferred));
    }
  }

  void handle_send_to(const boost::system::error_code& error, size_t bytes_sent)
  {
    socket_.async_receive_from(
        boost::asio::buffer(data_, max_length), sender_endpoint_,
        boost::bind(&RCVServer::handle_receive_from, this,
          boost::asio::placeholders::error,
          boost::asio::placeholders::bytes_transferred));
  }

  std::vector<double> getCurrentValues()
    {
        std::vector<double> ret = m_CurrentValues;
		Neobotix::TimeStamp curtime;
		curtime.SetNow();	
		if((curtime - m_TimeStamp)>0.1)
		  {
		    std::vector<double> zero;
		    zero.push_back(0.0);
		    zero.push_back(0.0);
		    zero.push_back(0.0);
		    m_CurrentValues = zero;
		  }
        return ret;

    }
  
  boost::asio::io_service& io_service_;
private:
  udp::socket socket_;
  udp::endpoint sender_endpoint_;
  enum { max_length = 1024 };
  char data_[max_length];
  std::vector<double> m_CurrentValues;
  Neobotix::TimeStamp m_TimeStamp;
};


class SNDServer
{
public:
  SNDServer(boost::asio::io_service& io_service,
      const boost::asio::ip::address& multicast_address, short port)
    : endpoint_(multicast_address, port),
      socket_(io_service, udp::v4()),
      timer_(io_service),
      message_count_(0)
  {
    std::ostringstream os;
    std::cout << "Message " << message_count_ << "\n";
    os << "Message " << message_count_++;
    message_ = os.str();

    socket_.async_send_to(
        boost::asio::buffer(message_), endpoint_,
        boost::bind(&SNDServer::handle_send_to, this,
          boost::asio::placeholders::error));
  }

  void handle_send_to(const boost::system::error_code& error)
  {
    if (!error)
    {
      /*timer_.expires_from_now(boost::posix_time::seconds(1));
      timer_.async_wait(
          boost::bind(&SNDServer::handle_timeout, this,
            boost::asio::placeholders::error));
        */
    }
    else
    {
        std::cout << "Send error\n";
    }
  }

  void handle_timeout(const boost::system::error_code& error)
  {
    if (!error)
    {
      std::ostringstream os;
      std::cout << "Message " << message_count_ << "\n";
      os << "Message " << message_count_++;
      message_ = os.str();

      socket_.async_send_to(
          boost::asio::buffer(message_), endpoint_,
          boost::bind(&SNDServer::handle_send_to, this,
            boost::asio::placeholders::error));
    }
    else
    {
        std::cout << "error\n";
    }
  }
  void sendPose(double x, double y, double z)
    {
        char request[3*sizeof(double)] ;
        union DblBuf db;
        db.d = x;
        for(int i = 0; i < sizeof(double);i++)
            request[i] = db.buf[i];
        db.d = y;
        for(int i = 0; i < sizeof(double);i++)
            request[i+sizeof(double)] = db.buf[i];
        db.d = z;
        for(int i = 0; i < sizeof(double);i++)
            request[i+2*sizeof(double)] = db.buf[i];
        size_t request_length = 3*sizeof(double);
        

        socket_.async_send_to(
          boost::asio::buffer(request, request_length), endpoint_,
          boost::bind(&SNDServer::handle_send_to, this,
            boost::asio::placeholders::error));
    }

  void sendForce(double Fx, double Fy, double Fz, double Tx, double Ty, double Tz)
    {
        char request[6*sizeof(double)] ;
        union DblBuf db;
        db.d = Fx;
        for(int i = 0; i < sizeof(double);i++)
	  request[i] = db.buf[i];
        db.d = Fy;
        for(int i = 0; i < sizeof(double);i++)
	  request[i+sizeof(double)] = db.buf[i];
        db.d = Fz;
        for(int i = 0; i < sizeof(double);i++)
	  request[i+2*sizeof(double)] = db.buf[i];
	db.d = Tx;
	for(int i = 0; i < sizeof(double);i++)
	  request[i+3*sizeof(double)] = db.buf[i];
	db.d = Ty;
	for(int i = 0; i < sizeof(double);i++)
	  request[i+4*sizeof(double)] = db.buf[i];
	db.d = Tz;
	for(int i = 0; i < sizeof(double);i++)
	  request[i+5*sizeof(double)] = db.buf[i];
        size_t request_length = 6*sizeof(double);
        

        socket_.async_send_to(
          boost::asio::buffer(request, request_length), endpoint_,
          boost::bind(&SNDServer::handle_send_to, this,
            boost::asio::placeholders::error));
    }

private:
  boost::asio::ip::udp::endpoint endpoint_;
  boost::asio::ip::udp::socket socket_;
  boost::asio::deadline_timer timer_;
  int message_count_;
  std::string message_;
};


#endif
