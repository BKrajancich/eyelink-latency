#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <msgpack.hpp>
#include <zmq.hpp>
#include <zmq_addon.hpp>

using namespace std;

struct SetCalibrationToHMD
{
  std::string subject;
  std::string name;
  float args[1];
  MSGPACK_DEFINE_MAP(subject, name, args)
};

struct CalibrationShouldStartStruct
{
    std::string subject;
    float translation_eye0[3]; //Like an FVector
    float translation_eye1[3];
    bool record;
    MSGPACK_DEFINE_MAP(subject, translation_eye0, translation_eye1, record)
};

struct StartEyeWindow
{
  std::string subject;
  int eye_id;
  std::string args;
  MSGPACK_DEFINE_MAP(subject, eye_id, args)
};

// struct datum {

// };

// struct CalibrationData {

// };

// struct StopCalibration {

// };
 
int main() {
  zmq::context_t context;

  zmq::socket_t sock( context, zmq::socket_type::req );
  fprintf( stderr, "Connecting to socket.\n" );
  sock.connect( "tcp://127.0.0.1:50020" );
  fprintf( stderr, "Connected.\n" );
  // sock.send(zmq::str_buffer("R"), zmq::send_flags::dontwait);
  // fprintf(stderr, "Sent.\n");

  fprintf( stderr, "Send SUB_PORT.\n" );
  sock.send( zmq::str_buffer( "SUB_PORT" ), zmq::send_flags::dontwait );
  zmq::message_t sub_port;
  fprintf( stderr, "Recv SUB_PORT.\n" );
  auto ret = sock.recv( sub_port, zmq::recv_flags::none );
  if ( !ret )
    return 1;
  cout << "SUB_PORT: " << sub_port.to_string() << "\n";

  sock.send( zmq::str_buffer( "PUB_PORT" ), zmq::send_flags::dontwait );
  zmq::message_t pub_port;
  ret = sock.recv( pub_port, zmq::recv_flags::none );
  if ( !ret )
    return 1;
  cout << "PUB_PORT: " << pub_port.to_string() << "\n";

  msgpack::sbuffer sbuf;
  StartEyeWindow eye0_window = { "eye_process.should_start.0", 0, "" };
  msgpack::pack(sbuf, eye0_window);



  // auto oh = msgpack::unpack(ss.str().data(), ss.str().size());
  // std::cout << oh.get() << std::endl;

  // SetCalibrationToHMD setCalib = { "start_plugin", "HMD3DChoreographyPlugin", {} };
  // msgpack::pack(ss, setCalib);
  // auto oh2 = msgpack::unpack(ss.str().data(), ss.str().size());
  // std::cout << oh2.get() << std::endl;

  // CalibrationShouldStartStruct startCalib = { "calibration.should_start", {34.75,0,0}, {-34.75,0,0}, true };
  // msgpack::pack(ss, startCalib);
  // auto oh3 = msgpack::unpack(ss.str().data(), ss.str().size());
  // std::cout << oh3.get() << std::endl;

  // PLAY WITH SENDING (lines 19-21 in python version)

  auto notify = "notify." + eye0_window.subject;
  sock.send( zmq::buffer(notify), zmq::send_flags::sndmore );

  zmq::message_t request(sbuf.size());
  memcpy((void *) request.data(), sbuf.data(), sbuf.size());
  sock.send( request, zmq::send_flags::none );

  zmq::message_t status;
  auto ret1 = sock.recv( status, zmq::recv_flags::none );
  if ( !ret1 )
    return 1;
  cout << "STATUS: " << status.to_string() << "\n";
}


int test()
{
  zmq::context_t context;

  zmq::socket_t sock( context, zmq::socket_type::req );
  fprintf( stderr, "Connecting to socket.\n" );
  sock.connect( "tcp://127.0.0.1:50020" );
  fprintf( stderr, "Connected.\n" );
  // sock.send(zmq::str_buffer("R"), zmq::send_flags::dontwait);
  // fprintf(stderr, "Sent.\n");

  fprintf( stderr, "Send SUB_PORT.\n" );
  sock.send( zmq::str_buffer( "SUB_PORT" ), zmq::send_flags::dontwait );
  zmq::message_t sub_port;
  fprintf( stderr, "Recv SUB_PORT.\n" );
  auto ret = sock.recv( sub_port, zmq::recv_flags::none );
  if ( !ret )
    return 1;
  cout << "SUB_PORT: " << sub_port.to_string() << "\n";

  sock.send( zmq::str_buffer( "PUB_PORT" ), zmq::send_flags::dontwait );
  zmq::message_t pub_port;
  ret = sock.recv( pub_port, zmq::recv_flags::none );
  if ( !ret )
    return 1;
  cout << "PUB_PORT: " << pub_port.to_string() << "\n";

  zmq::socket_t subscriber( context, zmq::socket_type::sub );
  subscriber.connect( "tcp://127.0.0.1:" + sub_port.to_string() );

  subscriber.setsockopt( ZMQ_SUBSCRIBE, "gaze", strlen( "gaze" ) );

  while ( true ) {
    vector<zmq::message_t> recv_msgs;
    ret = zmq::recv_multipart( subscriber, std::back_inserter( recv_msgs ) );
    if ( !ret )
      return 1;

    msgpack::object_handle oh = msgpack::unpack( (const char*)recv_msgs[1].data(), recv_msgs[1].size() );
    msgpack::object obj = oh.get();
    // cout << obj << endl;
    string eye = obj.via.array.ptr[1].as<std::string>();
    cout << "eye: " << eye[8] << endl;
    float timestamp = obj.via.array.ptr[7].as<float>();
    cout << "timestamp: " << timestamp << endl;
    float pos1 = obj.via.array.ptr[3].via.array.ptr[0].as<float>();
    float pos2 = obj.via.array.ptr[3].via.array.ptr[1].as<float>();
    cout << "pos: "
         << "[" << pos1 << ", " << pos2 << "]" << endl;
  }

  return 0;
}
