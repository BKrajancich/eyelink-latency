#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <msgpack.hpp>
#include <zmq.hpp>
#include <zmq_addon.hpp>

using namespace std;

int main()
{
  zmq::context_t context;

  zmq::socket_t sock( context, zmq::socket_type::req );
  fprintf( stderr, "Connecting to socket.\n" );
  sock.connect( "tcp://127.0.0.1:4587" );
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
