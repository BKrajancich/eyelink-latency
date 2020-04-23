#include <chrono>
#include <cstring>
#include <exception>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <thread>

#include <msgpack.hpp>
#include <unistd.h>
#include <zmq.hpp>
#include <zmq_addon.hpp>

#include "cairo_objects.hh"
#include "display.hh"

using namespace std;
using namespace std::chrono;

void program_body()
{
  VideoDisplay display { 1920, 1080, false }; // fullscreen window @ 1920x1080 luma resolution

  Cairo cairo { 1920, 1080 };
  Pango pango { cairo };

  /* open the PNG */
  PNGSurface png_image { "/home/brooke/repos/eyelink-latency/src/files/testim.png" };

  /* draw gray over everything */
  cairo_new_path( cairo );
  cairo_identity_matrix( cairo );
  cairo_rectangle( cairo, 500, 500, 100, 100 );
  cairo_set_source_rgba( cairo, 0, 0.9, 0, 0.5 );
  cairo_fill( cairo );

  /* draw the PNG */
  cairo_identity_matrix( cairo );
  cairo_scale( cairo, 0.8, 0.8 );
  double center_x = 960, center_y = 540;
  cairo_device_to_user( cairo, &center_x, &center_y );
  cairo_translate( cairo, center_x, center_y );
  cairo_set_source_surface( cairo, png_image, 0, 0 );
  cairo_paint( cairo );

  /* draw some text */
  Pango::Font myfont { "Times New Roman, 80" };
  Pango::Text mystring { cairo, pango, myfont, "Hello, world, Brooke, and Luke." };
  mystring.draw_centered_at( cairo, 960, 540 );
  cairo_set_source_rgba( cairo, 1, 1, 1, 1 );
  cairo_fill( cairo );

  /* finish and copy to YUV raster */
  cairo.flush();

  unsigned int stride = cairo.stride();
  Raster420 yuv_raster { 1920, 1080 };
  for ( unsigned int y = 0; y < 1080; y++ ) {
    for ( unsigned int x = 0; x < 1920; x++ ) {
      yuv_raster.Y.at( x, y ) = cairo.pixels()[y * stride + 1 + ( x * 4 )];
    }
  }

  Texture420 texture { yuv_raster };
  display.draw( texture );
  pause();
}

void writeTextRaster( Raster420& yuv_raster, float pos_x, float pos_y )
{
  Cairo cairo { 1920, 1080 };
  Pango pango { cairo };
  cairo_new_path( cairo );

  Pango::Font myfont { "Times New Roman, 80" };
  Pango::Text mystring { cairo, pango, myfont, "Eye" };
  mystring.draw_centered_at( cairo, pos_x * 1920, pos_y * 1080 );
  cairo_set_source_rgba( cairo, 1, 1, 1, 1 );
  cairo_fill( cairo );
  cairo.flush();

  unsigned int stride = cairo.stride();
  for ( unsigned int y = 0; y < 1080; y++ ) {
    for ( unsigned int x = 0; x < 1920; x++ ) {
      yuv_raster.Y.at( x, y ) = cairo.pixels()[y * stride + 1 + ( x * 4 )];
    }
  }
}

int gazeUpdate( float& pos_x )
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
    // string eye = obj.via.array.ptr[1].as<std::string>();
    // cout << "eye: " << eye[8] << endl;
    // float timestamp = obj.via.array.ptr[7].as<float>();
    // cout << "timestamp: " << timestamp << endl;
    float pos1 = obj.via.array.ptr[3].via.array.ptr[0].as<float>();
    // float pos2 = obj.via.array.ptr[3].via.array.ptr[1].as<float>();
    // cout << "pos: " << "[" << pos1 << ", " << pos2 << "]" << endl;
    if ( !isnan( pos1 ) ) {
      pos_x = pos1;
    }
  }
  return 0;
}

void test()
{
  VideoDisplay display { 1920, 1080, false }; // fullscreen window @ 1920x1080 luma resolution
  Raster420 yuv_raster { 1920, 1080 };
  float pos_x;
  gazeUpdate( pos_x );
  writeTextRaster( yuv_raster, pos_x, 0.5 );
  Texture420 texture { yuv_raster };
  display.draw( texture );
  pause();
}

int main()
{
  try {
    // program_body();
    test();
  } catch ( const exception& e ) {
    cerr << "Exception: " << e.what() << "\n";
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
