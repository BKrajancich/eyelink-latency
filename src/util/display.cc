/* -*-mode:c++; tab-width: 2; indent-tabs-mode: nil; c-basic-offset: 2 -*- */

/* Copyright 2013-2018 the Alfalfa authors
                       and the Massachusetts Institute of Technology
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are
   met:
      1. Redistributions of source code must retain the above copyright
         notice, this list of conditions and the following disclaimer.
      2. Redistributions in binary form must reproduce the above copyright
         notice, this list of conditions and the following disclaimer in the
         documentation and/or other materials provided with the distribution.
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
   HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

#include "display.hh"

using namespace std;

const string VideoDisplay::shader_source_scale_from_pixel_coordinates = R"( #version 130
      uniform uvec2 window_size;
      in vec2 position;
      in vec2 chroma_texcoord;
      out vec2 Y_texcoord;
      out vec2 uv_texcoord;
      void main()
      {
        gl_Position = vec4( 2 * position.x / window_size.x - 1.0,
                            1.0 - 2 * position.y / window_size.y, 0.0, 1.0 );
        Y_texcoord = vec2( position.x, position.y );
        uv_texcoord = vec2( chroma_texcoord.x, chroma_texcoord.y );
      }
    )";

/* octave> 255 * inv([219*[.7152 .0722 .2126]'
                      224*[-0.38542789394266 .5 -0.11457210605734]'
                      224*[-0.454152908305817 -0.0458470916941834 .5]']') */

/* switched to SMPTE 170M matrix (1/21/2017)
>> 255 * inv([219*[.587 .114 .299]' 224*[-.331 .500 -.169]' 224*[-.419 -.081 .5]']')
ans =
      1.16438356164384    -0.391260370716072    -0.813004933873461
      1.16438356164384      2.01741475897078   0.00112725996069348
      1.16438356164384  -0.00105499970680283      1.59567019581339
*/

const string VideoDisplay::shader_source_ycbcr = R"( #version 130
      #extension GL_ARB_texture_rectangle : enable
      precision mediump float;
      uniform sampler2DRect yTex;
      uniform sampler2DRect uTex;
      uniform sampler2DRect vTex;
      uniform vec3 head_orientation;
      uniform int vr_view;
      in vec2 Y_texcoord;
      in vec2 uv_texcoord;
      out vec4 outColor;   

      vec2 screen_res = vec2( 1920.0, 1080.0 ); 

      mat4 eul2rotm4( float rotX, float rotY, float rotZ ) {
        mat4 R_x = mat4(	1.0f,		0f,			0f,  0f,
              0f, 	cos(rotX),	sin(rotX), 0f,
              0f, 	-sin(rotX),	 cos(rotX), 0f,
              0f, 0f, 0f, 1.0f);
        mat4 R_y = mat4( cos(rotY),	0f, -sin(rotY), 0f,
                0f, 	1.0f,	0f, 0f,                                              
                sin(rotY), 	0f,	 cos(rotY), 0f,
                0f, 0f, 0f, 1.0f);
        mat4 R_z = mat4( cos(rotZ),	sin(rotZ), 0f, 0f,
                -sin(rotZ),	 cos(rotZ),	0f, 0f,
                0f, 	0f,	 1.0f, 0f,
                0f, 0f, 0f, 1.0f);


                                
        return R_z * R_y * R_x;        
      }  

      mat4 invCamMat = mat4(0.0015f, 0f, 0f, 0f,
                          0f, 0.0015f, 0f, 0f,
                          -1.4286f, -0.8036f, 1.0f, 0f,
                          0f, 0f, 0f, 1.0f);

      mat4 invCamMat_HMD_L = mat4(1.27908f, 0f, 0f, 0f,
                              0f, 1.42243f, 0f, 0f,
                              0f, 0f, 0f,  -1.66617f,
                              -0.07645f, -0.00716f, -1.0f, 1.66717f);

      mat4 invCamMat_HMD_R = mat4(1.28131f, 0f, 0f, 0f,
                              0f, 1.42354f, 0f, 0f,
                              0f, 0f, 0f, -1.66617f,
                              0.07635f, -0.00123f, -1.0f, -1.66717f);

      mat4 eyePos_L = mat4(1.0f, 0.0f, 0.0f, 0.0f,
                           0.0f, 1.0f, 0.0f, 0.0f,
                           0.0f, 0.0f, 1.0f, 0.0f,
                           -0.03345f, 0.0f, -0.15f, 1.0f);
      mat4 eyePos_R = mat4(1.0f, 0.0f, 0.0f, 0.0f,
                           0.0f, 1.0f, 0.0f, 0.0f,
                           0.0f, 0.0f, 1.0f, 0.0f,
                           0.03345f, 0.0f, -0.15f, 1.0f);

      vec2 get_latlong( vec2 texcoord ) {

        vec3 xyz = vec3( texcoord.x, texcoord.y, 1.0);
        vec3 ndc = vec3(2*texcoord.x/1440.0 -1, 2*texcoord.y/1600.0 -1, 1.0);
        vec4 clip = 1000*vec4(ndc.x, ndc.y, ndc.z, 1.0f);
        
        mat4 transform;

        if (Y_texcoord.x < 1440.0) {
          transform = invCamMat_HMD_L * eyePos_L;
        } else {
          transform = invCamMat_HMD_R * eyePos_R;
        }

        vec4 ray4d = eul2rotm4(head_orientation.x, head_orientation.y, head_orientation.z) * transform * clip;     
        vec3 ray3d = ray4d.xyz;   
        float theta = atan( ray3d.y, length(ray3d.xz) );
        float phi = atan( ray3d.x, ray3d.z );
        return vec2( phi, theta);
      }
      vec2 reproject_Y (vec2 texcoord ) {
        vec2 phi_theta = get_latlong(texcoord);
        vec2 xy_sphere = vec2( ((phi_theta.x / 3.14f) * 1920.0 + 1920.0)/2.0, (phi_theta.y + 3.14/2.0) * 1080.0 /3.14 );
        return xy_sphere;
      }
      vec2 reproject_uv( vec2 texcoord ) {
        vec2 phi_theta = get_latlong(texcoord);
        vec2 xy_sphere = vec2( ((phi_theta.x / 3.14f) * (1920.0/2.0) + (1920.0/2.0))/2.0, (phi_theta.y + 3.14/2.0) * (1080.0/2.0) /3.14 );
        return xy_sphere;
      }
      void main()
      {
        if (vr_view == 1) {
           screen_res = vec2( 1440.0, 1600.0 );
        }

        vec2 Y_reprojected = reproject_Y(mod(Y_texcoord, screen_res.x));
        vec2 uv_reprojected = reproject_uv(mod(Y_texcoord, screen_res.x)) + max( 0.0, min( 0.0, texture(uTex, uv_texcoord).x ) );  // to get rid of inefficiency bug
        float fY = texture(yTex, Y_reprojected).x;
        float fCb = texture(uTex, uv_reprojected).x;
        float fCr = texture(vTex, uv_reprojected).x;
        
        outColor = vec4(
          max(0, min(1.0, 1.16438356164384 * (fY - 0.06274509803921568627) + 1.59567019581339  * (fCr - 0.50196078431372549019))),
          max(0, min(1.0, 1.16438356164384 * (fY - 0.06274509803921568627) - 0.391260370716072 * (fCb - 0.50196078431372549019) - 0.813004933873461 * (fCr - 0.50196078431372549019))),
          max(0, min(1.0, 1.16438356164384 * (fY - 0.06274509803921568627) + 2.01741475897078  * (fCb - 0.50196078431372549019))),
          1.0
        );


      }
    )";

VideoDisplay::CurrentContextWindow::CurrentContextWindow( const unsigned int width,
                                                          const unsigned int height,
                                                          const string& title,
                                                          const bool fullscreen )
  : window_( width, height, title, fullscreen )
{
  window_.make_context_current();
}

VideoDisplay::VideoDisplay( const unsigned int width, const unsigned int height, const bool fullscreen, const bool vr_view )
  : width_( width )
  , height_( height )
  , current_context_window_( width_, height_, "OpenGL Example", fullscreen )
{
  texture_shader_program_.attach( scale_from_pixel_coordinates_ );
  texture_shader_program_.attach( ycbcr_shader_ );
  texture_shader_program_.link();
  glCheck( "after linking texture shader program" );

  texture_shader_array_object_.bind();
  ArrayBuffer::bind( screen_corners_ );
  glVertexAttribPointer(
    texture_shader_program_.attribute_location( "position" ), 2, GL_FLOAT, GL_FALSE, sizeof( VertexObject ), 0 );
  glEnableVertexAttribArray( texture_shader_program_.attribute_location( "position" ) );

  glVertexAttribPointer( texture_shader_program_.attribute_location( "chroma_texcoord" ),
                         2,
                         GL_FLOAT,
                         GL_FALSE,
                         sizeof( VertexObject ),
                         (const void*)( 2 * sizeof( float ) ) );
  glEnableVertexAttribArray( texture_shader_program_.attribute_location( "chroma_texcoord" ) );

  const auto window_size = window().framebuffer_size();
  resize( window_size.first, window_size.second );

  int vr_flag = 0;

  if (vr_view) {
    vr_flag = 1;
  }

  texture_shader_program_.use();
  glUniform1i( texture_shader_program_.uniform_location( "vr_view" ), vr_flag );

  glCheck( "VideoDisplay constructor" );
}

void VideoDisplay::update_head_orientation( const float x, const float y, const float z )
{
  texture_shader_program_.use();
  glUniform3f( texture_shader_program_.uniform_location( "head_orientation" ), x, y, z );
}

void VideoDisplay::resize( const unsigned int width, const unsigned int height )
{
  glViewport( 0, 0, width, height );

  texture_shader_program_.use();
  glUniform2ui( texture_shader_program_.uniform_location( "window_size" ), width, height );

  glUniform1i( texture_shader_program_.uniform_location( "yTex" ), 0 );
  glUniform1i( texture_shader_program_.uniform_location( "uTex" ), 1 );
  glUniform1i( texture_shader_program_.uniform_location( "vTex" ), 2 );

  const float xoffset = 0.25;

  float width_float = width;
  float height_float = height;

  vector<VertexObject> corners = { { 0, 0, xoffset, 0 },
                                   { 0, height_float, xoffset, height_float / 2 },
                                   { width_float, height_float, width_float / 2 + xoffset, height_float / 2 },
                                   { width_float, 0, width_float / 2 + xoffset, 0 } };

  texture_shader_array_object_.bind();
  ArrayBuffer::bind( screen_corners_ );
  ArrayBuffer::load( corners, GL_STATIC_DRAW );

  glCheck( "after resizing" );

  const auto new_window_size = window().window_size();
  if ( new_window_size.first != width or new_window_size.second != height ) {
    throw runtime_error( "failed to resize window to " + to_string( width ) + "x" + to_string( height ) );
  }

  ArrayBuffer::bind( screen_corners_ );
  texture_shader_array_object_.bind();
  texture_shader_program_.use();

  glCheck( "after installing shaders" );
}

void VideoDisplay::draw( Texture420& image )
{
  image.bind();
  repaint();
}

void VideoDisplay::repaint()
{
  const auto window_size = window().window_size();

  if ( window_size.first != width_ or window_size.second != height_ ) {
    width_ = window_size.first;
    height_ = window_size.second;
    resize( width_, height_ );
  }

  glDrawArrays( GL_TRIANGLE_FAN, 0, 4 );

  current_context_window_.window_.swap_buffers();
}