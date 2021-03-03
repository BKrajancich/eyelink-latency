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


/* octave> 255 * inv([219*[.7152 .0722 .2126]'view605734]'
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
      uniform mat4 MVP_L;
      uniform mat4 MVP_R;

      in vec2 Y_texcoord;
      in vec2 uv_texcoord;
      out vec4 outColor;   

      vec2 screen_res = vec2( 1440.0, 1600.0 );
      vec2 tex_res = vec2( 3840.0, 2048.0 );
      float PI = 3.14159265359f;

      float d_width = 2880.0;
      float d_height = 1600.0;
      vec2 COP = vec2( 788.854, 792.558 );
      float scaleX = 1.250902771949768;
      float scaleY = 1.1248387098312378;
      float k1 = -0.19615396996141743;
      float k2 = -0.043787285603857626;
      float k3 = -0.024965852625323637;
      
      vec2 transformPoint( vec2 position_in ) {
        float rX = COP.x - position_in.x;
        float rY = COP.y - position_in.y;

        float retX = COP.x - (rX*scaleX);
        float retY = COP.y - (rY*scaleY);

        vec2 offset = vec2( retX - COP.x, retY - COP.y );
        float r = sqrt( offset.x * offset.y + offset.y * offset.y );

        float maxRadius = d_width / 2;
        //sqrt( d_width/2 * d_width/2 + d_height/2 * d_height/2 );
        
        k1 = k1 * pow( r/maxRadius, 2);
        k2 = k2 * pow( r/maxRadius, 4);
        k3 = k3 * pow( r/maxRadius, 6);

        float k = 1 / (1 + k1 + k2 + k3 );
        float newX = COP.x + (k * offset.x);
        float newY = COP.y + (k * offset.y);

        return vec2 (newX, newY);

      }

      vec2 get_latlong( vec2 texcoord ) {

        vec4 ndc = vec4(2*texcoord.x/screen_res.x - 1.0, 1.0 - 2*texcoord.y/screen_res.y, 1.0, 1.0);
        
        mat4 inv_VP;

        if (Y_texcoord.x < 1440.0) {
          inv_VP = MVP_L;
        } else {
          inv_VP = MVP_R;
        }

        vec3 ray3d = normalize((inv_VP * ndc).xyz);     
        //outColor = vec4(ray3d.xyz, 1.0);
        float theta = asin( ray3d.y );
        //float theta = atan( ray3d.y, length(ray3d.xz) );
        float phi = atan( ray3d.x, ray3d.z );
        return vec2( phi, -theta);
      }
      vec2 reproject_Y (vec2 texcoord ) {
        vec2 phi_theta = get_latlong(texcoord);
        vec2 xy_sphere = vec2( ((phi_theta.x / PI) * tex_res.x + tex_res.x)/2.0, (phi_theta.y + PI/2.0) * tex_res.y /PI );
        return xy_sphere;
      }
      vec2 reproject_uv( vec2 texcoord ) {
        vec2 phi_theta = get_latlong(texcoord);
        vec2 xy_sphere = vec2( ((phi_theta.x / PI) * (tex_res.x/2.0) + (tex_res.x/2.0))/2.0, (phi_theta.y + PI/2.0) * (tex_res.y/2.0) / PI );
        return xy_sphere;
      }
      void main()
      {
        // vec2 texcoord = transformPoint( Y_texcoord );

        // if (texcoord.x < 0.0 || texcoord.y < 0.0 || texcoord.x > screen_res.x || texcoord.y > screen_res.y) {
        //   outColor = vec4 (0.0, 0.0, 0.0, 1.0);
        //   return;
        // }

        vec2 Y_reprojected = reproject_Y(vec2(mod(Y_texcoord.x, screen_res.x), Y_texcoord.y));
        vec2 uv_reprojected = reproject_uv(vec2(mod(Y_texcoord.x, screen_res.x), Y_texcoord.y)) + max( 0.0, min( 0.0, texture(uTex, uv_texcoord).x ) );  // to get rid of inefficiency bug
        float fY = texture(yTex, Y_reprojected).x;
        float fCb = texture(uTex, uv_reprojected).x;
        float fCr = texture(vTex, uv_reprojected).x;
        
        vec4 outColor2 = vec4(
          max(0, min(1.0, 1.16438356164384 * (fY - 0.06274509803921568627) + 1.59567019581339  * (fCr - 0.50196078431372549019))),
          max(0, min(1.0, 1.16438356164384 * (fY - 0.06274509803921568627) - 0.391260370716072 * (fCb - 0.50196078431372549019) - 0.813004933873461 * (fCr - 0.50196078431372549019))),
          max(0, min(1.0, 1.16438356164384 * (fY - 0.06274509803921568627) + 2.01741475897078  * (fCb - 0.50196078431372549019))),
          1.0
        );

        outColor = outColor2;

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

VideoDisplay::VideoDisplay( const unsigned int width, const unsigned int height, const bool fullscreen)
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

  glCheck( "VideoDisplay constructor" );
}

void VideoDisplay::update_MVP( const float MVP_L[16], const float MVP_R[16] )
{
  texture_shader_program_.use();
  glUniformMatrix4fv( texture_shader_program_.uniform_location( "MVP_L" ), 1, GL_FALSE, MVP_L );
  glUniformMatrix4fv( texture_shader_program_.uniform_location( "MVP_R" ), 1, GL_FALSE, MVP_R );
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