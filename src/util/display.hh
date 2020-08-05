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

#pragma once

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "gl_objects.hh"

class VideoDisplay
{
private:
  static const std::string shader_source_scale_from_pixel_coordinates;
  static const std::string shader_source_ycbcr;

  unsigned int width_, height_;

  struct CurrentContextWindow
  {
    GLFWContext glfw_context_ = {};
    Window window_;

    CurrentContextWindow( const unsigned int width,
                          const unsigned int height,
                          const std::string& title,
                          const bool fullscreen );
  } current_context_window_;

  VertexShader scale_from_pixel_coordinates_ = { shader_source_scale_from_pixel_coordinates };
  FragmentShader ycbcr_shader_ = { shader_source_ycbcr };

  Program texture_shader_program_ = {};

  VertexArrayObject texture_shader_array_object_ = {};
  VertexBufferObject screen_corners_ = {};
  VertexBufferObject other_vertices_ = {};

public:
  VideoDisplay( const unsigned int width, const unsigned int height, const bool fullscreen = false, const bool vr_view = false);

  void draw( Texture420& image );
  void repaint();
  void resize( const unsigned int width, const unsigned int height );

  //void updateHeadOrientation( const float pitch, const float roll, const float yaw);

  Window& window() { return current_context_window_.window_; }
  const Window& window() const { return current_context_window_.window_; }

  void update_head_orientation( const float x, const float y, const float z );

  /* forbid copying */
  VideoDisplay( const VideoDisplay& other ) = delete;
  VideoDisplay& operator=( const VideoDisplay& other ) = delete;
};