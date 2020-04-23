AM_CPPFLAGS = $(CXX17_FLAGS) $(GLU_CFLAGS) $(GLFW3_CFLAGS) $(GLEW_CFLAGS) $(PANGOCAIRO_CFLAGS)
AM_CXXFLAGS = $(PICKY_CXXFLAGS)

noinst_LIBRARIES = libgldemoutil.a

libgldemoutil_a_SOURCES = gl_objects.hh gl_objects.cc display.hh display.cc \
	cairo_objects.hh cairo_objects.cc matrices.h matrices.cpp openvr.h vectors.h
