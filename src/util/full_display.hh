//#include <glad/gl.h>
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#define GL_GLEXT_PROTOTYPES
#include <GL/glew.h>
#include "matrices.h"
#include "Sphere.h"
#include "Bmp.h"
 
//#include "linmath.h"
 
#include <stdlib.h>
#include <stdio.h>


class FullDisplay
{
private:
    static const float vertices;
    const char* vertex_shader_text;
    const char* fragment_shader_text;
    //void error_callback(int error, const char* description);
    //void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);
    GLuint loadTexture(const char* fileName, bool wrap=true);   
    Matrix4 toPerspective(bool left_eye);

public:

    int test();


};