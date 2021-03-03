 
#include <chrono>
#include <cstring>
#include <exception>
#include <iostream>
#include <thread>
#include <stdio.h>
#include <string>
#include <sstream>
#include <iomanip>
#include <fstream>

#include <unistd.h>

#include "cairo_objects.hh"
//#include "display.hh"
//#include "full_display.hh"

#include <GL/glut.h>
#include "matrices.h"
#include <openvr.h>
#include "../util/glExtension.h"                // helper for OpenGL extensions
#include "../util/matrices.h"
#include "../util/Bmp.h"
#include "../util/Sphere.h"

using namespace std;
using namespace std::chrono;

#define IMG_DIM_X 3840
#define IMG_DIM_Y 2048

  Matrix4 mat;
  vr::HmdVector3_t rot;

  vr::IVRSystem* m_pHMD;



  // GLUT CALLBACK functions
  void displayCB();
  void reshapeCB(int w, int h);
  void timerCB(int millisec);
  void keyboardCB(unsigned char key, int x, int y);
  void mouseCB(int button, int stat, int x, int y);
  void mouseMotionCB(int x, int y);

  void initGL();
  bool initGLSL();
  int  initGLUT(int argc, char **argv);
  bool initSharedMem();
  void clearSharedMem();
  void setCamera(float posX, float posY, float posZ, float targetX, float targetY, float targetZ);
  void toPerspective(bool left_eye);
  GLuint loadTexture(const char* fileName, bool wrap=true);

  void updateHeadPoseMat();

// blinn shading with texture =============================
const char* vsSource = R"(
// GLSL version
#version 110
// uniforms
uniform mat4 matrixModelView;
uniform mat4 matrixNormal;
uniform mat4 matrixModelViewProjection;
// vertex attribs (input)
attribute vec3 vertexPosition;
attribute vec3 vertexNormal;
attribute vec2 vertexTexCoord;
// varyings (output)
varying vec3 esVertex, esNormal;
varying vec2 texCoord0;
void main()
{
    esVertex = vec3(matrixModelView * vec4(vertexPosition, 1.0));
    esNormal = vec3(matrixNormal * vec4(vertexNormal, 1.0));
    texCoord0 = vertexTexCoord;
    gl_Position = matrixModelViewProjection * vec4(vertexPosition, 1.0);
}
)";

const char* fsSource = R"(
// GLSL version
#version 110
// uniforms
uniform sampler2D map0;                 // texture map #1
uniform bool textureUsed;               // flag for texture
// varyings
varying vec3 esVertex, esNormal;
varying vec2 texCoord0;
void main()
{
    vec3 normal = normalize(esNormal);

    float lon = atan(normal.z, normal.x);
    float lat = acos(normal.y);
    vec2 sphereCoords = vec2(lon, lat) * (1.0 / 3.141592653589793);
    vec2 equiUV = vec2( 1.0 - (sphereCoords.x * 0.5 + 0.5), 1.0 - sphereCoords.y);
    // set frag color
    gl_FragColor = vec4(texture2D(map0, texCoord0));
    //gl_FragColor = vec4(texture2D(map0, equiUV));
}
)";



// global variables
void *font = GLUT_BITMAP_8_BY_13;
bool mouseLeftDown;
bool mouseRightDown;
bool mouseMiddleDown;
float mouseX, mouseY;
float cameraAngleX;
float cameraAngleY;
float cameraDistance;
int drawMode;
bool vboSupported;
GLuint vboId1 = 0, vboId2 = 0;      // IDs of VBO for vertex arrays
GLuint iboId1 = 0, iboId2 = 0;      // IDs of VBO for index array
GLuint texId;
int imageWidth;
int imageHeight;
Matrix4 matrixModelView;
Matrix4 matrixProjection;
// GLSL
GLuint progId = 0;                  // ID of GLSL program
bool glslSupported;
GLint uniformMatrixModelView;
GLint uniformMatrixModelViewProjection;
GLint uniformMatrixNormal;
GLint uniformMap0;
GLint uniformTextureUsed;
GLint attribVertexPosition;
GLint attribVertexNormal;
GLint attribVertexTexCoord;

Matrix4 posMatrix;

// sphere: min sector = 3, min stack = 2
//Sphere sphere1(2.0f, 36, 18, false);    // radius, sectors, stacks, non-smooth (flat) shading
Sphere sphere2(100.0f, 36, 18);

// constants
const int   SCREEN_WIDTH    = 2880;
const int   SCREEN_HEIGHT   = 1600;
const float CAMERA_DISTANCE = 0.0f;

///////////////////////////////////////////////////////////////////////////////
// initialize GLUT for windowing
///////////////////////////////////////////////////////////////////////////////
int initGLUT(int argc, char *argv[])
{
    // GLUT stuff for windowing
    // initialization openGL window.
    // it is called before any other GLUT routine
    glutInit(&argc, argv);

    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH | GLUT_STENCIL);   // display mode

    glutInitWindowSize(1920, 1080);  // window size

    glutInitWindowPosition(100, 100);               // window location

    // finally, create a window with openGL context
    // Window will not displayed until glutMainLoop() is called
    // it returns a unique ID
    int handle = glutCreateWindow("test");     // param is the title of window

    // register GLUT callback functions
    glutDisplayFunc(displayCB);
    glutTimerFunc(33, timerCB, 33);             // redraw only every given millisec
    glutMouseFunc(mouseCB);
    glutMotionFunc(mouseMotionCB);

    return handle;
}

// initialize OpenGL
// disable unused features
///////////////////////////////////////////////////////////////////////////////
void initGL()
{
    glShadeModel(GL_SMOOTH);                    // shading mathod: GL_SMOOTH or GL_FLAT
    glPixelStorei(GL_UNPACK_ALIGNMENT, 4);      // 4-byte pixel alignment

    // enable /disable features
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    //glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_TEXTURE_2D);
    glDisable(GL_CULL_FACE);

    // track material ambient and diffuse from surface color, call it before glEnable(GL_COLOR_MATERIAL)
    //glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    //glEnable(GL_COLOR_MATERIAL);

    glClearColor(0, 0, 0, 0);                   // background color
    glClearStencil(0);                          // clear stencil buffer
    glClearDepth(1.0f);                         // 0 is near, 1 is far
    glDepthFunc(GL_LEQUAL);

    //initLights();
}

///////////////////////////////////////////////////////////////////////////////
// create glsl programs
///////////////////////////////////////////////////////////////////////////////
bool initGLSL()
{
    const int MAX_LENGTH = 2048;
    char log[MAX_LENGTH];
    int logLength = 0;

    // create shader and program
    GLuint vsId = glCreateShader(GL_VERTEX_SHADER);
    GLuint fsId = glCreateShader(GL_FRAGMENT_SHADER);
    progId = glCreateProgram();

    // load shader sources
    glShaderSource(vsId, 1, &vsSource, NULL);
    glShaderSource(fsId, 1, &fsSource, NULL);

    // compile shader sources
    glCompileShader(vsId);
    glCompileShader(fsId);

    //@@ debug
    int vsStatus, fsStatus;
    glGetShaderiv(vsId, GL_COMPILE_STATUS, &vsStatus);
    if(vsStatus == GL_FALSE)
    {
        glGetShaderiv(vsId, GL_INFO_LOG_LENGTH, &logLength);
        glGetShaderInfoLog(vsId, MAX_LENGTH, &logLength, log);
        std::cout << "===== Vertex Shader Log =====\n" << log << std::endl;
    }
    glGetShaderiv(fsId, GL_COMPILE_STATUS, &fsStatus);
    if(fsStatus == GL_FALSE)
    {
        glGetShaderiv(fsId, GL_INFO_LOG_LENGTH, &logLength);
        glGetShaderInfoLog(fsId, MAX_LENGTH, &logLength, log);
        std::cout << "===== Fragment Shader Log =====\n" << log << std::endl;
    }

    // attach shaders to the program
    glAttachShader(progId, vsId);
    glAttachShader(progId, fsId);

    // link program
    glLinkProgram(progId);

    // get uniform/attrib locations
    glUseProgram(progId);
    uniformMatrixModelView           = glGetUniformLocation(progId, "matrixModelView");
    uniformMatrixModelViewProjection = glGetUniformLocation(progId, "matrixModelViewProjection");
    uniformMatrixNormal              = glGetUniformLocation(progId, "matrixNormal");
    uniformMap0                      = glGetUniformLocation(progId, "map0");
    uniformTextureUsed               = glGetUniformLocation(progId, "textureUsed");
    attribVertexPosition = glGetAttribLocation(progId, "vertexPosition");
    attribVertexNormal   = glGetAttribLocation(progId, "vertexNormal");
    attribVertexTexCoord = glGetAttribLocation(progId, "vertexTexCoord");

    // set uniform values
    glUniform1i(uniformMap0, 0);
    glUniform1i(uniformTextureUsed, 1);

    // unbind GLSL
    glUseProgram(0);

    // check GLSL status
    int linkStatus;
    glGetProgramiv(progId, GL_LINK_STATUS, &linkStatus);
    if(linkStatus == GL_FALSE)
    {
        glGetProgramiv(progId, GL_INFO_LOG_LENGTH, &logLength);
        glGetProgramInfoLog(progId, MAX_LENGTH, &logLength, log);
        std::cout << "===== GLSL Program Log =====\n" << log << std::endl;
        return false;
    }
    else
    {
        return true;
    }
}

///////////////////////////////////////////////////////////////////////////////
// load raw image as a texture
///////////////////////////////////////////////////////////////////////////////
GLuint loadTexture(const char* fileName, bool wrap)
{
    Image::Bmp bmp;
    if(!bmp.read(fileName))
        return 0;     // exit if failed load image

    // get bmp info
    int width = bmp.getWidth();
    int height = bmp.getHeight();
    const unsigned char* data = bmp.getDataRGB();
    GLenum type = GL_UNSIGNED_BYTE;    // only allow BMP with 8-bit per channel

    // We assume the image is 8-bit, 24-bit or 32-bit BMP
    GLenum format;
    int bpp = bmp.getBitCount();
    if(bpp == 8)
        format = GL_LUMINANCE;
    else if(bpp == 24)
        format = GL_RGB;
    else if(bpp == 32)
        format = GL_RGBA;
    else
        return 0;               // NOT supported, exit

    // gen texture ID
    GLuint texture;
    glGenTextures(1, &texture);

    // set active texture and configure it
    glBindTexture(GL_TEXTURE_2D, texture);

    // select modulate to mix texture with color for shading
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);

    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    //glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP, GL_TRUE);

    // if wrap is true, the texture wraps over at the edges (repeat)
    //       ... false, the texture ends at the edges (clamp)
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, wrap ? GL_REPEAT : GL_CLAMP);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, wrap ? GL_REPEAT : GL_CLAMP);
    //glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    //glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    // copy texture data
    glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, type, data);
    //glGenerateMipmap(GL_TEXTURE_2D);

    // build our texture mipmaps
    switch(bpp)
    {
    case 8:
        gluBuild2DMipmaps(GL_TEXTURE_2D, 1, width, height, GL_LUMINANCE, type, data);
        break;
    case 24:
        gluBuild2DMipmaps(GL_TEXTURE_2D, 3, width, height, GL_RGB, type, data);
        break;
    case 32:
        gluBuild2DMipmaps(GL_TEXTURE_2D, 4, width, height, GL_RGBA, type, data);
        break;
    }

    return texture;
}

///////////////////////////////////////////////////////////////////////////////s
// set the projection matrix as perspective
///////////////////////////////////////////////////////////////////////////////
void toPerspective(bool left_eye)
{
    const float N = 0.3f;
    const float F = 100.0f;
    const float DEG2RAD = 3.141592f / 180;
    const float FOV_Y = 106 * DEG2RAD;

    if (left_eye) {
        glViewport(0, 0, (GLsizei)1440, (GLsizei)1600);
    } else {
        glViewport(1440, 0, (GLsizei)1440, (GLsizei)1600);
    }


    //construct perspective projection matrix
    float aspectRatio = (float)(1440) / 1600;
    float tangent = tanf(FOV_Y / 2.0f);     // tangent of half fovY
    float h = N * tangent;                  // half height of near plane
    float w = h * aspectRatio;              // half width of near plane
    matrixProjection.identity();
    // matrixProjection[0]  =  N / w;
    // matrixProjection[5]  =  N / h;
    // matrixProjection[10] = -(F + N) / (F - N);
    // matrixProjection[11] = -1;
    // matrixProjection[14] = -(2 * F * N) / (F - N);
    // matrixProjection[15] =  0;

    // std::cout << matrixProjection << std::endl;

    // matrixProjection[0]  =  0.0125f;
    // matrixProjection[5]  =  0.0112f;
    // matrixProjection[8]  =  0.0f;
    // matrixProjection[9]  =  0.0f;
    // matrixProjection[10] = -1.0006f;
    // matrixProjection[11] = -1;
    // matrixProjection[14] = -0.60018;
    // matrixProjection[15] =  0;
if (left_eye) {
    matrixProjection[0]  =  0.78181f;
    matrixProjection[5]  =  0.70302f;
    matrixProjection[8]  =  -0.05977f;
    matrixProjection[9]  =  -0.00503f;
    matrixProjection[10] = -1.0006f;
    matrixProjection[11] = -1;
    matrixProjection[14] = -0.60018;
    matrixProjection[15] =  0;
} else {
    matrixProjection[0]  =  0.78045f;
    matrixProjection[5]  =  0.70247f;
    matrixProjection[8]  =  0.05977f;
    matrixProjection[9]  =  -0.00086f;
    matrixProjection[10] = -1.0006f;
    matrixProjection[11] = -1;
    matrixProjection[14] = -0.60018;
    matrixProjection[15] =  0;

}

    // set perspective viewing frustum
    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf(matrixProjection.get());
    //@@ equivalent fixed pipeline
    //glLoadIdentity();
    //gluPerspective(40.0f, (float)(screenWidth)/screenHeight, 0.1f, 100.0f); // FOV, AspectRatio, NearClip, FarClip

    // switch to modelview matrix in order to set scene
    //glMatrixMode(GL_MODELVIEW);
    // glLoadIdentity();
}

void renderEye(bool left_eye, Matrix4 matrixModel, Matrix4 matrixView) {

     // bind GLSL, texture
    glUseProgram(progId);
    glBindTexture(GL_TEXTURE_2D, texId);

    // activate attribs
    glEnableVertexAttribArray(attribVertexPosition);
    glEnableVertexAttribArray(attribVertexNormal);
    glEnableVertexAttribArray(attribVertexTexCoord);

    // bind vbo for smooth sphere (center and right)
    glBindBuffer(GL_ARRAY_BUFFER, vboId2);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, iboId2);

    // set attrib arrays using glVertexAttribPointer()
    int stride = sphere2.getInterleavedStride();
    glVertexAttribPointer(attribVertexPosition, 3, GL_FLOAT, false, stride, 0);
    glVertexAttribPointer(attribVertexNormal, 3, GL_FLOAT, false, stride, (void*)(3 * sizeof(float)));
    glVertexAttribPointer(attribVertexTexCoord, 2, GL_FLOAT, false, stride, (void*)(6 * sizeof(float)));


    toPerspective(left_eye);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    //set matric uniforms for left eye
    matrixModelView = matrixView * matrixModel;
    Matrix4 matrixModelViewProjection = matrixProjection * matrixModelView;
    Matrix4 matrixNormal = matrixModelView;
    matrixNormal.setColumn(3, Vector4(0,0,0,1));
    glUniformMatrix4fv(uniformMatrixModelView, 1, false, matrixModelView.get());
    glUniformMatrix4fv(uniformMatrixModelViewProjection, 1, false, matrixModelViewProjection.get());
    glUniformMatrix4fv(uniformMatrixNormal, 1, false, matrixNormal.get());

    // right sphere is rendered with texture
    glUniform1i(uniformTextureUsed, 1);

    // draw left eye
    glDrawElements(GL_TRIANGLES,            // primitive type
                   sphere2.getIndexCount(), // # of indices
                   GL_UNSIGNED_INT,         // data type
                   (void*)0);               // ptr to indices
    glBindTexture(GL_TEXTURE_2D, 0);

}

Matrix4 ConvertSteamVRMatrixToMatrix4( const vr::HmdMatrix34_t& matPose )
{
  Matrix4 matrixObj( matPose.m[0][0],
                     matPose.m[1][0],
                     matPose.m[2][0],
                     0.0,
                     matPose.m[0][1],
                     matPose.m[1][1],
                     matPose.m[2][1],
                     0.0,
                     matPose.m[0][2],
                     matPose.m[1][2],
                     matPose.m[2][2],
                     0.0,
                     matPose.m[0][3],
                     matPose.m[1][3],
                     matPose.m[2][3],
                     1.0f );
  return matrixObj;
}

vr::HmdVector3_t GetRotation( vr::HmdMatrix34_t matrix )
{
  vr::HmdQuaternion_t q;

  q.w = sqrt( fmax( 0, 1 + matrix.m[0][0] + matrix.m[1][1] + matrix.m[2][2] ) ) / 2;
  q.x = sqrt( fmax( 0, 1 + matrix.m[0][0] - matrix.m[1][1] - matrix.m[2][2] ) ) / 2;
  q.y = sqrt( fmax( 0, 1 - matrix.m[0][0] + matrix.m[1][1] - matrix.m[2][2] ) ) / 2;
  q.z = sqrt( fmax( 0, 1 - matrix.m[0][0] - matrix.m[1][1] + matrix.m[2][2] ) ) / 2;
  q.x = copysign( q.x, matrix.m[2][1] - matrix.m[1][2] );
  q.y = copysign( q.y, matrix.m[0][2] - matrix.m[2][0] );
  q.z = copysign( q.z, matrix.m[1][0] - matrix.m[0][1] );

  vr::HmdVector3_t angles; // defined as [roll, pitch, yaw]

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.v[0] = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        angles.v[1] = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.v[1] = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.v[2] = std::atan2(siny_cosp, cosy_cosp);
  
  return angles;
}

void updateHeadPoseMat() {

  std::cout << "what about here" << std::endl;

    // Loading the SteamVR Runtime
  vr::EVRInitError eError = vr::VRInitError_None;
  m_pHMD = vr::VR_Init( &eError, vr::VRApplication_Scene );

  if ( eError != vr::VRInitError_None ) {
    m_pHMD = NULL;
    char buf[1024];
    // TODO: this is only since C11
    // sprintf_s( buf, sizeof( buf ), "Unable to init VR runtime: %s", vr::VR_GetVRInitErrorAsEnglishDescription( eError
    // ) );
    // SDL_ShowSimpleMessageBox( SDL_MESSAGEBOX_ERROR, "VR_Init Failed", buf, NULL );
    return;
  }

  vr::TrackedDevicePose_t m_rTrackedDevicePose[vr::k_unMaxTrackedDeviceCount];

  if ( !m_pHMD )
    return;

  vr::VRCompositor()->WaitGetPoses( m_rTrackedDevicePose, vr::k_unMaxTrackedDeviceCount, NULL, 0 );

  std::cout << "how about here" << std::endl;

  for ( int nDevice = 0; nDevice < vr::k_unMaxTrackedDeviceCount; ++nDevice ) {
    if ( m_rTrackedDevicePose[nDevice].bPoseIsValid ) {

      if ( m_pHMD->GetTrackedDeviceClass( nDevice ) == vr::TrackedDeviceClass_HMD ) {
        // printDevicePositionalData( "HMD",
        //                            m_rDevClassChar[nDevice],
        //                            m_rTrackedDevicePose[nDevice].mDeviceToAbsoluteTracking,
        //                            position,
        //                            quaternion );
        rot = GetRotation( m_rTrackedDevicePose[nDevice].mDeviceToAbsoluteTracking );
        mat = ConvertSteamVRMatrixToMatrix4( m_rTrackedDevicePose[nDevice].mDeviceToAbsoluteTracking );
        std::cout << "made it to func" << mat << std::endl;
        //return mat;
        //head_position = position;                           
      }
    }
  
  }
  //return Matrix4(); 
}

void displayCB()
{
    if(!vboSupported || !glslSupported)
        return;

    // clear buffer
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

    // transform camera (view)
    Matrix4 matrixView;

    std::cout << rot.v[1] << std::endl;

    Matrix4 matrixModelCommon;
    matrixModelCommon.rotateX(-90);
    matrixModelCommon.rotateY(cameraAngleY);
    matrixModelCommon.rotateX(cameraAngleX);

    // common model matrix
    //UpdateHMDMatrixPose();
    updateHeadPoseMat();
    //Matrix4 matrixModelCommon = mat;
    //matrixModelCommon.rotateX(90);
    //matrixModelCommon.rotateY(0);
    //matrixModelCommon.rotateZ(-90);
    // matrixModelCommon.rotateX(cameraAngleX);

    std::cout << "in funct   " << matrixModelCommon << std::endl;

    Matrix4 matrixModelL(matrixModelCommon);    // left
    Matrix4 matrixModelR(matrixModelCommon);    // right
    matrixModelL.translate(-0.032f, 0, 0);        // shift left
    matrixModelR.translate(0.032f, 0, 0);         // shift right

    renderEye(true, matrixModelL, matrixView);
    renderEye(false, matrixModelR, matrixView);

    glDisableVertexAttribArray(attribVertexPosition);
    glDisableVertexAttribArray(attribVertexNormal);
    glDisableVertexAttribArray(attribVertexTexCoord);

    // unbind
    glBindTexture(GL_TEXTURE_2D, 0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    glUseProgram(0);

    //showInfo();     // print max range of glDrawRangeElements

    glutSwapBuffers();
}

void timerCB(int millisec)
{
    glutTimerFunc(millisec, timerCB, millisec);
    glutPostRedisplay();
}

void mouseCB(int button, int state, int x, int y)
{
    mouseX = x;
    mouseY = y;

    if(button == GLUT_LEFT_BUTTON)
    {
        if(state == GLUT_DOWN)
        {
            mouseLeftDown = true;
        }
        else if(state == GLUT_UP)
            mouseLeftDown = false;
    }

    else if(button == GLUT_RIGHT_BUTTON)
    {
        if(state == GLUT_DOWN)
        {
            mouseRightDown = true;
        }
        else if(state == GLUT_UP)
            mouseRightDown = false;
    }

    else if(button == GLUT_MIDDLE_BUTTON)
    {
        if(state == GLUT_DOWN)
        {
            mouseMiddleDown = true;
        }
        else if(state == GLUT_UP)
            mouseMiddleDown = false;
    }
}


void mouseMotionCB(int x, int y)
{
    if(mouseLeftDown)
    {
        cameraAngleY += (x - mouseX);
        cameraAngleX += (y - mouseY);
        mouseX = x;
        mouseY = y;
    }
    if(mouseRightDown)
    {
        cameraDistance -= (y - mouseY) * 0.2f;
        mouseY = y;
    }
}

//-----------------------------------------------------------------------------
// Purpose:
//------------------------------------------------------------------------------
class CMainApplication
{
public:
  CMainApplication( int argc, char* argv[] );
  virtual ~CMainApplication();

  bool BInit();

  void Shutdown();

  void RunMainLoop();

  Matrix4 GetHMDMatrixProjectionEye( vr::Hmd_Eye nEye );
  //Matrix4 GetHMDMatrixPoseEye( vr::Hmd_Eye nEye );
  Matrix4 GetCurrentViewProjectionMatrix( vr::Hmd_Eye nEye );
  void UpdateHMDMatrixPose();
  Matrix4 GetTranslationMatrix(const vr::HmdVector3_t& head_position);
  Matrix4 GetRotationMatrix(const vr::HmdQuaternion_t& q);
  Matrix4 eul2rotm4( float rotX, float rotY, float rotZ );
  Matrix4 getViewMat( Matrix4 eyeMat , Matrix4 posMat );

  void printDevicePositionalData( const char* deviceName,
                                  const char devClass,
                                  vr::HmdMatrix34_t posMatrix,
                                  vr::HmdVector3_t position,
                                  vr::HmdQuaternion_t quaternion );
  vr::HmdVector3_t GetPosition( vr::HmdMatrix34_t matrix );
  vr::HmdQuaternion_t GetRotation( vr::HmdMatrix34_t matrix );

  Matrix4 ConvertSteamVRMatrixToMatrix4( const vr::HmdMatrix34_t& matPose );

  Matrix4 m_mat4HMDPose;

private:
  vr::IVRSystem* m_pHMD;
  std::string m_strDriver;
  std::string m_strDisplay;
  vr::TrackedDevicePose_t m_rTrackedDevicePose[vr::k_unMaxTrackedDeviceCount];
  Matrix4 m_rmat4DevicePose[vr::k_unMaxTrackedDeviceCount];

  struct ControllerInfo_t
  {
    vr::VRInputValueHandle_t m_source = vr::k_ulInvalidInputValueHandle;
    vr::VRActionHandle_t m_actionPose = vr::k_ulInvalidActionHandle;
    vr::VRActionHandle_t m_actionHaptic = vr::k_ulInvalidActionHandle;
    Matrix4 m_rmat4Pose;
    std::string m_sRenderModelName;
    bool m_bShowController;
  };

  enum EHand
  {
    Left = 0,
    Right = 1,
  };
  ControllerInfo_t m_rHand[2];


private: // OpenGL bookkeeping
  int m_iTrackedControllerCount;
  int m_iTrackedControllerCount_Last;
  int m_iValidPoseCount;
  int m_iValidPoseCount_Last;
  bool m_bShowCubes;
  Vector2 m_vAnalogValue;

  std::string m_strPoseClasses;                        // what classes we saw poses for this frame
  char m_rDevClassChar[vr::k_unMaxTrackedDeviceCount]; // for each device, a character representing its class

  int m_iSceneVolumeWidth;
  int m_iSceneVolumeHeight;
  int m_iSceneVolumeDepth;
  float m_fScaleSpacing;
  float m_fScale;

  int m_iSceneVolumeInit; // if you want something other than the default 20x20x20

  float m_fNearClip;
  float m_fFarClip;

  
  Matrix4 m_mat4eyePosLeft;
  Matrix4 m_mat4eyePosRight;

  Matrix4 m_mat4ProjectionCenter;
  Matrix4 m_mat4ProjectionLeft;
  Matrix4 m_mat4ProjectionRight;
};

//---------------------------------------------------------------------------------------------------------------------
// Purpose: Returns true if the action is active and had a rising edge
//---------------------------------------------------------------------------------------------------------------------
bool GetDigitalActionRisingEdge( vr::VRActionHandle_t action, vr::VRInputValueHandle_t* pDevicePath = nullptr )
{
  vr::InputDigitalActionData_t actionData;
  vr::VRInput()->GetDigitalActionData( action, &actionData, sizeof( actionData ), vr::k_ulInvalidInputValueHandle );
  if ( pDevicePath ) {
    *pDevicePath = vr::k_ulInvalidInputValueHandle;
    if ( actionData.bActive ) {
      vr::InputOriginInfo_t originInfo;
      if ( vr::VRInputError_None ==
           vr::VRInput()->GetOriginTrackedDeviceInfo( actionData.activeOrigin, &originInfo, sizeof( originInfo ) ) ) {
        *pDevicePath = originInfo.devicePath;
      }
    }
  }
  return actionData.bActive && actionData.bChanged && actionData.bState;
}

//---------------------------------------------------------------------------------------------------------------------
// Purpose: Returns true if the action is active and had a falling edge
//---------------------------------------------------------------------------------------------------------------------
bool GetDigitalActionFallingEdge( vr::VRActionHandle_t action, vr::VRInputValueHandle_t* pDevicePath = nullptr )
{
  vr::InputDigitalActionData_t actionData;
  vr::VRInput()->GetDigitalActionData( action, &actionData, sizeof( actionData ), vr::k_ulInvalidInputValueHandle );
  if ( pDevicePath ) {
    *pDevicePath = vr::k_ulInvalidInputValueHandle;
    if ( actionData.bActive ) {
      vr::InputOriginInfo_t originInfo;
      if ( vr::VRInputError_None ==
           vr::VRInput()->GetOriginTrackedDeviceInfo( actionData.activeOrigin, &originInfo, sizeof( originInfo ) ) ) {
        *pDevicePath = originInfo.devicePath;
      }
    }
  }
  return actionData.bActive && actionData.bChanged && !actionData.bState;
}

//---------------------------------------------------------------------------------------------------------------------
// Purpose: Returns true if the action is active and its state is true
//---------------------------------------------------------------------------------------------------------------------
bool GetDigitalActionState( vr::VRActionHandle_t action, vr::VRInputValueHandle_t* pDevicePath = nullptr )
{
  vr::InputDigitalActionData_t actionData;
  vr::VRInput()->GetDigitalActionData( action, &actionData, sizeof( actionData ), vr::k_ulInvalidInputValueHandle );
  if ( pDevicePath ) {
    *pDevicePath = vr::k_ulInvalidInputValueHandle;
    if ( actionData.bActive ) {
      vr::InputOriginInfo_t originInfo;
      if ( vr::VRInputError_None ==
           vr::VRInput()->GetOriginTrackedDeviceInfo( actionData.activeOrigin, &originInfo, sizeof( originInfo ) ) ) {
        *pDevicePath = originInfo.devicePath;
      }
    }
  }
  return actionData.bActive && actionData.bState;
}

//-----------------------------------------------------------------------------
// Purpose: Constructor
//-----------------------------------------------------------------------------
CMainApplication::CMainApplication( int argc, char* argv[] )
  : m_pHMD( NULL )
  , m_iValidPoseCount( 0 )
  , m_iValidPoseCount_Last( -1 )
  , m_iSceneVolumeInit( 20 )
  , m_strPoseClasses( "" ) {};

//-----------------------------------------------------------------------------
// Purpose: Destructor
//-----------------------------------------------------------------------------
CMainApplication::~CMainApplication()
{
  // work is done in Shutdown
  // dprintf( "Shutdown" );
}

//-----------------------------------------------------------------------------
// Purpose: Helper to get a string from a tracked device property and turn it
//			into a std::string
//-----------------------------------------------------------------------------
std::string GetTrackedDeviceString( vr::TrackedDeviceIndex_t unDevice,
                                    vr::TrackedDeviceProperty prop,
                                    vr::TrackedPropertyError* peError = NULL )
{
  uint32_t unRequiredBufferLen = vr::VRSystem()->GetStringTrackedDeviceProperty( unDevice, prop, NULL, 0, peError );
  if ( unRequiredBufferLen == 0 )
    return "";

  char* pchBuffer = new char[unRequiredBufferLen];
  unRequiredBufferLen =
    vr::VRSystem()->GetStringTrackedDeviceProperty( unDevice, prop, pchBuffer, unRequiredBufferLen, peError );
  std::string sResult = pchBuffer;
  delete[] pchBuffer;
  return sResult;
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
bool CMainApplication::BInit()
{

  // Loading the SteamVR Runtime
  vr::EVRInitError eError = vr::VRInitError_None;
  m_pHMD = vr::VR_Init( &eError, vr::VRApplication_Scene );

  if ( eError != vr::VRInitError_None ) {
    m_pHMD = NULL;
    char buf[1024];
    // TODO: this is only since C11
    // sprintf_s( buf, sizeof( buf ), "Unable to init VR runtime: %s", vr::VR_GetVRInitErrorAsEnglishDescription( eError
    // ) );
    // SDL_ShowSimpleMessageBox( SDL_MESSAGEBOX_ERROR, "VR_Init Failed", buf, NULL );
    return false;
  }

  m_strDriver = "No Driver";
  m_strDisplay = "No Display";


  return true;
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void CMainApplication::Shutdown()
{
  if ( m_pHMD ) {
    vr::VR_Shutdown();
    m_pHMD = NULL;
  }
}

//-----------------------------------------------------------------------------
// Purpose: Gets a Matrix Projection Eye with respect to nEye.
//-----------------------------------------------------------------------------
Matrix4 CMainApplication::GetHMDMatrixProjectionEye( vr::Hmd_Eye nEye )
{
  if ( !m_pHMD )
    return Matrix4();

  vr::HmdMatrix44_t mat = m_pHMD->GetProjectionMatrix( nEye, m_fNearClip, m_fFarClip );

  if (nEye == vr::Eye_Left) {
    return Matrix4( 0.78113, 0.0, -0.05968, 0.0,
                     0.0, 0.70275, -0.00295, 0.0,
                      0.0, 0.0, -1.006, -0.60018,
                      0.0, 0.0, -1.0, 0.0).transpose();
  } else {
    return Matrix4( 0.78113, 0.0, 0.05968, 0.0,
                     0.0, 0.70275, -0.00295, 0.0,
                      0.0, 0.0, -1.006, -0.60018,
                      0.0, 0.0, -1.0, 0.0).transpose();
  }

  //   if (nEye == vr::Eye_Left) {
  //   return Matrix4( 0.0125, 0.0, 0, 0.0,
  //                    0.0, 0.70275, 0, 0.0,
  //                     0.0, 0.0, -1.006, -0.60018,
  //                     0.0, 0.0, -1.0, 0.0).transpose();
  // } else {
  //   return Matrix4( 0.0125, 0.0, 0, 0.0,
  //                    0.0, 0.70275, 0.0, 0.0,
  //                     0.0, 0.0, -1.006, -0.60018,
  //                     0.0, 0.0, -1.0, 0.0).transpose();
  // }

  // return Matrix4( mat.m[0][0],
  //                 mat.m[1][0],
  //                 mat.m[2][0],
  //                 mat.m[3][0],
  //                 mat.m[0][1],
  //                 mat.m[1][1],
  //                 mat.m[2][1],
  //                 mat.m[3][1],
  //                 mat.m[0][2],
  //                 mat.m[1][2],
  //                 mat.m[2][2],
  //                 mat.m[3][2],
  //                 mat.m[0][3],
  //                 mat.m[1][3],
  //                 mat.m[2][3],
  //                 mat.m[3][3] );
}

//-----------------------------------------------------------------------------
// Purpose: Gets an HMDMatrixPoseEye with respect to nEye.
//-----------------------------------------------------------------------------
// Matrix4 CMainApplication::GetHMDMatrixPoseEye( vr::Hmd_Eye nEye )
// {
//   if ( !m_pHMD )
//     return Matrix4();

//   vr::HmdMatrix34_t matEyeRight = m_pHMD->GetEyeToHeadTransform( nEye );
//   Matrix4 matrixObj( matEyeRight.m[0][0],
//                      matEyeRight.m[1][0],
//                      matEyeRight.m[2][0],
//                      0.0,vr::TrackedDevicePose_t m_rTrackedDevicePose[vr::k_unMaxTrackedDeviceCount];
//     vr::VRCompositor()->WaitGetPoses( m_rTrackedDevicePose, vr::k_unMaxTrackedDeviceCount, NULL, 0 );
//                      matEyeRight.m[2][1],
//                      0.0,
//                      matEyeRight.m[0][2],
//                      matEyeRight.m[1][2],
//                      matEyeRight.m[2][2],
//                      0.0,
//                      matEyeRight.m[0][3],
//                      matEyeRight.m[1][3],
//                      matEyeRight.m[2][3],
//                      1.0f );

//   return matrixObj.invert();
// }

//-----------------------------------------------------------------------------
// Purpose: Gets a Current View Projection Matrix with respect to nEye,
//          which may be an Eye_Left or an Eye_Right.
//-----------------------------------------------------------------------------
Matrix4 CMainApplication::GetCurrentViewProjectionMatrix( vr::Hmd_Eye nEye )
{
  Matrix4 matMVP;
  if ( nEye == vr::Eye_Left ) {
    matMVP = m_mat4ProjectionLeft * m_mat4eyePosLeft * m_mat4HMDPose;

  } else if ( nEye == vr::Eye_Right ) {
    matMVP = m_mat4ProjectionRight * m_mat4eyePosRight * m_mat4HMDPose;
  }

  return matMVP;
}

vr::HmdQuaternion_t CMainApplication::GetRotation( vr::HmdMatrix34_t matrix )
{
  vr::HmdQuaternion_t q;

  q.w = sqrt( fmax( 0, 1 + matrix.m[0][0] + matrix.m[1][1] + matrix.m[2][2] ) ) / 2;
  q.x = sqrt( fmax( 0, 1 + matrix.m[0][0] - matrix.m[1][1] - matrix.m[2][2] ) ) / 2;
  q.y = sqrt( fmax( 0, 1 - matrix.m[0][0] + matrix.m[1][1] - matrix.m[2][2] ) ) / 2;
  q.z = sqrt( fmax( 0, 1 - matrix.m[0][0] - matrix.m[1][1] + matrix.m[2][2] ) ) / 2;
  q.x = copysign( q.x, matrix.m[2][1] - matrix.m[1][2] );
  q.y = copysign( q.y, matrix.m[0][2] - matrix.m[2][0] );
  q.z = copysign( q.z, matrix.m[1][0] - matrix.m[0][1] );
  return q;
}

vr::HmdVector3_t CMainApplication::GetPosition( vr::HmdMatrix34_t matrix )
{
  vr::HmdVector3_t vector;

  vector.v[0] = matrix.m[0][3];
  vector.v[1] = matrix.m[1][3];
  vector.v[2] = -matrix.m[2][3];

  return vector;
}

void CMainApplication::printDevicePositionalData( const char* deviceName,
                                                  const char devClass,
                                                  vr::HmdMatrix34_t posMatrix,
                                                  vr::HmdVector3_t position,
                                                  vr::HmdQuaternion_t quaternion )
{
  // Print position and quaternion (rotation).
  fprintf( stderr,
           "%s (%c), x = %.5f, y = %.5f, z = %.5f, qw = %.5f, qx = %.5f, qy = %.5f, qz = %.5f\n",
           deviceName,
           devClass,
           position.v[0],
           position.v[1],
           position.v[2],
           quaternion.w,
           quaternion.x,
           quaternion.y,
           quaternion.z );

  // Uncomment this if you want to print entire transform matrix that contains both position and rotation matrix.
  // dprintf("\n%lld,%s,%.5f,%.5f,%.5f,x: %.5f,%.5f,%.5f,%.5f,y: %.5f,%.5f,%.5f,%.5f,z: %.5f,qw: %.51000.0*
  //    posMatrix.m[2][0], posMatrix.m[2][1], posMatrix.m[2][2], posMatrix.m[2][3],
  //    quaternion.w, quaternion.x, quaternion.y, quaternion.z);
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void CMainApplication::UpdateHMDMatrixPose()
{
  if ( !m_pHMD )
    return;

  vr::VRCompositor()->WaitGetPoses( m_rTrackedDevicePose, vr::k_unMaxTrackedDeviceCount, NULL, 0 );

  m_iValidPoseCount = 0;
  m_strPoseClasses = "";
  for ( int nDevice = 0; nDevice < vr::k_unMaxTrackedDeviceCount; ++nDevice ) {
    if ( m_rTrackedDevicePose[nDevice].bPoseIsValid ) {
      m_iValidPoseCount++;
      m_rmat4DevicePose[nDevice] =
        ConvertSteamVRMatrixToMatrix4( m_rTrackedDevicePose[nDevice].mDeviceToAbsoluteTracking );
      vr::HmdVector3_t position = GetPosition( m_rTrackedDevicePose[nDevice].mDeviceToAbsoluteTracking );
      vr::HmdQuaternion_t quaternion = GetRotation( m_rTrackedDevicePose[nDevice].mDeviceToAbsoluteTracking );
      if ( m_rDevClassChar[nDevice] == 0 ) {
        switch ( m_pHMD->GetTrackedDeviceClass( nDevice ) ) {
          case vr::TrackedDeviceClass_Controller:
            m_rDevClassChar[nDevice] = 'C';
            break;
          case vr::TrackedDeviceClass_HMD:
            m_rDevClassChar[nDevice] = 'H';
            break;
          case vr::TrackedDeviceClass_Invalid:
            m_rDevClassChar[nDevice] = 'I';
            break;
          case vr::TrackedDeviceClass_GenericTracker:
            m_rDevClassChar[nDevice] = 'G';
            break;
          case vr::TrackedDeviceClass_TrackingReference:
            m_rDevClassChar[nDevice] = 'T';
            break;
          default:
            m_rDevClassChar[nDevice] = '?';
            break;
        }
      }
      if ( m_pHMD->GetTrackedDeviceClass( nDevice ) == vr::TrackedDeviceClass_HMD ) {
        printDevicePositionalData( "HMD",
                                   m_rDevClassChar[nDevice],
                                   m_rTrackedDevicePose[nDevice].mDeviceToAbsoluteTracking,
                                   position,
                                   quaternion );
        //mat = ConvertSteamVRMatrixToMatrix4( m_rTrackedDevicePose[nDevice].mDeviceToAbsoluteTracking );
        //head_position = position;                           
      }

      m_strPoseClasses += m_rDevClassChar[nDevice];
    }
  }

  if ( m_rTrackedDevicePose[vr::k_unTrackedDeviceIndex_Hmd].bPoseIsValid ) {
    m_mat4HMDPose = m_rmat4DevicePose[vr::k_unTrackedDeviceIndex_Hmd];
    //m_mat4HMDPose.invert();
  }
}

vr::HmdVector3_t QuaternionToEulerAngles(vr::HmdQuaternion_t& q) {

    vr::HmdVector3_t angles; // defined as [roll, pitch, yaw]

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.v[0] = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        angles.v[1] = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.v[1] = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.v[2] = std::atan2(siny_cosp, cosy_cosp);
  

    return angles;
}

//-----------------------------------------------------------------------------
// Purpose: Converts a SteamVR matrix to our local matrix class
//-----------------------------------------------------------------------------
Matrix4 CMainApplication::ConvertSteamVRMatrixToMatrix4( const vr::HmdMatrix34_t& matPose )
{
  Matrix4 matrixObj( matPose.m[0][0],
                     matPose.m[1][0],
                     matPose.m[2][0],
                     0.0,
                     matPose.m[0][1],
                     matPose.m[1][1],
                     matPose.m[2][1],
                     0.0,
                     matPose.m[0][2],
                     matPose.m[1][2],
                     matPose.m[2][2],
                     0.0,
                     matPose.m[0][3],
                     matPose.m[1][3],
                     matPose.m[2][3],
                     1.0f );
  return matrixObj;
}

Matrix4 CMainApplication::GetTranslationMatrix(const vr::HmdVector3_t& head_position) {
  Matrix4 matrixObj( 1.0, 0.0, 0.0, 0.0,
                      0.0, 1.0, 0.0, 0.0,
                      0.0, 0.0, 1.0, 0.0,
                      head_position.v[0], head_position.v[1], head_position.v[2], 1.0);
 //std::cout << "translate: " << matrixObj.getDeterminant() << std::endl;
  return matrixObj;
}

Matrix4 CMainApplication::GetRotationMatrix(const vr::HmdQuaternion_t& q) {
  Matrix4 matrixObj( q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z, 2*q.x*q.y - 2*q.w*q.z, 2*q.x*q.z + 2*q.w*q.y, 0.0,
                      2*q.w*q.y + 2*q.w*q.z, q.w*q.w - q.x*q.x+ q.y*q.y - q.z*q.z, 2*q.y*q.z - 2*q.w*q.x, 0.0,
                      2*q.x*q.z - 2*q.w*q.y, 2*q.y*q.z + 2*q.w*q.x, q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z, 0.0,
                      0.0, 0.0, 0.0, 1.0);
// std::cout << "translate: " << matrixObj.getDeterminant() << std::endl;
  return matrixObj;
}

Matrix4 CMainApplication::eul2rotm4( float rotX, float rotY, float rotZ ) {
  Matrix4 R_x = Matrix4(	1.0,		0.0,			0.0,  0.0,
        0.0, 	cos(rotX),	sin(rotX), 0.0,
        0.0, 	-sin(rotX),	 cos(rotX), 0.0,
        0.0, 0.0, 0.0, 1.0f);
  Matrix4 R_y = Matrix4( cos(rotY),	0.0, -sin(rotY), 0.0,
          0.0, 	1.0f,	0.0, 0.0,                                              
          sin(rotY), 	0.0,	 cos(rotY), 0.0,
          0.0, 0.0, 0.0, 1.0f);
  Matrix4 R_z = Matrix4( cos(rotZ),	sin(rotZ), 0.0, 0.0,
          -sin(rotZ),	 cos(rotZ),	0.0, 0.0,
          0.0, 	0.0,	 1.0f, 0.0,
          0.0, 0.0, 0.0, 1.0f);                             
  
  return R_z * R_y * R_x;        
}  

Matrix4 CMainApplication::getViewMat( Matrix4 eyeMat , Matrix4 posMat ) {
  Matrix4 eyePos = eyeMat * posMat;
  Matrix4 rot = eyePos;
  rot[12] = 0;
  rot[13] = 0;
  rot[14] = 0;
  Vector3 look_position = Vector3( eyePos[12], eyePos[13], eyePos[14] );
  Matrix4 worldMatrix = Matrix4( 1.0, 0.0, 0.0, 0.0,
                      0.0, 1.0, 0.0, 0.0,
                      0.0, 0.0, 1.0, 0.0,
                      eyePos[12], eyePos[13], eyePos[14], 1.0);
  Vector4 look_up_4 = rot * Vector4(0, 1.0, 0, 0);
  Vector4 look_at_4 = rot * Vector4(0, 0.0, 1.0, 0);

  Vector3 look_up = Vector3 (look_up_4[0], look_up_4[1], look_up_4[2]);
  Vector3 look_at = Vector3 (look_at_4[0], look_at_4[1], look_at_4[2]);
  Vector3 z = (look_at).normalize();
  Vector3 x = (look_up.cross(z)).normalize();
  Vector3 y = z.cross(x);
  Matrix4 viewMat = Matrix4(x[0], y[0], z[0], 0.0,
                            x[1], y[1], z[1], 0.0,
                            x[2], y[2], z[2], 0.0,
                            0.0, 0.0, 0.0, 1.0);

  //std::cout << rot[3] << std::endl;

  return worldMatrix.invert() * viewMat.invert();

  
  //std::cout << "look up: {" << look_up[0] << ", " << look_up[1] << ", " << look_up[2] << ", " << look_up[3] << "}" << std::endl;
}  

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void CMainApplication::RunMainLoop()
{
  uint SCREEN_RES_X = 2880;
  uint SCREEN_RES_Y = 1600;

  bool bQuit = false;
  //FullDisplay display { }; // fullscreen window @ 1920x1080 luma resolution
  // Raster420 yuv_raster { IMG_DIM_X, IMG_DIM_Y };
  // writePNGRaster(yuv_raster);
  // Texture420 texture { yuv_raster };

  vr::HmdQuaternion_t head_quaternion;
  vr::HmdVector3_t head_position;

  Matrix4 m_mat4ProjectionLeft = GetHMDMatrixProjectionEye( vr::Eye_Left );
	Matrix4 m_mat4ProjectionRight = GetHMDMatrixProjectionEye( vr::Eye_Right );

  //Matrix4 m_mat4eyePosLeft = GetHMDMatrixPoseEye( vr::Eye_Left );
	//Matrix4 m_mat4eyePosRight = GetHMDMatrixPoseEye( vr::Eye_Right );

    Matrix4 matrixView;
    Matrix4 matrixProjection = GetHMDMatrixProjectionEye( vr::Eye_Left ) ;

    std::cout << "hello" << std::endl;

    std::cout << "hello" << std::endl;

    // std::cout << "matrixModelView: " << matrixModelView << std::endl;
    // std::cout << "matrixModelViewProjection: " << matrixModelViewProjection << std::endl;
    // std::cout << "matrixNormal: " << matrixNormal << std::endl;
    posMatrix = m_mat4HMDPose;

  while ( !bQuit ) {

    std::cout << m_mat4HMDPose << std::endl;
    posMatrix = m_mat4HMDPose;
    //std::cout << "proj mat: " << GetHMDMatrixProjectionEye( vr::Eye_Left ) << std::endl;
    //std::cout << "view mat: " << GetHMDMatrixPoseEye( vr::Eye_Left ).invert() << std::endl;
    //std::cout << "model mat: " << m_mat4HMDPose << std::endl;

    UpdateHMDMatrixPose();    
    // vr::HmdVector3_t head_orientation = QuaternionToEulerAngles(head_quaternion);
    // Matrix4 rot_mat = eul2rotm4(-head_orientation.v[0], head_orientation.v[1], -head_orientation.v[2]);


    // Matrix4 MVP_L = getViewMat( GetHMDMatrixPoseEye( vr::Eye_Left ) , m_mat4HMDPose ) * GetHMDMatrixProjectionEye( vr::Eye_Left ).invert();
    // Matrix4 MVP_R = getViewMat( GetHMDMatrixPoseEye( vr::Eye_Right ) , m_mat4HMDPose ) * GetHMDMatrixProjectionEye( vr::Eye_Right ).invert();

    // //Matrix4 MVP_L =  m_mat4HMDPose.invert() * GetHMDMatrixPoseEye( vr::Eye_Left ).invert() * GetHMDMatrixProjectionEye( vr::Eye_Left ).invert();
    // //Matrix4 MVP_R =  m_mat4HMDPose.invert()  * GetHMDMatrixPoseEye( vr::Eye_Right ).invert() * GetHMDMatrixProjectionEye( vr::Eye_Right ).invert();

    // std::cout << "view Mat: " << GetHMDMatrixProjectionEye( vr::Eye_Left ) << std::endl;
    // std::cout << "view Mat inv: " << GetHMDMatrixProjectionEye( vr::Eye_Left ).invert() << std::endl;

// transform camera (view)
    // Matrix4 MVP_L = matrixModelView;
    // float m_L[16] = {MVP_L[0], MVP_L[4], MVP_L[8], MVP_L[12], MVP_L[1], MVP_L[5], MVP_L[9], MVP_L[13], MVP_L[2], MVP_L[6], MVP_L[10], MVP_L[14], MVP_L[3], MVP_L[7], MVP_L[11], MVP_L[15]};
    // display.update_MVP(m_L, m_L);
    // display.draw( texture );


  //updateHeadPose(0,0,0);

    
    // //float m_L[16] = {MVP_L[0], MVP_L[4], MVP_L[8], MVP_L[12], MVP_L[1], MVP_L[5], MVP_L[9], MVP_L[13], MVP_L[2], MVP_L[6], MVP_L[10], MVP_L[14], MVP_L[3], MVP_L[7], MVP_L[11], MVP_L[15]};
    // //float m_R[16] = {MVP_R[0], MVP_R[4], MVP_R[8], MVP_R[12], MVP_R[1], MVP_R[5], MVP_R[9], MVP_R[13], MVP_R[2], MVP_R[6], MVP_R[10], MVP_R[14], MVP_R[3], MVP_R[7], MVP_R[11], MVP_R[15]};
    
    // float m_L[16] = {MVP_L[0], MVP_L[1], MVP_L[2], MVP_L[3], MVP_L[4], MVP_L[5], MVP_L[6], MVP_L[7], MVP_L[8], MVP_L[9], MVP_L[10], MVP_L[11], MVP_L[12], MVP_L[13], MVP_L[14], MVP_L[15]};
    // float m_R[16] = {MVP_R[0], MVP_R[1], MVP_R[2], MVP_R[3], MVP_R[4], MVP_R[5], MVP_R[6], MVP_R[7], MVP_R[8], MVP_R[9], MVP_R[10], MVP_R[11], MVP_R[12], MVP_R[13], MVP_R[14], MVP_R[15]};
    // display.update_MVP( m_L, m_R);

    
    //FullDisplay.test();

    //FullDisplay.test("hello there!");

    //std::cout <<"hi" <<std::endl;


  }
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
int main( int argc, char* argv[] )
{
  CMainApplication* pMainApplication = new CMainApplication( argc, argv );

  if ( !pMainApplication->BInit() ) {
    pMainApplication->Shutdown();
    return 1;
  }

  initGLUT(argc, argv);

  initGL();

  glewInit();

    // get OpenGL extensions
    glExtension& ext = glExtension::getInstance();
    vboSupported = ext.isSupported("GL_ARB_vertex_buffer_object");
    if(vboSupported)
    {
        glGenBuffers(1, &vboId2);
        glBindBuffer(GL_ARRAY_BUFFER, vboId2);
        glBufferData(GL_ARRAY_BUFFER, sphere2.getInterleavedVertexSize(), sphere2.getInterleavedVertices(), GL_STATIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glGenBuffers(1, &iboId2);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, iboId2);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sphere2.getIndexSize(), sphere2.getIndices(), GL_STATIC_DRAW);

        // unbind
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
        std::cout << "Video card supports GL_ARB_vertex_buffer_object." << std::endl;
    }
    else
    {
        std::cout << "[WARNING] Video card does NOT support GL_ARB_vertex_buffer_object." << std::endl;
    }

    glslSupported = ext.isSupported("GL_ARB_vertex_program") && ext.isSupported("GL_ARB_fragment_program");
    if(glslSupported)
    {
        std::cout << "Video card supports GLSL." << std::endl;
        // compile shaders and create GLSL program
        // If failed to create GLSL, reset flag to false
        glslSupported = initGLSL();
    }
    else
    {
        std::cout << "[WARNING] Video card does NOT support GLSL." << std::endl;
    }

    //load BMP image
    texId = loadTexture("/home/brooke/repos/eyelink-latency/src/frontend/frame92.bmp", true);

    // the last GLUT call (LOOP)
    // window will be shown and display callback is triggered by events
    // NOTE: this call never return main().
    glutMainLoop(); /* Start GLUT event-processing loop */

  std::cout << "got to main looop" << std::endl;

  pMainApplication->RunMainLoop();

  pMainApplication->Shutdown();

  return 0;
}
