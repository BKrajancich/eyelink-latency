// ///////////////////////////////////////////////////////////////////////////////
// // main.cpp
// // ========
// // drawing a sphere using vertex array (glDrawElements)
// // dependency: freeglut/glut
// //
// //  AUTHOR: Song Ho Ahn (song.ahn@gmail.com)
// // CREATED: 2017-11-02
// // UPDATED: 2018-09-19
// ///////////////////////////////////////////////////////////////////////////////

// #define GL_GLEXT_PROTOTYPES
// #include <GL/glut.h>

 #include <iostream>
// #include <sstream>
// #include <iomanip>
// #include <fstream>
// #include <stdio.h> 
// #include <string.h> 
// #include "glExtension.h"                // helper for OpenGL extensions
// #include "matrices.h"
// #include "Bmp.h"
// #include "Sphere.h"
// //#include "full_display.hh"


// // GLUT CALLBACK functions
// int initDisp();
// void displayCB();
// void reshapeCB(int w, int h);
// void timerCB(int millisec);
// void keyboardCB(unsigned char key, int x, int y);
// void mouseCB(int button, int stat, int x, int y);
// void mouseMotionCB(int x, int y);

// void updateHeadPose(float rot_x, float rot_y, float rot_z);
// void initGL();
// bool initGLSL();
// int  initGLUT();
// bool initSharedMem();
// void clearSharedMem();
// void setCamera(float posX, float posY, float posZ, float targetX, float targetY, float targetZ);
// void toPerspective(bool left_eye);
// GLuint loadTexture(const char* fileName, bool wrap=true);


// // constants
// const int   SCREEN_WIDTH    = 2880;
// const int   SCREEN_HEIGHT   = 1600;
// const float CAMERA_DISTANCE = 0.0f;



// // blinn shading with texture =============================
// const char* vsSource = R"(
// // GLSL version
// #version 110
// // uniforms
// uniform mat4 matrixModelView;
// uniform mat4 matrixNormal;
// uniform mat4 matrixModelViewProjection;
// // vertex attribs (input)
// attribute vec3 vertexPosition;
// attribute vec3 vertexNormal;
// attribute vec2 vertexTexCoord;
// // varyings (output)
// varying vec3 esVertex, esNormal;
// varying vec2 texCoord0;
// void main()
// {
//     esVertex = vec3(matrixModelView * vec4(vertexPosition, 1.0));
//     esNormal = vec3(matrixNormal * vec4(vertexNormal, 1.0));
//     texCoord0 = vertexTexCoord;
//     gl_Position = matrixModelViewProjection * vec4(vertexPosition, 1.0);
// }
// )";

// const char* fsSource = R"(
// // GLSL version
// #version 110
// // uniforms
// uniform sampler2D map0;                 // texture map #1
// uniform bool textureUsed;               // flag for texture
// // varyings
// varying vec3 esVertex, esNormal;
// varying vec2 texCoord0;
// void main()
// {
//     vec3 normal = normalize(esNormal);

//     float lon = atan(normal.z, normal.x);
//     float lat = acos(normal.y);
//     vec2 sphereCoords = vec2(lon, lat) * (1.0 / 3.141592653589793);
//     vec2 equiUV = vec2( 1.0 - (sphereCoords.x * 0.5 + 0.5), 1.0 - sphereCoords.y);
//     // set frag color
//     gl_FragColor = vec4(texture2D(map0, texCoord0));
//     //gl_FragColor = vec4(texture2D(map0, equiUV));
// }
// )";



// // global variables
// void *font = GLUT_BITMAP_8_BY_13;
// int screenWidth;
// int screenHeight;
// bool mouseLeftDown;
// bool mouseRightDown;
// bool mouseMiddleDown;
// float mouseX, mouseY;
// float cameraAngleX;
// float cameraAngleY;
// float cameraDistance;
// int drawMode;
// bool vboSupported;
// GLuint vboId1 = 0, vboId2 = 0;      // IDs of VBO for vertex arrays
// GLuint iboId1 = 0, iboId2 = 0;      // IDs of VBO for index array
// GLuint texId;
// int imageWidth;
// int imageHeight;
// Matrix4 matrixModelView;
// Matrix4 matrixProjection;
// // GLSL
// GLuint progId = 0;                  // ID of GLSL program
// bool glslSupported;
// GLint uniformMatrixModelView;
// GLint uniformMatrixModelViewProjection;
// GLint uniformMatrixNormal;
// GLint uniformMap0;
// GLint uniformTextureUsed;
// GLint attribVertexPosition;
// GLint attribVertexNormal;
// GLint attribVertexTexCoord;

// // sphere: min sector = 3, min stack = 2
// //Sphere sphere1(2.0f, 36, 18, false);    // radius, sectors, stacks, non-smooth (flat) shading

// Sphere sphere2(100.0f, 36, 18);



// ///////////////////////////////////////////////////////////////////////////////
int initDisp()
{
     std::cout << "made it here!" << std::endl;
//     // init global vars
//     //initSharedMem();

//     // init GLUT and GLs
//     // initGLUT();
//     // initGL();

//     // // get OpenGL extensions
//     // glExtension& ext = glExtension::getInstance();
//     // vboSupported = ext.isSupported("GL_ARB_vertex_buffer_object");
//     // if(vboSupported)
//     // {

//     //     glGenBuffers(1, &vboId2);
//     //     glBindBuffer(GL_ARRAY_BUFFER, vboId2);
//     //     glBufferData(GL_ARRAY_BUFFER, sphere2.getInterleavedVertexSize(), sphere2.getInterleavedVertices(), GL_STATIC_DRAW);
//     //     glBindBuffer(GL_ARRAY_BUFFER, 0);
//     //     glGenBuffers(1, &iboId2);
//     //     glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, iboId2);
//     //     glBufferData(GL_ELEMENT_ARRAY_BUFFER, sphere2.getIndexSize(), sphere2.getIndices(), GL_STATIC_DRAW);

//     //     // unbind
//     //     glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
//     //     std::cout << "Video card supports GL_ARB_vertex_buffer_object." << std::endl;
//     // }
//     // else
//     // {
//     //     std::cout << "[WARNING] Video card does NOT support GL_ARB_vertex_buffer_object." << std::endl;
//     // }

//     // glslSupported = ext.isSupported("GL_ARB_vertex_program") && ext.isSupported("GL_ARB_fragment_program");
//     // if(glslSupported)
//     // {
//     //     std::cout << "Video card supports GLSL." << std::endl;
//     //     // compile shaders and create GLSL program
//     //     // If failed to create GLSL, reset flag to false
//     //     glslSupported = initGLSL();
//     // }
//     // else
//     // {
//     //     std::cout << "[WARNING] Video card does NOT support GLSL." << std::endl;
//     // }
    
//     // // load BMP image
//     // texId = loadTexture("/home/brooke/repos/eyelink-latency/src/frontend/frame92.bmp", true);

//     // // the last GLUT call (LOOP)
//     // // window will be shown and display callback is triggered by events
//     // // NOTE: this call never return main().
//     // glutMainLoop(); /* Start GLUT event-processing loop */

     return 0;
}

// void updateHeadPose(float rot_x, float rot_y, float rot_z)
// {
//     cameraAngleX = rot_x;
//     cameraAngleY = rot_y;
// }



// ///////////////////////////////////////////////////////////////////////////////
// // initialize GLUT for windowing
// ///////////////////////////////////////////////////////////////////////////////
// int initGLUT()
// {
//     // GLUT stuff for windowing
//     // initialization openGL window.
//     // it is called before any other GLUT routine
//     char *myargv [1];
//     int myargc=1;
//     char source[] = "GeeksForGeeks";
//     myargv[0]=strdup(source);
//     glutInit(&myargc, myargv);
//     //glutInit(&argc, argv);

//     glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH | GLUT_STENCIL);   // display mode

//     glutInitWindowSize(screenWidth, screenHeight);  // window size

//     glutInitWindowPosition(100, 100);               // window location

//     // finally, create a window with openGL context
//     // Window will not displayed until glutMainLoop() is called
//     // it returns a unique ID
//     int handle = glutCreateWindow("window");     // param is the title of window

//     // register GLUT callback functions
//     glutDisplayFunc(displayCB);
//     glutTimerFunc(33, timerCB, 33);             // redraw only every given millisec
//     //glutReshapeFunc(reshapeCB);
//     //glutKeyboardFunc(keyboardCB);
//     glutMouseFunc(mouseCB);
//     glutMotionFunc(mouseMotionCB);

//     return handle;
// }



// ///////////////////////////////////////////////////////////////////////////////
// // initialize OpenGL
// // disable unused features
// ///////////////////////////////////////////////////////////////////////////////
// void initGL()
// {
//     glShadeModel(GL_SMOOTH);                    // shading mathod: GL_SMOOTH or GL_FLAT
//     glPixelStorei(GL_UNPACK_ALIGNMENT, 4);      // 4-byte pixel alignment

//     // enable /disable features
//     glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
//     glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
//     //glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);
//     glDisable(GL_DEPTH_TEST);
//     glEnable(GL_LIGHTING);
//     glEnable(GL_TEXTURE_2D);
//     glDisable(GL_CULL_FACE);

//     // track material ambient and diffuse from surface color, call it before glEnable(GL_COLOR_MATERIAL)
//     //glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
//     //glEnable(GL_COLOR_MATERIAL);

//     glClearColor(0, 0, 0, 0);                   // background color
//     glClearStencil(0);                          // clear stencil buffer
//     glClearDepth(1.0f);                         // 0 is near, 1 is far
//     glDepthFunc(GL_LEQUAL);

//     //initLights();
// }



// ///////////////////////////////////////////////////////////////////////////////
// // create glsl programs
// ///////////////////////////////////////////////////////////////////////////////
// bool initGLSL()
// {
//     const int MAX_LENGTH = 2048;
//     char log[MAX_LENGTH];
//     int logLength = 0;

//     // create shader and program
//     GLuint vsId = glCreateShader(GL_VERTEX_SHADER);
//     GLuint fsId = glCreateShader(GL_FRAGMENT_SHADER);
//     progId = glCreateProgram();

//     // load shader sources
//     glShaderSource(vsId, 1, &vsSource, NULL);
//     glShaderSource(fsId, 1, &fsSource, NULL);

//     // compile shader sources
//     glCompileShader(vsId);
//     glCompileShader(fsId);

//     //@@ debug
//     int vsStatus, fsStatus;
//     glGetShaderiv(vsId, GL_COMPILE_STATUS, &vsStatus);
//     if(vsStatus == GL_FALSE)
//     {
//         glGetShaderiv(vsId, GL_INFO_LOG_LENGTH, &logLength);
//         glGetShaderInfoLog(vsId, MAX_LENGTH, &logLength, log);
//         std::cout << "===== Vertex Shader Log =====\n" << log << std::endl;
//     }
//     glGetShaderiv(fsId, GL_COMPILE_STATUS, &fsStatus);
//     if(fsStatus == GL_FALSE)
//     {
//         glGetShaderiv(fsId, GL_INFO_LOG_LENGTH, &logLength);
//         glGetShaderInfoLog(fsId, MAX_LENGTH, &logLength, log);
//         std::cout << "===== Fragment Shader Log =====\n" << log << std::endl;
//     }

//     // attach shaders to the program
//     glAttachShader(progId, vsId);
//     glAttachShader(progId, fsId);

//     // link program
//     glLinkProgram(progId);

//     // get uniform/attrib locations
//     glUseProgram(progId);
//     uniformMatrixModelView           = glGetUniformLocation(progId, "matrixModelView");
//     uniformMatrixModelViewProjection = glGetUniformLocation(progId, "matrixModelViewProjection");
//     uniformMatrixNormal              = glGetUniformLocation(progId, "matrixNormal");
//     uniformMap0                      = glGetUniformLocation(progId, "map0");
//     uniformTextureUsed               = glGetUniformLocation(progId, "textureUsed");
//     attribVertexPosition = glGetAttribLocation(progId, "vertexPosition");
//     attribVertexNormal   = glGetAttribLocation(progId, "vertexNormal");
//     attribVertexTexCoord = glGetAttribLocation(progId, "vertexTexCoord");

//     // set uniform values
//     glUniform1i(uniformMap0, 0);
//     glUniform1i(uniformTextureUsed, 1);

//     // unbind GLSL
//     glUseProgram(0);

//     // check GLSL status
//     int linkStatus;
//     glGetProgramiv(progId, GL_LINK_STATUS, &linkStatus);
//     if(linkStatus == GL_FALSE)
//     {
//         glGetProgramiv(progId, GL_INFO_LOG_LENGTH, &logLength);
//         glGetProgramInfoLog(progId, MAX_LENGTH, &logLength, log);
//         std::cout << "===== GLSL Program Log =====\n" << log << std::endl;
//         return false;
//     }
//     else
//     {
//         return true;
//     }
// }

// ///////////////////////////////////////////////////////////////////////////////
// // initialize global variables
// ///////////////////////////////////////////////////////////////////////////////
// bool initSharedMem()
// {
//     screenWidth = SCREEN_WIDTH;
//     screenHeight = SCREEN_HEIGHT;

//     mouseLeftDown = mouseRightDown = mouseMiddleDown = false;
//     mouseX = mouseY = 0;

//     cameraAngleX = cameraAngleY = 0.0f;
//     cameraDistance = CAMERA_DISTANCE;

//     drawMode = 0; // 0:fill, 1: wireframe, 2:points
//     // debug
//     sphere2.printSelf();

//     return true;
// }

// ///////////////////////////////////////////////////////////////////////////////
// // clean up global vars
// ///////////////////////////////////////////////////////////////////////////////
// void clearSharedMem()
// {
//     // clean up VBOs
//     if(vboSupported)
//     {
//         glDeleteBuffers(1, &vboId1);
//         glDeleteBuffers(1, &iboId1);
//         glDeleteBuffers(1, &vboId2);
//         glDeleteBuffers(1, &iboId2);
//         vboId1 = iboId1 = 0;
//         vboId2 = iboId2 = 0;
//     }
// }

// ///////////////////////////////////////////////////////////////////////////////
// // set camera position and lookat direction
// ///////////////////////////////////////////////////////////////////////////////
// // void setCamera(float posX, float posY, float posZ, float targetX, float targetY, float targetZ)
// // {
// //     glMatrixMode(GL_MODELVIEW);
// //     glLoadIdentity();
// //     // gluLookAt(0.0, 0.0, 5.0,
// //     //          0.0, 0.0, 0.0,
// //     //          0.0, 1.0, 0.0);
// //     gluLookAt(posX, posY, posZ, targetX, targetY, targetZ, 0, 1, 0); // eye(x,y,z), focal(x,y,z), up(x,y,z)
// // }



// ///////////////////////////////////////////////////////////////////////////////
// // load raw image as a texture
// ///////////////////////////////////////////////////////////////////////////////
// GLuint loadTexture(const char* fileName, bool wrap)
// {
//     Image::Bmp bmp;
//     if(!bmp.read(fileName))
//         return 0;     // exit if failed load image

//     // get bmp info
//     int width = bmp.getWidth();
//     int height = bmp.getHeight();
//     const unsigned char* data = bmp.getDataRGB();
//     GLenum type = GL_UNSIGNED_BYTE;    // only allow BMP with 8-bit per channel

//     // We assume the image is 8-bit, 24-bit or 32-bit BMP
//     GLenum format;
//     int bpp = bmp.getBitCount();
//     if(bpp == 8)
//         format = GL_LUMINANCE;
//     else if(bpp == 24)
//         format = GL_RGB;
//     else if(bpp == 32)
//         format = GL_RGBA;
//     else
//         return 0;               // NOT supported, exit

//     // gen texture ID
//     GLuint texture;
//     glGenTextures(1, &texture);

//     // set active texture and configure it
//     glBindTexture(GL_TEXTURE_2D, texture);

//     // select modulate to mix texture with color for shading
//     glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);

//     glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
//     glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
//     //glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP, GL_TRUE);

//     // if wrap is true, the texture wraps over at the edges (repeat)
//     //       ... false, the texture ends at the edges (clamp)
//     glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, wrap ? GL_REPEAT : GL_CLAMP);
//     glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, wrap ? GL_REPEAT : GL_CLAMP);
//     //glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
//     //glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

//     // copy texture data
//     glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, type, data);
//     //glGenerateMipmap(GL_TEXTURE_2D);

//     // build our texture mipmaps
//     switch(bpp)
//     {
//     case 8:
//         gluBuild2DMipmaps(GL_TEXTURE_2D, 1, width, height, GL_LUMINANCE, type, data);
//         break;
//     case 24:
//         gluBuild2DMipmaps(GL_TEXTURE_2D, 3, width, height, GL_RGB, type, data);
//         break;
//     case 32:
//         gluBuild2DMipmaps(GL_TEXTURE_2D, 4, width, height, GL_RGBA, type, data);
//         break;
//     }

//     return texture;
// }

// ///////////////////////////////////////////////////////////////////////////////s
// // set the projection matrix as perspective
// ///////////////////////////////////////////////////////////////////////////////
// void toPerspective(bool left_eye)
// {
//     const float N = 0.3f;
//     const float F = 100.0f;
//     const float DEG2RAD = 3.141592f / 180;
//     const float FOV_Y = 106 * DEG2RAD;

//     if (left_eye) {
//         glViewport(0, 0, (GLsizei)1440, (GLsizei)1600);
//     } else {
//         glViewport(1440, 0, (GLsizei)1440, (GLsizei)1600);
//     }


//     //construct perspective projection matrix
//     float aspectRatio = (float)(1440) / 1600;
//     float tangent = tanf(FOV_Y / 2.0f);     // tangent of half fovY
//     float h = N * tangent;                  // half height of near plane
//     float w = h * aspectRatio;              // half width of near plane
//     matrixProjection.identity();
//     // matrixProjection[0]  =  N / w;
//     // matrixProjection[5]  =  N / h;
//     // matrixProjection[10] = -(F + N) / (F - N);
//     // matrixProjection[11] = -1;
//     // matrixProjection[14] = -(2 * F * N) / (F - N);
//     // matrixProjection[15] =  0;

//     // std::cout << matrixProjection << std::endl;

//     // matrixProjection[0]  =  0.0125f;
//     // matrixProjection[5]  =  0.0112f;
//     // matrixProjection[8]  =  0.0f;
//     // matrixProjection[9]  =  0.0f;
//     // matrixProjection[10] = -1.0006f;
//     // matrixProjection[11] = -1;
//     // matrixProjection[14] = -0.60018;
//     // matrixProjection[15] =  0;
// if (left_eye) {
//     matrixProjection[0]  =  0.78181f;
//     matrixProjection[5]  =  0.70302f;
//     matrixProjection[8]  =  -0.05977f;
//     matrixProjection[9]  =  -0.00503f;
//     matrixProjection[10] = -1.0006f;
//     matrixProjection[11] = -1;
//     matrixProjection[14] = -0.60018;
//     matrixProjection[15] =  0;
// } else {
//     matrixProjection[0]  =  0.78045f;
//     matrixProjection[5]  =  0.70247f;
//     matrixProjection[8]  =  0.05977f;
//     matrixProjection[9]  =  -0.00086f;
//     matrixProjection[10] = -1.0006f;
//     matrixProjection[11] = -1;
//     matrixProjection[14] = -0.60018;
//     matrixProjection[15] =  0;

// }

//     // set perspective viewing frustum
//     glMatrixMode(GL_PROJECTION);
//     glLoadMatrixf(matrixProjection.get());
//     //@@ equivalent fixed pipeline
//     //glLoadIdentity();
//     //gluPerspective(40.0f, (float)(screenWidth)/screenHeight, 0.1f, 100.0f); // FOV, AspectRatio, NearClip, FarClip

//     // switch to modelview matrix in order to set scene
//     //glMatrixMode(GL_MODELVIEW);
//     // glLoadIdentity();
// }

// void renderEye(bool left_eye, Matrix4 matrixModel, Matrix4 matrixView) {

//      // bind GLSL, texture
//     glUseProgram(progId);
//     glBindTexture(GL_TEXTURE_2D, texId);

//     // activate attribs
//     glEnableVertexAttribArray(attribVertexPosition);
//     glEnableVertexAttribArray(attribVertexNormal);
//     glEnableVertexAttribArray(attribVertexTexCoord);

//     // bind vbo for smooth sphere (center and right)
//     glBindBuffer(GL_ARRAY_BUFFER, vboId2);
//     glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, iboId2);

//     // set attrib arrays using glVertexAttribPointer()
//     int stride = sphere2.getInterleavedStride();
//     glVertexAttribPointer(attribVertexPosition, 3, GL_FLOAT, false, stride, 0);
//     glVertexAttribPointer(attribVertexNormal, 3, GL_FLOAT, false, stride, (void*)(3 * sizeof(float)));
//     glVertexAttribPointer(attribVertexTexCoord, 2, GL_FLOAT, false, stride, (void*)(6 * sizeof(float)));


//     toPerspective(left_eye);
//     glMatrixMode(GL_MODELVIEW);
//     glLoadIdentity();

//     //set matric uniforms for left eye
//     matrixModelView = matrixView * matrixModel;
//     Matrix4 matrixModelViewProjection = matrixProjection * matrixModelView;
//     Matrix4 matrixNormal = matrixModelView;
//     matrixNormal.setColumn(3, Vector4(0,0,0,1));
//     glUniformMatrix4fv(uniformMatrixModelView, 1, false, matrixModelView.get());
//     glUniformMatrix4fv(uniformMatrixModelViewProjection, 1, false, matrixModelViewProjection.get());
//     glUniformMatrix4fv(uniformMatrixNormal, 1, false, matrixNormal.get());

//     // right sphere is rendered with texture
//     glUniform1i(uniformTextureUsed, 1);

//     // draw left eye
//     glDrawElements(GL_TRIANGLES,            // primitive type
//                    sphere2.getIndexCount(), // # of indices
//                    GL_UNSIGNED_INT,         // data type
//                    (void*)0);               // ptr to indices
//     glBindTexture(GL_TEXTURE_2D, 0);

// }






// //=============================================================================
// // CALLBACKS
// //=============================================================================

// void displayCB()
// {
//     if(!vboSupported || !glslSupported)
//         return;

//     // clear buffer
//     glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

//     // transform camera (view)
//     Matrix4 matrixView;

//     // common model matrix
//     Matrix4 matrixModelCommon;
//     matrixModelCommon.rotateX(-90);
//     matrixModelCommon.rotateY(cameraAngleY);
//     matrixModelCommon.rotateX(cameraAngleX);

//     Matrix4 matrixModelL(matrixModelCommon);    // left
//     Matrix4 matrixModelR(matrixModelCommon);    // right
//     matrixModelL.translate(-0.032f, 0, 0);        // shift left
//     matrixModelR.translate(0.032f, 0, 0);         // shift right

//     renderEye(true, matrixModelL, matrixView);
//     renderEye(false, matrixModelR, matrixView);

//     glDisableVertexAttribArray(attribVertexPosition);
//     glDisableVertexAttribArray(attribVertexNormal);
//     glDisableVertexAttribArray(attribVertexTexCoord);

//     // unbind
//     glBindTexture(GL_TEXTURE_2D, 0);
//     glBindBuffer(GL_ARRAY_BUFFER, 0);
//     glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
//     glUseProgram(0);

//     //showInfo();     // print max range of glDrawRangeElements

//     glutSwapBuffers();
// }


// void timerCB(int millisec)
// {
//     glutTimerFunc(millisec, timerCB, millisec);
//     glutPostRedisplay();
// }


// void mouseCB(int button, int state, int x, int y)
// {
//     mouseX = x;
//     mouseY = y;

//     if(button == GLUT_LEFT_BUTTON)
//     {
//         if(state == GLUT_DOWN)
//         {
//             mouseLeftDown = true;
//         }
//         else if(state == GLUT_UP)
//             mouseLeftDown = false;
//     }

//     else if(button == GLUT_RIGHT_BUTTON)
//     {
//         if(state == GLUT_DOWN)
//         {
//             mouseRightDown = true;
//         }
//         else if(state == GLUT_UP)
//             mouseRightDown = false;
//     }

//     else if(button == GLUT_MIDDLE_BUTTON)
//     {
//         if(state == GLUT_DOWN)
//         {
//             mouseMiddleDown = true;
//         }
//         else if(state == GLUT_UP)
//             mouseMiddleDown = false;
//     }
// }


// void mouseMotionCB(int x, int y)
// {
//     if(mouseLeftDown)
//     {
//         cameraAngleY += (x - mouseX);
//         cameraAngleX += (y - mouseY);
//         mouseX = x;
//         mouseY = y;
//     }
//     if(mouseRightDown)
//     {
//         cameraDistance -= (y - mouseY) * 0.2f;
//         mouseY = y;
//     }
// }
