#include "full_display.hh"


const char* vertex_shader_text = R"(
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
 
const char* fragment_shader_text = R"(
// GLSL version
#version 110
// uniforms
uniform sampler2D map0;                 // texture map #1
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
 
// void FullDisplay::error_callback(int error, const char* description)
// {
//     fprintf(stderr, "Error: %s\n", description);
// }
 
// void FullDisplay::key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
// {
//     if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
//         glfwSetWindowShouldClose(window, GLFW_TRUE);
// }

Matrix4 FullDisplay::toPerspective(bool left_eye)
{   Matrix4 matrixProjection;
    const float N = 0.3f;
    const float F = 100.0f;
    const float DEG2RAD = 3.141592f / 180;
    const float FOV_Y = 40 * DEG2RAD;

    if (left_eye) {
        glViewport(0, 0, (GLsizei)640, (GLsizei)480);
    } else {
        glViewport(0, 0, (GLsizei)640, (GLsizei)480);
    }
    

    //construct perspective projection matrix
    float aspectRatio = (float)(640) / 480;
    float tangent = tanf(FOV_Y / 2.0f);     // tangent of half fovY
    float h = N * tangent;                  // half height of near plane
    float w = h * aspectRatio;              // half width of near plane
    matrixProjection.identity();
    matrixProjection[0]  =  N / w;
    matrixProjection[5]  =  N / h;
    matrixProjection[10] = -(F + N) / (F - N);
    matrixProjection[11] = -1;
    matrixProjection[14] = -(2 * F * N) / (F - N);
    matrixProjection[15] =  0;

    // std::cout << matrixProjection << std::endl;

    // matrixProjection[0]  =  0.0125f;
    // matrixProjection[5]  =  0.0112f;
    // matrixProjection[8]  =  0.0f;
    // matrixProjection[9]  =  0.0f;
    // matrixProjection[10] = -1.0006f;
    // matrixProjection[11] = -1;
    // matrixProjection[14] = -0.60018;
    // matrixProjection[15] =  0;
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

    // set perspective viewing frustum
    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf(matrixProjection.get());

    // switch to modelview matrix in order to set scene
    //glMatrixMode(GL_MODELVIEW);
    // glLoadIdentity();
    return matrixProjection;
}

GLuint FullDisplay::loadTexture(const char* fileName, bool wrap)
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
 
int FullDisplay::test()
{
    int width, height;
    
    GLFWwindow* window;

    Matrix4 matrixModelView;
    GLuint vertex_buffer, vertex_shader, fragment_shader, program;
    GLint uniformMatrixModelView;
    GLint uniformMatrixModelViewProjection;
    GLint uniformMatrixNormal;
    GLint uniformMap0;
    GLint attribVertexPosition;
    GLint attribVertexNormal;
    GLint attribVertexTexCoord;
    GLuint index_buffer = 0;
    Sphere sphere2(100.0f, 36, 18);    
    GLuint texId;
    
    // load BMP image
    texId = loadTexture("/home/brooke/repos/eyelink-latency/src/frontend/frame92.bmp", true);

    //glfwSetErrorCallback(error_callback);
 
    if (!glfwInit())
        exit(EXIT_FAILURE);
    
    glfwDefaultWindowHints();

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    glfwWindowHint( GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE );

    glfwWindowHint( GLFW_RESIZABLE, GL_TRUE );
 
    window = glfwCreateWindow(640, 480, "Simple example", NULL, NULL);
    if (!window)
    {
        glfwTerminate();
        std::cout << "failed here" << std::endl;
        exit(EXIT_FAILURE);
    }
 
    //glfwSetKeyCallback(window, key_callback);
 
    glfwMakeContextCurrent(window);
    //gladLoadGL(glfwGetProcAddress);
    glfwSwapInterval(1);
 
    // NOTE: OpenGL error checks have been omitted for brevity
 
    glGenBuffers(1, &vertex_buffer);
    glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer);
    //glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
    glBufferData(GL_ARRAY_BUFFER, sphere2.getInterleavedVertexSize(), sphere2.getInterleavedVertices(), GL_STATIC_DRAW);
    glGenBuffers(1, &index_buffer);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_buffer);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sphere2.getIndexSize(), sphere2.getIndices(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    vertex_shader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertex_shader, 1, &vertex_shader_text, NULL);
    glCompileShader(vertex_shader);
 
    fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragment_shader, 1, &fragment_shader_text, NULL);
    glCompileShader(fragment_shader);
 
    program = glCreateProgram();

    //@@ debug

    const int MAX_LENGTH = 2048;
    char log[MAX_LENGTH];
    int logLength = 0;

    int vsStatus, fsStatus;
    glGetShaderiv(vertex_shader, GL_COMPILE_STATUS, &vsStatus);
    if(vsStatus == GL_FALSE)
    {
        glGetShaderiv(vertex_shader, GL_INFO_LOG_LENGTH, &logLength);
        glGetShaderInfoLog(vertex_shader, MAX_LENGTH, &logLength, log);
        std::cout << "===== Vertex Shader Log =====\n" << log << std::endl;
    }
    glGetShaderiv(fragment_shader, GL_COMPILE_STATUS, &fsStatus);
    if(fsStatus == GL_FALSE)
    {
        glGetShaderiv(fragment_shader, GL_INFO_LOG_LENGTH, &logLength);
        glGetShaderInfoLog(fragment_shader, MAX_LENGTH, &logLength, log);
        std::cout << "===== Fragment Shader Log =====\n" << log << std::endl;
    }


    glAttachShader(program, vertex_shader);
    glAttachShader(program, fragment_shader);
    glLinkProgram(program);


    // get uniform/attrib locations
    glUseProgram(program);
    uniformMatrixModelView           = glGetUniformLocation(program, "matrixModelView");
    uniformMatrixModelViewProjection = glGetUniformLocation(program, "matrixModelViewProjection");
    uniformMatrixNormal              = glGetUniformLocation(program, "matrixNormal");
    uniformMap0                      = glGetUniformLocation(program, "map0");
    attribVertexPosition = glGetAttribLocation(program, "vertexPosition");
    attribVertexNormal   = glGetAttribLocation(program, "vertexNormal");
    attribVertexTexCoord = glGetAttribLocation(program, "vertexTexCoord");

 
    // mvp_location = glGetUniformLocation(program, "MVP");
    // vpos_location = glGetAttribLocation(program, "vPos");
    // vcol_location = glGetAttribLocation(program, "vCol");
 
    // glEnableVertexAttribArray(vpos_location);
    // glVertexAttribPointer(vpos_location, 2, GL_FLOAT, GL_FALSE,
    //                       sizeof(vertices[0]), (void*) 0);
    // glEnableVertexAttribArray(vcol_location);
    // glVertexAttribPointer(vcol_location, 3, GL_FLOAT, GL_FALSE,
    //                       sizeof(vertices[0]), (void*) (sizeof(float) * 2));
 
    while (!glfwWindowShouldClose(window))
    {
        glfwGetFramebufferSize(window, &width, &height);
 
        glViewport(0, 0, width, height);

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
        

        // transform camera (view)
        Matrix4 matrixView;

        // common model matrix
        Matrix4 matrixModelCommon;
        matrixModelCommon.rotateX(-90);
        matrixModelCommon.rotateY(0);
        matrixModelCommon.rotateX(0);

        Matrix4 matrixModel(matrixModelCommon);    // left
        Matrix4 matrixModelR(matrixModelCommon);    // right
        matrixModel.translate(-0.032f, 0, 0);        // shift left
        matrixModelR.translate(0.032f, 0, 0);         // shift right

         // bind GLSL, texture
        glUseProgram(program);
        glBindTexture(GL_TEXTURE_2D, texId);

        // activate attribs
        glEnableVertexAttribArray(attribVertexPosition);
        glEnableVertexAttribArray(attribVertexNormal);
        glEnableVertexAttribArray(attribVertexTexCoord);

        // bind vbo for smooth sphere (center and right)
        glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_buffer);

        // set attrib arrays using glVertexAttribPointer()
        int stride = sphere2.getInterleavedStride();
        glVertexAttribPointer(attribVertexPosition, 3, GL_FLOAT, false, stride, 0);
        glVertexAttribPointer(attribVertexNormal, 3, GL_FLOAT, false, stride, (void*)(3 * sizeof(float)));
        glVertexAttribPointer(attribVertexTexCoord, 2, GL_FLOAT, false, stride, (void*)(6 * sizeof(float)));

        
        Matrix4 matrixProjection = toPerspective(true); 
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

        // draw left eye
        glDrawElements(GL_TRIANGLES,            // primitive type
                    sphere2.getIndexCount(), // # of indices
                    GL_UNSIGNED_INT,         // data type
                    (void*)0);               // ptr to indices
        glBindTexture(GL_TEXTURE_2D, 0);

        glDisableVertexAttribArray(attribVertexPosition);
        glDisableVertexAttribArray(attribVertexNormal);
        glDisableVertexAttribArray(attribVertexTexCoord);

        // unbind
        glBindTexture(GL_TEXTURE_2D, 0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
        glUseProgram(0);


 
        //glUseProgram(program);
        //glUniformMatrix4fv(mvp_location, 1, GL_FALSE, mvp.get());
        //glDrawArrays(GL_TRIANGLES, 0, 3);
 
        glfwSwapBuffers(window);
        glfwPollEvents();
    }
 
    glfwDestroyWindow(window);
 
    glfwTerminate();
    exit(EXIT_SUCCESS);
}