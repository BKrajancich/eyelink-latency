#include <GL/glew.h>
#include <GL/glut.h>

#include <GL/gl.h>

#include <list>

void display(void)
{
    int const window_width  = glutGet(GLUT_WINDOW_WIDTH);
    int const window_height = glutGet(GLUT_WINDOW_HEIGHT);
    float const window_aspect = (float)window_width / (float)window_height;

    glClearColor(0.5, 0.5, 1.0, 1.0);
    glClearDepth(1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glViewport(0, 0, window_width, window_height);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    if(window_aspect > 1.) {
        glOrtho(-window_aspect, window_aspect, -1, 1, -1, 1);
    }
    else {
        glOrtho(-1, 1, -1/window_aspect, 1/window_aspect, -1, 1);
    }
    
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    GLfloat const light_pos[4]     = {-1.00,  1.00,  1.00, 0.};
    GLfloat const light_color[4]   = { 0.85,  0.90,  0.70, 1.};
    GLfloat const light_ambient[4] = { 0.10,  0.10,  0.30, 1.};
    glLightfv(GL_LIGHT0, GL_POSITION, light_pos),
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light_color);
    glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
    
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);

    glEnable(GL_DEPTH_TEST);

    glutSolidSphere(0.1, 31, 10);

    glutSwapBuffers();
}

int main(
    int argc,
    char *argv[] )
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);

    glewInit();

    glutInitWindowSize(1920, 1080);  // window size
    glutInitWindowPosition(100, 100);               // window location

    glutCreateWindow("Click to place spheres");
    glutDisplayFunc(display);

    glutMainLoop();

    return 0;
}