/*

Compile with: 

    gcc physicsViewer.c -o physicsViewer -O2 -lm -std=c99 -lGL -lglut -lGLU -w

    -O2      Optimization
    -lm      Link to math lib
    -std=c99 Use of for(;;;) with declaration among other things
    -lGL     Basic OpenGL functions
    -lglut   Window & events manager
    -lGLU    OpenGL utilities
    -w       Hide warnings (ONLY FINAL VERSION)

Usage:

    ./physicsViewer dump.txt

Jan Mas Rovira
Andres Mingorance Lopez
Albert Puente Encinas
  
*/
#include <GL/gl.h>
#include <GL/freeglut.h>

#include <stdio.h>  // e.g. printf
#include <stdlib.h> // e.g. malloc, RAND_MAX
#include <math.h>   // e.g. sin, abs

// Physics algorithm parameters
int nPoints;
int itLimit;
float pointRadius;

#define FPS 60.0;   // Viewer maximum framerate

// c++ style
typedef int bool;
#define true 1
#define false 0

// OpenGL points array and VBO pointer
float* V;
int Vid, Cid;

// OpenGL shader program
GLuint sProgram;

void loadShaders() {
    
    GLuint vsh = glCreateShader(GL_VERTEX_SHADER);
    
    char* vshcode = "\
        varying vec3 pos;\
        void main() {\
            pos = gl_Vertex.xyz;\
            gl_Position = gl_ModelViewProjectionMatrix*gl_Vertex;\
            vec3 ndc = gl_Position.xyz/gl_Position.w;\
            float zDist = 1.0-ndc.z;\
            float size = zDist*10000.0;\
            gl_PointSize = size;\
        }";
        
    glShaderSource(vsh, 1, &vshcode, NULL);
    glCompileShader(vsh);
    GLint success = 0;
    glGetShaderiv(vsh, GL_COMPILE_STATUS, &success);
    printf("Compile vertex shader: %i\n", success);
   
    GLuint fsh = glCreateShader(GL_FRAGMENT_SHADER);

    char* fshcode = "\
        varying vec3 pos;\
        void main() {\
            float r = 1.0 - abs(pos[0]/10.0); \
            float g = 1.0 - abs(pos[1]/10.0); \
            float b = 1.0 - abs(pos[2]/10.0); \
            gl_FragColor = vec4(r,g,b,1.0);\
        }";
        
    glShaderSource(fsh, 1, &fshcode, NULL);
    glCompileShader(fsh);
    success = 0;
    glGetShaderiv(fsh, GL_COMPILE_STATUS, &success);
    printf("Compile fragment shader: %i\n", success);
    
    sProgram = glCreateProgram();
    glAttachShader(sProgram, vsh);
    glAttachShader(sProgram, fsh);
    glLinkProgram(sProgram);
    success = 0;
    glGetProgramiv(sProgram, GL_LINK_STATUS, &success);
    printf("Shaders linked: %i\n", success);
    
    glDetachShader(sProgram, vsh);
    glDetachShader(sProgram, fsh);
}

bool play = true;
float rotCamX = 0;
float rotCamY = 0;
float angleR = 3.141592 * -45/180;
float alturaCam = 3; // inicial
float viewportWidth, viewportHeight; // mida vigent
float ra; // relacio d'aspecte vigent
float scale = 0.6;
float T = 1/FPS;
int Fit;

inline void DD(char* debug_msg) {
    printf(debug_msg);
    printf("\n");
}

void drawText() {  
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(0.0, viewportHeight, 0.0, viewportHeight);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    glColor3f(1, 1, 1);
    glNormal3f(0, 1, 0);        
    float v1[3] = {1, 1, 1};
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, v1);
    float v2[3] = {1, 1, 1};
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, v2);
    float v3[3] = {1, 1, 1};
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, v3);  
    glRasterPos2i(10, 10);
    char buffer[100] = {(char)NULL};
    
    snprintf(buffer, sizeof(buffer), 
             "%i points Iteration: %i/%i (Controls: w a s d | q e | r SPACEBAR | + - )", 
             nPoints, Fit+1, itLimit);
    
    void * font = GLUT_BITMAP_HELVETICA_12;
    for (int i = 0; i < sizeof(buffer); ++i) {
        glutBitmapCharacter(font, buffer[i]);
    }
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
}

void drawPoints() {

    GLint uLocation = glGetUniformLocation(sProgram, "pointRadius");
    glUniform1f(uLocation, pointRadius);

    glPushMatrix();
    glColor3f(1, 1, 1);
    glScalef(scale, scale, scale); 

    glBindBuffer(GL_ARRAY_BUFFER, Vid);         
    glVertexPointer(3, GL_FLOAT, 0, 0);
    glEnableClientState(GL_VERTEX_ARRAY);

    glDrawArrays(GL_POINTS, Fit*nPoints, nPoints); 

    glDisableClientState(GL_VERTEX_ARRAY); 
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glPopMatrix();
}

void drawBox() {
    glPushMatrix();
    glScalef(scale, scale, scale);
    glBegin(GL_LINES);
    glColor3f(1.0, 1.0,1.0);
    glVertex3f(-10.0, 0.0, 10.0);
    glVertex3f(10.0, 0.0, 10.0);
    
    glVertex3f(10.0, 0.0, 10.0);
    glVertex3f(10.0, 0.0, -10.0);
    
    glVertex3f(10.0, 0.0, -10.0);
    glVertex3f(-10.0, 0.0, -10.0);
    
    glVertex3f(-10.0, 0.0, -10.0);
    glVertex3f(-10.0, 0.0, 10.0);
    
    glVertex3f(-10.0, 0.0, -10.0);
    glVertex3f(-10.0, 20, -10.0);
    
    glVertex3f(10.0, 0.0, -10.0);
    glVertex3f(10.0, 20, -10.0);
    
    glVertex3f(10.0, 0.0, 10.0);
    glVertex3f(10.0, 20, 10.0);
    
    glVertex3f(-10.0, 0.0, 10.0);
    glVertex3f(-10.0, 20, 10.0);
    glEnd();
    
    glPopMatrix(); 
}

void refresh() {
    
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    drawText();
    glUseProgram(sProgram);
    drawPoints();
    glUseProgram(0);
    drawBox();
    glutSwapBuffers();
}

void changeCameraPerspective() {
    glMatrixMode(GL_PROJECTION); 
    glLoadIdentity();     
    gluPerspective(60, ra, 0.01, 50);
    glMatrixMode(GL_MODELVIEW); 
}

void moveCamera() {
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(0+10*sin(angleR), 10 + alturaCam, 10*cos(angleR),
                0, 2, 0,
                0, 2, 0);
    changeCameraPerspective();
    glutPostRedisplay();
}

void resize(int ample, int alt) {  
    glViewport(0, 0, ample, alt);
    ra = (float)ample/(float)alt;
    changeCameraPerspective();
    glutPostRedisplay();
}

void keyPressed(unsigned char character, int posX, int posY) {
    if (character == 27) exit(0);
    else if (character == 'w') {
        rotCamY = 0.4;
    }
    else if (character == 's') {
        rotCamY = -0.4;
    }
    else if (character == 'a') {
        rotCamX = 0.02;
    }
    else if (character == 'd') {
        rotCamX = -0.02;
    }
    else if (character == ' ') {
        play = !play;
    }
    else if (character == 'q') {
        scale -= 0.01;
    }
    else if (character == 'e') {
        scale += 0.01;
    }
    else if (character == 'r') {
        Fit = 0;
    }
    moveCamera();
}

void initGLUT() {     
    glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
    viewportHeight = 600;
    viewportWidth = 900;
    ra = viewportHeight/viewportWidth;
    glutInitWindowSize(viewportWidth, viewportHeight);
    glutCreateWindow("Physics 3D");
    glutReshapeFunc(resize);
    glutDisplayFunc(refresh);
    glutKeyboardFunc(keyPressed); 
}

void girarCamera() {
    angleR += rotCamX;
    alturaCam += rotCamY;
    rotCamX *= 0.9;
    rotCamY *= 0.9;
    moveCamera(); 
}

void nextFrame(int v) {
    if (play) {
        Fit++;
        if (Fit >= itLimit) {
            Fit = 0;
        }
    }
    girarCamera();
    glutPostRedisplay();
    glutTimerFunc(T*1000, nextFrame, 0);
}

void initCamera() { 
    changeCameraPerspective();    
    moveCamera();    
}

void initGL() {
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.1, 0.1, 0.1, 0);     
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glEnable(GL_NORMALIZE);  
    glEnable(GL_SMOOTH);
    
    glDisable(GL_LIGHTING);
    
    float v01[3] = { 0.8, 0.8, 0.8};
    glLightfv(GL_LIGHT0, GL_DIFFUSE, v01);
    float v02[3] = { 0.3, 0.3, 0.3};
    glLightfv(GL_LIGHT0, GL_AMBIENT, v02);
    float v03[3] = { 1, 1, 1};
    glLightfv(GL_LIGHT0, GL_SPECULAR, v03);
    glEnable(GL_LIGHT0);        
    
    glEnable(GL_POINT_SMOOTH);
    glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);    
  
    glGenBuffers(1, &Vid);    
    glBindBuffer(GL_ARRAY_BUFFER, Vid);    
    glBufferData(GL_ARRAY_BUFFER, 3*sizeof(float)*(itLimit+1)*nPoints, V, GL_STATIC_DRAW);
    
    glEnable(GL_PROGRAM_POINT_SIZE);
    
    loadShaders();
} 

void open3DWindow(int argc, char** argv) {
    glutInit(&argc, (char **) argv);
    initGLUT();
    initGL();
    initCamera();
    
    Fit = 0;
    printf("Camera: w,a,s,d   Zoom: q, e\nStop: SPACE   Reset: r \n");
    glutTimerFunc(T*1000, nextFrame, 0);
    glutMainLoop();
}

int readFile(char* name) {
    FILE* file = fopen(name, "r");   

    fscanf(file, "%i", &nPoints);

    fscanf(file, "%i", &itLimit);
    
    fscanf(file, "%f", &pointRadius);
    pointRadius *= 20000;
    printf("%f\n", pointRadius);
    V = malloc(3*sizeof(float)*(itLimit+1)*nPoints);
    
    for (int i = 0; i < nPoints*(itLimit + 1); i++) {
        for (int j = 0 ; j < 3; j++) {
            fscanf(file, "%f", &V[3*i + j]);
        }
    }

    fclose(file);
}

int main(int argc, char** argv) {
    if (argc < 2) {
        printf("Usage: physicsViewer input\n");
        exit(EXIT_FAILURE);
    }
    else {
        printf("Starting viewer...\n");
    }
    readFile(argv[1]);
    open3DWindow(argc, argv);
    return 0;
}

