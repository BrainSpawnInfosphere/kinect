
//----------- C++ -------------
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <sys/types.h>
#include <time.h>
#include <math.h>
#include <list>
#include <vector>
#include <boost/thread/mutex.hpp>

//------------ PCL -------------
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
//#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
//#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <iostream>

//------------ ROS --------------
#include <ros/ros.h>
#include <image_geometry/pinhole_camera_model.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

namespace enc = sensor_msgs::image_encodings;

//----------- OpenCV -----------
#include <opencv2/opencv.hpp>
//#include <opencv2/highgui/highgui.hpp>

//----------- Glut & OpenGL -----
#include <GLUT/glut.h>	 // GLUT
#include <OpenGL/glu.h>	    // GLU
#include <OpenGL/gl.h>	    // OpenGL

#include "cloud_maker.hpp"


/*
#ifndef PI
#define PI 3.1415926535897
#endif
#ifndef LARGE
#define LARGE 3.1415926535897
#endif

using namespace std;

// colors
float WHITE[] = {1,1,1,1};
float BLACK[] = {0,0,0,1};
float DARK[] = {.1,.1,.1,1};
float LIGHTGRAY[] = {.8,.8,.8,1};
float LIGHT[] = {.9,.9,.9,1};
float RED[] = {1,0,0,1};
float BLUE[] = {0,0,1,1};
float GREEN[] = {0,1,0,1};
float ORANGE[] = {1,.5,0,1};
float LIGHTBLUE[] = {0,1,1,1};
float PURPLE[] = {1,0,1,1};
*/

// globals used for keeping track of display attributes
//int display_size_init = 600; // pixels
//float win_aspect_ratio = 1;
//float scale_factor = 1; // radius of the size of the word
/*
// globals used for user interaction
int left_pressed = 0;
int right_pressed = 0;
float mouse_pan_x_last;
float mouse_pan_y_last;
float mouse_zoom_y_init;
*/

// global 
CloudMaker *cm = NULL;
cv::Mat dummy(480,640,CV_8UC1);



/*----------------------- GLUT Callbacks --------------------------------*/

int window = 0;
int rotangles[2] = {0}; // Panning angles
float zoom = 1;         // zoom factor
int mx=-1,my=-1;        // Prevous mouse coordinates


/*
void initRendering() {
    //...
    glEnable(GL_LIGHTING); //Enable lighting
    glEnable(GL_LIGHT0); //Enable light #0
    glEnable(GL_LIGHT1); //Enable light #1
    glEnable(GL_NORMALIZE); //Have OpenGL automatically normalize our normals
    glShadeModel(GL_SMOOTH); //Enable smooth shading
    //...
}
*/

bool mouseMove = false;

void mouseMoved(int x, int y){
    if(!mouseMove) return;
    
    if (1 /*mx>=0 && my>=0*/) {
        rotangles[0] += y-my;
        rotangles[1] += x-mx;
    }
    mx = x;
    my = y;
}

void mousePress(int button, int state, int x, int y){
    if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
        mx = x;
        my = y;
        
        mouseMove = true;
    }
}
/*
void LoadVertexMatrix()
{
    float fx = 594.21f;
    float fy = 591.04f;
    float a = -0.0030711f;
    float b = 3.3309495f;
    float cx = 339.5f;
    float cy = 242.7f;
    GLfloat mat[16] = {
        1/fx,     0,  0, 0,
        0,    -1/fy,  0, 0,
        0,       0,  0, a,
        -cx/fx, cy/fy, -1, b
    };
    glMultMatrixf(mat);
}


void LoadRGBMatrix()
{
    float mat[16] = {
        5.34866271e+02,   3.89654806e+00,   0.00000000e+00,   1.74704200e-02,
        -4.70724694e+00,  -5.28843603e+02,   0.00000000e+00,  -1.22753400e-02,
        -3.19670762e+02,  -2.60999685e+02,   0.00000000e+00,  -9.99772000e-01,
        -6.98445586e+00,   3.31139785e+00,   0.00000000e+00,   1.09167360e-02
    };
    glMultMatrixf(mat);
}
*/


void GLViewer::drawAxis(float scale){
    glBegin(GL_LINES);
    glLineWidth(4);
    glColor4f (0.9, 0, 0, 1.0);
    glVertex3f(0, 0, 0);
    glColor4f (0.9, 0, 0, 0.0);
    glVertex3f(scale, 0, 0);
    glColor4f (0, 0.9, 0, 1.0);
    glVertex3f(0, 0, 0);
    glColor4f (0, 0.9, 0, 0.0);
    glVertex3f(0, scale, 0);
    glColor4f (0, 0, 0.9, 1.0);
    glVertex3f(0, 0, 0);
    glColor4f (0, 0, 0.9, 0.0);
    glVertex3f(0, 0, scale);
    glEnd();
}


// default display function, gets called once each loop through glut
void display(){

    if(0){
        glClear (GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
        glLoadIdentity ();             // clear the matrix 
        // viewing transformation  
        // +z-axis out of screen, camera looks down -z-axis
#if 0        
        gluLookAt (320.0, 240.0, -1000.0, // eye
                   320.0, 240.0, 100.0, // center
                   0.0, 1.0, 0.0); // up
#elif 0   
        gluLookAt (640.0, 480.0, -1000.0, // eye
                   320.0, 240.0, 100.0, // center
                   0.0, 1.0, 0.0); // up
#elif 1   
        gluLookAt (0.0, 1.0, 1.0, // eye
                   0.0, 0.0, 0.50, // center
                   0.0, 1.0, 0.0); // up
#endif        
        
    }
    else {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glLoadIdentity();
    
        glPushMatrix();
        glScalef(zoom,zoom,1);
        glTranslatef(0,0,-3.5);
        glRotatef(rotangles[0], 1,0,0);
        glRotatef(rotangles[1], 0,1,0);
        glTranslatef(0,0,1.5);
    
        //LoadVertexMatrix();
    
        // Set the projection from the XYZ to the texture image
        //glMatrixMode(GL_TEXTURE);
        //glLoadIdentity();
        //glScalef(1/640.0f,1/480.0f,1);
        //LoadRGBMatrix();
        //LoadVertexMatrix();
        glMatrixMode(GL_MODELVIEW);
    }
    
 
    
    //glutSolidSphere(0.05f, 15, 8);
    
    cm->gl();
    
    glPopMatrix();

    // draw the updated scene
    //glFlush();
    glutSwapBuffers();
}


// this routine is called when the window is resized
void reshape(int width,int height)
{
    
   GLfloat w = width, h = height;
   
   glViewport (0, 0, (GLsizei) w, (GLsizei) h); 
   glMatrixMode (GL_PROJECTION);
   glLoadIdentity ();
   gluPerspective(45.0,w/h,0.5,200.0);
   glMatrixMode (GL_MODELVIEW);
}


// this routine gets called when a normal key is pressed
void key(unsigned char key,int x,int y){
    // ESC = 27
    if (key == 'q' || key == 'Q' || key == 27){
        glutDestroyWindow(window);
        exit(0);
    }
    if (key == 'w') zoom *= 1.1f;
    if (key == 's') zoom /= 1.1f;
    if (key == 'r'){
        rotangles[0] = rotangles[1] = 0;
        zoom = 1;
    }
}

// this routine is called when nothing else is happening
void idle()
{
    ros::spinOnce();
    //ROS_INFO("idle()... ");
    glutPostRedisplay();
}

// this gets called when glut exits
void cleanup()
{
    ROS_INFO("Exit... ");
    exit(0);
}


void glutSetup(void) {
    int display_size_init = 600; // pixels

    //  Request double buffered, true color window with Z buffering at 600x600
    glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
    glutInitWindowSize(display_size_init,display_size_init);
    glutInitWindowPosition(0, 0);
    
    window = glutCreateWindow("Visualization");

    //  Set GLUT callbacks
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    //glutSpecialFunc(special);
    glutKeyboardFunc(key);

    //glutPassiveMotionFunc(motion);
    //glutMouseFunc(mouse);
    //glutMotionFunc(active_motion);
    glutMotionFunc(&mouseMoved);
    glutMouseFunc(&mousePress);
    glutIdleFunc(idle);
    atexit(cleanup);
    
    //initRendering(); // turn on lighting

    glutPostRedisplay();
}


int main(int argc, char *argv[])
{
    // init ROS stuff
    ros::init(argc, argv, "pcl_view");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);
    ROS_INFO("Start");
    
    CloudMaker cloud;
    cm = &cloud;
    
    //-- test
    cv::randu(dummy,cv::Scalar(0),cv::Scalar(100));


    image_transport::ImageTransport transport(n);
    image_transport::CameraSubscriber depth_sub = transport.subscribeCamera("/camera/depth/image_raw", 1, &CloudMaker::depthCb, cm /*, image_transport::TransportHints("compressed")*/);
  
    //  Initialize GLUT
    glutInit(&argc,argv);
    glutSetup();
    
    //sleep(3);

    //  Pass control to GLUT so it can interact with the user
    glutMainLoop();

    return 0;
}


