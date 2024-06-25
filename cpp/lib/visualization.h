#pragma once
#include <algorithm>
#include <iostream>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <ft2build.h>
#include FT_FREETYPE_H
#include "mpu9250.h"
#include "shader.h"
#include "utils.h"

using namespace std;
using namespace glm;

struct character {
    GLuint texture_id;
    int width;         // character width
    int height;        // character height
    int bearing_x;      // character x-axis bearing
    int bearing_y;      // character y-axis bearing
    GLuint advance;    // distance to next character
};

class imu_viewer
{
    /**
     * @brief Create window to display IMU information and visualize the IMU model

     * @param width: display window width
     * @param height: display window height
     * @param hz: window update rate
     * @param nav_frame: navigation frame
     */

    private:
        GLint width;
        GLint height;
        float hz;
        string nav_frame, rotation_seq;
        GLFWwindow* window;
        GLuint text_VAO, text_VBO;
        GLuint cube_VAO, cube_VBO;
        int vertex_size;
        float max_x=0.2f, min_x=-0.2f, max_y=0.5f, min_y=-0.5f, max_z=0.02f, min_z=-0.02f;
        map<GLchar, character> characters;
        FT_Library ft;
        FT_Face face;

    public:
        imu_viewer(GLint width, GLint height, float hz, string nav_frame){
            this->width = width;
            this->height = height;
            this->hz = hz;
            this->nav_frame = nav_frame;
            GLfloat* vertex_array = this->initialize_cube(this->max_x, this->min_x, this->max_y, this->min_y, this->max_z, this->min_z);
            this->initialize_glfw(vertex_array);
            this->initialize_freetype();

            if ((this->nav_frame != "ENU") and (this->nav_frame != "NED")){
                throw invalid_argument("Navigation frame should be either ENU or NED"); 
            }
            if (this->nav_frame == "ENU"){
                this->rotation_seq = "zxy";
            }
            else if (this->nav_frame == "NED"){
                this->rotation_seq = "zyx";
            }
            else{
                throw invalid_argument("Navigation frame should be either ENU or NED");
            }
        }
        void run(MPU9250& imu);
        GLfloat* initialize_cube(float max_x, float min_x, float max_y, float min_y, float max_z, float min_z);
        void initialize_freetype();
        void initialize_glfw(GLfloat* vertex_array);
        void cube_display(shader custom_shader, MPU9250& imu);
        void message_display(shader custom_shader, string text, GLfloat x, GLfloat y, GLfloat scale, vec3 color);
};