#pragma once
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <GL/glew.h>
#include <GLFW/glfw3.h>

using namespace std;

class shader
{
    private:
        GLuint vertex, fragment; // vertex and fragment shader

    public:
        GLuint program;
        shader(const GLchar *vertex_path, const GLchar *fragment_path)
        {
            string vertex_data;
            string fragment_data;
            
            // open the shader file
            ifstream vertex_shader(vertex_path, ios::in);
            ifstream fragment_shader(fragment_path, ios::in);
            vertex_shader.exceptions(ifstream::failbit | ifstream::badbit);
            fragment_shader.exceptions(ifstream::failbit | ifstream::badbit);

            if (vertex_shader.is_open()) {
                stringstream sstr;
                sstr << vertex_shader.rdbuf();
                vertex_data = sstr.str();
                vertex_shader.close();
            } else {
                throw invalid_argument("Fail to open vertex shader");
            }
            if (fragment_shader.is_open()) {
                stringstream sstr;
                sstr << fragment_shader.rdbuf();
                fragment_data = sstr.str();
                fragment_shader.close();
            } else {
                throw invalid_argument("Fail to open fragment shader");
            }

            // convert string format data into GLchar format
            const GLchar *vertex_shader_data = vertex_data.c_str();
            const GLchar *fragment_shader_data = fragment_data.c_str();  

            GLint flag;                                                // create compile flag
            GLchar infoLog[512];    

            // vertex shader part
            vertex = glCreateShader(GL_VERTEX_SHADER);                // create vertex shader object
            glShaderSource(vertex, 1, &vertex_shader_data, NULL);    // pass the vertex shader data into object
            glCompileShader(vertex);                                // compile vertex shader
            glGetShaderiv(vertex, GL_COMPILE_STATUS, &flag);        // get the compile status    
            if(!flag)
            {
                glGetShaderInfoLog(vertex, 512, NULL, infoLog);
                std::cerr << &infoLog[0] << std::endl;
                std::cerr << &infoLog[1] << std::endl;
                std::cerr << &infoLog[2] << std::endl;
                std::cerr << &infoLog[3] << std::endl;
                throw invalid_argument("Fail to compile vertex shader");
            }

            // fragment shader part
            fragment = glCreateShader(GL_FRAGMENT_SHADER);                // create fragment shader object
            glShaderSource(fragment, 1, &fragment_shader_data, NULL);   // pass the fragment shader data into object
            glCompileShader(fragment);                                    // compile fragment shader
            glGetShaderiv(fragment, GL_COMPILE_STATUS, &flag);            // get the compile status
            if(!flag)
            {
                glGetShaderInfoLog(fragment, 512, NULL, infoLog);
                throw invalid_argument("Fail to compile fragment shader");
            }

            // shader program
            program = glCreateProgram();
            glAttachShader(program, vertex);
            glAttachShader(program, fragment);
            glLinkProgram(program);
            glGetProgramiv(program, GL_LINK_STATUS, &flag);
            if(!flag)
            {
                glGetProgramInfoLog(this->program, 512, NULL, infoLog);
                throw invalid_argument("Fail to link shader program");
            }
            // delete shader object as already linked into program
            glDeleteShader(vertex);        
            glDeleteShader(fragment);
        }
};