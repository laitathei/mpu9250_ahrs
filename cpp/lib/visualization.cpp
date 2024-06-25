#include "visualization.h"

void imu_viewer::run(MPU9250& imu)
{
    shader cube_shader("../shader/cube_vertex.glsl", "../shader/cube_fragment.glsl"); // load cube shader
    shader text_shader("../shader/text_vertex.glsl", "../shader/text_fragment.glsl"); // load text shader

    const double fps = 1.0 / this->hz; // fps limitation
    double last_time = glfwGetTime();

    while (!glfwWindowShouldClose(this->window))
    {
        double current_time = glfwGetTime();
        if (current_time - last_time >= fps) {
            last_time = current_time;

            // render and clear color buff
            glClearColor(0.0f, 0.0f, 0.0f, 1.0f); // set background color as black
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            // render cube
            this->cube_display(cube_shader, imu);

            // round imu data to 2 decimal place
            string ax = round_to_string(imu.ax, 2);
            string ay = round_to_string(imu.ay, 2);
            string az = round_to_string(imu.az, 2);

            string gx = round_to_string(imu.gx, 2);
            string gy = round_to_string(imu.gy, 2);
            string gz = round_to_string(imu.gz, 2);
            
            string mx = round_to_string(imu.mx, 2);
            string my = round_to_string(imu.my, 2);
            string mz = round_to_string(imu.mz, 2);

            string roll = round_to_string(imu.roll, 2);
            string pitch = round_to_string(imu.pitch, 2);
            string yaw = round_to_string(imu.yaw, 2);

            string w = round_to_string(imu.w, 2);
            string x = round_to_string(imu.x, 2);
            string y = round_to_string(imu.y, 2);
            string z = round_to_string(imu.z, 2);

            // render text
            // display acceleration
            this->message_display(text_shader, "Acc (m/s^2):", this->width*0.0f, this->height-15.0f, 0.35f, vec3(1.0f, 1.0f, 1.0f));
            this->message_display(text_shader, "ax: "+ax, this->width*0.0f, this->height-35.0f, 0.35f, vec3(1.0f, 1.0f, 1.0f));
            this->message_display(text_shader, "ay: "+ay, this->width*0.0f, this->height-55.0f, 0.35f, vec3(1.0f, 1.0f, 1.0f));
            this->message_display(text_shader, "az: "+az, this->width*0.0f, this->height-75.0f, 0.35f, vec3(1.0f, 1.0f, 1.0f));

            // display angular velocity
            this->message_display(text_shader, "Gyro (rad/s):", this->width*0.1f, this->height-15.0f, 0.35f, vec3(1.0f, 1.0f, 1.0f));
            this->message_display(text_shader, "gx: "+gx, this->width*0.1f, this->height-35.0f, 0.35f, vec3(1.0f, 1.0f, 1.0f));
            this->message_display(text_shader, "gy: "+gy, this->width*0.1f, this->height-55.0f, 0.35f, vec3(1.0f, 1.0f, 1.0f));
            this->message_display(text_shader, "gz: "+gz, this->width*0.1f, this->height-70.0f, 0.35f, vec3(1.0f, 1.0f, 1.0f));

            // display magnetic field
            this->message_display(text_shader, "Mag (µT):", this->width*0.2f, this->height-15.0f, 0.35f, vec3(1.0f, 1.0f, 1.0f));
            this->message_display(text_shader, "mx: "+mx, this->width*0.2f, this->height-35.0f, 0.35f, vec3(1.0f, 1.0f, 1.0f));
            this->message_display(text_shader, "my: "+my, this->width*0.2f, this->height-55.0f, 0.35f, vec3(1.0f, 1.0f, 1.0f));
            this->message_display(text_shader, "mz: "+mz, this->width*0.2f, this->height-75.0f, 0.35f, vec3(1.0f, 1.0f, 1.0f));

            // display body and navigation frame setting
            this->message_display(text_shader, "Body frame: "+imu.body_frame, this->width*0.3f, this->height-15.0f, 0.35f, vec3(1.0f, 1.0f, 1.0f));
            this->message_display(text_shader, "Navigation frame: "+imu.nav_frame, this->width*0.3f, this->height-35.0f, 0.35f, vec3(1.0f, 1.0f, 1.0f));

            // display quaternion
            this->message_display(text_shader, "Quaternion:", this->width*0.7f, this->height-15.0f, 0.35f, vec3(1.0f, 1.0f, 1.0f));
            this->message_display(text_shader, "w: "+w, this->width*0.7f, this->height-35.0f, 0.35f, vec3(1.0f, 1.0f, 1.0f));
            this->message_display(text_shader, "x: "+x, this->width*0.7f, this->height-55.0f, 0.35f, vec3(1.0f, 1.0f, 1.0f));
            this->message_display(text_shader, "y: "+y, this->width*0.7f, this->height-75.0f, 0.35f, vec3(1.0f, 1.0f, 1.0f));
            this->message_display(text_shader, "z: "+z, this->width*0.7f, this->height-90.0f, 0.35f, vec3(1.0f, 1.0f, 1.0f));

            // display euler angle
            this->message_display(text_shader, "Euler Angles (Degree):", this->width*0.8f, this->height-15.0f, 0.35f, vec3(1.0f, 1.0f, 1.0f));
            if (this->nav_frame == "ENU"){ // zxy
                this->message_display(text_shader, "Roll: "+roll+" ("+static_cast<char>(toupper(imu.rotation_seq[1]))+"-axis) ", this->width*0.8f, this->height-35.0f, 0.35f, vec3(1.0f, 1.0f, 1.0f));
                this->message_display(text_shader, "Pitch: "+pitch+" ("+static_cast<char>(toupper(imu.rotation_seq[2]))+"-axis) ", this->width*0.8f, this->height-55.0f, 0.35f, vec3(1.0f, 1.0f, 1.0f));
                this->message_display(text_shader, "Yaw: "+yaw+" ("+static_cast<char>(toupper(imu.rotation_seq[0]))+"-axis) ", this->width*0.8f, this->height-75.0f, 0.35f, vec3(1.0f, 1.0f, 1.0f));
            }
            else if (this->nav_frame == "NED"){ // zyx
                this->message_display(text_shader, "Roll: "+roll+" ("+static_cast<char>(toupper(imu.rotation_seq[2]))+"-axis) ", this->width*0.8f, this->height-35.0f, 0.35f, vec3(1.0f, 1.0f, 1.0f));
                this->message_display(text_shader, "Pitch: "+pitch+" ("+static_cast<char>(toupper(imu.rotation_seq[1]))+"-axis) ", this->width*0.8f, this->height-55.0f, 0.35f, vec3(1.0f, 1.0f, 1.0f));
                this->message_display(text_shader, "Yaw: "+yaw+" ("+static_cast<char>(toupper(imu.rotation_seq[0]))+"-axis) ", this->width*0.8f, this->height-75.0f, 0.35f, vec3(1.0f, 1.0f, 1.0f));
            }
            // display observation message
            this->message_display(text_shader, "Please observe the imu from top view", this->width*0.375f, 15.0f, 0.35f, vec3(1.0f, 1.0f, 1.0f));

            // swap the buffer data
            glfwSwapBuffers(this->window);
            glfwPollEvents();
        }
    }

    // release resource
    glDeleteVertexArrays(1, &this->cube_VAO);	
    glDeleteBuffers(1, &this->cube_VBO);
    glDeleteVertexArrays(1, &this->text_VAO);	
    glDeleteBuffers(1, &this->text_VBO);
    glfwTerminate();
}

GLfloat* imu_viewer::initialize_cube(float max_x, float min_x, float max_y, float min_y, float max_z, float min_z)
{
    GLfloat* vertex_array = new GLfloat[216];
    if (this->nav_frame == "ENU"){
        GLfloat temp_vertex_array[] = {
            // position xyz              // color
            // down side(Yellow)(-Z)
            this->min_x, this->min_y, this->max_z,  1.0f, 1.0f, 0.0f,
            this->max_x, this->min_y, this->max_z,  1.0f, 1.0f, 0.0f,
            this->max_x, this->max_y, this->max_z,  1.0f, 1.0f, 0.0f,
            this->max_x, this->max_y, this->max_z,  1.0f, 1.0f, 0.0f,
            this->min_x, this->max_y, this->max_z,  1.0f, 1.0f, 0.0f,
            this->min_x, this->min_y, this->max_z,  1.0f, 1.0f, 0.0f,

            // upper side(Blue)(+Z)
            this->min_x, this->min_y, this->min_z,  0.0f, 0.0f, 1.0f,
            this->max_x, this->min_y, this->min_z,  0.0f, 0.0f, 1.0f,
            this->max_x, this->max_y, this->min_z,  0.0f, 0.0f, 1.0f,
            this->max_x, this->max_y, this->min_z,  0.0f, 0.0f, 1.0f,
            this->min_x, this->max_y, this->min_z,  0.0f, 0.0f, 1.0f,
            this->min_x, this->min_y, this->min_z,  0.0f, 0.0f, 1.0f,

            // left side(Cyan)(-X)
            this->min_x, this->max_y, this->max_z,  0.0f, 1.0f, 1.0f,
            this->min_x, this->max_y, this->min_z,  0.0f, 1.0f, 1.0f,
            this->min_x, this->min_y, this->min_z,  0.0f, 1.0f, 1.0f,
            this->min_x, this->min_y, this->min_z,  0.0f, 1.0f, 1.0f,
            this->min_x, this->min_y, this->max_z,  0.0f, 1.0f, 1.0f,
            this->min_x, this->max_y, this->max_z,  0.0f, 1.0f, 1.0f,

            // right side(Red)(+X)
            this->max_x, this->max_y, this->max_z,  1.0f, 0.0f, 0.0f,
            this->max_x, this->max_y, this->min_z,  1.0f, 0.0f, 0.0f,
            this->max_x, this->min_y, this->min_z,  1.0f, 0.0f, 0.0f,
            this->max_x, this->min_y, this->min_z,  1.0f, 0.0f, 0.0f,
            this->max_x, this->min_y, this->max_z,  1.0f, 0.0f, 0.0f,
            this->max_x, this->max_y, this->max_z,  1.0f, 0.0f, 0.0f,

            // front side(Green)(+Y)
            this->min_x, this->max_y, this->min_z,  0.0f, 1.0f, 0.0f,
            this->max_x, this->max_y, this->min_z,  0.0f, 1.0f, 0.0f,
            this->max_x, this->max_y, this->max_z,  0.0f, 1.0f, 0.0f,
            this->max_x, this->max_y, this->max_z,  0.0f, 1.0f, 0.0f,
            this->min_x, this->max_y, this->max_z,  0.0f, 1.0f, 0.0f,
            this->min_x, this->max_y, this->min_z,  0.0f, 1.0f, 0.0f,

            // back side(Magenta)(-Y)
            this->min_x, this->min_y, this->min_z,  1.0f, 0.0f, 1.0f,
            this->max_x, this->min_y, this->min_z,  1.0f, 0.0f, 1.0f,
            this->max_x, this->min_y, this->max_z,  1.0f, 0.0f, 1.0f,
            this->max_x, this->min_y, this->max_z,  1.0f, 0.0f, 1.0f,
            this->min_x, this->min_y, this->max_z,  1.0f, 0.0f, 1.0f,
            this->min_x, this->min_y, this->min_z,  1.0f, 0.0f, 1.0f
        };
        int n = sizeof(temp_vertex_array) / sizeof(temp_vertex_array[0]); // calculate the length of temp array
        this->vertex_size = sizeof(temp_vertex_array);
        copy(temp_vertex_array, temp_vertex_array + n, vertex_array); // copy the temp array to real array
    }
    else if (this->nav_frame == "NED"){
        GLfloat temp_vertex_array[] = {
            // position xyz              // color rgb
            // upper side(Yellow)(+Z)
            this->min_x, this->min_y, this->max_z,  0.0f, 0.0f, 1.0f,
            this->max_x, this->min_y, this->max_z,  0.0f, 0.0f, 1.0f,
            this->max_x, this->max_y, this->max_z,  0.0f, 0.0f, 1.0f,
            this->max_x, this->max_y, this->max_z,  0.0f, 0.0f, 1.0f,
            this->min_x, this->max_y, this->max_z,  0.0f, 0.0f, 1.0f,
            this->min_x, this->min_y, this->max_z,  0.0f, 0.0f, 1.0f,

            // down side(Blue)(-Z)
            this->min_x, this->min_y, this->min_z,  1.0f, 1.0f, 0.0f,
            this->max_x, this->min_y, this->min_z,  1.0f, 1.0f, 0.0f,
            this->max_x, this->max_y, this->min_z,  1.0f, 1.0f, 0.0f,
            this->max_x, this->max_y, this->min_z,  1.0f, 1.0f, 0.0f,
            this->min_x, this->max_y, this->min_z,  1.0f, 1.0f, 0.0f,
            this->min_x, this->min_y, this->min_z,  1.0f, 1.0f, 0.0f,

            // left side(Magenta)(-Y)
            this->min_x, this->max_y, this->max_z,  1.0f, 0.0f, 1.0f,
            this->min_x, this->max_y, this->min_z,  1.0f, 0.0f, 1.0f,
            this->min_x, this->min_y, this->min_z,  1.0f, 0.0f, 1.0f,
            this->min_x, this->min_y, this->min_z,  1.0f, 0.0f, 1.0f,
            this->min_x, this->min_y, this->max_z,  1.0f, 0.0f, 1.0f,
            this->min_x, this->max_y, this->max_z,  1.0f, 0.0f, 1.0f,

            // right side(Green)(+Y)
            this->max_x, this->max_y, this->max_z,  0.0f, 1.0f, 0.0f,
            this->max_x, this->max_y, this->min_z,  0.0f, 1.0f, 0.0f,
            this->max_x, this->min_y, this->min_z,  0.0f, 1.0f, 0.0f,
            this->max_x, this->min_y, this->min_z,  0.0f, 1.0f, 0.0f,
            this->max_x, this->min_y, this->max_z,  0.0f, 1.0f, 0.0f,
            this->max_x, this->max_y, this->max_z,  0.0f, 1.0f, 0.0f,

            // front side(Red)(+X)
            this->min_x, this->max_y, this->min_z,  1.0f, 0.0f, 0.0f,
            this->max_x, this->max_y, this->min_z,  1.0f, 0.0f, 0.0f,
            this->max_x, this->max_y, this->max_z,  1.0f, 0.0f, 0.0f,
            this->max_x, this->max_y, this->max_z,  1.0f, 0.0f, 0.0f,
            this->min_x, this->max_y, this->max_z,  1.0f, 0.0f, 0.0f,
            this->min_x, this->max_y, this->min_z,  1.0f, 0.0f, 0.0f,

            // back side(Cyan)(-X)
            this->min_x, this->min_y, this->min_z,  0.0f, 1.0f, 1.0f,
            this->max_x, this->min_y, this->min_z,  0.0f, 1.0f, 1.0f,
            this->max_x, this->min_y, this->max_z,  0.0f, 1.0f, 1.0f,
            this->max_x, this->min_y, this->max_z,  0.0f, 1.0f, 1.0f,
            this->min_x, this->min_y, this->max_z,  0.0f, 1.0f, 1.0f,
            this->min_x, this->min_y, this->min_z,  0.0f, 1.0f, 1.0f
        };
        int n = sizeof(temp_vertex_array) / sizeof(temp_vertex_array[0]); // calculate the length of temp array
        this->vertex_size = sizeof(temp_vertex_array);
        copy(temp_vertex_array, temp_vertex_array + n, vertex_array); // copy the temp array to real array
    }
    else{
        throw invalid_argument("Navigation frame should be either ENU or NED");
    }
    return vertex_array;
}

void imu_viewer::initialize_freetype()
{
    // load the FreeType library
    if (FT_Init_FreeType(&this->ft))
        throw invalid_argument("Could not init FreeType Library");

    // load the custom font file
    if (FT_New_Face(this->ft, "../font/arial.ttf", 0, &this->face))
        throw invalid_argument("Failed to load font");

    FT_Set_Pixel_Sizes(this->face, 0, 48); // set the height of character is 48 pixel size
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1); // remove character alignment restrictions

    // load and render 128 ASCII character
    for (GLubyte c = 0; c < 128; c++) {
        if (FT_Load_Char(this->face, c, FT_LOAD_RENDER)) {
            cerr << "Failed to load ASCII character: " << c << endl;
            continue;
        }
        GLuint texture; // create texture object
        glGenTextures(1, &texture); // store the character texture id
        glBindTexture(GL_TEXTURE_2D, texture); // bind current texture object
        glTexImage2D(
            GL_TEXTURE_2D,
            0,
            GL_RED,
            this->face->glyph->bitmap.width,
            this->face->glyph->bitmap.rows,
            0,
            GL_RED,
            GL_UNSIGNED_BYTE,
            this->face->glyph->bitmap.buffer
        ); 

        // create texture to the current texture object with bitmap
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        // stores texture ID, width, height, bearing and horizontal displacement of the character
        character current_character = {
            texture,
            this->face->glyph->bitmap.width,
            this->face->glyph->bitmap.rows,
            this->face->glyph->bitmap_left,
            this->face->glyph->bitmap_top,
            static_cast<unsigned int>(this->face->glyph->advance.x)
        };
        this->characters.insert(pair<GLchar, character>(c, current_character));
    }
    FT_Done_Face(this->face);
    FT_Done_FreeType(this->ft);

    // setup text_VAO object
    glGenVertexArrays(1, &this->text_VAO); // create 1 VAO object
    glBindVertexArray(this->text_VAO); // bind text_VAO as current working VAO object

    // setup text_VBO object
    glGenBuffers(1, &this->text_VBO);  // create 1 VBO object
    glBindBuffer(GL_ARRAY_BUFFER, this->text_VBO); // bind text_VBO as current working VBO object
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 6 * 4, nullptr, GL_DYNAMIC_DRAW); // put vertex_array to buffer (VBO object)

    // setup vertex vertex(position+vertex) attribute pointer
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(GLfloat), 0); // refers to text_vertex.glsl location = 0
    glEnableVertexAttribArray(0);	// turn on channel 0

    // unlink text_VAO and text_VBO
    glBindBuffer(GL_ARRAY_BUFFER, 0); // unlink text_VBO
    glBindVertexArray(0); // unlink text_VAO
}

void imu_viewer::initialize_glfw(GLfloat* vertex_array)
{
    glfwInit();
    if (!glfwInit()) {
        throw invalid_argument("Failed to initialize GLFW");
    }

    // create opengl window via glfw library
    this->window = glfwCreateWindow(this->width, this->height, "IMU attitude display with euler angle and quaternion", NULL, NULL);
    if (!this->window) {
        glfwTerminate();
        throw invalid_argument("Failed to open GLFW window");
    }
    glfwGetFramebufferSize(this->window, &this->width, &this->height);
    glfwMakeContextCurrent(this->window);

    glewInit();
    if (glewInit() != GLEW_OK) {
        throw invalid_argument("Failed to initialize GLEW");
    }

    // Define the viewport dimensions
    glViewport(0, 0, this->width, this->height);

    // Set OpenGL options
    // glEnable(GL_CULL_FACE); // Remove the unseen face
    glEnable(GL_BLEND); // new color render color blend with old color to achieve transparency
    glEnable(GL_DEPTH_TEST); // check fragment depth z axis value before rendering
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // setup cube_VAO object
    glGenVertexArrays(1, &this->cube_VAO); // create 1 VAO object
    glBindVertexArray(this->cube_VAO); // bind cube_VAO as current working VAO object

    // setup cube_VBO object
    glGenBuffers(1, &this->cube_VBO); // create 1 VBO object
    glBindBuffer(GL_ARRAY_BUFFER, this->cube_VBO); // bind cube_VBO as current working VBO object
    glBufferData(GL_ARRAY_BUFFER, this->vertex_size, vertex_array, GL_STATIC_DRAW); // put vertex_array to buffer (VBO object)

    // setup vertex position attribute pointer
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(GLfloat), (GLvoid*)0); // refers to cube_vertex.glsl location = 0
    glEnableVertexAttribArray(0); // turn on channel 0

    // setup vertex color attribute pointer
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(GLfloat), (GLvoid*)(3*sizeof(GLfloat))); // refers to cube_vertex.glsl location = 1
    glEnableVertexAttribArray(1); // turn on channel 1

    // unlink cube_VAO and cube_VBO
    glBindBuffer(GL_ARRAY_BUFFER, 0); // unlink VBO
    glBindVertexArray(0); // unlink VAO
}

void imu_viewer::cube_display(shader cube_shader, MPU9250& imu)
{
    // start shader program
    glUseProgram(cube_shader.program);

    // create rotation matrix
    mat4 orientation = mat4(1.0f);

    if (this->rotation_seq == "zxy"){
        orientation = rotate(orientation, radians(imu.yaw), vec3(0.0f, 0.0f, 1.0f)); // z axis rotation
        orientation = rotate(orientation, -radians(imu.roll), vec3(1.0f, 0.0f, 0.0f)); // x axis rotation
        orientation = rotate(orientation, -radians(imu.pitch), vec3(0.0f, 1.0f, 0.0f)); // y axis rotation
    }
    else if (this->rotation_seq == "zyx"){
        orientation = rotate(orientation, -radians(imu.yaw), vec3(0.0f, 0.0f, 1.0f)); // z axis rotation
        orientation = rotate(orientation, -radians(imu.roll), vec3(0.0f, 1.0f, 0.0f)); // y axis rotation
        orientation = rotate(orientation, -radians(imu.pitch), vec3(1.0f, 0.0f, 0.0f)); // x axis rotation
    }
    int orientation_location = glGetUniformLocation(cube_shader.program, "orientation");
    glUniformMatrix4fv(orientation_location, 1, GL_FALSE, value_ptr(orientation));

    // paint the rotation cube
    glBindVertexArray(this->cube_VAO); // bind cube_VAO as current working VAO object
    glDrawArrays(GL_TRIANGLES, 0, 36);
    glBindVertexArray(0); // unlink cube_VAO
}

void imu_viewer::message_display(shader text_shader, string text, GLfloat x, GLfloat y, GLfloat scale, vec3 color)
{
    // start shader program
    glUseProgram(text_shader.program);
    glBindVertexArray(this->text_VAO); // bind text_VAO as current working VAO object
    mat4 projection = ortho(0.0f, static_cast<GLfloat>(this->width), 0.0f, static_cast<GLfloat>(this->height));
    glUniformMatrix4fv(glGetUniformLocation(text_shader.program, "projection"), 1, GL_FALSE, value_ptr(projection));
    glUniform3f(glGetUniformLocation(text_shader.program, "text_color"), color.x, color.y, color.z);
    glActiveTexture(GL_TEXTURE0); // activate textrue 0

    // iterate through each character in a string
    for (auto c : text) {
        character ch = characters[c];

        GLfloat pos_x = x + ch.bearing_x * scale; // calculate the current character left top x-axis position
        GLfloat pos_y = y - (ch.height - ch.bearing_y) * scale; // calculate the current character left top y-axis position

        GLfloat w = ch.width * scale; // calculate width
        GLfloat h = ch.height * scale; // calculate height

        // Define 6 vertex (two triangles) to form rectangle
        GLfloat vertex[6][4] = {
            // position xy          render position uv
            { pos_x,     pos_y + h,   0.0, 0.0 },
            { pos_x,     pos_y,       0.0, 1.0 },
            { pos_x + w, pos_y,       1.0, 1.0 },

            { pos_x,     pos_y + h,   0.0, 0.0 },
            { pos_x + w, pos_y,       1.0, 1.0 },
            { pos_x + w, pos_y + h,   1.0, 0.0 }
        };

        // render glyph texture over quad
        glBindTexture(GL_TEXTURE_2D, ch.texture_id);

        // update content of VBO memory
        glBindBuffer(GL_ARRAY_BUFFER, this->text_VBO); // bind the VBO
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vertex), vertex); // send the vertex data to VBO
        glBindBuffer(GL_ARRAY_BUFFER, 0); // unbind the VBO 

        // Render quad
        glDrawArrays(GL_TRIANGLES, 0, 6); // create two triangles by 6 vertex

        x += (ch.advance >> 6) * scale; // move horizontally to next character position
    }
    glBindTexture(GL_TEXTURE_2D, 0); // unbind the texture
    glBindVertexArray(0); // unbind the text_VAO
}