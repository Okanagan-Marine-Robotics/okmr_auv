#include <GLFW/glfw3.h>
#include <iostream>
#include <cmath>
#include "bonxai/bonxai.hpp"
#include "bonxai/serialization.hpp"

#include "rclcpp/rclcpp.hpp"
#include "okmr_msgs/msg/voxel_grid.hpp"
#include "okmr_msgs/msg/classes.hpp"
#include "voxelData.hpp"
#include <sstream>
#include <vector>

float voxel_resolution=0;//update this to adapt to the grid
Bonxai::VoxelGrid<voxelData> grid = Bonxai::VoxelGrid<voxelData>(0);
std::shared_ptr<rclcpp::Node> node;

// Camera variables
GLfloat cameraPositionX = 0.0f;
GLfloat cameraPositionY = 0.0f;
GLfloat cameraPositionZ = 0.0f;

GLfloat cameraYaw = 0.0f;   // Yaw angle (rotation around the Y-axis)
GLfloat cameraPitch = 0.0f; // Pitch angle (rotation around the X-axis)
float cameraSpeed = 0.1f;
float sensitivity = 0.001f;

float degreesToRadians(float degrees) {
    return degrees * static_cast<float>(M_PI / 180.0);
}

// Function to handle keyboard input for camera movement
void processInput(GLFWwindow* window) {
    // Camera movement speed
    float cameraSpeed = 0.1f;

// Calculate movement direction based on yaw angle
float dx = sin(cameraYaw);
float dz = cos(cameraYaw);

if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) {
    cameraPositionX += dx * cameraSpeed;
    cameraPositionZ += dz * cameraSpeed;
}
if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) {
    cameraPositionX -= dx * cameraSpeed;
    cameraPositionZ -= dz * cameraSpeed;
}
if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) {
    cameraPositionX += dz * cameraSpeed;
    cameraPositionZ -= dx * cameraSpeed;
}
if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) {
    cameraPositionX -= dz * cameraSpeed;
    cameraPositionZ += dx * cameraSpeed;
}
if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS)
    cameraPositionY += cameraSpeed;
if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS)
    cameraPositionY -= cameraSpeed;
    }


void hsv_to_rgb(float h, float s, float v, float &r, float &g, float &b) {
    float c = v * s;
    float x = c * (1 - std::fabs(fmod(h / 60.0, 2) - 1));
    float m = v - c;

    if (h >= 0 && h < 60) {
        r = c, g = x, b = 0;
    } else if (h >= 60 && h < 120) {
        r = x, g = c, b = 0;
    } else if (h >= 120 && h < 180) {
        r = 0, g = c, b = x;
    } else if (h >= 180 && h < 240) {
        r = 0, g = x, b = c;
    } else if (h >= 240 && h < 300) {
        r = x, g = 0, b = c;
    } else {
        r = c, g = 0, b = x;
    }

    r += m;
    g += m;
    b += m;
}
// Function to draw a single voxel
void drawVoxel(float x, float y, float z, float size, voxelData data) {
    float r=data.r/255.,g=data.g/255.,b=data.b/255.,a=1.0,h,s,v;
    if(data.class_id==1){
        r=1.0;g=0.0;b=0.0;
    }
    if(data.class_id==2){
        r=0.0;g=1.0;b=0.0;
    }
    //a=data.confidence/100.0;

    //float dist = sqrt(x*x + y*y + z*z);

    //h = (int)(dist*30) % 360;
    //s=1.0;
    //v=1.0;
    //hsv_to_rgb(h,s,v,r,g,b);
    /*
    switch(data.class_id){//change class to be integer instead of float
        case okmr_msgs::msg::Classes::OBSTACLE:
            r=0.0;
            g=1.0;
            b=0.5;
        break;
        case okmr_msgs::msg::Classes::PATH:
            r=1.0;
            g=0.0;
            b=0.0;
        break;
        case okmr_msgs::msg::Classes::CHECKPOINT:
            r=0.5;
            g=0.0;
            b=0.5;
        break;
        case okmr_msgs::msg::Classes::INFLATED:
            r=0.0;
            g=0.5;
            b=1.0;
            //return;//dont want to draw inflated region
            break;
        case okmr_msgs::msg::Classes::GATE:
            r=1.0;
            g=0.67;
            b=0.0;
            //return;//dont want to draw inflated region
            break;
        default:
            return;//if the class is anything we dont recognize for some reason, dont display it
            break;
    }
    */

    glColor4f(r, g, b,a);
    glBegin(GL_QUADS);
    // Front face
    glVertex3f(x - size / 2, y - size / 2, z + size / 2);
    glVertex3f(x + size / 2, y - size / 2, z + size / 2);
    glVertex3f(x + size / 2, y + size / 2, z + size / 2);
    glVertex3f(x - size / 2, y + size / 2, z + size / 2);
    // Back face
    glVertex3f(x - size / 2, y - size / 2, z - size / 2);
    glVertex3f(x + size / 2, y - size / 2, z - size / 2);
    glVertex3f(x + size / 2, y + size / 2, z - size / 2);
    glVertex3f(x - size / 2, y + size / 2, z - size / 2);
    // Top face
    glVertex3f(x - size / 2, y + size / 2, z + size / 2);
    glVertex3f(x + size / 2, y + size / 2, z + size / 2);
    glVertex3f(x + size / 2, y + size / 2, z - size / 2);
    glVertex3f(x - size / 2, y + size / 2, z - size / 2);
    // Bottom face
    glVertex3f(x - size / 2, y - size / 2, z + size / 2);
    glVertex3f(x + size / 2, y - size / 2, z + size / 2);
    glVertex3f(x + size / 2, y - size / 2, z - size / 2);
    glVertex3f(x - size / 2, y - size / 2, z - size / 2);
    // Right face
    glVertex3f(x + size / 2, y - size / 2, z + size / 2);
    glVertex3f(x + size / 2, y - size / 2, z - size / 2);
    glVertex3f(x + size / 2, y + size / 2, z - size / 2);
    glVertex3f(x + size / 2, y + size / 2, z + size / 2);
    // Left face
    glVertex3f(x - size / 2, y - size / 2, z + size / 2);
    glVertex3f(x - size / 2, y - size / 2, z - size / 2);
    glVertex3f(x - size / 2, y + size / 2, z - size / 2);
    glVertex3f(x - size / 2, y + size / 2, z + size / 2);
    glEnd();

    glColor4f(0.0,0.0,0.5,0.1);
    float thickness = 1.0f; // Set your desired thickness here
    glLineWidth(thickness);
    glBegin(GL_LINES);
    // Front face
    glVertex3f(x - size / 2, y - size / 2, z + size / 2);
    glVertex3f(x + size / 2, y - size / 2, z + size / 2);

    glVertex3f(x + size / 2, y - size / 2, z + size / 2);
    glVertex3f(x + size / 2, y + size / 2, z + size / 2);

    glVertex3f(x + size / 2, y + size / 2, z + size / 2);
    glVertex3f(x - size / 2, y + size / 2, z + size / 2);

    glVertex3f(x - size / 2, y + size / 2, z + size / 2);
    glVertex3f(x - size / 2, y - size / 2, z + size / 2);
    // Back face
    glVertex3f(x - size / 2, y - size / 2, z - size / 2);
    glVertex3f(x + size / 2, y - size / 2, z - size / 2);

    glVertex3f(x + size / 2, y - size / 2, z - size / 2);
    glVertex3f(x + size / 2, y + size / 2, z - size / 2);

    glVertex3f(x + size / 2, y + size / 2, z - size / 2);
    glVertex3f(x - size / 2, y + size / 2, z - size / 2);

    glVertex3f(x - size / 2, y + size / 2, z - size / 2);
    glVertex3f(x - size / 2, y - size / 2, z - size / 2);
    // Connect front and back faces
    glVertex3f(x - size / 2, y - size / 2, z + size / 2);
    glVertex3f(x - size / 2, y - size / 2, z - size / 2);

    glVertex3f(x + size / 2, y - size / 2, z + size / 2);
    glVertex3f(x + size / 2, y - size / 2, z - size / 2);

    glVertex3f(x + size / 2, y + size / 2, z + size / 2);
    glVertex3f(x + size / 2, y + size / 2, z - size / 2);

    glVertex3f(x - size / 2, y + size / 2, z + size / 2);
    glVertex3f(x - size / 2, y + size / 2, z - size / 2);
    glEnd();
}

// Function to render the voxel grid
void renderVoxel(const voxelData& data, const Bonxai::CoordT& coord){
    Bonxai::Point3D pos = grid.coordToPos(coord);
    drawVoxel(pos.y,pos.z,pos.x,grid.resolution,data);
}

void mouseCallback(GLFWwindow* window, double xpos, double ypos) {
    static double lastX = xpos;
    static double lastY = ypos;

    double deltaX = xpos - lastX;
    double deltaY = ypos - lastY;
    lastX = xpos;
    lastY = ypos;

    cameraYaw -= deltaX * sensitivity;
    cameraPitch += deltaY * sensitivity;

    // Clamp pitch to avoid flipping
    if (cameraPitch > 89.0f) cameraPitch = 89.0f;
    if (cameraPitch < -89.0f) cameraPitch = -89.0f;
}// Function to set up the view matrix

void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
        glfwSetCursorPosCallback(window, nullptr);
    }
}

void setupCamera() {
    // Calculate the camera's direction vector
    GLfloat directionX = cos(cameraPitch) * sin(cameraYaw);
    GLfloat directionY = sin(cameraPitch);
    GLfloat directionZ = cos(cameraPitch) * cos(cameraYaw);

    // Calculate the right vector
    GLfloat rightX = sin(cameraYaw - M_PI/2.0f);
    GLfloat rightY = 0.0f;
    GLfloat rightZ = cos(cameraYaw - M_PI/2.0f);

    // Calculate the up vector
    GLfloat upX = directionY * rightZ - directionZ * rightY;
    GLfloat upY = directionZ * rightX - directionX * rightZ;
    GLfloat upZ = directionX * rightY - directionY * rightX;

    // Set up the view matrix
    GLfloat viewMatrix[16] = {
        rightX, upX, -directionX, 0.0f,
        rightY, upY, -directionY, 0.0f,
        rightZ, upZ, -directionZ, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f
    };

    // Load the view matrix
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glMultMatrixf(viewMatrix);

    // Translate the camera to its position
    glTranslatef(-cameraPositionX, cameraPositionY, -cameraPositionZ);
}

// Function to set up the projection matrix
void setupProjection(int width, int height) {
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    float aspectRatio = static_cast<float>(width) / static_cast<float>(height);
    float fov = 45.0f;
    float nearPlane = 0.1f; // Adjust this value
    float farPlane = 100.0f; // Adjust this value
    float top = tan(fov * M_PI / 360.0f) * nearPlane;
    float bottom = -top;
    float right = aspectRatio * top;
    float left = -right;
    glFrustum(left, right, bottom, top, nearPlane, farPlane);
    glMatrixMode(GL_MODELVIEW);
}

// Cross product helper function
GLfloat cross(GLfloat Ax, GLfloat Ay, GLfloat Az, GLfloat Bx, GLfloat By, GLfloat Bz) {
    return Ay * Bz - Az * By, Az * Bx - Ax * Bz, Ax * By - Ay * Bx;
}
void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
        glfwSetCursorPosCallback(window, mouseCallback);
    } 
}
bool loading=false;

void updateGridFromMsg(const okmr_msgs::msg::VoxelGrid &msg){
    if(loading)return;
    loading=true;
    std::string serialized_data(msg.data.begin(), msg.data.end());

    std::istringstream ifile(serialized_data, std::ios::binary);
    
    char header[256];
    ifile.getline(header, 256);
    Bonxai::HeaderInfo info = Bonxai::GetHeaderInfo(header);
    auto g = Bonxai::Deserialize<voxelData>(ifile, info);
    grid=std::move(g);
    loading=false;
}

void updateGridFromFile(std::string inputFileName){
    std::ifstream inputFile(inputFileName, std::ios::binary);
    if (!inputFile.is_open()) {
        std::cerr << "Error: Unable to open file " << inputFileName << std::endl;
    }

    // Read the header of the file to obtain information about the voxel grid
    try{
        char header[256];
        inputFile.getline(header, 256);
        Bonxai::HeaderInfo info = Bonxai::GetHeaderInfo(header);

        // Deserialize the voxel grid from the file
        auto g = Bonxai::Deserialize<voxelData>(inputFile, info);
        inputFile.close();
        grid=std::move(g);
    }
    catch(...){
        std::cout<<"could not parse bonxai file\n";
    }
}
void renderGridLines(float range) {
    glColor3f(0.5f, 0.5f, 0.5f); // Set grid line color (grey)
    glBegin(GL_LINES);
    // Draw horizontal lines (along X-axis)
    for(float i = -range; i <= range; i += 1.0f) {
        glVertex3f(-range, 0.0f, i);
        glVertex3f(range, 0.0f, i);
    }
    // Draw vertical lines (along Z-axis)
    for(float i = -range; i <= range; i += 1.0f) {
        glVertex3f(i, 0.0f, -range);
        glVertex3f(i, 0.0f, range);
    }
    glEnd();
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    //creates node as shared pointer
    node = rclcpp::Node::make_shared("voxel_visualizer");

    if  (false && argc == 1) {
        std::cerr << "Usage: " << argv[0] << " <input_file> \n Options: --file <file_name.bx> to load from file" << std::endl;
        return -1;
    }
    bool file=false;
    int fileIndex;
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--file") {
            file=true;
            fileIndex=i+1;
        }
    }
    
    rclcpp::Subscription<okmr_msgs::msg::VoxelGrid>::SharedPtr grid_subscription= node->create_subscription<okmr_msgs::msg::VoxelGrid>("/path_grid",10, &updateGridFromMsg);
    if(file)
        updateGridFromFile(argv[fileIndex]);

    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return -1;
    }
    // Create a windowed mode window and its OpenGL context
    GLFWwindow* window = glfwCreateWindow(1000, 1000, "Bonxai Voxel Visualizer", NULL, NULL);
    if (!window) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }

    // Make the window's context current
    glfwMakeContextCurrent(window);
    
    // Enable depth testing
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
    glfwSetCursorPosCallback(window, mouseCallback);
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    glfwSetKeyCallback(window, keyCallback);
    glfwSetMouseButtonCallback(window, mouseButtonCallback);
    // Loop until the user closes the window
    int frameSinceLastUpdate=-1;
    while (!glfwWindowShouldClose(window)) {
        // Process input
        processInput(window);

        // Clear the buffer
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Set up the view matrix
        setupCamera();
        setupProjection(1000,1000);
        if(!file){
            if(frameSinceLastUpdate>10 || frameSinceLastUpdate<0){
                rclcpp::spin_some(node);

                frameSinceLastUpdate=0;
            }
            frameSinceLastUpdate++;
        }

        renderGridLines(100.0);
        // Render the voxel grid
        grid.forEachCell(renderVoxel);

        // Swap front and back buffers
        glfwSwapBuffers(window);

        // Poll for and process events
        glfwPollEvents();
    }

    // Terminate GLFW
    glfwTerminate();
    rclcpp::shutdown();

    return 0;
}
