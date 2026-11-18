//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2003-2024, CHAI3D
    (www.chai3d.org)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of CHAI3D nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

    \author    <http://www.chai3d.org>
    \author    Francois Conti
    \version   3.3.0
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"
#include "chai3d.h"
#include "display/CCamera.h"
#include "graphics/COpenGLHeaders.h"
#include "imgui.h"
#include "math/CMaths.h"
#include "math/CMatrix3d.h"
#include "math/CVector3d.h"
//------------------------------------------------------------------------------
#include <GLFW/glfw3.h>
#include <set>

#include "graphics/HalfEdge.h"
#include "world/CMultiMesh.h"
//------------------------------------------------------------------------------
using namespace chai3d;
using namespace std;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// GENERAL SETTINGS
//------------------------------------------------------------------------------

// convert to resource path
#define RESOURCE_PATH(p) (const char *)((resourceRoot + string(p)).c_str())

// stereo Mode
/*
    C_STEREO_DISABLED:            Stereo is disabled
    C_STEREO_ACTIVE:              Active stereo for OpenGL NVDIA QUADRO cards
    C_STEREO_PASSIVE_LEFT_RIGHT:  Passive stereo where L/R images are rendered
   next to each other C_STEREO_PASSIVE_TOP_BOTTOM:  Passive stereo where L/R
   images are rendered above each other
*/
cStereoMode stereoMode = C_STEREO_DISABLED;

// fullscreen mode
bool fullscreen = false;

// mirrored display
bool mirroredDisplay = false;

bool show_demo_window = true;

//------------------------------------------------------------------------------
// DECLARED VARIABLES
//------------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
cWorld *world;

// a camera to render the world in the window display
cCamera *camera;

// a viewport to display the scene viewed by the camera
cViewport *viewport = nullptr;

// a light source to illuminate the objects in the world
cSpotLight *light;

// a haptic device handler
cHapticDeviceHandler *handler;

// a pointer to the current haptic device
cGenericHapticDevicePtr hapticDevice;

// a virtual tool representing the haptic device in the scene
cToolCursor *tool;


// a colored background
cBackground *background;

// a font for rendering text
cFontPtr font;
// a label to display the rate [Hz] at which the simulation is running
cLabel *labelRates;

// a label to describe application
cLabel *labelDescription;

// a flag that indicates if the haptic simulation is currently running
bool simulationRunning = false;

// a flag that indicates if the haptic simulation has terminated
bool simulationFinished = true;

// a frequency counter to measure the simulation graphic rate
cFrequencyCounter freqCounterGraphics;

// a frequency counter to measure the simulation haptic rate
cFrequencyCounter freqCounterHaptics;

// haptic thread
cThread *hapticsThread;

// a handle to window display context
GLFWwindow *window = nullptr;

// current size of GLFW window
int windowW = 0;
int windowH = 0;

// current size of GLFW framebuffer
int framebufferW = 0;
int framebufferH = 0;

// swap interval for the display context (vertical synchronization)
int swapInterval = 1;

// root resource path
string resourceRoot;

// My stuff

// a virtual mesh like object
// cMultiMesh *objectMM;
// cMesh *object;
// For deformation calculation
MyStuff::PriMoMesh primoMesh;

// Only for rendering purposes
chai3d::cMesh myMesh;

int faceIdx = 0;
bool doDeform = false;

chai3d::cVector3d objectTranslation;
chai3d::cVector3d objectRotation;

std::set<unsigned int> selectedTrianglesIndices, hoveredTrianglesIndices;

cColorf selectColor, unselectColor, hoverColor;

// My temporary stuff
// chai3d::cVector3d faceTranslation;
// chai3d::cVector3d faceRotation;

// My functions

void MyImGuiControlPanel() {
  // 1. Show the big demo window (Most of the sample code is in
  // ImGui::ShowDemoWindow()! You can browse its code to learn more about Dear
  // ImGui!).
  // if (show_demo_window)
  //   ImGui::ShowDemoWindow(&show_demo_window);

  constexpr double tMin = -2.0f;
  constexpr double tMax = 2.0f;
  constexpr double rMin = -3.14f;
  constexpr double rMax = 3.14f;

  ImGui::Begin("Test window");

  ImGui::SliderScalarN("Obj translation", ImGuiDataType_Double,
                       &objectTranslation(0), 3, &tMin, &tMax, "%.3f");
  ImGui::SliderScalarN("Obj rotation", ImGuiDataType_Double, &objectRotation(0),
                       3, &rMin, &rMax, "%.3f");

  ImGui::Checkbox("Deform?", &doDeform);

  ImGui::SliderInt("Face index", &faceIdx, 0, myMesh.getNumTriangles() - 1);

  unsigned int v0 = myMesh.m_triangles->getVertexIndex(faceIdx, 0);
  unsigned int v1 = myMesh.m_triangles->getVertexIndex(faceIdx, 1);
  unsigned int v2 = myMesh.m_triangles->getVertexIndex(faceIdx, 2);

  ImGui::Text("Triangle: %d %d %d", v0, v1, v2);

  ImGui::SliderScalarN("Face translation", ImGuiDataType_Double,
                       &primoMesh.faces[faceIdx].translate(0), 3, &tMin, &tMax,
                       "%.3f");
  ImGui::SliderScalarN("Face rotation", ImGuiDataType_Double,
                       &primoMesh.faces[faceIdx].rotate(0), 3, &rMin, &rMax,
                       "%.3f");

  ImGui::Checkbox("Constrain face", &primoMesh.faces[faceIdx].isConstrain);

  ImGui::End();

  ImGui::Begin("Obj inspection");

  for (unsigned int i = 0; i < myMesh.m_vertices->getNumElements(); i++) {
    auto pos = myMesh.m_vertices->getLocalPos(i);
    ImGui::Text("[%d]: %.3f %.3f %.3f", i, pos(0), pos(1), pos(2));
  }

  ImGui::End();

  ImGui::Begin("Another Primo inspection");

  for (unsigned int i = 0; i < primoMesh.faces.size(); i++) {
    std::array<unsigned int, 3> vids;
    unsigned int x = 0;

    for (auto eid : MyStuff::edges(primoMesh, i)) {
      vids[x++] = primoMesh.edges[eid].source;
    }

    ImGui::Text("[%d]: %d %d %d", i, vids[0], vids[1], vids[2]);
  }

  // for (unsigned int i = 0; i < primoMesh.verts.size());

  ImGui::End();
}

//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

// callback when the window is resized
void onWindowSizeCallback(GLFWwindow *a_window, int a_width, int a_height);

// callback when the window framebuffer is resized
void onFrameBufferSizeCallback(GLFWwindow *a_window, int a_width, int a_height);

// callback when an error GLFW occurs
void onErrorCallback(int a_error, const char *a_description);

// callback when a key is pressed
void onKeyCallback(GLFWwindow *a_window, int a_key, int a_scancode,
                   int a_action, int a_mods);

// callback when window content scaling is modified
void onWindowContentScaleCallback(GLFWwindow *a_window, float a_xscale,
                                  float a_yscale);

// callback to handle mouse click
void onMouseButtonCallback(GLFWwindow *a_window, int a_button, int a_action,
                           int a_mods);

// this function renders the scene
void renderGraphics(void);

// this function contains the main haptics simulation loop
void renderHaptics(void);

// this function closes the application
void close(void);

int main(int argc, char *argv[]) {
  //--------------------------------------------------------------------------
  // INITIALIZATION
  //--------------------------------------------------------------------------

  cout << endl;
  cout << "-----------------------------------" << endl;
  cout << "CHAI3D" << endl;
  cout << "Demo: 32-modelling" << endl;
  cout << "Copyright 2026" << endl;
  cout << "-----------------------------------" << endl << endl << endl;
  cout << "Keyboard Options:" << endl << endl;
  cout << "[1] - Wireframe (ON/OFF)" << endl;
  cout << "[f] - Enable/Disable full screen mode" << endl;
  cout << "[m] - Enable/Disable vertical mirroring" << endl;
  cout << "[q] - Exit application" << endl;
  cout << endl << endl;

  // parse first arg to try and locate resources
  resourceRoot =
      string(argv[0]).substr(0, string(argv[0]).find_last_of("/\\") + 1);

  //--------------------------------------------------------------------------
  // OPEN GL - WINDOW DISPLAY
  //--------------------------------------------------------------------------

  // initialize GLFW library
  if (!glfwInit()) {
    cout << "failed initialization" << endl;
    cSleepMs(1000);
    return 1;
  }

  // set GLFW error callback
  glfwSetErrorCallback(onErrorCallback);

  // compute desired size of window
  const GLFWvidmode *mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
  windowW = 0.8 * mode->height;
  windowH = 0.5 * mode->height;
  int x = 0.5 * (mode->width - windowW);
  int y = 0.5 * (mode->height - windowH);

  // set OpenGL version
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

  // enable double buffering
  glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_TRUE);

  // set the desired number of samples to use for multisampling
  glfwWindowHint(GLFW_SAMPLES, 4);

  // specify that window should be resized based on monitor content scale
  glfwWindowHint(GLFW_SCALE_TO_MONITOR, GLFW_TRUE);

  // set active stereo mode
  if (stereoMode == C_STEREO_ACTIVE) {
    glfwWindowHint(GLFW_STEREO, GL_TRUE);
  } else {
    glfwWindowHint(GLFW_STEREO, GL_FALSE);
  }

  float main_scale =
      ImGui_ImplGlfw_GetContentScaleForMonitor(glfwGetPrimaryMonitor());

  // create display context
  window = glfwCreateWindow(windowW, windowH, "CHAI3D", NULL, NULL);
  if (!window) {
    cout << "failed to create window" << endl;
    cSleepMs(1000);
    glfwTerminate();
    return 1;
  }

  // set GLFW key callback
  glfwSetKeyCallback(window, onKeyCallback);

  // set GLFW window size callback
  glfwSetWindowSizeCallback(window, onWindowSizeCallback);

  // set GLFW framebuffer size callback
  glfwSetFramebufferSizeCallback(window, onFrameBufferSizeCallback);

  // set GLFW window content scaling callback
  glfwSetWindowContentScaleCallback(window, onWindowContentScaleCallback);

  // set GLFW mouse button callback
  glfwSetMouseButtonCallback(window, onMouseButtonCallback);

  // get width and height of window
  glfwGetFramebufferSize(window, &framebufferW, &framebufferH);

  // set position of window
  glfwSetWindowPos(window, x, y);

  // set window size
  glfwSetWindowSize(window, windowW, windowH);

  // set GLFW current display context
  glfwMakeContextCurrent(window);

  // set GLFW swap interval for the current display context
  glfwSwapInterval(swapInterval);

#ifdef GLEW_VERSION
  // initialize GLEW library
  if (glewInit() != GLEW_OK) {
    cout << "failed to initialize GLEW library" << endl;
    glfwTerminate();
    return 1;
  }
#endif

  //--------------------------------------------------------------------------
  // ImGui
  //--------------------------------------------------------------------------

  // Setup Dear ImGui context
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO &io = ImGui::GetIO();
  io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

  // Setup Dear ImGui style
  ImGuiStyle &style = ImGui::GetStyle();
  ImGui::StyleColorsDark();
  // ImGui::StyleColorsLight();
  // style.FontSizeBase = 16.0f;           // Default to 13.0f when unspecified,
  // which allows crisp legacy font for non-4K screens.
  style.FrameRounding = 3.0f;
  style.GrabRounding = 3.0f;

  // Setup scaling
  style.ScaleAllSizes(
      main_scale); // Bake a fixed style scale. (until we have a solution for
                   // dynamic style scaling, changing this requires resetting
                   // Style + calling this again)
  style.FontScaleDpi =
      main_scale; // Set initial font scale. (in docking branch: using
                  // io.ConfigDpiScaleFonts=true automatically overrides this
                  // for every window depending on the current monitor)

  // Setup Platform/Renderer backends
  ImGui_ImplGlfw_InitForOpenGL(window, true);
  const char *glsl_version = nullptr;
  ImGui_ImplOpenGL3_Init(glsl_version);

  bool show_another_window = false;
  ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

  //--------------------------------------------------------------------------
  // WORLD - CAMERA - LIGHTING
  //--------------------------------------------------------------------------

  // create a new world.
  world = new cWorld();

  // set the background color of the environment
  world->m_backgroundColor.setBlack();

  // create a camera and insert it into the virtual world
  camera = new cCamera(world);
  world->addChild(camera);

  // position and orient the camera
  camera->set(cVector3d(4.0, 0.0, 0.0),  // camera position (eye)
              cVector3d(0.0, 0.0, 0.0),  // lookat position (target)
              cVector3d(0.0, 0.0, 1.0)); // direction of the (up) vector

  // set the near and far clipping planes of the camera
  // anything in front or behind these clipping planes will not be rendered
  camera->setClippingPlanes(0.01, 10.0);

  // set stereo mode
  camera->setStereoMode(stereoMode);

  // set stereo eye separation and focal length (applies only if stereo is
  // enabled)
  camera->setStereoEyeSeparation(1.3);
  camera->setStereoFocalLength(3.0);

  // set vertical mirrored display mode
  camera->setMirrorVertical(mirroredDisplay);

  // create a light source
  light = new cSpotLight(world);

  // attach light to camera
  camera->addChild(light);

  // enable light source
  light->setEnabled(true);

  // position the light source
  light->setLocalPos(0.0, 0.5, 0.0);

  // define the direction of the light beam
  light->setDir(-3.0, -0.5, 0.0);

  // enable this light source to generate shadows
  light->setShadowMapEnabled(true);

  // set the resolution of the shadow map
  light->m_shadowMap->setQualityLow();
  // light->m_shadowMap->setQualityMedium();

  // set light cone half angle
  light->setCutOffAngleDeg(20);

  //--------------------------------------------------------------------------
  // HAPTIC DEVICES / TOOLS
  //--------------------------------------------------------------------------

  // create a haptic device handler
  handler = new cHapticDeviceHandler();

  // get access to the first available haptic device found
  handler->getDevice(hapticDevice, 0);

  // retrieve information about the current haptic device
  cHapticDeviceInfo hapticDeviceInfo = hapticDevice->getSpecifications();

  // if the device has a gripper, enable the gripper to simulate a user switch
  hapticDevice->setEnableGripperUserSwitch(true);

  // create a tool (cursor) and insert into the world
  tool = new cToolCursor(world);
  world->addChild(tool);

  // connect the haptic device to the virtual tool
  tool->setHapticDevice(hapticDevice);

  // define the radius of the tool (sphere)
  double toolRadius = 0.05;

  // define a radius for the tool
  tool->setRadius(toolRadius);

  // hide the device sphere. only show proxy.
  tool->setShowContactPoints(true, false);

  // create a white cursor
  tool->m_hapticPoint->m_sphereProxy->m_material->setWhite();

  // enable if objects in the scene are going to rotate of translate
  // or possibly collide against the tool. If the environment
  // is entirely static, you can set this parameter to "false"
  tool->enableDynamicObjects(true);

  // map the physical workspace of the haptic device to a larger virtual
  // workspace.
  tool->setWorkspaceRadius(1.0);

  // haptic forces are enabled only if small forces are first sent to the
  // device; this mode avoids the force spike that occurs when the application
  // starts when the tool is located inside an object for instance.
  tool->setWaitForSmallForce(true);

  // start the haptic tool
  tool->start();

  //--------------------------------------------------------------------------
  // CREATE OBJECTS
  //--------------------------------------------------------------------------

  // read the scale factor between the physical workspace of the haptic
  // device and the virtual workspace defined for the tool
  double workspaceScaleFactor = tool->getWorkspaceScaleFactor();

  // stiffness properties
  double maxStiffness =
      hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;

  // create a virtual mesh
  chai3d::cMultiMesh *objectMM = new cMultiMesh();

  objectMM->m_name = "My multi mesh";

  bool loadSuccess = objectMM->loadFromFile(
      RESOURCE_PATH("../resources/models/cube/cube.obj"));

  if (!loadSuccess) {
    printf("Error - 3D Model failed to load correctly.\n");
    close();
    return (-1);
  }

  chai3d::cMesh *object = objectMM->getMesh(0);

  // Preprocessing the object
  object->computeBoundaryBox(true);
  double size =
      cSub(object->getBoundaryMax(), object->getBoundaryMin()).length();

  // resize object to screen
  if (size > 0) {
    object->scale(1.5 / size);
  }

  // Load object to PriMo representation
  MyStuff::BasicHEMesh heMesh = MyStuff::fromChai3DMesh(*object);
  primoMesh = MyStuff::fromHEMesh(heMesh);

  // NOTE: Just checking to make sure converting back from PriMo representation
  // works
  // MyStuff::Primo2Chai3DMesh(primoMesh, myMesh);

  myMesh.setVertexColor({0.1f, 0.7f, 0.2f});

  // set the position of the object at the center of the world
  myMesh.setLocalPos(0.0, 0.0, 0.0);

  // myMesh.rotateAboutLocalAxisRad({1.0, 0.0, 0.0}, 3.14 / 4.0);
  // myMesh.rotateAboutLocalAxisRad({0.0, 1.0, 0.0}, 3.14 / 4.0);

  // Since we want to see our polygons from both sides, we disable culling.
  myMesh.setUseCulling(false);

  // we indicate that we ware rendering triangles by using specific colors for
  // each of them (see above)
  myMesh.setUseVertexColors(true);

  // add object to world
  world->addChild(&myMesh);

  unselectColor.setGreen();
  selectColor.setRed();
  hoverColor.setBlue();

  // compute surface normals
  // object->computeAllNormals();

  // we indicate that we also using material properties. If you set this
  // parameter to 'false' you will notice that only vertex colors are used to
  // render triangle, and lighting will not longer have any effect.
  // object->setUseMaterial(true);

  // compute collision detection algorithm
  // object->createAABBCollisionDetector(toolRadius);

  // define a default stiffness for the object
  // object->m_material->setStiffness(0.1 * maxStiffness);

  // render triangles hapticaly on both sides
  // object->m_material->setHapticTriangleSides(true, true);

  //--------------------------------------------------------------------------
  // WIDGETS
  //--------------------------------------------------------------------------

  // create a font
  font = NEW_CFONT_CALIBRI_20();

  // create a label to display the haptic and graphic rate of the simulation
  labelRates = new cLabel(font);
  camera->m_frontLayer->addChild(labelRates);

  // create a label that describes how to operate the application
  labelDescription = new cLabel(font);
  camera->m_frontLayer->addChild(labelDescription);
  labelDescription->setText("use mouse to select a triangle");
  labelDescription->m_fontColor.setWhite();

  // create a background
  background = new cBackground();
  camera->m_backLayer->addChild(background);

  // set background color properties
  // set background properties
  background->setCornerColors(
      cColorf(0.44f, 0.44f, 0.88f), cColorf(0.44f, 0.44f, 0.88f),
      cColorf(0.00f, 0.00f, 0.00f), cColorf(0.00f, 0.00f, 0.00f));

  //--------------------------------------------------------------------------
  // VIEWPORT DISPLAY
  //--------------------------------------------------------------------------

  // get content scale factor
  float contentScaleW, contentScaleH;
  glfwGetWindowContentScale(window, &contentScaleW, &contentScaleH);

  // create a viewport to display the scene.
  viewport = new cViewport(camera, contentScaleW, contentScaleH);

  //--------------------------------------------------------------------------
  // START HAPTIC SIMULATION THREAD
  //--------------------------------------------------------------------------

  // create a thread which starts the main haptics rendering loop
  hapticsThread = new cThread();
  hapticsThread->start(renderHaptics, CTHREAD_PRIORITY_HAPTICS);

  // setup callback when application exits
  atexit(close);

  //--------------------------------------------------------------------------
  // MAIN GRAPHIC LOOP
  //--------------------------------------------------------------------------

  // main graphic loop
  while (!glfwWindowShouldClose(window)) {
    // primoMesh.faces[faceIdx].translate = faceTranslation;
    // primoMesh.faces[faceIdx].rotate = faceRotation;

    if (doDeform)
      MyStuff::optimizePriMo(primoMesh);

    MyStuff::Primo2Chai3DMesh(primoMesh, myMesh);

    chai3d::cMatrix3d rotMat = chai3d::cRotEulerRad(
        objectRotation(0), objectRotation(1), objectRotation(2),
        chai3d::cEulerOrder::C_EULER_ORDER_ZXY);

    myMesh.setLocalPos(objectTranslation);
    myMesh.setLocalRot(rotMat);

    // render graphics
    renderGraphics();

    // process events
    glfwPollEvents();
  }

  // Cleanup
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();

  // close window
  glfwDestroyWindow(window);

  // terminate GLFW library
  glfwTerminate();

  // exit
  return (0);
}

//------------------------------------------------------------------------------

void onWindowSizeCallback(GLFWwindow *a_window, int a_width, int a_height) {
  // update window size
  windowW = a_width;
  windowH = a_height;

  // render scene
  renderGraphics();
}

//------------------------------------------------------------------------------

void onFrameBufferSizeCallback(GLFWwindow *a_window, int a_width,
                               int a_height) {
  // update frame buffer size
  framebufferW = a_width;
  framebufferH = a_height;
}

//------------------------------------------------------------------------------

void onWindowContentScaleCallback(GLFWwindow *a_window, float a_xscale,
                                  float a_yscale) {
  // update window content scale factor
  viewport->setContentScale(a_xscale, a_yscale);
}

//------------------------------------------------------------------------------

void onErrorCallback(int a_error, const char *a_description) {
  cout << "Error: " << a_description << endl;
}

//------------------------------------------------------------------------------

void onKeyCallback(GLFWwindow *a_window, int a_key, int a_scancode,
                   int a_action, int a_mods) {
  // filter calls that only include a key press
  if ((a_action != GLFW_PRESS) && (a_action != GLFW_REPEAT)) {
    return;
  }

  // option - exit
  else if ((a_key == GLFW_KEY_ESCAPE) || (a_key == GLFW_KEY_Q)) {
    glfwSetWindowShouldClose(a_window, GLFW_TRUE);
  }

  // option - enable/disable wireframe mode
  // else if (a_key == GLFW_KEY_1) {
  //   bool useWireMode = !object->getWireMode();
  //   object->setWireMode(useWireMode);
  //   if (useWireMode)
  //     cout << "> Wire mode enabled  \r";
  //   else
  //     cout << "> Wire mode disabled \r";
  // }

  // option - toggle fullscreen
  else if (a_key == GLFW_KEY_F) {
    // toggle state variable
    fullscreen = !fullscreen;

    // get handle to monitor
    GLFWmonitor *monitor = glfwGetPrimaryMonitor();

    // get information about monitor
    const GLFWvidmode *mode = glfwGetVideoMode(monitor);

    // set fullscreen or window mode
    if (fullscreen) {
      glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height,
                           mode->refreshRate);
    } else {
      int w = 0.8 * mode->height;
      int h = 0.5 * mode->height;
      int x = 0.5 * (mode->width - w);
      int y = 0.5 * (mode->height - h);
      glfwSetWindowMonitor(window, NULL, x, y, w, h, mode->refreshRate);
    }

    // set the desired swap interval and number of samples to use for
    // multisampling
    glfwSwapInterval(swapInterval);
    glfwWindowHint(GLFW_SAMPLES, 4);
  }

  // option - toggle vertical mirroring
  else if (a_key == GLFW_KEY_M) {
    mirroredDisplay = !mirroredDisplay;
    camera->setMirrorVertical(mirroredDisplay);
  }
}

//------------------------------------------------------------------------------

void onMouseButtonCallback(GLFWwindow *a_window, int a_button, int a_action,
                           int a_mods) {
  if (a_button == GLFW_MOUSE_BUTTON_LEFT && a_action == GLFW_PRESS) {
    // store mouse position
    double x, y;
    glfwGetCursorPos(window, &x, &y);

    // detect for any collision between mouse and scene
    cCollisionRecorder recorder;
    cCollisionSettings settings;

    bool hit = camera->selectWorld(x, (windowH - y), windowW, windowH, recorder,
                                   settings);
    if (hit) {
      // create red color
      cColorf color;
      color.setRed();

      // retrieve triangle selected by mouse
      int index = recorder.m_nearestCollision.m_index;
      cTriangleArrayPtr triangles = recorder.m_nearestCollision.m_triangles;

      if (index > -1) {
        // get indices to all three vertices
        unsigned int index0 = triangles->getVertexIndex0(index);
        unsigned int index1 = triangles->getVertexIndex1(index);
        unsigned int index2 = triangles->getVertexIndex2(index);

        // for each vertex we attribute the new color;in this example
        // the color is the same for all vertices, but it is of course
        // possible to define a specific color for each vertex
        triangles->m_vertices->setColor(index0, color);
        triangles->m_vertices->setColor(index1, color);
        triangles->m_vertices->setColor(index2, color);
      }
    }
  }
}

//------------------------------------------------------------------------------

void close(void) {
  // stop the simulation
  simulationRunning = false;

  // wait for graphics and haptics loops to terminate
  while (!simulationFinished) {
    cSleepMs(100);
  }

  // close haptic device
  tool->stop();

  // delete resources
  delete hapticsThread;
  delete world;
  delete handler;
}

//------------------------------------------------------------------------------

void renderGraphics(void) {
  // sanity check
  if (viewport == nullptr) {
    return;
  }

  static ImGuiDockNodeFlags flags;

  /////////////////////////////////////////////////////////////////////
  // UPDATE WIDGETS
  /////////////////////////////////////////////////////////////////////

  // get width and height of CHAI3D internal rendering buffer
  int displayW = viewport->getDisplayWidth();
  int displayH = viewport->getDisplayHeight();

  // update haptic and graphic rate data
  labelRates->setText(cStr(freqCounterGraphics.getFrequency(), 0) + " Hz / " +
                      cStr(freqCounterHaptics.getFrequency(), 0) + " Hz");

  // update position of label
  labelRates->setLocalPos((int)(0.5 * (displayW - labelRates->getWidth())), 15);

  // update position of label
  labelDescription->setLocalPos(
      (int)(0.5 * (displayW - labelDescription->getWidth())), 40);

  /////////////////////////////////////////////////////////////////////
  // RENDER SCENE
  /////////////////////////////////////////////////////////////////////

  // Update triangle colors
  auto nTriangles = myMesh.getNumTriangles();
  for (unsigned int i = 0; i < nTriangles; i++) {
    auto &triangles = myMesh.m_triangles;

    unsigned int index0 = triangles->getVertexIndex0(i);
    unsigned int index1 = triangles->getVertexIndex1(i);
    unsigned int index2 = triangles->getVertexIndex2(i);

    triangles->m_vertices->setColor(index0, unselectColor);
    triangles->m_vertices->setColor(index1, unselectColor);
    triangles->m_vertices->setColor(index2, unselectColor);

    if (selectedTrianglesIndices.find(i) != selectedTrianglesIndices.end()) {
      triangles->m_vertices->setColor(index0, selectColor);
      triangles->m_vertices->setColor(index1, selectColor);
      triangles->m_vertices->setColor(index2, selectColor);
    }

    if (hoveredTrianglesIndices.find(i) != hoveredTrianglesIndices.end()) {
      triangles->m_vertices->setColor(index0, hoverColor);
      triangles->m_vertices->setColor(index1, hoverColor);
      triangles->m_vertices->setColor(index2, hoverColor);
    }

    if (primoMesh.faces[i].isConstrain) {
      triangles->m_vertices->setColor(index0, selectColor);
      triangles->m_vertices->setColor(index1, selectColor);
      triangles->m_vertices->setColor(index2, selectColor);
    }

    if (i == faceIdx) {
      triangles->m_vertices->setColor(index0, hoverColor);
      triangles->m_vertices->setColor(index1, hoverColor);
      triangles->m_vertices->setColor(index2, hoverColor);
    }
  }

  // update shadow maps (if any)
  world->updateShadowMaps(false, mirroredDisplay);

  // render world
  viewport->renderView(framebufferW, framebufferH);

  // Start the Dear ImGui frame
  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();

  // ImGui::DockSpaceOverViewport(0, nullptr, flags);

  MyImGuiControlPanel();

  // Rendering
  ImGui::Render();

  int display_w, display_h;
  glfwGetFramebufferSize(window, &display_w, &display_h);
  glViewport(0, 0, display_w, display_h);
  // glClearColor(clear_color.x * clear_color.w, clear_color.y *
  // clear_color.w,
  //              clear_color.z * clear_color.w, clear_color.w);
  // glClear(GL_COLOR_BUFFER_BIT);
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

  // wait until all GL commands are completed
  glFinish();

  // check for any OpenGL errors
  GLenum error = glGetError();
  if (error != GL_NO_ERROR)
    cout << "Error: " << gluErrorString(error) << endl;

  // swap buffers
  glfwSwapBuffers(window);

  // signal frequency counter
  freqCounterGraphics.signal(1);
}

//------------------------------------------------------------------------------

void renderHaptics(void) {
  // angular velocity of object
  // cVector3d angVel(0.0, 0.0, 0.4);

  // reset clock
  cPrecisionClock clock;
  clock.reset();

  // simulation in now running
  simulationRunning = true;
  simulationFinished = false;

  // read user-switch status (button 0)
  // static bool lastframe_btn0 = false;
  // static bool lastframe_btn1 = false;
  // static bool lastframe_btn2 = false;
  // static bool lastframe_btn3 = false;
  // bool button0, button1, button2, button3;
  // button0 = false;
  // button1 = false;
  // button2 = false;
  // button3 = false;

  // Select triangles
  auto touchPoint = tool->getHapticPoint(0);

  // main haptic simulation loop
  while (simulationRunning) {
    /////////////////////////////////////////////////////////////////////
    // SIMULATION TIME
    /////////////////////////////////////////////////////////////////////

    // stop the simulation clock
    clock.stop();

    // read the time increment in seconds
    double timeInterval = clock.getCurrentTimeSeconds();

    // restart the simulation clock
    clock.reset();
    clock.start();

    // signal frequency counter
    freqCounterHaptics.signal(1);

    /////////////////////////////////////////////////////////////////////
    // HAPTIC FORCE COMPUTATION
    /////////////////////////////////////////////////////////////////////

    // compute global reference frames for each object
    world->computeGlobalPositions(true);

    // update position and orientation of tool
    tool->updateFromDevice();

    // compute interaction forces
    tool->computeInteractionForces();

    // send forces to haptic device
    tool->applyToDevice();

    /////////////////////////////////////////////////////////////////////
    // MY CODE
    /////////////////////////////////////////////////////////////////////

    // hapticDevice->getUserSwitch(0, button0);
    // hapticDevice->getUserSwitch(1, button1);
    // hapticDevice->getUserSwitch(2, button2);
    // hapticDevice->getUserSwitch(3, button3);
    //
    // bool btn0_repeat = button0 && lastframe_btn0 == button0;
    // bool btn1_repeat = button1 && lastframe_btn1 == button1;
    // bool btn2_repeat = button2 && lastframe_btn2 == button2;
    // bool btn3_repeat = button3 && lastframe_btn3 == button3;

    // hoveredTrianglesIndices.clear();

    // Highlight hovering triangles
    // if (touchPoint->isInContact(&myMesh)) {
    //   auto nEvents = touchPoint->getNumCollisionEvents();
    //
    //   for (unsigned int i = 0; i < nEvents; i++) {
    //     auto event = touchPoint->getCollisionEvent(i);
    //
    //     if (event->m_object && event->m_triangles && event->m_index > -1) {
    //       auto index = event->m_index;
    //
    //       hoveredTrianglesIndices.insert(index);
    //     }
    //   }
    // }

    // static cVector3d lastCursorPos = tool->getDeviceGlobalPos();

    // Select and & select triangles
    // if (button0 && !btn0_repeat) {
    //   // Pressing the button, so (un)select hovering triangle
    //
    //   if (touchPoint->isInContact(&myMesh)) {
    //     auto nEvents = touchPoint->getNumCollisionEvents();
    //
    //     for (unsigned int i = 0; i < nEvents; i++) {
    //       auto event = touchPoint->getCollisionEvent(i);
    //
    //       if (event->m_object && event->m_index > -1) {
    //         auto index = event->m_index;
    //
    //         if (selectedTrianglesIndices.find(event->m_index) ==
    //             selectedTrianglesIndices.end()) {
    //           // Triangle not selected, so select it.
    //           std::cout << "Selecting triangle #" << index << std::endl;
    //           selectedTrianglesIndices.insert(index);
    //         } else {
    //           // Triangle selected, so unselect it.
    //           std::cout << "Unselecting triangle #" << index << std::endl;
    //           selectedTrianglesIndices.erase(index);
    //         }
    //       }
    //     }
    //   } else {
    //     selectedTrianglesIndices.clear();
    //   }
    // } else if (button1) { // Move selected triangles
    //   if (btn1_repeat) {
    //     // Move the triangles
    //     cVector3d cursorPos = tool->getDeviceGlobalPos();
    //     cVector3d dp = cursorPos - lastCursorPos;
    //     auto rotMat = myMesh.getGlobalRot();
    //     rotMat.invert();
    //
    //     // Move the triangles
    //     for (auto index : selectedTrianglesIndices) {
    //       auto &triangles = myMesh.m_triangles;
    //
    //       unsigned int index0 = triangles->getVertexIndex0(index);
    //       unsigned int index1 = triangles->getVertexIndex1(index);
    //       unsigned int index2 = triangles->getVertexIndex2(index);
    //
    //       auto pos0 = myMesh.m_vertices->getLocalPos(index0);
    //       auto pos1 = myMesh.m_vertices->getLocalPos(index1);
    //       auto pos2 = myMesh.m_vertices->getLocalPos(index2);
    //
    //       triangles->m_vertices->setLocalPos(index0, pos0 + rotMat * dp);
    //       triangles->m_vertices->setLocalPos(index1, pos1 + rotMat * dp);
    //       triangles->m_vertices->setLocalPos(index2, pos2 + rotMat * dp);
    //     }
    //   }
    //
    //   lastCursorPos = tool->getDeviceGlobalPos();
    // } else if (button2) {
    //   if (btn2_repeat) {
    //     // Move the triangles
    //     cVector3d cursorPos = tool->getDeviceGlobalPos();
    //     cVector3d dp = cursorPos - lastCursorPos;
    //
    //     cVector3d rotAngle = dp * 200.0f;
    //
    //     cMatrix3d rotMat =
    //         cRotEulerDeg(rotAngle.get(1), rotAngle.get(2), rotAngle.get(0),
    //                      chai3d::C_EULER_ORDER_ZYX);
    //
    //     // Move the triangles
    //     for (auto index : selectedTrianglesIndices) {
    //       auto &triangles = myMesh.m_triangles;
    //       unsigned int index0 = triangles->getVertexIndex0(index);
    //       unsigned int index1 = triangles->getVertexIndex1(index);
    //       unsigned int index2 = triangles->getVertexIndex2(index);
    //
    //       auto pos0 = myMesh.m_vertices->getLocalPos(index0);
    //       auto pos1 = myMesh.m_vertices->getLocalPos(index1);
    //       auto pos2 = myMesh.m_vertices->getLocalPos(index2);
    //       auto p = (pos0 + pos1 + pos2) / 3.0;
    //
    //       triangles->m_vertices->setLocalPos(index0, rotMat * (pos0 - p) +
    //       p); triangles->m_vertices->setLocalPos(index1, rotMat * (pos1 - p)
    //       + p); triangles->m_vertices->setLocalPos(index2, rotMat * (pos2 -
    //       p) + p);
    //     }
    //   }
    //
    //   lastCursorPos = tool->getDeviceGlobalPos();
    // } else if (button3) {
    //   if (btn3_repeat) {
    //     // Move the triangles
    //     cVector3d cursorPos = tool->getDeviceGlobalPos();
    //     cVector3d dp = cursorPos - lastCursorPos;
    //
    //     myMesh.translate(dp);
    //   }
    //
    //   lastCursorPos = tool->getDeviceGlobalPos();
    // }

    // Rotate selected triangles

    /////////////////////////////////////////////////////////////////////
    // DYNAMIC SIMULATION
    /////////////////////////////////////////////////////////////////////

    // some constants
    // const double INERTIA = 0.4;
    // const double MAX_ANG_VEL = 10.0;
    // const double DAMPING = 0.1;
    //
    // // get position of cursor in global coordinates
    // cVector3d toolPos = tool->getDeviceGlobalPos();
    //
    // // get position of object in global coordinates
    // cVector3d objectPos = object->getGlobalPos();
    //
    // // compute a vector from the center of mass of the object (point of
    // // rotation) to the tool
    // cVector3d v = cSub(toolPos, objectPos);
    //
    // // compute angular acceleration based on the interaction forces
    // // between the tool and the object
    // cVector3d angAcc(0, 0, 0);
    // if (v.length() > 0.0) {
    //   // get the last force applied to the cursor in global coordinates
    //   // we negate the result to obtain the opposite force that is applied on
    //   // the object
    //   cVector3d toolForce = -tool->getDeviceGlobalForce();
    //
    //   // compute the effective force that contributes to rotating the object.
    //   cVector3d force = toolForce - cProject(toolForce, v);
    //
    //   // compute the resulting torque
    //   cVector3d torque = cMul(v.length(), cCross(cNormalize(v), force));
    //
    //   // update rotational acceleration
    //   angAcc = (1.0 / INERTIA) * torque;
    // }

    // update rotational velocity
    // angVel.add(timeInterval * angAcc);
    //
    // // set a threshold on the rotational velocity term
    // double vel = angVel.length();
    // if (vel > MAX_ANG_VEL) {
    //   angVel.mul(MAX_ANG_VEL / vel);
    // }
    //
    // // add some damping too
    // angVel.mul(1.0 - DAMPING * timeInterval);
    //
    // // if user switch is pressed, set angular velocity to zero
    // if (tool->getUserSwitch(0) == 1) {
    //   angVel.zero();
    // }

    // compute the next rotation configuration of the object
    // if (angVel.length() > C_SMALL) {
    //   object->rotateAboutGlobalAxisRad(cNormalize(angVel),
    //                                    timeInterval * angVel.length());
    // }

    // lastframe_btn0 = button0;
    // lastframe_btn1 = button1;
    // lastframe_btn2 = button2;
    // lastframe_btn3 = button3;
  }

  // exit haptics thread
  simulationFinished = true;
}

//------------------------------------------------------------------------------
