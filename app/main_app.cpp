#define _USE_MATH_DEFINES
#include <SDL3/SDL.h>
#include <GL/glew.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>

// ImGui
#include "imgui.h"
#include "imgui_impl_sdl3.h"
#include "imgui_impl_opengl3.h"

// Project Headers
#include "Matrix4x4.hpp"
#include "Mesh.hpp"          // Conté la classe Mesh (Cube)
#include "GraphicsUtils.hpp" // Conté helpers per OpenGL
//necesito hacer un quat que sea donde esta mirando el objeto, donde tiene que rotat la matriz.
Quat LookRotation(const Vec3& forward, const Vec3& up)
{
    Vec3 f = forward.Normalize();
    Vec3 r = Vec3 :: Cross(up,f).Normalize();
    Vec3 u = Vec3::Cross(f,r);

    Matrix3x3 rot;
    rot.At(0, 0) = r.x; rot.At(0, 1) = r.y; rot.At(0, 2) = r.z;
    rot.At(1, 0) = u.x; rot.At(1, 1) = u.y; rot.At(1, 2) = u.z;
    rot.At(2, 0) = f.x; rot.At(2, 1) = f.y; rot.At(2, 2) = f.z;

    return Quat::FromMatrix3x3(rot).Normalized();
}
Vec3 Lerp(const Vec3& a, const Vec3& b, double t)
{
    return Vec3{
        a.x + (b.x - a.x) * t,
        a.y + (b.y - a.y) * t,
        a.z + (b.z - a.z) * t
    };
}

Quat Slerp(const Quat& a, const Quat& b, double t)
{
    Quat q1 = a.Normalized();
    Quat q2 = b.Normalized();

    double dot = q1.s * q2.s + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z;

    if (dot < 0.0)
    {
        q2.s = -q2.s;
        q2.x = -q2.x;
        q2.y = -q2.y;
        q2.z = -q2.z;
        dot = -dot;
    }

    const double DOT_THRESHOLD = 0.9995;
    if (dot > DOT_THRESHOLD)
    {
        Quat result = {
            q1.s + t * (q2.s - q1.s),
            q1.x + t * (q2.x - q1.x),
            q1.y + t * (q2.y - q1.y),
            q1.z + t * (q2.z - q1.z)
        };
        return result.Normalized();
    }

    double theta_0 = acos(dot);
    double theta = theta_0 * t;
    double sin_theta = sin(theta);
    double sin_theta_0 = sin(theta_0);

    Quat result;
    result.s = (cos(theta) - dot * sin_theta / sin_theta_0) * q1.s + (sin_theta / sin_theta_0) * q2.s;
    result.x = (cos(theta) - dot * sin_theta / sin_theta_0) * q1.x + (sin_theta / sin_theta_0) * q2.x;
    result.y = (cos(theta) - dot * sin_theta / sin_theta_0) * q1.y + (sin_theta / sin_theta_0) * q2.y;
    result.z = (cos(theta) - dot * sin_theta / sin_theta_0) * q1.z + (sin_theta / sin_theta_0) * q2.z;

    return result.Normalized();
}
// TODO: Placeholder
class Transform {
public:
    Vec3 position = { 0.0, 0.0, 0.0 };
    Vec3 rotation = { 0.0, 0.0, 0.0 }; 
    Vec3 scale = { 1.0, 1.0, 1.0 };

    Transform() {}

    Matrix4x4 GetLocalMatrix() const {
        const double rad = M_PI / 180.0;

        // Graus a Radians
        double radX = rotation.x * rad; 
        double radY = rotation.y * rad; 
        double radZ = rotation.z * rad; 

        //Crear la Matriu de Rotació
        Matrix3x3 matrot = Matrix3x3::FromEulerZYX(radY, radX, radZ);

        // Covertir a TRS
        return Matrix4x4::FromTRS(position, matrot, scale);
    }
};
class GameObject {
public:
    Transform transform;                  
    GameObject* parent = nullptr;         
    std::vector<GameObject*> children;    

    GameObject() {}

    // Construir jerarquía 
    void AddChild(GameObject* child) {
        if (child) {
            child->parent = this;       
            children.push_back(child);  
        }
    }

    Matrix4x4 GetGlobalMatrix() {
        // Matriu local de l'objecte
        Matrix4x4 localMatrix = transform.GetLocalMatrix();

        //si no es arrel
        if (parent != nullptr) {
            Matrix4x4 parentGlobal = parent->GetGlobalMatrix();

            // M_global = M_parent_global * M_local
            return parentGlobal.Multiply(localMatrix);
        }
        //si es arrel
        return localMatrix;
    }
};
class Camera
{
public:
    Transform transform;
    double fovY = 60.0;
    double aspectRatio = 1.777;
    double nearPlane = 0.1;
    double farPlane = 100.0;

    bool isMoving = false;
    bool isRotating = false;

    Quat currentRotation{ 1.0, 0.0, 0.0, 0.0 };
    Vec3 targetPosition{ 0.0, 0.0, 0.0 };
    Quat targetRotation{ 1.0, 0.0, 0.0, 0.0 };

    Matrix4x4 GetViewMatrix()
    {
        Matrix4x4 global = transform.GetLocalMatrix();
        return global.InverseTR();
    }

    Matrix4x4 GetProjectionMatrix() const
    {
        Matrix4x4 P;
        double rad = fovY * (M_PI / 180.0);
        double t = tan(rad / 2.0) * nearPlane;
        double r = t * aspectRatio;

        P.At(0, 0) = nearPlane / r;
        P.At(1, 1) = nearPlane / t;
        P.At(2, 2) = -(farPlane + nearPlane) / (farPlane - nearPlane);
        P.At(2, 3) = -(2.0 * farPlane * nearPlane) / (farPlane - nearPlane);
        P.At(3, 2) = -1.0;
        P.At(3, 3) = 0.0;
        return P;
    }
};
// -----------------------------------------------------------------------------
// HELPER: Càrrega de fitxers de text (per Shaders)
// -----------------------------------------------------------------------------
std::string LoadShaderFile(const std::string& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open shader file: " << filepath << std::endl;
        return "";
    }
    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

// -----------------------------------------------------------------------------
// HELPER: Compilació de Shaders
// -----------------------------------------------------------------------------
GLuint CompileShader(GLenum type, const std::string& source) {
    const char* srcPtr = source.c_str();
    GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &srcPtr, nullptr);
    glCompileShader(shader);

    int success;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        char infoLog[512];
        glGetShaderInfoLog(shader, 512, nullptr, infoLog);
        std::cerr << "ERROR::SHADER::COMPILATION_FAILED\n" << infoLog << std::endl;
    }
    return shader;
}

GLuint CreateShaderProgram(const std::string& vertPath, const std::string& fragPath) {
    std::string vertCode = LoadShaderFile(vertPath);
    std::string fragCode = LoadShaderFile(fragPath);

    if (vertCode.empty() || fragCode.empty()) return 0;

    GLuint vertexShader = CompileShader(GL_VERTEX_SHADER, vertCode);
    GLuint fragmentShader = CompileShader(GL_FRAGMENT_SHADER, fragCode);

    GLuint shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);

    int success;
    glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
    if (!success) {
        char infoLog[512];
        glGetProgramInfoLog(shaderProgram, 512, nullptr, infoLog);
        std::cerr << "ERROR::PROGRAM::LINKING_FAILED\n" << infoLog << std::endl;
    }

    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);
    return shaderProgram;
}

// -----------------------------------------------------------------------------
// UI: (TODO)
// -----------------------------------------------------------------------------
GameObject* selectedObject = nullptr;
GameObject* lastSelectedObject = nullptr;

void DrawHierarchyNode(GameObject* node, Camera& cam)
{
    if (!node) return;

    ImGuiTreeNodeFlags flags =
        ImGuiTreeNodeFlags_OpenOnArrow |
        ImGuiTreeNodeFlags_OpenOnDoubleClick;

    if (node == selectedObject)
        flags |= ImGuiTreeNodeFlags_Selected;

    bool open = ImGui::TreeNodeEx((void*)node, flags, "GameObject");

    if (ImGui::IsItemClicked())
    {
        selectedObject = node;

        if (selectedObject != lastSelectedObject)
        {

            Matrix4x4 g = selectedObject->GetGlobalMatrix();
            Vec3 objPos = g.GetTranslation();

            // --- Compute front direction in world space ---
            Vec3 localForward = Vec3(0, 0, -1); // cube's local front
            Vec3 objForward = Vec3(
                g.At(0, 0) * localForward.x + g.At(0, 1) * localForward.y + g.At(0, 2) * localForward.z,
                g.At(1, 0) * localForward.x + g.At(1, 1) * localForward.y + g.At(1, 2) * localForward.z,
                g.At(2, 0) * localForward.x + g.At(2, 1) * localForward.y + g.At(2, 2) * localForward.z
            ).Normalize();
            double offsetDistance = 5.0;
            cam.targetPosition = {
				objPos.x - objForward.x * offsetDistance,
				objPos.y - objForward.y * offsetDistance,
				objPos.z - objForward.z * offsetDistance

            };
            cam.isMoving = true;
            double distanceInFront = 2.0;
            Vec3 facePoint = {objPos.x + objForward.x * distanceInFront,
            objPos.y +objForward.y * distanceInFront,
            objPos.z +objForward.z * distanceInFront};
                
            // --- Compute target rotation ---
            Vec3 camDir = {
            facePoint.x - cam.targetPosition.x,
            facePoint.y - cam.transform.position.y,
            facePoint.z - cam.transform.position.z
            };

			camDir = camDir.Normalize();
            cam.targetRotation = LookRotation(camDir, Vec3(0, 1, 0));
            cam.isRotating = true;

            lastSelectedObject = selectedObject;


            lastSelectedObject = selectedObject;
        }
    }

    if (cam.isMoving)
    {
        cam.transform.position =
            Lerp(cam.transform.position, cam.targetPosition, 0.1);

        if (fabs(cam.transform.position.x - cam.targetPosition.x) < 0.1 &&
            fabs(cam.transform.position.y - cam.targetPosition.y) < 0.1 &&
            fabs(cam.transform.position.z - cam.targetPosition.z) < 0.1)
        {
            cam.isMoving = false;
        }
    }
    if (cam.isRotating)
    {
        cam.currentRotation = Slerp(cam.currentRotation.Normalized(), cam.targetRotation.Normalized(), 0.1);

        double yaw, pitch, roll;
        cam.currentRotation.ToEulerZYX(roll, pitch, yaw);

        cam.transform.rotation = Vec3(
            roll * (180.0 / M_PI),
            pitch * (180.0 / M_PI),
            yaw * (180.0 / M_PI)
        );

        double dot = cam.currentRotation.s * cam.targetRotation.s +
            cam.currentRotation.x * cam.targetRotation.x +
            cam.currentRotation.y * cam.targetRotation.y +
            cam.currentRotation.z * cam.targetRotation.z;

        if (dot > 0.9995)
        {
            cam.isRotating = false;
            cam.currentRotation = cam.targetRotation;
            cam.currentRotation.ToEulerZYX(roll, pitch, yaw);
            cam.transform.rotation = Vec3(
                roll * (180.0 / M_PI),
                pitch * (180.0 / M_PI),
                yaw * (180.0 / M_PI)
            );
        }
    }

    if (open)
    {
        for (auto* c : node->children)
            DrawHierarchyNode(c, cam);
        ImGui::TreePop();
    }
}
// -----------------------------------------------------------------------------
// RENDER (TODO)
// -----------------------------------------------------------------------------
void RenderNode(GameObject* node, GLuint shaderProgram, const Matrix4x4& view, const Matrix4x4& proj, Mesh& mesh) {
    if (!node) return;

    // TODO: Implementar el recorregut recursiu de renderitzat
    Matrix4x4 model = node->GetGlobalMatrix();
    // 1. Calcular la matriu Model (Global) de l'objecte actual.
    GraphicsUtils::UploadMVP(shaderProgram, model, view, proj);
    // 2. Enviar les matrius Model, View i Projection al shader (usant GraphicsUtils).
    GraphicsUtils::UploadColor(shaderProgram, Vec3(1.0, 0.0, 0.0));
    // 3. Enviar color (usant GraphicsUtils).
    mesh.Draw();
    // 4. Dibuixar la mesh.
    for (auto* child : node->children) {
        RenderNode(child, shaderProgram, view, proj, mesh);
    }
    // 5. Cridar recursivament RenderNode pels fills.
}

// -----------------------------------------------------------------------------
// MAIN (TODO)
// -----------------------------------------------------------------------------
bool mouseclicked = false;
int main(int argc, char** argv) {
    // 1. Setup SDL & OpenGL
    if (!SDL_Init(SDL_INIT_VIDEO)) {
        std::cerr << "SDL_Init Error: " << SDL_GetError() << std::endl;
        return 1;
    }

    // Configuració de context OpenGL 3.3 Core
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, 0);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);

    SDL_Window* window = SDL_CreateWindow("Project: Mini-Scene 3D", 1280, 720, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);
    if (!window) return 1;

    SDL_GLContext glContext = SDL_GL_CreateContext(window);
    SDL_GL_MakeCurrent(window, glContext);
    SDL_GL_SetSwapInterval(1); // VSync

    if (glewInit() != GLEW_OK) return 1;

    glEnable(GL_DEPTH_TEST);

    // 2. Setup ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    ImGui::StyleColorsDark();
    ImGui_ImplSDL3_InitForOpenGL(window, glContext);
    ImGui_ImplOpenGL3_Init("#version 330");

    // 3. Inicialització de recursos
    Mesh cubeMesh;
    cubeMesh.InitCube();

    // TODO: Assegureu-vos de tenir els fitxers vs.glsl i fs.glsl al mateix nivell de l'executable
    GLuint shaderProgram = CreateShaderProgram("vs.glsl", "fs.glsl");
    if (shaderProgram == 0) std::cerr << "Warning: Shaders not loaded properly." << std::endl;

    // 4. TODO: Preparar escena Inicial
    GameObject* rootObject = new GameObject();
    std::vector<GameObject*> sceneRoots = { rootObject };

	Camera mainCamera; //TODO: Inicialitzar la càmera
    mainCamera.transform.position.z = 5.0;
	// 5. Loop Principal
    bool running = true;
    while (running) {
        // --- INPUT ---
        SDL_Event event;
        while (SDL_PollEvent(&event))
        {
            ImGui_ImplSDL3_ProcessEvent(&event);

            if (event.type == SDL_EVENT_QUIT)
                running = false;
            
            //if the mouse clicks in the window
            if (event.type == SDL_EVENT_MOUSE_BUTTON_DOWN) {

				mouseclicked = true;
            }
			if (event.type == SDL_EVENT_MOUSE_BUTTON_UP) { mouseclicked = false; }
                if (event.type==SDL_EVENT_MOUSE_MOTION && mouseclicked && !ImGui::IsAnyItemActive())
                {
                    float dx = event.motion.xrel;
                    float dy = event.motion.yrel;

                    if(dx<0)
                        mainCamera.transform.rotation.x -= 1.0;
                    else if(dx>0)
						mainCamera.transform.rotation.x += 1.0;

                    if(dy<0)
                        mainCamera.transform.rotation.z -= 1.0;
                    else if(dy>0)
                        mainCamera.transform.rotation.z += 1.0;
				}
				
            {
                if (event.key.scancode == SDL_SCANCODE_W)
                    mainCamera.transform.position.y -= 0.1;
                else if (event.key.scancode == SDL_SCANCODE_S)
                    mainCamera.transform.position.y += 0.1;
                else if (event.key.scancode == SDL_SCANCODE_A)
                    mainCamera.transform.position.x -= 0.1;
                else if (event.key.scancode == SDL_SCANCODE_D)
                    mainCamera.transform.position.x += 0.1;
            }
        }


        // --- UPDATE UI ---
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplSDL3_NewFrame();
        ImGui::NewFrame();

        // UI: Jerarquia
        ImGui::Begin("Hierarchy");
        if (ImGui::Button("Add Object to Root")) 
        {
			//TODO: Afegir un nou GameObject a l'arrel de l'escena
        }
        ImGui::Separator();
        for (auto* obj : sceneRoots) DrawHierarchyNode(obj, mainCamera);
        ImGui::End();

        // UI: Inspector
        ImGui::Begin("Inspector");
        if (selectedObject) {
            ImGui::Text("Selected: %s", "TODO: <Nom Objecte>");
            ImGui::Separator();    

            float pos[3] = {
                (float)selectedObject->transform.position.x,
                (float)selectedObject->transform.position.y,
                (float)selectedObject->transform.position.z
            }; // TODO: Agafar la posició del selectedObject
            if (ImGui::DragFloat3("Position", pos, 0.1f))
            {
                selectedObject->transform.position = { (double)pos[0], (double)pos[1], (double)pos[2] };
				//TODO: Actualitzar la posició del selectedObject
            }

            float rot[3] = {
                (float)selectedObject->transform.rotation.x,
                (float)selectedObject->transform.rotation.y,
                (float)selectedObject->transform.rotation.z
            };  // TODO: Agafar la rotació del selectedObject
            if (ImGui::DragFloat3("Rotation (Euler)", rot, 0.5f))
            {
                selectedObject->transform.rotation = { (double)rot[0], (double)rot[1], (double)rot[2] };
				// TODO: Actualitzar la rotació del selectedObject
            }

            float scl[3] = {
                (float)selectedObject->transform.scale.x,
                (float)selectedObject->transform.scale.y,
                (float)selectedObject->transform.scale.z
            }; // TODO: Agafar l'escala del selectedObject
            if (ImGui::DragFloat3("Scale", scl, 0.1f))
            {
                selectedObject->transform.scale = { (double)scl[0], (double)scl[1], (double)scl[2] };
				// TODO: Actualitzar l'escala del selectedObject
            }

            ImGui::Separator();
            if (ImGui::Button("Add Child")) 
            {
                GameObject* child = new GameObject();
                selectedObject->AddChild(child);
				// TODO: Afegir un nou GameObject com a fill del selectedObject
            }

        }
        else {
            ImGui::Text("Select an object from Hierarchy.");
        }
        ImGui::End();

        // UI: Camera
        ImGui::Begin("Camera Settings");
        float fov = (float)mainCamera.fovY; // TODO: Agafar el FOV de la càmera
        if (ImGui::SliderFloat("FOV (Y)", &fov, 10.0f, 170.0f))
        {
            mainCamera.fovY = (double)fov;
			// TODO: Actualitzar el FOV de la càmera
        }

        float nearP = (float)mainCamera.nearPlane;
        float farP = (float)mainCamera.farPlane; // TODO: Agafar near i far de la càmera
        ImGui::DragFloat("Near Plane", &nearP, 0.1f);
        ImGui::DragFloat("Far Plane", &farP, 1.0f);
        
            if (nearP < 0.01f) nearP = 0.01f;
            if (farP <= nearP) farP = nearP + 0.1f;

            mainCamera.nearPlane = (double)nearP;
            mainCamera.farPlane = (double)farP;
        
        // TODO: Actualitzar near i far de la càmera si canvien


        ImGui::Separator();
        ImGui::Text("Camera Transform");
        float cPos[3] = {
            (float)mainCamera.transform.position.x,
            (float)mainCamera.transform.position.y,
            (float)mainCamera.transform.position.z
        }; // TODO: Agafar la posició de la càmera
        if (ImGui::DragFloat3("Pos", cPos, 0.1f))
        {
            mainCamera.transform.position = Vec3((double)cPos[0], (double)cPos[1], (double)cPos[2]);
			// TODO: Actualitzar la posició de la càmera
        }
        ImGui::End();

        // --- RENDER ---
        int w, h;
        SDL_GetWindowSize(window, &w, &h);
        glViewport(0, 0, w, h);
        if (h > 0)
        {
            mainCamera.aspectRatio = (double)w / (double)h;
			// TODO: Actualitzar aspect ratio de la càmera
        }

        glClearColor(0.1f, 0.1f, 0.15f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        if (shaderProgram != 0) {
            glUseProgram(shaderProgram);

            // TODO: Càlculs de Càmera (View i Projection)
            Matrix4x4 view = mainCamera.GetViewMatrix();
            Matrix4x4 proj = mainCamera.GetProjectionMatrix();

			// TODO: Recorregut de l'escena i renderitzat (RenderNode)
            for (auto* obj : sceneRoots) {
                RenderNode(obj, shaderProgram, view, proj, cubeMesh);
            }
        }

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        SDL_GL_SwapWindow(window);
    }

    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL3_Shutdown();
    ImGui::DestroyContext();
    glDeleteProgram(shaderProgram);
    SDL_GL_DestroyContext(glContext);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}