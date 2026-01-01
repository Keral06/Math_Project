#pragma once
#include <GL/glew.h>
#include <vector>

struct Mesh {
    GLuint vao = 0, vbo = 0, ebo = 0;
    int indexCount = 0;

    void InitCube() {
        float vertices[] = {
            // Front Face (Z+)
            -0.5f, -0.5f,  0.5f, // 0 BL
             0.5f, -0.5f,  0.5f, // 1 BR
             0.5f,  0.5f,  0.5f, // 2 TR
            -0.5f,  0.5f,  0.5f, // 3 TL

            // Back Face (Z-)
             0.5f, -0.5f, -0.5f, // 4 BR
            -0.5f, -0.5f, -0.5f, // 5 BL
            -0.5f,  0.5f, -0.5f, // 6 TL
             0.5f,  0.5f, -0.5f, // 7 TR

             // Right Face (X+)
              0.5f, -0.5f,  0.5f, // 8  Front-Bottom
              0.5f, -0.5f, -0.5f, // 9  Back-Bottom
              0.5f,  0.5f, -0.5f, // 10 Back-Top
              0.5f,  0.5f,  0.5f, // 11 Front-Top

              // Left Face (X-)
              -0.5f, -0.5f, -0.5f, // 12 Back-Bottom
              -0.5f, -0.5f,  0.5f, // 13 Front-Bottom
              -0.5f,  0.5f,  0.5f, // 14 Front-Top
              -0.5f,  0.5f, -0.5f, // 15 Back-Top

              // Top Face (Y+)
              -0.5f,  0.5f,  0.5f, // 16 Front-Left
               0.5f,  0.5f,  0.5f, // 17 Front-Right
               0.5f,  0.5f, -0.5f, // 18 Back-Right
              -0.5f,  0.5f, -0.5f, // 19 Back-Left

              // Bottom Face (Y-)
              -0.5f, -0.5f, -0.5f, // 20 Back-Left
               0.5f, -0.5f, -0.5f, // 21 Back-Right
               0.5f, -0.5f,  0.5f, // 22 Front-Right
              -0.5f, -0.5f,  0.5f  // 23 Front-Left
        };

        unsigned int indices[] = {
            // Front
            0, 1, 2, 2, 3, 0,
            // Back
            4, 5, 6, 6, 7, 4,
            // Right
            8, 9, 10, 10, 11, 8,
            // Left
            12, 13, 14, 14, 15, 12,
            // Top
            16, 17, 18, 18, 19, 16,
            // Bottom
            20, 21, 22, 22, 23, 20
        };

        indexCount = 36; // 6 cares * 2 triangles * 3 vèrtexs

        if (vao == 0) glGenVertexArrays(1, &vao);
        if (vbo == 0) glGenBuffers(1, &vbo);
        if (ebo == 0) glGenBuffers(1, &ebo);

        glBindVertexArray(vao);

        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

        // Posició (location = 0, 3 floats)
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);

        glBindVertexArray(0);
    }

    void Draw() {
        if (vao == 0) InitCube();
        glBindVertexArray(vao);
        glDrawElements(GL_TRIANGLES, indexCount, GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);
    }
};