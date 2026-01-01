#pragma once
#include <GL/glew.h>
#include <vector>
#include "Matrix4x4.hpp"

namespace GraphicsUtils {

    // Puja una Matrix4x4 (double) a un uniform OpenGL (float)
    // Transpose = true per defecte perquè la vostra lib és Row-Major i GL és Col-Major
    inline void UploadMatrix4(GLuint programId, const char* uniformName, const Matrix4x4& mat, bool transpose = true) {
        GLint loc = glGetUniformLocation(programId, uniformName);
        if (loc == -1) return;

        float matFloat[16];
        for (int i = 0; i < 16; ++i) matFloat[i] = static_cast<float>(mat.m[i]);

        glUniformMatrix4fv(loc, 1, transpose ? GL_TRUE : GL_FALSE, matFloat);
    }

    inline void UploadMVP(GLuint programId, const Matrix4x4& model, const Matrix4x4& view, const Matrix4x4& proj) {
        UploadMatrix4(programId, "u_Model", model);
        UploadMatrix4(programId, "u_View", view);
        UploadMatrix4(programId, "u_Projection", proj);
    }

    inline void UploadColor(GLuint programId, const Vec3& vec) {
        GLint loc = glGetUniformLocation(programId, "u_Color");
        if (loc == -1) return;

        glUniform3f(loc,
            static_cast<float>(vec.x),
            static_cast<float>(vec.y),
            static_cast<float>(vec.z)
        );
    }
}