#ifndef SHADER_H
#define SHADER_H

#include <glad/glad.h>

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <stdlib.h>

class Shader
{

public:
    unsigned int ID; // program ID
    
    Shader(const char* vertexPath, const char* fragmentPath)
    {
        std::string vertexCode;
        std::string fragmentCode;
        std::ifstream vShaderFile;
        std::ifstream vFragmentFile;

        vShaderFile.exceptions( std::ifstream::failbit | std::ifstream::badbit );
        vFragmentFile.exceptions( std::ifstream::failbit | std::ifstream::badbit );
        
        try
        {
            // open files
            vShaderFile.open(vertexPath);
            vFragmentFile.open(fragmentPath);
            std::stringstream vShaderStream, fShaderStream;

            // read file buffer
            vShaderStream << vShaderFile.rdbuf();
            fShaderStream << vFragmentFile.rdbuf();
            vShaderFile.close();
            vFragmentFile.close();

            vertexCode = vShaderStream.str();
            fragmentCode = fShaderStream.str();
        }
        catch(std::ifstream::failure e)
        {
            std::cout << "ERROR::SHADER::FILE_NOT_SUCCESSFULLY_READ" << std::endl;
            exit(1);
        }
        
        const char* vShaderCode = vertexCode.c_str();
        const char* fShaderCode = fragmentCode.c_str();

        unsigned int vertex, fragment;
        int success;
        char infoLog[512];

        vertex = glCreateShader(GL_VERTEX_SHADER);
        glShaderSource(vertex, 1, &vShaderCode, NULL);
        glCompileShader(vertex);
        glGetShaderiv(vertex, GL_COMPILE_STATUS, &success);
        if (!success)
        {
            glGetShaderInfoLog(vertex, 512, NULL, infoLog);
            std::cout << "ERROR::VERTEX_SHADER::COMPILATION_FAILED" << infoLog << std::endl;
            exit(1);
        }

        fragment = glCreateShader(GL_FRAGMENT_SHADER);
        glShaderSource(fragment, 1, &fShaderCode, NULL);
        glCompileShader(fragment);
        glGetShaderiv(fragment, GL_COMPILE_STATUS, &success);
        if (!success)
        {
            glGetShaderInfoLog(fragment, 512, NULL, infoLog);
            std::cout << "ERROR::FRAGMENT_SHADER::COMPILATION_FAILED" << infoLog << std::endl;
            exit(1);
        }

        ID = glCreateProgram();
        glAttachShader(ID, vertex);
        glAttachShader(ID, fragment);
        glLinkProgram(ID);
        glGetProgramiv(ID, GL_LINK_STATUS, &success);
        if (!success)
        {
            glGetProgramInfoLog(ID, 512, NULL, infoLog);
            std::cout << "ERROR::PROGRAM::LINK_FAILED" << infoLog << std::endl;
            exit(1);
        }

        glDeleteShader(vertex);
        glDeleteShader(fragment);
    }

    void use()
    {
        glUseProgram(ID);
    }

    // utility unifrom functions
    void setBool(const std::string &name, bool value) const
    {
        glUniform1i(glGetUniformLocation(ID, name.c_str()), (int) value);
    }
    void setInt(const std::string &name, int value) const
    {
        glUniform1i(glGetUniformLocation(ID, name.c_str()), value);
    }
    void setFloat(const std::string &name, float value) const
    {
        glUniform1f(glGetUniformLocation(ID, name.c_str()), value);
    }
    void setFloatMat4(const std::string& name, float* value)
    {
        glUniformMatrix4fv(glGetUniformLocation(ID, name.c_str()), 1, GL_FALSE, value);
    }
    void setFloatMat3(const std::string& name, float const * value)
    {
        glUniformMatrix3fv(glGetUniformLocation(ID, name.c_str()), 1, GL_FALSE, value);
    }
    void setVec2(const std::string& name, float* value)
    {
        glUniform2fv(glGetUniformLocation(ID, name.c_str()), 1, value);
    }
    void setVec3(const std::string& name, float* value)
    {
        glUniform3fv(glGetUniformLocation(ID, name.c_str()), 1, value);
    }
    void setVec3(const std::string& name, float x, float y, float z)
    {
        glUniform3f(glGetUniformLocation(ID, name.c_str()), x, y, z);
    }
    void setVec3s(const std::string& name, int count, float* values) {
        glUniform3fv(glGetUniformLocation(ID, name.c_str()), count, values);
    }
    void setVec4(const std::string& name, float* value)
    {
		glUniform4fv(glGetUniformLocation(ID, name.c_str()), 1, value);
	}
    void setVec4(const std::string& name, float x, float y, float z, float w)
    {
		glUniform4f(glGetUniformLocation(ID, name.c_str()), x, y, z, w);
	}
};

#endif
