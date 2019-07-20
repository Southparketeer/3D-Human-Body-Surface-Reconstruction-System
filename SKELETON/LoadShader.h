#pragma once

#include <GL/glew.h>
#include <string>
#include <iostream>
#include <fstream>
#include <sys/stat.h>
#include <math.h>

class Shader
{
public:
	Shader(std::string fName)
	{
		shaderId=createShader(fName);
	}
	GLuint getShaderId()
	{
		return shaderId;
	}
private:
	GLuint createShader(std::string fName);
	void printProgramInfoLog(GLuint obj);
	void printShaderInfoLog(GLuint obj);
	GLuint shaderId;
	std::string g_compile_link_log;
};

//----------------------------------------Implementation------------------------------------------//

GLuint Shader::createShader(std::string fName)
{
	GLuint vertex_shader = glCreateShader(GL_VERTEX_SHADER);
	GLuint pixel_shader = glCreateShader(GL_FRAGMENT_SHADER);
	
	
	// obtain file sizes:
	int fsize1, fsize2, bytes_read;
	std::string vert_shader = fName + ".vert";
	std::string frag_shader = fName + ".frag";
    
	struct stat statBuf;
	stat(vert_shader.c_str(), &statBuf);
	fsize1 = statBuf.st_size;
	stat(frag_shader.c_str(), &statBuf);
	fsize2 = statBuf.st_size;
	
	FILE *f1 = fopen(vert_shader.c_str(), "r");
	FILE *f2 = fopen(frag_shader.c_str(), "r");
	if (!f1 || !f2)
		return 0;
	
	// allocate memory to contain the whole file:
#undef max
	GLchar * shader_buffer[1];
	shader_buffer[0] = (GLchar*)malloc(std::max(fsize1, fsize2));
	
	// Read in the fragment shader
	bytes_read = fread ((void*)shader_buffer[0],1,fsize2,f2);
	glShaderSource(pixel_shader, 1, (const GLchar**)shader_buffer, &bytes_read);
	
	// Read in the vertex shader
	bytes_read = fread ((void*)shader_buffer[0],1,fsize1,f1);
	glShaderSource(vertex_shader, 1, (const GLchar**)shader_buffer, &bytes_read);
	
	free(shader_buffer[0]);
	
	GLint status;
	glCompileShader(vertex_shader);
	glGetShaderiv(vertex_shader, GL_COMPILE_STATUS, &status);
	if (status == GL_FALSE)
	{
		g_compile_link_log += vert_shader + ": ";
		printShaderInfoLog(vertex_shader);
		return 0;
	}
	glCompileShader(pixel_shader);
	glGetShaderiv(pixel_shader, GL_COMPILE_STATUS, &status);
	if (status == GL_FALSE)
	{
		g_compile_link_log += frag_shader + ": ";
		printShaderInfoLog(pixel_shader);
		return 0;
	}
	
	GLuint shader_program = glCreateProgram();
	glAttachShader(shader_program,vertex_shader);
	glAttachShader(shader_program,pixel_shader);
    
	glLinkProgram(shader_program);
	glGetProgramiv(shader_program, GL_LINK_STATUS, &status);
	if (status == GL_FALSE)
	{
		g_compile_link_log += fName + " link error: ";
		printProgramInfoLog(shader_program);
	}
    
	return shader_program;
}
void Shader::printProgramInfoLog(GLuint obj)
{
	int infologLength = 0;
	int charsWritten  = 0;
	char *infoLog;
	
	glGetProgramiv(obj, GL_INFO_LOG_LENGTH,&infologLength);
	
	if (infologLength > 0)
	{
		infoLog = (char *)malloc(infologLength);
		glGetProgramInfoLog(obj, infologLength, &charsWritten, infoLog);
		fprintf(stderr,"%s\n",infoLog);
		g_compile_link_log += std::string("\n") + infoLog;
		free(infoLog);
	}
}
void Shader::printShaderInfoLog(GLuint obj)
{
	int infologLength = 0;
	int charsWritten  = 0;
	char *infoLog;
	
	glGetShaderiv(obj, GL_INFO_LOG_LENGTH,&infologLength);
	
	if (infologLength > 0)
	{
		infoLog = (char *)malloc(infologLength);
		glGetShaderInfoLog(obj, infologLength, &charsWritten, infoLog);
		fprintf(stderr, "%s\n",infoLog);
		g_compile_link_log += std::string("\n") + infoLog;
		free(infoLog);
	}
}