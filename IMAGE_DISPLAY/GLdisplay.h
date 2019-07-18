#pragma once
#include <GL/glew.h>
#include <GL/freeglut.h>
#include <iostream>
#include <cuda_runtime.h>
#include "ppl.h"

#define RATIO 0.75

#ifndef SAFE_DELETE
#define SAFE_DELETE(p) { if (p) { delete (p); (p)=NULL; } }
#endif

class GLdisplay
{
public:
	GLdisplay(int w , int h):gl_width(w), gl_height(h)
	{
		gl_normal = NULL;
		gl_render = NULL;
		gl_depth_K0 = NULL;
		gl_depth_K1 = NULL;
		gl_Windows_Width = 3 * gl_width;
		gl_Windows_Height = 2 * gl_height;
		gl_fbo_tex_depth_K0 = 0;
		gl_fbo_tex_depth_K1 = 0;
		gl_fbo_tex_normal = 0;
		gl_fbo_tex_render = 0;
	}
	~GLdisplay()
	{
		SAFE_DELETE(gl_normal);
		SAFE_DELETE(gl_render);
		SAFE_DELETE(gl_depth_K0);
		SAFE_DELETE(gl_depth_K1);
		glDeleteTextures(1, &gl_fbo_tex_depth_K0);
		glDeleteTextures(1, &gl_fbo_tex_depth_K1);
		glDeleteTextures(1, &gl_fbo_tex_normal);
		glDeleteTextures(1, &gl_fbo_tex_render);
	}
	void setDepth(float * in_depth, BYTE * depth);
	void setNormal(float4 * in_normal);
	void setRender(float * in_render);

	void init();
	void display(float * in_depth_K0, float * in_depth_K1, float4* in_normal, float* in_render, bool if_normal);
	void setDisplayTexture(BYTE * img, GLuint& texture_id);

	const int gl_width;
	const int gl_height;

	BYTE * gl_normal;
	BYTE * gl_render;
	BYTE * gl_depth_K0;
	BYTE * gl_depth_K1;

	GLuint gl_fbo_tex_depth_K0;
	GLuint gl_fbo_tex_depth_K1;
	GLuint gl_fbo_tex_normal;
	GLuint gl_fbo_tex_render;

	int gl_Windows_Width;
	int gl_Windows_Height;
	const static int ratio = 0.75;
	void displayTexture2D(int x, int y, int w, int h, float scale, GLint tex)
	{
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, tex);
		glBegin(GL_QUADS);
		{
			glTexCoord2f(0, 0);	glVertex2f(x, y);	//	left-top
			glTexCoord2f(0, 1);	glVertex2f(x, y + h * scale);	//	left-bot
			glTexCoord2f(1, 1);	glVertex2f(x + w * scale, y + h * scale);//	right-bot
			glTexCoord2f(1, 0);	glVertex2f(x + w * scale, y);	//	right-top
		}
		glEnd();
		glDisable(GL_TEXTURE_2D);
	}
	void windowsDraw(GLuint texture_depth, GLuint texture_normal, GLuint texture_render)
	{

		glClearColor (0.0, 0.0, 0.0, 0.0);
		glClearDepth (1.0);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glViewport(0, 0, gl_Windows_Width * RATIO, gl_Windows_Height * RATIO);
		glMatrixMode( GL_PROJECTION );
		glLoadIdentity();
		glOrtho(0, gl_Windows_Width * RATIO, gl_Windows_Height * RATIO, 0.0f, 0.0f, 100.0f);
		glMatrixMode( GL_MODELVIEW );
		glLoadIdentity();
		glEnable(GL_TEXTURE_2D);
		displayTexture2D(gl_width * 2 * RATIO, 0, gl_width * RATIO, gl_height * RATIO, 1.f, texture_depth);
		displayTexture2D(gl_width * 2 * RATIO, gl_height * RATIO, gl_width * RATIO, gl_height * RATIO, 1.f, texture_normal);
		displayTexture2D(0, 0, gl_width * 2 * RATIO, gl_height * 2 * RATIO, 1.f, texture_render);
		glDisable(GL_TEXTURE_2D);
	}
};

void GLdisplay::init()
{
	if(gl_depth_K0 == NULL)
		gl_depth_K0 = new BYTE[gl_width * gl_height * 4];
	if(gl_depth_K1 == NULL)
		gl_depth_K1 = new BYTE[gl_width * gl_height * 4];
	if(gl_normal == NULL)
		gl_normal = new BYTE[gl_width * gl_height * 4];
	if(gl_render == NULL)
		gl_render = new BYTE[gl_width * gl_height * 4];
}


void GLdisplay::setDepth(float * in_depth, BYTE * depth)
{
	assert(gl_depth);

	Concurrency::parallel_for(0, gl_width * gl_height, [&](int k)
	{
		if(in_depth[k] == UINT16_MAX || in_depth[k] == 0) 
		{
			depth[k * 4 + 0] = 255;
			depth[k * 4 + 1] = 0;
			depth[k * 4 + 2] = 0;
			depth[k * 4 + 3] = 255;
		}
		else
		{
			depth[k * 4 + 0] = (BYTE)in_depth[k];
			depth[k * 4 + 1] = (BYTE)in_depth[k];
			depth[k * 4 + 2] = (BYTE)in_depth[k];
			depth[k * 4 + 3] = 255;
		}
	});
}

void GLdisplay::setNormal(float4 * in_normal)
{
	assert(gl_normal);
	Concurrency::parallel_for(0, gl_width * gl_height, [&](int k)
	{
		gl_normal[k * 4 + 0] = (BYTE)(fabsf(in_normal[k].x) * 255);
		gl_normal[k * 4 + 1] = (BYTE)(fabsf(in_normal[k].y) * 255);
		gl_normal[k * 4 + 2] = (BYTE)(fabsf(in_normal[k].z) * 255);
		gl_normal[k * 4 + 3] = 255;
	});
}

void GLdisplay::setRender(float * in_render)
{
	assert(gl_render);
	Concurrency::parallel_for(0, gl_width * gl_height, [&](int k)
	{
		gl_render[k * 4 + 0] = (BYTE)((in_render[k]) * 255);
		gl_render[k * 4 + 1] = (BYTE)((in_render[k]) * 255);
		gl_render[k * 4 + 2] = (BYTE)((in_render[k]) * 255);
		gl_render[k * 4 + 3] = 255;
	});
}

void GLdisplay::setDisplayTexture(BYTE* img , GLuint& texture_id)
{
	if(texture_id== 0)
	{
		glGenTextures(1,&texture_id);
	}
	glBindTexture(GL_TEXTURE_2D, texture_id);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, gl_width, gl_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, img);
	glBindTexture(GL_TEXTURE_2D, 0);
}



void GLdisplay::display(float * in_depth_K0, float * in_depth_K1, float4* in_normal, float* in_render, bool if_normal)
{
	setDepth(in_depth_K0, this->gl_depth_K0);
	setDisplayTexture(this->gl_depth_K0, this->gl_fbo_tex_depth_K0);
	setDepth(in_depth_K1, this->gl_depth_K1);
	setDisplayTexture(this->gl_depth_K1, this->gl_fbo_tex_depth_K1);
	setNormal(in_normal);
	setDisplayTexture(this->gl_normal, this->gl_fbo_tex_normal);
	setRender(in_render);
	setDisplayTexture(this->gl_render, this->gl_fbo_tex_render);
	if(!if_normal)
		windowsDraw(this->gl_fbo_tex_depth_K0, this->gl_fbo_tex_depth_K1, this->gl_fbo_tex_render);
	else
		windowsDraw(this->gl_fbo_tex_depth_K0, this->gl_fbo_tex_depth_K1, this->gl_fbo_tex_normal);
	glutSwapBuffers();
}