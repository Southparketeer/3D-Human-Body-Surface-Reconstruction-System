#pragma once
#include <GL/glew.h>
#include <GL/freeglut.h>
#include <iostream>
#include <cuda_runtime.h>
#include "ppl.h"


class GLdisplaySimple
{
public:
	GLdisplaySimple(int w , int h):gl_width(w), gl_height(h)
	{
		gl_depth = NULL;
		gl_Windows_Width = gl_width;
		gl_Windows_Height = gl_height;
		gl_fbo_tex_depth = 0;
	}
	~GLdisplaySimple()
	{

	}
	void setDepth(float * in_depth);

	void init();
	void display(float * in_depth);
	void setDisplayTexture(BYTE * img, GLuint& texture_id);

	const int gl_width;
	const int gl_height;

	BYTE * gl_depth;

	GLuint gl_fbo_tex_depth;

	int gl_Windows_Width;
	int gl_Windows_Height;
	const static int ratio = 0.75;

	void displayTexture2D(int x, int y, int w, int h, float scale, GLint tex)
	{
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, tex);
		glBegin(GL_QUADS);
		{
			glTexCoord2f(1, 0);	glVertex2f(x, y);	//	left-top
			glTexCoord2f(1, 1);	glVertex2f(x, y + h * scale);	//	left-bot
			glTexCoord2f(0, 1);	glVertex2f(x + w * scale, y + h * scale);//	right-bot
			glTexCoord2f(0, 0);	glVertex2f(x + w * scale, y);	//	right-top
		}
		glEnd();
		glDisable(GL_TEXTURE_2D);
	}
	void windowsDraw(GLuint texture_depth)
	{
		glClearColor (0.0, 0.0, 0.0, 0.0);
		glClearDepth (1.0);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glViewport(0, 0, gl_Windows_Width, gl_Windows_Height);
		glMatrixMode( GL_PROJECTION );
		glLoadIdentity();
		glOrtho(0, gl_Windows_Width, gl_Windows_Height, 0.0f, 0.0f, 100.0f);
		glMatrixMode( GL_MODELVIEW );
		glLoadIdentity();
		glEnable(GL_TEXTURE_2D);
		displayTexture2D(0, 0, gl_width, gl_height, 1.f, texture_depth);
		glDisable(GL_TEXTURE_2D);
	}
};

void GLdisplaySimple::init()
{
	if(gl_depth == NULL)
		gl_depth = new BYTE[gl_width * gl_height * 4];
}


void GLdisplaySimple::setDepth(float * in_depth)
{
	Concurrency::parallel_for(0, gl_width * gl_height, [&](int k)
	{
		gl_depth[k * 4 + 0] = (BYTE)in_depth[k];
		gl_depth[k * 4 + 1] = (BYTE)in_depth[k];
		gl_depth[k * 4 + 2] = (BYTE)in_depth[k];
		gl_depth[k * 4 + 3] = 255;
	});
}


void GLdisplaySimple::setDisplayTexture(BYTE* img , GLuint& texture_id)
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



void GLdisplaySimple::display(float * in_depth)
{
	setDepth(in_depth);
	setDisplayTexture(this->gl_depth, this->gl_fbo_tex_depth);
	windowsDraw(this->gl_fbo_tex_depth);
	glutSwapBuffers();
}