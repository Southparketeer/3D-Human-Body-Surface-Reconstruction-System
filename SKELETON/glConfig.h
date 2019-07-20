#include <GL/glew.h>
#include <GL/freeglut.h>
#include "LoadShader.h"
#include "Model.h"
#include <iostream>

class glConfig
{
public:

	glConfig();
	~glConfig();
	void init( int argc, char** argv );
	void init_fbo();
	void getDepth(Model * model, float *depth, bool is_front = true);

	GLuint gl_color_tex;
	GLuint gl_fb;
	GLuint gl_depth_rb;

	GLuint gl_VertexVBOID;
	GLuint gl_IndexVBOID;

	Shader *gl_shader1;
	int width, height;

	float PMatrix[16];
	float4 K;
	float4 Kinv;
};


//---------------------------- Implementation --------------------------------------//

glConfig::glConfig()
{
	gl_color_tex=0;
	gl_fb=0;
	gl_depth_rb=0;

	gl_VertexVBOID=0;
	gl_IndexVBOID=0;

	gl_shader1 = NULL;

	width = 512; 
	height = 424;

	int n = 1000;
	int f = 3000;

	float mat[16] = {
		0.715 * 2,	0,				0,					0,
		0,			0.864 * 2,		0,					0,
		0,			0,			   (n+f)/(n-f),		   -1,
		0,			0,			   (2*n*f)/(n-f),		0
	};
	K.y = 0.715;
	K.y = 0.864;
	K.z = 0.5;
	K.w = 0.5;
	Kinv.x = 1.3986;
	Kinv.y = 1.1574;
	Kinv.z = -0.6993;
	Kinv.w = -0.5787;

	memcpy(PMatrix, mat, 16 * sizeof(float));
}
glConfig::~glConfig()
{
	//cout<<"~glConfig()"<<endl;
	glDeleteRenderbuffersEXT(1, &gl_depth_rb);
	glDeleteTextures(1, &gl_color_tex);
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
	glDeleteFramebuffersEXT(1, &gl_fb);
}
void glConfig::init(int argc, char** argv )
{
	glutInit( &argc, argv );
	glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGB |GLUT_DEPTH );
	glutInitWindowSize( width, height ); 
	//glutInitWindowPosition( 100, 100 );
	glutCreateWindow( argv[0] );
	glutHideWindow();
	glewInit();
	gl_shader1 = new Shader("Source//SKELETON//phong");
	init_fbo();
}

void glConfig::init_fbo()
{
	glGenTextures(1, &gl_color_tex);
	glBindTexture(GL_TEXTURE_2D, gl_color_tex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, width, height, 0, GL_BGRA, GL_UNSIGNED_BYTE, NULL);

	//-------------------------
	glGenFramebuffersEXT(1, &gl_fb);
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, gl_fb);
	//Attach 2D texture to this FBO
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_2D, gl_color_tex, 0);
	//-------------------------
	glGenRenderbuffersEXT(1, &gl_depth_rb);
	glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, gl_depth_rb);
	glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_DEPTH_COMPONENT24, width, height);
	//-------------------------
	//Attach depth buffer to FBO
	glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_RENDERBUFFER_EXT, gl_depth_rb);
	//-------------------------
	//Does the GPU support current FBO configuration?
	GLenum status;
	status = glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT);
	switch(status)
	{
	case GL_FRAMEBUFFER_COMPLETE_EXT:
		std::cout<<"good"<<std::endl;
		break;
	default:
		std::cout<<"error"<<std::endl;
	}
	glBindTexture(GL_TEXTURE_2D, 0);
	glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, 0);
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
}

void glConfig::getDepth(Model * model, float *depth, bool is_front)
{
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, gl_fb);
	glClearColor(0.0, 0.0, 0.0, 0.0);
	glClearDepth(1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glViewport(0, 0, width, height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glMultMatrixf(PMatrix);
	//gluPerspective(45.0, (GLfloat)this->width/(GLfloat)this->height, 1.0, 4500.0);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	//float a = angle * PI / 180.;
	if(is_front)
		gluLookAt( 0, 1000, -2000, 0 , 1000 , 0 , 0 , 1 , 0 );
	else
		gluLookAt( 0, 1000,	2000, 0 , 1000 , 0 , 0 , 1 , 0 );
	glEnable(GL_DEPTH_TEST);
	glDisable(GL_CULL_FACE);
	model->BindVertexBuffer();
	model->renderModel(gl_shader1->getShaderId());
	glReadPixels(0,0,width, height, GL_RED, GL_FLOAT, depth);
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
}