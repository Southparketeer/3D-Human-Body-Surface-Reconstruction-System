#pragma once

#include <iostream>
#include <vector>
#include <fstream>
#include <GL/glew.h>
#include "..\StdAfx.h"
#include "..\Base.h"

using namespace std;
using namespace Eigen;
using namespace LUY_VEC;

class Model
{
public:
	Model();
	~Model();
	void init();
	void setMeshModel(vcg::CMeshO * in){model = in;};
	vector<LUY_VEC::Vertex>* getVertexBuff(){ return this->buff_vertex; }
	vector<vec3i>* getEdgeBuff(){ return this->buff_edge; }
	void TransformModel( Mat4& RT);
	void BindVertexBuffer();
	void renderModel(GLuint shaderId);
	void fillModel(vcg::CMeshO& cm);
protected:
	vector<LUY_VEC::Vertex> * buff_vertex;
	vector<vec3i>  * buff_edge;
	vcg::CMeshO * model;
	GLuint VertexVBOID;
	GLuint IndexVBOID;
};




//---------------------------- Implementation --------------------------------------//

Model::Model()
{
	buff_vertex = NULL;
	buff_edge = NULL;
	model = NULL;
	VertexVBOID = 0;
	IndexVBOID = 0;
}

Model::~Model()
{
	SAFE_DELETE(this->buff_vertex);
	SAFE_DELETE(this->buff_edge);
	//cout<<"~Model()"<<endl;
}

void Model::init()
{
	SAFE_DELETE(this->buff_vertex);
	SAFE_DELETE(this->buff_edge);
	this->buff_vertex = new vector<LUY_VEC::Vertex>;
	this->buff_edge = new vector<vec3i>;
}

void Model::renderModel(GLuint shaderId)
{
	glUseProgram(shaderId);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);
	glPolygonMode( GL_FRONT_AND_BACK, GL_FILL);

#define BUFFER_OFFSET(i) ((void*)(i))

	glBindBuffer(GL_ARRAY_BUFFER, VertexVBOID);
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3,GL_FLOAT, sizeof(LUY_VEC::Vertex),BUFFER_OFFSET(0));

	glEnableClientState(GL_NORMAL_ARRAY);
	glNormalPointer(GL_FLOAT, sizeof(LUY_VEC::Vertex),BUFFER_OFFSET(12));

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, IndexVBOID);

	glDrawElements(GL_TRIANGLES, buff_edge->size() * 3, GL_UNSIGNED_INT, 0);

	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_NORMAL_ARRAY);

	glUseProgram(0);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0); 
}


void Model::TransformModel( Mat4& RT)
{
	Concurrency::parallel_for( 0u, (UINT) this->buff_vertex->size(), [&](UINT k)
	{
		vec3f pos = this->buff_vertex->at(k).pos;
		vec3f nor = this->buff_vertex->at(k).nor;
		Vet4  P(pos.x, pos.y, pos.z, 1);
		Vet3  N(nor.x, nor.y, nor.z);
		P = RT * P;
		N = RT.block<3,3>(0,0) * N;
		this->buff_vertex->at(k).pos.Set(P.x(), P.y(), P.z());
		this->buff_vertex->at(k).nor.Set(N.x(), N.y(), N.z());
	});
}

void Model::BindVertexBuffer(){
	if(VertexVBOID == 0)	
		glGenBuffers(1,&VertexVBOID);
	//printf( "vid : %d\n", VertexVBOID );
	glBindBuffer(GL_ARRAY_BUFFER, VertexVBOID);
	glBufferData(GL_ARRAY_BUFFER, sizeof(LUY_VEC::Vertex)* this->buff_vertex->size(), buff_vertex->data(), GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	if(IndexVBOID == 0)	
		glGenBuffers(1, &IndexVBOID);
	//printf( "vid : %d\n", IndexVBOID );
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, IndexVBOID);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(vec3i)*this->buff_edge->size(), buff_edge->data(),GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0 );
}


void Model::fillModel(vcg::CMeshO& cm){
	this->buff_vertex->clear();
	this->buff_edge->clear();
	this->buff_vertex->resize(cm.VN());
	this->buff_edge->resize(cm.FN());

	vcg::CMeshO::VertexIterator vi = cm.vert.begin();
	vcg::CMeshO::FaceIterator fi = cm.face.begin();

	vcg::CVertexO* vi_begin = cm.vert.data(); //record the begin position
	vcg::Color4b red = vcg::Color4<unsigned char>::ColorConstant::Red;
	vcg::Color4b green = vcg::Color4<unsigned char>::ColorConstant::Green;

	Concurrency::parallel_for(0u, (UINT)cm.FN(), [&](UINT k){
		vcg::CFaceO currF = cm.face.at(k);
		vcg::CVertexO * edgeV[3] = {currF.V(0), currF.V(1), currF.V(2) };
		vec3i edge(edgeV[0] - vi_begin, edgeV[1] - vi_begin , edgeV[2] - vi_begin);
		this->buff_edge->at(k) = edge;
	});

	Concurrency::parallel_for(0u, (UINT)cm.VN(), [&](UINT k){
		vcg::CVertexO currv = cm.vert.at(k);
		vcg::Point3f position = currv.P();
		vcg::Point3f normal = currv.N();
		vcg::Color4b color = currv.C(); 
		LUY_VEC::Vertex vert;
		vert.pos.Set(position.X(), position.Y(), position.Z());
		vert.nor.Set(normal.X(), normal.Y(), normal.Z());
		if(color.Equal(red)) vert.col.Set(255.,0.,0.);
		else if(color.Equal(green)) vert.col.Set(0.,255.,0.);
		else vert.col.Set(0.,0.,0.);
		buff_vertex->at(k) = vert;
	});
}