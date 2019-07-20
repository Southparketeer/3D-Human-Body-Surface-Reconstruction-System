#pragma once
#include "..\Base.h"

using namespace vcg;
using namespace std;
class pixelTriangle
{
public: 
	static void  ConvertPixel2Triangle(const vector<Point3f>& image, CMeshO& mesh, int w, int h, const int& detailLevel = 1)
	{
		vector<int> vertexIdx(w * h , 0);
		
		mesh.vert.reserve(w * h / 2);

		int k = 0;
		for(int i = 0; i < image.size(); i++)
		{
			if(image[i] == Point3f::Zero())
				continue;
			else
			{
				vertexIdx[i] = k;
				CVertexO cp;
				cp.P() = Point3f(i % w, i / w, 0);
				cp.N() = Point3f(0, 0, 1.);
				cp.C() = Color4b(255, 255, 255, 255);
				mesh.vert.push_back(cp);
			}
			k++;
		}
		mesh.vn = k;
		mesh.face.reserve(k * 2);
		for(int y = 0; y < h - 1; y++)
		{
			for(int x = 0; x < w - 1; x++)
			{
				int a = 0, b = 0, c = 0, d = 0;
				int cnt = 0;

				if(vertexIdx[x + y * w] == 0)
				{
					continue;
				}
				else
				{
					a = x + y * w;
					cnt++;
				}
				if(vertexIdx[x + 1 + y * w] != 0)
				{
					b = x + 1 + y * w;
					cnt++;
				}
				if(vertexIdx[x + (y + 1) * w] != 0)
				{
					c = x + (y + 1) * w;
					cnt++;
				}
				if(vertexIdx[x + 1 + (y + 1) * w] != 0)
				{
					d = x + 1 + (y + 1) * w;
					cnt++;
				}

				if(cnt < 3)
					continue;
				if(cnt == 3)
				{
					if(b == 0)
					{
						CFaceO cf;
						cf.V(0) = mesh.vert.data() + vertexIdx[a];
						cf.V(1) = mesh.vert.data() + vertexIdx[d];
						cf.V(2) = mesh.vert.data() + vertexIdx[c];
						mesh.face.push_back(cf);
					}
					else if(c == 0)
					{
						CFaceO cf;
						cf.V(0) = mesh.vert.data() + vertexIdx[a];
						cf.V(1) = mesh.vert.data() + vertexIdx[b];
						cf.V(2) = mesh.vert.data() + vertexIdx[d];
						mesh.face.push_back(cf);
					}
					else if(d == 0)
					{
						CFaceO cf;
						cf.V(0) = mesh.vert.data() + vertexIdx[a];
						cf.V(1) = mesh.vert.data() + vertexIdx[b];
						cf.V(2) = mesh.vert.data() + vertexIdx[c];
						mesh.face.push_back(cf);
					}
				}
				else if(cnt == 4)
				{
					CFaceO cf1;
					cf1.V(0) = mesh.vert.data() + vertexIdx[a];
					cf1.V(1) = mesh.vert.data() + vertexIdx[b];
					cf1.V(2) = mesh.vert.data() + vertexIdx[d];
					mesh.face.push_back(cf1);
					CFaceO cf2;
					cf2.V(0) = mesh.vert.data() + vertexIdx[a];
					cf2.V(1) = mesh.vert.data() + vertexIdx[d];
					cf2.V(2) = mesh.vert.data() + vertexIdx[c];
					mesh.face.push_back(cf2);
				}
			}
		}
		mesh.fn = mesh.face.size();
	}
};