
varying vec4 v;
void main()
{
	v= gl_ModelViewMatrix * gl_Vertex;
    gl_Position = ftransform();
}
