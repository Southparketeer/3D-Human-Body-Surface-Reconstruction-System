varying vec4 v;
void main(void)
{
    float a =  -v.z;
    gl_FragColor = vec4( a );
}