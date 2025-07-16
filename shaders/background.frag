#version 330 core

in vec2 TexCoord;
out vec4 FragColor;

uniform sampler2D u_texture;

void main()
{
    // For testing, show a gradient background if no valid texture
    vec4 texColor = texture(u_texture, TexCoord);
    
    // If texture is black (no camera data), show a test pattern
    if (texColor.r + texColor.g + texColor.b < 0.1) {
        // Create a simple gradient test pattern
        FragColor = vec4(TexCoord.x, TexCoord.y, 0.5, 1.0);
    } else {
        FragColor = texColor;
    }
}
