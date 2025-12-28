#version 330

// Input vertex attributes (from vertex shader)
in vec3 fragPosition;
in vec2 fragTexCoord;
in vec3 fragNormal;
in vec4 fragColor;

// Input uniform values
uniform sampler2D texture0;
uniform vec4 colDiffuse;

// Custom uniforms for lighting
uniform vec3 viewPos;
uniform vec3 lightDir;
uniform vec4 lightColor;
uniform vec4 ambientColor;
uniform int useGrid;

// Output fragment color
out vec4 finalColor;

float grid(vec3 pos, float spacing) {
    vec2 coord = pos.xz / spacing;
    vec2 grid = abs(fract(coord - 0.5) - 0.5) / fwidth(coord);
    float line = min(grid.x, grid.y);
    return 1.0 - min(line, 1.0);
}

void main()
{
    // Texture color
    vec4 texelColor = texture(texture0, fragTexCoord);
    
    // Base color
    vec4 baseColor = texelColor * colDiffuse * fragColor;
    
    if (useGrid > 0) {
        float g1 = grid(fragPosition, 1.0);
        float g2 = grid(fragPosition, 10.0);
        float g = max(g1 * 0.5, g2);
        
        // Mix grid color (dark gray) with base color, subtle but visible
        baseColor.rgb = mix(baseColor.rgb, vec3(0.4), g * 0.35);
    }
    
    // Lighting
    vec3 normal = normalize(fragNormal);
    vec3 viewDir = normalize(viewPos - fragPosition);
    vec3 lightDirN = normalize(-lightDir);

    // Ambient
    vec3 ambient = ambientColor.rgb * ambientColor.a;

    // Diffuse
    float diff = max(dot(normal, lightDirN), 0.0);
    vec3 diffuse = diff * lightColor.rgb * lightColor.a;

    // Specular (Blinn-Phong)
    vec3 halfwayDir = normalize(lightDirN + viewDir);
    float spec = pow(max(dot(normal, halfwayDir), 0.0), 32.0);
    vec3 specular = vec3(0.5) * spec * lightColor.rgb; // Specular color same as light

    vec3 lighting = (ambient + diffuse + specular) * baseColor.rgb;    
    
    finalColor = vec4(lighting, baseColor.a);
}
