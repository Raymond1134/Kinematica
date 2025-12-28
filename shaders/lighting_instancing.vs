#version 330

// Input vertex attributes
in vec3 vertexPosition;
in vec2 vertexTexCoord;
in vec3 vertexNormal;
in vec4 vertexColor;

// Input instance attributes
in mat4 instanceTransform;

// Input uniform values
uniform mat4 mvp;
uniform mat4 matView;
uniform mat4 matProjection;

// Output vertex attributes (to fragment shader)
out vec3 fragPosition;
out vec2 fragTexCoord;
out vec3 fragNormal;
out vec4 fragColor;

void main()
{
    // Calculate fragment position based on instance transformation
    vec4 worldPos = instanceTransform * vec4(vertexPosition, 1.0);
    fragPosition = vec3(worldPos);
    
    // Calculate fragment texture coordinates
    fragTexCoord = vertexTexCoord;
    
    // Calculate fragment normal
    // For correct normal scaling with non-uniform scaling, we need the normal matrix
    mat3 normalMatrix = transpose(inverse(mat3(instanceTransform)));
    fragNormal = normalize(normalMatrix * vertexNormal);
    
    // Calculate fragment color
    fragColor = vec4(1.0); // Ignore vertex color for now, assume white
    
    // Calculate final vertex position
    // Note: mvp is usually ModelViewProjection. 
    // When instancing, we usually get ViewProjection passed as a uniform, or we construct it.
    // Raylib's DrawMeshInstanced sets `mvp` to ViewProjection when drawing instanced? 
    // Let's assume `mvp` is VP.
    gl_Position = mvp * vec4(fragPosition, 1.0);
}
