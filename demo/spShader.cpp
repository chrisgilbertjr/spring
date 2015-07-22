
#include "spShader.h"

const char* vertexShape = 
    "#version 330\n"

    "uniform mat4 v_transform;\n"

    "in  vec3 v_barycentric;\n"
    "in  vec2 v_position;\n"
    "in  vec2 v_aliasing;\n"
    "in  vec4 v_border;\n"
    "in  vec4 v_color;\n"

    "out vec3 p_barycentric;\n"
    "out vec2 p_aliasing;\n"
    "out vec4 p_border;\n"
    "out vec4 p_color;\n"

    "void main()\n"
    "{\n"
    "    mat4 transform = mat4(1.0f, 0.0f, 0.0f, 0.0f,\n"
    "                          0.0f, 1.0f, 0.0f, 0.0f,\n"
    "                          0.0f, 0.0f, 1.0f, 0.0f,\n"
    "                          0.0f, 0.0f, 0.0f, 1.0f);\n"
    "    p_color       = v_color;\n"
    "    p_border      = v_border;\n"
    "    p_aliasing    = v_aliasing;\n"
    "    p_barycentric = v_barycentric;\n"
    "    gl_Position   = v_transform * vec4(v_position, 0.0f, 1.0f);\n"
    "};\n";

const char* pixelShape = 
    "#version 330\n"

    //"uniform float v_size;\n"

    "in  vec3 p_barycentric;\n"
    "in  vec2 p_aliasing;\n"
    "in  vec4 p_border;\n"
    "in  vec4 p_color;\n"

    "out vec4 pixel;\n"

    "void main(void)\n"
    "{\n"
    "    float size = length(fwidth(p_aliasing));\n"
    "    if (p_barycentric.y == 1.0f) { size = size * 1.2f; }\n"
    "    float lerp   = length(p_aliasing);\n"
    "    float border = 1.0 - size;\n"
    "    float alpha  = 1.0 - smoothstep(border, 1.0, lerp);\n"

    "    lerp   = smoothstep(max(border - size, 0.0), border, lerp);\n"
    "    if (p_barycentric.x > border) { lerp = 0.0f; alpha = 1.0f; }\n"
    /// FIX THIS INTO AN IF ELSE BLOCK SO WE DONT ALWAYS MIX
    "    pixel  = mix(p_color, p_border, lerp) * alpha;\n"
    "}\n";

    //"void main(void)\n"
    //"{\n"
    //"    vec4 outline = vec4(0.0f, 0.0f, 0.0f, 1.0f);\n"

    //"    float size = length(fwidth(p_aliasing));\n"
    //"    if (size <= 1e-5)\n"
    //"    {\n"
    //"        vec4 color = p_color;\n"
    //"        vec3 width = fwidth(p_barycentric);\n"
    //"        float l = length(width);\n"
    //"        vec3 alpha = smoothstep(vec3(0.0), width*2.0, p_barycentric);\n"
    //"        float lerp = min(min(alpha.x, alpha.y), alpha.z);\n"
    //"        float a = 1.0f;\n"
    //"        float q = 1.0 - lerp;"
    //"        float o = 0.0;\n"
    //"        vec3 b = p_barycentric;\n"
    //"        float mb = max(max(b.x, b.y), b.z);\n"

    ////"        if (mb > lerp)\n"
    ////"        {\n"
    ////"            a = 1.0f - smoothstep(0.65f, 1.0f, q);\n"
    ////"            float t = (mb - lerp)/q;\n"
    ////"            color = mix(p_color, p_border, t);\n"
    ////"        }\n"

    //"        if (p_barycentric.z > lerp)\n"
    //"        {\n"
    //"            a = 1.0f - smoothstep(0.65f, 1.0f, q);\n" 
    //"            float t = (p_barycentric.z - lerp)/q;\n"
    //"            color = mix(p_color, p_border, a);\n"
    //"        }\n"
    //"        if (p_barycentric.y > lerp)\n"
    //"        {\n"
    //"            a = 1.0f - smoothstep(0.65f, 1.0f, q);\n"
    //"            float t = (p_barycentric.y - lerp)/q;\n"
    //"            color = mix(p_color, p_border, a);\n"
    //"        }\n"
    //"        if (p_barycentric.x > lerp)\n"
    //"        {\n"
    //"            a = 1.0f - smoothstep(0.65f, 1.0f, q);\n"
    //"            float t = (p_barycentric.x - lerp)/q;\n"
    //"            color = mix(p_color, p_border, a);\n"
    //"        }\n"

    //"        pixel = color * a;\n"

    //"    }\n"

    //"    else\n"
    //"    {\n"
    //"       float lerp   = length(p_aliasing);\n"
    //"       float border = 1.0 - size;\n"
    //"       float alpha  = 1.0 - smoothstep(border, 1.0, lerp);\n"

    //"       lerp   = smoothstep(max(border - size, 0.0), border, lerp);\n"
    //"       pixel  = mix(p_color, p_border, lerp) * alpha;\n"
    //"    }\n"
    //"}\n";
