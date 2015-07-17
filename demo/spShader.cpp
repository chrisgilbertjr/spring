
#include "spShader.h"

const char* vertexShape = 
    "#version 330\n"

    "in  vec3 v_barycentric;\n"
    "in  vec2 v_aliasing;\n"
    "in  vec2 v_position;\n"

    "out vec3 p_barycentric;\n"
    "out vec2 p_aliasing;\n"
    "out vec4 p_color;\n"

    "void main()\n"
    "{\n"
    "    p_color       = vec4(0.0f, 0.5f, 0.2f, 1.0f);\n"
    "    p_aliasing    = v_aliasing;\n"
    "    p_barycentric = v_barycentric;\n"
    "    gl_Position   = vec4(v_position, 0.0f, 1.0f);\n"
    "};\n";

const char* pixelShape = 
    "#version 330\n"

    "in  vec3 p_barycentric;\n"
    "in  vec2 p_aliasing;\n"
    "in  vec4 p_color;\n"

    "out vec4 pixel;\n"

    "void main(void)\n"
    "{\n"
    "    vec4 outline = vec4(0.0f, 0.0f, 0.0f, 1.0f);\n"

    "    float size = length(fwidth(p_aliasing));\n"
    "    if (size <= 1e-5)\n"
    "    {\n"
    //"        vec4 color = p_color;\n"
    //"        vec3 width = fwidth(p_barycentric);\n"
    //"        vec3 alpha = smoothstep(vec3(0.0), width*3.0, p_barycentric);\n"
    //"        float lerp = min(min(alpha.x, alpha.y), alpha.z);\n"
    //"        pixel = mix(outline, color, lerp);\n"

    "        vec4 color = p_color;\n"
    "        vec3 width = fwidth(p_barycentric);\n"
    "        float l = length(width);\n"
    "        vec3 alpha = smoothstep(vec3(0.0), width*2.0, p_barycentric);\n"
    "        float lerp = min(min(alpha.x, alpha.y), alpha.z);\n"
    "        float a = 1.0f;\n"
    "        float q = 1.0 - lerp;"
    "        float o = 0.0;\n"

    "        if (p_barycentric.z > lerp)\n"
    "        {\n"
    "            a = 1.0f - smoothstep(0.65f, 1.0f, q);\n"
    "            float t = (p_barycentric.z - lerp)/q;\n"
    "            color = mix(outline, p_color, a);\n"
    "        }\n"
    "        if (p_barycentric.y > lerp)\n"
    "        {\n"
    "            a = 1.0f - smoothstep(0.65f, 1.0f, q);\n"
    "            float t = (p_barycentric.y - lerp)/q;\n"
    "            color = mix(p_color, outline, a);\n"
    "        }\n"
    "        if (p_barycentric.x > lerp)\n"
    "        {\n"
    "            a = 1.0f - smoothstep(0.65f, 1.0f, q);\n"
    "            float t = (p_barycentric.x - lerp)/q;\n"
    "            color = mix(p_color, outline, a);\n"
    "        }\n"

    "        pixel = color * a;\n"

    "    }\n"

    "    else\n"
    "    {\n"
    "       float lerp   = length(p_aliasing);\n"
    "       float border = 1.0 - size;\n"
    "       float alpha  = 1.0 - smoothstep(border, 1.0, lerp);\n"

    "       lerp   = smoothstep(max(border - size, 0.0), border, lerp);\n"
    "       pixel  = mix(p_color, outline, lerp) * alpha;\n"
    "    }\n"
    "}\n";
