$FG_GLSL_VERSION

// Instrument glass in the HDR pipeline (Effects/Glass/instrument-glass): clear glass, which scatters no light of its
// own. Its two faces reflect the Fresnel fraction R of the light (8 % head on, all of it at grazing incidence) and
// pass the rest, so the blend (premultiplied) is: dial x (1 - R) + what the glass reflects. That is the sun, where
// the shadow map lets it reach the glass, and nothing else: FlightGear's environment map holds only the terrain, the
// clouds and the sky, not the aircraft around the instrument, whose dark cockpit is what the glass really mirrors.
// The glass is a mirror: the sun shows in it as its disc (0.27 deg radius, the solid angle of the HDR atmosphere)
// where the reflected line of sight meets it, R times as bright as the sun in FlightGear's sky (oursun.frag: the
// irradiance x 500, capped at 1e4 after exposure for the half-float buffers).

in VS_OUT {
    float flogz;
    vec2 texcoord;
    vec3 vertex_normal;
    vec3 view_vector;
    vec4 material_color;
    vec4 ap_color;
} fs_in;

FG_VIEW_GLOBAL
uniform vec3 fg_SunDirection[FG_NUM_VIEWS];
uniform mat4 fg_ViewMatrixInverse[FG_NUM_VIEWS];

// shadows.glsl
float get_shadowing(vec3 P, vec3 N, vec3 L);
// sun.glsl
vec3 get_sun_radiance(vec3 p);
// exposure.glsl
vec3 apply_exposure(vec3 color);
// logarithmic_depth.glsl
float logdepth_encode(float z);
// forward_pack.glsl
void forward_pack(vec4 color, float flogz);

const float IOR = 1.52;                 // soda-lime glass
const float SUN_SOLID_ANGLE = 6.8e-5;   // sr (atmos_spectral.glsl)

// reflectance of one air-glass face for unpolarised light at incidence cos_i
float fresnel(float cos_i)
{
    float sin_t2 = (1.0 - cos_i * cos_i) / (IOR * IOR);
    float cos_t = sqrt(max(1.0 - sin_t2, 0.0));
    float rs = (cos_i - IOR * cos_t) / (cos_i + IOR * cos_t);
    float rp = (IOR * cos_i - cos_t) / (IOR * cos_i + cos_t);
    return 0.5 * (rs * rs + rp * rp);
}

void main()
{
    vec3 V = normalize(-fs_in.view_vector);
    vec3 N = normalize(fs_in.vertex_normal);
    N = faceforward(N, -V, N);                              // the face toward the eye
    float cos_i = clamp(dot(N, V), 1e-4, 1.0);

    // both faces, their reflections between them summed: R = 2 r / (1 + r)
    float r = fresnel(cos_i);
    float R = 2.0 * r / (1.0 + r);

    // the sun's image: the reflected line of sight within the sun's disc (its edge smoothed over a pixel)
    vec3 L = fg_SunDirection[FG_VIEW_ID];
    vec3 glint = vec3(0.0);
    float off = length(reflect(-V, N) - L);                 // ~ the angle from the sun's centre, rad
    float disc = clamp((sqrt(SUN_SOLID_ANGLE / 3.14159265) - off) / max(fwidth(off), 1e-6) + 0.5, 0.0, 1.0);
    if (disc > 0.0 && dot(N, L) > 0.0) {
        vec3 ws_P = (fg_ViewMatrixInverse[FG_VIEW_ID] * vec4(fs_in.view_vector, 1.0)).xyz;
        glint = min(apply_exposure(get_sun_radiance(ws_P) * 500.0), vec3(1e4)) * R * disc
            * get_shadowing(fs_in.view_vector, N, L);
    }

    forward_pack(vec4(glint, R), fs_in.flogz);
    gl_FragDepth = logdepth_encode(fs_in.flogz);
}
