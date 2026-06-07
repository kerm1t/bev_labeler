#ifndef MATHBASE_H
#define MATHBASE_H

static float lerp(const float a, const float b, const float t) {
  return a + t * (b - a);
}

// hmm, cannot return vec3, as it is an array
static void lerp2(vec3& r, const vec3 a, const vec3 b, const float t) {
//  return a + t * (b - a);
  vec3 c;
  vec3 d;
  vec3_sub(c, b, a);
  vec3_scale(d, c, t);
  vec3_add(r, a, d);
}
#endif // MATHBASE_H