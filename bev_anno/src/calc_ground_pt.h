#ifndef CALC_GROUND_PT_H
#define CALC_GROUND_PT_H

namespace algo {
  struct Groundmodel {// quadratic grid model, 2do --> oo properly!!
    float maxdst;
    float m_a1;
    float m_a2;
    float m_b1;
    float m_b2;
    float m_d;
  };
  Groundmodel gmodel;

  float PlaneModel(float x, float y)
  {
    float result = -1 * (gmodel.m_a1 * x + gmodel.m_b1 * y + gmodel.m_d); // ax2 + bx + cy2 + dy + e
    return result;
  }

  float QuadraticPlaneModel(float x, float y)
  {
    float result = -1 * (gmodel.maxdst * gmodel.m_a1 * x + gmodel.m_a2 * (x * x) +
                         gmodel.m_b1 * y + gmodel.m_b2 * (y * y) + gmodel.m_d); // ax2 + bx + cy2 + dy + e
    return result;
  }

} // namespace algo

#endif // !CALC_GROUND_PT_H

