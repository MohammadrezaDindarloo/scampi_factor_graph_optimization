// -----------------------------------------------------------------------------
// This file was autogenerated by symforce from template:
//     function/FUNCTION.h.jinja
// Do NOT modify by hand.
// -----------------------------------------------------------------------------

#pragma once

#include <Eigen/Dense>

namespace sym {

/**
 * This function was autogenerated from a symbolic function. Do not modify by hand.
 *
 * Symbolic function: resedual_func_cost3_wrt_fv1_l1
 *
 * Args:
 *     fh1: Scalar
 *     fv1: Scalar
 *     rx: Scalar
 *     ry: Scalar
 *     rz: Scalar
 *     p_init0: Scalar
 *     p_init1: Scalar
 *     p_init2: Scalar
 *     rot_init_x: Scalar
 *     rot_init_y: Scalar
 *     rot_init_z: Scalar
 *     rot_init_w: Scalar
 *     epsilon: Scalar
 *
 * Outputs:
 *     res: Matrix41
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 4, 1> ResedualFuncCost3WrtFv1L1(
    const Scalar fh1, const Scalar fv1, const Scalar rx, const Scalar ry, const Scalar rz,
    const Scalar p_init0, const Scalar p_init1, const Scalar p_init2, const Scalar rot_init_x,
    const Scalar rot_init_y, const Scalar rot_init_z, const Scalar rot_init_w,
    const Scalar epsilon) {
  // Total ops: 301

  // Unused inputs
  (void)p_init2;
  (void)epsilon;

  // Input arrays

  // Intermediate terms (110)
  const Scalar _tmp0 = std::sqrt(
      Scalar(std::pow(rx, Scalar(2)) + std::pow(ry, Scalar(2)) + std::pow(rz, Scalar(2))));
  const Scalar _tmp1 = (Scalar(1) / Scalar(2)) * _tmp0;
  const Scalar _tmp2 = std::cos(_tmp1);
  const Scalar _tmp3 = std::sin(_tmp1) / _tmp0;
  const Scalar _tmp4 = _tmp3 * ry;
  const Scalar _tmp5 = _tmp3 * rx;
  const Scalar _tmp6 = _tmp3 * rz;
  const Scalar _tmp7 =
      _tmp2 * rot_init_y + _tmp4 * rot_init_w + _tmp5 * rot_init_z - _tmp6 * rot_init_x;
  const Scalar _tmp8 =
      _tmp2 * rot_init_x - _tmp4 * rot_init_z + _tmp5 * rot_init_w + _tmp6 * rot_init_y;
  const Scalar _tmp9 = 2 * _tmp7 * _tmp8;
  const Scalar _tmp10 =
      _tmp2 * rot_init_z + _tmp4 * rot_init_x - _tmp5 * rot_init_y + _tmp6 * rot_init_w;
  const Scalar _tmp11 = 2 * _tmp2 * rot_init_w - 2 * _tmp4 * rot_init_y - 2 * _tmp5 * rot_init_x -
                        2 * _tmp6 * rot_init_z;
  const Scalar _tmp12 = _tmp10 * _tmp11;
  const Scalar _tmp13 = Scalar(0.20999999999999999) * _tmp12 + Scalar(0.20999999999999999) * _tmp9;
  const Scalar _tmp14 = -_tmp13;
  const Scalar _tmp15 = 2 * _tmp10;
  const Scalar _tmp16 = _tmp15 * _tmp7;
  const Scalar _tmp17 = _tmp11 * _tmp8;
  const Scalar _tmp18 =
      -Scalar(0.010999999999999999) * _tmp16 + Scalar(0.010999999999999999) * _tmp17;
  const Scalar _tmp19 = -2 * std::pow(_tmp8, Scalar(2));
  const Scalar _tmp20 = -2 * std::pow(_tmp10, Scalar(2));
  const Scalar _tmp21 = Scalar(0.20999999999999999) * _tmp19 +
                        Scalar(0.20999999999999999) * _tmp20 + Scalar(0.20999999999999999);
  const Scalar _tmp22 = _tmp18 + _tmp21;
  const Scalar _tmp23 = _tmp14 + _tmp22;
  const Scalar _tmp24 = _tmp23 + p_init1 + Scalar(-4.8333311099999996);
  const Scalar _tmp25 = 1 - 2 * std::pow(_tmp7, Scalar(2));
  const Scalar _tmp26 = Scalar(0.20999999999999999) * _tmp20 + Scalar(0.20999999999999999) * _tmp25;
  const Scalar _tmp27 = -_tmp26;
  const Scalar _tmp28 = _tmp15 * _tmp8;
  const Scalar _tmp29 = _tmp11 * _tmp7;
  const Scalar _tmp30 =
      -Scalar(0.010999999999999999) * _tmp28 - Scalar(0.010999999999999999) * _tmp29;
  const Scalar _tmp31 = -Scalar(0.20999999999999999) * _tmp12 + Scalar(0.20999999999999999) * _tmp9;
  const Scalar _tmp32 = _tmp30 + _tmp31;
  const Scalar _tmp33 = _tmp27 + _tmp32;
  const Scalar _tmp34 = _tmp33 + p_init0 + Scalar(1.79662371);
  const Scalar _tmp35 = Scalar(1.0) / (_tmp34);
  const Scalar _tmp36 = _tmp24 * _tmp35;
  const Scalar _tmp37 = _tmp13 + _tmp22;
  const Scalar _tmp38 = _tmp37 + p_init1 + Scalar(-4.7752063900000001);
  const Scalar _tmp39 = _tmp26 + _tmp32;
  const Scalar _tmp40 = _tmp39 + p_init0 + Scalar(-2.71799795);
  const Scalar _tmp41 = std::pow(Scalar(std::pow(_tmp38, Scalar(2)) + std::pow(_tmp40, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp42 = _tmp40 * _tmp41;
  const Scalar _tmp43 = _tmp38 * _tmp41;
  const Scalar _tmp44 = Scalar(1.0) / (_tmp36 * _tmp42 - _tmp43);
  const Scalar _tmp45 = _tmp30 - _tmp31;
  const Scalar _tmp46 = _tmp26 + _tmp45;
  const Scalar _tmp47 = _tmp46 + p_init0 + Scalar(-2.5202214700000001);
  const Scalar _tmp48 = _tmp18 - _tmp21;
  const Scalar _tmp49 = _tmp13 + _tmp48;
  const Scalar _tmp50 = _tmp49 + p_init1 + Scalar(8.3888750099999996);
  const Scalar _tmp51 = std::pow(Scalar(std::pow(_tmp47, Scalar(2)) + std::pow(_tmp50, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp52 = _tmp50 * _tmp51;
  const Scalar _tmp53 = _tmp47 * _tmp51;
  const Scalar _tmp54 = _tmp36 * _tmp53 - _tmp52;
  const Scalar _tmp55 = _tmp44 * _tmp54;
  const Scalar _tmp56 =
      std::sqrt(Scalar(std::pow(_tmp24, Scalar(2)) + std::pow(_tmp34, Scalar(2))));
  const Scalar _tmp57 = Scalar(1.0) / (_tmp56);
  const Scalar _tmp58 = _tmp35 * _tmp56;
  const Scalar _tmp59 = _tmp58 * (-_tmp23 * _tmp34 * _tmp57 + _tmp24 * _tmp33 * _tmp57);
  const Scalar _tmp60 = _tmp37 * _tmp42 - _tmp39 * _tmp43 + _tmp42 * _tmp59;
  const Scalar _tmp61 = -_tmp46 * _tmp52 + _tmp49 * _tmp53 + _tmp53 * _tmp59 - _tmp55 * _tmp60;
  const Scalar _tmp62 = Scalar(1.0) / (_tmp61);
  const Scalar _tmp63 = Scalar(1.0) * _tmp62;
  const Scalar _tmp64 = _tmp14 + _tmp48;
  const Scalar _tmp65 = _tmp64 + p_init1 + Scalar(8.3196563700000006);
  const Scalar _tmp66 = _tmp27 + _tmp45;
  const Scalar _tmp67 = _tmp66 + p_init0 + Scalar(1.9874742000000001);
  const Scalar _tmp68 = std::pow(Scalar(std::pow(_tmp65, Scalar(2)) + std::pow(_tmp67, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp69 = _tmp67 * _tmp68;
  const Scalar _tmp70 = _tmp65 * _tmp68;
  const Scalar _tmp71 = fh1 * (-_tmp64 * _tmp69 + _tmp66 * _tmp70);
  const Scalar _tmp72 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp73 = Scalar(0.20999999999999999) * _tmp16 + Scalar(0.20999999999999999) * _tmp17;
  const Scalar _tmp74 =
      -Scalar(0.010999999999999999) * _tmp19 - Scalar(0.010999999999999999) * _tmp25;
  const Scalar _tmp75 = Scalar(0.20999999999999999) * _tmp28 - Scalar(0.20999999999999999) * _tmp29;
  const Scalar _tmp76 = _tmp74 + _tmp75;
  const Scalar _tmp77 = -_tmp73 + _tmp76;
  const Scalar _tmp78 = _tmp73 + _tmp74 - _tmp75;
  const Scalar _tmp79 = _tmp36 * _tmp78;
  const Scalar _tmp80 = _tmp73 + _tmp76;
  const Scalar _tmp81 = -_tmp42 * _tmp79 + _tmp43 * _tmp80;
  const Scalar _tmp82 = Scalar(1.0) * _tmp33;
  const Scalar _tmp83 = Scalar(1.0) * _tmp23;
  const Scalar _tmp84 = (-_tmp39 + _tmp82) / (_tmp37 - _tmp83);
  const Scalar _tmp85 = _tmp42 * _tmp78 - _tmp42 * _tmp80;
  const Scalar _tmp86 = -_tmp53 * _tmp77 + _tmp53 * _tmp78 - _tmp55 * _tmp85 -
                        _tmp84 * (_tmp52 * _tmp77 - _tmp53 * _tmp79 - _tmp55 * _tmp81);
  const Scalar _tmp87 = Scalar(1.0) / (_tmp86);
  const Scalar _tmp88 = _tmp82 + _tmp83 * _tmp84;
  const Scalar _tmp89 = 0;
  const Scalar _tmp90 = _tmp55 * _tmp89;
  const Scalar _tmp91 = _tmp58 * (-_tmp42 * _tmp90 + _tmp53 * _tmp89);
  const Scalar _tmp92 = Scalar(1.0) * _tmp44;
  const Scalar _tmp93 = _tmp81 * _tmp84 * _tmp92 - _tmp85 * _tmp92;
  const Scalar _tmp94 = _tmp61 * _tmp87;
  const Scalar _tmp95 = _tmp62 * _tmp86;
  const Scalar _tmp96 = _tmp93 + _tmp95 * (-_tmp60 * _tmp92 - _tmp93 * _tmp94);
  const Scalar _tmp97 = _tmp54 * _tmp87;
  const Scalar _tmp98 = -_tmp96 * _tmp97 + Scalar(1.0);
  const Scalar _tmp99 = _tmp42 * _tmp44;
  const Scalar _tmp100 = _tmp53 * _tmp87;
  const Scalar _tmp101 = _tmp58 * fh1;
  const Scalar _tmp102 = _tmp36 * _tmp44;
  const Scalar _tmp103 = _tmp102 * _tmp85 - _tmp78 - _tmp84 * (_tmp102 * _tmp81 + _tmp79);
  const Scalar _tmp104 = _tmp103 + _tmp95 * (_tmp102 * _tmp60 - _tmp103 * _tmp94 - _tmp59);
  const Scalar _tmp105 = -_tmp104 * _tmp97 - _tmp36;
  const Scalar _tmp106 = _tmp72 * _tmp89;
  const Scalar _tmp107 = _tmp44 * fh1;
  const Scalar _tmp108 = _tmp63 * _tmp71;
  const Scalar _tmp109 = _tmp87 * fh1;

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) = 0;
  _res(1, 0) =
      -_tmp91 *
      std::exp(_tmp101 * _tmp69 * (_tmp100 * _tmp104 + _tmp105 * _tmp99 + Scalar(1.0)) +
               _tmp101 * _tmp70 * (_tmp100 * _tmp96 + _tmp98 * _tmp99) +
               _tmp58 * _tmp71 * (-_tmp42 * _tmp55 * _tmp63 + _tmp53 * _tmp63) + _tmp72 * _tmp91);
  _res(2, 0) = -_tmp90 * std::exp(-_tmp105 * _tmp107 * _tmp69 + _tmp106 * _tmp55 -
                                  _tmp107 * _tmp70 * _tmp98 + _tmp108 * _tmp55);
  _res(3, 0) = _tmp89 * std::exp(-_tmp104 * _tmp109 * _tmp69 - _tmp106 - _tmp108 -
                                 _tmp109 * _tmp70 * _tmp96);

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
