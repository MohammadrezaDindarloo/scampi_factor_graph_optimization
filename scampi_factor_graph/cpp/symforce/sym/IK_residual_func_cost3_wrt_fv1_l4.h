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
 * Symbolic function: IK_residual_func_cost3_wrt_fv1_l4
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost3WrtFv1L4(
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
  const Scalar _tmp0 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp1 = std::sqrt(
      Scalar(std::pow(rx, Scalar(2)) + std::pow(ry, Scalar(2)) + std::pow(rz, Scalar(2))));
  const Scalar _tmp2 = (Scalar(1) / Scalar(2)) * _tmp1;
  const Scalar _tmp3 = std::cos(_tmp2);
  const Scalar _tmp4 = std::sin(_tmp2) / _tmp1;
  const Scalar _tmp5 = _tmp4 * ry;
  const Scalar _tmp6 = _tmp4 * rx;
  const Scalar _tmp7 = _tmp4 * rz;
  const Scalar _tmp8 =
      _tmp3 * rot_init_x - _tmp5 * rot_init_z + _tmp6 * rot_init_w + _tmp7 * rot_init_y;
  const Scalar _tmp9 =
      _tmp3 * rot_init_z + _tmp5 * rot_init_x - _tmp6 * rot_init_y + _tmp7 * rot_init_w;
  const Scalar _tmp10 = 2 * _tmp9;
  const Scalar _tmp11 = _tmp10 * _tmp8;
  const Scalar _tmp12 =
      _tmp3 * rot_init_y + _tmp5 * rot_init_w + _tmp6 * rot_init_z - _tmp7 * rot_init_x;
  const Scalar _tmp13 = 2 * _tmp3 * rot_init_w - 2 * _tmp5 * rot_init_y - 2 * _tmp6 * rot_init_x -
                        2 * _tmp7 * rot_init_z;
  const Scalar _tmp14 = _tmp12 * _tmp13;
  const Scalar _tmp15 = Scalar(0.20999999999999999) * _tmp11 - Scalar(0.20999999999999999) * _tmp14;
  const Scalar _tmp16 = -2 * std::pow(_tmp8, Scalar(2));
  const Scalar _tmp17 = 1 - 2 * std::pow(_tmp12, Scalar(2));
  const Scalar _tmp18 =
      -Scalar(0.010999999999999999) * _tmp16 - Scalar(0.010999999999999999) * _tmp17;
  const Scalar _tmp19 = _tmp10 * _tmp12;
  const Scalar _tmp20 = _tmp13 * _tmp8;
  const Scalar _tmp21 = Scalar(0.20999999999999999) * _tmp19 + Scalar(0.20999999999999999) * _tmp20;
  const Scalar _tmp22 = _tmp18 - _tmp21;
  const Scalar _tmp23 = _tmp15 + _tmp22;
  const Scalar _tmp24 = 2 * _tmp12 * _tmp8;
  const Scalar _tmp25 = _tmp13 * _tmp9;
  const Scalar _tmp26 = Scalar(0.20999999999999999) * _tmp24 - Scalar(0.20999999999999999) * _tmp25;
  const Scalar _tmp27 = -_tmp26;
  const Scalar _tmp28 =
      -Scalar(0.010999999999999999) * _tmp11 - Scalar(0.010999999999999999) * _tmp14;
  const Scalar _tmp29 = -2 * std::pow(_tmp9, Scalar(2));
  const Scalar _tmp30 = Scalar(0.20999999999999999) * _tmp17 + Scalar(0.20999999999999999) * _tmp29;
  const Scalar _tmp31 = _tmp28 + _tmp30;
  const Scalar _tmp32 = _tmp27 + _tmp31;
  const Scalar _tmp33 = _tmp32 + p_init0 + Scalar(-2.5202214700000001);
  const Scalar _tmp34 = Scalar(0.20999999999999999) * _tmp24 + Scalar(0.20999999999999999) * _tmp25;
  const Scalar _tmp35 = Scalar(0.20999999999999999) * _tmp16 +
                        Scalar(0.20999999999999999) * _tmp29 + Scalar(0.20999999999999999);
  const Scalar _tmp36 =
      -Scalar(0.010999999999999999) * _tmp19 + Scalar(0.010999999999999999) * _tmp20;
  const Scalar _tmp37 = -_tmp35 + _tmp36;
  const Scalar _tmp38 = _tmp34 + _tmp37;
  const Scalar _tmp39 = _tmp38 + p_init1 + Scalar(8.3888750099999996);
  const Scalar _tmp40 = std::pow(Scalar(std::pow(_tmp33, Scalar(2)) + std::pow(_tmp39, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp41 = _tmp33 * _tmp40;
  const Scalar _tmp42 = _tmp15 + _tmp18 + _tmp21;
  const Scalar _tmp43 = _tmp41 * _tmp42;
  const Scalar _tmp44 = -_tmp23 * _tmp41 + _tmp43;
  const Scalar _tmp45 = _tmp26 + _tmp31;
  const Scalar _tmp46 = _tmp45 + p_init0 + Scalar(-2.71799795);
  const Scalar _tmp47 = Scalar(1.0) / (_tmp46);
  const Scalar _tmp48 = _tmp35 + _tmp36;
  const Scalar _tmp49 = _tmp34 + _tmp48;
  const Scalar _tmp50 = _tmp49 + p_init1 + Scalar(-4.7752063900000001);
  const Scalar _tmp51 = _tmp47 * _tmp50;
  const Scalar _tmp52 = _tmp39 * _tmp40;
  const Scalar _tmp53 = Scalar(1.0) / (_tmp41 * _tmp51 - _tmp52);
  const Scalar _tmp54 = -_tmp34;
  const Scalar _tmp55 = _tmp37 + _tmp54;
  const Scalar _tmp56 = _tmp55 + p_init1 + Scalar(8.3196563700000006);
  const Scalar _tmp57 = _tmp28 - _tmp30;
  const Scalar _tmp58 = _tmp27 + _tmp57;
  const Scalar _tmp59 = _tmp58 + p_init0 + Scalar(1.9874742000000001);
  const Scalar _tmp60 = std::pow(Scalar(std::pow(_tmp56, Scalar(2)) + std::pow(_tmp59, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp61 = _tmp56 * _tmp60;
  const Scalar _tmp62 = _tmp59 * _tmp60;
  const Scalar _tmp63 = _tmp51 * _tmp62 - _tmp61;
  const Scalar _tmp64 = _tmp53 * _tmp63;
  const Scalar _tmp65 = _tmp42 * _tmp62;
  const Scalar _tmp66 = _tmp23 * _tmp52 - _tmp43 * _tmp51;
  const Scalar _tmp67 = -_tmp15 + _tmp22;
  const Scalar _tmp68 = Scalar(1.0) * _tmp49;
  const Scalar _tmp69 = Scalar(1.0) * _tmp45;
  const Scalar _tmp70 = (-_tmp32 + _tmp69) / (_tmp38 - _tmp68);
  const Scalar _tmp71 = -_tmp44 * _tmp64 - _tmp62 * _tmp67 + _tmp65 -
                        _tmp70 * (-_tmp51 * _tmp65 + _tmp61 * _tmp67 - _tmp64 * _tmp66);
  const Scalar _tmp72 = Scalar(1.0) / (_tmp71);
  const Scalar _tmp73 = _tmp68 * _tmp70 + _tmp69;
  const Scalar _tmp74 = 0;
  const Scalar _tmp75 = _tmp64 * _tmp74;
  const Scalar _tmp76 =
      std::sqrt(Scalar(std::pow(_tmp46, Scalar(2)) + std::pow(_tmp50, Scalar(2))));
  const Scalar _tmp77 = _tmp47 * _tmp76;
  const Scalar _tmp78 = _tmp77 * (-_tmp41 * _tmp75 + _tmp62 * _tmp74);
  const Scalar _tmp79 = Scalar(1.0) / (_tmp76);
  const Scalar _tmp80 = _tmp77 * (_tmp45 * _tmp50 * _tmp79 - _tmp46 * _tmp49 * _tmp79);
  const Scalar _tmp81 = -_tmp32 * _tmp52 + _tmp38 * _tmp41 + _tmp41 * _tmp80;
  const Scalar _tmp82 = _tmp55 * _tmp62 - _tmp58 * _tmp61 + _tmp62 * _tmp80 - _tmp64 * _tmp81;
  const Scalar _tmp83 = Scalar(1.0) / (_tmp82);
  const Scalar _tmp84 = Scalar(1.0) * _tmp53;
  const Scalar _tmp85 = _tmp63 * _tmp83 * _tmp84;
  const Scalar _tmp86 = Scalar(1.0) * _tmp83;
  const Scalar _tmp87 = _tmp26 + _tmp57;
  const Scalar _tmp88 = _tmp48 + _tmp54;
  const Scalar _tmp89 = _tmp88 + p_init1 + Scalar(-4.8333311099999996);
  const Scalar _tmp90 = _tmp87 + p_init0 + Scalar(1.79662371);
  const Scalar _tmp91 = std::pow(Scalar(std::pow(_tmp89, Scalar(2)) + std::pow(_tmp90, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp92 = _tmp89 * _tmp91;
  const Scalar _tmp93 = _tmp90 * _tmp91;
  const Scalar _tmp94 = fh1 * (_tmp87 * _tmp92 - _tmp88 * _tmp93);
  const Scalar _tmp95 = _tmp51 * _tmp53;
  const Scalar _tmp96 = -_tmp42 + _tmp44 * _tmp95 - _tmp70 * (_tmp42 * _tmp51 + _tmp66 * _tmp95);
  const Scalar _tmp97 = _tmp72 * _tmp82;
  const Scalar _tmp98 = _tmp71 * _tmp83;
  const Scalar _tmp99 = _tmp96 + _tmp98 * (-_tmp80 + _tmp81 * _tmp95 - _tmp96 * _tmp97);
  const Scalar _tmp100 = _tmp62 * _tmp72;
  const Scalar _tmp101 = _tmp63 * _tmp72;
  const Scalar _tmp102 = -_tmp101 * _tmp99 - _tmp51;
  const Scalar _tmp103 = _tmp41 * _tmp53;
  const Scalar _tmp104 = _tmp93 * fh1;
  const Scalar _tmp105 = -_tmp44 * _tmp84 + _tmp66 * _tmp70 * _tmp84;
  const Scalar _tmp106 = _tmp105 + _tmp98 * (-_tmp105 * _tmp97 - _tmp81 * _tmp84);
  const Scalar _tmp107 = -_tmp101 * _tmp106 + Scalar(1.0);
  const Scalar _tmp108 = _tmp92 * fh1;
  const Scalar _tmp109 = _tmp0 * _tmp74;

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) = 0;
  _res(1, 0) =
      -_tmp78 * std::exp(_tmp0 * _tmp78 +
                         _tmp104 * _tmp77 * (_tmp100 * _tmp99 + _tmp102 * _tmp103 + Scalar(1.0)) +
                         _tmp108 * _tmp77 * (_tmp100 * _tmp106 + _tmp103 * _tmp107) +
                         _tmp77 * _tmp94 * (-_tmp41 * _tmp85 + _tmp62 * _tmp86));
  _res(2, 0) = -_tmp75 * std::exp(-_tmp102 * _tmp104 * _tmp53 - _tmp107 * _tmp108 * _tmp53 +
                                  _tmp109 * _tmp64 + _tmp85 * _tmp94);
  _res(3, 0) = _tmp74 * std::exp(-_tmp104 * _tmp72 * _tmp99 - _tmp106 * _tmp108 * _tmp72 - _tmp109 -
                                 _tmp86 * _tmp94);

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
