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
 * Symbolic function: IK_residual_func_cost3_wrt_fv1_Nl15
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost3WrtFv1Nl15(
    const Scalar fh1, const Scalar fv1, const Scalar rx, const Scalar ry, const Scalar rz,
    const Scalar p_init0, const Scalar p_init1, const Scalar p_init2, const Scalar rot_init_x,
    const Scalar rot_init_y, const Scalar rot_init_z, const Scalar rot_init_w,
    const Scalar epsilon) {
  // Total ops: 306

  // Unused inputs
  (void)p_init2;
  (void)epsilon;

  // Input arrays

  // Intermediate terms (111)
  const Scalar _tmp0 = std::sqrt(
      Scalar(std::pow(rx, Scalar(2)) + std::pow(ry, Scalar(2)) + std::pow(rz, Scalar(2))));
  const Scalar _tmp1 = (Scalar(1) / Scalar(2)) * _tmp0;
  const Scalar _tmp2 = std::cos(_tmp1);
  const Scalar _tmp3 = std::sin(_tmp1) / _tmp0;
  const Scalar _tmp4 = _tmp3 * rot_init_w;
  const Scalar _tmp5 = _tmp3 * rx;
  const Scalar _tmp6 = _tmp3 * rz;
  const Scalar _tmp7 = _tmp2 * rot_init_y + _tmp4 * ry + _tmp5 * rot_init_z - _tmp6 * rot_init_x;
  const Scalar _tmp8 = _tmp3 * ry;
  const Scalar _tmp9 = _tmp2 * rot_init_x + _tmp4 * rx + _tmp6 * rot_init_y - _tmp8 * rot_init_z;
  const Scalar _tmp10 = 2 * _tmp9;
  const Scalar _tmp11 = _tmp10 * _tmp7;
  const Scalar _tmp12 = _tmp2 * rot_init_z + _tmp4 * rz - _tmp5 * rot_init_y + _tmp8 * rot_init_x;
  const Scalar _tmp13 = 2 * _tmp2 * rot_init_w - 2 * _tmp5 * rot_init_x - 2 * _tmp6 * rot_init_z -
                        2 * _tmp8 * rot_init_y;
  const Scalar _tmp14 = _tmp12 * _tmp13;
  const Scalar _tmp15 = Scalar(0.20999999999999999) * _tmp11 + Scalar(0.20999999999999999) * _tmp14;
  const Scalar _tmp16 = -2 * std::pow(_tmp9, Scalar(2));
  const Scalar _tmp17 = -2 * std::pow(_tmp12, Scalar(2));
  const Scalar _tmp18 = Scalar(0.20999999999999999) * _tmp16 +
                        Scalar(0.20999999999999999) * _tmp17 + Scalar(0.20999999999999999);
  const Scalar _tmp19 = 2 * _tmp12 * _tmp7;
  const Scalar _tmp20 = _tmp13 * _tmp9;
  const Scalar _tmp21 =
      -Scalar(0.010999999999999999) * _tmp19 + Scalar(0.010999999999999999) * _tmp20;
  const Scalar _tmp22 = -_tmp18 + _tmp21;
  const Scalar _tmp23 = _tmp15 + _tmp22;
  const Scalar _tmp24 = Scalar(1.0) * _tmp23;
  const Scalar _tmp25 = -_tmp15;
  const Scalar _tmp26 = _tmp18 + _tmp21;
  const Scalar _tmp27 = _tmp25 + _tmp26;
  const Scalar _tmp28 = Scalar(0.20999999999999999) * _tmp11 - Scalar(0.20999999999999999) * _tmp14;
  const Scalar _tmp29 = 1 - 2 * std::pow(_tmp7, Scalar(2));
  const Scalar _tmp30 = Scalar(0.20999999999999999) * _tmp17 + Scalar(0.20999999999999999) * _tmp29;
  const Scalar _tmp31 = _tmp10 * _tmp12;
  const Scalar _tmp32 = _tmp13 * _tmp7;
  const Scalar _tmp33 =
      -Scalar(0.010999999999999999) * _tmp31 - Scalar(0.010999999999999999) * _tmp32;
  const Scalar _tmp34 = -_tmp30 + _tmp33;
  const Scalar _tmp35 = _tmp28 + _tmp34;
  const Scalar _tmp36 = -_tmp28;
  const Scalar _tmp37 = _tmp30 + _tmp33;
  const Scalar _tmp38 = _tmp36 + _tmp37;
  const Scalar _tmp39 = Scalar(1.0) * _tmp38;
  const Scalar _tmp40 = (-_tmp35 + _tmp39) / (-_tmp24 + _tmp27);
  const Scalar _tmp41 = _tmp38 + p_init0 + Scalar(-2.5202214700000001);
  const Scalar _tmp42 = Scalar(1.0) / (_tmp41);
  const Scalar _tmp43 = _tmp23 + p_init1 + Scalar(8.3888750099999996);
  const Scalar _tmp44 = _tmp42 * _tmp43;
  const Scalar _tmp45 = _tmp27 + p_init1 + Scalar(-4.8333311099999996);
  const Scalar _tmp46 = _tmp35 + p_init0 + Scalar(1.79662371);
  const Scalar _tmp47 = std::pow(Scalar(std::pow(_tmp45, Scalar(2)) + std::pow(_tmp46, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp48 = _tmp46 * _tmp47;
  const Scalar _tmp49 = _tmp45 * _tmp47;
  const Scalar _tmp50 = Scalar(1.0) / (_tmp44 * _tmp48 - _tmp49);
  const Scalar _tmp51 = Scalar(0.20999999999999999) * _tmp31 - Scalar(0.20999999999999999) * _tmp32;
  const Scalar _tmp52 =
      -Scalar(0.010999999999999999) * _tmp16 - Scalar(0.010999999999999999) * _tmp29;
  const Scalar _tmp53 = Scalar(0.20999999999999999) * _tmp19 + Scalar(0.20999999999999999) * _tmp20;
  const Scalar _tmp54 = _tmp52 - _tmp53;
  const Scalar _tmp55 = _tmp51 + _tmp54;
  const Scalar _tmp56 = _tmp48 * _tmp55;
  const Scalar _tmp57 = -_tmp51;
  const Scalar _tmp58 = _tmp52 + _tmp53 + _tmp57;
  const Scalar _tmp59 = _tmp50 * (-_tmp44 * _tmp56 + _tmp49 * _tmp58);
  const Scalar _tmp60 = _tmp50 * (-_tmp48 * _tmp58 + _tmp56);
  const Scalar _tmp61 = Scalar(1.0) * _tmp40 * _tmp59 - Scalar(1.0) * _tmp60;
  const Scalar _tmp62 = _tmp22 + _tmp25;
  const Scalar _tmp63 = _tmp62 + p_init1 + Scalar(8.3196563700000006);
  const Scalar _tmp64 = _tmp34 + _tmp36;
  const Scalar _tmp65 = _tmp64 + p_init0 + Scalar(1.9874742000000001);
  const Scalar _tmp66 = std::pow(Scalar(std::pow(_tmp63, Scalar(2)) + std::pow(_tmp65, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp67 = _tmp65 * _tmp66;
  const Scalar _tmp68 = _tmp63 * _tmp66;
  const Scalar _tmp69 = _tmp44 * _tmp67 - _tmp68;
  const Scalar _tmp70 = _tmp54 + _tmp57;
  const Scalar _tmp71 = _tmp55 * _tmp67;
  const Scalar _tmp72 = -_tmp40 * (-_tmp44 * _tmp71 - _tmp59 * _tmp69 + _tmp68 * _tmp70) -
                        _tmp60 * _tmp69 - _tmp67 * _tmp70 + _tmp71;
  const Scalar _tmp73 = Scalar(1.0) / (_tmp72);
  const Scalar _tmp74 =
      std::sqrt(Scalar(std::pow(_tmp41, Scalar(2)) + std::pow(_tmp43, Scalar(2))));
  const Scalar _tmp75 = Scalar(1.0) / (_tmp74);
  const Scalar _tmp76 = _tmp42 * _tmp74;
  const Scalar _tmp77 = _tmp76 * (-_tmp23 * _tmp41 * _tmp75 + _tmp38 * _tmp43 * _tmp75);
  const Scalar _tmp78 = _tmp50 * (_tmp27 * _tmp48 - _tmp35 * _tmp49 + _tmp48 * _tmp77);
  const Scalar _tmp79 = _tmp62 * _tmp67 - _tmp64 * _tmp68 + _tmp67 * _tmp77 - _tmp69 * _tmp78;
  const Scalar _tmp80 = _tmp73 * _tmp79;
  const Scalar _tmp81 = Scalar(1.0) / (_tmp79);
  const Scalar _tmp82 = _tmp72 * _tmp81;
  const Scalar _tmp83 = _tmp61 + _tmp82 * (-_tmp61 * _tmp80 - Scalar(1.0) * _tmp78);
  const Scalar _tmp84 = _tmp67 * _tmp73;
  const Scalar _tmp85 = _tmp69 * _tmp73;
  const Scalar _tmp86 = -_tmp83 * _tmp85 + Scalar(1.0);
  const Scalar _tmp87 = _tmp48 * _tmp50;
  const Scalar _tmp88 = _tmp15 + _tmp26;
  const Scalar _tmp89 = _tmp88 + p_init1 + Scalar(-4.7752063900000001);
  const Scalar _tmp90 = _tmp28 + _tmp37;
  const Scalar _tmp91 = _tmp90 + p_init0 + Scalar(-2.71799795);
  const Scalar _tmp92 = std::pow(Scalar(std::pow(_tmp89, Scalar(2)) + std::pow(_tmp91, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp93 = _tmp89 * _tmp92;
  const Scalar _tmp94 = _tmp76 * fh1;
  const Scalar _tmp95 = Scalar(1.0) * _tmp81;
  const Scalar _tmp96 = _tmp91 * _tmp92;
  const Scalar _tmp97 = fh1 * (-_tmp88 * _tmp96 + _tmp90 * _tmp93);
  const Scalar _tmp98 = -_tmp40 * (_tmp44 * _tmp55 + _tmp44 * _tmp59) + _tmp44 * _tmp60 - _tmp55;
  const Scalar _tmp99 = _tmp82 * (_tmp44 * _tmp78 - _tmp77 - _tmp80 * _tmp98) + _tmp98;
  const Scalar _tmp100 = -_tmp44 - _tmp85 * _tmp99;
  const Scalar _tmp101 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp102 = _tmp24 * _tmp40 + _tmp39;
  const Scalar _tmp103 = 0;
  const Scalar _tmp104 = _tmp103 * _tmp85;
  const Scalar _tmp105 = _tmp103 * _tmp73;
  const Scalar _tmp106 = _tmp76 * (-_tmp104 * _tmp87 + _tmp105 * _tmp67);
  const Scalar _tmp107 = _tmp50 * fh1;
  const Scalar _tmp108 = _tmp95 * _tmp97;
  const Scalar _tmp109 = _tmp104 * _tmp50;
  const Scalar _tmp110 = _tmp73 * fh1;

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) = 0;
  _res(1, 0) =
      -_tmp106 *
      std::exp(_tmp101 * _tmp106 + _tmp76 * _tmp97 * (_tmp67 * _tmp95 - _tmp69 * _tmp87 * _tmp95) +
               _tmp93 * _tmp94 * (_tmp83 * _tmp84 + _tmp86 * _tmp87) +
               _tmp94 * _tmp96 * (_tmp100 * _tmp87 + _tmp84 * _tmp99 + Scalar(1.0)));
  _res(2, 0) = -_tmp109 * std::exp(-_tmp100 * _tmp107 * _tmp96 + _tmp101 * _tmp109 -
                                   _tmp107 * _tmp86 * _tmp93 + _tmp108 * _tmp50 * _tmp69);
  _res(3, 0) = _tmp105 * std::exp(-_tmp101 * _tmp105 - _tmp108 - _tmp110 * _tmp83 * _tmp93 -
                                  _tmp110 * _tmp96 * _tmp99);

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
