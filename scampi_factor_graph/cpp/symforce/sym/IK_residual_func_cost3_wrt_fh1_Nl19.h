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
 * Symbolic function: IK_residual_func_cost3_wrt_fh1_Nl19
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost3WrtFh1Nl19(
    const Scalar fh1, const Scalar fv1, const Scalar rx, const Scalar ry, const Scalar rz,
    const Scalar p_init0, const Scalar p_init1, const Scalar p_init2, const Scalar rot_init_x,
    const Scalar rot_init_y, const Scalar rot_init_z, const Scalar rot_init_w,
    const Scalar epsilon) {
  // Total ops: 321

  // Unused inputs
  (void)p_init2;
  (void)epsilon;

  // Input arrays

  // Intermediate terms (119)
  const Scalar _tmp0 = std::sqrt(
      Scalar(std::pow(rx, Scalar(2)) + std::pow(ry, Scalar(2)) + std::pow(rz, Scalar(2))));
  const Scalar _tmp1 = (Scalar(1) / Scalar(2)) * _tmp0;
  const Scalar _tmp2 = std::cos(_tmp1);
  const Scalar _tmp3 = std::sin(_tmp1) / _tmp0;
  const Scalar _tmp4 = _tmp3 * ry;
  const Scalar _tmp5 = _tmp3 * rx;
  const Scalar _tmp6 = _tmp3 * rot_init_x;
  const Scalar _tmp7 = _tmp2 * rot_init_y + _tmp4 * rot_init_w + _tmp5 * rot_init_z - _tmp6 * rz;
  const Scalar _tmp8 = _tmp3 * rz;
  const Scalar _tmp9 =
      _tmp2 * rot_init_x - _tmp4 * rot_init_z + _tmp5 * rot_init_w + _tmp8 * rot_init_y;
  const Scalar _tmp10 = 2 * _tmp7 * _tmp9;
  const Scalar _tmp11 = _tmp2 * rot_init_z - _tmp5 * rot_init_y + _tmp6 * ry + _tmp8 * rot_init_w;
  const Scalar _tmp12 =
      2 * _tmp2 * rot_init_w - 2 * _tmp4 * rot_init_y - 2 * _tmp6 * rx - 2 * _tmp8 * rot_init_z;
  const Scalar _tmp13 = _tmp11 * _tmp12;
  const Scalar _tmp14 = Scalar(0.20999999999999999) * _tmp10 - Scalar(0.20999999999999999) * _tmp13;
  const Scalar _tmp15 = -_tmp14;
  const Scalar _tmp16 = 2 * _tmp11;
  const Scalar _tmp17 = _tmp16 * _tmp9;
  const Scalar _tmp18 = _tmp12 * _tmp7;
  const Scalar _tmp19 =
      -Scalar(0.010999999999999999) * _tmp17 - Scalar(0.010999999999999999) * _tmp18;
  const Scalar _tmp20 = -2 * std::pow(_tmp11, Scalar(2));
  const Scalar _tmp21 = 1 - 2 * std::pow(_tmp7, Scalar(2));
  const Scalar _tmp22 = Scalar(0.20999999999999999) * _tmp20 + Scalar(0.20999999999999999) * _tmp21;
  const Scalar _tmp23 = _tmp19 + _tmp22;
  const Scalar _tmp24 = _tmp15 + _tmp23;
  const Scalar _tmp25 = _tmp24 + p_init0 + Scalar(-2.5202214700000001);
  const Scalar _tmp26 = Scalar(0.20999999999999999) * _tmp10 + Scalar(0.20999999999999999) * _tmp13;
  const Scalar _tmp27 = -2 * std::pow(_tmp9, Scalar(2));
  const Scalar _tmp28 = Scalar(0.20999999999999999) * _tmp20 +
                        Scalar(0.20999999999999999) * _tmp27 + Scalar(0.20999999999999999);
  const Scalar _tmp29 = _tmp16 * _tmp7;
  const Scalar _tmp30 = _tmp12 * _tmp9;
  const Scalar _tmp31 =
      -Scalar(0.010999999999999999) * _tmp29 + Scalar(0.010999999999999999) * _tmp30;
  const Scalar _tmp32 = -_tmp28 + _tmp31;
  const Scalar _tmp33 = _tmp26 + _tmp32;
  const Scalar _tmp34 = _tmp33 + p_init1 + Scalar(8.3888750099999996);
  const Scalar _tmp35 = std::pow(Scalar(std::pow(_tmp25, Scalar(2)) + std::pow(_tmp34, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp36 = _tmp25 * _tmp35;
  const Scalar _tmp37 = Scalar(0.20999999999999999) * _tmp29 + Scalar(0.20999999999999999) * _tmp30;
  const Scalar _tmp38 = -_tmp37;
  const Scalar _tmp39 =
      -Scalar(0.010999999999999999) * _tmp21 - Scalar(0.010999999999999999) * _tmp27;
  const Scalar _tmp40 = Scalar(0.20999999999999999) * _tmp17 - Scalar(0.20999999999999999) * _tmp18;
  const Scalar _tmp41 = _tmp39 + _tmp40;
  const Scalar _tmp42 = _tmp38 + _tmp41;
  const Scalar _tmp43 = _tmp34 * _tmp35;
  const Scalar _tmp44 = -_tmp26;
  const Scalar _tmp45 = _tmp32 + _tmp44;
  const Scalar _tmp46 = _tmp45 + p_init1 + Scalar(8.3196563700000006);
  const Scalar _tmp47 = _tmp19 - _tmp22;
  const Scalar _tmp48 = _tmp15 + _tmp47;
  const Scalar _tmp49 = _tmp48 + p_init0 + Scalar(1.9874742000000001);
  const Scalar _tmp50 = Scalar(1.0) / (_tmp49);
  const Scalar _tmp51 = _tmp46 * _tmp50;
  const Scalar _tmp52 = _tmp38 + _tmp39 - _tmp40;
  const Scalar _tmp53 = _tmp28 + _tmp31;
  const Scalar _tmp54 = _tmp26 + _tmp53;
  const Scalar _tmp55 = _tmp54 + p_init1 + Scalar(-4.7752063900000001);
  const Scalar _tmp56 = _tmp14 + _tmp23;
  const Scalar _tmp57 = _tmp56 + p_init0 + Scalar(-2.71799795);
  const Scalar _tmp58 = std::pow(Scalar(std::pow(_tmp55, Scalar(2)) + std::pow(_tmp57, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp59 = _tmp57 * _tmp58;
  const Scalar _tmp60 = _tmp52 * _tmp59;
  const Scalar _tmp61 = _tmp37 + _tmp41;
  const Scalar _tmp62 = _tmp55 * _tmp58;
  const Scalar _tmp63 = -_tmp51 * _tmp60 + _tmp61 * _tmp62;
  const Scalar _tmp64 = Scalar(1.0) / (_tmp51 * _tmp59 - _tmp62);
  const Scalar _tmp65 = _tmp36 * _tmp51 - _tmp43;
  const Scalar _tmp66 = _tmp64 * _tmp65;
  const Scalar _tmp67 = _tmp36 * _tmp52;
  const Scalar _tmp68 = Scalar(1.0) * _tmp48;
  const Scalar _tmp69 = Scalar(1.0) * _tmp45;
  const Scalar _tmp70 = (-_tmp56 + _tmp68) / (_tmp54 - _tmp69);
  const Scalar _tmp71 = -_tmp59 * _tmp61 + _tmp60;
  const Scalar _tmp72 = -_tmp36 * _tmp42 - _tmp66 * _tmp71 + _tmp67 -
                        _tmp70 * (_tmp42 * _tmp43 - _tmp51 * _tmp67 - _tmp63 * _tmp66);
  const Scalar _tmp73 = Scalar(1.0) / (_tmp72);
  const Scalar _tmp74 =
      std::sqrt(Scalar(std::pow(_tmp46, Scalar(2)) + std::pow(_tmp49, Scalar(2))));
  const Scalar _tmp75 = Scalar(1.0) / (_tmp74);
  const Scalar _tmp76 = _tmp50 * _tmp74;
  const Scalar _tmp77 = _tmp76 * (-_tmp45 * _tmp49 * _tmp75 + _tmp46 * _tmp48 * _tmp75);
  const Scalar _tmp78 = _tmp51 * _tmp64;
  const Scalar _tmp79 = -_tmp52 - _tmp70 * (_tmp51 * _tmp52 + _tmp63 * _tmp78) + _tmp71 * _tmp78;
  const Scalar _tmp80 = _tmp54 * _tmp59 - _tmp56 * _tmp62 + _tmp59 * _tmp77;
  const Scalar _tmp81 = -_tmp24 * _tmp43 + _tmp33 * _tmp36 + _tmp36 * _tmp77 - _tmp66 * _tmp80;
  const Scalar _tmp82 = _tmp73 * _tmp81;
  const Scalar _tmp83 = Scalar(1.0) / (_tmp81);
  const Scalar _tmp84 = _tmp72 * _tmp83;
  const Scalar _tmp85 = _tmp79 + _tmp84 * (-_tmp77 + _tmp78 * _tmp80 - _tmp79 * _tmp82);
  const Scalar _tmp86 = _tmp73 * _tmp85;
  const Scalar _tmp87 = _tmp65 * _tmp73;
  const Scalar _tmp88 = -_tmp51 - _tmp85 * _tmp87;
  const Scalar _tmp89 = _tmp59 * _tmp64;
  const Scalar _tmp90 = _tmp44 + _tmp53;
  const Scalar _tmp91 = _tmp90 + p_init1 + Scalar(-4.8333311099999996);
  const Scalar _tmp92 = _tmp14 + _tmp47;
  const Scalar _tmp93 = _tmp92 + p_init0 + Scalar(1.79662371);
  const Scalar _tmp94 = std::pow(Scalar(std::pow(_tmp91, Scalar(2)) + std::pow(_tmp93, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp95 = _tmp93 * _tmp94;
  const Scalar _tmp96 = _tmp76 * _tmp95 * (_tmp36 * _tmp86 + _tmp88 * _tmp89 + Scalar(1.0));
  const Scalar _tmp97 = _tmp91 * _tmp94;
  const Scalar _tmp98 = -_tmp90 * _tmp95 + _tmp92 * _tmp97;
  const Scalar _tmp99 = Scalar(1.0) * _tmp83;
  const Scalar _tmp100 = Scalar(1.0) * _tmp64;
  const Scalar _tmp101 = _tmp100 * _tmp65 * _tmp83;
  const Scalar _tmp102 = _tmp76 * _tmp98 * (-_tmp101 * _tmp59 + _tmp36 * _tmp99);
  const Scalar _tmp103 = _tmp100 * _tmp63 * _tmp70 - _tmp100 * _tmp71;
  const Scalar _tmp104 = _tmp103 + _tmp84 * (-_tmp100 * _tmp80 - _tmp103 * _tmp82);
  const Scalar _tmp105 = _tmp104 * _tmp73;
  const Scalar _tmp106 = -_tmp104 * _tmp87 + Scalar(1.0);
  const Scalar _tmp107 = _tmp76 * _tmp97 * (_tmp105 * _tmp36 + _tmp106 * _tmp89);
  const Scalar _tmp108 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp109 = _tmp68 + _tmp69 * _tmp70;
  const Scalar _tmp110 = 0;
  const Scalar _tmp111 = _tmp110 * _tmp87;
  const Scalar _tmp112 = _tmp110 * _tmp73;
  const Scalar _tmp113 = _tmp64 * _tmp88 * _tmp95;
  const Scalar _tmp114 = _tmp101 * _tmp98;
  const Scalar _tmp115 = _tmp106 * _tmp64 * _tmp97;
  const Scalar _tmp116 = _tmp105 * _tmp97;
  const Scalar _tmp117 = _tmp98 * _tmp99;
  const Scalar _tmp118 = _tmp86 * _tmp95;

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) = -std::exp(-fh1);
  _res(1, 0) = -(-_tmp102 - _tmp107 - _tmp96) *
               std::exp(_tmp102 * fh1 + _tmp107 * fh1 +
                        _tmp108 * _tmp76 * (-_tmp111 * _tmp89 + _tmp112 * _tmp36) + _tmp96 * fh1);
  _res(2, 0) = -(_tmp113 - _tmp114 + _tmp115) *
               std::exp(_tmp108 * _tmp111 * _tmp64 - _tmp113 * fh1 + _tmp114 * fh1 - _tmp115 * fh1);
  _res(3, 0) = -(_tmp116 + _tmp117 + _tmp118) *
               std::exp(-_tmp108 * _tmp112 - _tmp116 * fh1 - _tmp117 * fh1 - _tmp118 * fh1);

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
