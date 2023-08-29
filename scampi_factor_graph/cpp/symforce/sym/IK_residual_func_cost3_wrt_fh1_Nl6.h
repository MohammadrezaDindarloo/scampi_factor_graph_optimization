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
 * Symbolic function: IK_residual_func_cost3_wrt_fh1_Nl6
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost3WrtFh1Nl6(
    const Scalar fh1, const Scalar fv1, const Scalar rx, const Scalar ry, const Scalar rz,
    const Scalar p_init0, const Scalar p_init1, const Scalar p_init2, const Scalar rot_init_x,
    const Scalar rot_init_y, const Scalar rot_init_z, const Scalar rot_init_w,
    const Scalar epsilon) {
  // Total ops: 315

  // Unused inputs
  (void)p_init2;
  (void)epsilon;

  // Input arrays

  // Intermediate terms (113)
  const Scalar _tmp0 = std::sqrt(
      Scalar(std::pow(rx, Scalar(2)) + std::pow(ry, Scalar(2)) + std::pow(rz, Scalar(2))));
  const Scalar _tmp1 = (Scalar(1) / Scalar(2)) * _tmp0;
  const Scalar _tmp2 = std::cos(_tmp1);
  const Scalar _tmp3 = std::sin(_tmp1) / _tmp0;
  const Scalar _tmp4 = _tmp3 * ry;
  const Scalar _tmp5 = _tmp3 * rx;
  const Scalar _tmp6 = _tmp3 * rz;
  const Scalar _tmp7 =
      _tmp2 * rot_init_x - _tmp4 * rot_init_z + _tmp5 * rot_init_w + _tmp6 * rot_init_y;
  const Scalar _tmp8 = -2 * std::pow(_tmp7, Scalar(2));
  const Scalar _tmp9 =
      _tmp2 * rot_init_z + _tmp4 * rot_init_x - _tmp5 * rot_init_y + _tmp6 * rot_init_w;
  const Scalar _tmp10 = -2 * std::pow(_tmp9, Scalar(2));
  const Scalar _tmp11 = Scalar(0.20999999999999999) * _tmp10 + Scalar(0.20999999999999999) * _tmp8 +
                        Scalar(0.20999999999999999);
  const Scalar _tmp12 =
      _tmp2 * rot_init_y + _tmp4 * rot_init_w + _tmp5 * rot_init_z - _tmp6 * rot_init_x;
  const Scalar _tmp13 = 2 * _tmp12 * _tmp9;
  const Scalar _tmp14 = 2 * _tmp2 * rot_init_w - 2 * _tmp4 * rot_init_y - 2 * _tmp5 * rot_init_x -
                        2 * _tmp6 * rot_init_z;
  const Scalar _tmp15 = _tmp14 * _tmp7;
  const Scalar _tmp16 =
      -Scalar(0.010999999999999999) * _tmp13 + Scalar(0.010999999999999999) * _tmp15;
  const Scalar _tmp17 = 2 * _tmp7;
  const Scalar _tmp18 = _tmp12 * _tmp17;
  const Scalar _tmp19 = _tmp14 * _tmp9;
  const Scalar _tmp20 = Scalar(0.20999999999999999) * _tmp18 + Scalar(0.20999999999999999) * _tmp19;
  const Scalar _tmp21 = _tmp16 - _tmp20;
  const Scalar _tmp22 = _tmp11 + _tmp21;
  const Scalar _tmp23 = _tmp22 + p_init1 + Scalar(-4.8333311099999996);
  const Scalar _tmp24 = 1 - 2 * std::pow(_tmp12, Scalar(2));
  const Scalar _tmp25 = Scalar(0.20999999999999999) * _tmp10 + Scalar(0.20999999999999999) * _tmp24;
  const Scalar _tmp26 = -_tmp25;
  const Scalar _tmp27 = _tmp17 * _tmp9;
  const Scalar _tmp28 = _tmp12 * _tmp14;
  const Scalar _tmp29 =
      -Scalar(0.010999999999999999) * _tmp27 - Scalar(0.010999999999999999) * _tmp28;
  const Scalar _tmp30 = Scalar(0.20999999999999999) * _tmp18 - Scalar(0.20999999999999999) * _tmp19;
  const Scalar _tmp31 = _tmp29 + _tmp30;
  const Scalar _tmp32 = _tmp26 + _tmp31;
  const Scalar _tmp33 = _tmp32 + p_init0 + Scalar(1.79662371);
  const Scalar _tmp34 = std::pow(Scalar(std::pow(_tmp23, Scalar(2)) + std::pow(_tmp33, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp35 = _tmp33 * _tmp34;
  const Scalar _tmp36 = -_tmp11;
  const Scalar _tmp37 = _tmp21 + _tmp36;
  const Scalar _tmp38 = _tmp37 + p_init1 + Scalar(8.3196563700000006);
  const Scalar _tmp39 = _tmp29 - _tmp30;
  const Scalar _tmp40 = _tmp26 + _tmp39;
  const Scalar _tmp41 = _tmp40 + p_init0 + Scalar(1.9874742000000001);
  const Scalar _tmp42 = Scalar(1.0) / (_tmp41);
  const Scalar _tmp43 = _tmp38 * _tmp42;
  const Scalar _tmp44 = Scalar(0.20999999999999999) * _tmp13 + Scalar(0.20999999999999999) * _tmp15;
  const Scalar _tmp45 =
      -Scalar(0.010999999999999999) * _tmp24 - Scalar(0.010999999999999999) * _tmp8;
  const Scalar _tmp46 = Scalar(0.20999999999999999) * _tmp27 - Scalar(0.20999999999999999) * _tmp28;
  const Scalar _tmp47 = _tmp45 - _tmp46;
  const Scalar _tmp48 = -_tmp44 + _tmp47;
  const Scalar _tmp49 = _tmp35 * _tmp48;
  const Scalar _tmp50 = _tmp44 + _tmp47;
  const Scalar _tmp51 = _tmp23 * _tmp34;
  const Scalar _tmp52 = _tmp35 * _tmp43 - _tmp51;
  const Scalar _tmp53 = _tmp16 + _tmp20;
  const Scalar _tmp54 = _tmp11 + _tmp53;
  const Scalar _tmp55 = _tmp54 + p_init1 + Scalar(-4.7752063900000001);
  const Scalar _tmp56 = _tmp25 + _tmp31;
  const Scalar _tmp57 = _tmp56 + p_init0 + Scalar(-2.71799795);
  const Scalar _tmp58 = std::pow(Scalar(std::pow(_tmp55, Scalar(2)) + std::pow(_tmp57, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp59 = _tmp57 * _tmp58;
  const Scalar _tmp60 = _tmp55 * _tmp58;
  const Scalar _tmp61 = Scalar(1.0) / (_tmp43 * _tmp59 - _tmp60);
  const Scalar _tmp62 = _tmp48 * _tmp59;
  const Scalar _tmp63 = _tmp44 + _tmp45 + _tmp46;
  const Scalar _tmp64 = _tmp61 * (-_tmp43 * _tmp62 + _tmp60 * _tmp63);
  const Scalar _tmp65 = Scalar(1.0) * _tmp40;
  const Scalar _tmp66 = Scalar(1.0) * _tmp37;
  const Scalar _tmp67 = (-_tmp56 + _tmp65) / (_tmp54 - _tmp66);
  const Scalar _tmp68 = _tmp61 * (-_tmp59 * _tmp63 + _tmp62);
  const Scalar _tmp69 = -_tmp35 * _tmp50 + _tmp49 - _tmp52 * _tmp68 -
                        _tmp67 * (-_tmp43 * _tmp49 + _tmp50 * _tmp51 - _tmp52 * _tmp64);
  const Scalar _tmp70 = Scalar(1.0) / (_tmp69);
  const Scalar _tmp71 = Scalar(1.0) * _tmp64 * _tmp67 - Scalar(1.0) * _tmp68;
  const Scalar _tmp72 =
      std::sqrt(Scalar(std::pow(_tmp38, Scalar(2)) + std::pow(_tmp41, Scalar(2))));
  const Scalar _tmp73 = Scalar(1.0) / (_tmp72);
  const Scalar _tmp74 = _tmp42 * _tmp72;
  const Scalar _tmp75 = _tmp74 * (-_tmp37 * _tmp41 * _tmp73 + _tmp38 * _tmp40 * _tmp73);
  const Scalar _tmp76 = _tmp61 * (_tmp54 * _tmp59 - _tmp56 * _tmp60 + _tmp59 * _tmp75);
  const Scalar _tmp77 = _tmp22 * _tmp35 - _tmp32 * _tmp51 + _tmp35 * _tmp75 - _tmp52 * _tmp76;
  const Scalar _tmp78 = _tmp70 * _tmp77;
  const Scalar _tmp79 = Scalar(1.0) / (_tmp77);
  const Scalar _tmp80 = _tmp69 * _tmp79;
  const Scalar _tmp81 = _tmp71 + _tmp80 * (-_tmp71 * _tmp78 - Scalar(1.0) * _tmp76);
  const Scalar _tmp82 = _tmp70 * _tmp81;
  const Scalar _tmp83 = _tmp52 * _tmp70;
  const Scalar _tmp84 = _tmp61 * (-_tmp81 * _tmp83 + Scalar(1.0));
  const Scalar _tmp85 = _tmp25 + _tmp39;
  const Scalar _tmp86 = _tmp85 + p_init0 + Scalar(-2.5202214700000001);
  const Scalar _tmp87 = _tmp36 + _tmp53;
  const Scalar _tmp88 = _tmp87 + p_init1 + Scalar(8.3888750099999996);
  const Scalar _tmp89 = std::pow(Scalar(std::pow(_tmp86, Scalar(2)) + std::pow(_tmp88, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp90 = _tmp88 * _tmp89;
  const Scalar _tmp91 = _tmp74 * _tmp90 * (_tmp35 * _tmp82 + _tmp59 * _tmp84);
  const Scalar _tmp92 = _tmp43 * _tmp68 - _tmp48 - _tmp67 * (_tmp43 * _tmp48 + _tmp43 * _tmp64);
  const Scalar _tmp93 = _tmp80 * (_tmp43 * _tmp76 - _tmp75 - _tmp78 * _tmp92) + _tmp92;
  const Scalar _tmp94 = _tmp61 * (-_tmp43 - _tmp83 * _tmp93);
  const Scalar _tmp95 = _tmp70 * _tmp93;
  const Scalar _tmp96 = _tmp86 * _tmp89;
  const Scalar _tmp97 = _tmp74 * _tmp96 * (_tmp35 * _tmp95 + _tmp59 * _tmp94 + Scalar(1.0));
  const Scalar _tmp98 = _tmp85 * _tmp90 - _tmp87 * _tmp96;
  const Scalar _tmp99 = Scalar(1.0) * _tmp79;
  const Scalar _tmp100 = _tmp52 * _tmp61;
  const Scalar _tmp101 = _tmp100 * _tmp59;
  const Scalar _tmp102 = _tmp74 * _tmp98 * (-_tmp101 * _tmp99 + _tmp35 * _tmp99);
  const Scalar _tmp103 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp104 = _tmp65 + _tmp66 * _tmp67;
  const Scalar _tmp105 = 0;
  const Scalar _tmp106 = _tmp94 * _tmp96;
  const Scalar _tmp107 = _tmp84 * _tmp90;
  const Scalar _tmp108 = _tmp103 * _tmp105;
  const Scalar _tmp109 = _tmp98 * _tmp99;
  const Scalar _tmp110 = _tmp109 * fh1;
  const Scalar _tmp111 = _tmp82 * _tmp90;
  const Scalar _tmp112 = _tmp95 * _tmp96;

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) = -std::exp(-fh1);
  _res(1, 0) = -(-_tmp102 - _tmp91 - _tmp97) *
               std::exp(_tmp102 * fh1 + _tmp103 * _tmp74 * (-_tmp101 * _tmp105 + _tmp105 * _tmp35) +
                        _tmp91 * fh1 + _tmp97 * fh1);
  _res(2, 0) = -(-_tmp100 * _tmp109 + _tmp106 + _tmp107) *
               std::exp(_tmp100 * _tmp108 + _tmp100 * _tmp110 - _tmp106 * fh1 - _tmp107 * fh1);
  _res(3, 0) =
      -(_tmp109 + _tmp111 + _tmp112) * std::exp(-_tmp108 - _tmp110 - _tmp111 * fh1 - _tmp112 * fh1);

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym