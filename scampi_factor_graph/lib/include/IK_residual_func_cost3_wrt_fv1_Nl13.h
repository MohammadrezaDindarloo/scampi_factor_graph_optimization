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
 * Symbolic function: IK_residual_func_cost3_wrt_fv1_Nl13
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost3WrtFv1Nl13(
    const Scalar fh1, const Scalar fv1, const Scalar rx, const Scalar ry, const Scalar rz,
    const Scalar p_init0, const Scalar p_init1, const Scalar p_init2, const Scalar rot_init_x,
    const Scalar rot_init_y, const Scalar rot_init_z, const Scalar rot_init_w,
    const Scalar epsilon) {
  // Total ops: 305

  // Unused inputs
  (void)p_init2;
  (void)epsilon;

  // Input arrays

  // Intermediate terms (114)
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
      _tmp3 * rot_init_y + _tmp5 * rot_init_w + _tmp6 * rot_init_z - _tmp7 * rot_init_x;
  const Scalar _tmp9 = -2 * std::pow(_tmp8, Scalar(2));
  const Scalar _tmp10 = _tmp4 * rot_init_y;
  const Scalar _tmp11 = _tmp4 * rot_init_w;
  const Scalar _tmp12 = -_tmp10 * rx + _tmp11 * rz + _tmp3 * rot_init_z + _tmp5 * rot_init_x;
  const Scalar _tmp13 = 1 - 2 * std::pow(_tmp12, Scalar(2));
  const Scalar _tmp14 = Scalar(0.20999999999999999) * _tmp13 + Scalar(0.20999999999999999) * _tmp9;
  const Scalar _tmp15 = _tmp10 * rz + _tmp11 * rx + _tmp3 * rot_init_x - _tmp5 * rot_init_z;
  const Scalar _tmp16 = 2 * _tmp15 * _tmp8;
  const Scalar _tmp17 =
      -2 * _tmp10 * ry + 2 * _tmp3 * rot_init_w - 2 * _tmp6 * rot_init_x - 2 * _tmp7 * rot_init_z;
  const Scalar _tmp18 = _tmp12 * _tmp17;
  const Scalar _tmp19 = Scalar(0.20999999999999999) * _tmp16 - Scalar(0.20999999999999999) * _tmp18;
  const Scalar _tmp20 = 2 * _tmp12;
  const Scalar _tmp21 = _tmp15 * _tmp20;
  const Scalar _tmp22 = _tmp17 * _tmp8;
  const Scalar _tmp23 =
      -Scalar(0.010999999999999999) * _tmp21 - Scalar(0.010999999999999999) * _tmp22;
  const Scalar _tmp24 = -_tmp19 + _tmp23;
  const Scalar _tmp25 = _tmp14 + _tmp24;
  const Scalar _tmp26 = _tmp25 + p_init0 + Scalar(-2.5202214700000001);
  const Scalar _tmp27 = -2 * std::pow(_tmp15, Scalar(2));
  const Scalar _tmp28 = Scalar(0.20999999999999999) * _tmp13 + Scalar(0.20999999999999999) * _tmp27;
  const Scalar _tmp29 = -_tmp28;
  const Scalar _tmp30 = _tmp20 * _tmp8;
  const Scalar _tmp31 = _tmp15 * _tmp17;
  const Scalar _tmp32 =
      -Scalar(0.010999999999999999) * _tmp30 + Scalar(0.010999999999999999) * _tmp31;
  const Scalar _tmp33 = Scalar(0.20999999999999999) * _tmp16 + Scalar(0.20999999999999999) * _tmp18;
  const Scalar _tmp34 = _tmp32 + _tmp33;
  const Scalar _tmp35 = _tmp29 + _tmp34;
  const Scalar _tmp36 = _tmp35 + p_init1 + Scalar(8.3888750099999996);
  const Scalar _tmp37 = std::pow(Scalar(std::pow(_tmp26, Scalar(2)) + std::pow(_tmp36, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp38 = _tmp26 * _tmp37;
  const Scalar _tmp39 = -Scalar(0.010999999999999999) * _tmp27 -
                        Scalar(0.010999999999999999) * _tmp9 + Scalar(-0.010999999999999999);
  const Scalar _tmp40 = Scalar(0.20999999999999999) * _tmp30 + Scalar(0.20999999999999999) * _tmp31;
  const Scalar _tmp41 = -_tmp40;
  const Scalar _tmp42 = Scalar(0.20999999999999999) * _tmp21 - Scalar(0.20999999999999999) * _tmp22;
  const Scalar _tmp43 = _tmp39 + _tmp41 + _tmp42;
  const Scalar _tmp44 = _tmp36 * _tmp37;
  const Scalar _tmp45 = _tmp32 - _tmp33;
  const Scalar _tmp46 = _tmp29 + _tmp45;
  const Scalar _tmp47 = _tmp46 + p_init1 + Scalar(8.3196563700000006);
  const Scalar _tmp48 = -_tmp14;
  const Scalar _tmp49 = _tmp24 + _tmp48;
  const Scalar _tmp50 = _tmp49 + p_init0 + Scalar(1.9874742000000001);
  const Scalar _tmp51 = Scalar(1.0) / (_tmp50);
  const Scalar _tmp52 = _tmp47 * _tmp51;
  const Scalar _tmp53 = _tmp39 - _tmp42;
  const Scalar _tmp54 = _tmp41 + _tmp53;
  const Scalar _tmp55 = _tmp38 * _tmp54;
  const Scalar _tmp56 = _tmp28 + _tmp45;
  const Scalar _tmp57 = _tmp56 + p_init1 + Scalar(-4.8333311099999996);
  const Scalar _tmp58 = _tmp19 + _tmp23;
  const Scalar _tmp59 = _tmp48 + _tmp58;
  const Scalar _tmp60 = _tmp59 + p_init0 + Scalar(1.79662371);
  const Scalar _tmp61 = std::pow(Scalar(std::pow(_tmp57, Scalar(2)) + std::pow(_tmp60, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp62 = _tmp60 * _tmp61;
  const Scalar _tmp63 = _tmp54 * _tmp62;
  const Scalar _tmp64 = _tmp40 + _tmp53;
  const Scalar _tmp65 = _tmp57 * _tmp61;
  const Scalar _tmp66 = -_tmp52 * _tmp63 + _tmp64 * _tmp65;
  const Scalar _tmp67 = _tmp38 * _tmp52 - _tmp44;
  const Scalar _tmp68 = Scalar(1.0) / (_tmp52 * _tmp62 - _tmp65);
  const Scalar _tmp69 = _tmp67 * _tmp68;
  const Scalar _tmp70 = Scalar(1.0) * _tmp46;
  const Scalar _tmp71 = Scalar(1.0) * _tmp49;
  const Scalar _tmp72 = (-_tmp59 + _tmp71) / (_tmp56 - _tmp70);
  const Scalar _tmp73 = -_tmp62 * _tmp64 + _tmp63;
  const Scalar _tmp74 = -_tmp38 * _tmp43 + _tmp55 - _tmp69 * _tmp73 -
                        _tmp72 * (_tmp43 * _tmp44 - _tmp52 * _tmp55 - _tmp66 * _tmp69);
  const Scalar _tmp75 = Scalar(1.0) / (_tmp74);
  const Scalar _tmp76 = _tmp70 * _tmp72 + _tmp71;
  const Scalar _tmp77 = 0;
  const Scalar _tmp78 = _tmp75 * _tmp77;
  const Scalar _tmp79 = _tmp62 * _tmp68;
  const Scalar _tmp80 = _tmp67 * _tmp75;
  const Scalar _tmp81 = _tmp77 * _tmp80;
  const Scalar _tmp82 =
      std::sqrt(Scalar(std::pow(_tmp47, Scalar(2)) + std::pow(_tmp50, Scalar(2))));
  const Scalar _tmp83 = _tmp51 * _tmp82;
  const Scalar _tmp84 = _tmp83 * (_tmp38 * _tmp78 - _tmp79 * _tmp81);
  const Scalar _tmp85 = Scalar(1.0) / (_tmp82);
  const Scalar _tmp86 = _tmp83 * (-_tmp46 * _tmp50 * _tmp85 + _tmp47 * _tmp49 * _tmp85);
  const Scalar _tmp87 = _tmp52 * _tmp68;
  const Scalar _tmp88 = -_tmp54 - _tmp72 * (_tmp52 * _tmp54 + _tmp66 * _tmp87) + _tmp73 * _tmp87;
  const Scalar _tmp89 = _tmp56 * _tmp62 - _tmp59 * _tmp65 + _tmp62 * _tmp86;
  const Scalar _tmp90 = -_tmp25 * _tmp44 + _tmp35 * _tmp38 + _tmp38 * _tmp86 - _tmp69 * _tmp89;
  const Scalar _tmp91 = _tmp75 * _tmp90;
  const Scalar _tmp92 = Scalar(1.0) / (_tmp90);
  const Scalar _tmp93 = _tmp74 * _tmp92;
  const Scalar _tmp94 = _tmp88 + _tmp93 * (-_tmp86 + _tmp87 * _tmp89 - _tmp88 * _tmp91);
  const Scalar _tmp95 = -_tmp52 - _tmp80 * _tmp94;
  const Scalar _tmp96 = _tmp38 * _tmp75;
  const Scalar _tmp97 = _tmp28 + _tmp34;
  const Scalar _tmp98 = _tmp97 + p_init1 + Scalar(-4.7752063900000001);
  const Scalar _tmp99 = _tmp14 + _tmp58;
  const Scalar _tmp100 = _tmp99 + p_init0 + Scalar(-2.71799795);
  const Scalar _tmp101 =
      std::pow(Scalar(std::pow(_tmp100, Scalar(2)) + std::pow(_tmp98, Scalar(2))),
               Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp102 = _tmp100 * _tmp101;
  const Scalar _tmp103 = _tmp102 * fh1;
  const Scalar _tmp104 = Scalar(1.0) * _tmp68;
  const Scalar _tmp105 = _tmp104 * _tmp66 * _tmp72 - _tmp104 * _tmp73;
  const Scalar _tmp106 = _tmp105 + _tmp93 * (-_tmp104 * _tmp89 - _tmp105 * _tmp91);
  const Scalar _tmp107 = -_tmp106 * _tmp80 + Scalar(1.0);
  const Scalar _tmp108 = _tmp101 * _tmp98;
  const Scalar _tmp109 = _tmp108 * fh1;
  const Scalar _tmp110 = Scalar(1.0) * _tmp92;
  const Scalar _tmp111 = fh1 * (-_tmp102 * _tmp97 + _tmp108 * _tmp99);
  const Scalar _tmp112 = _tmp110 * _tmp111;
  const Scalar _tmp113 = _tmp68 * _tmp81;

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) = 0;
  _res(1, 0) =
      -_tmp84 * std::exp(_tmp0 * _tmp84 +
                         _tmp103 * _tmp83 * (_tmp79 * _tmp95 + _tmp94 * _tmp96 + Scalar(1.0)) +
                         _tmp109 * _tmp83 * (_tmp106 * _tmp96 + _tmp107 * _tmp79) +
                         _tmp111 * _tmp83 * (_tmp110 * _tmp38 - _tmp110 * _tmp62 * _tmp69));
  _res(2, 0) = -_tmp113 * std::exp(_tmp0 * _tmp113 - _tmp103 * _tmp68 * _tmp95 -
                                   _tmp107 * _tmp109 * _tmp68 + _tmp112 * _tmp69);
  _res(3, 0) = _tmp78 * std::exp(-_tmp0 * _tmp78 - _tmp103 * _tmp75 * _tmp94 -
                                 _tmp106 * _tmp109 * _tmp75 - _tmp112);

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
