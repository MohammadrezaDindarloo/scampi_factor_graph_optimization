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
 * Symbolic function: IK_residual_func_cost1_Nl20
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost1Nl20(
    const Scalar fh1, const Scalar fv1, const Scalar rx, const Scalar ry, const Scalar rz,
    const Scalar p_init0, const Scalar p_init1, const Scalar p_init2, const Scalar rot_init_x,
    const Scalar rot_init_y, const Scalar rot_init_z, const Scalar rot_init_w,
    const Scalar epsilon) {
  // Total ops: 525

  // Unused inputs
  (void)epsilon;

  // Input arrays

  // Intermediate terms (169)
  const Scalar _tmp0 = Scalar(1.0) / (fh1);
  const Scalar _tmp1 = std::asinh(_tmp0 * fv1);
  const Scalar _tmp2 = Scalar(9.6622558468725703) * fh1;
  const Scalar _tmp3 = std::sqrt(
      Scalar(std::pow(rx, Scalar(2)) + std::pow(ry, Scalar(2)) + std::pow(rz, Scalar(2))));
  const Scalar _tmp4 = (Scalar(1) / Scalar(2)) * _tmp3;
  const Scalar _tmp5 = std::cos(_tmp4);
  const Scalar _tmp6 = std::sin(_tmp4) / _tmp3;
  const Scalar _tmp7 = _tmp6 * ry;
  const Scalar _tmp8 = _tmp6 * rot_init_w;
  const Scalar _tmp9 = _tmp6 * rz;
  const Scalar _tmp10 = _tmp5 * rot_init_x - _tmp7 * rot_init_z + _tmp8 * rx + _tmp9 * rot_init_y;
  const Scalar _tmp11 = -2 * std::pow(_tmp10, Scalar(2));
  const Scalar _tmp12 = _tmp6 * rx;
  const Scalar _tmp13 = -_tmp12 * rot_init_y + _tmp5 * rot_init_z + _tmp7 * rot_init_x + _tmp8 * rz;
  const Scalar _tmp14 = -2 * std::pow(_tmp13, Scalar(2));
  const Scalar _tmp15 = Scalar(0.20999999999999999) * _tmp11 +
                        Scalar(0.20999999999999999) * _tmp14 + Scalar(0.20999999999999999);
  const Scalar _tmp16 =
      _tmp12 * rot_init_z + _tmp5 * rot_init_y + _tmp7 * rot_init_w - _tmp9 * rot_init_x;
  const Scalar _tmp17 = 2 * _tmp13 * _tmp16;
  const Scalar _tmp18 = -2 * _tmp12 * rot_init_x + 2 * _tmp5 * rot_init_w - 2 * _tmp7 * rot_init_y -
                        2 * _tmp9 * rot_init_z;
  const Scalar _tmp19 = _tmp10 * _tmp18;
  const Scalar _tmp20 = _tmp17 - _tmp19;
  const Scalar _tmp21 = -Scalar(0.010999999999999999) * _tmp20;
  const Scalar _tmp22 = 2 * _tmp10;
  const Scalar _tmp23 = _tmp16 * _tmp22;
  const Scalar _tmp24 = _tmp13 * _tmp18;
  const Scalar _tmp25 = Scalar(0.20999999999999999) * _tmp23 + Scalar(0.20999999999999999) * _tmp24;
  const Scalar _tmp26 = _tmp21 - _tmp25;
  const Scalar _tmp27 = _tmp15 + _tmp26;
  const Scalar _tmp28 = _tmp27 + p_init1;
  const Scalar _tmp29 = Scalar(0.20999999999999999) * _tmp23 - Scalar(0.20999999999999999) * _tmp24;
  const Scalar _tmp30 = _tmp13 * _tmp22;
  const Scalar _tmp31 = _tmp16 * _tmp18;
  const Scalar _tmp32 = _tmp30 + _tmp31;
  const Scalar _tmp33 = -Scalar(0.010999999999999999) * _tmp32;
  const Scalar _tmp34 = 1 - 2 * std::pow(_tmp16, Scalar(2));
  const Scalar _tmp35 = Scalar(0.20999999999999999) * _tmp14 + Scalar(0.20999999999999999) * _tmp34;
  const Scalar _tmp36 = _tmp33 - _tmp35;
  const Scalar _tmp37 = _tmp29 + _tmp36;
  const Scalar _tmp38 = _tmp37 + p_init0;
  const Scalar _tmp39 = Scalar(0.20999999999999999) * _tmp17 + Scalar(0.20999999999999999) * _tmp19;
  const Scalar _tmp40 =
      -Scalar(0.010999999999999999) * _tmp11 - Scalar(0.010999999999999999) * _tmp34;
  const Scalar _tmp41 = Scalar(0.20999999999999999) * _tmp30 - Scalar(0.20999999999999999) * _tmp31;
  const Scalar _tmp42 = _tmp40 - _tmp41;
  const Scalar _tmp43 = _tmp39 + _tmp42;
  const Scalar _tmp44 = _tmp28 + Scalar(-4.8333311099999996);
  const Scalar _tmp45 = _tmp38 + Scalar(1.79662371);
  const Scalar _tmp46 = std::pow(Scalar(std::pow(_tmp44, Scalar(2)) + std::pow(_tmp45, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp47 = _tmp45 * _tmp46;
  const Scalar _tmp48 = _tmp43 * fh1;
  const Scalar _tmp49 = Scalar(3.29616) * _tmp32 + _tmp37 * fv1 + _tmp47 * _tmp48;
  const Scalar _tmp50 = -_tmp15;
  const Scalar _tmp51 = _tmp21 + _tmp25;
  const Scalar _tmp52 = _tmp50 + _tmp51;
  const Scalar _tmp53 = Scalar(1.0) * _tmp52;
  const Scalar _tmp54 = -_tmp53;
  const Scalar _tmp55 = _tmp15 + _tmp51;
  const Scalar _tmp56 = _tmp54 + _tmp55;
  const Scalar _tmp57 = _tmp26 + _tmp50;
  const Scalar _tmp58 = Scalar(1.0) / (_tmp54 + _tmp57);
  const Scalar _tmp59 = -_tmp29;
  const Scalar _tmp60 = _tmp33 + _tmp35;
  const Scalar _tmp61 = _tmp59 + _tmp60;
  const Scalar _tmp62 = Scalar(1.0) * _tmp61;
  const Scalar _tmp63 = _tmp36 + _tmp59;
  const Scalar _tmp64 = _tmp62 - _tmp63;
  const Scalar _tmp65 = _tmp58 * _tmp64;
  const Scalar _tmp66 = _tmp56 * _tmp65;
  const Scalar _tmp67 = _tmp29 + _tmp60;
  const Scalar _tmp68 = Scalar(1.0) / (_tmp62 - _tmp66 - _tmp67);
  const Scalar _tmp69 = Scalar(1.0) * _tmp68;
  const Scalar _tmp70 = _tmp56 * _tmp58;
  const Scalar _tmp71 = _tmp61 + p_init0;
  const Scalar _tmp72 = _tmp71 + Scalar(-2.5202214700000001);
  const Scalar _tmp73 = _tmp52 + p_init1;
  const Scalar _tmp74 = _tmp73 + Scalar(8.3888750099999996);
  const Scalar _tmp75 =
      std::sqrt(Scalar(std::pow(_tmp72, Scalar(2)) + std::pow(_tmp74, Scalar(2))));
  const Scalar _tmp76 = Scalar(1.0) / (_tmp75);
  const Scalar _tmp77 = Scalar(1.0) / (_tmp72);
  const Scalar _tmp78 = _tmp75 * _tmp77;
  const Scalar _tmp79 = _tmp78 * (-_tmp52 * _tmp72 * _tmp76 + _tmp61 * _tmp74 * _tmp76);
  const Scalar _tmp80 = -_tmp39;
  const Scalar _tmp81 = _tmp40 + _tmp41;
  const Scalar _tmp82 = _tmp80 + _tmp81;
  const Scalar _tmp83 = _tmp42 + _tmp80;
  const Scalar _tmp84 = _tmp57 + p_init1;
  const Scalar _tmp85 = _tmp84 + Scalar(8.3196563700000006);
  const Scalar _tmp86 = _tmp63 + p_init0;
  const Scalar _tmp87 = _tmp86 + Scalar(1.9874742000000001);
  const Scalar _tmp88 = std::pow(Scalar(std::pow(_tmp85, Scalar(2)) + std::pow(_tmp87, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp89 = _tmp87 * _tmp88;
  const Scalar _tmp90 = _tmp82 * _tmp89;
  const Scalar _tmp91 = -_tmp83 * _tmp89 + _tmp90;
  const Scalar _tmp92 = _tmp74 * _tmp77;
  const Scalar _tmp93 = _tmp85 * _tmp88;
  const Scalar _tmp94 = Scalar(1.0) / (_tmp89 * _tmp92 - _tmp93);
  const Scalar _tmp95 = _tmp92 * _tmp94;
  const Scalar _tmp96 = _tmp94 * (_tmp83 * _tmp93 - _tmp90 * _tmp92);
  const Scalar _tmp97 = _tmp82 * _tmp92;
  const Scalar _tmp98 = _tmp92 * _tmp96 + _tmp97;
  const Scalar _tmp99 = -_tmp65 * _tmp98 - _tmp82 + _tmp91 * _tmp95;
  const Scalar _tmp100 = _tmp55 + p_init1;
  const Scalar _tmp101 = _tmp100 + Scalar(-4.7752063900000001);
  const Scalar _tmp102 = _tmp67 + p_init0;
  const Scalar _tmp103 = _tmp102 + Scalar(-2.71799795);
  const Scalar _tmp104 =
      std::pow(Scalar(std::pow(_tmp101, Scalar(2)) + std::pow(_tmp103, Scalar(2))),
               Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp105 = _tmp103 * _tmp104;
  const Scalar _tmp106 = _tmp101 * _tmp104;
  const Scalar _tmp107 = _tmp105 * _tmp92 - _tmp106;
  const Scalar _tmp108 = _tmp107 * _tmp94;
  const Scalar _tmp109 = _tmp39 + _tmp81;
  const Scalar _tmp110 = -_tmp105 * _tmp97 + _tmp106 * _tmp109 - _tmp107 * _tmp96;
  const Scalar _tmp111 =
      -_tmp105 * _tmp109 + _tmp105 * _tmp82 - _tmp108 * _tmp91 - _tmp110 * _tmp65;
  const Scalar _tmp112 = Scalar(1.0) / (_tmp111);
  const Scalar _tmp113 = _tmp57 * _tmp89 - _tmp63 * _tmp93 + _tmp79 * _tmp89;
  const Scalar _tmp114 = _tmp105 * _tmp55 + _tmp105 * _tmp79 - _tmp106 * _tmp67 - _tmp108 * _tmp113;
  const Scalar _tmp115 = _tmp112 * _tmp114;
  const Scalar _tmp116 = Scalar(1.0) / (_tmp114);
  const Scalar _tmp117 = _tmp111 * _tmp116;
  const Scalar _tmp118 = _tmp117 * (_tmp113 * _tmp95 - _tmp115 * _tmp99 - _tmp79);
  const Scalar _tmp119 = _tmp56 * _tmp68;
  const Scalar _tmp120 = _tmp118 + _tmp99;
  const Scalar _tmp121 = _tmp110 * _tmp112;
  const Scalar _tmp122 = _tmp118 * _tmp119 - _tmp120 * _tmp121 + _tmp98;
  const Scalar _tmp123 = Scalar(1.0) * _tmp58;
  const Scalar _tmp124 = Scalar(1.0) * fh1;
  const Scalar _tmp125 = _tmp117 * _tmp69;
  const Scalar _tmp126 = Scalar(1.0) * _tmp116;
  const Scalar _tmp127 = -_tmp110 * _tmp126 + _tmp125 * _tmp56;
  const Scalar _tmp128 = _tmp44 * _tmp46;
  const Scalar _tmp129 = fh1 * (_tmp128 * _tmp37 - _tmp27 * _tmp47);
  const Scalar _tmp130 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp131 = _tmp53 * _tmp65 + _tmp62;
  const Scalar _tmp132 = _tmp131 * _tmp68;
  const Scalar _tmp133 = 0;
  const Scalar _tmp134 = -_tmp121 * _tmp133 - _tmp132 * _tmp56 + _tmp54;
  const Scalar _tmp135 = -_tmp128 * _tmp48 - Scalar(3.29616) * _tmp20 - _tmp27 * fv1;
  const Scalar _tmp136 = _tmp66 * _tmp69 + Scalar(1.0);
  const Scalar _tmp137 = _tmp65 * _tmp69;
  const Scalar _tmp138 = Scalar(1.0) * _tmp94;
  const Scalar _tmp139 = _tmp123 * _tmp64 * _tmp96 - _tmp138 * _tmp91;
  const Scalar _tmp140 = _tmp117 * (-_tmp113 * _tmp138 - _tmp115 * _tmp139);
  const Scalar _tmp141 = _tmp139 + _tmp140;
  const Scalar _tmp142 = _tmp119 * _tmp140 - _tmp121 * _tmp141 - Scalar(1.0) * _tmp96;
  const Scalar _tmp143 = _tmp89 * _tmp94;
  const Scalar _tmp144 = _tmp107 * _tmp112;
  const Scalar _tmp145 = _tmp133 * _tmp144;
  const Scalar _tmp146 = _tmp112 * _tmp133;
  const Scalar _tmp147 = -_tmp120 * _tmp144 - _tmp92;
  const Scalar _tmp148 = _tmp105 * _tmp112;
  const Scalar _tmp149 = _tmp78 * fh1;
  const Scalar _tmp150 = _tmp107 * _tmp116 * _tmp138;
  const Scalar _tmp151 = -_tmp141 * _tmp144 + Scalar(1.0);
  const Scalar _tmp152 = -_tmp128 * _tmp149 * (_tmp141 * _tmp148 + _tmp143 * _tmp151) -
                         _tmp129 * _tmp78 * (_tmp105 * _tmp126 - _tmp150 * _tmp89) -
                         _tmp130 * _tmp78 * (_tmp105 * _tmp146 - _tmp143 * _tmp145) -
                         _tmp149 * _tmp47 * (_tmp120 * _tmp148 + _tmp143 * _tmp147 + Scalar(1.0));
  const Scalar _tmp153 = Scalar(1.0) / (_tmp152);
  const Scalar _tmp154 = std::asinh(
      _tmp153 * (_tmp124 * _tmp128 * (-_tmp123 * _tmp142 + _tmp140 * _tmp69) +
                 _tmp124 * _tmp47 * (_tmp118 * _tmp69 - _tmp122 * _tmp123) +
                 Scalar(1.0) * _tmp129 * (-_tmp123 * _tmp127 + _tmp125) +
                 Scalar(1.0) * _tmp130 * (-_tmp123 * _tmp134 - _tmp131 * _tmp69 + Scalar(1.0)) +
                 Scalar(1.0) * _tmp135 * (-_tmp123 * _tmp136 + _tmp137) +
                 Scalar(1.0) * _tmp49 * (_tmp69 * _tmp70 - _tmp69)));
  const Scalar _tmp155 = Scalar(9.6622558468725703) * _tmp152;
  const Scalar _tmp156 = _tmp94 * fh1;
  const Scalar _tmp157 = _tmp128 * _tmp151 * _tmp156 - _tmp129 * _tmp150 -
                         _tmp130 * _tmp145 * _tmp94 + _tmp147 * _tmp156 * _tmp47;
  const Scalar _tmp158 = Scalar(1.0) / (_tmp157);
  const Scalar _tmp159 = _tmp58 * fh1;
  const Scalar _tmp160 = _tmp49 * _tmp69;
  const Scalar _tmp161 =
      std::asinh(_tmp158 * (_tmp122 * _tmp159 * _tmp47 + _tmp127 * _tmp129 * _tmp58 +
                            _tmp128 * _tmp142 * _tmp159 + _tmp130 * _tmp134 * _tmp58 +
                            _tmp135 * _tmp136 * _tmp58 - _tmp160 * _tmp70));
  const Scalar _tmp162 = Scalar(9.6622558468725703) * _tmp157;
  const Scalar _tmp163 = _tmp112 * fh1;
  const Scalar _tmp164 = _tmp120 * _tmp163 * _tmp47 + _tmp126 * _tmp129 +
                         _tmp128 * _tmp141 * _tmp163 + _tmp130 * _tmp146;
  const Scalar _tmp165 = Scalar(1.0) / (_tmp164);
  const Scalar _tmp166 = _tmp68 * fh1;
  const Scalar _tmp167 = std::asinh(_tmp165 * (-_tmp118 * _tmp166 * _tmp47 - _tmp125 * _tmp129 -
                                               _tmp128 * _tmp140 * _tmp166 + _tmp130 * _tmp132 -
                                               _tmp135 * _tmp137 + _tmp160));
  const Scalar _tmp168 = Scalar(9.6622558468725703) * _tmp164;

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) =
      -_tmp2 * (Scalar(0.86625939559540499) * _tmp0 + std::cosh(Scalar(1.0) * _tmp1) -
                std::cosh(
                    Scalar(0.1034955) * _tmp0 *
                    (-_tmp1 * _tmp2 -
                     Scalar(4.8333311099999996) *
                         std::sqrt(Scalar(
                             std::pow(Scalar(1 - Scalar(0.20689664689659551) * _tmp28), Scalar(2)) +
                             Scalar(0.13817235445745474) *
                                 std::pow(Scalar(-Scalar(0.55659957866191134) * _tmp38 - 1),
                                          Scalar(2))))))) +
      _tmp43 + p_init2;
  _res(1, 0) =
      -_tmp155 *
          (Scalar(0.87653584775870996) * _tmp153 + std::cosh(Scalar(1.0) * _tmp154) -
           std::cosh(
               Scalar(0.1034955) * _tmp153 *
               (-_tmp154 * _tmp155 -
                Scalar(8.3888750099999996) *
                    std::sqrt(Scalar(
                        Scalar(0.090254729040973036) *
                            std::pow(Scalar(1 - Scalar(0.39679052492160538) * _tmp71), Scalar(2)) +
                        std::pow(Scalar(-Scalar(0.11920549523123722) * _tmp73 - 1),
                                 Scalar(2))))))) +
      _tmp82 + p_init2;
  _res(2, 0) =
      -_tmp162 *
          (Scalar(0.87679799772039002) * _tmp158 + std::cosh(Scalar(1.0) * _tmp161) -
           std::cosh(
               Scalar(0.1034955) * _tmp158 *
               (-_tmp161 * _tmp162 -
                Scalar(8.3196563700000006) *
                    std::sqrt(Scalar(
                        std::pow(Scalar(-Scalar(0.12019727204189803) * _tmp84 - 1), Scalar(2)) +
                        Scalar(0.057067943376852184) *
                            std::pow(Scalar(-Scalar(0.50315118556004401) * _tmp86 - 1),
                                     Scalar(2))))))) +
      _tmp83 + p_init2;
  _res(3, 0) =
      _tmp109 -
      _tmp168 *
          (Scalar(0.86565325453551001) * _tmp165 + std::cosh(Scalar(1.0) * _tmp167) -
           std::cosh(
               Scalar(0.1034955) * _tmp165 *
               (-_tmp167 * _tmp168 -
                Scalar(4.7752063900000001) *
                    std::sqrt(Scalar(
                        std::pow(Scalar(1 - Scalar(0.20941503221602112) * _tmp100), Scalar(2)) +
                        Scalar(0.32397683292140877) *
                            std::pow(Scalar(1 - Scalar(0.36791786395571047) * _tmp102),
                                     Scalar(2))))))) +
      p_init2;

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
