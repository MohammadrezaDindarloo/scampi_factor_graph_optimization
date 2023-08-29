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
 * Symbolic function: IK_residual_func_cost2_Nl1
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost2Nl1(const Scalar fh1, const Scalar fv1,
                                                   const Scalar rx, const Scalar ry,
                                                   const Scalar rz, const Scalar p_init0,
                                                   const Scalar p_init1, const Scalar p_init2,
                                                   const Scalar rot_init_x, const Scalar rot_init_y,
                                                   const Scalar rot_init_z, const Scalar rot_init_w,
                                                   const Scalar epsilon) {
  // Total ops: 545

  // Unused inputs
  (void)epsilon;

  // Input arrays

  // Intermediate terms (168)
  const Scalar _tmp0 = std::sqrt(
      Scalar(std::pow(rx, Scalar(2)) + std::pow(ry, Scalar(2)) + std::pow(rz, Scalar(2))));
  const Scalar _tmp1 = (Scalar(1) / Scalar(2)) * _tmp0;
  const Scalar _tmp2 = std::cos(_tmp1);
  const Scalar _tmp3 = std::sin(_tmp1) / _tmp0;
  const Scalar _tmp4 = _tmp3 * ry;
  const Scalar _tmp5 = _tmp3 * rx;
  const Scalar _tmp6 = _tmp3 * rz;
  const Scalar _tmp7 =
      _tmp2 * rot_init_z + _tmp4 * rot_init_x - _tmp5 * rot_init_y + _tmp6 * rot_init_w;
  const Scalar _tmp8 =
      _tmp2 * rot_init_x - _tmp4 * rot_init_z + _tmp5 * rot_init_w + _tmp6 * rot_init_y;
  const Scalar _tmp9 = 2 * _tmp8;
  const Scalar _tmp10 = _tmp7 * _tmp9;
  const Scalar _tmp11 =
      _tmp2 * rot_init_y + _tmp4 * rot_init_w + _tmp5 * rot_init_z - _tmp6 * rot_init_x;
  const Scalar _tmp12 = 2 * _tmp2 * rot_init_w - 2 * _tmp4 * rot_init_y - 2 * _tmp5 * rot_init_x -
                        2 * _tmp6 * rot_init_z;
  const Scalar _tmp13 = _tmp11 * _tmp12;
  const Scalar _tmp14 = Scalar(0.20999999999999999) * _tmp10 - Scalar(0.20999999999999999) * _tmp13;
  const Scalar _tmp15 = -_tmp14;
  const Scalar _tmp16 = -2 * std::pow(_tmp8, Scalar(2));
  const Scalar _tmp17 = -2 * std::pow(_tmp11, Scalar(2));
  const Scalar _tmp18 = -Scalar(0.010999999999999999) * _tmp16 -
                        Scalar(0.010999999999999999) * _tmp17 + Scalar(-0.010999999999999999);
  const Scalar _tmp19 = 2 * _tmp11 * _tmp7;
  const Scalar _tmp20 = _tmp12 * _tmp8;
  const Scalar _tmp21 = Scalar(0.20999999999999999) * _tmp19 + Scalar(0.20999999999999999) * _tmp20;
  const Scalar _tmp22 = _tmp18 - _tmp21;
  const Scalar _tmp23 = _tmp15 + _tmp22;
  const Scalar _tmp24 = 1 - 2 * std::pow(_tmp7, Scalar(2));
  const Scalar _tmp25 = Scalar(0.20999999999999999) * _tmp16 + Scalar(0.20999999999999999) * _tmp24;
  const Scalar _tmp26 = -_tmp25;
  const Scalar _tmp27 = _tmp19 - _tmp20;
  const Scalar _tmp28 = -Scalar(0.010999999999999999) * _tmp27;
  const Scalar _tmp29 = _tmp11 * _tmp9;
  const Scalar _tmp30 = _tmp12 * _tmp7;
  const Scalar _tmp31 = Scalar(0.20999999999999999) * _tmp29 + Scalar(0.20999999999999999) * _tmp30;
  const Scalar _tmp32 = _tmp28 - _tmp31;
  const Scalar _tmp33 = _tmp26 + _tmp32;
  const Scalar _tmp34 = _tmp33 + p_init1;
  const Scalar _tmp35 = Scalar(0.20999999999999999) * _tmp17 + Scalar(0.20999999999999999) * _tmp24;
  const Scalar _tmp36 = -_tmp35;
  const Scalar _tmp37 = Scalar(0.20999999999999999) * _tmp29 - Scalar(0.20999999999999999) * _tmp30;
  const Scalar _tmp38 = _tmp10 + _tmp13;
  const Scalar _tmp39 = -Scalar(0.010999999999999999) * _tmp38;
  const Scalar _tmp40 = -_tmp37 + _tmp39;
  const Scalar _tmp41 = _tmp36 + _tmp40;
  const Scalar _tmp42 = _tmp41 + p_init0;
  const Scalar _tmp43 = Scalar(69.216682114881593) *
                            std::pow(Scalar(-Scalar(0.12019727204189803) * _tmp34 - 1), Scalar(2)) +
                        Scalar(3.9500536956656402) *
                            std::pow(Scalar(-Scalar(0.50315118556004401) * _tmp42 - 1), Scalar(2));
  const Scalar _tmp44 = Scalar(1.0) / (fh1);
  const Scalar _tmp45 = std::asinh(_tmp44 * fv1);
  const Scalar _tmp46 = Scalar(9.6622558468725703) * fh1;
  const Scalar _tmp47 = _tmp14 + _tmp22;
  const Scalar _tmp48 = _tmp28 + _tmp31;
  const Scalar _tmp49 = _tmp26 + _tmp48;
  const Scalar _tmp50 = _tmp49 + p_init1;
  const Scalar _tmp51 = _tmp35 + _tmp40;
  const Scalar _tmp52 = _tmp51 + p_init0;
  const Scalar _tmp53 = Scalar(6.351516257848961) *
                            std::pow(Scalar(1 - Scalar(0.39679052492160538) * _tmp52), Scalar(2)) +
                        Scalar(70.3732239334025) *
                            std::pow(Scalar(-Scalar(0.11920549523123722) * _tmp50 - 1), Scalar(2));
  const Scalar _tmp54 = _tmp52 + Scalar(-2.5202214700000001);
  const Scalar _tmp55 = _tmp50 + Scalar(8.3888750099999996);
  const Scalar _tmp56 =
      std::sqrt(Scalar(std::pow(_tmp54, Scalar(2)) + std::pow(_tmp55, Scalar(2))));
  const Scalar _tmp57 = Scalar(1.0) / (_tmp56);
  const Scalar _tmp58 = Scalar(1.0) / (_tmp54);
  const Scalar _tmp59 = _tmp56 * _tmp58;
  const Scalar _tmp60 = _tmp59 * (-_tmp49 * _tmp54 * _tmp57 + _tmp51 * _tmp55 * _tmp57);
  const Scalar _tmp61 = _tmp55 * _tmp58;
  const Scalar _tmp62 = _tmp25 + _tmp32;
  const Scalar _tmp63 = _tmp62 + p_init1;
  const Scalar _tmp64 = _tmp63 + Scalar(-4.8333311099999996);
  const Scalar _tmp65 = _tmp37 + _tmp39;
  const Scalar _tmp66 = _tmp36 + _tmp65;
  const Scalar _tmp67 = _tmp66 + p_init0;
  const Scalar _tmp68 = _tmp67 + Scalar(1.79662371);
  const Scalar _tmp69 = std::pow(Scalar(std::pow(_tmp64, Scalar(2)) + std::pow(_tmp68, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp70 = _tmp68 * _tmp69;
  const Scalar _tmp71 = _tmp47 * _tmp70;
  const Scalar _tmp72 = _tmp18 + _tmp21;
  const Scalar _tmp73 = _tmp15 + _tmp72;
  const Scalar _tmp74 = _tmp64 * _tmp69;
  const Scalar _tmp75 = -_tmp61 * _tmp71 + _tmp73 * _tmp74;
  const Scalar _tmp76 = Scalar(1.0) / (_tmp61 * _tmp70 - _tmp74);
  const Scalar _tmp77 = _tmp61 * _tmp76;
  const Scalar _tmp78 = _tmp47 * _tmp61 + _tmp75 * _tmp77;
  const Scalar _tmp79 = Scalar(1.0) * _tmp49;
  const Scalar _tmp80 = -_tmp79;
  const Scalar _tmp81 = Scalar(1.0) / (_tmp62 + _tmp80);
  const Scalar _tmp82 = Scalar(1.0) * _tmp51;
  const Scalar _tmp83 = -_tmp66 + _tmp82;
  const Scalar _tmp84 = _tmp81 * _tmp83;
  const Scalar _tmp85 = _tmp76 * (-_tmp70 * _tmp73 + _tmp71);
  const Scalar _tmp86 = -_tmp47 + _tmp61 * _tmp85 - _tmp78 * _tmp84;
  const Scalar _tmp87 = _tmp25 + _tmp48;
  const Scalar _tmp88 = _tmp87 + p_init1;
  const Scalar _tmp89 = _tmp88 + Scalar(-4.7752063900000001);
  const Scalar _tmp90 = _tmp35 + _tmp65;
  const Scalar _tmp91 = _tmp90 + p_init0;
  const Scalar _tmp92 = _tmp91 + Scalar(-2.71799795);
  const Scalar _tmp93 = std::pow(Scalar(std::pow(_tmp89, Scalar(2)) + std::pow(_tmp92, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp94 = _tmp92 * _tmp93;
  const Scalar _tmp95 = _tmp89 * _tmp93;
  const Scalar _tmp96 = _tmp61 * _tmp94 - _tmp95;
  const Scalar _tmp97 = _tmp47 * _tmp94;
  const Scalar _tmp98 = _tmp14 + _tmp72;
  const Scalar _tmp99 = _tmp76 * _tmp96;
  const Scalar _tmp100 = -_tmp61 * _tmp97 - _tmp75 * _tmp99 + _tmp95 * _tmp98;
  const Scalar _tmp101 = -_tmp100 * _tmp84 - _tmp85 * _tmp96 - _tmp94 * _tmp98 + _tmp97;
  const Scalar _tmp102 = Scalar(1.0) / (_tmp101);
  const Scalar _tmp103 = _tmp60 * _tmp70 + _tmp62 * _tmp70 - _tmp66 * _tmp74;
  const Scalar _tmp104 = -_tmp103 * _tmp99 + _tmp60 * _tmp94 + _tmp87 * _tmp94 - _tmp90 * _tmp95;
  const Scalar _tmp105 = _tmp102 * _tmp104;
  const Scalar _tmp106 = Scalar(1.0) / (_tmp104);
  const Scalar _tmp107 = _tmp101 * _tmp106;
  const Scalar _tmp108 = _tmp107 * (_tmp103 * _tmp77 - _tmp105 * _tmp86 - _tmp60);
  const Scalar _tmp109 = _tmp108 + _tmp86;
  const Scalar _tmp110 = _tmp100 * _tmp102;
  const Scalar _tmp111 = _tmp80 + _tmp87;
  const Scalar _tmp112 = _tmp111 * _tmp84;
  const Scalar _tmp113 = Scalar(1.0) / (-_tmp112 + _tmp82 - _tmp90);
  const Scalar _tmp114 = _tmp111 * _tmp113;
  const Scalar _tmp115 = _tmp108 * _tmp114 - _tmp109 * _tmp110 + _tmp78;
  const Scalar _tmp116 = Scalar(1.0) * _tmp81;
  const Scalar _tmp117 = Scalar(1.0) * _tmp113;
  const Scalar _tmp118 = _tmp34 + Scalar(8.3196563700000006);
  const Scalar _tmp119 = _tmp42 + Scalar(1.9874742000000001);
  const Scalar _tmp120 =
      std::pow(Scalar(std::pow(_tmp118, Scalar(2)) + std::pow(_tmp119, Scalar(2))),
               Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp121 = _tmp119 * _tmp120;
  const Scalar _tmp122 = _tmp121 * fh1;
  const Scalar _tmp123 = _tmp116 * _tmp75 * _tmp76 * _tmp83 - Scalar(1.0) * _tmp85;
  const Scalar _tmp124 = Scalar(1.0) * _tmp76;
  const Scalar _tmp125 = _tmp107 * (-_tmp103 * _tmp124 - _tmp105 * _tmp123);
  const Scalar _tmp126 = _tmp123 + _tmp125;
  const Scalar _tmp127 = -_tmp110 * _tmp126 + _tmp114 * _tmp125 - _tmp124 * _tmp75;
  const Scalar _tmp128 = _tmp118 * _tmp120;
  const Scalar _tmp129 = _tmp128 * fh1;
  const Scalar _tmp130 = _tmp79 * _tmp84 + _tmp82;
  const Scalar _tmp131 = _tmp113 * _tmp130;
  const Scalar _tmp132 = 0;
  const Scalar _tmp133 = _tmp81 * (-_tmp100 * _tmp132 - _tmp111 * _tmp131 + _tmp80);
  const Scalar _tmp134 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp135 = _tmp23 * fh1;
  const Scalar _tmp136 = -_tmp128 * _tmp135 - Scalar(3.29616) * _tmp27 - _tmp33 * fv1;
  const Scalar _tmp137 = _tmp81 * (_tmp112 * _tmp117 + Scalar(1.0));
  const Scalar _tmp138 = _tmp117 * _tmp84;
  const Scalar _tmp139 = _tmp107 * _tmp117;
  const Scalar _tmp140 = Scalar(1.0) * _tmp106;
  const Scalar _tmp141 = -_tmp100 * _tmp140 + _tmp111 * _tmp139;
  const Scalar _tmp142 = fh1 * (-_tmp121 * _tmp33 + _tmp128 * _tmp41);
  const Scalar _tmp143 = _tmp121 * _tmp135 + Scalar(3.29616) * _tmp38 + _tmp41 * fv1;
  const Scalar _tmp144 = _tmp111 * _tmp81;
  const Scalar _tmp145 = _tmp102 * _tmp96;
  const Scalar _tmp146 = -_tmp126 * _tmp145 + Scalar(1.0);
  const Scalar _tmp147 = _tmp70 * _tmp76;
  const Scalar _tmp148 = _tmp102 * _tmp94;
  const Scalar _tmp149 = _tmp70 * _tmp99;
  const Scalar _tmp150 = -_tmp109 * _tmp145 - _tmp61;
  const Scalar _tmp151 = -_tmp122 * _tmp59 * (_tmp109 * _tmp148 + _tmp147 * _tmp150 + Scalar(1.0)) -
                         _tmp129 * _tmp59 * (_tmp126 * _tmp148 + _tmp146 * _tmp147) -
                         _tmp134 * _tmp59 * (-_tmp132 * _tmp149 + _tmp132 * _tmp94) -
                         _tmp142 * _tmp59 * (-_tmp140 * _tmp149 + _tmp140 * _tmp94);
  const Scalar _tmp152 = Scalar(1.0) / (_tmp151);
  const Scalar _tmp153 =
      std::asinh(_tmp152 * (Scalar(1.0) * _tmp122 * (_tmp108 * _tmp117 - _tmp115 * _tmp116) +
                            Scalar(1.0) * _tmp129 * (-_tmp116 * _tmp127 + _tmp117 * _tmp125) +
                            Scalar(1.0) * _tmp134 *
                                (-_tmp117 * _tmp130 - Scalar(1.0) * _tmp133 + Scalar(1.0)) +
                            Scalar(1.0) * _tmp136 * (-Scalar(1.0) * _tmp137 + _tmp138) +
                            Scalar(1.0) * _tmp142 * (-_tmp116 * _tmp141 + _tmp139) +
                            Scalar(1.0) * _tmp143 * (_tmp117 * _tmp144 - _tmp117)));
  const Scalar _tmp154 = Scalar(9.6622558468725703) * _tmp151;
  const Scalar _tmp155 = Scalar(23.361089618893828) *
                             std::pow(Scalar(1 - Scalar(0.20689664689659551) * _tmp63), Scalar(2)) +
                         Scalar(3.2278567553341642) *
                             std::pow(Scalar(-Scalar(0.55659957866191134) * _tmp67 - 1), Scalar(2));
  const Scalar _tmp156 = _tmp140 * _tmp142;
  const Scalar _tmp157 = _tmp132 * _tmp134;
  const Scalar _tmp158 =
      _tmp122 * _tmp150 * _tmp76 + _tmp129 * _tmp146 * _tmp76 - _tmp156 * _tmp99 - _tmp157 * _tmp99;
  const Scalar _tmp159 = Scalar(1.0) / (_tmp158);
  const Scalar _tmp160 = _tmp117 * _tmp143;
  const Scalar _tmp161 = std::asinh(
      _tmp159 * (_tmp115 * _tmp122 * _tmp81 + _tmp127 * _tmp129 * _tmp81 + _tmp133 * _tmp134 +
                 _tmp136 * _tmp137 + _tmp141 * _tmp142 * _tmp81 - _tmp144 * _tmp160));
  const Scalar _tmp162 = Scalar(9.6622558468725703) * _tmp158;
  const Scalar _tmp163 =
      _tmp102 * _tmp109 * _tmp122 + _tmp102 * _tmp126 * _tmp129 + _tmp156 + _tmp157;
  const Scalar _tmp164 = Scalar(1.0) / (_tmp163);
  const Scalar _tmp165 =
      std::asinh(_tmp164 * (-_tmp108 * _tmp113 * _tmp122 - _tmp113 * _tmp125 * _tmp129 +
                            _tmp131 * _tmp134 - _tmp136 * _tmp138 - _tmp139 * _tmp142 + _tmp160));
  const Scalar _tmp166 = Scalar(9.6622558468725703) * _tmp163;
  const Scalar _tmp167 = Scalar(22.802596067096832) *
                             std::pow(Scalar(1 - Scalar(0.20941503221602112) * _tmp88), Scalar(2)) +
                         Scalar(7.3875128562042027) *
                             std::pow(Scalar(1 - Scalar(0.36791786395571047) * _tmp91), Scalar(2));

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) =
      _tmp46 * (-std::sinh(Scalar(1.0) * _tmp45) -
                std::sinh(Scalar(0.1034955) * _tmp44 * (-std::sqrt(_tmp43) - _tmp45 * _tmp46))) -
      Scalar(8.4718465799999993) *
          std::sqrt(Scalar(Scalar(0.013932974275675287) * _tmp43 +
                           std::pow(Scalar(-Scalar(0.11803802046660766) * _tmp23 -
                                           Scalar(0.11803802046660766) * p_init2 + 1),
                                    Scalar(2))));
  _res(1, 0) =
      _tmp154 *
          (-std::sinh(Scalar(1.0) * _tmp153) -
           std::sinh(Scalar(0.1034955) * _tmp152 * (-_tmp153 * _tmp154 - std::sqrt(_tmp53)))) -
      Scalar(8.4693136199999994) *
          std::sqrt(Scalar(Scalar(0.013941309530580858) * _tmp53 +
                           std::pow(Scalar(-Scalar(0.11807332268798426) * _tmp47 -
                                           Scalar(0.11807332268798426) * p_init2 + 1),
                                    Scalar(2))));
  _res(2, 0) =
      _tmp162 *
          (-std::sinh(Scalar(1.0) * _tmp161) -
           std::sinh(Scalar(0.1034955) * _tmp159 * (-std::sqrt(_tmp155) - _tmp161 * _tmp162))) -
      Scalar(8.3700199099999999) *
          std::sqrt(Scalar(Scalar(0.01427404356387209) * _tmp155 +
                           std::pow(Scalar(-Scalar(0.11947402882581673) * _tmp73 -
                                           Scalar(0.11947402882581673) * p_init2 + 1),
                                    Scalar(2))));
  _res(3, 0) =
      _tmp166 *
          (-std::sinh(Scalar(1.0) * _tmp165) -
           std::sinh(Scalar(0.1034955) * _tmp164 * (-_tmp165 * _tmp166 - std::sqrt(_tmp167)))) -
      Scalar(8.36416322) *
          std::sqrt(Scalar(Scalar(0.014294040284261563) * _tmp167 +
                           std::pow(Scalar(-Scalar(0.1195576860108189) * _tmp98 -
                                           Scalar(0.1195576860108189) * p_init2 + 1),
                                    Scalar(2))));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
