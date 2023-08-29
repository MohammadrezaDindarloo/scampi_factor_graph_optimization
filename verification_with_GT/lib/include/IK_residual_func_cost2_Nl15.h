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
 * Symbolic function: IK_residual_func_cost2_Nl15
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost2Nl15(
    const Scalar fh1, const Scalar fv1, const Scalar rx, const Scalar ry, const Scalar rz,
    const Scalar p_init0, const Scalar p_init1, const Scalar p_init2, const Scalar rot_init_x,
    const Scalar rot_init_y, const Scalar rot_init_z, const Scalar rot_init_w,
    const Scalar epsilon) {
  // Total ops: 543

  // Unused inputs
  (void)epsilon;

  // Input arrays

  // Intermediate terms (166)
  const Scalar _tmp0 = Scalar(1.0) / (fh1);
  const Scalar _tmp1 = std::asinh(_tmp0 * fv1);
  const Scalar _tmp2 = Scalar(9.6622558468725703) * fh1;
  const Scalar _tmp3 = std::sqrt(
      Scalar(std::pow(rx, Scalar(2)) + std::pow(ry, Scalar(2)) + std::pow(rz, Scalar(2))));
  const Scalar _tmp4 = (Scalar(1) / Scalar(2)) * _tmp3;
  const Scalar _tmp5 = std::cos(_tmp4);
  const Scalar _tmp6 = std::sin(_tmp4) / _tmp3;
  const Scalar _tmp7 = _tmp6 * ry;
  const Scalar _tmp8 = _tmp6 * rx;
  const Scalar _tmp9 = _tmp6 * rz;
  const Scalar _tmp10 =
      _tmp5 * rot_init_x - _tmp7 * rot_init_z + _tmp8 * rot_init_w + _tmp9 * rot_init_y;
  const Scalar _tmp11 =
      _tmp5 * rot_init_y + _tmp7 * rot_init_w + _tmp8 * rot_init_z - _tmp9 * rot_init_x;
  const Scalar _tmp12 = 2 * _tmp11;
  const Scalar _tmp13 = _tmp10 * _tmp12;
  const Scalar _tmp14 =
      _tmp5 * rot_init_z + _tmp7 * rot_init_x - _tmp8 * rot_init_y + _tmp9 * rot_init_w;
  const Scalar _tmp15 =
      _tmp5 * rot_init_w - _tmp7 * rot_init_y - _tmp8 * rot_init_x - _tmp9 * rot_init_z;
  const Scalar _tmp16 = 2 * _tmp15;
  const Scalar _tmp17 = _tmp14 * _tmp16;
  const Scalar _tmp18 = Scalar(0.20999999999999999) * _tmp13 + Scalar(0.20999999999999999) * _tmp17;
  const Scalar _tmp19 = -2 * std::pow(_tmp10, Scalar(2));
  const Scalar _tmp20 = 1 - 2 * std::pow(_tmp14, Scalar(2));
  const Scalar _tmp21 = Scalar(0.20999999999999999) * _tmp19 + Scalar(0.20999999999999999) * _tmp20;
  const Scalar _tmp22 = _tmp12 * _tmp14;
  const Scalar _tmp23 = _tmp10 * _tmp16;
  const Scalar _tmp24 = _tmp22 - _tmp23;
  const Scalar _tmp25 = -Scalar(0.010999999999999999) * _tmp24;
  const Scalar _tmp26 = _tmp21 + _tmp25;
  const Scalar _tmp27 = _tmp18 + _tmp26;
  const Scalar _tmp28 = _tmp27 + p_init1;
  const Scalar _tmp29 = Scalar(0.20999999999999999) * _tmp13 - Scalar(0.20999999999999999) * _tmp17;
  const Scalar _tmp30 = -2 * std::pow(_tmp11, Scalar(2));
  const Scalar _tmp31 = Scalar(0.20999999999999999) * _tmp20 + Scalar(0.20999999999999999) * _tmp30;
  const Scalar _tmp32 = 2 * _tmp10 * _tmp14;
  const Scalar _tmp33 = _tmp12 * _tmp15;
  const Scalar _tmp34 = _tmp32 + _tmp33;
  const Scalar _tmp35 = -Scalar(0.010999999999999999) * _tmp34;
  const Scalar _tmp36 = _tmp31 + _tmp35;
  const Scalar _tmp37 = _tmp29 + _tmp36;
  const Scalar _tmp38 = _tmp37 + p_init0;
  const Scalar _tmp39 = Scalar(22.802596067096832) *
                            std::pow(Scalar(1 - Scalar(0.20941503221602112) * _tmp28), Scalar(2)) +
                        Scalar(7.3875128562042027) *
                            std::pow(Scalar(1 - Scalar(0.36791786395571047) * _tmp38), Scalar(2));
  const Scalar _tmp40 = Scalar(0.20999999999999999) * _tmp32 - Scalar(0.20999999999999999) * _tmp33;
  const Scalar _tmp41 = -Scalar(0.010999999999999999) * _tmp19 -
                        Scalar(0.010999999999999999) * _tmp30 + Scalar(-0.010999999999999999);
  const Scalar _tmp42 = Scalar(0.20999999999999999) * _tmp22 + Scalar(0.20999999999999999) * _tmp23;
  const Scalar _tmp43 = _tmp41 + _tmp42;
  const Scalar _tmp44 = _tmp40 + _tmp43;
  const Scalar _tmp45 = -_tmp21 + _tmp25;
  const Scalar _tmp46 = _tmp18 + _tmp45;
  const Scalar _tmp47 = _tmp46 + p_init1;
  const Scalar _tmp48 = -_tmp29;
  const Scalar _tmp49 = _tmp36 + _tmp48;
  const Scalar _tmp50 = _tmp49 + p_init0;
  const Scalar _tmp51 = Scalar(6.351516257848961) *
                            std::pow(Scalar(1 - Scalar(0.39679052492160538) * _tmp50), Scalar(2)) +
                        Scalar(70.3732239334025) *
                            std::pow(Scalar(-Scalar(0.11920549523123722) * _tmp47 - 1), Scalar(2));
  const Scalar _tmp52 = _tmp50 + Scalar(-2.5202214700000001);
  const Scalar _tmp53 = Scalar(1.0) / (_tmp52);
  const Scalar _tmp54 = _tmp47 + Scalar(8.3888750099999996);
  const Scalar _tmp55 = _tmp53 * _tmp54;
  const Scalar _tmp56 = -_tmp18;
  const Scalar _tmp57 = _tmp26 + _tmp56;
  const Scalar _tmp58 = _tmp57 + p_init1;
  const Scalar _tmp59 = _tmp58 + Scalar(-4.8333311099999996);
  const Scalar _tmp60 = -_tmp31 + _tmp35;
  const Scalar _tmp61 = _tmp29 + _tmp60;
  const Scalar _tmp62 = _tmp61 + p_init0;
  const Scalar _tmp63 = _tmp62 + Scalar(1.79662371);
  const Scalar _tmp64 = std::pow(Scalar(std::pow(_tmp59, Scalar(2)) + std::pow(_tmp63, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp65 = _tmp63 * _tmp64;
  const Scalar _tmp66 = _tmp59 * _tmp64;
  const Scalar _tmp67 = Scalar(1.0) / (_tmp55 * _tmp65 - _tmp66);
  const Scalar _tmp68 = _tmp41 - _tmp42;
  const Scalar _tmp69 = _tmp40 + _tmp68;
  const Scalar _tmp70 = _tmp55 * _tmp69;
  const Scalar _tmp71 = -_tmp40;
  const Scalar _tmp72 = _tmp43 + _tmp71;
  const Scalar _tmp73 = _tmp67 * (-_tmp65 * _tmp70 + _tmp66 * _tmp72);
  const Scalar _tmp74 = Scalar(1.0) * _tmp73;
  const Scalar _tmp75 = Scalar(1.0) * _tmp46;
  const Scalar _tmp76 = -_tmp75;
  const Scalar _tmp77 = Scalar(1.0) / (_tmp57 + _tmp76);
  const Scalar _tmp78 = Scalar(1.0) * _tmp49;
  const Scalar _tmp79 = _tmp77 * (-_tmp61 + _tmp78);
  const Scalar _tmp80 = _tmp67 * (_tmp65 * _tmp69 - _tmp65 * _tmp72);
  const Scalar _tmp81 = _tmp74 * _tmp79 - Scalar(1.0) * _tmp80;
  const Scalar _tmp82 = _tmp45 + _tmp56;
  const Scalar _tmp83 = _tmp82 + p_init1;
  const Scalar _tmp84 = _tmp83 + Scalar(8.3196563700000006);
  const Scalar _tmp85 = _tmp48 + _tmp60;
  const Scalar _tmp86 = _tmp85 + p_init0;
  const Scalar _tmp87 = _tmp86 + Scalar(1.9874742000000001);
  const Scalar _tmp88 = std::pow(Scalar(std::pow(_tmp84, Scalar(2)) + std::pow(_tmp87, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp89 = _tmp87 * _tmp88;
  const Scalar _tmp90 = _tmp84 * _tmp88;
  const Scalar _tmp91 = _tmp55 * _tmp89 - _tmp90;
  const Scalar _tmp92 = _tmp68 + _tmp71;
  const Scalar _tmp93 = _tmp69 * _tmp89;
  const Scalar _tmp94 = -_tmp55 * _tmp93 - _tmp73 * _tmp91 + _tmp90 * _tmp92;
  const Scalar _tmp95 = -_tmp79 * _tmp94 - _tmp80 * _tmp91 - _tmp89 * _tmp92 + _tmp93;
  const Scalar _tmp96 = Scalar(1.0) / (_tmp95);
  const Scalar _tmp97 =
      std::sqrt(Scalar(std::pow(_tmp52, Scalar(2)) + std::pow(_tmp54, Scalar(2))));
  const Scalar _tmp98 = Scalar(1.0) / (_tmp97);
  const Scalar _tmp99 = _tmp53 * _tmp97;
  const Scalar _tmp100 = _tmp99 * (-_tmp46 * _tmp52 * _tmp98 + _tmp49 * _tmp54 * _tmp98);
  const Scalar _tmp101 = _tmp67 * (_tmp100 * _tmp65 + _tmp57 * _tmp65 - _tmp61 * _tmp66);
  const Scalar _tmp102 = _tmp100 * _tmp89 - _tmp101 * _tmp91 + _tmp82 * _tmp89 - _tmp85 * _tmp90;
  const Scalar _tmp103 = _tmp102 * _tmp96;
  const Scalar _tmp104 = Scalar(1.0) / (_tmp102);
  const Scalar _tmp105 = _tmp104 * _tmp95;
  const Scalar _tmp106 = _tmp105 * (-Scalar(1.0) * _tmp101 - _tmp103 * _tmp81);
  const Scalar _tmp107 = _tmp106 + _tmp81;
  const Scalar _tmp108 = _tmp89 * _tmp96;
  const Scalar _tmp109 = _tmp91 * _tmp96;
  const Scalar _tmp110 = -_tmp107 * _tmp109 + Scalar(1.0);
  const Scalar _tmp111 = _tmp65 * _tmp67;
  const Scalar _tmp112 = _tmp28 + Scalar(-4.7752063900000001);
  const Scalar _tmp113 = _tmp38 + Scalar(-2.71799795);
  const Scalar _tmp114 =
      std::pow(Scalar(std::pow(_tmp112, Scalar(2)) + std::pow(_tmp113, Scalar(2))),
               Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp115 = _tmp112 * _tmp114;
  const Scalar _tmp116 = _tmp115 * fh1;
  const Scalar _tmp117 = Scalar(1.0) * _tmp104;
  const Scalar _tmp118 = _tmp113 * _tmp114;
  const Scalar _tmp119 = fh1 * (_tmp115 * _tmp37 - _tmp118 * _tmp27);
  const Scalar _tmp120 = _tmp55 * _tmp73 + _tmp70;
  const Scalar _tmp121 = -_tmp120 * _tmp79 + _tmp55 * _tmp80 - _tmp69;
  const Scalar _tmp122 = _tmp105 * (-_tmp100 + _tmp101 * _tmp55 - _tmp103 * _tmp121);
  const Scalar _tmp123 = _tmp121 + _tmp122;
  const Scalar _tmp124 = -_tmp109 * _tmp123 - _tmp55;
  const Scalar _tmp125 = _tmp118 * fh1;
  const Scalar _tmp126 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp127 = _tmp75 * _tmp79 + _tmp78;
  const Scalar _tmp128 = 0;
  const Scalar _tmp129 = _tmp109 * _tmp128;
  const Scalar _tmp130 = _tmp128 * _tmp96;
  const Scalar _tmp131 = -_tmp116 * _tmp99 * (_tmp107 * _tmp108 + _tmp110 * _tmp111) -
                         _tmp119 * _tmp99 * (-_tmp111 * _tmp117 * _tmp91 + _tmp117 * _tmp89) -
                         _tmp125 * _tmp99 * (_tmp108 * _tmp123 + _tmp111 * _tmp124 + Scalar(1.0)) -
                         _tmp126 * _tmp99 * (-_tmp111 * _tmp129 + _tmp130 * _tmp89);
  const Scalar _tmp132 = Scalar(1.0) / (_tmp131);
  const Scalar _tmp133 = _tmp76 + _tmp82;
  const Scalar _tmp134 = _tmp133 * _tmp79;
  const Scalar _tmp135 = Scalar(1.0) / (-_tmp134 + _tmp78 - _tmp85);
  const Scalar _tmp136 = _tmp133 * _tmp135;
  const Scalar _tmp137 = _tmp94 * _tmp96;
  const Scalar _tmp138 = _tmp106 * _tmp136 - _tmp107 * _tmp137 - _tmp74;
  const Scalar _tmp139 = Scalar(1.0) * _tmp77;
  const Scalar _tmp140 = Scalar(1.0) * _tmp135;
  const Scalar _tmp141 = _tmp44 * fh1;
  const Scalar _tmp142 = _tmp118 * _tmp141 + Scalar(3.29616) * _tmp34 + _tmp37 * fv1;
  const Scalar _tmp143 = _tmp133 * _tmp77;
  const Scalar _tmp144 = -_tmp115 * _tmp141 - Scalar(3.29616) * _tmp24 - _tmp27 * fv1;
  const Scalar _tmp145 = _tmp134 * _tmp140 + Scalar(1.0);
  const Scalar _tmp146 = _tmp140 * _tmp79;
  const Scalar _tmp147 = _tmp127 * _tmp135;
  const Scalar _tmp148 = _tmp77 * (-_tmp128 * _tmp137 - _tmp133 * _tmp147 + _tmp76);
  const Scalar _tmp149 = _tmp120 + _tmp122 * _tmp136 - _tmp123 * _tmp137;
  const Scalar _tmp150 = _tmp105 * _tmp140;
  const Scalar _tmp151 = -_tmp117 * _tmp94 + _tmp133 * _tmp150;
  const Scalar _tmp152 =
      std::asinh(_tmp132 * (Scalar(1.0) * _tmp116 * (_tmp106 * _tmp140 - _tmp138 * _tmp139) +
                            Scalar(1.0) * _tmp119 * (-_tmp139 * _tmp151 + _tmp150) +
                            Scalar(1.0) * _tmp125 * (_tmp122 * _tmp140 - _tmp139 * _tmp149) +
                            Scalar(1.0) * _tmp126 *
                                (-_tmp127 * _tmp140 - Scalar(1.0) * _tmp148 + Scalar(1.0)) +
                            Scalar(1.0) * _tmp142 * (_tmp140 * _tmp143 - _tmp140) +
                            Scalar(1.0) * _tmp144 * (-_tmp139 * _tmp145 + _tmp146)));
  const Scalar _tmp153 = Scalar(9.6622558468725703) * _tmp131;
  const Scalar _tmp154 = Scalar(23.361089618893828) *
                             std::pow(Scalar(1 - Scalar(0.20689664689659551) * _tmp58), Scalar(2)) +
                         Scalar(3.2278567553341642) *
                             std::pow(Scalar(-Scalar(0.55659957866191134) * _tmp62 - 1), Scalar(2));
  const Scalar _tmp155 = _tmp140 * _tmp142;
  const Scalar _tmp156 = _tmp117 * _tmp119;
  const Scalar _tmp157 = _tmp110 * _tmp116 * _tmp67 + _tmp124 * _tmp125 * _tmp67 -
                         _tmp126 * _tmp129 * _tmp67 - _tmp156 * _tmp67 * _tmp91;
  const Scalar _tmp158 = Scalar(1.0) / (_tmp157);
  const Scalar _tmp159 =
      std::asinh(_tmp158 * (_tmp116 * _tmp138 * _tmp77 + _tmp119 * _tmp151 * _tmp77 +
                            _tmp125 * _tmp149 * _tmp77 + _tmp126 * _tmp148 - _tmp143 * _tmp155 +
                            _tmp144 * _tmp145 * _tmp77));
  const Scalar _tmp160 = Scalar(9.6622558468725703) * _tmp157;
  const Scalar _tmp161 =
      Scalar(69.216682114881593) *
          std::pow(Scalar(-Scalar(0.12019727204189803) * _tmp83 - 1), Scalar(2)) +
      Scalar(3.9500536956656402) *
          std::pow(Scalar(-Scalar(0.50315118556004401) * _tmp86 - 1), Scalar(2));
  const Scalar _tmp162 =
      _tmp107 * _tmp116 * _tmp96 + _tmp123 * _tmp125 * _tmp96 + _tmp126 * _tmp130 + _tmp156;
  const Scalar _tmp163 = Scalar(1.0) / (_tmp162);
  const Scalar _tmp164 = std::asinh(_tmp163 * (-_tmp106 * _tmp116 * _tmp135 - _tmp119 * _tmp150 -
                                               _tmp122 * _tmp125 * _tmp135 + _tmp126 * _tmp147 -
                                               _tmp144 * _tmp146 + _tmp155));
  const Scalar _tmp165 = Scalar(9.6622558468725703) * _tmp162;

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) =
      _tmp2 * (-std::sinh(Scalar(1.0) * _tmp1) -
               std::sinh(Scalar(0.1034955) * _tmp0 * (-_tmp1 * _tmp2 - std::sqrt(_tmp39)))) -
      Scalar(8.36416322) *
          std::sqrt(Scalar(Scalar(0.014294040284261563) * _tmp39 +
                           std::pow(Scalar(-Scalar(0.1195576860108189) * _tmp44 -
                                           Scalar(0.1195576860108189) * p_init2 + 1),
                                    Scalar(2))));
  _res(1, 0) =
      _tmp153 *
          (-std::sinh(Scalar(1.0) * _tmp152) -
           std::sinh(Scalar(0.1034955) * _tmp132 * (-_tmp152 * _tmp153 - std::sqrt(_tmp51)))) -
      Scalar(8.4693136199999994) *
          std::sqrt(Scalar(Scalar(0.013941309530580858) * _tmp51 +
                           std::pow(Scalar(-Scalar(0.11807332268798426) * _tmp69 -
                                           Scalar(0.11807332268798426) * p_init2 + 1),
                                    Scalar(2))));
  _res(2, 0) =
      _tmp160 *
          (-std::sinh(Scalar(1.0) * _tmp159) -
           std::sinh(Scalar(0.1034955) * _tmp158 * (-std::sqrt(_tmp154) - _tmp159 * _tmp160))) -
      Scalar(8.3700199099999999) *
          std::sqrt(Scalar(Scalar(0.01427404356387209) * _tmp154 +
                           std::pow(Scalar(-Scalar(0.11947402882581673) * _tmp72 -
                                           Scalar(0.11947402882581673) * p_init2 + 1),
                                    Scalar(2))));
  _res(3, 0) =
      _tmp165 *
          (-std::sinh(Scalar(1.0) * _tmp164) -
           std::sinh(Scalar(0.1034955) * _tmp163 * (-std::sqrt(_tmp161) - _tmp164 * _tmp165))) -
      Scalar(8.4718465799999993) *
          std::sqrt(Scalar(Scalar(0.013932974275675287) * _tmp161 +
                           std::pow(Scalar(-Scalar(0.11803802046660766) * _tmp92 -
                                           Scalar(0.11803802046660766) * p_init2 + 1),
                                    Scalar(2))));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym