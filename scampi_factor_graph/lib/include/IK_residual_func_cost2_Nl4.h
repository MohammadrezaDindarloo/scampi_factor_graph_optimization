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
 * Symbolic function: IK_residual_func_cost2_Nl4
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost2Nl4(const Scalar fh1, const Scalar fv1,
                                                   const Scalar rx, const Scalar ry,
                                                   const Scalar rz, const Scalar p_init0,
                                                   const Scalar p_init1, const Scalar p_init2,
                                                   const Scalar rot_init_x, const Scalar rot_init_y,
                                                   const Scalar rot_init_z, const Scalar rot_init_w,
                                                   const Scalar epsilon) {
  // Total ops: 549

  // Unused inputs
  (void)epsilon;

  // Input arrays

  // Intermediate terms (169)
  const Scalar _tmp0 = std::sqrt(
      Scalar(std::pow(rx, Scalar(2)) + std::pow(ry, Scalar(2)) + std::pow(rz, Scalar(2))));
  const Scalar _tmp1 = (Scalar(1) / Scalar(2)) * _tmp0;
  const Scalar _tmp2 = std::cos(_tmp1);
  const Scalar _tmp3 = std::sin(_tmp1) / _tmp0;
  const Scalar _tmp4 = _tmp3 * rot_init_x;
  const Scalar _tmp5 = _tmp3 * rx;
  const Scalar _tmp6 = _tmp3 * rz;
  const Scalar _tmp7 = _tmp2 * rot_init_z + _tmp4 * ry - _tmp5 * rot_init_y + _tmp6 * rot_init_w;
  const Scalar _tmp8 = _tmp3 * ry;
  const Scalar _tmp9 = _tmp2 * rot_init_y - _tmp4 * rz + _tmp5 * rot_init_z + _tmp8 * rot_init_w;
  const Scalar _tmp10 = 2 * _tmp7 * _tmp9;
  const Scalar _tmp11 =
      _tmp2 * rot_init_x + _tmp5 * rot_init_w + _tmp6 * rot_init_y - _tmp8 * rot_init_z;
  const Scalar _tmp12 =
      2 * _tmp2 * rot_init_w - 2 * _tmp4 * rx - 2 * _tmp6 * rot_init_z - 2 * _tmp8 * rot_init_y;
  const Scalar _tmp13 = _tmp11 * _tmp12;
  const Scalar _tmp14 = Scalar(0.20999999999999999) * _tmp10 + Scalar(0.20999999999999999) * _tmp13;
  const Scalar _tmp15 = -_tmp14;
  const Scalar _tmp16 = -2 * std::pow(_tmp11, Scalar(2));
  const Scalar _tmp17 = 1 - 2 * std::pow(_tmp9, Scalar(2));
  const Scalar _tmp18 =
      -Scalar(0.010999999999999999) * _tmp16 - Scalar(0.010999999999999999) * _tmp17;
  const Scalar _tmp19 = 2 * _tmp11;
  const Scalar _tmp20 = _tmp19 * _tmp7;
  const Scalar _tmp21 = _tmp12 * _tmp9;
  const Scalar _tmp22 = Scalar(0.20999999999999999) * _tmp20 - Scalar(0.20999999999999999) * _tmp21;
  const Scalar _tmp23 = _tmp18 - _tmp22;
  const Scalar _tmp24 = _tmp15 + _tmp23;
  const Scalar _tmp25 = _tmp19 * _tmp9;
  const Scalar _tmp26 = _tmp12 * _tmp7;
  const Scalar _tmp27 = Scalar(0.20999999999999999) * _tmp25 + Scalar(0.20999999999999999) * _tmp26;
  const Scalar _tmp28 = -_tmp27;
  const Scalar _tmp29 = _tmp10 - _tmp13;
  const Scalar _tmp30 = -Scalar(0.010999999999999999) * _tmp29;
  const Scalar _tmp31 = -2 * std::pow(_tmp7, Scalar(2));
  const Scalar _tmp32 = Scalar(0.20999999999999999) * _tmp16 +
                        Scalar(0.20999999999999999) * _tmp31 + Scalar(0.20999999999999999);
  const Scalar _tmp33 = _tmp30 - _tmp32;
  const Scalar _tmp34 = _tmp28 + _tmp33;
  const Scalar _tmp35 = _tmp34 + p_init1;
  const Scalar _tmp36 = Scalar(0.20999999999999999) * _tmp25 - Scalar(0.20999999999999999) * _tmp26;
  const Scalar _tmp37 = -_tmp36;
  const Scalar _tmp38 = _tmp20 + _tmp21;
  const Scalar _tmp39 = -Scalar(0.010999999999999999) * _tmp38;
  const Scalar _tmp40 = Scalar(0.20999999999999999) * _tmp17 + Scalar(0.20999999999999999) * _tmp31;
  const Scalar _tmp41 = _tmp39 - _tmp40;
  const Scalar _tmp42 = _tmp37 + _tmp41;
  const Scalar _tmp43 = _tmp42 + p_init0;
  const Scalar _tmp44 = Scalar(69.216682114881593) *
                            std::pow(Scalar(-Scalar(0.12019727204189803) * _tmp35 - 1), Scalar(2)) +
                        Scalar(3.9500536956656402) *
                            std::pow(Scalar(-Scalar(0.50315118556004401) * _tmp43 - 1), Scalar(2));
  const Scalar _tmp45 = Scalar(1.0) / (fh1);
  const Scalar _tmp46 = std::asinh(_tmp45 * fv1);
  const Scalar _tmp47 = Scalar(9.6622558468725703) * fh1;
  const Scalar _tmp48 = _tmp14 + _tmp23;
  const Scalar _tmp49 = _tmp30 + _tmp32;
  const Scalar _tmp50 = _tmp28 + _tmp49;
  const Scalar _tmp51 = _tmp50 + p_init1;
  const Scalar _tmp52 = _tmp36 + _tmp41;
  const Scalar _tmp53 = _tmp52 + p_init0;
  const Scalar _tmp54 = Scalar(23.361089618893828) *
                            std::pow(Scalar(1 - Scalar(0.20689664689659551) * _tmp51), Scalar(2)) +
                        Scalar(3.2278567553341642) *
                            std::pow(Scalar(-Scalar(0.55659957866191134) * _tmp53 - 1), Scalar(2));
  const Scalar _tmp55 = _tmp39 + _tmp40;
  const Scalar _tmp56 = _tmp36 + _tmp55;
  const Scalar _tmp57 = _tmp27 + _tmp49;
  const Scalar _tmp58 = _tmp57 + p_init1;
  const Scalar _tmp59 = _tmp58 + Scalar(-4.7752063900000001);
  const Scalar _tmp60 = _tmp56 + p_init0;
  const Scalar _tmp61 = _tmp60 + Scalar(-2.71799795);
  const Scalar _tmp62 = std::pow(Scalar(std::pow(_tmp59, Scalar(2)) + std::pow(_tmp61, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp63 = _tmp59 * _tmp62;
  const Scalar _tmp64 = _tmp61 * _tmp62;
  const Scalar _tmp65 = _tmp51 + Scalar(-4.8333311099999996);
  const Scalar _tmp66 = _tmp53 + Scalar(1.79662371);
  const Scalar _tmp67 =
      std::sqrt(Scalar(std::pow(_tmp65, Scalar(2)) + std::pow(_tmp66, Scalar(2))));
  const Scalar _tmp68 = Scalar(1.0) / (_tmp67);
  const Scalar _tmp69 = Scalar(1.0) / (_tmp66);
  const Scalar _tmp70 = _tmp67 * _tmp69;
  const Scalar _tmp71 = _tmp70 * (-_tmp50 * _tmp66 * _tmp68 + _tmp52 * _tmp65 * _tmp68);
  const Scalar _tmp72 = _tmp37 + _tmp55;
  const Scalar _tmp73 = _tmp72 + p_init0;
  const Scalar _tmp74 = _tmp73 + Scalar(-2.5202214700000001);
  const Scalar _tmp75 = _tmp27 + _tmp33;
  const Scalar _tmp76 = _tmp75 + p_init1;
  const Scalar _tmp77 = _tmp76 + Scalar(8.3888750099999996);
  const Scalar _tmp78 = std::pow(Scalar(std::pow(_tmp74, Scalar(2)) + std::pow(_tmp77, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp79 = _tmp74 * _tmp78;
  const Scalar _tmp80 = _tmp77 * _tmp78;
  const Scalar _tmp81 = _tmp71 * _tmp79 - _tmp72 * _tmp80 + _tmp75 * _tmp79;
  const Scalar _tmp82 = _tmp65 * _tmp69;
  const Scalar _tmp83 = Scalar(1.0) / (_tmp79 * _tmp82 - _tmp80);
  const Scalar _tmp84 = -_tmp63 + _tmp64 * _tmp82;
  const Scalar _tmp85 = _tmp83 * _tmp84;
  const Scalar _tmp86 = -_tmp56 * _tmp63 + _tmp57 * _tmp64 + _tmp64 * _tmp71 - _tmp81 * _tmp85;
  const Scalar _tmp87 = Scalar(1.0) / (_tmp86);
  const Scalar _tmp88 = Scalar(1.0) * _tmp87;
  const Scalar _tmp89 = Scalar(1.0) * _tmp83;
  const Scalar _tmp90 = _tmp84 * _tmp87 * _tmp89;
  const Scalar _tmp91 = _tmp35 + Scalar(8.3196563700000006);
  const Scalar _tmp92 = _tmp43 + Scalar(1.9874742000000001);
  const Scalar _tmp93 = std::pow(Scalar(std::pow(_tmp91, Scalar(2)) + std::pow(_tmp92, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp94 = _tmp92 * _tmp93;
  const Scalar _tmp95 = _tmp91 * _tmp93;
  const Scalar _tmp96 = fh1 * (-_tmp34 * _tmp94 + _tmp42 * _tmp95);
  const Scalar _tmp97 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp98 = _tmp18 + _tmp22;
  const Scalar _tmp99 = _tmp15 + _tmp98;
  const Scalar _tmp100 = _tmp48 * _tmp82;
  const Scalar _tmp101 = -_tmp100 * _tmp79 + _tmp80 * _tmp99;
  const Scalar _tmp102 = _tmp14 + _tmp98;
  const Scalar _tmp103 = -_tmp100 * _tmp64 - _tmp101 * _tmp85 + _tmp102 * _tmp63;
  const Scalar _tmp104 = Scalar(1.0) * _tmp50;
  const Scalar _tmp105 = -_tmp104;
  const Scalar _tmp106 = Scalar(1.0) / (_tmp105 + _tmp75);
  const Scalar _tmp107 = Scalar(1.0) * _tmp52;
  const Scalar _tmp108 = _tmp107 - _tmp72;
  const Scalar _tmp109 = _tmp106 * _tmp108;
  const Scalar _tmp110 = _tmp48 * _tmp79 - _tmp79 * _tmp99;
  const Scalar _tmp111 = -_tmp102 * _tmp64 - _tmp103 * _tmp109 - _tmp110 * _tmp85 + _tmp48 * _tmp64;
  const Scalar _tmp112 = Scalar(1.0) / (_tmp111);
  const Scalar _tmp113 = _tmp104 * _tmp109 + _tmp107;
  const Scalar _tmp114 = 0;
  const Scalar _tmp115 = _tmp112 * _tmp114;
  const Scalar _tmp116 = _tmp79 * _tmp83;
  const Scalar _tmp117 = _tmp112 * _tmp84;
  const Scalar _tmp118 = _tmp114 * _tmp117;
  const Scalar _tmp119 = _tmp82 * _tmp83;
  const Scalar _tmp120 = _tmp100 + _tmp101 * _tmp119;
  const Scalar _tmp121 = -_tmp109 * _tmp120 + _tmp110 * _tmp119 - _tmp48;
  const Scalar _tmp122 = _tmp112 * _tmp86;
  const Scalar _tmp123 = _tmp111 * _tmp87;
  const Scalar _tmp124 = _tmp123 * (_tmp119 * _tmp81 - _tmp121 * _tmp122 - _tmp71);
  const Scalar _tmp125 = _tmp121 + _tmp124;
  const Scalar _tmp126 = _tmp112 * _tmp64;
  const Scalar _tmp127 = -_tmp117 * _tmp125 - _tmp82;
  const Scalar _tmp128 = _tmp94 * fh1;
  const Scalar _tmp129 = _tmp101 * _tmp89;
  const Scalar _tmp130 = _tmp109 * _tmp129 - _tmp110 * _tmp89;
  const Scalar _tmp131 = _tmp123 * (-_tmp122 * _tmp130 - _tmp81 * _tmp89);
  const Scalar _tmp132 = _tmp130 + _tmp131;
  const Scalar _tmp133 = -_tmp117 * _tmp132 + Scalar(1.0);
  const Scalar _tmp134 = _tmp95 * fh1;
  const Scalar _tmp135 = -_tmp128 * _tmp70 * (_tmp116 * _tmp127 + _tmp125 * _tmp126 + Scalar(1.0)) -
                         _tmp134 * _tmp70 * (_tmp116 * _tmp133 + _tmp126 * _tmp132) -
                         _tmp70 * _tmp96 * (_tmp64 * _tmp88 - _tmp79 * _tmp90) -
                         _tmp70 * _tmp97 * (_tmp115 * _tmp64 - _tmp116 * _tmp118);
  const Scalar _tmp136 = Scalar(1.0) / (_tmp135);
  const Scalar _tmp137 = _tmp24 * fh1;
  const Scalar _tmp138 = -_tmp137 * _tmp95 - Scalar(3.29616) * _tmp29 - _tmp34 * fv1;
  const Scalar _tmp139 = _tmp105 + _tmp57;
  const Scalar _tmp140 = _tmp109 * _tmp139;
  const Scalar _tmp141 = Scalar(1.0) / (_tmp107 - _tmp140 - _tmp56);
  const Scalar _tmp142 = Scalar(1.0) * _tmp141;
  const Scalar _tmp143 = _tmp140 * _tmp142 + Scalar(1.0);
  const Scalar _tmp144 = Scalar(1.0) * _tmp106;
  const Scalar _tmp145 = _tmp113 * _tmp141;
  const Scalar _tmp146 = -_tmp103 * _tmp115 + _tmp105 - _tmp139 * _tmp145;
  const Scalar _tmp147 = _tmp139 * _tmp141;
  const Scalar _tmp148 = _tmp103 * _tmp112;
  const Scalar _tmp149 = -_tmp129 + _tmp131 * _tmp147 - _tmp132 * _tmp148;
  const Scalar _tmp150 = _tmp137 * _tmp94 + Scalar(3.29616) * _tmp38 + _tmp42 * fv1;
  const Scalar _tmp151 = _tmp139 * _tmp142;
  const Scalar _tmp152 = _tmp120 + _tmp124 * _tmp147 - _tmp125 * _tmp148;
  const Scalar _tmp153 = _tmp123 * _tmp142;
  const Scalar _tmp154 = -_tmp103 * _tmp88 + _tmp123 * _tmp151;
  const Scalar _tmp155 = std::asinh(
      _tmp136 * (Scalar(1.0) * _tmp128 * (_tmp124 * _tmp142 - _tmp144 * _tmp152) +
                 Scalar(1.0) * _tmp134 * (_tmp131 * _tmp142 - _tmp144 * _tmp149) +
                 Scalar(1.0) * _tmp138 * (_tmp109 * _tmp142 - _tmp143 * _tmp144) +
                 Scalar(1.0) * _tmp150 * (_tmp106 * _tmp151 - _tmp142) +
                 Scalar(1.0) * _tmp96 * (-_tmp144 * _tmp154 + _tmp153) +
                 Scalar(1.0) * _tmp97 * (-_tmp113 * _tmp142 - _tmp144 * _tmp146 + Scalar(1.0))));
  const Scalar _tmp156 = Scalar(9.6622558468725703) * _tmp135;
  const Scalar _tmp157 = -_tmp118 * _tmp83 * _tmp97 + _tmp127 * _tmp128 * _tmp83 +
                         _tmp133 * _tmp134 * _tmp83 - _tmp90 * _tmp96;
  const Scalar _tmp158 = Scalar(1.0) / (_tmp157);
  const Scalar _tmp159 = _tmp106 * _tmp138;
  const Scalar _tmp160 = _tmp142 * _tmp150;
  const Scalar _tmp161 =
      std::asinh(_tmp158 * (_tmp106 * _tmp128 * _tmp152 + _tmp106 * _tmp134 * _tmp149 -
                            _tmp106 * _tmp139 * _tmp160 + _tmp106 * _tmp146 * _tmp97 +
                            _tmp106 * _tmp154 * _tmp96 + _tmp143 * _tmp159));
  const Scalar _tmp162 = Scalar(9.6622558468725703) * _tmp157;
  const Scalar _tmp163 = Scalar(6.351516257848961) *
                             std::pow(Scalar(1 - Scalar(0.39679052492160538) * _tmp73), Scalar(2)) +
                         Scalar(70.3732239334025) *
                             std::pow(Scalar(-Scalar(0.11920549523123722) * _tmp76 - 1), Scalar(2));
  const Scalar _tmp164 = Scalar(22.802596067096832) *
                             std::pow(Scalar(1 - Scalar(0.20941503221602112) * _tmp58), Scalar(2)) +
                         Scalar(7.3875128562042027) *
                             std::pow(Scalar(1 - Scalar(0.36791786395571047) * _tmp60), Scalar(2));
  const Scalar _tmp165 = _tmp112 * _tmp125 * _tmp128 + _tmp112 * _tmp132 * _tmp134 +
                         _tmp115 * _tmp97 + _tmp88 * _tmp96;
  const Scalar _tmp166 = Scalar(1.0) / (_tmp165);
  const Scalar _tmp167 = std::asinh(
      _tmp166 * (-_tmp108 * _tmp142 * _tmp159 - _tmp124 * _tmp128 * _tmp141 -
                 _tmp131 * _tmp134 * _tmp141 + _tmp145 * _tmp97 - _tmp153 * _tmp96 + _tmp160));
  const Scalar _tmp168 = Scalar(9.6622558468725703) * _tmp165;

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) =
      _tmp47 * (-std::sinh(Scalar(1.0) * _tmp46) -
                std::sinh(Scalar(0.1034955) * _tmp45 * (-std::sqrt(_tmp44) - _tmp46 * _tmp47))) -
      Scalar(8.4718465799999993) *
          std::sqrt(Scalar(Scalar(0.013932974275675287) * _tmp44 +
                           std::pow(Scalar(-Scalar(0.11803802046660766) * _tmp24 -
                                           Scalar(0.11803802046660766) * p_init2 + 1),
                                    Scalar(2))));
  _res(1, 0) =
      _tmp156 *
          (-std::sinh(Scalar(1.0) * _tmp155) -
           std::sinh(Scalar(0.1034955) * _tmp136 * (-_tmp155 * _tmp156 - std::sqrt(_tmp54)))) -
      Scalar(8.3700199099999999) *
          std::sqrt(Scalar(Scalar(0.01427404356387209) * _tmp54 +
                           std::pow(Scalar(-Scalar(0.11947402882581673) * _tmp48 -
                                           Scalar(0.11947402882581673) * p_init2 + 1),
                                    Scalar(2))));
  _res(2, 0) =
      _tmp162 *
          (-std::sinh(Scalar(1.0) * _tmp161) -
           std::sinh(Scalar(0.1034955) * _tmp158 * (-_tmp161 * _tmp162 - std::sqrt(_tmp163)))) -
      Scalar(8.4693136199999994) *
          std::sqrt(Scalar(Scalar(0.013941309530580858) * _tmp163 +
                           std::pow(Scalar(-Scalar(0.11807332268798426) * _tmp99 -
                                           Scalar(0.11807332268798426) * p_init2 + 1),
                                    Scalar(2))));
  _res(3, 0) =
      _tmp168 *
          (-std::sinh(Scalar(1.0) * _tmp167) -
           std::sinh(Scalar(0.1034955) * _tmp166 * (-std::sqrt(_tmp164) - _tmp167 * _tmp168))) -
      Scalar(8.36416322) *
          std::sqrt(Scalar(Scalar(0.014294040284261563) * _tmp164 +
                           std::pow(Scalar(-Scalar(0.1195576860108189) * _tmp102 -
                                           Scalar(0.1195576860108189) * p_init2 + 1),
                                    Scalar(2))));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
