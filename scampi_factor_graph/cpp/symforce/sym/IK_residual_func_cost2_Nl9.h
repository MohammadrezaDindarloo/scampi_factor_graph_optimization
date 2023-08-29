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
 * Symbolic function: IK_residual_func_cost2_Nl9
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost2Nl9(const Scalar fh1, const Scalar fv1,
                                                   const Scalar rx, const Scalar ry,
                                                   const Scalar rz, const Scalar p_init0,
                                                   const Scalar p_init1, const Scalar p_init2,
                                                   const Scalar rot_init_x, const Scalar rot_init_y,
                                                   const Scalar rot_init_z, const Scalar rot_init_w,
                                                   const Scalar epsilon) {
  // Total ops: 542

  // Unused inputs
  (void)epsilon;

  // Input arrays

  // Intermediate terms (164)
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
      _tmp2 * rot_init_y + _tmp4 * rot_init_w + _tmp5 * rot_init_z - _tmp6 * rot_init_x;
  const Scalar _tmp9 = 2 * _tmp8;
  const Scalar _tmp10 = _tmp7 * _tmp9;
  const Scalar _tmp11 =
      _tmp2 * rot_init_x - _tmp4 * rot_init_z + _tmp5 * rot_init_w + _tmp6 * rot_init_y;
  const Scalar _tmp12 = 2 * _tmp2 * rot_init_w - 2 * _tmp4 * rot_init_y - 2 * _tmp5 * rot_init_x -
                        2 * _tmp6 * rot_init_z;
  const Scalar _tmp13 = _tmp11 * _tmp12;
  const Scalar _tmp14 = Scalar(0.20999999999999999) * _tmp10 + Scalar(0.20999999999999999) * _tmp13;
  const Scalar _tmp15 = -_tmp14;
  const Scalar _tmp16 = -2 * std::pow(_tmp8, Scalar(2));
  const Scalar _tmp17 = 1 - 2 * std::pow(_tmp11, Scalar(2));
  const Scalar _tmp18 =
      -Scalar(0.010999999999999999) * _tmp16 - Scalar(0.010999999999999999) * _tmp17;
  const Scalar _tmp19 = 2 * _tmp11 * _tmp7;
  const Scalar _tmp20 = _tmp12 * _tmp8;
  const Scalar _tmp21 = Scalar(0.20999999999999999) * _tmp19 - Scalar(0.20999999999999999) * _tmp20;
  const Scalar _tmp22 = _tmp18 + _tmp21;
  const Scalar _tmp23 = _tmp15 + _tmp22;
  const Scalar _tmp24 = -2 * std::pow(_tmp7, Scalar(2));
  const Scalar _tmp25 = Scalar(0.20999999999999999) * _tmp17 + Scalar(0.20999999999999999) * _tmp24;
  const Scalar _tmp26 = -_tmp25;
  const Scalar _tmp27 = _tmp10 - _tmp13;
  const Scalar _tmp28 = -Scalar(0.010999999999999999) * _tmp27;
  const Scalar _tmp29 = _tmp11 * _tmp9;
  const Scalar _tmp30 = _tmp12 * _tmp7;
  const Scalar _tmp31 = Scalar(0.20999999999999999) * _tmp29 + Scalar(0.20999999999999999) * _tmp30;
  const Scalar _tmp32 = _tmp28 + _tmp31;
  const Scalar _tmp33 = _tmp26 + _tmp32;
  const Scalar _tmp34 = _tmp33 + p_init1;
  const Scalar _tmp35 = Scalar(0.20999999999999999) * _tmp16 +
                        Scalar(0.20999999999999999) * _tmp24 + Scalar(0.20999999999999999);
  const Scalar _tmp36 = _tmp19 + _tmp20;
  const Scalar _tmp37 = -Scalar(0.010999999999999999) * _tmp36;
  const Scalar _tmp38 = Scalar(0.20999999999999999) * _tmp29 - Scalar(0.20999999999999999) * _tmp30;
  const Scalar _tmp39 = _tmp37 - _tmp38;
  const Scalar _tmp40 = _tmp35 + _tmp39;
  const Scalar _tmp41 = _tmp40 + p_init0;
  const Scalar _tmp42 = Scalar(6.351516257848961) *
                            std::pow(Scalar(1 - Scalar(0.39679052492160538) * _tmp41), Scalar(2)) +
                        Scalar(70.3732239334025) *
                            std::pow(Scalar(-Scalar(0.11920549523123722) * _tmp34 - 1), Scalar(2));
  const Scalar _tmp43 = Scalar(1.0) / (fh1);
  const Scalar _tmp44 = std::asinh(_tmp43 * fv1);
  const Scalar _tmp45 = Scalar(9.6622558468725703) * fh1;
  const Scalar _tmp46 = _tmp25 + _tmp32;
  const Scalar _tmp47 = _tmp46 + p_init1;
  const Scalar _tmp48 = _tmp37 + _tmp38;
  const Scalar _tmp49 = _tmp35 + _tmp48;
  const Scalar _tmp50 = _tmp49 + p_init0;
  const Scalar _tmp51 = Scalar(22.802596067096832) *
                            std::pow(Scalar(1 - Scalar(0.20941503221602112) * _tmp47), Scalar(2)) +
                        Scalar(7.3875128562042027) *
                            std::pow(Scalar(1 - Scalar(0.36791786395571047) * _tmp50), Scalar(2));
  const Scalar _tmp52 = Scalar(1.0) * _tmp46;
  const Scalar _tmp53 = -_tmp52;
  const Scalar _tmp54 = _tmp28 - _tmp31;
  const Scalar _tmp55 = _tmp26 + _tmp54;
  const Scalar _tmp56 = _tmp53 + _tmp55;
  const Scalar _tmp57 = _tmp25 + _tmp54;
  const Scalar _tmp58 = Scalar(1.0) / (_tmp53 + _tmp57);
  const Scalar _tmp59 = -_tmp35;
  const Scalar _tmp60 = _tmp48 + _tmp59;
  const Scalar _tmp61 = Scalar(1.0) * _tmp49;
  const Scalar _tmp62 = _tmp58 * (-_tmp60 + _tmp61);
  const Scalar _tmp63 = _tmp56 * _tmp62;
  const Scalar _tmp64 = _tmp39 + _tmp59;
  const Scalar _tmp65 = Scalar(1.0) / (_tmp61 - _tmp63 - _tmp64);
  const Scalar _tmp66 = Scalar(1.0) * _tmp65;
  const Scalar _tmp67 = _tmp57 + p_init1;
  const Scalar _tmp68 = _tmp67 + Scalar(-4.8333311099999996);
  const Scalar _tmp69 = _tmp60 + p_init0;
  const Scalar _tmp70 = _tmp69 + Scalar(1.79662371);
  const Scalar _tmp71 = std::pow(Scalar(std::pow(_tmp68, Scalar(2)) + std::pow(_tmp70, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp72 = _tmp70 * _tmp71;
  const Scalar _tmp73 = _tmp47 + Scalar(-4.7752063900000001);
  const Scalar _tmp74 = _tmp50 + Scalar(-2.71799795);
  const Scalar _tmp75 =
      std::sqrt(Scalar(std::pow(_tmp73, Scalar(2)) + std::pow(_tmp74, Scalar(2))));
  const Scalar _tmp76 = Scalar(1.0) / (_tmp75);
  const Scalar _tmp77 = Scalar(1.0) / (_tmp74);
  const Scalar _tmp78 = _tmp75 * _tmp77;
  const Scalar _tmp79 = _tmp78 * (-_tmp46 * _tmp74 * _tmp76 + _tmp49 * _tmp73 * _tmp76);
  const Scalar _tmp80 = _tmp68 * _tmp71;
  const Scalar _tmp81 = _tmp57 * _tmp72 - _tmp60 * _tmp80 + _tmp72 * _tmp79;
  const Scalar _tmp82 = _tmp73 * _tmp77;
  const Scalar _tmp83 = Scalar(1.0) / (_tmp72 * _tmp82 - _tmp80);
  const Scalar _tmp84 = Scalar(1.0) * _tmp83;
  const Scalar _tmp85 = _tmp18 - _tmp21;
  const Scalar _tmp86 = _tmp14 + _tmp85;
  const Scalar _tmp87 = _tmp14 + _tmp22;
  const Scalar _tmp88 = _tmp72 * _tmp87;
  const Scalar _tmp89 = _tmp80 * _tmp86 - _tmp82 * _tmp88;
  const Scalar _tmp90 = _tmp84 * _tmp89;
  const Scalar _tmp91 = -_tmp72 * _tmp86 + _tmp88;
  const Scalar _tmp92 = _tmp62 * _tmp90 - _tmp84 * _tmp91;
  const Scalar _tmp93 = _tmp55 + p_init1;
  const Scalar _tmp94 = _tmp93 + Scalar(8.3196563700000006);
  const Scalar _tmp95 = _tmp64 + p_init0;
  const Scalar _tmp96 = _tmp95 + Scalar(1.9874742000000001);
  const Scalar _tmp97 = std::pow(Scalar(std::pow(_tmp94, Scalar(2)) + std::pow(_tmp96, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp98 = _tmp94 * _tmp97;
  const Scalar _tmp99 = _tmp96 * _tmp97;
  const Scalar _tmp100 = _tmp82 * _tmp99 - _tmp98;
  const Scalar _tmp101 = _tmp100 * _tmp83;
  const Scalar _tmp102 = _tmp87 * _tmp99;
  const Scalar _tmp103 = _tmp15 + _tmp85;
  const Scalar _tmp104 = -_tmp101 * _tmp89 - _tmp102 * _tmp82 + _tmp103 * _tmp98;
  const Scalar _tmp105 = -_tmp101 * _tmp91 + _tmp102 - _tmp103 * _tmp99 - _tmp104 * _tmp62;
  const Scalar _tmp106 = Scalar(1.0) / (_tmp105);
  const Scalar _tmp107 = -_tmp101 * _tmp81 + _tmp55 * _tmp99 - _tmp64 * _tmp98 + _tmp79 * _tmp99;
  const Scalar _tmp108 = _tmp106 * _tmp107;
  const Scalar _tmp109 = Scalar(1.0) / (_tmp107);
  const Scalar _tmp110 = _tmp105 * _tmp109;
  const Scalar _tmp111 = _tmp110 * (-_tmp108 * _tmp92 - _tmp81 * _tmp84);
  const Scalar _tmp112 = _tmp56 * _tmp65;
  const Scalar _tmp113 = _tmp106 * (_tmp111 + _tmp92);
  const Scalar _tmp114 = -_tmp104 * _tmp113 + _tmp111 * _tmp112 - _tmp90;
  const Scalar _tmp115 = Scalar(1.0) * _tmp58;
  const Scalar _tmp116 = _tmp41 + Scalar(-2.5202214700000001);
  const Scalar _tmp117 = _tmp34 + Scalar(8.3888750099999996);
  const Scalar _tmp118 =
      std::pow(Scalar(std::pow(_tmp116, Scalar(2)) + std::pow(_tmp117, Scalar(2))),
               Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp119 = _tmp117 * _tmp118;
  const Scalar _tmp120 = _tmp119 * fh1;
  const Scalar _tmp121 = Scalar(1.0) * _tmp109;
  const Scalar _tmp122 = _tmp56 * _tmp66;
  const Scalar _tmp123 = -_tmp104 * _tmp121 + _tmp110 * _tmp122;
  const Scalar _tmp124 = _tmp110 * _tmp66;
  const Scalar _tmp125 = _tmp116 * _tmp118;
  const Scalar _tmp126 = fh1 * (_tmp119 * _tmp40 - _tmp125 * _tmp33);
  const Scalar _tmp127 = _tmp63 * _tmp66 + Scalar(1.0);
  const Scalar _tmp128 = _tmp62 * _tmp66;
  const Scalar _tmp129 = _tmp23 * fh1;
  const Scalar _tmp130 = -_tmp119 * _tmp129 - Scalar(3.29616) * _tmp27 - _tmp33 * fv1;
  const Scalar _tmp131 = _tmp52 * _tmp62 + _tmp61;
  const Scalar _tmp132 = 0;
  const Scalar _tmp133 = _tmp131 * _tmp65;
  const Scalar _tmp134 = -_tmp104 * _tmp132 - _tmp133 * _tmp56 + _tmp53;
  const Scalar _tmp135 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp136 = _tmp82 * _tmp83;
  const Scalar _tmp137 = _tmp136 * _tmp89 + _tmp82 * _tmp87;
  const Scalar _tmp138 = _tmp136 * _tmp91 - _tmp137 * _tmp62 - _tmp87;
  const Scalar _tmp139 = _tmp110 * (-_tmp108 * _tmp138 + _tmp136 * _tmp81 - _tmp79);
  const Scalar _tmp140 = _tmp106 * (_tmp138 + _tmp139);
  const Scalar _tmp141 = -_tmp104 * _tmp140 + _tmp112 * _tmp139 + _tmp137;
  const Scalar _tmp142 = _tmp125 * fh1;
  const Scalar _tmp143 = _tmp125 * _tmp129 + Scalar(3.29616) * _tmp36 + _tmp40 * fv1;
  const Scalar _tmp144 = -_tmp100 * _tmp140 - _tmp82;
  const Scalar _tmp145 = _tmp72 * _tmp83;
  const Scalar _tmp146 = -_tmp100 * _tmp113 + Scalar(1.0);
  const Scalar _tmp147 = _tmp100 * _tmp109 * _tmp84;
  const Scalar _tmp148 = -_tmp120 * _tmp78 * (_tmp113 * _tmp99 + _tmp145 * _tmp146) -
                         _tmp126 * _tmp78 * (_tmp121 * _tmp99 - _tmp147 * _tmp72) -
                         _tmp135 * _tmp78 * (-_tmp101 * _tmp132 * _tmp72 + _tmp132 * _tmp99) -
                         _tmp142 * _tmp78 * (_tmp140 * _tmp99 + _tmp144 * _tmp145 + Scalar(1.0));
  const Scalar _tmp149 = Scalar(1.0) / (_tmp148);
  const Scalar _tmp150 = std::asinh(
      _tmp149 * (Scalar(1.0) * _tmp120 * (_tmp111 * _tmp66 - _tmp114 * _tmp115) +
                 Scalar(1.0) * _tmp126 * (-_tmp115 * _tmp123 + _tmp124) +
                 Scalar(1.0) * _tmp130 * (-_tmp115 * _tmp127 + _tmp128) +
                 Scalar(1.0) * _tmp135 * (-_tmp115 * _tmp134 - _tmp131 * _tmp66 + Scalar(1.0)) +
                 Scalar(1.0) * _tmp142 * (-_tmp115 * _tmp141 + _tmp139 * _tmp66) +
                 Scalar(1.0) * _tmp143 * (_tmp122 * _tmp58 - _tmp66)));
  const Scalar _tmp151 = Scalar(9.6622558468725703) * _tmp148;
  const Scalar _tmp152 = Scalar(23.361089618893828) *
                             std::pow(Scalar(1 - Scalar(0.20689664689659551) * _tmp67), Scalar(2)) +
                         Scalar(3.2278567553341642) *
                             std::pow(Scalar(-Scalar(0.55659957866191134) * _tmp69 - 1), Scalar(2));
  const Scalar _tmp153 = _tmp132 * _tmp135;
  const Scalar _tmp154 = -_tmp101 * _tmp153 + _tmp120 * _tmp146 * _tmp83 - _tmp126 * _tmp147 +
                         _tmp142 * _tmp144 * _tmp83;
  const Scalar _tmp155 = Scalar(1.0) / (_tmp154);
  const Scalar _tmp156 = _tmp143 * _tmp66;
  const Scalar _tmp157 =
      std::asinh(_tmp155 * (_tmp114 * _tmp120 * _tmp58 + _tmp123 * _tmp126 * _tmp58 +
                            _tmp127 * _tmp130 * _tmp58 + _tmp134 * _tmp135 * _tmp58 +
                            _tmp141 * _tmp142 * _tmp58 - _tmp156 * _tmp56 * _tmp58));
  const Scalar _tmp158 = Scalar(9.6622558468725703) * _tmp154;
  const Scalar _tmp159 =
      Scalar(69.216682114881593) *
          std::pow(Scalar(-Scalar(0.12019727204189803) * _tmp93 - 1), Scalar(2)) +
      Scalar(3.9500536956656402) *
          std::pow(Scalar(-Scalar(0.50315118556004401) * _tmp95 - 1), Scalar(2));
  const Scalar _tmp160 = _tmp113 * _tmp120 + _tmp121 * _tmp126 + _tmp140 * _tmp142 + _tmp153;
  const Scalar _tmp161 = Scalar(1.0) / (_tmp160);
  const Scalar _tmp162 =
      std::asinh(_tmp161 * (-_tmp111 * _tmp120 * _tmp65 - _tmp124 * _tmp126 - _tmp128 * _tmp130 +
                            _tmp133 * _tmp135 - _tmp139 * _tmp142 * _tmp65 + _tmp156));
  const Scalar _tmp163 = Scalar(9.6622558468725703) * _tmp160;

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) =
      _tmp45 * (-std::sinh(Scalar(1.0) * _tmp44) -
                std::sinh(Scalar(0.1034955) * _tmp43 * (-std::sqrt(_tmp42) - _tmp44 * _tmp45))) -
      Scalar(8.4693136199999994) *
          std::sqrt(Scalar(Scalar(0.013941309530580858) * _tmp42 +
                           std::pow(Scalar(-Scalar(0.11807332268798426) * _tmp23 -
                                           Scalar(0.11807332268798426) * p_init2 + 1),
                                    Scalar(2))));
  _res(1, 0) =
      _tmp151 *
          (-std::sinh(Scalar(1.0) * _tmp150) -
           std::sinh(Scalar(0.1034955) * _tmp149 * (-_tmp150 * _tmp151 - std::sqrt(_tmp51)))) -
      Scalar(8.36416322) *
          std::sqrt(Scalar(Scalar(0.014294040284261563) * _tmp51 +
                           std::pow(Scalar(-Scalar(0.1195576860108189) * _tmp87 -
                                           Scalar(0.1195576860108189) * p_init2 + 1),
                                    Scalar(2))));
  _res(2, 0) =
      _tmp158 *
          (-std::sinh(Scalar(1.0) * _tmp157) -
           std::sinh(Scalar(0.1034955) * _tmp155 * (-std::sqrt(_tmp152) - _tmp157 * _tmp158))) -
      Scalar(8.3700199099999999) *
          std::sqrt(Scalar(Scalar(0.01427404356387209) * _tmp152 +
                           std::pow(Scalar(-Scalar(0.11947402882581673) * _tmp86 -
                                           Scalar(0.11947402882581673) * p_init2 + 1),
                                    Scalar(2))));
  _res(3, 0) =
      _tmp163 *
          (-std::sinh(Scalar(1.0) * _tmp162) -
           std::sinh(Scalar(0.1034955) * _tmp161 * (-std::sqrt(_tmp159) - _tmp162 * _tmp163))) -
      Scalar(8.4718465799999993) *
          std::sqrt(Scalar(Scalar(0.013932974275675287) * _tmp159 +
                           std::pow(Scalar(-Scalar(0.11803802046660766) * _tmp103 -
                                           Scalar(0.11803802046660766) * p_init2 + 1),
                                    Scalar(2))));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym