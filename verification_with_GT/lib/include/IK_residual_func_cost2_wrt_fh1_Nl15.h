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
 * Symbolic function: IK_residual_func_cost2_wrt_fh1_Nl15
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost2WrtFh1Nl15(
    const Scalar fh1, const Scalar fv1, const Scalar rx, const Scalar ry, const Scalar rz,
    const Scalar p_init0, const Scalar p_init1, const Scalar p_init2, const Scalar rot_init_x,
    const Scalar rot_init_y, const Scalar rot_init_z, const Scalar rot_init_w,
    const Scalar epsilon) {
  // Total ops: 650

  // Unused inputs
  (void)p_init2;
  (void)epsilon;

  // Input arrays

  // Intermediate terms (218)
  const Scalar _tmp0 = std::pow(fh1, Scalar(-2));
  const Scalar _tmp1 = Scalar(1.0) / (fh1);
  const Scalar _tmp2 = _tmp1 * fv1;
  const Scalar _tmp3 = std::asinh(_tmp2);
  const Scalar _tmp4 = Scalar(9.6622558468725703) * _tmp3;
  const Scalar _tmp5 = std::sqrt(
      Scalar(std::pow(rx, Scalar(2)) + std::pow(ry, Scalar(2)) + std::pow(rz, Scalar(2))));
  const Scalar _tmp6 = (Scalar(1) / Scalar(2)) * _tmp5;
  const Scalar _tmp7 = std::cos(_tmp6);
  const Scalar _tmp8 = std::sin(_tmp6) / _tmp5;
  const Scalar _tmp9 = _tmp8 * rot_init_w;
  const Scalar _tmp10 = _tmp8 * rx;
  const Scalar _tmp11 = _tmp8 * rz;
  const Scalar _tmp12 = _tmp10 * rot_init_z - _tmp11 * rot_init_x + _tmp7 * rot_init_y + _tmp9 * ry;
  const Scalar _tmp13 = _tmp8 * ry;
  const Scalar _tmp14 = _tmp11 * rot_init_y - _tmp13 * rot_init_z + _tmp7 * rot_init_x + _tmp9 * rx;
  const Scalar _tmp15 = 2 * _tmp14;
  const Scalar _tmp16 = _tmp12 * _tmp15;
  const Scalar _tmp17 =
      -_tmp10 * rot_init_x - _tmp11 * rot_init_z - _tmp13 * rot_init_y + _tmp7 * rot_init_w;
  const Scalar _tmp18 =
      -_tmp10 * rot_init_y + _tmp13 * rot_init_x + _tmp7 * rot_init_z + _tmp9 * rz;
  const Scalar _tmp19 = 2 * _tmp18;
  const Scalar _tmp20 = _tmp17 * _tmp19;
  const Scalar _tmp21 = Scalar(0.20999999999999999) * _tmp16 + Scalar(0.20999999999999999) * _tmp20;
  const Scalar _tmp22 = -2 * std::pow(_tmp14, Scalar(2));
  const Scalar _tmp23 = 1 - 2 * std::pow(_tmp18, Scalar(2));
  const Scalar _tmp24 = Scalar(0.20999999999999999) * _tmp22 + Scalar(0.20999999999999999) * _tmp23;
  const Scalar _tmp25 = _tmp12 * _tmp19;
  const Scalar _tmp26 = _tmp15 * _tmp17;
  const Scalar _tmp27 = _tmp25 - _tmp26;
  const Scalar _tmp28 = -Scalar(0.010999999999999999) * _tmp27;
  const Scalar _tmp29 = _tmp24 + _tmp28;
  const Scalar _tmp30 = _tmp21 + _tmp29;
  const Scalar _tmp31 = _tmp30 + p_init1;
  const Scalar _tmp32 = -2 * std::pow(_tmp12, Scalar(2));
  const Scalar _tmp33 = Scalar(0.20999999999999999) * _tmp23 + Scalar(0.20999999999999999) * _tmp32;
  const Scalar _tmp34 = _tmp14 * _tmp19;
  const Scalar _tmp35 = 2 * _tmp12 * _tmp17;
  const Scalar _tmp36 = _tmp34 + _tmp35;
  const Scalar _tmp37 = -Scalar(0.010999999999999999) * _tmp36;
  const Scalar _tmp38 = Scalar(0.20999999999999999) * _tmp16 - Scalar(0.20999999999999999) * _tmp20;
  const Scalar _tmp39 = _tmp37 + _tmp38;
  const Scalar _tmp40 = _tmp33 + _tmp39;
  const Scalar _tmp41 = _tmp40 + p_init0;
  const Scalar _tmp42 =
      -Scalar(0.1034955) * _tmp4 * fh1 -
      Scalar(0.49421237293624504) *
          std::sqrt(
              Scalar(std::pow(Scalar(1 - Scalar(0.20941503221602112) * _tmp31), Scalar(2)) +
                     Scalar(0.32397683292140877) *
                         std::pow(Scalar(1 - Scalar(0.36791786395571047) * _tmp41), Scalar(2))));
  const Scalar _tmp43 =
      std::pow(Scalar(_tmp0 * std::pow(fv1, Scalar(2)) + 1), Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp44 = _tmp1 * _tmp42;
  const Scalar _tmp45 = Scalar(1.0) * _tmp3;
  const Scalar _tmp46 = -_tmp24 + _tmp28;
  const Scalar _tmp47 = _tmp21 + _tmp46;
  const Scalar _tmp48 = _tmp47 + p_init1;
  const Scalar _tmp49 = _tmp37 - _tmp38;
  const Scalar _tmp50 = _tmp33 + _tmp49;
  const Scalar _tmp51 = _tmp50 + p_init0;
  const Scalar _tmp52 = -_tmp21;
  const Scalar _tmp53 = _tmp46 + _tmp52;
  const Scalar _tmp54 = _tmp53 + p_init1;
  const Scalar _tmp55 = _tmp54 + Scalar(8.3196563700000006);
  const Scalar _tmp56 = -_tmp33;
  const Scalar _tmp57 = _tmp49 + _tmp56;
  const Scalar _tmp58 = _tmp57 + p_init0;
  const Scalar _tmp59 = _tmp58 + Scalar(1.9874742000000001);
  const Scalar _tmp60 = std::pow(Scalar(std::pow(_tmp55, Scalar(2)) + std::pow(_tmp59, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp61 = _tmp59 * _tmp60;
  const Scalar _tmp62 = Scalar(0.20999999999999999) * _tmp25 + Scalar(0.20999999999999999) * _tmp26;
  const Scalar _tmp63 = -_tmp62;
  const Scalar _tmp64 = -Scalar(0.010999999999999999) * _tmp22 -
                        Scalar(0.010999999999999999) * _tmp32 + Scalar(-0.010999999999999999);
  const Scalar _tmp65 = Scalar(0.20999999999999999) * _tmp34 - Scalar(0.20999999999999999) * _tmp35;
  const Scalar _tmp66 = _tmp64 + _tmp65;
  const Scalar _tmp67 = _tmp63 + _tmp66;
  const Scalar _tmp68 = _tmp51 + Scalar(-2.5202214700000001);
  const Scalar _tmp69 = Scalar(1.0) / (_tmp68);
  const Scalar _tmp70 = _tmp48 + Scalar(8.3888750099999996);
  const Scalar _tmp71 = _tmp69 * _tmp70;
  const Scalar _tmp72 = _tmp67 * _tmp71;
  const Scalar _tmp73 = _tmp29 + _tmp52;
  const Scalar _tmp74 = _tmp73 + p_init1;
  const Scalar _tmp75 = _tmp74 + Scalar(-4.8333311099999996);
  const Scalar _tmp76 = _tmp39 + _tmp56;
  const Scalar _tmp77 = _tmp76 + p_init0;
  const Scalar _tmp78 = _tmp77 + Scalar(1.79662371);
  const Scalar _tmp79 = std::pow(Scalar(std::pow(_tmp75, Scalar(2)) + std::pow(_tmp78, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp80 = _tmp78 * _tmp79;
  const Scalar _tmp81 = _tmp64 - _tmp65;
  const Scalar _tmp82 = _tmp62 + _tmp81;
  const Scalar _tmp83 = _tmp75 * _tmp79;
  const Scalar _tmp84 = -_tmp72 * _tmp80 + _tmp82 * _tmp83;
  const Scalar _tmp85 = _tmp55 * _tmp60;
  const Scalar _tmp86 = _tmp61 * _tmp71 - _tmp85;
  const Scalar _tmp87 = Scalar(1.0) / (_tmp71 * _tmp80 - _tmp83);
  const Scalar _tmp88 = _tmp86 * _tmp87;
  const Scalar _tmp89 = _tmp63 + _tmp81;
  const Scalar _tmp90 = -_tmp61 * _tmp72 - _tmp84 * _tmp88 + _tmp85 * _tmp89;
  const Scalar _tmp91 = Scalar(1.0) * _tmp47;
  const Scalar _tmp92 = -_tmp91;
  const Scalar _tmp93 = Scalar(1.0) / (_tmp73 + _tmp92);
  const Scalar _tmp94 = Scalar(1.0) * _tmp50;
  const Scalar _tmp95 = _tmp93 * (-_tmp76 + _tmp94);
  const Scalar _tmp96 = _tmp67 * _tmp80 - _tmp80 * _tmp82;
  const Scalar _tmp97 = _tmp61 * _tmp67 - _tmp61 * _tmp89 - _tmp88 * _tmp96 - _tmp90 * _tmp95;
  const Scalar _tmp98 = Scalar(1.0) / (_tmp97);
  const Scalar _tmp99 = Scalar(1.0) * _tmp87;
  const Scalar _tmp100 = _tmp84 * _tmp99;
  const Scalar _tmp101 = _tmp100 * _tmp95 - _tmp96 * _tmp99;
  const Scalar _tmp102 =
      std::sqrt(Scalar(std::pow(_tmp68, Scalar(2)) + std::pow(_tmp70, Scalar(2))));
  const Scalar _tmp103 = Scalar(1.0) / (_tmp102);
  const Scalar _tmp104 = _tmp102 * _tmp69;
  const Scalar _tmp105 = _tmp104 * (-_tmp103 * _tmp47 * _tmp68 + _tmp103 * _tmp50 * _tmp70);
  const Scalar _tmp106 = _tmp105 * _tmp80 + _tmp73 * _tmp80 - _tmp76 * _tmp83;
  const Scalar _tmp107 = _tmp105 * _tmp61 - _tmp106 * _tmp88 + _tmp53 * _tmp61 - _tmp57 * _tmp85;
  const Scalar _tmp108 = _tmp107 * _tmp98;
  const Scalar _tmp109 = Scalar(1.0) / (_tmp107);
  const Scalar _tmp110 = _tmp109 * _tmp97;
  const Scalar _tmp111 = _tmp110 * (-_tmp101 * _tmp108 - _tmp106 * _tmp99);
  const Scalar _tmp112 = _tmp98 * (_tmp101 + _tmp111);
  const Scalar _tmp113 = -_tmp112 * _tmp86 + Scalar(1.0);
  const Scalar _tmp114 = _tmp80 * _tmp87;
  const Scalar _tmp115 = _tmp31 + Scalar(-4.7752063900000001);
  const Scalar _tmp116 = _tmp41 + Scalar(-2.71799795);
  const Scalar _tmp117 =
      std::pow(Scalar(std::pow(_tmp115, Scalar(2)) + std::pow(_tmp116, Scalar(2))),
               Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp118 = _tmp115 * _tmp117;
  const Scalar _tmp119 = _tmp104 * _tmp118 * (_tmp112 * _tmp61 + _tmp113 * _tmp114);
  const Scalar _tmp120 = _tmp116 * _tmp117;
  const Scalar _tmp121 = _tmp118 * _tmp40 - _tmp120 * _tmp30;
  const Scalar _tmp122 = Scalar(1.0) * _tmp109;
  const Scalar _tmp123 = _tmp80 * _tmp88;
  const Scalar _tmp124 = _tmp104 * _tmp121 * (-_tmp122 * _tmp123 + _tmp122 * _tmp61);
  const Scalar _tmp125 = _tmp71 * _tmp87;
  const Scalar _tmp126 = _tmp125 * _tmp84 + _tmp72;
  const Scalar _tmp127 = _tmp125 * _tmp96 - _tmp126 * _tmp95 - _tmp67;
  const Scalar _tmp128 = _tmp110 * (-_tmp105 + _tmp106 * _tmp125 - _tmp108 * _tmp127);
  const Scalar _tmp129 = _tmp98 * (_tmp127 + _tmp128);
  const Scalar _tmp130 = -_tmp129 * _tmp86 - _tmp71;
  const Scalar _tmp131 = _tmp104 * _tmp120 * (_tmp114 * _tmp130 + _tmp129 * _tmp61 + Scalar(1.0));
  const Scalar _tmp132 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp133 = _tmp91 * _tmp95 + _tmp94;
  const Scalar _tmp134 = 0;
  const Scalar _tmp135 = -_tmp104 * _tmp132 * (-_tmp123 * _tmp134 + _tmp134 * _tmp61) -
                         _tmp119 * fh1 - _tmp124 * fh1 - _tmp131 * fh1;
  const Scalar _tmp136 = Scalar(1.0) / (_tmp135);
  const Scalar _tmp137 = _tmp53 + _tmp92;
  const Scalar _tmp138 = _tmp137 * _tmp95;
  const Scalar _tmp139 = Scalar(1.0) / (-_tmp138 - _tmp57 + _tmp94);
  const Scalar _tmp140 = _tmp137 * _tmp139;
  const Scalar _tmp141 = -_tmp100 + _tmp111 * _tmp140 - _tmp112 * _tmp90;
  const Scalar _tmp142 = Scalar(1.0) * _tmp93;
  const Scalar _tmp143 = Scalar(1.0) * _tmp139;
  const Scalar _tmp144 = Scalar(1.0) * _tmp118 * (_tmp111 * _tmp143 - _tmp141 * _tmp142);
  const Scalar _tmp145 = _tmp62 + _tmp66;
  const Scalar _tmp146 = _tmp145 * fh1;
  const Scalar _tmp147 = _tmp120 * _tmp146 + Scalar(3.29616) * _tmp36 + _tmp40 * fv1;
  const Scalar _tmp148 = _tmp137 * _tmp143;
  const Scalar _tmp149 = -Scalar(1.0) * _tmp143 + Scalar(1.0) * _tmp148 * _tmp93;
  const Scalar _tmp150 = -_tmp118 * _tmp146 - Scalar(3.29616) * _tmp27 - _tmp30 * fv1;
  const Scalar _tmp151 = _tmp93 * (_tmp138 * _tmp143 + Scalar(1.0));
  const Scalar _tmp152 = _tmp143 * _tmp95;
  const Scalar _tmp153 = -Scalar(1.0) * _tmp151 + Scalar(1.0) * _tmp152;
  const Scalar _tmp154 = _tmp133 * _tmp139;
  const Scalar _tmp155 = -_tmp134 * _tmp90 - _tmp137 * _tmp154 + _tmp92;
  const Scalar _tmp156 = _tmp126 + _tmp128 * _tmp140 - _tmp129 * _tmp90;
  const Scalar _tmp157 = Scalar(1.0) * _tmp120 * (_tmp128 * _tmp143 - _tmp142 * _tmp156);
  const Scalar _tmp158 = _tmp110 * _tmp148 - _tmp122 * _tmp90;
  const Scalar _tmp159 = _tmp110 * _tmp143;
  const Scalar _tmp160 = Scalar(1.0) * _tmp121 * (-_tmp142 * _tmp158 + _tmp159);
  const Scalar _tmp161 =
      Scalar(1.0) * _tmp132 * (-_tmp133 * _tmp143 - _tmp142 * _tmp155 + Scalar(1.0)) +
      _tmp144 * fh1 + _tmp147 * _tmp149 + _tmp150 * _tmp153 + _tmp157 * fh1 + _tmp160 * fh1;
  const Scalar _tmp162 = std::asinh(_tmp136 * _tmp161);
  const Scalar _tmp163 = Scalar(9.6622558468725703) * _tmp135;
  const Scalar _tmp164 =
      -_tmp162 * _tmp163 -
      Scalar(8.3888750099999996) *
          std::sqrt(
              Scalar(Scalar(0.090254729040973036) *
                         std::pow(Scalar(1 - Scalar(0.39679052492160538) * _tmp51), Scalar(2)) +
                     std::pow(Scalar(-Scalar(0.11920549523123722) * _tmp48 - 1), Scalar(2))));
  const Scalar _tmp165 = Scalar(0.1034955) * _tmp136;
  const Scalar _tmp166 = _tmp164 * _tmp165;
  const Scalar _tmp167 = Scalar(1.0) * _tmp162;
  const Scalar _tmp168 = -_tmp119 - _tmp124 - _tmp131;
  const Scalar _tmp169 = Scalar(9.6622558468725703) * _tmp168;
  const Scalar _tmp170 = std::pow(_tmp135, Scalar(-2));
  const Scalar _tmp171 = _tmp118 * _tmp145;
  const Scalar _tmp172 = _tmp120 * _tmp145;
  const Scalar _tmp173 = _tmp168 * _tmp170;
  const Scalar _tmp174 =
      (_tmp136 * (_tmp144 + _tmp149 * _tmp172 - _tmp153 * _tmp171 + _tmp157 + _tmp160) -
       _tmp161 * _tmp173) /
      std::sqrt(Scalar(std::pow(_tmp161, Scalar(2)) * _tmp170 + 1));
  const Scalar _tmp175 = _tmp143 * _tmp147;
  const Scalar _tmp176 = _tmp121 * _tmp158 * _tmp93;
  const Scalar _tmp177 = _tmp120 * _tmp93;
  const Scalar _tmp178 = _tmp156 * _tmp177;
  const Scalar _tmp179 = _tmp118 * _tmp141 * _tmp93;
  const Scalar _tmp180 = _tmp132 * _tmp155 * _tmp93 - _tmp137 * _tmp175 * _tmp93 +
                         _tmp150 * _tmp151 + _tmp176 * fh1 + _tmp178 * fh1 + _tmp179 * fh1;
  const Scalar _tmp181 = _tmp120 * _tmp130 * _tmp87;
  const Scalar _tmp182 = _tmp121 * _tmp122;
  const Scalar _tmp183 = _tmp182 * fh1;
  const Scalar _tmp184 = _tmp132 * _tmp134;
  const Scalar _tmp185 = _tmp113 * _tmp118 * _tmp87;
  const Scalar _tmp186 = _tmp181 * fh1 - _tmp183 * _tmp88 - _tmp184 * _tmp88 + _tmp185 * fh1;
  const Scalar _tmp187 = Scalar(1.0) / (_tmp186);
  const Scalar _tmp188 = std::asinh(_tmp180 * _tmp187);
  const Scalar _tmp189 = Scalar(1.0) * _tmp188;
  const Scalar _tmp190 = std::pow(_tmp186, Scalar(-2));
  const Scalar _tmp191 = _tmp181 - _tmp182 * _tmp88 + _tmp185;
  const Scalar _tmp192 = _tmp190 * _tmp191;
  const Scalar _tmp193 =
      (-_tmp180 * _tmp192 +
       _tmp187 * (-_tmp145 * _tmp148 * _tmp177 - _tmp151 * _tmp171 + _tmp176 + _tmp178 + _tmp179)) /
      std::sqrt(Scalar(std::pow(_tmp180, Scalar(2)) * _tmp190 + 1));
  const Scalar _tmp194 = Scalar(9.6622558468725703) * _tmp191;
  const Scalar _tmp195 = Scalar(9.6622558468725703) * _tmp186;
  const Scalar _tmp196 = Scalar(0.1034955) * _tmp187;
  const Scalar _tmp197 =
      -_tmp188 * _tmp195 -
      Scalar(4.8333311099999996) *
          std::sqrt(
              Scalar(std::pow(Scalar(1 - Scalar(0.20689664689659551) * _tmp74), Scalar(2)) +
                     Scalar(0.13817235445745474) *
                         std::pow(Scalar(-Scalar(0.55659957866191134) * _tmp77 - 1), Scalar(2))));
  const Scalar _tmp198 = _tmp196 * _tmp197;
  const Scalar _tmp199 = _tmp112 * _tmp118;
  const Scalar _tmp200 = _tmp120 * _tmp129;
  const Scalar _tmp201 = _tmp183 + _tmp184 + _tmp199 * fh1 + _tmp200 * fh1;
  const Scalar _tmp202 = Scalar(1.0) / (_tmp201);
  const Scalar _tmp203 = _tmp121 * _tmp159;
  const Scalar _tmp204 = _tmp120 * _tmp128 * _tmp139;
  const Scalar _tmp205 = _tmp111 * _tmp118 * _tmp139;
  const Scalar _tmp206 = _tmp132 * _tmp154 - _tmp150 * _tmp152 + _tmp175 - _tmp203 * fh1 -
                         _tmp204 * fh1 - _tmp205 * fh1;
  const Scalar _tmp207 = std::asinh(_tmp202 * _tmp206);
  const Scalar _tmp208 = Scalar(9.6622558468725703) * _tmp207;
  const Scalar _tmp209 =
      -_tmp201 * _tmp208 -
      Scalar(8.3196563700000006) *
          std::sqrt(
              Scalar(std::pow(Scalar(-Scalar(0.12019727204189803) * _tmp54 - 1), Scalar(2)) +
                     Scalar(0.057067943376852184) *
                         std::pow(Scalar(-Scalar(0.50315118556004401) * _tmp58 - 1), Scalar(2))));
  const Scalar _tmp210 = Scalar(0.1034955) * _tmp202;
  const Scalar _tmp211 = _tmp209 * _tmp210;
  const Scalar _tmp212 = Scalar(1.0) * _tmp207;
  const Scalar _tmp213 = _tmp182 + _tmp199 + _tmp200;
  const Scalar _tmp214 = std::pow(_tmp201, Scalar(-2));
  const Scalar _tmp215 = _tmp213 * _tmp214;
  const Scalar _tmp216 = Scalar(9.6622558468725703) * _tmp201;
  const Scalar _tmp217 =
      (_tmp202 * (_tmp143 * _tmp172 + _tmp152 * _tmp171 - _tmp203 - _tmp204 - _tmp205) -
       _tmp206 * _tmp215) /
      std::sqrt(Scalar(std::pow(_tmp206, Scalar(2)) * _tmp214 + 1));

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) = Scalar(9.6622558468725703) * fh1 *
                   (Scalar(1.0) * _tmp0 * _tmp43 * fv1 * std::cosh(_tmp45) -
                    (-_tmp0 * _tmp42 + Scalar(0.1034955) * _tmp1 *
                                           (Scalar(9.6622558468725703) * _tmp2 * _tmp43 - _tmp4)) *
                        std::cosh(_tmp44)) -
               Scalar(9.6622558468725703) * std::sinh(_tmp44) -
               Scalar(9.6622558468725703) * std::sinh(_tmp45);
  _res(1, 0) = _tmp163 * (-Scalar(1.0) * _tmp174 * std::cosh(_tmp167) -
                          (-Scalar(0.1034955) * _tmp164 * _tmp173 +
                           _tmp165 * (-_tmp162 * _tmp169 - _tmp163 * _tmp174)) *
                              std::cosh(_tmp166)) +
               _tmp169 * (-std::sinh(_tmp166) - std::sinh(_tmp167));
  _res(2, 0) = _tmp194 * (-std::sinh(_tmp189) - std::sinh(_tmp198)) +
               _tmp195 * (-Scalar(1.0) * _tmp193 * std::cosh(_tmp189) -
                          (-Scalar(0.1034955) * _tmp192 * _tmp197 +
                           _tmp196 * (-_tmp188 * _tmp194 - _tmp193 * _tmp195)) *
                              std::cosh(_tmp198));
  _res(3, 0) = Scalar(9.6622558468725703) * _tmp213 * (-std::sinh(_tmp211) - std::sinh(_tmp212)) +
               _tmp216 * (-Scalar(1.0) * _tmp217 * std::cosh(_tmp212) -
                          (-Scalar(0.1034955) * _tmp209 * _tmp215 +
                           _tmp210 * (-_tmp208 * _tmp213 - _tmp216 * _tmp217)) *
                              std::cosh(_tmp211));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym