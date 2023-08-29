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
 * Symbolic function: IK_residual_func_cost2_wrt_fh1_Nl6
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost2WrtFh1Nl6(
    const Scalar fh1, const Scalar fv1, const Scalar rx, const Scalar ry, const Scalar rz,
    const Scalar p_init0, const Scalar p_init1, const Scalar p_init2, const Scalar rot_init_x,
    const Scalar rot_init_y, const Scalar rot_init_z, const Scalar rot_init_w,
    const Scalar epsilon) {
  // Total ops: 658

  // Unused inputs
  (void)p_init2;
  (void)epsilon;

  // Input arrays

  // Intermediate terms (226)
  const Scalar _tmp0 = std::pow(fh1, Scalar(-2));
  const Scalar _tmp1 = std::sqrt(
      Scalar(std::pow(rx, Scalar(2)) + std::pow(ry, Scalar(2)) + std::pow(rz, Scalar(2))));
  const Scalar _tmp2 = (Scalar(1) / Scalar(2)) * _tmp1;
  const Scalar _tmp3 = std::cos(_tmp2);
  const Scalar _tmp4 = std::sin(_tmp2) / _tmp1;
  const Scalar _tmp5 = _tmp4 * ry;
  const Scalar _tmp6 = _tmp4 * rx;
  const Scalar _tmp7 = _tmp4 * rot_init_x;
  const Scalar _tmp8 = _tmp3 * rot_init_y + _tmp5 * rot_init_w + _tmp6 * rot_init_z - _tmp7 * rz;
  const Scalar _tmp9 = _tmp4 * rot_init_y;
  const Scalar _tmp10 = _tmp3 * rot_init_x - _tmp5 * rot_init_z + _tmp6 * rot_init_w + _tmp9 * rz;
  const Scalar _tmp11 = 2 * _tmp10;
  const Scalar _tmp12 = _tmp11 * _tmp8;
  const Scalar _tmp13 = _tmp4 * rz;
  const Scalar _tmp14 = _tmp13 * rot_init_w + _tmp3 * rot_init_z + _tmp7 * ry - _tmp9 * rx;
  const Scalar _tmp15 =
      -2 * _tmp13 * rot_init_z + 2 * _tmp3 * rot_init_w - 2 * _tmp7 * rx - 2 * _tmp9 * ry;
  const Scalar _tmp16 = _tmp14 * _tmp15;
  const Scalar _tmp17 = Scalar(0.20999999999999999) * _tmp12 + Scalar(0.20999999999999999) * _tmp16;
  const Scalar _tmp18 = -2 * std::pow(_tmp10, Scalar(2));
  const Scalar _tmp19 = -2 * std::pow(_tmp14, Scalar(2));
  const Scalar _tmp20 = Scalar(0.20999999999999999) * _tmp18 +
                        Scalar(0.20999999999999999) * _tmp19 + Scalar(0.20999999999999999);
  const Scalar _tmp21 = 2 * _tmp14 * _tmp8;
  const Scalar _tmp22 = _tmp10 * _tmp15;
  const Scalar _tmp23 = _tmp21 - _tmp22;
  const Scalar _tmp24 = -Scalar(0.010999999999999999) * _tmp23;
  const Scalar _tmp25 = -_tmp20 + _tmp24;
  const Scalar _tmp26 = _tmp17 + _tmp25;
  const Scalar _tmp27 = _tmp26 + p_init1;
  const Scalar _tmp28 = Scalar(0.20999999999999999) * _tmp12 - Scalar(0.20999999999999999) * _tmp16;
  const Scalar _tmp29 = -_tmp28;
  const Scalar _tmp30 = _tmp11 * _tmp14;
  const Scalar _tmp31 = _tmp15 * _tmp8;
  const Scalar _tmp32 = _tmp30 + _tmp31;
  const Scalar _tmp33 = -Scalar(0.010999999999999999) * _tmp32;
  const Scalar _tmp34 = 1 - 2 * std::pow(_tmp8, Scalar(2));
  const Scalar _tmp35 = Scalar(0.20999999999999999) * _tmp19 + Scalar(0.20999999999999999) * _tmp34;
  const Scalar _tmp36 = _tmp33 + _tmp35;
  const Scalar _tmp37 = _tmp29 + _tmp36;
  const Scalar _tmp38 = _tmp37 + p_init0;
  const Scalar _tmp39 = Scalar(1.0) / (fh1);
  const Scalar _tmp40 = _tmp39 * fv1;
  const Scalar _tmp41 = std::asinh(_tmp40);
  const Scalar _tmp42 = Scalar(9.6622558468725703) * _tmp41;
  const Scalar _tmp43 =
      -_tmp42 * fh1 -
      Scalar(8.3888750099999996) *
          std::sqrt(
              Scalar(Scalar(0.090254729040973036) *
                         std::pow(Scalar(1 - Scalar(0.39679052492160538) * _tmp38), Scalar(2)) +
                     std::pow(Scalar(-Scalar(0.11920549523123722) * _tmp27 - 1), Scalar(2))));
  const Scalar _tmp44 =
      std::pow(Scalar(_tmp0 * std::pow(fv1, Scalar(2)) + 1), Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp45 = Scalar(0.1034955) * _tmp39;
  const Scalar _tmp46 = _tmp43 * _tmp45;
  const Scalar _tmp47 = Scalar(1.0) * _tmp41;
  const Scalar _tmp48 = -_tmp17;
  const Scalar _tmp49 = _tmp20 + _tmp24;
  const Scalar _tmp50 = _tmp48 + _tmp49;
  const Scalar _tmp51 = _tmp50 + p_init1;
  const Scalar _tmp52 = _tmp51 + Scalar(-4.8333311099999996);
  const Scalar _tmp53 = _tmp33 - _tmp35;
  const Scalar _tmp54 = _tmp28 + _tmp53;
  const Scalar _tmp55 = _tmp54 + p_init0;
  const Scalar _tmp56 = _tmp55 + Scalar(1.79662371);
  const Scalar _tmp57 = std::pow(Scalar(std::pow(_tmp52, Scalar(2)) + std::pow(_tmp56, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp58 = _tmp56 * _tmp57;
  const Scalar _tmp59 = _tmp25 + _tmp48;
  const Scalar _tmp60 = _tmp59 + p_init1;
  const Scalar _tmp61 = _tmp60 + Scalar(8.3196563700000006);
  const Scalar _tmp62 = _tmp29 + _tmp53;
  const Scalar _tmp63 = _tmp62 + p_init0;
  const Scalar _tmp64 = _tmp63 + Scalar(1.9874742000000001);
  const Scalar _tmp65 = Scalar(1.0) / (_tmp64);
  const Scalar _tmp66 = _tmp61 * _tmp65;
  const Scalar _tmp67 = Scalar(0.20999999999999999) * _tmp30 - Scalar(0.20999999999999999) * _tmp31;
  const Scalar _tmp68 = -_tmp67;
  const Scalar _tmp69 =
      -Scalar(0.010999999999999999) * _tmp18 - Scalar(0.010999999999999999) * _tmp34;
  const Scalar _tmp70 = Scalar(0.20999999999999999) * _tmp21 + Scalar(0.20999999999999999) * _tmp22;
  const Scalar _tmp71 = _tmp69 - _tmp70;
  const Scalar _tmp72 = _tmp68 + _tmp71;
  const Scalar _tmp73 = _tmp58 * _tmp72;
  const Scalar _tmp74 = _tmp69 + _tmp70;
  const Scalar _tmp75 = _tmp68 + _tmp74;
  const Scalar _tmp76 = _tmp52 * _tmp57;
  const Scalar _tmp77 = _tmp17 + _tmp49;
  const Scalar _tmp78 = _tmp77 + p_init1;
  const Scalar _tmp79 = _tmp78 + Scalar(-4.7752063900000001);
  const Scalar _tmp80 = _tmp28 + _tmp36;
  const Scalar _tmp81 = _tmp80 + p_init0;
  const Scalar _tmp82 = _tmp81 + Scalar(-2.71799795);
  const Scalar _tmp83 = std::pow(Scalar(std::pow(_tmp79, Scalar(2)) + std::pow(_tmp82, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp84 = _tmp82 * _tmp83;
  const Scalar _tmp85 = _tmp72 * _tmp84;
  const Scalar _tmp86 = _tmp67 + _tmp74;
  const Scalar _tmp87 = _tmp79 * _tmp83;
  const Scalar _tmp88 = -_tmp66 * _tmp85 + _tmp86 * _tmp87;
  const Scalar _tmp89 = Scalar(1.0) / (_tmp66 * _tmp84 - _tmp87);
  const Scalar _tmp90 = _tmp58 * _tmp66 - _tmp76;
  const Scalar _tmp91 = _tmp89 * _tmp90;
  const Scalar _tmp92 = -_tmp66 * _tmp73 + _tmp75 * _tmp76 - _tmp88 * _tmp91;
  const Scalar _tmp93 = Scalar(1.0) * _tmp59;
  const Scalar _tmp94 = -_tmp93;
  const Scalar _tmp95 = Scalar(1.0) / (_tmp77 + _tmp94);
  const Scalar _tmp96 = Scalar(1.0) * _tmp62;
  const Scalar _tmp97 = _tmp95 * (-_tmp80 + _tmp96);
  const Scalar _tmp98 = -_tmp84 * _tmp86 + _tmp85;
  const Scalar _tmp99 = -_tmp58 * _tmp75 + _tmp73 - _tmp91 * _tmp98 - _tmp92 * _tmp97;
  const Scalar _tmp100 = Scalar(1.0) / (_tmp99);
  const Scalar _tmp101 = Scalar(1.0) * _tmp89;
  const Scalar _tmp102 = _tmp101 * _tmp88;
  const Scalar _tmp103 = -_tmp101 * _tmp98 + _tmp102 * _tmp97;
  const Scalar _tmp104 =
      std::sqrt(Scalar(std::pow(_tmp61, Scalar(2)) + std::pow(_tmp64, Scalar(2))));
  const Scalar _tmp105 = Scalar(1.0) / (_tmp104);
  const Scalar _tmp106 = _tmp104 * _tmp65;
  const Scalar _tmp107 = _tmp106 * (-_tmp105 * _tmp59 * _tmp64 + _tmp105 * _tmp61 * _tmp62);
  const Scalar _tmp108 = _tmp107 * _tmp84 + _tmp77 * _tmp84 - _tmp80 * _tmp87;
  const Scalar _tmp109 = _tmp107 * _tmp58 - _tmp108 * _tmp91 + _tmp50 * _tmp58 - _tmp54 * _tmp76;
  const Scalar _tmp110 = _tmp100 * _tmp109;
  const Scalar _tmp111 = Scalar(1.0) / (_tmp109);
  const Scalar _tmp112 = _tmp111 * _tmp99;
  const Scalar _tmp113 = _tmp112 * (-_tmp101 * _tmp108 - _tmp103 * _tmp110);
  const Scalar _tmp114 = _tmp103 + _tmp113;
  const Scalar _tmp115 = _tmp100 * _tmp114;
  const Scalar _tmp116 = _tmp100 * _tmp90;
  const Scalar _tmp117 = _tmp89 * (-_tmp114 * _tmp116 + Scalar(1.0));
  const Scalar _tmp118 = _tmp38 + Scalar(-2.5202214700000001);
  const Scalar _tmp119 = _tmp27 + Scalar(8.3888750099999996);
  const Scalar _tmp120 =
      std::pow(Scalar(std::pow(_tmp118, Scalar(2)) + std::pow(_tmp119, Scalar(2))),
               Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp121 = _tmp119 * _tmp120;
  const Scalar _tmp122 = _tmp106 * _tmp121 * (_tmp115 * _tmp58 + _tmp117 * _tmp84);
  const Scalar _tmp123 = _tmp66 * _tmp89;
  const Scalar _tmp124 = _tmp123 * _tmp88 + _tmp66 * _tmp72;
  const Scalar _tmp125 = _tmp123 * _tmp98 - _tmp124 * _tmp97 - _tmp72;
  const Scalar _tmp126 = _tmp112 * (-_tmp107 + _tmp108 * _tmp123 - _tmp110 * _tmp125);
  const Scalar _tmp127 = _tmp125 + _tmp126;
  const Scalar _tmp128 = _tmp89 * (-_tmp116 * _tmp127 - _tmp66);
  const Scalar _tmp129 = _tmp100 * _tmp127;
  const Scalar _tmp130 = _tmp118 * _tmp120;
  const Scalar _tmp131 = _tmp106 * _tmp130 * (_tmp128 * _tmp84 + _tmp129 * _tmp58 + Scalar(1.0));
  const Scalar _tmp132 = _tmp121 * _tmp37 - _tmp130 * _tmp26;
  const Scalar _tmp133 = _tmp101 * _tmp111 * _tmp90;
  const Scalar _tmp134 = Scalar(1.0) * _tmp111;
  const Scalar _tmp135 = _tmp106 * _tmp132 * (-_tmp133 * _tmp84 + _tmp134 * _tmp58);
  const Scalar _tmp136 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp137 = _tmp93 * _tmp97 + _tmp96;
  const Scalar _tmp138 = 0;
  const Scalar _tmp139 = _tmp100 * _tmp138;
  const Scalar _tmp140 = -_tmp106 * _tmp136 * (_tmp139 * _tmp58 - _tmp139 * _tmp84 * _tmp91) -
                         _tmp122 * fh1 - _tmp131 * fh1 - _tmp135 * fh1;
  const Scalar _tmp141 = Scalar(1.0) / (_tmp140);
  const Scalar _tmp142 = _tmp50 + _tmp94;
  const Scalar _tmp143 = _tmp142 * _tmp97;
  const Scalar _tmp144 = Scalar(1.0) / (-_tmp143 - _tmp54 + _tmp96);
  const Scalar _tmp145 = _tmp142 * _tmp144;
  const Scalar _tmp146 = _tmp100 * _tmp92;
  const Scalar _tmp147 = _tmp124 + _tmp126 * _tmp145 - _tmp127 * _tmp146;
  const Scalar _tmp148 = Scalar(1.0) * _tmp95;
  const Scalar _tmp149 = Scalar(1.0) * _tmp144;
  const Scalar _tmp150 = Scalar(1.0) * _tmp130 * (_tmp126 * _tmp149 - _tmp147 * _tmp148);
  const Scalar _tmp151 = _tmp67 + _tmp71;
  const Scalar _tmp152 = _tmp151 * fh1;
  const Scalar _tmp153 = -_tmp121 * _tmp152 - Scalar(3.29616) * _tmp23 - _tmp26 * fv1;
  const Scalar _tmp154 = _tmp143 * _tmp149 + Scalar(1.0);
  const Scalar _tmp155 = _tmp149 * _tmp97;
  const Scalar _tmp156 = -Scalar(1.0) * _tmp148 * _tmp154 + Scalar(1.0) * _tmp155;
  const Scalar _tmp157 = _tmp130 * _tmp152 + Scalar(3.29616) * _tmp32 + _tmp37 * fv1;
  const Scalar _tmp158 = _tmp142 * _tmp95;
  const Scalar _tmp159 = Scalar(1.0) * _tmp149 * _tmp158 - Scalar(1.0) * _tmp149;
  const Scalar _tmp160 = _tmp137 * _tmp144;
  const Scalar _tmp161 = -_tmp138 * _tmp146 - _tmp142 * _tmp160 + _tmp94;
  const Scalar _tmp162 = -_tmp102 + _tmp113 * _tmp145 - _tmp114 * _tmp146;
  const Scalar _tmp163 = Scalar(1.0) * _tmp121 * (_tmp113 * _tmp149 - _tmp148 * _tmp162);
  const Scalar _tmp164 = _tmp112 * _tmp149;
  const Scalar _tmp165 = _tmp95 * (-_tmp134 * _tmp92 + _tmp142 * _tmp164);
  const Scalar _tmp166 = Scalar(1.0) * _tmp132;
  const Scalar _tmp167 = _tmp166 * (_tmp164 - Scalar(1.0) * _tmp165);
  const Scalar _tmp168 =
      Scalar(1.0) * _tmp136 * (-_tmp137 * _tmp149 - _tmp148 * _tmp161 + Scalar(1.0)) +
      _tmp150 * fh1 + _tmp153 * _tmp156 + _tmp157 * _tmp159 + _tmp163 * fh1 + _tmp167 * fh1;
  const Scalar _tmp169 = std::asinh(_tmp141 * _tmp168);
  const Scalar _tmp170 = Scalar(1.0) * _tmp169;
  const Scalar _tmp171 = std::pow(_tmp140, Scalar(-2));
  const Scalar _tmp172 = -_tmp122 - _tmp131 - _tmp135;
  const Scalar _tmp173 = _tmp171 * _tmp172;
  const Scalar _tmp174 = _tmp121 * _tmp151;
  const Scalar _tmp175 = _tmp130 * _tmp151;
  const Scalar _tmp176 =
      (_tmp141 * (_tmp150 - _tmp156 * _tmp174 + _tmp159 * _tmp175 + _tmp163 + _tmp167) -
       _tmp168 * _tmp173) /
      std::sqrt(Scalar(std::pow(_tmp168, Scalar(2)) * _tmp171 + 1));
  const Scalar _tmp177 = Scalar(9.6622558468725703) * _tmp140;
  const Scalar _tmp178 =
      -_tmp169 * _tmp177 -
      Scalar(8.3196563700000006) *
          std::sqrt(
              Scalar(std::pow(Scalar(-Scalar(0.12019727204189803) * _tmp60 - 1), Scalar(2)) +
                     Scalar(0.057067943376852184) *
                         std::pow(Scalar(-Scalar(0.50315118556004401) * _tmp63 - 1), Scalar(2))));
  const Scalar _tmp179 = Scalar(9.6622558468725703) * _tmp172;
  const Scalar _tmp180 = Scalar(0.1034955) * _tmp141;
  const Scalar _tmp181 = _tmp178 * _tmp180;
  const Scalar _tmp182 = _tmp128 * _tmp130;
  const Scalar _tmp183 = _tmp117 * _tmp121;
  const Scalar _tmp184 = _tmp136 * _tmp139;
  const Scalar _tmp185 = _tmp132 * _tmp133;
  const Scalar _tmp186 = _tmp182 * fh1 + _tmp183 * fh1 - _tmp184 * _tmp91 - _tmp185 * fh1;
  const Scalar _tmp187 = Scalar(1.0) / (_tmp186);
  const Scalar _tmp188 = _tmp121 * _tmp162 * _tmp95;
  const Scalar _tmp189 = _tmp130 * _tmp95;
  const Scalar _tmp190 = _tmp147 * _tmp189;
  const Scalar _tmp191 = _tmp154 * _tmp95;
  const Scalar _tmp192 = _tmp132 * _tmp165;
  const Scalar _tmp193 = _tmp149 * _tmp157;
  const Scalar _tmp194 = _tmp136 * _tmp161 * _tmp95 + _tmp153 * _tmp191 - _tmp158 * _tmp193 +
                         _tmp188 * fh1 + _tmp190 * fh1 + _tmp192 * fh1;
  const Scalar _tmp195 = std::asinh(_tmp187 * _tmp194);
  const Scalar _tmp196 = Scalar(1.0) * _tmp195;
  const Scalar _tmp197 = std::pow(_tmp186, Scalar(-2));
  const Scalar _tmp198 = _tmp182 + _tmp183 - _tmp185;
  const Scalar _tmp199 = _tmp197 * _tmp198;
  const Scalar _tmp200 = (_tmp187 * (-_tmp142 * _tmp149 * _tmp151 * _tmp189 - _tmp174 * _tmp191 +
                                     _tmp188 + _tmp190 + _tmp192) -
                          _tmp194 * _tmp199) /
                         std::sqrt(Scalar(std::pow(_tmp194, Scalar(2)) * _tmp197 + 1));
  const Scalar _tmp201 = Scalar(9.6622558468725703) * _tmp186;
  const Scalar _tmp202 =
      -_tmp195 * _tmp201 -
      Scalar(4.7752063900000001) *
          std::sqrt(
              Scalar(std::pow(Scalar(1 - Scalar(0.20941503221602112) * _tmp78), Scalar(2)) +
                     Scalar(0.32397683292140877) *
                         std::pow(Scalar(1 - Scalar(0.36791786395571047) * _tmp81), Scalar(2))));
  const Scalar _tmp203 = Scalar(9.6622558468725703) * _tmp198;
  const Scalar _tmp204 = Scalar(0.1034955) * _tmp187;
  const Scalar _tmp205 = _tmp202 * _tmp204;
  const Scalar _tmp206 = _tmp113 * _tmp121 * _tmp144;
  const Scalar _tmp207 = _tmp126 * _tmp130 * _tmp144;
  const Scalar _tmp208 = _tmp132 * _tmp164;
  const Scalar _tmp209 = _tmp136 * _tmp160 - _tmp153 * _tmp155 + _tmp193 - _tmp206 * fh1 -
                         _tmp207 * fh1 - _tmp208 * fh1;
  const Scalar _tmp210 = _tmp115 * _tmp121;
  const Scalar _tmp211 = _tmp111 * _tmp166;
  const Scalar _tmp212 = _tmp129 * _tmp130;
  const Scalar _tmp213 = _tmp184 + _tmp210 * fh1 + _tmp211 * fh1 + _tmp212 * fh1;
  const Scalar _tmp214 = Scalar(1.0) / (_tmp213);
  const Scalar _tmp215 = std::asinh(_tmp209 * _tmp214);
  const Scalar _tmp216 = Scalar(1.0) * _tmp215;
  const Scalar _tmp217 = Scalar(9.6622558468725703) * _tmp213;
  const Scalar _tmp218 =
      -_tmp215 * _tmp217 -
      Scalar(4.8333311099999996) *
          std::sqrt(
              Scalar(std::pow(Scalar(1 - Scalar(0.20689664689659551) * _tmp51), Scalar(2)) +
                     Scalar(0.13817235445745474) *
                         std::pow(Scalar(-Scalar(0.55659957866191134) * _tmp55 - 1), Scalar(2))));
  const Scalar _tmp219 = Scalar(0.1034955) * _tmp214;
  const Scalar _tmp220 = _tmp218 * _tmp219;
  const Scalar _tmp221 = _tmp210 + _tmp211 + _tmp212;
  const Scalar _tmp222 = Scalar(9.6622558468725703) * _tmp221;
  const Scalar _tmp223 = std::pow(_tmp213, Scalar(-2));
  const Scalar _tmp224 = _tmp221 * _tmp223;
  const Scalar _tmp225 = (-_tmp209 * _tmp224 + _tmp214 * (_tmp149 * _tmp175 + _tmp155 * _tmp174 -
                                                          _tmp206 - _tmp207 - _tmp208)) /
                         std::sqrt(Scalar(std::pow(_tmp209, Scalar(2)) * _tmp223 + 1));

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) = Scalar(9.6622558468725703) * fh1 *
                   (Scalar(1.0) * _tmp0 * _tmp44 * fv1 * std::cosh(_tmp47) -
                    (-Scalar(0.1034955) * _tmp0 * _tmp43 +
                     _tmp45 * (Scalar(9.6622558468725703) * _tmp40 * _tmp44 - _tmp42)) *
                        std::cosh(_tmp46)) -
               Scalar(9.6622558468725703) * std::sinh(_tmp46) -
               Scalar(9.6622558468725703) * std::sinh(_tmp47);
  _res(1, 0) = _tmp177 * (-Scalar(1.0) * _tmp176 * std::cosh(_tmp170) -
                          (-Scalar(0.1034955) * _tmp173 * _tmp178 +
                           _tmp180 * (-_tmp169 * _tmp179 - _tmp176 * _tmp177)) *
                              std::cosh(_tmp181)) +
               _tmp179 * (-std::sinh(_tmp170) - std::sinh(_tmp181));
  _res(2, 0) = _tmp201 * (-Scalar(1.0) * _tmp200 * std::cosh(_tmp196) -
                          (-Scalar(0.1034955) * _tmp199 * _tmp202 +
                           _tmp204 * (-_tmp195 * _tmp203 - _tmp200 * _tmp201)) *
                              std::cosh(_tmp205)) +
               _tmp203 * (-std::sinh(_tmp196) - std::sinh(_tmp205));
  _res(3, 0) = _tmp217 * (-Scalar(1.0) * _tmp225 * std::cosh(_tmp216) -
                          (-Scalar(0.1034955) * _tmp218 * _tmp224 +
                           _tmp219 * (-_tmp215 * _tmp222 - _tmp217 * _tmp225)) *
                              std::cosh(_tmp220)) +
               _tmp222 * (-std::sinh(_tmp216) - std::sinh(_tmp220));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
