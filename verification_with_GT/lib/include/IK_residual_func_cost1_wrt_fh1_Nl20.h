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
 * Symbolic function: IK_residual_func_cost1_wrt_fh1_Nl20
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost1WrtFh1Nl20(
    const Scalar fh1, const Scalar fv1, const Scalar rx, const Scalar ry, const Scalar rz,
    const Scalar p_init0, const Scalar p_init1, const Scalar p_init2, const Scalar rot_init_x,
    const Scalar rot_init_y, const Scalar rot_init_z, const Scalar rot_init_w,
    const Scalar epsilon) {
  // Total ops: 671

  // Unused inputs
  (void)p_init2;
  (void)epsilon;

  // Input arrays

  // Intermediate terms (226)
  const Scalar _tmp0 = Scalar(1.0) / (fh1);
  const Scalar _tmp1 = _tmp0 * fv1;
  const Scalar _tmp2 = std::asinh(_tmp1);
  const Scalar _tmp3 = Scalar(9.6622558468725703) * _tmp2;
  const Scalar _tmp4 = std::sqrt(
      Scalar(std::pow(rx, Scalar(2)) + std::pow(ry, Scalar(2)) + std::pow(rz, Scalar(2))));
  const Scalar _tmp5 = (Scalar(1) / Scalar(2)) * _tmp4;
  const Scalar _tmp6 = std::cos(_tmp5);
  const Scalar _tmp7 = std::sin(_tmp5) / _tmp4;
  const Scalar _tmp8 = _tmp7 * ry;
  const Scalar _tmp9 = _tmp7 * rx;
  const Scalar _tmp10 = _tmp7 * rz;
  const Scalar _tmp11 =
      _tmp10 * rot_init_w + _tmp6 * rot_init_z + _tmp8 * rot_init_x - _tmp9 * rot_init_y;
  const Scalar _tmp12 = -2 * std::pow(_tmp11, Scalar(2));
  const Scalar _tmp13 = _tmp7 * rot_init_z;
  const Scalar _tmp14 = _tmp10 * rot_init_y - _tmp13 * ry + _tmp6 * rot_init_x + _tmp9 * rot_init_w;
  const Scalar _tmp15 = 1 - 2 * std::pow(_tmp14, Scalar(2));
  const Scalar _tmp16 = Scalar(0.20999999999999999) * _tmp12 + Scalar(0.20999999999999999) * _tmp15;
  const Scalar _tmp17 =
      -_tmp10 * rot_init_x + _tmp13 * rx + _tmp6 * rot_init_y + _tmp8 * rot_init_w;
  const Scalar _tmp18 = 2 * _tmp11;
  const Scalar _tmp19 = _tmp17 * _tmp18;
  const Scalar _tmp20 = -2 * _tmp10 * rot_init_z + 2 * _tmp6 * rot_init_w - 2 * _tmp8 * rot_init_y -
                        2 * _tmp9 * rot_init_x;
  const Scalar _tmp21 = _tmp14 * _tmp20;
  const Scalar _tmp22 = _tmp19 - _tmp21;
  const Scalar _tmp23 = -Scalar(0.010999999999999999) * _tmp22;
  const Scalar _tmp24 = 2 * _tmp14 * _tmp17;
  const Scalar _tmp25 = _tmp11 * _tmp20;
  const Scalar _tmp26 = Scalar(0.20999999999999999) * _tmp24 + Scalar(0.20999999999999999) * _tmp25;
  const Scalar _tmp27 = _tmp23 - _tmp26;
  const Scalar _tmp28 = _tmp16 + _tmp27;
  const Scalar _tmp29 = _tmp28 + p_init1;
  const Scalar _tmp30 = -2 * std::pow(_tmp17, Scalar(2));
  const Scalar _tmp31 = Scalar(0.20999999999999999) * _tmp12 +
                        Scalar(0.20999999999999999) * _tmp30 + Scalar(0.20999999999999999);
  const Scalar _tmp32 = -_tmp31;
  const Scalar _tmp33 = _tmp14 * _tmp18;
  const Scalar _tmp34 = _tmp17 * _tmp20;
  const Scalar _tmp35 = _tmp33 + _tmp34;
  const Scalar _tmp36 = -Scalar(0.010999999999999999) * _tmp35;
  const Scalar _tmp37 = Scalar(0.20999999999999999) * _tmp24 - Scalar(0.20999999999999999) * _tmp25;
  const Scalar _tmp38 = _tmp36 + _tmp37;
  const Scalar _tmp39 = _tmp32 + _tmp38;
  const Scalar _tmp40 = _tmp39 + p_init0;
  const Scalar _tmp41 =
      -_tmp3 * fh1 -
      Scalar(4.8333311099999996) *
          std::sqrt(
              Scalar(std::pow(Scalar(1 - Scalar(0.20689664689659551) * _tmp29), Scalar(2)) +
                     Scalar(0.13817235445745474) *
                         std::pow(Scalar(-Scalar(0.55659957866191134) * _tmp40 - 1), Scalar(2))));
  const Scalar _tmp42 = Scalar(0.1034955) * _tmp0;
  const Scalar _tmp43 = _tmp41 * _tmp42;
  const Scalar _tmp44 = Scalar(1.0) * _tmp2;
  const Scalar _tmp45 = std::pow(fh1, Scalar(-2));
  const Scalar _tmp46 =
      std::pow(Scalar(_tmp45 * std::pow(fv1, Scalar(2)) + 1), Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp47 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp48 = Scalar(0.20999999999999999) * _tmp33 - Scalar(0.20999999999999999) * _tmp34;
  const Scalar _tmp49 = -_tmp48;
  const Scalar _tmp50 =
      -Scalar(0.010999999999999999) * _tmp15 - Scalar(0.010999999999999999) * _tmp30;
  const Scalar _tmp51 = Scalar(0.20999999999999999) * _tmp19 + Scalar(0.20999999999999999) * _tmp21;
  const Scalar _tmp52 = _tmp50 - _tmp51;
  const Scalar _tmp53 = _tmp49 + _tmp52;
  const Scalar _tmp54 = -_tmp16;
  const Scalar _tmp55 = _tmp27 + _tmp54;
  const Scalar _tmp56 = _tmp55 + p_init1;
  const Scalar _tmp57 = _tmp56 + Scalar(8.3196563700000006);
  const Scalar _tmp58 = _tmp36 - _tmp37;
  const Scalar _tmp59 = _tmp32 + _tmp58;
  const Scalar _tmp60 = _tmp59 + p_init0;
  const Scalar _tmp61 = _tmp60 + Scalar(1.9874742000000001);
  const Scalar _tmp62 = std::pow(Scalar(std::pow(_tmp57, Scalar(2)) + std::pow(_tmp61, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp63 = _tmp61 * _tmp62;
  const Scalar _tmp64 = _tmp48 + _tmp52;
  const Scalar _tmp65 = _tmp63 * _tmp64;
  const Scalar _tmp66 = -_tmp53 * _tmp63 + _tmp65;
  const Scalar _tmp67 = _tmp31 + _tmp58;
  const Scalar _tmp68 = _tmp67 + p_init0;
  const Scalar _tmp69 = _tmp68 + Scalar(-2.5202214700000001);
  const Scalar _tmp70 = Scalar(1.0) / (_tmp69);
  const Scalar _tmp71 = _tmp23 + _tmp26;
  const Scalar _tmp72 = _tmp54 + _tmp71;
  const Scalar _tmp73 = _tmp72 + p_init1;
  const Scalar _tmp74 = _tmp73 + Scalar(8.3888750099999996);
  const Scalar _tmp75 = _tmp70 * _tmp74;
  const Scalar _tmp76 = _tmp57 * _tmp62;
  const Scalar _tmp77 = Scalar(1.0) / (_tmp63 * _tmp75 - _tmp76);
  const Scalar _tmp78 = _tmp16 + _tmp71;
  const Scalar _tmp79 = _tmp78 + p_init1;
  const Scalar _tmp80 = _tmp79 + Scalar(-4.7752063900000001);
  const Scalar _tmp81 = _tmp31 + _tmp38;
  const Scalar _tmp82 = _tmp81 + p_init0;
  const Scalar _tmp83 = _tmp82 + Scalar(-2.71799795);
  const Scalar _tmp84 = std::pow(Scalar(std::pow(_tmp80, Scalar(2)) + std::pow(_tmp83, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp85 = _tmp83 * _tmp84;
  const Scalar _tmp86 = _tmp80 * _tmp84;
  const Scalar _tmp87 = _tmp75 * _tmp85 - _tmp86;
  const Scalar _tmp88 = _tmp77 * _tmp87;
  const Scalar _tmp89 = _tmp50 + _tmp51;
  const Scalar _tmp90 = _tmp48 + _tmp89;
  const Scalar _tmp91 = _tmp64 * _tmp75;
  const Scalar _tmp92 = _tmp53 * _tmp76 - _tmp65 * _tmp75;
  const Scalar _tmp93 = -_tmp85 * _tmp91 + _tmp86 * _tmp90 - _tmp88 * _tmp92;
  const Scalar _tmp94 = Scalar(1.0) * _tmp72;
  const Scalar _tmp95 = -_tmp94;
  const Scalar _tmp96 = Scalar(1.0) / (_tmp55 + _tmp95);
  const Scalar _tmp97 = Scalar(1.0) * _tmp67;
  const Scalar _tmp98 = -_tmp59 + _tmp97;
  const Scalar _tmp99 = _tmp96 * _tmp98;
  const Scalar _tmp100 = _tmp64 * _tmp85 - _tmp66 * _tmp88 - _tmp85 * _tmp90 - _tmp93 * _tmp99;
  const Scalar _tmp101 = Scalar(1.0) / (_tmp100);
  const Scalar _tmp102 = _tmp94 * _tmp99 + _tmp97;
  const Scalar _tmp103 = 0;
  const Scalar _tmp104 = _tmp101 * _tmp103;
  const Scalar _tmp105 = _tmp63 * _tmp88;
  const Scalar _tmp106 =
      std::sqrt(Scalar(std::pow(_tmp69, Scalar(2)) + std::pow(_tmp74, Scalar(2))));
  const Scalar _tmp107 = _tmp106 * _tmp70;
  const Scalar _tmp108 = Scalar(1.0) / (_tmp106);
  const Scalar _tmp109 = _tmp107 * (_tmp108 * _tmp67 * _tmp74 - _tmp108 * _tmp69 * _tmp72);
  const Scalar _tmp110 = _tmp75 * _tmp77;
  const Scalar _tmp111 = _tmp110 * _tmp92 + _tmp91;
  const Scalar _tmp112 = _tmp110 * _tmp66 - _tmp111 * _tmp99 - _tmp64;
  const Scalar _tmp113 = _tmp109 * _tmp63 + _tmp55 * _tmp63 - _tmp59 * _tmp76;
  const Scalar _tmp114 = _tmp109 * _tmp85 - _tmp113 * _tmp88 + _tmp78 * _tmp85 - _tmp81 * _tmp86;
  const Scalar _tmp115 = _tmp101 * _tmp114;
  const Scalar _tmp116 = Scalar(1.0) / (_tmp114);
  const Scalar _tmp117 = _tmp100 * _tmp116;
  const Scalar _tmp118 = _tmp117 * (-_tmp109 + _tmp110 * _tmp113 - _tmp112 * _tmp115);
  const Scalar _tmp119 = _tmp112 + _tmp118;
  const Scalar _tmp120 = _tmp101 * _tmp87;
  const Scalar _tmp121 = -_tmp119 * _tmp120 - _tmp75;
  const Scalar _tmp122 = _tmp63 * _tmp77;
  const Scalar _tmp123 = _tmp101 * _tmp119;
  const Scalar _tmp124 = _tmp29 + Scalar(-4.8333311099999996);
  const Scalar _tmp125 = _tmp40 + Scalar(1.79662371);
  const Scalar _tmp126 =
      std::pow(Scalar(std::pow(_tmp124, Scalar(2)) + std::pow(_tmp125, Scalar(2))),
               Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp127 = _tmp125 * _tmp126;
  const Scalar _tmp128 = _tmp107 * _tmp127 * (_tmp121 * _tmp122 + _tmp123 * _tmp85 + Scalar(1.0));
  const Scalar _tmp129 = _tmp124 * _tmp126;
  const Scalar _tmp130 = -_tmp127 * _tmp28 + _tmp129 * _tmp39;
  const Scalar _tmp131 = Scalar(1.0) * _tmp116;
  const Scalar _tmp132 = _tmp107 * _tmp130 * (-_tmp105 * _tmp131 + _tmp131 * _tmp85);
  const Scalar _tmp133 = Scalar(1.0) * _tmp77;
  const Scalar _tmp134 = Scalar(1.0) * _tmp96;
  const Scalar _tmp135 = -_tmp133 * _tmp66 + _tmp134 * _tmp77 * _tmp92 * _tmp98;
  const Scalar _tmp136 = _tmp117 * (-_tmp113 * _tmp133 - _tmp115 * _tmp135);
  const Scalar _tmp137 = _tmp135 + _tmp136;
  const Scalar _tmp138 = -_tmp120 * _tmp137 + Scalar(1.0);
  const Scalar _tmp139 = _tmp101 * _tmp137;
  const Scalar _tmp140 = _tmp107 * _tmp129 * (_tmp122 * _tmp138 + _tmp139 * _tmp85);
  const Scalar _tmp141 = -_tmp107 * _tmp47 * (-_tmp104 * _tmp105 + _tmp104 * _tmp85) -
                         _tmp128 * fh1 - _tmp132 * fh1 - _tmp140 * fh1;
  const Scalar _tmp142 = std::pow(_tmp141, Scalar(-2));
  const Scalar _tmp143 = -_tmp128 - _tmp132 - _tmp140;
  const Scalar _tmp144 = _tmp142 * _tmp143;
  const Scalar _tmp145 = _tmp49 + _tmp89;
  const Scalar _tmp146 = _tmp145 * fh1;
  const Scalar _tmp147 = _tmp127 * _tmp146 + Scalar(3.29616) * _tmp35 + _tmp39 * fv1;
  const Scalar _tmp148 = _tmp78 + _tmp95;
  const Scalar _tmp149 = _tmp148 * _tmp99;
  const Scalar _tmp150 = Scalar(1.0) / (-_tmp149 - _tmp81 + _tmp97);
  const Scalar _tmp151 = Scalar(1.0) * _tmp150;
  const Scalar _tmp152 = _tmp148 * _tmp151;
  const Scalar _tmp153 = _tmp152 * _tmp96;
  const Scalar _tmp154 = -Scalar(1.0) * _tmp151 + Scalar(1.0) * _tmp153;
  const Scalar _tmp155 = _tmp148 * _tmp150;
  const Scalar _tmp156 = _tmp101 * _tmp93;
  const Scalar _tmp157 = _tmp111 + _tmp118 * _tmp155 - _tmp119 * _tmp156;
  const Scalar _tmp158 = Scalar(1.0) * _tmp127 * (_tmp118 * _tmp151 - _tmp134 * _tmp157);
  const Scalar _tmp159 = _tmp96 * (_tmp117 * _tmp152 - _tmp131 * _tmp93);
  const Scalar _tmp160 = _tmp117 * _tmp151;
  const Scalar _tmp161 = Scalar(1.0) * _tmp130;
  const Scalar _tmp162 = _tmp161 * (-Scalar(1.0) * _tmp159 + _tmp160);
  const Scalar _tmp163 = _tmp102 * _tmp150;
  const Scalar _tmp164 = -_tmp103 * _tmp156 - _tmp148 * _tmp163 + _tmp95;
  const Scalar _tmp165 = -_tmp129 * _tmp146 - Scalar(3.29616) * _tmp22 - _tmp28 * fv1;
  const Scalar _tmp166 = _tmp149 * _tmp151 + Scalar(1.0);
  const Scalar _tmp167 = _tmp151 * _tmp99;
  const Scalar _tmp168 = -Scalar(1.0) * _tmp134 * _tmp166 + Scalar(1.0) * _tmp167;
  const Scalar _tmp169 = -_tmp133 * _tmp92 + _tmp136 * _tmp155 - _tmp137 * _tmp156;
  const Scalar _tmp170 = Scalar(1.0) * _tmp129 * (-_tmp134 * _tmp169 + _tmp136 * _tmp151);
  const Scalar _tmp171 =
      _tmp147 * _tmp154 + _tmp158 * fh1 + _tmp162 * fh1 + _tmp165 * _tmp168 + _tmp170 * fh1 +
      Scalar(1.0) * _tmp47 * (-_tmp102 * _tmp151 - _tmp134 * _tmp164 + Scalar(1.0));
  const Scalar _tmp172 = Scalar(1.0) / (_tmp141);
  const Scalar _tmp173 = std::asinh(_tmp171 * _tmp172);
  const Scalar _tmp174 = Scalar(1.0) * _tmp173;
  const Scalar _tmp175 = _tmp127 * _tmp145;
  const Scalar _tmp176 = _tmp129 * _tmp145;
  const Scalar _tmp177 = (-_tmp144 * _tmp171 + _tmp172 * (_tmp154 * _tmp175 + _tmp158 + _tmp162 -
                                                          _tmp168 * _tmp176 + _tmp170)) /
                         std::sqrt(Scalar(_tmp142 * std::pow(_tmp171, Scalar(2)) + 1));
  const Scalar _tmp178 = Scalar(9.6622558468725703) * _tmp141;
  const Scalar _tmp179 =
      -_tmp173 * _tmp178 -
      Scalar(8.3888750099999996) *
          std::sqrt(
              Scalar(Scalar(0.090254729040973036) *
                         std::pow(Scalar(1 - Scalar(0.39679052492160538) * _tmp68), Scalar(2)) +
                     std::pow(Scalar(-Scalar(0.11920549523123722) * _tmp73 - 1), Scalar(2))));
  const Scalar _tmp180 = Scalar(0.1034955) * _tmp172;
  const Scalar _tmp181 = _tmp179 * _tmp180;
  const Scalar _tmp182 = Scalar(9.6622558468725703) * _tmp143;
  const Scalar _tmp183 = _tmp129 * _tmp169 * _tmp96;
  const Scalar _tmp184 = _tmp166 * _tmp96;
  const Scalar _tmp185 = _tmp130 * _tmp159;
  const Scalar _tmp186 = _tmp147 * _tmp151;
  const Scalar _tmp187 = _tmp127 * _tmp157 * _tmp96;
  const Scalar _tmp188 = -_tmp148 * _tmp186 * _tmp96 + _tmp164 * _tmp47 * _tmp96 +
                         _tmp165 * _tmp184 + _tmp183 * fh1 + _tmp185 * fh1 + _tmp187 * fh1;
  const Scalar _tmp189 = _tmp104 * _tmp47;
  const Scalar _tmp190 = _tmp116 * _tmp161;
  const Scalar _tmp191 = _tmp190 * fh1;
  const Scalar _tmp192 = _tmp129 * _tmp138 * _tmp77;
  const Scalar _tmp193 = _tmp121 * _tmp127 * _tmp77;
  const Scalar _tmp194 = -_tmp189 * _tmp88 - _tmp191 * _tmp88 + _tmp192 * fh1 + _tmp193 * fh1;
  const Scalar _tmp195 = Scalar(1.0) / (_tmp194);
  const Scalar _tmp196 = std::asinh(_tmp188 * _tmp195);
  const Scalar _tmp197 = Scalar(9.6622558468725703) * _tmp194;
  const Scalar _tmp198 =
      -_tmp196 * _tmp197 -
      Scalar(8.3196563700000006) *
          std::sqrt(
              Scalar(std::pow(Scalar(-Scalar(0.12019727204189803) * _tmp56 - 1), Scalar(2)) +
                     Scalar(0.057067943376852184) *
                         std::pow(Scalar(-Scalar(0.50315118556004401) * _tmp60 - 1), Scalar(2))));
  const Scalar _tmp199 = Scalar(0.1034955) * _tmp195;
  const Scalar _tmp200 = _tmp198 * _tmp199;
  const Scalar _tmp201 = Scalar(1.0) * _tmp196;
  const Scalar _tmp202 = -_tmp190 * _tmp88 + _tmp192 + _tmp193;
  const Scalar _tmp203 = Scalar(9.6622558468725703) * _tmp202;
  const Scalar _tmp204 = std::pow(_tmp194, Scalar(-2));
  const Scalar _tmp205 = _tmp202 * _tmp204;
  const Scalar _tmp206 = (-_tmp188 * _tmp205 + _tmp195 * (-_tmp153 * _tmp175 - _tmp176 * _tmp184 +
                                                          _tmp183 + _tmp185 + _tmp187)) /
                         std::sqrt(Scalar(std::pow(_tmp188, Scalar(2)) * _tmp204 + 1));
  const Scalar _tmp207 = _tmp123 * _tmp127;
  const Scalar _tmp208 = _tmp129 * _tmp139;
  const Scalar _tmp209 = _tmp189 + _tmp191 + _tmp207 * fh1 + _tmp208 * fh1;
  const Scalar _tmp210 = Scalar(1.0) / (_tmp209);
  const Scalar _tmp211 = _tmp118 * _tmp127 * _tmp150;
  const Scalar _tmp212 = _tmp129 * _tmp136 * _tmp150;
  const Scalar _tmp213 = _tmp130 * _tmp160;
  const Scalar _tmp214 = _tmp163 * _tmp47 - _tmp165 * _tmp167 + _tmp186 - _tmp211 * fh1 -
                         _tmp212 * fh1 - _tmp213 * fh1;
  const Scalar _tmp215 = std::asinh(_tmp210 * _tmp214);
  const Scalar _tmp216 = Scalar(9.6622558468725703) * _tmp209;
  const Scalar _tmp217 =
      -_tmp215 * _tmp216 -
      Scalar(4.7752063900000001) *
          std::sqrt(
              Scalar(std::pow(Scalar(1 - Scalar(0.20941503221602112) * _tmp79), Scalar(2)) +
                     Scalar(0.32397683292140877) *
                         std::pow(Scalar(1 - Scalar(0.36791786395571047) * _tmp82), Scalar(2))));
  const Scalar _tmp218 = Scalar(0.1034955) * _tmp210;
  const Scalar _tmp219 = _tmp217 * _tmp218;
  const Scalar _tmp220 = Scalar(1.0) * _tmp215;
  const Scalar _tmp221 = _tmp190 + _tmp207 + _tmp208;
  const Scalar _tmp222 = Scalar(9.6622558468725703) * _tmp221;
  const Scalar _tmp223 = std::pow(_tmp209, Scalar(-2));
  const Scalar _tmp224 = _tmp221 * _tmp223;
  const Scalar _tmp225 =
      (_tmp210 * (_tmp151 * _tmp175 + _tmp167 * _tmp176 - _tmp211 - _tmp212 - _tmp213) -
       _tmp214 * _tmp224) /
      std::sqrt(Scalar(std::pow(_tmp214, Scalar(2)) * _tmp223 + 1));

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) = -Scalar(8.3700199099999999) * _tmp0 -
               Scalar(9.6622558468725703) * fh1 *
                   (-Scalar(1.0) * _tmp45 * _tmp46 * fv1 * std::sinh(_tmp44) -
                    Scalar(0.86625939559540499) * _tmp45 -
                    (-Scalar(0.1034955) * _tmp41 * _tmp45 +
                     _tmp42 * (Scalar(9.6622558468725703) * _tmp1 * _tmp46 - _tmp3)) *
                        std::sinh(_tmp43)) +
               Scalar(9.6622558468725703) * std::cosh(_tmp43) -
               Scalar(9.6622558468725703) * std::cosh(_tmp44);
  _res(1, 0) =
      -_tmp178 *
          (-Scalar(0.87653584775870996) * _tmp144 + Scalar(1.0) * _tmp177 * std::sinh(_tmp174) -
           (-Scalar(0.1034955) * _tmp144 * _tmp179 +
            _tmp180 * (-_tmp173 * _tmp182 - _tmp177 * _tmp178)) *
               std::sinh(_tmp181)) -
      _tmp182 * (Scalar(0.87653584775870996) * _tmp172 + std::cosh(_tmp174) - std::cosh(_tmp181));
  _res(2, 0) =
      -_tmp197 *
          (-Scalar(0.87679799772039002) * _tmp205 + Scalar(1.0) * _tmp206 * std::sinh(_tmp201) -
           (-Scalar(0.1034955) * _tmp198 * _tmp205 +
            _tmp199 * (-_tmp196 * _tmp203 - _tmp197 * _tmp206)) *
               std::sinh(_tmp200)) -
      _tmp203 * (Scalar(0.87679799772039002) * _tmp195 - std::cosh(_tmp200) + std::cosh(_tmp201));
  _res(3, 0) =
      -_tmp216 *
          (-Scalar(0.86565325453551001) * _tmp224 + Scalar(1.0) * _tmp225 * std::sinh(_tmp220) -
           (-Scalar(0.1034955) * _tmp217 * _tmp224 +
            _tmp218 * (-_tmp215 * _tmp222 - _tmp216 * _tmp225)) *
               std::sinh(_tmp219)) -
      _tmp222 * (Scalar(0.86565325453551001) * _tmp210 - std::cosh(_tmp219) + std::cosh(_tmp220));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym