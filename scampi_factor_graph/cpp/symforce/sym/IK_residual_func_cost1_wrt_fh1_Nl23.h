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
 * Symbolic function: IK_residual_func_cost1_wrt_fh1_Nl23
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost1WrtFh1Nl23(
    const Scalar fh1, const Scalar fv1, const Scalar rx, const Scalar ry, const Scalar rz,
    const Scalar p_init0, const Scalar p_init1, const Scalar p_init2, const Scalar rot_init_x,
    const Scalar rot_init_y, const Scalar rot_init_z, const Scalar rot_init_w,
    const Scalar epsilon) {
  // Total ops: 671

  // Unused inputs
  (void)p_init2;
  (void)epsilon;

  // Input arrays

  // Intermediate terms (223)
  const Scalar _tmp0 = Scalar(1.0) / (fh1);
  const Scalar _tmp1 = _tmp0 * fv1;
  const Scalar _tmp2 = std::asinh(_tmp1);
  const Scalar _tmp3 = Scalar(9.6622558468725703) * _tmp2;
  const Scalar _tmp4 = std::sqrt(
      Scalar(std::pow(rx, Scalar(2)) + std::pow(ry, Scalar(2)) + std::pow(rz, Scalar(2))));
  const Scalar _tmp5 = (Scalar(1) / Scalar(2)) * _tmp4;
  const Scalar _tmp6 = std::cos(_tmp5);
  const Scalar _tmp7 = std::sin(_tmp5) / _tmp4;
  const Scalar _tmp8 = _tmp7 * rot_init_w;
  const Scalar _tmp9 = _tmp7 * rx;
  const Scalar _tmp10 = _tmp7 * rz;
  const Scalar _tmp11 = -_tmp10 * rot_init_x + _tmp6 * rot_init_y + _tmp8 * ry + _tmp9 * rot_init_z;
  const Scalar _tmp12 = _tmp7 * ry;
  const Scalar _tmp13 = _tmp10 * rot_init_y - _tmp12 * rot_init_z + _tmp6 * rot_init_x + _tmp8 * rx;
  const Scalar _tmp14 = 2 * _tmp11 * _tmp13;
  const Scalar _tmp15 = _tmp12 * rot_init_x + _tmp6 * rot_init_z + _tmp8 * rz - _tmp9 * rot_init_y;
  const Scalar _tmp16 = -2 * _tmp10 * rot_init_z - 2 * _tmp12 * rot_init_y +
                        2 * _tmp6 * rot_init_w - 2 * _tmp9 * rot_init_x;
  const Scalar _tmp17 = _tmp15 * _tmp16;
  const Scalar _tmp18 = Scalar(0.20999999999999999) * _tmp14 + Scalar(0.20999999999999999) * _tmp17;
  const Scalar _tmp19 = -_tmp18;
  const Scalar _tmp20 = 2 * _tmp15;
  const Scalar _tmp21 = _tmp11 * _tmp20;
  const Scalar _tmp22 = _tmp13 * _tmp16;
  const Scalar _tmp23 = _tmp21 - _tmp22;
  const Scalar _tmp24 = -Scalar(0.010999999999999999) * _tmp23;
  const Scalar _tmp25 = -2 * std::pow(_tmp13, Scalar(2));
  const Scalar _tmp26 = -2 * std::pow(_tmp15, Scalar(2));
  const Scalar _tmp27 = Scalar(0.20999999999999999) * _tmp25 +
                        Scalar(0.20999999999999999) * _tmp26 + Scalar(0.20999999999999999);
  const Scalar _tmp28 = _tmp24 + _tmp27;
  const Scalar _tmp29 = _tmp19 + _tmp28;
  const Scalar _tmp30 = _tmp29 + p_init1;
  const Scalar _tmp31 = Scalar(0.20999999999999999) * _tmp14 - Scalar(0.20999999999999999) * _tmp17;
  const Scalar _tmp32 = 1 - 2 * std::pow(_tmp11, Scalar(2));
  const Scalar _tmp33 = Scalar(0.20999999999999999) * _tmp26 + Scalar(0.20999999999999999) * _tmp32;
  const Scalar _tmp34 = _tmp13 * _tmp20;
  const Scalar _tmp35 = _tmp11 * _tmp16;
  const Scalar _tmp36 = _tmp34 + _tmp35;
  const Scalar _tmp37 = -Scalar(0.010999999999999999) * _tmp36;
  const Scalar _tmp38 = -_tmp33 + _tmp37;
  const Scalar _tmp39 = _tmp31 + _tmp38;
  const Scalar _tmp40 = _tmp39 + p_init0;
  const Scalar _tmp41 =
      -Scalar(0.1034955) * _tmp3 * fh1 -
      Scalar(0.50022801989500498) *
          std::sqrt(
              Scalar(std::pow(Scalar(1 - Scalar(0.20689664689659551) * _tmp30), Scalar(2)) +
                     Scalar(0.13817235445745474) *
                         std::pow(Scalar(-Scalar(0.55659957866191134) * _tmp40 - 1), Scalar(2))));
  const Scalar _tmp42 = _tmp0 * _tmp41;
  const Scalar _tmp43 = Scalar(1.0) * _tmp2;
  const Scalar _tmp44 = std::pow(fh1, Scalar(-2));
  const Scalar _tmp45 =
      std::pow(Scalar(_tmp44 * std::pow(fv1, Scalar(2)) + 1), Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp46 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp47 = _tmp24 - _tmp27;
  const Scalar _tmp48 = _tmp19 + _tmp47;
  const Scalar _tmp49 = _tmp48 + p_init1;
  const Scalar _tmp50 = _tmp49 + Scalar(8.3196563700000006);
  const Scalar _tmp51 = -_tmp31;
  const Scalar _tmp52 = _tmp38 + _tmp51;
  const Scalar _tmp53 = _tmp52 + p_init0;
  const Scalar _tmp54 = _tmp53 + Scalar(1.9874742000000001);
  const Scalar _tmp55 = std::pow(Scalar(std::pow(_tmp50, Scalar(2)) + std::pow(_tmp54, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp56 = _tmp54 * _tmp55;
  const Scalar _tmp57 = Scalar(0.20999999999999999) * _tmp21 + Scalar(0.20999999999999999) * _tmp22;
  const Scalar _tmp58 = -_tmp57;
  const Scalar _tmp59 =
      -Scalar(0.010999999999999999) * _tmp25 - Scalar(0.010999999999999999) * _tmp32;
  const Scalar _tmp60 = Scalar(0.20999999999999999) * _tmp34 - Scalar(0.20999999999999999) * _tmp35;
  const Scalar _tmp61 = _tmp59 + _tmp60;
  const Scalar _tmp62 = _tmp58 + _tmp61;
  const Scalar _tmp63 = _tmp33 + _tmp37;
  const Scalar _tmp64 = _tmp51 + _tmp63;
  const Scalar _tmp65 = _tmp64 + p_init0;
  const Scalar _tmp66 = _tmp65 + Scalar(-2.5202214700000001);
  const Scalar _tmp67 = _tmp18 + _tmp47;
  const Scalar _tmp68 = _tmp67 + p_init1;
  const Scalar _tmp69 = _tmp68 + Scalar(8.3888750099999996);
  const Scalar _tmp70 = std::pow(Scalar(std::pow(_tmp66, Scalar(2)) + std::pow(_tmp69, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp71 = _tmp66 * _tmp70;
  const Scalar _tmp72 = _tmp57 + _tmp61;
  const Scalar _tmp73 = _tmp71 * _tmp72;
  const Scalar _tmp74 = -_tmp62 * _tmp71 + _tmp73;
  const Scalar _tmp75 = _tmp50 * _tmp55;
  const Scalar _tmp76 = _tmp31 + _tmp63;
  const Scalar _tmp77 = _tmp76 + p_init0;
  const Scalar _tmp78 = _tmp77 + Scalar(-2.71799795);
  const Scalar _tmp79 = Scalar(1.0) / (_tmp78);
  const Scalar _tmp80 = _tmp18 + _tmp28;
  const Scalar _tmp81 = _tmp80 + p_init1;
  const Scalar _tmp82 = _tmp81 + Scalar(-4.7752063900000001);
  const Scalar _tmp83 = _tmp79 * _tmp82;
  const Scalar _tmp84 = _tmp56 * _tmp83 - _tmp75;
  const Scalar _tmp85 = _tmp69 * _tmp70;
  const Scalar _tmp86 = Scalar(1.0) / (_tmp71 * _tmp83 - _tmp85);
  const Scalar _tmp87 = _tmp84 * _tmp86;
  const Scalar _tmp88 = _tmp72 * _tmp83;
  const Scalar _tmp89 = _tmp62 * _tmp85 - _tmp73 * _tmp83;
  const Scalar _tmp90 = _tmp59 - _tmp60;
  const Scalar _tmp91 = _tmp58 + _tmp90;
  const Scalar _tmp92 = -_tmp56 * _tmp88 + _tmp75 * _tmp91 - _tmp87 * _tmp89;
  const Scalar _tmp93 = Scalar(1.0) * _tmp80;
  const Scalar _tmp94 = -_tmp93;
  const Scalar _tmp95 = Scalar(1.0) / (_tmp67 + _tmp94);
  const Scalar _tmp96 = Scalar(1.0) * _tmp76;
  const Scalar _tmp97 = -_tmp64 + _tmp96;
  const Scalar _tmp98 = _tmp95 * _tmp97;
  const Scalar _tmp99 = _tmp56 * _tmp72 - _tmp56 * _tmp91 - _tmp74 * _tmp87 - _tmp92 * _tmp98;
  const Scalar _tmp100 = Scalar(1.0) / (_tmp99);
  const Scalar _tmp101 = _tmp93 * _tmp98 + _tmp96;
  const Scalar _tmp102 = 0;
  const Scalar _tmp103 = _tmp100 * _tmp102;
  const Scalar _tmp104 = _tmp71 * _tmp87;
  const Scalar _tmp105 =
      std::sqrt(Scalar(std::pow(_tmp78, Scalar(2)) + std::pow(_tmp82, Scalar(2))));
  const Scalar _tmp106 = _tmp105 * _tmp79;
  const Scalar _tmp107 = _tmp30 + Scalar(-4.8333311099999996);
  const Scalar _tmp108 = _tmp40 + Scalar(1.79662371);
  const Scalar _tmp109 =
      std::pow(Scalar(std::pow(_tmp107, Scalar(2)) + std::pow(_tmp108, Scalar(2))),
               Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp110 = _tmp107 * _tmp109;
  const Scalar _tmp111 = _tmp108 * _tmp109;
  const Scalar _tmp112 = _tmp110 * _tmp39 - _tmp111 * _tmp29;
  const Scalar _tmp113 = Scalar(1.0) / (_tmp105);
  const Scalar _tmp114 = _tmp106 * (_tmp113 * _tmp76 * _tmp82 - _tmp113 * _tmp78 * _tmp80);
  const Scalar _tmp115 = _tmp114 * _tmp71 - _tmp64 * _tmp85 + _tmp67 * _tmp71;
  const Scalar _tmp116 = _tmp114 * _tmp56 - _tmp115 * _tmp87 + _tmp48 * _tmp56 - _tmp52 * _tmp75;
  const Scalar _tmp117 = Scalar(1.0) / (_tmp116);
  const Scalar _tmp118 = Scalar(1.0) * _tmp117;
  const Scalar _tmp119 = _tmp106 * _tmp112 * (-_tmp104 * _tmp118 + _tmp118 * _tmp56);
  const Scalar _tmp120 = _tmp83 * _tmp86;
  const Scalar _tmp121 = _tmp120 * _tmp89 + _tmp88;
  const Scalar _tmp122 = _tmp120 * _tmp74 - _tmp121 * _tmp98 - _tmp72;
  const Scalar _tmp123 = _tmp100 * _tmp116;
  const Scalar _tmp124 = _tmp117 * _tmp99;
  const Scalar _tmp125 = _tmp124 * (-_tmp114 + _tmp115 * _tmp120 - _tmp122 * _tmp123);
  const Scalar _tmp126 = _tmp122 + _tmp125;
  const Scalar _tmp127 = _tmp100 * _tmp126;
  const Scalar _tmp128 = _tmp100 * _tmp84;
  const Scalar _tmp129 = _tmp86 * (-_tmp126 * _tmp128 - _tmp83);
  const Scalar _tmp130 = _tmp106 * _tmp111 * (_tmp127 * _tmp56 + _tmp129 * _tmp71 + Scalar(1.0));
  const Scalar _tmp131 = Scalar(1.0) * _tmp86;
  const Scalar _tmp132 = Scalar(1.0) * _tmp95;
  const Scalar _tmp133 = -_tmp131 * _tmp74 + _tmp132 * _tmp86 * _tmp89 * _tmp97;
  const Scalar _tmp134 = _tmp124 * (-_tmp115 * _tmp131 - _tmp123 * _tmp133);
  const Scalar _tmp135 = _tmp133 + _tmp134;
  const Scalar _tmp136 = _tmp86 * (-_tmp128 * _tmp135 + Scalar(1.0));
  const Scalar _tmp137 = _tmp100 * _tmp135;
  const Scalar _tmp138 = _tmp106 * _tmp110 * (_tmp136 * _tmp71 + _tmp137 * _tmp56);
  const Scalar _tmp139 = -_tmp106 * _tmp46 * (-_tmp103 * _tmp104 + _tmp103 * _tmp56) -
                         _tmp119 * fh1 - _tmp130 * fh1 - _tmp138 * fh1;
  const Scalar _tmp140 = std::pow(_tmp139, Scalar(-2));
  const Scalar _tmp141 = -_tmp119 - _tmp130 - _tmp138;
  const Scalar _tmp142 = _tmp140 * _tmp141;
  const Scalar _tmp143 = Scalar(1.0) / (_tmp139);
  const Scalar _tmp144 = _tmp48 + _tmp94;
  const Scalar _tmp145 = _tmp144 * _tmp98;
  const Scalar _tmp146 = Scalar(1.0) / (-_tmp145 - _tmp52 + _tmp96);
  const Scalar _tmp147 = _tmp101 * _tmp146;
  const Scalar _tmp148 = _tmp100 * _tmp92;
  const Scalar _tmp149 = -_tmp102 * _tmp148 - _tmp144 * _tmp147 + _tmp94;
  const Scalar _tmp150 = Scalar(1.0) * _tmp146;
  const Scalar _tmp151 = _tmp144 * _tmp146;
  const Scalar _tmp152 = -_tmp131 * _tmp89 + _tmp134 * _tmp151 - _tmp135 * _tmp148;
  const Scalar _tmp153 = Scalar(1.0) * _tmp110 * (-_tmp132 * _tmp152 + _tmp134 * _tmp150);
  const Scalar _tmp154 = _tmp57 + _tmp90;
  const Scalar _tmp155 = _tmp154 * fh1;
  const Scalar _tmp156 = -_tmp110 * _tmp155 - Scalar(3.29616) * _tmp23 - _tmp29 * fv1;
  const Scalar _tmp157 = _tmp145 * _tmp150 + Scalar(1.0);
  const Scalar _tmp158 = _tmp150 * _tmp98;
  const Scalar _tmp159 = -Scalar(1.0) * _tmp132 * _tmp157 + Scalar(1.0) * _tmp158;
  const Scalar _tmp160 = _tmp111 * _tmp155 + Scalar(3.29616) * _tmp36 + _tmp39 * fv1;
  const Scalar _tmp161 = _tmp144 * _tmp150;
  const Scalar _tmp162 = -Scalar(1.0) * _tmp150 + Scalar(1.0) * _tmp161 * _tmp95;
  const Scalar _tmp163 = _tmp121 + _tmp125 * _tmp151 - _tmp126 * _tmp148;
  const Scalar _tmp164 = Scalar(1.0) * _tmp111 * (_tmp125 * _tmp150 - _tmp132 * _tmp163);
  const Scalar _tmp165 = -_tmp118 * _tmp92 + _tmp124 * _tmp161;
  const Scalar _tmp166 = _tmp124 * _tmp150;
  const Scalar _tmp167 = Scalar(1.0) * _tmp112 * (-_tmp132 * _tmp165 + _tmp166);
  const Scalar _tmp168 =
      _tmp153 * fh1 + _tmp156 * _tmp159 + _tmp160 * _tmp162 + _tmp164 * fh1 + _tmp167 * fh1 +
      Scalar(1.0) * _tmp46 * (-_tmp101 * _tmp150 - _tmp132 * _tmp149 + Scalar(1.0));
  const Scalar _tmp169 = std::asinh(_tmp143 * _tmp168);
  const Scalar _tmp170 = Scalar(9.6622558468725703) * _tmp141;
  const Scalar _tmp171 = Scalar(9.6622558468725703) * _tmp139;
  const Scalar _tmp172 = _tmp110 * _tmp154;
  const Scalar _tmp173 = _tmp111 * _tmp154;
  const Scalar _tmp174 = (-_tmp142 * _tmp168 + _tmp143 * (_tmp153 - _tmp159 * _tmp172 +
                                                          _tmp162 * _tmp173 + _tmp164 + _tmp167)) /
                         std::sqrt(Scalar(_tmp140 * std::pow(_tmp168, Scalar(2)) + 1));
  const Scalar _tmp175 = Scalar(0.1034955) * _tmp143;
  const Scalar _tmp176 =
      -_tmp169 * _tmp171 -
      Scalar(4.7752063900000001) *
          std::sqrt(
              Scalar(Scalar(0.32397683292140877) *
                         std::pow(Scalar(1 - Scalar(0.36791786395571047) * _tmp77), Scalar(2)) +
                     std::pow(Scalar(1 - Scalar(0.20941503221602112) * _tmp81), Scalar(2))));
  const Scalar _tmp177 = _tmp175 * _tmp176;
  const Scalar _tmp178 = Scalar(1.0) * _tmp169;
  const Scalar _tmp179 = _tmp112 * _tmp118;
  const Scalar _tmp180 = _tmp179 * fh1;
  const Scalar _tmp181 = _tmp103 * _tmp46;
  const Scalar _tmp182 = _tmp110 * _tmp136;
  const Scalar _tmp183 = _tmp111 * _tmp129;
  const Scalar _tmp184 = -_tmp180 * _tmp87 - _tmp181 * _tmp87 + _tmp182 * fh1 + _tmp183 * fh1;
  const Scalar _tmp185 = Scalar(1.0) / (_tmp184);
  const Scalar _tmp186 = _tmp112 * _tmp165 * _tmp95;
  const Scalar _tmp187 = _tmp157 * _tmp95;
  const Scalar _tmp188 = _tmp110 * _tmp152 * _tmp95;
  const Scalar _tmp189 = _tmp111 * _tmp95;
  const Scalar _tmp190 = _tmp163 * _tmp189;
  const Scalar _tmp191 = _tmp150 * _tmp160;
  const Scalar _tmp192 = -_tmp144 * _tmp191 * _tmp95 + _tmp149 * _tmp46 * _tmp95 +
                         _tmp156 * _tmp187 + _tmp186 * fh1 + _tmp188 * fh1 + _tmp190 * fh1;
  const Scalar _tmp193 = std::asinh(_tmp185 * _tmp192);
  const Scalar _tmp194 = Scalar(9.6622558468725703) * _tmp184;
  const Scalar _tmp195 =
      -_tmp193 * _tmp194 -
      Scalar(8.3888750099999996) *
          std::sqrt(
              Scalar(Scalar(0.090254729040973036) *
                         std::pow(Scalar(1 - Scalar(0.39679052492160538) * _tmp65), Scalar(2)) +
                     std::pow(Scalar(-Scalar(0.11920549523123722) * _tmp68 - 1), Scalar(2))));
  const Scalar _tmp196 = std::pow(_tmp184, Scalar(-2));
  const Scalar _tmp197 = -_tmp179 * _tmp87 + _tmp182 + _tmp183;
  const Scalar _tmp198 = _tmp196 * _tmp197;
  const Scalar _tmp199 = Scalar(9.6622558468725703) * _tmp197;
  const Scalar _tmp200 =
      (_tmp185 * (-_tmp154 * _tmp161 * _tmp189 - _tmp172 * _tmp187 + _tmp186 + _tmp188 + _tmp190) -
       _tmp192 * _tmp198) /
      std::sqrt(Scalar(std::pow(_tmp192, Scalar(2)) * _tmp196 + 1));
  const Scalar _tmp201 = Scalar(0.1034955) * _tmp185;
  const Scalar _tmp202 = _tmp195 * _tmp201;
  const Scalar _tmp203 = Scalar(1.0) * _tmp193;
  const Scalar _tmp204 = _tmp110 * _tmp137;
  const Scalar _tmp205 = _tmp111 * _tmp127;
  const Scalar _tmp206 = _tmp180 + _tmp181 + _tmp204 * fh1 + _tmp205 * fh1;
  const Scalar _tmp207 = std::pow(_tmp206, Scalar(-2));
  const Scalar _tmp208 = _tmp179 + _tmp204 + _tmp205;
  const Scalar _tmp209 = _tmp207 * _tmp208;
  const Scalar _tmp210 = Scalar(1.0) / (_tmp206);
  const Scalar _tmp211 = _tmp110 * _tmp134 * _tmp146;
  const Scalar _tmp212 = _tmp111 * _tmp125 * _tmp146;
  const Scalar _tmp213 = _tmp112 * _tmp166;
  const Scalar _tmp214 = _tmp147 * _tmp46 - _tmp156 * _tmp158 + _tmp191 - _tmp211 * fh1 -
                         _tmp212 * fh1 - _tmp213 * fh1;
  const Scalar _tmp215 = std::asinh(_tmp210 * _tmp214);
  const Scalar _tmp216 = Scalar(9.6622558468725703) * _tmp206;
  const Scalar _tmp217 =
      -_tmp215 * _tmp216 -
      Scalar(8.3196563700000006) *
          std::sqrt(
              Scalar(std::pow(Scalar(-Scalar(0.12019727204189803) * _tmp49 - 1), Scalar(2)) +
                     Scalar(0.057067943376852184) *
                         std::pow(Scalar(-Scalar(0.50315118556004401) * _tmp53 - 1), Scalar(2))));
  const Scalar _tmp218 = Scalar(9.6622558468725703) * _tmp208;
  const Scalar _tmp219 = (-_tmp209 * _tmp214 + _tmp210 * (_tmp150 * _tmp173 + _tmp158 * _tmp172 -
                                                          _tmp211 - _tmp212 - _tmp213)) /
                         std::sqrt(Scalar(_tmp207 * std::pow(_tmp214, Scalar(2)) + 1));
  const Scalar _tmp220 = Scalar(0.1034955) * _tmp210;
  const Scalar _tmp221 = _tmp217 * _tmp220;
  const Scalar _tmp222 = Scalar(1.0) * _tmp215;

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) =
      -Scalar(8.3700199099999999) * _tmp0 -
      Scalar(9.6622558468725703) * fh1 *
          (-Scalar(1.0) * _tmp44 * _tmp45 * fv1 * std::sinh(_tmp43) -
           Scalar(0.86625939559540499) * _tmp44 -
           (Scalar(0.1034955) * _tmp0 * (Scalar(9.6622558468725703) * _tmp1 * _tmp45 - _tmp3) -
            _tmp41 * _tmp44) *
               std::sinh(_tmp42)) +
      Scalar(9.6622558468725703) * std::cosh(_tmp42) -
      Scalar(9.6622558468725703) * std::cosh(_tmp43);
  _res(1, 0) =
      -_tmp170 * (Scalar(0.86565325453551001) * _tmp143 - std::cosh(_tmp177) + std::cosh(_tmp178)) -
      _tmp171 *
          (-Scalar(0.86565325453551001) * _tmp142 + Scalar(1.0) * _tmp174 * std::sinh(_tmp178) -
           (-Scalar(0.1034955) * _tmp142 * _tmp176 +
            _tmp175 * (-_tmp169 * _tmp170 - _tmp171 * _tmp174)) *
               std::sinh(_tmp177));
  _res(2, 0) =
      -_tmp194 *
          (-Scalar(0.87653584775870996) * _tmp198 + Scalar(1.0) * _tmp200 * std::sinh(_tmp203) -
           (-Scalar(0.1034955) * _tmp195 * _tmp198 +
            _tmp201 * (-_tmp193 * _tmp199 - _tmp194 * _tmp200)) *
               std::sinh(_tmp202)) -
      _tmp199 * (Scalar(0.87653584775870996) * _tmp185 - std::cosh(_tmp202) + std::cosh(_tmp203));
  _res(3, 0) =
      -_tmp216 *
          (-Scalar(0.87679799772039002) * _tmp209 + Scalar(1.0) * _tmp219 * std::sinh(_tmp222) -
           (-Scalar(0.1034955) * _tmp209 * _tmp217 +
            _tmp220 * (-_tmp215 * _tmp218 - _tmp216 * _tmp219)) *
               std::sinh(_tmp221)) -
      _tmp218 * (Scalar(0.87679799772039002) * _tmp210 - std::cosh(_tmp221) + std::cosh(_tmp222));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym