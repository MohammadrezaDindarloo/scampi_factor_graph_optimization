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
 * Symbolic function: IK_residual_func_cost1_wrt_fh1_Nl14
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost1WrtFh1Nl14(
    const Scalar fh1, const Scalar fv1, const Scalar rx, const Scalar ry, const Scalar rz,
    const Scalar p_init0, const Scalar p_init1, const Scalar p_init2, const Scalar rot_init_x,
    const Scalar rot_init_y, const Scalar rot_init_z, const Scalar rot_init_w,
    const Scalar epsilon) {
  // Total ops: 665

  // Unused inputs
  (void)p_init2;
  (void)epsilon;

  // Input arrays

  // Intermediate terms (220)
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
  const Scalar _tmp9 = _tmp8 * ry;
  const Scalar _tmp10 = _tmp8 * rx;
  const Scalar _tmp11 = _tmp8 * rz;
  const Scalar _tmp12 =
      -_tmp10 * rot_init_y + _tmp11 * rot_init_w + _tmp7 * rot_init_z + _tmp9 * rot_init_x;
  const Scalar _tmp13 = -2 * std::pow(_tmp12, Scalar(2));
  const Scalar _tmp14 =
      _tmp10 * rot_init_w + _tmp11 * rot_init_y + _tmp7 * rot_init_x - _tmp9 * rot_init_z;
  const Scalar _tmp15 = 1 - 2 * std::pow(_tmp14, Scalar(2));
  const Scalar _tmp16 = Scalar(0.20999999999999999) * _tmp13 + Scalar(0.20999999999999999) * _tmp15;
  const Scalar _tmp17 =
      _tmp10 * rot_init_z - _tmp11 * rot_init_x + _tmp7 * rot_init_y + _tmp9 * rot_init_w;
  const Scalar _tmp18 = 2 * _tmp17;
  const Scalar _tmp19 = _tmp12 * _tmp18;
  const Scalar _tmp20 =
      -_tmp10 * rot_init_x - _tmp11 * rot_init_z + _tmp7 * rot_init_w - _tmp9 * rot_init_y;
  const Scalar _tmp21 = 2 * _tmp20;
  const Scalar _tmp22 = _tmp14 * _tmp21;
  const Scalar _tmp23 = _tmp19 - _tmp22;
  const Scalar _tmp24 = -Scalar(0.010999999999999999) * _tmp23;
  const Scalar _tmp25 = _tmp14 * _tmp18;
  const Scalar _tmp26 = _tmp12 * _tmp21;
  const Scalar _tmp27 = Scalar(0.20999999999999999) * _tmp25 + Scalar(0.20999999999999999) * _tmp26;
  const Scalar _tmp28 = _tmp24 + _tmp27;
  const Scalar _tmp29 = _tmp16 + _tmp28;
  const Scalar _tmp30 = _tmp29 + p_init1;
  const Scalar _tmp31 = Scalar(0.20999999999999999) * _tmp25 - Scalar(0.20999999999999999) * _tmp26;
  const Scalar _tmp32 = -2 * std::pow(_tmp17, Scalar(2));
  const Scalar _tmp33 = Scalar(0.20999999999999999) * _tmp13 +
                        Scalar(0.20999999999999999) * _tmp32 + Scalar(0.20999999999999999);
  const Scalar _tmp34 = 2 * _tmp12 * _tmp14;
  const Scalar _tmp35 = _tmp18 * _tmp20;
  const Scalar _tmp36 = _tmp34 + _tmp35;
  const Scalar _tmp37 = -Scalar(0.010999999999999999) * _tmp36;
  const Scalar _tmp38 = _tmp33 + _tmp37;
  const Scalar _tmp39 = _tmp31 + _tmp38;
  const Scalar _tmp40 = _tmp39 + p_init0;
  const Scalar _tmp41 =
      -Scalar(0.1034955) * _tmp4 * fh1 -
      Scalar(0.49421237293624504) *
          std::sqrt(
              Scalar(std::pow(Scalar(1 - Scalar(0.20941503221602112) * _tmp30), Scalar(2)) +
                     Scalar(0.32397683292140877) *
                         std::pow(Scalar(1 - Scalar(0.36791786395571047) * _tmp40), Scalar(2))));
  const Scalar _tmp42 =
      std::pow(Scalar(_tmp0 * std::pow(fv1, Scalar(2)) + 1), Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp43 = _tmp1 * _tmp41;
  const Scalar _tmp44 = Scalar(1.0) * _tmp3;
  const Scalar _tmp45 = _tmp30 + Scalar(-4.7752063900000001);
  const Scalar _tmp46 = _tmp40 + Scalar(-2.71799795);
  const Scalar _tmp47 = std::pow(Scalar(std::pow(_tmp45, Scalar(2)) + std::pow(_tmp46, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp48 = _tmp45 * _tmp47;
  const Scalar _tmp49 = _tmp46 * _tmp47;
  const Scalar _tmp50 = -_tmp29 * _tmp49 + _tmp39 * _tmp48;
  const Scalar _tmp51 = -_tmp31;
  const Scalar _tmp52 = _tmp38 + _tmp51;
  const Scalar _tmp53 = _tmp52 + p_init0;
  const Scalar _tmp54 = _tmp53 + Scalar(-2.5202214700000001);
  const Scalar _tmp55 = Scalar(1.0) / (_tmp54);
  const Scalar _tmp56 = -_tmp16;
  const Scalar _tmp57 = _tmp28 + _tmp56;
  const Scalar _tmp58 = _tmp57 + p_init1;
  const Scalar _tmp59 = _tmp58 + Scalar(8.3888750099999996);
  const Scalar _tmp60 = _tmp55 * _tmp59;
  const Scalar _tmp61 = _tmp24 - _tmp27;
  const Scalar _tmp62 = _tmp16 + _tmp61;
  const Scalar _tmp63 = _tmp62 + p_init1;
  const Scalar _tmp64 = _tmp63 + Scalar(-4.8333311099999996);
  const Scalar _tmp65 = -_tmp33 + _tmp37;
  const Scalar _tmp66 = _tmp31 + _tmp65;
  const Scalar _tmp67 = _tmp66 + p_init0;
  const Scalar _tmp68 = _tmp67 + Scalar(1.79662371);
  const Scalar _tmp69 = std::pow(Scalar(std::pow(_tmp64, Scalar(2)) + std::pow(_tmp68, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp70 = _tmp68 * _tmp69;
  const Scalar _tmp71 = _tmp64 * _tmp69;
  const Scalar _tmp72 = _tmp60 * _tmp70 - _tmp71;
  const Scalar _tmp73 = _tmp56 + _tmp61;
  const Scalar _tmp74 = _tmp73 + p_init1;
  const Scalar _tmp75 = _tmp74 + Scalar(8.3196563700000006);
  const Scalar _tmp76 = _tmp51 + _tmp65;
  const Scalar _tmp77 = _tmp76 + p_init0;
  const Scalar _tmp78 = _tmp77 + Scalar(1.9874742000000001);
  const Scalar _tmp79 = std::pow(Scalar(std::pow(_tmp75, Scalar(2)) + std::pow(_tmp78, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp80 = _tmp78 * _tmp79;
  const Scalar _tmp81 = _tmp75 * _tmp79;
  const Scalar _tmp82 = Scalar(1.0) / (_tmp60 * _tmp80 - _tmp81);
  const Scalar _tmp83 =
      std::sqrt(Scalar(std::pow(_tmp54, Scalar(2)) + std::pow(_tmp59, Scalar(2))));
  const Scalar _tmp84 = Scalar(1.0) / (_tmp83);
  const Scalar _tmp85 = _tmp55 * _tmp83;
  const Scalar _tmp86 = _tmp85 * (_tmp52 * _tmp59 * _tmp84 - _tmp54 * _tmp57 * _tmp84);
  const Scalar _tmp87 = _tmp82 * (_tmp73 * _tmp80 - _tmp76 * _tmp81 + _tmp80 * _tmp86);
  const Scalar _tmp88 = _tmp62 * _tmp70 - _tmp66 * _tmp71 + _tmp70 * _tmp86 - _tmp72 * _tmp87;
  const Scalar _tmp89 = Scalar(1.0) / (_tmp88);
  const Scalar _tmp90 = Scalar(1.0) * _tmp89;
  const Scalar _tmp91 = _tmp72 * _tmp82;
  const Scalar _tmp92 = _tmp80 * _tmp91;
  const Scalar _tmp93 = _tmp50 * _tmp85 * (_tmp70 * _tmp90 - _tmp90 * _tmp92);
  const Scalar _tmp94 = Scalar(0.20999999999999999) * _tmp19 + Scalar(0.20999999999999999) * _tmp22;
  const Scalar _tmp95 = -_tmp94;
  const Scalar _tmp96 =
      -Scalar(0.010999999999999999) * _tmp15 - Scalar(0.010999999999999999) * _tmp32;
  const Scalar _tmp97 = Scalar(0.20999999999999999) * _tmp34 - Scalar(0.20999999999999999) * _tmp35;
  const Scalar _tmp98 = _tmp96 + _tmp97;
  const Scalar _tmp99 = _tmp95 + _tmp98;
  const Scalar _tmp100 = _tmp70 * _tmp99;
  const Scalar _tmp101 = _tmp96 - _tmp97;
  const Scalar _tmp102 = _tmp101 + _tmp95;
  const Scalar _tmp103 = _tmp80 * _tmp99;
  const Scalar _tmp104 = _tmp102 * _tmp81 - _tmp103 * _tmp60;
  const Scalar _tmp105 = _tmp101 + _tmp94;
  const Scalar _tmp106 = -_tmp100 * _tmp60 - _tmp104 * _tmp91 + _tmp105 * _tmp71;
  const Scalar _tmp107 = Scalar(1.0) * _tmp57;
  const Scalar _tmp108 = -_tmp107;
  const Scalar _tmp109 = Scalar(1.0) / (_tmp108 + _tmp73);
  const Scalar _tmp110 = Scalar(1.0) * _tmp52;
  const Scalar _tmp111 = _tmp110 - _tmp76;
  const Scalar _tmp112 = _tmp109 * _tmp111;
  const Scalar _tmp113 = -_tmp102 * _tmp80 + _tmp103;
  const Scalar _tmp114 = _tmp100 - _tmp105 * _tmp70 - _tmp106 * _tmp112 - _tmp113 * _tmp91;
  const Scalar _tmp115 = Scalar(1.0) / (_tmp114);
  const Scalar _tmp116 = _tmp60 * _tmp82;
  const Scalar _tmp117 = _tmp104 * _tmp116 + _tmp60 * _tmp99;
  const Scalar _tmp118 = -_tmp112 * _tmp117 + _tmp113 * _tmp116 - _tmp99;
  const Scalar _tmp119 = _tmp115 * _tmp88;
  const Scalar _tmp120 = _tmp114 * _tmp89;
  const Scalar _tmp121 = _tmp120 * (-_tmp118 * _tmp119 + _tmp60 * _tmp87 - _tmp86);
  const Scalar _tmp122 = _tmp115 * (_tmp118 + _tmp121);
  const Scalar _tmp123 = -_tmp122 * _tmp72 - _tmp60;
  const Scalar _tmp124 = _tmp80 * _tmp82;
  const Scalar _tmp125 = _tmp49 * _tmp85 * (_tmp122 * _tmp70 + _tmp123 * _tmp124 + Scalar(1.0));
  const Scalar _tmp126 = Scalar(1.0) * _tmp82;
  const Scalar _tmp127 = Scalar(1.0) * _tmp109;
  const Scalar _tmp128 = _tmp104 * _tmp111 * _tmp127 * _tmp82 - _tmp113 * _tmp126;
  const Scalar _tmp129 = _tmp120 * (-_tmp119 * _tmp128 - Scalar(1.0) * _tmp87);
  const Scalar _tmp130 = _tmp115 * (_tmp128 + _tmp129);
  const Scalar _tmp131 = -_tmp130 * _tmp72 + Scalar(1.0);
  const Scalar _tmp132 = _tmp48 * _tmp85 * (_tmp124 * _tmp131 + _tmp130 * _tmp70);
  const Scalar _tmp133 = -_tmp125 - _tmp132 - _tmp93;
  const Scalar _tmp134 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp135 = _tmp107 * _tmp112 + _tmp110;
  const Scalar _tmp136 = 0;
  const Scalar _tmp137 = -_tmp125 * fh1 - _tmp132 * fh1 -
                         _tmp134 * _tmp85 * (_tmp136 * _tmp70 - _tmp136 * _tmp92) - _tmp93 * fh1;
  const Scalar _tmp138 = Scalar(1.0) / (_tmp137);
  const Scalar _tmp139 = _tmp94 + _tmp98;
  const Scalar _tmp140 = _tmp139 * fh1;
  const Scalar _tmp141 = -_tmp140 * _tmp48 - Scalar(3.29616) * _tmp23 - _tmp29 * fv1;
  const Scalar _tmp142 = _tmp108 + _tmp62;
  const Scalar _tmp143 = _tmp112 * _tmp142;
  const Scalar _tmp144 = Scalar(1.0) / (_tmp110 - _tmp143 - _tmp66);
  const Scalar _tmp145 = Scalar(1.0) * _tmp144;
  const Scalar _tmp146 = _tmp143 * _tmp145 + Scalar(1.0);
  const Scalar _tmp147 = _tmp112 * _tmp145;
  const Scalar _tmp148 = -Scalar(1.0) * _tmp127 * _tmp146 + Scalar(1.0) * _tmp147;
  const Scalar _tmp149 = _tmp142 * _tmp144;
  const Scalar _tmp150 = -_tmp104 * _tmp126 - _tmp106 * _tmp130 + _tmp129 * _tmp149;
  const Scalar _tmp151 = Scalar(1.0) * _tmp48 * (-_tmp127 * _tmp150 + _tmp129 * _tmp145);
  const Scalar _tmp152 = _tmp140 * _tmp49 + Scalar(3.29616) * _tmp36 + _tmp39 * fv1;
  const Scalar _tmp153 = _tmp109 * _tmp142;
  const Scalar _tmp154 = _tmp145 * _tmp153;
  const Scalar _tmp155 = -Scalar(1.0) * _tmp145 + Scalar(1.0) * _tmp154;
  const Scalar _tmp156 = _tmp135 * _tmp144;
  const Scalar _tmp157 = _tmp109 * (-_tmp106 * _tmp136 + _tmp108 - _tmp142 * _tmp156);
  const Scalar _tmp158 = _tmp120 * _tmp145;
  const Scalar _tmp159 = _tmp109 * (-_tmp106 * _tmp90 + _tmp142 * _tmp158);
  const Scalar _tmp160 = Scalar(1.0) * _tmp50;
  const Scalar _tmp161 = _tmp160 * (_tmp158 - Scalar(1.0) * _tmp159);
  const Scalar _tmp162 = -_tmp106 * _tmp122 + _tmp117 + _tmp121 * _tmp149;
  const Scalar _tmp163 = Scalar(1.0) * _tmp49 * (_tmp121 * _tmp145 - _tmp127 * _tmp162);
  const Scalar _tmp164 =
      Scalar(1.0) * _tmp134 * (-_tmp135 * _tmp145 - Scalar(1.0) * _tmp157 + Scalar(1.0)) +
      _tmp141 * _tmp148 + _tmp151 * fh1 + _tmp152 * _tmp155 + _tmp161 * fh1 + _tmp163 * fh1;
  const Scalar _tmp165 = std::asinh(_tmp138 * _tmp164);
  const Scalar _tmp166 = Scalar(9.6622558468725703) * _tmp165;
  const Scalar _tmp167 =
      -_tmp137 * _tmp166 -
      Scalar(8.3888750099999996) *
          std::sqrt(
              Scalar(Scalar(0.090254729040973036) *
                         std::pow(Scalar(1 - Scalar(0.39679052492160538) * _tmp53), Scalar(2)) +
                     std::pow(Scalar(-Scalar(0.11920549523123722) * _tmp58 - 1), Scalar(2))));
  const Scalar _tmp168 = Scalar(0.1034955) * _tmp138;
  const Scalar _tmp169 = _tmp167 * _tmp168;
  const Scalar _tmp170 = Scalar(1.0) * _tmp165;
  const Scalar _tmp171 = std::pow(_tmp137, Scalar(-2));
  const Scalar _tmp172 = _tmp139 * _tmp49;
  const Scalar _tmp173 = _tmp139 * _tmp48;
  const Scalar _tmp174 = _tmp133 * _tmp171;
  const Scalar _tmp175 =
      (_tmp138 * (-_tmp148 * _tmp173 + _tmp151 + _tmp155 * _tmp172 + _tmp161 + _tmp163) -
       _tmp164 * _tmp174) /
      std::sqrt(Scalar(std::pow(_tmp164, Scalar(2)) * _tmp171 + 1));
  const Scalar _tmp176 = Scalar(9.6622558468725703) * _tmp137;
  const Scalar _tmp177 = _tmp131 * _tmp48 * _tmp82;
  const Scalar _tmp178 = _tmp134 * _tmp136;
  const Scalar _tmp179 = _tmp160 * _tmp89;
  const Scalar _tmp180 = _tmp179 * fh1;
  const Scalar _tmp181 = _tmp123 * _tmp49 * _tmp82;
  const Scalar _tmp182 = _tmp177 * fh1 - _tmp178 * _tmp91 - _tmp180 * _tmp91 + _tmp181 * fh1;
  const Scalar _tmp183 = Scalar(1.0) / (_tmp182);
  const Scalar _tmp184 = _tmp109 * _tmp150 * _tmp48;
  const Scalar _tmp185 = _tmp145 * _tmp152;
  const Scalar _tmp186 = _tmp159 * _tmp50;
  const Scalar _tmp187 = _tmp109 * _tmp146;
  const Scalar _tmp188 = _tmp109 * _tmp162 * _tmp49;
  const Scalar _tmp189 = _tmp134 * _tmp157 + _tmp141 * _tmp187 - _tmp153 * _tmp185 + _tmp184 * fh1 +
                         _tmp186 * fh1 + _tmp188 * fh1;
  const Scalar _tmp190 = std::asinh(_tmp183 * _tmp189);
  const Scalar _tmp191 = Scalar(9.6622558468725703) * _tmp182;
  const Scalar _tmp192 =
      -_tmp190 * _tmp191 -
      Scalar(8.3196563700000006) *
          std::sqrt(
              Scalar(std::pow(Scalar(-Scalar(0.12019727204189803) * _tmp74 - 1), Scalar(2)) +
                     Scalar(0.057067943376852184) *
                         std::pow(Scalar(-Scalar(0.50315118556004401) * _tmp77 - 1), Scalar(2))));
  const Scalar _tmp193 = Scalar(0.1034955) * _tmp183;
  const Scalar _tmp194 = _tmp192 * _tmp193;
  const Scalar _tmp195 = _tmp177 - _tmp179 * _tmp91 + _tmp181;
  const Scalar _tmp196 = Scalar(9.6622558468725703) * _tmp195;
  const Scalar _tmp197 = std::pow(_tmp182, Scalar(-2));
  const Scalar _tmp198 = _tmp195 * _tmp197;
  const Scalar _tmp199 =
      (_tmp183 * (-_tmp154 * _tmp172 - _tmp173 * _tmp187 + _tmp184 + _tmp186 + _tmp188) -
       _tmp189 * _tmp198) /
      std::sqrt(Scalar(std::pow(_tmp189, Scalar(2)) * _tmp197 + 1));
  const Scalar _tmp200 = Scalar(1.0) * _tmp190;
  const Scalar _tmp201 = _tmp121 * _tmp144 * _tmp49;
  const Scalar _tmp202 = _tmp129 * _tmp144 * _tmp48;
  const Scalar _tmp203 = _tmp158 * _tmp50;
  const Scalar _tmp204 = _tmp134 * _tmp156 - _tmp141 * _tmp147 + _tmp185 - _tmp201 * fh1 -
                         _tmp202 * fh1 - _tmp203 * fh1;
  const Scalar _tmp205 = _tmp122 * _tmp49;
  const Scalar _tmp206 = _tmp130 * _tmp48;
  const Scalar _tmp207 = _tmp178 + _tmp180 + _tmp205 * fh1 + _tmp206 * fh1;
  const Scalar _tmp208 = Scalar(1.0) / (_tmp207);
  const Scalar _tmp209 = std::asinh(_tmp204 * _tmp208);
  const Scalar _tmp210 = Scalar(9.6622558468725703) * _tmp207;
  const Scalar _tmp211 =
      -_tmp209 * _tmp210 -
      Scalar(4.8333311099999996) *
          std::sqrt(
              Scalar(std::pow(Scalar(1 - Scalar(0.20689664689659551) * _tmp63), Scalar(2)) +
                     Scalar(0.13817235445745474) *
                         std::pow(Scalar(-Scalar(0.55659957866191134) * _tmp67 - 1), Scalar(2))));
  const Scalar _tmp212 = Scalar(0.1034955) * _tmp208;
  const Scalar _tmp213 = _tmp211 * _tmp212;
  const Scalar _tmp214 = Scalar(1.0) * _tmp209;
  const Scalar _tmp215 = _tmp179 + _tmp205 + _tmp206;
  const Scalar _tmp216 = Scalar(9.6622558468725703) * _tmp215;
  const Scalar _tmp217 = std::pow(_tmp207, Scalar(-2));
  const Scalar _tmp218 = _tmp215 * _tmp217;
  const Scalar _tmp219 = (-_tmp204 * _tmp218 + _tmp208 * (_tmp145 * _tmp172 + _tmp147 * _tmp173 -
                                                          _tmp201 - _tmp202 - _tmp203)) /
                         std::sqrt(Scalar(std::pow(_tmp204, Scalar(2)) * _tmp217 + 1));

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) = -Scalar(8.3641632200000018) * _tmp1 -
               Scalar(9.6622558468725703) * fh1 *
                   (-Scalar(1.0) * _tmp0 * _tmp42 * fv1 * std::sinh(_tmp44) -
                    Scalar(0.86565325453551001) * _tmp0 -
                    (-_tmp0 * _tmp41 + Scalar(0.1034955) * _tmp1 *
                                           (Scalar(9.6622558468725703) * _tmp2 * _tmp42 - _tmp4)) *
                        std::sinh(_tmp43)) +
               Scalar(9.6622558468725703) * std::cosh(_tmp43) -
               Scalar(9.6622558468725703) * std::cosh(_tmp44);
  _res(1, 0) =
      -Scalar(9.6622558468725703) * _tmp133 *
          (Scalar(0.87653584775870996) * _tmp138 - std::cosh(_tmp169) + std::cosh(_tmp170)) -
      _tmp176 *
          (-Scalar(0.87653584775870996) * _tmp174 + Scalar(1.0) * _tmp175 * std::sinh(_tmp170) -
           (-Scalar(0.1034955) * _tmp167 * _tmp174 +
            _tmp168 * (-_tmp133 * _tmp166 - _tmp175 * _tmp176)) *
               std::sinh(_tmp169));
  _res(2, 0) =
      -_tmp191 *
          (-Scalar(0.87679799772039002) * _tmp198 + Scalar(1.0) * _tmp199 * std::sinh(_tmp200) -
           (-Scalar(0.1034955) * _tmp192 * _tmp198 +
            _tmp193 * (-_tmp190 * _tmp196 - _tmp191 * _tmp199)) *
               std::sinh(_tmp194)) -
      _tmp196 * (Scalar(0.87679799772039002) * _tmp183 - std::cosh(_tmp194) + std::cosh(_tmp200));
  _res(3, 0) =
      -_tmp210 *
          (-Scalar(0.86625939559540499) * _tmp218 + Scalar(1.0) * _tmp219 * std::sinh(_tmp214) -
           (-Scalar(0.1034955) * _tmp211 * _tmp218 +
            _tmp212 * (-_tmp209 * _tmp216 - _tmp210 * _tmp219)) *
               std::sinh(_tmp213)) -
      _tmp216 * (Scalar(0.86625939559540499) * _tmp208 - std::cosh(_tmp213) + std::cosh(_tmp214));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym