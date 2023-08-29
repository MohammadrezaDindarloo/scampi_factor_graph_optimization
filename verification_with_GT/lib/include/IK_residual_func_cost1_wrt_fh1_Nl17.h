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
 * Symbolic function: IK_residual_func_cost1_wrt_fh1_Nl17
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost1WrtFh1Nl17(
    const Scalar fh1, const Scalar fv1, const Scalar rx, const Scalar ry, const Scalar rz,
    const Scalar p_init0, const Scalar p_init1, const Scalar p_init2, const Scalar rot_init_x,
    const Scalar rot_init_y, const Scalar rot_init_z, const Scalar rot_init_w,
    const Scalar epsilon) {
  // Total ops: 667

  // Unused inputs
  (void)p_init2;
  (void)epsilon;

  // Input arrays

  // Intermediate terms (219)
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
  const Scalar _tmp10 = _tmp8 * rot_init_w;
  const Scalar _tmp11 = _tmp8 * rot_init_y;
  const Scalar _tmp12 = _tmp10 * rx + _tmp11 * rz + _tmp7 * rot_init_x - _tmp9 * rot_init_z;
  const Scalar _tmp13 = -2 * std::pow(_tmp12, Scalar(2));
  const Scalar _tmp14 = _tmp10 * rz - _tmp11 * rx + _tmp7 * rot_init_z + _tmp9 * rot_init_x;
  const Scalar _tmp15 = -2 * std::pow(_tmp14, Scalar(2));
  const Scalar _tmp16 = Scalar(0.20999999999999999) * _tmp13 +
                        Scalar(0.20999999999999999) * _tmp15 + Scalar(0.20999999999999999);
  const Scalar _tmp17 = _tmp8 * rot_init_z;
  const Scalar _tmp18 = _tmp8 * rot_init_x;
  const Scalar _tmp19 = _tmp10 * ry + _tmp17 * rx - _tmp18 * rz + _tmp7 * rot_init_y;
  const Scalar _tmp20 = 2 * _tmp14;
  const Scalar _tmp21 = _tmp19 * _tmp20;
  const Scalar _tmp22 = -_tmp11 * ry - _tmp17 * rz - _tmp18 * rx + _tmp7 * rot_init_w;
  const Scalar _tmp23 = 2 * _tmp12 * _tmp22;
  const Scalar _tmp24 = _tmp21 - _tmp23;
  const Scalar _tmp25 = -Scalar(0.010999999999999999) * _tmp24;
  const Scalar _tmp26 = 2 * _tmp19;
  const Scalar _tmp27 = _tmp12 * _tmp26;
  const Scalar _tmp28 = _tmp20 * _tmp22;
  const Scalar _tmp29 = Scalar(0.20999999999999999) * _tmp27 + Scalar(0.20999999999999999) * _tmp28;
  const Scalar _tmp30 = _tmp25 + _tmp29;
  const Scalar _tmp31 = _tmp16 + _tmp30;
  const Scalar _tmp32 = _tmp31 + p_init1;
  const Scalar _tmp33 = Scalar(0.20999999999999999) * _tmp27 - Scalar(0.20999999999999999) * _tmp28;
  const Scalar _tmp34 = _tmp12 * _tmp20;
  const Scalar _tmp35 = _tmp22 * _tmp26;
  const Scalar _tmp36 = _tmp34 + _tmp35;
  const Scalar _tmp37 = -Scalar(0.010999999999999999) * _tmp36;
  const Scalar _tmp38 = 1 - 2 * std::pow(_tmp19, Scalar(2));
  const Scalar _tmp39 = Scalar(0.20999999999999999) * _tmp15 + Scalar(0.20999999999999999) * _tmp38;
  const Scalar _tmp40 = _tmp37 + _tmp39;
  const Scalar _tmp41 = _tmp33 + _tmp40;
  const Scalar _tmp42 = _tmp41 + p_init0;
  const Scalar _tmp43 =
      -Scalar(0.1034955) * _tmp4 * fh1 -
      Scalar(0.49421237293624504) *
          std::sqrt(
              Scalar(std::pow(Scalar(1 - Scalar(0.20941503221602112) * _tmp32), Scalar(2)) +
                     Scalar(0.32397683292140877) *
                         std::pow(Scalar(1 - Scalar(0.36791786395571047) * _tmp42), Scalar(2))));
  const Scalar _tmp44 =
      std::pow(Scalar(_tmp0 * std::pow(fv1, Scalar(2)) + 1), Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp45 = _tmp1 * _tmp43;
  const Scalar _tmp46 = Scalar(1.0) * _tmp3;
  const Scalar _tmp47 = _tmp32 + Scalar(-4.7752063900000001);
  const Scalar _tmp48 = _tmp42 + Scalar(-2.71799795);
  const Scalar _tmp49 = std::pow(Scalar(std::pow(_tmp47, Scalar(2)) + std::pow(_tmp48, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp50 = _tmp47 * _tmp49;
  const Scalar _tmp51 = _tmp48 * _tmp49;
  const Scalar _tmp52 = -_tmp31 * _tmp51 + _tmp41 * _tmp50;
  const Scalar _tmp53 = _tmp25 - _tmp29;
  const Scalar _tmp54 = _tmp16 + _tmp53;
  const Scalar _tmp55 = _tmp54 + p_init1;
  const Scalar _tmp56 = _tmp55 + Scalar(-4.8333311099999996);
  const Scalar _tmp57 = _tmp37 - _tmp39;
  const Scalar _tmp58 = _tmp33 + _tmp57;
  const Scalar _tmp59 = _tmp58 + p_init0;
  const Scalar _tmp60 = _tmp59 + Scalar(1.79662371);
  const Scalar _tmp61 = Scalar(1.0) / (_tmp60);
  const Scalar _tmp62 = _tmp56 * _tmp61;
  const Scalar _tmp63 = -_tmp16;
  const Scalar _tmp64 = _tmp53 + _tmp63;
  const Scalar _tmp65 = _tmp64 + p_init1;
  const Scalar _tmp66 = _tmp65 + Scalar(8.3196563700000006);
  const Scalar _tmp67 = -_tmp33;
  const Scalar _tmp68 = _tmp57 + _tmp67;
  const Scalar _tmp69 = _tmp68 + p_init0;
  const Scalar _tmp70 = _tmp69 + Scalar(1.9874742000000001);
  const Scalar _tmp71 = std::pow(Scalar(std::pow(_tmp66, Scalar(2)) + std::pow(_tmp70, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp72 = _tmp70 * _tmp71;
  const Scalar _tmp73 = _tmp66 * _tmp71;
  const Scalar _tmp74 = _tmp62 * _tmp72 - _tmp73;
  const Scalar _tmp75 =
      std::sqrt(Scalar(std::pow(_tmp56, Scalar(2)) + std::pow(_tmp60, Scalar(2))));
  const Scalar _tmp76 = Scalar(1.0) / (_tmp75);
  const Scalar _tmp77 = _tmp61 * _tmp75;
  const Scalar _tmp78 = _tmp77 * (-_tmp54 * _tmp60 * _tmp76 + _tmp56 * _tmp58 * _tmp76);
  const Scalar _tmp79 = _tmp40 + _tmp67;
  const Scalar _tmp80 = _tmp79 + p_init0;
  const Scalar _tmp81 = _tmp80 + Scalar(-2.5202214700000001);
  const Scalar _tmp82 = _tmp30 + _tmp63;
  const Scalar _tmp83 = _tmp82 + p_init1;
  const Scalar _tmp84 = _tmp83 + Scalar(8.3888750099999996);
  const Scalar _tmp85 = std::pow(Scalar(std::pow(_tmp81, Scalar(2)) + std::pow(_tmp84, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp86 = _tmp81 * _tmp85;
  const Scalar _tmp87 = _tmp84 * _tmp85;
  const Scalar _tmp88 = Scalar(1.0) / (_tmp62 * _tmp86 - _tmp87);
  const Scalar _tmp89 = _tmp88 * (_tmp78 * _tmp86 - _tmp79 * _tmp87 + _tmp82 * _tmp86);
  const Scalar _tmp90 = _tmp64 * _tmp72 - _tmp68 * _tmp73 + _tmp72 * _tmp78 - _tmp74 * _tmp89;
  const Scalar _tmp91 = Scalar(1.0) / (_tmp90);
  const Scalar _tmp92 = Scalar(1.0) * _tmp91;
  const Scalar _tmp93 = _tmp86 * _tmp88;
  const Scalar _tmp94 = _tmp74 * _tmp93;
  const Scalar _tmp95 = _tmp52 * _tmp77 * (_tmp72 * _tmp92 - _tmp92 * _tmp94);
  const Scalar _tmp96 = Scalar(0.20999999999999999) * _tmp34 - Scalar(0.20999999999999999) * _tmp35;
  const Scalar _tmp97 =
      -Scalar(0.010999999999999999) * _tmp13 - Scalar(0.010999999999999999) * _tmp38;
  const Scalar _tmp98 = Scalar(0.20999999999999999) * _tmp21 + Scalar(0.20999999999999999) * _tmp23;
  const Scalar _tmp99 = _tmp97 - _tmp98;
  const Scalar _tmp100 = _tmp96 + _tmp99;
  const Scalar _tmp101 = -_tmp96;
  const Scalar _tmp102 = _tmp97 + _tmp98;
  const Scalar _tmp103 = _tmp101 + _tmp102;
  const Scalar _tmp104 = _tmp88 * (-_tmp100 * _tmp86 + _tmp103 * _tmp86);
  const Scalar _tmp105 = _tmp103 * _tmp62;
  const Scalar _tmp106 = _tmp88 * (_tmp100 * _tmp87 - _tmp105 * _tmp86);
  const Scalar _tmp107 = _tmp103 * _tmp72;
  const Scalar _tmp108 = _tmp101 + _tmp99;
  const Scalar _tmp109 = -_tmp106 * _tmp74 - _tmp107 * _tmp62 + _tmp108 * _tmp73;
  const Scalar _tmp110 = Scalar(1.0) * _tmp54;
  const Scalar _tmp111 = -_tmp110;
  const Scalar _tmp112 = Scalar(1.0) / (_tmp111 + _tmp82);
  const Scalar _tmp113 = Scalar(1.0) * _tmp58;
  const Scalar _tmp114 = _tmp113 - _tmp79;
  const Scalar _tmp115 = _tmp112 * _tmp114;
  const Scalar _tmp116 = -_tmp104 * _tmp74 + _tmp107 - _tmp108 * _tmp72 - _tmp109 * _tmp115;
  const Scalar _tmp117 = Scalar(1.0) / (_tmp116);
  const Scalar _tmp118 = Scalar(1.0) * _tmp112;
  const Scalar _tmp119 = -Scalar(1.0) * _tmp104 + _tmp106 * _tmp114 * _tmp118;
  const Scalar _tmp120 = _tmp117 * _tmp90;
  const Scalar _tmp121 = _tmp116 * _tmp91;
  const Scalar _tmp122 = _tmp121 * (-_tmp119 * _tmp120 - Scalar(1.0) * _tmp89);
  const Scalar _tmp123 = _tmp117 * (_tmp119 + _tmp122);
  const Scalar _tmp124 = -_tmp123 * _tmp74 + Scalar(1.0);
  const Scalar _tmp125 = _tmp50 * _tmp77 * (_tmp123 * _tmp72 + _tmp124 * _tmp93);
  const Scalar _tmp126 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp127 = _tmp110 * _tmp115 + _tmp113;
  const Scalar _tmp128 = 0;
  const Scalar _tmp129 = _tmp105 + _tmp106 * _tmp62;
  const Scalar _tmp130 = -_tmp103 + _tmp104 * _tmp62 - _tmp115 * _tmp129;
  const Scalar _tmp131 = _tmp121 * (-_tmp120 * _tmp130 + _tmp62 * _tmp89 - _tmp78);
  const Scalar _tmp132 = _tmp117 * (_tmp130 + _tmp131);
  const Scalar _tmp133 = -_tmp132 * _tmp74 - _tmp62;
  const Scalar _tmp134 = _tmp51 * _tmp77 * (_tmp132 * _tmp72 + _tmp133 * _tmp93 + Scalar(1.0));
  const Scalar _tmp135 = -_tmp125 * fh1 - _tmp126 * _tmp77 * (_tmp128 * _tmp72 - _tmp128 * _tmp94) -
                         _tmp134 * fh1 - _tmp95 * fh1;
  const Scalar _tmp136 = Scalar(1.0) / (_tmp135);
  const Scalar _tmp137 = _tmp102 + _tmp96;
  const Scalar _tmp138 = _tmp137 * fh1;
  const Scalar _tmp139 = _tmp138 * _tmp51 + Scalar(3.29616) * _tmp36 + _tmp41 * fv1;
  const Scalar _tmp140 = _tmp111 + _tmp64;
  const Scalar _tmp141 = _tmp115 * _tmp140;
  const Scalar _tmp142 = Scalar(1.0) / (_tmp113 - _tmp141 - _tmp68);
  const Scalar _tmp143 = Scalar(1.0) * _tmp142;
  const Scalar _tmp144 = _tmp112 * _tmp140;
  const Scalar _tmp145 = Scalar(1.0) * _tmp143 * _tmp144 - Scalar(1.0) * _tmp143;
  const Scalar _tmp146 = -_tmp138 * _tmp50 - Scalar(3.29616) * _tmp24 - _tmp31 * fv1;
  const Scalar _tmp147 = _tmp141 * _tmp143 + Scalar(1.0);
  const Scalar _tmp148 = _tmp115 * _tmp143;
  const Scalar _tmp149 = -Scalar(1.0) * _tmp118 * _tmp147 + Scalar(1.0) * _tmp148;
  const Scalar _tmp150 = _tmp121 * _tmp143;
  const Scalar _tmp151 = -_tmp109 * _tmp92 + _tmp140 * _tmp150;
  const Scalar _tmp152 = Scalar(1.0) * _tmp52 * (-_tmp118 * _tmp151 + _tmp150);
  const Scalar _tmp153 = _tmp140 * _tmp142;
  const Scalar _tmp154 = -_tmp109 * _tmp132 + _tmp129 + _tmp131 * _tmp153;
  const Scalar _tmp155 = Scalar(1.0) * _tmp51 * (-_tmp118 * _tmp154 + _tmp131 * _tmp143);
  const Scalar _tmp156 = -Scalar(1.0) * _tmp106 - _tmp109 * _tmp123 + _tmp122 * _tmp153;
  const Scalar _tmp157 = Scalar(1.0) * _tmp50 * (-_tmp118 * _tmp156 + _tmp122 * _tmp143);
  const Scalar _tmp158 = _tmp127 * _tmp142;
  const Scalar _tmp159 = -_tmp109 * _tmp128 + _tmp111 - _tmp140 * _tmp158;
  const Scalar _tmp160 =
      Scalar(1.0) * _tmp126 * (-_tmp118 * _tmp159 - _tmp127 * _tmp143 + Scalar(1.0)) +
      _tmp139 * _tmp145 + _tmp146 * _tmp149 + _tmp152 * fh1 + _tmp155 * fh1 + _tmp157 * fh1;
  const Scalar _tmp161 = std::asinh(_tmp136 * _tmp160);
  const Scalar _tmp162 = Scalar(1.0) * _tmp161;
  const Scalar _tmp163 = std::pow(_tmp135, Scalar(-2));
  const Scalar _tmp164 = _tmp137 * _tmp50;
  const Scalar _tmp165 = _tmp137 * _tmp51;
  const Scalar _tmp166 = -_tmp125 - _tmp134 - _tmp95;
  const Scalar _tmp167 = _tmp163 * _tmp166;
  const Scalar _tmp168 =
      (_tmp136 * (_tmp145 * _tmp165 - _tmp149 * _tmp164 + _tmp152 + _tmp155 + _tmp157) -
       _tmp160 * _tmp167) /
      std::sqrt(Scalar(std::pow(_tmp160, Scalar(2)) * _tmp163 + 1));
  const Scalar _tmp169 = Scalar(9.6622558468725703) * _tmp135;
  const Scalar _tmp170 =
      -_tmp161 * _tmp169 -
      Scalar(4.8333311099999996) *
          std::sqrt(
              Scalar(std::pow(Scalar(1 - Scalar(0.20689664689659551) * _tmp55), Scalar(2)) +
                     Scalar(0.13817235445745474) *
                         std::pow(Scalar(-Scalar(0.55659957866191134) * _tmp59 - 1), Scalar(2))));
  const Scalar _tmp171 = Scalar(0.1034955) * _tmp136;
  const Scalar _tmp172 = _tmp170 * _tmp171;
  const Scalar _tmp173 = Scalar(9.6622558468725703) * _tmp166;
  const Scalar _tmp174 = _tmp112 * _tmp151 * _tmp52;
  const Scalar _tmp175 = _tmp139 * _tmp143;
  const Scalar _tmp176 = _tmp112 * _tmp147;
  const Scalar _tmp177 = _tmp112 * _tmp156 * _tmp50;
  const Scalar _tmp178 = _tmp112 * _tmp51;
  const Scalar _tmp179 = _tmp154 * _tmp178;
  const Scalar _tmp180 = _tmp112 * _tmp126 * _tmp159 - _tmp144 * _tmp175 + _tmp146 * _tmp176 +
                         _tmp174 * fh1 + _tmp177 * fh1 + _tmp179 * fh1;
  const Scalar _tmp181 = _tmp126 * _tmp128;
  const Scalar _tmp182 = _tmp74 * _tmp88;
  const Scalar _tmp183 = _tmp52 * _tmp92;
  const Scalar _tmp184 = _tmp183 * fh1;
  const Scalar _tmp185 = _tmp133 * _tmp51 * _tmp88;
  const Scalar _tmp186 = _tmp124 * _tmp50 * _tmp88;
  const Scalar _tmp187 = -_tmp181 * _tmp182 - _tmp182 * _tmp184 + _tmp185 * fh1 + _tmp186 * fh1;
  const Scalar _tmp188 = Scalar(1.0) / (_tmp187);
  const Scalar _tmp189 = std::asinh(_tmp180 * _tmp188);
  const Scalar _tmp190 = Scalar(1.0) * _tmp189;
  const Scalar _tmp191 = Scalar(9.6622558468725703) * _tmp187;
  const Scalar _tmp192 =
      -_tmp189 * _tmp191 -
      Scalar(8.3888750099999996) *
          std::sqrt(
              Scalar(Scalar(0.090254729040973036) *
                         std::pow(Scalar(1 - Scalar(0.39679052492160538) * _tmp80), Scalar(2)) +
                     std::pow(Scalar(-Scalar(0.11920549523123722) * _tmp83 - 1), Scalar(2))));
  const Scalar _tmp193 = Scalar(0.1034955) * _tmp188;
  const Scalar _tmp194 = _tmp192 * _tmp193;
  const Scalar _tmp195 = -_tmp182 * _tmp183 + _tmp185 + _tmp186;
  const Scalar _tmp196 = Scalar(9.6622558468725703) * _tmp195;
  const Scalar _tmp197 = std::pow(_tmp187, Scalar(-2));
  const Scalar _tmp198 = _tmp195 * _tmp197;
  const Scalar _tmp199 =
      (-_tmp180 * _tmp198 + _tmp188 * (-_tmp137 * _tmp140 * _tmp143 * _tmp178 - _tmp164 * _tmp176 +
                                       _tmp174 + _tmp177 + _tmp179)) /
      std::sqrt(Scalar(std::pow(_tmp180, Scalar(2)) * _tmp197 + 1));
  const Scalar _tmp200 = _tmp132 * _tmp51;
  const Scalar _tmp201 = _tmp123 * _tmp50;
  const Scalar _tmp202 = _tmp181 + _tmp184 + _tmp200 * fh1 + _tmp201 * fh1;
  const Scalar _tmp203 = Scalar(1.0) / (_tmp202);
  const Scalar _tmp204 = _tmp131 * _tmp142 * _tmp51;
  const Scalar _tmp205 = _tmp122 * _tmp142 * _tmp50;
  const Scalar _tmp206 = _tmp150 * _tmp52;
  const Scalar _tmp207 = _tmp126 * _tmp158 - _tmp146 * _tmp148 + _tmp175 - _tmp204 * fh1 -
                         _tmp205 * fh1 - _tmp206 * fh1;
  const Scalar _tmp208 = std::asinh(_tmp203 * _tmp207);
  const Scalar _tmp209 = Scalar(1.0) * _tmp208;
  const Scalar _tmp210 = Scalar(9.6622558468725703) * _tmp202;
  const Scalar _tmp211 =
      -_tmp208 * _tmp210 -
      Scalar(8.3196563700000006) *
          std::sqrt(
              Scalar(std::pow(Scalar(-Scalar(0.12019727204189803) * _tmp65 - 1), Scalar(2)) +
                     Scalar(0.057067943376852184) *
                         std::pow(Scalar(-Scalar(0.50315118556004401) * _tmp69 - 1), Scalar(2))));
  const Scalar _tmp212 = Scalar(0.1034955) * _tmp203;
  const Scalar _tmp213 = _tmp211 * _tmp212;
  const Scalar _tmp214 = _tmp183 + _tmp200 + _tmp201;
  const Scalar _tmp215 = Scalar(9.6622558468725703) * _tmp214;
  const Scalar _tmp216 = std::pow(_tmp202, Scalar(-2));
  const Scalar _tmp217 = _tmp214 * _tmp216;
  const Scalar _tmp218 =
      (_tmp203 * (_tmp143 * _tmp165 + _tmp148 * _tmp164 - _tmp204 - _tmp205 - _tmp206) -
       _tmp207 * _tmp217) /
      std::sqrt(Scalar(std::pow(_tmp207, Scalar(2)) * _tmp216 + 1));

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) = -Scalar(8.3641632200000018) * _tmp1 -
               Scalar(9.6622558468725703) * fh1 *
                   (-Scalar(1.0) * _tmp0 * _tmp44 * fv1 * std::sinh(_tmp46) -
                    Scalar(0.86565325453551001) * _tmp0 -
                    (-_tmp0 * _tmp43 + Scalar(0.1034955) * _tmp1 *
                                           (Scalar(9.6622558468725703) * _tmp2 * _tmp44 - _tmp4)) *
                        std::sinh(_tmp45)) +
               Scalar(9.6622558468725703) * std::cosh(_tmp45) -
               Scalar(9.6622558468725703) * std::cosh(_tmp46);
  _res(1, 0) =
      -_tmp169 *
          (-Scalar(0.86625939559540499) * _tmp167 + Scalar(1.0) * _tmp168 * std::sinh(_tmp162) -
           (-Scalar(0.1034955) * _tmp167 * _tmp170 +
            _tmp171 * (-_tmp161 * _tmp173 - _tmp168 * _tmp169)) *
               std::sinh(_tmp172)) -
      _tmp173 * (Scalar(0.86625939559540499) * _tmp136 + std::cosh(_tmp162) - std::cosh(_tmp172));
  _res(2, 0) =
      -_tmp191 *
          (-Scalar(0.87653584775870996) * _tmp198 + Scalar(1.0) * _tmp199 * std::sinh(_tmp190) -
           (-Scalar(0.1034955) * _tmp192 * _tmp198 +
            _tmp193 * (-_tmp189 * _tmp196 - _tmp191 * _tmp199)) *
               std::sinh(_tmp194)) -
      _tmp196 * (Scalar(0.87653584775870996) * _tmp188 + std::cosh(_tmp190) - std::cosh(_tmp194));
  _res(3, 0) =
      -_tmp210 *
          (-Scalar(0.87679799772039002) * _tmp217 + Scalar(1.0) * _tmp218 * std::sinh(_tmp209) -
           (-Scalar(0.1034955) * _tmp211 * _tmp217 +
            _tmp212 * (-_tmp208 * _tmp215 - _tmp210 * _tmp218)) *
               std::sinh(_tmp213)) -
      _tmp215 * (Scalar(0.87679799772039002) * _tmp203 + std::cosh(_tmp209) - std::cosh(_tmp213));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym