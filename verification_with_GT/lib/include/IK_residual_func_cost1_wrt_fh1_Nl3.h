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
 * Symbolic function: IK_residual_func_cost1_wrt_fh1_Nl3
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost1WrtFh1Nl3(
    const Scalar fh1, const Scalar fv1, const Scalar rx, const Scalar ry, const Scalar rz,
    const Scalar p_init0, const Scalar p_init1, const Scalar p_init2, const Scalar rot_init_x,
    const Scalar rot_init_y, const Scalar rot_init_z, const Scalar rot_init_w,
    const Scalar epsilon) {
  // Total ops: 670

  // Unused inputs
  (void)p_init2;
  (void)epsilon;

  // Input arrays

  // Intermediate terms (217)
  const Scalar _tmp0 = std::pow(fh1, Scalar(-2));
  const Scalar _tmp1 = Scalar(1.0) / (fh1);
  const Scalar _tmp2 = std::sqrt(
      Scalar(std::pow(rx, Scalar(2)) + std::pow(ry, Scalar(2)) + std::pow(rz, Scalar(2))));
  const Scalar _tmp3 = (Scalar(1) / Scalar(2)) * _tmp2;
  const Scalar _tmp4 = std::cos(_tmp3);
  const Scalar _tmp5 = std::sin(_tmp3) / _tmp2;
  const Scalar _tmp6 = _tmp5 * ry;
  const Scalar _tmp7 = _tmp5 * rx;
  const Scalar _tmp8 = _tmp5 * rot_init_w;
  const Scalar _tmp9 = _tmp4 * rot_init_z + _tmp6 * rot_init_x - _tmp7 * rot_init_y + _tmp8 * rz;
  const Scalar _tmp10 = -2 * std::pow(_tmp9, Scalar(2));
  const Scalar _tmp11 = _tmp5 * rz;
  const Scalar _tmp12 = _tmp11 * rot_init_y + _tmp4 * rot_init_x - _tmp6 * rot_init_z + _tmp8 * rx;
  const Scalar _tmp13 = 1 - 2 * std::pow(_tmp12, Scalar(2));
  const Scalar _tmp14 = Scalar(0.20999999999999999) * _tmp10 + Scalar(0.20999999999999999) * _tmp13;
  const Scalar _tmp15 = -_tmp14;
  const Scalar _tmp16 = -_tmp11 * rot_init_x + _tmp4 * rot_init_y + _tmp7 * rot_init_z + _tmp8 * ry;
  const Scalar _tmp17 = 2 * _tmp16 * _tmp9;
  const Scalar _tmp18 = -2 * _tmp11 * rot_init_z + 2 * _tmp4 * rot_init_w - 2 * _tmp6 * rot_init_y -
                        2 * _tmp7 * rot_init_x;
  const Scalar _tmp19 = _tmp12 * _tmp18;
  const Scalar _tmp20 = _tmp17 - _tmp19;
  const Scalar _tmp21 = -Scalar(0.010999999999999999) * _tmp20;
  const Scalar _tmp22 = 2 * _tmp12;
  const Scalar _tmp23 = _tmp16 * _tmp22;
  const Scalar _tmp24 = _tmp18 * _tmp9;
  const Scalar _tmp25 = Scalar(0.20999999999999999) * _tmp23 + Scalar(0.20999999999999999) * _tmp24;
  const Scalar _tmp26 = _tmp21 - _tmp25;
  const Scalar _tmp27 = _tmp15 + _tmp26;
  const Scalar _tmp28 = _tmp27 + p_init1;
  const Scalar _tmp29 = -2 * std::pow(_tmp16, Scalar(2));
  const Scalar _tmp30 = Scalar(0.20999999999999999) * _tmp10 +
                        Scalar(0.20999999999999999) * _tmp29 + Scalar(0.20999999999999999);
  const Scalar _tmp31 = -_tmp30;
  const Scalar _tmp32 = Scalar(0.20999999999999999) * _tmp23 - Scalar(0.20999999999999999) * _tmp24;
  const Scalar _tmp33 = _tmp22 * _tmp9;
  const Scalar _tmp34 = _tmp16 * _tmp18;
  const Scalar _tmp35 = _tmp33 + _tmp34;
  const Scalar _tmp36 = -Scalar(0.010999999999999999) * _tmp35;
  const Scalar _tmp37 = -_tmp32 + _tmp36;
  const Scalar _tmp38 = _tmp31 + _tmp37;
  const Scalar _tmp39 = _tmp38 + p_init0;
  const Scalar _tmp40 = _tmp1 * fv1;
  const Scalar _tmp41 = std::asinh(_tmp40);
  const Scalar _tmp42 = Scalar(9.6622558468725703) * _tmp41;
  const Scalar _tmp43 =
      -Scalar(0.1034955) * _tmp42 * fh1 -
      Scalar(0.86104699584133515) *
          std::sqrt(
              Scalar(std::pow(Scalar(-Scalar(0.12019727204189803) * _tmp28 - 1), Scalar(2)) +
                     Scalar(0.057067943376852184) *
                         std::pow(Scalar(-Scalar(0.50315118556004401) * _tmp39 - 1), Scalar(2))));
  const Scalar _tmp44 = _tmp1 * _tmp43;
  const Scalar _tmp45 =
      std::pow(Scalar(_tmp0 * std::pow(fv1, Scalar(2)) + 1), Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp46 = Scalar(1.0) * _tmp41;
  const Scalar _tmp47 = _tmp30 + _tmp37;
  const Scalar _tmp48 = _tmp21 + _tmp25;
  const Scalar _tmp49 = _tmp14 + _tmp48;
  const Scalar _tmp50 = Scalar(1.0) * _tmp49;
  const Scalar _tmp51 = -_tmp50;
  const Scalar _tmp52 = _tmp15 + _tmp48;
  const Scalar _tmp53 = _tmp51 + _tmp52;
  const Scalar _tmp54 = _tmp14 + _tmp26;
  const Scalar _tmp55 = Scalar(1.0) / (_tmp51 + _tmp54);
  const Scalar _tmp56 = _tmp32 + _tmp36;
  const Scalar _tmp57 = _tmp31 + _tmp56;
  const Scalar _tmp58 = _tmp30 + _tmp56;
  const Scalar _tmp59 = Scalar(1.0) * _tmp58;
  const Scalar _tmp60 = _tmp55 * (-_tmp57 + _tmp59);
  const Scalar _tmp61 = _tmp53 * _tmp60;
  const Scalar _tmp62 = Scalar(1.0) / (-_tmp47 + _tmp59 - _tmp61);
  const Scalar _tmp63 = Scalar(1.0) * _tmp62;
  const Scalar _tmp64 = _tmp49 + p_init1;
  const Scalar _tmp65 = _tmp64 + Scalar(-4.7752063900000001);
  const Scalar _tmp66 = _tmp58 + p_init0;
  const Scalar _tmp67 = _tmp66 + Scalar(-2.71799795);
  const Scalar _tmp68 =
      std::sqrt(Scalar(std::pow(_tmp65, Scalar(2)) + std::pow(_tmp67, Scalar(2))));
  const Scalar _tmp69 = Scalar(1.0) / (_tmp68);
  const Scalar _tmp70 = Scalar(1.0) / (_tmp67);
  const Scalar _tmp71 = _tmp68 * _tmp70;
  const Scalar _tmp72 = _tmp71 * (-_tmp49 * _tmp67 * _tmp69 + _tmp58 * _tmp65 * _tmp69);
  const Scalar _tmp73 = Scalar(0.20999999999999999) * _tmp33 - Scalar(0.20999999999999999) * _tmp34;
  const Scalar _tmp74 =
      -Scalar(0.010999999999999999) * _tmp13 - Scalar(0.010999999999999999) * _tmp29;
  const Scalar _tmp75 = Scalar(0.20999999999999999) * _tmp17 + Scalar(0.20999999999999999) * _tmp19;
  const Scalar _tmp76 = _tmp74 + _tmp75;
  const Scalar _tmp77 = _tmp73 + _tmp76;
  const Scalar _tmp78 = _tmp54 + p_init1;
  const Scalar _tmp79 = _tmp78 + Scalar(-4.8333311099999996);
  const Scalar _tmp80 = _tmp57 + p_init0;
  const Scalar _tmp81 = _tmp80 + Scalar(1.79662371);
  const Scalar _tmp82 = std::pow(Scalar(std::pow(_tmp79, Scalar(2)) + std::pow(_tmp81, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp83 = _tmp81 * _tmp82;
  const Scalar _tmp84 = _tmp77 * _tmp83;
  const Scalar _tmp85 = -_tmp73;
  const Scalar _tmp86 = _tmp76 + _tmp85;
  const Scalar _tmp87 = -_tmp83 * _tmp86 + _tmp84;
  const Scalar _tmp88 = _tmp65 * _tmp70;
  const Scalar _tmp89 = _tmp79 * _tmp82;
  const Scalar _tmp90 = Scalar(1.0) / (_tmp83 * _tmp88 - _tmp89);
  const Scalar _tmp91 = _tmp88 * _tmp90;
  const Scalar _tmp92 = -_tmp84 * _tmp88 + _tmp86 * _tmp89;
  const Scalar _tmp93 = _tmp77 * _tmp88 + _tmp91 * _tmp92;
  const Scalar _tmp94 = -_tmp60 * _tmp93 - _tmp77 + _tmp87 * _tmp91;
  const Scalar _tmp95 = _tmp47 + p_init0;
  const Scalar _tmp96 = _tmp95 + Scalar(-2.5202214700000001);
  const Scalar _tmp97 = _tmp52 + p_init1;
  const Scalar _tmp98 = _tmp97 + Scalar(8.3888750099999996);
  const Scalar _tmp99 = std::pow(Scalar(std::pow(_tmp96, Scalar(2)) + std::pow(_tmp98, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp100 = _tmp96 * _tmp99;
  const Scalar _tmp101 = _tmp98 * _tmp99;
  const Scalar _tmp102 = _tmp100 * _tmp88 - _tmp101;
  const Scalar _tmp103 = _tmp102 * _tmp90;
  const Scalar _tmp104 = _tmp74 - _tmp75;
  const Scalar _tmp105 = _tmp104 + _tmp73;
  const Scalar _tmp106 = _tmp100 * _tmp77;
  const Scalar _tmp107 = _tmp101 * _tmp105 - _tmp103 * _tmp92 - _tmp106 * _tmp88;
  const Scalar _tmp108 = -_tmp100 * _tmp105 - _tmp103 * _tmp87 + _tmp106 - _tmp107 * _tmp60;
  const Scalar _tmp109 = Scalar(1.0) / (_tmp108);
  const Scalar _tmp110 = _tmp54 * _tmp83 - _tmp57 * _tmp89 + _tmp72 * _tmp83;
  const Scalar _tmp111 = _tmp100 * _tmp52 + _tmp100 * _tmp72 - _tmp101 * _tmp47 - _tmp103 * _tmp110;
  const Scalar _tmp112 = _tmp109 * _tmp111;
  const Scalar _tmp113 = Scalar(1.0) / (_tmp111);
  const Scalar _tmp114 = _tmp108 * _tmp113;
  const Scalar _tmp115 = _tmp114 * (_tmp110 * _tmp91 - _tmp112 * _tmp94 - _tmp72);
  const Scalar _tmp116 = _tmp53 * _tmp62;
  const Scalar _tmp117 = _tmp109 * (_tmp115 + _tmp94);
  const Scalar _tmp118 = -_tmp107 * _tmp117 + _tmp115 * _tmp116 + _tmp93;
  const Scalar _tmp119 = Scalar(1.0) * _tmp55;
  const Scalar _tmp120 = _tmp28 + Scalar(8.3196563700000006);
  const Scalar _tmp121 = _tmp39 + Scalar(1.9874742000000001);
  const Scalar _tmp122 =
      std::pow(Scalar(std::pow(_tmp120, Scalar(2)) + std::pow(_tmp121, Scalar(2))),
               Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp123 = _tmp121 * _tmp122;
  const Scalar _tmp124 = Scalar(1.0) * _tmp123 * (_tmp115 * _tmp63 - _tmp118 * _tmp119);
  const Scalar _tmp125 = _tmp104 + _tmp85;
  const Scalar _tmp126 = _tmp125 * fh1;
  const Scalar _tmp127 = _tmp123 * _tmp126 + Scalar(3.29616) * _tmp35 + _tmp38 * fv1;
  const Scalar _tmp128 = _tmp53 * _tmp55;
  const Scalar _tmp129 = Scalar(1.0) * _tmp128 * _tmp63 - Scalar(1.0) * _tmp63;
  const Scalar _tmp130 = Scalar(1.0) * _tmp90;
  const Scalar _tmp131 = _tmp130 * _tmp92;
  const Scalar _tmp132 = -_tmp130 * _tmp87 + _tmp131 * _tmp60;
  const Scalar _tmp133 = _tmp114 * (-_tmp110 * _tmp130 - _tmp112 * _tmp132);
  const Scalar _tmp134 = _tmp109 * (_tmp132 + _tmp133);
  const Scalar _tmp135 = -_tmp107 * _tmp134 + _tmp116 * _tmp133 - _tmp131;
  const Scalar _tmp136 = _tmp120 * _tmp122;
  const Scalar _tmp137 = Scalar(1.0) * _tmp136 * (-_tmp119 * _tmp135 + _tmp133 * _tmp63);
  const Scalar _tmp138 = -_tmp126 * _tmp136 - Scalar(3.29616) * _tmp20 - _tmp27 * fv1;
  const Scalar _tmp139 = _tmp60 * _tmp63;
  const Scalar _tmp140 = _tmp55 * (_tmp61 * _tmp63 + Scalar(1.0));
  const Scalar _tmp141 = Scalar(1.0) * _tmp139 - Scalar(1.0) * _tmp140;
  const Scalar _tmp142 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp143 = _tmp50 * _tmp60 + _tmp59;
  const Scalar _tmp144 = _tmp143 * _tmp62;
  const Scalar _tmp145 = 0;
  const Scalar _tmp146 = _tmp55 * (-_tmp107 * _tmp145 - _tmp144 * _tmp53 + _tmp51);
  const Scalar _tmp147 = _tmp114 * _tmp63;
  const Scalar _tmp148 = Scalar(1.0) * _tmp113;
  const Scalar _tmp149 = _tmp55 * (-_tmp107 * _tmp148 + _tmp147 * _tmp53);
  const Scalar _tmp150 = -_tmp123 * _tmp27 + _tmp136 * _tmp38;
  const Scalar _tmp151 = Scalar(1.0) * _tmp150 * (_tmp147 - Scalar(1.0) * _tmp149);
  const Scalar _tmp152 =
      _tmp124 * fh1 + _tmp127 * _tmp129 + _tmp137 * fh1 + _tmp138 * _tmp141 +
      Scalar(1.0) * _tmp142 * (-_tmp143 * _tmp63 - Scalar(1.0) * _tmp146 + Scalar(1.0)) +
      _tmp151 * fh1;
  const Scalar _tmp153 = _tmp90 * (-_tmp102 * _tmp117 - _tmp88);
  const Scalar _tmp154 = _tmp123 * _tmp71 * (_tmp100 * _tmp117 + _tmp153 * _tmp83 + Scalar(1.0));
  const Scalar _tmp155 = _tmp102 * _tmp113 * _tmp130;
  const Scalar _tmp156 = _tmp150 * _tmp71 * (_tmp100 * _tmp148 - _tmp155 * _tmp83);
  const Scalar _tmp157 = _tmp90 * (-_tmp102 * _tmp134 + Scalar(1.0));
  const Scalar _tmp158 = _tmp136 * _tmp71 * (_tmp100 * _tmp134 + _tmp157 * _tmp83);
  const Scalar _tmp159 = -_tmp142 * _tmp71 * (_tmp100 * _tmp145 - _tmp103 * _tmp145 * _tmp83) -
                         _tmp154 * fh1 - _tmp156 * fh1 - _tmp158 * fh1;
  const Scalar _tmp160 = Scalar(1.0) / (_tmp159);
  const Scalar _tmp161 = std::asinh(_tmp152 * _tmp160);
  const Scalar _tmp162 = Scalar(1.0) * _tmp161;
  const Scalar _tmp163 = _tmp125 * _tmp136;
  const Scalar _tmp164 = _tmp123 * _tmp125;
  const Scalar _tmp165 = std::pow(_tmp159, Scalar(-2));
  const Scalar _tmp166 = -_tmp154 - _tmp156 - _tmp158;
  const Scalar _tmp167 = _tmp165 * _tmp166;
  const Scalar _tmp168 = (-_tmp152 * _tmp167 + _tmp160 * (_tmp124 + _tmp129 * _tmp164 + _tmp137 -
                                                          _tmp141 * _tmp163 + _tmp151)) /
                         std::sqrt(Scalar(std::pow(_tmp152, Scalar(2)) * _tmp165 + 1));
  const Scalar _tmp169 = Scalar(9.6622558468725703) * _tmp161;
  const Scalar _tmp170 =
      -_tmp159 * _tmp169 -
      Scalar(4.7752063900000001) *
          std::sqrt(
              Scalar(std::pow(Scalar(1 - Scalar(0.20941503221602112) * _tmp64), Scalar(2)) +
                     Scalar(0.32397683292140877) *
                         std::pow(Scalar(1 - Scalar(0.36791786395571047) * _tmp66), Scalar(2))));
  const Scalar _tmp171 = Scalar(0.1034955) * _tmp160;
  const Scalar _tmp172 = _tmp170 * _tmp171;
  const Scalar _tmp173 = Scalar(9.6622558468725703) * _tmp159;
  const Scalar _tmp174 = _tmp123 * _tmp153;
  const Scalar _tmp175 = _tmp136 * _tmp157;
  const Scalar _tmp176 = _tmp150 * _tmp155;
  const Scalar _tmp177 = _tmp174 + _tmp175 - _tmp176;
  const Scalar _tmp178 = _tmp123 * _tmp55;
  const Scalar _tmp179 = _tmp118 * _tmp178;
  const Scalar _tmp180 = _tmp149 * _tmp150;
  const Scalar _tmp181 = _tmp127 * _tmp63;
  const Scalar _tmp182 = _tmp135 * _tmp136 * _tmp55;
  const Scalar _tmp183 = -_tmp128 * _tmp181 + _tmp138 * _tmp140 + _tmp142 * _tmp146 +
                         _tmp179 * fh1 + _tmp180 * fh1 + _tmp182 * fh1;
  const Scalar _tmp184 = _tmp142 * _tmp145;
  const Scalar _tmp185 = -_tmp103 * _tmp184 + _tmp174 * fh1 + _tmp175 * fh1 - _tmp176 * fh1;
  const Scalar _tmp186 = Scalar(1.0) / (_tmp185);
  const Scalar _tmp187 = std::asinh(_tmp183 * _tmp186);
  const Scalar _tmp188 = Scalar(1.0) * _tmp187;
  const Scalar _tmp189 = Scalar(9.6622558468725703) * _tmp187;
  const Scalar _tmp190 =
      -_tmp185 * _tmp189 -
      Scalar(4.8333311099999996) *
          std::sqrt(
              Scalar(std::pow(Scalar(1 - Scalar(0.20689664689659551) * _tmp78), Scalar(2)) +
                     Scalar(0.13817235445745474) *
                         std::pow(Scalar(-Scalar(0.55659957866191134) * _tmp80 - 1), Scalar(2))));
  const Scalar _tmp191 = Scalar(0.1034955) * _tmp186;
  const Scalar _tmp192 = _tmp190 * _tmp191;
  const Scalar _tmp193 = std::pow(_tmp185, Scalar(-2));
  const Scalar _tmp194 = _tmp177 * _tmp193;
  const Scalar _tmp195 = Scalar(9.6622558468725703) * _tmp185;
  const Scalar _tmp196 =
      (-_tmp183 * _tmp194 + _tmp186 * (-_tmp125 * _tmp178 * _tmp53 * _tmp63 - _tmp140 * _tmp163 +
                                       _tmp179 + _tmp180 + _tmp182)) /
      std::sqrt(Scalar(std::pow(_tmp183, Scalar(2)) * _tmp193 + 1));
  const Scalar _tmp197 = _tmp117 * _tmp123;
  const Scalar _tmp198 = _tmp148 * _tmp150;
  const Scalar _tmp199 = _tmp134 * _tmp136;
  const Scalar _tmp200 = _tmp197 + _tmp198 + _tmp199;
  const Scalar _tmp201 = _tmp184 + _tmp197 * fh1 + _tmp198 * fh1 + _tmp199 * fh1;
  const Scalar _tmp202 = Scalar(1.0) / (_tmp201);
  const Scalar _tmp203 = _tmp147 * _tmp150;
  const Scalar _tmp204 = _tmp133 * _tmp136 * _tmp62;
  const Scalar _tmp205 = _tmp115 * _tmp123 * _tmp62;
  const Scalar _tmp206 = -_tmp138 * _tmp139 + _tmp142 * _tmp144 + _tmp181 - _tmp203 * fh1 -
                         _tmp204 * fh1 - _tmp205 * fh1;
  const Scalar _tmp207 = std::asinh(_tmp202 * _tmp206);
  const Scalar _tmp208 = Scalar(1.0) * _tmp207;
  const Scalar _tmp209 = Scalar(9.6622558468725703) * _tmp207;
  const Scalar _tmp210 =
      -_tmp201 * _tmp209 -
      Scalar(8.3888750099999996) *
          std::sqrt(
              Scalar(Scalar(0.090254729040973036) *
                         std::pow(Scalar(1 - Scalar(0.39679052492160538) * _tmp95), Scalar(2)) +
                     std::pow(Scalar(-Scalar(0.11920549523123722) * _tmp97 - 1), Scalar(2))));
  const Scalar _tmp211 = Scalar(0.1034955) * _tmp202;
  const Scalar _tmp212 = _tmp210 * _tmp211;
  const Scalar _tmp213 = Scalar(9.6622558468725703) * _tmp201;
  const Scalar _tmp214 = std::pow(_tmp201, Scalar(-2));
  const Scalar _tmp215 = _tmp200 * _tmp214;
  const Scalar _tmp216 =
      (_tmp202 * (_tmp139 * _tmp163 + _tmp164 * _tmp63 - _tmp203 - _tmp204 - _tmp205) -
       _tmp206 * _tmp215) /
      std::sqrt(Scalar(std::pow(_tmp206, Scalar(2)) * _tmp214 + 1));

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) =
      -Scalar(8.4718465800000011) * _tmp1 -
      Scalar(9.6622558468725703) * fh1 *
          (-Scalar(1.0) * _tmp0 * _tmp45 * fv1 * std::sinh(_tmp46) -
           Scalar(0.87679799772039002) * _tmp0 -
           (-_tmp0 * _tmp43 +
            Scalar(0.1034955) * _tmp1 * (Scalar(9.6622558468725703) * _tmp40 * _tmp45 - _tmp42)) *
               std::sinh(_tmp44)) +
      Scalar(9.6622558468725703) * std::cosh(_tmp44) -
      Scalar(9.6622558468725703) * std::cosh(_tmp46);
  _res(1, 0) =
      -Scalar(9.6622558468725703) * _tmp166 *
          (Scalar(0.86565325453551001) * _tmp160 + std::cosh(_tmp162) - std::cosh(_tmp172)) -
      _tmp173 *
          (-Scalar(0.86565325453551001) * _tmp167 + Scalar(1.0) * _tmp168 * std::sinh(_tmp162) -
           (-Scalar(0.1034955) * _tmp167 * _tmp170 +
            _tmp171 * (-_tmp166 * _tmp169 - _tmp168 * _tmp173)) *
               std::sinh(_tmp172));
  _res(2, 0) =
      -Scalar(9.6622558468725703) * _tmp177 *
          (Scalar(0.86625939559540499) * _tmp186 + std::cosh(_tmp188) - std::cosh(_tmp192)) -
      _tmp195 *
          (-Scalar(0.86625939559540499) * _tmp194 + Scalar(1.0) * _tmp196 * std::sinh(_tmp188) -
           (-Scalar(0.1034955) * _tmp190 * _tmp194 +
            _tmp191 * (-_tmp177 * _tmp189 - _tmp195 * _tmp196)) *
               std::sinh(_tmp192));
  _res(3, 0) =
      -Scalar(9.6622558468725703) * _tmp200 *
          (Scalar(0.87653584775870996) * _tmp202 + std::cosh(_tmp208) - std::cosh(_tmp212)) -
      _tmp213 *
          (-Scalar(0.87653584775870996) * _tmp215 + Scalar(1.0) * _tmp216 * std::sinh(_tmp208) -
           (-Scalar(0.1034955) * _tmp210 * _tmp215 +
            _tmp211 * (-_tmp200 * _tmp209 - _tmp213 * _tmp216)) *
               std::sinh(_tmp212));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
