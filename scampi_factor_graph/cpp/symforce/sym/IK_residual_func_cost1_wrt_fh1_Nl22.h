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
 * Symbolic function: IK_residual_func_cost1_wrt_fh1_Nl22
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost1WrtFh1Nl22(
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
      -_tmp10 * rot_init_x + _tmp6 * rot_init_y + _tmp8 * rot_init_w + _tmp9 * rot_init_z;
  const Scalar _tmp12 = _tmp7 * rot_init_z;
  const Scalar _tmp13 = _tmp10 * rot_init_y - _tmp12 * ry + _tmp6 * rot_init_x + _tmp9 * rot_init_w;
  const Scalar _tmp14 = 2 * _tmp13;
  const Scalar _tmp15 = _tmp11 * _tmp14;
  const Scalar _tmp16 =
      _tmp10 * rot_init_w + _tmp6 * rot_init_z + _tmp8 * rot_init_x - _tmp9 * rot_init_y;
  const Scalar _tmp17 =
      -2 * _tmp12 * rz + 2 * _tmp6 * rot_init_w - 2 * _tmp8 * rot_init_y - 2 * _tmp9 * rot_init_x;
  const Scalar _tmp18 = _tmp16 * _tmp17;
  const Scalar _tmp19 = Scalar(0.20999999999999999) * _tmp15 + Scalar(0.20999999999999999) * _tmp18;
  const Scalar _tmp20 = -_tmp19;
  const Scalar _tmp21 = -2 * std::pow(_tmp16, Scalar(2));
  const Scalar _tmp22 = 1 - 2 * std::pow(_tmp13, Scalar(2));
  const Scalar _tmp23 = Scalar(0.20999999999999999) * _tmp21 + Scalar(0.20999999999999999) * _tmp22;
  const Scalar _tmp24 = 2 * _tmp11 * _tmp16;
  const Scalar _tmp25 = _tmp13 * _tmp17;
  const Scalar _tmp26 = _tmp24 - _tmp25;
  const Scalar _tmp27 = -Scalar(0.010999999999999999) * _tmp26;
  const Scalar _tmp28 = _tmp23 + _tmp27;
  const Scalar _tmp29 = _tmp20 + _tmp28;
  const Scalar _tmp30 = _tmp29 + p_init1;
  const Scalar _tmp31 = Scalar(0.20999999999999999) * _tmp15 - Scalar(0.20999999999999999) * _tmp18;
  const Scalar _tmp32 = -2 * std::pow(_tmp11, Scalar(2));
  const Scalar _tmp33 = Scalar(0.20999999999999999) * _tmp21 +
                        Scalar(0.20999999999999999) * _tmp32 + Scalar(0.20999999999999999);
  const Scalar _tmp34 = _tmp14 * _tmp16;
  const Scalar _tmp35 = _tmp11 * _tmp17;
  const Scalar _tmp36 = _tmp34 + _tmp35;
  const Scalar _tmp37 = -Scalar(0.010999999999999999) * _tmp36;
  const Scalar _tmp38 = -_tmp33 + _tmp37;
  const Scalar _tmp39 = _tmp31 + _tmp38;
  const Scalar _tmp40 = _tmp39 + p_init0;
  const Scalar _tmp41 =
      -_tmp3 * fh1 -
      Scalar(4.8333311099999996) *
          std::sqrt(
              Scalar(std::pow(Scalar(1 - Scalar(0.20689664689659551) * _tmp30), Scalar(2)) +
                     Scalar(0.13817235445745474) *
                         std::pow(Scalar(-Scalar(0.55659957866191134) * _tmp40 - 1), Scalar(2))));
  const Scalar _tmp42 = Scalar(0.1034955) * _tmp0;
  const Scalar _tmp43 = _tmp41 * _tmp42;
  const Scalar _tmp44 = Scalar(1.0) * _tmp2;
  const Scalar _tmp45 = std::pow(fh1, Scalar(-2));
  const Scalar _tmp46 =
      std::pow(Scalar(_tmp45 * std::pow(fv1, Scalar(2)) + 1), Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp47 = -_tmp23 + _tmp27;
  const Scalar _tmp48 = _tmp20 + _tmp47;
  const Scalar _tmp49 = _tmp48 + p_init1;
  const Scalar _tmp50 = _tmp49 + Scalar(8.3196563700000006);
  const Scalar _tmp51 = -_tmp31;
  const Scalar _tmp52 = _tmp38 + _tmp51;
  const Scalar _tmp53 = _tmp52 + p_init0;
  const Scalar _tmp54 = _tmp53 + Scalar(1.9874742000000001);
  const Scalar _tmp55 = std::pow(Scalar(std::pow(_tmp50, Scalar(2)) + std::pow(_tmp54, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp56 = _tmp54 * _tmp55;
  const Scalar _tmp57 = _tmp50 * _tmp55;
  const Scalar _tmp58 = _tmp33 + _tmp37;
  const Scalar _tmp59 = _tmp31 + _tmp58;
  const Scalar _tmp60 = _tmp59 + p_init0;
  const Scalar _tmp61 = _tmp60 + Scalar(-2.71799795);
  const Scalar _tmp62 = Scalar(1.0) / (_tmp61);
  const Scalar _tmp63 = _tmp19 + _tmp28;
  const Scalar _tmp64 = _tmp63 + p_init1;
  const Scalar _tmp65 = _tmp64 + Scalar(-4.7752063900000001);
  const Scalar _tmp66 = _tmp62 * _tmp65;
  const Scalar _tmp67 = Scalar(1.0) / (_tmp56 * _tmp66 - _tmp57);
  const Scalar _tmp68 = _tmp51 + _tmp58;
  const Scalar _tmp69 = _tmp68 + p_init0;
  const Scalar _tmp70 = _tmp69 + Scalar(-2.5202214700000001);
  const Scalar _tmp71 = _tmp19 + _tmp47;
  const Scalar _tmp72 = _tmp71 + p_init1;
  const Scalar _tmp73 = _tmp72 + Scalar(8.3888750099999996);
  const Scalar _tmp74 = std::pow(Scalar(std::pow(_tmp70, Scalar(2)) + std::pow(_tmp73, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp75 = _tmp70 * _tmp74;
  const Scalar _tmp76 = _tmp73 * _tmp74;
  const Scalar _tmp77 = _tmp66 * _tmp75 - _tmp76;
  const Scalar _tmp78 = Scalar(0.20999999999999999) * _tmp34 - Scalar(0.20999999999999999) * _tmp35;
  const Scalar _tmp79 =
      -Scalar(0.010999999999999999) * _tmp22 - Scalar(0.010999999999999999) * _tmp32;
  const Scalar _tmp80 = Scalar(0.20999999999999999) * _tmp24 + Scalar(0.20999999999999999) * _tmp25;
  const Scalar _tmp81 = _tmp79 - _tmp80;
  const Scalar _tmp82 = _tmp78 + _tmp81;
  const Scalar _tmp83 = _tmp79 + _tmp80;
  const Scalar _tmp84 = _tmp78 + _tmp83;
  const Scalar _tmp85 = _tmp75 * _tmp84;
  const Scalar _tmp86 = -_tmp78;
  const Scalar _tmp87 = _tmp81 + _tmp86;
  const Scalar _tmp88 = _tmp56 * _tmp84;
  const Scalar _tmp89 = -_tmp56 * _tmp87 + _tmp88;
  const Scalar _tmp90 = _tmp67 * _tmp77;
  const Scalar _tmp91 = _tmp57 * _tmp87 - _tmp66 * _tmp88;
  const Scalar _tmp92 = -_tmp66 * _tmp85 + _tmp76 * _tmp82 - _tmp90 * _tmp91;
  const Scalar _tmp93 = Scalar(1.0) * _tmp63;
  const Scalar _tmp94 = -_tmp93;
  const Scalar _tmp95 = Scalar(1.0) / (_tmp48 + _tmp94);
  const Scalar _tmp96 = Scalar(1.0) * _tmp59;
  const Scalar _tmp97 = _tmp95 * (-_tmp52 + _tmp96);
  const Scalar _tmp98 = -_tmp75 * _tmp82 + _tmp85 - _tmp89 * _tmp90 - _tmp92 * _tmp97;
  const Scalar _tmp99 = Scalar(1.0) / (_tmp98);
  const Scalar _tmp100 = Scalar(1.0) * _tmp67;
  const Scalar _tmp101 = _tmp100 * _tmp91;
  const Scalar _tmp102 = -_tmp100 * _tmp89 + _tmp101 * _tmp97;
  const Scalar _tmp103 =
      std::sqrt(Scalar(std::pow(_tmp61, Scalar(2)) + std::pow(_tmp65, Scalar(2))));
  const Scalar _tmp104 = Scalar(1.0) / (_tmp103);
  const Scalar _tmp105 = _tmp103 * _tmp62;
  const Scalar _tmp106 = _tmp105 * (_tmp104 * _tmp59 * _tmp65 - _tmp104 * _tmp61 * _tmp63);
  const Scalar _tmp107 = _tmp106 * _tmp56 + _tmp48 * _tmp56 - _tmp52 * _tmp57;
  const Scalar _tmp108 = _tmp106 * _tmp75 - _tmp107 * _tmp90 - _tmp68 * _tmp76 + _tmp71 * _tmp75;
  const Scalar _tmp109 = _tmp108 * _tmp99;
  const Scalar _tmp110 = Scalar(1.0) / (_tmp108);
  const Scalar _tmp111 = _tmp110 * _tmp98;
  const Scalar _tmp112 = _tmp111 * (-_tmp100 * _tmp107 - _tmp102 * _tmp109);
  const Scalar _tmp113 = _tmp99 * (_tmp102 + _tmp112);
  const Scalar _tmp114 = _tmp67 * (-_tmp113 * _tmp77 + Scalar(1.0));
  const Scalar _tmp115 = _tmp30 + Scalar(-4.8333311099999996);
  const Scalar _tmp116 = _tmp40 + Scalar(1.79662371);
  const Scalar _tmp117 =
      std::pow(Scalar(std::pow(_tmp115, Scalar(2)) + std::pow(_tmp116, Scalar(2))),
               Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp118 = _tmp115 * _tmp117;
  const Scalar _tmp119 = _tmp105 * _tmp118 * (_tmp113 * _tmp75 + _tmp114 * _tmp56);
  const Scalar _tmp120 = _tmp116 * _tmp117;
  const Scalar _tmp121 = _tmp118 * _tmp39 - _tmp120 * _tmp29;
  const Scalar _tmp122 = Scalar(1.0) * _tmp110;
  const Scalar _tmp123 = _tmp56 * _tmp90;
  const Scalar _tmp124 = _tmp105 * _tmp121 * (-_tmp122 * _tmp123 + _tmp122 * _tmp75);
  const Scalar _tmp125 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp126 = _tmp93 * _tmp97 + _tmp96;
  const Scalar _tmp127 = 0;
  const Scalar _tmp128 = _tmp66 * _tmp67;
  const Scalar _tmp129 = _tmp128 * _tmp91 + _tmp66 * _tmp84;
  const Scalar _tmp130 = _tmp128 * _tmp89 - _tmp129 * _tmp97 - _tmp84;
  const Scalar _tmp131 = _tmp111 * (-_tmp106 + _tmp107 * _tmp128 - _tmp109 * _tmp130);
  const Scalar _tmp132 = _tmp99 * (_tmp130 + _tmp131);
  const Scalar _tmp133 = _tmp67 * (-_tmp132 * _tmp77 - _tmp66);
  const Scalar _tmp134 = _tmp105 * _tmp120 * (_tmp132 * _tmp75 + _tmp133 * _tmp56 + Scalar(1.0));
  const Scalar _tmp135 = -_tmp105 * _tmp125 * (-_tmp123 * _tmp127 + _tmp127 * _tmp75) -
                         _tmp119 * fh1 - _tmp124 * fh1 - _tmp134 * fh1;
  const Scalar _tmp136 = std::pow(_tmp135, Scalar(-2));
  const Scalar _tmp137 = -_tmp119 - _tmp124 - _tmp134;
  const Scalar _tmp138 = _tmp136 * _tmp137;
  const Scalar _tmp139 = _tmp71 + _tmp94;
  const Scalar _tmp140 = _tmp139 * _tmp97;
  const Scalar _tmp141 = Scalar(1.0) / (-_tmp140 - _tmp68 + _tmp96);
  const Scalar _tmp142 = Scalar(1.0) * _tmp141;
  const Scalar _tmp143 = _tmp126 * _tmp141;
  const Scalar _tmp144 = _tmp95 * (-_tmp127 * _tmp92 - _tmp139 * _tmp143 + _tmp94);
  const Scalar _tmp145 = _tmp139 * _tmp141;
  const Scalar _tmp146 = -_tmp101 + _tmp112 * _tmp145 - _tmp113 * _tmp92;
  const Scalar _tmp147 = Scalar(1.0) * _tmp95;
  const Scalar _tmp148 = Scalar(1.0) * _tmp118 * (_tmp112 * _tmp142 - _tmp146 * _tmp147);
  const Scalar _tmp149 = _tmp83 + _tmp86;
  const Scalar _tmp150 = _tmp149 * fh1;
  const Scalar _tmp151 = -_tmp118 * _tmp150 - Scalar(3.29616) * _tmp26 - _tmp29 * fv1;
  const Scalar _tmp152 = _tmp95 * (_tmp140 * _tmp142 + Scalar(1.0));
  const Scalar _tmp153 = _tmp142 * _tmp97;
  const Scalar _tmp154 = -Scalar(1.0) * _tmp152 + Scalar(1.0) * _tmp153;
  const Scalar _tmp155 = _tmp111 * _tmp142;
  const Scalar _tmp156 = _tmp95 * (-_tmp122 * _tmp92 + _tmp139 * _tmp155);
  const Scalar _tmp157 = Scalar(1.0) * _tmp121;
  const Scalar _tmp158 = _tmp157 * (_tmp155 - Scalar(1.0) * _tmp156);
  const Scalar _tmp159 = _tmp129 + _tmp131 * _tmp145 - _tmp132 * _tmp92;
  const Scalar _tmp160 = Scalar(1.0) * _tmp120 * (_tmp131 * _tmp142 - _tmp147 * _tmp159);
  const Scalar _tmp161 = _tmp120 * _tmp150 + Scalar(3.29616) * _tmp36 + _tmp39 * fv1;
  const Scalar _tmp162 = _tmp139 * _tmp95;
  const Scalar _tmp163 = Scalar(1.0) * _tmp142 * _tmp162 - Scalar(1.0) * _tmp142;
  const Scalar _tmp164 =
      Scalar(1.0) * _tmp125 * (-_tmp126 * _tmp142 - Scalar(1.0) * _tmp144 + Scalar(1.0)) +
      _tmp148 * fh1 + _tmp151 * _tmp154 + _tmp158 * fh1 + _tmp160 * fh1 + _tmp161 * _tmp163;
  const Scalar _tmp165 = Scalar(1.0) / (_tmp135);
  const Scalar _tmp166 = std::asinh(_tmp164 * _tmp165);
  const Scalar _tmp167 = Scalar(1.0) * _tmp166;
  const Scalar _tmp168 = _tmp118 * _tmp149;
  const Scalar _tmp169 = _tmp120 * _tmp149;
  const Scalar _tmp170 = (-_tmp138 * _tmp164 + _tmp165 * (_tmp148 - _tmp154 * _tmp168 + _tmp158 +
                                                          _tmp160 + _tmp163 * _tmp169)) /
                         std::sqrt(Scalar(_tmp136 * std::pow(_tmp164, Scalar(2)) + 1));
  const Scalar _tmp171 = Scalar(9.6622558468725703) * _tmp135;
  const Scalar _tmp172 =
      -_tmp166 * _tmp171 -
      Scalar(4.7752063900000001) *
          std::sqrt(
              Scalar(Scalar(0.32397683292140877) *
                         std::pow(Scalar(1 - Scalar(0.36791786395571047) * _tmp60), Scalar(2)) +
                     std::pow(Scalar(1 - Scalar(0.20941503221602112) * _tmp64), Scalar(2))));
  const Scalar _tmp173 = Scalar(0.1034955) * _tmp165;
  const Scalar _tmp174 = _tmp172 * _tmp173;
  const Scalar _tmp175 = Scalar(9.6622558468725703) * _tmp137;
  const Scalar _tmp176 = _tmp125 * _tmp127;
  const Scalar _tmp177 = _tmp114 * _tmp118;
  const Scalar _tmp178 = _tmp110 * _tmp157;
  const Scalar _tmp179 = _tmp178 * fh1;
  const Scalar _tmp180 = _tmp120 * _tmp133;
  const Scalar _tmp181 = -_tmp176 * _tmp90 + _tmp177 * fh1 - _tmp179 * _tmp90 + _tmp180 * fh1;
  const Scalar _tmp182 = Scalar(9.6622558468725703) * _tmp181;
  const Scalar _tmp183 = Scalar(1.0) / (_tmp181);
  const Scalar _tmp184 = _tmp120 * _tmp95;
  const Scalar _tmp185 = _tmp159 * _tmp184;
  const Scalar _tmp186 = _tmp118 * _tmp146 * _tmp95;
  const Scalar _tmp187 = _tmp121 * _tmp156;
  const Scalar _tmp188 = _tmp142 * _tmp161;
  const Scalar _tmp189 = _tmp125 * _tmp144 + _tmp151 * _tmp152 - _tmp162 * _tmp188 + _tmp185 * fh1 +
                         _tmp186 * fh1 + _tmp187 * fh1;
  const Scalar _tmp190 = std::pow(_tmp181, Scalar(-2));
  const Scalar _tmp191 = _tmp177 - _tmp178 * _tmp90 + _tmp180;
  const Scalar _tmp192 = _tmp190 * _tmp191;
  const Scalar _tmp193 = (_tmp183 * (-_tmp139 * _tmp142 * _tmp149 * _tmp184 - _tmp152 * _tmp168 +
                                     _tmp185 + _tmp186 + _tmp187) -
                          _tmp189 * _tmp192) /
                         std::sqrt(Scalar(std::pow(_tmp189, Scalar(2)) * _tmp190 + 1));
  const Scalar _tmp194 = std::asinh(_tmp183 * _tmp189);
  const Scalar _tmp195 = Scalar(9.6622558468725703) * _tmp194;
  const Scalar _tmp196 = Scalar(0.1034955) * _tmp183;
  const Scalar _tmp197 =
      -_tmp181 * _tmp195 -
      Scalar(8.3196563700000006) *
          std::sqrt(
              Scalar(std::pow(Scalar(-Scalar(0.12019727204189803) * _tmp49 - 1), Scalar(2)) +
                     Scalar(0.057067943376852184) *
                         std::pow(Scalar(-Scalar(0.50315118556004401) * _tmp53 - 1), Scalar(2))));
  const Scalar _tmp198 = _tmp196 * _tmp197;
  const Scalar _tmp199 = Scalar(1.0) * _tmp194;
  const Scalar _tmp200 = _tmp113 * _tmp118;
  const Scalar _tmp201 = _tmp120 * _tmp132;
  const Scalar _tmp202 = _tmp176 + _tmp179 + _tmp200 * fh1 + _tmp201 * fh1;
  const Scalar _tmp203 = Scalar(1.0) / (_tmp202);
  const Scalar _tmp204 = _tmp120 * _tmp131 * _tmp141;
  const Scalar _tmp205 = _tmp121 * _tmp155;
  const Scalar _tmp206 = _tmp112 * _tmp118 * _tmp141;
  const Scalar _tmp207 = _tmp125 * _tmp143 - _tmp151 * _tmp153 + _tmp188 - _tmp204 * fh1 -
                         _tmp205 * fh1 - _tmp206 * fh1;
  const Scalar _tmp208 = std::asinh(_tmp203 * _tmp207);
  const Scalar _tmp209 = Scalar(1.0) * _tmp208;
  const Scalar _tmp210 = Scalar(9.6622558468725703) * _tmp208;
  const Scalar _tmp211 =
      -_tmp202 * _tmp210 -
      Scalar(8.3888750099999996) *
          std::sqrt(
              Scalar(Scalar(0.090254729040973036) *
                         std::pow(Scalar(1 - Scalar(0.39679052492160538) * _tmp69), Scalar(2)) +
                     std::pow(Scalar(-Scalar(0.11920549523123722) * _tmp72 - 1), Scalar(2))));
  const Scalar _tmp212 = Scalar(0.1034955) * _tmp203;
  const Scalar _tmp213 = _tmp211 * _tmp212;
  const Scalar _tmp214 = _tmp178 + _tmp200 + _tmp201;
  const Scalar _tmp215 = std::pow(_tmp202, Scalar(-2));
  const Scalar _tmp216 = _tmp214 * _tmp215;
  const Scalar _tmp217 =
      (_tmp203 * (_tmp142 * _tmp169 + _tmp153 * _tmp168 - _tmp204 - _tmp205 - _tmp206) -
       _tmp207 * _tmp216) /
      std::sqrt(Scalar(std::pow(_tmp207, Scalar(2)) * _tmp215 + 1));
  const Scalar _tmp218 = Scalar(9.6622558468725703) * _tmp202;

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
      -_tmp171 *
          (-Scalar(0.86565325453551001) * _tmp138 + Scalar(1.0) * _tmp170 * std::sinh(_tmp167) -
           (-Scalar(0.1034955) * _tmp138 * _tmp172 +
            _tmp173 * (-_tmp166 * _tmp175 - _tmp170 * _tmp171)) *
               std::sinh(_tmp174)) -
      _tmp175 * (Scalar(0.86565325453551001) * _tmp165 + std::cosh(_tmp167) - std::cosh(_tmp174));
  _res(2, 0) =
      -_tmp182 *
          (-Scalar(0.87679799772039002) * _tmp192 + Scalar(1.0) * _tmp193 * std::sinh(_tmp199) -
           (-Scalar(0.1034955) * _tmp192 * _tmp197 +
            _tmp196 * (-_tmp182 * _tmp193 - _tmp191 * _tmp195)) *
               std::sinh(_tmp198)) -
      Scalar(9.6622558468725703) * _tmp191 *
          (Scalar(0.87679799772039002) * _tmp183 - std::cosh(_tmp198) + std::cosh(_tmp199));
  _res(3, 0) =
      -Scalar(9.6622558468725703) * _tmp214 *
          (Scalar(0.87653584775870996) * _tmp203 + std::cosh(_tmp209) - std::cosh(_tmp213)) -
      _tmp218 *
          (-Scalar(0.87653584775870996) * _tmp216 + Scalar(1.0) * _tmp217 * std::sinh(_tmp209) -
           (-Scalar(0.1034955) * _tmp211 * _tmp216 +
            _tmp212 * (-_tmp210 * _tmp214 - _tmp217 * _tmp218)) *
               std::sinh(_tmp213));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym