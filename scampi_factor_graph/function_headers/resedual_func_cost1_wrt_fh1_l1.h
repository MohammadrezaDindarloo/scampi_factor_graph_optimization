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
 * Symbolic function: resedual_func_cost1_wrt_fh1_l1
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
Eigen::Matrix<Scalar, 4, 1> ResedualFuncCost1WrtFh1L1(
    const Scalar fh1, const Scalar fv1, const Scalar rx, const Scalar ry, const Scalar rz,
    const Scalar p_init0, const Scalar p_init1, const Scalar p_init2, const Scalar rot_init_x,
    const Scalar rot_init_y, const Scalar rot_init_z, const Scalar rot_init_w,
    const Scalar epsilon) {
  // Total ops: 670

  // Unused inputs
  (void)p_init2;
  (void)epsilon;

  // Input arrays

  // Intermediate terms (222)
  const Scalar _tmp0 = std::pow(fh1, Scalar(-2));
  const Scalar _tmp1 = Scalar(1.0) / (fh1);
  const Scalar _tmp2 = std::sqrt(
      Scalar(std::pow(rx, Scalar(2)) + std::pow(ry, Scalar(2)) + std::pow(rz, Scalar(2))));
  const Scalar _tmp3 = (Scalar(1) / Scalar(2)) * _tmp2;
  const Scalar _tmp4 = std::cos(_tmp3);
  const Scalar _tmp5 = std::sin(_tmp3) / _tmp2;
  const Scalar _tmp6 = _tmp5 * ry;
  const Scalar _tmp7 = _tmp5 * rx;
  const Scalar _tmp8 = _tmp5 * rot_init_y;
  const Scalar _tmp9 = _tmp4 * rot_init_x - _tmp6 * rot_init_z + _tmp7 * rot_init_w + _tmp8 * rz;
  const Scalar _tmp10 = -2 * std::pow(_tmp9, Scalar(2));
  const Scalar _tmp11 = _tmp5 * rz;
  const Scalar _tmp12 = _tmp11 * rot_init_w + _tmp4 * rot_init_z + _tmp6 * rot_init_x - _tmp8 * rx;
  const Scalar _tmp13 = -2 * std::pow(_tmp12, Scalar(2));
  const Scalar _tmp14 = Scalar(0.20999999999999999) * _tmp10 +
                        Scalar(0.20999999999999999) * _tmp13 + Scalar(0.20999999999999999);
  const Scalar _tmp15 = -_tmp14;
  const Scalar _tmp16 =
      -_tmp11 * rot_init_x + _tmp4 * rot_init_y + _tmp6 * rot_init_w + _tmp7 * rot_init_z;
  const Scalar _tmp17 = 2 * _tmp12 * _tmp16;
  const Scalar _tmp18 = -2 * _tmp11 * rot_init_z + 2 * _tmp4 * rot_init_w - 2 * _tmp6 * rot_init_y -
                        2 * _tmp7 * rot_init_x;
  const Scalar _tmp19 = _tmp18 * _tmp9;
  const Scalar _tmp20 = _tmp17 - _tmp19;
  const Scalar _tmp21 = -Scalar(0.010999999999999999) * _tmp20;
  const Scalar _tmp22 = 2 * _tmp9;
  const Scalar _tmp23 = _tmp16 * _tmp22;
  const Scalar _tmp24 = _tmp12 * _tmp18;
  const Scalar _tmp25 = Scalar(0.20999999999999999) * _tmp23 + Scalar(0.20999999999999999) * _tmp24;
  const Scalar _tmp26 = _tmp21 - _tmp25;
  const Scalar _tmp27 = _tmp15 + _tmp26;
  const Scalar _tmp28 = _tmp27 + p_init1;
  const Scalar _tmp29 = Scalar(0.20999999999999999) * _tmp23 - Scalar(0.20999999999999999) * _tmp24;
  const Scalar _tmp30 = -_tmp29;
  const Scalar _tmp31 = _tmp12 * _tmp22;
  const Scalar _tmp32 = _tmp16 * _tmp18;
  const Scalar _tmp33 = _tmp31 + _tmp32;
  const Scalar _tmp34 = -Scalar(0.010999999999999999) * _tmp33;
  const Scalar _tmp35 = 1 - 2 * std::pow(_tmp16, Scalar(2));
  const Scalar _tmp36 = Scalar(0.20999999999999999) * _tmp13 + Scalar(0.20999999999999999) * _tmp35;
  const Scalar _tmp37 = _tmp34 - _tmp36;
  const Scalar _tmp38 = _tmp30 + _tmp37;
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
  const Scalar _tmp47 = _tmp28 + Scalar(8.3196563700000006);
  const Scalar _tmp48 = _tmp39 + Scalar(1.9874742000000001);
  const Scalar _tmp49 = std::pow(Scalar(std::pow(_tmp47, Scalar(2)) + std::pow(_tmp48, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp50 = _tmp48 * _tmp49;
  const Scalar _tmp51 = _tmp47 * _tmp49;
  const Scalar _tmp52 = -_tmp27 * _tmp50 + _tmp38 * _tmp51;
  const Scalar _tmp53 = _tmp34 + _tmp36;
  const Scalar _tmp54 = _tmp29 + _tmp53;
  const Scalar _tmp55 = _tmp21 + _tmp25;
  const Scalar _tmp56 = _tmp14 + _tmp55;
  const Scalar _tmp57 = _tmp56 + p_init1;
  const Scalar _tmp58 = _tmp57 + Scalar(-4.7752063900000001);
  const Scalar _tmp59 = _tmp54 + p_init0;
  const Scalar _tmp60 = _tmp59 + Scalar(-2.71799795);
  const Scalar _tmp61 = std::pow(Scalar(std::pow(_tmp58, Scalar(2)) + std::pow(_tmp60, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp62 = _tmp58 * _tmp61;
  const Scalar _tmp63 = _tmp60 * _tmp61;
  const Scalar _tmp64 = _tmp14 + _tmp26;
  const Scalar _tmp65 = _tmp64 + p_init1;
  const Scalar _tmp66 = _tmp65 + Scalar(-4.8333311099999996);
  const Scalar _tmp67 = _tmp29 + _tmp37;
  const Scalar _tmp68 = _tmp67 + p_init0;
  const Scalar _tmp69 = _tmp68 + Scalar(1.79662371);
  const Scalar _tmp70 =
      std::sqrt(Scalar(std::pow(_tmp66, Scalar(2)) + std::pow(_tmp69, Scalar(2))));
  const Scalar _tmp71 = Scalar(1.0) / (_tmp70);
  const Scalar _tmp72 = Scalar(1.0) / (_tmp69);
  const Scalar _tmp73 = _tmp70 * _tmp72;
  const Scalar _tmp74 = _tmp73 * (-_tmp64 * _tmp69 * _tmp71 + _tmp66 * _tmp67 * _tmp71);
  const Scalar _tmp75 = -_tmp54 * _tmp62 + _tmp56 * _tmp63 + _tmp63 * _tmp74;
  const Scalar _tmp76 = _tmp66 * _tmp72;
  const Scalar _tmp77 = Scalar(1.0) / (-_tmp62 + _tmp63 * _tmp76);
  const Scalar _tmp78 = _tmp30 + _tmp53;
  const Scalar _tmp79 = _tmp78 + p_init0;
  const Scalar _tmp80 = _tmp79 + Scalar(-2.5202214700000001);
  const Scalar _tmp81 = _tmp15 + _tmp55;
  const Scalar _tmp82 = _tmp81 + p_init1;
  const Scalar _tmp83 = _tmp82 + Scalar(8.3888750099999996);
  const Scalar _tmp84 = std::pow(Scalar(std::pow(_tmp80, Scalar(2)) + std::pow(_tmp83, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp85 = _tmp83 * _tmp84;
  const Scalar _tmp86 = _tmp80 * _tmp84;
  const Scalar _tmp87 = _tmp76 * _tmp86 - _tmp85;
  const Scalar _tmp88 = _tmp77 * _tmp87;
  const Scalar _tmp89 = _tmp74 * _tmp86 - _tmp75 * _tmp88 - _tmp78 * _tmp85 + _tmp81 * _tmp86;
  const Scalar _tmp90 = Scalar(1.0) / (_tmp89);
  const Scalar _tmp91 = Scalar(1.0) * _tmp90;
  const Scalar _tmp92 = _tmp63 * _tmp88;
  const Scalar _tmp93 = _tmp52 * _tmp73 * (_tmp86 * _tmp91 - _tmp91 * _tmp92);
  const Scalar _tmp94 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp95 = Scalar(0.20999999999999999) * _tmp31 - Scalar(0.20999999999999999) * _tmp32;
  const Scalar _tmp96 =
      -Scalar(0.010999999999999999) * _tmp10 - Scalar(0.010999999999999999) * _tmp35;
  const Scalar _tmp97 = Scalar(0.20999999999999999) * _tmp17 + Scalar(0.20999999999999999) * _tmp19;
  const Scalar _tmp98 = _tmp96 - _tmp97;
  const Scalar _tmp99 = _tmp95 + _tmp98;
  const Scalar _tmp100 = -_tmp95;
  const Scalar _tmp101 = _tmp96 + _tmp97;
  const Scalar _tmp102 = _tmp100 + _tmp101;
  const Scalar _tmp103 = _tmp102 * _tmp63;
  const Scalar _tmp104 = _tmp101 + _tmp95;
  const Scalar _tmp105 = -_tmp103 * _tmp76 + _tmp104 * _tmp62;
  const Scalar _tmp106 = _tmp102 * _tmp76;
  const Scalar _tmp107 = -_tmp105 * _tmp88 - _tmp106 * _tmp86 + _tmp85 * _tmp99;
  const Scalar _tmp108 = Scalar(1.0) * _tmp64;
  const Scalar _tmp109 = -_tmp108;
  const Scalar _tmp110 = Scalar(1.0) / (_tmp109 + _tmp56);
  const Scalar _tmp111 = Scalar(1.0) * _tmp67;
  const Scalar _tmp112 = _tmp111 - _tmp54;
  const Scalar _tmp113 = _tmp110 * _tmp112;
  const Scalar _tmp114 = _tmp103 - _tmp104 * _tmp63;
  const Scalar _tmp115 = _tmp102 * _tmp86 - _tmp107 * _tmp113 - _tmp114 * _tmp88 - _tmp86 * _tmp99;
  const Scalar _tmp116 = Scalar(1.0) / (_tmp115);
  const Scalar _tmp117 = _tmp108 * _tmp113 + _tmp111;
  const Scalar _tmp118 = 0;
  const Scalar _tmp119 = _tmp116 * _tmp118;
  const Scalar _tmp120 = Scalar(1.0) * _tmp77;
  const Scalar _tmp121 = Scalar(1.0) * _tmp110;
  const Scalar _tmp122 = _tmp105 * _tmp112 * _tmp121 * _tmp77 - _tmp114 * _tmp120;
  const Scalar _tmp123 = _tmp116 * _tmp89;
  const Scalar _tmp124 = _tmp115 * _tmp90;
  const Scalar _tmp125 = _tmp124 * (-_tmp120 * _tmp75 - _tmp122 * _tmp123);
  const Scalar _tmp126 = _tmp122 + _tmp125;
  const Scalar _tmp127 = _tmp116 * _tmp87;
  const Scalar _tmp128 = _tmp77 * (-_tmp126 * _tmp127 + Scalar(1.0));
  const Scalar _tmp129 = _tmp116 * _tmp86;
  const Scalar _tmp130 = _tmp51 * _tmp73 * (_tmp126 * _tmp129 + _tmp128 * _tmp63);
  const Scalar _tmp131 = _tmp76 * _tmp77;
  const Scalar _tmp132 = _tmp105 * _tmp131 + _tmp106;
  const Scalar _tmp133 = -_tmp102 - _tmp113 * _tmp132 + _tmp114 * _tmp131;
  const Scalar _tmp134 = _tmp124 * (-_tmp123 * _tmp133 + _tmp131 * _tmp75 - _tmp74);
  const Scalar _tmp135 = _tmp133 + _tmp134;
  const Scalar _tmp136 = _tmp77 * (-_tmp127 * _tmp135 - _tmp76);
  const Scalar _tmp137 = _tmp50 * _tmp73 * (_tmp129 * _tmp135 + _tmp136 * _tmp63 + Scalar(1.0));
  const Scalar _tmp138 = -_tmp130 * fh1 - _tmp137 * fh1 -
                         _tmp73 * _tmp94 * (_tmp119 * _tmp86 - _tmp119 * _tmp92) - _tmp93 * fh1;
  const Scalar _tmp139 = Scalar(1.0) / (_tmp138);
  const Scalar _tmp140 = _tmp109 + _tmp81;
  const Scalar _tmp141 = _tmp113 * _tmp140;
  const Scalar _tmp142 = Scalar(1.0) / (_tmp111 - _tmp141 - _tmp78);
  const Scalar _tmp143 = Scalar(1.0) * _tmp142;
  const Scalar _tmp144 = _tmp107 * _tmp116;
  const Scalar _tmp145 = _tmp140 * _tmp142;
  const Scalar _tmp146 = -_tmp105 * _tmp120 + _tmp125 * _tmp145 - _tmp126 * _tmp144;
  const Scalar _tmp147 = Scalar(1.0) * _tmp51 * (-_tmp121 * _tmp146 + _tmp125 * _tmp143);
  const Scalar _tmp148 = _tmp124 * _tmp143;
  const Scalar _tmp149 = -_tmp107 * _tmp91 + _tmp140 * _tmp148;
  const Scalar _tmp150 = Scalar(1.0) * _tmp52 * (-_tmp121 * _tmp149 + _tmp148);
  const Scalar _tmp151 = _tmp132 + _tmp134 * _tmp145 - _tmp135 * _tmp144;
  const Scalar _tmp152 = Scalar(1.0) * _tmp50 * (-_tmp121 * _tmp151 + _tmp134 * _tmp143);
  const Scalar _tmp153 = _tmp100 + _tmp98;
  const Scalar _tmp154 = _tmp153 * fh1;
  const Scalar _tmp155 = _tmp154 * _tmp50 + Scalar(3.29616) * _tmp33 + _tmp38 * fv1;
  const Scalar _tmp156 = _tmp110 * _tmp140;
  const Scalar _tmp157 = _tmp143 * _tmp156;
  const Scalar _tmp158 = -Scalar(1.0) * _tmp143 + Scalar(1.0) * _tmp157;
  const Scalar _tmp159 = _tmp117 * _tmp142;
  const Scalar _tmp160 = _tmp109 - _tmp118 * _tmp144 - _tmp140 * _tmp159;
  const Scalar _tmp161 = -_tmp154 * _tmp51 - Scalar(3.29616) * _tmp20 - _tmp27 * fv1;
  const Scalar _tmp162 = _tmp141 * _tmp143 + Scalar(1.0);
  const Scalar _tmp163 = _tmp113 * _tmp143;
  const Scalar _tmp164 = -Scalar(1.0) * _tmp121 * _tmp162 + Scalar(1.0) * _tmp163;
  const Scalar _tmp165 =
      _tmp147 * fh1 + _tmp150 * fh1 + _tmp152 * fh1 + _tmp155 * _tmp158 + _tmp161 * _tmp164 +
      Scalar(1.0) * _tmp94 * (-_tmp117 * _tmp143 - _tmp121 * _tmp160 + Scalar(1.0));
  const Scalar _tmp166 = std::asinh(_tmp139 * _tmp165);
  const Scalar _tmp167 = Scalar(9.6622558468725703) * _tmp138;
  const Scalar _tmp168 =
      -_tmp166 * _tmp167 -
      Scalar(4.8333311099999996) *
          std::sqrt(
              Scalar(std::pow(Scalar(1 - Scalar(0.20689664689659551) * _tmp65), Scalar(2)) +
                     Scalar(0.13817235445745474) *
                         std::pow(Scalar(-Scalar(0.55659957866191134) * _tmp68 - 1), Scalar(2))));
  const Scalar _tmp169 = Scalar(0.1034955) * _tmp139;
  const Scalar _tmp170 = _tmp168 * _tmp169;
  const Scalar _tmp171 = _tmp153 * _tmp51;
  const Scalar _tmp172 = _tmp153 * _tmp50;
  const Scalar _tmp173 = std::pow(_tmp138, Scalar(-2));
  const Scalar _tmp174 = -_tmp130 - _tmp137 - _tmp93;
  const Scalar _tmp175 = _tmp173 * _tmp174;
  const Scalar _tmp176 =
      (_tmp139 * (_tmp147 + _tmp150 + _tmp152 + _tmp158 * _tmp172 - _tmp164 * _tmp171) -
       _tmp165 * _tmp175) /
      std::sqrt(Scalar(std::pow(_tmp165, Scalar(2)) * _tmp173 + 1));
  const Scalar _tmp177 = Scalar(9.6622558468725703) * _tmp174;
  const Scalar _tmp178 = Scalar(1.0) * _tmp166;
  const Scalar _tmp179 = _tmp119 * _tmp94;
  const Scalar _tmp180 = _tmp136 * _tmp50;
  const Scalar _tmp181 = _tmp52 * _tmp91;
  const Scalar _tmp182 = _tmp181 * fh1;
  const Scalar _tmp183 = _tmp128 * _tmp51;
  const Scalar _tmp184 = -_tmp179 * _tmp88 + _tmp180 * fh1 - _tmp182 * _tmp88 + _tmp183 * fh1;
  const Scalar _tmp185 = Scalar(1.0) / (_tmp184);
  const Scalar _tmp186 = _tmp110 * _tmp149 * _tmp52;
  const Scalar _tmp187 = _tmp110 * _tmp151 * _tmp50;
  const Scalar _tmp188 = _tmp110 * _tmp146 * _tmp51;
  const Scalar _tmp189 = _tmp143 * _tmp155;
  const Scalar _tmp190 = _tmp110 * _tmp162;
  const Scalar _tmp191 = _tmp110 * _tmp160 * _tmp94 - _tmp156 * _tmp189 + _tmp161 * _tmp190 +
                         _tmp186 * fh1 + _tmp187 * fh1 + _tmp188 * fh1;
  const Scalar _tmp192 = std::asinh(_tmp185 * _tmp191);
  const Scalar _tmp193 = Scalar(9.6622558468725703) * _tmp184;
  const Scalar _tmp194 =
      -_tmp192 * _tmp193 -
      Scalar(4.7752063900000001) *
          std::sqrt(
              Scalar(std::pow(Scalar(1 - Scalar(0.20941503221602112) * _tmp57), Scalar(2)) +
                     Scalar(0.32397683292140877) *
                         std::pow(Scalar(1 - Scalar(0.36791786395571047) * _tmp59), Scalar(2))));
  const Scalar _tmp195 = Scalar(0.1034955) * _tmp185;
  const Scalar _tmp196 = _tmp194 * _tmp195;
  const Scalar _tmp197 = Scalar(1.0) * _tmp192;
  const Scalar _tmp198 = _tmp180 - _tmp181 * _tmp88 + _tmp183;
  const Scalar _tmp199 = Scalar(9.6622558468725703) * _tmp198;
  const Scalar _tmp200 = std::pow(_tmp184, Scalar(-2));
  const Scalar _tmp201 = _tmp198 * _tmp200;
  const Scalar _tmp202 =
      (_tmp185 * (-_tmp157 * _tmp172 - _tmp171 * _tmp190 + _tmp186 + _tmp187 + _tmp188) -
       _tmp191 * _tmp201) /
      std::sqrt(Scalar(std::pow(_tmp191, Scalar(2)) * _tmp200 + 1));
  const Scalar _tmp203 = _tmp116 * _tmp135 * _tmp50;
  const Scalar _tmp204 = _tmp116 * _tmp126 * _tmp51;
  const Scalar _tmp205 = _tmp179 + _tmp182 + _tmp203 * fh1 + _tmp204 * fh1;
  const Scalar _tmp206 = Scalar(1.0) / (_tmp205);
  const Scalar _tmp207 = _tmp134 * _tmp142 * _tmp50;
  const Scalar _tmp208 = _tmp125 * _tmp142 * _tmp51;
  const Scalar _tmp209 = _tmp148 * _tmp52;
  const Scalar _tmp210 = _tmp159 * _tmp94 - _tmp161 * _tmp163 + _tmp189 - _tmp207 * fh1 -
                         _tmp208 * fh1 - _tmp209 * fh1;
  const Scalar _tmp211 = std::asinh(_tmp206 * _tmp210);
  const Scalar _tmp212 = Scalar(1.0) * _tmp211;
  const Scalar _tmp213 = std::pow(_tmp205, Scalar(-2));
  const Scalar _tmp214 = _tmp181 + _tmp203 + _tmp204;
  const Scalar _tmp215 = _tmp213 * _tmp214;
  const Scalar _tmp216 =
      (_tmp206 * (_tmp143 * _tmp172 + _tmp163 * _tmp171 - _tmp207 - _tmp208 - _tmp209) -
       _tmp210 * _tmp215) /
      std::sqrt(Scalar(std::pow(_tmp210, Scalar(2)) * _tmp213 + 1));
  const Scalar _tmp217 = Scalar(9.6622558468725703) * _tmp205;
  const Scalar _tmp218 =
      -_tmp211 * _tmp217 -
      Scalar(8.3888750099999996) *
          std::sqrt(
              Scalar(Scalar(0.090254729040973036) *
                         std::pow(Scalar(1 - Scalar(0.39679052492160538) * _tmp79), Scalar(2)) +
                     std::pow(Scalar(-Scalar(0.11920549523123722) * _tmp82 - 1), Scalar(2))));
  const Scalar _tmp219 = Scalar(0.1034955) * _tmp206;
  const Scalar _tmp220 = _tmp218 * _tmp219;
  const Scalar _tmp221 = Scalar(9.6622558468725703) * _tmp214;

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
      -_tmp167 *
          (-Scalar(0.86625939559540499) * _tmp175 + Scalar(1.0) * _tmp176 * std::sinh(_tmp178) -
           (-Scalar(0.1034955) * _tmp168 * _tmp175 +
            _tmp169 * (-_tmp166 * _tmp177 - _tmp167 * _tmp176)) *
               std::sinh(_tmp170)) -
      _tmp177 * (Scalar(0.86625939559540499) * _tmp139 - std::cosh(_tmp170) + std::cosh(_tmp178));
  _res(2, 0) =
      -_tmp193 *
          (-Scalar(0.86565325453551001) * _tmp201 + Scalar(1.0) * _tmp202 * std::sinh(_tmp197) -
           (-Scalar(0.1034955) * _tmp194 * _tmp201 +
            _tmp195 * (-_tmp192 * _tmp199 - _tmp193 * _tmp202)) *
               std::sinh(_tmp196)) -
      _tmp199 * (Scalar(0.86565325453551001) * _tmp185 - std::cosh(_tmp196) + std::cosh(_tmp197));
  _res(3, 0) =
      -_tmp217 *
          (-Scalar(0.87653584775870996) * _tmp215 + Scalar(1.0) * _tmp216 * std::sinh(_tmp212) -
           (-Scalar(0.1034955) * _tmp215 * _tmp218 +
            _tmp219 * (-_tmp211 * _tmp221 - _tmp216 * _tmp217)) *
               std::sinh(_tmp220)) -
      _tmp221 * (Scalar(0.87653584775870996) * _tmp206 + std::cosh(_tmp212) - std::cosh(_tmp220));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
