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
 * Symbolic function: IK_residual_func_cost2_wrt_fh1_Nl19
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost2WrtFh1Nl19(
    const Scalar fh1, const Scalar fv1, const Scalar rx, const Scalar ry, const Scalar rz,
    const Scalar p_init0, const Scalar p_init1, const Scalar p_init2, const Scalar rot_init_x,
    const Scalar rot_init_y, const Scalar rot_init_z, const Scalar rot_init_w,
    const Scalar epsilon) {
  // Total ops: 656

  // Unused inputs
  (void)p_init2;
  (void)epsilon;

  // Input arrays

  // Intermediate terms (222)
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
      _tmp10 * rot_init_y + _tmp6 * rot_init_x - _tmp8 * rot_init_z + _tmp9 * rot_init_w;
  const Scalar _tmp12 = -2 * std::pow(_tmp11, Scalar(2));
  const Scalar _tmp13 =
      _tmp10 * rot_init_w + _tmp6 * rot_init_z + _tmp8 * rot_init_x - _tmp9 * rot_init_y;
  const Scalar _tmp14 = 1 - 2 * std::pow(_tmp13, Scalar(2));
  const Scalar _tmp15 = Scalar(0.20999999999999999) * _tmp12 + Scalar(0.20999999999999999) * _tmp14;
  const Scalar _tmp16 =
      -_tmp10 * rot_init_x + _tmp6 * rot_init_y + _tmp8 * rot_init_w + _tmp9 * rot_init_z;
  const Scalar _tmp17 = 2 * _tmp13 * _tmp16;
  const Scalar _tmp18 = -2 * _tmp10 * rot_init_z + 2 * _tmp6 * rot_init_w - 2 * _tmp8 * rot_init_y -
                        2 * _tmp9 * rot_init_x;
  const Scalar _tmp19 = _tmp11 * _tmp18;
  const Scalar _tmp20 = _tmp17 - _tmp19;
  const Scalar _tmp21 = -Scalar(0.010999999999999999) * _tmp20;
  const Scalar _tmp22 = 2 * _tmp11;
  const Scalar _tmp23 = _tmp16 * _tmp22;
  const Scalar _tmp24 = _tmp13 * _tmp18;
  const Scalar _tmp25 = Scalar(0.20999999999999999) * _tmp23 + Scalar(0.20999999999999999) * _tmp24;
  const Scalar _tmp26 = _tmp21 - _tmp25;
  const Scalar _tmp27 = _tmp15 + _tmp26;
  const Scalar _tmp28 = _tmp27 + p_init1;
  const Scalar _tmp29 = -2 * std::pow(_tmp16, Scalar(2));
  const Scalar _tmp30 = Scalar(0.20999999999999999) * _tmp14 + Scalar(0.20999999999999999) * _tmp29;
  const Scalar _tmp31 = -_tmp30;
  const Scalar _tmp32 = _tmp13 * _tmp22;
  const Scalar _tmp33 = _tmp16 * _tmp18;
  const Scalar _tmp34 = _tmp32 + _tmp33;
  const Scalar _tmp35 = -Scalar(0.010999999999999999) * _tmp34;
  const Scalar _tmp36 = Scalar(0.20999999999999999) * _tmp23 - Scalar(0.20999999999999999) * _tmp24;
  const Scalar _tmp37 = _tmp35 + _tmp36;
  const Scalar _tmp38 = _tmp31 + _tmp37;
  const Scalar _tmp39 = _tmp38 + p_init0;
  const Scalar _tmp40 =
      -_tmp3 * fh1 -
      Scalar(4.8333311099999996) *
          std::sqrt(
              Scalar(std::pow(Scalar(1 - Scalar(0.20689664689659551) * _tmp28), Scalar(2)) +
                     Scalar(0.13817235445745474) *
                         std::pow(Scalar(-Scalar(0.55659957866191134) * _tmp39 - 1), Scalar(2))));
  const Scalar _tmp41 = Scalar(0.1034955) * _tmp0;
  const Scalar _tmp42 = _tmp40 * _tmp41;
  const Scalar _tmp43 = std::pow(fh1, Scalar(-2));
  const Scalar _tmp44 =
      std::pow(Scalar(_tmp43 * std::pow(fv1, Scalar(2)) + 1), Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp45 = Scalar(1.0) * _tmp2;
  const Scalar _tmp46 = _tmp35 - _tmp36;
  const Scalar _tmp47 = _tmp30 + _tmp46;
  const Scalar _tmp48 = _tmp31 + _tmp46;
  const Scalar _tmp49 = Scalar(1.0) * _tmp48;
  const Scalar _tmp50 = -_tmp15;
  const Scalar _tmp51 = _tmp26 + _tmp50;
  const Scalar _tmp52 = Scalar(1.0) * _tmp51;
  const Scalar _tmp53 = -_tmp52;
  const Scalar _tmp54 = _tmp21 + _tmp25;
  const Scalar _tmp55 = _tmp50 + _tmp54;
  const Scalar _tmp56 = _tmp53 + _tmp55;
  const Scalar _tmp57 = _tmp15 + _tmp54;
  const Scalar _tmp58 = Scalar(1.0) / (_tmp53 + _tmp57);
  const Scalar _tmp59 = _tmp30 + _tmp37;
  const Scalar _tmp60 = _tmp49 - _tmp59;
  const Scalar _tmp61 = _tmp58 * _tmp60;
  const Scalar _tmp62 = _tmp56 * _tmp61;
  const Scalar _tmp63 = Scalar(1.0) / (-_tmp47 + _tmp49 - _tmp62);
  const Scalar _tmp64 = Scalar(1.0) * _tmp63;
  const Scalar _tmp65 = Scalar(0.20999999999999999) * _tmp32 - Scalar(0.20999999999999999) * _tmp33;
  const Scalar _tmp66 = -Scalar(0.010999999999999999) * _tmp12 -
                        Scalar(0.010999999999999999) * _tmp29 + Scalar(-0.010999999999999999);
  const Scalar _tmp67 = Scalar(0.20999999999999999) * _tmp17 + Scalar(0.20999999999999999) * _tmp19;
  const Scalar _tmp68 = _tmp66 - _tmp67;
  const Scalar _tmp69 = _tmp65 + _tmp68;
  const Scalar _tmp70 = _tmp47 + p_init0;
  const Scalar _tmp71 = _tmp70 + Scalar(-2.5202214700000001);
  const Scalar _tmp72 = _tmp55 + p_init1;
  const Scalar _tmp73 = _tmp72 + Scalar(8.3888750099999996);
  const Scalar _tmp74 = std::pow(Scalar(std::pow(_tmp71, Scalar(2)) + std::pow(_tmp73, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp75 = _tmp73 * _tmp74;
  const Scalar _tmp76 = _tmp51 + p_init1;
  const Scalar _tmp77 = _tmp76 + Scalar(8.3196563700000006);
  const Scalar _tmp78 = _tmp48 + p_init0;
  const Scalar _tmp79 = _tmp78 + Scalar(1.9874742000000001);
  const Scalar _tmp80 = Scalar(1.0) / (_tmp79);
  const Scalar _tmp81 = _tmp77 * _tmp80;
  const Scalar _tmp82 = _tmp71 * _tmp74;
  const Scalar _tmp83 = -_tmp75 + _tmp81 * _tmp82;
  const Scalar _tmp84 = _tmp57 + p_init1;
  const Scalar _tmp85 = _tmp84 + Scalar(-4.7752063900000001);
  const Scalar _tmp86 = _tmp59 + p_init0;
  const Scalar _tmp87 = _tmp86 + Scalar(-2.71799795);
  const Scalar _tmp88 = std::pow(Scalar(std::pow(_tmp85, Scalar(2)) + std::pow(_tmp87, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp89 = _tmp87 * _tmp88;
  const Scalar _tmp90 = _tmp85 * _tmp88;
  const Scalar _tmp91 = Scalar(1.0) / (_tmp81 * _tmp89 - _tmp90);
  const Scalar _tmp92 = -_tmp65;
  const Scalar _tmp93 = _tmp68 + _tmp92;
  const Scalar _tmp94 = _tmp89 * _tmp93;
  const Scalar _tmp95 = _tmp66 + _tmp67;
  const Scalar _tmp96 = _tmp65 + _tmp95;
  const Scalar _tmp97 = _tmp91 * (-_tmp81 * _tmp94 + _tmp90 * _tmp96);
  const Scalar _tmp98 = _tmp82 * _tmp93;
  const Scalar _tmp99 = _tmp69 * _tmp75 - _tmp81 * _tmp98 - _tmp83 * _tmp97;
  const Scalar _tmp100 = _tmp91 * (-_tmp89 * _tmp96 + _tmp94);
  const Scalar _tmp101 = -_tmp100 * _tmp83 - _tmp61 * _tmp99 - _tmp69 * _tmp82 + _tmp98;
  const Scalar _tmp102 =
      std::sqrt(Scalar(std::pow(_tmp77, Scalar(2)) + std::pow(_tmp79, Scalar(2))));
  const Scalar _tmp103 = Scalar(1.0) / (_tmp102);
  const Scalar _tmp104 = _tmp102 * _tmp80;
  const Scalar _tmp105 = _tmp104 * (_tmp103 * _tmp48 * _tmp77 - _tmp103 * _tmp51 * _tmp79);
  const Scalar _tmp106 = _tmp91 * (_tmp105 * _tmp89 + _tmp57 * _tmp89 - _tmp59 * _tmp90);
  const Scalar _tmp107 = _tmp105 * _tmp82 - _tmp106 * _tmp83 - _tmp47 * _tmp75 + _tmp55 * _tmp82;
  const Scalar _tmp108 = Scalar(1.0) / (_tmp107);
  const Scalar _tmp109 = _tmp101 * _tmp108;
  const Scalar _tmp110 = _tmp109 * _tmp64;
  const Scalar _tmp111 = Scalar(1.0) * _tmp108;
  const Scalar _tmp112 = _tmp56 * _tmp64;
  const Scalar _tmp113 = _tmp109 * _tmp112 - _tmp111 * _tmp99;
  const Scalar _tmp114 = Scalar(1.0) * _tmp58;
  const Scalar _tmp115 = _tmp28 + Scalar(-4.8333311099999996);
  const Scalar _tmp116 = _tmp39 + Scalar(1.79662371);
  const Scalar _tmp117 =
      std::pow(Scalar(std::pow(_tmp115, Scalar(2)) + std::pow(_tmp116, Scalar(2))),
               Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp118 = _tmp115 * _tmp117;
  const Scalar _tmp119 = _tmp116 * _tmp117;
  const Scalar _tmp120 = _tmp118 * _tmp38 - _tmp119 * _tmp27;
  const Scalar _tmp121 = Scalar(1.0) * _tmp120;
  const Scalar _tmp122 = _tmp121 * (_tmp110 - _tmp113 * _tmp114);
  const Scalar _tmp123 = _tmp92 + _tmp95;
  const Scalar _tmp124 = _tmp123 * fh1;
  const Scalar _tmp125 = -_tmp118 * _tmp124 - Scalar(3.29616) * _tmp20 - _tmp27 * fv1;
  const Scalar _tmp126 = _tmp62 * _tmp64 + Scalar(1.0);
  const Scalar _tmp127 = _tmp61 * _tmp64;
  const Scalar _tmp128 = -Scalar(1.0) * _tmp114 * _tmp126 + Scalar(1.0) * _tmp127;
  const Scalar _tmp129 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp130 = _tmp49 + _tmp52 * _tmp61;
  const Scalar _tmp131 = _tmp130 * _tmp63;
  const Scalar _tmp132 = Scalar(1.0) / (_tmp101);
  const Scalar _tmp133 = 0;
  const Scalar _tmp134 = -_tmp131 * _tmp56 - _tmp133 * _tmp99 + _tmp53;
  const Scalar _tmp135 = -Scalar(1.0) * _tmp100 + _tmp114 * _tmp60 * _tmp97;
  const Scalar _tmp136 = _tmp107 * _tmp132;
  const Scalar _tmp137 = _tmp109 * (-Scalar(1.0) * _tmp106 - _tmp135 * _tmp136);
  const Scalar _tmp138 = _tmp135 + _tmp137;
  const Scalar _tmp139 = _tmp132 * _tmp99;
  const Scalar _tmp140 = _tmp56 * _tmp63;
  const Scalar _tmp141 = _tmp137 * _tmp140 - _tmp138 * _tmp139 - Scalar(1.0) * _tmp97;
  const Scalar _tmp142 = Scalar(1.0) * _tmp118 * (-_tmp114 * _tmp141 + _tmp137 * _tmp64);
  const Scalar _tmp143 = _tmp81 * _tmp93 + _tmp81 * _tmp97;
  const Scalar _tmp144 = _tmp100 * _tmp81 - _tmp143 * _tmp61 - _tmp93;
  const Scalar _tmp145 = _tmp109 * (-_tmp105 + _tmp106 * _tmp81 - _tmp136 * _tmp144);
  const Scalar _tmp146 = _tmp144 + _tmp145;
  const Scalar _tmp147 = -_tmp139 * _tmp146 + _tmp140 * _tmp145 + _tmp143;
  const Scalar _tmp148 = Scalar(1.0) * _tmp119 * (-_tmp114 * _tmp147 + _tmp145 * _tmp64);
  const Scalar _tmp149 = _tmp119 * _tmp124 + Scalar(3.29616) * _tmp34 + _tmp38 * fv1;
  const Scalar _tmp150 = _tmp112 * _tmp58;
  const Scalar _tmp151 = Scalar(1.0) * _tmp150 - Scalar(1.0) * _tmp64;
  const Scalar _tmp152 =
      _tmp122 * fh1 + _tmp125 * _tmp128 +
      Scalar(1.0) * _tmp129 * (-_tmp114 * _tmp134 - _tmp130 * _tmp64 + Scalar(1.0)) +
      _tmp142 * fh1 + _tmp148 * fh1 + _tmp149 * _tmp151;
  const Scalar _tmp153 = _tmp132 * _tmp146;
  const Scalar _tmp154 = _tmp132 * _tmp83;
  const Scalar _tmp155 = -_tmp146 * _tmp154 - _tmp81;
  const Scalar _tmp156 = _tmp89 * _tmp91;
  const Scalar _tmp157 = _tmp104 * _tmp119 * (_tmp153 * _tmp82 + _tmp155 * _tmp156 + Scalar(1.0));
  const Scalar _tmp158 = _tmp156 * _tmp83;
  const Scalar _tmp159 = _tmp104 * _tmp120 * (-_tmp111 * _tmp158 + _tmp111 * _tmp82);
  const Scalar _tmp160 = _tmp132 * _tmp138;
  const Scalar _tmp161 = -_tmp138 * _tmp154 + Scalar(1.0);
  const Scalar _tmp162 = _tmp104 * _tmp118 * (_tmp156 * _tmp161 + _tmp160 * _tmp82);
  const Scalar _tmp163 = -_tmp104 * _tmp129 * (-_tmp133 * _tmp158 + _tmp133 * _tmp82) -
                         _tmp157 * fh1 - _tmp159 * fh1 - _tmp162 * fh1;
  const Scalar _tmp164 = Scalar(1.0) / (_tmp163);
  const Scalar _tmp165 = std::asinh(_tmp152 * _tmp164);
  const Scalar _tmp166 = Scalar(9.6622558468725703) * _tmp163;
  const Scalar _tmp167 =
      -_tmp165 * _tmp166 -
      Scalar(8.3196563700000006) *
          std::sqrt(
              Scalar(std::pow(Scalar(-Scalar(0.12019727204189803) * _tmp76 - 1), Scalar(2)) +
                     Scalar(0.057067943376852184) *
                         std::pow(Scalar(-Scalar(0.50315118556004401) * _tmp78 - 1), Scalar(2))));
  const Scalar _tmp168 = Scalar(0.1034955) * _tmp164;
  const Scalar _tmp169 = _tmp167 * _tmp168;
  const Scalar _tmp170 = std::pow(_tmp163, Scalar(-2));
  const Scalar _tmp171 = -_tmp157 - _tmp159 - _tmp162;
  const Scalar _tmp172 = _tmp170 * _tmp171;
  const Scalar _tmp173 = _tmp118 * _tmp123;
  const Scalar _tmp174 = _tmp119 * _tmp123;
  const Scalar _tmp175 = (-_tmp152 * _tmp172 + _tmp164 * (_tmp122 - _tmp128 * _tmp173 + _tmp142 +
                                                          _tmp148 + _tmp151 * _tmp174)) /
                         std::sqrt(Scalar(std::pow(_tmp152, Scalar(2)) * _tmp170 + 1));
  const Scalar _tmp176 = Scalar(9.6622558468725703) * _tmp171;
  const Scalar _tmp177 = Scalar(1.0) * _tmp165;
  const Scalar _tmp178 = _tmp126 * _tmp58;
  const Scalar _tmp179 = _tmp149 * _tmp64;
  const Scalar _tmp180 = _tmp118 * _tmp141 * _tmp58;
  const Scalar _tmp181 = _tmp113 * _tmp120 * _tmp58;
  const Scalar _tmp182 = _tmp119 * _tmp147 * _tmp58;
  const Scalar _tmp183 = _tmp125 * _tmp178 + _tmp129 * _tmp134 * _tmp58 -
                         _tmp179 * _tmp56 * _tmp58 + _tmp180 * fh1 + _tmp181 * fh1 + _tmp182 * fh1;
  const Scalar _tmp184 = _tmp119 * _tmp155 * _tmp91;
  const Scalar _tmp185 = _tmp129 * _tmp133;
  const Scalar _tmp186 = _tmp83 * _tmp91;
  const Scalar _tmp187 = _tmp108 * _tmp121;
  const Scalar _tmp188 = _tmp187 * fh1;
  const Scalar _tmp189 = _tmp118 * _tmp161 * _tmp91;
  const Scalar _tmp190 = _tmp184 * fh1 - _tmp185 * _tmp186 - _tmp186 * _tmp188 + _tmp189 * fh1;
  const Scalar _tmp191 = Scalar(1.0) / (_tmp190);
  const Scalar _tmp192 = std::asinh(_tmp183 * _tmp191);
  const Scalar _tmp193 = Scalar(9.6622558468725703) * _tmp190;
  const Scalar _tmp194 =
      -_tmp192 * _tmp193 -
      Scalar(4.7752063900000001) *
          std::sqrt(
              Scalar(std::pow(Scalar(1 - Scalar(0.20941503221602112) * _tmp84), Scalar(2)) +
                     Scalar(0.32397683292140877) *
                         std::pow(Scalar(1 - Scalar(0.36791786395571047) * _tmp86), Scalar(2))));
  const Scalar _tmp195 = Scalar(0.1034955) * _tmp191;
  const Scalar _tmp196 = _tmp194 * _tmp195;
  const Scalar _tmp197 = Scalar(1.0) * _tmp192;
  const Scalar _tmp198 = _tmp184 - _tmp186 * _tmp187 + _tmp189;
  const Scalar _tmp199 = Scalar(9.6622558468725703) * _tmp198;
  const Scalar _tmp200 = std::pow(_tmp190, Scalar(-2));
  const Scalar _tmp201 = _tmp198 * _tmp200;
  const Scalar _tmp202 = (-_tmp183 * _tmp201 + _tmp191 * (-_tmp150 * _tmp174 - _tmp173 * _tmp178 +
                                                          _tmp180 + _tmp181 + _tmp182)) /
                         std::sqrt(Scalar(std::pow(_tmp183, Scalar(2)) * _tmp200 + 1));
  const Scalar _tmp203 = _tmp118 * _tmp160;
  const Scalar _tmp204 = _tmp119 * _tmp153;
  const Scalar _tmp205 = _tmp187 + _tmp203 + _tmp204;
  const Scalar _tmp206 = _tmp110 * _tmp120;
  const Scalar _tmp207 = _tmp118 * _tmp137 * _tmp63;
  const Scalar _tmp208 = _tmp119 * _tmp145 * _tmp63;
  const Scalar _tmp209 = -_tmp125 * _tmp127 + _tmp129 * _tmp131 + _tmp179 - _tmp206 * fh1 -
                         _tmp207 * fh1 - _tmp208 * fh1;
  const Scalar _tmp210 = _tmp185 + _tmp188 + _tmp203 * fh1 + _tmp204 * fh1;
  const Scalar _tmp211 = Scalar(1.0) / (_tmp210);
  const Scalar _tmp212 = std::asinh(_tmp209 * _tmp211);
  const Scalar _tmp213 = Scalar(1.0) * _tmp212;
  const Scalar _tmp214 = Scalar(9.6622558468725703) * _tmp212;
  const Scalar _tmp215 =
      -_tmp210 * _tmp214 -
      Scalar(8.3888750099999996) *
          std::sqrt(
              Scalar(Scalar(0.090254729040973036) *
                         std::pow(Scalar(1 - Scalar(0.39679052492160538) * _tmp70), Scalar(2)) +
                     std::pow(Scalar(-Scalar(0.11920549523123722) * _tmp72 - 1), Scalar(2))));
  const Scalar _tmp216 = Scalar(0.1034955) * _tmp211;
  const Scalar _tmp217 = _tmp215 * _tmp216;
  const Scalar _tmp218 = std::pow(_tmp210, Scalar(-2));
  const Scalar _tmp219 = _tmp205 * _tmp218;
  const Scalar _tmp220 = (-_tmp209 * _tmp219 + _tmp211 * (_tmp127 * _tmp173 + _tmp174 * _tmp64 -
                                                          _tmp206 - _tmp207 - _tmp208)) /
                         std::sqrt(Scalar(std::pow(_tmp209, Scalar(2)) * _tmp218 + 1));
  const Scalar _tmp221 = Scalar(9.6622558468725703) * _tmp210;

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) = Scalar(9.6622558468725703) * fh1 *
                   (Scalar(1.0) * _tmp43 * _tmp44 * fv1 * std::cosh(_tmp45) -
                    (-Scalar(0.1034955) * _tmp40 * _tmp43 +
                     _tmp41 * (Scalar(9.6622558468725703) * _tmp1 * _tmp44 - _tmp3)) *
                        std::cosh(_tmp42)) -
               Scalar(9.6622558468725703) * std::sinh(_tmp42) -
               Scalar(9.6622558468725703) * std::sinh(_tmp45);
  _res(1, 0) = _tmp166 * (-Scalar(1.0) * _tmp175 * std::cosh(_tmp177) -
                          (-Scalar(0.1034955) * _tmp167 * _tmp172 +
                           _tmp168 * (-_tmp165 * _tmp176 - _tmp166 * _tmp175)) *
                              std::cosh(_tmp169)) +
               _tmp176 * (-std::sinh(_tmp169) - std::sinh(_tmp177));
  _res(2, 0) = _tmp193 * (-Scalar(1.0) * _tmp202 * std::cosh(_tmp197) -
                          (-Scalar(0.1034955) * _tmp194 * _tmp201 +
                           _tmp195 * (-_tmp192 * _tmp199 - _tmp193 * _tmp202)) *
                              std::cosh(_tmp196)) +
               _tmp199 * (-std::sinh(_tmp196) - std::sinh(_tmp197));
  _res(3, 0) = Scalar(9.6622558468725703) * _tmp205 * (-std::sinh(_tmp213) - std::sinh(_tmp217)) +
               _tmp221 * (-Scalar(1.0) * _tmp220 * std::cosh(_tmp213) -
                          (-Scalar(0.1034955) * _tmp215 * _tmp219 +
                           _tmp216 * (-_tmp205 * _tmp214 - _tmp220 * _tmp221)) *
                              std::cosh(_tmp217));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym