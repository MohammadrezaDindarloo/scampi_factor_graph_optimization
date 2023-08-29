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
 * Symbolic function: IK_residual_func_cost2_wrt_fh1_Nl20
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost2WrtFh1Nl20(
    const Scalar fh1, const Scalar fv1, const Scalar rx, const Scalar ry, const Scalar rz,
    const Scalar p_init0, const Scalar p_init1, const Scalar p_init2, const Scalar rot_init_x,
    const Scalar rot_init_y, const Scalar rot_init_z, const Scalar rot_init_w,
    const Scalar epsilon) {
  // Total ops: 655

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
  const Scalar _tmp8 = _tmp7 * ry;
  const Scalar _tmp9 = _tmp7 * rx;
  const Scalar _tmp10 = _tmp7 * rz;
  const Scalar _tmp11 =
      -_tmp10 * rot_init_x + _tmp6 * rot_init_y + _tmp8 * rot_init_w + _tmp9 * rot_init_z;
  const Scalar _tmp12 =
      _tmp10 * rot_init_y + _tmp6 * rot_init_x - _tmp8 * rot_init_z + _tmp9 * rot_init_w;
  const Scalar _tmp13 = 2 * _tmp12;
  const Scalar _tmp14 = _tmp11 * _tmp13;
  const Scalar _tmp15 =
      _tmp10 * rot_init_w + _tmp6 * rot_init_z + _tmp8 * rot_init_x - _tmp9 * rot_init_y;
  const Scalar _tmp16 = -2 * _tmp10 * rot_init_z + 2 * _tmp6 * rot_init_w - 2 * _tmp8 * rot_init_y -
                        2 * _tmp9 * rot_init_x;
  const Scalar _tmp17 = _tmp15 * _tmp16;
  const Scalar _tmp18 = Scalar(0.20999999999999999) * _tmp14 + Scalar(0.20999999999999999) * _tmp17;
  const Scalar _tmp19 = -_tmp18;
  const Scalar _tmp20 = -2 * std::pow(_tmp12, Scalar(2));
  const Scalar _tmp21 = -2 * std::pow(_tmp15, Scalar(2));
  const Scalar _tmp22 = Scalar(0.20999999999999999) * _tmp20 +
                        Scalar(0.20999999999999999) * _tmp21 + Scalar(0.20999999999999999);
  const Scalar _tmp23 = 2 * _tmp11 * _tmp15;
  const Scalar _tmp24 = _tmp12 * _tmp16;
  const Scalar _tmp25 = _tmp23 - _tmp24;
  const Scalar _tmp26 = -Scalar(0.010999999999999999) * _tmp25;
  const Scalar _tmp27 = _tmp22 + _tmp26;
  const Scalar _tmp28 = _tmp19 + _tmp27;
  const Scalar _tmp29 = _tmp28 + p_init1;
  const Scalar _tmp30 = Scalar(0.20999999999999999) * _tmp14 - Scalar(0.20999999999999999) * _tmp17;
  const Scalar _tmp31 = _tmp13 * _tmp15;
  const Scalar _tmp32 = _tmp11 * _tmp16;
  const Scalar _tmp33 = _tmp31 + _tmp32;
  const Scalar _tmp34 = -Scalar(0.010999999999999999) * _tmp33;
  const Scalar _tmp35 = 1 - 2 * std::pow(_tmp11, Scalar(2));
  const Scalar _tmp36 = Scalar(0.20999999999999999) * _tmp21 + Scalar(0.20999999999999999) * _tmp35;
  const Scalar _tmp37 = _tmp34 - _tmp36;
  const Scalar _tmp38 = _tmp30 + _tmp37;
  const Scalar _tmp39 = _tmp38 + p_init0;
  const Scalar _tmp40 =
      -_tmp3 * fh1 -
      Scalar(4.8333311099999996) *
          std::sqrt(
              Scalar(std::pow(Scalar(1 - Scalar(0.20689664689659551) * _tmp29), Scalar(2)) +
                     Scalar(0.13817235445745474) *
                         std::pow(Scalar(-Scalar(0.55659957866191134) * _tmp39 - 1), Scalar(2))));
  const Scalar _tmp41 = Scalar(0.1034955) * _tmp0;
  const Scalar _tmp42 = _tmp40 * _tmp41;
  const Scalar _tmp43 = std::pow(fh1, Scalar(-2));
  const Scalar _tmp44 =
      std::pow(Scalar(_tmp43 * std::pow(fv1, Scalar(2)) + 1), Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp45 = Scalar(1.0) * _tmp2;
  const Scalar _tmp46 = -_tmp22 + _tmp26;
  const Scalar _tmp47 = _tmp18 + _tmp46;
  const Scalar _tmp48 = _tmp47 + p_init1;
  const Scalar _tmp49 = -_tmp30;
  const Scalar _tmp50 = _tmp34 + _tmp36;
  const Scalar _tmp51 = _tmp49 + _tmp50;
  const Scalar _tmp52 = _tmp51 + p_init0;
  const Scalar _tmp53 = _tmp29 + Scalar(-4.8333311099999996);
  const Scalar _tmp54 = _tmp39 + Scalar(1.79662371);
  const Scalar _tmp55 = std::pow(Scalar(std::pow(_tmp53, Scalar(2)) + std::pow(_tmp54, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp56 = _tmp54 * _tmp55;
  const Scalar _tmp57 = Scalar(0.20999999999999999) * _tmp31 - Scalar(0.20999999999999999) * _tmp32;
  const Scalar _tmp58 = -_tmp57;
  const Scalar _tmp59 =
      -Scalar(0.010999999999999999) * _tmp20 - Scalar(0.010999999999999999) * _tmp35;
  const Scalar _tmp60 = Scalar(0.20999999999999999) * _tmp23 + Scalar(0.20999999999999999) * _tmp24;
  const Scalar _tmp61 = _tmp59 + _tmp60;
  const Scalar _tmp62 = _tmp58 + _tmp61;
  const Scalar _tmp63 = _tmp62 * fh1;
  const Scalar _tmp64 = Scalar(3.29616) * _tmp33 + _tmp38 * fv1 + _tmp56 * _tmp63;
  const Scalar _tmp65 = Scalar(1.0) * _tmp47;
  const Scalar _tmp66 = -_tmp65;
  const Scalar _tmp67 = _tmp18 + _tmp27;
  const Scalar _tmp68 = _tmp66 + _tmp67;
  const Scalar _tmp69 = _tmp19 + _tmp46;
  const Scalar _tmp70 = Scalar(1.0) / (_tmp66 + _tmp69);
  const Scalar _tmp71 = Scalar(1.0) * _tmp51;
  const Scalar _tmp72 = _tmp37 + _tmp49;
  const Scalar _tmp73 = _tmp71 - _tmp72;
  const Scalar _tmp74 = _tmp70 * _tmp73;
  const Scalar _tmp75 = _tmp68 * _tmp74;
  const Scalar _tmp76 = _tmp30 + _tmp50;
  const Scalar _tmp77 = Scalar(1.0) / (_tmp71 - _tmp75 - _tmp76);
  const Scalar _tmp78 = Scalar(1.0) * _tmp77;
  const Scalar _tmp79 = _tmp68 * _tmp70;
  const Scalar _tmp80 = _tmp78 * _tmp79;
  const Scalar _tmp81 = -Scalar(1.0) * _tmp78 + Scalar(1.0) * _tmp80;
  const Scalar _tmp82 = _tmp52 + Scalar(-2.5202214700000001);
  const Scalar _tmp83 = _tmp48 + Scalar(8.3888750099999996);
  const Scalar _tmp84 =
      std::sqrt(Scalar(std::pow(_tmp82, Scalar(2)) + std::pow(_tmp83, Scalar(2))));
  const Scalar _tmp85 = Scalar(1.0) / (_tmp84);
  const Scalar _tmp86 = Scalar(1.0) / (_tmp82);
  const Scalar _tmp87 = _tmp84 * _tmp86;
  const Scalar _tmp88 = _tmp87 * (-_tmp47 * _tmp82 * _tmp85 + _tmp51 * _tmp83 * _tmp85);
  const Scalar _tmp89 = _tmp59 - _tmp60;
  const Scalar _tmp90 = _tmp57 + _tmp89;
  const Scalar _tmp91 = _tmp58 + _tmp89;
  const Scalar _tmp92 = _tmp69 + p_init1;
  const Scalar _tmp93 = _tmp92 + Scalar(8.3196563700000006);
  const Scalar _tmp94 = _tmp72 + p_init0;
  const Scalar _tmp95 = _tmp94 + Scalar(1.9874742000000001);
  const Scalar _tmp96 = std::pow(Scalar(std::pow(_tmp93, Scalar(2)) + std::pow(_tmp95, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp97 = _tmp95 * _tmp96;
  const Scalar _tmp98 = _tmp90 * _tmp97;
  const Scalar _tmp99 = -_tmp91 * _tmp97 + _tmp98;
  const Scalar _tmp100 = _tmp83 * _tmp86;
  const Scalar _tmp101 = _tmp93 * _tmp96;
  const Scalar _tmp102 = Scalar(1.0) / (_tmp100 * _tmp97 - _tmp101);
  const Scalar _tmp103 = _tmp100 * _tmp102;
  const Scalar _tmp104 = -_tmp100 * _tmp98 + _tmp101 * _tmp91;
  const Scalar _tmp105 = _tmp100 * _tmp90;
  const Scalar _tmp106 = _tmp103 * _tmp104 + _tmp105;
  const Scalar _tmp107 = _tmp103 * _tmp99 - _tmp106 * _tmp74 - _tmp90;
  const Scalar _tmp108 = _tmp67 + p_init1;
  const Scalar _tmp109 = _tmp108 + Scalar(-4.7752063900000001);
  const Scalar _tmp110 = _tmp76 + p_init0;
  const Scalar _tmp111 = _tmp110 + Scalar(-2.71799795);
  const Scalar _tmp112 =
      std::pow(Scalar(std::pow(_tmp109, Scalar(2)) + std::pow(_tmp111, Scalar(2))),
               Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp113 = _tmp111 * _tmp112;
  const Scalar _tmp114 = _tmp109 * _tmp112;
  const Scalar _tmp115 = _tmp100 * _tmp113 - _tmp114;
  const Scalar _tmp116 = _tmp102 * _tmp115;
  const Scalar _tmp117 = _tmp57 + _tmp61;
  const Scalar _tmp118 = -_tmp104 * _tmp116 - _tmp105 * _tmp113 + _tmp114 * _tmp117;
  const Scalar _tmp119 =
      -_tmp113 * _tmp117 + _tmp113 * _tmp90 - _tmp116 * _tmp99 - _tmp118 * _tmp74;
  const Scalar _tmp120 = Scalar(1.0) / (_tmp119);
  const Scalar _tmp121 = -_tmp101 * _tmp72 + _tmp69 * _tmp97 + _tmp88 * _tmp97;
  const Scalar _tmp122 = _tmp113 * _tmp67 + _tmp113 * _tmp88 - _tmp114 * _tmp76 - _tmp116 * _tmp121;
  const Scalar _tmp123 = _tmp120 * _tmp122;
  const Scalar _tmp124 = Scalar(1.0) / (_tmp122);
  const Scalar _tmp125 = _tmp119 * _tmp124;
  const Scalar _tmp126 = _tmp125 * (_tmp103 * _tmp121 - _tmp107 * _tmp123 - _tmp88);
  const Scalar _tmp127 = _tmp68 * _tmp77;
  const Scalar _tmp128 = _tmp107 + _tmp126;
  const Scalar _tmp129 = _tmp118 * _tmp120;
  const Scalar _tmp130 = _tmp106 + _tmp126 * _tmp127 - _tmp128 * _tmp129;
  const Scalar _tmp131 = Scalar(1.0) * _tmp70;
  const Scalar _tmp132 = Scalar(1.0) * _tmp56 * (_tmp126 * _tmp78 - _tmp130 * _tmp131);
  const Scalar _tmp133 = _tmp125 * _tmp78;
  const Scalar _tmp134 = Scalar(1.0) * _tmp124;
  const Scalar _tmp135 = -_tmp118 * _tmp134 + _tmp133 * _tmp68;
  const Scalar _tmp136 = _tmp53 * _tmp55;
  const Scalar _tmp137 = _tmp136 * _tmp38 - _tmp28 * _tmp56;
  const Scalar _tmp138 = Scalar(1.0) * _tmp137;
  const Scalar _tmp139 = _tmp138 * (-_tmp131 * _tmp135 + _tmp133);
  const Scalar _tmp140 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp141 = _tmp65 * _tmp74 + _tmp71;
  const Scalar _tmp142 = _tmp141 * _tmp77;
  const Scalar _tmp143 = 0;
  const Scalar _tmp144 = -_tmp129 * _tmp143 - _tmp142 * _tmp68 + _tmp66;
  const Scalar _tmp145 = -_tmp136 * _tmp63 - Scalar(3.29616) * _tmp25 - _tmp28 * fv1;
  const Scalar _tmp146 = _tmp75 * _tmp78 + Scalar(1.0);
  const Scalar _tmp147 = _tmp74 * _tmp78;
  const Scalar _tmp148 = -Scalar(1.0) * _tmp131 * _tmp146 + Scalar(1.0) * _tmp147;
  const Scalar _tmp149 = Scalar(1.0) * _tmp102;
  const Scalar _tmp150 = _tmp102 * _tmp104 * _tmp131 * _tmp73 - _tmp149 * _tmp99;
  const Scalar _tmp151 = _tmp125 * (-_tmp121 * _tmp149 - _tmp123 * _tmp150);
  const Scalar _tmp152 = _tmp150 + _tmp151;
  const Scalar _tmp153 = -_tmp104 * _tmp149 + _tmp127 * _tmp151 - _tmp129 * _tmp152;
  const Scalar _tmp154 = Scalar(1.0) * _tmp136 * (-_tmp131 * _tmp153 + _tmp151 * _tmp78);
  const Scalar _tmp155 =
      _tmp132 * fh1 + _tmp139 * fh1 +
      Scalar(1.0) * _tmp140 * (-_tmp131 * _tmp144 - _tmp141 * _tmp78 + Scalar(1.0)) +
      _tmp145 * _tmp148 + _tmp154 * fh1 + _tmp64 * _tmp81;
  const Scalar _tmp156 = _tmp120 * _tmp143;
  const Scalar _tmp157 = _tmp116 * _tmp97;
  const Scalar _tmp158 = _tmp115 * _tmp120;
  const Scalar _tmp159 = _tmp102 * (-_tmp100 - _tmp128 * _tmp158);
  const Scalar _tmp160 = _tmp113 * _tmp120;
  const Scalar _tmp161 = _tmp56 * _tmp87 * (_tmp128 * _tmp160 + _tmp159 * _tmp97 + Scalar(1.0));
  const Scalar _tmp162 = _tmp137 * _tmp87 * (_tmp113 * _tmp134 - _tmp134 * _tmp157);
  const Scalar _tmp163 = _tmp102 * (-_tmp152 * _tmp158 + Scalar(1.0));
  const Scalar _tmp164 = _tmp136 * _tmp87 * (_tmp152 * _tmp160 + _tmp163 * _tmp97);
  const Scalar _tmp165 = -_tmp140 * _tmp87 * (_tmp113 * _tmp156 - _tmp156 * _tmp157) -
                         _tmp161 * fh1 - _tmp162 * fh1 - _tmp164 * fh1;
  const Scalar _tmp166 = Scalar(1.0) / (_tmp165);
  const Scalar _tmp167 = std::asinh(_tmp155 * _tmp166);
  const Scalar _tmp168 = Scalar(9.6622558468725703) * _tmp165;
  const Scalar _tmp169 =
      -_tmp167 * _tmp168 -
      Scalar(8.3888750099999996) *
          std::sqrt(
              Scalar(Scalar(0.090254729040973036) *
                         std::pow(Scalar(1 - Scalar(0.39679052492160538) * _tmp52), Scalar(2)) +
                     std::pow(Scalar(-Scalar(0.11920549523123722) * _tmp48 - 1), Scalar(2))));
  const Scalar _tmp170 = Scalar(0.1034955) * _tmp166;
  const Scalar _tmp171 = _tmp169 * _tmp170;
  const Scalar _tmp172 = Scalar(1.0) * _tmp167;
  const Scalar _tmp173 = -_tmp161 - _tmp162 - _tmp164;
  const Scalar _tmp174 = Scalar(9.6622558468725703) * _tmp173;
  const Scalar _tmp175 = std::pow(_tmp165, Scalar(-2));
  const Scalar _tmp176 = _tmp56 * _tmp62;
  const Scalar _tmp177 = _tmp136 * _tmp62;
  const Scalar _tmp178 = _tmp173 * _tmp175;
  const Scalar _tmp179 = (-_tmp155 * _tmp178 + _tmp166 * (_tmp132 + _tmp139 - _tmp148 * _tmp177 +
                                                          _tmp154 + _tmp176 * _tmp81)) /
                         std::sqrt(Scalar(std::pow(_tmp155, Scalar(2)) * _tmp175 + 1));
  const Scalar _tmp180 = _tmp136 * _tmp153 * _tmp70;
  const Scalar _tmp181 = _tmp146 * _tmp70;
  const Scalar _tmp182 = _tmp135 * _tmp137 * _tmp70;
  const Scalar _tmp183 = _tmp64 * _tmp78;
  const Scalar _tmp184 = _tmp130 * _tmp56 * _tmp70;
  const Scalar _tmp185 = _tmp140 * _tmp144 * _tmp70 + _tmp145 * _tmp181 + _tmp180 * fh1 +
                         _tmp182 * fh1 - _tmp183 * _tmp79 + _tmp184 * fh1;
  const Scalar _tmp186 = _tmp140 * _tmp156;
  const Scalar _tmp187 = _tmp124 * _tmp138;
  const Scalar _tmp188 = _tmp187 * fh1;
  const Scalar _tmp189 = _tmp136 * _tmp163;
  const Scalar _tmp190 = _tmp159 * _tmp56;
  const Scalar _tmp191 = -_tmp116 * _tmp186 - _tmp116 * _tmp188 + _tmp189 * fh1 + _tmp190 * fh1;
  const Scalar _tmp192 = Scalar(1.0) / (_tmp191);
  const Scalar _tmp193 = std::asinh(_tmp185 * _tmp192);
  const Scalar _tmp194 = Scalar(9.6622558468725703) * _tmp191;
  const Scalar _tmp195 =
      -_tmp193 * _tmp194 -
      Scalar(8.3196563700000006) *
          std::sqrt(
              Scalar(std::pow(Scalar(-Scalar(0.12019727204189803) * _tmp92 - 1), Scalar(2)) +
                     Scalar(0.057067943376852184) *
                         std::pow(Scalar(-Scalar(0.50315118556004401) * _tmp94 - 1), Scalar(2))));
  const Scalar _tmp196 = Scalar(0.1034955) * _tmp192;
  const Scalar _tmp197 = _tmp195 * _tmp196;
  const Scalar _tmp198 = Scalar(1.0) * _tmp193;
  const Scalar _tmp199 = -_tmp116 * _tmp187 + _tmp189 + _tmp190;
  const Scalar _tmp200 = Scalar(9.6622558468725703) * _tmp199;
  const Scalar _tmp201 = std::pow(_tmp191, Scalar(-2));
  const Scalar _tmp202 = _tmp199 * _tmp201;
  const Scalar _tmp203 = (-_tmp185 * _tmp202 + _tmp192 * (-_tmp176 * _tmp80 - _tmp177 * _tmp181 +
                                                          _tmp180 + _tmp182 + _tmp184)) /
                         std::sqrt(Scalar(std::pow(_tmp185, Scalar(2)) * _tmp201 + 1));
  const Scalar _tmp204 = _tmp120 * _tmp128 * _tmp56;
  const Scalar _tmp205 = _tmp120 * _tmp136 * _tmp152;
  const Scalar _tmp206 = _tmp186 + _tmp188 + _tmp204 * fh1 + _tmp205 * fh1;
  const Scalar _tmp207 = Scalar(1.0) / (_tmp206);
  const Scalar _tmp208 = _tmp126 * _tmp56 * _tmp77;
  const Scalar _tmp209 = _tmp136 * _tmp151 * _tmp77;
  const Scalar _tmp210 = _tmp133 * _tmp137;
  const Scalar _tmp211 = _tmp140 * _tmp142 - _tmp145 * _tmp147 + _tmp183 - _tmp208 * fh1 -
                         _tmp209 * fh1 - _tmp210 * fh1;
  const Scalar _tmp212 = std::asinh(_tmp207 * _tmp211);
  const Scalar _tmp213 = Scalar(1.0) * _tmp212;
  const Scalar _tmp214 = std::pow(_tmp206, Scalar(-2));
  const Scalar _tmp215 = _tmp187 + _tmp204 + _tmp205;
  const Scalar _tmp216 = _tmp214 * _tmp215;
  const Scalar _tmp217 =
      (_tmp207 * (_tmp147 * _tmp177 + _tmp176 * _tmp78 - _tmp208 - _tmp209 - _tmp210) -
       _tmp211 * _tmp216) /
      std::sqrt(Scalar(std::pow(_tmp211, Scalar(2)) * _tmp214 + 1));
  const Scalar _tmp218 = Scalar(9.6622558468725703) * _tmp206;
  const Scalar _tmp219 =
      -_tmp212 * _tmp218 -
      Scalar(4.7752063900000001) *
          std::sqrt(
              Scalar(std::pow(Scalar(1 - Scalar(0.20941503221602112) * _tmp108), Scalar(2)) +
                     Scalar(0.32397683292140877) *
                         std::pow(Scalar(1 - Scalar(0.36791786395571047) * _tmp110), Scalar(2))));
  const Scalar _tmp220 = Scalar(9.6622558468725703) * _tmp215;
  const Scalar _tmp221 = Scalar(0.1034955) * _tmp207;
  const Scalar _tmp222 = _tmp219 * _tmp221;

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) = Scalar(9.6622558468725703) * fh1 *
                   (Scalar(1.0) * _tmp43 * _tmp44 * fv1 * std::cosh(_tmp45) -
                    (-Scalar(0.1034955) * _tmp40 * _tmp43 +
                     _tmp41 * (Scalar(9.6622558468725703) * _tmp1 * _tmp44 - _tmp3)) *
                        std::cosh(_tmp42)) -
               Scalar(9.6622558468725703) * std::sinh(_tmp42) -
               Scalar(9.6622558468725703) * std::sinh(_tmp45);
  _res(1, 0) = _tmp168 * (-Scalar(1.0) * _tmp179 * std::cosh(_tmp172) -
                          (-Scalar(0.1034955) * _tmp169 * _tmp178 +
                           _tmp170 * (-_tmp167 * _tmp174 - _tmp168 * _tmp179)) *
                              std::cosh(_tmp171)) +
               _tmp174 * (-std::sinh(_tmp171) - std::sinh(_tmp172));
  _res(2, 0) = _tmp194 * (-Scalar(1.0) * _tmp203 * std::cosh(_tmp198) -
                          (-Scalar(0.1034955) * _tmp195 * _tmp202 +
                           _tmp196 * (-_tmp193 * _tmp200 - _tmp194 * _tmp203)) *
                              std::cosh(_tmp197)) +
               _tmp200 * (-std::sinh(_tmp197) - std::sinh(_tmp198));
  _res(3, 0) = _tmp218 * (-Scalar(1.0) * _tmp217 * std::cosh(_tmp213) -
                          (-Scalar(0.1034955) * _tmp216 * _tmp219 +
                           _tmp221 * (-_tmp212 * _tmp220 - _tmp217 * _tmp218)) *
                              std::cosh(_tmp222)) +
               _tmp220 * (-std::sinh(_tmp213) - std::sinh(_tmp222));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
