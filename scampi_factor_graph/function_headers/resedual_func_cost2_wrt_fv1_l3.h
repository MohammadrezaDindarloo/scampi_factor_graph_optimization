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
 * Symbolic function: resedual_func_cost2_wrt_fv1_l3
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
Eigen::Matrix<Scalar, 4, 1> ResedualFuncCost2WrtFv1L3(
    const Scalar fh1, const Scalar fv1, const Scalar rx, const Scalar ry, const Scalar rz,
    const Scalar p_init0, const Scalar p_init1, const Scalar p_init2, const Scalar rot_init_x,
    const Scalar rot_init_y, const Scalar rot_init_z, const Scalar rot_init_w,
    const Scalar epsilon) {
  // Total ops: 613

  // Unused inputs
  (void)p_init2;
  (void)epsilon;

  // Input arrays

  // Intermediate terms (206)
  const Scalar _tmp0 = Scalar(1.0) / (fh1);
  const Scalar _tmp1 = std::asinh(_tmp0 * fv1);
  const Scalar _tmp2 = Scalar(1.0) * _tmp0 /
                       std::sqrt(Scalar(1 + std::pow(fv1, Scalar(2)) / std::pow(fh1, Scalar(2))));
  const Scalar _tmp3 = Scalar(9.6622558468725703) * fh1;
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
  const Scalar _tmp13 =
      _tmp10 * rot_init_y + _tmp6 * rot_init_x - _tmp8 * rot_init_z + _tmp9 * rot_init_w;
  const Scalar _tmp14 = 1 - 2 * std::pow(_tmp13, Scalar(2));
  const Scalar _tmp15 = Scalar(0.20999999999999999) * _tmp12 + Scalar(0.20999999999999999) * _tmp14;
  const Scalar _tmp16 =
      -_tmp10 * rot_init_x + _tmp6 * rot_init_y + _tmp8 * rot_init_w + _tmp9 * rot_init_z;
  const Scalar _tmp17 = 2 * _tmp11;
  const Scalar _tmp18 = _tmp16 * _tmp17;
  const Scalar _tmp19 =
      -_tmp10 * rot_init_z + _tmp6 * rot_init_w - _tmp8 * rot_init_y - _tmp9 * rot_init_x;
  const Scalar _tmp20 = 2 * _tmp13;
  const Scalar _tmp21 = _tmp19 * _tmp20;
  const Scalar _tmp22 = _tmp18 - _tmp21;
  const Scalar _tmp23 = Scalar(0.010999999999999999) * _tmp22;
  const Scalar _tmp24 = -_tmp23;
  const Scalar _tmp25 = _tmp16 * _tmp20;
  const Scalar _tmp26 = _tmp17 * _tmp19;
  const Scalar _tmp27 = Scalar(0.20999999999999999) * _tmp25 + Scalar(0.20999999999999999) * _tmp26;
  const Scalar _tmp28 = _tmp24 + _tmp27;
  const Scalar _tmp29 = _tmp15 + _tmp28;
  const Scalar _tmp30 = _tmp29 + p_init1;
  const Scalar _tmp31 = Scalar(0.20999999999999999) * _tmp25 - Scalar(0.20999999999999999) * _tmp26;
  const Scalar _tmp32 = _tmp11 * _tmp20;
  const Scalar _tmp33 = 2 * _tmp16 * _tmp19;
  const Scalar _tmp34 = _tmp32 + _tmp33;
  const Scalar _tmp35 = -Scalar(0.010999999999999999) * _tmp34;
  const Scalar _tmp36 = -2 * std::pow(_tmp16, Scalar(2));
  const Scalar _tmp37 = Scalar(0.20999999999999999) * _tmp12 +
                        Scalar(0.20999999999999999) * _tmp36 + Scalar(0.20999999999999999);
  const Scalar _tmp38 = _tmp35 + _tmp37;
  const Scalar _tmp39 = _tmp31 + _tmp38;
  const Scalar _tmp40 = _tmp39 + p_init0;
  const Scalar _tmp41 = Scalar(0.20999999999999999) * _tmp18 + Scalar(0.20999999999999999) * _tmp21;
  const Scalar _tmp42 = -_tmp41;
  const Scalar _tmp43 =
      -Scalar(0.010999999999999999) * _tmp14 - Scalar(0.010999999999999999) * _tmp36;
  const Scalar _tmp44 = Scalar(0.20999999999999999) * _tmp32 - Scalar(0.20999999999999999) * _tmp33;
  const Scalar _tmp45 = _tmp43 - _tmp44;
  const Scalar _tmp46 = _tmp42 + _tmp45;
  const Scalar _tmp47 = -_tmp15;
  const Scalar _tmp48 = -_tmp27;
  const Scalar _tmp49 = _tmp24 + _tmp48;
  const Scalar _tmp50 = _tmp47 + _tmp49;
  const Scalar _tmp51 = _tmp50 + p_init1;
  const Scalar _tmp52 = _tmp51 + Scalar(8.3196563700000006);
  const Scalar _tmp53 = -_tmp31;
  const Scalar _tmp54 = _tmp35 - _tmp37;
  const Scalar _tmp55 = _tmp53 + _tmp54;
  const Scalar _tmp56 = _tmp55 + p_init0;
  const Scalar _tmp57 = _tmp56 + Scalar(1.9874742000000001);
  const Scalar _tmp58 = std::pow(Scalar(std::pow(_tmp52, Scalar(2)) + std::pow(_tmp57, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp59 = _tmp57 * _tmp58;
  const Scalar _tmp60 = _tmp41 + _tmp45;
  const Scalar _tmp61 = _tmp59 * _tmp60;
  const Scalar _tmp62 = -_tmp46 * _tmp59 + _tmp61;
  const Scalar _tmp63 = _tmp15 + _tmp49;
  const Scalar _tmp64 = _tmp63 + p_init1;
  const Scalar _tmp65 = _tmp64 + Scalar(-4.8333311099999996);
  const Scalar _tmp66 = _tmp31 + _tmp54;
  const Scalar _tmp67 = _tmp66 + p_init0;
  const Scalar _tmp68 = _tmp67 + Scalar(1.79662371);
  const Scalar _tmp69 = Scalar(1.0) / (_tmp68);
  const Scalar _tmp70 = _tmp65 * _tmp69;
  const Scalar _tmp71 = _tmp52 * _tmp58;
  const Scalar _tmp72 = Scalar(1.0) / (_tmp59 * _tmp70 - _tmp71);
  const Scalar _tmp73 = Scalar(1.0) * _tmp72;
  const Scalar _tmp74 = Scalar(1.0) * _tmp63;
  const Scalar _tmp75 = -_tmp74;
  const Scalar _tmp76 = Scalar(1.0) / (_tmp50 + _tmp75);
  const Scalar _tmp77 = Scalar(1.0) * _tmp66;
  const Scalar _tmp78 = _tmp76 * (-_tmp55 + _tmp77);
  const Scalar _tmp79 = _tmp46 * _tmp71 - _tmp61 * _tmp70;
  const Scalar _tmp80 = _tmp73 * _tmp79;
  const Scalar _tmp81 = -_tmp62 * _tmp73 + _tmp78 * _tmp80;
  const Scalar _tmp82 = _tmp43 + _tmp44;
  const Scalar _tmp83 = _tmp42 + _tmp82;
  const Scalar _tmp84 = _tmp38 + _tmp53;
  const Scalar _tmp85 = _tmp84 + p_init0;
  const Scalar _tmp86 = _tmp85 + Scalar(-2.5202214700000001);
  const Scalar _tmp87 = _tmp28 + _tmp47;
  const Scalar _tmp88 = _tmp87 + p_init1;
  const Scalar _tmp89 = _tmp88 + Scalar(8.3888750099999996);
  const Scalar _tmp90 = std::pow(Scalar(std::pow(_tmp86, Scalar(2)) + std::pow(_tmp89, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp91 = _tmp89 * _tmp90;
  const Scalar _tmp92 = _tmp86 * _tmp90;
  const Scalar _tmp93 = _tmp60 * _tmp70;
  const Scalar _tmp94 = _tmp70 * _tmp92 - _tmp91;
  const Scalar _tmp95 = _tmp72 * _tmp94;
  const Scalar _tmp96 = -_tmp79 * _tmp95 + _tmp83 * _tmp91 - _tmp92 * _tmp93;
  const Scalar _tmp97 = _tmp60 * _tmp92 - _tmp62 * _tmp95 - _tmp78 * _tmp96 - _tmp83 * _tmp92;
  const Scalar _tmp98 = Scalar(1.0) / (_tmp97);
  const Scalar _tmp99 =
      std::sqrt(Scalar(std::pow(_tmp65, Scalar(2)) + std::pow(_tmp68, Scalar(2))));
  const Scalar _tmp100 = Scalar(1.0) / (_tmp99);
  const Scalar _tmp101 = _tmp69 * _tmp99;
  const Scalar _tmp102 = _tmp101 * (-_tmp100 * _tmp63 * _tmp68 + _tmp100 * _tmp65 * _tmp66);
  const Scalar _tmp103 = _tmp102 * _tmp59 + _tmp50 * _tmp59 - _tmp55 * _tmp71;
  const Scalar _tmp104 = _tmp102 * _tmp92 - _tmp103 * _tmp95 - _tmp84 * _tmp91 + _tmp87 * _tmp92;
  const Scalar _tmp105 = _tmp104 * _tmp98;
  const Scalar _tmp106 = Scalar(1.0) / (_tmp104);
  const Scalar _tmp107 = _tmp106 * _tmp97;
  const Scalar _tmp108 = _tmp107 * (-_tmp103 * _tmp73 - _tmp105 * _tmp81);
  const Scalar _tmp109 = _tmp108 + _tmp81;
  const Scalar _tmp110 = _tmp94 * _tmp98;
  const Scalar _tmp111 = -_tmp109 * _tmp110 + Scalar(1.0);
  const Scalar _tmp112 = _tmp59 * _tmp72;
  const Scalar _tmp113 = _tmp92 * _tmp98;
  const Scalar _tmp114 = _tmp30 + Scalar(-4.7752063900000001);
  const Scalar _tmp115 = _tmp40 + Scalar(-2.71799795);
  const Scalar _tmp116 =
      std::pow(Scalar(std::pow(_tmp114, Scalar(2)) + std::pow(_tmp115, Scalar(2))),
               Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp117 = _tmp114 * _tmp116;
  const Scalar _tmp118 = _tmp101 * fh1;
  const Scalar _tmp119 = _tmp106 * _tmp73 * _tmp94;
  const Scalar _tmp120 = Scalar(1.0) * _tmp106;
  const Scalar _tmp121 = _tmp115 * _tmp116;
  const Scalar _tmp122 = fh1 * (_tmp117 * _tmp39 - _tmp121 * _tmp29);
  const Scalar _tmp123 = _tmp70 * _tmp72;
  const Scalar _tmp124 = _tmp123 * _tmp79 + _tmp93;
  const Scalar _tmp125 = _tmp123 * _tmp62 - _tmp124 * _tmp78 - _tmp60;
  const Scalar _tmp126 = _tmp107 * (-_tmp102 + _tmp103 * _tmp123 - _tmp105 * _tmp125);
  const Scalar _tmp127 = _tmp125 + _tmp126;
  const Scalar _tmp128 = -_tmp110 * _tmp127 - _tmp70;
  const Scalar _tmp129 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp130 = _tmp74 * _tmp78 + _tmp77;
  const Scalar _tmp131 = 0;
  const Scalar _tmp132 = _tmp131 * _tmp95;
  const Scalar _tmp133 = _tmp101 * (_tmp131 * _tmp92 - _tmp132 * _tmp59);
  const Scalar _tmp134 = -_tmp101 * _tmp122 * (-_tmp119 * _tmp59 + _tmp120 * _tmp92) -
                         _tmp117 * _tmp118 * (_tmp109 * _tmp113 + _tmp111 * _tmp112) -
                         _tmp118 * _tmp121 * (_tmp112 * _tmp128 + _tmp113 * _tmp127 + Scalar(1.0)) -
                         _tmp129 * _tmp133;
  const Scalar _tmp135 = Scalar(1.0) / (_tmp134);
  const Scalar _tmp136 = _tmp75 + _tmp87;
  const Scalar _tmp137 = _tmp136 * _tmp78;
  const Scalar _tmp138 = Scalar(1.0) / (-_tmp137 + _tmp77 - _tmp84);
  const Scalar _tmp139 = Scalar(1.0) * _tmp138;
  const Scalar _tmp140 = _tmp136 * _tmp138;
  const Scalar _tmp141 = _tmp96 * _tmp98;
  const Scalar _tmp142 = _tmp108 * _tmp140 - _tmp109 * _tmp141 - _tmp80;
  const Scalar _tmp143 = Scalar(1.0) * _tmp76;
  const Scalar _tmp144 = Scalar(1.0) * fh1;
  const Scalar _tmp145 = fh1 * (_tmp41 + _tmp82);
  const Scalar _tmp146 = -_tmp117 * _tmp145 - Scalar(3.29616) * _tmp22 - _tmp29 * fv1;
  const Scalar _tmp147 = _tmp137 * _tmp139 + Scalar(1.0);
  const Scalar _tmp148 = _tmp139 * _tmp78;
  const Scalar _tmp149 = -Scalar(1.0) * _tmp143 * _tmp147 + Scalar(1.0) * _tmp148;
  const Scalar _tmp150 = _tmp130 * _tmp138;
  const Scalar _tmp151 = _tmp76 * (-_tmp131 * _tmp96 - _tmp136 * _tmp150 + _tmp75);
  const Scalar _tmp152 = -Scalar(1.0) * _tmp150 - Scalar(1.0) * _tmp151 + Scalar(1.0);
  const Scalar _tmp153 = _tmp121 * _tmp145 + Scalar(3.29616) * _tmp34 + _tmp39 * fv1;
  const Scalar _tmp154 = _tmp136 * _tmp139;
  const Scalar _tmp155 = -Scalar(1.0) * _tmp139 + Scalar(1.0) * _tmp154 * _tmp76;
  const Scalar _tmp156 = _tmp124 + _tmp126 * _tmp140 - _tmp127 * _tmp141;
  const Scalar _tmp157 = _tmp107 * _tmp139;
  const Scalar _tmp158 = _tmp107 * _tmp154 - _tmp120 * _tmp96;
  const Scalar _tmp159 = _tmp117 * _tmp144 * (_tmp108 * _tmp139 - _tmp142 * _tmp143) +
                         _tmp121 * _tmp144 * (_tmp126 * _tmp139 - _tmp143 * _tmp156) +
                         Scalar(1.0) * _tmp122 * (-_tmp143 * _tmp158 + _tmp157) +
                         _tmp129 * _tmp152 + _tmp146 * _tmp149 + _tmp153 * _tmp155;
  const Scalar _tmp160 = std::asinh(_tmp135 * _tmp159);
  const Scalar _tmp161 = Scalar(9.6622558468725703) * _tmp134;
  const Scalar _tmp162 =
      -_tmp160 * _tmp161 -
      Scalar(4.8333311099999996) *
          std::sqrt(
              Scalar(std::pow(Scalar(1 - Scalar(0.20689664689659551) * _tmp64), Scalar(2)) +
                     Scalar(0.13817235445745474) *
                         std::pow(Scalar(-Scalar(0.55659957866191134) * _tmp67 - 1), Scalar(2))));
  const Scalar _tmp163 = Scalar(0.1034955) * _tmp135;
  const Scalar _tmp164 = _tmp162 * _tmp163;
  const Scalar _tmp165 = Scalar(1.0) * _tmp160;
  const Scalar _tmp166 = Scalar(9.6622558468725703) * _tmp133;
  const Scalar _tmp167 = std::pow(_tmp134, Scalar(-2));
  const Scalar _tmp168 = _tmp133 * _tmp167;
  const Scalar _tmp169 = _tmp23 + _tmp47 + _tmp48;
  const Scalar _tmp170 =
      (_tmp135 * (_tmp149 * _tmp169 - _tmp152 + _tmp155 * _tmp39) - _tmp159 * _tmp168) /
      std::sqrt(Scalar(std::pow(_tmp159, Scalar(2)) * _tmp167 + 1));
  const Scalar _tmp171 = _tmp72 * fh1;
  const Scalar _tmp172 = _tmp129 * _tmp131;
  const Scalar _tmp173 = _tmp111 * _tmp117 * _tmp171 - _tmp119 * _tmp122 +
                         _tmp121 * _tmp128 * _tmp171 - _tmp172 * _tmp95;
  const Scalar _tmp174 = Scalar(1.0) / (_tmp173);
  const Scalar _tmp175 = _tmp76 * fh1;
  const Scalar _tmp176 = _tmp139 * _tmp153;
  const Scalar _tmp177 = _tmp136 * _tmp76;
  const Scalar _tmp178 = _tmp147 * _tmp76;
  const Scalar _tmp179 = _tmp117 * _tmp142 * _tmp175 + _tmp121 * _tmp156 * _tmp175 +
                         _tmp122 * _tmp158 * _tmp76 + _tmp129 * _tmp151 + _tmp146 * _tmp178 -
                         _tmp176 * _tmp177;
  const Scalar _tmp180 = std::asinh(_tmp174 * _tmp179);
  const Scalar _tmp181 = Scalar(9.6622558468725703) * _tmp173;
  const Scalar _tmp182 =
      -_tmp180 * _tmp181 -
      Scalar(8.3196563700000006) *
          std::sqrt(
              Scalar(std::pow(Scalar(-Scalar(0.12019727204189803) * _tmp51 - 1), Scalar(2)) +
                     Scalar(0.057067943376852184) *
                         std::pow(Scalar(-Scalar(0.50315118556004401) * _tmp56 - 1), Scalar(2))));
  const Scalar _tmp183 = Scalar(0.1034955) * _tmp174;
  const Scalar _tmp184 = _tmp182 * _tmp183;
  const Scalar _tmp185 = std::pow(_tmp173, Scalar(-2));
  const Scalar _tmp186 = _tmp132 * _tmp185;
  const Scalar _tmp187 = Scalar(9.6622558468725703) * _tmp131;
  const Scalar _tmp188 = _tmp187 * _tmp95;
  const Scalar _tmp189 = _tmp139 * _tmp39;
  const Scalar _tmp190 =
      (_tmp174 * (-_tmp151 + _tmp169 * _tmp178 - _tmp177 * _tmp189) - _tmp179 * _tmp186) /
      std::sqrt(Scalar(std::pow(_tmp179, Scalar(2)) * _tmp185 + 1));
  const Scalar _tmp191 = Scalar(1.0) * _tmp180;
  const Scalar _tmp192 = _tmp138 * fh1;
  const Scalar _tmp193 = -_tmp108 * _tmp117 * _tmp192 - _tmp121 * _tmp126 * _tmp192 -
                         _tmp122 * _tmp157 + _tmp129 * _tmp150 - _tmp146 * _tmp148 + _tmp176;
  const Scalar _tmp194 = _tmp98 * fh1;
  const Scalar _tmp195 =
      _tmp109 * _tmp117 * _tmp194 + _tmp120 * _tmp122 + _tmp121 * _tmp127 * _tmp194 + _tmp172;
  const Scalar _tmp196 = Scalar(1.0) / (_tmp195);
  const Scalar _tmp197 = std::asinh(_tmp193 * _tmp196);
  const Scalar _tmp198 = Scalar(9.6622558468725703) * _tmp195;
  const Scalar _tmp199 =
      -_tmp197 * _tmp198 -
      Scalar(8.3888750099999996) *
          std::sqrt(
              Scalar(Scalar(0.090254729040973036) *
                         std::pow(Scalar(1 - Scalar(0.39679052492160538) * _tmp85), Scalar(2)) +
                     std::pow(Scalar(-Scalar(0.11920549523123722) * _tmp88 - 1), Scalar(2))));
  const Scalar _tmp200 = Scalar(0.1034955) * _tmp196;
  const Scalar _tmp201 = _tmp199 * _tmp200;
  const Scalar _tmp202 = std::pow(_tmp195, Scalar(-2));
  const Scalar _tmp203 = _tmp131 * _tmp202;
  const Scalar _tmp204 = (_tmp193 * _tmp203 + _tmp196 * (-_tmp148 * _tmp169 - _tmp150 + _tmp189)) /
                         std::sqrt(Scalar(std::pow(_tmp193, Scalar(2)) * _tmp202 + 1));
  const Scalar _tmp205 = Scalar(1.0) * _tmp197;

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) =
      _tmp3 *
      (-_tmp2 * std::cosh(Scalar(1.0) * _tmp1) +
       _tmp2 * std::cosh(
                   Scalar(0.1034955) * _tmp0 *
                   (-_tmp1 * _tmp3 -
                    Scalar(4.7752063900000001) *
                        std::sqrt(Scalar(
                            std::pow(Scalar(1 - Scalar(0.20941503221602112) * _tmp30), Scalar(2)) +
                            Scalar(0.32397683292140877) *
                                std::pow(Scalar(1 - Scalar(0.36791786395571047) * _tmp40),
                                         Scalar(2)))))));
  _res(1, 0) = _tmp161 * (-Scalar(1.0) * _tmp170 * std::cosh(_tmp165) -
                          (-Scalar(0.1034955) * _tmp162 * _tmp168 +
                           _tmp163 * (-_tmp160 * _tmp166 - _tmp161 * _tmp170)) *
                              std::cosh(_tmp164)) +
               _tmp166 * (-std::sinh(_tmp164) - std::sinh(_tmp165));
  _res(2, 0) = _tmp181 * (-Scalar(1.0) * _tmp190 * std::cosh(_tmp191) -
                          (-Scalar(0.1034955) * _tmp182 * _tmp186 +
                           _tmp183 * (-_tmp180 * _tmp188 - _tmp181 * _tmp190)) *
                              std::cosh(_tmp184)) +
               _tmp188 * (-std::sinh(_tmp184) - std::sinh(_tmp191));
  _res(3, 0) = -_tmp187 * (-std::sinh(_tmp201) - std::sinh(_tmp205)) +
               _tmp198 * (-Scalar(1.0) * _tmp204 * std::cosh(_tmp205) -
                          (Scalar(0.1034955) * _tmp199 * _tmp203 +
                           _tmp200 * (_tmp187 * _tmp197 - _tmp198 * _tmp204)) *
                              std::cosh(_tmp201));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
