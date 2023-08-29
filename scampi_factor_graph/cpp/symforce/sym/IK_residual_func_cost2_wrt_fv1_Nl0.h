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
 * Symbolic function: IK_residual_func_cost2_wrt_fv1_Nl0
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost2WrtFv1Nl0(
    const Scalar fh1, const Scalar fv1, const Scalar rx, const Scalar ry, const Scalar rz,
    const Scalar p_init0, const Scalar p_init1, const Scalar p_init2, const Scalar rot_init_x,
    const Scalar rot_init_y, const Scalar rot_init_z, const Scalar rot_init_w,
    const Scalar epsilon) {
  // Total ops: 614

  // Unused inputs
  (void)p_init2;
  (void)epsilon;

  // Input arrays

  // Intermediate terms (200)
  const Scalar _tmp0 = Scalar(1.0) / (fh1);
  const Scalar _tmp1 = std::asinh(_tmp0 * fv1);
  const Scalar _tmp2 = Scalar(1.0) * _tmp0 /
                       std::sqrt(Scalar(1 + std::pow(fv1, Scalar(2)) / std::pow(fh1, Scalar(2))));
  const Scalar _tmp3 = std::sqrt(
      Scalar(std::pow(rx, Scalar(2)) + std::pow(ry, Scalar(2)) + std::pow(rz, Scalar(2))));
  const Scalar _tmp4 = (Scalar(1) / Scalar(2)) * _tmp3;
  const Scalar _tmp5 = std::cos(_tmp4);
  const Scalar _tmp6 = std::sin(_tmp4) / _tmp3;
  const Scalar _tmp7 = _tmp6 * ry;
  const Scalar _tmp8 = _tmp6 * rx;
  const Scalar _tmp9 = _tmp6 * rz;
  const Scalar _tmp10 =
      _tmp5 * rot_init_y + _tmp7 * rot_init_w + _tmp8 * rot_init_z - _tmp9 * rot_init_x;
  const Scalar _tmp11 =
      _tmp5 * rot_init_x - _tmp7 * rot_init_z + _tmp8 * rot_init_w + _tmp9 * rot_init_y;
  const Scalar _tmp12 = 2 * _tmp11;
  const Scalar _tmp13 = _tmp10 * _tmp12;
  const Scalar _tmp14 =
      _tmp5 * rot_init_z + _tmp7 * rot_init_x - _tmp8 * rot_init_y + _tmp9 * rot_init_w;
  const Scalar _tmp15 = 2 * _tmp5 * rot_init_w - 2 * _tmp7 * rot_init_y - 2 * _tmp8 * rot_init_x -
                        2 * _tmp9 * rot_init_z;
  const Scalar _tmp16 = _tmp14 * _tmp15;
  const Scalar _tmp17 = Scalar(0.20999999999999999) * _tmp13 + Scalar(0.20999999999999999) * _tmp16;
  const Scalar _tmp18 = -_tmp17;
  const Scalar _tmp19 = -2 * std::pow(_tmp14, Scalar(2));
  const Scalar _tmp20 = 1 - 2 * std::pow(_tmp11, Scalar(2));
  const Scalar _tmp21 = Scalar(0.20999999999999999) * _tmp19 + Scalar(0.20999999999999999) * _tmp20;
  const Scalar _tmp22 = 2 * _tmp10 * _tmp14;
  const Scalar _tmp23 = _tmp11 * _tmp15;
  const Scalar _tmp24 = _tmp22 - _tmp23;
  const Scalar _tmp25 = Scalar(0.010999999999999999) * _tmp24;
  const Scalar _tmp26 = -_tmp25;
  const Scalar _tmp27 = -_tmp21 + _tmp26;
  const Scalar _tmp28 = _tmp18 + _tmp27;
  const Scalar _tmp29 = _tmp28 + p_init1;
  const Scalar _tmp30 = Scalar(0.20999999999999999) * _tmp13 - Scalar(0.20999999999999999) * _tmp16;
  const Scalar _tmp31 = -_tmp30;
  const Scalar _tmp32 = -2 * std::pow(_tmp10, Scalar(2));
  const Scalar _tmp33 = Scalar(0.20999999999999999) * _tmp19 +
                        Scalar(0.20999999999999999) * _tmp32 + Scalar(0.20999999999999999);
  const Scalar _tmp34 = _tmp12 * _tmp14;
  const Scalar _tmp35 = _tmp10 * _tmp15;
  const Scalar _tmp36 = _tmp34 + _tmp35;
  const Scalar _tmp37 = -Scalar(0.010999999999999999) * _tmp36;
  const Scalar _tmp38 = -_tmp33 + _tmp37;
  const Scalar _tmp39 = _tmp31 + _tmp38;
  const Scalar _tmp40 = _tmp39 + p_init0;
  const Scalar _tmp41 = Scalar(9.6622558468725703) * fh1;
  const Scalar _tmp42 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp43 = _tmp21 + _tmp26;
  const Scalar _tmp44 = _tmp17 + _tmp43;
  const Scalar _tmp45 = _tmp44 + p_init1;
  const Scalar _tmp46 = _tmp45 + Scalar(-4.7752063900000001);
  const Scalar _tmp47 = _tmp33 + _tmp37;
  const Scalar _tmp48 = _tmp30 + _tmp47;
  const Scalar _tmp49 = _tmp48 + p_init0;
  const Scalar _tmp50 = _tmp49 + Scalar(-2.71799795);
  const Scalar _tmp51 = std::pow(Scalar(std::pow(_tmp46, Scalar(2)) + std::pow(_tmp50, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp52 = _tmp50 * _tmp51;
  const Scalar _tmp53 = _tmp31 + _tmp47;
  const Scalar _tmp54 = _tmp53 + p_init0;
  const Scalar _tmp55 = _tmp54 + Scalar(-2.5202214700000001);
  const Scalar _tmp56 = Scalar(1.0) / (_tmp55);
  const Scalar _tmp57 = _tmp17 + _tmp27;
  const Scalar _tmp58 = _tmp57 + p_init1;
  const Scalar _tmp59 = _tmp58 + Scalar(8.3888750099999996);
  const Scalar _tmp60 = _tmp56 * _tmp59;
  const Scalar _tmp61 = _tmp46 * _tmp51;
  const Scalar _tmp62 = Scalar(1.0) / (_tmp52 * _tmp60 - _tmp61);
  const Scalar _tmp63 = _tmp18 + _tmp43;
  const Scalar _tmp64 = _tmp63 + p_init1;
  const Scalar _tmp65 = _tmp64 + Scalar(-4.8333311099999996);
  const Scalar _tmp66 = _tmp30 + _tmp38;
  const Scalar _tmp67 = _tmp66 + p_init0;
  const Scalar _tmp68 = _tmp67 + Scalar(1.79662371);
  const Scalar _tmp69 = std::pow(Scalar(std::pow(_tmp65, Scalar(2)) + std::pow(_tmp68, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp70 = _tmp68 * _tmp69;
  const Scalar _tmp71 = _tmp65 * _tmp69;
  const Scalar _tmp72 = _tmp60 * _tmp70 - _tmp71;
  const Scalar _tmp73 = _tmp62 * _tmp72;
  const Scalar _tmp74 = Scalar(0.20999999999999999) * _tmp34 - Scalar(0.20999999999999999) * _tmp35;
  const Scalar _tmp75 =
      -Scalar(0.010999999999999999) * _tmp20 - Scalar(0.010999999999999999) * _tmp32;
  const Scalar _tmp76 = Scalar(0.20999999999999999) * _tmp22 + Scalar(0.20999999999999999) * _tmp23;
  const Scalar _tmp77 = _tmp75 - _tmp76;
  const Scalar _tmp78 = _tmp74 + _tmp77;
  const Scalar _tmp79 = _tmp52 * _tmp78;
  const Scalar _tmp80 = _tmp75 + _tmp76;
  const Scalar _tmp81 = _tmp74 + _tmp80;
  const Scalar _tmp82 = -_tmp60 * _tmp79 + _tmp61 * _tmp81;
  const Scalar _tmp83 = _tmp70 * _tmp78;
  const Scalar _tmp84 = -_tmp74;
  const Scalar _tmp85 = _tmp80 + _tmp84;
  const Scalar _tmp86 = -_tmp60 * _tmp83 + _tmp71 * _tmp85 - _tmp73 * _tmp82;
  const Scalar _tmp87 = Scalar(1.0) * _tmp57;
  const Scalar _tmp88 = -_tmp87;
  const Scalar _tmp89 = Scalar(1.0) / (_tmp44 + _tmp88);
  const Scalar _tmp90 = Scalar(1.0) * _tmp53;
  const Scalar _tmp91 = -_tmp48 + _tmp90;
  const Scalar _tmp92 = _tmp89 * _tmp91;
  const Scalar _tmp93 = -_tmp52 * _tmp81 + _tmp79;
  const Scalar _tmp94 = -_tmp70 * _tmp85 - _tmp73 * _tmp93 + _tmp83 - _tmp86 * _tmp92;
  const Scalar _tmp95 = Scalar(1.0) / (_tmp94);
  const Scalar _tmp96 = _tmp87 * _tmp92 + _tmp90;
  const Scalar _tmp97 = 0;
  const Scalar _tmp98 = _tmp95 * _tmp97;
  const Scalar _tmp99 = _tmp73 * _tmp98;
  const Scalar _tmp100 =
      std::sqrt(Scalar(std::pow(_tmp55, Scalar(2)) + std::pow(_tmp59, Scalar(2))));
  const Scalar _tmp101 = _tmp100 * _tmp56;
  const Scalar _tmp102 = _tmp101 * (-_tmp52 * _tmp99 + _tmp70 * _tmp98);
  const Scalar _tmp103 = Scalar(1.0) / (_tmp100);
  const Scalar _tmp104 = _tmp101 * (_tmp103 * _tmp53 * _tmp59 - _tmp103 * _tmp55 * _tmp57);
  const Scalar _tmp105 = _tmp104 * _tmp52 + _tmp44 * _tmp52 - _tmp48 * _tmp61;
  const Scalar _tmp106 = _tmp104 * _tmp70 - _tmp105 * _tmp73 + _tmp63 * _tmp70 - _tmp66 * _tmp71;
  const Scalar _tmp107 = Scalar(1.0) / (_tmp106);
  const Scalar _tmp108 = Scalar(1.0) * _tmp107;
  const Scalar _tmp109 = Scalar(1.0) * _tmp62;
  const Scalar _tmp110 = _tmp107 * _tmp109 * _tmp72;
  const Scalar _tmp111 = _tmp29 + Scalar(8.3196563700000006);
  const Scalar _tmp112 = _tmp40 + Scalar(1.9874742000000001);
  const Scalar _tmp113 =
      std::pow(Scalar(std::pow(_tmp111, Scalar(2)) + std::pow(_tmp112, Scalar(2))),
               Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp114 = _tmp112 * _tmp113;
  const Scalar _tmp115 = _tmp111 * _tmp113;
  const Scalar _tmp116 = fh1 * (-_tmp114 * _tmp28 + _tmp115 * _tmp39);
  const Scalar _tmp117 = Scalar(1.0) * _tmp89;
  const Scalar _tmp118 = -_tmp109 * _tmp93 + _tmp117 * _tmp62 * _tmp82 * _tmp91;
  const Scalar _tmp119 = _tmp106 * _tmp95;
  const Scalar _tmp120 = _tmp107 * _tmp94;
  const Scalar _tmp121 = _tmp120 * (-_tmp105 * _tmp109 - _tmp118 * _tmp119);
  const Scalar _tmp122 = _tmp118 + _tmp121;
  const Scalar _tmp123 = _tmp70 * _tmp95;
  const Scalar _tmp124 = _tmp72 * _tmp95;
  const Scalar _tmp125 = -_tmp122 * _tmp124 + Scalar(1.0);
  const Scalar _tmp126 = _tmp52 * _tmp62;
  const Scalar _tmp127 = _tmp115 * fh1;
  const Scalar _tmp128 = _tmp60 * _tmp62;
  const Scalar _tmp129 = _tmp128 * _tmp82 + _tmp60 * _tmp78;
  const Scalar _tmp130 = _tmp128 * _tmp93 - _tmp129 * _tmp92 - _tmp78;
  const Scalar _tmp131 = _tmp120 * (-_tmp104 + _tmp105 * _tmp128 - _tmp119 * _tmp130);
  const Scalar _tmp132 = _tmp130 + _tmp131;
  const Scalar _tmp133 = -_tmp124 * _tmp132 - _tmp60;
  const Scalar _tmp134 = _tmp114 * fh1;
  const Scalar _tmp135 = -_tmp101 * _tmp116 * (_tmp108 * _tmp70 - _tmp110 * _tmp52) -
                         _tmp101 * _tmp127 * (_tmp122 * _tmp123 + _tmp125 * _tmp126) -
                         _tmp101 * _tmp134 * (_tmp123 * _tmp132 + _tmp126 * _tmp133 + Scalar(1.0)) -
                         _tmp102 * _tmp42;
  const Scalar _tmp136 = Scalar(1.0) / (_tmp135);
  const Scalar _tmp137 = _tmp63 + _tmp88;
  const Scalar _tmp138 = _tmp137 * _tmp92;
  const Scalar _tmp139 = Scalar(1.0) / (-_tmp138 - _tmp66 + _tmp90);
  const Scalar _tmp140 = _tmp137 * _tmp139;
  const Scalar _tmp141 = _tmp86 * _tmp95;
  const Scalar _tmp142 = -_tmp109 * _tmp82 + _tmp121 * _tmp140 - _tmp122 * _tmp141;
  const Scalar _tmp143 = Scalar(1.0) * _tmp139;
  const Scalar _tmp144 = _tmp129 + _tmp131 * _tmp140 - _tmp132 * _tmp141;
  const Scalar _tmp145 = fh1 * (_tmp77 + _tmp84);
  const Scalar _tmp146 = _tmp114 * _tmp145 + Scalar(3.29616) * _tmp36 + _tmp39 * fv1;
  const Scalar _tmp147 = _tmp137 * _tmp89;
  const Scalar _tmp148 = Scalar(1.0) * _tmp143 * _tmp147 - Scalar(1.0) * _tmp143;
  const Scalar _tmp149 = _tmp139 * _tmp96;
  const Scalar _tmp150 = _tmp89 * (-_tmp137 * _tmp149 - _tmp141 * _tmp97 + _tmp88);
  const Scalar _tmp151 = -Scalar(1.0) * _tmp143 * _tmp96 - Scalar(1.0) * _tmp150 + Scalar(1.0);
  const Scalar _tmp152 = _tmp120 * _tmp143;
  const Scalar _tmp153 = -_tmp108 * _tmp86 + _tmp137 * _tmp152;
  const Scalar _tmp154 = -_tmp115 * _tmp145 - Scalar(3.29616) * _tmp24 - _tmp28 * fv1;
  const Scalar _tmp155 = _tmp138 * _tmp143 + Scalar(1.0);
  const Scalar _tmp156 = _tmp143 * _tmp92;
  const Scalar _tmp157 = -Scalar(1.0) * _tmp117 * _tmp155 + Scalar(1.0) * _tmp156;
  const Scalar _tmp158 = Scalar(1.0) * _tmp116 * (-_tmp117 * _tmp153 + _tmp152) +
                         Scalar(1.0) * _tmp127 * (-_tmp117 * _tmp142 + _tmp121 * _tmp143) +
                         Scalar(1.0) * _tmp134 * (-_tmp117 * _tmp144 + _tmp131 * _tmp143) +
                         _tmp146 * _tmp148 + _tmp151 * _tmp42 + _tmp154 * _tmp157;
  const Scalar _tmp159 = std::asinh(_tmp136 * _tmp158);
  const Scalar _tmp160 = Scalar(9.6622558468725703) * _tmp135;
  const Scalar _tmp161 =
      -_tmp159 * _tmp160 -
      Scalar(8.3888750099999996) *
          std::sqrt(
              Scalar(Scalar(0.090254729040973036) *
                         std::pow(Scalar(1 - Scalar(0.39679052492160538) * _tmp54), Scalar(2)) +
                     std::pow(Scalar(-Scalar(0.11920549523123722) * _tmp58 - 1), Scalar(2))));
  const Scalar _tmp162 = std::pow(_tmp135, Scalar(-2));
  const Scalar _tmp163 = _tmp102 * _tmp162;
  const Scalar _tmp164 = Scalar(9.6622558468725703) * _tmp102;
  const Scalar _tmp165 = _tmp17 + _tmp21 + _tmp25;
  const Scalar _tmp166 =
      (_tmp136 * (_tmp148 * _tmp39 - _tmp151 + _tmp157 * _tmp165) - _tmp158 * _tmp163) /
      std::sqrt(Scalar(std::pow(_tmp158, Scalar(2)) * _tmp162 + 1));
  const Scalar _tmp167 = Scalar(0.1034955) * _tmp136;
  const Scalar _tmp168 = _tmp161 * _tmp167;
  const Scalar _tmp169 = Scalar(1.0) * _tmp159;
  const Scalar _tmp170 = _tmp42 * _tmp98;
  const Scalar _tmp171 = -_tmp110 * _tmp116 + _tmp125 * _tmp127 * _tmp62 +
                         _tmp133 * _tmp134 * _tmp62 - _tmp170 * _tmp73;
  const Scalar _tmp172 = Scalar(1.0) / (_tmp171);
  const Scalar _tmp173 = _tmp143 * _tmp146;
  const Scalar _tmp174 = _tmp155 * _tmp89;
  const Scalar _tmp175 = _tmp116 * _tmp153 * _tmp89 + _tmp127 * _tmp142 * _tmp89 +
                         _tmp134 * _tmp144 * _tmp89 - _tmp147 * _tmp173 + _tmp150 * _tmp42 +
                         _tmp154 * _tmp174;
  const Scalar _tmp176 = std::asinh(_tmp172 * _tmp175);
  const Scalar _tmp177 = Scalar(1.0) * _tmp176;
  const Scalar _tmp178 = std::pow(_tmp171, Scalar(-2));
  const Scalar _tmp179 = _tmp178 * _tmp99;
  const Scalar _tmp180 = _tmp143 * _tmp39;
  const Scalar _tmp181 =
      (_tmp172 * (-_tmp147 * _tmp180 - _tmp150 + _tmp165 * _tmp174) - _tmp175 * _tmp179) /
      std::sqrt(Scalar(std::pow(_tmp175, Scalar(2)) * _tmp178 + 1));
  const Scalar _tmp182 = Scalar(9.6622558468725703) * _tmp171;
  const Scalar _tmp183 =
      -_tmp176 * _tmp182 -
      Scalar(4.7752063900000001) *
          std::sqrt(
              Scalar(std::pow(Scalar(1 - Scalar(0.20941503221602112) * _tmp45), Scalar(2)) +
                     Scalar(0.32397683292140877) *
                         std::pow(Scalar(1 - Scalar(0.36791786395571047) * _tmp49), Scalar(2))));
  const Scalar _tmp184 = Scalar(9.6622558468725703) * _tmp98;
  const Scalar _tmp185 = _tmp184 * _tmp73;
  const Scalar _tmp186 = Scalar(0.1034955) * _tmp172;
  const Scalar _tmp187 = _tmp183 * _tmp186;
  const Scalar _tmp188 = -_tmp116 * _tmp152 - _tmp121 * _tmp127 * _tmp139 -
                         _tmp131 * _tmp134 * _tmp139 + _tmp149 * _tmp42 - _tmp154 * _tmp156 +
                         _tmp173;
  const Scalar _tmp189 =
      _tmp108 * _tmp116 + _tmp122 * _tmp127 * _tmp95 + _tmp132 * _tmp134 * _tmp95 + _tmp170;
  const Scalar _tmp190 = Scalar(1.0) / (_tmp189);
  const Scalar _tmp191 = std::asinh(_tmp188 * _tmp190);
  const Scalar _tmp192 = Scalar(9.6622558468725703) * _tmp189;
  const Scalar _tmp193 =
      -_tmp191 * _tmp192 -
      Scalar(4.8333311099999996) *
          std::sqrt(
              Scalar(std::pow(Scalar(1 - Scalar(0.20689664689659551) * _tmp64), Scalar(2)) +
                     Scalar(0.13817235445745474) *
                         std::pow(Scalar(-Scalar(0.55659957866191134) * _tmp67 - 1), Scalar(2))));
  const Scalar _tmp194 = Scalar(0.1034955) * _tmp190;
  const Scalar _tmp195 = _tmp193 * _tmp194;
  const Scalar _tmp196 = std::pow(_tmp189, Scalar(-2));
  const Scalar _tmp197 = _tmp196 * _tmp98;
  const Scalar _tmp198 = (_tmp188 * _tmp197 + _tmp190 * (-_tmp149 - _tmp156 * _tmp165 + _tmp180)) /
                         std::sqrt(Scalar(std::pow(_tmp188, Scalar(2)) * _tmp196 + 1));
  const Scalar _tmp199 = Scalar(1.0) * _tmp191;

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) =
      _tmp41 *
      (-_tmp2 * std::cosh(Scalar(1.0) * _tmp1) +
       _tmp2 * std::cosh(
                   Scalar(0.1034955) * _tmp0 *
                   (-_tmp1 * _tmp41 -
                    Scalar(8.3196563700000006) *
                        std::sqrt(Scalar(
                            std::pow(Scalar(-Scalar(0.12019727204189803) * _tmp29 - 1), Scalar(2)) +
                            Scalar(0.057067943376852184) *
                                std::pow(Scalar(-Scalar(0.50315118556004401) * _tmp40 - 1),
                                         Scalar(2)))))));
  _res(1, 0) = _tmp160 * (-Scalar(1.0) * _tmp166 * std::cosh(_tmp169) -
                          (-Scalar(0.1034955) * _tmp161 * _tmp163 +
                           _tmp167 * (-_tmp159 * _tmp164 - _tmp160 * _tmp166)) *
                              std::cosh(_tmp168)) +
               _tmp164 * (-std::sinh(_tmp168) - std::sinh(_tmp169));
  _res(2, 0) = _tmp182 * (-Scalar(1.0) * _tmp181 * std::cosh(_tmp177) -
                          (-Scalar(0.1034955) * _tmp179 * _tmp183 +
                           _tmp186 * (-_tmp176 * _tmp185 - _tmp181 * _tmp182)) *
                              std::cosh(_tmp187)) +
               _tmp185 * (-std::sinh(_tmp177) - std::sinh(_tmp187));
  _res(3, 0) = -_tmp184 * (-std::sinh(_tmp195) - std::sinh(_tmp199)) +
               _tmp192 * (-Scalar(1.0) * _tmp198 * std::cosh(_tmp199) -
                          (Scalar(0.1034955) * _tmp193 * _tmp197 +
                           _tmp194 * (_tmp184 * _tmp191 - _tmp192 * _tmp198)) *
                              std::cosh(_tmp195));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
