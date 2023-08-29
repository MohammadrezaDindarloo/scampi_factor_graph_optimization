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
 * Symbolic function: IK_residual_func_cost1_wrt_fv1_Nl21
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost1WrtFv1Nl21(
    const Scalar fh1, const Scalar fv1, const Scalar rx, const Scalar ry, const Scalar rz,
    const Scalar p_init0, const Scalar p_init1, const Scalar p_init2, const Scalar rot_init_x,
    const Scalar rot_init_y, const Scalar rot_init_z, const Scalar rot_init_w,
    const Scalar epsilon) {
  // Total ops: 620

  // Unused inputs
  (void)p_init2;
  (void)epsilon;

  // Input arrays

  // Intermediate terms (200)
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
  const Scalar _tmp10 = _tmp7 * rot_init_x;
  const Scalar _tmp11 = -_tmp10 * rz + _tmp6 * rot_init_y + _tmp8 * rot_init_w + _tmp9 * rot_init_z;
  const Scalar _tmp12 = _tmp7 * rz;
  const Scalar _tmp13 =
      _tmp12 * rot_init_y + _tmp6 * rot_init_x - _tmp8 * rot_init_z + _tmp9 * rot_init_w;
  const Scalar _tmp14 = 2 * _tmp13;
  const Scalar _tmp15 = _tmp11 * _tmp14;
  const Scalar _tmp16 = _tmp10 * ry + _tmp12 * rot_init_w + _tmp6 * rot_init_z - _tmp9 * rot_init_y;
  const Scalar _tmp17 =
      -2 * _tmp10 * rx - 2 * _tmp12 * rot_init_z + 2 * _tmp6 * rot_init_w - 2 * _tmp8 * rot_init_y;
  const Scalar _tmp18 = _tmp16 * _tmp17;
  const Scalar _tmp19 = Scalar(0.20999999999999999) * _tmp15 + Scalar(0.20999999999999999) * _tmp18;
  const Scalar _tmp20 = -_tmp19;
  const Scalar _tmp21 = 2 * _tmp11 * _tmp16;
  const Scalar _tmp22 = _tmp13 * _tmp17;
  const Scalar _tmp23 = _tmp21 - _tmp22;
  const Scalar _tmp24 = Scalar(0.010999999999999999) * _tmp23;
  const Scalar _tmp25 = -_tmp24;
  const Scalar _tmp26 = -2 * std::pow(_tmp16, Scalar(2));
  const Scalar _tmp27 = 1 - 2 * std::pow(_tmp13, Scalar(2));
  const Scalar _tmp28 = Scalar(0.20999999999999999) * _tmp26 + Scalar(0.20999999999999999) * _tmp27;
  const Scalar _tmp29 = _tmp25 + _tmp28;
  const Scalar _tmp30 = _tmp20 + _tmp29;
  const Scalar _tmp31 = _tmp30 + p_init1;
  const Scalar _tmp32 = -2 * std::pow(_tmp11, Scalar(2));
  const Scalar _tmp33 = Scalar(0.20999999999999999) * _tmp26 +
                        Scalar(0.20999999999999999) * _tmp32 + Scalar(0.20999999999999999);
  const Scalar _tmp34 = -_tmp33;
  const Scalar _tmp35 = _tmp14 * _tmp16;
  const Scalar _tmp36 = _tmp11 * _tmp17;
  const Scalar _tmp37 = _tmp35 + _tmp36;
  const Scalar _tmp38 = -Scalar(0.010999999999999999) * _tmp37;
  const Scalar _tmp39 = Scalar(0.20999999999999999) * _tmp15 - Scalar(0.20999999999999999) * _tmp18;
  const Scalar _tmp40 = _tmp38 + _tmp39;
  const Scalar _tmp41 = _tmp34 + _tmp40;
  const Scalar _tmp42 = _tmp41 + p_init0;
  const Scalar _tmp43 = -_tmp28;
  const Scalar _tmp44 = _tmp25 + _tmp43;
  const Scalar _tmp45 = _tmp19 + _tmp44;
  const Scalar _tmp46 = _tmp45 + p_init1;
  const Scalar _tmp47 = _tmp38 - _tmp39;
  const Scalar _tmp48 = _tmp33 + _tmp47;
  const Scalar _tmp49 = _tmp48 + p_init0;
  const Scalar _tmp50 = _tmp19 + _tmp29;
  const Scalar _tmp51 = _tmp50 + p_init1;
  const Scalar _tmp52 = _tmp51 + Scalar(-4.7752063900000001);
  const Scalar _tmp53 = _tmp33 + _tmp40;
  const Scalar _tmp54 = _tmp53 + p_init0;
  const Scalar _tmp55 = _tmp54 + Scalar(-2.71799795);
  const Scalar _tmp56 = std::pow(Scalar(std::pow(_tmp52, Scalar(2)) + std::pow(_tmp55, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp57 = _tmp55 * _tmp56;
  const Scalar _tmp58 = _tmp20 + _tmp44;
  const Scalar _tmp59 = _tmp58 + p_init1;
  const Scalar _tmp60 = _tmp59 + Scalar(8.3196563700000006);
  const Scalar _tmp61 = _tmp34 + _tmp47;
  const Scalar _tmp62 = _tmp61 + p_init0;
  const Scalar _tmp63 = _tmp62 + Scalar(1.9874742000000001);
  const Scalar _tmp64 = std::pow(Scalar(std::pow(_tmp60, Scalar(2)) + std::pow(_tmp63, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp65 = _tmp63 * _tmp64;
  const Scalar _tmp66 = _tmp49 + Scalar(-2.5202214700000001);
  const Scalar _tmp67 = _tmp46 + Scalar(8.3888750099999996);
  const Scalar _tmp68 =
      std::sqrt(Scalar(std::pow(_tmp66, Scalar(2)) + std::pow(_tmp67, Scalar(2))));
  const Scalar _tmp69 = Scalar(1.0) / (_tmp68);
  const Scalar _tmp70 = Scalar(1.0) / (_tmp66);
  const Scalar _tmp71 = _tmp68 * _tmp70;
  const Scalar _tmp72 = _tmp71 * (-_tmp45 * _tmp66 * _tmp69 + _tmp48 * _tmp67 * _tmp69);
  const Scalar _tmp73 = _tmp52 * _tmp56;
  const Scalar _tmp74 = _tmp50 * _tmp57 - _tmp53 * _tmp73 + _tmp57 * _tmp72;
  const Scalar _tmp75 = _tmp67 * _tmp70;
  const Scalar _tmp76 = Scalar(1.0) / (_tmp57 * _tmp75 - _tmp73);
  const Scalar _tmp77 = _tmp60 * _tmp64;
  const Scalar _tmp78 = _tmp65 * _tmp75 - _tmp77;
  const Scalar _tmp79 = _tmp76 * _tmp78;
  const Scalar _tmp80 = _tmp58 * _tmp65 - _tmp61 * _tmp77 + _tmp65 * _tmp72 - _tmp74 * _tmp79;
  const Scalar _tmp81 = Scalar(1.0) / (_tmp80);
  const Scalar _tmp82 = Scalar(1.0) * _tmp76;
  const Scalar _tmp83 = _tmp78 * _tmp81 * _tmp82;
  const Scalar _tmp84 = Scalar(1.0) * _tmp81;
  const Scalar _tmp85 = _tmp31 + Scalar(-4.8333311099999996);
  const Scalar _tmp86 = _tmp42 + Scalar(1.79662371);
  const Scalar _tmp87 = std::pow(Scalar(std::pow(_tmp85, Scalar(2)) + std::pow(_tmp86, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp88 = _tmp85 * _tmp87;
  const Scalar _tmp89 = _tmp86 * _tmp87;
  const Scalar _tmp90 = fh1 * (-_tmp30 * _tmp89 + _tmp41 * _tmp88);
  const Scalar _tmp91 = _tmp75 * _tmp76;
  const Scalar _tmp92 = Scalar(0.20999999999999999) * _tmp21 + Scalar(0.20999999999999999) * _tmp22;
  const Scalar _tmp93 = -_tmp92;
  const Scalar _tmp94 =
      -Scalar(0.010999999999999999) * _tmp27 - Scalar(0.010999999999999999) * _tmp32;
  const Scalar _tmp95 = Scalar(0.20999999999999999) * _tmp35 - Scalar(0.20999999999999999) * _tmp36;
  const Scalar _tmp96 = _tmp94 + _tmp95;
  const Scalar _tmp97 = _tmp93 + _tmp96;
  const Scalar _tmp98 = _tmp57 * _tmp97;
  const Scalar _tmp99 = _tmp92 + _tmp96;
  const Scalar _tmp100 = -_tmp57 * _tmp99 + _tmp98;
  const Scalar _tmp101 = _tmp73 * _tmp99 - _tmp75 * _tmp98;
  const Scalar _tmp102 = _tmp101 * _tmp91 + _tmp75 * _tmp97;
  const Scalar _tmp103 = Scalar(1.0) * _tmp45;
  const Scalar _tmp104 = -_tmp103;
  const Scalar _tmp105 = Scalar(1.0) / (_tmp104 + _tmp50);
  const Scalar _tmp106 = Scalar(1.0) * _tmp48;
  const Scalar _tmp107 = _tmp105 * (_tmp106 - _tmp53);
  const Scalar _tmp108 = _tmp100 * _tmp91 - _tmp102 * _tmp107 - _tmp97;
  const Scalar _tmp109 = _tmp94 - _tmp95;
  const Scalar _tmp110 = _tmp109 + _tmp93;
  const Scalar _tmp111 = _tmp65 * _tmp97;
  const Scalar _tmp112 = -_tmp101 * _tmp79 + _tmp110 * _tmp77 - _tmp111 * _tmp75;
  const Scalar _tmp113 = -_tmp100 * _tmp79 - _tmp107 * _tmp112 - _tmp110 * _tmp65 + _tmp111;
  const Scalar _tmp114 = Scalar(1.0) / (_tmp113);
  const Scalar _tmp115 = _tmp114 * _tmp80;
  const Scalar _tmp116 = _tmp113 * _tmp81;
  const Scalar _tmp117 = _tmp116 * (-_tmp108 * _tmp115 - _tmp72 + _tmp74 * _tmp91);
  const Scalar _tmp118 = _tmp108 + _tmp117;
  const Scalar _tmp119 = _tmp114 * _tmp65;
  const Scalar _tmp120 = _tmp114 * _tmp78;
  const Scalar _tmp121 = -_tmp118 * _tmp120 - _tmp75;
  const Scalar _tmp122 = _tmp57 * _tmp76;
  const Scalar _tmp123 = _tmp89 * fh1;
  const Scalar _tmp124 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp125 = _tmp103 * _tmp107 + _tmp106;
  const Scalar _tmp126 = 0;
  const Scalar _tmp127 = _tmp126 * _tmp79;
  const Scalar _tmp128 = _tmp71 * (_tmp126 * _tmp65 - _tmp127 * _tmp57);
  const Scalar _tmp129 = _tmp101 * _tmp82;
  const Scalar _tmp130 = -_tmp100 * _tmp82 + _tmp107 * _tmp129;
  const Scalar _tmp131 = _tmp116 * (-_tmp115 * _tmp130 - _tmp74 * _tmp82);
  const Scalar _tmp132 = _tmp130 + _tmp131;
  const Scalar _tmp133 = -_tmp120 * _tmp132 + Scalar(1.0);
  const Scalar _tmp134 = _tmp88 * fh1;
  const Scalar _tmp135 = -_tmp123 * _tmp71 * (_tmp118 * _tmp119 + _tmp121 * _tmp122 + Scalar(1.0)) -
                         _tmp124 * _tmp128 -
                         _tmp134 * _tmp71 * (_tmp119 * _tmp132 + _tmp122 * _tmp133) -
                         _tmp71 * _tmp90 * (-_tmp57 * _tmp83 + _tmp65 * _tmp84);
  const Scalar _tmp136 = Scalar(1.0) / (_tmp135);
  const Scalar _tmp137 = _tmp104 + _tmp58;
  const Scalar _tmp138 = _tmp107 * _tmp137;
  const Scalar _tmp139 = Scalar(1.0) / (_tmp106 - _tmp138 - _tmp61);
  const Scalar _tmp140 = Scalar(1.0) * _tmp139;
  const Scalar _tmp141 = _tmp116 * _tmp140;
  const Scalar _tmp142 = -_tmp112 * _tmp84 + _tmp137 * _tmp141;
  const Scalar _tmp143 = Scalar(1.0) * _tmp105;
  const Scalar _tmp144 = _tmp112 * _tmp114;
  const Scalar _tmp145 = _tmp137 * _tmp139;
  const Scalar _tmp146 = _tmp102 + _tmp117 * _tmp145 - _tmp118 * _tmp144;
  const Scalar _tmp147 = fh1 * (_tmp109 + _tmp92);
  const Scalar _tmp148 = _tmp147 * _tmp89 + Scalar(3.29616) * _tmp37 + _tmp41 * fv1;
  const Scalar _tmp149 = _tmp105 * _tmp137;
  const Scalar _tmp150 = Scalar(1.0) * _tmp140 * _tmp149 - Scalar(1.0) * _tmp140;
  const Scalar _tmp151 = -_tmp129 + _tmp131 * _tmp145 - _tmp132 * _tmp144;
  const Scalar _tmp152 = _tmp125 * _tmp139;
  const Scalar _tmp153 = _tmp105 * (_tmp104 - _tmp112 * _tmp126 - _tmp137 * _tmp152);
  const Scalar _tmp154 = -Scalar(1.0) * _tmp152 - Scalar(1.0) * _tmp153 + Scalar(1.0);
  const Scalar _tmp155 = -_tmp147 * _tmp88 - Scalar(3.29616) * _tmp23 - _tmp30 * fv1;
  const Scalar _tmp156 = _tmp107 * _tmp140;
  const Scalar _tmp157 = _tmp105 * (_tmp138 * _tmp140 + Scalar(1.0));
  const Scalar _tmp158 = Scalar(1.0) * _tmp156 - Scalar(1.0) * _tmp157;
  const Scalar _tmp159 =
      Scalar(1.0) * _tmp123 * (_tmp117 * _tmp140 - _tmp143 * _tmp146) + _tmp124 * _tmp154 +
      Scalar(1.0) * _tmp134 * (_tmp131 * _tmp140 - _tmp143 * _tmp151) + _tmp148 * _tmp150 +
      _tmp155 * _tmp158 + Scalar(1.0) * _tmp90 * (_tmp141 - _tmp142 * _tmp143);
  const Scalar _tmp160 = std::asinh(_tmp136 * _tmp159);
  const Scalar _tmp161 = Scalar(9.6622558468725703) * _tmp135;
  const Scalar _tmp162 =
      -_tmp160 * _tmp161 -
      Scalar(8.3888750099999996) *
          std::sqrt(
              Scalar(Scalar(0.090254729040973036) *
                         std::pow(Scalar(1 - Scalar(0.39679052492160538) * _tmp49), Scalar(2)) +
                     std::pow(Scalar(-Scalar(0.11920549523123722) * _tmp46 - 1), Scalar(2))));
  const Scalar _tmp163 = Scalar(0.1034955) * _tmp136;
  const Scalar _tmp164 = _tmp162 * _tmp163;
  const Scalar _tmp165 = Scalar(1.0) * _tmp160;
  const Scalar _tmp166 = Scalar(9.6622558468725703) * _tmp128;
  const Scalar _tmp167 = std::pow(_tmp135, Scalar(-2));
  const Scalar _tmp168 = _tmp128 * _tmp167;
  const Scalar _tmp169 = _tmp19 + _tmp24 + _tmp43;
  const Scalar _tmp170 =
      (_tmp136 * (_tmp150 * _tmp41 - _tmp154 + _tmp158 * _tmp169) - _tmp159 * _tmp168) /
      std::sqrt(Scalar(std::pow(_tmp159, Scalar(2)) * _tmp167 + 1));
  const Scalar _tmp171 = _tmp124 * _tmp126;
  const Scalar _tmp172 =
      _tmp121 * _tmp123 * _tmp76 + _tmp133 * _tmp134 * _tmp76 - _tmp171 * _tmp79 - _tmp83 * _tmp90;
  const Scalar _tmp173 = Scalar(1.0) / (_tmp172);
  const Scalar _tmp174 = _tmp140 * _tmp148;
  const Scalar _tmp175 = _tmp105 * _tmp123 * _tmp146 + _tmp105 * _tmp134 * _tmp151 +
                         _tmp105 * _tmp142 * _tmp90 + _tmp124 * _tmp153 - _tmp149 * _tmp174 +
                         _tmp155 * _tmp157;
  const Scalar _tmp176 = std::asinh(_tmp173 * _tmp175);
  const Scalar _tmp177 = Scalar(1.0) * _tmp176;
  const Scalar _tmp178 = Scalar(9.6622558468725703) * _tmp172;
  const Scalar _tmp179 =
      -_tmp176 * _tmp178 -
      Scalar(4.7752063900000001) *
          std::sqrt(
              Scalar(std::pow(Scalar(1 - Scalar(0.20941503221602112) * _tmp51), Scalar(2)) +
                     Scalar(0.32397683292140877) *
                         std::pow(Scalar(1 - Scalar(0.36791786395571047) * _tmp54), Scalar(2))));
  const Scalar _tmp180 = Scalar(0.1034955) * _tmp173;
  const Scalar _tmp181 = _tmp179 * _tmp180;
  const Scalar _tmp182 = Scalar(9.6622558468725703) * _tmp126;
  const Scalar _tmp183 = _tmp182 * _tmp79;
  const Scalar _tmp184 = std::pow(_tmp172, Scalar(-2));
  const Scalar _tmp185 = _tmp127 * _tmp184;
  const Scalar _tmp186 = _tmp140 * _tmp41;
  const Scalar _tmp187 =
      (_tmp173 * (-_tmp149 * _tmp186 - _tmp153 + _tmp157 * _tmp169) - _tmp175 * _tmp185) /
      std::sqrt(Scalar(std::pow(_tmp175, Scalar(2)) * _tmp184 + 1));
  const Scalar _tmp188 =
      _tmp114 * _tmp118 * _tmp123 + _tmp114 * _tmp132 * _tmp134 + _tmp171 + _tmp84 * _tmp90;
  const Scalar _tmp189 = Scalar(1.0) / (_tmp188);
  const Scalar _tmp190 = -_tmp117 * _tmp123 * _tmp139 + _tmp124 * _tmp152 -
                         _tmp131 * _tmp134 * _tmp139 - _tmp141 * _tmp90 - _tmp155 * _tmp156 +
                         _tmp174;
  const Scalar _tmp191 = std::asinh(_tmp189 * _tmp190);
  const Scalar _tmp192 = Scalar(1.0) * _tmp191;
  const Scalar _tmp193 = Scalar(9.6622558468725703) * _tmp188;
  const Scalar _tmp194 =
      -_tmp191 * _tmp193 -
      Scalar(8.3196563700000006) *
          std::sqrt(
              Scalar(std::pow(Scalar(-Scalar(0.12019727204189803) * _tmp59 - 1), Scalar(2)) +
                     Scalar(0.057067943376852184) *
                         std::pow(Scalar(-Scalar(0.50315118556004401) * _tmp62 - 1), Scalar(2))));
  const Scalar _tmp195 = Scalar(0.1034955) * _tmp189;
  const Scalar _tmp196 = _tmp194 * _tmp195;
  const Scalar _tmp197 = std::pow(_tmp188, Scalar(-2));
  const Scalar _tmp198 = _tmp126 * _tmp197;
  const Scalar _tmp199 = (_tmp189 * (-_tmp152 - _tmp156 * _tmp169 + _tmp186) + _tmp190 * _tmp198) /
                         std::sqrt(Scalar(std::pow(_tmp190, Scalar(2)) * _tmp197 + 1));

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) =
      -_tmp3 *
      (_tmp2 * std::sinh(Scalar(1.0) * _tmp1) +
       _tmp2 * std::sinh(
                   Scalar(0.1034955) * _tmp0 *
                   (-_tmp1 * _tmp3 -
                    Scalar(4.8333311099999996) *
                        std::sqrt(Scalar(
                            std::pow(Scalar(1 - Scalar(0.20689664689659551) * _tmp31), Scalar(2)) +
                            Scalar(0.13817235445745474) *
                                std::pow(Scalar(-Scalar(0.55659957866191134) * _tmp42 - 1),
                                         Scalar(2)))))));
  _res(1, 0) =
      -_tmp161 *
          (-Scalar(0.87653584775870996) * _tmp168 + Scalar(1.0) * _tmp170 * std::sinh(_tmp165) -
           (-Scalar(0.1034955) * _tmp162 * _tmp168 +
            _tmp163 * (-_tmp160 * _tmp166 - _tmp161 * _tmp170)) *
               std::sinh(_tmp164)) -
      _tmp166 * (Scalar(0.87653584775870996) * _tmp136 - std::cosh(_tmp164) + std::cosh(_tmp165));
  _res(2, 0) =
      -_tmp178 *
          (-Scalar(0.86565325453551001) * _tmp185 + Scalar(1.0) * _tmp187 * std::sinh(_tmp177) -
           (-Scalar(0.1034955) * _tmp179 * _tmp185 +
            _tmp180 * (-_tmp176 * _tmp183 - _tmp178 * _tmp187)) *
               std::sinh(_tmp181)) -
      _tmp183 * (Scalar(0.86565325453551001) * _tmp173 + std::cosh(_tmp177) - std::cosh(_tmp181));
  _res(3, 0) =
      _tmp182 * (Scalar(0.87679799772039002) * _tmp189 + std::cosh(_tmp192) - std::cosh(_tmp196)) -
      _tmp193 *
          (Scalar(0.87679799772039002) * _tmp198 + Scalar(1.0) * _tmp199 * std::sinh(_tmp192) -
           (Scalar(0.1034955) * _tmp194 * _tmp198 +
            _tmp195 * (_tmp182 * _tmp191 - _tmp193 * _tmp199)) *
               std::sinh(_tmp196));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym