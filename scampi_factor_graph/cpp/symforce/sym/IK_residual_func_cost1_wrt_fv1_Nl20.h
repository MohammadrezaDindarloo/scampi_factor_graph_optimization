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
 * Symbolic function: IK_residual_func_cost1_wrt_fv1_Nl20
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost1WrtFv1Nl20(
    const Scalar fh1, const Scalar fv1, const Scalar rx, const Scalar ry, const Scalar rz,
    const Scalar p_init0, const Scalar p_init1, const Scalar p_init2, const Scalar rot_init_x,
    const Scalar rot_init_y, const Scalar rot_init_z, const Scalar rot_init_w,
    const Scalar epsilon) {
  // Total ops: 623

  // Unused inputs
  (void)p_init2;
  (void)epsilon;

  // Input arrays

  // Intermediate terms (198)
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
  const Scalar _tmp17 = 2 * _tmp11 * _tmp16;
  const Scalar _tmp18 = -2 * _tmp10 * rot_init_z + 2 * _tmp6 * rot_init_w - 2 * _tmp8 * rot_init_y -
                        2 * _tmp9 * rot_init_x;
  const Scalar _tmp19 = _tmp13 * _tmp18;
  const Scalar _tmp20 = _tmp17 - _tmp19;
  const Scalar _tmp21 = Scalar(0.010999999999999999) * _tmp20;
  const Scalar _tmp22 = -_tmp21;
  const Scalar _tmp23 = 2 * _tmp13;
  const Scalar _tmp24 = _tmp16 * _tmp23;
  const Scalar _tmp25 = _tmp11 * _tmp18;
  const Scalar _tmp26 = Scalar(0.20999999999999999) * _tmp24 + Scalar(0.20999999999999999) * _tmp25;
  const Scalar _tmp27 = _tmp22 - _tmp26;
  const Scalar _tmp28 = _tmp15 + _tmp27;
  const Scalar _tmp29 = _tmp28 + p_init1;
  const Scalar _tmp30 = -2 * std::pow(_tmp16, Scalar(2));
  const Scalar _tmp31 = Scalar(0.20999999999999999) * _tmp12 +
                        Scalar(0.20999999999999999) * _tmp30 + Scalar(0.20999999999999999);
  const Scalar _tmp32 = -_tmp31;
  const Scalar _tmp33 = _tmp11 * _tmp23;
  const Scalar _tmp34 = _tmp16 * _tmp18;
  const Scalar _tmp35 = _tmp33 + _tmp34;
  const Scalar _tmp36 = -Scalar(0.010999999999999999) * _tmp35;
  const Scalar _tmp37 = Scalar(0.20999999999999999) * _tmp24 - Scalar(0.20999999999999999) * _tmp25;
  const Scalar _tmp38 = _tmp36 + _tmp37;
  const Scalar _tmp39 = _tmp32 + _tmp38;
  const Scalar _tmp40 = _tmp39 + p_init0;
  const Scalar _tmp41 = _tmp29 + Scalar(-4.8333311099999996);
  const Scalar _tmp42 = _tmp40 + Scalar(1.79662371);
  const Scalar _tmp43 = std::pow(Scalar(std::pow(_tmp41, Scalar(2)) + std::pow(_tmp42, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp44 = _tmp42 * _tmp43;
  const Scalar _tmp45 = Scalar(0.20999999999999999) * _tmp33 - Scalar(0.20999999999999999) * _tmp34;
  const Scalar _tmp46 = -_tmp45;
  const Scalar _tmp47 =
      -Scalar(0.010999999999999999) * _tmp14 - Scalar(0.010999999999999999) * _tmp30;
  const Scalar _tmp48 = Scalar(0.20999999999999999) * _tmp17 + Scalar(0.20999999999999999) * _tmp19;
  const Scalar _tmp49 = _tmp47 + _tmp48;
  const Scalar _tmp50 = fh1 * (_tmp46 + _tmp49);
  const Scalar _tmp51 = Scalar(3.29616) * _tmp35 + _tmp39 * fv1 + _tmp44 * _tmp50;
  const Scalar _tmp52 = -_tmp15;
  const Scalar _tmp53 = _tmp26 + _tmp52;
  const Scalar _tmp54 = _tmp22 + _tmp53;
  const Scalar _tmp55 = Scalar(1.0) * _tmp54;
  const Scalar _tmp56 = -_tmp55;
  const Scalar _tmp57 = _tmp15 + _tmp22 + _tmp26;
  const Scalar _tmp58 = _tmp56 + _tmp57;
  const Scalar _tmp59 = _tmp27 + _tmp52;
  const Scalar _tmp60 = Scalar(1.0) / (_tmp56 + _tmp59);
  const Scalar _tmp61 = _tmp36 - _tmp37;
  const Scalar _tmp62 = _tmp31 + _tmp61;
  const Scalar _tmp63 = Scalar(1.0) * _tmp62;
  const Scalar _tmp64 = _tmp32 + _tmp61;
  const Scalar _tmp65 = _tmp63 - _tmp64;
  const Scalar _tmp66 = _tmp60 * _tmp65;
  const Scalar _tmp67 = _tmp58 * _tmp66;
  const Scalar _tmp68 = _tmp31 + _tmp38;
  const Scalar _tmp69 = Scalar(1.0) / (_tmp63 - _tmp67 - _tmp68);
  const Scalar _tmp70 = Scalar(1.0) * _tmp69;
  const Scalar _tmp71 = _tmp58 * _tmp60;
  const Scalar _tmp72 = Scalar(1.0) * _tmp70 * _tmp71 - Scalar(1.0) * _tmp70;
  const Scalar _tmp73 = _tmp62 + p_init0;
  const Scalar _tmp74 = _tmp73 + Scalar(-2.5202214700000001);
  const Scalar _tmp75 = _tmp54 + p_init1;
  const Scalar _tmp76 = _tmp75 + Scalar(8.3888750099999996);
  const Scalar _tmp77 =
      std::sqrt(Scalar(std::pow(_tmp74, Scalar(2)) + std::pow(_tmp76, Scalar(2))));
  const Scalar _tmp78 = Scalar(1.0) / (_tmp77);
  const Scalar _tmp79 = Scalar(1.0) / (_tmp74);
  const Scalar _tmp80 = _tmp77 * _tmp79;
  const Scalar _tmp81 = _tmp80 * (-_tmp54 * _tmp74 * _tmp78 + _tmp62 * _tmp76 * _tmp78);
  const Scalar _tmp82 = _tmp47 - _tmp48;
  const Scalar _tmp83 = _tmp45 + _tmp82;
  const Scalar _tmp84 = _tmp76 * _tmp79;
  const Scalar _tmp85 = _tmp59 + p_init1;
  const Scalar _tmp86 = _tmp85 + Scalar(8.3196563700000006);
  const Scalar _tmp87 = _tmp64 + p_init0;
  const Scalar _tmp88 = _tmp87 + Scalar(1.9874742000000001);
  const Scalar _tmp89 = std::pow(Scalar(std::pow(_tmp86, Scalar(2)) + std::pow(_tmp88, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp90 = _tmp88 * _tmp89;
  const Scalar _tmp91 = _tmp86 * _tmp89;
  const Scalar _tmp92 = Scalar(1.0) / (_tmp84 * _tmp90 - _tmp91);
  const Scalar _tmp93 = _tmp46 + _tmp82;
  const Scalar _tmp94 = _tmp83 * _tmp90;
  const Scalar _tmp95 = _tmp92 * (-_tmp90 * _tmp93 + _tmp94);
  const Scalar _tmp96 = _tmp92 * (-_tmp84 * _tmp94 + _tmp91 * _tmp93);
  const Scalar _tmp97 = _tmp83 * _tmp84;
  const Scalar _tmp98 = _tmp84 * _tmp96 + _tmp97;
  const Scalar _tmp99 = -_tmp66 * _tmp98 - _tmp83 + _tmp84 * _tmp95;
  const Scalar _tmp100 = _tmp57 + p_init1;
  const Scalar _tmp101 = _tmp100 + Scalar(-4.7752063900000001);
  const Scalar _tmp102 = _tmp68 + p_init0;
  const Scalar _tmp103 = _tmp102 + Scalar(-2.71799795);
  const Scalar _tmp104 =
      std::pow(Scalar(std::pow(_tmp101, Scalar(2)) + std::pow(_tmp103, Scalar(2))),
               Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp105 = _tmp103 * _tmp104;
  const Scalar _tmp106 = _tmp101 * _tmp104;
  const Scalar _tmp107 = _tmp105 * _tmp84 - _tmp106;
  const Scalar _tmp108 = _tmp45 + _tmp49;
  const Scalar _tmp109 = -_tmp105 * _tmp97 + _tmp106 * _tmp108 - _tmp107 * _tmp96;
  const Scalar _tmp110 =
      -_tmp105 * _tmp108 + _tmp105 * _tmp83 - _tmp107 * _tmp95 - _tmp109 * _tmp66;
  const Scalar _tmp111 = Scalar(1.0) / (_tmp110);
  const Scalar _tmp112 = _tmp92 * (_tmp59 * _tmp90 - _tmp64 * _tmp91 + _tmp81 * _tmp90);
  const Scalar _tmp113 = _tmp105 * _tmp57 + _tmp105 * _tmp81 - _tmp106 * _tmp68 - _tmp107 * _tmp112;
  const Scalar _tmp114 = _tmp111 * _tmp113;
  const Scalar _tmp115 = Scalar(1.0) / (_tmp113);
  const Scalar _tmp116 = _tmp110 * _tmp115;
  const Scalar _tmp117 = _tmp116 * (_tmp112 * _tmp84 - _tmp114 * _tmp99 - _tmp81);
  const Scalar _tmp118 = _tmp58 * _tmp69;
  const Scalar _tmp119 = _tmp117 + _tmp99;
  const Scalar _tmp120 = _tmp109 * _tmp111;
  const Scalar _tmp121 = _tmp117 * _tmp118 - _tmp119 * _tmp120 + _tmp98;
  const Scalar _tmp122 = Scalar(1.0) * _tmp60;
  const Scalar _tmp123 = _tmp44 * fh1;
  const Scalar _tmp124 = _tmp116 * _tmp70;
  const Scalar _tmp125 = Scalar(1.0) * _tmp115;
  const Scalar _tmp126 = -_tmp109 * _tmp125 + _tmp124 * _tmp58;
  const Scalar _tmp127 = _tmp41 * _tmp43;
  const Scalar _tmp128 = fh1 * (_tmp127 * _tmp39 - _tmp28 * _tmp44);
  const Scalar _tmp129 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp130 = _tmp55 * _tmp66 + _tmp63;
  const Scalar _tmp131 = _tmp130 * _tmp69;
  const Scalar _tmp132 = 0;
  const Scalar _tmp133 = _tmp60 * (-_tmp120 * _tmp132 - _tmp131 * _tmp58 + _tmp56);
  const Scalar _tmp134 = -Scalar(1.0) * _tmp131 - Scalar(1.0) * _tmp133 + Scalar(1.0);
  const Scalar _tmp135 = -_tmp127 * _tmp50 - Scalar(3.29616) * _tmp20 - _tmp28 * fv1;
  const Scalar _tmp136 = _tmp60 * (_tmp67 * _tmp70 + Scalar(1.0));
  const Scalar _tmp137 = _tmp66 * _tmp70;
  const Scalar _tmp138 = -Scalar(1.0) * _tmp136 + Scalar(1.0) * _tmp137;
  const Scalar _tmp139 = _tmp122 * _tmp65 * _tmp96 - Scalar(1.0) * _tmp95;
  const Scalar _tmp140 = _tmp116 * (-Scalar(1.0) * _tmp112 - _tmp114 * _tmp139);
  const Scalar _tmp141 = _tmp139 + _tmp140;
  const Scalar _tmp142 = _tmp118 * _tmp140 - _tmp120 * _tmp141 - Scalar(1.0) * _tmp96;
  const Scalar _tmp143 = _tmp127 * fh1;
  const Scalar _tmp144 = Scalar(1.0) * _tmp123 * (_tmp117 * _tmp70 - _tmp121 * _tmp122) +
                         Scalar(1.0) * _tmp128 * (-_tmp122 * _tmp126 + _tmp124) +
                         _tmp129 * _tmp134 + _tmp135 * _tmp138 +
                         Scalar(1.0) * _tmp143 * (-_tmp122 * _tmp142 + _tmp140 * _tmp70) +
                         _tmp51 * _tmp72;
  const Scalar _tmp145 = _tmp90 * _tmp92;
  const Scalar _tmp146 = _tmp107 * _tmp111;
  const Scalar _tmp147 = _tmp132 * _tmp146;
  const Scalar _tmp148 = _tmp111 * _tmp132;
  const Scalar _tmp149 = _tmp80 * (_tmp105 * _tmp148 - _tmp145 * _tmp147);
  const Scalar _tmp150 = -_tmp119 * _tmp146 - _tmp84;
  const Scalar _tmp151 = _tmp105 * _tmp111;
  const Scalar _tmp152 = -_tmp141 * _tmp146 + Scalar(1.0);
  const Scalar _tmp153 = -_tmp123 * _tmp80 * (_tmp119 * _tmp151 + _tmp145 * _tmp150 + Scalar(1.0)) -
                         _tmp128 * _tmp80 * (_tmp105 * _tmp125 - _tmp107 * _tmp125 * _tmp145) -
                         _tmp129 * _tmp149 -
                         _tmp143 * _tmp80 * (_tmp141 * _tmp151 + _tmp145 * _tmp152);
  const Scalar _tmp154 = Scalar(1.0) / (_tmp153);
  const Scalar _tmp155 = std::asinh(_tmp144 * _tmp154);
  const Scalar _tmp156 = Scalar(1.0) * _tmp155;
  const Scalar _tmp157 = Scalar(9.6622558468725703) * _tmp153;
  const Scalar _tmp158 =
      -_tmp155 * _tmp157 -
      Scalar(8.3888750099999996) *
          std::sqrt(
              Scalar(Scalar(0.090254729040973036) *
                         std::pow(Scalar(1 - Scalar(0.39679052492160538) * _tmp73), Scalar(2)) +
                     std::pow(Scalar(-Scalar(0.11920549523123722) * _tmp75 - 1), Scalar(2))));
  const Scalar _tmp159 = Scalar(0.1034955) * _tmp154;
  const Scalar _tmp160 = _tmp158 * _tmp159;
  const Scalar _tmp161 = Scalar(9.6622558468725703) * _tmp149;
  const Scalar _tmp162 = std::pow(_tmp153, Scalar(-2));
  const Scalar _tmp163 = _tmp149 * _tmp162;
  const Scalar _tmp164 = _tmp21 + _tmp53;
  const Scalar _tmp165 =
      (-_tmp144 * _tmp163 + _tmp154 * (-_tmp134 + _tmp138 * _tmp164 + _tmp39 * _tmp72)) /
      std::sqrt(Scalar(std::pow(_tmp144, Scalar(2)) * _tmp162 + 1));
  const Scalar _tmp166 = _tmp60 * fh1;
  const Scalar _tmp167 = _tmp51 * _tmp70;
  const Scalar _tmp168 = _tmp121 * _tmp166 * _tmp44 + _tmp126 * _tmp128 * _tmp60 +
                         _tmp127 * _tmp142 * _tmp166 + _tmp129 * _tmp133 + _tmp135 * _tmp136 -
                         _tmp167 * _tmp71;
  const Scalar _tmp169 = _tmp147 * _tmp92;
  const Scalar _tmp170 = _tmp125 * _tmp128;
  const Scalar _tmp171 = _tmp92 * fh1;
  const Scalar _tmp172 = -_tmp107 * _tmp170 * _tmp92 + _tmp127 * _tmp152 * _tmp171 -
                         _tmp129 * _tmp169 + _tmp150 * _tmp171 * _tmp44;
  const Scalar _tmp173 = Scalar(1.0) / (_tmp172);
  const Scalar _tmp174 = std::asinh(_tmp168 * _tmp173);
  const Scalar _tmp175 = Scalar(1.0) * _tmp174;
  const Scalar _tmp176 = std::pow(_tmp172, Scalar(-2));
  const Scalar _tmp177 = _tmp39 * _tmp70;
  const Scalar _tmp178 = _tmp169 * _tmp176;
  const Scalar _tmp179 =
      (-_tmp168 * _tmp178 + _tmp173 * (-_tmp133 + _tmp136 * _tmp164 - _tmp177 * _tmp71)) /
      std::sqrt(Scalar(std::pow(_tmp168, Scalar(2)) * _tmp176 + 1));
  const Scalar _tmp180 = Scalar(9.6622558468725703) * _tmp172;
  const Scalar _tmp181 =
      -_tmp174 * _tmp180 -
      Scalar(8.3196563700000006) *
          std::sqrt(
              Scalar(std::pow(Scalar(-Scalar(0.12019727204189803) * _tmp85 - 1), Scalar(2)) +
                     Scalar(0.057067943376852184) *
                         std::pow(Scalar(-Scalar(0.50315118556004401) * _tmp87 - 1), Scalar(2))));
  const Scalar _tmp182 = Scalar(0.1034955) * _tmp173;
  const Scalar _tmp183 = _tmp181 * _tmp182;
  const Scalar _tmp184 = Scalar(9.6622558468725703) * _tmp169;
  const Scalar _tmp185 =
      _tmp111 * _tmp119 * _tmp123 + _tmp111 * _tmp141 * _tmp143 + _tmp129 * _tmp148 + _tmp170;
  const Scalar _tmp186 = Scalar(1.0) / (_tmp185);
  const Scalar _tmp187 = -_tmp117 * _tmp123 * _tmp69 - _tmp124 * _tmp128 + _tmp129 * _tmp131 -
                         _tmp135 * _tmp137 - _tmp140 * _tmp143 * _tmp69 + _tmp167;
  const Scalar _tmp188 = std::asinh(_tmp186 * _tmp187);
  const Scalar _tmp189 = Scalar(9.6622558468725703) * _tmp185;
  const Scalar _tmp190 =
      -_tmp188 * _tmp189 -
      Scalar(4.7752063900000001) *
          std::sqrt(
              Scalar(std::pow(Scalar(1 - Scalar(0.20941503221602112) * _tmp100), Scalar(2)) +
                     Scalar(0.32397683292140877) *
                         std::pow(Scalar(1 - Scalar(0.36791786395571047) * _tmp102), Scalar(2))));
  const Scalar _tmp191 = Scalar(0.1034955) * _tmp186;
  const Scalar _tmp192 = _tmp190 * _tmp191;
  const Scalar _tmp193 = Scalar(1.0) * _tmp188;
  const Scalar _tmp194 = Scalar(9.6622558468725703) * _tmp148;
  const Scalar _tmp195 = std::pow(_tmp185, Scalar(-2));
  const Scalar _tmp196 = _tmp148 * _tmp195;
  const Scalar _tmp197 = (_tmp186 * (-_tmp131 - _tmp137 * _tmp164 + _tmp177) + _tmp187 * _tmp196) /
                         std::sqrt(Scalar(std::pow(_tmp187, Scalar(2)) * _tmp195 + 1));

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
                            std::pow(Scalar(1 - Scalar(0.20689664689659551) * _tmp29), Scalar(2)) +
                            Scalar(0.13817235445745474) *
                                std::pow(Scalar(-Scalar(0.55659957866191134) * _tmp40 - 1),
                                         Scalar(2)))))));
  _res(1, 0) =
      -_tmp157 *
          (-Scalar(0.87653584775870996) * _tmp163 + Scalar(1.0) * _tmp165 * std::sinh(_tmp156) -
           (-Scalar(0.1034955) * _tmp158 * _tmp163 +
            _tmp159 * (-_tmp155 * _tmp161 - _tmp157 * _tmp165)) *
               std::sinh(_tmp160)) -
      _tmp161 * (Scalar(0.87653584775870996) * _tmp154 + std::cosh(_tmp156) - std::cosh(_tmp160));
  _res(2, 0) =
      -_tmp180 *
          (-Scalar(0.87679799772039002) * _tmp178 + Scalar(1.0) * _tmp179 * std::sinh(_tmp175) -
           (-Scalar(0.1034955) * _tmp178 * _tmp181 +
            _tmp182 * (-_tmp174 * _tmp184 - _tmp179 * _tmp180)) *
               std::sinh(_tmp183)) -
      _tmp184 * (Scalar(0.87679799772039002) * _tmp173 + std::cosh(_tmp175) - std::cosh(_tmp183));
  _res(3, 0) =
      -_tmp189 *
          (Scalar(0.86565325453551001) * _tmp196 + Scalar(1.0) * _tmp197 * std::sinh(_tmp193) -
           (Scalar(0.1034955) * _tmp190 * _tmp196 +
            _tmp191 * (_tmp188 * _tmp194 - _tmp189 * _tmp197)) *
               std::sinh(_tmp192)) +
      _tmp194 * (Scalar(0.86565325453551001) * _tmp186 - std::cosh(_tmp192) + std::cosh(_tmp193));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
