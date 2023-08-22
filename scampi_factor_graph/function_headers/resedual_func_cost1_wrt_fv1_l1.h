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
 * Symbolic function: resedual_func_cost1_wrt_fv1_l1
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
Eigen::Matrix<Scalar, 4, 1> ResedualFuncCost1WrtFv1L1(
    const Scalar fh1, const Scalar fv1, const Scalar rx, const Scalar ry, const Scalar rz,
    const Scalar p_init0, const Scalar p_init1, const Scalar p_init2, const Scalar rot_init_x,
    const Scalar rot_init_y, const Scalar rot_init_z, const Scalar rot_init_w,
    const Scalar epsilon) {
  // Total ops: 622

  // Unused inputs
  (void)p_init2;
  (void)epsilon;

  // Input arrays

  // Intermediate terms (197)
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
  const Scalar _tmp30 = -2 * std::pow(_tmp10, Scalar(2));
  const Scalar _tmp31 = Scalar(0.20999999999999999) * _tmp19 +
                        Scalar(0.20999999999999999) * _tmp30 + Scalar(0.20999999999999999);
  const Scalar _tmp32 = -_tmp31;
  const Scalar _tmp33 = Scalar(0.20999999999999999) * _tmp13 - Scalar(0.20999999999999999) * _tmp16;
  const Scalar _tmp34 = _tmp12 * _tmp14;
  const Scalar _tmp35 = _tmp10 * _tmp15;
  const Scalar _tmp36 = _tmp34 + _tmp35;
  const Scalar _tmp37 = -Scalar(0.010999999999999999) * _tmp36;
  const Scalar _tmp38 = -_tmp33 + _tmp37;
  const Scalar _tmp39 = _tmp32 + _tmp38;
  const Scalar _tmp40 = _tmp39 + p_init0;
  const Scalar _tmp41 = Scalar(9.6622558468725703) * fh1;
  const Scalar _tmp42 = _tmp31 + _tmp38;
  const Scalar _tmp43 = _tmp42 + p_init0;
  const Scalar _tmp44 = _tmp43 + Scalar(-2.5202214700000001);
  const Scalar _tmp45 = _tmp17 + _tmp27;
  const Scalar _tmp46 = _tmp45 + p_init1;
  const Scalar _tmp47 = _tmp46 + Scalar(8.3888750099999996);
  const Scalar _tmp48 = std::pow(Scalar(std::pow(_tmp44, Scalar(2)) + std::pow(_tmp47, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp49 = _tmp47 * _tmp48;
  const Scalar _tmp50 = _tmp18 + _tmp21 + _tmp26;
  const Scalar _tmp51 = _tmp50 + p_init1;
  const Scalar _tmp52 = _tmp51 + Scalar(-4.8333311099999996);
  const Scalar _tmp53 = _tmp33 + _tmp37;
  const Scalar _tmp54 = _tmp32 + _tmp53;
  const Scalar _tmp55 = _tmp54 + p_init0;
  const Scalar _tmp56 = _tmp55 + Scalar(1.79662371);
  const Scalar _tmp57 = Scalar(1.0) / (_tmp56);
  const Scalar _tmp58 = _tmp52 * _tmp57;
  const Scalar _tmp59 = _tmp44 * _tmp48;
  const Scalar _tmp60 = -_tmp49 + _tmp58 * _tmp59;
  const Scalar _tmp61 = _tmp17 + _tmp21;
  const Scalar _tmp62 = _tmp26 + _tmp61;
  const Scalar _tmp63 = _tmp62 + p_init1;
  const Scalar _tmp64 = _tmp63 + Scalar(-4.7752063900000001);
  const Scalar _tmp65 = _tmp31 + _tmp53;
  const Scalar _tmp66 = _tmp65 + p_init0;
  const Scalar _tmp67 = _tmp66 + Scalar(-2.71799795);
  const Scalar _tmp68 = std::pow(Scalar(std::pow(_tmp64, Scalar(2)) + std::pow(_tmp67, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp69 = _tmp67 * _tmp68;
  const Scalar _tmp70 = _tmp64 * _tmp68;
  const Scalar _tmp71 = Scalar(1.0) / (_tmp58 * _tmp69 - _tmp70);
  const Scalar _tmp72 =
      std::sqrt(Scalar(std::pow(_tmp52, Scalar(2)) + std::pow(_tmp56, Scalar(2))));
  const Scalar _tmp73 = Scalar(1.0) / (_tmp72);
  const Scalar _tmp74 = _tmp57 * _tmp72;
  const Scalar _tmp75 = _tmp74 * (-_tmp50 * _tmp56 * _tmp73 + _tmp52 * _tmp54 * _tmp73);
  const Scalar _tmp76 = _tmp71 * (_tmp62 * _tmp69 - _tmp65 * _tmp70 + _tmp69 * _tmp75);
  const Scalar _tmp77 = -_tmp42 * _tmp49 + _tmp45 * _tmp59 + _tmp59 * _tmp75 - _tmp60 * _tmp76;
  const Scalar _tmp78 = Scalar(1.0) / (_tmp77);
  const Scalar _tmp79 = Scalar(1.0) * _tmp78;
  const Scalar _tmp80 = _tmp69 * _tmp71;
  const Scalar _tmp81 = _tmp29 + Scalar(8.3196563700000006);
  const Scalar _tmp82 = _tmp40 + Scalar(1.9874742000000001);
  const Scalar _tmp83 = std::pow(Scalar(std::pow(_tmp81, Scalar(2)) + std::pow(_tmp82, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp84 = _tmp82 * _tmp83;
  const Scalar _tmp85 = _tmp81 * _tmp83;
  const Scalar _tmp86 = fh1 * (-_tmp28 * _tmp84 + _tmp39 * _tmp85);
  const Scalar _tmp87 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp88 = Scalar(0.20999999999999999) * _tmp34 - Scalar(0.20999999999999999) * _tmp35;
  const Scalar _tmp89 =
      -Scalar(0.010999999999999999) * _tmp20 - Scalar(0.010999999999999999) * _tmp30;
  const Scalar _tmp90 = Scalar(0.20999999999999999) * _tmp22 + Scalar(0.20999999999999999) * _tmp23;
  const Scalar _tmp91 = _tmp89 - _tmp90;
  const Scalar _tmp92 = _tmp88 + _tmp91;
  const Scalar _tmp93 = -_tmp88;
  const Scalar _tmp94 = _tmp89 + _tmp90;
  const Scalar _tmp95 = _tmp93 + _tmp94;
  const Scalar _tmp96 = _tmp69 * _tmp95;
  const Scalar _tmp97 = _tmp88 + _tmp94;
  const Scalar _tmp98 = _tmp71 * (-_tmp58 * _tmp96 + _tmp70 * _tmp97);
  const Scalar _tmp99 = _tmp58 * _tmp95;
  const Scalar _tmp100 = _tmp49 * _tmp92 - _tmp59 * _tmp99 - _tmp60 * _tmp98;
  const Scalar _tmp101 = Scalar(1.0) * _tmp50;
  const Scalar _tmp102 = -_tmp101;
  const Scalar _tmp103 = Scalar(1.0) / (_tmp102 + _tmp62);
  const Scalar _tmp104 = Scalar(1.0) * _tmp54;
  const Scalar _tmp105 = _tmp103 * (_tmp104 - _tmp65);
  const Scalar _tmp106 = _tmp71 * (-_tmp69 * _tmp97 + _tmp96);
  const Scalar _tmp107 = -_tmp100 * _tmp105 - _tmp106 * _tmp60 - _tmp59 * _tmp92 + _tmp59 * _tmp95;
  const Scalar _tmp108 = Scalar(1.0) / (_tmp107);
  const Scalar _tmp109 = _tmp101 * _tmp105 + _tmp104;
  const Scalar _tmp110 = 0;
  const Scalar _tmp111 = _tmp108 * _tmp110;
  const Scalar _tmp112 = _tmp108 * _tmp60;
  const Scalar _tmp113 = _tmp110 * _tmp112;
  const Scalar _tmp114 = _tmp74 * (_tmp111 * _tmp59 - _tmp113 * _tmp80);
  const Scalar _tmp115 = Scalar(1.0) * _tmp98;
  const Scalar _tmp116 = _tmp105 * _tmp115 - Scalar(1.0) * _tmp106;
  const Scalar _tmp117 = _tmp108 * _tmp77;
  const Scalar _tmp118 = _tmp107 * _tmp78;
  const Scalar _tmp119 = _tmp118 * (-_tmp116 * _tmp117 - Scalar(1.0) * _tmp76);
  const Scalar _tmp120 = _tmp116 + _tmp119;
  const Scalar _tmp121 = -_tmp112 * _tmp120 + Scalar(1.0);
  const Scalar _tmp122 = _tmp108 * _tmp59;
  const Scalar _tmp123 = _tmp85 * fh1;
  const Scalar _tmp124 = _tmp58 * _tmp98 + _tmp99;
  const Scalar _tmp125 = -_tmp105 * _tmp124 + _tmp106 * _tmp58 - _tmp95;
  const Scalar _tmp126 = _tmp118 * (-_tmp117 * _tmp125 + _tmp58 * _tmp76 - _tmp75);
  const Scalar _tmp127 = _tmp125 + _tmp126;
  const Scalar _tmp128 = -_tmp112 * _tmp127 - _tmp58;
  const Scalar _tmp129 = _tmp84 * fh1;
  const Scalar _tmp130 = -_tmp114 * _tmp87 -
                         _tmp123 * _tmp74 * (_tmp120 * _tmp122 + _tmp121 * _tmp80) -
                         _tmp129 * _tmp74 * (_tmp122 * _tmp127 + _tmp128 * _tmp80 + Scalar(1.0)) -
                         _tmp74 * _tmp86 * (_tmp59 * _tmp79 - _tmp60 * _tmp79 * _tmp80);
  const Scalar _tmp131 = Scalar(1.0) / (_tmp130);
  const Scalar _tmp132 = _tmp102 + _tmp45;
  const Scalar _tmp133 = _tmp105 * _tmp132;
  const Scalar _tmp134 = Scalar(1.0) / (_tmp104 - _tmp133 - _tmp42);
  const Scalar _tmp135 = Scalar(1.0) * _tmp134;
  const Scalar _tmp136 = _tmp100 * _tmp108;
  const Scalar _tmp137 = _tmp132 * _tmp134;
  const Scalar _tmp138 = -_tmp115 + _tmp119 * _tmp137 - _tmp120 * _tmp136;
  const Scalar _tmp139 = Scalar(1.0) * _tmp103;
  const Scalar _tmp140 = _tmp118 * _tmp135;
  const Scalar _tmp141 = -_tmp100 * _tmp79 + _tmp132 * _tmp140;
  const Scalar _tmp142 = _tmp124 + _tmp126 * _tmp137 - _tmp127 * _tmp136;
  const Scalar _tmp143 = fh1 * (_tmp91 + _tmp93);
  const Scalar _tmp144 = _tmp143 * _tmp84 + Scalar(3.29616) * _tmp36 + _tmp39 * fv1;
  const Scalar _tmp145 = _tmp103 * _tmp132;
  const Scalar _tmp146 = Scalar(1.0) * _tmp135 * _tmp145 - Scalar(1.0) * _tmp135;
  const Scalar _tmp147 = _tmp109 * _tmp134;
  const Scalar _tmp148 = _tmp103 * (_tmp102 - _tmp110 * _tmp136 - _tmp132 * _tmp147);
  const Scalar _tmp149 = -Scalar(1.0) * _tmp109 * _tmp135 - Scalar(1.0) * _tmp148 + Scalar(1.0);
  const Scalar _tmp150 = -_tmp143 * _tmp85 - Scalar(3.29616) * _tmp24 - _tmp28 * fv1;
  const Scalar _tmp151 = _tmp133 * _tmp135 + Scalar(1.0);
  const Scalar _tmp152 = _tmp105 * _tmp135;
  const Scalar _tmp153 = -Scalar(1.0) * _tmp139 * _tmp151 + Scalar(1.0) * _tmp152;
  const Scalar _tmp154 = Scalar(1.0) * _tmp123 * (_tmp119 * _tmp135 - _tmp138 * _tmp139) +
                         Scalar(1.0) * _tmp129 * (_tmp126 * _tmp135 - _tmp139 * _tmp142) +
                         _tmp144 * _tmp146 + _tmp149 * _tmp87 + _tmp150 * _tmp153 +
                         Scalar(1.0) * _tmp86 * (-_tmp139 * _tmp141 + _tmp140);
  const Scalar _tmp155 = std::asinh(_tmp131 * _tmp154);
  const Scalar _tmp156 = Scalar(1.0) * _tmp155;
  const Scalar _tmp157 = _tmp25 + _tmp61;
  const Scalar _tmp158 = std::pow(_tmp130, Scalar(-2));
  const Scalar _tmp159 = _tmp114 * _tmp158;
  const Scalar _tmp160 =
      (_tmp131 * (_tmp146 * _tmp39 - _tmp149 + _tmp153 * _tmp157) - _tmp154 * _tmp159) /
      std::sqrt(Scalar(std::pow(_tmp154, Scalar(2)) * _tmp158 + 1));
  const Scalar _tmp161 = Scalar(9.6622558468725703) * _tmp130;
  const Scalar _tmp162 = Scalar(9.6622558468725703) * _tmp114;
  const Scalar _tmp163 = Scalar(0.1034955) * _tmp131;
  const Scalar _tmp164 =
      -_tmp155 * _tmp161 -
      Scalar(4.8333311099999996) *
          std::sqrt(
              Scalar(std::pow(Scalar(1 - Scalar(0.20689664689659551) * _tmp51), Scalar(2)) +
                     Scalar(0.13817235445745474) *
                         std::pow(Scalar(-Scalar(0.55659957866191134) * _tmp55 - 1), Scalar(2))));
  const Scalar _tmp165 = _tmp163 * _tmp164;
  const Scalar _tmp166 = _tmp113 * _tmp71;
  const Scalar _tmp167 = _tmp79 * _tmp86;
  const Scalar _tmp168 = _tmp121 * _tmp123 * _tmp71 + _tmp128 * _tmp129 * _tmp71 -
                         _tmp166 * _tmp87 - _tmp167 * _tmp60 * _tmp71;
  const Scalar _tmp169 = Scalar(1.0) / (_tmp168);
  const Scalar _tmp170 = _tmp135 * _tmp144;
  const Scalar _tmp171 = _tmp103 * _tmp151;
  const Scalar _tmp172 = _tmp103 * _tmp123 * _tmp138 + _tmp103 * _tmp129 * _tmp142 +
                         _tmp103 * _tmp141 * _tmp86 - _tmp145 * _tmp170 + _tmp148 * _tmp87 +
                         _tmp150 * _tmp171;
  const Scalar _tmp173 = std::asinh(_tmp169 * _tmp172);
  const Scalar _tmp174 = Scalar(9.6622558468725703) * _tmp168;
  const Scalar _tmp175 =
      -_tmp173 * _tmp174 -
      Scalar(4.7752063900000001) *
          std::sqrt(
              Scalar(std::pow(Scalar(1 - Scalar(0.20941503221602112) * _tmp63), Scalar(2)) +
                     Scalar(0.32397683292140877) *
                         std::pow(Scalar(1 - Scalar(0.36791786395571047) * _tmp66), Scalar(2))));
  const Scalar _tmp176 = Scalar(0.1034955) * _tmp169;
  const Scalar _tmp177 = _tmp175 * _tmp176;
  const Scalar _tmp178 = Scalar(1.0) * _tmp173;
  const Scalar _tmp179 = Scalar(9.6622558468725703) * _tmp166;
  const Scalar _tmp180 = std::pow(_tmp168, Scalar(-2));
  const Scalar _tmp181 = _tmp135 * _tmp39;
  const Scalar _tmp182 = _tmp166 * _tmp180;
  const Scalar _tmp183 =
      (_tmp169 * (-_tmp145 * _tmp181 - _tmp148 + _tmp157 * _tmp171) - _tmp172 * _tmp182) /
      std::sqrt(Scalar(std::pow(_tmp172, Scalar(2)) * _tmp180 + 1));
  const Scalar _tmp184 =
      _tmp108 * _tmp120 * _tmp123 + _tmp108 * _tmp127 * _tmp129 + _tmp111 * _tmp87 + _tmp167;
  const Scalar _tmp185 = Scalar(1.0) / (_tmp184);
  const Scalar _tmp186 = -_tmp119 * _tmp123 * _tmp134 - _tmp126 * _tmp129 * _tmp134 -
                         _tmp140 * _tmp86 + _tmp147 * _tmp87 - _tmp150 * _tmp152 + _tmp170;
  const Scalar _tmp187 = std::asinh(_tmp185 * _tmp186);
  const Scalar _tmp188 = Scalar(1.0) * _tmp187;
  const Scalar _tmp189 = Scalar(9.6622558468725703) * _tmp184;
  const Scalar _tmp190 =
      -_tmp187 * _tmp189 -
      Scalar(8.3888750099999996) *
          std::sqrt(
              Scalar(Scalar(0.090254729040973036) *
                         std::pow(Scalar(1 - Scalar(0.39679052492160538) * _tmp43), Scalar(2)) +
                     std::pow(Scalar(-Scalar(0.11920549523123722) * _tmp46 - 1), Scalar(2))));
  const Scalar _tmp191 = Scalar(0.1034955) * _tmp185;
  const Scalar _tmp192 = _tmp190 * _tmp191;
  const Scalar _tmp193 = Scalar(9.6622558468725703) * _tmp111;
  const Scalar _tmp194 = std::pow(_tmp184, Scalar(-2));
  const Scalar _tmp195 = _tmp111 * _tmp194;
  const Scalar _tmp196 = (_tmp185 * (-_tmp147 - _tmp152 * _tmp157 + _tmp181) + _tmp186 * _tmp195) /
                         std::sqrt(Scalar(std::pow(_tmp186, Scalar(2)) * _tmp194 + 1));

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) =
      -_tmp41 *
      (_tmp2 * std::sinh(Scalar(1.0) * _tmp1) +
       _tmp2 * std::sinh(
                   Scalar(0.1034955) * _tmp0 *
                   (-_tmp1 * _tmp41 -
                    Scalar(8.3196563700000006) *
                        std::sqrt(Scalar(
                            std::pow(Scalar(-Scalar(0.12019727204189803) * _tmp29 - 1), Scalar(2)) +
                            Scalar(0.057067943376852184) *
                                std::pow(Scalar(-Scalar(0.50315118556004401) * _tmp40 - 1),
                                         Scalar(2)))))));
  _res(1, 0) =
      -_tmp161 *
          (-Scalar(0.86625939559540499) * _tmp159 + Scalar(1.0) * _tmp160 * std::sinh(_tmp156) -
           (-Scalar(0.1034955) * _tmp159 * _tmp164 +
            _tmp163 * (-_tmp155 * _tmp162 - _tmp160 * _tmp161)) *
               std::sinh(_tmp165)) -
      _tmp162 * (Scalar(0.86625939559540499) * _tmp131 + std::cosh(_tmp156) - std::cosh(_tmp165));
  _res(2, 0) =
      -_tmp174 *
          (-Scalar(0.86565325453551001) * _tmp182 + Scalar(1.0) * _tmp183 * std::sinh(_tmp178) -
           (-Scalar(0.1034955) * _tmp175 * _tmp182 +
            _tmp176 * (-_tmp173 * _tmp179 - _tmp174 * _tmp183)) *
               std::sinh(_tmp177)) -
      _tmp179 * (Scalar(0.86565325453551001) * _tmp169 - std::cosh(_tmp177) + std::cosh(_tmp178));
  _res(3, 0) =
      -_tmp189 *
          (Scalar(0.87653584775870996) * _tmp195 + Scalar(1.0) * _tmp196 * std::sinh(_tmp188) -
           (Scalar(0.1034955) * _tmp190 * _tmp195 +
            _tmp191 * (_tmp187 * _tmp193 - _tmp189 * _tmp196)) *
               std::sinh(_tmp192)) +
      _tmp193 * (Scalar(0.87653584775870996) * _tmp185 + std::cosh(_tmp188) - std::cosh(_tmp192));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
