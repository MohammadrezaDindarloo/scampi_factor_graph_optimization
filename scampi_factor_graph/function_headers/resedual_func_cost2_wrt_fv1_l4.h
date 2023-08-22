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
 * Symbolic function: resedual_func_cost2_wrt_fv1_l4
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
Eigen::Matrix<Scalar, 4, 1> ResedualFuncCost2WrtFv1L4(
    const Scalar fh1, const Scalar fv1, const Scalar rx, const Scalar ry, const Scalar rz,
    const Scalar p_init0, const Scalar p_init1, const Scalar p_init2, const Scalar rot_init_x,
    const Scalar rot_init_y, const Scalar rot_init_z, const Scalar rot_init_w,
    const Scalar epsilon) {
  // Total ops: 613

  // Unused inputs
  (void)p_init2;
  (void)epsilon;

  // Input arrays

  // Intermediate terms (197)
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
  const Scalar _tmp20 = 2 * _tmp11 * _tmp15;
  const Scalar _tmp21 = _tmp12 * _tmp16;
  const Scalar _tmp22 = _tmp20 - _tmp21;
  const Scalar _tmp23 = Scalar(0.010999999999999999) * _tmp22;
  const Scalar _tmp24 = -_tmp23;
  const Scalar _tmp25 = -2 * std::pow(_tmp12, Scalar(2));
  const Scalar _tmp26 = -2 * std::pow(_tmp15, Scalar(2));
  const Scalar _tmp27 = Scalar(0.20999999999999999) * _tmp25 +
                        Scalar(0.20999999999999999) * _tmp26 + Scalar(0.20999999999999999);
  const Scalar _tmp28 = _tmp24 + _tmp27;
  const Scalar _tmp29 = _tmp19 + _tmp28;
  const Scalar _tmp30 = _tmp29 + p_init1;
  const Scalar _tmp31 = Scalar(0.20999999999999999) * _tmp14 - Scalar(0.20999999999999999) * _tmp17;
  const Scalar _tmp32 = 1 - 2 * std::pow(_tmp11, Scalar(2));
  const Scalar _tmp33 = Scalar(0.20999999999999999) * _tmp26 + Scalar(0.20999999999999999) * _tmp32;
  const Scalar _tmp34 = _tmp13 * _tmp15;
  const Scalar _tmp35 = _tmp11 * _tmp16;
  const Scalar _tmp36 = _tmp34 + _tmp35;
  const Scalar _tmp37 = -Scalar(0.010999999999999999) * _tmp36;
  const Scalar _tmp38 = -_tmp33 + _tmp37;
  const Scalar _tmp39 = _tmp31 + _tmp38;
  const Scalar _tmp40 = _tmp39 + p_init0;
  const Scalar _tmp41 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp42 = -_tmp27;
  const Scalar _tmp43 = _tmp24 + _tmp42;
  const Scalar _tmp44 = _tmp19 + _tmp43;
  const Scalar _tmp45 = _tmp44 + p_init1;
  const Scalar _tmp46 = _tmp45 + Scalar(8.3196563700000006);
  const Scalar _tmp47 = -_tmp31;
  const Scalar _tmp48 = _tmp38 + _tmp47;
  const Scalar _tmp49 = _tmp48 + p_init0;
  const Scalar _tmp50 = _tmp49 + Scalar(1.9874742000000001);
  const Scalar _tmp51 = std::pow(Scalar(std::pow(_tmp46, Scalar(2)) + std::pow(_tmp50, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp52 = _tmp50 * _tmp51;
  const Scalar _tmp53 = _tmp46 * _tmp51;
  const Scalar _tmp54 = _tmp33 + _tmp37;
  const Scalar _tmp55 = _tmp31 + _tmp54;
  const Scalar _tmp56 = _tmp55 + p_init0;
  const Scalar _tmp57 = _tmp56 + Scalar(-2.71799795);
  const Scalar _tmp58 = Scalar(1.0) / (_tmp57);
  const Scalar _tmp59 = _tmp18 + _tmp28;
  const Scalar _tmp60 = _tmp59 + p_init1;
  const Scalar _tmp61 = _tmp60 + Scalar(-4.7752063900000001);
  const Scalar _tmp62 = _tmp58 * _tmp61;
  const Scalar _tmp63 = _tmp52 * _tmp62 - _tmp53;
  const Scalar _tmp64 = _tmp47 + _tmp54;
  const Scalar _tmp65 = _tmp64 + p_init0;
  const Scalar _tmp66 = _tmp65 + Scalar(-2.5202214700000001);
  const Scalar _tmp67 = _tmp18 + _tmp43;
  const Scalar _tmp68 = _tmp67 + p_init1;
  const Scalar _tmp69 = _tmp68 + Scalar(8.3888750099999996);
  const Scalar _tmp70 = std::pow(Scalar(std::pow(_tmp66, Scalar(2)) + std::pow(_tmp69, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp71 = _tmp66 * _tmp70;
  const Scalar _tmp72 = _tmp69 * _tmp70;
  const Scalar _tmp73 = Scalar(1.0) / (_tmp62 * _tmp71 - _tmp72);
  const Scalar _tmp74 = Scalar(0.20999999999999999) * _tmp34 - Scalar(0.20999999999999999) * _tmp35;
  const Scalar _tmp75 =
      -Scalar(0.010999999999999999) * _tmp25 - Scalar(0.010999999999999999) * _tmp32;
  const Scalar _tmp76 = Scalar(0.20999999999999999) * _tmp20 + Scalar(0.20999999999999999) * _tmp21;
  const Scalar _tmp77 = _tmp75 - _tmp76;
  const Scalar _tmp78 = _tmp74 + _tmp77;
  const Scalar _tmp79 = _tmp75 + _tmp76;
  const Scalar _tmp80 = _tmp74 + _tmp79;
  const Scalar _tmp81 = _tmp71 * _tmp80;
  const Scalar _tmp82 = _tmp73 * (-_tmp71 * _tmp78 + _tmp81);
  const Scalar _tmp83 = _tmp52 * _tmp80;
  const Scalar _tmp84 = _tmp73 * (-_tmp62 * _tmp81 + _tmp72 * _tmp78);
  const Scalar _tmp85 = -_tmp74;
  const Scalar _tmp86 = _tmp77 + _tmp85;
  const Scalar _tmp87 = _tmp53 * _tmp86 - _tmp62 * _tmp83 - _tmp63 * _tmp84;
  const Scalar _tmp88 = Scalar(1.0) * _tmp59;
  const Scalar _tmp89 = -_tmp88;
  const Scalar _tmp90 = Scalar(1.0) / (_tmp67 + _tmp89);
  const Scalar _tmp91 = Scalar(1.0) * _tmp55;
  const Scalar _tmp92 = -_tmp64 + _tmp91;
  const Scalar _tmp93 = _tmp90 * _tmp92;
  const Scalar _tmp94 = -_tmp52 * _tmp86 - _tmp63 * _tmp82 + _tmp83 - _tmp87 * _tmp93;
  const Scalar _tmp95 = Scalar(1.0) / (_tmp94);
  const Scalar _tmp96 = _tmp88 * _tmp93 + _tmp91;
  const Scalar _tmp97 = 0;
  const Scalar _tmp98 = _tmp95 * _tmp97;
  const Scalar _tmp99 = _tmp71 * _tmp73;
  const Scalar _tmp100 = _tmp63 * _tmp95;
  const Scalar _tmp101 = _tmp100 * _tmp97;
  const Scalar _tmp102 =
      std::sqrt(Scalar(std::pow(_tmp57, Scalar(2)) + std::pow(_tmp61, Scalar(2))));
  const Scalar _tmp103 = _tmp102 * _tmp58;
  const Scalar _tmp104 = _tmp103 * (-_tmp101 * _tmp99 + _tmp52 * _tmp98);
  const Scalar _tmp105 = Scalar(1.0) / (_tmp102);
  const Scalar _tmp106 = _tmp103 * (_tmp105 * _tmp55 * _tmp61 - _tmp105 * _tmp57 * _tmp59);
  const Scalar _tmp107 = _tmp73 * (_tmp106 * _tmp71 - _tmp64 * _tmp72 + _tmp67 * _tmp71);
  const Scalar _tmp108 = _tmp106 * _tmp52 - _tmp107 * _tmp63 + _tmp44 * _tmp52 - _tmp48 * _tmp53;
  const Scalar _tmp109 = Scalar(1.0) / (_tmp108);
  const Scalar _tmp110 = Scalar(1.0) * _tmp109;
  const Scalar _tmp111 = _tmp30 + Scalar(-4.8333311099999996);
  const Scalar _tmp112 = _tmp40 + Scalar(1.79662371);
  const Scalar _tmp113 =
      std::pow(Scalar(std::pow(_tmp111, Scalar(2)) + std::pow(_tmp112, Scalar(2))),
               Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp114 = _tmp111 * _tmp113;
  const Scalar _tmp115 = _tmp112 * _tmp113;
  const Scalar _tmp116 = fh1 * (_tmp114 * _tmp39 - _tmp115 * _tmp29);
  const Scalar _tmp117 = _tmp62 * _tmp80 + _tmp62 * _tmp84;
  const Scalar _tmp118 = -_tmp117 * _tmp93 + _tmp62 * _tmp82 - _tmp80;
  const Scalar _tmp119 = _tmp108 * _tmp95;
  const Scalar _tmp120 = _tmp109 * _tmp94;
  const Scalar _tmp121 = _tmp120 * (-_tmp106 + _tmp107 * _tmp62 - _tmp118 * _tmp119);
  const Scalar _tmp122 = _tmp118 + _tmp121;
  const Scalar _tmp123 = _tmp52 * _tmp95;
  const Scalar _tmp124 = -_tmp100 * _tmp122 - _tmp62;
  const Scalar _tmp125 = _tmp115 * fh1;
  const Scalar _tmp126 = Scalar(1.0) * _tmp90;
  const Scalar _tmp127 = _tmp126 * _tmp84 * _tmp92 - Scalar(1.0) * _tmp82;
  const Scalar _tmp128 = _tmp120 * (-Scalar(1.0) * _tmp107 - _tmp119 * _tmp127);
  const Scalar _tmp129 = _tmp127 + _tmp128;
  const Scalar _tmp130 = -_tmp100 * _tmp129 + Scalar(1.0);
  const Scalar _tmp131 = _tmp114 * fh1;
  const Scalar _tmp132 = -_tmp103 * _tmp116 * (_tmp110 * _tmp52 - _tmp110 * _tmp63 * _tmp99) -
                         _tmp103 * _tmp125 * (_tmp122 * _tmp123 + _tmp124 * _tmp99 + Scalar(1.0)) -
                         _tmp103 * _tmp131 * (_tmp123 * _tmp129 + _tmp130 * _tmp99) -
                         _tmp104 * _tmp41;
  const Scalar _tmp133 = Scalar(1.0) / (_tmp132);
  const Scalar _tmp134 = _tmp44 + _tmp89;
  const Scalar _tmp135 = _tmp134 * _tmp93;
  const Scalar _tmp136 = Scalar(1.0) / (-_tmp135 - _tmp48 + _tmp91);
  const Scalar _tmp137 = _tmp136 * _tmp96;
  const Scalar _tmp138 = _tmp87 * _tmp95;
  const Scalar _tmp139 = _tmp90 * (-_tmp134 * _tmp137 - _tmp138 * _tmp97 + _tmp89);
  const Scalar _tmp140 = -Scalar(1.0) * _tmp137 - Scalar(1.0) * _tmp139 + Scalar(1.0);
  const Scalar _tmp141 = _tmp134 * _tmp136;
  const Scalar _tmp142 = _tmp128 * _tmp141 - _tmp129 * _tmp138 - Scalar(1.0) * _tmp84;
  const Scalar _tmp143 = Scalar(1.0) * _tmp136;
  const Scalar _tmp144 = fh1 * (_tmp79 + _tmp85);
  const Scalar _tmp145 = -_tmp114 * _tmp144 - Scalar(3.29616) * _tmp22 - _tmp29 * fv1;
  const Scalar _tmp146 = _tmp90 * (_tmp135 * _tmp143 + Scalar(1.0));
  const Scalar _tmp147 = _tmp143 * _tmp93;
  const Scalar _tmp148 = -Scalar(1.0) * _tmp146 + Scalar(1.0) * _tmp147;
  const Scalar _tmp149 = _tmp115 * _tmp144 + Scalar(3.29616) * _tmp36 + _tmp39 * fv1;
  const Scalar _tmp150 = _tmp134 * _tmp90;
  const Scalar _tmp151 = Scalar(1.0) * _tmp143 * _tmp150 - Scalar(1.0) * _tmp143;
  const Scalar _tmp152 = _tmp117 + _tmp121 * _tmp141 - _tmp122 * _tmp138;
  const Scalar _tmp153 = _tmp120 * _tmp143;
  const Scalar _tmp154 = -_tmp110 * _tmp87 + _tmp134 * _tmp153;
  const Scalar _tmp155 = Scalar(1.0) * _tmp116 * (-_tmp126 * _tmp154 + _tmp153) +
                         Scalar(1.0) * _tmp125 * (_tmp121 * _tmp143 - _tmp126 * _tmp152) +
                         Scalar(1.0) * _tmp131 * (-_tmp126 * _tmp142 + _tmp128 * _tmp143) +
                         _tmp140 * _tmp41 + _tmp145 * _tmp148 + _tmp149 * _tmp151;
  const Scalar _tmp156 = std::asinh(_tmp133 * _tmp155);
  const Scalar _tmp157 = Scalar(9.6622558468725703) * _tmp132;
  const Scalar _tmp158 =
      -_tmp156 * _tmp157 -
      Scalar(4.7752063900000001) *
          std::sqrt(
              Scalar(Scalar(0.32397683292140877) *
                         std::pow(Scalar(1 - Scalar(0.36791786395571047) * _tmp56), Scalar(2)) +
                     std::pow(Scalar(1 - Scalar(0.20941503221602112) * _tmp60), Scalar(2))));
  const Scalar _tmp159 = std::pow(_tmp132, Scalar(-2));
  const Scalar _tmp160 = _tmp104 * _tmp159;
  const Scalar _tmp161 = _tmp18 + _tmp23 + _tmp42;
  const Scalar _tmp162 =
      (_tmp133 * (-_tmp140 + _tmp148 * _tmp161 + _tmp151 * _tmp39) - _tmp155 * _tmp160) /
      std::sqrt(Scalar(std::pow(_tmp155, Scalar(2)) * _tmp159 + 1));
  const Scalar _tmp163 = Scalar(9.6622558468725703) * _tmp104;
  const Scalar _tmp164 = Scalar(0.1034955) * _tmp133;
  const Scalar _tmp165 = _tmp158 * _tmp164;
  const Scalar _tmp166 = Scalar(1.0) * _tmp156;
  const Scalar _tmp167 = _tmp110 * _tmp116;
  const Scalar _tmp168 = _tmp101 * _tmp73;
  const Scalar _tmp169 = _tmp124 * _tmp125 * _tmp73 + _tmp130 * _tmp131 * _tmp73 -
                         _tmp167 * _tmp63 * _tmp73 - _tmp168 * _tmp41;
  const Scalar _tmp170 = Scalar(9.6622558468725703) * _tmp169;
  const Scalar _tmp171 = Scalar(1.0) / (_tmp169);
  const Scalar _tmp172 = _tmp143 * _tmp39;
  const Scalar _tmp173 = _tmp143 * _tmp149;
  const Scalar _tmp174 = _tmp116 * _tmp154 * _tmp90 + _tmp125 * _tmp152 * _tmp90 +
                         _tmp131 * _tmp142 * _tmp90 + _tmp139 * _tmp41 + _tmp145 * _tmp146 -
                         _tmp150 * _tmp173;
  const Scalar _tmp175 = std::pow(_tmp169, Scalar(-2));
  const Scalar _tmp176 = _tmp168 * _tmp175;
  const Scalar _tmp177 =
      (_tmp171 * (-_tmp139 + _tmp146 * _tmp161 - _tmp150 * _tmp172) - _tmp174 * _tmp176) /
      std::sqrt(Scalar(std::pow(_tmp174, Scalar(2)) * _tmp175 + 1));
  const Scalar _tmp178 = std::asinh(_tmp171 * _tmp174);
  const Scalar _tmp179 = Scalar(9.6622558468725703) * _tmp168;
  const Scalar _tmp180 = Scalar(0.1034955) * _tmp171;
  const Scalar _tmp181 =
      -_tmp170 * _tmp178 -
      Scalar(8.3888750099999996) *
          std::sqrt(
              Scalar(Scalar(0.090254729040973036) *
                         std::pow(Scalar(1 - Scalar(0.39679052492160538) * _tmp65), Scalar(2)) +
                     std::pow(Scalar(-Scalar(0.11920549523123722) * _tmp68 - 1), Scalar(2))));
  const Scalar _tmp182 = _tmp180 * _tmp181;
  const Scalar _tmp183 = Scalar(1.0) * _tmp178;
  const Scalar _tmp184 =
      _tmp122 * _tmp125 * _tmp95 + _tmp129 * _tmp131 * _tmp95 + _tmp167 + _tmp41 * _tmp98;
  const Scalar _tmp185 = Scalar(1.0) / (_tmp184);
  const Scalar _tmp186 = -_tmp116 * _tmp153 - _tmp121 * _tmp125 * _tmp136 -
                         _tmp128 * _tmp131 * _tmp136 + _tmp137 * _tmp41 - _tmp145 * _tmp147 +
                         _tmp173;
  const Scalar _tmp187 = std::asinh(_tmp185 * _tmp186);
  const Scalar _tmp188 = Scalar(1.0) * _tmp187;
  const Scalar _tmp189 = Scalar(9.6622558468725703) * _tmp184;
  const Scalar _tmp190 =
      -_tmp187 * _tmp189 -
      Scalar(8.3196563700000006) *
          std::sqrt(
              Scalar(std::pow(Scalar(-Scalar(0.12019727204189803) * _tmp45 - 1), Scalar(2)) +
                     Scalar(0.057067943376852184) *
                         std::pow(Scalar(-Scalar(0.50315118556004401) * _tmp49 - 1), Scalar(2))));
  const Scalar _tmp191 = Scalar(0.1034955) * _tmp185;
  const Scalar _tmp192 = _tmp190 * _tmp191;
  const Scalar _tmp193 = Scalar(9.6622558468725703) * _tmp98;
  const Scalar _tmp194 = std::pow(_tmp184, Scalar(-2));
  const Scalar _tmp195 = _tmp194 * _tmp98;
  const Scalar _tmp196 = (_tmp185 * (-_tmp137 - _tmp147 * _tmp161 + _tmp172) + _tmp186 * _tmp195) /
                         std::sqrt(Scalar(std::pow(_tmp186, Scalar(2)) * _tmp194 + 1));

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) =
      _tmp3 *
      (-_tmp2 * std::cosh(Scalar(1.0) * _tmp1) +
       _tmp2 * std::cosh(
                   Scalar(0.1034955) * _tmp0 *
                   (-_tmp1 * _tmp3 -
                    Scalar(4.8333311099999996) *
                        std::sqrt(Scalar(
                            std::pow(Scalar(1 - Scalar(0.20689664689659551) * _tmp30), Scalar(2)) +
                            Scalar(0.13817235445745474) *
                                std::pow(Scalar(-Scalar(0.55659957866191134) * _tmp40 - 1),
                                         Scalar(2)))))));
  _res(1, 0) = _tmp157 * (-Scalar(1.0) * _tmp162 * std::cosh(_tmp166) -
                          (-Scalar(0.1034955) * _tmp158 * _tmp160 +
                           _tmp164 * (-_tmp156 * _tmp163 - _tmp157 * _tmp162)) *
                              std::cosh(_tmp165)) +
               _tmp163 * (-std::sinh(_tmp165) - std::sinh(_tmp166));
  _res(2, 0) = _tmp170 * (-Scalar(1.0) * _tmp177 * std::cosh(_tmp183) -
                          (-Scalar(0.1034955) * _tmp176 * _tmp181 +
                           _tmp180 * (-_tmp170 * _tmp177 - _tmp178 * _tmp179)) *
                              std::cosh(_tmp182)) +
               _tmp179 * (-std::sinh(_tmp182) - std::sinh(_tmp183));
  _res(3, 0) = _tmp189 * (-Scalar(1.0) * _tmp196 * std::cosh(_tmp188) -
                          (Scalar(0.1034955) * _tmp190 * _tmp195 +
                           _tmp191 * (_tmp187 * _tmp193 - _tmp189 * _tmp196)) *
                              std::cosh(_tmp192)) -
               _tmp193 * (-std::sinh(_tmp188) - std::sinh(_tmp192));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym