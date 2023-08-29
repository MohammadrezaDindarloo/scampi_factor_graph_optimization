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
 * Symbolic function: IK_residual_func_cost2_wrt_fv1_Nl14
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost2WrtFv1Nl14(
    const Scalar fh1, const Scalar fv1, const Scalar rx, const Scalar ry, const Scalar rz,
    const Scalar p_init0, const Scalar p_init1, const Scalar p_init2, const Scalar rot_init_x,
    const Scalar rot_init_y, const Scalar rot_init_z, const Scalar rot_init_w,
    const Scalar epsilon) {
  // Total ops: 609

  // Unused inputs
  (void)p_init2;
  (void)epsilon;

  // Input arrays

  // Intermediate terms (196)
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
  const Scalar _tmp27 = _tmp22 + _tmp26;
  const Scalar _tmp28 = _tmp15 + _tmp27;
  const Scalar _tmp29 = _tmp28 + p_init1;
  const Scalar _tmp30 = -2 * std::pow(_tmp16, Scalar(2));
  const Scalar _tmp31 = Scalar(0.20999999999999999) * _tmp12 +
                        Scalar(0.20999999999999999) * _tmp30 + Scalar(0.20999999999999999);
  const Scalar _tmp32 = _tmp11 * _tmp23;
  const Scalar _tmp33 = _tmp16 * _tmp18;
  const Scalar _tmp34 = _tmp32 + _tmp33;
  const Scalar _tmp35 = -Scalar(0.010999999999999999) * _tmp34;
  const Scalar _tmp36 = Scalar(0.20999999999999999) * _tmp24 - Scalar(0.20999999999999999) * _tmp25;
  const Scalar _tmp37 = _tmp35 + _tmp36;
  const Scalar _tmp38 = _tmp31 + _tmp37;
  const Scalar _tmp39 = _tmp38 + p_init0;
  const Scalar _tmp40 = _tmp35 - _tmp36;
  const Scalar _tmp41 = _tmp31 + _tmp40;
  const Scalar _tmp42 = _tmp41 + p_init0;
  const Scalar _tmp43 = _tmp42 + Scalar(-2.5202214700000001);
  const Scalar _tmp44 = Scalar(1.0) / (_tmp43);
  const Scalar _tmp45 = -_tmp15;
  const Scalar _tmp46 = _tmp27 + _tmp45;
  const Scalar _tmp47 = _tmp46 + p_init1;
  const Scalar _tmp48 = _tmp47 + Scalar(8.3888750099999996);
  const Scalar _tmp49 = _tmp44 * _tmp48;
  const Scalar _tmp50 = -_tmp26;
  const Scalar _tmp51 = _tmp22 + _tmp50;
  const Scalar _tmp52 = _tmp15 + _tmp51;
  const Scalar _tmp53 = _tmp52 + p_init1;
  const Scalar _tmp54 = _tmp53 + Scalar(-4.8333311099999996);
  const Scalar _tmp55 = -_tmp31;
  const Scalar _tmp56 = _tmp37 + _tmp55;
  const Scalar _tmp57 = _tmp56 + p_init0;
  const Scalar _tmp58 = _tmp57 + Scalar(1.79662371);
  const Scalar _tmp59 = std::pow(Scalar(std::pow(_tmp54, Scalar(2)) + std::pow(_tmp58, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp60 = _tmp58 * _tmp59;
  const Scalar _tmp61 = _tmp54 * _tmp59;
  const Scalar _tmp62 = _tmp49 * _tmp60 - _tmp61;
  const Scalar _tmp63 = Scalar(0.20999999999999999) * _tmp17 + Scalar(0.20999999999999999) * _tmp19;
  const Scalar _tmp64 = -_tmp63;
  const Scalar _tmp65 =
      -Scalar(0.010999999999999999) * _tmp14 - Scalar(0.010999999999999999) * _tmp30;
  const Scalar _tmp66 = Scalar(0.20999999999999999) * _tmp32 - Scalar(0.20999999999999999) * _tmp33;
  const Scalar _tmp67 = _tmp65 + _tmp66;
  const Scalar _tmp68 = _tmp64 + _tmp67;
  const Scalar _tmp69 = _tmp60 * _tmp68;
  const Scalar _tmp70 = _tmp65 - _tmp66;
  const Scalar _tmp71 = _tmp64 + _tmp70;
  const Scalar _tmp72 = _tmp45 + _tmp51;
  const Scalar _tmp73 = _tmp72 + p_init1;
  const Scalar _tmp74 = _tmp73 + Scalar(8.3196563700000006);
  const Scalar _tmp75 = _tmp40 + _tmp55;
  const Scalar _tmp76 = _tmp75 + p_init0;
  const Scalar _tmp77 = _tmp76 + Scalar(1.9874742000000001);
  const Scalar _tmp78 = std::pow(Scalar(std::pow(_tmp74, Scalar(2)) + std::pow(_tmp77, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp79 = _tmp74 * _tmp78;
  const Scalar _tmp80 = _tmp77 * _tmp78;
  const Scalar _tmp81 = _tmp68 * _tmp80;
  const Scalar _tmp82 = -_tmp49 * _tmp81 + _tmp71 * _tmp79;
  const Scalar _tmp83 = Scalar(1.0) / (_tmp49 * _tmp80 - _tmp79);
  const Scalar _tmp84 = _tmp62 * _tmp83;
  const Scalar _tmp85 = _tmp63 + _tmp70;
  const Scalar _tmp86 = -_tmp49 * _tmp69 + _tmp61 * _tmp85 - _tmp82 * _tmp84;
  const Scalar _tmp87 = Scalar(1.0) * _tmp46;
  const Scalar _tmp88 = -_tmp87;
  const Scalar _tmp89 = Scalar(1.0) / (_tmp72 + _tmp88);
  const Scalar _tmp90 = Scalar(1.0) * _tmp41;
  const Scalar _tmp91 = -_tmp75 + _tmp90;
  const Scalar _tmp92 = _tmp89 * _tmp91;
  const Scalar _tmp93 = -_tmp71 * _tmp80 + _tmp81;
  const Scalar _tmp94 = -_tmp60 * _tmp85 + _tmp69 - _tmp84 * _tmp93 - _tmp86 * _tmp92;
  const Scalar _tmp95 = Scalar(1.0) / (_tmp94);
  const Scalar _tmp96 =
      std::sqrt(Scalar(std::pow(_tmp43, Scalar(2)) + std::pow(_tmp48, Scalar(2))));
  const Scalar _tmp97 = Scalar(1.0) / (_tmp96);
  const Scalar _tmp98 = _tmp44 * _tmp96;
  const Scalar _tmp99 = _tmp98 * (_tmp41 * _tmp48 * _tmp97 - _tmp43 * _tmp46 * _tmp97);
  const Scalar _tmp100 = _tmp83 * (_tmp72 * _tmp80 - _tmp75 * _tmp79 + _tmp80 * _tmp99);
  const Scalar _tmp101 = Scalar(1.0) * _tmp83;
  const Scalar _tmp102 = Scalar(1.0) * _tmp89;
  const Scalar _tmp103 = -_tmp101 * _tmp93 + _tmp102 * _tmp82 * _tmp83 * _tmp91;
  const Scalar _tmp104 = -_tmp100 * _tmp62 + _tmp52 * _tmp60 - _tmp56 * _tmp61 + _tmp60 * _tmp99;
  const Scalar _tmp105 = _tmp104 * _tmp95;
  const Scalar _tmp106 = Scalar(1.0) / (_tmp104);
  const Scalar _tmp107 = _tmp106 * _tmp94;
  const Scalar _tmp108 = _tmp107 * (-Scalar(1.0) * _tmp100 - _tmp103 * _tmp105);
  const Scalar _tmp109 = _tmp95 * (_tmp103 + _tmp108);
  const Scalar _tmp110 = -_tmp109 * _tmp62 + Scalar(1.0);
  const Scalar _tmp111 = _tmp80 * _tmp83;
  const Scalar _tmp112 = _tmp29 + Scalar(-4.7752063900000001);
  const Scalar _tmp113 = _tmp39 + Scalar(-2.71799795);
  const Scalar _tmp114 =
      std::pow(Scalar(std::pow(_tmp112, Scalar(2)) + std::pow(_tmp113, Scalar(2))),
               Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp115 = _tmp112 * _tmp114;
  const Scalar _tmp116 = _tmp115 * fh1;
  const Scalar _tmp117 = Scalar(1.0) * _tmp106;
  const Scalar _tmp118 = _tmp113 * _tmp114;
  const Scalar _tmp119 = fh1 * (_tmp115 * _tmp38 - _tmp118 * _tmp28);
  const Scalar _tmp120 = _tmp49 * _tmp83;
  const Scalar _tmp121 = _tmp120 * _tmp82 + _tmp49 * _tmp68;
  const Scalar _tmp122 = _tmp120 * _tmp93 - _tmp121 * _tmp92 - _tmp68;
  const Scalar _tmp123 = _tmp107 * (_tmp100 * _tmp49 - _tmp105 * _tmp122 - _tmp99);
  const Scalar _tmp124 = _tmp95 * (_tmp122 + _tmp123);
  const Scalar _tmp125 = -_tmp124 * _tmp62 - _tmp49;
  const Scalar _tmp126 = _tmp118 * fh1;
  const Scalar _tmp127 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp128 = _tmp87 * _tmp92 + _tmp90;
  const Scalar _tmp129 = 0;
  const Scalar _tmp130 = _tmp129 * _tmp84;
  const Scalar _tmp131 = _tmp98 * (_tmp129 * _tmp60 - _tmp130 * _tmp80);
  const Scalar _tmp132 = -_tmp116 * _tmp98 * (_tmp109 * _tmp60 + _tmp110 * _tmp111) -
                         _tmp119 * _tmp98 * (_tmp117 * _tmp60 - _tmp117 * _tmp80 * _tmp84) -
                         _tmp126 * _tmp98 * (_tmp111 * _tmp125 + _tmp124 * _tmp60 + Scalar(1.0)) -
                         _tmp127 * _tmp131;
  const Scalar _tmp133 = Scalar(1.0) / (_tmp132);
  const Scalar _tmp134 = fh1 * (_tmp63 + _tmp67);
  const Scalar _tmp135 = -_tmp115 * _tmp134 - Scalar(3.29616) * _tmp20 - _tmp28 * fv1;
  const Scalar _tmp136 = _tmp52 + _tmp88;
  const Scalar _tmp137 = _tmp136 * _tmp92;
  const Scalar _tmp138 = Scalar(1.0) / (-_tmp137 - _tmp56 + _tmp90);
  const Scalar _tmp139 = Scalar(1.0) * _tmp138;
  const Scalar _tmp140 = _tmp89 * (_tmp137 * _tmp139 + Scalar(1.0));
  const Scalar _tmp141 = _tmp139 * _tmp92;
  const Scalar _tmp142 = -Scalar(1.0) * _tmp140 + Scalar(1.0) * _tmp141;
  const Scalar _tmp143 = _tmp136 * _tmp138;
  const Scalar _tmp144 = -_tmp101 * _tmp82 + _tmp108 * _tmp143 - _tmp109 * _tmp86;
  const Scalar _tmp145 = _tmp118 * _tmp134 + Scalar(3.29616) * _tmp34 + _tmp38 * fv1;
  const Scalar _tmp146 = _tmp136 * _tmp89;
  const Scalar _tmp147 = Scalar(1.0) * _tmp139 * _tmp146 - Scalar(1.0) * _tmp139;
  const Scalar _tmp148 = _tmp128 * _tmp138;
  const Scalar _tmp149 = _tmp89 * (-_tmp129 * _tmp86 - _tmp136 * _tmp148 + _tmp88);
  const Scalar _tmp150 = -Scalar(1.0) * _tmp148 - Scalar(1.0) * _tmp149 + Scalar(1.0);
  const Scalar _tmp151 = _tmp107 * _tmp139;
  const Scalar _tmp152 = -_tmp117 * _tmp86 + _tmp136 * _tmp151;
  const Scalar _tmp153 = _tmp121 + _tmp123 * _tmp143 - _tmp124 * _tmp86;
  const Scalar _tmp154 = Scalar(1.0) * _tmp116 * (-_tmp102 * _tmp144 + _tmp108 * _tmp139) +
                         Scalar(1.0) * _tmp119 * (-_tmp102 * _tmp152 + _tmp151) +
                         Scalar(1.0) * _tmp126 * (-_tmp102 * _tmp153 + _tmp123 * _tmp139) +
                         _tmp127 * _tmp150 + _tmp135 * _tmp142 + _tmp145 * _tmp147;
  const Scalar _tmp155 = std::asinh(_tmp133 * _tmp154);
  const Scalar _tmp156 = Scalar(9.6622558468725703) * _tmp132;
  const Scalar _tmp157 =
      -_tmp155 * _tmp156 -
      Scalar(8.3888750099999996) *
          std::sqrt(
              Scalar(Scalar(0.090254729040973036) *
                         std::pow(Scalar(1 - Scalar(0.39679052492160538) * _tmp42), Scalar(2)) +
                     std::pow(Scalar(-Scalar(0.11920549523123722) * _tmp47 - 1), Scalar(2))));
  const Scalar _tmp158 = Scalar(0.1034955) * _tmp133;
  const Scalar _tmp159 = _tmp157 * _tmp158;
  const Scalar _tmp160 = Scalar(1.0) * _tmp155;
  const Scalar _tmp161 = Scalar(9.6622558468725703) * _tmp131;
  const Scalar _tmp162 = std::pow(_tmp132, Scalar(-2));
  const Scalar _tmp163 = _tmp21 + _tmp45 + _tmp50;
  const Scalar _tmp164 = _tmp131 * _tmp162;
  const Scalar _tmp165 =
      (_tmp133 * (_tmp142 * _tmp163 + _tmp147 * _tmp38 - _tmp150) - _tmp154 * _tmp164) /
      std::sqrt(Scalar(std::pow(_tmp154, Scalar(2)) * _tmp162 + 1));
  const Scalar _tmp166 = _tmp127 * _tmp129;
  const Scalar _tmp167 = _tmp117 * _tmp119;
  const Scalar _tmp168 =
      _tmp110 * _tmp116 * _tmp83 + _tmp125 * _tmp126 * _tmp83 - _tmp166 * _tmp84 - _tmp167 * _tmp84;
  const Scalar _tmp169 = Scalar(1.0) / (_tmp168);
  const Scalar _tmp170 = _tmp139 * _tmp145;
  const Scalar _tmp171 = _tmp116 * _tmp144 * _tmp89 + _tmp119 * _tmp152 * _tmp89 +
                         _tmp126 * _tmp153 * _tmp89 + _tmp127 * _tmp149 + _tmp135 * _tmp140 -
                         _tmp146 * _tmp170;
  const Scalar _tmp172 = std::asinh(_tmp169 * _tmp171);
  const Scalar _tmp173 = Scalar(1.0) * _tmp172;
  const Scalar _tmp174 = Scalar(9.6622558468725703) * _tmp168;
  const Scalar _tmp175 =
      -_tmp172 * _tmp174 -
      Scalar(8.3196563700000006) *
          std::sqrt(
              Scalar(std::pow(Scalar(-Scalar(0.12019727204189803) * _tmp73 - 1), Scalar(2)) +
                     Scalar(0.057067943376852184) *
                         std::pow(Scalar(-Scalar(0.50315118556004401) * _tmp76 - 1), Scalar(2))));
  const Scalar _tmp176 = Scalar(0.1034955) * _tmp169;
  const Scalar _tmp177 = _tmp175 * _tmp176;
  const Scalar _tmp178 = Scalar(9.6622558468725703) * _tmp129;
  const Scalar _tmp179 = _tmp178 * _tmp84;
  const Scalar _tmp180 = std::pow(_tmp168, Scalar(-2));
  const Scalar _tmp181 = _tmp139 * _tmp38;
  const Scalar _tmp182 = _tmp130 * _tmp180;
  const Scalar _tmp183 =
      (_tmp169 * (_tmp140 * _tmp163 - _tmp146 * _tmp181 - _tmp149) - _tmp171 * _tmp182) /
      std::sqrt(Scalar(std::pow(_tmp171, Scalar(2)) * _tmp180 + 1));
  const Scalar _tmp184 = -_tmp108 * _tmp116 * _tmp138 - _tmp119 * _tmp151 -
                         _tmp123 * _tmp126 * _tmp138 + _tmp127 * _tmp148 - _tmp135 * _tmp141 +
                         _tmp170;
  const Scalar _tmp185 = _tmp109 * _tmp116 + _tmp124 * _tmp126 + _tmp166 + _tmp167;
  const Scalar _tmp186 = Scalar(1.0) / (_tmp185);
  const Scalar _tmp187 = std::asinh(_tmp184 * _tmp186);
  const Scalar _tmp188 = Scalar(9.6622558468725703) * _tmp185;
  const Scalar _tmp189 =
      -_tmp187 * _tmp188 -
      Scalar(4.8333311099999996) *
          std::sqrt(
              Scalar(std::pow(Scalar(1 - Scalar(0.20689664689659551) * _tmp53), Scalar(2)) +
                     Scalar(0.13817235445745474) *
                         std::pow(Scalar(-Scalar(0.55659957866191134) * _tmp57 - 1), Scalar(2))));
  const Scalar _tmp190 = Scalar(0.1034955) * _tmp186;
  const Scalar _tmp191 = _tmp189 * _tmp190;
  const Scalar _tmp192 = std::pow(_tmp185, Scalar(-2));
  const Scalar _tmp193 = _tmp129 * _tmp192;
  const Scalar _tmp194 = (_tmp184 * _tmp193 + _tmp186 * (-_tmp141 * _tmp163 - _tmp148 + _tmp181)) /
                         std::sqrt(Scalar(std::pow(_tmp184, Scalar(2)) * _tmp192 + 1));
  const Scalar _tmp195 = Scalar(1.0) * _tmp187;

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
                            std::pow(Scalar(1 - Scalar(0.20941503221602112) * _tmp29), Scalar(2)) +
                            Scalar(0.32397683292140877) *
                                std::pow(Scalar(1 - Scalar(0.36791786395571047) * _tmp39),
                                         Scalar(2)))))));
  _res(1, 0) = _tmp156 * (-Scalar(1.0) * _tmp165 * std::cosh(_tmp160) -
                          (-Scalar(0.1034955) * _tmp157 * _tmp164 +
                           _tmp158 * (-_tmp155 * _tmp161 - _tmp156 * _tmp165)) *
                              std::cosh(_tmp159)) +
               _tmp161 * (-std::sinh(_tmp159) - std::sinh(_tmp160));
  _res(2, 0) = _tmp174 * (-Scalar(1.0) * _tmp183 * std::cosh(_tmp173) -
                          (-Scalar(0.1034955) * _tmp175 * _tmp182 +
                           _tmp176 * (-_tmp172 * _tmp179 - _tmp174 * _tmp183)) *
                              std::cosh(_tmp177)) +
               _tmp179 * (-std::sinh(_tmp173) - std::sinh(_tmp177));
  _res(3, 0) = -_tmp178 * (-std::sinh(_tmp191) - std::sinh(_tmp195)) +
               _tmp188 * (-Scalar(1.0) * _tmp194 * std::cosh(_tmp195) -
                          (Scalar(0.1034955) * _tmp189 * _tmp193 +
                           _tmp190 * (_tmp178 * _tmp187 - _tmp188 * _tmp194)) *
                              std::cosh(_tmp191));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym