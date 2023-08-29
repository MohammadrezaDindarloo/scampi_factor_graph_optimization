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
 * Symbolic function: IK_residual_func_cost2_wrt_fv1_Nl19
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost2WrtFv1Nl19(
    const Scalar fh1, const Scalar fv1, const Scalar rx, const Scalar ry, const Scalar rz,
    const Scalar p_init0, const Scalar p_init1, const Scalar p_init2, const Scalar rot_init_x,
    const Scalar rot_init_y, const Scalar rot_init_z, const Scalar rot_init_w,
    const Scalar epsilon) {
  // Total ops: 617

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
  const Scalar _tmp8 = _tmp7 * rot_init_x;
  const Scalar _tmp9 = _tmp7 * rx;
  const Scalar _tmp10 = _tmp7 * rz;
  const Scalar _tmp11 = _tmp10 * rot_init_w + _tmp6 * rot_init_z + _tmp8 * ry - _tmp9 * rot_init_y;
  const Scalar _tmp12 = -2 * std::pow(_tmp11, Scalar(2));
  const Scalar _tmp13 = _tmp7 * rot_init_z;
  const Scalar _tmp14 = _tmp10 * rot_init_y - _tmp13 * ry + _tmp6 * rot_init_x + _tmp9 * rot_init_w;
  const Scalar _tmp15 = 1 - 2 * std::pow(_tmp14, Scalar(2));
  const Scalar _tmp16 = Scalar(0.20999999999999999) * _tmp12 + Scalar(0.20999999999999999) * _tmp15;
  const Scalar _tmp17 = _tmp7 * ry;
  const Scalar _tmp18 = _tmp13 * rx + _tmp17 * rot_init_w + _tmp6 * rot_init_y - _tmp8 * rz;
  const Scalar _tmp19 = 2 * _tmp11 * _tmp18;
  const Scalar _tmp20 =
      -2 * _tmp13 * rz - 2 * _tmp17 * rot_init_y + 2 * _tmp6 * rot_init_w - 2 * _tmp8 * rx;
  const Scalar _tmp21 = _tmp14 * _tmp20;
  const Scalar _tmp22 = _tmp19 - _tmp21;
  const Scalar _tmp23 = Scalar(0.010999999999999999) * _tmp22;
  const Scalar _tmp24 = -_tmp23;
  const Scalar _tmp25 = 2 * _tmp14;
  const Scalar _tmp26 = _tmp18 * _tmp25;
  const Scalar _tmp27 = _tmp11 * _tmp20;
  const Scalar _tmp28 = Scalar(0.20999999999999999) * _tmp26 + Scalar(0.20999999999999999) * _tmp27;
  const Scalar _tmp29 = _tmp24 - _tmp28;
  const Scalar _tmp30 = _tmp16 + _tmp29;
  const Scalar _tmp31 = _tmp30 + p_init1;
  const Scalar _tmp32 = -2 * std::pow(_tmp18, Scalar(2));
  const Scalar _tmp33 = Scalar(0.20999999999999999) * _tmp12 +
                        Scalar(0.20999999999999999) * _tmp32 + Scalar(0.20999999999999999);
  const Scalar _tmp34 = -_tmp33;
  const Scalar _tmp35 = _tmp11 * _tmp25;
  const Scalar _tmp36 = _tmp18 * _tmp20;
  const Scalar _tmp37 = _tmp35 + _tmp36;
  const Scalar _tmp38 = -Scalar(0.010999999999999999) * _tmp37;
  const Scalar _tmp39 = Scalar(0.20999999999999999) * _tmp26 - Scalar(0.20999999999999999) * _tmp27;
  const Scalar _tmp40 = _tmp38 + _tmp39;
  const Scalar _tmp41 = _tmp34 + _tmp40;
  const Scalar _tmp42 = _tmp41 + p_init0;
  const Scalar _tmp43 = _tmp38 - _tmp39;
  const Scalar _tmp44 = _tmp33 + _tmp43;
  const Scalar _tmp45 = _tmp34 + _tmp43;
  const Scalar _tmp46 = Scalar(1.0) * _tmp45;
  const Scalar _tmp47 = -_tmp16;
  const Scalar _tmp48 = _tmp29 + _tmp47;
  const Scalar _tmp49 = Scalar(1.0) * _tmp48;
  const Scalar _tmp50 = -_tmp49;
  const Scalar _tmp51 = _tmp28 + _tmp47;
  const Scalar _tmp52 = _tmp24 + _tmp51;
  const Scalar _tmp53 = _tmp50 + _tmp52;
  const Scalar _tmp54 = _tmp16 + _tmp24 + _tmp28;
  const Scalar _tmp55 = Scalar(1.0) / (_tmp50 + _tmp54);
  const Scalar _tmp56 = _tmp33 + _tmp40;
  const Scalar _tmp57 = _tmp46 - _tmp56;
  const Scalar _tmp58 = _tmp55 * _tmp57;
  const Scalar _tmp59 = _tmp53 * _tmp58;
  const Scalar _tmp60 = Scalar(1.0) / (-_tmp44 + _tmp46 - _tmp59);
  const Scalar _tmp61 = Scalar(1.0) * _tmp60;
  const Scalar _tmp62 = Scalar(0.20999999999999999) * _tmp35 - Scalar(0.20999999999999999) * _tmp36;
  const Scalar _tmp63 =
      -Scalar(0.010999999999999999) * _tmp15 - Scalar(0.010999999999999999) * _tmp32;
  const Scalar _tmp64 = Scalar(0.20999999999999999) * _tmp19 + Scalar(0.20999999999999999) * _tmp21;
  const Scalar _tmp65 = _tmp63 - _tmp64;
  const Scalar _tmp66 = _tmp62 + _tmp65;
  const Scalar _tmp67 = _tmp44 + p_init0;
  const Scalar _tmp68 = _tmp67 + Scalar(-2.5202214700000001);
  const Scalar _tmp69 = _tmp52 + p_init1;
  const Scalar _tmp70 = _tmp69 + Scalar(8.3888750099999996);
  const Scalar _tmp71 = std::pow(Scalar(std::pow(_tmp68, Scalar(2)) + std::pow(_tmp70, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp72 = _tmp70 * _tmp71;
  const Scalar _tmp73 = _tmp48 + p_init1;
  const Scalar _tmp74 = _tmp73 + Scalar(8.3196563700000006);
  const Scalar _tmp75 = _tmp45 + p_init0;
  const Scalar _tmp76 = _tmp75 + Scalar(1.9874742000000001);
  const Scalar _tmp77 = Scalar(1.0) / (_tmp76);
  const Scalar _tmp78 = _tmp74 * _tmp77;
  const Scalar _tmp79 = _tmp68 * _tmp71;
  const Scalar _tmp80 = -_tmp72 + _tmp78 * _tmp79;
  const Scalar _tmp81 = _tmp54 + p_init1;
  const Scalar _tmp82 = _tmp81 + Scalar(-4.7752063900000001);
  const Scalar _tmp83 = _tmp56 + p_init0;
  const Scalar _tmp84 = _tmp83 + Scalar(-2.71799795);
  const Scalar _tmp85 = std::pow(Scalar(std::pow(_tmp82, Scalar(2)) + std::pow(_tmp84, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp86 = _tmp84 * _tmp85;
  const Scalar _tmp87 = _tmp82 * _tmp85;
  const Scalar _tmp88 = Scalar(1.0) / (_tmp78 * _tmp86 - _tmp87);
  const Scalar _tmp89 = -_tmp62;
  const Scalar _tmp90 = _tmp65 + _tmp89;
  const Scalar _tmp91 = _tmp86 * _tmp90;
  const Scalar _tmp92 = _tmp63 + _tmp64;
  const Scalar _tmp93 = _tmp62 + _tmp92;
  const Scalar _tmp94 = _tmp88 * (-_tmp78 * _tmp91 + _tmp87 * _tmp93);
  const Scalar _tmp95 = _tmp79 * _tmp90;
  const Scalar _tmp96 = _tmp66 * _tmp72 - _tmp78 * _tmp95 - _tmp80 * _tmp94;
  const Scalar _tmp97 = _tmp88 * (-_tmp86 * _tmp93 + _tmp91);
  const Scalar _tmp98 = -_tmp58 * _tmp96 - _tmp66 * _tmp79 - _tmp80 * _tmp97 + _tmp95;
  const Scalar _tmp99 =
      std::sqrt(Scalar(std::pow(_tmp74, Scalar(2)) + std::pow(_tmp76, Scalar(2))));
  const Scalar _tmp100 = Scalar(1.0) / (_tmp99);
  const Scalar _tmp101 = _tmp77 * _tmp99;
  const Scalar _tmp102 = _tmp101 * (_tmp100 * _tmp45 * _tmp74 - _tmp100 * _tmp48 * _tmp76);
  const Scalar _tmp103 = _tmp88 * (_tmp102 * _tmp86 + _tmp54 * _tmp86 - _tmp56 * _tmp87);
  const Scalar _tmp104 = _tmp102 * _tmp79 - _tmp103 * _tmp80 - _tmp44 * _tmp72 + _tmp52 * _tmp79;
  const Scalar _tmp105 = Scalar(1.0) / (_tmp104);
  const Scalar _tmp106 = _tmp105 * _tmp98;
  const Scalar _tmp107 = _tmp106 * _tmp61;
  const Scalar _tmp108 = Scalar(1.0) * _tmp105;
  const Scalar _tmp109 = _tmp107 * _tmp53 - _tmp108 * _tmp96;
  const Scalar _tmp110 = Scalar(1.0) * _tmp55;
  const Scalar _tmp111 = _tmp31 + Scalar(-4.8333311099999996);
  const Scalar _tmp112 = _tmp42 + Scalar(1.79662371);
  const Scalar _tmp113 =
      std::pow(Scalar(std::pow(_tmp111, Scalar(2)) + std::pow(_tmp112, Scalar(2))),
               Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp114 = _tmp111 * _tmp113;
  const Scalar _tmp115 = _tmp112 * _tmp113;
  const Scalar _tmp116 = fh1 * (_tmp114 * _tmp41 - _tmp115 * _tmp30);
  const Scalar _tmp117 = fh1 * (_tmp89 + _tmp92);
  const Scalar _tmp118 = -_tmp114 * _tmp117 - Scalar(3.29616) * _tmp22 - _tmp30 * fv1;
  const Scalar _tmp119 = _tmp59 * _tmp61 + Scalar(1.0);
  const Scalar _tmp120 = _tmp58 * _tmp61;
  const Scalar _tmp121 = -Scalar(1.0) * _tmp110 * _tmp119 + Scalar(1.0) * _tmp120;
  const Scalar _tmp122 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp123 = _tmp46 + _tmp49 * _tmp58;
  const Scalar _tmp124 = _tmp123 * _tmp60;
  const Scalar _tmp125 = Scalar(1.0) / (_tmp98);
  const Scalar _tmp126 = 0;
  const Scalar _tmp127 = _tmp125 * _tmp126;
  const Scalar _tmp128 = _tmp55 * (-_tmp124 * _tmp53 - _tmp127 * _tmp96 + _tmp50);
  const Scalar _tmp129 = -Scalar(1.0) * _tmp123 * _tmp61 - Scalar(1.0) * _tmp128 + Scalar(1.0);
  const Scalar _tmp130 = _tmp110 * _tmp57 * _tmp94 - Scalar(1.0) * _tmp97;
  const Scalar _tmp131 = _tmp104 * _tmp125;
  const Scalar _tmp132 = _tmp106 * (-Scalar(1.0) * _tmp103 - _tmp130 * _tmp131);
  const Scalar _tmp133 = _tmp130 + _tmp132;
  const Scalar _tmp134 = _tmp125 * _tmp96;
  const Scalar _tmp135 = _tmp53 * _tmp60;
  const Scalar _tmp136 = _tmp132 * _tmp135 - _tmp133 * _tmp134 - Scalar(1.0) * _tmp94;
  const Scalar _tmp137 = _tmp114 * fh1;
  const Scalar _tmp138 = _tmp78 * _tmp90 + _tmp78 * _tmp94;
  const Scalar _tmp139 = -_tmp138 * _tmp58 + _tmp78 * _tmp97 - _tmp90;
  const Scalar _tmp140 = _tmp106 * (-_tmp102 + _tmp103 * _tmp78 - _tmp131 * _tmp139);
  const Scalar _tmp141 = _tmp139 + _tmp140;
  const Scalar _tmp142 = -_tmp134 * _tmp141 + _tmp135 * _tmp140 + _tmp138;
  const Scalar _tmp143 = _tmp115 * fh1;
  const Scalar _tmp144 = _tmp115 * _tmp117 + Scalar(3.29616) * _tmp37 + _tmp41 * fv1;
  const Scalar _tmp145 = _tmp53 * _tmp55;
  const Scalar _tmp146 = Scalar(1.0) * _tmp145 * _tmp61 - Scalar(1.0) * _tmp61;
  const Scalar _tmp147 =
      Scalar(1.0) * _tmp116 * (_tmp107 - _tmp109 * _tmp110) + _tmp118 * _tmp121 +
      _tmp122 * _tmp129 + Scalar(1.0) * _tmp137 * (-_tmp110 * _tmp136 + _tmp132 * _tmp61) +
      Scalar(1.0) * _tmp143 * (-_tmp110 * _tmp142 + _tmp140 * _tmp61) + _tmp144 * _tmp146;
  const Scalar _tmp148 = _tmp125 * _tmp79;
  const Scalar _tmp149 = _tmp125 * _tmp80;
  const Scalar _tmp150 = -_tmp141 * _tmp149 - _tmp78;
  const Scalar _tmp151 = _tmp86 * _tmp88;
  const Scalar _tmp152 = -_tmp133 * _tmp149 + Scalar(1.0);
  const Scalar _tmp153 = _tmp126 * _tmp149;
  const Scalar _tmp154 = _tmp101 * (_tmp127 * _tmp79 - _tmp151 * _tmp153);
  const Scalar _tmp155 = -_tmp101 * _tmp116 * (-_tmp108 * _tmp151 * _tmp80 + _tmp108 * _tmp79) -
                         _tmp101 * _tmp137 * (_tmp133 * _tmp148 + _tmp151 * _tmp152) -
                         _tmp101 * _tmp143 * (_tmp141 * _tmp148 + _tmp150 * _tmp151 + Scalar(1.0)) -
                         _tmp122 * _tmp154;
  const Scalar _tmp156 = Scalar(1.0) / (_tmp155);
  const Scalar _tmp157 = std::asinh(_tmp147 * _tmp156);
  const Scalar _tmp158 = Scalar(9.6622558468725703) * _tmp155;
  const Scalar _tmp159 =
      -_tmp157 * _tmp158 -
      Scalar(8.3196563700000006) *
          std::sqrt(
              Scalar(std::pow(Scalar(-Scalar(0.12019727204189803) * _tmp73 - 1), Scalar(2)) +
                     Scalar(0.057067943376852184) *
                         std::pow(Scalar(-Scalar(0.50315118556004401) * _tmp75 - 1), Scalar(2))));
  const Scalar _tmp160 = Scalar(0.1034955) * _tmp156;
  const Scalar _tmp161 = _tmp159 * _tmp160;
  const Scalar _tmp162 = Scalar(1.0) * _tmp157;
  const Scalar _tmp163 = Scalar(9.6622558468725703) * _tmp154;
  const Scalar _tmp164 = std::pow(_tmp155, Scalar(-2));
  const Scalar _tmp165 = _tmp154 * _tmp164;
  const Scalar _tmp166 = _tmp23 + _tmp51;
  const Scalar _tmp167 =
      (-_tmp147 * _tmp165 + _tmp156 * (_tmp121 * _tmp166 - _tmp129 + _tmp146 * _tmp41)) /
      std::sqrt(Scalar(std::pow(_tmp147, Scalar(2)) * _tmp164 + 1));
  const Scalar _tmp168 = _tmp119 * _tmp55;
  const Scalar _tmp169 = _tmp144 * _tmp61;
  const Scalar _tmp170 = _tmp109 * _tmp116 * _tmp55 + _tmp118 * _tmp168 + _tmp122 * _tmp128 +
                         _tmp136 * _tmp137 * _tmp55 + _tmp142 * _tmp143 * _tmp55 -
                         _tmp145 * _tmp169;
  const Scalar _tmp171 = _tmp153 * _tmp88;
  const Scalar _tmp172 = _tmp108 * _tmp116;
  const Scalar _tmp173 = -_tmp122 * _tmp171 + _tmp137 * _tmp152 * _tmp88 +
                         _tmp143 * _tmp150 * _tmp88 - _tmp172 * _tmp80 * _tmp88;
  const Scalar _tmp174 = Scalar(1.0) / (_tmp173);
  const Scalar _tmp175 = std::asinh(_tmp170 * _tmp174);
  const Scalar _tmp176 = Scalar(1.0) * _tmp175;
  const Scalar _tmp177 = std::pow(_tmp173, Scalar(-2));
  const Scalar _tmp178 = _tmp171 * _tmp177;
  const Scalar _tmp179 = _tmp41 * _tmp61;
  const Scalar _tmp180 =
      (-_tmp170 * _tmp178 + _tmp174 * (-_tmp128 - _tmp145 * _tmp179 + _tmp166 * _tmp168)) /
      std::sqrt(Scalar(std::pow(_tmp170, Scalar(2)) * _tmp177 + 1));
  const Scalar _tmp181 = Scalar(9.6622558468725703) * _tmp173;
  const Scalar _tmp182 =
      -_tmp175 * _tmp181 -
      Scalar(4.7752063900000001) *
          std::sqrt(
              Scalar(std::pow(Scalar(1 - Scalar(0.20941503221602112) * _tmp81), Scalar(2)) +
                     Scalar(0.32397683292140877) *
                         std::pow(Scalar(1 - Scalar(0.36791786395571047) * _tmp83), Scalar(2))));
  const Scalar _tmp183 = Scalar(0.1034955) * _tmp174;
  const Scalar _tmp184 = _tmp182 * _tmp183;
  const Scalar _tmp185 = Scalar(9.6622558468725703) * _tmp171;
  const Scalar _tmp186 = -_tmp107 * _tmp116 - _tmp118 * _tmp120 + _tmp122 * _tmp124 -
                         _tmp132 * _tmp137 * _tmp60 - _tmp140 * _tmp143 * _tmp60 + _tmp169;
  const Scalar _tmp187 = _tmp125 * fh1;
  const Scalar _tmp188 =
      _tmp114 * _tmp133 * _tmp187 + _tmp115 * _tmp141 * _tmp187 + _tmp122 * _tmp127 + _tmp172;
  const Scalar _tmp189 = Scalar(1.0) / (_tmp188);
  const Scalar _tmp190 = std::asinh(_tmp186 * _tmp189);
  const Scalar _tmp191 = Scalar(1.0) * _tmp190;
  const Scalar _tmp192 = Scalar(9.6622558468725703) * _tmp188;
  const Scalar _tmp193 =
      -_tmp190 * _tmp192 -
      Scalar(8.3888750099999996) *
          std::sqrt(
              Scalar(Scalar(0.090254729040973036) *
                         std::pow(Scalar(1 - Scalar(0.39679052492160538) * _tmp67), Scalar(2)) +
                     std::pow(Scalar(-Scalar(0.11920549523123722) * _tmp69 - 1), Scalar(2))));
  const Scalar _tmp194 = Scalar(0.1034955) * _tmp189;
  const Scalar _tmp195 = _tmp193 * _tmp194;
  const Scalar _tmp196 = Scalar(9.6622558468725703) * _tmp127;
  const Scalar _tmp197 = std::pow(_tmp188, Scalar(-2));
  const Scalar _tmp198 = _tmp127 * _tmp197;
  const Scalar _tmp199 = (_tmp186 * _tmp198 + _tmp189 * (-_tmp120 * _tmp166 - _tmp124 + _tmp179)) /
                         std::sqrt(Scalar(std::pow(_tmp186, Scalar(2)) * _tmp197 + 1));

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
                            std::pow(Scalar(1 - Scalar(0.20689664689659551) * _tmp31), Scalar(2)) +
                            Scalar(0.13817235445745474) *
                                std::pow(Scalar(-Scalar(0.55659957866191134) * _tmp42 - 1),
                                         Scalar(2)))))));
  _res(1, 0) = _tmp158 * (-Scalar(1.0) * _tmp167 * std::cosh(_tmp162) -
                          (-Scalar(0.1034955) * _tmp159 * _tmp165 +
                           _tmp160 * (-_tmp157 * _tmp163 - _tmp158 * _tmp167)) *
                              std::cosh(_tmp161)) +
               _tmp163 * (-std::sinh(_tmp161) - std::sinh(_tmp162));
  _res(2, 0) = _tmp181 * (-Scalar(1.0) * _tmp180 * std::cosh(_tmp176) -
                          (-Scalar(0.1034955) * _tmp178 * _tmp182 +
                           _tmp183 * (-_tmp175 * _tmp185 - _tmp180 * _tmp181)) *
                              std::cosh(_tmp184)) +
               _tmp185 * (-std::sinh(_tmp176) - std::sinh(_tmp184));
  _res(3, 0) = _tmp192 * (-Scalar(1.0) * _tmp199 * std::cosh(_tmp191) -
                          (Scalar(0.1034955) * _tmp193 * _tmp198 +
                           _tmp194 * (_tmp190 * _tmp196 - _tmp192 * _tmp199)) *
                              std::cosh(_tmp195)) -
               _tmp196 * (-std::sinh(_tmp191) - std::sinh(_tmp195));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym