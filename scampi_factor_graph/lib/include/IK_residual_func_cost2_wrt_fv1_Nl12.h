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
 * Symbolic function: IK_residual_func_cost2_wrt_fv1_Nl12
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost2WrtFv1Nl12(
    const Scalar fh1, const Scalar fv1, const Scalar rx, const Scalar ry, const Scalar rz,
    const Scalar p_init0, const Scalar p_init1, const Scalar p_init2, const Scalar rot_init_x,
    const Scalar rot_init_y, const Scalar rot_init_z, const Scalar rot_init_w,
    const Scalar epsilon) {
  // Total ops: 613

  // Unused inputs
  (void)p_init2;
  (void)epsilon;

  // Input arrays

  // Intermediate terms (202)
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
  const Scalar _tmp10 = _tmp7 * rot_init_y;
  const Scalar _tmp11 = _tmp10 * rz + _tmp6 * rot_init_x - _tmp8 * rot_init_z + _tmp9 * rot_init_w;
  const Scalar _tmp12 = -2 * std::pow(_tmp11, Scalar(2));
  const Scalar _tmp13 = _tmp7 * rz;
  const Scalar _tmp14 =
      -_tmp10 * rx + _tmp13 * rot_init_w + _tmp6 * rot_init_z + _tmp8 * rot_init_x;
  const Scalar _tmp15 = -2 * std::pow(_tmp14, Scalar(2));
  const Scalar _tmp16 = Scalar(0.20999999999999999) * _tmp12 +
                        Scalar(0.20999999999999999) * _tmp15 + Scalar(0.20999999999999999);
  const Scalar _tmp17 =
      -_tmp13 * rot_init_x + _tmp6 * rot_init_y + _tmp8 * rot_init_w + _tmp9 * rot_init_z;
  const Scalar _tmp18 = 2 * _tmp14 * _tmp17;
  const Scalar _tmp19 =
      -2 * _tmp10 * ry - 2 * _tmp13 * rot_init_z + 2 * _tmp6 * rot_init_w - 2 * _tmp9 * rot_init_x;
  const Scalar _tmp20 = _tmp11 * _tmp19;
  const Scalar _tmp21 = _tmp18 - _tmp20;
  const Scalar _tmp22 = Scalar(0.010999999999999999) * _tmp21;
  const Scalar _tmp23 = -_tmp22;
  const Scalar _tmp24 = 2 * _tmp11;
  const Scalar _tmp25 = _tmp17 * _tmp24;
  const Scalar _tmp26 = _tmp14 * _tmp19;
  const Scalar _tmp27 = Scalar(0.20999999999999999) * _tmp25 + Scalar(0.20999999999999999) * _tmp26;
  const Scalar _tmp28 = _tmp23 + _tmp27;
  const Scalar _tmp29 = _tmp16 + _tmp28;
  const Scalar _tmp30 = _tmp29 + p_init1;
  const Scalar _tmp31 = 1 - 2 * std::pow(_tmp17, Scalar(2));
  const Scalar _tmp32 = Scalar(0.20999999999999999) * _tmp15 + Scalar(0.20999999999999999) * _tmp31;
  const Scalar _tmp33 = Scalar(0.20999999999999999) * _tmp25 - Scalar(0.20999999999999999) * _tmp26;
  const Scalar _tmp34 = _tmp14 * _tmp24;
  const Scalar _tmp35 = _tmp17 * _tmp19;
  const Scalar _tmp36 = _tmp34 + _tmp35;
  const Scalar _tmp37 = -Scalar(0.010999999999999999) * _tmp36;
  const Scalar _tmp38 = _tmp33 + _tmp37;
  const Scalar _tmp39 = _tmp32 + _tmp38;
  const Scalar _tmp40 = _tmp39 + p_init0;
  const Scalar _tmp41 = -_tmp16;
  const Scalar _tmp42 = -_tmp27;
  const Scalar _tmp43 = _tmp41 + _tmp42;
  const Scalar _tmp44 = _tmp23 + _tmp43;
  const Scalar _tmp45 = _tmp44 + p_init1;
  const Scalar _tmp46 = -_tmp32;
  const Scalar _tmp47 = -_tmp33 + _tmp37;
  const Scalar _tmp48 = _tmp46 + _tmp47;
  const Scalar _tmp49 = _tmp48 + p_init0;
  const Scalar _tmp50 = _tmp30 + Scalar(-4.7752063900000001);
  const Scalar _tmp51 = _tmp40 + Scalar(-2.71799795);
  const Scalar _tmp52 = std::pow(Scalar(std::pow(_tmp50, Scalar(2)) + std::pow(_tmp51, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp53 = _tmp50 * _tmp52;
  const Scalar _tmp54 = Scalar(0.20999999999999999) * _tmp34 - Scalar(0.20999999999999999) * _tmp35;
  const Scalar _tmp55 =
      -Scalar(0.010999999999999999) * _tmp12 - Scalar(0.010999999999999999) * _tmp31;
  const Scalar _tmp56 = Scalar(0.20999999999999999) * _tmp18 + Scalar(0.20999999999999999) * _tmp20;
  const Scalar _tmp57 = _tmp55 + _tmp56;
  const Scalar _tmp58 = fh1 * (_tmp54 + _tmp57);
  const Scalar _tmp59 = -Scalar(3.29616) * _tmp21 - _tmp29 * fv1 - _tmp53 * _tmp58;
  const Scalar _tmp60 = _tmp38 + _tmp46;
  const Scalar _tmp61 = Scalar(1.0) * _tmp44;
  const Scalar _tmp62 = -_tmp61;
  const Scalar _tmp63 = _tmp16 + _tmp23 + _tmp42;
  const Scalar _tmp64 = _tmp62 + _tmp63;
  const Scalar _tmp65 = _tmp28 + _tmp41;
  const Scalar _tmp66 = Scalar(1.0) / (_tmp62 + _tmp65);
  const Scalar _tmp67 = _tmp32 + _tmp47;
  const Scalar _tmp68 = Scalar(1.0) * _tmp48;
  const Scalar _tmp69 = _tmp66 * (-_tmp67 + _tmp68);
  const Scalar _tmp70 = _tmp64 * _tmp69;
  const Scalar _tmp71 = Scalar(1.0) / (-_tmp60 + _tmp68 - _tmp70);
  const Scalar _tmp72 = Scalar(1.0) * _tmp71;
  const Scalar _tmp73 = _tmp69 * _tmp72;
  const Scalar _tmp74 = _tmp70 * _tmp72 + Scalar(1.0);
  const Scalar _tmp75 = Scalar(1.0) * _tmp66;
  const Scalar _tmp76 = Scalar(1.0) * _tmp73 - Scalar(1.0) * _tmp74 * _tmp75;
  const Scalar _tmp77 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp78 = _tmp61 * _tmp69 + _tmp68;
  const Scalar _tmp79 = _tmp71 * _tmp78;
  const Scalar _tmp80 = _tmp45 + Scalar(8.3196563700000006);
  const Scalar _tmp81 = _tmp49 + Scalar(1.9874742000000001);
  const Scalar _tmp82 = Scalar(1.0) / (_tmp81);
  const Scalar _tmp83 = _tmp80 * _tmp82;
  const Scalar _tmp84 = -_tmp54;
  const Scalar _tmp85 = _tmp55 - _tmp56;
  const Scalar _tmp86 = _tmp84 + _tmp85;
  const Scalar _tmp87 = _tmp63 + p_init1;
  const Scalar _tmp88 = _tmp87 + Scalar(-4.8333311099999996);
  const Scalar _tmp89 = _tmp60 + p_init0;
  const Scalar _tmp90 = _tmp89 + Scalar(1.79662371);
  const Scalar _tmp91 = std::pow(Scalar(std::pow(_tmp88, Scalar(2)) + std::pow(_tmp90, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp92 = _tmp90 * _tmp91;
  const Scalar _tmp93 = _tmp86 * _tmp92;
  const Scalar _tmp94 = _tmp57 + _tmp84;
  const Scalar _tmp95 = _tmp88 * _tmp91;
  const Scalar _tmp96 = _tmp54 + _tmp85;
  const Scalar _tmp97 = _tmp67 + p_init0;
  const Scalar _tmp98 = _tmp97 + Scalar(-2.5202214700000001);
  const Scalar _tmp99 = _tmp65 + p_init1;
  const Scalar _tmp100 = _tmp99 + Scalar(8.3888750099999996);
  const Scalar _tmp101 =
      std::pow(Scalar(std::pow(_tmp100, Scalar(2)) + std::pow(_tmp98, Scalar(2))),
               Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp102 = _tmp100 * _tmp101;
  const Scalar _tmp103 = _tmp101 * _tmp98;
  const Scalar _tmp104 = _tmp103 * _tmp86;
  const Scalar _tmp105 = _tmp102 * _tmp96 - _tmp104 * _tmp83;
  const Scalar _tmp106 = Scalar(1.0) / (-_tmp102 + _tmp103 * _tmp83);
  const Scalar _tmp107 = _tmp83 * _tmp92 - _tmp95;
  const Scalar _tmp108 = _tmp106 * _tmp107;
  const Scalar _tmp109 = -_tmp105 * _tmp108 - _tmp83 * _tmp93 + _tmp94 * _tmp95;
  const Scalar _tmp110 = -_tmp103 * _tmp96 + _tmp104;
  const Scalar _tmp111 = -_tmp108 * _tmp110 - _tmp109 * _tmp69 - _tmp92 * _tmp94 + _tmp93;
  const Scalar _tmp112 = Scalar(1.0) / (_tmp111);
  const Scalar _tmp113 = 0;
  const Scalar _tmp114 = _tmp66 * (-_tmp109 * _tmp113 + _tmp62 - _tmp64 * _tmp79);
  const Scalar _tmp115 = -Scalar(1.0) * _tmp114 - Scalar(1.0) * _tmp72 * _tmp78 + Scalar(1.0);
  const Scalar _tmp116 =
      std::sqrt(Scalar(std::pow(_tmp80, Scalar(2)) + std::pow(_tmp81, Scalar(2))));
  const Scalar _tmp117 = Scalar(1.0) / (_tmp116);
  const Scalar _tmp118 = _tmp116 * _tmp82;
  const Scalar _tmp119 = _tmp118 * (-_tmp117 * _tmp44 * _tmp81 + _tmp117 * _tmp48 * _tmp80);
  const Scalar _tmp120 = -_tmp102 * _tmp67 + _tmp103 * _tmp119 + _tmp103 * _tmp65;
  const Scalar _tmp121 = Scalar(1.0) * _tmp106;
  const Scalar _tmp122 = _tmp105 * _tmp121;
  const Scalar _tmp123 = -_tmp110 * _tmp121 + _tmp122 * _tmp69;
  const Scalar _tmp124 = -_tmp108 * _tmp120 + _tmp119 * _tmp92 - _tmp60 * _tmp95 + _tmp63 * _tmp92;
  const Scalar _tmp125 = _tmp112 * _tmp124;
  const Scalar _tmp126 = Scalar(1.0) / (_tmp124);
  const Scalar _tmp127 = _tmp111 * _tmp126;
  const Scalar _tmp128 = _tmp127 * (-_tmp120 * _tmp121 - _tmp123 * _tmp125);
  const Scalar _tmp129 = _tmp64 * _tmp71;
  const Scalar _tmp130 = _tmp123 + _tmp128;
  const Scalar _tmp131 = _tmp109 * _tmp112;
  const Scalar _tmp132 = -_tmp122 + _tmp128 * _tmp129 - _tmp130 * _tmp131;
  const Scalar _tmp133 = _tmp53 * fh1;
  const Scalar _tmp134 = _tmp127 * _tmp72;
  const Scalar _tmp135 = Scalar(1.0) * _tmp126;
  const Scalar _tmp136 = -_tmp109 * _tmp135 + _tmp134 * _tmp64;
  const Scalar _tmp137 = _tmp51 * _tmp52;
  const Scalar _tmp138 = fh1 * (-_tmp137 * _tmp29 + _tmp39 * _tmp53);
  const Scalar _tmp139 = _tmp106 * _tmp83;
  const Scalar _tmp140 = _tmp105 * _tmp139 + _tmp83 * _tmp86;
  const Scalar _tmp141 = _tmp110 * _tmp139 - _tmp140 * _tmp69 - _tmp86;
  const Scalar _tmp142 = _tmp127 * (-_tmp119 + _tmp120 * _tmp139 - _tmp125 * _tmp141);
  const Scalar _tmp143 = _tmp141 + _tmp142;
  const Scalar _tmp144 = _tmp129 * _tmp142 - _tmp131 * _tmp143 + _tmp140;
  const Scalar _tmp145 = _tmp137 * fh1;
  const Scalar _tmp146 = _tmp137 * _tmp58 + Scalar(3.29616) * _tmp36 + _tmp39 * fv1;
  const Scalar _tmp147 = _tmp64 * _tmp66;
  const Scalar _tmp148 = Scalar(1.0) * _tmp147 * _tmp72 - Scalar(1.0) * _tmp72;
  const Scalar _tmp149 = _tmp115 * _tmp77 +
                         Scalar(1.0) * _tmp133 * (_tmp128 * _tmp72 - _tmp132 * _tmp75) +
                         Scalar(1.0) * _tmp138 * (_tmp134 - _tmp136 * _tmp75) +
                         Scalar(1.0) * _tmp145 * (_tmp142 * _tmp72 - _tmp144 * _tmp75) +
                         _tmp146 * _tmp148 + _tmp59 * _tmp76;
  const Scalar _tmp150 = _tmp112 * _tmp92;
  const Scalar _tmp151 = _tmp107 * _tmp112;
  const Scalar _tmp152 = -_tmp130 * _tmp151 + Scalar(1.0);
  const Scalar _tmp153 = _tmp103 * _tmp106;
  const Scalar _tmp154 = _tmp108 * _tmp113;
  const Scalar _tmp155 = _tmp118 * (-_tmp103 * _tmp154 + _tmp113 * _tmp92);
  const Scalar _tmp156 = -_tmp143 * _tmp151 - _tmp83;
  const Scalar _tmp157 = -_tmp118 * _tmp133 * (_tmp130 * _tmp150 + _tmp152 * _tmp153) -
                         _tmp118 * _tmp138 * (-_tmp103 * _tmp108 * _tmp135 + _tmp135 * _tmp92) -
                         _tmp118 * _tmp145 * (_tmp143 * _tmp150 + _tmp153 * _tmp156 + Scalar(1.0)) -
                         _tmp155 * _tmp77;
  const Scalar _tmp158 = Scalar(1.0) / (_tmp157);
  const Scalar _tmp159 = std::asinh(_tmp149 * _tmp158);
  const Scalar _tmp160 = Scalar(9.6622558468725703) * _tmp157;
  const Scalar _tmp161 =
      -_tmp159 * _tmp160 -
      Scalar(8.3196563700000006) *
          std::sqrt(
              Scalar(std::pow(Scalar(-Scalar(0.12019727204189803) * _tmp45 - 1), Scalar(2)) +
                     Scalar(0.057067943376852184) *
                         std::pow(Scalar(-Scalar(0.50315118556004401) * _tmp49 - 1), Scalar(2))));
  const Scalar _tmp162 = Scalar(0.1034955) * _tmp158;
  const Scalar _tmp163 = _tmp161 * _tmp162;
  const Scalar _tmp164 = Scalar(1.0) * _tmp159;
  const Scalar _tmp165 = Scalar(9.6622558468725703) * _tmp155;
  const Scalar _tmp166 = std::pow(_tmp157, Scalar(-2));
  const Scalar _tmp167 = _tmp155 * _tmp166;
  const Scalar _tmp168 = _tmp22 + _tmp43;
  const Scalar _tmp169 =
      (-_tmp149 * _tmp167 + _tmp158 * (-_tmp115 + _tmp148 * _tmp39 + _tmp168 * _tmp76)) /
      std::sqrt(Scalar(std::pow(_tmp149, Scalar(2)) * _tmp166 + 1));
  const Scalar _tmp170 = _tmp66 * fh1;
  const Scalar _tmp171 = _tmp66 * _tmp74;
  const Scalar _tmp172 = _tmp146 * _tmp72;
  const Scalar _tmp173 = _tmp114 * _tmp77 + _tmp132 * _tmp170 * _tmp53 +
                         _tmp136 * _tmp138 * _tmp66 + _tmp137 * _tmp144 * _tmp170 -
                         _tmp147 * _tmp172 + _tmp171 * _tmp59;
  const Scalar _tmp174 = _tmp113 * _tmp77;
  const Scalar _tmp175 = _tmp135 * _tmp138;
  const Scalar _tmp176 = _tmp106 * _tmp133 * _tmp152 + _tmp106 * _tmp145 * _tmp156 -
                         _tmp108 * _tmp174 - _tmp108 * _tmp175;
  const Scalar _tmp177 = Scalar(1.0) / (_tmp176);
  const Scalar _tmp178 = std::asinh(_tmp173 * _tmp177);
  const Scalar _tmp179 = Scalar(1.0) * _tmp178;
  const Scalar _tmp180 = _tmp39 * _tmp72;
  const Scalar _tmp181 = std::pow(_tmp176, Scalar(-2));
  const Scalar _tmp182 = _tmp154 * _tmp181;
  const Scalar _tmp183 =
      (-_tmp173 * _tmp182 + _tmp177 * (-_tmp114 - _tmp147 * _tmp180 + _tmp168 * _tmp171)) /
      std::sqrt(Scalar(std::pow(_tmp173, Scalar(2)) * _tmp181 + 1));
  const Scalar _tmp184 = Scalar(9.6622558468725703) * _tmp176;
  const Scalar _tmp185 =
      -_tmp178 * _tmp184 -
      Scalar(8.3888750099999996) *
          std::sqrt(
              Scalar(Scalar(0.090254729040973036) *
                         std::pow(Scalar(1 - Scalar(0.39679052492160538) * _tmp97), Scalar(2)) +
                     std::pow(Scalar(-Scalar(0.11920549523123722) * _tmp99 - 1), Scalar(2))));
  const Scalar _tmp186 = Scalar(0.1034955) * _tmp177;
  const Scalar _tmp187 = _tmp185 * _tmp186;
  const Scalar _tmp188 = Scalar(9.6622558468725703) * _tmp113;
  const Scalar _tmp189 = _tmp108 * _tmp188;
  const Scalar _tmp190 = -_tmp128 * _tmp133 * _tmp71 - _tmp134 * _tmp138 -
                         _tmp142 * _tmp145 * _tmp71 + _tmp172 - _tmp59 * _tmp73 + _tmp77 * _tmp79;
  const Scalar _tmp191 =
      _tmp112 * _tmp130 * _tmp133 + _tmp112 * _tmp143 * _tmp145 + _tmp174 + _tmp175;
  const Scalar _tmp192 = Scalar(1.0) / (_tmp191);
  const Scalar _tmp193 = std::asinh(_tmp190 * _tmp192);
  const Scalar _tmp194 = Scalar(1.0) * _tmp193;
  const Scalar _tmp195 = Scalar(9.6622558468725703) * _tmp191;
  const Scalar _tmp196 =
      -_tmp193 * _tmp195 -
      Scalar(4.8333311099999996) *
          std::sqrt(
              Scalar(std::pow(Scalar(1 - Scalar(0.20689664689659551) * _tmp87), Scalar(2)) +
                     Scalar(0.13817235445745474) *
                         std::pow(Scalar(-Scalar(0.55659957866191134) * _tmp89 - 1), Scalar(2))));
  const Scalar _tmp197 = Scalar(0.1034955) * _tmp192;
  const Scalar _tmp198 = _tmp196 * _tmp197;
  const Scalar _tmp199 = std::pow(_tmp191, Scalar(-2));
  const Scalar _tmp200 = _tmp113 * _tmp199;
  const Scalar _tmp201 = (_tmp190 * _tmp200 + _tmp192 * (-_tmp168 * _tmp73 + _tmp180 - _tmp79)) /
                         std::sqrt(Scalar(std::pow(_tmp190, Scalar(2)) * _tmp199 + 1));

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
  _res(1, 0) = _tmp160 * (-Scalar(1.0) * _tmp169 * std::cosh(_tmp164) -
                          (-Scalar(0.1034955) * _tmp161 * _tmp167 +
                           _tmp162 * (-_tmp159 * _tmp165 - _tmp160 * _tmp169)) *
                              std::cosh(_tmp163)) +
               _tmp165 * (-std::sinh(_tmp163) - std::sinh(_tmp164));
  _res(2, 0) = _tmp184 * (-Scalar(1.0) * _tmp183 * std::cosh(_tmp179) -
                          (-Scalar(0.1034955) * _tmp182 * _tmp185 +
                           _tmp186 * (-_tmp178 * _tmp189 - _tmp183 * _tmp184)) *
                              std::cosh(_tmp187)) +
               _tmp189 * (-std::sinh(_tmp179) - std::sinh(_tmp187));
  _res(3, 0) = -_tmp188 * (-std::sinh(_tmp194) - std::sinh(_tmp198)) +
               _tmp195 * (-Scalar(1.0) * _tmp201 * std::cosh(_tmp194) -
                          (Scalar(0.1034955) * _tmp196 * _tmp200 +
                           _tmp197 * (_tmp188 * _tmp193 - _tmp195 * _tmp201)) *
                              std::cosh(_tmp198));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym