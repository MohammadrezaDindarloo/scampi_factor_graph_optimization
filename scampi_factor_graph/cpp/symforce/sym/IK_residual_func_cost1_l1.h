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
 * Symbolic function: IK_residual_func_cost1_l1
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost1L1(const Scalar fh1, const Scalar fv1,
                                                  const Scalar rx, const Scalar ry, const Scalar rz,
                                                  const Scalar p_init0, const Scalar p_init1,
                                                  const Scalar p_init2, const Scalar rot_init_x,
                                                  const Scalar rot_init_y, const Scalar rot_init_z,
                                                  const Scalar rot_init_w, const Scalar epsilon) {
  // Total ops: 520

  // Unused inputs
  (void)epsilon;

  // Input arrays

  // Intermediate terms (165)
  const Scalar _tmp0 = Scalar(1.0) / (fh1);
  const Scalar _tmp1 = std::asinh(_tmp0 * fv1);
  const Scalar _tmp2 = std::sqrt(
      Scalar(std::pow(rx, Scalar(2)) + std::pow(ry, Scalar(2)) + std::pow(rz, Scalar(2))));
  const Scalar _tmp3 = (Scalar(1) / Scalar(2)) * _tmp2;
  const Scalar _tmp4 = std::cos(_tmp3);
  const Scalar _tmp5 = std::sin(_tmp3) / _tmp2;
  const Scalar _tmp6 = _tmp5 * ry;
  const Scalar _tmp7 = _tmp5 * rx;
  const Scalar _tmp8 = _tmp5 * rz;
  const Scalar _tmp9 =
      _tmp4 * rot_init_x - _tmp6 * rot_init_z + _tmp7 * rot_init_w + _tmp8 * rot_init_y;
  const Scalar _tmp10 = -2 * std::pow(_tmp9, Scalar(2));
  const Scalar _tmp11 =
      _tmp4 * rot_init_z + _tmp6 * rot_init_x - _tmp7 * rot_init_y + _tmp8 * rot_init_w;
  const Scalar _tmp12 = -2 * std::pow(_tmp11, Scalar(2));
  const Scalar _tmp13 = Scalar(0.20999999999999999) * _tmp10 +
                        Scalar(0.20999999999999999) * _tmp12 + Scalar(0.20999999999999999);
  const Scalar _tmp14 = -_tmp13;
  const Scalar _tmp15 = _tmp5 * rot_init_z;
  const Scalar _tmp16 = _tmp15 * rx + _tmp4 * rot_init_y + _tmp6 * rot_init_w - _tmp8 * rot_init_x;
  const Scalar _tmp17 = 2 * _tmp16;
  const Scalar _tmp18 = _tmp11 * _tmp17;
  const Scalar _tmp19 =
      -2 * _tmp15 * rz + 2 * _tmp4 * rot_init_w - 2 * _tmp6 * rot_init_y - 2 * _tmp7 * rot_init_x;
  const Scalar _tmp20 = _tmp19 * _tmp9;
  const Scalar _tmp21 = _tmp18 - _tmp20;
  const Scalar _tmp22 = -Scalar(0.010999999999999999) * _tmp21;
  const Scalar _tmp23 = _tmp17 * _tmp9;
  const Scalar _tmp24 = _tmp11 * _tmp19;
  const Scalar _tmp25 = Scalar(0.20999999999999999) * _tmp23 + Scalar(0.20999999999999999) * _tmp24;
  const Scalar _tmp26 = _tmp22 - _tmp25;
  const Scalar _tmp27 = _tmp14 + _tmp26;
  const Scalar _tmp28 = _tmp27 + p_init1;
  const Scalar _tmp29 = Scalar(0.20999999999999999) * _tmp23 - Scalar(0.20999999999999999) * _tmp24;
  const Scalar _tmp30 = -_tmp29;
  const Scalar _tmp31 = 2 * _tmp11 * _tmp9;
  const Scalar _tmp32 = _tmp16 * _tmp19;
  const Scalar _tmp33 = _tmp31 + _tmp32;
  const Scalar _tmp34 = -Scalar(0.010999999999999999) * _tmp33;
  const Scalar _tmp35 = 1 - 2 * std::pow(_tmp16, Scalar(2));
  const Scalar _tmp36 = Scalar(0.20999999999999999) * _tmp12 + Scalar(0.20999999999999999) * _tmp35;
  const Scalar _tmp37 = _tmp34 - _tmp36;
  const Scalar _tmp38 = _tmp30 + _tmp37;
  const Scalar _tmp39 = _tmp38 + p_init0;
  const Scalar _tmp40 = Scalar(9.6622558468725703) * fh1;
  const Scalar _tmp41 = Scalar(0.20999999999999999) * _tmp18 + Scalar(0.20999999999999999) * _tmp20;
  const Scalar _tmp42 = -_tmp41;
  const Scalar _tmp43 =
      -Scalar(0.010999999999999999) * _tmp10 - Scalar(0.010999999999999999) * _tmp35;
  const Scalar _tmp44 = Scalar(0.20999999999999999) * _tmp31 - Scalar(0.20999999999999999) * _tmp32;
  const Scalar _tmp45 = _tmp43 - _tmp44;
  const Scalar _tmp46 = _tmp42 + _tmp45;
  const Scalar _tmp47 = _tmp22 + _tmp25;
  const Scalar _tmp48 = _tmp13 + _tmp47;
  const Scalar _tmp49 = _tmp48 + p_init1;
  const Scalar _tmp50 = _tmp49 + Scalar(-4.7752063900000001);
  const Scalar _tmp51 = _tmp34 + _tmp36;
  const Scalar _tmp52 = _tmp29 + _tmp51;
  const Scalar _tmp53 = _tmp52 + p_init0;
  const Scalar _tmp54 = _tmp53 + Scalar(-2.71799795);
  const Scalar _tmp55 = std::pow(Scalar(std::pow(_tmp50, Scalar(2)) + std::pow(_tmp54, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp56 = _tmp54 * _tmp55;
  const Scalar _tmp57 = _tmp50 * _tmp55;
  const Scalar _tmp58 = _tmp13 + _tmp26;
  const Scalar _tmp59 = _tmp58 + p_init1;
  const Scalar _tmp60 = _tmp59 + Scalar(-4.8333311099999996);
  const Scalar _tmp61 = _tmp29 + _tmp37;
  const Scalar _tmp62 = _tmp61 + p_init0;
  const Scalar _tmp63 = _tmp62 + Scalar(1.79662371);
  const Scalar _tmp64 =
      std::sqrt(Scalar(std::pow(_tmp60, Scalar(2)) + std::pow(_tmp63, Scalar(2))));
  const Scalar _tmp65 = Scalar(1.0) / (_tmp64);
  const Scalar _tmp66 = Scalar(1.0) / (_tmp63);
  const Scalar _tmp67 = _tmp64 * _tmp66;
  const Scalar _tmp68 = _tmp67 * (-_tmp58 * _tmp63 * _tmp65 + _tmp60 * _tmp61 * _tmp65);
  const Scalar _tmp69 = _tmp48 * _tmp56 - _tmp52 * _tmp57 + _tmp56 * _tmp68;
  const Scalar _tmp70 = _tmp60 * _tmp66;
  const Scalar _tmp71 = Scalar(1.0) / (_tmp56 * _tmp70 - _tmp57);
  const Scalar _tmp72 = _tmp30 + _tmp51;
  const Scalar _tmp73 = _tmp72 + p_init0;
  const Scalar _tmp74 = _tmp73 + Scalar(-2.5202214700000001);
  const Scalar _tmp75 = _tmp14 + _tmp47;
  const Scalar _tmp76 = _tmp75 + p_init1;
  const Scalar _tmp77 = _tmp76 + Scalar(8.3888750099999996);
  const Scalar _tmp78 = std::pow(Scalar(std::pow(_tmp74, Scalar(2)) + std::pow(_tmp77, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp79 = _tmp77 * _tmp78;
  const Scalar _tmp80 = _tmp74 * _tmp78;
  const Scalar _tmp81 = _tmp70 * _tmp80 - _tmp79;
  const Scalar _tmp82 = _tmp71 * _tmp81;
  const Scalar _tmp83 = _tmp68 * _tmp80 - _tmp69 * _tmp82 - _tmp72 * _tmp79 + _tmp75 * _tmp80;
  const Scalar _tmp84 = Scalar(1.0) / (_tmp83);
  const Scalar _tmp85 = Scalar(1.0) * _tmp71;
  const Scalar _tmp86 = _tmp81 * _tmp84 * _tmp85;
  const Scalar _tmp87 = Scalar(1.0) * _tmp84;
  const Scalar _tmp88 = _tmp28 + Scalar(8.3196563700000006);
  const Scalar _tmp89 = _tmp39 + Scalar(1.9874742000000001);
  const Scalar _tmp90 = std::pow(Scalar(std::pow(_tmp88, Scalar(2)) + std::pow(_tmp89, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp91 = _tmp89 * _tmp90;
  const Scalar _tmp92 = _tmp88 * _tmp90;
  const Scalar _tmp93 = fh1 * (-_tmp27 * _tmp91 + _tmp38 * _tmp92);
  const Scalar _tmp94 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp95 = _tmp43 + _tmp44;
  const Scalar _tmp96 = _tmp42 + _tmp95;
  const Scalar _tmp97 = _tmp41 + _tmp45;
  const Scalar _tmp98 = _tmp80 * _tmp97;
  const Scalar _tmp99 = _tmp70 * _tmp97;
  const Scalar _tmp100 = _tmp41 + _tmp95;
  const Scalar _tmp101 = _tmp100 * _tmp57 - _tmp56 * _tmp99;
  const Scalar _tmp102 = -_tmp101 * _tmp82 - _tmp70 * _tmp98 + _tmp79 * _tmp96;
  const Scalar _tmp103 = Scalar(1.0) * _tmp58;
  const Scalar _tmp104 = -_tmp103;
  const Scalar _tmp105 = Scalar(1.0) / (_tmp104 + _tmp48);
  const Scalar _tmp106 = Scalar(1.0) * _tmp61;
  const Scalar _tmp107 = _tmp106 - _tmp52;
  const Scalar _tmp108 = _tmp105 * _tmp107;
  const Scalar _tmp109 = -_tmp100 * _tmp56 + _tmp56 * _tmp97;
  const Scalar _tmp110 = -_tmp102 * _tmp108 - _tmp109 * _tmp82 - _tmp80 * _tmp96 + _tmp98;
  const Scalar _tmp111 = Scalar(1.0) / (_tmp110);
  const Scalar _tmp112 = _tmp103 * _tmp108 + _tmp106;
  const Scalar _tmp113 = 0;
  const Scalar _tmp114 = _tmp111 * _tmp113;
  const Scalar _tmp115 = Scalar(1.0) * _tmp105;
  const Scalar _tmp116 = _tmp101 * _tmp107 * _tmp115 * _tmp71 - _tmp109 * _tmp85;
  const Scalar _tmp117 = _tmp111 * _tmp83;
  const Scalar _tmp118 = _tmp110 * _tmp84;
  const Scalar _tmp119 = _tmp118 * (-_tmp116 * _tmp117 - _tmp69 * _tmp85);
  const Scalar _tmp120 = _tmp116 + _tmp119;
  const Scalar _tmp121 = _tmp111 * _tmp81;
  const Scalar _tmp122 = -_tmp120 * _tmp121 + Scalar(1.0);
  const Scalar _tmp123 = _tmp56 * _tmp71;
  const Scalar _tmp124 = _tmp111 * _tmp80;
  const Scalar _tmp125 = _tmp92 * fh1;
  const Scalar _tmp126 = _tmp70 * _tmp71;
  const Scalar _tmp127 = _tmp101 * _tmp126 + _tmp99;
  const Scalar _tmp128 = -_tmp108 * _tmp127 + _tmp109 * _tmp126 - _tmp97;
  const Scalar _tmp129 = _tmp118 * (-_tmp117 * _tmp128 + _tmp126 * _tmp69 - _tmp68);
  const Scalar _tmp130 = _tmp128 + _tmp129;
  const Scalar _tmp131 = -_tmp121 * _tmp130 - _tmp70;
  const Scalar _tmp132 = _tmp91 * fh1;
  const Scalar _tmp133 = -_tmp125 * _tmp67 * (_tmp120 * _tmp124 + _tmp122 * _tmp123) -
                         _tmp132 * _tmp67 * (_tmp123 * _tmp131 + _tmp124 * _tmp130 + Scalar(1.0)) -
                         _tmp67 * _tmp93 * (-_tmp56 * _tmp86 + _tmp80 * _tmp87) -
                         _tmp67 * _tmp94 * (-_tmp114 * _tmp56 * _tmp82 + _tmp114 * _tmp80);
  const Scalar _tmp134 = Scalar(1.0) / (_tmp133);
  const Scalar _tmp135 = _tmp104 + _tmp75;
  const Scalar _tmp136 = _tmp108 * _tmp135;
  const Scalar _tmp137 = Scalar(1.0) / (_tmp106 - _tmp136 - _tmp72);
  const Scalar _tmp138 = Scalar(1.0) * _tmp137;
  const Scalar _tmp139 = _tmp102 * _tmp111;
  const Scalar _tmp140 = _tmp135 * _tmp137;
  const Scalar _tmp141 = -_tmp101 * _tmp85 + _tmp119 * _tmp140 - _tmp120 * _tmp139;
  const Scalar _tmp142 = _tmp118 * _tmp138;
  const Scalar _tmp143 = -_tmp102 * _tmp87 + _tmp135 * _tmp142;
  const Scalar _tmp144 = _tmp127 + _tmp129 * _tmp140 - _tmp130 * _tmp139;
  const Scalar _tmp145 = _tmp46 * fh1;
  const Scalar _tmp146 = _tmp145 * _tmp91 + Scalar(3.29616) * _tmp33 + _tmp38 * fv1;
  const Scalar _tmp147 = _tmp105 * _tmp135;
  const Scalar _tmp148 = _tmp112 * _tmp137;
  const Scalar _tmp149 = _tmp104 - _tmp113 * _tmp139 - _tmp135 * _tmp148;
  const Scalar _tmp150 = _tmp136 * _tmp138 + Scalar(1.0);
  const Scalar _tmp151 = _tmp108 * _tmp138;
  const Scalar _tmp152 = -_tmp145 * _tmp92 - Scalar(3.29616) * _tmp21 - _tmp27 * fv1;
  const Scalar _tmp153 = std::asinh(
      _tmp134 * (Scalar(1.0) * _tmp125 * (-_tmp115 * _tmp141 + _tmp119 * _tmp138) +
                 Scalar(1.0) * _tmp132 * (-_tmp115 * _tmp144 + _tmp129 * _tmp138) +
                 Scalar(1.0) * _tmp146 * (_tmp138 * _tmp147 - _tmp138) +
                 Scalar(1.0) * _tmp152 * (-_tmp115 * _tmp150 + _tmp151) +
                 Scalar(1.0) * _tmp93 * (-_tmp115 * _tmp143 + _tmp142) +
                 Scalar(1.0) * _tmp94 * (-_tmp112 * _tmp138 - _tmp115 * _tmp149 + Scalar(1.0))));
  const Scalar _tmp154 = Scalar(9.6622558468725703) * _tmp133;
  const Scalar _tmp155 = _tmp114 * _tmp94;
  const Scalar _tmp156 =
      _tmp122 * _tmp125 * _tmp71 + _tmp131 * _tmp132 * _tmp71 - _tmp155 * _tmp82 - _tmp86 * _tmp93;
  const Scalar _tmp157 = Scalar(1.0) / (_tmp156);
  const Scalar _tmp158 = _tmp138 * _tmp146;
  const Scalar _tmp159 =
      std::asinh(_tmp157 * (_tmp105 * _tmp125 * _tmp141 + _tmp105 * _tmp132 * _tmp144 +
                            _tmp105 * _tmp143 * _tmp93 + _tmp105 * _tmp149 * _tmp94 +
                            _tmp105 * _tmp150 * _tmp152 - _tmp147 * _tmp158));
  const Scalar _tmp160 = Scalar(9.6622558468725703) * _tmp156;
  const Scalar _tmp161 =
      _tmp111 * _tmp120 * _tmp125 + _tmp111 * _tmp130 * _tmp132 + _tmp155 + _tmp87 * _tmp93;
  const Scalar _tmp162 = Scalar(1.0) / (_tmp161);
  const Scalar _tmp163 =
      std::asinh(_tmp162 * (-_tmp119 * _tmp125 * _tmp137 - _tmp129 * _tmp132 * _tmp137 -
                            _tmp142 * _tmp93 + _tmp148 * _tmp94 - _tmp151 * _tmp152 + _tmp158));
  const Scalar _tmp164 = Scalar(9.6622558468725703) * _tmp161;

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) =
      -_tmp40 *
          (Scalar(0.87679799772039002) * _tmp0 + std::cosh(Scalar(1.0) * _tmp1) -
           std::cosh(
               Scalar(0.1034955) * _tmp0 *
               (-_tmp1 * _tmp40 -
                Scalar(8.3196563700000006) *
                    std::sqrt(Scalar(
                        std::pow(Scalar(-Scalar(0.12019727204189803) * _tmp28 - 1), Scalar(2)) +
                        Scalar(0.057067943376852184) *
                            std::pow(Scalar(-Scalar(0.50315118556004401) * _tmp39 - 1),
                                     Scalar(2))))))) +
      _tmp46 + p_init2;
  _res(1, 0) =
      -_tmp154 *
          (Scalar(0.86625939559540499) * _tmp134 + std::cosh(Scalar(1.0) * _tmp153) -
           std::cosh(
               Scalar(0.1034955) * _tmp134 *
               (-_tmp153 * _tmp154 -
                Scalar(4.8333311099999996) *
                    std::sqrt(Scalar(
                        std::pow(Scalar(1 - Scalar(0.20689664689659551) * _tmp59), Scalar(2)) +
                        Scalar(0.13817235445745474) *
                            std::pow(Scalar(-Scalar(0.55659957866191134) * _tmp62 - 1),
                                     Scalar(2))))))) +
      _tmp97 + p_init2;
  _res(2, 0) =
      _tmp100 -
      _tmp160 *
          (Scalar(0.86565325453551001) * _tmp157 + std::cosh(Scalar(1.0) * _tmp159) -
           std::cosh(
               Scalar(0.1034955) * _tmp157 *
               (-_tmp159 * _tmp160 -
                Scalar(4.7752063900000001) *
                    std::sqrt(Scalar(
                        std::pow(Scalar(1 - Scalar(0.20941503221602112) * _tmp49), Scalar(2)) +
                        Scalar(0.32397683292140877) *
                            std::pow(Scalar(1 - Scalar(0.36791786395571047) * _tmp53),
                                     Scalar(2))))))) +
      p_init2;
  _res(3, 0) =
      -_tmp164 *
          (Scalar(0.87653584775870996) * _tmp162 + std::cosh(Scalar(1.0) * _tmp163) -
           std::cosh(
               Scalar(0.1034955) * _tmp162 *
               (-_tmp163 * _tmp164 -
                Scalar(8.3888750099999996) *
                    std::sqrt(Scalar(
                        Scalar(0.090254729040973036) *
                            std::pow(Scalar(1 - Scalar(0.39679052492160538) * _tmp73), Scalar(2)) +
                        std::pow(Scalar(-Scalar(0.11920549523123722) * _tmp76 - 1),
                                 Scalar(2))))))) +
      _tmp96 + p_init2;

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
