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
 * Symbolic function: IK_residual_func_cost1_Nl15
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost1Nl15(
    const Scalar fh1, const Scalar fv1, const Scalar rx, const Scalar ry, const Scalar rz,
    const Scalar p_init0, const Scalar p_init1, const Scalar p_init2, const Scalar rot_init_x,
    const Scalar rot_init_y, const Scalar rot_init_z, const Scalar rot_init_w,
    const Scalar epsilon) {
  // Total ops: 513

  // Unused inputs
  (void)epsilon;

  // Input arrays

  // Intermediate terms (166)
  const Scalar _tmp0 = Scalar(1.0) / (fh1);
  const Scalar _tmp1 = std::asinh(_tmp0 * fv1);
  const Scalar _tmp2 = Scalar(9.6622558468725703) * fh1;
  const Scalar _tmp3 = std::sqrt(
      Scalar(std::pow(rx, Scalar(2)) + std::pow(ry, Scalar(2)) + std::pow(rz, Scalar(2))));
  const Scalar _tmp4 = (Scalar(1) / Scalar(2)) * _tmp3;
  const Scalar _tmp5 = std::cos(_tmp4);
  const Scalar _tmp6 = std::sin(_tmp4) / _tmp3;
  const Scalar _tmp7 = _tmp6 * ry;
  const Scalar _tmp8 = _tmp6 * rx;
  const Scalar _tmp9 = _tmp6 * rz;
  const Scalar _tmp10 =
      _tmp5 * rot_init_x - _tmp7 * rot_init_z + _tmp8 * rot_init_w + _tmp9 * rot_init_y;
  const Scalar _tmp11 = -2 * std::pow(_tmp10, Scalar(2));
  const Scalar _tmp12 =
      _tmp5 * rot_init_z + _tmp7 * rot_init_x - _tmp8 * rot_init_y + _tmp9 * rot_init_w;
  const Scalar _tmp13 = 1 - 2 * std::pow(_tmp12, Scalar(2));
  const Scalar _tmp14 = Scalar(0.20999999999999999) * _tmp11 + Scalar(0.20999999999999999) * _tmp13;
  const Scalar _tmp15 =
      _tmp5 * rot_init_y + _tmp7 * rot_init_w + _tmp8 * rot_init_z - _tmp9 * rot_init_x;
  const Scalar _tmp16 = 2 * _tmp12;
  const Scalar _tmp17 = _tmp15 * _tmp16;
  const Scalar _tmp18 =
      _tmp5 * rot_init_w - _tmp7 * rot_init_y - _tmp8 * rot_init_x - _tmp9 * rot_init_z;
  const Scalar _tmp19 = 2 * _tmp18;
  const Scalar _tmp20 = _tmp10 * _tmp19;
  const Scalar _tmp21 = _tmp17 - _tmp20;
  const Scalar _tmp22 = -Scalar(0.010999999999999999) * _tmp21;
  const Scalar _tmp23 = 2 * _tmp10 * _tmp15;
  const Scalar _tmp24 = _tmp16 * _tmp18;
  const Scalar _tmp25 = Scalar(0.20999999999999999) * _tmp23 + Scalar(0.20999999999999999) * _tmp24;
  const Scalar _tmp26 = _tmp22 + _tmp25;
  const Scalar _tmp27 = _tmp14 + _tmp26;
  const Scalar _tmp28 = _tmp27 + p_init1;
  const Scalar _tmp29 = -2 * std::pow(_tmp15, Scalar(2));
  const Scalar _tmp30 = Scalar(0.20999999999999999) * _tmp13 + Scalar(0.20999999999999999) * _tmp29;
  const Scalar _tmp31 = _tmp10 * _tmp16;
  const Scalar _tmp32 = _tmp15 * _tmp19;
  const Scalar _tmp33 = _tmp31 + _tmp32;
  const Scalar _tmp34 = -Scalar(0.010999999999999999) * _tmp33;
  const Scalar _tmp35 = Scalar(0.20999999999999999) * _tmp23 - Scalar(0.20999999999999999) * _tmp24;
  const Scalar _tmp36 = _tmp34 + _tmp35;
  const Scalar _tmp37 = _tmp30 + _tmp36;
  const Scalar _tmp38 = _tmp37 + p_init0;
  const Scalar _tmp39 = Scalar(0.20999999999999999) * _tmp17 + Scalar(0.20999999999999999) * _tmp20;
  const Scalar _tmp40 = -Scalar(0.010999999999999999) * _tmp11 -
                        Scalar(0.010999999999999999) * _tmp29 + Scalar(-0.010999999999999999);
  const Scalar _tmp41 = Scalar(0.20999999999999999) * _tmp31 - Scalar(0.20999999999999999) * _tmp32;
  const Scalar _tmp42 = _tmp40 + _tmp41;
  const Scalar _tmp43 = _tmp39 + _tmp42;
  const Scalar _tmp44 = -_tmp14;
  const Scalar _tmp45 = _tmp26 + _tmp44;
  const Scalar _tmp46 = _tmp45 + p_init1;
  const Scalar _tmp47 = _tmp34 - _tmp35;
  const Scalar _tmp48 = _tmp30 + _tmp47;
  const Scalar _tmp49 = _tmp48 + p_init0;
  const Scalar _tmp50 = Scalar(1.0) * _tmp45;
  const Scalar _tmp51 = -_tmp50;
  const Scalar _tmp52 = _tmp22 - _tmp25;
  const Scalar _tmp53 = _tmp14 + _tmp52;
  const Scalar _tmp54 = Scalar(1.0) / (_tmp51 + _tmp53);
  const Scalar _tmp55 = -_tmp30;
  const Scalar _tmp56 = _tmp36 + _tmp55;
  const Scalar _tmp57 = Scalar(1.0) * _tmp48;
  const Scalar _tmp58 = _tmp54 * (-_tmp56 + _tmp57);
  const Scalar _tmp59 = _tmp53 + p_init1;
  const Scalar _tmp60 = _tmp59 + Scalar(-4.8333311099999996);
  const Scalar _tmp61 = _tmp56 + p_init0;
  const Scalar _tmp62 = _tmp61 + Scalar(1.79662371);
  const Scalar _tmp63 = std::pow(Scalar(std::pow(_tmp60, Scalar(2)) + std::pow(_tmp62, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp64 = _tmp62 * _tmp63;
  const Scalar _tmp65 = -_tmp39;
  const Scalar _tmp66 = _tmp42 + _tmp65;
  const Scalar _tmp67 = _tmp49 + Scalar(-2.5202214700000001);
  const Scalar _tmp68 = Scalar(1.0) / (_tmp67);
  const Scalar _tmp69 = _tmp46 + Scalar(8.3888750099999996);
  const Scalar _tmp70 = _tmp68 * _tmp69;
  const Scalar _tmp71 = _tmp66 * _tmp70;
  const Scalar _tmp72 = _tmp40 - _tmp41;
  const Scalar _tmp73 = _tmp39 + _tmp72;
  const Scalar _tmp74 = _tmp60 * _tmp63;
  const Scalar _tmp75 = -_tmp64 * _tmp71 + _tmp73 * _tmp74;
  const Scalar _tmp76 = Scalar(1.0) / (_tmp64 * _tmp70 - _tmp74);
  const Scalar _tmp77 = Scalar(1.0) * _tmp76;
  const Scalar _tmp78 = _tmp75 * _tmp77;
  const Scalar _tmp79 = _tmp64 * _tmp66 - _tmp64 * _tmp73;
  const Scalar _tmp80 = _tmp58 * _tmp78 - _tmp77 * _tmp79;
  const Scalar _tmp81 = _tmp44 + _tmp52;
  const Scalar _tmp82 = _tmp81 + p_init1;
  const Scalar _tmp83 = _tmp82 + Scalar(8.3196563700000006);
  const Scalar _tmp84 = _tmp47 + _tmp55;
  const Scalar _tmp85 = _tmp84 + p_init0;
  const Scalar _tmp86 = _tmp85 + Scalar(1.9874742000000001);
  const Scalar _tmp87 = std::pow(Scalar(std::pow(_tmp83, Scalar(2)) + std::pow(_tmp86, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp88 = _tmp86 * _tmp87;
  const Scalar _tmp89 = _tmp83 * _tmp87;
  const Scalar _tmp90 = _tmp70 * _tmp88 - _tmp89;
  const Scalar _tmp91 = _tmp76 * _tmp90;
  const Scalar _tmp92 = _tmp65 + _tmp72;
  const Scalar _tmp93 = -_tmp71 * _tmp88 - _tmp75 * _tmp91 + _tmp89 * _tmp92;
  const Scalar _tmp94 = -_tmp58 * _tmp93 + _tmp66 * _tmp88 - _tmp79 * _tmp91 - _tmp88 * _tmp92;
  const Scalar _tmp95 = Scalar(1.0) / (_tmp94);
  const Scalar _tmp96 =
      std::sqrt(Scalar(std::pow(_tmp67, Scalar(2)) + std::pow(_tmp69, Scalar(2))));
  const Scalar _tmp97 = Scalar(1.0) / (_tmp96);
  const Scalar _tmp98 = _tmp68 * _tmp96;
  const Scalar _tmp99 = _tmp98 * (-_tmp45 * _tmp67 * _tmp97 + _tmp48 * _tmp69 * _tmp97);
  const Scalar _tmp100 = _tmp53 * _tmp64 - _tmp56 * _tmp74 + _tmp64 * _tmp99;
  const Scalar _tmp101 = -_tmp100 * _tmp91 + _tmp81 * _tmp88 - _tmp84 * _tmp89 + _tmp88 * _tmp99;
  const Scalar _tmp102 = _tmp101 * _tmp95;
  const Scalar _tmp103 = Scalar(1.0) / (_tmp101);
  const Scalar _tmp104 = _tmp103 * _tmp94;
  const Scalar _tmp105 = _tmp104 * (-_tmp100 * _tmp77 - _tmp102 * _tmp80);
  const Scalar _tmp106 = _tmp105 + _tmp80;
  const Scalar _tmp107 = _tmp88 * _tmp95;
  const Scalar _tmp108 = _tmp90 * _tmp95;
  const Scalar _tmp109 = -_tmp106 * _tmp108 + Scalar(1.0);
  const Scalar _tmp110 = _tmp64 * _tmp76;
  const Scalar _tmp111 = _tmp28 + Scalar(-4.7752063900000001);
  const Scalar _tmp112 = _tmp38 + Scalar(-2.71799795);
  const Scalar _tmp113 =
      std::pow(Scalar(std::pow(_tmp111, Scalar(2)) + std::pow(_tmp112, Scalar(2))),
               Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp114 = _tmp111 * _tmp113;
  const Scalar _tmp115 = _tmp114 * fh1;
  const Scalar _tmp116 = Scalar(1.0) * _tmp103;
  const Scalar _tmp117 = _tmp64 * _tmp91;
  const Scalar _tmp118 = _tmp112 * _tmp113;
  const Scalar _tmp119 = fh1 * (_tmp114 * _tmp37 - _tmp118 * _tmp27);
  const Scalar _tmp120 = _tmp70 * _tmp76;
  const Scalar _tmp121 = _tmp120 * _tmp75 + _tmp71;
  const Scalar _tmp122 = _tmp120 * _tmp79 - _tmp121 * _tmp58 - _tmp66;
  const Scalar _tmp123 = _tmp104 * (_tmp100 * _tmp120 - _tmp102 * _tmp122 - _tmp99);
  const Scalar _tmp124 = _tmp122 + _tmp123;
  const Scalar _tmp125 = -_tmp108 * _tmp124 - _tmp70;
  const Scalar _tmp126 = _tmp118 * fh1;
  const Scalar _tmp127 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp128 = _tmp50 * _tmp58 + _tmp57;
  const Scalar _tmp129 = 0;
  const Scalar _tmp130 = _tmp129 * _tmp95;
  const Scalar _tmp131 = -_tmp115 * _tmp98 * (_tmp106 * _tmp107 + _tmp109 * _tmp110) -
                         _tmp119 * _tmp98 * (-_tmp116 * _tmp117 + _tmp116 * _tmp88) -
                         _tmp126 * _tmp98 * (_tmp107 * _tmp124 + _tmp110 * _tmp125 + Scalar(1.0)) -
                         _tmp127 * _tmp98 * (-_tmp117 * _tmp130 + _tmp130 * _tmp88);
  const Scalar _tmp132 = Scalar(1.0) / (_tmp131);
  const Scalar _tmp133 = _tmp51 + _tmp81;
  const Scalar _tmp134 = _tmp133 * _tmp58;
  const Scalar _tmp135 = Scalar(1.0) / (-_tmp134 + _tmp57 - _tmp84);
  const Scalar _tmp136 = _tmp133 * _tmp135;
  const Scalar _tmp137 = _tmp93 * _tmp95;
  const Scalar _tmp138 = _tmp105 * _tmp136 - _tmp106 * _tmp137 - _tmp78;
  const Scalar _tmp139 = Scalar(1.0) * _tmp54;
  const Scalar _tmp140 = Scalar(1.0) * _tmp135;
  const Scalar _tmp141 = _tmp43 * fh1;
  const Scalar _tmp142 = _tmp118 * _tmp141 + Scalar(3.29616) * _tmp33 + _tmp37 * fv1;
  const Scalar _tmp143 = _tmp133 * _tmp54;
  const Scalar _tmp144 = -_tmp114 * _tmp141 - Scalar(3.29616) * _tmp21 - _tmp27 * fv1;
  const Scalar _tmp145 = _tmp134 * _tmp140 + Scalar(1.0);
  const Scalar _tmp146 = _tmp140 * _tmp58;
  const Scalar _tmp147 = _tmp128 * _tmp135;
  const Scalar _tmp148 = _tmp54 * (-_tmp129 * _tmp137 - _tmp133 * _tmp147 + _tmp51);
  const Scalar _tmp149 = _tmp121 + _tmp123 * _tmp136 - _tmp124 * _tmp137;
  const Scalar _tmp150 = _tmp104 * _tmp140;
  const Scalar _tmp151 = -_tmp116 * _tmp93 + _tmp133 * _tmp150;
  const Scalar _tmp152 =
      std::asinh(_tmp132 * (Scalar(1.0) * _tmp115 * (_tmp105 * _tmp140 - _tmp138 * _tmp139) +
                            Scalar(1.0) * _tmp119 * (-_tmp139 * _tmp151 + _tmp150) +
                            Scalar(1.0) * _tmp126 * (_tmp123 * _tmp140 - _tmp139 * _tmp149) +
                            Scalar(1.0) * _tmp127 *
                                (-_tmp128 * _tmp140 - Scalar(1.0) * _tmp148 + Scalar(1.0)) +
                            Scalar(1.0) * _tmp142 * (_tmp140 * _tmp143 - _tmp140) +
                            Scalar(1.0) * _tmp144 * (-_tmp139 * _tmp145 + _tmp146)));
  const Scalar _tmp153 = Scalar(9.6622558468725703) * _tmp131;
  const Scalar _tmp154 = _tmp116 * _tmp119;
  const Scalar _tmp155 = _tmp127 * _tmp130;
  const Scalar _tmp156 =
      _tmp109 * _tmp115 * _tmp76 + _tmp125 * _tmp126 * _tmp76 - _tmp154 * _tmp91 - _tmp155 * _tmp91;
  const Scalar _tmp157 = Scalar(1.0) / (_tmp156);
  const Scalar _tmp158 = _tmp140 * _tmp142;
  const Scalar _tmp159 = _tmp54 * fh1;
  const Scalar _tmp160 =
      std::asinh(_tmp157 * (_tmp114 * _tmp138 * _tmp159 + _tmp118 * _tmp149 * _tmp159 +
                            _tmp119 * _tmp151 * _tmp54 + _tmp127 * _tmp148 - _tmp143 * _tmp158 +
                            _tmp144 * _tmp145 * _tmp54));
  const Scalar _tmp161 = Scalar(9.6622558468725703) * _tmp156;
  const Scalar _tmp162 =
      _tmp106 * _tmp115 * _tmp95 + _tmp124 * _tmp126 * _tmp95 + _tmp154 + _tmp155;
  const Scalar _tmp163 = Scalar(1.0) / (_tmp162);
  const Scalar _tmp164 = std::asinh(_tmp163 * (-_tmp105 * _tmp115 * _tmp135 - _tmp119 * _tmp150 -
                                               _tmp123 * _tmp126 * _tmp135 + _tmp127 * _tmp147 -
                                               _tmp144 * _tmp146 + _tmp158));
  const Scalar _tmp165 = Scalar(9.6622558468725703) * _tmp162;

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) =
      -_tmp2 * (Scalar(0.86565325453551001) * _tmp0 + std::cosh(Scalar(1.0) * _tmp1) -
                std::cosh(
                    Scalar(0.1034955) * _tmp0 *
                    (-_tmp1 * _tmp2 -
                     Scalar(4.7752063900000001) *
                         std::sqrt(Scalar(
                             std::pow(Scalar(1 - Scalar(0.20941503221602112) * _tmp28), Scalar(2)) +
                             Scalar(0.32397683292140877) *
                                 std::pow(Scalar(1 - Scalar(0.36791786395571047) * _tmp38),
                                          Scalar(2))))))) +
      _tmp43 + p_init2;
  _res(1, 0) =
      -_tmp153 *
          (Scalar(0.87653584775870996) * _tmp132 + std::cosh(Scalar(1.0) * _tmp152) -
           std::cosh(
               Scalar(0.1034955) * _tmp132 *
               (-_tmp152 * _tmp153 -
                Scalar(8.3888750099999996) *
                    std::sqrt(Scalar(
                        Scalar(0.090254729040973036) *
                            std::pow(Scalar(1 - Scalar(0.39679052492160538) * _tmp49), Scalar(2)) +
                        std::pow(Scalar(-Scalar(0.11920549523123722) * _tmp46 - 1),
                                 Scalar(2))))))) +
      _tmp66 + p_init2;
  _res(2, 0) =
      -_tmp161 *
          (Scalar(0.86625939559540499) * _tmp157 + std::cosh(Scalar(1.0) * _tmp160) -
           std::cosh(
               Scalar(0.1034955) * _tmp157 *
               (-_tmp160 * _tmp161 -
                Scalar(4.8333311099999996) *
                    std::sqrt(Scalar(
                        std::pow(Scalar(1 - Scalar(0.20689664689659551) * _tmp59), Scalar(2)) +
                        Scalar(0.13817235445745474) *
                            std::pow(Scalar(-Scalar(0.55659957866191134) * _tmp61 - 1),
                                     Scalar(2))))))) +
      _tmp73 + p_init2;
  _res(3, 0) =
      -_tmp165 *
          (Scalar(0.87679799772039002) * _tmp163 + std::cosh(Scalar(1.0) * _tmp164) -
           std::cosh(
               Scalar(0.1034955) * _tmp163 *
               (-_tmp164 * _tmp165 -
                Scalar(8.3196563700000006) *
                    std::sqrt(Scalar(
                        std::pow(Scalar(-Scalar(0.12019727204189803) * _tmp82 - 1), Scalar(2)) +
                        Scalar(0.057067943376852184) *
                            std::pow(Scalar(-Scalar(0.50315118556004401) * _tmp85 - 1),
                                     Scalar(2))))))) +
      _tmp92 + p_init2;

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
