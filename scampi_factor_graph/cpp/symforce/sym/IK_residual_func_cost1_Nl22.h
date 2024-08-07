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
 * Symbolic function: IK_residual_func_cost1_Nl22
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost1Nl22(
    const Scalar fh1, const Scalar fv1, const Scalar rx, const Scalar ry, const Scalar rz,
    const Scalar p_init0, const Scalar p_init1, const Scalar p_init2, const Scalar rot_init_x,
    const Scalar rot_init_y, const Scalar rot_init_z, const Scalar rot_init_w,
    const Scalar epsilon) {
  // Total ops: 509

  // Unused inputs
  (void)epsilon;

  // Input arrays

  // Intermediate terms (162)
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
      _tmp5 * rot_init_z + _tmp7 * rot_init_x - _tmp8 * rot_init_y + _tmp9 * rot_init_w;
  const Scalar _tmp11 = -2 * std::pow(_tmp10, Scalar(2));
  const Scalar _tmp12 =
      _tmp5 * rot_init_x - _tmp7 * rot_init_z + _tmp8 * rot_init_w + _tmp9 * rot_init_y;
  const Scalar _tmp13 = 1 - 2 * std::pow(_tmp12, Scalar(2));
  const Scalar _tmp14 = Scalar(0.20999999999999999) * _tmp11 + Scalar(0.20999999999999999) * _tmp13;
  const Scalar _tmp15 =
      _tmp5 * rot_init_y + _tmp7 * rot_init_w + _tmp8 * rot_init_z - _tmp9 * rot_init_x;
  const Scalar _tmp16 = 2 * _tmp10;
  const Scalar _tmp17 = _tmp15 * _tmp16;
  const Scalar _tmp18 =
      _tmp5 * rot_init_w - _tmp7 * rot_init_y - _tmp8 * rot_init_x - _tmp9 * rot_init_z;
  const Scalar _tmp19 = 2 * _tmp12;
  const Scalar _tmp20 = _tmp18 * _tmp19;
  const Scalar _tmp21 = _tmp17 - _tmp20;
  const Scalar _tmp22 = -Scalar(0.010999999999999999) * _tmp21;
  const Scalar _tmp23 = _tmp15 * _tmp19;
  const Scalar _tmp24 = _tmp16 * _tmp18;
  const Scalar _tmp25 = Scalar(0.20999999999999999) * _tmp23 + Scalar(0.20999999999999999) * _tmp24;
  const Scalar _tmp26 = _tmp22 - _tmp25;
  const Scalar _tmp27 = _tmp14 + _tmp26;
  const Scalar _tmp28 = _tmp27 + p_init1;
  const Scalar _tmp29 = Scalar(0.20999999999999999) * _tmp23 - Scalar(0.20999999999999999) * _tmp24;
  const Scalar _tmp30 = _tmp10 * _tmp19;
  const Scalar _tmp31 = 2 * _tmp15 * _tmp18;
  const Scalar _tmp32 = _tmp30 + _tmp31;
  const Scalar _tmp33 = -Scalar(0.010999999999999999) * _tmp32;
  const Scalar _tmp34 = -2 * std::pow(_tmp15, Scalar(2));
  const Scalar _tmp35 = Scalar(0.20999999999999999) * _tmp11 +
                        Scalar(0.20999999999999999) * _tmp34 + Scalar(0.20999999999999999);
  const Scalar _tmp36 = _tmp33 - _tmp35;
  const Scalar _tmp37 = _tmp29 + _tmp36;
  const Scalar _tmp38 = _tmp37 + p_init0;
  const Scalar _tmp39 = Scalar(0.20999999999999999) * _tmp30 - Scalar(0.20999999999999999) * _tmp31;
  const Scalar _tmp40 = -_tmp39;
  const Scalar _tmp41 =
      -Scalar(0.010999999999999999) * _tmp13 - Scalar(0.010999999999999999) * _tmp34;
  const Scalar _tmp42 = Scalar(0.20999999999999999) * _tmp17 + Scalar(0.20999999999999999) * _tmp20;
  const Scalar _tmp43 = _tmp41 + _tmp42;
  const Scalar _tmp44 = _tmp40 + _tmp43;
  const Scalar _tmp45 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp46 = _tmp22 + _tmp25;
  const Scalar _tmp47 = _tmp14 + _tmp46;
  const Scalar _tmp48 = Scalar(1.0) * _tmp47;
  const Scalar _tmp49 = -_tmp29;
  const Scalar _tmp50 = _tmp36 + _tmp49;
  const Scalar _tmp51 = _tmp33 + _tmp35;
  const Scalar _tmp52 = _tmp29 + _tmp51;
  const Scalar _tmp53 = Scalar(1.0) * _tmp52;
  const Scalar _tmp54 = -_tmp48;
  const Scalar _tmp55 = -_tmp14;
  const Scalar _tmp56 = _tmp26 + _tmp55;
  const Scalar _tmp57 = Scalar(1.0) / (_tmp54 + _tmp56);
  const Scalar _tmp58 = _tmp57 * (-_tmp50 + _tmp53);
  const Scalar _tmp59 = _tmp48 * _tmp58 + _tmp53;
  const Scalar _tmp60 = _tmp49 + _tmp51;
  const Scalar _tmp61 = _tmp46 + _tmp55;
  const Scalar _tmp62 = _tmp54 + _tmp61;
  const Scalar _tmp63 = _tmp58 * _tmp62;
  const Scalar _tmp64 = Scalar(1.0) / (_tmp53 - _tmp60 - _tmp63);
  const Scalar _tmp65 = Scalar(1.0) * _tmp64;
  const Scalar _tmp66 = _tmp41 - _tmp42;
  const Scalar _tmp67 = _tmp39 + _tmp66;
  const Scalar _tmp68 = _tmp60 + p_init0;
  const Scalar _tmp69 = _tmp68 + Scalar(-2.5202214700000001);
  const Scalar _tmp70 = _tmp61 + p_init1;
  const Scalar _tmp71 = _tmp70 + Scalar(8.3888750099999996);
  const Scalar _tmp72 = std::pow(Scalar(std::pow(_tmp69, Scalar(2)) + std::pow(_tmp71, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp73 = _tmp71 * _tmp72;
  const Scalar _tmp74 = _tmp52 + p_init0;
  const Scalar _tmp75 = _tmp74 + Scalar(-2.71799795);
  const Scalar _tmp76 = Scalar(1.0) / (_tmp75);
  const Scalar _tmp77 = _tmp47 + p_init1;
  const Scalar _tmp78 = _tmp77 + Scalar(-4.7752063900000001);
  const Scalar _tmp79 = _tmp76 * _tmp78;
  const Scalar _tmp80 = _tmp39 + _tmp43;
  const Scalar _tmp81 = _tmp69 * _tmp72;
  const Scalar _tmp82 = _tmp80 * _tmp81;
  const Scalar _tmp83 = -_tmp73 + _tmp79 * _tmp81;
  const Scalar _tmp84 = _tmp56 + p_init1;
  const Scalar _tmp85 = _tmp84 + Scalar(8.3196563700000006);
  const Scalar _tmp86 = _tmp50 + p_init0;
  const Scalar _tmp87 = _tmp86 + Scalar(1.9874742000000001);
  const Scalar _tmp88 = std::pow(Scalar(std::pow(_tmp85, Scalar(2)) + std::pow(_tmp87, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp89 = _tmp85 * _tmp88;
  const Scalar _tmp90 = _tmp87 * _tmp88;
  const Scalar _tmp91 = Scalar(1.0) / (_tmp79 * _tmp90 - _tmp89);
  const Scalar _tmp92 = _tmp80 * _tmp90;
  const Scalar _tmp93 = _tmp40 + _tmp66;
  const Scalar _tmp94 = _tmp91 * (-_tmp79 * _tmp92 + _tmp89 * _tmp93);
  const Scalar _tmp95 = _tmp67 * _tmp73 - _tmp79 * _tmp82 - _tmp83 * _tmp94;
  const Scalar _tmp96 = -_tmp90 * _tmp93 + _tmp92;
  const Scalar _tmp97 = _tmp83 * _tmp91;
  const Scalar _tmp98 = -_tmp58 * _tmp95 - _tmp67 * _tmp81 + _tmp82 - _tmp96 * _tmp97;
  const Scalar _tmp99 = Scalar(1.0) / (_tmp98);
  const Scalar _tmp100 = 0;
  const Scalar _tmp101 = _tmp59 * _tmp64;
  const Scalar _tmp102 = _tmp57 * (-_tmp100 * _tmp95 - _tmp101 * _tmp62 + _tmp54);
  const Scalar _tmp103 = Scalar(1.0) * _tmp94;
  const Scalar _tmp104 = Scalar(1.0) * _tmp91;
  const Scalar _tmp105 = _tmp103 * _tmp58 - _tmp104 * _tmp96;
  const Scalar _tmp106 =
      std::sqrt(Scalar(std::pow(_tmp75, Scalar(2)) + std::pow(_tmp78, Scalar(2))));
  const Scalar _tmp107 = Scalar(1.0) / (_tmp106);
  const Scalar _tmp108 = _tmp106 * _tmp76;
  const Scalar _tmp109 = _tmp108 * (-_tmp107 * _tmp47 * _tmp75 + _tmp107 * _tmp52 * _tmp78);
  const Scalar _tmp110 = _tmp109 * _tmp90 - _tmp50 * _tmp89 + _tmp56 * _tmp90;
  const Scalar _tmp111 = _tmp109 * _tmp81 - _tmp110 * _tmp97 - _tmp60 * _tmp73 + _tmp61 * _tmp81;
  const Scalar _tmp112 = _tmp111 * _tmp99;
  const Scalar _tmp113 = Scalar(1.0) / (_tmp111);
  const Scalar _tmp114 = _tmp113 * _tmp98;
  const Scalar _tmp115 = _tmp114 * (-_tmp104 * _tmp110 - _tmp105 * _tmp112);
  const Scalar _tmp116 = _tmp99 * (_tmp105 + _tmp115);
  const Scalar _tmp117 = _tmp62 * _tmp64;
  const Scalar _tmp118 = -_tmp103 + _tmp115 * _tmp117 - _tmp116 * _tmp95;
  const Scalar _tmp119 = Scalar(1.0) * _tmp57;
  const Scalar _tmp120 = _tmp28 + Scalar(-4.8333311099999996);
  const Scalar _tmp121 = _tmp38 + Scalar(1.79662371);
  const Scalar _tmp122 =
      std::pow(Scalar(std::pow(_tmp120, Scalar(2)) + std::pow(_tmp121, Scalar(2))),
               Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp123 = _tmp120 * _tmp122;
  const Scalar _tmp124 = _tmp123 * fh1;
  const Scalar _tmp125 = _tmp44 * fh1;
  const Scalar _tmp126 = -_tmp123 * _tmp125 - Scalar(3.29616) * _tmp21 - _tmp27 * fv1;
  const Scalar _tmp127 = _tmp57 * (_tmp63 * _tmp65 + Scalar(1.0));
  const Scalar _tmp128 = _tmp58 * _tmp65;
  const Scalar _tmp129 = _tmp114 * _tmp65;
  const Scalar _tmp130 = Scalar(1.0) * _tmp113;
  const Scalar _tmp131 = _tmp129 * _tmp62 - _tmp130 * _tmp95;
  const Scalar _tmp132 = _tmp121 * _tmp122;
  const Scalar _tmp133 = fh1 * (_tmp123 * _tmp37 - _tmp132 * _tmp27);
  const Scalar _tmp134 = _tmp79 * _tmp80 + _tmp79 * _tmp94;
  const Scalar _tmp135 = _tmp79 * _tmp91;
  const Scalar _tmp136 = -_tmp134 * _tmp58 + _tmp135 * _tmp96 - _tmp80;
  const Scalar _tmp137 = _tmp114 * (-_tmp109 + _tmp110 * _tmp135 - _tmp112 * _tmp136);
  const Scalar _tmp138 = _tmp99 * (_tmp136 + _tmp137);
  const Scalar _tmp139 = _tmp117 * _tmp137 + _tmp134 - _tmp138 * _tmp95;
  const Scalar _tmp140 = _tmp132 * fh1;
  const Scalar _tmp141 = _tmp57 * _tmp62;
  const Scalar _tmp142 = _tmp125 * _tmp132 + Scalar(3.29616) * _tmp32 + _tmp37 * fv1;
  const Scalar _tmp143 = -_tmp116 * _tmp83 + Scalar(1.0);
  const Scalar _tmp144 = _tmp90 * _tmp91;
  const Scalar _tmp145 = _tmp90 * _tmp97;
  const Scalar _tmp146 = -_tmp138 * _tmp83 - _tmp79;
  const Scalar _tmp147 = -_tmp108 * _tmp124 * (_tmp116 * _tmp81 + _tmp143 * _tmp144) -
                         _tmp108 * _tmp133 * (-_tmp130 * _tmp145 + _tmp130 * _tmp81) -
                         _tmp108 * _tmp140 * (_tmp138 * _tmp81 + _tmp144 * _tmp146 + Scalar(1.0)) -
                         _tmp108 * _tmp45 * (-_tmp100 * _tmp145 + _tmp100 * _tmp81);
  const Scalar _tmp148 = Scalar(1.0) / (_tmp147);
  const Scalar _tmp149 = std::asinh(
      _tmp148 * (Scalar(1.0) * _tmp124 * (_tmp115 * _tmp65 - _tmp118 * _tmp119) +
                 Scalar(1.0) * _tmp126 * (-Scalar(1.0) * _tmp127 + _tmp128) +
                 Scalar(1.0) * _tmp133 * (-_tmp119 * _tmp131 + _tmp129) +
                 Scalar(1.0) * _tmp140 * (-_tmp119 * _tmp139 + _tmp137 * _tmp65) +
                 Scalar(1.0) * _tmp142 * (_tmp141 * _tmp65 - _tmp65) +
                 Scalar(1.0) * _tmp45 * (-Scalar(1.0) * _tmp102 - _tmp59 * _tmp65 + Scalar(1.0))));
  const Scalar _tmp150 = Scalar(9.6622558468725703) * _tmp147;
  const Scalar _tmp151 = _tmp100 * _tmp45;
  const Scalar _tmp152 = _tmp130 * _tmp133;
  const Scalar _tmp153 =
      _tmp124 * _tmp143 * _tmp91 + _tmp140 * _tmp146 * _tmp91 - _tmp151 * _tmp97 - _tmp152 * _tmp97;
  const Scalar _tmp154 = Scalar(1.0) / (_tmp153);
  const Scalar _tmp155 = _tmp142 * _tmp65;
  const Scalar _tmp156 = std::asinh(_tmp154 * (_tmp102 * _tmp45 + _tmp118 * _tmp124 * _tmp57 +
                                               _tmp126 * _tmp127 + _tmp131 * _tmp133 * _tmp57 +
                                               _tmp139 * _tmp140 * _tmp57 - _tmp141 * _tmp155));
  const Scalar _tmp157 = Scalar(9.6622558468725703) * _tmp153;
  const Scalar _tmp158 = _tmp116 * _tmp124 + _tmp138 * _tmp140 + _tmp151 + _tmp152;
  const Scalar _tmp159 = Scalar(1.0) / (_tmp158);
  const Scalar _tmp160 =
      std::asinh(_tmp159 * (_tmp101 * _tmp45 - _tmp115 * _tmp124 * _tmp64 - _tmp126 * _tmp128 -
                            _tmp129 * _tmp133 - _tmp137 * _tmp140 * _tmp64 + _tmp155));
  const Scalar _tmp161 = Scalar(9.6622558468725703) * _tmp158;

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) =
      -_tmp2 * (Scalar(0.86625939559540499) * _tmp0 + std::cosh(Scalar(1.0) * _tmp1) -
                std::cosh(
                    Scalar(0.1034955) * _tmp0 *
                    (-_tmp1 * _tmp2 -
                     Scalar(4.8333311099999996) *
                         std::sqrt(Scalar(
                             std::pow(Scalar(1 - Scalar(0.20689664689659551) * _tmp28), Scalar(2)) +
                             Scalar(0.13817235445745474) *
                                 std::pow(Scalar(-Scalar(0.55659957866191134) * _tmp38 - 1),
                                          Scalar(2))))))) +
      _tmp44 + p_init2;
  _res(1, 0) =
      -_tmp150 *
          (Scalar(0.86565325453551001) * _tmp148 + std::cosh(Scalar(1.0) * _tmp149) -
           std::cosh(
               Scalar(0.1034955) * _tmp148 *
               (-_tmp149 * _tmp150 -
                Scalar(4.7752063900000001) *
                    std::sqrt(Scalar(
                        Scalar(0.32397683292140877) *
                            std::pow(Scalar(1 - Scalar(0.36791786395571047) * _tmp74), Scalar(2)) +
                        std::pow(Scalar(1 - Scalar(0.20941503221602112) * _tmp77), Scalar(2))))))) +
      _tmp80 + p_init2;
  _res(2, 0) =
      -_tmp157 *
          (Scalar(0.87679799772039002) * _tmp154 + std::cosh(Scalar(1.0) * _tmp156) -
           std::cosh(
               Scalar(0.1034955) * _tmp154 *
               (-_tmp156 * _tmp157 -
                Scalar(8.3196563700000006) *
                    std::sqrt(Scalar(
                        std::pow(Scalar(-Scalar(0.12019727204189803) * _tmp84 - 1), Scalar(2)) +
                        Scalar(0.057067943376852184) *
                            std::pow(Scalar(-Scalar(0.50315118556004401) * _tmp86 - 1),
                                     Scalar(2))))))) +
      _tmp93 + p_init2;
  _res(3, 0) =
      -_tmp161 *
          (Scalar(0.87653584775870996) * _tmp159 + std::cosh(Scalar(1.0) * _tmp160) -
           std::cosh(
               Scalar(0.1034955) * _tmp159 *
               (-_tmp160 * _tmp161 -
                Scalar(8.3888750099999996) *
                    std::sqrt(Scalar(
                        Scalar(0.090254729040973036) *
                            std::pow(Scalar(1 - Scalar(0.39679052492160538) * _tmp68), Scalar(2)) +
                        std::pow(Scalar(-Scalar(0.11920549523123722) * _tmp70 - 1),
                                 Scalar(2))))))) +
      _tmp67 + p_init2;

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
