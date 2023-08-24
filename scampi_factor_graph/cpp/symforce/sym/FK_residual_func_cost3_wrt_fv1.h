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
 * Symbolic function: FK_residual_func_cost3_wrt_fv1
 *
 * Args:
 *     fh1: Scalar
 *     fv1: Scalar
 *     rx: Scalar
 *     ry: Scalar
 *     rz: Scalar
 *     tx: Scalar
 *     ty: Scalar
 *     tz: Scalar
 *     rot_init_x: Scalar
 *     rot_init_y: Scalar
 *     rot_init_z: Scalar
 *     rot_init_w: Scalar
 *     lc0: Scalar
 *     lc1: Scalar
 *     lc2: Scalar
 *     lc3: Scalar
 *     epsilon: Scalar
 *
 * Outputs:
 *     res: Matrix41
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 4, 1> FkResidualFuncCost3WrtFv1(
    const Scalar fh1, const Scalar fv1, const Scalar rx, const Scalar ry, const Scalar rz,
    const Scalar tx, const Scalar ty, const Scalar tz, const Scalar rot_init_x,
    const Scalar rot_init_y, const Scalar rot_init_z, const Scalar rot_init_w, const Scalar lc0,
    const Scalar lc1, const Scalar lc2, const Scalar lc3, const Scalar epsilon) {
  // Total ops: 301

  // Unused inputs
  (void)tz;
  (void)lc0;
  (void)lc1;
  (void)lc2;
  (void)lc3;
  (void)epsilon;

  // Input arrays

  // Intermediate terms (111)
  const Scalar _tmp0 = std::sqrt(
      Scalar(std::pow(rx, Scalar(2)) + std::pow(ry, Scalar(2)) + std::pow(rz, Scalar(2))));
  const Scalar _tmp1 = (Scalar(1) / Scalar(2)) * _tmp0;
  const Scalar _tmp2 = std::cos(_tmp1);
  const Scalar _tmp3 = std::sin(_tmp1) / _tmp0;
  const Scalar _tmp4 = _tmp3 * ry;
  const Scalar _tmp5 = _tmp3 * rot_init_w;
  const Scalar _tmp6 = _tmp3 * rz;
  const Scalar _tmp7 = _tmp2 * rot_init_x - _tmp4 * rot_init_z + _tmp5 * rx + _tmp6 * rot_init_y;
  const Scalar _tmp8 = -2 * std::pow(_tmp7, Scalar(2));
  const Scalar _tmp9 = _tmp3 * rx;
  const Scalar _tmp10 = _tmp2 * rot_init_z + _tmp4 * rot_init_x + _tmp5 * rz - _tmp9 * rot_init_y;
  const Scalar _tmp11 = -2 * std::pow(_tmp10, Scalar(2));
  const Scalar _tmp12 = Scalar(0.20999999999999999) * _tmp11 + Scalar(0.20999999999999999) * _tmp8 +
                        Scalar(0.20999999999999999);
  const Scalar _tmp13 = -_tmp12;
  const Scalar _tmp14 =
      _tmp2 * rot_init_y + _tmp4 * rot_init_w - _tmp6 * rot_init_x + _tmp9 * rot_init_z;
  const Scalar _tmp15 = 2 * _tmp10;
  const Scalar _tmp16 = _tmp14 * _tmp15;
  const Scalar _tmp17 = 2 * _tmp2 * rot_init_w - 2 * _tmp4 * rot_init_y - 2 * _tmp6 * rot_init_z -
                        2 * _tmp9 * rot_init_x;
  const Scalar _tmp18 = _tmp17 * _tmp7;
  const Scalar _tmp19 =
      -Scalar(0.010999999999999999) * _tmp16 + Scalar(0.010999999999999999) * _tmp18;
  const Scalar _tmp20 = 2 * _tmp14 * _tmp7;
  const Scalar _tmp21 = _tmp10 * _tmp17;
  const Scalar _tmp22 = Scalar(0.20999999999999999) * _tmp20 + Scalar(0.20999999999999999) * _tmp21;
  const Scalar _tmp23 = _tmp19 + _tmp22;
  const Scalar _tmp24 = _tmp13 + _tmp23;
  const Scalar _tmp25 = _tmp24 + ty + Scalar(8.3888750099999996);
  const Scalar _tmp26 = 1 - 2 * std::pow(_tmp14, Scalar(2));
  const Scalar _tmp27 = Scalar(0.20999999999999999) * _tmp11 + Scalar(0.20999999999999999) * _tmp26;
  const Scalar _tmp28 = Scalar(0.20999999999999999) * _tmp20 - Scalar(0.20999999999999999) * _tmp21;
  const Scalar _tmp29 = _tmp15 * _tmp7;
  const Scalar _tmp30 = _tmp14 * _tmp17;
  const Scalar _tmp31 =
      -Scalar(0.010999999999999999) * _tmp29 - Scalar(0.010999999999999999) * _tmp30;
  const Scalar _tmp32 = -_tmp28 + _tmp31;
  const Scalar _tmp33 = _tmp27 + _tmp32;
  const Scalar _tmp34 = _tmp33 + tx + Scalar(-2.5202214700000001);
  const Scalar _tmp35 =
      std::sqrt(Scalar(std::pow(_tmp25, Scalar(2)) + std::pow(_tmp34, Scalar(2))));
  const Scalar _tmp36 = Scalar(1.0) / (_tmp35);
  const Scalar _tmp37 = Scalar(1.0) / (_tmp34);
  const Scalar _tmp38 = _tmp35 * _tmp37;
  const Scalar _tmp39 = _tmp38 * (-_tmp24 * _tmp34 * _tmp36 + _tmp25 * _tmp33 * _tmp36);
  const Scalar _tmp40 = _tmp12 + _tmp23;
  const Scalar _tmp41 = _tmp28 + _tmp31;
  const Scalar _tmp42 = _tmp27 + _tmp41;
  const Scalar _tmp43 = _tmp42 + tx + Scalar(-2.71799795);
  const Scalar _tmp44 = _tmp40 + ty + Scalar(-4.7752063900000001);
  const Scalar _tmp45 = std::pow(Scalar(std::pow(_tmp43, Scalar(2)) + std::pow(_tmp44, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp46 = _tmp43 * _tmp45;
  const Scalar _tmp47 = _tmp44 * _tmp45;
  const Scalar _tmp48 = _tmp39 * _tmp46 + _tmp40 * _tmp46 - _tmp42 * _tmp47;
  const Scalar _tmp49 = _tmp25 * _tmp37;
  const Scalar _tmp50 = Scalar(1.0) / (_tmp46 * _tmp49 - _tmp47);
  const Scalar _tmp51 = _tmp49 * _tmp50;
  const Scalar _tmp52 =
      -Scalar(0.010999999999999999) * _tmp26 - Scalar(0.010999999999999999) * _tmp8;
  const Scalar _tmp53 = Scalar(0.20999999999999999) * _tmp16 + Scalar(0.20999999999999999) * _tmp18;
  const Scalar _tmp54 = Scalar(0.20999999999999999) * _tmp29 - Scalar(0.20999999999999999) * _tmp30;
  const Scalar _tmp55 = _tmp52 - _tmp53 + _tmp54;
  const Scalar _tmp56 = _tmp46 * _tmp55;
  const Scalar _tmp57 = _tmp52 + _tmp53;
  const Scalar _tmp58 = _tmp54 + _tmp57;
  const Scalar _tmp59 = -_tmp46 * _tmp58 + _tmp56;
  const Scalar _tmp60 = _tmp47 * _tmp58 - _tmp49 * _tmp56;
  const Scalar _tmp61 = Scalar(1.0) * _tmp33;
  const Scalar _tmp62 = Scalar(1.0) * _tmp24;
  const Scalar _tmp63 = (-_tmp42 + _tmp61) / (_tmp40 - _tmp62);
  const Scalar _tmp64 = _tmp51 * _tmp59 - _tmp55 - _tmp63 * (_tmp49 * _tmp55 + _tmp51 * _tmp60);
  const Scalar _tmp65 = -_tmp27;
  const Scalar _tmp66 = _tmp41 + _tmp65;
  const Scalar _tmp67 = _tmp66 + tx + Scalar(1.79662371);
  const Scalar _tmp68 = _tmp19 - _tmp22;
  const Scalar _tmp69 = _tmp12 + _tmp68;
  const Scalar _tmp70 = _tmp69 + ty + Scalar(-4.8333311099999996);
  const Scalar _tmp71 = std::pow(Scalar(std::pow(_tmp67, Scalar(2)) + std::pow(_tmp70, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp72 = _tmp67 * _tmp71;
  const Scalar _tmp73 = _tmp55 * _tmp72;
  const Scalar _tmp74 = -_tmp54 + _tmp57;
  const Scalar _tmp75 = _tmp70 * _tmp71;
  const Scalar _tmp76 = _tmp49 * _tmp72 - _tmp75;
  const Scalar _tmp77 = _tmp50 * _tmp76;
  const Scalar _tmp78 = -_tmp59 * _tmp77 -
                        _tmp63 * (-_tmp49 * _tmp73 - _tmp60 * _tmp77 + _tmp74 * _tmp75) -
                        _tmp72 * _tmp74 + _tmp73;
  const Scalar _tmp79 = Scalar(1.0) / (_tmp78);
  const Scalar _tmp80 = _tmp39 * _tmp72 - _tmp48 * _tmp77 - _tmp66 * _tmp75 + _tmp69 * _tmp72;
  const Scalar _tmp81 = _tmp79 * _tmp80;
  const Scalar _tmp82 = Scalar(1.0) / (_tmp80);
  const Scalar _tmp83 = _tmp78 * _tmp82;
  const Scalar _tmp84 = _tmp64 + _tmp83 * (-_tmp39 + _tmp48 * _tmp51 - _tmp64 * _tmp81);
  const Scalar _tmp85 = _tmp72 * _tmp79;
  const Scalar _tmp86 = _tmp76 * _tmp79;
  const Scalar _tmp87 = -_tmp49 - _tmp84 * _tmp86;
  const Scalar _tmp88 = _tmp46 * _tmp50;
  const Scalar _tmp89 = _tmp32 + _tmp65;
  const Scalar _tmp90 = _tmp89 + tx + Scalar(1.9874742000000001);
  const Scalar _tmp91 = _tmp13 + _tmp68;
  const Scalar _tmp92 = _tmp91 + ty + Scalar(8.3196563700000006);
  const Scalar _tmp93 = std::pow(Scalar(std::pow(_tmp90, Scalar(2)) + std::pow(_tmp92, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp94 = _tmp90 * _tmp93;
  const Scalar _tmp95 = _tmp94 * fh1;
  const Scalar _tmp96 = Scalar(1.0) * _tmp82;
  const Scalar _tmp97 = _tmp92 * _tmp93;
  const Scalar _tmp98 = fh1 * (_tmp89 * _tmp97 - _tmp91 * _tmp94);
  const Scalar _tmp99 = Scalar(1.0) * _tmp50;
  const Scalar _tmp100 = -_tmp59 * _tmp99 + _tmp60 * _tmp63 * _tmp99;
  const Scalar _tmp101 = _tmp100 + _tmp83 * (-_tmp100 * _tmp81 - _tmp48 * _tmp99);
  const Scalar _tmp102 = -_tmp101 * _tmp86 + Scalar(1.0);
  const Scalar _tmp103 = _tmp97 * fh1;
  const Scalar _tmp104 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp105 = _tmp61 + _tmp62 * _tmp63;
  const Scalar _tmp106 = 0;
  const Scalar _tmp107 = _tmp106 * _tmp77;
  const Scalar _tmp108 = _tmp38 * (_tmp106 * _tmp72 - _tmp107 * _tmp46);
  const Scalar _tmp109 = _tmp104 * _tmp106;
  const Scalar _tmp110 = _tmp96 * _tmp98;

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) = 0;
  _res(1, 0) =
      -_tmp108 *
      std::exp(_tmp103 * _tmp38 * (_tmp101 * _tmp85 + _tmp102 * _tmp88) + _tmp104 * _tmp108 +
               _tmp38 * _tmp95 * (_tmp84 * _tmp85 + _tmp87 * _tmp88 + Scalar(1.0)) +
               _tmp38 * _tmp98 * (-_tmp46 * _tmp77 * _tmp96 + _tmp72 * _tmp96));
  _res(2, 0) = -_tmp107 * std::exp(-_tmp102 * _tmp103 * _tmp50 + _tmp109 * _tmp77 +
                                   _tmp110 * _tmp77 - _tmp50 * _tmp87 * _tmp95);
  _res(3, 0) = _tmp106 *
               std::exp(-_tmp101 * _tmp103 * _tmp79 - _tmp109 - _tmp110 - _tmp79 * _tmp84 * _tmp95);

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
