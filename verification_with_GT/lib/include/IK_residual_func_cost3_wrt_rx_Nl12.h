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
 * Symbolic function: IK_residual_func_cost3_wrt_rx_Nl12
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost3WrtRxNl12(
    const Scalar fh1, const Scalar fv1, const Scalar rx, const Scalar ry, const Scalar rz,
    const Scalar p_init0, const Scalar p_init1, const Scalar p_init2, const Scalar rot_init_x,
    const Scalar rot_init_y, const Scalar rot_init_z, const Scalar rot_init_w,
    const Scalar epsilon) {
  // Total ops: 989

  // Unused inputs
  (void)p_init2;
  (void)epsilon;

  // Input arrays

  // Intermediate terms (316)
  const Scalar _tmp0 = std::pow(rx, Scalar(2));
  const Scalar _tmp1 = _tmp0 + std::pow(ry, Scalar(2)) + std::pow(rz, Scalar(2));
  const Scalar _tmp2 = std::sqrt(_tmp1);
  const Scalar _tmp3 = (Scalar(1) / Scalar(2)) * _tmp2;
  const Scalar _tmp4 = std::cos(_tmp3);
  const Scalar _tmp5 = _tmp4 * rot_init_y;
  const Scalar _tmp6 = std::sin(_tmp3);
  const Scalar _tmp7 = _tmp6 / _tmp2;
  const Scalar _tmp8 = _tmp7 * rot_init_w;
  const Scalar _tmp9 = _tmp7 * rot_init_z;
  const Scalar _tmp10 = _tmp9 * rx;
  const Scalar _tmp11 = _tmp7 * rot_init_x;
  const Scalar _tmp12 = _tmp10 - _tmp11 * rz + _tmp5 + _tmp8 * ry;
  const Scalar _tmp13 = _tmp4 * rot_init_x;
  const Scalar _tmp14 = _tmp8 * rx;
  const Scalar _tmp15 = _tmp7 * rot_init_y;
  const Scalar _tmp16 = _tmp13 + _tmp14 + _tmp15 * rz - _tmp9 * ry;
  const Scalar _tmp17 = 2 * _tmp16;
  const Scalar _tmp18 = _tmp12 * _tmp17;
  const Scalar _tmp19 = _tmp4 * rot_init_z;
  const Scalar _tmp20 = _tmp15 * rx;
  const Scalar _tmp21 = _tmp11 * ry + _tmp19 - _tmp20 + _tmp8 * rz;
  const Scalar _tmp22 = _tmp4 * rot_init_w;
  const Scalar _tmp23 = _tmp11 * rx;
  const Scalar _tmp24 = -_tmp15 * ry + _tmp22 - _tmp23 - _tmp9 * rz;
  const Scalar _tmp25 = 2 * _tmp21 * _tmp24;
  const Scalar _tmp26 = Scalar(0.20999999999999999) * _tmp18 + Scalar(0.20999999999999999) * _tmp25;
  const Scalar _tmp27 = -_tmp26;
  const Scalar _tmp28 = -2 * std::pow(_tmp16, Scalar(2));
  const Scalar _tmp29 = -2 * std::pow(_tmp21, Scalar(2));
  const Scalar _tmp30 = Scalar(0.20999999999999999) * _tmp28 +
                        Scalar(0.20999999999999999) * _tmp29 + Scalar(0.20999999999999999);
  const Scalar _tmp31 = 2 * _tmp12;
  const Scalar _tmp32 = _tmp21 * _tmp31;
  const Scalar _tmp33 = _tmp17 * _tmp24;
  const Scalar _tmp34 =
      -Scalar(0.010999999999999999) * _tmp32 + Scalar(0.010999999999999999) * _tmp33;
  const Scalar _tmp35 = _tmp30 + _tmp34;
  const Scalar _tmp36 = _tmp27 + _tmp35;
  const Scalar _tmp37 = _tmp36 + p_init1 + Scalar(-4.8333311099999996);
  const Scalar _tmp38 = Scalar(0.20999999999999999) * _tmp18 - Scalar(0.20999999999999999) * _tmp25;
  const Scalar _tmp39 = _tmp17 * _tmp21;
  const Scalar _tmp40 = _tmp24 * _tmp31;
  const Scalar _tmp41 =
      -Scalar(0.010999999999999999) * _tmp39 - Scalar(0.010999999999999999) * _tmp40;
  const Scalar _tmp42 = 1 - 2 * std::pow(_tmp12, Scalar(2));
  const Scalar _tmp43 = Scalar(0.20999999999999999) * _tmp29 + Scalar(0.20999999999999999) * _tmp42;
  const Scalar _tmp44 = _tmp41 - _tmp43;
  const Scalar _tmp45 = _tmp38 + _tmp44;
  const Scalar _tmp46 = _tmp45 + p_init0 + Scalar(1.79662371);
  const Scalar _tmp47 = std::pow(_tmp37, Scalar(2)) + std::pow(_tmp46, Scalar(2));
  const Scalar _tmp48 = std::pow(_tmp47, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp49 = _tmp46 * _tmp48;
  const Scalar _tmp50 = -_tmp30 + _tmp34;
  const Scalar _tmp51 = _tmp27 + _tmp50;
  const Scalar _tmp52 = _tmp51 + p_init1 + Scalar(8.3196563700000006);
  const Scalar _tmp53 = -_tmp38;
  const Scalar _tmp54 = _tmp44 + _tmp53;
  const Scalar _tmp55 = _tmp54 + p_init0 + Scalar(1.9874742000000001);
  const Scalar _tmp56 = Scalar(1.0) / (_tmp55);
  const Scalar _tmp57 = _tmp52 * _tmp56;
  const Scalar _tmp58 = Scalar(0.20999999999999999) * _tmp39 - Scalar(0.20999999999999999) * _tmp40;
  const Scalar _tmp59 = -_tmp58;
  const Scalar _tmp60 =
      -Scalar(0.010999999999999999) * _tmp28 - Scalar(0.010999999999999999) * _tmp42;
  const Scalar _tmp61 = Scalar(0.20999999999999999) * _tmp32 + Scalar(0.20999999999999999) * _tmp33;
  const Scalar _tmp62 = _tmp60 - _tmp61;
  const Scalar _tmp63 = _tmp59 + _tmp62;
  const Scalar _tmp64 = _tmp49 * _tmp63;
  const Scalar _tmp65 = _tmp59 + _tmp60 + _tmp61;
  const Scalar _tmp66 = _tmp37 * _tmp48;
  const Scalar _tmp67 = _tmp58 + _tmp62;
  const Scalar _tmp68 = _tmp41 + _tmp43;
  const Scalar _tmp69 = _tmp53 + _tmp68;
  const Scalar _tmp70 = _tmp69 + p_init0 + Scalar(-2.5202214700000001);
  const Scalar _tmp71 = _tmp26 + _tmp50;
  const Scalar _tmp72 = _tmp71 + p_init1 + Scalar(8.3888750099999996);
  const Scalar _tmp73 = std::pow(_tmp70, Scalar(2)) + std::pow(_tmp72, Scalar(2));
  const Scalar _tmp74 = std::pow(_tmp73, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp75 = _tmp72 * _tmp74;
  const Scalar _tmp76 = _tmp70 * _tmp74;
  const Scalar _tmp77 = _tmp63 * _tmp76;
  const Scalar _tmp78 = -_tmp57 * _tmp77 + _tmp67 * _tmp75;
  const Scalar _tmp79 = _tmp57 * _tmp76 - _tmp75;
  const Scalar _tmp80 = Scalar(1.0) / (_tmp79);
  const Scalar _tmp81 = _tmp49 * _tmp57 - _tmp66;
  const Scalar _tmp82 = _tmp80 * _tmp81;
  const Scalar _tmp83 = -_tmp57 * _tmp64 + _tmp65 * _tmp66 - _tmp78 * _tmp82;
  const Scalar _tmp84 = Scalar(1.0) * _tmp51;
  const Scalar _tmp85 = _tmp71 - _tmp84;
  const Scalar _tmp86 = Scalar(1.0) / (_tmp85);
  const Scalar _tmp87 = Scalar(1.0) * _tmp54;
  const Scalar _tmp88 = -_tmp69 + _tmp87;
  const Scalar _tmp89 = _tmp86 * _tmp88;
  const Scalar _tmp90 = -_tmp67 * _tmp76 + _tmp77;
  const Scalar _tmp91 = -_tmp49 * _tmp65 + _tmp64 - _tmp82 * _tmp90 - _tmp83 * _tmp89;
  const Scalar _tmp92 = Scalar(1.0) / (_tmp91);
  const Scalar _tmp93 = std::pow(_tmp55, Scalar(2));
  const Scalar _tmp94 = std::pow(_tmp52, Scalar(2)) + _tmp93;
  const Scalar _tmp95 = std::sqrt(_tmp94);
  const Scalar _tmp96 = Scalar(1.0) / (_tmp95);
  const Scalar _tmp97 = _tmp51 * _tmp96;
  const Scalar _tmp98 = _tmp52 * _tmp96;
  const Scalar _tmp99 = _tmp54 * _tmp98 - _tmp55 * _tmp97;
  const Scalar _tmp100 = _tmp56 * _tmp95;
  const Scalar _tmp101 = _tmp100 * _tmp99;
  const Scalar _tmp102 = _tmp101 * _tmp76 - _tmp69 * _tmp75 + _tmp71 * _tmp76;
  const Scalar _tmp103 = Scalar(1.0) * _tmp80;
  const Scalar _tmp104 = _tmp103 * _tmp78;
  const Scalar _tmp105 = -_tmp103 * _tmp90 + _tmp104 * _tmp89;
  const Scalar _tmp106 = _tmp101 * _tmp49 - _tmp102 * _tmp82 + _tmp36 * _tmp49 - _tmp45 * _tmp66;
  const Scalar _tmp107 = _tmp106 * _tmp92;
  const Scalar _tmp108 = -_tmp102 * _tmp103 - _tmp105 * _tmp107;
  const Scalar _tmp109 = Scalar(1.0) / (_tmp106);
  const Scalar _tmp110 = _tmp109 * _tmp91;
  const Scalar _tmp111 = _tmp105 + _tmp108 * _tmp110;
  const Scalar _tmp112 = _tmp111 * _tmp92;
  const Scalar _tmp113 = _tmp81 * _tmp92;
  const Scalar _tmp114 = -_tmp111 * _tmp113 + Scalar(1.0);
  const Scalar _tmp115 = _tmp76 * _tmp80;
  const Scalar _tmp116 = _tmp112 * _tmp49 + _tmp114 * _tmp115;
  const Scalar _tmp117 = _tmp26 + _tmp35;
  const Scalar _tmp118 = _tmp117 + p_init1 + Scalar(-4.7752063900000001);
  const Scalar _tmp119 = _tmp38 + _tmp68;
  const Scalar _tmp120 = _tmp119 + p_init0 + Scalar(-2.71799795);
  const Scalar _tmp121 = std::pow(_tmp118, Scalar(2)) + std::pow(_tmp120, Scalar(2));
  const Scalar _tmp122 = std::pow(_tmp121, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp123 = _tmp118 * _tmp122;
  const Scalar _tmp124 = _tmp123 * fh1;
  const Scalar _tmp125 = _tmp100 * _tmp124;
  const Scalar _tmp126 = Scalar(1.0) * _tmp109;
  const Scalar _tmp127 = _tmp103 * _tmp109;
  const Scalar _tmp128 = _tmp127 * _tmp81;
  const Scalar _tmp129 = _tmp126 * _tmp49 - _tmp128 * _tmp76;
  const Scalar _tmp130 = _tmp120 * _tmp122;
  const Scalar _tmp131 = -_tmp117 * _tmp130 + _tmp119 * _tmp123;
  const Scalar _tmp132 = _tmp131 * fh1;
  const Scalar _tmp133 = _tmp100 * _tmp132;
  const Scalar _tmp134 = _tmp84 * _tmp89 + _tmp87;
  const Scalar _tmp135 = 0;
  const Scalar _tmp136 = _tmp113 * _tmp135;
  const Scalar _tmp137 = _tmp135 * _tmp92;
  const Scalar _tmp138 = -_tmp115 * _tmp136 + _tmp137 * _tmp49;
  const Scalar _tmp139 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp140 = _tmp100 * _tmp139;
  const Scalar _tmp141 = _tmp57 * _tmp63;
  const Scalar _tmp142 = _tmp57 * _tmp80;
  const Scalar _tmp143 = _tmp141 + _tmp142 * _tmp78;
  const Scalar _tmp144 = _tmp142 * _tmp90 - _tmp143 * _tmp89 - _tmp63;
  const Scalar _tmp145 = -_tmp101 + _tmp102 * _tmp142 - _tmp107 * _tmp144;
  const Scalar _tmp146 = _tmp110 * _tmp145 + _tmp144;
  const Scalar _tmp147 = _tmp146 * _tmp92;
  const Scalar _tmp148 = -_tmp147 * _tmp81 - _tmp57;
  const Scalar _tmp149 = _tmp115 * _tmp148 + _tmp147 * _tmp49 + Scalar(1.0);
  const Scalar _tmp150 = _tmp130 * fh1;
  const Scalar _tmp151 = _tmp100 * _tmp150;
  const Scalar _tmp152 = (Scalar(1) / Scalar(2)) / _tmp1;
  const Scalar _tmp153 = _tmp0 * _tmp152;
  const Scalar _tmp154 = _tmp152 * rx;
  const Scalar _tmp155 = _tmp154 * rz;
  const Scalar _tmp156 = _tmp154 * ry;
  const Scalar _tmp157 = _tmp6 / (_tmp1 * std::sqrt(_tmp1));
  const Scalar _tmp158 = _tmp0 * _tmp157;
  const Scalar _tmp159 = _tmp157 * rx;
  const Scalar _tmp160 = _tmp159 * ry;
  const Scalar _tmp161 = _tmp159 * rz;
  const Scalar _tmp162 = -_tmp13 * _tmp155 + _tmp153 * _tmp19 + _tmp156 * _tmp22 -
                         _tmp158 * rot_init_z - _tmp160 * rot_init_w + _tmp161 * rot_init_x -
                         Scalar(1) / Scalar(2) * _tmp20 + _tmp9;
  const Scalar _tmp163 = Scalar(0.41999999999999998) * _tmp16;
  const Scalar _tmp164 = _tmp162 * _tmp163;
  const Scalar _tmp165 = _tmp154 * _tmp19;
  const Scalar _tmp166 = _tmp159 * rot_init_z;
  const Scalar _tmp167 = _tmp153 * _tmp22 + _tmp155 * _tmp5 - _tmp158 * rot_init_w -
                         _tmp161 * rot_init_y - _tmp165 * ry + _tmp166 * ry -
                         Scalar(1) / Scalar(2) * _tmp23 + _tmp8;
  const Scalar _tmp168 = Scalar(0.41999999999999998) * _tmp167;
  const Scalar _tmp169 = _tmp12 * _tmp168;
  const Scalar _tmp170 = -_tmp164 - _tmp169;
  const Scalar _tmp171 = -Scalar(1) / Scalar(2) * _tmp10 + _tmp13 * _tmp156 - _tmp15 -
                         _tmp153 * _tmp5 + _tmp155 * _tmp22 + _tmp158 * rot_init_y -
                         _tmp160 * rot_init_x - _tmp161 * rot_init_w;
  const Scalar _tmp172 = _tmp171 * _tmp21;
  const Scalar _tmp173 = Scalar(0.83999999999999997) * _tmp172;
  const Scalar _tmp174 = -_tmp11 - _tmp13 * _tmp153 - Scalar(1) / Scalar(2) * _tmp14 -
                         _tmp156 * _tmp5 + _tmp158 * rot_init_x + _tmp160 * rot_init_y -
                         _tmp165 * rz + _tmp166 * rz;
  const Scalar _tmp175 = Scalar(0.41999999999999998) * _tmp174;
  const Scalar _tmp176 = _tmp175 * _tmp21;
  const Scalar _tmp177 = Scalar(0.41999999999999998) * _tmp171;
  const Scalar _tmp178 = _tmp177 * _tmp24;
  const Scalar _tmp179 = _tmp176 + _tmp178;
  const Scalar _tmp180 = _tmp173 + _tmp179;
  const Scalar _tmp181 = _tmp12 * _tmp162;
  const Scalar _tmp182 = Scalar(0.83999999999999997) * _tmp181;
  const Scalar _tmp183 = Scalar(0.021999999999999999) * _tmp174;
  const Scalar _tmp184 = Scalar(0.021999999999999999) * _tmp171;
  const Scalar _tmp185 = Scalar(0.021999999999999999) * _tmp21;
  const Scalar _tmp186 = Scalar(0.021999999999999999) * _tmp24;
  const Scalar _tmp187 =
      -_tmp12 * _tmp183 - _tmp16 * _tmp184 - _tmp162 * _tmp186 - _tmp167 * _tmp185;
  const Scalar _tmp188 = _tmp182 + _tmp187;
  const Scalar _tmp189 = _tmp170 + _tmp180 + _tmp188;
  const Scalar _tmp190 = -_tmp176 - _tmp178;
  const Scalar _tmp191 = Scalar(0.83999999999999997) * _tmp16;
  const Scalar _tmp192 = _tmp167 * _tmp191;
  const Scalar _tmp193 =
      -_tmp12 * _tmp184 + _tmp16 * _tmp183 - _tmp162 * _tmp185 + _tmp167 * _tmp186;
  const Scalar _tmp194 = _tmp192 + _tmp193;
  const Scalar _tmp195 = _tmp170 + _tmp173 + _tmp190 + _tmp194;
  const Scalar _tmp196 = _tmp189 * _tmp55 + _tmp195 * _tmp52;
  const Scalar _tmp197 = _tmp196 * _tmp56 * _tmp96;
  const Scalar _tmp198 = _tmp129 * _tmp132;
  const Scalar _tmp199 = _tmp100 * _tmp116;
  const Scalar _tmp200 = -_tmp173;
  const Scalar _tmp201 = -_tmp182 + _tmp187;
  const Scalar _tmp202 = _tmp164 + _tmp169;
  const Scalar _tmp203 = _tmp190 + _tmp202;
  const Scalar _tmp204 = _tmp200 + _tmp201 + _tmp203;
  const Scalar _tmp205 = -_tmp192 + _tmp193;
  const Scalar _tmp206 = _tmp179 + _tmp200 + _tmp202 + _tmp205;
  const Scalar _tmp207 = (Scalar(1) / Scalar(2)) * (2 * _tmp118 * _tmp206 + 2 * _tmp120 * _tmp204) /
                         (_tmp121 * std::sqrt(_tmp121));
  const Scalar _tmp208 = _tmp118 * _tmp207;
  const Scalar _tmp209 = _tmp208 * fh1;
  const Scalar _tmp210 = _tmp170 + _tmp200;
  const Scalar _tmp211 = _tmp179 + _tmp201 + _tmp210;
  const Scalar _tmp212 = _tmp180 + _tmp194 + _tmp202;
  const Scalar _tmp213 =
      (2 * _tmp211 * _tmp70 + 2 * _tmp212 * _tmp72) / (_tmp73 * std::sqrt(_tmp73));
  const Scalar _tmp214 = (Scalar(1) / Scalar(2)) * _tmp213;
  const Scalar _tmp215 = _tmp214 * _tmp70;
  const Scalar _tmp216 = _tmp114 * _tmp80;
  const Scalar _tmp217 = _tmp173 + _tmp188 + _tmp203;
  const Scalar _tmp218 = _tmp190 + _tmp205 + _tmp210;
  const Scalar _tmp219 =
      (2 * _tmp217 * _tmp46 + 2 * _tmp218 * _tmp37) / (_tmp47 * std::sqrt(_tmp47));
  const Scalar _tmp220 = (Scalar(1) / Scalar(2)) * _tmp219;
  const Scalar _tmp221 = _tmp220 * _tmp46;
  const Scalar _tmp222 = _tmp217 * _tmp48;
  const Scalar _tmp223 = _tmp222 * _tmp63;
  const Scalar _tmp224 = _tmp220 * _tmp37;
  const Scalar _tmp225 = _tmp195 * _tmp56;
  const Scalar _tmp226 = _tmp225 * _tmp63;
  const Scalar _tmp227 = _tmp163 * _tmp174;
  const Scalar _tmp228 = _tmp12 * _tmp177;
  const Scalar _tmp229 = Scalar(0.41999999999999998) * _tmp162;
  const Scalar _tmp230 = _tmp21 * _tmp229;
  const Scalar _tmp231 = _tmp168 * _tmp24;
  const Scalar _tmp232 = _tmp227 + _tmp228 + _tmp230 + _tmp231;
  const Scalar _tmp233 = _tmp12 * _tmp175;
  const Scalar _tmp234 = _tmp163 * _tmp171;
  const Scalar _tmp235 = _tmp168 * _tmp21;
  const Scalar _tmp236 = _tmp229 * _tmp24;
  const Scalar _tmp237 = Scalar(0.043999999999999997) * _tmp181;
  const Scalar _tmp238 = Scalar(0.043999999999999997) * _tmp16 * _tmp167;
  const Scalar _tmp239 = _tmp237 + _tmp238;
  const Scalar _tmp240 = _tmp233 - _tmp234 - _tmp235 + _tmp236 + _tmp239;
  const Scalar _tmp241 = _tmp232 + _tmp240;
  const Scalar _tmp242 = _tmp212 * _tmp74;
  const Scalar _tmp243 = _tmp211 * _tmp74;
  const Scalar _tmp244 = -_tmp227 - _tmp228 - _tmp230 - _tmp231;
  const Scalar _tmp245 = -_tmp233 + _tmp234 + _tmp235 - _tmp236;
  const Scalar _tmp246 = _tmp239 + _tmp244 + _tmp245;
  const Scalar _tmp247 = _tmp214 * _tmp72;
  const Scalar _tmp248 = _tmp189 / _tmp93;
  const Scalar _tmp249 = _tmp248 * _tmp52;
  const Scalar _tmp250 = _tmp240 + _tmp244;
  const Scalar _tmp251 = _tmp250 * _tmp57;
  const Scalar _tmp252 = _tmp141 * _tmp215 - _tmp141 * _tmp243 - _tmp226 * _tmp76 +
                         _tmp242 * _tmp67 + _tmp246 * _tmp75 - _tmp247 * _tmp67 + _tmp249 * _tmp77 -
                         _tmp251 * _tmp76;
  const Scalar _tmp253 = _tmp218 * _tmp48;
  const Scalar _tmp254 = -_tmp221 * _tmp57 + _tmp222 * _tmp57 + _tmp224 + _tmp225 * _tmp49 -
                         _tmp249 * _tmp49 - _tmp253;
  const Scalar _tmp255 = _tmp254 * _tmp80;
  const Scalar _tmp256 = (-_tmp215 * _tmp57 + _tmp225 * _tmp76 - _tmp242 + _tmp243 * _tmp57 +
                          _tmp247 - _tmp249 * _tmp76) /
                         std::pow(_tmp79, Scalar(2));
  const Scalar _tmp257 = _tmp256 * _tmp81;
  const Scalar _tmp258 = _tmp256 * _tmp90;
  const Scalar _tmp259 =
      _tmp86 * (Scalar(1.6799999999999999) * _tmp172 + Scalar(1.6799999999999999) * _tmp181);
  const Scalar _tmp260 = _tmp88 *
                         (Scalar(0.83999999999999997) * _tmp12 * _tmp167 + _tmp162 * _tmp191 +
                          Scalar(0.83999999999999997) * _tmp171 * _tmp24 +
                          Scalar(0.83999999999999997) * _tmp174 * _tmp21) /
                         std::pow(_tmp85, Scalar(2));
  const Scalar _tmp261 = -_tmp215 * _tmp63 + _tmp215 * _tmp67 + _tmp243 * _tmp63 -
                         _tmp243 * _tmp67 - _tmp246 * _tmp76 + _tmp250 * _tmp76;
  const Scalar _tmp262 =
      -_tmp221 * _tmp63 + _tmp221 * _tmp65 - _tmp222 * _tmp65 + _tmp223 - _tmp241 * _tmp49 +
      _tmp250 * _tmp49 - _tmp255 * _tmp90 + _tmp258 * _tmp81 - _tmp259 * _tmp83 + _tmp260 * _tmp83 -
      _tmp261 * _tmp82 -
      _tmp89 * (_tmp141 * _tmp221 - _tmp223 * _tmp57 - _tmp224 * _tmp65 - _tmp226 * _tmp49 +
                _tmp241 * _tmp66 + _tmp249 * _tmp64 - _tmp251 * _tmp49 - _tmp252 * _tmp82 +
                _tmp253 * _tmp65 - _tmp255 * _tmp78 + _tmp257 * _tmp78);
  const Scalar _tmp263 = _tmp262 / std::pow(_tmp91, Scalar(2));
  const Scalar _tmp264 = _tmp263 * _tmp49;
  const Scalar _tmp265 = _tmp263 * _tmp81;
  const Scalar _tmp266 = _tmp248 * _tmp95;
  const Scalar _tmp267 = _tmp266 * _tmp99;
  const Scalar _tmp268 = _tmp196 / (_tmp94 * std::sqrt(_tmp94));
  const Scalar _tmp269 = _tmp195 * _tmp96;
  const Scalar _tmp270 =
      _tmp100 * (-_tmp189 * _tmp97 + _tmp189 * _tmp98 + _tmp268 * _tmp51 * _tmp55 -
                 _tmp268 * _tmp52 * _tmp54 + _tmp269 * _tmp54 - _tmp269 * _tmp55);
  const Scalar _tmp271 = _tmp197 * _tmp99;
  const Scalar _tmp272 = -_tmp101 * _tmp215 + _tmp101 * _tmp243 - _tmp211 * _tmp75 -
                         _tmp215 * _tmp71 - _tmp242 * _tmp69 + _tmp242 * _tmp70 + _tmp243 * _tmp71 +
                         _tmp247 * _tmp69 - _tmp267 * _tmp76 + _tmp270 * _tmp76 + _tmp271 * _tmp76;
  const Scalar _tmp273 = -_tmp101 * _tmp221 + _tmp101 * _tmp222 - _tmp102 * _tmp255 +
                         _tmp102 * _tmp257 - _tmp217 * _tmp66 - _tmp221 * _tmp36 +
                         _tmp222 * _tmp36 + _tmp224 * _tmp45 - _tmp253 * _tmp45 + _tmp253 * _tmp46 -
                         _tmp267 * _tmp49 + _tmp270 * _tmp49 + _tmp271 * _tmp49 - _tmp272 * _tmp82;
  const Scalar _tmp274 = _tmp273 / std::pow(_tmp106, Scalar(2));
  const Scalar _tmp275 = _tmp274 * _tmp91;
  const Scalar _tmp276 = _tmp273 * _tmp92;
  const Scalar _tmp277 = _tmp102 * _tmp256;
  const Scalar _tmp278 = _tmp106 * _tmp263;
  const Scalar _tmp279 = _tmp256 * _tmp78;
  const Scalar _tmp280 = _tmp103 * _tmp252 * _tmp89 - _tmp103 * _tmp261 + _tmp104 * _tmp259 -
                         _tmp104 * _tmp260 + Scalar(1.0) * _tmp258 - Scalar(1.0) * _tmp279 * _tmp89;
  const Scalar _tmp281 = _tmp109 * _tmp262;
  const Scalar _tmp282 = -_tmp108 * _tmp275 + _tmp108 * _tmp281 +
                         _tmp110 * (-_tmp103 * _tmp272 - _tmp105 * _tmp276 + _tmp105 * _tmp278 -
                                    _tmp107 * _tmp280 + Scalar(1.0) * _tmp277) +
                         _tmp280;
  const Scalar _tmp283 = _tmp111 * _tmp265 - _tmp112 * _tmp254 - _tmp113 * _tmp282;
  const Scalar _tmp284 = _tmp49 * _tmp92;
  const Scalar _tmp285 = _tmp243 * _tmp80;
  const Scalar _tmp286 = _tmp256 * _tmp76;
  const Scalar _tmp287 = _tmp122 * _tmp206;
  const Scalar _tmp288 = _tmp287 * fh1;
  const Scalar _tmp289 = Scalar(0.5) * _tmp109;
  const Scalar _tmp290 = Scalar(1.0) * _tmp274;
  const Scalar _tmp291 = _tmp103 * _tmp274 * _tmp81;
  const Scalar _tmp292 = _tmp127 * _tmp254;
  const Scalar _tmp293 = _tmp138 * _tmp139;
  const Scalar _tmp294 = _tmp148 * _tmp80;
  const Scalar _tmp295 = _tmp249 * _tmp80;
  const Scalar _tmp296 = _tmp225 * _tmp80;
  const Scalar _tmp297 = _tmp142 * _tmp261 - _tmp143 * _tmp259 + _tmp143 * _tmp260 + _tmp232 -
                         _tmp237 - _tmp238 + _tmp245 - _tmp258 * _tmp57 - _tmp295 * _tmp90 +
                         _tmp296 * _tmp90 -
                         _tmp89 * (_tmp142 * _tmp252 + _tmp226 - _tmp249 * _tmp63 + _tmp251 -
                                   _tmp279 * _tmp57 - _tmp295 * _tmp78 + _tmp296 * _tmp78);
  const Scalar _tmp298 = _tmp110 * (-_tmp102 * _tmp295 + _tmp102 * _tmp296 - _tmp107 * _tmp297 +
                                    _tmp142 * _tmp272 - _tmp144 * _tmp276 + _tmp144 * _tmp278 +
                                    _tmp267 - _tmp270 - _tmp271 - _tmp277 * _tmp57) -
                         _tmp145 * _tmp275 + _tmp145 * _tmp281 + _tmp297;
  const Scalar _tmp299 =
      -_tmp113 * _tmp298 + _tmp146 * _tmp265 - _tmp147 * _tmp254 - _tmp225 + _tmp249;
  const Scalar _tmp300 = _tmp116 * _tmp124;
  const Scalar _tmp301 = _tmp100 * _tmp149;
  const Scalar _tmp302 = _tmp120 * _tmp207;
  const Scalar _tmp303 = _tmp302 * fh1;
  const Scalar _tmp304 = _tmp122 * _tmp204;
  const Scalar _tmp305 = _tmp117 * _tmp302 - _tmp117 * _tmp304 - _tmp119 * _tmp208 +
                         _tmp119 * _tmp287 + _tmp123 * _tmp204 - _tmp130 * _tmp206;
  const Scalar _tmp306 = _tmp305 * fh1;
  const Scalar _tmp307 = _tmp149 * _tmp150;
  const Scalar _tmp308 = _tmp304 * fh1;
  const Scalar _tmp309 = _tmp135 * _tmp139;
  const Scalar _tmp310 = _tmp113 * _tmp309;
  const Scalar _tmp311 = _tmp124 * _tmp80;
  const Scalar _tmp312 = _tmp137 * _tmp139;
  const Scalar _tmp313 = _tmp126 * fh1;
  const Scalar _tmp314 = _tmp131 * _tmp313;
  const Scalar _tmp315 = _tmp263 * _tmp309;

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) = 0;
  _res(1, 0) =
      -(-_tmp100 * _tmp129 * _tmp306 -
        _tmp125 * (-_tmp111 * _tmp264 - _tmp112 * _tmp221 + _tmp112 * _tmp222 + _tmp114 * _tmp285 -
                   _tmp114 * _tmp286 + _tmp115 * _tmp283 - _tmp215 * _tmp216 + _tmp282 * _tmp284) -
        _tmp133 * (_tmp126 * _tmp222 + _tmp126 * _tmp257 * _tmp76 - _tmp128 * _tmp243 +
                   _tmp213 * _tmp289 * _tmp70 * _tmp82 - _tmp219 * _tmp289 * _tmp46 -
                   _tmp290 * _tmp49 + _tmp291 * _tmp76 - _tmp292 * _tmp76) -
        _tmp140 * (_tmp135 * _tmp263 * _tmp76 * _tmp82 - _tmp135 * _tmp264 +
                   _tmp136 * _tmp215 * _tmp80 - _tmp136 * _tmp285 + _tmp136 * _tmp286 -
                   _tmp137 * _tmp221 + _tmp137 * _tmp222 - _tmp137 * _tmp255 * _tmp76) -
        _tmp151 * (_tmp115 * _tmp299 - _tmp146 * _tmp264 - _tmp147 * _tmp221 + _tmp147 * _tmp222 +
                   _tmp148 * _tmp285 - _tmp148 * _tmp286 - _tmp215 * _tmp294 + _tmp284 * _tmp298) -
        _tmp197 * _tmp198 - _tmp197 * _tmp293 - _tmp197 * _tmp300 - _tmp197 * _tmp307 +
        _tmp198 * _tmp266 + _tmp199 * _tmp209 - _tmp199 * _tmp288 + _tmp266 * _tmp293 +
        _tmp266 * _tmp300 + _tmp266 * _tmp307 + _tmp301 * _tmp303 - _tmp301 * _tmp308) *
      std::exp(_tmp116 * _tmp125 + _tmp129 * _tmp133 + _tmp138 * _tmp140 + _tmp149 * _tmp151);
  _res(2, 0) =
      -(-_tmp114 * _tmp124 * _tmp256 - _tmp128 * _tmp306 + _tmp132 * _tmp291 - _tmp132 * _tmp292 -
        _tmp148 * _tmp150 * _tmp256 + _tmp150 * _tmp299 * _tmp80 - _tmp209 * _tmp216 +
        _tmp216 * _tmp288 - _tmp255 * _tmp312 + _tmp256 * _tmp310 + _tmp257 * _tmp314 +
        _tmp283 * _tmp311 - _tmp294 * _tmp303 + _tmp294 * _tmp308 + _tmp315 * _tmp82) *
      std::exp(-_tmp114 * _tmp311 + _tmp128 * _tmp132 - _tmp150 * _tmp294 + _tmp310 * _tmp80);
  _res(3, 0) = -(-_tmp111 * _tmp124 * _tmp263 - _tmp112 * _tmp209 + _tmp112 * _tmp288 +
                 _tmp124 * _tmp282 * _tmp92 - _tmp132 * _tmp290 - _tmp146 * _tmp150 * _tmp263 -
                 _tmp147 * _tmp303 + _tmp147 * _tmp308 + _tmp150 * _tmp298 * _tmp92 +
                 _tmp305 * _tmp313 - _tmp315) *
               std::exp(-_tmp112 * _tmp124 - _tmp147 * _tmp150 - _tmp312 - _tmp314);

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym