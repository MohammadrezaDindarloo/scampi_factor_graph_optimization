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
 * Symbolic function: IK_residual_func_cost1_wrt_ry_Nl10
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost1WrtRyNl10(
    const Scalar fh1, const Scalar fv1, const Scalar rx, const Scalar ry, const Scalar rz,
    const Scalar p_init0, const Scalar p_init1, const Scalar p_init2, const Scalar rot_init_x,
    const Scalar rot_init_y, const Scalar rot_init_z, const Scalar rot_init_w,
    const Scalar epsilon) {
  // Total ops: 1634

  // Unused inputs
  (void)p_init2;
  (void)epsilon;

  // Input arrays

  // Intermediate terms (502)
  const Scalar _tmp0 = Scalar(1.0) / (fh1);
  const Scalar _tmp1 = std::pow(ry, Scalar(2));
  const Scalar _tmp2 = _tmp1 + std::pow(rx, Scalar(2)) + std::pow(rz, Scalar(2));
  const Scalar _tmp3 = std::sqrt(_tmp2);
  const Scalar _tmp4 = (Scalar(1) / Scalar(2)) * _tmp3;
  const Scalar _tmp5 = std::cos(_tmp4);
  const Scalar _tmp6 = _tmp5 * rot_init_y;
  const Scalar _tmp7 = std::sin(_tmp4);
  const Scalar _tmp8 = _tmp7 / _tmp3;
  const Scalar _tmp9 = _tmp8 * rot_init_w;
  const Scalar _tmp10 = _tmp9 * ry;
  const Scalar _tmp11 = _tmp8 * rot_init_z;
  const Scalar _tmp12 = _tmp8 * rot_init_x;
  const Scalar _tmp13 = _tmp10 + _tmp11 * rx - _tmp12 * rz + _tmp6;
  const Scalar _tmp14 = _tmp5 * rot_init_x;
  const Scalar _tmp15 = _tmp11 * ry;
  const Scalar _tmp16 = _tmp8 * rot_init_y;
  const Scalar _tmp17 = _tmp14 - _tmp15 + _tmp16 * rz + _tmp9 * rx;
  const Scalar _tmp18 = 2 * _tmp13 * _tmp17;
  const Scalar _tmp19 = _tmp5 * rot_init_z;
  const Scalar _tmp20 = _tmp12 * ry;
  const Scalar _tmp21 = -_tmp16 * rx + _tmp19 + _tmp20 + _tmp9 * rz;
  const Scalar _tmp22 = _tmp5 * rot_init_w;
  const Scalar _tmp23 = _tmp16 * ry;
  const Scalar _tmp24 = -_tmp11 * rz - _tmp12 * rx + _tmp22 - _tmp23;
  const Scalar _tmp25 = 2 * _tmp24;
  const Scalar _tmp26 = _tmp21 * _tmp25;
  const Scalar _tmp27 = Scalar(0.20999999999999999) * _tmp18 + Scalar(0.20999999999999999) * _tmp26;
  const Scalar _tmp28 = -2 * std::pow(_tmp17, Scalar(2));
  const Scalar _tmp29 = 1 - 2 * std::pow(_tmp21, Scalar(2));
  const Scalar _tmp30 = Scalar(0.20999999999999999) * _tmp28 + Scalar(0.20999999999999999) * _tmp29;
  const Scalar _tmp31 = 2 * _tmp21;
  const Scalar _tmp32 = _tmp13 * _tmp31;
  const Scalar _tmp33 = _tmp17 * _tmp25;
  const Scalar _tmp34 = _tmp32 - _tmp33;
  const Scalar _tmp35 = -Scalar(0.010999999999999999) * _tmp34;
  const Scalar _tmp36 = -_tmp30 + _tmp35;
  const Scalar _tmp37 = _tmp27 + _tmp36;
  const Scalar _tmp38 = _tmp37 + p_init1;
  const Scalar _tmp39 = -_tmp38 + Scalar(-8.3888750099999996);
  const Scalar _tmp40 = Scalar(0.20999999999999999) * _tmp18 - Scalar(0.20999999999999999) * _tmp26;
  const Scalar _tmp41 = -_tmp40;
  const Scalar _tmp42 = _tmp17 * _tmp31;
  const Scalar _tmp43 = _tmp13 * _tmp25;
  const Scalar _tmp44 = _tmp42 + _tmp43;
  const Scalar _tmp45 = -Scalar(0.010999999999999999) * _tmp44;
  const Scalar _tmp46 = -2 * std::pow(_tmp13, Scalar(2));
  const Scalar _tmp47 = Scalar(0.20999999999999999) * _tmp29 + Scalar(0.20999999999999999) * _tmp46;
  const Scalar _tmp48 = _tmp45 + _tmp47;
  const Scalar _tmp49 = _tmp41 + _tmp48;
  const Scalar _tmp50 = _tmp49 + p_init0;
  const Scalar _tmp51 = Scalar(2.5202214700000001) - _tmp50;
  const Scalar _tmp52 =
      std::sqrt(Scalar(std::pow(_tmp39, Scalar(2)) + std::pow(_tmp51, Scalar(2))));
  const Scalar _tmp53 = (Scalar(1) / Scalar(2)) / _tmp2;
  const Scalar _tmp54 = _tmp1 * _tmp53;
  const Scalar _tmp55 = _tmp53 * ry;
  const Scalar _tmp56 = _tmp55 * rz;
  const Scalar _tmp57 = _tmp55 * rx;
  const Scalar _tmp58 = _tmp7 / (_tmp2 * std::sqrt(_tmp2));
  const Scalar _tmp59 = _tmp1 * _tmp58;
  const Scalar _tmp60 = _tmp58 * ry;
  const Scalar _tmp61 = _tmp60 * rx;
  const Scalar _tmp62 = _tmp60 * rz;
  const Scalar _tmp63 = _tmp12 + _tmp14 * _tmp54 - Scalar(1) / Scalar(2) * _tmp15 +
                        _tmp22 * _tmp56 - _tmp57 * _tmp6 - _tmp59 * rot_init_x +
                        _tmp61 * rot_init_y - _tmp62 * rot_init_w;
  const Scalar _tmp64 = Scalar(0.83999999999999997) * _tmp63;
  const Scalar _tmp65 = _tmp21 * _tmp64;
  const Scalar _tmp66 = -_tmp65;
  const Scalar _tmp67 = -_tmp11 - _tmp19 * _tmp54 - Scalar(1) / Scalar(2) * _tmp20 +
                        _tmp22 * _tmp57 + _tmp56 * _tmp6 + _tmp59 * rot_init_z -
                        _tmp61 * rot_init_w - _tmp62 * rot_init_y;
  const Scalar _tmp68 = _tmp17 * _tmp67;
  const Scalar _tmp69 = Scalar(0.83999999999999997) * _tmp68;
  const Scalar _tmp70 = _tmp66 - _tmp69;
  const Scalar _tmp71 = -_tmp14 * _tmp56 + _tmp19 * _tmp57 + _tmp22 * _tmp54 -
                        Scalar(1) / Scalar(2) * _tmp23 - _tmp59 * rot_init_w - _tmp61 * rot_init_z +
                        _tmp62 * rot_init_x + _tmp9;
  const Scalar _tmp72 = Scalar(0.41999999999999998) * _tmp17;
  const Scalar _tmp73 = _tmp71 * _tmp72;
  const Scalar _tmp74 = Scalar(0.41999999999999998) * _tmp67;
  const Scalar _tmp75 = _tmp13 * _tmp74;
  const Scalar _tmp76 = -_tmp73 - _tmp75;
  const Scalar _tmp77 = Scalar(0.41999999999999998) * _tmp24 * _tmp63;
  const Scalar _tmp78 = -Scalar(1) / Scalar(2) * _tmp10 - _tmp14 * _tmp57 - _tmp16 -
                        _tmp19 * _tmp56 - _tmp54 * _tmp6 + _tmp59 * rot_init_y +
                        _tmp61 * rot_init_x + _tmp62 * rot_init_z;
  const Scalar _tmp79 = Scalar(0.41999999999999998) * _tmp78;
  const Scalar _tmp80 = _tmp21 * _tmp79;
  const Scalar _tmp81 = -_tmp77 - _tmp80;
  const Scalar _tmp82 = _tmp76 + _tmp81;
  const Scalar _tmp83 = Scalar(0.021999999999999999) * _tmp24;
  const Scalar _tmp84 = _tmp67 * _tmp83;
  const Scalar _tmp85 = Scalar(0.021999999999999999) * _tmp78;
  const Scalar _tmp86 = _tmp17 * _tmp85;
  const Scalar _tmp87 = _tmp13 * _tmp63;
  const Scalar _tmp88 = Scalar(0.021999999999999999) * _tmp87;
  const Scalar _tmp89 = _tmp21 * _tmp71;
  const Scalar _tmp90 = Scalar(0.021999999999999999) * _tmp89;
  const Scalar _tmp91 = -_tmp84 - _tmp86 + _tmp88 + _tmp90;
  const Scalar _tmp92 = _tmp82 + _tmp91;
  const Scalar _tmp93 = _tmp71 * _tmp83;
  const Scalar _tmp94 = _tmp13 * _tmp85;
  const Scalar _tmp95 = _tmp17 * _tmp63;
  const Scalar _tmp96 = Scalar(0.021999999999999999) * _tmp95;
  const Scalar _tmp97 = _tmp21 * _tmp67;
  const Scalar _tmp98 = Scalar(0.021999999999999999) * _tmp97;
  const Scalar _tmp99 = _tmp93 + _tmp94 + _tmp96 + _tmp98;
  const Scalar _tmp100 = _tmp13 * _tmp71;
  const Scalar _tmp101 = Scalar(0.83999999999999997) * _tmp100;
  const Scalar _tmp102 = _tmp101 + _tmp65;
  const Scalar _tmp103 = _tmp73 + _tmp75;
  const Scalar _tmp104 = _tmp103 + _tmp81;
  const Scalar _tmp105 = _tmp102 + _tmp104;
  const Scalar _tmp106 = _tmp24 * _tmp71;
  const Scalar _tmp107 = Scalar(0.41999999999999998) * _tmp106;
  const Scalar _tmp108 = _tmp13 * _tmp79;
  const Scalar _tmp109 = _tmp63 * _tmp72;
  const Scalar _tmp110 = _tmp21 * _tmp74;
  const Scalar _tmp111 = -_tmp107 - _tmp108 + _tmp109 + _tmp110;
  const Scalar _tmp112 = Scalar(0.043999999999999997) * _tmp100;
  const Scalar _tmp113 = Scalar(0.043999999999999997) * _tmp68;
  const Scalar _tmp114 = _tmp112 + _tmp113;
  const Scalar _tmp115 = _tmp24 * _tmp74;
  const Scalar _tmp116 = _tmp72 * _tmp78;
  const Scalar _tmp117 = Scalar(0.41999999999999998) * _tmp87;
  const Scalar _tmp118 = Scalar(0.41999999999999998) * _tmp89;
  const Scalar _tmp119 = -_tmp115 - _tmp116 - _tmp117 - _tmp118;
  const Scalar _tmp120 = _tmp114 + _tmp119;
  const Scalar _tmp121 = _tmp111 + _tmp120;
  const Scalar _tmp122 = _tmp50 + Scalar(-2.5202214700000001);
  const Scalar _tmp123 = _tmp38 + Scalar(8.3888750099999996);
  const Scalar _tmp124 = std::pow(_tmp122, Scalar(2)) + std::pow(_tmp123, Scalar(2));
  const Scalar _tmp125 = std::pow(_tmp124, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp126 = _tmp122 * _tmp125;
  const Scalar _tmp127 = _tmp126 * fh1;
  const Scalar _tmp128 = -_tmp27;
  const Scalar _tmp129 = _tmp30 + _tmp35;
  const Scalar _tmp130 = _tmp128 + _tmp129;
  const Scalar _tmp131 = Scalar(1.0) * _tmp130;
  const Scalar _tmp132 = -_tmp131;
  const Scalar _tmp133 = _tmp129 + _tmp27;
  const Scalar _tmp134 = _tmp132 + _tmp133;
  const Scalar _tmp135 = _tmp128 + _tmp36;
  const Scalar _tmp136 = _tmp132 + _tmp135;
  const Scalar _tmp137 = Scalar(1.0) / (_tmp136);
  const Scalar _tmp138 = _tmp45 - _tmp47;
  const Scalar _tmp139 = _tmp138 + _tmp40;
  const Scalar _tmp140 = Scalar(1.0) * _tmp139;
  const Scalar _tmp141 = _tmp138 + _tmp41;
  const Scalar _tmp142 = _tmp140 - _tmp141;
  const Scalar _tmp143 = _tmp137 * _tmp142;
  const Scalar _tmp144 = _tmp134 * _tmp143;
  const Scalar _tmp145 = _tmp40 + _tmp48;
  const Scalar _tmp146 = _tmp140 - _tmp144 - _tmp145;
  const Scalar _tmp147 = Scalar(1.0) / (_tmp146);
  const Scalar _tmp148 = Scalar(1.0) * _tmp147;
  const Scalar _tmp149 =
      Scalar(0.20999999999999999) * _tmp32 + Scalar(0.20999999999999999) * _tmp33;
  const Scalar _tmp150 = -Scalar(0.010999999999999999) * _tmp28 -
                         Scalar(0.010999999999999999) * _tmp46 + Scalar(-0.010999999999999999);
  const Scalar _tmp151 =
      Scalar(0.20999999999999999) * _tmp42 - Scalar(0.20999999999999999) * _tmp43;
  const Scalar _tmp152 = _tmp150 - _tmp151;
  const Scalar _tmp153 = _tmp149 + _tmp152;
  const Scalar _tmp154 = -_tmp149;
  const Scalar _tmp155 = _tmp152 + _tmp154;
  const Scalar _tmp156 = _tmp135 + p_init1;
  const Scalar _tmp157 = _tmp156 + Scalar(8.3196563700000006);
  const Scalar _tmp158 = _tmp141 + p_init0;
  const Scalar _tmp159 = _tmp158 + Scalar(1.9874742000000001);
  const Scalar _tmp160 = std::pow(_tmp157, Scalar(2)) + std::pow(_tmp159, Scalar(2));
  const Scalar _tmp161 = std::pow(_tmp160, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp162 = _tmp159 * _tmp161;
  const Scalar _tmp163 = _tmp153 * _tmp162;
  const Scalar _tmp164 = -_tmp155 * _tmp162 + _tmp163;
  const Scalar _tmp165 = _tmp130 + p_init1;
  const Scalar _tmp166 = _tmp165 + Scalar(-4.8333311099999996);
  const Scalar _tmp167 = _tmp139 + p_init0;
  const Scalar _tmp168 = _tmp167 + Scalar(1.79662371);
  const Scalar _tmp169 = Scalar(1.0) / (_tmp168);
  const Scalar _tmp170 = _tmp166 * _tmp169;
  const Scalar _tmp171 = _tmp157 * _tmp161;
  const Scalar _tmp172 = _tmp162 * _tmp170 - _tmp171;
  const Scalar _tmp173 = Scalar(1.0) / (_tmp172);
  const Scalar _tmp174 = _tmp170 * _tmp173;
  const Scalar _tmp175 = _tmp153 * _tmp170;
  const Scalar _tmp176 = _tmp155 * _tmp171 - _tmp163 * _tmp170;
  const Scalar _tmp177 = _tmp174 * _tmp176 + _tmp175;
  const Scalar _tmp178 = -_tmp143 * _tmp177 - _tmp153 + _tmp164 * _tmp174;
  const Scalar _tmp179 = _tmp133 + p_init1;
  const Scalar _tmp180 = _tmp179 + Scalar(-4.7752063900000001);
  const Scalar _tmp181 = _tmp145 + p_init0;
  const Scalar _tmp182 = _tmp181 + Scalar(-2.71799795);
  const Scalar _tmp183 = std::pow(_tmp180, Scalar(2)) + std::pow(_tmp182, Scalar(2));
  const Scalar _tmp184 = std::pow(_tmp183, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp185 = _tmp182 * _tmp184;
  const Scalar _tmp186 = _tmp180 * _tmp184;
  const Scalar _tmp187 = _tmp170 * _tmp185 - _tmp186;
  const Scalar _tmp188 = _tmp173 * _tmp187;
  const Scalar _tmp189 = _tmp153 * _tmp185;
  const Scalar _tmp190 = _tmp150 + _tmp151;
  const Scalar _tmp191 = _tmp149 + _tmp190;
  const Scalar _tmp192 = -_tmp170 * _tmp189 - _tmp176 * _tmp188 + _tmp186 * _tmp191;
  const Scalar _tmp193 = -_tmp143 * _tmp192 - _tmp164 * _tmp188 - _tmp185 * _tmp191 + _tmp189;
  const Scalar _tmp194 = Scalar(1.0) / (_tmp193);
  const Scalar _tmp195 = std::pow(_tmp168, Scalar(2));
  const Scalar _tmp196 = std::pow(_tmp166, Scalar(2)) + _tmp195;
  const Scalar _tmp197 = std::sqrt(_tmp196);
  const Scalar _tmp198 = Scalar(1.0) / (_tmp197);
  const Scalar _tmp199 = _tmp139 * _tmp198;
  const Scalar _tmp200 = _tmp168 * _tmp198;
  const Scalar _tmp201 = -_tmp130 * _tmp200 + _tmp166 * _tmp199;
  const Scalar _tmp202 = _tmp169 * _tmp197;
  const Scalar _tmp203 = _tmp201 * _tmp202;
  const Scalar _tmp204 = _tmp135 * _tmp162 - _tmp141 * _tmp171 + _tmp162 * _tmp203;
  const Scalar _tmp205 =
      _tmp133 * _tmp185 - _tmp145 * _tmp186 + _tmp185 * _tmp203 - _tmp188 * _tmp204;
  const Scalar _tmp206 = _tmp194 * _tmp205;
  const Scalar _tmp207 = _tmp174 * _tmp204 - _tmp178 * _tmp206 - _tmp203;
  const Scalar _tmp208 = Scalar(1.0) / (_tmp205);
  const Scalar _tmp209 = _tmp193 * _tmp208;
  const Scalar _tmp210 = _tmp207 * _tmp209;
  const Scalar _tmp211 = _tmp134 * _tmp147;
  const Scalar _tmp212 = _tmp178 + _tmp210;
  const Scalar _tmp213 = _tmp194 * _tmp212;
  const Scalar _tmp214 = _tmp177 - _tmp192 * _tmp213 + _tmp210 * _tmp211;
  const Scalar _tmp215 = Scalar(1.0) * _tmp137;
  const Scalar _tmp216 = _tmp148 * _tmp210 - _tmp214 * _tmp215;
  const Scalar _tmp217 = Scalar(1.0) * _tmp216;
  const Scalar _tmp218 = _tmp131 * _tmp143 + _tmp140;
  const Scalar _tmp219 = 0;
  const Scalar _tmp220 = _tmp192 * _tmp194;
  const Scalar _tmp221 = _tmp147 * _tmp218;
  const Scalar _tmp222 = _tmp132 - _tmp134 * _tmp221 - _tmp219 * _tmp220;
  const Scalar _tmp223 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp224 = Scalar(1.0) * _tmp223;
  const Scalar _tmp225 = _tmp148 * _tmp209;
  const Scalar _tmp226 = Scalar(1.0) * _tmp208;
  const Scalar _tmp227 = _tmp134 * _tmp225 - _tmp192 * _tmp226;
  const Scalar _tmp228 = -_tmp215 * _tmp227 + _tmp225;
  const Scalar _tmp229 = _tmp125 * _tmp37;
  const Scalar _tmp230 = _tmp123 * _tmp125;
  const Scalar _tmp231 = fh1 * (-_tmp122 * _tmp229 + _tmp230 * _tmp49);
  const Scalar _tmp232 = Scalar(1.0) * _tmp231;
  const Scalar _tmp233 = fh1 * (_tmp154 + _tmp190);
  const Scalar _tmp234 = -_tmp230 * _tmp233 - Scalar(3.29616) * _tmp34 - _tmp37 * fv1;
  const Scalar _tmp235 = _tmp144 * _tmp148 + Scalar(1.0);
  const Scalar _tmp236 = _tmp143 * _tmp148;
  const Scalar _tmp237 = -Scalar(1.0) * _tmp215 * _tmp235 + Scalar(1.0) * _tmp236;
  const Scalar _tmp238 = Scalar(1.0) * _tmp173;
  const Scalar _tmp239 = _tmp142 * _tmp215;
  const Scalar _tmp240 = _tmp173 * _tmp239;
  const Scalar _tmp241 = -_tmp164 * _tmp238 + _tmp176 * _tmp240;
  const Scalar _tmp242 = -_tmp204 * _tmp238 - _tmp206 * _tmp241;
  const Scalar _tmp243 = _tmp209 * _tmp242;
  const Scalar _tmp244 = _tmp176 * _tmp238;
  const Scalar _tmp245 = _tmp241 + _tmp243;
  const Scalar _tmp246 = _tmp211 * _tmp243 - _tmp220 * _tmp245 - _tmp244;
  const Scalar _tmp247 = _tmp148 * _tmp243 - _tmp215 * _tmp246;
  const Scalar _tmp248 = _tmp230 * fh1;
  const Scalar _tmp249 = Scalar(1.0) * _tmp248;
  const Scalar _tmp250 = _tmp125 * _tmp233;
  const Scalar _tmp251 = _tmp122 * _tmp250 + Scalar(3.29616) * _tmp44 + _tmp49 * fv1;
  const Scalar _tmp252 = _tmp137 * _tmp148;
  const Scalar _tmp253 = Scalar(1.0) * _tmp134 * _tmp252 - Scalar(1.0) * _tmp148;
  const Scalar _tmp254 =
      _tmp127 * _tmp217 + _tmp224 * (-_tmp148 * _tmp218 - _tmp215 * _tmp222 + Scalar(1.0)) +
      _tmp228 * _tmp232 + _tmp234 * _tmp237 + _tmp247 * _tmp249 + _tmp251 * _tmp253;
  const Scalar _tmp255 = _tmp194 * _tmp219;
  const Scalar _tmp256 = _tmp162 * _tmp255;
  const Scalar _tmp257 = _tmp185 * _tmp255 - _tmp188 * _tmp256;
  const Scalar _tmp258 = _tmp202 * _tmp223;
  const Scalar _tmp259 = -_tmp170 - _tmp187 * _tmp213;
  const Scalar _tmp260 = _tmp162 * _tmp173;
  const Scalar _tmp261 = _tmp185 * _tmp213 + _tmp259 * _tmp260 + Scalar(1.0);
  const Scalar _tmp262 = _tmp127 * _tmp202;
  const Scalar _tmp263 = _tmp194 * _tmp245;
  const Scalar _tmp264 = -_tmp187 * _tmp263 + Scalar(1.0);
  const Scalar _tmp265 = _tmp185 * _tmp263 + _tmp260 * _tmp264;
  const Scalar _tmp266 = _tmp202 * _tmp248;
  const Scalar _tmp267 = _tmp208 * _tmp238;
  const Scalar _tmp268 = _tmp187 * _tmp267;
  const Scalar _tmp269 = -_tmp162 * _tmp268 + _tmp185 * _tmp226;
  const Scalar _tmp270 = _tmp202 * _tmp231;
  const Scalar _tmp271 =
      -_tmp257 * _tmp258 - _tmp261 * _tmp262 - _tmp265 * _tmp266 - _tmp269 * _tmp270;
  const Scalar _tmp272 = Scalar(1.0) / (_tmp271);
  const Scalar _tmp273 = std::asinh(_tmp254 * _tmp272);
  const Scalar _tmp274 = Scalar(1.0) * _tmp273;
  const Scalar _tmp275 = std::pow(_tmp271, Scalar(-2));
  const Scalar _tmp276 = _tmp77 + _tmp80;
  const Scalar _tmp277 = _tmp103 + _tmp276;
  const Scalar _tmp278 = _tmp84 + _tmp86 - _tmp88 - _tmp90;
  const Scalar _tmp279 = _tmp65 + _tmp69;
  const Scalar _tmp280 = _tmp278 + _tmp279;
  const Scalar _tmp281 = _tmp277 + _tmp280;
  const Scalar _tmp282 = _tmp276 + _tmp76;
  const Scalar _tmp283 = -_tmp101 + _tmp66;
  const Scalar _tmp284 = -_tmp93 - _tmp94 - _tmp96 - _tmp98;
  const Scalar _tmp285 = _tmp283 + _tmp284;
  const Scalar _tmp286 = _tmp282 + _tmp285;
  const Scalar _tmp287 =
      (2 * _tmp122 * _tmp286 + 2 * _tmp123 * _tmp281) / (_tmp124 * std::sqrt(_tmp124));
  const Scalar _tmp288 = Scalar(0.5) * _tmp287 * fh1;
  const Scalar _tmp289 = _tmp125 * _tmp281;
  const Scalar _tmp290 = (Scalar(1) / Scalar(2)) * _tmp287;
  const Scalar _tmp291 = _tmp123 * _tmp290;
  const Scalar _tmp292 = Scalar(6.59232) * _tmp78;
  const Scalar _tmp293 = _tmp121 * fh1;
  const Scalar _tmp294 = _tmp17 * _tmp292 - _tmp230 * _tmp293 - _tmp233 * _tmp289 +
                         _tmp233 * _tmp291 + Scalar(6.59232) * _tmp24 * _tmp67 - _tmp281 * fv1 -
                         Scalar(6.59232) * _tmp87 - Scalar(6.59232) * _tmp89;
  const Scalar _tmp295 = _tmp289 * fh1;
  const Scalar _tmp296 = _tmp122 * _tmp290;
  const Scalar _tmp297 = Scalar(6.59232) * _tmp106 + _tmp126 * _tmp293 + _tmp13 * _tmp292 -
                         _tmp233 * _tmp296 + _tmp250 * _tmp286 + _tmp286 * fv1 +
                         Scalar(6.59232) * _tmp95 + Scalar(6.59232) * _tmp97;
  const Scalar _tmp298 = Scalar(1.6799999999999999) * _tmp21 * _tmp63;
  const Scalar _tmp299 =
      (_tmp298 + Scalar(1.6799999999999999) * _tmp68) / std::pow(_tmp136, Scalar(2));
  const Scalar _tmp300 = _tmp134 * _tmp142;
  const Scalar _tmp301 = _tmp299 * _tmp300;
  const Scalar _tmp302 = _tmp24 * _tmp64;
  const Scalar _tmp303 = Scalar(0.83999999999999997) * _tmp21 * _tmp78;
  const Scalar _tmp304 =
      Scalar(0.83999999999999997) * _tmp13 * _tmp67 + Scalar(0.83999999999999997) * _tmp17 * _tmp71;
  const Scalar _tmp305 = _tmp302 + _tmp303 + _tmp304;
  const Scalar _tmp306 = _tmp143 * _tmp305;
  const Scalar _tmp307 = -_tmp302 - _tmp303 + _tmp304;
  const Scalar _tmp308 = _tmp137 * _tmp307;
  const Scalar _tmp309 = _tmp134 * _tmp308;
  const Scalar _tmp310 =
      (Scalar(1.6799999999999999) * _tmp100 + _tmp298 + _tmp301 - _tmp306 - _tmp309) /
      std::pow(_tmp146, Scalar(2));
  const Scalar _tmp311 = _tmp134 * _tmp310;
  const Scalar _tmp312 = _tmp215 * _tmp311;
  const Scalar _tmp313 = _tmp134 * _tmp299;
  const Scalar _tmp314 = Scalar(1.0) * _tmp310;
  const Scalar _tmp315 = fh1 * (-_tmp122 * _tmp289 - _tmp229 * _tmp286 + _tmp230 * _tmp286 +
                                _tmp289 * _tmp49 - _tmp291 * _tmp49 + _tmp296 * _tmp37);
  const Scalar _tmp316 =
      -_tmp148 * _tmp301 + _tmp148 * _tmp306 + _tmp148 * _tmp309 - _tmp215 * _tmp300 * _tmp310;
  const Scalar _tmp317 = _tmp142 * _tmp299;
  const Scalar _tmp318 = _tmp148 * _tmp308;
  const Scalar _tmp319 = _tmp239 * _tmp310;
  const Scalar _tmp320 = Scalar(1.0) * _tmp299;
  const Scalar _tmp321 = _tmp125 * _tmp286;
  const Scalar _tmp322 = _tmp321 * fh1;
  const Scalar _tmp323 = _tmp105 + _tmp284;
  const Scalar _tmp324 = _tmp278 + _tmp70;
  const Scalar _tmp325 = _tmp324 + _tmp82;
  const Scalar _tmp326 = _tmp166 * _tmp325 + _tmp168 * _tmp323;
  const Scalar _tmp327 = _tmp169 * _tmp198 * _tmp326;
  const Scalar _tmp328 = _tmp201 * _tmp327;
  const Scalar _tmp329 = Scalar(1.0) / (_tmp195);
  const Scalar _tmp330 = _tmp197 * _tmp323 * _tmp329;
  const Scalar _tmp331 = _tmp201 * _tmp330;
  const Scalar _tmp332 = _tmp277 + _tmp324;
  const Scalar _tmp333 = _tmp104 + _tmp285;
  const Scalar _tmp334 =
      (2 * _tmp180 * _tmp332 + 2 * _tmp182 * _tmp333) / (_tmp183 * std::sqrt(_tmp183));
  const Scalar _tmp335 = (Scalar(1) / Scalar(2)) * _tmp334;
  const Scalar _tmp336 = _tmp182 * _tmp335;
  const Scalar _tmp337 = _tmp184 * _tmp333;
  const Scalar _tmp338 = _tmp184 * _tmp332;
  const Scalar _tmp339 = _tmp102 + _tmp282;
  const Scalar _tmp340 = _tmp284 + _tmp339;
  const Scalar _tmp341 = _tmp280 + _tmp82;
  const Scalar _tmp342 =
      (2 * _tmp157 * _tmp341 + 2 * _tmp159 * _tmp340) / (_tmp160 * std::sqrt(_tmp160));
  const Scalar _tmp343 = (Scalar(1) / Scalar(2)) * _tmp342;
  const Scalar _tmp344 = _tmp159 * _tmp343;
  const Scalar _tmp345 = _tmp157 * _tmp343;
  const Scalar _tmp346 = _tmp169 * _tmp325;
  const Scalar _tmp347 = _tmp161 * _tmp341;
  const Scalar _tmp348 = _tmp166 * _tmp323;
  const Scalar _tmp349 = _tmp329 * _tmp348;
  const Scalar _tmp350 = _tmp161 * _tmp340;
  const Scalar _tmp351 = (_tmp162 * _tmp346 - _tmp162 * _tmp349 - _tmp170 * _tmp344 +
                          _tmp170 * _tmp350 + _tmp345 - _tmp347) /
                         std::pow(_tmp172, Scalar(2));
  const Scalar _tmp352 = _tmp204 * _tmp351;
  const Scalar _tmp353 = _tmp326 / (_tmp196 * std::sqrt(_tmp196));
  const Scalar _tmp354 = _tmp202 * (_tmp130 * _tmp168 * _tmp353 - _tmp130 * _tmp198 * _tmp323 -
                                    _tmp139 * _tmp166 * _tmp353 + _tmp198 * _tmp348 +
                                    _tmp199 * _tmp325 - _tmp200 * _tmp325);
  const Scalar _tmp355 = -_tmp135 * _tmp344 + _tmp135 * _tmp350 + _tmp141 * _tmp345 -
                         _tmp141 * _tmp347 + _tmp159 * _tmp347 + _tmp162 * _tmp328 -
                         _tmp162 * _tmp331 + _tmp162 * _tmp354 - _tmp171 * _tmp340 -
                         _tmp203 * _tmp344 + _tmp203 * _tmp350;
  const Scalar _tmp356 = _tmp180 * _tmp335;
  const Scalar _tmp357 = -_tmp170 * _tmp336 + _tmp170 * _tmp337 + _tmp185 * _tmp346 -
                         _tmp185 * _tmp349 - _tmp338 + _tmp356;
  const Scalar _tmp358 = _tmp173 * _tmp357;
  const Scalar _tmp359 = -_tmp133 * _tmp336 + _tmp133 * _tmp337 - _tmp145 * _tmp338 +
                         _tmp145 * _tmp356 + _tmp182 * _tmp338 + _tmp185 * _tmp328 -
                         _tmp185 * _tmp331 + _tmp185 * _tmp354 - _tmp186 * _tmp333 +
                         _tmp187 * _tmp352 - _tmp188 * _tmp355 - _tmp203 * _tmp336 +
                         _tmp203 * _tmp337 - _tmp204 * _tmp358;
  const Scalar _tmp360 = _tmp359 / std::pow(_tmp205, Scalar(2));
  const Scalar _tmp361 = _tmp193 * _tmp360;
  const Scalar _tmp362 = _tmp148 * _tmp361;
  const Scalar _tmp363 = _tmp114 + _tmp115 + _tmp116 + _tmp117 + _tmp118;
  const Scalar _tmp364 = _tmp111 + _tmp363;
  const Scalar _tmp365 = _tmp107 + _tmp108 - _tmp109 - _tmp110;
  const Scalar _tmp366 = _tmp363 + _tmp365;
  const Scalar _tmp367 = _tmp185 * _tmp366;
  const Scalar _tmp368 = _tmp187 * _tmp351;
  const Scalar _tmp369 = _tmp153 * _tmp346;
  const Scalar _tmp370 = _tmp162 * _tmp366;
  const Scalar _tmp371 = _tmp120 + _tmp365;
  const Scalar _tmp372 = _tmp153 * _tmp350;
  const Scalar _tmp373 = -_tmp155 * _tmp345 + _tmp155 * _tmp347 - _tmp163 * _tmp346 +
                         _tmp163 * _tmp349 - _tmp170 * _tmp370 - _tmp170 * _tmp372 +
                         _tmp171 * _tmp371 + _tmp175 * _tmp344;
  const Scalar _tmp374 = -_tmp170 * _tmp367 + _tmp175 * _tmp336 - _tmp175 * _tmp337 -
                         _tmp176 * _tmp358 + _tmp176 * _tmp368 - _tmp185 * _tmp369 +
                         _tmp186 * _tmp364 - _tmp188 * _tmp373 + _tmp189 * _tmp349 +
                         _tmp191 * _tmp338 - _tmp191 * _tmp356;
  const Scalar _tmp375 = -_tmp153 * _tmp344 + _tmp155 * _tmp344 - _tmp155 * _tmp350 -
                         _tmp162 * _tmp371 + _tmp370 + _tmp372;
  const Scalar _tmp376 = -_tmp143 * _tmp374 - _tmp153 * _tmp336 + _tmp153 * _tmp337 -
                         _tmp164 * _tmp358 + _tmp164 * _tmp368 - _tmp185 * _tmp364 -
                         _tmp188 * _tmp375 + _tmp191 * _tmp336 - _tmp191 * _tmp337 -
                         _tmp192 * _tmp308 + _tmp192 * _tmp317 + _tmp367;
  const Scalar _tmp377 = _tmp208 * _tmp376;
  const Scalar _tmp378 = _tmp148 * _tmp377;
  const Scalar _tmp379 = Scalar(1.0) * _tmp360;
  const Scalar _tmp380 = -_tmp134 * _tmp362 + _tmp134 * _tmp378 + _tmp192 * _tmp379 -
                         _tmp193 * _tmp226 * _tmp311 + _tmp225 * _tmp305 - _tmp226 * _tmp374;
  const Scalar _tmp381 = _tmp226 * _tmp310;
  const Scalar _tmp382 = _tmp193 * _tmp381;
  const Scalar _tmp383 = _tmp131 * _tmp308 - _tmp131 * _tmp317 + _tmp239 * _tmp325 + _tmp323;
  const Scalar _tmp384 = _tmp147 * _tmp383;
  const Scalar _tmp385 = _tmp218 * _tmp310;
  const Scalar _tmp386 = _tmp376 / std::pow(_tmp193, Scalar(2));
  const Scalar _tmp387 = _tmp192 * _tmp386;
  const Scalar _tmp388 = _tmp277 + _tmp91;
  const Scalar _tmp389 = _tmp279 + _tmp388;
  const Scalar _tmp390 = -_tmp134 * _tmp384 + _tmp134 * _tmp385 + _tmp219 * _tmp387 -
                         _tmp221 * _tmp305 - _tmp255 * _tmp374 + _tmp389;
  const Scalar _tmp391 = _tmp242 * _tmp377;
  const Scalar _tmp392 = _tmp193 * _tmp242;
  const Scalar _tmp393 = _tmp360 * _tmp392;
  const Scalar _tmp394 = _tmp147 * _tmp305;
  const Scalar _tmp395 = _tmp205 * _tmp386;
  const Scalar _tmp396 = Scalar(1.0) * _tmp351;
  const Scalar _tmp397 = _tmp164 * _tmp396 + _tmp173 * _tmp176 * _tmp215 * _tmp307 -
                         _tmp176 * _tmp239 * _tmp351 - _tmp238 * _tmp375 + _tmp240 * _tmp373 -
                         _tmp244 * _tmp317;
  const Scalar _tmp398 = _tmp194 * _tmp359;
  const Scalar _tmp399 = _tmp209 * (-_tmp206 * _tmp397 - _tmp238 * _tmp355 + _tmp241 * _tmp395 -
                                    _tmp241 * _tmp398 + Scalar(1.0) * _tmp352);
  const Scalar _tmp400 = _tmp391 - _tmp393 + _tmp397 + _tmp399;
  const Scalar _tmp401 = _tmp176 * _tmp396 + _tmp211 * _tmp391 - _tmp211 * _tmp393 +
                         _tmp211 * _tmp399 - _tmp220 * _tmp400 - _tmp238 * _tmp373 -
                         _tmp243 * _tmp311 + _tmp243 * _tmp394 + _tmp245 * _tmp387 -
                         _tmp263 * _tmp374;
  const Scalar _tmp402 = _tmp207 * _tmp361;
  const Scalar _tmp403 = _tmp207 * _tmp377;
  const Scalar _tmp404 = _tmp173 * _tmp346;
  const Scalar _tmp405 = _tmp173 * _tmp349;
  const Scalar _tmp406 = _tmp170 * _tmp351;
  const Scalar _tmp407 = -_tmp153 * _tmp349 + _tmp170 * _tmp366 + _tmp174 * _tmp373 +
                         _tmp176 * _tmp404 - _tmp176 * _tmp405 - _tmp176 * _tmp406 + _tmp369;
  const Scalar _tmp408 = _tmp111 - _tmp112 - _tmp113 + _tmp119 - _tmp143 * _tmp407 +
                         _tmp164 * _tmp404 - _tmp164 * _tmp405 - _tmp164 * _tmp406 +
                         _tmp174 * _tmp375 - _tmp177 * _tmp308 + _tmp177 * _tmp317;
  const Scalar _tmp409 = _tmp209 * (-_tmp170 * _tmp352 + _tmp174 * _tmp355 + _tmp178 * _tmp395 -
                                    _tmp178 * _tmp398 + _tmp204 * _tmp404 - _tmp204 * _tmp405 -
                                    _tmp206 * _tmp408 - _tmp328 + _tmp331 - _tmp354);
  const Scalar _tmp410 = -_tmp402 + _tmp403 + _tmp408 + _tmp409;
  const Scalar _tmp411 = -_tmp210 * _tmp311 + _tmp210 * _tmp394 - _tmp211 * _tmp402 +
                         _tmp211 * _tmp403 + _tmp211 * _tmp409 + _tmp212 * _tmp387 -
                         _tmp213 * _tmp374 - _tmp220 * _tmp410 + _tmp407;
  const Scalar _tmp412 = _tmp202 * _tmp261;
  const Scalar _tmp413 = _tmp296 * fh1;
  const Scalar _tmp414 = _tmp127 * _tmp261;
  const Scalar _tmp415 = _tmp202 * _tmp265;
  const Scalar _tmp416 = _tmp173 * _tmp259;
  const Scalar _tmp417 = _tmp187 * _tmp194;
  const Scalar _tmp418 = _tmp187 * _tmp386;
  const Scalar _tmp419 =
      _tmp212 * _tmp418 - _tmp213 * _tmp357 - _tmp346 + _tmp349 - _tmp410 * _tmp417;
  const Scalar _tmp420 = _tmp185 * _tmp386;
  const Scalar _tmp421 = _tmp185 * _tmp194;
  const Scalar _tmp422 = _tmp173 * _tmp350;
  const Scalar _tmp423 = _tmp162 * _tmp351;
  const Scalar _tmp424 = _tmp248 * _tmp265;
  const Scalar _tmp425 = _tmp188 * _tmp255;
  const Scalar _tmp426 = _tmp219 * _tmp386;
  const Scalar _tmp427 = _tmp219 * _tmp417;
  const Scalar _tmp428 = _tmp223 * _tmp257;
  const Scalar _tmp429 = _tmp291 * fh1;
  const Scalar _tmp430 = _tmp231 * _tmp269;
  const Scalar _tmp431 = Scalar(0.5) * _tmp208;
  const Scalar _tmp432 = _tmp187 * _tmp238 * _tmp360;
  const Scalar _tmp433 = _tmp267 * _tmp357;
  const Scalar _tmp434 = _tmp245 * _tmp418 - _tmp263 * _tmp357 - _tmp400 * _tmp417;
  const Scalar _tmp435 = _tmp173 * _tmp264;
  const Scalar _tmp436 =
      -_tmp202 * _tmp269 * _tmp315 -
      _tmp258 *
          (_tmp162 * _tmp188 * _tmp426 - _tmp185 * _tmp426 - _tmp255 * _tmp336 + _tmp255 * _tmp337 -
           _tmp256 * _tmp358 + _tmp344 * _tmp425 - _tmp350 * _tmp425 + _tmp423 * _tmp427) -
      _tmp262 * (-_tmp212 * _tmp420 - _tmp213 * _tmp336 + _tmp213 * _tmp337 + _tmp259 * _tmp422 -
                 _tmp259 * _tmp423 + _tmp260 * _tmp419 - _tmp344 * _tmp416 + _tmp410 * _tmp421) -
      _tmp266 * (-_tmp245 * _tmp420 + _tmp260 * _tmp434 - _tmp263 * _tmp336 + _tmp263 * _tmp337 +
                 _tmp264 * _tmp422 - _tmp264 * _tmp423 - _tmp344 * _tmp435 + _tmp400 * _tmp421) -
      _tmp270 * (_tmp159 * _tmp188 * _tmp342 * _tmp431 + _tmp162 * _tmp226 * _tmp368 +
                 _tmp162 * _tmp432 - _tmp162 * _tmp433 - _tmp182 * _tmp334 * _tmp431 -
                 _tmp185 * _tmp379 + _tmp226 * _tmp337 - _tmp268 * _tmp350) -
      _tmp295 * _tmp415 - _tmp322 * _tmp412 - _tmp327 * _tmp414 - _tmp327 * _tmp424 -
      _tmp327 * _tmp428 - _tmp327 * _tmp430 + _tmp330 * _tmp414 + _tmp330 * _tmp424 +
      _tmp330 * _tmp428 + _tmp330 * _tmp430 + _tmp412 * _tmp413 + _tmp415 * _tmp429;
  const Scalar _tmp437 = _tmp275 * _tmp436;
  const Scalar _tmp438 =
      (-_tmp254 * _tmp437 +
       _tmp272 *
           (-_tmp122 * _tmp216 * _tmp288 - _tmp123 * _tmp247 * _tmp288 +
            Scalar(1.0) * _tmp127 *
                (-_tmp148 * _tmp402 + _tmp148 * _tmp403 + _tmp148 * _tmp409 - _tmp207 * _tmp382 +
                 _tmp214 * _tmp320 - _tmp215 * _tmp411) +
            _tmp217 * _tmp322 +
            _tmp224 *
                (-_tmp148 * _tmp383 - _tmp215 * _tmp390 + _tmp218 * _tmp314 + _tmp222 * _tmp320) +
            Scalar(1.0) * _tmp228 * _tmp315 +
            _tmp232 * (-_tmp215 * _tmp380 + _tmp227 * _tmp320 - _tmp362 + _tmp378 - _tmp382) +
            Scalar(1.0) * _tmp234 *
                (-_tmp148 * _tmp317 - _tmp215 * _tmp316 + _tmp235 * _tmp320 + _tmp318 - _tmp319) +
            _tmp237 * _tmp294 + Scalar(1.0) * _tmp247 * _tmp295 +
            _tmp249 * (_tmp148 * _tmp391 - _tmp148 * _tmp393 + _tmp148 * _tmp399 -
                       _tmp215 * _tmp401 + _tmp246 * _tmp320 - _tmp381 * _tmp392) +
            Scalar(1.0) * _tmp251 * (-_tmp148 * _tmp313 + _tmp252 * _tmp305 - _tmp312 + _tmp314) +
            _tmp253 * _tmp297)) /
      std::sqrt(Scalar(std::pow(_tmp254, Scalar(2)) * _tmp275 + 1));
  const Scalar _tmp439 = Scalar(9.6622558468725703) * _tmp271;
  const Scalar _tmp440 = Scalar(4.8333311099999996) - _tmp165;
  const Scalar _tmp441 = -_tmp167 + Scalar(-1.79662371);
  const Scalar _tmp442 =
      std::sqrt(Scalar(std::pow(_tmp440, Scalar(2)) + std::pow(_tmp441, Scalar(2))));
  const Scalar _tmp443 = -_tmp273 * _tmp439 - _tmp442;
  const Scalar _tmp444 = Scalar(0.1034955) * _tmp272;
  const Scalar _tmp445 = _tmp443 * _tmp444;
  const Scalar _tmp446 = Scalar(9.6622558468725703) * _tmp436;
  const Scalar _tmp447 = _tmp283 + _tmp99;
  const Scalar _tmp448 = _tmp223 * _tmp255;
  const Scalar _tmp449 = _tmp127 * _tmp173;
  const Scalar _tmp450 =
      -_tmp188 * _tmp448 - _tmp231 * _tmp268 + _tmp248 * _tmp435 + _tmp259 * _tmp449;
  const Scalar _tmp451 = Scalar(1.0) / (_tmp450);
  const Scalar _tmp452 = _tmp137 * _tmp223;
  const Scalar _tmp453 = _tmp137 * _tmp235;
  const Scalar _tmp454 = _tmp137 * fh1;
  const Scalar _tmp455 = _tmp246 * _tmp454;
  const Scalar _tmp456 = _tmp127 * _tmp137;
  const Scalar _tmp457 = _tmp148 * _tmp251;
  const Scalar _tmp458 = _tmp137 * _tmp457;
  const Scalar _tmp459 = _tmp137 * _tmp231;
  const Scalar _tmp460 = -_tmp134 * _tmp458 + _tmp214 * _tmp456 + _tmp222 * _tmp452 +
                         _tmp227 * _tmp459 + _tmp230 * _tmp455 + _tmp234 * _tmp453;
  const Scalar _tmp461 = std::asinh(_tmp451 * _tmp460);
  const Scalar _tmp462 = Scalar(1.0) * _tmp461;
  const Scalar _tmp463 = -_tmp156 + Scalar(-8.3196563700000006);
  const Scalar _tmp464 = -_tmp158 + Scalar(-1.9874742000000001);
  const Scalar _tmp465 =
      std::sqrt(Scalar(std::pow(_tmp463, Scalar(2)) + std::pow(_tmp464, Scalar(2))));
  const Scalar _tmp466 = Scalar(9.6622558468725703) * _tmp450;
  const Scalar _tmp467 = -_tmp461 * _tmp466 - _tmp465;
  const Scalar _tmp468 = Scalar(0.1034955) * _tmp451;
  const Scalar _tmp469 = _tmp467 * _tmp468;
  const Scalar _tmp470 = _tmp226 * _tmp231;
  const Scalar _tmp471 = _tmp223 * _tmp426;
  const Scalar _tmp472 = -_tmp127 * _tmp259 * _tmp351 + _tmp173 * _tmp248 * _tmp434 +
                         _tmp188 * _tmp471 + _tmp223 * _tmp351 * _tmp427 + _tmp231 * _tmp432 -
                         _tmp231 * _tmp433 - _tmp248 * _tmp264 * _tmp351 - _tmp268 * _tmp315 +
                         _tmp295 * _tmp435 + _tmp322 * _tmp416 - _tmp358 * _tmp448 +
                         _tmp368 * _tmp470 - _tmp413 * _tmp416 + _tmp419 * _tmp449 -
                         _tmp429 * _tmp435;
  const Scalar _tmp473 = Scalar(9.6622558468725703) * _tmp472;
  const Scalar _tmp474 = std::pow(_tmp450, Scalar(-2));
  const Scalar _tmp475 = _tmp472 * _tmp474;
  const Scalar _tmp476 = _tmp148 * _tmp297;
  const Scalar _tmp477 = _tmp214 * _tmp454;
  const Scalar _tmp478 = _tmp234 * _tmp299;
  const Scalar _tmp479 =
      (_tmp451 * (-_tmp127 * _tmp214 * _tmp299 - _tmp134 * _tmp137 * _tmp476 +
                  _tmp137 * _tmp227 * _tmp315 + _tmp137 * _tmp234 * _tmp316 -
                  _tmp222 * _tmp223 * _tmp299 - _tmp227 * _tmp231 * _tmp299 +
                  _tmp230 * _tmp401 * _tmp454 - _tmp235 * _tmp478 - _tmp246 * _tmp248 * _tmp299 +
                  _tmp251 * _tmp312 + _tmp289 * _tmp455 - _tmp291 * _tmp455 + _tmp294 * _tmp453 -
                  _tmp296 * _tmp477 - _tmp305 * _tmp458 + _tmp313 * _tmp457 + _tmp321 * _tmp477 +
                  _tmp380 * _tmp459 + _tmp390 * _tmp452 + _tmp411 * _tmp456) -
       _tmp460 * _tmp475) /
      std::sqrt(Scalar(std::pow(_tmp460, Scalar(2)) * _tmp474 + 1));
  const Scalar _tmp480 = _tmp127 * _tmp213 + _tmp248 * _tmp263 + _tmp448 + _tmp470;
  const Scalar _tmp481 = Scalar(1.0) / (_tmp480);
  const Scalar _tmp482 = _tmp147 * _tmp210;
  const Scalar _tmp483 = _tmp147 * _tmp248;
  const Scalar _tmp484 = -_tmp127 * _tmp482 + _tmp221 * _tmp223 - _tmp225 * _tmp231 -
                         _tmp234 * _tmp236 - _tmp243 * _tmp483 + _tmp457;
  const Scalar _tmp485 = std::asinh(_tmp481 * _tmp484);
  const Scalar _tmp486 = Scalar(9.6622558468725703) * _tmp480;
  const Scalar _tmp487 = Scalar(4.7752063900000001) - _tmp179;
  const Scalar _tmp488 = Scalar(2.71799795) - _tmp181;
  const Scalar _tmp489 =
      std::sqrt(Scalar(std::pow(_tmp487, Scalar(2)) + std::pow(_tmp488, Scalar(2))));
  const Scalar _tmp490 = -_tmp485 * _tmp486 - _tmp489;
  const Scalar _tmp491 = Scalar(0.1034955) * _tmp481;
  const Scalar _tmp492 = _tmp490 * _tmp491;
  const Scalar _tmp493 = std::pow(_tmp480, Scalar(-2));
  const Scalar _tmp494 = _tmp213 * fh1;
  const Scalar _tmp495 = _tmp127 * _tmp194 * _tmp410 - _tmp127 * _tmp212 * _tmp386 +
                         _tmp194 * _tmp248 * _tmp400 + _tmp226 * _tmp315 - _tmp232 * _tmp360 -
                         _tmp245 * _tmp248 * _tmp386 + _tmp263 * _tmp295 - _tmp263 * _tmp429 -
                         _tmp296 * _tmp494 + _tmp321 * _tmp494 - _tmp471;
  const Scalar _tmp496 = _tmp493 * _tmp495;
  const Scalar _tmp497 = _tmp127 * _tmp147;
  const Scalar _tmp498 = _tmp147 * _tmp243;
  const Scalar _tmp499 =
      (_tmp481 * (_tmp127 * _tmp210 * _tmp310 + _tmp142 * _tmp148 * _tmp478 +
                  _tmp193 * _tmp310 * _tmp470 + _tmp223 * _tmp384 - _tmp223 * _tmp385 -
                  _tmp225 * _tmp315 + _tmp231 * _tmp362 - _tmp231 * _tmp378 - _tmp234 * _tmp318 +
                  _tmp234 * _tmp319 - _tmp236 * _tmp294 + _tmp243 * _tmp248 * _tmp310 -
                  _tmp251 * _tmp314 - _tmp295 * _tmp498 - _tmp322 * _tmp482 - _tmp391 * _tmp483 +
                  _tmp393 * _tmp483 - _tmp399 * _tmp483 + _tmp402 * _tmp497 - _tmp403 * _tmp497 -
                  _tmp409 * _tmp497 + _tmp413 * _tmp482 + _tmp429 * _tmp498 + _tmp476) -
       _tmp484 * _tmp496) /
      std::sqrt(Scalar(std::pow(_tmp484, Scalar(2)) * _tmp493 + 1));
  const Scalar _tmp500 = Scalar(9.6622558468725703) * _tmp495;
  const Scalar _tmp501 = Scalar(1.0) * _tmp485;

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) =
      _tmp121 -
      Scalar(0.5) * (2 * _tmp39 * (_tmp70 + _tmp92) + 2 * _tmp51 * (_tmp105 + _tmp99)) *
          std::sinh(Scalar(0.1034955) * _tmp0 *
                    (-_tmp52 - Scalar(9.6622558468725703) * fh1 * std::asinh(_tmp0 * fv1))) /
          _tmp52;
  _res(1, 0) =
      _tmp366 -
      _tmp439 *
          (-Scalar(0.86625939559540499) * _tmp437 + Scalar(1.0) * _tmp438 * std::sinh(_tmp274) -
           (-Scalar(0.1034955) * _tmp437 * _tmp443 +
            _tmp444 * (-_tmp273 * _tmp446 - _tmp438 * _tmp439 -
                       Scalar(1) / Scalar(2) *
                           (2 * _tmp389 * _tmp440 + 2 * _tmp441 * (_tmp282 + _tmp447)) / _tmp442)) *
               std::sinh(_tmp445)) -
      _tmp446 * (Scalar(0.86625939559540499) * _tmp272 + std::cosh(_tmp274) - std::cosh(_tmp445));
  _res(2, 0) =
      _tmp371 -
      _tmp466 *
          (-Scalar(0.87679799772039002) * _tmp475 + Scalar(1.0) * _tmp479 * std::sinh(_tmp462) -
           (-Scalar(0.1034955) * _tmp467 * _tmp475 +
            _tmp468 * (-_tmp461 * _tmp473 - _tmp466 * _tmp479 -
                       Scalar(1) / Scalar(2) *
                           (2 * _tmp463 * (_tmp388 + _tmp70) + 2 * _tmp464 * (_tmp104 + _tmp447)) /
                           _tmp465)) *
               std::sinh(_tmp469)) -
      _tmp473 * (Scalar(0.87679799772039002) * _tmp451 + std::cosh(_tmp462) - std::cosh(_tmp469));
  _res(3, 0) =
      _tmp364 -
      _tmp486 *
          (-Scalar(0.86565325453551001) * _tmp496 + Scalar(1.0) * _tmp499 * std::sinh(_tmp501) -
           (-Scalar(0.1034955) * _tmp490 * _tmp496 +
            _tmp491 * (-_tmp485 * _tmp500 - _tmp486 * _tmp499 -
                       Scalar(1) / Scalar(2) *
                           (2 * _tmp487 * (_tmp279 + _tmp92) + 2 * _tmp488 * (_tmp339 + _tmp99)) /
                           _tmp489)) *
               std::sinh(_tmp492)) -
      _tmp500 * (Scalar(0.86565325453551001) * _tmp481 - std::cosh(_tmp492) + std::cosh(_tmp501));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
