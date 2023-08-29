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
 * Symbolic function: IK_residual_func_cost1_wrt_ry_Nl11
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost1WrtRyNl11(
    const Scalar fh1, const Scalar fv1, const Scalar rx, const Scalar ry, const Scalar rz,
    const Scalar p_init0, const Scalar p_init1, const Scalar p_init2, const Scalar rot_init_x,
    const Scalar rot_init_y, const Scalar rot_init_z, const Scalar rot_init_w,
    const Scalar epsilon) {
  // Total ops: 1632

  // Unused inputs
  (void)p_init2;
  (void)epsilon;

  // Input arrays

  // Intermediate terms (505)
  const Scalar _tmp0 = Scalar(1.0) / (fh1);
  const Scalar _tmp1 = std::pow(ry, Scalar(2));
  const Scalar _tmp2 = _tmp1 + std::pow(rx, Scalar(2)) + std::pow(rz, Scalar(2));
  const Scalar _tmp3 = std::sqrt(_tmp2);
  const Scalar _tmp4 = (Scalar(1) / Scalar(2)) * _tmp3;
  const Scalar _tmp5 = std::cos(_tmp4);
  const Scalar _tmp6 = _tmp5 * rot_init_x;
  const Scalar _tmp7 = std::sin(_tmp4);
  const Scalar _tmp8 = _tmp7 / _tmp3;
  const Scalar _tmp9 = _tmp8 * rot_init_z;
  const Scalar _tmp10 = _tmp9 * ry;
  const Scalar _tmp11 = _tmp8 * rot_init_w;
  const Scalar _tmp12 = _tmp8 * rot_init_y;
  const Scalar _tmp13 = -_tmp10 + _tmp11 * rx + _tmp12 * rz + _tmp6;
  const Scalar _tmp14 = _tmp5 * rot_init_y;
  const Scalar _tmp15 = _tmp11 * ry;
  const Scalar _tmp16 = _tmp8 * rot_init_x;
  const Scalar _tmp17 = _tmp14 + _tmp15 - _tmp16 * rz + _tmp9 * rx;
  const Scalar _tmp18 = 2 * _tmp17;
  const Scalar _tmp19 = _tmp13 * _tmp18;
  const Scalar _tmp20 = _tmp5 * rot_init_z;
  const Scalar _tmp21 = _tmp16 * ry;
  const Scalar _tmp22 = _tmp11 * rz - _tmp12 * rx + _tmp20 + _tmp21;
  const Scalar _tmp23 = _tmp5 * rot_init_w;
  const Scalar _tmp24 = _tmp12 * ry;
  const Scalar _tmp25 = -_tmp16 * rx + _tmp23 - _tmp24 - _tmp9 * rz;
  const Scalar _tmp26 = 2 * _tmp25;
  const Scalar _tmp27 = _tmp22 * _tmp26;
  const Scalar _tmp28 = Scalar(0.20999999999999999) * _tmp19 + Scalar(0.20999999999999999) * _tmp27;
  const Scalar _tmp29 = -2 * std::pow(_tmp13, Scalar(2));
  const Scalar _tmp30 = -2 * std::pow(_tmp22, Scalar(2));
  const Scalar _tmp31 = Scalar(0.20999999999999999) * _tmp29 +
                        Scalar(0.20999999999999999) * _tmp30 + Scalar(0.20999999999999999);
  const Scalar _tmp32 = _tmp18 * _tmp22;
  const Scalar _tmp33 = _tmp13 * _tmp26;
  const Scalar _tmp34 = _tmp32 - _tmp33;
  const Scalar _tmp35 = -Scalar(0.010999999999999999) * _tmp34;
  const Scalar _tmp36 = -_tmp31 + _tmp35;
  const Scalar _tmp37 = _tmp28 + _tmp36;
  const Scalar _tmp38 = _tmp37 + p_init1;
  const Scalar _tmp39 = -_tmp38 + Scalar(-8.3888750099999996);
  const Scalar _tmp40 = 1 - 2 * std::pow(_tmp17, Scalar(2));
  const Scalar _tmp41 = Scalar(0.20999999999999999) * _tmp30 + Scalar(0.20999999999999999) * _tmp40;
  const Scalar _tmp42 = 2 * _tmp13 * _tmp22;
  const Scalar _tmp43 = _tmp18 * _tmp25;
  const Scalar _tmp44 = _tmp42 + _tmp43;
  const Scalar _tmp45 = -Scalar(0.010999999999999999) * _tmp44;
  const Scalar _tmp46 = Scalar(0.20999999999999999) * _tmp19 - Scalar(0.20999999999999999) * _tmp27;
  const Scalar _tmp47 = _tmp45 - _tmp46;
  const Scalar _tmp48 = _tmp41 + _tmp47;
  const Scalar _tmp49 = _tmp48 + p_init0;
  const Scalar _tmp50 = Scalar(2.5202214700000001) - _tmp49;
  const Scalar _tmp51 =
      std::sqrt(Scalar(std::pow(_tmp39, Scalar(2)) + std::pow(_tmp50, Scalar(2))));
  const Scalar _tmp52 = (Scalar(1) / Scalar(2)) / _tmp2;
  const Scalar _tmp53 = _tmp1 * _tmp52;
  const Scalar _tmp54 = _tmp52 * ry;
  const Scalar _tmp55 = _tmp54 * rz;
  const Scalar _tmp56 = _tmp54 * rx;
  const Scalar _tmp57 = _tmp7 / (_tmp2 * std::sqrt(_tmp2));
  const Scalar _tmp58 = _tmp1 * _tmp57;
  const Scalar _tmp59 = _tmp57 * ry;
  const Scalar _tmp60 = _tmp59 * rx;
  const Scalar _tmp61 = _tmp59 * rz;
  const Scalar _tmp62 = _tmp11 + _tmp20 * _tmp56 + _tmp23 * _tmp53 -
                        Scalar(1) / Scalar(2) * _tmp24 - _tmp55 * _tmp6 - _tmp58 * rot_init_w -
                        _tmp60 * rot_init_z + _tmp61 * rot_init_x;
  const Scalar _tmp63 = Scalar(0.41999999999999998) * _tmp62;
  const Scalar _tmp64 = _tmp13 * _tmp63;
  const Scalar _tmp65 = _tmp14 * _tmp55 - _tmp20 * _tmp53 - Scalar(1) / Scalar(2) * _tmp21 +
                        _tmp23 * _tmp56 + _tmp58 * rot_init_z - _tmp60 * rot_init_w -
                        _tmp61 * rot_init_y - _tmp9;
  const Scalar _tmp66 = Scalar(0.41999999999999998) * _tmp65;
  const Scalar _tmp67 = _tmp17 * _tmp66;
  const Scalar _tmp68 = -_tmp64 - _tmp67;
  const Scalar _tmp69 = -Scalar(1) / Scalar(2) * _tmp10 - _tmp14 * _tmp56 + _tmp16 +
                        _tmp23 * _tmp55 + _tmp53 * _tmp6 - _tmp58 * rot_init_x +
                        _tmp60 * rot_init_y - _tmp61 * rot_init_w;
  const Scalar _tmp70 = Scalar(0.41999999999999998) * _tmp25;
  const Scalar _tmp71 = _tmp69 * _tmp70;
  const Scalar _tmp72 = -_tmp12 - _tmp14 * _tmp53 - Scalar(1) / Scalar(2) * _tmp15 -
                        _tmp20 * _tmp55 - _tmp56 * _tmp6 + _tmp58 * rot_init_y +
                        _tmp60 * rot_init_x + _tmp61 * rot_init_z;
  const Scalar _tmp73 = Scalar(0.41999999999999998) * _tmp72;
  const Scalar _tmp74 = _tmp22 * _tmp73;
  const Scalar _tmp75 = -_tmp71 - _tmp74;
  const Scalar _tmp76 = _tmp68 + _tmp75;
  const Scalar _tmp77 = Scalar(0.83999999999999997) * _tmp69;
  const Scalar _tmp78 = _tmp22 * _tmp77;
  const Scalar _tmp79 = -_tmp78;
  const Scalar _tmp80 = Scalar(0.83999999999999997) * _tmp65;
  const Scalar _tmp81 = _tmp13 * _tmp80;
  const Scalar _tmp82 = _tmp79 - _tmp81;
  const Scalar _tmp83 = Scalar(0.021999999999999999) * _tmp25;
  const Scalar _tmp84 = _tmp65 * _tmp83;
  const Scalar _tmp85 = Scalar(0.021999999999999999) * _tmp72;
  const Scalar _tmp86 = _tmp13 * _tmp85;
  const Scalar _tmp87 = Scalar(0.021999999999999999) * _tmp69;
  const Scalar _tmp88 = _tmp17 * _tmp87;
  const Scalar _tmp89 = Scalar(0.021999999999999999) * _tmp22;
  const Scalar _tmp90 = _tmp62 * _tmp89;
  const Scalar _tmp91 = -_tmp84 - _tmp86 + _tmp88 + _tmp90;
  const Scalar _tmp92 = _tmp82 + _tmp91;
  const Scalar _tmp93 = _tmp62 * _tmp83;
  const Scalar _tmp94 = _tmp17 * _tmp85;
  const Scalar _tmp95 = _tmp13 * _tmp87;
  const Scalar _tmp96 = _tmp65 * _tmp89;
  const Scalar _tmp97 = _tmp93 + _tmp94 + _tmp95 + _tmp96;
  const Scalar _tmp98 = _tmp17 * _tmp62;
  const Scalar _tmp99 = Scalar(0.83999999999999997) * _tmp98;
  const Scalar _tmp100 = _tmp78 + _tmp99;
  const Scalar _tmp101 = _tmp100 + _tmp97;
  const Scalar _tmp102 = _tmp64 + _tmp67;
  const Scalar _tmp103 = _tmp102 + _tmp75;
  const Scalar _tmp104 = _tmp65 * _tmp70;
  const Scalar _tmp105 = -_tmp104;
  const Scalar _tmp106 = _tmp13 * _tmp73;
  const Scalar _tmp107 = -_tmp106;
  const Scalar _tmp108 = Scalar(0.41999999999999998) * _tmp69;
  const Scalar _tmp109 = _tmp108 * _tmp17;
  const Scalar _tmp110 = -_tmp109;
  const Scalar _tmp111 = _tmp22 * _tmp63;
  const Scalar _tmp112 = -_tmp111;
  const Scalar _tmp113 = Scalar(0.043999999999999997) * _tmp98;
  const Scalar _tmp114 = _tmp13 * _tmp65;
  const Scalar _tmp115 = Scalar(0.043999999999999997) * _tmp114;
  const Scalar _tmp116 = _tmp113 + _tmp115;
  const Scalar _tmp117 = _tmp105 + _tmp107 + _tmp110 + _tmp112 + _tmp116;
  const Scalar _tmp118 = _tmp62 * _tmp70;
  const Scalar _tmp119 = _tmp17 * _tmp73;
  const Scalar _tmp120 = _tmp108 * _tmp13;
  const Scalar _tmp121 = _tmp22 * _tmp66;
  const Scalar _tmp122 = -_tmp118 - _tmp119 + _tmp120 + _tmp121;
  const Scalar _tmp123 = _tmp117 + _tmp122;
  const Scalar _tmp124 = -_tmp28;
  const Scalar _tmp125 = _tmp124 + _tmp36;
  const Scalar _tmp126 = _tmp125 + p_init1;
  const Scalar _tmp127 = _tmp126 + Scalar(8.3196563700000006);
  const Scalar _tmp128 = -_tmp41;
  const Scalar _tmp129 = _tmp128 + _tmp47;
  const Scalar _tmp130 = _tmp129 + p_init0;
  const Scalar _tmp131 = _tmp130 + Scalar(1.9874742000000001);
  const Scalar _tmp132 = std::pow(_tmp127, Scalar(2)) + std::pow(_tmp131, Scalar(2));
  const Scalar _tmp133 = std::pow(_tmp132, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp134 = _tmp131 * _tmp133;
  const Scalar _tmp135 = _tmp31 + _tmp35;
  const Scalar _tmp136 = _tmp124 + _tmp135;
  const Scalar _tmp137 = _tmp136 + p_init1;
  const Scalar _tmp138 = _tmp137 + Scalar(-4.8333311099999996);
  const Scalar _tmp139 = _tmp45 + _tmp46;
  const Scalar _tmp140 = _tmp128 + _tmp139;
  const Scalar _tmp141 = _tmp140 + p_init0;
  const Scalar _tmp142 = _tmp141 + Scalar(1.79662371);
  const Scalar _tmp143 = Scalar(1.0) / (_tmp142);
  const Scalar _tmp144 = _tmp138 * _tmp143;
  const Scalar _tmp145 =
      Scalar(0.20999999999999999) * _tmp32 + Scalar(0.20999999999999999) * _tmp33;
  const Scalar _tmp146 =
      -Scalar(0.010999999999999999) * _tmp29 - Scalar(0.010999999999999999) * _tmp40;
  const Scalar _tmp147 =
      Scalar(0.20999999999999999) * _tmp42 - Scalar(0.20999999999999999) * _tmp43;
  const Scalar _tmp148 = _tmp146 - _tmp147;
  const Scalar _tmp149 = _tmp145 + _tmp148;
  const Scalar _tmp150 = _tmp134 * _tmp149;
  const Scalar _tmp151 = -_tmp145;
  const Scalar _tmp152 = _tmp148 + _tmp151;
  const Scalar _tmp153 = _tmp127 * _tmp133;
  const Scalar _tmp154 = _tmp135 + _tmp28;
  const Scalar _tmp155 = _tmp154 + p_init1;
  const Scalar _tmp156 = _tmp155 + Scalar(-4.7752063900000001);
  const Scalar _tmp157 = _tmp139 + _tmp41;
  const Scalar _tmp158 = _tmp157 + p_init0;
  const Scalar _tmp159 = _tmp158 + Scalar(-2.71799795);
  const Scalar _tmp160 = std::pow(_tmp156, Scalar(2)) + std::pow(_tmp159, Scalar(2));
  const Scalar _tmp161 = std::pow(_tmp160, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp162 = _tmp159 * _tmp161;
  const Scalar _tmp163 = _tmp144 * _tmp149;
  const Scalar _tmp164 = _tmp146 + _tmp147;
  const Scalar _tmp165 = _tmp145 + _tmp164;
  const Scalar _tmp166 = _tmp156 * _tmp161;
  const Scalar _tmp167 = -_tmp162 * _tmp163 + _tmp165 * _tmp166;
  const Scalar _tmp168 = _tmp134 * _tmp144 - _tmp153;
  const Scalar _tmp169 = _tmp144 * _tmp162 - _tmp166;
  const Scalar _tmp170 = Scalar(1.0) / (_tmp169);
  const Scalar _tmp171 = _tmp168 * _tmp170;
  const Scalar _tmp172 = -_tmp144 * _tmp150 + _tmp152 * _tmp153 - _tmp167 * _tmp171;
  const Scalar _tmp173 = Scalar(1.0) * _tmp136;
  const Scalar _tmp174 = -_tmp173;
  const Scalar _tmp175 = _tmp154 + _tmp174;
  const Scalar _tmp176 = Scalar(1.0) / (_tmp175);
  const Scalar _tmp177 = Scalar(1.0) * _tmp140;
  const Scalar _tmp178 = -_tmp157 + _tmp177;
  const Scalar _tmp179 = _tmp176 * _tmp178;
  const Scalar _tmp180 = _tmp149 * _tmp162;
  const Scalar _tmp181 = -_tmp162 * _tmp165 + _tmp180;
  const Scalar _tmp182 = -_tmp134 * _tmp152 + _tmp150 - _tmp171 * _tmp181 - _tmp172 * _tmp179;
  const Scalar _tmp183 = Scalar(1.0) / (_tmp182);
  const Scalar _tmp184 = _tmp173 * _tmp179 + _tmp177;
  const Scalar _tmp185 = 0;
  const Scalar _tmp186 = _tmp183 * _tmp185;
  const Scalar _tmp187 = _tmp162 * _tmp171;
  const Scalar _tmp188 = _tmp134 * _tmp186 - _tmp186 * _tmp187;
  const Scalar _tmp189 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp190 = std::pow(_tmp142, Scalar(2));
  const Scalar _tmp191 = std::pow(_tmp138, Scalar(2)) + _tmp190;
  const Scalar _tmp192 = std::sqrt(_tmp191);
  const Scalar _tmp193 = _tmp143 * _tmp192;
  const Scalar _tmp194 = _tmp189 * _tmp193;
  const Scalar _tmp195 = _tmp49 + Scalar(-2.5202214700000001);
  const Scalar _tmp196 = _tmp38 + Scalar(8.3888750099999996);
  const Scalar _tmp197 = std::pow(_tmp195, Scalar(2)) + std::pow(_tmp196, Scalar(2));
  const Scalar _tmp198 = std::pow(_tmp197, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp199 = _tmp198 * _tmp37;
  const Scalar _tmp200 = _tmp196 * _tmp198;
  const Scalar _tmp201 = fh1 * (-_tmp195 * _tmp199 + _tmp200 * _tmp48);
  const Scalar _tmp202 = Scalar(1.0) / (_tmp192);
  const Scalar _tmp203 = _tmp140 * _tmp202;
  const Scalar _tmp204 = _tmp142 * _tmp202;
  const Scalar _tmp205 = -_tmp136 * _tmp204 + _tmp138 * _tmp203;
  const Scalar _tmp206 = _tmp193 * _tmp205;
  const Scalar _tmp207 = _tmp154 * _tmp162 - _tmp157 * _tmp166 + _tmp162 * _tmp206;
  const Scalar _tmp208 =
      _tmp125 * _tmp134 - _tmp129 * _tmp153 + _tmp134 * _tmp206 - _tmp171 * _tmp207;
  const Scalar _tmp209 = Scalar(1.0) / (_tmp208);
  const Scalar _tmp210 = Scalar(1.0) * _tmp209;
  const Scalar _tmp211 = _tmp134 * _tmp210 - _tmp187 * _tmp210;
  const Scalar _tmp212 = _tmp193 * _tmp211;
  const Scalar _tmp213 = Scalar(1.0) * _tmp170;
  const Scalar _tmp214 = Scalar(1.0) * _tmp176;
  const Scalar _tmp215 = _tmp178 * _tmp214;
  const Scalar _tmp216 = _tmp167 * _tmp170;
  const Scalar _tmp217 = -_tmp181 * _tmp213 + _tmp215 * _tmp216;
  const Scalar _tmp218 = _tmp183 * _tmp208;
  const Scalar _tmp219 = -_tmp207 * _tmp213 - _tmp217 * _tmp218;
  const Scalar _tmp220 = _tmp209 * _tmp219;
  const Scalar _tmp221 = _tmp182 * _tmp220;
  const Scalar _tmp222 = _tmp217 + _tmp221;
  const Scalar _tmp223 = _tmp183 * _tmp222;
  const Scalar _tmp224 = -_tmp168 * _tmp223 + Scalar(1.0);
  const Scalar _tmp225 = _tmp162 * _tmp170;
  const Scalar _tmp226 = _tmp134 * _tmp223 + _tmp224 * _tmp225;
  const Scalar _tmp227 = _tmp200 * fh1;
  const Scalar _tmp228 = _tmp193 * _tmp227;
  const Scalar _tmp229 = _tmp195 * _tmp198;
  const Scalar _tmp230 = _tmp229 * fh1;
  const Scalar _tmp231 = _tmp144 * _tmp170;
  const Scalar _tmp232 = _tmp163 + _tmp167 * _tmp231;
  const Scalar _tmp233 = -_tmp149 - _tmp179 * _tmp232 + _tmp181 * _tmp231;
  const Scalar _tmp234 = -_tmp206 + _tmp207 * _tmp231 - _tmp218 * _tmp233;
  const Scalar _tmp235 = _tmp182 * _tmp209;
  const Scalar _tmp236 = _tmp234 * _tmp235;
  const Scalar _tmp237 = _tmp233 + _tmp236;
  const Scalar _tmp238 = _tmp183 * _tmp237;
  const Scalar _tmp239 = _tmp168 * _tmp183;
  const Scalar _tmp240 = -_tmp144 - _tmp237 * _tmp239;
  const Scalar _tmp241 = _tmp134 * _tmp238 + _tmp225 * _tmp240 + Scalar(1.0);
  const Scalar _tmp242 = _tmp193 * _tmp241;
  const Scalar _tmp243 =
      -_tmp188 * _tmp194 - _tmp201 * _tmp212 - _tmp226 * _tmp228 - _tmp230 * _tmp242;
  const Scalar _tmp244 = Scalar(1.0) / (_tmp243);
  const Scalar _tmp245 = _tmp125 + _tmp174;
  const Scalar _tmp246 = _tmp179 * _tmp245;
  const Scalar _tmp247 = -_tmp129 + _tmp177 - _tmp246;
  const Scalar _tmp248 = Scalar(1.0) / (_tmp247);
  const Scalar _tmp249 = Scalar(1.0) * _tmp248;
  const Scalar _tmp250 = _tmp235 * _tmp249;
  const Scalar _tmp251 = -_tmp172 * _tmp210 + _tmp245 * _tmp250;
  const Scalar _tmp252 = -_tmp214 * _tmp251 + _tmp250;
  const Scalar _tmp253 = Scalar(1.0) * _tmp201;
  const Scalar _tmp254 = _tmp245 * _tmp248;
  const Scalar _tmp255 = _tmp167 * _tmp213;
  const Scalar _tmp256 = -_tmp172 * _tmp223 + _tmp221 * _tmp254 - _tmp255;
  const Scalar _tmp257 = -_tmp214 * _tmp256 + _tmp221 * _tmp249;
  const Scalar _tmp258 = Scalar(1.0) * _tmp227;
  const Scalar _tmp259 = fh1 * (_tmp151 + _tmp164);
  const Scalar _tmp260 = _tmp198 * _tmp259;
  const Scalar _tmp261 = _tmp195 * _tmp260 + Scalar(3.29616) * _tmp44 + _tmp48 * fv1;
  const Scalar _tmp262 = _tmp176 * _tmp249;
  const Scalar _tmp263 = Scalar(1.0) * _tmp245 * _tmp262 - Scalar(1.0) * _tmp249;
  const Scalar _tmp264 = _tmp172 * _tmp183;
  const Scalar _tmp265 = _tmp232 + _tmp236 * _tmp254 - _tmp237 * _tmp264;
  const Scalar _tmp266 = -_tmp214 * _tmp265 + _tmp236 * _tmp249;
  const Scalar _tmp267 = Scalar(1.0) * _tmp230;
  const Scalar _tmp268 = _tmp179 * _tmp249;
  const Scalar _tmp269 = _tmp246 * _tmp249 + Scalar(1.0);
  const Scalar _tmp270 = -_tmp214 * _tmp269 + _tmp268;
  const Scalar _tmp271 = -_tmp200 * _tmp259 - Scalar(3.29616) * _tmp34 - _tmp37 * fv1;
  const Scalar _tmp272 = Scalar(1.0) * _tmp271;
  const Scalar _tmp273 = _tmp184 * _tmp248;
  const Scalar _tmp274 = _tmp174 - _tmp185 * _tmp264 - _tmp245 * _tmp273;
  const Scalar _tmp275 = Scalar(1.0) * _tmp189;
  const Scalar _tmp276 = _tmp252 * _tmp253 + _tmp257 * _tmp258 + _tmp261 * _tmp263 +
                         _tmp266 * _tmp267 + _tmp270 * _tmp272 +
                         _tmp275 * (-_tmp184 * _tmp249 - _tmp214 * _tmp274 + Scalar(1.0));
  const Scalar _tmp277 = std::asinh(_tmp244 * _tmp276);
  const Scalar _tmp278 = Scalar(9.6622558468725703) * _tmp243;
  const Scalar _tmp279 = Scalar(4.8333311099999996) - _tmp137;
  const Scalar _tmp280 = -_tmp141 + Scalar(-1.79662371);
  const Scalar _tmp281 =
      std::sqrt(Scalar(std::pow(_tmp279, Scalar(2)) + std::pow(_tmp280, Scalar(2))));
  const Scalar _tmp282 = -_tmp277 * _tmp278 - _tmp281;
  const Scalar _tmp283 = Scalar(0.1034955) * _tmp244;
  const Scalar _tmp284 = _tmp282 * _tmp283;
  const Scalar _tmp285 = std::pow(_tmp243, Scalar(-2));
  const Scalar _tmp286 = _tmp78 + _tmp81;
  const Scalar _tmp287 = _tmp71 + _tmp74;
  const Scalar _tmp288 = _tmp102 + _tmp287;
  const Scalar _tmp289 = _tmp84 + _tmp86 - _tmp88 - _tmp90;
  const Scalar _tmp290 = _tmp288 + _tmp289;
  const Scalar _tmp291 = _tmp286 + _tmp290;
  const Scalar _tmp292 = _tmp198 * _tmp291;
  const Scalar _tmp293 = _tmp292 * fh1;
  const Scalar _tmp294 = _tmp193 * _tmp226;
  const Scalar _tmp295 = -_tmp93 - _tmp94 - _tmp95 - _tmp96;
  const Scalar _tmp296 = _tmp100 + _tmp295;
  const Scalar _tmp297 = _tmp103 + _tmp296;
  const Scalar _tmp298 = Scalar(1.0) / (_tmp190);
  const Scalar _tmp299 = _tmp192 * _tmp297 * _tmp298;
  const Scalar _tmp300 = _tmp201 * _tmp211;
  const Scalar _tmp301 = _tmp226 * _tmp227;
  const Scalar _tmp302 = _tmp138 * _tmp297;
  const Scalar _tmp303 = _tmp289 + _tmp76;
  const Scalar _tmp304 = _tmp303 + _tmp82;
  const Scalar _tmp305 = _tmp138 * _tmp304 + _tmp142 * _tmp297;
  const Scalar _tmp306 = _tmp305 / (_tmp191 * std::sqrt(_tmp191));
  const Scalar _tmp307 = _tmp193 * (_tmp136 * _tmp142 * _tmp306 - _tmp136 * _tmp202 * _tmp297 -
                                    _tmp138 * _tmp140 * _tmp306 + _tmp202 * _tmp302 +
                                    _tmp203 * _tmp304 - _tmp204 * _tmp304);
  const Scalar _tmp308 = _tmp287 + _tmp68;
  const Scalar _tmp309 = _tmp296 + _tmp308;
  const Scalar _tmp310 = _tmp286 + _tmp303;
  const Scalar _tmp311 =
      (2 * _tmp127 * _tmp310 + 2 * _tmp131 * _tmp309) / (_tmp132 * std::sqrt(_tmp132));
  const Scalar _tmp312 = (Scalar(1) / Scalar(2)) * _tmp311;
  const Scalar _tmp313 = _tmp131 * _tmp312;
  const Scalar _tmp314 = _tmp127 * _tmp312;
  const Scalar _tmp315 = _tmp143 * _tmp304;
  const Scalar _tmp316 = _tmp133 * _tmp310;
  const Scalar _tmp317 = _tmp298 * _tmp302;
  const Scalar _tmp318 = _tmp133 * _tmp309;
  const Scalar _tmp319 = _tmp134 * _tmp315 - _tmp134 * _tmp317 - _tmp144 * _tmp313 +
                         _tmp144 * _tmp318 + _tmp314 - _tmp316;
  const Scalar _tmp320 = _tmp170 * _tmp319;
  const Scalar _tmp321 = _tmp143 * _tmp202 * _tmp305;
  const Scalar _tmp322 = _tmp205 * _tmp321;
  const Scalar _tmp323 = _tmp205 * _tmp299;
  const Scalar _tmp324 = _tmp290 + _tmp82;
  const Scalar _tmp325 = _tmp79 - _tmp99;
  const Scalar _tmp326 = _tmp103 + _tmp325;
  const Scalar _tmp327 = _tmp295 + _tmp326;
  const Scalar _tmp328 =
      (2 * _tmp156 * _tmp324 + 2 * _tmp159 * _tmp327) / (_tmp160 * std::sqrt(_tmp160));
  const Scalar _tmp329 = (Scalar(1) / Scalar(2)) * _tmp328;
  const Scalar _tmp330 = _tmp159 * _tmp329;
  const Scalar _tmp331 = _tmp161 * _tmp327;
  const Scalar _tmp332 = _tmp161 * _tmp324;
  const Scalar _tmp333 = _tmp156 * _tmp329;
  const Scalar _tmp334 = -_tmp154 * _tmp330 + _tmp154 * _tmp331 - _tmp157 * _tmp332 +
                         _tmp157 * _tmp333 + _tmp159 * _tmp332 + _tmp162 * _tmp307 +
                         _tmp162 * _tmp322 - _tmp162 * _tmp323 - _tmp166 * _tmp327 -
                         _tmp206 * _tmp330 + _tmp206 * _tmp331;
  const Scalar _tmp335 = (-_tmp144 * _tmp330 + _tmp144 * _tmp331 + _tmp162 * _tmp315 -
                          _tmp162 * _tmp317 - _tmp332 + _tmp333) /
                         std::pow(_tmp169, Scalar(2));
  const Scalar _tmp336 = _tmp168 * _tmp335;
  const Scalar _tmp337 = -_tmp125 * _tmp313 + _tmp125 * _tmp318 + _tmp129 * _tmp314 -
                         _tmp129 * _tmp316 + _tmp131 * _tmp316 + _tmp134 * _tmp307 +
                         _tmp134 * _tmp322 - _tmp134 * _tmp323 - _tmp153 * _tmp309 -
                         _tmp171 * _tmp334 - _tmp206 * _tmp313 + _tmp206 * _tmp318 -
                         _tmp207 * _tmp320 + _tmp207 * _tmp336;
  const Scalar _tmp338 = _tmp337 / std::pow(_tmp208, Scalar(2));
  const Scalar _tmp339 = Scalar(1.0) * _tmp338;
  const Scalar _tmp340 = Scalar(0.5) * _tmp209;
  const Scalar _tmp341 = _tmp162 * _tmp320;
  const Scalar _tmp342 = _tmp168 * _tmp213 * _tmp338;
  const Scalar _tmp343 = _tmp162 * _tmp336;
  const Scalar _tmp344 = _tmp171 * _tmp331;
  const Scalar _tmp345 = _tmp308 + _tmp325;
  const Scalar _tmp346 = _tmp295 + _tmp345;
  const Scalar _tmp347 =
      (2 * _tmp195 * _tmp346 + 2 * _tmp196 * _tmp291) / (_tmp197 * std::sqrt(_tmp197));
  const Scalar _tmp348 = (Scalar(1) / Scalar(2)) * _tmp347;
  const Scalar _tmp349 = _tmp195 * _tmp348;
  const Scalar _tmp350 = _tmp196 * _tmp348;
  const Scalar _tmp351 = fh1 * (-_tmp195 * _tmp292 - _tmp199 * _tmp346 + _tmp200 * _tmp346 +
                                _tmp292 * _tmp48 + _tmp349 * _tmp37 - _tmp350 * _tmp48);
  const Scalar _tmp352 = _tmp170 * _tmp331;
  const Scalar _tmp353 = _tmp25 * _tmp77;
  const Scalar _tmp354 = Scalar(0.83999999999999997) * _tmp22 * _tmp72;
  const Scalar _tmp355 = Scalar(0.83999999999999997) * _tmp13 * _tmp62 + _tmp17 * _tmp80;
  const Scalar _tmp356 = (_tmp353 + _tmp354 + _tmp355) / std::pow(_tmp175, Scalar(2));
  const Scalar _tmp357 = _tmp178 * _tmp356;
  const Scalar _tmp358 = Scalar(1.6799999999999999) * _tmp22 * _tmp69;
  const Scalar _tmp359 = _tmp358 + Scalar(1.6799999999999999) * _tmp98;
  const Scalar _tmp360 = _tmp176 * _tmp359;
  const Scalar _tmp361 = _tmp118 + _tmp119 - _tmp120 - _tmp121;
  const Scalar _tmp362 = _tmp117 + _tmp361;
  const Scalar _tmp363 = _tmp104 + _tmp106 + _tmp109 + _tmp111 + _tmp116;
  const Scalar _tmp364 = _tmp361 + _tmp363;
  const Scalar _tmp365 = _tmp134 * _tmp364;
  const Scalar _tmp366 = _tmp122 + _tmp363;
  const Scalar _tmp367 = _tmp149 * _tmp315;
  const Scalar _tmp368 = _tmp162 * _tmp364;
  const Scalar _tmp369 = -_tmp144 * _tmp368 - _tmp162 * _tmp367 + _tmp163 * _tmp330 -
                         _tmp163 * _tmp331 + _tmp165 * _tmp332 - _tmp165 * _tmp333 +
                         _tmp166 * _tmp366 + _tmp180 * _tmp317;
  const Scalar _tmp370 = _tmp149 * _tmp318;
  const Scalar _tmp371 = -_tmp134 * _tmp367 - _tmp144 * _tmp365 - _tmp144 * _tmp370 +
                         _tmp150 * _tmp317 - _tmp152 * _tmp314 + _tmp152 * _tmp316 +
                         _tmp153 * _tmp362 + _tmp163 * _tmp313 - _tmp167 * _tmp320 +
                         _tmp167 * _tmp336 - _tmp171 * _tmp369;
  const Scalar _tmp372 = -_tmp149 * _tmp330 + _tmp149 * _tmp331 - _tmp162 * _tmp366 +
                         _tmp165 * _tmp330 - _tmp165 * _tmp331 + _tmp368;
  const Scalar _tmp373 = -_tmp134 * _tmp362 - _tmp149 * _tmp313 + _tmp152 * _tmp313 -
                         _tmp152 * _tmp318 - _tmp171 * _tmp372 + _tmp172 * _tmp357 -
                         _tmp172 * _tmp360 - _tmp179 * _tmp371 - _tmp181 * _tmp320 +
                         _tmp181 * _tmp336 + _tmp365 + _tmp370;
  const Scalar _tmp374 = _tmp373 / std::pow(_tmp182, Scalar(2));
  const Scalar _tmp375 = _tmp134 * _tmp374;
  const Scalar _tmp376 = _tmp162 * _tmp335;
  const Scalar _tmp377 = _tmp209 * _tmp373;
  const Scalar _tmp378 = _tmp234 * _tmp377;
  const Scalar _tmp379 = _tmp144 * _tmp335;
  const Scalar _tmp380 = _tmp208 * _tmp374;
  const Scalar _tmp381 = _tmp170 * _tmp317;
  const Scalar _tmp382 = _tmp183 * _tmp337;
  const Scalar _tmp383 = _tmp170 * _tmp315;
  const Scalar _tmp384 = _tmp144 * _tmp364 - _tmp149 * _tmp317 - _tmp167 * _tmp379 -
                         _tmp167 * _tmp381 + _tmp167 * _tmp383 + _tmp231 * _tmp369 + _tmp367;
  const Scalar _tmp385 = _tmp105 + _tmp107 + _tmp110 + _tmp112 - _tmp113 - _tmp115 + _tmp122 -
                         _tmp179 * _tmp384 - _tmp181 * _tmp379 - _tmp181 * _tmp381 +
                         _tmp181 * _tmp383 + _tmp231 * _tmp372 + _tmp232 * _tmp357 -
                         _tmp232 * _tmp360;
  const Scalar _tmp386 = _tmp235 * (-_tmp207 * _tmp379 - _tmp207 * _tmp381 + _tmp207 * _tmp383 -
                                    _tmp218 * _tmp385 + _tmp231 * _tmp334 + _tmp233 * _tmp380 -
                                    _tmp233 * _tmp382 - _tmp307 - _tmp322 + _tmp323);
  const Scalar _tmp387 = _tmp182 * _tmp338;
  const Scalar _tmp388 = _tmp234 * _tmp387;
  const Scalar _tmp389 = _tmp183 * (_tmp378 + _tmp385 + _tmp386 - _tmp388);
  const Scalar _tmp390 = _tmp170 * _tmp240;
  const Scalar _tmp391 = _tmp168 * _tmp374;
  const Scalar _tmp392 =
      -_tmp168 * _tmp389 + _tmp237 * _tmp391 - _tmp238 * _tmp319 - _tmp315 + _tmp317;
  const Scalar _tmp393 = _tmp349 * fh1;
  const Scalar _tmp394 = _tmp188 * _tmp189;
  const Scalar _tmp395 = _tmp230 * _tmp241;
  const Scalar _tmp396 = _tmp198 * _tmp346;
  const Scalar _tmp397 = _tmp396 * fh1;
  const Scalar _tmp398 = _tmp185 * _tmp374;
  const Scalar _tmp399 = _tmp350 * fh1;
  const Scalar _tmp400 = _tmp220 * _tmp373;
  const Scalar _tmp401 = Scalar(1.0) * _tmp335;
  const Scalar _tmp402 = -_tmp167 * _tmp215 * _tmp335 + _tmp170 * _tmp215 * _tmp369 +
                         _tmp181 * _tmp401 - _tmp213 * _tmp372 + _tmp214 * _tmp216 * _tmp359 -
                         _tmp255 * _tmp357;
  const Scalar _tmp403 = _tmp235 * (_tmp207 * _tmp401 - _tmp213 * _tmp334 + _tmp217 * _tmp380 -
                                    _tmp217 * _tmp382 - _tmp218 * _tmp402);
  const Scalar _tmp404 = _tmp219 * _tmp387;
  const Scalar _tmp405 = _tmp400 + _tmp402 + _tmp403 - _tmp404;
  const Scalar _tmp406 = _tmp183 * _tmp405;
  const Scalar _tmp407 = _tmp222 * _tmp391 - _tmp223 * _tmp319 - _tmp239 * _tmp405;
  const Scalar _tmp408 = _tmp170 * _tmp224;
  const Scalar _tmp409 =
      -_tmp193 * _tmp201 *
          (-_tmp131 * _tmp311 * _tmp340 - _tmp134 * _tmp339 +
           _tmp159 * _tmp171 * _tmp328 * _tmp340 + _tmp162 * _tmp342 + _tmp210 * _tmp318 -
           _tmp210 * _tmp341 + _tmp210 * _tmp343 - _tmp210 * _tmp344) -
      _tmp193 * _tmp230 *
          (_tmp134 * _tmp389 + _tmp225 * _tmp392 - _tmp237 * _tmp375 - _tmp238 * _tmp313 +
           _tmp238 * _tmp318 + _tmp240 * _tmp352 - _tmp240 * _tmp376 - _tmp330 * _tmp390) -
      _tmp194 * (-_tmp134 * _tmp398 + _tmp171 * _tmp186 * _tmp330 - _tmp186 * _tmp313 +
                 _tmp186 * _tmp318 - _tmp186 * _tmp341 + _tmp186 * _tmp343 - _tmp186 * _tmp344 +
                 _tmp187 * _tmp398) -
      _tmp212 * _tmp351 -
      _tmp228 * (_tmp134 * _tmp406 - _tmp222 * _tmp375 - _tmp223 * _tmp313 + _tmp223 * _tmp318 +
                 _tmp224 * _tmp352 - _tmp224 * _tmp376 + _tmp225 * _tmp407 - _tmp330 * _tmp408) +
      _tmp242 * _tmp393 - _tmp242 * _tmp397 - _tmp293 * _tmp294 + _tmp294 * _tmp399 +
      _tmp299 * _tmp300 + _tmp299 * _tmp301 + _tmp299 * _tmp394 + _tmp299 * _tmp395 -
      _tmp300 * _tmp321 - _tmp301 * _tmp321 - _tmp321 * _tmp394 - _tmp321 * _tmp395;
  const Scalar _tmp410 = _tmp285 * _tmp409;
  const Scalar _tmp411 = Scalar(9.6622558468725703) * _tmp409;
  const Scalar _tmp412 = Scalar(0.5) * _tmp347 * fh1;
  const Scalar _tmp413 = Scalar(6.59232) * _tmp25;
  const Scalar _tmp414 = Scalar(6.59232) * _tmp72;
  const Scalar _tmp415 = Scalar(6.59232) * _tmp69;
  const Scalar _tmp416 = Scalar(6.59232) * _tmp22;
  const Scalar _tmp417 = _tmp123 * fh1;
  const Scalar _tmp418 = _tmp13 * _tmp414 - _tmp17 * _tmp415 - _tmp200 * _tmp417 -
                         _tmp259 * _tmp292 + _tmp259 * _tmp350 - _tmp291 * fv1 + _tmp413 * _tmp65 -
                         _tmp416 * _tmp62;
  const Scalar _tmp419 = _tmp172 * _tmp374;
  const Scalar _tmp420 = _tmp245 * _tmp360;
  const Scalar _tmp421 = _tmp245 * _tmp357;
  const Scalar _tmp422 = Scalar(1.6799999999999999) * _tmp114 + _tmp358;
  const Scalar _tmp423 = _tmp179 * _tmp422;
  const Scalar _tmp424 =
      (-_tmp353 - _tmp354 + _tmp355 - _tmp420 + _tmp421 - _tmp423) / std::pow(_tmp247, Scalar(2));
  const Scalar _tmp425 = _tmp245 * _tmp424;
  const Scalar _tmp426 = _tmp183 * _tmp371;
  const Scalar _tmp427 = _tmp248 * _tmp422;
  const Scalar _tmp428 = _tmp167 * _tmp401 - _tmp213 * _tmp369 - _tmp221 * _tmp425 +
                         _tmp221 * _tmp427 + _tmp222 * _tmp419 - _tmp222 * _tmp426 +
                         _tmp254 * _tmp400 + _tmp254 * _tmp403 - _tmp254 * _tmp404 -
                         _tmp264 * _tmp405;
  const Scalar _tmp429 = Scalar(1.0) * _tmp424;
  const Scalar _tmp430 = Scalar(1.0) * _tmp356;
  const Scalar _tmp431 = -_tmp173 * _tmp357 + _tmp173 * _tmp360 + _tmp215 * _tmp304 + _tmp297;
  const Scalar _tmp432 = _tmp248 * _tmp431;
  const Scalar _tmp433 = _tmp184 * _tmp424;
  const Scalar _tmp434 = _tmp286 + _tmp91;
  const Scalar _tmp435 = _tmp288 + _tmp434;
  const Scalar _tmp436 = _tmp185 * _tmp419 - _tmp185 * _tmp426 - _tmp245 * _tmp432 +
                         _tmp245 * _tmp433 - _tmp273 * _tmp422 + _tmp435;
  const Scalar _tmp437 = _tmp214 * _tmp425;
  const Scalar _tmp438 = _tmp245 * _tmp356;
  const Scalar _tmp439 = _tmp13 * _tmp415 + _tmp17 * _tmp414 + _tmp229 * _tmp417 -
                         _tmp259 * _tmp349 + _tmp260 * _tmp346 + _tmp346 * fv1 + _tmp413 * _tmp62 +
                         _tmp416 * _tmp65;
  const Scalar _tmp440 = -_tmp172 * _tmp389 - _tmp236 * _tmp425 + _tmp236 * _tmp427 +
                         _tmp237 * _tmp419 - _tmp237 * _tmp426 + _tmp254 * _tmp378 +
                         _tmp254 * _tmp386 - _tmp254 * _tmp388 + _tmp384;
  const Scalar _tmp441 = _tmp182 * _tmp424;
  const Scalar _tmp442 = _tmp210 * _tmp441;
  const Scalar _tmp443 = _tmp215 * _tmp424;
  const Scalar _tmp444 =
      -_tmp245 * _tmp443 + _tmp249 * _tmp420 - _tmp249 * _tmp421 + _tmp249 * _tmp423;
  const Scalar _tmp445 = _tmp249 * _tmp377;
  const Scalar _tmp446 = _tmp249 * _tmp387;
  const Scalar _tmp447 = _tmp172 * _tmp339 - _tmp210 * _tmp371 - _tmp245 * _tmp442 +
                         _tmp245 * _tmp445 - _tmp245 * _tmp446 + _tmp250 * _tmp422;
  const Scalar _tmp448 =
      (_tmp244 *
           (-_tmp195 * _tmp266 * _tmp412 - _tmp196 * _tmp257 * _tmp412 +
            Scalar(1.0) * _tmp252 * _tmp351 +
            _tmp253 * (-_tmp214 * _tmp447 + _tmp251 * _tmp430 - _tmp442 + _tmp445 - _tmp446) +
            Scalar(1.0) * _tmp257 * _tmp293 +
            _tmp258 * (-_tmp214 * _tmp428 - _tmp221 * _tmp429 + _tmp249 * _tmp400 +
                       _tmp249 * _tmp403 - _tmp249 * _tmp404 + _tmp256 * _tmp430) +
            Scalar(1.0) * _tmp261 * (-_tmp249 * _tmp438 + _tmp262 * _tmp422 + _tmp429 - _tmp437) +
            _tmp263 * _tmp439 + Scalar(1.0) * _tmp266 * _tmp397 +
            _tmp267 * (-_tmp214 * _tmp440 - _tmp234 * _tmp442 + _tmp249 * _tmp378 +
                       _tmp249 * _tmp386 - _tmp249 * _tmp388 + _tmp265 * _tmp430) +
            Scalar(1.0) * _tmp270 * _tmp418 +
            _tmp272 * (-_tmp214 * _tmp444 - _tmp249 * _tmp357 + _tmp249 * _tmp360 +
                       _tmp269 * _tmp430 - _tmp443) +
            _tmp275 *
                (_tmp184 * _tmp429 - _tmp214 * _tmp436 - _tmp249 * _tmp431 + _tmp274 * _tmp430)) -
       _tmp276 * _tmp410) /
      std::sqrt(Scalar(std::pow(_tmp276, Scalar(2)) * _tmp285 + 1));
  const Scalar _tmp449 = Scalar(1.0) * _tmp277;
  const Scalar _tmp450 = _tmp186 * _tmp189;
  const Scalar _tmp451 = _tmp201 * _tmp210;
  const Scalar _tmp452 =
      -_tmp171 * _tmp450 - _tmp171 * _tmp451 + _tmp227 * _tmp408 + _tmp230 * _tmp390;
  const Scalar _tmp453 = Scalar(9.6622558468725703) * _tmp452;
  const Scalar _tmp454 = _tmp249 * _tmp261;
  const Scalar _tmp455 = _tmp176 * _tmp245;
  const Scalar _tmp456 = _tmp176 * _tmp227;
  const Scalar _tmp457 = _tmp176 * _tmp189;
  const Scalar _tmp458 = _tmp176 * _tmp271;
  const Scalar _tmp459 = _tmp176 * _tmp201;
  const Scalar _tmp460 = _tmp176 * _tmp230;
  const Scalar _tmp461 = _tmp251 * _tmp459 + _tmp256 * _tmp456 + _tmp265 * _tmp460 +
                         _tmp269 * _tmp458 + _tmp274 * _tmp457 - _tmp454 * _tmp455;
  const Scalar _tmp462 = std::pow(_tmp452, Scalar(-2));
  const Scalar _tmp463 = _tmp189 * _tmp398;
  const Scalar _tmp464 = _tmp210 * _tmp351;
  const Scalar _tmp465 = _tmp170 * _tmp227 * _tmp407 + _tmp170 * _tmp230 * _tmp392 +
                         _tmp171 * _tmp463 - _tmp171 * _tmp464 + _tmp201 * _tmp342 -
                         _tmp224 * _tmp227 * _tmp335 - _tmp230 * _tmp240 * _tmp335 +
                         _tmp293 * _tmp408 - _tmp320 * _tmp450 - _tmp320 * _tmp451 +
                         _tmp336 * _tmp450 + _tmp336 * _tmp451 - _tmp390 * _tmp393 +
                         _tmp390 * _tmp397 - _tmp399 * _tmp408;
  const Scalar _tmp466 = _tmp462 * _tmp465;
  const Scalar _tmp467 = _tmp176 * _tmp256;
  const Scalar _tmp468 = _tmp176 * _tmp265;
  const Scalar _tmp469 = _tmp249 * _tmp439;
  const Scalar _tmp470 = _tmp271 * _tmp356;
  const Scalar _tmp471 = Scalar(1.0) / (_tmp452);
  const Scalar _tmp472 =
      (-_tmp461 * _tmp466 +
       _tmp471 * (_tmp176 * _tmp251 * _tmp351 + _tmp176 * _tmp269 * _tmp418 -
                  _tmp176 * _tmp422 * _tmp454 - _tmp189 * _tmp274 * _tmp356 -
                  _tmp201 * _tmp251 * _tmp356 - _tmp227 * _tmp256 * _tmp356 -
                  _tmp230 * _tmp265 * _tmp356 + _tmp261 * _tmp437 - _tmp269 * _tmp470 +
                  _tmp293 * _tmp467 - _tmp393 * _tmp468 + _tmp397 * _tmp468 - _tmp399 * _tmp467 +
                  _tmp428 * _tmp456 + _tmp436 * _tmp457 + _tmp438 * _tmp454 + _tmp440 * _tmp460 +
                  _tmp444 * _tmp458 + _tmp447 * _tmp459 - _tmp455 * _tmp469)) /
      std::sqrt(Scalar(std::pow(_tmp461, Scalar(2)) * _tmp462 + 1));
  const Scalar _tmp473 = std::asinh(_tmp461 * _tmp471);
  const Scalar _tmp474 = Scalar(9.6622558468725703) * _tmp473;
  const Scalar _tmp475 = Scalar(4.7752063900000001) - _tmp155;
  const Scalar _tmp476 = Scalar(2.71799795) - _tmp158;
  const Scalar _tmp477 =
      std::sqrt(Scalar(std::pow(_tmp475, Scalar(2)) + std::pow(_tmp476, Scalar(2))));
  const Scalar _tmp478 = Scalar(0.1034955) * _tmp471;
  const Scalar _tmp479 = -_tmp452 * _tmp474 - _tmp477;
  const Scalar _tmp480 = _tmp478 * _tmp479;
  const Scalar _tmp481 = Scalar(1.0) * _tmp473;
  const Scalar _tmp482 = _tmp223 * _tmp227 + _tmp230 * _tmp238 + _tmp450 + _tmp451;
  const Scalar _tmp483 = Scalar(1.0) / (_tmp482);
  const Scalar _tmp484 = _tmp227 * _tmp248;
  const Scalar _tmp485 = _tmp249 * _tmp458;
  const Scalar _tmp486 = _tmp230 * _tmp248;
  const Scalar _tmp487 = -_tmp178 * _tmp485 + _tmp189 * _tmp273 - _tmp201 * _tmp250 -
                         _tmp221 * _tmp484 - _tmp236 * _tmp486 + _tmp454;
  const Scalar _tmp488 = std::asinh(_tmp483 * _tmp487);
  const Scalar _tmp489 = Scalar(9.6622558468725703) * _tmp482;
  const Scalar _tmp490 = -_tmp126 + Scalar(-8.3196563700000006);
  const Scalar _tmp491 = -_tmp130 + Scalar(-1.9874742000000001);
  const Scalar _tmp492 =
      std::sqrt(Scalar(std::pow(_tmp490, Scalar(2)) + std::pow(_tmp491, Scalar(2))));
  const Scalar _tmp493 = -_tmp488 * _tmp489 - _tmp492;
  const Scalar _tmp494 = Scalar(0.1034955) * _tmp483;
  const Scalar _tmp495 = _tmp493 * _tmp494;
  const Scalar _tmp496 = Scalar(1.0) * _tmp488;
  const Scalar _tmp497 = _tmp238 * fh1;
  const Scalar _tmp498 = -_tmp222 * _tmp227 * _tmp374 + _tmp223 * _tmp293 - _tmp223 * _tmp399 +
                         _tmp227 * _tmp406 - _tmp230 * _tmp237 * _tmp374 + _tmp230 * _tmp389 -
                         _tmp253 * _tmp338 - _tmp349 * _tmp497 + _tmp396 * _tmp497 - _tmp463 +
                         _tmp464;
  const Scalar _tmp499 = Scalar(9.6622558468725703) * _tmp498;
  const Scalar _tmp500 = std::pow(_tmp482, Scalar(-2));
  const Scalar _tmp501 = _tmp498 * _tmp500;
  const Scalar _tmp502 = _tmp236 * _tmp248;
  const Scalar _tmp503 = _tmp221 * _tmp248;
  const Scalar _tmp504 =
      (_tmp483 * (_tmp178 * _tmp249 * _tmp470 + _tmp178 * _tmp429 * _tmp458 + _tmp189 * _tmp432 -
                  _tmp189 * _tmp433 - _tmp201 * _tmp445 + _tmp201 * _tmp446 +
                  _tmp221 * _tmp227 * _tmp424 + _tmp230 * _tmp236 * _tmp424 - _tmp250 * _tmp351 -
                  _tmp261 * _tmp429 - _tmp268 * _tmp418 - _tmp293 * _tmp503 - _tmp359 * _tmp485 -
                  _tmp378 * _tmp486 - _tmp386 * _tmp486 + _tmp388 * _tmp486 + _tmp393 * _tmp502 -
                  _tmp397 * _tmp502 + _tmp399 * _tmp503 - _tmp400 * _tmp484 - _tmp403 * _tmp484 +
                  _tmp404 * _tmp484 + _tmp441 * _tmp451 + _tmp469) -
       _tmp487 * _tmp501) /
      std::sqrt(Scalar(std::pow(_tmp487, Scalar(2)) * _tmp500 + 1));

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) =
      _tmp123 -
      Scalar(0.5) * (2 * _tmp39 * (_tmp76 + _tmp92) + 2 * _tmp50 * (_tmp101 + _tmp103)) *
          std::sinh(Scalar(0.1034955) * _tmp0 *
                    (-_tmp51 - Scalar(9.6622558468725703) * fh1 * std::asinh(_tmp0 * fv1))) /
          _tmp51;
  _res(1, 0) =
      -_tmp278 *
          (-Scalar(0.86625939559540499) * _tmp410 + Scalar(1.0) * _tmp448 * std::sinh(_tmp449) -
           (-Scalar(0.1034955) * _tmp282 * _tmp410 +
            _tmp283 * (-_tmp277 * _tmp411 - _tmp278 * _tmp448 -
                       Scalar(1) / Scalar(2) *
                           (2 * _tmp279 * _tmp435 + 2 * _tmp280 * (_tmp345 + _tmp97)) / _tmp281)) *
               std::sinh(_tmp284)) +
      _tmp364 -
      _tmp411 * (Scalar(0.86625939559540499) * _tmp244 - std::cosh(_tmp284) + std::cosh(_tmp449));
  _res(2, 0) =
      _tmp366 -
      _tmp453 *
          (-Scalar(0.86565325453551001) * _tmp466 + Scalar(1.0) * _tmp472 * std::sinh(_tmp481) -
           (-Scalar(0.1034955) * _tmp466 * _tmp479 +
            _tmp478 * (-_tmp453 * _tmp472 - _tmp465 * _tmp474 -
                       Scalar(1) / Scalar(2) *
                           (2 * _tmp475 * (_tmp434 + _tmp76) + 2 * _tmp476 * (_tmp101 + _tmp308)) /
                           _tmp477)) *
               std::sinh(_tmp480)) -
      Scalar(9.6622558468725703) * _tmp465 *
          (Scalar(0.86565325453551001) * _tmp471 - std::cosh(_tmp480) + std::cosh(_tmp481));
  _res(3, 0) =
      _tmp362 -
      _tmp489 *
          (-Scalar(0.87679799772039002) * _tmp501 + Scalar(1.0) * _tmp504 * std::sinh(_tmp496) -
           (-Scalar(0.1034955) * _tmp493 * _tmp501 +
            _tmp494 * (-_tmp488 * _tmp499 - _tmp489 * _tmp504 -
                       Scalar(1) / Scalar(2) *
                           (2 * _tmp490 * (_tmp288 + _tmp92) + 2 * _tmp491 * (_tmp326 + _tmp97)) /
                           _tmp492)) *
               std::sinh(_tmp495)) -
      _tmp499 * (Scalar(0.87679799772039002) * _tmp483 - std::cosh(_tmp495) + std::cosh(_tmp496));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym