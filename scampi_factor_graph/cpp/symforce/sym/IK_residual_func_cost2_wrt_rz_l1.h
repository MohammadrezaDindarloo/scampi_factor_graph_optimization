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
 * Symbolic function: IK_residual_func_cost2_wrt_rz_l1
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost2WrtRzL1(
    const Scalar fh1, const Scalar fv1, const Scalar rx, const Scalar ry, const Scalar rz,
    const Scalar p_init0, const Scalar p_init1, const Scalar p_init2, const Scalar rot_init_x,
    const Scalar rot_init_y, const Scalar rot_init_z, const Scalar rot_init_w,
    const Scalar epsilon) {
  // Total ops: 1656

  // Unused inputs
  (void)epsilon;

  // Input arrays

  // Intermediate terms (506)
  const Scalar _tmp0 = std::pow(rz, Scalar(2));
  const Scalar _tmp1 = _tmp0 + std::pow(rx, Scalar(2)) + std::pow(ry, Scalar(2));
  const Scalar _tmp2 = std::sqrt(_tmp1);
  const Scalar _tmp3 = (Scalar(1) / Scalar(2)) * _tmp2;
  const Scalar _tmp4 = std::cos(_tmp3);
  const Scalar _tmp5 = _tmp4 * rot_init_z;
  const Scalar _tmp6 = std::sin(_tmp3);
  const Scalar _tmp7 = _tmp6 / _tmp2;
  const Scalar _tmp8 = _tmp7 * rot_init_x;
  const Scalar _tmp9 = _tmp7 * rot_init_y;
  const Scalar _tmp10 = _tmp7 * rot_init_w;
  const Scalar _tmp11 = _tmp10 * rz;
  const Scalar _tmp12 = _tmp11 + _tmp5 + _tmp8 * ry - _tmp9 * rx;
  const Scalar _tmp13 = _tmp4 * rot_init_y;
  const Scalar _tmp14 = _tmp7 * rot_init_z;
  const Scalar _tmp15 = _tmp8 * rz;
  const Scalar _tmp16 = _tmp10 * ry + _tmp13 + _tmp14 * rx - _tmp15;
  const Scalar _tmp17 = 2 * _tmp12 * _tmp16;
  const Scalar _tmp18 = _tmp4 * rot_init_x;
  const Scalar _tmp19 = _tmp9 * rz;
  const Scalar _tmp20 = _tmp10 * rx - _tmp14 * ry + _tmp18 + _tmp19;
  const Scalar _tmp21 = _tmp4 * rot_init_w;
  const Scalar _tmp22 = _tmp14 * rz;
  const Scalar _tmp23 = _tmp21 - _tmp22 - _tmp8 * rx - _tmp9 * ry;
  const Scalar _tmp24 = 2 * _tmp23;
  const Scalar _tmp25 = _tmp20 * _tmp24;
  const Scalar _tmp26 = Scalar(0.20999999999999999) * _tmp17 + Scalar(0.20999999999999999) * _tmp25;
  const Scalar _tmp27 = -_tmp26;
  const Scalar _tmp28 = -2 * std::pow(_tmp20, Scalar(2));
  const Scalar _tmp29 = 1 - 2 * std::pow(_tmp16, Scalar(2));
  const Scalar _tmp30 =
      -Scalar(0.010999999999999999) * _tmp28 - Scalar(0.010999999999999999) * _tmp29;
  const Scalar _tmp31 = 2 * _tmp20;
  const Scalar _tmp32 = _tmp12 * _tmp31;
  const Scalar _tmp33 = _tmp16 * _tmp24;
  const Scalar _tmp34 = Scalar(0.20999999999999999) * _tmp32 - Scalar(0.20999999999999999) * _tmp33;
  const Scalar _tmp35 = _tmp30 - _tmp34;
  const Scalar _tmp36 = _tmp27 + _tmp35;
  const Scalar _tmp37 = -_tmp36 - p_init2 + Scalar(8.4718465799999993);
  const Scalar _tmp38 = -2 * std::pow(_tmp12, Scalar(2));
  const Scalar _tmp39 = Scalar(0.20999999999999999) * _tmp28 +
                        Scalar(0.20999999999999999) * _tmp38 + Scalar(0.20999999999999999);
  const Scalar _tmp40 = -_tmp39;
  const Scalar _tmp41 = _tmp17 - _tmp25;
  const Scalar _tmp42 = -Scalar(0.010999999999999999) * _tmp41;
  const Scalar _tmp43 = _tmp16 * _tmp31;
  const Scalar _tmp44 = _tmp12 * _tmp24;
  const Scalar _tmp45 = Scalar(0.20999999999999999) * _tmp43 + Scalar(0.20999999999999999) * _tmp44;
  const Scalar _tmp46 = _tmp42 - _tmp45;
  const Scalar _tmp47 = _tmp40 + _tmp46;
  const Scalar _tmp48 = _tmp47 + p_init1;
  const Scalar _tmp49 = -_tmp48 + Scalar(-8.3196563700000006);
  const Scalar _tmp50 = Scalar(0.20999999999999999) * _tmp43 - Scalar(0.20999999999999999) * _tmp44;
  const Scalar _tmp51 = -_tmp50;
  const Scalar _tmp52 = _tmp32 + _tmp33;
  const Scalar _tmp53 = -Scalar(0.010999999999999999) * _tmp52;
  const Scalar _tmp54 = Scalar(0.20999999999999999) * _tmp29 + Scalar(0.20999999999999999) * _tmp38;
  const Scalar _tmp55 = _tmp53 - _tmp54;
  const Scalar _tmp56 = _tmp51 + _tmp55;
  const Scalar _tmp57 = _tmp56 + p_init0;
  const Scalar _tmp58 = -_tmp57 + Scalar(-1.9874742000000001);
  const Scalar _tmp59 = std::pow(_tmp49, Scalar(2)) + std::pow(_tmp58, Scalar(2));
  const Scalar _tmp60 = (Scalar(1) / Scalar(2)) / _tmp1;
  const Scalar _tmp61 = _tmp0 * _tmp60;
  const Scalar _tmp62 = _tmp60 * rz;
  const Scalar _tmp63 = _tmp62 * ry;
  const Scalar _tmp64 = _tmp62 * rx;
  const Scalar _tmp65 = _tmp6 / (_tmp1 * std::sqrt(_tmp1));
  const Scalar _tmp66 = _tmp0 * _tmp65;
  const Scalar _tmp67 = _tmp65 * rz;
  const Scalar _tmp68 = _tmp67 * rx;
  const Scalar _tmp69 = _tmp67 * ry;
  const Scalar _tmp70 = -Scalar(1) / Scalar(2) * _tmp11 - _tmp13 * _tmp63 - _tmp14 -
                        _tmp18 * _tmp64 - _tmp5 * _tmp61 + _tmp66 * rot_init_z +
                        _tmp68 * rot_init_x + _tmp69 * rot_init_y;
  const Scalar _tmp71 = Scalar(0.41999999999999998) * _tmp70;
  const Scalar _tmp72 = _tmp20 * _tmp71;
  const Scalar _tmp73 = -_tmp18 * _tmp61 - Scalar(1) / Scalar(2) * _tmp19 + _tmp21 * _tmp63 +
                        _tmp5 * _tmp64 + _tmp66 * rot_init_x - _tmp68 * rot_init_z -
                        _tmp69 * rot_init_w - _tmp8;
  const Scalar _tmp74 = Scalar(0.41999999999999998) * _tmp12;
  const Scalar _tmp75 = _tmp73 * _tmp74;
  const Scalar _tmp76 = _tmp10 - _tmp13 * _tmp64 + _tmp18 * _tmp63 + _tmp21 * _tmp61 -
                        Scalar(1) / Scalar(2) * _tmp22 - _tmp66 * rot_init_w + _tmp68 * rot_init_y -
                        _tmp69 * rot_init_x;
  const Scalar _tmp77 = Scalar(0.41999999999999998) * _tmp76;
  const Scalar _tmp78 = _tmp16 * _tmp77;
  const Scalar _tmp79 = _tmp13 * _tmp61 - Scalar(1) / Scalar(2) * _tmp15 + _tmp21 * _tmp64 -
                        _tmp5 * _tmp63 - _tmp66 * rot_init_y - _tmp68 * rot_init_w +
                        _tmp69 * rot_init_z + _tmp9;
  const Scalar _tmp80 = Scalar(0.41999999999999998) * _tmp23;
  const Scalar _tmp81 = _tmp79 * _tmp80;
  const Scalar _tmp82 = _tmp72 + _tmp75 + _tmp78 + _tmp81;
  const Scalar _tmp83 = _tmp20 * _tmp79;
  const Scalar _tmp84 = Scalar(0.043999999999999997) * _tmp83;
  const Scalar _tmp85 = _tmp16 * _tmp73;
  const Scalar _tmp86 = Scalar(0.043999999999999997) * _tmp85;
  const Scalar _tmp87 = -_tmp84 - _tmp86;
  const Scalar _tmp88 = _tmp82 + _tmp87;
  const Scalar _tmp89 = _tmp16 * _tmp71;
  const Scalar _tmp90 = _tmp74 * _tmp79;
  const Scalar _tmp91 = _tmp20 * _tmp77;
  const Scalar _tmp92 = _tmp73 * _tmp80;
  const Scalar _tmp93 = -_tmp89 + _tmp90 + _tmp91 - _tmp92;
  const Scalar _tmp94 = Scalar(0.021999999999999999) * _tmp70;
  const Scalar _tmp95 = _tmp20 * _tmp94;
  const Scalar _tmp96 = Scalar(0.021999999999999999) * _tmp12;
  const Scalar _tmp97 = _tmp73 * _tmp96;
  const Scalar _tmp98 = Scalar(0.021999999999999999) * _tmp76;
  const Scalar _tmp99 = _tmp16 * _tmp98;
  const Scalar _tmp100 = Scalar(0.021999999999999999) * _tmp23;
  const Scalar _tmp101 = _tmp100 * _tmp79;
  const Scalar _tmp102 = -_tmp101 - _tmp95 + _tmp97 + _tmp99;
  const Scalar _tmp103 = _tmp12 * _tmp76;
  const Scalar _tmp104 = Scalar(0.83999999999999997) * _tmp103;
  const Scalar _tmp105 = -_tmp104;
  const Scalar _tmp106 = Scalar(0.83999999999999997) * _tmp83;
  const Scalar _tmp107 = _tmp105 - _tmp106;
  const Scalar _tmp108 = _tmp102 + _tmp107;
  const Scalar _tmp109 = _tmp16 * _tmp79;
  const Scalar _tmp110 = Scalar(0.41999999999999998) * _tmp109;
  const Scalar _tmp111 = _tmp20 * _tmp73;
  const Scalar _tmp112 = Scalar(0.41999999999999998) * _tmp111;
  const Scalar _tmp113 = _tmp110 + _tmp112;
  const Scalar _tmp114 = _tmp70 * _tmp74;
  const Scalar _tmp115 = _tmp23 * _tmp77;
  const Scalar _tmp116 = _tmp114 + _tmp115;
  const Scalar _tmp117 = _tmp113 + _tmp116;
  const Scalar _tmp118 = -_tmp114 - _tmp115;
  const Scalar _tmp119 = _tmp113 + _tmp118;
  const Scalar _tmp120 = Scalar(0.83999999999999997) * _tmp85;
  const Scalar _tmp121 = _tmp105 - _tmp120;
  const Scalar _tmp122 = _tmp119 + _tmp121;
  const Scalar _tmp123 = _tmp16 * _tmp94;
  const Scalar _tmp124 = _tmp79 * _tmp96;
  const Scalar _tmp125 = _tmp20 * _tmp98;
  const Scalar _tmp126 = _tmp100 * _tmp73;
  const Scalar _tmp127 = _tmp123 + _tmp124 + _tmp125 + _tmp126;
  const Scalar _tmp128 = 2 * _tmp49 * (_tmp108 + _tmp117) + 2 * _tmp58 * (_tmp122 + _tmp127);
  const Scalar _tmp129 = std::sqrt(_tmp59);
  const Scalar _tmp130 = Scalar(1.0) / (fh1);
  const Scalar _tmp131 = -_tmp72 - _tmp75 - _tmp78 - _tmp81;
  const Scalar _tmp132 = _tmp131 + _tmp93;
  const Scalar _tmp133 = _tmp132 + _tmp87;
  const Scalar _tmp134 = _tmp26 + _tmp35;
  const Scalar _tmp135 = -_tmp134 - p_init2 + Scalar(8.3700199099999999);
  const Scalar _tmp136 = _tmp39 + _tmp46;
  const Scalar _tmp137 = _tmp136 + p_init1;
  const Scalar _tmp138 = Scalar(4.8333311099999996) - _tmp137;
  const Scalar _tmp139 = _tmp104 + _tmp106;
  const Scalar _tmp140 = _tmp117 + _tmp139;
  const Scalar _tmp141 = _tmp102 + _tmp140;
  const Scalar _tmp142 = -_tmp110 - _tmp112;
  const Scalar _tmp143 = _tmp116 + _tmp142;
  const Scalar _tmp144 = _tmp121 + _tmp143;
  const Scalar _tmp145 = _tmp50 + _tmp55;
  const Scalar _tmp146 = _tmp145 + p_init0;
  const Scalar _tmp147 = -_tmp146 + Scalar(-1.79662371);
  const Scalar _tmp148 = 2 * _tmp138 * _tmp141 + 2 * _tmp147 * (_tmp127 + _tmp144);
  const Scalar _tmp149 = std::pow(_tmp138, Scalar(2)) + std::pow(_tmp147, Scalar(2));
  const Scalar _tmp150 = _tmp146 + Scalar(1.79662371);
  const Scalar _tmp151 = Scalar(1.0) / (_tmp150);
  const Scalar _tmp152 = _tmp137 + Scalar(-4.8333311099999996);
  const Scalar _tmp153 = std::pow(_tmp150, Scalar(2));
  const Scalar _tmp154 = std::pow(_tmp152, Scalar(2)) + _tmp153;
  const Scalar _tmp155 = std::sqrt(_tmp154);
  const Scalar _tmp156 = Scalar(1.0) / (_tmp155);
  const Scalar _tmp157 = -_tmp123 - _tmp124 - _tmp125 - _tmp126;
  const Scalar _tmp158 = _tmp104 + _tmp120;
  const Scalar _tmp159 = _tmp119 + _tmp158;
  const Scalar _tmp160 = _tmp157 + _tmp159;
  const Scalar _tmp161 = _tmp118 + _tmp142;
  const Scalar _tmp162 = _tmp101 + _tmp95 - _tmp97 - _tmp99;
  const Scalar _tmp163 = _tmp107 + _tmp162;
  const Scalar _tmp164 = _tmp161 + _tmp163;
  const Scalar _tmp165 = _tmp150 * _tmp160 + _tmp152 * _tmp164;
  const Scalar _tmp166 = _tmp151 * _tmp156 * _tmp165;
  const Scalar _tmp167 = _tmp42 + _tmp45;
  const Scalar _tmp168 = _tmp167 + _tmp39;
  const Scalar _tmp169 = _tmp168 + p_init1;
  const Scalar _tmp170 = _tmp169 + Scalar(-4.7752063900000001);
  const Scalar _tmp171 = _tmp53 + _tmp54;
  const Scalar _tmp172 = _tmp171 + _tmp50;
  const Scalar _tmp173 = _tmp172 + p_init0;
  const Scalar _tmp174 = _tmp173 + Scalar(-2.71799795);
  const Scalar _tmp175 = std::pow(_tmp170, Scalar(2)) + std::pow(_tmp174, Scalar(2));
  const Scalar _tmp176 = std::pow(_tmp175, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp177 = _tmp174 * _tmp176;
  const Scalar _tmp178 = _tmp151 * _tmp152;
  const Scalar _tmp179 = _tmp170 * _tmp176;
  const Scalar _tmp180 = _tmp177 * _tmp178 - _tmp179;
  const Scalar _tmp181 = Scalar(1.0) / (_tmp180);
  const Scalar _tmp182 = _tmp171 + _tmp51;
  const Scalar _tmp183 = _tmp182 + p_init0;
  const Scalar _tmp184 = _tmp183 + Scalar(-2.5202214700000001);
  const Scalar _tmp185 = _tmp167 + _tmp40;
  const Scalar _tmp186 = _tmp185 + p_init1;
  const Scalar _tmp187 = _tmp186 + Scalar(8.3888750099999996);
  const Scalar _tmp188 = std::pow(_tmp184, Scalar(2)) + std::pow(_tmp187, Scalar(2));
  const Scalar _tmp189 = std::pow(_tmp188, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp190 = _tmp187 * _tmp189;
  const Scalar _tmp191 = _tmp184 * _tmp189;
  const Scalar _tmp192 = _tmp178 * _tmp191 - _tmp190;
  const Scalar _tmp193 = _tmp30 + _tmp34;
  const Scalar _tmp194 = _tmp193 + _tmp27;
  const Scalar _tmp195 = _tmp134 * _tmp191;
  const Scalar _tmp196 = _tmp134 * _tmp177;
  const Scalar _tmp197 = _tmp193 + _tmp26;
  const Scalar _tmp198 = -_tmp178 * _tmp196 + _tmp179 * _tmp197;
  const Scalar _tmp199 = _tmp181 * _tmp192;
  const Scalar _tmp200 = -_tmp178 * _tmp195 + _tmp190 * _tmp194 - _tmp198 * _tmp199;
  const Scalar _tmp201 = Scalar(1.0) * _tmp136;
  const Scalar _tmp202 = -_tmp201;
  const Scalar _tmp203 = _tmp168 + _tmp202;
  const Scalar _tmp204 = Scalar(1.0) / (_tmp203);
  const Scalar _tmp205 = Scalar(1.0) * _tmp145;
  const Scalar _tmp206 = -_tmp172 + _tmp205;
  const Scalar _tmp207 = _tmp204 * _tmp206;
  const Scalar _tmp208 = -_tmp177 * _tmp197 + _tmp196;
  const Scalar _tmp209 = _tmp181 * _tmp208;
  const Scalar _tmp210 = -_tmp191 * _tmp194 - _tmp192 * _tmp209 + _tmp195 - _tmp200 * _tmp207;
  const Scalar _tmp211 = Scalar(1.0) / (_tmp210);
  const Scalar _tmp212 = Scalar(1.0) * _tmp181;
  const Scalar _tmp213 = _tmp198 * _tmp212;
  const Scalar _tmp214 = _tmp207 * _tmp213 - _tmp208 * _tmp212;
  const Scalar _tmp215 = _tmp145 * _tmp156;
  const Scalar _tmp216 = _tmp136 * _tmp156;
  const Scalar _tmp217 = -_tmp150 * _tmp216 + _tmp152 * _tmp215;
  const Scalar _tmp218 = _tmp151 * _tmp155;
  const Scalar _tmp219 = _tmp217 * _tmp218;
  const Scalar _tmp220 = _tmp168 * _tmp177 - _tmp172 * _tmp179 + _tmp177 * _tmp219;
  const Scalar _tmp221 = _tmp181 * _tmp220;
  const Scalar _tmp222 =
      -_tmp182 * _tmp190 + _tmp185 * _tmp191 + _tmp191 * _tmp219 - _tmp192 * _tmp221;
  const Scalar _tmp223 = _tmp211 * _tmp222;
  const Scalar _tmp224 = -_tmp214 * _tmp223 - Scalar(1.0) * _tmp221;
  const Scalar _tmp225 = Scalar(1.0) / (_tmp222);
  const Scalar _tmp226 = _tmp210 * _tmp225;
  const Scalar _tmp227 = _tmp224 * _tmp226;
  const Scalar _tmp228 = _tmp214 + _tmp227;
  const Scalar _tmp229 = _tmp211 * _tmp228;
  const Scalar _tmp230 = -_tmp192 * _tmp229 + Scalar(1.0);
  const Scalar _tmp231 = _tmp181 * _tmp230;
  const Scalar _tmp232 = _tmp177 * _tmp231 + _tmp191 * _tmp229;
  const Scalar _tmp233 = _tmp48 + Scalar(8.3196563700000006);
  const Scalar _tmp234 = _tmp57 + Scalar(1.9874742000000001);
  const Scalar _tmp235 = std::pow(_tmp233, Scalar(2)) + std::pow(_tmp234, Scalar(2));
  const Scalar _tmp236 = std::pow(_tmp235, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp237 = _tmp233 * _tmp236;
  const Scalar _tmp238 = _tmp237 * fh1;
  const Scalar _tmp239 = _tmp232 * _tmp238;
  const Scalar _tmp240 = _tmp143 + _tmp158;
  const Scalar _tmp241 = _tmp157 + _tmp240;
  const Scalar _tmp242 = _tmp236 * _tmp241;
  const Scalar _tmp243 = _tmp139 + _tmp161;
  const Scalar _tmp244 = _tmp162 + _tmp243;
  const Scalar _tmp245 =
      (2 * _tmp233 * _tmp244 + 2 * _tmp234 * _tmp241) / (_tmp235 * std::sqrt(_tmp235));
  const Scalar _tmp246 = (Scalar(1) / Scalar(2)) * _tmp245;
  const Scalar _tmp247 = _tmp233 * _tmp246;
  const Scalar _tmp248 = _tmp236 * _tmp244;
  const Scalar _tmp249 = _tmp234 * _tmp246;
  const Scalar _tmp250 = _tmp236 * _tmp56;
  const Scalar _tmp251 = _tmp236 * _tmp47;
  const Scalar _tmp252 = fh1 * (_tmp233 * _tmp242 - _tmp234 * _tmp248 - _tmp241 * _tmp251 +
                                _tmp244 * _tmp250 - _tmp247 * _tmp56 + _tmp249 * _tmp47);
  const Scalar _tmp253 = _tmp212 * _tmp225;
  const Scalar _tmp254 = _tmp192 * _tmp253;
  const Scalar _tmp255 = Scalar(1.0) * _tmp225;
  const Scalar _tmp256 = -_tmp177 * _tmp254 + _tmp191 * _tmp255;
  const Scalar _tmp257 = _tmp218 * _tmp256;
  const Scalar _tmp258 = _tmp248 * fh1;
  const Scalar _tmp259 = _tmp218 * _tmp232;
  const Scalar _tmp260 = _tmp242 * fh1;
  const Scalar _tmp261 = _tmp134 * _tmp178;
  const Scalar _tmp262 = _tmp181 * _tmp198;
  const Scalar _tmp263 = _tmp178 * _tmp262 + _tmp261;
  const Scalar _tmp264 = -_tmp134 + _tmp178 * _tmp209 - _tmp207 * _tmp263;
  const Scalar _tmp265 = _tmp178 * _tmp221 - _tmp219 - _tmp223 * _tmp264;
  const Scalar _tmp266 = _tmp226 * _tmp265;
  const Scalar _tmp267 = _tmp264 + _tmp266;
  const Scalar _tmp268 = _tmp211 * _tmp267;
  const Scalar _tmp269 = -_tmp178 - _tmp192 * _tmp268;
  const Scalar _tmp270 = _tmp181 * _tmp269;
  const Scalar _tmp271 = _tmp177 * _tmp270 + _tmp191 * _tmp268 + Scalar(1.0);
  const Scalar _tmp272 = _tmp218 * _tmp271;
  const Scalar _tmp273 = _tmp234 * _tmp236;
  const Scalar _tmp274 = _tmp273 * fh1;
  const Scalar _tmp275 = _tmp271 * _tmp274;
  const Scalar _tmp276 = Scalar(1.0) / (_tmp153);
  const Scalar _tmp277 = _tmp155 * _tmp160 * _tmp276;
  const Scalar _tmp278 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp279 = _tmp201 * _tmp207 + _tmp205;
  const Scalar _tmp280 = 0;
  const Scalar _tmp281 = _tmp211 * _tmp280;
  const Scalar _tmp282 = _tmp199 * _tmp281;
  const Scalar _tmp283 = -_tmp177 * _tmp282 + _tmp191 * _tmp281;
  const Scalar _tmp284 = _tmp278 * _tmp283;
  const Scalar _tmp285 = fh1 * (_tmp233 * _tmp250 - _tmp234 * _tmp251);
  const Scalar _tmp286 = _tmp256 * _tmp285;
  const Scalar _tmp287 = _tmp249 * fh1;
  const Scalar _tmp288 = _tmp144 + _tmp157;
  const Scalar _tmp289 = _tmp140 + _tmp162;
  const Scalar _tmp290 =
      (2 * _tmp184 * _tmp288 + 2 * _tmp187 * _tmp289) / (_tmp188 * std::sqrt(_tmp188));
  const Scalar _tmp291 = (Scalar(1) / Scalar(2)) * _tmp290;
  const Scalar _tmp292 = _tmp184 * _tmp291;
  const Scalar _tmp293 = _tmp117 + _tmp163;
  const Scalar _tmp294 = _tmp122 + _tmp157;
  const Scalar _tmp295 =
      (2 * _tmp170 * _tmp293 + 2 * _tmp174 * _tmp294) / (_tmp175 * std::sqrt(_tmp175));
  const Scalar _tmp296 = (Scalar(1) / Scalar(2)) * _tmp295;
  const Scalar _tmp297 = _tmp174 * _tmp296;
  const Scalar _tmp298 = _tmp176 * _tmp294;
  const Scalar _tmp299 = _tmp189 * _tmp288;
  const Scalar _tmp300 = _tmp151 * _tmp164;
  const Scalar _tmp301 = _tmp152 * _tmp160;
  const Scalar _tmp302 = _tmp276 * _tmp301;
  const Scalar _tmp303 = _tmp189 * _tmp289;
  const Scalar _tmp304 = _tmp187 * _tmp291;
  const Scalar _tmp305 = -_tmp178 * _tmp292 + _tmp178 * _tmp299 + _tmp191 * _tmp300 -
                         _tmp191 * _tmp302 - _tmp303 + _tmp304;
  const Scalar _tmp306 = _tmp177 * _tmp181;
  const Scalar _tmp307 = _tmp170 * _tmp296;
  const Scalar _tmp308 = _tmp176 * _tmp293;
  const Scalar _tmp309 = (_tmp177 * _tmp300 - _tmp177 * _tmp302 - _tmp178 * _tmp297 +
                          _tmp178 * _tmp298 + _tmp307 - _tmp308) /
                         std::pow(_tmp180, Scalar(2));
  const Scalar _tmp310 = _tmp192 * _tmp309;
  const Scalar _tmp311 = _tmp177 * _tmp310;
  const Scalar _tmp312 = _tmp89 - _tmp90 - _tmp91 + _tmp92;
  const Scalar _tmp313 = _tmp84 + _tmp86;
  const Scalar _tmp314 = _tmp313 + _tmp82;
  const Scalar _tmp315 = _tmp312 + _tmp314;
  const Scalar _tmp316 = _tmp178 * _tmp315;
  const Scalar _tmp317 = _tmp134 * _tmp298;
  const Scalar _tmp318 = _tmp314 + _tmp93;
  const Scalar _tmp319 = -_tmp177 * _tmp316 - _tmp178 * _tmp317 + _tmp179 * _tmp318 -
                         _tmp196 * _tmp300 + _tmp196 * _tmp302 - _tmp197 * _tmp307 +
                         _tmp197 * _tmp308 + _tmp261 * _tmp297;
  const Scalar _tmp320 = _tmp191 * _tmp315;
  const Scalar _tmp321 = _tmp132 + _tmp313;
  const Scalar _tmp322 = -_tmp178 * _tmp320 + _tmp190 * _tmp321 + _tmp194 * _tmp303 -
                         _tmp194 * _tmp304 - _tmp195 * _tmp300 + _tmp195 * _tmp302 +
                         _tmp198 * _tmp310 - _tmp199 * _tmp319 + _tmp261 * _tmp292 -
                         _tmp261 * _tmp299 - _tmp262 * _tmp305;
  const Scalar _tmp323 = Scalar(1.6799999999999999) * _tmp103;
  const Scalar _tmp324 = _tmp323 + Scalar(1.6799999999999999) * _tmp85;
  const Scalar _tmp325 = _tmp204 * _tmp324;
  const Scalar _tmp326 = -_tmp134 * _tmp297 + _tmp177 * _tmp315 - _tmp177 * _tmp318 +
                         _tmp197 * _tmp297 - _tmp197 * _tmp298 + _tmp317;
  const Scalar _tmp327 = Scalar(0.83999999999999997) * _tmp12 * _tmp70;
  const Scalar _tmp328 = Scalar(0.83999999999999997) * _tmp23 * _tmp76;
  const Scalar _tmp329 =
      Scalar(0.83999999999999997) * _tmp109 + Scalar(0.83999999999999997) * _tmp111;
  const Scalar _tmp330 = _tmp327 + _tmp328 + _tmp329;
  const Scalar _tmp331 = _tmp330 / std::pow(_tmp203, Scalar(2));
  const Scalar _tmp332 = _tmp206 * _tmp331;
  const Scalar _tmp333 = -_tmp134 * _tmp292 + _tmp134 * _tmp299 - _tmp191 * _tmp321 +
                         _tmp194 * _tmp292 - _tmp194 * _tmp299 - _tmp199 * _tmp326 -
                         _tmp200 * _tmp325 + _tmp200 * _tmp332 - _tmp207 * _tmp322 +
                         _tmp208 * _tmp310 - _tmp209 * _tmp305 + _tmp320;
  const Scalar _tmp334 = _tmp333 / std::pow(_tmp210, Scalar(2));
  const Scalar _tmp335 = _tmp280 * _tmp334;
  const Scalar _tmp336 = _tmp218 * _tmp278;
  const Scalar _tmp337 = _tmp267 * _tmp334;
  const Scalar _tmp338 = _tmp225 * _tmp333;
  const Scalar _tmp339 = _tmp265 * _tmp338;
  const Scalar _tmp340 = _tmp165 / (_tmp154 * std::sqrt(_tmp154));
  const Scalar _tmp341 = _tmp218 * (_tmp136 * _tmp150 * _tmp340 - _tmp145 * _tmp152 * _tmp340 -
                                    _tmp150 * _tmp156 * _tmp164 + _tmp156 * _tmp301 -
                                    _tmp160 * _tmp216 + _tmp164 * _tmp215);
  const Scalar _tmp342 = _tmp217 * _tmp277;
  const Scalar _tmp343 = _tmp166 * _tmp217;
  const Scalar _tmp344 = -_tmp168 * _tmp297 + _tmp168 * _tmp298 + _tmp172 * _tmp307 -
                         _tmp172 * _tmp308 + _tmp174 * _tmp308 + _tmp177 * _tmp341 -
                         _tmp177 * _tmp342 + _tmp177 * _tmp343 - _tmp179 * _tmp294 -
                         _tmp219 * _tmp297 + _tmp219 * _tmp298;
  const Scalar _tmp345 = -_tmp182 * _tmp303 + _tmp182 * _tmp304 + _tmp184 * _tmp303 -
                         _tmp185 * _tmp292 + _tmp185 * _tmp299 - _tmp190 * _tmp288 +
                         _tmp191 * _tmp341 - _tmp191 * _tmp342 + _tmp191 * _tmp343 -
                         _tmp199 * _tmp344 - _tmp219 * _tmp292 + _tmp219 * _tmp299 +
                         _tmp220 * _tmp310 - _tmp221 * _tmp305;
  const Scalar _tmp346 = _tmp345 / std::pow(_tmp222, Scalar(2));
  const Scalar _tmp347 = _tmp210 * _tmp346;
  const Scalar _tmp348 = _tmp265 * _tmp347;
  const Scalar _tmp349 = _tmp178 * _tmp181;
  const Scalar _tmp350 = _tmp178 * _tmp309;
  const Scalar _tmp351 = _tmp134 * _tmp300 - _tmp134 * _tmp302 - _tmp198 * _tmp350 +
                         _tmp262 * _tmp300 - _tmp262 * _tmp302 + _tmp316 + _tmp319 * _tmp349;
  const Scalar _tmp352 = _tmp133 - _tmp207 * _tmp351 - _tmp208 * _tmp350 + _tmp209 * _tmp300 -
                         _tmp209 * _tmp302 - _tmp263 * _tmp325 + _tmp263 * _tmp332 +
                         _tmp326 * _tmp349;
  const Scalar _tmp353 = _tmp211 * _tmp345;
  const Scalar _tmp354 = _tmp222 * _tmp334;
  const Scalar _tmp355 = _tmp226 * (-_tmp220 * _tmp350 + _tmp221 * _tmp300 - _tmp221 * _tmp302 -
                                    _tmp223 * _tmp352 - _tmp264 * _tmp353 + _tmp264 * _tmp354 -
                                    _tmp341 + _tmp342 - _tmp343 + _tmp344 * _tmp349);
  const Scalar _tmp356 = _tmp211 * (_tmp339 - _tmp348 + _tmp352 + _tmp355);
  const Scalar _tmp357 =
      _tmp192 * _tmp337 - _tmp192 * _tmp356 - _tmp268 * _tmp305 - _tmp300 + _tmp302;
  const Scalar _tmp358 = _tmp177 * _tmp309;
  const Scalar _tmp359 = _tmp253 * _tmp305;
  const Scalar _tmp360 = _tmp192 * _tmp212 * _tmp346;
  const Scalar _tmp361 = Scalar(0.5) * _tmp225;
  const Scalar _tmp362 = Scalar(1.0) * _tmp346;
  const Scalar _tmp363 = _tmp247 * fh1;
  const Scalar _tmp364 = Scalar(1.0) * _tmp309;
  const Scalar _tmp365 = _tmp212 * _tmp319;
  const Scalar _tmp366 = _tmp198 * _tmp364;
  const Scalar _tmp367 = Scalar(1.0) * _tmp204;
  const Scalar _tmp368 = _tmp207 * _tmp365 - _tmp207 * _tmp366 + _tmp208 * _tmp364 -
                         _tmp212 * _tmp326 - _tmp213 * _tmp332 + _tmp262 * _tmp324 * _tmp367;
  const Scalar _tmp369 = _tmp226 * (-_tmp212 * _tmp344 - _tmp214 * _tmp353 + _tmp214 * _tmp354 +
                                    _tmp220 * _tmp364 - _tmp223 * _tmp368);
  const Scalar _tmp370 = _tmp224 * _tmp347;
  const Scalar _tmp371 = _tmp224 * _tmp338;
  const Scalar _tmp372 = _tmp211 * (_tmp368 + _tmp369 - _tmp370 + _tmp371);
  const Scalar _tmp373 = _tmp228 * _tmp334;
  const Scalar _tmp374 = -_tmp192 * _tmp372 + _tmp192 * _tmp373 - _tmp229 * _tmp305;
  const Scalar _tmp375 =
      -_tmp166 * _tmp239 - _tmp166 * _tmp275 - _tmp166 * _tmp284 - _tmp166 * _tmp286 -
      _tmp218 * _tmp238 *
          (_tmp191 * _tmp372 - _tmp191 * _tmp373 - _tmp229 * _tmp292 + _tmp229 * _tmp299 -
           _tmp230 * _tmp358 - _tmp231 * _tmp297 + _tmp231 * _tmp298 + _tmp306 * _tmp374) -
      _tmp218 * _tmp274 *
          (-_tmp191 * _tmp337 + _tmp191 * _tmp356 - _tmp268 * _tmp292 + _tmp268 * _tmp299 -
           _tmp269 * _tmp358 - _tmp270 * _tmp297 + _tmp270 * _tmp298 + _tmp306 * _tmp357) -
      _tmp218 * _tmp285 *
          (_tmp174 * _tmp199 * _tmp295 * _tmp361 - _tmp177 * _tmp359 + _tmp177 * _tmp360 -
           _tmp184 * _tmp290 * _tmp361 - _tmp191 * _tmp362 - _tmp254 * _tmp298 + _tmp255 * _tmp299 +
           _tmp255 * _tmp311) +
      _tmp239 * _tmp277 - _tmp252 * _tmp257 - _tmp258 * _tmp259 + _tmp259 * _tmp363 -
      _tmp260 * _tmp272 + _tmp272 * _tmp287 + _tmp275 * _tmp277 + _tmp277 * _tmp284 +
      _tmp277 * _tmp286 -
      _tmp336 *
          (_tmp177 * _tmp199 * _tmp335 - _tmp191 * _tmp335 - _tmp281 * _tmp292 + _tmp281 * _tmp299 -
           _tmp281 * _tmp305 * _tmp306 + _tmp281 * _tmp311 + _tmp282 * _tmp297 - _tmp282 * _tmp298);
  const Scalar _tmp376 =
      -_tmp238 * _tmp259 - _tmp257 * _tmp285 - _tmp272 * _tmp274 - _tmp283 * _tmp336;
  const Scalar _tmp377 = Scalar(1.0) / (_tmp376);
  const Scalar _tmp378 = _tmp185 + _tmp202;
  const Scalar _tmp379 = _tmp207 * _tmp378;
  const Scalar _tmp380 = -_tmp182 + _tmp205 - _tmp379;
  const Scalar _tmp381 = Scalar(1.0) / (_tmp380);
  const Scalar _tmp382 = Scalar(1.0) * _tmp381;
  const Scalar _tmp383 = _tmp378 * _tmp381;
  const Scalar _tmp384 = -_tmp200 * _tmp229 - _tmp213 + _tmp227 * _tmp383;
  const Scalar _tmp385 = _tmp227 * _tmp382 - _tmp367 * _tmp384;
  const Scalar _tmp386 = Scalar(1.0) * _tmp238;
  const Scalar _tmp387 = _tmp226 * _tmp382;
  const Scalar _tmp388 = -_tmp200 * _tmp255 + _tmp378 * _tmp387;
  const Scalar _tmp389 = -_tmp367 * _tmp388 + _tmp387;
  const Scalar _tmp390 = Scalar(1.0) * _tmp285;
  const Scalar _tmp391 = -_tmp200 * _tmp268 + _tmp263 + _tmp266 * _tmp383;
  const Scalar _tmp392 = _tmp266 * _tmp382 - _tmp367 * _tmp391;
  const Scalar _tmp393 = Scalar(1.0) * _tmp392;
  const Scalar _tmp394 = _tmp204 * _tmp382;
  const Scalar _tmp395 = _tmp378 * _tmp394 - _tmp382;
  const Scalar _tmp396 = _tmp36 * fh1;
  const Scalar _tmp397 = _tmp273 * _tmp396 + Scalar(3.29616) * _tmp52 + _tmp56 * fv1;
  const Scalar _tmp398 = Scalar(1.0) * _tmp397;
  const Scalar _tmp399 = -_tmp200 * _tmp281 + _tmp202 - _tmp279 * _tmp383;
  const Scalar _tmp400 = Scalar(1.0) * _tmp278;
  const Scalar _tmp401 = -_tmp237 * _tmp396 - Scalar(3.29616) * _tmp41 - _tmp47 * fv1;
  const Scalar _tmp402 = _tmp379 * _tmp382 + Scalar(1.0);
  const Scalar _tmp403 = _tmp207 * _tmp382;
  const Scalar _tmp404 = -Scalar(1.0) * _tmp367 * _tmp402 + Scalar(1.0) * _tmp403;
  const Scalar _tmp405 =
      _tmp274 * _tmp393 + _tmp385 * _tmp386 + _tmp389 * _tmp390 + _tmp395 * _tmp398 +
      _tmp400 * (-_tmp279 * _tmp382 - _tmp367 * _tmp399 + Scalar(1.0)) + _tmp401 * _tmp404;
  const Scalar _tmp406 = std::asinh(_tmp377 * _tmp405);
  const Scalar _tmp407 = Scalar(9.6622558468725703) * _tmp406;
  const Scalar _tmp408 = std::sqrt(_tmp149);
  const Scalar _tmp409 = -_tmp376 * _tmp407 - _tmp408;
  const Scalar _tmp410 = Scalar(0.1034955) * _tmp377;
  const Scalar _tmp411 = _tmp409 * _tmp410;
  const Scalar _tmp412 = Scalar(1.0) * _tmp406;
  const Scalar _tmp413 = std::pow(_tmp376, Scalar(-2));
  const Scalar _tmp414 = _tmp375 * _tmp413;
  const Scalar _tmp415 = Scalar(9.6622558468725703) * _tmp376;
  const Scalar _tmp416 = Scalar(1.0) * _tmp331;
  const Scalar _tmp417 = _tmp347 * _tmp382;
  const Scalar _tmp418 = _tmp323 + _tmp330 + Scalar(1.6799999999999999) * _tmp83;
  const Scalar _tmp419 = _tmp207 * _tmp418;
  const Scalar _tmp420 = _tmp325 * _tmp378;
  const Scalar _tmp421 = _tmp332 * _tmp378;
  const Scalar _tmp422 = (_tmp324 - _tmp327 - _tmp328 + _tmp329 - _tmp419 - _tmp420 + _tmp421) /
                         std::pow(_tmp380, Scalar(2));
  const Scalar _tmp423 = Scalar(1.0) * _tmp422;
  const Scalar _tmp424 = _tmp226 * _tmp423;
  const Scalar _tmp425 = _tmp338 * _tmp382;
  const Scalar _tmp426 = _tmp200 * _tmp362 - _tmp255 * _tmp322 - _tmp378 * _tmp417 -
                         _tmp378 * _tmp424 + _tmp378 * _tmp425 + _tmp387 * _tmp418;
  const Scalar _tmp427 = Scalar(6.59232) * _tmp70;
  const Scalar _tmp428 = Scalar(6.59232) * _tmp12;
  const Scalar _tmp429 = Scalar(6.59232) * _tmp76;
  const Scalar _tmp430 = Scalar(6.59232) * _tmp23;
  const Scalar _tmp431 = _tmp131 + _tmp312;
  const Scalar _tmp432 = fh1 * (_tmp313 + _tmp431);
  const Scalar _tmp433 = -_tmp16 * _tmp429 + _tmp20 * _tmp427 - _tmp237 * _tmp432 - _tmp244 * fv1 +
                         _tmp247 * _tmp396 - _tmp248 * _tmp396 - _tmp428 * _tmp73 +
                         _tmp430 * _tmp79;
  const Scalar _tmp434 = Scalar(0.5) * _tmp245 * fh1;
  const Scalar _tmp435 =
      -_tmp379 * _tmp423 + _tmp382 * _tmp419 + _tmp382 * _tmp420 - _tmp382 * _tmp421;
  const Scalar _tmp436 = _tmp332 * _tmp382;
  const Scalar _tmp437 = _tmp16 * _tmp427 + _tmp20 * _tmp429 + _tmp241 * fv1 + _tmp242 * _tmp396 -
                         _tmp249 * _tmp396 + _tmp273 * _tmp432 + _tmp428 * _tmp79 +
                         _tmp430 * _tmp73;
  const Scalar _tmp438 = _tmp378 * _tmp422;
  const Scalar _tmp439 = _tmp381 * _tmp418;
  const Scalar _tmp440 = -_tmp200 * _tmp372 + _tmp200 * _tmp373 - _tmp227 * _tmp438 +
                         _tmp227 * _tmp439 - _tmp229 * _tmp322 - _tmp365 + _tmp366 +
                         _tmp369 * _tmp383 - _tmp370 * _tmp383 + _tmp371 * _tmp383;
  const Scalar _tmp441 = _tmp200 * _tmp334;
  const Scalar _tmp442 = -_tmp200 * _tmp356 - _tmp266 * _tmp438 + _tmp266 * _tmp439 +
                         _tmp267 * _tmp441 - _tmp268 * _tmp322 + _tmp339 * _tmp383 -
                         _tmp348 * _tmp383 + _tmp351 + _tmp355 * _tmp383;
  const Scalar _tmp443 =
      _tmp160 + Scalar(1.0) * _tmp164 * _tmp207 + _tmp201 * _tmp325 - _tmp201 * _tmp332;
  const Scalar _tmp444 = _tmp279 * _tmp422;
  const Scalar _tmp445 = _tmp141 - _tmp279 * _tmp439 + _tmp280 * _tmp441 - _tmp281 * _tmp322 +
                         _tmp378 * _tmp444 - _tmp383 * _tmp443;
  const Scalar _tmp446 = _tmp331 * _tmp378;
  const Scalar _tmp447 =
      (_tmp377 *
           (-_tmp233 * _tmp385 * _tmp434 - _tmp234 * _tmp392 * _tmp434 +
            Scalar(1.0) * _tmp252 * _tmp389 + Scalar(1.0) * _tmp258 * _tmp385 + _tmp260 * _tmp393 +
            Scalar(1.0) * _tmp274 *
                (-_tmp266 * _tmp423 + _tmp339 * _tmp382 - _tmp348 * _tmp382 + _tmp355 * _tmp382 -
                 _tmp367 * _tmp442 + _tmp391 * _tmp416) +
            _tmp386 * (-_tmp227 * _tmp423 - _tmp367 * _tmp440 + _tmp369 * _tmp382 -
                       _tmp370 * _tmp382 + _tmp371 * _tmp382 + _tmp384 * _tmp416) +
            _tmp390 * (-_tmp367 * _tmp426 + _tmp388 * _tmp416 - _tmp417 - _tmp424 + _tmp425) +
            Scalar(1.0) * _tmp395 * _tmp437 +
            _tmp398 * (-_tmp367 * _tmp438 - _tmp382 * _tmp446 + _tmp394 * _tmp418 + _tmp423) +
            _tmp400 *
                (_tmp279 * _tmp423 - _tmp367 * _tmp445 - _tmp382 * _tmp443 + _tmp399 * _tmp416) +
            Scalar(1.0) * _tmp401 *
                (-_tmp207 * _tmp423 + _tmp325 * _tmp382 - _tmp367 * _tmp435 + _tmp402 * _tmp416 -
                 _tmp436) +
            _tmp404 * _tmp433) -
       _tmp405 * _tmp414) /
      std::sqrt(Scalar(std::pow(_tmp405, Scalar(2)) * _tmp413 + 1));
  const Scalar _tmp448 = _tmp278 * _tmp281;
  const Scalar _tmp449 =
      -_tmp199 * _tmp448 + _tmp231 * _tmp238 - _tmp254 * _tmp285 + _tmp270 * _tmp274;
  const Scalar _tmp450 = Scalar(1.0) / (_tmp449);
  const Scalar _tmp451 = _tmp204 * _tmp278;
  const Scalar _tmp452 = _tmp204 * _tmp388;
  const Scalar _tmp453 = _tmp204 * _tmp391;
  const Scalar _tmp454 = _tmp204 * _tmp384;
  const Scalar _tmp455 = _tmp382 * _tmp397;
  const Scalar _tmp456 = _tmp204 * _tmp378;
  const Scalar _tmp457 = _tmp204 * _tmp401;
  const Scalar _tmp458 = _tmp238 * _tmp454 + _tmp274 * _tmp453 + _tmp285 * _tmp452 +
                         _tmp399 * _tmp451 + _tmp402 * _tmp457 - _tmp455 * _tmp456;
  const Scalar _tmp459 = std::asinh(_tmp450 * _tmp458);
  const Scalar _tmp460 = Scalar(9.6622558468725703) * _tmp449;
  const Scalar _tmp461 = Scalar(4.7752063900000001) - _tmp169;
  const Scalar _tmp462 = Scalar(2.71799795) - _tmp173;
  const Scalar _tmp463 = std::pow(_tmp461, Scalar(2)) + std::pow(_tmp462, Scalar(2));
  const Scalar _tmp464 = std::sqrt(_tmp463);
  const Scalar _tmp465 = -_tmp459 * _tmp460 - _tmp464;
  const Scalar _tmp466 = Scalar(0.1034955) * _tmp450;
  const Scalar _tmp467 = _tmp465 * _tmp466;
  const Scalar _tmp468 = Scalar(1.0) * _tmp459;
  const Scalar _tmp469 = _tmp278 * _tmp335;
  const Scalar _tmp470 = _tmp255 * _tmp285;
  const Scalar _tmp471 =
      _tmp181 * _tmp238 * _tmp374 + _tmp181 * _tmp274 * _tmp357 - _tmp181 * _tmp305 * _tmp448 +
      _tmp199 * _tmp469 - _tmp230 * _tmp238 * _tmp309 + _tmp231 * _tmp258 - _tmp231 * _tmp363 -
      _tmp252 * _tmp254 + _tmp260 * _tmp270 - _tmp269 * _tmp274 * _tmp309 - _tmp270 * _tmp287 -
      _tmp285 * _tmp359 + _tmp285 * _tmp360 + _tmp310 * _tmp448 + _tmp310 * _tmp470;
  const Scalar _tmp472 = Scalar(9.6622558468725703) * _tmp471;
  const Scalar _tmp473 = std::pow(_tmp449, Scalar(-2));
  const Scalar _tmp474 = _tmp471 * _tmp473;
  const Scalar _tmp475 = 2 * _tmp461 * (_tmp102 + _tmp243) + 2 * _tmp462 * (_tmp127 + _tmp240);
  const Scalar _tmp476 = _tmp398 * _tmp422;
  const Scalar _tmp477 = _tmp382 * _tmp437;
  const Scalar _tmp478 =
      (_tmp450 * (_tmp204 * _tmp238 * _tmp440 + _tmp204 * _tmp274 * _tmp442 +
                  _tmp204 * _tmp285 * _tmp426 + _tmp204 * _tmp402 * _tmp433 -
                  _tmp204 * _tmp418 * _tmp455 - _tmp238 * _tmp331 * _tmp384 + _tmp252 * _tmp452 +
                  _tmp258 * _tmp454 + _tmp260 * _tmp453 - _tmp274 * _tmp331 * _tmp391 -
                  _tmp278 * _tmp331 * _tmp399 - _tmp285 * _tmp331 * _tmp388 - _tmp287 * _tmp453 -
                  _tmp331 * _tmp401 * _tmp402 - _tmp363 * _tmp454 + _tmp435 * _tmp457 +
                  _tmp445 * _tmp451 + _tmp446 * _tmp455 + _tmp456 * _tmp476 - _tmp456 * _tmp477) -
       _tmp458 * _tmp474) /
      std::sqrt(Scalar(std::pow(_tmp458, Scalar(2)) * _tmp473 + 1));
  const Scalar _tmp479 = -_tmp197 - p_init2 + Scalar(8.36416322);
  const Scalar _tmp480 = -_tmp186 + Scalar(-8.3888750099999996);
  const Scalar _tmp481 = Scalar(2.5202214700000001) - _tmp183;
  const Scalar _tmp482 = 2 * _tmp480 * (_tmp108 + _tmp161) + 2 * _tmp481 * (_tmp127 + _tmp159);
  const Scalar _tmp483 = std::pow(_tmp480, Scalar(2)) + std::pow(_tmp481, Scalar(2));
  const Scalar _tmp484 = std::sqrt(_tmp483);
  const Scalar _tmp485 = _tmp229 * _tmp238 + _tmp268 * _tmp274 + _tmp448 + _tmp470;
  const Scalar _tmp486 = Scalar(9.6622558468725703) * _tmp485;
  const Scalar _tmp487 = Scalar(1.0) / (_tmp485);
  const Scalar _tmp488 = _tmp238 * _tmp381;
  const Scalar _tmp489 = _tmp227 * _tmp381;
  const Scalar _tmp490 = _tmp278 * _tmp381;
  const Scalar _tmp491 = _tmp274 * _tmp381;
  const Scalar _tmp492 = _tmp266 * _tmp381;
  const Scalar _tmp493 = _tmp382 * _tmp457;
  const Scalar _tmp494 = -_tmp206 * _tmp493 - _tmp227 * _tmp488 - _tmp266 * _tmp491 +
                         _tmp279 * _tmp490 - _tmp285 * _tmp387 + _tmp455;
  const Scalar _tmp495 = std::pow(_tmp485, Scalar(-2));
  const Scalar _tmp496 = _tmp229 * _tmp258 - _tmp229 * _tmp363 + _tmp238 * _tmp372 -
                         _tmp238 * _tmp373 + _tmp252 * _tmp255 + _tmp260 * _tmp268 -
                         _tmp268 * _tmp287 - _tmp274 * _tmp337 + _tmp274 * _tmp356 -
                         _tmp285 * _tmp362 - _tmp469;
  const Scalar _tmp497 = _tmp495 * _tmp496;
  const Scalar _tmp498 =
      (_tmp487 * (_tmp206 * _tmp423 * _tmp457 + _tmp227 * _tmp238 * _tmp422 - _tmp252 * _tmp387 -
                  _tmp258 * _tmp489 - _tmp260 * _tmp492 + _tmp266 * _tmp274 * _tmp422 -
                  _tmp278 * _tmp444 + _tmp285 * _tmp417 + _tmp285 * _tmp424 - _tmp285 * _tmp425 +
                  _tmp287 * _tmp492 - _tmp324 * _tmp493 - _tmp339 * _tmp491 + _tmp348 * _tmp491 -
                  _tmp355 * _tmp491 + _tmp363 * _tmp489 - _tmp369 * _tmp488 + _tmp370 * _tmp488 -
                  _tmp371 * _tmp488 + _tmp401 * _tmp436 - _tmp403 * _tmp433 + _tmp443 * _tmp490 -
                  _tmp476 + _tmp477) -
       _tmp494 * _tmp497) /
      std::sqrt(Scalar(std::pow(_tmp494, Scalar(2)) * _tmp495 + 1));
  const Scalar _tmp499 = std::asinh(_tmp487 * _tmp494);
  const Scalar _tmp500 = Scalar(9.6622558468725703) * _tmp499;
  const Scalar _tmp501 = Scalar(0.1034955) * _tmp487;
  const Scalar _tmp502 = -_tmp484 - _tmp485 * _tmp500;
  const Scalar _tmp503 = _tmp501 * _tmp502;
  const Scalar _tmp504 = Scalar(1.0) * _tmp499;
  const Scalar _tmp505 = -_tmp194 - p_init2 + Scalar(8.4693136199999994);

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) =
      Scalar(0.5) * _tmp128 *
          std::cosh(Scalar(0.1034955) * _tmp130 *
                    (-_tmp129 - Scalar(9.6622558468725703) * fh1 * std::asinh(_tmp130 * fv1))) /
          _tmp129 -
      Scalar(1) / Scalar(2) * (_tmp128 + 2 * _tmp37 * (_tmp88 + _tmp93)) /
          std::sqrt(Scalar(std::pow(_tmp37, Scalar(2)) + _tmp59));
  _res(1, 0) = Scalar(9.6622558468725703) * _tmp375 * (-std::sinh(_tmp411) - std::sinh(_tmp412)) +
               _tmp415 * (-Scalar(1.0) * _tmp447 * std::cosh(_tmp412) -
                          (-Scalar(0.1034955) * _tmp409 * _tmp414 +
                           _tmp410 * (-Scalar(1) / Scalar(2) * _tmp148 / _tmp408 -
                                      _tmp375 * _tmp407 - _tmp415 * _tmp447)) *
                              std::cosh(_tmp411)) -
               Scalar(1) / Scalar(2) * (2 * _tmp133 * _tmp135 + _tmp148) /
                   std::sqrt(Scalar(std::pow(_tmp135, Scalar(2)) + _tmp149));
  _res(2, 0) = _tmp460 * (-Scalar(1.0) * _tmp478 * std::cosh(_tmp468) -
                          (-Scalar(0.1034955) * _tmp465 * _tmp474 +
                           _tmp466 * (-_tmp459 * _tmp472 - _tmp460 * _tmp478 -
                                      Scalar(1) / Scalar(2) * _tmp475 / _tmp464)) *
                              std::cosh(_tmp467)) +
               _tmp472 * (-std::sinh(_tmp467) - std::sinh(_tmp468)) -
               Scalar(1) / Scalar(2) * (_tmp475 + 2 * _tmp479 * (_tmp431 + _tmp87)) /
                   std::sqrt(Scalar(_tmp463 + std::pow(_tmp479, Scalar(2))));
  _res(3, 0) = _tmp486 * (-Scalar(1.0) * _tmp498 * std::cosh(_tmp504) -
                          (-Scalar(0.1034955) * _tmp497 * _tmp502 +
                           _tmp501 * (-Scalar(1) / Scalar(2) * _tmp482 / _tmp484 -
                                      _tmp486 * _tmp498 - _tmp496 * _tmp500)) *
                              std::cosh(_tmp503)) +
               Scalar(9.6622558468725703) * _tmp496 * (-std::sinh(_tmp503) - std::sinh(_tmp504)) -
               Scalar(1) / Scalar(2) * (_tmp482 + 2 * _tmp505 * (_tmp312 + _tmp88)) /
                   std::sqrt(Scalar(_tmp483 + std::pow(_tmp505, Scalar(2))));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
