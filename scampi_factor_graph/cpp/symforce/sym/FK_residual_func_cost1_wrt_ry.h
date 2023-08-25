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
 * Symbolic function: FK_residual_func_cost1_wrt_ry
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
Eigen::Matrix<Scalar, 4, 1> FkResidualFuncCost1WrtRy(
    const Scalar fh1, const Scalar fv1, const Scalar rx, const Scalar ry, const Scalar rz,
    const Scalar tx, const Scalar ty, const Scalar tz, const Scalar rot_init_x,
    const Scalar rot_init_y, const Scalar rot_init_z, const Scalar rot_init_w, const Scalar lc0,
    const Scalar lc1, const Scalar lc2, const Scalar lc3, const Scalar epsilon) {
  // Total ops: 1632

  // Unused inputs
  (void)tz;
  (void)lc0;
  (void)lc1;
  (void)lc2;
  (void)lc3;
  (void)epsilon;

  // Input arrays

  // Intermediate terms (506)
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
  const Scalar _tmp28 = -_tmp27;
  const Scalar _tmp29 = -2 * std::pow(_tmp17, Scalar(2));
  const Scalar _tmp30 = -2 * std::pow(_tmp21, Scalar(2));
  const Scalar _tmp31 = Scalar(0.20999999999999999) * _tmp29 +
                        Scalar(0.20999999999999999) * _tmp30 + Scalar(0.20999999999999999);
  const Scalar _tmp32 = 2 * _tmp21;
  const Scalar _tmp33 = _tmp13 * _tmp32;
  const Scalar _tmp34 = _tmp17 * _tmp25;
  const Scalar _tmp35 = _tmp33 - _tmp34;
  const Scalar _tmp36 = -Scalar(0.010999999999999999) * _tmp35;
  const Scalar _tmp37 = -_tmp31 + _tmp36;
  const Scalar _tmp38 = _tmp28 + _tmp37;
  const Scalar _tmp39 = _tmp38 + ty;
  const Scalar _tmp40 = -_tmp39 + Scalar(-8.3196563700000006);
  const Scalar _tmp41 = 1 - 2 * std::pow(_tmp13, Scalar(2));
  const Scalar _tmp42 = Scalar(0.20999999999999999) * _tmp30 + Scalar(0.20999999999999999) * _tmp41;
  const Scalar _tmp43 = -_tmp42;
  const Scalar _tmp44 = Scalar(0.20999999999999999) * _tmp18 - Scalar(0.20999999999999999) * _tmp26;
  const Scalar _tmp45 = _tmp17 * _tmp32;
  const Scalar _tmp46 = _tmp13 * _tmp25;
  const Scalar _tmp47 = _tmp45 + _tmp46;
  const Scalar _tmp48 = -Scalar(0.010999999999999999) * _tmp47;
  const Scalar _tmp49 = -_tmp44 + _tmp48;
  const Scalar _tmp50 = _tmp43 + _tmp49;
  const Scalar _tmp51 = _tmp50 + tx;
  const Scalar _tmp52 = -_tmp51 + Scalar(-1.9874742000000001);
  const Scalar _tmp53 =
      std::sqrt(Scalar(std::pow(_tmp40, Scalar(2)) + std::pow(_tmp52, Scalar(2))));
  const Scalar _tmp54 = (Scalar(1) / Scalar(2)) / _tmp2;
  const Scalar _tmp55 = _tmp1 * _tmp54;
  const Scalar _tmp56 = _tmp54 * ry;
  const Scalar _tmp57 = _tmp56 * rz;
  const Scalar _tmp58 = _tmp56 * rx;
  const Scalar _tmp59 = _tmp7 / (_tmp2 * std::sqrt(_tmp2));
  const Scalar _tmp60 = _tmp1 * _tmp59;
  const Scalar _tmp61 = _tmp59 * ry;
  const Scalar _tmp62 = _tmp61 * rx;
  const Scalar _tmp63 = _tmp61 * rz;
  const Scalar _tmp64 = -_tmp11 - _tmp19 * _tmp55 - Scalar(1) / Scalar(2) * _tmp20 +
                        _tmp22 * _tmp58 + _tmp57 * _tmp6 + _tmp60 * rot_init_z -
                        _tmp62 * rot_init_w - _tmp63 * rot_init_y;
  const Scalar _tmp65 = Scalar(0.021999999999999999) * _tmp24;
  const Scalar _tmp66 = _tmp64 * _tmp65;
  const Scalar _tmp67 = -Scalar(1) / Scalar(2) * _tmp10 - _tmp14 * _tmp58 - _tmp16 -
                        _tmp19 * _tmp57 - _tmp55 * _tmp6 + _tmp60 * rot_init_y +
                        _tmp62 * rot_init_x + _tmp63 * rot_init_z;
  const Scalar _tmp68 = _tmp17 * _tmp67;
  const Scalar _tmp69 = Scalar(0.021999999999999999) * _tmp68;
  const Scalar _tmp70 = _tmp12 + _tmp14 * _tmp55 - Scalar(1) / Scalar(2) * _tmp15 +
                        _tmp22 * _tmp57 - _tmp58 * _tmp6 - _tmp60 * rot_init_x +
                        _tmp62 * rot_init_y - _tmp63 * rot_init_w;
  const Scalar _tmp71 = Scalar(0.021999999999999999) * _tmp70;
  const Scalar _tmp72 = _tmp13 * _tmp71;
  const Scalar _tmp73 = -_tmp14 * _tmp57 + _tmp19 * _tmp58 + _tmp22 * _tmp55 -
                        Scalar(1) / Scalar(2) * _tmp23 - _tmp60 * rot_init_w - _tmp62 * rot_init_z +
                        _tmp63 * rot_init_x + _tmp9;
  const Scalar _tmp74 = Scalar(0.021999999999999999) * _tmp21;
  const Scalar _tmp75 = _tmp73 * _tmp74;
  const Scalar _tmp76 = -_tmp66 - _tmp69 + _tmp72 + _tmp75;
  const Scalar _tmp77 = Scalar(0.83999999999999997) * _tmp70;
  const Scalar _tmp78 = _tmp21 * _tmp77;
  const Scalar _tmp79 = -_tmp78;
  const Scalar _tmp80 = Scalar(0.83999999999999997) * _tmp64;
  const Scalar _tmp81 = _tmp17 * _tmp80;
  const Scalar _tmp82 = _tmp79 - _tmp81;
  const Scalar _tmp83 = Scalar(0.41999999999999998) * _tmp24;
  const Scalar _tmp84 = _tmp70 * _tmp83;
  const Scalar _tmp85 = _tmp21 * _tmp67;
  const Scalar _tmp86 = Scalar(0.41999999999999998) * _tmp85;
  const Scalar _tmp87 = _tmp84 + _tmp86;
  const Scalar _tmp88 = Scalar(0.41999999999999998) * _tmp17;
  const Scalar _tmp89 = _tmp73 * _tmp88;
  const Scalar _tmp90 = Scalar(0.41999999999999998) * _tmp13;
  const Scalar _tmp91 = _tmp64 * _tmp90;
  const Scalar _tmp92 = _tmp89 + _tmp91;
  const Scalar _tmp93 = _tmp87 + _tmp92;
  const Scalar _tmp94 = _tmp82 + _tmp93;
  const Scalar _tmp95 = Scalar(0.83999999999999997) * _tmp73;
  const Scalar _tmp96 = _tmp13 * _tmp95;
  const Scalar _tmp97 = _tmp79 - _tmp96;
  const Scalar _tmp98 = _tmp65 * _tmp73;
  const Scalar _tmp99 = _tmp13 * _tmp67;
  const Scalar _tmp100 = Scalar(0.021999999999999999) * _tmp99;
  const Scalar _tmp101 = _tmp17 * _tmp71;
  const Scalar _tmp102 = _tmp64 * _tmp74;
  const Scalar _tmp103 = _tmp100 + _tmp101 + _tmp102 + _tmp98;
  const Scalar _tmp104 = -_tmp84 - _tmp86;
  const Scalar _tmp105 = _tmp104 + _tmp92;
  const Scalar _tmp106 = _tmp103 + _tmp105;
  const Scalar _tmp107 = _tmp64 * _tmp83;
  const Scalar _tmp108 = _tmp67 * _tmp88;
  const Scalar _tmp109 = _tmp70 * _tmp90;
  const Scalar _tmp110 = Scalar(0.41999999999999998) * _tmp21;
  const Scalar _tmp111 = _tmp110 * _tmp73;
  const Scalar _tmp112 = -_tmp107 - _tmp108 - _tmp109 - _tmp111;
  const Scalar _tmp113 = _tmp73 * _tmp83;
  const Scalar _tmp114 = Scalar(0.41999999999999998) * _tmp99;
  const Scalar _tmp115 = _tmp70 * _tmp88;
  const Scalar _tmp116 = -_tmp115;
  const Scalar _tmp117 = _tmp110 * _tmp64;
  const Scalar _tmp118 = -_tmp117;
  const Scalar _tmp119 = _tmp13 * _tmp73;
  const Scalar _tmp120 = Scalar(0.043999999999999997) * _tmp119;
  const Scalar _tmp121 = _tmp17 * _tmp64;
  const Scalar _tmp122 = Scalar(0.043999999999999997) * _tmp121;
  const Scalar _tmp123 = _tmp120 + _tmp122;
  const Scalar _tmp124 = _tmp113 + _tmp114 + _tmp116 + _tmp118 + _tmp123;
  const Scalar _tmp125 = _tmp112 + _tmp124;
  const Scalar _tmp126 = _tmp39 + Scalar(8.3196563700000006);
  const Scalar _tmp127 = _tmp51 + Scalar(1.9874742000000001);
  const Scalar _tmp128 = std::pow(_tmp126, Scalar(2)) + std::pow(_tmp127, Scalar(2));
  const Scalar _tmp129 = std::pow(_tmp128, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp130 = _tmp126 * _tmp129;
  const Scalar _tmp131 =
      Scalar(0.20999999999999999) * _tmp45 - Scalar(0.20999999999999999) * _tmp46;
  const Scalar _tmp132 = -_tmp131;
  const Scalar _tmp133 =
      -Scalar(0.010999999999999999) * _tmp29 - Scalar(0.010999999999999999) * _tmp41;
  const Scalar _tmp134 =
      Scalar(0.20999999999999999) * _tmp33 + Scalar(0.20999999999999999) * _tmp34;
  const Scalar _tmp135 = _tmp133 - _tmp134;
  const Scalar _tmp136 = fh1 * (_tmp132 + _tmp135);
  const Scalar _tmp137 = -_tmp130 * _tmp136 - Scalar(3.29616) * _tmp35 - _tmp38 * fv1;
  const Scalar _tmp138 = _tmp44 + _tmp48;
  const Scalar _tmp139 = _tmp138 + _tmp43;
  const Scalar _tmp140 = _tmp42 + _tmp49;
  const Scalar _tmp141 = Scalar(1.0) * _tmp140;
  const Scalar _tmp142 = _tmp27 + _tmp37;
  const Scalar _tmp143 = Scalar(1.0) * _tmp142;
  const Scalar _tmp144 = -_tmp143;
  const Scalar _tmp145 = _tmp31 + _tmp36;
  const Scalar _tmp146 = _tmp145 + _tmp28;
  const Scalar _tmp147 = _tmp144 + _tmp146;
  const Scalar _tmp148 = _tmp138 + _tmp42;
  const Scalar _tmp149 = _tmp141 - _tmp148;
  const Scalar _tmp150 = _tmp145 + _tmp27;
  const Scalar _tmp151 = _tmp144 + _tmp150;
  const Scalar _tmp152 = Scalar(1.0) / (_tmp151);
  const Scalar _tmp153 = _tmp149 * _tmp152;
  const Scalar _tmp154 = _tmp147 * _tmp153;
  const Scalar _tmp155 = -_tmp139 + _tmp141 - _tmp154;
  const Scalar _tmp156 = Scalar(1.0) / (_tmp155);
  const Scalar _tmp157 = Scalar(1.0) * _tmp156;
  const Scalar _tmp158 = _tmp154 * _tmp157 + Scalar(1.0);
  const Scalar _tmp159 = Scalar(1.0) * _tmp152;
  const Scalar _tmp160 = _tmp153 * _tmp157;
  const Scalar _tmp161 = -Scalar(1.0) * _tmp158 * _tmp159 + Scalar(1.0) * _tmp160;
  const Scalar _tmp162 = _tmp127 * _tmp129;
  const Scalar _tmp163 = _tmp136 * _tmp162 + Scalar(3.29616) * _tmp47 + _tmp50 * fv1;
  const Scalar _tmp164 = _tmp152 * _tmp157;
  const Scalar _tmp165 = Scalar(1.0) * _tmp147 * _tmp164 - Scalar(1.0) * _tmp157;
  const Scalar _tmp166 = _tmp131 + _tmp135;
  const Scalar _tmp167 = _tmp148 + tx;
  const Scalar _tmp168 = _tmp167 + Scalar(-2.71799795);
  const Scalar _tmp169 = _tmp150 + ty;
  const Scalar _tmp170 = _tmp169 + Scalar(-4.7752063900000001);
  const Scalar _tmp171 = std::pow(_tmp168, Scalar(2)) + std::pow(_tmp170, Scalar(2));
  const Scalar _tmp172 = std::pow(_tmp171, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp173 = _tmp168 * _tmp172;
  const Scalar _tmp174 = _tmp166 * _tmp173;
  const Scalar _tmp175 = _tmp133 + _tmp134;
  const Scalar _tmp176 = _tmp131 + _tmp175;
  const Scalar _tmp177 = -_tmp173 * _tmp176 + _tmp174;
  const Scalar _tmp178 = _tmp142 + ty;
  const Scalar _tmp179 = _tmp178 + Scalar(8.3888750099999996);
  const Scalar _tmp180 = _tmp140 + tx;
  const Scalar _tmp181 = _tmp180 + Scalar(-2.5202214700000001);
  const Scalar _tmp182 = Scalar(1.0) / (_tmp181);
  const Scalar _tmp183 = _tmp179 * _tmp182;
  const Scalar _tmp184 = _tmp170 * _tmp172;
  const Scalar _tmp185 = _tmp173 * _tmp183 - _tmp184;
  const Scalar _tmp186 = Scalar(1.0) / (_tmp185);
  const Scalar _tmp187 = Scalar(1.0) * _tmp186;
  const Scalar _tmp188 = -_tmp174 * _tmp183 + _tmp176 * _tmp184;
  const Scalar _tmp189 = _tmp187 * _tmp188;
  const Scalar _tmp190 = _tmp153 * _tmp189 - _tmp177 * _tmp187;
  const Scalar _tmp191 = _tmp139 + tx;
  const Scalar _tmp192 = _tmp191 + Scalar(1.79662371);
  const Scalar _tmp193 = _tmp146 + ty;
  const Scalar _tmp194 = _tmp193 + Scalar(-4.8333311099999996);
  const Scalar _tmp195 = std::pow(_tmp192, Scalar(2)) + std::pow(_tmp194, Scalar(2));
  const Scalar _tmp196 = std::pow(_tmp195, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp197 = _tmp192 * _tmp196;
  const Scalar _tmp198 = _tmp166 * _tmp197;
  const Scalar _tmp199 = _tmp132 + _tmp175;
  const Scalar _tmp200 = _tmp194 * _tmp196;
  const Scalar _tmp201 = _tmp183 * _tmp197 - _tmp200;
  const Scalar _tmp202 = _tmp186 * _tmp201;
  const Scalar _tmp203 = -_tmp183 * _tmp198 - _tmp188 * _tmp202 + _tmp199 * _tmp200;
  const Scalar _tmp204 = -_tmp153 * _tmp203 - _tmp177 * _tmp202 - _tmp197 * _tmp199 + _tmp198;
  const Scalar _tmp205 = Scalar(1.0) / (_tmp204);
  const Scalar _tmp206 = std::pow(_tmp181, Scalar(2));
  const Scalar _tmp207 = std::pow(_tmp179, Scalar(2)) + _tmp206;
  const Scalar _tmp208 = std::sqrt(_tmp207);
  const Scalar _tmp209 = Scalar(1.0) / (_tmp208);
  const Scalar _tmp210 = _tmp181 * _tmp209;
  const Scalar _tmp211 = _tmp140 * _tmp209;
  const Scalar _tmp212 = -_tmp142 * _tmp210 + _tmp179 * _tmp211;
  const Scalar _tmp213 = _tmp182 * _tmp208;
  const Scalar _tmp214 = _tmp212 * _tmp213;
  const Scalar _tmp215 = -_tmp148 * _tmp184 + _tmp150 * _tmp173 + _tmp173 * _tmp214;
  const Scalar _tmp216 =
      -_tmp139 * _tmp200 + _tmp146 * _tmp197 + _tmp197 * _tmp214 - _tmp202 * _tmp215;
  const Scalar _tmp217 = _tmp205 * _tmp216;
  const Scalar _tmp218 = -_tmp187 * _tmp215 - _tmp190 * _tmp217;
  const Scalar _tmp219 = Scalar(1.0) / (_tmp216);
  const Scalar _tmp220 = _tmp204 * _tmp219;
  const Scalar _tmp221 = _tmp218 * _tmp220;
  const Scalar _tmp222 = _tmp147 * _tmp156;
  const Scalar _tmp223 = _tmp190 + _tmp221;
  const Scalar _tmp224 = _tmp203 * _tmp205;
  const Scalar _tmp225 = -_tmp189 + _tmp221 * _tmp222 - _tmp223 * _tmp224;
  const Scalar _tmp226 = _tmp157 * _tmp221 - _tmp159 * _tmp225;
  const Scalar _tmp227 = Scalar(1.0) * _tmp226;
  const Scalar _tmp228 = _tmp130 * fh1;
  const Scalar _tmp229 = _tmp183 * _tmp186;
  const Scalar _tmp230 = _tmp166 * _tmp183;
  const Scalar _tmp231 = _tmp188 * _tmp229 + _tmp230;
  const Scalar _tmp232 = -_tmp153 * _tmp231 - _tmp166 + _tmp177 * _tmp229;
  const Scalar _tmp233 = -_tmp214 + _tmp215 * _tmp229 - _tmp217 * _tmp232;
  const Scalar _tmp234 = _tmp220 * _tmp233;
  const Scalar _tmp235 = _tmp232 + _tmp234;
  const Scalar _tmp236 = _tmp222 * _tmp234 - _tmp224 * _tmp235 + _tmp231;
  const Scalar _tmp237 = _tmp157 * _tmp234 - _tmp159 * _tmp236;
  const Scalar _tmp238 = _tmp162 * fh1;
  const Scalar _tmp239 = Scalar(1.0) * _tmp238;
  const Scalar _tmp240 = _tmp141 + _tmp143 * _tmp153;
  const Scalar _tmp241 = 0;
  const Scalar _tmp242 = _tmp156 * _tmp240;
  const Scalar _tmp243 = _tmp144 - _tmp147 * _tmp242 - _tmp224 * _tmp241;
  const Scalar _tmp244 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp245 = Scalar(1.0) * _tmp244;
  const Scalar _tmp246 = Scalar(1.0) * _tmp219;
  const Scalar _tmp247 = _tmp157 * _tmp220;
  const Scalar _tmp248 = _tmp147 * _tmp247 - _tmp203 * _tmp246;
  const Scalar _tmp249 = -_tmp159 * _tmp248 + _tmp247;
  const Scalar _tmp250 = fh1 * (_tmp130 * _tmp50 - _tmp162 * _tmp38);
  const Scalar _tmp251 = Scalar(1.0) * _tmp250;
  const Scalar _tmp252 =
      _tmp137 * _tmp161 + _tmp163 * _tmp165 + _tmp227 * _tmp228 + _tmp237 * _tmp239 +
      _tmp245 * (-_tmp157 * _tmp240 - _tmp159 * _tmp243 + Scalar(1.0)) + _tmp249 * _tmp251;
  const Scalar _tmp253 = _tmp197 * _tmp205;
  const Scalar _tmp254 = _tmp201 * _tmp205;
  const Scalar _tmp255 = -_tmp183 - _tmp235 * _tmp254;
  const Scalar _tmp256 = _tmp173 * _tmp186;
  const Scalar _tmp257 = _tmp235 * _tmp253 + _tmp255 * _tmp256 + Scalar(1.0);
  const Scalar _tmp258 = _tmp213 * _tmp238;
  const Scalar _tmp259 = _tmp187 * _tmp219;
  const Scalar _tmp260 = _tmp201 * _tmp259;
  const Scalar _tmp261 = -_tmp173 * _tmp260 + _tmp197 * _tmp246;
  const Scalar _tmp262 = _tmp213 * _tmp250;
  const Scalar _tmp263 = -_tmp223 * _tmp254 + Scalar(1.0);
  const Scalar _tmp264 = _tmp223 * _tmp253 + _tmp256 * _tmp263;
  const Scalar _tmp265 = _tmp213 * _tmp228;
  const Scalar _tmp266 = _tmp205 * _tmp241;
  const Scalar _tmp267 = _tmp173 * _tmp266;
  const Scalar _tmp268 = _tmp197 * _tmp266 - _tmp202 * _tmp267;
  const Scalar _tmp269 = _tmp213 * _tmp244;
  const Scalar _tmp270 =
      -_tmp257 * _tmp258 - _tmp261 * _tmp262 - _tmp264 * _tmp265 - _tmp268 * _tmp269;
  const Scalar _tmp271 = Scalar(1.0) / (_tmp270);
  const Scalar _tmp272 = std::asinh(_tmp252 * _tmp271);
  const Scalar _tmp273 = Scalar(1.0) * _tmp272;
  const Scalar _tmp274 = _tmp78 + _tmp96;
  const Scalar _tmp275 = -_tmp100 - _tmp101 - _tmp102 - _tmp98;
  const Scalar _tmp276 = _tmp105 + _tmp275;
  const Scalar _tmp277 = _tmp274 + _tmp276;
  const Scalar _tmp278 = _tmp196 * _tmp277;
  const Scalar _tmp279 = _tmp166 * _tmp278;
  const Scalar _tmp280 = -_tmp89 - _tmp91;
  const Scalar _tmp281 = _tmp104 + _tmp280;
  const Scalar _tmp282 = _tmp281 + _tmp82;
  const Scalar _tmp283 = _tmp66 + _tmp69 - _tmp72 - _tmp75;
  const Scalar _tmp284 = _tmp282 + _tmp283;
  const Scalar _tmp285 =
      (2 * _tmp192 * _tmp277 + 2 * _tmp194 * _tmp284) / (_tmp195 * std::sqrt(_tmp195));
  const Scalar _tmp286 = (Scalar(1) / Scalar(2)) * _tmp285;
  const Scalar _tmp287 = _tmp194 * _tmp286;
  const Scalar _tmp288 = _tmp78 + _tmp81;
  const Scalar _tmp289 = _tmp283 + _tmp288;
  const Scalar _tmp290 = _tmp289 + _tmp93;
  const Scalar _tmp291 = _tmp182 * _tmp290;
  const Scalar _tmp292 = _tmp276 + _tmp97;
  const Scalar _tmp293 = _tmp283 + _tmp94;
  const Scalar _tmp294 =
      (2 * _tmp168 * _tmp292 + 2 * _tmp170 * _tmp293) / (_tmp171 * std::sqrt(_tmp171));
  const Scalar _tmp295 = (Scalar(1) / Scalar(2)) * _tmp294;
  const Scalar _tmp296 = _tmp168 * _tmp295;
  const Scalar _tmp297 = -_tmp113 - _tmp114 + _tmp115 + _tmp117 + _tmp123;
  const Scalar _tmp298 = _tmp112 + _tmp297;
  const Scalar _tmp299 = _tmp173 * _tmp298;
  const Scalar _tmp300 = _tmp172 * _tmp293;
  const Scalar _tmp301 = _tmp172 * _tmp292;
  const Scalar _tmp302 = _tmp166 * _tmp301;
  const Scalar _tmp303 = _tmp170 * _tmp295;
  const Scalar _tmp304 = _tmp166 * _tmp291;
  const Scalar _tmp305 = _tmp107 + _tmp108 + _tmp109 + _tmp111;
  const Scalar _tmp306 = _tmp297 + _tmp305;
  const Scalar _tmp307 = _tmp280 + _tmp87;
  const Scalar _tmp308 = _tmp307 + _tmp97;
  const Scalar _tmp309 = _tmp275 + _tmp308;
  const Scalar _tmp310 = _tmp309 / _tmp206;
  const Scalar _tmp311 = _tmp179 * _tmp310;
  const Scalar _tmp312 = -_tmp173 * _tmp304 + _tmp174 * _tmp311 + _tmp176 * _tmp300 -
                         _tmp176 * _tmp303 - _tmp183 * _tmp299 - _tmp183 * _tmp302 +
                         _tmp184 * _tmp306 + _tmp230 * _tmp296;
  const Scalar _tmp313 = _tmp183 * _tmp298;
  const Scalar _tmp314 = _tmp124 + _tmp305;
  const Scalar _tmp315 = _tmp196 * _tmp284;
  const Scalar _tmp316 = _tmp192 * _tmp286;
  const Scalar _tmp317 = _tmp183 * _tmp278 - _tmp183 * _tmp316 + _tmp197 * _tmp291 -
                         _tmp197 * _tmp311 + _tmp287 - _tmp315;
  const Scalar _tmp318 = _tmp186 * _tmp317;
  const Scalar _tmp319 = (_tmp173 * _tmp291 - _tmp173 * _tmp311 - _tmp183 * _tmp296 +
                          _tmp183 * _tmp301 - _tmp300 + _tmp303) /
                         std::pow(_tmp185, Scalar(2));
  const Scalar _tmp320 = _tmp201 * _tmp319;
  const Scalar _tmp321 = -_tmp183 * _tmp279 - _tmp188 * _tmp318 + _tmp188 * _tmp320 -
                         _tmp197 * _tmp313 - _tmp198 * _tmp291 + _tmp198 * _tmp311 -
                         _tmp199 * _tmp287 + _tmp199 * _tmp315 + _tmp200 * _tmp314 -
                         _tmp202 * _tmp312 + _tmp230 * _tmp316;
  const Scalar _tmp322 = _tmp205 * _tmp321;
  const Scalar _tmp323 = _tmp186 * _tmp291;
  const Scalar _tmp324 = _tmp183 * _tmp319;
  const Scalar _tmp325 = _tmp186 * _tmp311;
  const Scalar _tmp326 = -_tmp166 * _tmp311 + _tmp188 * _tmp323 - _tmp188 * _tmp324 -
                         _tmp188 * _tmp325 + _tmp229 * _tmp312 + _tmp304 + _tmp313;
  const Scalar _tmp327 = _tmp24 * _tmp77;
  const Scalar _tmp328 = Scalar(0.83999999999999997) * _tmp85;
  const Scalar _tmp329 = -_tmp13 * _tmp80 - _tmp17 * _tmp95;
  const Scalar _tmp330 = _tmp327 + _tmp328 + _tmp329;
  const Scalar _tmp331 = _tmp152 * _tmp330;
  const Scalar _tmp332 = -_tmp166 * _tmp296 - _tmp173 * _tmp306 + _tmp176 * _tmp296 -
                         _tmp176 * _tmp301 + _tmp299 + _tmp302;
  const Scalar _tmp333 = _tmp177 * _tmp319;
  const Scalar _tmp334 = -Scalar(1.6799999999999999) * _tmp21 * _tmp70;
  const Scalar _tmp335 = -Scalar(1.6799999999999999) * _tmp121 + _tmp334;
  const Scalar _tmp336 = _tmp335 / std::pow(_tmp151, Scalar(2));
  const Scalar _tmp337 = _tmp149 * _tmp336;
  const Scalar _tmp338 = _tmp113 + _tmp114 + _tmp116 + _tmp118 - _tmp120 - _tmp122 -
                         _tmp153 * _tmp326 + _tmp177 * _tmp323 - _tmp177 * _tmp325 -
                         _tmp183 * _tmp333 + _tmp229 * _tmp332 - _tmp231 * _tmp331 +
                         _tmp231 * _tmp337 + _tmp305;
  const Scalar _tmp339 = _tmp179 * _tmp290 + _tmp181 * _tmp309;
  const Scalar _tmp340 = _tmp182 * _tmp209 * _tmp339;
  const Scalar _tmp341 = _tmp212 * _tmp340;
  const Scalar _tmp342 = -_tmp153 * _tmp321 - _tmp166 * _tmp316 - _tmp177 * _tmp318 +
                         _tmp197 * _tmp298 - _tmp197 * _tmp314 - _tmp199 * _tmp278 +
                         _tmp199 * _tmp316 + _tmp201 * _tmp333 - _tmp202 * _tmp332 -
                         _tmp203 * _tmp331 + _tmp203 * _tmp337 + _tmp279;
  const Scalar _tmp343 = _tmp342 / std::pow(_tmp204, Scalar(2));
  const Scalar _tmp344 = _tmp216 * _tmp343;
  const Scalar _tmp345 = _tmp339 / (_tmp207 * std::sqrt(_tmp207));
  const Scalar _tmp346 = _tmp209 * _tmp309;
  const Scalar _tmp347 =
      _tmp213 * (-_tmp140 * _tmp179 * _tmp345 + _tmp142 * _tmp181 * _tmp345 - _tmp142 * _tmp346 +
                 _tmp179 * _tmp346 - _tmp210 * _tmp290 + _tmp211 * _tmp290);
  const Scalar _tmp348 = _tmp208 * _tmp310;
  const Scalar _tmp349 = _tmp212 * _tmp348;
  const Scalar _tmp350 =
      _tmp186 * (-_tmp148 * _tmp300 + _tmp148 * _tmp303 - _tmp150 * _tmp296 + _tmp150 * _tmp301 +
                 _tmp168 * _tmp300 + _tmp173 * _tmp341 + _tmp173 * _tmp347 - _tmp173 * _tmp349 -
                 _tmp184 * _tmp292 - _tmp214 * _tmp296 + _tmp214 * _tmp301);
  const Scalar _tmp351 = _tmp139 * _tmp287 - _tmp139 * _tmp315 + _tmp146 * _tmp278 -
                         _tmp146 * _tmp316 + _tmp192 * _tmp315 + _tmp197 * _tmp341 +
                         _tmp197 * _tmp347 - _tmp197 * _tmp349 - _tmp200 * _tmp277 -
                         _tmp201 * _tmp350 + _tmp214 * _tmp278 - _tmp214 * _tmp316 -
                         _tmp215 * _tmp318 + _tmp215 * _tmp320;
  const Scalar _tmp352 = _tmp205 * _tmp351;
  const Scalar _tmp353 = _tmp220 * (_tmp183 * _tmp350 + _tmp215 * _tmp323 - _tmp215 * _tmp324 -
                                    _tmp215 * _tmp325 - _tmp217 * _tmp338 + _tmp232 * _tmp344 -
                                    _tmp232 * _tmp352 - _tmp341 - _tmp347 + _tmp349);
  const Scalar _tmp354 = _tmp351 / std::pow(_tmp216, Scalar(2));
  const Scalar _tmp355 = _tmp204 * _tmp354;
  const Scalar _tmp356 = _tmp233 * _tmp355;
  const Scalar _tmp357 = _tmp219 * _tmp342;
  const Scalar _tmp358 = _tmp233 * _tmp357;
  const Scalar _tmp359 = _tmp338 + _tmp353 - _tmp356 + _tmp358;
  const Scalar _tmp360 = _tmp203 * _tmp343;
  const Scalar _tmp361 = -_tmp327 - _tmp328 + _tmp329 + _tmp335;
  const Scalar _tmp362 = _tmp153 * _tmp361;
  const Scalar _tmp363 = _tmp147 * _tmp337;
  const Scalar _tmp364 = _tmp147 * _tmp331;
  const Scalar _tmp365 =
      (-Scalar(1.6799999999999999) * _tmp119 + _tmp330 + _tmp334 - _tmp362 + _tmp363 - _tmp364) /
      std::pow(_tmp155, Scalar(2));
  const Scalar _tmp366 = _tmp147 * _tmp365;
  const Scalar _tmp367 = _tmp156 * _tmp361;
  const Scalar _tmp368 = _tmp222 * _tmp353 - _tmp222 * _tmp356 + _tmp222 * _tmp358 -
                         _tmp224 * _tmp359 - _tmp234 * _tmp366 + _tmp234 * _tmp367 -
                         _tmp235 * _tmp322 + _tmp235 * _tmp360 + _tmp326;
  const Scalar _tmp369 = Scalar(1.0) * _tmp365;
  const Scalar _tmp370 = Scalar(1.0) * _tmp336;
  const Scalar _tmp371 = Scalar(6.59232) * _tmp24;
  const Scalar _tmp372 = _tmp125 * fh1;
  const Scalar _tmp373 = _tmp274 + _tmp307;
  const Scalar _tmp374 = _tmp275 + _tmp373;
  const Scalar _tmp375 = Scalar(6.59232) * _tmp70;
  const Scalar _tmp376 = Scalar(6.59232) * _tmp21;
  const Scalar _tmp377 = _tmp281 + _tmp289;
  const Scalar _tmp378 =
      (2 * _tmp126 * _tmp377 + 2 * _tmp127 * _tmp374) / (_tmp128 * std::sqrt(_tmp128));
  const Scalar _tmp379 = (Scalar(1) / Scalar(2)) * _tmp378;
  const Scalar _tmp380 = _tmp127 * _tmp379;
  const Scalar _tmp381 = _tmp129 * _tmp374;
  const Scalar _tmp382 = -_tmp136 * _tmp380 + _tmp136 * _tmp381 + _tmp162 * _tmp372 +
                         _tmp17 * _tmp375 + _tmp371 * _tmp73 + _tmp374 * fv1 + _tmp376 * _tmp64 +
                         Scalar(6.59232) * _tmp99;
  const Scalar _tmp383 = Scalar(0.5) * _tmp378 * fh1;
  const Scalar _tmp384 = _tmp157 * _tmp331;
  const Scalar _tmp385 = _tmp157 * _tmp337;
  const Scalar _tmp386 =
      -_tmp154 * _tmp369 + _tmp157 * _tmp362 - _tmp157 * _tmp363 + _tmp157 * _tmp364;
  const Scalar _tmp387 = _tmp153 * _tmp369;
  const Scalar _tmp388 = _tmp158 * _tmp336;
  const Scalar _tmp389 = _tmp381 * fh1;
  const Scalar _tmp390 = Scalar(1.0) * _tmp319;
  const Scalar _tmp391 = _tmp187 * _tmp312;
  const Scalar _tmp392 = _tmp188 * _tmp390;
  const Scalar _tmp393 = _tmp153 * _tmp391 - _tmp153 * _tmp392 - _tmp187 * _tmp332 +
                         _tmp189 * _tmp331 - _tmp189 * _tmp337 + Scalar(1.0) * _tmp333;
  const Scalar _tmp394 = _tmp220 * (_tmp190 * _tmp344 - _tmp190 * _tmp352 + _tmp215 * _tmp390 -
                                    _tmp217 * _tmp393 - Scalar(1.0) * _tmp350);
  const Scalar _tmp395 = _tmp218 * _tmp357;
  const Scalar _tmp396 = _tmp218 * _tmp355;
  const Scalar _tmp397 = _tmp393 + _tmp394 + _tmp395 - _tmp396;
  const Scalar _tmp398 = -_tmp221 * _tmp366 + _tmp221 * _tmp367 + _tmp222 * _tmp394 +
                         _tmp222 * _tmp395 - _tmp222 * _tmp396 - _tmp223 * _tmp322 +
                         _tmp223 * _tmp360 - _tmp224 * _tmp397 - _tmp391 + _tmp392;
  const Scalar _tmp399 = _tmp157 * _tmp357;
  const Scalar _tmp400 = _tmp157 * _tmp355;
  const Scalar _tmp401 = _tmp220 * _tmp369;
  const Scalar _tmp402 = Scalar(1.0) * _tmp354;
  const Scalar _tmp403 = _tmp147 * _tmp399 - _tmp147 * _tmp400 - _tmp147 * _tmp401 +
                         _tmp203 * _tmp402 - _tmp246 * _tmp321 + _tmp247 * _tmp361;
  const Scalar _tmp404 = _tmp126 * _tmp379;
  const Scalar _tmp405 = _tmp129 * _tmp377;
  const Scalar _tmp406 = fh1 * (_tmp126 * _tmp381 - _tmp127 * _tmp405 + _tmp38 * _tmp380 -
                                _tmp38 * _tmp381 - _tmp404 * _tmp50 + _tmp405 * _tmp50);
  const Scalar _tmp407 =
      _tmp143 * _tmp331 - _tmp143 * _tmp337 + Scalar(1.0) * _tmp153 * _tmp290 + _tmp309;
  const Scalar _tmp408 = _tmp240 * _tmp365;
  const Scalar _tmp409 = _tmp156 * _tmp407;
  const Scalar _tmp410 = _tmp282 + _tmp76;
  const Scalar _tmp411 = _tmp147 * _tmp408 - _tmp147 * _tmp409 - _tmp241 * _tmp322 +
                         _tmp241 * _tmp360 - _tmp242 * _tmp361 + _tmp410;
  const Scalar _tmp412 = _tmp405 * fh1;
  const Scalar _tmp413 = _tmp159 * _tmp366;
  const Scalar _tmp414 = _tmp147 * _tmp336;
  const Scalar _tmp415 = -_tmp13 * _tmp375 - _tmp130 * _tmp372 + _tmp136 * _tmp404 -
                         _tmp136 * _tmp405 + _tmp371 * _tmp64 - _tmp376 * _tmp73 - _tmp377 * fv1 +
                         Scalar(6.59232) * _tmp68;
  const Scalar _tmp416 = std::pow(_tmp270, Scalar(-2));
  const Scalar _tmp417 = _tmp244 * _tmp268;
  const Scalar _tmp418 = _tmp187 * _tmp201 * _tmp354;
  const Scalar _tmp419 = Scalar(0.5) * _tmp219;
  const Scalar _tmp420 = _tmp259 * _tmp317;
  const Scalar _tmp421 = _tmp250 * _tmp261;
  const Scalar _tmp422 = _tmp228 * _tmp264;
  const Scalar _tmp423 = _tmp186 * _tmp263;
  const Scalar _tmp424 = _tmp205 * _tmp223;
  const Scalar _tmp425 = _tmp205 * _tmp317;
  const Scalar _tmp426 = _tmp201 * _tmp343;
  const Scalar _tmp427 = -_tmp223 * _tmp425 + _tmp223 * _tmp426 - _tmp254 * _tmp397;
  const Scalar _tmp428 = _tmp205 * _tmp278;
  const Scalar _tmp429 = _tmp186 * _tmp301;
  const Scalar _tmp430 = _tmp197 * _tmp343;
  const Scalar _tmp431 = _tmp173 * _tmp319;
  const Scalar _tmp432 = _tmp238 * _tmp257;
  const Scalar _tmp433 = _tmp213 * _tmp257;
  const Scalar _tmp434 = _tmp380 * fh1;
  const Scalar _tmp435 = _tmp241 * _tmp343;
  const Scalar _tmp436 = _tmp202 * _tmp266;
  const Scalar _tmp437 = _tmp241 * _tmp254;
  const Scalar _tmp438 = _tmp213 * _tmp264;
  const Scalar _tmp439 = _tmp404 * fh1;
  const Scalar _tmp440 = _tmp186 * _tmp255;
  const Scalar _tmp441 =
      -_tmp235 * _tmp425 + _tmp235 * _tmp426 - _tmp254 * _tmp359 - _tmp291 + _tmp311;
  const Scalar _tmp442 =
      -_tmp213 * _tmp261 * _tmp406 -
      _tmp258 * (-_tmp205 * _tmp235 * _tmp316 + _tmp235 * _tmp428 - _tmp235 * _tmp430 +
                 _tmp253 * _tmp359 + _tmp255 * _tmp429 - _tmp255 * _tmp431 + _tmp256 * _tmp441 -
                 _tmp296 * _tmp440) -
      _tmp262 * (_tmp168 * _tmp202 * _tmp294 * _tmp419 + _tmp173 * _tmp246 * _tmp320 +
                 _tmp173 * _tmp418 - _tmp173 * _tmp420 - _tmp192 * _tmp285 * _tmp419 -
                 _tmp197 * _tmp402 + _tmp246 * _tmp278 - _tmp260 * _tmp301) -
      _tmp265 * (_tmp223 * _tmp428 - _tmp223 * _tmp430 + _tmp253 * _tmp397 + _tmp256 * _tmp427 +
                 _tmp263 * _tmp429 - _tmp263 * _tmp431 - _tmp296 * _tmp423 - _tmp316 * _tmp424) -
      _tmp269 *
          (_tmp173 * _tmp202 * _tmp435 - _tmp197 * _tmp435 + _tmp266 * _tmp278 - _tmp266 * _tmp316 -
           _tmp267 * _tmp318 + _tmp296 * _tmp436 - _tmp301 * _tmp436 + _tmp431 * _tmp437) -
      _tmp340 * _tmp417 - _tmp340 * _tmp421 - _tmp340 * _tmp422 - _tmp340 * _tmp432 +
      _tmp348 * _tmp417 + _tmp348 * _tmp421 + _tmp348 * _tmp422 + _tmp348 * _tmp432 -
      _tmp389 * _tmp433 - _tmp412 * _tmp438 + _tmp433 * _tmp434 + _tmp438 * _tmp439;
  const Scalar _tmp443 = _tmp416 * _tmp442;
  const Scalar _tmp444 =
      (-_tmp252 * _tmp443 +
       _tmp271 *
           (-_tmp126 * _tmp226 * _tmp383 - _tmp127 * _tmp237 * _tmp383 +
            Scalar(1.0) * _tmp137 *
                (-_tmp159 * _tmp386 + _tmp384 - _tmp385 - _tmp387 + Scalar(1.0) * _tmp388) +
            _tmp161 * _tmp415 +
            Scalar(1.0) * _tmp163 * (-_tmp157 * _tmp414 + _tmp164 * _tmp361 + _tmp369 - _tmp413) +
            _tmp165 * _tmp382 + _tmp227 * _tmp412 +
            Scalar(1.0) * _tmp228 *
                (_tmp157 * _tmp394 + _tmp157 * _tmp395 - _tmp157 * _tmp396 - _tmp159 * _tmp398 -
                 _tmp221 * _tmp369 + _tmp225 * _tmp370) +
            Scalar(1.0) * _tmp237 * _tmp389 +
            _tmp239 * (_tmp157 * _tmp353 - _tmp157 * _tmp356 + _tmp157 * _tmp358 -
                       _tmp159 * _tmp368 - _tmp234 * _tmp369 + _tmp236 * _tmp370) +
            _tmp245 *
                (-_tmp157 * _tmp407 - _tmp159 * _tmp411 + _tmp240 * _tmp369 + _tmp243 * _tmp370) +
            Scalar(1.0) * _tmp249 * _tmp406 +
            _tmp251 * (-_tmp159 * _tmp403 + _tmp248 * _tmp370 + _tmp399 - _tmp400 - _tmp401))) /
      std::sqrt(Scalar(std::pow(_tmp252, Scalar(2)) * _tmp416 + 1));
  const Scalar _tmp445 = Scalar(9.6622558468725703) * _tmp270;
  const Scalar _tmp446 = Scalar(2.5202214700000001) - _tmp180;
  const Scalar _tmp447 = -_tmp178 + Scalar(-8.3888750099999996);
  const Scalar _tmp448 =
      std::sqrt(Scalar(std::pow(_tmp446, Scalar(2)) + std::pow(_tmp447, Scalar(2))));
  const Scalar _tmp449 = -_tmp272 * _tmp445 - _tmp448;
  const Scalar _tmp450 = Scalar(9.6622558468725703) * _tmp442;
  const Scalar _tmp451 = Scalar(0.1034955) * _tmp271;
  const Scalar _tmp452 = _tmp449 * _tmp451;
  const Scalar _tmp453 = _tmp244 * _tmp266;
  const Scalar _tmp454 =
      -_tmp202 * _tmp453 + _tmp228 * _tmp423 + _tmp238 * _tmp440 - _tmp250 * _tmp260;
  const Scalar _tmp455 = Scalar(1.0) / (_tmp454);
  const Scalar _tmp456 = _tmp152 * _tmp248;
  const Scalar _tmp457 = _tmp157 * _tmp163;
  const Scalar _tmp458 = _tmp147 * _tmp152;
  const Scalar _tmp459 = _tmp152 * _tmp236;
  const Scalar _tmp460 = _tmp152 * _tmp244;
  const Scalar _tmp461 = _tmp152 * _tmp158;
  const Scalar _tmp462 = _tmp152 * _tmp225;
  const Scalar _tmp463 = _tmp137 * _tmp461 + _tmp228 * _tmp462 + _tmp238 * _tmp459 +
                         _tmp243 * _tmp460 + _tmp250 * _tmp456 - _tmp457 * _tmp458;
  const Scalar _tmp464 = std::asinh(_tmp455 * _tmp463);
  const Scalar _tmp465 = Scalar(1.0) * _tmp464;
  const Scalar _tmp466 = std::pow(_tmp454, Scalar(-2));
  const Scalar _tmp467 = _tmp244 * _tmp435;
  const Scalar _tmp468 = _tmp246 * _tmp250;
  const Scalar _tmp469 = _tmp186 * _tmp228 * _tmp427 + _tmp186 * _tmp238 * _tmp441 +
                         _tmp202 * _tmp467 - _tmp228 * _tmp263 * _tmp319 -
                         _tmp238 * _tmp255 * _tmp319 + _tmp244 * _tmp319 * _tmp437 +
                         _tmp250 * _tmp418 - _tmp250 * _tmp420 - _tmp260 * _tmp406 -
                         _tmp318 * _tmp453 + _tmp320 * _tmp468 + _tmp389 * _tmp440 +
                         _tmp412 * _tmp423 - _tmp423 * _tmp439 - _tmp434 * _tmp440;
  const Scalar _tmp470 = _tmp466 * _tmp469;
  const Scalar _tmp471 = _tmp157 * _tmp382;
  const Scalar _tmp472 =
      (_tmp455 * (_tmp137 * _tmp152 * _tmp386 - _tmp137 * _tmp388 + _tmp152 * _tmp228 * _tmp398 +
                  _tmp152 * _tmp238 * _tmp368 + _tmp152 * _tmp250 * _tmp403 -
                  _tmp152 * _tmp361 * _tmp457 + _tmp163 * _tmp413 - _tmp225 * _tmp228 * _tmp336 -
                  _tmp236 * _tmp238 * _tmp336 - _tmp243 * _tmp244 * _tmp336 -
                  _tmp248 * _tmp250 * _tmp336 + _tmp389 * _tmp459 + _tmp406 * _tmp456 +
                  _tmp411 * _tmp460 + _tmp412 * _tmp462 + _tmp414 * _tmp457 + _tmp415 * _tmp461 -
                  _tmp434 * _tmp459 - _tmp439 * _tmp462 - _tmp458 * _tmp471) -
       _tmp463 * _tmp470) /
      std::sqrt(Scalar(std::pow(_tmp463, Scalar(2)) * _tmp466 + 1));
  const Scalar _tmp473 = Scalar(9.6622558468725703) * _tmp454;
  const Scalar _tmp474 = Scalar(4.7752063900000001) - _tmp169;
  const Scalar _tmp475 = Scalar(2.71799795) - _tmp167;
  const Scalar _tmp476 =
      std::sqrt(Scalar(std::pow(_tmp474, Scalar(2)) + std::pow(_tmp475, Scalar(2))));
  const Scalar _tmp477 = -_tmp464 * _tmp473 - _tmp476;
  const Scalar _tmp478 = Scalar(0.1034955) * _tmp455;
  const Scalar _tmp479 = _tmp477 * _tmp478;
  const Scalar _tmp480 = _tmp288 + _tmp76;
  const Scalar _tmp481 = Scalar(9.6622558468725703) * _tmp469;
  const Scalar _tmp482 = _tmp205 * fh1;
  const Scalar _tmp483 = _tmp235 * _tmp482;
  const Scalar _tmp484 = _tmp130 * _tmp482;
  const Scalar _tmp485 = _tmp162 * _tmp483 + _tmp223 * _tmp484 + _tmp453 + _tmp468;
  const Scalar _tmp486 = Scalar(1.0) / (_tmp485);
  const Scalar _tmp487 = _tmp156 * _tmp238;
  const Scalar _tmp488 = _tmp156 * _tmp228;
  const Scalar _tmp489 = -_tmp137 * _tmp160 - _tmp221 * _tmp488 - _tmp234 * _tmp487 +
                         _tmp242 * _tmp244 - _tmp247 * _tmp250 + _tmp457;
  const Scalar _tmp490 = std::asinh(_tmp486 * _tmp489);
  const Scalar _tmp491 = Scalar(1.0) * _tmp490;
  const Scalar _tmp492 = Scalar(9.6622558468725703) * _tmp485;
  const Scalar _tmp493 = -_tmp191 + Scalar(-1.79662371);
  const Scalar _tmp494 = Scalar(4.8333311099999996) - _tmp193;
  const Scalar _tmp495 =
      std::sqrt(Scalar(std::pow(_tmp493, Scalar(2)) + std::pow(_tmp494, Scalar(2))));
  const Scalar _tmp496 = -_tmp490 * _tmp492 - _tmp495;
  const Scalar _tmp497 = Scalar(0.1034955) * _tmp486;
  const Scalar _tmp498 = _tmp496 * _tmp497;
  const Scalar _tmp499 = _tmp162 * _tmp359 * _tmp482 - _tmp223 * _tmp228 * _tmp343 -
                         _tmp223 * _tmp404 * _tmp482 - _tmp235 * _tmp238 * _tmp343 +
                         _tmp246 * _tmp406 - _tmp251 * _tmp354 - _tmp380 * _tmp483 +
                         _tmp381 * _tmp483 + _tmp397 * _tmp484 + _tmp412 * _tmp424 - _tmp467;
  const Scalar _tmp500 = Scalar(9.6622558468725703) * _tmp499;
  const Scalar _tmp501 = std::pow(_tmp485, Scalar(-2));
  const Scalar _tmp502 = _tmp499 * _tmp501;
  const Scalar _tmp503 = _tmp156 * _tmp221;
  const Scalar _tmp504 = _tmp156 * _tmp234;
  const Scalar _tmp505 =
      (_tmp486 * (-_tmp137 * _tmp384 + _tmp137 * _tmp385 + _tmp137 * _tmp387 - _tmp160 * _tmp415 -
                  _tmp163 * _tmp369 + _tmp221 * _tmp228 * _tmp365 + _tmp234 * _tmp238 * _tmp365 -
                  _tmp244 * _tmp408 + _tmp244 * _tmp409 - _tmp247 * _tmp406 - _tmp250 * _tmp399 +
                  _tmp250 * _tmp400 + _tmp250 * _tmp401 - _tmp353 * _tmp487 + _tmp356 * _tmp487 -
                  _tmp358 * _tmp487 - _tmp389 * _tmp504 - _tmp394 * _tmp488 - _tmp395 * _tmp488 +
                  _tmp396 * _tmp488 - _tmp412 * _tmp503 + _tmp434 * _tmp504 + _tmp439 * _tmp503 +
                  _tmp471) -
       _tmp489 * _tmp502) /
      std::sqrt(Scalar(std::pow(_tmp489, Scalar(2)) * _tmp501 + 1));

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) =
      _tmp125 -
      Scalar(0.5) * (2 * _tmp40 * (_tmp76 + _tmp94) + 2 * _tmp52 * (_tmp106 + _tmp97)) *
          std::sinh(Scalar(0.1034955) * _tmp0 *
                    (-_tmp53 - Scalar(9.6622558468725703) * fh1 * std::asinh(_tmp0 * fv1))) /
          _tmp53;
  _res(1, 0) =
      _tmp298 -
      _tmp445 *
          (-Scalar(0.87653584775870996) * _tmp443 + Scalar(1.0) * _tmp444 * std::sinh(_tmp273) -
           (-Scalar(0.1034955) * _tmp443 * _tmp449 +
            _tmp451 * (-_tmp272 * _tmp450 - _tmp444 * _tmp445 -
                       Scalar(1) / Scalar(2) *
                           (2 * _tmp410 * _tmp447 + 2 * _tmp446 * (_tmp106 + _tmp274)) / _tmp448)) *
               std::sinh(_tmp452)) -
      _tmp450 * (Scalar(0.87653584775870996) * _tmp271 + std::cosh(_tmp273) - std::cosh(_tmp452));
  _res(2, 0) =
      _tmp306 -
      _tmp473 *
          (-Scalar(0.86565325453551001) * _tmp470 + Scalar(1.0) * _tmp472 * std::sinh(_tmp465) -
           (-Scalar(0.1034955) * _tmp470 * _tmp477 +
            _tmp478 * (-_tmp464 * _tmp481 - _tmp472 * _tmp473 -
                       Scalar(1) / Scalar(2) *
                           (2 * _tmp474 * (_tmp281 + _tmp480) + 2 * _tmp475 * (_tmp103 + _tmp373)) /
                           _tmp476)) *
               std::sinh(_tmp479)) -
      _tmp481 * (Scalar(0.86565325453551001) * _tmp455 + std::cosh(_tmp465) - std::cosh(_tmp479));
  _res(3, 0) =
      _tmp314 -
      _tmp492 *
          (-Scalar(0.86625939559540499) * _tmp502 + Scalar(1.0) * _tmp505 * std::sinh(_tmp491) -
           (-Scalar(0.1034955) * _tmp496 * _tmp502 +
            _tmp497 * (-_tmp490 * _tmp500 - _tmp492 * _tmp505 -
                       Scalar(1) / Scalar(2) *
                           (2 * _tmp493 * (_tmp103 + _tmp308) + 2 * _tmp494 * (_tmp480 + _tmp93)) /
                           _tmp495)) *
               std::sinh(_tmp498)) -
      _tmp500 * (Scalar(0.86625939559540499) * _tmp486 + std::cosh(_tmp491) - std::cosh(_tmp498));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym