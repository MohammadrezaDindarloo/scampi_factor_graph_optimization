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
 * Symbolic function: FK_residual_func_cost2_wrt_rx
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
Eigen::Matrix<Scalar, 4, 1> FkResidualFuncCost2WrtRx(
    const Scalar fh1, const Scalar fv1, const Scalar rx, const Scalar ry, const Scalar rz,
    const Scalar tx, const Scalar ty, const Scalar tz, const Scalar rot_init_x,
    const Scalar rot_init_y, const Scalar rot_init_z, const Scalar rot_init_w, const Scalar lc0,
    const Scalar lc1, const Scalar lc2, const Scalar lc3, const Scalar epsilon) {
  // Total ops: 1618

  // Unused inputs
  (void)tz;
  (void)lc0;
  (void)lc1;
  (void)lc2;
  (void)lc3;
  (void)epsilon;

  // Input arrays

  // Intermediate terms (503)
  const Scalar _tmp0 = std::pow(rx, Scalar(2));
  const Scalar _tmp1 = _tmp0 + std::pow(ry, Scalar(2)) + std::pow(rz, Scalar(2));
  const Scalar _tmp2 = std::sqrt(_tmp1);
  const Scalar _tmp3 = (Scalar(1) / Scalar(2)) * _tmp2;
  const Scalar _tmp4 = std::cos(_tmp3);
  const Scalar _tmp5 = _tmp4 * rot_init_z;
  const Scalar _tmp6 = std::sin(_tmp3);
  const Scalar _tmp7 = _tmp6 / _tmp2;
  const Scalar _tmp8 = _tmp7 * rot_init_x;
  const Scalar _tmp9 = _tmp7 * rot_init_y;
  const Scalar _tmp10 = _tmp9 * rx;
  const Scalar _tmp11 = _tmp7 * rot_init_w;
  const Scalar _tmp12 = -_tmp10 + _tmp11 * rz + _tmp5 + _tmp8 * ry;
  const Scalar _tmp13 = _tmp4 * rot_init_x;
  const Scalar _tmp14 = (Scalar(1) / Scalar(2)) / _tmp1;
  const Scalar _tmp15 = _tmp0 * _tmp14;
  const Scalar _tmp16 = _tmp14 * rx;
  const Scalar _tmp17 = _tmp16 * rz;
  const Scalar _tmp18 = _tmp4 * rot_init_y;
  const Scalar _tmp19 = _tmp16 * ry;
  const Scalar _tmp20 = _tmp11 * rx;
  const Scalar _tmp21 = _tmp6 / (_tmp1 * std::sqrt(_tmp1));
  const Scalar _tmp22 = _tmp0 * _tmp21;
  const Scalar _tmp23 = _tmp21 * rx;
  const Scalar _tmp24 = _tmp23 * ry;
  const Scalar _tmp25 = _tmp23 * rz;
  const Scalar _tmp26 = -_tmp13 * _tmp15 - _tmp17 * _tmp5 - _tmp18 * _tmp19 -
                        Scalar(1) / Scalar(2) * _tmp20 + _tmp22 * rot_init_x + _tmp24 * rot_init_y +
                        _tmp25 * rot_init_z - _tmp8;
  const Scalar _tmp27 = Scalar(0.41999999999999998) * _tmp26;
  const Scalar _tmp28 = _tmp12 * _tmp27;
  const Scalar _tmp29 = _tmp4 * rot_init_w;
  const Scalar _tmp30 = _tmp8 * rx;
  const Scalar _tmp31 = _tmp7 * rot_init_z;
  const Scalar _tmp32 = _tmp29 - _tmp30 - _tmp31 * rz - _tmp9 * ry;
  const Scalar _tmp33 = _tmp31 * rx;
  const Scalar _tmp34 = _tmp13 * _tmp19 - _tmp15 * _tmp18 + _tmp17 * _tmp29 + _tmp22 * rot_init_y -
                        _tmp24 * rot_init_x - _tmp25 * rot_init_w - Scalar(1) / Scalar(2) * _tmp33 -
                        _tmp9;
  const Scalar _tmp35 = Scalar(0.41999999999999998) * _tmp34;
  const Scalar _tmp36 = _tmp32 * _tmp35;
  const Scalar _tmp37 = _tmp28 + _tmp36;
  const Scalar _tmp38 = _tmp13 + _tmp20 - _tmp31 * ry + _tmp9 * rz;
  const Scalar _tmp39 = -Scalar(1) / Scalar(2) * _tmp10 - _tmp13 * _tmp17 + _tmp15 * _tmp5 +
                        _tmp19 * _tmp29 - _tmp22 * rot_init_z - _tmp24 * rot_init_w +
                        _tmp25 * rot_init_x + _tmp31;
  const Scalar _tmp40 = Scalar(0.41999999999999998) * _tmp39;
  const Scalar _tmp41 = _tmp38 * _tmp40;
  const Scalar _tmp42 = _tmp11 * ry + _tmp18 + _tmp33 - _tmp8 * rz;
  const Scalar _tmp43 = _tmp11 + _tmp15 * _tmp29 + _tmp17 * _tmp18 - _tmp19 * _tmp5 -
                        _tmp22 * rot_init_w + _tmp24 * rot_init_z - _tmp25 * rot_init_y -
                        Scalar(1) / Scalar(2) * _tmp30;
  const Scalar _tmp44 = Scalar(0.41999999999999998) * _tmp43;
  const Scalar _tmp45 = _tmp42 * _tmp44;
  const Scalar _tmp46 = _tmp41 + _tmp45;
  const Scalar _tmp47 = _tmp37 + _tmp46;
  const Scalar _tmp48 = _tmp12 * _tmp34;
  const Scalar _tmp49 = Scalar(0.83999999999999997) * _tmp48;
  const Scalar _tmp50 = -_tmp49;
  const Scalar _tmp51 = _tmp38 * _tmp43;
  const Scalar _tmp52 = Scalar(0.83999999999999997) * _tmp51;
  const Scalar _tmp53 = _tmp50 - _tmp52;
  const Scalar _tmp54 = _tmp26 * _tmp38;
  const Scalar _tmp55 = Scalar(0.021999999999999999) * _tmp54;
  const Scalar _tmp56 = Scalar(0.021999999999999999) * _tmp42;
  const Scalar _tmp57 = _tmp34 * _tmp56;
  const Scalar _tmp58 = Scalar(0.021999999999999999) * _tmp12;
  const Scalar _tmp59 = _tmp39 * _tmp58;
  const Scalar _tmp60 = Scalar(0.021999999999999999) * _tmp32;
  const Scalar _tmp61 = _tmp43 * _tmp60;
  const Scalar _tmp62 = -_tmp55 + _tmp57 + _tmp59 - _tmp61;
  const Scalar _tmp63 = _tmp53 + _tmp62;
  const Scalar _tmp64 = 2 * _tmp38 * _tmp42;
  const Scalar _tmp65 = 2 * _tmp32;
  const Scalar _tmp66 = _tmp12 * _tmp65;
  const Scalar _tmp67 = Scalar(0.20999999999999999) * _tmp64 + Scalar(0.20999999999999999) * _tmp66;
  const Scalar _tmp68 = -_tmp67;
  const Scalar _tmp69 = -2 * std::pow(_tmp12, Scalar(2));
  const Scalar _tmp70 = 1 - 2 * std::pow(_tmp38, Scalar(2));
  const Scalar _tmp71 = Scalar(0.20999999999999999) * _tmp69 + Scalar(0.20999999999999999) * _tmp70;
  const Scalar _tmp72 = 2 * _tmp12;
  const Scalar _tmp73 = _tmp42 * _tmp72;
  const Scalar _tmp74 = _tmp38 * _tmp65;
  const Scalar _tmp75 = _tmp73 - _tmp74;
  const Scalar _tmp76 = -Scalar(0.010999999999999999) * _tmp75;
  const Scalar _tmp77 = -_tmp71 + _tmp76;
  const Scalar _tmp78 = _tmp68 + _tmp77;
  const Scalar _tmp79 = _tmp78 + ty;
  const Scalar _tmp80 = -_tmp79 + Scalar(-8.3196563700000006);
  const Scalar _tmp81 = -_tmp28 - _tmp36;
  const Scalar _tmp82 = _tmp46 + _tmp81;
  const Scalar _tmp83 = _tmp39 * _tmp42;
  const Scalar _tmp84 = Scalar(0.83999999999999997) * _tmp83;
  const Scalar _tmp85 = _tmp50 - _tmp84;
  const Scalar _tmp86 = _tmp26 * _tmp56;
  const Scalar _tmp87 = _tmp34 * _tmp38;
  const Scalar _tmp88 = Scalar(0.021999999999999999) * _tmp87;
  const Scalar _tmp89 = _tmp43 * _tmp58;
  const Scalar _tmp90 = _tmp39 * _tmp60;
  const Scalar _tmp91 = _tmp86 + _tmp88 + _tmp89 + _tmp90;
  const Scalar _tmp92 = _tmp85 + _tmp91;
  const Scalar _tmp93 = -2 * std::pow(_tmp42, Scalar(2));
  const Scalar _tmp94 = Scalar(0.20999999999999999) * _tmp69 +
                        Scalar(0.20999999999999999) * _tmp93 + Scalar(0.20999999999999999);
  const Scalar _tmp95 = -_tmp94;
  const Scalar _tmp96 = Scalar(0.20999999999999999) * _tmp64 - Scalar(0.20999999999999999) * _tmp66;
  const Scalar _tmp97 = _tmp38 * _tmp72;
  const Scalar _tmp98 = _tmp42 * _tmp65;
  const Scalar _tmp99 = _tmp97 + _tmp98;
  const Scalar _tmp100 = -Scalar(0.010999999999999999) * _tmp99;
  const Scalar _tmp101 = _tmp100 - _tmp96;
  const Scalar _tmp102 = _tmp101 + _tmp95;
  const Scalar _tmp103 = _tmp102 + tx;
  const Scalar _tmp104 = -_tmp103 + Scalar(-1.9874742000000001);
  const Scalar _tmp105 = Scalar(1.0) / (fh1);
  const Scalar _tmp106 =
      std::sqrt(Scalar(std::pow(_tmp104, Scalar(2)) + std::pow(_tmp80, Scalar(2))));
  const Scalar _tmp107 = _tmp100 + _tmp96;
  const Scalar _tmp108 = _tmp107 + _tmp95;
  const Scalar _tmp109 = _tmp101 + _tmp94;
  const Scalar _tmp110 = Scalar(1.0) * _tmp109;
  const Scalar _tmp111 = _tmp67 + _tmp77;
  const Scalar _tmp112 = Scalar(1.0) * _tmp111;
  const Scalar _tmp113 = -_tmp112;
  const Scalar _tmp114 = _tmp71 + _tmp76;
  const Scalar _tmp115 = _tmp114 + _tmp68;
  const Scalar _tmp116 = _tmp113 + _tmp115;
  const Scalar _tmp117 = _tmp114 + _tmp67;
  const Scalar _tmp118 = _tmp113 + _tmp117;
  const Scalar _tmp119 = Scalar(1.0) / (_tmp118);
  const Scalar _tmp120 = _tmp107 + _tmp94;
  const Scalar _tmp121 = _tmp110 - _tmp120;
  const Scalar _tmp122 = _tmp119 * _tmp121;
  const Scalar _tmp123 = _tmp116 * _tmp122;
  const Scalar _tmp124 = -_tmp108 + _tmp110 - _tmp123;
  const Scalar _tmp125 = Scalar(1.0) / (_tmp124);
  const Scalar _tmp126 = Scalar(1.0) * _tmp125;
  const Scalar _tmp127 = _tmp123 * _tmp126 + Scalar(1.0);
  const Scalar _tmp128 = _tmp119 * _tmp127;
  const Scalar _tmp129 = _tmp122 * _tmp126;
  const Scalar _tmp130 = -Scalar(1.0) * _tmp128 + _tmp129;
  const Scalar _tmp131 = _tmp79 + Scalar(8.3196563700000006);
  const Scalar _tmp132 = _tmp103 + Scalar(1.9874742000000001);
  const Scalar _tmp133 = std::pow(_tmp131, Scalar(2)) + std::pow(_tmp132, Scalar(2));
  const Scalar _tmp134 = std::pow(_tmp133, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp135 = _tmp131 * _tmp134;
  const Scalar _tmp136 =
      Scalar(0.20999999999999999) * _tmp73 + Scalar(0.20999999999999999) * _tmp74;
  const Scalar _tmp137 = -_tmp136;
  const Scalar _tmp138 =
      -Scalar(0.010999999999999999) * _tmp70 - Scalar(0.010999999999999999) * _tmp93;
  const Scalar _tmp139 =
      Scalar(0.20999999999999999) * _tmp97 - Scalar(0.20999999999999999) * _tmp98;
  const Scalar _tmp140 = _tmp138 - _tmp139;
  const Scalar _tmp141 = fh1 * (_tmp137 + _tmp140);
  const Scalar _tmp142 = -_tmp135 * _tmp141 - Scalar(3.29616) * _tmp75 - _tmp78 * fv1;
  const Scalar _tmp143 = Scalar(1.0) * _tmp142;
  const Scalar _tmp144 = _tmp132 * _tmp134;
  const Scalar _tmp145 = _tmp102 * fv1 + _tmp141 * _tmp144 + Scalar(3.29616) * _tmp99;
  const Scalar _tmp146 = _tmp119 * _tmp126;
  const Scalar _tmp147 = Scalar(1.0) * _tmp116 * _tmp146 - Scalar(1.0) * _tmp126;
  const Scalar _tmp148 = _tmp111 + ty;
  const Scalar _tmp149 = _tmp148 + Scalar(8.3888750099999996);
  const Scalar _tmp150 = _tmp109 + tx;
  const Scalar _tmp151 = _tmp150 + Scalar(-2.5202214700000001);
  const Scalar _tmp152 = Scalar(1.0) / (_tmp151);
  const Scalar _tmp153 = _tmp149 * _tmp152;
  const Scalar _tmp154 = _tmp120 + tx;
  const Scalar _tmp155 = _tmp154 + Scalar(-2.71799795);
  const Scalar _tmp156 = _tmp117 + ty;
  const Scalar _tmp157 = _tmp156 + Scalar(-4.7752063900000001);
  const Scalar _tmp158 = std::pow(_tmp155, Scalar(2)) + std::pow(_tmp157, Scalar(2));
  const Scalar _tmp159 = std::pow(_tmp158, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp160 = _tmp155 * _tmp159;
  const Scalar _tmp161 = _tmp157 * _tmp159;
  const Scalar _tmp162 = _tmp153 * _tmp160 - _tmp161;
  const Scalar _tmp163 = Scalar(1.0) / (_tmp162);
  const Scalar _tmp164 = _tmp138 + _tmp139;
  const Scalar _tmp165 = _tmp137 + _tmp164;
  const Scalar _tmp166 = _tmp160 * _tmp165;
  const Scalar _tmp167 = _tmp136 + _tmp164;
  const Scalar _tmp168 = -_tmp160 * _tmp167 + _tmp166;
  const Scalar _tmp169 = _tmp163 * _tmp168;
  const Scalar _tmp170 = -_tmp153 * _tmp166 + _tmp161 * _tmp167;
  const Scalar _tmp171 = _tmp163 * _tmp170;
  const Scalar _tmp172 = Scalar(1.0) * _tmp171;
  const Scalar _tmp173 = _tmp122 * _tmp172 - Scalar(1.0) * _tmp169;
  const Scalar _tmp174 = _tmp108 + tx;
  const Scalar _tmp175 = _tmp174 + Scalar(1.79662371);
  const Scalar _tmp176 = _tmp115 + ty;
  const Scalar _tmp177 = _tmp176 + Scalar(-4.8333311099999996);
  const Scalar _tmp178 = std::pow(_tmp175, Scalar(2)) + std::pow(_tmp177, Scalar(2));
  const Scalar _tmp179 = std::pow(_tmp178, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp180 = _tmp175 * _tmp179;
  const Scalar _tmp181 = _tmp165 * _tmp180;
  const Scalar _tmp182 = _tmp136 + _tmp140;
  const Scalar _tmp183 = _tmp177 * _tmp179;
  const Scalar _tmp184 = _tmp153 * _tmp180 - _tmp183;
  const Scalar _tmp185 = -_tmp153 * _tmp181 - _tmp171 * _tmp184 + _tmp182 * _tmp183;
  const Scalar _tmp186 = _tmp163 * _tmp184;
  const Scalar _tmp187 = -_tmp122 * _tmp185 - _tmp168 * _tmp186 - _tmp180 * _tmp182 + _tmp181;
  const Scalar _tmp188 = Scalar(1.0) / (_tmp187);
  const Scalar _tmp189 = std::pow(_tmp151, Scalar(2));
  const Scalar _tmp190 = std::pow(_tmp149, Scalar(2)) + _tmp189;
  const Scalar _tmp191 = std::sqrt(_tmp190);
  const Scalar _tmp192 = Scalar(1.0) / (_tmp191);
  const Scalar _tmp193 = _tmp111 * _tmp192;
  const Scalar _tmp194 = _tmp149 * _tmp192;
  const Scalar _tmp195 = _tmp109 * _tmp194 - _tmp151 * _tmp193;
  const Scalar _tmp196 = _tmp152 * _tmp191;
  const Scalar _tmp197 = _tmp195 * _tmp196;
  const Scalar _tmp198 = _tmp117 * _tmp160 - _tmp120 * _tmp161 + _tmp160 * _tmp197;
  const Scalar _tmp199 =
      -_tmp108 * _tmp183 + _tmp115 * _tmp180 + _tmp180 * _tmp197 - _tmp186 * _tmp198;
  const Scalar _tmp200 = _tmp188 * _tmp199;
  const Scalar _tmp201 = Scalar(1.0) * _tmp163;
  const Scalar _tmp202 = -_tmp173 * _tmp200 - _tmp198 * _tmp201;
  const Scalar _tmp203 = Scalar(1.0) / (_tmp199);
  const Scalar _tmp204 = _tmp187 * _tmp203;
  const Scalar _tmp205 = _tmp202 * _tmp204;
  const Scalar _tmp206 = _tmp116 * _tmp125;
  const Scalar _tmp207 = _tmp173 + _tmp205;
  const Scalar _tmp208 = _tmp185 * _tmp188;
  const Scalar _tmp209 = -_tmp172 + _tmp205 * _tmp206 - _tmp207 * _tmp208;
  const Scalar _tmp210 = Scalar(1.0) * _tmp119;
  const Scalar _tmp211 = _tmp126 * _tmp205 - _tmp209 * _tmp210;
  const Scalar _tmp212 = Scalar(1.0) * fh1;
  const Scalar _tmp213 = _tmp135 * _tmp212;
  const Scalar _tmp214 = _tmp163 * _tmp198;
  const Scalar _tmp215 = _tmp153 * _tmp165;
  const Scalar _tmp216 = _tmp153 * _tmp171 + _tmp215;
  const Scalar _tmp217 = -_tmp122 * _tmp216 + _tmp153 * _tmp169 - _tmp165;
  const Scalar _tmp218 = _tmp153 * _tmp214 - _tmp197 - _tmp200 * _tmp217;
  const Scalar _tmp219 = _tmp204 * _tmp218;
  const Scalar _tmp220 = _tmp217 + _tmp219;
  const Scalar _tmp221 = _tmp206 * _tmp219 - _tmp208 * _tmp220 + _tmp216;
  const Scalar _tmp222 = _tmp126 * _tmp219 - _tmp210 * _tmp221;
  const Scalar _tmp223 = _tmp212 * _tmp222;
  const Scalar _tmp224 = _tmp110 + _tmp112 * _tmp122;
  const Scalar _tmp225 = 0;
  const Scalar _tmp226 = _tmp125 * _tmp224;
  const Scalar _tmp227 = _tmp113 - _tmp116 * _tmp226 - _tmp208 * _tmp225;
  const Scalar _tmp228 = _tmp119 * _tmp227;
  const Scalar _tmp229 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp230 = Scalar(1.0) * _tmp229;
  const Scalar _tmp231 = Scalar(1.0) * _tmp203;
  const Scalar _tmp232 = _tmp126 * _tmp204;
  const Scalar _tmp233 = _tmp116 * _tmp232 - _tmp185 * _tmp231;
  const Scalar _tmp234 = -Scalar(1.0) * _tmp210 * _tmp233 + Scalar(1.0) * _tmp232;
  const Scalar _tmp235 = _tmp134 * _tmp78;
  const Scalar _tmp236 = fh1 * (_tmp102 * _tmp135 - _tmp132 * _tmp235);
  const Scalar _tmp237 =
      _tmp130 * _tmp143 + _tmp144 * _tmp223 + _tmp145 * _tmp147 + _tmp211 * _tmp213 +
      _tmp230 * (-_tmp126 * _tmp224 - Scalar(1.0) * _tmp228 + Scalar(1.0)) + _tmp234 * _tmp236;
  const Scalar _tmp238 = _tmp180 * _tmp188;
  const Scalar _tmp239 = _tmp184 * _tmp188;
  const Scalar _tmp240 = -_tmp153 - _tmp220 * _tmp239;
  const Scalar _tmp241 = _tmp160 * _tmp163;
  const Scalar _tmp242 = _tmp220 * _tmp238 + _tmp240 * _tmp241 + Scalar(1.0);
  const Scalar _tmp243 = _tmp144 * fh1;
  const Scalar _tmp244 = _tmp196 * _tmp243;
  const Scalar _tmp245 = _tmp160 * _tmp186;
  const Scalar _tmp246 = _tmp180 * _tmp231 - _tmp231 * _tmp245;
  const Scalar _tmp247 = _tmp196 * _tmp236;
  const Scalar _tmp248 = -_tmp207 * _tmp239 + Scalar(1.0);
  const Scalar _tmp249 = _tmp207 * _tmp238 + _tmp241 * _tmp248;
  const Scalar _tmp250 = _tmp135 * fh1;
  const Scalar _tmp251 = _tmp196 * _tmp250;
  const Scalar _tmp252 = _tmp188 * _tmp225;
  const Scalar _tmp253 = _tmp225 * _tmp238 - _tmp245 * _tmp252;
  const Scalar _tmp254 = _tmp196 * _tmp229;
  const Scalar _tmp255 =
      -_tmp242 * _tmp244 - _tmp246 * _tmp247 - _tmp249 * _tmp251 - _tmp253 * _tmp254;
  const Scalar _tmp256 = Scalar(1.0) / (_tmp255);
  const Scalar _tmp257 = std::asinh(_tmp237 * _tmp256);
  const Scalar _tmp258 = Scalar(1.0) * _tmp257;
  const Scalar _tmp259 = Scalar(9.6622558468725703) * _tmp255;
  const Scalar _tmp260 = Scalar(2.5202214700000001) - _tmp150;
  const Scalar _tmp261 = -_tmp148 + Scalar(-8.3888750099999996);
  const Scalar _tmp262 =
      std::sqrt(Scalar(std::pow(_tmp260, Scalar(2)) + std::pow(_tmp261, Scalar(2))));
  const Scalar _tmp263 = -_tmp257 * _tmp259 - _tmp262;
  const Scalar _tmp264 = Scalar(0.1034955) * _tmp256;
  const Scalar _tmp265 = _tmp263 * _tmp264;
  const Scalar _tmp266 = _tmp55 - _tmp57 - _tmp59 + _tmp61;
  const Scalar _tmp267 = _tmp49 + _tmp52;
  const Scalar _tmp268 = _tmp267 + _tmp47;
  const Scalar _tmp269 = _tmp266 + _tmp268;
  const Scalar _tmp270 = _tmp152 * _tmp269;
  const Scalar _tmp271 = _tmp266 + _tmp53;
  const Scalar _tmp272 = _tmp271 + _tmp47;
  const Scalar _tmp273 = _tmp159 * _tmp272;
  const Scalar _tmp274 = -_tmp86 - _tmp88 - _tmp89 - _tmp90;
  const Scalar _tmp275 = _tmp274 + _tmp85;
  const Scalar _tmp276 = _tmp275 + _tmp82;
  const Scalar _tmp277 =
      (2 * _tmp155 * _tmp276 + 2 * _tmp157 * _tmp272) / (_tmp158 * std::sqrt(_tmp158));
  const Scalar _tmp278 = (Scalar(1) / Scalar(2)) * _tmp277;
  const Scalar _tmp279 = _tmp157 * _tmp278;
  const Scalar _tmp280 = -_tmp41 - _tmp45;
  const Scalar _tmp281 = _tmp280 + _tmp37;
  const Scalar _tmp282 = _tmp275 + _tmp281;
  const Scalar _tmp283 = _tmp282 / _tmp189;
  const Scalar _tmp284 = _tmp149 * _tmp283;
  const Scalar _tmp285 = _tmp155 * _tmp278;
  const Scalar _tmp286 = _tmp159 * _tmp276;
  const Scalar _tmp287 = (-_tmp153 * _tmp285 + _tmp153 * _tmp286 + _tmp160 * _tmp270 -
                          _tmp160 * _tmp284 - _tmp273 + _tmp279) /
                         std::pow(_tmp162, Scalar(2));
  const Scalar _tmp288 = _tmp160 * _tmp287;
  const Scalar _tmp289 = _tmp163 * _tmp286;
  const Scalar _tmp290 = _tmp184 * _tmp287;
  const Scalar _tmp291 = _tmp27 * _tmp38;
  const Scalar _tmp292 = _tmp35 * _tmp42;
  const Scalar _tmp293 = _tmp12 * _tmp40;
  const Scalar _tmp294 = _tmp32 * _tmp44;
  const Scalar _tmp295 = -_tmp291 - _tmp292 - _tmp293 - _tmp294;
  const Scalar _tmp296 = _tmp27 * _tmp42;
  const Scalar _tmp297 = Scalar(0.41999999999999998) * _tmp87;
  const Scalar _tmp298 = _tmp12 * _tmp44;
  const Scalar _tmp299 = _tmp32 * _tmp40;
  const Scalar _tmp300 = Scalar(0.043999999999999997) * _tmp83;
  const Scalar _tmp301 = Scalar(0.043999999999999997) * _tmp51;
  const Scalar _tmp302 = _tmp300 + _tmp301;
  const Scalar _tmp303 = -_tmp296 + _tmp297 + _tmp298 - _tmp299 + _tmp302;
  const Scalar _tmp304 = _tmp295 + _tmp303;
  const Scalar _tmp305 = -_tmp297;
  const Scalar _tmp306 = -_tmp298;
  const Scalar _tmp307 = _tmp296 + _tmp299 + _tmp302 + _tmp305 + _tmp306;
  const Scalar _tmp308 = _tmp291 + _tmp292 + _tmp293 + _tmp294;
  const Scalar _tmp309 = _tmp307 + _tmp308;
  const Scalar _tmp310 = _tmp280 + _tmp81;
  const Scalar _tmp311 = _tmp271 + _tmp310;
  const Scalar _tmp312 = _tmp49 + _tmp84;
  const Scalar _tmp313 = _tmp274 + _tmp312;
  const Scalar _tmp314 = _tmp313 + _tmp82;
  const Scalar _tmp315 =
      (2 * _tmp175 * _tmp314 + 2 * _tmp177 * _tmp311) / (_tmp178 * std::sqrt(_tmp178));
  const Scalar _tmp316 = (Scalar(1) / Scalar(2)) * _tmp315;
  const Scalar _tmp317 = _tmp177 * _tmp316;
  const Scalar _tmp318 = _tmp179 * _tmp314;
  const Scalar _tmp319 = _tmp165 * _tmp318;
  const Scalar _tmp320 = _tmp153 * _tmp304;
  const Scalar _tmp321 = _tmp175 * _tmp316;
  const Scalar _tmp322 = _tmp179 * _tmp311;
  const Scalar _tmp323 = _tmp153 * _tmp318 - _tmp153 * _tmp321 + _tmp180 * _tmp270 -
                         _tmp180 * _tmp284 + _tmp317 - _tmp322;
  const Scalar _tmp324 = _tmp170 * _tmp287;
  const Scalar _tmp325 = _tmp303 + _tmp308;
  const Scalar _tmp326 = _tmp165 * _tmp286;
  const Scalar _tmp327 = -_tmp153 * _tmp326 - _tmp160 * _tmp320 + _tmp161 * _tmp325 -
                         _tmp166 * _tmp270 + _tmp166 * _tmp284 + _tmp167 * _tmp273 -
                         _tmp167 * _tmp279 + _tmp215 * _tmp285;
  const Scalar _tmp328 = -_tmp153 * _tmp319 - _tmp171 * _tmp323 - _tmp180 * _tmp320 -
                         _tmp181 * _tmp270 + _tmp181 * _tmp284 - _tmp182 * _tmp317 +
                         _tmp182 * _tmp322 + _tmp183 * _tmp309 + _tmp184 * _tmp324 -
                         _tmp186 * _tmp327 + _tmp215 * _tmp321;
  const Scalar _tmp329 = _tmp160 * _tmp304 - _tmp160 * _tmp325 - _tmp165 * _tmp285 +
                         _tmp167 * _tmp285 - _tmp167 * _tmp286 + _tmp326;
  const Scalar _tmp330 = -Scalar(1.6799999999999999) * _tmp48;
  const Scalar _tmp331 = _tmp330 - Scalar(1.6799999999999999) * _tmp51;
  const Scalar _tmp332 = _tmp331 / std::pow(_tmp118, Scalar(2));
  const Scalar _tmp333 = _tmp121 * _tmp332;
  const Scalar _tmp334 = Scalar(0.83999999999999997) * _tmp12 * _tmp26;
  const Scalar _tmp335 = Scalar(0.83999999999999997) * _tmp32 * _tmp34;
  const Scalar _tmp336 = -Scalar(0.83999999999999997) * _tmp38 * _tmp39 -
                         Scalar(0.83999999999999997) * _tmp42 * _tmp43;
  const Scalar _tmp337 = _tmp334 + _tmp335 + _tmp336;
  const Scalar _tmp338 = _tmp119 * _tmp337;
  const Scalar _tmp339 = -_tmp122 * _tmp328 - _tmp165 * _tmp321 + _tmp168 * _tmp290 -
                         _tmp169 * _tmp323 + _tmp180 * _tmp304 - _tmp180 * _tmp309 -
                         _tmp182 * _tmp318 + _tmp182 * _tmp321 + _tmp185 * _tmp333 -
                         _tmp185 * _tmp338 - _tmp186 * _tmp329 + _tmp319;
  const Scalar _tmp340 = _tmp339 / std::pow(_tmp187, Scalar(2));
  const Scalar _tmp341 = _tmp220 * _tmp340;
  const Scalar _tmp342 = _tmp163 * _tmp240;
  const Scalar _tmp343 = _tmp203 * _tmp339;
  const Scalar _tmp344 = _tmp218 * _tmp343;
  const Scalar _tmp345 = _tmp149 * _tmp269 + _tmp151 * _tmp282;
  const Scalar _tmp346 = _tmp152 * _tmp192 * _tmp345;
  const Scalar _tmp347 = _tmp195 * _tmp346;
  const Scalar _tmp348 = _tmp345 / (_tmp190 * std::sqrt(_tmp190));
  const Scalar _tmp349 = _tmp192 * _tmp269;
  const Scalar _tmp350 =
      _tmp196 * (-_tmp109 * _tmp149 * _tmp348 + _tmp109 * _tmp349 + _tmp111 * _tmp151 * _tmp348 -
                 _tmp151 * _tmp349 - _tmp193 * _tmp282 + _tmp194 * _tmp282);
  const Scalar _tmp351 = _tmp191 * _tmp283;
  const Scalar _tmp352 = _tmp195 * _tmp351;
  const Scalar _tmp353 = -_tmp117 * _tmp285 + _tmp117 * _tmp286 - _tmp120 * _tmp273 +
                         _tmp120 * _tmp279 + _tmp155 * _tmp273 + _tmp160 * _tmp347 +
                         _tmp160 * _tmp350 - _tmp160 * _tmp352 - _tmp161 * _tmp276 -
                         _tmp197 * _tmp285 + _tmp197 * _tmp286;
  const Scalar _tmp354 = _tmp108 * _tmp317 - _tmp108 * _tmp322 + _tmp115 * _tmp318 -
                         _tmp115 * _tmp321 + _tmp175 * _tmp322 + _tmp180 * _tmp347 +
                         _tmp180 * _tmp350 - _tmp180 * _tmp352 - _tmp183 * _tmp314 -
                         _tmp186 * _tmp353 + _tmp197 * _tmp318 - _tmp197 * _tmp321 +
                         _tmp198 * _tmp290 - _tmp214 * _tmp323;
  const Scalar _tmp355 = _tmp354 / std::pow(_tmp199, Scalar(2));
  const Scalar _tmp356 = _tmp187 * _tmp355;
  const Scalar _tmp357 = _tmp218 * _tmp356;
  const Scalar _tmp358 = _tmp153 * _tmp163;
  const Scalar _tmp359 = -_tmp153 * _tmp324 + _tmp165 * _tmp270 - _tmp165 * _tmp284 +
                         _tmp171 * _tmp270 - _tmp171 * _tmp284 + _tmp320 + _tmp327 * _tmp358;
  const Scalar _tmp360 = _tmp153 * _tmp287;
  const Scalar _tmp361 = -_tmp122 * _tmp359 - _tmp168 * _tmp360 + _tmp169 * _tmp270 -
                         _tmp169 * _tmp284 + _tmp216 * _tmp333 - _tmp216 * _tmp338 + _tmp296 +
                         _tmp299 - _tmp300 - _tmp301 + _tmp305 + _tmp306 + _tmp308 +
                         _tmp329 * _tmp358;
  const Scalar _tmp362 = _tmp199 * _tmp340;
  const Scalar _tmp363 = _tmp188 * _tmp354;
  const Scalar _tmp364 = _tmp204 * (-_tmp198 * _tmp360 - _tmp200 * _tmp361 + _tmp214 * _tmp270 -
                                    _tmp214 * _tmp284 + _tmp217 * _tmp362 - _tmp217 * _tmp363 -
                                    _tmp347 - _tmp350 + _tmp352 + _tmp353 * _tmp358);
  const Scalar _tmp365 = _tmp344 - _tmp357 + _tmp361 + _tmp364;
  const Scalar _tmp366 = _tmp188 * _tmp323;
  const Scalar _tmp367 =
      _tmp184 * _tmp341 - _tmp220 * _tmp366 - _tmp239 * _tmp365 - _tmp270 + _tmp284;
  const Scalar _tmp368 = _tmp188 * _tmp318;
  const Scalar _tmp369 = _tmp188 * _tmp220;
  const Scalar _tmp370 = _tmp236 * _tmp246;
  const Scalar _tmp371 = _tmp196 * _tmp242;
  const Scalar _tmp372 = _tmp267 + _tmp310;
  const Scalar _tmp373 = _tmp266 + _tmp372;
  const Scalar _tmp374 = _tmp281 + _tmp313;
  const Scalar _tmp375 =
      (2 * _tmp131 * _tmp373 + 2 * _tmp132 * _tmp374) / (_tmp133 * std::sqrt(_tmp133));
  const Scalar _tmp376 = (Scalar(1) / Scalar(2)) * _tmp375;
  const Scalar _tmp377 = _tmp132 * _tmp376;
  const Scalar _tmp378 = _tmp377 * fh1;
  const Scalar _tmp379 = Scalar(0.5) * _tmp203;
  const Scalar _tmp380 = _tmp186 * _tmp286;
  const Scalar _tmp381 = Scalar(1.0) * _tmp355;
  const Scalar _tmp382 = _tmp201 * _tmp203 * _tmp323;
  const Scalar _tmp383 = _tmp202 * _tmp356;
  const Scalar _tmp384 = _tmp202 * _tmp343;
  const Scalar _tmp385 = Scalar(1.0) * _tmp287;
  const Scalar _tmp386 = _tmp201 * _tmp327;
  const Scalar _tmp387 = Scalar(1.0) * _tmp324;
  const Scalar _tmp388 = _tmp122 * _tmp386 - _tmp122 * _tmp387 + _tmp168 * _tmp385 -
                         _tmp172 * _tmp333 + _tmp172 * _tmp338 - _tmp201 * _tmp329;
  const Scalar _tmp389 = _tmp204 * (_tmp173 * _tmp362 - _tmp173 * _tmp363 + _tmp198 * _tmp385 -
                                    _tmp200 * _tmp388 - _tmp201 * _tmp353);
  const Scalar _tmp390 = -_tmp383 + _tmp384 + _tmp388 + _tmp389;
  const Scalar _tmp391 = _tmp207 * _tmp340;
  const Scalar _tmp392 = _tmp184 * _tmp391 - _tmp207 * _tmp366 - _tmp239 * _tmp390;
  const Scalar _tmp393 = _tmp188 * _tmp207;
  const Scalar _tmp394 = _tmp163 * _tmp248;
  const Scalar _tmp395 = _tmp225 * _tmp340;
  const Scalar _tmp396 = _tmp229 * _tmp253;
  const Scalar _tmp397 = _tmp134 * _tmp373;
  const Scalar _tmp398 = _tmp131 * _tmp376;
  const Scalar _tmp399 = fh1 * (_tmp102 * _tmp397 - _tmp102 * _tmp398 - _tmp132 * _tmp397 +
                                _tmp135 * _tmp374 - _tmp235 * _tmp374 + _tmp377 * _tmp78);
  const Scalar _tmp400 = _tmp242 * _tmp243;
  const Scalar _tmp401 = _tmp249 * _tmp250;
  const Scalar _tmp402 = _tmp397 * fh1;
  const Scalar _tmp403 = _tmp196 * _tmp249;
  const Scalar _tmp404 = _tmp134 * _tmp374;
  const Scalar _tmp405 = _tmp404 * fh1;
  const Scalar _tmp406 = _tmp398 * fh1;
  const Scalar _tmp407 =
      -_tmp196 * _tmp246 * _tmp399 -
      _tmp244 * (-_tmp180 * _tmp341 + _tmp220 * _tmp368 + _tmp238 * _tmp365 - _tmp240 * _tmp288 +
                 _tmp240 * _tmp289 + _tmp241 * _tmp367 - _tmp285 * _tmp342 - _tmp321 * _tmp369) -
      _tmp247 * (_tmp155 * _tmp186 * _tmp277 * _tmp379 + _tmp160 * _tmp231 * _tmp290 -
                 _tmp160 * _tmp382 - _tmp175 * _tmp315 * _tmp379 - _tmp180 * _tmp381 +
                 _tmp231 * _tmp318 - _tmp231 * _tmp380 + _tmp245 * _tmp381) -
      _tmp251 * (-_tmp180 * _tmp391 + _tmp207 * _tmp368 + _tmp238 * _tmp390 + _tmp241 * _tmp392 -
                 _tmp248 * _tmp288 + _tmp248 * _tmp289 - _tmp285 * _tmp394 - _tmp321 * _tmp393) -
      _tmp254 * (-_tmp180 * _tmp395 + _tmp186 * _tmp252 * _tmp285 + _tmp225 * _tmp239 * _tmp288 -
                 _tmp225 * _tmp241 * _tmp366 + _tmp225 * _tmp368 + _tmp245 * _tmp395 -
                 _tmp252 * _tmp321 - _tmp252 * _tmp380) -
      _tmp346 * _tmp370 - _tmp346 * _tmp396 - _tmp346 * _tmp400 - _tmp346 * _tmp401 +
      _tmp351 * _tmp370 + _tmp351 * _tmp396 + _tmp351 * _tmp400 + _tmp351 * _tmp401 +
      _tmp371 * _tmp378 - _tmp371 * _tmp405 - _tmp402 * _tmp403 + _tmp403 * _tmp406;
  const Scalar _tmp408 = Scalar(9.6622558468725703) * _tmp407;
  const Scalar _tmp409 = Scalar(1.0) * _tmp332;
  const Scalar _tmp410 = _tmp126 * _tmp343;
  const Scalar _tmp411 = _tmp126 * _tmp356;
  const Scalar _tmp412 = _tmp116 * _tmp338;
  const Scalar _tmp413 = _tmp331 - _tmp334 - _tmp335 + _tmp336;
  const Scalar _tmp414 = _tmp122 * _tmp413;
  const Scalar _tmp415 = _tmp116 * _tmp333;
  const Scalar _tmp416 =
      (_tmp330 + _tmp337 - _tmp412 - _tmp414 + _tmp415 - Scalar(1.6799999999999999) * _tmp83) /
      std::pow(_tmp124, Scalar(2));
  const Scalar _tmp417 = Scalar(1.0) * _tmp416;
  const Scalar _tmp418 = _tmp204 * _tmp417;
  const Scalar _tmp419 = _tmp116 * _tmp410 - _tmp116 * _tmp411 - _tmp116 * _tmp418 +
                         _tmp185 * _tmp381 - _tmp231 * _tmp328 + _tmp232 * _tmp413;
  const Scalar _tmp420 = _tmp188 * _tmp328;
  const Scalar _tmp421 = _tmp125 * _tmp413;
  const Scalar _tmp422 = _tmp116 * _tmp416;
  const Scalar _tmp423 = _tmp185 * _tmp391 + _tmp205 * _tmp421 - _tmp205 * _tmp422 -
                         _tmp206 * _tmp383 + _tmp206 * _tmp384 + _tmp206 * _tmp389 -
                         _tmp207 * _tmp420 - _tmp208 * _tmp390 - _tmp386 + _tmp387;
  const Scalar _tmp424 = _tmp126 * _tmp338;
  const Scalar _tmp425 =
      _tmp119 * (-_tmp123 * _tmp417 + _tmp126 * _tmp412 + _tmp126 * _tmp414 - _tmp126 * _tmp415);
  const Scalar _tmp426 = _tmp122 * _tmp417;
  const Scalar _tmp427 = _tmp127 * _tmp332;
  const Scalar _tmp428 = _tmp126 * _tmp333;
  const Scalar _tmp429 = Scalar(6.59232) * _tmp42;
  const Scalar _tmp430 = Scalar(6.59232) * _tmp12;
  const Scalar _tmp431 = Scalar(6.59232) * _tmp32;
  const Scalar _tmp432 = fh1 * (_tmp295 + _tmp307);
  const Scalar _tmp433 = -_tmp141 * _tmp377 + _tmp141 * _tmp404 + _tmp144 * _tmp432 +
                         _tmp26 * _tmp429 + _tmp374 * fv1 + _tmp39 * _tmp431 + _tmp43 * _tmp430 +
                         Scalar(6.59232) * _tmp87;
  const Scalar _tmp434 =
      -_tmp112 * _tmp333 + _tmp112 * _tmp338 + Scalar(1.0) * _tmp122 * _tmp269 + _tmp282;
  const Scalar _tmp435 = _tmp227 * _tmp332;
  const Scalar _tmp436 = _tmp224 * _tmp416;
  const Scalar _tmp437 = _tmp125 * _tmp434;
  const Scalar _tmp438 = _tmp310 + _tmp63;
  const Scalar _tmp439 = _tmp116 * _tmp436 - _tmp116 * _tmp437 + _tmp185 * _tmp395 -
                         _tmp225 * _tmp420 - _tmp226 * _tmp413 + _tmp438;
  const Scalar _tmp440 = Scalar(0.5) * _tmp375 * fh1;
  const Scalar _tmp441 = _tmp116 * _tmp332;
  const Scalar _tmp442 = _tmp210 * _tmp422;
  const Scalar _tmp443 = _tmp185 * _tmp341 + _tmp206 * _tmp344 - _tmp206 * _tmp357 +
                         _tmp206 * _tmp364 - _tmp208 * _tmp365 + _tmp219 * _tmp421 -
                         _tmp219 * _tmp422 - _tmp220 * _tmp420 + _tmp359;
  const Scalar _tmp444 = -_tmp135 * _tmp432 - _tmp141 * _tmp397 + _tmp141 * _tmp398 -
                         _tmp34 * _tmp429 - _tmp373 * fv1 - _tmp39 * _tmp430 + _tmp43 * _tmp431 +
                         Scalar(6.59232) * _tmp54;
  const Scalar _tmp445 = std::pow(_tmp255, Scalar(-2));
  const Scalar _tmp446 = _tmp407 * _tmp445;
  const Scalar _tmp447 =
      (-_tmp237 * _tmp446 +
       _tmp256 *
           (Scalar(1.0) * _tmp130 * _tmp444 - _tmp131 * _tmp211 * _tmp440 -
            _tmp132 * _tmp222 * _tmp440 +
            _tmp143 *
                (_tmp424 - Scalar(1.0) * _tmp425 - _tmp426 + Scalar(1.0) * _tmp427 - _tmp428) +
            _tmp144 * _tmp212 *
                (_tmp126 * _tmp344 - _tmp126 * _tmp357 + _tmp126 * _tmp364 - _tmp210 * _tmp443 -
                 _tmp219 * _tmp417 + _tmp221 * _tmp409) +
            Scalar(1.0) * _tmp145 * (-_tmp126 * _tmp441 + _tmp146 * _tmp413 + _tmp417 - _tmp442) +
            _tmp147 * _tmp433 + _tmp211 * _tmp212 * _tmp397 +
            _tmp213 * (-_tmp126 * _tmp383 + _tmp126 * _tmp384 + _tmp126 * _tmp389 -
                       _tmp205 * _tmp417 + _tmp209 * _tmp409 - _tmp210 * _tmp423) +
            _tmp223 * _tmp404 +
            _tmp230 * (-_tmp126 * _tmp434 - _tmp210 * _tmp439 + _tmp224 * _tmp417 +
                       Scalar(1.0) * _tmp435) +
            _tmp234 * _tmp399 +
            Scalar(1.0) * _tmp236 *
                (-_tmp210 * _tmp419 + _tmp233 * _tmp409 + _tmp410 - _tmp411 - _tmp418))) /
      std::sqrt(Scalar(std::pow(_tmp237, Scalar(2)) * _tmp445 + 1));
  const Scalar _tmp448 = _tmp312 + _tmp91;
  const Scalar _tmp449 = _tmp163 * _tmp243;
  const Scalar _tmp450 = _tmp163 * _tmp250;
  const Scalar _tmp451 = _tmp225 * _tmp229;
  const Scalar _tmp452 = _tmp188 * _tmp451;
  const Scalar _tmp453 = _tmp231 * _tmp236;
  const Scalar _tmp454 =
      -_tmp186 * _tmp452 - _tmp186 * _tmp453 + _tmp240 * _tmp449 + _tmp248 * _tmp450;
  const Scalar _tmp455 = Scalar(1.0) / (_tmp454);
  const Scalar _tmp456 = _tmp119 * _tmp233;
  const Scalar _tmp457 = _tmp126 * _tmp145;
  const Scalar _tmp458 = _tmp116 * _tmp119;
  const Scalar _tmp459 = _tmp119 * _tmp243;
  const Scalar _tmp460 = _tmp119 * _tmp209;
  const Scalar _tmp461 = _tmp128 * _tmp142 + _tmp221 * _tmp459 + _tmp228 * _tmp229 +
                         _tmp236 * _tmp456 + _tmp250 * _tmp460 - _tmp457 * _tmp458;
  const Scalar _tmp462 = std::asinh(_tmp455 * _tmp461);
  const Scalar _tmp463 = Scalar(9.6622558468725703) * _tmp454;
  const Scalar _tmp464 = Scalar(4.7752063900000001) - _tmp156;
  const Scalar _tmp465 = Scalar(2.71799795) - _tmp154;
  const Scalar _tmp466 =
      std::sqrt(Scalar(std::pow(_tmp464, Scalar(2)) + std::pow(_tmp465, Scalar(2))));
  const Scalar _tmp467 = -_tmp462 * _tmp463 - _tmp466;
  const Scalar _tmp468 = Scalar(0.1034955) * _tmp455;
  const Scalar _tmp469 = _tmp467 * _tmp468;
  const Scalar _tmp470 = Scalar(1.0) * _tmp462;
  const Scalar _tmp471 = _tmp236 * _tmp381;
  const Scalar _tmp472 = _tmp340 * _tmp451;
  const Scalar _tmp473 = _tmp231 * _tmp399;
  const Scalar _tmp474 =
      -_tmp163 * _tmp366 * _tmp451 + _tmp186 * _tmp471 + _tmp186 * _tmp472 - _tmp186 * _tmp473 -
      _tmp236 * _tmp382 + _tmp239 * _tmp287 * _tmp451 - _tmp240 * _tmp243 * _tmp287 -
      _tmp248 * _tmp250 * _tmp287 + _tmp290 * _tmp453 - _tmp342 * _tmp378 + _tmp342 * _tmp405 +
      _tmp367 * _tmp449 + _tmp392 * _tmp450 + _tmp394 * _tmp402 - _tmp394 * _tmp406;
  const Scalar _tmp475 = Scalar(9.6622558468725703) * _tmp474;
  const Scalar _tmp476 = std::pow(_tmp454, Scalar(-2));
  const Scalar _tmp477 = _tmp119 * _tmp221;
  const Scalar _tmp478 = _tmp126 * _tmp433;
  const Scalar _tmp479 = _tmp474 * _tmp476;
  const Scalar _tmp480 =
      (_tmp455 * (_tmp119 * _tmp229 * _tmp439 + _tmp119 * _tmp236 * _tmp419 +
                  _tmp119 * _tmp250 * _tmp423 - _tmp119 * _tmp413 * _tmp457 + _tmp128 * _tmp444 +
                  _tmp142 * _tmp425 - _tmp142 * _tmp427 + _tmp145 * _tmp442 -
                  _tmp209 * _tmp250 * _tmp332 - _tmp221 * _tmp243 * _tmp332 - _tmp229 * _tmp435 -
                  _tmp233 * _tmp236 * _tmp332 - _tmp378 * _tmp477 + _tmp399 * _tmp456 +
                  _tmp402 * _tmp460 + _tmp405 * _tmp477 - _tmp406 * _tmp460 + _tmp441 * _tmp457 +
                  _tmp443 * _tmp459 - _tmp458 * _tmp478) -
       _tmp461 * _tmp479) /
      std::sqrt(Scalar(std::pow(_tmp461, Scalar(2)) * _tmp476 + 1));
  const Scalar _tmp481 = _tmp188 * _tmp250;
  const Scalar _tmp482 = _tmp207 * _tmp481 + _tmp243 * _tmp369 + _tmp452 + _tmp453;
  const Scalar _tmp483 = Scalar(1.0) / (_tmp482);
  const Scalar _tmp484 = _tmp125 * _tmp219;
  const Scalar _tmp485 = _tmp125 * _tmp205;
  const Scalar _tmp486 = -_tmp129 * _tmp142 + _tmp226 * _tmp229 - _tmp232 * _tmp236 -
                         _tmp243 * _tmp484 - _tmp250 * _tmp485 + _tmp457;
  const Scalar _tmp487 = std::asinh(_tmp483 * _tmp486);
  const Scalar _tmp488 = Scalar(9.6622558468725703) * _tmp482;
  const Scalar _tmp489 = -_tmp174 + Scalar(-1.79662371);
  const Scalar _tmp490 = Scalar(4.8333311099999996) - _tmp176;
  const Scalar _tmp491 =
      std::sqrt(Scalar(std::pow(_tmp489, Scalar(2)) + std::pow(_tmp490, Scalar(2))));
  const Scalar _tmp492 = -_tmp487 * _tmp488 - _tmp491;
  const Scalar _tmp493 = Scalar(0.1034955) * _tmp483;
  const Scalar _tmp494 = _tmp492 * _tmp493;
  const Scalar _tmp495 = Scalar(1.0) * _tmp487;
  const Scalar _tmp496 = _tmp188 * _tmp243 * _tmp365 - _tmp243 * _tmp341 - _tmp250 * _tmp391 -
                         _tmp369 * _tmp378 + _tmp369 * _tmp405 + _tmp390 * _tmp481 +
                         _tmp393 * _tmp402 - _tmp393 * _tmp406 - _tmp471 - _tmp472 + _tmp473;
  const Scalar _tmp497 = Scalar(9.6622558468725703) * _tmp496;
  const Scalar _tmp498 = std::pow(_tmp482, Scalar(-2));
  const Scalar _tmp499 = _tmp496 * _tmp498;
  const Scalar _tmp500 = _tmp125 * _tmp243;
  const Scalar _tmp501 = _tmp125 * _tmp250;
  const Scalar _tmp502 =
      (_tmp483 * (-_tmp129 * _tmp444 - _tmp142 * _tmp424 + _tmp142 * _tmp426 + _tmp142 * _tmp428 -
                  _tmp145 * _tmp417 + _tmp205 * _tmp250 * _tmp416 + _tmp219 * _tmp243 * _tmp416 -
                  _tmp229 * _tmp436 + _tmp229 * _tmp437 - _tmp232 * _tmp399 - _tmp236 * _tmp410 +
                  _tmp236 * _tmp411 + _tmp236 * _tmp418 - _tmp344 * _tmp500 + _tmp357 * _tmp500 -
                  _tmp364 * _tmp500 + _tmp378 * _tmp484 + _tmp383 * _tmp501 - _tmp384 * _tmp501 -
                  _tmp389 * _tmp501 - _tmp402 * _tmp485 - _tmp405 * _tmp484 + _tmp406 * _tmp485 +
                  _tmp478) -
       _tmp486 * _tmp499) /
      std::sqrt(Scalar(std::pow(_tmp486, Scalar(2)) * _tmp498 + 1));

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) =
      Scalar(0.5) * (2 * _tmp104 * (_tmp82 + _tmp92) + 2 * _tmp80 * (_tmp47 + _tmp63)) *
      std::cosh(Scalar(0.1034955) * _tmp105 *
                (-_tmp106 - Scalar(9.6622558468725703) * fh1 * std::asinh(_tmp105 * fv1))) /
      _tmp106;
  _res(1, 0) =
      _tmp259 *
          (-Scalar(1.0) * _tmp447 * std::cosh(_tmp258) -
           (-Scalar(0.1034955) * _tmp263 * _tmp446 +
            _tmp264 * (-_tmp257 * _tmp408 - _tmp259 * _tmp447 -
                       Scalar(1) / Scalar(2) *
                           (2 * _tmp260 * (_tmp448 + _tmp82) + 2 * _tmp261 * _tmp438) / _tmp262)) *
               std::cosh(_tmp265)) +
      _tmp408 * (-std::sinh(_tmp258) - std::sinh(_tmp265));
  _res(2, 0) =
      _tmp463 *
          (-Scalar(1.0) * _tmp480 * std::cosh(_tmp470) -
           (-Scalar(0.1034955) * _tmp467 * _tmp479 +
            _tmp468 * (-_tmp462 * _tmp475 - _tmp463 * _tmp480 -
                       Scalar(1) / Scalar(2) *
                           (2 * _tmp464 * (_tmp372 + _tmp62) + 2 * _tmp465 * (_tmp281 + _tmp448)) /
                           _tmp466)) *
               std::cosh(_tmp469)) +
      _tmp475 * (-std::sinh(_tmp469) - std::sinh(_tmp470));
  _res(3, 0) =
      _tmp488 *
          (-Scalar(1.0) * _tmp502 * std::cosh(_tmp495) -
           (-Scalar(0.1034955) * _tmp492 * _tmp499 +
            _tmp493 * (-_tmp487 * _tmp497 - _tmp488 * _tmp502 -
                       Scalar(1) / Scalar(2) *
                           (2 * _tmp489 * (_tmp281 + _tmp92) + 2 * _tmp490 * (_tmp268 + _tmp62)) /
                           _tmp491)) *
               std::cosh(_tmp494)) +
      _tmp497 * (-std::sinh(_tmp494) - std::sinh(_tmp495));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
