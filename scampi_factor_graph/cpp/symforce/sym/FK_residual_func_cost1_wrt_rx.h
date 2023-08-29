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
 * Symbolic function: FK_residual_func_cost1_wrt_rx
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
Eigen::Matrix<Scalar, 4, 1> FkResidualFuncCost1WrtRx(
    const Scalar fh1, const Scalar fv1, const Scalar rx, const Scalar ry, const Scalar rz,
    const Scalar tx, const Scalar ty, const Scalar tz, const Scalar rot_init_x,
    const Scalar rot_init_y, const Scalar rot_init_z, const Scalar rot_init_w, const Scalar lc0,
    const Scalar lc1, const Scalar lc2, const Scalar lc3, const Scalar epsilon) {
  // Total ops: 1628

  // Unused inputs
  (void)tz;
  (void)lc0;
  (void)lc1;
  (void)lc2;
  (void)lc3;
  (void)epsilon;

  // Input arrays

  // Intermediate terms (504)
  const Scalar _tmp0 = std::pow(rx, Scalar(2));
  const Scalar _tmp1 = _tmp0 + std::pow(ry, Scalar(2)) + std::pow(rz, Scalar(2));
  const Scalar _tmp2 = std::sqrt(_tmp1);
  const Scalar _tmp3 = (Scalar(1) / Scalar(2)) * _tmp2;
  const Scalar _tmp4 = std::sin(_tmp3);
  const Scalar _tmp5 = _tmp4 / _tmp2;
  const Scalar _tmp6 = _tmp5 * rot_init_z;
  const Scalar _tmp7 = std::cos(_tmp3);
  const Scalar _tmp8 = _tmp7 * rot_init_z;
  const Scalar _tmp9 = (Scalar(1) / Scalar(2)) / _tmp1;
  const Scalar _tmp10 = _tmp0 * _tmp9;
  const Scalar _tmp11 = _tmp7 * rot_init_x;
  const Scalar _tmp12 = _tmp9 * rx;
  const Scalar _tmp13 = _tmp12 * rz;
  const Scalar _tmp14 = _tmp7 * rot_init_w;
  const Scalar _tmp15 = _tmp12 * ry;
  const Scalar _tmp16 = _tmp5 * rot_init_y;
  const Scalar _tmp17 = _tmp16 * rx;
  const Scalar _tmp18 = _tmp4 / (_tmp1 * std::sqrt(_tmp1));
  const Scalar _tmp19 = _tmp0 * _tmp18;
  const Scalar _tmp20 = _tmp18 * rx;
  const Scalar _tmp21 = _tmp20 * ry;
  const Scalar _tmp22 = _tmp20 * rz;
  const Scalar _tmp23 = _tmp10 * _tmp8 - _tmp11 * _tmp13 + _tmp14 * _tmp15 -
                        Scalar(1) / Scalar(2) * _tmp17 - _tmp19 * rot_init_z - _tmp21 * rot_init_w +
                        _tmp22 * rot_init_x + _tmp6;
  const Scalar _tmp24 = _tmp5 * rot_init_w;
  const Scalar _tmp25 = _tmp24 * rx;
  const Scalar _tmp26 = _tmp11 + _tmp16 * rz + _tmp25 - _tmp6 * ry;
  const Scalar _tmp27 = Scalar(0.41999999999999998) * _tmp26;
  const Scalar _tmp28 = _tmp23 * _tmp27;
  const Scalar _tmp29 = _tmp7 * rot_init_y;
  const Scalar _tmp30 = _tmp5 * rot_init_x;
  const Scalar _tmp31 = _tmp30 * rx;
  const Scalar _tmp32 = _tmp10 * _tmp14 + _tmp13 * _tmp29 - _tmp15 * _tmp8 - _tmp19 * rot_init_w +
                        _tmp21 * rot_init_z - _tmp22 * rot_init_y + _tmp24 -
                        Scalar(1) / Scalar(2) * _tmp31;
  const Scalar _tmp33 = _tmp6 * rx;
  const Scalar _tmp34 = _tmp24 * ry + _tmp29 - _tmp30 * rz + _tmp33;
  const Scalar _tmp35 = Scalar(0.41999999999999998) * _tmp34;
  const Scalar _tmp36 = _tmp32 * _tmp35;
  const Scalar _tmp37 = _tmp28 + _tmp36;
  const Scalar _tmp38 = -_tmp10 * _tmp11 - _tmp13 * _tmp8 - _tmp15 * _tmp29 + _tmp19 * rot_init_x +
                        _tmp21 * rot_init_y + _tmp22 * rot_init_z - Scalar(1) / Scalar(2) * _tmp25 -
                        _tmp30;
  const Scalar _tmp39 = -_tmp17 + _tmp24 * rz + _tmp30 * ry + _tmp8;
  const Scalar _tmp40 = Scalar(0.41999999999999998) * _tmp39;
  const Scalar _tmp41 = _tmp38 * _tmp40;
  const Scalar _tmp42 = -_tmp10 * _tmp29 + _tmp11 * _tmp15 + _tmp13 * _tmp14 - _tmp16 +
                        _tmp19 * rot_init_y - _tmp21 * rot_init_x - _tmp22 * rot_init_w -
                        Scalar(1) / Scalar(2) * _tmp33;
  const Scalar _tmp43 = _tmp14 - _tmp16 * ry - _tmp31 - _tmp6 * rz;
  const Scalar _tmp44 = Scalar(0.41999999999999998) * _tmp43;
  const Scalar _tmp45 = _tmp42 * _tmp44;
  const Scalar _tmp46 = _tmp41 + _tmp45;
  const Scalar _tmp47 = _tmp37 + _tmp46;
  const Scalar _tmp48 = Scalar(0.83999999999999997) * _tmp39;
  const Scalar _tmp49 = _tmp42 * _tmp48;
  const Scalar _tmp50 = -_tmp49;
  const Scalar _tmp51 = _tmp26 * _tmp32;
  const Scalar _tmp52 = Scalar(0.83999999999999997) * _tmp51;
  const Scalar _tmp53 = _tmp50 - _tmp52;
  const Scalar _tmp54 = _tmp26 * _tmp38;
  const Scalar _tmp55 = Scalar(0.021999999999999999) * _tmp54;
  const Scalar _tmp56 = Scalar(0.021999999999999999) * _tmp42;
  const Scalar _tmp57 = _tmp34 * _tmp56;
  const Scalar _tmp58 = Scalar(0.021999999999999999) * _tmp39;
  const Scalar _tmp59 = _tmp23 * _tmp58;
  const Scalar _tmp60 = Scalar(0.021999999999999999) * _tmp43;
  const Scalar _tmp61 = _tmp32 * _tmp60;
  const Scalar _tmp62 = -_tmp55 + _tmp57 + _tmp59 - _tmp61;
  const Scalar _tmp63 = _tmp53 + _tmp62;
  const Scalar _tmp64 = 2 * _tmp26 * _tmp34;
  const Scalar _tmp65 = 2 * _tmp43;
  const Scalar _tmp66 = _tmp39 * _tmp65;
  const Scalar _tmp67 = Scalar(0.20999999999999999) * _tmp64 + Scalar(0.20999999999999999) * _tmp66;
  const Scalar _tmp68 = -_tmp67;
  const Scalar _tmp69 = 2 * _tmp39;
  const Scalar _tmp70 = _tmp34 * _tmp69;
  const Scalar _tmp71 = _tmp26 * _tmp65;
  const Scalar _tmp72 = _tmp70 - _tmp71;
  const Scalar _tmp73 = -Scalar(0.010999999999999999) * _tmp72;
  const Scalar _tmp74 = -2 * std::pow(_tmp39, Scalar(2));
  const Scalar _tmp75 = 1 - 2 * std::pow(_tmp26, Scalar(2));
  const Scalar _tmp76 = Scalar(0.20999999999999999) * _tmp74 + Scalar(0.20999999999999999) * _tmp75;
  const Scalar _tmp77 = _tmp73 - _tmp76;
  const Scalar _tmp78 = _tmp68 + _tmp77;
  const Scalar _tmp79 = _tmp78 + ty;
  const Scalar _tmp80 = -_tmp79 + Scalar(-8.3196563700000006);
  const Scalar _tmp81 = -_tmp41 - _tmp45;
  const Scalar _tmp82 = _tmp37 + _tmp81;
  const Scalar _tmp83 = _tmp23 * _tmp34;
  const Scalar _tmp84 = Scalar(0.83999999999999997) * _tmp83;
  const Scalar _tmp85 = _tmp50 - _tmp84;
  const Scalar _tmp86 = _tmp34 * _tmp38;
  const Scalar _tmp87 = Scalar(0.021999999999999999) * _tmp86;
  const Scalar _tmp88 = _tmp26 * _tmp56;
  const Scalar _tmp89 = _tmp32 * _tmp58;
  const Scalar _tmp90 = _tmp23 * _tmp60;
  const Scalar _tmp91 = _tmp87 + _tmp88 + _tmp89 + _tmp90;
  const Scalar _tmp92 = _tmp85 + _tmp91;
  const Scalar _tmp93 = -2 * std::pow(_tmp34, Scalar(2));
  const Scalar _tmp94 = Scalar(0.20999999999999999) * _tmp74 +
                        Scalar(0.20999999999999999) * _tmp93 + Scalar(0.20999999999999999);
  const Scalar _tmp95 = -_tmp94;
  const Scalar _tmp96 = _tmp26 * _tmp69;
  const Scalar _tmp97 = _tmp34 * _tmp65;
  const Scalar _tmp98 = _tmp96 + _tmp97;
  const Scalar _tmp99 = -Scalar(0.010999999999999999) * _tmp98;
  const Scalar _tmp100 =
      Scalar(0.20999999999999999) * _tmp64 - Scalar(0.20999999999999999) * _tmp66;
  const Scalar _tmp101 = -_tmp100 + _tmp99;
  const Scalar _tmp102 = _tmp101 + _tmp95;
  const Scalar _tmp103 = _tmp102 + tx;
  const Scalar _tmp104 = -_tmp103 + Scalar(-1.9874742000000001);
  const Scalar _tmp105 = Scalar(1.0) / (fh1);
  const Scalar _tmp106 =
      std::sqrt(Scalar(std::pow(_tmp104, Scalar(2)) + std::pow(_tmp80, Scalar(2))));
  const Scalar _tmp107 = Scalar(0.41999999999999998) * _tmp86;
  const Scalar _tmp108 = _tmp27 * _tmp42;
  const Scalar _tmp109 = _tmp32 * _tmp40;
  const Scalar _tmp110 = _tmp23 * _tmp44;
  const Scalar _tmp111 = _tmp107 - _tmp108 - _tmp109 + _tmp110;
  const Scalar _tmp112 = Scalar(0.41999999999999998) * _tmp54;
  const Scalar _tmp113 = _tmp35 * _tmp42;
  const Scalar _tmp114 = _tmp23 * _tmp40;
  const Scalar _tmp115 = _tmp32 * _tmp44;
  const Scalar _tmp116 = Scalar(0.043999999999999997) * _tmp83;
  const Scalar _tmp117 = Scalar(0.043999999999999997) * _tmp51;
  const Scalar _tmp118 = _tmp116 + _tmp117;
  const Scalar _tmp119 = -_tmp112 - _tmp113 - _tmp114 - _tmp115 + _tmp118;
  const Scalar _tmp120 = _tmp111 + _tmp119;
  const Scalar _tmp121 = _tmp103 + Scalar(1.9874742000000001);
  const Scalar _tmp122 = _tmp79 + Scalar(8.3196563700000006);
  const Scalar _tmp123 = std::pow(_tmp121, Scalar(2)) + std::pow(_tmp122, Scalar(2));
  const Scalar _tmp124 = std::pow(_tmp123, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp125 = _tmp121 * _tmp124;
  const Scalar _tmp126 = _tmp125 * fh1;
  const Scalar _tmp127 = _tmp101 + _tmp94;
  const Scalar _tmp128 = _tmp127 + tx;
  const Scalar _tmp129 = _tmp128 + Scalar(-2.5202214700000001);
  const Scalar _tmp130 = _tmp67 + _tmp77;
  const Scalar _tmp131 = _tmp130 + ty;
  const Scalar _tmp132 = _tmp131 + Scalar(8.3888750099999996);
  const Scalar _tmp133 = std::pow(_tmp129, Scalar(2));
  const Scalar _tmp134 = std::pow(_tmp132, Scalar(2)) + _tmp133;
  const Scalar _tmp135 = std::sqrt(_tmp134);
  const Scalar _tmp136 = Scalar(1.0) / (_tmp135);
  const Scalar _tmp137 = _tmp130 * _tmp136;
  const Scalar _tmp138 = _tmp132 * _tmp136;
  const Scalar _tmp139 = _tmp127 * _tmp138 - _tmp129 * _tmp137;
  const Scalar _tmp140 = Scalar(1.0) / (_tmp129);
  const Scalar _tmp141 = _tmp135 * _tmp140;
  const Scalar _tmp142 = _tmp139 * _tmp141;
  const Scalar _tmp143 = _tmp132 * _tmp140;
  const Scalar _tmp144 = _tmp100 + _tmp99;
  const Scalar _tmp145 = _tmp144 + _tmp94;
  const Scalar _tmp146 = _tmp145 + tx;
  const Scalar _tmp147 = _tmp146 + Scalar(-2.71799795);
  const Scalar _tmp148 = _tmp73 + _tmp76;
  const Scalar _tmp149 = _tmp148 + _tmp67;
  const Scalar _tmp150 = _tmp149 + ty;
  const Scalar _tmp151 = _tmp150 + Scalar(-4.7752063900000001);
  const Scalar _tmp152 = std::pow(_tmp147, Scalar(2)) + std::pow(_tmp151, Scalar(2));
  const Scalar _tmp153 = std::pow(_tmp152, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp154 = _tmp147 * _tmp153;
  const Scalar _tmp155 = _tmp151 * _tmp153;
  const Scalar _tmp156 = _tmp143 * _tmp154 - _tmp155;
  const Scalar _tmp157 = Scalar(1.0) / (_tmp156);
  const Scalar _tmp158 = _tmp142 * _tmp154 - _tmp145 * _tmp155 + _tmp149 * _tmp154;
  const Scalar _tmp159 = _tmp157 * _tmp158;
  const Scalar _tmp160 =
      Scalar(0.20999999999999999) * _tmp70 + Scalar(0.20999999999999999) * _tmp71;
  const Scalar _tmp161 = -_tmp160;
  const Scalar _tmp162 =
      -Scalar(0.010999999999999999) * _tmp75 - Scalar(0.010999999999999999) * _tmp93;
  const Scalar _tmp163 =
      Scalar(0.20999999999999999) * _tmp96 - Scalar(0.20999999999999999) * _tmp97;
  const Scalar _tmp164 = _tmp162 + _tmp163;
  const Scalar _tmp165 = _tmp161 + _tmp164;
  const Scalar _tmp166 = _tmp154 * _tmp165;
  const Scalar _tmp167 = _tmp160 + _tmp164;
  const Scalar _tmp168 = -_tmp154 * _tmp167 + _tmp166;
  const Scalar _tmp169 = _tmp157 * _tmp168;
  const Scalar _tmp170 = _tmp143 * _tmp165;
  const Scalar _tmp171 = -_tmp154 * _tmp170 + _tmp155 * _tmp167;
  const Scalar _tmp172 = _tmp157 * _tmp171;
  const Scalar _tmp173 = _tmp143 * _tmp172 + _tmp170;
  const Scalar _tmp174 = Scalar(1.0) * _tmp130;
  const Scalar _tmp175 = -_tmp174;
  const Scalar _tmp176 = _tmp149 + _tmp175;
  const Scalar _tmp177 = Scalar(1.0) / (_tmp176);
  const Scalar _tmp178 = Scalar(1.0) * _tmp127;
  const Scalar _tmp179 = -_tmp145 + _tmp178;
  const Scalar _tmp180 = _tmp177 * _tmp179;
  const Scalar _tmp181 = _tmp143 * _tmp169 - _tmp165 - _tmp173 * _tmp180;
  const Scalar _tmp182 = _tmp144 + _tmp95;
  const Scalar _tmp183 = _tmp182 + tx;
  const Scalar _tmp184 = _tmp183 + Scalar(1.79662371);
  const Scalar _tmp185 = _tmp148 + _tmp68;
  const Scalar _tmp186 = _tmp185 + ty;
  const Scalar _tmp187 = _tmp186 + Scalar(-4.8333311099999996);
  const Scalar _tmp188 = std::pow(_tmp184, Scalar(2)) + std::pow(_tmp187, Scalar(2));
  const Scalar _tmp189 = std::pow(_tmp188, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp190 = _tmp184 * _tmp189;
  const Scalar _tmp191 = _tmp165 * _tmp190;
  const Scalar _tmp192 = _tmp162 - _tmp163;
  const Scalar _tmp193 = _tmp160 + _tmp192;
  const Scalar _tmp194 = _tmp187 * _tmp189;
  const Scalar _tmp195 = _tmp143 * _tmp190 - _tmp194;
  const Scalar _tmp196 = _tmp157 * _tmp195;
  const Scalar _tmp197 = -_tmp143 * _tmp191 - _tmp171 * _tmp196 + _tmp193 * _tmp194;
  const Scalar _tmp198 = -_tmp168 * _tmp196 - _tmp180 * _tmp197 - _tmp190 * _tmp193 + _tmp191;
  const Scalar _tmp199 = Scalar(1.0) / (_tmp198);
  const Scalar _tmp200 =
      _tmp142 * _tmp190 - _tmp158 * _tmp196 - _tmp182 * _tmp194 + _tmp185 * _tmp190;
  const Scalar _tmp201 = _tmp199 * _tmp200;
  const Scalar _tmp202 = -_tmp142 + _tmp143 * _tmp159 - _tmp181 * _tmp201;
  const Scalar _tmp203 = Scalar(1.0) / (_tmp200);
  const Scalar _tmp204 = _tmp198 * _tmp203;
  const Scalar _tmp205 = _tmp202 * _tmp204;
  const Scalar _tmp206 = _tmp181 + _tmp205;
  const Scalar _tmp207 = _tmp190 * _tmp199;
  const Scalar _tmp208 = _tmp195 * _tmp199;
  const Scalar _tmp209 = -_tmp143 - _tmp206 * _tmp208;
  const Scalar _tmp210 = _tmp154 * _tmp157;
  const Scalar _tmp211 = _tmp206 * _tmp207 + _tmp209 * _tmp210 + Scalar(1.0);
  const Scalar _tmp212 = _tmp141 * _tmp211;
  const Scalar _tmp213 = _tmp122 * _tmp124;
  const Scalar _tmp214 = _tmp124 * _tmp78;
  const Scalar _tmp215 = _tmp102 * _tmp213 - _tmp121 * _tmp214;
  const Scalar _tmp216 = _tmp215 * fh1;
  const Scalar _tmp217 = Scalar(1.0) * _tmp203;
  const Scalar _tmp218 = _tmp154 * _tmp196;
  const Scalar _tmp219 = _tmp190 * _tmp217 - _tmp217 * _tmp218;
  const Scalar _tmp220 = _tmp141 * _tmp219;
  const Scalar _tmp221 = _tmp213 * fh1;
  const Scalar _tmp222 = Scalar(1.0) * _tmp172;
  const Scalar _tmp223 = -Scalar(1.0) * _tmp169 + _tmp180 * _tmp222;
  const Scalar _tmp224 = Scalar(1.0) * _tmp157;
  const Scalar _tmp225 = -_tmp158 * _tmp224 - _tmp201 * _tmp223;
  const Scalar _tmp226 = _tmp204 * _tmp225;
  const Scalar _tmp227 = _tmp223 + _tmp226;
  const Scalar _tmp228 = -_tmp208 * _tmp227 + Scalar(1.0);
  const Scalar _tmp229 = _tmp207 * _tmp227 + _tmp210 * _tmp228;
  const Scalar _tmp230 = _tmp141 * _tmp229;
  const Scalar _tmp231 = _tmp174 * _tmp180 + _tmp178;
  const Scalar _tmp232 = 0;
  const Scalar _tmp233 = _tmp199 * _tmp232;
  const Scalar _tmp234 = _tmp190 * _tmp233 - _tmp218 * _tmp233;
  const Scalar _tmp235 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp236 = _tmp141 * _tmp235;
  const Scalar _tmp237 =
      -_tmp126 * _tmp212 - _tmp216 * _tmp220 - _tmp221 * _tmp230 - _tmp234 * _tmp236;
  const Scalar _tmp238 = Scalar(1.0) / (_tmp237);
  const Scalar _tmp239 = _tmp175 + _tmp185;
  const Scalar _tmp240 = _tmp180 * _tmp239;
  const Scalar _tmp241 = _tmp178 - _tmp182 - _tmp240;
  const Scalar _tmp242 = Scalar(1.0) / (_tmp241);
  const Scalar _tmp243 = Scalar(1.0) * _tmp242;
  const Scalar _tmp244 = _tmp240 * _tmp243 + Scalar(1.0);
  const Scalar _tmp245 = _tmp177 * _tmp244;
  const Scalar _tmp246 = _tmp180 * _tmp243;
  const Scalar _tmp247 = -Scalar(1.0) * _tmp245 + _tmp246;
  const Scalar _tmp248 = _tmp161 + _tmp192;
  const Scalar _tmp249 = -_tmp221 * _tmp248 - Scalar(3.29616) * _tmp72 - _tmp78 * fv1;
  const Scalar _tmp250 = Scalar(1.0) * _tmp249;
  const Scalar _tmp251 = _tmp248 * fh1;
  const Scalar _tmp252 = _tmp102 * fv1 + _tmp125 * _tmp251 + Scalar(3.29616) * _tmp98;
  const Scalar _tmp253 = _tmp177 * _tmp243;
  const Scalar _tmp254 = Scalar(1.0) * _tmp239 * _tmp253 - Scalar(1.0) * _tmp243;
  const Scalar _tmp255 = _tmp239 * _tmp242;
  const Scalar _tmp256 = _tmp197 * _tmp199;
  const Scalar _tmp257 = -_tmp222 + _tmp226 * _tmp255 - _tmp227 * _tmp256;
  const Scalar _tmp258 = Scalar(1.0) * _tmp177;
  const Scalar _tmp259 = _tmp226 * _tmp243 - _tmp257 * _tmp258;
  const Scalar _tmp260 = Scalar(1.0) * fh1;
  const Scalar _tmp261 = _tmp213 * _tmp260;
  const Scalar _tmp262 = _tmp173 + _tmp205 * _tmp255 - _tmp206 * _tmp256;
  const Scalar _tmp263 = _tmp205 * _tmp243 - _tmp258 * _tmp262;
  const Scalar _tmp264 = _tmp260 * _tmp263;
  const Scalar _tmp265 = _tmp231 * _tmp242;
  const Scalar _tmp266 = _tmp175 - _tmp232 * _tmp256 - _tmp239 * _tmp265;
  const Scalar _tmp267 = _tmp177 * _tmp266;
  const Scalar _tmp268 = Scalar(1.0) * _tmp235;
  const Scalar _tmp269 = _tmp204 * _tmp243;
  const Scalar _tmp270 = -_tmp197 * _tmp217 + _tmp239 * _tmp269;
  const Scalar _tmp271 = _tmp260 * (-_tmp258 * _tmp270 + _tmp269);
  const Scalar _tmp272 = _tmp125 * _tmp264 + _tmp215 * _tmp271 + _tmp247 * _tmp250 +
                         _tmp252 * _tmp254 + _tmp259 * _tmp261 +
                         _tmp268 * (-_tmp231 * _tmp243 - Scalar(1.0) * _tmp267 + Scalar(1.0));
  const Scalar _tmp273 = std::asinh(_tmp238 * _tmp272);
  const Scalar _tmp274 = Scalar(9.6622558468725703) * _tmp237;
  const Scalar _tmp275 = Scalar(2.5202214700000001) - _tmp128;
  const Scalar _tmp276 = -_tmp131 + Scalar(-8.3888750099999996);
  const Scalar _tmp277 =
      std::sqrt(Scalar(std::pow(_tmp275, Scalar(2)) + std::pow(_tmp276, Scalar(2))));
  const Scalar _tmp278 = -_tmp273 * _tmp274 - _tmp277;
  const Scalar _tmp279 = Scalar(0.1034955) * _tmp238;
  const Scalar _tmp280 = _tmp278 * _tmp279;
  const Scalar _tmp281 = Scalar(1.0) * _tmp273;
  const Scalar _tmp282 = _tmp55 - _tmp57 - _tmp59 + _tmp61;
  const Scalar _tmp283 = _tmp49 + _tmp52;
  const Scalar _tmp284 = _tmp283 + _tmp47;
  const Scalar _tmp285 = _tmp282 + _tmp284;
  const Scalar _tmp286 = _tmp140 * _tmp285;
  const Scalar _tmp287 = _tmp282 + _tmp53;
  const Scalar _tmp288 = _tmp287 + _tmp47;
  const Scalar _tmp289 = _tmp153 * _tmp288;
  const Scalar _tmp290 = -_tmp87 - _tmp88 - _tmp89 - _tmp90;
  const Scalar _tmp291 = _tmp290 + _tmp85;
  const Scalar _tmp292 = _tmp291 + _tmp82;
  const Scalar _tmp293 =
      (2 * _tmp147 * _tmp292 + 2 * _tmp151 * _tmp288) / (_tmp152 * std::sqrt(_tmp152));
  const Scalar _tmp294 = (Scalar(1) / Scalar(2)) * _tmp293;
  const Scalar _tmp295 = _tmp151 * _tmp294;
  const Scalar _tmp296 = -_tmp28 - _tmp36;
  const Scalar _tmp297 = _tmp296 + _tmp46;
  const Scalar _tmp298 = _tmp291 + _tmp297;
  const Scalar _tmp299 = _tmp298 / _tmp133;
  const Scalar _tmp300 = _tmp132 * _tmp299;
  const Scalar _tmp301 = _tmp147 * _tmp294;
  const Scalar _tmp302 = _tmp153 * _tmp292;
  const Scalar _tmp303 = (-_tmp143 * _tmp301 + _tmp143 * _tmp302 + _tmp154 * _tmp286 -
                          _tmp154 * _tmp300 - _tmp289 + _tmp295) /
                         std::pow(_tmp156, Scalar(2));
  const Scalar _tmp304 = _tmp154 * _tmp303;
  const Scalar _tmp305 = _tmp157 * _tmp302;
  const Scalar _tmp306 = _tmp195 * _tmp303;
  const Scalar _tmp307 = -_tmp107 + _tmp108 + _tmp109 - _tmp110;
  const Scalar _tmp308 = _tmp119 + _tmp307;
  const Scalar _tmp309 = _tmp112 + _tmp113 + _tmp114 + _tmp115 + _tmp118;
  const Scalar _tmp310 = _tmp111 + _tmp309;
  const Scalar _tmp311 = _tmp296 + _tmp81;
  const Scalar _tmp312 = _tmp287 + _tmp311;
  const Scalar _tmp313 = _tmp49 + _tmp84;
  const Scalar _tmp314 = _tmp290 + _tmp313;
  const Scalar _tmp315 = _tmp314 + _tmp82;
  const Scalar _tmp316 =
      (2 * _tmp184 * _tmp315 + 2 * _tmp187 * _tmp312) / (_tmp188 * std::sqrt(_tmp188));
  const Scalar _tmp317 = (Scalar(1) / Scalar(2)) * _tmp316;
  const Scalar _tmp318 = _tmp187 * _tmp317;
  const Scalar _tmp319 = _tmp189 * _tmp315;
  const Scalar _tmp320 = _tmp165 * _tmp319;
  const Scalar _tmp321 = _tmp143 * _tmp308;
  const Scalar _tmp322 = _tmp184 * _tmp317;
  const Scalar _tmp323 = _tmp189 * _tmp312;
  const Scalar _tmp324 = _tmp143 * _tmp319 - _tmp143 * _tmp322 + _tmp190 * _tmp286 -
                         _tmp190 * _tmp300 + _tmp318 - _tmp323;
  const Scalar _tmp325 = _tmp154 * _tmp308;
  const Scalar _tmp326 = _tmp307 + _tmp309;
  const Scalar _tmp327 = _tmp165 * _tmp286;
  const Scalar _tmp328 = _tmp165 * _tmp302;
  const Scalar _tmp329 = -_tmp143 * _tmp325 - _tmp143 * _tmp328 - _tmp154 * _tmp327 +
                         _tmp155 * _tmp326 + _tmp166 * _tmp300 + _tmp167 * _tmp289 -
                         _tmp167 * _tmp295 + _tmp170 * _tmp301;
  const Scalar _tmp330 = -_tmp143 * _tmp320 + _tmp170 * _tmp322 + _tmp171 * _tmp306 -
                         _tmp172 * _tmp324 - _tmp190 * _tmp321 - _tmp191 * _tmp286 +
                         _tmp191 * _tmp300 - _tmp193 * _tmp318 + _tmp193 * _tmp323 +
                         _tmp194 * _tmp310 - _tmp196 * _tmp329;
  const Scalar _tmp331 = -_tmp154 * _tmp326 - _tmp165 * _tmp301 + _tmp167 * _tmp301 -
                         _tmp167 * _tmp302 + _tmp325 + _tmp328;
  const Scalar _tmp332 = -Scalar(1.6799999999999999) * _tmp39 * _tmp42;
  const Scalar _tmp333 = _tmp332 - Scalar(1.6799999999999999) * _tmp51;
  const Scalar _tmp334 = _tmp333 / std::pow(_tmp176, Scalar(2));
  const Scalar _tmp335 = _tmp179 * _tmp334;
  const Scalar _tmp336 = _tmp38 * _tmp48;
  const Scalar _tmp337 = Scalar(0.83999999999999997) * _tmp42 * _tmp43;
  const Scalar _tmp338 = -Scalar(0.83999999999999997) * _tmp23 * _tmp26 -
                         Scalar(0.83999999999999997) * _tmp32 * _tmp34;
  const Scalar _tmp339 = _tmp336 + _tmp337 + _tmp338;
  const Scalar _tmp340 = _tmp177 * _tmp339;
  const Scalar _tmp341 = -_tmp165 * _tmp322 + _tmp168 * _tmp306 - _tmp169 * _tmp324 -
                         _tmp180 * _tmp330 + _tmp190 * _tmp308 - _tmp190 * _tmp310 -
                         _tmp193 * _tmp319 + _tmp193 * _tmp322 - _tmp196 * _tmp331 +
                         _tmp197 * _tmp335 - _tmp197 * _tmp340 + _tmp320;
  const Scalar _tmp342 = _tmp341 / std::pow(_tmp198, Scalar(2));
  const Scalar _tmp343 = _tmp190 * _tmp342;
  const Scalar _tmp344 = _tmp157 * _tmp209;
  const Scalar _tmp345 = _tmp203 * _tmp341;
  const Scalar _tmp346 = _tmp202 * _tmp345;
  const Scalar _tmp347 = _tmp129 * _tmp298 + _tmp132 * _tmp285;
  const Scalar _tmp348 = _tmp136 * _tmp140 * _tmp347;
  const Scalar _tmp349 = _tmp139 * _tmp348;
  const Scalar _tmp350 = _tmp347 / (_tmp134 * std::sqrt(_tmp134));
  const Scalar _tmp351 = _tmp136 * _tmp285;
  const Scalar _tmp352 =
      _tmp141 * (-_tmp127 * _tmp132 * _tmp350 + _tmp127 * _tmp351 + _tmp129 * _tmp130 * _tmp350 -
                 _tmp129 * _tmp351 - _tmp137 * _tmp298 + _tmp138 * _tmp298);
  const Scalar _tmp353 = _tmp135 * _tmp299;
  const Scalar _tmp354 = _tmp139 * _tmp353;
  const Scalar _tmp355 = -_tmp142 * _tmp301 + _tmp142 * _tmp302 - _tmp145 * _tmp289 +
                         _tmp145 * _tmp295 + _tmp147 * _tmp289 - _tmp149 * _tmp301 +
                         _tmp149 * _tmp302 + _tmp154 * _tmp349 + _tmp154 * _tmp352 -
                         _tmp154 * _tmp354 - _tmp155 * _tmp292;
  const Scalar _tmp356 = _tmp142 * _tmp319 - _tmp142 * _tmp322 + _tmp158 * _tmp306 -
                         _tmp159 * _tmp324 + _tmp182 * _tmp318 - _tmp182 * _tmp323 +
                         _tmp184 * _tmp323 + _tmp185 * _tmp319 - _tmp185 * _tmp322 +
                         _tmp190 * _tmp349 + _tmp190 * _tmp352 - _tmp190 * _tmp354 -
                         _tmp194 * _tmp315 - _tmp196 * _tmp355;
  const Scalar _tmp357 = _tmp356 / std::pow(_tmp200, Scalar(2));
  const Scalar _tmp358 = _tmp198 * _tmp357;
  const Scalar _tmp359 = _tmp202 * _tmp358;
  const Scalar _tmp360 = _tmp143 * _tmp157;
  const Scalar _tmp361 = _tmp143 * _tmp303;
  const Scalar _tmp362 = -_tmp165 * _tmp300 - _tmp171 * _tmp361 + _tmp172 * _tmp286 -
                         _tmp172 * _tmp300 + _tmp321 + _tmp327 + _tmp329 * _tmp360;
  const Scalar _tmp363 = _tmp111 + _tmp112 + _tmp113 + _tmp114 + _tmp115 - _tmp116 - _tmp117 -
                         _tmp168 * _tmp361 + _tmp169 * _tmp286 - _tmp169 * _tmp300 +
                         _tmp173 * _tmp335 - _tmp173 * _tmp340 - _tmp180 * _tmp362 +
                         _tmp331 * _tmp360;
  const Scalar _tmp364 = _tmp200 * _tmp342;
  const Scalar _tmp365 = _tmp199 * _tmp356;
  const Scalar _tmp366 = _tmp204 * (-_tmp158 * _tmp361 + _tmp159 * _tmp286 - _tmp159 * _tmp300 +
                                    _tmp181 * _tmp364 - _tmp181 * _tmp365 - _tmp201 * _tmp363 -
                                    _tmp349 - _tmp352 + _tmp354 + _tmp355 * _tmp360);
  const Scalar _tmp367 = _tmp346 - _tmp359 + _tmp363 + _tmp366;
  const Scalar _tmp368 = _tmp195 * _tmp342;
  const Scalar _tmp369 = _tmp199 * _tmp324;
  const Scalar _tmp370 =
      _tmp206 * _tmp368 - _tmp206 * _tmp369 - _tmp208 * _tmp367 - _tmp286 + _tmp300;
  const Scalar _tmp371 = _tmp199 * _tmp319;
  const Scalar _tmp372 = _tmp199 * _tmp206;
  const Scalar _tmp373 = _tmp216 * _tmp219;
  const Scalar _tmp374 = _tmp283 + _tmp311;
  const Scalar _tmp375 = _tmp282 + _tmp374;
  const Scalar _tmp376 = _tmp297 + _tmp314;
  const Scalar _tmp377 =
      (2 * _tmp121 * _tmp376 + 2 * _tmp122 * _tmp375) / (_tmp123 * std::sqrt(_tmp123));
  const Scalar _tmp378 = (Scalar(1) / Scalar(2)) * _tmp377;
  const Scalar _tmp379 = _tmp121 * _tmp378;
  const Scalar _tmp380 = _tmp379 * fh1;
  const Scalar _tmp381 = Scalar(0.5) * _tmp203;
  const Scalar _tmp382 = _tmp196 * _tmp302;
  const Scalar _tmp383 = Scalar(1.0) * _tmp357;
  const Scalar _tmp384 = _tmp203 * _tmp224 * _tmp324;
  const Scalar _tmp385 = _tmp225 * _tmp358;
  const Scalar _tmp386 = _tmp225 * _tmp345;
  const Scalar _tmp387 = Scalar(1.0) * _tmp303;
  const Scalar _tmp388 = _tmp224 * _tmp329;
  const Scalar _tmp389 = Scalar(1.0) * _tmp180;
  const Scalar _tmp390 = _tmp168 * _tmp387 - _tmp171 * _tmp303 * _tmp389 + _tmp180 * _tmp388 -
                         _tmp222 * _tmp335 + _tmp222 * _tmp340 - _tmp224 * _tmp331;
  const Scalar _tmp391 = _tmp204 * (_tmp158 * _tmp387 - _tmp201 * _tmp390 + _tmp223 * _tmp364 -
                                    _tmp223 * _tmp365 - _tmp224 * _tmp355);
  const Scalar _tmp392 = -_tmp385 + _tmp386 + _tmp390 + _tmp391;
  const Scalar _tmp393 = -_tmp208 * _tmp392 + _tmp227 * _tmp368 - _tmp227 * _tmp369;
  const Scalar _tmp394 = _tmp199 * _tmp227;
  const Scalar _tmp395 = _tmp157 * _tmp228;
  const Scalar _tmp396 = _tmp232 * _tmp342;
  const Scalar _tmp397 = _tmp234 * _tmp235;
  const Scalar _tmp398 = _tmp124 * _tmp375;
  const Scalar _tmp399 = _tmp122 * _tmp378;
  const Scalar _tmp400 = _tmp102 * _tmp398 - _tmp102 * _tmp399 - _tmp121 * _tmp398 +
                         _tmp213 * _tmp376 - _tmp214 * _tmp376 + _tmp379 * _tmp78;
  const Scalar _tmp401 = _tmp400 * fh1;
  const Scalar _tmp402 = _tmp126 * _tmp211;
  const Scalar _tmp403 = _tmp221 * _tmp229;
  const Scalar _tmp404 = _tmp398 * fh1;
  const Scalar _tmp405 = _tmp124 * _tmp376;
  const Scalar _tmp406 = _tmp405 * fh1;
  const Scalar _tmp407 = _tmp399 * fh1;
  const Scalar _tmp408 =
      -_tmp126 * _tmp141 *
          (-_tmp206 * _tmp343 + _tmp206 * _tmp371 + _tmp207 * _tmp367 - _tmp209 * _tmp304 +
           _tmp209 * _tmp305 + _tmp210 * _tmp370 - _tmp301 * _tmp344 - _tmp322 * _tmp372) -
      _tmp141 * _tmp216 *
          (_tmp147 * _tmp196 * _tmp293 * _tmp381 + _tmp154 * _tmp217 * _tmp306 - _tmp154 * _tmp384 -
           _tmp184 * _tmp316 * _tmp381 - _tmp190 * _tmp383 + _tmp217 * _tmp319 - _tmp217 * _tmp382 +
           _tmp218 * _tmp383) -
      _tmp141 * _tmp221 *
          (_tmp207 * _tmp392 + _tmp210 * _tmp393 - _tmp227 * _tmp343 + _tmp227 * _tmp371 -
           _tmp228 * _tmp304 + _tmp228 * _tmp305 - _tmp301 * _tmp395 - _tmp322 * _tmp394) +
      _tmp212 * _tmp380 - _tmp212 * _tmp406 - _tmp220 * _tmp401 - _tmp230 * _tmp404 +
      _tmp230 * _tmp407 -
      _tmp236 * (-_tmp190 * _tmp396 + _tmp196 * _tmp233 * _tmp301 + _tmp208 * _tmp232 * _tmp304 -
                 _tmp210 * _tmp232 * _tmp369 + _tmp218 * _tmp396 + _tmp233 * _tmp319 -
                 _tmp233 * _tmp322 - _tmp233 * _tmp382) -
      _tmp348 * _tmp373 - _tmp348 * _tmp397 - _tmp348 * _tmp402 - _tmp348 * _tmp403 +
      _tmp353 * _tmp373 + _tmp353 * _tmp397 + _tmp353 * _tmp402 + _tmp353 * _tmp403;
  const Scalar _tmp409 = Scalar(9.6622558468725703) * _tmp408;
  const Scalar _tmp410 = std::pow(_tmp237, Scalar(-2));
  const Scalar _tmp411 = _tmp408 * _tmp410;
  const Scalar _tmp412 = Scalar(1.0) * _tmp334;
  const Scalar _tmp413 = _tmp243 * _tmp345;
  const Scalar _tmp414 = _tmp243 * _tmp358;
  const Scalar _tmp415 = _tmp239 * _tmp340;
  const Scalar _tmp416 = _tmp333 - _tmp336 - _tmp337 + _tmp338;
  const Scalar _tmp417 = _tmp180 * _tmp416;
  const Scalar _tmp418 = _tmp239 * _tmp335;
  const Scalar _tmp419 =
      (_tmp332 + _tmp339 - _tmp415 - _tmp417 + _tmp418 - Scalar(1.6799999999999999) * _tmp83) /
      std::pow(_tmp241, Scalar(2));
  const Scalar _tmp420 = Scalar(1.0) * _tmp419;
  const Scalar _tmp421 = _tmp204 * _tmp420;
  const Scalar _tmp422 = _tmp197 * _tmp383 - _tmp217 * _tmp330 + _tmp239 * _tmp413 -
                         _tmp239 * _tmp414 - _tmp239 * _tmp421 + _tmp269 * _tmp416;
  const Scalar _tmp423 = _tmp215 * _tmp260;
  const Scalar _tmp424 = _tmp199 * _tmp330;
  const Scalar _tmp425 = _tmp242 * _tmp416;
  const Scalar _tmp426 = _tmp239 * _tmp419;
  const Scalar _tmp427 = _tmp197 * _tmp342;
  const Scalar _tmp428 = _tmp171 * _tmp387 + _tmp226 * _tmp425 - _tmp226 * _tmp426 -
                         _tmp227 * _tmp424 + _tmp227 * _tmp427 - _tmp255 * _tmp385 +
                         _tmp255 * _tmp386 + _tmp255 * _tmp391 - _tmp256 * _tmp392 - _tmp388;
  const Scalar _tmp429 =
      -_tmp240 * _tmp420 + _tmp243 * _tmp415 + _tmp243 * _tmp417 - _tmp243 * _tmp418;
  const Scalar _tmp430 = _tmp244 * _tmp334;
  const Scalar _tmp431 = _tmp243 * _tmp335;
  const Scalar _tmp432 = Scalar(6.59232) * _tmp42;
  const Scalar _tmp433 = Scalar(6.59232) * _tmp39;
  const Scalar _tmp434 = Scalar(6.59232) * _tmp43;
  const Scalar _tmp435 = _tmp120 * _tmp126 + _tmp23 * _tmp434 - _tmp251 * _tmp379 +
                         _tmp251 * _tmp405 + _tmp26 * _tmp432 + _tmp32 * _tmp433 + _tmp376 * fv1 +
                         Scalar(6.59232) * _tmp86;
  const Scalar _tmp436 = -_tmp174 * _tmp335 + _tmp174 * _tmp340 + _tmp285 * _tmp389 + _tmp298;
  const Scalar _tmp437 = _tmp231 * _tmp419;
  const Scalar _tmp438 = _tmp242 * _tmp436;
  const Scalar _tmp439 = _tmp311 + _tmp63;
  const Scalar _tmp440 = -_tmp232 * _tmp424 + _tmp232 * _tmp427 + _tmp239 * _tmp437 -
                         _tmp239 * _tmp438 - _tmp265 * _tmp416 + _tmp439;
  const Scalar _tmp441 = Scalar(0.5) * _tmp377 * fh1;
  const Scalar _tmp442 = _tmp239 * _tmp334;
  const Scalar _tmp443 = _tmp258 * _tmp426;
  const Scalar _tmp444 = _tmp205 * _tmp425 - _tmp205 * _tmp426 - _tmp206 * _tmp424 +
                         _tmp206 * _tmp427 + _tmp255 * _tmp346 - _tmp255 * _tmp359 +
                         _tmp255 * _tmp366 - _tmp256 * _tmp367 + _tmp362;
  const Scalar _tmp445 = -_tmp120 * _tmp221 - _tmp23 * _tmp433 - _tmp251 * _tmp398 +
                         _tmp251 * _tmp399 + _tmp32 * _tmp434 - _tmp34 * _tmp432 - _tmp375 * fv1 +
                         Scalar(6.59232) * _tmp54;
  const Scalar _tmp446 =
      (_tmp238 *
           (-_tmp121 * _tmp263 * _tmp441 - _tmp122 * _tmp259 * _tmp441 +
            _tmp125 * _tmp260 *
                (-_tmp205 * _tmp420 + _tmp243 * _tmp346 - _tmp243 * _tmp359 + _tmp243 * _tmp366 -
                 _tmp258 * _tmp444 + _tmp262 * _tmp412) +
            Scalar(1.0) * _tmp247 * _tmp445 +
            _tmp250 * (-_tmp180 * _tmp420 + _tmp243 * _tmp340 - _tmp258 * _tmp429 +
                       Scalar(1.0) * _tmp430 - _tmp431) +
            Scalar(1.0) * _tmp252 * (-_tmp243 * _tmp442 + _tmp253 * _tmp416 + _tmp420 - _tmp443) +
            _tmp254 * _tmp435 + _tmp259 * _tmp260 * _tmp398 +
            _tmp261 * (-_tmp226 * _tmp420 - _tmp243 * _tmp385 + _tmp243 * _tmp386 +
                       _tmp243 * _tmp391 + _tmp257 * _tmp412 - _tmp258 * _tmp428) +
            _tmp264 * _tmp405 +
            _tmp268 *
                (_tmp231 * _tmp420 - _tmp243 * _tmp436 - _tmp258 * _tmp440 + _tmp266 * _tmp412) +
            _tmp271 * _tmp400 +
            _tmp423 * (-_tmp258 * _tmp422 + _tmp270 * _tmp412 + _tmp413 - _tmp414 - _tmp421)) -
       _tmp272 * _tmp411) /
      std::sqrt(Scalar(std::pow(_tmp272, Scalar(2)) * _tmp410 + 1));
  const Scalar _tmp447 = _tmp313 + _tmp91;
  const Scalar _tmp448 = _tmp126 * _tmp157;
  const Scalar _tmp449 = _tmp157 * _tmp221;
  const Scalar _tmp450 = _tmp233 * _tmp235;
  const Scalar _tmp451 = _tmp216 * _tmp217;
  const Scalar _tmp452 =
      -_tmp196 * _tmp450 - _tmp196 * _tmp451 + _tmp209 * _tmp448 + _tmp228 * _tmp449;
  const Scalar _tmp453 = std::pow(_tmp452, Scalar(-2));
  const Scalar _tmp454 = _tmp357 * _tmp423;
  const Scalar _tmp455 = _tmp232 * _tmp235;
  const Scalar _tmp456 = _tmp235 * _tmp396;
  const Scalar _tmp457 = _tmp217 * _tmp401;
  const Scalar _tmp458 =
      -_tmp126 * _tmp209 * _tmp303 - _tmp157 * _tmp369 * _tmp455 + _tmp196 * _tmp454 +
      _tmp196 * _tmp456 - _tmp196 * _tmp457 + _tmp208 * _tmp303 * _tmp455 - _tmp216 * _tmp384 -
      _tmp221 * _tmp228 * _tmp303 + _tmp306 * _tmp451 - _tmp344 * _tmp380 + _tmp344 * _tmp406 +
      _tmp370 * _tmp448 + _tmp393 * _tmp449 + _tmp395 * _tmp404 - _tmp395 * _tmp407;
  const Scalar _tmp459 = _tmp453 * _tmp458;
  const Scalar _tmp460 = Scalar(9.6622558468725703) * _tmp452;
  const Scalar _tmp461 = _tmp177 * _tmp216;
  const Scalar _tmp462 = _tmp243 * _tmp252;
  const Scalar _tmp463 = _tmp177 * _tmp239;
  const Scalar _tmp464 = _tmp126 * _tmp177;
  const Scalar _tmp465 = _tmp177 * _tmp249;
  const Scalar _tmp466 = _tmp177 * _tmp257;
  const Scalar _tmp467 = _tmp221 * _tmp466 + _tmp235 * _tmp267 + _tmp244 * _tmp465 +
                         _tmp262 * _tmp464 + _tmp270 * _tmp461 - _tmp462 * _tmp463;
  const Scalar _tmp468 = Scalar(1.0) / (_tmp452);
  const Scalar _tmp469 = _tmp177 * _tmp262;
  const Scalar _tmp470 = _tmp243 * _tmp435;
  const Scalar _tmp471 =
      (-_tmp459 * _tmp467 +
       _tmp468 * (-_tmp126 * _tmp262 * _tmp334 + _tmp177 * _tmp221 * _tmp428 +
                  _tmp177 * _tmp235 * _tmp440 + _tmp177 * _tmp270 * _tmp401 -
                  _tmp177 * _tmp416 * _tmp462 - _tmp216 * _tmp270 * _tmp334 -
                  _tmp221 * _tmp257 * _tmp334 - _tmp235 * _tmp266 * _tmp334 + _tmp245 * _tmp445 -
                  _tmp249 * _tmp430 + _tmp252 * _tmp443 - _tmp380 * _tmp469 + _tmp404 * _tmp466 +
                  _tmp406 * _tmp469 - _tmp407 * _tmp466 + _tmp422 * _tmp461 + _tmp429 * _tmp465 +
                  _tmp442 * _tmp462 + _tmp444 * _tmp464 - _tmp463 * _tmp470)) /
      std::sqrt(Scalar(_tmp453 * std::pow(_tmp467, Scalar(2)) + 1));
  const Scalar _tmp472 = Scalar(2.71799795) - _tmp146;
  const Scalar _tmp473 = Scalar(4.7752063900000001) - _tmp150;
  const Scalar _tmp474 =
      std::sqrt(Scalar(std::pow(_tmp472, Scalar(2)) + std::pow(_tmp473, Scalar(2))));
  const Scalar _tmp475 = std::asinh(_tmp467 * _tmp468);
  const Scalar _tmp476 = Scalar(9.6622558468725703) * _tmp458;
  const Scalar _tmp477 = Scalar(0.1034955) * _tmp468;
  const Scalar _tmp478 = -_tmp460 * _tmp475 - _tmp474;
  const Scalar _tmp479 = _tmp477 * _tmp478;
  const Scalar _tmp480 = Scalar(1.0) * _tmp475;
  const Scalar _tmp481 = _tmp199 * _tmp221;
  const Scalar _tmp482 = _tmp126 * _tmp372 + _tmp227 * _tmp481 + _tmp450 + _tmp451;
  const Scalar _tmp483 = Scalar(1.0) / (_tmp482);
  const Scalar _tmp484 = _tmp243 * _tmp465;
  const Scalar _tmp485 = _tmp126 * _tmp242;
  const Scalar _tmp486 = _tmp221 * _tmp242;
  const Scalar _tmp487 = -_tmp179 * _tmp484 - _tmp205 * _tmp485 - _tmp216 * _tmp269 -
                         _tmp226 * _tmp486 + _tmp235 * _tmp265 + _tmp462;
  const Scalar _tmp488 = std::asinh(_tmp483 * _tmp487);
  const Scalar _tmp489 = Scalar(1.0) * _tmp488;
  const Scalar _tmp490 = Scalar(9.6622558468725703) * _tmp482;
  const Scalar _tmp491 = -_tmp183 + Scalar(-1.79662371);
  const Scalar _tmp492 = Scalar(4.8333311099999996) - _tmp186;
  const Scalar _tmp493 =
      std::sqrt(Scalar(std::pow(_tmp491, Scalar(2)) + std::pow(_tmp492, Scalar(2))));
  const Scalar _tmp494 = -_tmp488 * _tmp490 - _tmp493;
  const Scalar _tmp495 = Scalar(0.1034955) * _tmp483;
  const Scalar _tmp496 = _tmp494 * _tmp495;
  const Scalar _tmp497 = _tmp126 * _tmp199 * _tmp367 - _tmp126 * _tmp206 * _tmp342 -
                         _tmp221 * _tmp227 * _tmp342 - _tmp372 * _tmp380 + _tmp372 * _tmp406 +
                         _tmp392 * _tmp481 + _tmp394 * _tmp404 - _tmp394 * _tmp407 - _tmp454 -
                         _tmp456 + _tmp457;
  const Scalar _tmp498 = Scalar(9.6622558468725703) * _tmp497;
  const Scalar _tmp499 = std::pow(_tmp482, Scalar(-2));
  const Scalar _tmp500 = _tmp497 * _tmp499;
  const Scalar _tmp501 = _tmp226 * _tmp242;
  const Scalar _tmp502 = _tmp205 * _tmp242;
  const Scalar _tmp503 =
      (_tmp483 * (_tmp126 * _tmp205 * _tmp419 + _tmp179 * _tmp420 * _tmp465 - _tmp216 * _tmp413 +
                  _tmp216 * _tmp414 + _tmp216 * _tmp421 + _tmp221 * _tmp226 * _tmp419 -
                  _tmp235 * _tmp437 + _tmp235 * _tmp438 - _tmp246 * _tmp445 + _tmp249 * _tmp431 -
                  _tmp252 * _tmp420 - _tmp269 * _tmp401 - _tmp339 * _tmp484 - _tmp346 * _tmp485 +
                  _tmp359 * _tmp485 - _tmp366 * _tmp485 + _tmp380 * _tmp502 + _tmp385 * _tmp486 -
                  _tmp386 * _tmp486 - _tmp391 * _tmp486 - _tmp404 * _tmp501 - _tmp406 * _tmp502 +
                  _tmp407 * _tmp501 + _tmp470) -
       _tmp487 * _tmp500) /
      std::sqrt(Scalar(std::pow(_tmp487, Scalar(2)) * _tmp499 + 1));

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) =
      _tmp120 -
      Scalar(0.5) * (2 * _tmp104 * (_tmp82 + _tmp92) + 2 * _tmp80 * (_tmp47 + _tmp63)) *
          std::sinh(Scalar(0.1034955) * _tmp105 *
                    (-_tmp106 - Scalar(9.6622558468725703) * fh1 * std::asinh(_tmp105 * fv1))) /
          _tmp106;
  _res(1, 0) =
      -_tmp274 *
          (-Scalar(0.87653584775870996) * _tmp411 + Scalar(1.0) * _tmp446 * std::sinh(_tmp281) -
           (-Scalar(0.1034955) * _tmp278 * _tmp411 +
            _tmp279 * (-_tmp273 * _tmp409 - _tmp274 * _tmp446 -
                       Scalar(1) / Scalar(2) *
                           (2 * _tmp275 * (_tmp447 + _tmp82) + 2 * _tmp276 * _tmp439) / _tmp277)) *
               std::sinh(_tmp280)) +
      _tmp308 -
      _tmp409 * (Scalar(0.87653584775870996) * _tmp238 - std::cosh(_tmp280) + std::cosh(_tmp281));
  _res(2, 0) =
      _tmp326 -
      _tmp460 *
          (-Scalar(0.86565325453551001) * _tmp459 + Scalar(1.0) * _tmp471 * std::sinh(_tmp480) -
           (-Scalar(0.1034955) * _tmp459 * _tmp478 +
            _tmp477 * (-_tmp460 * _tmp471 - _tmp475 * _tmp476 -
                       Scalar(1) / Scalar(2) *
                           (2 * _tmp472 * (_tmp297 + _tmp447) + 2 * _tmp473 * (_tmp374 + _tmp62)) /
                           _tmp474)) *
               std::sinh(_tmp479)) -
      _tmp476 * (Scalar(0.86565325453551001) * _tmp468 - std::cosh(_tmp479) + std::cosh(_tmp480));
  _res(3, 0) =
      _tmp310 -
      _tmp490 *
          (-Scalar(0.86625939559540499) * _tmp500 + Scalar(1.0) * _tmp503 * std::sinh(_tmp489) -
           (-Scalar(0.1034955) * _tmp494 * _tmp500 +
            _tmp495 * (-_tmp488 * _tmp498 - _tmp490 * _tmp503 -
                       Scalar(1) / Scalar(2) *
                           (2 * _tmp491 * (_tmp297 + _tmp92) + 2 * _tmp492 * (_tmp284 + _tmp62)) /
                           _tmp493)) *
               std::sinh(_tmp496)) -
      _tmp498 * (Scalar(0.86625939559540499) * _tmp483 + std::cosh(_tmp489) - std::cosh(_tmp496));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
