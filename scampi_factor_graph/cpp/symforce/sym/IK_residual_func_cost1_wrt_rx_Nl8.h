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
 * Symbolic function: IK_residual_func_cost1_wrt_rx_Nl8
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost1WrtRxNl8(
    const Scalar fh1, const Scalar fv1, const Scalar rx, const Scalar ry, const Scalar rz,
    const Scalar p_init0, const Scalar p_init1, const Scalar p_init2, const Scalar rot_init_x,
    const Scalar rot_init_y, const Scalar rot_init_z, const Scalar rot_init_w,
    const Scalar epsilon) {
  // Total ops: 1632

  // Unused inputs
  (void)p_init2;
  (void)epsilon;

  // Input arrays

  // Intermediate terms (502)
  const Scalar _tmp0 = std::pow(rx, Scalar(2));
  const Scalar _tmp1 = _tmp0 + std::pow(ry, Scalar(2)) + std::pow(rz, Scalar(2));
  const Scalar _tmp2 = std::sqrt(_tmp1);
  const Scalar _tmp3 = (Scalar(1) / Scalar(2)) * _tmp2;
  const Scalar _tmp4 = std::sin(_tmp3);
  const Scalar _tmp5 = _tmp4 / _tmp2;
  const Scalar _tmp6 = _tmp5 * rot_init_y;
  const Scalar _tmp7 = std::cos(_tmp3);
  const Scalar _tmp8 = _tmp7 * rot_init_y;
  const Scalar _tmp9 = (Scalar(1) / Scalar(2)) / _tmp1;
  const Scalar _tmp10 = _tmp0 * _tmp9;
  const Scalar _tmp11 = _tmp7 * rot_init_w;
  const Scalar _tmp12 = _tmp9 * rx;
  const Scalar _tmp13 = _tmp12 * rz;
  const Scalar _tmp14 = _tmp7 * rot_init_x;
  const Scalar _tmp15 = _tmp12 * ry;
  const Scalar _tmp16 = _tmp5 * rot_init_z;
  const Scalar _tmp17 = _tmp16 * rx;
  const Scalar _tmp18 = _tmp4 / (_tmp1 * std::sqrt(_tmp1));
  const Scalar _tmp19 = _tmp0 * _tmp18;
  const Scalar _tmp20 = _tmp18 * rx;
  const Scalar _tmp21 = _tmp20 * ry;
  const Scalar _tmp22 = _tmp20 * rz;
  const Scalar _tmp23 = -_tmp10 * _tmp8 + _tmp11 * _tmp13 + _tmp14 * _tmp15 -
                        Scalar(1) / Scalar(2) * _tmp17 + _tmp19 * rot_init_y - _tmp21 * rot_init_x -
                        _tmp22 * rot_init_w - _tmp6;
  const Scalar _tmp24 = _tmp7 * rot_init_z;
  const Scalar _tmp25 = _tmp5 * rot_init_x;
  const Scalar _tmp26 = _tmp6 * rx;
  const Scalar _tmp27 = _tmp5 * rot_init_w;
  const Scalar _tmp28 = _tmp24 + _tmp25 * ry - _tmp26 + _tmp27 * rz;
  const Scalar _tmp29 = Scalar(0.83999999999999997) * _tmp28;
  const Scalar _tmp30 = _tmp23 * _tmp29;
  const Scalar _tmp31 = -_tmp30;
  const Scalar _tmp32 = _tmp27 * rx;
  const Scalar _tmp33 = _tmp14 - _tmp16 * ry + _tmp32 + _tmp6 * rz;
  const Scalar _tmp34 = _tmp25 * rx;
  const Scalar _tmp35 = _tmp10 * _tmp11 + _tmp13 * _tmp8 - _tmp15 * _tmp24 - _tmp19 * rot_init_w +
                        _tmp21 * rot_init_z - _tmp22 * rot_init_y + _tmp27 -
                        Scalar(1) / Scalar(2) * _tmp34;
  const Scalar _tmp36 = Scalar(0.83999999999999997) * _tmp35;
  const Scalar _tmp37 = _tmp33 * _tmp36;
  const Scalar _tmp38 = _tmp31 - _tmp37;
  const Scalar _tmp39 = _tmp10 * _tmp24 + _tmp11 * _tmp15 - _tmp13 * _tmp14 + _tmp16 -
                        _tmp19 * rot_init_z - _tmp21 * rot_init_w + _tmp22 * rot_init_x -
                        Scalar(1) / Scalar(2) * _tmp26;
  const Scalar _tmp40 = Scalar(0.41999999999999998) * _tmp33;
  const Scalar _tmp41 = _tmp39 * _tmp40;
  const Scalar _tmp42 = _tmp17 - _tmp25 * rz + _tmp27 * ry + _tmp8;
  const Scalar _tmp43 = Scalar(0.41999999999999998) * _tmp42;
  const Scalar _tmp44 = _tmp35 * _tmp43;
  const Scalar _tmp45 = -_tmp41 - _tmp44;
  const Scalar _tmp46 = -_tmp10 * _tmp14 - _tmp13 * _tmp24 - _tmp15 * _tmp8 + _tmp19 * rot_init_x +
                        _tmp21 * rot_init_y + _tmp22 * rot_init_z - _tmp25 -
                        Scalar(1) / Scalar(2) * _tmp32;
  const Scalar _tmp47 = Scalar(0.41999999999999998) * _tmp28;
  const Scalar _tmp48 = _tmp46 * _tmp47;
  const Scalar _tmp49 = _tmp11 - _tmp16 * rz - _tmp34 - _tmp6 * ry;
  const Scalar _tmp50 = _tmp23 * _tmp49;
  const Scalar _tmp51 = Scalar(0.41999999999999998) * _tmp50;
  const Scalar _tmp52 = -_tmp48 - _tmp51;
  const Scalar _tmp53 = _tmp45 + _tmp52;
  const Scalar _tmp54 = _tmp38 + _tmp53;
  const Scalar _tmp55 = Scalar(0.021999999999999999) * _tmp46;
  const Scalar _tmp56 = _tmp33 * _tmp55;
  const Scalar _tmp57 = Scalar(0.021999999999999999) * _tmp23;
  const Scalar _tmp58 = _tmp42 * _tmp57;
  const Scalar _tmp59 = Scalar(0.021999999999999999) * _tmp28;
  const Scalar _tmp60 = _tmp39 * _tmp59;
  const Scalar _tmp61 = _tmp35 * _tmp49;
  const Scalar _tmp62 = Scalar(0.021999999999999999) * _tmp61;
  const Scalar _tmp63 = -_tmp56 + _tmp58 + _tmp60 - _tmp62;
  const Scalar _tmp64 = 2 * _tmp42;
  const Scalar _tmp65 = _tmp33 * _tmp64;
  const Scalar _tmp66 = 2 * _tmp49;
  const Scalar _tmp67 = _tmp28 * _tmp66;
  const Scalar _tmp68 = Scalar(0.20999999999999999) * _tmp65 + Scalar(0.20999999999999999) * _tmp67;
  const Scalar _tmp69 = _tmp28 * _tmp64;
  const Scalar _tmp70 = _tmp33 * _tmp66;
  const Scalar _tmp71 = _tmp69 - _tmp70;
  const Scalar _tmp72 = -Scalar(0.010999999999999999) * _tmp71;
  const Scalar _tmp73 = -2 * std::pow(_tmp33, Scalar(2));
  const Scalar _tmp74 = -2 * std::pow(_tmp28, Scalar(2));
  const Scalar _tmp75 = Scalar(0.20999999999999999) * _tmp73 +
                        Scalar(0.20999999999999999) * _tmp74 + Scalar(0.20999999999999999);
  const Scalar _tmp76 = _tmp72 - _tmp75;
  const Scalar _tmp77 = _tmp68 + _tmp76;
  const Scalar _tmp78 = _tmp77 + p_init1;
  const Scalar _tmp79 = -_tmp78 + Scalar(-8.3888750099999996);
  const Scalar _tmp80 = _tmp41 + _tmp44;
  const Scalar _tmp81 = _tmp52 + _tmp80;
  const Scalar _tmp82 = _tmp42 * _tmp55;
  const Scalar _tmp83 = _tmp33 * _tmp57;
  const Scalar _tmp84 = _tmp35 * _tmp59;
  const Scalar _tmp85 = _tmp39 * _tmp49;
  const Scalar _tmp86 = Scalar(0.021999999999999999) * _tmp85;
  const Scalar _tmp87 = _tmp82 + _tmp83 + _tmp84 + _tmp86;
  const Scalar _tmp88 = Scalar(0.83999999999999997) * _tmp39;
  const Scalar _tmp89 = _tmp42 * _tmp88;
  const Scalar _tmp90 = _tmp30 + _tmp89;
  const Scalar _tmp91 = _tmp87 + _tmp90;
  const Scalar _tmp92 = Scalar(0.20999999999999999) * _tmp65 - Scalar(0.20999999999999999) * _tmp67;
  const Scalar _tmp93 = -_tmp92;
  const Scalar _tmp94 = 1 - 2 * std::pow(_tmp42, Scalar(2));
  const Scalar _tmp95 = Scalar(0.20999999999999999) * _tmp74 + Scalar(0.20999999999999999) * _tmp94;
  const Scalar _tmp96 = 2 * _tmp28 * _tmp33;
  const Scalar _tmp97 = _tmp42 * _tmp66;
  const Scalar _tmp98 = _tmp96 + _tmp97;
  const Scalar _tmp99 = -Scalar(0.010999999999999999) * _tmp98;
  const Scalar _tmp100 = _tmp95 + _tmp99;
  const Scalar _tmp101 = _tmp100 + _tmp93;
  const Scalar _tmp102 = _tmp101 + p_init0;
  const Scalar _tmp103 = Scalar(2.5202214700000001) - _tmp102;
  const Scalar _tmp104 = Scalar(1.0) / (fh1);
  const Scalar _tmp105 =
      std::sqrt(Scalar(std::pow(_tmp103, Scalar(2)) + std::pow(_tmp79, Scalar(2))));
  const Scalar _tmp106 = _tmp43 * _tmp46;
  const Scalar _tmp107 = _tmp23 * _tmp40;
  const Scalar _tmp108 = _tmp35 * _tmp47;
  const Scalar _tmp109 = Scalar(0.41999999999999998) * _tmp85;
  const Scalar _tmp110 = -_tmp106 + _tmp107 + _tmp108 - _tmp109;
  const Scalar _tmp111 = _tmp40 * _tmp46;
  const Scalar _tmp112 = -_tmp111;
  const Scalar _tmp113 = _tmp23 * _tmp43;
  const Scalar _tmp114 = -_tmp113;
  const Scalar _tmp115 = _tmp39 * _tmp47;
  const Scalar _tmp116 = -_tmp115;
  const Scalar _tmp117 = Scalar(0.41999999999999998) * _tmp61;
  const Scalar _tmp118 = -_tmp117;
  const Scalar _tmp119 = _tmp39 * _tmp42;
  const Scalar _tmp120 = Scalar(0.043999999999999997) * _tmp119;
  const Scalar _tmp121 = _tmp33 * _tmp35;
  const Scalar _tmp122 = Scalar(0.043999999999999997) * _tmp121;
  const Scalar _tmp123 = _tmp120 + _tmp122;
  const Scalar _tmp124 = _tmp112 + _tmp114 + _tmp116 + _tmp118 + _tmp123;
  const Scalar _tmp125 = _tmp110 + _tmp124;
  const Scalar _tmp126 = _tmp72 + _tmp75;
  const Scalar _tmp127 = _tmp126 + _tmp68;
  const Scalar _tmp128 = Scalar(1.0) * _tmp127;
  const Scalar _tmp129 = -_tmp128;
  const Scalar _tmp130 = -_tmp68;
  const Scalar _tmp131 = _tmp130 + _tmp76;
  const Scalar _tmp132 = _tmp129 + _tmp131;
  const Scalar _tmp133 = Scalar(1.0) / (_tmp132);
  const Scalar _tmp134 = -_tmp95 + _tmp99;
  const Scalar _tmp135 = _tmp134 + _tmp93;
  const Scalar _tmp136 = _tmp100 + _tmp92;
  const Scalar _tmp137 = Scalar(1.0) * _tmp136;
  const Scalar _tmp138 = -_tmp135 + _tmp137;
  const Scalar _tmp139 = _tmp133 * _tmp138;
  const Scalar _tmp140 = _tmp128 * _tmp139 + _tmp137;
  const Scalar _tmp141 = _tmp134 + _tmp92;
  const Scalar _tmp142 = _tmp126 + _tmp130;
  const Scalar _tmp143 = _tmp129 + _tmp142;
  const Scalar _tmp144 = _tmp133 * _tmp143;
  const Scalar _tmp145 = _tmp138 * _tmp144;
  const Scalar _tmp146 = _tmp137 - _tmp141 - _tmp145;
  const Scalar _tmp147 = Scalar(1.0) / (_tmp146);
  const Scalar _tmp148 = Scalar(1.0) * _tmp147;
  const Scalar _tmp149 = _tmp140 * _tmp147;
  const Scalar _tmp150 = 0;
  const Scalar _tmp151 =
      Scalar(0.20999999999999999) * _tmp69 + Scalar(0.20999999999999999) * _tmp70;
  const Scalar _tmp152 =
      -Scalar(0.010999999999999999) * _tmp73 - Scalar(0.010999999999999999) * _tmp94;
  const Scalar _tmp153 =
      Scalar(0.20999999999999999) * _tmp96 - Scalar(0.20999999999999999) * _tmp97;
  const Scalar _tmp154 = _tmp152 - _tmp153;
  const Scalar _tmp155 = _tmp151 + _tmp154;
  const Scalar _tmp156 = _tmp142 + p_init1;
  const Scalar _tmp157 = _tmp156 + Scalar(-4.8333311099999996);
  const Scalar _tmp158 = _tmp141 + p_init0;
  const Scalar _tmp159 = _tmp158 + Scalar(1.79662371);
  const Scalar _tmp160 = std::pow(_tmp157, Scalar(2)) + std::pow(_tmp159, Scalar(2));
  const Scalar _tmp161 = std::pow(_tmp160, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp162 = _tmp157 * _tmp161;
  const Scalar _tmp163 = _tmp136 + p_init0;
  const Scalar _tmp164 = _tmp163 + Scalar(-2.71799795);
  const Scalar _tmp165 = Scalar(1.0) / (_tmp164);
  const Scalar _tmp166 = _tmp127 + p_init1;
  const Scalar _tmp167 = _tmp166 + Scalar(-4.7752063900000001);
  const Scalar _tmp168 = _tmp165 * _tmp167;
  const Scalar _tmp169 = _tmp152 + _tmp153;
  const Scalar _tmp170 = _tmp151 + _tmp169;
  const Scalar _tmp171 = _tmp159 * _tmp161;
  const Scalar _tmp172 = _tmp170 * _tmp171;
  const Scalar _tmp173 = _tmp131 + p_init1;
  const Scalar _tmp174 = _tmp173 + Scalar(8.3196563700000006);
  const Scalar _tmp175 = _tmp135 + p_init0;
  const Scalar _tmp176 = _tmp175 + Scalar(1.9874742000000001);
  const Scalar _tmp177 = std::pow(_tmp174, Scalar(2)) + std::pow(_tmp176, Scalar(2));
  const Scalar _tmp178 = std::pow(_tmp177, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp179 = _tmp176 * _tmp178;
  const Scalar _tmp180 = _tmp170 * _tmp179;
  const Scalar _tmp181 = -_tmp151;
  const Scalar _tmp182 = _tmp154 + _tmp181;
  const Scalar _tmp183 = _tmp174 * _tmp178;
  const Scalar _tmp184 = -_tmp168 * _tmp180 + _tmp182 * _tmp183;
  const Scalar _tmp185 = _tmp168 * _tmp179 - _tmp183;
  const Scalar _tmp186 = Scalar(1.0) / (_tmp185);
  const Scalar _tmp187 = -_tmp162 + _tmp168 * _tmp171;
  const Scalar _tmp188 = _tmp186 * _tmp187;
  const Scalar _tmp189 = _tmp155 * _tmp162 - _tmp168 * _tmp172 - _tmp184 * _tmp188;
  const Scalar _tmp190 = -_tmp179 * _tmp182 + _tmp180;
  const Scalar _tmp191 = _tmp155 * _tmp161;
  const Scalar _tmp192 = -_tmp139 * _tmp189 - _tmp159 * _tmp191 + _tmp172 - _tmp188 * _tmp190;
  const Scalar _tmp193 = Scalar(1.0) / (_tmp192);
  const Scalar _tmp194 = _tmp189 * _tmp193;
  const Scalar _tmp195 = _tmp129 - _tmp143 * _tmp149 - _tmp150 * _tmp194;
  const Scalar _tmp196 = Scalar(1.0) * _tmp133;
  const Scalar _tmp197 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp198 = Scalar(1.0) * _tmp197;
  const Scalar _tmp199 = _tmp145 * _tmp148 + Scalar(1.0);
  const Scalar _tmp200 = _tmp133 * _tmp199;
  const Scalar _tmp201 = _tmp139 * _tmp148;
  const Scalar _tmp202 = -Scalar(1.0) * _tmp200 + _tmp201;
  const Scalar _tmp203 = _tmp169 + _tmp181;
  const Scalar _tmp204 = _tmp78 + Scalar(8.3888750099999996);
  const Scalar _tmp205 = _tmp102 + Scalar(-2.5202214700000001);
  const Scalar _tmp206 = std::pow(_tmp204, Scalar(2)) + std::pow(_tmp205, Scalar(2));
  const Scalar _tmp207 = std::pow(_tmp206, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp208 = _tmp207 * fh1;
  const Scalar _tmp209 = _tmp204 * _tmp208;
  const Scalar _tmp210 = -_tmp203 * _tmp209 - Scalar(3.29616) * _tmp71 - _tmp77 * fv1;
  const Scalar _tmp211 = Scalar(1.0) * _tmp210;
  const Scalar _tmp212 = _tmp205 * _tmp207;
  const Scalar _tmp213 = _tmp203 * fh1;
  const Scalar _tmp214 = _tmp101 * fv1 + _tmp212 * _tmp213 + Scalar(3.29616) * _tmp98;
  const Scalar _tmp215 = Scalar(1.0) * _tmp144 * _tmp148 - Scalar(1.0) * _tmp148;
  const Scalar _tmp216 = _tmp138 * _tmp196;
  const Scalar _tmp217 = _tmp184 * _tmp186;
  const Scalar _tmp218 = Scalar(1.0) * _tmp186;
  const Scalar _tmp219 = -_tmp190 * _tmp218 + _tmp216 * _tmp217;
  const Scalar _tmp220 = std::pow(_tmp164, Scalar(2));
  const Scalar _tmp221 = std::pow(_tmp167, Scalar(2)) + _tmp220;
  const Scalar _tmp222 = std::sqrt(_tmp221);
  const Scalar _tmp223 = Scalar(1.0) / (_tmp222);
  const Scalar _tmp224 = _tmp167 * _tmp223;
  const Scalar _tmp225 = _tmp127 * _tmp223;
  const Scalar _tmp226 = _tmp136 * _tmp224 - _tmp164 * _tmp225;
  const Scalar _tmp227 = _tmp165 * _tmp222;
  const Scalar _tmp228 = _tmp226 * _tmp227;
  const Scalar _tmp229 = _tmp131 * _tmp179 - _tmp135 * _tmp183 + _tmp179 * _tmp228;
  const Scalar _tmp230 =
      -_tmp141 * _tmp162 + _tmp142 * _tmp171 + _tmp171 * _tmp228 - _tmp188 * _tmp229;
  const Scalar _tmp231 = _tmp193 * _tmp230;
  const Scalar _tmp232 = -_tmp218 * _tmp229 - _tmp219 * _tmp231;
  const Scalar _tmp233 = Scalar(1.0) / (_tmp230);
  const Scalar _tmp234 = _tmp232 * _tmp233;
  const Scalar _tmp235 = _tmp192 * _tmp234;
  const Scalar _tmp236 = _tmp219 + _tmp235;
  const Scalar _tmp237 = _tmp143 * _tmp147;
  const Scalar _tmp238 = _tmp184 * _tmp218;
  const Scalar _tmp239 = -_tmp194 * _tmp236 + _tmp235 * _tmp237 - _tmp238;
  const Scalar _tmp240 = _tmp148 * _tmp235 - _tmp196 * _tmp239;
  const Scalar _tmp241 = Scalar(1.0) * _tmp209;
  const Scalar _tmp242 = _tmp192 * _tmp233;
  const Scalar _tmp243 = _tmp148 * _tmp242;
  const Scalar _tmp244 = Scalar(1.0) * _tmp233;
  const Scalar _tmp245 = _tmp143 * _tmp243 - _tmp189 * _tmp244;
  const Scalar _tmp246 = -Scalar(1.0) * _tmp196 * _tmp245 + Scalar(1.0) * _tmp243;
  const Scalar _tmp247 = _tmp101 * _tmp207;
  const Scalar _tmp248 = fh1 * (_tmp204 * _tmp247 - _tmp212 * _tmp77);
  const Scalar _tmp249 = _tmp212 * fh1;
  const Scalar _tmp250 = _tmp186 * _tmp229;
  const Scalar _tmp251 = _tmp168 * _tmp170;
  const Scalar _tmp252 = _tmp168 * _tmp186;
  const Scalar _tmp253 = _tmp184 * _tmp252 + _tmp251;
  const Scalar _tmp254 = -_tmp139 * _tmp253 - _tmp170 + _tmp190 * _tmp252;
  const Scalar _tmp255 = _tmp193 * _tmp254;
  const Scalar _tmp256 = _tmp168 * _tmp250 - _tmp228 - _tmp230 * _tmp255;
  const Scalar _tmp257 = _tmp233 * _tmp256;
  const Scalar _tmp258 = _tmp192 * _tmp257;
  const Scalar _tmp259 = _tmp254 + _tmp258;
  const Scalar _tmp260 = -_tmp194 * _tmp259 + _tmp237 * _tmp258 + _tmp253;
  const Scalar _tmp261 = _tmp148 * _tmp258 - _tmp196 * _tmp260;
  const Scalar _tmp262 = Scalar(1.0) * _tmp261;
  const Scalar _tmp263 = _tmp198 * (-_tmp140 * _tmp148 - _tmp195 * _tmp196 + Scalar(1.0)) +
                         _tmp202 * _tmp211 + _tmp214 * _tmp215 + _tmp240 * _tmp241 +
                         _tmp246 * _tmp248 + _tmp249 * _tmp262;
  const Scalar _tmp264 = _tmp193 * _tmp236;
  const Scalar _tmp265 = -_tmp187 * _tmp264 + Scalar(1.0);
  const Scalar _tmp266 = _tmp179 * _tmp186;
  const Scalar _tmp267 = _tmp171 * _tmp264 + _tmp265 * _tmp266;
  const Scalar _tmp268 = _tmp227 * _tmp267;
  const Scalar _tmp269 = _tmp193 * _tmp259;
  const Scalar _tmp270 = _tmp187 * _tmp193;
  const Scalar _tmp271 = -_tmp168 - _tmp259 * _tmp270;
  const Scalar _tmp272 = _tmp171 * _tmp269 + _tmp266 * _tmp271 + Scalar(1.0);
  const Scalar _tmp273 = _tmp227 * _tmp272;
  const Scalar _tmp274 = _tmp150 * _tmp193;
  const Scalar _tmp275 = _tmp188 * _tmp274;
  const Scalar _tmp276 = _tmp171 * _tmp274 - _tmp179 * _tmp275;
  const Scalar _tmp277 = _tmp197 * _tmp227;
  const Scalar _tmp278 = _tmp188 * _tmp244;
  const Scalar _tmp279 = _tmp171 * _tmp244 - _tmp179 * _tmp278;
  const Scalar _tmp280 = _tmp227 * _tmp248;
  const Scalar _tmp281 =
      -_tmp209 * _tmp268 - _tmp249 * _tmp273 - _tmp276 * _tmp277 - _tmp279 * _tmp280;
  const Scalar _tmp282 = Scalar(1.0) / (_tmp281);
  const Scalar _tmp283 = std::asinh(_tmp263 * _tmp282);
  const Scalar _tmp284 = Scalar(1.0) * _tmp283;
  const Scalar _tmp285 = Scalar(4.7752063900000001) - _tmp166;
  const Scalar _tmp286 = Scalar(2.71799795) - _tmp163;
  const Scalar _tmp287 =
      std::sqrt(Scalar(std::pow(_tmp285, Scalar(2)) + std::pow(_tmp286, Scalar(2))));
  const Scalar _tmp288 = Scalar(9.6622558468725703) * _tmp281;
  const Scalar _tmp289 = -_tmp283 * _tmp288 - _tmp287;
  const Scalar _tmp290 = Scalar(0.1034955) * _tmp282;
  const Scalar _tmp291 = _tmp289 * _tmp290;
  const Scalar _tmp292 = -_tmp82 - _tmp83 - _tmp84 - _tmp86;
  const Scalar _tmp293 = _tmp292 + _tmp90;
  const Scalar _tmp294 = _tmp293 + _tmp81;
  const Scalar _tmp295 = _tmp56 - _tmp58 - _tmp60 + _tmp62;
  const Scalar _tmp296 = _tmp295 + _tmp54;
  const Scalar _tmp297 =
      (2 * _tmp157 * _tmp296 + 2 * _tmp159 * _tmp294) / (_tmp160 * std::sqrt(_tmp160));
  const Scalar _tmp298 = (Scalar(1) / Scalar(2)) * _tmp297;
  const Scalar _tmp299 = _tmp159 * _tmp298;
  const Scalar _tmp300 = _tmp31 - _tmp89;
  const Scalar _tmp301 = _tmp300 + _tmp81;
  const Scalar _tmp302 = _tmp292 + _tmp301;
  const Scalar _tmp303 = _tmp302 / _tmp220;
  const Scalar _tmp304 = _tmp167 * _tmp303;
  const Scalar _tmp305 = _tmp157 * _tmp298;
  const Scalar _tmp306 = _tmp161 * _tmp294;
  const Scalar _tmp307 = _tmp48 + _tmp51;
  const Scalar _tmp308 = _tmp307 + _tmp80;
  const Scalar _tmp309 = _tmp308 + _tmp38;
  const Scalar _tmp310 = _tmp295 + _tmp309;
  const Scalar _tmp311 = _tmp165 * _tmp310;
  const Scalar _tmp312 = _tmp161 * _tmp296;
  const Scalar _tmp313 = -_tmp168 * _tmp299 + _tmp168 * _tmp306 - _tmp171 * _tmp304 +
                         _tmp171 * _tmp311 + _tmp305 - _tmp312;
  const Scalar _tmp314 = _tmp186 * _tmp313;
  const Scalar _tmp315 = _tmp111 + _tmp113 + _tmp115 + _tmp117 + _tmp123;
  const Scalar _tmp316 = _tmp110 + _tmp315;
  const Scalar _tmp317 = _tmp171 * _tmp316;
  const Scalar _tmp318 = _tmp179 * _tmp316;
  const Scalar _tmp319 = _tmp307 + _tmp45;
  const Scalar _tmp320 = _tmp293 + _tmp319;
  const Scalar _tmp321 = _tmp30 + _tmp37;
  const Scalar _tmp322 = _tmp321 + _tmp53;
  const Scalar _tmp323 = _tmp295 + _tmp322;
  const Scalar _tmp324 =
      (2 * _tmp174 * _tmp323 + 2 * _tmp176 * _tmp320) / (_tmp177 * std::sqrt(_tmp177));
  const Scalar _tmp325 = (Scalar(1) / Scalar(2)) * _tmp324;
  const Scalar _tmp326 = _tmp176 * _tmp325;
  const Scalar _tmp327 = _tmp178 * _tmp320;
  const Scalar _tmp328 = _tmp170 * _tmp327;
  const Scalar _tmp329 = _tmp106 - _tmp107 - _tmp108 + _tmp109;
  const Scalar _tmp330 = _tmp124 + _tmp329;
  const Scalar _tmp331 = _tmp174 * _tmp325;
  const Scalar _tmp332 = _tmp178 * _tmp323;
  const Scalar _tmp333 = -_tmp168 * _tmp318 - _tmp168 * _tmp328 + _tmp180 * _tmp304 -
                         _tmp180 * _tmp311 - _tmp182 * _tmp331 + _tmp182 * _tmp332 +
                         _tmp183 * _tmp330 + _tmp251 * _tmp326;
  const Scalar _tmp334 = _tmp315 + _tmp329;
  const Scalar _tmp335 = (-_tmp168 * _tmp326 + _tmp168 * _tmp327 - _tmp179 * _tmp304 +
                          _tmp179 * _tmp311 + _tmp331 - _tmp332) /
                         std::pow(_tmp185, Scalar(2));
  const Scalar _tmp336 = _tmp184 * _tmp335;
  const Scalar _tmp337 = _tmp170 * _tmp311;
  const Scalar _tmp338 = -_tmp155 * _tmp305 + _tmp155 * _tmp312 + _tmp162 * _tmp334 -
                         _tmp168 * _tmp317 - _tmp171 * _tmp337 + _tmp172 * _tmp304 -
                         _tmp184 * _tmp314 + _tmp187 * _tmp336 - _tmp188 * _tmp333 +
                         _tmp251 * _tmp299 - _tmp251 * _tmp306;
  const Scalar _tmp339 = _tmp190 * _tmp335;
  const Scalar _tmp340 = _tmp33 * _tmp88;
  const Scalar _tmp341 = _tmp36 * _tmp42;
  const Scalar _tmp342 = -_tmp29 * _tmp46 - Scalar(0.83999999999999997) * _tmp50;
  const Scalar _tmp343 = Scalar(1.6799999999999999) * _tmp23 * _tmp28;
  const Scalar _tmp344 = -Scalar(1.6799999999999999) * _tmp119 - _tmp343;
  const Scalar _tmp345 = _tmp340 + _tmp341 + _tmp342 + _tmp344;
  const Scalar _tmp346 = _tmp133 * _tmp345;
  const Scalar _tmp347 = -_tmp170 * _tmp326 - _tmp179 * _tmp330 + _tmp182 * _tmp326 -
                         _tmp182 * _tmp327 + _tmp318 + _tmp328;
  const Scalar _tmp348 = -_tmp340 - _tmp341 + _tmp342;
  const Scalar _tmp349 =
      (Scalar(1.6799999999999999) * _tmp121 + _tmp343 + _tmp348) / std::pow(_tmp132, Scalar(2));
  const Scalar _tmp350 = _tmp138 * _tmp349;
  const Scalar _tmp351 = -_tmp139 * _tmp338 + _tmp155 * _tmp299 - _tmp170 * _tmp299 +
                         _tmp170 * _tmp306 - _tmp171 * _tmp334 + _tmp187 * _tmp339 -
                         _tmp188 * _tmp347 - _tmp189 * _tmp346 + _tmp189 * _tmp350 -
                         _tmp190 * _tmp314 - _tmp191 * _tmp294 + _tmp317;
  const Scalar _tmp352 = _tmp351 / std::pow(_tmp192, Scalar(2));
  const Scalar _tmp353 = _tmp150 * _tmp352;
  const Scalar _tmp354 = _tmp179 * _tmp188;
  const Scalar _tmp355 = _tmp179 * _tmp335;
  const Scalar _tmp356 = _tmp193 * _tmp313;
  const Scalar _tmp357 = _tmp164 * _tmp302 + _tmp167 * _tmp310;
  const Scalar _tmp358 = _tmp165 * _tmp223 * _tmp357;
  const Scalar _tmp359 = _tmp248 * _tmp279;
  const Scalar _tmp360 = _tmp300 + _tmp319;
  const Scalar _tmp361 = _tmp292 + _tmp360;
  const Scalar _tmp362 = _tmp207 * _tmp361;
  const Scalar _tmp363 = _tmp362 * fh1;
  const Scalar _tmp364 = _tmp222 * _tmp303;
  const Scalar _tmp365 = _tmp249 * _tmp272;
  const Scalar _tmp366 = _tmp197 * _tmp276;
  const Scalar _tmp367 = _tmp171 * _tmp352;
  const Scalar _tmp368 = _tmp186 * _tmp327;
  const Scalar _tmp369 = _tmp186 * _tmp271;
  const Scalar _tmp370 = _tmp226 * _tmp358;
  const Scalar _tmp371 = _tmp229 * _tmp335;
  const Scalar _tmp372 = _tmp226 * _tmp364;
  const Scalar _tmp373 = _tmp357 / (_tmp221 * std::sqrt(_tmp221));
  const Scalar _tmp374 = _tmp223 * _tmp310;
  const Scalar _tmp375 =
      _tmp227 * (_tmp127 * _tmp164 * _tmp373 - _tmp136 * _tmp167 * _tmp373 + _tmp136 * _tmp374 -
                 _tmp164 * _tmp374 + _tmp224 * _tmp302 - _tmp225 * _tmp302);
  const Scalar _tmp376 = -_tmp131 * _tmp326 + _tmp131 * _tmp327 + _tmp135 * _tmp331 -
                         _tmp135 * _tmp332 + _tmp176 * _tmp332 + _tmp179 * _tmp370 -
                         _tmp179 * _tmp372 + _tmp179 * _tmp375 - _tmp183 * _tmp320 -
                         _tmp228 * _tmp326 + _tmp228 * _tmp327;
  const Scalar _tmp377 = _tmp141 * _tmp305 - _tmp141 * _tmp312 - _tmp142 * _tmp299 +
                         _tmp142 * _tmp306 + _tmp159 * _tmp312 - _tmp162 * _tmp294 +
                         _tmp171 * _tmp370 - _tmp171 * _tmp372 + _tmp171 * _tmp375 +
                         _tmp187 * _tmp371 - _tmp188 * _tmp376 - _tmp228 * _tmp299 +
                         _tmp228 * _tmp306 - _tmp229 * _tmp314;
  const Scalar _tmp378 = _tmp377 / std::pow(_tmp230, Scalar(2));
  const Scalar _tmp379 = _tmp192 * _tmp378;
  const Scalar _tmp380 = _tmp256 * _tmp379;
  const Scalar _tmp381 = _tmp257 * _tmp351;
  const Scalar _tmp382 = _tmp186 * _tmp311;
  const Scalar _tmp383 = _tmp168 * _tmp316 - _tmp168 * _tmp336 - _tmp170 * _tmp304 +
                         _tmp184 * _tmp382 - _tmp217 * _tmp304 + _tmp252 * _tmp333 + _tmp337;
  const Scalar _tmp384 = _tmp112 + _tmp114 + _tmp116 + _tmp118 - _tmp120 - _tmp122 -
                         _tmp139 * _tmp383 - _tmp168 * _tmp339 - _tmp186 * _tmp190 * _tmp304 +
                         _tmp190 * _tmp382 + _tmp252 * _tmp347 - _tmp253 * _tmp346 +
                         _tmp253 * _tmp350 + _tmp329;
  const Scalar _tmp385 = _tmp230 * _tmp352;
  const Scalar _tmp386 = _tmp242 * (-_tmp168 * _tmp371 - _tmp231 * _tmp384 - _tmp250 * _tmp304 +
                                    _tmp250 * _tmp311 + _tmp252 * _tmp376 + _tmp254 * _tmp385 -
                                    _tmp255 * _tmp377 - _tmp370 + _tmp372 - _tmp375);
  const Scalar _tmp387 = -_tmp380 + _tmp381 + _tmp384 + _tmp386;
  const Scalar _tmp388 = _tmp187 * _tmp352;
  const Scalar _tmp389 =
      -_tmp259 * _tmp356 + _tmp259 * _tmp388 - _tmp270 * _tmp387 + _tmp304 - _tmp311;
  const Scalar _tmp390 = _tmp171 * _tmp193;
  const Scalar _tmp391 = _tmp308 + _tmp321;
  const Scalar _tmp392 = _tmp295 + _tmp391;
  const Scalar _tmp393 =
      (2 * _tmp204 * _tmp392 + 2 * _tmp205 * _tmp361) / (_tmp206 * std::sqrt(_tmp206));
  const Scalar _tmp394 = (Scalar(1) / Scalar(2)) * _tmp393;
  const Scalar _tmp395 = _tmp205 * _tmp394;
  const Scalar _tmp396 = _tmp395 * fh1;
  const Scalar _tmp397 = Scalar(0.5) * _tmp233;
  const Scalar _tmp398 = Scalar(1.0) * _tmp378;
  const Scalar _tmp399 = _tmp204 * _tmp394;
  const Scalar _tmp400 = fh1 * (-_tmp101 * _tmp399 + _tmp204 * _tmp362 - _tmp212 * _tmp392 +
                                _tmp247 * _tmp392 - _tmp362 * _tmp77 + _tmp395 * _tmp77);
  const Scalar _tmp401 = _tmp208 * _tmp392;
  const Scalar _tmp402 = _tmp232 * _tmp379;
  const Scalar _tmp403 = _tmp234 * _tmp351;
  const Scalar _tmp404 = _tmp186 * _tmp216 * _tmp333 + _tmp196 * _tmp217 * _tmp345 -
                         _tmp216 * _tmp336 - _tmp218 * _tmp347 - _tmp238 * _tmp350 +
                         Scalar(1.0) * _tmp339;
  const Scalar _tmp405 = _tmp242 * (-_tmp193 * _tmp219 * _tmp377 - _tmp218 * _tmp376 +
                                    _tmp219 * _tmp385 - _tmp231 * _tmp404 + Scalar(1.0) * _tmp371);
  const Scalar _tmp406 = -_tmp402 + _tmp403 + _tmp404 + _tmp405;
  const Scalar _tmp407 = _tmp186 * _tmp265;
  const Scalar _tmp408 = -_tmp236 * _tmp356 + _tmp236 * _tmp388 - _tmp270 * _tmp406;
  const Scalar _tmp409 = _tmp209 * _tmp267;
  const Scalar _tmp410 = _tmp399 * fh1;
  const Scalar _tmp411 =
      -_tmp209 * _tmp227 *
          (-_tmp236 * _tmp367 - _tmp264 * _tmp299 + _tmp264 * _tmp306 - _tmp265 * _tmp355 +
           _tmp265 * _tmp368 + _tmp266 * _tmp408 - _tmp326 * _tmp407 + _tmp390 * _tmp406) -
      _tmp227 * _tmp249 *
          (-_tmp259 * _tmp367 + _tmp266 * _tmp389 - _tmp269 * _tmp299 + _tmp269 * _tmp306 -
           _tmp271 * _tmp355 + _tmp271 * _tmp368 - _tmp326 * _tmp369 + _tmp387 * _tmp390) -
      _tmp227 * _tmp279 * _tmp400 - _tmp268 * _tmp401 + _tmp268 * _tmp410 - _tmp273 * _tmp363 +
      _tmp273 * _tmp396 -
      _tmp277 * (-_tmp150 * _tmp266 * _tmp356 + _tmp150 * _tmp270 * _tmp355 - _tmp171 * _tmp353 -
                 _tmp274 * _tmp299 + _tmp274 * _tmp306 + _tmp275 * _tmp326 - _tmp275 * _tmp327 +
                 _tmp353 * _tmp354) -
      _tmp280 * (-_tmp159 * _tmp297 * _tmp397 - _tmp171 * _tmp398 +
                 _tmp176 * _tmp188 * _tmp324 * _tmp397 - _tmp179 * _tmp244 * _tmp314 +
                 _tmp187 * _tmp244 * _tmp355 + _tmp244 * _tmp306 - _tmp278 * _tmp327 +
                 _tmp354 * _tmp398) -
      _tmp358 * _tmp359 - _tmp358 * _tmp365 - _tmp358 * _tmp366 - _tmp358 * _tmp409 +
      _tmp359 * _tmp364 + _tmp364 * _tmp365 + _tmp364 * _tmp366 + _tmp364 * _tmp409;
  const Scalar _tmp412 = Scalar(9.6622558468725703) * _tmp411;
  const Scalar _tmp413 = std::pow(_tmp281, Scalar(-2));
  const Scalar _tmp414 = Scalar(6.59232) * _tmp42;
  const Scalar _tmp415 = Scalar(6.59232) * _tmp33;
  const Scalar _tmp416 = Scalar(6.59232) * _tmp28;
  const Scalar _tmp417 = Scalar(6.59232) * _tmp49;
  const Scalar _tmp418 = _tmp125 * _tmp249 + _tmp213 * _tmp362 - _tmp213 * _tmp395 +
                         _tmp23 * _tmp415 + _tmp35 * _tmp416 + _tmp361 * fv1 + _tmp39 * _tmp417 +
                         _tmp414 * _tmp46;
  const Scalar _tmp419 = _tmp143 * _tmp350;
  const Scalar _tmp420 = _tmp144 * _tmp345;
  const Scalar _tmp421 = _tmp139 * _tmp348;
  const Scalar _tmp422 = (_tmp344 + _tmp419 - _tmp420 - _tmp421) / std::pow(_tmp146, Scalar(2));
  const Scalar _tmp423 = Scalar(1.0) * _tmp422;
  const Scalar _tmp424 = _tmp139 * _tmp423;
  const Scalar _tmp425 =
      -_tmp145 * _tmp423 - _tmp148 * _tmp419 + _tmp148 * _tmp420 + _tmp148 * _tmp421;
  const Scalar _tmp426 = _tmp148 * _tmp346;
  const Scalar _tmp427 = _tmp148 * _tmp350;
  const Scalar _tmp428 = Scalar(1.0) * _tmp349;
  const Scalar _tmp429 = _tmp189 * _tmp352;
  const Scalar _tmp430 = _tmp147 * _tmp348;
  const Scalar _tmp431 = _tmp143 * _tmp422;
  const Scalar _tmp432 = -_tmp194 * _tmp406 - _tmp218 * _tmp333 + _tmp235 * _tmp430 -
                         _tmp235 * _tmp431 + _tmp236 * _tmp429 - _tmp237 * _tmp402 +
                         _tmp237 * _tmp403 + _tmp237 * _tmp405 - _tmp264 * _tmp338 +
                         Scalar(1.0) * _tmp336;
  const Scalar _tmp433 = Scalar(0.5) * _tmp393 * fh1;
  const Scalar _tmp434 = -_tmp194 * _tmp387 - _tmp237 * _tmp380 + _tmp237 * _tmp381 +
                         _tmp237 * _tmp386 + _tmp258 * _tmp430 - _tmp258 * _tmp431 +
                         _tmp259 * _tmp429 - _tmp269 * _tmp338 + _tmp383;
  const Scalar _tmp435 = _tmp128 * _tmp346 - _tmp128 * _tmp350 + _tmp216 * _tmp310 + _tmp302;
  const Scalar _tmp436 = _tmp147 * _tmp435;
  const Scalar _tmp437 = _tmp140 * _tmp422;
  const Scalar _tmp438 = _tmp322 + _tmp63;
  const Scalar _tmp439 = _tmp133 * (-_tmp143 * _tmp436 + _tmp143 * _tmp437 - _tmp149 * _tmp348 +
                                    _tmp150 * _tmp429 - _tmp274 * _tmp338 + _tmp438);
  const Scalar _tmp440 = _tmp133 * _tmp348;
  const Scalar _tmp441 = _tmp143 * _tmp349;
  const Scalar _tmp442 = _tmp148 * _tmp379;
  const Scalar _tmp443 = _tmp148 * _tmp233 * _tmp351;
  const Scalar _tmp444 = _tmp242 * _tmp423;
  const Scalar _tmp445 = -_tmp143 * _tmp442 + _tmp143 * _tmp443 - _tmp143 * _tmp444 +
                         _tmp189 * _tmp398 + _tmp243 * _tmp348 - _tmp244 * _tmp338;
  const Scalar _tmp446 = -_tmp125 * _tmp209 - _tmp203 * _tmp401 + _tmp213 * _tmp399 -
                         _tmp23 * _tmp414 + _tmp35 * _tmp417 - _tmp39 * _tmp416 - _tmp392 * fv1 +
                         _tmp415 * _tmp46;
  const Scalar _tmp447 = _tmp411 * _tmp413;
  const Scalar _tmp448 =
      (-_tmp263 * _tmp447 +
       _tmp282 * (_tmp198 * (_tmp140 * _tmp423 - _tmp148 * _tmp435 + _tmp195 * _tmp428 -
                             Scalar(1.0) * _tmp439) +
                  Scalar(1.0) * _tmp202 * _tmp446 - _tmp204 * _tmp240 * _tmp433 -
                  _tmp205 * _tmp261 * _tmp433 +
                  _tmp211 * (-_tmp196 * _tmp425 + _tmp199 * _tmp428 - _tmp424 + _tmp426 - _tmp427) +
                  Scalar(1.0) * _tmp214 *
                      (-_tmp144 * _tmp423 + _tmp148 * _tmp440 - _tmp148 * _tmp441 + _tmp423) +
                  _tmp215 * _tmp418 + Scalar(1.0) * _tmp240 * _tmp401 +
                  _tmp241 * (-_tmp148 * _tmp402 + _tmp148 * _tmp403 + _tmp148 * _tmp405 -
                             _tmp196 * _tmp432 - _tmp235 * _tmp423 + _tmp239 * _tmp428) +
                  _tmp246 * _tmp400 +
                  Scalar(1.0) * _tmp248 *
                      (-_tmp196 * _tmp445 + _tmp245 * _tmp428 - _tmp442 + _tmp443 - _tmp444) +
                  Scalar(1.0) * _tmp249 *
                      (-_tmp148 * _tmp380 + _tmp148 * _tmp381 + _tmp148 * _tmp386 -
                       _tmp196 * _tmp434 - _tmp258 * _tmp423 + _tmp260 * _tmp428) +
                  _tmp262 * _tmp363)) /
      std::sqrt(Scalar(std::pow(_tmp263, Scalar(2)) * _tmp413 + 1));
  const Scalar _tmp449 = _tmp197 * _tmp274;
  const Scalar _tmp450 = _tmp186 * _tmp249;
  const Scalar _tmp451 = _tmp244 * _tmp248;
  const Scalar _tmp452 =
      -_tmp188 * _tmp449 - _tmp188 * _tmp451 + _tmp209 * _tmp407 + _tmp271 * _tmp450;
  const Scalar _tmp453 = Scalar(1.0) / (_tmp452);
  const Scalar _tmp454 = _tmp133 * _tmp249;
  const Scalar _tmp455 = _tmp195 * _tmp197;
  const Scalar _tmp456 = _tmp133 * _tmp245;
  const Scalar _tmp457 = _tmp148 * _tmp214;
  const Scalar _tmp458 = _tmp133 * _tmp239;
  const Scalar _tmp459 = _tmp133 * _tmp455 - _tmp144 * _tmp457 + _tmp200 * _tmp210 +
                         _tmp209 * _tmp458 + _tmp248 * _tmp456 + _tmp260 * _tmp454;
  const Scalar _tmp460 = std::asinh(_tmp453 * _tmp459);
  const Scalar _tmp461 = Scalar(1.0) * _tmp460;
  const Scalar _tmp462 = std::pow(_tmp452, Scalar(-2));
  const Scalar _tmp463 = _tmp150 * _tmp197;
  const Scalar _tmp464 = _tmp197 * _tmp353;
  const Scalar _tmp465 = _tmp248 * _tmp398;
  const Scalar _tmp466 = _tmp244 * _tmp400;
  const Scalar _tmp467 = _tmp186 * _tmp209 * _tmp408 - _tmp186 * _tmp356 * _tmp463 +
                         _tmp187 * _tmp335 * _tmp451 + _tmp188 * _tmp464 + _tmp188 * _tmp465 -
                         _tmp188 * _tmp466 - _tmp209 * _tmp265 * _tmp335 -
                         _tmp249 * _tmp271 * _tmp335 + _tmp270 * _tmp335 * _tmp463 -
                         _tmp314 * _tmp451 + _tmp363 * _tmp369 - _tmp369 * _tmp396 +
                         _tmp389 * _tmp450 + _tmp401 * _tmp407 - _tmp407 * _tmp410;
  const Scalar _tmp468 = _tmp462 * _tmp467;
  const Scalar _tmp469 = _tmp133 * _tmp260;
  const Scalar _tmp470 = _tmp214 * _tmp423;
  const Scalar _tmp471 = _tmp148 * _tmp418;
  const Scalar _tmp472 =
      (_tmp453 * (_tmp133 * _tmp209 * _tmp432 + _tmp133 * _tmp210 * _tmp425 +
                  _tmp133 * _tmp248 * _tmp445 + _tmp144 * _tmp470 - _tmp144 * _tmp471 +
                  _tmp197 * _tmp439 - _tmp199 * _tmp210 * _tmp349 + _tmp200 * _tmp446 -
                  _tmp209 * _tmp239 * _tmp349 - _tmp245 * _tmp248 * _tmp349 -
                  _tmp249 * _tmp260 * _tmp349 - _tmp349 * _tmp455 + _tmp363 * _tmp469 -
                  _tmp396 * _tmp469 + _tmp400 * _tmp456 + _tmp401 * _tmp458 - _tmp410 * _tmp458 +
                  _tmp434 * _tmp454 - _tmp440 * _tmp457 + _tmp441 * _tmp457) -
       _tmp459 * _tmp468) /
      std::sqrt(Scalar(std::pow(_tmp459, Scalar(2)) * _tmp462 + 1));
  const Scalar _tmp473 = Scalar(9.6622558468725703) * _tmp452;
  const Scalar _tmp474 = Scalar(9.6622558468725703) * _tmp467;
  const Scalar _tmp475 = -_tmp175 + Scalar(-1.9874742000000001);
  const Scalar _tmp476 = -_tmp173 + Scalar(-8.3196563700000006);
  const Scalar _tmp477 =
      std::sqrt(Scalar(std::pow(_tmp475, Scalar(2)) + std::pow(_tmp476, Scalar(2))));
  const Scalar _tmp478 = Scalar(0.1034955) * _tmp453;
  const Scalar _tmp479 = -_tmp460 * _tmp473 - _tmp477;
  const Scalar _tmp480 = _tmp478 * _tmp479;
  const Scalar _tmp481 = _tmp209 * _tmp264 + _tmp249 * _tmp269 + _tmp449 + _tmp451;
  const Scalar _tmp482 = std::pow(_tmp481, Scalar(-2));
  const Scalar _tmp483 = _tmp193 * _tmp209 * _tmp406 + _tmp193 * _tmp249 * _tmp387 -
                         _tmp209 * _tmp236 * _tmp352 - _tmp249 * _tmp259 * _tmp352 +
                         _tmp264 * _tmp401 - _tmp264 * _tmp410 + _tmp269 * _tmp363 -
                         _tmp269 * _tmp396 - _tmp464 - _tmp465 + _tmp466;
  const Scalar _tmp484 = _tmp482 * _tmp483;
  const Scalar _tmp485 = Scalar(1.0) / (_tmp481);
  const Scalar _tmp486 = _tmp147 * _tmp209;
  const Scalar _tmp487 = _tmp147 * _tmp249;
  const Scalar _tmp488 = _tmp149 * _tmp197 - _tmp201 * _tmp210 - _tmp235 * _tmp486 -
                         _tmp243 * _tmp248 - _tmp258 * _tmp487 + _tmp457;
  const Scalar _tmp489 = std::asinh(_tmp485 * _tmp488);
  const Scalar _tmp490 = Scalar(1.0) * _tmp489;
  const Scalar _tmp491 = _tmp147 * _tmp235;
  const Scalar _tmp492 = _tmp147 * _tmp258;
  const Scalar _tmp493 =
      (-_tmp484 * _tmp488 +
       _tmp485 * (_tmp197 * _tmp436 - _tmp197 * _tmp437 - _tmp201 * _tmp446 +
                  _tmp209 * _tmp235 * _tmp422 + _tmp210 * _tmp424 - _tmp210 * _tmp426 +
                  _tmp210 * _tmp427 - _tmp243 * _tmp400 + _tmp248 * _tmp442 - _tmp248 * _tmp443 +
                  _tmp248 * _tmp444 + _tmp249 * _tmp258 * _tmp422 - _tmp363 * _tmp492 +
                  _tmp380 * _tmp487 - _tmp381 * _tmp487 - _tmp386 * _tmp487 + _tmp396 * _tmp492 -
                  _tmp401 * _tmp491 + _tmp402 * _tmp486 - _tmp403 * _tmp486 - _tmp405 * _tmp486 +
                  _tmp410 * _tmp491 - _tmp470 + _tmp471)) /
      std::sqrt(Scalar(_tmp482 * std::pow(_tmp488, Scalar(2)) + 1));
  const Scalar _tmp494 = Scalar(9.6622558468725703) * _tmp481;
  const Scalar _tmp495 = Scalar(4.8333311099999996) - _tmp156;
  const Scalar _tmp496 = -_tmp158 + Scalar(-1.79662371);
  const Scalar _tmp497 =
      std::sqrt(Scalar(std::pow(_tmp495, Scalar(2)) + std::pow(_tmp496, Scalar(2))));
  const Scalar _tmp498 = -_tmp489 * _tmp494 - _tmp497;
  const Scalar _tmp499 = Scalar(0.1034955) * _tmp485;
  const Scalar _tmp500 = _tmp498 * _tmp499;
  const Scalar _tmp501 = Scalar(9.6622558468725703) * _tmp483;

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) =
      _tmp125 -
      Scalar(0.5) * (2 * _tmp103 * (_tmp81 + _tmp91) + 2 * _tmp79 * (_tmp54 + _tmp63)) *
          std::sinh(Scalar(0.1034955) * _tmp104 *
                    (-_tmp105 - Scalar(9.6622558468725703) * fh1 * std::asinh(_tmp104 * fv1))) /
          _tmp105;
  _res(1, 0) =
      -_tmp288 *
          (-Scalar(0.86565325453551001) * _tmp447 + Scalar(1.0) * _tmp448 * std::sinh(_tmp284) -
           (-Scalar(0.1034955) * _tmp289 * _tmp447 +
            _tmp290 * (-_tmp283 * _tmp412 - _tmp288 * _tmp448 -
                       Scalar(1) / Scalar(2) *
                           (2 * _tmp285 * _tmp438 + 2 * _tmp286 * (_tmp319 + _tmp91)) / _tmp287)) *
               std::sinh(_tmp291)) +
      _tmp316 -
      _tmp412 * (Scalar(0.86565325453551001) * _tmp282 + std::cosh(_tmp284) - std::cosh(_tmp291));
  _res(2, 0) =
      _tmp330 -
      _tmp473 *
          (-Scalar(0.87679799772039002) * _tmp468 + Scalar(1.0) * _tmp472 * std::sinh(_tmp461) -
           (-Scalar(0.1034955) * _tmp468 * _tmp479 +
            _tmp478 * (-_tmp460 * _tmp474 - _tmp472 * _tmp473 -
                       Scalar(1) / Scalar(2) *
                           (2 * _tmp475 * (_tmp301 + _tmp87) + 2 * _tmp476 * (_tmp309 + _tmp63)) /
                           _tmp477)) *
               std::sinh(_tmp480)) -
      _tmp474 * (Scalar(0.87679799772039002) * _tmp453 + std::cosh(_tmp461) - std::cosh(_tmp480));
  _res(3, 0) =
      _tmp334 -
      _tmp494 *
          (-Scalar(0.86625939559540499) * _tmp484 + Scalar(1.0) * _tmp493 * std::sinh(_tmp490) -
           (-Scalar(0.1034955) * _tmp484 * _tmp498 +
            _tmp499 * (-_tmp489 * _tmp501 - _tmp493 * _tmp494 -
                       Scalar(1) / Scalar(2) *
                           (2 * _tmp495 * (_tmp391 + _tmp63) + 2 * _tmp496 * (_tmp360 + _tmp87)) /
                           _tmp497)) *
               std::sinh(_tmp500)) -
      _tmp501 * (Scalar(0.86625939559540499) * _tmp485 + std::cosh(_tmp490) - std::cosh(_tmp500));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
