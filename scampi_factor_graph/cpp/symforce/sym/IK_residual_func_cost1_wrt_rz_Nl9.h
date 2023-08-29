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
 * Symbolic function: IK_residual_func_cost1_wrt_rz_Nl9
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost1WrtRzNl9(
    const Scalar fh1, const Scalar fv1, const Scalar rx, const Scalar ry, const Scalar rz,
    const Scalar p_init0, const Scalar p_init1, const Scalar p_init2, const Scalar rot_init_x,
    const Scalar rot_init_y, const Scalar rot_init_z, const Scalar rot_init_w,
    const Scalar epsilon) {
  // Total ops: 1626

  // Unused inputs
  (void)p_init2;
  (void)epsilon;

  // Input arrays

  // Intermediate terms (504)
  const Scalar _tmp0 = Scalar(1.0) / (fh1);
  const Scalar _tmp1 = std::pow(rz, Scalar(2));
  const Scalar _tmp2 = _tmp1 + std::pow(rx, Scalar(2)) + std::pow(ry, Scalar(2));
  const Scalar _tmp3 = std::sqrt(_tmp2);
  const Scalar _tmp4 = (Scalar(1) / Scalar(2)) * _tmp3;
  const Scalar _tmp5 = std::cos(_tmp4);
  const Scalar _tmp6 = _tmp5 * rot_init_x;
  const Scalar _tmp7 = std::sin(_tmp4);
  const Scalar _tmp8 = _tmp7 / _tmp3;
  const Scalar _tmp9 = _tmp8 * rot_init_z;
  const Scalar _tmp10 = _tmp8 * rot_init_w;
  const Scalar _tmp11 = _tmp8 * rot_init_y;
  const Scalar _tmp12 = _tmp11 * rz;
  const Scalar _tmp13 = _tmp10 * rx + _tmp12 + _tmp6 - _tmp9 * ry;
  const Scalar _tmp14 = -2 * std::pow(_tmp13, Scalar(2));
  const Scalar _tmp15 = _tmp5 * rot_init_z;
  const Scalar _tmp16 = _tmp8 * rot_init_x;
  const Scalar _tmp17 = _tmp10 * rz;
  const Scalar _tmp18 = -_tmp11 * rx + _tmp15 + _tmp16 * ry + _tmp17;
  const Scalar _tmp19 = -2 * std::pow(_tmp18, Scalar(2));
  const Scalar _tmp20 = Scalar(0.20999999999999999) * _tmp14 +
                        Scalar(0.20999999999999999) * _tmp19 + Scalar(0.20999999999999999);
  const Scalar _tmp21 = -_tmp20;
  const Scalar _tmp22 = _tmp5 * rot_init_y;
  const Scalar _tmp23 = _tmp16 * rz;
  const Scalar _tmp24 = _tmp10 * ry + _tmp22 - _tmp23 + _tmp9 * rx;
  const Scalar _tmp25 = 2 * _tmp24;
  const Scalar _tmp26 = _tmp18 * _tmp25;
  const Scalar _tmp27 = _tmp5 * rot_init_w;
  const Scalar _tmp28 = _tmp9 * rz;
  const Scalar _tmp29 = -_tmp11 * ry - _tmp16 * rx + _tmp27 - _tmp28;
  const Scalar _tmp30 = 2 * _tmp29;
  const Scalar _tmp31 = _tmp13 * _tmp30;
  const Scalar _tmp32 = _tmp26 - _tmp31;
  const Scalar _tmp33 = -Scalar(0.010999999999999999) * _tmp32;
  const Scalar _tmp34 = _tmp13 * _tmp25;
  const Scalar _tmp35 = _tmp18 * _tmp30;
  const Scalar _tmp36 = Scalar(0.20999999999999999) * _tmp34 + Scalar(0.20999999999999999) * _tmp35;
  const Scalar _tmp37 = _tmp33 + _tmp36;
  const Scalar _tmp38 = _tmp21 + _tmp37;
  const Scalar _tmp39 = _tmp38 + p_init1;
  const Scalar _tmp40 = -_tmp39 + Scalar(-8.3888750099999996);
  const Scalar _tmp41 = 1 - 2 * std::pow(_tmp24, Scalar(2));
  const Scalar _tmp42 = Scalar(0.20999999999999999) * _tmp19 + Scalar(0.20999999999999999) * _tmp41;
  const Scalar _tmp43 = 2 * _tmp13 * _tmp18;
  const Scalar _tmp44 = _tmp24 * _tmp30;
  const Scalar _tmp45 = _tmp43 + _tmp44;
  const Scalar _tmp46 = -Scalar(0.010999999999999999) * _tmp45;
  const Scalar _tmp47 = Scalar(0.20999999999999999) * _tmp34 - Scalar(0.20999999999999999) * _tmp35;
  const Scalar _tmp48 = _tmp46 - _tmp47;
  const Scalar _tmp49 = _tmp42 + _tmp48;
  const Scalar _tmp50 = _tmp49 + p_init0;
  const Scalar _tmp51 = Scalar(2.5202214700000001) - _tmp50;
  const Scalar _tmp52 =
      std::sqrt(Scalar(std::pow(_tmp40, Scalar(2)) + std::pow(_tmp51, Scalar(2))));
  const Scalar _tmp53 = (Scalar(1) / Scalar(2)) / _tmp2;
  const Scalar _tmp54 = _tmp1 * _tmp53;
  const Scalar _tmp55 = _tmp53 * rz;
  const Scalar _tmp56 = _tmp55 * ry;
  const Scalar _tmp57 = _tmp55 * rx;
  const Scalar _tmp58 = _tmp7 / (_tmp2 * std::sqrt(_tmp2));
  const Scalar _tmp59 = _tmp1 * _tmp58;
  const Scalar _tmp60 = _tmp58 * rz;
  const Scalar _tmp61 = _tmp60 * rx;
  const Scalar _tmp62 = _tmp60 * ry;
  const Scalar _tmp63 = _tmp11 - _tmp15 * _tmp56 + _tmp22 * _tmp54 -
                        Scalar(1) / Scalar(2) * _tmp23 + _tmp27 * _tmp57 - _tmp59 * rot_init_y -
                        _tmp61 * rot_init_w + _tmp62 * rot_init_z;
  const Scalar _tmp64 = Scalar(0.83999999999999997) * _tmp63;
  const Scalar _tmp65 = _tmp13 * _tmp64;
  const Scalar _tmp66 = _tmp10 - _tmp22 * _tmp57 + _tmp27 * _tmp54 -
                        Scalar(1) / Scalar(2) * _tmp28 + _tmp56 * _tmp6 - _tmp59 * rot_init_w +
                        _tmp61 * rot_init_y - _tmp62 * rot_init_x;
  const Scalar _tmp67 = Scalar(0.83999999999999997) * _tmp18;
  const Scalar _tmp68 = _tmp66 * _tmp67;
  const Scalar _tmp69 = -_tmp68;
  const Scalar _tmp70 = -_tmp65 + _tmp69;
  const Scalar _tmp71 = -_tmp15 * _tmp54 - Scalar(1) / Scalar(2) * _tmp17 - _tmp22 * _tmp56 -
                        _tmp57 * _tmp6 + _tmp59 * rot_init_z + _tmp61 * rot_init_x +
                        _tmp62 * rot_init_y - _tmp9;
  const Scalar _tmp72 = Scalar(0.41999999999999998) * _tmp71;
  const Scalar _tmp73 = _tmp18 * _tmp72;
  const Scalar _tmp74 = Scalar(0.41999999999999998) * _tmp66;
  const Scalar _tmp75 = _tmp29 * _tmp74;
  const Scalar _tmp76 = -_tmp73 - _tmp75;
  const Scalar _tmp77 = Scalar(0.41999999999999998) * _tmp63;
  const Scalar _tmp78 = _tmp24 * _tmp77;
  const Scalar _tmp79 = -Scalar(1) / Scalar(2) * _tmp12 + _tmp15 * _tmp57 - _tmp16 +
                        _tmp27 * _tmp56 - _tmp54 * _tmp6 + _tmp59 * rot_init_x -
                        _tmp61 * rot_init_z - _tmp62 * rot_init_w;
  const Scalar _tmp80 = Scalar(0.41999999999999998) * _tmp79;
  const Scalar _tmp81 = _tmp13 * _tmp80;
  const Scalar _tmp82 = -_tmp78 - _tmp81;
  const Scalar _tmp83 = _tmp76 + _tmp82;
  const Scalar _tmp84 = _tmp70 + _tmp83;
  const Scalar _tmp85 = Scalar(0.021999999999999999) * _tmp71;
  const Scalar _tmp86 = _tmp13 * _tmp85;
  const Scalar _tmp87 = Scalar(0.021999999999999999) * _tmp79;
  const Scalar _tmp88 = _tmp18 * _tmp87;
  const Scalar _tmp89 = Scalar(0.021999999999999999) * _tmp66;
  const Scalar _tmp90 = _tmp24 * _tmp89;
  const Scalar _tmp91 = Scalar(0.021999999999999999) * _tmp63;
  const Scalar _tmp92 = _tmp29 * _tmp91;
  const Scalar _tmp93 = -_tmp86 + _tmp88 + _tmp90 - _tmp92;
  const Scalar _tmp94 = Scalar(0.83999999999999997) * _tmp79;
  const Scalar _tmp95 = _tmp24 * _tmp94;
  const Scalar _tmp96 = _tmp68 + _tmp95;
  const Scalar _tmp97 = _tmp78 + _tmp81;
  const Scalar _tmp98 = _tmp76 + _tmp97;
  const Scalar _tmp99 = _tmp96 + _tmp98;
  const Scalar _tmp100 = _tmp24 * _tmp85;
  const Scalar _tmp101 = _tmp18 * _tmp91;
  const Scalar _tmp102 = _tmp13 * _tmp89;
  const Scalar _tmp103 = _tmp29 * _tmp87;
  const Scalar _tmp104 = _tmp100 + _tmp101 + _tmp102 + _tmp103;
  const Scalar _tmp105 = _tmp13 * _tmp72;
  const Scalar _tmp106 = -_tmp105;
  const Scalar _tmp107 = _tmp18 * _tmp80;
  const Scalar _tmp108 = -_tmp107;
  const Scalar _tmp109 = _tmp24 * _tmp74;
  const Scalar _tmp110 = -_tmp109;
  const Scalar _tmp111 = _tmp29 * _tmp77;
  const Scalar _tmp112 = -_tmp111;
  const Scalar _tmp113 = _tmp13 * _tmp63;
  const Scalar _tmp114 = Scalar(0.043999999999999997) * _tmp113;
  const Scalar _tmp115 = _tmp24 * _tmp79;
  const Scalar _tmp116 = Scalar(0.043999999999999997) * _tmp115;
  const Scalar _tmp117 = _tmp114 + _tmp116;
  const Scalar _tmp118 = _tmp106 + _tmp108 + _tmp110 + _tmp112 + _tmp117;
  const Scalar _tmp119 = _tmp24 * _tmp72;
  const Scalar _tmp120 = _tmp18 * _tmp77;
  const Scalar _tmp121 = _tmp13 * _tmp74;
  const Scalar _tmp122 = _tmp29 * _tmp80;
  const Scalar _tmp123 = -_tmp119 + _tmp120 + _tmp121 - _tmp122;
  const Scalar _tmp124 = _tmp118 + _tmp123;
  const Scalar _tmp125 = _tmp50 + Scalar(-2.5202214700000001);
  const Scalar _tmp126 = _tmp39 + Scalar(8.3888750099999996);
  const Scalar _tmp127 = std::pow(_tmp125, Scalar(2)) + std::pow(_tmp126, Scalar(2));
  const Scalar _tmp128 = std::pow(_tmp127, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp129 = _tmp126 * _tmp128;
  const Scalar _tmp130 = _tmp129 * fh1;
  const Scalar _tmp131 = _tmp20 + _tmp37;
  const Scalar _tmp132 = Scalar(1.0) * _tmp131;
  const Scalar _tmp133 = -_tmp132;
  const Scalar _tmp134 = _tmp33 - _tmp36;
  const Scalar _tmp135 = _tmp134 + _tmp21;
  const Scalar _tmp136 = _tmp133 + _tmp135;
  const Scalar _tmp137 = _tmp134 + _tmp20;
  const Scalar _tmp138 = _tmp133 + _tmp137;
  const Scalar _tmp139 = Scalar(1.0) / (_tmp138);
  const Scalar _tmp140 = -_tmp42;
  const Scalar _tmp141 = _tmp46 + _tmp47;
  const Scalar _tmp142 = _tmp140 + _tmp141;
  const Scalar _tmp143 = _tmp141 + _tmp42;
  const Scalar _tmp144 = Scalar(1.0) * _tmp143;
  const Scalar _tmp145 = -_tmp142 + _tmp144;
  const Scalar _tmp146 = _tmp139 * _tmp145;
  const Scalar _tmp147 = _tmp136 * _tmp146;
  const Scalar _tmp148 = _tmp140 + _tmp48;
  const Scalar _tmp149 = _tmp144 - _tmp147 - _tmp148;
  const Scalar _tmp150 = Scalar(1.0) / (_tmp149);
  const Scalar _tmp151 = Scalar(1.0) * _tmp150;
  const Scalar _tmp152 = _tmp137 + p_init1;
  const Scalar _tmp153 = _tmp152 + Scalar(-4.8333311099999996);
  const Scalar _tmp154 = _tmp142 + p_init0;
  const Scalar _tmp155 = _tmp154 + Scalar(1.79662371);
  const Scalar _tmp156 = std::pow(_tmp153, Scalar(2)) + std::pow(_tmp155, Scalar(2));
  const Scalar _tmp157 = std::pow(_tmp156, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp158 = _tmp155 * _tmp157;
  const Scalar _tmp159 = _tmp131 + p_init1;
  const Scalar _tmp160 = _tmp159 + Scalar(-4.7752063900000001);
  const Scalar _tmp161 = _tmp143 + p_init0;
  const Scalar _tmp162 = _tmp161 + Scalar(-2.71799795);
  const Scalar _tmp163 = std::pow(_tmp162, Scalar(2));
  const Scalar _tmp164 = std::pow(_tmp160, Scalar(2)) + _tmp163;
  const Scalar _tmp165 = std::sqrt(_tmp164);
  const Scalar _tmp166 = Scalar(1.0) / (_tmp165);
  const Scalar _tmp167 = _tmp143 * _tmp166;
  const Scalar _tmp168 = _tmp162 * _tmp166;
  const Scalar _tmp169 = -_tmp131 * _tmp168 + _tmp160 * _tmp167;
  const Scalar _tmp170 = Scalar(1.0) / (_tmp162);
  const Scalar _tmp171 = _tmp165 * _tmp170;
  const Scalar _tmp172 = _tmp169 * _tmp171;
  const Scalar _tmp173 = _tmp153 * _tmp157;
  const Scalar _tmp174 = _tmp137 * _tmp158 - _tmp142 * _tmp173 + _tmp158 * _tmp172;
  const Scalar _tmp175 = _tmp160 * _tmp170;
  const Scalar _tmp176 = _tmp158 * _tmp175 - _tmp173;
  const Scalar _tmp177 = Scalar(1.0) / (_tmp176);
  const Scalar _tmp178 = Scalar(1.0) * _tmp177;
  const Scalar _tmp179 =
      Scalar(0.20999999999999999) * _tmp26 + Scalar(0.20999999999999999) * _tmp31;
  const Scalar _tmp180 =
      -Scalar(0.010999999999999999) * _tmp14 - Scalar(0.010999999999999999) * _tmp41;
  const Scalar _tmp181 =
      Scalar(0.20999999999999999) * _tmp43 - Scalar(0.20999999999999999) * _tmp44;
  const Scalar _tmp182 = _tmp180 - _tmp181;
  const Scalar _tmp183 = _tmp179 + _tmp182;
  const Scalar _tmp184 = _tmp180 + _tmp181;
  const Scalar _tmp185 = _tmp179 + _tmp184;
  const Scalar _tmp186 = _tmp158 * _tmp185;
  const Scalar _tmp187 = _tmp173 * _tmp183 - _tmp175 * _tmp186;
  const Scalar _tmp188 = _tmp178 * _tmp187;
  const Scalar _tmp189 = -_tmp158 * _tmp183 + _tmp186;
  const Scalar _tmp190 = _tmp146 * _tmp188 - _tmp178 * _tmp189;
  const Scalar _tmp191 = _tmp135 + p_init1;
  const Scalar _tmp192 = _tmp191 + Scalar(8.3196563700000006);
  const Scalar _tmp193 = _tmp148 + p_init0;
  const Scalar _tmp194 = _tmp193 + Scalar(1.9874742000000001);
  const Scalar _tmp195 = std::pow(_tmp192, Scalar(2)) + std::pow(_tmp194, Scalar(2));
  const Scalar _tmp196 = std::pow(_tmp195, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp197 = _tmp192 * _tmp196;
  const Scalar _tmp198 = _tmp194 * _tmp196;
  const Scalar _tmp199 = _tmp175 * _tmp198 - _tmp197;
  const Scalar _tmp200 = _tmp177 * _tmp199;
  const Scalar _tmp201 = _tmp185 * _tmp198;
  const Scalar _tmp202 = -_tmp179;
  const Scalar _tmp203 = _tmp182 + _tmp202;
  const Scalar _tmp204 = -_tmp175 * _tmp201 - _tmp187 * _tmp200 + _tmp197 * _tmp203;
  const Scalar _tmp205 = -_tmp146 * _tmp204 - _tmp189 * _tmp200 - _tmp198 * _tmp203 + _tmp201;
  const Scalar _tmp206 = Scalar(1.0) / (_tmp205);
  const Scalar _tmp207 =
      _tmp135 * _tmp198 - _tmp148 * _tmp197 + _tmp172 * _tmp198 - _tmp174 * _tmp200;
  const Scalar _tmp208 = _tmp206 * _tmp207;
  const Scalar _tmp209 = -_tmp174 * _tmp178 - _tmp190 * _tmp208;
  const Scalar _tmp210 = Scalar(1.0) / (_tmp207);
  const Scalar _tmp211 = _tmp205 * _tmp210;
  const Scalar _tmp212 = _tmp209 * _tmp211;
  const Scalar _tmp213 = _tmp136 * _tmp150;
  const Scalar _tmp214 = _tmp190 + _tmp212;
  const Scalar _tmp215 = _tmp206 * _tmp214;
  const Scalar _tmp216 = -_tmp188 - _tmp204 * _tmp215 + _tmp212 * _tmp213;
  const Scalar _tmp217 = Scalar(1.0) * _tmp139;
  const Scalar _tmp218 = _tmp151 * _tmp212 - _tmp216 * _tmp217;
  const Scalar _tmp219 = Scalar(1.0) * _tmp218;
  const Scalar _tmp220 = Scalar(1.0) * _tmp210;
  const Scalar _tmp221 = _tmp151 * _tmp211;
  const Scalar _tmp222 = _tmp136 * _tmp221 - _tmp204 * _tmp220;
  const Scalar _tmp223 = -_tmp217 * _tmp222 + _tmp221;
  const Scalar _tmp224 = _tmp125 * _tmp128;
  const Scalar _tmp225 = _tmp128 * _tmp49;
  const Scalar _tmp226 = fh1 * (_tmp126 * _tmp225 - _tmp224 * _tmp38);
  const Scalar _tmp227 = Scalar(1.0) * _tmp226;
  const Scalar _tmp228 = _tmp147 * _tmp151 + Scalar(1.0);
  const Scalar _tmp229 = _tmp146 * _tmp151;
  const Scalar _tmp230 = -_tmp217 * _tmp228 + _tmp229;
  const Scalar _tmp231 = fh1 * (_tmp184 + _tmp202);
  const Scalar _tmp232 = -_tmp129 * _tmp231 - Scalar(3.29616) * _tmp32 - _tmp38 * fv1;
  const Scalar _tmp233 = Scalar(1.0) * _tmp232;
  const Scalar _tmp234 = _tmp132 * _tmp146 + _tmp144;
  const Scalar _tmp235 = 0;
  const Scalar _tmp236 = _tmp204 * _tmp206;
  const Scalar _tmp237 = _tmp150 * _tmp234;
  const Scalar _tmp238 = _tmp133 - _tmp136 * _tmp237 - _tmp235 * _tmp236;
  const Scalar _tmp239 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp240 = Scalar(1.0) * _tmp239;
  const Scalar _tmp241 = _tmp224 * fh1;
  const Scalar _tmp242 = _tmp175 * _tmp177;
  const Scalar _tmp243 = _tmp175 * _tmp185;
  const Scalar _tmp244 = _tmp187 * _tmp242 + _tmp243;
  const Scalar _tmp245 = _tmp139 * _tmp244;
  const Scalar _tmp246 = -_tmp145 * _tmp245 - _tmp185 + _tmp189 * _tmp242;
  const Scalar _tmp247 = -_tmp172 + _tmp174 * _tmp242 - _tmp208 * _tmp246;
  const Scalar _tmp248 = _tmp211 * _tmp247;
  const Scalar _tmp249 = _tmp246 + _tmp248;
  const Scalar _tmp250 = _tmp206 * _tmp249;
  const Scalar _tmp251 = -_tmp204 * _tmp250 + _tmp213 * _tmp248 + _tmp244;
  const Scalar _tmp252 = _tmp151 * _tmp248 - _tmp217 * _tmp251;
  const Scalar _tmp253 = Scalar(1.0) * _tmp252;
  const Scalar _tmp254 = _tmp139 * _tmp151;
  const Scalar _tmp255 = _tmp136 * _tmp254 - _tmp151;
  const Scalar _tmp256 = _tmp224 * _tmp231 + Scalar(3.29616) * _tmp45 + _tmp49 * fv1;
  const Scalar _tmp257 = Scalar(1.0) * _tmp256;
  const Scalar _tmp258 = _tmp130 * _tmp219 + _tmp223 * _tmp227 + _tmp230 * _tmp233 +
                         _tmp240 * (-_tmp151 * _tmp234 - _tmp217 * _tmp238 + Scalar(1.0)) +
                         _tmp241 * _tmp253 + _tmp255 * _tmp257;
  const Scalar _tmp259 = -_tmp175 - _tmp199 * _tmp250;
  const Scalar _tmp260 = _tmp158 * _tmp177;
  const Scalar _tmp261 = _tmp198 * _tmp250 + _tmp259 * _tmp260 + Scalar(1.0);
  const Scalar _tmp262 = _tmp171 * _tmp241;
  const Scalar _tmp263 = -_tmp199 * _tmp215 + Scalar(1.0);
  const Scalar _tmp264 = _tmp198 * _tmp215 + _tmp260 * _tmp263;
  const Scalar _tmp265 = _tmp171 * _tmp264;
  const Scalar _tmp266 = _tmp206 * _tmp235;
  const Scalar _tmp267 = _tmp158 * _tmp266;
  const Scalar _tmp268 = _tmp198 * _tmp266 - _tmp200 * _tmp267;
  const Scalar _tmp269 = _tmp171 * _tmp239;
  const Scalar _tmp270 = _tmp158 * _tmp220;
  const Scalar _tmp271 = _tmp198 * _tmp220 - _tmp200 * _tmp270;
  const Scalar _tmp272 = _tmp171 * _tmp271;
  const Scalar _tmp273 =
      -_tmp130 * _tmp265 - _tmp226 * _tmp272 - _tmp261 * _tmp262 - _tmp268 * _tmp269;
  const Scalar _tmp274 = Scalar(1.0) / (_tmp273);
  const Scalar _tmp275 = std::asinh(_tmp258 * _tmp274);
  const Scalar _tmp276 = Scalar(1.0) * _tmp275;
  const Scalar _tmp277 = Scalar(4.7752063900000001) - _tmp159;
  const Scalar _tmp278 = Scalar(2.71799795) - _tmp161;
  const Scalar _tmp279 =
      std::sqrt(Scalar(std::pow(_tmp277, Scalar(2)) + std::pow(_tmp278, Scalar(2))));
  const Scalar _tmp280 = Scalar(9.6622558468725703) * _tmp273;
  const Scalar _tmp281 = -_tmp275 * _tmp280 - _tmp279;
  const Scalar _tmp282 = Scalar(0.1034955) * _tmp274;
  const Scalar _tmp283 = _tmp281 * _tmp282;
  const Scalar _tmp284 = _tmp86 - _tmp88 - _tmp90 + _tmp92;
  const Scalar _tmp285 = _tmp73 + _tmp75;
  const Scalar _tmp286 = _tmp285 + _tmp97;
  const Scalar _tmp287 = _tmp286 + _tmp70;
  const Scalar _tmp288 = _tmp284 + _tmp287;
  const Scalar _tmp289 = -_tmp100 - _tmp101 - _tmp102 - _tmp103;
  const Scalar _tmp290 = _tmp69 - _tmp95;
  const Scalar _tmp291 = _tmp290 + _tmp98;
  const Scalar _tmp292 = _tmp289 + _tmp291;
  const Scalar _tmp293 = _tmp160 * _tmp288 + _tmp162 * _tmp292;
  const Scalar _tmp294 = _tmp166 * _tmp170 * _tmp293;
  const Scalar _tmp295 = _tmp130 * _tmp264;
  const Scalar _tmp296 = _tmp292 / _tmp163;
  const Scalar _tmp297 = _tmp165 * _tmp296;
  const Scalar _tmp298 = _tmp241 * _tmp261;
  const Scalar _tmp299 = _tmp285 + _tmp82;
  const Scalar _tmp300 = _tmp289 + _tmp299;
  const Scalar _tmp301 = _tmp300 + _tmp96;
  const Scalar _tmp302 = _tmp196 * _tmp301;
  const Scalar _tmp303 = _tmp169 * _tmp294;
  const Scalar _tmp304 = _tmp65 + _tmp68;
  const Scalar _tmp305 = _tmp284 + _tmp304;
  const Scalar _tmp306 = _tmp305 + _tmp83;
  const Scalar _tmp307 =
      (2 * _tmp192 * _tmp306 + 2 * _tmp194 * _tmp301) / (_tmp195 * std::sqrt(_tmp195));
  const Scalar _tmp308 = (Scalar(1) / Scalar(2)) * _tmp307;
  const Scalar _tmp309 = _tmp194 * _tmp308;
  const Scalar _tmp310 = _tmp192 * _tmp308;
  const Scalar _tmp311 = _tmp169 * _tmp297;
  const Scalar _tmp312 = _tmp196 * _tmp306;
  const Scalar _tmp313 = _tmp170 * _tmp288;
  const Scalar _tmp314 = _tmp160 * _tmp296;
  const Scalar _tmp315 = _tmp175 * _tmp302 - _tmp175 * _tmp309 + _tmp198 * _tmp313 -
                         _tmp198 * _tmp314 + _tmp310 - _tmp312;
  const Scalar _tmp316 = _tmp177 * _tmp315;
  const Scalar _tmp317 = _tmp289 + _tmp99;
  const Scalar _tmp318 = _tmp284 + _tmp84;
  const Scalar _tmp319 =
      (2 * _tmp153 * _tmp318 + 2 * _tmp155 * _tmp317) / (_tmp156 * std::sqrt(_tmp156));
  const Scalar _tmp320 = (Scalar(1) / Scalar(2)) * _tmp319;
  const Scalar _tmp321 = _tmp153 * _tmp320;
  const Scalar _tmp322 = _tmp157 * _tmp317;
  const Scalar _tmp323 = _tmp155 * _tmp320;
  const Scalar _tmp324 = _tmp157 * _tmp318;
  const Scalar _tmp325 = _tmp293 / (_tmp164 * std::sqrt(_tmp164));
  const Scalar _tmp326 = _tmp166 * _tmp292;
  const Scalar _tmp327 =
      _tmp171 * (_tmp131 * _tmp162 * _tmp325 - _tmp131 * _tmp326 - _tmp143 * _tmp160 * _tmp325 +
                 _tmp160 * _tmp326 + _tmp167 * _tmp288 - _tmp168 * _tmp288);
  const Scalar _tmp328 = _tmp137 * _tmp322 - _tmp137 * _tmp323 + _tmp142 * _tmp321 -
                         _tmp142 * _tmp324 + _tmp155 * _tmp324 + _tmp158 * _tmp303 -
                         _tmp158 * _tmp311 + _tmp158 * _tmp327 + _tmp172 * _tmp322 -
                         _tmp172 * _tmp323 - _tmp173 * _tmp317;
  const Scalar _tmp329 = (_tmp158 * _tmp313 - _tmp158 * _tmp314 + _tmp175 * _tmp322 -
                          _tmp175 * _tmp323 + _tmp321 - _tmp324) /
                         std::pow(_tmp176, Scalar(2));
  const Scalar _tmp330 = _tmp199 * _tmp329;
  const Scalar _tmp331 = _tmp135 * _tmp302 - _tmp135 * _tmp309 + _tmp148 * _tmp310 -
                         _tmp148 * _tmp312 + _tmp172 * _tmp302 - _tmp172 * _tmp309 -
                         _tmp174 * _tmp316 + _tmp174 * _tmp330 + _tmp194 * _tmp312 -
                         _tmp197 * _tmp301 + _tmp198 * _tmp303 - _tmp198 * _tmp311 +
                         _tmp198 * _tmp327 - _tmp200 * _tmp328;
  const Scalar _tmp332 = _tmp331 / std::pow(_tmp207, Scalar(2));
  const Scalar _tmp333 = Scalar(1.0) * _tmp332;
  const Scalar _tmp334 = _tmp200 * _tmp322;
  const Scalar _tmp335 = Scalar(0.5) * _tmp210;
  const Scalar _tmp336 = _tmp178 * _tmp199 * _tmp332;
  const Scalar _tmp337 = _tmp290 + _tmp300;
  const Scalar _tmp338 = _tmp128 * _tmp337;
  const Scalar _tmp339 = _tmp338 * fh1;
  const Scalar _tmp340 = _tmp171 * _tmp261;
  const Scalar _tmp341 = _tmp119 - _tmp120 - _tmp121 + _tmp122;
  const Scalar _tmp342 = _tmp118 + _tmp341;
  const Scalar _tmp343 = _tmp105 + _tmp107 + _tmp109 + _tmp111 + _tmp117;
  const Scalar _tmp344 = _tmp123 + _tmp343;
  const Scalar _tmp345 = _tmp198 * _tmp344;
  const Scalar _tmp346 = Scalar(1.6799999999999999) * _tmp18 * _tmp66;
  const Scalar _tmp347 = -Scalar(1.6799999999999999) * _tmp115 - _tmp346;
  const Scalar _tmp348 = _tmp139 * _tmp347;
  const Scalar _tmp349 = _tmp185 * _tmp302;
  const Scalar _tmp350 = _tmp24 * _tmp64;
  const Scalar _tmp351 = _tmp13 * _tmp94;
  const Scalar _tmp352 = -Scalar(0.83999999999999997) * _tmp29 * _tmp66 - _tmp67 * _tmp71;
  const Scalar _tmp353 = -_tmp350 - _tmp351 + _tmp352;
  const Scalar _tmp354 = _tmp353 / std::pow(_tmp138, Scalar(2));
  const Scalar _tmp355 = _tmp145 * _tmp354;
  const Scalar _tmp356 = _tmp185 * _tmp322;
  const Scalar _tmp357 = _tmp158 * _tmp344;
  const Scalar _tmp358 = _tmp341 + _tmp343;
  const Scalar _tmp359 = _tmp173 * _tmp358 - _tmp175 * _tmp356 - _tmp175 * _tmp357 -
                         _tmp183 * _tmp321 + _tmp183 * _tmp324 - _tmp186 * _tmp313 +
                         _tmp186 * _tmp314 + _tmp243 * _tmp323;
  const Scalar _tmp360 = -_tmp175 * _tmp345 - _tmp175 * _tmp349 - _tmp187 * _tmp316 +
                         _tmp187 * _tmp330 + _tmp197 * _tmp342 - _tmp200 * _tmp359 -
                         _tmp201 * _tmp313 + _tmp201 * _tmp314 - _tmp203 * _tmp310 +
                         _tmp203 * _tmp312 + _tmp243 * _tmp309;
  const Scalar _tmp361 = -_tmp158 * _tmp358 - _tmp183 * _tmp322 + _tmp183 * _tmp323 -
                         _tmp185 * _tmp323 + _tmp356 + _tmp357;
  const Scalar _tmp362 = -_tmp146 * _tmp360 - _tmp185 * _tmp309 - _tmp189 * _tmp316 +
                         _tmp189 * _tmp330 - _tmp198 * _tmp342 - _tmp200 * _tmp361 -
                         _tmp203 * _tmp302 + _tmp203 * _tmp309 - _tmp204 * _tmp348 +
                         _tmp204 * _tmp355 + _tmp345 + _tmp349;
  const Scalar _tmp363 = _tmp362 / std::pow(_tmp205, Scalar(2));
  const Scalar _tmp364 = _tmp235 * _tmp363;
  const Scalar _tmp365 = _tmp226 * _tmp271;
  const Scalar _tmp366 = _tmp286 + _tmp305;
  const Scalar _tmp367 =
      (2 * _tmp125 * _tmp337 + 2 * _tmp126 * _tmp366) / (_tmp127 * std::sqrt(_tmp127));
  const Scalar _tmp368 = (Scalar(1) / Scalar(2)) * _tmp367;
  const Scalar _tmp369 = _tmp126 * _tmp368;
  const Scalar _tmp370 = _tmp369 * fh1;
  const Scalar _tmp371 = _tmp125 * _tmp368;
  const Scalar _tmp372 = _tmp371 * fh1;
  const Scalar _tmp373 = _tmp158 * _tmp329;
  const Scalar _tmp374 = _tmp177 * _tmp263;
  const Scalar _tmp375 = Scalar(1.0) * _tmp329;
  const Scalar _tmp376 = _tmp207 * _tmp363;
  const Scalar _tmp377 = _tmp178 * _tmp359;
  const Scalar _tmp378 = _tmp145 * _tmp217;
  const Scalar _tmp379 = _tmp146 * _tmp377 - _tmp178 * _tmp361 - _tmp187 * _tmp329 * _tmp378 +
                         _tmp188 * _tmp348 - _tmp188 * _tmp355 + _tmp189 * _tmp375;
  const Scalar _tmp380 = _tmp206 * _tmp331;
  const Scalar _tmp381 = _tmp211 * (_tmp174 * _tmp375 - _tmp178 * _tmp328 + _tmp190 * _tmp376 -
                                    _tmp190 * _tmp380 - _tmp208 * _tmp379);
  const Scalar _tmp382 = _tmp210 * _tmp362;
  const Scalar _tmp383 = _tmp209 * _tmp382;
  const Scalar _tmp384 = _tmp205 * _tmp209;
  const Scalar _tmp385 = _tmp332 * _tmp384;
  const Scalar _tmp386 = _tmp379 + _tmp381 + _tmp383 - _tmp385;
  const Scalar _tmp387 = _tmp198 * _tmp206;
  const Scalar _tmp388 = _tmp214 * _tmp363;
  const Scalar _tmp389 = _tmp199 * _tmp206;
  const Scalar _tmp390 = _tmp199 * _tmp388 - _tmp215 * _tmp315 - _tmp386 * _tmp389;
  const Scalar _tmp391 = _tmp177 * _tmp322;
  const Scalar _tmp392 = _tmp239 * _tmp268;
  const Scalar _tmp393 = fh1 * (_tmp126 * _tmp338 - _tmp224 * _tmp366 + _tmp225 * _tmp366 -
                                _tmp338 * _tmp38 - _tmp369 * _tmp49 + _tmp371 * _tmp38);
  const Scalar _tmp394 = _tmp128 * _tmp366;
  const Scalar _tmp395 = _tmp394 * fh1;
  const Scalar _tmp396 = _tmp175 * _tmp329;
  const Scalar _tmp397 = _tmp177 * _tmp314;
  const Scalar _tmp398 = _tmp177 * _tmp313;
  const Scalar _tmp399 = _tmp175 * _tmp344 + _tmp185 * _tmp313 - _tmp185 * _tmp314 -
                         _tmp187 * _tmp396 - _tmp187 * _tmp397 + _tmp187 * _tmp398 +
                         _tmp242 * _tmp359;
  const Scalar _tmp400 = _tmp106 + _tmp108 + _tmp110 + _tmp112 - _tmp114 - _tmp116 -
                         _tmp146 * _tmp399 - _tmp189 * _tmp396 - _tmp189 * _tmp397 +
                         _tmp189 * _tmp398 + _tmp242 * _tmp361 + _tmp244 * _tmp355 -
                         _tmp245 * _tmp347 + _tmp341;
  const Scalar _tmp401 = _tmp211 * (-_tmp174 * _tmp396 - _tmp174 * _tmp397 + _tmp174 * _tmp398 -
                                    _tmp208 * _tmp400 + _tmp242 * _tmp328 + _tmp246 * _tmp376 -
                                    _tmp246 * _tmp380 - _tmp303 + _tmp311 - _tmp327);
  const Scalar _tmp402 = _tmp205 * _tmp332;
  const Scalar _tmp403 = _tmp247 * _tmp402;
  const Scalar _tmp404 = _tmp247 * _tmp382;
  const Scalar _tmp405 = _tmp400 + _tmp401 - _tmp403 + _tmp404;
  const Scalar _tmp406 = _tmp249 * _tmp363;
  const Scalar _tmp407 =
      _tmp199 * _tmp406 - _tmp250 * _tmp315 - _tmp313 + _tmp314 - _tmp389 * _tmp405;
  const Scalar _tmp408 = _tmp177 * _tmp259;
  const Scalar _tmp409 =
      -_tmp130 * _tmp171 *
          (-_tmp198 * _tmp388 + _tmp215 * _tmp302 - _tmp215 * _tmp309 + _tmp260 * _tmp390 -
           _tmp263 * _tmp373 + _tmp263 * _tmp391 - _tmp323 * _tmp374 + _tmp386 * _tmp387) -
      _tmp171 * _tmp226 *
          (_tmp155 * _tmp200 * _tmp319 * _tmp335 + _tmp158 * _tmp336 - _tmp194 * _tmp307 * _tmp335 -
           _tmp198 * _tmp333 + _tmp220 * _tmp302 - _tmp220 * _tmp334 - _tmp270 * _tmp316 +
           _tmp270 * _tmp330) -
      _tmp262 * (-_tmp198 * _tmp406 + _tmp250 * _tmp302 - _tmp250 * _tmp309 - _tmp259 * _tmp373 +
                 _tmp259 * _tmp391 + _tmp260 * _tmp407 - _tmp323 * _tmp408 + _tmp387 * _tmp405) +
      _tmp265 * _tmp370 - _tmp265 * _tmp395 -
      _tmp269 * (_tmp158 * _tmp200 * _tmp364 - _tmp198 * _tmp364 + _tmp200 * _tmp266 * _tmp323 +
                 _tmp266 * _tmp302 - _tmp266 * _tmp309 - _tmp266 * _tmp334 - _tmp267 * _tmp316 +
                 _tmp267 * _tmp330) -
      _tmp272 * _tmp393 - _tmp294 * _tmp295 - _tmp294 * _tmp298 - _tmp294 * _tmp365 -
      _tmp294 * _tmp392 + _tmp295 * _tmp297 + _tmp297 * _tmp298 + _tmp297 * _tmp365 +
      _tmp297 * _tmp392 - _tmp339 * _tmp340 + _tmp340 * _tmp372;
  const Scalar _tmp410 = Scalar(9.6622558468725703) * _tmp409;
  const Scalar _tmp411 = std::pow(_tmp273, Scalar(-2));
  const Scalar _tmp412 = _tmp409 * _tmp411;
  const Scalar _tmp413 = Scalar(0.5) * _tmp367 * fh1;
  const Scalar _tmp414 = _tmp136 * _tmp348;
  const Scalar _tmp415 = Scalar(1.6799999999999999) * _tmp113 + _tmp346 + _tmp353;
  const Scalar _tmp416 = _tmp146 * _tmp415;
  const Scalar _tmp417 = _tmp136 * _tmp355;
  const Scalar _tmp418 = (_tmp347 + _tmp350 + _tmp351 + _tmp352 - _tmp414 - _tmp416 + _tmp417) /
                         std::pow(_tmp149, Scalar(2));
  const Scalar _tmp419 = _tmp220 * _tmp418;
  const Scalar _tmp420 = _tmp205 * _tmp419;
  const Scalar _tmp421 = Scalar(1.0) * _tmp354;
  const Scalar _tmp422 = _tmp150 * _tmp415;
  const Scalar _tmp423 = _tmp136 * _tmp418;
  const Scalar _tmp424 = _tmp204 * _tmp406 + _tmp213 * _tmp401 - _tmp213 * _tmp403 +
                         _tmp213 * _tmp404 - _tmp236 * _tmp405 + _tmp248 * _tmp422 -
                         _tmp248 * _tmp423 - _tmp250 * _tmp360 + _tmp399;
  const Scalar _tmp425 = _tmp151 * _tmp402;
  const Scalar _tmp426 = _tmp136 * _tmp151;
  const Scalar _tmp427 = _tmp151 * _tmp382;
  const Scalar _tmp428 = -_tmp136 * _tmp420 + _tmp136 * _tmp427 + _tmp204 * _tmp333 -
                         _tmp220 * _tmp360 + _tmp221 * _tmp415 - _tmp402 * _tmp426;
  const Scalar _tmp429 = Scalar(6.59232) * _tmp71;
  const Scalar _tmp430 = Scalar(6.59232) * _tmp18;
  const Scalar _tmp431 = Scalar(6.59232) * _tmp66;
  const Scalar _tmp432 = _tmp124 * fh1;
  const Scalar _tmp433 = Scalar(6.59232) * _tmp29;
  const Scalar _tmp434 = -_tmp129 * _tmp432 + _tmp13 * _tmp429 + _tmp231 * _tmp369 -
                         _tmp231 * _tmp394 - _tmp24 * _tmp431 - _tmp366 * fv1 - _tmp430 * _tmp79 +
                         _tmp433 * _tmp63;
  const Scalar _tmp435 = _tmp378 * _tmp418;
  const Scalar _tmp436 = _tmp151 * _tmp348;
  const Scalar _tmp437 =
      _tmp151 * _tmp414 + _tmp151 * _tmp416 - _tmp151 * _tmp417 - _tmp378 * _tmp423;
  const Scalar _tmp438 = _tmp132 * _tmp348 - _tmp132 * _tmp355 + _tmp288 * _tmp378 + _tmp292;
  const Scalar _tmp439 = _tmp150 * _tmp438;
  const Scalar _tmp440 = _tmp234 * _tmp418;
  const Scalar _tmp441 = _tmp304 + _tmp93;
  const Scalar _tmp442 = _tmp441 + _tmp83;
  const Scalar _tmp443 = -_tmp136 * _tmp439 + _tmp136 * _tmp440 + _tmp204 * _tmp364 -
                         _tmp237 * _tmp415 - _tmp266 * _tmp360 + _tmp442;
  const Scalar _tmp444 = Scalar(1.0) * _tmp418;
  const Scalar _tmp445 = _tmp13 * _tmp431 + _tmp224 * _tmp432 + _tmp231 * _tmp338 -
                         _tmp231 * _tmp371 + _tmp24 * _tmp429 + _tmp337 * fv1 + _tmp430 * _tmp63 +
                         _tmp433 * _tmp79;
  const Scalar _tmp446 = _tmp187 * _tmp375 + _tmp204 * _tmp388 + _tmp212 * _tmp422 -
                         _tmp212 * _tmp423 + _tmp213 * _tmp381 + _tmp213 * _tmp383 -
                         _tmp213 * _tmp385 - _tmp215 * _tmp360 - _tmp236 * _tmp386 - _tmp377;
  const Scalar _tmp447 = _tmp217 * _tmp423;
  const Scalar _tmp448 =
      (-_tmp258 * _tmp412 +
       _tmp274 * (-_tmp125 * _tmp252 * _tmp413 - _tmp126 * _tmp218 * _tmp413 +
                  Scalar(1.0) * _tmp130 *
                      (_tmp151 * _tmp381 + _tmp151 * _tmp383 - _tmp151 * _tmp385 +
                       _tmp216 * _tmp421 - _tmp217 * _tmp446 - _tmp384 * _tmp419) +
                  _tmp219 * _tmp395 + Scalar(1.0) * _tmp223 * _tmp393 +
                  _tmp227 * (-_tmp217 * _tmp428 + _tmp222 * _tmp421 - _tmp420 - _tmp425 + _tmp427) +
                  Scalar(1.0) * _tmp230 * _tmp434 +
                  _tmp233 * (-_tmp151 * _tmp355 - _tmp217 * _tmp437 + _tmp228 * _tmp421 - _tmp435 +
                             _tmp436) +
                  _tmp240 * (-_tmp151 * _tmp438 - _tmp217 * _tmp443 + _tmp234 * _tmp444 +
                             _tmp238 * _tmp421) +
                  Scalar(1.0) * _tmp241 *
                      (_tmp151 * _tmp401 - _tmp151 * _tmp403 + _tmp151 * _tmp404 -
                       _tmp217 * _tmp424 - _tmp247 * _tmp420 + _tmp251 * _tmp421) +
                  _tmp253 * _tmp339 + Scalar(1.0) * _tmp255 * _tmp445 +
                  _tmp257 * (_tmp254 * _tmp415 - _tmp354 * _tmp426 + _tmp444 - _tmp447))) /
      std::sqrt(Scalar(std::pow(_tmp258, Scalar(2)) * _tmp411 + 1));
  const Scalar _tmp449 = _tmp104 + _tmp299;
  const Scalar _tmp450 = _tmp220 * _tmp226;
  const Scalar _tmp451 = _tmp239 * _tmp266;
  const Scalar _tmp452 =
      _tmp130 * _tmp374 - _tmp200 * _tmp450 - _tmp200 * _tmp451 + _tmp241 * _tmp408;
  const Scalar _tmp453 = std::pow(_tmp452, Scalar(-2));
  const Scalar _tmp454 = _tmp220 * _tmp393;
  const Scalar _tmp455 = _tmp239 * _tmp364;
  const Scalar _tmp456 =
      _tmp130 * _tmp177 * _tmp390 - _tmp130 * _tmp263 * _tmp329 + _tmp177 * _tmp241 * _tmp407 -
      _tmp200 * _tmp454 + _tmp200 * _tmp455 + _tmp226 * _tmp336 - _tmp241 * _tmp259 * _tmp329 -
      _tmp316 * _tmp450 - _tmp316 * _tmp451 + _tmp330 * _tmp450 + _tmp330 * _tmp451 +
      _tmp339 * _tmp408 - _tmp370 * _tmp374 - _tmp372 * _tmp408 + _tmp374 * _tmp395;
  const Scalar _tmp457 = _tmp453 * _tmp456;
  const Scalar _tmp458 = _tmp130 * _tmp139;
  const Scalar _tmp459 = _tmp139 * _tmp239;
  const Scalar _tmp460 = _tmp139 * _tmp226;
  const Scalar _tmp461 = _tmp139 * _tmp241;
  const Scalar _tmp462 = _tmp139 * _tmp228;
  const Scalar _tmp463 = _tmp151 * _tmp256;
  const Scalar _tmp464 = _tmp136 * _tmp139;
  const Scalar _tmp465 = _tmp216 * _tmp458 + _tmp222 * _tmp460 + _tmp232 * _tmp462 +
                         _tmp238 * _tmp459 + _tmp251 * _tmp461 - _tmp463 * _tmp464;
  const Scalar _tmp466 = Scalar(1.0) / (_tmp452);
  const Scalar _tmp467 = std::asinh(_tmp465 * _tmp466);
  const Scalar _tmp468 = Scalar(1.0) * _tmp467;
  const Scalar _tmp469 = _tmp139 * _tmp216;
  const Scalar _tmp470 = _tmp139 * _tmp251;
  const Scalar _tmp471 = _tmp232 * _tmp354;
  const Scalar _tmp472 = _tmp151 * _tmp445;
  const Scalar _tmp473 =
      (-_tmp457 * _tmp465 +
       _tmp466 * (-_tmp130 * _tmp216 * _tmp354 + _tmp136 * _tmp354 * _tmp463 +
                  _tmp139 * _tmp222 * _tmp393 + _tmp139 * _tmp232 * _tmp437 -
                  _tmp139 * _tmp415 * _tmp463 - _tmp222 * _tmp226 * _tmp354 - _tmp228 * _tmp471 -
                  _tmp238 * _tmp239 * _tmp354 - _tmp241 * _tmp251 * _tmp354 + _tmp256 * _tmp447 +
                  _tmp339 * _tmp470 - _tmp370 * _tmp469 - _tmp372 * _tmp470 + _tmp395 * _tmp469 +
                  _tmp424 * _tmp461 + _tmp428 * _tmp460 + _tmp434 * _tmp462 + _tmp443 * _tmp459 +
                  _tmp446 * _tmp458 - _tmp464 * _tmp472)) /
      std::sqrt(Scalar(_tmp453 * std::pow(_tmp465, Scalar(2)) + 1));
  const Scalar _tmp474 = Scalar(4.8333311099999996) - _tmp152;
  const Scalar _tmp475 = -_tmp154 + Scalar(-1.79662371);
  const Scalar _tmp476 =
      std::sqrt(Scalar(std::pow(_tmp474, Scalar(2)) + std::pow(_tmp475, Scalar(2))));
  const Scalar _tmp477 = Scalar(9.6622558468725703) * _tmp467;
  const Scalar _tmp478 = Scalar(9.6622558468725703) * _tmp452;
  const Scalar _tmp479 = Scalar(0.1034955) * _tmp466;
  const Scalar _tmp480 = -_tmp452 * _tmp477 - _tmp476;
  const Scalar _tmp481 = _tmp479 * _tmp480;
  const Scalar _tmp482 = _tmp130 * _tmp215 + _tmp241 * _tmp250 + _tmp450 + _tmp451;
  const Scalar _tmp483 = Scalar(1.0) / (_tmp482);
  const Scalar _tmp484 = -_tmp191 + Scalar(-8.3196563700000006);
  const Scalar _tmp485 = -_tmp193 + Scalar(-1.9874742000000001);
  const Scalar _tmp486 =
      std::sqrt(Scalar(std::pow(_tmp484, Scalar(2)) + std::pow(_tmp485, Scalar(2))));
  const Scalar _tmp487 = _tmp150 * _tmp241;
  const Scalar _tmp488 = _tmp130 * _tmp150;
  const Scalar _tmp489 = -_tmp212 * _tmp488 - _tmp221 * _tmp226 - _tmp229 * _tmp232 +
                         _tmp237 * _tmp239 - _tmp248 * _tmp487 + _tmp463;
  const Scalar _tmp490 = std::asinh(_tmp483 * _tmp489);
  const Scalar _tmp491 = Scalar(9.6622558468725703) * _tmp482;
  const Scalar _tmp492 = -_tmp486 - _tmp490 * _tmp491;
  const Scalar _tmp493 = Scalar(0.1034955) * _tmp483;
  const Scalar _tmp494 = _tmp492 * _tmp493;
  const Scalar _tmp495 = Scalar(1.0) * _tmp490;
  const Scalar _tmp496 = _tmp206 * fh1;
  const Scalar _tmp497 = _tmp129 * _tmp386 * _tmp496 - _tmp130 * _tmp388 - _tmp215 * _tmp370 +
                         _tmp215 * _tmp395 + _tmp224 * _tmp405 * _tmp496 - _tmp227 * _tmp332 -
                         _tmp241 * _tmp406 + _tmp250 * _tmp339 - _tmp250 * _tmp372 + _tmp454 -
                         _tmp455;
  const Scalar _tmp498 = Scalar(9.6622558468725703) * _tmp497;
  const Scalar _tmp499 = std::pow(_tmp482, Scalar(-2));
  const Scalar _tmp500 = _tmp497 * _tmp499;
  const Scalar _tmp501 = _tmp150 * _tmp212;
  const Scalar _tmp502 = _tmp150 * _tmp248;
  const Scalar _tmp503 =
      (_tmp483 * (_tmp130 * _tmp212 * _tmp418 + _tmp145 * _tmp151 * _tmp471 +
                  _tmp205 * _tmp418 * _tmp450 - _tmp221 * _tmp393 + _tmp226 * _tmp425 -
                  _tmp226 * _tmp427 - _tmp229 * _tmp434 + _tmp232 * _tmp435 - _tmp232 * _tmp436 +
                  _tmp239 * _tmp439 - _tmp239 * _tmp440 + _tmp241 * _tmp248 * _tmp418 -
                  _tmp257 * _tmp418 - _tmp339 * _tmp502 + _tmp370 * _tmp501 + _tmp372 * _tmp502 -
                  _tmp381 * _tmp488 - _tmp383 * _tmp488 + _tmp385 * _tmp488 - _tmp395 * _tmp501 -
                  _tmp401 * _tmp487 + _tmp403 * _tmp487 - _tmp404 * _tmp487 + _tmp472) -
       _tmp489 * _tmp500) /
      std::sqrt(Scalar(std::pow(_tmp489, Scalar(2)) * _tmp499 + 1));

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) =
      _tmp124 -
      Scalar(0.5) * (2 * _tmp40 * (_tmp84 + _tmp93) + 2 * _tmp51 * (_tmp104 + _tmp99)) *
          std::sinh(Scalar(0.1034955) * _tmp0 *
                    (-_tmp52 - Scalar(9.6622558468725703) * fh1 * std::asinh(_tmp0 * fv1))) /
          _tmp52;
  _res(1, 0) =
      -_tmp280 *
          (-Scalar(0.86565325453551001) * _tmp412 + Scalar(1.0) * _tmp448 * std::sinh(_tmp276) -
           (-Scalar(0.1034955) * _tmp281 * _tmp412 +
            _tmp282 * (-_tmp275 * _tmp410 - _tmp280 * _tmp448 -
                       Scalar(1) / Scalar(2) *
                           (2 * _tmp277 * _tmp442 + 2 * _tmp278 * (_tmp449 + _tmp96)) / _tmp279)) *
               std::sinh(_tmp283)) +
      _tmp344 -
      _tmp410 * (Scalar(0.86565325453551001) * _tmp274 + std::cosh(_tmp276) - std::cosh(_tmp283));
  _res(2, 0) =
      _tmp358 -
      Scalar(9.6622558468725703) * _tmp456 *
          (Scalar(0.86625939559540499) * _tmp466 + std::cosh(_tmp468) - std::cosh(_tmp481)) -
      _tmp478 *
          (-Scalar(0.86625939559540499) * _tmp457 + Scalar(1.0) * _tmp473 * std::sinh(_tmp468) -
           (-Scalar(0.1034955) * _tmp457 * _tmp480 +
            _tmp479 * (-_tmp456 * _tmp477 - _tmp473 * _tmp478 -
                       Scalar(1) / Scalar(2) *
                           (2 * _tmp474 * (_tmp286 + _tmp441) + 2 * _tmp475 * (_tmp290 + _tmp449)) /
                           _tmp476)) *
               std::sinh(_tmp481));
  _res(3, 0) =
      _tmp342 -
      _tmp491 *
          (-Scalar(0.87679799772039002) * _tmp500 + Scalar(1.0) * _tmp503 * std::sinh(_tmp495) -
           (-Scalar(0.1034955) * _tmp492 * _tmp500 +
            _tmp493 * (-_tmp490 * _tmp498 - _tmp491 * _tmp503 -
                       Scalar(1) / Scalar(2) *
                           (2 * _tmp484 * (_tmp287 + _tmp93) + 2 * _tmp485 * (_tmp104 + _tmp291)) /
                           _tmp486)) *
               std::sinh(_tmp494)) -
      _tmp498 * (Scalar(0.87679799772039002) * _tmp483 - std::cosh(_tmp494) + std::cosh(_tmp495));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
