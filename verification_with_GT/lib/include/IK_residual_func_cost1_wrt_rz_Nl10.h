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
 * Symbolic function: IK_residual_func_cost1_wrt_rz_Nl10
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost1WrtRzNl10(
    const Scalar fh1, const Scalar fv1, const Scalar rx, const Scalar ry, const Scalar rz,
    const Scalar p_init0, const Scalar p_init1, const Scalar p_init2, const Scalar rot_init_x,
    const Scalar rot_init_y, const Scalar rot_init_z, const Scalar rot_init_w,
    const Scalar epsilon) {
  // Total ops: 1615

  // Unused inputs
  (void)p_init2;
  (void)epsilon;

  // Input arrays

  // Intermediate terms (502)
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
  const Scalar _tmp25 = 2 * _tmp18;
  const Scalar _tmp26 = _tmp24 * _tmp25;
  const Scalar _tmp27 = _tmp5 * rot_init_w;
  const Scalar _tmp28 = _tmp9 * rz;
  const Scalar _tmp29 = -_tmp11 * ry - _tmp16 * rx + _tmp27 - _tmp28;
  const Scalar _tmp30 = 2 * _tmp29;
  const Scalar _tmp31 = _tmp13 * _tmp30;
  const Scalar _tmp32 = _tmp26 - _tmp31;
  const Scalar _tmp33 = -Scalar(0.010999999999999999) * _tmp32;
  const Scalar _tmp34 = 2 * _tmp13 * _tmp24;
  const Scalar _tmp35 = _tmp18 * _tmp30;
  const Scalar _tmp36 = Scalar(0.20999999999999999) * _tmp34 + Scalar(0.20999999999999999) * _tmp35;
  const Scalar _tmp37 = _tmp33 + _tmp36;
  const Scalar _tmp38 = _tmp21 + _tmp37;
  const Scalar _tmp39 = _tmp38 + p_init1;
  const Scalar _tmp40 = -_tmp39 + Scalar(-8.3888750099999996);
  const Scalar _tmp41 = Scalar(0.20999999999999999) * _tmp34 - Scalar(0.20999999999999999) * _tmp35;
  const Scalar _tmp42 = -_tmp41;
  const Scalar _tmp43 = _tmp13 * _tmp25;
  const Scalar _tmp44 = _tmp24 * _tmp30;
  const Scalar _tmp45 = _tmp43 + _tmp44;
  const Scalar _tmp46 = -Scalar(0.010999999999999999) * _tmp45;
  const Scalar _tmp47 = 1 - 2 * std::pow(_tmp24, Scalar(2));
  const Scalar _tmp48 = Scalar(0.20999999999999999) * _tmp19 + Scalar(0.20999999999999999) * _tmp47;
  const Scalar _tmp49 = _tmp46 + _tmp48;
  const Scalar _tmp50 = _tmp42 + _tmp49;
  const Scalar _tmp51 = _tmp50 + p_init0;
  const Scalar _tmp52 = Scalar(2.5202214700000001) - _tmp51;
  const Scalar _tmp53 =
      std::sqrt(Scalar(std::pow(_tmp40, Scalar(2)) + std::pow(_tmp52, Scalar(2))));
  const Scalar _tmp54 = (Scalar(1) / Scalar(2)) / _tmp2;
  const Scalar _tmp55 = _tmp1 * _tmp54;
  const Scalar _tmp56 = _tmp54 * rz;
  const Scalar _tmp57 = _tmp56 * ry;
  const Scalar _tmp58 = _tmp56 * rx;
  const Scalar _tmp59 = _tmp7 / (_tmp2 * std::sqrt(_tmp2));
  const Scalar _tmp60 = _tmp1 * _tmp59;
  const Scalar _tmp61 = _tmp59 * rz;
  const Scalar _tmp62 = _tmp61 * rx;
  const Scalar _tmp63 = _tmp61 * ry;
  const Scalar _tmp64 = -_tmp15 * _tmp55 - Scalar(1) / Scalar(2) * _tmp17 - _tmp22 * _tmp57 -
                        _tmp58 * _tmp6 + _tmp60 * rot_init_z + _tmp62 * rot_init_x +
                        _tmp63 * rot_init_y - _tmp9;
  const Scalar _tmp65 = _tmp18 * _tmp64;
  const Scalar _tmp66 = Scalar(0.41999999999999998) * _tmp65;
  const Scalar _tmp67 = _tmp10 - _tmp22 * _tmp58 + _tmp27 * _tmp55 -
                        Scalar(1) / Scalar(2) * _tmp28 + _tmp57 * _tmp6 - _tmp60 * rot_init_w +
                        _tmp62 * rot_init_y - _tmp63 * rot_init_x;
  const Scalar _tmp68 = Scalar(0.41999999999999998) * _tmp67;
  const Scalar _tmp69 = _tmp29 * _tmp68;
  const Scalar _tmp70 = -_tmp66 - _tmp69;
  const Scalar _tmp71 = _tmp11 - _tmp15 * _tmp57 + _tmp22 * _tmp55 -
                        Scalar(1) / Scalar(2) * _tmp23 + _tmp27 * _tmp58 - _tmp60 * rot_init_y -
                        _tmp62 * rot_init_w + _tmp63 * rot_init_z;
  const Scalar _tmp72 = Scalar(0.41999999999999998) * _tmp71;
  const Scalar _tmp73 = _tmp24 * _tmp72;
  const Scalar _tmp74 = -Scalar(1) / Scalar(2) * _tmp12 + _tmp15 * _tmp58 - _tmp16 +
                        _tmp27 * _tmp57 - _tmp55 * _tmp6 + _tmp60 * rot_init_x -
                        _tmp62 * rot_init_z - _tmp63 * rot_init_w;
  const Scalar _tmp75 = Scalar(0.41999999999999998) * _tmp74;
  const Scalar _tmp76 = _tmp13 * _tmp75;
  const Scalar _tmp77 = -_tmp73 - _tmp76;
  const Scalar _tmp78 = _tmp70 + _tmp77;
  const Scalar _tmp79 = Scalar(0.83999999999999997) * _tmp67;
  const Scalar _tmp80 = _tmp18 * _tmp79;
  const Scalar _tmp81 = -_tmp80;
  const Scalar _tmp82 = Scalar(0.83999999999999997) * _tmp13;
  const Scalar _tmp83 = _tmp71 * _tmp82;
  const Scalar _tmp84 = _tmp81 - _tmp83;
  const Scalar _tmp85 = Scalar(0.021999999999999999) * _tmp64;
  const Scalar _tmp86 = _tmp13 * _tmp85;
  const Scalar _tmp87 = Scalar(0.021999999999999999) * _tmp18;
  const Scalar _tmp88 = _tmp74 * _tmp87;
  const Scalar _tmp89 = Scalar(0.021999999999999999) * _tmp67;
  const Scalar _tmp90 = _tmp24 * _tmp89;
  const Scalar _tmp91 = Scalar(0.021999999999999999) * _tmp29;
  const Scalar _tmp92 = _tmp71 * _tmp91;
  const Scalar _tmp93 = -_tmp86 + _tmp88 + _tmp90 - _tmp92;
  const Scalar _tmp94 = _tmp84 + _tmp93;
  const Scalar _tmp95 = Scalar(0.83999999999999997) * _tmp24;
  const Scalar _tmp96 = _tmp74 * _tmp95;
  const Scalar _tmp97 = _tmp80 + _tmp96;
  const Scalar _tmp98 = _tmp73 + _tmp76;
  const Scalar _tmp99 = _tmp70 + _tmp98;
  const Scalar _tmp100 = _tmp24 * _tmp85;
  const Scalar _tmp101 = _tmp71 * _tmp87;
  const Scalar _tmp102 = _tmp13 * _tmp89;
  const Scalar _tmp103 = _tmp74 * _tmp91;
  const Scalar _tmp104 = _tmp100 + _tmp101 + _tmp102 + _tmp103;
  const Scalar _tmp105 = _tmp104 + _tmp99;
  const Scalar _tmp106 = Scalar(0.41999999999999998) * _tmp64;
  const Scalar _tmp107 = _tmp106 * _tmp24;
  const Scalar _tmp108 = _tmp18 * _tmp72;
  const Scalar _tmp109 = _tmp13 * _tmp68;
  const Scalar _tmp110 = _tmp29 * _tmp75;
  const Scalar _tmp111 = -_tmp107 + _tmp108 + _tmp109 - _tmp110;
  const Scalar _tmp112 = _tmp13 * _tmp71;
  const Scalar _tmp113 = Scalar(0.043999999999999997) * _tmp112;
  const Scalar _tmp114 = _tmp24 * _tmp74;
  const Scalar _tmp115 = Scalar(0.043999999999999997) * _tmp114;
  const Scalar _tmp116 = _tmp113 + _tmp115;
  const Scalar _tmp117 = _tmp106 * _tmp13;
  const Scalar _tmp118 = _tmp18 * _tmp75;
  const Scalar _tmp119 = _tmp24 * _tmp68;
  const Scalar _tmp120 = _tmp29 * _tmp72;
  const Scalar _tmp121 = -_tmp117 - _tmp118 - _tmp119 - _tmp120;
  const Scalar _tmp122 = _tmp116 + _tmp121;
  const Scalar _tmp123 = _tmp111 + _tmp122;
  const Scalar _tmp124 = _tmp33 - _tmp36;
  const Scalar _tmp125 = _tmp124 + _tmp20;
  const Scalar _tmp126 = Scalar(1.0) * _tmp125;
  const Scalar _tmp127 = -_tmp126;
  const Scalar _tmp128 = _tmp20 + _tmp37;
  const Scalar _tmp129 = _tmp127 + _tmp128;
  const Scalar _tmp130 = _tmp124 + _tmp21;
  const Scalar _tmp131 = _tmp127 + _tmp130;
  const Scalar _tmp132 = Scalar(1.0) / (_tmp131);
  const Scalar _tmp133 = _tmp46 - _tmp48;
  const Scalar _tmp134 = _tmp133 + _tmp41;
  const Scalar _tmp135 = Scalar(1.0) * _tmp134;
  const Scalar _tmp136 = _tmp133 + _tmp42;
  const Scalar _tmp137 = _tmp135 - _tmp136;
  const Scalar _tmp138 = _tmp132 * _tmp137;
  const Scalar _tmp139 = _tmp129 * _tmp138;
  const Scalar _tmp140 = _tmp41 + _tmp49;
  const Scalar _tmp141 = _tmp135 - _tmp139 - _tmp140;
  const Scalar _tmp142 = Scalar(1.0) / (_tmp141);
  const Scalar _tmp143 = Scalar(1.0) * _tmp142;
  const Scalar _tmp144 =
      Scalar(0.20999999999999999) * _tmp43 - Scalar(0.20999999999999999) * _tmp44;
  const Scalar _tmp145 = -_tmp144;
  const Scalar _tmp146 =
      -Scalar(0.010999999999999999) * _tmp14 - Scalar(0.010999999999999999) * _tmp47;
  const Scalar _tmp147 =
      Scalar(0.20999999999999999) * _tmp26 + Scalar(0.20999999999999999) * _tmp31;
  const Scalar _tmp148 = _tmp146 + _tmp147;
  const Scalar _tmp149 = _tmp145 + _tmp148;
  const Scalar _tmp150 = _tmp125 + p_init1;
  const Scalar _tmp151 = _tmp150 + Scalar(-4.8333311099999996);
  const Scalar _tmp152 = _tmp134 + p_init0;
  const Scalar _tmp153 = _tmp152 + Scalar(1.79662371);
  const Scalar _tmp154 = Scalar(1.0) / (_tmp153);
  const Scalar _tmp155 = _tmp151 * _tmp154;
  const Scalar _tmp156 = _tmp130 + p_init1;
  const Scalar _tmp157 = _tmp156 + Scalar(8.3196563700000006);
  const Scalar _tmp158 = _tmp136 + p_init0;
  const Scalar _tmp159 = _tmp158 + Scalar(1.9874742000000001);
  const Scalar _tmp160 = std::pow(_tmp157, Scalar(2)) + std::pow(_tmp159, Scalar(2));
  const Scalar _tmp161 = std::pow(_tmp160, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp162 = _tmp159 * _tmp161;
  const Scalar _tmp163 = _tmp157 * _tmp161;
  const Scalar _tmp164 = _tmp155 * _tmp162 - _tmp163;
  const Scalar _tmp165 = Scalar(1.0) / (_tmp164);
  const Scalar _tmp166 = _tmp146 - _tmp147;
  const Scalar _tmp167 = _tmp145 + _tmp166;
  const Scalar _tmp168 = _tmp149 * _tmp162;
  const Scalar _tmp169 = -_tmp162 * _tmp167 + _tmp168;
  const Scalar _tmp170 = _tmp165 * _tmp169;
  const Scalar _tmp171 = _tmp149 * _tmp155;
  const Scalar _tmp172 = -_tmp155 * _tmp168 + _tmp163 * _tmp167;
  const Scalar _tmp173 = _tmp155 * _tmp165;
  const Scalar _tmp174 = _tmp171 + _tmp172 * _tmp173;
  const Scalar _tmp175 = -_tmp138 * _tmp174 - _tmp149 + _tmp155 * _tmp170;
  const Scalar _tmp176 = _tmp128 + p_init1;
  const Scalar _tmp177 = _tmp176 + Scalar(-4.7752063900000001);
  const Scalar _tmp178 = _tmp140 + p_init0;
  const Scalar _tmp179 = _tmp178 + Scalar(-2.71799795);
  const Scalar _tmp180 = std::pow(_tmp177, Scalar(2)) + std::pow(_tmp179, Scalar(2));
  const Scalar _tmp181 = std::pow(_tmp180, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp182 = _tmp179 * _tmp181;
  const Scalar _tmp183 = _tmp177 * _tmp181;
  const Scalar _tmp184 = _tmp155 * _tmp182 - _tmp183;
  const Scalar _tmp185 = _tmp165 * _tmp184;
  const Scalar _tmp186 = _tmp149 * _tmp182;
  const Scalar _tmp187 = _tmp144 + _tmp148;
  const Scalar _tmp188 = _tmp181 * _tmp187;
  const Scalar _tmp189 = -_tmp171 * _tmp182 - _tmp172 * _tmp185 + _tmp183 * _tmp187;
  const Scalar _tmp190 = -_tmp138 * _tmp189 - _tmp169 * _tmp185 - _tmp179 * _tmp188 + _tmp186;
  const Scalar _tmp191 = Scalar(1.0) / (_tmp190);
  const Scalar _tmp192 = std::pow(_tmp153, Scalar(2));
  const Scalar _tmp193 = std::pow(_tmp151, Scalar(2)) + _tmp192;
  const Scalar _tmp194 = std::sqrt(_tmp193);
  const Scalar _tmp195 = Scalar(1.0) / (_tmp194);
  const Scalar _tmp196 = _tmp134 * _tmp195;
  const Scalar _tmp197 = _tmp125 * _tmp195;
  const Scalar _tmp198 = _tmp151 * _tmp196 - _tmp153 * _tmp197;
  const Scalar _tmp199 = _tmp154 * _tmp194;
  const Scalar _tmp200 = _tmp198 * _tmp199;
  const Scalar _tmp201 = _tmp130 * _tmp162 - _tmp136 * _tmp163 + _tmp162 * _tmp200;
  const Scalar _tmp202 =
      _tmp128 * _tmp182 - _tmp140 * _tmp183 + _tmp182 * _tmp200 - _tmp185 * _tmp201;
  const Scalar _tmp203 = _tmp191 * _tmp202;
  const Scalar _tmp204 = _tmp173 * _tmp201 - _tmp175 * _tmp203 - _tmp200;
  const Scalar _tmp205 = Scalar(1.0) / (_tmp202);
  const Scalar _tmp206 = _tmp190 * _tmp205;
  const Scalar _tmp207 = _tmp204 * _tmp206;
  const Scalar _tmp208 = _tmp129 * _tmp142;
  const Scalar _tmp209 = _tmp175 + _tmp207;
  const Scalar _tmp210 = _tmp191 * _tmp209;
  const Scalar _tmp211 = _tmp174 - _tmp189 * _tmp210 + _tmp207 * _tmp208;
  const Scalar _tmp212 = Scalar(1.0) * _tmp132;
  const Scalar _tmp213 = _tmp143 * _tmp207 - _tmp211 * _tmp212;
  const Scalar _tmp214 = _tmp51 + Scalar(-2.5202214700000001);
  const Scalar _tmp215 = _tmp39 + Scalar(8.3888750099999996);
  const Scalar _tmp216 = std::pow(_tmp214, Scalar(2)) + std::pow(_tmp215, Scalar(2));
  const Scalar _tmp217 = std::pow(_tmp216, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp218 = _tmp217 * fh1;
  const Scalar _tmp219 = _tmp214 * _tmp218;
  const Scalar _tmp220 = Scalar(1.0) * _tmp219;
  const Scalar _tmp221 = _tmp126 * _tmp138 + _tmp135;
  const Scalar _tmp222 = 0;
  const Scalar _tmp223 = _tmp191 * _tmp222;
  const Scalar _tmp224 = _tmp142 * _tmp221;
  const Scalar _tmp225 = _tmp127 - _tmp129 * _tmp224 - _tmp189 * _tmp223;
  const Scalar _tmp226 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp227 = Scalar(1.0) * _tmp226;
  const Scalar _tmp228 = _tmp143 * _tmp206;
  const Scalar _tmp229 = Scalar(1.0) * _tmp205;
  const Scalar _tmp230 = _tmp129 * _tmp228 - _tmp189 * _tmp229;
  const Scalar _tmp231 = -_tmp212 * _tmp230 + _tmp228;
  const Scalar _tmp232 = _tmp217 * _tmp38;
  const Scalar _tmp233 = _tmp215 * _tmp217;
  const Scalar _tmp234 = fh1 * (-_tmp214 * _tmp232 + _tmp233 * _tmp50);
  const Scalar _tmp235 = Scalar(1.0) * _tmp234;
  const Scalar _tmp236 = _tmp144 + _tmp166;
  const Scalar _tmp237 = _tmp236 * fh1;
  const Scalar _tmp238 = -_tmp233 * _tmp237 - Scalar(3.29616) * _tmp32 - _tmp38 * fv1;
  const Scalar _tmp239 = _tmp139 * _tmp143 + Scalar(1.0);
  const Scalar _tmp240 = _tmp132 * _tmp239;
  const Scalar _tmp241 = _tmp138 * _tmp143;
  const Scalar _tmp242 = -Scalar(1.0) * _tmp240 + Scalar(1.0) * _tmp241;
  const Scalar _tmp243 = Scalar(1.0) * _tmp165;
  const Scalar _tmp244 = _tmp172 * _tmp243;
  const Scalar _tmp245 = _tmp138 * _tmp244 - _tmp169 * _tmp243;
  const Scalar _tmp246 = -_tmp201 * _tmp243 - _tmp203 * _tmp245;
  const Scalar _tmp247 = _tmp206 * _tmp246;
  const Scalar _tmp248 = _tmp245 + _tmp247;
  const Scalar _tmp249 = _tmp191 * _tmp248;
  const Scalar _tmp250 = -_tmp189 * _tmp249 + _tmp208 * _tmp247 - _tmp244;
  const Scalar _tmp251 = _tmp143 * _tmp247 - _tmp212 * _tmp250;
  const Scalar _tmp252 = _tmp233 * fh1;
  const Scalar _tmp253 = Scalar(1.0) * _tmp252;
  const Scalar _tmp254 = _tmp218 * _tmp236;
  const Scalar _tmp255 = _tmp214 * _tmp254 + Scalar(3.29616) * _tmp45 + _tmp50 * fv1;
  const Scalar _tmp256 = _tmp132 * _tmp143;
  const Scalar _tmp257 = Scalar(1.0) * _tmp129 * _tmp256 - Scalar(1.0) * _tmp143;
  const Scalar _tmp258 =
      _tmp213 * _tmp220 + _tmp227 * (-_tmp143 * _tmp221 - _tmp212 * _tmp225 + Scalar(1.0)) +
      _tmp231 * _tmp235 + _tmp238 * _tmp242 + _tmp251 * _tmp253 + _tmp255 * _tmp257;
  const Scalar _tmp259 = _tmp162 * _tmp185;
  const Scalar _tmp260 = _tmp182 * _tmp223 - _tmp223 * _tmp259;
  const Scalar _tmp261 = _tmp199 * _tmp226;
  const Scalar _tmp262 = -_tmp155 - _tmp184 * _tmp210;
  const Scalar _tmp263 = _tmp162 * _tmp165;
  const Scalar _tmp264 = _tmp182 * _tmp210 + _tmp262 * _tmp263 + Scalar(1.0);
  const Scalar _tmp265 = _tmp199 * _tmp264;
  const Scalar _tmp266 = -_tmp184 * _tmp249 + Scalar(1.0);
  const Scalar _tmp267 = _tmp182 * _tmp249 + _tmp263 * _tmp266;
  const Scalar _tmp268 = _tmp199 * _tmp252;
  const Scalar _tmp269 = _tmp182 * _tmp229 - _tmp229 * _tmp259;
  const Scalar _tmp270 = _tmp199 * _tmp234;
  const Scalar _tmp271 =
      -_tmp219 * _tmp265 - _tmp260 * _tmp261 - _tmp267 * _tmp268 - _tmp269 * _tmp270;
  const Scalar _tmp272 = Scalar(1.0) / (_tmp271);
  const Scalar _tmp273 = std::asinh(_tmp258 * _tmp272);
  const Scalar _tmp274 = Scalar(1.0) * _tmp273;
  const Scalar _tmp275 = Scalar(9.6622558468725703) * _tmp271;
  const Scalar _tmp276 = Scalar(4.8333311099999996) - _tmp150;
  const Scalar _tmp277 = -_tmp152 + Scalar(-1.79662371);
  const Scalar _tmp278 =
      std::sqrt(Scalar(std::pow(_tmp276, Scalar(2)) + std::pow(_tmp277, Scalar(2))));
  const Scalar _tmp279 = -_tmp273 * _tmp275 - _tmp278;
  const Scalar _tmp280 = Scalar(0.1034955) * _tmp272;
  const Scalar _tmp281 = _tmp279 * _tmp280;
  const Scalar _tmp282 = -_tmp100 - _tmp101 - _tmp102 - _tmp103;
  const Scalar _tmp283 = _tmp282 + _tmp97;
  const Scalar _tmp284 = _tmp283 + _tmp99;
  const Scalar _tmp285 = Scalar(1.0) / (_tmp192);
  const Scalar _tmp286 = _tmp194 * _tmp284 * _tmp285;
  const Scalar _tmp287 = _tmp234 * _tmp269;
  const Scalar _tmp288 = _tmp219 * _tmp264;
  const Scalar _tmp289 = _tmp86 - _tmp88 - _tmp90 + _tmp92;
  const Scalar _tmp290 = _tmp289 + _tmp84;
  const Scalar _tmp291 = _tmp290 + _tmp78;
  const Scalar _tmp292 = _tmp151 * _tmp291 + _tmp153 * _tmp284;
  const Scalar _tmp293 = _tmp154 * _tmp195 * _tmp292;
  const Scalar _tmp294 = _tmp66 + _tmp69;
  const Scalar _tmp295 = _tmp294 + _tmp77;
  const Scalar _tmp296 = _tmp283 + _tmp295;
  const Scalar _tmp297 = _tmp161 * _tmp296;
  const Scalar _tmp298 = _tmp185 * _tmp297;
  const Scalar _tmp299 = _tmp116 + _tmp117 + _tmp118 + _tmp119 + _tmp120;
  const Scalar _tmp300 = _tmp111 + _tmp299;
  const Scalar _tmp301 = _tmp107 - _tmp108 - _tmp109 + _tmp110;
  const Scalar _tmp302 = _tmp299 + _tmp301;
  const Scalar _tmp303 = Scalar(0.83999999999999997) * _tmp65;
  const Scalar _tmp304 = _tmp29 * _tmp79;
  const Scalar _tmp305 = _tmp71 * _tmp95 + _tmp74 * _tmp82;
  const Scalar _tmp306 = _tmp132 * (-_tmp303 - _tmp304 + _tmp305);
  const Scalar _tmp307 = _tmp294 + _tmp98;
  const Scalar _tmp308 = _tmp290 + _tmp307;
  const Scalar _tmp309 = _tmp81 - _tmp96;
  const Scalar _tmp310 = _tmp282 + _tmp309;
  const Scalar _tmp311 = _tmp310 + _tmp99;
  const Scalar _tmp312 =
      (2 * _tmp177 * _tmp308 + 2 * _tmp179 * _tmp311) / (_tmp180 * std::sqrt(_tmp180));
  const Scalar _tmp313 = (Scalar(1) / Scalar(2)) * _tmp312;
  const Scalar _tmp314 = _tmp177 * _tmp313;
  const Scalar _tmp315 = _tmp151 * _tmp284;
  const Scalar _tmp316 = _tmp285 * _tmp315;
  const Scalar _tmp317 = _tmp80 + _tmp83;
  const Scalar _tmp318 = _tmp317 + _tmp78;
  const Scalar _tmp319 = _tmp289 + _tmp318;
  const Scalar _tmp320 =
      (2 * _tmp157 * _tmp319 + 2 * _tmp159 * _tmp296) / (_tmp160 * std::sqrt(_tmp160));
  const Scalar _tmp321 = (Scalar(1) / Scalar(2)) * _tmp320;
  const Scalar _tmp322 = _tmp157 * _tmp321;
  const Scalar _tmp323 = _tmp161 * _tmp319;
  const Scalar _tmp324 = _tmp154 * _tmp291;
  const Scalar _tmp325 = _tmp159 * _tmp321;
  const Scalar _tmp326 = (_tmp155 * _tmp297 - _tmp155 * _tmp325 - _tmp162 * _tmp316 +
                          _tmp162 * _tmp324 + _tmp322 - _tmp323) /
                         std::pow(_tmp164, Scalar(2));
  const Scalar _tmp327 = _tmp172 * _tmp326;
  const Scalar _tmp328 = _tmp149 * _tmp297;
  const Scalar _tmp329 = _tmp155 * _tmp302;
  const Scalar _tmp330 = _tmp149 * _tmp324;
  const Scalar _tmp331 = _tmp122 + _tmp301;
  const Scalar _tmp332 = -_tmp155 * _tmp328 - _tmp162 * _tmp329 - _tmp162 * _tmp330 +
                         _tmp163 * _tmp331 - _tmp167 * _tmp322 + _tmp167 * _tmp323 +
                         _tmp168 * _tmp316 + _tmp171 * _tmp325;
  const Scalar _tmp333 = _tmp181 * _tmp311;
  const Scalar _tmp334 = _tmp179 * _tmp313;
  const Scalar _tmp335 = _tmp181 * _tmp308;
  const Scalar _tmp336 = _tmp155 * _tmp333 - _tmp155 * _tmp334 - _tmp182 * _tmp316 +
                         _tmp182 * _tmp324 + _tmp314 - _tmp335;
  const Scalar _tmp337 = _tmp165 * _tmp336;
  const Scalar _tmp338 = _tmp149 * _tmp333;
  const Scalar _tmp339 = -_tmp155 * _tmp338 + _tmp171 * _tmp334 - _tmp172 * _tmp337 -
                         _tmp182 * _tmp329 - _tmp182 * _tmp330 + _tmp183 * _tmp300 +
                         _tmp184 * _tmp327 - _tmp185 * _tmp332 + _tmp186 * _tmp316 -
                         _tmp187 * _tmp314 + _tmp187 * _tmp335;
  const Scalar _tmp340 = -_tmp149 * _tmp325 + _tmp162 * _tmp302 - _tmp162 * _tmp331 -
                         _tmp167 * _tmp297 + _tmp167 * _tmp325 + _tmp328;
  const Scalar _tmp341 = _tmp169 * _tmp326;
  const Scalar _tmp342 = Scalar(1.6799999999999999) * _tmp18 * _tmp67;
  const Scalar _tmp343 =
      (Scalar(1.6799999999999999) * _tmp112 + _tmp342) / std::pow(_tmp131, Scalar(2));
  const Scalar _tmp344 = _tmp137 * _tmp343;
  const Scalar _tmp345 = -_tmp138 * _tmp339 - _tmp149 * _tmp334 - _tmp169 * _tmp337 -
                         _tmp182 * _tmp300 + _tmp182 * _tmp302 + _tmp184 * _tmp341 -
                         _tmp185 * _tmp340 + _tmp187 * _tmp334 - _tmp188 * _tmp311 -
                         _tmp189 * _tmp306 + _tmp189 * _tmp344 + _tmp338;
  const Scalar _tmp346 = _tmp345 / std::pow(_tmp190, Scalar(2));
  const Scalar _tmp347 = _tmp222 * _tmp346;
  const Scalar _tmp348 = _tmp162 * _tmp326;
  const Scalar _tmp349 = _tmp184 * _tmp348;
  const Scalar _tmp350 = _tmp226 * _tmp260;
  const Scalar _tmp351 = _tmp252 * _tmp267;
  const Scalar _tmp352 = _tmp307 + _tmp317;
  const Scalar _tmp353 = _tmp289 + _tmp352;
  const Scalar _tmp354 = _tmp217 * _tmp353;
  const Scalar _tmp355 = _tmp295 + _tmp310;
  const Scalar _tmp356 =
      (2 * _tmp214 * _tmp355 + 2 * _tmp215 * _tmp353) / (_tmp216 * std::sqrt(_tmp216));
  const Scalar _tmp357 = (Scalar(1) / Scalar(2)) * _tmp356;
  const Scalar _tmp358 = _tmp215 * _tmp357;
  const Scalar _tmp359 = _tmp214 * _tmp357;
  const Scalar _tmp360 = fh1 * (-_tmp214 * _tmp354 - _tmp232 * _tmp355 + _tmp233 * _tmp355 +
                                _tmp354 * _tmp50 - _tmp358 * _tmp50 + _tmp359 * _tmp38);
  const Scalar _tmp361 = _tmp182 * _tmp346;
  const Scalar _tmp362 = _tmp165 * _tmp266;
  const Scalar _tmp363 = _tmp205 * _tmp345;
  const Scalar _tmp364 = _tmp246 * _tmp363;
  const Scalar _tmp365 = _tmp198 * _tmp293;
  const Scalar _tmp366 = _tmp292 / (_tmp193 * std::sqrt(_tmp193));
  const Scalar _tmp367 = _tmp199 * (_tmp125 * _tmp153 * _tmp366 - _tmp134 * _tmp151 * _tmp366 -
                                    _tmp153 * _tmp195 * _tmp291 + _tmp195 * _tmp315 +
                                    _tmp196 * _tmp291 - _tmp197 * _tmp284);
  const Scalar _tmp368 = _tmp198 * _tmp286;
  const Scalar _tmp369 = _tmp130 * _tmp297 - _tmp130 * _tmp325 + _tmp136 * _tmp322 -
                         _tmp136 * _tmp323 + _tmp159 * _tmp323 + _tmp162 * _tmp365 +
                         _tmp162 * _tmp367 - _tmp162 * _tmp368 - _tmp163 * _tmp296 +
                         _tmp200 * _tmp297 - _tmp200 * _tmp325;
  const Scalar _tmp370 = _tmp202 * _tmp346;
  const Scalar _tmp371 = _tmp201 * _tmp326;
  const Scalar _tmp372 = _tmp128 * _tmp333 - _tmp128 * _tmp334 + _tmp140 * _tmp314 -
                         _tmp140 * _tmp335 + _tmp179 * _tmp335 + _tmp182 * _tmp365 +
                         _tmp182 * _tmp367 - _tmp182 * _tmp368 - _tmp183 * _tmp311 +
                         _tmp184 * _tmp371 - _tmp185 * _tmp369 + _tmp200 * _tmp333 -
                         _tmp200 * _tmp334 - _tmp201 * _tmp337;
  const Scalar _tmp373 = _tmp191 * _tmp372;
  const Scalar _tmp374 = Scalar(1.0) * _tmp138;
  const Scalar _tmp375 = _tmp243 * _tmp332;
  const Scalar _tmp376 = _tmp138 * _tmp375 - _tmp243 * _tmp340 + _tmp244 * _tmp306 -
                         _tmp244 * _tmp344 - _tmp327 * _tmp374 + Scalar(1.0) * _tmp341;
  const Scalar _tmp377 = _tmp206 * (-_tmp203 * _tmp376 - _tmp243 * _tmp369 + _tmp245 * _tmp370 -
                                    _tmp245 * _tmp373 + Scalar(1.0) * _tmp371);
  const Scalar _tmp378 = _tmp372 / std::pow(_tmp202, Scalar(2));
  const Scalar _tmp379 = _tmp190 * _tmp378;
  const Scalar _tmp380 = _tmp246 * _tmp379;
  const Scalar _tmp381 = _tmp364 + _tmp376 + _tmp377 - _tmp380;
  const Scalar _tmp382 = _tmp182 * _tmp191;
  const Scalar _tmp383 = _tmp165 * _tmp297;
  const Scalar _tmp384 = _tmp184 * _tmp346;
  const Scalar _tmp385 = _tmp184 * _tmp191;
  const Scalar _tmp386 = _tmp248 * _tmp384 - _tmp249 * _tmp336 - _tmp381 * _tmp385;
  const Scalar _tmp387 = _tmp165 * _tmp262;
  const Scalar _tmp388 = _tmp204 * _tmp379;
  const Scalar _tmp389 = _tmp204 * _tmp363;
  const Scalar _tmp390 = _tmp165 * _tmp324;
  const Scalar _tmp391 = _tmp165 * _tmp316;
  const Scalar _tmp392 = -_tmp149 * _tmp316 - _tmp155 * _tmp327 + _tmp172 * _tmp390 -
                         _tmp172 * _tmp391 + _tmp173 * _tmp332 + _tmp329 + _tmp330;
  const Scalar _tmp393 = _tmp111 - _tmp113 - _tmp115 + _tmp121 - _tmp138 * _tmp392 -
                         _tmp155 * _tmp341 - _tmp170 * _tmp316 + _tmp170 * _tmp324 +
                         _tmp173 * _tmp340 - _tmp174 * _tmp306 + _tmp174 * _tmp344;
  const Scalar _tmp394 = _tmp206 * (-_tmp155 * _tmp371 + _tmp173 * _tmp369 + _tmp175 * _tmp370 -
                                    _tmp175 * _tmp373 + _tmp201 * _tmp390 - _tmp201 * _tmp391 -
                                    _tmp203 * _tmp393 - _tmp365 - _tmp367 + _tmp368);
  const Scalar _tmp395 = -_tmp388 + _tmp389 + _tmp393 + _tmp394;
  const Scalar _tmp396 =
      _tmp209 * _tmp384 - _tmp210 * _tmp336 + _tmp316 - _tmp324 - _tmp385 * _tmp395;
  const Scalar _tmp397 = _tmp359 * fh1;
  const Scalar _tmp398 = _tmp354 * fh1;
  const Scalar _tmp399 = _tmp199 * _tmp267;
  const Scalar _tmp400 = Scalar(0.5) * _tmp205;
  const Scalar _tmp401 = Scalar(1.0) * _tmp378;
  const Scalar _tmp402 = _tmp205 * _tmp243 * _tmp336;
  const Scalar _tmp403 = _tmp358 * fh1;
  const Scalar _tmp404 = _tmp218 * _tmp355;
  const Scalar _tmp405 =
      -_tmp199 * _tmp219 *
          (-_tmp209 * _tmp361 + _tmp210 * _tmp333 - _tmp210 * _tmp334 - _tmp262 * _tmp348 +
           _tmp262 * _tmp383 + _tmp263 * _tmp396 - _tmp325 * _tmp387 + _tmp382 * _tmp395) -
      _tmp199 * _tmp269 * _tmp360 -
      _tmp261 * (-_tmp162 * _tmp223 * _tmp337 - _tmp182 * _tmp347 + _tmp185 * _tmp223 * _tmp325 -
                 _tmp223 * _tmp298 + _tmp223 * _tmp333 - _tmp223 * _tmp334 + _tmp223 * _tmp349 +
                 _tmp259 * _tmp347) +
      _tmp265 * _tmp397 - _tmp265 * _tmp404 -
      _tmp268 * (-_tmp248 * _tmp361 + _tmp249 * _tmp333 - _tmp249 * _tmp334 + _tmp263 * _tmp386 -
                 _tmp266 * _tmp348 + _tmp266 * _tmp383 - _tmp325 * _tmp362 + _tmp381 * _tmp382) -
      _tmp270 * (_tmp159 * _tmp185 * _tmp320 * _tmp400 - _tmp162 * _tmp402 -
                 _tmp179 * _tmp312 * _tmp400 - _tmp182 * _tmp401 - _tmp229 * _tmp298 +
                 _tmp229 * _tmp333 + _tmp229 * _tmp349 + _tmp259 * _tmp401) +
      _tmp286 * _tmp287 + _tmp286 * _tmp288 + _tmp286 * _tmp350 + _tmp286 * _tmp351 -
      _tmp287 * _tmp293 - _tmp288 * _tmp293 - _tmp293 * _tmp350 - _tmp293 * _tmp351 -
      _tmp398 * _tmp399 + _tmp399 * _tmp403;
  const Scalar _tmp406 = Scalar(9.6622558468725703) * _tmp405;
  const Scalar _tmp407 = std::pow(_tmp271, Scalar(-2));
  const Scalar _tmp408 = Scalar(0.5) * _tmp356 * fh1;
  const Scalar _tmp409 = _tmp129 * _tmp344;
  const Scalar _tmp410 = _tmp129 * _tmp306;
  const Scalar _tmp411 = _tmp303 + _tmp304 + _tmp305;
  const Scalar _tmp412 = _tmp138 * _tmp411;
  const Scalar _tmp413 =
      (Scalar(1.6799999999999999) * _tmp114 + _tmp342 + _tmp409 - _tmp410 - _tmp412) /
      std::pow(_tmp141, Scalar(2));
  const Scalar _tmp414 = Scalar(1.0) * _tmp413;
  const Scalar _tmp415 = _tmp138 * _tmp414;
  const Scalar _tmp416 =
      -_tmp139 * _tmp414 - _tmp143 * _tmp409 + _tmp143 * _tmp410 + _tmp143 * _tmp412;
  const Scalar _tmp417 = _tmp143 * _tmp306;
  const Scalar _tmp418 = _tmp239 * _tmp343;
  const Scalar _tmp419 = _tmp143 * _tmp344;
  const Scalar _tmp420 = _tmp129 * _tmp132;
  const Scalar _tmp421 = _tmp129 * _tmp343;
  const Scalar _tmp422 = Scalar(6.59232) * _tmp64;
  const Scalar _tmp423 = Scalar(6.59232) * _tmp18;
  const Scalar _tmp424 = Scalar(6.59232) * _tmp67;
  const Scalar _tmp425 = Scalar(6.59232) * _tmp29;
  const Scalar _tmp426 = -_tmp123 * _tmp252 + _tmp13 * _tmp422 - _tmp237 * _tmp354 +
                         _tmp237 * _tmp358 - _tmp24 * _tmp424 - _tmp353 * fv1 - _tmp423 * _tmp74 +
                         _tmp425 * _tmp71;
  const Scalar _tmp427 = _tmp189 * _tmp191;
  const Scalar _tmp428 = _tmp189 * _tmp346;
  const Scalar _tmp429 = _tmp142 * _tmp411;
  const Scalar _tmp430 = _tmp129 * _tmp413;
  const Scalar _tmp431 = _tmp207 * _tmp429 - _tmp207 * _tmp430 - _tmp208 * _tmp388 +
                         _tmp208 * _tmp389 + _tmp208 * _tmp394 + _tmp209 * _tmp428 -
                         _tmp210 * _tmp339 + _tmp392 - _tmp395 * _tmp427;
  const Scalar _tmp432 = Scalar(1.0) * _tmp343;
  const Scalar _tmp433 = _tmp143 * _tmp379;
  const Scalar _tmp434 = _tmp206 * _tmp414;
  const Scalar _tmp435 = _tmp143 * _tmp363;
  const Scalar _tmp436 = -_tmp129 * _tmp433 - _tmp129 * _tmp434 + _tmp129 * _tmp435 +
                         _tmp189 * _tmp401 + _tmp228 * _tmp411 - _tmp229 * _tmp339;
  const Scalar _tmp437 = _tmp126 * _tmp306 - _tmp126 * _tmp344 + _tmp284 + _tmp291 * _tmp374;
  const Scalar _tmp438 = _tmp221 * _tmp413;
  const Scalar _tmp439 = _tmp352 + _tmp93;
  const Scalar _tmp440 = _tmp129 * _tmp438 + _tmp189 * _tmp347 - _tmp208 * _tmp437 -
                         _tmp223 * _tmp339 - _tmp224 * _tmp411 + _tmp439;
  const Scalar _tmp441 = _tmp208 * _tmp364 + _tmp208 * _tmp377 - _tmp208 * _tmp380 +
                         _tmp247 * _tmp429 - _tmp247 * _tmp430 + _tmp248 * _tmp428 -
                         _tmp249 * _tmp339 + Scalar(1.0) * _tmp327 - _tmp375 - _tmp381 * _tmp427;
  const Scalar _tmp442 = _tmp123 * _tmp219 + _tmp13 * _tmp424 - _tmp237 * _tmp359 +
                         _tmp24 * _tmp422 + _tmp254 * _tmp355 + _tmp355 * fv1 + _tmp423 * _tmp71 +
                         _tmp425 * _tmp74;
  const Scalar _tmp443 = _tmp405 * _tmp407;
  const Scalar _tmp444 =
      (-_tmp258 * _tmp443 +
       _tmp272 * (-_tmp213 * _tmp214 * _tmp408 + Scalar(1.0) * _tmp213 * _tmp404 -
                  _tmp215 * _tmp251 * _tmp408 +
                  _tmp220 * (-_tmp143 * _tmp388 + _tmp143 * _tmp389 + _tmp143 * _tmp394 -
                             _tmp207 * _tmp414 + _tmp211 * _tmp432 - _tmp212 * _tmp431) +
                  _tmp227 * (-_tmp143 * _tmp437 - _tmp212 * _tmp440 + _tmp221 * _tmp414 +
                             _tmp225 * _tmp432) +
                  Scalar(1.0) * _tmp231 * _tmp360 +
                  _tmp235 * (-_tmp212 * _tmp436 + _tmp230 * _tmp432 - _tmp433 - _tmp434 + _tmp435) +
                  Scalar(1.0) * _tmp238 *
                      (-_tmp212 * _tmp416 - _tmp415 + _tmp417 + Scalar(1.0) * _tmp418 - _tmp419) +
                  _tmp242 * _tmp426 + Scalar(1.0) * _tmp251 * _tmp398 +
                  _tmp253 * (_tmp143 * _tmp364 + _tmp143 * _tmp377 - _tmp143 * _tmp380 -
                             _tmp212 * _tmp441 - _tmp247 * _tmp414 + _tmp250 * _tmp432) +
                  Scalar(1.0) * _tmp255 *
                      (-_tmp143 * _tmp421 + _tmp256 * _tmp411 - _tmp414 * _tmp420 + _tmp414) +
                  _tmp257 * _tmp442)) /
      std::sqrt(Scalar(std::pow(_tmp258, Scalar(2)) * _tmp407 + 1));
  const Scalar _tmp445 = _tmp104 + _tmp295;
  const Scalar _tmp446 = _tmp165 * _tmp252;
  const Scalar _tmp447 = _tmp223 * _tmp226;
  const Scalar _tmp448 = _tmp229 * _tmp234;
  const Scalar _tmp449 = _tmp165 * _tmp219;
  const Scalar _tmp450 =
      -_tmp185 * _tmp447 - _tmp185 * _tmp448 + _tmp262 * _tmp449 + _tmp266 * _tmp446;
  const Scalar _tmp451 = Scalar(1.0) / (_tmp450);
  const Scalar _tmp452 = _tmp132 * _tmp226;
  const Scalar _tmp453 = _tmp132 * _tmp250;
  const Scalar _tmp454 = _tmp132 * _tmp219;
  const Scalar _tmp455 = _tmp143 * _tmp255;
  const Scalar _tmp456 = _tmp132 * _tmp455;
  const Scalar _tmp457 = _tmp132 * _tmp234;
  const Scalar _tmp458 = -_tmp129 * _tmp456 + _tmp211 * _tmp454 + _tmp225 * _tmp452 +
                         _tmp230 * _tmp457 + _tmp238 * _tmp240 + _tmp252 * _tmp453;
  const Scalar _tmp459 = std::asinh(_tmp451 * _tmp458);
  const Scalar _tmp460 = Scalar(1.0) * _tmp459;
  const Scalar _tmp461 = -_tmp156 + Scalar(-8.3196563700000006);
  const Scalar _tmp462 = -_tmp158 + Scalar(-1.9874742000000001);
  const Scalar _tmp463 =
      std::sqrt(Scalar(std::pow(_tmp461, Scalar(2)) + std::pow(_tmp462, Scalar(2))));
  const Scalar _tmp464 = Scalar(9.6622558468725703) * _tmp450;
  const Scalar _tmp465 = -_tmp459 * _tmp464 - _tmp463;
  const Scalar _tmp466 = Scalar(0.1034955) * _tmp451;
  const Scalar _tmp467 = _tmp465 * _tmp466;
  const Scalar _tmp468 = _tmp184 * _tmp326;
  const Scalar _tmp469 = _tmp234 * _tmp401;
  const Scalar _tmp470 = _tmp229 * _tmp360;
  const Scalar _tmp471 = _tmp226 * _tmp347;
  const Scalar _tmp472 =
      _tmp185 * _tmp469 - _tmp185 * _tmp470 + _tmp185 * _tmp471 - _tmp219 * _tmp262 * _tmp326 -
      _tmp234 * _tmp402 - _tmp252 * _tmp266 * _tmp326 - _tmp337 * _tmp447 + _tmp362 * _tmp398 -
      _tmp362 * _tmp403 + _tmp386 * _tmp446 - _tmp387 * _tmp397 + _tmp387 * _tmp404 +
      _tmp396 * _tmp449 + _tmp447 * _tmp468 + _tmp448 * _tmp468;
  const Scalar _tmp473 = Scalar(9.6622558468725703) * _tmp472;
  const Scalar _tmp474 = std::pow(_tmp450, Scalar(-2));
  const Scalar _tmp475 = _tmp472 * _tmp474;
  const Scalar _tmp476 = _tmp132 * _tmp211;
  const Scalar _tmp477 = _tmp255 * _tmp414;
  const Scalar _tmp478 = _tmp143 * _tmp442;
  const Scalar _tmp479 =
      (_tmp451 * (_tmp132 * _tmp230 * _tmp360 + _tmp132 * _tmp238 * _tmp416 +
                  _tmp132 * _tmp252 * _tmp441 - _tmp211 * _tmp219 * _tmp343 -
                  _tmp225 * _tmp226 * _tmp343 - _tmp230 * _tmp234 * _tmp343 - _tmp238 * _tmp418 +
                  _tmp240 * _tmp426 - _tmp250 * _tmp252 * _tmp343 - _tmp397 * _tmp476 +
                  _tmp398 * _tmp453 - _tmp403 * _tmp453 + _tmp404 * _tmp476 - _tmp411 * _tmp456 +
                  _tmp420 * _tmp477 - _tmp420 * _tmp478 + _tmp421 * _tmp455 + _tmp431 * _tmp454 +
                  _tmp436 * _tmp457 + _tmp440 * _tmp452) -
       _tmp458 * _tmp475) /
      std::sqrt(Scalar(std::pow(_tmp458, Scalar(2)) * _tmp474 + 1));
  const Scalar _tmp480 = _tmp210 * _tmp219 + _tmp249 * _tmp252 + _tmp447 + _tmp448;
  const Scalar _tmp481 = Scalar(1.0) / (_tmp480);
  const Scalar _tmp482 = _tmp142 * _tmp226;
  const Scalar _tmp483 = _tmp142 * _tmp219;
  const Scalar _tmp484 = _tmp142 * _tmp252;
  const Scalar _tmp485 = -_tmp207 * _tmp483 + _tmp221 * _tmp482 - _tmp228 * _tmp234 -
                         _tmp238 * _tmp241 - _tmp247 * _tmp484 + _tmp455;
  const Scalar _tmp486 = std::asinh(_tmp481 * _tmp485);
  const Scalar _tmp487 = Scalar(1.0) * _tmp486;
  const Scalar _tmp488 = Scalar(9.6622558468725703) * _tmp480;
  const Scalar _tmp489 = Scalar(4.7752063900000001) - _tmp176;
  const Scalar _tmp490 = Scalar(2.71799795) - _tmp178;
  const Scalar _tmp491 =
      std::sqrt(Scalar(std::pow(_tmp489, Scalar(2)) + std::pow(_tmp490, Scalar(2))));
  const Scalar _tmp492 = -_tmp486 * _tmp488 - _tmp491;
  const Scalar _tmp493 = Scalar(0.1034955) * _tmp481;
  const Scalar _tmp494 = _tmp492 * _tmp493;
  const Scalar _tmp495 = _tmp191 * _tmp219 * _tmp395 + _tmp191 * _tmp252 * _tmp381 -
                         _tmp209 * _tmp219 * _tmp346 - _tmp210 * _tmp397 + _tmp210 * _tmp404 -
                         _tmp248 * _tmp252 * _tmp346 + _tmp249 * _tmp398 - _tmp249 * _tmp403 -
                         _tmp469 + _tmp470 - _tmp471;
  const Scalar _tmp496 = Scalar(9.6622558468725703) * _tmp495;
  const Scalar _tmp497 = std::pow(_tmp480, Scalar(-2));
  const Scalar _tmp498 = _tmp495 * _tmp497;
  const Scalar _tmp499 = _tmp142 * _tmp247;
  const Scalar _tmp500 = _tmp142 * _tmp207;
  const Scalar _tmp501 =
      (_tmp481 * (_tmp207 * _tmp219 * _tmp413 - _tmp226 * _tmp438 - _tmp228 * _tmp360 +
                  _tmp234 * _tmp433 + _tmp234 * _tmp434 - _tmp234 * _tmp435 + _tmp238 * _tmp415 -
                  _tmp238 * _tmp417 + _tmp238 * _tmp419 - _tmp241 * _tmp426 +
                  _tmp247 * _tmp252 * _tmp413 - _tmp364 * _tmp484 - _tmp377 * _tmp484 +
                  _tmp380 * _tmp484 + _tmp388 * _tmp483 - _tmp389 * _tmp483 - _tmp394 * _tmp483 +
                  _tmp397 * _tmp500 - _tmp398 * _tmp499 + _tmp403 * _tmp499 - _tmp404 * _tmp500 +
                  _tmp437 * _tmp482 - _tmp477 + _tmp478) -
       _tmp485 * _tmp498) /
      std::sqrt(Scalar(std::pow(_tmp485, Scalar(2)) * _tmp497 + 1));

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) =
      _tmp123 -
      Scalar(0.5) * (2 * _tmp40 * (_tmp78 + _tmp94) + 2 * _tmp52 * (_tmp105 + _tmp97)) *
          std::sinh(Scalar(0.1034955) * _tmp0 *
                    (-_tmp53 - Scalar(9.6622558468725703) * fh1 * std::asinh(_tmp0 * fv1))) /
          _tmp53;
  _res(1, 0) =
      -_tmp275 *
          (-Scalar(0.86625939559540499) * _tmp443 + Scalar(1.0) * _tmp444 * std::sinh(_tmp274) -
           (-Scalar(0.1034955) * _tmp279 * _tmp443 +
            _tmp280 * (-_tmp273 * _tmp406 - _tmp275 * _tmp444 -
                       Scalar(1) / Scalar(2) *
                           (2 * _tmp276 * _tmp439 + 2 * _tmp277 * (_tmp309 + _tmp445)) / _tmp278)) *
               std::sinh(_tmp281)) +
      _tmp302 -
      _tmp406 * (Scalar(0.86625939559540499) * _tmp272 + std::cosh(_tmp274) - std::cosh(_tmp281));
  _res(2, 0) =
      _tmp331 -
      _tmp464 *
          (-Scalar(0.87679799772039002) * _tmp475 + Scalar(1.0) * _tmp479 * std::sinh(_tmp460) -
           (-Scalar(0.1034955) * _tmp465 * _tmp475 +
            _tmp466 * (-_tmp459 * _tmp473 - _tmp464 * _tmp479 -
                       Scalar(1) / Scalar(2) *
                           (2 * _tmp461 * (_tmp307 + _tmp94) + 2 * _tmp462 * (_tmp105 + _tmp309)) /
                           _tmp463)) *
               std::sinh(_tmp467)) -
      _tmp473 * (Scalar(0.87679799772039002) * _tmp451 + std::cosh(_tmp460) - std::cosh(_tmp467));
  _res(3, 0) =
      _tmp300 -
      _tmp488 *
          (-Scalar(0.86565325453551001) * _tmp498 + Scalar(1.0) * _tmp501 * std::sinh(_tmp487) -
           (-Scalar(0.1034955) * _tmp492 * _tmp498 +
            _tmp493 * (-_tmp486 * _tmp496 - _tmp488 * _tmp501 -
                       Scalar(1) / Scalar(2) *
                           (2 * _tmp489 * (_tmp318 + _tmp93) + 2 * _tmp490 * (_tmp445 + _tmp97)) /
                           _tmp491)) *
               std::sinh(_tmp494)) -
      _tmp496 * (Scalar(0.86565325453551001) * _tmp481 + std::cosh(_tmp487) - std::cosh(_tmp494));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym