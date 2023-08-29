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
 * Symbolic function: IK_residual_func_cost2_wrt_rx_Nl12
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost2WrtRxNl12(
    const Scalar fh1, const Scalar fv1, const Scalar rx, const Scalar ry, const Scalar rz,
    const Scalar p_init0, const Scalar p_init1, const Scalar p_init2, const Scalar rot_init_x,
    const Scalar rot_init_y, const Scalar rot_init_z, const Scalar rot_init_w,
    const Scalar epsilon) {
  // Total ops: 1662

  // Unused inputs
  (void)epsilon;

  // Input arrays

  // Intermediate terms (514)
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
  const Scalar _tmp14 = _tmp7 * rot_init_z;
  const Scalar _tmp15 = _tmp11 * rx;
  const Scalar _tmp16 = _tmp13 - _tmp14 * ry + _tmp15 + _tmp9 * rz;
  const Scalar _tmp17 = 2 * _tmp16;
  const Scalar _tmp18 = _tmp12 * _tmp17;
  const Scalar _tmp19 = _tmp4 * rot_init_y;
  const Scalar _tmp20 = _tmp14 * rx;
  const Scalar _tmp21 = _tmp11 * ry + _tmp19 + _tmp20 - _tmp8 * rz;
  const Scalar _tmp22 = _tmp4 * rot_init_w;
  const Scalar _tmp23 = _tmp8 * rx;
  const Scalar _tmp24 = -_tmp14 * rz + _tmp22 - _tmp23 - _tmp9 * ry;
  const Scalar _tmp25 = 2 * _tmp24;
  const Scalar _tmp26 = _tmp21 * _tmp25;
  const Scalar _tmp27 = Scalar(0.20999999999999999) * _tmp18 - Scalar(0.20999999999999999) * _tmp26;
  const Scalar _tmp28 = -2 * std::pow(_tmp16, Scalar(2));
  const Scalar _tmp29 = 1 - 2 * std::pow(_tmp21, Scalar(2));
  const Scalar _tmp30 =
      -Scalar(0.010999999999999999) * _tmp28 - Scalar(0.010999999999999999) * _tmp29;
  const Scalar _tmp31 = 2 * _tmp12 * _tmp21;
  const Scalar _tmp32 = _tmp16 * _tmp25;
  const Scalar _tmp33 = Scalar(0.20999999999999999) * _tmp31 + Scalar(0.20999999999999999) * _tmp32;
  const Scalar _tmp34 = _tmp30 + _tmp33;
  const Scalar _tmp35 = _tmp27 + _tmp34;
  const Scalar _tmp36 = -_tmp35 - p_init2 + Scalar(8.36416322);
  const Scalar _tmp37 = _tmp17 * _tmp21;
  const Scalar _tmp38 = _tmp12 * _tmp25;
  const Scalar _tmp39 = Scalar(0.20999999999999999) * _tmp37 + Scalar(0.20999999999999999) * _tmp38;
  const Scalar _tmp40 = _tmp31 - _tmp32;
  const Scalar _tmp41 = -Scalar(0.010999999999999999) * _tmp40;
  const Scalar _tmp42 = -2 * std::pow(_tmp12, Scalar(2));
  const Scalar _tmp43 = Scalar(0.20999999999999999) * _tmp28 +
                        Scalar(0.20999999999999999) * _tmp42 + Scalar(0.20999999999999999);
  const Scalar _tmp44 = _tmp41 + _tmp43;
  const Scalar _tmp45 = _tmp39 + _tmp44;
  const Scalar _tmp46 = _tmp45 + p_init1;
  const Scalar _tmp47 = Scalar(4.7752063900000001) - _tmp46;
  const Scalar _tmp48 = Scalar(0.20999999999999999) * _tmp37 - Scalar(0.20999999999999999) * _tmp38;
  const Scalar _tmp49 = _tmp18 + _tmp26;
  const Scalar _tmp50 = -Scalar(0.010999999999999999) * _tmp49;
  const Scalar _tmp51 = Scalar(0.20999999999999999) * _tmp29 + Scalar(0.20999999999999999) * _tmp42;
  const Scalar _tmp52 = _tmp50 + _tmp51;
  const Scalar _tmp53 = _tmp48 + _tmp52;
  const Scalar _tmp54 = _tmp53 + p_init0;
  const Scalar _tmp55 = Scalar(2.71799795) - _tmp54;
  const Scalar _tmp56 = std::pow(_tmp47, Scalar(2)) + std::pow(_tmp55, Scalar(2));
  const Scalar _tmp57 = (Scalar(1) / Scalar(2)) / _tmp1;
  const Scalar _tmp58 = _tmp0 * _tmp57;
  const Scalar _tmp59 = _tmp57 * rx;
  const Scalar _tmp60 = _tmp59 * rz;
  const Scalar _tmp61 = _tmp59 * ry;
  const Scalar _tmp62 = _tmp6 / (_tmp1 * std::sqrt(_tmp1));
  const Scalar _tmp63 = _tmp0 * _tmp62;
  const Scalar _tmp64 = _tmp62 * rx;
  const Scalar _tmp65 = _tmp64 * ry;
  const Scalar _tmp66 = _tmp64 * rz;
  const Scalar _tmp67 = -_tmp13 * _tmp58 - Scalar(1) / Scalar(2) * _tmp15 - _tmp19 * _tmp61 -
                        _tmp5 * _tmp60 + _tmp63 * rot_init_x + _tmp65 * rot_init_y +
                        _tmp66 * rot_init_z - _tmp8;
  const Scalar _tmp68 = Scalar(0.41999999999999998) * _tmp67;
  const Scalar _tmp69 = _tmp16 * _tmp68;
  const Scalar _tmp70 = _tmp13 * _tmp61 - _tmp19 * _tmp58 - Scalar(1) / Scalar(2) * _tmp20 +
                        _tmp22 * _tmp60 + _tmp63 * rot_init_y - _tmp65 * rot_init_x -
                        _tmp66 * rot_init_w - _tmp9;
  const Scalar _tmp71 = Scalar(0.41999999999999998) * _tmp70;
  const Scalar _tmp72 = _tmp21 * _tmp71;
  const Scalar _tmp73 = -Scalar(1) / Scalar(2) * _tmp10 - _tmp13 * _tmp60 + _tmp14 +
                        _tmp22 * _tmp61 + _tmp5 * _tmp58 - _tmp63 * rot_init_z -
                        _tmp65 * rot_init_w + _tmp66 * rot_init_x;
  const Scalar _tmp74 = Scalar(0.41999999999999998) * _tmp73;
  const Scalar _tmp75 = _tmp12 * _tmp74;
  const Scalar _tmp76 = _tmp11 + _tmp19 * _tmp60 + _tmp22 * _tmp58 -
                        Scalar(1) / Scalar(2) * _tmp23 - _tmp5 * _tmp61 - _tmp63 * rot_init_w +
                        _tmp65 * rot_init_z - _tmp66 * rot_init_y;
  const Scalar _tmp77 = Scalar(0.41999999999999998) * _tmp76;
  const Scalar _tmp78 = _tmp24 * _tmp77;
  const Scalar _tmp79 = -_tmp69 - _tmp72 - _tmp75 - _tmp78;
  const Scalar _tmp80 = _tmp21 * _tmp68;
  const Scalar _tmp81 = _tmp16 * _tmp71;
  const Scalar _tmp82 = _tmp12 * _tmp77;
  const Scalar _tmp83 = _tmp24 * _tmp74;
  const Scalar _tmp84 = _tmp80 - _tmp81 - _tmp82 + _tmp83;
  const Scalar _tmp85 = _tmp21 * _tmp73;
  const Scalar _tmp86 = Scalar(0.043999999999999997) * _tmp85;
  const Scalar _tmp87 = _tmp16 * _tmp76;
  const Scalar _tmp88 = Scalar(0.043999999999999997) * _tmp87;
  const Scalar _tmp89 = -_tmp86 - _tmp88;
  const Scalar _tmp90 = _tmp84 + _tmp89;
  const Scalar _tmp91 = _tmp12 * _tmp68;
  const Scalar _tmp92 = _tmp24 * _tmp71;
  const Scalar _tmp93 = _tmp91 + _tmp92;
  const Scalar _tmp94 = _tmp16 * _tmp74;
  const Scalar _tmp95 = _tmp21 * _tmp77;
  const Scalar _tmp96 = -_tmp94 - _tmp95;
  const Scalar _tmp97 = _tmp93 + _tmp96;
  const Scalar _tmp98 = Scalar(0.83999999999999997) * _tmp12;
  const Scalar _tmp99 = _tmp70 * _tmp98;
  const Scalar _tmp100 = Scalar(0.83999999999999997) * _tmp85;
  const Scalar _tmp101 = _tmp100 + _tmp99;
  const Scalar _tmp102 = _tmp101 + _tmp97;
  const Scalar _tmp103 = Scalar(0.021999999999999999) * _tmp67;
  const Scalar _tmp104 = _tmp103 * _tmp21;
  const Scalar _tmp105 = Scalar(0.021999999999999999) * _tmp70;
  const Scalar _tmp106 = _tmp105 * _tmp16;
  const Scalar _tmp107 = Scalar(0.021999999999999999) * _tmp12;
  const Scalar _tmp108 = _tmp107 * _tmp76;
  const Scalar _tmp109 = Scalar(0.021999999999999999) * _tmp24;
  const Scalar _tmp110 = _tmp109 * _tmp73;
  const Scalar _tmp111 = _tmp104 + _tmp106 + _tmp108 + _tmp110;
  const Scalar _tmp112 = _tmp103 * _tmp16;
  const Scalar _tmp113 = _tmp105 * _tmp21;
  const Scalar _tmp114 = _tmp107 * _tmp73;
  const Scalar _tmp115 = _tmp109 * _tmp76;
  const Scalar _tmp116 = -_tmp112 + _tmp113 + _tmp114 - _tmp115;
  const Scalar _tmp117 = -_tmp91 - _tmp92;
  const Scalar _tmp118 = _tmp117 + _tmp96;
  const Scalar _tmp119 = Scalar(0.83999999999999997) * _tmp76;
  const Scalar _tmp120 = _tmp119 * _tmp16;
  const Scalar _tmp121 = _tmp120 + _tmp99;
  const Scalar _tmp122 = _tmp118 + _tmp121;
  const Scalar _tmp123 = 2 * _tmp47 * (_tmp116 + _tmp122) + 2 * _tmp55 * (_tmp102 + _tmp111);
  const Scalar _tmp124 = std::sqrt(_tmp56);
  const Scalar _tmp125 = Scalar(1.0) / (fh1);
  const Scalar _tmp126 = -_tmp27;
  const Scalar _tmp127 = _tmp30 - _tmp33;
  const Scalar _tmp128 = _tmp126 + _tmp127;
  const Scalar _tmp129 = -_tmp128 - p_init2 + Scalar(8.4718465799999993);
  const Scalar _tmp130 = -_tmp39;
  const Scalar _tmp131 = _tmp41 - _tmp43;
  const Scalar _tmp132 = _tmp130 + _tmp131;
  const Scalar _tmp133 = _tmp132 + p_init1;
  const Scalar _tmp134 = -_tmp133 + Scalar(-8.3196563700000006);
  const Scalar _tmp135 = -_tmp48;
  const Scalar _tmp136 = _tmp50 - _tmp51;
  const Scalar _tmp137 = _tmp135 + _tmp136;
  const Scalar _tmp138 = _tmp137 + p_init0;
  const Scalar _tmp139 = -_tmp138 + Scalar(-1.9874742000000001);
  const Scalar _tmp140 = std::pow(_tmp134, Scalar(2)) + std::pow(_tmp139, Scalar(2));
  const Scalar _tmp141 = _tmp69 + _tmp72 + _tmp75 + _tmp78;
  const Scalar _tmp142 = -_tmp80 + _tmp81 + _tmp82 - _tmp83;
  const Scalar _tmp143 = _tmp141 + _tmp142;
  const Scalar _tmp144 = _tmp143 + _tmp89;
  const Scalar _tmp145 = -_tmp99;
  const Scalar _tmp146 = -_tmp100 + _tmp145;
  const Scalar _tmp147 = _tmp94 + _tmp95;
  const Scalar _tmp148 = _tmp117 + _tmp147;
  const Scalar _tmp149 = _tmp111 + _tmp148;
  const Scalar _tmp150 = -_tmp120 + _tmp145;
  const Scalar _tmp151 = _tmp147 + _tmp93;
  const Scalar _tmp152 = _tmp150 + _tmp151;
  const Scalar _tmp153 = _tmp116 + _tmp152;
  const Scalar _tmp154 = 2 * _tmp134 * _tmp153 + 2 * _tmp139 * (_tmp146 + _tmp149);
  const Scalar _tmp155 = std::sqrt(_tmp140);
  const Scalar _tmp156 = _tmp35 * fh1;
  const Scalar _tmp157 = _tmp46 + Scalar(-4.7752063900000001);
  const Scalar _tmp158 = _tmp54 + Scalar(-2.71799795);
  const Scalar _tmp159 = std::pow(_tmp157, Scalar(2)) + std::pow(_tmp158, Scalar(2));
  const Scalar _tmp160 = std::pow(_tmp159, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp161 = _tmp157 * _tmp160;
  const Scalar _tmp162 = -_tmp156 * _tmp161 - Scalar(3.29616) * _tmp40 - _tmp45 * fv1;
  const Scalar _tmp163 = _tmp136 + _tmp48;
  const Scalar _tmp164 = Scalar(1.0) * _tmp132;
  const Scalar _tmp165 = -_tmp164;
  const Scalar _tmp166 = _tmp130 + _tmp44;
  const Scalar _tmp167 = _tmp165 + _tmp166;
  const Scalar _tmp168 = _tmp131 + _tmp39;
  const Scalar _tmp169 = _tmp165 + _tmp168;
  const Scalar _tmp170 = Scalar(1.0) / (_tmp169);
  const Scalar _tmp171 = _tmp135 + _tmp52;
  const Scalar _tmp172 = Scalar(1.0) * _tmp137;
  const Scalar _tmp173 = -_tmp171 + _tmp172;
  const Scalar _tmp174 = _tmp170 * _tmp173;
  const Scalar _tmp175 = _tmp167 * _tmp174;
  const Scalar _tmp176 = -_tmp163 + _tmp172 - _tmp175;
  const Scalar _tmp177 = Scalar(1.0) / (_tmp176);
  const Scalar _tmp178 = Scalar(1.0) * _tmp177;
  const Scalar _tmp179 = _tmp174 * _tmp178;
  const Scalar _tmp180 = _tmp175 * _tmp178 + Scalar(1.0);
  const Scalar _tmp181 = Scalar(1.0) * _tmp170;
  const Scalar _tmp182 = Scalar(1.0) * _tmp179 - Scalar(1.0) * _tmp180 * _tmp181;
  const Scalar _tmp183 = _tmp164 * _tmp174 + _tmp172;
  const Scalar _tmp184 = _tmp177 * _tmp183;
  const Scalar _tmp185 = 0;
  const Scalar _tmp186 = _tmp133 + Scalar(8.3196563700000006);
  const Scalar _tmp187 = _tmp138 + Scalar(1.9874742000000001);
  const Scalar _tmp188 = Scalar(1.0) / (_tmp187);
  const Scalar _tmp189 = _tmp186 * _tmp188;
  const Scalar _tmp190 = _tmp166 + p_init1;
  const Scalar _tmp191 = _tmp190 + Scalar(-4.8333311099999996);
  const Scalar _tmp192 = _tmp163 + p_init0;
  const Scalar _tmp193 = _tmp192 + Scalar(1.79662371);
  const Scalar _tmp194 = std::pow(_tmp191, Scalar(2)) + std::pow(_tmp193, Scalar(2));
  const Scalar _tmp195 = std::pow(_tmp194, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp196 = _tmp193 * _tmp195;
  const Scalar _tmp197 = _tmp128 * _tmp196;
  const Scalar _tmp198 = _tmp126 + _tmp34;
  const Scalar _tmp199 = _tmp191 * _tmp195;
  const Scalar _tmp200 = _tmp127 + _tmp27;
  const Scalar _tmp201 = _tmp171 + p_init0;
  const Scalar _tmp202 = _tmp201 + Scalar(-2.5202214700000001);
  const Scalar _tmp203 = _tmp168 + p_init1;
  const Scalar _tmp204 = _tmp203 + Scalar(8.3888750099999996);
  const Scalar _tmp205 = std::pow(_tmp202, Scalar(2)) + std::pow(_tmp204, Scalar(2));
  const Scalar _tmp206 = std::pow(_tmp205, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp207 = _tmp204 * _tmp206;
  const Scalar _tmp208 = _tmp202 * _tmp206;
  const Scalar _tmp209 = _tmp128 * _tmp208;
  const Scalar _tmp210 = -_tmp189 * _tmp209 + _tmp200 * _tmp207;
  const Scalar _tmp211 = _tmp189 * _tmp196 - _tmp199;
  const Scalar _tmp212 = _tmp189 * _tmp208 - _tmp207;
  const Scalar _tmp213 = Scalar(1.0) / (_tmp212);
  const Scalar _tmp214 = _tmp211 * _tmp213;
  const Scalar _tmp215 = -_tmp189 * _tmp197 + _tmp198 * _tmp199 - _tmp210 * _tmp214;
  const Scalar _tmp216 = -_tmp200 * _tmp208 + _tmp209;
  const Scalar _tmp217 = -_tmp174 * _tmp215 - _tmp196 * _tmp198 + _tmp197 - _tmp214 * _tmp216;
  const Scalar _tmp218 = Scalar(1.0) / (_tmp217);
  const Scalar _tmp219 = _tmp215 * _tmp218;
  const Scalar _tmp220 = _tmp165 - _tmp167 * _tmp184 - _tmp185 * _tmp219;
  const Scalar _tmp221 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp222 = Scalar(1.0) * _tmp221;
  const Scalar _tmp223 = std::pow(_tmp187, Scalar(2));
  const Scalar _tmp224 = std::pow(_tmp186, Scalar(2)) + _tmp223;
  const Scalar _tmp225 = std::sqrt(_tmp224);
  const Scalar _tmp226 = Scalar(1.0) / (_tmp225);
  const Scalar _tmp227 = _tmp132 * _tmp226;
  const Scalar _tmp228 = _tmp137 * _tmp186;
  const Scalar _tmp229 = -_tmp187 * _tmp227 + _tmp226 * _tmp228;
  const Scalar _tmp230 = _tmp188 * _tmp225;
  const Scalar _tmp231 = _tmp229 * _tmp230;
  const Scalar _tmp232 = _tmp168 * _tmp208 - _tmp171 * _tmp207 + _tmp208 * _tmp231;
  const Scalar _tmp233 = Scalar(1.0) * _tmp213;
  const Scalar _tmp234 = _tmp210 * _tmp233;
  const Scalar _tmp235 = _tmp174 * _tmp234 - _tmp216 * _tmp233;
  const Scalar _tmp236 =
      -_tmp163 * _tmp199 + _tmp166 * _tmp196 + _tmp196 * _tmp231 - _tmp214 * _tmp232;
  const Scalar _tmp237 = _tmp218 * _tmp236;
  const Scalar _tmp238 = -_tmp232 * _tmp233 - _tmp235 * _tmp237;
  const Scalar _tmp239 = Scalar(1.0) / (_tmp236);
  const Scalar _tmp240 = _tmp217 * _tmp239;
  const Scalar _tmp241 = _tmp238 * _tmp240;
  const Scalar _tmp242 = _tmp167 * _tmp177;
  const Scalar _tmp243 = _tmp235 + _tmp241;
  const Scalar _tmp244 = _tmp218 * _tmp243;
  const Scalar _tmp245 = -_tmp215 * _tmp244 - _tmp234 + _tmp241 * _tmp242;
  const Scalar _tmp246 = _tmp178 * _tmp241 - _tmp181 * _tmp245;
  const Scalar _tmp247 = _tmp161 * fh1;
  const Scalar _tmp248 = Scalar(1.0) * _tmp247;
  const Scalar _tmp249 = _tmp178 * _tmp240;
  const Scalar _tmp250 = Scalar(1.0) * _tmp239;
  const Scalar _tmp251 = _tmp167 * _tmp249 - _tmp215 * _tmp250;
  const Scalar _tmp252 = -_tmp181 * _tmp251 + _tmp249;
  const Scalar _tmp253 = _tmp160 * _tmp53;
  const Scalar _tmp254 = _tmp160 * _tmp45;
  const Scalar _tmp255 = fh1 * (_tmp157 * _tmp253 - _tmp158 * _tmp254);
  const Scalar _tmp256 = Scalar(1.0) * _tmp255;
  const Scalar _tmp257 = _tmp128 * _tmp189;
  const Scalar _tmp258 = _tmp189 * _tmp213;
  const Scalar _tmp259 = _tmp210 * _tmp258 + _tmp257;
  const Scalar _tmp260 = -_tmp128 - _tmp174 * _tmp259 + _tmp216 * _tmp258;
  const Scalar _tmp261 = -_tmp231 + _tmp232 * _tmp258 - _tmp237 * _tmp260;
  const Scalar _tmp262 = _tmp240 * _tmp261;
  const Scalar _tmp263 = _tmp260 + _tmp262;
  const Scalar _tmp264 = -_tmp219 * _tmp263 + _tmp242 * _tmp262 + _tmp259;
  const Scalar _tmp265 = _tmp178 * _tmp262 - _tmp181 * _tmp264;
  const Scalar _tmp266 = _tmp158 * _tmp160;
  const Scalar _tmp267 = Scalar(1.0) * fh1;
  const Scalar _tmp268 = _tmp266 * _tmp267;
  const Scalar _tmp269 = _tmp156 * _tmp266 + Scalar(3.29616) * _tmp49 + _tmp53 * fv1;
  const Scalar _tmp270 = _tmp170 * _tmp178;
  const Scalar _tmp271 = Scalar(1.0) * _tmp167 * _tmp270 - Scalar(1.0) * _tmp178;
  const Scalar _tmp272 =
      _tmp162 * _tmp182 + _tmp222 * (-_tmp178 * _tmp183 - _tmp181 * _tmp220 + Scalar(1.0)) +
      _tmp246 * _tmp248 + _tmp252 * _tmp256 + _tmp265 * _tmp268 + _tmp269 * _tmp271;
  const Scalar _tmp273 = -_tmp211 * _tmp244 + Scalar(1.0);
  const Scalar _tmp274 = _tmp213 * _tmp273;
  const Scalar _tmp275 = _tmp196 * _tmp244 + _tmp208 * _tmp274;
  const Scalar _tmp276 = _tmp230 * _tmp247;
  const Scalar _tmp277 = _tmp233 * _tmp239;
  const Scalar _tmp278 = _tmp211 * _tmp277;
  const Scalar _tmp279 = _tmp196 * _tmp250 - _tmp208 * _tmp278;
  const Scalar _tmp280 = _tmp230 * _tmp255;
  const Scalar _tmp281 = _tmp185 * _tmp218;
  const Scalar _tmp282 = _tmp208 * _tmp281;
  const Scalar _tmp283 = _tmp196 * _tmp281 - _tmp214 * _tmp282;
  const Scalar _tmp284 = _tmp221 * _tmp230;
  const Scalar _tmp285 = _tmp266 * fh1;
  const Scalar _tmp286 = _tmp218 * _tmp263;
  const Scalar _tmp287 = -_tmp189 - _tmp211 * _tmp286;
  const Scalar _tmp288 = _tmp208 * _tmp213;
  const Scalar _tmp289 = _tmp196 * _tmp286 + _tmp287 * _tmp288 + Scalar(1.0);
  const Scalar _tmp290 = _tmp230 * _tmp289;
  const Scalar _tmp291 =
      -_tmp275 * _tmp276 - _tmp279 * _tmp280 - _tmp283 * _tmp284 - _tmp285 * _tmp290;
  const Scalar _tmp292 = Scalar(1.0) / (_tmp291);
  const Scalar _tmp293 = std::asinh(_tmp272 * _tmp292);
  const Scalar _tmp294 = Scalar(9.6622558468725703) * _tmp291;
  const Scalar _tmp295 = -_tmp155 - _tmp293 * _tmp294;
  const Scalar _tmp296 = Scalar(0.1034955) * _tmp292;
  const Scalar _tmp297 = _tmp295 * _tmp296;
  const Scalar _tmp298 = std::pow(_tmp291, Scalar(-2));
  const Scalar _tmp299 = -_tmp104 - _tmp106 - _tmp108 - _tmp110;
  const Scalar _tmp300 = _tmp102 + _tmp299;
  const Scalar _tmp301 = _tmp112 - _tmp113 - _tmp114 + _tmp115;
  const Scalar _tmp302 = _tmp122 + _tmp301;
  const Scalar _tmp303 = _tmp186 * _tmp302 + _tmp187 * _tmp300;
  const Scalar _tmp304 = _tmp188 * _tmp226 * _tmp303;
  const Scalar _tmp305 = _tmp255 * _tmp279;
  const Scalar _tmp306 = _tmp230 * _tmp275;
  const Scalar _tmp307 = _tmp148 + _tmp299;
  const Scalar _tmp308 = _tmp146 + _tmp307;
  const Scalar _tmp309 = _tmp152 + _tmp301;
  const Scalar _tmp310 =
      (2 * _tmp157 * _tmp309 + 2 * _tmp158 * _tmp308) / (_tmp159 * std::sqrt(_tmp159));
  const Scalar _tmp311 = (Scalar(1) / Scalar(2)) * _tmp310;
  const Scalar _tmp312 = _tmp157 * _tmp311;
  const Scalar _tmp313 = _tmp312 * fh1;
  const Scalar _tmp314 = _tmp146 + _tmp97;
  const Scalar _tmp315 = _tmp299 + _tmp314;
  const Scalar _tmp316 = _tmp121 + _tmp151;
  const Scalar _tmp317 = _tmp301 + _tmp316;
  const Scalar _tmp318 =
      (2 * _tmp202 * _tmp315 + 2 * _tmp204 * _tmp317) / (_tmp205 * std::sqrt(_tmp205));
  const Scalar _tmp319 = (Scalar(1) / Scalar(2)) * _tmp318;
  const Scalar _tmp320 = _tmp202 * _tmp319;
  const Scalar _tmp321 = _tmp101 + _tmp307;
  const Scalar _tmp322 = _tmp118 + _tmp150;
  const Scalar _tmp323 = _tmp301 + _tmp322;
  const Scalar _tmp324 =
      (2 * _tmp191 * _tmp323 + 2 * _tmp193 * _tmp321) / (_tmp194 * std::sqrt(_tmp194));
  const Scalar _tmp325 = (Scalar(1) / Scalar(2)) * _tmp324;
  const Scalar _tmp326 = _tmp193 * _tmp325;
  const Scalar _tmp327 = _tmp195 * _tmp321;
  const Scalar _tmp328 = _tmp128 * _tmp327;
  const Scalar _tmp329 = _tmp191 * _tmp325;
  const Scalar _tmp330 = _tmp188 * _tmp302;
  const Scalar _tmp331 = _tmp86 + _tmp88;
  const Scalar _tmp332 = _tmp331 + _tmp84;
  const Scalar _tmp333 = _tmp141 + _tmp332;
  const Scalar _tmp334 = _tmp206 * _tmp317;
  const Scalar _tmp335 = _tmp206 * _tmp315;
  const Scalar _tmp336 = _tmp128 * _tmp335;
  const Scalar _tmp337 = _tmp142 + _tmp79;
  const Scalar _tmp338 = _tmp331 + _tmp337;
  const Scalar _tmp339 = _tmp204 * _tmp319;
  const Scalar _tmp340 = _tmp128 * _tmp330;
  const Scalar _tmp341 = Scalar(1.0) / (_tmp223);
  const Scalar _tmp342 = _tmp186 * _tmp300;
  const Scalar _tmp343 = _tmp341 * _tmp342;
  const Scalar _tmp344 = _tmp332 + _tmp79;
  const Scalar _tmp345 = _tmp189 * _tmp344;
  const Scalar _tmp346 = -_tmp189 * _tmp336 + _tmp200 * _tmp334 - _tmp200 * _tmp339 +
                         _tmp207 * _tmp338 - _tmp208 * _tmp340 - _tmp208 * _tmp345 +
                         _tmp209 * _tmp343 + _tmp257 * _tmp320;
  const Scalar _tmp347 = _tmp195 * _tmp323;
  const Scalar _tmp348 = -_tmp189 * _tmp326 + _tmp189 * _tmp327 + _tmp196 * _tmp330 -
                         _tmp196 * _tmp343 + _tmp329 - _tmp347;
  const Scalar _tmp349 = _tmp213 * _tmp348;
  const Scalar _tmp350 = _tmp196 * _tmp344;
  const Scalar _tmp351 = (-_tmp189 * _tmp320 + _tmp189 * _tmp335 + _tmp208 * _tmp330 -
                          _tmp208 * _tmp343 - _tmp334 + _tmp339) /
                         std::pow(_tmp212, Scalar(2));
  const Scalar _tmp352 = _tmp210 * _tmp351;
  const Scalar _tmp353 = -_tmp189 * _tmp328 - _tmp189 * _tmp350 - _tmp197 * _tmp330 +
                         _tmp197 * _tmp343 - _tmp198 * _tmp329 + _tmp198 * _tmp347 +
                         _tmp199 * _tmp333 - _tmp210 * _tmp349 + _tmp211 * _tmp352 -
                         _tmp214 * _tmp346 + _tmp257 * _tmp326;
  const Scalar _tmp354 = _tmp216 * _tmp351;
  const Scalar _tmp355 = Scalar(1.6799999999999999) * _tmp12 * _tmp70;
  const Scalar _tmp356 = _tmp355 + Scalar(1.6799999999999999) * _tmp85;
  const Scalar _tmp357 = _tmp170 * _tmp356;
  const Scalar _tmp358 = Scalar(0.83999999999999997) * _tmp16 * _tmp73;
  const Scalar _tmp359 = _tmp119 * _tmp21;
  const Scalar _tmp360 = Scalar(0.83999999999999997) * _tmp24 * _tmp70 + _tmp67 * _tmp98;
  const Scalar _tmp361 = (_tmp358 + _tmp359 + _tmp360) / std::pow(_tmp169, Scalar(2));
  const Scalar _tmp362 = _tmp173 * _tmp361;
  const Scalar _tmp363 = -_tmp128 * _tmp320 + _tmp200 * _tmp320 - _tmp200 * _tmp335 -
                         _tmp208 * _tmp338 + _tmp208 * _tmp344 + _tmp336;
  const Scalar _tmp364 = -_tmp128 * _tmp326 - _tmp174 * _tmp353 - _tmp196 * _tmp333 +
                         _tmp198 * _tmp326 - _tmp198 * _tmp327 + _tmp211 * _tmp354 -
                         _tmp214 * _tmp363 - _tmp215 * _tmp357 + _tmp215 * _tmp362 -
                         _tmp216 * _tmp349 + _tmp328 + _tmp350;
  const Scalar _tmp365 = _tmp364 / std::pow(_tmp217, Scalar(2));
  const Scalar _tmp366 = _tmp196 * _tmp365;
  const Scalar _tmp367 = _tmp211 * _tmp365;
  const Scalar _tmp368 = _tmp225 * _tmp300 * _tmp341;
  const Scalar _tmp369 = _tmp229 * _tmp368;
  const Scalar _tmp370 = _tmp232 * _tmp351;
  const Scalar _tmp371 = _tmp303 / (_tmp224 * std::sqrt(_tmp224));
  const Scalar _tmp372 = _tmp226 * _tmp302;
  const Scalar _tmp373 =
      _tmp230 * (_tmp132 * _tmp187 * _tmp371 + _tmp137 * _tmp372 - _tmp187 * _tmp372 +
                 _tmp226 * _tmp342 - _tmp227 * _tmp300 - _tmp228 * _tmp371);
  const Scalar _tmp374 = _tmp229 * _tmp304;
  const Scalar _tmp375 = -_tmp168 * _tmp320 + _tmp168 * _tmp335 - _tmp171 * _tmp334 +
                         _tmp171 * _tmp339 + _tmp202 * _tmp334 - _tmp207 * _tmp315 -
                         _tmp208 * _tmp369 + _tmp208 * _tmp373 + _tmp208 * _tmp374 -
                         _tmp231 * _tmp320 + _tmp231 * _tmp335;
  const Scalar _tmp376 = _tmp163 * _tmp329 - _tmp163 * _tmp347 - _tmp166 * _tmp326 +
                         _tmp166 * _tmp327 + _tmp193 * _tmp347 - _tmp196 * _tmp369 +
                         _tmp196 * _tmp373 + _tmp196 * _tmp374 - _tmp199 * _tmp321 +
                         _tmp211 * _tmp370 - _tmp214 * _tmp375 - _tmp231 * _tmp326 +
                         _tmp231 * _tmp327 - _tmp232 * _tmp349;
  const Scalar _tmp377 = _tmp376 / std::pow(_tmp236, Scalar(2));
  const Scalar _tmp378 = _tmp217 * _tmp377;
  const Scalar _tmp379 = _tmp238 * _tmp378;
  const Scalar _tmp380 = _tmp218 * _tmp376;
  const Scalar _tmp381 = _tmp236 * _tmp365;
  const Scalar _tmp382 = _tmp210 * _tmp213;
  const Scalar _tmp383 = _tmp233 * _tmp346;
  const Scalar _tmp384 = Scalar(1.0) * _tmp352;
  const Scalar _tmp385 = _tmp174 * _tmp383 - _tmp174 * _tmp384 + _tmp181 * _tmp356 * _tmp382 -
                         _tmp233 * _tmp363 - _tmp234 * _tmp362 + Scalar(1.0) * _tmp354;
  const Scalar _tmp386 = _tmp240 * (-_tmp233 * _tmp375 - _tmp235 * _tmp380 + _tmp235 * _tmp381 -
                                    _tmp237 * _tmp385 + Scalar(1.0) * _tmp370);
  const Scalar _tmp387 = _tmp239 * _tmp364;
  const Scalar _tmp388 = _tmp238 * _tmp387;
  const Scalar _tmp389 = -_tmp379 + _tmp385 + _tmp386 + _tmp388;
  const Scalar _tmp390 = _tmp211 * _tmp218;
  const Scalar _tmp391 = _tmp243 * _tmp367 - _tmp244 * _tmp348 - _tmp389 * _tmp390;
  const Scalar _tmp392 = _tmp196 * _tmp218;
  const Scalar _tmp393 = _tmp208 * _tmp351;
  const Scalar _tmp394 = _tmp160 * _tmp309;
  const Scalar _tmp395 = _tmp394 * fh1;
  const Scalar _tmp396 = Scalar(0.5) * _tmp239;
  const Scalar _tmp397 = Scalar(1.0) * _tmp377;
  const Scalar _tmp398 = _tmp211 * _tmp233 * _tmp377;
  const Scalar _tmp399 = _tmp277 * _tmp348;
  const Scalar _tmp400 = _tmp221 * _tmp283;
  const Scalar _tmp401 = _tmp213 * _tmp287;
  const Scalar _tmp402 = _tmp261 * _tmp378;
  const Scalar _tmp403 = _tmp213 * _tmp343;
  const Scalar _tmp404 = _tmp213 * _tmp330;
  const Scalar _tmp405 = -_tmp128 * _tmp343 - _tmp189 * _tmp352 + _tmp210 * _tmp404 +
                         _tmp258 * _tmp346 + _tmp340 - _tmp343 * _tmp382 + _tmp345;
  const Scalar _tmp406 = _tmp144 - _tmp174 * _tmp405 - _tmp189 * _tmp354 - _tmp216 * _tmp403 +
                         _tmp216 * _tmp404 + _tmp258 * _tmp363 - _tmp259 * _tmp357 +
                         _tmp259 * _tmp362;
  const Scalar _tmp407 = _tmp240 * (-_tmp189 * _tmp370 - _tmp232 * _tmp403 + _tmp232 * _tmp404 -
                                    _tmp237 * _tmp406 + _tmp258 * _tmp375 - _tmp260 * _tmp380 +
                                    _tmp260 * _tmp381 + _tmp369 - _tmp373 - _tmp374);
  const Scalar _tmp408 = _tmp261 * _tmp387;
  const Scalar _tmp409 = -_tmp402 + _tmp406 + _tmp407 + _tmp408;
  const Scalar _tmp410 =
      _tmp263 * _tmp367 - _tmp286 * _tmp348 - _tmp330 + _tmp343 - _tmp390 * _tmp409;
  const Scalar _tmp411 = _tmp247 * _tmp275;
  const Scalar _tmp412 = _tmp158 * _tmp311;
  const Scalar _tmp413 = _tmp412 * fh1;
  const Scalar _tmp414 = _tmp185 * _tmp365;
  const Scalar _tmp415 = _tmp214 * _tmp281;
  const Scalar _tmp416 = _tmp185 * _tmp390;
  const Scalar _tmp417 = _tmp160 * _tmp308;
  const Scalar _tmp418 = fh1 * (_tmp157 * _tmp417 - _tmp158 * _tmp394 + _tmp253 * _tmp309 -
                                _tmp254 * _tmp308 - _tmp312 * _tmp53 + _tmp412 * _tmp45);
  const Scalar _tmp419 = _tmp285 * _tmp289;
  const Scalar _tmp420 = _tmp417 * fh1;
  const Scalar _tmp421 =
      -_tmp230 * _tmp279 * _tmp418 -
      _tmp230 * _tmp285 *
          (-_tmp263 * _tmp366 - _tmp286 * _tmp326 + _tmp286 * _tmp327 - _tmp287 * _tmp393 +
           _tmp288 * _tmp410 - _tmp320 * _tmp401 + _tmp335 * _tmp401 + _tmp392 * _tmp409) -
      _tmp276 * (-_tmp243 * _tmp366 - _tmp244 * _tmp326 + _tmp244 * _tmp327 - _tmp273 * _tmp393 -
                 _tmp274 * _tmp320 + _tmp274 * _tmp335 + _tmp288 * _tmp391 + _tmp389 * _tmp392) -
      _tmp280 * (-_tmp193 * _tmp324 * _tmp396 - _tmp196 * _tmp397 +
                 _tmp202 * _tmp214 * _tmp318 * _tmp396 + _tmp208 * _tmp398 - _tmp208 * _tmp399 +
                 _tmp211 * _tmp250 * _tmp393 + _tmp250 * _tmp327 - _tmp278 * _tmp335) -
      _tmp284 * (-_tmp196 * _tmp414 + _tmp208 * _tmp214 * _tmp414 - _tmp281 * _tmp326 +
                 _tmp281 * _tmp327 - _tmp282 * _tmp349 + _tmp320 * _tmp415 - _tmp335 * _tmp415 +
                 _tmp393 * _tmp416) +
      _tmp290 * _tmp413 - _tmp290 * _tmp420 - _tmp304 * _tmp305 - _tmp304 * _tmp400 -
      _tmp304 * _tmp411 - _tmp304 * _tmp419 + _tmp305 * _tmp368 + _tmp306 * _tmp313 -
      _tmp306 * _tmp395 + _tmp368 * _tmp400 + _tmp368 * _tmp411 + _tmp368 * _tmp419;
  const Scalar _tmp422 = _tmp298 * _tmp421;
  const Scalar _tmp423 = Scalar(9.6622558468725703) * _tmp421;
  const Scalar _tmp424 = _tmp167 * _tmp357;
  const Scalar _tmp425 = _tmp167 * _tmp362;
  const Scalar _tmp426 = -_tmp355 - Scalar(1.6799999999999999) * _tmp87;
  const Scalar _tmp427 = _tmp174 * _tmp426;
  const Scalar _tmp428 =
      (-_tmp358 - _tmp359 + _tmp360 - _tmp424 + _tmp425 - _tmp427) / std::pow(_tmp176, Scalar(2));
  const Scalar _tmp429 = Scalar(1.0) * _tmp428;
  const Scalar _tmp430 = _tmp167 * _tmp361;
  const Scalar _tmp431 = _tmp167 * _tmp428;
  const Scalar _tmp432 = _tmp181 * _tmp431;
  const Scalar _tmp433 = _tmp177 * _tmp426;
  const Scalar _tmp434 = _tmp215 * _tmp365;
  const Scalar _tmp435 = -_tmp219 * _tmp409 - _tmp242 * _tmp402 + _tmp242 * _tmp407 +
                         _tmp242 * _tmp408 - _tmp262 * _tmp431 + _tmp262 * _tmp433 +
                         _tmp263 * _tmp434 - _tmp286 * _tmp353 + _tmp405;
  const Scalar _tmp436 = Scalar(1.0) * _tmp361;
  const Scalar _tmp437 = _tmp183 * _tmp428;
  const Scalar _tmp438 =
      _tmp164 * _tmp357 - _tmp164 * _tmp362 + Scalar(1.0) * _tmp174 * _tmp302 + _tmp300;
  const Scalar _tmp439 = _tmp177 * _tmp438;
  const Scalar _tmp440 = _tmp153 + _tmp167 * _tmp437 - _tmp167 * _tmp439 - _tmp184 * _tmp426 +
                         _tmp185 * _tmp434 - _tmp281 * _tmp353;
  const Scalar _tmp441 = -_tmp219 * _tmp389 - _tmp241 * _tmp431 + _tmp241 * _tmp433 -
                         _tmp242 * _tmp379 + _tmp242 * _tmp386 + _tmp242 * _tmp388 +
                         _tmp243 * _tmp434 - _tmp244 * _tmp353 - _tmp383 + _tmp384;
  const Scalar _tmp442 = _tmp178 * _tmp378;
  const Scalar _tmp443 = _tmp178 * _tmp387;
  const Scalar _tmp444 = _tmp240 * _tmp429;
  const Scalar _tmp445 = -_tmp167 * _tmp442 + _tmp167 * _tmp443 - _tmp167 * _tmp444 +
                         _tmp215 * _tmp397 + _tmp249 * _tmp426 - _tmp250 * _tmp353;
  const Scalar _tmp446 = Scalar(6.59232) * _tmp67;
  const Scalar _tmp447 = Scalar(6.59232) * _tmp70;
  const Scalar _tmp448 = Scalar(6.59232) * _tmp12;
  const Scalar _tmp449 = fh1 * (_tmp143 + _tmp331);
  const Scalar _tmp450 = Scalar(6.59232) * _tmp24;
  const Scalar _tmp451 = _tmp156 * _tmp312 - _tmp156 * _tmp394 + _tmp16 * _tmp446 -
                         _tmp161 * _tmp449 - _tmp21 * _tmp447 - _tmp309 * fv1 - _tmp448 * _tmp73 +
                         _tmp450 * _tmp76;
  const Scalar _tmp452 = _tmp178 * _tmp362;
  const Scalar _tmp453 =
      -_tmp175 * _tmp429 + _tmp178 * _tmp424 - _tmp178 * _tmp425 + _tmp178 * _tmp427;
  const Scalar _tmp454 = _tmp174 * _tmp429;
  const Scalar _tmp455 = Scalar(0.5) * _tmp310 * fh1;
  const Scalar _tmp456 = -_tmp156 * _tmp412 + _tmp156 * _tmp417 + _tmp16 * _tmp447 +
                         _tmp21 * _tmp446 + _tmp266 * _tmp449 + _tmp308 * fv1 + _tmp448 * _tmp76 +
                         _tmp450 * _tmp73;
  const Scalar _tmp457 =
      (-_tmp272 * _tmp422 +
       _tmp292 *
           (-_tmp157 * _tmp246 * _tmp455 - _tmp158 * _tmp265 * _tmp455 +
            Scalar(1.0) * _tmp162 *
                (_tmp178 * _tmp357 + _tmp180 * _tmp436 - _tmp181 * _tmp453 - _tmp452 - _tmp454) +
            _tmp182 * _tmp451 +
            _tmp222 *
                (-_tmp178 * _tmp438 - _tmp181 * _tmp440 + _tmp183 * _tmp429 + _tmp220 * _tmp436) +
            _tmp246 * _tmp267 * _tmp394 +
            _tmp248 * (-_tmp178 * _tmp379 + _tmp178 * _tmp386 + _tmp178 * _tmp388 -
                       _tmp181 * _tmp441 - _tmp241 * _tmp429 + _tmp245 * _tmp436) +
            Scalar(1.0) * _tmp252 * _tmp418 +
            _tmp256 * (-_tmp181 * _tmp445 + _tmp251 * _tmp436 - _tmp442 + _tmp443 - _tmp444) +
            _tmp265 * _tmp267 * _tmp417 +
            _tmp268 * (-_tmp178 * _tmp402 + _tmp178 * _tmp407 + _tmp178 * _tmp408 -
                       _tmp181 * _tmp435 - _tmp262 * _tmp429 + _tmp264 * _tmp436) +
            Scalar(1.0) * _tmp269 * (-_tmp178 * _tmp430 + _tmp270 * _tmp426 + _tmp429 - _tmp432) +
            _tmp271 * _tmp456)) /
      std::sqrt(Scalar(std::pow(_tmp272, Scalar(2)) * _tmp298 + 1));
  const Scalar _tmp458 = Scalar(1.0) * _tmp293;
  const Scalar _tmp459 = _tmp170 * _tmp245;
  const Scalar _tmp460 = _tmp170 * _tmp255;
  const Scalar _tmp461 = _tmp162 * _tmp170;
  const Scalar _tmp462 = _tmp170 * _tmp264;
  const Scalar _tmp463 = _tmp170 * _tmp221;
  const Scalar _tmp464 = _tmp178 * _tmp269;
  const Scalar _tmp465 = _tmp170 * _tmp464;
  const Scalar _tmp466 = -_tmp167 * _tmp465 + _tmp180 * _tmp461 + _tmp220 * _tmp463 +
                         _tmp247 * _tmp459 + _tmp251 * _tmp460 + _tmp285 * _tmp462;
  const Scalar _tmp467 = _tmp221 * _tmp281;
  const Scalar _tmp468 =
      -_tmp214 * _tmp467 + _tmp247 * _tmp274 - _tmp255 * _tmp278 + _tmp285 * _tmp401;
  const Scalar _tmp469 = Scalar(1.0) / (_tmp468);
  const Scalar _tmp470 = std::asinh(_tmp466 * _tmp469);
  const Scalar _tmp471 = Scalar(1.0) * _tmp470;
  const Scalar _tmp472 = std::pow(_tmp468, Scalar(-2));
  const Scalar _tmp473 = _tmp178 * _tmp456;
  const Scalar _tmp474 = _tmp250 * _tmp255;
  const Scalar _tmp475 = _tmp221 * _tmp414;
  const Scalar _tmp476 = _tmp211 * _tmp351 * _tmp474 + _tmp213 * _tmp247 * _tmp391 +
                         _tmp213 * _tmp285 * _tmp410 + _tmp214 * _tmp475 +
                         _tmp221 * _tmp351 * _tmp416 - _tmp247 * _tmp273 * _tmp351 +
                         _tmp255 * _tmp398 - _tmp255 * _tmp399 - _tmp274 * _tmp313 +
                         _tmp274 * _tmp395 - _tmp278 * _tmp418 - _tmp285 * _tmp287 * _tmp351 -
                         _tmp349 * _tmp467 - _tmp401 * _tmp413 + _tmp401 * _tmp420;
  const Scalar _tmp477 = _tmp472 * _tmp476;
  const Scalar _tmp478 =
      (-_tmp466 * _tmp477 +
       _tmp469 * (-_tmp162 * _tmp180 * _tmp361 - _tmp167 * _tmp170 * _tmp473 +
                  _tmp170 * _tmp180 * _tmp451 + _tmp170 * _tmp247 * _tmp441 +
                  _tmp170 * _tmp251 * _tmp418 + _tmp170 * _tmp285 * _tmp435 -
                  _tmp220 * _tmp221 * _tmp361 - _tmp245 * _tmp247 * _tmp361 -
                  _tmp251 * _tmp255 * _tmp361 - _tmp264 * _tmp285 * _tmp361 + _tmp269 * _tmp432 -
                  _tmp313 * _tmp459 + _tmp395 * _tmp459 - _tmp413 * _tmp462 + _tmp420 * _tmp462 -
                  _tmp426 * _tmp465 + _tmp430 * _tmp464 + _tmp440 * _tmp463 + _tmp445 * _tmp460 +
                  _tmp453 * _tmp461)) /
      std::sqrt(Scalar(std::pow(_tmp466, Scalar(2)) * _tmp472 + 1));
  const Scalar _tmp479 = -_tmp203 + Scalar(-8.3888750099999996);
  const Scalar _tmp480 = Scalar(2.5202214700000001) - _tmp201;
  const Scalar _tmp481 = std::pow(_tmp479, Scalar(2)) + std::pow(_tmp480, Scalar(2));
  const Scalar _tmp482 = std::sqrt(_tmp481);
  const Scalar _tmp483 = Scalar(9.6622558468725703) * _tmp468;
  const Scalar _tmp484 = -_tmp470 * _tmp483 - _tmp482;
  const Scalar _tmp485 = Scalar(0.1034955) * _tmp469;
  const Scalar _tmp486 = _tmp484 * _tmp485;
  const Scalar _tmp487 = 2 * _tmp479 * (_tmp116 + _tmp322) + 2 * _tmp480 * (_tmp101 + _tmp149);
  const Scalar _tmp488 = Scalar(9.6622558468725703) * _tmp476;
  const Scalar _tmp489 = -_tmp200 - p_init2 + Scalar(8.4693136199999994);
  const Scalar _tmp490 = -_tmp198 - p_init2 + Scalar(8.3700199099999999);
  const Scalar _tmp491 = -_tmp192 + Scalar(-1.79662371);
  const Scalar _tmp492 = Scalar(4.8333311099999996) - _tmp190;
  const Scalar _tmp493 = 2 * _tmp491 * (_tmp111 + _tmp314) + 2 * _tmp492 * (_tmp116 + _tmp316);
  const Scalar _tmp494 = std::pow(_tmp491, Scalar(2)) + std::pow(_tmp492, Scalar(2));
  const Scalar _tmp495 = _tmp177 * _tmp285;
  const Scalar _tmp496 = _tmp177 * _tmp247;
  const Scalar _tmp497 = -_tmp162 * _tmp179 + _tmp184 * _tmp221 - _tmp241 * _tmp496 -
                         _tmp249 * _tmp255 - _tmp262 * _tmp495 + _tmp464;
  const Scalar _tmp498 = _tmp244 * _tmp247 + _tmp285 * _tmp286 + _tmp467 + _tmp474;
  const Scalar _tmp499 = Scalar(1.0) / (_tmp498);
  const Scalar _tmp500 = std::asinh(_tmp497 * _tmp499);
  const Scalar _tmp501 = Scalar(1.0) * _tmp500;
  const Scalar _tmp502 = Scalar(9.6622558468725703) * _tmp498;
  const Scalar _tmp503 = std::sqrt(_tmp494);
  const Scalar _tmp504 = -_tmp500 * _tmp502 - _tmp503;
  const Scalar _tmp505 = Scalar(0.1034955) * _tmp499;
  const Scalar _tmp506 = _tmp504 * _tmp505;
  const Scalar _tmp507 = _tmp218 * _tmp247 * _tmp389 + _tmp218 * _tmp285 * _tmp409 -
                         _tmp243 * _tmp247 * _tmp365 - _tmp244 * _tmp313 + _tmp244 * _tmp395 +
                         _tmp250 * _tmp418 - _tmp256 * _tmp377 - _tmp263 * _tmp285 * _tmp365 -
                         _tmp286 * _tmp413 + _tmp286 * _tmp420 - _tmp475;
  const Scalar _tmp508 = Scalar(9.6622558468725703) * _tmp507;
  const Scalar _tmp509 = std::pow(_tmp498, Scalar(-2));
  const Scalar _tmp510 = _tmp507 * _tmp509;
  const Scalar _tmp511 = _tmp177 * _tmp241;
  const Scalar _tmp512 = _tmp177 * _tmp262;
  const Scalar _tmp513 =
      (-_tmp497 * _tmp510 +
       _tmp499 * (_tmp162 * _tmp452 + _tmp162 * _tmp454 - _tmp178 * _tmp356 * _tmp461 -
                  _tmp179 * _tmp451 - _tmp221 * _tmp437 + _tmp221 * _tmp439 +
                  _tmp241 * _tmp247 * _tmp428 - _tmp249 * _tmp418 + _tmp255 * _tmp442 -
                  _tmp255 * _tmp443 + _tmp255 * _tmp444 + _tmp262 * _tmp285 * _tmp428 -
                  _tmp269 * _tmp429 + _tmp313 * _tmp511 + _tmp379 * _tmp496 - _tmp386 * _tmp496 -
                  _tmp388 * _tmp496 - _tmp395 * _tmp511 + _tmp402 * _tmp495 - _tmp407 * _tmp495 -
                  _tmp408 * _tmp495 + _tmp413 * _tmp512 - _tmp420 * _tmp512 + _tmp473)) /
      std::sqrt(Scalar(std::pow(_tmp497, Scalar(2)) * _tmp509 + 1));

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) =
      Scalar(0.5) * _tmp123 *
          std::cosh(Scalar(0.1034955) * _tmp125 *
                    (-_tmp124 - Scalar(9.6622558468725703) * fh1 * std::asinh(_tmp125 * fv1))) /
          _tmp124 -
      Scalar(1) / Scalar(2) * (_tmp123 + 2 * _tmp36 * (_tmp79 + _tmp90)) /
          std::sqrt(Scalar(std::pow(_tmp36, Scalar(2)) + _tmp56));
  _res(1, 0) = _tmp294 * (-Scalar(1.0) * _tmp457 * std::cosh(_tmp458) -
                          (-Scalar(0.1034955) * _tmp295 * _tmp422 +
                           _tmp296 * (-Scalar(1) / Scalar(2) * _tmp154 / _tmp155 -
                                      _tmp293 * _tmp423 - _tmp294 * _tmp457)) *
                              std::cosh(_tmp297)) +
               _tmp423 * (-std::sinh(_tmp297) - std::sinh(_tmp458)) -
               Scalar(1) / Scalar(2) * (2 * _tmp129 * _tmp144 + _tmp154) /
                   std::sqrt(Scalar(std::pow(_tmp129, Scalar(2)) + _tmp140));
  _res(2, 0) = _tmp483 * (-Scalar(1.0) * _tmp478 * std::cosh(_tmp471) -
                          (-Scalar(0.1034955) * _tmp477 * _tmp484 +
                           _tmp485 * (-_tmp470 * _tmp488 - _tmp478 * _tmp483 -
                                      Scalar(1) / Scalar(2) * _tmp487 / _tmp482)) *
                              std::cosh(_tmp486)) +
               _tmp488 * (-std::sinh(_tmp471) - std::sinh(_tmp486)) -
               Scalar(1) / Scalar(2) * (_tmp487 + 2 * _tmp489 * (_tmp141 + _tmp90)) /
                   std::sqrt(Scalar(_tmp481 + std::pow(_tmp489, Scalar(2))));
  _res(3, 0) = _tmp502 * (-Scalar(1.0) * _tmp513 * std::cosh(_tmp501) -
                          (-Scalar(0.1034955) * _tmp504 * _tmp510 +
                           _tmp505 * (-Scalar(1) / Scalar(2) * _tmp493 / _tmp503 -
                                      _tmp500 * _tmp508 - _tmp502 * _tmp513)) *
                              std::cosh(_tmp506)) +
               _tmp508 * (-std::sinh(_tmp501) - std::sinh(_tmp506)) -
               Scalar(1) / Scalar(2) * (2 * _tmp490 * (_tmp337 + _tmp89) + _tmp493) /
                   std::sqrt(Scalar(std::pow(_tmp490, Scalar(2)) + _tmp494));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
