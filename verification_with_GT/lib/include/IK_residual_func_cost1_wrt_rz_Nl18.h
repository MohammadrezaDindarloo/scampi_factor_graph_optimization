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
 * Symbolic function: IK_residual_func_cost1_wrt_rz_Nl18
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost1WrtRzNl18(
    const Scalar fh1, const Scalar fv1, const Scalar rx, const Scalar ry, const Scalar rz,
    const Scalar p_init0, const Scalar p_init1, const Scalar p_init2, const Scalar rot_init_x,
    const Scalar rot_init_y, const Scalar rot_init_z, const Scalar rot_init_w,
    const Scalar epsilon) {
  // Total ops: 1616

  // Unused inputs
  (void)p_init2;
  (void)epsilon;

  // Input arrays

  // Intermediate terms (497)
  const Scalar _tmp0 = std::pow(rz, Scalar(2));
  const Scalar _tmp1 = _tmp0 + std::pow(rx, Scalar(2)) + std::pow(ry, Scalar(2));
  const Scalar _tmp2 = std::sqrt(_tmp1);
  const Scalar _tmp3 = (Scalar(1) / Scalar(2)) * _tmp2;
  const Scalar _tmp4 = std::cos(_tmp3);
  const Scalar _tmp5 = _tmp4 * rot_init_y;
  const Scalar _tmp6 = std::sin(_tmp3);
  const Scalar _tmp7 = _tmp6 / _tmp2;
  const Scalar _tmp8 = _tmp7 * rot_init_w;
  const Scalar _tmp9 = _tmp7 * rot_init_z;
  const Scalar _tmp10 = _tmp7 * rot_init_x;
  const Scalar _tmp11 = _tmp10 * rz;
  const Scalar _tmp12 = -_tmp11 + _tmp5 + _tmp8 * ry + _tmp9 * rx;
  const Scalar _tmp13 = _tmp4 * rot_init_x;
  const Scalar _tmp14 = _tmp7 * rot_init_y;
  const Scalar _tmp15 = _tmp14 * rz;
  const Scalar _tmp16 = _tmp13 + _tmp15 + _tmp8 * rx - _tmp9 * ry;
  const Scalar _tmp17 = 2 * _tmp12 * _tmp16;
  const Scalar _tmp18 = _tmp4 * rot_init_z;
  const Scalar _tmp19 = _tmp8 * rz;
  const Scalar _tmp20 = _tmp10 * ry - _tmp14 * rx + _tmp18 + _tmp19;
  const Scalar _tmp21 = _tmp4 * rot_init_w;
  const Scalar _tmp22 = _tmp9 * rz;
  const Scalar _tmp23 = -_tmp10 * rx - _tmp14 * ry + _tmp21 - _tmp22;
  const Scalar _tmp24 = 2 * _tmp23;
  const Scalar _tmp25 = _tmp20 * _tmp24;
  const Scalar _tmp26 = Scalar(0.20999999999999999) * _tmp17 + Scalar(0.20999999999999999) * _tmp25;
  const Scalar _tmp27 = -_tmp26;
  const Scalar _tmp28 = 2 * _tmp20;
  const Scalar _tmp29 = _tmp12 * _tmp28;
  const Scalar _tmp30 = _tmp16 * _tmp24;
  const Scalar _tmp31 = _tmp29 - _tmp30;
  const Scalar _tmp32 = -Scalar(0.010999999999999999) * _tmp31;
  const Scalar _tmp33 = -2 * std::pow(_tmp16, Scalar(2));
  const Scalar _tmp34 = -2 * std::pow(_tmp20, Scalar(2));
  const Scalar _tmp35 = Scalar(0.20999999999999999) * _tmp33 +
                        Scalar(0.20999999999999999) * _tmp34 + Scalar(0.20999999999999999);
  const Scalar _tmp36 = _tmp32 + _tmp35;
  const Scalar _tmp37 = _tmp27 + _tmp36;
  const Scalar _tmp38 = _tmp37 + p_init1;
  const Scalar _tmp39 = Scalar(4.8333311099999996) - _tmp38;
  const Scalar _tmp40 = Scalar(0.20999999999999999) * _tmp17 - Scalar(0.20999999999999999) * _tmp25;
  const Scalar _tmp41 = 1 - 2 * std::pow(_tmp12, Scalar(2));
  const Scalar _tmp42 = Scalar(0.20999999999999999) * _tmp34 + Scalar(0.20999999999999999) * _tmp41;
  const Scalar _tmp43 = _tmp16 * _tmp28;
  const Scalar _tmp44 = _tmp12 * _tmp24;
  const Scalar _tmp45 = _tmp43 + _tmp44;
  const Scalar _tmp46 = -Scalar(0.010999999999999999) * _tmp45;
  const Scalar _tmp47 = -_tmp42 + _tmp46;
  const Scalar _tmp48 = _tmp40 + _tmp47;
  const Scalar _tmp49 = _tmp48 + p_init0;
  const Scalar _tmp50 = -_tmp49 + Scalar(-1.79662371);
  const Scalar _tmp51 =
      std::sqrt(Scalar(std::pow(_tmp39, Scalar(2)) + std::pow(_tmp50, Scalar(2))));
  const Scalar _tmp52 = Scalar(1.0) / (fh1);
  const Scalar _tmp53 = (Scalar(1) / Scalar(2)) / _tmp1;
  const Scalar _tmp54 = _tmp0 * _tmp53;
  const Scalar _tmp55 = _tmp53 * rz;
  const Scalar _tmp56 = _tmp55 * ry;
  const Scalar _tmp57 = _tmp55 * rx;
  const Scalar _tmp58 = _tmp6 / (_tmp1 * std::sqrt(_tmp1));
  const Scalar _tmp59 = _tmp0 * _tmp58;
  const Scalar _tmp60 = _tmp58 * rz;
  const Scalar _tmp61 = _tmp60 * rx;
  const Scalar _tmp62 = _tmp60 * ry;
  const Scalar _tmp63 = -_tmp13 * _tmp57 - _tmp18 * _tmp54 - Scalar(1) / Scalar(2) * _tmp19 -
                        _tmp5 * _tmp56 + _tmp59 * rot_init_z + _tmp61 * rot_init_x +
                        _tmp62 * rot_init_y - _tmp9;
  const Scalar _tmp64 = Scalar(0.021999999999999999) * _tmp16;
  const Scalar _tmp65 = _tmp63 * _tmp64;
  const Scalar _tmp66 = -_tmp10 - _tmp13 * _tmp54 - Scalar(1) / Scalar(2) * _tmp15 +
                        _tmp18 * _tmp57 + _tmp21 * _tmp56 + _tmp59 * rot_init_x -
                        _tmp61 * rot_init_z - _tmp62 * rot_init_w;
  const Scalar _tmp67 = Scalar(0.021999999999999999) * _tmp20;
  const Scalar _tmp68 = _tmp66 * _tmp67;
  const Scalar _tmp69 = _tmp13 * _tmp56 + _tmp21 * _tmp54 - Scalar(1) / Scalar(2) * _tmp22 -
                        _tmp5 * _tmp57 - _tmp59 * rot_init_w + _tmp61 * rot_init_y -
                        _tmp62 * rot_init_x + _tmp8;
  const Scalar _tmp70 = Scalar(0.021999999999999999) * _tmp12;
  const Scalar _tmp71 = _tmp69 * _tmp70;
  const Scalar _tmp72 = -Scalar(1) / Scalar(2) * _tmp11 + _tmp14 - _tmp18 * _tmp56 +
                        _tmp21 * _tmp57 + _tmp5 * _tmp54 - _tmp59 * rot_init_y -
                        _tmp61 * rot_init_w + _tmp62 * rot_init_z;
  const Scalar _tmp73 = Scalar(0.021999999999999999) * _tmp23;
  const Scalar _tmp74 = _tmp72 * _tmp73;
  const Scalar _tmp75 = -_tmp65 + _tmp68 + _tmp71 - _tmp74;
  const Scalar _tmp76 = Scalar(0.83999999999999997) * _tmp69;
  const Scalar _tmp77 = _tmp20 * _tmp76;
  const Scalar _tmp78 = Scalar(0.83999999999999997) * _tmp72;
  const Scalar _tmp79 = _tmp16 * _tmp78;
  const Scalar _tmp80 = _tmp77 + _tmp79;
  const Scalar _tmp81 = _tmp20 * _tmp63;
  const Scalar _tmp82 = Scalar(0.41999999999999998) * _tmp81;
  const Scalar _tmp83 = Scalar(0.41999999999999998) * _tmp23;
  const Scalar _tmp84 = _tmp69 * _tmp83;
  const Scalar _tmp85 = _tmp82 + _tmp84;
  const Scalar _tmp86 = Scalar(0.41999999999999998) * _tmp12;
  const Scalar _tmp87 = _tmp72 * _tmp86;
  const Scalar _tmp88 = Scalar(0.41999999999999998) * _tmp16;
  const Scalar _tmp89 = _tmp66 * _tmp88;
  const Scalar _tmp90 = _tmp87 + _tmp89;
  const Scalar _tmp91 = _tmp85 + _tmp90;
  const Scalar _tmp92 = _tmp80 + _tmp91;
  const Scalar _tmp93 = Scalar(0.83999999999999997) * _tmp66;
  const Scalar _tmp94 = _tmp12 * _tmp93;
  const Scalar _tmp95 = -_tmp77;
  const Scalar _tmp96 = -_tmp94 + _tmp95;
  const Scalar _tmp97 = -_tmp87 - _tmp89;
  const Scalar _tmp98 = _tmp85 + _tmp97;
  const Scalar _tmp99 = _tmp96 + _tmp98;
  const Scalar _tmp100 = _tmp63 * _tmp70;
  const Scalar _tmp101 = _tmp67 * _tmp72;
  const Scalar _tmp102 = _tmp64 * _tmp69;
  const Scalar _tmp103 = _tmp66 * _tmp73;
  const Scalar _tmp104 = _tmp100 + _tmp101 + _tmp102 + _tmp103;
  const Scalar _tmp105 = _tmp63 * _tmp86;
  const Scalar _tmp106 = Scalar(0.41999999999999998) * _tmp20;
  const Scalar _tmp107 = _tmp106 * _tmp72;
  const Scalar _tmp108 = _tmp69 * _tmp88;
  const Scalar _tmp109 = _tmp66 * _tmp83;
  const Scalar _tmp110 = _tmp105 - _tmp107 - _tmp108 + _tmp109;
  const Scalar _tmp111 = _tmp63 * _tmp88;
  const Scalar _tmp112 = _tmp106 * _tmp66;
  const Scalar _tmp113 = _tmp69 * _tmp86;
  const Scalar _tmp114 = _tmp72 * _tmp83;
  const Scalar _tmp115 = _tmp16 * _tmp72;
  const Scalar _tmp116 = Scalar(0.043999999999999997) * _tmp115;
  const Scalar _tmp117 = _tmp12 * _tmp66;
  const Scalar _tmp118 = Scalar(0.043999999999999997) * _tmp117;
  const Scalar _tmp119 = _tmp116 + _tmp118;
  const Scalar _tmp120 = _tmp111 + _tmp112 + _tmp113 + _tmp114 + _tmp119;
  const Scalar _tmp121 = _tmp110 + _tmp120;
  const Scalar _tmp122 = _tmp38 + Scalar(-4.8333311099999996);
  const Scalar _tmp123 = _tmp49 + Scalar(1.79662371);
  const Scalar _tmp124 = std::pow(_tmp122, Scalar(2)) + std::pow(_tmp123, Scalar(2));
  const Scalar _tmp125 = std::pow(_tmp124, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp126 = _tmp122 * _tmp125;
  const Scalar _tmp127 = _tmp126 * fh1;
  const Scalar _tmp128 =
      Scalar(0.20999999999999999) * _tmp29 + Scalar(0.20999999999999999) * _tmp30;
  const Scalar _tmp129 = -_tmp128;
  const Scalar _tmp130 =
      -Scalar(0.010999999999999999) * _tmp33 - Scalar(0.010999999999999999) * _tmp41;
  const Scalar _tmp131 =
      Scalar(0.20999999999999999) * _tmp43 - Scalar(0.20999999999999999) * _tmp44;
  const Scalar _tmp132 = _tmp130 + _tmp131;
  const Scalar _tmp133 = _tmp129 + _tmp132;
  const Scalar _tmp134 = -_tmp40;
  const Scalar _tmp135 = _tmp42 + _tmp46;
  const Scalar _tmp136 = _tmp134 + _tmp135;
  const Scalar _tmp137 = _tmp136 + p_init0;
  const Scalar _tmp138 = _tmp137 + Scalar(-2.5202214700000001);
  const Scalar _tmp139 = _tmp32 - _tmp35;
  const Scalar _tmp140 = _tmp139 + _tmp26;
  const Scalar _tmp141 = _tmp140 + p_init1;
  const Scalar _tmp142 = _tmp141 + Scalar(8.3888750099999996);
  const Scalar _tmp143 = std::pow(_tmp138, Scalar(2)) + std::pow(_tmp142, Scalar(2));
  const Scalar _tmp144 = std::pow(_tmp143, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp145 = _tmp138 * _tmp144;
  const Scalar _tmp146 = _tmp130 - _tmp131;
  const Scalar _tmp147 = _tmp129 + _tmp146;
  const Scalar _tmp148 = _tmp145 * _tmp147;
  const Scalar _tmp149 = -_tmp133 * _tmp145 + _tmp148;
  const Scalar _tmp150 = _tmp139 + _tmp27;
  const Scalar _tmp151 = _tmp150 + p_init1;
  const Scalar _tmp152 = _tmp151 + Scalar(8.3196563700000006);
  const Scalar _tmp153 = _tmp134 + _tmp47;
  const Scalar _tmp154 = _tmp153 + p_init0;
  const Scalar _tmp155 = _tmp154 + Scalar(1.9874742000000001);
  const Scalar _tmp156 = Scalar(1.0) / (_tmp155);
  const Scalar _tmp157 = _tmp152 * _tmp156;
  const Scalar _tmp158 = _tmp26 + _tmp36;
  const Scalar _tmp159 = _tmp158 + p_init1;
  const Scalar _tmp160 = _tmp159 + Scalar(-4.7752063900000001);
  const Scalar _tmp161 = _tmp135 + _tmp40;
  const Scalar _tmp162 = _tmp161 + p_init0;
  const Scalar _tmp163 = _tmp162 + Scalar(-2.71799795);
  const Scalar _tmp164 = std::pow(_tmp160, Scalar(2)) + std::pow(_tmp163, Scalar(2));
  const Scalar _tmp165 = std::pow(_tmp164, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp166 = _tmp163 * _tmp165;
  const Scalar _tmp167 = _tmp160 * _tmp165;
  const Scalar _tmp168 = _tmp157 * _tmp166 - _tmp167;
  const Scalar _tmp169 = _tmp142 * _tmp144;
  const Scalar _tmp170 = _tmp145 * _tmp157 - _tmp169;
  const Scalar _tmp171 = Scalar(1.0) / (_tmp170);
  const Scalar _tmp172 = _tmp168 * _tmp171;
  const Scalar _tmp173 = _tmp147 * _tmp166;
  const Scalar _tmp174 = _tmp128 + _tmp132;
  const Scalar _tmp175 = _tmp133 * _tmp169 - _tmp148 * _tmp157;
  const Scalar _tmp176 = -_tmp157 * _tmp173 + _tmp167 * _tmp174 - _tmp172 * _tmp175;
  const Scalar _tmp177 = Scalar(1.0) * _tmp150;
  const Scalar _tmp178 = -_tmp177;
  const Scalar _tmp179 = _tmp140 + _tmp178;
  const Scalar _tmp180 = Scalar(1.0) / (_tmp179);
  const Scalar _tmp181 = Scalar(1.0) * _tmp153;
  const Scalar _tmp182 = -_tmp136 + _tmp181;
  const Scalar _tmp183 = _tmp180 * _tmp182;
  const Scalar _tmp184 = -_tmp149 * _tmp172 - _tmp166 * _tmp174 + _tmp173 - _tmp176 * _tmp183;
  const Scalar _tmp185 = Scalar(1.0) / (_tmp184);
  const Scalar _tmp186 = std::pow(_tmp155, Scalar(2));
  const Scalar _tmp187 = std::pow(_tmp152, Scalar(2)) + _tmp186;
  const Scalar _tmp188 = std::sqrt(_tmp187);
  const Scalar _tmp189 = Scalar(1.0) / (_tmp188);
  const Scalar _tmp190 = _tmp150 * _tmp189;
  const Scalar _tmp191 = _tmp152 * _tmp153;
  const Scalar _tmp192 = -_tmp155 * _tmp190 + _tmp189 * _tmp191;
  const Scalar _tmp193 = _tmp156 * _tmp188;
  const Scalar _tmp194 = _tmp192 * _tmp193;
  const Scalar _tmp195 = -_tmp136 * _tmp169 + _tmp140 * _tmp145 + _tmp145 * _tmp194;
  const Scalar _tmp196 = Scalar(1.0) * _tmp171;
  const Scalar _tmp197 = _tmp175 * _tmp196;
  const Scalar _tmp198 = -_tmp149 * _tmp196 + _tmp183 * _tmp197;
  const Scalar _tmp199 =
      _tmp158 * _tmp166 - _tmp161 * _tmp167 + _tmp166 * _tmp194 - _tmp172 * _tmp195;
  const Scalar _tmp200 = _tmp185 * _tmp199;
  const Scalar _tmp201 = -_tmp195 * _tmp196 - _tmp198 * _tmp200;
  const Scalar _tmp202 = Scalar(1.0) / (_tmp199);
  const Scalar _tmp203 = _tmp184 * _tmp202;
  const Scalar _tmp204 = _tmp201 * _tmp203;
  const Scalar _tmp205 = _tmp198 + _tmp204;
  const Scalar _tmp206 = _tmp185 * _tmp205;
  const Scalar _tmp207 = _tmp168 * _tmp185;
  const Scalar _tmp208 = -_tmp205 * _tmp207 + Scalar(1.0);
  const Scalar _tmp209 = _tmp171 * _tmp208;
  const Scalar _tmp210 = _tmp145 * _tmp209 + _tmp166 * _tmp206;
  const Scalar _tmp211 = _tmp193 * _tmp210;
  const Scalar _tmp212 = _tmp123 * _tmp125;
  const Scalar _tmp213 = _tmp212 * fh1;
  const Scalar _tmp214 = _tmp147 * _tmp157;
  const Scalar _tmp215 = _tmp157 * _tmp171;
  const Scalar _tmp216 = _tmp175 * _tmp215 + _tmp214;
  const Scalar _tmp217 = -_tmp147 + _tmp149 * _tmp215 - _tmp183 * _tmp216;
  const Scalar _tmp218 = -_tmp194 + _tmp195 * _tmp215 - _tmp200 * _tmp217;
  const Scalar _tmp219 = _tmp203 * _tmp218;
  const Scalar _tmp220 = _tmp217 + _tmp219;
  const Scalar _tmp221 = -_tmp157 - _tmp207 * _tmp220;
  const Scalar _tmp222 = _tmp145 * _tmp171;
  const Scalar _tmp223 = _tmp185 * _tmp220;
  const Scalar _tmp224 = _tmp166 * _tmp223 + _tmp221 * _tmp222 + Scalar(1.0);
  const Scalar _tmp225 = _tmp193 * _tmp224;
  const Scalar _tmp226 = _tmp125 * _tmp48;
  const Scalar _tmp227 = _tmp125 * _tmp37;
  const Scalar _tmp228 = fh1 * (_tmp122 * _tmp226 - _tmp123 * _tmp227);
  const Scalar _tmp229 = Scalar(1.0) * _tmp202;
  const Scalar _tmp230 = _tmp172 * _tmp229;
  const Scalar _tmp231 = -_tmp145 * _tmp230 + _tmp166 * _tmp229;
  const Scalar _tmp232 = _tmp193 * _tmp231;
  const Scalar _tmp233 = _tmp177 * _tmp183 + _tmp181;
  const Scalar _tmp234 = 0;
  const Scalar _tmp235 = _tmp185 * _tmp234;
  const Scalar _tmp236 = _tmp172 * _tmp235;
  const Scalar _tmp237 = -_tmp145 * _tmp236 + _tmp166 * _tmp235;
  const Scalar _tmp238 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp239 = _tmp193 * _tmp238;
  const Scalar _tmp240 =
      -_tmp127 * _tmp211 - _tmp213 * _tmp225 - _tmp228 * _tmp232 - _tmp237 * _tmp239;
  const Scalar _tmp241 = Scalar(1.0) / (_tmp240);
  const Scalar _tmp242 = _tmp158 + _tmp178;
  const Scalar _tmp243 = _tmp183 * _tmp242;
  const Scalar _tmp244 = -_tmp161 + _tmp181 - _tmp243;
  const Scalar _tmp245 = Scalar(1.0) / (_tmp244);
  const Scalar _tmp246 = _tmp233 * _tmp245;
  const Scalar _tmp247 = -_tmp176 * _tmp235 + _tmp178 - _tmp242 * _tmp246;
  const Scalar _tmp248 = Scalar(1.0) * _tmp180;
  const Scalar _tmp249 = Scalar(1.0) * _tmp245;
  const Scalar _tmp250 = Scalar(1.0) * _tmp238;
  const Scalar _tmp251 = _tmp242 * _tmp245;
  const Scalar _tmp252 = -_tmp176 * _tmp206 - _tmp197 + _tmp204 * _tmp251;
  const Scalar _tmp253 = _tmp204 * _tmp249 - _tmp248 * _tmp252;
  const Scalar _tmp254 = Scalar(1.0) * _tmp253;
  const Scalar _tmp255 = fh1 * (_tmp128 + _tmp146);
  const Scalar _tmp256 = -_tmp126 * _tmp255 - Scalar(3.29616) * _tmp31 - _tmp37 * fv1;
  const Scalar _tmp257 = _tmp243 * _tmp249 + Scalar(1.0);
  const Scalar _tmp258 = _tmp183 * _tmp249;
  const Scalar _tmp259 = -Scalar(1.0) * _tmp248 * _tmp257 + Scalar(1.0) * _tmp258;
  const Scalar _tmp260 = _tmp180 * _tmp249;
  const Scalar _tmp261 = _tmp242 * _tmp260 - _tmp249;
  const Scalar _tmp262 = _tmp212 * _tmp255 + Scalar(3.29616) * _tmp45 + _tmp48 * fv1;
  const Scalar _tmp263 = Scalar(1.0) * _tmp262;
  const Scalar _tmp264 = _tmp203 * _tmp249;
  const Scalar _tmp265 = -_tmp176 * _tmp229 + _tmp242 * _tmp264;
  const Scalar _tmp266 = -Scalar(1.0) * _tmp248 * _tmp265 + Scalar(1.0) * _tmp264;
  const Scalar _tmp267 = _tmp176 * _tmp185;
  const Scalar _tmp268 = _tmp216 + _tmp219 * _tmp251 - _tmp220 * _tmp267;
  const Scalar _tmp269 = _tmp219 * _tmp249 - _tmp248 * _tmp268;
  const Scalar _tmp270 = Scalar(1.0) * _tmp213;
  const Scalar _tmp271 = _tmp127 * _tmp254 + _tmp228 * _tmp266 +
                         _tmp250 * (-_tmp233 * _tmp249 - _tmp247 * _tmp248 + Scalar(1.0)) +
                         _tmp256 * _tmp259 + _tmp261 * _tmp263 + _tmp269 * _tmp270;
  const Scalar _tmp272 = std::asinh(_tmp241 * _tmp271);
  const Scalar _tmp273 = Scalar(1.0) * _tmp272;
  const Scalar _tmp274 = Scalar(9.6622558468725703) * _tmp240;
  const Scalar _tmp275 = -_tmp151 + Scalar(-8.3196563700000006);
  const Scalar _tmp276 = -_tmp154 + Scalar(-1.9874742000000001);
  const Scalar _tmp277 =
      std::sqrt(Scalar(std::pow(_tmp275, Scalar(2)) + std::pow(_tmp276, Scalar(2))));
  const Scalar _tmp278 = -_tmp272 * _tmp274 - _tmp277;
  const Scalar _tmp279 = Scalar(0.1034955) * _tmp241;
  const Scalar _tmp280 = _tmp278 * _tmp279;
  const Scalar _tmp281 = -_tmp82 - _tmp84;
  const Scalar _tmp282 = _tmp281 + _tmp90;
  const Scalar _tmp283 = -_tmp100 - _tmp101 - _tmp102 - _tmp103;
  const Scalar _tmp284 = _tmp77 + _tmp94;
  const Scalar _tmp285 = _tmp283 + _tmp284;
  const Scalar _tmp286 = _tmp282 + _tmp285;
  const Scalar _tmp287 = _tmp281 + _tmp97;
  const Scalar _tmp288 = _tmp65 - _tmp68 - _tmp71 + _tmp74;
  const Scalar _tmp289 = -_tmp79 + _tmp95;
  const Scalar _tmp290 = _tmp288 + _tmp289;
  const Scalar _tmp291 = _tmp287 + _tmp290;
  const Scalar _tmp292 =
      (2 * _tmp122 * _tmp291 + 2 * _tmp123 * _tmp286) / (_tmp124 * std::sqrt(_tmp124));
  const Scalar _tmp293 = (Scalar(1) / Scalar(2)) * _tmp292;
  const Scalar _tmp294 = _tmp123 * _tmp293;
  const Scalar _tmp295 = _tmp294 * fh1;
  const Scalar _tmp296 = _tmp285 + _tmp98;
  const Scalar _tmp297 = Scalar(1.0) / (_tmp186);
  const Scalar _tmp298 = _tmp188 * _tmp296 * _tmp297;
  const Scalar _tmp299 = _tmp213 * _tmp224;
  const Scalar _tmp300 = _tmp228 * _tmp231;
  const Scalar _tmp301 = _tmp122 * _tmp293;
  const Scalar _tmp302 = _tmp301 * fh1;
  const Scalar _tmp303 = _tmp282 + _tmp96;
  const Scalar _tmp304 = _tmp283 + _tmp303;
  const Scalar _tmp305 = _tmp165 * _tmp304;
  const Scalar _tmp306 = _tmp290 + _tmp91;
  const Scalar _tmp307 =
      (2 * _tmp160 * _tmp306 + 2 * _tmp163 * _tmp304) / (_tmp164 * std::sqrt(_tmp164));
  const Scalar _tmp308 = (Scalar(1) / Scalar(2)) * _tmp307;
  const Scalar _tmp309 = _tmp163 * _tmp308;
  const Scalar _tmp310 = _tmp152 * _tmp296;
  const Scalar _tmp311 = _tmp297 * _tmp310;
  const Scalar _tmp312 = _tmp283 + _tmp99;
  const Scalar _tmp313 = _tmp288 + _tmp92;
  const Scalar _tmp314 =
      (2 * _tmp138 * _tmp312 + 2 * _tmp142 * _tmp313) / (_tmp143 * std::sqrt(_tmp143));
  const Scalar _tmp315 = (Scalar(1) / Scalar(2)) * _tmp314;
  const Scalar _tmp316 = _tmp138 * _tmp315;
  const Scalar _tmp317 = _tmp144 * _tmp312;
  const Scalar _tmp318 = _tmp144 * _tmp313;
  const Scalar _tmp319 = _tmp287 + _tmp80;
  const Scalar _tmp320 = _tmp288 + _tmp319;
  const Scalar _tmp321 = _tmp156 * _tmp320;
  const Scalar _tmp322 = _tmp142 * _tmp315;
  const Scalar _tmp323 = (-_tmp145 * _tmp311 + _tmp145 * _tmp321 - _tmp157 * _tmp316 +
                          _tmp157 * _tmp317 - _tmp318 + _tmp322) /
                         std::pow(_tmp170, Scalar(2));
  const Scalar _tmp324 = _tmp145 * _tmp323;
  const Scalar _tmp325 = _tmp160 * _tmp308;
  const Scalar _tmp326 = _tmp165 * _tmp306;
  const Scalar _tmp327 = _tmp157 * _tmp305 - _tmp157 * _tmp309 - _tmp166 * _tmp311 +
                         _tmp166 * _tmp321 + _tmp325 - _tmp326;
  const Scalar _tmp328 = _tmp147 * _tmp317;
  const Scalar _tmp329 = -_tmp105 + _tmp107 + _tmp108 - _tmp109;
  const Scalar _tmp330 = -_tmp111 - _tmp112 - _tmp113 - _tmp114 + _tmp119;
  const Scalar _tmp331 = _tmp329 + _tmp330;
  const Scalar _tmp332 = _tmp110 + _tmp330;
  const Scalar _tmp333 = _tmp145 * _tmp332;
  const Scalar _tmp334 = _tmp133 * _tmp316 - _tmp133 * _tmp317 - _tmp145 * _tmp331 -
                         _tmp147 * _tmp316 + _tmp328 + _tmp333;
  const Scalar _tmp335 = _tmp168 * _tmp323;
  const Scalar _tmp336 = Scalar(1.6799999999999999) * _tmp20 * _tmp69;
  const Scalar _tmp337 = Scalar(1.6799999999999999) * _tmp117 + _tmp336;
  const Scalar _tmp338 = _tmp180 * _tmp337;
  const Scalar _tmp339 = _tmp120 + _tmp329;
  const Scalar _tmp340 = _tmp12 * _tmp78;
  const Scalar _tmp341 = _tmp16 * _tmp93;
  const Scalar _tmp342 = _tmp23 * _tmp76 + Scalar(0.83999999999999997) * _tmp81;
  const Scalar _tmp343 = _tmp340 + _tmp341 + _tmp342;
  const Scalar _tmp344 = _tmp343 / std::pow(_tmp179, Scalar(2));
  const Scalar _tmp345 = _tmp182 * _tmp344;
  const Scalar _tmp346 = _tmp171 * _tmp327;
  const Scalar _tmp347 = _tmp147 * _tmp305;
  const Scalar _tmp348 = _tmp157 * _tmp332;
  const Scalar _tmp349 = _tmp133 * _tmp318 - _tmp133 * _tmp322 + _tmp148 * _tmp311 -
                         _tmp148 * _tmp321 - _tmp157 * _tmp328 - _tmp157 * _tmp333 +
                         _tmp169 * _tmp331 + _tmp214 * _tmp316;
  const Scalar _tmp350 = -_tmp157 * _tmp347 - _tmp166 * _tmp348 + _tmp167 * _tmp339 -
                         _tmp172 * _tmp349 + _tmp173 * _tmp311 - _tmp173 * _tmp321 -
                         _tmp174 * _tmp325 + _tmp174 * _tmp326 + _tmp175 * _tmp335 -
                         _tmp175 * _tmp346 + _tmp214 * _tmp309;
  const Scalar _tmp351 = -_tmp147 * _tmp309 + _tmp149 * _tmp335 - _tmp149 * _tmp346 +
                         _tmp166 * _tmp332 - _tmp166 * _tmp339 - _tmp172 * _tmp334 -
                         _tmp174 * _tmp305 + _tmp174 * _tmp309 - _tmp176 * _tmp338 +
                         _tmp176 * _tmp345 - _tmp183 * _tmp350 + _tmp347;
  const Scalar _tmp352 = _tmp351 / std::pow(_tmp184, Scalar(2));
  const Scalar _tmp353 = _tmp220 * _tmp352;
  const Scalar _tmp354 = _tmp192 * _tmp298;
  const Scalar _tmp355 = _tmp195 * _tmp323;
  const Scalar _tmp356 = _tmp152 * _tmp320 + _tmp155 * _tmp296;
  const Scalar _tmp357 = _tmp356 / (_tmp187 * std::sqrt(_tmp187));
  const Scalar _tmp358 = _tmp189 * _tmp320;
  const Scalar _tmp359 =
      _tmp193 * (_tmp150 * _tmp155 * _tmp357 + _tmp153 * _tmp358 - _tmp155 * _tmp358 +
                 _tmp189 * _tmp310 - _tmp190 * _tmp296 - _tmp191 * _tmp357);
  const Scalar _tmp360 = _tmp156 * _tmp189 * _tmp356;
  const Scalar _tmp361 = _tmp192 * _tmp360;
  const Scalar _tmp362 = -_tmp136 * _tmp318 + _tmp136 * _tmp322 + _tmp138 * _tmp318 -
                         _tmp140 * _tmp316 + _tmp140 * _tmp317 - _tmp145 * _tmp354 +
                         _tmp145 * _tmp359 + _tmp145 * _tmp361 - _tmp169 * _tmp312 -
                         _tmp194 * _tmp316 + _tmp194 * _tmp317;
  const Scalar _tmp363 = _tmp158 * _tmp305 - _tmp158 * _tmp309 + _tmp161 * _tmp325 -
                         _tmp161 * _tmp326 + _tmp163 * _tmp326 - _tmp166 * _tmp354 +
                         _tmp166 * _tmp359 + _tmp166 * _tmp361 - _tmp167 * _tmp304 +
                         _tmp168 * _tmp355 - _tmp172 * _tmp362 + _tmp194 * _tmp305 -
                         _tmp194 * _tmp309 - _tmp195 * _tmp346;
  const Scalar _tmp364 = _tmp185 * _tmp363;
  const Scalar _tmp365 = _tmp171 * _tmp321;
  const Scalar _tmp366 = _tmp157 * _tmp323;
  const Scalar _tmp367 = _tmp171 * _tmp311;
  const Scalar _tmp368 = -_tmp147 * _tmp311 + _tmp147 * _tmp321 + _tmp175 * _tmp365 -
                         _tmp175 * _tmp366 - _tmp175 * _tmp367 + _tmp215 * _tmp349 + _tmp348;
  const Scalar _tmp369 = _tmp111 + _tmp112 + _tmp113 + _tmp114 - _tmp116 - _tmp118 +
                         _tmp149 * _tmp365 - _tmp149 * _tmp366 - _tmp149 * _tmp367 -
                         _tmp183 * _tmp368 + _tmp215 * _tmp334 - _tmp216 * _tmp338 +
                         _tmp216 * _tmp345 + _tmp329;
  const Scalar _tmp370 = _tmp199 * _tmp352;
  const Scalar _tmp371 = _tmp203 * (-_tmp157 * _tmp355 + _tmp195 * _tmp365 - _tmp195 * _tmp367 -
                                    _tmp200 * _tmp369 + _tmp215 * _tmp362 - _tmp217 * _tmp364 +
                                    _tmp217 * _tmp370 + _tmp354 - _tmp359 - _tmp361);
  const Scalar _tmp372 = _tmp202 * _tmp351;
  const Scalar _tmp373 = _tmp218 * _tmp372;
  const Scalar _tmp374 = _tmp363 / std::pow(_tmp199, Scalar(2));
  const Scalar _tmp375 = _tmp184 * _tmp374;
  const Scalar _tmp376 = _tmp218 * _tmp375;
  const Scalar _tmp377 = _tmp369 + _tmp371 + _tmp373 - _tmp376;
  const Scalar _tmp378 =
      _tmp168 * _tmp353 - _tmp207 * _tmp377 - _tmp223 * _tmp327 + _tmp311 - _tmp321;
  const Scalar _tmp379 = _tmp171 * _tmp221;
  const Scalar _tmp380 = _tmp166 * _tmp185;
  const Scalar _tmp381 = _tmp127 * _tmp210;
  const Scalar _tmp382 = fh1 * (_tmp126 * _tmp286 - _tmp212 * _tmp291 + _tmp226 * _tmp291 -
                                _tmp227 * _tmp286 + _tmp294 * _tmp37 - _tmp301 * _tmp48);
  const Scalar _tmp383 = Scalar(1.0) * _tmp183;
  const Scalar _tmp384 = Scalar(1.0) * _tmp323;
  const Scalar _tmp385 = _tmp196 * _tmp349;
  const Scalar _tmp386 = _tmp149 * _tmp384 - _tmp175 * _tmp323 * _tmp383 + _tmp183 * _tmp385 -
                         _tmp196 * _tmp334 + _tmp197 * _tmp338 - _tmp197 * _tmp345;
  const Scalar _tmp387 = _tmp203 * (-_tmp196 * _tmp362 - _tmp198 * _tmp364 + _tmp198 * _tmp370 -
                                    _tmp200 * _tmp386 + Scalar(1.0) * _tmp355);
  const Scalar _tmp388 = _tmp201 * _tmp372;
  const Scalar _tmp389 = _tmp201 * _tmp375;
  const Scalar _tmp390 = _tmp386 + _tmp387 + _tmp388 - _tmp389;
  const Scalar _tmp391 = _tmp205 * _tmp352;
  const Scalar _tmp392 = _tmp168 * _tmp391 - _tmp206 * _tmp327 - _tmp207 * _tmp390;
  const Scalar _tmp393 = _tmp237 * _tmp238;
  const Scalar _tmp394 = Scalar(1.0) * _tmp374;
  const Scalar _tmp395 = _tmp145 * _tmp346;
  const Scalar _tmp396 = Scalar(0.5) * _tmp202;
  const Scalar _tmp397 = _tmp145 * _tmp172;
  const Scalar _tmp398 = _tmp145 * _tmp335;
  const Scalar _tmp399 = _tmp234 * _tmp352;
  const Scalar _tmp400 = _tmp125 * fh1;
  const Scalar _tmp401 = _tmp286 * _tmp400;
  const Scalar _tmp402 = _tmp291 * _tmp400;
  const Scalar _tmp403 =
      -_tmp127 * _tmp193 *
          (-_tmp166 * _tmp391 + _tmp206 * _tmp305 - _tmp206 * _tmp309 - _tmp208 * _tmp324 -
           _tmp209 * _tmp316 + _tmp209 * _tmp317 + _tmp222 * _tmp392 + _tmp380 * _tmp390) -
      _tmp193 * _tmp213 *
          (-_tmp166 * _tmp353 - _tmp221 * _tmp324 + _tmp222 * _tmp378 + _tmp223 * _tmp305 -
           _tmp223 * _tmp309 - _tmp316 * _tmp379 + _tmp317 * _tmp379 + _tmp377 * _tmp380) -
      _tmp193 * _tmp228 *
          (_tmp138 * _tmp172 * _tmp314 * _tmp396 - _tmp163 * _tmp307 * _tmp396 - _tmp166 * _tmp394 +
           _tmp229 * _tmp305 - _tmp229 * _tmp395 + _tmp229 * _tmp398 - _tmp230 * _tmp317 +
           _tmp394 * _tmp397) +
      _tmp211 * _tmp302 - _tmp211 * _tmp402 + _tmp225 * _tmp295 - _tmp225 * _tmp401 -
      _tmp232 * _tmp382 -
      _tmp239 * (-_tmp166 * _tmp399 + _tmp235 * _tmp305 - _tmp235 * _tmp309 - _tmp235 * _tmp395 +
                 _tmp235 * _tmp398 + _tmp236 * _tmp316 - _tmp236 * _tmp317 + _tmp397 * _tmp399) +
      _tmp298 * _tmp299 + _tmp298 * _tmp300 + _tmp298 * _tmp381 + _tmp298 * _tmp393 -
      _tmp299 * _tmp360 - _tmp300 * _tmp360 - _tmp360 * _tmp381 - _tmp360 * _tmp393;
  const Scalar _tmp404 = Scalar(9.6622558468725703) * _tmp403;
  const Scalar _tmp405 = _tmp249 * _tmp372;
  const Scalar _tmp406 = _tmp249 * _tmp375;
  const Scalar _tmp407 = _tmp242 * _tmp338;
  const Scalar _tmp408 = -Scalar(1.6799999999999999) * _tmp115 - _tmp336 + _tmp343;
  const Scalar _tmp409 = _tmp183 * _tmp408;
  const Scalar _tmp410 = _tmp242 * _tmp345;
  const Scalar _tmp411 = (_tmp337 - _tmp340 - _tmp341 + _tmp342 - _tmp407 - _tmp409 + _tmp410) /
                         std::pow(_tmp244, Scalar(2));
  const Scalar _tmp412 = Scalar(1.0) * _tmp411;
  const Scalar _tmp413 = _tmp203 * _tmp412;
  const Scalar _tmp414 = _tmp242 * _tmp249;
  const Scalar _tmp415 = _tmp176 * _tmp394 - _tmp229 * _tmp350 + _tmp242 * _tmp405 -
                         _tmp242 * _tmp413 + _tmp264 * _tmp408 - _tmp375 * _tmp414;
  const Scalar _tmp416 = Scalar(1.0) * _tmp344;
  const Scalar _tmp417 = _tmp177 * _tmp338 - _tmp177 * _tmp345 + _tmp296 + _tmp320 * _tmp383;
  const Scalar _tmp418 = _tmp245 * _tmp417;
  const Scalar _tmp419 = _tmp233 * _tmp411;
  const Scalar _tmp420 = _tmp289 + _tmp75;
  const Scalar _tmp421 = _tmp420 + _tmp91;
  const Scalar _tmp422 = _tmp176 * _tmp399 - _tmp235 * _tmp350 - _tmp242 * _tmp418 +
                         _tmp242 * _tmp419 - _tmp246 * _tmp408 + _tmp421;
  const Scalar _tmp423 = Scalar(0.5) * _tmp292 * fh1;
  const Scalar _tmp424 = Scalar(6.59232) * _tmp16;
  const Scalar _tmp425 = Scalar(6.59232) * _tmp20;
  const Scalar _tmp426 = Scalar(6.59232) * _tmp12;
  const Scalar _tmp427 = _tmp125 * _tmp255;
  const Scalar _tmp428 = Scalar(6.59232) * _tmp23;
  const Scalar _tmp429 = _tmp121 * fh1;
  const Scalar _tmp430 = -_tmp126 * _tmp429 + _tmp255 * _tmp301 - _tmp291 * _tmp427 -
                         _tmp291 * fv1 + _tmp424 * _tmp63 - _tmp425 * _tmp66 - _tmp426 * _tmp69 +
                         _tmp428 * _tmp72;
  const Scalar _tmp431 = _tmp183 * _tmp412;
  const Scalar _tmp432 = _tmp249 * _tmp338;
  const Scalar _tmp433 =
      -_tmp243 * _tmp412 + _tmp249 * _tmp407 + _tmp249 * _tmp409 - _tmp249 * _tmp410;
  const Scalar _tmp434 = _tmp245 * _tmp408;
  const Scalar _tmp435 = _tmp242 * _tmp411;
  const Scalar _tmp436 = _tmp175 * _tmp384 + _tmp176 * _tmp391 + _tmp204 * _tmp434 -
                         _tmp204 * _tmp435 - _tmp206 * _tmp350 + _tmp251 * _tmp387 +
                         _tmp251 * _tmp388 - _tmp251 * _tmp389 - _tmp267 * _tmp390 - _tmp385;
  const Scalar _tmp437 = _tmp176 * _tmp353 + _tmp219 * _tmp434 - _tmp219 * _tmp435 -
                         _tmp223 * _tmp350 + _tmp251 * _tmp371 + _tmp251 * _tmp373 -
                         _tmp251 * _tmp376 - _tmp267 * _tmp377 + _tmp368;
  const Scalar _tmp438 = _tmp212 * _tmp429 - _tmp255 * _tmp294 + _tmp286 * _tmp427 + _tmp286 * fv1 +
                         _tmp424 * _tmp69 + _tmp425 * _tmp72 + _tmp426 * _tmp63 + _tmp428 * _tmp66;
  const Scalar _tmp439 = _tmp248 * _tmp435;
  const Scalar _tmp440 = std::pow(_tmp240, Scalar(-2));
  const Scalar _tmp441 = _tmp403 * _tmp440;
  const Scalar _tmp442 =
      (_tmp241 *
           (-_tmp122 * _tmp253 * _tmp423 - _tmp123 * _tmp269 * _tmp423 +
            Scalar(1.0) * _tmp127 *
                (-_tmp204 * _tmp412 - _tmp248 * _tmp436 + _tmp249 * _tmp387 + _tmp249 * _tmp388 -
                 _tmp249 * _tmp389 + _tmp252 * _tmp416) +
            Scalar(1.0) * _tmp228 *
                (-_tmp248 * _tmp415 + _tmp265 * _tmp416 + _tmp405 - _tmp406 - _tmp413) +
            _tmp250 *
                (_tmp233 * _tmp412 + _tmp247 * _tmp416 - _tmp248 * _tmp422 - _tmp249 * _tmp417) +
            _tmp254 * _tmp402 +
            Scalar(1.0) * _tmp256 *
                (-_tmp248 * _tmp433 - _tmp249 * _tmp345 + _tmp257 * _tmp416 - _tmp431 + _tmp432) +
            _tmp259 * _tmp430 + Scalar(1.0) * _tmp261 * _tmp438 +
            _tmp263 * (_tmp260 * _tmp408 - _tmp344 * _tmp414 + _tmp412 - _tmp439) +
            _tmp266 * _tmp382 + Scalar(1.0) * _tmp269 * _tmp401 +
            _tmp270 * (-_tmp219 * _tmp412 - _tmp248 * _tmp437 + _tmp249 * _tmp371 +
                       _tmp249 * _tmp373 - _tmp249 * _tmp376 + _tmp268 * _tmp416)) -
       _tmp271 * _tmp441) /
      std::sqrt(Scalar(std::pow(_tmp271, Scalar(2)) * _tmp440 + 1));
  const Scalar _tmp443 = _tmp180 * _tmp238;
  const Scalar _tmp444 = _tmp180 * _tmp228;
  const Scalar _tmp445 = _tmp127 * _tmp180;
  const Scalar _tmp446 = _tmp180 * _tmp268;
  const Scalar _tmp447 = _tmp180 * _tmp256;
  const Scalar _tmp448 = _tmp249 * _tmp262;
  const Scalar _tmp449 = _tmp180 * _tmp448;
  const Scalar _tmp450 = _tmp213 * _tmp446 - _tmp242 * _tmp449 + _tmp247 * _tmp443 +
                         _tmp252 * _tmp445 + _tmp257 * _tmp447 + _tmp265 * _tmp444;
  const Scalar _tmp451 = _tmp235 * _tmp238;
  const Scalar _tmp452 = _tmp228 * _tmp229;
  const Scalar _tmp453 =
      _tmp127 * _tmp209 - _tmp172 * _tmp451 - _tmp172 * _tmp452 + _tmp213 * _tmp379;
  const Scalar _tmp454 = Scalar(1.0) / (_tmp453);
  const Scalar _tmp455 = std::asinh(_tmp450 * _tmp454);
  const Scalar _tmp456 = Scalar(1.0) * _tmp455;
  const Scalar _tmp457 = Scalar(9.6622558468725703) * _tmp453;
  const Scalar _tmp458 = -_tmp141 + Scalar(-8.3888750099999996);
  const Scalar _tmp459 = Scalar(2.5202214700000001) - _tmp137;
  const Scalar _tmp460 =
      std::sqrt(Scalar(std::pow(_tmp458, Scalar(2)) + std::pow(_tmp459, Scalar(2))));
  const Scalar _tmp461 = -_tmp455 * _tmp457 - _tmp460;
  const Scalar _tmp462 = Scalar(0.1034955) * _tmp454;
  const Scalar _tmp463 = _tmp461 * _tmp462;
  const Scalar _tmp464 = _tmp228 * _tmp394;
  const Scalar _tmp465 = _tmp229 * _tmp382;
  const Scalar _tmp466 = _tmp238 * _tmp399;
  const Scalar _tmp467 =
      _tmp127 * _tmp171 * _tmp392 - _tmp127 * _tmp208 * _tmp323 + _tmp171 * _tmp213 * _tmp378 +
      _tmp172 * _tmp464 - _tmp172 * _tmp465 + _tmp172 * _tmp466 - _tmp209 * _tmp302 +
      _tmp209 * _tmp402 - _tmp213 * _tmp221 * _tmp323 - _tmp295 * _tmp379 + _tmp335 * _tmp451 +
      _tmp335 * _tmp452 - _tmp346 * _tmp451 - _tmp346 * _tmp452 + _tmp379 * _tmp401;
  const Scalar _tmp468 = Scalar(9.6622558468725703) * _tmp467;
  const Scalar _tmp469 = std::pow(_tmp453, Scalar(-2));
  const Scalar _tmp470 = _tmp467 * _tmp469;
  const Scalar _tmp471 = _tmp180 * _tmp252;
  const Scalar _tmp472 = _tmp249 * _tmp438;
  const Scalar _tmp473 = _tmp256 * _tmp344;
  const Scalar _tmp474 =
      (-_tmp450 * _tmp470 +
       _tmp454 * (-_tmp127 * _tmp252 * _tmp344 + _tmp180 * _tmp213 * _tmp437 -
                  _tmp180 * _tmp242 * _tmp472 + _tmp180 * _tmp257 * _tmp430 +
                  _tmp180 * _tmp265 * _tmp382 - _tmp213 * _tmp268 * _tmp344 -
                  _tmp228 * _tmp265 * _tmp344 - _tmp238 * _tmp247 * _tmp344 +
                  _tmp242 * _tmp344 * _tmp448 - _tmp257 * _tmp473 + _tmp262 * _tmp439 -
                  _tmp295 * _tmp446 - _tmp302 * _tmp471 + _tmp401 * _tmp446 + _tmp402 * _tmp471 -
                  _tmp408 * _tmp449 + _tmp415 * _tmp444 + _tmp422 * _tmp443 + _tmp433 * _tmp447 +
                  _tmp436 * _tmp445)) /
      std::sqrt(Scalar(std::pow(_tmp450, Scalar(2)) * _tmp469 + 1));
  const Scalar _tmp475 = _tmp104 + _tmp284;
  const Scalar _tmp476 = _tmp127 * _tmp245;
  const Scalar _tmp477 = _tmp219 * _tmp245;
  const Scalar _tmp478 = -_tmp204 * _tmp476 - _tmp213 * _tmp477 - _tmp228 * _tmp264 +
                         _tmp238 * _tmp246 - _tmp256 * _tmp258 + _tmp448;
  const Scalar _tmp479 = _tmp127 * _tmp206 + _tmp213 * _tmp223 + _tmp451 + _tmp452;
  const Scalar _tmp480 = Scalar(1.0) / (_tmp479);
  const Scalar _tmp481 = std::asinh(_tmp478 * _tmp480);
  const Scalar _tmp482 = Scalar(1.0) * _tmp481;
  const Scalar _tmp483 = _tmp213 * _tmp245;
  const Scalar _tmp484 = _tmp204 * _tmp245;
  const Scalar _tmp485 = std::pow(_tmp479, Scalar(-2));
  const Scalar _tmp486 = _tmp127 * _tmp185 * _tmp390 - _tmp127 * _tmp391 +
                         _tmp185 * _tmp213 * _tmp377 - _tmp206 * _tmp302 + _tmp206 * _tmp402 -
                         _tmp213 * _tmp353 - _tmp223 * _tmp295 + _tmp223 * _tmp401 - _tmp464 +
                         _tmp465 - _tmp466;
  const Scalar _tmp487 = _tmp485 * _tmp486;
  const Scalar _tmp488 =
      (-_tmp478 * _tmp487 +
       _tmp480 * (_tmp127 * _tmp204 * _tmp411 + _tmp182 * _tmp249 * _tmp473 +
                  _tmp213 * _tmp219 * _tmp411 - _tmp228 * _tmp405 + _tmp228 * _tmp406 +
                  _tmp228 * _tmp413 + _tmp238 * _tmp418 - _tmp238 * _tmp419 + _tmp256 * _tmp431 -
                  _tmp256 * _tmp432 - _tmp258 * _tmp430 - _tmp263 * _tmp411 - _tmp264 * _tmp382 +
                  _tmp295 * _tmp477 + _tmp302 * _tmp484 - _tmp371 * _tmp483 - _tmp373 * _tmp483 +
                  _tmp376 * _tmp483 - _tmp387 * _tmp476 - _tmp388 * _tmp476 + _tmp389 * _tmp476 -
                  _tmp401 * _tmp477 - _tmp402 * _tmp484 + _tmp472)) /
      std::sqrt(Scalar(std::pow(_tmp478, Scalar(2)) * _tmp485 + 1));
  const Scalar _tmp489 = Scalar(4.7752063900000001) - _tmp159;
  const Scalar _tmp490 = Scalar(2.71799795) - _tmp162;
  const Scalar _tmp491 =
      std::sqrt(Scalar(std::pow(_tmp489, Scalar(2)) + std::pow(_tmp490, Scalar(2))));
  const Scalar _tmp492 = Scalar(9.6622558468725703) * _tmp479;
  const Scalar _tmp493 = -_tmp481 * _tmp492 - _tmp491;
  const Scalar _tmp494 = Scalar(9.6622558468725703) * _tmp486;
  const Scalar _tmp495 = Scalar(0.1034955) * _tmp480;
  const Scalar _tmp496 = _tmp493 * _tmp495;

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) =
      _tmp121 -
      Scalar(0.5) * (2 * _tmp39 * (_tmp75 + _tmp92) + 2 * _tmp50 * (_tmp104 + _tmp99)) *
          std::sinh(Scalar(0.1034955) * _tmp52 *
                    (-_tmp51 - Scalar(9.6622558468725703) * fh1 * std::asinh(_tmp52 * fv1))) /
          _tmp51;
  _res(1, 0) =
      -_tmp274 *
          (-Scalar(0.87679799772039002) * _tmp441 + Scalar(1.0) * _tmp442 * std::sinh(_tmp273) -
           (-Scalar(0.1034955) * _tmp278 * _tmp441 +
            _tmp279 * (-_tmp272 * _tmp404 - _tmp274 * _tmp442 -
                       Scalar(1) / Scalar(2) *
                           (2 * _tmp275 * _tmp421 + 2 * _tmp276 * (_tmp104 + _tmp303)) / _tmp277)) *
               std::sinh(_tmp280)) +
      _tmp332 -
      _tmp404 * (Scalar(0.87679799772039002) * _tmp241 + std::cosh(_tmp273) - std::cosh(_tmp280));
  _res(2, 0) =
      _tmp331 -
      _tmp457 *
          (-Scalar(0.87653584775870996) * _tmp470 + Scalar(1.0) * _tmp474 * std::sinh(_tmp456) -
           (-Scalar(0.1034955) * _tmp461 * _tmp470 +
            _tmp462 * (-_tmp455 * _tmp468 - _tmp457 * _tmp474 -
                       Scalar(1) / Scalar(2) *
                           (2 * _tmp458 * (_tmp287 + _tmp420) + 2 * _tmp459 * (_tmp282 + _tmp475)) /
                           _tmp460)) *
               std::sinh(_tmp463)) -
      _tmp468 * (Scalar(0.87653584775870996) * _tmp454 + std::cosh(_tmp456) - std::cosh(_tmp463));
  _res(3, 0) =
      _tmp339 -
      _tmp492 *
          (-Scalar(0.86565325453551001) * _tmp487 + Scalar(1.0) * _tmp488 * std::sinh(_tmp482) -
           (-Scalar(0.1034955) * _tmp487 * _tmp493 +
            _tmp495 * (-_tmp481 * _tmp494 - _tmp488 * _tmp492 -
                       Scalar(1) / Scalar(2) *
                           (2 * _tmp489 * (_tmp319 + _tmp75) + 2 * _tmp490 * (_tmp475 + _tmp98)) /
                           _tmp491)) *
               std::sinh(_tmp496)) -
      _tmp494 * (Scalar(0.86565325453551001) * _tmp480 + std::cosh(_tmp482) - std::cosh(_tmp496));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
