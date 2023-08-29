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
 * Symbolic function: IK_residual_func_cost1_wrt_ry_Nl18
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost1WrtRyNl18(
    const Scalar fh1, const Scalar fv1, const Scalar rx, const Scalar ry, const Scalar rz,
    const Scalar p_init0, const Scalar p_init1, const Scalar p_init2, const Scalar rot_init_x,
    const Scalar rot_init_y, const Scalar rot_init_z, const Scalar rot_init_w,
    const Scalar epsilon) {
  // Total ops: 1626

  // Unused inputs
  (void)p_init2;
  (void)epsilon;

  // Input arrays

  // Intermediate terms (501)
  const Scalar _tmp0 = std::pow(ry, Scalar(2));
  const Scalar _tmp1 = _tmp0 + std::pow(rx, Scalar(2)) + std::pow(rz, Scalar(2));
  const Scalar _tmp2 = std::sqrt(_tmp1);
  const Scalar _tmp3 = (Scalar(1) / Scalar(2)) * _tmp2;
  const Scalar _tmp4 = std::cos(_tmp3);
  const Scalar _tmp5 = _tmp4 * rot_init_x;
  const Scalar _tmp6 = std::sin(_tmp3);
  const Scalar _tmp7 = _tmp6 / _tmp2;
  const Scalar _tmp8 = _tmp7 * rot_init_z;
  const Scalar _tmp9 = _tmp8 * ry;
  const Scalar _tmp10 = _tmp7 * rot_init_w;
  const Scalar _tmp11 = _tmp7 * rot_init_y;
  const Scalar _tmp12 = _tmp10 * rx + _tmp11 * rz + _tmp5 - _tmp9;
  const Scalar _tmp13 = _tmp4 * rot_init_y;
  const Scalar _tmp14 = _tmp10 * ry;
  const Scalar _tmp15 = _tmp7 * rot_init_x;
  const Scalar _tmp16 = _tmp13 + _tmp14 - _tmp15 * rz + _tmp8 * rx;
  const Scalar _tmp17 = 2 * _tmp16;
  const Scalar _tmp18 = _tmp12 * _tmp17;
  const Scalar _tmp19 = _tmp4 * rot_init_z;
  const Scalar _tmp20 = _tmp15 * ry;
  const Scalar _tmp21 = _tmp10 * rz - _tmp11 * rx + _tmp19 + _tmp20;
  const Scalar _tmp22 = _tmp4 * rot_init_w;
  const Scalar _tmp23 = _tmp11 * ry;
  const Scalar _tmp24 = -_tmp15 * rx + _tmp22 - _tmp23 - _tmp8 * rz;
  const Scalar _tmp25 = 2 * _tmp24;
  const Scalar _tmp26 = _tmp21 * _tmp25;
  const Scalar _tmp27 = Scalar(0.20999999999999999) * _tmp18 + Scalar(0.20999999999999999) * _tmp26;
  const Scalar _tmp28 = -_tmp27;
  const Scalar _tmp29 = _tmp17 * _tmp21;
  const Scalar _tmp30 = _tmp12 * _tmp25;
  const Scalar _tmp31 = _tmp29 - _tmp30;
  const Scalar _tmp32 = -Scalar(0.010999999999999999) * _tmp31;
  const Scalar _tmp33 = -2 * std::pow(_tmp12, Scalar(2));
  const Scalar _tmp34 = -2 * std::pow(_tmp21, Scalar(2));
  const Scalar _tmp35 = Scalar(0.20999999999999999) * _tmp33 +
                        Scalar(0.20999999999999999) * _tmp34 + Scalar(0.20999999999999999);
  const Scalar _tmp36 = _tmp32 + _tmp35;
  const Scalar _tmp37 = _tmp28 + _tmp36;
  const Scalar _tmp38 = _tmp37 + p_init1;
  const Scalar _tmp39 = Scalar(4.8333311099999996) - _tmp38;
  const Scalar _tmp40 = Scalar(0.20999999999999999) * _tmp18 - Scalar(0.20999999999999999) * _tmp26;
  const Scalar _tmp41 = 1 - 2 * std::pow(_tmp16, Scalar(2));
  const Scalar _tmp42 = Scalar(0.20999999999999999) * _tmp34 + Scalar(0.20999999999999999) * _tmp41;
  const Scalar _tmp43 = 2 * _tmp12 * _tmp21;
  const Scalar _tmp44 = _tmp16 * _tmp25;
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
  const Scalar _tmp55 = _tmp53 * ry;
  const Scalar _tmp56 = _tmp55 * rz;
  const Scalar _tmp57 = _tmp55 * rx;
  const Scalar _tmp58 = _tmp6 / (_tmp1 * std::sqrt(_tmp1));
  const Scalar _tmp59 = _tmp0 * _tmp58;
  const Scalar _tmp60 = _tmp58 * ry;
  const Scalar _tmp61 = _tmp60 * rx;
  const Scalar _tmp62 = _tmp60 * rz;
  const Scalar _tmp63 = _tmp13 * _tmp56 - _tmp19 * _tmp54 - Scalar(1) / Scalar(2) * _tmp20 +
                        _tmp22 * _tmp57 + _tmp59 * rot_init_z - _tmp61 * rot_init_w -
                        _tmp62 * rot_init_y - _tmp8;
  const Scalar _tmp64 = Scalar(0.021999999999999999) * _tmp24;
  const Scalar _tmp65 = _tmp63 * _tmp64;
  const Scalar _tmp66 = -_tmp11 - _tmp13 * _tmp54 - Scalar(1) / Scalar(2) * _tmp14 -
                        _tmp19 * _tmp56 - _tmp5 * _tmp57 + _tmp59 * rot_init_y +
                        _tmp61 * rot_init_x + _tmp62 * rot_init_z;
  const Scalar _tmp67 = Scalar(0.021999999999999999) * _tmp66;
  const Scalar _tmp68 = _tmp12 * _tmp67;
  const Scalar _tmp69 = -_tmp13 * _tmp57 + _tmp15 + _tmp22 * _tmp56 + _tmp5 * _tmp54 -
                        _tmp59 * rot_init_x + _tmp61 * rot_init_y - _tmp62 * rot_init_w -
                        Scalar(1) / Scalar(2) * _tmp9;
  const Scalar _tmp70 = Scalar(0.021999999999999999) * _tmp69;
  const Scalar _tmp71 = _tmp16 * _tmp70;
  const Scalar _tmp72 = _tmp10 + _tmp19 * _tmp57 + _tmp22 * _tmp54 -
                        Scalar(1) / Scalar(2) * _tmp23 - _tmp5 * _tmp56 - _tmp59 * rot_init_w -
                        _tmp61 * rot_init_z + _tmp62 * rot_init_x;
  const Scalar _tmp73 = Scalar(0.021999999999999999) * _tmp21;
  const Scalar _tmp74 = _tmp72 * _tmp73;
  const Scalar _tmp75 = -_tmp65 - _tmp68 + _tmp71 + _tmp74;
  const Scalar _tmp76 = Scalar(0.83999999999999997) * _tmp21;
  const Scalar _tmp77 = _tmp69 * _tmp76;
  const Scalar _tmp78 = Scalar(0.83999999999999997) * _tmp63;
  const Scalar _tmp79 = _tmp12 * _tmp78;
  const Scalar _tmp80 = _tmp77 + _tmp79;
  const Scalar _tmp81 = _tmp75 + _tmp80;
  const Scalar _tmp82 = Scalar(0.41999999999999998) * _tmp72;
  const Scalar _tmp83 = _tmp12 * _tmp82;
  const Scalar _tmp84 = Scalar(0.41999999999999998) * _tmp63;
  const Scalar _tmp85 = _tmp16 * _tmp84;
  const Scalar _tmp86 = _tmp83 + _tmp85;
  const Scalar _tmp87 = Scalar(0.41999999999999998) * _tmp69;
  const Scalar _tmp88 = _tmp24 * _tmp87;
  const Scalar _tmp89 = Scalar(0.41999999999999998) * _tmp66;
  const Scalar _tmp90 = _tmp21 * _tmp89;
  const Scalar _tmp91 = _tmp88 + _tmp90;
  const Scalar _tmp92 = _tmp86 + _tmp91;
  const Scalar _tmp93 = -_tmp83 - _tmp85;
  const Scalar _tmp94 = _tmp91 + _tmp93;
  const Scalar _tmp95 = -_tmp77;
  const Scalar _tmp96 = Scalar(0.83999999999999997) * _tmp72;
  const Scalar _tmp97 = _tmp16 * _tmp96;
  const Scalar _tmp98 = _tmp95 - _tmp97;
  const Scalar _tmp99 = _tmp64 * _tmp72;
  const Scalar _tmp100 = _tmp16 * _tmp67;
  const Scalar _tmp101 = _tmp12 * _tmp70;
  const Scalar _tmp102 = _tmp63 * _tmp73;
  const Scalar _tmp103 = _tmp100 + _tmp101 + _tmp102 + _tmp99;
  const Scalar _tmp104 = _tmp103 + _tmp98;
  const Scalar _tmp105 = _tmp24 * _tmp82;
  const Scalar _tmp106 = _tmp16 * _tmp89;
  const Scalar _tmp107 = _tmp12 * _tmp87;
  const Scalar _tmp108 = _tmp21 * _tmp84;
  const Scalar _tmp109 = _tmp105 + _tmp106 - _tmp107 - _tmp108;
  const Scalar _tmp110 = _tmp24 * _tmp84;
  const Scalar _tmp111 = _tmp12 * _tmp89;
  const Scalar _tmp112 = _tmp16 * _tmp87;
  const Scalar _tmp113 = _tmp21 * _tmp82;
  const Scalar _tmp114 = _tmp16 * _tmp72;
  const Scalar _tmp115 = Scalar(0.043999999999999997) * _tmp114;
  const Scalar _tmp116 = _tmp12 * _tmp63;
  const Scalar _tmp117 = Scalar(0.043999999999999997) * _tmp116;
  const Scalar _tmp118 = _tmp115 + _tmp117;
  const Scalar _tmp119 = _tmp110 + _tmp111 + _tmp112 + _tmp113 + _tmp118;
  const Scalar _tmp120 = _tmp109 + _tmp119;
  const Scalar _tmp121 = _tmp27 + _tmp36;
  const Scalar _tmp122 = _tmp121 + p_init1;
  const Scalar _tmp123 = _tmp122 + Scalar(-4.7752063900000001);
  const Scalar _tmp124 = _tmp42 + _tmp46;
  const Scalar _tmp125 = _tmp124 + _tmp40;
  const Scalar _tmp126 = _tmp125 + p_init0;
  const Scalar _tmp127 = _tmp126 + Scalar(-2.71799795);
  const Scalar _tmp128 = std::pow(_tmp123, Scalar(2)) + std::pow(_tmp127, Scalar(2));
  const Scalar _tmp129 = std::pow(_tmp128, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp130 = _tmp127 * _tmp129;
  const Scalar _tmp131 =
      Scalar(0.20999999999999999) * _tmp43 - Scalar(0.20999999999999999) * _tmp44;
  const Scalar _tmp132 =
      -Scalar(0.010999999999999999) * _tmp33 - Scalar(0.010999999999999999) * _tmp41;
  const Scalar _tmp133 =
      Scalar(0.20999999999999999) * _tmp29 + Scalar(0.20999999999999999) * _tmp30;
  const Scalar _tmp134 = _tmp132 - _tmp133;
  const Scalar _tmp135 = _tmp131 + _tmp134;
  const Scalar _tmp136 = -_tmp40;
  const Scalar _tmp137 = _tmp124 + _tmp136;
  const Scalar _tmp138 = _tmp137 + p_init0;
  const Scalar _tmp139 = _tmp138 + Scalar(-2.5202214700000001);
  const Scalar _tmp140 = _tmp32 - _tmp35;
  const Scalar _tmp141 = _tmp140 + _tmp27;
  const Scalar _tmp142 = _tmp141 + p_init1;
  const Scalar _tmp143 = _tmp142 + Scalar(8.3888750099999996);
  const Scalar _tmp144 = std::pow(_tmp139, Scalar(2)) + std::pow(_tmp143, Scalar(2));
  const Scalar _tmp145 = std::pow(_tmp144, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp146 = _tmp139 * _tmp145;
  const Scalar _tmp147 = -_tmp131;
  const Scalar _tmp148 = _tmp134 + _tmp147;
  const Scalar _tmp149 = _tmp146 * _tmp148;
  const Scalar _tmp150 = -_tmp135 * _tmp146 + _tmp149;
  const Scalar _tmp151 = _tmp140 + _tmp28;
  const Scalar _tmp152 = _tmp151 + p_init1;
  const Scalar _tmp153 = _tmp152 + Scalar(8.3196563700000006);
  const Scalar _tmp154 = _tmp136 + _tmp47;
  const Scalar _tmp155 = _tmp154 + p_init0;
  const Scalar _tmp156 = _tmp155 + Scalar(1.9874742000000001);
  const Scalar _tmp157 = Scalar(1.0) / (_tmp156);
  const Scalar _tmp158 = _tmp153 * _tmp157;
  const Scalar _tmp159 = _tmp123 * _tmp129;
  const Scalar _tmp160 = _tmp130 * _tmp158 - _tmp159;
  const Scalar _tmp161 = _tmp143 * _tmp145;
  const Scalar _tmp162 = _tmp146 * _tmp158 - _tmp161;
  const Scalar _tmp163 = Scalar(1.0) / (_tmp162);
  const Scalar _tmp164 = _tmp160 * _tmp163;
  const Scalar _tmp165 = _tmp130 * _tmp148;
  const Scalar _tmp166 = _tmp132 + _tmp133;
  const Scalar _tmp167 = _tmp131 + _tmp166;
  const Scalar _tmp168 = _tmp135 * _tmp161 - _tmp149 * _tmp158;
  const Scalar _tmp169 = -_tmp158 * _tmp165 + _tmp159 * _tmp167 - _tmp164 * _tmp168;
  const Scalar _tmp170 = Scalar(1.0) * _tmp151;
  const Scalar _tmp171 = -_tmp170;
  const Scalar _tmp172 = _tmp141 + _tmp171;
  const Scalar _tmp173 = Scalar(1.0) / (_tmp172);
  const Scalar _tmp174 = Scalar(1.0) * _tmp154;
  const Scalar _tmp175 = -_tmp137 + _tmp174;
  const Scalar _tmp176 = _tmp173 * _tmp175;
  const Scalar _tmp177 = -_tmp130 * _tmp167 - _tmp150 * _tmp164 + _tmp165 - _tmp169 * _tmp176;
  const Scalar _tmp178 = Scalar(1.0) / (_tmp177);
  const Scalar _tmp179 = std::pow(_tmp156, Scalar(2));
  const Scalar _tmp180 = std::pow(_tmp153, Scalar(2)) + _tmp179;
  const Scalar _tmp181 = std::sqrt(_tmp180);
  const Scalar _tmp182 = Scalar(1.0) / (_tmp181);
  const Scalar _tmp183 = _tmp151 * _tmp156;
  const Scalar _tmp184 = _tmp153 * _tmp154;
  const Scalar _tmp185 = -_tmp182 * _tmp183 + _tmp182 * _tmp184;
  const Scalar _tmp186 = _tmp157 * _tmp181;
  const Scalar _tmp187 = _tmp185 * _tmp186;
  const Scalar _tmp188 = _tmp141 * _tmp145;
  const Scalar _tmp189 = -_tmp137 * _tmp161 + _tmp139 * _tmp188 + _tmp146 * _tmp187;
  const Scalar _tmp190 = Scalar(1.0) * _tmp163;
  const Scalar _tmp191 =
      _tmp121 * _tmp130 - _tmp125 * _tmp159 + _tmp130 * _tmp187 - _tmp164 * _tmp189;
  const Scalar _tmp192 = Scalar(1.0) * _tmp173;
  const Scalar _tmp193 = _tmp175 * _tmp192;
  const Scalar _tmp194 = _tmp163 * _tmp193;
  const Scalar _tmp195 = -_tmp150 * _tmp190 + _tmp168 * _tmp194;
  const Scalar _tmp196 = _tmp178 * _tmp195;
  const Scalar _tmp197 = -_tmp189 * _tmp190 - _tmp191 * _tmp196;
  const Scalar _tmp198 = Scalar(1.0) / (_tmp191);
  const Scalar _tmp199 = _tmp177 * _tmp198;
  const Scalar _tmp200 = _tmp197 * _tmp199;
  const Scalar _tmp201 = _tmp195 + _tmp200;
  const Scalar _tmp202 = _tmp178 * _tmp201;
  const Scalar _tmp203 = _tmp160 * _tmp178;
  const Scalar _tmp204 = -_tmp201 * _tmp203 + Scalar(1.0);
  const Scalar _tmp205 = _tmp146 * _tmp163;
  const Scalar _tmp206 = _tmp130 * _tmp202 + _tmp204 * _tmp205;
  const Scalar _tmp207 = _tmp38 + Scalar(-4.8333311099999996);
  const Scalar _tmp208 = _tmp49 + Scalar(1.79662371);
  const Scalar _tmp209 = std::pow(_tmp207, Scalar(2)) + std::pow(_tmp208, Scalar(2));
  const Scalar _tmp210 = std::pow(_tmp209, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp211 = _tmp207 * _tmp210;
  const Scalar _tmp212 = _tmp211 * fh1;
  const Scalar _tmp213 = _tmp186 * _tmp212;
  const Scalar _tmp214 = _tmp148 * _tmp158;
  const Scalar _tmp215 = _tmp158 * _tmp163;
  const Scalar _tmp216 = _tmp168 * _tmp215 + _tmp214;
  const Scalar _tmp217 = -_tmp148 + _tmp150 * _tmp215 - _tmp176 * _tmp216;
  const Scalar _tmp218 = _tmp178 * _tmp217;
  const Scalar _tmp219 = -_tmp187 + _tmp189 * _tmp215 - _tmp191 * _tmp218;
  const Scalar _tmp220 = _tmp199 * _tmp219;
  const Scalar _tmp221 = _tmp217 + _tmp220;
  const Scalar _tmp222 = -_tmp158 - _tmp203 * _tmp221;
  const Scalar _tmp223 = _tmp163 * _tmp222;
  const Scalar _tmp224 = _tmp178 * _tmp221;
  const Scalar _tmp225 = _tmp130 * _tmp224 + _tmp146 * _tmp223 + Scalar(1.0);
  const Scalar _tmp226 = _tmp208 * _tmp210;
  const Scalar _tmp227 = _tmp226 * fh1;
  const Scalar _tmp228 = _tmp186 * _tmp227;
  const Scalar _tmp229 = Scalar(1.0) * _tmp198;
  const Scalar _tmp230 = _tmp146 * _tmp164;
  const Scalar _tmp231 = _tmp130 * _tmp229 - _tmp229 * _tmp230;
  const Scalar _tmp232 = _tmp210 * _tmp48;
  const Scalar _tmp233 = fh1 * (_tmp207 * _tmp232 - _tmp226 * _tmp37);
  const Scalar _tmp234 = _tmp186 * _tmp233;
  const Scalar _tmp235 = _tmp170 * _tmp176 + _tmp174;
  const Scalar _tmp236 = 0;
  const Scalar _tmp237 = _tmp178 * _tmp236;
  const Scalar _tmp238 = _tmp146 * _tmp237;
  const Scalar _tmp239 = _tmp130 * _tmp237 - _tmp164 * _tmp238;
  const Scalar _tmp240 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp241 = _tmp186 * _tmp240;
  const Scalar _tmp242 =
      -_tmp206 * _tmp213 - _tmp225 * _tmp228 - _tmp231 * _tmp234 - _tmp239 * _tmp241;
  const Scalar _tmp243 = Scalar(1.0) / (_tmp242);
  const Scalar _tmp244 = _tmp121 + _tmp171;
  const Scalar _tmp245 = _tmp176 * _tmp244;
  const Scalar _tmp246 = -_tmp125 + _tmp174 - _tmp245;
  const Scalar _tmp247 = Scalar(1.0) / (_tmp246);
  const Scalar _tmp248 = _tmp235 * _tmp247;
  const Scalar _tmp249 = -_tmp169 * _tmp237 + _tmp171 - _tmp244 * _tmp248;
  const Scalar _tmp250 = Scalar(1.0) * _tmp247;
  const Scalar _tmp251 = Scalar(1.0) * _tmp240;
  const Scalar _tmp252 = _tmp168 * _tmp190;
  const Scalar _tmp253 = _tmp169 * _tmp178;
  const Scalar _tmp254 = _tmp244 * _tmp247;
  const Scalar _tmp255 = _tmp200 * _tmp254 - _tmp201 * _tmp253 - _tmp252;
  const Scalar _tmp256 = -_tmp192 * _tmp255 + _tmp200 * _tmp250;
  const Scalar _tmp257 = Scalar(1.0) * _tmp212;
  const Scalar _tmp258 = _tmp245 * _tmp250 + Scalar(1.0);
  const Scalar _tmp259 = _tmp176 * _tmp250;
  const Scalar _tmp260 = -_tmp192 * _tmp258 + _tmp259;
  const Scalar _tmp261 = fh1 * (_tmp147 + _tmp166);
  const Scalar _tmp262 = _tmp210 * _tmp261;
  const Scalar _tmp263 = -_tmp207 * _tmp262 - Scalar(3.29616) * _tmp31 - _tmp37 * fv1;
  const Scalar _tmp264 = Scalar(1.0) * _tmp263;
  const Scalar _tmp265 = _tmp226 * _tmp261 + Scalar(3.29616) * _tmp45 + _tmp48 * fv1;
  const Scalar _tmp266 = _tmp173 * _tmp250;
  const Scalar _tmp267 = Scalar(1.0) * _tmp244 * _tmp266 - Scalar(1.0) * _tmp250;
  const Scalar _tmp268 = _tmp199 * _tmp250;
  const Scalar _tmp269 = -_tmp169 * _tmp229 + _tmp244 * _tmp268;
  const Scalar _tmp270 = -Scalar(1.0) * _tmp192 * _tmp269 + Scalar(1.0) * _tmp268;
  const Scalar _tmp271 = -_tmp169 * _tmp224 + _tmp216 + _tmp220 * _tmp254;
  const Scalar _tmp272 = -_tmp192 * _tmp271 + _tmp220 * _tmp250;
  const Scalar _tmp273 = Scalar(1.0) * _tmp227;
  const Scalar _tmp274 =
      _tmp233 * _tmp270 + _tmp251 * (-_tmp192 * _tmp249 - _tmp235 * _tmp250 + Scalar(1.0)) +
      _tmp256 * _tmp257 + _tmp260 * _tmp264 + _tmp265 * _tmp267 + _tmp272 * _tmp273;
  const Scalar _tmp275 = std::asinh(_tmp243 * _tmp274);
  const Scalar _tmp276 = Scalar(1.0) * _tmp275;
  const Scalar _tmp277 = Scalar(9.6622558468725703) * _tmp242;
  const Scalar _tmp278 = -_tmp152 + Scalar(-8.3196563700000006);
  const Scalar _tmp279 = -_tmp155 + Scalar(-1.9874742000000001);
  const Scalar _tmp280 =
      std::sqrt(Scalar(std::pow(_tmp278, Scalar(2)) + std::pow(_tmp279, Scalar(2))));
  const Scalar _tmp281 = -_tmp275 * _tmp277 - _tmp280;
  const Scalar _tmp282 = Scalar(0.1034955) * _tmp243;
  const Scalar _tmp283 = _tmp281 * _tmp282;
  const Scalar _tmp284 = -_tmp100 - _tmp101 - _tmp102 - _tmp99;
  const Scalar _tmp285 = _tmp77 + _tmp97;
  const Scalar _tmp286 = _tmp285 + _tmp94;
  const Scalar _tmp287 = _tmp284 + _tmp286;
  const Scalar _tmp288 = -_tmp88 - _tmp90;
  const Scalar _tmp289 = _tmp288 + _tmp93;
  const Scalar _tmp290 = _tmp65 + _tmp68 - _tmp71 - _tmp74;
  const Scalar _tmp291 = _tmp289 + _tmp290 + _tmp80;
  const Scalar _tmp292 = _tmp153 * _tmp291 + _tmp156 * _tmp287;
  const Scalar _tmp293 = _tmp157 * _tmp182 * _tmp292;
  const Scalar _tmp294 = _tmp206 * _tmp212;
  const Scalar _tmp295 = _tmp290 + _tmp92;
  const Scalar _tmp296 = _tmp295 + _tmp80;
  const Scalar _tmp297 = _tmp284 + _tmp98;
  const Scalar _tmp298 = _tmp297 + _tmp94;
  const Scalar _tmp299 =
      (2 * _tmp139 * _tmp298 + 2 * _tmp143 * _tmp296) / (_tmp144 * std::sqrt(_tmp144));
  const Scalar _tmp300 = Scalar(0.5) * _tmp198;
  const Scalar _tmp301 = _tmp292 / (_tmp180 * std::sqrt(_tmp180));
  const Scalar _tmp302 = _tmp182 * _tmp291;
  const Scalar _tmp303 = _tmp182 * _tmp287;
  const Scalar _tmp304 = _tmp186 * (-_tmp151 * _tmp303 + _tmp153 * _tmp303 + _tmp154 * _tmp302 -
                                    _tmp156 * _tmp302 + _tmp183 * _tmp301 - _tmp184 * _tmp301);
  const Scalar _tmp305 = _tmp287 / _tmp179;
  const Scalar _tmp306 = _tmp181 * _tmp305;
  const Scalar _tmp307 = _tmp185 * _tmp306;
  const Scalar _tmp308 = (Scalar(1) / Scalar(2)) * _tmp299;
  const Scalar _tmp309 = _tmp139 * _tmp308;
  const Scalar _tmp310 = _tmp157 * _tmp291;
  const Scalar _tmp311 = _tmp143 * _tmp308;
  const Scalar _tmp312 = _tmp153 * _tmp305;
  const Scalar _tmp313 = _tmp145 * _tmp298;
  const Scalar _tmp314 = _tmp145 * _tmp296;
  const Scalar _tmp315 = (_tmp146 * _tmp310 - _tmp146 * _tmp312 - _tmp158 * _tmp309 +
                          _tmp158 * _tmp313 + _tmp311 - _tmp314) /
                         std::pow(_tmp162, Scalar(2));
  const Scalar _tmp316 = _tmp160 * _tmp315;
  const Scalar _tmp317 = _tmp185 * _tmp293;
  const Scalar _tmp318 = -_tmp79 + _tmp95;
  const Scalar _tmp319 = _tmp295 + _tmp318;
  const Scalar _tmp320 = _tmp129 * _tmp319;
  const Scalar _tmp321 = _tmp288 + _tmp86;
  const Scalar _tmp322 = _tmp297 + _tmp321;
  const Scalar _tmp323 = _tmp129 * _tmp322;
  const Scalar _tmp324 = _tmp137 * _tmp311 - _tmp137 * _tmp314 + _tmp139 * _tmp314 -
                         _tmp141 * _tmp309 + _tmp146 * _tmp304 - _tmp146 * _tmp307 +
                         _tmp146 * _tmp317 - _tmp161 * _tmp298 - _tmp187 * _tmp309 +
                         _tmp187 * _tmp313 + _tmp188 * _tmp298;
  const Scalar _tmp325 =
      (2 * _tmp123 * _tmp319 + 2 * _tmp127 * _tmp322) / (_tmp128 * std::sqrt(_tmp128));
  const Scalar _tmp326 = (Scalar(1) / Scalar(2)) * _tmp325;
  const Scalar _tmp327 = _tmp127 * _tmp326;
  const Scalar _tmp328 = _tmp123 * _tmp326;
  const Scalar _tmp329 = _tmp130 * _tmp310 - _tmp130 * _tmp312 + _tmp158 * _tmp323 -
                         _tmp158 * _tmp327 - _tmp320 + _tmp328;
  const Scalar _tmp330 = _tmp163 * _tmp329;
  const Scalar _tmp331 = _tmp121 * _tmp323 - _tmp121 * _tmp327 - _tmp125 * _tmp320 +
                         _tmp125 * _tmp328 + _tmp127 * _tmp320 + _tmp130 * _tmp304 -
                         _tmp130 * _tmp307 + _tmp130 * _tmp317 - _tmp159 * _tmp322 -
                         _tmp164 * _tmp324 + _tmp187 * _tmp323 - _tmp187 * _tmp327 +
                         _tmp189 * _tmp316 - _tmp189 * _tmp330;
  const Scalar _tmp332 = _tmp331 / std::pow(_tmp191, Scalar(2));
  const Scalar _tmp333 = Scalar(1.0) * _tmp332;
  const Scalar _tmp334 = _tmp164 * _tmp313;
  const Scalar _tmp335 = _tmp190 * _tmp198 * _tmp329;
  const Scalar _tmp336 = _tmp285 + _tmp321;
  const Scalar _tmp337 = _tmp284 + _tmp336;
  const Scalar _tmp338 = _tmp210 * _tmp337;
  const Scalar _tmp339 = _tmp338 * fh1;
  const Scalar _tmp340 = _tmp186 * _tmp225;
  const Scalar _tmp341 = -_tmp110 - _tmp111 - _tmp112 - _tmp113 + _tmp118;
  const Scalar _tmp342 = _tmp109 + _tmp341;
  const Scalar _tmp343 = -_tmp105 - _tmp106 + _tmp107 + _tmp108;
  const Scalar _tmp344 = _tmp119 + _tmp343;
  const Scalar _tmp345 = _tmp12 * _tmp96;
  const Scalar _tmp346 = _tmp16 * _tmp78;
  const Scalar _tmp347 = Scalar(0.83999999999999997) * _tmp24 * _tmp69 + _tmp66 * _tmp76;
  const Scalar _tmp348 = _tmp345 + _tmp346 + _tmp347;
  const Scalar _tmp349 = _tmp348 / std::pow(_tmp172, Scalar(2));
  const Scalar _tmp350 = _tmp175 * _tmp349;
  const Scalar _tmp351 = _tmp148 * _tmp323;
  const Scalar _tmp352 = _tmp341 + _tmp343;
  const Scalar _tmp353 = _tmp158 * _tmp342;
  const Scalar _tmp354 = -_tmp135 * _tmp311 + _tmp135 * _tmp314 - _tmp146 * _tmp353 -
                         _tmp149 * _tmp310 + _tmp149 * _tmp312 + _tmp161 * _tmp352 +
                         _tmp214 * _tmp309 - _tmp214 * _tmp313;
  const Scalar _tmp355 = -_tmp130 * _tmp353 - _tmp158 * _tmp351 + _tmp159 * _tmp344 -
                         _tmp164 * _tmp354 - _tmp165 * _tmp310 + _tmp165 * _tmp312 +
                         _tmp167 * _tmp320 - _tmp167 * _tmp328 + _tmp168 * _tmp316 -
                         _tmp168 * _tmp330 + _tmp214 * _tmp327;
  const Scalar _tmp356 = _tmp135 * _tmp309 - _tmp135 * _tmp313 + _tmp146 * _tmp342 -
                         _tmp146 * _tmp352 - _tmp148 * _tmp309 + _tmp148 * _tmp313;
  const Scalar _tmp357 = Scalar(1.6799999999999999) * _tmp21 * _tmp69;
  const Scalar _tmp358 = Scalar(1.6799999999999999) * _tmp114 + _tmp357;
  const Scalar _tmp359 = _tmp173 * _tmp358;
  const Scalar _tmp360 = _tmp130 * _tmp342 - _tmp130 * _tmp344 - _tmp148 * _tmp327 +
                         _tmp150 * _tmp316 - _tmp150 * _tmp330 - _tmp164 * _tmp356 -
                         _tmp167 * _tmp323 + _tmp167 * _tmp327 + _tmp169 * _tmp350 -
                         _tmp169 * _tmp359 - _tmp176 * _tmp355 + _tmp351;
  const Scalar _tmp361 = _tmp360 / std::pow(_tmp177, Scalar(2));
  const Scalar _tmp362 = _tmp236 * _tmp361;
  const Scalar _tmp363 = _tmp289 + _tmp318;
  const Scalar _tmp364 = _tmp290 + _tmp363;
  const Scalar _tmp365 =
      (2 * _tmp207 * _tmp364 + 2 * _tmp208 * _tmp337) / (_tmp209 * std::sqrt(_tmp209));
  const Scalar _tmp366 = (Scalar(1) / Scalar(2)) * _tmp365;
  const Scalar _tmp367 = _tmp208 * _tmp366;
  const Scalar _tmp368 = _tmp367 * fh1;
  const Scalar _tmp369 = _tmp207 * _tmp366;
  const Scalar _tmp370 = fh1 * (_tmp207 * _tmp338 - _tmp226 * _tmp364 + _tmp232 * _tmp364 -
                                _tmp338 * _tmp37 + _tmp367 * _tmp37 - _tmp369 * _tmp48);
  const Scalar _tmp371 = _tmp231 * _tmp233;
  const Scalar _tmp372 = _tmp239 * _tmp240;
  const Scalar _tmp373 = _tmp225 * _tmp227;
  const Scalar _tmp374 = _tmp163 * _tmp204;
  const Scalar _tmp375 = _tmp160 * _tmp361;
  const Scalar _tmp376 = _tmp177 * _tmp332;
  const Scalar _tmp377 = _tmp197 * _tmp376;
  const Scalar _tmp378 = Scalar(1.0) * _tmp315;
  const Scalar _tmp379 = _tmp150 * _tmp378 + _tmp163 * _tmp168 * _tmp192 * _tmp358 -
                         _tmp168 * _tmp193 * _tmp315 - _tmp190 * _tmp356 + _tmp194 * _tmp354 -
                         _tmp252 * _tmp350;
  const Scalar _tmp380 = _tmp178 * _tmp191;
  const Scalar _tmp381 = _tmp191 * _tmp361;
  const Scalar _tmp382 = _tmp199 * (_tmp189 * _tmp378 - _tmp190 * _tmp324 + _tmp195 * _tmp381 -
                                    _tmp196 * _tmp331 - _tmp379 * _tmp380);
  const Scalar _tmp383 = _tmp198 * _tmp360;
  const Scalar _tmp384 = _tmp197 * _tmp383;
  const Scalar _tmp385 = -_tmp377 + _tmp379 + _tmp382 + _tmp384;
  const Scalar _tmp386 = _tmp201 * _tmp375 - _tmp202 * _tmp329 - _tmp203 * _tmp385;
  const Scalar _tmp387 = _tmp146 * _tmp315;
  const Scalar _tmp388 = _tmp130 * _tmp361;
  const Scalar _tmp389 = _tmp130 * _tmp178;
  const Scalar _tmp390 = _tmp210 * _tmp364;
  const Scalar _tmp391 = _tmp390 * fh1;
  const Scalar _tmp392 = _tmp186 * _tmp206;
  const Scalar _tmp393 = _tmp219 * _tmp383;
  const Scalar _tmp394 = _tmp163 * _tmp312;
  const Scalar _tmp395 = _tmp163 * _tmp310;
  const Scalar _tmp396 = _tmp158 * _tmp315;
  const Scalar _tmp397 = _tmp148 * _tmp310 - _tmp148 * _tmp312 - _tmp168 * _tmp394 +
                         _tmp168 * _tmp395 - _tmp168 * _tmp396 + _tmp215 * _tmp354 + _tmp353;
  const Scalar _tmp398 = _tmp110 + _tmp111 + _tmp112 + _tmp113 - _tmp115 - _tmp117 -
                         _tmp150 * _tmp394 + _tmp150 * _tmp395 - _tmp150 * _tmp396 -
                         _tmp176 * _tmp397 + _tmp215 * _tmp356 + _tmp216 * _tmp350 -
                         _tmp216 * _tmp359 + _tmp343;
  const Scalar _tmp399 = _tmp199 * (-_tmp189 * _tmp394 + _tmp189 * _tmp395 - _tmp189 * _tmp396 +
                                    _tmp215 * _tmp324 + _tmp217 * _tmp381 - _tmp218 * _tmp331 -
                                    _tmp304 + _tmp307 - _tmp317 - _tmp380 * _tmp398);
  const Scalar _tmp400 = _tmp219 * _tmp376;
  const Scalar _tmp401 = _tmp393 + _tmp398 + _tmp399 - _tmp400;
  const Scalar _tmp402 =
      -_tmp203 * _tmp401 + _tmp221 * _tmp375 - _tmp224 * _tmp329 - _tmp310 + _tmp312;
  const Scalar _tmp403 = _tmp369 * fh1;
  const Scalar _tmp404 =
      -_tmp186 * _tmp231 * _tmp370 -
      _tmp213 * (-_tmp201 * _tmp388 + _tmp202 * _tmp323 - _tmp202 * _tmp327 - _tmp204 * _tmp387 +
                 _tmp205 * _tmp386 - _tmp309 * _tmp374 + _tmp313 * _tmp374 + _tmp385 * _tmp389) -
      _tmp228 * (_tmp205 * _tmp402 - _tmp221 * _tmp388 - _tmp222 * _tmp387 - _tmp223 * _tmp309 +
                 _tmp223 * _tmp313 + _tmp224 * _tmp323 - _tmp224 * _tmp327 + _tmp389 * _tmp401) -
      _tmp234 * (-_tmp127 * _tmp300 * _tmp325 - _tmp130 * _tmp333 +
                 _tmp139 * _tmp164 * _tmp299 * _tmp300 + _tmp146 * _tmp229 * _tmp316 -
                 _tmp146 * _tmp335 + _tmp229 * _tmp323 - _tmp229 * _tmp334 + _tmp230 * _tmp333) -
      _tmp241 * (-_tmp130 * _tmp362 + _tmp164 * _tmp237 * _tmp309 + _tmp230 * _tmp362 +
                 _tmp237 * _tmp323 - _tmp237 * _tmp327 - _tmp237 * _tmp334 + _tmp238 * _tmp316 -
                 _tmp238 * _tmp330) -
      _tmp293 * _tmp294 - _tmp293 * _tmp371 - _tmp293 * _tmp372 - _tmp293 * _tmp373 +
      _tmp294 * _tmp306 + _tmp306 * _tmp371 + _tmp306 * _tmp372 + _tmp306 * _tmp373 -
      _tmp339 * _tmp340 + _tmp340 * _tmp368 - _tmp391 * _tmp392 + _tmp392 * _tmp403;
  const Scalar _tmp405 = Scalar(9.6622558468725703) * _tmp404;
  const Scalar _tmp406 = std::pow(_tmp242, Scalar(-2));
  const Scalar _tmp407 = _tmp404 * _tmp406;
  const Scalar _tmp408 = Scalar(6.59232) * _tmp24;
  const Scalar _tmp409 = _tmp120 * fh1;
  const Scalar _tmp410 = Scalar(6.59232) * _tmp66;
  const Scalar _tmp411 = Scalar(6.59232) * _tmp69;
  const Scalar _tmp412 = Scalar(6.59232) * _tmp21;
  const Scalar _tmp413 = _tmp12 * _tmp410 - _tmp16 * _tmp411 - _tmp211 * _tmp409 +
                         _tmp261 * _tmp369 - _tmp262 * _tmp364 - _tmp364 * fv1 + _tmp408 * _tmp63 -
                         _tmp412 * _tmp72;
  const Scalar _tmp414 = Scalar(1.0) * _tmp349;
  const Scalar _tmp415 = -Scalar(1.6799999999999999) * _tmp116 + _tmp348 - _tmp357;
  const Scalar _tmp416 = _tmp247 * _tmp415;
  const Scalar _tmp417 = _tmp169 * _tmp361;
  const Scalar _tmp418 = _tmp176 * _tmp415;
  const Scalar _tmp419 = _tmp244 * _tmp350;
  const Scalar _tmp420 = _tmp244 * _tmp359;
  const Scalar _tmp421 = (-_tmp345 - _tmp346 + _tmp347 + _tmp358 - _tmp418 + _tmp419 - _tmp420) /
                         std::pow(_tmp246, Scalar(2));
  const Scalar _tmp422 = _tmp244 * _tmp421;
  const Scalar _tmp423 = _tmp168 * _tmp378 - _tmp190 * _tmp354 + _tmp200 * _tmp416 -
                         _tmp200 * _tmp422 + _tmp201 * _tmp417 - _tmp202 * _tmp355 -
                         _tmp253 * _tmp385 - _tmp254 * _tmp377 + _tmp254 * _tmp382 +
                         _tmp254 * _tmp384;
  const Scalar _tmp424 = _tmp177 * _tmp229;
  const Scalar _tmp425 = _tmp421 * _tmp424;
  const Scalar _tmp426 = _tmp250 * _tmp376;
  const Scalar _tmp427 = _tmp250 * _tmp383;
  const Scalar _tmp428 = _tmp169 * _tmp333 - _tmp229 * _tmp355 - _tmp244 * _tmp426 +
                         _tmp244 * _tmp427 + _tmp268 * _tmp415 - _tmp422 * _tmp424;
  const Scalar _tmp429 = _tmp12 * _tmp411 + _tmp16 * _tmp410 + _tmp226 * _tmp409 +
                         _tmp261 * _tmp338 - _tmp261 * _tmp367 + _tmp337 * fv1 + _tmp408 * _tmp72 +
                         _tmp412 * _tmp63;
  const Scalar _tmp430 = _tmp256 * fh1;
  const Scalar _tmp431 = Scalar(0.5) * _tmp365;
  const Scalar _tmp432 = _tmp220 * _tmp416 - _tmp220 * _tmp422 + _tmp221 * _tmp417 -
                         _tmp224 * _tmp355 - _tmp253 * _tmp401 + _tmp254 * _tmp393 +
                         _tmp254 * _tmp399 - _tmp254 * _tmp400 + _tmp397;
  const Scalar _tmp433 = _tmp249 * _tmp349;
  const Scalar _tmp434 = -_tmp170 * _tmp350 + _tmp170 * _tmp359 + _tmp193 * _tmp291 + _tmp287;
  const Scalar _tmp435 = _tmp235 * _tmp421;
  const Scalar _tmp436 = _tmp247 * _tmp434;
  const Scalar _tmp437 = _tmp318 + _tmp75 + _tmp92;
  const Scalar _tmp438 = _tmp169 * _tmp362 - _tmp237 * _tmp355 + _tmp244 * _tmp435 -
                         _tmp244 * _tmp436 - _tmp248 * _tmp415 + _tmp437;
  const Scalar _tmp439 = Scalar(1.0) * _tmp421;
  const Scalar _tmp440 = _tmp244 * _tmp349;
  const Scalar _tmp441 = _tmp192 * _tmp422;
  const Scalar _tmp442 = Scalar(1.0) * _tmp265;
  const Scalar _tmp443 = _tmp193 * _tmp421;
  const Scalar _tmp444 = _tmp258 * _tmp349;
  const Scalar _tmp445 =
      -_tmp193 * _tmp422 + _tmp250 * _tmp418 - _tmp250 * _tmp419 + _tmp250 * _tmp420;
  const Scalar _tmp446 = _tmp250 * _tmp350;
  const Scalar _tmp447 =
      (_tmp243 * (-_tmp207 * _tmp430 * _tmp431 - _tmp208 * _tmp272 * _tmp431 * fh1 +
                  Scalar(1.0) * _tmp233 *
                      (-_tmp192 * _tmp428 + _tmp269 * _tmp414 - _tmp425 - _tmp426 + _tmp427) +
                  _tmp251 * (-_tmp192 * _tmp438 + _tmp235 * _tmp439 - _tmp250 * _tmp434 +
                             Scalar(1.0) * _tmp433) +
                  _tmp257 * (-_tmp192 * _tmp423 - _tmp197 * _tmp425 - _tmp250 * _tmp377 +
                             _tmp250 * _tmp382 + _tmp250 * _tmp384 + _tmp255 * _tmp414) +
                  Scalar(1.0) * _tmp260 * _tmp413 +
                  _tmp264 * (-_tmp192 * _tmp445 + _tmp250 * _tmp359 - _tmp443 +
                             Scalar(1.0) * _tmp444 - _tmp446) +
                  _tmp267 * _tmp429 + _tmp270 * _tmp370 + Scalar(1.0) * _tmp272 * _tmp339 +
                  _tmp273 * (-_tmp192 * _tmp432 - _tmp219 * _tmp425 + _tmp250 * _tmp393 +
                             _tmp250 * _tmp399 - _tmp250 * _tmp400 + _tmp271 * _tmp414) +
                  Scalar(1.0) * _tmp390 * _tmp430 +
                  _tmp442 * (-_tmp250 * _tmp440 + _tmp266 * _tmp415 + _tmp439 - _tmp441)) -
       _tmp274 * _tmp407) /
      std::sqrt(Scalar(std::pow(_tmp274, Scalar(2)) * _tmp406 + 1));
  const Scalar _tmp448 = _tmp173 * _tmp240;
  const Scalar _tmp449 = _tmp173 * _tmp233;
  const Scalar _tmp450 = _tmp173 * _tmp255;
  const Scalar _tmp451 = _tmp173 * _tmp227;
  const Scalar _tmp452 = _tmp173 * _tmp263;
  const Scalar _tmp453 = _tmp250 * _tmp265;
  const Scalar _tmp454 = _tmp173 * _tmp244;
  const Scalar _tmp455 = _tmp212 * _tmp450 + _tmp249 * _tmp448 + _tmp258 * _tmp452 +
                         _tmp269 * _tmp449 + _tmp271 * _tmp451 - _tmp453 * _tmp454;
  const Scalar _tmp456 = _tmp237 * _tmp240;
  const Scalar _tmp457 = _tmp229 * _tmp233;
  const Scalar _tmp458 =
      -_tmp164 * _tmp456 - _tmp164 * _tmp457 + _tmp212 * _tmp374 + _tmp223 * _tmp227;
  const Scalar _tmp459 = Scalar(1.0) / (_tmp458);
  const Scalar _tmp460 = std::asinh(_tmp455 * _tmp459);
  const Scalar _tmp461 = Scalar(1.0) * _tmp460;
  const Scalar _tmp462 = std::pow(_tmp458, Scalar(-2));
  const Scalar _tmp463 = _tmp229 * _tmp370;
  const Scalar _tmp464 = _tmp240 * _tmp362;
  const Scalar _tmp465 = _tmp233 * _tmp333;
  const Scalar _tmp466 = _tmp163 * _tmp212 * _tmp386 + _tmp163 * _tmp227 * _tmp402 -
                         _tmp164 * _tmp463 + _tmp164 * _tmp464 + _tmp164 * _tmp465 -
                         _tmp204 * _tmp212 * _tmp315 - _tmp222 * _tmp227 * _tmp315 +
                         _tmp223 * _tmp339 - _tmp223 * _tmp368 - _tmp233 * _tmp335 +
                         _tmp316 * _tmp456 + _tmp316 * _tmp457 - _tmp330 * _tmp456 +
                         _tmp374 * _tmp391 - _tmp374 * _tmp403;
  const Scalar _tmp467 = _tmp462 * _tmp466;
  const Scalar _tmp468 = _tmp173 * _tmp271;
  const Scalar _tmp469 = _tmp250 * _tmp429;
  const Scalar _tmp470 =
      (-_tmp455 * _tmp467 +
       _tmp459 * (_tmp173 * _tmp212 * _tmp423 + _tmp173 * _tmp258 * _tmp413 +
                  _tmp173 * _tmp269 * _tmp370 - _tmp173 * _tmp415 * _tmp453 -
                  _tmp212 * _tmp255 * _tmp349 - _tmp227 * _tmp271 * _tmp349 -
                  _tmp233 * _tmp269 * _tmp349 - _tmp240 * _tmp433 - _tmp263 * _tmp444 +
                  _tmp265 * _tmp441 + _tmp339 * _tmp468 - _tmp368 * _tmp468 + _tmp391 * _tmp450 -
                  _tmp403 * _tmp450 + _tmp428 * _tmp449 + _tmp432 * _tmp451 + _tmp438 * _tmp448 +
                  _tmp440 * _tmp453 + _tmp445 * _tmp452 - _tmp454 * _tmp469)) /
      std::sqrt(Scalar(std::pow(_tmp455, Scalar(2)) * _tmp462 + 1));
  const Scalar _tmp471 = Scalar(9.6622558468725703) * _tmp466;
  const Scalar _tmp472 = Scalar(9.6622558468725703) * _tmp458;
  const Scalar _tmp473 = -_tmp142 + Scalar(-8.3888750099999996);
  const Scalar _tmp474 = Scalar(2.5202214700000001) - _tmp138;
  const Scalar _tmp475 =
      std::sqrt(Scalar(std::pow(_tmp473, Scalar(2)) + std::pow(_tmp474, Scalar(2))));
  const Scalar _tmp476 = Scalar(0.1034955) * _tmp459;
  const Scalar _tmp477 = -_tmp460 * _tmp472 - _tmp475;
  const Scalar _tmp478 = _tmp476 * _tmp477;
  const Scalar _tmp479 = Scalar(4.7752063900000001) - _tmp122;
  const Scalar _tmp480 = Scalar(2.71799795) - _tmp126;
  const Scalar _tmp481 =
      std::sqrt(Scalar(std::pow(_tmp479, Scalar(2)) + std::pow(_tmp480, Scalar(2))));
  const Scalar _tmp482 = _tmp250 * _tmp452;
  const Scalar _tmp483 = _tmp200 * _tmp247;
  const Scalar _tmp484 = _tmp227 * _tmp247;
  const Scalar _tmp485 = -_tmp175 * _tmp482 - _tmp212 * _tmp483 - _tmp220 * _tmp484 -
                         _tmp233 * _tmp268 + _tmp240 * _tmp248 + _tmp453;
  const Scalar _tmp486 = _tmp202 * _tmp212 + _tmp224 * _tmp227 + _tmp456 + _tmp457;
  const Scalar _tmp487 = Scalar(1.0) / (_tmp486);
  const Scalar _tmp488 = std::asinh(_tmp485 * _tmp487);
  const Scalar _tmp489 = Scalar(9.6622558468725703) * _tmp486;
  const Scalar _tmp490 = -_tmp481 - _tmp488 * _tmp489;
  const Scalar _tmp491 = std::pow(_tmp486, Scalar(-2));
  const Scalar _tmp492 = _tmp178 * _tmp212 * _tmp385 + _tmp178 * _tmp227 * _tmp401 -
                         _tmp201 * _tmp212 * _tmp361 + _tmp202 * _tmp391 - _tmp202 * _tmp403 -
                         _tmp221 * _tmp227 * _tmp361 + _tmp224 * _tmp339 - _tmp224 * _tmp368 +
                         _tmp463 - _tmp464 - _tmp465;
  const Scalar _tmp493 = _tmp491 * _tmp492;
  const Scalar _tmp494 = Scalar(9.6622558468725703) * _tmp492;
  const Scalar _tmp495 = _tmp212 * _tmp247;
  const Scalar _tmp496 = _tmp220 * _tmp247;
  const Scalar _tmp497 =
      (-_tmp485 * _tmp493 +
       _tmp487 * (_tmp177 * _tmp421 * _tmp457 + _tmp200 * _tmp212 * _tmp421 +
                  _tmp220 * _tmp227 * _tmp421 + _tmp233 * _tmp426 - _tmp233 * _tmp427 -
                  _tmp240 * _tmp435 + _tmp240 * _tmp436 - _tmp259 * _tmp413 + _tmp263 * _tmp443 +
                  _tmp263 * _tmp446 - _tmp268 * _tmp370 - _tmp339 * _tmp496 - _tmp358 * _tmp482 +
                  _tmp368 * _tmp496 + _tmp377 * _tmp495 - _tmp382 * _tmp495 - _tmp384 * _tmp495 -
                  _tmp391 * _tmp483 - _tmp393 * _tmp484 - _tmp399 * _tmp484 + _tmp400 * _tmp484 +
                  _tmp403 * _tmp483 - _tmp421 * _tmp442 + _tmp469)) /
      std::sqrt(Scalar(std::pow(_tmp485, Scalar(2)) * _tmp491 + 1));
  const Scalar _tmp498 = Scalar(0.1034955) * _tmp487;
  const Scalar _tmp499 = _tmp490 * _tmp498;
  const Scalar _tmp500 = Scalar(1.0) * _tmp488;

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) =
      _tmp120 -
      Scalar(0.5) * (2 * _tmp39 * (_tmp81 + _tmp92) + 2 * _tmp50 * (_tmp104 + _tmp94)) *
          std::sinh(Scalar(0.1034955) * _tmp52 *
                    (-_tmp51 - Scalar(9.6622558468725703) * fh1 * std::asinh(_tmp52 * fv1))) /
          _tmp51;
  _res(1, 0) =
      -_tmp277 *
          (-Scalar(0.87679799772039002) * _tmp407 + Scalar(1.0) * _tmp447 * std::sinh(_tmp276) -
           (-Scalar(0.1034955) * _tmp281 * _tmp407 +
            _tmp282 * (-_tmp275 * _tmp405 - _tmp277 * _tmp447 -
                       Scalar(1) / Scalar(2) *
                           (2 * _tmp278 * _tmp437 + 2 * _tmp279 * (_tmp104 + _tmp321)) / _tmp280)) *
               std::sinh(_tmp283)) +
      _tmp342 -
      _tmp405 * (Scalar(0.87679799772039002) * _tmp243 + std::cosh(_tmp276) - std::cosh(_tmp283));
  _res(2, 0) =
      _tmp352 -
      _tmp471 * (Scalar(0.87653584775870996) * _tmp459 + std::cosh(_tmp461) - std::cosh(_tmp478)) -
      _tmp472 *
          (-Scalar(0.87653584775870996) * _tmp467 + Scalar(1.0) * _tmp470 * std::sinh(_tmp461) -
           (-Scalar(0.1034955) * _tmp467 * _tmp477 +
            _tmp476 * (-_tmp460 * _tmp471 - _tmp470 * _tmp472 -
                       Scalar(1) / Scalar(2) *
                           (2 * _tmp473 * (_tmp363 + _tmp75) + 2 * _tmp474 * (_tmp103 + _tmp336)) /
                           _tmp475)) *
               std::sinh(_tmp478));
  _res(3, 0) =
      _tmp344 -
      _tmp489 *
          (-Scalar(0.86565325453551001) * _tmp493 + Scalar(1.0) * _tmp497 * std::sinh(_tmp500) -
           (-Scalar(0.1034955) * _tmp490 * _tmp493 +
            _tmp498 * (-_tmp488 * _tmp494 - _tmp489 * _tmp497 -
                       Scalar(1) / Scalar(2) *
                           (2 * _tmp479 * (_tmp289 + _tmp81) + 2 * _tmp480 * (_tmp103 + _tmp286)) /
                           _tmp481)) *
               std::sinh(_tmp499)) -
      _tmp494 * (Scalar(0.86565325453551001) * _tmp487 - std::cosh(_tmp499) + std::cosh(_tmp500));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
