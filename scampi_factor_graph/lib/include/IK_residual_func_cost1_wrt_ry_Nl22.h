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
 * Symbolic function: IK_residual_func_cost1_wrt_ry_Nl22
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost1WrtRyNl22(
    const Scalar fh1, const Scalar fv1, const Scalar rx, const Scalar ry, const Scalar rz,
    const Scalar p_init0, const Scalar p_init1, const Scalar p_init2, const Scalar rot_init_x,
    const Scalar rot_init_y, const Scalar rot_init_z, const Scalar rot_init_w,
    const Scalar epsilon) {
  // Total ops: 1613

  // Unused inputs
  (void)p_init2;
  (void)epsilon;

  // Input arrays

  // Intermediate terms (495)
  const Scalar _tmp0 = std::pow(ry, Scalar(2));
  const Scalar _tmp1 = _tmp0 + std::pow(rx, Scalar(2)) + std::pow(rz, Scalar(2));
  const Scalar _tmp2 = std::sqrt(_tmp1);
  const Scalar _tmp3 = (Scalar(1) / Scalar(2)) * _tmp2;
  const Scalar _tmp4 = std::cos(_tmp3);
  const Scalar _tmp5 = _tmp4 * rot_init_y;
  const Scalar _tmp6 = std::sin(_tmp3);
  const Scalar _tmp7 = _tmp6 / _tmp2;
  const Scalar _tmp8 = _tmp7 * rot_init_w;
  const Scalar _tmp9 = _tmp8 * ry;
  const Scalar _tmp10 = _tmp7 * rot_init_z;
  const Scalar _tmp11 = _tmp7 * rot_init_x;
  const Scalar _tmp12 = _tmp10 * rx - _tmp11 * rz + _tmp5 + _tmp9;
  const Scalar _tmp13 = _tmp4 * rot_init_x;
  const Scalar _tmp14 = _tmp10 * ry;
  const Scalar _tmp15 = _tmp7 * rot_init_y;
  const Scalar _tmp16 = _tmp13 - _tmp14 + _tmp15 * rz + _tmp8 * rx;
  const Scalar _tmp17 = 2 * _tmp16;
  const Scalar _tmp18 = _tmp12 * _tmp17;
  const Scalar _tmp19 = _tmp4 * rot_init_z;
  const Scalar _tmp20 = _tmp11 * ry;
  const Scalar _tmp21 = -_tmp15 * rx + _tmp19 + _tmp20 + _tmp8 * rz;
  const Scalar _tmp22 = _tmp4 * rot_init_w;
  const Scalar _tmp23 = _tmp15 * ry;
  const Scalar _tmp24 = -_tmp10 * rz - _tmp11 * rx + _tmp22 - _tmp23;
  const Scalar _tmp25 = 2 * _tmp24;
  const Scalar _tmp26 = _tmp21 * _tmp25;
  const Scalar _tmp27 = Scalar(0.20999999999999999) * _tmp18 + Scalar(0.20999999999999999) * _tmp26;
  const Scalar _tmp28 = -_tmp27;
  const Scalar _tmp29 = 2 * _tmp12 * _tmp21;
  const Scalar _tmp30 = _tmp16 * _tmp25;
  const Scalar _tmp31 = _tmp29 - _tmp30;
  const Scalar _tmp32 = -Scalar(0.010999999999999999) * _tmp31;
  const Scalar _tmp33 = -2 * std::pow(_tmp16, Scalar(2));
  const Scalar _tmp34 = 1 - 2 * std::pow(_tmp21, Scalar(2));
  const Scalar _tmp35 = Scalar(0.20999999999999999) * _tmp33 + Scalar(0.20999999999999999) * _tmp34;
  const Scalar _tmp36 = _tmp32 + _tmp35;
  const Scalar _tmp37 = _tmp28 + _tmp36;
  const Scalar _tmp38 = _tmp37 + p_init1;
  const Scalar _tmp39 = Scalar(4.8333311099999996) - _tmp38;
  const Scalar _tmp40 = -2 * std::pow(_tmp12, Scalar(2));
  const Scalar _tmp41 = Scalar(0.20999999999999999) * _tmp34 + Scalar(0.20999999999999999) * _tmp40;
  const Scalar _tmp42 = -_tmp41;
  const Scalar _tmp43 = Scalar(0.20999999999999999) * _tmp18 - Scalar(0.20999999999999999) * _tmp26;
  const Scalar _tmp44 = _tmp17 * _tmp21;
  const Scalar _tmp45 = _tmp12 * _tmp25;
  const Scalar _tmp46 = _tmp44 + _tmp45;
  const Scalar _tmp47 = -Scalar(0.010999999999999999) * _tmp46;
  const Scalar _tmp48 = _tmp43 + _tmp47;
  const Scalar _tmp49 = _tmp42 + _tmp48;
  const Scalar _tmp50 = _tmp49 + p_init0;
  const Scalar _tmp51 = -_tmp50 + Scalar(-1.79662371);
  const Scalar _tmp52 =
      std::sqrt(Scalar(std::pow(_tmp39, Scalar(2)) + std::pow(_tmp51, Scalar(2))));
  const Scalar _tmp53 = Scalar(1.0) / (fh1);
  const Scalar _tmp54 = (Scalar(1) / Scalar(2)) / _tmp1;
  const Scalar _tmp55 = _tmp0 * _tmp54;
  const Scalar _tmp56 = _tmp54 * ry;
  const Scalar _tmp57 = _tmp56 * rz;
  const Scalar _tmp58 = _tmp56 * rx;
  const Scalar _tmp59 = _tmp6 / (_tmp1 * std::sqrt(_tmp1));
  const Scalar _tmp60 = _tmp0 * _tmp59;
  const Scalar _tmp61 = _tmp59 * ry;
  const Scalar _tmp62 = _tmp61 * rx;
  const Scalar _tmp63 = _tmp61 * rz;
  const Scalar _tmp64 = -_tmp10 - _tmp19 * _tmp55 - Scalar(1) / Scalar(2) * _tmp20 +
                        _tmp22 * _tmp58 + _tmp5 * _tmp57 + _tmp60 * rot_init_z -
                        _tmp62 * rot_init_w - _tmp63 * rot_init_y;
  const Scalar _tmp65 = Scalar(0.021999999999999999) * _tmp24;
  const Scalar _tmp66 = _tmp64 * _tmp65;
  const Scalar _tmp67 = -_tmp13 * _tmp58 - _tmp15 - _tmp19 * _tmp57 - _tmp5 * _tmp55 +
                        _tmp60 * rot_init_y + _tmp62 * rot_init_x + _tmp63 * rot_init_z -
                        Scalar(1) / Scalar(2) * _tmp9;
  const Scalar _tmp68 = Scalar(0.021999999999999999) * _tmp67;
  const Scalar _tmp69 = _tmp16 * _tmp68;
  const Scalar _tmp70 = _tmp11 + _tmp13 * _tmp55 - Scalar(1) / Scalar(2) * _tmp14 +
                        _tmp22 * _tmp57 - _tmp5 * _tmp58 - _tmp60 * rot_init_x +
                        _tmp62 * rot_init_y - _tmp63 * rot_init_w;
  const Scalar _tmp71 = Scalar(0.021999999999999999) * _tmp70;
  const Scalar _tmp72 = _tmp12 * _tmp71;
  const Scalar _tmp73 = -_tmp13 * _tmp57 + _tmp19 * _tmp58 + _tmp22 * _tmp55 -
                        Scalar(1) / Scalar(2) * _tmp23 - _tmp60 * rot_init_w - _tmp62 * rot_init_z +
                        _tmp63 * rot_init_x + _tmp8;
  const Scalar _tmp74 = Scalar(0.021999999999999999) * _tmp21;
  const Scalar _tmp75 = _tmp73 * _tmp74;
  const Scalar _tmp76 = -_tmp66 - _tmp69 + _tmp72 + _tmp75;
  const Scalar _tmp77 = Scalar(0.83999999999999997) * _tmp70;
  const Scalar _tmp78 = _tmp21 * _tmp77;
  const Scalar _tmp79 = _tmp16 * _tmp64;
  const Scalar _tmp80 = Scalar(0.83999999999999997) * _tmp79;
  const Scalar _tmp81 = _tmp78 + _tmp80;
  const Scalar _tmp82 = _tmp16 * _tmp73;
  const Scalar _tmp83 = Scalar(0.41999999999999998) * _tmp82;
  const Scalar _tmp84 = _tmp12 * _tmp64;
  const Scalar _tmp85 = Scalar(0.41999999999999998) * _tmp84;
  const Scalar _tmp86 = _tmp83 + _tmp85;
  const Scalar _tmp87 = Scalar(0.41999999999999998) * _tmp70;
  const Scalar _tmp88 = _tmp24 * _tmp87;
  const Scalar _tmp89 = Scalar(0.41999999999999998) * _tmp21;
  const Scalar _tmp90 = _tmp67 * _tmp89;
  const Scalar _tmp91 = _tmp88 + _tmp90;
  const Scalar _tmp92 = _tmp86 + _tmp91;
  const Scalar _tmp93 = _tmp81 + _tmp92;
  const Scalar _tmp94 = _tmp12 * _tmp73;
  const Scalar _tmp95 = Scalar(0.83999999999999997) * _tmp94;
  const Scalar _tmp96 = -_tmp78;
  const Scalar _tmp97 = -_tmp95 + _tmp96;
  const Scalar _tmp98 = -_tmp83 - _tmp85;
  const Scalar _tmp99 = _tmp91 + _tmp98;
  const Scalar _tmp100 = _tmp97 + _tmp99;
  const Scalar _tmp101 = _tmp65 * _tmp73;
  const Scalar _tmp102 = _tmp12 * _tmp68;
  const Scalar _tmp103 = _tmp16 * _tmp71;
  const Scalar _tmp104 = _tmp64 * _tmp74;
  const Scalar _tmp105 = _tmp101 + _tmp102 + _tmp103 + _tmp104;
  const Scalar _tmp106 = Scalar(0.41999999999999998) * _tmp24;
  const Scalar _tmp107 = _tmp106 * _tmp73;
  const Scalar _tmp108 = Scalar(0.41999999999999998) * _tmp67;
  const Scalar _tmp109 = _tmp108 * _tmp12;
  const Scalar _tmp110 = _tmp16 * _tmp87;
  const Scalar _tmp111 = _tmp64 * _tmp89;
  const Scalar _tmp112 = _tmp107 + _tmp109 - _tmp110 - _tmp111;
  const Scalar _tmp113 = _tmp106 * _tmp64;
  const Scalar _tmp114 = _tmp108 * _tmp16;
  const Scalar _tmp115 = _tmp12 * _tmp87;
  const Scalar _tmp116 = _tmp73 * _tmp89;
  const Scalar _tmp117 = Scalar(0.043999999999999997) * _tmp94;
  const Scalar _tmp118 = Scalar(0.043999999999999997) * _tmp79;
  const Scalar _tmp119 = _tmp117 + _tmp118;
  const Scalar _tmp120 = _tmp113 + _tmp114 + _tmp115 + _tmp116 + _tmp119;
  const Scalar _tmp121 = _tmp112 + _tmp120;
  const Scalar _tmp122 = _tmp38 + Scalar(-4.8333311099999996);
  const Scalar _tmp123 = _tmp50 + Scalar(1.79662371);
  const Scalar _tmp124 = std::pow(_tmp122, Scalar(2)) + std::pow(_tmp123, Scalar(2));
  const Scalar _tmp125 = std::pow(_tmp124, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp126 = _tmp125 * fh1;
  const Scalar _tmp127 = _tmp122 * _tmp126;
  const Scalar _tmp128 = _tmp41 + _tmp48;
  const Scalar _tmp129 = _tmp128 + p_init0;
  const Scalar _tmp130 = _tmp129 + Scalar(-2.71799795);
  const Scalar _tmp131 = Scalar(1.0) / (_tmp130);
  const Scalar _tmp132 = _tmp27 + _tmp36;
  const Scalar _tmp133 = _tmp132 + p_init1;
  const Scalar _tmp134 = _tmp133 + Scalar(-4.7752063900000001);
  const Scalar _tmp135 = _tmp131 * _tmp134;
  const Scalar _tmp136 = -_tmp43 + _tmp47;
  const Scalar _tmp137 = _tmp136 + _tmp41;
  const Scalar _tmp138 = _tmp137 + p_init0;
  const Scalar _tmp139 = _tmp138 + Scalar(-2.5202214700000001);
  const Scalar _tmp140 = _tmp32 - _tmp35;
  const Scalar _tmp141 = _tmp140 + _tmp27;
  const Scalar _tmp142 = _tmp141 + p_init1;
  const Scalar _tmp143 = _tmp142 + Scalar(8.3888750099999996);
  const Scalar _tmp144 = std::pow(_tmp139, Scalar(2)) + std::pow(_tmp143, Scalar(2));
  const Scalar _tmp145 = std::pow(_tmp144, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp146 = _tmp139 * _tmp145;
  const Scalar _tmp147 = _tmp143 * _tmp145;
  const Scalar _tmp148 = _tmp135 * _tmp146 - _tmp147;
  const Scalar _tmp149 =
      Scalar(0.20999999999999999) * _tmp29 + Scalar(0.20999999999999999) * _tmp30;
  const Scalar _tmp150 = -_tmp149;
  const Scalar _tmp151 = -Scalar(0.010999999999999999) * _tmp33 -
                         Scalar(0.010999999999999999) * _tmp40 + Scalar(-0.010999999999999999);
  const Scalar _tmp152 =
      Scalar(0.20999999999999999) * _tmp44 - Scalar(0.20999999999999999) * _tmp45;
  const Scalar _tmp153 = _tmp151 + _tmp152;
  const Scalar _tmp154 = _tmp150 + _tmp153;
  const Scalar _tmp155 = _tmp149 + _tmp153;
  const Scalar _tmp156 = _tmp146 * _tmp155;
  const Scalar _tmp157 = _tmp151 - _tmp152;
  const Scalar _tmp158 = _tmp150 + _tmp157;
  const Scalar _tmp159 = _tmp140 + _tmp28;
  const Scalar _tmp160 = _tmp159 + p_init1;
  const Scalar _tmp161 = _tmp160 + Scalar(8.3196563700000006);
  const Scalar _tmp162 = _tmp136 + _tmp42;
  const Scalar _tmp163 = _tmp162 + p_init0;
  const Scalar _tmp164 = _tmp163 + Scalar(1.9874742000000001);
  const Scalar _tmp165 = std::pow(_tmp161, Scalar(2)) + std::pow(_tmp164, Scalar(2));
  const Scalar _tmp166 = std::pow(_tmp165, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp167 = _tmp164 * _tmp166;
  const Scalar _tmp168 = _tmp155 * _tmp167;
  const Scalar _tmp169 = -_tmp158 * _tmp167 + _tmp168;
  const Scalar _tmp170 = _tmp161 * _tmp166;
  const Scalar _tmp171 = _tmp135 * _tmp167 - _tmp170;
  const Scalar _tmp172 = Scalar(1.0) / (_tmp171);
  const Scalar _tmp173 = _tmp148 * _tmp172;
  const Scalar _tmp174 = -_tmp135 * _tmp168 + _tmp158 * _tmp170;
  const Scalar _tmp175 = -_tmp135 * _tmp156 + _tmp147 * _tmp154 - _tmp173 * _tmp174;
  const Scalar _tmp176 = Scalar(1.0) * _tmp128;
  const Scalar _tmp177 = -_tmp162 + _tmp176;
  const Scalar _tmp178 = Scalar(1.0) * _tmp132;
  const Scalar _tmp179 = -_tmp178;
  const Scalar _tmp180 = _tmp159 + _tmp179;
  const Scalar _tmp181 = Scalar(1.0) / (_tmp180);
  const Scalar _tmp182 = _tmp177 * _tmp181;
  const Scalar _tmp183 = -_tmp146 * _tmp154 + _tmp156 - _tmp169 * _tmp173 - _tmp175 * _tmp182;
  const Scalar _tmp184 = Scalar(1.0) / (_tmp183);
  const Scalar _tmp185 = Scalar(1.0) * _tmp172;
  const Scalar _tmp186 = _tmp174 * _tmp185;
  const Scalar _tmp187 = -_tmp169 * _tmp185 + _tmp182 * _tmp186;
  const Scalar _tmp188 = std::pow(_tmp130, Scalar(2));
  const Scalar _tmp189 = std::pow(_tmp134, Scalar(2)) + _tmp188;
  const Scalar _tmp190 = std::sqrt(_tmp189);
  const Scalar _tmp191 = Scalar(1.0) / (_tmp190);
  const Scalar _tmp192 = _tmp128 * _tmp191;
  const Scalar _tmp193 = _tmp130 * _tmp191;
  const Scalar _tmp194 = -_tmp132 * _tmp193 + _tmp134 * _tmp192;
  const Scalar _tmp195 = _tmp131 * _tmp190;
  const Scalar _tmp196 = _tmp194 * _tmp195;
  const Scalar _tmp197 = _tmp159 * _tmp167 - _tmp162 * _tmp170 + _tmp167 * _tmp196;
  const Scalar _tmp198 =
      -_tmp137 * _tmp147 + _tmp141 * _tmp146 + _tmp146 * _tmp196 - _tmp173 * _tmp197;
  const Scalar _tmp199 = _tmp184 * _tmp198;
  const Scalar _tmp200 = -_tmp185 * _tmp197 - _tmp187 * _tmp199;
  const Scalar _tmp201 = Scalar(1.0) / (_tmp198);
  const Scalar _tmp202 = _tmp183 * _tmp201;
  const Scalar _tmp203 = _tmp200 * _tmp202;
  const Scalar _tmp204 = _tmp187 + _tmp203;
  const Scalar _tmp205 = _tmp184 * _tmp204;
  const Scalar _tmp206 = -_tmp148 * _tmp205 + Scalar(1.0);
  const Scalar _tmp207 = _tmp167 * _tmp172;
  const Scalar _tmp208 = _tmp146 * _tmp205 + _tmp206 * _tmp207;
  const Scalar _tmp209 = _tmp195 * _tmp208;
  const Scalar _tmp210 = _tmp125 * _tmp49;
  const Scalar _tmp211 = _tmp123 * _tmp125;
  const Scalar _tmp212 = fh1 * (_tmp122 * _tmp210 - _tmp211 * _tmp37);
  const Scalar _tmp213 = Scalar(1.0) * _tmp201;
  const Scalar _tmp214 = _tmp167 * _tmp173;
  const Scalar _tmp215 = _tmp146 * _tmp213 - _tmp213 * _tmp214;
  const Scalar _tmp216 = _tmp195 * _tmp215;
  const Scalar _tmp217 = _tmp176 + _tmp178 * _tmp182;
  const Scalar _tmp218 = 0;
  const Scalar _tmp219 = _tmp184 * _tmp218;
  const Scalar _tmp220 = _tmp146 * _tmp219 - _tmp214 * _tmp219;
  const Scalar _tmp221 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp222 = _tmp195 * _tmp221;
  const Scalar _tmp223 = _tmp211 * fh1;
  const Scalar _tmp224 = _tmp135 * _tmp155;
  const Scalar _tmp225 = _tmp135 * _tmp172;
  const Scalar _tmp226 = _tmp174 * _tmp225 + _tmp224;
  const Scalar _tmp227 = -_tmp155 + _tmp169 * _tmp225 - _tmp182 * _tmp226;
  const Scalar _tmp228 = -_tmp196 + _tmp197 * _tmp225 - _tmp199 * _tmp227;
  const Scalar _tmp229 = _tmp202 * _tmp228;
  const Scalar _tmp230 = _tmp227 + _tmp229;
  const Scalar _tmp231 = _tmp184 * _tmp230;
  const Scalar _tmp232 = -_tmp135 - _tmp148 * _tmp231;
  const Scalar _tmp233 = _tmp146 * _tmp231 + _tmp207 * _tmp232 + Scalar(1.0);
  const Scalar _tmp234 = _tmp195 * _tmp233;
  const Scalar _tmp235 =
      -_tmp127 * _tmp209 - _tmp212 * _tmp216 - _tmp220 * _tmp222 - _tmp223 * _tmp234;
  const Scalar _tmp236 = std::pow(_tmp235, Scalar(-2));
  const Scalar _tmp237 = -_tmp101 - _tmp102 - _tmp103 - _tmp104;
  const Scalar _tmp238 = _tmp100 + _tmp237;
  const Scalar _tmp239 = _tmp145 * _tmp238;
  const Scalar _tmp240 = _tmp66 + _tmp69 - _tmp72 - _tmp75;
  const Scalar _tmp241 = _tmp240 + _tmp93;
  const Scalar _tmp242 =
      (2 * _tmp139 * _tmp238 + 2 * _tmp143 * _tmp241) / (_tmp144 * std::sqrt(_tmp144));
  const Scalar _tmp243 = (Scalar(1) / Scalar(2)) * _tmp242;
  const Scalar _tmp244 = _tmp139 * _tmp243;
  const Scalar _tmp245 = -_tmp80 + _tmp96;
  const Scalar _tmp246 = _tmp245 + _tmp92;
  const Scalar _tmp247 = _tmp240 + _tmp246;
  const Scalar _tmp248 = _tmp131 * _tmp247;
  const Scalar _tmp249 = _tmp143 * _tmp243;
  const Scalar _tmp250 = -_tmp88 - _tmp90;
  const Scalar _tmp251 = _tmp250 + _tmp86;
  const Scalar _tmp252 = _tmp251 + _tmp97;
  const Scalar _tmp253 = _tmp237 + _tmp252;
  const Scalar _tmp254 = _tmp253 / _tmp188;
  const Scalar _tmp255 = _tmp134 * _tmp254;
  const Scalar _tmp256 = _tmp145 * _tmp241;
  const Scalar _tmp257 = _tmp135 * _tmp239 - _tmp135 * _tmp244 + _tmp146 * _tmp248 -
                         _tmp146 * _tmp255 + _tmp249 - _tmp256;
  const Scalar _tmp258 = _tmp172 * _tmp257;
  const Scalar _tmp259 = _tmp78 + _tmp95;
  const Scalar _tmp260 = _tmp237 + _tmp259;
  const Scalar _tmp261 = _tmp260 + _tmp99;
  const Scalar _tmp262 = _tmp250 + _tmp98;
  const Scalar _tmp263 = _tmp262 + _tmp81;
  const Scalar _tmp264 = _tmp240 + _tmp263;
  const Scalar _tmp265 =
      (2 * _tmp161 * _tmp264 + 2 * _tmp164 * _tmp261) / (_tmp165 * std::sqrt(_tmp165));
  const Scalar _tmp266 = (Scalar(1) / Scalar(2)) * _tmp265;
  const Scalar _tmp267 = _tmp161 * _tmp266;
  const Scalar _tmp268 = _tmp164 * _tmp266;
  const Scalar _tmp269 = _tmp166 * _tmp264;
  const Scalar _tmp270 = _tmp166 * _tmp261;
  const Scalar _tmp271 = (-_tmp135 * _tmp268 + _tmp135 * _tmp270 + _tmp167 * _tmp248 -
                          _tmp167 * _tmp255 + _tmp267 - _tmp269) /
                         std::pow(_tmp171, Scalar(2));
  const Scalar _tmp272 = _tmp148 * _tmp271;
  const Scalar _tmp273 = _tmp130 * _tmp253 + _tmp134 * _tmp247;
  const Scalar _tmp274 = _tmp131 * _tmp191 * _tmp273;
  const Scalar _tmp275 = _tmp194 * _tmp274;
  const Scalar _tmp276 = _tmp190 * _tmp254;
  const Scalar _tmp277 = _tmp194 * _tmp276;
  const Scalar _tmp278 = _tmp191 * _tmp253;
  const Scalar _tmp279 = _tmp273 / (_tmp189 * std::sqrt(_tmp189));
  const Scalar _tmp280 =
      _tmp195 * (-_tmp128 * _tmp134 * _tmp279 + _tmp130 * _tmp132 * _tmp279 - _tmp132 * _tmp278 +
                 _tmp134 * _tmp278 + _tmp192 * _tmp247 - _tmp193 * _tmp247);
  const Scalar _tmp281 = -_tmp159 * _tmp268 + _tmp159 * _tmp270 + _tmp162 * _tmp267 -
                         _tmp162 * _tmp269 + _tmp164 * _tmp269 + _tmp167 * _tmp275 -
                         _tmp167 * _tmp277 + _tmp167 * _tmp280 - _tmp170 * _tmp261 -
                         _tmp196 * _tmp268 + _tmp196 * _tmp270;
  const Scalar _tmp282 = _tmp137 * _tmp249 - _tmp137 * _tmp256 + _tmp139 * _tmp256 +
                         _tmp141 * _tmp239 - _tmp141 * _tmp244 + _tmp146 * _tmp275 -
                         _tmp146 * _tmp277 + _tmp146 * _tmp280 - _tmp147 * _tmp238 -
                         _tmp173 * _tmp281 + _tmp196 * _tmp239 - _tmp196 * _tmp244 -
                         _tmp197 * _tmp258 + _tmp197 * _tmp272;
  const Scalar _tmp283 = _tmp282 / std::pow(_tmp198, Scalar(2));
  const Scalar _tmp284 = _tmp183 * _tmp283;
  const Scalar _tmp285 = _tmp228 * _tmp284;
  const Scalar _tmp286 = _tmp155 * _tmp239;
  const Scalar _tmp287 = -_tmp113 - _tmp114 - _tmp115 - _tmp116;
  const Scalar _tmp288 = _tmp119 + _tmp287;
  const Scalar _tmp289 = -_tmp107 - _tmp109 + _tmp110 + _tmp111;
  const Scalar _tmp290 = _tmp288 + _tmp289;
  const Scalar _tmp291 = _tmp120 + _tmp289;
  const Scalar _tmp292 = _tmp135 * _tmp291;
  const Scalar _tmp293 = _tmp167 * _tmp291;
  const Scalar _tmp294 = _tmp112 + _tmp288;
  const Scalar _tmp295 = _tmp155 * _tmp270;
  const Scalar _tmp296 = -_tmp135 * _tmp293 - _tmp135 * _tmp295 - _tmp158 * _tmp267 +
                         _tmp158 * _tmp269 - _tmp168 * _tmp248 + _tmp168 * _tmp255 +
                         _tmp170 * _tmp294 + _tmp224 * _tmp268;
  const Scalar _tmp297 = -_tmp135 * _tmp286 - _tmp146 * _tmp292 + _tmp147 * _tmp290 -
                         _tmp154 * _tmp249 + _tmp154 * _tmp256 - _tmp156 * _tmp248 +
                         _tmp156 * _tmp255 - _tmp173 * _tmp296 - _tmp174 * _tmp258 +
                         _tmp174 * _tmp272 + _tmp224 * _tmp244;
  const Scalar _tmp298 = Scalar(0.83999999999999997) * _tmp82;
  const Scalar _tmp299 = Scalar(0.83999999999999997) * _tmp84;
  const Scalar _tmp300 = Scalar(1.6799999999999999) * _tmp21 * _tmp70;
  const Scalar _tmp301 = _tmp300 + Scalar(1.6799999999999999) * _tmp79;
  const Scalar _tmp302 = -Scalar(0.83999999999999997) * _tmp21 * _tmp67 - _tmp24 * _tmp77;
  const Scalar _tmp303 = (-_tmp298 - _tmp299 + _tmp301 + _tmp302) / std::pow(_tmp180, Scalar(2));
  const Scalar _tmp304 = _tmp177 * _tmp303;
  const Scalar _tmp305 = -_tmp155 * _tmp268 + _tmp158 * _tmp268 - _tmp158 * _tmp270 -
                         _tmp167 * _tmp294 + _tmp293 + _tmp295;
  const Scalar _tmp306 = _tmp298 + _tmp299 + _tmp302;
  const Scalar _tmp307 = _tmp181 * (-_tmp300 + _tmp306 - Scalar(1.6799999999999999) * _tmp94);
  const Scalar _tmp308 = -_tmp146 * _tmp290 + _tmp146 * _tmp291 - _tmp154 * _tmp239 +
                         _tmp154 * _tmp244 - _tmp155 * _tmp244 - _tmp169 * _tmp258 +
                         _tmp169 * _tmp272 - _tmp173 * _tmp305 + _tmp175 * _tmp304 -
                         _tmp175 * _tmp307 - _tmp182 * _tmp297 + _tmp286;
  const Scalar _tmp309 = _tmp201 * _tmp308;
  const Scalar _tmp310 = _tmp228 * _tmp309;
  const Scalar _tmp311 = _tmp172 * _tmp255;
  const Scalar _tmp312 = _tmp135 * _tmp271;
  const Scalar _tmp313 = _tmp172 * _tmp248;
  const Scalar _tmp314 = _tmp155 * _tmp248 - _tmp155 * _tmp255 - _tmp174 * _tmp311 -
                         _tmp174 * _tmp312 + _tmp174 * _tmp313 + _tmp225 * _tmp296 + _tmp292;
  const Scalar _tmp315 = _tmp112 - _tmp117 - _tmp118 - _tmp169 * _tmp311 - _tmp169 * _tmp312 +
                         _tmp169 * _tmp313 - _tmp182 * _tmp314 + _tmp225 * _tmp305 +
                         _tmp226 * _tmp304 - _tmp226 * _tmp307 + _tmp287;
  const Scalar _tmp316 = _tmp184 * _tmp282;
  const Scalar _tmp317 = _tmp308 / std::pow(_tmp183, Scalar(2));
  const Scalar _tmp318 = _tmp198 * _tmp317;
  const Scalar _tmp319 = _tmp202 * (-_tmp197 * _tmp311 - _tmp197 * _tmp312 + _tmp197 * _tmp313 -
                                    _tmp199 * _tmp315 + _tmp225 * _tmp281 - _tmp227 * _tmp316 +
                                    _tmp227 * _tmp318 - _tmp275 + _tmp277 - _tmp280);
  const Scalar _tmp320 = _tmp184 * (-_tmp285 + _tmp310 + _tmp315 + _tmp319);
  const Scalar _tmp321 = _tmp146 * _tmp317;
  const Scalar _tmp322 = _tmp172 * _tmp270;
  const Scalar _tmp323 = _tmp167 * _tmp271;
  const Scalar _tmp324 = _tmp148 * _tmp317;
  const Scalar _tmp325 =
      -_tmp148 * _tmp320 + _tmp230 * _tmp324 - _tmp231 * _tmp257 - _tmp248 + _tmp255;
  const Scalar _tmp326 = _tmp172 * _tmp232;
  const Scalar _tmp327 = _tmp251 + _tmp260;
  const Scalar _tmp328 = _tmp245 + _tmp262;
  const Scalar _tmp329 = _tmp240 + _tmp328;
  const Scalar _tmp330 =
      (2 * _tmp122 * _tmp329 + 2 * _tmp123 * _tmp327) / (_tmp124 * std::sqrt(_tmp124));
  const Scalar _tmp331 = (Scalar(1) / Scalar(2)) * _tmp330;
  const Scalar _tmp332 = _tmp123 * _tmp331;
  const Scalar _tmp333 = _tmp332 * fh1;
  const Scalar _tmp334 = _tmp223 * _tmp233;
  const Scalar _tmp335 = _tmp127 * _tmp208;
  const Scalar _tmp336 = _tmp212 * _tmp215;
  const Scalar _tmp337 = _tmp220 * _tmp221;
  const Scalar _tmp338 = Scalar(1.0) * _tmp283;
  const Scalar _tmp339 = _tmp173 * _tmp270;
  const Scalar _tmp340 = Scalar(0.5) * _tmp201;
  const Scalar _tmp341 = _tmp185 * _tmp201 * _tmp257;
  const Scalar _tmp342 = _tmp126 * _tmp329;
  const Scalar _tmp343 = _tmp122 * _tmp331;
  const Scalar _tmp344 = _tmp343 * fh1;
  const Scalar _tmp345 = _tmp125 * _tmp327;
  const Scalar _tmp346 = _tmp345 * fh1;
  const Scalar _tmp347 = _tmp218 * _tmp317;
  const Scalar _tmp348 = _tmp167 * _tmp219;
  const Scalar _tmp349 = _tmp200 * _tmp284;
  const Scalar _tmp350 = _tmp200 * _tmp309;
  const Scalar _tmp351 = Scalar(1.0) * _tmp271;
  const Scalar _tmp352 = _tmp185 * _tmp296;
  const Scalar _tmp353 = _tmp174 * _tmp351;
  const Scalar _tmp354 = _tmp169 * _tmp351 + _tmp182 * _tmp352 - _tmp182 * _tmp353 -
                         _tmp185 * _tmp305 - _tmp186 * _tmp304 + _tmp186 * _tmp307;
  const Scalar _tmp355 = _tmp202 * (-_tmp185 * _tmp281 - _tmp187 * _tmp316 + _tmp187 * _tmp318 +
                                    _tmp197 * _tmp351 - _tmp199 * _tmp354);
  const Scalar _tmp356 = _tmp184 * (-_tmp349 + _tmp350 + _tmp354 + _tmp355);
  const Scalar _tmp357 = _tmp172 * _tmp206;
  const Scalar _tmp358 = -_tmp148 * _tmp356 + _tmp204 * _tmp324 - _tmp205 * _tmp257;
  const Scalar _tmp359 = fh1 * (_tmp122 * _tmp345 + _tmp210 * _tmp329 - _tmp211 * _tmp329 +
                                _tmp332 * _tmp37 - _tmp343 * _tmp49 - _tmp345 * _tmp37);
  const Scalar _tmp360 =
      -_tmp127 * _tmp195 *
          (_tmp146 * _tmp356 - _tmp204 * _tmp321 + _tmp205 * _tmp239 - _tmp205 * _tmp244 +
           _tmp206 * _tmp322 - _tmp206 * _tmp323 + _tmp207 * _tmp358 - _tmp268 * _tmp357) -
      _tmp195 * _tmp212 *
          (-_tmp139 * _tmp242 * _tmp340 - _tmp146 * _tmp338 +
           _tmp164 * _tmp173 * _tmp265 * _tmp340 + _tmp167 * _tmp213 * _tmp272 - _tmp167 * _tmp341 +
           _tmp213 * _tmp239 - _tmp213 * _tmp339 + _tmp214 * _tmp338) -
      _tmp195 * _tmp223 *
          (_tmp146 * _tmp320 + _tmp207 * _tmp325 - _tmp230 * _tmp321 + _tmp231 * _tmp239 -
           _tmp231 * _tmp244 + _tmp232 * _tmp322 - _tmp232 * _tmp323 - _tmp268 * _tmp326) -
      _tmp209 * _tmp342 + _tmp209 * _tmp344 - _tmp216 * _tmp359 -
      _tmp222 * (-_tmp146 * _tmp347 + _tmp173 * _tmp219 * _tmp268 + _tmp214 * _tmp347 +
                 _tmp219 * _tmp239 - _tmp219 * _tmp244 - _tmp219 * _tmp339 - _tmp258 * _tmp348 +
                 _tmp272 * _tmp348) +
      _tmp234 * _tmp333 - _tmp234 * _tmp346 - _tmp274 * _tmp334 - _tmp274 * _tmp335 -
      _tmp274 * _tmp336 - _tmp274 * _tmp337 + _tmp276 * _tmp334 + _tmp276 * _tmp335 +
      _tmp276 * _tmp336 + _tmp276 * _tmp337;
  const Scalar _tmp361 = _tmp236 * _tmp360;
  const Scalar _tmp362 = _tmp141 + _tmp179;
  const Scalar _tmp363 = _tmp182 * _tmp362;
  const Scalar _tmp364 = -_tmp137 + _tmp176 - _tmp363;
  const Scalar _tmp365 = Scalar(1.0) / (_tmp364);
  const Scalar _tmp366 = Scalar(1.0) * _tmp365;
  const Scalar _tmp367 = _tmp217 * _tmp365;
  const Scalar _tmp368 = -_tmp175 * _tmp219 + _tmp179 - _tmp362 * _tmp367;
  const Scalar _tmp369 = Scalar(1.0) * _tmp181;
  const Scalar _tmp370 = Scalar(1.0) * _tmp221;
  const Scalar _tmp371 = _tmp362 * _tmp365;
  const Scalar _tmp372 = -_tmp175 * _tmp205 - _tmp186 + _tmp203 * _tmp371;
  const Scalar _tmp373 = _tmp203 * _tmp366 - _tmp369 * _tmp372;
  const Scalar _tmp374 = Scalar(1.0) * _tmp127;
  const Scalar _tmp375 = _tmp149 + _tmp157;
  const Scalar _tmp376 = _tmp375 * fh1;
  const Scalar _tmp377 = _tmp125 * _tmp376;
  const Scalar _tmp378 = -_tmp122 * _tmp377 - Scalar(3.29616) * _tmp31 - _tmp37 * fv1;
  const Scalar _tmp379 = _tmp363 * _tmp366 + Scalar(1.0);
  const Scalar _tmp380 = _tmp182 * _tmp366;
  const Scalar _tmp381 = -Scalar(1.0) * _tmp369 * _tmp379 + Scalar(1.0) * _tmp380;
  const Scalar _tmp382 = _tmp202 * _tmp366;
  const Scalar _tmp383 = -_tmp175 * _tmp213 + _tmp362 * _tmp382;
  const Scalar _tmp384 = -Scalar(1.0) * _tmp369 * _tmp383 + Scalar(1.0) * _tmp382;
  const Scalar _tmp385 = -_tmp175 * _tmp231 + _tmp226 + _tmp229 * _tmp371;
  const Scalar _tmp386 = _tmp229 * _tmp366 - _tmp369 * _tmp385;
  const Scalar _tmp387 = Scalar(1.0) * _tmp223;
  const Scalar _tmp388 = _tmp181 * _tmp366;
  const Scalar _tmp389 = _tmp362 * _tmp388 - _tmp366;
  const Scalar _tmp390 = _tmp223 * _tmp375 + Scalar(3.29616) * _tmp46 + _tmp49 * fv1;
  const Scalar _tmp391 = Scalar(1.0) * _tmp390;
  const Scalar _tmp392 =
      _tmp212 * _tmp384 + _tmp370 * (-_tmp217 * _tmp366 - _tmp368 * _tmp369 + Scalar(1.0)) +
      _tmp373 * _tmp374 + _tmp378 * _tmp381 + _tmp386 * _tmp387 + _tmp389 * _tmp391;
  const Scalar _tmp393 = Scalar(1.0) / (_tmp235);
  const Scalar _tmp394 = std::asinh(_tmp392 * _tmp393);
  const Scalar _tmp395 = Scalar(1.0) * _tmp394;
  const Scalar _tmp396 = _tmp309 * _tmp366;
  const Scalar _tmp397 = _tmp182 * _tmp301;
  const Scalar _tmp398 = _tmp307 * _tmp362;
  const Scalar _tmp399 = _tmp304 * _tmp362;
  const Scalar _tmp400 = (_tmp306 - _tmp397 - _tmp398 + _tmp399) / std::pow(_tmp364, Scalar(2));
  const Scalar _tmp401 = Scalar(1.0) * _tmp400;
  const Scalar _tmp402 = _tmp202 * _tmp401;
  const Scalar _tmp403 = _tmp284 * _tmp366;
  const Scalar _tmp404 = _tmp175 * _tmp338 - _tmp213 * _tmp297 + _tmp301 * _tmp382 +
                         _tmp362 * _tmp396 - _tmp362 * _tmp402 - _tmp362 * _tmp403;
  const Scalar _tmp405 = Scalar(1.0) * _tmp303;
  const Scalar _tmp406 = Scalar(1.0) * _tmp212;
  const Scalar _tmp407 = _tmp217 * _tmp400;
  const Scalar _tmp408 = _tmp175 * _tmp317;
  const Scalar _tmp409 = _tmp184 * _tmp297;
  const Scalar _tmp410 =
      -_tmp178 * _tmp304 + _tmp178 * _tmp307 + Scalar(1.0) * _tmp182 * _tmp247 + _tmp253;
  const Scalar _tmp411 = _tmp365 * _tmp410;
  const Scalar _tmp412 = _tmp263 + _tmp76;
  const Scalar _tmp413 = _tmp218 * _tmp408 - _tmp218 * _tmp409 - _tmp301 * _tmp367 +
                         _tmp362 * _tmp407 - _tmp362 * _tmp411 + _tmp412;
  const Scalar _tmp414 = Scalar(6.59232) * _tmp24;
  const Scalar _tmp415 = Scalar(6.59232) * _tmp16;
  const Scalar _tmp416 = Scalar(6.59232) * _tmp12;
  const Scalar _tmp417 = Scalar(6.59232) * _tmp21;
  const Scalar _tmp418 = -_tmp121 * _tmp127 - _tmp329 * _tmp377 - _tmp329 * fv1 +
                         _tmp343 * _tmp376 + _tmp414 * _tmp64 + _tmp415 * _tmp67 -
                         _tmp416 * _tmp70 - _tmp417 * _tmp73;
  const Scalar _tmp419 = _tmp301 * _tmp365;
  const Scalar _tmp420 = _tmp362 * _tmp400;
  const Scalar _tmp421 = -_tmp175 * _tmp356 + _tmp203 * _tmp419 - _tmp203 * _tmp420 +
                         _tmp204 * _tmp408 - _tmp204 * _tmp409 - _tmp349 * _tmp371 +
                         _tmp350 * _tmp371 - _tmp352 + _tmp353 + _tmp355 * _tmp371;
  const Scalar _tmp422 = Scalar(0.5) * _tmp330 * fh1;
  const Scalar _tmp423 = _tmp121 * _tmp223 + _tmp327 * fv1 - _tmp332 * _tmp376 + _tmp345 * _tmp376 +
                         _tmp414 * _tmp73 + _tmp415 * _tmp70 + _tmp416 * _tmp67 + _tmp417 * _tmp64;
  const Scalar _tmp424 = _tmp303 * _tmp362;
  const Scalar _tmp425 = -_tmp175 * _tmp320 + _tmp229 * _tmp419 - _tmp229 * _tmp420 +
                         _tmp230 * _tmp408 - _tmp230 * _tmp409 - _tmp285 * _tmp371 +
                         _tmp310 * _tmp371 + _tmp314 + _tmp319 * _tmp371;
  const Scalar _tmp426 = _tmp307 * _tmp366;
  const Scalar _tmp427 = _tmp182 * _tmp401;
  const Scalar _tmp428 =
      -_tmp363 * _tmp401 + _tmp366 * _tmp397 + _tmp366 * _tmp398 - _tmp366 * _tmp399;
  const Scalar _tmp429 =
      (-_tmp361 * _tmp392 +
       _tmp393 *
           (-_tmp122 * _tmp373 * _tmp422 - _tmp123 * _tmp386 * _tmp422 +
            Scalar(1.0) * _tmp342 * _tmp373 + Scalar(1.0) * _tmp346 * _tmp386 + _tmp359 * _tmp384 +
            _tmp370 *
                (_tmp217 * _tmp401 - _tmp366 * _tmp410 + _tmp368 * _tmp405 - _tmp369 * _tmp413) +
            _tmp374 * (-_tmp203 * _tmp401 - _tmp349 * _tmp366 + _tmp350 * _tmp366 +
                       _tmp355 * _tmp366 - _tmp369 * _tmp421 + _tmp372 * _tmp405) +
            Scalar(1.0) * _tmp378 *
                (-_tmp304 * _tmp366 - _tmp369 * _tmp428 + _tmp379 * _tmp405 + _tmp426 - _tmp427) +
            _tmp381 * _tmp418 +
            _tmp387 * (-_tmp229 * _tmp401 - _tmp285 * _tmp366 + _tmp310 * _tmp366 +
                       _tmp319 * _tmp366 - _tmp369 * _tmp425 + _tmp385 * _tmp405) +
            Scalar(1.0) * _tmp389 * _tmp423 +
            _tmp391 * (_tmp301 * _tmp388 - _tmp366 * _tmp424 - _tmp369 * _tmp420 + _tmp401) +
            _tmp406 * (-_tmp369 * _tmp404 + _tmp383 * _tmp405 + _tmp396 - _tmp402 - _tmp403))) /
      std::sqrt(Scalar(_tmp236 * std::pow(_tmp392, Scalar(2)) + 1));
  const Scalar _tmp430 = Scalar(9.6622558468725703) * _tmp235;
  const Scalar _tmp431 = Scalar(4.7752063900000001) - _tmp133;
  const Scalar _tmp432 = Scalar(2.71799795) - _tmp129;
  const Scalar _tmp433 =
      std::sqrt(Scalar(std::pow(_tmp431, Scalar(2)) + std::pow(_tmp432, Scalar(2))));
  const Scalar _tmp434 = -_tmp394 * _tmp430 - _tmp433;
  const Scalar _tmp435 = Scalar(0.1034955) * _tmp393;
  const Scalar _tmp436 = _tmp434 * _tmp435;
  const Scalar _tmp437 = _tmp105 + _tmp259;
  const Scalar _tmp438 = Scalar(9.6622558468725703) * _tmp360;
  const Scalar _tmp439 = _tmp219 * _tmp221;
  const Scalar _tmp440 = _tmp127 * _tmp172;
  const Scalar _tmp441 = _tmp212 * _tmp213;
  const Scalar _tmp442 = _tmp172 * _tmp223;
  const Scalar _tmp443 =
      -_tmp173 * _tmp439 - _tmp173 * _tmp441 + _tmp206 * _tmp440 + _tmp232 * _tmp442;
  const Scalar _tmp444 = Scalar(1.0) / (_tmp443);
  const Scalar _tmp445 = _tmp181 * _tmp383;
  const Scalar _tmp446 = _tmp181 * _tmp379;
  const Scalar _tmp447 = _tmp181 * _tmp385;
  const Scalar _tmp448 = _tmp366 * _tmp390;
  const Scalar _tmp449 = _tmp181 * _tmp362;
  const Scalar _tmp450 = _tmp127 * _tmp181;
  const Scalar _tmp451 = _tmp181 * _tmp221;
  const Scalar _tmp452 = _tmp212 * _tmp445 + _tmp223 * _tmp447 + _tmp368 * _tmp451 +
                         _tmp372 * _tmp450 + _tmp378 * _tmp446 - _tmp448 * _tmp449;
  const Scalar _tmp453 = std::asinh(_tmp444 * _tmp452);
  const Scalar _tmp454 = Scalar(1.0) * _tmp453;
  const Scalar _tmp455 = Scalar(9.6622558468725703) * _tmp443;
  const Scalar _tmp456 = -_tmp160 + Scalar(-8.3196563700000006);
  const Scalar _tmp457 = -_tmp163 + Scalar(-1.9874742000000001);
  const Scalar _tmp458 =
      std::sqrt(Scalar(std::pow(_tmp456, Scalar(2)) + std::pow(_tmp457, Scalar(2))));
  const Scalar _tmp459 = -_tmp453 * _tmp455 - _tmp458;
  const Scalar _tmp460 = Scalar(0.1034955) * _tmp444;
  const Scalar _tmp461 = _tmp459 * _tmp460;
  const Scalar _tmp462 = _tmp283 * _tmp406;
  const Scalar _tmp463 = _tmp221 * _tmp347;
  const Scalar _tmp464 = _tmp213 * _tmp359;
  const Scalar _tmp465 = -_tmp127 * _tmp206 * _tmp271 + _tmp173 * _tmp462 + _tmp173 * _tmp463 -
                         _tmp173 * _tmp464 - _tmp212 * _tmp341 - _tmp223 * _tmp232 * _tmp271 -
                         _tmp258 * _tmp439 + _tmp272 * _tmp439 + _tmp272 * _tmp441 +
                         _tmp325 * _tmp442 - _tmp326 * _tmp333 + _tmp326 * _tmp346 +
                         _tmp342 * _tmp357 - _tmp344 * _tmp357 + _tmp358 * _tmp440;
  const Scalar _tmp466 = Scalar(9.6622558468725703) * _tmp465;
  const Scalar _tmp467 = std::pow(_tmp443, Scalar(-2));
  const Scalar _tmp468 = _tmp465 * _tmp467;
  const Scalar _tmp469 = _tmp366 * _tmp423;
  const Scalar _tmp470 = _tmp391 * _tmp400;
  const Scalar _tmp471 = _tmp181 * _tmp372;
  const Scalar _tmp472 = _tmp303 * _tmp378;
  const Scalar _tmp473 =
      (_tmp444 * (-_tmp127 * _tmp303 * _tmp372 + _tmp181 * _tmp212 * _tmp404 +
                  _tmp181 * _tmp223 * _tmp425 - _tmp181 * _tmp301 * _tmp448 +
                  _tmp181 * _tmp378 * _tmp428 - _tmp212 * _tmp303 * _tmp383 -
                  _tmp221 * _tmp303 * _tmp368 - _tmp223 * _tmp303 * _tmp385 - _tmp333 * _tmp447 +
                  _tmp342 * _tmp471 - _tmp344 * _tmp471 + _tmp346 * _tmp447 + _tmp359 * _tmp445 -
                  _tmp379 * _tmp472 + _tmp413 * _tmp451 + _tmp418 * _tmp446 + _tmp421 * _tmp450 +
                  _tmp424 * _tmp448 - _tmp449 * _tmp469 + _tmp449 * _tmp470) -
       _tmp452 * _tmp468) /
      std::sqrt(Scalar(std::pow(_tmp452, Scalar(2)) * _tmp467 + 1));
  const Scalar _tmp474 = _tmp127 * _tmp205 + _tmp223 * _tmp231 + _tmp439 + _tmp441;
  const Scalar _tmp475 = Scalar(1.0) / (_tmp474);
  const Scalar _tmp476 = _tmp223 * _tmp365;
  const Scalar _tmp477 = _tmp127 * _tmp365;
  const Scalar _tmp478 = -_tmp203 * _tmp477 - _tmp212 * _tmp382 + _tmp221 * _tmp367 -
                         _tmp229 * _tmp476 - _tmp378 * _tmp380 + _tmp448;
  const Scalar _tmp479 = std::asinh(_tmp475 * _tmp478);
  const Scalar _tmp480 = Scalar(1.0) * _tmp479;
  const Scalar _tmp481 = -_tmp142 + Scalar(-8.3888750099999996);
  const Scalar _tmp482 = Scalar(2.5202214700000001) - _tmp138;
  const Scalar _tmp483 =
      std::sqrt(Scalar(std::pow(_tmp481, Scalar(2)) + std::pow(_tmp482, Scalar(2))));
  const Scalar _tmp484 = Scalar(9.6622558468725703) * _tmp474;
  const Scalar _tmp485 = -_tmp479 * _tmp484 - _tmp483;
  const Scalar _tmp486 = Scalar(0.1034955) * _tmp475;
  const Scalar _tmp487 = _tmp485 * _tmp486;
  const Scalar _tmp488 = -_tmp127 * _tmp204 * _tmp317 + _tmp127 * _tmp356 + _tmp205 * _tmp342 -
                         _tmp205 * _tmp344 - _tmp223 * _tmp230 * _tmp317 + _tmp223 * _tmp320 -
                         _tmp231 * _tmp333 + _tmp231 * _tmp346 - _tmp462 - _tmp463 + _tmp464;
  const Scalar _tmp489 = Scalar(9.6622558468725703) * _tmp488;
  const Scalar _tmp490 = std::pow(_tmp474, Scalar(-2));
  const Scalar _tmp491 = _tmp488 * _tmp490;
  const Scalar _tmp492 = _tmp229 * _tmp365;
  const Scalar _tmp493 = _tmp203 * _tmp365;
  const Scalar _tmp494 =
      (_tmp475 * (_tmp127 * _tmp203 * _tmp400 + _tmp177 * _tmp366 * _tmp472 - _tmp212 * _tmp396 +
                  _tmp212 * _tmp402 + _tmp212 * _tmp403 - _tmp221 * _tmp407 + _tmp221 * _tmp411 +
                  _tmp223 * _tmp229 * _tmp400 + _tmp285 * _tmp476 - _tmp310 * _tmp476 -
                  _tmp319 * _tmp476 + _tmp333 * _tmp492 - _tmp342 * _tmp493 + _tmp344 * _tmp493 -
                  _tmp346 * _tmp492 + _tmp349 * _tmp477 - _tmp350 * _tmp477 - _tmp355 * _tmp477 -
                  _tmp359 * _tmp382 - _tmp378 * _tmp426 + _tmp378 * _tmp427 - _tmp380 * _tmp418 +
                  _tmp469 - _tmp470) -
       _tmp478 * _tmp491) /
      std::sqrt(Scalar(std::pow(_tmp478, Scalar(2)) * _tmp490 + 1));

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) =
      _tmp121 -
      Scalar(0.5) * (2 * _tmp39 * (_tmp76 + _tmp93) + 2 * _tmp51 * (_tmp100 + _tmp105)) *
          std::sinh(Scalar(0.1034955) * _tmp53 *
                    (-_tmp52 - Scalar(9.6622558468725703) * fh1 * std::asinh(_tmp53 * fv1))) /
          _tmp52;
  _res(1, 0) =
      _tmp291 -
      _tmp430 *
          (-Scalar(0.86565325453551001) * _tmp361 + Scalar(1.0) * _tmp429 * std::sinh(_tmp395) -
           (-Scalar(0.1034955) * _tmp361 * _tmp434 +
            _tmp435 * (-_tmp394 * _tmp438 - _tmp429 * _tmp430 -
                       Scalar(1) / Scalar(2) *
                           (2 * _tmp412 * _tmp431 + 2 * _tmp432 * (_tmp437 + _tmp99)) / _tmp433)) *
               std::sinh(_tmp436)) -
      _tmp438 * (Scalar(0.86565325453551001) * _tmp393 + std::cosh(_tmp395) - std::cosh(_tmp436));
  _res(2, 0) =
      _tmp294 -
      _tmp455 *
          (-Scalar(0.87679799772039002) * _tmp468 + Scalar(1.0) * _tmp473 * std::sinh(_tmp454) -
           (-Scalar(0.1034955) * _tmp459 * _tmp468 +
            _tmp460 * (-_tmp453 * _tmp466 - _tmp455 * _tmp473 -
                       Scalar(1) / Scalar(2) *
                           (2 * _tmp456 * (_tmp246 + _tmp76) + 2 * _tmp457 * (_tmp105 + _tmp252)) /
                           _tmp458)) *
               std::sinh(_tmp461)) -
      _tmp466 * (Scalar(0.87679799772039002) * _tmp444 + std::cosh(_tmp454) - std::cosh(_tmp461));
  _res(3, 0) =
      _tmp290 -
      _tmp484 *
          (-Scalar(0.87653584775870996) * _tmp491 + Scalar(1.0) * _tmp494 * std::sinh(_tmp480) -
           (-Scalar(0.1034955) * _tmp485 * _tmp491 +
            _tmp486 * (-_tmp479 * _tmp489 - _tmp484 * _tmp494 -
                       Scalar(1) / Scalar(2) *
                           (2 * _tmp481 * (_tmp328 + _tmp76) + 2 * _tmp482 * (_tmp251 + _tmp437)) /
                           _tmp483)) *
               std::sinh(_tmp487)) -
      _tmp489 * (Scalar(0.87653584775870996) * _tmp475 + std::cosh(_tmp480) - std::cosh(_tmp487));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym