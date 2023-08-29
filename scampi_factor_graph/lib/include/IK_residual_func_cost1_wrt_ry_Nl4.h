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
 * Symbolic function: IK_residual_func_cost1_wrt_ry_Nl4
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost1WrtRyNl4(
    const Scalar fh1, const Scalar fv1, const Scalar rx, const Scalar ry, const Scalar rz,
    const Scalar p_init0, const Scalar p_init1, const Scalar p_init2, const Scalar rot_init_x,
    const Scalar rot_init_y, const Scalar rot_init_z, const Scalar rot_init_w,
    const Scalar epsilon) {
  // Total ops: 1626

  // Unused inputs
  (void)p_init2;
  (void)epsilon;

  // Input arrays

  // Intermediate terms (505)
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
  const Scalar _tmp13 = _tmp4 * rot_init_w;
  const Scalar _tmp14 = (Scalar(1) / Scalar(2)) / _tmp1;
  const Scalar _tmp15 = _tmp0 * _tmp14;
  const Scalar _tmp16 = _tmp14 * ry;
  const Scalar _tmp17 = _tmp16 * rz;
  const Scalar _tmp18 = _tmp4 * rot_init_z;
  const Scalar _tmp19 = _tmp16 * rx;
  const Scalar _tmp20 = _tmp11 * ry;
  const Scalar _tmp21 = _tmp6 / (_tmp1 * std::sqrt(_tmp1));
  const Scalar _tmp22 = _tmp0 * _tmp21;
  const Scalar _tmp23 = _tmp21 * ry;
  const Scalar _tmp24 = _tmp23 * rx;
  const Scalar _tmp25 = _tmp23 * rz;
  const Scalar _tmp26 = _tmp10 + _tmp13 * _tmp15 - _tmp17 * _tmp5 + _tmp18 * _tmp19 -
                        Scalar(1) / Scalar(2) * _tmp20 - _tmp22 * rot_init_w - _tmp24 * rot_init_z +
                        _tmp25 * rot_init_x;
  const Scalar _tmp27 = Scalar(0.41999999999999998) * _tmp26;
  const Scalar _tmp28 = _tmp12 * _tmp27;
  const Scalar _tmp29 = _tmp4 * rot_init_y;
  const Scalar _tmp30 = _tmp10 * ry;
  const Scalar _tmp31 = _tmp7 * rot_init_x;
  const Scalar _tmp32 = _tmp29 + _tmp30 - _tmp31 * rz + _tmp8 * rx;
  const Scalar _tmp33 = _tmp31 * ry;
  const Scalar _tmp34 = _tmp13 * _tmp19 - _tmp15 * _tmp18 + _tmp17 * _tmp29 + _tmp22 * rot_init_z -
                        _tmp24 * rot_init_w - _tmp25 * rot_init_y - Scalar(1) / Scalar(2) * _tmp33 -
                        _tmp8;
  const Scalar _tmp35 = Scalar(0.41999999999999998) * _tmp34;
  const Scalar _tmp36 = _tmp32 * _tmp35;
  const Scalar _tmp37 = _tmp28 + _tmp36;
  const Scalar _tmp38 = _tmp13 - _tmp20 - _tmp31 * rx - _tmp8 * rz;
  const Scalar _tmp39 = _tmp13 * _tmp17 + _tmp15 * _tmp5 - _tmp19 * _tmp29 - _tmp22 * rot_init_x +
                        _tmp24 * rot_init_y - _tmp25 * rot_init_w + _tmp31 -
                        Scalar(1) / Scalar(2) * _tmp9;
  const Scalar _tmp40 = Scalar(0.41999999999999998) * _tmp39;
  const Scalar _tmp41 = _tmp38 * _tmp40;
  const Scalar _tmp42 = _tmp10 * rz - _tmp11 * rx + _tmp18 + _tmp33;
  const Scalar _tmp43 = -_tmp11 - _tmp15 * _tmp29 - _tmp17 * _tmp18 - _tmp19 * _tmp5 +
                        _tmp22 * rot_init_y + _tmp24 * rot_init_x + _tmp25 * rot_init_z -
                        Scalar(1) / Scalar(2) * _tmp30;
  const Scalar _tmp44 = Scalar(0.41999999999999998) * _tmp43;
  const Scalar _tmp45 = _tmp42 * _tmp44;
  const Scalar _tmp46 = _tmp41 + _tmp45;
  const Scalar _tmp47 = _tmp37 + _tmp46;
  const Scalar _tmp48 = Scalar(0.83999999999999997) * _tmp42;
  const Scalar _tmp49 = _tmp39 * _tmp48;
  const Scalar _tmp50 = -_tmp49;
  const Scalar _tmp51 = Scalar(0.83999999999999997) * _tmp34;
  const Scalar _tmp52 = _tmp12 * _tmp51;
  const Scalar _tmp53 = _tmp50 - _tmp52;
  const Scalar _tmp54 = Scalar(0.021999999999999999) * _tmp38;
  const Scalar _tmp55 = _tmp34 * _tmp54;
  const Scalar _tmp56 = Scalar(0.021999999999999999) * _tmp43;
  const Scalar _tmp57 = _tmp12 * _tmp56;
  const Scalar _tmp58 = Scalar(0.021999999999999999) * _tmp39;
  const Scalar _tmp59 = _tmp32 * _tmp58;
  const Scalar _tmp60 = Scalar(0.021999999999999999) * _tmp26 * _tmp42;
  const Scalar _tmp61 = -_tmp55 - _tmp57 + _tmp59 + _tmp60;
  const Scalar _tmp62 = _tmp53 + _tmp61;
  const Scalar _tmp63 = 2 * _tmp12 * _tmp32;
  const Scalar _tmp64 = 2 * _tmp38;
  const Scalar _tmp65 = _tmp42 * _tmp64;
  const Scalar _tmp66 = Scalar(0.20999999999999999) * _tmp63 + Scalar(0.20999999999999999) * _tmp65;
  const Scalar _tmp67 = -_tmp66;
  const Scalar _tmp68 = -2 * std::pow(_tmp12, Scalar(2));
  const Scalar _tmp69 = 1 - 2 * std::pow(_tmp42, Scalar(2));
  const Scalar _tmp70 = Scalar(0.20999999999999999) * _tmp68 + Scalar(0.20999999999999999) * _tmp69;
  const Scalar _tmp71 = 2 * _tmp42;
  const Scalar _tmp72 = _tmp32 * _tmp71;
  const Scalar _tmp73 = _tmp12 * _tmp64;
  const Scalar _tmp74 = _tmp72 - _tmp73;
  const Scalar _tmp75 = -Scalar(0.010999999999999999) * _tmp74;
  const Scalar _tmp76 = -_tmp70 + _tmp75;
  const Scalar _tmp77 = _tmp67 + _tmp76;
  const Scalar _tmp78 = _tmp77 + p_init1;
  const Scalar _tmp79 = -_tmp78 + Scalar(-8.3196563700000006);
  const Scalar _tmp80 = -_tmp41 - _tmp45;
  const Scalar _tmp81 = _tmp37 + _tmp80;
  const Scalar _tmp82 = _tmp26 * _tmp54;
  const Scalar _tmp83 = _tmp32 * _tmp56;
  const Scalar _tmp84 = _tmp12 * _tmp58;
  const Scalar _tmp85 = _tmp34 * _tmp42;
  const Scalar _tmp86 = Scalar(0.021999999999999999) * _tmp85;
  const Scalar _tmp87 = _tmp82 + _tmp83 + _tmp84 + _tmp86;
  const Scalar _tmp88 = Scalar(0.83999999999999997) * _tmp26;
  const Scalar _tmp89 = _tmp32 * _tmp88;
  const Scalar _tmp90 = _tmp50 - _tmp89;
  const Scalar _tmp91 = _tmp87 + _tmp90;
  const Scalar _tmp92 = Scalar(0.20999999999999999) * _tmp63 - Scalar(0.20999999999999999) * _tmp65;
  const Scalar _tmp93 = -_tmp92;
  const Scalar _tmp94 = -2 * std::pow(_tmp32, Scalar(2));
  const Scalar _tmp95 = Scalar(0.20999999999999999) * _tmp69 + Scalar(0.20999999999999999) * _tmp94;
  const Scalar _tmp96 = _tmp12 * _tmp71;
  const Scalar _tmp97 = _tmp32 * _tmp64;
  const Scalar _tmp98 = _tmp96 + _tmp97;
  const Scalar _tmp99 = -Scalar(0.010999999999999999) * _tmp98;
  const Scalar _tmp100 = -_tmp95 + _tmp99;
  const Scalar _tmp101 = _tmp100 + _tmp93;
  const Scalar _tmp102 = _tmp101 + p_init0;
  const Scalar _tmp103 = -_tmp102 + Scalar(-1.9874742000000001);
  const Scalar _tmp104 =
      std::sqrt(Scalar(std::pow(_tmp103, Scalar(2)) + std::pow(_tmp79, Scalar(2))));
  const Scalar _tmp105 = Scalar(1.0) / (fh1);
  const Scalar _tmp106 = _tmp27 * _tmp38;
  const Scalar _tmp107 = _tmp32 * _tmp44;
  const Scalar _tmp108 = _tmp12 * _tmp40;
  const Scalar _tmp109 = Scalar(0.41999999999999998) * _tmp85;
  const Scalar _tmp110 = _tmp106 + _tmp107 - _tmp108 - _tmp109;
  const Scalar _tmp111 = _tmp26 * _tmp32;
  const Scalar _tmp112 = Scalar(0.043999999999999997) * _tmp111;
  const Scalar _tmp113 = _tmp12 * _tmp34;
  const Scalar _tmp114 = Scalar(0.043999999999999997) * _tmp113;
  const Scalar _tmp115 = _tmp112 + _tmp114;
  const Scalar _tmp116 = _tmp35 * _tmp38;
  const Scalar _tmp117 = _tmp12 * _tmp44;
  const Scalar _tmp118 = _tmp32 * _tmp40;
  const Scalar _tmp119 = _tmp27 * _tmp42;
  const Scalar _tmp120 = -_tmp116 - _tmp117 - _tmp118 - _tmp119;
  const Scalar _tmp121 = _tmp115 + _tmp120;
  const Scalar _tmp122 = _tmp110 + _tmp121;
  const Scalar _tmp123 = _tmp95 + _tmp99;
  const Scalar _tmp124 = _tmp123 + _tmp92;
  const Scalar _tmp125 = _tmp70 + _tmp75;
  const Scalar _tmp126 = _tmp125 + _tmp66;
  const Scalar _tmp127 = _tmp126 + p_init1;
  const Scalar _tmp128 = _tmp127 + Scalar(-4.7752063900000001);
  const Scalar _tmp129 = _tmp124 + p_init0;
  const Scalar _tmp130 = _tmp129 + Scalar(-2.71799795);
  const Scalar _tmp131 = std::pow(_tmp128, Scalar(2)) + std::pow(_tmp130, Scalar(2));
  const Scalar _tmp132 = std::pow(_tmp131, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp133 = _tmp128 * _tmp132;
  const Scalar _tmp134 = _tmp130 * _tmp132;
  const Scalar _tmp135 = _tmp125 + _tmp67;
  const Scalar _tmp136 = _tmp135 + p_init1;
  const Scalar _tmp137 = _tmp136 + Scalar(-4.8333311099999996);
  const Scalar _tmp138 = _tmp100 + _tmp92;
  const Scalar _tmp139 = _tmp138 + p_init0;
  const Scalar _tmp140 = _tmp139 + Scalar(1.79662371);
  const Scalar _tmp141 = std::pow(_tmp140, Scalar(2));
  const Scalar _tmp142 = std::pow(_tmp137, Scalar(2)) + _tmp141;
  const Scalar _tmp143 = std::sqrt(_tmp142);
  const Scalar _tmp144 = Scalar(1.0) / (_tmp143);
  const Scalar _tmp145 = _tmp138 * _tmp144;
  const Scalar _tmp146 = _tmp140 * _tmp144;
  const Scalar _tmp147 = -_tmp135 * _tmp146 + _tmp137 * _tmp145;
  const Scalar _tmp148 = Scalar(1.0) / (_tmp140);
  const Scalar _tmp149 = _tmp143 * _tmp148;
  const Scalar _tmp150 = _tmp147 * _tmp149;
  const Scalar _tmp151 = _tmp123 + _tmp93;
  const Scalar _tmp152 = _tmp151 + p_init0;
  const Scalar _tmp153 = _tmp152 + Scalar(-2.5202214700000001);
  const Scalar _tmp154 = _tmp66 + _tmp76;
  const Scalar _tmp155 = _tmp154 + p_init1;
  const Scalar _tmp156 = _tmp155 + Scalar(8.3888750099999996);
  const Scalar _tmp157 = std::pow(_tmp153, Scalar(2)) + std::pow(_tmp156, Scalar(2));
  const Scalar _tmp158 = std::pow(_tmp157, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp159 = _tmp153 * _tmp158;
  const Scalar _tmp160 = _tmp154 * _tmp158;
  const Scalar _tmp161 = _tmp156 * _tmp158;
  const Scalar _tmp162 = _tmp150 * _tmp159 - _tmp151 * _tmp161 + _tmp153 * _tmp160;
  const Scalar _tmp163 = _tmp137 * _tmp148;
  const Scalar _tmp164 = -_tmp133 + _tmp134 * _tmp163;
  const Scalar _tmp165 = _tmp159 * _tmp163 - _tmp161;
  const Scalar _tmp166 = Scalar(1.0) / (_tmp165);
  const Scalar _tmp167 = _tmp164 * _tmp166;
  const Scalar _tmp168 =
      -_tmp124 * _tmp133 + _tmp126 * _tmp134 + _tmp134 * _tmp150 - _tmp162 * _tmp167;
  const Scalar _tmp169 = Scalar(1.0) / (_tmp168);
  const Scalar _tmp170 = Scalar(1.0) * _tmp169;
  const Scalar _tmp171 = _tmp159 * _tmp167;
  const Scalar _tmp172 = _tmp134 * _tmp170 - _tmp170 * _tmp171;
  const Scalar _tmp173 = _tmp102 + Scalar(1.9874742000000001);
  const Scalar _tmp174 = _tmp78 + Scalar(8.3196563700000006);
  const Scalar _tmp175 = std::pow(_tmp173, Scalar(2)) + std::pow(_tmp174, Scalar(2));
  const Scalar _tmp176 = std::pow(_tmp175, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp177 = _tmp173 * _tmp176;
  const Scalar _tmp178 = _tmp174 * _tmp176;
  const Scalar _tmp179 = _tmp101 * _tmp178 - _tmp177 * _tmp77;
  const Scalar _tmp180 = _tmp179 * fh1;
  const Scalar _tmp181 = _tmp149 * _tmp180;
  const Scalar _tmp182 =
      Scalar(0.20999999999999999) * _tmp96 - Scalar(0.20999999999999999) * _tmp97;
  const Scalar _tmp183 = -_tmp182;
  const Scalar _tmp184 = -Scalar(0.010999999999999999) * _tmp68 -
                         Scalar(0.010999999999999999) * _tmp94 + Scalar(-0.010999999999999999);
  const Scalar _tmp185 =
      Scalar(0.20999999999999999) * _tmp72 + Scalar(0.20999999999999999) * _tmp73;
  const Scalar _tmp186 = _tmp184 + _tmp185;
  const Scalar _tmp187 = _tmp183 + _tmp186;
  const Scalar _tmp188 = _tmp134 * _tmp187;
  const Scalar _tmp189 = _tmp184 - _tmp185;
  const Scalar _tmp190 = _tmp182 + _tmp189;
  const Scalar _tmp191 = _tmp159 * _tmp187;
  const Scalar _tmp192 = _tmp161 * _tmp190 - _tmp163 * _tmp191;
  const Scalar _tmp193 = _tmp182 + _tmp186;
  const Scalar _tmp194 = _tmp133 * _tmp193 - _tmp163 * _tmp188 - _tmp167 * _tmp192;
  const Scalar _tmp195 = Scalar(1.0) * _tmp135;
  const Scalar _tmp196 = -_tmp195;
  const Scalar _tmp197 = _tmp154 + _tmp196;
  const Scalar _tmp198 = Scalar(1.0) / (_tmp197);
  const Scalar _tmp199 = Scalar(1.0) * _tmp138;
  const Scalar _tmp200 = -_tmp151 + _tmp199;
  const Scalar _tmp201 = _tmp198 * _tmp200;
  const Scalar _tmp202 = -_tmp159 * _tmp190 + _tmp191;
  const Scalar _tmp203 = -_tmp134 * _tmp193 - _tmp167 * _tmp202 + _tmp188 - _tmp194 * _tmp201;
  const Scalar _tmp204 = Scalar(1.0) / (_tmp203);
  const Scalar _tmp205 = _tmp195 * _tmp201 + _tmp199;
  const Scalar _tmp206 = 0;
  const Scalar _tmp207 = _tmp204 * _tmp206;
  const Scalar _tmp208 = _tmp134 * _tmp207 - _tmp171 * _tmp207;
  const Scalar _tmp209 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp210 = _tmp149 * _tmp209;
  const Scalar _tmp211 = _tmp177 * fh1;
  const Scalar _tmp212 = _tmp163 * _tmp166;
  const Scalar _tmp213 = _tmp163 * _tmp187;
  const Scalar _tmp214 = _tmp192 * _tmp212 + _tmp213;
  const Scalar _tmp215 = -_tmp187 - _tmp201 * _tmp214 + _tmp202 * _tmp212;
  const Scalar _tmp216 = _tmp168 * _tmp204;
  const Scalar _tmp217 = -_tmp150 + _tmp162 * _tmp212 - _tmp215 * _tmp216;
  const Scalar _tmp218 = _tmp169 * _tmp203;
  const Scalar _tmp219 = _tmp217 * _tmp218;
  const Scalar _tmp220 = _tmp215 + _tmp219;
  const Scalar _tmp221 = _tmp134 * _tmp204;
  const Scalar _tmp222 = _tmp164 * _tmp204;
  const Scalar _tmp223 = -_tmp163 - _tmp220 * _tmp222;
  const Scalar _tmp224 = _tmp166 * _tmp223;
  const Scalar _tmp225 = _tmp159 * _tmp224 + _tmp220 * _tmp221 + Scalar(1.0);
  const Scalar _tmp226 = _tmp149 * _tmp225;
  const Scalar _tmp227 = Scalar(1.0) * _tmp166;
  const Scalar _tmp228 = Scalar(1.0) * _tmp198;
  const Scalar _tmp229 = _tmp200 * _tmp228;
  const Scalar _tmp230 = _tmp166 * _tmp229;
  const Scalar _tmp231 = _tmp192 * _tmp230 - _tmp202 * _tmp227;
  const Scalar _tmp232 = -_tmp162 * _tmp227 - _tmp216 * _tmp231;
  const Scalar _tmp233 = _tmp218 * _tmp232;
  const Scalar _tmp234 = _tmp231 + _tmp233;
  const Scalar _tmp235 = -_tmp222 * _tmp234 + Scalar(1.0);
  const Scalar _tmp236 = _tmp159 * _tmp166;
  const Scalar _tmp237 = _tmp221 * _tmp234 + _tmp235 * _tmp236;
  const Scalar _tmp238 = _tmp178 * fh1;
  const Scalar _tmp239 = _tmp149 * _tmp238;
  const Scalar _tmp240 =
      -_tmp172 * _tmp181 - _tmp208 * _tmp210 - _tmp211 * _tmp226 - _tmp237 * _tmp239;
  const Scalar _tmp241 = Scalar(1.0) / (_tmp240);
  const Scalar _tmp242 = _tmp126 + _tmp196;
  const Scalar _tmp243 = _tmp198 * _tmp242;
  const Scalar _tmp244 = _tmp200 * _tmp243;
  const Scalar _tmp245 = -_tmp124 + _tmp199 - _tmp244;
  const Scalar _tmp246 = Scalar(1.0) / (_tmp245);
  const Scalar _tmp247 = Scalar(1.0) * _tmp246;
  const Scalar _tmp248 = _tmp244 * _tmp247 + Scalar(1.0);
  const Scalar _tmp249 = _tmp201 * _tmp247;
  const Scalar _tmp250 = -_tmp228 * _tmp248 + _tmp249;
  const Scalar _tmp251 = _tmp183 + _tmp189;
  const Scalar _tmp252 = -_tmp238 * _tmp251 - Scalar(3.29616) * _tmp74 - _tmp77 * fv1;
  const Scalar _tmp253 = Scalar(1.0) * _tmp252;
  const Scalar _tmp254 = _tmp205 * _tmp246;
  const Scalar _tmp255 = _tmp194 * _tmp204;
  const Scalar _tmp256 = _tmp196 - _tmp206 * _tmp255 - _tmp242 * _tmp254;
  const Scalar _tmp257 = Scalar(1.0) * _tmp209;
  const Scalar _tmp258 = _tmp242 * _tmp246;
  const Scalar _tmp259 = _tmp192 * _tmp227;
  const Scalar _tmp260 = _tmp233 * _tmp258 - _tmp234 * _tmp255 - _tmp259;
  const Scalar _tmp261 = -_tmp228 * _tmp260 + _tmp233 * _tmp247;
  const Scalar _tmp262 = Scalar(1.0) * _tmp261;
  const Scalar _tmp263 = _tmp101 * fv1 + _tmp211 * _tmp251 + Scalar(3.29616) * _tmp98;
  const Scalar _tmp264 = Scalar(1.0) * _tmp243 * _tmp247 - Scalar(1.0) * _tmp247;
  const Scalar _tmp265 = _tmp214 + _tmp219 * _tmp258 - _tmp220 * _tmp255;
  const Scalar _tmp266 = _tmp219 * _tmp247 - _tmp228 * _tmp265;
  const Scalar _tmp267 = Scalar(1.0) * _tmp266;
  const Scalar _tmp268 = _tmp218 * _tmp247;
  const Scalar _tmp269 = -_tmp170 * _tmp194 + _tmp242 * _tmp268;
  const Scalar _tmp270 = -Scalar(1.0) * _tmp228 * _tmp269 + Scalar(1.0) * _tmp268;
  const Scalar _tmp271 =
      _tmp180 * _tmp270 + _tmp211 * _tmp267 + _tmp238 * _tmp262 + _tmp250 * _tmp253 +
      _tmp257 * (-_tmp205 * _tmp247 - _tmp228 * _tmp256 + Scalar(1.0)) + _tmp263 * _tmp264;
  const Scalar _tmp272 = std::asinh(_tmp241 * _tmp271);
  const Scalar _tmp273 = Scalar(9.6622558468725703) * _tmp240;
  const Scalar _tmp274 = Scalar(4.8333311099999996) - _tmp136;
  const Scalar _tmp275 = -_tmp139 + Scalar(-1.79662371);
  const Scalar _tmp276 =
      std::sqrt(Scalar(std::pow(_tmp274, Scalar(2)) + std::pow(_tmp275, Scalar(2))));
  const Scalar _tmp277 = -_tmp272 * _tmp273 - _tmp276;
  const Scalar _tmp278 = Scalar(0.1034955) * _tmp241;
  const Scalar _tmp279 = _tmp277 * _tmp278;
  const Scalar _tmp280 = Scalar(1.0) * _tmp272;
  const Scalar _tmp281 = -_tmp82 - _tmp83 - _tmp84 - _tmp86;
  const Scalar _tmp282 = _tmp49 + _tmp89;
  const Scalar _tmp283 = -_tmp28 - _tmp36;
  const Scalar _tmp284 = _tmp283 + _tmp46;
  const Scalar _tmp285 = _tmp282 + _tmp284;
  const Scalar _tmp286 = _tmp281 + _tmp285;
  const Scalar _tmp287 = _tmp55 + _tmp57 - _tmp59 - _tmp60;
  const Scalar _tmp288 = _tmp49 + _tmp52;
  const Scalar _tmp289 = _tmp283 + _tmp80;
  const Scalar _tmp290 = _tmp288 + _tmp289;
  const Scalar _tmp291 = _tmp287 + _tmp290;
  const Scalar _tmp292 =
      (2 * _tmp173 * _tmp286 + 2 * _tmp174 * _tmp291) / (_tmp175 * std::sqrt(_tmp175));
  const Scalar _tmp293 = (Scalar(1) / Scalar(2)) * _tmp292;
  const Scalar _tmp294 = _tmp173 * _tmp293;
  const Scalar _tmp295 = _tmp174 * _tmp293;
  const Scalar _tmp296 = _tmp176 * _tmp291;
  const Scalar _tmp297 = _tmp176 * _tmp286;
  const Scalar _tmp298 = -_tmp101 * _tmp295 + _tmp101 * _tmp296 - _tmp173 * _tmp296 +
                         _tmp174 * _tmp297 + _tmp294 * _tmp77 - _tmp297 * _tmp77;
  const Scalar _tmp299 = _tmp298 * fh1;
  const Scalar _tmp300 = _tmp282 + _tmp81;
  const Scalar _tmp301 = _tmp281 + _tmp300;
  const Scalar _tmp302 = _tmp287 + _tmp53;
  const Scalar _tmp303 = _tmp289 + _tmp302;
  const Scalar _tmp304 = _tmp137 * _tmp303 + _tmp140 * _tmp301;
  const Scalar _tmp305 = _tmp144 * _tmp148 * _tmp304;
  const Scalar _tmp306 = _tmp147 * _tmp305;
  const Scalar _tmp307 = Scalar(1.0) / (_tmp141);
  const Scalar _tmp308 = _tmp143 * _tmp301 * _tmp307;
  const Scalar _tmp309 = _tmp147 * _tmp308;
  const Scalar _tmp310 = _tmp302 + _tmp47;
  const Scalar _tmp311 = _tmp281 + _tmp90;
  const Scalar _tmp312 = _tmp311 + _tmp81;
  const Scalar _tmp313 =
      (2 * _tmp128 * _tmp310 + 2 * _tmp130 * _tmp312) / (_tmp131 * std::sqrt(_tmp131));
  const Scalar _tmp314 = (Scalar(1) / Scalar(2)) * _tmp313;
  const Scalar _tmp315 = _tmp130 * _tmp314;
  const Scalar _tmp316 = _tmp132 * _tmp312;
  const Scalar _tmp317 = _tmp288 + _tmp47;
  const Scalar _tmp318 = _tmp287 + _tmp317;
  const Scalar _tmp319 = _tmp284 + _tmp311;
  const Scalar _tmp320 =
      (2 * _tmp153 * _tmp319 + 2 * _tmp156 * _tmp318) / (_tmp157 * std::sqrt(_tmp157));
  const Scalar _tmp321 = (Scalar(1) / Scalar(2)) * _tmp320;
  const Scalar _tmp322 = _tmp156 * _tmp321;
  const Scalar _tmp323 = _tmp158 * _tmp319;
  const Scalar _tmp324 = _tmp137 * _tmp301;
  const Scalar _tmp325 = _tmp307 * _tmp324;
  const Scalar _tmp326 = _tmp148 * _tmp303;
  const Scalar _tmp327 = _tmp153 * _tmp321;
  const Scalar _tmp328 = _tmp158 * _tmp318;
  const Scalar _tmp329 = (-_tmp159 * _tmp325 + _tmp159 * _tmp326 + _tmp163 * _tmp323 -
                          _tmp163 * _tmp327 + _tmp322 - _tmp328) /
                         std::pow(_tmp165, Scalar(2));
  const Scalar _tmp330 = _tmp164 * _tmp329;
  const Scalar _tmp331 = _tmp132 * _tmp310;
  const Scalar _tmp332 = _tmp304 / (_tmp142 * std::sqrt(_tmp142));
  const Scalar _tmp333 = _tmp149 * (_tmp135 * _tmp140 * _tmp332 - _tmp135 * _tmp144 * _tmp301 -
                                    _tmp137 * _tmp138 * _tmp332 + _tmp144 * _tmp324 +
                                    _tmp145 * _tmp303 - _tmp146 * _tmp303);
  const Scalar _tmp334 = _tmp150 * _tmp323 - _tmp150 * _tmp327 + _tmp151 * _tmp322 -
                         _tmp151 * _tmp328 + _tmp153 * _tmp328 - _tmp154 * _tmp327 +
                         _tmp159 * _tmp306 - _tmp159 * _tmp309 + _tmp159 * _tmp333 +
                         _tmp160 * _tmp319 - _tmp161 * _tmp319;
  const Scalar _tmp335 = _tmp128 * _tmp314;
  const Scalar _tmp336 = -_tmp134 * _tmp325 + _tmp134 * _tmp326 - _tmp163 * _tmp315 +
                         _tmp163 * _tmp316 - _tmp331 + _tmp335;
  const Scalar _tmp337 = _tmp166 * _tmp336;
  const Scalar _tmp338 = -_tmp124 * _tmp331 + _tmp124 * _tmp335 - _tmp126 * _tmp315 +
                         _tmp126 * _tmp316 + _tmp130 * _tmp331 - _tmp133 * _tmp312 +
                         _tmp134 * _tmp306 - _tmp134 * _tmp309 + _tmp134 * _tmp333 -
                         _tmp150 * _tmp315 + _tmp150 * _tmp316 + _tmp162 * _tmp330 -
                         _tmp162 * _tmp337 - _tmp167 * _tmp334;
  const Scalar _tmp339 = _tmp338 / std::pow(_tmp168, Scalar(2));
  const Scalar _tmp340 = Scalar(1.0) * _tmp339;
  const Scalar _tmp341 = _tmp159 * _tmp337;
  const Scalar _tmp342 = Scalar(0.5) * _tmp169;
  const Scalar _tmp343 = _tmp167 * _tmp323;
  const Scalar _tmp344 = _tmp237 * _tmp238;
  const Scalar _tmp345 = _tmp172 * _tmp180;
  const Scalar _tmp346 = _tmp166 * _tmp235;
  const Scalar _tmp347 = _tmp204 * _tmp336;
  const Scalar _tmp348 = Scalar(1.6799999999999999) * _tmp39 * _tmp42;
  const Scalar _tmp349 = Scalar(0.83999999999999997) * _tmp38 * _tmp39;
  const Scalar _tmp350 = _tmp43 * _tmp48;
  const Scalar _tmp351 = _tmp12 * _tmp88 + _tmp32 * _tmp51;
  const Scalar _tmp352 = _tmp349 + _tmp350 + _tmp351;
  const Scalar _tmp353 =
      (Scalar(1.6799999999999999) * _tmp113 + _tmp348 + _tmp352) / std::pow(_tmp197, Scalar(2));
  const Scalar _tmp354 = _tmp200 * _tmp353;
  const Scalar _tmp355 = _tmp115 + _tmp116 + _tmp117 + _tmp118 + _tmp119;
  const Scalar _tmp356 = -_tmp106 - _tmp107 + _tmp108 + _tmp109;
  const Scalar _tmp357 = _tmp355 + _tmp356;
  const Scalar _tmp358 = _tmp110 + _tmp355;
  const Scalar _tmp359 = _tmp134 * _tmp358;
  const Scalar _tmp360 = _tmp187 * _tmp316;
  const Scalar _tmp361 = _tmp121 + _tmp356;
  const Scalar _tmp362 = _tmp159 * _tmp358;
  const Scalar _tmp363 = -_tmp159 * _tmp361 + _tmp187 * _tmp323 - _tmp187 * _tmp327 -
                         _tmp190 * _tmp323 + _tmp190 * _tmp327 + _tmp362;
  const Scalar _tmp364 = _tmp202 * _tmp329;
  const Scalar _tmp365 = _tmp187 * _tmp326;
  const Scalar _tmp366 = -_tmp159 * _tmp365 + _tmp161 * _tmp361 - _tmp163 * _tmp362 -
                         _tmp190 * _tmp322 + _tmp190 * _tmp328 + _tmp191 * _tmp325 -
                         _tmp213 * _tmp323 + _tmp213 * _tmp327;
  const Scalar _tmp367 = _tmp133 * _tmp357 - _tmp134 * _tmp365 - _tmp163 * _tmp359 -
                         _tmp163 * _tmp360 - _tmp167 * _tmp366 + _tmp188 * _tmp325 +
                         _tmp192 * _tmp330 - _tmp192 * _tmp337 + _tmp193 * _tmp331 -
                         _tmp193 * _tmp335 + _tmp213 * _tmp315;
  const Scalar _tmp368 = Scalar(1.6799999999999999) * _tmp111 + _tmp348;
  const Scalar _tmp369 = -_tmp349 - _tmp350 + _tmp351 + _tmp368;
  const Scalar _tmp370 = _tmp198 * _tmp369;
  const Scalar _tmp371 = -_tmp134 * _tmp357 + _tmp164 * _tmp364 - _tmp167 * _tmp363 -
                         _tmp187 * _tmp315 + _tmp193 * _tmp315 - _tmp193 * _tmp316 +
                         _tmp194 * _tmp354 - _tmp194 * _tmp370 - _tmp201 * _tmp367 -
                         _tmp202 * _tmp337 + _tmp359 + _tmp360;
  const Scalar _tmp372 = _tmp371 / std::pow(_tmp203, Scalar(2));
  const Scalar _tmp373 = _tmp164 * _tmp372;
  const Scalar _tmp374 = _tmp169 * _tmp371;
  const Scalar _tmp375 = _tmp232 * _tmp374;
  const Scalar _tmp376 = _tmp203 * _tmp339;
  const Scalar _tmp377 = _tmp232 * _tmp376;
  const Scalar _tmp378 = Scalar(1.0) * _tmp329;
  const Scalar _tmp379 = _tmp204 * _tmp338;
  const Scalar _tmp380 = _tmp168 * _tmp372;
  const Scalar _tmp381 = _tmp166 * _tmp192 * _tmp228 * _tmp369 - _tmp192 * _tmp229 * _tmp329 -
                         _tmp227 * _tmp363 + _tmp230 * _tmp366 - _tmp259 * _tmp354 +
                         Scalar(1.0) * _tmp364;
  const Scalar _tmp382 = _tmp218 * (_tmp162 * _tmp378 - _tmp216 * _tmp381 - _tmp227 * _tmp334 -
                                    _tmp231 * _tmp379 + _tmp231 * _tmp380);
  const Scalar _tmp383 = _tmp375 - _tmp377 + _tmp381 + _tmp382;
  const Scalar _tmp384 = -_tmp222 * _tmp383 - _tmp234 * _tmp347 + _tmp234 * _tmp373;
  const Scalar _tmp385 = _tmp204 * _tmp234;
  const Scalar _tmp386 = _tmp159 * _tmp329;
  const Scalar _tmp387 = _tmp134 * _tmp372;
  const Scalar _tmp388 = _tmp204 * _tmp316;
  const Scalar _tmp389 = _tmp211 * _tmp225;
  const Scalar _tmp390 = _tmp208 * _tmp209;
  const Scalar _tmp391 = _tmp149 * _tmp237;
  const Scalar _tmp392 = _tmp295 * fh1;
  const Scalar _tmp393 = _tmp296 * fh1;
  const Scalar _tmp394 = _tmp206 * _tmp372;
  const Scalar _tmp395 = _tmp206 * _tmp222;
  const Scalar _tmp396 = _tmp297 * fh1;
  const Scalar _tmp397 = _tmp294 * fh1;
  const Scalar _tmp398 = _tmp217 * _tmp374;
  const Scalar _tmp399 = _tmp163 * _tmp329;
  const Scalar _tmp400 = _tmp166 * _tmp326;
  const Scalar _tmp401 = _tmp166 * _tmp325;
  const Scalar _tmp402 = _tmp163 * _tmp358 - _tmp187 * _tmp325 - _tmp192 * _tmp399 +
                         _tmp192 * _tmp400 - _tmp192 * _tmp401 + _tmp212 * _tmp366 + _tmp365;
  const Scalar _tmp403 = -_tmp112 - _tmp114 + _tmp120 - _tmp163 * _tmp364 - _tmp201 * _tmp402 +
                         _tmp202 * _tmp400 - _tmp202 * _tmp401 + _tmp212 * _tmp363 +
                         _tmp214 * _tmp354 - _tmp214 * _tmp370 + _tmp356;
  const Scalar _tmp404 = _tmp218 * (-_tmp162 * _tmp399 + _tmp162 * _tmp400 - _tmp162 * _tmp401 +
                                    _tmp212 * _tmp334 - _tmp215 * _tmp379 + _tmp215 * _tmp380 -
                                    _tmp216 * _tmp403 - _tmp306 + _tmp309 - _tmp333);
  const Scalar _tmp405 = _tmp203 * _tmp217;
  const Scalar _tmp406 = _tmp339 * _tmp405;
  const Scalar _tmp407 = _tmp398 + _tmp403 + _tmp404 - _tmp406;
  const Scalar _tmp408 =
      -_tmp220 * _tmp347 + _tmp220 * _tmp373 - _tmp222 * _tmp407 + _tmp325 - _tmp326;
  const Scalar _tmp409 = _tmp204 * _tmp220;
  const Scalar _tmp410 =
      -_tmp149 * _tmp172 * _tmp299 -
      _tmp149 * _tmp211 *
          (-_tmp220 * _tmp387 + _tmp220 * _tmp388 + _tmp221 * _tmp407 - _tmp223 * _tmp386 +
           _tmp224 * _tmp323 - _tmp224 * _tmp327 + _tmp236 * _tmp408 - _tmp315 * _tmp409) -
      _tmp181 * (-_tmp130 * _tmp313 * _tmp342 - _tmp134 * _tmp340 +
                 _tmp153 * _tmp167 * _tmp320 * _tmp342 + _tmp159 * _tmp170 * _tmp330 +
                 _tmp170 * _tmp316 - _tmp170 * _tmp341 - _tmp170 * _tmp343 + _tmp171 * _tmp340) -
      _tmp210 * (-_tmp134 * _tmp394 + _tmp167 * _tmp207 * _tmp327 + _tmp171 * _tmp394 -
                 _tmp207 * _tmp315 + _tmp207 * _tmp316 - _tmp207 * _tmp341 - _tmp207 * _tmp343 +
                 _tmp386 * _tmp395) -
      _tmp226 * _tmp396 + _tmp226 * _tmp397 -
      _tmp239 * (_tmp221 * _tmp383 - _tmp234 * _tmp387 + _tmp234 * _tmp388 - _tmp235 * _tmp386 +
                 _tmp236 * _tmp384 - _tmp315 * _tmp385 + _tmp323 * _tmp346 - _tmp327 * _tmp346) -
      _tmp305 * _tmp344 - _tmp305 * _tmp345 - _tmp305 * _tmp389 - _tmp305 * _tmp390 +
      _tmp308 * _tmp344 + _tmp308 * _tmp345 + _tmp308 * _tmp389 + _tmp308 * _tmp390 +
      _tmp391 * _tmp392 - _tmp391 * _tmp393;
  const Scalar _tmp411 = Scalar(9.6622558468725703) * _tmp410;
  const Scalar _tmp412 = std::pow(_tmp240, Scalar(-2));
  const Scalar _tmp413 = _tmp410 * _tmp412;
  const Scalar _tmp414 = _tmp242 * _tmp354;
  const Scalar _tmp415 = _tmp201 * _tmp352;
  const Scalar _tmp416 = _tmp243 * _tmp369;
  const Scalar _tmp417 = (_tmp368 + _tmp414 - _tmp415 - _tmp416) / std::pow(_tmp245, Scalar(2));
  const Scalar _tmp418 = _tmp170 * _tmp417;
  const Scalar _tmp419 = _tmp242 * _tmp417;
  const Scalar _tmp420 = _tmp194 * _tmp372;
  const Scalar _tmp421 = _tmp204 * _tmp367;
  const Scalar _tmp422 = _tmp246 * _tmp352;
  const Scalar _tmp423 = -_tmp219 * _tmp419 + _tmp219 * _tmp422 + _tmp220 * _tmp420 -
                         _tmp220 * _tmp421 - _tmp255 * _tmp407 + _tmp258 * _tmp398 +
                         _tmp258 * _tmp404 - _tmp258 * _tmp406 + _tmp402;
  const Scalar _tmp424 = Scalar(1.0) * _tmp353;
  const Scalar _tmp425 = _tmp247 * _tmp370;
  const Scalar _tmp426 = Scalar(1.0) * _tmp417;
  const Scalar _tmp427 =
      _tmp198 * (-_tmp244 * _tmp426 - _tmp247 * _tmp414 + _tmp247 * _tmp415 + _tmp247 * _tmp416);
  const Scalar _tmp428 = _tmp229 * _tmp417;
  const Scalar _tmp429 = _tmp247 * _tmp354;
  const Scalar _tmp430 = _tmp203 * _tmp418;
  const Scalar _tmp431 = _tmp247 * _tmp374;
  const Scalar _tmp432 = _tmp247 * _tmp376;
  const Scalar _tmp433 = -_tmp170 * _tmp367 + _tmp194 * _tmp340 - _tmp242 * _tmp430 +
                         _tmp242 * _tmp431 - _tmp242 * _tmp432 + _tmp268 * _tmp352;
  const Scalar _tmp434 = Scalar(0.5) * _tmp292 * fh1;
  const Scalar _tmp435 = Scalar(6.59232) * _tmp26;
  const Scalar _tmp436 = _tmp251 * fh1;
  const Scalar _tmp437 = Scalar(6.59232) * _tmp43;
  const Scalar _tmp438 = Scalar(6.59232) * _tmp39;
  const Scalar _tmp439 = _tmp12 * _tmp438 + _tmp122 * _tmp211 + _tmp286 * fv1 - _tmp294 * _tmp436 +
                         _tmp297 * _tmp436 + _tmp32 * _tmp437 + _tmp38 * _tmp435 +
                         Scalar(6.59232) * _tmp85;
  const Scalar _tmp440 = _tmp12 * _tmp437 - _tmp122 * _tmp238 - _tmp291 * fv1 + _tmp295 * _tmp436 -
                         _tmp296 * _tmp436 - _tmp32 * _tmp438 + Scalar(6.59232) * _tmp34 * _tmp38 -
                         _tmp42 * _tmp435;
  const Scalar _tmp441 = -_tmp195 * _tmp354 + _tmp195 * _tmp370 + _tmp229 * _tmp303 + _tmp301;
  const Scalar _tmp442 = _tmp246 * _tmp441;
  const Scalar _tmp443 = _tmp205 * _tmp417;
  const Scalar _tmp444 = _tmp317 + _tmp61;
  const Scalar _tmp445 = _tmp206 * _tmp420 - _tmp206 * _tmp421 - _tmp242 * _tmp442 +
                         _tmp242 * _tmp443 - _tmp254 * _tmp352 + _tmp444;
  const Scalar _tmp446 = _tmp198 * _tmp352;
  const Scalar _tmp447 = _tmp242 * _tmp353;
  const Scalar _tmp448 = _tmp192 * _tmp378 - _tmp227 * _tmp366 - _tmp233 * _tmp419 +
                         _tmp233 * _tmp422 + _tmp234 * _tmp420 - _tmp234 * _tmp421 -
                         _tmp255 * _tmp383 + _tmp258 * _tmp375 - _tmp258 * _tmp377 +
                         _tmp258 * _tmp382;
  const Scalar _tmp449 =
      (_tmp241 *
           (-_tmp173 * _tmp266 * _tmp434 - _tmp174 * _tmp261 * _tmp434 +
            Scalar(1.0) * _tmp180 *
                (-_tmp228 * _tmp433 + _tmp269 * _tmp424 - _tmp430 + _tmp431 - _tmp432) +
            Scalar(1.0) * _tmp211 *
                (-_tmp228 * _tmp423 + _tmp247 * _tmp398 + _tmp247 * _tmp404 - _tmp247 * _tmp406 +
                 _tmp265 * _tmp424 - _tmp405 * _tmp418) +
            Scalar(1.0) * _tmp238 *
                (-_tmp228 * _tmp448 - _tmp232 * _tmp430 + _tmp247 * _tmp375 - _tmp247 * _tmp377 +
                 _tmp247 * _tmp382 + _tmp260 * _tmp424) +
            Scalar(1.0) * _tmp250 * _tmp440 +
            _tmp253 * (_tmp248 * _tmp424 + _tmp425 - Scalar(1.0) * _tmp427 - _tmp428 - _tmp429) +
            _tmp257 *
                (_tmp205 * _tmp426 - _tmp228 * _tmp445 - _tmp247 * _tmp441 + _tmp256 * _tmp424) +
            _tmp262 * _tmp393 +
            Scalar(1.0) * _tmp263 *
                (-_tmp243 * _tmp426 + _tmp247 * _tmp446 - _tmp247 * _tmp447 + _tmp426) +
            _tmp264 * _tmp439 + _tmp267 * _tmp396 + _tmp270 * _tmp299) -
       _tmp271 * _tmp413) /
      std::sqrt(Scalar(std::pow(_tmp271, Scalar(2)) * _tmp412 + 1));
  const Scalar _tmp450 = _tmp170 * fh1;
  const Scalar _tmp451 = _tmp179 * _tmp450;
  const Scalar _tmp452 = _tmp207 * _tmp209;
  const Scalar _tmp453 =
      -_tmp167 * _tmp451 - _tmp167 * _tmp452 + _tmp211 * _tmp224 + _tmp238 * _tmp346;
  const Scalar _tmp454 = Scalar(1.0) / (_tmp453);
  const Scalar _tmp455 = _tmp198 * _tmp248;
  const Scalar _tmp456 = _tmp198 * _tmp209;
  const Scalar _tmp457 = _tmp198 * _tmp269;
  const Scalar _tmp458 = _tmp198 * _tmp238;
  const Scalar _tmp459 = _tmp198 * _tmp265;
  const Scalar _tmp460 = _tmp247 * _tmp263;
  const Scalar _tmp461 = _tmp180 * _tmp457 + _tmp211 * _tmp459 - _tmp243 * _tmp460 +
                         _tmp252 * _tmp455 + _tmp256 * _tmp456 + _tmp260 * _tmp458;
  const Scalar _tmp462 = std::asinh(_tmp454 * _tmp461);
  const Scalar _tmp463 = Scalar(1.0) * _tmp462;
  const Scalar _tmp464 = _tmp198 * _tmp260;
  const Scalar _tmp465 = _tmp247 * _tmp439;
  const Scalar _tmp466 = _tmp263 * _tmp426;
  const Scalar _tmp467 = std::pow(_tmp453, Scalar(-2));
  const Scalar _tmp468 = _tmp180 * _tmp340;
  const Scalar _tmp469 = _tmp298 * _tmp450;
  const Scalar _tmp470 = _tmp209 * _tmp394;
  const Scalar _tmp471 = _tmp166 * _tmp211 * _tmp408 + _tmp166 * _tmp238 * _tmp384 +
                         _tmp167 * _tmp468 - _tmp167 * _tmp469 + _tmp167 * _tmp470 +
                         _tmp209 * _tmp329 * _tmp395 - _tmp211 * _tmp223 * _tmp329 +
                         _tmp224 * _tmp396 - _tmp224 * _tmp397 - _tmp235 * _tmp238 * _tmp329 +
                         _tmp330 * _tmp451 - _tmp337 * _tmp451 - _tmp337 * _tmp452 -
                         _tmp346 * _tmp392 + _tmp346 * _tmp393;
  const Scalar _tmp472 = _tmp467 * _tmp471;
  const Scalar _tmp473 =
      (_tmp454 * (_tmp180 * _tmp198 * _tmp433 - _tmp180 * _tmp269 * _tmp353 +
                  _tmp198 * _tmp211 * _tmp423 - _tmp209 * _tmp256 * _tmp353 -
                  _tmp211 * _tmp265 * _tmp353 - _tmp238 * _tmp260 * _tmp353 - _tmp243 * _tmp465 +
                  _tmp243 * _tmp466 - _tmp248 * _tmp252 * _tmp353 + _tmp252 * _tmp427 +
                  _tmp299 * _tmp457 - _tmp392 * _tmp464 + _tmp393 * _tmp464 + _tmp396 * _tmp459 -
                  _tmp397 * _tmp459 + _tmp440 * _tmp455 + _tmp445 * _tmp456 - _tmp446 * _tmp460 +
                  _tmp447 * _tmp460 + _tmp448 * _tmp458) -
       _tmp461 * _tmp472) /
      std::sqrt(Scalar(std::pow(_tmp461, Scalar(2)) * _tmp467 + 1));
  const Scalar _tmp474 = Scalar(9.6622558468725703) * _tmp471;
  const Scalar _tmp475 = -_tmp155 + Scalar(-8.3888750099999996);
  const Scalar _tmp476 = Scalar(2.5202214700000001) - _tmp152;
  const Scalar _tmp477 =
      std::sqrt(Scalar(std::pow(_tmp475, Scalar(2)) + std::pow(_tmp476, Scalar(2))));
  const Scalar _tmp478 = Scalar(9.6622558468725703) * _tmp453;
  const Scalar _tmp479 = Scalar(0.1034955) * _tmp454;
  const Scalar _tmp480 = -_tmp462 * _tmp478 - _tmp477;
  const Scalar _tmp481 = _tmp479 * _tmp480;
  const Scalar _tmp482 = Scalar(4.7752063900000001) - _tmp127;
  const Scalar _tmp483 = Scalar(2.71799795) - _tmp129;
  const Scalar _tmp484 =
      std::sqrt(Scalar(std::pow(_tmp482, Scalar(2)) + std::pow(_tmp483, Scalar(2))));
  const Scalar _tmp485 = _tmp233 * _tmp246;
  const Scalar _tmp486 = _tmp219 * _tmp246;
  const Scalar _tmp487 = -_tmp180 * _tmp268 + _tmp209 * _tmp254 - _tmp211 * _tmp486 -
                         _tmp238 * _tmp485 - _tmp249 * _tmp252 + _tmp460;
  const Scalar _tmp488 = _tmp204 * _tmp211;
  const Scalar _tmp489 = _tmp204 * _tmp238;
  const Scalar _tmp490 = _tmp220 * _tmp488 + _tmp234 * _tmp489 + _tmp451 + _tmp452;
  const Scalar _tmp491 = Scalar(1.0) / (_tmp490);
  const Scalar _tmp492 = std::asinh(_tmp487 * _tmp491);
  const Scalar _tmp493 = Scalar(9.6622558468725703) * _tmp490;
  const Scalar _tmp494 = -_tmp484 - _tmp492 * _tmp493;
  const Scalar _tmp495 = Scalar(0.1034955) * _tmp491;
  const Scalar _tmp496 = _tmp494 * _tmp495;
  const Scalar _tmp497 = Scalar(1.0) * _tmp492;
  const Scalar _tmp498 = -_tmp211 * _tmp220 * _tmp372 - _tmp234 * _tmp238 * _tmp372 +
                         _tmp383 * _tmp489 - _tmp385 * _tmp392 + _tmp385 * _tmp393 +
                         _tmp396 * _tmp409 - _tmp397 * _tmp409 + _tmp407 * _tmp488 - _tmp468 +
                         _tmp469 - _tmp470;
  const Scalar _tmp499 = Scalar(9.6622558468725703) * _tmp498;
  const Scalar _tmp500 = std::pow(_tmp490, Scalar(-2));
  const Scalar _tmp501 = _tmp498 * _tmp500;
  const Scalar _tmp502 = _tmp211 * _tmp246;
  const Scalar _tmp503 = _tmp238 * _tmp246;
  const Scalar _tmp504 =
      (-_tmp487 * _tmp501 +
       _tmp491 * (-_tmp180 * _tmp431 + _tmp180 * _tmp432 + _tmp203 * _tmp417 * _tmp451 +
                  _tmp209 * _tmp442 - _tmp209 * _tmp443 + _tmp211 * _tmp219 * _tmp417 +
                  _tmp233 * _tmp238 * _tmp417 - _tmp249 * _tmp440 - _tmp252 * _tmp425 +
                  _tmp252 * _tmp428 + _tmp252 * _tmp429 - _tmp268 * _tmp299 - _tmp375 * _tmp503 +
                  _tmp377 * _tmp503 - _tmp382 * _tmp503 + _tmp392 * _tmp485 - _tmp393 * _tmp485 -
                  _tmp396 * _tmp486 + _tmp397 * _tmp486 - _tmp398 * _tmp502 - _tmp404 * _tmp502 +
                  _tmp406 * _tmp502 + _tmp465 - _tmp466)) /
      std::sqrt(Scalar(std::pow(_tmp487, Scalar(2)) * _tmp500 + 1));

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) =
      _tmp122 -
      Scalar(0.5) * (2 * _tmp103 * (_tmp81 + _tmp91) + 2 * _tmp79 * (_tmp47 + _tmp62)) *
          std::sinh(Scalar(0.1034955) * _tmp105 *
                    (-_tmp104 - Scalar(9.6622558468725703) * fh1 * std::asinh(_tmp105 * fv1))) /
          _tmp104;
  _res(1, 0) =
      -_tmp273 *
          (-Scalar(0.86625939559540499) * _tmp413 + Scalar(1.0) * _tmp449 * std::sinh(_tmp280) -
           (-Scalar(0.1034955) * _tmp277 * _tmp413 +
            _tmp278 * (-_tmp272 * _tmp411 - _tmp273 * _tmp449 -
                       Scalar(1) / Scalar(2) *
                           (2 * _tmp274 * _tmp444 + 2 * _tmp275 * (_tmp284 + _tmp91)) / _tmp276)) *
               std::sinh(_tmp279)) +
      _tmp358 -
      _tmp411 * (Scalar(0.86625939559540499) * _tmp241 - std::cosh(_tmp279) + std::cosh(_tmp280));
  _res(2, 0) =
      _tmp361 -
      _tmp474 * (Scalar(0.87653584775870996) * _tmp454 + std::cosh(_tmp463) - std::cosh(_tmp481)) -
      _tmp478 *
          (-Scalar(0.87653584775870996) * _tmp472 + Scalar(1.0) * _tmp473 * std::sinh(_tmp463) -
           (-Scalar(0.1034955) * _tmp472 * _tmp480 +
            _tmp479 * (-_tmp462 * _tmp474 - _tmp473 * _tmp478 -
                       Scalar(1) / Scalar(2) *
                           (2 * _tmp475 * (_tmp289 + _tmp62) + 2 * _tmp476 * (_tmp300 + _tmp87)) /
                           _tmp477)) *
               std::sinh(_tmp481));
  _res(3, 0) =
      _tmp357 -
      _tmp493 *
          (-Scalar(0.86565325453551001) * _tmp501 + Scalar(1.0) * _tmp504 * std::sinh(_tmp497) -
           (-Scalar(0.1034955) * _tmp494 * _tmp501 +
            _tmp495 * (-_tmp492 * _tmp499 - _tmp493 * _tmp504 -
                       Scalar(1) / Scalar(2) *
                           (2 * _tmp482 * (_tmp290 + _tmp61) + 2 * _tmp483 * (_tmp285 + _tmp87)) /
                           _tmp484)) *
               std::sinh(_tmp496)) -
      _tmp499 * (Scalar(0.86565325453551001) * _tmp491 - std::cosh(_tmp496) + std::cosh(_tmp497));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
