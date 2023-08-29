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
 * Symbolic function: IK_residual_func_cost2_wrt_rx_Nl5
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost2WrtRxNl5(
    const Scalar fh1, const Scalar fv1, const Scalar rx, const Scalar ry, const Scalar rz,
    const Scalar p_init0, const Scalar p_init1, const Scalar p_init2, const Scalar rot_init_x,
    const Scalar rot_init_y, const Scalar rot_init_z, const Scalar rot_init_w,
    const Scalar epsilon) {
  // Total ops: 1651

  // Unused inputs
  (void)epsilon;

  // Input arrays

  // Intermediate terms (511)
  const Scalar _tmp0 = std::pow(rx, Scalar(2));
  const Scalar _tmp1 = _tmp0 + std::pow(ry, Scalar(2)) + std::pow(rz, Scalar(2));
  const Scalar _tmp2 = std::sqrt(_tmp1);
  const Scalar _tmp3 = (Scalar(1) / Scalar(2)) * _tmp2;
  const Scalar _tmp4 = std::cos(_tmp3);
  const Scalar _tmp5 = _tmp4 * rot_init_x;
  const Scalar _tmp6 = std::sin(_tmp3);
  const Scalar _tmp7 = _tmp6 / _tmp2;
  const Scalar _tmp8 = _tmp7 * rot_init_z;
  const Scalar _tmp9 = _tmp7 * rot_init_w;
  const Scalar _tmp10 = _tmp9 * rx;
  const Scalar _tmp11 = _tmp7 * rot_init_y;
  const Scalar _tmp12 = _tmp10 + _tmp11 * rz + _tmp5 - _tmp8 * ry;
  const Scalar _tmp13 = _tmp4 * rot_init_z;
  const Scalar _tmp14 = _tmp7 * rot_init_x;
  const Scalar _tmp15 = _tmp11 * rx;
  const Scalar _tmp16 = _tmp13 + _tmp14 * ry - _tmp15 + _tmp9 * rz;
  const Scalar _tmp17 = 2 * _tmp16;
  const Scalar _tmp18 = _tmp12 * _tmp17;
  const Scalar _tmp19 = _tmp4 * rot_init_y;
  const Scalar _tmp20 = _tmp8 * rx;
  const Scalar _tmp21 = -_tmp14 * rz + _tmp19 + _tmp20 + _tmp9 * ry;
  const Scalar _tmp22 = _tmp4 * rot_init_w;
  const Scalar _tmp23 = _tmp14 * rx;
  const Scalar _tmp24 = -_tmp11 * ry + _tmp22 - _tmp23 - _tmp8 * rz;
  const Scalar _tmp25 = 2 * _tmp24;
  const Scalar _tmp26 = _tmp21 * _tmp25;
  const Scalar _tmp27 = Scalar(0.20999999999999999) * _tmp18 - Scalar(0.20999999999999999) * _tmp26;
  const Scalar _tmp28 = -_tmp27;
  const Scalar _tmp29 = -2 * std::pow(_tmp21, Scalar(2));
  const Scalar _tmp30 = 1 - 2 * std::pow(_tmp12, Scalar(2));
  const Scalar _tmp31 =
      -Scalar(0.010999999999999999) * _tmp29 - Scalar(0.010999999999999999) * _tmp30;
  const Scalar _tmp32 = _tmp17 * _tmp21;
  const Scalar _tmp33 = _tmp12 * _tmp25;
  const Scalar _tmp34 = Scalar(0.20999999999999999) * _tmp32 + Scalar(0.20999999999999999) * _tmp33;
  const Scalar _tmp35 = _tmp31 - _tmp34;
  const Scalar _tmp36 = _tmp28 + _tmp35;
  const Scalar _tmp37 = -_tmp36 - p_init2 + Scalar(8.4718465799999993);
  const Scalar _tmp38 = 2 * _tmp12 * _tmp21;
  const Scalar _tmp39 = _tmp16 * _tmp25;
  const Scalar _tmp40 = Scalar(0.20999999999999999) * _tmp38 + Scalar(0.20999999999999999) * _tmp39;
  const Scalar _tmp41 = -_tmp40;
  const Scalar _tmp42 = _tmp32 - _tmp33;
  const Scalar _tmp43 = -Scalar(0.010999999999999999) * _tmp42;
  const Scalar _tmp44 = -2 * std::pow(_tmp16, Scalar(2));
  const Scalar _tmp45 = Scalar(0.20999999999999999) * _tmp30 + Scalar(0.20999999999999999) * _tmp44;
  const Scalar _tmp46 = _tmp43 - _tmp45;
  const Scalar _tmp47 = _tmp41 + _tmp46;
  const Scalar _tmp48 = _tmp47 + p_init1;
  const Scalar _tmp49 = -_tmp48 + Scalar(-8.3196563700000006);
  const Scalar _tmp50 = Scalar(0.20999999999999999) * _tmp29 +
                        Scalar(0.20999999999999999) * _tmp44 + Scalar(0.20999999999999999);
  const Scalar _tmp51 = -_tmp50;
  const Scalar _tmp52 = Scalar(0.20999999999999999) * _tmp38 - Scalar(0.20999999999999999) * _tmp39;
  const Scalar _tmp53 = _tmp18 + _tmp26;
  const Scalar _tmp54 = -Scalar(0.010999999999999999) * _tmp53;
  const Scalar _tmp55 = -_tmp52 + _tmp54;
  const Scalar _tmp56 = _tmp51 + _tmp55;
  const Scalar _tmp57 = _tmp56 + p_init0;
  const Scalar _tmp58 = -_tmp57 + Scalar(-1.9874742000000001);
  const Scalar _tmp59 = std::pow(_tmp49, Scalar(2)) + std::pow(_tmp58, Scalar(2));
  const Scalar _tmp60 = (Scalar(1) / Scalar(2)) / _tmp1;
  const Scalar _tmp61 = _tmp0 * _tmp60;
  const Scalar _tmp62 = _tmp60 * rx;
  const Scalar _tmp63 = _tmp62 * rz;
  const Scalar _tmp64 = _tmp62 * ry;
  const Scalar _tmp65 = _tmp6 / (_tmp1 * std::sqrt(_tmp1));
  const Scalar _tmp66 = _tmp0 * _tmp65;
  const Scalar _tmp67 = _tmp65 * rx;
  const Scalar _tmp68 = _tmp67 * ry;
  const Scalar _tmp69 = _tmp67 * rz;
  const Scalar _tmp70 = _tmp13 * _tmp61 - Scalar(1) / Scalar(2) * _tmp15 + _tmp22 * _tmp64 -
                        _tmp5 * _tmp63 - _tmp66 * rot_init_z - _tmp68 * rot_init_w +
                        _tmp69 * rot_init_x + _tmp8;
  const Scalar _tmp71 = _tmp21 * _tmp70;
  const Scalar _tmp72 = Scalar(0.043999999999999997) * _tmp71;
  const Scalar _tmp73 = -_tmp13 * _tmp64 + _tmp19 * _tmp63 + _tmp22 * _tmp61 -
                        Scalar(1) / Scalar(2) * _tmp23 - _tmp66 * rot_init_w + _tmp68 * rot_init_z -
                        _tmp69 * rot_init_y + _tmp9;
  const Scalar _tmp74 = _tmp12 * _tmp73;
  const Scalar _tmp75 = Scalar(0.043999999999999997) * _tmp74;
  const Scalar _tmp76 = -_tmp72 - _tmp75;
  const Scalar _tmp77 = -Scalar(1) / Scalar(2) * _tmp10 - _tmp13 * _tmp63 - _tmp14 -
                        _tmp19 * _tmp64 - _tmp5 * _tmp61 + _tmp66 * rot_init_x +
                        _tmp68 * rot_init_y + _tmp69 * rot_init_z;
  const Scalar _tmp78 = _tmp21 * _tmp77;
  const Scalar _tmp79 = Scalar(0.41999999999999998) * _tmp78;
  const Scalar _tmp80 = -_tmp11 - _tmp19 * _tmp61 - Scalar(1) / Scalar(2) * _tmp20 +
                        _tmp22 * _tmp63 + _tmp5 * _tmp64 + _tmp66 * rot_init_y -
                        _tmp68 * rot_init_x - _tmp69 * rot_init_w;
  const Scalar _tmp81 = Scalar(0.41999999999999998) * _tmp80;
  const Scalar _tmp82 = _tmp12 * _tmp81;
  const Scalar _tmp83 = Scalar(0.41999999999999998) * _tmp73;
  const Scalar _tmp84 = _tmp16 * _tmp83;
  const Scalar _tmp85 = Scalar(0.41999999999999998) * _tmp70;
  const Scalar _tmp86 = _tmp24 * _tmp85;
  const Scalar _tmp87 = -_tmp79 + _tmp82 + _tmp84 - _tmp86;
  const Scalar _tmp88 = _tmp76 + _tmp87;
  const Scalar _tmp89 = Scalar(0.41999999999999998) * _tmp77;
  const Scalar _tmp90 = _tmp12 * _tmp89;
  const Scalar _tmp91 = _tmp21 * _tmp81;
  const Scalar _tmp92 = _tmp16 * _tmp85;
  const Scalar _tmp93 = _tmp24 * _tmp83;
  const Scalar _tmp94 = _tmp90 + _tmp91 + _tmp92 + _tmp93;
  const Scalar _tmp95 = _tmp16 * _tmp80;
  const Scalar _tmp96 = Scalar(0.83999999999999997) * _tmp95;
  const Scalar _tmp97 = -_tmp96;
  const Scalar _tmp98 = Scalar(0.83999999999999997) * _tmp71;
  const Scalar _tmp99 = _tmp97 - _tmp98;
  const Scalar _tmp100 = _tmp16 * _tmp89;
  const Scalar _tmp101 = _tmp24 * _tmp81;
  const Scalar _tmp102 = -_tmp100 - _tmp101;
  const Scalar _tmp103 = _tmp12 * _tmp85;
  const Scalar _tmp104 = _tmp21 * _tmp83;
  const Scalar _tmp105 = _tmp103 + _tmp104;
  const Scalar _tmp106 = _tmp102 + _tmp105;
  const Scalar _tmp107 = _tmp106 + _tmp99;
  const Scalar _tmp108 = Scalar(0.021999999999999999) * _tmp78;
  const Scalar _tmp109 = Scalar(0.021999999999999999) * _tmp80;
  const Scalar _tmp110 = _tmp109 * _tmp12;
  const Scalar _tmp111 = Scalar(0.021999999999999999) * _tmp16;
  const Scalar _tmp112 = _tmp111 * _tmp73;
  const Scalar _tmp113 = Scalar(0.021999999999999999) * _tmp24;
  const Scalar _tmp114 = _tmp113 * _tmp70;
  const Scalar _tmp115 = _tmp108 + _tmp110 + _tmp112 + _tmp114;
  const Scalar _tmp116 = _tmp12 * _tmp77;
  const Scalar _tmp117 = Scalar(0.021999999999999999) * _tmp116;
  const Scalar _tmp118 = _tmp109 * _tmp21;
  const Scalar _tmp119 = _tmp111 * _tmp70;
  const Scalar _tmp120 = _tmp113 * _tmp73;
  const Scalar _tmp121 = -_tmp117 + _tmp118 + _tmp119 - _tmp120;
  const Scalar _tmp122 = Scalar(0.83999999999999997) * _tmp74;
  const Scalar _tmp123 = -_tmp122 + _tmp97;
  const Scalar _tmp124 = _tmp100 + _tmp101;
  const Scalar _tmp125 = _tmp105 + _tmp124;
  const Scalar _tmp126 = _tmp123 + _tmp125;
  const Scalar _tmp127 = 2 * _tmp49 * (_tmp121 + _tmp126) + 2 * _tmp58 * (_tmp107 + _tmp115);
  const Scalar _tmp128 = std::sqrt(_tmp59);
  const Scalar _tmp129 = Scalar(1.0) / (fh1);
  const Scalar _tmp130 = -_tmp90 - _tmp91 - _tmp92 - _tmp93;
  const Scalar _tmp131 = _tmp130 + _tmp88;
  const Scalar _tmp132 = _tmp31 + _tmp34;
  const Scalar _tmp133 = _tmp132 + _tmp28;
  const Scalar _tmp134 = -_tmp133 - p_init2 + Scalar(8.3700199099999999);
  const Scalar _tmp135 = -_tmp103 - _tmp104;
  const Scalar _tmp136 = _tmp124 + _tmp135;
  const Scalar _tmp137 = _tmp136 + _tmp99;
  const Scalar _tmp138 = _tmp52 + _tmp54;
  const Scalar _tmp139 = _tmp138 + _tmp51;
  const Scalar _tmp140 = _tmp139 + p_init0;
  const Scalar _tmp141 = -_tmp140 + Scalar(-1.79662371);
  const Scalar _tmp142 = _tmp43 + _tmp45;
  const Scalar _tmp143 = _tmp142 + _tmp41;
  const Scalar _tmp144 = _tmp143 + p_init1;
  const Scalar _tmp145 = Scalar(4.8333311099999996) - _tmp144;
  const Scalar _tmp146 = _tmp122 + _tmp96;
  const Scalar _tmp147 = _tmp125 + _tmp146;
  const Scalar _tmp148 = _tmp121 + _tmp147;
  const Scalar _tmp149 = 2 * _tmp141 * (_tmp115 + _tmp137) + 2 * _tmp145 * _tmp148;
  const Scalar _tmp150 = std::pow(_tmp141, Scalar(2)) + std::pow(_tmp145, Scalar(2));
  const Scalar _tmp151 = _tmp142 + _tmp40;
  const Scalar _tmp152 = _tmp151 + p_init1;
  const Scalar _tmp153 = _tmp152 + Scalar(-4.7752063900000001);
  const Scalar _tmp154 = _tmp138 + _tmp50;
  const Scalar _tmp155 = _tmp154 + p_init0;
  const Scalar _tmp156 = _tmp155 + Scalar(-2.71799795);
  const Scalar _tmp157 = std::pow(_tmp153, Scalar(2)) + std::pow(_tmp156, Scalar(2));
  const Scalar _tmp158 = std::pow(_tmp157, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp159 = _tmp156 * _tmp158;
  const Scalar _tmp160 = _tmp144 + Scalar(-4.8333311099999996);
  const Scalar _tmp161 = _tmp140 + Scalar(1.79662371);
  const Scalar _tmp162 = Scalar(1.0) / (_tmp161);
  const Scalar _tmp163 = _tmp160 * _tmp162;
  const Scalar _tmp164 = _tmp153 * _tmp158;
  const Scalar _tmp165 = _tmp159 * _tmp163 - _tmp164;
  const Scalar _tmp166 = Scalar(1.0) / (_tmp165);
  const Scalar _tmp167 = _tmp50 + _tmp55;
  const Scalar _tmp168 = _tmp167 + p_init0;
  const Scalar _tmp169 = _tmp168 + Scalar(-2.5202214700000001);
  const Scalar _tmp170 = _tmp40 + _tmp46;
  const Scalar _tmp171 = _tmp170 + p_init1;
  const Scalar _tmp172 = _tmp171 + Scalar(8.3888750099999996);
  const Scalar _tmp173 = std::pow(_tmp169, Scalar(2)) + std::pow(_tmp172, Scalar(2));
  const Scalar _tmp174 = std::pow(_tmp173, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp175 = _tmp172 * _tmp174;
  const Scalar _tmp176 = _tmp169 * _tmp174;
  const Scalar _tmp177 = _tmp163 * _tmp176 - _tmp175;
  const Scalar _tmp178 = _tmp166 * _tmp177;
  const Scalar _tmp179 = std::pow(_tmp161, Scalar(2));
  const Scalar _tmp180 = std::pow(_tmp160, Scalar(2)) + _tmp179;
  const Scalar _tmp181 = std::sqrt(_tmp180);
  const Scalar _tmp182 = Scalar(1.0) / (_tmp181);
  const Scalar _tmp183 = _tmp139 * _tmp182;
  const Scalar _tmp184 = _tmp143 * _tmp182;
  const Scalar _tmp185 = _tmp160 * _tmp183 - _tmp161 * _tmp184;
  const Scalar _tmp186 = _tmp162 * _tmp181;
  const Scalar _tmp187 = _tmp185 * _tmp186;
  const Scalar _tmp188 = _tmp151 * _tmp159 - _tmp154 * _tmp164 + _tmp159 * _tmp187;
  const Scalar _tmp189 =
      -_tmp167 * _tmp175 + _tmp170 * _tmp176 + _tmp176 * _tmp187 - _tmp178 * _tmp188;
  const Scalar _tmp190 = Scalar(1.0) / (_tmp189);
  const Scalar _tmp191 = Scalar(1.0) * _tmp190;
  const Scalar _tmp192 = _tmp178 * _tmp191;
  const Scalar _tmp193 = -_tmp159 * _tmp192 + _tmp176 * _tmp191;
  const Scalar _tmp194 = _tmp57 + Scalar(1.9874742000000001);
  const Scalar _tmp195 = _tmp48 + Scalar(8.3196563700000006);
  const Scalar _tmp196 = std::pow(_tmp194, Scalar(2)) + std::pow(_tmp195, Scalar(2));
  const Scalar _tmp197 = std::pow(_tmp196, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp198 = _tmp194 * _tmp197;
  const Scalar _tmp199 = _tmp195 * _tmp197;
  const Scalar _tmp200 = fh1 * (-_tmp198 * _tmp47 + _tmp199 * _tmp56);
  const Scalar _tmp201 = _tmp186 * _tmp200;
  const Scalar _tmp202 = _tmp27 + _tmp35;
  const Scalar _tmp203 = _tmp133 * _tmp176;
  const Scalar _tmp204 = _tmp133 * _tmp159;
  const Scalar _tmp205 = _tmp132 + _tmp27;
  const Scalar _tmp206 = -_tmp163 * _tmp204 + _tmp164 * _tmp205;
  const Scalar _tmp207 = -_tmp163 * _tmp203 + _tmp175 * _tmp202 - _tmp178 * _tmp206;
  const Scalar _tmp208 = Scalar(1.0) * _tmp143;
  const Scalar _tmp209 = -_tmp208;
  const Scalar _tmp210 = _tmp151 + _tmp209;
  const Scalar _tmp211 = Scalar(1.0) / (_tmp210);
  const Scalar _tmp212 = Scalar(1.0) * _tmp139;
  const Scalar _tmp213 = -_tmp154 + _tmp212;
  const Scalar _tmp214 = _tmp211 * _tmp213;
  const Scalar _tmp215 = -_tmp159 * _tmp205 + _tmp204;
  const Scalar _tmp216 = -_tmp176 * _tmp202 - _tmp178 * _tmp215 + _tmp203 - _tmp207 * _tmp214;
  const Scalar _tmp217 = Scalar(1.0) / (_tmp216);
  const Scalar _tmp218 = _tmp208 * _tmp214 + _tmp212;
  const Scalar _tmp219 = 0;
  const Scalar _tmp220 = _tmp217 * _tmp219;
  const Scalar _tmp221 = _tmp178 * _tmp220;
  const Scalar _tmp222 = -_tmp159 * _tmp221 + _tmp176 * _tmp220;
  const Scalar _tmp223 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp224 = _tmp186 * _tmp223;
  const Scalar _tmp225 = _tmp199 * fh1;
  const Scalar _tmp226 = Scalar(1.0) * _tmp166;
  const Scalar _tmp227 = Scalar(1.0) * _tmp211;
  const Scalar _tmp228 = _tmp213 * _tmp227;
  const Scalar _tmp229 = _tmp166 * _tmp228;
  const Scalar _tmp230 = _tmp206 * _tmp229 - _tmp215 * _tmp226;
  const Scalar _tmp231 = _tmp189 * _tmp217;
  const Scalar _tmp232 = -_tmp188 * _tmp226 - _tmp230 * _tmp231;
  const Scalar _tmp233 = _tmp190 * _tmp216;
  const Scalar _tmp234 = _tmp232 * _tmp233;
  const Scalar _tmp235 = _tmp230 + _tmp234;
  const Scalar _tmp236 = _tmp217 * _tmp235;
  const Scalar _tmp237 = -_tmp177 * _tmp236 + Scalar(1.0);
  const Scalar _tmp238 = _tmp159 * _tmp166;
  const Scalar _tmp239 = _tmp176 * _tmp236 + _tmp237 * _tmp238;
  const Scalar _tmp240 = _tmp186 * _tmp239;
  const Scalar _tmp241 = _tmp198 * fh1;
  const Scalar _tmp242 = _tmp163 * _tmp166;
  const Scalar _tmp243 = _tmp133 * _tmp163;
  const Scalar _tmp244 = _tmp206 * _tmp242 + _tmp243;
  const Scalar _tmp245 = -_tmp133 - _tmp214 * _tmp244 + _tmp215 * _tmp242;
  const Scalar _tmp246 = _tmp217 * _tmp245;
  const Scalar _tmp247 = -_tmp187 + _tmp188 * _tmp242 - _tmp189 * _tmp246;
  const Scalar _tmp248 = _tmp233 * _tmp247;
  const Scalar _tmp249 = _tmp245 + _tmp248;
  const Scalar _tmp250 = _tmp217 * _tmp249;
  const Scalar _tmp251 = _tmp177 * _tmp217;
  const Scalar _tmp252 = -_tmp163 - _tmp249 * _tmp251;
  const Scalar _tmp253 = _tmp176 * _tmp250 + _tmp238 * _tmp252 + Scalar(1.0);
  const Scalar _tmp254 = _tmp186 * _tmp253;
  const Scalar _tmp255 =
      -_tmp193 * _tmp201 - _tmp222 * _tmp224 - _tmp225 * _tmp240 - _tmp241 * _tmp254;
  const Scalar _tmp256 = Scalar(1.0) / (_tmp255);
  const Scalar _tmp257 = _tmp170 + _tmp209;
  const Scalar _tmp258 = _tmp214 * _tmp257;
  const Scalar _tmp259 = -_tmp167 + _tmp212 - _tmp258;
  const Scalar _tmp260 = Scalar(1.0) / (_tmp259);
  const Scalar _tmp261 = Scalar(1.0) * _tmp260;
  const Scalar _tmp262 = _tmp257 * _tmp260;
  const Scalar _tmp263 = _tmp206 * _tmp226;
  const Scalar _tmp264 = -_tmp207 * _tmp236 + _tmp234 * _tmp262 - _tmp263;
  const Scalar _tmp265 = -_tmp227 * _tmp264 + _tmp234 * _tmp261;
  const Scalar _tmp266 = Scalar(1.0) * _tmp225;
  const Scalar _tmp267 = _tmp233 * _tmp261;
  const Scalar _tmp268 = -_tmp191 * _tmp207 + _tmp257 * _tmp267;
  const Scalar _tmp269 = -Scalar(1.0) * _tmp227 * _tmp268 + Scalar(1.0) * _tmp267;
  const Scalar _tmp270 = -_tmp207 * _tmp250 + _tmp244 + _tmp248 * _tmp262;
  const Scalar _tmp271 = -_tmp227 * _tmp270 + _tmp248 * _tmp261;
  const Scalar _tmp272 = Scalar(1.0) * _tmp271;
  const Scalar _tmp273 = _tmp211 * _tmp261;
  const Scalar _tmp274 = _tmp257 * _tmp273 - _tmp261;
  const Scalar _tmp275 = _tmp241 * _tmp36 + Scalar(3.29616) * _tmp53 + _tmp56 * fv1;
  const Scalar _tmp276 = Scalar(1.0) * _tmp275;
  const Scalar _tmp277 = _tmp218 * _tmp260;
  const Scalar _tmp278 = -_tmp207 * _tmp220 + _tmp209 - _tmp257 * _tmp277;
  const Scalar _tmp279 = Scalar(1.0) * _tmp223;
  const Scalar _tmp280 = _tmp258 * _tmp261 + Scalar(1.0);
  const Scalar _tmp281 = _tmp214 * _tmp261;
  const Scalar _tmp282 = -_tmp227 * _tmp280 + _tmp281;
  const Scalar _tmp283 = _tmp36 * fh1;
  const Scalar _tmp284 = -_tmp199 * _tmp283 - Scalar(3.29616) * _tmp42 - _tmp47 * fv1;
  const Scalar _tmp285 = Scalar(1.0) * _tmp284;
  const Scalar _tmp286 =
      _tmp200 * _tmp269 + _tmp241 * _tmp272 + _tmp265 * _tmp266 + _tmp274 * _tmp276 +
      _tmp279 * (-_tmp218 * _tmp261 - _tmp227 * _tmp278 + Scalar(1.0)) + _tmp282 * _tmp285;
  const Scalar _tmp287 = std::asinh(_tmp256 * _tmp286);
  const Scalar _tmp288 = Scalar(1.0) * _tmp287;
  const Scalar _tmp289 = std::pow(_tmp255, Scalar(-2));
  const Scalar _tmp290 = -_tmp108 - _tmp110 - _tmp112 - _tmp114;
  const Scalar _tmp291 = _tmp96 + _tmp98;
  const Scalar _tmp292 = _tmp290 + _tmp291;
  const Scalar _tmp293 = _tmp106 + _tmp292;
  const Scalar _tmp294 = Scalar(1.0) / (_tmp179);
  const Scalar _tmp295 = _tmp181 * _tmp293 * _tmp294;
  const Scalar _tmp296 = _tmp241 * _tmp253;
  const Scalar _tmp297 = _tmp117 - _tmp118 - _tmp119 + _tmp120;
  const Scalar _tmp298 = _tmp102 + _tmp135;
  const Scalar _tmp299 = _tmp123 + _tmp298;
  const Scalar _tmp300 = _tmp297 + _tmp299;
  const Scalar _tmp301 = _tmp160 * _tmp300 + _tmp161 * _tmp293;
  const Scalar _tmp302 = _tmp162 * _tmp182 * _tmp301;
  const Scalar _tmp303 = _tmp193 * _tmp200;
  const Scalar _tmp304 = _tmp225 * _tmp239;
  const Scalar _tmp305 = _tmp222 * _tmp223;
  const Scalar _tmp306 = _tmp137 + _tmp290;
  const Scalar _tmp307 = _tmp147 + _tmp297;
  const Scalar _tmp308 =
      (2 * _tmp169 * _tmp306 + 2 * _tmp172 * _tmp307) / (_tmp173 * std::sqrt(_tmp173));
  const Scalar _tmp309 = (Scalar(1) / Scalar(2)) * _tmp308;
  const Scalar _tmp310 = _tmp169 * _tmp309;
  const Scalar _tmp311 = _tmp172 * _tmp309;
  const Scalar _tmp312 = _tmp126 + _tmp297;
  const Scalar _tmp313 = _tmp158 * _tmp312;
  const Scalar _tmp314 = _tmp107 + _tmp290;
  const Scalar _tmp315 = _tmp158 * _tmp314;
  const Scalar _tmp316 = _tmp160 * _tmp293;
  const Scalar _tmp317 = _tmp294 * _tmp316;
  const Scalar _tmp318 =
      (2 * _tmp153 * _tmp312 + 2 * _tmp156 * _tmp314) / (_tmp157 * std::sqrt(_tmp157));
  const Scalar _tmp319 = (Scalar(1) / Scalar(2)) * _tmp318;
  const Scalar _tmp320 = _tmp153 * _tmp319;
  const Scalar _tmp321 = _tmp156 * _tmp319;
  const Scalar _tmp322 = _tmp162 * _tmp300;
  const Scalar _tmp323 = (-_tmp159 * _tmp317 + _tmp159 * _tmp322 + _tmp163 * _tmp315 -
                          _tmp163 * _tmp321 - _tmp313 + _tmp320) /
                         std::pow(_tmp165, Scalar(2));
  const Scalar _tmp324 = _tmp177 * _tmp323;
  const Scalar _tmp325 = _tmp174 * _tmp307;
  const Scalar _tmp326 = _tmp174 * _tmp306;
  const Scalar _tmp327 = -_tmp163 * _tmp310 + _tmp163 * _tmp326 - _tmp176 * _tmp317 +
                         _tmp176 * _tmp322 + _tmp311 - _tmp325;
  const Scalar _tmp328 = _tmp166 * _tmp327;
  const Scalar _tmp329 = _tmp301 / (_tmp180 * std::sqrt(_tmp180));
  const Scalar _tmp330 = _tmp186 * (-_tmp139 * _tmp160 * _tmp329 + _tmp143 * _tmp161 * _tmp329 -
                                    _tmp161 * _tmp182 * _tmp300 + _tmp182 * _tmp316 +
                                    _tmp183 * _tmp300 - _tmp184 * _tmp293);
  const Scalar _tmp331 = _tmp185 * _tmp295;
  const Scalar _tmp332 = _tmp185 * _tmp302;
  const Scalar _tmp333 = _tmp151 * _tmp315 - _tmp151 * _tmp321 - _tmp154 * _tmp313 +
                         _tmp154 * _tmp320 + _tmp156 * _tmp313 + _tmp159 * _tmp330 -
                         _tmp159 * _tmp331 + _tmp159 * _tmp332 - _tmp164 * _tmp314 +
                         _tmp187 * _tmp315 - _tmp187 * _tmp321;
  const Scalar _tmp334 = _tmp167 * _tmp311 - _tmp167 * _tmp325 + _tmp169 * _tmp325 -
                         _tmp170 * _tmp310 + _tmp170 * _tmp326 - _tmp175 * _tmp306 +
                         _tmp176 * _tmp330 - _tmp176 * _tmp331 + _tmp176 * _tmp332 -
                         _tmp178 * _tmp333 - _tmp187 * _tmp310 + _tmp187 * _tmp326 +
                         _tmp188 * _tmp324 - _tmp188 * _tmp328;
  const Scalar _tmp335 = _tmp334 / std::pow(_tmp189, Scalar(2));
  const Scalar _tmp336 = _tmp216 * _tmp335;
  const Scalar _tmp337 = _tmp232 * _tmp336;
  const Scalar _tmp338 = Scalar(1.0) * _tmp323;
  const Scalar _tmp339 = _tmp72 + _tmp75;
  const Scalar _tmp340 = _tmp339 + _tmp87;
  const Scalar _tmp341 = _tmp130 + _tmp340;
  const Scalar _tmp342 = _tmp79 - _tmp82 - _tmp84 + _tmp86;
  const Scalar _tmp343 = _tmp342 + _tmp94;
  const Scalar _tmp344 = _tmp339 + _tmp343;
  const Scalar _tmp345 = _tmp133 * _tmp326;
  const Scalar _tmp346 = _tmp340 + _tmp94;
  const Scalar _tmp347 = _tmp163 * _tmp344;
  const Scalar _tmp348 = -_tmp159 * _tmp347 + _tmp164 * _tmp346 + _tmp204 * _tmp317 -
                         _tmp204 * _tmp322 + _tmp205 * _tmp313 - _tmp205 * _tmp320 -
                         _tmp243 * _tmp315 + _tmp243 * _tmp321;
  const Scalar _tmp349 = -_tmp163 * _tmp345 + _tmp175 * _tmp341 - _tmp176 * _tmp347 -
                         _tmp178 * _tmp348 - _tmp202 * _tmp311 + _tmp202 * _tmp325 +
                         _tmp203 * _tmp317 - _tmp203 * _tmp322 + _tmp206 * _tmp324 -
                         _tmp206 * _tmp328 + _tmp243 * _tmp310;
  const Scalar _tmp350 = Scalar(1.6799999999999999) * _tmp95;
  const Scalar _tmp351 = _tmp350 + Scalar(1.6799999999999999) * _tmp71;
  const Scalar _tmp352 = _tmp211 * _tmp351;
  const Scalar _tmp353 = Scalar(0.83999999999999997) * _tmp16 * _tmp77;
  const Scalar _tmp354 = Scalar(0.83999999999999997) * _tmp24 * _tmp80;
  const Scalar _tmp355 =
      Scalar(0.83999999999999997) * _tmp12 * _tmp70 + Scalar(0.83999999999999997) * _tmp21 * _tmp73;
  const Scalar _tmp356 = _tmp353 + _tmp354 + _tmp355;
  const Scalar _tmp357 = _tmp356 / std::pow(_tmp210, Scalar(2));
  const Scalar _tmp358 = _tmp213 * _tmp357;
  const Scalar _tmp359 = _tmp133 * _tmp315 - _tmp133 * _tmp321 + _tmp159 * _tmp344 -
                         _tmp159 * _tmp346 - _tmp205 * _tmp315 + _tmp205 * _tmp321;
  const Scalar _tmp360 = -_tmp133 * _tmp310 - _tmp176 * _tmp341 + _tmp176 * _tmp344 -
                         _tmp178 * _tmp359 + _tmp202 * _tmp310 - _tmp202 * _tmp326 -
                         _tmp207 * _tmp352 + _tmp207 * _tmp358 - _tmp214 * _tmp349 +
                         _tmp215 * _tmp324 - _tmp215 * _tmp328 + _tmp345;
  const Scalar _tmp361 = _tmp360 / std::pow(_tmp216, Scalar(2));
  const Scalar _tmp362 = _tmp189 * _tmp361;
  const Scalar _tmp363 = -_tmp206 * _tmp228 * _tmp323 + _tmp215 * _tmp338 - _tmp226 * _tmp359 +
                         _tmp229 * _tmp348 + _tmp263 * _tmp352 - _tmp263 * _tmp358;
  const Scalar _tmp364 = _tmp233 * (_tmp188 * _tmp338 - _tmp217 * _tmp230 * _tmp334 -
                                    _tmp226 * _tmp333 + _tmp230 * _tmp362 - _tmp231 * _tmp363);
  const Scalar _tmp365 = _tmp190 * _tmp360;
  const Scalar _tmp366 = _tmp232 * _tmp365;
  const Scalar _tmp367 = -_tmp337 + _tmp363 + _tmp364 + _tmp366;
  const Scalar _tmp368 = _tmp176 * _tmp217;
  const Scalar _tmp369 = _tmp235 * _tmp361;
  const Scalar _tmp370 = _tmp166 * _tmp237;
  const Scalar _tmp371 = _tmp166 * _tmp315;
  const Scalar _tmp372 = _tmp159 * _tmp323;
  const Scalar _tmp373 = _tmp177 * _tmp369 - _tmp236 * _tmp327 - _tmp251 * _tmp367;
  const Scalar _tmp374 = _tmp249 * _tmp361;
  const Scalar _tmp375 = _tmp166 * _tmp252;
  const Scalar _tmp376 = _tmp247 * _tmp365;
  const Scalar _tmp377 = _tmp166 * _tmp317;
  const Scalar _tmp378 = _tmp163 * _tmp323;
  const Scalar _tmp379 = _tmp166 * _tmp322;
  const Scalar _tmp380 = -_tmp133 * _tmp317 + _tmp133 * _tmp322 - _tmp206 * _tmp377 -
                         _tmp206 * _tmp378 + _tmp206 * _tmp379 + _tmp242 * _tmp348 + _tmp347;
  const Scalar _tmp381 = _tmp131 - _tmp214 * _tmp380 - _tmp215 * _tmp377 - _tmp215 * _tmp378 +
                         _tmp215 * _tmp379 + _tmp242 * _tmp359 - _tmp244 * _tmp352 +
                         _tmp244 * _tmp358;
  const Scalar _tmp382 = _tmp233 * (-_tmp188 * _tmp377 - _tmp188 * _tmp378 + _tmp188 * _tmp379 -
                                    _tmp231 * _tmp381 + _tmp242 * _tmp333 + _tmp245 * _tmp362 -
                                    _tmp246 * _tmp334 - _tmp330 + _tmp331 - _tmp332);
  const Scalar _tmp383 = _tmp247 * _tmp336;
  const Scalar _tmp384 = _tmp376 + _tmp381 + _tmp382 - _tmp383;
  const Scalar _tmp385 =
      _tmp177 * _tmp374 - _tmp250 * _tmp327 - _tmp251 * _tmp384 + _tmp317 - _tmp322;
  const Scalar _tmp386 = _tmp146 + _tmp298;
  const Scalar _tmp387 = _tmp297 + _tmp386;
  const Scalar _tmp388 = _tmp197 * _tmp387;
  const Scalar _tmp389 = _tmp388 * fh1;
  const Scalar _tmp390 = _tmp159 * _tmp324;
  const Scalar _tmp391 = _tmp159 * _tmp328;
  const Scalar _tmp392 = _tmp219 * _tmp361;
  const Scalar _tmp393 = _tmp159 * _tmp178;
  const Scalar _tmp394 = Scalar(1.0) * _tmp335;
  const Scalar _tmp395 = Scalar(0.5) * _tmp190;
  const Scalar _tmp396 = _tmp136 + _tmp292;
  const Scalar _tmp397 = _tmp197 * _tmp396;
  const Scalar _tmp398 = _tmp397 * fh1;
  const Scalar _tmp399 =
      (2 * _tmp194 * _tmp396 + 2 * _tmp195 * _tmp387) / (_tmp196 * std::sqrt(_tmp196));
  const Scalar _tmp400 = (Scalar(1) / Scalar(2)) * _tmp399;
  const Scalar _tmp401 = _tmp195 * _tmp400;
  const Scalar _tmp402 = _tmp194 * _tmp400;
  const Scalar _tmp403 = fh1 * (-_tmp194 * _tmp388 + _tmp195 * _tmp397 + _tmp388 * _tmp56 -
                                _tmp397 * _tmp47 - _tmp401 * _tmp56 + _tmp402 * _tmp47);
  const Scalar _tmp404 = _tmp401 * fh1;
  const Scalar _tmp405 = _tmp402 * fh1;
  const Scalar _tmp406 =
      -_tmp186 * _tmp193 * _tmp403 -
      _tmp186 * _tmp225 *
          (-_tmp176 * _tmp369 - _tmp236 * _tmp310 + _tmp236 * _tmp326 + _tmp237 * _tmp371 -
           _tmp237 * _tmp372 + _tmp238 * _tmp373 - _tmp321 * _tmp370 + _tmp367 * _tmp368) -
      _tmp186 * _tmp241 *
          (-_tmp176 * _tmp374 + _tmp238 * _tmp385 - _tmp250 * _tmp310 + _tmp250 * _tmp326 +
           _tmp252 * _tmp371 - _tmp252 * _tmp372 - _tmp321 * _tmp375 + _tmp368 * _tmp384) -
      _tmp201 * (_tmp156 * _tmp178 * _tmp318 * _tmp395 - _tmp169 * _tmp308 * _tmp395 -
                 _tmp176 * _tmp394 + _tmp191 * _tmp326 + _tmp191 * _tmp390 - _tmp191 * _tmp391 -
                 _tmp192 * _tmp315 + _tmp393 * _tmp394) -
      _tmp224 * (-_tmp176 * _tmp392 - _tmp220 * _tmp310 + _tmp220 * _tmp326 + _tmp220 * _tmp390 -
                 _tmp220 * _tmp391 - _tmp221 * _tmp315 + _tmp221 * _tmp321 + _tmp392 * _tmp393) -
      _tmp240 * _tmp389 + _tmp240 * _tmp404 - _tmp254 * _tmp398 + _tmp254 * _tmp405 +
      _tmp295 * _tmp296 + _tmp295 * _tmp303 + _tmp295 * _tmp304 + _tmp295 * _tmp305 -
      _tmp296 * _tmp302 - _tmp302 * _tmp303 - _tmp302 * _tmp304 - _tmp302 * _tmp305;
  const Scalar _tmp407 = _tmp289 * _tmp406;
  const Scalar _tmp408 = Scalar(0.5) * _tmp399 * fh1;
  const Scalar _tmp409 = Scalar(6.59232) * _tmp80;
  const Scalar _tmp410 = Scalar(6.59232) * _tmp16;
  const Scalar _tmp411 = _tmp130 + _tmp342;
  const Scalar _tmp412 = _tmp339 + _tmp411;
  const Scalar _tmp413 = Scalar(6.59232) * _tmp24;
  const Scalar _tmp414 = _tmp12 * _tmp409 + _tmp241 * _tmp412 + _tmp283 * _tmp397 -
                         _tmp283 * _tmp402 + _tmp396 * fv1 + _tmp410 * _tmp73 + _tmp413 * _tmp70 +
                         Scalar(6.59232) * _tmp78;
  const Scalar _tmp415 = _tmp208 * _tmp352 - _tmp208 * _tmp358 + _tmp228 * _tmp300 + _tmp293;
  const Scalar _tmp416 = _tmp260 * _tmp415;
  const Scalar _tmp417 = _tmp350 + _tmp356 + Scalar(1.6799999999999999) * _tmp74;
  const Scalar _tmp418 = _tmp257 * _tmp358;
  const Scalar _tmp419 = _tmp214 * _tmp417;
  const Scalar _tmp420 = _tmp257 * _tmp352;
  const Scalar _tmp421 = (_tmp351 - _tmp353 - _tmp354 + _tmp355 + _tmp418 - _tmp419 - _tmp420) /
                         std::pow(_tmp259, Scalar(2));
  const Scalar _tmp422 = _tmp218 * _tmp421;
  const Scalar _tmp423 = _tmp148 + _tmp207 * _tmp392 - _tmp220 * _tmp349 - _tmp257 * _tmp416 +
                         _tmp257 * _tmp422 - _tmp277 * _tmp417;
  const Scalar _tmp424 = Scalar(1.0) * _tmp421;
  const Scalar _tmp425 = _tmp278 * _tmp357;
  const Scalar _tmp426 = _tmp207 * _tmp217;
  const Scalar _tmp427 = _tmp260 * _tmp417;
  const Scalar _tmp428 = _tmp257 * _tmp421;
  const Scalar _tmp429 = _tmp206 * _tmp338 + _tmp207 * _tmp369 - _tmp226 * _tmp348 +
                         _tmp234 * _tmp427 - _tmp234 * _tmp428 - _tmp236 * _tmp349 -
                         _tmp262 * _tmp337 + _tmp262 * _tmp364 + _tmp262 * _tmp366 -
                         _tmp367 * _tmp426;
  const Scalar _tmp430 = Scalar(1.0) * _tmp357;
  const Scalar _tmp431 = Scalar(6.59232) * _tmp116 - _tmp21 * _tmp409 - _tmp225 * _tmp412 -
                         _tmp283 * _tmp388 + _tmp283 * _tmp401 - _tmp387 * fv1 - _tmp410 * _tmp70 +
                         _tmp413 * _tmp73;
  const Scalar _tmp432 = _tmp207 * _tmp374 + _tmp248 * _tmp427 - _tmp248 * _tmp428 -
                         _tmp250 * _tmp349 + _tmp262 * _tmp376 + _tmp262 * _tmp382 -
                         _tmp262 * _tmp383 + _tmp380 - _tmp384 * _tmp426;
  const Scalar _tmp433 = _tmp261 * _tmp336;
  const Scalar _tmp434 = _tmp233 * _tmp424;
  const Scalar _tmp435 = _tmp261 * _tmp365;
  const Scalar _tmp436 = -_tmp191 * _tmp349 + _tmp207 * _tmp394 - _tmp257 * _tmp433 -
                         _tmp257 * _tmp434 + _tmp257 * _tmp435 + _tmp267 * _tmp417;
  const Scalar _tmp437 = _tmp227 * _tmp428;
  const Scalar _tmp438 = _tmp257 * _tmp357;
  const Scalar _tmp439 = _tmp261 * _tmp352;
  const Scalar _tmp440 = _tmp261 * _tmp358;
  const Scalar _tmp441 = _tmp228 * _tmp421;
  const Scalar _tmp442 =
      -_tmp257 * _tmp441 - _tmp261 * _tmp418 + _tmp261 * _tmp419 + _tmp261 * _tmp420;
  const Scalar _tmp443 =
      (_tmp256 *
           (-_tmp194 * _tmp271 * _tmp408 - _tmp195 * _tmp265 * _tmp408 +
            Scalar(1.0) * _tmp200 *
                (-_tmp227 * _tmp436 + _tmp268 * _tmp430 - _tmp433 - _tmp434 + _tmp435) +
            Scalar(1.0) * _tmp241 *
                (-_tmp227 * _tmp432 - _tmp248 * _tmp424 + _tmp261 * _tmp376 + _tmp261 * _tmp382 -
                 _tmp261 * _tmp383 + _tmp270 * _tmp430) +
            Scalar(1.0) * _tmp265 * _tmp389 +
            _tmp266 * (-_tmp227 * _tmp429 - _tmp234 * _tmp424 - _tmp261 * _tmp337 +
                       _tmp261 * _tmp364 + _tmp261 * _tmp366 + _tmp264 * _tmp430) +
            _tmp269 * _tmp403 + _tmp272 * _tmp398 + Scalar(1.0) * _tmp274 * _tmp414 +
            _tmp276 * (-_tmp261 * _tmp438 + _tmp273 * _tmp417 + _tmp424 - _tmp437) +
            _tmp279 * (_tmp218 * _tmp424 - _tmp227 * _tmp423 - _tmp261 * _tmp415 +
                       Scalar(1.0) * _tmp425) +
            Scalar(1.0) * _tmp282 * _tmp431 +
            _tmp285 * (-_tmp227 * _tmp442 + _tmp280 * _tmp430 + _tmp439 - _tmp440 - _tmp441)) -
       _tmp286 * _tmp407) /
      std::sqrt(Scalar(std::pow(_tmp286, Scalar(2)) * _tmp289 + 1));
  const Scalar _tmp444 = Scalar(9.6622558468725703) * _tmp255;
  const Scalar _tmp445 = std::sqrt(_tmp150);
  const Scalar _tmp446 = -_tmp287 * _tmp444 - _tmp445;
  const Scalar _tmp447 = Scalar(0.1034955) * _tmp256;
  const Scalar _tmp448 = _tmp446 * _tmp447;
  const Scalar _tmp449 = Scalar(9.6622558468725703) * _tmp406;
  const Scalar _tmp450 = -_tmp205 - p_init2 + Scalar(8.36416322);
  const Scalar _tmp451 = Scalar(4.7752063900000001) - _tmp152;
  const Scalar _tmp452 = Scalar(2.71799795) - _tmp155;
  const Scalar _tmp453 = std::pow(_tmp451, Scalar(2)) + std::pow(_tmp452, Scalar(2));
  const Scalar _tmp454 = _tmp115 + _tmp291;
  const Scalar _tmp455 = 2 * _tmp451 * (_tmp121 + _tmp386) + 2 * _tmp452 * (_tmp136 + _tmp454);
  const Scalar _tmp456 = _tmp220 * _tmp223;
  const Scalar _tmp457 = _tmp166 * _tmp241;
  const Scalar _tmp458 = _tmp191 * _tmp200;
  const Scalar _tmp459 = _tmp166 * _tmp225;
  const Scalar _tmp460 =
      -_tmp178 * _tmp456 - _tmp178 * _tmp458 + _tmp237 * _tmp459 + _tmp252 * _tmp457;
  const Scalar _tmp461 = Scalar(1.0) / (_tmp460);
  const Scalar _tmp462 = _tmp211 * _tmp223;
  const Scalar _tmp463 = _tmp211 * _tmp268;
  const Scalar _tmp464 = _tmp211 * _tmp270;
  const Scalar _tmp465 = _tmp211 * _tmp225;
  const Scalar _tmp466 = _tmp261 * _tmp275;
  const Scalar _tmp467 = _tmp211 * _tmp466;
  const Scalar _tmp468 = _tmp211 * _tmp280;
  const Scalar _tmp469 = _tmp200 * _tmp463 + _tmp241 * _tmp464 - _tmp257 * _tmp467 +
                         _tmp264 * _tmp465 + _tmp278 * _tmp462 + _tmp284 * _tmp468;
  const Scalar _tmp470 = std::asinh(_tmp461 * _tmp469);
  const Scalar _tmp471 = Scalar(9.6622558468725703) * _tmp460;
  const Scalar _tmp472 = std::sqrt(_tmp453);
  const Scalar _tmp473 = -_tmp470 * _tmp471 - _tmp472;
  const Scalar _tmp474 = Scalar(0.1034955) * _tmp461;
  const Scalar _tmp475 = _tmp473 * _tmp474;
  const Scalar _tmp476 = Scalar(1.0) * _tmp470;
  const Scalar _tmp477 = _tmp191 * _tmp403;
  const Scalar _tmp478 = _tmp223 * _tmp392;
  const Scalar _tmp479 = _tmp200 * _tmp394;
  const Scalar _tmp480 =
      -_tmp178 * _tmp477 + _tmp178 * _tmp478 + _tmp178 * _tmp479 - _tmp225 * _tmp237 * _tmp323 -
      _tmp241 * _tmp252 * _tmp323 + _tmp324 * _tmp456 + _tmp324 * _tmp458 - _tmp328 * _tmp456 -
      _tmp328 * _tmp458 + _tmp370 * _tmp389 - _tmp370 * _tmp404 + _tmp373 * _tmp459 +
      _tmp375 * _tmp398 - _tmp375 * _tmp405 + _tmp385 * _tmp457;
  const Scalar _tmp481 = Scalar(9.6622558468725703) * _tmp480;
  const Scalar _tmp482 = _tmp211 * _tmp264;
  const Scalar _tmp483 = _tmp261 * _tmp414;
  const Scalar _tmp484 = std::pow(_tmp460, Scalar(-2));
  const Scalar _tmp485 = _tmp480 * _tmp484;
  const Scalar _tmp486 =
      (_tmp461 * (_tmp200 * _tmp211 * _tmp436 - _tmp200 * _tmp268 * _tmp357 +
                  _tmp211 * _tmp241 * _tmp432 - _tmp211 * _tmp257 * _tmp483 +
                  _tmp211 * _tmp284 * _tmp442 - _tmp223 * _tmp425 - _tmp225 * _tmp264 * _tmp357 -
                  _tmp241 * _tmp270 * _tmp357 + _tmp275 * _tmp437 - _tmp280 * _tmp284 * _tmp357 +
                  _tmp389 * _tmp482 + _tmp398 * _tmp464 + _tmp403 * _tmp463 - _tmp404 * _tmp482 -
                  _tmp405 * _tmp464 - _tmp417 * _tmp467 + _tmp423 * _tmp462 + _tmp429 * _tmp465 +
                  _tmp431 * _tmp468 + _tmp438 * _tmp466) -
       _tmp469 * _tmp485) /
      std::sqrt(Scalar(std::pow(_tmp469, Scalar(2)) * _tmp484 + 1));
  const Scalar _tmp487 = -_tmp171 + Scalar(-8.3888750099999996);
  const Scalar _tmp488 = Scalar(2.5202214700000001) - _tmp168;
  const Scalar _tmp489 = std::pow(_tmp487, Scalar(2)) + std::pow(_tmp488, Scalar(2));
  const Scalar _tmp490 = std::sqrt(_tmp489);
  const Scalar _tmp491 = _tmp225 * _tmp236 + _tmp241 * _tmp250 + _tmp456 + _tmp458;
  const Scalar _tmp492 = Scalar(1.0) / (_tmp491);
  const Scalar _tmp493 = _tmp241 * _tmp260;
  const Scalar _tmp494 = _tmp234 * _tmp260;
  const Scalar _tmp495 = -_tmp200 * _tmp267 + _tmp223 * _tmp277 - _tmp225 * _tmp494 -
                         _tmp248 * _tmp493 - _tmp281 * _tmp284 + _tmp466;
  const Scalar _tmp496 = std::asinh(_tmp492 * _tmp495);
  const Scalar _tmp497 = Scalar(9.6622558468725703) * _tmp496;
  const Scalar _tmp498 = -_tmp490 - _tmp491 * _tmp497;
  const Scalar _tmp499 = Scalar(0.1034955) * _tmp492;
  const Scalar _tmp500 = _tmp498 * _tmp499;
  const Scalar _tmp501 = Scalar(1.0) * _tmp496;
  const Scalar _tmp502 = _tmp217 * _tmp225 * _tmp367 + _tmp217 * _tmp241 * _tmp384 -
                         _tmp225 * _tmp369 + _tmp236 * _tmp389 - _tmp236 * _tmp404 -
                         _tmp241 * _tmp374 + _tmp250 * _tmp398 - _tmp250 * _tmp405 + _tmp477 -
                         _tmp478 - _tmp479;
  const Scalar _tmp503 = -_tmp202 - p_init2 + Scalar(8.4693136199999994);
  const Scalar _tmp504 = 2 * _tmp487 * (_tmp121 + _tmp299) + 2 * _tmp488 * (_tmp106 + _tmp454);
  const Scalar _tmp505 = std::pow(_tmp491, Scalar(-2));
  const Scalar _tmp506 = _tmp248 * _tmp260;
  const Scalar _tmp507 = _tmp225 * _tmp260;
  const Scalar _tmp508 = _tmp502 * _tmp505;
  const Scalar _tmp509 =
      (_tmp492 * (_tmp200 * _tmp433 + _tmp200 * _tmp434 - _tmp200 * _tmp435 + _tmp223 * _tmp416 -
                  _tmp223 * _tmp422 + _tmp225 * _tmp234 * _tmp421 + _tmp241 * _tmp248 * _tmp421 -
                  _tmp267 * _tmp403 - _tmp276 * _tmp421 - _tmp281 * _tmp431 - _tmp284 * _tmp439 +
                  _tmp284 * _tmp440 + _tmp284 * _tmp441 + _tmp337 * _tmp507 - _tmp364 * _tmp507 -
                  _tmp366 * _tmp507 - _tmp376 * _tmp493 - _tmp382 * _tmp493 + _tmp383 * _tmp493 -
                  _tmp389 * _tmp494 - _tmp398 * _tmp506 + _tmp404 * _tmp494 + _tmp405 * _tmp506 +
                  _tmp483) -
       _tmp495 * _tmp508) /
      std::sqrt(Scalar(std::pow(_tmp495, Scalar(2)) * _tmp505 + 1));
  const Scalar _tmp510 = Scalar(9.6622558468725703) * _tmp491;

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) =
      Scalar(0.5) * _tmp127 *
          std::cosh(Scalar(0.1034955) * _tmp129 *
                    (-_tmp128 - Scalar(9.6622558468725703) * fh1 * std::asinh(_tmp129 * fv1))) /
          _tmp128 -
      Scalar(1) / Scalar(2) * (_tmp127 + 2 * _tmp37 * (_tmp88 + _tmp94)) /
          std::sqrt(Scalar(std::pow(_tmp37, Scalar(2)) + _tmp59));
  _res(1, 0) = _tmp444 * (-Scalar(1.0) * _tmp443 * std::cosh(_tmp288) -
                          (-Scalar(0.1034955) * _tmp407 * _tmp446 +
                           _tmp447 * (-Scalar(1) / Scalar(2) * _tmp149 / _tmp445 -
                                      _tmp287 * _tmp449 - _tmp443 * _tmp444)) *
                              std::cosh(_tmp448)) +
               _tmp449 * (-std::sinh(_tmp288) - std::sinh(_tmp448)) -
               Scalar(1) / Scalar(2) * (2 * _tmp131 * _tmp134 + _tmp149) /
                   std::sqrt(Scalar(std::pow(_tmp134, Scalar(2)) + _tmp150));
  _res(2, 0) = _tmp471 * (-Scalar(1.0) * _tmp486 * std::cosh(_tmp476) -
                          (-Scalar(0.1034955) * _tmp473 * _tmp485 +
                           _tmp474 * (-Scalar(1) / Scalar(2) * _tmp455 / _tmp472 -
                                      _tmp470 * _tmp481 - _tmp471 * _tmp486)) *
                              std::cosh(_tmp475)) +
               _tmp481 * (-std::sinh(_tmp475) - std::sinh(_tmp476)) -
               Scalar(1) / Scalar(2) * (2 * _tmp450 * (_tmp411 + _tmp76) + _tmp455) /
                   std::sqrt(Scalar(std::pow(_tmp450, Scalar(2)) + _tmp453));
  _res(3, 0) = Scalar(9.6622558468725703) * _tmp502 * (-std::sinh(_tmp500) - std::sinh(_tmp501)) +
               _tmp510 * (-Scalar(1.0) * _tmp509 * std::cosh(_tmp501) -
                          (-Scalar(0.1034955) * _tmp498 * _tmp508 +
                           _tmp499 * (-_tmp497 * _tmp502 - _tmp509 * _tmp510 -
                                      Scalar(1) / Scalar(2) * _tmp504 / _tmp490)) *
                              std::cosh(_tmp500)) -
               Scalar(1) / Scalar(2) * (2 * _tmp503 * (_tmp343 + _tmp76) + _tmp504) /
                   std::sqrt(Scalar(_tmp489 + std::pow(_tmp503, Scalar(2))));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym