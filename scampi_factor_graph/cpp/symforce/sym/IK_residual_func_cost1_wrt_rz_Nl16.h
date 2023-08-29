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
 * Symbolic function: IK_residual_func_cost1_wrt_rz_Nl16
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost1WrtRzNl16(
    const Scalar fh1, const Scalar fv1, const Scalar rx, const Scalar ry, const Scalar rz,
    const Scalar p_init0, const Scalar p_init1, const Scalar p_init2, const Scalar rot_init_x,
    const Scalar rot_init_y, const Scalar rot_init_z, const Scalar rot_init_w,
    const Scalar epsilon) {
  // Total ops: 1629

  // Unused inputs
  (void)p_init2;
  (void)epsilon;

  // Input arrays

  // Intermediate terms (508)
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
  const Scalar _tmp27 = 2 * _tmp20;
  const Scalar _tmp28 = _tmp12 * _tmp27;
  const Scalar _tmp29 = _tmp16 * _tmp24;
  const Scalar _tmp30 = _tmp28 - _tmp29;
  const Scalar _tmp31 = -Scalar(0.010999999999999999) * _tmp30;
  const Scalar _tmp32 = -2 * std::pow(_tmp16, Scalar(2));
  const Scalar _tmp33 = -2 * std::pow(_tmp20, Scalar(2));
  const Scalar _tmp34 = Scalar(0.20999999999999999) * _tmp32 +
                        Scalar(0.20999999999999999) * _tmp33 + Scalar(0.20999999999999999);
  const Scalar _tmp35 = _tmp31 + _tmp34;
  const Scalar _tmp36 = _tmp26 + _tmp35;
  const Scalar _tmp37 = _tmp36 + p_init1;
  const Scalar _tmp38 = Scalar(4.7752063900000001) - _tmp37;
  const Scalar _tmp39 = Scalar(0.20999999999999999) * _tmp17 - Scalar(0.20999999999999999) * _tmp25;
  const Scalar _tmp40 = _tmp16 * _tmp27;
  const Scalar _tmp41 = _tmp12 * _tmp24;
  const Scalar _tmp42 = _tmp40 + _tmp41;
  const Scalar _tmp43 = -Scalar(0.010999999999999999) * _tmp42;
  const Scalar _tmp44 = 1 - 2 * std::pow(_tmp12, Scalar(2));
  const Scalar _tmp45 = Scalar(0.20999999999999999) * _tmp33 + Scalar(0.20999999999999999) * _tmp44;
  const Scalar _tmp46 = _tmp43 + _tmp45;
  const Scalar _tmp47 = _tmp39 + _tmp46;
  const Scalar _tmp48 = _tmp47 + p_init0;
  const Scalar _tmp49 = Scalar(2.71799795) - _tmp48;
  const Scalar _tmp50 =
      std::sqrt(Scalar(std::pow(_tmp38, Scalar(2)) + std::pow(_tmp49, Scalar(2))));
  const Scalar _tmp51 = Scalar(1.0) / (fh1);
  const Scalar _tmp52 = (Scalar(1) / Scalar(2)) / _tmp1;
  const Scalar _tmp53 = _tmp0 * _tmp52;
  const Scalar _tmp54 = _tmp52 * rz;
  const Scalar _tmp55 = _tmp54 * ry;
  const Scalar _tmp56 = _tmp54 * rx;
  const Scalar _tmp57 = _tmp6 / (_tmp1 * std::sqrt(_tmp1));
  const Scalar _tmp58 = _tmp0 * _tmp57;
  const Scalar _tmp59 = _tmp57 * rz;
  const Scalar _tmp60 = _tmp59 * rx;
  const Scalar _tmp61 = _tmp59 * ry;
  const Scalar _tmp62 = -Scalar(1) / Scalar(2) * _tmp11 + _tmp14 - _tmp18 * _tmp55 +
                        _tmp21 * _tmp56 + _tmp5 * _tmp53 - _tmp58 * rot_init_y -
                        _tmp60 * rot_init_w + _tmp61 * rot_init_z;
  const Scalar _tmp63 = Scalar(0.41999999999999998) * _tmp62;
  const Scalar _tmp64 = _tmp12 * _tmp63;
  const Scalar _tmp65 = -_tmp10 - _tmp13 * _tmp53 - Scalar(1) / Scalar(2) * _tmp15 +
                        _tmp18 * _tmp56 + _tmp21 * _tmp55 + _tmp58 * rot_init_x -
                        _tmp60 * rot_init_z - _tmp61 * rot_init_w;
  const Scalar _tmp66 = Scalar(0.41999999999999998) * _tmp65;
  const Scalar _tmp67 = _tmp16 * _tmp66;
  const Scalar _tmp68 = -_tmp64 - _tmp67;
  const Scalar _tmp69 = -_tmp13 * _tmp56 - _tmp18 * _tmp53 - Scalar(1) / Scalar(2) * _tmp19 -
                        _tmp5 * _tmp55 + _tmp58 * rot_init_z + _tmp60 * rot_init_x +
                        _tmp61 * rot_init_y - _tmp9;
  const Scalar _tmp70 = Scalar(0.41999999999999998) * _tmp69;
  const Scalar _tmp71 = _tmp20 * _tmp70;
  const Scalar _tmp72 = _tmp13 * _tmp55 + _tmp21 * _tmp53 - Scalar(1) / Scalar(2) * _tmp22 -
                        _tmp5 * _tmp56 - _tmp58 * rot_init_w + _tmp60 * rot_init_y -
                        _tmp61 * rot_init_x + _tmp8;
  const Scalar _tmp73 = Scalar(0.41999999999999998) * _tmp72;
  const Scalar _tmp74 = _tmp23 * _tmp73;
  const Scalar _tmp75 = _tmp71 + _tmp74;
  const Scalar _tmp76 = _tmp68 + _tmp75;
  const Scalar _tmp77 = Scalar(0.83999999999999997) * _tmp12;
  const Scalar _tmp78 = _tmp65 * _tmp77;
  const Scalar _tmp79 = Scalar(0.83999999999999997) * _tmp20;
  const Scalar _tmp80 = _tmp72 * _tmp79;
  const Scalar _tmp81 = _tmp78 + _tmp80;
  const Scalar _tmp82 = Scalar(0.021999999999999999) * _tmp12;
  const Scalar _tmp83 = _tmp69 * _tmp82;
  const Scalar _tmp84 = Scalar(0.021999999999999999) * _tmp20;
  const Scalar _tmp85 = _tmp62 * _tmp84;
  const Scalar _tmp86 = Scalar(0.021999999999999999) * _tmp16;
  const Scalar _tmp87 = _tmp72 * _tmp86;
  const Scalar _tmp88 = Scalar(0.021999999999999999) * _tmp23;
  const Scalar _tmp89 = _tmp65 * _tmp88;
  const Scalar _tmp90 = _tmp83 + _tmp85 + _tmp87 + _tmp89;
  const Scalar _tmp91 = _tmp81 + _tmp90;
  const Scalar _tmp92 = -_tmp71 - _tmp74;
  const Scalar _tmp93 = _tmp68 + _tmp92;
  const Scalar _tmp94 = _tmp69 * _tmp86;
  const Scalar _tmp95 = _tmp65 * _tmp84;
  const Scalar _tmp96 = _tmp72 * _tmp82;
  const Scalar _tmp97 = _tmp62 * _tmp88;
  const Scalar _tmp98 = -_tmp94 + _tmp95 + _tmp96 - _tmp97;
  const Scalar _tmp99 = _tmp16 * _tmp62;
  const Scalar _tmp100 = Scalar(0.83999999999999997) * _tmp99;
  const Scalar _tmp101 = _tmp100 + _tmp80;
  const Scalar _tmp102 = _tmp101 + _tmp98;
  const Scalar _tmp103 = _tmp16 * _tmp70;
  const Scalar _tmp104 = _tmp20 * _tmp66;
  const Scalar _tmp105 = _tmp12 * _tmp73;
  const Scalar _tmp106 = _tmp23 * _tmp63;
  const Scalar _tmp107 = Scalar(0.043999999999999997) * _tmp99;
  const Scalar _tmp108 = _tmp12 * _tmp65;
  const Scalar _tmp109 = Scalar(0.043999999999999997) * _tmp108;
  const Scalar _tmp110 = _tmp107 + _tmp109;
  const Scalar _tmp111 = _tmp103 + _tmp104 + _tmp105 + _tmp106 + _tmp110;
  const Scalar _tmp112 = _tmp12 * _tmp70;
  const Scalar _tmp113 = _tmp20 * _tmp63;
  const Scalar _tmp114 = _tmp16 * _tmp73;
  const Scalar _tmp115 = _tmp23 * _tmp66;
  const Scalar _tmp116 = -_tmp112 + _tmp113 + _tmp114 - _tmp115;
  const Scalar _tmp117 = _tmp111 + _tmp116;
  const Scalar _tmp118 = -_tmp39;
  const Scalar _tmp119 = _tmp118 + _tmp46;
  const Scalar _tmp120 = _tmp119 + p_init0;
  const Scalar _tmp121 = _tmp120 + Scalar(-2.5202214700000001);
  const Scalar _tmp122 = _tmp31 - _tmp34;
  const Scalar _tmp123 = _tmp122 + _tmp26;
  const Scalar _tmp124 = _tmp123 + p_init1;
  const Scalar _tmp125 = _tmp124 + Scalar(8.3888750099999996);
  const Scalar _tmp126 = std::pow(_tmp121, Scalar(2)) + std::pow(_tmp125, Scalar(2));
  const Scalar _tmp127 = std::pow(_tmp126, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp128 = _tmp125 * _tmp127;
  const Scalar _tmp129 = -_tmp26;
  const Scalar _tmp130 = _tmp129 + _tmp35;
  const Scalar _tmp131 = _tmp130 + p_init1;
  const Scalar _tmp132 = _tmp131 + Scalar(-4.8333311099999996);
  const Scalar _tmp133 = _tmp43 - _tmp45;
  const Scalar _tmp134 = _tmp133 + _tmp39;
  const Scalar _tmp135 = _tmp134 + p_init0;
  const Scalar _tmp136 = _tmp135 + Scalar(1.79662371);
  const Scalar _tmp137 = Scalar(1.0) / (_tmp136);
  const Scalar _tmp138 = _tmp132 * _tmp137;
  const Scalar _tmp139 = _tmp121 * _tmp127;
  const Scalar _tmp140 = -_tmp128 + _tmp138 * _tmp139;
  const Scalar _tmp141 =
      Scalar(0.20999999999999999) * _tmp40 - Scalar(0.20999999999999999) * _tmp41;
  const Scalar _tmp142 =
      -Scalar(0.010999999999999999) * _tmp32 - Scalar(0.010999999999999999) * _tmp44;
  const Scalar _tmp143 =
      Scalar(0.20999999999999999) * _tmp28 + Scalar(0.20999999999999999) * _tmp29;
  const Scalar _tmp144 = _tmp142 - _tmp143;
  const Scalar _tmp145 = _tmp141 + _tmp144;
  const Scalar _tmp146 = -_tmp141;
  const Scalar _tmp147 = _tmp142 + _tmp143;
  const Scalar _tmp148 = _tmp146 + _tmp147;
  const Scalar _tmp149 = _tmp139 * _tmp148;
  const Scalar _tmp150 = _tmp122 + _tmp129;
  const Scalar _tmp151 = _tmp150 + p_init1;
  const Scalar _tmp152 = _tmp151 + Scalar(8.3196563700000006);
  const Scalar _tmp153 = _tmp118 + _tmp133;
  const Scalar _tmp154 = _tmp153 + p_init0;
  const Scalar _tmp155 = _tmp154 + Scalar(1.9874742000000001);
  const Scalar _tmp156 = std::pow(_tmp152, Scalar(2)) + std::pow(_tmp155, Scalar(2));
  const Scalar _tmp157 = std::pow(_tmp156, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp158 = _tmp155 * _tmp157;
  const Scalar _tmp159 = _tmp148 * _tmp158;
  const Scalar _tmp160 = _tmp144 + _tmp146;
  const Scalar _tmp161 = _tmp152 * _tmp157;
  const Scalar _tmp162 = -_tmp138 * _tmp159 + _tmp160 * _tmp161;
  const Scalar _tmp163 = _tmp138 * _tmp158 - _tmp161;
  const Scalar _tmp164 = Scalar(1.0) / (_tmp163);
  const Scalar _tmp165 = _tmp140 * _tmp164;
  const Scalar _tmp166 = _tmp128 * _tmp145 - _tmp138 * _tmp149 - _tmp162 * _tmp165;
  const Scalar _tmp167 = Scalar(1.0) * _tmp130;
  const Scalar _tmp168 = -_tmp167;
  const Scalar _tmp169 = _tmp150 + _tmp168;
  const Scalar _tmp170 = Scalar(1.0) / (_tmp169);
  const Scalar _tmp171 = Scalar(1.0) * _tmp134;
  const Scalar _tmp172 = -_tmp153 + _tmp171;
  const Scalar _tmp173 = _tmp170 * _tmp172;
  const Scalar _tmp174 = -_tmp158 * _tmp160 + _tmp159;
  const Scalar _tmp175 = -_tmp139 * _tmp145 + _tmp149 - _tmp165 * _tmp174 - _tmp166 * _tmp173;
  const Scalar _tmp176 = Scalar(1.0) / (_tmp175);
  const Scalar _tmp177 = Scalar(1.0) * _tmp164;
  const Scalar _tmp178 = Scalar(1.0) * _tmp170;
  const Scalar _tmp179 = _tmp172 * _tmp178;
  const Scalar _tmp180 = _tmp164 * _tmp179;
  const Scalar _tmp181 = _tmp162 * _tmp180 - _tmp174 * _tmp177;
  const Scalar _tmp182 = std::pow(_tmp136, Scalar(2));
  const Scalar _tmp183 = std::pow(_tmp132, Scalar(2)) + _tmp182;
  const Scalar _tmp184 = std::sqrt(_tmp183);
  const Scalar _tmp185 = Scalar(1.0) / (_tmp184);
  const Scalar _tmp186 = _tmp134 * _tmp185;
  const Scalar _tmp187 = _tmp136 * _tmp185;
  const Scalar _tmp188 = -_tmp130 * _tmp187 + _tmp132 * _tmp186;
  const Scalar _tmp189 = _tmp137 * _tmp184;
  const Scalar _tmp190 = _tmp188 * _tmp189;
  const Scalar _tmp191 = _tmp150 * _tmp158 - _tmp153 * _tmp161 + _tmp158 * _tmp190;
  const Scalar _tmp192 =
      -_tmp119 * _tmp128 + _tmp123 * _tmp139 + _tmp139 * _tmp190 - _tmp165 * _tmp191;
  const Scalar _tmp193 = _tmp176 * _tmp192;
  const Scalar _tmp194 = -_tmp177 * _tmp191 - _tmp181 * _tmp193;
  const Scalar _tmp195 = Scalar(1.0) / (_tmp192);
  const Scalar _tmp196 = _tmp175 * _tmp195;
  const Scalar _tmp197 = _tmp194 * _tmp196;
  const Scalar _tmp198 = _tmp181 + _tmp197;
  const Scalar _tmp199 = _tmp176 * _tmp198;
  const Scalar _tmp200 = -_tmp140 * _tmp199 + Scalar(1.0);
  const Scalar _tmp201 = _tmp158 * _tmp164;
  const Scalar _tmp202 = _tmp139 * _tmp199 + _tmp200 * _tmp201;
  const Scalar _tmp203 = _tmp37 + Scalar(-4.7752063900000001);
  const Scalar _tmp204 = _tmp48 + Scalar(-2.71799795);
  const Scalar _tmp205 = std::pow(_tmp203, Scalar(2)) + std::pow(_tmp204, Scalar(2));
  const Scalar _tmp206 = std::pow(_tmp205, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp207 = _tmp203 * _tmp206;
  const Scalar _tmp208 = _tmp207 * fh1;
  const Scalar _tmp209 = _tmp189 * _tmp208;
  const Scalar _tmp210 = Scalar(1.0) * _tmp195;
  const Scalar _tmp211 = _tmp158 * _tmp210;
  const Scalar _tmp212 = _tmp139 * _tmp210 - _tmp165 * _tmp211;
  const Scalar _tmp213 = _tmp206 * _tmp36;
  const Scalar _tmp214 = fh1 * (-_tmp204 * _tmp213 + _tmp207 * _tmp47);
  const Scalar _tmp215 = _tmp189 * _tmp214;
  const Scalar _tmp216 = _tmp204 * _tmp206;
  const Scalar _tmp217 = _tmp216 * fh1;
  const Scalar _tmp218 = _tmp138 * _tmp164;
  const Scalar _tmp219 = _tmp138 * _tmp148;
  const Scalar _tmp220 = _tmp162 * _tmp218 + _tmp219;
  const Scalar _tmp221 = _tmp170 * _tmp220;
  const Scalar _tmp222 = -_tmp148 - _tmp172 * _tmp221 + _tmp174 * _tmp218;
  const Scalar _tmp223 = -_tmp190 + _tmp191 * _tmp218 - _tmp193 * _tmp222;
  const Scalar _tmp224 = _tmp196 * _tmp223;
  const Scalar _tmp225 = _tmp222 + _tmp224;
  const Scalar _tmp226 = _tmp176 * _tmp225;
  const Scalar _tmp227 = -_tmp138 - _tmp140 * _tmp226;
  const Scalar _tmp228 = _tmp139 * _tmp226 + _tmp201 * _tmp227 + Scalar(1.0);
  const Scalar _tmp229 = _tmp189 * _tmp228;
  const Scalar _tmp230 = _tmp167 * _tmp173 + _tmp171;
  const Scalar _tmp231 = 0;
  const Scalar _tmp232 = _tmp176 * _tmp231;
  const Scalar _tmp233 = _tmp158 * _tmp232;
  const Scalar _tmp234 = _tmp139 * _tmp232 - _tmp165 * _tmp233;
  const Scalar _tmp235 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp236 = _tmp189 * _tmp235;
  const Scalar _tmp237 =
      -_tmp202 * _tmp209 - _tmp212 * _tmp215 - _tmp217 * _tmp229 - _tmp234 * _tmp236;
  const Scalar _tmp238 = Scalar(1.0) / (_tmp237);
  const Scalar _tmp239 = _tmp123 + _tmp168;
  const Scalar _tmp240 = _tmp173 * _tmp239;
  const Scalar _tmp241 = -_tmp119 + _tmp171 - _tmp240;
  const Scalar _tmp242 = Scalar(1.0) / (_tmp241);
  const Scalar _tmp243 = Scalar(1.0) * _tmp242;
  const Scalar _tmp244 = _tmp162 * _tmp177;
  const Scalar _tmp245 = _tmp239 * _tmp242;
  const Scalar _tmp246 = -_tmp166 * _tmp199 + _tmp197 * _tmp245 - _tmp244;
  const Scalar _tmp247 = -_tmp178 * _tmp246 + _tmp197 * _tmp243;
  const Scalar _tmp248 = Scalar(1.0) * _tmp247;
  const Scalar _tmp249 = _tmp240 * _tmp243 + Scalar(1.0);
  const Scalar _tmp250 = _tmp173 * _tmp243;
  const Scalar _tmp251 = -_tmp178 * _tmp249 + _tmp250;
  const Scalar _tmp252 = fh1 * (_tmp141 + _tmp147);
  const Scalar _tmp253 = -_tmp207 * _tmp252 - Scalar(3.29616) * _tmp30 - _tmp36 * fv1;
  const Scalar _tmp254 = Scalar(1.0) * _tmp253;
  const Scalar _tmp255 = _tmp230 * _tmp242;
  const Scalar _tmp256 = -_tmp166 * _tmp232 + _tmp168 - _tmp239 * _tmp255;
  const Scalar _tmp257 = Scalar(1.0) * _tmp235;
  const Scalar _tmp258 = _tmp170 * _tmp243;
  const Scalar _tmp259 = _tmp239 * _tmp258 - _tmp243;
  const Scalar _tmp260 = _tmp206 * _tmp252;
  const Scalar _tmp261 = _tmp204 * _tmp260 + Scalar(3.29616) * _tmp42 + _tmp47 * fv1;
  const Scalar _tmp262 = Scalar(1.0) * _tmp261;
  const Scalar _tmp263 = -_tmp166 * _tmp226 + _tmp220 + _tmp224 * _tmp245;
  const Scalar _tmp264 = -_tmp178 * _tmp263 + _tmp224 * _tmp243;
  const Scalar _tmp265 = Scalar(1.0) * _tmp217;
  const Scalar _tmp266 = _tmp196 * _tmp243;
  const Scalar _tmp267 = -_tmp166 * _tmp210 + _tmp239 * _tmp266;
  const Scalar _tmp268 = -_tmp178 * _tmp267 + _tmp266;
  const Scalar _tmp269 = Scalar(1.0) * _tmp214;
  const Scalar _tmp270 = _tmp208 * _tmp248 + _tmp251 * _tmp254 +
                         _tmp257 * (-_tmp178 * _tmp256 - _tmp230 * _tmp243 + Scalar(1.0)) +
                         _tmp259 * _tmp262 + _tmp264 * _tmp265 + _tmp268 * _tmp269;
  const Scalar _tmp271 = std::asinh(_tmp238 * _tmp270);
  const Scalar _tmp272 = Scalar(9.6622558468725703) * _tmp271;
  const Scalar _tmp273 = Scalar(4.8333311099999996) - _tmp131;
  const Scalar _tmp274 = -_tmp135 + Scalar(-1.79662371);
  const Scalar _tmp275 =
      std::sqrt(Scalar(std::pow(_tmp273, Scalar(2)) + std::pow(_tmp274, Scalar(2))));
  const Scalar _tmp276 = -_tmp237 * _tmp272 - _tmp275;
  const Scalar _tmp277 = std::pow(_tmp237, Scalar(-2));
  const Scalar _tmp278 = _tmp64 + _tmp67;
  const Scalar _tmp279 = _tmp278 + _tmp92;
  const Scalar _tmp280 = -_tmp83 - _tmp85 - _tmp87 - _tmp89;
  const Scalar _tmp281 = _tmp280 + _tmp81;
  const Scalar _tmp282 = _tmp279 + _tmp281;
  const Scalar _tmp283 = Scalar(1.0) / (_tmp182);
  const Scalar _tmp284 = _tmp184 * _tmp282 * _tmp283;
  const Scalar _tmp285 = _tmp234 * _tmp235;
  const Scalar _tmp286 = -_tmp80;
  const Scalar _tmp287 = -_tmp100 + _tmp286;
  const Scalar _tmp288 = _tmp287 + _tmp93;
  const Scalar _tmp289 = _tmp94 - _tmp95 - _tmp96 + _tmp97;
  const Scalar _tmp290 = _tmp288 + _tmp289;
  const Scalar _tmp291 = _tmp132 * _tmp290 + _tmp136 * _tmp282;
  const Scalar _tmp292 = _tmp137 * _tmp185 * _tmp291;
  const Scalar _tmp293 = _tmp217 * _tmp228;
  const Scalar _tmp294 = _tmp278 + _tmp75;
  const Scalar _tmp295 = _tmp287 + _tmp294;
  const Scalar _tmp296 = _tmp289 + _tmp295;
  const Scalar _tmp297 = _tmp286 - _tmp78;
  const Scalar _tmp298 = _tmp279 + _tmp297;
  const Scalar _tmp299 = _tmp280 + _tmp298;
  const Scalar _tmp300 =
      (2 * _tmp203 * _tmp296 + 2 * _tmp204 * _tmp299) / (_tmp205 * std::sqrt(_tmp205));
  const Scalar _tmp301 = (Scalar(1) / Scalar(2)) * _tmp300;
  const Scalar _tmp302 = _tmp204 * _tmp301;
  const Scalar _tmp303 = _tmp302 * fh1;
  const Scalar _tmp304 = _tmp137 * _tmp290;
  const Scalar _tmp305 = _tmp297 + _tmp76;
  const Scalar _tmp306 = _tmp280 + _tmp305;
  const Scalar _tmp307 = _tmp101 + _tmp289;
  const Scalar _tmp308 = _tmp294 + _tmp307;
  const Scalar _tmp309 =
      (2 * _tmp121 * _tmp306 + 2 * _tmp125 * _tmp308) / (_tmp126 * std::sqrt(_tmp126));
  const Scalar _tmp310 = (Scalar(1) / Scalar(2)) * _tmp309;
  const Scalar _tmp311 = _tmp121 * _tmp310;
  const Scalar _tmp312 = _tmp132 * _tmp282;
  const Scalar _tmp313 = _tmp283 * _tmp312;
  const Scalar _tmp314 = _tmp127 * _tmp308;
  const Scalar _tmp315 = _tmp125 * _tmp310;
  const Scalar _tmp316 = _tmp127 * _tmp306;
  const Scalar _tmp317 = -_tmp138 * _tmp311 + _tmp138 * _tmp316 + _tmp139 * _tmp304 -
                         _tmp139 * _tmp313 - _tmp314 + _tmp315;
  const Scalar _tmp318 = _tmp164 * _tmp317;
  const Scalar _tmp319 = _tmp307 + _tmp93;
  const Scalar _tmp320 = _tmp281 + _tmp76;
  const Scalar _tmp321 =
      (2 * _tmp152 * _tmp319 + 2 * _tmp155 * _tmp320) / (_tmp156 * std::sqrt(_tmp156));
  const Scalar _tmp322 = (Scalar(1) / Scalar(2)) * _tmp321;
  const Scalar _tmp323 = _tmp152 * _tmp322;
  const Scalar _tmp324 = _tmp157 * _tmp319;
  const Scalar _tmp325 = _tmp157 * _tmp320;
  const Scalar _tmp326 = _tmp155 * _tmp322;
  const Scalar _tmp327 = (_tmp138 * _tmp325 - _tmp138 * _tmp326 + _tmp158 * _tmp304 -
                          _tmp158 * _tmp313 + _tmp323 - _tmp324) /
                         std::pow(_tmp163, Scalar(2));
  const Scalar _tmp328 = _tmp158 * _tmp327;
  const Scalar _tmp329 = _tmp140 * _tmp328;
  const Scalar _tmp330 = _tmp112 - _tmp113 - _tmp114 + _tmp115;
  const Scalar _tmp331 = _tmp111 + _tmp330;
  const Scalar _tmp332 = _tmp139 * _tmp331;
  const Scalar _tmp333 = _tmp148 * _tmp325;
  const Scalar _tmp334 = _tmp138 * _tmp331;
  const Scalar _tmp335 = -_tmp103 - _tmp104 - _tmp105 - _tmp106;
  const Scalar _tmp336 = _tmp110 + _tmp335;
  const Scalar _tmp337 = _tmp330 + _tmp336;
  const Scalar _tmp338 = -_tmp138 * _tmp333 - _tmp158 * _tmp334 - _tmp159 * _tmp304 +
                         _tmp159 * _tmp313 - _tmp160 * _tmp323 + _tmp160 * _tmp324 +
                         _tmp161 * _tmp337 + _tmp219 * _tmp326;
  const Scalar _tmp339 = _tmp148 * _tmp316;
  const Scalar _tmp340 = _tmp116 + _tmp336;
  const Scalar _tmp341 = _tmp162 * _tmp327;
  const Scalar _tmp342 = _tmp128 * _tmp340 - _tmp138 * _tmp332 - _tmp138 * _tmp339 +
                         _tmp140 * _tmp341 + _tmp145 * _tmp314 - _tmp145 * _tmp315 -
                         _tmp149 * _tmp304 + _tmp149 * _tmp313 - _tmp162 * _tmp318 -
                         _tmp165 * _tmp338 + _tmp219 * _tmp311;
  const Scalar _tmp343 = Scalar(1.6799999999999999) * _tmp20 * _tmp72;
  const Scalar _tmp344 = _tmp343 + Scalar(1.6799999999999999) * _tmp99;
  const Scalar _tmp345 = _tmp344 / std::pow(_tmp169, Scalar(2));
  const Scalar _tmp346 = _tmp172 * _tmp345;
  const Scalar _tmp347 = _tmp174 * _tmp327;
  const Scalar _tmp348 = _tmp69 * _tmp79;
  const Scalar _tmp349 = Scalar(0.83999999999999997) * _tmp23 * _tmp72;
  const Scalar _tmp350 = Scalar(0.83999999999999997) * _tmp16 * _tmp65 + _tmp62 * _tmp77;
  const Scalar _tmp351 = -_tmp348 - _tmp349 + _tmp350;
  const Scalar _tmp352 = _tmp170 * _tmp351;
  const Scalar _tmp353 = -_tmp148 * _tmp326 + _tmp158 * _tmp331 - _tmp158 * _tmp337 -
                         _tmp160 * _tmp325 + _tmp160 * _tmp326 + _tmp333;
  const Scalar _tmp354 = -_tmp139 * _tmp340 + _tmp140 * _tmp347 + _tmp145 * _tmp311 -
                         _tmp145 * _tmp316 - _tmp148 * _tmp311 - _tmp165 * _tmp353 +
                         _tmp166 * _tmp346 - _tmp166 * _tmp352 - _tmp173 * _tmp342 -
                         _tmp174 * _tmp318 + _tmp332 + _tmp339;
  const Scalar _tmp355 = _tmp354 / std::pow(_tmp175, Scalar(2));
  const Scalar _tmp356 = _tmp231 * _tmp355;
  const Scalar _tmp357 = _tmp165 * _tmp325;
  const Scalar _tmp358 = _tmp140 * _tmp355;
  const Scalar _tmp359 = _tmp291 / (_tmp183 * std::sqrt(_tmp183));
  const Scalar _tmp360 = _tmp189 * (_tmp130 * _tmp136 * _tmp359 - _tmp130 * _tmp185 * _tmp282 -
                                    _tmp132 * _tmp134 * _tmp359 + _tmp185 * _tmp312 +
                                    _tmp186 * _tmp290 - _tmp187 * _tmp290);
  const Scalar _tmp361 = _tmp191 * _tmp327;
  const Scalar _tmp362 = _tmp188 * _tmp292;
  const Scalar _tmp363 = _tmp188 * _tmp284;
  const Scalar _tmp364 = _tmp150 * _tmp325 - _tmp150 * _tmp326 + _tmp153 * _tmp323 -
                         _tmp153 * _tmp324 + _tmp155 * _tmp324 + _tmp158 * _tmp360 +
                         _tmp158 * _tmp362 - _tmp158 * _tmp363 - _tmp161 * _tmp320 +
                         _tmp190 * _tmp325 - _tmp190 * _tmp326;
  const Scalar _tmp365 = -_tmp119 * _tmp314 + _tmp119 * _tmp315 + _tmp121 * _tmp314 -
                         _tmp123 * _tmp311 + _tmp123 * _tmp316 - _tmp128 * _tmp306 +
                         _tmp139 * _tmp360 + _tmp139 * _tmp362 - _tmp139 * _tmp363 +
                         _tmp140 * _tmp361 - _tmp165 * _tmp364 - _tmp190 * _tmp311 +
                         _tmp190 * _tmp316 - _tmp191 * _tmp318;
  const Scalar _tmp366 = _tmp365 / std::pow(_tmp192, Scalar(2));
  const Scalar _tmp367 = _tmp175 * _tmp194;
  const Scalar _tmp368 = _tmp366 * _tmp367;
  const Scalar _tmp369 = _tmp176 * _tmp365;
  const Scalar _tmp370 = _tmp192 * _tmp355;
  const Scalar _tmp371 = _tmp162 * _tmp164 * _tmp178 * _tmp351 - _tmp177 * _tmp353 -
                         _tmp179 * _tmp341 + _tmp180 * _tmp338 - _tmp244 * _tmp346 +
                         Scalar(1.0) * _tmp347;
  const Scalar _tmp372 = _tmp196 * (-_tmp177 * _tmp364 - _tmp181 * _tmp369 + _tmp181 * _tmp370 -
                                    _tmp193 * _tmp371 + Scalar(1.0) * _tmp361);
  const Scalar _tmp373 = _tmp195 * _tmp354;
  const Scalar _tmp374 = _tmp194 * _tmp373;
  const Scalar _tmp375 = -_tmp368 + _tmp371 + _tmp372 + _tmp374;
  const Scalar _tmp376 = _tmp140 * _tmp176;
  const Scalar _tmp377 = _tmp198 * _tmp358 - _tmp199 * _tmp317 - _tmp375 * _tmp376;
  const Scalar _tmp378 = _tmp139 * _tmp355;
  const Scalar _tmp379 = _tmp164 * _tmp325;
  const Scalar _tmp380 = _tmp164 * _tmp326;
  const Scalar _tmp381 = _tmp139 * _tmp176;
  const Scalar _tmp382 = Scalar(0.5) * _tmp195;
  const Scalar _tmp383 = _tmp140 * _tmp177 * _tmp366;
  const Scalar _tmp384 = Scalar(1.0) * _tmp366;
  const Scalar _tmp385 = _tmp202 * _tmp208;
  const Scalar _tmp386 = _tmp175 * _tmp366;
  const Scalar _tmp387 = _tmp223 * _tmp386;
  const Scalar _tmp388 = _tmp164 * _tmp304;
  const Scalar _tmp389 = _tmp164 * _tmp313;
  const Scalar _tmp390 = -_tmp138 * _tmp341 + _tmp148 * _tmp304 - _tmp148 * _tmp313 +
                         _tmp162 * _tmp388 - _tmp162 * _tmp389 + _tmp218 * _tmp338 + _tmp334;
  const Scalar _tmp391 = -_tmp107 - _tmp109 + _tmp116 - _tmp138 * _tmp347 - _tmp173 * _tmp390 +
                         _tmp174 * _tmp388 - _tmp174 * _tmp389 + _tmp218 * _tmp353 +
                         _tmp220 * _tmp346 - _tmp221 * _tmp351 + _tmp335;
  const Scalar _tmp392 = _tmp196 * (-_tmp138 * _tmp361 + _tmp191 * _tmp388 - _tmp191 * _tmp389 -
                                    _tmp193 * _tmp391 + _tmp218 * _tmp364 - _tmp222 * _tmp369 +
                                    _tmp222 * _tmp370 - _tmp360 - _tmp362 + _tmp363);
  const Scalar _tmp393 = _tmp223 * _tmp373;
  const Scalar _tmp394 = -_tmp387 + _tmp391 + _tmp392 + _tmp393;
  const Scalar _tmp395 =
      _tmp225 * _tmp358 - _tmp226 * _tmp317 - _tmp304 + _tmp313 - _tmp376 * _tmp394;
  const Scalar _tmp396 = _tmp203 * _tmp301;
  const Scalar _tmp397 = _tmp206 * _tmp296;
  const Scalar _tmp398 = fh1 * (-_tmp204 * _tmp397 + _tmp207 * _tmp299 - _tmp213 * _tmp299 +
                                _tmp302 * _tmp36 - _tmp396 * _tmp47 + _tmp397 * _tmp47);
  const Scalar _tmp399 = _tmp212 * _tmp214;
  const Scalar _tmp400 = _tmp206 * _tmp299;
  const Scalar _tmp401 = _tmp400 * fh1;
  const Scalar _tmp402 = _tmp189 * _tmp202;
  const Scalar _tmp403 = _tmp396 * fh1;
  const Scalar _tmp404 = _tmp397 * fh1;
  const Scalar _tmp405 =
      -_tmp189 * _tmp212 * _tmp398 -
      _tmp189 * _tmp217 *
          (_tmp201 * _tmp395 - _tmp225 * _tmp378 - _tmp226 * _tmp311 + _tmp226 * _tmp316 -
           _tmp227 * _tmp328 + _tmp227 * _tmp379 - _tmp227 * _tmp380 + _tmp381 * _tmp394) -
      _tmp209 * (-_tmp198 * _tmp378 - _tmp199 * _tmp311 + _tmp199 * _tmp316 - _tmp200 * _tmp328 +
                 _tmp200 * _tmp379 - _tmp200 * _tmp380 + _tmp201 * _tmp377 + _tmp375 * _tmp381) -
      _tmp215 * (-_tmp121 * _tmp309 * _tmp382 - _tmp139 * _tmp384 +
                 _tmp155 * _tmp165 * _tmp321 * _tmp382 + _tmp158 * _tmp383 + _tmp210 * _tmp316 +
                 _tmp210 * _tmp329 - _tmp210 * _tmp357 - _tmp211 * _tmp318) +
      _tmp229 * _tmp303 - _tmp229 * _tmp401 -
      _tmp236 * (-_tmp139 * _tmp356 + _tmp158 * _tmp165 * _tmp356 + _tmp165 * _tmp232 * _tmp326 -
                 _tmp232 * _tmp311 + _tmp232 * _tmp316 + _tmp232 * _tmp329 - _tmp232 * _tmp357 -
                 _tmp233 * _tmp318) +
      _tmp284 * _tmp285 + _tmp284 * _tmp293 + _tmp284 * _tmp385 + _tmp284 * _tmp399 -
      _tmp285 * _tmp292 - _tmp292 * _tmp293 - _tmp292 * _tmp385 - _tmp292 * _tmp399 +
      _tmp402 * _tmp403 - _tmp402 * _tmp404;
  const Scalar _tmp406 = _tmp277 * _tmp405;
  const Scalar _tmp407 = _tmp102 + _tmp294;
  const Scalar _tmp408 = Scalar(9.6622558468725703) * _tmp237;
  const Scalar _tmp409 = -_tmp167 * _tmp346 + _tmp167 * _tmp352 + _tmp179 * _tmp290 + _tmp282;
  const Scalar _tmp410 = _tmp239 * _tmp346;
  const Scalar _tmp411 = _tmp344 + _tmp348 + _tmp349 + _tmp350;
  const Scalar _tmp412 = _tmp173 * _tmp411;
  const Scalar _tmp413 = _tmp239 * _tmp352;
  const Scalar _tmp414 =
      (Scalar(1.6799999999999999) * _tmp108 + _tmp343 + _tmp351 + _tmp410 - _tmp412 - _tmp413) /
      std::pow(_tmp241, Scalar(2));
  const Scalar _tmp415 = _tmp230 * _tmp414;
  const Scalar _tmp416 = _tmp166 * _tmp355;
  const Scalar _tmp417 = _tmp242 * _tmp409;
  const Scalar _tmp418 = _tmp231 * _tmp416 - _tmp232 * _tmp342 + _tmp239 * _tmp415 -
                         _tmp239 * _tmp417 - _tmp255 * _tmp411 + _tmp407;
  const Scalar _tmp419 = Scalar(1.0) * _tmp414;
  const Scalar _tmp420 = Scalar(1.0) * _tmp345;
  const Scalar _tmp421 = _tmp210 * _tmp414;
  const Scalar _tmp422 = _tmp239 * _tmp414;
  const Scalar _tmp423 = _tmp242 * _tmp411;
  const Scalar _tmp424 = _tmp166 * _tmp176;
  const Scalar _tmp425 = -_tmp177 * _tmp338 - _tmp197 * _tmp422 + _tmp197 * _tmp423 +
                         _tmp198 * _tmp416 - _tmp199 * _tmp342 - _tmp245 * _tmp368 +
                         _tmp245 * _tmp372 + _tmp245 * _tmp374 + Scalar(1.0) * _tmp341 -
                         _tmp375 * _tmp424;
  const Scalar _tmp426 = Scalar(6.59232) * _tmp69;
  const Scalar _tmp427 = Scalar(6.59232) * _tmp20;
  const Scalar _tmp428 = Scalar(6.59232) * _tmp72;
  const Scalar _tmp429 = _tmp117 * fh1;
  const Scalar _tmp430 = Scalar(6.59232) * _tmp23;
  const Scalar _tmp431 = _tmp12 * _tmp426 + _tmp16 * _tmp428 + _tmp216 * _tmp429 -
                         _tmp252 * _tmp302 + _tmp260 * _tmp299 + _tmp299 * fv1 + _tmp427 * _tmp62 +
                         _tmp430 * _tmp65;
  const Scalar _tmp432 = _tmp239 * _tmp345;
  const Scalar _tmp433 = _tmp175 * _tmp421;
  const Scalar _tmp434 = -_tmp224 * _tmp422 + _tmp224 * _tmp423 + _tmp225 * _tmp416 -
                         _tmp226 * _tmp342 - _tmp245 * _tmp387 + _tmp245 * _tmp392 +
                         _tmp245 * _tmp393 + _tmp390 - _tmp394 * _tmp424;
  const Scalar _tmp435 = Scalar(0.5) * _tmp300 * fh1;
  const Scalar _tmp436 = _tmp243 * _tmp373;
  const Scalar _tmp437 = _tmp243 * _tmp386;
  const Scalar _tmp438 = _tmp166 * _tmp384 - _tmp175 * _tmp210 * _tmp422 - _tmp210 * _tmp342 +
                         _tmp239 * _tmp436 - _tmp239 * _tmp437 + _tmp266 * _tmp411;
  const Scalar _tmp439 = -_tmp12 * _tmp428 + _tmp16 * _tmp426 - _tmp207 * _tmp429 +
                         _tmp252 * _tmp396 - _tmp252 * _tmp397 - _tmp296 * fv1 - _tmp427 * _tmp65 +
                         _tmp430 * _tmp62;
  const Scalar _tmp440 =
      _tmp170 * (-_tmp179 * _tmp422 - _tmp243 * _tmp410 + _tmp243 * _tmp412 + _tmp243 * _tmp413);
  const Scalar _tmp441 = _tmp249 * _tmp345;
  const Scalar _tmp442 = _tmp179 * _tmp414;
  const Scalar _tmp443 = _tmp243 * _tmp352;
  const Scalar _tmp444 = _tmp243 * _tmp346;
  const Scalar _tmp445 =
      (_tmp238 *
           (-_tmp203 * _tmp247 * _tmp435 - _tmp204 * _tmp264 * _tmp435 +
            Scalar(1.0) * _tmp208 *
                (-_tmp178 * _tmp425 - _tmp243 * _tmp368 + _tmp243 * _tmp372 + _tmp243 * _tmp374 +
                 _tmp246 * _tmp420 - _tmp367 * _tmp421) +
            _tmp248 * _tmp404 + Scalar(1.0) * _tmp251 * _tmp439 +
            _tmp254 *
                (-Scalar(1.0) * _tmp440 + Scalar(1.0) * _tmp441 - _tmp442 + _tmp443 - _tmp444) +
            _tmp257 *
                (-_tmp178 * _tmp418 + _tmp230 * _tmp419 - _tmp243 * _tmp409 + _tmp256 * _tmp420) +
            Scalar(1.0) * _tmp259 * _tmp431 +
            _tmp262 * (-_tmp178 * _tmp422 - _tmp243 * _tmp432 + _tmp258 * _tmp411 + _tmp419) +
            Scalar(1.0) * _tmp264 * _tmp401 +
            _tmp265 * (-_tmp178 * _tmp434 - _tmp223 * _tmp433 - _tmp243 * _tmp387 +
                       _tmp243 * _tmp392 + _tmp243 * _tmp393 + _tmp263 * _tmp420) +
            Scalar(1.0) * _tmp268 * _tmp398 +
            _tmp269 * (-_tmp178 * _tmp438 + _tmp267 * _tmp420 - _tmp433 + _tmp436 - _tmp437)) -
       _tmp270 * _tmp406) /
      std::sqrt(Scalar(std::pow(_tmp270, Scalar(2)) * _tmp277 + 1));
  const Scalar _tmp446 = Scalar(0.1034955) * _tmp238;
  const Scalar _tmp447 = _tmp276 * _tmp446;
  const Scalar _tmp448 = Scalar(1.0) * _tmp271;
  const Scalar _tmp449 = _tmp164 * _tmp217;
  const Scalar _tmp450 = _tmp164 * _tmp208;
  const Scalar _tmp451 = _tmp210 * _tmp214;
  const Scalar _tmp452 = _tmp232 * _tmp235;
  const Scalar _tmp453 =
      -_tmp165 * _tmp451 - _tmp165 * _tmp452 + _tmp200 * _tmp450 + _tmp227 * _tmp449;
  const Scalar _tmp454 = Scalar(1.0) / (_tmp453);
  const Scalar _tmp455 = _tmp170 * _tmp217;
  const Scalar _tmp456 = _tmp170 * _tmp214;
  const Scalar _tmp457 = _tmp170 * _tmp235;
  const Scalar _tmp458 = _tmp243 * _tmp261;
  const Scalar _tmp459 = _tmp170 * _tmp239;
  const Scalar _tmp460 = _tmp170 * _tmp249;
  const Scalar _tmp461 = _tmp170 * _tmp208;
  const Scalar _tmp462 = _tmp246 * _tmp461 + _tmp253 * _tmp460 + _tmp256 * _tmp457 +
                         _tmp263 * _tmp455 + _tmp267 * _tmp456 - _tmp458 * _tmp459;
  const Scalar _tmp463 = std::asinh(_tmp454 * _tmp462);
  const Scalar _tmp464 = Scalar(1.0) * _tmp463;
  const Scalar _tmp465 = std::pow(_tmp453, Scalar(-2));
  const Scalar _tmp466 = _tmp210 * _tmp398;
  const Scalar _tmp467 = _tmp235 * _tmp356;
  const Scalar _tmp468 = _tmp164 * fh1;
  const Scalar _tmp469 = _tmp200 * _tmp468;
  const Scalar _tmp470 = _tmp140 * _tmp327;
  const Scalar _tmp471 = _tmp227 * _tmp468;
  const Scalar _tmp472 = -_tmp165 * _tmp466 + _tmp165 * _tmp467 - _tmp200 * _tmp208 * _tmp327 +
                         _tmp214 * _tmp383 - _tmp217 * _tmp227 * _tmp327 - _tmp302 * _tmp471 -
                         _tmp318 * _tmp451 - _tmp318 * _tmp452 + _tmp377 * _tmp450 +
                         _tmp395 * _tmp449 - _tmp396 * _tmp469 + _tmp397 * _tmp469 +
                         _tmp400 * _tmp471 + _tmp451 * _tmp470 + _tmp452 * _tmp470;
  const Scalar _tmp473 = _tmp465 * _tmp472;
  const Scalar _tmp474 = _tmp170 * _tmp246;
  const Scalar _tmp475 = _tmp170 * _tmp263;
  const Scalar _tmp476 = _tmp262 * _tmp414;
  const Scalar _tmp477 = _tmp243 * _tmp431;
  const Scalar _tmp478 =
      (_tmp454 * (_tmp170 * _tmp267 * _tmp398 - _tmp170 * _tmp411 * _tmp458 -
                  _tmp208 * _tmp246 * _tmp345 - _tmp214 * _tmp267 * _tmp345 -
                  _tmp217 * _tmp263 * _tmp345 - _tmp235 * _tmp256 * _tmp345 + _tmp253 * _tmp440 -
                  _tmp253 * _tmp441 - _tmp303 * _tmp475 + _tmp401 * _tmp475 - _tmp403 * _tmp474 +
                  _tmp404 * _tmp474 + _tmp418 * _tmp457 + _tmp425 * _tmp461 + _tmp432 * _tmp458 +
                  _tmp434 * _tmp455 + _tmp438 * _tmp456 + _tmp439 * _tmp460 + _tmp459 * _tmp476 -
                  _tmp459 * _tmp477) -
       _tmp462 * _tmp473) /
      std::sqrt(Scalar(std::pow(_tmp462, Scalar(2)) * _tmp465 + 1));
  const Scalar _tmp479 = Scalar(9.6622558468725703) * _tmp463;
  const Scalar _tmp480 = -_tmp151 + Scalar(-8.3196563700000006);
  const Scalar _tmp481 = -_tmp154 + Scalar(-1.9874742000000001);
  const Scalar _tmp482 =
      std::sqrt(Scalar(std::pow(_tmp480, Scalar(2)) + std::pow(_tmp481, Scalar(2))));
  const Scalar _tmp483 = -_tmp453 * _tmp479 - _tmp482;
  const Scalar _tmp484 = Scalar(0.1034955) * _tmp454;
  const Scalar _tmp485 = _tmp483 * _tmp484;
  const Scalar _tmp486 = Scalar(9.6622558468725703) * _tmp453;
  const Scalar _tmp487 = -_tmp124 + Scalar(-8.3888750099999996);
  const Scalar _tmp488 = Scalar(2.5202214700000001) - _tmp120;
  const Scalar _tmp489 =
      std::sqrt(Scalar(std::pow(_tmp487, Scalar(2)) + std::pow(_tmp488, Scalar(2))));
  const Scalar _tmp490 = _tmp217 * _tmp242;
  const Scalar _tmp491 = _tmp197 * _tmp242;
  const Scalar _tmp492 = -_tmp208 * _tmp491 - _tmp214 * _tmp266 - _tmp224 * _tmp490 +
                         _tmp235 * _tmp255 - _tmp250 * _tmp253 + _tmp458;
  const Scalar _tmp493 = _tmp199 * _tmp208 + _tmp217 * _tmp226 + _tmp451 + _tmp452;
  const Scalar _tmp494 = Scalar(1.0) / (_tmp493);
  const Scalar _tmp495 = std::asinh(_tmp492 * _tmp494);
  const Scalar _tmp496 = Scalar(9.6622558468725703) * _tmp493;
  const Scalar _tmp497 = -_tmp489 - _tmp495 * _tmp496;
  const Scalar _tmp498 = Scalar(0.1034955) * _tmp494;
  const Scalar _tmp499 = _tmp497 * _tmp498;
  const Scalar _tmp500 = Scalar(1.0) * _tmp495;
  const Scalar _tmp501 = _tmp176 * _tmp208 * _tmp375 + _tmp176 * _tmp217 * _tmp394 -
                         _tmp198 * _tmp208 * _tmp355 - _tmp199 * _tmp403 + _tmp199 * _tmp404 -
                         _tmp217 * _tmp225 * _tmp355 - _tmp226 * _tmp303 + _tmp226 * _tmp401 -
                         _tmp269 * _tmp366 + _tmp466 - _tmp467;
  const Scalar _tmp502 = Scalar(9.6622558468725703) * _tmp501;
  const Scalar _tmp503 = std::pow(_tmp493, Scalar(-2));
  const Scalar _tmp504 = _tmp501 * _tmp503;
  const Scalar _tmp505 = _tmp208 * _tmp242;
  const Scalar _tmp506 = _tmp224 * _tmp242;
  const Scalar _tmp507 =
      (-_tmp492 * _tmp504 +
       _tmp494 * (_tmp175 * _tmp414 * _tmp451 + _tmp197 * _tmp208 * _tmp414 - _tmp214 * _tmp436 +
                  _tmp214 * _tmp437 + _tmp217 * _tmp224 * _tmp414 - _tmp235 * _tmp415 +
                  _tmp235 * _tmp417 - _tmp250 * _tmp439 + _tmp253 * _tmp442 - _tmp253 * _tmp443 +
                  _tmp253 * _tmp444 - _tmp266 * _tmp398 + _tmp303 * _tmp506 + _tmp368 * _tmp505 -
                  _tmp372 * _tmp505 - _tmp374 * _tmp505 + _tmp387 * _tmp490 - _tmp392 * _tmp490 -
                  _tmp393 * _tmp490 - _tmp401 * _tmp506 + _tmp403 * _tmp491 - _tmp404 * _tmp491 -
                  _tmp476 + _tmp477)) /
      std::sqrt(Scalar(std::pow(_tmp492, Scalar(2)) * _tmp503 + 1));

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) =
      _tmp117 -
      Scalar(0.5) * (2 * _tmp38 * (_tmp102 + _tmp93) + 2 * _tmp49 * (_tmp76 + _tmp91)) *
          std::sinh(Scalar(0.1034955) * _tmp51 *
                    (-_tmp50 - Scalar(9.6622558468725703) * fh1 * std::asinh(_tmp51 * fv1))) /
          _tmp50;
  _res(1, 0) =
      _tmp331 -
      Scalar(9.6622558468725703) * _tmp405 *
          (Scalar(0.86625939559540499) * _tmp238 - std::cosh(_tmp447) + std::cosh(_tmp448)) -
      _tmp408 *
          (-Scalar(0.86625939559540499) * _tmp406 + Scalar(1.0) * _tmp445 * std::sinh(_tmp448) -
           (-Scalar(0.1034955) * _tmp276 * _tmp406 +
            _tmp446 * (-_tmp272 * _tmp405 - _tmp408 * _tmp445 -
                       Scalar(1) / Scalar(2) *
                           (2 * _tmp273 * _tmp407 + 2 * _tmp274 * (_tmp305 + _tmp90)) / _tmp275)) *
               std::sinh(_tmp447));
  _res(2, 0) =
      _tmp337 -
      Scalar(9.6622558468725703) * _tmp472 *
          (Scalar(0.87679799772039002) * _tmp454 + std::cosh(_tmp464) - std::cosh(_tmp485)) -
      _tmp486 *
          (-Scalar(0.87679799772039002) * _tmp473 + Scalar(1.0) * _tmp478 * std::sinh(_tmp464) -
           (-Scalar(0.1034955) * _tmp473 * _tmp483 +
            _tmp484 * (-_tmp472 * _tmp479 - _tmp478 * _tmp486 -
                       Scalar(1) / Scalar(2) *
                           (2 * _tmp480 * (_tmp295 + _tmp98) + 2 * _tmp481 * (_tmp298 + _tmp90)) /
                           _tmp482)) *
               std::sinh(_tmp485));
  _res(3, 0) =
      _tmp340 -
      _tmp496 *
          (-Scalar(0.87653584775870996) * _tmp504 + Scalar(1.0) * _tmp507 * std::sinh(_tmp500) -
           (-Scalar(0.1034955) * _tmp497 * _tmp504 +
            _tmp498 * (-_tmp495 * _tmp502 - _tmp496 * _tmp507 -
                       Scalar(1) / Scalar(2) *
                           (2 * _tmp487 * (_tmp288 + _tmp98) + 2 * _tmp488 * (_tmp279 + _tmp91)) /
                           _tmp489)) *
               std::sinh(_tmp499)) -
      _tmp502 * (Scalar(0.87653584775870996) * _tmp494 - std::cosh(_tmp499) + std::cosh(_tmp500));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
