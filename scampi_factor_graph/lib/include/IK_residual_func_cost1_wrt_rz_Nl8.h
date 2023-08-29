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
 * Symbolic function: IK_residual_func_cost1_wrt_rz_Nl8
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost1WrtRzNl8(
    const Scalar fh1, const Scalar fv1, const Scalar rx, const Scalar ry, const Scalar rz,
    const Scalar p_init0, const Scalar p_init1, const Scalar p_init2, const Scalar rot_init_x,
    const Scalar rot_init_y, const Scalar rot_init_z, const Scalar rot_init_w,
    const Scalar epsilon) {
  // Total ops: 1613

  // Unused inputs
  (void)p_init2;
  (void)epsilon;

  // Input arrays

  // Intermediate terms (494)
  const Scalar _tmp0 = Scalar(1.0) / (fh1);
  const Scalar _tmp1 = std::pow(rz, Scalar(2));
  const Scalar _tmp2 = _tmp1 + std::pow(rx, Scalar(2)) + std::pow(ry, Scalar(2));
  const Scalar _tmp3 = std::sqrt(_tmp2);
  const Scalar _tmp4 = (Scalar(1) / Scalar(2)) * _tmp3;
  const Scalar _tmp5 = std::cos(_tmp4);
  const Scalar _tmp6 = _tmp5 * rot_init_y;
  const Scalar _tmp7 = std::sin(_tmp4);
  const Scalar _tmp8 = _tmp7 / _tmp3;
  const Scalar _tmp9 = _tmp8 * rot_init_w;
  const Scalar _tmp10 = _tmp8 * rot_init_z;
  const Scalar _tmp11 = _tmp8 * rot_init_x;
  const Scalar _tmp12 = _tmp11 * rz;
  const Scalar _tmp13 = _tmp10 * rx - _tmp12 + _tmp6 + _tmp9 * ry;
  const Scalar _tmp14 = _tmp5 * rot_init_x;
  const Scalar _tmp15 = _tmp8 * rot_init_y;
  const Scalar _tmp16 = _tmp15 * rz;
  const Scalar _tmp17 = -_tmp10 * ry + _tmp14 + _tmp16 + _tmp9 * rx;
  const Scalar _tmp18 = 2 * _tmp17;
  const Scalar _tmp19 = _tmp13 * _tmp18;
  const Scalar _tmp20 = _tmp5 * rot_init_z;
  const Scalar _tmp21 = _tmp9 * rz;
  const Scalar _tmp22 = _tmp11 * ry - _tmp15 * rx + _tmp20 + _tmp21;
  const Scalar _tmp23 = _tmp5 * rot_init_w;
  const Scalar _tmp24 = _tmp10 * rz;
  const Scalar _tmp25 = -_tmp11 * rx - _tmp15 * ry + _tmp23 - _tmp24;
  const Scalar _tmp26 = 2 * _tmp22 * _tmp25;
  const Scalar _tmp27 = Scalar(0.20999999999999999) * _tmp19 + Scalar(0.20999999999999999) * _tmp26;
  const Scalar _tmp28 = -2 * std::pow(_tmp22, Scalar(2));
  const Scalar _tmp29 = 1 - 2 * std::pow(_tmp17, Scalar(2));
  const Scalar _tmp30 = Scalar(0.20999999999999999) * _tmp28 + Scalar(0.20999999999999999) * _tmp29;
  const Scalar _tmp31 = 2 * _tmp13;
  const Scalar _tmp32 = _tmp22 * _tmp31;
  const Scalar _tmp33 = _tmp18 * _tmp25;
  const Scalar _tmp34 = _tmp32 - _tmp33;
  const Scalar _tmp35 = -Scalar(0.010999999999999999) * _tmp34;
  const Scalar _tmp36 = -_tmp30 + _tmp35;
  const Scalar _tmp37 = _tmp27 + _tmp36;
  const Scalar _tmp38 = _tmp37 + p_init1;
  const Scalar _tmp39 = -_tmp38 + Scalar(-8.3888750099999996);
  const Scalar _tmp40 = -2 * std::pow(_tmp13, Scalar(2));
  const Scalar _tmp41 = Scalar(0.20999999999999999) * _tmp28 +
                        Scalar(0.20999999999999999) * _tmp40 + Scalar(0.20999999999999999);
  const Scalar _tmp42 = Scalar(0.20999999999999999) * _tmp19 - Scalar(0.20999999999999999) * _tmp26;
  const Scalar _tmp43 = _tmp18 * _tmp22;
  const Scalar _tmp44 = _tmp25 * _tmp31;
  const Scalar _tmp45 = _tmp43 + _tmp44;
  const Scalar _tmp46 = -Scalar(0.010999999999999999) * _tmp45;
  const Scalar _tmp47 = -_tmp42 + _tmp46;
  const Scalar _tmp48 = _tmp41 + _tmp47;
  const Scalar _tmp49 = _tmp48 + p_init0;
  const Scalar _tmp50 = Scalar(2.5202214700000001) - _tmp49;
  const Scalar _tmp51 =
      std::sqrt(Scalar(std::pow(_tmp39, Scalar(2)) + std::pow(_tmp50, Scalar(2))));
  const Scalar _tmp52 = (Scalar(1) / Scalar(2)) / _tmp2;
  const Scalar _tmp53 = _tmp1 * _tmp52;
  const Scalar _tmp54 = _tmp52 * rz;
  const Scalar _tmp55 = _tmp54 * ry;
  const Scalar _tmp56 = _tmp54 * rx;
  const Scalar _tmp57 = _tmp7 / (_tmp2 * std::sqrt(_tmp2));
  const Scalar _tmp58 = _tmp1 * _tmp57;
  const Scalar _tmp59 = _tmp57 * rz;
  const Scalar _tmp60 = _tmp59 * rx;
  const Scalar _tmp61 = _tmp59 * ry;
  const Scalar _tmp62 = -_tmp10 - _tmp14 * _tmp56 - _tmp20 * _tmp53 -
                        Scalar(1) / Scalar(2) * _tmp21 - _tmp55 * _tmp6 + _tmp58 * rot_init_z +
                        _tmp60 * rot_init_x + _tmp61 * rot_init_y;
  const Scalar _tmp63 = Scalar(0.41999999999999998) * _tmp62;
  const Scalar _tmp64 = _tmp22 * _tmp63;
  const Scalar _tmp65 = _tmp14 * _tmp55 + _tmp23 * _tmp53 - Scalar(1) / Scalar(2) * _tmp24 -
                        _tmp56 * _tmp6 - _tmp58 * rot_init_w + _tmp60 * rot_init_y -
                        _tmp61 * rot_init_x + _tmp9;
  const Scalar _tmp66 = Scalar(0.41999999999999998) * _tmp65;
  const Scalar _tmp67 = _tmp25 * _tmp66;
  const Scalar _tmp68 = -_tmp64 - _tmp67;
  const Scalar _tmp69 = -Scalar(1) / Scalar(2) * _tmp12 + _tmp15 - _tmp20 * _tmp55 +
                        _tmp23 * _tmp56 + _tmp53 * _tmp6 - _tmp58 * rot_init_y -
                        _tmp60 * rot_init_w + _tmp61 * rot_init_z;
  const Scalar _tmp70 = Scalar(0.41999999999999998) * _tmp69;
  const Scalar _tmp71 = _tmp13 * _tmp70;
  const Scalar _tmp72 = -_tmp11 - _tmp14 * _tmp53 - Scalar(1) / Scalar(2) * _tmp16 +
                        _tmp20 * _tmp56 + _tmp23 * _tmp55 + _tmp58 * rot_init_x -
                        _tmp60 * rot_init_z - _tmp61 * rot_init_w;
  const Scalar _tmp73 = Scalar(0.41999999999999998) * _tmp72;
  const Scalar _tmp74 = _tmp17 * _tmp73;
  const Scalar _tmp75 = -_tmp71 - _tmp74;
  const Scalar _tmp76 = _tmp68 + _tmp75;
  const Scalar _tmp77 = Scalar(0.83999999999999997) * _tmp22;
  const Scalar _tmp78 = _tmp65 * _tmp77;
  const Scalar _tmp79 = -_tmp78;
  const Scalar _tmp80 = Scalar(0.83999999999999997) * _tmp69;
  const Scalar _tmp81 = _tmp17 * _tmp80;
  const Scalar _tmp82 = _tmp79 - _tmp81;
  const Scalar _tmp83 = _tmp76 + _tmp82;
  const Scalar _tmp84 = Scalar(0.021999999999999999) * _tmp62;
  const Scalar _tmp85 = _tmp17 * _tmp84;
  const Scalar _tmp86 = Scalar(0.021999999999999999) * _tmp72;
  const Scalar _tmp87 = _tmp22 * _tmp86;
  const Scalar _tmp88 = Scalar(0.021999999999999999) * _tmp65;
  const Scalar _tmp89 = _tmp13 * _tmp88;
  const Scalar _tmp90 = Scalar(0.021999999999999999) * _tmp69;
  const Scalar _tmp91 = _tmp25 * _tmp90;
  const Scalar _tmp92 = -_tmp85 + _tmp87 + _tmp89 - _tmp91;
  const Scalar _tmp93 = Scalar(0.83999999999999997) * _tmp72;
  const Scalar _tmp94 = _tmp13 * _tmp93;
  const Scalar _tmp95 = _tmp78 + _tmp94;
  const Scalar _tmp96 = _tmp71 + _tmp74;
  const Scalar _tmp97 = _tmp68 + _tmp96;
  const Scalar _tmp98 = _tmp13 * _tmp84;
  const Scalar _tmp99 = _tmp22 * _tmp90;
  const Scalar _tmp100 = _tmp17 * _tmp88;
  const Scalar _tmp101 = _tmp25 * _tmp86;
  const Scalar _tmp102 = _tmp100 + _tmp101 + _tmp98 + _tmp99;
  const Scalar _tmp103 = _tmp102 + _tmp97;
  const Scalar _tmp104 = _tmp17 * _tmp63;
  const Scalar _tmp105 = _tmp22 * _tmp73;
  const Scalar _tmp106 = _tmp13 * _tmp66;
  const Scalar _tmp107 = _tmp25 * _tmp70;
  const Scalar _tmp108 = -_tmp104 - _tmp105 - _tmp106 - _tmp107;
  const Scalar _tmp109 = _tmp13 * _tmp63;
  const Scalar _tmp110 = _tmp22 * _tmp70;
  const Scalar _tmp111 = _tmp17 * _tmp66;
  const Scalar _tmp112 = _tmp25 * _tmp73;
  const Scalar _tmp113 = _tmp17 * _tmp69;
  const Scalar _tmp114 = Scalar(0.043999999999999997) * _tmp113;
  const Scalar _tmp115 = _tmp13 * _tmp72;
  const Scalar _tmp116 = Scalar(0.043999999999999997) * _tmp115;
  const Scalar _tmp117 = _tmp114 + _tmp116;
  const Scalar _tmp118 = -_tmp109 + _tmp110 + _tmp111 - _tmp112 + _tmp117;
  const Scalar _tmp119 = _tmp108 + _tmp118;
  const Scalar _tmp120 = _tmp30 + _tmp35;
  const Scalar _tmp121 = _tmp120 + _tmp27;
  const Scalar _tmp122 = Scalar(1.0) * _tmp121;
  const Scalar _tmp123 = -_tmp122;
  const Scalar _tmp124 = -_tmp27;
  const Scalar _tmp125 = _tmp124 + _tmp36;
  const Scalar _tmp126 = _tmp123 + _tmp125;
  const Scalar _tmp127 = Scalar(1.0) / (_tmp126);
  const Scalar _tmp128 = -_tmp41;
  const Scalar _tmp129 = _tmp128 + _tmp47;
  const Scalar _tmp130 = _tmp42 + _tmp46;
  const Scalar _tmp131 = _tmp130 + _tmp41;
  const Scalar _tmp132 = Scalar(1.0) * _tmp131;
  const Scalar _tmp133 = -_tmp129 + _tmp132;
  const Scalar _tmp134 = _tmp127 * _tmp133;
  const Scalar _tmp135 = _tmp122 * _tmp134 + _tmp132;
  const Scalar _tmp136 = _tmp128 + _tmp130;
  const Scalar _tmp137 = _tmp120 + _tmp124;
  const Scalar _tmp138 = _tmp123 + _tmp137;
  const Scalar _tmp139 = _tmp134 * _tmp138;
  const Scalar _tmp140 = _tmp132 - _tmp136 - _tmp139;
  const Scalar _tmp141 = Scalar(1.0) / (_tmp140);
  const Scalar _tmp142 = Scalar(1.0) * _tmp141;
  const Scalar _tmp143 = _tmp138 * _tmp141;
  const Scalar _tmp144 =
      Scalar(0.20999999999999999) * _tmp43 - Scalar(0.20999999999999999) * _tmp44;
  const Scalar _tmp145 = -_tmp144;
  const Scalar _tmp146 =
      -Scalar(0.010999999999999999) * _tmp29 - Scalar(0.010999999999999999) * _tmp40;
  const Scalar _tmp147 =
      Scalar(0.20999999999999999) * _tmp32 + Scalar(0.20999999999999999) * _tmp33;
  const Scalar _tmp148 = _tmp146 + _tmp147;
  const Scalar _tmp149 = _tmp145 + _tmp148;
  const Scalar _tmp150 = _tmp137 + p_init1;
  const Scalar _tmp151 = _tmp150 + Scalar(-4.8333311099999996);
  const Scalar _tmp152 = _tmp136 + p_init0;
  const Scalar _tmp153 = _tmp152 + Scalar(1.79662371);
  const Scalar _tmp154 = std::pow(_tmp151, Scalar(2)) + std::pow(_tmp153, Scalar(2));
  const Scalar _tmp155 = std::pow(_tmp154, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp156 = _tmp151 * _tmp155;
  const Scalar _tmp157 = _tmp131 + p_init0;
  const Scalar _tmp158 = _tmp157 + Scalar(-2.71799795);
  const Scalar _tmp159 = Scalar(1.0) / (_tmp158);
  const Scalar _tmp160 = _tmp121 + p_init1;
  const Scalar _tmp161 = _tmp160 + Scalar(-4.7752063900000001);
  const Scalar _tmp162 = _tmp159 * _tmp161;
  const Scalar _tmp163 = _tmp144 + _tmp148;
  const Scalar _tmp164 = _tmp153 * _tmp155;
  const Scalar _tmp165 = _tmp163 * _tmp164;
  const Scalar _tmp166 = _tmp125 + p_init1;
  const Scalar _tmp167 = _tmp166 + Scalar(8.3196563700000006);
  const Scalar _tmp168 = _tmp129 + p_init0;
  const Scalar _tmp169 = _tmp168 + Scalar(1.9874742000000001);
  const Scalar _tmp170 = std::pow(_tmp167, Scalar(2)) + std::pow(_tmp169, Scalar(2));
  const Scalar _tmp171 = std::pow(_tmp170, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp172 = _tmp169 * _tmp171;
  const Scalar _tmp173 = _tmp163 * _tmp172;
  const Scalar _tmp174 = _tmp146 - _tmp147;
  const Scalar _tmp175 = _tmp145 + _tmp174;
  const Scalar _tmp176 = _tmp167 * _tmp171;
  const Scalar _tmp177 = -_tmp162 * _tmp173 + _tmp175 * _tmp176;
  const Scalar _tmp178 = _tmp162 * _tmp172 - _tmp176;
  const Scalar _tmp179 = Scalar(1.0) / (_tmp178);
  const Scalar _tmp180 = -_tmp156 + _tmp162 * _tmp164;
  const Scalar _tmp181 = _tmp179 * _tmp180;
  const Scalar _tmp182 = _tmp149 * _tmp156 - _tmp162 * _tmp165 - _tmp177 * _tmp181;
  const Scalar _tmp183 = 0;
  const Scalar _tmp184 = -_tmp172 * _tmp175 + _tmp173;
  const Scalar _tmp185 = -_tmp134 * _tmp182 - _tmp149 * _tmp164 + _tmp165 - _tmp181 * _tmp184;
  const Scalar _tmp186 = Scalar(1.0) / (_tmp185);
  const Scalar _tmp187 = _tmp183 * _tmp186;
  const Scalar _tmp188 = _tmp123 - _tmp135 * _tmp143 - _tmp182 * _tmp187;
  const Scalar _tmp189 = _tmp127 * _tmp188;
  const Scalar _tmp190 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp191 = Scalar(1.0) * _tmp190;
  const Scalar _tmp192 = _tmp38 + Scalar(8.3888750099999996);
  const Scalar _tmp193 = _tmp49 + Scalar(-2.5202214700000001);
  const Scalar _tmp194 = std::pow(_tmp192, Scalar(2)) + std::pow(_tmp193, Scalar(2));
  const Scalar _tmp195 = std::pow(_tmp194, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp196 = fh1 * (_tmp144 + _tmp174);
  const Scalar _tmp197 = _tmp195 * _tmp196;
  const Scalar _tmp198 = -_tmp192 * _tmp197 - Scalar(3.29616) * _tmp34 - _tmp37 * fv1;
  const Scalar _tmp199 = _tmp139 * _tmp142 + Scalar(1.0);
  const Scalar _tmp200 = _tmp127 * _tmp199;
  const Scalar _tmp201 = _tmp134 * _tmp142;
  const Scalar _tmp202 = -Scalar(1.0) * _tmp200 + Scalar(1.0) * _tmp201;
  const Scalar _tmp203 = _tmp127 * _tmp142;
  const Scalar _tmp204 = _tmp138 * _tmp203 - _tmp142;
  const Scalar _tmp205 = _tmp193 * _tmp195;
  const Scalar _tmp206 = _tmp196 * _tmp205 + Scalar(3.29616) * _tmp45 + _tmp48 * fv1;
  const Scalar _tmp207 = Scalar(1.0) * _tmp206;
  const Scalar _tmp208 = _tmp192 * _tmp195;
  const Scalar _tmp209 = _tmp208 * fh1;
  const Scalar _tmp210 = Scalar(1.0) * _tmp179;
  const Scalar _tmp211 = _tmp177 * _tmp210;
  const Scalar _tmp212 = _tmp134 * _tmp211 - _tmp184 * _tmp210;
  const Scalar _tmp213 = std::pow(_tmp158, Scalar(2));
  const Scalar _tmp214 = std::pow(_tmp161, Scalar(2)) + _tmp213;
  const Scalar _tmp215 = std::sqrt(_tmp214);
  const Scalar _tmp216 = Scalar(1.0) / (_tmp215);
  const Scalar _tmp217 = _tmp161 * _tmp216;
  const Scalar _tmp218 = _tmp158 * _tmp216;
  const Scalar _tmp219 = -_tmp121 * _tmp218 + _tmp131 * _tmp217;
  const Scalar _tmp220 = _tmp159 * _tmp215;
  const Scalar _tmp221 = _tmp219 * _tmp220;
  const Scalar _tmp222 = _tmp125 * _tmp172 - _tmp129 * _tmp176 + _tmp172 * _tmp221;
  const Scalar _tmp223 =
      -_tmp136 * _tmp156 + _tmp137 * _tmp164 + _tmp164 * _tmp221 - _tmp181 * _tmp222;
  const Scalar _tmp224 = _tmp186 * _tmp223;
  const Scalar _tmp225 = -_tmp210 * _tmp222 - _tmp212 * _tmp224;
  const Scalar _tmp226 = Scalar(1.0) / (_tmp223);
  const Scalar _tmp227 = _tmp185 * _tmp226;
  const Scalar _tmp228 = _tmp225 * _tmp227;
  const Scalar _tmp229 = _tmp212 + _tmp228;
  const Scalar _tmp230 = _tmp186 * _tmp229;
  const Scalar _tmp231 = _tmp143 * _tmp228 - _tmp182 * _tmp230 - _tmp211;
  const Scalar _tmp232 = Scalar(1.0) * _tmp127;
  const Scalar _tmp233 = _tmp142 * _tmp228 - _tmp231 * _tmp232;
  const Scalar _tmp234 = Scalar(1.0) * _tmp233;
  const Scalar _tmp235 = _tmp142 * _tmp227;
  const Scalar _tmp236 = Scalar(1.0) * _tmp226;
  const Scalar _tmp237 = _tmp138 * _tmp235 - _tmp182 * _tmp236;
  const Scalar _tmp238 = -_tmp232 * _tmp237 + _tmp235;
  const Scalar _tmp239 = _tmp195 * _tmp48;
  const Scalar _tmp240 = fh1 * (_tmp192 * _tmp239 - _tmp205 * _tmp37);
  const Scalar _tmp241 = Scalar(1.0) * _tmp240;
  const Scalar _tmp242 = _tmp162 * _tmp179;
  const Scalar _tmp243 = _tmp162 * _tmp163;
  const Scalar _tmp244 = _tmp177 * _tmp242 + _tmp243;
  const Scalar _tmp245 = -_tmp134 * _tmp244 - _tmp163 + _tmp184 * _tmp242;
  const Scalar _tmp246 = -_tmp221 + _tmp222 * _tmp242 - _tmp224 * _tmp245;
  const Scalar _tmp247 = _tmp227 * _tmp246;
  const Scalar _tmp248 = _tmp245 + _tmp247;
  const Scalar _tmp249 = _tmp182 * _tmp186;
  const Scalar _tmp250 = _tmp143 * _tmp247 + _tmp244 - _tmp248 * _tmp249;
  const Scalar _tmp251 = _tmp142 * _tmp247 - _tmp232 * _tmp250;
  const Scalar _tmp252 = Scalar(1.0) * _tmp251;
  const Scalar _tmp253 = _tmp205 * fh1;
  const Scalar _tmp254 = _tmp191 * (-_tmp135 * _tmp142 - Scalar(1.0) * _tmp189 + Scalar(1.0)) +
                         _tmp198 * _tmp202 + _tmp204 * _tmp207 + _tmp209 * _tmp234 +
                         _tmp238 * _tmp241 + _tmp252 * _tmp253;
  const Scalar _tmp255 = -_tmp180 * _tmp230 + Scalar(1.0);
  const Scalar _tmp256 = _tmp179 * _tmp255;
  const Scalar _tmp257 = _tmp164 * _tmp230 + _tmp172 * _tmp256;
  const Scalar _tmp258 = _tmp220 * _tmp257;
  const Scalar _tmp259 = _tmp186 * _tmp248;
  const Scalar _tmp260 = -_tmp162 - _tmp180 * _tmp259;
  const Scalar _tmp261 = _tmp179 * _tmp260;
  const Scalar _tmp262 = _tmp164 * _tmp259 + _tmp172 * _tmp261 + Scalar(1.0);
  const Scalar _tmp263 = _tmp220 * _tmp253;
  const Scalar _tmp264 = _tmp181 * _tmp187;
  const Scalar _tmp265 = _tmp164 * _tmp187 - _tmp172 * _tmp264;
  const Scalar _tmp266 = _tmp190 * _tmp220;
  const Scalar _tmp267 = _tmp181 * _tmp236;
  const Scalar _tmp268 = _tmp164 * _tmp236 - _tmp172 * _tmp267;
  const Scalar _tmp269 = _tmp220 * _tmp240;
  const Scalar _tmp270 =
      -_tmp209 * _tmp258 - _tmp262 * _tmp263 - _tmp265 * _tmp266 - _tmp268 * _tmp269;
  const Scalar _tmp271 = Scalar(1.0) / (_tmp270);
  const Scalar _tmp272 = std::asinh(_tmp254 * _tmp271);
  const Scalar _tmp273 = Scalar(1.0) * _tmp272;
  const Scalar _tmp274 = Scalar(4.7752063900000001) - _tmp160;
  const Scalar _tmp275 = Scalar(2.71799795) - _tmp157;
  const Scalar _tmp276 =
      std::sqrt(Scalar(std::pow(_tmp274, Scalar(2)) + std::pow(_tmp275, Scalar(2))));
  const Scalar _tmp277 = Scalar(9.6622558468725703) * _tmp270;
  const Scalar _tmp278 = -_tmp272 * _tmp277 - _tmp276;
  const Scalar _tmp279 = Scalar(0.1034955) * _tmp271;
  const Scalar _tmp280 = _tmp278 * _tmp279;
  const Scalar _tmp281 = _tmp79 - _tmp94;
  const Scalar _tmp282 = -_tmp100 - _tmp101 - _tmp98 - _tmp99;
  const Scalar _tmp283 = _tmp281 + _tmp282;
  const Scalar _tmp284 = _tmp283 + _tmp97;
  const Scalar _tmp285 = _tmp284 / _tmp213;
  const Scalar _tmp286 = _tmp215 * _tmp285;
  const Scalar _tmp287 = _tmp253 * _tmp262;
  const Scalar _tmp288 = _tmp190 * _tmp265;
  const Scalar _tmp289 = _tmp85 - _tmp87 - _tmp89 + _tmp91;
  const Scalar _tmp290 = _tmp64 + _tmp67;
  const Scalar _tmp291 = _tmp290 + _tmp96;
  const Scalar _tmp292 = _tmp289 + _tmp291;
  const Scalar _tmp293 = _tmp292 + _tmp82;
  const Scalar _tmp294 = _tmp158 * _tmp284 + _tmp161 * _tmp293;
  const Scalar _tmp295 = _tmp159 * _tmp216 * _tmp294;
  const Scalar _tmp296 = _tmp290 + _tmp75;
  const Scalar _tmp297 = _tmp283 + _tmp296;
  const Scalar _tmp298 = _tmp195 * _tmp297;
  const Scalar _tmp299 = _tmp298 * fh1;
  const Scalar _tmp300 = _tmp220 * _tmp262;
  const Scalar _tmp301 = _tmp209 * _tmp257;
  const Scalar _tmp302 = _tmp78 + _tmp81;
  const Scalar _tmp303 = _tmp302 + _tmp76;
  const Scalar _tmp304 = _tmp289 + _tmp303;
  const Scalar _tmp305 = _tmp282 + _tmp95;
  const Scalar _tmp306 = _tmp296 + _tmp305;
  const Scalar _tmp307 =
      (2 * _tmp167 * _tmp304 + 2 * _tmp169 * _tmp306) / (_tmp170 * std::sqrt(_tmp170));
  const Scalar _tmp308 = (Scalar(1) / Scalar(2)) * _tmp307;
  const Scalar _tmp309 = _tmp167 * _tmp308;
  const Scalar _tmp310 = _tmp171 * _tmp304;
  const Scalar _tmp311 = _tmp159 * _tmp293;
  const Scalar _tmp312 = _tmp169 * _tmp308;
  const Scalar _tmp313 = _tmp171 * _tmp306;
  const Scalar _tmp314 = _tmp161 * _tmp285;
  const Scalar _tmp315 = (-_tmp162 * _tmp312 + _tmp162 * _tmp313 + _tmp172 * _tmp311 -
                          _tmp172 * _tmp314 + _tmp309 - _tmp310) /
                         std::pow(_tmp178, Scalar(2));
  const Scalar _tmp316 = _tmp180 * _tmp315;
  const Scalar _tmp317 = _tmp305 + _tmp97;
  const Scalar _tmp318 = _tmp289 + _tmp83;
  const Scalar _tmp319 =
      (2 * _tmp151 * _tmp318 + 2 * _tmp153 * _tmp317) / (_tmp154 * std::sqrt(_tmp154));
  const Scalar _tmp320 = (Scalar(1) / Scalar(2)) * _tmp319;
  const Scalar _tmp321 = _tmp151 * _tmp320;
  const Scalar _tmp322 = _tmp155 * _tmp318;
  const Scalar _tmp323 = _tmp153 * _tmp320;
  const Scalar _tmp324 = _tmp155 * _tmp317;
  const Scalar _tmp325 = -_tmp162 * _tmp323 + _tmp162 * _tmp324 + _tmp164 * _tmp311 -
                         _tmp164 * _tmp314 + _tmp321 - _tmp322;
  const Scalar _tmp326 = _tmp179 * _tmp325;
  const Scalar _tmp327 = _tmp104 + _tmp105 + _tmp106 + _tmp107;
  const Scalar _tmp328 = _tmp118 + _tmp327;
  const Scalar _tmp329 = _tmp162 * _tmp328;
  const Scalar _tmp330 = _tmp163 * _tmp313;
  const Scalar _tmp331 = _tmp109 - _tmp110 - _tmp111 + _tmp112;
  const Scalar _tmp332 = _tmp117 + _tmp331;
  const Scalar _tmp333 = _tmp108 + _tmp332;
  const Scalar _tmp334 = -_tmp162 * _tmp330 - _tmp172 * _tmp329 - _tmp173 * _tmp311 +
                         _tmp173 * _tmp314 - _tmp175 * _tmp309 + _tmp175 * _tmp310 +
                         _tmp176 * _tmp333 + _tmp243 * _tmp312;
  const Scalar _tmp335 = _tmp163 * _tmp324;
  const Scalar _tmp336 = _tmp164 * _tmp328;
  const Scalar _tmp337 = _tmp327 + _tmp332;
  const Scalar _tmp338 = -_tmp149 * _tmp321 + _tmp149 * _tmp322 + _tmp156 * _tmp337 -
                         _tmp162 * _tmp335 - _tmp162 * _tmp336 - _tmp165 * _tmp311 +
                         _tmp165 * _tmp314 + _tmp177 * _tmp316 - _tmp177 * _tmp326 -
                         _tmp181 * _tmp334 + _tmp243 * _tmp323;
  const Scalar _tmp339 = Scalar(1.6799999999999999) * _tmp22 * _tmp65;
  const Scalar _tmp340 = _tmp13 * _tmp80;
  const Scalar _tmp341 = _tmp17 * _tmp93;
  const Scalar _tmp342 = -Scalar(0.83999999999999997) * _tmp25 * _tmp65 - _tmp62 * _tmp77;
  const Scalar _tmp343 = -_tmp340 - _tmp341 + _tmp342;
  const Scalar _tmp344 =
      (Scalar(1.6799999999999999) * _tmp113 + _tmp339 + _tmp343) / std::pow(_tmp126, Scalar(2));
  const Scalar _tmp345 = _tmp133 * _tmp344;
  const Scalar _tmp346 = -_tmp163 * _tmp312 + _tmp172 * _tmp328 - _tmp172 * _tmp333 +
                         _tmp175 * _tmp312 - _tmp175 * _tmp313 + _tmp330;
  const Scalar _tmp347 = -Scalar(1.6799999999999999) * _tmp115 - _tmp339;
  const Scalar _tmp348 = _tmp127 * (_tmp340 + _tmp341 + _tmp342 + _tmp347);
  const Scalar _tmp349 = -_tmp134 * _tmp338 + _tmp149 * _tmp323 - _tmp149 * _tmp324 -
                         _tmp163 * _tmp323 - _tmp164 * _tmp337 - _tmp181 * _tmp346 +
                         _tmp182 * _tmp345 - _tmp182 * _tmp348 + _tmp184 * _tmp316 -
                         _tmp184 * _tmp326 + _tmp335 + _tmp336;
  const Scalar _tmp350 = _tmp349 / std::pow(_tmp185, Scalar(2));
  const Scalar _tmp351 = _tmp183 * _tmp350;
  const Scalar _tmp352 = _tmp172 * _tmp181;
  const Scalar _tmp353 = _tmp172 * _tmp316;
  const Scalar _tmp354 = _tmp172 * _tmp326;
  const Scalar _tmp355 = _tmp172 * _tmp315;
  const Scalar _tmp356 = _tmp229 * _tmp350;
  const Scalar _tmp357 = _tmp219 * _tmp286;
  const Scalar _tmp358 = _tmp219 * _tmp295;
  const Scalar _tmp359 = _tmp294 / (_tmp214 * std::sqrt(_tmp214));
  const Scalar _tmp360 = _tmp220 * (_tmp121 * _tmp158 * _tmp359 - _tmp121 * _tmp216 * _tmp284 -
                                    _tmp131 * _tmp161 * _tmp359 + _tmp131 * _tmp216 * _tmp293 +
                                    _tmp217 * _tmp284 - _tmp218 * _tmp293);
  const Scalar _tmp361 = -_tmp125 * _tmp312 + _tmp125 * _tmp313 + _tmp129 * _tmp309 -
                         _tmp129 * _tmp310 + _tmp169 * _tmp310 - _tmp172 * _tmp357 +
                         _tmp172 * _tmp358 + _tmp172 * _tmp360 - _tmp176 * _tmp306 -
                         _tmp221 * _tmp312 + _tmp221 * _tmp313;
  const Scalar _tmp362 = _tmp136 * _tmp321 - _tmp136 * _tmp322 - _tmp137 * _tmp323 +
                         _tmp137 * _tmp324 + _tmp153 * _tmp322 - _tmp156 * _tmp317 -
                         _tmp164 * _tmp357 + _tmp164 * _tmp358 + _tmp164 * _tmp360 -
                         _tmp181 * _tmp361 - _tmp221 * _tmp323 + _tmp221 * _tmp324 +
                         _tmp222 * _tmp316 - _tmp222 * _tmp326;
  const Scalar _tmp363 = _tmp362 / std::pow(_tmp223, Scalar(2));
  const Scalar _tmp364 = _tmp185 * _tmp363;
  const Scalar _tmp365 = _tmp225 * _tmp364;
  const Scalar _tmp366 = Scalar(1.0) * _tmp315;
  const Scalar _tmp367 = _tmp210 * _tmp334;
  const Scalar _tmp368 = _tmp177 * _tmp366;
  const Scalar _tmp369 = _tmp134 * _tmp367 - _tmp134 * _tmp368 + _tmp184 * _tmp366 -
                         _tmp210 * _tmp346 - _tmp211 * _tmp345 + _tmp211 * _tmp348;
  const Scalar _tmp370 = _tmp186 * _tmp362;
  const Scalar _tmp371 = _tmp223 * _tmp350;
  const Scalar _tmp372 = _tmp227 * (-_tmp210 * _tmp361 - _tmp212 * _tmp370 + _tmp212 * _tmp371 +
                                    _tmp222 * _tmp366 - _tmp224 * _tmp369);
  const Scalar _tmp373 = _tmp226 * _tmp349;
  const Scalar _tmp374 = _tmp225 * _tmp373;
  const Scalar _tmp375 = _tmp186 * (-_tmp365 + _tmp369 + _tmp372 + _tmp374);
  const Scalar _tmp376 = _tmp180 * _tmp356 - _tmp180 * _tmp375 - _tmp230 * _tmp325;
  const Scalar _tmp377 = _tmp172 * _tmp179;
  const Scalar _tmp378 = _tmp240 * _tmp268;
  const Scalar _tmp379 = _tmp185 * _tmp246;
  const Scalar _tmp380 = _tmp363 * _tmp379;
  const Scalar _tmp381 = _tmp179 * _tmp314;
  const Scalar _tmp382 = _tmp162 * _tmp315;
  const Scalar _tmp383 = _tmp179 * _tmp311;
  const Scalar _tmp384 = _tmp163 * _tmp311 - _tmp163 * _tmp314 - _tmp177 * _tmp381 -
                         _tmp177 * _tmp382 + _tmp177 * _tmp383 + _tmp242 * _tmp334 + _tmp329;
  const Scalar _tmp385 = _tmp108 - _tmp114 - _tmp116 - _tmp134 * _tmp384 - _tmp184 * _tmp381 -
                         _tmp184 * _tmp382 + _tmp184 * _tmp383 + _tmp242 * _tmp346 +
                         _tmp244 * _tmp345 - _tmp244 * _tmp348 + _tmp331;
  const Scalar _tmp386 = _tmp227 * (-_tmp222 * _tmp381 - _tmp222 * _tmp382 + _tmp222 * _tmp383 -
                                    _tmp224 * _tmp385 + _tmp242 * _tmp361 - _tmp245 * _tmp370 +
                                    _tmp245 * _tmp371 + _tmp357 - _tmp358 - _tmp360);
  const Scalar _tmp387 = _tmp246 * _tmp373;
  const Scalar _tmp388 = -_tmp380 + _tmp385 + _tmp386 + _tmp387;
  const Scalar _tmp389 = _tmp186 * _tmp388;
  const Scalar _tmp390 = _tmp248 * _tmp350;
  const Scalar _tmp391 =
      -_tmp180 * _tmp389 + _tmp180 * _tmp390 - _tmp259 * _tmp325 - _tmp311 + _tmp314;
  const Scalar _tmp392 = _tmp292 + _tmp302;
  const Scalar _tmp393 = _tmp195 * _tmp392 * fh1;
  const Scalar _tmp394 =
      (2 * _tmp192 * _tmp392 + 2 * _tmp193 * _tmp297) / (_tmp194 * std::sqrt(_tmp194));
  const Scalar _tmp395 = (Scalar(1) / Scalar(2)) * _tmp394;
  const Scalar _tmp396 = _tmp192 * _tmp395;
  const Scalar _tmp397 = _tmp193 * _tmp395;
  const Scalar _tmp398 = fh1 * (_tmp192 * _tmp298 - _tmp205 * _tmp392 + _tmp239 * _tmp392 -
                                _tmp298 * _tmp37 + _tmp37 * _tmp397 - _tmp396 * _tmp48);
  const Scalar _tmp399 = _tmp397 * fh1;
  const Scalar _tmp400 = Scalar(1.0) * _tmp363;
  const Scalar _tmp401 = Scalar(0.5) * _tmp226;
  const Scalar _tmp402 = _tmp396 * fh1;
  const Scalar _tmp403 =
      -_tmp209 * _tmp220 *
          (-_tmp164 * _tmp356 + _tmp164 * _tmp375 - _tmp230 * _tmp323 + _tmp230 * _tmp324 -
           _tmp255 * _tmp355 - _tmp256 * _tmp312 + _tmp256 * _tmp313 + _tmp376 * _tmp377) -
      _tmp220 * _tmp268 * _tmp398 - _tmp258 * _tmp393 + _tmp258 * _tmp402 -
      _tmp263 * (_tmp164 * _tmp389 - _tmp164 * _tmp390 - _tmp259 * _tmp323 + _tmp259 * _tmp324 -
                 _tmp260 * _tmp355 - _tmp261 * _tmp312 + _tmp261 * _tmp313 + _tmp377 * _tmp391) -
      _tmp266 * (-_tmp164 * _tmp351 - _tmp187 * _tmp323 + _tmp187 * _tmp324 + _tmp187 * _tmp353 -
                 _tmp187 * _tmp354 + _tmp264 * _tmp312 - _tmp264 * _tmp313 + _tmp351 * _tmp352) -
      _tmp269 * (-_tmp153 * _tmp319 * _tmp401 - _tmp164 * _tmp400 +
                 _tmp169 * _tmp181 * _tmp307 * _tmp401 + _tmp236 * _tmp324 + _tmp236 * _tmp353 -
                 _tmp236 * _tmp354 - _tmp267 * _tmp313 + _tmp352 * _tmp400) +
      _tmp286 * _tmp287 + _tmp286 * _tmp288 + _tmp286 * _tmp301 + _tmp286 * _tmp378 -
      _tmp287 * _tmp295 - _tmp288 * _tmp295 - _tmp295 * _tmp301 - _tmp295 * _tmp378 -
      _tmp299 * _tmp300 + _tmp300 * _tmp399;
  const Scalar _tmp404 = Scalar(9.6622558468725703) * _tmp403;
  const Scalar _tmp405 = std::pow(_tmp270, Scalar(-2));
  const Scalar _tmp406 = _tmp403 * _tmp405;
  const Scalar _tmp407 = _tmp102 + _tmp296;
  const Scalar _tmp408 = _tmp303 + _tmp92;
  const Scalar _tmp409 = Scalar(1.0) * _tmp344;
  const Scalar _tmp410 = _tmp142 * _tmp345;
  const Scalar _tmp411 = _tmp134 * _tmp343;
  const Scalar _tmp412 = _tmp138 * _tmp345;
  const Scalar _tmp413 = _tmp138 * _tmp348;
  const Scalar _tmp414 = (_tmp347 - _tmp411 + _tmp412 - _tmp413) / std::pow(_tmp140, Scalar(2));
  const Scalar _tmp415 = Scalar(1.0) * _tmp414;
  const Scalar _tmp416 = _tmp134 * _tmp415;
  const Scalar _tmp417 =
      -_tmp139 * _tmp415 + _tmp142 * _tmp411 - _tmp142 * _tmp412 + _tmp142 * _tmp413;
  const Scalar _tmp418 = _tmp142 * _tmp348;
  const Scalar _tmp419 = Scalar(6.59232) * _tmp62;
  const Scalar _tmp420 = Scalar(6.59232) * _tmp22;
  const Scalar _tmp421 = Scalar(6.59232) * _tmp65;
  const Scalar _tmp422 = _tmp119 * fh1;
  const Scalar _tmp423 = Scalar(6.59232) * _tmp25;
  const Scalar _tmp424 = -_tmp13 * _tmp421 + _tmp17 * _tmp419 + _tmp196 * _tmp396 -
                         _tmp197 * _tmp392 - _tmp208 * _tmp422 - _tmp392 * fv1 - _tmp420 * _tmp72 +
                         _tmp423 * _tmp69;
  const Scalar _tmp425 = Scalar(0.5) * _tmp394 * fh1;
  const Scalar _tmp426 = _tmp138 * _tmp414;
  const Scalar _tmp427 = _tmp138 * _tmp344;
  const Scalar _tmp428 = _tmp185 * _tmp236;
  const Scalar _tmp429 = _tmp414 * _tmp428;
  const Scalar _tmp430 = _tmp142 * _tmp373;
  const Scalar _tmp431 = _tmp142 * _tmp364;
  const Scalar _tmp432 = _tmp138 * _tmp430 - _tmp138 * _tmp431 + _tmp182 * _tmp400 +
                         _tmp235 * _tmp343 - _tmp236 * _tmp338 - _tmp426 * _tmp428;
  const Scalar _tmp433 = _tmp141 * _tmp343;
  const Scalar _tmp434 = -_tmp143 * _tmp380 + _tmp143 * _tmp386 + _tmp143 * _tmp387 +
                         _tmp182 * _tmp390 - _tmp247 * _tmp426 + _tmp247 * _tmp433 -
                         _tmp249 * _tmp388 - _tmp259 * _tmp338 + _tmp384;
  const Scalar _tmp435 = _tmp135 * _tmp414;
  const Scalar _tmp436 =
      -_tmp122 * _tmp345 + _tmp122 * _tmp348 + Scalar(1.0) * _tmp134 * _tmp293 + _tmp284;
  const Scalar _tmp437 = -_tmp135 * _tmp433 + _tmp138 * _tmp435 - _tmp143 * _tmp436 +
                         _tmp182 * _tmp351 - _tmp187 * _tmp338 + _tmp408;
  const Scalar _tmp438 = _tmp13 * _tmp419 + _tmp17 * _tmp421 + _tmp196 * _tmp298 -
                         _tmp196 * _tmp397 + _tmp205 * _tmp422 + _tmp297 * fv1 + _tmp420 * _tmp69 +
                         _tmp423 * _tmp72;
  const Scalar _tmp439 = -_tmp143 * _tmp365 + _tmp143 * _tmp372 + _tmp143 * _tmp374 +
                         _tmp182 * _tmp356 - _tmp182 * _tmp375 - _tmp228 * _tmp426 +
                         _tmp228 * _tmp433 - _tmp230 * _tmp338 - _tmp367 + _tmp368;
  const Scalar _tmp440 =
      (-_tmp254 * _tmp406 +
       _tmp271 * (_tmp191 * (_tmp135 * _tmp415 - _tmp142 * _tmp436 + _tmp188 * _tmp409 -
                             _tmp232 * _tmp437) -
                  _tmp192 * _tmp233 * _tmp425 - _tmp193 * _tmp251 * _tmp425 +
                  Scalar(1.0) * _tmp198 *
                      (_tmp199 * _tmp409 - _tmp232 * _tmp417 - _tmp410 - _tmp416 + _tmp418) +
                  _tmp202 * _tmp424 + Scalar(1.0) * _tmp204 * _tmp438 +
                  _tmp207 * (-_tmp142 * _tmp427 + _tmp203 * _tmp343 - _tmp232 * _tmp426 + _tmp415) +
                  Scalar(1.0) * _tmp209 *
                      (-_tmp142 * _tmp365 + _tmp142 * _tmp372 + _tmp142 * _tmp374 -
                       _tmp225 * _tmp429 + _tmp231 * _tmp409 - _tmp232 * _tmp439) +
                  _tmp234 * _tmp393 + Scalar(1.0) * _tmp238 * _tmp398 +
                  _tmp241 * (-_tmp232 * _tmp432 + _tmp237 * _tmp409 - _tmp429 + _tmp430 - _tmp431) +
                  _tmp252 * _tmp299 +
                  Scalar(1.0) * _tmp253 *
                      (-_tmp142 * _tmp380 + _tmp142 * _tmp386 + _tmp142 * _tmp387 -
                       _tmp232 * _tmp434 - _tmp236 * _tmp379 * _tmp414 + _tmp250 * _tmp409))) /
      std::sqrt(Scalar(std::pow(_tmp254, Scalar(2)) * _tmp405 + 1));
  const Scalar _tmp441 = _tmp187 * _tmp190;
  const Scalar _tmp442 = _tmp236 * _tmp240;
  const Scalar _tmp443 =
      -_tmp181 * _tmp441 - _tmp181 * _tmp442 + _tmp209 * _tmp256 + _tmp253 * _tmp261;
  const Scalar _tmp444 = Scalar(1.0) / (_tmp443);
  const Scalar _tmp445 = _tmp127 * _tmp250;
  const Scalar _tmp446 = _tmp127 * _tmp240;
  const Scalar _tmp447 = _tmp142 * _tmp206;
  const Scalar _tmp448 = _tmp127 * _tmp447;
  const Scalar _tmp449 = _tmp127 * _tmp231;
  const Scalar _tmp450 = -_tmp138 * _tmp448 + _tmp189 * _tmp190 + _tmp198 * _tmp200 +
                         _tmp209 * _tmp449 + _tmp237 * _tmp446 + _tmp253 * _tmp445;
  const Scalar _tmp451 = std::asinh(_tmp444 * _tmp450);
  const Scalar _tmp452 = Scalar(1.0) * _tmp451;
  const Scalar _tmp453 = -_tmp166 + Scalar(-8.3196563700000006);
  const Scalar _tmp454 = -_tmp168 + Scalar(-1.9874742000000001);
  const Scalar _tmp455 =
      std::sqrt(Scalar(std::pow(_tmp453, Scalar(2)) + std::pow(_tmp454, Scalar(2))));
  const Scalar _tmp456 = Scalar(9.6622558468725703) * _tmp451;
  const Scalar _tmp457 = -_tmp443 * _tmp456 - _tmp455;
  const Scalar _tmp458 = Scalar(0.1034955) * _tmp444;
  const Scalar _tmp459 = _tmp457 * _tmp458;
  const Scalar _tmp460 = _tmp236 * _tmp398;
  const Scalar _tmp461 = _tmp241 * _tmp363;
  const Scalar _tmp462 = _tmp190 * _tmp351;
  const Scalar _tmp463 = _tmp179 * _tmp209 * _tmp376 + _tmp179 * _tmp253 * _tmp391 -
                         _tmp181 * _tmp460 + _tmp181 * _tmp461 + _tmp181 * _tmp462 -
                         _tmp209 * _tmp255 * _tmp315 - _tmp253 * _tmp260 * _tmp315 +
                         _tmp256 * _tmp393 - _tmp256 * _tmp402 + _tmp261 * _tmp299 -
                         _tmp261 * _tmp399 + _tmp316 * _tmp441 + _tmp316 * _tmp442 -
                         _tmp326 * _tmp441 - _tmp326 * _tmp442;
  const Scalar _tmp464 = Scalar(9.6622558468725703) * _tmp443;
  const Scalar _tmp465 = std::pow(_tmp443, Scalar(-2));
  const Scalar _tmp466 = _tmp207 * _tmp414;
  const Scalar _tmp467 = _tmp127 * _tmp138;
  const Scalar _tmp468 = _tmp142 * _tmp438;
  const Scalar _tmp469 = _tmp463 * _tmp465;
  const Scalar _tmp470 =
      (_tmp444 * (_tmp127 * _tmp190 * _tmp437 + _tmp127 * _tmp198 * _tmp417 +
                  _tmp127 * _tmp209 * _tmp439 + _tmp127 * _tmp237 * _tmp398 +
                  _tmp127 * _tmp253 * _tmp434 - _tmp188 * _tmp190 * _tmp344 -
                  _tmp198 * _tmp199 * _tmp344 + _tmp200 * _tmp424 - _tmp209 * _tmp231 * _tmp344 -
                  _tmp237 * _tmp240 * _tmp344 - _tmp250 * _tmp253 * _tmp344 + _tmp299 * _tmp445 -
                  _tmp343 * _tmp448 + _tmp393 * _tmp449 - _tmp399 * _tmp445 - _tmp402 * _tmp449 +
                  _tmp427 * _tmp447 + _tmp432 * _tmp446 + _tmp466 * _tmp467 - _tmp467 * _tmp468) -
       _tmp450 * _tmp469) /
      std::sqrt(Scalar(std::pow(_tmp450, Scalar(2)) * _tmp465 + 1));
  const Scalar _tmp471 = _tmp291 + _tmp92;
  const Scalar _tmp472 = _tmp209 * _tmp230 + _tmp253 * _tmp259 + _tmp441 + _tmp442;
  const Scalar _tmp473 = Scalar(1.0) / (_tmp472);
  const Scalar _tmp474 = _tmp141 * _tmp209;
  const Scalar _tmp475 = _tmp141 * _tmp253;
  const Scalar _tmp476 = _tmp141 * _tmp190;
  const Scalar _tmp477 = _tmp135 * _tmp476 - _tmp198 * _tmp201 - _tmp228 * _tmp474 -
                         _tmp235 * _tmp240 - _tmp247 * _tmp475 + _tmp447;
  const Scalar _tmp478 = std::asinh(_tmp473 * _tmp477);
  const Scalar _tmp479 = Scalar(9.6622558468725703) * _tmp472;
  const Scalar _tmp480 = Scalar(4.8333311099999996) - _tmp150;
  const Scalar _tmp481 = -_tmp152 + Scalar(-1.79662371);
  const Scalar _tmp482 =
      std::sqrt(Scalar(std::pow(_tmp480, Scalar(2)) + std::pow(_tmp481, Scalar(2))));
  const Scalar _tmp483 = -_tmp478 * _tmp479 - _tmp482;
  const Scalar _tmp484 = Scalar(0.1034955) * _tmp473;
  const Scalar _tmp485 = _tmp483 * _tmp484;
  const Scalar _tmp486 = Scalar(1.0) * _tmp478;
  const Scalar _tmp487 = -_tmp209 * _tmp356 + _tmp209 * _tmp375 + _tmp230 * _tmp393 -
                         _tmp230 * _tmp402 + _tmp253 * _tmp389 - _tmp253 * _tmp390 +
                         _tmp259 * _tmp299 - _tmp259 * _tmp399 + _tmp460 - _tmp461 - _tmp462;
  const Scalar _tmp488 = Scalar(9.6622558468725703) * _tmp487;
  const Scalar _tmp489 = std::pow(_tmp472, Scalar(-2));
  const Scalar _tmp490 = _tmp487 * _tmp489;
  const Scalar _tmp491 = _tmp141 * _tmp247;
  const Scalar _tmp492 = _tmp141 * _tmp228;
  const Scalar _tmp493 =
      (_tmp473 * (_tmp185 * _tmp414 * _tmp442 - _tmp190 * _tmp435 + _tmp198 * _tmp410 +
                  _tmp198 * _tmp416 - _tmp198 * _tmp418 - _tmp201 * _tmp424 +
                  _tmp209 * _tmp228 * _tmp414 - _tmp235 * _tmp398 - _tmp240 * _tmp430 +
                  _tmp240 * _tmp431 + _tmp247 * _tmp253 * _tmp414 - _tmp299 * _tmp491 +
                  _tmp365 * _tmp474 - _tmp372 * _tmp474 - _tmp374 * _tmp474 + _tmp380 * _tmp475 -
                  _tmp386 * _tmp475 - _tmp387 * _tmp475 - _tmp393 * _tmp492 + _tmp399 * _tmp491 +
                  _tmp402 * _tmp492 + _tmp436 * _tmp476 - _tmp466 + _tmp468) -
       _tmp477 * _tmp490) /
      std::sqrt(Scalar(std::pow(_tmp477, Scalar(2)) * _tmp489 + 1));

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) =
      _tmp119 -
      Scalar(0.5) * (2 * _tmp39 * (_tmp83 + _tmp92) + 2 * _tmp50 * (_tmp103 + _tmp95)) *
          std::sinh(Scalar(0.1034955) * _tmp0 *
                    (-_tmp51 - Scalar(9.6622558468725703) * fh1 * std::asinh(_tmp0 * fv1))) /
          _tmp51;
  _res(1, 0) =
      -_tmp277 *
          (-Scalar(0.86565325453551001) * _tmp406 + Scalar(1.0) * _tmp440 * std::sinh(_tmp273) -
           (-Scalar(0.1034955) * _tmp278 * _tmp406 +
            _tmp279 * (-_tmp272 * _tmp404 - _tmp277 * _tmp440 -
                       Scalar(1) / Scalar(2) *
                           (2 * _tmp274 * _tmp408 + 2 * _tmp275 * (_tmp407 + _tmp95)) / _tmp276)) *
               std::sinh(_tmp280)) +
      _tmp328 -
      _tmp404 * (Scalar(0.86565325453551001) * _tmp271 + std::cosh(_tmp273) - std::cosh(_tmp280));
  _res(2, 0) =
      _tmp333 -
      Scalar(9.6622558468725703) * _tmp463 *
          (Scalar(0.87679799772039002) * _tmp444 + std::cosh(_tmp452) - std::cosh(_tmp459)) -
      _tmp464 *
          (-Scalar(0.87679799772039002) * _tmp469 + Scalar(1.0) * _tmp470 * std::sinh(_tmp452) -
           (-Scalar(0.1034955) * _tmp457 * _tmp469 +
            _tmp458 * (-_tmp456 * _tmp463 - _tmp464 * _tmp470 -
                       Scalar(1) / Scalar(2) *
                           (2 * _tmp453 * (_tmp471 + _tmp82) + 2 * _tmp454 * (_tmp103 + _tmp281)) /
                           _tmp455)) *
               std::sinh(_tmp459));
  _res(3, 0) =
      _tmp337 -
      _tmp479 *
          (-Scalar(0.86625939559540499) * _tmp490 + Scalar(1.0) * _tmp493 * std::sinh(_tmp486) -
           (-Scalar(0.1034955) * _tmp483 * _tmp490 +
            _tmp484 * (-_tmp478 * _tmp488 - _tmp479 * _tmp493 -
                       Scalar(1) / Scalar(2) *
                           (2 * _tmp480 * (_tmp302 + _tmp471) + 2 * _tmp481 * (_tmp281 + _tmp407)) /
                           _tmp482)) *
               std::sinh(_tmp485)) -
      _tmp488 * (Scalar(0.86625939559540499) * _tmp473 - std::cosh(_tmp485) + std::cosh(_tmp486));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
