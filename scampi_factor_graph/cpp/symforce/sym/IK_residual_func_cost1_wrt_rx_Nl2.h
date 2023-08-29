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
 * Symbolic function: IK_residual_func_cost1_wrt_rx_Nl2
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost1WrtRxNl2(
    const Scalar fh1, const Scalar fv1, const Scalar rx, const Scalar ry, const Scalar rz,
    const Scalar p_init0, const Scalar p_init1, const Scalar p_init2, const Scalar rot_init_x,
    const Scalar rot_init_y, const Scalar rot_init_z, const Scalar rot_init_w,
    const Scalar epsilon) {
  // Total ops: 1621

  // Unused inputs
  (void)p_init2;
  (void)epsilon;

  // Input arrays

  // Intermediate terms (500)
  const Scalar _tmp0 = std::pow(rx, Scalar(2));
  const Scalar _tmp1 = _tmp0 + std::pow(ry, Scalar(2)) + std::pow(rz, Scalar(2));
  const Scalar _tmp2 = std::sqrt(_tmp1);
  const Scalar _tmp3 = (Scalar(1) / Scalar(2)) * _tmp2;
  const Scalar _tmp4 = std::cos(_tmp3);
  const Scalar _tmp5 = _tmp4 * rot_init_y;
  const Scalar _tmp6 = std::sin(_tmp3);
  const Scalar _tmp7 = _tmp6 / _tmp2;
  const Scalar _tmp8 = _tmp7 * rot_init_w;
  const Scalar _tmp9 = _tmp7 * rot_init_z;
  const Scalar _tmp10 = _tmp9 * rx;
  const Scalar _tmp11 = _tmp7 * rot_init_x;
  const Scalar _tmp12 = _tmp10 - _tmp11 * rz + _tmp5 + _tmp8 * ry;
  const Scalar _tmp13 = _tmp4 * rot_init_x;
  const Scalar _tmp14 = (Scalar(1) / Scalar(2)) / _tmp1;
  const Scalar _tmp15 = _tmp0 * _tmp14;
  const Scalar _tmp16 = _tmp4 * rot_init_z;
  const Scalar _tmp17 = _tmp14 * rx;
  const Scalar _tmp18 = _tmp17 * rz;
  const Scalar _tmp19 = _tmp17 * ry;
  const Scalar _tmp20 = _tmp8 * rx;
  const Scalar _tmp21 = _tmp6 / (_tmp1 * std::sqrt(_tmp1));
  const Scalar _tmp22 = _tmp0 * _tmp21;
  const Scalar _tmp23 = _tmp21 * rx;
  const Scalar _tmp24 = _tmp23 * ry;
  const Scalar _tmp25 = _tmp23 * rz;
  const Scalar _tmp26 = -_tmp11 - _tmp13 * _tmp15 - _tmp16 * _tmp18 - _tmp19 * _tmp5 -
                        Scalar(1) / Scalar(2) * _tmp20 + _tmp22 * rot_init_x + _tmp24 * rot_init_y +
                        _tmp25 * rot_init_z;
  const Scalar _tmp27 = Scalar(0.021999999999999999) * _tmp26;
  const Scalar _tmp28 = _tmp12 * _tmp27;
  const Scalar _tmp29 = _tmp7 * rot_init_y;
  const Scalar _tmp30 = _tmp13 + _tmp20 + _tmp29 * rz - _tmp9 * ry;
  const Scalar _tmp31 = _tmp4 * rot_init_w;
  const Scalar _tmp32 = -Scalar(1) / Scalar(2) * _tmp10 + _tmp13 * _tmp19 - _tmp15 * _tmp5 +
                        _tmp18 * _tmp31 + _tmp22 * rot_init_y - _tmp24 * rot_init_x -
                        _tmp25 * rot_init_w - _tmp29;
  const Scalar _tmp33 = Scalar(0.021999999999999999) * _tmp32;
  const Scalar _tmp34 = _tmp30 * _tmp33;
  const Scalar _tmp35 = _tmp11 * rx;
  const Scalar _tmp36 = _tmp15 * _tmp31 - _tmp16 * _tmp19 + _tmp18 * _tmp5 - _tmp22 * rot_init_w +
                        _tmp24 * rot_init_z - _tmp25 * rot_init_y - Scalar(1) / Scalar(2) * _tmp35 +
                        _tmp8;
  const Scalar _tmp37 = _tmp29 * rx;
  const Scalar _tmp38 = _tmp11 * ry + _tmp16 - _tmp37 + _tmp8 * rz;
  const Scalar _tmp39 = Scalar(0.021999999999999999) * _tmp38;
  const Scalar _tmp40 = _tmp36 * _tmp39;
  const Scalar _tmp41 = -_tmp13 * _tmp18 + _tmp15 * _tmp16 + _tmp19 * _tmp31 - _tmp22 * rot_init_z -
                        _tmp24 * rot_init_w + _tmp25 * rot_init_x - Scalar(1) / Scalar(2) * _tmp37 +
                        _tmp9;
  const Scalar _tmp42 = -_tmp29 * ry + _tmp31 - _tmp35 - _tmp9 * rz;
  const Scalar _tmp43 = Scalar(0.021999999999999999) * _tmp42;
  const Scalar _tmp44 = _tmp41 * _tmp43;
  const Scalar _tmp45 = _tmp28 + _tmp34 + _tmp40 + _tmp44;
  const Scalar _tmp46 = Scalar(0.83999999999999997) * _tmp38;
  const Scalar _tmp47 = _tmp32 * _tmp46;
  const Scalar _tmp48 = -_tmp47;
  const Scalar _tmp49 = Scalar(0.83999999999999997) * _tmp12;
  const Scalar _tmp50 = _tmp41 * _tmp49;
  const Scalar _tmp51 = _tmp48 - _tmp50;
  const Scalar _tmp52 = Scalar(0.41999999999999998) * _tmp41;
  const Scalar _tmp53 = _tmp30 * _tmp52;
  const Scalar _tmp54 = Scalar(0.41999999999999998) * _tmp36;
  const Scalar _tmp55 = _tmp12 * _tmp54;
  const Scalar _tmp56 = _tmp53 + _tmp55;
  const Scalar _tmp57 = Scalar(0.41999999999999998) * _tmp26;
  const Scalar _tmp58 = _tmp38 * _tmp57;
  const Scalar _tmp59 = Scalar(0.41999999999999998) * _tmp32;
  const Scalar _tmp60 = _tmp42 * _tmp59;
  const Scalar _tmp61 = -_tmp58 - _tmp60;
  const Scalar _tmp62 = _tmp56 + _tmp61;
  const Scalar _tmp63 = _tmp51 + _tmp62;
  const Scalar _tmp64 = -2 * std::pow(_tmp12, Scalar(2));
  const Scalar _tmp65 = -2 * std::pow(_tmp38, Scalar(2));
  const Scalar _tmp66 = Scalar(0.20999999999999999) * _tmp64 +
                        Scalar(0.20999999999999999) * _tmp65 + Scalar(0.20999999999999999);
  const Scalar _tmp67 = -_tmp66;
  const Scalar _tmp68 = 2 * _tmp12;
  const Scalar _tmp69 = _tmp30 * _tmp68;
  const Scalar _tmp70 = 2 * _tmp38;
  const Scalar _tmp71 = _tmp42 * _tmp70;
  const Scalar _tmp72 = Scalar(0.20999999999999999) * _tmp69 - Scalar(0.20999999999999999) * _tmp71;
  const Scalar _tmp73 = _tmp30 * _tmp70;
  const Scalar _tmp74 = _tmp42 * _tmp68;
  const Scalar _tmp75 = _tmp73 + _tmp74;
  const Scalar _tmp76 = -Scalar(0.010999999999999999) * _tmp75;
  const Scalar _tmp77 = -_tmp72 + _tmp76;
  const Scalar _tmp78 = _tmp67 + _tmp77;
  const Scalar _tmp79 = _tmp78 + p_init0;
  const Scalar _tmp80 = -_tmp79 + Scalar(-1.9874742000000001);
  const Scalar _tmp81 = _tmp58 + _tmp60;
  const Scalar _tmp82 = _tmp56 + _tmp81;
  const Scalar _tmp83 = Scalar(0.83999999999999997) * _tmp30;
  const Scalar _tmp84 = _tmp36 * _tmp83;
  const Scalar _tmp85 = _tmp48 - _tmp84;
  const Scalar _tmp86 = _tmp27 * _tmp30;
  const Scalar _tmp87 = _tmp12 * _tmp33;
  const Scalar _tmp88 = _tmp39 * _tmp41;
  const Scalar _tmp89 = _tmp36 * _tmp43;
  const Scalar _tmp90 = -_tmp86 + _tmp87 + _tmp88 - _tmp89;
  const Scalar _tmp91 = _tmp85 + _tmp90;
  const Scalar _tmp92 = Scalar(0.20999999999999999) * _tmp69 + Scalar(0.20999999999999999) * _tmp71;
  const Scalar _tmp93 = -_tmp92;
  const Scalar _tmp94 = _tmp12 * _tmp70;
  const Scalar _tmp95 = 2 * _tmp30 * _tmp42;
  const Scalar _tmp96 = _tmp94 - _tmp95;
  const Scalar _tmp97 = -Scalar(0.010999999999999999) * _tmp96;
  const Scalar _tmp98 = 1 - 2 * std::pow(_tmp30, Scalar(2));
  const Scalar _tmp99 = Scalar(0.20999999999999999) * _tmp65 + Scalar(0.20999999999999999) * _tmp98;
  const Scalar _tmp100 = _tmp97 - _tmp99;
  const Scalar _tmp101 = _tmp100 + _tmp93;
  const Scalar _tmp102 = _tmp101 + p_init1;
  const Scalar _tmp103 = -_tmp102 + Scalar(-8.3196563700000006);
  const Scalar _tmp104 =
      std::sqrt(Scalar(std::pow(_tmp103, Scalar(2)) + std::pow(_tmp80, Scalar(2))));
  const Scalar _tmp105 = Scalar(1.0) / (fh1);
  const Scalar _tmp106 = _tmp30 * _tmp57;
  const Scalar _tmp107 = _tmp12 * _tmp59;
  const Scalar _tmp108 = _tmp38 * _tmp52;
  const Scalar _tmp109 = _tmp42 * _tmp54;
  const Scalar _tmp110 = -_tmp106 - _tmp107 - _tmp108 - _tmp109;
  const Scalar _tmp111 = _tmp12 * _tmp41;
  const Scalar _tmp112 = Scalar(0.043999999999999997) * _tmp111;
  const Scalar _tmp113 = _tmp30 * _tmp36;
  const Scalar _tmp114 = Scalar(0.043999999999999997) * _tmp113;
  const Scalar _tmp115 = _tmp112 + _tmp114;
  const Scalar _tmp116 = _tmp12 * _tmp57;
  const Scalar _tmp117 = _tmp30 * _tmp59;
  const Scalar _tmp118 = _tmp38 * _tmp54;
  const Scalar _tmp119 = _tmp42 * _tmp52;
  const Scalar _tmp120 = _tmp116 - _tmp117 - _tmp118 + _tmp119;
  const Scalar _tmp121 = _tmp115 + _tmp120;
  const Scalar _tmp122 = _tmp110 + _tmp121;
  const Scalar _tmp123 = _tmp72 + _tmp76;
  const Scalar _tmp124 = _tmp123 + _tmp66;
  const Scalar _tmp125 = _tmp124 + p_init0;
  const Scalar _tmp126 = _tmp125 + Scalar(-2.71799795);
  const Scalar _tmp127 = Scalar(1.0) / (_tmp126);
  const Scalar _tmp128 = _tmp97 + _tmp99;
  const Scalar _tmp129 = _tmp128 + _tmp92;
  const Scalar _tmp130 = _tmp129 + p_init1;
  const Scalar _tmp131 = _tmp130 + Scalar(-4.7752063900000001);
  const Scalar _tmp132 = _tmp127 * _tmp131;
  const Scalar _tmp133 = _tmp66 + _tmp77;
  const Scalar _tmp134 = _tmp133 + p_init0;
  const Scalar _tmp135 = _tmp134 + Scalar(-2.5202214700000001);
  const Scalar _tmp136 = _tmp100 + _tmp92;
  const Scalar _tmp137 = _tmp136 + p_init1;
  const Scalar _tmp138 = _tmp137 + Scalar(8.3888750099999996);
  const Scalar _tmp139 = std::pow(_tmp135, Scalar(2)) + std::pow(_tmp138, Scalar(2));
  const Scalar _tmp140 = std::pow(_tmp139, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp141 = _tmp135 * _tmp140;
  const Scalar _tmp142 = _tmp138 * _tmp140;
  const Scalar _tmp143 = _tmp132 * _tmp141 - _tmp142;
  const Scalar _tmp144 = Scalar(1.0) / (_tmp143);
  const Scalar _tmp145 = _tmp141 * _tmp144;
  const Scalar _tmp146 = Scalar(1.0) * _tmp129;
  const Scalar _tmp147 = -_tmp146;
  const Scalar _tmp148 = _tmp136 + _tmp147;
  const Scalar _tmp149 = Scalar(1.0) / (_tmp148);
  const Scalar _tmp150 = Scalar(1.0) * _tmp124;
  const Scalar _tmp151 = -_tmp133 + _tmp150;
  const Scalar _tmp152 = _tmp149 * _tmp151;
  const Scalar _tmp153 = _tmp146 * _tmp152 + _tmp150;
  const Scalar _tmp154 = 0;
  const Scalar _tmp155 =
      Scalar(0.20999999999999999) * _tmp73 - Scalar(0.20999999999999999) * _tmp74;
  const Scalar _tmp156 =
      -Scalar(0.010999999999999999) * _tmp64 - Scalar(0.010999999999999999) * _tmp98;
  const Scalar _tmp157 =
      Scalar(0.20999999999999999) * _tmp94 + Scalar(0.20999999999999999) * _tmp95;
  const Scalar _tmp158 = _tmp156 - _tmp157;
  const Scalar _tmp159 = _tmp155 + _tmp158;
  const Scalar _tmp160 = _tmp156 + _tmp157;
  const Scalar _tmp161 = _tmp155 + _tmp160;
  const Scalar _tmp162 = _tmp132 * _tmp161;
  const Scalar _tmp163 = -_tmp141 * _tmp162 + _tmp142 * _tmp159;
  const Scalar _tmp164 = _tmp128 + _tmp93;
  const Scalar _tmp165 = _tmp164 + p_init1;
  const Scalar _tmp166 = _tmp165 + Scalar(-4.8333311099999996);
  const Scalar _tmp167 = _tmp123 + _tmp67;
  const Scalar _tmp168 = _tmp167 + p_init0;
  const Scalar _tmp169 = _tmp168 + Scalar(1.79662371);
  const Scalar _tmp170 = std::pow(_tmp166, Scalar(2)) + std::pow(_tmp169, Scalar(2));
  const Scalar _tmp171 = std::pow(_tmp170, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp172 = _tmp169 * _tmp171;
  const Scalar _tmp173 = _tmp166 * _tmp171;
  const Scalar _tmp174 = _tmp132 * _tmp172 - _tmp173;
  const Scalar _tmp175 = _tmp144 * _tmp174;
  const Scalar _tmp176 = -_tmp155;
  const Scalar _tmp177 = _tmp160 + _tmp176;
  const Scalar _tmp178 = _tmp161 * _tmp172;
  const Scalar _tmp179 = -_tmp132 * _tmp178 - _tmp163 * _tmp175 + _tmp173 * _tmp177;
  const Scalar _tmp180 = _tmp141 * _tmp161;
  const Scalar _tmp181 = -_tmp141 * _tmp159 + _tmp180;
  const Scalar _tmp182 = _tmp144 * _tmp181;
  const Scalar _tmp183 = -_tmp152 * _tmp179 - _tmp172 * _tmp177 - _tmp174 * _tmp182 + _tmp178;
  const Scalar _tmp184 = Scalar(1.0) / (_tmp183);
  const Scalar _tmp185 = _tmp174 * _tmp184;
  const Scalar _tmp186 = _tmp154 * _tmp185;
  const Scalar _tmp187 = _tmp154 * _tmp184;
  const Scalar _tmp188 = -_tmp145 * _tmp186 + _tmp172 * _tmp187;
  const Scalar _tmp189 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp190 = std::pow(_tmp126, Scalar(2));
  const Scalar _tmp191 = std::pow(_tmp131, Scalar(2)) + _tmp190;
  const Scalar _tmp192 = std::sqrt(_tmp191);
  const Scalar _tmp193 = _tmp127 * _tmp192;
  const Scalar _tmp194 = _tmp189 * _tmp193;
  const Scalar _tmp195 = Scalar(1.0) / (_tmp192);
  const Scalar _tmp196 = _tmp124 * _tmp195;
  const Scalar _tmp197 = _tmp126 * _tmp195;
  const Scalar _tmp198 = -_tmp129 * _tmp197 + _tmp131 * _tmp196;
  const Scalar _tmp199 = _tmp193 * _tmp198;
  const Scalar _tmp200 = -_tmp133 * _tmp142 + _tmp136 * _tmp141 + _tmp141 * _tmp199;
  const Scalar _tmp201 = _tmp132 * _tmp144;
  const Scalar _tmp202 = _tmp162 + _tmp163 * _tmp201;
  const Scalar _tmp203 = _tmp132 * _tmp182 - _tmp152 * _tmp202 - _tmp161;
  const Scalar _tmp204 =
      _tmp164 * _tmp172 - _tmp167 * _tmp173 + _tmp172 * _tmp199 - _tmp175 * _tmp200;
  const Scalar _tmp205 = _tmp184 * _tmp204;
  const Scalar _tmp206 = -_tmp199 + _tmp200 * _tmp201 - _tmp203 * _tmp205;
  const Scalar _tmp207 = Scalar(1.0) / (_tmp204);
  const Scalar _tmp208 = _tmp183 * _tmp207;
  const Scalar _tmp209 = _tmp206 * _tmp208;
  const Scalar _tmp210 = _tmp203 + _tmp209;
  const Scalar _tmp211 = _tmp184 * _tmp210;
  const Scalar _tmp212 = -_tmp132 - _tmp174 * _tmp211;
  const Scalar _tmp213 = _tmp145 * _tmp212 + _tmp172 * _tmp211 + Scalar(1.0);
  const Scalar _tmp214 = _tmp79 + Scalar(1.9874742000000001);
  const Scalar _tmp215 = _tmp102 + Scalar(8.3196563700000006);
  const Scalar _tmp216 = std::pow(_tmp214, Scalar(2)) + std::pow(_tmp215, Scalar(2));
  const Scalar _tmp217 = std::pow(_tmp216, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp218 = _tmp217 * fh1;
  const Scalar _tmp219 = _tmp214 * _tmp218;
  const Scalar _tmp220 = _tmp193 * _tmp219;
  const Scalar _tmp221 = Scalar(1.0) * _tmp207;
  const Scalar _tmp222 = Scalar(1.0) * _tmp144;
  const Scalar _tmp223 = _tmp207 * _tmp222;
  const Scalar _tmp224 = _tmp174 * _tmp223;
  const Scalar _tmp225 = -_tmp141 * _tmp224 + _tmp172 * _tmp221;
  const Scalar _tmp226 = _tmp101 * _tmp217;
  const Scalar _tmp227 = _tmp215 * _tmp217;
  const Scalar _tmp228 = fh1 * (-_tmp214 * _tmp226 + _tmp227 * _tmp78);
  const Scalar _tmp229 = _tmp193 * _tmp228;
  const Scalar _tmp230 = _tmp227 * fh1;
  const Scalar _tmp231 = Scalar(1.0) * _tmp149;
  const Scalar _tmp232 = _tmp151 * _tmp231;
  const Scalar _tmp233 = _tmp144 * _tmp232;
  const Scalar _tmp234 = _tmp163 * _tmp233 - Scalar(1.0) * _tmp182;
  const Scalar _tmp235 = -_tmp200 * _tmp222 - _tmp205 * _tmp234;
  const Scalar _tmp236 = _tmp207 * _tmp235;
  const Scalar _tmp237 = _tmp183 * _tmp236;
  const Scalar _tmp238 = _tmp234 + _tmp237;
  const Scalar _tmp239 = -_tmp185 * _tmp238 + Scalar(1.0);
  const Scalar _tmp240 = _tmp184 * _tmp238;
  const Scalar _tmp241 = _tmp145 * _tmp239 + _tmp172 * _tmp240;
  const Scalar _tmp242 = _tmp193 * _tmp241;
  const Scalar _tmp243 =
      -_tmp188 * _tmp194 - _tmp213 * _tmp220 - _tmp225 * _tmp229 - _tmp230 * _tmp242;
  const Scalar _tmp244 = std::pow(_tmp243, Scalar(-2));
  const Scalar _tmp245 = _tmp193 * _tmp213;
  const Scalar _tmp246 = -_tmp53 - _tmp55;
  const Scalar _tmp247 = _tmp246 + _tmp81;
  const Scalar _tmp248 = -_tmp28 - _tmp34 - _tmp40 - _tmp44;
  const Scalar _tmp249 = _tmp47 + _tmp50;
  const Scalar _tmp250 = _tmp248 + _tmp249;
  const Scalar _tmp251 = _tmp247 + _tmp250;
  const Scalar _tmp252 = _tmp86 - _tmp87 - _tmp88 + _tmp89;
  const Scalar _tmp253 = _tmp47 + _tmp84;
  const Scalar _tmp254 = _tmp246 + _tmp61;
  const Scalar _tmp255 = _tmp253 + _tmp254;
  const Scalar _tmp256 = _tmp252 + _tmp255;
  const Scalar _tmp257 =
      (2 * _tmp214 * _tmp251 + 2 * _tmp215 * _tmp256) / (_tmp216 * std::sqrt(_tmp216));
  const Scalar _tmp258 = (Scalar(1) / Scalar(2)) * _tmp257;
  const Scalar _tmp259 = _tmp214 * _tmp258;
  const Scalar _tmp260 = _tmp259 * fh1;
  const Scalar _tmp261 = _tmp188 * _tmp189;
  const Scalar _tmp262 = _tmp248 + _tmp63;
  const Scalar _tmp263 = _tmp252 + _tmp85;
  const Scalar _tmp264 = _tmp263 + _tmp82;
  const Scalar _tmp265 = _tmp126 * _tmp262 + _tmp131 * _tmp264;
  const Scalar _tmp266 = _tmp127 * _tmp195 * _tmp265;
  const Scalar _tmp267 = _tmp225 * _tmp228;
  const Scalar _tmp268 = _tmp230 * _tmp241;
  const Scalar _tmp269 = _tmp262 / _tmp190;
  const Scalar _tmp270 = _tmp192 * _tmp269;
  const Scalar _tmp271 = _tmp250 + _tmp62;
  const Scalar _tmp272 = _tmp171 * _tmp271;
  const Scalar _tmp273 = _tmp254 + _tmp263;
  const Scalar _tmp274 =
      (2 * _tmp166 * _tmp273 + 2 * _tmp169 * _tmp271) / (_tmp170 * std::sqrt(_tmp170));
  const Scalar _tmp275 = (Scalar(1) / Scalar(2)) * _tmp274;
  const Scalar _tmp276 = _tmp169 * _tmp275;
  const Scalar _tmp277 = _tmp171 * _tmp273;
  const Scalar _tmp278 = _tmp198 * _tmp266;
  const Scalar _tmp279 = _tmp131 * _tmp269;
  const Scalar _tmp280 = _tmp166 * _tmp275;
  const Scalar _tmp281 = _tmp127 * _tmp264;
  const Scalar _tmp282 = _tmp132 * _tmp272 - _tmp132 * _tmp276 - _tmp172 * _tmp279 +
                         _tmp172 * _tmp281 - _tmp277 + _tmp280;
  const Scalar _tmp283 = _tmp144 * _tmp282;
  const Scalar _tmp284 = _tmp195 * _tmp262;
  const Scalar _tmp285 = _tmp265 / (_tmp191 * std::sqrt(_tmp191));
  const Scalar _tmp286 =
      _tmp193 * (-_tmp124 * _tmp131 * _tmp285 + _tmp126 * _tmp129 * _tmp285 - _tmp129 * _tmp284 +
                 _tmp131 * _tmp284 + _tmp196 * _tmp264 - _tmp197 * _tmp264);
  const Scalar _tmp287 = _tmp247 + _tmp51;
  const Scalar _tmp288 = _tmp248 + _tmp287;
  const Scalar _tmp289 = _tmp253 + _tmp82;
  const Scalar _tmp290 = _tmp252 + _tmp289;
  const Scalar _tmp291 =
      (2 * _tmp135 * _tmp288 + 2 * _tmp138 * _tmp290) / (_tmp139 * std::sqrt(_tmp139));
  const Scalar _tmp292 = (Scalar(1) / Scalar(2)) * _tmp291;
  const Scalar _tmp293 = _tmp138 * _tmp292;
  const Scalar _tmp294 = _tmp140 * _tmp288;
  const Scalar _tmp295 = _tmp140 * _tmp290;
  const Scalar _tmp296 = _tmp135 * _tmp292;
  const Scalar _tmp297 = _tmp198 * _tmp270;
  const Scalar _tmp298 = _tmp133 * _tmp293 - _tmp133 * _tmp295 + _tmp135 * _tmp295 +
                         _tmp136 * _tmp294 - _tmp136 * _tmp296 + _tmp141 * _tmp278 +
                         _tmp141 * _tmp286 - _tmp141 * _tmp297 - _tmp142 * _tmp288 +
                         _tmp199 * _tmp294 - _tmp199 * _tmp296;
  const Scalar _tmp299 = (_tmp132 * _tmp294 - _tmp132 * _tmp296 - _tmp141 * _tmp279 +
                          _tmp141 * _tmp281 + _tmp293 - _tmp295) /
                         std::pow(_tmp143, Scalar(2));
  const Scalar _tmp300 = _tmp174 * _tmp299;
  const Scalar _tmp301 = _tmp164 * _tmp272 - _tmp164 * _tmp276 - _tmp167 * _tmp277 +
                         _tmp167 * _tmp280 + _tmp169 * _tmp277 + _tmp172 * _tmp278 +
                         _tmp172 * _tmp286 - _tmp172 * _tmp297 - _tmp173 * _tmp271 -
                         _tmp175 * _tmp298 + _tmp199 * _tmp272 - _tmp199 * _tmp276 -
                         _tmp200 * _tmp283 + _tmp200 * _tmp300;
  const Scalar _tmp302 = _tmp301 / std::pow(_tmp204, Scalar(2));
  const Scalar _tmp303 = _tmp174 * _tmp222 * _tmp302;
  const Scalar _tmp304 = Scalar(0.5) * _tmp207;
  const Scalar _tmp305 = _tmp223 * _tmp282;
  const Scalar _tmp306 = Scalar(1.0) * _tmp302;
  const Scalar _tmp307 = _tmp217 * _tmp256;
  const Scalar _tmp308 = _tmp307 * fh1;
  const Scalar _tmp309 = _tmp218 * _tmp251;
  const Scalar _tmp310 = _tmp115 - _tmp116 + _tmp117 + _tmp118 - _tmp119;
  const Scalar _tmp311 = _tmp106 + _tmp107 + _tmp108 + _tmp109;
  const Scalar _tmp312 = _tmp310 + _tmp311;
  const Scalar _tmp313 = _tmp172 * _tmp312;
  const Scalar _tmp314 = _tmp110 + _tmp310;
  const Scalar _tmp315 = _tmp161 * _tmp294;
  const Scalar _tmp316 = _tmp141 * _tmp312;
  const Scalar _tmp317 = -_tmp132 * _tmp315 - _tmp132 * _tmp316 + _tmp142 * _tmp314 -
                         _tmp159 * _tmp293 + _tmp159 * _tmp295 + _tmp162 * _tmp296 +
                         _tmp180 * _tmp279 - _tmp180 * _tmp281;
  const Scalar _tmp318 = _tmp121 + _tmp311;
  const Scalar _tmp319 = -_tmp132 * _tmp313 - _tmp162 * _tmp272 + _tmp162 * _tmp276 -
                         _tmp163 * _tmp283 + _tmp163 * _tmp300 + _tmp173 * _tmp318 -
                         _tmp175 * _tmp317 + _tmp177 * _tmp277 - _tmp177 * _tmp280 +
                         _tmp178 * _tmp279 - _tmp178 * _tmp281;
  const Scalar _tmp320 = Scalar(1.6799999999999999) * _tmp32 * _tmp38;
  const Scalar _tmp321 =
      (Scalar(1.6799999999999999) * _tmp113 + _tmp320) / std::pow(_tmp148, Scalar(2));
  const Scalar _tmp322 = _tmp151 * _tmp321;
  const Scalar _tmp323 = -_tmp141 * _tmp314 - _tmp159 * _tmp294 + _tmp159 * _tmp296 -
                         _tmp161 * _tmp296 + _tmp315 + _tmp316;
  const Scalar _tmp324 = _tmp41 * _tmp83;
  const Scalar _tmp325 = _tmp36 * _tmp49;
  const Scalar _tmp326 = -_tmp26 * _tmp46 - Scalar(0.83999999999999997) * _tmp32 * _tmp42;
  const Scalar _tmp327 = _tmp324 + _tmp325 + _tmp326;
  const Scalar _tmp328 = _tmp149 * _tmp327;
  const Scalar _tmp329 = -_tmp152 * _tmp319 + _tmp161 * _tmp272 - _tmp161 * _tmp276 -
                         _tmp172 * _tmp318 - _tmp175 * _tmp323 - _tmp177 * _tmp272 +
                         _tmp177 * _tmp276 + _tmp179 * _tmp322 - _tmp179 * _tmp328 +
                         _tmp181 * _tmp300 - _tmp182 * _tmp282 + _tmp313;
  const Scalar _tmp330 = _tmp329 / std::pow(_tmp183, Scalar(2));
  const Scalar _tmp331 = _tmp172 * _tmp330;
  const Scalar _tmp332 = _tmp141 * _tmp299;
  const Scalar _tmp333 = _tmp144 * _tmp294;
  const Scalar _tmp334 = _tmp213 * _tmp219;
  const Scalar _tmp335 = _tmp215 * _tmp258;
  const Scalar _tmp336 = fh1 * (_tmp101 * _tmp259 - _tmp214 * _tmp307 - _tmp226 * _tmp251 +
                                _tmp227 * _tmp251 + _tmp307 * _tmp78 - _tmp335 * _tmp78);
  const Scalar _tmp337 = _tmp132 * _tmp299;
  const Scalar _tmp338 = _tmp144 * _tmp281;
  const Scalar _tmp339 = _tmp144 * _tmp279;
  const Scalar _tmp340 = _tmp132 * _tmp312 - _tmp161 * _tmp279 + _tmp161 * _tmp281 -
                         _tmp163 * _tmp337 + _tmp163 * _tmp338 - _tmp163 * _tmp339 +
                         _tmp201 * _tmp317;
  const Scalar _tmp341 = _tmp110 - _tmp112 - _tmp114 + _tmp120 - _tmp152 * _tmp340 -
                         _tmp181 * _tmp337 - _tmp182 * _tmp279 + _tmp182 * _tmp281 +
                         _tmp201 * _tmp323 + _tmp202 * _tmp322 - _tmp202 * _tmp328;
  const Scalar _tmp342 = _tmp184 * _tmp301;
  const Scalar _tmp343 = _tmp204 * _tmp330;
  const Scalar _tmp344 = _tmp208 * (-_tmp200 * _tmp337 + _tmp200 * _tmp338 - _tmp200 * _tmp339 +
                                    _tmp201 * _tmp298 - _tmp203 * _tmp342 + _tmp203 * _tmp343 -
                                    _tmp205 * _tmp341 - _tmp278 - _tmp286 + _tmp297);
  const Scalar _tmp345 = _tmp207 * _tmp329;
  const Scalar _tmp346 = _tmp206 * _tmp345;
  const Scalar _tmp347 = _tmp183 * _tmp302;
  const Scalar _tmp348 = _tmp206 * _tmp347;
  const Scalar _tmp349 = _tmp341 + _tmp344 + _tmp346 - _tmp348;
  const Scalar _tmp350 = _tmp174 * _tmp330;
  const Scalar _tmp351 =
      -_tmp185 * _tmp349 + _tmp210 * _tmp350 - _tmp211 * _tmp282 + _tmp279 - _tmp281;
  const Scalar _tmp352 = _tmp144 * _tmp212;
  const Scalar _tmp353 = _tmp172 * _tmp184;
  const Scalar _tmp354 = _tmp144 * _tmp239;
  const Scalar _tmp355 = _tmp235 * _tmp347;
  const Scalar _tmp356 = _tmp236 * _tmp329;
  const Scalar _tmp357 = _tmp163 * _tmp222;
  const Scalar _tmp358 = Scalar(1.0) * _tmp299;
  const Scalar _tmp359 = _tmp144 * _tmp163 * _tmp231 * _tmp327 - _tmp163 * _tmp232 * _tmp299 +
                         _tmp181 * _tmp358 - _tmp222 * _tmp323 + _tmp233 * _tmp317 -
                         _tmp322 * _tmp357;
  const Scalar _tmp360 = _tmp208 * (_tmp200 * _tmp358 - _tmp205 * _tmp359 - _tmp222 * _tmp298 -
                                    _tmp234 * _tmp342 + _tmp234 * _tmp343);
  const Scalar _tmp361 = -_tmp355 + _tmp356 + _tmp359 + _tmp360;
  const Scalar _tmp362 = -_tmp185 * _tmp361 + _tmp238 * _tmp350 - _tmp240 * _tmp282;
  const Scalar _tmp363 = _tmp335 * fh1;
  const Scalar _tmp364 =
      -_tmp193 * _tmp225 * _tmp336 -
      _tmp193 * _tmp230 *
          (_tmp145 * _tmp362 - _tmp238 * _tmp331 - _tmp239 * _tmp332 + _tmp239 * _tmp333 +
           _tmp240 * _tmp272 - _tmp240 * _tmp276 - _tmp296 * _tmp354 + _tmp353 * _tmp361) -
      _tmp194 * (_tmp141 * _tmp154 * _tmp175 * _tmp330 - _tmp141 * _tmp187 * _tmp283 +
                 _tmp144 * _tmp186 * _tmp296 - _tmp154 * _tmp331 + _tmp186 * _tmp332 -
                 _tmp186 * _tmp333 + _tmp187 * _tmp272 - _tmp187 * _tmp276) -
      _tmp220 * (_tmp145 * _tmp351 - _tmp210 * _tmp331 + _tmp211 * _tmp272 - _tmp211 * _tmp276 -
                 _tmp212 * _tmp332 + _tmp212 * _tmp333 - _tmp296 * _tmp352 + _tmp349 * _tmp353) -
      _tmp229 * (_tmp135 * _tmp175 * _tmp291 * _tmp304 + _tmp141 * _tmp221 * _tmp300 +
                 _tmp141 * _tmp303 - _tmp141 * _tmp305 - _tmp169 * _tmp274 * _tmp304 -
                 _tmp172 * _tmp306 + _tmp221 * _tmp272 - _tmp224 * _tmp294) -
      _tmp242 * _tmp308 + _tmp242 * _tmp363 + _tmp245 * _tmp260 - _tmp245 * _tmp309 -
      _tmp261 * _tmp266 + _tmp261 * _tmp270 - _tmp266 * _tmp267 - _tmp266 * _tmp268 -
      _tmp266 * _tmp334 + _tmp267 * _tmp270 + _tmp268 * _tmp270 + _tmp270 * _tmp334;
  const Scalar _tmp365 = _tmp244 * _tmp364;
  const Scalar _tmp366 = Scalar(1.0) / (_tmp243);
  const Scalar _tmp367 = _tmp147 + _tmp164;
  const Scalar _tmp368 = _tmp152 * _tmp367;
  const Scalar _tmp369 = _tmp150 - _tmp167 - _tmp368;
  const Scalar _tmp370 = Scalar(1.0) / (_tmp369);
  const Scalar _tmp371 = Scalar(1.0) * _tmp370;
  const Scalar _tmp372 = _tmp208 * _tmp371;
  const Scalar _tmp373 = -_tmp179 * _tmp221 + _tmp367 * _tmp372;
  const Scalar _tmp374 = -_tmp231 * _tmp373 + _tmp372;
  const Scalar _tmp375 = Scalar(1.0) * _tmp228;
  const Scalar _tmp376 = _tmp367 * _tmp370;
  const Scalar _tmp377 = _tmp147 - _tmp153 * _tmp376 - _tmp179 * _tmp187;
  const Scalar _tmp378 = Scalar(1.0) * _tmp189;
  const Scalar _tmp379 = _tmp152 * _tmp371;
  const Scalar _tmp380 = _tmp368 * _tmp371 + Scalar(1.0);
  const Scalar _tmp381 = -_tmp231 * _tmp380 + _tmp379;
  const Scalar _tmp382 = _tmp158 + _tmp176;
  const Scalar _tmp383 = -_tmp101 * fv1 - _tmp230 * _tmp382 - Scalar(3.29616) * _tmp96;
  const Scalar _tmp384 = Scalar(1.0) * _tmp383;
  const Scalar _tmp385 = _tmp149 * _tmp371;
  const Scalar _tmp386 = _tmp367 * _tmp385 - _tmp371;
  const Scalar _tmp387 = _tmp218 * _tmp382;
  const Scalar _tmp388 = _tmp214 * _tmp387 + Scalar(3.29616) * _tmp75 + _tmp78 * fv1;
  const Scalar _tmp389 = Scalar(1.0) * _tmp388;
  const Scalar _tmp390 = -_tmp179 * _tmp211 + _tmp202 + _tmp209 * _tmp376;
  const Scalar _tmp391 = _tmp209 * _tmp371 - _tmp231 * _tmp390;
  const Scalar _tmp392 = Scalar(1.0) * _tmp391;
  const Scalar _tmp393 = -_tmp179 * _tmp240 + _tmp237 * _tmp376 - _tmp357;
  const Scalar _tmp394 = -_tmp231 * _tmp393 + _tmp237 * _tmp371;
  const Scalar _tmp395 = Scalar(1.0) * _tmp394;
  const Scalar _tmp396 = _tmp219 * _tmp392 + _tmp230 * _tmp395 + _tmp374 * _tmp375 +
                         _tmp378 * (-_tmp153 * _tmp371 - _tmp231 * _tmp377 + Scalar(1.0)) +
                         _tmp381 * _tmp384 + _tmp386 * _tmp389;
  const Scalar _tmp397 = std::asinh(_tmp366 * _tmp396);
  const Scalar _tmp398 = Scalar(1.0) * _tmp397;
  const Scalar _tmp399 = Scalar(6.59232) * _tmp26;
  const Scalar _tmp400 = Scalar(6.59232) * _tmp32;
  const Scalar _tmp401 = Scalar(6.59232) * _tmp38;
  const Scalar _tmp402 = Scalar(6.59232) * _tmp42;
  const Scalar _tmp403 = _tmp382 * fh1;
  const Scalar _tmp404 = -_tmp12 * _tmp400 - _tmp122 * _tmp230 - _tmp256 * fv1 + _tmp30 * _tmp399 -
                         _tmp308 * _tmp382 + _tmp335 * _tmp403 + _tmp36 * _tmp402 -
                         _tmp401 * _tmp41;
  const Scalar _tmp405 = _tmp328 * _tmp367;
  const Scalar _tmp406 = -_tmp324 - _tmp325 + _tmp326;
  const Scalar _tmp407 = _tmp152 * _tmp406;
  const Scalar _tmp408 = _tmp322 * _tmp367;
  const Scalar _tmp409 =
      (-Scalar(1.6799999999999999) * _tmp111 - _tmp320 - _tmp405 - _tmp407 + _tmp408) /
      std::pow(_tmp369, Scalar(2));
  const Scalar _tmp410 = Scalar(1.0) * _tmp409;
  const Scalar _tmp411 = _tmp179 * _tmp330;
  const Scalar _tmp412 = -_tmp146 * _tmp322 + _tmp146 * _tmp328 + _tmp232 * _tmp264 + _tmp262;
  const Scalar _tmp413 = _tmp153 * _tmp409;
  const Scalar _tmp414 = _tmp370 * _tmp406;
  const Scalar _tmp415 = _tmp255 + _tmp90;
  const Scalar _tmp416 = -_tmp153 * _tmp414 + _tmp154 * _tmp411 - _tmp187 * _tmp319 +
                         _tmp367 * _tmp413 - _tmp376 * _tmp412 + _tmp415;
  const Scalar _tmp417 = Scalar(1.0) * _tmp321;
  const Scalar _tmp418 = _tmp367 * _tmp409;
  const Scalar _tmp419 = _tmp179 * _tmp184;
  const Scalar _tmp420 = _tmp209 * _tmp414 - _tmp209 * _tmp418 + _tmp210 * _tmp411 -
                         _tmp211 * _tmp319 + _tmp340 + _tmp344 * _tmp376 + _tmp346 * _tmp376 -
                         _tmp348 * _tmp376 - _tmp349 * _tmp419;
  const Scalar _tmp421 = _tmp163 * _tmp358 - _tmp222 * _tmp317 + _tmp237 * _tmp414 -
                         _tmp237 * _tmp418 + _tmp238 * _tmp411 - _tmp240 * _tmp319 -
                         _tmp355 * _tmp376 + _tmp356 * _tmp376 + _tmp360 * _tmp376 -
                         _tmp361 * _tmp419;
  const Scalar _tmp422 = _tmp12 * _tmp399 + _tmp122 * _tmp219 + _tmp251 * _tmp387 + _tmp251 * fv1 -
                         _tmp259 * _tmp403 + _tmp30 * _tmp400 + _tmp36 * _tmp401 + _tmp402 * _tmp41;
  const Scalar _tmp423 = Scalar(0.5) * _tmp257 * fh1;
  const Scalar _tmp424 = _tmp321 * _tmp367;
  const Scalar _tmp425 = _tmp231 * _tmp418;
  const Scalar _tmp426 = _tmp208 * _tmp410;
  const Scalar _tmp427 = _tmp345 * _tmp371;
  const Scalar _tmp428 = _tmp347 * _tmp371;
  const Scalar _tmp429 = _tmp179 * _tmp306 - _tmp221 * _tmp319 - _tmp367 * _tmp426 +
                         _tmp367 * _tmp427 - _tmp367 * _tmp428 + _tmp372 * _tmp406;
  const Scalar _tmp430 = _tmp232 * _tmp409;
  const Scalar _tmp431 =
      -_tmp367 * _tmp430 + _tmp371 * _tmp405 + _tmp371 * _tmp407 - _tmp371 * _tmp408;
  const Scalar _tmp432 = _tmp322 * _tmp371;
  const Scalar _tmp433 = _tmp328 * _tmp371;
  const Scalar _tmp434 = _tmp321 * _tmp380;
  const Scalar _tmp435 =
      (-_tmp365 * _tmp396 +
       _tmp366 *
           (-_tmp214 * _tmp391 * _tmp423 - _tmp215 * _tmp394 * _tmp423 +
            Scalar(1.0) * _tmp219 *
                (-_tmp209 * _tmp410 - _tmp231 * _tmp420 + _tmp344 * _tmp371 + _tmp346 * _tmp371 -
                 _tmp348 * _tmp371 + _tmp390 * _tmp417) +
            Scalar(1.0) * _tmp230 *
                (-_tmp231 * _tmp421 - _tmp237 * _tmp410 - _tmp355 * _tmp371 + _tmp356 * _tmp371 +
                 _tmp360 * _tmp371 + _tmp393 * _tmp417) +
            _tmp308 * _tmp395 + _tmp309 * _tmp392 + Scalar(1.0) * _tmp336 * _tmp374 +
            _tmp375 * (-_tmp231 * _tmp429 + _tmp373 * _tmp417 - _tmp426 + _tmp427 - _tmp428) +
            _tmp378 *
                (_tmp153 * _tmp410 - _tmp231 * _tmp416 - _tmp371 * _tmp412 + _tmp377 * _tmp417) +
            Scalar(1.0) * _tmp381 * _tmp404 +
            _tmp384 * (-_tmp231 * _tmp431 - _tmp430 - _tmp432 + _tmp433 + Scalar(1.0) * _tmp434) +
            Scalar(1.0) * _tmp386 * _tmp422 +
            _tmp389 * (-_tmp371 * _tmp424 + _tmp385 * _tmp406 + _tmp410 - _tmp425))) /
      std::sqrt(Scalar(_tmp244 * std::pow(_tmp396, Scalar(2)) + 1));
  const Scalar _tmp436 = Scalar(9.6622558468725703) * _tmp243;
  const Scalar _tmp437 = Scalar(4.7752063900000001) - _tmp130;
  const Scalar _tmp438 = Scalar(2.71799795) - _tmp125;
  const Scalar _tmp439 =
      std::sqrt(Scalar(std::pow(_tmp437, Scalar(2)) + std::pow(_tmp438, Scalar(2))));
  const Scalar _tmp440 = -_tmp397 * _tmp436 - _tmp439;
  const Scalar _tmp441 = Scalar(9.6622558468725703) * _tmp364;
  const Scalar _tmp442 = _tmp249 + _tmp45;
  const Scalar _tmp443 = Scalar(0.1034955) * _tmp366;
  const Scalar _tmp444 = _tmp440 * _tmp443;
  const Scalar _tmp445 = _tmp154 * _tmp189;
  const Scalar _tmp446 = _tmp185 * _tmp445;
  const Scalar _tmp447 = _tmp144 * _tmp219;
  const Scalar _tmp448 =
      -_tmp144 * _tmp446 + _tmp212 * _tmp447 - _tmp224 * _tmp228 + _tmp230 * _tmp354;
  const Scalar _tmp449 = std::pow(_tmp448, Scalar(-2));
  const Scalar _tmp450 = _tmp187 * _tmp189;
  const Scalar _tmp451 = _tmp330 * _tmp445;
  const Scalar _tmp452 = _tmp221 * _tmp228;
  const Scalar _tmp453 =
      _tmp144 * _tmp230 * _tmp362 + _tmp175 * _tmp451 - _tmp212 * _tmp219 * _tmp299 -
      _tmp224 * _tmp336 + _tmp228 * _tmp303 - _tmp228 * _tmp305 - _tmp230 * _tmp239 * _tmp299 -
      _tmp260 * _tmp352 - _tmp283 * _tmp450 + _tmp299 * _tmp446 + _tmp300 * _tmp452 +
      _tmp308 * _tmp354 + _tmp309 * _tmp352 + _tmp351 * _tmp447 - _tmp354 * _tmp363;
  const Scalar _tmp454 = _tmp449 * _tmp453;
  const Scalar _tmp455 = _tmp149 * _tmp373;
  const Scalar _tmp456 = _tmp149 * _tmp230;
  const Scalar _tmp457 = _tmp149 * _tmp189;
  const Scalar _tmp458 = _tmp149 * _tmp219;
  const Scalar _tmp459 = _tmp149 * _tmp380;
  const Scalar _tmp460 = _tmp371 * _tmp388;
  const Scalar _tmp461 = _tmp149 * _tmp367;
  const Scalar _tmp462 = _tmp228 * _tmp455 + _tmp377 * _tmp457 + _tmp383 * _tmp459 +
                         _tmp390 * _tmp458 + _tmp393 * _tmp456 - _tmp460 * _tmp461;
  const Scalar _tmp463 = Scalar(1.0) / (_tmp448);
  const Scalar _tmp464 = std::asinh(_tmp462 * _tmp463);
  const Scalar _tmp465 = Scalar(1.0) * _tmp464;
  const Scalar _tmp466 = _tmp371 * _tmp422;
  const Scalar _tmp467 = _tmp149 * _tmp390;
  const Scalar _tmp468 = _tmp149 * _tmp393;
  const Scalar _tmp469 =
      (-_tmp454 * _tmp462 +
       _tmp463 * (_tmp149 * _tmp228 * _tmp429 + _tmp149 * _tmp383 * _tmp431 -
                  _tmp149 * _tmp406 * _tmp460 - _tmp189 * _tmp321 * _tmp377 -
                  _tmp219 * _tmp321 * _tmp390 - _tmp228 * _tmp321 * _tmp373 -
                  _tmp230 * _tmp321 * _tmp393 - _tmp260 * _tmp467 + _tmp308 * _tmp468 +
                  _tmp309 * _tmp467 + _tmp336 * _tmp455 - _tmp363 * _tmp468 - _tmp383 * _tmp434 +
                  _tmp388 * _tmp425 + _tmp404 * _tmp459 + _tmp416 * _tmp457 + _tmp420 * _tmp458 +
                  _tmp421 * _tmp456 + _tmp424 * _tmp460 - _tmp461 * _tmp466)) /
      std::sqrt(Scalar(_tmp449 * std::pow(_tmp462, Scalar(2)) + 1));
  const Scalar _tmp470 = -_tmp137 + Scalar(-8.3888750099999996);
  const Scalar _tmp471 = Scalar(2.5202214700000001) - _tmp134;
  const Scalar _tmp472 =
      std::sqrt(Scalar(std::pow(_tmp470, Scalar(2)) + std::pow(_tmp471, Scalar(2))));
  const Scalar _tmp473 = Scalar(9.6622558468725703) * _tmp448;
  const Scalar _tmp474 = -_tmp464 * _tmp473 - _tmp472;
  const Scalar _tmp475 = Scalar(0.1034955) * _tmp463;
  const Scalar _tmp476 = _tmp474 * _tmp475;
  const Scalar _tmp477 = Scalar(9.6622558468725703) * _tmp453;
  const Scalar _tmp478 = _tmp211 * _tmp219 + _tmp230 * _tmp240 + _tmp450 + _tmp452;
  const Scalar _tmp479 = std::pow(_tmp478, Scalar(-2));
  const Scalar _tmp480 = _tmp184 * _tmp219 * _tmp349 + _tmp184 * _tmp230 * _tmp361 -
                         _tmp210 * _tmp219 * _tmp330 - _tmp211 * _tmp260 + _tmp211 * _tmp309 +
                         _tmp221 * _tmp336 - _tmp230 * _tmp238 * _tmp330 + _tmp240 * _tmp308 -
                         _tmp240 * _tmp363 - _tmp302 * _tmp375 - _tmp451;
  const Scalar _tmp481 = _tmp479 * _tmp480;
  const Scalar _tmp482 = Scalar(1.0) / (_tmp478);
  const Scalar _tmp483 = _tmp237 * _tmp370;
  const Scalar _tmp484 = _tmp209 * _tmp370;
  const Scalar _tmp485 = _tmp189 * _tmp370;
  const Scalar _tmp486 = _tmp153 * _tmp485 - _tmp219 * _tmp484 - _tmp228 * _tmp372 -
                         _tmp230 * _tmp483 - _tmp379 * _tmp383 + _tmp460;
  const Scalar _tmp487 = std::asinh(_tmp482 * _tmp486);
  const Scalar _tmp488 = Scalar(9.6622558468725703) * _tmp478;
  const Scalar _tmp489 = Scalar(4.8333311099999996) - _tmp165;
  const Scalar _tmp490 = -_tmp168 + Scalar(-1.79662371);
  const Scalar _tmp491 =
      std::sqrt(Scalar(std::pow(_tmp489, Scalar(2)) + std::pow(_tmp490, Scalar(2))));
  const Scalar _tmp492 = -_tmp487 * _tmp488 - _tmp491;
  const Scalar _tmp493 = Scalar(9.6622558468725703) * _tmp480;
  const Scalar _tmp494 = _tmp230 * _tmp370;
  const Scalar _tmp495 = _tmp219 * _tmp370;
  const Scalar _tmp496 =
      (-_tmp481 * _tmp486 +
       _tmp482 * (_tmp152 * _tmp384 * _tmp409 - _tmp189 * _tmp413 + _tmp209 * _tmp219 * _tmp409 +
                  _tmp228 * _tmp426 - _tmp228 * _tmp427 + _tmp228 * _tmp428 +
                  _tmp230 * _tmp237 * _tmp409 + _tmp260 * _tmp484 - _tmp308 * _tmp483 -
                  _tmp309 * _tmp484 - _tmp336 * _tmp372 - _tmp344 * _tmp495 - _tmp346 * _tmp495 +
                  _tmp348 * _tmp495 + _tmp355 * _tmp494 - _tmp356 * _tmp494 - _tmp360 * _tmp494 +
                  _tmp363 * _tmp483 - _tmp379 * _tmp404 + _tmp383 * _tmp432 - _tmp383 * _tmp433 -
                  _tmp389 * _tmp409 + _tmp412 * _tmp485 + _tmp466)) /
      std::sqrt(Scalar(_tmp479 * std::pow(_tmp486, Scalar(2)) + 1));
  const Scalar _tmp497 = Scalar(0.1034955) * _tmp482;
  const Scalar _tmp498 = _tmp492 * _tmp497;
  const Scalar _tmp499 = Scalar(1.0) * _tmp487;

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) =
      _tmp122 -
      Scalar(0.5) * (2 * _tmp103 * (_tmp82 + _tmp91) + 2 * _tmp80 * (_tmp45 + _tmp63)) *
          std::sinh(Scalar(0.1034955) * _tmp105 *
                    (-_tmp104 - Scalar(9.6622558468725703) * fh1 * std::asinh(_tmp105 * fv1))) /
          _tmp104;
  _res(1, 0) =
      _tmp312 -
      _tmp436 *
          (-Scalar(0.86565325453551001) * _tmp365 + Scalar(1.0) * _tmp435 * std::sinh(_tmp398) -
           (-Scalar(0.1034955) * _tmp365 * _tmp440 +
            _tmp443 * (-_tmp397 * _tmp441 - _tmp435 * _tmp436 -
                       Scalar(1) / Scalar(2) *
                           (2 * _tmp415 * _tmp437 + 2 * _tmp438 * (_tmp247 + _tmp442)) / _tmp439)) *
               std::sinh(_tmp444)) -
      _tmp441 * (Scalar(0.86565325453551001) * _tmp366 + std::cosh(_tmp398) - std::cosh(_tmp444));
  _res(2, 0) =
      _tmp314 -
      _tmp473 *
          (-Scalar(0.87653584775870996) * _tmp454 + Scalar(1.0) * _tmp469 * std::sinh(_tmp465) -
           (-Scalar(0.1034955) * _tmp454 * _tmp474 +
            _tmp475 * (-_tmp464 * _tmp477 - _tmp469 * _tmp473 -
                       Scalar(1) / Scalar(2) *
                           (2 * _tmp470 * (_tmp254 + _tmp91) + 2 * _tmp471 * (_tmp442 + _tmp62)) /
                           _tmp472)) *
               std::sinh(_tmp476)) -
      _tmp477 * (Scalar(0.87653584775870996) * _tmp463 + std::cosh(_tmp465) - std::cosh(_tmp476));
  _res(3, 0) =
      _tmp318 -
      _tmp488 *
          (-Scalar(0.86625939559540499) * _tmp481 + Scalar(1.0) * _tmp496 * std::sinh(_tmp499) -
           (-Scalar(0.1034955) * _tmp481 * _tmp492 +
            _tmp497 * (-_tmp487 * _tmp493 - _tmp488 * _tmp496 -
                       Scalar(1) / Scalar(2) *
                           (2 * _tmp489 * (_tmp289 + _tmp90) + 2 * _tmp490 * (_tmp287 + _tmp45)) /
                           _tmp491)) *
               std::sinh(_tmp498)) -
      _tmp493 * (Scalar(0.86625939559540499) * _tmp482 - std::cosh(_tmp498) + std::cosh(_tmp499));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym