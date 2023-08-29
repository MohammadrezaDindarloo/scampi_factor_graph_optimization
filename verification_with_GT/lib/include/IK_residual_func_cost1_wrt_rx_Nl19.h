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
 * Symbolic function: IK_residual_func_cost1_wrt_rx_Nl19
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost1WrtRxNl19(
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
  const Scalar _tmp13 = -2 * std::pow(_tmp12, Scalar(2));
  const Scalar _tmp14 = _tmp4 * rot_init_z;
  const Scalar _tmp15 = _tmp7 * rot_init_x;
  const Scalar _tmp16 = _tmp11 * rx;
  const Scalar _tmp17 = _tmp14 + _tmp15 * ry - _tmp16 + _tmp9 * rz;
  const Scalar _tmp18 = 1 - 2 * std::pow(_tmp17, Scalar(2));
  const Scalar _tmp19 = Scalar(0.20999999999999999) * _tmp13 + Scalar(0.20999999999999999) * _tmp18;
  const Scalar _tmp20 = _tmp4 * rot_init_y;
  const Scalar _tmp21 = _tmp8 * rx;
  const Scalar _tmp22 = -_tmp15 * rz + _tmp20 + _tmp21 + _tmp9 * ry;
  const Scalar _tmp23 = 2 * _tmp17 * _tmp22;
  const Scalar _tmp24 = _tmp4 * rot_init_w;
  const Scalar _tmp25 = _tmp15 * rx;
  const Scalar _tmp26 = -_tmp11 * ry + _tmp24 - _tmp25 - _tmp8 * rz;
  const Scalar _tmp27 = 2 * _tmp26;
  const Scalar _tmp28 = _tmp12 * _tmp27;
  const Scalar _tmp29 = _tmp23 - _tmp28;
  const Scalar _tmp30 = -Scalar(0.010999999999999999) * _tmp29;
  const Scalar _tmp31 = 2 * _tmp12;
  const Scalar _tmp32 = _tmp22 * _tmp31;
  const Scalar _tmp33 = _tmp17 * _tmp27;
  const Scalar _tmp34 = Scalar(0.20999999999999999) * _tmp32 + Scalar(0.20999999999999999) * _tmp33;
  const Scalar _tmp35 = _tmp30 - _tmp34;
  const Scalar _tmp36 = _tmp19 + _tmp35;
  const Scalar _tmp37 = _tmp36 + p_init1;
  const Scalar _tmp38 = Scalar(4.8333311099999996) - _tmp37;
  const Scalar _tmp39 = -2 * std::pow(_tmp22, Scalar(2));
  const Scalar _tmp40 = Scalar(0.20999999999999999) * _tmp18 + Scalar(0.20999999999999999) * _tmp39;
  const Scalar _tmp41 = -_tmp40;
  const Scalar _tmp42 = Scalar(0.20999999999999999) * _tmp32 - Scalar(0.20999999999999999) * _tmp33;
  const Scalar _tmp43 = _tmp17 * _tmp31;
  const Scalar _tmp44 = _tmp22 * _tmp27;
  const Scalar _tmp45 = _tmp43 + _tmp44;
  const Scalar _tmp46 = -Scalar(0.010999999999999999) * _tmp45;
  const Scalar _tmp47 = _tmp42 + _tmp46;
  const Scalar _tmp48 = _tmp41 + _tmp47;
  const Scalar _tmp49 = _tmp48 + p_init0;
  const Scalar _tmp50 = -_tmp49 + Scalar(-1.79662371);
  const Scalar _tmp51 =
      std::sqrt(Scalar(std::pow(_tmp38, Scalar(2)) + std::pow(_tmp50, Scalar(2))));
  const Scalar _tmp52 = Scalar(1.0) / (fh1);
  const Scalar _tmp53 = (Scalar(1) / Scalar(2)) / _tmp1;
  const Scalar _tmp54 = _tmp0 * _tmp53;
  const Scalar _tmp55 = _tmp53 * rx;
  const Scalar _tmp56 = _tmp55 * rz;
  const Scalar _tmp57 = _tmp55 * ry;
  const Scalar _tmp58 = _tmp6 / (_tmp1 * std::sqrt(_tmp1));
  const Scalar _tmp59 = _tmp0 * _tmp58;
  const Scalar _tmp60 = _tmp58 * rx;
  const Scalar _tmp61 = _tmp60 * ry;
  const Scalar _tmp62 = _tmp60 * rz;
  const Scalar _tmp63 = _tmp14 * _tmp54 - Scalar(1) / Scalar(2) * _tmp16 + _tmp24 * _tmp57 -
                        _tmp5 * _tmp56 - _tmp59 * rot_init_z - _tmp61 * rot_init_w +
                        _tmp62 * rot_init_x + _tmp8;
  const Scalar _tmp64 = Scalar(0.41999999999999998) * _tmp63;
  const Scalar _tmp65 = _tmp12 * _tmp64;
  const Scalar _tmp66 = -_tmp14 * _tmp57 + _tmp20 * _tmp56 + _tmp24 * _tmp54 -
                        Scalar(1) / Scalar(2) * _tmp25 - _tmp59 * rot_init_w + _tmp61 * rot_init_z -
                        _tmp62 * rot_init_y + _tmp9;
  const Scalar _tmp67 = Scalar(0.41999999999999998) * _tmp66;
  const Scalar _tmp68 = _tmp22 * _tmp67;
  const Scalar _tmp69 = -_tmp65 - _tmp68;
  const Scalar _tmp70 = -Scalar(1) / Scalar(2) * _tmp10 - _tmp14 * _tmp56 - _tmp15 -
                        _tmp20 * _tmp57 - _tmp5 * _tmp54 + _tmp59 * rot_init_x +
                        _tmp61 * rot_init_y + _tmp62 * rot_init_z;
  const Scalar _tmp71 = Scalar(0.41999999999999998) * _tmp70;
  const Scalar _tmp72 = _tmp17 * _tmp71;
  const Scalar _tmp73 = -_tmp11 - _tmp20 * _tmp54 - Scalar(1) / Scalar(2) * _tmp21 +
                        _tmp24 * _tmp56 + _tmp5 * _tmp57 + _tmp59 * rot_init_y -
                        _tmp61 * rot_init_x - _tmp62 * rot_init_w;
  const Scalar _tmp74 = Scalar(0.41999999999999998) * _tmp73;
  const Scalar _tmp75 = _tmp26 * _tmp74;
  const Scalar _tmp76 = _tmp72 + _tmp75;
  const Scalar _tmp77 = _tmp69 + _tmp76;
  const Scalar _tmp78 = Scalar(0.021999999999999999) * _tmp70;
  const Scalar _tmp79 = _tmp22 * _tmp78;
  const Scalar _tmp80 = Scalar(0.021999999999999999) * _tmp73;
  const Scalar _tmp81 = _tmp12 * _tmp80;
  const Scalar _tmp82 = Scalar(0.021999999999999999) * _tmp17;
  const Scalar _tmp83 = _tmp66 * _tmp82;
  const Scalar _tmp84 = _tmp26 * _tmp63;
  const Scalar _tmp85 = Scalar(0.021999999999999999) * _tmp84;
  const Scalar _tmp86 = _tmp79 + _tmp81 + _tmp83 + _tmp85;
  const Scalar _tmp87 = Scalar(0.83999999999999997) * _tmp17;
  const Scalar _tmp88 = _tmp73 * _tmp87;
  const Scalar _tmp89 = -_tmp88;
  const Scalar _tmp90 = Scalar(0.83999999999999997) * _tmp63;
  const Scalar _tmp91 = _tmp22 * _tmp90;
  const Scalar _tmp92 = _tmp89 - _tmp91;
  const Scalar _tmp93 = _tmp86 + _tmp92;
  const Scalar _tmp94 = Scalar(0.83999999999999997) * _tmp66;
  const Scalar _tmp95 = _tmp12 * _tmp94;
  const Scalar _tmp96 = _tmp88 + _tmp95;
  const Scalar _tmp97 = _tmp65 + _tmp68;
  const Scalar _tmp98 = _tmp76 + _tmp97;
  const Scalar _tmp99 = _tmp96 + _tmp98;
  const Scalar _tmp100 = _tmp12 * _tmp78;
  const Scalar _tmp101 = _tmp22 * _tmp80;
  const Scalar _tmp102 = _tmp63 * _tmp82;
  const Scalar _tmp103 = _tmp26 * _tmp66;
  const Scalar _tmp104 = Scalar(0.021999999999999999) * _tmp103;
  const Scalar _tmp105 = -_tmp100 + _tmp101 + _tmp102 - _tmp104;
  const Scalar _tmp106 = _tmp22 * _tmp71;
  const Scalar _tmp107 = _tmp12 * _tmp74;
  const Scalar _tmp108 = _tmp17 * _tmp67;
  const Scalar _tmp109 = _tmp26 * _tmp64;
  const Scalar _tmp110 = _tmp106 - _tmp107 - _tmp108 + _tmp109;
  const Scalar _tmp111 = _tmp12 * _tmp71;
  const Scalar _tmp112 = _tmp22 * _tmp74;
  const Scalar _tmp113 = _tmp17 * _tmp64;
  const Scalar _tmp114 = _tmp26 * _tmp67;
  const Scalar _tmp115 = _tmp22 * _tmp63;
  const Scalar _tmp116 = Scalar(0.043999999999999997) * _tmp115;
  const Scalar _tmp117 = _tmp12 * _tmp66;
  const Scalar _tmp118 = Scalar(0.043999999999999997) * _tmp117;
  const Scalar _tmp119 = _tmp116 + _tmp118;
  const Scalar _tmp120 = _tmp111 + _tmp112 + _tmp113 + _tmp114 + _tmp119;
  const Scalar _tmp121 = _tmp110 + _tmp120;
  const Scalar _tmp122 = -_tmp42 + _tmp46;
  const Scalar _tmp123 = _tmp122 + _tmp40;
  const Scalar _tmp124 = _tmp123 + p_init0;
  const Scalar _tmp125 = _tmp124 + Scalar(-2.5202214700000001);
  const Scalar _tmp126 = -_tmp19;
  const Scalar _tmp127 = _tmp30 + _tmp34;
  const Scalar _tmp128 = _tmp126 + _tmp127;
  const Scalar _tmp129 = _tmp128 + p_init1;
  const Scalar _tmp130 = _tmp129 + Scalar(8.3888750099999996);
  const Scalar _tmp131 = std::pow(_tmp125, Scalar(2)) + std::pow(_tmp130, Scalar(2));
  const Scalar _tmp132 = std::pow(_tmp131, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp133 = _tmp125 * _tmp132;
  const Scalar _tmp134 =
      Scalar(0.20999999999999999) * _tmp23 + Scalar(0.20999999999999999) * _tmp28;
  const Scalar _tmp135 = -_tmp134;
  const Scalar _tmp136 = -Scalar(0.010999999999999999) * _tmp13 -
                         Scalar(0.010999999999999999) * _tmp39 + Scalar(-0.010999999999999999);
  const Scalar _tmp137 =
      Scalar(0.20999999999999999) * _tmp43 - Scalar(0.20999999999999999) * _tmp44;
  const Scalar _tmp138 = _tmp136 + _tmp137;
  const Scalar _tmp139 = _tmp135 + _tmp138;
  const Scalar _tmp140 = _tmp130 * _tmp132;
  const Scalar _tmp141 = _tmp126 + _tmp35;
  const Scalar _tmp142 = _tmp141 + p_init1;
  const Scalar _tmp143 = _tmp142 + Scalar(8.3196563700000006);
  const Scalar _tmp144 = _tmp122 + _tmp41;
  const Scalar _tmp145 = _tmp144 + p_init0;
  const Scalar _tmp146 = _tmp145 + Scalar(1.9874742000000001);
  const Scalar _tmp147 = Scalar(1.0) / (_tmp146);
  const Scalar _tmp148 = _tmp143 * _tmp147;
  const Scalar _tmp149 = _tmp136 - _tmp137;
  const Scalar _tmp150 = _tmp135 + _tmp149;
  const Scalar _tmp151 = _tmp127 + _tmp19;
  const Scalar _tmp152 = _tmp151 + p_init1;
  const Scalar _tmp153 = _tmp152 + Scalar(-4.7752063900000001);
  const Scalar _tmp154 = _tmp40 + _tmp47;
  const Scalar _tmp155 = _tmp154 + p_init0;
  const Scalar _tmp156 = _tmp155 + Scalar(-2.71799795);
  const Scalar _tmp157 = std::pow(_tmp153, Scalar(2)) + std::pow(_tmp156, Scalar(2));
  const Scalar _tmp158 = std::pow(_tmp157, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp159 = _tmp156 * _tmp158;
  const Scalar _tmp160 = _tmp150 * _tmp159;
  const Scalar _tmp161 = _tmp134 + _tmp138;
  const Scalar _tmp162 = _tmp153 * _tmp158;
  const Scalar _tmp163 = -_tmp148 * _tmp160 + _tmp161 * _tmp162;
  const Scalar _tmp164 = _tmp148 * _tmp159 - _tmp162;
  const Scalar _tmp165 = Scalar(1.0) / (_tmp164);
  const Scalar _tmp166 = _tmp133 * _tmp148 - _tmp140;
  const Scalar _tmp167 = _tmp165 * _tmp166;
  const Scalar _tmp168 = _tmp133 * _tmp150;
  const Scalar _tmp169 = _tmp139 * _tmp140 - _tmp148 * _tmp168 - _tmp163 * _tmp167;
  const Scalar _tmp170 = Scalar(1.0) * _tmp141;
  const Scalar _tmp171 = -_tmp170;
  const Scalar _tmp172 = _tmp151 + _tmp171;
  const Scalar _tmp173 = Scalar(1.0) / (_tmp172);
  const Scalar _tmp174 = Scalar(1.0) * _tmp144;
  const Scalar _tmp175 = -_tmp154 + _tmp174;
  const Scalar _tmp176 = _tmp173 * _tmp175;
  const Scalar _tmp177 = -_tmp159 * _tmp161 + _tmp160;
  const Scalar _tmp178 = _tmp165 * _tmp177;
  const Scalar _tmp179 = -_tmp133 * _tmp139 - _tmp166 * _tmp178 + _tmp168 - _tmp169 * _tmp176;
  const Scalar _tmp180 = Scalar(1.0) / (_tmp179);
  const Scalar _tmp181 = std::pow(_tmp146, Scalar(2));
  const Scalar _tmp182 = std::pow(_tmp143, Scalar(2)) + _tmp181;
  const Scalar _tmp183 = std::sqrt(_tmp182);
  const Scalar _tmp184 = Scalar(1.0) / (_tmp183);
  const Scalar _tmp185 = _tmp146 * _tmp184;
  const Scalar _tmp186 = _tmp144 * _tmp184;
  const Scalar _tmp187 = -_tmp141 * _tmp185 + _tmp143 * _tmp186;
  const Scalar _tmp188 = _tmp147 * _tmp183;
  const Scalar _tmp189 = _tmp187 * _tmp188;
  const Scalar _tmp190 = _tmp151 * _tmp159 - _tmp154 * _tmp162 + _tmp159 * _tmp189;
  const Scalar _tmp191 = _tmp165 * _tmp190;
  const Scalar _tmp192 =
      -_tmp123 * _tmp140 + _tmp128 * _tmp133 + _tmp133 * _tmp189 - _tmp166 * _tmp191;
  const Scalar _tmp193 = _tmp163 * _tmp165;
  const Scalar _tmp194 = _tmp148 * _tmp150;
  const Scalar _tmp195 = _tmp148 * _tmp193 + _tmp194;
  const Scalar _tmp196 = _tmp148 * _tmp178 - _tmp150 - _tmp176 * _tmp195;
  const Scalar _tmp197 = _tmp180 * _tmp196;
  const Scalar _tmp198 = _tmp148 * _tmp191 - _tmp189 - _tmp192 * _tmp197;
  const Scalar _tmp199 = Scalar(1.0) / (_tmp192);
  const Scalar _tmp200 = _tmp179 * _tmp199;
  const Scalar _tmp201 = _tmp198 * _tmp200;
  const Scalar _tmp202 = _tmp196 + _tmp201;
  const Scalar _tmp203 = _tmp180 * _tmp202;
  const Scalar _tmp204 = _tmp166 * _tmp180;
  const Scalar _tmp205 = -_tmp148 - _tmp202 * _tmp204;
  const Scalar _tmp206 = _tmp159 * _tmp165;
  const Scalar _tmp207 = _tmp133 * _tmp203 + _tmp205 * _tmp206 + Scalar(1.0);
  const Scalar _tmp208 = _tmp37 + Scalar(-4.8333311099999996);
  const Scalar _tmp209 = _tmp49 + Scalar(1.79662371);
  const Scalar _tmp210 = std::pow(_tmp208, Scalar(2)) + std::pow(_tmp209, Scalar(2));
  const Scalar _tmp211 = std::pow(_tmp210, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp212 = _tmp209 * _tmp211;
  const Scalar _tmp213 = _tmp212 * fh1;
  const Scalar _tmp214 = _tmp188 * _tmp213;
  const Scalar _tmp215 = _tmp211 * _tmp48;
  const Scalar _tmp216 = _tmp211 * _tmp36;
  const Scalar _tmp217 = _tmp208 * _tmp215 - _tmp209 * _tmp216;
  const Scalar _tmp218 = _tmp217 * fh1;
  const Scalar _tmp219 = Scalar(1.0) * _tmp199;
  const Scalar _tmp220 = _tmp167 * _tmp219;
  const Scalar _tmp221 = _tmp133 * _tmp219 - _tmp159 * _tmp220;
  const Scalar _tmp222 = _tmp188 * _tmp221;
  const Scalar _tmp223 = _tmp208 * _tmp211;
  const Scalar _tmp224 = _tmp223 * fh1;
  const Scalar _tmp225 = Scalar(1.0) * _tmp173;
  const Scalar _tmp226 = _tmp175 * _tmp225;
  const Scalar _tmp227 = -Scalar(1.0) * _tmp178 + _tmp193 * _tmp226;
  const Scalar _tmp228 = _tmp180 * _tmp192;
  const Scalar _tmp229 = -Scalar(1.0) * _tmp191 - _tmp227 * _tmp228;
  const Scalar _tmp230 = _tmp200 * _tmp229;
  const Scalar _tmp231 = _tmp227 + _tmp230;
  const Scalar _tmp232 = _tmp180 * _tmp231;
  const Scalar _tmp233 = -_tmp204 * _tmp231 + Scalar(1.0);
  const Scalar _tmp234 = _tmp133 * _tmp232 + _tmp206 * _tmp233;
  const Scalar _tmp235 = _tmp188 * _tmp234;
  const Scalar _tmp236 = _tmp170 * _tmp176 + _tmp174;
  const Scalar _tmp237 = 0;
  const Scalar _tmp238 = _tmp180 * _tmp237;
  const Scalar _tmp239 = _tmp167 * _tmp238;
  const Scalar _tmp240 = _tmp133 * _tmp238 - _tmp159 * _tmp239;
  const Scalar _tmp241 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp242 = _tmp188 * _tmp241;
  const Scalar _tmp243 =
      -_tmp207 * _tmp214 - _tmp218 * _tmp222 - _tmp224 * _tmp235 - _tmp240 * _tmp242;
  const Scalar _tmp244 = std::pow(_tmp243, Scalar(-2));
  const Scalar _tmp245 = _tmp88 + _tmp91;
  const Scalar _tmp246 = -_tmp72 - _tmp75;
  const Scalar _tmp247 = _tmp246 + _tmp97;
  const Scalar _tmp248 = -_tmp79 - _tmp81 - _tmp83 - _tmp85;
  const Scalar _tmp249 = _tmp247 + _tmp248;
  const Scalar _tmp250 = _tmp245 + _tmp249;
  const Scalar _tmp251 = _tmp246 + _tmp69;
  const Scalar _tmp252 = _tmp100 - _tmp101 - _tmp102 + _tmp104;
  const Scalar _tmp253 = _tmp89 - _tmp95;
  const Scalar _tmp254 = _tmp252 + _tmp253;
  const Scalar _tmp255 = _tmp251 + _tmp254;
  const Scalar _tmp256 =
      (2 * _tmp208 * _tmp255 + 2 * _tmp209 * _tmp250) / (_tmp210 * std::sqrt(_tmp210));
  const Scalar _tmp257 = (Scalar(1) / Scalar(2)) * _tmp256;
  const Scalar _tmp258 = _tmp208 * _tmp257;
  const Scalar _tmp259 = _tmp258 * fh1;
  const Scalar _tmp260 = _tmp211 * _tmp255 * fh1;
  const Scalar _tmp261 = _tmp248 + _tmp77;
  const Scalar _tmp262 = _tmp245 + _tmp261;
  const Scalar _tmp263 = Scalar(1.0) / (_tmp181);
  const Scalar _tmp264 = _tmp183 * _tmp262 * _tmp263;
  const Scalar _tmp265 = _tmp240 * _tmp241;
  const Scalar _tmp266 = _tmp261 + _tmp92;
  const Scalar _tmp267 = _tmp132 * _tmp266;
  const Scalar _tmp268 = _tmp249 + _tmp92;
  const Scalar _tmp269 = _tmp158 * _tmp268;
  const Scalar _tmp270 = _tmp165 * _tmp269;
  const Scalar _tmp271 = _tmp254 + _tmp98;
  const Scalar _tmp272 =
      (2 * _tmp153 * _tmp271 + 2 * _tmp156 * _tmp268) / (_tmp157 * std::sqrt(_tmp157));
  const Scalar _tmp273 = (Scalar(1) / Scalar(2)) * _tmp272;
  const Scalar _tmp274 = _tmp156 * _tmp273;
  const Scalar _tmp275 = _tmp165 * _tmp233;
  const Scalar _tmp276 = _tmp158 * _tmp271;
  const Scalar _tmp277 = _tmp153 * _tmp273;
  const Scalar _tmp278 = _tmp143 * _tmp262;
  const Scalar _tmp279 = _tmp263 * _tmp278;
  const Scalar _tmp280 = _tmp251 + _tmp96;
  const Scalar _tmp281 = _tmp252 + _tmp280;
  const Scalar _tmp282 = _tmp147 * _tmp281;
  const Scalar _tmp283 = (_tmp148 * _tmp269 - _tmp148 * _tmp274 - _tmp159 * _tmp279 +
                          _tmp159 * _tmp282 - _tmp276 + _tmp277) /
                         std::pow(_tmp164, Scalar(2));
  const Scalar _tmp284 = _tmp159 * _tmp283;
  const Scalar _tmp285 = _tmp252 + _tmp99;
  const Scalar _tmp286 =
      (2 * _tmp125 * _tmp266 + 2 * _tmp130 * _tmp285) / (_tmp131 * std::sqrt(_tmp131));
  const Scalar _tmp287 = (Scalar(1) / Scalar(2)) * _tmp286;
  const Scalar _tmp288 = _tmp125 * _tmp287;
  const Scalar _tmp289 = _tmp130 * _tmp287;
  const Scalar _tmp290 = _tmp132 * _tmp285;
  const Scalar _tmp291 = _tmp143 * _tmp281 + _tmp146 * _tmp262;
  const Scalar _tmp292 = _tmp291 / (_tmp182 * std::sqrt(_tmp182));
  const Scalar _tmp293 = _tmp188 * (_tmp141 * _tmp146 * _tmp292 - _tmp141 * _tmp184 * _tmp262 -
                                    _tmp143 * _tmp144 * _tmp292 + _tmp184 * _tmp278 -
                                    _tmp185 * _tmp281 + _tmp186 * _tmp281);
  const Scalar _tmp294 = _tmp187 * _tmp264;
  const Scalar _tmp295 = _tmp147 * _tmp184 * _tmp291;
  const Scalar _tmp296 = _tmp187 * _tmp295;
  const Scalar _tmp297 = _tmp151 * _tmp269 - _tmp151 * _tmp274 - _tmp154 * _tmp276 +
                         _tmp154 * _tmp277 + _tmp156 * _tmp276 + _tmp159 * _tmp293 -
                         _tmp159 * _tmp294 + _tmp159 * _tmp296 - _tmp162 * _tmp268 +
                         _tmp189 * _tmp269 - _tmp189 * _tmp274;
  const Scalar _tmp298 = _tmp166 * _tmp283;
  const Scalar _tmp299 = -_tmp133 * _tmp279 + _tmp133 * _tmp282 + _tmp148 * _tmp267 -
                         _tmp148 * _tmp288 + _tmp289 - _tmp290;
  const Scalar _tmp300 = _tmp123 * _tmp289 - _tmp123 * _tmp290 + _tmp125 * _tmp290 +
                         _tmp128 * _tmp267 - _tmp128 * _tmp288 + _tmp133 * _tmp293 -
                         _tmp133 * _tmp294 + _tmp133 * _tmp296 - _tmp140 * _tmp266 -
                         _tmp167 * _tmp297 + _tmp189 * _tmp267 - _tmp189 * _tmp288 +
                         _tmp190 * _tmp298 - _tmp191 * _tmp299;
  const Scalar _tmp301 = _tmp300 / std::pow(_tmp192, Scalar(2));
  const Scalar _tmp302 = _tmp179 * _tmp301;
  const Scalar _tmp303 = _tmp229 * _tmp302;
  const Scalar _tmp304 = _tmp150 * _tmp267;
  const Scalar _tmp305 = -_tmp111 - _tmp112 - _tmp113 - _tmp114 + _tmp119;
  const Scalar _tmp306 = -_tmp106 + _tmp107 + _tmp108 - _tmp109;
  const Scalar _tmp307 = _tmp305 + _tmp306;
  const Scalar _tmp308 = _tmp110 + _tmp305;
  const Scalar _tmp309 = _tmp133 * _tmp308;
  const Scalar _tmp310 = _tmp120 + _tmp306;
  const Scalar _tmp311 = _tmp148 * _tmp308;
  const Scalar _tmp312 = _tmp150 * _tmp282;
  const Scalar _tmp313 = -_tmp159 * _tmp311 - _tmp159 * _tmp312 + _tmp160 * _tmp279 +
                         _tmp161 * _tmp276 - _tmp161 * _tmp277 + _tmp162 * _tmp310 -
                         _tmp194 * _tmp269 + _tmp194 * _tmp274;
  const Scalar _tmp314 = -_tmp139 * _tmp289 + _tmp139 * _tmp290 + _tmp140 * _tmp307 -
                         _tmp148 * _tmp304 - _tmp148 * _tmp309 + _tmp163 * _tmp298 -
                         _tmp167 * _tmp313 + _tmp168 * _tmp279 - _tmp168 * _tmp282 -
                         _tmp193 * _tmp299 + _tmp194 * _tmp288;
  const Scalar _tmp315 = Scalar(1.6799999999999999) * _tmp17 * _tmp73;
  const Scalar _tmp316 = _tmp12 * _tmp90;
  const Scalar _tmp317 = _tmp22 * _tmp94;
  const Scalar _tmp318 = Scalar(0.83999999999999997) * _tmp26 * _tmp73 + _tmp70 * _tmp87;
  const Scalar _tmp319 = _tmp316 + _tmp317 + _tmp318;
  const Scalar _tmp320 =
      (-Scalar(1.6799999999999999) * _tmp117 - _tmp315 + _tmp319) / std::pow(_tmp172, Scalar(2));
  const Scalar _tmp321 = _tmp175 * _tmp320;
  const Scalar _tmp322 = _tmp150 * _tmp269 - _tmp150 * _tmp274 + _tmp159 * _tmp308 -
                         _tmp159 * _tmp310 - _tmp161 * _tmp269 + _tmp161 * _tmp274;
  const Scalar _tmp323 = Scalar(1.6799999999999999) * _tmp115 + _tmp315;
  const Scalar _tmp324 = -_tmp316 - _tmp317 + _tmp318 + _tmp323;
  const Scalar _tmp325 = _tmp173 * _tmp324;
  const Scalar _tmp326 = -_tmp133 * _tmp307 - _tmp139 * _tmp267 + _tmp139 * _tmp288 -
                         _tmp150 * _tmp288 - _tmp167 * _tmp322 + _tmp169 * _tmp321 -
                         _tmp169 * _tmp325 - _tmp176 * _tmp314 + _tmp177 * _tmp298 -
                         _tmp178 * _tmp299 + _tmp304 + _tmp309;
  const Scalar _tmp327 = _tmp199 * _tmp326;
  const Scalar _tmp328 = _tmp229 * _tmp327;
  const Scalar _tmp329 = Scalar(1.0) * _tmp165;
  const Scalar _tmp330 = _tmp326 / std::pow(_tmp179, Scalar(2));
  const Scalar _tmp331 = _tmp192 * _tmp330;
  const Scalar _tmp332 = Scalar(1.0) * _tmp283;
  const Scalar _tmp333 = _tmp163 * _tmp329;
  const Scalar _tmp334 = -_tmp163 * _tmp226 * _tmp283 + _tmp165 * _tmp226 * _tmp313 +
                         _tmp177 * _tmp332 + _tmp193 * _tmp225 * _tmp324 - _tmp321 * _tmp333 -
                         _tmp322 * _tmp329;
  const Scalar _tmp335 = _tmp200 * (-_tmp180 * _tmp227 * _tmp300 + _tmp190 * _tmp332 +
                                    _tmp227 * _tmp331 - _tmp228 * _tmp334 - _tmp297 * _tmp329);
  const Scalar _tmp336 = -_tmp303 + _tmp328 + _tmp334 + _tmp335;
  const Scalar _tmp337 = _tmp133 * _tmp180;
  const Scalar _tmp338 = _tmp166 * _tmp330;
  const Scalar _tmp339 = -_tmp204 * _tmp336 + _tmp231 * _tmp338 - _tmp232 * _tmp299;
  const Scalar _tmp340 = _tmp133 * _tmp330;
  const Scalar _tmp341 = _tmp218 * _tmp221;
  const Scalar _tmp342 = _tmp224 * _tmp234;
  const Scalar _tmp343 = _tmp211 * _tmp250;
  const Scalar _tmp344 = _tmp343 * fh1;
  const Scalar _tmp345 = _tmp188 * _tmp207;
  const Scalar _tmp346 = Scalar(0.5) * _tmp199;
  const Scalar _tmp347 = _tmp206 * _tmp299;
  const Scalar _tmp348 = Scalar(1.0) * _tmp301;
  const Scalar _tmp349 = _tmp159 * _tmp167;
  const Scalar _tmp350 = _tmp159 * _tmp298;
  const Scalar _tmp351 = _tmp237 * _tmp330;
  const Scalar _tmp352 = _tmp207 * _tmp213;
  const Scalar _tmp353 = _tmp209 * _tmp257;
  const Scalar _tmp354 = _tmp353 * fh1;
  const Scalar _tmp355 = _tmp165 * _tmp205;
  const Scalar _tmp356 = _tmp198 * _tmp302;
  const Scalar _tmp357 = _tmp198 * _tmp327;
  const Scalar _tmp358 = _tmp148 * _tmp283;
  const Scalar _tmp359 = _tmp148 * _tmp165;
  const Scalar _tmp360 = -_tmp150 * _tmp279 - _tmp163 * _tmp358 - _tmp193 * _tmp279 +
                         _tmp193 * _tmp282 + _tmp311 + _tmp312 + _tmp313 * _tmp359;
  const Scalar _tmp361 = _tmp111 + _tmp112 + _tmp113 + _tmp114 - _tmp116 - _tmp118 -
                         _tmp176 * _tmp360 - _tmp177 * _tmp358 - _tmp178 * _tmp279 +
                         _tmp178 * _tmp282 + _tmp195 * _tmp321 - _tmp195 * _tmp325 + _tmp306 +
                         _tmp322 * _tmp359;
  const Scalar _tmp362 = _tmp200 * (-_tmp190 * _tmp358 - _tmp191 * _tmp279 + _tmp191 * _tmp282 +
                                    _tmp196 * _tmp331 - _tmp197 * _tmp300 - _tmp228 * _tmp361 -
                                    _tmp293 + _tmp294 - _tmp296 + _tmp297 * _tmp359);
  const Scalar _tmp363 = -_tmp356 + _tmp357 + _tmp361 + _tmp362;
  const Scalar _tmp364 =
      _tmp202 * _tmp338 - _tmp203 * _tmp299 - _tmp204 * _tmp363 + _tmp279 - _tmp282;
  const Scalar _tmp365 = _tmp208 * _tmp343 - _tmp212 * _tmp255 + _tmp215 * _tmp255 -
                         _tmp216 * _tmp250 - _tmp258 * _tmp48 + _tmp353 * _tmp36;
  const Scalar _tmp366 = _tmp365 * fh1;
  const Scalar _tmp367 =
      -_tmp188 * _tmp218 *
          (-_tmp125 * _tmp286 * _tmp346 - _tmp133 * _tmp348 +
           _tmp156 * _tmp167 * _tmp272 * _tmp346 + _tmp219 * _tmp267 - _tmp219 * _tmp347 +
           _tmp219 * _tmp350 - _tmp220 * _tmp269 + _tmp348 * _tmp349) -
      _tmp188 * _tmp224 *
          (_tmp206 * _tmp339 - _tmp231 * _tmp340 + _tmp232 * _tmp267 - _tmp232 * _tmp288 +
           _tmp233 * _tmp270 - _tmp233 * _tmp284 - _tmp274 * _tmp275 + _tmp336 * _tmp337) -
      _tmp214 * (-_tmp202 * _tmp340 + _tmp203 * _tmp267 - _tmp203 * _tmp288 + _tmp205 * _tmp270 -
                 _tmp205 * _tmp284 + _tmp206 * _tmp364 - _tmp274 * _tmp355 + _tmp337 * _tmp363) -
      _tmp222 * _tmp366 + _tmp235 * _tmp259 - _tmp235 * _tmp260 -
      _tmp242 * (-_tmp133 * _tmp351 + _tmp238 * _tmp267 - _tmp238 * _tmp288 - _tmp238 * _tmp347 +
                 _tmp238 * _tmp350 - _tmp239 * _tmp269 + _tmp239 * _tmp274 + _tmp349 * _tmp351) +
      _tmp264 * _tmp265 + _tmp264 * _tmp341 + _tmp264 * _tmp342 + _tmp264 * _tmp352 -
      _tmp265 * _tmp295 - _tmp295 * _tmp341 - _tmp295 * _tmp342 - _tmp295 * _tmp352 -
      _tmp344 * _tmp345 + _tmp345 * _tmp354;
  const Scalar _tmp368 = _tmp244 * _tmp367;
  const Scalar _tmp369 = _tmp128 + _tmp171;
  const Scalar _tmp370 = _tmp176 * _tmp369;
  const Scalar _tmp371 = -_tmp123 + _tmp174 - _tmp370;
  const Scalar _tmp372 = Scalar(1.0) / (_tmp371);
  const Scalar _tmp373 = Scalar(1.0) * _tmp372;
  const Scalar _tmp374 = _tmp200 * _tmp373;
  const Scalar _tmp375 = -_tmp169 * _tmp219 + _tmp369 * _tmp374;
  const Scalar _tmp376 = -_tmp225 * _tmp375 + _tmp374;
  const Scalar _tmp377 = Scalar(1.0) * _tmp218;
  const Scalar _tmp378 = fh1 * (_tmp134 + _tmp149);
  const Scalar _tmp379 = _tmp211 * _tmp378;
  const Scalar _tmp380 = -_tmp208 * _tmp379 - Scalar(3.29616) * _tmp29 - _tmp36 * fv1;
  const Scalar _tmp381 = _tmp370 * _tmp373 + Scalar(1.0);
  const Scalar _tmp382 = _tmp176 * _tmp373;
  const Scalar _tmp383 = -Scalar(1.0) * _tmp225 * _tmp381 + Scalar(1.0) * _tmp382;
  const Scalar _tmp384 = _tmp236 * _tmp372;
  const Scalar _tmp385 = -_tmp169 * _tmp238 + _tmp171 - _tmp369 * _tmp384;
  const Scalar _tmp386 = Scalar(1.0) * _tmp241;
  const Scalar _tmp387 = _tmp169 * _tmp180;
  const Scalar _tmp388 = _tmp369 * _tmp372;
  const Scalar _tmp389 = _tmp230 * _tmp388 - _tmp231 * _tmp387 - _tmp333;
  const Scalar _tmp390 = -_tmp225 * _tmp389 + _tmp230 * _tmp373;
  const Scalar _tmp391 = Scalar(1.0) * _tmp224;
  const Scalar _tmp392 = _tmp195 + _tmp201 * _tmp388 - _tmp202 * _tmp387;
  const Scalar _tmp393 = _tmp201 * _tmp373 - _tmp225 * _tmp392;
  const Scalar _tmp394 = Scalar(1.0) * _tmp393;
  const Scalar _tmp395 = _tmp212 * _tmp378 + Scalar(3.29616) * _tmp45 + _tmp48 * fv1;
  const Scalar _tmp396 = _tmp173 * _tmp373;
  const Scalar _tmp397 = Scalar(1.0) * _tmp369 * _tmp396 - Scalar(1.0) * _tmp373;
  const Scalar _tmp398 = _tmp213 * _tmp394 + _tmp376 * _tmp377 + _tmp380 * _tmp383 +
                         _tmp386 * (-_tmp225 * _tmp385 - _tmp236 * _tmp373 + Scalar(1.0)) +
                         _tmp390 * _tmp391 + _tmp395 * _tmp397;
  const Scalar _tmp399 = Scalar(1.0) / (_tmp243);
  const Scalar _tmp400 = std::asinh(_tmp398 * _tmp399);
  const Scalar _tmp401 = Scalar(1.0) * _tmp400;
  const Scalar _tmp402 = Scalar(6.59232) * _tmp70;
  const Scalar _tmp403 = Scalar(6.59232) * _tmp73;
  const Scalar _tmp404 = Scalar(6.59232) * _tmp17;
  const Scalar _tmp405 = _tmp121 * fh1;
  const Scalar _tmp406 = _tmp12 * _tmp403 + _tmp212 * _tmp405 + _tmp22 * _tmp402 + _tmp250 * fv1 +
                         _tmp343 * _tmp378 - _tmp353 * _tmp378 + _tmp404 * _tmp66 +
                         Scalar(6.59232) * _tmp84;
  const Scalar _tmp407 = _tmp321 * _tmp369;
  const Scalar _tmp408 = _tmp325 * _tmp369;
  const Scalar _tmp409 = _tmp176 * _tmp319;
  const Scalar _tmp410 = (_tmp323 + _tmp407 - _tmp408 - _tmp409) / std::pow(_tmp371, Scalar(2));
  const Scalar _tmp411 = _tmp369 * _tmp410;
  const Scalar _tmp412 = _tmp169 * _tmp330;
  const Scalar _tmp413 = _tmp319 * _tmp372;
  const Scalar _tmp414 = -_tmp201 * _tmp411 + _tmp201 * _tmp413 + _tmp202 * _tmp412 -
                         _tmp203 * _tmp314 - _tmp356 * _tmp388 + _tmp357 * _tmp388 + _tmp360 +
                         _tmp362 * _tmp388 - _tmp363 * _tmp387;
  const Scalar _tmp415 = Scalar(1.0) * _tmp410;
  const Scalar _tmp416 = Scalar(1.0) * _tmp320;
  const Scalar _tmp417 = Scalar(0.5) * _tmp256 * fh1;
  const Scalar _tmp418 = _tmp163 * _tmp332 - _tmp230 * _tmp411 + _tmp230 * _tmp413 +
                         _tmp231 * _tmp412 - _tmp232 * _tmp314 - _tmp303 * _tmp388 -
                         _tmp313 * _tmp329 + _tmp328 * _tmp388 + _tmp335 * _tmp388 -
                         _tmp336 * _tmp387;
  const Scalar _tmp419 = _tmp225 * _tmp411;
  const Scalar _tmp420 = _tmp369 * _tmp373;
  const Scalar _tmp421 = _tmp226 * _tmp410;
  const Scalar _tmp422 =
      -_tmp369 * _tmp421 - _tmp373 * _tmp407 + _tmp373 * _tmp408 + _tmp373 * _tmp409;
  const Scalar _tmp423 = _tmp321 * _tmp373;
  const Scalar _tmp424 = -_tmp170 * _tmp321 + _tmp170 * _tmp325 + _tmp226 * _tmp281 + _tmp262;
  const Scalar _tmp425 = _tmp320 * _tmp385;
  const Scalar _tmp426 = _tmp372 * _tmp424;
  const Scalar _tmp427 = _tmp236 * _tmp410;
  const Scalar _tmp428 = _tmp105 + _tmp253;
  const Scalar _tmp429 = _tmp428 + _tmp98;
  const Scalar _tmp430 = _tmp237 * _tmp412 - _tmp238 * _tmp314 - _tmp319 * _tmp384 -
                         _tmp369 * _tmp426 + _tmp369 * _tmp427 + _tmp429;
  const Scalar _tmp431 = _tmp200 * _tmp415;
  const Scalar _tmp432 = _tmp302 * _tmp373;
  const Scalar _tmp433 = _tmp327 * _tmp373;
  const Scalar _tmp434 = _tmp169 * _tmp348 - _tmp219 * _tmp314 - _tmp302 * _tmp420 +
                         _tmp319 * _tmp374 - _tmp369 * _tmp431 + _tmp369 * _tmp433;
  const Scalar _tmp435 = Scalar(6.59232) * _tmp103 + _tmp12 * _tmp402 - _tmp22 * _tmp403 -
                         _tmp223 * _tmp405 - _tmp255 * _tmp379 - _tmp255 * fv1 + _tmp258 * _tmp378 -
                         _tmp404 * _tmp63;
  const Scalar _tmp436 =
      (-_tmp368 * _tmp398 +
       _tmp399 *
           (-_tmp208 * _tmp390 * _tmp417 - _tmp209 * _tmp393 * _tmp417 +
            Scalar(1.0) * _tmp213 *
                (-_tmp201 * _tmp415 - _tmp225 * _tmp414 - _tmp356 * _tmp373 + _tmp357 * _tmp373 +
                 _tmp362 * _tmp373 + _tmp392 * _tmp416) +
            Scalar(1.0) * _tmp260 * _tmp390 + _tmp344 * _tmp394 + Scalar(1.0) * _tmp366 * _tmp376 +
            _tmp377 * (-_tmp225 * _tmp434 + _tmp375 * _tmp416 - _tmp431 - _tmp432 + _tmp433) +
            Scalar(1.0) * _tmp380 *
                (-_tmp225 * _tmp422 + _tmp325 * _tmp373 + _tmp381 * _tmp416 - _tmp421 - _tmp423) +
            _tmp383 * _tmp435 +
            _tmp386 * (-_tmp225 * _tmp430 + _tmp236 * _tmp415 - _tmp373 * _tmp424 +
                       Scalar(1.0) * _tmp425) +
            _tmp391 * (-_tmp225 * _tmp418 - _tmp230 * _tmp415 - _tmp303 * _tmp373 +
                       _tmp328 * _tmp373 + _tmp335 * _tmp373 + _tmp389 * _tmp416) +
            Scalar(1.0) * _tmp395 * (_tmp319 * _tmp396 - _tmp320 * _tmp420 + _tmp415 - _tmp419) +
            _tmp397 * _tmp406)) /
      std::sqrt(Scalar(_tmp244 * std::pow(_tmp398, Scalar(2)) + 1));
  const Scalar _tmp437 = Scalar(9.6622558468725703) * _tmp243;
  const Scalar _tmp438 = -_tmp142 + Scalar(-8.3196563700000006);
  const Scalar _tmp439 = -_tmp145 + Scalar(-1.9874742000000001);
  const Scalar _tmp440 =
      std::sqrt(Scalar(std::pow(_tmp438, Scalar(2)) + std::pow(_tmp439, Scalar(2))));
  const Scalar _tmp441 = -_tmp400 * _tmp437 - _tmp440;
  const Scalar _tmp442 = Scalar(0.1034955) * _tmp399;
  const Scalar _tmp443 = _tmp441 * _tmp442;
  const Scalar _tmp444 = Scalar(9.6622558468725703) * _tmp367;
  const Scalar _tmp445 = _tmp165 * _tmp213;
  const Scalar _tmp446 = _tmp238 * _tmp241;
  const Scalar _tmp447 = _tmp219 * fh1;
  const Scalar _tmp448 = _tmp217 * _tmp447;
  const Scalar _tmp449 =
      -_tmp167 * _tmp446 - _tmp167 * _tmp448 + _tmp205 * _tmp445 + _tmp224 * _tmp275;
  const Scalar _tmp450 = Scalar(1.0) / (_tmp449);
  const Scalar _tmp451 = _tmp173 * _tmp380;
  const Scalar _tmp452 = _tmp373 * _tmp395;
  const Scalar _tmp453 = _tmp173 * _tmp452;
  const Scalar _tmp454 = _tmp173 * _tmp224;
  const Scalar _tmp455 = _tmp173 * _tmp241;
  const Scalar _tmp456 = _tmp173 * _tmp375;
  const Scalar _tmp457 = _tmp173 * _tmp392;
  const Scalar _tmp458 = _tmp213 * _tmp457 + _tmp218 * _tmp456 - _tmp369 * _tmp453 +
                         _tmp381 * _tmp451 + _tmp385 * _tmp455 + _tmp389 * _tmp454;
  const Scalar _tmp459 = std::asinh(_tmp450 * _tmp458);
  const Scalar _tmp460 = Scalar(9.6622558468725703) * _tmp449;
  const Scalar _tmp461 = Scalar(4.7752063900000001) - _tmp152;
  const Scalar _tmp462 = Scalar(2.71799795) - _tmp155;
  const Scalar _tmp463 =
      std::sqrt(Scalar(std::pow(_tmp461, Scalar(2)) + std::pow(_tmp462, Scalar(2))));
  const Scalar _tmp464 = -_tmp459 * _tmp460 - _tmp463;
  const Scalar _tmp465 = Scalar(0.1034955) * _tmp450;
  const Scalar _tmp466 = _tmp464 * _tmp465;
  const Scalar _tmp467 = Scalar(1.0) * _tmp459;
  const Scalar _tmp468 = _tmp165 * _tmp299;
  const Scalar _tmp469 = _tmp365 * _tmp447;
  const Scalar _tmp470 = _tmp241 * _tmp351;
  const Scalar _tmp471 = _tmp301 * _tmp377;
  const Scalar _tmp472 =
      _tmp165 * _tmp224 * _tmp339 - _tmp167 * _tmp469 + _tmp167 * _tmp470 + _tmp167 * _tmp471 -
      _tmp205 * _tmp213 * _tmp283 - _tmp224 * _tmp233 * _tmp283 - _tmp259 * _tmp275 +
      _tmp260 * _tmp275 + _tmp298 * _tmp446 + _tmp298 * _tmp448 + _tmp344 * _tmp355 -
      _tmp354 * _tmp355 + _tmp364 * _tmp445 - _tmp446 * _tmp468 - _tmp448 * _tmp468;
  const Scalar _tmp473 = Scalar(9.6622558468725703) * _tmp472;
  const Scalar _tmp474 = _tmp173 * _tmp389;
  const Scalar _tmp475 = _tmp373 * _tmp406;
  const Scalar _tmp476 = std::pow(_tmp449, Scalar(-2));
  const Scalar _tmp477 = _tmp472 * _tmp476;
  const Scalar _tmp478 =
      (_tmp450 * (_tmp173 * _tmp213 * _tmp414 + _tmp173 * _tmp218 * _tmp434 -
                  _tmp173 * _tmp369 * _tmp475 + _tmp173 * _tmp381 * _tmp435 -
                  _tmp213 * _tmp320 * _tmp392 - _tmp218 * _tmp320 * _tmp375 -
                  _tmp224 * _tmp320 * _tmp389 - _tmp241 * _tmp425 - _tmp259 * _tmp474 +
                  _tmp260 * _tmp474 - _tmp319 * _tmp453 + _tmp320 * _tmp369 * _tmp452 -
                  _tmp320 * _tmp380 * _tmp381 + _tmp344 * _tmp457 - _tmp354 * _tmp457 +
                  _tmp366 * _tmp456 + _tmp395 * _tmp419 + _tmp418 * _tmp454 + _tmp422 * _tmp451 +
                  _tmp430 * _tmp455) -
       _tmp458 * _tmp477) /
      std::sqrt(Scalar(std::pow(_tmp458, Scalar(2)) * _tmp476 + 1));
  const Scalar _tmp479 = _tmp245 + _tmp86;
  const Scalar _tmp480 = _tmp203 * _tmp213 + _tmp224 * _tmp232 + _tmp446 + _tmp448;
  const Scalar _tmp481 = std::pow(_tmp480, Scalar(-2));
  const Scalar _tmp482 = _tmp180 * _tmp213 * _tmp363 + _tmp180 * _tmp224 * _tmp336 -
                         _tmp202 * _tmp213 * _tmp330 + _tmp203 * _tmp344 - _tmp203 * _tmp354 -
                         _tmp224 * _tmp231 * _tmp330 - _tmp232 * _tmp259 + _tmp232 * _tmp260 +
                         _tmp469 - _tmp470 - _tmp471;
  const Scalar _tmp483 = _tmp481 * _tmp482;
  const Scalar _tmp484 = -_tmp129 + Scalar(-8.3888750099999996);
  const Scalar _tmp485 = Scalar(2.5202214700000001) - _tmp124;
  const Scalar _tmp486 =
      std::sqrt(Scalar(std::pow(_tmp484, Scalar(2)) + std::pow(_tmp485, Scalar(2))));
  const Scalar _tmp487 = _tmp224 * _tmp372;
  const Scalar _tmp488 = _tmp213 * _tmp372;
  const Scalar _tmp489 = -_tmp201 * _tmp488 - _tmp218 * _tmp374 - _tmp230 * _tmp487 +
                         _tmp241 * _tmp384 - _tmp380 * _tmp382 + _tmp452;
  const Scalar _tmp490 = Scalar(1.0) / (_tmp480);
  const Scalar _tmp491 = std::asinh(_tmp489 * _tmp490);
  const Scalar _tmp492 = Scalar(9.6622558468725703) * _tmp480;
  const Scalar _tmp493 = -_tmp486 - _tmp491 * _tmp492;
  const Scalar _tmp494 = _tmp230 * _tmp372;
  const Scalar _tmp495 = _tmp201 * _tmp372;
  const Scalar _tmp496 =
      (-_tmp483 * _tmp489 +
       _tmp490 * (_tmp201 * _tmp213 * _tmp410 + _tmp218 * _tmp431 + _tmp218 * _tmp432 -
                  _tmp218 * _tmp433 + _tmp224 * _tmp230 * _tmp410 + _tmp241 * _tmp426 -
                  _tmp241 * _tmp427 + _tmp259 * _tmp494 - _tmp260 * _tmp494 + _tmp303 * _tmp487 -
                  _tmp324 * _tmp373 * _tmp451 - _tmp328 * _tmp487 - _tmp335 * _tmp487 -
                  _tmp344 * _tmp495 + _tmp354 * _tmp495 + _tmp356 * _tmp488 - _tmp357 * _tmp488 -
                  _tmp362 * _tmp488 - _tmp366 * _tmp374 + _tmp380 * _tmp421 + _tmp380 * _tmp423 -
                  _tmp382 * _tmp435 - _tmp395 * _tmp415 + _tmp475)) /
      std::sqrt(Scalar(_tmp481 * std::pow(_tmp489, Scalar(2)) + 1));
  const Scalar _tmp497 = Scalar(9.6622558468725703) * _tmp482;
  const Scalar _tmp498 = Scalar(0.1034955) * _tmp490;
  const Scalar _tmp499 = _tmp493 * _tmp498;
  const Scalar _tmp500 = Scalar(1.0) * _tmp491;

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) =
      _tmp121 -
      Scalar(0.5) * (2 * _tmp38 * (_tmp105 + _tmp99) + 2 * _tmp50 * (_tmp77 + _tmp93)) *
          std::sinh(Scalar(0.1034955) * _tmp52 *
                    (-_tmp51 - Scalar(9.6622558468725703) * fh1 * std::asinh(_tmp52 * fv1))) /
          _tmp51;
  _res(1, 0) =
      _tmp308 -
      _tmp437 *
          (-Scalar(0.87679799772039002) * _tmp368 + Scalar(1.0) * _tmp436 * std::sinh(_tmp401) -
           (-Scalar(0.1034955) * _tmp368 * _tmp441 +
            _tmp442 * (-_tmp400 * _tmp444 - _tmp436 * _tmp437 -
                       Scalar(1) / Scalar(2) *
                           (2 * _tmp429 * _tmp438 + 2 * _tmp439 * (_tmp247 + _tmp93)) / _tmp440)) *
               std::sinh(_tmp443)) -
      _tmp444 * (Scalar(0.87679799772039002) * _tmp399 + std::cosh(_tmp401) - std::cosh(_tmp443));
  _res(2, 0) =
      _tmp310 -
      _tmp460 *
          (-Scalar(0.86565325453551001) * _tmp477 + Scalar(1.0) * _tmp478 * std::sinh(_tmp467) -
           (-Scalar(0.1034955) * _tmp464 * _tmp477 +
            _tmp465 * (-_tmp459 * _tmp473 - _tmp460 * _tmp478 -
                       Scalar(1) / Scalar(2) *
                           (2 * _tmp461 * (_tmp105 + _tmp280) + 2 * _tmp462 * (_tmp479 + _tmp77)) /
                           _tmp463)) *
               std::sinh(_tmp466)) -
      _tmp473 * (Scalar(0.86565325453551001) * _tmp450 - std::cosh(_tmp466) + std::cosh(_tmp467));
  _res(3, 0) =
      _tmp307 -
      _tmp492 *
          (-Scalar(0.87653584775870996) * _tmp483 + Scalar(1.0) * _tmp496 * std::sinh(_tmp500) -
           (-Scalar(0.1034955) * _tmp483 * _tmp493 +
            _tmp498 * (-_tmp491 * _tmp497 - _tmp492 * _tmp496 -
                       Scalar(1) / Scalar(2) *
                           (2 * _tmp484 * (_tmp251 + _tmp428) + 2 * _tmp485 * (_tmp247 + _tmp479)) /
                           _tmp486)) *
               std::sinh(_tmp499)) -
      _tmp497 * (Scalar(0.87653584775870996) * _tmp490 - std::cosh(_tmp499) + std::cosh(_tmp500));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
