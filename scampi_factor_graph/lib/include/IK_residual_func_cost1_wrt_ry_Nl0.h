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
 * Symbolic function: IK_residual_func_cost1_wrt_ry_Nl0
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost1WrtRyNl0(
    const Scalar fh1, const Scalar fv1, const Scalar rx, const Scalar ry, const Scalar rz,
    const Scalar p_init0, const Scalar p_init1, const Scalar p_init2, const Scalar rot_init_x,
    const Scalar rot_init_y, const Scalar rot_init_z, const Scalar rot_init_w,
    const Scalar epsilon) {
  // Total ops: 1618

  // Unused inputs
  (void)p_init2;
  (void)epsilon;

  // Input arrays

  // Intermediate terms (498)
  const Scalar _tmp0 = std::pow(ry, Scalar(2));
  const Scalar _tmp1 = _tmp0 + std::pow(rx, Scalar(2)) + std::pow(rz, Scalar(2));
  const Scalar _tmp2 = std::sqrt(_tmp1);
  const Scalar _tmp3 = (Scalar(1) / Scalar(2)) * _tmp2;
  const Scalar _tmp4 = std::cos(_tmp3);
  const Scalar _tmp5 = _tmp4 * rot_init_z;
  const Scalar _tmp6 = std::sin(_tmp3);
  const Scalar _tmp7 = _tmp6 / _tmp2;
  const Scalar _tmp8 = _tmp7 * rot_init_x;
  const Scalar _tmp9 = _tmp8 * ry;
  const Scalar _tmp10 = _tmp7 * rot_init_y;
  const Scalar _tmp11 = _tmp7 * rot_init_w;
  const Scalar _tmp12 = -_tmp10 * rx + _tmp11 * rz + _tmp5 + _tmp9;
  const Scalar _tmp13 = _tmp4 * rot_init_x;
  const Scalar _tmp14 = (Scalar(1) / Scalar(2)) / _tmp1;
  const Scalar _tmp15 = _tmp0 * _tmp14;
  const Scalar _tmp16 = _tmp4 * rot_init_w;
  const Scalar _tmp17 = _tmp14 * ry;
  const Scalar _tmp18 = _tmp17 * rz;
  const Scalar _tmp19 = _tmp4 * rot_init_y;
  const Scalar _tmp20 = _tmp17 * rx;
  const Scalar _tmp21 = _tmp7 * rot_init_z;
  const Scalar _tmp22 = _tmp21 * ry;
  const Scalar _tmp23 = _tmp6 / (_tmp1 * std::sqrt(_tmp1));
  const Scalar _tmp24 = _tmp0 * _tmp23;
  const Scalar _tmp25 = _tmp23 * ry;
  const Scalar _tmp26 = _tmp25 * rx;
  const Scalar _tmp27 = _tmp25 * rz;
  const Scalar _tmp28 = _tmp13 * _tmp15 + _tmp16 * _tmp18 - _tmp19 * _tmp20 -
                        Scalar(1) / Scalar(2) * _tmp22 - _tmp24 * rot_init_x + _tmp26 * rot_init_y -
                        _tmp27 * rot_init_w + _tmp8;
  const Scalar _tmp29 = Scalar(0.83999999999999997) * _tmp28;
  const Scalar _tmp30 = _tmp12 * _tmp29;
  const Scalar _tmp31 = -_tmp30;
  const Scalar _tmp32 = -_tmp15 * _tmp5 + _tmp16 * _tmp20 + _tmp18 * _tmp19 - _tmp21 +
                        _tmp24 * rot_init_z - _tmp26 * rot_init_w - _tmp27 * rot_init_y -
                        Scalar(1) / Scalar(2) * _tmp9;
  const Scalar _tmp33 = _tmp10 * rz + _tmp11 * rx + _tmp13 - _tmp22;
  const Scalar _tmp34 = _tmp32 * _tmp33;
  const Scalar _tmp35 = Scalar(0.83999999999999997) * _tmp34;
  const Scalar _tmp36 = _tmp31 - _tmp35;
  const Scalar _tmp37 = _tmp10 * ry;
  const Scalar _tmp38 = _tmp11 - _tmp13 * _tmp18 + _tmp15 * _tmp16 + _tmp20 * _tmp5 -
                        _tmp24 * rot_init_w - _tmp26 * rot_init_z + _tmp27 * rot_init_x -
                        Scalar(1) / Scalar(2) * _tmp37;
  const Scalar _tmp39 = _tmp33 * _tmp38;
  const Scalar _tmp40 = Scalar(0.41999999999999998) * _tmp39;
  const Scalar _tmp41 = _tmp11 * ry;
  const Scalar _tmp42 = _tmp19 + _tmp21 * rx + _tmp41 - _tmp8 * rz;
  const Scalar _tmp43 = _tmp32 * _tmp42;
  const Scalar _tmp44 = Scalar(0.41999999999999998) * _tmp43;
  const Scalar _tmp45 = _tmp40 + _tmp44;
  const Scalar _tmp46 = _tmp16 - _tmp21 * rz - _tmp37 - _tmp8 * rx;
  const Scalar _tmp47 = Scalar(0.41999999999999998) * _tmp28;
  const Scalar _tmp48 = _tmp46 * _tmp47;
  const Scalar _tmp49 = -_tmp10 - _tmp13 * _tmp20 - _tmp15 * _tmp19 - _tmp18 * _tmp5 +
                        _tmp24 * rot_init_y + _tmp26 * rot_init_x + _tmp27 * rot_init_z -
                        Scalar(1) / Scalar(2) * _tmp41;
  const Scalar _tmp50 = _tmp12 * _tmp49;
  const Scalar _tmp51 = Scalar(0.41999999999999998) * _tmp50;
  const Scalar _tmp52 = _tmp48 + _tmp51;
  const Scalar _tmp53 = _tmp45 + _tmp52;
  const Scalar _tmp54 = Scalar(0.021999999999999999) * _tmp46;
  const Scalar _tmp55 = _tmp32 * _tmp54;
  const Scalar _tmp56 = _tmp33 * _tmp49;
  const Scalar _tmp57 = Scalar(0.021999999999999999) * _tmp56;
  const Scalar _tmp58 = Scalar(0.021999999999999999) * _tmp28;
  const Scalar _tmp59 = _tmp42 * _tmp58;
  const Scalar _tmp60 = Scalar(0.021999999999999999) * _tmp12;
  const Scalar _tmp61 = _tmp38 * _tmp60;
  const Scalar _tmp62 = -_tmp55 - _tmp57 + _tmp59 + _tmp61;
  const Scalar _tmp63 = _tmp53 + _tmp62;
  const Scalar _tmp64 = -2 * std::pow(_tmp33, Scalar(2));
  const Scalar _tmp65 = -2 * std::pow(_tmp12, Scalar(2));
  const Scalar _tmp66 = Scalar(0.20999999999999999) * _tmp64 +
                        Scalar(0.20999999999999999) * _tmp65 + Scalar(0.20999999999999999);
  const Scalar _tmp67 = -_tmp66;
  const Scalar _tmp68 = 2 * _tmp12 * _tmp42;
  const Scalar _tmp69 = 2 * _tmp46;
  const Scalar _tmp70 = _tmp33 * _tmp69;
  const Scalar _tmp71 = _tmp68 - _tmp70;
  const Scalar _tmp72 = -Scalar(0.010999999999999999) * _tmp71;
  const Scalar _tmp73 = 2 * _tmp33;
  const Scalar _tmp74 = _tmp42 * _tmp73;
  const Scalar _tmp75 = _tmp12 * _tmp69;
  const Scalar _tmp76 = Scalar(0.20999999999999999) * _tmp74 + Scalar(0.20999999999999999) * _tmp75;
  const Scalar _tmp77 = _tmp72 - _tmp76;
  const Scalar _tmp78 = _tmp67 + _tmp77;
  const Scalar _tmp79 = _tmp78 + p_init1;
  const Scalar _tmp80 = -_tmp79 + Scalar(-8.3196563700000006);
  const Scalar _tmp81 = _tmp38 * _tmp42;
  const Scalar _tmp82 = Scalar(0.83999999999999997) * _tmp81;
  const Scalar _tmp83 = _tmp31 - _tmp82;
  const Scalar _tmp84 = -_tmp48 - _tmp51;
  const Scalar _tmp85 = _tmp45 + _tmp84;
  const Scalar _tmp86 = _tmp83 + _tmp85;
  const Scalar _tmp87 = _tmp38 * _tmp54;
  const Scalar _tmp88 = _tmp42 * _tmp49;
  const Scalar _tmp89 = Scalar(0.021999999999999999) * _tmp88;
  const Scalar _tmp90 = _tmp33 * _tmp58;
  const Scalar _tmp91 = _tmp32 * _tmp60;
  const Scalar _tmp92 = _tmp87 + _tmp89 + _tmp90 + _tmp91;
  const Scalar _tmp93 = Scalar(0.20999999999999999) * _tmp74 - Scalar(0.20999999999999999) * _tmp75;
  const Scalar _tmp94 = -_tmp93;
  const Scalar _tmp95 = 1 - 2 * std::pow(_tmp42, Scalar(2));
  const Scalar _tmp96 = Scalar(0.20999999999999999) * _tmp65 + Scalar(0.20999999999999999) * _tmp95;
  const Scalar _tmp97 = _tmp12 * _tmp73;
  const Scalar _tmp98 = _tmp42 * _tmp69;
  const Scalar _tmp99 = _tmp97 + _tmp98;
  const Scalar _tmp100 = -Scalar(0.010999999999999999) * _tmp99;
  const Scalar _tmp101 = _tmp100 - _tmp96;
  const Scalar _tmp102 = _tmp101 + _tmp94;
  const Scalar _tmp103 = _tmp102 + p_init0;
  const Scalar _tmp104 = -_tmp103 + Scalar(-1.9874742000000001);
  const Scalar _tmp105 =
      std::sqrt(Scalar(std::pow(_tmp104, Scalar(2)) + std::pow(_tmp80, Scalar(2))));
  const Scalar _tmp106 = Scalar(1.0) / (fh1);
  const Scalar _tmp107 = Scalar(0.41999999999999998) * _tmp46;
  const Scalar _tmp108 = _tmp107 * _tmp32;
  const Scalar _tmp109 = Scalar(0.41999999999999998) * _tmp56;
  const Scalar _tmp110 = _tmp42 * _tmp47;
  const Scalar _tmp111 = Scalar(0.41999999999999998) * _tmp12;
  const Scalar _tmp112 = _tmp111 * _tmp38;
  const Scalar _tmp113 = Scalar(0.043999999999999997) * _tmp81;
  const Scalar _tmp114 = Scalar(0.043999999999999997) * _tmp34;
  const Scalar _tmp115 = _tmp113 + _tmp114;
  const Scalar _tmp116 = -_tmp108 - _tmp109 - _tmp110 - _tmp112 + _tmp115;
  const Scalar _tmp117 = _tmp107 * _tmp38;
  const Scalar _tmp118 = Scalar(0.41999999999999998) * _tmp88;
  const Scalar _tmp119 = _tmp33 * _tmp47;
  const Scalar _tmp120 = _tmp111 * _tmp32;
  const Scalar _tmp121 = _tmp117 + _tmp118 - _tmp119 - _tmp120;
  const Scalar _tmp122 = _tmp116 + _tmp121;
  const Scalar _tmp123 = _tmp100 + _tmp96;
  const Scalar _tmp124 = _tmp123 + _tmp94;
  const Scalar _tmp125 = _tmp124 + p_init0;
  const Scalar _tmp126 = _tmp125 + Scalar(-2.5202214700000001);
  const Scalar _tmp127 = Scalar(1.0) / (_tmp126);
  const Scalar _tmp128 = _tmp72 + _tmp76;
  const Scalar _tmp129 = _tmp128 + _tmp67;
  const Scalar _tmp130 = _tmp129 + p_init1;
  const Scalar _tmp131 = _tmp130 + Scalar(8.3888750099999996);
  const Scalar _tmp132 = _tmp127 * _tmp131;
  const Scalar _tmp133 = _tmp128 + _tmp66;
  const Scalar _tmp134 = _tmp133 + p_init1;
  const Scalar _tmp135 = _tmp134 + Scalar(-4.7752063900000001);
  const Scalar _tmp136 = _tmp123 + _tmp93;
  const Scalar _tmp137 = _tmp136 + p_init0;
  const Scalar _tmp138 = _tmp137 + Scalar(-2.71799795);
  const Scalar _tmp139 = std::pow(_tmp135, Scalar(2)) + std::pow(_tmp138, Scalar(2));
  const Scalar _tmp140 = std::pow(_tmp139, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp141 = _tmp138 * _tmp140;
  const Scalar _tmp142 = _tmp135 * _tmp140;
  const Scalar _tmp143 = _tmp132 * _tmp141 - _tmp142;
  const Scalar _tmp144 = Scalar(1.0) / (_tmp143);
  const Scalar _tmp145 = _tmp141 * _tmp144;
  const Scalar _tmp146 = Scalar(1.0) * _tmp124;
  const Scalar _tmp147 = Scalar(1.0) * _tmp129;
  const Scalar _tmp148 = -_tmp147;
  const Scalar _tmp149 = _tmp133 + _tmp148;
  const Scalar _tmp150 = Scalar(1.0) / (_tmp149);
  const Scalar _tmp151 = -_tmp136 + _tmp146;
  const Scalar _tmp152 = _tmp150 * _tmp151;
  const Scalar _tmp153 = _tmp146 + _tmp147 * _tmp152;
  const Scalar _tmp154 = 0;
  const Scalar _tmp155 =
      Scalar(0.20999999999999999) * _tmp68 + Scalar(0.20999999999999999) * _tmp70;
  const Scalar _tmp156 = -_tmp155;
  const Scalar _tmp157 =
      -Scalar(0.010999999999999999) * _tmp64 - Scalar(0.010999999999999999) * _tmp95;
  const Scalar _tmp158 =
      Scalar(0.20999999999999999) * _tmp97 - Scalar(0.20999999999999999) * _tmp98;
  const Scalar _tmp159 = _tmp157 + _tmp158;
  const Scalar _tmp160 = _tmp156 + _tmp159;
  const Scalar _tmp161 = _tmp141 * _tmp160;
  const Scalar _tmp162 = _tmp155 + _tmp159;
  const Scalar _tmp163 = -_tmp132 * _tmp161 + _tmp142 * _tmp162;
  const Scalar _tmp164 = _tmp66 + _tmp77;
  const Scalar _tmp165 = _tmp164 + p_init1;
  const Scalar _tmp166 = _tmp165 + Scalar(-4.8333311099999996);
  const Scalar _tmp167 = _tmp101 + _tmp93;
  const Scalar _tmp168 = _tmp167 + p_init0;
  const Scalar _tmp169 = _tmp168 + Scalar(1.79662371);
  const Scalar _tmp170 = std::pow(_tmp166, Scalar(2)) + std::pow(_tmp169, Scalar(2));
  const Scalar _tmp171 = std::pow(_tmp170, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp172 = _tmp169 * _tmp171;
  const Scalar _tmp173 = _tmp166 * _tmp171;
  const Scalar _tmp174 = _tmp132 * _tmp172 - _tmp173;
  const Scalar _tmp175 = _tmp144 * _tmp174;
  const Scalar _tmp176 = _tmp132 * _tmp160;
  const Scalar _tmp177 = _tmp157 - _tmp158;
  const Scalar _tmp178 = _tmp155 + _tmp177;
  const Scalar _tmp179 = -_tmp163 * _tmp175 - _tmp172 * _tmp176 + _tmp173 * _tmp178;
  const Scalar _tmp180 = -_tmp141 * _tmp162 + _tmp161;
  const Scalar _tmp181 = _tmp160 * _tmp172;
  const Scalar _tmp182 = -_tmp152 * _tmp179 - _tmp172 * _tmp178 - _tmp175 * _tmp180 + _tmp181;
  const Scalar _tmp183 = Scalar(1.0) / (_tmp182);
  const Scalar _tmp184 = _tmp174 * _tmp183;
  const Scalar _tmp185 = _tmp154 * _tmp184;
  const Scalar _tmp186 = _tmp154 * _tmp183;
  const Scalar _tmp187 = -_tmp145 * _tmp185 + _tmp172 * _tmp186;
  const Scalar _tmp188 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp189 = std::pow(_tmp126, Scalar(2));
  const Scalar _tmp190 = std::pow(_tmp131, Scalar(2)) + _tmp189;
  const Scalar _tmp191 = std::sqrt(_tmp190);
  const Scalar _tmp192 = _tmp127 * _tmp191;
  const Scalar _tmp193 = _tmp188 * _tmp192;
  const Scalar _tmp194 = _tmp133 * _tmp140;
  const Scalar _tmp195 = Scalar(1.0) / (_tmp191);
  const Scalar _tmp196 = _tmp126 * _tmp195;
  const Scalar _tmp197 = _tmp124 * _tmp195;
  const Scalar _tmp198 = -_tmp129 * _tmp196 + _tmp131 * _tmp197;
  const Scalar _tmp199 = _tmp192 * _tmp198;
  const Scalar _tmp200 = -_tmp136 * _tmp142 + _tmp138 * _tmp194 + _tmp141 * _tmp199;
  const Scalar _tmp201 =
      _tmp164 * _tmp172 - _tmp167 * _tmp173 + _tmp172 * _tmp199 - _tmp175 * _tmp200;
  const Scalar _tmp202 = Scalar(1.0) / (_tmp201);
  const Scalar _tmp203 = Scalar(1.0) * _tmp202;
  const Scalar _tmp204 = _tmp175 * _tmp203;
  const Scalar _tmp205 = -_tmp141 * _tmp204 + _tmp172 * _tmp203;
  const Scalar _tmp206 = _tmp103 + Scalar(1.9874742000000001);
  const Scalar _tmp207 = _tmp79 + Scalar(8.3196563700000006);
  const Scalar _tmp208 = std::pow(_tmp206, Scalar(2)) + std::pow(_tmp207, Scalar(2));
  const Scalar _tmp209 = std::pow(_tmp208, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp210 = _tmp209 * _tmp78;
  const Scalar _tmp211 = _tmp207 * _tmp209;
  const Scalar _tmp212 = fh1 * (_tmp102 * _tmp211 - _tmp206 * _tmp210);
  const Scalar _tmp213 = _tmp192 * _tmp212;
  const Scalar _tmp214 = _tmp211 * fh1;
  const Scalar _tmp215 = _tmp144 * _tmp200;
  const Scalar _tmp216 = _tmp144 * _tmp163;
  const Scalar _tmp217 = Scalar(1.0) * _tmp150;
  const Scalar _tmp218 = _tmp151 * _tmp217;
  const Scalar _tmp219 = Scalar(1.0) * _tmp144;
  const Scalar _tmp220 = -_tmp180 * _tmp219 + _tmp216 * _tmp218;
  const Scalar _tmp221 = _tmp183 * _tmp201;
  const Scalar _tmp222 = -Scalar(1.0) * _tmp215 - _tmp220 * _tmp221;
  const Scalar _tmp223 = _tmp182 * _tmp202;
  const Scalar _tmp224 = _tmp222 * _tmp223;
  const Scalar _tmp225 = _tmp220 + _tmp224;
  const Scalar _tmp226 = _tmp183 * _tmp225;
  const Scalar _tmp227 = -_tmp184 * _tmp225 + Scalar(1.0);
  const Scalar _tmp228 = _tmp145 * _tmp227 + _tmp172 * _tmp226;
  const Scalar _tmp229 = _tmp192 * _tmp228;
  const Scalar _tmp230 = _tmp144 * _tmp180;
  const Scalar _tmp231 = _tmp132 * _tmp216 + _tmp176;
  const Scalar _tmp232 = _tmp132 * _tmp230 - _tmp152 * _tmp231 - _tmp160;
  const Scalar _tmp233 = _tmp132 * _tmp215 - _tmp199 - _tmp221 * _tmp232;
  const Scalar _tmp234 = _tmp223 * _tmp233;
  const Scalar _tmp235 = _tmp232 + _tmp234;
  const Scalar _tmp236 = -_tmp132 - _tmp184 * _tmp235;
  const Scalar _tmp237 = _tmp183 * _tmp235;
  const Scalar _tmp238 = _tmp145 * _tmp236 + _tmp172 * _tmp237 + Scalar(1.0);
  const Scalar _tmp239 = _tmp206 * _tmp209;
  const Scalar _tmp240 = _tmp239 * fh1;
  const Scalar _tmp241 = _tmp192 * _tmp240;
  const Scalar _tmp242 =
      -_tmp187 * _tmp193 - _tmp205 * _tmp213 - _tmp214 * _tmp229 - _tmp238 * _tmp241;
  const Scalar _tmp243 = Scalar(1.0) / (_tmp242);
  const Scalar _tmp244 = _tmp148 + _tmp164;
  const Scalar _tmp245 = _tmp152 * _tmp244;
  const Scalar _tmp246 = _tmp146 - _tmp167 - _tmp245;
  const Scalar _tmp247 = Scalar(1.0) / (_tmp246);
  const Scalar _tmp248 = _tmp224 * _tmp247;
  const Scalar _tmp249 = Scalar(1.0) * _tmp216;
  const Scalar _tmp250 = _tmp179 * _tmp183;
  const Scalar _tmp251 = -_tmp225 * _tmp250 + _tmp244 * _tmp248 - _tmp249;
  const Scalar _tmp252 = Scalar(1.0) * _tmp247;
  const Scalar _tmp253 = -_tmp217 * _tmp251 + _tmp224 * _tmp252;
  const Scalar _tmp254 = Scalar(1.0) * _tmp214;
  const Scalar _tmp255 = _tmp244 * _tmp247;
  const Scalar _tmp256 = _tmp231 + _tmp234 * _tmp255 - _tmp235 * _tmp250;
  const Scalar _tmp257 = -_tmp217 * _tmp256 + _tmp234 * _tmp252;
  const Scalar _tmp258 = Scalar(1.0) * _tmp257;
  const Scalar _tmp259 = fh1 * (_tmp156 + _tmp177);
  const Scalar _tmp260 = _tmp102 * fv1 + _tmp239 * _tmp259 + Scalar(3.29616) * _tmp99;
  const Scalar _tmp261 = _tmp150 * _tmp252;
  const Scalar _tmp262 = Scalar(1.0) * _tmp244 * _tmp261 - Scalar(1.0) * _tmp252;
  const Scalar _tmp263 = _tmp153 * _tmp247;
  const Scalar _tmp264 = _tmp148 - _tmp154 * _tmp250 - _tmp244 * _tmp263;
  const Scalar _tmp265 = Scalar(1.0) * _tmp188;
  const Scalar _tmp266 = _tmp223 * _tmp252;
  const Scalar _tmp267 = -_tmp179 * _tmp203 + _tmp244 * _tmp266;
  const Scalar _tmp268 = -Scalar(1.0) * _tmp217 * _tmp267 + Scalar(1.0) * _tmp266;
  const Scalar _tmp269 = _tmp245 * _tmp252 + Scalar(1.0);
  const Scalar _tmp270 = _tmp152 * _tmp252;
  const Scalar _tmp271 = -_tmp217 * _tmp269 + _tmp270;
  const Scalar _tmp272 = -_tmp211 * _tmp259 - Scalar(3.29616) * _tmp71 - _tmp78 * fv1;
  const Scalar _tmp273 = Scalar(1.0) * _tmp272;
  const Scalar _tmp274 =
      _tmp212 * _tmp268 + _tmp240 * _tmp258 + _tmp253 * _tmp254 + _tmp260 * _tmp262 +
      _tmp265 * (-_tmp153 * _tmp252 - _tmp217 * _tmp264 + Scalar(1.0)) + _tmp271 * _tmp273;
  const Scalar _tmp275 = std::asinh(_tmp243 * _tmp274);
  const Scalar _tmp276 = Scalar(1.0) * _tmp275;
  const Scalar _tmp277 = Scalar(9.6622558468725703) * _tmp275;
  const Scalar _tmp278 = -_tmp130 + Scalar(-8.3888750099999996);
  const Scalar _tmp279 = Scalar(2.5202214700000001) - _tmp125;
  const Scalar _tmp280 =
      std::sqrt(Scalar(std::pow(_tmp278, Scalar(2)) + std::pow(_tmp279, Scalar(2))));
  const Scalar _tmp281 = -_tmp242 * _tmp277 - _tmp280;
  const Scalar _tmp282 = Scalar(0.1034955) * _tmp243;
  const Scalar _tmp283 = _tmp281 * _tmp282;
  const Scalar _tmp284 = _tmp55 + _tmp57 - _tmp59 - _tmp61;
  const Scalar _tmp285 = _tmp284 + _tmp36;
  const Scalar _tmp286 = _tmp285 + _tmp53;
  const Scalar _tmp287 = -_tmp87 - _tmp89 - _tmp90 - _tmp91;
  const Scalar _tmp288 = _tmp287 + _tmp86;
  const Scalar _tmp289 =
      (2 * _tmp135 * _tmp286 + 2 * _tmp138 * _tmp288) / (_tmp139 * std::sqrt(_tmp139));
  const Scalar _tmp290 = (Scalar(1) / Scalar(2)) * _tmp289;
  const Scalar _tmp291 = _tmp138 * _tmp290;
  const Scalar _tmp292 = _tmp30 + _tmp35;
  const Scalar _tmp293 = _tmp284 + _tmp292;
  const Scalar _tmp294 = _tmp293 + _tmp53;
  const Scalar _tmp295 = _tmp127 * _tmp294;
  const Scalar _tmp296 = _tmp140 * _tmp286;
  const Scalar _tmp297 = -_tmp40 - _tmp44;
  const Scalar _tmp298 = _tmp297 + _tmp52;
  const Scalar _tmp299 = _tmp298 + _tmp83;
  const Scalar _tmp300 = _tmp287 + _tmp299;
  const Scalar _tmp301 = _tmp300 / _tmp189;
  const Scalar _tmp302 = _tmp131 * _tmp301;
  const Scalar _tmp303 = _tmp135 * _tmp290;
  const Scalar _tmp304 = _tmp140 * _tmp288;
  const Scalar _tmp305 = (-_tmp132 * _tmp291 + _tmp132 * _tmp304 + _tmp141 * _tmp295 -
                          _tmp141 * _tmp302 - _tmp296 + _tmp303) /
                         std::pow(_tmp143, Scalar(2));
  const Scalar _tmp306 = _tmp174 * _tmp305;
  const Scalar _tmp307 = -_tmp117 - _tmp118 + _tmp119 + _tmp120;
  const Scalar _tmp308 = _tmp116 + _tmp307;
  const Scalar _tmp309 = _tmp108 + _tmp109 + _tmp110 + _tmp112;
  const Scalar _tmp310 = _tmp115 + _tmp309;
  const Scalar _tmp311 = _tmp307 + _tmp310;
  const Scalar _tmp312 = _tmp141 * _tmp308 - _tmp141 * _tmp311 - _tmp160 * _tmp291 +
                         _tmp160 * _tmp304 + _tmp162 * _tmp291 - _tmp162 * _tmp304;
  const Scalar _tmp313 = _tmp121 + _tmp310;
  const Scalar _tmp314 = _tmp132 * _tmp308;
  const Scalar _tmp315 = _tmp160 * _tmp295;
  const Scalar _tmp316 = -_tmp141 * _tmp314 - _tmp141 * _tmp315 + _tmp142 * _tmp311 +
                         _tmp161 * _tmp302 + _tmp162 * _tmp296 - _tmp162 * _tmp303 +
                         _tmp176 * _tmp291 - _tmp176 * _tmp304;
  const Scalar _tmp317 = _tmp30 + _tmp82;
  const Scalar _tmp318 = _tmp287 + _tmp317;
  const Scalar _tmp319 = _tmp318 + _tmp85;
  const Scalar _tmp320 = _tmp297 + _tmp84;
  const Scalar _tmp321 = _tmp285 + _tmp320;
  const Scalar _tmp322 =
      (2 * _tmp166 * _tmp321 + 2 * _tmp169 * _tmp319) / (_tmp170 * std::sqrt(_tmp170));
  const Scalar _tmp323 = (Scalar(1) / Scalar(2)) * _tmp322;
  const Scalar _tmp324 = _tmp166 * _tmp323;
  const Scalar _tmp325 = _tmp171 * _tmp319;
  const Scalar _tmp326 = _tmp160 * _tmp325;
  const Scalar _tmp327 = _tmp169 * _tmp323;
  const Scalar _tmp328 = _tmp171 * _tmp321;
  const Scalar _tmp329 = _tmp132 * _tmp325 - _tmp132 * _tmp327 + _tmp172 * _tmp295 -
                         _tmp172 * _tmp302 + _tmp324 - _tmp328;
  const Scalar _tmp330 = -_tmp132 * _tmp326 + _tmp163 * _tmp306 - _tmp172 * _tmp314 -
                         _tmp172 * _tmp315 + _tmp173 * _tmp313 - _tmp175 * _tmp316 +
                         _tmp176 * _tmp327 - _tmp178 * _tmp324 + _tmp178 * _tmp328 +
                         _tmp181 * _tmp302 - _tmp216 * _tmp329;
  const Scalar _tmp331 = _tmp29 * _tmp46;
  const Scalar _tmp332 = Scalar(0.83999999999999997) * _tmp50;
  const Scalar _tmp333 =
      -Scalar(0.83999999999999997) * _tmp39 - Scalar(0.83999999999999997) * _tmp43;
  const Scalar _tmp334 = _tmp331 + _tmp332 + _tmp333;
  const Scalar _tmp335 = _tmp150 * _tmp334;
  const Scalar _tmp336 = -Scalar(1.6799999999999999) * _tmp12 * _tmp28;
  const Scalar _tmp337 = _tmp336 - Scalar(1.6799999999999999) * _tmp34;
  const Scalar _tmp338 = _tmp337 / std::pow(_tmp149, Scalar(2));
  const Scalar _tmp339 = _tmp151 * _tmp338;
  const Scalar _tmp340 = -_tmp152 * _tmp330 - _tmp160 * _tmp327 + _tmp172 * _tmp308 -
                         _tmp172 * _tmp313 - _tmp175 * _tmp312 - _tmp178 * _tmp325 +
                         _tmp178 * _tmp327 - _tmp179 * _tmp335 + _tmp179 * _tmp339 +
                         _tmp180 * _tmp306 - _tmp230 * _tmp329 + _tmp326;
  const Scalar _tmp341 = _tmp340 / std::pow(_tmp182, Scalar(2));
  const Scalar _tmp342 = _tmp154 * _tmp341;
  const Scalar _tmp343 = _tmp141 * _tmp305;
  const Scalar _tmp344 = _tmp141 * _tmp175;
  const Scalar _tmp345 = _tmp145 * _tmp329;
  const Scalar _tmp346 = _tmp144 * _tmp304;
  const Scalar _tmp347 = _tmp235 * _tmp341;
  const Scalar _tmp348 = _tmp132 * _tmp305;
  const Scalar _tmp349 = _tmp126 * _tmp300 + _tmp131 * _tmp294;
  const Scalar _tmp350 = _tmp127 * _tmp195 * _tmp349;
  const Scalar _tmp351 = _tmp198 * _tmp350;
  const Scalar _tmp352 = _tmp191 * _tmp301;
  const Scalar _tmp353 = _tmp198 * _tmp352;
  const Scalar _tmp354 = _tmp195 * _tmp300;
  const Scalar _tmp355 = _tmp349 / (_tmp190 * std::sqrt(_tmp190));
  const Scalar _tmp356 =
      _tmp192 * (-_tmp124 * _tmp131 * _tmp355 + _tmp126 * _tmp129 * _tmp355 - _tmp129 * _tmp354 +
                 _tmp131 * _tmp354 - _tmp196 * _tmp294 + _tmp197 * _tmp294);
  const Scalar _tmp357 = -_tmp133 * _tmp291 - _tmp136 * _tmp296 + _tmp136 * _tmp303 +
                         _tmp138 * _tmp296 + _tmp141 * _tmp351 - _tmp141 * _tmp353 +
                         _tmp141 * _tmp356 - _tmp142 * _tmp288 + _tmp194 * _tmp288 -
                         _tmp199 * _tmp291 + _tmp199 * _tmp304;
  const Scalar _tmp358 = _tmp164 * _tmp325 - _tmp164 * _tmp327 + _tmp167 * _tmp324 -
                         _tmp167 * _tmp328 + _tmp169 * _tmp328 + _tmp172 * _tmp351 -
                         _tmp172 * _tmp353 + _tmp172 * _tmp356 - _tmp173 * _tmp319 -
                         _tmp175 * _tmp357 + _tmp199 * _tmp325 - _tmp199 * _tmp327 +
                         _tmp200 * _tmp306 - _tmp215 * _tmp329;
  const Scalar _tmp359 = _tmp183 * _tmp358;
  const Scalar _tmp360 = _tmp132 * _tmp144;
  const Scalar _tmp361 = -_tmp160 * _tmp302 - _tmp163 * _tmp348 + _tmp216 * _tmp295 -
                         _tmp216 * _tmp302 + _tmp314 + _tmp315 + _tmp316 * _tmp360;
  const Scalar _tmp362 = -_tmp113 - _tmp114 + _tmp121 - _tmp152 * _tmp361 - _tmp180 * _tmp348 +
                         _tmp230 * _tmp295 - _tmp230 * _tmp302 - _tmp231 * _tmp335 +
                         _tmp231 * _tmp339 + _tmp309 + _tmp312 * _tmp360;
  const Scalar _tmp363 = _tmp201 * _tmp341;
  const Scalar _tmp364 = _tmp223 * (-_tmp200 * _tmp348 + _tmp215 * _tmp295 - _tmp215 * _tmp302 -
                                    _tmp221 * _tmp362 - _tmp232 * _tmp359 + _tmp232 * _tmp363 -
                                    _tmp351 + _tmp353 - _tmp356 + _tmp357 * _tmp360);
  const Scalar _tmp365 = _tmp358 / std::pow(_tmp201, Scalar(2));
  const Scalar _tmp366 = _tmp182 * _tmp365;
  const Scalar _tmp367 = _tmp233 * _tmp366;
  const Scalar _tmp368 = _tmp202 * _tmp340;
  const Scalar _tmp369 = _tmp233 * _tmp368;
  const Scalar _tmp370 = _tmp362 + _tmp364 - _tmp367 + _tmp369;
  const Scalar _tmp371 =
      _tmp174 * _tmp347 - _tmp184 * _tmp370 - _tmp237 * _tmp329 - _tmp295 + _tmp302;
  const Scalar _tmp372 = _tmp172 * _tmp183;
  const Scalar _tmp373 = _tmp144 * _tmp236;
  const Scalar _tmp374 = _tmp205 * _tmp212;
  const Scalar _tmp375 = _tmp238 * _tmp240;
  const Scalar _tmp376 = Scalar(1.0) * _tmp305;
  const Scalar _tmp377 = _tmp144 * _tmp218 * _tmp316 - _tmp163 * _tmp218 * _tmp305 +
                         _tmp180 * _tmp376 + _tmp216 * _tmp217 * _tmp334 - _tmp219 * _tmp312 -
                         _tmp249 * _tmp339;
  const Scalar _tmp378 = _tmp223 * (_tmp200 * _tmp376 - _tmp219 * _tmp357 - _tmp220 * _tmp359 +
                                    _tmp220 * _tmp363 - _tmp221 * _tmp377);
  const Scalar _tmp379 = _tmp222 * _tmp368;
  const Scalar _tmp380 = _tmp222 * _tmp366;
  const Scalar _tmp381 = _tmp377 + _tmp378 + _tmp379 - _tmp380;
  const Scalar _tmp382 = _tmp144 * _tmp227;
  const Scalar _tmp383 = _tmp225 * _tmp341;
  const Scalar _tmp384 = _tmp174 * _tmp383 - _tmp184 * _tmp381 - _tmp226 * _tmp329;
  const Scalar _tmp385 = _tmp298 + _tmp318;
  const Scalar _tmp386 = _tmp293 + _tmp320;
  const Scalar _tmp387 =
      (2 * _tmp206 * _tmp385 + 2 * _tmp207 * _tmp386) / (_tmp208 * std::sqrt(_tmp208));
  const Scalar _tmp388 = (Scalar(1) / Scalar(2)) * _tmp387;
  const Scalar _tmp389 = _tmp207 * _tmp388;
  const Scalar _tmp390 = _tmp389 * fh1;
  const Scalar _tmp391 = Scalar(0.5) * _tmp202;
  const Scalar _tmp392 = Scalar(1.0) * _tmp365;
  const Scalar _tmp393 = _tmp209 * _tmp385;
  const Scalar _tmp394 = _tmp393 * fh1;
  const Scalar _tmp395 = _tmp192 * _tmp238;
  const Scalar _tmp396 = _tmp206 * _tmp388;
  const Scalar _tmp397 = _tmp396 * fh1;
  const Scalar _tmp398 = _tmp214 * _tmp228;
  const Scalar _tmp399 = _tmp209 * _tmp386;
  const Scalar _tmp400 = fh1 * (-_tmp102 * _tmp389 + _tmp102 * _tmp399 - _tmp206 * _tmp399 -
                                _tmp210 * _tmp385 + _tmp211 * _tmp385 + _tmp396 * _tmp78);
  const Scalar _tmp401 = _tmp399 * fh1;
  const Scalar _tmp402 = _tmp187 * _tmp188;
  const Scalar _tmp403 =
      -_tmp192 * _tmp205 * _tmp400 -
      _tmp192 * _tmp214 *
          (_tmp145 * _tmp384 - _tmp172 * _tmp383 + _tmp226 * _tmp325 - _tmp226 * _tmp327 -
           _tmp227 * _tmp343 + _tmp227 * _tmp346 - _tmp291 * _tmp382 + _tmp372 * _tmp381) -
      _tmp193 *
          (_tmp144 * _tmp185 * _tmp291 - _tmp172 * _tmp342 + _tmp185 * _tmp343 - _tmp185 * _tmp346 +
           _tmp186 * _tmp325 - _tmp186 * _tmp327 - _tmp186 * _tmp345 + _tmp342 * _tmp344) -
      _tmp213 * (_tmp138 * _tmp175 * _tmp289 * _tmp391 + _tmp141 * _tmp203 * _tmp306 -
                 _tmp169 * _tmp322 * _tmp391 - _tmp172 * _tmp392 + _tmp203 * _tmp325 -
                 _tmp203 * _tmp345 - _tmp204 * _tmp304 + _tmp344 * _tmp392) +
      _tmp229 * _tmp390 - _tmp229 * _tmp401 -
      _tmp241 * (_tmp145 * _tmp371 - _tmp172 * _tmp347 - _tmp236 * _tmp343 + _tmp236 * _tmp346 +
                 _tmp237 * _tmp325 - _tmp237 * _tmp327 - _tmp291 * _tmp373 + _tmp370 * _tmp372) -
      _tmp350 * _tmp374 - _tmp350 * _tmp375 - _tmp350 * _tmp398 - _tmp350 * _tmp402 +
      _tmp352 * _tmp374 + _tmp352 * _tmp375 + _tmp352 * _tmp398 + _tmp352 * _tmp402 -
      _tmp394 * _tmp395 + _tmp395 * _tmp397;
  const Scalar _tmp404 = std::pow(_tmp242, Scalar(-2));
  const Scalar _tmp405 = _tmp403 * _tmp404;
  const Scalar _tmp406 = _tmp320 + _tmp62;
  const Scalar _tmp407 = _tmp36 + _tmp406;
  const Scalar _tmp408 = _tmp317 + _tmp92;
  const Scalar _tmp409 = Scalar(9.6622558468725703) * _tmp242;
  const Scalar _tmp410 = -_tmp331 - _tmp332 + _tmp333 + _tmp337;
  const Scalar _tmp411 = _tmp152 * _tmp410;
  const Scalar _tmp412 = _tmp244 * _tmp339;
  const Scalar _tmp413 = _tmp244 * _tmp335;
  const Scalar _tmp414 =
      (_tmp334 + _tmp336 - _tmp411 + _tmp412 - _tmp413 - Scalar(1.6799999999999999) * _tmp81) /
      std::pow(_tmp246, Scalar(2));
  const Scalar _tmp415 = _tmp244 * _tmp414;
  const Scalar _tmp416 = _tmp217 * _tmp415;
  const Scalar _tmp417 = Scalar(1.0) * _tmp414;
  const Scalar _tmp418 = _tmp244 * _tmp338;
  const Scalar _tmp419 = _tmp252 * _tmp339;
  const Scalar _tmp420 = _tmp218 * _tmp414;
  const Scalar _tmp421 =
      -_tmp244 * _tmp420 + _tmp252 * _tmp411 - _tmp252 * _tmp412 + _tmp252 * _tmp413;
  const Scalar _tmp422 = Scalar(1.0) * _tmp338;
  const Scalar _tmp423 = _tmp153 * _tmp414;
  const Scalar _tmp424 = _tmp147 * _tmp335 - _tmp147 * _tmp339 + _tmp218 * _tmp294 + _tmp300;
  const Scalar _tmp425 = _tmp247 * _tmp424;
  const Scalar _tmp426 = _tmp179 * _tmp342 - _tmp186 * _tmp330 + _tmp244 * _tmp423 -
                         _tmp244 * _tmp425 - _tmp263 * _tmp410 + _tmp407;
  const Scalar _tmp427 = _tmp264 * _tmp338;
  const Scalar _tmp428 = Scalar(6.59232) * _tmp46;
  const Scalar _tmp429 = Scalar(6.59232) * _tmp28;
  const Scalar _tmp430 = Scalar(6.59232) * _tmp12;
  const Scalar _tmp431 = _tmp122 * fh1;
  const Scalar _tmp432 = -_tmp211 * _tmp431 + _tmp259 * _tmp389 - _tmp259 * _tmp399 +
                         _tmp32 * _tmp428 - _tmp38 * _tmp430 - _tmp386 * fv1 - _tmp42 * _tmp429 +
                         Scalar(6.59232) * _tmp56;
  const Scalar _tmp433 = _tmp239 * _tmp431 + _tmp259 * _tmp393 - _tmp259 * _tmp396 +
                         _tmp32 * _tmp430 + _tmp33 * _tmp429 + _tmp38 * _tmp428 + _tmp385 * fv1 +
                         Scalar(6.59232) * _tmp88;
  const Scalar _tmp434 = Scalar(0.5) * _tmp387 * fh1;
  const Scalar _tmp435 = _tmp163 * _tmp376 + _tmp179 * _tmp383 - _tmp219 * _tmp316 -
                         _tmp224 * _tmp415 - _tmp226 * _tmp330 + _tmp248 * _tmp410 -
                         _tmp250 * _tmp381 + _tmp255 * _tmp378 + _tmp255 * _tmp379 -
                         _tmp255 * _tmp380;
  const Scalar _tmp436 = _tmp234 * _tmp247;
  const Scalar _tmp437 = _tmp179 * _tmp347 - _tmp234 * _tmp415 - _tmp237 * _tmp330 -
                         _tmp250 * _tmp370 + _tmp255 * _tmp364 - _tmp255 * _tmp367 +
                         _tmp255 * _tmp369 + _tmp361 + _tmp410 * _tmp436;
  const Scalar _tmp438 = _tmp252 * _tmp368;
  const Scalar _tmp439 = _tmp252 * _tmp366;
  const Scalar _tmp440 = _tmp223 * _tmp417;
  const Scalar _tmp441 = _tmp179 * _tmp392 - _tmp203 * _tmp330 + _tmp244 * _tmp438 -
                         _tmp244 * _tmp439 - _tmp244 * _tmp440 + _tmp266 * _tmp410;
  const Scalar _tmp442 =
      (_tmp243 *
           (-_tmp206 * _tmp257 * _tmp434 - _tmp207 * _tmp253 * _tmp434 +
            Scalar(1.0) * _tmp212 *
                (-_tmp217 * _tmp441 + _tmp267 * _tmp422 + _tmp438 - _tmp439 - _tmp440) +
            Scalar(1.0) * _tmp240 *
                (-_tmp217 * _tmp437 - _tmp234 * _tmp417 + _tmp252 * _tmp364 - _tmp252 * _tmp367 +
                 _tmp252 * _tmp369 + _tmp256 * _tmp422) +
            Scalar(1.0) * _tmp253 * _tmp401 +
            _tmp254 * (-_tmp217 * _tmp435 - _tmp224 * _tmp417 + _tmp251 * _tmp422 +
                       _tmp252 * _tmp378 + _tmp252 * _tmp379 - _tmp252 * _tmp380) +
            _tmp258 * _tmp394 +
            Scalar(1.0) * _tmp260 * (-_tmp252 * _tmp418 + _tmp261 * _tmp410 - _tmp416 + _tmp417) +
            _tmp262 * _tmp433 +
            _tmp265 * (_tmp153 * _tmp417 - _tmp217 * _tmp426 - _tmp252 * _tmp424 +
                       Scalar(1.0) * _tmp427) +
            _tmp268 * _tmp400 + Scalar(1.0) * _tmp271 * _tmp432 +
            _tmp273 *
                (-_tmp217 * _tmp421 + _tmp252 * _tmp335 + _tmp269 * _tmp422 - _tmp419 - _tmp420)) -
       _tmp274 * _tmp405) /
      std::sqrt(Scalar(std::pow(_tmp274, Scalar(2)) * _tmp404 + 1));
  const Scalar _tmp443 = _tmp144 * _tmp240;
  const Scalar _tmp444 = _tmp144 * _tmp214;
  const Scalar _tmp445 = _tmp185 * _tmp188;
  const Scalar _tmp446 = _tmp203 * _tmp212;
  const Scalar _tmp447 =
      -_tmp144 * _tmp445 - _tmp175 * _tmp446 + _tmp227 * _tmp444 + _tmp236 * _tmp443;
  const Scalar _tmp448 = Scalar(1.0) / (_tmp447);
  const Scalar _tmp449 = _tmp150 * _tmp212;
  const Scalar _tmp450 = _tmp252 * _tmp260;
  const Scalar _tmp451 = _tmp150 * _tmp244;
  const Scalar _tmp452 = _tmp150 * _tmp214;
  const Scalar _tmp453 = _tmp150 * _tmp272;
  const Scalar _tmp454 = _tmp150 * _tmp188;
  const Scalar _tmp455 = _tmp150 * _tmp256;
  const Scalar _tmp456 = _tmp240 * _tmp455 + _tmp251 * _tmp452 + _tmp264 * _tmp454 +
                         _tmp267 * _tmp449 + _tmp269 * _tmp453 - _tmp450 * _tmp451;
  const Scalar _tmp457 = std::asinh(_tmp448 * _tmp456);
  const Scalar _tmp458 = Scalar(9.6622558468725703) * _tmp447;
  const Scalar _tmp459 = Scalar(4.7752063900000001) - _tmp134;
  const Scalar _tmp460 = Scalar(2.71799795) - _tmp137;
  const Scalar _tmp461 =
      std::sqrt(Scalar(std::pow(_tmp459, Scalar(2)) + std::pow(_tmp460, Scalar(2))));
  const Scalar _tmp462 = -_tmp457 * _tmp458 - _tmp461;
  const Scalar _tmp463 = Scalar(0.1034955) * _tmp448;
  const Scalar _tmp464 = _tmp462 * _tmp463;
  const Scalar _tmp465 = Scalar(1.0) * _tmp457;
  const Scalar _tmp466 = _tmp186 * _tmp188;
  const Scalar _tmp467 = _tmp144 * _tmp329;
  const Scalar _tmp468 = _tmp188 * _tmp342;
  const Scalar _tmp469 = _tmp203 * _tmp400;
  const Scalar _tmp470 = _tmp212 * _tmp392;
  const Scalar _tmp471 =
      _tmp175 * _tmp468 - _tmp175 * _tmp469 + _tmp175 * _tmp470 - _tmp214 * _tmp227 * _tmp305 -
      _tmp236 * _tmp240 * _tmp305 + _tmp305 * _tmp445 + _tmp306 * _tmp446 + _tmp371 * _tmp443 +
      _tmp373 * _tmp394 - _tmp373 * _tmp397 - _tmp382 * _tmp390 + _tmp382 * _tmp401 +
      _tmp384 * _tmp444 - _tmp446 * _tmp467 - _tmp466 * _tmp467;
  const Scalar _tmp472 = Scalar(9.6622558468725703) * _tmp471;
  const Scalar _tmp473 = std::pow(_tmp447, Scalar(-2));
  const Scalar _tmp474 = _tmp471 * _tmp473;
  const Scalar _tmp475 = _tmp150 * _tmp251;
  const Scalar _tmp476 = _tmp252 * _tmp433;
  const Scalar _tmp477 =
      (_tmp448 * (_tmp150 * _tmp240 * _tmp437 + _tmp150 * _tmp267 * _tmp400 +
                  _tmp150 * _tmp269 * _tmp432 - _tmp150 * _tmp410 * _tmp450 - _tmp188 * _tmp427 -
                  _tmp212 * _tmp267 * _tmp338 - _tmp214 * _tmp251 * _tmp338 -
                  _tmp240 * _tmp256 * _tmp338 + _tmp260 * _tmp416 - _tmp269 * _tmp272 * _tmp338 -
                  _tmp390 * _tmp475 + _tmp394 * _tmp455 - _tmp397 * _tmp455 + _tmp401 * _tmp475 +
                  _tmp418 * _tmp450 + _tmp421 * _tmp453 + _tmp426 * _tmp454 + _tmp435 * _tmp452 +
                  _tmp441 * _tmp449 - _tmp451 * _tmp476) -
       _tmp456 * _tmp474) /
      std::sqrt(Scalar(std::pow(_tmp456, Scalar(2)) * _tmp473 + 1));
  const Scalar _tmp478 = _tmp252 * _tmp453;
  const Scalar _tmp479 = -_tmp151 * _tmp478 + _tmp188 * _tmp263 - _tmp212 * _tmp266 -
                         _tmp214 * _tmp248 - _tmp240 * _tmp436 + _tmp450;
  const Scalar _tmp480 = _tmp214 * _tmp226 + _tmp237 * _tmp240 + _tmp446 + _tmp466;
  const Scalar _tmp481 = Scalar(1.0) / (_tmp480);
  const Scalar _tmp482 = std::asinh(_tmp479 * _tmp481);
  const Scalar _tmp483 = Scalar(1.0) * _tmp482;
  const Scalar _tmp484 = Scalar(9.6622558468725703) * _tmp480;
  const Scalar _tmp485 = Scalar(4.8333311099999996) - _tmp165;
  const Scalar _tmp486 = -_tmp168 + Scalar(-1.79662371);
  const Scalar _tmp487 =
      std::sqrt(Scalar(std::pow(_tmp485, Scalar(2)) + std::pow(_tmp486, Scalar(2))));
  const Scalar _tmp488 = -_tmp482 * _tmp484 - _tmp487;
  const Scalar _tmp489 = Scalar(0.1034955) * _tmp481;
  const Scalar _tmp490 = _tmp488 * _tmp489;
  const Scalar _tmp491 = _tmp183 * _tmp214 * _tmp381 + _tmp183 * _tmp240 * _tmp370 -
                         _tmp214 * _tmp383 - _tmp226 * _tmp390 + _tmp226 * _tmp401 +
                         _tmp237 * _tmp394 - _tmp237 * _tmp397 - _tmp240 * _tmp347 - _tmp468 +
                         _tmp469 - _tmp470;
  const Scalar _tmp492 = Scalar(9.6622558468725703) * _tmp491;
  const Scalar _tmp493 = std::pow(_tmp480, Scalar(-2));
  const Scalar _tmp494 = _tmp214 * _tmp247;
  const Scalar _tmp495 = _tmp240 * _tmp247;
  const Scalar _tmp496 = _tmp491 * _tmp493;
  const Scalar _tmp497 =
      (-_tmp479 * _tmp496 +
       _tmp481 * (_tmp152 * _tmp273 * _tmp414 - _tmp188 * _tmp423 + _tmp188 * _tmp425 -
                  _tmp212 * _tmp438 + _tmp212 * _tmp439 + _tmp212 * _tmp440 +
                  _tmp214 * _tmp224 * _tmp414 + _tmp234 * _tmp240 * _tmp414 + _tmp248 * _tmp390 -
                  _tmp248 * _tmp401 - _tmp260 * _tmp417 - _tmp266 * _tmp400 - _tmp270 * _tmp432 +
                  _tmp272 * _tmp419 - _tmp334 * _tmp478 - _tmp364 * _tmp495 + _tmp367 * _tmp495 -
                  _tmp369 * _tmp495 - _tmp378 * _tmp494 - _tmp379 * _tmp494 + _tmp380 * _tmp494 -
                  _tmp394 * _tmp436 + _tmp397 * _tmp436 + _tmp476)) /
      std::sqrt(Scalar(std::pow(_tmp479, Scalar(2)) * _tmp493 + 1));

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) =
      _tmp122 -
      Scalar(0.5) * (2 * _tmp104 * (_tmp86 + _tmp92) + 2 * _tmp80 * (_tmp36 + _tmp63)) *
          std::sinh(Scalar(0.1034955) * _tmp106 *
                    (-_tmp105 - Scalar(9.6622558468725703) * fh1 * std::asinh(_tmp106 * fv1))) /
          _tmp105;
  _res(1, 0) =
      _tmp308 -
      Scalar(9.6622558468725703) * _tmp403 *
          (Scalar(0.87653584775870996) * _tmp243 + std::cosh(_tmp276) - std::cosh(_tmp283)) -
      _tmp409 *
          (-Scalar(0.87653584775870996) * _tmp405 + Scalar(1.0) * _tmp442 * std::sinh(_tmp276) -
           (-Scalar(0.1034955) * _tmp281 * _tmp405 +
            _tmp282 * (-_tmp277 * _tmp403 - _tmp409 * _tmp442 -
                       Scalar(1) / Scalar(2) *
                           (2 * _tmp278 * _tmp407 + 2 * _tmp279 * (_tmp408 + _tmp85)) / _tmp280)) *
               std::sinh(_tmp283));
  _res(2, 0) =
      _tmp311 -
      _tmp458 *
          (-Scalar(0.86565325453551001) * _tmp474 + Scalar(1.0) * _tmp477 * std::sinh(_tmp465) -
           (-Scalar(0.1034955) * _tmp462 * _tmp474 +
            _tmp463 * (-_tmp457 * _tmp472 - _tmp458 * _tmp477 -
                       Scalar(1) / Scalar(2) *
                           (2 * _tmp459 * (_tmp292 + _tmp406) + 2 * _tmp460 * (_tmp298 + _tmp408)) /
                           _tmp461)) *
               std::sinh(_tmp464)) -
      _tmp472 * (Scalar(0.86565325453551001) * _tmp448 - std::cosh(_tmp464) + std::cosh(_tmp465));
  _res(3, 0) =
      _tmp313 -
      _tmp484 *
          (-Scalar(0.86625939559540499) * _tmp496 + Scalar(1.0) * _tmp497 * std::sinh(_tmp483) -
           (-Scalar(0.1034955) * _tmp488 * _tmp496 +
            _tmp489 * (-_tmp482 * _tmp492 - _tmp484 * _tmp497 -
                       Scalar(1) / Scalar(2) *
                           (2 * _tmp485 * (_tmp292 + _tmp63) + 2 * _tmp486 * (_tmp299 + _tmp92)) /
                           _tmp487)) *
               std::sinh(_tmp490)) -
      _tmp492 * (Scalar(0.86625939559540499) * _tmp481 + std::cosh(_tmp483) - std::cosh(_tmp490));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
