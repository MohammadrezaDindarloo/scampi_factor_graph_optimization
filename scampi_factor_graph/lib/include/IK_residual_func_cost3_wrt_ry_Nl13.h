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
 * Symbolic function: IK_residual_func_cost3_wrt_ry_Nl13
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost3WrtRyNl13(
    const Scalar fh1, const Scalar fv1, const Scalar rx, const Scalar ry, const Scalar rz,
    const Scalar p_init0, const Scalar p_init1, const Scalar p_init2, const Scalar rot_init_x,
    const Scalar rot_init_y, const Scalar rot_init_z, const Scalar rot_init_w,
    const Scalar epsilon) {
  // Total ops: 988

  // Unused inputs
  (void)p_init2;
  (void)epsilon;

  // Input arrays

  // Intermediate terms (319)
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
  const Scalar _tmp13 = -2 * std::pow(_tmp12, Scalar(2));
  const Scalar _tmp14 = _tmp4 * rot_init_z;
  const Scalar _tmp15 = _tmp7 * rot_init_x;
  const Scalar _tmp16 = _tmp15 * ry;
  const Scalar _tmp17 = _tmp10 * rz - _tmp11 * rx + _tmp14 + _tmp16;
  const Scalar _tmp18 = -2 * std::pow(_tmp17, Scalar(2));
  const Scalar _tmp19 = Scalar(0.20999999999999999) * _tmp13 +
                        Scalar(0.20999999999999999) * _tmp18 + Scalar(0.20999999999999999);
  const Scalar _tmp20 = -_tmp19;
  const Scalar _tmp21 = _tmp4 * rot_init_y;
  const Scalar _tmp22 = _tmp10 * ry;
  const Scalar _tmp23 = -_tmp15 * rz + _tmp21 + _tmp22 + _tmp8 * rx;
  const Scalar _tmp24 = 2 * _tmp17;
  const Scalar _tmp25 = _tmp23 * _tmp24;
  const Scalar _tmp26 = _tmp4 * rot_init_w;
  const Scalar _tmp27 = _tmp11 * ry;
  const Scalar _tmp28 = -_tmp15 * rx + _tmp26 - _tmp27 - _tmp8 * rz;
  const Scalar _tmp29 = 2 * _tmp28;
  const Scalar _tmp30 = _tmp12 * _tmp29;
  const Scalar _tmp31 =
      -Scalar(0.010999999999999999) * _tmp25 + Scalar(0.010999999999999999) * _tmp30;
  const Scalar _tmp32 = 2 * _tmp12 * _tmp23;
  const Scalar _tmp33 = _tmp17 * _tmp29;
  const Scalar _tmp34 = Scalar(0.20999999999999999) * _tmp32 + Scalar(0.20999999999999999) * _tmp33;
  const Scalar _tmp35 = _tmp31 - _tmp34;
  const Scalar _tmp36 = _tmp20 + _tmp35;
  const Scalar _tmp37 = Scalar(1.0) * _tmp36;
  const Scalar _tmp38 = _tmp19 + _tmp35;
  const Scalar _tmp39 = -_tmp37 + _tmp38;
  const Scalar _tmp40 = Scalar(1.0) / (_tmp39);
  const Scalar _tmp41 = 1 - 2 * std::pow(_tmp23, Scalar(2));
  const Scalar _tmp42 = Scalar(0.20999999999999999) * _tmp18 + Scalar(0.20999999999999999) * _tmp41;
  const Scalar _tmp43 = -_tmp42;
  const Scalar _tmp44 = Scalar(0.20999999999999999) * _tmp32 - Scalar(0.20999999999999999) * _tmp33;
  const Scalar _tmp45 = _tmp12 * _tmp24;
  const Scalar _tmp46 = _tmp23 * _tmp29;
  const Scalar _tmp47 =
      -Scalar(0.010999999999999999) * _tmp45 - Scalar(0.010999999999999999) * _tmp46;
  const Scalar _tmp48 = _tmp44 + _tmp47;
  const Scalar _tmp49 = _tmp43 + _tmp48;
  const Scalar _tmp50 = -_tmp44 + _tmp47;
  const Scalar _tmp51 = _tmp43 + _tmp50;
  const Scalar _tmp52 = Scalar(1.0) * _tmp51;
  const Scalar _tmp53 = -_tmp49 + _tmp52;
  const Scalar _tmp54 = _tmp40 * _tmp53;
  const Scalar _tmp55 = _tmp37 * _tmp54 + _tmp52;
  const Scalar _tmp56 = 0;
  const Scalar _tmp57 = Scalar(0.20999999999999999) * _tmp45 - Scalar(0.20999999999999999) * _tmp46;
  const Scalar _tmp58 =
      -Scalar(0.010999999999999999) * _tmp13 - Scalar(0.010999999999999999) * _tmp41;
  const Scalar _tmp59 = Scalar(0.20999999999999999) * _tmp25 + Scalar(0.20999999999999999) * _tmp30;
  const Scalar _tmp60 = _tmp58 - _tmp59;
  const Scalar _tmp61 = _tmp57 + _tmp60;
  const Scalar _tmp62 = _tmp42 + _tmp50;
  const Scalar _tmp63 = _tmp62 + p_init0 + Scalar(-2.5202214700000001);
  const Scalar _tmp64 = _tmp31 + _tmp34;
  const Scalar _tmp65 = _tmp20 + _tmp64;
  const Scalar _tmp66 = _tmp65 + p_init1 + Scalar(8.3888750099999996);
  const Scalar _tmp67 = std::pow(_tmp63, Scalar(2)) + std::pow(_tmp66, Scalar(2));
  const Scalar _tmp68 = std::pow(_tmp67, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp69 = _tmp63 * _tmp68;
  const Scalar _tmp70 = _tmp66 * _tmp68;
  const Scalar _tmp71 = _tmp36 + p_init1 + Scalar(8.3196563700000006);
  const Scalar _tmp72 = _tmp51 + p_init0 + Scalar(1.9874742000000001);
  const Scalar _tmp73 = Scalar(1.0) / (_tmp72);
  const Scalar _tmp74 = _tmp71 * _tmp73;
  const Scalar _tmp75 = -_tmp57;
  const Scalar _tmp76 = _tmp60 + _tmp75;
  const Scalar _tmp77 = _tmp69 * _tmp76;
  const Scalar _tmp78 = _tmp38 + p_init1 + Scalar(-4.8333311099999996);
  const Scalar _tmp79 = _tmp49 + p_init0 + Scalar(1.79662371);
  const Scalar _tmp80 = std::pow(_tmp78, Scalar(2)) + std::pow(_tmp79, Scalar(2));
  const Scalar _tmp81 = std::pow(_tmp80, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp82 = _tmp79 * _tmp81;
  const Scalar _tmp83 = _tmp76 * _tmp82;
  const Scalar _tmp84 = _tmp58 + _tmp59 + _tmp75;
  const Scalar _tmp85 = _tmp78 * _tmp81;
  const Scalar _tmp86 = -_tmp74 * _tmp83 + _tmp84 * _tmp85;
  const Scalar _tmp87 = _tmp69 * _tmp74 - _tmp70;
  const Scalar _tmp88 = _tmp74 * _tmp82 - _tmp85;
  const Scalar _tmp89 = Scalar(1.0) / (_tmp88);
  const Scalar _tmp90 = _tmp87 * _tmp89;
  const Scalar _tmp91 = _tmp61 * _tmp70 - _tmp74 * _tmp77 - _tmp86 * _tmp90;
  const Scalar _tmp92 = -_tmp82 * _tmp84 + _tmp83;
  const Scalar _tmp93 = -_tmp54 * _tmp91 - _tmp61 * _tmp69 + _tmp77 - _tmp90 * _tmp92;
  const Scalar _tmp94 = Scalar(1.0) / (_tmp93);
  const Scalar _tmp95 = _tmp69 * _tmp94;
  const Scalar _tmp96 = _tmp82 * _tmp89;
  const Scalar _tmp97 = _tmp87 * _tmp94;
  const Scalar _tmp98 = _tmp56 * _tmp97;
  const Scalar _tmp99 = _tmp56 * _tmp95 - _tmp96 * _tmp98;
  const Scalar _tmp100 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp101 = std::pow(_tmp72, Scalar(2));
  const Scalar _tmp102 = _tmp101 + std::pow(_tmp71, Scalar(2));
  const Scalar _tmp103 = std::sqrt(_tmp102);
  const Scalar _tmp104 = _tmp103 * _tmp73;
  const Scalar _tmp105 = _tmp100 * _tmp104;
  const Scalar _tmp106 = _tmp19 + _tmp64;
  const Scalar _tmp107 = _tmp106 + p_init1 + Scalar(-4.7752063900000001);
  const Scalar _tmp108 = _tmp42 + _tmp48;
  const Scalar _tmp109 = _tmp108 + p_init0 + Scalar(-2.71799795);
  const Scalar _tmp110 = std::pow(_tmp107, Scalar(2)) + std::pow(_tmp109, Scalar(2));
  const Scalar _tmp111 = std::pow(_tmp110, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp112 = _tmp109 * _tmp111;
  const Scalar _tmp113 = Scalar(1.0) / (_tmp103);
  const Scalar _tmp114 = _tmp113 * _tmp36;
  const Scalar _tmp115 = _tmp51 * _tmp71;
  const Scalar _tmp116 = _tmp113 * _tmp115 - _tmp114 * _tmp72;
  const Scalar _tmp117 = _tmp104 * _tmp116;
  const Scalar _tmp118 = _tmp117 * _tmp82 + _tmp38 * _tmp82 - _tmp49 * _tmp85;
  const Scalar _tmp119 = _tmp65 * _tmp68;
  const Scalar _tmp120 = _tmp117 * _tmp69 - _tmp118 * _tmp90 + _tmp119 * _tmp63 - _tmp62 * _tmp70;
  const Scalar _tmp121 = _tmp74 * _tmp89;
  const Scalar _tmp122 = _tmp74 * _tmp76;
  const Scalar _tmp123 = _tmp121 * _tmp86 + _tmp122;
  const Scalar _tmp124 = _tmp121 * _tmp92 - _tmp123 * _tmp54 - _tmp76;
  const Scalar _tmp125 = _tmp124 * _tmp94;
  const Scalar _tmp126 = -_tmp117 + _tmp118 * _tmp121 - _tmp120 * _tmp125;
  const Scalar _tmp127 = Scalar(1.0) / (_tmp120);
  const Scalar _tmp128 = _tmp127 * _tmp93;
  const Scalar _tmp129 = _tmp124 + _tmp126 * _tmp128;
  const Scalar _tmp130 = -_tmp129 * _tmp97 - _tmp74;
  const Scalar _tmp131 = _tmp129 * _tmp95 + _tmp130 * _tmp96 + Scalar(1.0);
  const Scalar _tmp132 = _tmp104 * fh1;
  const Scalar _tmp133 = _tmp131 * _tmp132;
  const Scalar _tmp134 = Scalar(1.0) * _tmp89;
  const Scalar _tmp135 = _tmp134 * _tmp54;
  const Scalar _tmp136 = -_tmp134 * _tmp92 + _tmp135 * _tmp86;
  const Scalar _tmp137 = _tmp136 * _tmp94;
  const Scalar _tmp138 = -_tmp118 * _tmp134 - _tmp120 * _tmp137;
  const Scalar _tmp139 = _tmp128 * _tmp138 + _tmp136;
  const Scalar _tmp140 = -_tmp139 * _tmp97 + Scalar(1.0);
  const Scalar _tmp141 = _tmp139 * _tmp95 + _tmp140 * _tmp96;
  const Scalar _tmp142 = _tmp107 * _tmp111;
  const Scalar _tmp143 = _tmp132 * _tmp142;
  const Scalar _tmp144 = fh1 * (-_tmp106 * _tmp112 + _tmp108 * _tmp142);
  const Scalar _tmp145 = Scalar(1.0) * _tmp127;
  const Scalar _tmp146 = _tmp145 * _tmp90;
  const Scalar _tmp147 = _tmp145 * _tmp69 - _tmp146 * _tmp82;
  const Scalar _tmp148 = _tmp104 * _tmp147;
  const Scalar _tmp149 = (Scalar(1) / Scalar(2)) / _tmp1;
  const Scalar _tmp150 = _tmp0 * _tmp149;
  const Scalar _tmp151 = _tmp149 * ry;
  const Scalar _tmp152 = _tmp151 * rz;
  const Scalar _tmp153 = _tmp151 * rx;
  const Scalar _tmp154 = _tmp6 / (_tmp1 * std::sqrt(_tmp1));
  const Scalar _tmp155 = _tmp0 * _tmp154;
  const Scalar _tmp156 = _tmp154 * ry;
  const Scalar _tmp157 = _tmp156 * rx;
  const Scalar _tmp158 = _tmp156 * rz;
  const Scalar _tmp159 = _tmp10 + _tmp14 * _tmp153 + _tmp150 * _tmp26 - _tmp152 * _tmp5 -
                         _tmp155 * rot_init_w - _tmp157 * rot_init_z + _tmp158 * rot_init_x -
                         Scalar(1) / Scalar(2) * _tmp27;
  const Scalar _tmp160 = Scalar(0.83999999999999997) * _tmp159;
  const Scalar _tmp161 = _tmp160 * _tmp23;
  const Scalar _tmp162 = _tmp15 + _tmp150 * _tmp5 + _tmp152 * _tmp26 - _tmp153 * _tmp21 -
                         _tmp155 * rot_init_x + _tmp157 * rot_init_y - _tmp158 * rot_init_w -
                         Scalar(1) / Scalar(2) * _tmp9;
  const Scalar _tmp163 = Scalar(0.83999999999999997) * _tmp17;
  const Scalar _tmp164 = _tmp162 * _tmp163;
  const Scalar _tmp165 = _tmp162 * _tmp28;
  const Scalar _tmp166 = Scalar(0.41999999999999998) * _tmp165;
  const Scalar _tmp167 = -_tmp11 - _tmp14 * _tmp152 - _tmp150 * _tmp21 - _tmp153 * _tmp5 +
                         _tmp155 * rot_init_y + _tmp157 * rot_init_x + _tmp158 * rot_init_z -
                         Scalar(1) / Scalar(2) * _tmp22;
  const Scalar _tmp168 = Scalar(0.41999999999999998) * _tmp17;
  const Scalar _tmp169 = _tmp167 * _tmp168;
  const Scalar _tmp170 = _tmp166 + _tmp169;
  const Scalar _tmp171 = _tmp164 + _tmp170;
  const Scalar _tmp172 = Scalar(0.41999999999999998) * _tmp159;
  const Scalar _tmp173 = _tmp12 * _tmp172;
  const Scalar _tmp174 = -_tmp14 * _tmp150 + _tmp152 * _tmp21 + _tmp153 * _tmp26 +
                         _tmp155 * rot_init_z - _tmp157 * rot_init_w - _tmp158 * rot_init_y -
                         Scalar(1) / Scalar(2) * _tmp16 - _tmp8;
  const Scalar _tmp175 = _tmp174 * _tmp23;
  const Scalar _tmp176 = Scalar(0.41999999999999998) * _tmp175;
  const Scalar _tmp177 = -_tmp173 - _tmp176;
  const Scalar _tmp178 = Scalar(0.021999999999999999) * _tmp28;
  const Scalar _tmp179 = Scalar(0.021999999999999999) * _tmp167;
  const Scalar _tmp180 = Scalar(0.021999999999999999) * _tmp162;
  const Scalar _tmp181 = Scalar(0.021999999999999999) * _tmp17;
  const Scalar _tmp182 =
      -_tmp12 * _tmp180 - _tmp159 * _tmp178 - _tmp174 * _tmp181 - _tmp179 * _tmp23;
  const Scalar _tmp183 = _tmp177 + _tmp182;
  const Scalar _tmp184 = _tmp161 + _tmp171 + _tmp183;
  const Scalar _tmp185 = Scalar(1.0) / (_tmp101);
  const Scalar _tmp186 = _tmp103 * _tmp184 * _tmp185;
  const Scalar _tmp187 = _tmp112 * fh1;
  const Scalar _tmp188 = _tmp131 * _tmp187;
  const Scalar _tmp189 = -_tmp161;
  const Scalar _tmp190 = -_tmp164;
  const Scalar _tmp191 = _tmp170 + _tmp190;
  const Scalar _tmp192 = _tmp183 + _tmp189 + _tmp191;
  const Scalar _tmp193 = _tmp192 * _tmp68;
  const Scalar _tmp194 = _tmp193 * _tmp94;
  const Scalar _tmp195 = _tmp173 + _tmp176;
  const Scalar _tmp196 = _tmp12 * _tmp174;
  const Scalar _tmp197 = Scalar(0.83999999999999997) * _tmp196;
  const Scalar _tmp198 =
      _tmp12 * _tmp179 - _tmp159 * _tmp181 + _tmp174 * _tmp178 - _tmp180 * _tmp23;
  const Scalar _tmp199 = _tmp197 + _tmp198;
  const Scalar _tmp200 = _tmp171 + _tmp195 + _tmp199;
  const Scalar _tmp201 =
      (2 * _tmp192 * _tmp63 + 2 * _tmp200 * _tmp66) / (_tmp67 * std::sqrt(_tmp67));
  const Scalar _tmp202 = (Scalar(1) / Scalar(2)) * _tmp201;
  const Scalar _tmp203 = _tmp202 * _tmp63;
  const Scalar _tmp204 =
      _tmp53 *
      (-Scalar(1.6799999999999999) * _tmp162 * _tmp17 - Scalar(1.6799999999999999) * _tmp196) /
      std::pow(_tmp39, Scalar(2));
  const Scalar _tmp205 = -_tmp166 - _tmp169;
  const Scalar _tmp206 = _tmp164 + _tmp205;
  const Scalar _tmp207 = _tmp182 + _tmp195;
  const Scalar _tmp208 = _tmp161 + _tmp206 + _tmp207;
  const Scalar _tmp209 = _tmp208 * _tmp81;
  const Scalar _tmp210 = _tmp209 * _tmp76;
  const Scalar _tmp211 = -_tmp197 + _tmp198;
  const Scalar _tmp212 = _tmp190 + _tmp205;
  const Scalar _tmp213 = _tmp177 + _tmp211 + _tmp212;
  const Scalar _tmp214 =
      (2 * _tmp208 * _tmp79 + 2 * _tmp213 * _tmp78) / (_tmp80 * std::sqrt(_tmp80));
  const Scalar _tmp215 = (Scalar(1) / Scalar(2)) * _tmp214;
  const Scalar _tmp216 = _tmp215 * _tmp79;
  const Scalar _tmp217 = Scalar(0.41999999999999998) * _tmp174 * _tmp28;
  const Scalar _tmp218 = Scalar(0.41999999999999998) * _tmp167;
  const Scalar _tmp219 = _tmp12 * _tmp218;
  const Scalar _tmp220 = Scalar(0.043999999999999997) * _tmp159 * _tmp23;
  const Scalar _tmp221 = Scalar(0.41999999999999998) * _tmp162;
  const Scalar _tmp222 = _tmp221 * _tmp23;
  const Scalar _tmp223 = _tmp17 * _tmp172;
  const Scalar _tmp224 = Scalar(0.043999999999999997) * _tmp196;
  const Scalar _tmp225 = -_tmp217 - _tmp219 + _tmp220 - _tmp222 - _tmp223 + _tmp224;
  const Scalar _tmp226 = _tmp172 * _tmp28;
  const Scalar _tmp227 = _tmp218 * _tmp23;
  const Scalar _tmp228 = _tmp12 * _tmp221;
  const Scalar _tmp229 = _tmp168 * _tmp174;
  const Scalar _tmp230 = _tmp226 + _tmp227 - _tmp228 - _tmp229;
  const Scalar _tmp231 = _tmp225 + _tmp230;
  const Scalar _tmp232 = _tmp231 * _tmp82;
  const Scalar _tmp233 = _tmp217 + _tmp219 + _tmp222 + _tmp223;
  const Scalar _tmp234 = _tmp220 + _tmp224 + _tmp230 + _tmp233;
  const Scalar _tmp235 = -_tmp209 * _tmp84 + _tmp210 - _tmp216 * _tmp76 + _tmp216 * _tmp84 +
                         _tmp232 - _tmp234 * _tmp82;
  const Scalar _tmp236 = -_tmp226 - _tmp227 + _tmp228 + _tmp229;
  const Scalar _tmp237 = _tmp225 + _tmp236;
  const Scalar _tmp238 = _tmp231 * _tmp69;
  const Scalar _tmp239 =
      _tmp40 * (-_tmp12 * _tmp160 + _tmp163 * _tmp167 + Scalar(0.83999999999999997) * _tmp165 -
                Scalar(0.83999999999999997) * _tmp175);
  const Scalar _tmp240 = _tmp177 + _tmp199 + _tmp206;
  const Scalar _tmp241 = _tmp240 * _tmp73;
  const Scalar _tmp242 = _tmp202 * _tmp66;
  const Scalar _tmp243 = _tmp184 * _tmp71;
  const Scalar _tmp244 = _tmp185 * _tmp243;
  const Scalar _tmp245 = _tmp200 * _tmp68;
  const Scalar _tmp246 =
      _tmp193 * _tmp74 - _tmp203 * _tmp74 + _tmp241 * _tmp69 + _tmp242 - _tmp244 * _tmp69 - _tmp245;
  const Scalar _tmp247 = _tmp246 * _tmp89;
  const Scalar _tmp248 = _tmp215 * _tmp78;
  const Scalar _tmp249 = _tmp213 * _tmp81;
  const Scalar _tmp250 = (_tmp209 * _tmp74 - _tmp216 * _tmp74 + _tmp241 * _tmp82 -
                          _tmp244 * _tmp82 + _tmp248 - _tmp249) /
                         std::pow(_tmp88, Scalar(2));
  const Scalar _tmp251 = _tmp250 * _tmp87;
  const Scalar _tmp252 = _tmp122 * _tmp216 - _tmp210 * _tmp74 - _tmp232 * _tmp74 +
                         _tmp234 * _tmp85 - _tmp241 * _tmp83 + _tmp244 * _tmp83 - _tmp248 * _tmp84 +
                         _tmp249 * _tmp84;
  const Scalar _tmp253 =
      -_tmp193 * _tmp61 + _tmp193 * _tmp76 + _tmp203 * _tmp61 - _tmp203 * _tmp76 +
      _tmp204 * _tmp91 - _tmp235 * _tmp90 - _tmp237 * _tmp69 + _tmp238 - _tmp239 * _tmp91 -
      _tmp247 * _tmp92 + _tmp251 * _tmp92 -
      _tmp54 * (-_tmp122 * _tmp193 + _tmp122 * _tmp203 + _tmp237 * _tmp70 - _tmp238 * _tmp74 -
                _tmp241 * _tmp77 - _tmp242 * _tmp61 + _tmp244 * _tmp77 + _tmp245 * _tmp61 -
                _tmp247 * _tmp86 + _tmp251 * _tmp86 - _tmp252 * _tmp90);
  const Scalar _tmp254 = _tmp253 / std::pow(_tmp93, Scalar(2));
  const Scalar _tmp255 = _tmp82 * _tmp90;
  const Scalar _tmp256 = _tmp254 * _tmp69;
  const Scalar _tmp257 = _tmp247 * _tmp82;
  const Scalar _tmp258 = _tmp209 * _tmp89;
  const Scalar _tmp259 = _tmp216 * _tmp89;
  const Scalar _tmp260 = _tmp250 * _tmp82;
  const Scalar _tmp261 = _tmp203 * _tmp94;
  const Scalar _tmp262 = _tmp184 * _tmp72 + _tmp240 * _tmp71;
  const Scalar _tmp263 = _tmp113 * _tmp262 * _tmp73;
  const Scalar _tmp264 = _tmp100 * _tmp99;
  const Scalar _tmp265 = _tmp191 + _tmp195 + _tmp211;
  const Scalar _tmp266 = _tmp189 + _tmp207 + _tmp212;
  const Scalar _tmp267 = (Scalar(1) / Scalar(2)) * (2 * _tmp107 * _tmp265 + 2 * _tmp109 * _tmp266) /
                         (_tmp110 * std::sqrt(_tmp110));
  const Scalar _tmp268 = _tmp107 * _tmp267;
  const Scalar _tmp269 = _tmp132 * _tmp141;
  const Scalar _tmp270 = _tmp142 * fh1;
  const Scalar _tmp271 = _tmp141 * _tmp270;
  const Scalar _tmp272 = _tmp111 * _tmp266;
  const Scalar _tmp273 = _tmp144 * _tmp147;
  const Scalar _tmp274 = _tmp116 * _tmp263;
  const Scalar _tmp275 = _tmp116 * _tmp186;
  const Scalar _tmp276 = _tmp262 / (_tmp102 * std::sqrt(_tmp102));
  const Scalar _tmp277 = _tmp113 * _tmp240;
  const Scalar _tmp278 =
      _tmp104 * (_tmp113 * _tmp243 - _tmp114 * _tmp184 - _tmp115 * _tmp276 +
                 _tmp276 * _tmp36 * _tmp72 + _tmp277 * _tmp51 - _tmp277 * _tmp72);
  const Scalar _tmp279 =
      _tmp89 * (_tmp117 * _tmp209 - _tmp117 * _tmp216 - _tmp208 * _tmp85 + _tmp209 * _tmp38 -
                _tmp216 * _tmp38 + _tmp248 * _tmp49 - _tmp249 * _tmp49 + _tmp249 * _tmp79 +
                _tmp274 * _tmp82 - _tmp275 * _tmp82 + _tmp278 * _tmp82);
  const Scalar _tmp280 = _tmp241 * _tmp89;
  const Scalar _tmp281 = _tmp250 * _tmp74;
  const Scalar _tmp282 = _tmp244 * _tmp89;
  const Scalar _tmp283 =
      _tmp121 * _tmp235 + _tmp123 * _tmp204 - _tmp123 * _tmp239 - _tmp220 - _tmp224 + _tmp233 +
      _tmp236 + _tmp280 * _tmp92 - _tmp281 * _tmp92 - _tmp282 * _tmp92 -
      _tmp54 * (_tmp121 * _tmp252 + _tmp231 * _tmp74 + _tmp241 * _tmp76 - _tmp244 * _tmp76 +
                _tmp280 * _tmp86 - _tmp281 * _tmp86 - _tmp282 * _tmp86);
  const Scalar _tmp284 = _tmp120 * _tmp94;
  const Scalar _tmp285 = _tmp117 * _tmp193 - _tmp117 * _tmp203 - _tmp118 * _tmp247 +
                         _tmp118 * _tmp251 + _tmp119 * _tmp192 - _tmp192 * _tmp70 -
                         _tmp203 * _tmp65 + _tmp242 * _tmp62 - _tmp245 * _tmp62 + _tmp245 * _tmp63 +
                         _tmp274 * _tmp69 - _tmp275 * _tmp69 + _tmp278 * _tmp69 - _tmp279 * _tmp87;
  const Scalar _tmp286 = _tmp120 * _tmp254;
  const Scalar _tmp287 = _tmp285 / std::pow(_tmp120, Scalar(2));
  const Scalar _tmp288 = _tmp287 * _tmp93;
  const Scalar _tmp289 = _tmp127 * _tmp253;
  const Scalar _tmp290 = -_tmp126 * _tmp288 + _tmp126 * _tmp289 +
                         _tmp128 * (_tmp118 * _tmp280 - _tmp118 * _tmp281 - _tmp118 * _tmp282 +
                                    _tmp124 * _tmp286 - _tmp125 * _tmp285 - _tmp274 + _tmp275 -
                                    _tmp278 + _tmp279 * _tmp74 - _tmp283 * _tmp284) +
                         _tmp283;
  const Scalar _tmp291 = _tmp254 * _tmp87;
  const Scalar _tmp292 = _tmp246 * _tmp94;
  const Scalar _tmp293 =
      _tmp129 * _tmp291 - _tmp129 * _tmp292 - _tmp241 + _tmp244 - _tmp290 * _tmp97;
  const Scalar _tmp294 = Scalar(0.5) * _tmp127;
  const Scalar _tmp295 = Scalar(1.0) * _tmp287;
  const Scalar _tmp296 = Scalar(1.0) * _tmp250;
  const Scalar _tmp297 = _tmp134 * _tmp86;
  const Scalar _tmp298 = -_tmp134 * _tmp235 + _tmp135 * _tmp252 - _tmp204 * _tmp297 +
                         _tmp239 * _tmp297 - _tmp296 * _tmp54 * _tmp86 + _tmp296 * _tmp92;
  const Scalar _tmp299 = _tmp128 * (_tmp118 * _tmp296 + _tmp136 * _tmp286 - _tmp137 * _tmp285 -
                                    Scalar(1.0) * _tmp279 - _tmp284 * _tmp298) -
                         _tmp138 * _tmp288 + _tmp138 * _tmp289 + _tmp298;
  const Scalar _tmp300 = _tmp139 * _tmp291 - _tmp139 * _tmp292 - _tmp299 * _tmp97;
  const Scalar _tmp301 = _tmp111 * _tmp265;
  const Scalar _tmp302 = _tmp109 * _tmp267;
  const Scalar _tmp303 = fh1 * (-_tmp106 * _tmp272 + _tmp106 * _tmp302 - _tmp108 * _tmp268 +
                                _tmp108 * _tmp301 - _tmp112 * _tmp265 + _tmp142 * _tmp266);
  const Scalar _tmp304 = _tmp144 * _tmp145;
  const Scalar _tmp305 = _tmp100 * _tmp56;
  const Scalar _tmp306 = _tmp305 * _tmp97;
  const Scalar _tmp307 = _tmp89 * fh1;
  const Scalar _tmp308 = _tmp130 * _tmp307;
  const Scalar _tmp309 = _tmp140 * _tmp307;
  const Scalar _tmp310 = _tmp144 * _tmp295;
  const Scalar _tmp311 = _tmp254 * _tmp305;
  const Scalar _tmp312 = _tmp145 * _tmp303;
  const Scalar _tmp313 = _tmp305 * _tmp94;
  const Scalar _tmp314 = _tmp94 * fh1;
  const Scalar _tmp315 = _tmp112 * _tmp314;
  const Scalar _tmp316 = _tmp142 * _tmp314;
  const Scalar _tmp317 = _tmp129 * _tmp314;
  const Scalar _tmp318 = _tmp139 * _tmp314;

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) = 0;
  _res(1, 0) =
      -(-_tmp104 * _tmp144 *
            (_tmp145 * _tmp193 + _tmp145 * _tmp251 * _tmp82 - _tmp145 * _tmp257 -
             _tmp146 * _tmp209 - _tmp201 * _tmp294 * _tmp63 + _tmp214 * _tmp294 * _tmp79 * _tmp90 +
             _tmp255 * _tmp295 - _tmp295 * _tmp69) -
        _tmp105 * (_tmp194 * _tmp56 + _tmp254 * _tmp255 * _tmp56 - _tmp256 * _tmp56 -
                   _tmp257 * _tmp56 * _tmp94 - _tmp258 * _tmp98 + _tmp259 * _tmp98 +
                   _tmp260 * _tmp98 - _tmp261 * _tmp56) -
        _tmp112 * _tmp132 *
            (_tmp129 * _tmp194 - _tmp129 * _tmp256 - _tmp129 * _tmp261 + _tmp130 * _tmp258 -
             _tmp130 * _tmp259 - _tmp130 * _tmp260 + _tmp290 * _tmp95 + _tmp293 * _tmp96) -
        _tmp133 * _tmp272 + _tmp133 * _tmp302 -
        _tmp143 * (_tmp139 * _tmp194 - _tmp139 * _tmp256 - _tmp139 * _tmp261 + _tmp140 * _tmp258 -
                   _tmp140 * _tmp259 - _tmp140 * _tmp260 + _tmp299 * _tmp95 + _tmp300 * _tmp96) -
        _tmp148 * _tmp303 + _tmp186 * _tmp188 + _tmp186 * _tmp264 + _tmp186 * _tmp271 +
        _tmp186 * _tmp273 - _tmp188 * _tmp263 - _tmp263 * _tmp264 - _tmp263 * _tmp271 -
        _tmp263 * _tmp273 + _tmp268 * _tmp269 - _tmp269 * _tmp301) *
      std::exp(_tmp105 * _tmp99 + _tmp112 * _tmp133 + _tmp141 * _tmp143 + _tmp144 * _tmp148);
  _res(2, 0) =
      -(_tmp112 * _tmp293 * _tmp307 - _tmp130 * _tmp187 * _tmp250 - _tmp140 * _tmp250 * _tmp270 +
        _tmp142 * _tmp300 * _tmp307 - _tmp247 * _tmp304 - _tmp247 * _tmp313 + _tmp250 * _tmp306 +
        _tmp251 * _tmp304 - _tmp268 * _tmp309 + _tmp272 * _tmp308 + _tmp301 * _tmp309 -
        _tmp302 * _tmp308 + _tmp310 * _tmp90 + _tmp311 * _tmp90 - _tmp312 * _tmp90) *
      std::exp(-_tmp112 * _tmp308 - _tmp142 * _tmp309 + _tmp304 * _tmp90 + _tmp306 * _tmp89);
  _res(3, 0) = -(-_tmp129 * _tmp187 * _tmp254 - _tmp139 * _tmp254 * _tmp270 - _tmp268 * _tmp318 +
                 _tmp272 * _tmp317 + _tmp290 * _tmp315 + _tmp299 * _tmp316 + _tmp301 * _tmp318 -
                 _tmp302 * _tmp317 - _tmp310 - _tmp311 + _tmp312) *
               std::exp(-_tmp129 * _tmp315 - _tmp139 * _tmp316 - _tmp304 - _tmp313);

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym