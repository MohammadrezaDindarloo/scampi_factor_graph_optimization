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
 * Symbolic function: IK_residual_func_cost3_wrt_rz_Nl12
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost3WrtRzNl12(
    const Scalar fh1, const Scalar fv1, const Scalar rx, const Scalar ry, const Scalar rz,
    const Scalar p_init0, const Scalar p_init1, const Scalar p_init2, const Scalar rot_init_x,
    const Scalar rot_init_y, const Scalar rot_init_z, const Scalar rot_init_w,
    const Scalar epsilon) {
  // Total ops: 983

  // Unused inputs
  (void)p_init2;
  (void)epsilon;

  // Input arrays

  // Intermediate terms (314)
  const Scalar _tmp0 = std::pow(rz, Scalar(2));
  const Scalar _tmp1 = _tmp0 + std::pow(rx, Scalar(2)) + std::pow(ry, Scalar(2));
  const Scalar _tmp2 = std::sqrt(_tmp1);
  const Scalar _tmp3 = (Scalar(1) / Scalar(2)) * _tmp2;
  const Scalar _tmp4 = std::cos(_tmp3);
  const Scalar _tmp5 = _tmp4 * rot_init_x;
  const Scalar _tmp6 = std::sin(_tmp3);
  const Scalar _tmp7 = _tmp6 / _tmp2;
  const Scalar _tmp8 = _tmp7 * rot_init_z;
  const Scalar _tmp9 = _tmp7 * rot_init_w;
  const Scalar _tmp10 = _tmp7 * rot_init_y;
  const Scalar _tmp11 = _tmp10 * rz;
  const Scalar _tmp12 = _tmp11 + _tmp5 - _tmp8 * ry + _tmp9 * rx;
  const Scalar _tmp13 = -2 * std::pow(_tmp12, Scalar(2));
  const Scalar _tmp14 = _tmp4 * rot_init_z;
  const Scalar _tmp15 = _tmp7 * rot_init_x;
  const Scalar _tmp16 = _tmp9 * rz;
  const Scalar _tmp17 = -_tmp10 * rx + _tmp14 + _tmp15 * ry + _tmp16;
  const Scalar _tmp18 = 1 - 2 * std::pow(_tmp17, Scalar(2));
  const Scalar _tmp19 = Scalar(0.20999999999999999) * _tmp13 + Scalar(0.20999999999999999) * _tmp18;
  const Scalar _tmp20 = _tmp4 * rot_init_y;
  const Scalar _tmp21 = _tmp15 * rz;
  const Scalar _tmp22 = _tmp20 - _tmp21 + _tmp8 * rx + _tmp9 * ry;
  const Scalar _tmp23 = 2 * _tmp17;
  const Scalar _tmp24 = _tmp22 * _tmp23;
  const Scalar _tmp25 = _tmp4 * rot_init_w;
  const Scalar _tmp26 = _tmp8 * rz;
  const Scalar _tmp27 = -_tmp10 * ry - _tmp15 * rx + _tmp25 - _tmp26;
  const Scalar _tmp28 = 2 * _tmp27;
  const Scalar _tmp29 = _tmp12 * _tmp28;
  const Scalar _tmp30 =
      -Scalar(0.010999999999999999) * _tmp24 + Scalar(0.010999999999999999) * _tmp29;
  const Scalar _tmp31 = 2 * _tmp12 * _tmp22;
  const Scalar _tmp32 = _tmp17 * _tmp28;
  const Scalar _tmp33 = Scalar(0.20999999999999999) * _tmp31 + Scalar(0.20999999999999999) * _tmp32;
  const Scalar _tmp34 = _tmp30 + _tmp33;
  const Scalar _tmp35 = _tmp19 + _tmp34;
  const Scalar _tmp36 = _tmp35 + p_init1 + Scalar(-4.7752063900000001);
  const Scalar _tmp37 = -2 * std::pow(_tmp22, Scalar(2));
  const Scalar _tmp38 = Scalar(0.20999999999999999) * _tmp18 + Scalar(0.20999999999999999) * _tmp37;
  const Scalar _tmp39 = Scalar(0.20999999999999999) * _tmp31 - Scalar(0.20999999999999999) * _tmp32;
  const Scalar _tmp40 = _tmp12 * _tmp23;
  const Scalar _tmp41 = _tmp22 * _tmp28;
  const Scalar _tmp42 =
      -Scalar(0.010999999999999999) * _tmp40 - Scalar(0.010999999999999999) * _tmp41;
  const Scalar _tmp43 = _tmp39 + _tmp42;
  const Scalar _tmp44 = _tmp38 + _tmp43;
  const Scalar _tmp45 = _tmp44 + p_init0 + Scalar(-2.71799795);
  const Scalar _tmp46 = std::pow(_tmp36, Scalar(2)) + std::pow(_tmp45, Scalar(2));
  const Scalar _tmp47 = std::pow(_tmp46, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp48 = _tmp36 * _tmp47;
  const Scalar _tmp49 = _tmp48 * fh1;
  const Scalar _tmp50 = _tmp30 - _tmp33;
  const Scalar _tmp51 = _tmp19 + _tmp50;
  const Scalar _tmp52 = _tmp51 + p_init1 + Scalar(-4.8333311099999996);
  const Scalar _tmp53 = -_tmp38;
  const Scalar _tmp54 = _tmp43 + _tmp53;
  const Scalar _tmp55 = _tmp54 + p_init0 + Scalar(1.79662371);
  const Scalar _tmp56 = std::pow(_tmp52, Scalar(2)) + std::pow(_tmp55, Scalar(2));
  const Scalar _tmp57 = std::pow(_tmp56, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp58 = _tmp55 * _tmp57;
  const Scalar _tmp59 = -_tmp19;
  const Scalar _tmp60 = _tmp50 + _tmp59;
  const Scalar _tmp61 = _tmp60 + p_init1 + Scalar(8.3196563700000006);
  const Scalar _tmp62 = -_tmp39 + _tmp42;
  const Scalar _tmp63 = _tmp53 + _tmp62;
  const Scalar _tmp64 = _tmp63 + p_init0 + Scalar(1.9874742000000001);
  const Scalar _tmp65 = Scalar(1.0) / (_tmp64);
  const Scalar _tmp66 = _tmp61 * _tmp65;
  const Scalar _tmp67 = Scalar(0.20999999999999999) * _tmp24 + Scalar(0.20999999999999999) * _tmp29;
  const Scalar _tmp68 = -_tmp67;
  const Scalar _tmp69 = -Scalar(0.010999999999999999) * _tmp13 -
                        Scalar(0.010999999999999999) * _tmp37 + Scalar(-0.010999999999999999);
  const Scalar _tmp70 = Scalar(0.20999999999999999) * _tmp40 - Scalar(0.20999999999999999) * _tmp41;
  const Scalar _tmp71 = _tmp69 - _tmp70;
  const Scalar _tmp72 = _tmp68 + _tmp71;
  const Scalar _tmp73 = _tmp58 * _tmp72;
  const Scalar _tmp74 = _tmp67 + _tmp71;
  const Scalar _tmp75 = _tmp52 * _tmp57;
  const Scalar _tmp76 = _tmp68 + _tmp69 + _tmp70;
  const Scalar _tmp77 = _tmp38 + _tmp62;
  const Scalar _tmp78 = _tmp77 + p_init0 + Scalar(-2.5202214700000001);
  const Scalar _tmp79 = _tmp34 + _tmp59;
  const Scalar _tmp80 = _tmp79 + p_init1 + Scalar(8.3888750099999996);
  const Scalar _tmp81 = std::pow(_tmp78, Scalar(2)) + std::pow(_tmp80, Scalar(2));
  const Scalar _tmp82 = std::pow(_tmp81, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp83 = _tmp80 * _tmp82;
  const Scalar _tmp84 = _tmp78 * _tmp82;
  const Scalar _tmp85 = _tmp72 * _tmp84;
  const Scalar _tmp86 = -_tmp66 * _tmp85 + _tmp76 * _tmp83;
  const Scalar _tmp87 = _tmp66 * _tmp84 - _tmp83;
  const Scalar _tmp88 = Scalar(1.0) / (_tmp87);
  const Scalar _tmp89 = _tmp58 * _tmp66 - _tmp75;
  const Scalar _tmp90 = _tmp88 * _tmp89;
  const Scalar _tmp91 = -_tmp66 * _tmp73 + _tmp74 * _tmp75 - _tmp86 * _tmp90;
  const Scalar _tmp92 = Scalar(1.0) * _tmp60;
  const Scalar _tmp93 = _tmp79 - _tmp92;
  const Scalar _tmp94 = Scalar(1.0) / (_tmp93);
  const Scalar _tmp95 = Scalar(1.0) * _tmp63;
  const Scalar _tmp96 = -_tmp77 + _tmp95;
  const Scalar _tmp97 = _tmp94 * _tmp96;
  const Scalar _tmp98 = -_tmp76 * _tmp84 + _tmp85;
  const Scalar _tmp99 = -_tmp58 * _tmp74 + _tmp73 - _tmp90 * _tmp98 - _tmp91 * _tmp97;
  const Scalar _tmp100 = Scalar(1.0) / (_tmp99);
  const Scalar _tmp101 = std::pow(_tmp64, Scalar(2));
  const Scalar _tmp102 = _tmp101 + std::pow(_tmp61, Scalar(2));
  const Scalar _tmp103 = std::sqrt(_tmp102);
  const Scalar _tmp104 = Scalar(1.0) / (_tmp103);
  const Scalar _tmp105 = _tmp104 * _tmp64;
  const Scalar _tmp106 = _tmp104 * _tmp63;
  const Scalar _tmp107 = -_tmp105 * _tmp60 + _tmp106 * _tmp61;
  const Scalar _tmp108 = _tmp103 * _tmp65;
  const Scalar _tmp109 = _tmp107 * _tmp108;
  const Scalar _tmp110 = _tmp109 * _tmp84 - _tmp77 * _tmp83 + _tmp79 * _tmp84;
  const Scalar _tmp111 = Scalar(1.0) * _tmp88;
  const Scalar _tmp112 = _tmp109 * _tmp58 - _tmp110 * _tmp90 + _tmp51 * _tmp58 - _tmp54 * _tmp75;
  const Scalar _tmp113 = _tmp111 * _tmp97;
  const Scalar _tmp114 = -_tmp111 * _tmp98 + _tmp113 * _tmp86;
  const Scalar _tmp115 = _tmp100 * _tmp114;
  const Scalar _tmp116 = -_tmp110 * _tmp111 - _tmp112 * _tmp115;
  const Scalar _tmp117 = Scalar(1.0) / (_tmp112);
  const Scalar _tmp118 = _tmp117 * _tmp99;
  const Scalar _tmp119 = _tmp114 + _tmp116 * _tmp118;
  const Scalar _tmp120 = _tmp100 * _tmp119;
  const Scalar _tmp121 = _tmp100 * _tmp89;
  const Scalar _tmp122 = -_tmp119 * _tmp121 + Scalar(1.0);
  const Scalar _tmp123 = _tmp84 * _tmp88;
  const Scalar _tmp124 = _tmp120 * _tmp58 + _tmp122 * _tmp123;
  const Scalar _tmp125 = _tmp108 * _tmp124;
  const Scalar _tmp126 = Scalar(1.0) * _tmp117;
  const Scalar _tmp127 = _tmp84 * _tmp90;
  const Scalar _tmp128 = -_tmp126 * _tmp127 + _tmp126 * _tmp58;
  const Scalar _tmp129 = _tmp45 * _tmp47;
  const Scalar _tmp130 = fh1 * (-_tmp129 * _tmp35 + _tmp44 * _tmp48);
  const Scalar _tmp131 = _tmp108 * _tmp130;
  const Scalar _tmp132 = _tmp92 * _tmp97 + _tmp95;
  const Scalar _tmp133 = 0;
  const Scalar _tmp134 = _tmp100 * _tmp133;
  const Scalar _tmp135 = -_tmp127 * _tmp134 + _tmp134 * _tmp58;
  const Scalar _tmp136 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp137 = _tmp108 * _tmp136;
  const Scalar _tmp138 = _tmp129 * fh1;
  const Scalar _tmp139 = _tmp66 * _tmp72;
  const Scalar _tmp140 = _tmp66 * _tmp88;
  const Scalar _tmp141 = _tmp139 + _tmp140 * _tmp86;
  const Scalar _tmp142 = _tmp140 * _tmp98 - _tmp141 * _tmp97 - _tmp72;
  const Scalar _tmp143 = _tmp100 * _tmp142;
  const Scalar _tmp144 = -_tmp109 + _tmp110 * _tmp140 - _tmp112 * _tmp143;
  const Scalar _tmp145 = _tmp118 * _tmp144 + _tmp142;
  const Scalar _tmp146 = -_tmp121 * _tmp145 - _tmp66;
  const Scalar _tmp147 = _tmp146 * _tmp88;
  const Scalar _tmp148 = _tmp100 * _tmp145;
  const Scalar _tmp149 = _tmp147 * _tmp84 + _tmp148 * _tmp58 + Scalar(1.0);
  const Scalar _tmp150 = _tmp108 * _tmp149;
  const Scalar _tmp151 = (Scalar(1) / Scalar(2)) / _tmp1;
  const Scalar _tmp152 = _tmp0 * _tmp151;
  const Scalar _tmp153 = _tmp151 * rz;
  const Scalar _tmp154 = _tmp153 * ry;
  const Scalar _tmp155 = _tmp153 * rx;
  const Scalar _tmp156 = _tmp6 / (_tmp1 * std::sqrt(_tmp1));
  const Scalar _tmp157 = _tmp0 * _tmp156;
  const Scalar _tmp158 = _tmp156 * rz;
  const Scalar _tmp159 = _tmp158 * rx;
  const Scalar _tmp160 = _tmp158 * ry;
  const Scalar _tmp161 = _tmp10 - _tmp14 * _tmp154 + _tmp152 * _tmp20 + _tmp155 * _tmp25 -
                         _tmp157 * rot_init_y - _tmp159 * rot_init_w + _tmp160 * rot_init_z -
                         Scalar(1) / Scalar(2) * _tmp21;
  const Scalar _tmp162 = Scalar(0.83999999999999997) * _tmp161;
  const Scalar _tmp163 = _tmp12 * _tmp162;
  const Scalar _tmp164 = _tmp152 * _tmp25 + _tmp154 * _tmp5 - _tmp155 * _tmp20 -
                         _tmp157 * rot_init_w + _tmp159 * rot_init_y - _tmp160 * rot_init_x -
                         Scalar(1) / Scalar(2) * _tmp26 + _tmp9;
  const Scalar _tmp165 = Scalar(0.83999999999999997) * _tmp17;
  const Scalar _tmp166 = _tmp164 * _tmp165;
  const Scalar _tmp167 = -_tmp14 * _tmp152 - _tmp154 * _tmp20 - _tmp155 * _tmp5 +
                         _tmp157 * rot_init_z + _tmp159 * rot_init_x -
                         Scalar(1) / Scalar(2) * _tmp16 + _tmp160 * rot_init_y - _tmp8;
  const Scalar _tmp168 = Scalar(0.41999999999999998) * _tmp167;
  const Scalar _tmp169 = _tmp168 * _tmp17;
  const Scalar _tmp170 = _tmp164 * _tmp27;
  const Scalar _tmp171 = Scalar(0.41999999999999998) * _tmp170;
  const Scalar _tmp172 = -_tmp169 - _tmp171;
  const Scalar _tmp173 = _tmp166 + _tmp172;
  const Scalar _tmp174 = Scalar(0.41999999999999998) * _tmp22;
  const Scalar _tmp175 = _tmp161 * _tmp174;
  const Scalar _tmp176 = -Scalar(1) / Scalar(2) * _tmp11 + _tmp14 * _tmp155 - _tmp15 -
                         _tmp152 * _tmp5 + _tmp154 * _tmp25 + _tmp157 * rot_init_x -
                         _tmp159 * rot_init_z - _tmp160 * rot_init_w;
  const Scalar _tmp177 = Scalar(0.41999999999999998) * _tmp12;
  const Scalar _tmp178 = _tmp176 * _tmp177;
  const Scalar _tmp179 = -_tmp175 - _tmp178;
  const Scalar _tmp180 = Scalar(0.021999999999999999) * _tmp167;
  const Scalar _tmp181 = Scalar(0.021999999999999999) * _tmp17;
  const Scalar _tmp182 = Scalar(0.021999999999999999) * _tmp164;
  const Scalar _tmp183 = _tmp161 * _tmp27;
  const Scalar _tmp184 = _tmp12 * _tmp180 - _tmp176 * _tmp181 - _tmp182 * _tmp22 +
                         Scalar(0.021999999999999999) * _tmp183;
  const Scalar _tmp185 = _tmp179 + _tmp184;
  const Scalar _tmp186 = _tmp163 + _tmp173 + _tmp185;
  const Scalar _tmp187 = Scalar(0.83999999999999997) * _tmp176;
  const Scalar _tmp188 = _tmp187 * _tmp22;
  const Scalar _tmp189 = _tmp176 * _tmp27;
  const Scalar _tmp190 = -_tmp12 * _tmp182 - _tmp161 * _tmp181 - _tmp180 * _tmp22 -
                         Scalar(0.021999999999999999) * _tmp189;
  const Scalar _tmp191 = _tmp188 + _tmp190;
  const Scalar _tmp192 = _tmp169 + _tmp171;
  const Scalar _tmp193 = _tmp166 + _tmp192;
  const Scalar _tmp194 = _tmp179 + _tmp191 + _tmp193;
  const Scalar _tmp195 = _tmp186 * _tmp61 + _tmp194 * _tmp64;
  const Scalar _tmp196 = _tmp104 * _tmp195 * _tmp65;
  const Scalar _tmp197 = _tmp138 * _tmp149;
  const Scalar _tmp198 = _tmp128 * _tmp130;
  const Scalar _tmp199 = _tmp194 / _tmp101;
  const Scalar _tmp200 = _tmp103 * _tmp199;
  const Scalar _tmp201 = -_tmp163;
  const Scalar _tmp202 = -_tmp166;
  const Scalar _tmp203 = _tmp192 + _tmp202;
  const Scalar _tmp204 = _tmp175 + _tmp178;
  const Scalar _tmp205 = _tmp184 + _tmp204;
  const Scalar _tmp206 = _tmp201 + _tmp203 + _tmp205;
  const Scalar _tmp207 = _tmp172 + _tmp202;
  const Scalar _tmp208 = -_tmp188 + _tmp190;
  const Scalar _tmp209 = _tmp204 + _tmp207 + _tmp208;
  const Scalar _tmp210 = (Scalar(1) / Scalar(2)) * (2 * _tmp206 * _tmp36 + 2 * _tmp209 * _tmp45) /
                         (_tmp46 * std::sqrt(_tmp46));
  const Scalar _tmp211 = _tmp210 * _tmp36;
  const Scalar _tmp212 = _tmp211 * fh1;
  const Scalar _tmp213 = _tmp210 * _tmp45;
  const Scalar _tmp214 = _tmp213 * fh1;
  const Scalar _tmp215 = _tmp173 + _tmp191 + _tmp204;
  const Scalar _tmp216 = _tmp185 + _tmp201 + _tmp207;
  const Scalar _tmp217 =
      (2 * _tmp215 * _tmp55 + 2 * _tmp216 * _tmp52) / (_tmp56 * std::sqrt(_tmp56));
  const Scalar _tmp218 = Scalar(0.5) * _tmp117;
  const Scalar _tmp219 = _tmp179 + _tmp203 + _tmp208;
  const Scalar _tmp220 = _tmp219 * _tmp82;
  const Scalar _tmp221 = _tmp220 * _tmp90;
  const Scalar _tmp222 = _tmp107 * _tmp200;
  const Scalar _tmp223 = _tmp163 + _tmp193 + _tmp205;
  const Scalar _tmp224 = _tmp223 * _tmp82;
  const Scalar _tmp225 =
      (2 * _tmp219 * _tmp78 + 2 * _tmp223 * _tmp80) / (_tmp81 * std::sqrt(_tmp81));
  const Scalar _tmp226 = (Scalar(1) / Scalar(2)) * _tmp225;
  const Scalar _tmp227 = _tmp226 * _tmp80;
  const Scalar _tmp228 = _tmp226 * _tmp78;
  const Scalar _tmp229 = _tmp104 * _tmp194;
  const Scalar _tmp230 = _tmp195 / (_tmp102 * std::sqrt(_tmp102));
  const Scalar _tmp231 =
      _tmp108 * (-_tmp105 * _tmp186 + _tmp106 * _tmp186 - _tmp229 * _tmp60 + _tmp229 * _tmp61 +
                 _tmp230 * _tmp60 * _tmp64 - _tmp230 * _tmp61 * _tmp63);
  const Scalar _tmp232 = _tmp107 * _tmp196;
  const Scalar _tmp233 = _tmp109 * _tmp220 - _tmp109 * _tmp228 - _tmp219 * _tmp83 +
                         _tmp220 * _tmp79 - _tmp222 * _tmp84 - _tmp224 * _tmp77 + _tmp224 * _tmp78 +
                         _tmp227 * _tmp77 - _tmp228 * _tmp79 + _tmp231 * _tmp84 + _tmp232 * _tmp84;
  const Scalar _tmp234 = (Scalar(1) / Scalar(2)) * _tmp217;
  const Scalar _tmp235 = _tmp234 * _tmp52;
  const Scalar _tmp236 = _tmp199 * _tmp61;
  const Scalar _tmp237 = _tmp186 * _tmp65;
  const Scalar _tmp238 = (_tmp220 * _tmp66 - _tmp224 + _tmp227 - _tmp228 * _tmp66 -
                          _tmp236 * _tmp84 + _tmp237 * _tmp84) /
                         std::pow(_tmp87, Scalar(2));
  const Scalar _tmp239 = _tmp238 * _tmp89;
  const Scalar _tmp240 = _tmp216 * _tmp57;
  const Scalar _tmp241 = _tmp215 * _tmp57;
  const Scalar _tmp242 = _tmp234 * _tmp55;
  const Scalar _tmp243 =
      _tmp235 - _tmp236 * _tmp58 + _tmp237 * _tmp58 - _tmp240 + _tmp241 * _tmp66 - _tmp242 * _tmp66;
  const Scalar _tmp244 = _tmp243 * _tmp88;
  const Scalar _tmp245 = _tmp109 * _tmp241 - _tmp109 * _tmp242 + _tmp110 * _tmp239 -
                         _tmp110 * _tmp244 - _tmp215 * _tmp75 - _tmp222 * _tmp58 +
                         _tmp231 * _tmp58 + _tmp232 * _tmp58 - _tmp233 * _tmp90 + _tmp235 * _tmp54 -
                         _tmp240 * _tmp54 + _tmp240 * _tmp55 + _tmp241 * _tmp51 - _tmp242 * _tmp51;
  const Scalar _tmp246 = _tmp245 / std::pow(_tmp112, Scalar(2));
  const Scalar _tmp247 = Scalar(1.0) * _tmp246;
  const Scalar _tmp248 = _tmp244 * _tmp84;
  const Scalar _tmp249 = Scalar(1.0) * _tmp238;
  const Scalar _tmp250 = _tmp117 * _tmp249 * _tmp89;
  const Scalar _tmp251 = _tmp238 * _tmp84;
  const Scalar _tmp252 = _tmp121 * _tmp133;
  const Scalar _tmp253 = _tmp176 * _tmp22;
  const Scalar _tmp254 = _tmp94 * (Scalar(1.6799999999999999) * _tmp164 * _tmp17 +
                                   Scalar(1.6799999999999999) * _tmp253);
  const Scalar _tmp255 = _tmp220 * _tmp72;
  const Scalar _tmp256 = Scalar(0.043999999999999997) * _tmp12 * _tmp161;
  const Scalar _tmp257 = Scalar(0.043999999999999997) * _tmp253;
  const Scalar _tmp258 = _tmp167 * _tmp177;
  const Scalar _tmp259 = Scalar(0.41999999999999998) * _tmp17;
  const Scalar _tmp260 = _tmp176 * _tmp259;
  const Scalar _tmp261 = _tmp164 * _tmp174;
  const Scalar _tmp262 = Scalar(0.41999999999999998) * _tmp183;
  const Scalar _tmp263 = -_tmp258 - _tmp260 - _tmp261 - _tmp262;
  const Scalar _tmp264 = _tmp168 * _tmp22;
  const Scalar _tmp265 = _tmp161 * _tmp259;
  const Scalar _tmp266 = _tmp164 * _tmp177;
  const Scalar _tmp267 = Scalar(0.41999999999999998) * _tmp189;
  const Scalar _tmp268 = -_tmp264 + _tmp265 + _tmp266 - _tmp267;
  const Scalar _tmp269 = _tmp256 + _tmp257 + _tmp263 + _tmp268;
  const Scalar _tmp270 = _tmp256 + _tmp257 + _tmp264 - _tmp265 - _tmp266 + _tmp267;
  const Scalar _tmp271 = _tmp263 + _tmp270;
  const Scalar _tmp272 = -_tmp220 * _tmp76 - _tmp228 * _tmp72 + _tmp228 * _tmp76 + _tmp255 -
                         _tmp269 * _tmp84 + _tmp271 * _tmp84;
  const Scalar _tmp273 = _tmp96 *
                         (_tmp12 * _tmp187 + _tmp162 * _tmp22 + _tmp165 * _tmp167 +
                          Scalar(0.83999999999999997) * _tmp170) /
                         std::pow(_tmp93, Scalar(2));
  const Scalar _tmp274 = _tmp241 * _tmp72;
  const Scalar _tmp275 = _tmp271 * _tmp58;
  const Scalar _tmp276 = _tmp258 + _tmp260 + _tmp261 + _tmp262;
  const Scalar _tmp277 = _tmp270 + _tmp276;
  const Scalar _tmp278 = _tmp271 * _tmp66;
  const Scalar _tmp279 = _tmp139 * _tmp228 + _tmp224 * _tmp76 - _tmp227 * _tmp76 +
                         _tmp236 * _tmp85 - _tmp237 * _tmp85 - _tmp255 * _tmp66 + _tmp269 * _tmp83 -
                         _tmp278 * _tmp84;
  const Scalar _tmp280 =
      _tmp239 * _tmp98 - _tmp241 * _tmp74 - _tmp242 * _tmp72 + _tmp242 * _tmp74 - _tmp244 * _tmp98 -
      _tmp254 * _tmp91 - _tmp272 * _tmp90 + _tmp273 * _tmp91 + _tmp274 + _tmp275 -
      _tmp277 * _tmp58 -
      _tmp97 * (_tmp139 * _tmp242 - _tmp235 * _tmp74 + _tmp236 * _tmp73 - _tmp237 * _tmp73 +
                _tmp239 * _tmp86 + _tmp240 * _tmp74 - _tmp244 * _tmp86 - _tmp274 * _tmp66 -
                _tmp275 * _tmp66 + _tmp277 * _tmp75 - _tmp279 * _tmp90);
  const Scalar _tmp281 = _tmp280 / std::pow(_tmp99, Scalar(2));
  const Scalar _tmp282 = _tmp133 * _tmp281;
  const Scalar _tmp283 = _tmp206 * _tmp47;
  const Scalar _tmp284 = _tmp283 * fh1;
  const Scalar _tmp285 = _tmp135 * _tmp136;
  const Scalar _tmp286 = _tmp246 * _tmp99;
  const Scalar _tmp287 = _tmp117 * _tmp280;
  const Scalar _tmp288 = _tmp237 * _tmp88;
  const Scalar _tmp289 = _tmp238 * _tmp66;
  const Scalar _tmp290 = _tmp236 * _tmp88;
  const Scalar _tmp291 =
      _tmp140 * _tmp272 - _tmp141 * _tmp254 + _tmp141 * _tmp273 - _tmp256 - _tmp257 + _tmp268 +
      _tmp276 + _tmp288 * _tmp98 - _tmp289 * _tmp98 - _tmp290 * _tmp98 -
      _tmp97 * (_tmp140 * _tmp279 - _tmp236 * _tmp72 + _tmp237 * _tmp72 + _tmp278 +
                _tmp288 * _tmp86 - _tmp289 * _tmp86 - _tmp290 * _tmp86);
  const Scalar _tmp292 = _tmp100 * _tmp112;
  const Scalar _tmp293 = _tmp112 * _tmp281;
  const Scalar _tmp294 = _tmp118 * (_tmp110 * _tmp288 - _tmp110 * _tmp289 - _tmp110 * _tmp290 +
                                    _tmp140 * _tmp233 + _tmp142 * _tmp293 - _tmp143 * _tmp245 +
                                    _tmp222 - _tmp231 - _tmp232 - _tmp291 * _tmp292) -
                         _tmp144 * _tmp286 + _tmp144 * _tmp287 + _tmp291;
  const Scalar _tmp295 = _tmp100 * _tmp58;
  const Scalar _tmp296 = _tmp281 * _tmp58;
  const Scalar _tmp297 = _tmp100 * _tmp243;
  const Scalar _tmp298 = _tmp281 * _tmp89;
  const Scalar _tmp299 =
      -_tmp121 * _tmp294 - _tmp145 * _tmp297 + _tmp145 * _tmp298 + _tmp236 - _tmp237;
  const Scalar _tmp300 = _tmp124 * _tmp49;
  const Scalar _tmp301 = _tmp122 * _tmp88;
  const Scalar _tmp302 = _tmp111 * _tmp86;
  const Scalar _tmp303 = -_tmp111 * _tmp272 + _tmp113 * _tmp279 - _tmp249 * _tmp86 * _tmp97 +
                         _tmp249 * _tmp98 + _tmp254 * _tmp302 - _tmp273 * _tmp302;
  const Scalar _tmp304 = -_tmp116 * _tmp286 + _tmp116 * _tmp287 +
                         _tmp118 * (_tmp110 * _tmp249 - _tmp111 * _tmp233 + _tmp114 * _tmp293 -
                                    _tmp115 * _tmp245 - _tmp292 * _tmp303) +
                         _tmp303;
  const Scalar _tmp305 = -_tmp119 * _tmp297 + _tmp119 * _tmp298 - _tmp121 * _tmp304;
  const Scalar _tmp306 = _tmp209 * _tmp47;
  const Scalar _tmp307 = fh1 * (-_tmp129 * _tmp206 + _tmp209 * _tmp48 - _tmp211 * _tmp44 +
                                _tmp213 * _tmp35 + _tmp283 * _tmp44 - _tmp306 * _tmp35);
  const Scalar _tmp308 = _tmp306 * fh1;
  const Scalar _tmp309 = _tmp134 * _tmp136;
  const Scalar _tmp310 = _tmp126 * _tmp130;
  const Scalar _tmp311 = _tmp130 * _tmp247;
  const Scalar _tmp312 = _tmp136 * _tmp282;
  const Scalar _tmp313 = _tmp126 * _tmp307;

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) = 0;
  _res(1, 0) =
      -(-_tmp108 * _tmp128 * _tmp307 -
        _tmp108 * _tmp138 *
            (_tmp123 * _tmp299 - _tmp145 * _tmp296 - _tmp146 * _tmp251 + _tmp147 * _tmp220 -
             _tmp147 * _tmp228 + _tmp148 * _tmp241 - _tmp148 * _tmp242 + _tmp294 * _tmp295) -
        _tmp108 * _tmp49 *
            (-_tmp119 * _tmp296 + _tmp120 * _tmp241 - _tmp120 * _tmp242 - _tmp122 * _tmp251 +
             _tmp123 * _tmp305 + _tmp220 * _tmp301 - _tmp228 * _tmp301 + _tmp295 * _tmp304) +
        _tmp125 * _tmp212 - _tmp125 * _tmp284 -
        _tmp131 * (-_tmp126 * _tmp221 + _tmp126 * _tmp241 - _tmp126 * _tmp248 + _tmp127 * _tmp247 -
                   _tmp217 * _tmp218 * _tmp55 + _tmp218 * _tmp225 * _tmp78 * _tmp90 -
                   _tmp247 * _tmp58 + _tmp250 * _tmp84) -
        _tmp137 * (_tmp127 * _tmp282 - _tmp134 * _tmp221 + _tmp134 * _tmp228 * _tmp90 +
                   _tmp134 * _tmp241 - _tmp134 * _tmp242 - _tmp134 * _tmp248 + _tmp251 * _tmp252 -
                   _tmp282 * _tmp58) +
        _tmp150 * _tmp214 - _tmp150 * _tmp308 - _tmp196 * _tmp197 - _tmp196 * _tmp198 -
        _tmp196 * _tmp285 - _tmp196 * _tmp300 + _tmp197 * _tmp200 + _tmp198 * _tmp200 +
        _tmp200 * _tmp285 + _tmp200 * _tmp300) *
      std::exp(_tmp125 * _tmp49 + _tmp128 * _tmp131 + _tmp135 * _tmp137 + _tmp138 * _tmp150);
  _res(2, 0) =
      -(-_tmp122 * _tmp238 * _tmp49 + _tmp130 * _tmp250 + _tmp136 * _tmp238 * _tmp252 -
        _tmp138 * _tmp146 * _tmp238 + _tmp138 * _tmp299 * _tmp88 - _tmp147 * _tmp214 +
        _tmp147 * _tmp308 - _tmp212 * _tmp301 - _tmp244 * _tmp309 - _tmp244 * _tmp310 +
        _tmp284 * _tmp301 + _tmp305 * _tmp49 * _tmp88 + _tmp311 * _tmp90 + _tmp312 * _tmp90 -
        _tmp313 * _tmp90) *
      std::exp(-_tmp138 * _tmp147 - _tmp301 * _tmp49 + _tmp309 * _tmp90 + _tmp310 * _tmp90);
  _res(3, 0) =
      -(_tmp100 * _tmp138 * _tmp294 + _tmp100 * _tmp304 * _tmp49 - _tmp119 * _tmp281 * _tmp49 -
        _tmp120 * _tmp212 + _tmp120 * _tmp284 - _tmp138 * _tmp145 * _tmp281 - _tmp148 * _tmp214 +
        _tmp148 * _tmp308 - _tmp311 - _tmp312 + _tmp313) *
      std::exp(-_tmp120 * _tmp49 - _tmp138 * _tmp148 - _tmp309 - _tmp310);

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym