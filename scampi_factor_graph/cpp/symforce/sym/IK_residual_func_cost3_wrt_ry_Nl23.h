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
 * Symbolic function: IK_residual_func_cost3_wrt_ry_Nl23
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost3WrtRyNl23(
    const Scalar fh1, const Scalar fv1, const Scalar rx, const Scalar ry, const Scalar rz,
    const Scalar p_init0, const Scalar p_init1, const Scalar p_init2, const Scalar rot_init_x,
    const Scalar rot_init_y, const Scalar rot_init_z, const Scalar rot_init_w,
    const Scalar epsilon) {
  // Total ops: 991

  // Unused inputs
  (void)p_init2;
  (void)epsilon;

  // Input arrays

  // Intermediate terms (321)
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
  const Scalar _tmp20 = _tmp4 * rot_init_y;
  const Scalar _tmp21 = _tmp10 * ry;
  const Scalar _tmp22 = -_tmp15 * rz + _tmp20 + _tmp21 + _tmp8 * rx;
  const Scalar _tmp23 = 2 * _tmp17 * _tmp22;
  const Scalar _tmp24 = _tmp4 * rot_init_w;
  const Scalar _tmp25 = _tmp11 * ry;
  const Scalar _tmp26 = -_tmp15 * rx + _tmp24 - _tmp25 - _tmp8 * rz;
  const Scalar _tmp27 = 2 * _tmp26;
  const Scalar _tmp28 = _tmp12 * _tmp27;
  const Scalar _tmp29 =
      -Scalar(0.010999999999999999) * _tmp23 + Scalar(0.010999999999999999) * _tmp28;
  const Scalar _tmp30 = 2 * _tmp12;
  const Scalar _tmp31 = _tmp22 * _tmp30;
  const Scalar _tmp32 = _tmp17 * _tmp27;
  const Scalar _tmp33 = Scalar(0.20999999999999999) * _tmp31 + Scalar(0.20999999999999999) * _tmp32;
  const Scalar _tmp34 = _tmp29 + _tmp33;
  const Scalar _tmp35 = _tmp19 + _tmp34;
  const Scalar _tmp36 = Scalar(1.0) * _tmp35;
  const Scalar _tmp37 = -_tmp19;
  const Scalar _tmp38 = _tmp34 + _tmp37;
  const Scalar _tmp39 = -_tmp36 + _tmp38;
  const Scalar _tmp40 = Scalar(1.0) / (_tmp39);
  const Scalar _tmp41 = Scalar(0.20999999999999999) * _tmp31 - Scalar(0.20999999999999999) * _tmp32;
  const Scalar _tmp42 = -_tmp41;
  const Scalar _tmp43 = _tmp17 * _tmp30;
  const Scalar _tmp44 = _tmp22 * _tmp27;
  const Scalar _tmp45 =
      -Scalar(0.010999999999999999) * _tmp43 - Scalar(0.010999999999999999) * _tmp44;
  const Scalar _tmp46 = 1 - 2 * std::pow(_tmp22, Scalar(2));
  const Scalar _tmp47 = Scalar(0.20999999999999999) * _tmp18 + Scalar(0.20999999999999999) * _tmp46;
  const Scalar _tmp48 = _tmp45 + _tmp47;
  const Scalar _tmp49 = _tmp42 + _tmp48;
  const Scalar _tmp50 = _tmp41 + _tmp48;
  const Scalar _tmp51 = Scalar(1.0) * _tmp50;
  const Scalar _tmp52 = -_tmp49 + _tmp51;
  const Scalar _tmp53 = _tmp40 * _tmp52;
  const Scalar _tmp54 = _tmp36 * _tmp53 + _tmp51;
  const Scalar _tmp55 = 0;
  const Scalar _tmp56 = Scalar(0.20999999999999999) * _tmp43 - Scalar(0.20999999999999999) * _tmp44;
  const Scalar _tmp57 =
      -Scalar(0.010999999999999999) * _tmp13 - Scalar(0.010999999999999999) * _tmp46;
  const Scalar _tmp58 = Scalar(0.20999999999999999) * _tmp23 + Scalar(0.20999999999999999) * _tmp28;
  const Scalar _tmp59 = _tmp57 - _tmp58;
  const Scalar _tmp60 = _tmp56 + _tmp59;
  const Scalar _tmp61 = _tmp49 + p_init0 + Scalar(-2.5202214700000001);
  const Scalar _tmp62 = _tmp38 + p_init1 + Scalar(8.3888750099999996);
  const Scalar _tmp63 = std::pow(_tmp61, Scalar(2)) + std::pow(_tmp62, Scalar(2));
  const Scalar _tmp64 = std::pow(_tmp63, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp65 = _tmp61 * _tmp64;
  const Scalar _tmp66 = _tmp56 + _tmp57 + _tmp58;
  const Scalar _tmp67 = _tmp65 * _tmp66;
  const Scalar _tmp68 = -_tmp60 * _tmp65 + _tmp67;
  const Scalar _tmp69 = _tmp29 - _tmp33;
  const Scalar _tmp70 = _tmp37 + _tmp69;
  const Scalar _tmp71 = _tmp70 + p_init1 + Scalar(8.3196563700000006);
  const Scalar _tmp72 = _tmp45 - _tmp47;
  const Scalar _tmp73 = _tmp42 + _tmp72;
  const Scalar _tmp74 = _tmp73 + p_init0 + Scalar(1.9874742000000001);
  const Scalar _tmp75 = std::pow(_tmp71, Scalar(2)) + std::pow(_tmp74, Scalar(2));
  const Scalar _tmp76 = std::pow(_tmp75, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp77 = _tmp71 * _tmp76;
  const Scalar _tmp78 = _tmp50 + p_init0 + Scalar(-2.71799795);
  const Scalar _tmp79 = Scalar(1.0) / (_tmp78);
  const Scalar _tmp80 = _tmp35 + p_init1 + Scalar(-4.7752063900000001);
  const Scalar _tmp81 = _tmp79 * _tmp80;
  const Scalar _tmp82 = _tmp74 * _tmp76;
  const Scalar _tmp83 = -_tmp77 + _tmp81 * _tmp82;
  const Scalar _tmp84 = _tmp62 * _tmp64;
  const Scalar _tmp85 = _tmp65 * _tmp81 - _tmp84;
  const Scalar _tmp86 = Scalar(1.0) / (_tmp85);
  const Scalar _tmp87 = _tmp83 * _tmp86;
  const Scalar _tmp88 = _tmp66 * _tmp82;
  const Scalar _tmp89 = _tmp60 * _tmp84 - _tmp67 * _tmp81;
  const Scalar _tmp90 = _tmp86 * _tmp89;
  const Scalar _tmp91 = -_tmp56 + _tmp59;
  const Scalar _tmp92 = _tmp77 * _tmp91 - _tmp81 * _tmp88 - _tmp83 * _tmp90;
  const Scalar _tmp93 = -_tmp53 * _tmp92 - _tmp68 * _tmp87 - _tmp82 * _tmp91 + _tmp88;
  const Scalar _tmp94 = Scalar(1.0) / (_tmp93);
  const Scalar _tmp95 = _tmp82 * _tmp94;
  const Scalar _tmp96 = _tmp65 * _tmp86;
  const Scalar _tmp97 = _tmp83 * _tmp94;
  const Scalar _tmp98 = _tmp55 * _tmp97;
  const Scalar _tmp99 = _tmp55 * _tmp95 - _tmp96 * _tmp98;
  const Scalar _tmp100 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp101 = std::pow(_tmp78, Scalar(2));
  const Scalar _tmp102 = _tmp101 + std::pow(_tmp80, Scalar(2));
  const Scalar _tmp103 = std::sqrt(_tmp102);
  const Scalar _tmp104 = _tmp103 * _tmp79;
  const Scalar _tmp105 = _tmp100 * _tmp104;
  const Scalar _tmp106 = Scalar(1.0) / (_tmp103);
  const Scalar _tmp107 = _tmp106 * _tmp80;
  const Scalar _tmp108 = _tmp106 * _tmp35;
  const Scalar _tmp109 = _tmp107 * _tmp50 - _tmp108 * _tmp78;
  const Scalar _tmp110 = _tmp104 * _tmp109;
  const Scalar _tmp111 = _tmp110 * _tmp65 + _tmp38 * _tmp65 - _tmp49 * _tmp84;
  const Scalar _tmp112 = _tmp111 * _tmp86;
  const Scalar _tmp113 = _tmp110 * _tmp82 - _tmp112 * _tmp83 + _tmp70 * _tmp82 - _tmp73 * _tmp77;
  const Scalar _tmp114 = Scalar(1.0) / (_tmp113);
  const Scalar _tmp115 = Scalar(1.0) * _tmp114;
  const Scalar _tmp116 = _tmp115 * _tmp87;
  const Scalar _tmp117 = _tmp115 * _tmp82 - _tmp116 * _tmp65;
  const Scalar _tmp118 = _tmp19 + _tmp69;
  const Scalar _tmp119 = _tmp118 + p_init1 + Scalar(-4.8333311099999996);
  const Scalar _tmp120 = _tmp41 + _tmp72;
  const Scalar _tmp121 = _tmp120 + p_init0 + Scalar(1.79662371);
  const Scalar _tmp122 = std::pow(_tmp119, Scalar(2)) + std::pow(_tmp121, Scalar(2));
  const Scalar _tmp123 = std::pow(_tmp122, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp124 = _tmp120 * _tmp123;
  const Scalar _tmp125 = _tmp121 * _tmp123;
  const Scalar _tmp126 = -_tmp118 * _tmp125 + _tmp119 * _tmp124;
  const Scalar _tmp127 = _tmp126 * fh1;
  const Scalar _tmp128 = _tmp104 * _tmp127;
  const Scalar _tmp129 = _tmp66 * _tmp81;
  const Scalar _tmp130 = _tmp129 + _tmp81 * _tmp90;
  const Scalar _tmp131 = _tmp68 * _tmp86;
  const Scalar _tmp132 = -_tmp130 * _tmp53 + _tmp131 * _tmp81 - _tmp66;
  const Scalar _tmp133 = _tmp113 * _tmp94;
  const Scalar _tmp134 = -_tmp110 + _tmp112 * _tmp81 - _tmp132 * _tmp133;
  const Scalar _tmp135 = _tmp114 * _tmp93;
  const Scalar _tmp136 = _tmp132 + _tmp134 * _tmp135;
  const Scalar _tmp137 = -_tmp136 * _tmp97 - _tmp81;
  const Scalar _tmp138 = _tmp136 * _tmp95 + _tmp137 * _tmp96 + Scalar(1.0);
  const Scalar _tmp139 = _tmp125 * fh1;
  const Scalar _tmp140 = _tmp104 * _tmp139;
  const Scalar _tmp141 = Scalar(1.0) * _tmp53;
  const Scalar _tmp142 = -Scalar(1.0) * _tmp131 + _tmp141 * _tmp90;
  const Scalar _tmp143 = -Scalar(1.0) * _tmp112 - _tmp133 * _tmp142;
  const Scalar _tmp144 = _tmp114 * _tmp143;
  const Scalar _tmp145 = _tmp142 + _tmp144 * _tmp93;
  const Scalar _tmp146 = -_tmp145 * _tmp97 + Scalar(1.0);
  const Scalar _tmp147 = _tmp145 * _tmp95 + _tmp146 * _tmp96;
  const Scalar _tmp148 = _tmp119 * _tmp123;
  const Scalar _tmp149 = _tmp104 * fh1;
  const Scalar _tmp150 = _tmp148 * _tmp149;
  const Scalar _tmp151 = (Scalar(1) / Scalar(2)) / _tmp1;
  const Scalar _tmp152 = _tmp0 * _tmp151;
  const Scalar _tmp153 = _tmp151 * ry;
  const Scalar _tmp154 = _tmp153 * rz;
  const Scalar _tmp155 = _tmp153 * rx;
  const Scalar _tmp156 = _tmp6 / (_tmp1 * std::sqrt(_tmp1));
  const Scalar _tmp157 = _tmp0 * _tmp156;
  const Scalar _tmp158 = _tmp156 * ry;
  const Scalar _tmp159 = _tmp158 * rx;
  const Scalar _tmp160 = _tmp158 * rot_init_x;
  const Scalar _tmp161 = _tmp10 + _tmp14 * _tmp155 + _tmp152 * _tmp24 - _tmp154 * _tmp5 -
                         _tmp157 * rot_init_w - _tmp159 * rot_init_z + _tmp160 * rz -
                         Scalar(1) / Scalar(2) * _tmp25;
  const Scalar _tmp162 = Scalar(0.41999999999999998) * _tmp12;
  const Scalar _tmp163 = _tmp161 * _tmp162;
  const Scalar _tmp164 = _tmp158 * rz;
  const Scalar _tmp165 = -_tmp14 * _tmp152 + _tmp154 * _tmp20 + _tmp155 * _tmp24 +
                         _tmp157 * rot_init_z - _tmp159 * rot_init_w -
                         Scalar(1) / Scalar(2) * _tmp16 - _tmp164 * rot_init_y - _tmp8;
  const Scalar _tmp166 = Scalar(0.41999999999999998) * _tmp22;
  const Scalar _tmp167 = _tmp165 * _tmp166;
  const Scalar _tmp168 = _tmp163 + _tmp167;
  const Scalar _tmp169 = _tmp15 + _tmp152 * _tmp5 + _tmp154 * _tmp24 - _tmp155 * _tmp20 -
                         _tmp157 * rot_init_x + _tmp159 * rot_init_y - _tmp164 * rot_init_w -
                         Scalar(1) / Scalar(2) * _tmp9;
  const Scalar _tmp170 = Scalar(0.83999999999999997) * _tmp169;
  const Scalar _tmp171 = _tmp17 * _tmp170;
  const Scalar _tmp172 = -_tmp171;
  const Scalar _tmp173 = Scalar(0.41999999999999998) * _tmp26;
  const Scalar _tmp174 = _tmp169 * _tmp173;
  const Scalar _tmp175 = -_tmp11 - _tmp14 * _tmp154 - _tmp152 * _tmp20 - _tmp155 * _tmp5 +
                         _tmp157 * rot_init_y + _tmp160 * rx + _tmp164 * rot_init_z -
                         Scalar(1) / Scalar(2) * _tmp21;
  const Scalar _tmp176 = Scalar(0.41999999999999998) * _tmp17;
  const Scalar _tmp177 = _tmp175 * _tmp176;
  const Scalar _tmp178 = -_tmp174 - _tmp177;
  const Scalar _tmp179 = _tmp172 + _tmp178;
  const Scalar _tmp180 = Scalar(0.83999999999999997) * _tmp22;
  const Scalar _tmp181 = _tmp161 * _tmp180;
  const Scalar _tmp182 = Scalar(0.021999999999999999) * _tmp26;
  const Scalar _tmp183 = Scalar(0.021999999999999999) * _tmp175;
  const Scalar _tmp184 = Scalar(0.021999999999999999) * _tmp169;
  const Scalar _tmp185 = Scalar(0.021999999999999999) * _tmp17;
  const Scalar _tmp186 =
      -_tmp12 * _tmp184 - _tmp161 * _tmp182 - _tmp165 * _tmp185 - _tmp183 * _tmp22;
  const Scalar _tmp187 = -_tmp181 + _tmp186;
  const Scalar _tmp188 = _tmp168 + _tmp179 + _tmp187;
  const Scalar _tmp189 = _tmp188 / _tmp101;
  const Scalar _tmp190 = _tmp103 * _tmp189;
  const Scalar _tmp191 = _tmp148 * fh1;
  const Scalar _tmp192 = _tmp147 * _tmp191;
  const Scalar _tmp193 = _tmp171 + _tmp178;
  const Scalar _tmp194 = _tmp181 + _tmp186;
  const Scalar _tmp195 = _tmp168 + _tmp193 + _tmp194;
  const Scalar _tmp196 = _tmp123 * _tmp195;
  const Scalar _tmp197 = -_tmp163 - _tmp167;
  const Scalar _tmp198 = _tmp12 * _tmp165;
  const Scalar _tmp199 = Scalar(0.83999999999999997) * _tmp198;
  const Scalar _tmp200 =
      _tmp12 * _tmp183 - _tmp161 * _tmp185 + _tmp165 * _tmp182 - _tmp184 * _tmp22;
  const Scalar _tmp201 = -_tmp199 + _tmp200;
  const Scalar _tmp202 = _tmp179 + _tmp197 + _tmp201;
  const Scalar _tmp203 = (Scalar(1) / Scalar(2)) * (2 * _tmp119 * _tmp202 + 2 * _tmp121 * _tmp195) /
                         (_tmp122 * std::sqrt(_tmp122));
  const Scalar _tmp204 = _tmp121 * _tmp203;
  const Scalar _tmp205 = _tmp119 * _tmp203;
  const Scalar _tmp206 = -_tmp118 * _tmp196 + _tmp118 * _tmp204 + _tmp119 * _tmp196 -
                         _tmp120 * _tmp205 + _tmp124 * _tmp202 - _tmp125 * _tmp202;
  const Scalar _tmp207 = _tmp117 * _tmp127;
  const Scalar _tmp208 = _tmp174 + _tmp177;
  const Scalar _tmp209 = _tmp168 + _tmp208;
  const Scalar _tmp210 = _tmp172 + _tmp201 + _tmp209;
  const Scalar _tmp211 = _tmp188 * _tmp78 + _tmp210 * _tmp80;
  const Scalar _tmp212 = _tmp106 * _tmp211 * _tmp79;
  const Scalar _tmp213 = _tmp138 * _tmp139;
  const Scalar _tmp214 = _tmp138 * _tmp149;
  const Scalar _tmp215 = _tmp197 + _tmp208;
  const Scalar _tmp216 = _tmp172 + _tmp187 + _tmp215;
  const Scalar _tmp217 = _tmp216 * _tmp64;
  const Scalar _tmp218 = _tmp217 * _tmp86;
  const Scalar _tmp219 = _tmp171 + _tmp194 + _tmp215;
  const Scalar _tmp220 = _tmp219 * _tmp76;
  const Scalar _tmp221 = _tmp199 + _tmp200;
  const Scalar _tmp222 = _tmp193 + _tmp197 + _tmp221;
  const Scalar _tmp223 =
      (2 * _tmp219 * _tmp74 + 2 * _tmp222 * _tmp71) / (_tmp75 * std::sqrt(_tmp75));
  const Scalar _tmp224 = (Scalar(1) / Scalar(2)) * _tmp223;
  const Scalar _tmp225 = _tmp224 * _tmp74;
  const Scalar _tmp226 = _tmp171 + _tmp209 + _tmp221;
  const Scalar _tmp227 =
      (2 * _tmp216 * _tmp61 + 2 * _tmp226 * _tmp62) / (_tmp63 * std::sqrt(_tmp63));
  const Scalar _tmp228 = (Scalar(1) / Scalar(2)) * _tmp227;
  const Scalar _tmp229 = _tmp228 * _tmp61;
  const Scalar _tmp230 = _tmp210 * _tmp79;
  const Scalar _tmp231 = _tmp228 * _tmp62;
  const Scalar _tmp232 = _tmp189 * _tmp80;
  const Scalar _tmp233 = _tmp226 * _tmp64;
  const Scalar _tmp234 = (_tmp217 * _tmp81 - _tmp229 * _tmp81 + _tmp230 * _tmp65 + _tmp231 -
                          _tmp232 * _tmp65 - _tmp233) /
                         std::pow(_tmp85, Scalar(2));
  const Scalar _tmp235 = _tmp234 * _tmp83;
  const Scalar _tmp236 = _tmp165 * _tmp173;
  const Scalar _tmp237 = _tmp162 * _tmp175;
  const Scalar _tmp238 = _tmp166 * _tmp169;
  const Scalar _tmp239 = _tmp161 * _tmp176;
  const Scalar _tmp240 = Scalar(0.043999999999999997) * _tmp161 * _tmp22;
  const Scalar _tmp241 = Scalar(0.043999999999999997) * _tmp198;
  const Scalar _tmp242 = _tmp240 + _tmp241;
  const Scalar _tmp243 = _tmp161 * _tmp173;
  const Scalar _tmp244 = _tmp166 * _tmp175;
  const Scalar _tmp245 = _tmp162 * _tmp169;
  const Scalar _tmp246 = _tmp165 * _tmp176;
  const Scalar _tmp247 = -_tmp243 - _tmp244 + _tmp245 + _tmp246;
  const Scalar _tmp248 = _tmp236 + _tmp237 + _tmp238 + _tmp239 + _tmp242 + _tmp247;
  const Scalar _tmp249 = _tmp248 * _tmp82;
  const Scalar _tmp250 = -_tmp236;
  const Scalar _tmp251 = -_tmp237;
  const Scalar _tmp252 = -_tmp238;
  const Scalar _tmp253 = -_tmp239;
  const Scalar _tmp254 = _tmp242 + _tmp250 + _tmp251 + _tmp252 + _tmp253;
  const Scalar _tmp255 = _tmp247 + _tmp254;
  const Scalar _tmp256 = _tmp248 * _tmp81;
  const Scalar _tmp257 = -_tmp129 * _tmp217 + _tmp129 * _tmp229 - _tmp230 * _tmp67 -
                         _tmp231 * _tmp60 + _tmp232 * _tmp67 + _tmp233 * _tmp60 + _tmp255 * _tmp84 -
                         _tmp256 * _tmp65;
  const Scalar _tmp258 = _tmp224 * _tmp71;
  const Scalar _tmp259 = _tmp243 + _tmp244 - _tmp245 - _tmp246;
  const Scalar _tmp260 = _tmp254 + _tmp259;
  const Scalar _tmp261 = _tmp222 * _tmp76;
  const Scalar _tmp262 =
      _tmp220 * _tmp81 - _tmp225 * _tmp81 + _tmp230 * _tmp82 - _tmp232 * _tmp82 + _tmp258 - _tmp261;
  const Scalar _tmp263 =
      _tmp40 * (Scalar(0.83999999999999997) * _tmp12 * _tmp161 + _tmp165 * _tmp180 -
                Scalar(0.83999999999999997) * _tmp17 * _tmp175 - _tmp170 * _tmp26);
  const Scalar _tmp264 = _tmp86 * (-_tmp217 * _tmp60 + _tmp217 * _tmp66 + _tmp229 * _tmp60 -
                                   _tmp229 * _tmp66 + _tmp248 * _tmp65 - _tmp255 * _tmp65);
  const Scalar _tmp265 =
      _tmp52 *
      (Scalar(1.6799999999999999) * _tmp169 * _tmp17 + Scalar(1.6799999999999999) * _tmp198) /
      std::pow(_tmp39, Scalar(2));
  const Scalar _tmp266 =
      -_tmp131 * _tmp262 + _tmp220 * _tmp66 - _tmp220 * _tmp91 - _tmp225 * _tmp66 +
      _tmp225 * _tmp91 + _tmp235 * _tmp68 + _tmp249 - _tmp260 * _tmp82 - _tmp263 * _tmp92 -
      _tmp264 * _tmp83 + _tmp265 * _tmp92 -
      _tmp53 * (-_tmp129 * _tmp220 + _tmp129 * _tmp225 - _tmp230 * _tmp88 + _tmp232 * _tmp88 +
                _tmp235 * _tmp89 - _tmp249 * _tmp81 - _tmp257 * _tmp87 - _tmp258 * _tmp91 +
                _tmp260 * _tmp77 + _tmp261 * _tmp91 - _tmp262 * _tmp90);
  const Scalar _tmp267 = _tmp109 * _tmp212;
  const Scalar _tmp268 = _tmp109 * _tmp190;
  const Scalar _tmp269 = _tmp106 * _tmp210;
  const Scalar _tmp270 = _tmp211 / (_tmp102 * std::sqrt(_tmp102));
  const Scalar _tmp271 =
      _tmp104 * (_tmp107 * _tmp188 - _tmp108 * _tmp188 + _tmp269 * _tmp50 - _tmp269 * _tmp78 +
                 _tmp270 * _tmp35 * _tmp78 - _tmp270 * _tmp50 * _tmp80);
  const Scalar _tmp272 = _tmp110 * _tmp217 - _tmp110 * _tmp229 - _tmp216 * _tmp84 +
                         _tmp217 * _tmp38 - _tmp229 * _tmp38 + _tmp231 * _tmp49 - _tmp233 * _tmp49 +
                         _tmp233 * _tmp61 + _tmp267 * _tmp65 - _tmp268 * _tmp65 + _tmp271 * _tmp65;
  const Scalar _tmp273 = _tmp110 * _tmp220 - _tmp110 * _tmp225 + _tmp111 * _tmp235 -
                         _tmp112 * _tmp262 - _tmp219 * _tmp77 + _tmp220 * _tmp70 -
                         _tmp225 * _tmp70 + _tmp258 * _tmp73 - _tmp261 * _tmp73 + _tmp261 * _tmp74 +
                         _tmp267 * _tmp82 - _tmp268 * _tmp82 + _tmp271 * _tmp82 - _tmp272 * _tmp87;
  const Scalar _tmp274 = _tmp273 / std::pow(_tmp113, Scalar(2));
  const Scalar _tmp275 = _tmp274 * _tmp93;
  const Scalar _tmp276 = _tmp266 / std::pow(_tmp93, Scalar(2));
  const Scalar _tmp277 = _tmp113 * _tmp276;
  const Scalar _tmp278 = _tmp234 * _tmp81;
  const Scalar _tmp279 = _tmp257 * _tmp86;
  const Scalar _tmp280 =
      -_tmp130 * _tmp263 + _tmp130 * _tmp265 + _tmp131 * _tmp230 - _tmp131 * _tmp232 - _tmp240 -
      _tmp241 + _tmp250 + _tmp251 + _tmp252 + _tmp253 + _tmp259 + _tmp264 * _tmp81 -
      _tmp278 * _tmp68 -
      _tmp53 * (_tmp230 * _tmp66 + _tmp230 * _tmp90 - _tmp232 * _tmp66 - _tmp232 * _tmp90 +
                _tmp256 - _tmp278 * _tmp89 + _tmp279 * _tmp81);
  const Scalar _tmp281 = _tmp272 * _tmp86;
  const Scalar _tmp282 = _tmp273 * _tmp94;
  const Scalar _tmp283 = _tmp114 * _tmp134 * _tmp266 - _tmp134 * _tmp275 +
                         _tmp135 * (-_tmp111 * _tmp278 + _tmp112 * _tmp230 - _tmp112 * _tmp232 +
                                    _tmp132 * _tmp277 - _tmp132 * _tmp282 - _tmp133 * _tmp280 -
                                    _tmp267 + _tmp268 - _tmp271 + _tmp281 * _tmp81) +
                         _tmp280;
  const Scalar _tmp284 = _tmp225 * _tmp94;
  const Scalar _tmp285 = _tmp220 * _tmp94;
  const Scalar _tmp286 = _tmp276 * _tmp82;
  const Scalar _tmp287 = _tmp234 * _tmp65;
  const Scalar _tmp288 = _tmp262 * _tmp94;
  const Scalar _tmp289 = _tmp276 * _tmp83;
  const Scalar _tmp290 =
      -_tmp136 * _tmp288 + _tmp136 * _tmp289 - _tmp230 + _tmp232 - _tmp283 * _tmp97;
  const Scalar _tmp291 = _tmp229 * _tmp86;
  const Scalar _tmp292 = _tmp100 * _tmp99;
  const Scalar _tmp293 = _tmp147 * _tmp149;
  const Scalar _tmp294 = Scalar(1.0) * _tmp274;
  const Scalar _tmp295 = _tmp65 * _tmp87;
  const Scalar _tmp296 = Scalar(0.5) * _tmp114;
  const Scalar _tmp297 = _tmp123 * _tmp202;
  const Scalar _tmp298 = Scalar(1.0) * _tmp90;
  const Scalar _tmp299 = Scalar(1.0) * _tmp234;
  const Scalar _tmp300 = _tmp141 * _tmp279 + _tmp263 * _tmp298 - Scalar(1.0) * _tmp264 -
                         _tmp265 * _tmp298 - _tmp299 * _tmp53 * _tmp89 + _tmp299 * _tmp68;
  const Scalar _tmp301 = _tmp135 * (_tmp111 * _tmp299 - _tmp133 * _tmp300 + _tmp142 * _tmp277 -
                                    _tmp142 * _tmp282 - Scalar(1.0) * _tmp281) -
                         _tmp143 * _tmp275 + _tmp144 * _tmp266 + _tmp300;
  const Scalar _tmp302 = -_tmp145 * _tmp288 + _tmp145 * _tmp289 - _tmp301 * _tmp97;
  const Scalar _tmp303 = _tmp115 * fh1;
  const Scalar _tmp304 = _tmp126 * _tmp303;
  const Scalar _tmp305 = _tmp100 * _tmp55;
  const Scalar _tmp306 = _tmp305 * _tmp97;
  const Scalar _tmp307 = _tmp86 * fh1;
  const Scalar _tmp308 = _tmp146 * _tmp307;
  const Scalar _tmp309 = _tmp139 * _tmp86;
  const Scalar _tmp310 = _tmp305 * _tmp94;
  const Scalar _tmp311 = _tmp262 * _tmp86;
  const Scalar _tmp312 = _tmp127 * _tmp294;
  const Scalar _tmp313 = _tmp137 * _tmp307;
  const Scalar _tmp314 = _tmp206 * _tmp303;
  const Scalar _tmp315 = _tmp276 * _tmp305;
  const Scalar _tmp316 = _tmp94 * fh1;
  const Scalar _tmp317 = _tmp148 * _tmp316;
  const Scalar _tmp318 = _tmp139 * _tmp94;
  const Scalar _tmp319 = _tmp136 * _tmp316;
  const Scalar _tmp320 = _tmp145 * _tmp316;

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) = 0;
  _res(1, 0) =
      -(-_tmp105 *
            (-_tmp218 * _tmp98 + _tmp276 * _tmp295 * _tmp55 - _tmp284 * _tmp55 + _tmp285 * _tmp55 -
             _tmp286 * _tmp55 + _tmp287 * _tmp98 - _tmp288 * _tmp55 * _tmp96 + _tmp291 * _tmp98) -
        _tmp117 * _tmp149 * _tmp206 -
        _tmp128 * (_tmp115 * _tmp220 + _tmp115 * _tmp235 * _tmp65 - _tmp115 * _tmp262 * _tmp96 -
                   _tmp116 * _tmp217 - _tmp223 * _tmp296 * _tmp74 +
                   _tmp227 * _tmp296 * _tmp61 * _tmp87 + _tmp294 * _tmp295 - _tmp294 * _tmp82) -
        _tmp140 * (-_tmp136 * _tmp284 + _tmp136 * _tmp285 - _tmp136 * _tmp286 + _tmp137 * _tmp218 -
                   _tmp137 * _tmp287 - _tmp137 * _tmp291 + _tmp283 * _tmp95 + _tmp290 * _tmp96) -
        _tmp150 * (-_tmp145 * _tmp284 + _tmp145 * _tmp285 - _tmp145 * _tmp286 + _tmp146 * _tmp218 -
                   _tmp146 * _tmp287 - _tmp146 * _tmp291 + _tmp301 * _tmp95 + _tmp302 * _tmp96) +
        _tmp190 * _tmp192 + _tmp190 * _tmp207 + _tmp190 * _tmp213 + _tmp190 * _tmp292 -
        _tmp192 * _tmp212 - _tmp196 * _tmp214 + _tmp204 * _tmp214 + _tmp205 * _tmp293 -
        _tmp207 * _tmp212 - _tmp212 * _tmp213 - _tmp212 * _tmp292 - _tmp293 * _tmp297) *
      std::exp(_tmp105 * _tmp99 + _tmp117 * _tmp128 + _tmp138 * _tmp140 + _tmp147 * _tmp150);
  _res(2, 0) =
      -(-_tmp137 * _tmp139 * _tmp234 - _tmp146 * _tmp191 * _tmp234 + _tmp148 * _tmp302 * _tmp307 +
        _tmp196 * _tmp313 - _tmp204 * _tmp313 - _tmp205 * _tmp308 + _tmp234 * _tmp306 +
        _tmp235 * _tmp304 + _tmp290 * _tmp309 + _tmp297 * _tmp308 - _tmp304 * _tmp311 -
        _tmp310 * _tmp311 + _tmp312 * _tmp87 - _tmp314 * _tmp87 + _tmp315 * _tmp87) *
      std::exp(-_tmp137 * _tmp309 - _tmp148 * _tmp308 + _tmp304 * _tmp87 + _tmp306 * _tmp86);
  _res(3, 0) = -(-_tmp136 * _tmp139 * _tmp276 - _tmp145 * _tmp191 * _tmp276 + _tmp196 * _tmp319 -
                 _tmp204 * _tmp319 - _tmp205 * _tmp320 + _tmp283 * _tmp318 + _tmp297 * _tmp320 +
                 _tmp301 * _tmp317 - _tmp312 + _tmp314 - _tmp315) *
               std::exp(-_tmp136 * _tmp318 - _tmp145 * _tmp317 - _tmp304 - _tmp310);

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
