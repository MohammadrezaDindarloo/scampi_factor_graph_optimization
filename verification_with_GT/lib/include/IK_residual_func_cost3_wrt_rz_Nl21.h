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
 * Symbolic function: IK_residual_func_cost3_wrt_rz_Nl21
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost3WrtRzNl21(
    const Scalar fh1, const Scalar fv1, const Scalar rx, const Scalar ry, const Scalar rz,
    const Scalar p_init0, const Scalar p_init1, const Scalar p_init2, const Scalar rot_init_x,
    const Scalar rot_init_y, const Scalar rot_init_z, const Scalar rot_init_w,
    const Scalar epsilon) {
  // Total ops: 982

  // Unused inputs
  (void)p_init2;
  (void)epsilon;

  // Input arrays

  // Intermediate terms (318)
  const Scalar _tmp0 = std::pow(rz, Scalar(2));
  const Scalar _tmp1 = _tmp0 + std::pow(rx, Scalar(2)) + std::pow(ry, Scalar(2));
  const Scalar _tmp2 = std::sqrt(_tmp1);
  const Scalar _tmp3 = (Scalar(1) / Scalar(2)) * _tmp2;
  const Scalar _tmp4 = std::cos(_tmp3);
  const Scalar _tmp5 = _tmp4 * rot_init_z;
  const Scalar _tmp6 = std::sin(_tmp3);
  const Scalar _tmp7 = _tmp6 / _tmp2;
  const Scalar _tmp8 = _tmp7 * rot_init_x;
  const Scalar _tmp9 = _tmp7 * rot_init_y;
  const Scalar _tmp10 = _tmp7 * rot_init_w;
  const Scalar _tmp11 = _tmp10 * rz;
  const Scalar _tmp12 = _tmp11 + _tmp5 + _tmp8 * ry - _tmp9 * rx;
  const Scalar _tmp13 = -2 * std::pow(_tmp12, Scalar(2));
  const Scalar _tmp14 = _tmp4 * rot_init_y;
  const Scalar _tmp15 = _tmp7 * rot_init_z;
  const Scalar _tmp16 = _tmp8 * rz;
  const Scalar _tmp17 = _tmp10 * ry + _tmp14 + _tmp15 * rx - _tmp16;
  const Scalar _tmp18 = 1 - 2 * std::pow(_tmp17, Scalar(2));
  const Scalar _tmp19 = Scalar(0.20999999999999999) * _tmp13 + Scalar(0.20999999999999999) * _tmp18;
  const Scalar _tmp20 = _tmp4 * rot_init_x;
  const Scalar _tmp21 = _tmp9 * rz;
  const Scalar _tmp22 = _tmp10 * rx - _tmp15 * ry + _tmp20 + _tmp21;
  const Scalar _tmp23 = 2 * _tmp12;
  const Scalar _tmp24 = _tmp22 * _tmp23;
  const Scalar _tmp25 = _tmp4 * rot_init_w;
  const Scalar _tmp26 = _tmp15 * rz;
  const Scalar _tmp27 = _tmp25 - _tmp26 - _tmp8 * rx - _tmp9 * ry;
  const Scalar _tmp28 = 2 * _tmp27;
  const Scalar _tmp29 = _tmp17 * _tmp28;
  const Scalar _tmp30 =
      -Scalar(0.010999999999999999) * _tmp24 - Scalar(0.010999999999999999) * _tmp29;
  const Scalar _tmp31 = 2 * _tmp17 * _tmp22;
  const Scalar _tmp32 = _tmp23 * _tmp27;
  const Scalar _tmp33 = Scalar(0.20999999999999999) * _tmp31 - Scalar(0.20999999999999999) * _tmp32;
  const Scalar _tmp34 = _tmp30 - _tmp33;
  const Scalar _tmp35 = _tmp19 + _tmp34;
  const Scalar _tmp36 = _tmp35 + p_init0 + Scalar(-2.5202214700000001);
  const Scalar _tmp37 = Scalar(1.0) / (_tmp36);
  const Scalar _tmp38 = -2 * std::pow(_tmp22, Scalar(2));
  const Scalar _tmp39 = Scalar(0.20999999999999999) * _tmp13 +
                        Scalar(0.20999999999999999) * _tmp38 + Scalar(0.20999999999999999);
  const Scalar _tmp40 = -_tmp39;
  const Scalar _tmp41 = _tmp17 * _tmp23;
  const Scalar _tmp42 = _tmp22 * _tmp28;
  const Scalar _tmp43 =
      -Scalar(0.010999999999999999) * _tmp41 + Scalar(0.010999999999999999) * _tmp42;
  const Scalar _tmp44 = Scalar(0.20999999999999999) * _tmp31 + Scalar(0.20999999999999999) * _tmp32;
  const Scalar _tmp45 = _tmp43 + _tmp44;
  const Scalar _tmp46 = _tmp40 + _tmp45;
  const Scalar _tmp47 = _tmp46 + p_init1 + Scalar(8.3888750099999996);
  const Scalar _tmp48 = _tmp37 * _tmp47;
  const Scalar _tmp49 = _tmp43 - _tmp44;
  const Scalar _tmp50 = _tmp40 + _tmp49;
  const Scalar _tmp51 = _tmp50 + p_init1 + Scalar(8.3196563700000006);
  const Scalar _tmp52 = -_tmp19;
  const Scalar _tmp53 = _tmp34 + _tmp52;
  const Scalar _tmp54 = _tmp53 + p_init0 + Scalar(1.9874742000000001);
  const Scalar _tmp55 = std::pow(_tmp51, Scalar(2)) + std::pow(_tmp54, Scalar(2));
  const Scalar _tmp56 = std::pow(_tmp55, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp57 = _tmp54 * _tmp56;
  const Scalar _tmp58 = _tmp51 * _tmp56;
  const Scalar _tmp59 = _tmp48 * _tmp57 - _tmp58;
  const Scalar _tmp60 = _tmp39 + _tmp45;
  const Scalar _tmp61 = _tmp60 + p_init1 + Scalar(-4.7752063900000001);
  const Scalar _tmp62 = _tmp30 + _tmp33;
  const Scalar _tmp63 = _tmp19 + _tmp62;
  const Scalar _tmp64 = _tmp63 + p_init0 + Scalar(-2.71799795);
  const Scalar _tmp65 = std::pow(_tmp61, Scalar(2)) + std::pow(_tmp64, Scalar(2));
  const Scalar _tmp66 = std::pow(_tmp65, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp67 = _tmp64 * _tmp66;
  const Scalar _tmp68 = _tmp61 * _tmp66;
  const Scalar _tmp69 = _tmp48 * _tmp67 - _tmp68;
  const Scalar _tmp70 = Scalar(1.0) / (_tmp69);
  const Scalar _tmp71 = _tmp59 * _tmp70;
  const Scalar _tmp72 = std::pow(_tmp36, Scalar(2));
  const Scalar _tmp73 = std::pow(_tmp47, Scalar(2)) + _tmp72;
  const Scalar _tmp74 = std::sqrt(_tmp73);
  const Scalar _tmp75 = Scalar(1.0) / (_tmp74);
  const Scalar _tmp76 = _tmp36 * _tmp75;
  const Scalar _tmp77 = _tmp35 * _tmp75;
  const Scalar _tmp78 = -_tmp46 * _tmp76 + _tmp47 * _tmp77;
  const Scalar _tmp79 = _tmp37 * _tmp74;
  const Scalar _tmp80 = _tmp78 * _tmp79;
  const Scalar _tmp81 = _tmp60 * _tmp67 - _tmp63 * _tmp68 + _tmp67 * _tmp80;
  const Scalar _tmp82 = _tmp50 * _tmp57 - _tmp53 * _tmp58 + _tmp57 * _tmp80 - _tmp71 * _tmp81;
  const Scalar _tmp83 = Scalar(1.0) / (_tmp82);
  const Scalar _tmp84 = Scalar(1.0) * _tmp83;
  const Scalar _tmp85 = _tmp67 * _tmp84;
  const Scalar _tmp86 = _tmp57 * _tmp84 - _tmp71 * _tmp85;
  const Scalar _tmp87 = _tmp39 + _tmp49;
  const Scalar _tmp88 = _tmp87 + p_init1 + Scalar(-4.8333311099999996);
  const Scalar _tmp89 = _tmp52 + _tmp62;
  const Scalar _tmp90 = _tmp89 + p_init0 + Scalar(1.79662371);
  const Scalar _tmp91 = std::pow(_tmp88, Scalar(2)) + std::pow(_tmp90, Scalar(2));
  const Scalar _tmp92 = std::pow(_tmp91, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp93 = _tmp89 * _tmp92;
  const Scalar _tmp94 = _tmp87 * _tmp92;
  const Scalar _tmp95 = fh1 * (_tmp88 * _tmp93 - _tmp90 * _tmp94);
  const Scalar _tmp96 = _tmp79 * _tmp95;
  const Scalar _tmp97 = Scalar(0.20999999999999999) * _tmp41 + Scalar(0.20999999999999999) * _tmp42;
  const Scalar _tmp98 = -_tmp97;
  const Scalar _tmp99 =
      -Scalar(0.010999999999999999) * _tmp18 - Scalar(0.010999999999999999) * _tmp38;
  const Scalar _tmp100 =
      Scalar(0.20999999999999999) * _tmp24 - Scalar(0.20999999999999999) * _tmp29;
  const Scalar _tmp101 = _tmp100 + _tmp99;
  const Scalar _tmp102 = _tmp101 + _tmp98;
  const Scalar _tmp103 = _tmp102 * _tmp67;
  const Scalar _tmp104 = _tmp101 + _tmp97;
  const Scalar _tmp105 = _tmp103 - _tmp104 * _tmp67;
  const Scalar _tmp106 = -_tmp100 + _tmp98 + _tmp99;
  const Scalar _tmp107 = _tmp102 * _tmp57;
  const Scalar _tmp108 = -_tmp103 * _tmp48 + _tmp104 * _tmp68;
  const Scalar _tmp109 = _tmp102 * _tmp48;
  const Scalar _tmp110 = _tmp106 * _tmp58 - _tmp108 * _tmp71 - _tmp109 * _tmp57;
  const Scalar _tmp111 = Scalar(1.0) * _tmp46;
  const Scalar _tmp112 = -_tmp111 + _tmp60;
  const Scalar _tmp113 = Scalar(1.0) / (_tmp112);
  const Scalar _tmp114 = Scalar(1.0) * _tmp35;
  const Scalar _tmp115 = _tmp114 - _tmp63;
  const Scalar _tmp116 = _tmp113 * _tmp115;
  const Scalar _tmp117 = -_tmp105 * _tmp71 - _tmp106 * _tmp57 + _tmp107 - _tmp110 * _tmp116;
  const Scalar _tmp118 = Scalar(1.0) / (_tmp117);
  const Scalar _tmp119 = _tmp48 * _tmp70;
  const Scalar _tmp120 = _tmp108 * _tmp119 + _tmp109;
  const Scalar _tmp121 = _tmp113 * _tmp120;
  const Scalar _tmp122 = -_tmp102 + _tmp105 * _tmp119 - _tmp115 * _tmp121;
  const Scalar _tmp123 = _tmp118 * _tmp82;
  const Scalar _tmp124 = _tmp119 * _tmp81 - _tmp122 * _tmp123 - _tmp80;
  const Scalar _tmp125 = _tmp117 * _tmp83;
  const Scalar _tmp126 = _tmp122 + _tmp124 * _tmp125;
  const Scalar _tmp127 = _tmp118 * _tmp126;
  const Scalar _tmp128 = _tmp118 * _tmp59;
  const Scalar _tmp129 = -_tmp126 * _tmp128 - _tmp48;
  const Scalar _tmp130 = _tmp67 * _tmp70;
  const Scalar _tmp131 = _tmp127 * _tmp57 + _tmp129 * _tmp130 + Scalar(1.0);
  const Scalar _tmp132 = _tmp92 * fh1;
  const Scalar _tmp133 = _tmp132 * _tmp90;
  const Scalar _tmp134 = _tmp133 * _tmp79;
  const Scalar _tmp135 = _tmp111 * _tmp116 + _tmp114;
  const Scalar _tmp136 = 0;
  const Scalar _tmp137 = _tmp118 * _tmp136;
  const Scalar _tmp138 = _tmp137 * _tmp67;
  const Scalar _tmp139 = _tmp137 * _tmp57 - _tmp138 * _tmp71;
  const Scalar _tmp140 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp141 = _tmp140 * _tmp79;
  const Scalar _tmp142 = Scalar(1.0) * _tmp70;
  const Scalar _tmp143 = _tmp116 * _tmp142;
  const Scalar _tmp144 = -_tmp105 * _tmp142 + _tmp108 * _tmp143;
  const Scalar _tmp145 = -_tmp123 * _tmp144 - _tmp142 * _tmp81;
  const Scalar _tmp146 = _tmp125 * _tmp145 + _tmp144;
  const Scalar _tmp147 = _tmp118 * _tmp146;
  const Scalar _tmp148 = -_tmp147 * _tmp59 + Scalar(1.0);
  const Scalar _tmp149 = _tmp130 * _tmp148 + _tmp147 * _tmp57;
  const Scalar _tmp150 = _tmp88 * _tmp92;
  const Scalar _tmp151 = _tmp150 * fh1;
  const Scalar _tmp152 = _tmp151 * _tmp79;
  const Scalar _tmp153 = (Scalar(1) / Scalar(2)) / _tmp1;
  const Scalar _tmp154 = _tmp0 * _tmp153;
  const Scalar _tmp155 = _tmp153 * rz;
  const Scalar _tmp156 = _tmp155 * ry;
  const Scalar _tmp157 = _tmp155 * rx;
  const Scalar _tmp158 = _tmp6 / (_tmp1 * std::sqrt(_tmp1));
  const Scalar _tmp159 = _tmp0 * _tmp158;
  const Scalar _tmp160 = _tmp158 * rz;
  const Scalar _tmp161 = _tmp160 * rx;
  const Scalar _tmp162 = _tmp160 * ry;
  const Scalar _tmp163 = -Scalar(1) / Scalar(2) * _tmp11 - _tmp14 * _tmp156 - _tmp15 -
                         _tmp154 * _tmp5 - _tmp157 * _tmp20 + _tmp159 * rot_init_z +
                         _tmp161 * rot_init_x + _tmp162 * rot_init_y;
  const Scalar _tmp164 = Scalar(0.41999999999999998) * _tmp12;
  const Scalar _tmp165 = _tmp163 * _tmp164;
  const Scalar _tmp166 = _tmp10 - _tmp14 * _tmp157 + _tmp154 * _tmp25 + _tmp156 * _tmp20 -
                         _tmp159 * rot_init_w + _tmp161 * rot_init_y - _tmp162 * rot_init_x -
                         Scalar(1) / Scalar(2) * _tmp26;
  const Scalar _tmp167 = _tmp166 * _tmp27;
  const Scalar _tmp168 = Scalar(0.41999999999999998) * _tmp167;
  const Scalar _tmp169 = _tmp165 + _tmp168;
  const Scalar _tmp170 = _tmp14 * _tmp154 - _tmp156 * _tmp5 + _tmp157 * _tmp25 -
                         _tmp159 * rot_init_y - Scalar(1) / Scalar(2) * _tmp16 -
                         _tmp161 * rot_init_w + _tmp162 * rot_init_z + _tmp9;
  const Scalar _tmp171 = Scalar(0.41999999999999998) * _tmp17;
  const Scalar _tmp172 = _tmp170 * _tmp171;
  const Scalar _tmp173 = -_tmp154 * _tmp20 + _tmp156 * _tmp25 + _tmp157 * _tmp5 +
                         _tmp159 * rot_init_x - _tmp161 * rot_init_z - _tmp162 * rot_init_w -
                         Scalar(1) / Scalar(2) * _tmp21 - _tmp8;
  const Scalar _tmp174 = Scalar(0.41999999999999998) * _tmp22;
  const Scalar _tmp175 = _tmp173 * _tmp174;
  const Scalar _tmp176 = -_tmp172 - _tmp175;
  const Scalar _tmp177 = _tmp12 * _tmp166;
  const Scalar _tmp178 = Scalar(0.83999999999999997) * _tmp177;
  const Scalar _tmp179 = -_tmp178;
  const Scalar _tmp180 = _tmp17 * _tmp173;
  const Scalar _tmp181 = Scalar(0.83999999999999997) * _tmp180;
  const Scalar _tmp182 = Scalar(0.021999999999999999) * _tmp163;
  const Scalar _tmp183 = Scalar(0.021999999999999999) * _tmp170;
  const Scalar _tmp184 = Scalar(0.021999999999999999) * _tmp166;
  const Scalar _tmp185 = Scalar(0.021999999999999999) * _tmp173;
  const Scalar _tmp186 = -_tmp12 * _tmp183 - _tmp17 * _tmp182 - _tmp184 * _tmp22 - _tmp185 * _tmp27;
  const Scalar _tmp187 = _tmp179 - _tmp181 + _tmp186;
  const Scalar _tmp188 = _tmp169 + _tmp176 + _tmp187;
  const Scalar _tmp189 = _tmp170 * _tmp22;
  const Scalar _tmp190 = Scalar(0.83999999999999997) * _tmp189;
  const Scalar _tmp191 = _tmp169 + _tmp178;
  const Scalar _tmp192 = _tmp172 + _tmp175;
  const Scalar _tmp193 = -_tmp12 * _tmp185 - _tmp17 * _tmp184 + _tmp182 * _tmp22 + _tmp183 * _tmp27;
  const Scalar _tmp194 = _tmp192 + _tmp193;
  const Scalar _tmp195 = _tmp190 + _tmp191 + _tmp194;
  const Scalar _tmp196 = _tmp188 * _tmp36 + _tmp195 * _tmp47;
  const Scalar _tmp197 = _tmp196 * _tmp37 * _tmp75;
  const Scalar _tmp198 = _tmp86 * _tmp95;
  const Scalar _tmp199 = -_tmp165 - _tmp168;
  const Scalar _tmp200 = _tmp187 + _tmp192 + _tmp199;
  const Scalar _tmp201 = _tmp200 * _tmp66;
  const Scalar _tmp202 = _tmp201 * _tmp71;
  const Scalar _tmp203 = _tmp195 * _tmp37;
  const Scalar _tmp204 = _tmp178 + _tmp199;
  const Scalar _tmp205 = _tmp176 + _tmp193;
  const Scalar _tmp206 = _tmp190 + _tmp204 + _tmp205;
  const Scalar _tmp207 = _tmp181 + _tmp186;
  const Scalar _tmp208 = _tmp176 + _tmp191 + _tmp207;
  const Scalar _tmp209 =
      (2 * _tmp206 * _tmp51 + 2 * _tmp208 * _tmp54) / (_tmp55 * std::sqrt(_tmp55));
  const Scalar _tmp210 = (Scalar(1) / Scalar(2)) * _tmp209;
  const Scalar _tmp211 = _tmp210 * _tmp51;
  const Scalar _tmp212 = _tmp206 * _tmp56;
  const Scalar _tmp213 = _tmp188 / _tmp72;
  const Scalar _tmp214 = _tmp213 * _tmp47;
  const Scalar _tmp215 = _tmp210 * _tmp54;
  const Scalar _tmp216 = _tmp208 * _tmp56;
  const Scalar _tmp217 =
      _tmp203 * _tmp57 + _tmp211 - _tmp212 - _tmp214 * _tmp57 - _tmp215 * _tmp48 + _tmp216 * _tmp48;
  const Scalar _tmp218 = _tmp217 * _tmp70;
  const Scalar _tmp219 = _tmp163 * _tmp174;
  const Scalar _tmp220 = Scalar(0.043999999999999997) * _tmp189;
  const Scalar _tmp221 = Scalar(0.043999999999999997) * _tmp180;
  const Scalar _tmp222 = _tmp164 * _tmp173;
  const Scalar _tmp223 = _tmp166 * _tmp171;
  const Scalar _tmp224 = Scalar(0.41999999999999998) * _tmp27;
  const Scalar _tmp225 = _tmp170 * _tmp224;
  const Scalar _tmp226 = -_tmp219 + _tmp220 + _tmp221 - _tmp222 - _tmp223 - _tmp225;
  const Scalar _tmp227 = _tmp163 * _tmp171;
  const Scalar _tmp228 = _tmp164 * _tmp170;
  const Scalar _tmp229 = _tmp166 * _tmp174;
  const Scalar _tmp230 = _tmp173 * _tmp224;
  const Scalar _tmp231 = -_tmp227 + _tmp228 + _tmp229 - _tmp230;
  const Scalar _tmp232 = _tmp226 + _tmp231;
  const Scalar _tmp233 = _tmp232 * _tmp57;
  const Scalar _tmp234 = _tmp227 - _tmp228 - _tmp229 + _tmp230;
  const Scalar _tmp235 = _tmp226 + _tmp234;
  const Scalar _tmp236 = _tmp179 - _tmp190;
  const Scalar _tmp237 = _tmp169 + _tmp194 + _tmp236;
  const Scalar _tmp238 =
      (2 * _tmp200 * _tmp64 + 2 * _tmp237 * _tmp61) / (_tmp65 * std::sqrt(_tmp65));
  const Scalar _tmp239 = (Scalar(1) / Scalar(2)) * _tmp238;
  const Scalar _tmp240 = _tmp239 * _tmp61;
  const Scalar _tmp241 = _tmp237 * _tmp66;
  const Scalar _tmp242 = _tmp239 * _tmp64;
  const Scalar _tmp243 = (_tmp201 * _tmp48 + _tmp203 * _tmp67 - _tmp214 * _tmp67 + _tmp240 -
                          _tmp241 - _tmp242 * _tmp48) /
                         std::pow(_tmp69, Scalar(2));
  const Scalar _tmp244 = _tmp243 * _tmp59;
  const Scalar _tmp245 = _tmp232 * _tmp67;
  const Scalar _tmp246 = _tmp219 + _tmp222 + _tmp223 + _tmp225;
  const Scalar _tmp247 = _tmp220 + _tmp221 + _tmp231 + _tmp246;
  const Scalar _tmp248 = _tmp102 * _tmp201;
  const Scalar _tmp249 = -_tmp102 * _tmp242 - _tmp104 * _tmp201 + _tmp104 * _tmp242 + _tmp245 -
                         _tmp247 * _tmp67 + _tmp248;
  const Scalar _tmp250 = _tmp102 * _tmp203;
  const Scalar _tmp251 = _tmp103 * _tmp214 - _tmp104 * _tmp240 + _tmp104 * _tmp241 +
                         _tmp109 * _tmp242 - _tmp245 * _tmp48 + _tmp247 * _tmp68 -
                         _tmp248 * _tmp48 - _tmp250 * _tmp67;
  const Scalar _tmp252 = _tmp102 * _tmp216;
  const Scalar _tmp253 =
      _tmp115 * (-Scalar(1.6799999999999999) * _tmp177 - Scalar(1.6799999999999999) * _tmp189) /
      std::pow(_tmp112, Scalar(2));
  const Scalar _tmp254 = Scalar(0.83999999999999997) * _tmp12 * _tmp163 +
                         Scalar(0.83999999999999997) * _tmp167 -
                         Scalar(0.83999999999999997) * _tmp17 * _tmp170 -
                         Scalar(0.83999999999999997) * _tmp173 * _tmp22;
  const Scalar _tmp255 = _tmp113 * _tmp254;
  const Scalar _tmp256 =
      -_tmp102 * _tmp215 - _tmp105 * _tmp218 + _tmp105 * _tmp244 + _tmp106 * _tmp215 -
      _tmp106 * _tmp216 + _tmp110 * _tmp253 - _tmp110 * _tmp255 -
      _tmp116 * (-_tmp106 * _tmp211 + _tmp106 * _tmp212 + _tmp107 * _tmp214 - _tmp108 * _tmp218 +
                 _tmp108 * _tmp244 + _tmp109 * _tmp215 - _tmp233 * _tmp48 + _tmp235 * _tmp58 -
                 _tmp250 * _tmp57 - _tmp251 * _tmp71 - _tmp252 * _tmp48) +
      _tmp233 - _tmp235 * _tmp57 - _tmp249 * _tmp71 + _tmp252;
  const Scalar _tmp257 = _tmp256 / std::pow(_tmp117, Scalar(2));
  const Scalar _tmp258 = _tmp136 * _tmp257;
  const Scalar _tmp259 = _tmp243 * _tmp67;
  const Scalar _tmp260 = _tmp128 * _tmp136;
  const Scalar _tmp261 = _tmp67 * _tmp71;
  const Scalar _tmp262 = _tmp149 * _tmp79;
  const Scalar _tmp263 = _tmp192 + _tmp204 + _tmp207;
  const Scalar _tmp264 = _tmp199 + _tmp205 + _tmp236;
  const Scalar _tmp265 = (Scalar(1) / Scalar(2)) * (2 * _tmp263 * _tmp90 + 2 * _tmp264 * _tmp88) /
                         (_tmp91 * std::sqrt(_tmp91));
  const Scalar _tmp266 = _tmp265 * _tmp88;
  const Scalar _tmp267 = _tmp266 * fh1;
  const Scalar _tmp268 = _tmp132 * _tmp263;
  const Scalar _tmp269 = _tmp131 * _tmp79;
  const Scalar _tmp270 = _tmp149 * _tmp151;
  const Scalar _tmp271 = _tmp213 * _tmp74;
  const Scalar _tmp272 = _tmp271 * _tmp78;
  const Scalar _tmp273 = _tmp196 / (_tmp73 * std::sqrt(_tmp73));
  const Scalar _tmp274 = _tmp188 * _tmp75;
  const Scalar _tmp275 =
      _tmp79 * (-_tmp195 * _tmp76 + _tmp195 * _tmp77 - _tmp273 * _tmp35 * _tmp47 +
                _tmp273 * _tmp36 * _tmp46 - _tmp274 * _tmp46 + _tmp274 * _tmp47);
  const Scalar _tmp276 = _tmp197 * _tmp78;
  const Scalar _tmp277 = -_tmp200 * _tmp68 + _tmp201 * _tmp60 + _tmp201 * _tmp80 +
                         _tmp240 * _tmp63 - _tmp241 * _tmp63 + _tmp241 * _tmp64 - _tmp242 * _tmp60 -
                         _tmp242 * _tmp80 - _tmp272 * _tmp67 + _tmp275 * _tmp67 + _tmp276 * _tmp67;
  const Scalar _tmp278 = -_tmp208 * _tmp58 + _tmp211 * _tmp53 - _tmp212 * _tmp53 +
                         _tmp212 * _tmp54 - _tmp215 * _tmp50 - _tmp215 * _tmp80 + _tmp216 * _tmp50 +
                         _tmp216 * _tmp80 - _tmp218 * _tmp81 + _tmp244 * _tmp81 - _tmp272 * _tmp57 +
                         _tmp275 * _tmp57 + _tmp276 * _tmp57 - _tmp277 * _tmp71;
  const Scalar _tmp279 = _tmp278 / std::pow(_tmp82, Scalar(2));
  const Scalar _tmp280 = Scalar(1.0) * _tmp279;
  const Scalar _tmp281 = Scalar(0.5) * _tmp83;
  const Scalar _tmp282 = _tmp131 * _tmp133;
  const Scalar _tmp283 = _tmp265 * _tmp90;
  const Scalar _tmp284 = _tmp264 * _tmp92;
  const Scalar _tmp285 = fh1 * (_tmp150 * _tmp263 - _tmp263 * _tmp94 + _tmp264 * _tmp93 -
                                _tmp266 * _tmp89 + _tmp283 * _tmp87 - _tmp284 * _tmp90);
  const Scalar _tmp286 = _tmp139 * _tmp140;
  const Scalar _tmp287 = _tmp283 * fh1;
  const Scalar _tmp288 = _tmp284 * fh1;
  const Scalar _tmp289 = _tmp256 * _tmp83;
  const Scalar _tmp290 = _tmp118 * _tmp278;
  const Scalar _tmp291 = _tmp257 * _tmp82;
  const Scalar _tmp292 = _tmp108 * _tmp142;
  const Scalar _tmp293 = Scalar(1.0) * _tmp243;
  const Scalar _tmp294 = _tmp105 * _tmp293 - _tmp108 * _tmp116 * _tmp293 - _tmp142 * _tmp249 +
                         _tmp143 * _tmp251 - _tmp253 * _tmp292 + _tmp255 * _tmp292;
  const Scalar _tmp295 = _tmp117 * _tmp279;
  const Scalar _tmp296 = _tmp125 * (-_tmp123 * _tmp294 - _tmp142 * _tmp277 - _tmp144 * _tmp290 +
                                    _tmp144 * _tmp291 + _tmp293 * _tmp81) +
                         _tmp145 * _tmp289 - _tmp145 * _tmp295 + _tmp294;
  const Scalar _tmp297 = _tmp118 * _tmp57;
  const Scalar _tmp298 = _tmp257 * _tmp59;
  const Scalar _tmp299 = -_tmp128 * _tmp296 + _tmp146 * _tmp298 - _tmp147 * _tmp217;
  const Scalar _tmp300 = _tmp148 * _tmp70;
  const Scalar _tmp301 = _tmp257 * _tmp57;
  const Scalar _tmp302 = _tmp201 * _tmp70;
  const Scalar _tmp303 = _tmp243 * _tmp48;
  const Scalar _tmp304 = _tmp214 * _tmp70;
  const Scalar _tmp305 = _tmp203 * _tmp70;
  const Scalar _tmp306 =
      -_tmp105 * _tmp303 - _tmp105 * _tmp304 + _tmp105 * _tmp305 -
      _tmp116 * (-_tmp102 * _tmp214 - _tmp108 * _tmp303 - _tmp108 * _tmp304 + _tmp108 * _tmp305 +
                 _tmp119 * _tmp251 + _tmp232 * _tmp48 + _tmp250) +
      _tmp119 * _tmp249 + _tmp120 * _tmp253 - _tmp121 * _tmp254 - _tmp220 - _tmp221 + _tmp234 +
      _tmp246;
  const Scalar _tmp307 = _tmp124 * _tmp289 - _tmp124 * _tmp295 +
                         _tmp125 * (_tmp119 * _tmp277 - _tmp122 * _tmp290 + _tmp122 * _tmp291 -
                                    _tmp123 * _tmp306 + _tmp272 - _tmp275 - _tmp276 -
                                    _tmp303 * _tmp81 - _tmp304 * _tmp81 + _tmp305 * _tmp81) +
                         _tmp306;
  const Scalar _tmp308 =
      _tmp126 * _tmp298 - _tmp127 * _tmp217 - _tmp128 * _tmp307 - _tmp203 + _tmp214;
  const Scalar _tmp309 = _tmp129 * _tmp70;
  const Scalar _tmp310 = _tmp133 * _tmp70;
  const Scalar _tmp311 = _tmp137 * _tmp140;
  const Scalar _tmp312 = _tmp84 * _tmp95;
  const Scalar _tmp313 = _tmp151 * _tmp70;
  const Scalar _tmp314 = _tmp140 * _tmp258;
  const Scalar _tmp315 = _tmp280 * _tmp95;
  const Scalar _tmp316 = _tmp285 * _tmp84;
  const Scalar _tmp317 = _tmp147 * fh1;

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) = 0;
  _res(1, 0) =
      -(-_tmp134 * (-_tmp126 * _tmp301 - _tmp127 * _tmp215 + _tmp127 * _tmp216 - _tmp129 * _tmp259 +
                    _tmp129 * _tmp302 + _tmp130 * _tmp308 - _tmp242 * _tmp309 + _tmp297 * _tmp307) -
        _tmp141 * (-_tmp137 * _tmp202 - _tmp137 * _tmp215 + _tmp137 * _tmp216 +
                   _tmp137 * _tmp242 * _tmp71 - _tmp138 * _tmp218 + _tmp258 * _tmp261 -
                   _tmp258 * _tmp57 + _tmp259 * _tmp260) -
        _tmp152 * (_tmp130 * _tmp299 - _tmp146 * _tmp301 - _tmp147 * _tmp215 + _tmp147 * _tmp216 -
                   _tmp148 * _tmp259 + _tmp148 * _tmp302 - _tmp242 * _tmp300 + _tmp296 * _tmp297) -
        _tmp197 * _tmp198 - _tmp197 * _tmp270 - _tmp197 * _tmp282 - _tmp197 * _tmp286 +
        _tmp198 * _tmp271 + _tmp262 * _tmp267 - _tmp262 * _tmp288 - _tmp268 * _tmp269 +
        _tmp269 * _tmp287 + _tmp270 * _tmp271 + _tmp271 * _tmp282 + _tmp271 * _tmp286 -
        _tmp285 * _tmp79 * _tmp86 -
        _tmp96 * (-_tmp202 * _tmp84 - _tmp209 * _tmp281 * _tmp54 + _tmp216 * _tmp84 -
                  _tmp218 * _tmp85 + _tmp238 * _tmp281 * _tmp64 * _tmp71 + _tmp244 * _tmp85 +
                  _tmp261 * _tmp280 - _tmp280 * _tmp57)) *
      std::exp(_tmp131 * _tmp134 + _tmp139 * _tmp141 + _tmp149 * _tmp152 + _tmp86 * _tmp96);
  _res(2, 0) =
      -(-_tmp129 * _tmp133 * _tmp243 + _tmp140 * _tmp243 * _tmp260 - _tmp148 * _tmp151 * _tmp243 -
        _tmp218 * _tmp311 - _tmp218 * _tmp312 + _tmp244 * _tmp312 - _tmp267 * _tmp300 +
        _tmp268 * _tmp309 - _tmp287 * _tmp309 + _tmp288 * _tmp300 + _tmp299 * _tmp313 +
        _tmp308 * _tmp310 + _tmp314 * _tmp71 + _tmp315 * _tmp71 - _tmp316 * _tmp71) *
      std::exp(-_tmp129 * _tmp310 - _tmp148 * _tmp313 + _tmp311 * _tmp71 + _tmp312 * _tmp71);
  _res(3, 0) =
      -(_tmp118 * _tmp133 * _tmp307 + _tmp118 * _tmp151 * _tmp296 - _tmp126 * _tmp133 * _tmp257 +
        _tmp127 * _tmp268 - _tmp127 * _tmp287 - _tmp146 * _tmp151 * _tmp257 - _tmp266 * _tmp317 +
        _tmp284 * _tmp317 - _tmp314 - _tmp315 + _tmp316) *
      std::exp(-_tmp127 * _tmp133 - _tmp150 * _tmp317 - _tmp311 - _tmp312);

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
