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
 * Symbolic function: IK_residual_func_cost3_wrt_rz_Nl14
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost3WrtRzNl14(
    const Scalar fh1, const Scalar fv1, const Scalar rx, const Scalar ry, const Scalar rz,
    const Scalar p_init0, const Scalar p_init1, const Scalar p_init2, const Scalar rot_init_x,
    const Scalar rot_init_y, const Scalar rot_init_z, const Scalar rot_init_w,
    const Scalar epsilon) {
  // Total ops: 997

  // Unused inputs
  (void)p_init2;
  (void)epsilon;

  // Input arrays

  // Intermediate terms (317)
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
  const Scalar _tmp18 = -2 * std::pow(_tmp17, Scalar(2));
  const Scalar _tmp19 = Scalar(0.20999999999999999) * _tmp13 +
                        Scalar(0.20999999999999999) * _tmp18 + Scalar(0.20999999999999999);
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
  const Scalar _tmp37 = 1 - 2 * std::pow(_tmp22, Scalar(2));
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
  const Scalar _tmp48 = _tmp47 * fh1;
  const Scalar _tmp49 = _tmp36 * _tmp48;
  const Scalar _tmp50 = -_tmp39 + _tmp42;
  const Scalar _tmp51 = _tmp38 + _tmp50;
  const Scalar _tmp52 = _tmp51 + p_init0 + Scalar(-2.5202214700000001);
  const Scalar _tmp53 = Scalar(1.0) / (_tmp52);
  const Scalar _tmp54 = -_tmp19;
  const Scalar _tmp55 = _tmp34 + _tmp54;
  const Scalar _tmp56 = _tmp55 + p_init1 + Scalar(8.3888750099999996);
  const Scalar _tmp57 = _tmp53 * _tmp56;
  const Scalar _tmp58 = _tmp30 - _tmp33;
  const Scalar _tmp59 = _tmp19 + _tmp58;
  const Scalar _tmp60 = _tmp59 + p_init1 + Scalar(-4.8333311099999996);
  const Scalar _tmp61 = -_tmp38;
  const Scalar _tmp62 = _tmp43 + _tmp61;
  const Scalar _tmp63 = _tmp62 + p_init0 + Scalar(1.79662371);
  const Scalar _tmp64 = std::pow(_tmp60, Scalar(2)) + std::pow(_tmp63, Scalar(2));
  const Scalar _tmp65 = std::pow(_tmp64, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp66 = _tmp63 * _tmp65;
  const Scalar _tmp67 = _tmp60 * _tmp65;
  const Scalar _tmp68 = _tmp57 * _tmp66 - _tmp67;
  const Scalar _tmp69 = Scalar(0.20999999999999999) * _tmp40 - Scalar(0.20999999999999999) * _tmp41;
  const Scalar _tmp70 =
      -Scalar(0.010999999999999999) * _tmp13 - Scalar(0.010999999999999999) * _tmp37;
  const Scalar _tmp71 = Scalar(0.20999999999999999) * _tmp24 + Scalar(0.20999999999999999) * _tmp29;
  const Scalar _tmp72 = _tmp70 - _tmp71;
  const Scalar _tmp73 = _tmp69 + _tmp72;
  const Scalar _tmp74 = _tmp57 * _tmp73;
  const Scalar _tmp75 = -_tmp69;
  const Scalar _tmp76 = _tmp72 + _tmp75;
  const Scalar _tmp77 = _tmp54 + _tmp58;
  const Scalar _tmp78 = _tmp77 + p_init1 + Scalar(8.3196563700000006);
  const Scalar _tmp79 = _tmp50 + _tmp61;
  const Scalar _tmp80 = _tmp79 + p_init0 + Scalar(1.9874742000000001);
  const Scalar _tmp81 = std::pow(_tmp78, Scalar(2)) + std::pow(_tmp80, Scalar(2));
  const Scalar _tmp82 = std::pow(_tmp81, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp83 = _tmp78 * _tmp82;
  const Scalar _tmp84 = _tmp80 * _tmp82;
  const Scalar _tmp85 = -_tmp74 * _tmp84 + _tmp76 * _tmp83;
  const Scalar _tmp86 = _tmp57 * _tmp84 - _tmp83;
  const Scalar _tmp87 = Scalar(1.0) / (_tmp86);
  const Scalar _tmp88 = _tmp68 * _tmp87;
  const Scalar _tmp89 = _tmp70 + _tmp71 + _tmp75;
  const Scalar _tmp90 = -_tmp66 * _tmp74 + _tmp67 * _tmp89 - _tmp85 * _tmp88;
  const Scalar _tmp91 = Scalar(1.0) * _tmp51;
  const Scalar _tmp92 = -_tmp79 + _tmp91;
  const Scalar _tmp93 = Scalar(1.0) * _tmp55;
  const Scalar _tmp94 = _tmp77 - _tmp93;
  const Scalar _tmp95 = Scalar(1.0) / (_tmp94);
  const Scalar _tmp96 = _tmp92 * _tmp95;
  const Scalar _tmp97 = _tmp73 * _tmp84;
  const Scalar _tmp98 = -_tmp76 * _tmp84 + _tmp97;
  const Scalar _tmp99 = _tmp66 * _tmp73;
  const Scalar _tmp100 = -_tmp66 * _tmp89 - _tmp88 * _tmp98 - _tmp90 * _tmp96 + _tmp99;
  const Scalar _tmp101 = Scalar(1.0) / (_tmp100);
  const Scalar _tmp102 = std::pow(_tmp52, Scalar(2));
  const Scalar _tmp103 = _tmp102 + std::pow(_tmp56, Scalar(2));
  const Scalar _tmp104 = std::sqrt(_tmp103);
  const Scalar _tmp105 = Scalar(1.0) / (_tmp104);
  const Scalar _tmp106 = _tmp105 * _tmp52;
  const Scalar _tmp107 = _tmp105 * _tmp51;
  const Scalar _tmp108 = -_tmp106 * _tmp55 + _tmp107 * _tmp56;
  const Scalar _tmp109 = _tmp104 * _tmp53;
  const Scalar _tmp110 = _tmp108 * _tmp109;
  const Scalar _tmp111 = _tmp110 * _tmp84 + _tmp77 * _tmp84 - _tmp79 * _tmp83;
  const Scalar _tmp112 = Scalar(1.0) * _tmp87;
  const Scalar _tmp113 = _tmp112 * _tmp96;
  const Scalar _tmp114 = -_tmp112 * _tmp98 + _tmp113 * _tmp85;
  const Scalar _tmp115 = _tmp110 * _tmp66 - _tmp111 * _tmp88 + _tmp59 * _tmp66 - _tmp62 * _tmp67;
  const Scalar _tmp116 = _tmp101 * _tmp115;
  const Scalar _tmp117 = -_tmp111 * _tmp112 - _tmp114 * _tmp116;
  const Scalar _tmp118 = Scalar(1.0) / (_tmp115);
  const Scalar _tmp119 = _tmp100 * _tmp118;
  const Scalar _tmp120 = _tmp114 + _tmp117 * _tmp119;
  const Scalar _tmp121 = _tmp101 * _tmp120;
  const Scalar _tmp122 = -_tmp121 * _tmp68 + Scalar(1.0);
  const Scalar _tmp123 = _tmp84 * _tmp87;
  const Scalar _tmp124 = _tmp121 * _tmp66 + _tmp122 * _tmp123;
  const Scalar _tmp125 = _tmp109 * _tmp124;
  const Scalar _tmp126 = _tmp44 * _tmp47;
  const Scalar _tmp127 = _tmp45 * _tmp47;
  const Scalar _tmp128 = _tmp126 * _tmp36 - _tmp127 * _tmp35;
  const Scalar _tmp129 = _tmp128 * fh1;
  const Scalar _tmp130 = _tmp112 * _tmp118;
  const Scalar _tmp131 = _tmp130 * _tmp68;
  const Scalar _tmp132 = Scalar(1.0) * _tmp118;
  const Scalar _tmp133 = -_tmp131 * _tmp84 + _tmp132 * _tmp66;
  const Scalar _tmp134 = _tmp109 * _tmp133;
  const Scalar _tmp135 = _tmp127 * fh1;
  const Scalar _tmp136 = _tmp57 * _tmp87;
  const Scalar _tmp137 = _tmp136 * _tmp85 + _tmp74;
  const Scalar _tmp138 = _tmp136 * _tmp98 - _tmp137 * _tmp96 - _tmp73;
  const Scalar _tmp139 = -_tmp110 + _tmp111 * _tmp136 - _tmp116 * _tmp138;
  const Scalar _tmp140 = _tmp119 * _tmp139 + _tmp138;
  const Scalar _tmp141 = _tmp101 * _tmp140;
  const Scalar _tmp142 = -_tmp141 * _tmp68 - _tmp57;
  const Scalar _tmp143 = _tmp123 * _tmp142 + _tmp141 * _tmp66 + Scalar(1.0);
  const Scalar _tmp144 = _tmp109 * _tmp143;
  const Scalar _tmp145 = _tmp91 + _tmp93 * _tmp96;
  const Scalar _tmp146 = 0;
  const Scalar _tmp147 = _tmp101 * _tmp66;
  const Scalar _tmp148 = _tmp101 * _tmp146;
  const Scalar _tmp149 = _tmp148 * _tmp84;
  const Scalar _tmp150 = _tmp146 * _tmp147 - _tmp149 * _tmp88;
  const Scalar _tmp151 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp152 = _tmp109 * _tmp151;
  const Scalar _tmp153 = (Scalar(1) / Scalar(2)) / _tmp1;
  const Scalar _tmp154 = _tmp0 * _tmp153;
  const Scalar _tmp155 = _tmp153 * rz;
  const Scalar _tmp156 = _tmp14 * _tmp155;
  const Scalar _tmp157 = _tmp155 * rx;
  const Scalar _tmp158 = _tmp6 / (_tmp1 * std::sqrt(_tmp1));
  const Scalar _tmp159 = _tmp0 * _tmp158;
  const Scalar _tmp160 = _tmp158 * rz;
  const Scalar _tmp161 = _tmp160 * rx;
  const Scalar _tmp162 = _tmp160 * ry;
  const Scalar _tmp163 = _tmp10 + _tmp154 * _tmp20 - _tmp156 * ry + _tmp157 * _tmp25 -
                         _tmp159 * rot_init_y - _tmp161 * rot_init_w + _tmp162 * rot_init_z -
                         Scalar(1) / Scalar(2) * _tmp21;
  const Scalar _tmp164 = Scalar(0.83999999999999997) * _tmp12;
  const Scalar _tmp165 = _tmp163 * _tmp164;
  const Scalar _tmp166 = _tmp155 * ry;
  const Scalar _tmp167 = _tmp154 * _tmp25 - _tmp157 * _tmp20 - _tmp159 * rot_init_w +
                         _tmp161 * rot_init_y - _tmp162 * rot_init_x + _tmp166 * _tmp5 -
                         Scalar(1) / Scalar(2) * _tmp26 + _tmp9;
  const Scalar _tmp168 = Scalar(0.83999999999999997) * _tmp17;
  const Scalar _tmp169 = _tmp167 * _tmp168;
  const Scalar _tmp170 = -_tmp14 * _tmp154 - _tmp157 * _tmp5 + _tmp159 * rot_init_z -
                         Scalar(1) / Scalar(2) * _tmp16 + _tmp161 * rot_init_x +
                         _tmp162 * rot_init_y - _tmp166 * _tmp20 - _tmp8;
  const Scalar _tmp171 = Scalar(0.41999999999999998) * _tmp17;
  const Scalar _tmp172 = _tmp170 * _tmp171;
  const Scalar _tmp173 = Scalar(0.41999999999999998) * _tmp27;
  const Scalar _tmp174 = _tmp167 * _tmp173;
  const Scalar _tmp175 = _tmp172 + _tmp174;
  const Scalar _tmp176 = _tmp169 + _tmp175;
  const Scalar _tmp177 = Scalar(0.41999999999999998) * _tmp22;
  const Scalar _tmp178 = _tmp163 * _tmp177;
  const Scalar _tmp179 = -Scalar(1) / Scalar(2) * _tmp11 - _tmp15 - _tmp154 * _tmp5 + _tmp156 * rx +
                         _tmp159 * rot_init_x - _tmp161 * rot_init_z - _tmp162 * rot_init_w +
                         _tmp166 * _tmp25;
  const Scalar _tmp180 = Scalar(0.41999999999999998) * _tmp12;
  const Scalar _tmp181 = _tmp179 * _tmp180;
  const Scalar _tmp182 = _tmp178 + _tmp181;
  const Scalar _tmp183 = Scalar(0.021999999999999999) * _tmp12;
  const Scalar _tmp184 = Scalar(0.021999999999999999) * _tmp17;
  const Scalar _tmp185 = Scalar(0.021999999999999999) * _tmp22;
  const Scalar _tmp186 = Scalar(0.021999999999999999) * _tmp163 * _tmp27 - _tmp167 * _tmp185 +
                         _tmp170 * _tmp183 - _tmp179 * _tmp184;
  const Scalar _tmp187 = _tmp182 + _tmp186;
  const Scalar _tmp188 = _tmp165 + _tmp176 + _tmp187;
  const Scalar _tmp189 = _tmp188 * _tmp53;
  const Scalar _tmp190 = -_tmp172 - _tmp174;
  const Scalar _tmp191 = _tmp169 + _tmp190;
  const Scalar _tmp192 = -_tmp178 - _tmp181;
  const Scalar _tmp193 = _tmp186 + _tmp192;
  const Scalar _tmp194 = _tmp165 + _tmp191 + _tmp193;
  const Scalar _tmp195 = _tmp179 * _tmp22;
  const Scalar _tmp196 = Scalar(0.83999999999999997) * _tmp195;
  const Scalar _tmp197 = _tmp179 * _tmp27;
  const Scalar _tmp198 = -_tmp163 * _tmp184 - _tmp167 * _tmp183 - _tmp170 * _tmp185 -
                         Scalar(0.021999999999999999) * _tmp197;
  const Scalar _tmp199 = _tmp196 + _tmp198;
  const Scalar _tmp200 = _tmp176 + _tmp192 + _tmp199;
  const Scalar _tmp201 =
      (2 * _tmp194 * _tmp78 + 2 * _tmp200 * _tmp80) / (_tmp81 * std::sqrt(_tmp81));
  const Scalar _tmp202 = (Scalar(1) / Scalar(2)) * _tmp201;
  const Scalar _tmp203 = _tmp202 * _tmp78;
  const Scalar _tmp204 = _tmp194 * _tmp82;
  const Scalar _tmp205 = -_tmp196 + _tmp198;
  const Scalar _tmp206 = -_tmp169;
  const Scalar _tmp207 = _tmp175 + _tmp206;
  const Scalar _tmp208 = _tmp192 + _tmp205 + _tmp207;
  const Scalar _tmp209 = _tmp208 / _tmp102;
  const Scalar _tmp210 = _tmp209 * _tmp56;
  const Scalar _tmp211 = _tmp202 * _tmp80;
  const Scalar _tmp212 = _tmp200 * _tmp82;
  const Scalar _tmp213 = (_tmp189 * _tmp84 + _tmp203 - _tmp204 - _tmp210 * _tmp84 -
                          _tmp211 * _tmp57 + _tmp212 * _tmp57) /
                         std::pow(_tmp86, Scalar(2));
  const Scalar _tmp214 = _tmp213 * _tmp84;
  const Scalar _tmp215 = _tmp182 + _tmp191 + _tmp199;
  const Scalar _tmp216 = _tmp215 * _tmp65;
  const Scalar _tmp217 = _tmp142 * _tmp87;
  const Scalar _tmp218 = -_tmp165;
  const Scalar _tmp219 = _tmp190 + _tmp206;
  const Scalar _tmp220 = _tmp193 + _tmp218 + _tmp219;
  const Scalar _tmp221 =
      (2 * _tmp215 * _tmp63 + 2 * _tmp220 * _tmp60) / (_tmp64 * std::sqrt(_tmp64));
  const Scalar _tmp222 = (Scalar(1) / Scalar(2)) * _tmp221;
  const Scalar _tmp223 = _tmp222 * _tmp63;
  const Scalar _tmp224 = _tmp212 * _tmp87;
  const Scalar _tmp225 = _tmp92 *
                         (-Scalar(0.83999999999999997) * _tmp163 * _tmp22 - _tmp164 * _tmp179 -
                          Scalar(0.83999999999999997) * _tmp167 * _tmp27 - _tmp168 * _tmp170) /
                         std::pow(_tmp94, Scalar(2));
  const Scalar _tmp226 = _tmp220 * _tmp65;
  const Scalar _tmp227 = _tmp222 * _tmp60;
  const Scalar _tmp228 =
      _tmp189 * _tmp66 - _tmp210 * _tmp66 + _tmp216 * _tmp57 - _tmp223 * _tmp57 - _tmp226 + _tmp227;
  const Scalar _tmp229 = _tmp228 * _tmp87;
  const Scalar _tmp230 = _tmp213 * _tmp98;
  const Scalar _tmp231 = Scalar(0.043999999999999997) * _tmp12 * _tmp163;
  const Scalar _tmp232 = Scalar(0.043999999999999997) * _tmp195;
  const Scalar _tmp233 = _tmp170 * _tmp177;
  const Scalar _tmp234 = _tmp163 * _tmp171;
  const Scalar _tmp235 = _tmp167 * _tmp180;
  const Scalar _tmp236 = Scalar(0.41999999999999998) * _tmp197;
  const Scalar _tmp237 = _tmp170 * _tmp180;
  const Scalar _tmp238 = _tmp171 * _tmp179;
  const Scalar _tmp239 = _tmp167 * _tmp177;
  const Scalar _tmp240 = _tmp163 * _tmp173;
  const Scalar _tmp241 = -_tmp237 - _tmp238 - _tmp239 - _tmp240;
  const Scalar _tmp242 = _tmp231 + _tmp232 - _tmp233 + _tmp234 + _tmp235 - _tmp236 + _tmp241;
  const Scalar _tmp243 = _tmp233 - _tmp234 - _tmp235 + _tmp236;
  const Scalar _tmp244 = _tmp231 + _tmp232 + _tmp243;
  const Scalar _tmp245 = _tmp241 + _tmp244;
  const Scalar _tmp246 = _tmp212 * _tmp73;
  const Scalar _tmp247 = -_tmp211 * _tmp73 + _tmp211 * _tmp76 - _tmp212 * _tmp76 +
                         _tmp242 * _tmp84 - _tmp245 * _tmp84 + _tmp246;
  const Scalar _tmp248 = _tmp216 * _tmp73;
  const Scalar _tmp249 = _tmp95 * (-Scalar(1.6799999999999999) * _tmp167 * _tmp17 -
                                   Scalar(1.6799999999999999) * _tmp195);
  const Scalar _tmp250 = _tmp242 * _tmp66;
  const Scalar _tmp251 = _tmp237 + _tmp238 + _tmp239 + _tmp240;
  const Scalar _tmp252 = _tmp244 + _tmp251;
  const Scalar _tmp253 = _tmp213 * _tmp68;
  const Scalar _tmp254 = _tmp242 * _tmp57;
  const Scalar _tmp255 = -_tmp189 * _tmp97 - _tmp203 * _tmp76 + _tmp204 * _tmp76 +
                         _tmp210 * _tmp97 + _tmp211 * _tmp74 + _tmp245 * _tmp83 - _tmp246 * _tmp57 -
                         _tmp254 * _tmp84;
  const Scalar _tmp256 =
      -_tmp216 * _tmp89 - _tmp223 * _tmp73 + _tmp223 * _tmp89 + _tmp225 * _tmp90 -
      _tmp229 * _tmp98 + _tmp230 * _tmp68 - _tmp247 * _tmp88 + _tmp248 - _tmp249 * _tmp90 +
      _tmp250 - _tmp252 * _tmp66 -
      _tmp96 * (-_tmp189 * _tmp99 + _tmp210 * _tmp99 + _tmp223 * _tmp74 + _tmp226 * _tmp89 -
                _tmp227 * _tmp89 - _tmp229 * _tmp85 - _tmp248 * _tmp57 - _tmp250 * _tmp57 +
                _tmp252 * _tmp67 + _tmp253 * _tmp85 - _tmp255 * _tmp88);
  const Scalar _tmp257 = _tmp256 / std::pow(_tmp100, Scalar(2));
  const Scalar _tmp258 = _tmp257 * _tmp68;
  const Scalar _tmp259 = _tmp104 * _tmp209;
  const Scalar _tmp260 = _tmp108 * _tmp259;
  const Scalar _tmp261 = 2 * _tmp188 * _tmp56 + 2 * _tmp208 * _tmp52;
  const Scalar _tmp262 = (Scalar(1) / Scalar(2)) * _tmp261;
  const Scalar _tmp263 = _tmp262 / (_tmp103 * std::sqrt(_tmp103));
  const Scalar _tmp264 = _tmp105 * _tmp208;
  const Scalar _tmp265 =
      _tmp109 * (-_tmp106 * _tmp188 + _tmp107 * _tmp188 - _tmp263 * _tmp51 * _tmp56 +
                 _tmp263 * _tmp52 * _tmp55 - _tmp264 * _tmp55 + _tmp264 * _tmp56);
  const Scalar _tmp266 = _tmp105 * _tmp53;
  const Scalar _tmp267 = _tmp262 * _tmp266;
  const Scalar _tmp268 = _tmp108 * _tmp267;
  const Scalar _tmp269 = -_tmp110 * _tmp211 + _tmp110 * _tmp212 - _tmp200 * _tmp83 +
                         _tmp203 * _tmp79 - _tmp204 * _tmp79 + _tmp204 * _tmp80 - _tmp211 * _tmp77 +
                         _tmp212 * _tmp77 - _tmp260 * _tmp84 + _tmp265 * _tmp84 + _tmp268 * _tmp84;
  const Scalar _tmp270 = _tmp110 * _tmp216 - _tmp110 * _tmp223 - _tmp111 * _tmp229 +
                         _tmp111 * _tmp253 - _tmp215 * _tmp67 + _tmp216 * _tmp59 -
                         _tmp223 * _tmp59 - _tmp226 * _tmp62 + _tmp226 * _tmp63 + _tmp227 * _tmp62 -
                         _tmp260 * _tmp66 + _tmp265 * _tmp66 + _tmp268 * _tmp66 - _tmp269 * _tmp88;
  const Scalar _tmp271 = _tmp270 / std::pow(_tmp115, Scalar(2));
  const Scalar _tmp272 = _tmp100 * _tmp271;
  const Scalar _tmp273 = _tmp118 * _tmp256;
  const Scalar _tmp274 = _tmp189 * _tmp87;
  const Scalar _tmp275 = _tmp210 * _tmp87;
  const Scalar _tmp276 = _tmp213 * _tmp85;
  const Scalar _tmp277 =
      _tmp136 * _tmp247 + _tmp137 * _tmp225 - _tmp137 * _tmp249 - _tmp230 * _tmp57 - _tmp231 -
      _tmp232 + _tmp243 + _tmp251 + _tmp274 * _tmp98 - _tmp275 * _tmp98 -
      _tmp96 * (_tmp136 * _tmp255 + _tmp189 * _tmp73 - _tmp210 * _tmp73 + _tmp254 +
                _tmp274 * _tmp85 - _tmp275 * _tmp85 - _tmp276 * _tmp57);
  const Scalar _tmp278 = _tmp115 * _tmp257;
  const Scalar _tmp279 = _tmp101 * _tmp270;
  const Scalar _tmp280 = _tmp111 * _tmp213;
  const Scalar _tmp281 = _tmp119 * (_tmp111 * _tmp274 - _tmp111 * _tmp275 - _tmp116 * _tmp277 +
                                    _tmp136 * _tmp269 + _tmp138 * _tmp278 - _tmp138 * _tmp279 +
                                    _tmp260 - _tmp265 - _tmp268 - _tmp280 * _tmp57) -
                         _tmp139 * _tmp272 + _tmp139 * _tmp273 + _tmp277;
  const Scalar _tmp282 = _tmp101 * _tmp68;
  const Scalar _tmp283 =
      _tmp140 * _tmp258 - _tmp141 * _tmp228 - _tmp189 + _tmp210 - _tmp281 * _tmp282;
  const Scalar _tmp284 = _tmp257 * _tmp66;
  const Scalar _tmp285 = _tmp182 + _tmp205 + _tmp219;
  const Scalar _tmp286 = _tmp285 * _tmp47;
  const Scalar _tmp287 = _tmp286 * fh1;
  const Scalar _tmp288 = _tmp150 * _tmp151;
  const Scalar _tmp289 = _tmp112 * _tmp85;
  const Scalar _tmp290 = -_tmp112 * _tmp247 + _tmp113 * _tmp255 - _tmp225 * _tmp289 +
                         Scalar(1.0) * _tmp230 + _tmp249 * _tmp289 - Scalar(1.0) * _tmp276 * _tmp96;
  const Scalar _tmp291 = -_tmp117 * _tmp272 + _tmp117 * _tmp273 +
                         _tmp119 * (-_tmp112 * _tmp269 + _tmp114 * _tmp278 - _tmp114 * _tmp279 -
                                    _tmp116 * _tmp290 + Scalar(1.0) * _tmp280) +
                         _tmp290;
  const Scalar _tmp292 = _tmp120 * _tmp258 - _tmp121 * _tmp228 - _tmp282 * _tmp291;
  const Scalar _tmp293 = _tmp122 * _tmp87;
  const Scalar _tmp294 = _tmp129 * _tmp133;
  const Scalar _tmp295 = _tmp187 + _tmp207 + _tmp218;
  const Scalar _tmp296 =
      (2 * _tmp285 * _tmp45 + 2 * _tmp295 * _tmp36) / (_tmp46 * std::sqrt(_tmp46));
  const Scalar _tmp297 = (Scalar(1) / Scalar(2)) * _tmp296 * _tmp45;
  const Scalar _tmp298 = _tmp297 * fh1;
  const Scalar _tmp299 = Scalar(0.5) * _tmp118;
  const Scalar _tmp300 = _tmp112 * _tmp271 * _tmp68;
  const Scalar _tmp301 = _tmp130 * _tmp228;
  const Scalar _tmp302 = Scalar(1.0) * _tmp271;
  const Scalar _tmp303 = (Scalar(1) / Scalar(2)) * _tmp36;
  const Scalar _tmp304 = _tmp296 * _tmp303;
  const Scalar _tmp305 = _tmp126 * _tmp295 - _tmp127 * _tmp295 - _tmp286 * _tmp35 +
                         _tmp286 * _tmp36 + _tmp297 * _tmp35 - _tmp304 * _tmp44;
  const Scalar _tmp306 = _tmp305 * fh1;
  const Scalar _tmp307 = _tmp295 * _tmp48;
  const Scalar _tmp308 = _tmp304 * fh1;
  const Scalar _tmp309 = _tmp135 * _tmp143;
  const Scalar _tmp310 = _tmp148 * _tmp88;
  const Scalar _tmp311 = _tmp49 * _tmp87;
  const Scalar _tmp312 = _tmp146 * _tmp151;
  const Scalar _tmp313 = _tmp101 * _tmp312;
  const Scalar _tmp314 = _tmp132 * fh1;
  const Scalar _tmp315 = _tmp128 * _tmp314;
  const Scalar _tmp316 = _tmp257 * _tmp312;

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) = 0;
  _res(1, 0) =
      -(-_tmp109 * _tmp129 *
            (-_tmp131 * _tmp212 + _tmp132 * _tmp216 + _tmp132 * _tmp253 * _tmp84 +
             _tmp201 * _tmp299 * _tmp80 * _tmp88 - _tmp221 * _tmp299 * _tmp63 + _tmp300 * _tmp84 -
             _tmp301 * _tmp84 - _tmp302 * _tmp66) -
        _tmp109 * _tmp135 *
            (_tmp123 * _tmp283 - _tmp140 * _tmp284 + _tmp141 * _tmp216 - _tmp141 * _tmp223 -
             _tmp142 * _tmp214 + _tmp142 * _tmp224 + _tmp147 * _tmp281 - _tmp211 * _tmp217) -
        _tmp109 * _tmp49 *
            (-_tmp120 * _tmp284 + _tmp121 * _tmp216 - _tmp121 * _tmp223 - _tmp122 * _tmp214 +
             _tmp122 * _tmp224 + _tmp123 * _tmp292 + _tmp147 * _tmp291 - _tmp211 * _tmp293) +
        _tmp124 * _tmp259 * _tmp49 - _tmp124 * _tmp261 * _tmp266 * _tmp303 * _tmp48 -
        _tmp125 * _tmp307 + _tmp125 * _tmp308 - _tmp134 * _tmp306 - _tmp144 * _tmp287 +
        _tmp144 * _tmp298 -
        _tmp152 * (_tmp146 * _tmp214 * _tmp282 + _tmp146 * _tmp257 * _tmp84 * _tmp88 -
                   _tmp146 * _tmp284 + _tmp148 * _tmp216 - _tmp148 * _tmp223 - _tmp149 * _tmp229 +
                   _tmp211 * _tmp310 - _tmp212 * _tmp310) +
        _tmp259 * _tmp288 + _tmp259 * _tmp294 + _tmp259 * _tmp309 - _tmp267 * _tmp288 -
        _tmp267 * _tmp294 - _tmp267 * _tmp309) *
      std::exp(_tmp125 * _tmp49 + _tmp129 * _tmp134 + _tmp135 * _tmp144 + _tmp150 * _tmp152);
  _res(2, 0) =
      -(-_tmp122 * _tmp213 * _tmp49 + _tmp129 * _tmp300 - _tmp129 * _tmp301 - _tmp131 * _tmp306 -
        _tmp135 * _tmp142 * _tmp213 + _tmp135 * _tmp283 * _tmp87 + _tmp217 * _tmp287 -
        _tmp217 * _tmp298 - _tmp229 * _tmp313 + _tmp253 * _tmp313 + _tmp253 * _tmp315 +
        _tmp292 * _tmp311 + _tmp293 * _tmp307 - _tmp293 * _tmp308 + _tmp316 * _tmp88) *
      std::exp(-_tmp122 * _tmp311 + _tmp129 * _tmp131 - _tmp135 * _tmp217 + _tmp313 * _tmp88);
  _res(3, 0) =
      -(_tmp101 * _tmp135 * _tmp281 + _tmp101 * _tmp291 * _tmp49 - _tmp120 * _tmp257 * _tmp49 +
        _tmp121 * _tmp307 - _tmp121 * _tmp308 - _tmp129 * _tmp302 - _tmp135 * _tmp140 * _tmp257 +
        _tmp141 * _tmp287 - _tmp141 * _tmp298 + _tmp305 * _tmp314 - _tmp316) *
      std::exp(-_tmp121 * _tmp49 - _tmp135 * _tmp141 - _tmp313 - _tmp315);

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
