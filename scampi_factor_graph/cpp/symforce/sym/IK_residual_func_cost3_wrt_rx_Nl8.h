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
 * Symbolic function: IK_residual_func_cost3_wrt_rx_Nl8
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost3WrtRxNl8(
    const Scalar fh1, const Scalar fv1, const Scalar rx, const Scalar ry, const Scalar rz,
    const Scalar p_init0, const Scalar p_init1, const Scalar p_init2, const Scalar rot_init_x,
    const Scalar rot_init_y, const Scalar rot_init_z, const Scalar rot_init_w,
    const Scalar epsilon) {
  // Total ops: 997

  // Unused inputs
  (void)p_init2;
  (void)epsilon;

  // Input arrays

  // Intermediate terms (329)
  const Scalar _tmp0 = std::pow(rx, Scalar(2));
  const Scalar _tmp1 = _tmp0 + std::pow(ry, Scalar(2)) + std::pow(rz, Scalar(2));
  const Scalar _tmp2 = std::sqrt(_tmp1);
  const Scalar _tmp3 = (Scalar(1) / Scalar(2)) * _tmp2;
  const Scalar _tmp4 = std::cos(_tmp3);
  const Scalar _tmp5 = _tmp4 * rot_init_z;
  const Scalar _tmp6 = std::sin(_tmp3);
  const Scalar _tmp7 = _tmp6 / _tmp2;
  const Scalar _tmp8 = _tmp7 * rot_init_x;
  const Scalar _tmp9 = _tmp7 * rot_init_y;
  const Scalar _tmp10 = _tmp9 * rx;
  const Scalar _tmp11 = _tmp7 * rot_init_w;
  const Scalar _tmp12 = -_tmp10 + _tmp11 * rz + _tmp5 + _tmp8 * ry;
  const Scalar _tmp13 = _tmp4 * rot_init_x;
  const Scalar _tmp14 = _tmp7 * rot_init_z;
  const Scalar _tmp15 = _tmp11 * rx;
  const Scalar _tmp16 = _tmp13 - _tmp14 * ry + _tmp15 + _tmp9 * rz;
  const Scalar _tmp17 = 2 * _tmp16;
  const Scalar _tmp18 = _tmp12 * _tmp17;
  const Scalar _tmp19 = _tmp4 * rot_init_y;
  const Scalar _tmp20 = _tmp14 * rx;
  const Scalar _tmp21 = _tmp11 * ry + _tmp19 + _tmp20 - _tmp8 * rz;
  const Scalar _tmp22 = _tmp4 * rot_init_w;
  const Scalar _tmp23 = _tmp8 * rx;
  const Scalar _tmp24 = -_tmp14 * rz + _tmp22 - _tmp23 - _tmp9 * ry;
  const Scalar _tmp25 = 2 * _tmp24;
  const Scalar _tmp26 = _tmp21 * _tmp25;
  const Scalar _tmp27 = Scalar(0.20999999999999999) * _tmp18 - Scalar(0.20999999999999999) * _tmp26;
  const Scalar _tmp28 = -_tmp27;
  const Scalar _tmp29 = -2 * std::pow(_tmp16, Scalar(2));
  const Scalar _tmp30 = -2 * std::pow(_tmp21, Scalar(2));
  const Scalar _tmp31 = -Scalar(0.010999999999999999) * _tmp29 -
                        Scalar(0.010999999999999999) * _tmp30 + Scalar(-0.010999999999999999);
  const Scalar _tmp32 = 2 * _tmp12 * _tmp21;
  const Scalar _tmp33 = _tmp16 * _tmp25;
  const Scalar _tmp34 = Scalar(0.20999999999999999) * _tmp32 + Scalar(0.20999999999999999) * _tmp33;
  const Scalar _tmp35 = _tmp31 + _tmp34;
  const Scalar _tmp36 = _tmp28 + _tmp35;
  const Scalar _tmp37 = 1 - 2 * std::pow(_tmp12, Scalar(2));
  const Scalar _tmp38 = Scalar(0.20999999999999999) * _tmp29 + Scalar(0.20999999999999999) * _tmp37;
  const Scalar _tmp39 =
      -Scalar(0.010999999999999999) * _tmp32 + Scalar(0.010999999999999999) * _tmp33;
  const Scalar _tmp40 = _tmp17 * _tmp21;
  const Scalar _tmp41 = _tmp12 * _tmp25;
  const Scalar _tmp42 = Scalar(0.20999999999999999) * _tmp40 + Scalar(0.20999999999999999) * _tmp41;
  const Scalar _tmp43 = _tmp39 - _tmp42;
  const Scalar _tmp44 = _tmp38 + _tmp43;
  const Scalar _tmp45 = _tmp44 + p_init1 + Scalar(-4.8333311099999996);
  const Scalar _tmp46 = Scalar(0.20999999999999999) * _tmp30 + Scalar(0.20999999999999999) * _tmp37;
  const Scalar _tmp47 = -_tmp46;
  const Scalar _tmp48 = Scalar(0.20999999999999999) * _tmp40 - Scalar(0.20999999999999999) * _tmp41;
  const Scalar _tmp49 =
      -Scalar(0.010999999999999999) * _tmp18 - Scalar(0.010999999999999999) * _tmp26;
  const Scalar _tmp50 = _tmp48 + _tmp49;
  const Scalar _tmp51 = _tmp47 + _tmp50;
  const Scalar _tmp52 = _tmp51 + p_init0 + Scalar(1.79662371);
  const Scalar _tmp53 = std::pow(_tmp45, Scalar(2)) + std::pow(_tmp52, Scalar(2));
  const Scalar _tmp54 = std::pow(_tmp53, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp55 = _tmp45 * _tmp54;
  const Scalar _tmp56 = _tmp46 + _tmp50;
  const Scalar _tmp57 = _tmp56 + p_init0 + Scalar(-2.71799795);
  const Scalar _tmp58 = Scalar(1.0) / (_tmp57);
  const Scalar _tmp59 = _tmp39 + _tmp42;
  const Scalar _tmp60 = _tmp38 + _tmp59;
  const Scalar _tmp61 = _tmp60 + p_init1 + Scalar(-4.7752063900000001);
  const Scalar _tmp62 = _tmp58 * _tmp61;
  const Scalar _tmp63 = _tmp27 + _tmp35;
  const Scalar _tmp64 = _tmp52 * _tmp54;
  const Scalar _tmp65 = _tmp63 * _tmp64;
  const Scalar _tmp66 = -_tmp55 + _tmp62 * _tmp64;
  const Scalar _tmp67 = -_tmp38;
  const Scalar _tmp68 = _tmp43 + _tmp67;
  const Scalar _tmp69 = _tmp68 + p_init1 + Scalar(8.3196563700000006);
  const Scalar _tmp70 = -_tmp48 + _tmp49;
  const Scalar _tmp71 = _tmp47 + _tmp70;
  const Scalar _tmp72 = _tmp71 + p_init0 + Scalar(1.9874742000000001);
  const Scalar _tmp73 = std::pow(_tmp69, Scalar(2)) + std::pow(_tmp72, Scalar(2));
  const Scalar _tmp74 = std::pow(_tmp73, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp75 = _tmp69 * _tmp74;
  const Scalar _tmp76 = _tmp72 * _tmp74;
  const Scalar _tmp77 = _tmp62 * _tmp76 - _tmp75;
  const Scalar _tmp78 = Scalar(1.0) / (_tmp77);
  const Scalar _tmp79 = _tmp63 * _tmp76;
  const Scalar _tmp80 = _tmp28 + _tmp31 - _tmp34;
  const Scalar _tmp81 = -_tmp62 * _tmp79 + _tmp75 * _tmp80;
  const Scalar _tmp82 = _tmp78 * _tmp81;
  const Scalar _tmp83 = _tmp36 * _tmp55 - _tmp62 * _tmp65 - _tmp66 * _tmp82;
  const Scalar _tmp84 = Scalar(1.0) * _tmp60;
  const Scalar _tmp85 = _tmp68 - _tmp84;
  const Scalar _tmp86 = Scalar(1.0) / (_tmp85);
  const Scalar _tmp87 = Scalar(1.0) * _tmp56;
  const Scalar _tmp88 = -_tmp71 + _tmp87;
  const Scalar _tmp89 = _tmp86 * _tmp88;
  const Scalar _tmp90 = -_tmp76 * _tmp80 + _tmp79;
  const Scalar _tmp91 = _tmp66 * _tmp78;
  const Scalar _tmp92 = -_tmp36 * _tmp64 + _tmp65 - _tmp83 * _tmp89 - _tmp90 * _tmp91;
  const Scalar _tmp93 = std::pow(_tmp57, Scalar(2));
  const Scalar _tmp94 = std::pow(_tmp61, Scalar(2)) + _tmp93;
  const Scalar _tmp95 = std::sqrt(_tmp94);
  const Scalar _tmp96 = Scalar(1.0) / (_tmp95);
  const Scalar _tmp97 = _tmp61 * _tmp96;
  const Scalar _tmp98 = _tmp60 * _tmp96;
  const Scalar _tmp99 = _tmp56 * _tmp97 - _tmp57 * _tmp98;
  const Scalar _tmp100 = _tmp58 * _tmp95;
  const Scalar _tmp101 = _tmp100 * _tmp99;
  const Scalar _tmp102 = _tmp101 * _tmp76 + _tmp68 * _tmp76 - _tmp71 * _tmp75;
  const Scalar _tmp103 = _tmp102 * _tmp78;
  const Scalar _tmp104 = _tmp101 * _tmp64 - _tmp103 * _tmp66 + _tmp44 * _tmp64 - _tmp51 * _tmp55;
  const Scalar _tmp105 = Scalar(1.0) / (_tmp92);
  const Scalar _tmp106 = Scalar(1.0) * _tmp82;
  const Scalar _tmp107 = _tmp78 * _tmp90;
  const Scalar _tmp108 = _tmp106 * _tmp89 - Scalar(1.0) * _tmp107;
  const Scalar _tmp109 = _tmp105 * _tmp108;
  const Scalar _tmp110 = -Scalar(1.0) * _tmp103 - _tmp104 * _tmp109;
  const Scalar _tmp111 = Scalar(1.0) / (_tmp104);
  const Scalar _tmp112 = _tmp110 * _tmp111;
  const Scalar _tmp113 = _tmp108 + _tmp112 * _tmp92;
  const Scalar _tmp114 = _tmp105 * _tmp66;
  const Scalar _tmp115 = -_tmp113 * _tmp114 + Scalar(1.0);
  const Scalar _tmp116 = _tmp76 * _tmp78;
  const Scalar _tmp117 = _tmp105 * _tmp64;
  const Scalar _tmp118 = _tmp113 * _tmp117 + _tmp115 * _tmp116;
  const Scalar _tmp119 = _tmp59 + _tmp67;
  const Scalar _tmp120 = _tmp119 + p_init1 + Scalar(8.3888750099999996);
  const Scalar _tmp121 = _tmp46 + _tmp70;
  const Scalar _tmp122 = _tmp121 + p_init0 + Scalar(-2.5202214700000001);
  const Scalar _tmp123 = std::pow(_tmp120, Scalar(2)) + std::pow(_tmp122, Scalar(2));
  const Scalar _tmp124 = std::pow(_tmp123, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp125 = _tmp120 * _tmp124;
  const Scalar _tmp126 = _tmp100 * fh1;
  const Scalar _tmp127 = _tmp125 * _tmp126;
  const Scalar _tmp128 = _tmp62 * _tmp63;
  const Scalar _tmp129 = _tmp128 + _tmp62 * _tmp82;
  const Scalar _tmp130 = _tmp129 * _tmp86;
  const Scalar _tmp131 = _tmp107 * _tmp62 - _tmp130 * _tmp88 - _tmp63;
  const Scalar _tmp132 = _tmp105 * _tmp131;
  const Scalar _tmp133 = -_tmp101 + _tmp103 * _tmp62 - _tmp104 * _tmp132;
  const Scalar _tmp134 = _tmp111 * _tmp133;
  const Scalar _tmp135 = _tmp131 + _tmp134 * _tmp92;
  const Scalar _tmp136 = -_tmp114 * _tmp135 - _tmp62;
  const Scalar _tmp137 = _tmp136 * _tmp78;
  const Scalar _tmp138 = _tmp117 * _tmp135 + _tmp137 * _tmp76 + Scalar(1.0);
  const Scalar _tmp139 = _tmp122 * _tmp124;
  const Scalar _tmp140 = _tmp126 * _tmp139;
  const Scalar _tmp141 = _tmp84 * _tmp89 + _tmp87;
  const Scalar _tmp142 = 0;
  const Scalar _tmp143 = _tmp116 * _tmp142;
  const Scalar _tmp144 = -_tmp114 * _tmp143 + _tmp117 * _tmp142;
  const Scalar _tmp145 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp146 = _tmp100 * _tmp145;
  const Scalar _tmp147 = Scalar(1.0) * _tmp111;
  const Scalar _tmp148 = _tmp147 * _tmp91;
  const Scalar _tmp149 = _tmp147 * _tmp64 - _tmp148 * _tmp76;
  const Scalar _tmp150 = -_tmp119 * _tmp139 + _tmp121 * _tmp125;
  const Scalar _tmp151 = _tmp150 * fh1;
  const Scalar _tmp152 = _tmp100 * _tmp151;
  const Scalar _tmp153 = (Scalar(1) / Scalar(2)) / _tmp1;
  const Scalar _tmp154 = _tmp0 * _tmp153;
  const Scalar _tmp155 = _tmp153 * rx;
  const Scalar _tmp156 = _tmp155 * rz;
  const Scalar _tmp157 = _tmp155 * ry;
  const Scalar _tmp158 = _tmp6 / (_tmp1 * std::sqrt(_tmp1));
  const Scalar _tmp159 = _tmp0 * _tmp158;
  const Scalar _tmp160 = _tmp158 * rx;
  const Scalar _tmp161 = _tmp160 * ry;
  const Scalar _tmp162 = _tmp160 * rz;
  const Scalar _tmp163 = -Scalar(1) / Scalar(2) * _tmp10 - _tmp13 * _tmp156 + _tmp14 +
                         _tmp154 * _tmp5 + _tmp157 * _tmp22 - _tmp159 * rot_init_z -
                         _tmp161 * rot_init_w + _tmp162 * rot_init_x;
  const Scalar _tmp164 = Scalar(0.41999999999999998) * _tmp163;
  const Scalar _tmp165 = _tmp16 * _tmp164;
  const Scalar _tmp166 = _tmp155 * _tmp19;
  const Scalar _tmp167 = _tmp11 + _tmp154 * _tmp22 - _tmp157 * _tmp5 - _tmp159 * rot_init_w +
                         _tmp161 * rot_init_z - _tmp162 * rot_init_y + _tmp166 * rz -
                         Scalar(1) / Scalar(2) * _tmp23;
  const Scalar _tmp168 = Scalar(0.41999999999999998) * _tmp167;
  const Scalar _tmp169 = _tmp168 * _tmp21;
  const Scalar _tmp170 = _tmp165 + _tmp169;
  const Scalar _tmp171 = _tmp13 * _tmp157 - _tmp154 * _tmp19 + _tmp156 * _tmp22 +
                         _tmp159 * rot_init_y - _tmp161 * rot_init_x - _tmp162 * rot_init_w -
                         Scalar(1) / Scalar(2) * _tmp20 - _tmp9;
  const Scalar _tmp172 = Scalar(0.83999999999999997) * _tmp12;
  const Scalar _tmp173 = _tmp171 * _tmp172;
  const Scalar _tmp174 = -_tmp13 * _tmp154 - Scalar(1) / Scalar(2) * _tmp15 - _tmp156 * _tmp5 +
                         _tmp159 * rot_init_x + _tmp161 * rot_init_y + _tmp162 * rot_init_z -
                         _tmp166 * ry - _tmp8;
  const Scalar _tmp175 = Scalar(0.41999999999999998) * _tmp174;
  const Scalar _tmp176 = _tmp12 * _tmp175;
  const Scalar _tmp177 = Scalar(0.41999999999999998) * _tmp171;
  const Scalar _tmp178 = _tmp177 * _tmp24;
  const Scalar _tmp179 = -_tmp176 - _tmp178;
  const Scalar _tmp180 = _tmp173 + _tmp179;
  const Scalar _tmp181 = Scalar(0.83999999999999997) * _tmp163;
  const Scalar _tmp182 = _tmp181 * _tmp21;
  const Scalar _tmp183 = Scalar(0.021999999999999999) * _tmp174;
  const Scalar _tmp184 = Scalar(0.021999999999999999) * _tmp171;
  const Scalar _tmp185 = Scalar(0.021999999999999999) * _tmp12;
  const Scalar _tmp186 = Scalar(0.021999999999999999) * _tmp24;
  const Scalar _tmp187 =
      -_tmp16 * _tmp184 - _tmp163 * _tmp186 - _tmp167 * _tmp185 - _tmp183 * _tmp21;
  const Scalar _tmp188 = _tmp182 + _tmp187;
  const Scalar _tmp189 = _tmp170 + _tmp180 + _tmp188;
  const Scalar _tmp190 = -_tmp173;
  const Scalar _tmp191 = -_tmp165 - _tmp169;
  const Scalar _tmp192 = _tmp190 + _tmp191;
  const Scalar _tmp193 = Scalar(0.83999999999999997) * _tmp167;
  const Scalar _tmp194 = _tmp16 * _tmp193;
  const Scalar _tmp195 =
      _tmp16 * _tmp183 - _tmp163 * _tmp185 + _tmp167 * _tmp186 - _tmp184 * _tmp21;
  const Scalar _tmp196 = -_tmp194 + _tmp195;
  const Scalar _tmp197 = _tmp179 + _tmp192 + _tmp196;
  const Scalar _tmp198 =
      (2 * _tmp189 * _tmp52 + 2 * _tmp197 * _tmp45) / (_tmp53 * std::sqrt(_tmp53));
  const Scalar _tmp199 = (Scalar(1) / Scalar(2)) * _tmp198;
  const Scalar _tmp200 = _tmp199 * _tmp52;
  const Scalar _tmp201 = -_tmp182 + _tmp187;
  const Scalar _tmp202 = _tmp170 + _tmp179 + _tmp190 + _tmp201;
  const Scalar _tmp203 = _tmp202 / _tmp93;
  const Scalar _tmp204 = _tmp203 * _tmp61;
  const Scalar _tmp205 = _tmp199 * _tmp45;
  const Scalar _tmp206 = _tmp189 * _tmp54;
  const Scalar _tmp207 = _tmp176 + _tmp178;
  const Scalar _tmp208 = _tmp170 + _tmp207;
  const Scalar _tmp209 = _tmp190 + _tmp196 + _tmp208;
  const Scalar _tmp210 = _tmp209 * _tmp58;
  const Scalar _tmp211 = _tmp197 * _tmp54;
  const Scalar _tmp212 = -_tmp200 * _tmp62 - _tmp204 * _tmp64 + _tmp205 + _tmp206 * _tmp62 +
                         _tmp210 * _tmp64 - _tmp211;
  const Scalar _tmp213 = _tmp175 * _tmp21;
  const Scalar _tmp214 = _tmp16 * _tmp177;
  const Scalar _tmp215 = _tmp12 * _tmp168;
  const Scalar _tmp216 = _tmp164 * _tmp24;
  const Scalar _tmp217 = _tmp16 * _tmp175;
  const Scalar _tmp218 = _tmp177 * _tmp21;
  const Scalar _tmp219 = _tmp12 * _tmp164;
  const Scalar _tmp220 = _tmp168 * _tmp24;
  const Scalar _tmp221 = _tmp163 * _tmp21;
  const Scalar _tmp222 = Scalar(0.043999999999999997) * _tmp221;
  const Scalar _tmp223 = _tmp16 * _tmp167;
  const Scalar _tmp224 = Scalar(0.043999999999999997) * _tmp223;
  const Scalar _tmp225 = _tmp222 + _tmp224;
  const Scalar _tmp226 = _tmp217 + _tmp218 + _tmp219 + _tmp220 + _tmp225;
  const Scalar _tmp227 = -_tmp213 + _tmp214 + _tmp215 - _tmp216 + _tmp226;
  const Scalar _tmp228 = _tmp227 * _tmp64;
  const Scalar _tmp229 = _tmp227 * _tmp76;
  const Scalar _tmp230 = _tmp173 + _tmp188 + _tmp191 + _tmp207;
  const Scalar _tmp231 = _tmp194 + _tmp195;
  const Scalar _tmp232 = _tmp180 + _tmp191 + _tmp231;
  const Scalar _tmp233 =
      (2 * _tmp230 * _tmp72 + 2 * _tmp232 * _tmp69) / (_tmp73 * std::sqrt(_tmp73));
  const Scalar _tmp234 = (Scalar(1) / Scalar(2)) * _tmp233;
  const Scalar _tmp235 = _tmp234 * _tmp72;
  const Scalar _tmp236 = _tmp230 * _tmp74;
  const Scalar _tmp237 = _tmp236 * _tmp63;
  const Scalar _tmp238 = _tmp210 * _tmp63;
  const Scalar _tmp239 = _tmp213 - _tmp214 - _tmp215 + _tmp216;
  const Scalar _tmp240 = -_tmp217 - _tmp218 - _tmp219 - _tmp220 + _tmp239;
  const Scalar _tmp241 = _tmp225 + _tmp240;
  const Scalar _tmp242 = _tmp234 * _tmp69;
  const Scalar _tmp243 = _tmp232 * _tmp74;
  const Scalar _tmp244 = _tmp128 * _tmp235 + _tmp204 * _tmp79 - _tmp229 * _tmp62 -
                         _tmp237 * _tmp62 - _tmp238 * _tmp76 + _tmp241 * _tmp75 - _tmp242 * _tmp80 +
                         _tmp243 * _tmp80;
  const Scalar _tmp245 = _tmp226 + _tmp239;
  const Scalar _tmp246 = (-_tmp204 * _tmp76 + _tmp210 * _tmp76 - _tmp235 * _tmp62 +
                          _tmp236 * _tmp62 + _tmp242 - _tmp243) /
                         std::pow(_tmp77, Scalar(2));
  const Scalar _tmp247 = _tmp246 * _tmp66;
  const Scalar _tmp248 = Scalar(1.6799999999999999) * _tmp12 * _tmp171;
  const Scalar _tmp249 = _tmp16 * _tmp181;
  const Scalar _tmp250 = _tmp193 * _tmp21;
  const Scalar _tmp251 = -Scalar(0.83999999999999997) * _tmp171 * _tmp24 - _tmp172 * _tmp174;
  const Scalar _tmp252 =
      -Scalar(1.6799999999999999) * _tmp221 - _tmp248 + _tmp249 + _tmp250 + _tmp251;
  const Scalar _tmp253 = _tmp252 * _tmp86;
  const Scalar _tmp254 =
      _tmp229 - _tmp235 * _tmp63 + _tmp235 * _tmp80 - _tmp236 * _tmp80 + _tmp237 - _tmp241 * _tmp76;
  const Scalar _tmp255 =
      _tmp88 * (Scalar(1.6799999999999999) * _tmp223 + _tmp248 - _tmp249 - _tmp250 + _tmp251) /
      std::pow(_tmp85, Scalar(2));
  const Scalar _tmp256 =
      -_tmp107 * _tmp212 + _tmp200 * _tmp36 - _tmp200 * _tmp63 - _tmp206 * _tmp36 +
      _tmp206 * _tmp63 + _tmp228 - _tmp245 * _tmp64 + _tmp247 * _tmp90 - _tmp253 * _tmp83 -
      _tmp254 * _tmp91 + _tmp255 * _tmp83 -
      _tmp89 * (_tmp128 * _tmp200 - _tmp128 * _tmp206 + _tmp204 * _tmp65 - _tmp205 * _tmp36 -
                _tmp210 * _tmp65 + _tmp211 * _tmp36 - _tmp212 * _tmp82 - _tmp228 * _tmp62 -
                _tmp244 * _tmp91 + _tmp245 * _tmp55 + _tmp247 * _tmp81);
  const Scalar _tmp257 = _tmp256 / std::pow(_tmp92, Scalar(2));
  const Scalar _tmp258 = _tmp76 * _tmp91;
  const Scalar _tmp259 = _tmp114 * _tmp78;
  const Scalar _tmp260 = _tmp142 * _tmp259;
  const Scalar _tmp261 = _tmp105 * _tmp206;
  const Scalar _tmp262 = _tmp105 * _tmp200;
  const Scalar _tmp263 = _tmp246 * _tmp76;
  const Scalar _tmp264 = _tmp105 * _tmp212;
  const Scalar _tmp265 = _tmp257 * _tmp64;
  const Scalar _tmp266 = _tmp202 * _tmp57 + _tmp209 * _tmp61;
  const Scalar _tmp267 = _tmp266 * _tmp58 * _tmp96;
  const Scalar _tmp268 = _tmp149 * _tmp151;
  const Scalar _tmp269 = _tmp192 + _tmp201 + _tmp207;
  const Scalar _tmp270 = _tmp124 * _tmp269;
  const Scalar _tmp271 = _tmp126 * _tmp138;
  const Scalar _tmp272 = _tmp203 * _tmp95;
  const Scalar _tmp273 = _tmp272 * fh1;
  const Scalar _tmp274 = _tmp138 * _tmp139;
  const Scalar _tmp275 = _tmp144 * _tmp145;
  const Scalar _tmp276 = _tmp267 * _tmp99;
  const Scalar _tmp277 = _tmp272 * _tmp99;
  const Scalar _tmp278 = _tmp266 / (_tmp94 * std::sqrt(_tmp94));
  const Scalar _tmp279 = _tmp209 * _tmp96;
  const Scalar _tmp280 =
      _tmp100 * (_tmp202 * _tmp97 - _tmp202 * _tmp98 - _tmp278 * _tmp56 * _tmp61 +
                 _tmp278 * _tmp57 * _tmp60 + _tmp279 * _tmp56 - _tmp279 * _tmp57);
  const Scalar _tmp281 =
      _tmp78 * (-_tmp101 * _tmp235 + _tmp101 * _tmp236 - _tmp230 * _tmp75 - _tmp235 * _tmp68 +
                _tmp236 * _tmp68 + _tmp242 * _tmp71 - _tmp243 * _tmp71 + _tmp243 * _tmp72 +
                _tmp276 * _tmp76 - _tmp277 * _tmp76 + _tmp280 * _tmp76);
  const Scalar _tmp282 = -_tmp101 * _tmp200 + _tmp101 * _tmp206 + _tmp102 * _tmp247 -
                         _tmp103 * _tmp212 - _tmp189 * _tmp55 - _tmp200 * _tmp44 +
                         _tmp205 * _tmp51 + _tmp206 * _tmp44 - _tmp211 * _tmp51 + _tmp211 * _tmp52 +
                         _tmp276 * _tmp64 - _tmp277 * _tmp64 + _tmp280 * _tmp64 - _tmp281 * _tmp66;
  const Scalar _tmp283 = _tmp282 / std::pow(_tmp104, Scalar(2));
  const Scalar _tmp284 = _tmp283 * _tmp92;
  const Scalar _tmp285 = _tmp246 * _tmp62;
  const Scalar _tmp286 = _tmp254 * _tmp78;
  const Scalar _tmp287 = _tmp244 * _tmp78;
  const Scalar _tmp288 =
      -_tmp107 * _tmp204 + _tmp107 * _tmp210 + _tmp129 * _tmp255 - _tmp130 * _tmp252 - _tmp222 -
      _tmp224 + _tmp240 - _tmp285 * _tmp90 + _tmp286 * _tmp62 -
      _tmp89 * (-_tmp204 * _tmp63 - _tmp204 * _tmp82 + _tmp210 * _tmp82 + _tmp227 * _tmp62 +
                _tmp238 - _tmp285 * _tmp81 + _tmp287 * _tmp62);
  const Scalar _tmp289 = _tmp104 * _tmp105;
  const Scalar _tmp290 = _tmp104 * _tmp257;
  const Scalar _tmp291 = _tmp111 * _tmp92;
  const Scalar _tmp292 = -_tmp133 * _tmp284 + _tmp134 * _tmp256 + _tmp288 +
                         _tmp291 * (-_tmp102 * _tmp285 - _tmp103 * _tmp204 + _tmp103 * _tmp210 +
                                    _tmp131 * _tmp290 - _tmp132 * _tmp282 - _tmp276 + _tmp277 -
                                    _tmp280 + _tmp281 * _tmp62 - _tmp288 * _tmp289);
  const Scalar _tmp293 = _tmp257 * _tmp66;
  const Scalar _tmp294 =
      -_tmp114 * _tmp292 - _tmp135 * _tmp264 + _tmp135 * _tmp293 + _tmp204 - _tmp210;
  const Scalar _tmp295 = _tmp173 + _tmp208 + _tmp231;
  const Scalar _tmp296 = (Scalar(1) / Scalar(2)) * (2 * _tmp120 * _tmp295 + 2 * _tmp122 * _tmp269) /
                         (_tmp123 * std::sqrt(_tmp123));
  const Scalar _tmp297 = _tmp122 * _tmp296;
  const Scalar _tmp298 = Scalar(0.5) * _tmp111;
  const Scalar _tmp299 = Scalar(1.0) * _tmp283;
  const Scalar _tmp300 = _tmp120 * _tmp296;
  const Scalar _tmp301 = _tmp124 * _tmp295;
  const Scalar _tmp302 = -_tmp119 * _tmp270 + _tmp119 * _tmp297 - _tmp121 * _tmp300 +
                         _tmp121 * _tmp301 + _tmp125 * _tmp269 - _tmp139 * _tmp295;
  const Scalar _tmp303 = _tmp118 * _tmp126;
  const Scalar _tmp304 = Scalar(1.0) * _tmp246;
  const Scalar _tmp305 = _tmp106 * _tmp253 - _tmp106 * _tmp255 - Scalar(1.0) * _tmp286 +
                         Scalar(1.0) * _tmp287 * _tmp89 - _tmp304 * _tmp81 * _tmp89 +
                         _tmp304 * _tmp90;
  const Scalar _tmp306 = -_tmp110 * _tmp284 + _tmp112 * _tmp256 +
                         _tmp291 * (_tmp102 * _tmp304 + _tmp108 * _tmp290 - _tmp109 * _tmp282 -
                                    Scalar(1.0) * _tmp281 - _tmp289 * _tmp305) +
                         _tmp305;
  const Scalar _tmp307 = _tmp115 * _tmp78;
  const Scalar _tmp308 = -_tmp113 * _tmp264 + _tmp113 * _tmp293 - _tmp114 * _tmp306;
  const Scalar _tmp309 = _tmp118 * _tmp125;
  const Scalar _tmp310 = _tmp267 * fh1;
  const Scalar _tmp311 = _tmp142 * _tmp145;
  const Scalar _tmp312 = _tmp137 * fh1;
  const Scalar _tmp313 = _tmp147 * fh1;
  const Scalar _tmp314 = _tmp150 * _tmp313;
  const Scalar _tmp315 = _tmp307 * fh1;
  const Scalar _tmp316 = _tmp246 * fh1;
  const Scalar _tmp317 = _tmp257 * _tmp311;
  const Scalar _tmp318 = _tmp78 * fh1;
  const Scalar _tmp319 = _tmp151 * _tmp299;
  const Scalar _tmp320 = _tmp302 * _tmp313;
  const Scalar _tmp321 = _tmp105 * _tmp311;
  const Scalar _tmp322 = _tmp212 * _tmp78;
  const Scalar _tmp323 = _tmp105 * fh1;
  const Scalar _tmp324 = _tmp125 * _tmp323;
  const Scalar _tmp325 = _tmp139 * _tmp323;
  const Scalar _tmp326 = _tmp257 * fh1;
  const Scalar _tmp327 = _tmp113 * _tmp323;
  const Scalar _tmp328 = _tmp135 * _tmp323;

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) = 0;
  _res(1, 0) =
      -(-_tmp126 * _tmp149 * _tmp302 -
        _tmp127 * (_tmp113 * _tmp261 - _tmp113 * _tmp262 - _tmp113 * _tmp265 - _tmp115 * _tmp263 +
                   _tmp116 * _tmp308 + _tmp117 * _tmp306 - _tmp235 * _tmp307 + _tmp236 * _tmp307) -
        _tmp140 * (_tmp116 * _tmp294 + _tmp117 * _tmp292 + _tmp135 * _tmp261 - _tmp135 * _tmp262 -
                   _tmp135 * _tmp265 - _tmp136 * _tmp263 - _tmp137 * _tmp235 + _tmp137 * _tmp236) -
        _tmp146 * (_tmp114 * _tmp142 * _tmp263 + _tmp142 * _tmp257 * _tmp258 + _tmp142 * _tmp261 -
                   _tmp142 * _tmp262 - _tmp142 * _tmp265 - _tmp143 * _tmp264 + _tmp235 * _tmp260 -
                   _tmp236 * _tmp260) -
        _tmp152 * (-_tmp116 * _tmp147 * _tmp212 + _tmp147 * _tmp206 + _tmp147 * _tmp247 * _tmp76 -
                   _tmp148 * _tmp236 - _tmp198 * _tmp298 * _tmp52 +
                   _tmp233 * _tmp298 * _tmp72 * _tmp91 + _tmp258 * _tmp299 - _tmp299 * _tmp64) -
        _tmp267 * _tmp268 - _tmp267 * _tmp275 + _tmp268 * _tmp272 - _tmp270 * _tmp271 +
        _tmp271 * _tmp297 + _tmp272 * _tmp275 + _tmp273 * _tmp274 + _tmp273 * _tmp309 -
        _tmp274 * _tmp310 + _tmp300 * _tmp303 - _tmp301 * _tmp303 - _tmp309 * _tmp310) *
      std::exp(_tmp118 * _tmp127 + _tmp138 * _tmp140 + _tmp144 * _tmp146 + _tmp149 * _tmp152);
  _res(2, 0) =
      -(_tmp114 * _tmp246 * _tmp311 - _tmp115 * _tmp125 * _tmp316 + _tmp125 * _tmp308 * _tmp318 -
        _tmp136 * _tmp139 * _tmp316 + _tmp139 * _tmp294 * _tmp318 + _tmp247 * _tmp314 +
        _tmp270 * _tmp312 - _tmp297 * _tmp312 - _tmp300 * _tmp315 + _tmp301 * _tmp315 -
        _tmp314 * _tmp322 + _tmp317 * _tmp91 + _tmp319 * _tmp91 - _tmp320 * _tmp91 -
        _tmp321 * _tmp322) *
      std::exp(-_tmp125 * _tmp315 - _tmp139 * _tmp312 + _tmp259 * _tmp311 + _tmp314 * _tmp91);
  _res(3, 0) = -(-_tmp113 * _tmp125 * _tmp326 - _tmp135 * _tmp139 * _tmp326 + _tmp270 * _tmp328 +
                 _tmp292 * _tmp325 - _tmp297 * _tmp328 - _tmp300 * _tmp327 + _tmp301 * _tmp327 +
                 _tmp306 * _tmp324 - _tmp317 - _tmp319 + _tmp320) *
               std::exp(-_tmp113 * _tmp324 - _tmp135 * _tmp325 - _tmp314 - _tmp321);

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
