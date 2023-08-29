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
 * Symbolic function: IK_residual_func_cost3_wrt_rz_Nl9
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost3WrtRzNl9(
    const Scalar fh1, const Scalar fv1, const Scalar rx, const Scalar ry, const Scalar rz,
    const Scalar p_init0, const Scalar p_init1, const Scalar p_init2, const Scalar rot_init_x,
    const Scalar rot_init_y, const Scalar rot_init_z, const Scalar rot_init_w,
    const Scalar epsilon) {
  // Total ops: 993

  // Unused inputs
  (void)p_init2;
  (void)epsilon;

  // Input arrays

  // Intermediate terms (319)
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
  const Scalar _tmp13 = _tmp4 * rot_init_y;
  const Scalar _tmp14 = _tmp7 * rot_init_x;
  const Scalar _tmp15 = _tmp14 * rz;
  const Scalar _tmp16 = _tmp13 - _tmp15 + _tmp8 * rx + _tmp9 * ry;
  const Scalar _tmp17 = 2 * _tmp16;
  const Scalar _tmp18 = _tmp12 * _tmp17;
  const Scalar _tmp19 = _tmp4 * rot_init_z;
  const Scalar _tmp20 = _tmp9 * rz;
  const Scalar _tmp21 = -_tmp10 * rx + _tmp14 * ry + _tmp19 + _tmp20;
  const Scalar _tmp22 = _tmp4 * rot_init_w;
  const Scalar _tmp23 = _tmp8 * rz;
  const Scalar _tmp24 = -_tmp10 * ry - _tmp14 * rx + _tmp22 - _tmp23;
  const Scalar _tmp25 = 2 * _tmp24;
  const Scalar _tmp26 = _tmp21 * _tmp25;
  const Scalar _tmp27 = Scalar(0.20999999999999999) * _tmp18 + Scalar(0.20999999999999999) * _tmp26;
  const Scalar _tmp28 = _tmp17 * _tmp21;
  const Scalar _tmp29 = _tmp12 * _tmp25;
  const Scalar _tmp30 =
      -Scalar(0.010999999999999999) * _tmp28 + Scalar(0.010999999999999999) * _tmp29;
  const Scalar _tmp31 = -2 * std::pow(_tmp21, Scalar(2));
  const Scalar _tmp32 = 1 - 2 * std::pow(_tmp12, Scalar(2));
  const Scalar _tmp33 = Scalar(0.20999999999999999) * _tmp31 + Scalar(0.20999999999999999) * _tmp32;
  const Scalar _tmp34 = _tmp30 + _tmp33;
  const Scalar _tmp35 = _tmp27 + _tmp34;
  const Scalar _tmp36 = _tmp35 + p_init1 + Scalar(-4.7752063900000001);
  const Scalar _tmp37 = -2 * std::pow(_tmp16, Scalar(2));
  const Scalar _tmp38 = Scalar(0.20999999999999999) * _tmp31 +
                        Scalar(0.20999999999999999) * _tmp37 + Scalar(0.20999999999999999);
  const Scalar _tmp39 = Scalar(0.20999999999999999) * _tmp18 - Scalar(0.20999999999999999) * _tmp26;
  const Scalar _tmp40 = 2 * _tmp12 * _tmp21;
  const Scalar _tmp41 = _tmp16 * _tmp25;
  const Scalar _tmp42 =
      -Scalar(0.010999999999999999) * _tmp40 - Scalar(0.010999999999999999) * _tmp41;
  const Scalar _tmp43 = _tmp39 + _tmp42;
  const Scalar _tmp44 = _tmp38 + _tmp43;
  const Scalar _tmp45 = _tmp44 + p_init0 + Scalar(-2.71799795);
  const Scalar _tmp46 = std::pow(_tmp45, Scalar(2));
  const Scalar _tmp47 = std::pow(_tmp36, Scalar(2)) + _tmp46;
  const Scalar _tmp48 = std::sqrt(_tmp47);
  const Scalar _tmp49 = Scalar(1.0) / (_tmp48);
  const Scalar _tmp50 = _tmp36 * _tmp44;
  const Scalar _tmp51 = _tmp35 * _tmp49;
  const Scalar _tmp52 = -_tmp45 * _tmp51 + _tmp49 * _tmp50;
  const Scalar _tmp53 = Scalar(1.0) / (_tmp45);
  const Scalar _tmp54 = _tmp48 * _tmp53;
  const Scalar _tmp55 = _tmp52 * _tmp54;
  const Scalar _tmp56 = -_tmp27;
  const Scalar _tmp57 = _tmp34 + _tmp56;
  const Scalar _tmp58 = _tmp57 + p_init1 + Scalar(-4.8333311099999996);
  const Scalar _tmp59 = -_tmp38;
  const Scalar _tmp60 = _tmp43 + _tmp59;
  const Scalar _tmp61 = _tmp60 + p_init0 + Scalar(1.79662371);
  const Scalar _tmp62 = std::pow(_tmp58, Scalar(2)) + std::pow(_tmp61, Scalar(2));
  const Scalar _tmp63 = std::pow(_tmp62, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp64 = _tmp61 * _tmp63;
  const Scalar _tmp65 = _tmp58 * _tmp63;
  const Scalar _tmp66 = _tmp55 * _tmp64 + _tmp57 * _tmp64 - _tmp60 * _tmp65;
  const Scalar _tmp67 = _tmp36 * _tmp53;
  const Scalar _tmp68 = _tmp64 * _tmp67 - _tmp65;
  const Scalar _tmp69 = Scalar(1.0) / (_tmp68);
  const Scalar _tmp70 = _tmp67 * _tmp69;
  const Scalar _tmp71 = Scalar(0.20999999999999999) * _tmp40 - Scalar(0.20999999999999999) * _tmp41;
  const Scalar _tmp72 =
      -Scalar(0.010999999999999999) * _tmp32 - Scalar(0.010999999999999999) * _tmp37;
  const Scalar _tmp73 = Scalar(0.20999999999999999) * _tmp28 + Scalar(0.20999999999999999) * _tmp29;
  const Scalar _tmp74 = _tmp72 + _tmp73;
  const Scalar _tmp75 = _tmp71 + _tmp74;
  const Scalar _tmp76 = _tmp64 * _tmp75;
  const Scalar _tmp77 = -_tmp71;
  const Scalar _tmp78 = _tmp74 + _tmp77;
  const Scalar _tmp79 = -_tmp64 * _tmp78 + _tmp76;
  const Scalar _tmp80 = _tmp67 * _tmp75;
  const Scalar _tmp81 = -_tmp64 * _tmp80 + _tmp65 * _tmp78;
  const Scalar _tmp82 = _tmp69 * _tmp81;
  const Scalar _tmp83 = _tmp67 * _tmp82 + _tmp80;
  const Scalar _tmp84 = Scalar(1.0) * _tmp35;
  const Scalar _tmp85 = _tmp57 - _tmp84;
  const Scalar _tmp86 = Scalar(1.0) / (_tmp85);
  const Scalar _tmp87 = Scalar(1.0) * _tmp44;
  const Scalar _tmp88 = -_tmp60 + _tmp87;
  const Scalar _tmp89 = _tmp86 * _tmp88;
  const Scalar _tmp90 = _tmp70 * _tmp79 - _tmp75 - _tmp83 * _tmp89;
  const Scalar _tmp91 = _tmp30 - _tmp33;
  const Scalar _tmp92 = _tmp56 + _tmp91;
  const Scalar _tmp93 = _tmp92 + p_init1 + Scalar(8.3196563700000006);
  const Scalar _tmp94 = -_tmp39 + _tmp42;
  const Scalar _tmp95 = _tmp59 + _tmp94;
  const Scalar _tmp96 = _tmp95 + p_init0 + Scalar(1.9874742000000001);
  const Scalar _tmp97 = std::pow(_tmp93, Scalar(2)) + std::pow(_tmp96, Scalar(2));
  const Scalar _tmp98 = std::pow(_tmp97, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp99 = _tmp93 * _tmp98;
  const Scalar _tmp100 = _tmp96 * _tmp98;
  const Scalar _tmp101 = _tmp100 * _tmp67 - _tmp99;
  const Scalar _tmp102 = _tmp101 * _tmp69;
  const Scalar _tmp103 = _tmp100 * _tmp75;
  const Scalar _tmp104 = _tmp72 - _tmp73 + _tmp77;
  const Scalar _tmp105 = -_tmp102 * _tmp81 - _tmp103 * _tmp67 + _tmp104 * _tmp99;
  const Scalar _tmp106 = -_tmp100 * _tmp104 - _tmp102 * _tmp79 + _tmp103 - _tmp105 * _tmp89;
  const Scalar _tmp107 = Scalar(1.0) / (_tmp106);
  const Scalar _tmp108 = _tmp100 * _tmp55 + _tmp100 * _tmp92 - _tmp102 * _tmp66 - _tmp95 * _tmp99;
  const Scalar _tmp109 = _tmp107 * _tmp108;
  const Scalar _tmp110 = -_tmp109 * _tmp90 - _tmp55 + _tmp66 * _tmp70;
  const Scalar _tmp111 = Scalar(1.0) / (_tmp108);
  const Scalar _tmp112 = _tmp106 * _tmp111;
  const Scalar _tmp113 = _tmp110 * _tmp112 + _tmp90;
  const Scalar _tmp114 = _tmp100 * _tmp107;
  const Scalar _tmp115 = _tmp101 * _tmp107;
  const Scalar _tmp116 = -_tmp113 * _tmp115 - _tmp67;
  const Scalar _tmp117 = _tmp64 * _tmp69;
  const Scalar _tmp118 = _tmp113 * _tmp114 + _tmp116 * _tmp117 + Scalar(1.0);
  const Scalar _tmp119 = _tmp38 + _tmp94;
  const Scalar _tmp120 = _tmp119 + p_init0 + Scalar(-2.5202214700000001);
  const Scalar _tmp121 = _tmp27 + _tmp91;
  const Scalar _tmp122 = _tmp121 + p_init1 + Scalar(8.3888750099999996);
  const Scalar _tmp123 = std::pow(_tmp120, Scalar(2)) + std::pow(_tmp122, Scalar(2));
  const Scalar _tmp124 = std::pow(_tmp123, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp125 = _tmp120 * _tmp124;
  const Scalar _tmp126 = _tmp54 * fh1;
  const Scalar _tmp127 = _tmp125 * _tmp126;
  const Scalar _tmp128 = _tmp122 * _tmp124;
  const Scalar _tmp129 = Scalar(1.0) * _tmp69;
  const Scalar _tmp130 = _tmp129 * _tmp81;
  const Scalar _tmp131 = -_tmp129 * _tmp79 + _tmp130 * _tmp89;
  const Scalar _tmp132 = -_tmp109 * _tmp131 - _tmp129 * _tmp66;
  const Scalar _tmp133 = _tmp112 * _tmp132 + _tmp131;
  const Scalar _tmp134 = -_tmp115 * _tmp133 + Scalar(1.0);
  const Scalar _tmp135 = _tmp114 * _tmp133 + _tmp117 * _tmp134;
  const Scalar _tmp136 = _tmp126 * _tmp135;
  const Scalar _tmp137 = _tmp84 * _tmp89 + _tmp87;
  const Scalar _tmp138 = 0;
  const Scalar _tmp139 = _tmp115 * _tmp138;
  const Scalar _tmp140 = _tmp114 * _tmp138 - _tmp117 * _tmp139;
  const Scalar _tmp141 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp142 = _tmp141 * _tmp54;
  const Scalar _tmp143 = Scalar(1.0) * _tmp111;
  const Scalar _tmp144 = _tmp102 * _tmp143;
  const Scalar _tmp145 = _tmp100 * _tmp143 - _tmp144 * _tmp64;
  const Scalar _tmp146 = _tmp121 * _tmp124;
  const Scalar _tmp147 = fh1 * (_tmp119 * _tmp128 - _tmp120 * _tmp146);
  const Scalar _tmp148 = _tmp147 * _tmp54;
  const Scalar _tmp149 = (Scalar(1) / Scalar(2)) / _tmp1;
  const Scalar _tmp150 = _tmp0 * _tmp149;
  const Scalar _tmp151 = _tmp149 * rz;
  const Scalar _tmp152 = _tmp151 * ry;
  const Scalar _tmp153 = _tmp151 * rx;
  const Scalar _tmp154 = _tmp6 / (_tmp1 * std::sqrt(_tmp1));
  const Scalar _tmp155 = _tmp0 * _tmp154;
  const Scalar _tmp156 = _tmp154 * rz;
  const Scalar _tmp157 = _tmp156 * rx;
  const Scalar _tmp158 = _tmp156 * ry;
  const Scalar _tmp159 = _tmp10 + _tmp13 * _tmp150 - Scalar(1) / Scalar(2) * _tmp15 -
                         _tmp152 * _tmp19 + _tmp153 * _tmp22 - _tmp155 * rot_init_y -
                         _tmp157 * rot_init_w + _tmp158 * rot_init_z;
  const Scalar _tmp160 = Scalar(0.83999999999999997) * _tmp12;
  const Scalar _tmp161 = _tmp159 * _tmp160;
  const Scalar _tmp162 = -_tmp161;
  const Scalar _tmp163 = -_tmp13 * _tmp153 + _tmp150 * _tmp22 + _tmp152 * _tmp5 -
                         _tmp155 * rot_init_w + _tmp157 * rot_init_y - _tmp158 * rot_init_x -
                         Scalar(1) / Scalar(2) * _tmp23 + _tmp9;
  const Scalar _tmp164 = Scalar(0.83999999999999997) * _tmp21;
  const Scalar _tmp165 = _tmp163 * _tmp164;
  const Scalar _tmp166 = -_tmp165;
  const Scalar _tmp167 = -_tmp13 * _tmp152 - _tmp150 * _tmp19 - _tmp153 * _tmp5 +
                         _tmp155 * rot_init_z + _tmp157 * rot_init_x + _tmp158 * rot_init_y -
                         Scalar(1) / Scalar(2) * _tmp20 - _tmp8;
  const Scalar _tmp168 = Scalar(0.41999999999999998) * _tmp167;
  const Scalar _tmp169 = _tmp168 * _tmp21;
  const Scalar _tmp170 = Scalar(0.41999999999999998) * _tmp163;
  const Scalar _tmp171 = _tmp170 * _tmp24;
  const Scalar _tmp172 = _tmp169 + _tmp171;
  const Scalar _tmp173 = _tmp166 + _tmp172;
  const Scalar _tmp174 = Scalar(0.41999999999999998) * _tmp159;
  const Scalar _tmp175 = _tmp16 * _tmp174;
  const Scalar _tmp176 = -Scalar(1) / Scalar(2) * _tmp11 - _tmp14 - _tmp150 * _tmp5 +
                         _tmp152 * _tmp22 + _tmp153 * _tmp19 + _tmp155 * rot_init_x -
                         _tmp157 * rot_init_z - _tmp158 * rot_init_w;
  const Scalar _tmp177 = Scalar(0.41999999999999998) * _tmp176;
  const Scalar _tmp178 = _tmp12 * _tmp177;
  const Scalar _tmp179 = _tmp175 + _tmp178;
  const Scalar _tmp180 = Scalar(0.021999999999999999) * _tmp167;
  const Scalar _tmp181 = Scalar(0.021999999999999999) * _tmp176;
  const Scalar _tmp182 = Scalar(0.021999999999999999) * _tmp163;
  const Scalar _tmp183 = Scalar(0.021999999999999999) * _tmp159;
  const Scalar _tmp184 = _tmp12 * _tmp180 - _tmp16 * _tmp182 - _tmp181 * _tmp21 + _tmp183 * _tmp24;
  const Scalar _tmp185 = _tmp179 + _tmp184;
  const Scalar _tmp186 = _tmp162 + _tmp173 + _tmp185;
  const Scalar _tmp187 = -_tmp169 - _tmp171;
  const Scalar _tmp188 = _tmp166 + _tmp187;
  const Scalar _tmp189 = Scalar(0.83999999999999997) * _tmp16;
  const Scalar _tmp190 = _tmp176 * _tmp189;
  const Scalar _tmp191 = -_tmp12 * _tmp182 - _tmp16 * _tmp180 - _tmp181 * _tmp24 - _tmp183 * _tmp21;
  const Scalar _tmp192 = -_tmp190 + _tmp191;
  const Scalar _tmp193 = _tmp179 + _tmp188 + _tmp192;
  const Scalar _tmp194 = _tmp186 * _tmp36 + _tmp193 * _tmp45;
  const Scalar _tmp195 = _tmp194 * _tmp49 * _tmp53;
  const Scalar _tmp196 = _tmp128 * fh1;
  const Scalar _tmp197 = _tmp135 * _tmp196;
  const Scalar _tmp198 = Scalar(1.0) / (_tmp46);
  const Scalar _tmp199 = _tmp193 * _tmp198 * _tmp48;
  const Scalar _tmp200 = _tmp125 * fh1;
  const Scalar _tmp201 = _tmp118 * _tmp200;
  const Scalar _tmp202 = _tmp190 + _tmp191;
  const Scalar _tmp203 = -_tmp175 - _tmp178;
  const Scalar _tmp204 = _tmp165 + _tmp172;
  const Scalar _tmp205 = _tmp202 + _tmp203 + _tmp204;
  const Scalar _tmp206 = _tmp205 * _tmp98;
  const Scalar _tmp207 = _tmp195 * _tmp52;
  const Scalar _tmp208 = _tmp165 + _tmp187;
  const Scalar _tmp209 = _tmp184 + _tmp203;
  const Scalar _tmp210 = _tmp161 + _tmp208 + _tmp209;
  const Scalar _tmp211 =
      (2 * _tmp205 * _tmp96 + 2 * _tmp210 * _tmp93) / (_tmp97 * std::sqrt(_tmp97));
  const Scalar _tmp212 = (Scalar(1) / Scalar(2)) * _tmp211;
  const Scalar _tmp213 = _tmp212 * _tmp96;
  const Scalar _tmp214 = _tmp212 * _tmp93;
  const Scalar _tmp215 = _tmp199 * _tmp52;
  const Scalar _tmp216 = _tmp210 * _tmp98;
  const Scalar _tmp217 = _tmp186 * _tmp53;
  const Scalar _tmp218 = _tmp193 * _tmp36;
  const Scalar _tmp219 = _tmp198 * _tmp218;
  const Scalar _tmp220 = _tmp100 * _tmp217 - _tmp100 * _tmp219 + _tmp206 * _tmp67 -
                         _tmp213 * _tmp67 + _tmp214 - _tmp216;
  const Scalar _tmp221 = _tmp220 * _tmp69;
  const Scalar _tmp222 = _tmp179 + _tmp202 + _tmp208;
  const Scalar _tmp223 = _tmp162 + _tmp188 + _tmp209;
  const Scalar _tmp224 =
      (2 * _tmp222 * _tmp61 + 2 * _tmp223 * _tmp58) / (_tmp62 * std::sqrt(_tmp62));
  const Scalar _tmp225 = (Scalar(1) / Scalar(2)) * _tmp224;
  const Scalar _tmp226 = _tmp225 * _tmp58;
  const Scalar _tmp227 = _tmp222 * _tmp63;
  const Scalar _tmp228 = _tmp225 * _tmp61;
  const Scalar _tmp229 = _tmp223 * _tmp63;
  const Scalar _tmp230 = _tmp194 / (_tmp47 * std::sqrt(_tmp47));
  const Scalar _tmp231 = _tmp186 * _tmp49;
  const Scalar _tmp232 =
      _tmp54 * (-_tmp193 * _tmp51 + _tmp218 * _tmp49 + _tmp230 * _tmp35 * _tmp45 -
                _tmp230 * _tmp50 + _tmp231 * _tmp44 - _tmp231 * _tmp45);
  const Scalar _tmp233 =
      _tmp69 * (_tmp207 * _tmp64 - _tmp215 * _tmp64 - _tmp222 * _tmp65 + _tmp226 * _tmp60 +
                _tmp227 * _tmp55 + _tmp227 * _tmp57 - _tmp228 * _tmp55 - _tmp228 * _tmp57 -
                _tmp229 * _tmp60 + _tmp229 * _tmp61 + _tmp232 * _tmp64);
  const Scalar _tmp234 = (_tmp217 * _tmp64 - _tmp219 * _tmp64 + _tmp226 + _tmp227 * _tmp67 -
                          _tmp228 * _tmp67 - _tmp229) /
                         std::pow(_tmp68, Scalar(2));
  const Scalar _tmp235 = _tmp101 * _tmp234;
  const Scalar _tmp236 = _tmp100 * _tmp207 - _tmp100 * _tmp215 + _tmp100 * _tmp232 -
                         _tmp101 * _tmp233 - _tmp205 * _tmp99 + _tmp206 * _tmp55 +
                         _tmp206 * _tmp92 - _tmp213 * _tmp55 - _tmp213 * _tmp92 + _tmp214 * _tmp95 -
                         _tmp216 * _tmp95 + _tmp216 * _tmp96 - _tmp221 * _tmp66 + _tmp235 * _tmp66;
  const Scalar _tmp237 = _tmp236 / std::pow(_tmp108, Scalar(2));
  const Scalar _tmp238 = Scalar(1.0) * _tmp237;
  const Scalar _tmp239 = Scalar(0.5) * _tmp111;
  const Scalar _tmp240 = _tmp221 * _tmp64;
  const Scalar _tmp241 = _tmp102 * _tmp64;
  const Scalar _tmp242 = _tmp173 + _tmp192 + _tmp203;
  const Scalar _tmp243 = _tmp124 * _tmp242;
  const Scalar _tmp244 = _tmp118 * _tmp126;
  const Scalar _tmp245 = _tmp16 * _tmp168;
  const Scalar _tmp246 = _tmp174 * _tmp21;
  const Scalar _tmp247 = -_tmp246;
  const Scalar _tmp248 = _tmp12 * _tmp170;
  const Scalar _tmp249 = -_tmp248;
  const Scalar _tmp250 = _tmp177 * _tmp24;
  const Scalar _tmp251 = Scalar(0.043999999999999997) * _tmp12 * _tmp159;
  const Scalar _tmp252 = _tmp16 * _tmp176;
  const Scalar _tmp253 = Scalar(0.043999999999999997) * _tmp252;
  const Scalar _tmp254 = _tmp251 + _tmp253;
  const Scalar _tmp255 = _tmp245 + _tmp247 + _tmp249 + _tmp250 + _tmp254;
  const Scalar _tmp256 = _tmp12 * _tmp168;
  const Scalar _tmp257 = _tmp177 * _tmp21;
  const Scalar _tmp258 = _tmp16 * _tmp170;
  const Scalar _tmp259 = _tmp174 * _tmp24;
  const Scalar _tmp260 = -_tmp256 - _tmp257 - _tmp258 - _tmp259;
  const Scalar _tmp261 = _tmp255 + _tmp260;
  const Scalar _tmp262 = _tmp256 + _tmp257 + _tmp258 + _tmp259;
  const Scalar _tmp263 = -_tmp245 + _tmp246 + _tmp248 - _tmp250 + _tmp254 + _tmp262;
  const Scalar _tmp264 = _tmp100 * _tmp263;
  const Scalar _tmp265 = _tmp86 * (-Scalar(1.6799999999999999) * _tmp163 * _tmp21 -
                                   Scalar(1.6799999999999999) * _tmp252);
  const Scalar _tmp266 = _tmp88 *
                         (-_tmp159 * _tmp189 - _tmp160 * _tmp176 -
                          Scalar(0.83999999999999997) * _tmp163 * _tmp24 - _tmp164 * _tmp167) /
                         std::pow(_tmp85, Scalar(2));
  const Scalar _tmp267 = _tmp227 * _tmp75;
  const Scalar _tmp268 = _tmp263 * _tmp64;
  const Scalar _tmp269 = _tmp255 + _tmp262;
  const Scalar _tmp270 = -_tmp217 * _tmp76 + _tmp219 * _tmp76 - _tmp226 * _tmp78 +
                         _tmp228 * _tmp80 + _tmp229 * _tmp78 - _tmp267 * _tmp67 - _tmp268 * _tmp67 +
                         _tmp269 * _tmp65;
  const Scalar _tmp271 = -_tmp227 * _tmp78 - _tmp228 * _tmp75 + _tmp228 * _tmp78 + _tmp267 +
                         _tmp268 - _tmp269 * _tmp64;
  const Scalar _tmp272 =
      -_tmp100 * _tmp261 - _tmp102 * _tmp271 - _tmp104 * _tmp206 + _tmp104 * _tmp213 -
      _tmp105 * _tmp265 + _tmp105 * _tmp266 + _tmp206 * _tmp75 - _tmp213 * _tmp75 -
      _tmp221 * _tmp79 + _tmp235 * _tmp79 + _tmp264 -
      _tmp89 * (-_tmp102 * _tmp270 - _tmp103 * _tmp217 + _tmp103 * _tmp219 - _tmp104 * _tmp214 +
                _tmp104 * _tmp216 - _tmp206 * _tmp80 + _tmp213 * _tmp80 - _tmp221 * _tmp81 +
                _tmp235 * _tmp81 + _tmp261 * _tmp99 - _tmp264 * _tmp67);
  const Scalar _tmp273 = _tmp272 / std::pow(_tmp106, Scalar(2));
  const Scalar _tmp274 = _tmp107 * _tmp213;
  const Scalar _tmp275 = _tmp107 * _tmp206;
  const Scalar _tmp276 = _tmp100 * _tmp273;
  const Scalar _tmp277 = _tmp227 * _tmp69;
  const Scalar _tmp278 = _tmp234 * _tmp64;
  const Scalar _tmp279 = _tmp228 * _tmp69;
  const Scalar _tmp280 = _tmp145 * _tmp147;
  const Scalar _tmp281 = _tmp161 + _tmp185 + _tmp204;
  const Scalar _tmp282 = (Scalar(1) / Scalar(2)) * (2 * _tmp120 * _tmp242 + 2 * _tmp122 * _tmp281) /
                         (_tmp123 * std::sqrt(_tmp123));
  const Scalar _tmp283 = _tmp122 * _tmp282;
  const Scalar _tmp284 = _tmp120 * _tmp282;
  const Scalar _tmp285 = Scalar(1.0) * _tmp234;
  const Scalar _tmp286 = _tmp108 * _tmp273;
  const Scalar _tmp287 = _tmp129 * _tmp270 * _tmp89 - _tmp129 * _tmp271 + _tmp130 * _tmp265 -
                         _tmp130 * _tmp266 + _tmp285 * _tmp79 - _tmp285 * _tmp81 * _tmp89;
  const Scalar _tmp288 = _tmp107 * _tmp236;
  const Scalar _tmp289 = _tmp111 * _tmp272;
  const Scalar _tmp290 = _tmp106 * _tmp237;
  const Scalar _tmp291 = _tmp112 * (-_tmp109 * _tmp287 + _tmp131 * _tmp286 - _tmp131 * _tmp288 -
                                    Scalar(1.0) * _tmp233 + _tmp285 * _tmp66) +
                         _tmp132 * _tmp289 - _tmp132 * _tmp290 + _tmp287;
  const Scalar _tmp292 = _tmp101 * _tmp273;
  const Scalar _tmp293 = _tmp107 * _tmp220;
  const Scalar _tmp294 = -_tmp115 * _tmp291 + _tmp133 * _tmp292 - _tmp133 * _tmp293;
  const Scalar _tmp295 = _tmp140 * _tmp141;
  const Scalar _tmp296 = _tmp124 * _tmp281;
  const Scalar _tmp297 = fh1 * (-_tmp119 * _tmp283 + _tmp119 * _tmp296 - _tmp120 * _tmp296 +
                                _tmp121 * _tmp284 + _tmp128 * _tmp242 - _tmp146 * _tmp242);
  const Scalar _tmp298 = _tmp234 * _tmp67;
  const Scalar _tmp299 = _tmp219 * _tmp69;
  const Scalar _tmp300 = _tmp217 * _tmp69;
  const Scalar _tmp301 =
      _tmp245 + _tmp247 + _tmp249 + _tmp250 - _tmp251 - _tmp253 + _tmp260 - _tmp265 * _tmp83 +
      _tmp266 * _tmp83 + _tmp271 * _tmp70 - _tmp298 * _tmp79 - _tmp299 * _tmp79 + _tmp300 * _tmp79 -
      _tmp89 * (_tmp217 * _tmp75 + _tmp217 * _tmp82 - _tmp219 * _tmp75 - _tmp219 * _tmp82 +
                _tmp263 * _tmp67 + _tmp270 * _tmp70 - _tmp298 * _tmp81);
  const Scalar _tmp302 = _tmp110 * _tmp289 - _tmp110 * _tmp290 +
                         _tmp112 * (-_tmp109 * _tmp301 - _tmp207 + _tmp215 - _tmp232 +
                                    _tmp233 * _tmp67 + _tmp286 * _tmp90 - _tmp288 * _tmp90 -
                                    _tmp298 * _tmp66 - _tmp299 * _tmp66 + _tmp300 * _tmp66) +
                         _tmp301;
  const Scalar _tmp303 =
      _tmp113 * _tmp292 - _tmp113 * _tmp293 - _tmp115 * _tmp302 - _tmp217 + _tmp219;
  const Scalar _tmp304 = _tmp143 * _tmp147;
  const Scalar _tmp305 = _tmp138 * _tmp141;
  const Scalar _tmp306 = _tmp115 * _tmp305;
  const Scalar _tmp307 = _tmp69 * fh1;
  const Scalar _tmp308 = _tmp128 * _tmp307;
  const Scalar _tmp309 = _tmp125 * _tmp307;
  const Scalar _tmp310 = _tmp143 * _tmp297;
  const Scalar _tmp311 = _tmp273 * _tmp305;
  const Scalar _tmp312 = _tmp116 * _tmp307;
  const Scalar _tmp313 = _tmp134 * _tmp307;
  const Scalar _tmp314 = _tmp107 * _tmp305;
  const Scalar _tmp315 = _tmp147 * _tmp238;
  const Scalar _tmp316 = _tmp107 * fh1;
  const Scalar _tmp317 = _tmp133 * _tmp316;
  const Scalar _tmp318 = _tmp113 * _tmp316;

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) = 0;
  _res(1, 0) =
      -(-_tmp126 * _tmp128 *
            (_tmp114 * _tmp291 + _tmp117 * _tmp294 - _tmp133 * _tmp274 + _tmp133 * _tmp275 -
             _tmp133 * _tmp276 + _tmp134 * _tmp277 - _tmp134 * _tmp278 - _tmp134 * _tmp279) -
        _tmp127 * (-_tmp113 * _tmp274 + _tmp113 * _tmp275 - _tmp113 * _tmp276 + _tmp114 * _tmp302 +
                   _tmp116 * _tmp277 - _tmp116 * _tmp278 - _tmp116 * _tmp279 + _tmp117 * _tmp303) +
        _tmp136 * _tmp283 - _tmp136 * _tmp296 -
        _tmp142 * (-_tmp107 * _tmp138 * _tmp240 + _tmp138 * _tmp241 * _tmp273 - _tmp138 * _tmp274 +
                   _tmp138 * _tmp275 - _tmp138 * _tmp276 - _tmp139 * _tmp277 + _tmp139 * _tmp278 +
                   _tmp139 * _tmp279) -
        _tmp145 * _tmp297 * _tmp54 -
        _tmp148 * (-_tmp100 * _tmp238 + _tmp102 * _tmp224 * _tmp239 * _tmp61 + _tmp143 * _tmp206 +
                   _tmp143 * _tmp235 * _tmp64 - _tmp143 * _tmp240 - _tmp144 * _tmp227 -
                   _tmp211 * _tmp239 * _tmp96 + _tmp238 * _tmp241) -
        _tmp195 * _tmp197 - _tmp195 * _tmp201 - _tmp195 * _tmp280 - _tmp195 * _tmp295 +
        _tmp197 * _tmp199 + _tmp199 * _tmp201 + _tmp199 * _tmp280 + _tmp199 * _tmp295 -
        _tmp243 * _tmp244 + _tmp244 * _tmp284) *
      std::exp(_tmp118 * _tmp127 + _tmp128 * _tmp136 + _tmp140 * _tmp142 + _tmp145 * _tmp148);
  _res(2, 0) =
      -(-_tmp102 * _tmp310 + _tmp102 * _tmp311 + _tmp102 * _tmp315 - _tmp116 * _tmp200 * _tmp234 -
        _tmp134 * _tmp196 * _tmp234 - _tmp221 * _tmp304 - _tmp221 * _tmp314 + _tmp234 * _tmp306 +
        _tmp235 * _tmp304 + _tmp243 * _tmp312 - _tmp283 * _tmp313 - _tmp284 * _tmp312 +
        _tmp294 * _tmp308 + _tmp296 * _tmp313 + _tmp303 * _tmp309) *
      std::exp(_tmp102 * _tmp304 - _tmp116 * _tmp309 - _tmp134 * _tmp308 + _tmp306 * _tmp69);
  _res(3, 0) =
      -(-_tmp113 * _tmp200 * _tmp273 + _tmp125 * _tmp302 * _tmp316 + _tmp128 * _tmp291 * _tmp316 -
        _tmp133 * _tmp196 * _tmp273 + _tmp243 * _tmp318 - _tmp283 * _tmp317 - _tmp284 * _tmp318 +
        _tmp296 * _tmp317 + _tmp310 - _tmp311 - _tmp315) *
      std::exp(-_tmp125 * _tmp318 - _tmp128 * _tmp317 - _tmp304 - _tmp314);

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
