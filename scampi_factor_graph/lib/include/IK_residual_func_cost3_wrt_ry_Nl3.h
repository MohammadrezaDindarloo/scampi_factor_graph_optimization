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
 * Symbolic function: IK_residual_func_cost3_wrt_ry_Nl3
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost3WrtRyNl3(
    const Scalar fh1, const Scalar fv1, const Scalar rx, const Scalar ry, const Scalar rz,
    const Scalar p_init0, const Scalar p_init1, const Scalar p_init2, const Scalar rot_init_x,
    const Scalar rot_init_y, const Scalar rot_init_z, const Scalar rot_init_w,
    const Scalar epsilon) {
  // Total ops: 992

  // Unused inputs
  (void)p_init2;
  (void)epsilon;

  // Input arrays

  // Intermediate terms (320)
  const Scalar _tmp0 = std::pow(ry, Scalar(2));
  const Scalar _tmp1 = _tmp0 + std::pow(rx, Scalar(2)) + std::pow(rz, Scalar(2));
  const Scalar _tmp2 = std::sqrt(_tmp1);
  const Scalar _tmp3 = (Scalar(1) / Scalar(2)) * _tmp2;
  const Scalar _tmp4 = std::cos(_tmp3);
  const Scalar _tmp5 = _tmp4 * rot_init_y;
  const Scalar _tmp6 = std::sin(_tmp3);
  const Scalar _tmp7 = _tmp6 / _tmp2;
  const Scalar _tmp8 = _tmp7 * rot_init_w;
  const Scalar _tmp9 = _tmp8 * ry;
  const Scalar _tmp10 = _tmp7 * rot_init_z;
  const Scalar _tmp11 = _tmp7 * rot_init_x;
  const Scalar _tmp12 = _tmp10 * rx - _tmp11 * rz + _tmp5 + _tmp9;
  const Scalar _tmp13 = -2 * std::pow(_tmp12, Scalar(2));
  const Scalar _tmp14 = _tmp4 * rot_init_z;
  const Scalar _tmp15 = _tmp11 * ry;
  const Scalar _tmp16 = _tmp7 * rot_init_y;
  const Scalar _tmp17 = _tmp14 + _tmp15 - _tmp16 * rx + _tmp8 * rz;
  const Scalar _tmp18 = -2 * std::pow(_tmp17, Scalar(2));
  const Scalar _tmp19 = Scalar(0.20999999999999999) * _tmp13 +
                        Scalar(0.20999999999999999) * _tmp18 + Scalar(0.20999999999999999);
  const Scalar _tmp20 = _tmp4 * rot_init_x;
  const Scalar _tmp21 = _tmp10 * ry;
  const Scalar _tmp22 = _tmp16 * rz + _tmp20 - _tmp21 + _tmp8 * rx;
  const Scalar _tmp23 = 2 * _tmp22;
  const Scalar _tmp24 = _tmp17 * _tmp23;
  const Scalar _tmp25 = _tmp4 * rot_init_w;
  const Scalar _tmp26 = _tmp16 * ry;
  const Scalar _tmp27 = -_tmp10 * rz - _tmp11 * rx + _tmp25 - _tmp26;
  const Scalar _tmp28 = 2 * _tmp27;
  const Scalar _tmp29 = _tmp12 * _tmp28;
  const Scalar _tmp30 =
      -Scalar(0.010999999999999999) * _tmp24 - Scalar(0.010999999999999999) * _tmp29;
  const Scalar _tmp31 = _tmp12 * _tmp23;
  const Scalar _tmp32 = _tmp17 * _tmp28;
  const Scalar _tmp33 = Scalar(0.20999999999999999) * _tmp31 - Scalar(0.20999999999999999) * _tmp32;
  const Scalar _tmp34 = _tmp30 + _tmp33;
  const Scalar _tmp35 = _tmp19 + _tmp34;
  const Scalar _tmp36 = _tmp35 + p_init0 + Scalar(-2.71799795);
  const Scalar _tmp37 = Scalar(1.0) / (_tmp36);
  const Scalar _tmp38 = Scalar(0.20999999999999999) * _tmp31 + Scalar(0.20999999999999999) * _tmp32;
  const Scalar _tmp39 = 2 * _tmp12 * _tmp17;
  const Scalar _tmp40 = _tmp22 * _tmp28;
  const Scalar _tmp41 =
      -Scalar(0.010999999999999999) * _tmp39 + Scalar(0.010999999999999999) * _tmp40;
  const Scalar _tmp42 = 1 - 2 * std::pow(_tmp22, Scalar(2));
  const Scalar _tmp43 = Scalar(0.20999999999999999) * _tmp18 + Scalar(0.20999999999999999) * _tmp42;
  const Scalar _tmp44 = _tmp41 + _tmp43;
  const Scalar _tmp45 = _tmp38 + _tmp44;
  const Scalar _tmp46 = _tmp45 + p_init1 + Scalar(-4.7752063900000001);
  const Scalar _tmp47 = _tmp37 * _tmp46;
  const Scalar _tmp48 = Scalar(0.20999999999999999) * _tmp24 - Scalar(0.20999999999999999) * _tmp29;
  const Scalar _tmp49 =
      -Scalar(0.010999999999999999) * _tmp13 - Scalar(0.010999999999999999) * _tmp42;
  const Scalar _tmp50 = Scalar(0.20999999999999999) * _tmp39 + Scalar(0.20999999999999999) * _tmp40;
  const Scalar _tmp51 = _tmp49 + _tmp50;
  const Scalar _tmp52 = _tmp48 + _tmp51;
  const Scalar _tmp53 = -_tmp38;
  const Scalar _tmp54 = _tmp44 + _tmp53;
  const Scalar _tmp55 = _tmp54 + p_init1 + Scalar(-4.8333311099999996);
  const Scalar _tmp56 = -_tmp19;
  const Scalar _tmp57 = _tmp34 + _tmp56;
  const Scalar _tmp58 = _tmp57 + p_init0 + Scalar(1.79662371);
  const Scalar _tmp59 = std::pow(_tmp55, Scalar(2)) + std::pow(_tmp58, Scalar(2));
  const Scalar _tmp60 = std::pow(_tmp59, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp61 = _tmp58 * _tmp60;
  const Scalar _tmp62 = _tmp52 * _tmp61;
  const Scalar _tmp63 = -_tmp48 + _tmp51;
  const Scalar _tmp64 = -_tmp61 * _tmp63 + _tmp62;
  const Scalar _tmp65 = _tmp55 * _tmp60;
  const Scalar _tmp66 = _tmp47 * _tmp61 - _tmp65;
  const Scalar _tmp67 = Scalar(1.0) / (_tmp66);
  const Scalar _tmp68 = _tmp30 - _tmp33;
  const Scalar _tmp69 = _tmp19 + _tmp68;
  const Scalar _tmp70 = _tmp69 + p_init0 + Scalar(-2.5202214700000001);
  const Scalar _tmp71 = _tmp41 - _tmp43;
  const Scalar _tmp72 = _tmp38 + _tmp71;
  const Scalar _tmp73 = _tmp72 + p_init1 + Scalar(8.3888750099999996);
  const Scalar _tmp74 = std::pow(_tmp70, Scalar(2)) + std::pow(_tmp73, Scalar(2));
  const Scalar _tmp75 = std::pow(_tmp74, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp76 = _tmp70 * _tmp75;
  const Scalar _tmp77 = _tmp73 * _tmp75;
  const Scalar _tmp78 = _tmp47 * _tmp76 - _tmp77;
  const Scalar _tmp79 = _tmp67 * _tmp78;
  const Scalar _tmp80 = _tmp48 + _tmp49 - _tmp50;
  const Scalar _tmp81 = _tmp52 * _tmp76;
  const Scalar _tmp82 = -_tmp47 * _tmp62 + _tmp63 * _tmp65;
  const Scalar _tmp83 = -_tmp47 * _tmp81 + _tmp77 * _tmp80 - _tmp79 * _tmp82;
  const Scalar _tmp84 = Scalar(1.0) * _tmp45;
  const Scalar _tmp85 = _tmp54 - _tmp84;
  const Scalar _tmp86 = Scalar(1.0) / (_tmp85);
  const Scalar _tmp87 = Scalar(1.0) * _tmp35;
  const Scalar _tmp88 = -_tmp57 + _tmp87;
  const Scalar _tmp89 = _tmp86 * _tmp88;
  const Scalar _tmp90 = -_tmp64 * _tmp79 - _tmp76 * _tmp80 + _tmp81 - _tmp83 * _tmp89;
  const Scalar _tmp91 = std::pow(_tmp36, Scalar(2));
  const Scalar _tmp92 = std::pow(_tmp46, Scalar(2)) + _tmp91;
  const Scalar _tmp93 = std::sqrt(_tmp92);
  const Scalar _tmp94 = Scalar(1.0) / (_tmp93);
  const Scalar _tmp95 = _tmp46 * _tmp94;
  const Scalar _tmp96 = _tmp36 * _tmp94;
  const Scalar _tmp97 = _tmp35 * _tmp95 - _tmp45 * _tmp96;
  const Scalar _tmp98 = _tmp37 * _tmp93;
  const Scalar _tmp99 = _tmp97 * _tmp98;
  const Scalar _tmp100 = _tmp54 * _tmp61 - _tmp57 * _tmp65 + _tmp61 * _tmp99;
  const Scalar _tmp101 = -_tmp100 * _tmp79 - _tmp69 * _tmp77 + _tmp72 * _tmp76 + _tmp76 * _tmp99;
  const Scalar _tmp102 = Scalar(1.0) / (_tmp90);
  const Scalar _tmp103 = _tmp47 * _tmp67;
  const Scalar _tmp104 = _tmp47 * _tmp52;
  const Scalar _tmp105 = _tmp103 * _tmp82 + _tmp104;
  const Scalar _tmp106 = _tmp103 * _tmp64 - _tmp105 * _tmp89 - _tmp52;
  const Scalar _tmp107 = _tmp102 * _tmp106;
  const Scalar _tmp108 = _tmp100 * _tmp103 - _tmp101 * _tmp107 - _tmp99;
  const Scalar _tmp109 = Scalar(1.0) / (_tmp101);
  const Scalar _tmp110 = _tmp108 * _tmp109;
  const Scalar _tmp111 = _tmp106 + _tmp110 * _tmp90;
  const Scalar _tmp112 = _tmp102 * _tmp78;
  const Scalar _tmp113 = -_tmp111 * _tmp112 - _tmp47;
  const Scalar _tmp114 = _tmp61 * _tmp67;
  const Scalar _tmp115 = _tmp102 * _tmp76;
  const Scalar _tmp116 = _tmp111 * _tmp115 + _tmp113 * _tmp114 + Scalar(1.0);
  const Scalar _tmp117 = _tmp56 + _tmp68;
  const Scalar _tmp118 = _tmp117 + p_init0 + Scalar(1.9874742000000001);
  const Scalar _tmp119 = _tmp53 + _tmp71;
  const Scalar _tmp120 = _tmp119 + p_init1 + Scalar(8.3196563700000006);
  const Scalar _tmp121 = std::pow(_tmp118, Scalar(2)) + std::pow(_tmp120, Scalar(2));
  const Scalar _tmp122 = std::pow(_tmp121, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp123 = _tmp118 * _tmp122;
  const Scalar _tmp124 = _tmp123 * fh1;
  const Scalar _tmp125 = _tmp124 * _tmp98;
  const Scalar _tmp126 = _tmp84 * _tmp89 + _tmp87;
  const Scalar _tmp127 = 0;
  const Scalar _tmp128 = _tmp102 * _tmp127;
  const Scalar _tmp129 = _tmp128 * _tmp79;
  const Scalar _tmp130 = _tmp128 * _tmp76 - _tmp129 * _tmp61;
  const Scalar _tmp131 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp132 = _tmp131 * _tmp98;
  const Scalar _tmp133 = _tmp120 * _tmp122;
  const Scalar _tmp134 = _tmp117 * _tmp133 - _tmp119 * _tmp123;
  const Scalar _tmp135 = Scalar(1.0) * _tmp109;
  const Scalar _tmp136 = _tmp135 * _tmp79;
  const Scalar _tmp137 = _tmp135 * _tmp76 - _tmp136 * _tmp61;
  const Scalar _tmp138 = _tmp98 * fh1;
  const Scalar _tmp139 = _tmp137 * _tmp138;
  const Scalar _tmp140 = Scalar(1.0) * _tmp67;
  const Scalar _tmp141 = _tmp140 * _tmp82;
  const Scalar _tmp142 = -_tmp140 * _tmp64 + _tmp141 * _tmp89;
  const Scalar _tmp143 = _tmp101 * _tmp102;
  const Scalar _tmp144 = -_tmp100 * _tmp140 - _tmp142 * _tmp143;
  const Scalar _tmp145 = _tmp109 * _tmp90;
  const Scalar _tmp146 = _tmp142 + _tmp144 * _tmp145;
  const Scalar _tmp147 = -_tmp112 * _tmp146 + Scalar(1.0);
  const Scalar _tmp148 = _tmp114 * _tmp147 + _tmp115 * _tmp146;
  const Scalar _tmp149 = _tmp133 * fh1;
  const Scalar _tmp150 = _tmp149 * _tmp98;
  const Scalar _tmp151 = (Scalar(1) / Scalar(2)) / _tmp1;
  const Scalar _tmp152 = _tmp0 * _tmp151;
  const Scalar _tmp153 = _tmp151 * ry;
  const Scalar _tmp154 = _tmp153 * rz;
  const Scalar _tmp155 = _tmp153 * rx;
  const Scalar _tmp156 = _tmp6 / (_tmp1 * std::sqrt(_tmp1));
  const Scalar _tmp157 = _tmp0 * _tmp156;
  const Scalar _tmp158 = _tmp156 * ry;
  const Scalar _tmp159 = _tmp158 * rx;
  const Scalar _tmp160 = _tmp158 * rz;
  const Scalar _tmp161 = _tmp11 + _tmp152 * _tmp20 + _tmp154 * _tmp25 - _tmp155 * _tmp5 -
                         _tmp157 * rot_init_x + _tmp159 * rot_init_y - _tmp160 * rot_init_w -
                         Scalar(1) / Scalar(2) * _tmp21;
  const Scalar _tmp162 = Scalar(0.41999999999999998) * _tmp27;
  const Scalar _tmp163 = _tmp161 * _tmp162;
  const Scalar _tmp164 = -_tmp14 * _tmp154 - _tmp152 * _tmp5 - _tmp155 * _tmp20 +
                         _tmp157 * rot_init_y + _tmp159 * rot_init_x - _tmp16 +
                         _tmp160 * rot_init_z - Scalar(1) / Scalar(2) * _tmp9;
  const Scalar _tmp165 = Scalar(0.41999999999999998) * _tmp17;
  const Scalar _tmp166 = _tmp164 * _tmp165;
  const Scalar _tmp167 = -_tmp163 - _tmp166;
  const Scalar _tmp168 = Scalar(0.83999999999999997) * _tmp161;
  const Scalar _tmp169 = _tmp168 * _tmp17;
  const Scalar _tmp170 = -_tmp169;
  const Scalar _tmp171 = _tmp14 * _tmp155 + _tmp152 * _tmp25 - _tmp154 * _tmp20 -
                         _tmp157 * rot_init_w - _tmp159 * rot_init_z + _tmp160 * rot_init_x -
                         Scalar(1) / Scalar(2) * _tmp26 + _tmp8;
  const Scalar _tmp172 = Scalar(0.41999999999999998) * _tmp171;
  const Scalar _tmp173 = _tmp172 * _tmp22;
  const Scalar _tmp174 = -_tmp10 - _tmp14 * _tmp152 - Scalar(1) / Scalar(2) * _tmp15 +
                         _tmp154 * _tmp5 + _tmp155 * _tmp25 + _tmp157 * rot_init_z -
                         _tmp159 * rot_init_w - _tmp160 * rot_init_y;
  const Scalar _tmp175 = Scalar(0.41999999999999998) * _tmp12;
  const Scalar _tmp176 = _tmp174 * _tmp175;
  const Scalar _tmp177 = _tmp173 + _tmp176;
  const Scalar _tmp178 = _tmp170 + _tmp177;
  const Scalar _tmp179 = _tmp12 * _tmp171;
  const Scalar _tmp180 = Scalar(0.83999999999999997) * _tmp179;
  const Scalar _tmp181 = Scalar(0.021999999999999999) * _tmp27;
  const Scalar _tmp182 = Scalar(0.021999999999999999) * _tmp164;
  const Scalar _tmp183 = Scalar(0.021999999999999999) * _tmp161;
  const Scalar _tmp184 = Scalar(0.021999999999999999) * _tmp17;
  const Scalar _tmp185 =
      -_tmp12 * _tmp182 - _tmp171 * _tmp181 - _tmp174 * _tmp184 - _tmp183 * _tmp22;
  const Scalar _tmp186 = -_tmp180 + _tmp185;
  const Scalar _tmp187 = _tmp167 + _tmp178 + _tmp186;
  const Scalar _tmp188 = _tmp187 / _tmp91;
  const Scalar _tmp189 = _tmp188 * _tmp93;
  const Scalar _tmp190 = _tmp134 * fh1;
  const Scalar _tmp191 = _tmp137 * _tmp190;
  const Scalar _tmp192 = _tmp163 + _tmp166;
  const Scalar _tmp193 = Scalar(0.83999999999999997) * _tmp22;
  const Scalar _tmp194 = _tmp174 * _tmp193;
  const Scalar _tmp195 =
      -_tmp12 * _tmp183 - _tmp171 * _tmp184 + _tmp174 * _tmp181 + _tmp182 * _tmp22;
  const Scalar _tmp196 = -_tmp194 + _tmp195;
  const Scalar _tmp197 = _tmp178 + _tmp192 + _tmp196;
  const Scalar _tmp198 = _tmp187 * _tmp36 + _tmp197 * _tmp46;
  const Scalar _tmp199 = _tmp198 * _tmp37 * _tmp94;
  const Scalar _tmp200 = _tmp116 * _tmp124;
  const Scalar _tmp201 = _tmp180 + _tmp185;
  const Scalar _tmp202 = _tmp167 + _tmp169;
  const Scalar _tmp203 = _tmp177 + _tmp201 + _tmp202;
  const Scalar _tmp204 = -_tmp173 - _tmp176;
  const Scalar _tmp205 = _tmp170 + _tmp204;
  const Scalar _tmp206 = _tmp167 + _tmp196 + _tmp205;
  const Scalar _tmp207 =
      (2 * _tmp203 * _tmp58 + 2 * _tmp206 * _tmp55) / (_tmp59 * std::sqrt(_tmp59));
  const Scalar _tmp208 = (Scalar(1) / Scalar(2)) * _tmp207;
  const Scalar _tmp209 = _tmp208 * _tmp58;
  const Scalar _tmp210 = _tmp113 * _tmp67;
  const Scalar _tmp211 = _tmp169 + _tmp192;
  const Scalar _tmp212 = _tmp194 + _tmp195;
  const Scalar _tmp213 = _tmp177 + _tmp211 + _tmp212;
  const Scalar _tmp214 = _tmp186 + _tmp192 + _tmp205;
  const Scalar _tmp215 =
      (2 * _tmp213 * _tmp73 + 2 * _tmp214 * _tmp70) / (_tmp74 * std::sqrt(_tmp74));
  const Scalar _tmp216 = (Scalar(1) / Scalar(2)) * _tmp215;
  const Scalar _tmp217 = _tmp216 * _tmp70;
  const Scalar _tmp218 = _tmp102 * _tmp111;
  const Scalar _tmp219 = _tmp214 * _tmp75;
  const Scalar _tmp220 = _tmp219 * _tmp52;
  const Scalar _tmp221 =
      _tmp88 *
      (-Scalar(0.83999999999999997) * _tmp12 * _tmp174 -
       Scalar(0.83999999999999997) * _tmp164 * _tmp17 - _tmp168 * _tmp27 - _tmp171 * _tmp193) /
      std::pow(_tmp85, Scalar(2));
  const Scalar _tmp222 = _tmp197 * _tmp37;
  const Scalar _tmp223 = _tmp216 * _tmp73;
  const Scalar _tmp224 = _tmp188 * _tmp46;
  const Scalar _tmp225 = _tmp213 * _tmp75;
  const Scalar _tmp226 = -_tmp217 * _tmp47 + _tmp219 * _tmp47 + _tmp222 * _tmp76 + _tmp223 -
                         _tmp224 * _tmp76 - _tmp225;
  const Scalar _tmp227 = _tmp226 * _tmp67;
  const Scalar _tmp228 = _tmp162 * _tmp174;
  const Scalar _tmp229 = Scalar(0.41999999999999998) * _tmp22;
  const Scalar _tmp230 = _tmp164 * _tmp229;
  const Scalar _tmp231 = _tmp161 * _tmp175;
  const Scalar _tmp232 = _tmp17 * _tmp172;
  const Scalar _tmp233 = -_tmp228 - _tmp230 - _tmp231 - _tmp232;
  const Scalar _tmp234 = _tmp172 * _tmp27;
  const Scalar _tmp235 = _tmp164 * _tmp175;
  const Scalar _tmp236 = _tmp161 * _tmp229;
  const Scalar _tmp237 = _tmp165 * _tmp174;
  const Scalar _tmp238 = Scalar(0.043999999999999997) * _tmp179;
  const Scalar _tmp239 = Scalar(0.043999999999999997) * _tmp174 * _tmp22;
  const Scalar _tmp240 = _tmp238 + _tmp239;
  const Scalar _tmp241 = -_tmp234 - _tmp235 + _tmp236 + _tmp237 + _tmp240;
  const Scalar _tmp242 = _tmp233 + _tmp241;
  const Scalar _tmp243 = _tmp228 + _tmp230 + _tmp231 + _tmp232;
  const Scalar _tmp244 = _tmp241 + _tmp243;
  const Scalar _tmp245 = _tmp244 * _tmp76;
  const Scalar _tmp246 = _tmp86 * (-Scalar(1.6799999999999999) * _tmp161 * _tmp17 -
                                   Scalar(1.6799999999999999) * _tmp179);
  const Scalar _tmp247 = _tmp203 * _tmp60;
  const Scalar _tmp248 = _tmp247 * _tmp52;
  const Scalar _tmp249 = _tmp244 * _tmp61;
  const Scalar _tmp250 = _tmp234 + _tmp235 - _tmp236 - _tmp237;
  const Scalar _tmp251 = _tmp240 + _tmp243 + _tmp250;
  const Scalar _tmp252 = -_tmp209 * _tmp52 + _tmp209 * _tmp63 - _tmp247 * _tmp63 + _tmp248 +
                         _tmp249 - _tmp251 * _tmp61;
  const Scalar _tmp253 = _tmp208 * _tmp55;
  const Scalar _tmp254 = _tmp206 * _tmp60;
  const Scalar _tmp255 = _tmp104 * _tmp209 - _tmp222 * _tmp62 + _tmp224 * _tmp62 -
                         _tmp248 * _tmp47 - _tmp249 * _tmp47 + _tmp251 * _tmp65 - _tmp253 * _tmp63 +
                         _tmp254 * _tmp63;
  const Scalar _tmp256 = (-_tmp209 * _tmp47 + _tmp222 * _tmp61 - _tmp224 * _tmp61 +
                          _tmp247 * _tmp47 + _tmp253 - _tmp254) /
                         std::pow(_tmp66, Scalar(2));
  const Scalar _tmp257 = _tmp256 * _tmp78;
  const Scalar _tmp258 =
      -_tmp217 * _tmp52 + _tmp217 * _tmp80 - _tmp219 * _tmp80 + _tmp220 + _tmp221 * _tmp83 -
      _tmp227 * _tmp64 - _tmp242 * _tmp76 + _tmp245 - _tmp246 * _tmp83 - _tmp252 * _tmp79 +
      _tmp257 * _tmp64 -
      _tmp89 * (_tmp104 * _tmp217 - _tmp220 * _tmp47 - _tmp222 * _tmp81 - _tmp223 * _tmp80 +
                _tmp224 * _tmp81 + _tmp225 * _tmp80 - _tmp227 * _tmp82 + _tmp242 * _tmp77 -
                _tmp245 * _tmp47 - _tmp255 * _tmp79 + _tmp257 * _tmp82);
  const Scalar _tmp259 = _tmp198 / (_tmp92 * std::sqrt(_tmp92));
  const Scalar _tmp260 =
      _tmp98 * (-_tmp187 * _tmp45 * _tmp94 + _tmp187 * _tmp95 + _tmp197 * _tmp35 * _tmp94 -
                _tmp197 * _tmp96 - _tmp259 * _tmp35 * _tmp46 + _tmp259 * _tmp36 * _tmp45);
  const Scalar _tmp261 = _tmp189 * _tmp97;
  const Scalar _tmp262 = _tmp199 * _tmp97;
  const Scalar _tmp263 = -_tmp203 * _tmp65 - _tmp209 * _tmp54 - _tmp209 * _tmp99 +
                         _tmp247 * _tmp54 + _tmp247 * _tmp99 + _tmp253 * _tmp57 - _tmp254 * _tmp57 +
                         _tmp254 * _tmp58 + _tmp260 * _tmp61 - _tmp261 * _tmp61 + _tmp262 * _tmp61;
  const Scalar _tmp264 = -_tmp100 * _tmp227 + _tmp100 * _tmp257 - _tmp214 * _tmp77 -
                         _tmp217 * _tmp72 - _tmp217 * _tmp99 + _tmp219 * _tmp72 + _tmp219 * _tmp99 +
                         _tmp223 * _tmp69 - _tmp225 * _tmp69 + _tmp225 * _tmp70 + _tmp260 * _tmp76 -
                         _tmp261 * _tmp76 + _tmp262 * _tmp76 - _tmp263 * _tmp79;
  const Scalar _tmp265 = _tmp264 / std::pow(_tmp101, Scalar(2));
  const Scalar _tmp266 = _tmp265 * _tmp90;
  const Scalar _tmp267 = _tmp222 * _tmp67;
  const Scalar _tmp268 = _tmp256 * _tmp47;
  const Scalar _tmp269 = _tmp224 * _tmp67;
  const Scalar _tmp270 =
      _tmp103 * _tmp252 + _tmp105 * _tmp221 - _tmp105 * _tmp246 + _tmp233 - _tmp238 - _tmp239 +
      _tmp250 + _tmp267 * _tmp64 - _tmp268 * _tmp64 - _tmp269 * _tmp64 -
      _tmp89 * (_tmp103 * _tmp255 + _tmp222 * _tmp52 - _tmp224 * _tmp52 + _tmp244 * _tmp47 +
                _tmp267 * _tmp82 - _tmp268 * _tmp82 - _tmp269 * _tmp82);
  const Scalar _tmp271 = _tmp258 / std::pow(_tmp90, Scalar(2));
  const Scalar _tmp272 = _tmp101 * _tmp271;
  const Scalar _tmp273 = -_tmp108 * _tmp266 + _tmp110 * _tmp258 +
                         _tmp145 * (_tmp100 * _tmp267 - _tmp100 * _tmp268 - _tmp100 * _tmp269 +
                                    _tmp103 * _tmp263 + _tmp106 * _tmp272 - _tmp107 * _tmp264 -
                                    _tmp143 * _tmp270 - _tmp260 + _tmp261 - _tmp262) +
                         _tmp270;
  const Scalar _tmp274 = _tmp247 * _tmp67;
  const Scalar _tmp275 = _tmp256 * _tmp61;
  const Scalar _tmp276 = _tmp102 * _tmp226;
  const Scalar _tmp277 = _tmp271 * _tmp78;
  const Scalar _tmp278 =
      -_tmp111 * _tmp276 + _tmp111 * _tmp277 - _tmp112 * _tmp273 - _tmp222 + _tmp224;
  const Scalar _tmp279 = _tmp271 * _tmp76;
  const Scalar _tmp280 = _tmp102 * _tmp219;
  const Scalar _tmp281 = _tmp130 * _tmp131;
  const Scalar _tmp282 = _tmp202 + _tmp204 + _tmp212;
  const Scalar _tmp283 = _tmp122 * _tmp282;
  const Scalar _tmp284 = _tmp138 * _tmp148;
  const Scalar _tmp285 = _tmp148 * _tmp149;
  const Scalar _tmp286 = _tmp201 + _tmp204 + _tmp211;
  const Scalar _tmp287 = (Scalar(1) / Scalar(2)) * (2 * _tmp118 * _tmp286 + 2 * _tmp120 * _tmp282) /
                         (_tmp121 * std::sqrt(_tmp121));
  const Scalar _tmp288 = _tmp120 * _tmp287;
  const Scalar _tmp289 = _tmp118 * _tmp287;
  const Scalar _tmp290 = _tmp122 * _tmp286;
  const Scalar _tmp291 = _tmp117 * _tmp283 - _tmp117 * _tmp288 + _tmp119 * _tmp289 -
                         _tmp119 * _tmp290 - _tmp123 * _tmp282 + _tmp133 * _tmp286;
  const Scalar _tmp292 = Scalar(1.0) * _tmp265;
  const Scalar _tmp293 = _tmp61 * _tmp79;
  const Scalar _tmp294 = Scalar(0.5) * _tmp109;
  const Scalar _tmp295 = _tmp109 * _tmp61;
  const Scalar _tmp296 = _tmp140 * _tmp226;
  const Scalar _tmp297 = Scalar(1.0) * _tmp256;
  const Scalar _tmp298 = _tmp297 * _tmp78;
  const Scalar _tmp299 = _tmp147 * _tmp67;
  const Scalar _tmp300 = -_tmp140 * _tmp252 + _tmp140 * _tmp255 * _tmp89 - _tmp141 * _tmp221 +
                         _tmp141 * _tmp246 + _tmp297 * _tmp64 - _tmp297 * _tmp82 * _tmp89;
  const Scalar _tmp301 = _tmp109 * _tmp144 * _tmp258 - _tmp144 * _tmp266 +
                         _tmp145 * (_tmp100 * _tmp297 - _tmp102 * _tmp142 * _tmp264 -
                                    _tmp140 * _tmp263 + _tmp142 * _tmp272 - _tmp143 * _tmp300) +
                         _tmp300;
  const Scalar _tmp302 = -_tmp112 * _tmp301 - _tmp146 * _tmp276 + _tmp146 * _tmp277;
  const Scalar _tmp303 = _tmp102 * _tmp146;
  const Scalar _tmp304 = _tmp290 * fh1;
  const Scalar _tmp305 = _tmp127 * _tmp271;
  const Scalar _tmp306 = _tmp135 * fh1;
  const Scalar _tmp307 = _tmp134 * _tmp306;
  const Scalar _tmp308 = _tmp149 * _tmp67;
  const Scalar _tmp309 = _tmp124 * _tmp67;
  const Scalar _tmp310 = _tmp128 * _tmp131;
  const Scalar _tmp311 = _tmp283 * fh1;
  const Scalar _tmp312 = _tmp291 * _tmp306;
  const Scalar _tmp313 = _tmp109 * _tmp190;
  const Scalar _tmp314 = _tmp289 * fh1;
  const Scalar _tmp315 = _tmp131 * _tmp305;
  const Scalar _tmp316 = _tmp127 * _tmp131;
  const Scalar _tmp317 = _tmp190 * _tmp292;
  const Scalar _tmp318 = _tmp288 * fh1;
  const Scalar _tmp319 = _tmp102 * _tmp124;

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) = 0;
  _res(1, 0) =
      -(_tmp116 * _tmp138 * _tmp289 - _tmp116 * _tmp304 * _tmp98 -
        _tmp125 * (-_tmp111 * _tmp279 + _tmp111 * _tmp280 + _tmp113 * _tmp274 - _tmp113 * _tmp275 +
                   _tmp114 * _tmp278 + _tmp115 * _tmp273 - _tmp209 * _tmp210 - _tmp217 * _tmp218) -
        _tmp132 * (_tmp112 * _tmp127 * _tmp275 - _tmp114 * _tmp127 * _tmp276 - _tmp128 * _tmp217 +
                   _tmp128 * _tmp219 + _tmp129 * _tmp209 - _tmp129 * _tmp247 + _tmp293 * _tmp305 -
                   _tmp305 * _tmp76) -
        _tmp134 * _tmp138 *
            (_tmp135 * _tmp219 - _tmp136 * _tmp247 + _tmp207 * _tmp294 * _tmp58 * _tmp79 -
             _tmp215 * _tmp294 * _tmp70 + _tmp292 * _tmp293 - _tmp292 * _tmp76 - _tmp295 * _tmp296 +
             _tmp295 * _tmp298) -
        _tmp139 * _tmp291 -
        _tmp150 * (_tmp114 * _tmp302 + _tmp115 * _tmp301 - _tmp146 * _tmp279 + _tmp146 * _tmp280 +
                   _tmp147 * _tmp274 - _tmp147 * _tmp275 - _tmp209 * _tmp299 - _tmp217 * _tmp303) +
        _tmp189 * _tmp191 + _tmp189 * _tmp200 + _tmp189 * _tmp281 + _tmp189 * _tmp285 -
        _tmp191 * _tmp199 - _tmp199 * _tmp200 - _tmp199 * _tmp281 - _tmp199 * _tmp285 -
        _tmp283 * _tmp284 + _tmp284 * _tmp288) *
      std::exp(_tmp116 * _tmp125 + _tmp130 * _tmp132 + _tmp134 * _tmp139 + _tmp148 * _tmp150);
  _res(2, 0) =
      -(_tmp112 * _tmp256 * _tmp316 - _tmp113 * _tmp124 * _tmp256 - _tmp147 * _tmp149 * _tmp256 +
        _tmp210 * _tmp304 - _tmp210 * _tmp314 - _tmp276 * _tmp316 * _tmp67 + _tmp278 * _tmp309 -
        _tmp296 * _tmp313 + _tmp298 * _tmp313 + _tmp299 * _tmp311 - _tmp299 * _tmp318 +
        _tmp302 * _tmp308 - _tmp312 * _tmp79 + _tmp315 * _tmp79 + _tmp317 * _tmp79) *
      std::exp(-_tmp113 * _tmp309 - _tmp147 * _tmp308 + _tmp307 * _tmp79 + _tmp310 * _tmp79);
  _res(3, 0) =
      -(_tmp102 * _tmp149 * _tmp301 - _tmp111 * _tmp124 * _tmp271 - _tmp146 * _tmp149 * _tmp271 +
        _tmp218 * _tmp304 - _tmp218 * _tmp314 + _tmp273 * _tmp319 + _tmp303 * _tmp311 -
        _tmp303 * _tmp318 + _tmp312 - _tmp315 - _tmp317) *
      std::exp(-_tmp111 * _tmp319 - _tmp149 * _tmp303 - _tmp307 - _tmp310);

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
