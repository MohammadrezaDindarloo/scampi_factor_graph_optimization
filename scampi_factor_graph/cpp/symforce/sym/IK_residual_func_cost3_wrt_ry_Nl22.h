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
 * Symbolic function: IK_residual_func_cost3_wrt_ry_Nl22
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost3WrtRyNl22(
    const Scalar fh1, const Scalar fv1, const Scalar rx, const Scalar ry, const Scalar rz,
    const Scalar p_init0, const Scalar p_init1, const Scalar p_init2, const Scalar rot_init_x,
    const Scalar rot_init_y, const Scalar rot_init_z, const Scalar rot_init_w,
    const Scalar epsilon) {
  // Total ops: 998

  // Unused inputs
  (void)p_init2;
  (void)epsilon;

  // Input arrays

  // Intermediate terms (323)
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
  const Scalar _tmp23 = 2 * _tmp12 * _tmp22;
  const Scalar _tmp24 = _tmp4 * rot_init_w;
  const Scalar _tmp25 = _tmp16 * ry;
  const Scalar _tmp26 = -_tmp10 * rz - _tmp11 * rx + _tmp24 - _tmp25;
  const Scalar _tmp27 = 2 * _tmp26;
  const Scalar _tmp28 = _tmp17 * _tmp27;
  const Scalar _tmp29 = Scalar(0.20999999999999999) * _tmp23 - Scalar(0.20999999999999999) * _tmp28;
  const Scalar _tmp30 = 2 * _tmp17;
  const Scalar _tmp31 = _tmp22 * _tmp30;
  const Scalar _tmp32 = _tmp12 * _tmp27;
  const Scalar _tmp33 =
      -Scalar(0.010999999999999999) * _tmp31 - Scalar(0.010999999999999999) * _tmp32;
  const Scalar _tmp34 = _tmp29 + _tmp33;
  const Scalar _tmp35 = _tmp19 + _tmp34;
  const Scalar _tmp36 = _tmp35 + p_init0 + Scalar(-2.71799795);
  const Scalar _tmp37 = Scalar(1.0) / (_tmp36);
  const Scalar _tmp38 = Scalar(0.20999999999999999) * _tmp23 + Scalar(0.20999999999999999) * _tmp28;
  const Scalar _tmp39 = _tmp12 * _tmp30;
  const Scalar _tmp40 = _tmp22 * _tmp27;
  const Scalar _tmp41 =
      -Scalar(0.010999999999999999) * _tmp39 + Scalar(0.010999999999999999) * _tmp40;
  const Scalar _tmp42 = 1 - 2 * std::pow(_tmp22, Scalar(2));
  const Scalar _tmp43 = Scalar(0.20999999999999999) * _tmp18 + Scalar(0.20999999999999999) * _tmp42;
  const Scalar _tmp44 = _tmp41 + _tmp43;
  const Scalar _tmp45 = _tmp38 + _tmp44;
  const Scalar _tmp46 = _tmp45 + p_init1 + Scalar(-4.7752063900000001);
  const Scalar _tmp47 = _tmp37 * _tmp46;
  const Scalar _tmp48 = Scalar(0.20999999999999999) * _tmp39 + Scalar(0.20999999999999999) * _tmp40;
  const Scalar _tmp49 =
      -Scalar(0.010999999999999999) * _tmp13 - Scalar(0.010999999999999999) * _tmp42;
  const Scalar _tmp50 = Scalar(0.20999999999999999) * _tmp31 - Scalar(0.20999999999999999) * _tmp32;
  const Scalar _tmp51 = _tmp49 + _tmp50;
  const Scalar _tmp52 = _tmp48 + _tmp51;
  const Scalar _tmp53 = -_tmp38;
  const Scalar _tmp54 = _tmp41 - _tmp43;
  const Scalar _tmp55 = _tmp53 + _tmp54;
  const Scalar _tmp56 = _tmp55 + p_init1 + Scalar(8.3196563700000006);
  const Scalar _tmp57 = -_tmp19;
  const Scalar _tmp58 = -_tmp29 + _tmp33;
  const Scalar _tmp59 = _tmp57 + _tmp58;
  const Scalar _tmp60 = _tmp59 + p_init0 + Scalar(1.9874742000000001);
  const Scalar _tmp61 = std::pow(_tmp56, Scalar(2)) + std::pow(_tmp60, Scalar(2));
  const Scalar _tmp62 = std::pow(_tmp61, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp63 = _tmp60 * _tmp62;
  const Scalar _tmp64 = _tmp52 * _tmp63;
  const Scalar _tmp65 = -_tmp48;
  const Scalar _tmp66 = _tmp49 - _tmp50 + _tmp65;
  const Scalar _tmp67 = _tmp56 * _tmp62;
  const Scalar _tmp68 = -_tmp47 * _tmp64 + _tmp66 * _tmp67;
  const Scalar _tmp69 = _tmp47 * _tmp63 - _tmp67;
  const Scalar _tmp70 = Scalar(1.0) / (_tmp69);
  const Scalar _tmp71 = Scalar(1.0) * _tmp70;
  const Scalar _tmp72 = Scalar(1.0) * _tmp35;
  const Scalar _tmp73 = -_tmp59 + _tmp72;
  const Scalar _tmp74 = Scalar(1.0) * _tmp45;
  const Scalar _tmp75 = _tmp55 - _tmp74;
  const Scalar _tmp76 = Scalar(1.0) / (_tmp75);
  const Scalar _tmp77 = _tmp73 * _tmp76;
  const Scalar _tmp78 = _tmp71 * _tmp77;
  const Scalar _tmp79 = -_tmp63 * _tmp66 + _tmp64;
  const Scalar _tmp80 = _tmp68 * _tmp78 - _tmp71 * _tmp79;
  const Scalar _tmp81 = _tmp51 + _tmp65;
  const Scalar _tmp82 = _tmp19 + _tmp58;
  const Scalar _tmp83 = _tmp82 + p_init0 + Scalar(-2.5202214700000001);
  const Scalar _tmp84 = _tmp38 + _tmp54;
  const Scalar _tmp85 = _tmp84 + p_init1 + Scalar(8.3888750099999996);
  const Scalar _tmp86 = std::pow(_tmp83, Scalar(2)) + std::pow(_tmp85, Scalar(2));
  const Scalar _tmp87 = std::pow(_tmp86, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp88 = _tmp83 * _tmp87;
  const Scalar _tmp89 = _tmp52 * _tmp88;
  const Scalar _tmp90 = _tmp85 * _tmp87;
  const Scalar _tmp91 = _tmp47 * _tmp88 - _tmp90;
  const Scalar _tmp92 = _tmp70 * _tmp91;
  const Scalar _tmp93 = -_tmp47 * _tmp89 - _tmp68 * _tmp92 + _tmp81 * _tmp90;
  const Scalar _tmp94 = -_tmp77 * _tmp93 - _tmp79 * _tmp92 - _tmp81 * _tmp88 + _tmp89;
  const Scalar _tmp95 = Scalar(1.0) / (_tmp94);
  const Scalar _tmp96 = std::pow(_tmp36, Scalar(2));
  const Scalar _tmp97 = std::pow(_tmp46, Scalar(2)) + _tmp96;
  const Scalar _tmp98 = std::sqrt(_tmp97);
  const Scalar _tmp99 = Scalar(1.0) / (_tmp98);
  const Scalar _tmp100 = _tmp35 * _tmp99;
  const Scalar _tmp101 = _tmp36 * _tmp99;
  const Scalar _tmp102 = _tmp100 * _tmp46 - _tmp101 * _tmp45;
  const Scalar _tmp103 = _tmp37 * _tmp98;
  const Scalar _tmp104 = _tmp102 * _tmp103;
  const Scalar _tmp105 = _tmp104 * _tmp63 + _tmp55 * _tmp63 - _tmp59 * _tmp67;
  const Scalar _tmp106 = _tmp84 * _tmp87;
  const Scalar _tmp107 = _tmp104 * _tmp88 - _tmp105 * _tmp92 + _tmp106 * _tmp83 - _tmp82 * _tmp90;
  const Scalar _tmp108 = _tmp107 * _tmp95;
  const Scalar _tmp109 = -_tmp105 * _tmp71 - _tmp108 * _tmp80;
  const Scalar _tmp110 = Scalar(1.0) / (_tmp107);
  const Scalar _tmp111 = _tmp110 * _tmp94;
  const Scalar _tmp112 = _tmp109 * _tmp111 + _tmp80;
  const Scalar _tmp113 = _tmp91 * _tmp95;
  const Scalar _tmp114 = -_tmp112 * _tmp113 + Scalar(1.0);
  const Scalar _tmp115 = _tmp63 * _tmp70;
  const Scalar _tmp116 = _tmp112 * _tmp95;
  const Scalar _tmp117 = _tmp114 * _tmp115 + _tmp116 * _tmp88;
  const Scalar _tmp118 = _tmp44 + _tmp53;
  const Scalar _tmp119 = _tmp118 + p_init1 + Scalar(-4.8333311099999996);
  const Scalar _tmp120 = _tmp34 + _tmp57;
  const Scalar _tmp121 = _tmp120 + p_init0 + Scalar(1.79662371);
  const Scalar _tmp122 = std::pow(_tmp119, Scalar(2)) + std::pow(_tmp121, Scalar(2));
  const Scalar _tmp123 = std::pow(_tmp122, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp124 = _tmp123 * fh1;
  const Scalar _tmp125 = _tmp119 * _tmp124;
  const Scalar _tmp126 = _tmp103 * _tmp125;
  const Scalar _tmp127 = _tmp120 * _tmp123;
  const Scalar _tmp128 = _tmp121 * _tmp123;
  const Scalar _tmp129 = -_tmp118 * _tmp128 + _tmp119 * _tmp127;
  const Scalar _tmp130 = _tmp129 * fh1;
  const Scalar _tmp131 = Scalar(1.0) * _tmp110;
  const Scalar _tmp132 = _tmp110 * _tmp71;
  const Scalar _tmp133 = _tmp132 * _tmp91;
  const Scalar _tmp134 = _tmp131 * _tmp88 - _tmp133 * _tmp63;
  const Scalar _tmp135 = _tmp103 * _tmp134;
  const Scalar _tmp136 = _tmp72 + _tmp74 * _tmp77;
  const Scalar _tmp137 = 0;
  const Scalar _tmp138 = _tmp88 * _tmp95;
  const Scalar _tmp139 = _tmp137 * _tmp95;
  const Scalar _tmp140 = _tmp139 * _tmp92;
  const Scalar _tmp141 = _tmp137 * _tmp138 - _tmp140 * _tmp63;
  const Scalar _tmp142 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp143 = _tmp103 * _tmp142;
  const Scalar _tmp144 = _tmp128 * fh1;
  const Scalar _tmp145 = _tmp47 * _tmp52;
  const Scalar _tmp146 = _tmp47 * _tmp70;
  const Scalar _tmp147 = _tmp145 + _tmp146 * _tmp68;
  const Scalar _tmp148 = _tmp146 * _tmp79 - _tmp147 * _tmp77 - _tmp52;
  const Scalar _tmp149 = -_tmp104 + _tmp105 * _tmp146 - _tmp108 * _tmp148;
  const Scalar _tmp150 = _tmp111 * _tmp149 + _tmp148;
  const Scalar _tmp151 = _tmp150 * _tmp95;
  const Scalar _tmp152 = -_tmp151 * _tmp91 - _tmp47;
  const Scalar _tmp153 = _tmp115 * _tmp152 + _tmp151 * _tmp88 + Scalar(1.0);
  const Scalar _tmp154 = _tmp103 * _tmp153;
  const Scalar _tmp155 = (Scalar(1) / Scalar(2)) / _tmp1;
  const Scalar _tmp156 = _tmp0 * _tmp155;
  const Scalar _tmp157 = _tmp155 * ry;
  const Scalar _tmp158 = _tmp157 * rz;
  const Scalar _tmp159 = _tmp157 * rx;
  const Scalar _tmp160 = _tmp6 / (_tmp1 * std::sqrt(_tmp1));
  const Scalar _tmp161 = _tmp0 * _tmp160;
  const Scalar _tmp162 = _tmp160 * ry;
  const Scalar _tmp163 = _tmp162 * rx;
  const Scalar _tmp164 = _tmp162 * rz;
  const Scalar _tmp165 = _tmp14 * _tmp159 + _tmp156 * _tmp24 - _tmp158 * _tmp20 -
                         _tmp161 * rot_init_w - _tmp163 * rot_init_z + _tmp164 * rot_init_x -
                         Scalar(1) / Scalar(2) * _tmp25 + _tmp8;
  const Scalar _tmp166 = _tmp12 * _tmp165;
  const Scalar _tmp167 = Scalar(0.83999999999999997) * _tmp166;
  const Scalar _tmp168 = Scalar(0.021999999999999999) * _tmp26;
  const Scalar _tmp169 = -_tmp14 * _tmp158 - _tmp156 * _tmp5 - _tmp159 * _tmp20 - _tmp16 +
                         _tmp161 * rot_init_y + _tmp163 * rot_init_x + _tmp164 * rot_init_z -
                         Scalar(1) / Scalar(2) * _tmp9;
  const Scalar _tmp170 = Scalar(0.021999999999999999) * _tmp169;
  const Scalar _tmp171 = _tmp11 + _tmp156 * _tmp20 + _tmp158 * _tmp24 - _tmp159 * _tmp5 -
                         _tmp161 * rot_init_x + _tmp163 * rot_init_y - _tmp164 * rot_init_w -
                         Scalar(1) / Scalar(2) * _tmp21;
  const Scalar _tmp172 = Scalar(0.021999999999999999) * _tmp171;
  const Scalar _tmp173 = -_tmp10 - _tmp14 * _tmp156 - Scalar(1) / Scalar(2) * _tmp15 +
                         _tmp158 * _tmp5 + _tmp159 * _tmp24 + _tmp161 * rot_init_z -
                         _tmp163 * rot_init_w - _tmp164 * rot_init_y;
  const Scalar _tmp174 = Scalar(0.021999999999999999) * _tmp17;
  const Scalar _tmp175 =
      -_tmp12 * _tmp170 - _tmp165 * _tmp168 - _tmp172 * _tmp22 - _tmp173 * _tmp174;
  const Scalar _tmp176 = -_tmp167 + _tmp175;
  const Scalar _tmp177 = _tmp165 * _tmp22;
  const Scalar _tmp178 = Scalar(0.41999999999999998) * _tmp177;
  const Scalar _tmp179 = _tmp12 * _tmp173;
  const Scalar _tmp180 = Scalar(0.41999999999999998) * _tmp179;
  const Scalar _tmp181 = -_tmp178 - _tmp180;
  const Scalar _tmp182 = Scalar(0.83999999999999997) * _tmp171;
  const Scalar _tmp183 = _tmp17 * _tmp182;
  const Scalar _tmp184 = -_tmp183;
  const Scalar _tmp185 = Scalar(0.41999999999999998) * _tmp26;
  const Scalar _tmp186 = _tmp171 * _tmp185;
  const Scalar _tmp187 = Scalar(0.41999999999999998) * _tmp17;
  const Scalar _tmp188 = _tmp169 * _tmp187;
  const Scalar _tmp189 = _tmp186 + _tmp188;
  const Scalar _tmp190 = _tmp184 + _tmp189;
  const Scalar _tmp191 = _tmp176 + _tmp181 + _tmp190;
  const Scalar _tmp192 = _tmp191 * _tmp87;
  const Scalar _tmp193 = _tmp178 + _tmp180;
  const Scalar _tmp194 = _tmp183 + _tmp189;
  const Scalar _tmp195 = _tmp173 * _tmp22;
  const Scalar _tmp196 = Scalar(0.83999999999999997) * _tmp195;
  const Scalar _tmp197 =
      -_tmp12 * _tmp172 - _tmp165 * _tmp174 + _tmp168 * _tmp173 + _tmp170 * _tmp22;
  const Scalar _tmp198 = _tmp196 + _tmp197;
  const Scalar _tmp199 = _tmp193 + _tmp194 + _tmp198;
  const Scalar _tmp200 =
      (2 * _tmp191 * _tmp83 + 2 * _tmp199 * _tmp85) / (_tmp86 * std::sqrt(_tmp86));
  const Scalar _tmp201 = (Scalar(1) / Scalar(2)) * _tmp200;
  const Scalar _tmp202 = _tmp201 * _tmp83;
  const Scalar _tmp203 = -_tmp196 + _tmp197;
  const Scalar _tmp204 = _tmp190 + _tmp193 + _tmp203;
  const Scalar _tmp205 = _tmp204 * _tmp37;
  const Scalar _tmp206 = _tmp201 * _tmp85;
  const Scalar _tmp207 = -_tmp186 - _tmp188;
  const Scalar _tmp208 = _tmp184 + _tmp207;
  const Scalar _tmp209 = _tmp176 + _tmp193 + _tmp208;
  const Scalar _tmp210 = _tmp209 / _tmp96;
  const Scalar _tmp211 = _tmp210 * _tmp46;
  const Scalar _tmp212 = _tmp199 * _tmp87;
  const Scalar _tmp213 =
      _tmp192 * _tmp47 - _tmp202 * _tmp47 + _tmp205 * _tmp88 + _tmp206 - _tmp211 * _tmp88 - _tmp212;
  const Scalar _tmp214 = _tmp213 * _tmp70;
  const Scalar _tmp215 = _tmp167 + _tmp175;
  const Scalar _tmp216 = _tmp181 + _tmp194 + _tmp215;
  const Scalar _tmp217 = _tmp183 + _tmp207;
  const Scalar _tmp218 = _tmp181 + _tmp198 + _tmp217;
  const Scalar _tmp219 =
      (2 * _tmp216 * _tmp60 + 2 * _tmp218 * _tmp56) / (_tmp61 * std::sqrt(_tmp61));
  const Scalar _tmp220 = (Scalar(1) / Scalar(2)) * _tmp219;
  const Scalar _tmp221 = _tmp220 * _tmp56;
  const Scalar _tmp222 = _tmp220 * _tmp60;
  const Scalar _tmp223 = _tmp218 * _tmp62;
  const Scalar _tmp224 = _tmp216 * _tmp62;
  const Scalar _tmp225 = (_tmp205 * _tmp63 - _tmp211 * _tmp63 + _tmp221 - _tmp222 * _tmp47 -
                          _tmp223 + _tmp224 * _tmp47) /
                         std::pow(_tmp69, Scalar(2));
  const Scalar _tmp226 = _tmp225 * _tmp91;
  const Scalar _tmp227 = 2 * _tmp204 * _tmp46 + 2 * _tmp209 * _tmp36;
  const Scalar _tmp228 = (Scalar(1) / Scalar(2)) * _tmp227;
  const Scalar _tmp229 = _tmp37 * _tmp99;
  const Scalar _tmp230 = _tmp228 * _tmp229;
  const Scalar _tmp231 = _tmp102 * _tmp230;
  const Scalar _tmp232 = _tmp210 * _tmp98;
  const Scalar _tmp233 = _tmp102 * _tmp232;
  const Scalar _tmp234 = _tmp209 * _tmp99;
  const Scalar _tmp235 = _tmp228 / (_tmp97 * std::sqrt(_tmp97));
  const Scalar _tmp236 =
      _tmp103 * (_tmp100 * _tmp204 - _tmp101 * _tmp204 - _tmp234 * _tmp45 + _tmp234 * _tmp46 -
                 _tmp235 * _tmp35 * _tmp46 + _tmp235 * _tmp36 * _tmp45);
  const Scalar _tmp237 = -_tmp104 * _tmp222 + _tmp104 * _tmp224 - _tmp216 * _tmp67 +
                         _tmp221 * _tmp59 - _tmp222 * _tmp55 - _tmp223 * _tmp59 + _tmp223 * _tmp60 +
                         _tmp224 * _tmp55 + _tmp231 * _tmp63 - _tmp233 * _tmp63 + _tmp236 * _tmp63;
  const Scalar _tmp238 = _tmp104 * _tmp192 - _tmp104 * _tmp202 - _tmp105 * _tmp214 +
                         _tmp105 * _tmp226 + _tmp106 * _tmp191 - _tmp191 * _tmp90 -
                         _tmp202 * _tmp84 + _tmp206 * _tmp82 - _tmp212 * _tmp82 + _tmp212 * _tmp83 +
                         _tmp231 * _tmp88 - _tmp233 * _tmp88 + _tmp236 * _tmp88 - _tmp237 * _tmp92;
  const Scalar _tmp239 = _tmp238 / std::pow(_tmp107, Scalar(2));
  const Scalar _tmp240 = _tmp239 * _tmp94;
  const Scalar _tmp241 = _tmp192 * _tmp52;
  const Scalar _tmp242 = _tmp165 * _tmp185;
  const Scalar _tmp243 = Scalar(0.41999999999999998) * _tmp12;
  const Scalar _tmp244 = _tmp169 * _tmp243;
  const Scalar _tmp245 = Scalar(0.41999999999999998) * _tmp22;
  const Scalar _tmp246 = _tmp171 * _tmp245;
  const Scalar _tmp247 = _tmp173 * _tmp187;
  const Scalar _tmp248 = Scalar(0.043999999999999997) * _tmp166;
  const Scalar _tmp249 = Scalar(0.043999999999999997) * _tmp195;
  const Scalar _tmp250 = _tmp248 + _tmp249;
  const Scalar _tmp251 = -_tmp242 - _tmp244 + _tmp246 + _tmp247 + _tmp250;
  const Scalar _tmp252 = _tmp173 * _tmp185;
  const Scalar _tmp253 = _tmp169 * _tmp245;
  const Scalar _tmp254 = _tmp171 * _tmp243;
  const Scalar _tmp255 = _tmp165 * _tmp187;
  const Scalar _tmp256 = -_tmp252 - _tmp253 - _tmp254 - _tmp255;
  const Scalar _tmp257 = _tmp251 + _tmp256;
  const Scalar _tmp258 = _tmp251 + _tmp252 + _tmp253 + _tmp254 + _tmp255;
  const Scalar _tmp259 = _tmp258 * _tmp47;
  const Scalar _tmp260 = _tmp258 * _tmp63;
  const Scalar _tmp261 = _tmp242 + _tmp244 - _tmp246 - _tmp247 + _tmp256;
  const Scalar _tmp262 = _tmp250 + _tmp261;
  const Scalar _tmp263 = _tmp224 * _tmp52;
  const Scalar _tmp264 = _tmp145 * _tmp222 - _tmp205 * _tmp64 + _tmp211 * _tmp64 -
                         _tmp221 * _tmp66 + _tmp223 * _tmp66 - _tmp260 * _tmp47 + _tmp262 * _tmp67 -
                         _tmp263 * _tmp47;
  const Scalar _tmp265 = Scalar(0.83999999999999997) * _tmp177;
  const Scalar _tmp266 = Scalar(1.6799999999999999) * _tmp17 * _tmp171;
  const Scalar _tmp267 = Scalar(0.83999999999999997) * _tmp179;
  const Scalar _tmp268 = -Scalar(0.83999999999999997) * _tmp169 * _tmp17 - _tmp182 * _tmp26;
  const Scalar _tmp269 =
      _tmp73 * (Scalar(1.6799999999999999) * _tmp195 - _tmp265 + _tmp266 - _tmp267 + _tmp268) /
      std::pow(_tmp75, Scalar(2));
  const Scalar _tmp270 = -_tmp222 * _tmp52 + _tmp222 * _tmp66 - _tmp224 * _tmp66 + _tmp260 -
                         _tmp262 * _tmp63 + _tmp263;
  const Scalar _tmp271 =
      _tmp76 * (-Scalar(1.6799999999999999) * _tmp166 + _tmp265 - _tmp266 + _tmp267 + _tmp268);
  const Scalar _tmp272 =
      -_tmp192 * _tmp81 - _tmp202 * _tmp52 + _tmp202 * _tmp81 - _tmp214 * _tmp79 +
      _tmp226 * _tmp79 + _tmp241 - _tmp257 * _tmp88 + _tmp258 * _tmp88 + _tmp269 * _tmp93 -
      _tmp270 * _tmp92 - _tmp271 * _tmp93 -
      _tmp77 * (_tmp145 * _tmp202 - _tmp205 * _tmp89 - _tmp206 * _tmp81 + _tmp211 * _tmp89 +
                _tmp212 * _tmp81 - _tmp214 * _tmp68 + _tmp226 * _tmp68 - _tmp241 * _tmp47 +
                _tmp257 * _tmp90 - _tmp259 * _tmp88 - _tmp264 * _tmp92);
  const Scalar _tmp273 = _tmp110 * _tmp272;
  const Scalar _tmp274 = _tmp211 * _tmp70;
  const Scalar _tmp275 = _tmp225 * _tmp47;
  const Scalar _tmp276 = _tmp205 * _tmp70;
  const Scalar _tmp277 =
      _tmp146 * _tmp270 + _tmp147 * _tmp269 - _tmp147 * _tmp271 - _tmp248 - _tmp249 + _tmp261 -
      _tmp274 * _tmp79 - _tmp275 * _tmp79 + _tmp276 * _tmp79 -
      _tmp77 * (_tmp146 * _tmp264 + _tmp205 * _tmp52 - _tmp211 * _tmp52 + _tmp259 -
                _tmp274 * _tmp68 - _tmp275 * _tmp68 + _tmp276 * _tmp68);
  const Scalar _tmp278 = _tmp238 * _tmp95;
  const Scalar _tmp279 = _tmp272 / std::pow(_tmp94, Scalar(2));
  const Scalar _tmp280 = _tmp107 * _tmp279;
  const Scalar _tmp281 = _tmp111 * (-_tmp105 * _tmp274 - _tmp105 * _tmp275 + _tmp105 * _tmp276 -
                                    _tmp108 * _tmp277 + _tmp146 * _tmp237 - _tmp148 * _tmp278 +
                                    _tmp148 * _tmp280 - _tmp231 + _tmp233 - _tmp236) -
                         _tmp149 * _tmp240 + _tmp149 * _tmp273 + _tmp277;
  const Scalar _tmp282 = _tmp279 * _tmp88;
  const Scalar _tmp283 = _tmp224 * _tmp70;
  const Scalar _tmp284 = _tmp225 * _tmp63;
  const Scalar _tmp285 = _tmp279 * _tmp91;
  const Scalar _tmp286 =
      -_tmp113 * _tmp281 + _tmp150 * _tmp285 - _tmp151 * _tmp213 - _tmp205 + _tmp211;
  const Scalar _tmp287 = _tmp152 * _tmp70;
  const Scalar _tmp288 = _tmp193 + _tmp215 + _tmp217;
  const Scalar _tmp289 = _tmp181 + _tmp203 + _tmp208;
  const Scalar _tmp290 =
      (2 * _tmp119 * _tmp289 + 2 * _tmp121 * _tmp288) / (_tmp122 * std::sqrt(_tmp122));
  const Scalar _tmp291 = (Scalar(1) / Scalar(2)) * _tmp121 * _tmp290;
  const Scalar _tmp292 = _tmp291 * fh1;
  const Scalar _tmp293 = _tmp144 * _tmp153;
  const Scalar _tmp294 = _tmp130 * _tmp134;
  const Scalar _tmp295 = _tmp141 * _tmp142;
  const Scalar _tmp296 = _tmp239 * _tmp71 * _tmp91;
  const Scalar _tmp297 = Scalar(0.5) * _tmp110;
  const Scalar _tmp298 = Scalar(1.0) * _tmp239;
  const Scalar _tmp299 = _tmp132 * _tmp213;
  const Scalar _tmp300 = _tmp124 * _tmp289;
  const Scalar _tmp301 = _tmp103 * _tmp117;
  const Scalar _tmp302 = (Scalar(1) / Scalar(2)) * _tmp119;
  const Scalar _tmp303 = _tmp290 * _tmp302;
  const Scalar _tmp304 = _tmp303 * fh1;
  const Scalar _tmp305 = _tmp123 * _tmp288;
  const Scalar _tmp306 = _tmp305 * fh1;
  const Scalar _tmp307 = Scalar(1.0) * _tmp225;
  const Scalar _tmp308 = _tmp68 * _tmp71;
  const Scalar _tmp309 = _tmp264 * _tmp78 - _tmp269 * _tmp308 - _tmp270 * _tmp71 +
                         _tmp271 * _tmp308 - _tmp307 * _tmp68 * _tmp77 + _tmp307 * _tmp79;
  const Scalar _tmp310 = -_tmp109 * _tmp240 + _tmp109 * _tmp273 +
                         _tmp111 * (_tmp105 * _tmp307 - _tmp108 * _tmp309 - _tmp237 * _tmp71 -
                                    _tmp278 * _tmp80 + _tmp280 * _tmp80) +
                         _tmp309;
  const Scalar _tmp311 = _tmp114 * _tmp70;
  const Scalar _tmp312 = _tmp112 * _tmp285 - _tmp113 * _tmp310 - _tmp116 * _tmp213;
  const Scalar _tmp313 = _tmp118 * _tmp291 - _tmp118 * _tmp305 + _tmp119 * _tmp305 -
                         _tmp120 * _tmp303 + _tmp127 * _tmp289 - _tmp128 * _tmp289;
  const Scalar _tmp314 = _tmp313 * fh1;
  const Scalar _tmp315 = _tmp137 * _tmp142;
  const Scalar _tmp316 = _tmp315 * _tmp95;
  const Scalar _tmp317 = _tmp125 * _tmp70;
  const Scalar _tmp318 = _tmp144 * _tmp70;
  const Scalar _tmp319 = _tmp131 * fh1;
  const Scalar _tmp320 = _tmp129 * _tmp319;
  const Scalar _tmp321 = _tmp279 * _tmp315;
  const Scalar _tmp322 = _tmp116 * _tmp124;

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) = 0;
  _res(1, 0) =
      -(-_tmp103 * _tmp130 *
            (_tmp131 * _tmp192 + _tmp131 * _tmp226 * _tmp63 - _tmp133 * _tmp224 -
             _tmp200 * _tmp297 * _tmp83 + _tmp219 * _tmp297 * _tmp60 * _tmp92 + _tmp296 * _tmp63 -
             _tmp298 * _tmp88 - _tmp299 * _tmp63) -
        _tmp103 * _tmp144 *
            (_tmp115 * _tmp286 + _tmp138 * _tmp281 - _tmp150 * _tmp282 + _tmp151 * _tmp192 -
             _tmp151 * _tmp202 + _tmp152 * _tmp283 - _tmp152 * _tmp284 - _tmp222 * _tmp287) -
        _tmp117 * _tmp124 * _tmp227 * _tmp229 * _tmp302 + _tmp117 * _tmp125 * _tmp232 -
        _tmp126 * (-_tmp112 * _tmp282 + _tmp114 * _tmp283 - _tmp114 * _tmp284 + _tmp115 * _tmp312 +
                   _tmp116 * _tmp192 - _tmp116 * _tmp202 + _tmp138 * _tmp310 - _tmp222 * _tmp311) -
        _tmp135 * _tmp314 -
        _tmp143 * (_tmp113 * _tmp137 * _tmp284 + _tmp137 * _tmp279 * _tmp63 * _tmp92 -
                   _tmp137 * _tmp282 + _tmp139 * _tmp192 - _tmp139 * _tmp202 -
                   _tmp139 * _tmp214 * _tmp63 + _tmp140 * _tmp222 - _tmp140 * _tmp224) +
        _tmp154 * _tmp292 - _tmp154 * _tmp306 - _tmp230 * _tmp293 - _tmp230 * _tmp294 -
        _tmp230 * _tmp295 + _tmp232 * _tmp293 + _tmp232 * _tmp294 + _tmp232 * _tmp295 -
        _tmp300 * _tmp301 + _tmp301 * _tmp304) *
      std::exp(_tmp117 * _tmp126 + _tmp130 * _tmp135 + _tmp141 * _tmp143 + _tmp144 * _tmp154);
  _res(2, 0) =
      -(-_tmp114 * _tmp125 * _tmp225 + _tmp130 * _tmp296 - _tmp130 * _tmp299 - _tmp133 * _tmp314 -
        _tmp144 * _tmp152 * _tmp225 - _tmp214 * _tmp316 + _tmp226 * _tmp316 + _tmp226 * _tmp320 +
        _tmp286 * _tmp318 - _tmp287 * _tmp292 + _tmp287 * _tmp306 + _tmp300 * _tmp311 -
        _tmp304 * _tmp311 + _tmp312 * _tmp317 + _tmp321 * _tmp92) *
      std::exp(-_tmp114 * _tmp317 + _tmp130 * _tmp133 - _tmp152 * _tmp318 + _tmp316 * _tmp92);
  _res(3, 0) =
      -(-_tmp112 * _tmp125 * _tmp279 - _tmp116 * _tmp304 + _tmp125 * _tmp310 * _tmp95 -
        _tmp130 * _tmp298 - _tmp144 * _tmp150 * _tmp279 + _tmp144 * _tmp281 * _tmp95 -
        _tmp151 * _tmp292 + _tmp151 * _tmp306 + _tmp289 * _tmp322 + _tmp313 * _tmp319 - _tmp321) *
      std::exp(-_tmp119 * _tmp322 - _tmp144 * _tmp151 - _tmp316 - _tmp320);

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
