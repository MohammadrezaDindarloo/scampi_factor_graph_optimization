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
 * Symbolic function: resedual_func_cost3_wrt_rz_l2
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
Eigen::Matrix<Scalar, 4, 1> ResedualFuncCost3WrtRzL2(
    const Scalar fh1, const Scalar fv1, const Scalar rx, const Scalar ry, const Scalar rz,
    const Scalar p_init0, const Scalar p_init1, const Scalar p_init2, const Scalar rot_init_x,
    const Scalar rot_init_y, const Scalar rot_init_z, const Scalar rot_init_w,
    const Scalar epsilon) {
  // Total ops: 997

  // Unused inputs
  (void)p_init2;
  (void)epsilon;

  // Input arrays

  // Intermediate terms (325)
  const Scalar _tmp0 = std::pow(rz, Scalar(2));
  const Scalar _tmp1 = _tmp0 + std::pow(rx, Scalar(2)) + std::pow(ry, Scalar(2));
  const Scalar _tmp2 = std::sqrt(_tmp1);
  const Scalar _tmp3 = (Scalar(1) / Scalar(2)) * _tmp2;
  const Scalar _tmp4 = std::cos(_tmp3);
  const Scalar _tmp5 = _tmp4 * rot_init_y;
  const Scalar _tmp6 = std::sin(_tmp3);
  const Scalar _tmp7 = _tmp6 / _tmp2;
  const Scalar _tmp8 = _tmp7 * rot_init_w;
  const Scalar _tmp9 = _tmp7 * rot_init_z;
  const Scalar _tmp10 = _tmp7 * rot_init_x;
  const Scalar _tmp11 = _tmp10 * rz;
  const Scalar _tmp12 = -_tmp11 + _tmp5 + _tmp8 * ry + _tmp9 * rx;
  const Scalar _tmp13 = _tmp4 * rot_init_x;
  const Scalar _tmp14 = _tmp7 * rot_init_y;
  const Scalar _tmp15 = _tmp14 * rz;
  const Scalar _tmp16 = _tmp13 + _tmp15 + _tmp8 * rx - _tmp9 * ry;
  const Scalar _tmp17 = 2 * _tmp16;
  const Scalar _tmp18 = _tmp12 * _tmp17;
  const Scalar _tmp19 = _tmp4 * rot_init_z;
  const Scalar _tmp20 = _tmp8 * rz;
  const Scalar _tmp21 = _tmp10 * ry - _tmp14 * rx + _tmp19 + _tmp20;
  const Scalar _tmp22 = _tmp4 * rot_init_w;
  const Scalar _tmp23 = _tmp9 * rz;
  const Scalar _tmp24 = -_tmp10 * rx - _tmp14 * ry + _tmp22 - _tmp23;
  const Scalar _tmp25 = 2 * _tmp24;
  const Scalar _tmp26 = _tmp21 * _tmp25;
  const Scalar _tmp27 = Scalar(0.20999999999999999) * _tmp18 + Scalar(0.20999999999999999) * _tmp26;
  const Scalar _tmp28 = -_tmp27;
  const Scalar _tmp29 = 2 * _tmp12 * _tmp21;
  const Scalar _tmp30 = _tmp16 * _tmp25;
  const Scalar _tmp31 =
      -Scalar(0.010999999999999999) * _tmp29 + Scalar(0.010999999999999999) * _tmp30;
  const Scalar _tmp32 = -2 * std::pow(_tmp16, Scalar(2));
  const Scalar _tmp33 = 1 - 2 * std::pow(_tmp21, Scalar(2));
  const Scalar _tmp34 = Scalar(0.20999999999999999) * _tmp32 + Scalar(0.20999999999999999) * _tmp33;
  const Scalar _tmp35 = _tmp31 - _tmp34;
  const Scalar _tmp36 = _tmp28 + _tmp35;
  const Scalar _tmp37 = _tmp36 + p_init1 + Scalar(8.3196563700000006);
  const Scalar _tmp38 = Scalar(0.20999999999999999) * _tmp18 - Scalar(0.20999999999999999) * _tmp26;
  const Scalar _tmp39 = -_tmp38;
  const Scalar _tmp40 = -2 * std::pow(_tmp12, Scalar(2));
  const Scalar _tmp41 = Scalar(0.20999999999999999) * _tmp33 + Scalar(0.20999999999999999) * _tmp40;
  const Scalar _tmp42 = _tmp17 * _tmp21;
  const Scalar _tmp43 = _tmp12 * _tmp25;
  const Scalar _tmp44 =
      -Scalar(0.010999999999999999) * _tmp42 - Scalar(0.010999999999999999) * _tmp43;
  const Scalar _tmp45 = -_tmp41 + _tmp44;
  const Scalar _tmp46 = _tmp39 + _tmp45;
  const Scalar _tmp47 = _tmp46 + p_init0 + Scalar(1.9874742000000001);
  const Scalar _tmp48 = std::pow(_tmp37, Scalar(2)) + std::pow(_tmp47, Scalar(2));
  const Scalar _tmp49 = std::pow(_tmp48, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp50 = _tmp47 * _tmp49;
  const Scalar _tmp51 = Scalar(0.20999999999999999) * _tmp29 + Scalar(0.20999999999999999) * _tmp30;
  const Scalar _tmp52 = -Scalar(0.010999999999999999) * _tmp32 -
                        Scalar(0.010999999999999999) * _tmp40 + Scalar(-0.010999999999999999);
  const Scalar _tmp53 = Scalar(0.20999999999999999) * _tmp42 - Scalar(0.20999999999999999) * _tmp43;
  const Scalar _tmp54 = _tmp52 - _tmp53;
  const Scalar _tmp55 = _tmp51 + _tmp54;
  const Scalar _tmp56 = _tmp31 + _tmp34;
  const Scalar _tmp57 = _tmp28 + _tmp56;
  const Scalar _tmp58 = _tmp57 + p_init1 + Scalar(-4.8333311099999996);
  const Scalar _tmp59 = _tmp38 + _tmp45;
  const Scalar _tmp60 = _tmp59 + p_init0 + Scalar(1.79662371);
  const Scalar _tmp61 = Scalar(1.0) / (_tmp60);
  const Scalar _tmp62 = _tmp58 * _tmp61;
  const Scalar _tmp63 = _tmp55 * _tmp62;
  const Scalar _tmp64 = -_tmp51 + _tmp54;
  const Scalar _tmp65 = _tmp37 * _tmp49;
  const Scalar _tmp66 = _tmp27 + _tmp56;
  const Scalar _tmp67 = _tmp66 + p_init1 + Scalar(-4.7752063900000001);
  const Scalar _tmp68 = _tmp41 + _tmp44;
  const Scalar _tmp69 = _tmp38 + _tmp68;
  const Scalar _tmp70 = _tmp69 + p_init0 + Scalar(-2.71799795);
  const Scalar _tmp71 = std::pow(_tmp67, Scalar(2)) + std::pow(_tmp70, Scalar(2));
  const Scalar _tmp72 = std::pow(_tmp71, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp73 = _tmp70 * _tmp72;
  const Scalar _tmp74 = _tmp51 + _tmp52 + _tmp53;
  const Scalar _tmp75 = _tmp67 * _tmp72;
  const Scalar _tmp76 = -_tmp63 * _tmp73 + _tmp74 * _tmp75;
  const Scalar _tmp77 = _tmp62 * _tmp73 - _tmp75;
  const Scalar _tmp78 = Scalar(1.0) / (_tmp77);
  const Scalar _tmp79 = _tmp50 * _tmp62 - _tmp65;
  const Scalar _tmp80 = _tmp78 * _tmp79;
  const Scalar _tmp81 = -_tmp50 * _tmp63 + _tmp64 * _tmp65 - _tmp76 * _tmp80;
  const Scalar _tmp82 = Scalar(1.0) * _tmp57;
  const Scalar _tmp83 = _tmp66 - _tmp82;
  const Scalar _tmp84 = Scalar(1.0) / (_tmp83);
  const Scalar _tmp85 = Scalar(1.0) * _tmp59;
  const Scalar _tmp86 = -_tmp69 + _tmp85;
  const Scalar _tmp87 = _tmp84 * _tmp86;
  const Scalar _tmp88 = _tmp55 * _tmp73;
  const Scalar _tmp89 = -_tmp73 * _tmp74 + _tmp88;
  const Scalar _tmp90 = _tmp49 * _tmp55;
  const Scalar _tmp91 = _tmp47 * _tmp90;
  const Scalar _tmp92 = -_tmp50 * _tmp64 - _tmp80 * _tmp89 - _tmp81 * _tmp87 + _tmp91;
  const Scalar _tmp93 = Scalar(1.0) / (_tmp92);
  const Scalar _tmp94 = _tmp82 * _tmp87 + _tmp85;
  const Scalar _tmp95 = 0;
  const Scalar _tmp96 = _tmp93 * _tmp95;
  const Scalar _tmp97 = _tmp80 * _tmp96;
  const Scalar _tmp98 = _tmp50 * _tmp96 - _tmp73 * _tmp97;
  const Scalar _tmp99 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp100 = std::pow(_tmp60, Scalar(2));
  const Scalar _tmp101 = _tmp100 + std::pow(_tmp58, Scalar(2));
  const Scalar _tmp102 = std::sqrt(_tmp101);
  const Scalar _tmp103 = _tmp102 * _tmp61;
  const Scalar _tmp104 = _tmp103 * _tmp99;
  const Scalar _tmp105 = Scalar(1.0) / (_tmp102);
  const Scalar _tmp106 = _tmp105 * _tmp59;
  const Scalar _tmp107 = _tmp105 * _tmp60;
  const Scalar _tmp108 = _tmp106 * _tmp58 - _tmp107 * _tmp57;
  const Scalar _tmp109 = _tmp103 * _tmp108;
  const Scalar _tmp110 = _tmp109 * _tmp73 + _tmp66 * _tmp73 - _tmp69 * _tmp75;
  const Scalar _tmp111 = _tmp36 * _tmp49;
  const Scalar _tmp112 = _tmp109 * _tmp50 - _tmp110 * _tmp80 + _tmp111 * _tmp47 - _tmp46 * _tmp65;
  const Scalar _tmp113 = Scalar(1.0) / (_tmp112);
  const Scalar _tmp114 = Scalar(1.0) * _tmp113;
  const Scalar _tmp115 = Scalar(1.0) * _tmp78;
  const Scalar _tmp116 = _tmp113 * _tmp115;
  const Scalar _tmp117 = _tmp116 * _tmp79;
  const Scalar _tmp118 = _tmp114 * _tmp50 - _tmp117 * _tmp73;
  const Scalar _tmp119 = _tmp39 + _tmp68;
  const Scalar _tmp120 = _tmp119 + p_init0 + Scalar(-2.5202214700000001);
  const Scalar _tmp121 = _tmp27 + _tmp35;
  const Scalar _tmp122 = _tmp121 + p_init1 + Scalar(8.3888750099999996);
  const Scalar _tmp123 = std::pow(_tmp120, Scalar(2)) + std::pow(_tmp122, Scalar(2));
  const Scalar _tmp124 = std::pow(_tmp123, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp125 = _tmp121 * _tmp124;
  const Scalar _tmp126 = _tmp122 * _tmp124;
  const Scalar _tmp127 = _tmp119 * _tmp126 - _tmp120 * _tmp125;
  const Scalar _tmp128 = _tmp127 * fh1;
  const Scalar _tmp129 = _tmp103 * _tmp128;
  const Scalar _tmp130 = _tmp115 * _tmp76;
  const Scalar _tmp131 = -_tmp115 * _tmp89 + _tmp130 * _tmp87;
  const Scalar _tmp132 = _tmp112 * _tmp93;
  const Scalar _tmp133 = -_tmp110 * _tmp115 - _tmp131 * _tmp132;
  const Scalar _tmp134 = _tmp113 * _tmp92;
  const Scalar _tmp135 = _tmp131 + _tmp133 * _tmp134;
  const Scalar _tmp136 = _tmp50 * _tmp93;
  const Scalar _tmp137 = _tmp79 * _tmp93;
  const Scalar _tmp138 = -_tmp135 * _tmp137 + Scalar(1.0);
  const Scalar _tmp139 = _tmp73 * _tmp78;
  const Scalar _tmp140 = _tmp135 * _tmp136 + _tmp138 * _tmp139;
  const Scalar _tmp141 = _tmp103 * fh1;
  const Scalar _tmp142 = _tmp126 * _tmp141;
  const Scalar _tmp143 = _tmp62 * _tmp78;
  const Scalar _tmp144 = _tmp78 * _tmp89;
  const Scalar _tmp145 = _tmp76 * _tmp78;
  const Scalar _tmp146 = _tmp145 * _tmp62 + _tmp63;
  const Scalar _tmp147 = _tmp146 * _tmp84;
  const Scalar _tmp148 = _tmp144 * _tmp62 - _tmp147 * _tmp86 - _tmp55;
  const Scalar _tmp149 = -_tmp109 + _tmp110 * _tmp143 - _tmp132 * _tmp148;
  const Scalar _tmp150 = _tmp134 * _tmp149 + _tmp148;
  const Scalar _tmp151 = -_tmp137 * _tmp150 - _tmp62;
  const Scalar _tmp152 = _tmp136 * _tmp150 + _tmp139 * _tmp151 + Scalar(1.0);
  const Scalar _tmp153 = _tmp120 * _tmp124;
  const Scalar _tmp154 = _tmp141 * _tmp153;
  const Scalar _tmp155 = (Scalar(1) / Scalar(2)) / _tmp1;
  const Scalar _tmp156 = _tmp0 * _tmp155;
  const Scalar _tmp157 = _tmp155 * rz;
  const Scalar _tmp158 = _tmp157 * ry;
  const Scalar _tmp159 = _tmp157 * rx;
  const Scalar _tmp160 = _tmp6 / (_tmp1 * std::sqrt(_tmp1));
  const Scalar _tmp161 = _tmp0 * _tmp160;
  const Scalar _tmp162 = _tmp160 * rz;
  const Scalar _tmp163 = _tmp162 * rx;
  const Scalar _tmp164 = _tmp162 * ry;
  const Scalar _tmp165 = -Scalar(1) / Scalar(2) * _tmp11 + _tmp14 + _tmp156 * _tmp5 -
                         _tmp158 * _tmp19 + _tmp159 * _tmp22 - _tmp161 * rot_init_y -
                         _tmp163 * rot_init_w + _tmp164 * rot_init_z;
  const Scalar _tmp166 = _tmp16 * _tmp165;
  const Scalar _tmp167 = Scalar(0.83999999999999997) * _tmp166;
  const Scalar _tmp168 = -_tmp167;
  const Scalar _tmp169 = _tmp13 * _tmp158 + _tmp156 * _tmp22 - _tmp159 * _tmp5 -
                         _tmp161 * rot_init_w + _tmp163 * rot_init_y - _tmp164 * rot_init_x -
                         Scalar(1) / Scalar(2) * _tmp23 + _tmp8;
  const Scalar _tmp170 = _tmp169 * _tmp21;
  const Scalar _tmp171 = Scalar(0.83999999999999997) * _tmp170;
  const Scalar _tmp172 = -_tmp171;
  const Scalar _tmp173 = -_tmp13 * _tmp159 - _tmp156 * _tmp19 - _tmp158 * _tmp5 +
                         _tmp161 * rot_init_z + _tmp163 * rot_init_x + _tmp164 * rot_init_y -
                         Scalar(1) / Scalar(2) * _tmp20 - _tmp9;
  const Scalar _tmp174 = _tmp173 * _tmp21;
  const Scalar _tmp175 = Scalar(0.41999999999999998) * _tmp174;
  const Scalar _tmp176 = Scalar(0.41999999999999998) * _tmp169;
  const Scalar _tmp177 = _tmp176 * _tmp24;
  const Scalar _tmp178 = _tmp175 + _tmp177;
  const Scalar _tmp179 = _tmp172 + _tmp178;
  const Scalar _tmp180 = _tmp12 * _tmp165;
  const Scalar _tmp181 = Scalar(0.41999999999999998) * _tmp180;
  const Scalar _tmp182 = -_tmp10 - _tmp13 * _tmp156 - Scalar(1) / Scalar(2) * _tmp15 +
                         _tmp158 * _tmp22 + _tmp159 * _tmp19 + _tmp161 * rot_init_x -
                         _tmp163 * rot_init_z - _tmp164 * rot_init_w;
  const Scalar _tmp183 = _tmp16 * _tmp182;
  const Scalar _tmp184 = Scalar(0.41999999999999998) * _tmp183;
  const Scalar _tmp185 = _tmp181 + _tmp184;
  const Scalar _tmp186 = Scalar(0.021999999999999999) * _tmp173;
  const Scalar _tmp187 = Scalar(0.021999999999999999) * _tmp21;
  const Scalar _tmp188 = Scalar(0.021999999999999999) * _tmp169;
  const Scalar _tmp189 = Scalar(0.021999999999999999) * _tmp24;
  const Scalar _tmp190 =
      -_tmp12 * _tmp188 + _tmp16 * _tmp186 + _tmp165 * _tmp189 - _tmp182 * _tmp187;
  const Scalar _tmp191 = _tmp185 + _tmp190;
  const Scalar _tmp192 = _tmp168 + _tmp179 + _tmp191;
  const Scalar _tmp193 = -_tmp175 - _tmp177;
  const Scalar _tmp194 = _tmp172 + _tmp193;
  const Scalar _tmp195 = _tmp12 * _tmp182;
  const Scalar _tmp196 = Scalar(0.83999999999999997) * _tmp195;
  const Scalar _tmp197 =
      -_tmp12 * _tmp186 - _tmp16 * _tmp188 - _tmp165 * _tmp187 - _tmp182 * _tmp189;
  const Scalar _tmp198 = -_tmp196 + _tmp197;
  const Scalar _tmp199 = _tmp185 + _tmp194 + _tmp198;
  const Scalar _tmp200 =
      (2 * _tmp192 * _tmp67 + 2 * _tmp199 * _tmp70) / (_tmp71 * std::sqrt(_tmp71));
  const Scalar _tmp201 = (Scalar(1) / Scalar(2)) * _tmp200;
  const Scalar _tmp202 = _tmp201 * _tmp67;
  const Scalar _tmp203 = _tmp199 * _tmp72;
  const Scalar _tmp204 = _tmp201 * _tmp70;
  const Scalar _tmp205 = _tmp192 * _tmp72;
  const Scalar _tmp206 = Scalar(1.0) / (_tmp100);
  const Scalar _tmp207 = _tmp171 + _tmp193;
  const Scalar _tmp208 = _tmp196 + _tmp197;
  const Scalar _tmp209 = _tmp185 + _tmp207 + _tmp208;
  const Scalar _tmp210 = _tmp209 * _tmp58;
  const Scalar _tmp211 = _tmp206 * _tmp210;
  const Scalar _tmp212 = -_tmp181 - _tmp184;
  const Scalar _tmp213 = _tmp190 + _tmp212;
  const Scalar _tmp214 = _tmp168 + _tmp194 + _tmp213;
  const Scalar _tmp215 = _tmp214 * _tmp61;
  const Scalar _tmp216 = (_tmp202 + _tmp203 * _tmp62 - _tmp204 * _tmp62 - _tmp205 -
                          _tmp211 * _tmp73 + _tmp215 * _tmp73) /
                         std::pow(_tmp77, Scalar(2));
  const Scalar _tmp217 = _tmp216 * _tmp79;
  const Scalar _tmp218 = _tmp167 + _tmp207 + _tmp213;
  const Scalar _tmp219 = _tmp171 + _tmp178;
  const Scalar _tmp220 = _tmp208 + _tmp212 + _tmp219;
  const Scalar _tmp221 =
      (2 * _tmp218 * _tmp37 + 2 * _tmp220 * _tmp47) / (_tmp48 * std::sqrt(_tmp48));
  const Scalar _tmp222 = Scalar(0.5) * _tmp113;
  const Scalar _tmp223 = (Scalar(1) / Scalar(2)) * _tmp221;
  const Scalar _tmp224 = _tmp223 * _tmp37;
  const Scalar _tmp225 = _tmp218 * _tmp49;
  const Scalar _tmp226 = _tmp220 * _tmp49;
  const Scalar _tmp227 = _tmp223 * _tmp47;
  const Scalar _tmp228 = -_tmp211 * _tmp50 + _tmp215 * _tmp50 + _tmp224 - _tmp225 +
                         _tmp226 * _tmp62 - _tmp227 * _tmp62;
  const Scalar _tmp229 = _tmp116 * _tmp228;
  const Scalar _tmp230 = _tmp209 * _tmp60 + _tmp214 * _tmp58;
  const Scalar _tmp231 = _tmp105 * _tmp230 * _tmp61;
  const Scalar _tmp232 = _tmp108 * _tmp231;
  const Scalar _tmp233 = _tmp102 * _tmp206 * _tmp209;
  const Scalar _tmp234 = _tmp108 * _tmp233;
  const Scalar _tmp235 = _tmp230 / (_tmp101 * std::sqrt(_tmp101));
  const Scalar _tmp236 =
      _tmp103 * (-_tmp105 * _tmp209 * _tmp57 + _tmp105 * _tmp210 + _tmp106 * _tmp214 -
                 _tmp107 * _tmp214 + _tmp235 * _tmp57 * _tmp60 - _tmp235 * _tmp58 * _tmp59);
  const Scalar _tmp237 = _tmp109 * _tmp203 - _tmp109 * _tmp204 - _tmp199 * _tmp75 +
                         _tmp202 * _tmp69 + _tmp203 * _tmp66 - _tmp204 * _tmp66 - _tmp205 * _tmp69 +
                         _tmp205 * _tmp70 + _tmp232 * _tmp73 - _tmp234 * _tmp73 + _tmp236 * _tmp73;
  const Scalar _tmp238 = _tmp228 * _tmp78;
  const Scalar _tmp239 = _tmp109 * _tmp226 - _tmp109 * _tmp227 + _tmp110 * _tmp217 -
                         _tmp110 * _tmp238 + _tmp111 * _tmp220 - _tmp220 * _tmp65 +
                         _tmp224 * _tmp46 - _tmp225 * _tmp46 + _tmp225 * _tmp47 - _tmp227 * _tmp36 +
                         _tmp232 * _tmp50 - _tmp234 * _tmp50 + _tmp236 * _tmp50 - _tmp237 * _tmp80;
  const Scalar _tmp240 = _tmp239 / std::pow(_tmp112, Scalar(2));
  const Scalar _tmp241 = _tmp115 * _tmp240 * _tmp79;
  const Scalar _tmp242 = Scalar(1.0) * _tmp240;
  const Scalar _tmp243 = _tmp167 + _tmp191 + _tmp219;
  const Scalar _tmp244 = _tmp124 * _tmp243;
  const Scalar _tmp245 = _tmp140 * _tmp141;
  const Scalar _tmp246 = _tmp118 * _tmp128;
  const Scalar _tmp247 = _tmp233 * fh1;
  const Scalar _tmp248 = _tmp126 * _tmp140;
  const Scalar _tmp249 = _tmp231 * fh1;
  const Scalar _tmp250 = _tmp179 + _tmp198 + _tmp212;
  const Scalar _tmp251 = _tmp124 * _tmp250;
  const Scalar _tmp252 = _tmp141 * _tmp152;
  const Scalar _tmp253 = Scalar(0.043999999999999997) * _tmp166;
  const Scalar _tmp254 = Scalar(0.043999999999999997) * _tmp195;
  const Scalar _tmp255 = _tmp253 + _tmp254;
  const Scalar _tmp256 = Scalar(0.41999999999999998) * _tmp173;
  const Scalar _tmp257 = _tmp12 * _tmp256;
  const Scalar _tmp258 = Scalar(0.41999999999999998) * _tmp21;
  const Scalar _tmp259 = _tmp165 * _tmp258;
  const Scalar _tmp260 = _tmp16 * _tmp176;
  const Scalar _tmp261 = Scalar(0.41999999999999998) * _tmp24;
  const Scalar _tmp262 = _tmp182 * _tmp261;
  const Scalar _tmp263 = _tmp257 - _tmp259 - _tmp260 + _tmp262;
  const Scalar _tmp264 = _tmp16 * _tmp256;
  const Scalar _tmp265 = _tmp182 * _tmp258;
  const Scalar _tmp266 = _tmp12 * _tmp176;
  const Scalar _tmp267 = _tmp165 * _tmp261;
  const Scalar _tmp268 = -_tmp264 - _tmp265 - _tmp266 - _tmp267;
  const Scalar _tmp269 = _tmp255 + _tmp263 + _tmp268;
  const Scalar _tmp270 = _tmp255 + _tmp264 + _tmp265 + _tmp266 + _tmp267;
  const Scalar _tmp271 = _tmp263 + _tmp270;
  const Scalar _tmp272 =
      Scalar(1.6799999999999999) * _tmp170 + Scalar(1.6799999999999999) * _tmp195;
  const Scalar _tmp273 = _tmp272 * _tmp84;
  const Scalar _tmp274 = _tmp271 * _tmp62;
  const Scalar _tmp275 = -_tmp257 + _tmp259 + _tmp260 - _tmp262;
  const Scalar _tmp276 = _tmp270 + _tmp275;
  const Scalar _tmp277 = -_tmp202 * _tmp74 - _tmp203 * _tmp63 + _tmp204 * _tmp63 +
                         _tmp205 * _tmp74 + _tmp211 * _tmp88 - _tmp215 * _tmp88 - _tmp274 * _tmp73 +
                         _tmp276 * _tmp75;
  const Scalar _tmp278 = _tmp203 * _tmp55 - _tmp203 * _tmp74 - _tmp204 * _tmp55 + _tmp204 * _tmp74 +
                         _tmp271 * _tmp73 - _tmp276 * _tmp73;
  const Scalar _tmp279 =
      _tmp86 *
      (Scalar(0.83999999999999997) * _tmp169 * _tmp24 + Scalar(0.83999999999999997) * _tmp174 +
       Scalar(0.83999999999999997) * _tmp180 + Scalar(0.83999999999999997) * _tmp183) /
      std::pow(_tmp83, Scalar(2));
  const Scalar _tmp280 =
      _tmp217 * _tmp89 + _tmp220 * _tmp90 - _tmp226 * _tmp64 - _tmp227 * _tmp55 + _tmp227 * _tmp64 -
      _tmp238 * _tmp89 - _tmp269 * _tmp50 + _tmp271 * _tmp50 - _tmp273 * _tmp81 - _tmp278 * _tmp80 +
      _tmp279 * _tmp81 -
      _tmp87 * (_tmp211 * _tmp91 - _tmp215 * _tmp91 + _tmp217 * _tmp76 - _tmp224 * _tmp64 +
                _tmp225 * _tmp64 - _tmp226 * _tmp63 + _tmp227 * _tmp63 - _tmp238 * _tmp76 +
                _tmp269 * _tmp65 - _tmp274 * _tmp50 - _tmp277 * _tmp80);
  const Scalar _tmp281 = _tmp280 / std::pow(_tmp92, Scalar(2));
  const Scalar _tmp282 = _tmp281 * _tmp95;
  const Scalar _tmp283 = _tmp216 * _tmp73;
  const Scalar _tmp284 = _tmp137 * _tmp95;
  const Scalar _tmp285 = (Scalar(1) / Scalar(2)) * (2 * _tmp120 * _tmp250 + 2 * _tmp122 * _tmp243) /
                         (_tmp123 * std::sqrt(_tmp123));
  const Scalar _tmp286 = _tmp122 * _tmp285;
  const Scalar _tmp287 = _tmp120 * _tmp285;
  const Scalar _tmp288 = _tmp119 * _tmp244 - _tmp119 * _tmp286 - _tmp120 * _tmp244 +
                         _tmp121 * _tmp287 - _tmp125 * _tmp250 + _tmp126 * _tmp250;
  const Scalar _tmp289 = _tmp152 * _tmp153;
  const Scalar _tmp290 = _tmp240 * _tmp92;
  const Scalar _tmp291 = _tmp113 * _tmp280;
  const Scalar _tmp292 = _tmp216 * _tmp62;
  const Scalar _tmp293 = _tmp112 * _tmp281;
  const Scalar _tmp294 = _tmp239 * _tmp93;
  const Scalar _tmp295 =
      _tmp143 * _tmp278 - _tmp144 * _tmp211 + _tmp144 * _tmp215 + _tmp146 * _tmp279 -
      _tmp147 * _tmp272 - _tmp253 - _tmp254 + _tmp268 + _tmp275 - _tmp292 * _tmp89 -
      _tmp87 * (_tmp143 * _tmp277 - _tmp145 * _tmp211 + _tmp145 * _tmp215 - _tmp211 * _tmp55 +
                _tmp215 * _tmp55 + _tmp274 - _tmp292 * _tmp76);
  const Scalar _tmp296 = _tmp110 * _tmp78;
  const Scalar _tmp297 = _tmp134 * (-_tmp110 * _tmp292 - _tmp132 * _tmp295 + _tmp143 * _tmp237 +
                                    _tmp148 * _tmp293 - _tmp148 * _tmp294 - _tmp211 * _tmp296 +
                                    _tmp215 * _tmp296 - _tmp232 + _tmp234 - _tmp236) -
                         _tmp149 * _tmp290 + _tmp149 * _tmp291 + _tmp295;
  const Scalar _tmp298 = _tmp203 * _tmp78;
  const Scalar _tmp299 = _tmp281 * _tmp79;
  const Scalar _tmp300 = _tmp228 * _tmp93;
  const Scalar _tmp301 =
      -_tmp137 * _tmp297 + _tmp150 * _tmp299 - _tmp150 * _tmp300 + _tmp211 - _tmp215;
  const Scalar _tmp302 = _tmp227 * _tmp93;
  const Scalar _tmp303 = _tmp281 * _tmp50;
  const Scalar _tmp304 = _tmp226 * _tmp93;
  const Scalar _tmp305 = _tmp204 * _tmp78;
  const Scalar _tmp306 = _tmp98 * _tmp99;
  const Scalar _tmp307 = Scalar(1.0) * _tmp216;
  const Scalar _tmp308 = _tmp115 * _tmp277 * _tmp87 - _tmp115 * _tmp278 + _tmp130 * _tmp273 -
                         _tmp130 * _tmp279 - _tmp307 * _tmp76 * _tmp87 + _tmp307 * _tmp89;
  const Scalar _tmp309 = -_tmp133 * _tmp290 + _tmp133 * _tmp291 +
                         _tmp134 * (_tmp110 * _tmp307 - _tmp115 * _tmp237 + _tmp131 * _tmp293 -
                                    _tmp131 * _tmp294 - _tmp132 * _tmp308) +
                         _tmp308;
  const Scalar _tmp310 = _tmp135 * _tmp299 - _tmp135 * _tmp300 - _tmp137 * _tmp309;
  const Scalar _tmp311 = _tmp96 * _tmp99;
  const Scalar _tmp312 = _tmp78 * fh1;
  const Scalar _tmp313 = _tmp124 * _tmp312;
  const Scalar _tmp314 = _tmp151 * _tmp313;
  const Scalar _tmp315 = _tmp126 * _tmp312;
  const Scalar _tmp316 = _tmp216 * fh1;
  const Scalar _tmp317 = _tmp138 * _tmp312;
  const Scalar _tmp318 = _tmp282 * _tmp99;
  const Scalar _tmp319 = _tmp114 * fh1;
  const Scalar _tmp320 = _tmp127 * _tmp319;
  const Scalar _tmp321 = _tmp93 * fh1;
  const Scalar _tmp322 = _tmp135 * _tmp321;
  const Scalar _tmp323 = _tmp150 * _tmp321;
  const Scalar _tmp324 = _tmp281 * fh1;

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) = 0;
  _res(1, 0) =
      -(-_tmp104 * (-_tmp203 * _tmp97 + _tmp204 * _tmp97 + _tmp226 * _tmp96 - _tmp227 * _tmp96 -
                    _tmp238 * _tmp73 * _tmp96 - _tmp282 * _tmp50 + _tmp282 * _tmp73 * _tmp80 +
                    _tmp283 * _tmp284) -
        _tmp118 * _tmp141 * _tmp288 -
        _tmp129 * (_tmp114 * _tmp217 * _tmp73 + _tmp114 * _tmp226 - _tmp117 * _tmp203 +
                   _tmp200 * _tmp222 * _tmp70 * _tmp80 - _tmp221 * _tmp222 * _tmp47 -
                   _tmp229 * _tmp73 + _tmp241 * _tmp73 - _tmp242 * _tmp50) -
        _tmp142 * (-_tmp135 * _tmp302 - _tmp135 * _tmp303 + _tmp135 * _tmp304 + _tmp136 * _tmp309 -
                   _tmp138 * _tmp283 + _tmp138 * _tmp298 - _tmp138 * _tmp305 + _tmp139 * _tmp310) -
        _tmp154 * (_tmp136 * _tmp297 + _tmp139 * _tmp301 - _tmp150 * _tmp302 - _tmp150 * _tmp303 +
                   _tmp150 * _tmp304 - _tmp151 * _tmp283 + _tmp151 * _tmp298 - _tmp151 * _tmp305) -
        _tmp231 * _tmp246 - _tmp231 * _tmp306 + _tmp233 * _tmp246 + _tmp233 * _tmp306 -
        _tmp244 * _tmp245 + _tmp245 * _tmp286 + _tmp247 * _tmp248 + _tmp247 * _tmp289 -
        _tmp248 * _tmp249 - _tmp249 * _tmp289 - _tmp251 * _tmp252 + _tmp252 * _tmp287) *
      std::exp(_tmp104 * _tmp98 + _tmp118 * _tmp129 + _tmp140 * _tmp142 + _tmp152 * _tmp154);
  _res(2, 0) =
      -(-_tmp117 * _tmp288 * fh1 + _tmp120 * _tmp301 * _tmp313 - _tmp126 * _tmp138 * _tmp316 -
        _tmp128 * _tmp229 + _tmp128 * _tmp241 - _tmp151 * _tmp153 * _tmp316 -
        _tmp151 * _tmp287 * _tmp312 + _tmp216 * _tmp284 * _tmp99 + _tmp217 * _tmp320 -
        _tmp238 * _tmp311 + _tmp244 * _tmp317 + _tmp250 * _tmp314 - _tmp286 * _tmp317 +
        _tmp310 * _tmp315 + _tmp318 * _tmp80) *
      std::exp(_tmp117 * _tmp128 - _tmp120 * _tmp314 - _tmp138 * _tmp315 + _tmp311 * _tmp80);
  _res(3, 0) =
      -(-_tmp126 * _tmp135 * _tmp324 + _tmp126 * _tmp309 * _tmp321 - _tmp128 * _tmp242 -
        _tmp150 * _tmp153 * _tmp324 + _tmp153 * _tmp297 * _tmp321 + _tmp244 * _tmp322 +
        _tmp251 * _tmp323 - _tmp286 * _tmp322 - _tmp287 * _tmp323 + _tmp288 * _tmp319 - _tmp318) *
      std::exp(-_tmp126 * _tmp322 - _tmp153 * _tmp323 - _tmp311 - _tmp320);

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
