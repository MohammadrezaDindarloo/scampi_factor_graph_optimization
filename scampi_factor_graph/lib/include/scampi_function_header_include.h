// ************************************ FK ************************************
// cost1 ---------
#include "FK_residual_func_cost1.h"
#include "FK_residual_func_cost1_wrt_fh1.h"
#include "FK_residual_func_cost1_wrt_fv1.h"
#include "FK_residual_func_cost1_wrt_rx.h"
#include "FK_residual_func_cost1_wrt_ry.h"
#include "FK_residual_func_cost1_wrt_rz.h"
#include "FK_residual_func_cost1_wrt_tx.h"
#include "FK_residual_func_cost1_wrt_ty.h"
#include "FK_residual_func_cost1_wrt_tz.h"
// cost2 ---------
#include "FK_residual_func_cost2.h"
#include "FK_residual_func_cost2_wrt_fh1.h"
#include "FK_residual_func_cost2_wrt_fv1.h"
#include "FK_residual_func_cost2_wrt_rx.h"
#include "FK_residual_func_cost2_wrt_ry.h"
#include "FK_residual_func_cost2_wrt_rz.h"
#include "FK_residual_func_cost2_wrt_tx.h"
#include "FK_residual_func_cost2_wrt_ty.h"
#include "FK_residual_func_cost2_wrt_tz.h"
// cost3 ---------
#include "FK_residual_func_cost3.h"
#include "FK_residual_func_cost3_wrt_fh1.h"
#include "FK_residual_func_cost3_wrt_fv1.h"
#include "FK_residual_func_cost3_wrt_rx.h"
#include "FK_residual_func_cost3_wrt_ry.h"
#include "FK_residual_func_cost3_wrt_rz.h"
#include "FK_residual_func_cost3_wrt_tx.h"
#include "FK_residual_func_cost3_wrt_ty.h"
#include "FK_residual_func_cost3_wrt_tz.h"
// ************************************ IK ************************************
// cost1 ---------
// l1 ------------
#include "IK_residual_func_cost1_l1.h"
#include "IK_residual_func_cost1_wrt_fh1_l1.h"
#include "IK_residual_func_cost1_wrt_fv1_l1.h"
#include "IK_residual_func_cost1_wrt_rx_l1.h"
#include "IK_residual_func_cost1_wrt_ry_l1.h"
#include "IK_residual_func_cost1_wrt_rz_l1.h"
// cost2 ---------
// l1 ------------
#include "IK_residual_func_cost2_l1.h"
#include "IK_residual_func_cost2_wrt_fh1_l1.h"
#include "IK_residual_func_cost2_wrt_fv1_l1.h"
#include "IK_residual_func_cost2_wrt_rx_l1.h"
#include "IK_residual_func_cost2_wrt_ry_l1.h"
#include "IK_residual_func_cost2_wrt_rz_l1.h"
// cost3 ---------
// l1 ------------
#include "IK_residual_func_cost3_l1.h"
#include "IK_residual_func_cost3_wrt_fh1_l1.h"
#include "IK_residual_func_cost3_wrt_fv1_l1.h"
#include "IK_residual_func_cost3_wrt_rx_l1.h"
#include "IK_residual_func_cost3_wrt_ry_l1.h"
#include "IK_residual_func_cost3_wrt_rz_l1.h"
// cost1 ---------
// l2 ------------
#include "IK_residual_func_cost1_l2.h"
#include "IK_residual_func_cost1_wrt_fh1_l2.h"
#include "IK_residual_func_cost1_wrt_fv1_l2.h"
#include "IK_residual_func_cost1_wrt_rx_l2.h"
#include "IK_residual_func_cost1_wrt_ry_l2.h"
#include "IK_residual_func_cost1_wrt_rz_l2.h"
// cost2 ---------
// l2 ------------
#include "IK_residual_func_cost2_l2.h"
#include "IK_residual_func_cost2_wrt_fh1_l2.h"
#include "IK_residual_func_cost2_wrt_fv1_l2.h"
#include "IK_residual_func_cost2_wrt_rx_l2.h"
#include "IK_residual_func_cost2_wrt_ry_l2.h"
#include "IK_residual_func_cost2_wrt_rz_l2.h"
// cost3 ---------
// l2 ------------
#include "IK_residual_func_cost3_l2.h"
#include "IK_residual_func_cost3_wrt_fh1_l2.h"
#include "IK_residual_func_cost3_wrt_fv1_l2.h"
#include "IK_residual_func_cost3_wrt_rx_l2.h"
#include "IK_residual_func_cost3_wrt_ry_l2.h"
#include "IK_residual_func_cost3_wrt_rz_l2.h"
// cost1 ---------
// l3 ------------
#include "IK_residual_func_cost1_l3.h"
#include "IK_residual_func_cost1_wrt_fh1_l3.h"
#include "IK_residual_func_cost1_wrt_fv1_l3.h"
#include "IK_residual_func_cost1_wrt_rx_l3.h"
#include "IK_residual_func_cost1_wrt_ry_l3.h"
#include "IK_residual_func_cost1_wrt_rz_l3.h"
// cost2 ---------
// l3 ------------
#include "IK_residual_func_cost2_l3.h"
#include "IK_residual_func_cost2_wrt_fh1_l3.h"
#include "IK_residual_func_cost2_wrt_fv1_l3.h"
#include "IK_residual_func_cost2_wrt_rx_l3.h"
#include "IK_residual_func_cost2_wrt_ry_l3.h"
#include "IK_residual_func_cost2_wrt_rz_l3.h"
// cost3 ---------
// l3 ------------
#include "IK_residual_func_cost3_l3.h"
#include "IK_residual_func_cost3_wrt_fh1_l3.h"
#include "IK_residual_func_cost3_wrt_fv1_l3.h"
#include "IK_residual_func_cost3_wrt_rx_l3.h"
#include "IK_residual_func_cost3_wrt_ry_l3.h"
#include "IK_residual_func_cost3_wrt_rz_l3.h"
// cost1 ---------
// l4 ------------
#include "IK_residual_func_cost1_l4.h"
#include "IK_residual_func_cost1_wrt_fh1_l4.h"
#include "IK_residual_func_cost1_wrt_fv1_l4.h"
#include "IK_residual_func_cost1_wrt_rx_l4.h"
#include "IK_residual_func_cost1_wrt_ry_l4.h"
#include "IK_residual_func_cost1_wrt_rz_l4.h"
// cost2 ---------
// l4 ------------
#include "IK_residual_func_cost2_l4.h"
#include "IK_residual_func_cost2_wrt_fh1_l4.h"
#include "IK_residual_func_cost2_wrt_fv1_l4.h"
#include "IK_residual_func_cost2_wrt_rx_l4.h"
#include "IK_residual_func_cost2_wrt_ry_l4.h"
#include "IK_residual_func_cost2_wrt_rz_l4.h"
// cost3 ---------
// l4 ------------
#include "IK_residual_func_cost3_l4.h"
#include "IK_residual_func_cost3_wrt_fh1_l4.h"
#include "IK_residual_func_cost3_wrt_fv1_l4.h"
#include "IK_residual_func_cost3_wrt_rx_l4.h"
#include "IK_residual_func_cost3_wrt_ry_l4.h"
#include "IK_residual_func_cost3_wrt_rz_l4.h"