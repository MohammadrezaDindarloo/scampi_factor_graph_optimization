#include <iostream>
#include "scampi_fg_IK.h"
#include "scampi_fg_utils.h"
#include "scampi_fg_FK.h"

using namespace std;
using namespace gtsam;

// ****************************************** IK optimization *******************************************
// a class fot kepping cable parameters
class CableRobotParams
{
  public:
    CableRobotParams(const double g_c, const double f_g): f_g_(f_g), g_c_(g_c) {}

    Eigen::Vector3d p1_, p2_, p3_, p4_;
    Eigen::Vector3d b1_, b2_, b3_, b4_;
    Eigen::Vector3d r_to_cog_;
    
    double g_c_, f_g_;
    void setPulleyPoses(Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Vector3d p3, Eigen::Vector3d p4)
    {
      p1_ = p1;
      p2_ = p2;
      p3_ = p3;
      p4_ = p4;
    }
    void setEEAnchors(Eigen::Vector3d b1, Eigen::Vector3d b2, Eigen::Vector3d b3, Eigen::Vector3d b4)
    {
      b1_ = b1;
      b2_ = b2;
      b3_ = b3;
      b4_ = b4;
    }
    void setCog(Eigen::Vector3d r_to_cog)
    {
      r_to_cog_ = r_to_cog;
    }
};

// a function to create ivnerse kinematic factor graph 
Values inverse_kinematic_factor_graph_optimizer(double p_init_0, double p_init_1, double p_init_2,
                              double rot_init_x, double rot_init_y, double rot_init_z, double rot_init_w, 
                              int largest_cable,
                              double init_h1, double init_v1, double init_rx,  double init_ry,  double init_rz)
{
    NonlinearFactorGraph graph;
    Values initial_estimate;

    auto Sensor_noiseModel_cost1 = gtsam::noiseModel::Isotropic::Sigma(4, 0.10);
    auto Sensor_noiseModel_cost2 = gtsam::noiseModel::Isotropic::Sigma(4, 0.05);
    auto Sensor_noiseModel_cost3 = gtsam::noiseModel::Isotropic::Sigma(4, 0.01);

    graph.add(std::make_shared<IK_factor_graoh_cost1>(Symbol('h', 1), Symbol('v', 1), Symbol('x', 1), Symbol('x', 2), Symbol('x', 3), p_init_0, p_init_1, p_init_2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, largest_cable, Sensor_noiseModel_cost1));
    graph.add(std::make_shared<IK_factor_graoh_cost2>(Symbol('h', 1), Symbol('v', 1), Symbol('x', 1), Symbol('x', 2), Symbol('x', 3), p_init_0, p_init_1, p_init_2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, largest_cable, Sensor_noiseModel_cost2));
    graph.add(std::make_shared<IK_factor_graoh_cost3>(Symbol('h', 1), Symbol('v', 1), Symbol('x', 1), Symbol('x', 2), Symbol('x', 3), p_init_0, p_init_1, p_init_2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, largest_cable, Sensor_noiseModel_cost3));

    initial_estimate.insert(Symbol('h', 1), init_h1);
    initial_estimate.insert(Symbol('v', 1), init_v1);
    initial_estimate.insert(Symbol('x', 1), init_rx);
    initial_estimate.insert(Symbol('x', 2), init_ry);
    initial_estimate.insert(Symbol('x', 3), init_rz);

    LevenbergMarquardtOptimizer optimizer(graph, initial_estimate);
    Values result_LM = optimizer.optimize();
    std::cout << std::endl << "-------------------catenary result--------------------------" << std::endl;
    std::cout << std::endl << "inverse kinematic optimization error: " << optimizer.error() << std::endl;

    return result_LM;

}

// a function to hold parameters and invoke optimizer and back the optimized data
void ikSolver(CableRobotParams robot_params, 
              Eigen::Matrix3d rot_init, 
              Eigen::Vector3d p_platform,
              IKDataOut<double> *result)
{
    RobotParameters<double> params;
    //initilize the pulley locations
    params.pulleys.push_back(robot_params.p1_);
    params.pulleys.push_back(robot_params.p2_);
    params.pulleys.push_back(robot_params.p3_);
    params.pulleys.push_back(robot_params.p4_);
    params.r_to_cog = robot_params.r_to_cog_;

    //initilize cable attachement points
    params.ef_points.push_back(robot_params.b1_);
    params.ef_points.push_back(robot_params.b2_);
    params.ef_points.push_back(robot_params.b3_);
    params.ef_points.push_back(robot_params.b4_);
    params.g_c= robot_params.g_c_;
    params.f_g = robot_params.f_g_;

    //reorder the cable forces and choose the cable with largest length as the the first cable (for numerical stability)
    VectorXi reorder_idx(params.pulleys.size());
    RobotParameters<double> params_reord;
    RobotState<double> state;
    state.rot_platform = rot_init;
    state.p_platform = p_platform;
    changeOrderForSolver<double>(state, params, &params_reord, &reorder_idx);
    //Compute initil cable forces as starting points for the solver
    double fh0, fv0;
    computeInitCableForces<double>(&fh0, &fv0, p_platform, rot_init, params_reord);
    // Initialize the quaternion array
    double rotation_matrix_double[9];
    for (int i = 0; i < 9; ++i) {
        rotation_matrix_double[i] = rot_init.data()[i];
    }
    double quaternion[4];    
    ceres::RotationMatrixToQuaternion<double>(rotation_matrix_double, quaternion);
    double rot_init_x = quaternion[1];
    double rot_init_y = quaternion[2];
    double rot_init_z = quaternion[3];
    double rot_init_w = quaternion[0];
    // initial p_platfrom
    double p_init_0 = p_platform[0];
    double p_init_1 = p_platform[1];
    double p_init_2 = p_platform[2];
    // define largest cable
    int largest_cable = reorder_idx[0] + 1; 
    // initial values for variable 
    double init_h1 = fh0;
    double init_v1 = -fv0;
    double init_rx = 0.0001;
    double init_ry = 0.0001;
    double init_rz = 0.0001;
    // run optimization!
    Values optimization_result = inverse_kinematic_factor_graph_optimizer(p_init_0, p_init_1, p_init_2,
                                                        rot_init_x, rot_init_y, rot_init_z, rot_init_w, 
                                                        largest_cable,
                                                        init_h1, init_v1, init_rx, init_ry, init_rz);

    //harvest the results
    double fh = optimization_result.at<double>(Symbol('h', 1)); //optimized horizontal force for the first cable
    double fv = optimization_result.at<double>(Symbol('v', 1)); //optimized vertical force for the first cable
    // Extract the optimized orientation matrix of the moving platform
    manif::SO3Tangentd xi;
    xi << optimization_result.at<double>(Symbol('x', 1)), optimization_result.at<double>(Symbol('x', 2)), optimization_result.at<double>(Symbol('x', 3));
    manif::SO3<double> rot_optimized = manif::exp(xi);
    Matrix3d rot_result = rot_init * rot_optimized.rotation();
    result->rot_platform = rot_result;

    // Use the utils functions once again to compute the force in other cables and the catenary variables
    GeometricVariables<double> geom_vars;
    CatenaryVariables<double> cat_vars;

    state.rot_platform = rot_result; // Now we know the optimzed value of end-effector position so just use it

    getGeometricVariables<double>(state,params_reord,&geom_vars);
    getCableForces<double>(fh, fv, &state, params_reord,geom_vars);
    getCatenaryVariables<double>(state,params_reord, geom_vars,&cat_vars);
    //reverse the order of cables back to the normal configuration (base on notebook index)
    switch(reorder_idx[0])
    {
        case 0:
            reorder_idx[0] = 0;
            reorder_idx[1] = 3;
            reorder_idx[2] = 2;
            reorder_idx[3] = 1;
            break;
        case 1:
            reorder_idx[0] = 1;
            reorder_idx[1] = 3;
            reorder_idx[2] = 2;
            reorder_idx[3] = 0;
            break;
        case 2:
            reorder_idx[0] = 2;
            reorder_idx[1] = 3;
            reorder_idx[2] = 0;
            reorder_idx[3] = 1;
            break;
        case 3:
            reorder_idx[0] = 3;
            reorder_idx[1] = 2;
            reorder_idx[2] = 1;
            reorder_idx[3] = 0;
            break;
        default:
            break;
    }
    reverseOrderForSolver<double>(state, geom_vars, cat_vars, result, reorder_idx);
}

// a function to modify parameters, invoke optimizer and harvest the result
std::vector<MatrixXd> IK_Factor_Graph_Optimization(CableRobotParams robot_params, 
                                                   Eigen::Matrix3d rot_init, 
                                                   Eigen::Vector3d p_platform)
{
    std::vector<MatrixXd> results_list;
    IKDataOut<double> ik_result;
    ikSolver(robot_params, rot_init, p_platform, &ik_result);

    //Extract The results and return them as a list of matrices to python
    results_list.push_back(ik_result.rot_platform);
    //Catenary cable lengths  
    int N = 4;

    VectorXd lc_cat(N);
    lc_cat << ik_result.lc_cat[0], ik_result.lc_cat[1], ik_result.lc_cat[2], ik_result.lc_cat[3];
    results_list.push_back(lc_cat);

    //Cable Forces
    MatrixXd cable_forces(2,N);
    for(int i=0; i<N; i++)
        cable_forces.col(i) =  ik_result.cable_forces[i];
    results_list.push_back(cable_forces);

    //Catenary parameters
    //C1
    MatrixXd c1_(1, N);
    for(int i=0; i<N; i++)
        c1_(0,i) = ik_result.c1[i];
    results_list.push_back(c1_);

    //C2
    MatrixXd c2_(1, N);
    for(int i=0; i<N; i++)
        c2_(0,i) = ik_result.c2[i];
    results_list.push_back(c2_);

    //body coordinates represented in the world frame
    MatrixXd b_in_w_(3,N);
    for(int i=0; i<N; i++)
        b_in_w_.col(i) =  ik_result.b_in_w[i];
    results_list.push_back(b_in_w_);

    return results_list;

}

// ****************************************** FK optimization *******************************************
// a function to create forward kinematic factor graph 
Values forward_kinematic_factor_graph_optimizer(double lc0, double lc1, double lc2, double lc3,
                              double rot_init_x, double rot_init_y, double rot_init_z, double rot_init_w, 
                              double init_h1, double init_v1, double init_rx,  double init_ry,  double init_rz,
                              double init_tx, double init_ty, double init_tz)
{
    NonlinearFactorGraph graph;
    Values initial_estimate;

    auto Sensor_noiseModel_cost1 = gtsam::noiseModel::Isotropic::Sigma(4, 0.01);
    auto Sensor_noiseModel_cost2 = gtsam::noiseModel::Isotropic::Sigma(4, 0.01);
    auto Sensor_noiseModel_cost3 = gtsam::noiseModel::Isotropic::Sigma(4, 0.01);

    graph.add(std::make_shared<FK_factor_graoh_cost1>(Symbol('h', 1), Symbol('v', 1), Symbol('x', 1), Symbol('x', 2), Symbol('x', 3), Symbol('t', 1), Symbol('t', 2), Symbol('t', 3), lc0, lc1, lc2, lc3, rot_init_x, rot_init_y, rot_init_z, rot_init_w, Sensor_noiseModel_cost1));
    graph.add(std::make_shared<FK_factor_graoh_cost2>(Symbol('h', 1), Symbol('v', 1), Symbol('x', 1), Symbol('x', 2), Symbol('x', 3), Symbol('t', 1), Symbol('t', 2), Symbol('t', 3), lc0, lc1, lc2, lc3, rot_init_x, rot_init_y, rot_init_z, rot_init_w, Sensor_noiseModel_cost2));
    graph.add(std::make_shared<FK_factor_graoh_cost3>(Symbol('h', 1), Symbol('v', 1), Symbol('x', 1), Symbol('x', 2), Symbol('x', 3), Symbol('t', 1), Symbol('t', 2), Symbol('t', 3), lc0, lc1, lc2, lc3, rot_init_x, rot_init_y, rot_init_z, rot_init_w, Sensor_noiseModel_cost3));

    initial_estimate.insert(Symbol('h', 1), init_h1);
    initial_estimate.insert(Symbol('v', 1), init_v1);
    initial_estimate.insert(Symbol('x', 1), init_rx);
    initial_estimate.insert(Symbol('x', 2), init_ry);
    initial_estimate.insert(Symbol('x', 3), init_rz);
    initial_estimate.insert(Symbol('t', 1), init_tx);
    initial_estimate.insert(Symbol('t', 2), init_ty);
    initial_estimate.insert(Symbol('t', 3), init_tz);


    LevenbergMarquardtOptimizer optimizer(graph, initial_estimate);
    Values result_LM = optimizer.optimize();
    std::cout << std::endl << "-------------------catenary result--------------------------" << std::endl;
    std::cout << std::endl << "forward kinematic optimization error: " << optimizer.error() << std::endl;

    return result_LM;

}

// a function to hold parameters and invoke optimizer and back the optimized data
void fkSolver(double *lc_cat, 
              Vector3d pos_init,  
              Matrix3d rot_init, Vector2d fc0, 
              CableRobotParams robot_params,
              FKDataOut<double> *result)
{  
    RobotParameters<double> params;
    //initilize the pulley locations
    params.pulleys.push_back(robot_params.p1_);
    params.pulleys.push_back(robot_params.p2_);
    params.pulleys.push_back(robot_params.p3_);
    params.pulleys.push_back(robot_params.p4_);
    params.r_to_cog = robot_params.r_to_cog_;

    //initilize cable attachement points
    params.ef_points.push_back(robot_params.b1_);
    params.ef_points.push_back(robot_params.b2_);
    params.ef_points.push_back(robot_params.b3_);
    params.ef_points.push_back(robot_params.b4_);
    params.g_c= robot_params.g_c_;
    params.f_g = robot_params.f_g_;

    // Initialize the quaternion array
    double rotation_matrix_double[9];
    for (int i = 0; i < 9; ++i) {
        rotation_matrix_double[i] = rot_init.data()[i];
    }
    double quaternion[4];    
    ceres::RotationMatrixToQuaternion<double>(rotation_matrix_double, quaternion);
    double rot_init_x = quaternion[1];
    double rot_init_y = quaternion[2];
    double rot_init_z = quaternion[3];
    double rot_init_w = quaternion[0];

    double lc0 = lc_cat[0];   
    double lc1 = lc_cat[1];
    double lc2 = lc_cat[2];
    double lc3 = lc_cat[3];


    // initial values for variable 
    double init_h1 = fc0[0];
    double init_v1 = fc0[1];
    double init_rx = 0.0001;
    double init_ry = 0.0001;
    double init_rz = 0.0001;
    double init_tx = pos_init[0];
    double init_ty = pos_init[1];
    double init_tz = pos_init[2];

    // run optimization!
    Values optimization_result = forward_kinematic_factor_graph_optimizer(lc0, lc1, lc2, lc3,
                                                        rot_init_x, rot_init_y, rot_init_z, rot_init_w, 
                                                        init_h1, init_v1, init_rx, init_ry, init_rz, init_tx, init_ty, init_tz);

    //harvest the results
    double fh = optimization_result.at<double>(Symbol('h', 1)); //optimized horizontal force for the first cable
    double fv = optimization_result.at<double>(Symbol('v', 1)); //optimized vertical force for the first cable
    // Extract the optimized orientation matrix of the moving platform
    manif::SO3Tangentd xi;
    xi << optimization_result.at<double>(Symbol('x', 1)), optimization_result.at<double>(Symbol('x', 2)), optimization_result.at<double>(Symbol('x', 3));
    manif::SO3<double> rot_optimized = manif::exp(xi);
    Matrix3d rot_result = rot_init * rot_optimized.rotation();
    result->rot_platform = rot_result;

    result->p_platform << optimization_result.at<double>(Symbol('t', 1)), optimization_result.at<double>(Symbol('t', 2)), optimization_result.at<double>(Symbol('t', 3));

    // Use the utils functions once again to compute the force in other cables and the catenary variables
    GeometricVariables<double> geom_vars;
    CatenaryVariables<double> cat_vars;
    RobotState<double> state;

    state.rot_platform = rot_result;
    state.p_platform = result->p_platform;

    getGeometricVariables<double>(state,params,&geom_vars);
    getCableForces<double>(fh, fv, &state, params,geom_vars);
    getCatenaryVariables<double>(state,params, geom_vars,&cat_vars);

    result->c1 = cat_vars.c1;
    result->c2 = cat_vars.c2;
    result->lc_cat = cat_vars.lc_cat;
    result->cable_forces = state.cable_forces;
    result->b_in_w = geom_vars.b_in_w;
}

// a function to modify parameters, invoke optimizer and harvest the result
std::vector<MatrixXd> FK_Factor_Graph_Optimization(CableRobotParams robot_params, VectorXd lc_cat, Vector2d fc_1, Vector3d pos_init, Matrix3d rot_init)
{
    std::vector<MatrixXd> results_list;
    FKDataOut<double> fk_results;
    fkSolver(lc_cat.data(), pos_init, rot_init, fc_1, robot_params, &fk_results);

    //Extract The results and return them as a list of matrices to python
    
    results_list.push_back(fk_results.rot_platform);
    results_list.push_back(fk_results.p_platform);

    int N = 4;

    //Cable Forces
    MatrixXd cable_forces(2,N);
    for(int i=0; i<N; i++)
        cable_forces.col(i) =  fk_results.cable_forces[i];
    results_list.push_back(cable_forces);

    //Catenary parameters
    //C1
    MatrixXd c1_(1, N);
    for(int i=0; i<N; i++)
        c1_(0,i) = fk_results.c1[i];
    results_list.push_back(c1_);

    //C2
    MatrixXd c2_(1, N);
    for(int i=0; i<N; i++)
        c2_(0,i) = fk_results.c2[i];
    results_list.push_back(c2_);

    //body coordinates represented in the world frame
    MatrixXd b_in_w(3,N);
    for(int i=0; i<N; i++)
        b_in_w.col(i) =  fk_results.b_in_w[i];
    results_list.push_back(b_in_w);

    return results_list;
}
