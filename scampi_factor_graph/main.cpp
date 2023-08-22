#include </home/mohammad/scampi_factor_graph_optimization/scampi_factor_graph/scampi_fg.cpp>
// g++ -std=c++17 main.cpp -lgtsam -lboost_system -lboost_timer -ltbb -ltbbmalloc -lmetis -lboost_serialization && ./a.out
#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

int main(int argc, char *argv[])
{   
    // robot characteristic
    CableRobotParams robot_params(0.1034955, 43.164);

    Eigen::Vector3d Pulley_a(-1.9874742 , -8.31965637,  8.47184658);
    Eigen::Vector3d Pulley_b(2.52022147, -8.38887501,  8.46931362);
    Eigen::Vector3d Pulley_c(2.71799795, 4.77520639, 8.36416322);
    Eigen::Vector3d Pulley_d(-1.79662371,  4.83333111,  8.37001991);
    robot_params.setPulleyPoses(Pulley_a, Pulley_b, Pulley_c, Pulley_d);

    Eigen::Vector3d Ee_a(-0.21 , -0.21 , -0.011);  
    Eigen::Vector3d Ee_b(0.21  , -0.21 , -0.011);
    Eigen::Vector3d Ee_c(0.21  ,  0.21 , -0.011);
    Eigen::Vector3d Ee_d(-0.21 ,  0.21 , -0.011);
    robot_params.setEEAnchors(Ee_a, Ee_b, Ee_c, Ee_d);

    Eigen::Vector3d r_to_cog(0, 0, -0.12);
    robot_params.setCog(r_to_cog);

    Eigen::Vector3d p_platform(3.09173747e-01, -1.83715841e+00,  2.18367984e+00);
    Eigen::Matrix3d rot_init;
    rot_init << 0.99268615,  0.11337417, -0.04147891,
               -0.11309773,  0.99354347,  0.00895918,
                0.04222684, -0.00420248,  0.99909921; 
    
    // start optimization
    std::vector<MatrixXd> results = IK_Factor_Graph_Optimization(robot_params, rot_init, p_platform);

    // the result of optimization
    std::cout << std::endl << "-------------------caternary result--------------------------" << std::endl;
    std::cout << std::endl << "rot_platform: " << std::endl << results[0] << std::endl;
    std::cout << std::endl << "l_cat: " << std::endl << results[1] << std::endl;
    std::cout << std::endl << "cable_forces: " << std::endl << results[2] << std::endl;
    std::cout << std::endl << "c1: " << std::endl << results[3] << std::endl;
    std::cout << std::endl << "c2: " << std::endl << results[4] << std::endl;
    std::cout << std::endl << "b_in_w: " << std::endl << results[5] << std::endl;

    return 0;
}