#pragma once

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <boost/optional.hpp>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/inference/Symbol.h>
#include "scampi_function_header_include.h"

using namespace gtsam;
using namespace std;

namespace gtsam
{
    class IK_factor_graoh_cost1 : public NoiseModelFactor5<double, double, double, double, double>
    {

    private:
        double p_init0;
        double p_init1; 
        double p_init2; 
        double rot_init_x; 
        double rot_init_y; 
        double rot_init_z; 
        double rot_init_w; 
        double epsilon = 0.0;
        int largest_cable = 0;

    public:
        // Constructor
        IK_factor_graoh_cost1(Key key1, Key key2, Key key3, Key key4, Key key5, double p_init0_, double p_init1_, double p_init2_, double rot_init_x_, double rot_init_y_, double rot_init_z_, double rot_init_w_, const int largest_cable_, const SharedNoiseModel &model) 
        : NoiseModelFactor5<double, double, double, double, double>(model, key1, key2, key3, key4, key5), p_init0(p_init0_), p_init1(p_init1_), p_init2(p_init2_), rot_init_x(rot_init_x_), rot_init_y(rot_init_y_), rot_init_z(rot_init_z_), rot_init_w(rot_init_w_), largest_cable(largest_cable_) {}

        // Evaluate the error
        Vector evaluateError(const double &fh1, const double &fv1, const double &rx, const double &ry, const double &rz,
                             OptionalMatrixType H1,
                             OptionalMatrixType H2,
                             OptionalMatrixType H3,
                             OptionalMatrixType H4,
                             OptionalMatrixType H5) const override
        {   
            Eigen::Matrix<double, 4, 1> Ikresidual_func;

            switch (largest_cable)
            {   
                case 0:
                    Ikresidual_func = sym::IkResidualFuncCost1Nl0(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl0 = sym::IkResidualFuncCost1WrtFh1Nl0(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl0).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl0 = sym::IkResidualFuncCost1WrtFv1Nl0(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl0).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl0 = sym::IkResidualFuncCost1WrtRxNl0(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl0).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl0 = sym::IkResidualFuncCost1WrtRyNl0(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl0).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl0 = sym::IkResidualFuncCost1WrtRzNl0(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl0).finished();
                    }
                    break;


                case 1:
                    Ikresidual_func = sym::IkResidualFuncCost1Nl1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl1 = sym::IkResidualFuncCost1WrtFh1Nl1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl1).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl1 = sym::IkResidualFuncCost1WrtFv1Nl1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl1).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl1 = sym::IkResidualFuncCost1WrtRxNl1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl1).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl1 = sym::IkResidualFuncCost1WrtRyNl1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl1).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl1 = sym::IkResidualFuncCost1WrtRzNl1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl1).finished();
                    }
                    break;


                case 2:
                    Ikresidual_func = sym::IkResidualFuncCost1Nl2(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl2 = sym::IkResidualFuncCost1WrtFh1Nl2(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl2).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl2 = sym::IkResidualFuncCost1WrtFv1Nl2(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl2).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl2 = sym::IkResidualFuncCost1WrtRxNl2(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl2).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl2 = sym::IkResidualFuncCost1WrtRyNl2(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl2).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl2 = sym::IkResidualFuncCost1WrtRzNl2(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl2).finished();
                    }
                    break;



                case 3:
                    Ikresidual_func = sym::IkResidualFuncCost1Nl1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl1 = sym::IkResidualFuncCost1WrtFh1Nl1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl1).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl1 = sym::IkResidualFuncCost1WrtFv1Nl1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl1).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl1 = sym::IkResidualFuncCost1WrtRxNl1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl1).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl1 = sym::IkResidualFuncCost1WrtRyNl1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl1).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl1 = sym::IkResidualFuncCost1WrtRzNl1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl1).finished();
                    }
                    break;



                case 4:
                    Ikresidual_func = sym::IkResidualFuncCost1Nl4(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl4 = sym::IkResidualFuncCost1WrtFh1Nl4(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl4).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl4 = sym::IkResidualFuncCost1WrtFv1Nl4(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl4).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl4 = sym::IkResidualFuncCost1WrtRxNl4(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl4).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl4 = sym::IkResidualFuncCost1WrtRyNl4(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl4).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl4 = sym::IkResidualFuncCost1WrtRzNl4(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl4).finished();
                    }
                    break;



                case 5:
                    Ikresidual_func = sym::IkResidualFuncCost1Nl6(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl6 = sym::IkResidualFuncCost1WrtFh1Nl6(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl6).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl6 = sym::IkResidualFuncCost1WrtFv1Nl6(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl6).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl6 = sym::IkResidualFuncCost1WrtRxNl6(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl6).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl6 = sym::IkResidualFuncCost1WrtRyNl6(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl6).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl6 = sym::IkResidualFuncCost1WrtRzNl6(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl6).finished();
                    }
                    break;



                case 7:
                    Ikresidual_func = sym::IkResidualFuncCost1Nl1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl7 = sym::IkResidualFuncCost1WrtFh1Nl7(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl7).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl7 = sym::IkResidualFuncCost1WrtFv1Nl7(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl7).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl7 = sym::IkResidualFuncCost1WrtRxNl7(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl7).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl7 = sym::IkResidualFuncCost1WrtRyNl7(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl7).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl7 = sym::IkResidualFuncCost1WrtRzNl7(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl7).finished();
                    }
                    break;


                case 8:
                    Ikresidual_func = sym::IkResidualFuncCost1Nl8(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl8 = sym::IkResidualFuncCost1WrtFh1Nl8(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl8).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl8 = sym::IkResidualFuncCost1WrtFv1Nl8(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl8).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl8 = sym::IkResidualFuncCost1WrtRxNl8(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl8).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl8 = sym::IkResidualFuncCost1WrtRyNl8(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl8).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl8 = sym::IkResidualFuncCost1WrtRzNl8(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl8).finished();
                    }
                    break;



                case 9:
                    Ikresidual_func = sym::IkResidualFuncCost1Nl9(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl9 = sym::IkResidualFuncCost1WrtFh1Nl9(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl9).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl9 = sym::IkResidualFuncCost1WrtFv1Nl9(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl9).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl9 = sym::IkResidualFuncCost1WrtRxNl9(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl9).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl9 = sym::IkResidualFuncCost1WrtRyNl9(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl9).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl9 = sym::IkResidualFuncCost1WrtRzNl9(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl9).finished();
                    }
                    break;



                case 10:
                    Ikresidual_func = sym::IkResidualFuncCost1Nl10(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl10 = sym::IkResidualFuncCost1WrtFh1Nl10(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl10).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl10 = sym::IkResidualFuncCost1WrtFv1Nl10(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl10).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl10 = sym::IkResidualFuncCost1WrtRxNl10(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl10).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl10 = sym::IkResidualFuncCost1WrtRyNl10(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl10).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl10 = sym::IkResidualFuncCost1WrtRzNl10(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl10).finished();
                    }
                    break;



                case 11:
                    Ikresidual_func = sym::IkResidualFuncCost1Nl11(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl11 = sym::IkResidualFuncCost1WrtFh1Nl11(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl11).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl11 = sym::IkResidualFuncCost1WrtFv1Nl11(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl11).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl11 = sym::IkResidualFuncCost1WrtRxNl11(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl11).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl11 = sym::IkResidualFuncCost1WrtRyNl11(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl11).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl11 = sym::IkResidualFuncCost1WrtRzNl11(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl11).finished();
                    }
                    break;



                case 12:
                    Ikresidual_func = sym::IkResidualFuncCost1Nl12(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl12 = sym::IkResidualFuncCost1WrtFh1Nl12(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl12).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl12 = sym::IkResidualFuncCost1WrtFv1Nl12(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl12).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl12 = sym::IkResidualFuncCost1WrtRxNl12(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl12).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl12 = sym::IkResidualFuncCost1WrtRyNl12(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl12).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl12 = sym::IkResidualFuncCost1WrtRzNl12(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl12).finished();
                    }
                    break;



                case 13:
                    Ikresidual_func = sym::IkResidualFuncCost1Nl13(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl13 = sym::IkResidualFuncCost1WrtFh1Nl13(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl13).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl13 = sym::IkResidualFuncCost1WrtFv1Nl13(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl13).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl13 = sym::IkResidualFuncCost1WrtRxNl13(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl13).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl13 = sym::IkResidualFuncCost1WrtRyNl13(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl13).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl13 = sym::IkResidualFuncCost1WrtRzNl13(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl13).finished();
                    }
                    break;



                case 14:
                    Ikresidual_func = sym::IkResidualFuncCost1Nl14(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl14 = sym::IkResidualFuncCost1WrtFh1Nl14(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl14).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl14 = sym::IkResidualFuncCost1WrtFv1Nl14(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl14).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl14 = sym::IkResidualFuncCost1WrtRxNl14(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl14).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl14 = sym::IkResidualFuncCost1WrtRyNl14(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl14).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl14 = sym::IkResidualFuncCost1WrtRzNl14(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl14).finished();
                    }
                    break;



                case 15:
                    Ikresidual_func = sym::IkResidualFuncCost1Nl15(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl15 = sym::IkResidualFuncCost1WrtFh1Nl15(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl15).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl15 = sym::IkResidualFuncCost1WrtFv1Nl15(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl15).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl15 = sym::IkResidualFuncCost1WrtRxNl15(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl15).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl15 = sym::IkResidualFuncCost1WrtRyNl15(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl15).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl15 = sym::IkResidualFuncCost1WrtRzNl15(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl15).finished();
                    }
                    break;



                case 16:
                    Ikresidual_func = sym::IkResidualFuncCost1Nl16(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl16 = sym::IkResidualFuncCost1WrtFh1Nl16(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl16).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl16 = sym::IkResidualFuncCost1WrtFv1Nl16(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl16).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl16 = sym::IkResidualFuncCost1WrtRxNl16(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl16).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl16 = sym::IkResidualFuncCost1WrtRyNl16(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl16).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl16 = sym::IkResidualFuncCost1WrtRzNl16(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl16).finished();
                    }
                    break;



                case 17:
                    Ikresidual_func = sym::IkResidualFuncCost1Nl17(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl17 = sym::IkResidualFuncCost1WrtFh1Nl17(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl17).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl17 = sym::IkResidualFuncCost1WrtFv1Nl17(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl17).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl17 = sym::IkResidualFuncCost1WrtRxNl17(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl17).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl17 = sym::IkResidualFuncCost1WrtRyNl17(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl17).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl17 = sym::IkResidualFuncCost1WrtRzNl17(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl17).finished();
                    }
                    break;


                case 18:
                    Ikresidual_func = sym::IkResidualFuncCost1Nl18(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl18 = sym::IkResidualFuncCost1WrtFh1Nl18(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl18).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl18 = sym::IkResidualFuncCost1WrtFv1Nl18(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl18).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl18 = sym::IkResidualFuncCost1WrtRxNl18(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl18).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl18 = sym::IkResidualFuncCost1WrtRyNl18(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl18).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl18 = sym::IkResidualFuncCost1WrtRzNl18(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl18).finished();
                    }
                    break;



                case 19:
                    Ikresidual_func = sym::IkResidualFuncCost1Nl19(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl19 = sym::IkResidualFuncCost1WrtFh1Nl19(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl19).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl19 = sym::IkResidualFuncCost1WrtFv1Nl19(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl19).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl19 = sym::IkResidualFuncCost1WrtRxNl19(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl19).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl19 = sym::IkResidualFuncCost1WrtRyNl19(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl19).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl19 = sym::IkResidualFuncCost1WrtRzNl19(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl19).finished();
                    }
                    break;



                case 20:
                    Ikresidual_func = sym::IkResidualFuncCost1Nl20(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl20 = sym::IkResidualFuncCost1WrtFh1Nl20(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl20).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl20 = sym::IkResidualFuncCost1WrtFv1Nl20(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl20).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl20 = sym::IkResidualFuncCost1WrtRxNl20(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl20).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl20 = sym::IkResidualFuncCost1WrtRyNl20(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl20).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl20 = sym::IkResidualFuncCost1WrtRzNl20(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl20).finished();
                    }
                    break;



                case 21:
                    Ikresidual_func = sym::IkResidualFuncCost1Nl21(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl21 = sym::IkResidualFuncCost1WrtFh1Nl21(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl21).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl21 = sym::IkResidualFuncCost1WrtFv1Nl21(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl21).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl21 = sym::IkResidualFuncCost1WrtRxNl21(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl21).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl21 = sym::IkResidualFuncCost1WrtRyNl21(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl21).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl21 = sym::IkResidualFuncCost1WrtRzNl21(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl21).finished();
                    }
                    break;



                case 22:
                    Ikresidual_func = sym::IkResidualFuncCost1Nl22(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl22 = sym::IkResidualFuncCost1WrtFh1Nl22(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl22).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl22 = sym::IkResidualFuncCost1WrtFv1Nl22(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl22).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl22 = sym::IkResidualFuncCost1WrtRxNl22(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl22).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl22 = sym::IkResidualFuncCost1WrtRyNl22(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl22).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl22 = sym::IkResidualFuncCost1WrtRzNl22(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl22).finished();
                    }
                    break;



                case 23:
                    Ikresidual_func = sym::IkResidualFuncCost1Nl23(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl23 = sym::IkResidualFuncCost1WrtFh1Nl23(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl23).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl23 = sym::IkResidualFuncCost1WrtFv1Nl23(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl23).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl23 = sym::IkResidualFuncCost1WrtRxNl23(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl23).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl23 = sym::IkResidualFuncCost1WrtRyNl23(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl23).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl23 = sym::IkResidualFuncCost1WrtRzNl23(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl23).finished();
                    }
                    break;
            }
            return (Vector(4) << Ikresidual_func).finished();
        }
    };

    class IK_factor_graoh_cost2 : public NoiseModelFactor5<double, double, double, double, double>
    {

    private:
        double p_init0;
        double p_init1; 
        double p_init2; 
        double rot_init_x; 
        double rot_init_y; 
        double rot_init_z; 
        double rot_init_w; 
        double epsilon = 0.0;
        int largest_cable = 0;

    public:
        // Constructor
        IK_factor_graoh_cost2(Key key1, Key key2, Key key3, Key key4, Key key5, double p_init0_, double p_init1_, double p_init2_, double rot_init_x_, double rot_init_y_, double rot_init_z_, double rot_init_w_, const int largest_cable_, const SharedNoiseModel &model) 
        : NoiseModelFactor5<double, double, double, double, double>(model, key1, key2, key3, key4, key5), p_init0(p_init0_), p_init1(p_init1_), p_init2(p_init2_), rot_init_x(rot_init_x_), rot_init_y(rot_init_y_), rot_init_z(rot_init_z_), rot_init_w(rot_init_w_), largest_cable(largest_cable_) {}

        // Evaluate the error
        Vector evaluateError(const double &fh1, const double &fv1, const double &rx, const double &ry, const double &rz,
                             OptionalMatrixType H1,
                             OptionalMatrixType H2,
                             OptionalMatrixType H3,
                             OptionalMatrixType H4,
                             OptionalMatrixType H5) const override
        {   
            Eigen::Matrix<double, 4, 1> Ikresidual_func;

            switch (largest_cable)
            {   
                case 0:
                    Ikresidual_func = sym::IkResidualFuncCost2Nl0(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl0 = sym::IkResidualFuncCost2WrtFh1Nl0(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl0).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl0 = sym::IkResidualFuncCost2WrtFv1Nl0(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl0).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl0 = sym::IkResidualFuncCost2WrtRxNl0(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl0).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl0 = sym::IkResidualFuncCost2WrtRyNl0(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl0).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl0 = sym::IkResidualFuncCost2WrtRzNl0(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl0).finished();
                    }
                    break;


                case 1:
                    Ikresidual_func = sym::IkResidualFuncCost2Nl1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl1 = sym::IkResidualFuncCost2WrtFh1Nl1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl1).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl1 = sym::IkResidualFuncCost2WrtFv1Nl1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl1).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl1 = sym::IkResidualFuncCost2WrtRxNl1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl1).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl1 = sym::IkResidualFuncCost2WrtRyNl1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl1).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl1 = sym::IkResidualFuncCost2WrtRzNl1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl1).finished();
                    }
                    break;


                case 2:
                    Ikresidual_func = sym::IkResidualFuncCost2Nl2(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl2 = sym::IkResidualFuncCost2WrtFh1Nl2(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl2).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl2 = sym::IkResidualFuncCost2WrtFv1Nl2(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl2).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl2 = sym::IkResidualFuncCost2WrtRxNl2(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl2).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl2 = sym::IkResidualFuncCost2WrtRyNl2(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl2).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl2 = sym::IkResidualFuncCost2WrtRzNl2(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl2).finished();
                    }
                    break;



                case 3:
                    Ikresidual_func = sym::IkResidualFuncCost2Nl1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl1 = sym::IkResidualFuncCost2WrtFh1Nl1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl1).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl1 = sym::IkResidualFuncCost2WrtFv1Nl1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl1).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl1 = sym::IkResidualFuncCost2WrtRxNl1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl1).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl1 = sym::IkResidualFuncCost2WrtRyNl1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl1).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl1 = sym::IkResidualFuncCost2WrtRzNl1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl1).finished();
                    }
                    break;



                case 4:
                    Ikresidual_func = sym::IkResidualFuncCost2Nl4(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl4 = sym::IkResidualFuncCost2WrtFh1Nl4(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl4).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl4 = sym::IkResidualFuncCost2WrtFv1Nl4(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl4).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl4 = sym::IkResidualFuncCost2WrtRxNl4(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl4).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl4 = sym::IkResidualFuncCost2WrtRyNl4(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl4).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl4 = sym::IkResidualFuncCost2WrtRzNl4(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl4).finished();
                    }
                    break;



                case 5:
                    Ikresidual_func = sym::IkResidualFuncCost2Nl6(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl6 = sym::IkResidualFuncCost2WrtFh1Nl6(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl6).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl6 = sym::IkResidualFuncCost2WrtFv1Nl6(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl6).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl6 = sym::IkResidualFuncCost2WrtRxNl6(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl6).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl6 = sym::IkResidualFuncCost2WrtRyNl6(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl6).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl6 = sym::IkResidualFuncCost2WrtRzNl6(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl6).finished();
                    }
                    break;



                case 7:
                    Ikresidual_func = sym::IkResidualFuncCost2Nl1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl7 = sym::IkResidualFuncCost2WrtFh1Nl7(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl7).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl7 = sym::IkResidualFuncCost2WrtFv1Nl7(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl7).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl7 = sym::IkResidualFuncCost2WrtRxNl7(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl7).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl7 = sym::IkResidualFuncCost2WrtRyNl7(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl7).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl7 = sym::IkResidualFuncCost2WrtRzNl7(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl7).finished();
                    }
                    break;


                case 8:
                    Ikresidual_func = sym::IkResidualFuncCost2Nl8(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl8 = sym::IkResidualFuncCost2WrtFh1Nl8(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl8).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl8 = sym::IkResidualFuncCost2WrtFv1Nl8(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl8).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl8 = sym::IkResidualFuncCost2WrtRxNl8(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl8).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl8 = sym::IkResidualFuncCost2WrtRyNl8(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl8).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl8 = sym::IkResidualFuncCost2WrtRzNl8(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl8).finished();
                    }
                    break;



                case 9:
                    Ikresidual_func = sym::IkResidualFuncCost2Nl9(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl9 = sym::IkResidualFuncCost2WrtFh1Nl9(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl9).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl9 = sym::IkResidualFuncCost2WrtFv1Nl9(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl9).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl9 = sym::IkResidualFuncCost2WrtRxNl9(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl9).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl9 = sym::IkResidualFuncCost2WrtRyNl9(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl9).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl9 = sym::IkResidualFuncCost2WrtRzNl9(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl9).finished();
                    }
                    break;



                case 10:
                    Ikresidual_func = sym::IkResidualFuncCost2Nl10(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl10 = sym::IkResidualFuncCost2WrtFh1Nl10(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl10).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl10 = sym::IkResidualFuncCost2WrtFv1Nl10(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl10).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl10 = sym::IkResidualFuncCost2WrtRxNl10(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl10).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl10 = sym::IkResidualFuncCost2WrtRyNl10(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl10).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl10 = sym::IkResidualFuncCost2WrtRzNl10(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl10).finished();
                    }
                    break;



                case 11:
                    Ikresidual_func = sym::IkResidualFuncCost2Nl11(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl11 = sym::IkResidualFuncCost2WrtFh1Nl11(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl11).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl11 = sym::IkResidualFuncCost2WrtFv1Nl11(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl11).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl11 = sym::IkResidualFuncCost2WrtRxNl11(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl11).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl11 = sym::IkResidualFuncCost2WrtRyNl11(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl11).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl11 = sym::IkResidualFuncCost2WrtRzNl11(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl11).finished();
                    }
                    break;



                case 12:
                    Ikresidual_func = sym::IkResidualFuncCost2Nl12(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl12 = sym::IkResidualFuncCost2WrtFh1Nl12(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl12).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl12 = sym::IkResidualFuncCost2WrtFv1Nl12(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl12).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl12 = sym::IkResidualFuncCost2WrtRxNl12(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl12).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl12 = sym::IkResidualFuncCost2WrtRyNl12(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl12).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl12 = sym::IkResidualFuncCost2WrtRzNl12(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl12).finished();
                    }
                    break;



                case 13:
                    Ikresidual_func = sym::IkResidualFuncCost2Nl13(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl13 = sym::IkResidualFuncCost2WrtFh1Nl13(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl13).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl13 = sym::IkResidualFuncCost2WrtFv1Nl13(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl13).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl13 = sym::IkResidualFuncCost2WrtRxNl13(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl13).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl13 = sym::IkResidualFuncCost2WrtRyNl13(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl13).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl13 = sym::IkResidualFuncCost2WrtRzNl13(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl13).finished();
                    }
                    break;



                case 14:
                    Ikresidual_func = sym::IkResidualFuncCost2Nl14(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl14 = sym::IkResidualFuncCost2WrtFh1Nl14(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl14).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl14 = sym::IkResidualFuncCost2WrtFv1Nl14(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl14).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl14 = sym::IkResidualFuncCost2WrtRxNl14(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl14).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl14 = sym::IkResidualFuncCost2WrtRyNl14(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl14).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl14 = sym::IkResidualFuncCost2WrtRzNl14(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl14).finished();
                    }
                    break;



                case 15:
                    Ikresidual_func = sym::IkResidualFuncCost2Nl15(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl15 = sym::IkResidualFuncCost2WrtFh1Nl15(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl15).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl15 = sym::IkResidualFuncCost2WrtFv1Nl15(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl15).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl15 = sym::IkResidualFuncCost2WrtRxNl15(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl15).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl15 = sym::IkResidualFuncCost2WrtRyNl15(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl15).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl15 = sym::IkResidualFuncCost2WrtRzNl15(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl15).finished();
                    }
                    break;



                case 16:
                    Ikresidual_func = sym::IkResidualFuncCost2Nl16(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl16 = sym::IkResidualFuncCost2WrtFh1Nl16(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl16).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl16 = sym::IkResidualFuncCost2WrtFv1Nl16(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl16).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl16 = sym::IkResidualFuncCost2WrtRxNl16(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl16).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl16 = sym::IkResidualFuncCost2WrtRyNl16(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl16).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl16 = sym::IkResidualFuncCost2WrtRzNl16(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl16).finished();
                    }
                    break;



                case 17:
                    Ikresidual_func = sym::IkResidualFuncCost2Nl17(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl17 = sym::IkResidualFuncCost2WrtFh1Nl17(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl17).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl17 = sym::IkResidualFuncCost2WrtFv1Nl17(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl17).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl17 = sym::IkResidualFuncCost2WrtRxNl17(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl17).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl17 = sym::IkResidualFuncCost2WrtRyNl17(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl17).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl17 = sym::IkResidualFuncCost2WrtRzNl17(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl17).finished();
                    }
                    break;


                case 18:
                    Ikresidual_func = sym::IkResidualFuncCost2Nl18(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl18 = sym::IkResidualFuncCost2WrtFh1Nl18(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl18).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl18 = sym::IkResidualFuncCost2WrtFv1Nl18(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl18).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl18 = sym::IkResidualFuncCost2WrtRxNl18(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl18).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl18 = sym::IkResidualFuncCost2WrtRyNl18(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl18).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl18 = sym::IkResidualFuncCost2WrtRzNl18(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl18).finished();
                    }
                    break;



                case 19:
                    Ikresidual_func = sym::IkResidualFuncCost2Nl19(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl19 = sym::IkResidualFuncCost2WrtFh1Nl19(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl19).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl19 = sym::IkResidualFuncCost2WrtFv1Nl19(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl19).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl19 = sym::IkResidualFuncCost2WrtRxNl19(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl19).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl19 = sym::IkResidualFuncCost2WrtRyNl19(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl19).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl19 = sym::IkResidualFuncCost2WrtRzNl19(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl19).finished();
                    }
                    break;



                case 20:
                    Ikresidual_func = sym::IkResidualFuncCost2Nl20(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl20 = sym::IkResidualFuncCost2WrtFh1Nl20(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl20).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl20 = sym::IkResidualFuncCost2WrtFv1Nl20(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl20).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl20 = sym::IkResidualFuncCost2WrtRxNl20(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl20).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl20 = sym::IkResidualFuncCost2WrtRyNl20(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl20).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl20 = sym::IkResidualFuncCost2WrtRzNl20(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl20).finished();
                    }
                    break;



                case 21:
                    Ikresidual_func = sym::IkResidualFuncCost2Nl21(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl21 = sym::IkResidualFuncCost2WrtFh1Nl21(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl21).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl21 = sym::IkResidualFuncCost2WrtFv1Nl21(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl21).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl21 = sym::IkResidualFuncCost2WrtRxNl21(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl21).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl21 = sym::IkResidualFuncCost2WrtRyNl21(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl21).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl21 = sym::IkResidualFuncCost2WrtRzNl21(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl21).finished();
                    }
                    break;



                case 22:
                    Ikresidual_func = sym::IkResidualFuncCost2Nl22(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl22 = sym::IkResidualFuncCost2WrtFh1Nl22(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl22).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl22 = sym::IkResidualFuncCost2WrtFv1Nl22(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl22).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl22 = sym::IkResidualFuncCost2WrtRxNl22(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl22).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl22 = sym::IkResidualFuncCost2WrtRyNl22(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl22).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl22 = sym::IkResidualFuncCost2WrtRzNl22(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl22).finished();
                    }
                    break;



                case 23:
                    Ikresidual_func = sym::IkResidualFuncCost2Nl23(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl23 = sym::IkResidualFuncCost2WrtFh1Nl23(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl23).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl23 = sym::IkResidualFuncCost2WrtFv1Nl23(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl23).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl23 = sym::IkResidualFuncCost2WrtRxNl23(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl23).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl23 = sym::IkResidualFuncCost2WrtRyNl23(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl23).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl23 = sym::IkResidualFuncCost2WrtRzNl23(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl23).finished();
                    }
                    break;
            }
            return (Vector(4) << Ikresidual_func).finished(); 
        }
    };


    class IK_factor_graoh_cost3 : public NoiseModelFactor5<double, double, double, double, double>
    {

    private:
        double p_init0;
        double p_init1; 
        double p_init2; 
        double rot_init_x; 
        double rot_init_y; 
        double rot_init_z; 
        double rot_init_w; 
        double epsilon = 0.0;
        int largest_cable;

    public:
        // Constructor
        IK_factor_graoh_cost3(Key key1, Key key2, Key key3, Key key4, Key key5, double p_init0_, double p_init1_, double p_init2_, double rot_init_x_, double rot_init_y_, double rot_init_z_, double rot_init_w_, const int largest_cable_, const SharedNoiseModel &model) 
        : NoiseModelFactor5<double, double, double, double, double>(model, key1, key2, key3, key4, key5), p_init0(p_init0_), p_init1(p_init1_), p_init2(p_init2_), rot_init_x(rot_init_x_), rot_init_y(rot_init_y_), rot_init_z(rot_init_z_), rot_init_w(rot_init_w_), largest_cable(largest_cable_) {}

        // Evaluate the error
        Vector evaluateError(const double &fh1, const double &fv1, const double &rx, const double &ry, const double &rz,
                             OptionalMatrixType H1,
                             OptionalMatrixType H2,
                             OptionalMatrixType H3,
                             OptionalMatrixType H4,
                             OptionalMatrixType H5) const override
        {   
            Eigen::Matrix<double, 4, 1> Ikresidual_func; 

            switch (largest_cable)
            {   
                case 0:
                    Ikresidual_func = sym::IkResidualFuncCost3Nl0(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl0 = sym::IkResidualFuncCost3WrtFh1Nl0(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl0).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl0 = sym::IkResidualFuncCost3WrtFv1Nl0(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl0).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl0 = sym::IkResidualFuncCost3WrtRxNl0(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl0).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl0 = sym::IkResidualFuncCost3WrtRyNl0(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl0).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl0 = sym::IkResidualFuncCost3WrtRzNl0(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl0).finished();
                    }
                    break;


                case 1:
                    Ikresidual_func = sym::IkResidualFuncCost3Nl1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl1 = sym::IkResidualFuncCost3WrtFh1Nl1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl1).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl1 = sym::IkResidualFuncCost3WrtFv1Nl1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl1).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl1 = sym::IkResidualFuncCost3WrtRxNl1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl1).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl1 = sym::IkResidualFuncCost3WrtRyNl1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl1).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl1 = sym::IkResidualFuncCost3WrtRzNl1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl1).finished();
                    }
                    break;


                case 2:
                    Ikresidual_func = sym::IkResidualFuncCost3Nl2(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl2 = sym::IkResidualFuncCost3WrtFh1Nl2(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl2).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl2 = sym::IkResidualFuncCost3WrtFv1Nl2(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl2).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl2 = sym::IkResidualFuncCost3WrtRxNl2(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl2).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl2 = sym::IkResidualFuncCost3WrtRyNl2(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl2).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl2 = sym::IkResidualFuncCost3WrtRzNl2(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl2).finished();
                    }
                    break;



                case 3:
                    Ikresidual_func = sym::IkResidualFuncCost3Nl1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl1 = sym::IkResidualFuncCost3WrtFh1Nl1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl1).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl1 = sym::IkResidualFuncCost3WrtFv1Nl1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl1).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl1 = sym::IkResidualFuncCost3WrtRxNl1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl1).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl1 = sym::IkResidualFuncCost3WrtRyNl1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl1).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl1 = sym::IkResidualFuncCost3WrtRzNl1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl1).finished();
                    }
                    break;



                case 4:
                    Ikresidual_func = sym::IkResidualFuncCost3Nl4(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl4 = sym::IkResidualFuncCost3WrtFh1Nl4(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl4).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl4 = sym::IkResidualFuncCost3WrtFv1Nl4(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl4).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl4 = sym::IkResidualFuncCost3WrtRxNl4(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl4).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl4 = sym::IkResidualFuncCost3WrtRyNl4(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl4).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl4 = sym::IkResidualFuncCost3WrtRzNl4(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl4).finished();
                    }
                    break;



                case 5:
                    Ikresidual_func = sym::IkResidualFuncCost3Nl6(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl6 = sym::IkResidualFuncCost3WrtFh1Nl6(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl6).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl6 = sym::IkResidualFuncCost3WrtFv1Nl6(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl6).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl6 = sym::IkResidualFuncCost3WrtRxNl6(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl6).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl6 = sym::IkResidualFuncCost3WrtRyNl6(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl6).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl6 = sym::IkResidualFuncCost3WrtRzNl6(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl6).finished();
                    }
                    break;



                case 7:
                    Ikresidual_func = sym::IkResidualFuncCost3Nl1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl7 = sym::IkResidualFuncCost3WrtFh1Nl7(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl7).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl7 = sym::IkResidualFuncCost3WrtFv1Nl7(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl7).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl7 = sym::IkResidualFuncCost3WrtRxNl7(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl7).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl7 = sym::IkResidualFuncCost3WrtRyNl7(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl7).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl7 = sym::IkResidualFuncCost3WrtRzNl7(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl7).finished();
                    }
                    break;


                case 8:
                    Ikresidual_func = sym::IkResidualFuncCost3Nl8(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl8 = sym::IkResidualFuncCost3WrtFh1Nl8(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl8).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl8 = sym::IkResidualFuncCost3WrtFv1Nl8(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl8).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl8 = sym::IkResidualFuncCost3WrtRxNl8(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl8).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl8 = sym::IkResidualFuncCost3WrtRyNl8(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl8).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl8 = sym::IkResidualFuncCost3WrtRzNl8(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl8).finished();
                    }
                    break;



                case 9:
                    Ikresidual_func = sym::IkResidualFuncCost3Nl9(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl9 = sym::IkResidualFuncCost3WrtFh1Nl9(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl9).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl9 = sym::IkResidualFuncCost3WrtFv1Nl9(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl9).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl9 = sym::IkResidualFuncCost3WrtRxNl9(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl9).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl9 = sym::IkResidualFuncCost3WrtRyNl9(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl9).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl9 = sym::IkResidualFuncCost3WrtRzNl9(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl9).finished();
                    }
                    break;



                case 10:
                    Ikresidual_func = sym::IkResidualFuncCost3Nl10(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl10 = sym::IkResidualFuncCost3WrtFh1Nl10(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl10).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl10 = sym::IkResidualFuncCost3WrtFv1Nl10(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl10).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl10 = sym::IkResidualFuncCost3WrtRxNl10(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl10).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl10 = sym::IkResidualFuncCost3WrtRyNl10(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl10).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl10 = sym::IkResidualFuncCost3WrtRzNl10(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl10).finished();
                    }
                    break;



                case 11:
                    Ikresidual_func = sym::IkResidualFuncCost3Nl11(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl11 = sym::IkResidualFuncCost3WrtFh1Nl11(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl11).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl11 = sym::IkResidualFuncCost3WrtFv1Nl11(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl11).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl11 = sym::IkResidualFuncCost3WrtRxNl11(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl11).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl11 = sym::IkResidualFuncCost3WrtRyNl11(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl11).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl11 = sym::IkResidualFuncCost3WrtRzNl11(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl11).finished();
                    }
                    break;



                case 12:
                    Ikresidual_func = sym::IkResidualFuncCost3Nl12(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl12 = sym::IkResidualFuncCost3WrtFh1Nl12(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl12).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl12 = sym::IkResidualFuncCost3WrtFv1Nl12(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl12).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl12 = sym::IkResidualFuncCost3WrtRxNl12(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl12).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl12 = sym::IkResidualFuncCost3WrtRyNl12(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl12).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl12 = sym::IkResidualFuncCost3WrtRzNl12(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl12).finished();
                    }
                    break;



                case 13:
                    Ikresidual_func = sym::IkResidualFuncCost3Nl13(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl13 = sym::IkResidualFuncCost3WrtFh1Nl13(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl13).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl13 = sym::IkResidualFuncCost3WrtFv1Nl13(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl13).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl13 = sym::IkResidualFuncCost3WrtRxNl13(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl13).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl13 = sym::IkResidualFuncCost3WrtRyNl13(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl13).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl13 = sym::IkResidualFuncCost3WrtRzNl13(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl13).finished();
                    }
                    break;



                case 14:
                    Ikresidual_func = sym::IkResidualFuncCost3Nl14(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl14 = sym::IkResidualFuncCost3WrtFh1Nl14(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl14).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl14 = sym::IkResidualFuncCost3WrtFv1Nl14(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl14).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl14 = sym::IkResidualFuncCost3WrtRxNl14(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl14).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl14 = sym::IkResidualFuncCost3WrtRyNl14(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl14).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl14 = sym::IkResidualFuncCost3WrtRzNl14(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl14).finished();
                    }
                    break;



                case 15:
                    Ikresidual_func = sym::IkResidualFuncCost3Nl15(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl15 = sym::IkResidualFuncCost3WrtFh1Nl15(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl15).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl15 = sym::IkResidualFuncCost3WrtFv1Nl15(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl15).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl15 = sym::IkResidualFuncCost3WrtRxNl15(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl15).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl15 = sym::IkResidualFuncCost3WrtRyNl15(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl15).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl15 = sym::IkResidualFuncCost3WrtRzNl15(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl15).finished();
                    }
                    break;



                case 16:
                    Ikresidual_func = sym::IkResidualFuncCost3Nl16(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl16 = sym::IkResidualFuncCost3WrtFh1Nl16(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl16).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl16 = sym::IkResidualFuncCost3WrtFv1Nl16(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl16).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl16 = sym::IkResidualFuncCost3WrtRxNl16(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl16).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl16 = sym::IkResidualFuncCost3WrtRyNl16(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl16).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl16 = sym::IkResidualFuncCost3WrtRzNl16(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl16).finished();
                    }
                    break;



                case 17:
                    Ikresidual_func = sym::IkResidualFuncCost3Nl17(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl17 = sym::IkResidualFuncCost3WrtFh1Nl17(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl17).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl17 = sym::IkResidualFuncCost3WrtFv1Nl17(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl17).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl17 = sym::IkResidualFuncCost3WrtRxNl17(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl17).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl17 = sym::IkResidualFuncCost3WrtRyNl17(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl17).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl17 = sym::IkResidualFuncCost3WrtRzNl17(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl17).finished();
                    }
                    break;


                case 18:
                    Ikresidual_func = sym::IkResidualFuncCost3Nl18(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl18 = sym::IkResidualFuncCost3WrtFh1Nl18(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl18).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl18 = sym::IkResidualFuncCost3WrtFv1Nl18(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl18).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl18 = sym::IkResidualFuncCost3WrtRxNl18(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl18).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl18 = sym::IkResidualFuncCost3WrtRyNl18(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl18).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl18 = sym::IkResidualFuncCost3WrtRzNl18(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl18).finished();
                    }
                    break;



                case 19:
                    Ikresidual_func = sym::IkResidualFuncCost3Nl19(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl19 = sym::IkResidualFuncCost3WrtFh1Nl19(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl19).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl19 = sym::IkResidualFuncCost3WrtFv1Nl19(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl19).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl19 = sym::IkResidualFuncCost3WrtRxNl19(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl19).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl19 = sym::IkResidualFuncCost3WrtRyNl19(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl19).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl19 = sym::IkResidualFuncCost3WrtRzNl19(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl19).finished();
                    }
                    break;



                case 20:
                    Ikresidual_func = sym::IkResidualFuncCost3Nl20(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl20 = sym::IkResidualFuncCost3WrtFh1Nl20(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl20).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl20 = sym::IkResidualFuncCost3WrtFv1Nl20(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl20).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl20 = sym::IkResidualFuncCost3WrtRxNl20(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl20).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl20 = sym::IkResidualFuncCost3WrtRyNl20(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl20).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl20 = sym::IkResidualFuncCost3WrtRzNl20(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl20).finished();
                    }
                    break;



                case 21:
                    Ikresidual_func = sym::IkResidualFuncCost3Nl21(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl21 = sym::IkResidualFuncCost3WrtFh1Nl21(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl21).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl21 = sym::IkResidualFuncCost3WrtFv1Nl21(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl21).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl21 = sym::IkResidualFuncCost3WrtRxNl21(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl21).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl21 = sym::IkResidualFuncCost3WrtRyNl21(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl21).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl21 = sym::IkResidualFuncCost3WrtRzNl21(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl21).finished();
                    }
                    break;



                case 22:
                    Ikresidual_func = sym::IkResidualFuncCost3Nl22(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl22 = sym::IkResidualFuncCost3WrtFh1Nl22(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl22).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl22 = sym::IkResidualFuncCost3WrtFv1Nl22(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl22).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl22 = sym::IkResidualFuncCost3WrtRxNl22(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl22).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl22 = sym::IkResidualFuncCost3WrtRyNl22(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl22).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl22 = sym::IkResidualFuncCost3WrtRzNl22(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl22).finished();
                    }
                    break;



                case 23:
                    Ikresidual_func = sym::IkResidualFuncCost3Nl23(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1_Nl23 = sym::IkResidualFuncCost3WrtFh1Nl23(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1_Nl23).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1_Nl23 = sym::IkResidualFuncCost3WrtFv1Nl23(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1_Nl23).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx_Nl23 = sym::IkResidualFuncCost3WrtRxNl23(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx_Nl23).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry_Nl23 = sym::IkResidualFuncCost3WrtRyNl23(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry_Nl23).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz_Nl23 = sym::IkResidualFuncCost3WrtRzNl23(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz_Nl23).finished();
                    }
                    break;
            }
            return (Vector(4) << Ikresidual_func).finished();
        }
    };
}