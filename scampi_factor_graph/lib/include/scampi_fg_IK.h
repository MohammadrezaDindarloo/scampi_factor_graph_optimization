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
                case 1:
                    Ikresidual_func = sym::IkResidualFuncCost1L1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1 = sym::IkResidualFuncCost1WrtFh1L1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1 = sym::IkResidualFuncCost1WrtFv1L1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx = sym::IkResidualFuncCost1WrtRxL1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry = sym::IkResidualFuncCost1WrtRyL1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz = sym::IkResidualFuncCost1WrtRzL1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz).finished();
                    }
                    break;

                case 2:
                    Ikresidual_func = sym::IkResidualFuncCost1L2(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1 = sym::IkResidualFuncCost1WrtFh1L2(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1 = sym::IkResidualFuncCost1WrtFv1L2(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx = sym::IkResidualFuncCost1WrtRxL2(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry = sym::IkResidualFuncCost1WrtRyL2(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz = sym::IkResidualFuncCost1WrtRzL2(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz).finished();
                    }
                    break;

                case 3:
                    Ikresidual_func = sym::IkResidualFuncCost1L3(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1 = sym::IkResidualFuncCost1WrtFh1L3(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1 = sym::IkResidualFuncCost1WrtFv1L3(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx = sym::IkResidualFuncCost1WrtRxL3(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry = sym::IkResidualFuncCost1WrtRyL3(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz = sym::IkResidualFuncCost1WrtRzL3(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz).finished();
                    }
                    break;

                case 4:
                    Ikresidual_func = sym::IkResidualFuncCost1L4(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1 = sym::IkResidualFuncCost1WrtFh1L4(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1 = sym::IkResidualFuncCost1WrtFv1L4(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx = sym::IkResidualFuncCost1WrtRxL4(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry = sym::IkResidualFuncCost1WrtRyL4(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz = sym::IkResidualFuncCost1WrtRzL4(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz).finished();
                    }
                    break;
                default:
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
                case 1:
                    Ikresidual_func = sym::IkResidualFuncCost2L1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1 = sym::IkResidualFuncCost2WrtFh1L1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1 = sym::IkResidualFuncCost2WrtFv1L1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx = sym::IkResidualFuncCost2WrtRxL1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry = sym::IkResidualFuncCost2WrtRyL1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz = sym::IkResidualFuncCost2WrtRzL1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz).finished();
                    }
                    break;

                case 2:
                    Ikresidual_func = sym::IkResidualFuncCost2L2(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1 = sym::IkResidualFuncCost2WrtFh1L2(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1 = sym::IkResidualFuncCost2WrtFv1L2(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx = sym::IkResidualFuncCost2WrtRxL2(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry = sym::IkResidualFuncCost2WrtRyL2(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz = sym::IkResidualFuncCost2WrtRzL2(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz).finished();
                    }
                    break;

                case 3:
                    Ikresidual_func = sym::IkResidualFuncCost2L3(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1 = sym::IkResidualFuncCost2WrtFh1L3(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1 = sym::IkResidualFuncCost2WrtFv1L3(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx = sym::IkResidualFuncCost2WrtRxL3(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry = sym::IkResidualFuncCost2WrtRyL3(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz = sym::IkResidualFuncCost2WrtRzL3(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz).finished();
                    }
                    break;

                case 4:
                    Ikresidual_func = sym::IkResidualFuncCost2L4(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1 = sym::IkResidualFuncCost2WrtFh1L4(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1 = sym::IkResidualFuncCost2WrtFv1L4(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx = sym::IkResidualFuncCost2WrtRxL4(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry = sym::IkResidualFuncCost2WrtRyL4(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz = sym::IkResidualFuncCost2WrtRzL4(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz).finished();
                    }
                    break;  
                
                default:
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
                case 1:   
                    Ikresidual_func = sym::IkResidualFuncCost3L1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1 = sym::IkResidualFuncCost3WrtFh1L1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1 = sym::IkResidualFuncCost3WrtFv1L1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx = sym::IkResidualFuncCost3WrtRxL1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry = sym::IkResidualFuncCost3WrtRyL1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz = sym::IkResidualFuncCost3WrtRzL1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz).finished();
                    }
                    break;

                case 2:
                    Ikresidual_func = sym::IkResidualFuncCost3L2(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1 = sym::IkResidualFuncCost3WrtFh1L2(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1 = sym::IkResidualFuncCost3WrtFv1L2(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx = sym::IkResidualFuncCost3WrtRxL2(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry = sym::IkResidualFuncCost3WrtRyL2(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz = sym::IkResidualFuncCost3WrtRzL2(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz).finished();
                    }
                    break;

                case 3:
                    Ikresidual_func = sym::IkResidualFuncCost3L3(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1 = sym::IkResidualFuncCost3WrtFh1L3(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1 = sym::IkResidualFuncCost3WrtFv1L3(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx = sym::IkResidualFuncCost3WrtRxL3(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry = sym::IkResidualFuncCost3WrtRyL3(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz = sym::IkResidualFuncCost3WrtRzL3(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz).finished();
                    }
                    break;

                case 4:
                    Ikresidual_func = sym::IkResidualFuncCost3L4(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    if (H1)
                    {
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fh1 = sym::IkResidualFuncCost3WrtFh1L4(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H1 = (Matrix(4, 1) << Ikresidual_func_wrt_fh1).finished();
                    }
                    if (H2)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_fv1 = sym::IkResidualFuncCost3WrtFv1L4(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H2 = (Matrix(4, 1) << Ikresidual_func_wrt_fv1).finished();
                    }
                    if (H3)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rx = sym::IkResidualFuncCost3WrtRxL4(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H3 = (Matrix(4, 1) << Ikresidual_func_wrt_rx).finished();
                    }
                    if (H4)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_ry = sym::IkResidualFuncCost3WrtRyL4(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H4 = (Matrix(4, 1) << Ikresidual_func_wrt_ry).finished();
                    }
                    if (H5)
                    {   
                        Eigen::Matrix<double, 4, 1> Ikresidual_func_wrt_rz = sym::IkResidualFuncCost3WrtRzL4(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                        *H5 = (Matrix(4, 1) << Ikresidual_func_wrt_rz).finished();
                    }
                    break;
                
                default:
                    break;
            }
            return (Vector(4) << Ikresidual_func).finished();
        }
    };
}