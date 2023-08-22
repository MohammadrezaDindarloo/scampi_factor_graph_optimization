#pragma once

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <boost/optional.hpp>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/inference/Symbol.h>
#include "function_headers/include_file.h"



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
            Eigen::Matrix<double, 4, 1> resedual_func;

            if (largest_cable == 1)
            {
                resedual_func = sym::ResedualFuncCost1L1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                if (H1)
                {
                    Eigen::Matrix<double, 4, 1> resedual_func_wrt_fh1 = sym::ResedualFuncCost1WrtFh1L1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    *H1 = (Matrix(4, 1) << resedual_func_wrt_fh1).finished();
                }
                if (H2)
                {   
                    Eigen::Matrix<double, 4, 1> resedual_func_wrt_fv1 = sym::ResedualFuncCost1WrtFv1L1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    *H2 = (Matrix(4, 1) << resedual_func_wrt_fv1).finished();
                }
                if (H3)
                {   
                    Eigen::Matrix<double, 4, 1> resedual_func_wrt_rx = sym::ResedualFuncCost1WrtRxL1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    *H3 = (Matrix(4, 1) << resedual_func_wrt_rx).finished();
                }
                if (H4)
                {   
                    Eigen::Matrix<double, 4, 1> resedual_func_wrt_ry = sym::ResedualFuncCost1WrtRyL1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    *H4 = (Matrix(4, 1) << resedual_func_wrt_ry).finished();
                }
                if (H5)
                {   
                    Eigen::Matrix<double, 4, 1> resedual_func_wrt_rz = sym::ResedualFuncCost1WrtRzL1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    *H5 = (Matrix(4, 1) << resedual_func_wrt_rz).finished();
                }

                return (Vector(4) << resedual_func).finished();
            }

            if (largest_cable == 2)
            {
                resedual_func = sym::ResedualFuncCost1L2(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                if (H1)
                {
                    Eigen::Matrix<double, 4, 1> resedual_func_wrt_fh1 = sym::ResedualFuncCost1WrtFh1L2(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    *H1 = (Matrix(4, 1) << resedual_func_wrt_fh1).finished();
                }
                if (H2)
                {   
                    Eigen::Matrix<double, 4, 1> resedual_func_wrt_fv1 = sym::ResedualFuncCost1WrtFv1L2(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    *H2 = (Matrix(4, 1) << resedual_func_wrt_fv1).finished();
                }
                if (H3)
                {   
                    Eigen::Matrix<double, 4, 1> resedual_func_wrt_rx = sym::ResedualFuncCost1WrtRxL2(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    *H3 = (Matrix(4, 1) << resedual_func_wrt_rx).finished();
                }
                if (H4)
                {   
                    Eigen::Matrix<double, 4, 1> resedual_func_wrt_ry = sym::ResedualFuncCost1WrtRyL2(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    *H4 = (Matrix(4, 1) << resedual_func_wrt_ry).finished();
                }
                if (H5)
                {   
                    Eigen::Matrix<double, 4, 1> resedual_func_wrt_rz = sym::ResedualFuncCost1WrtRzL2(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    *H5 = (Matrix(4, 1) << resedual_func_wrt_rz).finished();
                }

                return (Vector(4) << resedual_func).finished();
            }

            if (largest_cable == 3)
            {
                resedual_func = sym::ResedualFuncCost1L3(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                if (H1)
                {
                    Eigen::Matrix<double, 4, 1> resedual_func_wrt_fh1 = sym::ResedualFuncCost1WrtFh1L3(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    *H1 = (Matrix(4, 1) << resedual_func_wrt_fh1).finished();
                }
                if (H2)
                {   
                    Eigen::Matrix<double, 4, 1> resedual_func_wrt_fv1 = sym::ResedualFuncCost1WrtFv1L3(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    *H2 = (Matrix(4, 1) << resedual_func_wrt_fv1).finished();
                }
                if (H3)
                {   
                    Eigen::Matrix<double, 4, 1> resedual_func_wrt_rx = sym::ResedualFuncCost1WrtRxL3(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    *H3 = (Matrix(4, 1) << resedual_func_wrt_rx).finished();
                }
                if (H4)
                {   
                    Eigen::Matrix<double, 4, 1> resedual_func_wrt_ry = sym::ResedualFuncCost1WrtRyL3(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    *H4 = (Matrix(4, 1) << resedual_func_wrt_ry).finished();
                }
                if (H5)
                {   
                    Eigen::Matrix<double, 4, 1> resedual_func_wrt_rz = sym::ResedualFuncCost1WrtRzL3(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    *H5 = (Matrix(4, 1) << resedual_func_wrt_rz).finished();
                }

                return (Vector(4) << resedual_func).finished();
            }

            if (largest_cable == 4)
            {
                resedual_func = sym::ResedualFuncCost1L4(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                if (H1)
                {
                    Eigen::Matrix<double, 4, 1> resedual_func_wrt_fh1 = sym::ResedualFuncCost1WrtFh1L4(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    *H1 = (Matrix(4, 1) << resedual_func_wrt_fh1).finished();
                }
                if (H2)
                {   
                    Eigen::Matrix<double, 4, 1> resedual_func_wrt_fv1 = sym::ResedualFuncCost1WrtFv1L4(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    *H2 = (Matrix(4, 1) << resedual_func_wrt_fv1).finished();
                }
                if (H3)
                {   
                    Eigen::Matrix<double, 4, 1> resedual_func_wrt_rx = sym::ResedualFuncCost1WrtRxL4(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    *H3 = (Matrix(4, 1) << resedual_func_wrt_rx).finished();
                }
                if (H4)
                {   
                    Eigen::Matrix<double, 4, 1> resedual_func_wrt_ry = sym::ResedualFuncCost1WrtRyL4(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    *H4 = (Matrix(4, 1) << resedual_func_wrt_ry).finished();
                }
                if (H5)
                {   
                    Eigen::Matrix<double, 4, 1> resedual_func_wrt_rz = sym::ResedualFuncCost1WrtRzL4(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    *H5 = (Matrix(4, 1) << resedual_func_wrt_rz).finished();
                }

                
            }
            
            return (Vector(4) << resedual_func).finished();
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
            Eigen::Matrix<double, 4, 1> resedual_func;

            if (largest_cable == 1)
            {
                resedual_func = sym::ResedualFuncCost2L1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                if (H1)
                {
                    Eigen::Matrix<double, 4, 1> resedual_func_wrt_fh1 = sym::ResedualFuncCost2WrtFh1L1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    *H1 = (Matrix(4, 1) << resedual_func_wrt_fh1).finished();
                }
                if (H2)
                {   
                    Eigen::Matrix<double, 4, 1> resedual_func_wrt_fv1 = sym::ResedualFuncCost2WrtFv1L1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    *H2 = (Matrix(4, 1) << resedual_func_wrt_fv1).finished();
                }
                if (H3)
                {   
                    Eigen::Matrix<double, 4, 1> resedual_func_wrt_rx = sym::ResedualFuncCost2WrtRxL1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    *H3 = (Matrix(4, 1) << resedual_func_wrt_rx).finished();
                }
                if (H4)
                {   
                    Eigen::Matrix<double, 4, 1> resedual_func_wrt_ry = sym::ResedualFuncCost2WrtRyL1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    *H4 = (Matrix(4, 1) << resedual_func_wrt_ry).finished();
                }
                if (H5)
                {   
                    Eigen::Matrix<double, 4, 1> resedual_func_wrt_rz = sym::ResedualFuncCost2WrtRzL1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    *H5 = (Matrix(4, 1) << resedual_func_wrt_rz).finished();
                }

                return (Vector(4) << resedual_func).finished();
            }

            if (largest_cable == 2)
            {
                resedual_func = sym::ResedualFuncCost2L2(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                if (H1)
                {
                    Eigen::Matrix<double, 4, 1> resedual_func_wrt_fh1 = sym::ResedualFuncCost2WrtFh1L2(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    *H1 = (Matrix(4, 1) << resedual_func_wrt_fh1).finished();
                }
                if (H2)
                {   
                    Eigen::Matrix<double, 4, 1> resedual_func_wrt_fv1 = sym::ResedualFuncCost2WrtFv1L2(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    *H2 = (Matrix(4, 1) << resedual_func_wrt_fv1).finished();
                }
                if (H3)
                {   
                    Eigen::Matrix<double, 4, 1> resedual_func_wrt_rx = sym::ResedualFuncCost2WrtRxL2(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    *H3 = (Matrix(4, 1) << resedual_func_wrt_rx).finished();
                }
                if (H4)
                {   
                    Eigen::Matrix<double, 4, 1> resedual_func_wrt_ry = sym::ResedualFuncCost2WrtRyL2(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    *H4 = (Matrix(4, 1) << resedual_func_wrt_ry).finished();
                }
                if (H5)
                {   
                    Eigen::Matrix<double, 4, 1> resedual_func_wrt_rz = sym::ResedualFuncCost2WrtRzL2(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    *H5 = (Matrix(4, 1) << resedual_func_wrt_rz).finished();
                }

                return (Vector(4) << resedual_func).finished();
            }

            if (largest_cable == 3)
            {
                resedual_func = sym::ResedualFuncCost2L3(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                if (H1)
                {
                    Eigen::Matrix<double, 4, 1> resedual_func_wrt_fh1 = sym::ResedualFuncCost2WrtFh1L3(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    *H1 = (Matrix(4, 1) << resedual_func_wrt_fh1).finished();
                }
                if (H2)
                {   
                    Eigen::Matrix<double, 4, 1> resedual_func_wrt_fv1 = sym::ResedualFuncCost2WrtFv1L3(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    *H2 = (Matrix(4, 1) << resedual_func_wrt_fv1).finished();
                }
                if (H3)
                {   
                    Eigen::Matrix<double, 4, 1> resedual_func_wrt_rx = sym::ResedualFuncCost2WrtRxL3(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    *H3 = (Matrix(4, 1) << resedual_func_wrt_rx).finished();
                }
                if (H4)
                {   
                    Eigen::Matrix<double, 4, 1> resedual_func_wrt_ry = sym::ResedualFuncCost2WrtRyL3(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    *H4 = (Matrix(4, 1) << resedual_func_wrt_ry).finished();
                }
                if (H5)
                {   
                    Eigen::Matrix<double, 4, 1> resedual_func_wrt_rz = sym::ResedualFuncCost2WrtRzL3(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    *H5 = (Matrix(4, 1) << resedual_func_wrt_rz).finished();
                }

                return (Vector(4) << resedual_func).finished();
            }

            if (largest_cable == 4)
            {
                resedual_func = sym::ResedualFuncCost2L4(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                if (H1)
                {
                    Eigen::Matrix<double, 4, 1> resedual_func_wrt_fh1 = sym::ResedualFuncCost2WrtFh1L4(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    *H1 = (Matrix(4, 1) << resedual_func_wrt_fh1).finished();
                }
                if (H2)
                {   
                    Eigen::Matrix<double, 4, 1> resedual_func_wrt_fv1 = sym::ResedualFuncCost2WrtFv1L4(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    *H2 = (Matrix(4, 1) << resedual_func_wrt_fv1).finished();
                }
                if (H3)
                {   
                    Eigen::Matrix<double, 4, 1> resedual_func_wrt_rx = sym::ResedualFuncCost2WrtRxL4(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    *H3 = (Matrix(4, 1) << resedual_func_wrt_rx).finished();
                }
                if (H4)
                {   
                    Eigen::Matrix<double, 4, 1> resedual_func_wrt_ry = sym::ResedualFuncCost2WrtRyL4(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    *H4 = (Matrix(4, 1) << resedual_func_wrt_ry).finished();
                }
                if (H5)
                {   
                    Eigen::Matrix<double, 4, 1> resedual_func_wrt_rz = sym::ResedualFuncCost2WrtRzL4(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    *H5 = (Matrix(4, 1) << resedual_func_wrt_rz).finished();
                }

                
            }
            
            return (Vector(4) << resedual_func).finished(); 
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
            Eigen::Matrix<double, 4, 1> resedual_func; 

            if (largest_cable == 1)
            {   
                resedual_func = sym::ResedualFuncCost3L1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                if (H1)
                {
                    Eigen::Matrix<double, 4, 1> resedual_func_wrt_fh1 = sym::ResedualFuncCost3WrtFh1L1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    *H1 = (Matrix(4, 1) << resedual_func_wrt_fh1).finished();
                }
                if (H2)
                {   
                    Eigen::Matrix<double, 4, 1> resedual_func_wrt_fv1 = sym::ResedualFuncCost3WrtFv1L1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    *H2 = (Matrix(4, 1) << resedual_func_wrt_fv1).finished();
                }
                if (H3)
                {   
                    Eigen::Matrix<double, 4, 1> resedual_func_wrt_rx = sym::ResedualFuncCost3WrtRxL1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    *H3 = (Matrix(4, 1) << resedual_func_wrt_rx).finished();
                }
                if (H4)
                {   
                    Eigen::Matrix<double, 4, 1> resedual_func_wrt_ry = sym::ResedualFuncCost3WrtRyL1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    *H4 = (Matrix(4, 1) << resedual_func_wrt_ry).finished();
                }
                if (H5)
                {   
                    Eigen::Matrix<double, 4, 1> resedual_func_wrt_rz = sym::ResedualFuncCost3WrtRzL1(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    *H5 = (Matrix(4, 1) << resedual_func_wrt_rz).finished();
                }

                return (Vector(4) << resedual_func).finished();
            }

            if (largest_cable == 2)
            {
                resedual_func = sym::ResedualFuncCost3L2(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                if (H1)
                {
                    Eigen::Matrix<double, 4, 1> resedual_func_wrt_fh1 = sym::ResedualFuncCost3WrtFh1L2(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    *H1 = (Matrix(4, 1) << resedual_func_wrt_fh1).finished();
                }
                if (H2)
                {   
                    Eigen::Matrix<double, 4, 1> resedual_func_wrt_fv1 = sym::ResedualFuncCost3WrtFv1L2(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    *H2 = (Matrix(4, 1) << resedual_func_wrt_fv1).finished();
                }
                if (H3)
                {   
                    Eigen::Matrix<double, 4, 1> resedual_func_wrt_rx = sym::ResedualFuncCost3WrtRxL2(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    *H3 = (Matrix(4, 1) << resedual_func_wrt_rx).finished();
                }
                if (H4)
                {   
                    Eigen::Matrix<double, 4, 1> resedual_func_wrt_ry = sym::ResedualFuncCost3WrtRyL2(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    *H4 = (Matrix(4, 1) << resedual_func_wrt_ry).finished();
                }
                if (H5)
                {   
                    Eigen::Matrix<double, 4, 1> resedual_func_wrt_rz = sym::ResedualFuncCost3WrtRzL2(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    *H5 = (Matrix(4, 1) << resedual_func_wrt_rz).finished();
                }

                return (Vector(4) << resedual_func).finished();
            }

            if (largest_cable == 3)
            {
                resedual_func = sym::ResedualFuncCost3L3(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                if (H1)
                {
                    Eigen::Matrix<double, 4, 1> resedual_func_wrt_fh1 = sym::ResedualFuncCost3WrtFh1L3(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    *H1 = (Matrix(4, 1) << resedual_func_wrt_fh1).finished();
                }
                if (H2)
                {   
                    Eigen::Matrix<double, 4, 1> resedual_func_wrt_fv1 = sym::ResedualFuncCost3WrtFv1L3(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    *H2 = (Matrix(4, 1) << resedual_func_wrt_fv1).finished();
                }
                if (H3)
                {   
                    Eigen::Matrix<double, 4, 1> resedual_func_wrt_rx = sym::ResedualFuncCost3WrtRxL3(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    *H3 = (Matrix(4, 1) << resedual_func_wrt_rx).finished();
                }
                if (H4)
                {   
                    Eigen::Matrix<double, 4, 1> resedual_func_wrt_ry = sym::ResedualFuncCost3WrtRyL3(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    *H4 = (Matrix(4, 1) << resedual_func_wrt_ry).finished();
                }
                if (H5)
                {   
                    Eigen::Matrix<double, 4, 1> resedual_func_wrt_rz = sym::ResedualFuncCost3WrtRzL3(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    *H5 = (Matrix(4, 1) << resedual_func_wrt_rz).finished();
                }

                return (Vector(4) << resedual_func).finished();
            }

            if (largest_cable == 4)
            {
                resedual_func = sym::ResedualFuncCost3L4(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                if (H1)
                {
                    Eigen::Matrix<double, 4, 1> resedual_func_wrt_fh1 = sym::ResedualFuncCost3WrtFh1L4(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    *H1 = (Matrix(4, 1) << resedual_func_wrt_fh1).finished();
                }
                if (H2)
                {   
                    Eigen::Matrix<double, 4, 1> resedual_func_wrt_fv1 = sym::ResedualFuncCost3WrtFv1L4(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    *H2 = (Matrix(4, 1) << resedual_func_wrt_fv1).finished();
                }
                if (H3)
                {   
                    Eigen::Matrix<double, 4, 1> resedual_func_wrt_rx = sym::ResedualFuncCost3WrtRxL4(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    *H3 = (Matrix(4, 1) << resedual_func_wrt_rx).finished();
                }
                if (H4)
                {   
                    Eigen::Matrix<double, 4, 1> resedual_func_wrt_ry = sym::ResedualFuncCost3WrtRyL4(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    *H4 = (Matrix(4, 1) << resedual_func_wrt_ry).finished();
                }
                if (H5)
                {   
                    Eigen::Matrix<double, 4, 1> resedual_func_wrt_rz = sym::ResedualFuncCost3WrtRzL4(fh1, fv1, rx, ry, rz, p_init0, p_init1, p_init2, rot_init_x, rot_init_y, rot_init_z, rot_init_w, epsilon);
                    *H5 = (Matrix(4, 1) << resedual_func_wrt_rz).finished();
                }

                
            }
            
            return (Vector(4) << resedual_func).finished();
        }
    };


}