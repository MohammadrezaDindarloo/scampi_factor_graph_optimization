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
    class FK_factor_graoh_cost1 : public NoiseModelFactorN<double, double, double, double, double, double, double, double>
    {

    private:
        double lc0;
        double lc1; 
        double lc2; 
        double lc3;         
        double rot_init_x; 
        double rot_init_y; 
        double rot_init_z; 
        double rot_init_w; 
        double epsilon = 0.0;

    public:
        // Constructor
        FK_factor_graoh_cost1(Key key1, Key key2, Key key3, Key key4, Key key5, Key key6, Key key7, Key key8, double lc0_, double lc1_, double lc2_, double lc3_, double rot_init_x_, double rot_init_y_, double rot_init_z_, double rot_init_w_, const SharedNoiseModel &model) 
        : NoiseModelFactorN<double, double, double, double, double, double, double, double>(model, key1, key2, key3, key4, key5, key6, key7, key8), lc0(lc0_), lc1(lc1_), lc2(lc2_), lc3(lc3_), rot_init_x(rot_init_x_), rot_init_y(rot_init_y_), rot_init_z(rot_init_z_), rot_init_w(rot_init_w_) {}

        // Evaluate the error
        Vector evaluateError(const double &fh1, const double &fv1, const double &rx, const double &ry, const double &rz, const double &tx, const double &ty, const double &tz,
                             OptionalMatrixType H1,
                             OptionalMatrixType H2,
                             OptionalMatrixType H3,
                             OptionalMatrixType H4,
                             OptionalMatrixType H5,
                             OptionalMatrixType H6,
                             OptionalMatrixType H7,
                             OptionalMatrixType H8) const override
        {   
            Eigen::Matrix<double, 4, 1> Fkresidual_func = sym::FkResidualFuncCost1(fh1, fv1, rx, ry, rz, tx, ty, tz, rot_init_x, rot_init_y, rot_init_z, rot_init_w, lc0, lc1, lc2, lc3, epsilon);
            if(H1)
            {
                Eigen::Matrix<double, 4, 1> Fkresidual_func_wrt_fh1 = sym::FkResidualFuncCost1WrtFh1(fh1, fv1, rx, ry, rz, tx, ty, tz, rot_init_x, rot_init_y, rot_init_z, rot_init_w, lc0, lc1, lc2, lc3, epsilon);
                *H1 = (Matrix(4, 1) << Fkresidual_func_wrt_fh1).finished();
            }
            if(H2)
            {
                Eigen::Matrix<double, 4, 1> Fkresidual_func_wrt_fv1 = sym::FkResidualFuncCost1WrtFv1(fh1, fv1, rx, ry, rz, tx, ty, tz, rot_init_x, rot_init_y, rot_init_z, rot_init_w, lc0, lc1, lc2, lc3, epsilon);
                *H2 = (Matrix(4, 1) << Fkresidual_func_wrt_fv1).finished();
            }
            if(H3)
            {
                Eigen::Matrix<double, 4, 1> Fkresidual_func_wrt_rx = sym::FkResidualFuncCost1WrtRx(fh1, fv1, rx, ry, rz, tx, ty, tz, rot_init_x, rot_init_y, rot_init_z, rot_init_w, lc0, lc1, lc2, lc3, epsilon);
                *H3 = (Matrix(4, 1) << Fkresidual_func_wrt_rx).finished();
            }
            if(H4)
            {
                Eigen::Matrix<double, 4, 1> Fkresidual_func_wrt_ry = sym::FkResidualFuncCost1WrtRy(fh1, fv1, rx, ry, rz, tx, ty, tz, rot_init_x, rot_init_y, rot_init_z, rot_init_w, lc0, lc1, lc2, lc3, epsilon);
                *H4 = (Matrix(4, 1) << Fkresidual_func_wrt_ry).finished();
            }
            if(H5)
            {
                Eigen::Matrix<double, 4, 1> Fkresidual_func_wrt_rz = sym::FkResidualFuncCost1WrtRz(fh1, fv1, rx, ry, rz, tx, ty, tz, rot_init_x, rot_init_y, rot_init_z, rot_init_w, lc0, lc1, lc2, lc3, epsilon);
                *H5 = (Matrix(4, 1) << Fkresidual_func_wrt_rz).finished();
            }
            if(H6)
            {
                Eigen::Matrix<double, 4, 1> Fkresidual_func_wrt_tx = sym::FkResidualFuncCost1WrtTx(fh1, fv1, rx, ry, rz, tx, ty, tz, rot_init_x, rot_init_y, rot_init_z, rot_init_w, lc0, lc1, lc2, lc3, epsilon);
                *H6 = (Matrix(4, 1) << Fkresidual_func_wrt_tx).finished();
            }
            if(H7)
            {
                Eigen::Matrix<double, 4, 1> Fkresidual_func_wrt_ty = sym::FkResidualFuncCost1WrtTy(fh1, fv1, rx, ry, rz, tx, ty, tz, rot_init_x, rot_init_y, rot_init_z, rot_init_w, lc0, lc1, lc2, lc3, epsilon);
                *H7 = (Matrix(4, 1) << Fkresidual_func_wrt_ty).finished();
            }
            if(H8)
            {
                Eigen::Matrix<double, 4, 1> Fkresidual_func_wrt_tz = sym::FkResidualFuncCost1WrtTz(fh1, fv1, rx, ry, rz, tx, ty, tz, rot_init_x, rot_init_y, rot_init_z, rot_init_w, lc0, lc1, lc2, lc3, epsilon);
                *H8 = (Matrix(4, 1) << Fkresidual_func_wrt_tz).finished();
            }

            return (Vector(4) << Fkresidual_func).finished();
        }
    };


    class FK_factor_graoh_cost2 : public NoiseModelFactorN<double, double, double, double, double, double, double, double>
    {

    private:
        double lc0;
        double lc1; 
        double lc2; 
        double lc3;         
        double rot_init_x; 
        double rot_init_y; 
        double rot_init_z; 
        double rot_init_w; 
        double epsilon = 0.0;

    public:
        // Constructor
        FK_factor_graoh_cost2(Key key1, Key key2, Key key3, Key key4, Key key5, Key key6, Key key7, Key key8, double lc0_, double lc1_, double lc2_, double lc3_, double rot_init_x_, double rot_init_y_, double rot_init_z_, double rot_init_w_, const SharedNoiseModel &model) 
        : NoiseModelFactorN<double, double, double, double, double, double, double, double>(model, key1, key2, key3, key4, key5, key6, key7, key8), lc0(lc0_), lc1(lc1_), lc2(lc2_), lc3(lc3_), rot_init_x(rot_init_x_), rot_init_y(rot_init_y_), rot_init_z(rot_init_z_), rot_init_w(rot_init_w_) {}

        // Evaluate the error
        Vector evaluateError(const double &fh1, const double &fv1, const double &rx, const double &ry, const double &rz, const double &tx, const double &ty, const double &tz,
                             OptionalMatrixType H1,
                             OptionalMatrixType H2,
                             OptionalMatrixType H3,
                             OptionalMatrixType H4,
                             OptionalMatrixType H5,
                             OptionalMatrixType H6,
                             OptionalMatrixType H7,
                             OptionalMatrixType H8) const override
        {   
            Eigen::Matrix<double, 4, 1> Fkresidual_func = sym::FkResidualFuncCost2(fh1, fv1, rx, ry, rz, tx, ty, tz, rot_init_x, rot_init_y, rot_init_z, rot_init_w, lc0, lc1, lc2, lc3, epsilon);
            if(H1)
            {
                Eigen::Matrix<double, 4, 1> Fkresidual_func_wrt_fh1 = sym::FkResidualFuncCost2WrtFh1(fh1, fv1, rx, ry, rz, tx, ty, tz, rot_init_x, rot_init_y, rot_init_z, rot_init_w, lc0, lc1, lc2, lc3, epsilon);
                *H1 = (Matrix(4, 1) << Fkresidual_func_wrt_fh1).finished();
            }
            if(H2)
            {
                Eigen::Matrix<double, 4, 1> Fkresidual_func_wrt_fv1 = sym::FkResidualFuncCost2WrtFv1(fh1, fv1, rx, ry, rz, tx, ty, tz, rot_init_x, rot_init_y, rot_init_z, rot_init_w, lc0, lc1, lc2, lc3, epsilon);
                *H2 = (Matrix(4, 1) << Fkresidual_func_wrt_fv1).finished();
            }
            if(H3)
            {
                Eigen::Matrix<double, 4, 1> Fkresidual_func_wrt_rx = sym::FkResidualFuncCost2WrtRx(fh1, fv1, rx, ry, rz, tx, ty, tz, rot_init_x, rot_init_y, rot_init_z, rot_init_w, lc0, lc1, lc2, lc3, epsilon);
                *H3 = (Matrix(4, 1) << Fkresidual_func_wrt_rx).finished();
            }
            if(H4)
            {
                Eigen::Matrix<double, 4, 1> Fkresidual_func_wrt_ry = sym::FkResidualFuncCost2WrtRy(fh1, fv1, rx, ry, rz, tx, ty, tz, rot_init_x, rot_init_y, rot_init_z, rot_init_w, lc0, lc1, lc2, lc3, epsilon);
                *H4 = (Matrix(4, 1) << Fkresidual_func_wrt_ry).finished();
            }
            if(H5)
            {
                Eigen::Matrix<double, 4, 1> Fkresidual_func_wrt_rz = sym::FkResidualFuncCost2WrtRz(fh1, fv1, rx, ry, rz, tx, ty, tz, rot_init_x, rot_init_y, rot_init_z, rot_init_w, lc0, lc1, lc2, lc3, epsilon);
                *H5 = (Matrix(4, 1) << Fkresidual_func_wrt_rz).finished();
            }
            if(H6)
            {
                Eigen::Matrix<double, 4, 1> Fkresidual_func_wrt_tx = sym::FkResidualFuncCost2WrtTx(fh1, fv1, rx, ry, rz, tx, ty, tz, rot_init_x, rot_init_y, rot_init_z, rot_init_w, lc0, lc1, lc2, lc3, epsilon);
                *H6 = (Matrix(4, 1) << Fkresidual_func_wrt_tx).finished();
            }
            if(H7)
            {
                Eigen::Matrix<double, 4, 1> Fkresidual_func_wrt_ty = sym::FkResidualFuncCost2WrtTy(fh1, fv1, rx, ry, rz, tx, ty, tz, rot_init_x, rot_init_y, rot_init_z, rot_init_w, lc0, lc1, lc2, lc3, epsilon);
                *H7 = (Matrix(4, 1) << Fkresidual_func_wrt_ty).finished();
            }
            if(H8)
            {
                Eigen::Matrix<double, 4, 1> Fkresidual_func_wrt_tz = sym::FkResidualFuncCost2WrtTz(fh1, fv1, rx, ry, rz, tx, ty, tz, rot_init_x, rot_init_y, rot_init_z, rot_init_w, lc0, lc1, lc2, lc3, epsilon);
                *H8 = (Matrix(4, 1) << Fkresidual_func_wrt_tz).finished();
            }

            return (Vector(4) << Fkresidual_func).finished();
        }
    };


    class FK_factor_graoh_cost3 : public NoiseModelFactorN<double, double, double, double, double, double, double, double>
    {

    private:
        double lc0;
        double lc1; 
        double lc2; 
        double lc3;         
        double rot_init_x; 
        double rot_init_y; 
        double rot_init_z; 
        double rot_init_w; 
        double epsilon = 0.0;

    public:
        // Constructor
        FK_factor_graoh_cost3(Key key1, Key key2, Key key3, Key key4, Key key5, Key key6, Key key7, Key key8, double lc0_, double lc1_, double lc2_, double lc3_, double rot_init_x_, double rot_init_y_, double rot_init_z_, double rot_init_w_, const SharedNoiseModel &model) 
        : NoiseModelFactorN<double, double, double, double, double, double, double, double>(model, key1, key2, key3, key4, key5, key6, key7, key8), lc0(lc0_), lc1(lc1_), lc2(lc2_), lc3(lc3_), rot_init_x(rot_init_x_), rot_init_y(rot_init_y_), rot_init_z(rot_init_z_), rot_init_w(rot_init_w_) {}

        // Evaluate the error
        Vector evaluateError(const double &fh1, const double &fv1, const double &rx, const double &ry, const double &rz, const double &tx, const double &ty, const double &tz,
                             OptionalMatrixType H1,
                             OptionalMatrixType H2,
                             OptionalMatrixType H3,
                             OptionalMatrixType H4,
                             OptionalMatrixType H5,
                             OptionalMatrixType H6,
                             OptionalMatrixType H7,
                             OptionalMatrixType H8) const override
        {   
            Eigen::Matrix<double, 4, 1> Fkresidual_func = sym::FkResidualFuncCost3(fh1, fv1, rx, ry, rz, tx, ty, tz, rot_init_x, rot_init_y, rot_init_z, rot_init_w, lc0, lc1, lc2, lc3, epsilon);
            if(H1)
            {
                Eigen::Matrix<double, 4, 1> Fkresidual_func_wrt_fh1 = sym::FkResidualFuncCost3WrtFh1(fh1, fv1, rx, ry, rz, tx, ty, tz, rot_init_x, rot_init_y, rot_init_z, rot_init_w, lc0, lc1, lc2, lc3, epsilon);
                *H1 = (Matrix(4, 1) << Fkresidual_func_wrt_fh1).finished();
            }
            if(H2)
            {
                Eigen::Matrix<double, 4, 1> Fkresidual_func_wrt_fv1 = sym::FkResidualFuncCost3WrtFv1(fh1, fv1, rx, ry, rz, tx, ty, tz, rot_init_x, rot_init_y, rot_init_z, rot_init_w, lc0, lc1, lc2, lc3, epsilon);
                *H2 = (Matrix(4, 1) << Fkresidual_func_wrt_fv1).finished();
            }
            if(H3)
            {
                Eigen::Matrix<double, 4, 1> Fkresidual_func_wrt_rx = sym::FkResidualFuncCost3WrtRx(fh1, fv1, rx, ry, rz, tx, ty, tz, rot_init_x, rot_init_y, rot_init_z, rot_init_w, lc0, lc1, lc2, lc3, epsilon);
                *H3 = (Matrix(4, 1) << Fkresidual_func_wrt_rx).finished();
            }
            if(H4)
            {
                Eigen::Matrix<double, 4, 1> Fkresidual_func_wrt_ry = sym::FkResidualFuncCost3WrtRy(fh1, fv1, rx, ry, rz, tx, ty, tz, rot_init_x, rot_init_y, rot_init_z, rot_init_w, lc0, lc1, lc2, lc3, epsilon);
                *H4 = (Matrix(4, 1) << Fkresidual_func_wrt_ry).finished();
            }
            if(H5)
            {
                Eigen::Matrix<double, 4, 1> Fkresidual_func_wrt_rz = sym::FkResidualFuncCost3WrtRz(fh1, fv1, rx, ry, rz, tx, ty, tz, rot_init_x, rot_init_y, rot_init_z, rot_init_w, lc0, lc1, lc2, lc3, epsilon);
                *H5 = (Matrix(4, 1) << Fkresidual_func_wrt_rz).finished();
            }
            if(H6)
            {
                Eigen::Matrix<double, 4, 1> Fkresidual_func_wrt_tx = sym::FkResidualFuncCost3WrtTx(fh1, fv1, rx, ry, rz, tx, ty, tz, rot_init_x, rot_init_y, rot_init_z, rot_init_w, lc0, lc1, lc2, lc3, epsilon);
                *H6 = (Matrix(4, 1) << Fkresidual_func_wrt_tx).finished();
            }
            if(H7)
            {
                Eigen::Matrix<double, 4, 1> Fkresidual_func_wrt_ty = sym::FkResidualFuncCost3WrtTy(fh1, fv1, rx, ry, rz, tx, ty, tz, rot_init_x, rot_init_y, rot_init_z, rot_init_w, lc0, lc1, lc2, lc3, epsilon);
                *H7 = (Matrix(4, 1) << Fkresidual_func_wrt_ty).finished();
            }
            if(H8)
            {
                Eigen::Matrix<double, 4, 1> Fkresidual_func_wrt_tz = sym::FkResidualFuncCost3WrtTz(fh1, fv1, rx, ry, rz, tx, ty, tz, rot_init_x, rot_init_y, rot_init_z, rot_init_w, lc0, lc1, lc2, lc3, epsilon);
                *H8 = (Matrix(4, 1) << Fkresidual_func_wrt_tz).finished();
            }

            return (Vector(4) << Fkresidual_func).finished();
        }
    };

}
