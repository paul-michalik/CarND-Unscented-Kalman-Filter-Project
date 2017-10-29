#include "stdafx.h"
#include "CppUnitTest.h"
#include <tools.h>
#include <ukf.cpp>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace Microsoft {
    namespace VisualStudio {
        namespace CppUnitTestFramework {
            template<> inline std::wstring ToString<Eigen::MatrixXd>(const MatrixXd& m)
            {
                std::stringstream out;
                out << m;
                return ToString(out.str());
            }
        }
    }
}

template<typename DerivedA, typename DerivedB>
bool all_close(const Eigen::DenseBase<DerivedA>& a,
    const Eigen::DenseBase<DerivedB>& b,
    const typename DerivedA::RealScalar& rtol
    = Eigen::NumTraits<typename DerivedA::RealScalar>::dummy_precision(),
    const typename DerivedA::RealScalar& atol
    = Eigen::NumTraits<typename DerivedA::RealScalar>::epsilon())
{
    return ((a.derived() - b.derived()).array().abs()
        <= (atol + rtol * b.derived().array().abs())).all();
}

namespace TestsCarNDUKFProject
{		
	TEST_CLASS(TestClass_UKF)
	{
        void arrange(UKF& sut) const
        {
            sut.x_ <<
                5.7441,
                1.3800,
                2.2049,
                0.5015,
                0.3528;

            sut.P_ <<
                0.0043, -0.0013, 0.0030, -0.0022, -0.0020,
                -0.0013, 0.0077, 0.0011, 0.0071, 0.0060,
                0.0030, 0.0011, 0.0054, 0.0007, 0.0008,
                -0.0022, 0.0071, 0.0007, 0.0098, 0.0100,
                -0.0020, 0.0060, 0.0008, 0.0100, 0.0123;
        }
	public:
		TEST_METHOD(CreateAugmentedSigmaPoints)
		{
            UKF sut;
            arrange(sut);

            sut.CreateAugmentedSigmaPoints();

            MatrixXd Xsig_aug_exp{sut.n_aug_, 2 * sut.n_aug_ + 1};
            Xsig_aug_exp <<
                5.7441, 5.85768, 5.7441, 5.7441, 5.7441, 5.7441, 5.7441, 5.7441, 5.63052, 5.7441, 5.7441, 5.7441, 5.7441, 5.7441, 5.7441,
                1.38, 1.34566, 1.52806, 1.38, 1.38, 1.38, 1.38, 1.38, 1.41434, 1.23194, 1.38, 1.38, 1.38, 1.38, 1.38,
                2.2049, 2.28414, 2.24557, 2.29582, 2.2049, 2.2049, 2.2049, 2.2049, 2.12566, 2.16423, 2.11398, 2.2049, 2.2049, 2.2049, 2.2049,
                0.5015, 0.44339, 0.631886, 0.516923, 0.595227, 0.5015, 0.5015, 0.5015, 0.55961, 0.371114, 0.486077, 0.407773, 0.5015, 0.5015, 0.5015,
                0.3528, 0.299973, 0.462123, 0.376339, 0.48417, 0.418721, 0.3528, 0.3528, 0.405627, 0.243477, 0.329261, 0.22143, 0.286879, 0.3528, 0.3528,
                0, 0, 0, 0, 0, 0, 0.34641, 0, 0, 0, 0, 0, 0, -0.34641, 0,
                0, 0, 0, 0, 0, 0, 0, 0.34641, 0, 0, 0, 0, 0, 0, -0.34641;

            std::stringstream out;
            out << "expected Xsig_aug:" << std::endl 
                << Xsig_aug_exp << std::endl 
                << "actual Xsig_aug:" << std::endl 
                << sut.Xsig_aug_ << std::endl;
            Logger::WriteMessage(out.str().c_str());

            Assert::IsTrue(Xsig_aug_exp.isApprox(sut.Xsig_aug_, 1.e-6), L"Xsig_aug");
		}

        TEST_METHOD(PredictAugmentedSigmaPoints)
        {
            UKF sut;
            arrange(sut);

            sut.CreateAugmentedSigmaPoints();
            sut.PredictAugmentedSigmaPoints(0.1);

            MatrixXd Xsig_pred_exp{sut.Xsig_pred_.rows(), sut.Xsig_pred_.cols()};
            Xsig_pred_exp <<
                5.93553, 6.06251, 5.92217, 5.9415, 5.92361, 5.93516, 5.93705, 5.93553, 5.80832, 5.94481, 5.92935, 5.94553, 5.93589, 5.93401, 5.93553,
                1.48939, 1.44673, 1.66484, 1.49719, 1.508, 1.49001, 1.49022, 1.48939, 1.5308, 1.31287, 1.48182, 1.46967, 1.48876, 1.48855, 1.48939,
                2.2049, 2.28414, 2.24557, 2.29582, 2.2049, 2.2049, 2.23954, 2.2049, 2.12566, 2.16423, 2.11398, 2.2049, 2.2049, 2.17026, 2.2049,
                0.53678, 0.473387, 0.678098, 0.554557, 0.643644, 0.543372, 0.53678, 0.538512, 0.600173, 0.395462, 0.519003, 0.429916, 0.530188, 0.53678, 0.535048,
                0.3528, 0.299973, 0.462123, 0.376339, 0.48417, 0.418721, 0.3528, 0.387441, 0.405627, 0.243477, 0.329261, 0.22143, 0.286879, 0.3528, 0.318159;

            std::stringstream out;
            out << "expected Xsig_pred:" << std::endl
                << Xsig_pred_exp << std::endl
                << "actual Xsig_pred:" << std::endl
                << sut.Xsig_pred_ << std::endl;
            Logger::WriteMessage(out.str().c_str());

            Assert::IsTrue(Xsig_pred_exp.isApprox(sut.Xsig_pred_, 1.e-6), L"Xsig_pred");
        }

        TEST_METHOD(PredictMeanAndCovariance)
        {
            UKF sut;
            arrange(sut);

            MatrixXd Xsig_pred_exp{sut.n_x_, 2 * sut.n_aug_ + 1};
            Xsig_pred_exp <<
                5.9374, 6.0640, 5.925, 5.9436, 5.9266, 5.9374, 5.9389, 5.9374, 5.8106, 5.9457, 5.9310, 5.9465, 5.9374, 5.9359, 5.93744,
                1.48, 1.4436, 1.660, 1.4934, 1.5036, 1.48, 1.4868, 1.48, 1.5271, 1.3104, 1.4787, 1.4674, 1.48, 1.4851, 1.486,
                2.204, 2.2841, 2.2455, 2.2958, 2.204, 2.204, 2.2395, 2.204, 2.1256, 2.1642, 2.1139, 2.204, 2.204, 2.1702, 2.2049,
                0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337, 0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188, 0.5367, 0.535048,
                0.352, 0.29997, 0.46212, 0.37633, 0.4841, 0.41872, 0.352, 0.38744, 0.40562, 0.24347, 0.32926, 0.2214, 0.28687, 0.352, 0.318159;

            VectorXd x_exp{sut.n_x_};
            x_exp <<
                5.93637,
                1.49035,
                2.20528,
                0.536853,
                0.353577;

            MatrixXd P_exp{sut.n_x_, sut.n_x_};
            P_exp <<
                0.00543425, -0.0024053, 0.00341576, -0.00348196, -0.00299378,
                -0.0024053, 0.010845, 0.0014923, 0.00980182, 0.00791091,
                0.00341576, 0.0014923, 0.00580129, 0.000778632, 0.000792973,
                -0.00348196, 0.00980182, 0.000778632, 0.0119238, 0.0112491,
                -0.00299378, 0.00791091, 0.000792973, 0.0112491, 0.0126972;

            
            VectorXd weights_exp{2 * sut.n_aug_ + 1};
            {
                double weight_0 = sut.lambda_ / (sut.lambda_ + sut.n_aug_);
                weights_exp(0) = weight_0;
                for (int i = 1; i < 2 * sut.n_aug_ + 1; i++) {  //2n+1 weights
                    double weight = 0.5 / (sut.n_aug_ + sut.lambda_);
                    weights_exp(i) = weight;
                }
            }

            sut.Xsig_pred_ = Xsig_pred_exp;
            sut.PredictMeanAndCovariance();

            std::stringstream out;
            out << "expected weights:" << std::endl
                << weights_exp << std::endl
                << "actual weights:" << std::endl
                << sut.weights_ << std::endl
                << "expected Xsig_pred:" << std::endl
                << Xsig_pred_exp << std::endl
                << "actual Xsig_pred:" << std::endl
                << sut.Xsig_pred_ << std::endl
                << "expected x:" << std::endl
                << x_exp << std::endl
                << "actual x:" << std::endl
                << sut.x_ << std::endl
                << "expected P:" << std::endl
                << P_exp << std::endl
                << "actual P:" << std::endl
                << sut.P_ << std::endl;

            Logger::WriteMessage(out.str().c_str());

            Assert::IsTrue(weights_exp.isApprox(sut.weights_, 1.e-6), L"weights");
            Assert::IsTrue(Xsig_pred_exp.isApprox(sut.Xsig_pred_, 1.e-6), L"Xsig_pred");
            Assert::IsTrue(x_exp.isApprox(sut.x_, 1.e-6), L"state vectors");
            Assert::IsTrue(P_exp.isApprox(sut.P_, 1.e-5), L"covariance matrix");
        }
	};
}