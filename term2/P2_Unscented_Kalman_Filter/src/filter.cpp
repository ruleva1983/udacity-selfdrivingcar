#include "filter.h"

BayesianFilter::BayesianFilter(){}
BayesianFilter::~BayesianFilter(){}

UFK::UFK(){}
UFK::~UFK(){}


void UFK::apply_kalman(Eigen::VectorXd& z_meas, Eigen::VectorXd& X_pred, Eigen::MatrixXd& P_pred,
                       Eigen::VectorXd& X_meas, Eigen::MatrixXd& P_meas, Eigen::VectorXd& X_, Eigen::MatrixXd& P_)
{
        Eigen::MatrixXd Sigma_pred = Generator.Points_pred;
        Eigen::MatrixXd Sigma_meas = Generator.Points_meas;
        
        int n_z = Sigma_meas.rows(); // should be 3 for radar and 2 for lidar
        int n_x = Sigma_pred.rows(); // Should be 5 equal to the number of state variables
        int nb_sigma = Sigma_pred.cols();
        double lambda = -4;
    
        Eigen::MatrixXd T = Eigen::MatrixXd::Zero(n_x, n_z);
        
        for (int i = 0; i < nb_sigma; i++) {
            double wi;
            if (i == 0)
                wi = lambda/(lambda+7);
            else
                wi =  0.5/(7+lambda);
            
            Eigen::VectorXd z_diff = Sigma_meas.col(i) - X_meas;
            
            if (n_z == 3)
            {
                while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
                while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
            }
            
            Eigen::VectorXd x_diff = Sigma_pred.col(i) - X_pred;
            
            if (n_z == 3)
            {
                while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
                while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
            }

            T = T + wi * x_diff * z_diff.transpose();
            }
        
        Eigen::MatrixXd K = T * P_meas.inverse();
        Eigen::VectorXd z_diff = z_meas - X_meas;
        if (n_z == 3)
            {
                while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
                while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
            }
        X_ = X_ + K * z_diff;
        P_ = P_ - K*P_meas*K.transpose();
        
}



