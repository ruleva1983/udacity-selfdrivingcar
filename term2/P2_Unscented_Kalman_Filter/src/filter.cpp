#include "filter.h"
#include "constants.h"


BayesianFilter::BayesianFilter(){}
BayesianFilter::~BayesianFilter(){}

UFK::UFK(){}
UFK::~UFK(){}

void UFK::apply_kalman(Eigen::VectorXd& z_meas, Eigen::VectorXd& X_pred, Eigen::MatrixXd& P_pred,
                       Eigen::VectorXd& X_meas, Eigen::MatrixXd& P_meas, Eigen::VectorXd& X_, Eigen::MatrixXd& P_)
{
        Eigen::MatrixXd Sigma_pred = Generator.Points_pred;
        Eigen::MatrixXd Sigma_meas = Generator.Points_meas;
        Eigen::VectorXd weights = Generator.weights;
        
        int n_z = Sigma_meas.rows(); 
        int n_x = Sigma_pred.rows(); 
        int nb_sigma = Sigma_pred.cols();
    
        Eigen::MatrixXd T = Eigen::MatrixXd::Zero(n_x, n_z);
        
        for (int i = 0; i < nb_sigma; i++) {
            Eigen::VectorXd z_diff = Sigma_meas.col(i) - X_meas;
            if (n_z == 3){
                while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
                while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
            }
            
            Eigen::VectorXd x_diff = Sigma_pred.col(i) - X_pred;
            while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
            while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

            T = T + weights(i) * x_diff * z_diff.transpose();
            }
        
        Eigen::MatrixXd K = T * P_meas.inverse();
        Eigen::VectorXd z_diff = z_meas - X_meas;
        if (n_z == 3){
            while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
            while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
        }
        X_ = X_pred + K * z_diff;
        P_ = P_pred - K*P_meas*K.transpose();
    
}



