#include "nuslam/nuslam.hpp"

using namespace Eigen;

EKF::EKF()
{
  j = 1;
}

void EKF::circle(std::vector<std::vector<float>> &x_in, std::vector<std::vector<float>> &y_in)
{ 
    for (unsigned int k =0;k<x_in.size(); k++)
    {
        VectorXf  A;
        float x_mean;
        float y_mean;
        float z_mean;

        std::vector<float> current_x_cluster = x_in[k];
        std::vector<float> current_y_cluster = y_in[k];

        std::vector<float> current_z_cluster;
        int n = current_x_cluster.size();
        
        float sum_x = std::accumulate(std::begin(current_x_cluster), std::end(current_x_cluster), 0.0);
        x_mean =  sum_x / current_x_cluster.size();
        
        float sum_y = std::accumulate(std::begin(current_y_cluster), std::end(current_y_cluster), 0.0);
        y_mean =  sum_y / current_y_cluster.size();
        
        for(unsigned int l = 0;  l< (current_x_cluster.size()); l++ )
        {
            current_x_cluster[l] = current_x_cluster[l] - x_mean;   
            current_y_cluster[l] = current_y_cluster[l] - y_mean;   
            float z_temp = current_x_cluster[l] * current_x_cluster[l] + current_y_cluster[l] * current_y_cluster[l] ;
            current_z_cluster.push_back(z_temp); 
        }
        
        float sum_z = std::accumulate(std::begin(current_z_cluster), std::end(current_z_cluster), 0.0);
        z_mean =  sum_z / n;

        MatrixXf Z(n,4);
        Map<VectorXf> temp2((&current_x_cluster[0]),current_x_cluster.size());
        Map<VectorXf> temp1((&current_z_cluster[0]),current_z_cluster.size());
        Map<VectorXf> temp3((&current_y_cluster[0]),current_y_cluster.size());
        VectorXf temp4(n,1);
        temp4 << MatrixXf::Ones(n,1);

        Z << temp1, temp2, temp3, temp4; 

        Matrix<float,Dynamic,Dynamic> M;

        M =  (Z.transpose() * Z )/n;

        Matrix<float,4,4> H;

        H << 8.0*z_mean,0.0,0.0,2,
             0.0,1.0,0.0,0.0,
             0.0,0.0,1.0,0.0,
             2.0,0.0,0.0,0.0;

        Matrix<float,4,4> H_inv;

        H_inv << 0.0,0.0,0.0,0.5,
                 0.0,1.0,0.0,0.0,
                 0.0,0.0,1.0,0.0,
                 0.5,0.0,0.0,(-2.0*z_mean);

        Matrix<float,Dynamic,Dynamic> U;
        Matrix<float,Dynamic,Dynamic> V;
        std::vector<float> values;

        JacobiSVD<MatrixXf> svd(Z, ComputeThinU | ComputeThinV);
        U = svd.matrixU();
        V = svd.matrixV();

        VectorXf temp_(4,1);
        MatrixXf ma(4,4);
        for(unsigned int i =0; i<svd.singularValues().size(); i++)
        {
        values.push_back(svd.singularValues()[i]);
        }
        temp_ << svd.singularValues();
        std::sort(values.begin(),values.end());

        ma = temp_.asDiagonal();

 
        if(values[0] < pow(10,-12))
        {
          A = V.col(3);
        }

        else 

        {
        
        MatrixXf Y;
        MatrixXf Q;

        Y = V * ma * V.adjoint();
        Q = Y * H_inv * Y;

        SelfAdjointEigenSolver<MatrixXf> es(Q);

        std::vector<float> temp_a; 

        for(unsigned int i =0; i<es.eigenvalues().size(); i++)
        {
          temp_a.push_back(abs(es.eigenvalues()[i]));
        }

        int minElementIndex = std::min_element(temp_a.begin(),temp_a.end()) - temp_a.begin();
        VectorXf A_star;
        A_star = es.eigenvectors().col(minElementIndex);

        A = Y.colPivHouseholderQr().solve(A_star);

        }
        
        float small_a = A(1)/(2*A(0));
        float small_b = -1.0 * A(2)/(2.0*A(0));

        float r_squared =(A(1) * A(1) + A(2) * A(2) - 4*A(0) * A(3)) /(4*A(0)*A(0)) ;
        float r = sqrt(r_squared);

        float located_x = small_a + x_mean;
        float located_y = small_b + y_mean;

  
        x_data.push_back(located_x);
        y_data.push_back(located_y);
        r_data.push_back(r);
        
}
}