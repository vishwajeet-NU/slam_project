/// \file
/// \brief This file defines the methods that have been 
/// initialized in the header file called nuslam.hpp.

#include "nuslam/nuslam.hpp"

#include"rigid2d/rigid2d.hpp"

using namespace Eigen;

EKF::EKF()
{
  j =3;
}

void EKF::initialize_matrices(int landmark_no, double sigma_r,double sigma_theta,double sigma_landmark, double r_param)
{
  
    int m_land = landmark_no;     
    int n_land = m_land * 2;
    int total = n_land + 3;

    mu_t_bar.resize(total,1);
    mu_t_bar.fill(0.0);
    mu_t_posterior.resize(total,1);
    w_t.resize(3,1);
    v_t.resize(2,1);
    g_t.resize(total,total);
    g_t.fill(0.0);
    g_t = g_t + Eigen::MatrixXd::Identity(total,total); 
    
    q_t.resize(total,total);
    q_t.fill(0.0);
    q_t.block(0,0,3,3) << sigma_r*sigma_r,0,0,0,sigma_theta*sigma_theta,0,0,0,sigma_landmark*sigma_landmark;
    
    pose_var_prior.resize(total,total);
    pose_var_bar.resize(total,total);
    pose_var_posterior.resize(total,total);
    pose_var_prior.fill(0.0);
    Eigen::MatrixXd temp_covar(n_land,n_land);
    Eigen::VectorXd temp_temp(n_land);
    temp_temp.fill(10000);
    temp_covar.fill(0.0);
    temp_covar = temp_temp.asDiagonal(); 
    pose_var_prior.bottomRightCorner(n_land,n_land) = temp_covar;
    z_t.resize(2,1);
    small_h.resize(2,1);
    large_h.resize(2,total);
    large_h.fill(0.0);
    k_gain.resize(total,2);
    R.resize(2,2);
    R << r_param,0,0,r_param; //parametrize this later

}


std::vector<std::vector<std::vector<float>>> EKF::circle_or_not_circle(std::vector<std::vector<float>> &x_in, std::vector<std::vector<float>> &y_in)
{


  float first_x,last_x,first_y,last_y;
  
  std::vector<std::vector<std::vector<float>>> selected_values;

  std::vector<std::vector<float>> selected_x;
  std::vector<std::vector<float>> selected_y;
  
 
  for (unsigned int k =0;k<x_in.size(); k++)
  {
    std::vector<float> angle_list;
    std::vector<float> current_x_cluster = x_in[k];
    std::vector<float> current_y_cluster = y_in[k];
    
    first_x = *(current_x_cluster.begin());
    last_x  = *(current_x_cluster.end()-1);

    first_y = *(current_y_cluster.begin());
    last_y  = *(current_y_cluster.end()-1);

    for( unsigned int l = 1;  l< (current_x_cluster.size()-1); l++ )
    {

      float y_delta_first =  (current_y_cluster[l] - first_y);
      float x_delta_first = (current_x_cluster[l] - first_x);

      float y_delta_last = (current_y_cluster[l] - last_y);
      float x_delta_last = (current_x_cluster[l] - last_x);
      

      float first_ang = atan2(x_delta_first,y_delta_first);
      float last_ang = atan2(x_delta_last,y_delta_last);

      angle_list.push_back(first_ang-last_ang);      
      
    }

    double sum = std::accumulate(std::begin(angle_list), std::end(angle_list), 0.0);
    double m =  sum / angle_list.size();
    double accum = 0.0;
    std::for_each (std::begin(angle_list), std::end(angle_list), [&](const double d) 
    {
    accum += (d - m) * (d - m);
    });
    double stdev = sqrt(accum / (angle_list.size()));

    if(m<0.0)
    {
      m = m + 2.0*rigid2d::PI;
    }
//    std::cout<<"m out ="<<m<<"\n";

    if((m>=(rigid2d::PI)/2.0) && (m<= (3.0* rigid2d::PI)/4.0)  && stdev <0.15)
    {

      selected_x.push_back(x_in[k]);
      selected_y.push_back(y_in[k]);
    }

    angle_list.clear();
  }
  
  selected_values.push_back(selected_x);
  selected_values.push_back(selected_y);

return selected_values;

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
        
        float small_a = -1.0* A(1)/(2*A(0));
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

 std::mt19937 & EKF::get_random()
 {
     static std::random_device rd{}; 
     static std::mt19937 mt{rd()};
     return mt;
 }

void EKF::ekf_predict(rigid2d::Twist2D body_v, double co_var_w, int total)
{
    theta_start = mu_t_bar.coeff(0,0);
    x_start = mu_t_bar.coeff(1,0);
    y_start = mu_t_bar.coeff(2,0);

    // std::cout<<"theta in for predict "<< theta_start<<"\n";
    if(body_v.w == 0)
    {   
        x_start = x_start + body_v.v_x*cos(theta_start);
        y_start = y_start + body_v.v_x*sin(theta_start);
    }
    else
    {
        x_start = x_start - body_v.v_x*sin(theta_start)/body_v.w + body_v.v_x*sin(body_v.w + theta_start)/body_v.w;
        y_start = y_start + body_v.v_x*cos(theta_start)/body_v.w - body_v.v_x*cos(body_v.w + theta_start)/body_v.w;
        theta_start = theta_start + body_v.w;

    }

    // std::cout<<"theta out from predict "<< theta_start<<"\n";

    mu_t_bar.row(0) << theta_start;
    mu_t_bar.row(1) << x_start;
    mu_t_bar.row(2) << y_start;
    std::normal_distribution<double> d(0.0, co_var_w);
    std::vector<double> random_nos_w;
    for( int i = 0; i < 3 ;i++)
    {
     random_nos_w.push_back(d(get_random()));   
    }

    Eigen::Map<Eigen::VectorXd> temp_rand_w((&random_nos_w[0]),random_nos_w.size());
    w_t << temp_rand_w;
    // std::cout<<"wt nosise = \n"<<w_t<<"  
    mu_t_bar.row(0) = mu_t_bar.row(0) + w_t.row(0);
    mu_t_bar.row(1) = mu_t_bar.row(1) + w_t.row(1);
    mu_t_bar.row(2) = mu_t_bar.row(2) + w_t.row(2); 
    int mat_len = mu_t_bar.rows();
    Eigen::VectorXd temp_for_gt(mat_len,1);
    temp_for_gt.fill(0.0);
    if(body_v.w == 0)
    {   
        temp_for_gt.row(1) <<  -body_v.v_x * sin(theta_start);
        temp_for_gt.row(2) <<  body_v.v_x * cos(theta_start);
    }
    else
    {
        temp_for_gt.row(1)<< (body_v.v_x * (-cos(theta_start) + cos(theta_start + body_v.w)))/body_v.w;
        temp_for_gt.row(2)<< (body_v.v_x * (-sin(theta_start) + sin(theta_start + body_v.w)))/body_v.w;
    }
    g_t.col(0) << temp_for_gt;
    pose_var_bar = g_t*pose_var_prior*g_t.transpose() + q_t; 
    // std::cout<<"state post prdedict \n"<< mu_t_bar <<"\n";
    // std::cout<<"variance post predict \n"<< pose_var_bar <<"\n";
}
double EKF::wrap_angles(double incoming_angle)
{
    incoming_angle = atan2(sin(incoming_angle),cos(incoming_angle));

    return incoming_angle;
}

void EKF::ekf_update(int m_land, double co_var_v, std::vector<float> x_center, std::vector<float> y_center, int total, double threshold, double upper_threshold)
{
  int number_of_landmarks = (mu_t_bar.rows() - 3)/2;
  for( int i = 0; i<number_of_landmarks; i++ )
   {
      // int id;

      // id = which_landmark(threshold,upper_threshold,x_center[i],y_center[i]);
      // std::cout<<"id coming in"<<id<<"\n";
      int id = 2*(i+1)+1;
      int mat_len = mu_t_bar.rows();
      double y_del = (mu_t_bar.coeff(id+1,0) -mu_t_bar.coeff(2,0)); 
      double x_del = (mu_t_bar.coeff(id,0) - mu_t_bar.coeff(1,0));
      double known_range = sqrt(x_del * x_del + y_del * y_del); 
      double known_bearing =  ( wrap_angles(atan2(y_del,x_del)))- wrap_angles(mu_t_bar.coeff(0,0));
                 
      double range = sqrt((x_center[i] * x_center[i]) + (y_center[i] * y_center[i]));
      double bearing = wrap_angles(atan2(y_center[i],x_center[i])) ;

      small_h.row(0) << known_range;
      small_h.row(1) << known_bearing;

      std::vector<double> random_nos_v;
      std::normal_distribution<double> e(0.0, co_var_v);
      for( int i = 1; i < 3 ;i++)
      {
        random_nos_v.push_back(e(get_random()));   
      }
      Eigen::Map<Eigen::VectorXd> temp_rand_v((&random_nos_v[0]),random_nos_v.size());
      v_t << temp_rand_v;
      double x_delta = mu_t_bar.coeff(2*(i+1)+1,0) - mu_t_bar.coeff(1,0) ;
      double y_delta = mu_t_bar.coeff(2*(i+1)+2,0) - mu_t_bar.coeff(2,0);
      double delta = x_delta * x_delta + y_delta * y_delta;
      large_h.col(0) << 0,-1.0;
      large_h.col(1) << (-x_delta/sqrt(delta)), (y_delta/(delta));
      large_h.col(2) << (-y_delta/sqrt(delta)), -(x_delta/(delta));
    
      large_h.col(id) << (x_delta/sqrt(delta)), (-y_delta/delta);
      large_h.col(id+1) << (y_delta/sqrt(delta)),(x_delta/delta);
      z_t << range,bearing;
      z_t = z_t + v_t;
      Eigen::MatrixXd diff(2,1);
      diff = z_t - small_h;
      diff(1) = wrap_angles(diff(1));
      k_gain = pose_var_bar * large_h.transpose() * (large_h * pose_var_bar * large_h.transpose() + R ).inverse();
       
      mu_t_posterior = mu_t_bar + k_gain* diff;

      Eigen::MatrixXd identity_temp;
      identity_temp.setIdentity(mat_len,mat_len);
      pose_var_posterior = (identity_temp - k_gain*large_h) * pose_var_bar;
      //  std::cout<<"pose_var_posterior = \n"<<pose_var_posterior<<"\n \n"; 
      mu_t_bar = mu_t_posterior;
      pose_var_bar = pose_var_posterior;
                
    }   
}


void EKF::resize_matrices(int current_size, float x_reading, float y_reading)
{
  Eigen::Map<MatrixXd> temp(mu_t_bar.data(), current_size+2,1); 
  mu_t_bar = temp;
  mu_t_bar.row(current_size) << x_reading;
  mu_t_bar.row(current_size+1) << y_reading;

  Eigen::Map<MatrixXd> temp1(mu_t_posterior.data(), current_size+2,1); 
  mu_t_posterior = temp1;


  std::cout<<"mu t post size"<<mu_t_posterior.size()<<"\n";

  Eigen::Map<MatrixXd> temp2(g_t.data(), current_size+2,current_size+2); 
  g_t = temp2;
  g_t.fill(0.0);
  g_t = g_t + Eigen::MatrixXd::Identity(current_size+2,current_size+2); 

  std::cout<<"gt"<<g_t.size()<<"\n";

  Eigen::Map<MatrixXd> temp3(q_t.data(), current_size+2,current_size+2); 
  q_t = temp3;
  q_t.row(current_size).fill(0.0);
  q_t.row(current_size+1).fill(0.0);
  q_t.col(current_size).fill(0.0);
  q_t.col(current_size+1).fill(0.0);
  
  std::cout<<"Qt"<<q_t.size()<<"\n";

  
  Eigen::Map<MatrixXd> temp4(pose_var_prior.data(), current_size+2,current_size+2); 
  pose_var_prior = temp4;


  std::cout<<"pose_var_prior"<<pose_var_prior.size()<<"\n";

  Eigen::Map<MatrixXd> temp5(pose_var_bar.data(), current_size+2,current_size+2); 
  pose_var_bar = temp5;


  std::cout<<"pose_var_bar"<<pose_var_bar.size()<<"\n";

  Eigen::Map<MatrixXd> temp6(pose_var_posterior.data(), current_size+2,current_size+2); 
  pose_var_posterior = temp6;


  std::cout<<"pose_var_posterior"<<pose_var_posterior.size()<<"\n";

  Eigen::Map<MatrixXd> temp7(large_h.data(), 2,current_size+2); 
  large_h = temp7;
  large_h.col(current_size).fill(0.0);
  large_h.col(current_size+1).fill(0.0);


  std::cout<<"large_h"<<large_h.size()<<"\n";
  
  Eigen::Map<MatrixXd> temp8(k_gain.data(),current_size+2,2); 
  k_gain = temp8;  

  std::cout<<"k_gain"<<k_gain.size()<<"\n";
}


int EKF::which_landmark(double threshold, double upper_threshold , float x_reading, float y_reading)
{
  int index =0;
  double dist = 0.0;
  double curr = 100.0;
  int closest_index= 0;
  int current_size = mu_t_bar.rows();
  std::vector<float> temp;
  // std::cout<<"mu_t_bar \n"<<mu_t_bar<<"\n \n";
     
  for(int i = 3; i<current_size; i+=2)
  {
   std::cout<<"inside for loop i ="<<i<<"\t";


    rigid2d::Vector2D robot(mu_t_bar.coeff(1,0),mu_t_bar.coeff(2,0));
    rigid2d::Vector2D sensor(x_reading, y_reading);
    rigid2d::Transform2D Tsb(robot,mu_t_bar.coeff(0.0));
    rigid2d::Transform2D Tlb(sensor);

    rigid2d::Transform2D Tsl = operator*(Tsb,Tlb );
      
    dist = sqrt((Tsl.m3- mu_t_bar.coeff(i,0)) * (Tsl.m3 - mu_t_bar.coeff(i,0)) + (Tsl.m6 - mu_t_bar.coeff(i+1,0))*(Tsl.m6 - mu_t_bar.coeff(i+1,0)));
    if(dist<curr)
    {
      closest_index = i;
      curr = dist;
    }
    if(dist < threshold )
    { 
      // found corresponding landmark inside state vector 
      index = i;
      std::cout<<"foud landmark, assigned index "<<index<<"\n";

      decision = true;
    }
  }

  if(!decision && dist < upper_threshold)
  {
    // no corresponding landmark found
    // add this reading as landmark to state
     resize_matrices(current_size,x_reading,y_reading);
     index = current_size;
     std::cout<<"no landmark found, resizing index = "<<index<<"\n";
  }
  else
  {
    index = closest_index;
    std::cout<<"too big, asigning new index "<<index<<"\n";

  }
  
  std::cout<<"index = "<<index<<"\n";  
  decision = false;

  return index;
}
