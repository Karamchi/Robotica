#include "PioneerRRTPlanner.h"

#include <angles/angles.h>
#include <queue>
#include <map>
#include <vector>
#include <random>
#include <queue>

typedef robmovil_planning::PioneerRRTPlanner::SpaceConfiguration SpaceConfiguration;

robmovil_planning::PioneerRRTPlanner::PioneerRRTPlanner(ros::NodeHandle& nh)
: RRTPlanner(nh, 0, 0)
{ 
  nh.param<double>("goal_bias", goal_bias_, 0.6);
  int it_tmp;
  nh.param<int>("max_iterations", it_tmp, 20000);
  max_iterations_ = it_tmp >= 0 ? it_tmp : 20000;
  nh.param<double>("linear_velocity_stepping", Vx_step_, 0.05);
  nh.param<double>("angular_velocity_stepping", Wz_step_, 0.025);
}

SpaceConfiguration robmovil_planning::PioneerRRTPlanner::defineStartConfig()
{
  /* Se toma la variable global de la pose incial y se la traduce a SpaceConfiguration */
  return SpaceConfiguration( { starting_pose_.getOrigin().getX(), starting_pose_.getOrigin().getY(), tf::getYaw(starting_pose_.getRotation()) } );
}

SpaceConfiguration robmovil_planning::PioneerRRTPlanner::defineGoalConfig()
{
  /* Se toma la variable global de la pose del goal y se la traduce a SpaceConfiguration */
  return SpaceConfiguration( { goal_pose_.getOrigin().getX(), goal_pose_.getOrigin().getY(), tf::getYaw(goal_pose_.getRotation()) } );
}

SpaceConfiguration robmovil_planning::PioneerRRTPlanner::generateRandomConfig()
{  
  
    /* COMPLETAR: Deben retornar una configuracion aleatoria dentro del espacio de busqueda.
     * 
     * ATENCION: - Tener encuenta el valor de la variable global goal_bias_ 
     *           - Pueden utilizar la funcion randBetween(a,b) para la generacion de numeros aleatorios 
     *           - Utilizar las funciones getOriginOfCell() y la informacion de la grilla para establecer el espacio de busqueda:
     *                grid_->info.width, grid_->info.height, grid_->info.resolution */  
    double x, y, theta;
    double infinitecoin = randBetween(0,1);
    if (infinitecoin < goal_bias_) {
      double modulo = randBetween(0, 1);
      double angulo = randBetween(0, 2*M_PI);
      x = modulo * cos(angulo) + goal_config_.get(0);
      y = modulo * sin(angulo) + goal_config_.get(1);
      theta = randBetween(goal_config_.get(2) - M_PI / 4, goal_config_.get(2) + M_PI / 4);

    } else {
      // multiplicar por grid_->info.resolution?
      double origin_x;
      double origin_y;
      getOriginOfCell(0, 0, origin_x, origin_y);

      x = randBetween(origin_x, grid_-> info.width*grid_->info.resolution + origin_x);
      y = randBetween(origin_y, grid_-> info.height*grid_->info.resolution + origin_y);
      theta = randBetween(-M_PI, M_PI);
    }
    
    SpaceConfiguration rand( {x, y, theta} );

    return rand;
}

double robmovil_planning::PioneerRRTPlanner::distancesBetween(const SpaceConfiguration& c1, const SpaceConfiguration& c2)
{
  /* COMPLETAR: Funcion auxiliar recomendada para evaluar la distancia entre configuraciones
   * 
   * ATENCION: Utilizar abs( angles::shortest_angular_distance(c1.get(2), c2.get(2)) )
   *           para medir la distancia entre las orientaciones */
  
  double dist_ori = abs( angles::shortest_angular_distance(c1.get(2), c2.get(2)) );
  double dist_euclid = sqrt(pow(c1.get(1) - c2.get(1), 2) + pow(c1.get(0) - c2.get(0), 2));
  
  return dist_ori * 0.5 + dist_euclid * 0.25;
}

SpaceConfiguration robmovil_planning::PioneerRRTPlanner::nearest()
{
  /* COMPLETAR: Retornar configuracion mas cercana a la aleatoria (rand_config_). DEBE TENER HIJOS LIBRES
   * 
   * ATENCION: - Deberan recorrer la variable global graph_ la cual contiene los nodos del arbol
   *             generado hasta el momento como CLAVES DEL DICCIONARIO
   *           - Se recomienda establecer una relacion de distancia entre configuraciones en distancesBetween() 
   *             y utilizar esa funcion como auxiliar */
  
  SpaceConfiguration nearest;
  double min_distance = std::numeric_limits<double>::max(); 
  
  for(const auto& config : graph_ )
  {
    double d = distancesBetween(config.first, rand_config_);
    if (d < min_distance && config.second.size() < 3) {
      min_distance = d;
      nearest = config.first;
    }
  }
  return nearest;
}

SpaceConfiguration robmovil_planning::PioneerRRTPlanner::steer()
{  
  /* COMPLETAR: Retornar una nueva configuracion a partir de la mas cercana near_config_.
   *            La nueva configuracion debe ser ademas la mas cercana a rand_config_ de entre las posibles.
   * 
   * ATENCION: - Utilizar las variables globales Vx_step_ , Wz_step_ para la aplicaciones de las velocidades
   *           - Pensar en la conversion de coordenadas polares a cartesianas al establecer la nueva configuracion
   *           - Utilizar angles::normalize_angle() */
  
  /* Ejemplo de como construir una posible configuracion: */
  //double x_posible = near_config_.get(0) /* + algo */;
  //double y_posible = near_config_.get(0) /* + algo */;
  //double theta_posible = angles::normalize_angle(near_config_.get(2) /* + algo */ );
  
  //SpaceConfiguration s_posible({ x_posible, y_posible, theta_posible });
  
  SpaceConfiguration steer;
  
  /* Conjunto de steers ya ocupados en la configuracion near_config_ */
  const std::list<SpaceConfiguration> occupied_steerings = graph_[near_config_];
  std::vector<SpaceConfiguration> free_steerings;

  double modulo = Vx_step_;
  double angulo;
  for (double delta = -Wz_step_; delta <= Wz_step_; delta += Wz_step_) {
    angulo = near_config_.get(2) + delta;
    double x_posible = near_config_.get(0) + modulo * cos(angulo);
    double y_posible = near_config_.get(1) + modulo * sin(angulo);
    double theta_posible = angles::normalize_angle(angulo);
    free_steerings.push_back(SpaceConfiguration ({x_posible, y_posible, theta_posible}));
  }
  
  /* RECOMENDACION: Establecer configuraciones posibles en free_steerings y calcular la mas cercana a rand_config_ */

  double min_distance = std::numeric_limits<double>::max(); 
  
  for(auto& config : free_steerings ) {
    double d = distancesBetween(config, rand_config_);
    if (d < min_distance) {
      min_distance = d;
      steer = config;
    }
  }
  return steer;
}

bool robmovil_planning::PioneerRRTPlanner::isFree()
{
  /* COMPLETAR: Utilizar la variable global new_config_ para establecer si existe un area segura alrededor de esta */

  for (int i = -1; i <= 1; i++) {
    for (int j = -1; j <= 1; j++) {
      if (isPositionOccupy(new_config_.get(0)+i*grid_->info.resolution, new_config_.get(1)+j*grid_->info.resolution)) {
        return false;
      }
    }
  }
  return true;
}

bool robmovil_planning::PioneerRRTPlanner::isGoalAchieve()
{
  
  /* COMPLETAR: Comprobar si new_config_ se encuentra lo suficientemente cerca del goal.
   * 
   * ATENCION: Utilizar abs( angles::shortest_angular_distance(c1.get(2), c2.get(2)) )
   *           para medir la distancia entre las orientaciones */

  double dist_euclid = sqrt (pow(new_config_.get(0) - goal_config_.get(0), 2) + pow(new_config_.get(1) - goal_config_.get(1), 2));
  double dist_angular = fabs(angles::shortest_angular_distance(new_config_.get(2), goal_config_.get(2)));

  return dist_euclid < 0.1 && dist_angular < M_PI/2;
}



/* DESDE AQUI YA NO HACE FALTA COMPLETAR */



bool robmovil_planning::PioneerRRTPlanner::isValid()
{ return true; }

void robmovil_planning::PioneerRRTPlanner::notifyTrajectory(robmovil_msgs::Trajectory& result_trajectory,
                                                            const SpaceConfiguration& start, const SpaceConfiguration& goal, 
                                                            std::map<SpaceConfiguration, SpaceConfiguration>& came_from) const
{
  std::vector<SpaceConfiguration> path;
  SpaceConfiguration current = goal;
  
  path.push_back(current);
  
  while(current != start)
  {
    current = came_from[current];
    path.push_back(current);
  }
  
  result_trajectory.header.stamp = ros::Time::now();
  result_trajectory.header.frame_id = "odom";
  
  ros::Duration t_from_start = ros::Duration(0);
  ros::Duration delta_t = ros::Duration(1);

  /* Se recorre de atras para adelante */
  for (auto it = path.rbegin(); it != path.rend(); ++it) {    
    double config_x = it->get(0);
    double config_y = it->get(1);
    double config_theta = it->get(2);
    
    tf::Transform wp_odom_ref;
    wp_odom_ref.getOrigin().setX(config_x);
    wp_odom_ref.getOrigin().setY(config_y);
    wp_odom_ref.getOrigin().setZ(0);
    wp_odom_ref.setRotation(tf::createQuaternionFromYaw(config_theta));
    
    wp_odom_ref = map_to_odom_.inverse() * wp_odom_ref;
    
    // Se crean los waypoints de la trayectoria
    robmovil_msgs::TrajectoryPoint point_msg;
    
    transformTFToMsg(wp_odom_ref, point_msg.transform);
    
    if(it != path.rend()-1) {
      double config_dx = (it+1)->get(0) - config_x;
      double config_dy = (it+1)->get(1) - config_y;
      double config_dtheta = angles::shortest_angular_distance(config_theta, (it+1)->get(2));
      point_msg.velocity.linear.x = sqrt(pow(config_dx,2) + pow(config_dy,2));
      point_msg.velocity.angular.z = config_dtheta;
    }else{
      point_msg.velocity.linear.x = 0;
      point_msg.velocity.angular.z = 0;
    }
    
    point_msg.time_from_start = t_from_start;
    
    result_trajectory.points.push_back( point_msg );
    
    t_from_start += delta_t;
  }
}
