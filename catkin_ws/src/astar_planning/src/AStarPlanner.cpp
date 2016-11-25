#include "AStarPlanner.h"
#include <angles/angles.h>
#include <queue>
#include <map>
#include <vector>

#define COST_BETWEEN_CELLS 1

robmovil_planning::AStarPlanner::AStarPlanner(ros::NodeHandle& nh)
: GridPlanner(nh)
{ }

std::vector<robmovil_planning::AStarPlanner::Cell> robmovil_planning::AStarPlanner::neighbors(const Cell& c)
{
  /* COMPLETAR: Calcular un vector de vecinos (distintos de c).
   * IMPORTANTE: Tener en cuenta los limites de la grilla (utilizar grid_->info.width y grid_->info.heigh)
   *             y aquellas celdas ocupadas */
  
  std::vector<Cell> neighbors;

  int l [3] = {-1,0,1};

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      Cell n = Cell(c.i+l[i], c.j+l[j]);
      if (!(l[j] == 0 && l[i] == 0) && (!isCellOccupy(n.i,n.j)) &&(c.j+l[j] < grid_->info.width)  && (c.i+l[i] < grid_->info.height)  && (c.j+l[j] > 0)  && (c.i+l[i] > 0)) {
        neighbors.push_back(n);
      }
    } 
  }

  return neighbors;
}

bool robmovil_planning::AStarPlanner::isaNeighborCellOccupy(const Cell& c){
  
  std::vector<robmovil_planning::AStarPlanner::Cell> ns = robmovil_planning::AStarPlanner::neighbors(c);
  for (int i = 0; i < ns.size(); i++) {
      if (isCellOccupy(ns[i].i, ns[i].j)) {
        return true;
      }
  }
  return false;
}

std::vector<robmovil_planning::AStarPlanner::Cell> robmovil_planning::AStarPlanner::neighbors2(const Cell& c)
{
  /* COMPLETAR: Calcular un vector de vecinos (distintos de c).
   * IMPORTANTE: Tener en cuenta los limites de la grilla (utilizar grid_->info.width y grid_->info.heigh)
   *             y aquellas celdas ocupadas */
  
  std::vector<Cell> neighbors;

  int l [3] = {-1,0,1};

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      Cell n = Cell(c.i+l[i], c.j+l[j]);
      if (!(l[j] == 0 && l[i] == 0) && !isaNeighborCellOccupy(n)) {
        neighbors.push_back(n);
      }
    } 
  }

  return neighbors;
}

double robmovil_planning::AStarPlanner::heuristic_cost(const Cell& start, const Cell& goal, const Cell& current)
{  
  /* COMPLETAR: Funcion de heuristica de costo */

  double goal_i, goal_j;
  double current_i, current_j;

  getCenterOfCell(goal.i, goal.j, goal_i, goal_j);
  getCenterOfCell(current.i, current.j, current_i, current_j);

  return fabs(goal_i - current_i) + fabs(goal_j - current_j);
}

bool contains(std::vector<robmovil_planning::AStarPlanner::Cell> v, robmovil_planning::AStarPlanner::Cell c) {
  for (int i = 0; i<v.size(); i++)
    if (v[i] == c) return true;
  return false;
}

bool robmovil_planning::AStarPlanner::do_planning(robmovil_msgs::Trajectory& result_trajectory)
{
  uint start_i, start_j;
  uint goal_i, goal_j;
  
  getCellOfPosition(starting_pose_.getOrigin().getX(), starting_pose_.getOrigin().getY(), start_i, start_j);
  getCellOfPosition(goal_pose_.getOrigin().getX(), goal_pose_.getOrigin().getY(), goal_i, goal_j);
  
  /* Celdas de inicio y destino */
  Cell start = Cell(start_i, start_j);
  Cell goal = Cell(goal_i, goal_j);
  
  /* Contenedores auxiliares recomendados para completar el algoritmo */
  std::priority_queue<CellWithPriority, std::vector<CellWithPriority>, PriorityCompare> frontier;
  std::map<Cell, Cell> came_from;
  std::map<Cell, double> cost_so_far;
  std::vector<Cell> closed;
  
  bool path_found = false;
  
  //std::cout << goal_i << ' ' << goal_j << std::endl;

  /* Inicializacion de los contenedores (start comienza con costo 0) */
  frontier.push(CellWithPriority(start, 0));
  cost_so_far[start] = 0;
  
  /* COMPLETAR: Utilizar los contenedores auxiliares para la implementacion del algoritmo A*
   * NOTA: Pueden utilizar las funciones neighbors(const Cell& c) y heuristic_cost(const Cell& start, const Cell& goal, const Cell& current)
   *       para la resolucion */
  
   while (frontier.size() > 0)  {
    Cell current = frontier.top();//vertex in OPEN with min f[];
    if (contains(closed, current)) continue;
    //std::cout << current.i << ' ' << current.j << std::endl;
    if (current == goal) {
      /*while (current != start) {
        result_trajectory.push_front(current);
        current = came_from[current];
      }
      result_trajectory.push_front(start);*/
      path_found = true;
      break;

    } 
    frontier.pop();
    closed.push_back(current);

    std::vector<Cell> neighbours = neighbors(current);
    for (int i = 0; i < neighbours.size(); i++) {
      Cell neighbor = neighbours[i];
      if (contains(closed, neighbor)) {
        continue;
      }
      int cost = cost_so_far[current] + COST_BETWEEN_CELLS;
      if (cost_so_far.count(neighbor) && cost >= cost_so_far[neighbor]) {
        continue; 
      }
      came_from[neighbor] = current;
      cost_so_far[neighbor] = cost;
      //std::cout << neighbor.i << ' ' << neighbor.j <<  ' ' << cost_so_far[neighbor] << '+' << heuristic_cost(start, goal, neighbor) << std::endl;

      frontier.push(CellWithPriority(neighbor, cost_so_far[neighbor] + heuristic_cost(start, goal, neighbor)));
    }
  }

  if(not path_found)
    return false;
  
  /* Construccion y notificacion de la trajectoria.
   * NOTA: Se espera que came_from sea un diccionario representando un grafo de forma que:
   *       goal -> intermedio2 -> intermedio1 -> start */
  notifyTrajectory(result_trajectory, start, goal, came_from);

  return true;
}

void robmovil_planning::AStarPlanner::notifyTrajectory(robmovil_msgs::Trajectory& result_trajectory, const Cell& start, const Cell& goal, 
                                                       std::map<Cell, Cell>& came_from)
{
  std::vector<Cell> path;
  Cell current = goal;
  
  path.push_back(current);
  
  while(current != start)
  {
    current = came_from[current];
    path.push_back(current);
  }

  /* Se recorre de atras para adelante */
  for (auto it = path.rbegin(); it != path.rend(); ++it) {
    ROS_INFO_STREAM("Path " << it->i << ", " << it->j);
    
    double cell_x, cell_y;
    
    getCenterOfCell(it->i, it->j, cell_x, cell_y);
    
    // Se crean los waypoints de la trajectoria
    robmovil_msgs::TrajectoryPoint point_msg;

    point_msg.transform.translation.x = cell_x;
    point_msg.transform.translation.y = cell_y;
    point_msg.transform.translation.z = 0;
    
    if(it != path.rend()-1){
      double delta_x, delta_y;
      getCenterOfCell((it+1)->i, (it+1)->j, delta_x, delta_y);
      delta_x = delta_x - cell_x;
      delta_y = delta_y - cell_y;
      point_msg.transform.rotation = tf::createQuaternionMsgFromYaw( angles::normalize_angle(atan2(delta_y, delta_x)) );
    } else
      point_msg.transform.rotation = tf::createQuaternionMsgFromYaw( angles::normalize_angle(atan2(cell_y, cell_x)) );
    
    result_trajectory.points.push_back( point_msg );
  }
}
