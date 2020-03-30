#include "sparse_voronoi/voronoi_algorithm.h"

#include "sparse_voronoi/constrained/CDT.h"
#include "sparse_voronoi/delaunator/delaunator.hpp"

namespace sparse_voronoi
{
typedef float CoordType;
typedef CDT::Triangulation<CoordType> Triangulation;
typedef CDT::V2d<CoordType> V2d;
typedef CDT::Vertex<CoordType> Vertex;
typedef CDT::Triangle Triangle;
typedef CDT::Box2d<CoordType> Box2d;
typedef CDT::Index Index;
typedef CDT::Edge Edge;
typedef CDT::VertInd VertInd;

Voronoi_Algorithm::Voronoi_Algorithm() {}

Voronoi_Algorithm::~Voronoi_Algorithm() {}

void Voronoi_Algorithm::initialize()
{
  if (pub_markers_)
  {
    voronoi_lines_.type = visualization_msgs::Marker::LINE_LIST;
    voronoi_lines_.color.b = 1.0;
    voronoi_lines_.color.a = 1.0;
    voronoi_lines_.scale.x = 0.02;
    delaunay_lines_.type = visualization_msgs::Marker::LINE_LIST;
    delaunay_lines_.color.r = 1.0;
    delaunay_lines_.color.a = 1.0;
    delaunay_lines_.scale.x = 0.02;
  }
  // intialize cmd weights
  cmd_weights_ = base_utils::linspace(0.0, 1.0, cmd_buffersize_);

  if (!use_dynamic_voronoi_)
  {
    voronoi_horizon_ = 1;  
    voronoi_dt_ = 0.0;  
  }
  
}

void Voronoi_Algorithm::processVoronoi(std_msgs::Header &header)
{
  // Update and clear containers
  base_header_ = header;
  delaunay_lines_.points.clear();
  delaunay_lines_.header = base_header_;
  voronoi_lines_.points.clear();
  voronoi_lines_.header = base_header_;

  // Convert EnvObjectList of obstacles  (possibly from various sources, but same frame_id) to lists of points. 
  obstacles_pts_ = obst_utils::convertObjectListToPoints(obstacles_, base_frame_id_,tf_listener_);

  if (tri_method_ == "constrained")
    constrainedTriangulation();
  else if (tri_method_ == "delaunay")
    delaunayTriangulation();
  else{
    ROS_ERROR("triangulation method (parameter: tri_method) not recognized!");
    ros::requestShutdown();
  }
  
  buildGraph();

  if (pub_markers_)
    buildMarkers();

}

void Voronoi_Algorithm::processMDP(geometry_msgs::Pose2D &global_pose2D, geometry_msgs::Twist &target_vel)
{
  if (voronoi_available_)
  {
    // ROS_DEBUG_STREAM("Starting MDP with v_u = " << target_vel_.linear.x << " w_u = " << target_vel_.angular.z);
    target_vel_ = target_vel;
    target_heading_ = base_utils::getAngle(target_vel);

    if (use_dynamic_reward_)
      updateBuffer(target_heading_, global_pose2D);
    // Update initial reward to all nodes based on latest user target heading 'target_heading_'

    updateRewards();    
    if (mdp_method_ == "valueiteration")
      valueIteration();
    else if (mdp_method_ == "qlearning")
      Qlearning();
    else{
      ROS_ERROR("MDP method (parameter: mdp_method) not recognized!");
      ros::requestShutdown();
    }

    findFullPath();
    findVoroPath();
    interpPath();

    // Can be used to produce terminal output information of the graph for debugging
    // for (auto time = 0; time < states_.size(); time++)
    // {
    //   ROS_WARN_STREAM("Time: " << time);
    //   ROS_WARN_STREAM("agent vertex index: " << v_agent_);
    //   ROS_WARN_STREAM("agent node index: " << t_agent_);
    //   for (auto &state : states_.at(time))
    //   {
    //     cout << " State: " << state.first;
    //     cout << " Reward: " << state.second.reward;
    //     cout << " Value: " << state.second.value;
    //     cout << " pos: x = " << state.second.pos.x << " y = " << state.second.pos.y << endl;

    //     cout << " Actions: ";
    //     for (auto &action : state.second.Q)
    //     {
    //       cout << "t_a: " << action.first.first << ", ";
    //       cout << "s_a: " << action.first.second << ", ";
    //       cout << " Q: " << action.second << ", ";
    //     }
    //     cout << endl;
    //   }
    // }

  }
  else
  {
    ROS_WARN_STREAM("Warning: Voronoi value iteration attempted before Voronoi diagram was constructed.");
  }
}

void Voronoi_Algorithm::delaunayTriangulation()
{
  // ROS_DEBUG_STREAM("Starting triangulation");
  states_.clear();
  edges_.clear();
  vertices_.clear();
  for (size_t time = 0; time < voronoi_horizon_; time++ )
  {    
    // ROS_DEBUG_STREAM("Starting triangulation for time " << time);
    states_t_.clear();
    edges_t_.clear();
    vertices_t_.clear();
    float dt = static_cast<float>(time) * voronoi_dt_;

    ////// Obstacles Processing //////
    // Transform obstacle parameters into a list of coordinates
    vector<double> coordinates;
    for (auto& obstacle : obstacles_.objects)
    {
      // Add circles.
      if (obstacle.ID == 7 || obstacle.ID == 8)
      {
        coordinates.push_back(obstacle.params[0] + obstacle.params[4]*dt);
        coordinates.push_back(obstacle.params[1] + obstacle.params[5]*dt);      
      }
      // Add lines
      else if ( obstacle.ID == 1 || obstacle.ID == 2 || obstacle.ID == 3 || obstacle.ID == 6)
      {
        coordinates.push_back(obstacle.params[0]);
        coordinates.push_back(obstacle.params[1]);        
        coordinates.push_back(obstacle.params[3]);
        coordinates.push_back(obstacle.params[4]);        
      }     
      // Add polygons
      else if (obstacle.ID == 4 || obstacle.ID == 5)
      {
        for (size_t i = 1; i < obstacle.params.size(); i+=3)
        {
          coordinates.push_back(obstacle.params[i]);
          coordinates.push_back(obstacle.params[i+1]);    
        }
      }
    }
    
    // Insert agent as obstacle point (vertex), at origin (local coordinate frame)
    v_agent_ = coordinates.size() / 2;
    coordinates.push_back(0);
    coordinates.push_back(0);

    ////// Triangulation //////
    delaunator::Delaunator delaunator(coordinates);

    // Insert agent as voronoi node, at origin
    size_t N_triangles = delaunator.triangles.size() / 3;
    size_t N_edges = delaunator.halfedges.size();
    N_states_ = N_triangles;

    ////// Building graph nodes for current layer //////
    // Find voronoi node locations by calculating incenter / circumcenter
    for (size_t t = 0; t < N_triangles; t++)
    {
      /* Triangles: 
             v3
             /\
        e3  / t\ e2
           /____\
        v1  e1  v2                 */      
      
      // Half edges of triangle t
      size_t edges[3] = {3 * t, 3 * t + 1, 3 * t + 2};
      vector<geometry_msgs::Point> pts;
      // Find the corner points of triangle t and store in vector pts
      for (size_t i = 0; i < 3; i++)
      {
        size_t vx = 2 * delaunator.triangles[edges[i]];
        size_t vy = 2 * delaunator.triangles[edges[i]] + 1;
        pts.push_back(rosmsg::makePoint(delaunator.coords[vx], delaunator.coords[vy]));
       
        // Store edges and vertices of the triangulation
        if (!vertices_t_.count(vx))
          vertices_t_.emplace(make_pair(vx, pts[i]));

        size_t e_opp = delaunator.halfedges[edges[i]];
        if (e_opp < N_edges) // if opposite edge index (much) larger than number of edges, there is no opposite triangle
        {
          size_t t_opp = floor(e_opp / 3);
          size_t v2 = 2*delaunator.triangles[edges[(i+1)%3]];
          edges_t_.emplace(make_pair(make_pair(vx,v2), make_pair(t, t_opp)));
        }

        // Create ROS visualization message
        if (pub_markers_  && time == 0) // only store the first triangulation diagram for visualization
        {
            delaunay_lines_.points.push_back(pts[i]);
            delaunay_lines_.points.push_back(pts[(i+1)%3]);
        }        
      }
      
      // Calculate node circumcenter, create a new node (State struct), and insert in current layer
      geometry_msgs::Point node = geom_utils::findInCenter(pts);
      states_t_.emplace(createNode(time, t, node));      
    }

    // Store nodes, edges and vertices of this layer, before moving to the next layer.
    states_.emplace(make_pair(time, states_t_));
    edges_.emplace(make_pair(time,edges_t_));
    vertices_.emplace(make_pair(time, vertices_t_));

    if (time == 0)
    {
      // insert agent
      N_states_+=1;  
      t_agent_ = N_triangles;
      states_.at(time).emplace(createNode(time, t_agent_, rosmsg::makePoint(0.0, 0.0)));    
      // Add voronoi connection to agent if one of the vertices of the triangle is the agent.
      for (size_t t = 0; t < N_triangles; t++)
      {
        size_t edges[3] = {3 * t, 3 * t + 1, 3 * t + 2};
        for (size_t e : edges)
        {
          size_t v = delaunator.triangles[e];
          if (v == v_agent_)
          {
            addAction(time, t_agent_, time, t);
          }
        }
      }
    }
  }
}

void Voronoi_Algorithm::constrainedTriangulation()
{
  // ROS_DEBUG_STREAM("Starting constrained triangulation");
  // making multiple layers of voronoi diagram, one for each time step in the prediction horizon.
  // These are stored in the states_ Dictionary. Each layer consists of a single dictionary of node objects (states_t_)
  // The algorithm only makes connections between layers, not within the layer itself. 
  states_.clear();
  edges_.clear();
  vertices_.clear();
  for (size_t time = 0; time < voronoi_horizon_; time++ )
  {    
    // ROS_DEBUG_STREAM("Starting triangulation for time " << time);
    states_t_.clear();
    edges_t_.clear();
    vertices_t_.clear();
    float dt = static_cast<float>(time) * voronoi_dt_;
    
    ////// Obstacles Processing //////
    // Transform obstacle parameters into a list of vertices and edges (constraints)
    vector<V2d> cdt_vertices;
    vector<Edge> cdt_edges;    
    VertInd vert_ind = 0;

    // ROS_DEBUG_STREAM("Processing obstacles");
    for (auto& obstacle : obstacles_.objects)
    {
      // Add circles.
      if (obstacle.ID == 7 || obstacle.ID == 8)
      {
        cdt_vertices.push_back(V2d::make(obstacle.params[0] + obstacle.params[4]*dt,  // x + vx*dt
                                         obstacle.params[1] + obstacle.params[5]*dt)); // y + vy*dt
        vert_ind += 1;                                    
      }
      // Add lines
      else if ( obstacle.ID == 1 || obstacle.ID == 2 || obstacle.ID == 3 || obstacle.ID == 6)
      {
        cdt_vertices.push_back(V2d::make(obstacle.params[0], obstacle.params[1])); 
        vert_ind += 1;     
        cdt_vertices.push_back(V2d::make(obstacle.params[3], obstacle.params[4])); 
        Edge cdt_edge = {vert_ind - 1, vert_ind};
        cdt_edges.push_back(cdt_edge);
        vert_ind += 1;     
      }     
      // Add polygons
      else if (obstacle.ID == 4 || obstacle.ID == 5)
      {
        cdt_vertices.push_back(V2d::make(obstacle.params[0], obstacle.params[1])); 
        vert_ind += 1;
        for (size_t i = 1; i < obstacle.params.size(); i+=3)
        {
          cdt_vertices.push_back(V2d::make(obstacle.params[i], obstacle.params[i+1]));
          Edge cdt_edge = {vert_ind - 1, vert_ind};
          cdt_edges.push_back(cdt_edge);
          vert_ind += 1;
        }
        Edge cdt_edge = {vert_ind-1 - obstacle.params.size()/3, vert_ind-1};
        cdt_edges.push_back(cdt_edge);
      }
    }

    // Insert agent as obstacle point (vertex), at origin (local coordinate frame)
    v_agent_ = cdt_vertices.size();
    cdt_vertices.push_back(V2d::make(0.0, 0.0));

    ////// Triangulation //////
    // ROS_DEBUG_STREAM("Perform Triangulation");
    Triangulation triangulator = Triangulation(CDT::FindingClosestPoint::BoostRTree);
    triangulator.insertVertices(cdt_vertices);
    triangulator.insertEdges(cdt_edges);
    triangulator.eraseSuperTriangle();

    // Insert agent as voronoi node, at origin (local coordinate frame)
    size_t N_triangles = triangulator.triangles.size();
    size_t N_edges = cdt_edges.size();
    N_states_ = N_triangles;
    ////// Building graph nodes for current layer //////
    // ROS_DEBUG_STREAM("Build graph nodes");
    // Find voronoi node locations by calculating incenter / circumcenter
    for (size_t t = 0; t < N_triangles; t++)
    {
      /* Triangles: Counter-clockwise winding, vertices v, neighbors n, triangles t:
            v3
            /\
          n3/  \n2
          /____\
        v1  n1  v2                 */

      Triangle& triangle = triangulator.triangles[t];      
      // Store Voronoi nodes (=triangle incenters)
      // Find the corner points of triangle t and store in vector pts
      vector<geometry_msgs::Point> pts;
      for (auto& v : triangle.vertices)
        pts.push_back(rosmsg::makePoint(triangulator.vertices[v].pos.x, triangulator.vertices[v].pos.y));
    
      // Store edges and vertices of the triangulation in separate dictionaries, stored in the voronoi_algorithm object, (not in the CDT triangulation object)
      for (size_t i = 0; i < 3; i++)
      {
        // Vertices representing the corners of triangles of the triangulation
        if (!vertices_t_.count(triangle.vertices[i]))
          vertices_t_.emplace(make_pair(triangle.vertices[i], pts[i]));
        // Edges representing the connections between two vertices of the triangulation
        if (triangle.neighbors[i] < N_triangles)
          edges_t_.emplace(make_pair(make_pair(triangle.vertices[i],triangle.vertices[(i+1)%3]), make_pair(t, triangle.neighbors[i])));

        // Create ROS visualization message
        if (pub_markers_ && time == 0) // only store the first triangulation diagram for visualization
        {
          delaunay_lines_.points.push_back(pts[i]);
          delaunay_lines_.points.push_back(pts[(i+1)%3]);
        }
      }
      
      // Calculate node circumcenter, create a new node (State struct), and insert in current layer
      geometry_msgs::Point node = geom_utils::findInCenter(pts);
      
      bool node_ok = true;
      // Check edges of the triangle, to calculate whether node is too close to an obstacle.
      // Node will not be added to graph if it is closer than the min_gap set in the parameters. 
      for (size_t i = 0; i < 3; i++)
      {
        Edge edge = {triangle.vertices[i], triangle.vertices[(i+1)%3]};
        for (auto& fixed_edge : triangulator.fixedEdges)
        {
          if (edge == fixed_edge)
          {
            auto point1 = rosmsg::makePoint(triangulator.vertices[edge.v1()].pos.x, triangulator.vertices[edge.v1()].pos.y);
            auto point2 = rosmsg::makePoint(triangulator.vertices[edge.v2()].pos.x, triangulator.vertices[edge.v2()].pos.y);
            if (geom_utils::distanceToLineSegment(node,point1,point2) < min_gap_)
              node_ok = false;
          }
        }
      } 

      if (node_ok)
      {
        states_t_.emplace(createNode(time, t, node));

        if (use_targets_)   
        {
          // Check if there are any additional targets (stored as a PoseArray) located inside this triangle. 
          // If so, add targets as additional states, and connect them to this node. 
          for (auto& target : voro_targets_.poses)
          {
              // Check if the target is placed inside the current triangle:
            if (geom_utils::isInsidePolygon(pts,target.position))
            {
              // Create a new node. 
              N_states_ += 1;
              states_t_.emplace(createNode(time, N_states_, target.position));
              // Add connection from the triangle node to this target. 
              addAction(time, t, time, N_states_);
            }
          }
        }
      }
    }

    // ROS_DEBUG_STREAM("Store graph nodes");
    // Store nodes, edges and vertices of this layer, before moving to the next layer.
    states_.emplace(make_pair(time, states_t_));
    edges_.emplace(make_pair(time,edges_t_));
    vertices_.emplace(make_pair(time, vertices_t_));

    if (time == 0)
    {
      // insert agent as Voronoi Node at the start
      N_states_ += 1;
      t_agent_ = N_states_;
      states_.at(time).emplace(createNode(time, t_agent_, rosmsg::makePoint(0.0, 0.0))); 
      // Add voronoi connection to agent if one of the vertices of the triangle is the agent.
      for (size_t t = 0; t < N_triangles; t++)
      {
        Triangle& triangle = triangulator.triangles[t];
        for (auto& v : triangle.vertices)
        {
          if (v == v_agent_)
            addAction(time, t_agent_, time, t);
        }
      }
    }
  }
  states_t_.clear();
  edges_t_.clear();
  vertices_t_.clear();  
}

void Voronoi_Algorithm::buildGraph()
{
  ////// Building graph connections //////
  // Build graph by connecting adjacent triangles / nodes in previous time layer
  // This is done by iterating through the half_edges of  the current layer, built in previous step,
  // and comparing these to half_edges stored in previous layers.
  // ROS_DEBUG_STREAM("Building graph");
  size_t time_start = 0;
  // Iterate over layers of voronoi diagrams, each with their distinct time index 'time1'
  for (size_t time1 = 0; time1 < voronoi_horizon_; time1++ )
  {
    if (use_dynamic_voronoi_)
      time_start = time1 + 1;

    // Iterate over the voronoi lines connecting nodes (called graph edges) in the current time layer
    for (auto& edge : edges_.at(time1))
    {
      auto vert_pair = edge.first;    // provides the indices of the 2 vertices (obstacles) related to this graph edge / node connection
      auto node_pair = edge.second;   // provides the indices of the 2 nodes (states) related to this graph edge / node connection
      // If possible, a connection will be made from this node t1, in layer time1, to another node with the same vertices, in a different layer time2
      size_t t1 = node_pair.first;    
      // Iterate over subsequent layers of voronoi diagrams, each with their distinct time index 'time2'
      for (size_t time2 = time_start; time2 < voronoi_horizon_; time2++ )
      {
        // Check if this layer contains the same pair of vertices (i.e. the same graph edge / node connection)
        if (edges_.at(time2).count(vert_pair))
        {
          // This is the next node to which t1 may be connected. 
          size_t& t2 = edges_.at(time2).at(vert_pair).second;
          // Check if both states are avaiable. Else do nothing. 
          if (states_.at(time1).count(t1) && states_.at(time2).count(t2))
          {
            // Check for some constraints: 
            // The minimum gap for the robot to pass through and 
            // The maximum distance the robot will cover in this time interval.
            double gap = base_utils::euclideanDistance(vertices_.at(time1).at(vert_pair.first), vertices_.at(time2).at(vert_pair.second));
            double dist = base_utils::euclideanDistance(states_.at(time1).at(t1).pos, states_.at(time2).at(t2).pos);
            double max_dist =  sqrt(2) * fabs(target_vel_.linear.x) * static_cast<double>(time2-time1);
            // double max_dist =  sqrt(2) * 0.5 * static_cast<double>(time2-time1);
            if (gap > min_gap_)
            {
              if (!use_dynamic_constraint_)
              {
                addAction(time1, t1, time2, t2);
                break; // don't add the same edge again
              }
              else if (dist < max_dist) // check whether wheelchair is able to traverse this distance in the given time
              {
                addAction(time1, t1, time2, t2);
                break; // don't add the same edge again
              }
            }
          }
        }
      }
    }
  }
  voronoi_available_ = true;
}

void Voronoi_Algorithm::buildMarkers()
{
  // ROS_DEBUG_STREAM("Building graph visualization");
  geometry_msgs::Point node1, node2;
  for (size_t time = 0; time < voronoi_horizon_; time++ )
  {
    for (auto& state1 : states_.at(time))
    {
      // int conn = 0;
      for (auto& action : state1.second.Q)
      {
        auto& state2 = states_.at(action.first.first).at(action.first.second);
        voronoi_lines_.points.push_back(state1.second.pos);
        voronoi_lines_.points.push_back(state2.pos);
        // conn += 1;
      }
      // ROS_DEBUG_STREAM("Layer: " << time << " Node: " << state1.first << " Connections: " << conn << " X: " << state1.second.pos.x << " Y: " << state1.second.pos.y);
    }
    // ROS_DEBUG_STREAM("Layer: " << time << " nr of nodes: " << states_.at(time).size() << " nr of edges: " << edges_.at(time].size());
  }
  // ROS_DEBUG_STREAM("size voronoi visualization diagram: " << voronoi_lines_.points.size()/2);  
}

void Voronoi_Algorithm::updateRewards()
{
  // ROS_DEBUG_STREAM("Update Rewards");
  // Update all (static) rewards of coming to a certain node
  for (size_t time = 0; time < voronoi_horizon_; time++)
  {
    for (auto &state : states_.at(time))
    {
      state.second.reward = calcReward(state.second.pos);   // calculate reward (static)
      state.second.value = state.second.reward;             // initialize value (dynamic)
    }
  }
  states_.at(0).at(t_agent_).reward = -1;
  states_.at(0).at(t_agent_).value = -1;
}

void Voronoi_Algorithm::valueIteration()
{
  // Value iteration performed on the dictionary states_t_, which contains all information about the graph.
  // ROS_DEBUG_STREAM("Start Value iteration");
  // initialize error and start value iteration loop
  double error = 0, prev_error = 0;
  for (int i = 0; i < max_it_; i++)
  {
    error = 0;
    for (auto time = 0; time < voronoi_horizon_; time++)
    {
      for (auto& state : states_.at(time))
      {
        double V = state.second.value;
        // Apply bellman equation to each state.
        for (auto& Q_sa : state.second.Q) // Q_sa is reward for going to next state
        {
          auto& action = Q_sa.first;
          auto& next_state = states_.at(action.first).at(action.second); 
          // Q value based on reward received in next state + discounted value of next state (determined by future rewards)
          // Q_sa.second = next_state.reward + gamma_ * next_state.value;
          // Q value based on reward received in next state + reward (cost) of following edge + discounted value of next state (determined by future rewards)
          Q_sa.second = (next_state.reward + state.second.R.at(action)) + gamma_ * next_state.value;
        }
        // Value of each state is defined by highest achievable reward of next state
        if (state.second.Q.size() > 0){
          double best_reward = get_max(state.second.Q).second;
          if (!isnan(best_reward)){
            state.second.value = best_reward;
          }
          else state.second.value = 0.0;
        }
        else state.second.value = 0.0;

        // ROS_DEBUG_STREAM("value " << state.second.value << " prev value " << V);
        error +=  state.second.value - V;
      }
    }
    // Stop iterating when converged.
    if (i > min_it_)
    {
      // ROS_DEBUG_STREAM("iteration: " << i << " residual error: " << error);
      if (fabs(prev_error - error) < it_err_thresh_)
      {
        break;
      }
    }
    prev_error = error;    
  }
}

void Voronoi_Algorithm::Qlearning()
{
  // Value iteration performed on the dictionary states_t_, which contains all information about the graph.
  // ROS_DEBUG_STREAM("Start Q-Value iteration");
  // initialize error and start value iteration loop
  double error, prev_error = 0;
  double Qmax_ns = 0;
  for (int i = 0; i < max_it_; i++)
  {
    error = 0;
    for (auto time = 0; time < voronoi_horizon_; time++)
    {
      for (auto& state : states_.at(time))
      {
        double V = state.second.value;
        // Apply bellman equation to each state.
        for (auto& Q_s : state.second.Q) // Q_s is reward for going from state s to next state ns
        {
          auto& action = Q_s.first;
          auto& next_state = states_.at(action.first).at(action.second);
          
          // Modify Q values by including the cost of making sharp turns.
          auto Q_ns = next_state.Q;
          double angle1 = base_utils::getAngle(state.second.pos, next_state.pos);
          for (auto& Q_nsa : Q_ns)
          {
            auto& action2 = Q_nsa.first;
            double angle2 = base_utils::getAngle(next_state.pos,
                                                  states_.at(action2.first).at(action2.second).pos);
            Q_nsa.second += -cost_angle2_*pow(base_utils::shiftAngle(angle2-angle1)/M_PI,2);
          }
          // This incorporates an additional cost for sharp costs at the next action, into the value calculation of the current action. 
          if (Q_ns.size() > 0){
            double Qmax_ns = get_max(state.second.Q).second;
            if (isnan(Qmax_ns)){
              Qmax_ns = 0;
            }
          }
          else Qmax_ns = 0;

          Q_s.second = Q_s.second + alpha_ * ( (next_state.reward + state.second.R.at(action)) +  gamma_ * Qmax_ns - Q_s.second);
        }
        // Value of each state is defined by highest achievable reward of next state
        if (state.second.Q.size() > 0){
          double best_reward = get_max(state.second.Q).second;
          if (!isnan(best_reward)){
            state.second.value = best_reward;
          }
          else state.second.value = 0.0;  
        }
        else state.second.value = 0.0;        
        
        error +=  state.second.value - V;
      }
    }
    // Stop iterating when converged.
    if (i > min_it_)
    {
      // ROS_DEBUG_STREAM("iteration: " << i << " residual error: " << error);
      if (fabs(prev_error - error) < it_err_thresh_)
        break;
    }
    prev_error = error;    
  }
}

void Voronoi_Algorithm::findVoroPath()
{
  // ROS_DEBUG_STREAM("Find Voro Path");
  voro_path_.poses.clear();
  voro_path_.header = base_header_;  
  std::pair<size_t,size_t> ns_index, s_index = {0, t_agent_}; // indices of current and next state along trajectory
  voropath_states_.clear();
  voropath_states_.push_back(s_index);
  State prev_state;
  geometry_msgs::PoseStamped waypoint = rosmsg::makePoseStamped(0.0, 0.0, 0.0, base_header_);
  voro_path_.poses.push_back(waypoint);
  double step_size = fabs(target_vel_.linear.x) * time_interval_;
  double path_dist = 0;
  int tot_steps = 0;
  size_t time_ns = 0;

  while (true)
  {
    auto& state = states_.at(s_index.first).at(s_index.second);

    // ROS_DEBUG_STREAM("VoroPath Time: " << s_index.first << " State: " << s_index.second);

    // Quit when current state doesn't have any connections
    if (state.Q.size() == 0)
    {
      // ROS_DEBUG_STREAM("Voropath quit because current state has no connections");
      state.edge_steps += prediction_horizon_ - 2 - tot_steps;
      tot_steps += prediction_horizon_ - 2 - tot_steps;    
      break;
    }

    ns_index = get_max(state.Q).first;
    auto& next_state = states_.at(ns_index.first).at(ns_index.second);
    time_ns = ns_index.first;
    double angle = base_utils::shiftAngle(base_utils::getAngle(state.pos, next_state.pos) - 
                                          base_utils::getAngle(prev_state.pos, state.pos));  

    // Quit when 
    // - path already contains the next state
    // - constructor tries to make a very sharp turn
    // - path exceeds max number of states    
    if (count(voropath_states_.begin(), voropath_states_.end(), ns_index) ||
        (voropath_states_.size() > 1 && fabs(angle) > max_path_angle_) ||
        (voro_path_.poses.size() > max_nr_states_))
    {
      // ROS_DEBUG_STREAM("Voronoi path finished before exceeding prediciton horizon");
      // Adjust current trajectory such that it contains the correct number of steps (= prediction horizon)
      state.edge_steps += prediction_horizon_ - 2 - tot_steps;
      tot_steps += prediction_horizon_ - 2 - tot_steps;
      break;
    }

    // Stop path construction when (creating new interpolated node and add to path):
    // - path exceeds prediction horizon
    next_state.edge_length = base_utils::euclideanDistance(state.pos, next_state.pos);
    next_state.edge_steps = round(next_state.edge_length / step_size);
    
    if (tot_steps + next_state.edge_steps >= (prediction_horizon_ - 1))
    {
      // ROS_DEBUG_STREAM("voronoi path stopped after exceeding prediction horizon");
      // Define number of steps left in prediction horizon and update final state such that predicted interpolated trajectory matches prediction horizon
      next_state.edge_steps = prediction_horizon_ - 1 - tot_steps;
      double edge_frac = next_state.edge_steps * step_size / next_state.edge_length;
      double dx = (next_state.pos.x - state.pos.x) * edge_frac;
      double dy = (next_state.pos.y - state.pos.y) * edge_frac;

      // Create a new node, defining the end of the prediction horizon, along the current edge.
      N_states_+=1;
      size_t new_node = N_states_;
      states_.at(time_ns).emplace(createNode(time_ns, new_node,
                                          rosmsg::makePoint(state.pos.x + dx,
                                                            state.pos.y + dy)));

      auto& new_state = states_.at(time_ns).at(new_node);
      new_state.edge_length = base_utils::euclideanDistance(new_state.pos, state.pos);
      new_state.edge_steps = prediction_horizon_ - 1 - tot_steps;

      // Place new node in trajectory.
      state.ns_index = make_pair(time_ns,new_node);
      waypoint.pose.position = states_.at(time_ns).at(new_node).pos;
      voro_path_.poses.push_back(waypoint);

      tot_steps += new_state.edge_steps;
      path_dist += new_state.edge_length;
      break;
    }
    else // add new node to path and move on.
    {
      // Place next node in trajectory
      waypoint.pose.position = next_state.pos;
      voro_path_.poses.push_back(waypoint);
      voropath_states_.push_back(ns_index);
      state.ns_index = ns_index;
      prev_state = state;

      tot_steps += next_state.edge_steps;
      path_dist += next_state.edge_length;

    }

    s_index = ns_index;
  }
  // ROS_DEBUG_STREAM("path distance: " << path_dist);
  // ROS_DEBUG_STREAM("path steps: " << tot_steps);
  // ROS_DEBUG_STREAM("path states size: " << voropath_states_.size());
}

void Voronoi_Algorithm::interpPath()
{
  // ROS_DEBUG_STREAM("interpolating voro path");
  interp_path_.clear();
  std::vector<geometry_msgs::Pose2D> path;
  geometry_msgs::Pose2D waypoint = rosmsg::makePose2D(0, 0, 0);
  path.push_back(waypoint);

  for (auto& s_index : voropath_states_)
  {
    auto& state = states_.at(s_index.first).at(s_index.second);
    // ROS_DEBUG_STREAM( " State: " << s_index.second);
    // ROS_DEBUG_STREAM(" Next State: " << state.ns_index.second);

    if (states_.at(state.ns_index.first).count(state.ns_index.second))
    {
      auto& next_state = states_.at(state.ns_index.first).at(state.ns_index.second);
      for (int j = 0; j < static_cast<int>(next_state.edge_steps); j++)
      {
        waypoint.x = state.pos.x + (next_state.pos.x - state.pos.x) * static_cast<double>(j) / next_state.edge_steps;
        waypoint.y = state.pos.y + (next_state.pos.y - state.pos.y) * static_cast<double>(j) / next_state.edge_steps;
        waypoint.theta = atan2(next_state.pos.y - state.pos.y, next_state.pos.x - state.pos.x);
        path.push_back(waypoint);
      }
    }
    else
    {
      // ROS_DEBUG_STREAM("tried looking up a state which is not available: " << state.ns_index.second);
      for (int j = path.size(); j < prediction_horizon_; j++)
      {
        path.push_back(waypoint);
      }
    }  
  }
  if (path.size() > prediction_horizon_)
  {
    for (int i = 0; i < prediction_horizon_; i++)
      interp_path_.push_back(path[i]);
  }
  else
{
  interp_path_ = path;
}

}

void Voronoi_Algorithm::findFullPath()
{
  // ROS_DEBUG_STREAM("Find full path");
  full_path_.poses.clear();
  full_path_.header = base_header_;
  std::pair<size_t,size_t> ns_index, s_index = {0, t_agent_}; // indices of current and next state along trajectory
  fullpath_states_.clear();
  fullpath_states_.push_back(s_index);
  State prev_state;
  geometry_msgs::PoseStamped waypoint = rosmsg::makePoseStamped(0.0, 0.0, 0.0, base_header_);
  full_path_.poses.push_back(waypoint);  

  while (true)
  {
    auto& state = states_.at(s_index.first).at(s_index.second);

    // ROS_DEBUG_STREAM("Full Path Time: " << s_index.first << " State: " << s_index.second);

    // Quit when current state doesn't have any connections
    if (state.Q.size() == 0)
      break;

    ns_index = get_max(state.Q).first;
    auto& next_state = states_.at(ns_index.first).at(ns_index.second);

    // Stop path construction when:
    // - path already contains the new state
    // - constructor tries to make a very sharp turn
    // - path exceeds max number of states
    double angle = base_utils::shiftAngle(base_utils::getAngle(state.pos, next_state.pos) - 
                                          base_utils::getAngle(prev_state.pos, state.pos));     
    if (count(fullpath_states_.begin(), fullpath_states_.end(), ns_index) ||
        (fullpath_states_.size() > 0 && fabs(angle) > max_path_angle_) ||
        full_path_.poses.size() > max_nr_states_)
      break;   

    // Add new state to trajectory
    waypoint.pose.position = next_state.pos;
    full_path_.poses.push_back(waypoint);
    fullpath_states_.push_back(ns_index);
    s_index = ns_index;
    prev_state = state;
  }
}

pair<size_t, State> Voronoi_Algorithm::createNode(size_t time, size_t t, geometry_msgs::Point point)
{
  // Create node t
  State node;
  node.pos = point;
  node.state = t;
  node.time = time;
  node.s_index = make_pair(time,t);
  return pair<size_t, State>(t, node);
}

void Voronoi_Algorithm::addAction(size_t& time1, size_t& tri1, size_t& time2, size_t& tri2)
{
  // i denotes a moment in time (i.e. one of the voronoi layers), t denotes a triangle / node / state in the denoted layer.
  if (states_.at(time1).count(tri1) && states_.at(time2).count(tri2))
  {
    if (noIntersections(states_[time1][tri1].pos, states_[time2][tri2].pos))
    {
      // ROS_DEBUG_STREAM("Adding connection to Graph");

      // The cost/reward of a certain action is calculated here as 
      // the angle compared to current user direction, divided by the distance. 
      double R = 0.0;
      if (cost_angle_ > 0.01)
      {
        double dist = base_utils::euclideanDistance(states_[time1][tri1].pos, states_[time2][tri2].pos);
        double angle = base_utils::shiftAngle(base_utils::getAngle(states_[time1][tri1].pos, states_[time2][tri2].pos));
        R = -cost_angle_/(1+dist) * pow(angle/M_PI,2);
      }
       
      states_[time1][tri1].Q.emplace(make_pair(make_pair(time2,tri2), R));
      states_[time1][tri1].R.emplace(make_pair(make_pair(time2,tri2), R));
    }
  }
  // else
  // {
  //   ROS_DEBUG_STREAM("Tried to connect two nodes one of which don't exist. Connected nodes:");
  //   ROS_DEBUG_STREAM("tri1: " << tri1 << " exists: " << states_.at(time1).count(tri1));
  //   ROS_DEBUG_STREAM("tri2: " << tri2 << " exists: " << states_.at(time2).count(tri2));
  // }
}

double Voronoi_Algorithm::calcReward(geometry_msgs::Point node)
{
  if (use_dynamic_reward_)
  {
    double reward = 0.0;
    for (int i = 0; i < local_pos_list_.size(); i++)
    {
      double heading_node = base_utils::getAngle(local_pos_list_[i], node);
      double heading_user = local_userheading_list_[i];
      reward += -pow(base_utils::shiftAngle(heading_user - heading_node)/M_PI, 2);
    }
    return reward;
  }
  else
  {
    return -pow(base_utils::shiftAngle(target_heading_ - base_utils::getAngle(node))/M_PI, 2);
  }

}

bool Voronoi_Algorithm::noIntersections(geometry_msgs::Point t1, geometry_msgs::Point t2)
{

  for (auto obstacle : obstacles_pts_)
  {

    for (int i = 1; i < obstacle.size(); i++)
    {
      // Iterate through segments. Check for collision with each obstacle.
      if (geom_utils::doIntersect(t1, t2, obstacle[i], obstacle[i - 1]))
      {
        return false;
      }
    }
    if (obstacle.size() > 2)
    {
      // for polygon, we have to 'close' the polygon.
      if (geom_utils::doIntersect(t1, t2, obstacle[obstacle.size() - 1], obstacle[0]))
      {
        return false;
      }
    }
  }
  return true;
}

void Voronoi_Algorithm::updateBuffer(double &target_heading, geometry_msgs::Pose2D &global_pose2D)
{
  // ROS_DEBUG_STREAM("Updating cmd buffer");
  // recalculate previous poses and user input to current local frame
  global_pose_list_.push_front(global_pose2D);
  if (global_pose_list_.size() > cmd_buffersize_)
    global_pose_list_.pop_back();

  global_userheading_list_.push_front(target_heading + global_pose2D.theta);
  if (global_userheading_list_.size() > cmd_buffersize_)
    global_userheading_list_.pop_back();

  local_userheading_list_.resize(global_pose_list_.size());
  local_pos_list_.resize(global_pose_list_.size());

  std_msgs::Header global_header = rosmsg::makeHeader(fixed_frame_id_, base_header_.stamp);
  std_msgs::Header local_header = rosmsg::makeHeader(base_frame_id_, base_header_.stamp);
  geometry_msgs::PointStamped pt_global, pt_local = rosmsg::makePointStamped(0.0, 0.0, local_header);
  ros::Time last_time = ros::Time(0);

  // Use tf_listener to find a transform  from fixed frame to current robot frame,
  // and use this transform to trannsform the stored global positions at previous iterations to the local frame.
  // ROS_DEBUG_STREAM("transforming pose buffer to local frame.");
  for (int i = 0; i < global_pose_list_.size(); i++)
  {
    try
    {
      pt_global = rosmsg::makePointStamped(global_pose_list_[i].x, global_pose_list_[i].y, global_header);
      tf_listener_.waitForTransform(fixed_frame_id_, base_frame_id_, last_time, ros::Duration(0.2));
      tf_listener_.transformPoint(base_frame_id_, last_time, pt_global, fixed_frame_id_, pt_local);
      local_pos_list_[i] = pt_local.point;
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::requestShutdown();
      continue;
    }
  }

  // ROS_DEBUG_STREAM("transforming user input buffer to local frame.");
  for (int i = 0; i < cmd_buffersize_; i++)
  {
    double local_heading = global_userheading_list_[i] - global_pose2D.theta;

    if (local_heading > M_PI)
      local_heading -= 2 * M_PI;
    else if (local_heading < -M_PI)
      local_heading += 2 * M_PI;

    local_userheading_list_[i] = local_heading;
  }
}


} // namespace sparse_voronoi
