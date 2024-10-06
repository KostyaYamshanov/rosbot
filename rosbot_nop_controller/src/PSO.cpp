#include "PSO.hpp"
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PointStamped.h"
#include "gazebo_msgs/ModelStates.h"

namespace pso
{

void run_to_goal(Model::State& currState, const Model::State& Goal, const float dt, const float max_time, float& time_spent)
{
    NetOper nop = NetOper();
    nop.setLocalTestsParameters();

    Model model(currState, dt);
    Controller controller(Goal, nop);

    Runner runner(model, controller); 
    runner.init(currState);
    runner.setGoal(Goal);

    float time = 0.;                 
    while (time < max_time) 
    {
        // currState.print();
        // Goal.print();
        // std::cout<< "DELTA " << currState.dist(Goal) <<std::endl;
        currState = runner.makeStep();
        time += dt;
        if (currState.distXY(Goal) < EPS)
        {
            // std::cout<< "DELTA BREAK" << currState.dist(Goal) <<std::endl;
            break;
        } 
    }
    // time_spent += time;
}

// TODO добавить время
float CostFunction(std::vector<float> points,  Model::State MainGoal, const float time_step, const float dt)
{
    Model::State currState = {0., 0., 0.}; // start position
    float time_spent = 0.;
    for(size_t i = 0; i < points.size(); i = i + 3)
    {
        Model::State Goal = {points[i], points[i+1], points[i+2]};
        run_to_goal(currState, Goal, dt, time_step, time_spent);
    }
    return std::sqrt(currState.dist(MainGoal) * currState.dist(MainGoal)) + time_spent; 
}


Particle::Particle(const std::vector<float>& initial_state, Model::State main_goal, const float time_step, const float dt):
main_goal(main_goal), time_step(time_step), dt(dt)
{
    // set random velocities
    N = initial_state.size();
    curr_state = initial_state;
    best_state = initial_state;
    velocities.resize(N);

    for(auto& v : velocities)
    {
        v = (float)rand() / ((float)RAND_MAX + 0.1);
    }
}

void Particle::evaluate()
{
    curr_error = CostFunction(curr_state, main_goal, time_step, dt);
    if (curr_error <= best_error)
    {
        best_state = curr_state;
        best_error = curr_error;
    }
    
}

void Particle::update_velocities(std::vector<float> best_global_state)
{
    float w=1.0;   // constant inertia weight (how much to weigh the previous velocity)
    float c1=0.65;    // cognative constant
    float c2=0.65;    // social constant
    
    for(size_t i = 0; i < N; ++i){
        float r1 = (float)rand() / ((float)RAND_MAX + 0.1);
        float r2 = (float)rand() / ((float)RAND_MAX + 0.1);
        float vel_cognitive = c1 * r1 * (best_state[i] - curr_state[i]);
        float vel_social = c2 * r2 * (best_global_state[i] - curr_state[i]);
        velocities[i] = w * velocities[i] + vel_cognitive + vel_social;
    }
}

void Particle::update_state()
{
    for (size_t i = 0; i < N; ++i)
    {
        curr_state[i]=curr_state[i] + velocities[i];
    }
}



PSO::PSO(std::vector<float> initial_state, size_t numParticles, size_t maxIter, Model::State main_goal, float time_step, float dt): 
    maxIter(maxIter)
    , dt(dt)
    , time_step(time_step)
    , main_goal(main_goal)
{   
    best_global_state = initial_state;
    // init particles swarm
    swarm.resize(numParticles);
    for(size_t i = 0; i < swarm.size(); ++i)
        swarm[i] = Particle(initial_state, main_goal, time_step, dt);

    fit();
}


std::vector<float> PSO::fit()
{
    for(size_t i = 0; i < maxIter; ++i)
    {  
        for (auto& p : swarm)
        {
            p.evaluate();
            if (p.curr_error < best_global_error)
            {
                best_global_state=p.curr_state;
                best_global_error=p.curr_error;
            }

            p.update_velocities(best_global_state);
            p.update_state();
        }
    }
    return best_global_state;
}


};


Model::State rosbot_state{};
// constexpr size_t rosbot_model_id = 1;

void model_state_cb(const nav_msgs::Odometry::ConstPtr &msg) {

  const auto &q = msg->pose.pose.orientation;

  double siny_cosp = 2. * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1. - 2. * (q.y * q.y + q.z * q.z);
  double yaw = std::atan2(siny_cosp, cosy_cosp);

  rosbot_state.x = msg->pose.pose.position.x;
  rosbot_state.y = msg->pose.pose.position.y;
  rosbot_state.yaw = yaw;
  std::cout<<"!!!"<<std::endl;
  rosbot_state.print();
}

int main(int argc, char **argv)
{   
    ros::init(argc, argv, "PSO");
    ros::NodeHandle nh;
        // Создание публикации в топик /clicked_point
    ros::Publisher point_pub = nh.advertise<geometry_msgs::PointStamped>("/clicked_point", 10);
    ros::Subscriber models_sub = nh.subscribe("/odom", 1, model_state_cb);
    // TS 5 and q.size = 3 - ok without last point
	std::cout<<"PSO START"<<std::endl;
    float time_step = 5; // sec
    float dt = 0.01; // sec
    size_t numParticles = 50;
    size_t maxIter = 10;
    Model::State main_goal = {1.,1.,0.};
    // std::vector<float> q = {0.,0.,0., 0.,0.,0., 0.,0.,0., 0.,0.,0., 0.,0.,0., 0.,0.,0., 0.,0.,0., 0.,0.,0., 0.,0.,0., 0.,0.,0., 0.,0.,0., 0.,0.,0.}; // vector to be optimized (a.k.a initial state vector)
    std::vector<float> q = {0.,0.,0., 0.,0.,0., 0.,0.,0.}; // vector to be optimized (a.k.a initial state vector)
    
    auto pso = pso::PSO(q, numParticles, maxIter, main_goal, time_step, dt);

    std::cout<<"q: ";
    for(auto i : q)
        std::cout<<i<<" ";
    std::cout<<std::endl<<"Result: ";
    for(auto i : pso.best_global_state)
        std::cout<<i<<" ";
    std::cout<<std::endl;


    Model::State currState = {0., 0., 0.};
    float time_spend;
    for(size_t i = 0; i < pso.best_global_state.size(); i = i + 3)
    {
        // std::cout<< "RESULT PATH: " <<std::endl;
        Model::State Goal = {pso.best_global_state[i], pso.best_global_state[i+1], pso.best_global_state[i+2]};
        pso::run_to_goal(currState, Goal, dt, time_step, time_spend);
    }
    std::cout<< "RESULT POSITION: " <<std::endl;
    std::cout<<currState.x<<" "<<currState.y<<" "<<currState.yaw<<" "<<std::endl;


    // rostopic pub /clicked_point geometry_msgs/PointStamped
    ros::Rate rate(1.0/time_step); // Период 2 секунды (0.5 Гц)

    for(size_t i = 0; i < pso.best_global_state.size(); i = i + 3)
    {
        // std::cout<< "RESULT PATH: " <<std::endl;
        // Создание сообщения типа geometry_msgs::PointStamped
        geometry_msgs::PointStamped point_msg;
        point_msg.header.stamp = ros::Time::now();
        point_msg.header.frame_id = "map"; // или любое другое имя системы координат
        point_msg.point.x = pso.best_global_state[i];
        point_msg.point.y = pso.best_global_state[i+1];
        point_msg.point.z = pso.best_global_state[i+2];
        

        // Публикация сообщения
        point_pub.publish(point_msg);
        ROS_INFO("Published point: [%f, %f, %f]", point_msg.point.x, point_msg.point.y, point_msg.point.z);

        // Сон на 2 секунды
        rate.sleep();
        ros::spinOnce();
    }

    rate.sleep();
    ros::spinOnce();
    geometry_msgs::PointStamped point_msg;
    point_msg.header.stamp = ros::Time::now();
    point_msg.header.frame_id = "map"; // или любое другое имя системы координат
    point_msg.point.x = rosbot_state.x;
    point_msg.point.y = rosbot_state.y;
    point_msg.point.z = rosbot_state.yaw;
    
    point_pub.publish(point_msg);
    ROS_INFO("Published current point: [%f, %f, %f]", point_msg.point.x, point_msg.point.y, point_msg.point.z);



    // std::cout<<"q: ";
    // for(auto i : q)
    //     std::cout<<i<<" ";
    // std::cout<<std::endl<<"Result: ";
    // for(auto i : pso.best_global_state)
    //     std::cout<<i<<" ";
    // std::cout<<std::endl;

	return 0;
}