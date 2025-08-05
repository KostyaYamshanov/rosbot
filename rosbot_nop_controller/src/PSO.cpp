#include "PSO.hpp"
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PointStamped.h"
#include "gazebo_msgs/ModelStates.h"

namespace pso
{

std::vector<Model::State> obstacles = {
    {1.25, 1.25, 0}, // Пример препятствия
};

bool is_near_obstacle(Model::State& state, const std::vector<Model::State>& obstacles, float threshold = 0.25) {
    for (const auto& obs : obstacles) {
        auto point = Model::State(obs);
        if (state.distXY(point) < threshold) {
            return true;
        }
    }
    return false;
}


float run_to_goal(Model::State& currState, 
                const Model::State& Goal, 
                const float dt,  // Фиксированный шаг управления
                float& available_time,
                float& time_step_limit,  // Лимит времени для этого отрезка (передается по ссылке)
                float& time_spent,
                float& obs_cost
                ) 
{
    NetOper nop = NetOper();
    nop.setLocalTestsParameters();

    Model model(currState, dt);
    Controller controller(Goal, nop);

    Runner runner(model, controller); 
    runner.init(currState);
    runner.setGoal(Goal);

    float time_used = 0.0f;
    bool goal_reached = false;
    while (time_used < time_step_limit && available_time > 0)
    {
        currState = runner.makeStep();
        time_used += dt;
        available_time -= dt;
        time_spent += dt;

        const float obstacle_penalty = 15.0f;

        if (is_near_obstacle(currState, obstacles)) {
            obs_cost += obstacle_penalty;
        }

        if (currState.dist(Goal) < EPS) {
            goal_reached = true;
            break;
        }
    }

    return time_used;
}

Particle::Particle(const std::vector<float>& initial_state, Model::State main_goal, const float time_step, const float dt, const float t_max):
main_goal(main_goal), time_step(time_step), dt(dt), Tmax(t_max)
{
    N = initial_state.size();
    curr_state = initial_state;
    best_state = initial_state;
    velocities.resize(N);

    for(auto& v : velocities) {
        v = (float)rand() / ((float)RAND_MAX + 0.05);
    }
}

float Particle::CostFunction() {
    Model::State initial_state = {0., 0., 0.};  // Стартовая позиция
    float available_time = Tmax;                // Общий доступный бюджет времени
    float time_spent = 0.0f;                    // Суммарное затраченное время
    float segment_time_limit = time_step;       // Локальная переменная для адаптации лимитов
    const float distance_weight = 1.0f;         // Вес ошибки позиционирования
    const float time_weight = 0.25f;             // Вес временной ошибки
    float obs_cost = 0.;
    // Обработка промежуточных точек
    for (size_t i = 0; i < curr_state.size(); i += 3) {
        if (available_time <= 0) break;
        
        Model::State goal = {
            curr_state[i], 
            curr_state[i+1], 
            curr_state[i+2]
        };
        
        // Вызов с адаптируемым лимитом времени для отрезка
        run_to_goal(initial_state, goal, dt, available_time, segment_time_limit, time_spent, obs_cost);
    }

    // Финализация движения к основной цели
    // if (available_time > 0) {
    //     available_time = 1.0; // TODO TEST
    //     run_to_goal(initial_state, main_goal, dt, available_time, segment_time_limit, time_spent, obs_cost);
    // }


    if (initial_state.dist(main_goal) < EPS) {
    // Если достигли цели быстрее - уменьшаем лимит для следующих отрезков
        time_step = float(time_spent /  float(float(curr_state.size())/3.0));
        Tmax = time_spent;
        std::cout<<"time_spent: "<<time_spent<<std::endl;
        std::cout<<"time_step: "<<time_step<<std::endl;
        std::cout<<"new Tmax: "<<Tmax<<std::endl;
    } 

    // Расчет ошибок
    float position_error = initial_state.distXY(main_goal) * 10.0;
    float time_error = std::max(-available_time, 0.0f);  // Отрицательное время = опоздание
    std::cout<<"obs_cost = "<<obs_cost<<std::endl;
    // Комбинированная функция стоимости
    return distance_weight * position_error + 
           time_weight * (time_error + time_spent/Tmax) + obs_cost;
}

void Particle::evaluate() {
    curr_error = CostFunction();
    if (curr_error <= best_error) {
        best_state = curr_state;
        best_error = curr_error;
    }
}

void Particle::update_velocities(std::vector<float> best_global_state) {
    float w=1.0;
    float c1=0.65;
    float c2=0.65;
    
    for(size_t i = 0; i < N; ++i){
        float r1 = (float)rand() / ((float)RAND_MAX + 0.1);
        float r2 = (float)rand() / ((float)RAND_MAX + 0.1);
        float vel_cognitive = c1 * r1 * (best_state[i] - curr_state[i]);
        float vel_social = c2 * r2 * (best_global_state[i] - curr_state[i]);
        velocities[i] = w * velocities[i] + vel_cognitive + vel_social;
    }
}

void Particle::update_state() {
    for (size_t i = 0; i < N; ++i) {
        curr_state[i] += velocities[i];
    }
}

PSO::PSO(std::vector<float> initial_state, size_t numParticles, size_t maxIter, Model::State main_goal, float time_step, float dt, float t_max): 
    maxIter_(maxIter), dt_(dt), time_step_(time_step), main_goal_(main_goal)
{   
    best_global_state_ = initial_state;
    swarm_.resize(numParticles);
    for(size_t i = 0; i < swarm_.size(); ++i)
        swarm_[i] = Particle(initial_state, main_goal, time_step, dt, t_max);

    fit();
}

std::vector<float> PSO::fit() {
    for(size_t i = 0; i < maxIter_; ++i) {  
        for (auto& p : swarm_) {
            p.evaluate();
            if (p.curr_error < best_global_error_) {
                best_global_state_ = p.curr_state;
                best_global_error_ = p.curr_error;
                best_particle_ = p;
            }
            p.update_velocities(best_global_state_);
            p.update_state();
        }
    }
    return best_global_state_;
}

};

Model::State rosbot_state{};

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

int main(int argc, char **argv) {   
    ros::init(argc, argv, "PSO");
    ros::NodeHandle nh;
    ros::Publisher point_pub = nh.advertise<geometry_msgs::PointStamped>("/clicked_point", 10);
    ros::Subscriber models_sub = nh.subscribe("/odom", 1, model_state_cb);

    float time_step = 2; 
    float dt = 0.01;
    size_t numParticles = 50;
    size_t maxIter = 150;
    float t_max = 8;
    Model::State main_goal = {2., 2., 0.};
    size_t N = size_t(t_max / time_step) * 3;
    // std::vector<float> q = {0.,0.,0., 0.,0.,0., 0.,0.,0.};
    std::vector<float> q = std::vector<float>(N, 0.0);
    
    auto pso = pso::PSO(q, numParticles, maxIter, main_goal, time_step, dt, t_max);

    Model::State currState = {0., 0., 0.};
    float time_spent = 0.0f;
    float obs_cost = 0.0f;
    for(size_t i = 0; i < pso.best_global_state_.size(); i += 3) {
        Model::State Goal = {pso.best_global_state_[i], pso.best_global_state_[i+1], pso.best_global_state_[i+2]};
        float available_time = pso.best_particle_.Tmax - time_spent;
        float current_dt = pso.best_particle_.dt;
        float optimal_time_step = pso.best_particle_.time_step;
        pso::run_to_goal(currState, Goal, current_dt, available_time, time_step, time_spent, obs_cost);
    }

    std::cout<<"q: ";
    for(auto i : q)
        std::cout<<i<<" ";
    std::cout<<std::endl<<"Result: ";
    for(auto i : pso.best_global_state_)
        std::cout<<i<<" ";
    std::cout<<std::endl;
    std::cout<<"currState -> "<<currState.x<<" "<<currState.y<<std::endl;



    ros::Rate rate(1.0/pso.best_particle_.time_step);
    for(size_t i = 0; i < pso.best_global_state_.size(); i += 3) {
        geometry_msgs::PointStamped point_msg;
        point_msg.header.stamp = ros::Time::now();
        point_msg.header.frame_id = "map";
        point_msg.point.x = pso.best_global_state_[i];
        point_msg.point.y = pso.best_global_state_[i+1];
        point_msg.point.z = pso.best_global_state_[i+2];
        point_pub.publish(point_msg);
        ROS_INFO("Published point: [%f, %f, %f]", point_msg.point.x, point_msg.point.y, point_msg.point.z);
        rate.sleep();
        ros::spinOnce();
    }

    geometry_msgs::PointStamped point_msg;
    point_msg.header.stamp = ros::Time::now();
    point_msg.header.frame_id = "map";
    point_msg.point.x = main_goal.x;
    point_msg.point.y = main_goal.y;
    point_msg.point.z = main_goal.yaw;
    point_pub.publish(point_msg);
    ROS_INFO("Published current point: [%f, %f, %f]", point_msg.point.x, point_msg.point.y, point_msg.point.z);
    std::cout<<"Tmax: "<<pso.best_particle_.Tmax<<std::endl;
    return 0;
}
