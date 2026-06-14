#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>

#include <jsoncpp/json/json.h>

#include <fstream>
#include <cmath>
#include <vector>
#include <string>
#include <map>

struct AgentData
{
    std::string name;                    // "robot1"
    std::string group;                   // "irb"
    std::vector<std::string> joint_names;// exact MoveIt joint names
    std::vector<double> times;           // t0, t1, t2...
    std::vector<std::vector<double>> positions;  // joint arrays
};

// ---------------------------------------------------------------
int main(int argc, char** argv)
{
    ros::init(argc, argv, "trajectory_visualizer");
    ros::NodeHandle nh("~");

    // ===== Read parameters =====
    double delta_time = 1.0;
    double rate_scale = 1.0;

    nh.param("delta_time", delta_time, 1.0);
    nh.param("rate_scale", rate_scale, 1.0);

    std::string folder = "default_config";
    std::string file   = "trajectory.json";
    nh.param("output_folder", folder, folder);
    nh.param("output_file",   file,   file);

    std::string parent_group = "arms";
    nh.param("parent_group", parent_group, parent_group);

    // ===== Load agents from YAML =====
    XmlRpc::XmlRpcValue agents_yaml;
    if (!nh.getParam("agents", agents_yaml))
    {
        ROS_ERROR("trajectory_visualizer: Failed to load 'agents' YAML array.");
        return 1;
    }

    std::vector<AgentData> agents;

    for (int i = 0; i < agents_yaml.size(); ++i)
    {
        AgentData A;
        A.name = static_cast<std::string>(agents_yaml[i]["name"]);
        A.group = static_cast<std::string>(agents_yaml[i]["move_group"]);

        // load joint names
        XmlRpc::XmlRpcValue jn = agents_yaml[i]["joint_names"];
        for (int j = 0; j < jn.size(); ++j)
            A.joint_names.push_back(static_cast<std::string>(jn[j]));

        agents.push_back(A);
    }

    ROS_INFO("Loaded %zu agents from YAML.", agents.size());

    // ===== Build full JSON path =====
    std::string pkg_path = ros::package::getPath("rhcr_mapf_planners");
    std::string json_path = pkg_path + "/output/" + folder + "/" + file;

    ROS_INFO_STREAM("Loading trajectory file: " << json_path);

    // ===== Load JSON file =====
    std::ifstream ifs(json_path);
    if (!ifs.is_open())
    {
        ROS_ERROR("Failed to open trajectory JSON file.");
        return 1;
    }

    Json::Value root;
    ifs >> root;

    // ===== Parse "agents" array in JSON =====
    const Json::Value json_agents = root["agents"];

    if (json_agents.size() != agents.size())
    {
        ROS_WARN("JSON agent count (%u) != YAML agent count (%zu).",
                 json_agents.size(), agents.size());
    }

    for (unsigned int i = 0; i < json_agents.size() && i < agents.size(); ++i)
    {
        const Json::Value traj = json_agents[i]["trajectory"];

        for (unsigned int k = 0; k < traj.size(); ++k)
        {
            double t = traj[k]["time"].asDouble();
            agents[i].times.push_back(t);

            const Json::Value pos = traj[k]["positions"];
            std::vector<double> P;
            for (unsigned int m = 0; m < pos.size(); ++m)
                P.push_back(pos[m].asDouble());

            agents[i].positions.push_back(P);
        }
    }

    // ===== Compute global timeline =====
    double global_end = 0.0;
    for (auto &A : agents)
        if (!A.times.empty())
            global_end = std::max(global_end, A.times.back());

    int total_steps = static_cast<int>(std::ceil(global_end / delta_time));
    ROS_INFO("Total visualization steps = %d", total_steps);

    // ===== Flatten parent group's joint order =====
    // Based on known concatenation order:
    // [agent1 joints..., agent2 joints..., ...]
    std::vector<std::string> full_joint_order;
    for (auto &A : agents)
        for (auto &jn : A.joint_names)
            full_joint_order.push_back(jn);

    // ===== Prepare publisher =====
    ros::Publisher pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
    ros::Rate rate((1.0/delta_time) * rate_scale);

    // ===== Visualization playback loop =====
    for (int step = 0; step <= total_steps && ros::ok(); ++step)
    {
        double t = step * delta_time;

        sensor_msgs::JointState js;
        js.header.stamp = ros::Time::now();

        // Build full multi-agent joint vector
        for (auto &A : agents)
        {
            size_t idx = 0;

            if (t <= A.times.front()) {
                idx = 0;
            } else if (t >= A.times.back()) {
                idx = A.times.size() - 1;
            } else {
                for (size_t q = 0; q < A.times.size(); ++q)
                    if (A.times[q] >= t) { idx = q; break; }
            }

            // append joint data in correct order
            for (size_t j = 0; j < A.joint_names.size(); ++j)
            {
                js.name.push_back(A.joint_names[j]);
                js.position.push_back(A.positions[idx][j]);
            }
        }

        pub.publish(js);
        rate.sleep();
    }

    ROS_INFO("Trajectory visualization completed.");
    return 0;
}
