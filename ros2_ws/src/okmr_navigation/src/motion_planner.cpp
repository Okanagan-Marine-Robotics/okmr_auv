#include <string>
#include "rclcpp/rclcpp.hpp"
#include "okmr_msgs/msg/movement_command.hpp"
#include "okmr_msgs/msg/status.hpp"
#include "okmr_msgs/msg/goal_pose.hpp"
#include "okmr_msgs/msg/voxel_grid.hpp"
#include "okmr_msgs/msg/classes.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "okmr_msgs/srv/status.hpp"
#include "bonxai/bonxai.hpp"
#include "bonxai/serialization.hpp"
#include <tf2/LinearMath/Vector3.h>
#include "voxelData.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;
typedef okmr_msgs::msg::GoalPose gpose;

class MotionPlannerNode : public rclcpp::Node
{
    public:
        bool inserted=false;
        MotionPlannerNode() 
        : Node("motion_planner_node"), costmap(0.04){ 

        end_pose_subscription = this->create_subscription<okmr_msgs::msg::GoalPose>("/end_goal_pose", 10, std::bind(&MotionPlannerNode::end_pose_callback, this, _1));

        goal_pose_publisher = this->create_publisher<okmr_msgs::msg::GoalPose>("/current_goal_pose", 10);

        costmap_subscription = this->create_subscription<okmr_msgs::msg::VoxelGrid>("/voxel_grid", 10, std::bind(&MotionPlannerNode::costmap_callback, this, _1));

        current_pose_subscription = this->create_subscription<geometry_msgs::msg::PoseStamped>("/pose", 10, std::bind(&MotionPlannerNode::current_pose_callback, this, _1));

        gridPublisher = this->create_publisher<okmr_msgs::msg::VoxelGrid>("/path_grid", 10);//voxel grid used specifically for visualization of path

        status_service=this->create_service<okmr_msgs::srv::Status>("motion_planner_status", std::bind(&MotionPlannerNode::status_callback, this, std::placeholders::_1, std::placeholders::_2));

        subNode = rclcpp::Node::make_shared("_motion_planner_client");
        status_client=subNode->create_client<okmr_msgs::srv::Status>("motor_cortex_status");
    }
    private:
        void end_pose_callback(okmr_msgs::msg::GoalPose msg){
            //set end goal pose
            currentEndPoseMsg=msg;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "got path request");
            haveGoal=true;
            newGoal=true;
        }
        void current_pose_callback(geometry_msgs::msg::PoseStamped msg){
            //set current pose
            currentPoseMsg=msg;
            if(haveGoal){
                currentGoalPose=calculatePath();
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "calculated path");
                goal_pose_publisher->publish(currentGoalPose);
            }
        }

        okmr_msgs::srv::Status::Response sendStatusRequest(){  //TODO: fix names          
            auto request = std::make_shared<okmr_msgs::srv::Status::Request>();

            while (!status_client->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                }
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
            }

            auto result = status_client->async_send_request(request);
            // Wait for the result.
            if (rclcpp::spin_until_future_complete(subNode, result) ==
                rclcpp::FutureReturnCode::SUCCESS)
            {
                //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "got object location");
            } else {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service find_object");
            }
            return *result.get();
        }

        void status_callback(const std::shared_ptr<okmr_msgs::srv::Status::Request> request,
                                        std::shared_ptr<okmr_msgs::srv::Status::Response>      response)
        {
            auto status = sendStatusRequest();
            if(status.ongoing==false){
                response->ongoing=false;
                response->success=status.success;
            }
            else
                response->ongoing=true;

            if(failedSearch){
                response->ongoing=false;
                response->success=false;
            }
        }

        void costmap_callback(okmr_msgs::msg::VoxelGrid msg){
            replaceCostmap(msg);
            if(haveGoal){//check if there are any new obstacles from curr pose to current goal
                currentGoalPose=calculatePath();
                goal_pose_publisher->publish(currentGoalPose);
            }
        }

        bool replaceCostmap(okmr_msgs::msg::VoxelGrid msg){
            if(loading || searching)return false;
            loading=true;
            std::string serialized_data(msg.data.begin(), msg.data.end());

            std::istringstream ifile(serialized_data, std::ios::binary);
    
            char header[256];
            ifile.getline(header, 256);
            Bonxai::HeaderInfo info = Bonxai::GetHeaderInfo(header);
            auto g=Bonxai::Deserialize<voxelData>(ifile, info);
            costmap=std::move(g);
            loading=false;
            inserted=true;
            return true;
        }
        class node{
            public:
            float x,y,z,dist,cost;
            bool operator==(const node& n) const{
                return abs(x - n.x)<0.1 && abs(y - n.y)<0.1 && abs(z - n.z)<0.1;
            }
            bool operator<(const node& n) const{
                return (cost+dist)<(n.cost+n.dist);
            }

        };
        
        struct nodeHash {
            std::size_t operator()(const node& s) const {
                std::size_t hashX = std::hash<float>()(s.x);
                std::size_t hashY = std::hash<float>()(s.y);
                std::size_t hashZ = std::hash<float>()(s.z);

                // Combine hash values using bitwise XOR and multiplication
                return hashX ^ (hashY << 1) ^ (hashZ << 2);
            }
        };
        
        struct nodeCompare {
            bool operator()(const node& n1, const node& n2) const { return (n1.cost+n1.dist)>(n2.cost+n2.dist); }
        };

        float dist(node n1, node n2){
            return pow(
                        pow(n1.x-n2.x,2)+
                        pow(n1.y-n2.y,2)+
                        pow(n1.z-n2.z,2)
                        ,1.0/2.0);
        }

        void clear_path_nodes(std::vector<node> pathNodes){
            auto accessor=costmap.createAccessor();
            for(node n: pathNodes){
                Bonxai::CoordT coord = costmap.posToCoord(n.x, n.y, n.z);
                accessor.setCellOff( coord );
                //accessor.setValue( coord, 0.0 );
            }
        }

        void publishVoxelGrid(){//for visualization and debugging
            std::ostringstream ofile(std::ios::binary);
            Bonxai::Serialize(ofile, costmap);
            std::string s=ofile.str();
    
            okmr_msgs::msg::VoxelGrid gridMsg;

            std::vector<unsigned char> charVector(s.begin(), s.end());
            gridMsg.data=charVector;

            gridPublisher->publish(gridMsg);
        }

        bool isValidNode(voxelData* pointer){
            if(pointer==nullptr)return true;
            //if((*pointer)[0]==okmr_msgs::msg::Classes::INFLATED)return false;//node is valid to be travelled to if its part of the costmap or if its empty
            return false;
        }

        bool isDangerousNode(node n, Bonxai::VoxelGrid<voxelData>::Accessor accessor){
            float radius=0.6;
            for(float x=n.x-radius;x<n.x+radius;x+=2*costmap.resolution){
                for(float y=n.y-radius;y<n.y+radius;y+=2*costmap.resolution){
                    for(float z=n.z-radius;z<n.z+radius;z+=2*costmap.resolution){
                            float squaredDist = (x - n.x) * (x - n.x) +
                                (y - n.y) * (y - n.y) +
                                (z - n.z) * (z - n.z);
                        if (squaredDist <= radius*radius) {
                            if (accessor.value(costmap.posToCoord(x, y, z))) {//if the cell is occupied 
                                return true;
                            }
                        }
                    }
                }
            }
            return false;
        }

        std::vector<node> aStarSearch(){
            std::vector<node> result;
            failedSearch=false;
            auto accessor=costmap.createAccessor();
            //RCLCPP_INFO(this->get_logger(), "starting a*");
            node start,goal;
            start.x = currentPoseMsg.pose.position.x;
            start.y = currentPoseMsg.pose.position.y;
            start.z = -currentPoseMsg.pose.position.z;
            //start.x=0;
            //start.y=0;
            //start.z=0;
            goal.x = currentEndPoseMsg.pose.position.x;
            goal.y = currentEndPoseMsg.pose.position.y;
            goal.z = -currentEndPoseMsg.pose.position.z;//why does this need to be negative i have no clue

            voxelData *goal_value = accessor.value(costmap.posToCoord(goal.x, goal.y, goal.z)), 
                    *start_value = accessor.value(costmap.posToCoord(start.x, start.y, start.z));
            if(!isValidNode(goal_value) || !isValidNode(start_value)){
                failedSearch=true;
                return result;
            }
            //if goal is inside an obstacle dont even try

            std::vector<node> surrounding;
            std::unordered_set<node, nodeHash> checked;
            std::unordered_map<node, node, nodeHash> previous;
            std::priority_queue<node, std::vector<node>, nodeCompare> openSet;
            start.cost=0;
            start.dist=dist(start,goal);
            openSet.push(start);
            checked.insert(start);
            bool reachable=true;

            while(!openSet.empty() && reachable){
                node current=openSet.top();
                if(current==goal){

                    //RCLCPP_INFO(this->get_logger(), "found goal");
                    while(previous.count(current)>0){
                        result.insert(result.begin(),current);
                        current=previous[current];
                    }
                    //std::string logMessage = "returning with " + std::to_string(result.size()) + " path nodes";
                    //RCLCPP_INFO(this->get_logger(), logMessage.c_str());
                    return result;
                }
                openSet.pop();
                //checked.erase(current);
                //creating set of neighboring nodes
                for(int x=-1;x<=1;x++){
                    for(int y=-1;y<=1;y++){
                        for(int z=-1;z<=1;z++){
                            node tempNode;
                            tempNode.x=current.x+x*costmap.resolution*0.95;
                            tempNode.y=current.y+y*costmap.resolution*0.95;
                            tempNode.z=current.z+z*costmap.resolution*0.95;
                            Bonxai::CoordT coord = costmap.posToCoord(tempNode.x, tempNode.y, tempNode.z);
                            voxelData* value_ptr = accessor.value( coord );
                            if(isValidNode(value_ptr)){
                                if(checked.count(tempNode)>0){
                                    tempNode=*checked.find(tempNode);
                                }else
                                    tempNode.cost=std::numeric_limits<float>::infinity();
                                surrounding.push_back(tempNode);
                            }
                        }
                    }
                }
                for(node n: surrounding){
                    float score=current.cost+dist(current,n);
                    //std::array<int, 2>* value_ptr = accessor.value(costmap.posToCoord(n.x, n.y, n.z));
                    if(isDangerousNode(n, accessor))
                        score+=100*dist(current,n);//to discourage using dangerous nodes, but allowing it if really needed
                    if(score<n.cost){
                        previous[n]=current;
                        n.cost=score;
                        n.dist=dist(n,goal)*dist(n,goal)*50;
                        //if(dist(n,goal)>dist(start,goal)*2)//if we are exploring nodes way far out, terminate to stop infinite looping
                            //reachable=false;//this might trigger early if goal and start are close
                        if(checked.count(n)==0){
                            checked.insert(n);
                            openSet.push(n);
                            //result.push_back(n);
                        }
                    }
                }
                surrounding.clear();
                //std::string logMessage = "explored nodes " + std::to_string(checked.size());
                //RCLCPP_INFO(this->get_logger(), logMessage.c_str());
            }
            return result;
        }
    float start2EndError(std::vector<node> nodes) {
        float result = 0;
        if (nodes.size() < 2) {
            return result; // Not enough nodes to calculate error
        }

        tf2::Vector3 line_point1(nodes.front().x, nodes.front().y, nodes.front().z);
        tf2::Vector3 line_point2(nodes.back().x, nodes.back().y, nodes.back().z);
    
        for (const node& n : nodes) {
            tf2::Vector3 point(n.x, n.y, n.z);
            tf2::Vector3 line_direction = line_point2 - line_point1;

        // Calculate the vector from one of the points on the line to the separate point
            tf2::Vector3 vector_to_point = point - line_point1;

            // Calculate the squared magnitude of the direction vector
            double mag_squared = line_direction.length2();
            if (mag_squared == 0) {
                continue; // Avoid division by zero
            }

            // Calculate the dot product
            double dot_product = vector_to_point.dot(line_direction);

            // Calculate the projection of vector_to_point onto the direction vector of the line
            tf2::Vector3 projection = (dot_product / mag_squared) * line_direction;

            // Calculate the vector from the original point to the projected point
            tf2::Vector3 distance_vector = point - (line_point1 + projection);

            // Calculate the magnitude of the distance vector
            double distance = distance_vector.length();
            if (!std::isnan(distance)) {
                result += distance;
            }
        }
        return result;
    }

        gpose calculatePath(){
            gpose result=gpose();
            if(searching || loading)return result;
            auto accessor=costmap.createAccessor();
            searching=true;
            std::vector<node> path=aStarSearch();
            if(path.size()>1){
                size_t last=0,current=0;
                float threshold=0.35;

                //RCLCPP_INFO(this->get_logger(), "starting node -> pose");
                while(start2EndError(std::vector<node>(path.begin()+current,path.begin()+last+1))<threshold && last<path.size()-1){
                    last++;
                }
                //std::string logMessage = "Error :" + std::to_string(start2EndError(std::vector<node>(path.begin()+current,path.begin()+last)));
                //RCLCPP_INFO(this->get_logger(), logMessage.c_str());
                current=last;
                voxelData vd = voxelData(path[current].x, 
                                        path[current].y, 
                                        path[current].z,
                                        2, 100, 0,255,0);
                accessor.setValue(costmap.posToCoord(path[current].x,path[current].y,path[current].z), vd );
                gpose temp=gpose();
                temp.copy_orientation=false;
                temp.pose.position.x=path[current].x;
                temp.pose.position.y=path[current].y;
                temp.pose.position.z=-path[current].z;
                result=temp;
            }
            else if(path.size()==1){
                result=currentEndPoseMsg;
            }
            else{
                gpose temp=gpose();
                temp.copy_orientation=true;
                temp.pose=currentPoseMsg.pose;//if already at goal or failure to calculate path, then just stay where it is
                result=temp;
            }
            for(node n: path){ 
                voxelData vd = voxelData(n.x, 
                                        n.y, 
                                        n.z,
                                        1, 100, 255,0,0);
                accessor.setCellOn(costmap.posToCoord(n.x,n.y,n.z), vd);
            }
            
            //insert the path nodes into the costmap and publish under /path_grid
            publishVoxelGrid();
            clear_path_nodes(path);
            searching=false;
            return result;
        }

        bool loading=false, haveGoal=false, searching=false, newGoal=true, failedSearch=false;
        Bonxai::VoxelGrid<voxelData> costmap;
        gpose currentGoalPose;
        rclcpp::Publisher<okmr_msgs::msg::VoxelGrid>::SharedPtr gridPublisher;
        geometry_msgs::msg::PoseStamped currentPoseMsg;
        okmr_msgs::msg::GoalPose currentEndPoseMsg;
        rclcpp::Subscription<okmr_msgs::msg::GoalPose>::SharedPtr end_pose_subscription;
        rclcpp::Subscription<okmr_msgs::msg::VoxelGrid>::SharedPtr costmap_subscription;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr  current_pose_subscription;
        rclcpp::Publisher<okmr_msgs::msg::GoalPose>::SharedPtr goal_pose_publisher;
        rclcpp::Service<okmr_msgs::srv::Status>::SharedPtr status_service;
        std::shared_ptr<rclcpp::Node> subNode;
        rclcpp::Client<okmr_msgs::srv::Status>::SharedPtr status_client;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotionPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
