#include "ee4308_turtle/planner.hpp"

namespace ee4308::turtle
{

    // ======================== Nav2 Planner Plugin ===============================
    void Planner::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
        const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
        // initialize states / variables
        node_ = parent.lock(); // this class is not a node. It is instantiated as part of a node `parent`.
        tf_ = tf;
        plugin_name_ = name;
        costmap_ = costmap_ros->getCostmap();
        global_frame_id_ = costmap_ros->getGlobalFrameID();

        // initialize parameters
        initParam(node_, plugin_name_ + ".max_access_cost", max_access_cost_, 255);
        initParam(node_, plugin_name_ + ".interpolation_distance", interpolation_distance_, 0.05);
        initParam(node_, plugin_name_ + ".sg_half_window", sg_half_window_, 5);
        initParam(node_, plugin_name_ + ".sg_order", sg_order_, 3);
    }

    nav_msgs::msg::Path Planner::createPlan(
        const geometry_msgs::msg::PoseStamped &start,
        const geometry_msgs::msg::PoseStamped &goal)
    {
        // ========= initializations =========
        // Step 1
        OpenList open_list;
        // Step 2
        PlannerNodes nodes(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());

        int start_mx, start_my, goal_mx, goal_my;
        costmap_->worldToMapEnforceBounds(
            start.pose.position.x, start.pose.position.y,
            start_mx, start_my);
        costmap_->worldToMapEnforceBounds(
            goal.pose.position.x, goal.pose.position.y,
            goal_mx, goal_my);

        // Step 3
        PlannerNode *start_node = nodes.getNode(start_mx, start_my);
        start_node->g = 0;
        start_node->f = 0;
        // Step 4
        open_list.queue(start_node);

        // ========= A* Algorithm =========
        // Step 5
        while (!open_list.empty()){

            // Step 6
            PlannerNode *current = open_list.pop(); // get first node in the priority queue and removes it from the list
            if (!current) break; //maybe this if-loop is not neccessary?!

            // Step 7
            if (current->expanded) continue;

            // Step 8
            else if (current->mx == goal_mx && current->my == goal_my){
                std::vector<std::array<int, 2>> coords;
                while (current){
                    coords.push_back({current->mx, current->my});
                    current = current->parent;
                }

                std::reverse(coords.begin(), coords.end());
                
                // Convert to world coordinates
                std::vector<std::array<double, 2>> world_coords;
                for (const auto &coord : coords){
                    double wx, wy;
                    costmap_->mapToWorld(coord[0], coord[1], wx, wy);
                    world_coords.push_back({wx, wy});
                }

                // Apply Savitzky-Golay smoothing
                //world_coords = applySavitzkyGolaySmoothing(world_coords, sg_half_window_, sg_order_);

                return writeToPath(world_coords, goal);
            }

            // Step 9
            current->expanded = true;

            // Step 10
            static const std::array<std::array<int, 2>, 8> directions = {{{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {-1, -1}, {1, -1}, {-1, 1}}};
            for (const auto &dir : directions){                
                int nx = current->mx + dir[0];
                int ny = current->my + dir[1];
                PlannerNode *neighbor = nodes.getNode(nx, ny);
                if (!neighbor || neighbor->expanded) continue; // check if node is valid

                // Check the cost of the neighbor cell
                if (costmap_->getCost(nx, ny) >= max_access_cost_) continue; // Skip walls or obstacles

                // Step 11: Compute tentative g-cost
                double step_cost = std::hypot(dir[0], dir[1]);
                double tentative_g = current->g + step_cost * (costmap_->getCost(nx, ny) + 1);

                // Step 12: If new g-cost is lower, update node and queue it
                if (tentative_g < neighbor->g && costmap_->getCost(nx, ny) < max_access_cost_)
                {
                    neighbor->g = tentative_g;
                    neighbor->f = tentative_g + std::hypot(nx - goal_mx, ny - goal_my);
                    neighbor->parent = current;
                    open_list.queue(neighbor);
                }

            }

        }
               
        return nav_msgs::msg::Path();
    }


    // nav_msgs::msg::Path Planner::createPlan(
    //     const geometry_msgs::msg::PoseStamped &start,
    //     const geometry_msgs::msg::PoseStamped &goal)
    // {
    //     // initializations
    //     // 2. Initialize empty open-list.
    //     OpenList open_list;

    //     // 3. Initialize all nodes with ∞ -cost and no parent.
    //     PlannerNodes nodes(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());

    //     int start_mx, start_my, goal_mx, goal_my;
    //     costmap_->worldToMapEnforceBounds(
    //         start.pose.position.x, start.pose.position.y,
    //         start_mx, start_my);
    //     costmap_->worldToMapEnforceBounds(
    //         goal.pose.position.x, goal.pose.position.y,
    //         goal_mx, goal_my);

    //     // 4. Initialize start node with 0 g-cost.
    //     PlannerNode *start_node = nodes.getNode(start_mx, start_my);
    //     start_node->g = 0;
    //     start_node->f = 0;

    //     // 5. Queue start node into open-list.
    //     open_list.queue(start_node);

    //     // 6. While open-list not empty Do
    //     while (!open_list.empty()){
    //         // 7. n ← cheapest f-cost node polled from open-list.
    //         PlannerNode *current = open_list.pop();
    //         if (!current) break;

    //         // 8. If n was previously expanded Then
    //         if (current->expanded) continue;

    //         // 10. Else If n is at goal Then create the path
    //         else if (current->mx == goal_mx && current->my == goal_my){
    //             std::vector<std::array<int, 2>> coords;
    //             while (current){
    //                 coords.push_back({current->mx, current->my});
    //                 current = current->parent;
    //             }

    //             // reverse the path
    //             std::reverse(coords.begin(), coords.end());

    //             // Convert the path from map coordinates to world coordinates.
    //             std::vector<std::array<double, 2>> world_coords;
    //             for (const auto &coord : coords){
    //                 double wx, wy;
    //                 costmap_->mapToWorld(coord[0], coord[1], wx, wy);
    //                 world_coords.push_back({wx, wy});
    //             }
                
    //             // Apply Savitsky-Golay smoothing to the path.
    //             // world_coords = applySavitskyGolaySmoothing(world_coords, sg_half_window_, sg_order_);
    //             return writeToPath(coords, goal); // 
    //         }

    //         // 17 Mark n as expanded.
    //         current->expanded = true;

    //         // 18 For each accessible neighbor node m of n Do
    //         static const std::array<std::array<int, 2>, 8> directions = {{{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {-1, -1}, {1, -1}, {-1, 1}}};
    //         for (const auto &dir : directions){
    //             int nx = current->mx + dir[0];
    //             int ny = current->my + dir[1];
    //             PlannerNode *neighbor = nodes.getNode(nx, ny);
    //             if (!neighbor || neighbor->expanded) continue; // check if node is valid

    //             // Check the cost of the neighbor cell
    //             if (costmap_->getCost(nx, ny) >= max_access_cost_) continue; // Skip walls or obstacles

    //             // 20 If g~< g-cost of m Then g-cost of m ← g~parent of m ← n
    //             double step_cost = std::hypot(dir[0], dir[1]);
    //             double tentative_g = current->g + step_cost * (costmap_->getCost(nx, ny) + 1);

    //             // Step 12: If new g-cost is lower, update node and queue it
    //             if (tentative_g < neighbor->g && costmap_->getCost(nx, ny) < max_access_cost_)
    //             {
    //                 neighbor->g = tentative_g;
    //                 neighbor->f = tentative_g + std::hypot(nx - goal_mx, ny - goal_my);
    //                 neighbor->parent = current;
    //                 open_list.queue(neighbor);
    //             }

    //         }

    //     }

    //     return nav_msgs::msg::Path(); // return empty path if no path is found
    // }

    nav_msgs::msg::Path Planner::writeToPath(
        std::vector<std::array<double, 2>> coords,
        geometry_msgs::msg::PoseStamped goal)
    {
        nav_msgs::msg::Path path;
        path.poses.clear();
        path.header.frame_id = global_frame_id_;
        path.header.stamp = node_->now();

        for (const auto &coord : coords)
        { 
            // convert map coordinates to world coordiantes
            double wx, wy;
            costmap_->mapToWorld(coord[0], coord[1], wx, wy);
            
            // push the pose into the messages.
            geometry_msgs::msg::PoseStamped pose; // do not fill the header with timestamp or frame information. 
            pose.pose.position.x = wx;
            pose.pose.position.y = wy;
            pose.pose.orientation.w = 1; // normalized quaternion
            path.poses.push_back(pose);
        }
        
        // push the goal
        goal.header.frame_id = "";  // remove frame id to prevent incorrect transformations.
        goal.header.stamp = rclcpp::Time();  // remove timestamp from header, otherwise there will be time extrapolation issues.
        path.poses.push_back(goal);

        // return path;
        return path;
    }


    void Planner::cleanup()
    {
        RCLCPP_INFO_STREAM(node_->get_logger(), "Cleaning up plugin " << plugin_name_ << " of type ee4308::turtle::Planner");
    }

    void Planner::activate()
    {
        RCLCPP_INFO_STREAM(node_->get_logger(), "Activating plugin " << plugin_name_ << " of type ee4308::turtle::Planner");
    }

    void Planner::deactivate()
    {
        RCLCPP_INFO_STREAM(node_->get_logger(), "Deactivating plugin " << plugin_name_ << " of type ee4308::turtle::Planner");
    }


    // ====================== Planner Node ===================
    PlannerNode::PlannerNode(int mx, int my) : mx(mx), my(my) {}

    // ======================= Open List Implemetations ===========
    bool OpenListComparator::operator()(PlannerNode *l, PlannerNode *r) const { return l->f > r->f; }

    void OpenList::queue(PlannerNode *node) { pq.push(node); }

    PlannerNode *OpenList::pop()
    {
        if (pq.empty())
            return nullptr;
        PlannerNode *cheapest_node = pq.top();
        pq.pop();
        return cheapest_node;
    }

    bool OpenList::empty() const { return pq.empty(); }

    // ======================== Nodes ===============================
    PlannerNodes::PlannerNodes(int num_cells_x, int num_cells_y)
    {
        size_mx = num_cells_x;
        size_my = num_cells_y;

        nodes.reserve(num_cells_x * num_cells_y);
        for (int mx = 0; mx < size_mx; ++mx)
            for (int my = 0; my < size_my; ++my)
                nodes[mx * size_my + my] = PlannerNode(mx, my);
    }

    PlannerNode *PlannerNodes::getNode(int mx, int my)
    {
        if (mx < 0 || my < 0 || mx >= size_mx || my >= size_my)
            return nullptr;
        return &nodes[mx * size_my + my];
    }
}

PLUGINLIB_EXPORT_CLASS(ee4308::turtle::Planner, nav2_core::GlobalPlanner)