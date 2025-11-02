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

        // Build and the Vandermonde matrix kernel for Savitzky-Golay smoothing
        buildVandermondeMatrix();
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
                
                // Apply Savitzky-Golay smoothing
                coords = applySavitzkyGolaySmoothing(coords);

                return writeToPath(coords, goal);
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


    nav_msgs::msg::Path Planner::writeToPath(
        std::vector<std::array<int, 2>> coords,
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

    std::vector<std::array<int, 2>> Planner::applySavitzkyGolaySmoothing(
        std::vector<std::array<int, 2>> coords)
    {
        // // initialize
        int m = sg_half_window_;
        const int nPoints = coords.size();
        std::vector<std::array<int, 2>> smoothed_coords;
        smoothed_coords.reserve(nPoints);

        // // inverse via solver -> more stable
        // Eigen::MatrixXd JTJ = J.transpose() * J;
        // // LDLT or PartialPivLU or other decomposition can be used:
        // Eigen::LDLT<Eigen::MatrixXd> ldlt(JTJ);
        // // Solve for (J^T * J) * X = J^T using the decomposition
        // Eigen::MatrixXd A_solver = ldlt.solve(J.transpose());


        // extract first row of of J_kernel
        Eigen::VectorXd kernel = vander_matrix_first_kernel_row;

        std::cout << "vannder_matrix_first_kernel_row in time calc: ";
        for (int i = 0; i < kernel.size(); ++i){
            std::cout << kernel(i) << " ";
        }

        // apply kernel to coords
        for (int i = 0; i < nPoints; ++i)
        {
            double x = 0.0;
            double y = 0.0;

            for (int j = -m; j <= m; ++j)
            {
                // clamp k [0, nPoints-1]
                int k = i + j;
                if (k < 0) k = 0;
                if (k >= nPoints) k = nPoints - 1;

                x += kernel(j + m) * coords[k][0];
                y += kernel(j + m) * coords[k][1];
            }

            // round to int to fit into other functions
            smoothed_coords.push_back({
                static_cast<int>(std::round(x)), 
                static_cast<int>(std::round(y))
            });
        }

        std::cout << "Savitzky-Golay smoothing applied." << std::endl;
        std::cout << "original coords: " << coords.size() << std::endl;
        std::cout << "smoothed coords: " << smoothed_coords.size() << std::endl;

        return smoothed_coords;
    }

    Eigen::MatrixXd Planner::buildVandermondeMatrix()
    {
        // initialize
        int m = sg_half_window_;
        int n = sg_order_;

        Eigen::MatrixXd J(2*m+1, n+1);

        // fill vandermonde matrix
        for (int r = 0; r < 2*m+1; ++r){
            double x = r - m;
            for (int c = 0; c < n+1; ++c){
                J(r, c) = std::pow(x, c); // works because r and c are one smaler than in formula
            }
        }

        // using pseudo inverse in case J^T * J is close to singular
        Eigen::MatrixXd J_kernel = (J.transpose() * J).completeOrthogonalDecomposition().pseudoInverse() * J.transpose();

        std::cout << "J_kernel matrix:\n" << J_kernel << std::endl;


        vander_matrix_first_kernel_row = J_kernel.row(0);

        // proposed improvements using hann-square window weights applied on the kernel
        std::vector<double> hannSquareWindow(2*m+1);
        for (int n = 0; n < 2*m+1; ++n) {
            double hann = 0.5 * (1 - std::cos(2 * M_PI * n / (2*m)));
            hannSquareWindow[n] = hann * hann;
        }

        for (int i = 0; i < 2*m+1; ++i) {
            vander_matrix_first_kernel_row[i] = hannSquareWindow[i] * vander_matrix_first_kernel_row[i];
        }

        return vander_matrix_first_kernel_row;
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