// This file is based on work by Franz Pucher, Diffbot, (2020), GitHub repository, https://github.com/fjp/diffbot
#include <rclcpp/duration.hpp>
#include <control_toolbox/pid.hpp>
#include <boost/thread/mutex.hpp>

#include "rclcpp/rclcpp.hpp"

class PID : public control_toolbox::Pid
{
    public:
        /**
         * @brief Construct a new PID object
         * 
         * @param p The proportional gain.
         * @param i The integral gain.
         * @param d The derivative gain.
         * @param i_max The max integral windup.
         * @param i_min The min integral windup.
         * @param out_min The min computed output.
         * @param out_max The max computed output.
         */
        PID(double p=0.0, double i=0.0, double d=0.0, double i_max=0.0, double i_min=0.0, bool antiwindup=false, double out_max=0.0, double out_min=0.0);

        /**
         * @brief Initialize the 
         * 
         * @param kF The feed forward gain.
         * @param kP The proportional gain.
         * @param kI The integral gain.
         * @param kD The derivative gain.
         * @param i_max The max integral windup.
         * @param i_min The min integral windup.
         * @param antiwindup 
         * @param out_min The min computed output.
         * @param out_max The max computed output.
         */
        void init(double f, double p, double i, double d, double i_max, double i_min, bool antiwindup, double out_max, double out_min);

        /**
         * @brief Compute PID output value from error using process value, set point and time period
         * 
         * @param measured_value The process value measured by sensors.
         * @param setpoint The desired target value.
         * @param dt The delta time or period since the last call.
         * @return double Computed PID output value.
         */
        double operator()(const double &measured_value, const double &setpoint, const rclcpp::Duration &dt);

        /**
         * @brief Get the FPID parameters
         * 
         * @param f The feed forward gain.
         * @param p The proportional gain.
         * @param i The integral gain.
         * @param d The derivative gain.
         * @param i_max The max integral windup.
         * @param i_min The min integral windup.
         * @param antiwindup Enable or disable antiwindup check.
         */
        void getParameters(double &f, double &p, double &i, double &d, double &i_max, double &i_min);
        void getParameters(double &f, double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup);

        /**
         * @brief Set the Parameters using the Gains object of control_toolbox::Pid
         * 
         * @param f The feed forward gain.
         * @param p The proportional gain.
         * @param i The integral gain.
         * @param d The derivative gain.
         * @param i_max The max integral windup.
         * @param i_min The min integral windup.
         * @param antiwindup Enable or disable antiwindup check.
         */
        void setParameters(double f, double p, double i, double d, double i_max, double i_min, bool antiwindup = false);

        /**
         * @brief Set the Output Limits of the PID controller
         * 
         * @param out_min 
         * @param out_max 
         */
        void setOutputLimits(double out_min, double out_max);

        /**
         * @brief Clam given value to upper and lower limits.
         * 
         * @param value Input value that's possibly clamped.
         * @param lower_limit Lower limit which the value must not exceed.
         * @param upper_limit Upper limit which the value must not exceed.
         * @return double Clamped value to range in between [lower_limit, upper_limit].
         */
        double clamp(const double& value, const double& lower_limit, const double& upper_limit);

        /**
         * @brief Get the current error.
         * 
         * @return double The current error computed from the measured and target value.
         */
        inline double getError() const { return error_; };

        /**
         * @brief Start the dynamic reconfigure node and load the default values
         * @param node - a node handle where dynamic reconfigure services will be published
         */
       // void initDynamicReconfig(ros::NodeHandle &node);

        /**
         * @brief Set Dynamic Reconfigure's gains to PID's values
         */
        //void updateDynamicReconfig();
        //void updateDynamicReconfig(Gains gains_config);
        //void updateDynamicReconfig(hoverboard_driver::HoverboardConfig config);

        /**
         * \brief Update the PID parameters from dynamics reconfigure
         */
        //void dynamicReconfigCallback(hoverboard_driver::HoverboardConfig &config, uint32_t /*level*/);

    private:
        double f_;
        double error_;
        double out_min_;
        double out_max_;
        rclcpp::Node::SharedPtr node_;

        // Dynamic reconfigure
        bool dynamic_reconfig_initialized_;
       // typedef dynamic_reconfigure::Server<hoverboard_driver::HoverboardConfig> DynamicReconfigServer;
        //boost::shared_ptr<DynamicReconfigServer> param_reconfig_server_;
        //DynamicReconfigServer::CallbackType param_reconfig_callback_;

      //  boost::recursive_mutex param_reconfig_mutex_;
};
