/************************************************************************
*
* @author Alonso Marco
* Contact: amarco@berkeley.edu
************************************************************************/

#ifndef _DATA_PARSER_
#define _DATA_PARSER_

#include <ros/ros.h>
#include <string>
#include <eigen3/Eigen/Core>

/*

ros::NodeHandle Class Reference
http://docs.ros.org/en/melodic/api/roscpp/html/classros_1_1NodeHandle.html

ros::this_node::getName()
ros::NodeHandle::resolveName
http://wiki.ros.org/roscpp/Overview/Names%20and%20Node%20Information#Manipulating_Names

*/


/* 	We need to have the entire implementation into data_parser.hpp due to the presence of the template. Otheriwse, 
	when linking against position_holder.cpp, the explicit usages of the template won't be understood.
	See https://stackoverflow.com/questions/8752837/undefined-reference-to-template-class-constructor
*/

class DataParser{

public:
    DataParser(const ros::NodeHandle & nh): nh_(nh){
    	this->clean_format = Eigen::IOFormat(4, 0, ", ", "\n", "[", "]");
    }

    ~DataParser(){
        std::cout << "Destroying DataParser class ...\n";
    }

	template<typename T>
	void get(const std::string & var_name, T & var_out);

	void get_vec(const std::string & vec_name, Eigen::VectorXd & vec_out);

private:
	Eigen::IOFormat clean_format;
    const ros::NodeHandle nh_;
	void resolve_full_parameter_name(const std::string & var_name, std::string & name_param_full);
};


void DataParser::resolve_full_parameter_name(const std::string & var_name, std::string & name_param_full){

    std::string node_name = ros::this_node::getName();
    // std::cout << "this_node: " << this_node << "\n";

    // Resolve a name using the NodeHandle's namespace. 
    // It returns ns+var_name; ns="/" by default, can be overridden in the launch file
    std::string resolved_name = this->nh_.resolveName(var_name); 
    // std::cout << "resolved_name: " << resolved_name << "\n";

    name_param_full = node_name + resolved_name;

    return;
}

template<typename T>
void DataParser::get(const std::string & var_name, T & var_out){

	std::string name_param_full;
	this->resolve_full_parameter_name(var_name,name_param_full);

    if(this->nh_.getParam(name_param_full,var_out)){
        std::cout << " * " << name_param_full << ": " << var_out << "\n";
    }
    else{
        std::cout <<  " * " << name_param_full << " failed to load because it doesn't exist...\n";
    }

    return;
}


void DataParser::get_vec(const std::string & vec_name, Eigen::VectorXd & vec_out){

	std::string name_param_full;
	this->resolve_full_parameter_name(vec_name,name_param_full);
    
    std::vector<double> vec_doubles;
    if(this->nh_.getParam(name_param_full,vec_doubles)){
        vec_out = Eigen::Map<Eigen::VectorXd>(vec_doubles.data(), vec_doubles.size());
        std::cout <<  " * " << name_param_full << ": " << vec_out.transpose().format(this->clean_format) << "\n";
    }
    else{
        std::cout <<  " * " << name_param_full << " failed to load because it doesn't exist...\n";
    }

    return;
}




#endif // _DATA_PARSER_
