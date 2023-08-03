#!/usr/bin/env python3
import os
import rclpy

def param_set_string(self, this_param:str, default_param)->str:
    
    return_param:str = ""
    is_being_used = 0
    
    # Searching for ENV variable
    try:        
        return_param = os.environ[this_param.upper()]
        is_being_used = 3
    except:
        # if fail ENV searching from params.yaml file for variable
        return_param= ""
        try:
            self.declare_parameter(this_param, rclpy.Parameter.Type.STRING)
            return_param = self.get_parameter(this_param).get_parameter_value().string_value
            is_being_used = 2
        except:
            return_param= ""
            # if non found return default value "default_param"    
            return_param = default_param
            is_being_used = 1
    
    if is_being_used == 3 :
        self.get_logger().info("Param "+str(this_param.upper())+ "=" +str(return_param)+"; and is being used from ENV", once=True)
    elif is_being_used == 2 :
        self.get_logger().info("Param "+str(this_param)+ "=" +str(return_param)+"; and is being used from params.yaml", once=True)
    elif is_being_used == 1 :
        self.get_logger().info("Param "+str(this_param)+ "=" +str(return_param)+"; using the default value", once=True)
    
    return str(return_param)
