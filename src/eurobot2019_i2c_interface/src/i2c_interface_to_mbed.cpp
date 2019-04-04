#include "i2c_interface_to_mbed.hpp"

I2C_interface_to_mbed::I2C_interface_to_mbed(){}

  I2C_interface_to_mbed::~I2C_interface_to_mbed(){}

    void I2C_interface_to_mbed::send_drive_i2c_msg(const geometry_msgs::Twist& cmd_vel_msg){
      //consider changing to array?
      std::vector<float> drive_motor_msg;
      // Gives rotations per second (frequency)
      // x +ve is forward, y +ve is left, (need to check if same as navigation stack, if not change appropriately)
      drive_motor_msg.push_back((cmd_vel_msg.linear.x - cmd_vel_msg.linear.y - 261.236363668644*cmd_vel_msg.angular.z)/RADIUS/3.1415926/2); //top left
      drive_motor_msg.push_back((cmd_vel_msg.linear.x + cmd_vel_msg.linear.y + 261.236363668644*cmd_vel_msg.angular.z)/RADIUS/3.1415926/2); //top right
      drive_motor_msg.push_back((cmd_vel_msg.linear.x + cmd_vel_msg.linear.y - 187.763636356656*cmd_vel_msg.angular.z)/RADIUS/3.1415926/2); //bottom left
      drive_motor_msg.push_back((cmd_vel_msg.linear.x - cmd_vel_msg.linear.y + 187.763636356656*cmd_vel_msg.angular.z)/RADIUS/3.1415926/2); //bottom right

      std::string drive_i2c_msg;
      // save pointer for speed of  in char pointer d
      char *d;
      for(int j = 0; j < 4; j++){
        d = (char*)&(drive_motor_msg[j]);
        for(int i = 0; i < 4; i++){
          // dereference every byte of a wheel_vel as a char into drive_i2c_msg
          drive_i2c_msg += *(d++);
        }
      }

      //Send the converted drive_motor_msg from the interface to a target unknown in DRIVE.
      i2c_handler_.write(DRIVE, drive_i2c_msg);
      ROS_INFO("Msg：%0.1f, %0.1f, %0.1f, %0.1f", drive_motor_msg[0], drive_motor_msg[1], drive_motor_msg[2], drive_motor_msg[3]);
    }

    void I2C_interface_to_mbed::get_drive_i2c_msg(std::vector<float>& wheel_vel_msg){
        std::string wheel_vel_i2c_msg;
        wheel_vel_i2c_msg = i2c_handler_.read(DRIVE, 16); //reads motor values, length assumed to be 16
        char str[16];
        sprintf(str, "%s", wheel_vel_i2c_msg.c_str());
        float* tmp = (float*)str;

        for(int i = 0; i < 4; i++){
            wheel_vel_msg.push_back(*(tmp++));
        }
    }

    void I2C_interface_to_mbed::send_grabber_i2c_msg(const eurobot2019_messages::grabber_motors& grabber_motors_msg){
      //consider changing to array?
      std::vector<float> grabber_vector;
      // Gives grabber values (pos, servo_state, e.t.c)
      // 3 floats and a bool
      grabber_vector.push_back(grabber_motors_msg.y_pos_mm);
      grabber_vector.push_back(grabber_motors_msg.open_pos_mm);
      grabber_vector.push_back(grabber_motors_msg.z_twist_rad);
      grabber_vector.push_back(grabber_motors_msg.servo_state);
      std::string grabber_i2c_msg;
      // save pointer for eurobot2019_messages/grabber_motors elements in char pointer d
      char *d;
      for(int j = 0; j < 4; j++){
        d = (char*)&(grabber_vector[j]);
        for(int i = 0; i < 4; i++){
          // dereference every byte of element as a char into grabber_i2c_msg
          grabber_i2c_msg += *(d++);
        }
      }

      //Send the converted grabber_motors_msg from the interface to a target unknown in GRABBER.
      i2c_handler_.write(GRABBER, grabber_i2c_msg);
    }

      /* void I2C_interface_to_mbed::get_grabber_i2c_msg(/*type?*///& grabber_status) {
        /* code */
      //}
      //*/

      void I2C_interface_to_mbed::send_dropper_i2c_msg(const eurobot2019_messages::drop_motors& drop_motors_msg) {
        std::string dropper_i2c_msg;
        char* d = (char*)&drop_motors_msg.tower_num;
        dropper_i2c_msg += *(d);
        //Send the converted drop_motors_msg from the interface to a target unknown in DROPPER.
        i2c_handler_.write(DROPPER, dropper_i2c_msg);
      }

      void I2C_interface_to_mbed::get_dropper_i2c_msg(eurobot2019_messages::drop_status& drop_status){
        std::string dropper_i2c_msg;
        dropper_i2c_msg = i2c_handler_.read(DROPPER, 3); //reads motor values, length assumed to be 16
        drop_status.left_tower_contents = dropper_i2c_msg[0];
        drop_status.middle_tower_contents = dropper_i2c_msg[1];
        drop_status.right_tower_contents = dropper_i2c_msg[2];
      }
