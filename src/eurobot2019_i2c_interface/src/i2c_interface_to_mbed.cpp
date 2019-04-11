#include "i2c_interface_to_mbed.hpp"

I2C_interface_to_mbed::I2C_interface_to_mbed(){}

  I2C_interface_to_mbed::~I2C_interface_to_mbed(){}

    void I2C_interface_to_mbed::send_drive_i2c_msg(const geometry_msgs::Twist& cmd_vel_msg){
      //consider changing to array?
      std::vector<short> drive_motor_msg;
      // Gives rotations per second (frequency)
      // x +ve is forward, y +ve is left, (need to check if same as navigation stack, if not change appropriately)
      drive_motor_msg.push_back(((cmd_vel_msg.linear.x - cmd_vel_msg.linear.y - 0.2422545173*cmd_vel_msg.angular.z)*100)/RADIUS/3.1415926/2); //top left, *100, needed when converting to shorts (keep 2 dp)
      drive_motor_msg.push_back(((cmd_vel_msg.linear.x + cmd_vel_msg.linear.y + 0.2422545173*cmd_vel_msg.angular.z)*100)/RADIUS/3.1415926/2); //top right
      drive_motor_msg.push_back(((cmd_vel_msg.linear.x + cmd_vel_msg.linear.y - 0.2067455443*cmd_vel_msg.angular.z)*100)/RADIUS/3.1415926/2); //bottom left
      drive_motor_msg.push_back(((cmd_vel_msg.linear.x - cmd_vel_msg.linear.y + 0.2067455443*cmd_vel_msg.angular.z)*100)/RADIUS/3.1415926/2); //bottom right

      std::string drive_i2c_msg;
      // save pointer for speed of  in char pointer d
      char *d;
      for(int j = 0; j < 4; j++){
        d = (char*)&(drive_motor_msg[j]);
        for(int i = 0; i < 2; i++){
          // dereference every byte of a wheel_vel as a char into drive_i2c_msg
          drive_i2c_msg += *(d++);
        }
      }

      //Send the converted drive_motor_msg from the interface to a target unknown in DRIVE.
      i2c_handler_.write(DRIVE, drive_i2c_msg);
      ROS_INFO("driveMsg: %hi, %hi, %hi, %hi", drive_motor_msg[0], drive_motor_msg[1], drive_motor_msg[2], drive_motor_msg[3]);
    }

    void I2C_interface_to_mbed::get_drive_i2c_msg(std::vector<float>& wheel_vel_msg){
        char wheel_vel_i2c_msg[8];
        i2c_handler_.read(DRIVE, 8, wheel_vel_i2c_msg); // reads motor values, length is 8 bytes

        for(int i = 0; i < 4; i++){
            short hibyte = wheel_vel_i2c_msg[2*i];
            short lobyte = wheel_vel_i2c_msg[2*i+1];
            int16_t x = hibyte << 8 | lobyte;
            std::cout << "The " << i << "th value is " << x << std::endl; //debug msgs
            wheel_vel_msg.push_back((float) x/100.f);
        }
        /* test
        wheel_vel_msg.push_back(6.12);
        wheel_vel_msg.push_back(6.12);
        wheel_vel_msg.push_back(6.12);
        wheel_vel_msg.push_back(6.12);
        */
        ROS_INFO("wheel_vel_msg: %0.1f, %0.1f, %0.1f", wheel_vel_msg[0], wheel_vel_msg[1], wheel_vel_msg[2], wheel_vel_msg[3]);
    }

    void I2C_interface_to_mbed::send_grabber_i2c_msg(const eurobot2019_messages::grabber_motors& grabber_motors_msg){
      //consider changing to array?
      std::vector<short> grabber_vector;
      // Gives grabber values (pos, servo_state, e.t.c)
      // 3 floats and a bool
      grabber_vector.push_back(grabber_motors_msg.z_pos_mm * 100);
      grabber_vector.push_back(grabber_motors_msg.open_pos_mm * 100);
      grabber_vector.push_back(grabber_motors_msg.z_twist_rad * 100);
      grabber_vector.push_back(grabber_motors_msg.servo_state);
      std::string grabber_i2c_msg;
      // save pointer for eurobot2019_messages/grabber_motors elements in char pointer d
      char *d;
      for(int j = 0; j < 4; j++){
        d = (char*)&(grabber_vector[j]);
        for(int i = 0; i < 2; i++){
          // dereference every byte of element as a char into grabber_i2c_msg
          grabber_i2c_msg += *(d++);
        }
      }

      //Send the converted grabber_motors_msg from the interface to a target unknown in GRABBER.
      i2c_handler_.write(GRABBER, grabber_i2c_msg);
    }

    void I2C_interface_to_mbed::get_grabber_i2c_msg(eurobot2019_messages::grabber_motors& grabber_status_msg) {
        char grabber_motors_i2c_msg[7];
        i2c_handler_.read(GRABBER, 7, grabber_motors_i2c_msg); //reads motor values, length assumed to be 7
        short* tmp = (short*)grabber_motors_i2c_msg;

        grabber_status_msg.z_pos_mm = (float) *(tmp++)/100.f;
        grabber_status_msg.open_pos_mm = (float) *(tmp++)/100.f;
        grabber_status_msg.z_twist_rad = (float) *(tmp++)/100.f;

        char* tmp_bool = (char*)tmp;

        grabber_status_msg.servo_state = (bool) *(tmp_bool);
      }

      void I2C_interface_to_mbed::send_dropper_i2c_msg(const eurobot2019_messages::drop_motors& drop_motors_msg) {
          //consider changing to array?
          std::vector<short> dropper_vector;
          // Gives dropper values (left_x (position of pusher), left_z (position of stepper, i.e. atom platform), e.t.c)
          // 3 floats and a bool
          dropper_vector.push_back(drop_motors_msg.left_z * 100);
          dropper_vector.push_back(drop_motors_msg.left_x * 100);
          dropper_vector.push_back(drop_motors_msg.right_z * 100);
          dropper_vector.push_back(drop_motors_msg.right_x * 100);
          dropper_vector.push_back(drop_motors_msg.middle_z * 100);
          dropper_vector.push_back(drop_motors_msg.middle_x * 100);
          std::string dropper_i2c_msg;
          // save pointer for eurobot2019_messages/drop_motors elements in char pointer d
          char *d;
          for(int j = 0; j < 6; j++){
            d = (char*)&(dropper_vector[j]);
            for(int i = 0; i < 2; i++){
              // dereference every byte of element as a char into dropper_i2c_msg
              dropper_i2c_msg += *(d++);
            }
          }

          //Send the converted drop_motors_msg from the interface to a target unknown in DROPPER.
          i2c_handler_.write(DROPPER, dropper_i2c_msg);
      }

      void I2C_interface_to_mbed::get_dropper_i2c_msg(eurobot2019_messages::drop_motors& drop_status_msg){
          char dropper_status_i2c_msg[12];
          i2c_handler_.read(DROPPER, 12, dropper_status_i2c_msg); //reads motor values, length assumed to be 12
          short* tmp = (short*)dropper_status_i2c_msg;

          drop_status_msg.left_z = (float) *(tmp++)/100.f;
          drop_status_msg.left_x = (float) *(tmp++)/100.f;
          drop_status_msg.right_z = (float) *(tmp++)/100.f;
          drop_status_msg.right_x = (float) *(tmp++)/100.f;
          drop_status_msg.middle_z = (float) *(tmp++)/100.f;
          drop_status_msg.middle_x = (float) *(tmp)/100.f;
      }
