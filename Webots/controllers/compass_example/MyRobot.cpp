/**
 * @file    MyRobot.cpp
 * @brief   A simple example for maintaining a straight line with the compass.
 *
 * @author  Sara Marqués Villarroya <smarques@ing.uc3m.es>
 * @author  Juan José Gamboa Montero <jgamboa@ing.uc3m.es>
 * @date    2020-10
 */

#include "MyRobot.h"

//////////////////////////////////////////////

MyRobot::MyRobot() : Robot()
{
    // init default values
    _time_step = 64;

    _left_speed = 0;
    _right_speed = 0;

    // get and enable the compass device
    _my_compass = getCompass("compass");
   
    
    _left_wheel_motor = getMotor("left wheel motor");
    _right_wheel_motor = getMotor("right wheel motor");
    
    distanceSensors[0]=getDistanceSensor("ds0");
    distanceSensors[0]-> enable(_time_step);
    
    distanceSensors[1]=getDistanceSensor("ds3");
    distanceSensors[1]-> enable(_time_step);
   
    distanceSensors[2]=getDistanceSensor("ds13");
    distanceSensors[2]-> enable(_time_step);
      _mode = FORWARD;
    
    //set position to infinity to allow velocity control 
    _right_wheel_motor-> setPosition(_infinity);
    _left_wheel_motor-> setPosition(_infinity);
    
    //to control the velocity
    _right_wheel_motor-> setVelocity(0.0);
    _left_wheel_motor-> setVelocity(0.0);
    
    
}

//////////////////////////////////////////////

MyRobot::~MyRobot()
{
    // disable devices
    
    distanceSensors[0]-> disable();
    distanceSensors[1]-> disable();    
    distanceSensors[2]-> disable();

    
    
}

//////////////////////////////////////////////

void MyRobot::run()
{
    
    
    double ir_front,ir_left, ir_right;

    while (step(_time_step) != -1) {
      
      
      ir_front=distanceSensors[0]->getValue();
      ir_left=distanceSensors[1]->getValue();
      ir_right=distanceSensors[2]->getValue();
      cout<<"ir_front:"<< ir_front<<"ir_left:"<<ir_left<<"ir_right:"<<ir_right<<endl;
      
       if (_mode==FORWARD){
         if (ir_front>DISTANCE_SENSOR_THRESHOLD){
             _mode=TURNING_LEFT;
          }
       }else if (_mode==TURNING_LEFT){
           if (ir_front < DISTANCE_SENSOR_THRESHOLD){
               _mode=WALLFOLLOW;
           }
        } else if (_mode==WALLFOLLOW){
             if (ir_front > DISTANCE_SENSOR_THRESHOLD){
                 _mode=TURNING_LEFT;
             }
             if (ir_right< DISTANCE_SENSOR_THRESHOLD){
               _mode= TURNING_RIGHT;
             } 
             
         
           } else if (_mode==TURNING_RIGHT){
                if (ir_right> DISTANCE_SENSOR_THRESHOLD){
                    _mode=WALLFOLLOW;
                }
            }
            
                  
        
        
       
      

       
       
       
       // ñode for the motors
       
        switch (_mode){
           case FORWARD:
              _left_speed = MAX_SPEED ;
              _right_speed =MAX_SPEED ;
           
             break;
           case TURNING_RIGHT:
                _left_speed = MAX_SPEED ;
                _right_speed =MAX_SPEED/1.85 ;
             break;
           case TURNING_LEFT:
                _left_speed = MAX_SPEED /1.85 ;
                _right_speed =MAX_SPEED;
             break;
           case WALLFOLLOW:
                _left_speed = MAX_SPEED;
                _right_speed =MAX_SPEED ;
             break;
           case STOP:
                _left_speed = 0;
                _right_speed =0;
            break;
         }
         // set the motor position to non-stop moving
        _left_wheel_motor->setPosition(_infinity);
        _right_wheel_motor->setPosition(_infinity);
        
        // set the motor speeds
        _left_wheel_motor->setVelocity(_left_speed );
        _right_wheel_motor->setVelocity(_right_speed);
    
     
       
    
        
    }
}

//////////////////////////////////////////////

double MyRobot::convert_bearing_to_degrees(const double* in_vector)
{
    double rad = atan2(in_vector[0], in_vector[2]);
    double deg = rad * (180.0 / M_PI);

    return deg;
}

//////////////////////////////////////////////

