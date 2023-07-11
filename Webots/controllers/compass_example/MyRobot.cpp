/**
 * @file    MyRobot.cpp
 * @brief   A simple example for maintaining a straight line with the compass.
 *
 * @author  Fernando Alonso Martín
 * @date    2022-10
 */

#include "MyRobot.h"

//////////////////////////////////////////////

MyRobot::MyRobot() : Robot(){
    // init default values
    _time_step = 64;

    _left_speed = 0;
    _right_speed = 0;

    // get and enable the compass device
    _my_compass = getCompass("compass");
    _my_compass->enable(_time_step);
   
    _left_wheel_motor = getMotor("left wheel motor");
    _right_wheel_motor = getMotor("right wheel motor");
   
    distanceSensors[0]= getDistanceSensor("ds0");    
    distanceSensors[0]->enable(_time_step);
   
    distanceSensors[1]= getDistanceSensor("ds3");
    distanceSensors[1]->enable(_time_step);
   
    distanceSensors[2]= getDistanceSensor("ds13");
    distanceSensors[2]->enable(_time_step);
   
   
    //to control the robot on velicity
    // set position to infinity, to allow velocity control
    _left_wheel_motor->setPosition(_infinity);
    _right_wheel_motor->setPosition(_infinity);

    // set velocity to 0
    _left_wheel_motor->setVelocity(0.0);
    _right_wheel_motor->setVelocity(0.0);
   
   
    _mode = FORWARD;
   
   
   
    //for odometry
    _x = _y = _theta = 0.0;    
    _odometriaAcumuladaRuedaDerecha = _odometriaAcumuladaRuedaIzquierda = 0.0;
   
      // Motor Position Sensor initialization
    _left_wheel_sensor = getPositionSensor("left wheel sensor");
    _right_wheel_sensor = getPositionSensor("right wheel sensor");
   
     _left_wheel_sensor->enable(_time_step);
    _right_wheel_sensor->enable(_time_step);
   
   
    goalReached = false;
   
   
     // get cameras and enable them
    _forward_camera = getCamera("camera_f");
    _forward_camera->enable(_time_step);
    _spherical_camera = getCamera("camera_s");
    _spherical_camera->enable(_time_step);


   
}

//////////////////////////////////////////////

MyRobot::~MyRobot(){
    // disable devices
    _my_compass->disable();    
    distanceSensors[0]->disable();    
    distanceSensors[1]->disable();
    distanceSensors[2]->disable();
   
    _left_wheel_sensor->disable();
    _right_wheel_sensor->disable();
   
   
     // disable camera devices
    _forward_camera->disable();
    _spherical_camera->disable();
   
}

//////////////////////////////////////////////

void MyRobot::run(){
   
   
    // get size of images for forward camera
    image_width_f = _forward_camera->getWidth();
    image_height_f = _forward_camera->getHeight();
    cout << "Size of forward camera image: " << image_width_f << ", " <<  image_height_f << endl;

    // get size of images for spherical camera
    image_width_s = _spherical_camera->getWidth();
    image_height_s = _spherical_camera->getHeight();
    cout << "Size of spherical camera image: " << image_width_s << ", " << image_height_s << endl;
     
     

    while ((step(_time_step) != -1) && !goalReached){
   
     
      //get odometry
      compute_odometry();
      cout << "ODOMETRY_INFORMATION: x->" << _x << "; y->" << _y << "; theta-> " << _theta << endl;
     
       //goal reached!!!!
       if (_x > 14 || _x < -14){
         goalReached = true;
       }
     
   
      //read compass and distance sensors values
      const double* compass_values = _my_compass->getValues();
      compass_angle = convert_bearing_to_degrees(compass_values);
      ir_front = distanceSensors[0]->getValue();
      ir_left = distanceSensors[1]->getValue();
      ir_right = distanceSensors[2]->getValue();
      cout <<  "compass: " << RED << compass_angle << RESET_COLOR <<
      " ir_front: " << RED << ir_front << RESET_COLOR <<
      " ir_left: " << RED << ir_left << RESET_COLOR <<
      " ir_right: "<< RED << ir_right << RESET_COLOR << endl;
     
     
     
       
      //no obstacles or walls detected by distance sensors
      if ( (ir_front < DISTANCE_SENSOR_THRESHOLD) &&
           (ir_left < DISTANCE_SENSOR_THRESHOLD) &&
           (ir_right < DISTANCE_SENSOR_THRESHOLD) ){      
         
         compassStrategy();
           
      // there are obstacles or walls
      }else{  
     
          //TODO: changeIt for cameraStrategy
          //distanceSensorsStrategy();
          camerasStrategy();  
         
         
      }
           

     
      //regarding the mode we set a speed to the motors
      switch (_mode){
        case FORWARD:
          //cout << "Forward"<<endl;
          _left_speed = MAX_SPEED;
          _right_speed = MAX_SPEED;
          break;
        case TURNING_RIGHT:
          //cout << "Turning right"<<endl;
          _left_speed = MAX_SPEED;
          _right_speed = MAX_SPEED/3;
          break;
         
        case TURNING_LEFT:
          //cout << "Turning left"<<endl;
          _left_speed = MAX_SPEED/2;
          _right_speed = MAX_SPEED;
          break;
         
        case WALLFOLLOW:
           //cout << "Wallfollow"<<endl;
           _left_speed = MAX_SPEED;
          _right_speed = MAX_SPEED;        
          break;
         
        case STOP:
          _left_speed = 0;
          _right_speed = 0;  
          break;
      }
         
        //to set the speed of the motors      
       _left_wheel_motor->setPosition(_infinity);
       _right_wheel_motor->setPosition(_infinity);
       
       
       _left_wheel_motor->setVelocity(_left_speed);
       _right_wheel_motor->setVelocity(_right_speed);
 
   
    }
     
   
}

//////////////////////////////////////////////

double MyRobot::convert_bearing_to_degrees(const double* in_vector){
    double rad = atan2(in_vector[0], in_vector[2]);
    double deg = rad * (180.0 / M_PI);
    return deg;
}

//////////////////////////////////////////////

/** Compute the distance accumualated */

void MyRobot::compute_odometry(){
    float nuevaOdometriaRuedaDer = WHEEL_RADIUS * _right_wheel_sensor->getValue();
    float incrementoOdometriaRuedaDerecha = nuevaOdometriaRuedaDer - _odometriaAcumuladaRuedaDerecha;
   
    float nuevaOdometriaRuedaIzq = WHEEL_RADIUS * _left_wheel_sensor->getValue();
    float incrementoOdometriaRuedaIzquierda = nuevaOdometriaRuedaIzq - _odometriaAcumuladaRuedaIzquierda;
       
   
    _x = _x + ((incrementoOdometriaRuedaDerecha + incrementoOdometriaRuedaIzquierda)/2 * cos(_theta+((incrementoOdometriaRuedaDerecha-incrementoOdometriaRuedaIzquierda)/(2*WHEELS_DISTANCE))));

    _y = _y + ((incrementoOdometriaRuedaDerecha + incrementoOdometriaRuedaIzquierda)/2 * sin(_theta+((incrementoOdometriaRuedaDerecha-incrementoOdometriaRuedaIzquierda)/(2*WHEELS_DISTANCE))));    
   
    _theta = _theta + ((incrementoOdometriaRuedaDerecha - incrementoOdometriaRuedaIzquierda)/WHEELS_DISTANCE);
   
    _odometriaAcumuladaRuedaDerecha = nuevaOdometriaRuedaDer;
    _odometriaAcumuladaRuedaIzquierda = nuevaOdometriaRuedaIzq;

}



/**
   * Usign compass strategy
   */
void MyRobot::compassStrategy(){
     cout <<  GREEN "\nUsing compass strategy" RESET_COLOR<< endl;
           
     //compass controler
     if (compass_angle < (DESIRED_ANGLE - 2)) {
          // turn right
          _mode = TURNING_RIGHT;
          cout << "Turning right by compass"<< endl;
     }else {
          if (compass_angle > (DESIRED_ANGLE + 2)) {
                // turn left
                 _mode = TURNING_LEFT;
                 cout << "Turning left by compass"<< endl;
         }else{
                // move straight forward
                _mode = FORWARD;
                cout << "Move forward by compass"<< endl;
         }
     }
}



/**
  * Usign distanceSensorsStrategy strategy
  */
void MyRobot::distanceSensorsStrategy(){
       
        cout <<  BLUE << "\nUsign distance sensor controller" << RESET_COLOR<< endl;

        //the logic about wallfollower
        if (_mode == FORWARD){
            if (ir_front > DISTANCE_SENSOR_THRESHOLD){  //close to the wall
                _mode = TURNING_LEFT;
          }        
        }else if (_mode == TURNING_LEFT){
            if (ir_front < DISTANCE_SENSOR_THRESHOLD){
              //cout << "Wallfollow" << endl;
               _mode = WALLFOLLOW;
            }
        }else if (_mode == WALLFOLLOW){
            if (ir_front > DISTANCE_SENSOR_THRESHOLD){
                 _mode = TURNING_LEFT;
            }
         
            if (ir_right < DISTANCE_SENSOR_THRESHOLD){
                //cout << "Turn right" << endl;
                _mode = TURNING_RIGHT;
            }
         
        }else if (_mode == TURNING_RIGHT){
             if (ir_right > DISTANCE_SENSOR_THRESHOLD){
                 //cout << "Wallfow" << endl;
                _mode =WALLFOLLOW;
              }
        }
}



/**
  * Usign distanceSensorsStrategy strategy
  */
void MyRobot::camerasStrategy(){


      //TODO: IMPLEMENTS IT to NAVIGATE IN THE MAZE!!!!!!
     
     
      //read forward camera
      const unsigned char *image_f = _forward_camera->getImage();      
      unsigned char green = 0, red = 0, blue = 0;
      int sum_of_pixels_1 = 0,sum_of_pixels_2 = 0,sum_of_pixels_3 = 0;
      double percentage_black_1 = 0.0,percentage_black_2 = 0.0,percentage_black_3 = 0.0;
      //dimensions of the parts upper limit (in the width axe)
      int image_P1_upper_limit=image_width_f/3;  //first part upper limit from coordinates 0 to image_width_f/3
      int image_P2_upper_limit=2*image_width_f/3;  //second part upper limit from coordinates image_width_f/3 to 2*image_width_f/3
      int image_P3_upper_limit=image_width_f;  //second part upper limit from coordinates image_width_f/3 to 2*image_width_f/3
     
       //dimensions of the parts bottom limit (in the width axe)
      int image_P1_bottom_limit=0;  //first part bottom limit from coordinates 0 to image_width_f/3
      int image_P2_bottom_limit=image_width_f/3;  //second bottom limit from coordinates image_width_f/3 to 2*image_width_f/3
      int image_P3_bottom_limit=2*image_width_f/3;  //third part bottom limit from coordinates image_width_f/3 to 2*image_width_f/3
     
      cout<< "part 1 boundaries x =["<<image_P1_bottom_limit<<";"<<image_P1_upper_limit<<"]\n"<<endl;
      cout<< "part 2 boundaries x =["<<image_P2_bottom_limit<<";"<<image_P2_upper_limit<<"]\n"<<endl;
      cout<< "part 3 boundaries x =["<<image_P3_bottom_limit<<";"<<image_P3_upper_limit<<"]\n"<<endl;
      // count number of pixels that are black
      // divide the image into 4 parts : the upper half one and three same sized images in the
      //botom half
     
      //first 1/3 part
      for (int x =image_P1_bottom_limit; x < image_P1_upper_limit; x++) {
            for (int y = image_height_f/2; y <image_height_f ; y++) {
                green = _forward_camera->imageGetGreen(image_f, image_P1_upper_limit, x, y);
                red = _forward_camera->imageGetRed(image_f, image_P1_upper_limit, x, y);
                blue = _forward_camera->imageGetBlue(image_f, image_P1_upper_limit, x, y);
                       
                if ((green < COLOR_THRESHOLD) && (red < COLOR_THRESHOLD) && (blue < COLOR_THRESHOLD)) {
                    sum_of_pixels_1++;
                }
            }
      }
     percentage_black_1 = (sum_of_pixels_1 / (float) (image_P1_upper_limit * image_height_f/2)) * 100;
     
     cout << "Percentage of black in forward camera image part 1: " << percentage_black_1 << endl;
     
     //second 1/3 part
     for (int x = image_P2_bottom_limit; x < image_P2_upper_limit; x++) {
            for (int y = image_height_f/2; y <image_height_f; y++) {
                green = _forward_camera->imageGetGreen(image_f, image_P2_upper_limit, x, y);
                red = _forward_camera->imageGetRed(image_f, image_P2_upper_limit, x, y);
                blue = _forward_camera->imageGetBlue(image_f, image_P2_upper_limit, x, y);

                if ((green < COLOR_THRESHOLD) && (red < COLOR_THRESHOLD) && (blue < COLOR_THRESHOLD)) {
                    sum_of_pixels_2 ++;
                }
            }
      }
     percentage_black_2 = (sum_of_pixels_2  / (float) ((image_P2_upper_limit-image_P2_bottom_limit) * image_height_f/2)) * 100;
     cout << "Percentage of black in forward camera image part 2: " << percentage_black_2 << endl;

 //third 1/3 part
     for (int x = image_P3_bottom_limit; x < image_P3_upper_limit; x++) {
            for (int y = image_height_f/2; y <image_height_f; y++) {
                green = _forward_camera->imageGetGreen(image_f, image_P3_upper_limit, x, y);
                red = _forward_camera->imageGetRed(image_f, image_P3_upper_limit, x, y);
                blue = _forward_camera->imageGetBlue(image_f, image_P3_upper_limit, x, y);

                if ((green < COLOR_THRESHOLD) && (red < COLOR_THRESHOLD) && (blue < COLOR_THRESHOLD)) {
                    sum_of_pixels_3++;
                }
            }
      }
     percentage_black_3  = (sum_of_pixels_3 / (float) ((image_P3_upper_limit-image_P3_bottom_limit)* image_height_f/2)) * 100;
     cout << "Percentage of black in forward camera image part 3: " << percentage_black_3 << endl;
     
     
     
     //float m = std::max({percentage_black_1, percentage_black_2, percentage_black_3});
     float m = std::max({percentage_black_1, percentage_black_2, percentage_black_3});
    cout<<RED <<"the maximum is : "<<m<<RESET_COLOR<<endl;
     
     //if the max surpasses a certain threshhold and there's no obstacles
     if (m>0)
          {
         if((m==sum_of_pixels_1)//&&((ir_right < DISTANCE_SENSOR_THRESHOLD)||(ir_front < DISTANCE_SENSOR_THRESHOLD))
         ){
           _mode = TURNING_LEFT;
           cout<<BLUE<<"\nturning left using camera"<<RESET_COLOR<<endl;
         
         }
         if(m==sum_of_pixels_2){
           _mode =FORWARD;
           cout<<BLUE<<"\nmoving forward using camera"<<RESET_COLOR<<endl;
         }
         if((m==sum_of_pixels_3)//&&((ir_front < DISTANCE_SENSOR_THRESHOLD) ||(ir_left < DISTANCE_SENSOR_THRESHOLD) )
         ){
           _mode =TURNING_RIGHT;
           cout<<BLUE<<"\nturning right using camera"<<RESET_COLOR<<endl;
         }
         
      }
      else {
           _mode =WALLFOLLOW;
           MyRobot::distanceSensorsStrategy();
           
            /*if (ir_front > DISTANCE_SENSOR_THRESHOLD){
                 _mode = TURNING_LEFT;
            }
         
            if (ir_right < DISTANCE_SENSOR_THRESHOLD){
                //cout << "Turn right" << endl;
                _mode = TURNING_RIGHT;
            }*/
        cout<<BLUE<<"\nfollowing the wall using camera"<<RESET_COLOR<<endl;
      }
     


}
