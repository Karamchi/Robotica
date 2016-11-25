#include "keys_to_twist.h"

using namespace robmovil;

KeysToTwist::KeysToTwist(ros::NodeHandle& nh)
  : nh_(nh)
{
  // nos susbribimos a los tópicos que reportan cuando se apreta o suelta una tecla.
  key_up_sub_ = nh.subscribe("/keyboard/keyup", 1, &KeysToTwist::on_key_up, this);
  key_down_sub_ = nh.subscribe("/keyboard/keydown", 1, &KeysToTwist::on_key_down, this);

  // anunciamos que vamos a publicar un mensaje de tipo geometry_msgs::Twist
  // en el tópico /robot/cmd_vel.
  twist_pub_ = nh.advertise<geometry_msgs::Twist>("/robot/cmd_vel", 1);
  
  /** ATENCION: Pueden declara variables en el cuerpo de la clase KeysToTwist, en el archivo keys_to_twist.h
   *  e iniciarlas aqui ****/
   
}
geometry_msgs::Twist twist;

void KeysToTwist::on_key_down(const keyboard::Key& key_event)
{

  switch( key_event.code )
  {
    case keyboard::Key::KEY_UP:
      twist.linear.x = 3.9;
      break;

    case keyboard::Key::KEY_DOWN:
      twist.linear.x = -3.9;
      // completar ...
      break;

    case keyboard::Key::KEY_LEFT:
      // completar ...
      twist.angular.z = 3.9;
      break;

    case keyboard::Key::KEY_RIGHT:
      twist.angular.z = -3.9;
      // completar ...
      break;

    default:
      return;
  }

  /* definir los parametros del twist publicado en base a las teclas.. */

  twist_pub_.publish( twist );
}

void KeysToTwist::on_key_up(const keyboard::Key& key_event)
{
//  geometry_msgs::Twist twist;

  switch( key_event.code )
  {
  case keyboard::Key::KEY_UP:
      twist.linear.x = 0;
      break;

    case keyboard::Key::KEY_DOWN:
    twist.linear.x = 0;
      // completar ...
      break;

    case keyboard::Key::KEY_LEFT:
      // completar ...
      twist.angular.z = 0;
      break;

    case keyboard::Key::KEY_RIGHT:
      twist.angular.z = 0;
      // completar ...
      break;

    default:
      return;
  }
  
  /* definir los parametros del twist publicado en base a las teclas.. */

  twist_pub_.publish( twist );
}
