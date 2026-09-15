Проєкт для автономної посадки безпілотника на рухому платформу в симуляторі Gazebo.

Суть програми дуже проста: дрон отримує координати рухомої цілі (чорної платформи), летить за нею, вирівнює свою швидкість зі швидкістю платформи та автоматично приземляється прямо на неї. 
Вся логіка прийняття рішень (коли летіти, а коли сідати) побудована на деревах поведінки (Behavior Trees), а безпосереднім керуванням моторів займається автопілот PX4.

 Технології
ROS 2
PX4 Autopilot
Gazebo Simulator
BehaviorTree.CPP
                        Як запустити
1. Запустіть симулятор PX4 з потрібним світом та моделлю дрона(також за потреби змінити назви топіків в Groot).
2. Запустити містки PX4-ROS2 та ROS2-Gazebo:
   MicroXRCEAgent udp4 -p 8888
   ros2 run ros_gz_bridge parameter_bridge /model/plate/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist /model/plate/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry
3. Зберіть пакет у робочій області:
   colcon build
4. Запустіть проект:
   ros2 run project1 run
