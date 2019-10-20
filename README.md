### Описание репозитория Hackathon StarLine 2019

На роботе turtlebot 2 дополнительно установлены: rgbd-камера Astra Orbbec и лидар RPLidar A2. В данном репозитории содержиться Dockerfile для сборки контейнера на базе Ubuntu16.04 внутри которого доступны драйвера для всего аппаратного обеспечения и ROS kinetic.

### Установка требуемого ПО

Для успешного запуска докер-контейнера предварительно требуется установить необходимое ПО и udev-правила для используемого на turtlebot аппаратного обеспечения. Для этого из корня проекта требуется выполнить:

    cd scripts && bash setup.bash

### Сборка контейнера

    bash docker/build_docker.sh

### Запуск контейнера

    bash docker/run_docker.sh

### Запуск bash в контейнере

    bash docker/into_docker.sh

### Организация процесса разработки

Для удобства разработки в корне проекта созданы два [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/quick_start.html) окруженя. В drivers_tws располагаются драйвера аппаратного обеспечения робота, catkin_tws - пользовательское окружение.  Оба окружения пробрасываются внутрь контейнера.

При первом входе в контейнер требуется собрать драйвера в окружении drivers_tws, для этого:

    cd /drivers_tws
    catkin init
    catkin build
    
### Использование

Для запуска драйверов следует использовать соответствующие launch файлы из пакета tb:

    roslaunch tb rplidar.launch
    roslaunch tb base.launch
    roslaunch tb astra.launch

### Информация о роботах

| TurtleBot # | Hostname | Password| Status |
|:--|:--:|---:|---:|
| TurtleBot 1 | tb1@tb1 | tb | ready |
| TurtleBot 2 | tb2@tb2 | tb | only OS |
| TurtleBot 3 | tb3@tb3 | tb | - |
| TurtleBot 4 | tb4@tb4 | tb | - |
| TurtleBot 5 | tb5@tb5 | tb | only OS |
| TurtleBot 6 | tb6@tb6 | tb | only OS |
| TurtleBot 7 | tb7@tb7 | tb | only OS |
| TurtleBot 8 | tb8@tb8 | tb | only OS |
| TurtleBot 9 | tb9@tb9 | tb | - |
| TurtleBot 10 | tb10@tb10 | tb | - |
| TurtleBot 11 | tb11@tb11 | tb | - |
| TurtleBot 12 | tb12@tb12 | tb | - |
| TurtleBot 13 | tb13@tb13 | tb | only OS |
