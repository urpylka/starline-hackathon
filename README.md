### Описание репозитория Hackathon StarLine 2019

На роботе turtlebot 2 дополнительно установлены: rgbd-камера Astra Orbbec и лидар RPLidar A2. В данном репозитории содержиться Dockerfile для сборки контейнера на базе Ubuntu16.04 внутри которого доступны драйвера для всего аппаратного обеспечения и ROS kinetic.

![kobuki_view](https://github.com/NickoDema/kobuki/blob/master/docs/pics/kobuki_view.png)

### Установка требуемого ПО

Для успешного запуска докер-контейнера предварительно требуется установить необходимое ПО и udev-правила для используемого на turtlebot аппаратного обеспечения. Для этого из корня проекта требуется выполнить:

    bash scripts/setup.bash

После этой операции следует перезайти в систему для корректной работы docker. После этого приведенные ниже команды можно использовать из находясь в любой директории.

### Сборка контейнера

    kobuki_docker_build

### Запуск контейнера

    kobuki_docker_run

### Запуск bash в контейнере

    kobuki_docker_into

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
| TurtleBot 2 | tb2@tb2 | tb | ready (base charging problem) |
| TurtleBot 3 | tb3@tb3 | tb | ready |
| TurtleBot 4 | tb4@tb4 | tb | ready |
| TurtleBot 5 | tb5@tb5 | tb | ready |
| TurtleBot 6 | tb6@tb6 | tb | ready |
| TurtleBot 7 | tb7@tb7 | tb | ready |
| TurtleBot 8 | tb8@tb8 | tb | ready |
| TurtleBot 9 | tb9@tb9 | tb | ready |
| TurtleBot 10 | tb10@tb10 | tb | ready (base charging problem) |
| TurtleBot 11 | tb11@tb11 | tb | ready |
| TurtleBot 12 | tb12@tb12 | tb | ready |
| TurtleBot 13 | tb13@tb13 | tb | ready |
