# Stimulator G

## Видео работы решения

[![Видео работы решения](http://img.youtube.com/vi/Bpk09-oP4mI/0.jpg)](http://www.youtube.com/watch?v=Bpk09-oP4mI)

Кликабельно ↑↑↑ Youtube

## Коротко о процессе разработки

В первую очередь мы разделились на выполнение двух задач: сборка быстрого решения и настройка общих процессов (для прохождения квалификации) и более долгий и правильный подход для решения более общих задач.

### Первый этап

Запустить симуляцию удалось сразу, проблем с этим не возникло. Далее необходимо было построить карту для ориентации робота в прострастве. Для экономного по ресурсам решения был выбран 2D SLAM. Для построения карты мы решили попробовать `gmapping`. Почитав документацию `gmapping`, был составлен простой launch-файл. А также установлен и настроен виртуальный лазер `hokuyo_laser`. После построения карты выяснилось, что есть такая штука как **бордюры**, уровень которых не детектится лазером, а робот при ориентации по такой карте норовит врезаться в бордюр и застрять. Для решения этой проблемы мы решили дополнить датчики робота fake-laser'ом, работающим на облаке точек виртуальной RGBD-камеры. Данные из `PointCloud` камеры в формат `LaserScan` мы сконвертировали с помощью пакета `pointcloud_to_laserscan` таким образом, чтобы данные выбирались на высоте бордюров. Для лучшего обзора пришлось переместить RGBD-сенсор в переднюю часть (так в кадр перестал попадать робот).

Получив данные с RGBD-камеры, мы предприняли попытку построить карту с учётом бордюров. Для построения карты топик `kinect_scan` (виртуального лазера) и топик `hokuyo_laser` были объединены в один топик `scan` с помощью `ira_laser_tools`.

Однако карта все равно строилась преймущественно с учётом данных лазерного дальномера, тк данные о бордюрах не фиксировались из-за:

1. Ограниченного угла обзора лазера. Он смотрел только в одну сторону, перед роботом и слишком близко. В то время когда со спины и со сторон робота `hokuyo_laser` брал далеко и опять же не учитывал бордюров;
2. Частота fake-laser была мала.

> Строить карту только на втором лазере нормально не получалось, тк он смотрит только в одну сторону, достаточно недалеко => частота обновления данных маленькая и получается много артефактов.
Позже выяснилось, что даже если бы мы построили такую карту, то ориентация по ней с помощью 2D SLAM была бы практически невозможна (опять же из угла обзора лазера для бордюров).

### Второй этап

Для управления роботом нужны:

* map_server
* amcl
* move_base
* local/global planner

Данная связка вместе с launch-файлами была представлена в пакете `turtlebot_navigation`. При установке из исходников возникли проблемы с компилятором, при установке из deb-репозитория проблема с зависимостью `ros-kinetic-realsense` (нашли заметку, сделали patch, установили).

Затем был переписан дефолтный launch-файл для запуска `amcl` + `move_base` + `planner`. Выпилив оттуда запуск аппаратного лазера, и сделав remap `/cmd_vel` на `/mobile_base/commands/velocity`, мы смогли заставить робота ориентироваться в построенной карте.

Однако ввиду неправильной карты (не содержащей бордюров), робот не всегда мог доехать до конечной точки маршрута, т.к. застревал и почти не учитывал локальные препятствия.

### Третий этап

Т.к. топик `/scan` при столкновении с препядствием всё же говорил корректную информацию о нахождении бордюра, было принято решение повлиять на алгоритм построения маршрута робота.

Было решено с нуля настроить необходимые ноды из пакета `navigation` для корректного перемещения робота в пространстве с учётом динамических препятствий (в том числе в виде бордюров).

Поиграв с настройками дефолтных плагинов для пакета navigation `global_planner` и `local_planner` и позапускав робота из разных точек карты, мы решили попробовать альтернативные варианты. Для `global_planner` был выбран более гибкий `global_planner/GlobalPlanner`, а для `local_planner` - `teb_local_planner/TebLocalPlannerROS`, который на ходу перестраивает локальное изменение маршрута. С помощью настройки данных плагинов удалось получить устойчивое решение для ориентации и навигации робота по виртуальному городу.

## Запуск решения

Склонируйте данный репозиторий. Выполните инструкции по настройке докера из [исходного репозитория](https://gitlab.com/starline/hackathon_kobuki##установка-требуемого-по-1). Соберите и запустите докер контейнер для дальнейшего запуска симулятора и решения. Для этого из корневой директории этого репозитория выполните команды

``` cmd
./docker/simulator/build_docker.sh
./docker/simulator/run_docker.sh
```

В случае, если на вашем ПК используется видеокарта от nvidia, то обозначенные выше скрипты следует исполнить с параметром -n или --nvidia

Запустите симулятор в открывшейся консоли контейнера:

``` cmd
roslaunch tb_gazebo turtltown.launch
```

В отдельной командной строке выполните:

``` cmd
./docker/simulator/into_docker.sh
roslaunch solution_1 move_base.launch
```

Для навигации робота по карте (в том числе из точки старта в области `S` в точку финиша в области `F`), а также визуализации данных с робота, используйте программу `rviz`. Для этого в отдельной командной строке выполните

``` cmd
./docker/simulator/into_docker.sh
rviz
```

Для быстрой настройки окружения загрузите настройки из файла `/simulator_ws/rviz/tb_config.rviz`
