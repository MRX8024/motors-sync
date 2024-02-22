Скрипт подстраивает положение вала на моторе X1/Y1 и с помощью акселерометра замеряет "удары" при подачи питания на мотор X/Y.

### 1. Установка скрипта калибровки на хост принтера. (последует перезагрузка клиппера!)
```
    cd ~
    git clone https://github.com/MRX8024/motors_sync
    bash ~/motors_sync/install.sh
```
В файле `motors_sync.py` в 8 строке `return np.sqrt(accel_x**2 + accel_y**2 + accel_z**2)` необходимо исключить ось акселерометра перпендикулярную земле,
чтобы уменьшить уровень шумов акселерометра. Проверить можно это командой `ACCELEROMETER_QUERY chip=<акселерометр>`. Либо посмотреть на сам акселерометр, на котором обязана присутствовать маркировка, с учетом, что оси не переопределены в конфигурации принтера. Например, если перпендикулярна земле ось Z акселерометра, то строка должна выглядеть так `return np.sqrt(accel_x**2 + accel_y**2)`.

Если вы используете иной акселерометр, нежели который указан в `[resonance_tester]`, добавьте блок приведенный ниже в любое место конфигурации принтера, и укажите свой акселерометр между ' '. Если оба пункта не указаны, будет выбран `adxl345`.
```
[gcode_macro _MOTORS_SYNC_VARIABLES]
variable_accelerometer: ''                                                                 ; Force accelerometer
gcode:
```

2. Подключаем акселерометр как при измерении резонансов для input_shaper.

3. Синхронизация моторов:
   4. Запускаем макрос MOTORS_SYNC из панели макросов на главной странице веб интерфейса и ждем завершения процесса.
 

4. Примечания:

    1. Синхронизация происходит с точностью 1/16 шага. 
    2. Не включайте нагрев хотенда во время синхронизации. В моем случае работающий вентилятор добавляет шума на акселерометр.
    3. Чем сильнее рассинхронизированы моторы, тем дольше длится синхронизация. В худшем случае я тратил 4 минуты.
    Есть возможность ускорить процесс уменьшив количество итераций, но мне для начала нужен фидбек от пользователей.
    Макрос возможно запустить в начале печати во время нагрева стола. Для этого нужно изменить макрос паузы и возобновления.
```
[gcode_macro PAUSE]
rename_existing: BASE_PAUSE
gcode:
  {% set sync_motors = params.SYNC_MOTORS|default(0)|int %}
  SET_IDLE_TIMEOUT TIMEOUT=43200
  {% if sync_motors == 1 %}
    G1 X10 Y10 Z10 F16500
    SAVE_GCODE_STATE NAME=PAUSESYNC
    BASE_PAUSE
    MOTORS_SYNC
  {% else %}
    _Ваш макрос паузы_
  {% endif %}
```
```
[gcode_macro RESUME]
rename_existing: BASE_RESUME
gcode:
    {% set sync_motors = params.SYNC_MOTORS|default(0)|int %}
    SET_IDLE_TIMEOUT TIMEOUT={printer.configfile.settings.idle_timeout.timeout}
   {% if sync_motors == 1 %}
      RESTORE_GCODE_STATE NAME=PAUSESYNC MOVE=1 MOVE_SPEED=275
      BASE_RESUME
    {% else %}
      _Ваш макрос возобновления_
    {% endif %}
```
А в начале макроса начала печати указать:
```
M140 S[bed_temperature_initial_layer_single] ;set bed temp
G28
PAUSE SYNC_MOTORS=1
G28
M190 S[bed_temperature_initial_layer_single] ; wait for bed temp to stabilize
```
Мои макросы [здесь](/wiki/pause_resume.cfg).