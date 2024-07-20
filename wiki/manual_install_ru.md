1. Скачайте репозиторий - `git clone https://github.com/MRX8024/motors-sync`
2. Создайте ссылку к программе - `ln -sf ~/motors-sync/motors_sync.py ~/klipper/klippy/extras/`
3. Установите пакеты -

    ```sudo apt-get install libatlas-base-dev libopenblas-dev```

    ```~/klippy-env/bin/pip install numpy matplotlib scipy```


4. Добавьте в конфигурацию принтера строки - 
```
[force_move]
enable_force_move: True
```

Также, по желанию, можно добавить раздел обновления в moonraker для последующих обновлений через ведморду в разделе обновлений.
```
[update_manager motors-sync]
type: git_repo
path: ~/motors-sync/
origin: https://github.com/MRX8024/motors-sync.git
primary_branch: main
managed_services: klipper
```
