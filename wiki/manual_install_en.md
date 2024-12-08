1. Download a program - `git clone https://github.com/MRX8024/motors-sync`
2. Create a link to the program - `ln -sf ~/motors-sync/motors_sync.py ~/klipper/klippy/extras/`
3. Install packages -

    ```sudo apt-get install libatlas-base-dev libopenblas-dev```

    ```~/klippy-env/bin/pip install numpy matplotlib scipy```

You can also optionally add an update section to moonraker for subsequent updates via Fluidd / Mainsail update managers.
```
[update_manager motors-sync]
type: git_repo
path: ~/motors-sync/
origin: https://github.com/MRX8024/motors-sync.git
primary_branch: main
managed_services: klipper
```
