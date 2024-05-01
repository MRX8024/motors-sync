#!/bin/bash
repo=motors-sync
repo_path=~/motors-sync/

# Сворачивание от root
if [ "$(id -u)" = "0" ]; then
    echo "Script must run from non-root !!!"
    exit
fi

module_name=motors_sync.py
module_path=~/klipper/klippy/extras/
cfg_incl_path=~/printer_data/config/printer.cfg

ln -sf "$repo_path/$module_name" $module_path # Перезапись

# Добавление строки [force_move] в printer.cfg
if [ -f "$cfg_incl_path" ]; then
    if ! grep -q "^\[force_move\]$" "$cfg_incl_path"; then
        sudo service klipper stop
        sed -i "1i\enable_force_move: True" "$cfg_incl_path"
        sed -i "1i\[force_move\]" "$cfg_incl_path"

        sudo service klipper start
    else
        echo "Including [force_move] aborted, [force_move] already exists in $cfg_incl_path"
    fi
fi

blk_path=~/printer_data/config/moonraker.conf
# Добавление блока обновления в moonraker.conf
if [ -f "$blk_path" ]; then
    if ! grep -q "^\[update_manager $repo\]$" "$blk_path"; then
        read -p " Do you want to install updater? (y/n): " answer
        if [ "$answer" != "${answer#[Yy]}" ]; then
          sudo service moonraker stop
          sed -i "\$a \ " "$blk_path"
          sed -i "\$a [update_manager $repo]" "$blk_path"
          sed -i "\$a type: git_repo" "$blk_path"
          sed -i "\$a path: $repo_path" "$blk_path"
          sed -i "\$a origin: https://github.com/MRX8024/$repo.git" "$blk_path"
          sed -i "\$a primary_branch: main" "$blk_path"
          sed -i "\$a managed_services: klipper" "$blk_path"
          # echo "Including [update_manager] to $blk_path successfully complete"
          sudo service moonraker start
        else
          echo "Installing updater aborted"
        fi
    else
        echo "Including [update_manager] aborted, [update_manager] already exists in $blk_path"
    fi
fi

sudo apt update
sudo apt install python3-numpy python3-matplotlib libatlas-base-dev libopenblas-dev
sudo ~/klippy-env/bin/pip install -r "$repo_path/"wiki/requirements.txt