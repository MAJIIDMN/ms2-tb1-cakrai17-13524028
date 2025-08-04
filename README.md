# ms2-tb1-cakrai17-13524028

|Nama|NIM|
|----|---|
|Muhammad Nur Majiid|13524028|
---
# How to run

Clone Repository github:
```
git clone https://github.com/MAJIIDMN/ms2-tb1-cakrai17-13524028.git
```
buka direktori pkg:
```
cd ms2-tb1-cakrai17-13524028
```
load environment dari workspace:
```
source install/setup.bash
```
build pkg:
```
colcon build
```
load environment ROS 2 dari instalasi utama (global) yang ada di sistem(sesuaikan type ros2):
```
source /opt/ros/humble/setup.bash
```
jalankan semua node:
```
ros2 launch pkg_13524028 milestone2.launch.py
```
---
# Screenshot

### Spek Wajib
membuat node yang menghubungkan antara node /twist_command_randomizer dengan node /movement_reader agar dapat mempublish topic /autonomous_vel.

![Spek Wajib](https://i.imgur.com/J0STQDx.png)

### Spek Bonus
menjadikan node baru tersebut bersifat multiplexer, dimana Node yang dibuat harus melakukan subscription ke ketiga Topic dari twist_command_randomizer lalu meneruskan pesan Twist ke cmd_vel berdasarkan prioritas. Contoh urutan prioritas yang dapat digunakan adalah keyboard_vel > joy_vel > autonomous_vel. Node ini juga harus melakukan publish ke cmd_type dengan nilai pesan sesuai dengan jenis sumber Twist yang digunakan, apakah itu ‘autonomous’, ‘joy’, atau ‘keyboard’.

![Spek bonus](https://i.imgur.com/4om2fk4.png)
### Twist pada topic /keyboard_vel
![terminal keyboard topic](https://i.imgur.com/DqXvRNz.png)
### Twist pada topic /joy_vel
![terminal joy topic](https://i.imgur.com/hkka40m.png)
### Twist pada topic /autonomous_vel
![terminal autonomous topic](https://i.imgur.com/9ZHsylJ.png)
### tampilan terminal gabungan (All in one)
![semua terminal](https://i.imgur.com/ZJiU52u.png)
