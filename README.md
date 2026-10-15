# TurtleBot3 ROS2 Go-To-Goal (Yıldız Rover Destek Ekip - Ödev 4)

Gazebo'daki TurtleBot3'ü bir **Python ROS2 node'u** ile belirli bir (x, y) noktasına götüren ödev projesi.
Node, `/cmd_vel` topic'ine hız yayınlar (publish), `/odom` topic'inden robotun konumunu okur (subscribe).
Hedefe varıldığı anlaşılınca hız **sıfırlanır** ve node kapanır.

> **Durum notu:** Kontrol mantığı, ROS'suz bir diferansiyel sürüş modeliyle denendi. Gazebo'da gerçek çalıştırma
> ve ekran görüntüleri proje sahibi tarafından alınacaktır. Aşağıdaki "Test Sonuçları" bölümü o zamana kadar boştur.

## Kullanılan Teknolojiler
- Ubuntu 22.04, ROS2 **Humble**
- Python 3 (`rclpy`), `colcon`
- Gazebo (Gazebo Classic) + TurtleBot3 paketleri

## Gereksinimler
Ubuntu 22.04 üzerinde ROS2 Humble kurulu olmalı. Ardından:

```bash
sudo apt update
sudo apt install ros-humble-turtlebot3* ros-humble-gazebo-ros-pkgs python3-colcon-common-extensions
```

`~/.bashrc` dosyasına (bir kez):

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
```

Yeni terminal aç veya `source ~/.bashrc` çalıştır.

## Kurulum ve Build

```bash
git clone GITHUB_LINK_BURAYA_EKLENECEK turtlebot3-ros2-homework
cd turtlebot3-ros2-homework/ros2_ws
colcon build --packages-select turtlebot_controller
source install/setup.bash
```

## Çalıştırma

**Terminal 1: Gazebo**

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

Bu dünyada duvar ve engeller bulunur. Ödev "engel olmayan ortam" istediği için Gazebo penceresinde her engelin
üzerine **sağ tık → Delete** yap. Sahnede yalnızca robot kalmalı. (Gazebo'yu kapatıp açınca engeller geri gelir,
silme işlemi her açılışta tekrarlanır.)

**Terminal 2: Node** (yeni açılan her terminalde önce `source ros2_ws/install/setup.bash` çalıştır)

```bash
ros2 run turtlebot_controller go_to_goal
```

Hedefi parametreyle değiştirmek için (koordinatlar `odom` çerçevesindedir, robotun doğduğu nokta (0, 0), baktığı yön +x):

```bash
ros2 run turtlebot_controller go_to_goal --ros-args -p goal_x:=1.5 -p goal_y:=1.0
```

Varsayılan hedef: **(1.5, 1.0) m**. Robot başlangıçta +x yönüne baktığı için hedef başlangıç doğrultusunda değildir
(hedefe açı ≈ 33.7°). Node, hedef başlangıç doğrultusuyla ±10° içinde ise bunu hata olarak bildirip çalışmayı reddeder.

> Her denemeden önce Gazebo'yu yeniden başlatmak (veya robotu sıfırlamak, `Ctrl+R`) gerekir, çünkü `/odom`
> başlangıcı robotun doğduğu yerdir.

## Kullanılan ROS2 Topic'leri

| Topic | Yön | Mesaj tipi | Amaç |
|---|---|---|---|
| `/cmd_vel` | node yayınlar | `geometry_msgs/msg/Twist` | Lineer ve açısal hız komutu |
| `/odom` | node dinler | `nav_msgs/msg/Odometry` | Robotun konumu ve yönü |

Mesaj tiplerini kendi sisteminde doğrulamak için: `ros2 topic info /cmd_vel` ve `ros2 topic info /odom`.

## Hedefe Ulaşma Mantığı
1. `/odom` geldikçe robotun x, y ve yaw (quaternion'dan hesaplanır) değeri güncellenir.
2. 10 Hz'lik zamanlayıcı ile hedefe uzaklık `d = hypot(dx, dy)` ve açı hatası `atan2(dy, dx) - yaw` hesaplanır.
3. Açı hatası 0.3 rad'dan büyükse robot yerinde döner; küçükse ilerler ve giderken yönünü düzeltir.
4. Hız sınırları: lineer 0.15 m/s, açısal 0.8 rad/s.

## Hedefe Ulaşınca Nasıl Durur
- `d < 0.05 m` olduğunda hedefe varılmış sayılır.
- Sıfır hızlı `Twist` mesajı 10 kez yayınlanır (komut kaybına karşı), sonra node kapanır.
- `Ctrl+C` ile kapatılsa bile çıkışta bir kez sıfır hız yayınlanır.

## Ekran Görüntüleri
Gazebo'da çekilecek ve `screenshots/` klasörüne eklenecek:

| Dosya | İçerik |
|---|---|
| `screenshots/start.png` | Engelsiz ortamda başlangıç konumu |
| `screenshots/moving.png` | Hareket halindeki robot |
| `screenshots/goal.png` | Hedefe ulaşmış ve durmuş robot |

## Test Sonuçları
_Gazebo'da çalıştırıldıktan sonra doldurulacak._

## Proje Yapısı
```
turtlebot3-ros2-homework/
├── README.md
├── .gitignore
├── screenshots/
└── ros2_ws/
    └── src/
        └── turtlebot_controller/
            ├── package.xml
            ├── setup.py
            ├── setup.cfg
            ├── resource/turtlebot_controller
            └── turtlebot_controller/
                ├── __init__.py
                └── go_to_goal.py
```
