#!/usr/bin/env bash
# Spawnea los tres cubos (blanco, negro, azul) en la mesa de Gazebo.
# Asume que el workspace ROS esta en ~/irb120_ws (ajustar abajo si no).
# Uso:
#   bash $(ros2 pkg prefix irb120pe_cognitive)/share/irb120pe_cognitive/scripts/spawn_cubos.sh
# o, mas comodo, dejando una copia en el workspace root:
#   bash ~/irb120_ws/spawn_cubos.sh
# Requisito: la demo (Gazebo) ya debe estar lanzada en otro panel.

source /opt/ros/humble/setup.bash
source ~/irb120_ws/install/setup.bash

# Borra primero TODOS los cubos (viejos y canonicos) y ESPERA a que Gazebo procese la
# eliminacion. Sin esa espera, el borrado asincrono del --replace puede completarse
# DESPUES del spawn y borrar el cubo recien creado ("desaparece nada mas aparecer").
for name in white_1 black_1 blue_1 WhiteCube BlackCube BlueCube; do
  ros2 run irb120pe_cognitive gazebo_cube_helper delete --name "$name" >/dev/null 2>&1 || true
done
sleep 1.5

# IMPORTANTE: el nombre de la entidad debe ser el canonico (BlueCube/BlackCube/WhiteCube)
# porque el adaptador de acciones pide a /ATTACHLINK exactamente ese nombre de modelo/link.
# Sin --replace: ya los hemos borrado arriba, asi evitamos la carrera delete/spawn.
ros2 run irb120pe_cognitive gazebo_cube_helper spawn --cube WhiteCube --name WhiteCube --x 0.55 --y 0.30 --z 0.90
ros2 run irb120pe_cognitive gazebo_cube_helper spawn --cube BlackCube --name BlackCube --x 0.55 --y 0.45 --z 0.90
ros2 run irb120pe_cognitive gazebo_cube_helper spawn --cube BlueCube  --name BlueCube  --x 0.55 --y 0.60 --z 0.90

echo "=== Cubos spawneados (mira la ventana de Gazebo) ==="
