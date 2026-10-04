# Monte Carlo Localization (MCL) - ROS 2

Pacote ROS 2 (C++) que implementa o algoritmo de **Localização de Monte Carlo** (*Particle Filter*) para robôs móveis diferenciais utilizando leituras de LiDAR 2D e odometria sobre um mapa de grade de ocupação (*Occupancy Grid*).

---

## 📌 O que é e Como Funciona

O algoritmo estima probabilisticamente a pose 2D do robô $(x, y, \theta)$ no referencial do mapa (`map`), mitigando o erro acumulado da odometria (*drift*).

### Pipeline de Execução:
1. **Inicialização:**
   * **Global (`global_localization_on_start: true`):** Partículas distribuídas uniformemente apenas pelas células livres do mapa.
   * **Local (`auto_initialize: true`):** Partículas inicializadas com distribuição Gaussiana em torno de uma pose inicial configurada no YAML.
   * **Interativa:** Recebe estimativas de pose a qualquer momento pelo tópico `/initialpose` (ferramenta *2D Pose Estimate* do RViz).
2. **Predição (*Motion Model*):**
   * A cada nova leitura de odometria (`/odom`), calcula $\delta\text{rot}_1, \delta\text{trans}, \delta\text{rot}_2$ e propaga as partículas adicionando ruído gaussiano calibrado pelos coeficientes $\alpha_1, \alpha_2, \alpha_3, \alpha_4$.
3. **Correção (*Sensor Model - Likelihood Field*):**
   * A cada varredura do LiDAR (`/scan`), calcula o ponto final no mundo de cada feixe válido e consulta a distância ao obstáculo mais próximo via mapa de distâncias euclidianas pré-computado.
   * Pondera os feixes através de um modelo gaussiano de acerto ($z_{\text{hit}}, \sigma_{\text{hit}}$) combinado a um componente aleatório uniforme ($z_{\text{rand}}$).
4. **Reamostragem (*Resampling*):**
   * Reamostragem de baixa variância (*Low-Variance Resampling*) disparada seletivamente baseada no número efetivo de partículas ($N_{\text{eff}} < 0.75 \cdot N$).
   * Partículas que caem fora dos limites do mapa ou dentro de obstáculos são automaticamente reamostradas para regiões livres.
5. **Estimativa e Árvore de TF:**
   * Agrupa a nuvem em torno da partícula de maior peso (pico de verossimilhança) para evitar erros de média aritmética em distribuições multimodais.
   * Calcula e publica continuamente a transformada dinâmica **`map -> odom`**.

---

## 📡 Tópicos e Árvore de Transformadas (TF)

### Tópicos de Entrada (Subscriptions):
| Tópico | Tipo de Mensagem | Descrição |
| :--- | :--- | :--- |
| `/map` | `nav_msgs/msg/OccupancyGrid` | Mapa de ocupação do ambiente (fornecido pelo `map_server`). |
| `/odom` | `nav_msgs/msg/Odometry` | Odometria do robô contendo posição e orientação relativa. |
| `/scan` | `sensor_msgs/msg/LaserScan` | Varredura de distâncias do sensor LiDAR 2D. |
| `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Pose inicial enviada manualmente via RViz. |

### Tópicos de Saída (Publications):
| Tópico | Tipo de Mensagem | Descrição |
| :--- | :--- | :--- |
| `/mcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Pose estimada do robô $(x, y, \theta)$ e matriz de covariância $6 \times 6$. |
| `/particlecloud` | `geometry_msgs/msg/PoseArray` | Amostra das partículas ativas para visualização no RViz. |

### Transformadas (TF):
* **Requeridas:** `odom -> base_link` (ou `odom -> base_footprint -> base_link`) e `base_link -> base_scan`.
* **Publicada pelo nó:** **`map -> odom`**.

---

## ⚙️ Arquivos de Parâmetros (`config/`)

* **[`params.yaml`](config/params.yaml):** Configuração principal do nó de Monte Carlo para o cenário da fábrica (`map_factory`):
  * `num_particles`: Quantidade de partículas ativas (ex: `8000`).
  * `update_min_d`, `update_min_a`: Limiares mínimos de translação ($0.1\text{m}$) e rotação ($0.1\text{rad}$) para disparar atualização com o laser.
  * `alpha1` a `alpha4`: Coeficientes de ruído do modelo de movimento por odometria.
  * `z_hit`, `z_rand`, `sigma_hit`: Pesos e desvio padrão do modelo de verossimilhança do laser.
  * `global_localization_on_start`: `true` para espalhar partículas pelo mapa ou `false` para usar a pose inicial do arquivo.
  * `initial_pose_x`, `initial_pose_y`, `initial_pose_yaw`: Pose inicial (em metros e **radianos**).
* **[`params_gazebo.yaml`](config/params_gazebo.yaml):** Parâmetros calibrados especificamente para o mundo de simulação do Gazebo (`map_gazebo_world`).
* **[`amcl_params.yaml`](config/amcl_params.yaml):** Configuração oficial do `nav2_amcl` para testes comparativos de referência (*benchmark*).

---

## 🚀 Arquivos de Launch (`launch/`)

* **`localization_launch.py`:**
  Inicia o pipeline completo para o mapa da fábrica:
  1. `nav2_map_server` carregando `maps/map_factory.yaml`;
  2. `nav2_lifecycle_manager` para transição automática do servidor de mapa;
  3. `montecarlo_localization_node` carregando `config/params.yaml`;
  4. Interface RViz2 com layout pronto.
  ```bash
  ros2 launch montecarlo_localization localization_launch.py
  ```

* **`localization_gazebo_launch.py`:**
  Inicia a localização adaptada ao ambiente do Gazebo (`map_gazebo_world`), com `use_sim_time: true`:
  ```bash
  ros2 launch montecarlo_localization localization_gazebo_launch.py
  ```

* **`amcl_launch.py`:**
  Launch auxiliar para executar o AMCL nativo do Nav2 no mesmo mapa para comparação de desempenho:
  ```bash
  ros2 launch montecarlo_localization amcl_launch.py
  ```

---

## 🖥️ Visualização no RViz (`rviz/`)

O arquivo **[`rviz/mcl_visualization.rviz`](rviz/mcl_visualization.rviz)** vem pré-configurado com:
* **Fixed Frame:** `map`
* **Map Display:** Tópico `/map` renderizado no plano do chão.
* **LaserScan Display:** Tópico `/scan` colorido por intensidade ou decaimento.
* **TF Display:** Árvore de transformadas visualizando `map`, `odom`, `base_link` e `base_scan`.
* **Robot Pose:** Tópico `/mcl_pose` representado por uma seta vermelha indicando a pose estimada atual.
* **Particle Cloud:** Tópico `/particlecloud` exibindo a dispersão das partículas em verde/amarelo sobre o mapa.
* **Ground Truth (se ativo):** Tópico `/pose_ground_truth` para comparação visual imediata entre a estimativa e o valor real do simulador.

---

## 🛠️ Compilação e Execução

No seu workspace ROS 2:
```bash
cd ~/ros2_ws
colcon build --packages-select montecarlo_localization --symlink-install
source install/setup.bash
ros2 launch montecarlo_localization localization_launch.py
```
